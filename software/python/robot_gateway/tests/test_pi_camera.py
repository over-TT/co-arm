from __future__ import annotations

import builtins
from dataclasses import FrozenInstanceError
from datetime import datetime, timezone
import hashlib
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from robot_gateway.camera_profiles import (
    AUTO_PROFILE_ID,
    CAPTURE_PROFILE_DETAIL,
    CAPTURE_PROFILE_SURVEY,
    MODULE3_WIDE_PROFILE_ID,
    OV5647_PROFILE_ID,
)
from robot_gateway.pi_camera import (
    AutofocusAttempt,
    AutofocusStatus,
    CameraAutofocusError,
    CameraCaptureError,
    CameraConfigurationError,
    CameraLifecycleError,
    DEFAULT_MAX_FRAME_BYTES,
    FocusQuality,
    PiCameraProvider,
)


JPEG = b"\xff\xd8camera-frame\xff\xd9"
CAPTURED_AT = datetime(2026, 8, 2, 12, 30, tzinfo=timezone.utc)


class FakePicamera2:
    def __init__(
        self,
        payload: bytes = JPEG,
        *,
        camera_controls: dict[str, object] | None = None,
        capture_metadata: dict[str, object] | None = None,
        camera_properties: dict[str, object] | None = None,
    ) -> None:
        self.payload = payload
        self.camera_controls = camera_controls or {}
        self.camera_properties = camera_properties or {}
        self.capture_metadata = capture_metadata
        self.created_main = None
        self.created_mains: list[dict[str, object]] = []
        self.created_transform = None
        self.configured_with = None
        self.configurations: list[dict[str, object]] = []
        self.started = False
        self.start_calls = 0
        self.stopped = False
        self.stop_calls = 0
        self.closed = False
        self.capture_formats: list[str] = []
        self.controls_set: list[dict[str, object]] = []

    def create_still_configuration(self, *, main, transform=None):
        self.created_main = main
        self.created_mains.append(main)
        self.created_transform = transform
        return {"kind": "still", "main": main}

    def configure(self, configuration) -> None:
        self.configured_with = configuration
        self.configurations.append(configuration)

    def start(self) -> None:
        self.started = True
        self.start_calls += 1

    def set_controls(self, controls: dict[str, object]) -> None:
        self.controls_set.append(controls)

    def capture_file(self, output, *, format: str):
        self.capture_formats.append(format)
        output.write(self.payload)
        return self.capture_metadata

    def stop(self) -> None:
        self.stopped = True
        self.stop_calls += 1

    def close(self) -> None:
        self.closed = True


def make_provider(camera: FakePicamera2, **kwargs) -> PiCameraProvider:
    return PiCameraProvider(
        camera_factory=lambda: camera,
        transform_factory=lambda: SimpleNamespace(hflip=1, vflip=1),
        warmup_seconds=0,
        sleeper=lambda _seconds: None,
        utc_clock=lambda: CAPTURED_AT,
        monotonic_clock=lambda: 100.0,
        **kwargs,
    )


def test_configuration_is_lazy_conservative_and_warmed() -> None:
    camera = FakePicamera2()
    factories, sleeps = [], []
    provider = PiCameraProvider(
        camera_factory=lambda: factories.append(1) or camera,
        transform_factory=lambda: SimpleNamespace(hflip=1, vflip=1),
        warmup_seconds=1.25,
        sleeper=sleeps.append,
    )
    assert factories == []
    provider.start()
    assert factories == [1]
    assert camera.created_mains == [
        {"size": (2304, 1296)},
        {"size": (4608, 2592)},
    ]
    assert camera.configured_with == {
        "kind": "still",
        "main": {"size": (2304, 1296)},
    }
    assert camera.started
    assert sleeps == [1.25]
    assert provider.autofocus_status == AutofocusStatus(
        capability="unsupported",
        mode="fixed",
        state="fixed",
    )


def test_module3_wide_profiles_verify_driver_identity_and_select_focus_ranges(
    monkeypatch,
) -> None:
    continuous, normal, full, macro = object(), object(), object(), object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(
                    Normal=normal,
                    Full=full,
                    Macro=macro,
                ),
            )
        ),
    )
    camera = FakePicamera2(
        camera_controls={"AfMode": object(), "AfRange": object()},
        camera_properties={
            "Model": "imx708_wide",
            "PixelArraySize": SimpleNamespace(width=4608, height=2592),
            "ScalerCropMaximum": (0, 0, 4608, 2592),
        },
    )
    provider = make_provider(camera)

    provider.start()

    assert provider.sensor_model == "imx708"
    assert provider.identity_confidence == "driver_reported"
    assert provider.camera_profile_metadata == {
        "id": MODULE3_WIDE_PROFILE_ID,
        "productName": "Raspberry Pi Camera Module 3 Wide",
        "sensorModel": "imx708",
        "lensVariant": "wide",
        "nativeDimensions": {"width": 4608, "height": 2592},
        "nominalFocalLengthMm": 2.75,
        "nominalFieldOfViewDegrees": {"horizontal": 102.0, "vertical": 67.0},
        "captureProfiles": {
            "survey": {"width": 2304, "height": 1296},
            "detail": {"width": 4608, "height": 2592},
        },
    }
    assert camera.configurations == [
        {"kind": "still", "main": {"size": (2304, 1296)}}
    ]
    assert provider.autofocus_status.range == "normal"

    survey = provider.capture(profile=CAPTURE_PROFILE_SURVEY)
    detail = provider.capture()

    assert (survey.width, survey.height) == (2304, 1296)
    assert (detail.width, detail.height) == (4608, 2592)
    assert camera.configurations == [
        {"kind": "still", "main": {"size": (2304, 1296)}},
        {"kind": "still", "main": {"size": (4608, 2592)}},
    ]
    assert camera.controls_set == [
        {"AfMode": continuous, "AfRange": normal},
        {"AfMode": continuous, "AfRange": macro},
    ]
    assert camera.stop_calls == 1
    assert camera.start_calls == 2


def test_module3_wide_focus_ranges_fall_back_to_full(monkeypatch) -> None:
    continuous, normal, full, macro = object(), object(), object(), object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(
                    Normal=normal,
                    Full=full,
                    Macro=macro,
                ),
            )
        ),
    )

    class FullFallbackCamera(FakePicamera2):
        def set_controls(self, controls: dict[str, object]) -> None:
            self.controls_set.append(controls)
            if controls.get("AfRange") in {normal, macro}:
                raise RuntimeError("primary range rejected")

    camera = FullFallbackCamera(
        camera_controls={"AfMode": object(), "AfRange": object()}
    )
    provider = make_provider(camera)

    provider.start()
    assert provider.autofocus_status.range == "full"
    provider.capture()

    assert provider.autofocus_status.range == "full"
    assert camera.controls_set == [
        {"AfMode": continuous, "AfRange": normal},
        {"AfMode": continuous, "AfRange": full},
        {"AfMode": continuous, "AfRange": macro},
        {"AfMode": continuous, "AfRange": full},
    ]


@pytest.mark.parametrize(
    ("properties", "message"),
    [
        (
            {"Model": "ov5647", "PixelArraySize": (2592, 1944)},
            "sensor does not match",
        ),
        (
            {"Model": "imx708", "PixelArraySize": (4608, 2591)},
            "resolution does not match",
        ),
        (
            {
                "Model": "imx708",
                "PixelArraySize": (4608, 2592),
                "ScalerCropMaximum": (0, 0, 2304, 1296),
            },
            "full configured sensor area",
        ),
    ],
)
def test_module3_wide_driver_mismatch_fails_closed(
    properties: dict[str, object],
    message: str,
) -> None:
    camera = FakePicamera2(camera_properties=properties)
    provider = make_provider(camera)

    with pytest.raises(CameraConfigurationError, match=message):
        provider.start()

    assert camera.closed is True


def test_auto_profile_preserves_explicit_ov5647_compatibility() -> None:
    camera = FakePicamera2(
        camera_properties={
            "Model": "ov5647",
            "PixelArraySize": (2592, 1944),
            "ScalerCropMaximum": (0, 0, 2592, 1944),
        }
    )
    provider = make_provider(camera, camera_profile=AUTO_PROFILE_ID)

    provider.start()
    frame = provider.capture()

    assert provider.camera_profile_metadata["id"] == OV5647_PROFILE_ID
    assert provider.identity_confidence == "driver_reported"
    assert (frame.width, frame.height) == (2592, 1944)
    assert camera.created_mains == [
        {"size": (1296, 972)},
        {"size": (2592, 1944)},
    ]
    assert provider.autofocus_status.capability == "unsupported"


def test_continuous_autofocus_is_enabled_when_libcamera_exposes_it(
    monkeypatch,
) -> None:
    continuous = object()
    macro = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(Macro=macro),
            )
        ),
    )
    camera = FakePicamera2(
        camera_controls={"AfMode": object(), "AfRange": object()},
        capture_metadata={
            "AfState": SimpleNamespace(name="Focused"),
            "LensPosition": 2.25,
            "FocusFoM": 7_654,
        },
    )
    provider = make_provider(camera)

    provider.start()
    survey_configured = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        range="unavailable",
    )
    assert provider.autofocus_status == survey_configured
    frame = provider.capture()

    assert camera.controls_set == [
        {"AfMode": continuous},
        {"AfMode": continuous, "AfRange": macro},
    ]
    expected = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="focused",
        range="macro",
        lens_position=2.25,
    )
    assert provider.autofocus_status == expected
    assert frame.autofocus == expected
    assert frame.focus_quality == FocusQuality(status="measured", value=7_654)


@pytest.mark.parametrize(
    ("driver_result", "expected_result"),
    [(True, "focused"), (False, "not_focused")],
)
def test_manual_autofocus_runs_one_bounded_cycle_and_restores_continuous_mode(
    monkeypatch,
    driver_result: bool,
    expected_result: str,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )

    class AutofocusCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(camera_controls={"AfMode": object()})
            self.job = object()
            self.autofocus_waits: list[bool] = []
            self.wait_calls: list[tuple[object, float]] = []

        def autofocus_cycle(self, *, wait: bool):
            self.autofocus_waits.append(wait)
            return self.job

        def wait(self, job: object, *, timeout: float):
            self.wait_calls.append((job, timeout))
            return driver_result

    camera = AutofocusCamera()
    provider = make_provider(camera)
    provider.start()

    attempt = provider.autofocus()

    expected_status = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
    )
    assert attempt == AutofocusAttempt(
        attempted=True,
        result=expected_result,
        autofocus=expected_status,
    )
    assert camera.autofocus_waits == [False]
    assert camera.wait_calls == [(camera.job, 10.0)]
    assert camera.controls_set == [
        {"AfMode": continuous},
        {"AfMode": continuous},
    ]
    assert provider.autofocus_status == expected_status


@pytest.mark.parametrize(
    ("capture_profile", "restored_range"),
    [
        (CAPTURE_PROFILE_SURVEY, "normal"),
        (CAPTURE_PROFILE_DETAIL, "macro"),
    ],
)
def test_manual_autofocus_widens_to_full_then_restores_the_capture_profile(
    monkeypatch,
    capture_profile: str,
    restored_range: str,
) -> None:
    continuous, normal, full, macro = object(), object(), object(), object()
    range_values = {"normal": normal, "macro": macro}
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(
                    Normal=normal,
                    Full=full,
                    Macro=macro,
                ),
            )
        ),
    )

    class AutofocusCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(
                camera_controls={"AfMode": object(), "AfRange": object()}
            )

        def autofocus_cycle(self, *, wait: bool):
            assert wait is False
            return object()

        def wait(self, _job: object, *, timeout: float):
            assert timeout == 10.0
            return True

    camera = AutofocusCamera()
    provider = make_provider(camera)
    provider.start()
    if capture_profile == CAPTURE_PROFILE_DETAIL:
        provider.capture(profile=CAPTURE_PROFILE_DETAIL)

    before_manual = len(camera.controls_set)
    attempt = provider.autofocus()

    assert attempt.result == "focused"
    assert attempt.autofocus.range == restored_range
    assert camera.controls_set[before_manual:] == [
        {"AfRange": full},
        {
            "AfMode": continuous,
            "AfRange": range_values[restored_range],
        },
    ]


def test_manual_autofocus_continues_when_full_range_is_rejected(monkeypatch) -> None:
    continuous, normal, full = object(), object(), object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(Normal=normal, Full=full),
            )
        ),
    )

    class NoFullAutofocusCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(
                camera_controls={"AfMode": object(), "AfRange": object()}
            )

        def set_controls(self, controls: dict[str, object]) -> None:
            self.controls_set.append(controls)
            if controls == {"AfRange": full}:
                raise RuntimeError("full range rejected")

        def autofocus_cycle(self, *, wait: bool):
            assert wait is False
            return object()

        def wait(self, _job: object, *, timeout: float):
            assert timeout == 10.0
            return False

    camera = NoFullAutofocusCamera()
    provider = make_provider(camera)
    provider.start()

    attempt = provider.autofocus()

    assert attempt.result == "not_focused"
    assert attempt.autofocus.range == "normal"
    assert camera.controls_set == [
        {"AfMode": continuous, "AfRange": normal},
        {"AfRange": full},
        {"AfMode": continuous, "AfRange": normal},
    ]


def test_manual_autofocus_reports_fixed_focus_without_calling_driver() -> None:
    class FixedFocusCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__()
            self.autofocus_calls = 0

        def autofocus_cycle(self, *, wait: bool):
            self.autofocus_calls += 1
            raise AssertionError("fixed-focus driver must not be called")

    camera = FixedFocusCamera()
    provider = make_provider(camera)
    provider.start()

    attempt = provider.autofocus()

    assert attempt == AutofocusAttempt(
        attempted=False,
        result="unsupported",
        autofocus=AutofocusStatus(
            capability="unsupported",
            mode="fixed",
            state="fixed",
        ),
    )
    assert camera.autofocus_calls == 0


def test_manual_autofocus_reports_unavailable_when_picamera_helper_is_missing(
    monkeypatch,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )
    camera = FakePicamera2(camera_controls={"AfMode": object()})
    provider = make_provider(camera)
    provider.start()

    attempt = provider.autofocus()

    assert attempt == AutofocusAttempt(
        attempted=False,
        result="unavailable",
        autofocus=AutofocusStatus(
            capability="supported",
            mode="continuous",
            state="configured",
        ),
    )
    assert camera.controls_set == [{"AfMode": continuous}]


def test_manual_autofocus_timeout_clears_job_and_restores_continuous_mode(
    monkeypatch,
) -> None:
    continuous = object()
    cancel = object()
    normal = object()
    full = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfTriggerEnum=SimpleNamespace(Cancel=cancel),
                AfRangeEnum=SimpleNamespace(Normal=normal, Full=full),
            )
        ),
    )

    class TimedOutAutofocusCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(
                camera_controls={"AfMode": object(), "AfRange": object()}
            )
            self.wait_calls = 0

        def autofocus_cycle(self, *, wait: bool):
            assert wait is False
            return object()

        def wait(self, _job: object, *, timeout: float):
            self.wait_calls += 1
            if self.wait_calls == 1:
                assert timeout == 10.0
                raise TimeoutError
            assert timeout == 2.0
            return False

    camera = TimedOutAutofocusCamera()
    provider = make_provider(camera)
    provider.start()

    attempt = provider.autofocus()

    assert attempt.result == "timed_out"
    assert attempt.attempted is True
    assert attempt.autofocus.state == "configured"
    assert camera.wait_calls == 2
    assert camera.controls_set == [
        {"AfMode": continuous, "AfRange": normal},
        {"AfRange": full},
        {"AfTrigger": cancel},
        {"AfMode": continuous, "AfRange": normal},
    ]


def test_manual_autofocus_fails_closed_when_timed_out_job_cannot_be_cleared(
    monkeypatch,
) -> None:
    continuous = object()
    cancel = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfTriggerEnum=SimpleNamespace(Cancel=cancel),
            )
        ),
    )

    class UnsafeTimeoutCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(camera_controls={"AfMode": object()})

        def autofocus_cycle(self, *, wait: bool):
            return object()

        def wait(self, _job: object, *, timeout: float):
            raise TimeoutError

    camera = UnsafeTimeoutCamera()
    provider = make_provider(camera)
    provider.start()

    with pytest.raises(CameraAutofocusError, match="could not be cleared"):
        provider.autofocus()
    assert provider.closed is True
    assert camera.stopped is True
    assert camera.closed is True
    assert camera.controls_set == [
        {"AfMode": continuous},
        {"AfTrigger": cancel},
    ]


@pytest.mark.parametrize(
    ("reported_state", "expected_state"),
    [
        ("Idle", "idle"),
        ("Scanning", "scanning"),
        ("Focused", "focused"),
        ("Failed", "failed"),
    ],
)
def test_continuous_autofocus_uses_only_same_frame_state_metadata(
    monkeypatch,
    reported_state: str,
    expected_state: str,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )
    camera = FakePicamera2(
        camera_controls={"AfMode": object()},
        capture_metadata={"AfState": SimpleNamespace(name=reported_state)},
    )
    provider = make_provider(camera)

    provider.start()
    assert provider.autofocus_status.state == "configured"

    frame = provider.capture()

    assert camera.controls_set == [
        {"AfMode": continuous},
        {"AfMode": continuous},
    ]
    assert frame.autofocus.state == expected_state
    assert provider.autofocus_status.state == expected_state


def test_continuous_autofocus_never_claims_focus_without_frame_metadata(
    monkeypatch,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )
    provider = make_provider(
        FakePicamera2(camera_controls={"AfMode": object()})
    )

    provider.start()
    frame = provider.capture()

    assert frame.autofocus == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
    )


def test_continuous_autofocus_is_configured_before_start_and_warmup(
    monkeypatch,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )
    events: list[str] = []

    class OrderedAutofocusCamera(FakePicamera2):
        def start(self) -> None:
            super().start()
            events.append("camera_started")

        def set_controls(self, controls: dict[str, object]) -> None:
            super().set_controls(controls)
            events.append("continuous_autofocus_enabled")

    camera = OrderedAutofocusCamera(camera_controls={"AfMode": object()})
    provider = PiCameraProvider(
        camera_factory=lambda: camera,
        transform_factory=lambda: SimpleNamespace(hflip=1, vflip=1),
        warmup_seconds=2,
        sleeper=lambda _seconds: events.append("warmup_complete"),
        utc_clock=lambda: CAPTURED_AT,
        monotonic_clock=lambda: 100.0,
    )

    provider.start()

    assert camera.controls_set == [{"AfMode": continuous}]
    assert events == [
        "continuous_autofocus_enabled",
        "camera_started",
        "warmup_complete",
    ]


def test_missing_af_state_never_reuses_a_previous_focused_claim(
    monkeypatch,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )

    class SequencedMetadataCamera(FakePicamera2):
        def __init__(self) -> None:
            super().__init__(camera_controls={"AfMode": object()})
            self.metadata = iter(
                (
                    {
                        "AfState": SimpleNamespace(name="Focused"),
                        "LensPosition": 3.5,
                    },
                    {},
                    {"AfState": "driver-private-state", "LensPosition": 2.0},
                )
            )

        def capture_file(self, output, *, format: str):
            self.capture_formats.append(format)
            output.write(self.payload)
            return next(self.metadata)

    provider = make_provider(SequencedMetadataCamera())
    provider.start()

    first = provider.capture()
    missing = provider.capture()
    unknown = provider.capture()

    assert first.autofocus == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="focused",
        lens_position=3.5,
    )
    assert missing.autofocus == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
    )
    assert unknown.autofocus == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        lens_position=2.0,
    )


@pytest.mark.parametrize(
    "focus_fom",
    [None, True, -1, 2_147_483_648, 12.5, "900"],
)
def test_focus_quality_is_same_frame_relative_only_and_fail_closed(
    focus_fom: object,
) -> None:
    metadata = {} if focus_fom is None else {"FocusFoM": focus_fom}
    provider = make_provider(FakePicamera2(capture_metadata=metadata))

    provider.start()
    frame = provider.capture()

    assert frame.focus_quality.as_dict() == {
        "metric": "libcamera_focus_fom",
        "status": "unavailable",
        "value": None,
        "higherIsSharper": True,
        "comparison": "same_subject_similar_framing_only",
    }


@pytest.mark.parametrize("focus_fom", [0, 2_147_483_647])
def test_focus_quality_accepts_the_full_bounded_libcamera_range(
    focus_fom: int,
) -> None:
    provider = make_provider(
        FakePicamera2(capture_metadata={"FocusFoM": focus_fom})
    )

    provider.start()
    frame = provider.capture()

    assert frame.focus_quality.as_dict() == {
        "metric": "libcamera_focus_fom",
        "status": "measured",
        "value": focus_fom,
        "higherIsSharper": True,
        "comparison": "same_subject_similar_framing_only",
    }


def test_autofocus_configuration_failure_is_reported_without_losing_capture(
    monkeypatch,
) -> None:
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=object())
            )
        ),
    )

    class FailingAutofocusCamera(FakePicamera2):
        def set_controls(self, controls: dict[str, object]) -> None:
            raise RuntimeError("private driver detail")

    camera = FailingAutofocusCamera(
        camera_controls={"AfMode": object()},
        capture_metadata={
            "AfState": SimpleNamespace(name="Focused"),
            "LensPosition": 1.5,
        },
    )
    provider = make_provider(camera)

    provider.start()
    frame = provider.capture()

    expected = AutofocusStatus(
        capability="supported",
        mode="unknown",
        state="configuration_failed",
    )
    assert provider.autofocus_status == expected
    assert frame.autofocus == expected
    assert frame.data == JPEG


def test_rejected_macro_range_falls_back_to_continuous_autofocus(
    monkeypatch,
) -> None:
    continuous = object()
    macro = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous),
                AfRangeEnum=SimpleNamespace(Macro=macro),
            )
        ),
    )

    class NoMacroCamera(FakePicamera2):
        def set_controls(self, controls: dict[str, object]) -> None:
            self.controls_set.append(controls)
            if "AfRange" in controls:
                raise RuntimeError("macro range rejected")

    camera = NoMacroCamera(
        camera_controls={"AfMode": object(), "AfRange": object()}
    )
    provider = make_provider(camera)

    provider.start()
    provider.capture()

    assert camera.controls_set == [
        {"AfMode": continuous},
        {"AfMode": continuous, "AfRange": macro},
        {"AfMode": continuous},
    ]
    assert provider.autofocus_status == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        range="unavailable",
    )


def test_missing_macro_enum_keeps_continuous_autofocus_available(
    monkeypatch,
) -> None:
    continuous = object()
    monkeypatch.setitem(
        sys.modules,
        "libcamera",
        SimpleNamespace(
            controls=SimpleNamespace(
                AfModeEnum=SimpleNamespace(Continuous=continuous)
            )
        ),
    )
    camera = FakePicamera2(
        camera_controls={"AfMode": object(), "AfRange": object()}
    )
    provider = make_provider(camera)

    provider.start()

    assert camera.controls_set == [{"AfMode": continuous}]
    assert provider.autofocus_status == AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        range="unavailable",
    )


def test_upside_down_mount_is_rotated_before_any_frame_is_delivered(
    monkeypatch,
) -> None:
    class FakeTransform:
        def __init__(self, *, hflip: int = 0, vflip: int = 0) -> None:
            self.hflip = hflip
            self.vflip = vflip

    monkeypatch.setitem(sys.modules, "libcamera", SimpleNamespace(Transform=FakeTransform))
    camera = FakePicamera2()

    PiCameraProvider(
        camera_factory=lambda: camera,
        warmup_seconds=0,
    ).start()

    assert isinstance(camera.created_transform, FakeTransform)
    assert (camera.created_transform.hflip, camera.created_transform.vflip) == (1, 1)


def test_capture_metadata_hash_age_and_immutability() -> None:
    camera = FakePicamera2()
    ticks = iter((50.0, 52.75))
    provider = PiCameraProvider(
        width=640,
        height=480,
        camera_id="bench-camera",
        sensor_model="ov5647",
        camera_factory=lambda: camera,
        transform_factory=lambda: SimpleNamespace(hflip=1, vflip=1),
        warmup_seconds=0,
        sleeper=lambda _seconds: None,
        utc_clock=lambda: CAPTURED_AT,
        monotonic_clock=lambda: next(ticks),
    )
    provider.start()
    frame = provider.capture()
    assert provider.camera_profile_metadata["nativeDimensions"] == {
        "width": 2592,
        "height": 1944,
    }
    assert provider.camera_profile_metadata["captureProfiles"] == {
        "survey": {"width": 640, "height": 480},
        "detail": {"width": 640, "height": 480},
    }
    assert camera.capture_formats == ["jpeg"]
    assert frame.data == frame.jpeg_bytes == JPEG
    assert frame.mime_type == "image/jpeg"
    assert (frame.width, frame.height) == (640, 480)
    assert frame.captured_at_utc == CAPTURED_AT
    assert frame.captured_monotonic == 50.0
    assert frame.age_seconds == pytest.approx(2.75)
    assert frame.sha256 == hashlib.sha256(JPEG).hexdigest()
    assert (frame.camera_id, frame.sensor_model) == ("bench-camera", "ov5647")
    assert frame.autofocus == AutofocusStatus(
        capability="unsupported",
        mode="fixed",
        state="fixed",
    )
    with pytest.raises(FrozenInstanceError):
        frame.width = 1


def test_capture_before_start_and_after_close_fails_clearly() -> None:
    camera = FakePicamera2()
    provider = make_provider(camera)
    with pytest.raises(CameraLifecycleError, match="not been started"):
        provider.capture()
    provider.start()
    provider.close()
    assert camera.stopped and camera.closed
    assert provider.autofocus_status.state == "closed"
    with pytest.raises(CameraLifecycleError, match="closed"):
        provider.capture()
    with pytest.raises(CameraLifecycleError, match="closed"):
        provider.start()


def test_missing_picamera2_is_lazy_and_sanitized(monkeypatch) -> None:
    provider = PiCameraProvider(warmup_seconds=0)
    real_import = builtins.__import__

    def missing(name, *args, **kwargs):
        if name == "picamera2":
            raise ModuleNotFoundError("secret/internal/path")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", missing)
    with pytest.raises(CameraConfigurationError) as error:
        provider.start()
    assert "unavailable" in str(error.value)
    assert "secret/internal/path" not in str(error.value)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("width", 319),
        ("width", 4609),
        ("height", 239),
        ("height", 2593),
        ("max_frame_bytes", 0),
        ("max_frame_bytes", DEFAULT_MAX_FRAME_BYTES + 1),
        ("warmup_seconds", -0.01),
        ("warmup_seconds", 61),
    ],
)
def test_bounds(name: str, value: int | float) -> None:
    with pytest.raises(ValueError, match=name):
        PiCameraProvider(**{name: value})


@pytest.mark.parametrize(
    ("payload", "limit", "message"),
    [(b"", 64, "empty"), (b"12345", 4, "exceeds")],
)
def test_empty_and_oversized_frames_fail(
    payload: bytes, limit: int, message: str
) -> None:
    provider = make_provider(FakePicamera2(payload), max_frame_bytes=limit)
    provider.start()
    with pytest.raises(CameraCaptureError, match=message):
        provider.capture()


def test_capture_is_serialized() -> None:
    entered = maximum_entered = 0
    counter_lock = threading.Lock()
    first_entered = threading.Event()
    release = threading.Event()

    class BlockingCamera(FakePicamera2):
        def capture_file(self, output, *, format: str):
            nonlocal entered, maximum_entered
            with counter_lock:
                entered += 1
                maximum_entered = max(maximum_entered, entered)
                first_entered.set()
            release.wait(timeout=2)
            output.write(JPEG)
            with counter_lock:
                entered -= 1
            return self.capture_metadata

    provider = make_provider(BlockingCamera())
    provider.start()
    errors: list[BaseException] = []

    def capture() -> None:
        try:
            provider.capture()
        except BaseException as error:
            errors.append(error)

    first = threading.Thread(target=capture)
    second = threading.Thread(target=capture)
    first.start()
    assert first_entered.wait(timeout=1)
    second.start()
    time.sleep(0.05)
    assert maximum_entered == 1
    assert second.is_alive()
    release.set()
    first.join(timeout=1)
    second.join(timeout=1)
    assert errors == []
    assert not first.is_alive() and not second.is_alive()
    assert maximum_entered == 1
