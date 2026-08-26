from __future__ import annotations

from collections.abc import Callable
from datetime import datetime, timezone
import hashlib
from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from robot_gateway.camera_api import (
    CameraEvidenceCaptureError,
    CameraEvidenceService,
    CameraEvidenceTransferUnavailableError,
)
from robot_gateway.camera_profiles import CAPTURE_PROFILE_DETAIL, MODULE3_WIDE_PROFILE
from robot_gateway.pi_camera import (
    AutofocusAttempt,
    AutofocusStatus,
    CameraAutofocusFatalError,
    CapturedFrame,
    FocusQuality,
    JPEG_MIME_TYPE,
)
from robot_gateway.runtime import create_app


TOKEN = "camera-token-" + ("c" * 40)
AUTH = {"Authorization": f"Bearer {TOKEN}"}
FIXED_AUTOFOCUS = AutofocusStatus(
    capability="unsupported",
    mode="fixed",
    state="fixed",
)


def make_frame(
    payload: bytes,
    *,
    monotonic_clock: Callable[[], float] = lambda: 12.5,
    captured_monotonic: float = 10.0,
    autofocus: AutofocusStatus = FIXED_AUTOFOCUS,
    focus_quality: FocusQuality = FocusQuality(status="unavailable"),
    camera_id: str = "rpi-camera-0",
    sensor_model: str = "ov5647",
    width: int = 640,
    height: int = 480,
) -> CapturedFrame:
    return CapturedFrame(
        data=payload,
        mime_type=JPEG_MIME_TYPE,
        width=width,
        height=height,
        captured_at_utc=datetime(2026, 8, 2, 20, 30, tzinfo=timezone.utc),
        captured_monotonic=captured_monotonic,
        sha256=hashlib.sha256(payload).hexdigest(),
        camera_id=camera_id,
        sensor_model=sensor_model,
        _monotonic_clock=monotonic_clock,
        autofocus=autofocus,
        focus_quality=focus_quality,
    )


class FakeProvider:
    camera_id = "rpi-camera-0"
    sensor_model = "ov5647"
    autofocus_status = FIXED_AUTOFOCUS

    def __init__(
        self,
        frames: list[CapturedFrame],
        *,
        start_error: Exception | None = None,
        capture_error: Exception | None = None,
        autofocus_status: AutofocusStatus | None = None,
        autofocus_attempt: AutofocusAttempt | None = None,
        autofocus_error: Exception | None = None,
    ) -> None:
        self.max_frame_bytes = max(
            (len(frame.data) for frame in frames),
            default=1,
        )
        self.frames = frames
        self.start_error = start_error
        self.capture_error = capture_error
        if autofocus_status is not None:
            self.autofocus_status = autofocus_status
        self.autofocus_attempt = autofocus_attempt
        self.autofocus_error = autofocus_error
        self.start_calls = 0
        self.capture_calls = 0
        self.autofocus_calls = 0
        self.close_calls = 0

    def start(self) -> None:
        self.start_calls += 1
        if self.start_error is not None:
            raise self.start_error

    def capture(self) -> CapturedFrame:
        self.capture_calls += 1
        if self.capture_error is not None:
            raise self.capture_error
        return self.frames.pop(0)

    def autofocus(self) -> AutofocusAttempt:
        self.autofocus_calls += 1
        if self.autofocus_error is not None:
            raise self.autofocus_error
        if self.autofocus_attempt is not None:
            return self.autofocus_attempt
        if self.autofocus_status.capability == "unsupported":
            return AutofocusAttempt(
                attempted=False,
                result="unsupported",
                autofocus=self.autofocus_status,
            )
        return AutofocusAttempt(
            attempted=False,
            result="unavailable",
            autofocus=self.autofocus_status,
        )

    def close(self) -> None:
        self.close_calls += 1


class ProfileAwareProvider(FakeProvider):
    sensor_model = "imx708"
    identity_confidence = "configured_candidate"

    def __init__(self, frames: list[CapturedFrame]) -> None:
        super().__init__(frames)
        self.capture_profiles: list[str] = []

    @property
    def camera_profile_metadata(self) -> dict[str, object]:
        return MODULE3_WIDE_PROFILE.as_dict()

    def start(self) -> None:
        super().start()
        self.identity_confidence = "driver_reported"

    def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame:
        self.capture_profiles.append(profile)
        return super().capture()


def token_file(tmp_path: Path) -> Path:
    path = tmp_path / "gateway.token"
    path.write_text(TOKEN, encoding="utf-8")
    return path


def test_authenticated_camera_lifecycle_capture_and_frame_evidence(
    tmp_path: Path,
) -> None:
    payload = b"\xff\xd8camera-jpeg\xff\xd9"
    provider = FakeProvider([make_frame(payload)])
    app = create_app(
        token_file=token_file(tmp_path),
        camera_provider=provider,
    )

    with TestClient(app) as client:
        assert provider.start_calls == 1
        unauthorized = client.get("/api/camera/status")
        assert unauthorized.status_code == 401

        camera_status = client.get("/api/camera/status", headers=AUTH)
        assert camera_status.status_code == 200
        assert camera_status.json() == {
            "simulated": False,
            "readOnly": True,
            "cameraId": "rpi-camera-0",
            "sensorModel": "ov5647",
            "identityConfidence": "configured_candidate",
            "autofocus": {
                "capability": "unsupported",
                "mode": "fixed",
                "state": "fixed",
                "range": "unavailable",
            },
            "state": "started",
            "available": True,
            "latestFrameId": None,
            "historySize": 0,
            "historyLimit": 8,
            "retainedBytes": 0,
            "byteLimit": 64 * 1024 * 1024,
        }
        assert (
            client.get("/api/camera/observations/latest", headers=AUTH).status_code
            == 404
        )

        assert client.post("/api/camera/autofocus").status_code == 401
        autofocus = client.post("/api/camera/autofocus", headers=AUTH)
        assert autofocus.status_code == 200
        assert autofocus.json() == {
            "simulated": False,
            "physicalArmMotion": False,
            "attempted": False,
            "result": "unsupported",
            "autofocus": {
                "capability": "unsupported",
                "mode": "fixed",
                "state": "fixed",
                "range": "unavailable",
            },
        }
        assert provider.autofocus_calls == 1

        capture = client.post("/api/camera/captures", headers=AUTH)
        assert capture.status_code == 200
        observation = capture.json()
        digest = hashlib.sha256(payload).hexdigest()
        assert observation["simulated"] is False
        assert observation["readOnly"] is True
        assert observation["cameraId"] == "rpi-camera-0"
        assert observation["sensorModel"] == "ov5647"
        assert observation["identityConfidence"] == "configured_candidate"
        assert observation["autofocus"] == {
            "capability": "unsupported",
            "mode": "fixed",
            "state": "fixed",
            "range": "unavailable",
        }
        assert observation["focusQuality"] == {
            "metric": "libcamera_focus_fom",
            "status": "unavailable",
            "value": None,
            "higherIsSharper": True,
            "comparison": "same_subject_similar_framing_only",
        }
        assert isinstance(observation["stateRevision"], int)
        assert observation["stateRevision"] >= 0
        assert observation["ageSeconds"] == 2.5
        assert observation["dimensions"] == {"width": 640, "height": 480}
        assert observation["byteCount"] == len(payload)
        assert observation["sha256"] == digest
        assert observation["contentSha256"] == f"sha256:{digest}"

        latest = client.get("/api/camera/observations/latest", headers=AUTH)
        assert latest.status_code == 200
        assert latest.json()["frameId"] == observation["frameId"]

        frame = client.get(observation["frameUrl"], headers=AUTH)
        assert frame.status_code == 200
        assert frame.content == payload
        assert frame.headers["content-type"] == "image/jpeg"
        assert frame.headers["cache-control"] == "no-store"
        assert frame.headers["etag"] == f'"sha256:{digest}"'
        assert frame.headers["x-content-sha256"] == f"sha256:{digest}"
        assert frame.headers["x-simulated"] == "false"
        assert frame.headers["x-read-only"] == "true"
        assert client.get("/api/camera/frames/missing", headers=AUTH).status_code == 404

    assert provider.close_calls == 1


def test_profile_aware_camera_exposes_identity_and_honors_strict_capture_profiles(
    tmp_path: Path,
) -> None:
    provider = ProfileAwareProvider(
        [
            make_frame(
                b"detail",
                sensor_model="imx708",
                width=4608,
                height=2592,
            ),
            make_frame(
                b"survey",
                sensor_model="imx708",
                width=2304,
                height=1296,
            ),
        ]
    )
    app = create_app(
        token_file=token_file(tmp_path),
        camera_provider=provider,
    )

    with TestClient(app) as client:
        camera_status = client.get("/api/camera/status", headers=AUTH).json()
        assert camera_status["identityConfidence"] == "driver_reported"
        assert camera_status["cameraProfile"] == MODULE3_WIDE_PROFILE.as_dict()

        default_capture = client.post("/api/camera/captures", headers=AUTH)
        survey_capture = client.post(
            "/api/camera/captures",
            headers=AUTH,
            json={"captureProfile": "survey"},
        )

        assert default_capture.status_code == 200
        assert default_capture.json()["captureProfile"] == "detail"
        assert default_capture.json()["cameraProfile"]["id"] == "module3-wide"
        assert survey_capture.status_code == 200
        assert survey_capture.json()["captureProfile"] == "survey"
        assert provider.capture_profiles == ["detail", "survey"]

        invalid_value = client.post(
            "/api/camera/captures",
            headers=AUTH,
            json={"captureProfile": "macro"},
        )
        invalid_fields = client.post(
            "/api/camera/captures",
            headers=AUTH,
            json={"captureProfile": "survey", "extra": True},
        )
        assert invalid_value.status_code == 422
        assert invalid_fields.status_code == 422


def test_profile_aware_provider_cannot_return_detail_dimensions_as_survey() -> None:
    provider = ProfileAwareProvider(
        [
            make_frame(
                b"mislabeled-detail",
                sensor_model="imx708",
                width=4608,
                height=2592,
            )
        ]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
    )
    service.start()

    with pytest.raises(CameraEvidenceCaptureError):
        service.capture(profile="survey")

    assert provider.capture_profiles == ["survey"]
    assert service.status()["historySize"] == 0


def test_legacy_provider_supports_detail_default_but_does_not_mislabel_survey() -> None:
    provider = FakeProvider([make_frame(b"detail")])
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
    )
    service.start()

    assert service.capture()["captureProfile"] == "detail"
    with pytest.raises(CameraEvidenceCaptureError):
        service.capture(profile="survey")
    assert provider.capture_calls == 1


def test_service_evicts_oldest_frames_by_count_and_total_bytes() -> None:
    ids = iter(["frame_a", "frame_b", "frame_c", "frame_d", "frame_e"])
    provider = FakeProvider(
        [
            make_frame(b"1111"),
            make_frame(b"2222"),
            make_frame(b"3333"),
            make_frame(b"44444"),
            make_frame(b"55555"),
        ]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 7},
        max_frames=2,
        max_total_bytes=8,
        id_factory=lambda: next(ids),
    )
    service.start()

    first = service.capture()
    service.capture()
    third = service.capture()
    assert service.frame(first["frameId"]) is None
    assert service.frame(third["frameId"]) is not None
    assert service.status()["historySize"] == 2
    assert service.status()["retainedBytes"] == 8

    fourth = service.capture()
    assert service.status()["historySize"] == 1
    assert service.status()["retainedBytes"] == 5
    fifth = service.capture()
    assert service.frame(fourth["frameId"]) is None
    assert service.frame(fifth["frameId"]) is not None
    assert service.status()["retainedBytes"] == 5


def test_one_use_transfer_survives_history_churn_and_ordinary_frame_reads(
    tmp_path: Path,
) -> None:
    ids = iter(["combined", "ordinary_a", "ordinary_b", "ordinary_c", "ordinary_d"])
    provider = FakeProvider(
        [
            make_frame(b"combined"),
            make_frame(b"ordinary-a"),
            make_frame(b"ordinary-b"),
            make_frame(b"ordinary-c"),
            make_frame(b"ordinary-d"),
        ]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 9},
        max_frames=2,
        max_total_bytes=64,
        id_factory=lambda: next(ids),
        transfer_token_factory=lambda: "transfer_exact",
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_service=service,
        )
    ) as client:
        combined = service.capture_transfer()
        assert client.get(
            "/api/camera/observations/latest", headers=AUTH
        ).status_code == 404
        for _ in range(4):
            assert client.post("/api/camera/captures", headers=AUTH).status_code == 200
            latest = client.get("/api/camera/observations/latest", headers=AUTH)
            assert latest.status_code == 200
            assert client.get(latest.json()["frameUrl"], headers=AUTH).status_code == 200

        assert client.get(
            f"/api/camera/frames/{combined['frameId']}", headers=AUTH
        ).status_code == 404
        exact = client.get(combined["transferUrl"], headers=AUTH)
        duplicate = client.get(combined["transferUrl"], headers=AUTH)

    assert exact.status_code == 200
    assert exact.content == b"combined"
    assert exact.headers["x-one-use-transfer"] == "true"
    assert duplicate.status_code == 404


def test_transfer_capacity_refuses_without_retiring_live_entry_and_ttl_releases_it() -> None:
    now = [100.0]
    ids = iter(["first", "after_expiry"])
    provider = FakeProvider(
        [make_frame(b"first"), make_frame(b"after-expiry")]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 10},
        max_transfers=1,
        transfer_ttl_seconds=2.0,
        id_factory=lambda: next(ids),
        transfer_token_factory=lambda: f"transfer_{provider.capture_calls}",
        monotonic_clock=lambda: now[0],
    )
    service.start()

    first = service.capture_transfer()
    with pytest.raises(CameraEvidenceTransferUnavailableError):
        service.capture_transfer()
    assert provider.capture_calls == 1
    assert service.has_transfer(first["transferToken"]) is True

    now[0] = 102.1
    after_expiry = service.capture_transfer()
    assert provider.capture_calls == 2
    assert service.has_transfer(first["transferToken"]) is False
    assert service.has_transfer(after_expiry["transferToken"]) is True


@pytest.mark.parametrize("bound_case", ["missing", "invalid", "oversize"])
def test_composed_transfer_requires_a_valid_provider_frame_bound(
    bound_case: str,
) -> None:
    provider = FakeProvider([make_frame(b"jpeg")])
    if bound_case == "missing":
        del provider.max_frame_bytes
    elif bound_case == "invalid":
        provider.max_frame_bytes = True
    else:
        provider.max_frame_bytes = 9
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 10},
        max_transfer_bytes=8,
    )
    service.start()

    assert service.transfer_available() is False
    with pytest.raises(CameraEvidenceTransferUnavailableError):
        service.capture_transfer()
    assert provider.capture_calls == 0

    # The bound protects the composed evidence handoff only. A consumer that
    # does not couple capture to arm motion may still use ordinary history.
    assert service.capture()["byteCount"] == 4
    assert provider.capture_calls == 1


def test_capture_rejects_a_frame_from_the_wrong_configured_camera() -> None:
    wrong = make_frame(b"wrong-origin")
    object.__setattr__(wrong, "camera_id", "another-camera")
    provider = FakeProvider([wrong])
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 10},
    )
    service.start()

    with pytest.raises(CameraEvidenceCaptureError):
        service.capture()


def test_status_and_observation_distinguish_configured_from_observed_focus() -> None:
    configured = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        range="macro",
    )
    observed = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="focused",
        range="macro",
        lens_position=1.75,
    )
    provider = FakeProvider(
        [make_frame(b"focused", autofocus=observed)],
        autofocus_status=configured,
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 8},
        id_factory=lambda: "frame_focus",
    )
    service.start()

    assert service.status()["autofocus"] == {
        "capability": "supported",
        "mode": "continuous",
        "state": "configured",
        "range": "macro",
    }
    assert service.capture()["autofocus"] == {
        "capability": "supported",
        "mode": "continuous",
        "state": "focused",
        "range": "macro",
        "lensPosition": 1.75,
    }


def test_capture_preserves_same_frame_relative_focus_quality() -> None:
    provider = FakeProvider(
        [
            make_frame(
                b"focused",
                focus_quality=FocusQuality(status="measured", value=8_420),
            )
        ]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 8},
        id_factory=lambda: "frame_focus_quality",
    )
    service.start()

    assert service.capture()["focusQuality"] == {
        "metric": "libcamera_focus_fom",
        "status": "measured",
        "value": 8_420,
        "higherIsSharper": True,
        "comparison": "same_subject_similar_framing_only",
    }


def test_camera_age_is_current_and_never_negative() -> None:
    now = [4.0]
    provider = FakeProvider(
        [make_frame(b"jpeg", monotonic_clock=lambda: now[0], captured_monotonic=5.0)]
    )
    service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 0},
        id_factory=lambda: "frame_age",
    )
    service.start()

    observation = service.capture()
    assert observation["ageSeconds"] == 0.0
    now[0] = 8.0
    assert service.latest()["ageSeconds"] == 3.0  # type: ignore[index]


def test_start_and_capture_failures_are_sanitized_http_errors(tmp_path: Path) -> None:
    private_path = r"C:\private\camera-secret.conf"
    unavailable = FakeProvider(
        [], start_error=FileNotFoundError(private_path)
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_provider=unavailable,
        )
    ) as client:
        status_response = client.get("/api/camera/status", headers=AUTH)
        assert status_response.json()["state"] == "unavailable"
        response = client.post("/api/camera/captures", headers=AUTH)
        assert response.status_code == 503
        assert response.json() == {"detail": "Camera is unavailable."}
        assert private_path not in response.text

    failed_closed_autofocus = FakeProvider(
        [], autofocus_error=CameraAutofocusFatalError(private_path)
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_provider=failed_closed_autofocus,
        )
    ) as client:
        response = client.post("/api/camera/autofocus", headers=AUTH)
        assert response.status_code == 502
        assert response.json() == {"detail": "Camera autofocus test failed."}
        assert private_path not in response.text
        assert client.get("/api/camera/status", headers=AUTH).json()["state"] == "unavailable"
        retry = client.post("/api/camera/autofocus", headers=AUTH)
        assert retry.status_code == 503
        assert failed_closed_autofocus.autofocus_calls == 1
        autofocus = client.post("/api/camera/autofocus", headers=AUTH)
        assert autofocus.status_code == 503
        assert autofocus.json() == {"detail": "Camera is unavailable."}
        assert private_path not in autofocus.text

    failed_capture = FakeProvider(
        [], capture_error=RuntimeError(private_path)
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_provider=failed_capture,
        )
    ) as client:
        response = client.post("/api/camera/captures", headers=AUTH)
        assert response.status_code == 502
        assert response.json() == {"detail": "Camera capture failed."}
        assert private_path not in response.text

    failed_autofocus = FakeProvider(
        [], autofocus_error=RuntimeError(private_path)
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_provider=failed_autofocus,
        )
    ) as client:
        response = client.post("/api/camera/autofocus", headers=AUTH)
        assert response.status_code == 502
        assert response.json() == {"detail": "Camera autofocus test failed."}
        assert private_path not in response.text


def test_supported_autofocus_cycle_returns_only_bounded_result_metadata(
    tmp_path: Path,
) -> None:
    configured = AutofocusStatus(
        capability="supported",
        mode="continuous",
        state="configured",
        range="macro",
    )
    provider = FakeProvider(
        [],
        autofocus_status=configured,
        autofocus_attempt=AutofocusAttempt(
            attempted=True,
            result="focused",
            autofocus=configured,
        ),
    )
    with TestClient(
        create_app(
            token_file=token_file(tmp_path),
            camera_provider=provider,
        )
    ) as client:
        response = client.post("/api/camera/autofocus", headers=AUTH)

    assert response.status_code == 200
    assert response.json() == {
        "simulated": False,
        "physicalArmMotion": False,
        "attempted": True,
        "result": "focused",
        "autofocus": {
            "capability": "supported",
            "mode": "continuous",
            "state": "configured",
            "range": "macro",
        },
    }
