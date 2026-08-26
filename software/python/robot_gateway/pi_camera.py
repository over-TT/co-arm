"""Fail-closed, in-memory Raspberry Pi camera access.

Picamera2 is intentionally imported only when ``start`` is called so this
module remains importable and testable away from Raspberry Pi hardware.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import io
import math
import threading
import time
from typing import Any, Callable

from .camera_profiles import (
    AUTO_PROFILE_ID,
    CAPTURE_PROFILE_DETAIL,
    CAPTURE_PROFILE_SURVEY,
    MODULE3_WIDE_PROFILE,
    MODULE3_WIDE_PROFILE_ID,
    CameraHardwareProfile,
    camera_profile as resolve_camera_profile,
    camera_profile_for_sensor,
    normalize_capture_profile,
)


JPEG_MIME_TYPE = "image/jpeg"
DEFAULT_WIDTH = MODULE3_WIDE_PROFILE.detail.width
DEFAULT_HEIGHT = MODULE3_WIDE_PROFILE.detail.height
MIN_WIDTH, MAX_WIDTH = 320, MODULE3_WIDE_PROFILE.native_width
MIN_HEIGHT, MAX_HEIGHT = 240, MODULE3_WIDE_PROFILE.native_height
DEFAULT_MAX_FRAME_BYTES = 16 * 1024 * 1024
MAX_FRAME_BYTES = DEFAULT_MAX_FRAME_BYTES
MAX_WARMUP_SECONDS = 60.0
DEFAULT_AUTOFOCUS_TIMEOUT_SECONDS = 10.0
DEFAULT_AUTOFOCUS_CANCEL_TIMEOUT_SECONDS = 2.0

AUTOFOCUS_CAPABILITY_UNKNOWN = "unknown"
AUTOFOCUS_CAPABILITY_UNSUPPORTED = "unsupported"
AUTOFOCUS_CAPABILITY_SUPPORTED = "supported"
AUTOFOCUS_MODE_UNKNOWN = "unknown"
AUTOFOCUS_MODE_FIXED = "fixed"
AUTOFOCUS_MODE_CONTINUOUS = "continuous"
AUTOFOCUS_STATE_NOT_STARTED = "not_started"
AUTOFOCUS_STATE_FIXED = "fixed"
AUTOFOCUS_STATE_CONFIGURED = "configured"
AUTOFOCUS_STATE_CONFIGURATION_FAILED = "configuration_failed"
AUTOFOCUS_STATE_CLOSED = "closed"
AUTOFOCUS_STATE_IDLE = "idle"
AUTOFOCUS_STATE_SCANNING = "scanning"
AUTOFOCUS_STATE_FOCUSED = "focused"
AUTOFOCUS_STATE_FAILED = "failed"
AUTOFOCUS_RANGE_UNAVAILABLE = "unavailable"
AUTOFOCUS_RANGE_NORMAL = "normal"
AUTOFOCUS_RANGE_FULL = "full"
AUTOFOCUS_RANGE_MACRO = "macro"
AUTOFOCUS_RESULT_FOCUSED = "focused"
AUTOFOCUS_RESULT_NOT_FOCUSED = "not_focused"
AUTOFOCUS_RESULT_UNSUPPORTED = "unsupported"
AUTOFOCUS_RESULT_UNAVAILABLE = "unavailable"
AUTOFOCUS_RESULT_TIMED_OUT = "timed_out"
FOCUS_QUALITY_METRIC = "libcamera_focus_fom"
FOCUS_QUALITY_STATUS_UNAVAILABLE = "unavailable"
FOCUS_QUALITY_STATUS_MEASURED = "measured"
FOCUS_QUALITY_COMPARISON = "same_subject_similar_framing_only"
MAX_FOCUS_FOM = 2_147_483_647

_AUTOFOCUS_CAPABILITIES = frozenset(
    {
        AUTOFOCUS_CAPABILITY_UNKNOWN,
        AUTOFOCUS_CAPABILITY_UNSUPPORTED,
        AUTOFOCUS_CAPABILITY_SUPPORTED,
    }
)
_AUTOFOCUS_MODES = frozenset(
    {
        AUTOFOCUS_MODE_UNKNOWN,
        AUTOFOCUS_MODE_FIXED,
        AUTOFOCUS_MODE_CONTINUOUS,
    }
)
_AUTOFOCUS_STATES = frozenset(
    {
        AUTOFOCUS_STATE_NOT_STARTED,
        AUTOFOCUS_STATE_FIXED,
        AUTOFOCUS_STATE_CONFIGURED,
        AUTOFOCUS_STATE_CONFIGURATION_FAILED,
        AUTOFOCUS_STATE_CLOSED,
        AUTOFOCUS_STATE_IDLE,
        AUTOFOCUS_STATE_SCANNING,
        AUTOFOCUS_STATE_FOCUSED,
        AUTOFOCUS_STATE_FAILED,
    }
)
_AUTOFOCUS_RANGES = frozenset(
    {
        AUTOFOCUS_RANGE_UNAVAILABLE,
        AUTOFOCUS_RANGE_NORMAL,
        AUTOFOCUS_RANGE_FULL,
        AUTOFOCUS_RANGE_MACRO,
    }
)
_AUTOFOCUS_RESULTS = frozenset(
    {
        AUTOFOCUS_RESULT_FOCUSED,
        AUTOFOCUS_RESULT_NOT_FOCUSED,
        AUTOFOCUS_RESULT_UNSUPPORTED,
        AUTOFOCUS_RESULT_UNAVAILABLE,
        AUTOFOCUS_RESULT_TIMED_OUT,
    }
)


class CameraError(RuntimeError):
    """Base class for sanitized camera errors."""


class CameraConfigurationError(CameraError):
    """Camera support is unavailable or could not be configured."""


class CameraLifecycleError(CameraError):
    """The requested operation is invalid in the current lifecycle state."""


class CameraCaptureError(CameraError):
    """A JPEG could not be captured and validated safely."""


class CameraAutofocusError(CameraError):
    """A requested autofocus cycle could not be completed safely."""


class CameraAutofocusFatalError(CameraAutofocusError):
    """Autofocus cleanup failed and the camera was taken out of service."""


MonotonicClock = Callable[[], float]


@dataclass(frozen=True, slots=True)
class AutofocusStatus:
    """Runtime focus capability without inferring hardware from a model label."""

    capability: str
    mode: str
    state: str
    range: str = AUTOFOCUS_RANGE_UNAVAILABLE
    lens_position: float | None = None

    def __post_init__(self) -> None:
        if self.capability not in _AUTOFOCUS_CAPABILITIES:
            raise ValueError("invalid autofocus capability")
        if self.mode not in _AUTOFOCUS_MODES:
            raise ValueError("invalid autofocus mode")
        if self.state not in _AUTOFOCUS_STATES:
            raise ValueError("invalid autofocus state")
        if self.range not in _AUTOFOCUS_RANGES:
            raise ValueError("invalid autofocus range")
        if self.lens_position is not None:
            if (
                isinstance(self.lens_position, bool)
                or not isinstance(self.lens_position, (int, float))
                or not math.isfinite(float(self.lens_position))
            ):
                raise ValueError("lens_position must be a finite number or None")
            object.__setattr__(self, "lens_position", float(self.lens_position))

    def as_dict(self) -> dict[str, object]:
        metadata: dict[str, object] = {
            "capability": self.capability,
            "mode": self.mode,
            "state": self.state,
            "range": self.range,
        }
        if self.lens_position is not None:
            metadata["lensPosition"] = self.lens_position
        return metadata


UNKNOWN_AUTOFOCUS_STATUS = AutofocusStatus(
    capability=AUTOFOCUS_CAPABILITY_UNKNOWN,
    mode=AUTOFOCUS_MODE_UNKNOWN,
    state=AUTOFOCUS_STATE_NOT_STARTED,
)


@dataclass(frozen=True, slots=True)
class AutofocusAttempt:
    """One bounded, operator-requested autofocus-cycle outcome.

    ``focused`` is only the camera state machine's report. It is not image
    evidence that subject detail became sharp.
    """

    attempted: bool
    result: str
    autofocus: AutofocusStatus

    def __post_init__(self) -> None:
        if not isinstance(self.attempted, bool):
            raise ValueError("attempted must be a boolean")
        if self.result not in _AUTOFOCUS_RESULTS:
            raise ValueError("invalid autofocus attempt result")
        if not isinstance(self.autofocus, AutofocusStatus):
            raise ValueError("autofocus must be an AutofocusStatus")
        expected_attempted = self.result in {
            AUTOFOCUS_RESULT_FOCUSED,
            AUTOFOCUS_RESULT_NOT_FOCUSED,
            AUTOFOCUS_RESULT_TIMED_OUT,
        }
        if self.attempted is not expected_attempted:
            raise ValueError("attempted does not match autofocus result")
        if (
            self.result == AUTOFOCUS_RESULT_UNSUPPORTED
            and self.autofocus.capability != AUTOFOCUS_CAPABILITY_UNSUPPORTED
        ):
            raise ValueError("unsupported result requires unsupported capability")
        if (
            self.result in {
                AUTOFOCUS_RESULT_FOCUSED,
                AUTOFOCUS_RESULT_NOT_FOCUSED,
                AUTOFOCUS_RESULT_TIMED_OUT,
            }
            and self.autofocus.capability != AUTOFOCUS_CAPABILITY_SUPPORTED
        ):
            raise ValueError("attempted autofocus requires supported capability")


@dataclass(frozen=True, slots=True)
class FocusQuality:
    """A same-frame, relative focus indication reported by libcamera.

    ``FocusFoM`` is deliberately not converted into a "sharp"/"blurry"
    judgement.  Its scale is platform- and scene-dependent, so it is useful
    only for comparing the same subject at similar framing while choosing a
    better camera standoff.
    """

    status: str
    value: int | None = None

    def __post_init__(self) -> None:
        if self.status not in {
            FOCUS_QUALITY_STATUS_UNAVAILABLE,
            FOCUS_QUALITY_STATUS_MEASURED,
        }:
            raise ValueError("invalid focus quality status")
        if self.status == FOCUS_QUALITY_STATUS_UNAVAILABLE:
            if self.value is not None:
                raise ValueError("unavailable focus quality must not have a value")
            return
        if (
            isinstance(self.value, bool)
            or not isinstance(self.value, int)
            or not 0 <= self.value <= MAX_FOCUS_FOM
        ):
            raise ValueError("measured focus quality must be a bounded integer")

    def as_dict(self) -> dict[str, object]:
        metadata: dict[str, object] = {
            "metric": FOCUS_QUALITY_METRIC,
            "status": self.status,
            "value": self.value,
            "higherIsSharper": True,
            "comparison": FOCUS_QUALITY_COMPARISON,
        }
        return metadata


UNKNOWN_FOCUS_QUALITY = FocusQuality(
    status=FOCUS_QUALITY_STATUS_UNAVAILABLE,
)


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    """Immutable JPEG data with stable origin and capture metadata."""

    data: bytes
    mime_type: str
    width: int
    height: int
    captured_at_utc: datetime
    captured_monotonic: float
    sha256: str
    camera_id: str
    sensor_model: str
    _monotonic_clock: MonotonicClock = field(repr=False, compare=False)
    autofocus: AutofocusStatus = UNKNOWN_AUTOFOCUS_STATUS
    focus_quality: FocusQuality = UNKNOWN_FOCUS_QUALITY

    @property
    def age_seconds(self) -> float:
        return self.age_at(self._monotonic_clock())

    def age_at(self, monotonic_now: float) -> float:
        if isinstance(monotonic_now, bool) or not isinstance(
            monotonic_now, (int, float)
        ) or not math.isfinite(float(monotonic_now)):
            raise ValueError("monotonic_now must be a finite number")
        return max(0.0, float(monotonic_now) - self.captured_monotonic)

    @property
    def jpeg_bytes(self) -> bytes:
        return self.data

    @property
    def captured_at(self) -> datetime:
        return self.captured_at_utc

    @property
    def sha256_hex(self) -> str:
        return self.sha256


class _FrameLimitExceeded(Exception):
    pass


class _BoundedBytesIO(io.BytesIO):
    def __init__(self, limit: int) -> None:
        super().__init__()
        self._limit = limit

    def write(self, data: Any) -> int:
        resulting_size = max(len(self.getbuffer()), self.tell() + len(data))
        if resulting_size > self._limit:
            raise _FrameLimitExceeded
        return super().write(data)


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _default_camera_factory() -> Any:
    try:
        from picamera2 import Picamera2

        return Picamera2()
    except (ImportError, ModuleNotFoundError):
        raise CameraConfigurationError(
            "Raspberry Pi camera support is unavailable; install Picamera2 on the Pi."
        ) from None
    except Exception:
        raise CameraConfigurationError(
            "Raspberry Pi camera initialization failed."
        ) from None


def _upside_down_mount_transform() -> Any:
    """Return Picamera2's native 180-degree transform for this camera mount."""

    try:
        from libcamera import Transform

        return Transform(hflip=1, vflip=1)
    except (ImportError, ModuleNotFoundError):
        raise CameraConfigurationError(
            "Raspberry Pi camera support is unavailable; install Picamera2 on the Pi."
        ) from None
    except Exception:
        raise CameraConfigurationError(
            "Raspberry Pi camera orientation could not be configured."
        ) from None


def _continuous_autofocus_value() -> Any:
    """Return libcamera's typed continuous-AF value when that API exists."""

    try:
        from libcamera import controls

        return controls.AfModeEnum.Continuous
    except (AttributeError, ImportError, ModuleNotFoundError):
        raise CameraConfigurationError(
            "Raspberry Pi continuous autofocus controls are unavailable."
        ) from None
    except Exception:
        raise CameraConfigurationError(
            "Raspberry Pi continuous autofocus controls could not be configured."
        ) from None


def _autofocus_range_value(name: str) -> Any:
    """Return one of libcamera's typed, bounded autofocus-range values."""

    attribute = {
        AUTOFOCUS_RANGE_NORMAL: "Normal",
        AUTOFOCUS_RANGE_FULL: "Full",
        AUTOFOCUS_RANGE_MACRO: "Macro",
    }.get(name)
    if attribute is None:
        raise CameraConfigurationError("Raspberry Pi autofocus range is invalid.")
    try:
        from libcamera import controls

        return getattr(controls.AfRangeEnum, attribute)
    except (AttributeError, ImportError, ModuleNotFoundError):
        raise CameraConfigurationError(
            f"Raspberry Pi {name} autofocus controls are unavailable."
        ) from None
    except Exception:
        raise CameraConfigurationError(
            f"Raspberry Pi {name} autofocus controls could not be configured."
        ) from None


def _cancel_autofocus_value() -> Any:
    """Return libcamera's typed one-shot autofocus cancellation value."""

    try:
        from libcamera import controls

        return controls.AfTriggerEnum.Cancel
    except (AttributeError, ImportError, ModuleNotFoundError):
        raise CameraConfigurationError(
            "Raspberry Pi autofocus cancellation controls are unavailable."
        ) from None
    except Exception:
        raise CameraConfigurationError(
            "Raspberry Pi autofocus cancellation could not be configured."
        ) from None


def _capture_autofocus_state(value: Any) -> str | None:
    """Normalize only the documented libcamera AF states."""

    name = getattr(value, "name", None)
    if not isinstance(name, str):
        name = value if isinstance(value, str) else None
    if name is None:
        return None
    normalized = name.rsplit(".", 1)[-1].strip().lower()
    return {
        "idle": AUTOFOCUS_STATE_IDLE,
        "scanning": AUTOFOCUS_STATE_SCANNING,
        "focused": AUTOFOCUS_STATE_FOCUSED,
        "failed": AUTOFOCUS_STATE_FAILED,
    }.get(normalized)


def _capture_lens_position(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _capture_focus_quality(metadata: Any) -> FocusQuality:
    """Keep only libcamera's bounded same-frame FocusFoM metadata."""

    if not isinstance(metadata, Mapping):
        return UNKNOWN_FOCUS_QUALITY
    value = metadata.get("FocusFoM")
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= MAX_FOCUS_FOM
    ):
        return UNKNOWN_FOCUS_QUALITY
    return FocusQuality(status=FOCUS_QUALITY_STATUS_MEASURED, value=value)


def _bounded_int(name: str, value: int, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _label(name: str, value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _camera_properties(camera: Any) -> Mapping[str, Any]:
    try:
        value = camera.camera_properties
    except Exception:
        return {}
    return value if isinstance(value, Mapping) else {}


def _driver_sensor_model(properties: Mapping[str, Any]) -> str | None:
    value = properties.get("Model")
    if value is None:
        return None
    if (
        not isinstance(value, str)
        or not value.strip()
        or len(value.strip()) > 64
        or not value.strip().isprintable()
    ):
        raise CameraConfigurationError("Raspberry Pi camera identity is invalid.")
    normalized = value.strip().lower()
    if normalized.startswith(("imx708_", "imx708-")):
        return MODULE3_WIDE_PROFILE.sensor_model
    if normalized.startswith(("ov5647_", "ov5647-")):
        return "ov5647"
    return normalized


def _dimension_pair(value: Any) -> tuple[int, int] | None:
    if value is None:
        return None
    if isinstance(value, (tuple, list)) and len(value) == 2:
        width, height = value
    else:
        width, height = getattr(value, "width", None), getattr(value, "height", None)
    if (
        isinstance(width, bool)
        or not isinstance(width, int)
        or width <= 0
        or isinstance(height, bool)
        or not isinstance(height, int)
        or height <= 0
    ):
        raise CameraConfigurationError("Raspberry Pi camera sensor dimensions are invalid.")
    return width, height


def _crop_dimensions(value: Any) -> tuple[int, int] | None:
    if value is None:
        return None
    if isinstance(value, (tuple, list)) and len(value) == 4:
        width, height = value[2], value[3]
    else:
        width, height = getattr(value, "width", None), getattr(value, "height", None)
    return _dimension_pair((width, height))


class PiCameraProvider:
    """Serialize a conservative Picamera2 still-capture lifecycle."""

    def __init__(
        self,
        *,
        width: int | None = None,
        height: int | None = None,
        max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
        warmup_seconds: float = 2.0,
        camera_id: str = "rpi-camera-0",
        sensor_model: str | None = None,
        camera_profile: str = MODULE3_WIDE_PROFILE_ID,
        camera_factory: Callable[[], Any] | None = None,
        transform_factory: Callable[[], Any] = _upside_down_mount_transform,
        sleeper: Callable[[float], None] = time.sleep,
        utc_clock: Callable[[], datetime] = _utc_now,
        monotonic_clock: MonotonicClock = time.monotonic,
    ) -> None:
        if camera_profile == AUTO_PROFILE_ID:
            configured_profile = MODULE3_WIDE_PROFILE
        else:
            configured_profile = resolve_camera_profile(camera_profile)
        if sensor_model is not None:
            # ``sensor_model`` predates camera profiles. Keep it as an explicit
            # legacy/auto candidate so existing OV5647 deployments do not
            # silently acquire Module 3 dimensions, while profile metadata is
            # still canonical after driver verification.
            legacy_profile = camera_profile_for_sensor(sensor_model)
            if legacy_profile is None:
                raise ValueError("sensor_model is not supported")
            if (
                camera_profile not in {AUTO_PROFILE_ID, MODULE3_WIDE_PROFILE_ID}
                and legacy_profile.profile_id != camera_profile
            ):
                raise ValueError("sensor_model does not match camera_profile")
            configured_profile = legacy_profile
        if width is not None:
            width = _bounded_int("width", width, MIN_WIDTH, MAX_WIDTH)
        if height is not None:
            height = _bounded_int("height", height, MIN_HEIGHT, MAX_HEIGHT)
        if width is not None and width > configured_profile.native_width:
            raise ValueError(
                f"width must be between {MIN_WIDTH} and {configured_profile.native_width}"
            )
        if height is not None and height > configured_profile.native_height:
            raise ValueError(
                f"height must be between {MIN_HEIGHT} and {configured_profile.native_height}"
            )
        self._configured_profile_id = camera_profile
        self._hardware_profile: CameraHardwareProfile = configured_profile
        self._width_override = width
        self._height_override = height
        self.width = width or configured_profile.detail.width
        self.height = height or configured_profile.detail.height
        self.max_frame_bytes = _bounded_int(
            "max_frame_bytes", max_frame_bytes, 1, MAX_FRAME_BYTES
        )
        if isinstance(warmup_seconds, bool) or not isinstance(
            warmup_seconds, (int, float)
        ):
            raise ValueError("warmup_seconds must be a finite number")
        self.warmup_seconds = float(warmup_seconds)
        if not math.isfinite(self.warmup_seconds) or not (
            0 <= self.warmup_seconds <= MAX_WARMUP_SECONDS
        ):
            raise ValueError("warmup_seconds must be between 0 and 60")
        callbacks = {
            "camera_factory": camera_factory or _default_camera_factory,
            "transform_factory": transform_factory,
            "sleeper": sleeper,
            "utc_clock": utc_clock,
            "monotonic_clock": monotonic_clock,
        }
        for name, callback in callbacks.items():
            if not callable(callback):
                raise ValueError(f"{name} must be callable")

        self.camera_id = _label("camera_id", camera_id)
        self.sensor_model = configured_profile.sensor_model
        self.identity_confidence = "configured_candidate"
        self._camera_factory = callbacks["camera_factory"]
        self._transform_factory = callbacks["transform_factory"]
        self._sleeper = sleeper
        self._utc_clock = utc_clock
        self._monotonic_clock = monotonic_clock
        self._camera: Any | None = None
        self._capture_configurations: dict[str, Any] = {}
        self._current_capture_profile: str | None = None
        self._autofocus_status = UNKNOWN_AUTOFOCUS_STATUS
        self._state = "new"
        self._lock = threading.Lock()

    @property
    def started(self) -> bool:
        with self._lock:
            return self._state == "started"

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._state == "closed"

    @property
    def autofocus_status(self) -> AutofocusStatus:
        with self._lock:
            return self._autofocus_status

    @property
    def camera_profile_metadata(self) -> dict[str, object]:
        with self._lock:
            metadata = self._hardware_profile.as_dict()
            metadata["captureProfiles"] = {
                capture_profile: {
                    "width": dimensions[0],
                    "height": dimensions[1],
                }
                for capture_profile in (
                    CAPTURE_PROFILE_SURVEY,
                    CAPTURE_PROFILE_DETAIL,
                )
                for dimensions in (self._mode_dimensions(capture_profile),)
            }
            return metadata

    def _mode_dimensions(self, capture_profile: str) -> tuple[int, int]:
        mode = self._hardware_profile.mode(capture_profile)
        return self._width_override or mode.width, self._height_override or mode.height

    def _select_and_verify_hardware_profile(self, camera: Any) -> None:
        properties = _camera_properties(camera)
        driver_model = _driver_sensor_model(properties)
        driver_dimensions = _dimension_pair(properties.get("PixelArraySize"))

        profile = self._hardware_profile
        if self._configured_profile_id == AUTO_PROFILE_ID and driver_model is not None:
            detected = camera_profile_for_sensor(driver_model)
            if detected is None:
                raise CameraConfigurationError(
                    "Raspberry Pi camera sensor is not supported by this gateway."
                )
            profile = detected

        if driver_model is not None and driver_model != profile.sensor_model:
            raise CameraConfigurationError(
                "Raspberry Pi camera sensor does not match the configured profile."
            )
        expected_dimensions = (profile.native_width, profile.native_height)
        if driver_dimensions is not None and driver_dimensions != expected_dimensions:
            raise CameraConfigurationError(
                "Raspberry Pi camera resolution does not match the configured profile."
            )

        self._hardware_profile = profile
        self.sensor_model = driver_model or profile.sensor_model
        self.identity_confidence = (
            "driver_reported" if driver_model is not None else "configured_candidate"
        )
        self.width, self.height = self._mode_dimensions(CAPTURE_PROFILE_DETAIL)

    def _verify_full_sensor_crop(self, camera: Any) -> None:
        properties = _camera_properties(camera)
        crop = _crop_dimensions(properties.get("ScalerCropMaximum"))
        if crop is not None and crop != (
            self._hardware_profile.native_width,
            self._hardware_profile.native_height,
        ):
            raise CameraConfigurationError(
                "Raspberry Pi camera mode does not expose the full configured sensor area."
            )

    def _activate_capture_profile(
        self,
        camera: Any,
        capture_profile: str,
        *,
        initial: bool,
    ) -> None:
        normalized = normalize_capture_profile(capture_profile)
        configuration = self._capture_configurations[normalized]
        if not initial:
            stop = getattr(camera, "stop", None)
            if not callable(stop):
                raise CameraConfigurationError(
                    "Raspberry Pi camera cannot switch capture profiles."
                )
            stop()
        camera.configure(configuration)
        self._verify_full_sensor_crop(camera)
        self._current_capture_profile = normalized
        self._autofocus_status = self._configure_autofocus(camera)
        camera.start()
        if self.warmup_seconds:
            self._sleeper(self.warmup_seconds)

    def start(self) -> None:
        with self._lock:
            if self._state == "closed":
                raise CameraLifecycleError("Camera provider is closed and cannot start.")
            if self._state == "started":
                return
            camera = None
            try:
                camera = self._camera_factory()
                if camera is None:
                    raise RuntimeError
                self._select_and_verify_hardware_profile(camera)
                transform = self._transform_factory()
                self._capture_configurations = {
                    capture_profile: camera.create_still_configuration(
                        main={"size": self._mode_dimensions(capture_profile)},
                        transform=transform,
                    )
                    for capture_profile in (
                        CAPTURE_PROFILE_SURVEY,
                        CAPTURE_PROFILE_DETAIL,
                    )
                }
                # Fresh action sessions need the full-FOV survey immediately;
                # capture() itself keeps the historical detail default.
                self._activate_capture_profile(
                    camera,
                    CAPTURE_PROFILE_SURVEY,
                    initial=True,
                )
            except CameraConfigurationError:
                self._release(camera)
                raise
            except Exception:
                self._release(camera)
                raise CameraConfigurationError(
                    "Raspberry Pi camera configuration failed."
                ) from None
            self._camera = camera
            self._state = "started"

    def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame:
        normalized_profile = normalize_capture_profile(profile)
        with self._lock:
            if self._state == "new":
                raise CameraLifecycleError(
                    "Camera provider has not been started; call start() before capture()."
                )
            if self._state == "closed":
                raise CameraLifecycleError("Camera provider is closed and cannot capture.")
            if self._camera is None:
                raise CameraLifecycleError("Camera provider is not ready to capture.")

            if self._current_capture_profile != normalized_profile:
                try:
                    self._activate_capture_profile(
                        self._camera,
                        normalized_profile,
                        initial=False,
                    )
                except Exception:
                    self._current_capture_profile = None
                    self._fail_autofocus_closed(self._camera)
                    raise CameraCaptureError(
                        "Raspberry Pi camera profile switch failed."
                    ) from None

            output = _BoundedBytesIO(self.max_frame_bytes)
            try:
                capture_metadata = self._camera.capture_file(output, format="jpeg")
                payload = output.getvalue()
            except _FrameLimitExceeded:
                raise CameraCaptureError(
                    f"Camera JPEG exceeds the {self.max_frame_bytes}-byte frame limit."
                ) from None
            except Exception:
                raise CameraCaptureError("Camera JPEG capture failed.") from None
            finally:
                output.close()
            if not payload:
                raise CameraCaptureError("Camera returned an empty JPEG frame.")
            if len(payload) > self.max_frame_bytes:
                raise CameraCaptureError(
                    f"Camera JPEG exceeds the {self.max_frame_bytes}-byte frame limit."
                )

            self._autofocus_status = self._capture_autofocus_status(
                capture_metadata
            )

            try:
                captured_monotonic = float(self._monotonic_clock())
                captured_at = self._utc_clock()
                if not math.isfinite(captured_monotonic):
                    raise ValueError
                if not isinstance(captured_at, datetime) or captured_at.tzinfo is None:
                    raise ValueError
                captured_at = captured_at.astimezone(timezone.utc)
            except Exception:
                raise CameraCaptureError("Camera capture timestamping failed.") from None
            data = bytes(payload)
            width, height = self._mode_dimensions(normalized_profile)
            return CapturedFrame(
                data=data,
                mime_type=JPEG_MIME_TYPE,
                width=width,
                height=height,
                captured_at_utc=captured_at,
                captured_monotonic=captured_monotonic,
                sha256=hashlib.sha256(data).hexdigest(),
                camera_id=self.camera_id,
                sensor_model=self.sensor_model,
                _monotonic_clock=self._monotonic_clock,
                autofocus=self._autofocus_status,
                focus_quality=_capture_focus_quality(capture_metadata),
            )

    def autofocus(self) -> AutofocusAttempt:
        """Run one explicit, bounded Picamera2 autofocus cycle.

        Picamera2's helper switches the camera into one-shot Auto mode. Every
        exit therefore restores the normal continuous/macro configuration. A
        driver-reported focus is returned as a result only; same-frame image
        metadata remains the stronger evidence for any subsequent capture.
        """

        with self._lock:
            if self._state != "started" or self._camera is None:
                raise CameraLifecycleError("Camera provider is not ready to autofocus.")
            current = self._autofocus_status
            if current.capability == AUTOFOCUS_CAPABILITY_UNSUPPORTED:
                return AutofocusAttempt(
                    attempted=False,
                    result=AUTOFOCUS_RESULT_UNSUPPORTED,
                    autofocus=current,
                )
            if (
                current.capability != AUTOFOCUS_CAPABILITY_SUPPORTED
                or current.mode != AUTOFOCUS_MODE_CONTINUOUS
                or current.state == AUTOFOCUS_STATE_CONFIGURATION_FAILED
            ):
                return AutofocusAttempt(
                    attempted=False,
                    result=AUTOFOCUS_RESULT_UNAVAILABLE,
                    autofocus=current,
                )

            camera = self._camera
            autofocus_cycle = getattr(camera, "autofocus_cycle", None)
            wait = getattr(camera, "wait", None)
            if not callable(autofocus_cycle) or not callable(wait):
                return AutofocusAttempt(
                    attempted=False,
                    result=AUTOFOCUS_RESULT_UNAVAILABLE,
                    autofocus=current,
                )

            self._prefer_full_range_for_manual_autofocus(camera)
            try:
                job = autofocus_cycle(wait=False)
            except Exception:
                try:
                    self._autofocus_status = self._configure_autofocus(camera)
                except Exception:
                    pass
                raise CameraAutofocusError(
                    "Raspberry Pi autofocus cycle could not be started."
                ) from None

            try:
                focused = wait(job, timeout=DEFAULT_AUTOFOCUS_TIMEOUT_SECONDS)
            except TimeoutError:
                try:
                    set_controls = getattr(camera, "set_controls", None)
                    if not callable(set_controls):
                        raise RuntimeError
                    set_controls({"AfTrigger": _cancel_autofocus_value()})
                    settled = wait(
                        job,
                        timeout=DEFAULT_AUTOFOCUS_CANCEL_TIMEOUT_SECONDS,
                    )
                    if not isinstance(settled, bool):
                        raise RuntimeError
                    restored = self._configure_autofocus(camera)
                    if not self._autofocus_configuration_is_ready(restored):
                        raise RuntimeError
                except Exception:
                    emergency_cancel = getattr(camera, "cancel_all_and_flush", None)
                    if callable(emergency_cancel):
                        try:
                            emergency_cancel()
                        except Exception:
                            pass
                    self._fail_autofocus_closed(camera)
                    raise CameraAutofocusFatalError(
                        "Raspberry Pi autofocus cycle timed out and could not be cleared."
                    ) from None
                self._autofocus_status = restored
                return AutofocusAttempt(
                    attempted=True,
                    result=AUTOFOCUS_RESULT_TIMED_OUT,
                    autofocus=restored,
                )
            except Exception:
                emergency_cancel = getattr(camera, "cancel_all_and_flush", None)
                if callable(emergency_cancel):
                    try:
                        emergency_cancel()
                    except Exception:
                        pass
                self._fail_autofocus_closed(camera)
                raise CameraAutofocusFatalError(
                    "Raspberry Pi autofocus cycle failed."
                ) from None

            restored = self._configure_autofocus(camera)
            if not self._autofocus_configuration_is_ready(restored):
                self._fail_autofocus_closed(camera)
                raise CameraAutofocusFatalError(
                    "Raspberry Pi autofocus cycle completed but normal focus mode was not restored."
                )
            self._autofocus_status = restored
            if not isinstance(focused, bool):
                raise CameraAutofocusError(
                    "Raspberry Pi autofocus cycle returned an invalid result."
                )
            return AutofocusAttempt(
                attempted=True,
                result=(
                    AUTOFOCUS_RESULT_FOCUSED
                    if focused
                    else AUTOFOCUS_RESULT_NOT_FOCUSED
                ),
                autofocus=restored,
            )

    def close(self) -> None:
        with self._lock:
            if self._state == "closed":
                return
            camera = self._camera
            self._camera = None
            self._capture_configurations = {}
            self._current_capture_profile = None
            self._state = "closed"
            self._autofocus_status = AutofocusStatus(
                capability=self._autofocus_status.capability,
                mode=self._autofocus_status.mode,
                state=AUTOFOCUS_STATE_CLOSED,
                range=self._autofocus_status.range,
            )
            if self._release(camera):
                raise CameraLifecycleError(
                    "Raspberry Pi camera did not shut down cleanly."
                ) from None

    @staticmethod
    def _autofocus_configuration_is_ready(status: AutofocusStatus) -> bool:
        return (
            status.capability == AUTOFOCUS_CAPABILITY_SUPPORTED
            and status.mode == AUTOFOCUS_MODE_CONTINUOUS
            and status.state == AUTOFOCUS_STATE_CONFIGURED
        )

    def _fail_autofocus_closed(self, camera: Any) -> None:
        """Make an indeterminate Picamera2 instance unavailable before release."""

        current = self._autofocus_status
        self._camera = None
        self._capture_configurations = {}
        self._current_capture_profile = None
        self._state = "closed"
        self._autofocus_status = AutofocusStatus(
            capability=current.capability,
            mode=current.mode,
            state=AUTOFOCUS_STATE_CLOSED,
            range=current.range,
        )
        self._release(camera)

    @staticmethod
    def _prefer_full_range_for_manual_autofocus(camera: Any) -> None:
        """Widen one explicit AF search without weakening its bounded outcome.

        Continuous survey and detail capture deliberately prefer Normal and
        Macro respectively. A user-triggered autofocus cycle is different: it
        should cover the Module 3 lens's full advertised travel, then the
        existing completion paths restore the active profile configuration.
        Older libcamera stacks that omit or reject ``AfRange.Full`` continue
        with their already-configured range.
        """

        try:
            camera_controls = camera.camera_controls
        except Exception:
            return
        if not isinstance(camera_controls, Mapping) or "AfRange" not in camera_controls:
            return
        set_controls = getattr(camera, "set_controls", None)
        if not callable(set_controls):
            return
        try:
            full_range = _autofocus_range_value(AUTOFOCUS_RANGE_FULL)
            set_controls({"AfRange": full_range})
        except Exception:
            # Full is an optional widening hint. The bounded autofocus cycle
            # remains useful on older stacks with only Normal or Macro.
            return

    def _configure_autofocus(self, camera: Any) -> AutofocusStatus:
        """Enable continuous AF only when Picamera2 advertises the control.

        Absence of ``AfMode`` means the live stack offers no motorized focus
        control, so captures remain in fixed-focus mode. If the control exists
        but cannot be configured, camera capture stays available and the
        failure is exposed in metadata instead of claiming focus is active.
        """

        try:
            camera_controls = camera.camera_controls
        except Exception:
            return AutofocusStatus(
                capability=AUTOFOCUS_CAPABILITY_UNKNOWN,
                mode=AUTOFOCUS_MODE_UNKNOWN,
                state=AUTOFOCUS_STATE_CONFIGURATION_FAILED,
            )
        if (
            not isinstance(camera_controls, Mapping)
            or "AfMode" not in camera_controls
        ):
            return AutofocusStatus(
                capability=AUTOFOCUS_CAPABILITY_UNSUPPORTED,
                mode=AUTOFOCUS_MODE_FIXED,
                state=AUTOFOCUS_STATE_FIXED,
            )

        set_controls = getattr(camera, "set_controls", None)
        if not callable(set_controls):
            return AutofocusStatus(
                capability=AUTOFOCUS_CAPABILITY_SUPPORTED,
                mode=AUTOFOCUS_MODE_UNKNOWN,
                state=AUTOFOCUS_STATE_CONFIGURATION_FAILED,
            )
        try:
            continuous_controls = {"AfMode": _continuous_autofocus_value()}
        except Exception:
            return AutofocusStatus(
                capability=AUTOFOCUS_CAPABILITY_SUPPORTED,
                mode=AUTOFOCUS_MODE_UNKNOWN,
                state=AUTOFOCUS_STATE_CONFIGURATION_FAILED,
            )

        active_profile = self._current_capture_profile or CAPTURE_PROFILE_SURVEY
        preferences = self._hardware_profile.mode(active_profile).autofocus_ranges
        if "AfRange" not in camera_controls:
            preferences = (AUTOFOCUS_RANGE_UNAVAILABLE,)

        for focus_range in preferences:
            requested_controls = dict(continuous_controls)
            if focus_range != AUTOFOCUS_RANGE_UNAVAILABLE:
                try:
                    requested_controls["AfRange"] = _autofocus_range_value(
                        focus_range
                    )
                except Exception:
                    continue
            try:
                set_controls(requested_controls)
            except Exception:
                continue
            return AutofocusStatus(
                capability=AUTOFOCUS_CAPABILITY_SUPPORTED,
                mode=AUTOFOCUS_MODE_CONTINUOUS,
                state=AUTOFOCUS_STATE_CONFIGURED,
                range=focus_range,
            )

        return AutofocusStatus(
            capability=AUTOFOCUS_CAPABILITY_SUPPORTED,
            mode=AUTOFOCUS_MODE_UNKNOWN,
            state=AUTOFOCUS_STATE_CONFIGURATION_FAILED,
        )

    def _capture_autofocus_status(self, metadata: Any) -> AutofocusStatus:
        current = self._autofocus_status
        if (
            current.capability != AUTOFOCUS_CAPABILITY_SUPPORTED
            or current.mode != AUTOFOCUS_MODE_CONTINUOUS
        ):
            return current
        if not isinstance(metadata, Mapping):
            return AutofocusStatus(
                capability=current.capability,
                mode=current.mode,
                state=AUTOFOCUS_STATE_CONFIGURED,
                range=current.range,
            )

        observed_state = _capture_autofocus_state(metadata.get("AfState"))
        lens_position = _capture_lens_position(metadata.get("LensPosition"))
        return AutofocusStatus(
            capability=current.capability,
            mode=current.mode,
            state=observed_state or AUTOFOCUS_STATE_CONFIGURED,
            range=current.range,
            lens_position=lens_position,
        )

    @staticmethod
    def _release(camera: Any | None) -> bool:
        failed = False
        if camera is not None:
            for method_name in ("stop", "close"):
                method = getattr(camera, method_name, None)
                if callable(method):
                    try:
                        method()
                    except Exception:
                        failed = True
        return failed

    def __enter__(self) -> "PiCameraProvider":
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


RaspberryPiCameraProvider = PiCameraProvider
PiCamera2Provider = PiCameraProvider

__all__ = [
    "AutofocusAttempt",
    "AutofocusStatus",
    "CameraAutofocusError",
    "CameraAutofocusFatalError",
    "CameraCaptureError",
    "CameraConfigurationError",
    "CameraError",
    "CameraLifecycleError",
    "CapturedFrame",
    "FocusQuality",
    "PiCamera2Provider",
    "PiCameraProvider",
    "RaspberryPiCameraProvider",
]
