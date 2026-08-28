"""Authenticated, read-only camera evidence for the Raspberry Pi gateway.

Frames remain in a small in-memory history.  This module intentionally offers
no filesystem, shell, GPIO, serial, or robot-motion capability.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Mapping
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from datetime import timezone
import hashlib
import inspect
import math
import re
import secrets
import threading
import time
from typing import Protocol

from fastapi import APIRouter, Body, HTTPException, Response, status

from .camera_profiles import (
    CAPTURE_PROFILE_DETAIL,
    CAPTURE_PROFILE_SURVEY,
    normalize_capture_profile,
)

from .pi_camera import (
    AutofocusAttempt,
    AutofocusStatus,
    CameraAutofocusFatalError,
    CapturedFrame,
    FocusQuality,
    JPEG_MIME_TYPE,
    UNKNOWN_AUTOFOCUS_STATUS,
)


DEFAULT_MAX_FRAMES = 8
DEFAULT_MAX_TOTAL_BYTES = 64 * 1024 * 1024
MAX_TOTAL_BYTES = 64 * 1024 * 1024
DEFAULT_MAX_TRANSFERS = 4
DEFAULT_MAX_TRANSFER_BYTES = 64 * 1024 * 1024
DEFAULT_TRANSFER_TTL_SECONDS = 30.0
IDENTITY_CONFIDENCE = "configured_candidate"
_SAFE_FRAME_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")
_SAFE_CAMERA_SOURCE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


class CameraProvider(Protocol):
    """Narrow provider contract used by the evidence service and test fakes."""

    camera_id: str
    sensor_model: str

    @property
    def max_frame_bytes(self) -> int: ...

    @property
    def autofocus_status(self) -> AutofocusStatus: ...

    def start(self) -> None: ...

    def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame: ...

    def autofocus(self) -> AutofocusAttempt: ...

    def close(self) -> None: ...


class CameraUnavailableError(RuntimeError):
    """The configured camera did not become available."""


class CameraEvidenceCaptureError(RuntimeError):
    """A frame could not be captured and bound to robot state safely."""


class CameraEvidenceCaptureCancelledError(RuntimeError):
    """The physical shutter authorization was revoked before capture began."""


class CameraEvidenceTransferUnavailableError(RuntimeError):
    """The bounded exact-frame transfer lane cannot accept another capture."""


class CameraEvidenceAutofocusError(RuntimeError):
    """An autofocus cycle could not be attempted safely."""


@dataclass(frozen=True, slots=True)
class CameraEvidenceRecord:
    frame_id: str
    frame: CapturedFrame
    capture_profile: str
    state_revision: int
    content_sha256: str


@dataclass(frozen=True, slots=True)
class CameraEvidenceTransfer:
    token: str
    record: CameraEvidenceRecord
    expires_at: float


def _opaque_frame_id() -> str:
    return f"camera_{secrets.token_urlsafe(18)}"


def _opaque_transfer_token() -> str:
    return f"transfer_{secrets.token_urlsafe(24)}"


def _configured_label(provider: CameraProvider, name: str) -> str:
    value = getattr(provider, name, None)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"camera provider {name} must be a non-empty string")
    return value.strip()


def _bounded_profile_text(value: object, name: str, *, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > maximum
        or not value.isprintable()
    ):
        raise ValueError(f"camera profile {name} must be a bounded label")
    return value


def _bounded_profile_dimension(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65_535:
        raise ValueError(f"camera profile {name} must be a bounded dimension")
    return value


def _bounded_profile_number(
    value: object,
    name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or not minimum <= float(value) <= maximum
    ):
        raise ValueError(f"camera profile {name} must be a bounded number")
    return float(value)


def _profile_dimensions(value: object, name: str) -> dict[str, int]:
    if not isinstance(value, Mapping) or set(value) != {"width", "height"}:
        raise ValueError(f"camera profile {name} dimensions are invalid")
    return {
        "width": _bounded_profile_dimension(value.get("width"), f"{name} width"),
        "height": _bounded_profile_dimension(value.get("height"), f"{name} height"),
    }


def _provider_camera_profile(provider: CameraProvider) -> dict[str, object] | None:
    try:
        value = getattr(provider, "camera_profile_metadata")
    except Exception:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("camera provider camera profile must be an object")
    required = {
        "id",
        "productName",
        "sensorModel",
        "lensVariant",
        "nativeDimensions",
        "nominalFocalLengthMm",
        "nominalFieldOfViewDegrees",
        "captureProfiles",
    }
    optional = {
        "simulatedPinholeFitAxis",
        "simulatedPinholeFieldOfViewDegrees",
    }
    if not required <= set(value) <= required | optional:
        raise ValueError("camera provider camera profile has invalid fields")
    if bool(optional & set(value)) and not optional <= set(value):
        raise ValueError("camera provider simulated projection is incomplete")
    fov = value.get("nominalFieldOfViewDegrees")
    if not isinstance(fov, Mapping) or set(fov) != {"horizontal", "vertical"}:
        raise ValueError("camera provider field of view is invalid")
    capture_profiles = value.get("captureProfiles")
    if not isinstance(capture_profiles, Mapping) or set(capture_profiles) != {
        CAPTURE_PROFILE_SURVEY,
        CAPTURE_PROFILE_DETAIL,
    }:
        raise ValueError("camera provider capture profiles are invalid")
    normalized = {
        "id": _bounded_profile_text(value.get("id"), "id"),
        "productName": _bounded_profile_text(value.get("productName"), "productName"),
        "sensorModel": _bounded_profile_text(value.get("sensorModel"), "sensorModel"),
        "lensVariant": _bounded_profile_text(value.get("lensVariant"), "lensVariant"),
        "nativeDimensions": _profile_dimensions(
            value.get("nativeDimensions"), "native"
        ),
        "nominalFocalLengthMm": _bounded_profile_number(
            value.get("nominalFocalLengthMm"),
            "nominalFocalLengthMm",
            minimum=0.1,
            maximum=100.0,
        ),
        "nominalFieldOfViewDegrees": {
            "horizontal": _bounded_profile_number(
                fov.get("horizontal"), "horizontal field of view", minimum=0.1, maximum=179.9
            ),
            "vertical": _bounded_profile_number(
                fov.get("vertical"), "vertical field of view", minimum=0.1, maximum=179.9
            ),
        },
        "captureProfiles": {
            CAPTURE_PROFILE_SURVEY: _profile_dimensions(
                capture_profiles.get(CAPTURE_PROFILE_SURVEY), CAPTURE_PROFILE_SURVEY
            ),
            CAPTURE_PROFILE_DETAIL: _profile_dimensions(
                capture_profiles.get(CAPTURE_PROFILE_DETAIL), CAPTURE_PROFILE_DETAIL
            ),
        },
    }
    if optional <= set(value):
        fit_axis = value.get("simulatedPinholeFitAxis")
        simulated_fov = value.get("simulatedPinholeFieldOfViewDegrees")
        if fit_axis not in {"horizontal", "vertical"}:
            raise ValueError("camera provider simulated pinhole fit axis is invalid")
        if (
            not isinstance(simulated_fov, Mapping)
            or set(simulated_fov) != {"horizontal", "vertical"}
        ):
            raise ValueError("camera provider simulated pinhole field of view is invalid")
        normalized["simulatedPinholeFitAxis"] = fit_axis
        normalized["simulatedPinholeFieldOfViewDegrees"] = {
            "horizontal": _bounded_profile_number(
                simulated_fov.get("horizontal"),
                "simulated horizontal field of view",
                minimum=0.1,
                maximum=179.9,
            ),
            "vertical": _bounded_profile_number(
                simulated_fov.get("vertical"),
                "simulated vertical field of view",
                minimum=0.1,
                maximum=179.9,
            ),
        }
    return normalized


def _provider_accepts_capture_profile(provider: CameraProvider) -> bool:
    try:
        parameters = inspect.signature(provider.capture).parameters.values()
    except (TypeError, ValueError):
        return False
    return any(
        parameter.name == "profile"
        or parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in parameters
    )


class CameraEvidenceService:
    """Serialize camera access and retain a byte- and count-bounded history."""

    def __init__(
        self,
        provider: CameraProvider,
        *,
        status_callback: Callable[[], Mapping[str, object]],
        max_frames: int = DEFAULT_MAX_FRAMES,
        max_total_bytes: int = DEFAULT_MAX_TOTAL_BYTES,
        max_transfers: int = DEFAULT_MAX_TRANSFERS,
        max_transfer_bytes: int = DEFAULT_MAX_TRANSFER_BYTES,
        transfer_ttl_seconds: float = DEFAULT_TRANSFER_TTL_SECONDS,
        id_factory: Callable[[], str] = _opaque_frame_id,
        transfer_token_factory: Callable[[], str] = _opaque_transfer_token,
        monotonic_clock: Callable[[], float] = time.monotonic,
        simulated: bool = False,
        source: str = "picamera2",
        identity_confidence: str = IDENTITY_CONFIDENCE,
    ) -> None:
        if isinstance(max_frames, bool) or not isinstance(max_frames, int):
            raise ValueError("max_frames must be an integer")
        if not 1 <= max_frames <= DEFAULT_MAX_FRAMES:
            raise ValueError(f"max_frames must be between 1 and {DEFAULT_MAX_FRAMES}")
        if isinstance(max_total_bytes, bool) or not isinstance(max_total_bytes, int):
            raise ValueError("max_total_bytes must be an integer")
        if not 1 <= max_total_bytes <= MAX_TOTAL_BYTES:
            raise ValueError(
                f"max_total_bytes must be between 1 and {MAX_TOTAL_BYTES}"
            )
        if not callable(status_callback):
            raise ValueError("status_callback must be callable")
        if not callable(id_factory):
            raise ValueError("id_factory must be callable")
        if isinstance(max_transfers, bool) or not isinstance(max_transfers, int):
            raise ValueError("max_transfers must be an integer")
        if not 1 <= max_transfers <= DEFAULT_MAX_TRANSFERS:
            raise ValueError(
                f"max_transfers must be between 1 and {DEFAULT_MAX_TRANSFERS}"
            )
        if isinstance(max_transfer_bytes, bool) or not isinstance(
            max_transfer_bytes, int
        ):
            raise ValueError("max_transfer_bytes must be an integer")
        if not 1 <= max_transfer_bytes <= MAX_TOTAL_BYTES:
            raise ValueError(
                f"max_transfer_bytes must be between 1 and {MAX_TOTAL_BYTES}"
            )
        if (
            isinstance(transfer_ttl_seconds, bool)
            or not isinstance(transfer_ttl_seconds, (int, float))
            or not math.isfinite(float(transfer_ttl_seconds))
            or not 1.0 <= float(transfer_ttl_seconds) <= 300.0
        ):
            raise ValueError("transfer_ttl_seconds must be between 1 and 300")
        if not callable(transfer_token_factory):
            raise ValueError("transfer_token_factory must be callable")
        if not callable(monotonic_clock):
            raise ValueError("monotonic_clock must be callable")
        if not isinstance(simulated, bool):
            raise ValueError("simulated must be a boolean")
        if not isinstance(source, str) or _SAFE_CAMERA_SOURCE.fullmatch(source) is None:
            raise ValueError("camera source must be a bounded identifier")
        if (
            not isinstance(identity_confidence, str)
            or not identity_confidence
            or len(identity_confidence) > 64
            or not identity_confidence.isprintable()
        ):
            raise ValueError("camera identity confidence must be a bounded label")

        self._provider = provider
        self._camera_id = _configured_label(provider, "camera_id")
        self._sensor_model = _configured_label(provider, "sensor_model")
        self._status_callback = status_callback
        self._simulated = simulated
        self._source = source
        self._identity_confidence = identity_confidence
        self._camera_profile = _provider_camera_profile(provider)
        self._provider_capture_has_profile = _provider_accepts_capture_profile(provider)
        self._max_frames = max_frames
        self._max_total_bytes = max_total_bytes
        self._max_transfers = max_transfers
        self._max_transfer_bytes = max_transfer_bytes
        provider_frame_limit = getattr(provider, "max_frame_bytes", None)
        # A composed move may start only when the transfer lane can reserve the
        # provider's complete worst-case JPEG. Never invent a per-slot ceiling:
        # a structurally valid non-Pi provider without a declared bound could
        # otherwise move the arm and operate the shutter before discovering
        # that its frame cannot be handed off.
        self._transfer_frame_limit: int | None = (
            provider_frame_limit
            if isinstance(provider_frame_limit, int)
            and not isinstance(provider_frame_limit, bool)
            and provider_frame_limit > 0
            and provider_frame_limit <= max_transfer_bytes
            else None
        )
        self._transfer_ttl_seconds = float(transfer_ttl_seconds)
        self._id_factory = id_factory
        self._transfer_token_factory = transfer_token_factory
        self._monotonic_clock = monotonic_clock
        self._records: OrderedDict[str, CameraEvidenceRecord] = OrderedDict()
        # Exact composed-operation frames never enter ordinary camera history.
        # An opaque, one-use token addresses a separately bounded handoff store,
        # so dashboard latest/frame reads cannot consume or evict arm evidence.
        self._transfers: OrderedDict[str, CameraEvidenceTransfer] = OrderedDict()
        self._total_bytes = 0
        self._transfer_bytes = 0
        self._state = "new"
        self._lock = threading.RLock()

    def _refresh_provider_identity(self) -> None:
        self._camera_id = _configured_label(self._provider, "camera_id")
        self._sensor_model = _configured_label(self._provider, "sensor_model")
        confidence = getattr(
            self._provider,
            "identity_confidence",
            self._identity_confidence,
        )
        if (
            not isinstance(confidence, str)
            or not confidence
            or len(confidence) > 64
            or not confidence.isprintable()
        ):
            raise ValueError("camera provider identity confidence is invalid")
        self._identity_confidence = confidence
        self._camera_profile = _provider_camera_profile(self._provider)

    @property
    def simulated(self) -> bool:
        return self._simulated

    @property
    def source(self) -> str:
        return self._source

    def start(self) -> None:
        """Start once, retaining only a sanitized unavailable state on failure."""

        with self._lock:
            if self._state == "started":
                return
            if self._state in {"unavailable", "closed"}:
                return
            try:
                self._provider.start()
                self._refresh_provider_identity()
            except Exception:
                # Provider exceptions can contain device paths or driver details.
                try:
                    self._provider.close()
                except Exception:
                    pass
                self._state = "unavailable"
                return
            self._state = "started"

    def close(self) -> None:
        """Close the explicitly configured provider without leaking driver errors."""

        with self._lock:
            if self._state == "closed":
                return
            try:
                self._provider.close()
            except Exception:
                pass
            self._transfers.clear()
            self._transfer_bytes = 0
            self._state = "closed"

    def status(self) -> dict[str, object]:
        with self._lock:
            latest_id = next(reversed(self._records), None)
            report: dict[str, object] = {
                "simulated": self._simulated,
                "readOnly": True,
                "cameraId": self._camera_id,
                "sensorModel": self._sensor_model,
                "identityConfidence": self._identity_confidence,
                "autofocus": self._provider_autofocus_status().as_dict(),
                "state": self._state,
                "available": self._state == "started",
                "latestFrameId": latest_id,
                "historySize": len(self._records),
                "historyLimit": self._max_frames,
                "retainedBytes": self._total_bytes,
                "byteLimit": self._max_total_bytes,
            }
            if self._camera_profile is not None:
                report["cameraProfile"] = self._camera_profile
            return report

    def capture(
        self, *, profile: str = CAPTURE_PROFILE_DETAIL
    ) -> dict[str, object]:
        normalized_profile = normalize_capture_profile(profile)
        with self._lock:
            record = self._capture_record(
                self._max_total_bytes,
                capture_profile=normalized_profile,
            )
            self._records[record.frame_id] = record
            self._total_bytes += len(record.frame.data)
            self._evict_to_limits()
            return self._metadata(record)

    def capture_transfer(
        self,
        *,
        profile: str = CAPTURE_PROFILE_DETAIL,
        shutter_guard: AbstractContextManager[object] | None = None,
    ) -> dict[str, object]:
        """Capture into the bounded one-use exact-frame transfer lane."""

        normalized_profile = normalize_capture_profile(profile)
        with self._lock:
            self._expire_transfers_locked()
            frame_limit = self._transfer_frame_limit
            if frame_limit is None:
                raise CameraEvidenceTransferUnavailableError(
                    "Camera transfer capacity is unavailable."
                )
            # Refuse before operating the shutter when count capacity is known
            # to be unavailable. Live handoffs are never silently retired.
            if len(self._transfers) >= self._max_transfers:
                raise CameraEvidenceTransferUnavailableError(
                    "Camera transfer capacity is unavailable."
                )
            if (
                self._max_transfer_bytes - self._transfer_bytes
                < frame_limit
            ):
                raise CameraEvidenceTransferUnavailableError(
                    "Camera transfer capacity is unavailable."
                )
            # Enter the caller's physical authorization only after camera
            # serialization and capacity checks. Holding it across the
            # provider call gives STOP and shutter start one honest ordering:
            # a STOP that wins while this request waits for the camera lock
            # cancels before the provider can operate the shutter.
            with shutter_guard if shutter_guard is not None else nullcontext():
                record = self._capture_record(
                    frame_limit,
                    transfer=True,
                    capture_profile=normalized_profile,
                )
            byte_count = len(record.frame.data)
            if self._transfer_bytes + byte_count > self._max_transfer_bytes:
                raise CameraEvidenceTransferUnavailableError(
                    "Camera transfer capacity is unavailable."
                )
            token = self._new_transfer_token()
            self._transfers[token] = CameraEvidenceTransfer(
                token=token,
                record=record,
                expires_at=self._now() + self._transfer_ttl_seconds,
            )
            self._transfer_bytes += byte_count
            metadata = self._metadata(record)
            transfer_url = f"/api/camera/transfers/{token}"
            metadata.update(
                {
                    "frameUrl": transfer_url,
                    "transferToken": token,
                    "transferUrl": transfer_url,
                    "oneUse": True,
                }
            )
            return metadata

    def transfer_available(self) -> bool:
        """Whether a composed operation can reserve the next transfer slot."""

        with self._lock:
            self._expire_transfers_locked()
            frame_limit = self._transfer_frame_limit
            return (
                frame_limit is not None
                and len(self._transfers) < self._max_transfers
                and self._max_transfer_bytes - self._transfer_bytes
                >= frame_limit
            )

    def has_transfer(self, token: str) -> bool:
        with self._lock:
            self._expire_transfers_locked()
            return token in self._transfers

    def transfer_captured_monotonic(self, token: str) -> float | None:
        """Return the shutter stamp for an unconsumed exact-frame handoff."""

        with self._lock:
            self._expire_transfers_locked()
            transfer = self._transfers.get(token)
            return (
                transfer.record.frame.captured_monotonic
                if transfer is not None
                else None
            )

    def claim_transfer(self, token: str) -> CameraEvidenceRecord | None:
        """Atomically consume one exact-frame handoff by its opaque token."""

        with self._lock:
            self._expire_transfers_locked()
            transfer = self._transfers.pop(token, None)
            if transfer is None:
                return None
            self._transfer_bytes -= len(transfer.record.frame.data)
            return transfer.record

    def discard_transfer(self, token: str) -> bool:
        with self._lock:
            self._expire_transfers_locked()
            transfer = self._transfers.pop(token, None)
            if transfer is None:
                return False
            self._transfer_bytes -= len(transfer.record.frame.data)
            return True

    def discard(self, frame_id: str) -> bool:
        """Unpublish a frame whose surrounding physical evidence was invalidated."""

        with self._lock:
            record = self._records.pop(frame_id, None)
            if record is None:
                return False
            self._total_bytes -= len(record.frame.data)
            return True

    def autofocus(self) -> dict[str, object]:
        with self._lock:
            if self._state != "started":
                raise CameraUnavailableError("Camera is unavailable.")
            try:
                attempt = self._provider.autofocus()
            except CameraAutofocusFatalError:
                self._state = "unavailable"
                raise CameraEvidenceAutofocusError(
                    "Camera autofocus test failed."
                ) from None
            except Exception:
                raise CameraEvidenceAutofocusError(
                    "Camera autofocus test failed."
                ) from None
            if not isinstance(attempt, AutofocusAttempt):
                raise CameraEvidenceAutofocusError("Camera autofocus test failed.")
            return {
                "simulated": self._simulated,
                "physicalArmMotion": False,
                "attempted": attempt.attempted,
                "result": attempt.result,
                "autofocus": attempt.autofocus.as_dict(),
            }

    def latest(self) -> dict[str, object] | None:
        with self._lock:
            if not self._records:
                return None
            return self._metadata(next(reversed(self._records.values())))

    def frame(self, frame_id: str) -> CameraEvidenceRecord | None:
        with self._lock:
            return self._records.get(frame_id)

    def _capture_record(
        self,
        max_frame_bytes: int,
        *,
        transfer: bool = False,
        capture_profile: str = CAPTURE_PROFILE_DETAIL,
    ) -> CameraEvidenceRecord:
        if self._state != "started":
            raise CameraUnavailableError("Camera is unavailable.")
        if (
            not self._provider_capture_has_profile
            and capture_profile != CAPTURE_PROFILE_DETAIL
        ):
            # Legacy providers remain valid for the historical detail default,
            # but a survey request must never be mislabeled when the provider
            # cannot actually select that mode.
            raise CameraEvidenceCaptureError("Camera capture failed.")
        try:
            frame = (
                self._provider.capture(profile=capture_profile)
                if self._provider_capture_has_profile
                else self._provider.capture()
            )
        except Exception:
            raise CameraEvidenceCaptureError("Camera capture failed.") from None
        try:
            self._validate_frame(frame, capture_profile=capture_profile)
        except CameraEvidenceCaptureError:
            raise
        except Exception:
            raise CameraEvidenceCaptureError("Camera capture failed.") from None
        if len(frame.data) > max_frame_bytes:
            if transfer:
                raise CameraEvidenceTransferUnavailableError(
                    "Camera transfer capacity is unavailable."
                )
            raise CameraEvidenceCaptureError("Camera capture failed.")
        try:
            revision = self._status_callback()["stateRevision"]
        except Exception:
            raise CameraEvidenceCaptureError("Camera capture failed.") from None
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise CameraEvidenceCaptureError("Camera capture failed.")
        frame_id = self._new_frame_id()
        digest = hashlib.sha256(frame.data).hexdigest()
        return CameraEvidenceRecord(
            frame_id=frame_id,
            frame=frame,
            capture_profile=capture_profile,
            state_revision=revision,
            content_sha256=f"sha256:{digest}",
        )

    def _new_frame_id(self) -> str:
        for _ in range(8):
            try:
                identifier = self._id_factory()
            except Exception:
                raise CameraEvidenceCaptureError("Camera capture failed.") from None
            if not isinstance(identifier, str) or _SAFE_FRAME_ID.fullmatch(identifier) is None:
                raise CameraEvidenceCaptureError("Camera capture failed.")
            if identifier not in self._records and all(
                transfer.record.frame_id != identifier
                for transfer in self._transfers.values()
            ):
                return identifier
        raise CameraEvidenceCaptureError("Camera capture failed.")

    def _new_transfer_token(self) -> str:
        for _ in range(8):
            try:
                token = self._transfer_token_factory()
            except Exception:
                raise CameraEvidenceCaptureError("Camera capture failed.") from None
            if (
                isinstance(token, str)
                and _SAFE_FRAME_ID.fullmatch(token) is not None
                and token not in self._transfers
            ):
                return token
        raise CameraEvidenceCaptureError("Camera capture failed.")

    def _now(self) -> float:
        try:
            now = float(self._monotonic_clock())
        except Exception:
            raise CameraEvidenceCaptureError("Camera capture failed.") from None
        if not math.isfinite(now):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        return now

    def _expire_transfers_locked(self) -> None:
        now = self._now()
        expired = [
            token
            for token, transfer in self._transfers.items()
            if transfer.expires_at <= now
        ]
        for token in expired:
            transfer = self._transfers.pop(token)
            self._transfer_bytes -= len(transfer.record.frame.data)

    def _provider_autofocus_status(self) -> AutofocusStatus:
        try:
            autofocus = self._provider.autofocus_status
        except Exception:
            return UNKNOWN_AUTOFOCUS_STATUS
        if not isinstance(autofocus, AutofocusStatus):
            return UNKNOWN_AUTOFOCUS_STATUS
        return autofocus

    def _validate_frame(
        self,
        frame: CapturedFrame,
        *,
        capture_profile: str,
    ) -> None:
        if not isinstance(frame, CapturedFrame):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if not isinstance(frame.autofocus, AutofocusStatus):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if not isinstance(frame.focus_quality, FocusQuality):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if frame.mime_type != JPEG_MIME_TYPE or not frame.data:
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if (
            frame.camera_id != self._camera_id
            or frame.sensor_model != self._sensor_model
        ):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if (
            frame.captured_at_utc.tzinfo is None
            or frame.captured_at_utc.utcoffset() is None
            or isinstance(frame.captured_monotonic, bool)
            or not isinstance(frame.captured_monotonic, (int, float))
            or not math.isfinite(float(frame.captured_monotonic))
            or not isinstance(frame.sha256, str)
        ):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if (
            isinstance(frame.width, bool)
            or not isinstance(frame.width, int)
            or frame.width <= 0
            or isinstance(frame.height, bool)
            or not isinstance(frame.height, int)
            or frame.height <= 0
        ):
            raise CameraEvidenceCaptureError("Camera capture failed.")
        if self._camera_profile is not None:
            capture_profiles = self._camera_profile.get("captureProfiles")
            expected = (
                capture_profiles.get(capture_profile)
                if isinstance(capture_profiles, Mapping)
                else None
            )
            if not isinstance(expected, Mapping) or (
                frame.width != expected.get("width")
                or frame.height != expected.get("height")
            ):
                raise CameraEvidenceCaptureError("Camera capture failed.")
        digest = hashlib.sha256(frame.data).hexdigest()
        if not secrets.compare_digest(digest, frame.sha256):
            raise CameraEvidenceCaptureError("Camera capture failed.")

    def _evict_to_limits(self) -> None:
        while (
            len(self._records) > self._max_frames
            or self._total_bytes > self._max_total_bytes
        ):
            _, evicted = self._records.popitem(last=False)
            self._total_bytes -= len(evicted.frame.data)

    @staticmethod
    def _age_seconds(frame: CapturedFrame) -> float:
        try:
            age = float(frame.age_seconds)
        except Exception:
            return 0.0
        if not math.isfinite(age):
            return 0.0
        return max(0.0, age)

    def _metadata(self, record: CameraEvidenceRecord) -> dict[str, object]:
        captured_at = (
            record.frame.captured_at_utc.astimezone(timezone.utc)
            .isoformat(timespec="milliseconds")
            .replace("+00:00", "Z")
        )
        metadata: dict[str, object] = {
            "simulated": self._simulated,
            "readOnly": True,
            "source": self._source,
            "frameId": record.frame_id,
            "cameraId": self._camera_id,
            "sensorModel": self._sensor_model,
            "identityConfidence": self._identity_confidence,
            "captureProfile": record.capture_profile,
            "autofocus": record.frame.autofocus.as_dict(),
            "focusQuality": record.frame.focus_quality.as_dict(),
            "capturedAt": captured_at,
            "ageSeconds": self._age_seconds(record.frame),
            "dimensions": {
                "width": record.frame.width,
                "height": record.frame.height,
            },
            "byteCount": len(record.frame.data),
            "sha256": record.frame.sha256,
            "contentSha256": record.content_sha256,
            "stateRevision": record.state_revision,
            "frameUrl": f"/api/camera/frames/{record.frame_id}",
        }
        if self._camera_profile is not None:
            metadata["cameraProfile"] = self._camera_profile
        return metadata


def create_camera_router(service: CameraEvidenceService) -> APIRouter:
    router = APIRouter(prefix="/api/camera", tags=["pi-camera"])

    @router.get("/status")
    def camera_status() -> dict[str, object]:
        return service.status()

    @router.post("/captures")
    def capture(
        request: dict[str, object] | None = Body(default=None),
    ) -> dict[str, object]:
        if request is None:
            capture_profile = CAPTURE_PROFILE_DETAIL
        elif set(request) == {"captureProfile"}:
            try:
                capture_profile = normalize_capture_profile(
                    request.get("captureProfile")  # type: ignore[arg-type]
                )
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                    detail="captureProfile must be survey or detail.",
                ) from None
        else:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Camera capture request has invalid fields.",
            )
        try:
            return service.capture(profile=capture_profile)
        except CameraUnavailableError:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Camera is unavailable.",
            ) from None
        except CameraEvidenceCaptureError:
            raise HTTPException(
                status_code=status.HTTP_502_BAD_GATEWAY,
                detail="Camera capture failed.",
            ) from None

    @router.post("/autofocus")
    def autofocus() -> dict[str, object]:
        try:
            return service.autofocus()
        except CameraUnavailableError:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Camera is unavailable.",
            ) from None
        except CameraEvidenceAutofocusError:
            raise HTTPException(
                status_code=status.HTTP_502_BAD_GATEWAY,
                detail="Camera autofocus test failed.",
            ) from None

    @router.get("/observations/latest")
    def latest() -> dict[str, object]:
        observation = service.latest()
        if observation is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="No camera observation is available.",
            )
        return observation

    @router.get("/frames/{frame_id}", response_class=Response)
    def frame(frame_id: str) -> Response:
        record = service.frame(frame_id)
        if record is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Camera frame was not found in bounded history.",
            )
        return Response(
            content=record.frame.data,
            media_type=JPEG_MIME_TYPE,
            headers={
                "Cache-Control": "no-store",
                "ETag": f'"{record.content_sha256}"',
                "X-Frame-Id": record.frame_id,
                "X-Simulated": str(service.simulated).lower(),
                "X-Camera-Source": service.source,
                "X-Read-Only": "true",
                "X-Content-SHA256": record.content_sha256,
            },
        )

    @router.get("/transfers/{token}", response_class=Response)
    def transfer(token: str) -> Response:
        record = service.claim_transfer(token)
        if record is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Camera transfer was not found or is no longer available.",
            )
        return Response(
            content=record.frame.data,
            media_type=JPEG_MIME_TYPE,
            headers={
                "Cache-Control": "no-store",
                "ETag": f'"{record.content_sha256}"',
                "X-Frame-Id": record.frame_id,
                "X-Simulated": str(service.simulated).lower(),
                "X-Camera-Source": service.source,
                "X-Read-Only": "true",
                "X-Content-SHA256": record.content_sha256,
                "X-One-Use-Transfer": "true",
            },
        )

    return router


__all__ = [
    "CameraEvidenceAutofocusError",
    "CameraEvidenceCaptureCancelledError",
    "CameraEvidenceCaptureError",
    "CameraEvidenceRecord",
    "CameraEvidenceService",
    "CameraEvidenceTransferUnavailableError",
    "CameraProvider",
    "CameraUnavailableError",
    "DEFAULT_MAX_FRAMES",
    "DEFAULT_MAX_TOTAL_BYTES",
    "DEFAULT_MAX_TRANSFERS",
    "DEFAULT_MAX_TRANSFER_BYTES",
    "DEFAULT_TRANSFER_TTL_SECONDS",
    "IDENTITY_CONFIDENCE",
    "MAX_TOTAL_BYTES",
    "create_camera_router",
]
