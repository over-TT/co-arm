"""Simulator-engine contract owned by the persistent Isaac process."""

from __future__ import annotations

from datetime import datetime, timezone
import secrets
from threading import RLock
from typing import Mapping, Protocol, runtime_checkable

from arm_sim.camera_profile import load_camera_profile

from .protocol import JOINT_IDS, capture_result


_CAMERA_PROFILE = load_camera_profile()


def _utc_timestamp() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


@runtime_checkable
class BridgeEngine(Protocol):
    """Five operations an Isaac-side engine must implement.

    Implementations own all USD/PhysX/camera details and must return only the
    physical-equivalent values defined in ``protocol.py``.
    """

    def health(self) -> dict[str, object]: ...

    def reset(self, *, seed: int | None) -> dict[str, object]: ...

    def get_state(self) -> dict[str, object]: ...

    def set_joint_targets(
        self, *, targets: Mapping[str, float], duration_ms: int
    ) -> dict[str, object]: ...

    def capture(self, profile: str = "detail") -> dict[str, object]: ...


# A tiny deterministic JPEG-like payload for IPC and adapter contract tests.
# The camera evidence layer intentionally validates digest, type, dimensions,
# and provenance rather than decoding pixels.  Real Isaac engines must return
# their rendered JPEG bytes instead.
_DEV_JPEG = b"\xff\xd8\xff\xe0ARM_SIM_CONTRACT_FRAME\xff\xd9"


class InMemoryBridgeEngine:
    """Isaac-free development engine; never presented as calibrated physics."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._revision = 0
        self._positions = {joint_id: 0.0 for joint_id in JOINT_IDS}
        self._stopped = False

    def health(self) -> dict[str, object]:
        return {
            "status": "ok",
            "engine": "in_memory_contract_engine",
            "calibrationStatus": "provisional",
        }

    def reset(self, *, seed: int | None) -> dict[str, object]:
        del seed
        with self._lock:
            self._revision += 1
            self._positions = {joint_id: 0.0 for joint_id in JOINT_IDS}
            self._stopped = False
            return self._state_locked()

    def get_state(self) -> dict[str, object]:
        with self._lock:
            return self._state_locked()

    def set_joint_targets(
        self, *, targets: Mapping[str, float], duration_ms: int
    ) -> dict[str, object]:
        del duration_ms
        with self._lock:
            if self._stopped:
                raise RuntimeError("simulator is stopped")
            self._positions.update({key: float(value) for key, value in targets.items()})
            self._revision += 1
            return self._state_locked()

    def capture(self, profile: str = "detail") -> dict[str, object]:
        render_profile = _CAMERA_PROFILE.render(profile)
        with self._lock:
            timestamp = _utc_timestamp()
            return capture_result(
                frame_id=f"simframe_{secrets.token_urlsafe(12)}",
                data=_DEV_JPEG,
                width=render_profile.width_px,
                height=render_profile.height_px,
                captured_at=timestamp,
                state_revision=self._revision,
                joint_positions_degrees=self._positions,
            )

    def _state_locked(self) -> dict[str, object]:
        return {
            "stateRevision": self._revision,
            "jointPositionsDegrees": dict(self._positions),
            "moving": False,
            "stopped": self._stopped,
            "capturedAt": _utc_timestamp(),
        }


__all__ = ["BridgeEngine", "InMemoryBridgeEngine"]
