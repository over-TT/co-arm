"""Gateway-side adapters for the persistent Arm/Isaac bridge.

This module is imported only by a simulator gateway entrypoint.  Production Pi
runtime wiring remains unchanged.  The adapter translates the controller's raw
servo contract to logical joint degrees, while the bridge itself exposes only
bounded physical-equivalent state, targets, and JPEG evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import secrets
from threading import RLock
import time
from typing import Mapping

from arm_sim.camera_profile import CameraProfileError, load_camera_profile
from arm_sim.bridge.client import (
    BridgeClient,
    BridgeClientError,
    BridgeProtocolError,
    BridgeRemoteError,
)
from arm_sim.bridge.protocol import JOINT_IDS, decoded_capture_bytes

from .arm_controller import (
    ControllerCommandError,
    ControllerProtocolError,
    ControllerTransportError,
    MoveSetTuple,
)
from .camera_api import CameraEvidenceService
from .pi_camera import (
    AUTOFOCUS_CAPABILITY_UNSUPPORTED,
    AUTOFOCUS_MODE_UNKNOWN,
    AUTOFOCUS_RESULT_UNSUPPORTED,
    AUTOFOCUS_STATE_NOT_STARTED,
    AutofocusAttempt,
    AutofocusStatus,
    CameraCaptureError,
    CameraLifecycleError,
    CapturedFrame,
    JPEG_MIME_TYPE,
)


_SIM_CAPABILITIES = (
    "scan",
    "set_position_mode",
    "capture",
    "hold_set",
    "torque_lease",
    "telemetry",
    "stop",
    "multi_turn_sense",
    "direct_move",
    "servo_family",
    "multi_turn_absolute_v1",
    "move_set_v1",
)
_SIM_CAMERA_PROFILE = load_camera_profile()
_SIM_AUTOFOCUS = AutofocusStatus(
    capability=AUTOFOCUS_CAPABILITY_UNSUPPORTED,
    mode=AUTOFOCUS_MODE_UNKNOWN,
    state=AUTOFOCUS_STATE_NOT_STARTED,
)


@dataclass(frozen=True, slots=True)
class ServoJointMapping:
    """Exact raw-servo conversion shared with the simulator JointStore.

    ``raw_zero``, direction, ratio, and encoder resolution must match the
    separate simulator calibration state supplied to ``JointStore``.  Defaults
    are useful for contract tests, not physical calibration claims.
    """

    servo_id: int
    joint_id: str
    raw_zero: int
    ticks_per_turn: int
    ratio: float = 1.0
    direction: int = 1
    raw_min: int = 0
    raw_max: int = 4095
    multi_turn: bool = False
    family: str = "STS"

    def __post_init__(self) -> None:
        if isinstance(self.servo_id, bool) or not isinstance(self.servo_id, int):
            raise ValueError("servo_id must be an integer")
        if not 0 <= self.servo_id <= 253 or self.joint_id not in JOINT_IDS:
            raise ValueError("invalid servo to joint mapping")
        if self.ticks_per_turn not in {1024, 4096}:
            raise ValueError("ticks_per_turn must be 1024 or 4096")
        if (
            isinstance(self.ratio, bool)
            or not isinstance(self.ratio, (int, float))
            or not math.isfinite(float(self.ratio))
            or float(self.ratio) <= 0
        ):
            raise ValueError("ratio must be a positive finite number")
        if self.direction not in {-1, 1}:
            raise ValueError("direction must be -1 or 1")
        if self.family not in {"STS", "SCS"}:
            raise ValueError("family must be STS or SCS")
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in (self.raw_zero, self.raw_min, self.raw_max)
        ):
            raise ValueError("raw mapping values must be integers")
        if self.raw_min > self.raw_max or not self.raw_min <= self.raw_zero <= self.raw_max:
            raise ValueError("raw range must contain raw_zero")

    @property
    def ticks_per_degree(self) -> float:
        return float(self.ticks_per_turn) * float(self.ratio) / 360.0

    def degrees_from_raw(self, raw: int) -> float:
        return (raw - self.raw_zero) * self.direction / self.ticks_per_degree

    def raw_from_degrees(self, degrees: float) -> int:
        return round(
            self.raw_zero + degrees * self.ticks_per_degree * self.direction
        )

    def validate_raw(self, raw: object) -> int:
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ValueError("raw goal must be an integer")
        if not self.raw_min <= raw <= self.raw_max:
            raise ValueError("raw goal is outside mapped simulator range")
        return raw


def default_servo_mappings() -> tuple[ServoJointMapping, ...]:
    """Return a provisional, internally consistent simulator-only mapping."""

    return (
        ServoJointMapping(
            1,
            "joint_1",
            raw_zero=0,
            ticks_per_turn=4096,
            ratio=4.0,
            raw_min=-8192,
            raw_max=8192,
            multi_turn=True,
        ),
        ServoJointMapping(
            2,
            "joint_2",
            raw_zero=2048,
            ticks_per_turn=4096,
            raw_min=1024,
            raw_max=3072,
        ),
        ServoJointMapping(
            3,
            "joint_3",
            raw_zero=2048,
            ticks_per_turn=4096,
            raw_min=1024,
            raw_max=3072,
        ),
        ServoJointMapping(
            4,
            "joint_4",
            raw_zero=512,
            ticks_per_turn=1024,
            raw_min=256,
            raw_max=768,
            family="SCS",
        ),
    )


class IsaacArmController:
    """ArmController-compatible facade over logical simulator commands."""

    def __init__(
        self,
        client: BridgeClient,
        *,
        mappings: tuple[ServoJointMapping, ...] | None = None,
        command_duration_ms: int = 0,
        monotonic_clock=time.monotonic,
    ) -> None:
        configured = mappings or default_servo_mappings()
        if len(configured) != len(JOINT_IDS):
            raise ValueError("exactly four servo mappings are required")
        by_servo = {mapping.servo_id: mapping for mapping in configured}
        by_joint = {mapping.joint_id: mapping for mapping in configured}
        if len(by_servo) != 4 or set(by_joint) != set(JOINT_IDS):
            raise ValueError("servo mappings must be one-to-one for all four joints")
        if (
            isinstance(command_duration_ms, bool)
            or not isinstance(command_duration_ms, int)
            or not 0 <= command_duration_ms <= 120_000
        ):
            raise ValueError("command_duration_ms is outside its allowed range")
        self._client = client
        self._by_servo = by_servo
        self._by_joint = by_joint
        self._command_duration_ms = command_duration_ms
        self._monotonic = monotonic_clock
        self._lock = RLock()
        self._started = False
        self._closed = False
        self._stopped = False
        self._leases: dict[int, float] = {}
        self._hold_ids: set[int] = set()
        self._families = {
            mapping.servo_id: mapping.family for mapping in configured
        }
        self._native_multi_turn_ids = {
            mapping.servo_id for mapping in configured if mapping.multi_turn
        }
        self._last_state: dict[str, object] | None = None

    def start(self) -> dict[str, object]:
        with self._lock:
            if self._closed:
                raise ControllerTransportError("simulator controller is closed")
            try:
                self._client.health()
                reply = self._client.get_state()
            except BridgeClientError:
                raise ControllerTransportError("simulator bridge is unavailable") from None
            self._started = True
            self._last_state = dict(reply.result)
            return self._render_transport(reply.result, reply.backend_instance_id)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._started = False
            self._leases.clear()
            self._hold_ids.clear()

    def reconnect(self) -> dict[str, object]:
        with self._lock:
            if self._closed:
                raise ControllerTransportError("simulator controller is closed")
            self._client.reconnect()
            self._started = False
            self._stopped = False
        return self.start()

    def transport_state(self) -> dict[str, object]:
        try:
            return self.status()
        except (ControllerTransportError, ControllerProtocolError):
            return self._offline_transport()

    def status(self) -> dict[str, object]:
        with self._lock:
            self._require_started()
            self._expire_leases()
            try:
                reply = self._client.get_state()
            except (BridgeProtocolError, BridgeRemoteError):
                raise ControllerProtocolError("simulator state is invalid") from None
            except BridgeClientError:
                raise ControllerTransportError("simulator bridge is unavailable") from None
            self._last_state = dict(reply.result)
            return self._render_transport(reply.result, reply.backend_instance_id)

    def scan(self, minimum_id: int, maximum_id: int) -> dict[str, object]:
        if minimum_id > maximum_id:
            raise ValueError("scan range minimum must not exceed maximum")
        self.status()
        found = sorted(
            servo_id
            for servo_id in self._by_servo
            if minimum_id <= servo_id <= maximum_id
        )
        return {
            "foundIds": found,
            "completeRange": {"minId": minimum_id, "maxId": maximum_id},
            "collisionSuspected": False,
            **self._backend_fields(),
        }

    def assign_id(self, old_id: int, new_id: int) -> dict[str, object]:
        del old_id, new_id
        raise ControllerCommandError("UNSUPPORTED_IN_SIM")

    def declare_family(self, servo_id: int, family: str) -> None:
        mapping = self._mapping(servo_id)
        if family not in {"STS", "SCS"} or family != mapping.family:
            raise ControllerCommandError("SERVO_FAMILY_MISMATCH")
        with self._lock:
            self._families[servo_id] = family

    def declare_native_multi_turn_servos(self, servo_ids: list[int]) -> None:
        """Accept only the simulator mapping's complete multi-turn ID set.

        The physical serial adapter uses this host declaration to gate a
        narrowly scoped old-HAT Scan recovery. Isaac needs no such recovery,
        but it implements the same controller contract without silently
        accepting a Pi mapping that disagrees with the simulated arm.
        """

        if (
            not isinstance(servo_ids, list)
            or len(set(servo_ids)) != len(servo_ids)
            or any(
                isinstance(servo_id, bool)
                or not isinstance(servo_id, int)
                or not 0 <= servo_id <= 253
                for servo_id in servo_ids
            )
        ):
            raise ValueError("native multi-turn servo ids must be unique ids 0..253")
        declared = set(servo_ids)
        expected = {
            mapping.servo_id
            for mapping in self._by_servo.values()
            if mapping.multi_turn
        }
        if declared != expected:
            raise ControllerCommandError("NATIVE_MULTI_TURN_ID_MISMATCH")
        with self._lock:
            self._native_multi_turn_ids = declared

    def set_position_mode(self, servo_id: int) -> dict[str, object]:
        mapping = self._mapping(servo_id)
        self.status()
        self.torque_off(servo_id)
        return {
            "servoId": servo_id,
            "previousOperatingMode": 0,
            "operatingMode": 0,
            "verified": True,
            "locked": True,
            "torqueState": "off",
            "minimumPosition": mapping.raw_min,
            "maximumPosition": mapping.raw_max,
            **self._backend_fields(),
        }

    def capture(self, servo_id: int) -> dict[str, object]:
        mapping = self._mapping(servo_id)
        reported = self.status()
        telemetry = self._telemetry_by_id(reported, servo_id)
        raw = self._full_raw(mapping, reported)
        return {
            **telemetry,
            "sampleCount": 1,
            "odometerValid": mapping.multi_turn,
            "revolutions": raw // mapping.ticks_per_turn if mapping.multi_turn else 0,
            "multiTurnPosition": raw,
            "evidenceId": f"simobs_{secrets.token_urlsafe(12)}",
            **self._backend_fields(),
        }

    def read_servo_registers(
        self, servo_id: int, address: int, length: int
    ) -> dict[str, object]:
        del servo_id, address, length
        raise ControllerCommandError("REGISTER_ACCESS_DISABLED_IN_SIM")

    def write_servo_registers(
        self, servo_id: int, address: int, values: list[int]
    ) -> dict[str, object]:
        del servo_id, address, values
        raise ControllerCommandError("REGISTER_ACCESS_DISABLED_IN_SIM")

    def move(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        return self._move_set([(servo_id, goal, speed, acceleration)], grouped=False)

    def move_multi_turn(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        mapping = self._mapping(servo_id)
        if not mapping.multi_turn:
            raise ControllerCommandError("MULTI_TURN_UNAVAILABLE")
        return self._move_set([(servo_id, goal, speed, acceleration)], grouped=False)

    def move_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        return self._move_set(moves, grouped=True)

    def follow_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        """Keep the REAL-only Fast Follow protocol explicit in simulation.

        Isaac intentionally does not advertise either live-follow capability.
        Implementing the controller interface with a fail-closed response keeps
        ordinary simulator motion structurally compatible without inventing
        physical-servo latency or feedback evidence.
        """

        del moves
        raise ControllerCommandError("UNSUPPORTED")

    def follow_read(self, servo_ids: list[int]) -> dict[str, object]:
        """Reject physical-servo feedback sampling in the simulator."""

        del servo_ids
        raise ControllerCommandError("UNSUPPORTED")

    def _move_set(
        self, moves: list[MoveSetTuple], *, grouped: bool
    ) -> dict[str, object]:
        normalized = self._normalize_moves(moves)
        with self._lock:
            self._require_started()
            self._expire_leases()
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            targets: dict[str, float] = {}
            rendered: list[dict[str, object]] = []
            for servo_id, goal, speed, acceleration in normalized:
                if servo_id not in self._leases:
                    raise ControllerCommandError("NO_TORQUE_LEASE")
                mapping = self._mapping(servo_id)
                try:
                    mapping.validate_raw(goal)
                except ValueError:
                    raise ControllerCommandError("OUT_OF_RANGE") from None
                targets[mapping.joint_id] = mapping.degrees_from_raw(goal)
                rendered.append(
                    {
                        "servoId": servo_id,
                        "goal": goal,
                        "speed": speed,
                        "acceleration": acceleration,
                    }
                )
            try:
                reply = self._client.set_joint_targets(
                    targets=targets,
                    duration_ms=self._command_duration_ms,
                )
            except BridgeRemoteError:
                raise ControllerCommandError("SIM_COMMAND_FAILED") from None
            except BridgeProtocolError:
                raise ControllerProtocolError("simulator motion result is invalid") from None
            except BridgeClientError:
                raise ControllerTransportError("simulator bridge is unavailable") from None
            self._last_state = dict(reply.result)
            if not grouped:
                return {**rendered[0], **reply.backend}
            return {
                "controllerId": "isaac-sim-controller",
                "bootId": reply.backend_instance_id,
                "firmwareVersion": "arm-sim-bridge-v1",
                "protocolVersion": 1,
                "dispatch": "simulator_joint_target_group",
                "crossFamilyAtomic": True,
                "count": len(rendered),
                "moved": rendered,
                **reply.backend,
            }

    def set_multi_turn(self, servo_id: int, enabled: bool) -> dict[str, object]:
        mapping = self._mapping(servo_id)
        if not mapping.multi_turn or not isinstance(enabled, bool):
            raise ControllerCommandError("MULTI_TURN_UNAVAILABLE")
        self.torque_off(servo_id)
        return {
            "servoId": servo_id,
            "multiTurn": enabled,
            "operatingMode": 0,
            "angleMin": 0,
            "angleMax": 0 if enabled else mapping.ticks_per_turn - 1,
            **self._backend_fields(),
        }

    def odometer_zero(self, servo_id: int) -> dict[str, object]:
        return self.odometer_read(servo_id)

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        mapping = self._mapping(servo_id)
        if not mapping.multi_turn:
            raise ControllerCommandError("MULTI_TURN_UNAVAILABLE")
        reported = self.status()
        raw = self._full_raw(mapping, reported)
        return {
            "servoId": servo_id,
            "tracking": True,
            "valid": True,
            "stepMode": False,
            "revolutions": raw // mapping.ticks_per_turn,
            "rawPosition": raw % mapping.ticks_per_turn,
            "multiTurnPosition": raw,
            "sampleAgeMs": 0,
            "stepOutstanding": False,
            "countdownObserved": False,
            "resyncNeeded": False,
            "resyncCount": 0,
            **self._backend_fields(),
        }

    def home_multi_turn(self, servo_id: int) -> dict[str, object]:
        return self.odometer_read(servo_id)

    def torque_lease(self, servo_id: int, lease_ms: int) -> dict[str, object]:
        self._mapping(servo_id)
        if isinstance(lease_ms, bool) or not isinstance(lease_ms, int) or not 1 <= lease_ms <= 2_000:
            raise ValueError("lease_ms must be between 1 and 2000")
        with self._lock:
            self._require_started()
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            self._leases[servo_id] = self._monotonic() + lease_ms / 1000.0
        return {
            "servoId": servo_id,
            "leaseMs": lease_ms,
            "torqueState": "on",
            **self._backend_fields(),
        }

    def set_hold_servos(
        self, servo_ids: list[int], lease_ms: int = 1_500
    ) -> dict[str, object]:
        if not isinstance(servo_ids, list) or len(servo_ids) > 4 or len(set(servo_ids)) != len(servo_ids):
            raise ValueError("servo_ids must be a unique list of up to four ids")
        for servo_id in servo_ids:
            self._mapping(servo_id)
        if isinstance(lease_ms, bool) or not isinstance(lease_ms, int) or not 1 <= lease_ms <= 2_000:
            raise ValueError("lease_ms must be between 1 and 2000")
        with self._lock:
            self._require_started()
            if self._stopped and servo_ids:
                raise ControllerCommandError("STOPPED")
            self._hold_ids = set(servo_ids)
            expiry = self._monotonic() + lease_ms / 1000.0
            for servo_id in list(self._leases):
                if servo_id not in self._hold_ids:
                    self._leases.pop(servo_id, None)
            for servo_id in servo_ids:
                self._leases[servo_id] = expiry
        return {
            "servoIds": list(servo_ids),
            "leaseMs": lease_ms,
            "torqueState": "on" if servo_ids else "off",
            **self._backend_fields(),
        }

    def torque_off(self, servo_id: int | None = None) -> dict[str, object]:
        with self._lock:
            if servo_id is None:
                self._leases.clear()
                self._hold_ids.clear()
            else:
                self._mapping(servo_id)
                self._leases.pop(servo_id, None)
                self._hold_ids.discard(servo_id)
        return {
            "servoId": servo_id,
            "torqueState": "off",
            **self._backend_fields(),
        }

    def prepare_nudge(
        self,
        servo_id: int,
        delta_ticks: int,
        speed: int,
        acceleration: int,
    ) -> dict[str, object]:
        del servo_id, delta_ticks, speed, acceleration
        raise ControllerCommandError("UNSUPPORTED_IN_SIM")

    def execute_nudge(
        self, proposal_id: str, proposal_hash: str | None = None
    ) -> dict[str, object]:
        del proposal_id, proposal_hash
        raise ControllerCommandError("UNSUPPORTED_IN_SIM")

    def stop(self) -> dict[str, object]:
        with self._lock:
            self._stopped = True
            self._leases.clear()
            self._hold_ids.clear()
        return self.transport_state()

    def reset(self, *, inspected: bool = False) -> dict[str, object]:
        del inspected
        with self._lock:
            self._require_started()
            try:
                reply = self._client.reset()
            except BridgeRemoteError:
                raise ControllerCommandError("SIM_RESET_FAILED") from None
            except BridgeProtocolError:
                raise ControllerProtocolError("simulator reset result is invalid") from None
            except BridgeClientError:
                raise ControllerTransportError("simulator bridge is unavailable") from None
            self._stopped = False
            self._leases.clear()
            self._hold_ids.clear()
            self._last_state = dict(reply.result)
            return self._render_transport(reply.result, reply.backend_instance_id)

    @staticmethod
    def _normalize_moves(moves: object) -> list[MoveSetTuple]:
        if not isinstance(moves, list) or not 1 <= len(moves) <= 4:
            raise ValueError("moves must contain between 1 and 4 tuples")
        normalized: list[MoveSetTuple] = []
        seen: set[int] = set()
        for move in moves:
            if not isinstance(move, (tuple, list)) or len(move) != 4:
                raise ValueError("each move must contain four integers")
            if any(isinstance(value, bool) or not isinstance(value, int) for value in move):
                raise ValueError("move fields must be integers")
            servo_id, goal, speed, acceleration = move
            if servo_id in seen or not 0 <= servo_id <= 253:
                raise ValueError("move contains an invalid or duplicate servo id")
            if not -30_719 <= goal <= 30_719 or not 1 <= speed <= 4_095 or not 1 <= acceleration <= 255:
                raise ValueError("move fields are outside their allowed range")
            seen.add(servo_id)
            normalized.append((servo_id, goal, speed, acceleration))
        return normalized

    def _mapping(self, servo_id: object) -> ServoJointMapping:
        if isinstance(servo_id, bool) or not isinstance(servo_id, int):
            raise ValueError("servo_id must be an integer")
        mapping = self._by_servo.get(servo_id)
        if mapping is None:
            raise ControllerCommandError("SERVO_NOT_FOUND")
        return mapping

    def _require_started(self) -> None:
        if self._closed or not self._started:
            raise ControllerTransportError("simulator controller is unavailable")

    def _expire_leases(self) -> None:
        now = self._monotonic()
        expired = [servo_id for servo_id, expiry in self._leases.items() if now >= expiry]
        for servo_id in expired:
            self._leases.pop(servo_id, None)
            self._hold_ids.discard(servo_id)

    def _backend_fields(self) -> dict[str, object]:
        return {
            "backendId": "sim",
            "backendInstanceId": self._client.backend_instance_id,
            "simulated": True,
        }

    def _offline_transport(self) -> dict[str, object]:
        return {
            "configured": True,
            "connection": "offline",
            "bus": "unknown",
            "servos": "unknown",
            "motionState": "blocked",
            "torqueState": "unknown",
            "identity": None,
            "servoCount": None,
            "heartbeatAgeMs": None,
            **self._backend_fields(),
        }

    def _render_transport(
        self, state: Mapping[str, object], instance_id: str
    ) -> dict[str, object]:
        positions = state.get("jointPositionsDegrees")
        if not isinstance(positions, Mapping):
            raise ControllerProtocolError("simulator positions are invalid")
        self._expire_leases()
        telemetry: list[dict[str, object]] = []
        for joint_id in JOINT_IDS:
            value = positions.get(joint_id)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ControllerProtocolError("simulator positions are invalid")
            mapping = self._by_joint[joint_id]
            full_raw = mapping.raw_from_degrees(float(value))
            raw = full_raw % mapping.ticks_per_turn if mapping.multi_turn else full_raw
            telemetry.append(
                {
                    "id": mapping.servo_id,
                    "rawPosition": raw,
                    "speed": 0,
                    "load": 0,
                    "voltageVolts": 12.0,
                    "temperatureC": 25,
                    "moving": bool(state.get("moving")),
                    "currentMilliamps": 0,
                    "operatingMode": 0,
                    "torqueState": "on" if mapping.servo_id in self._leases else "off",
                    "packetAgeMs": 0,
                    "errors": [],
                    "online": True,
                    "fresh": True,
                    "statusError": 0,
                }
            )
        torque_state = "on" if self._leases else "off"
        return {
            "configured": True,
            "connection": "online",
            "bus": "online",
            "servos": "online",
            "motionState": (
                "stopped"
                if self._stopped or state.get("stopped") is True
                else "moving"
                if state.get("moving") is True
                else "ready"
            ),
            "torqueState": torque_state,
            "identity": {
                "controllerId": "isaac-sim-controller",
                "bootId": instance_id,
                "firmwareVersion": "arm-sim-bridge-v1",
                "protocolVersion": 1,
                "capabilities": list(_SIM_CAPABILITIES),
            },
            "servoCount": 4,
            "heartbeatAgeMs": 0,
            "busBaud": 1_000_000,
            "lastScan": None,
            "servosTelemetry": telemetry,
            "hardwareEstop": "not_detected",
            "stateRevision": state.get("stateRevision"),
            "backendId": "sim",
            "backendInstanceId": instance_id,
            "simulated": True,
        }

    def _full_raw(
        self, mapping: ServoJointMapping, reported: Mapping[str, object]
    ) -> int:
        telemetry = self._telemetry_by_id(reported, mapping.servo_id)
        raw = telemetry.get("rawPosition")
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ControllerProtocolError("simulator telemetry is invalid")
        if not mapping.multi_turn:
            return raw
        positions = self._last_state.get("jointPositionsDegrees") if self._last_state else None
        if not isinstance(positions, Mapping):
            raise ControllerProtocolError("simulator state is invalid")
        degrees = positions.get(mapping.joint_id)
        if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
            raise ControllerProtocolError("simulator state is invalid")
        return mapping.raw_from_degrees(float(degrees))

    @staticmethod
    def _telemetry_by_id(
        reported: Mapping[str, object], servo_id: int
    ) -> dict[str, object]:
        telemetry = reported.get("servosTelemetry")
        if not isinstance(telemetry, list):
            raise ControllerProtocolError("simulator telemetry is missing")
        for candidate in telemetry:
            if isinstance(candidate, dict) and candidate.get("id") == servo_id:
                return dict(candidate)
        raise ControllerCommandError("SERVO_NOT_FOUND")


class IsaacCameraProvider:
    """CameraProvider-compatible facade over rendered bridge JPEGs."""

    camera_id = _SIM_CAMERA_PROFILE.camera_id
    sensor_model = _SIM_CAMERA_PROFILE.sensor_model
    identity_confidence = _SIM_CAMERA_PROFILE.identity_confidence
    camera_profile = _SIM_CAMERA_PROFILE.report_metadata()

    def __init__(
        self,
        client: BridgeClient,
        *,
        max_frame_bytes: int = 16 * 1024 * 1024,
        monotonic_clock=time.monotonic,
    ) -> None:
        if (
            isinstance(max_frame_bytes, bool)
            or not isinstance(max_frame_bytes, int)
            or not 1 <= max_frame_bytes <= 16 * 1024 * 1024
        ):
            raise ValueError("max_frame_bytes is outside its allowed range")
        self._client = client
        self._max_frame_bytes = max_frame_bytes
        self._monotonic = monotonic_clock
        self._lock = RLock()
        self._started = False
        self._closed = False
        self._last_state_revision = 0

    @property
    def max_frame_bytes(self) -> int:
        return self._max_frame_bytes

    @property
    def autofocus_status(self) -> AutofocusStatus:
        return _SIM_AUTOFOCUS

    @property
    def camera_profile_metadata(self) -> dict[str, object]:
        """Profile metadata discovered by the shared camera evidence service."""

        reference = _SIM_CAMERA_PROFILE.document["referenceHardware"]
        assert isinstance(reference, Mapping)
        native = reference["sensorResolutionPx"]
        nominal_fov = reference["fieldOfViewDeg"]
        assert isinstance(native, Mapping)
        assert isinstance(nominal_fov, Mapping)
        return {
            "id": _SIM_CAMERA_PROFILE.profile_id,
            "productName": f'{reference["manufacturer"]} {reference["module"]}',
            "sensorModel": _SIM_CAMERA_PROFILE.reference_sensor_model,
            "lensVariant": str(reference["variant"]),
            "nativeDimensions": {
                "width": int(native["width"]),
                "height": int(native["height"]),
            },
            "nominalFocalLengthMm": float(reference["focalLengthMm"]),
            "nominalFieldOfViewDegrees": {
                "horizontal": float(nominal_fov["horizontal"]),
                "vertical": float(nominal_fov["vertical"]),
            },
            "captureProfiles": {
                "survey": {
                    "width": _SIM_CAMERA_PROFILE.survey.width_px,
                    "height": _SIM_CAMERA_PROFILE.survey.height_px,
                },
                "detail": {
                    "width": _SIM_CAMERA_PROFILE.detail.width_px,
                    "height": _SIM_CAMERA_PROFILE.detail.height_px,
                },
            },
        }

    @property
    def backend_identity(self) -> dict[str, object]:
        return {
            "backendId": "sim",
            "backendInstanceId": self._client.backend_instance_id,
            "simulated": True,
            "cameraProfile": self.camera_profile_metadata,
        }

    @property
    def last_state_revision(self) -> int:
        with self._lock:
            return self._last_state_revision

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise CameraLifecycleError("Simulator camera is closed.")
            try:
                self._client.health()
            except BridgeClientError:
                raise CameraLifecycleError("Simulator camera is unavailable.") from None
            self._started = True

    def capture(self, profile: str = "detail") -> CapturedFrame:
        with self._lock:
            if self._closed or not self._started:
                raise CameraLifecycleError("Simulator camera is unavailable.")
            try:
                render_profile = _SIM_CAMERA_PROFILE.render(profile)
                reply = self._client.capture(profile=profile)
                result = reply.result
                data = decoded_capture_bytes(result)
                captured_at = datetime.fromisoformat(
                    str(result["capturedAt"]).replace("Z", "+00:00")
                ).astimezone(timezone.utc)
            except (
                BridgeClientError,
                CameraProfileError,
                ValueError,
                KeyError,
                TypeError,
            ):
                raise CameraCaptureError("Simulator camera capture failed.") from None
            if len(data) > self._max_frame_bytes:
                raise CameraCaptureError("Simulator camera capture failed.")
            if (
                result.get("width") != render_profile.width_px
                or result.get("height") != render_profile.height_px
            ):
                raise CameraCaptureError("Simulator camera capture failed.")
            digest = result["sha256"]
            state_revision = result["stateRevision"]
            assert isinstance(digest, str)
            assert isinstance(state_revision, int) and not isinstance(state_revision, bool)
            self._last_state_revision = state_revision
            return CapturedFrame(
                data=data,
                mime_type=JPEG_MIME_TYPE,
                width=int(result["width"]),
                height=int(result["height"]),
                captured_at_utc=captured_at,
                captured_monotonic=float(self._monotonic()),
                sha256=digest.removeprefix("sha256:"),
                camera_id=self.camera_id,
                sensor_model=self.sensor_model,
                _monotonic_clock=self._monotonic,
                autofocus=_SIM_AUTOFOCUS,
            )

    def autofocus(self) -> AutofocusAttempt:
        with self._lock:
            if self._closed or not self._started:
                raise CameraLifecycleError("Simulator camera is unavailable.")
            return AutofocusAttempt(
                attempted=False,
                result=AUTOFOCUS_RESULT_UNSUPPORTED,
                autofocus=_SIM_AUTOFOCUS,
            )

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._started = False


def create_isaac_camera_service(
    provider: IsaacCameraProvider,
) -> CameraEvidenceService:
    """Bind evidence revision and simulated metadata to the rendered frame.

    Pass this service through ``create_app(camera_service=...)`` rather than
    passing the provider directly. That binds simulator captures to the exact
    Isaac bridge revision instead of Raspberry Pi telemetry.
    """

    if not isinstance(provider, IsaacCameraProvider):
        raise TypeError("provider must be an IsaacCameraProvider")
    return CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": provider.last_state_revision},
        simulated=True,
        source="isaac_rgb",
        identity_confidence=provider.identity_confidence,
    )


__all__ = [
    "IsaacArmController",
    "IsaacCameraProvider",
    "ServoJointMapping",
    "create_isaac_camera_service",
    "default_servo_mappings",
]
