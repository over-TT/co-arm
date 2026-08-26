"""Fail-closed controller boundary for physical arm commissioning.

The public gateway never exposes servo registers or arbitrary position goals.
Every implementation behind this protocol is limited to discovery, evidence,
short torque leases, bounded two-phase nudges, and safe-stop operations.
"""

from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import json
import secrets
import threading
import time
from typing import Literal, Protocol, runtime_checkable


TorqueState = Literal["off", "on", "unknown"]

# Mirrors the firmware's HELLO capability list. A fake that under-advertises
# leaves capability gates (like multi-turn arming) silently untested.
REPLAY_CAPABILITIES: tuple[str, ...] = (
    "scan", "single_servo_id", "set_position_mode", "capture", "hold_set",
    "torque_lease", "bounded_nudge", "telemetry", "stop", "multi_turn_sense",
    "register_access", "runtime_config", "direct_move", "servo_family",
    "multi_turn_absolute_v1", "move_set_v1", "follow_set_feedback_v1",
    "follow_feedback_v1",
)


MoveSetTuple = tuple[int, int, int, int]


def _normalize_move_set(moves: object) -> list[MoveSetTuple]:
    """Validate the compact A1 MOVE_SET contract before any side effect."""

    if not isinstance(moves, list) or not 1 <= len(moves) <= 4:
        raise ValueError("moves must contain between 1 and 4 tuples")
    normalized: list[MoveSetTuple] = []
    seen_ids: set[int] = set()
    for candidate in moves:
        if not isinstance(candidate, (tuple, list)) or len(candidate) != 4:
            raise ValueError("each move must be (servo_id, goal, speed, acceleration)")
        servo_id, goal, speed, acceleration = candidate
        fields = (servo_id, goal, speed, acceleration)
        if any(isinstance(value, bool) or not isinstance(value, int) for value in fields):
            raise ValueError("move fields must be integers")
        if not 0 <= servo_id <= 253:
            raise ValueError("servo id must be between 0 and 253")
        if servo_id in seen_ids:
            raise ValueError("move set contains a duplicate servo id")
        if not -30_719 <= goal <= 30_719:
            raise ValueError("goal must be between -30719 and 30719")
        if not 1 <= speed <= 4_095:
            raise ValueError("speed must be between 1 and 4095")
        if not 1 <= acceleration <= 255:
            raise ValueError("acceleration must be between 1 and 255")
        seen_ids.add(servo_id)
        normalized.append((servo_id, goal, speed, acceleration))
    return normalized


def _normalize_follow_set(moves: object) -> list[MoveSetTuple]:
    """Validate the narrow two-member live-follow request before I/O."""

    normalized = _normalize_move_set(moves)
    if len(normalized) != 2:
        raise ValueError("follow set requires exactly two servo tuples")
    if any(speed > 2_400 for _, _, speed, _ in normalized):
        raise ValueError("follow set speed must be between 1 and 2400")
    if any(acceleration > 50 for _, _, _, acceleration in normalized):
        raise ValueError("follow set acceleration must be between 1 and 50")
    return normalized


def _normalize_follow_read(servo_ids: object) -> list[int]:
    """Validate the exact ordered pair used by read-only live feedback."""

    if not isinstance(servo_ids, list) or len(servo_ids) != 2:
        raise ValueError("follow read requires exactly two servo ids")
    if any(
        isinstance(servo_id, bool)
        or not isinstance(servo_id, int)
        or not 0 <= servo_id <= 253
        for servo_id in servo_ids
    ):
        raise ValueError("follow read servo ids must be integers between 0 and 253")
    if servo_ids[0] == servo_ids[1]:
        raise ValueError("follow read requires unique servo ids")
    return list(servo_ids)


class ArmControllerError(RuntimeError):
    """Base class for sanitized physical-controller failures."""


class ControllerUnavailableError(ArmControllerError):
    """The configured controller transport is unavailable."""


class ControllerTransportError(ControllerUnavailableError):
    """A request may not have reached the controller."""


class ControllerProtocolError(ArmControllerError):
    """The controller sent a frame that cannot be trusted."""


class ControllerCommandError(ArmControllerError):
    """The controller rejected a well-formed command."""

    def __init__(self, code: str, payload: dict[str, object] | None = None) -> None:
        super().__init__(code)
        self.code = code
        self.payload = deepcopy(payload or {})


@runtime_checkable
class ArmController(Protocol):
    """Narrow commissioning contract implemented by replay and serial paths."""

    def start(self) -> dict[str, object]: ...

    def close(self) -> None: ...

    def reconnect(self) -> dict[str, object]: ...

    def transport_state(self) -> dict[str, object]: ...

    def status(self) -> dict[str, object]: ...

    def scan(self, minimum_id: int, maximum_id: int) -> dict[str, object]: ...

    def assign_id(self, old_id: int, new_id: int) -> dict[str, object]: ...

    def declare_family(self, servo_id: int, family: str) -> None: ...

    def declare_native_multi_turn_servos(self, servo_ids: list[int]) -> None: ...

    def set_position_mode(self, servo_id: int) -> dict[str, object]: ...

    def capture(self, servo_id: int) -> dict[str, object]: ...

    def read_servo_registers(
        self, servo_id: int, address: int, length: int
    ) -> dict[str, object]: ...

    def write_servo_registers(
        self, servo_id: int, address: int, values: list[int]
    ) -> dict[str, object]: ...

    def move(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]: ...

    def move_multi_turn(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]: ...

    def move_set(self, moves: list[MoveSetTuple]) -> dict[str, object]: ...

    def follow_set(self, moves: list[MoveSetTuple]) -> dict[str, object]: ...

    def follow_read(self, servo_ids: list[int]) -> dict[str, object]: ...

    def set_multi_turn(self, servo_id: int, enabled: bool) -> dict[str, object]: ...

    def odometer_zero(self, servo_id: int) -> dict[str, object]: ...

    def odometer_read(self, servo_id: int) -> dict[str, object]: ...

    def home_multi_turn(self, servo_id: int) -> dict[str, object]: ...

    def torque_lease(self, servo_id: int, lease_ms: int) -> dict[str, object]: ...

    def set_hold_servos(
        self, servo_ids: list[int], lease_ms: int = 1_500
    ) -> dict[str, object]: ...

    def torque_off(self, servo_id: int | None = None) -> dict[str, object]: ...

    def prepare_nudge(
        self,
        servo_id: int,
        delta_ticks: int,
        speed: int,
        acceleration: int,
    ) -> dict[str, object]: ...

    def execute_nudge(
        self, proposal_id: str, proposal_hash: str | None = None
    ) -> dict[str, object]: ...

    def stop(self) -> dict[str, object]: ...

    def reset(self, *, inspected: bool = False) -> dict[str, object]: ...


class UnavailableArmController:
    """Explicit physical-controller placeholder used when no port is selected."""

    def __init__(self, *, configured: bool = False) -> None:
        self._configured = configured

    def start(self) -> dict[str, object]:
        return self.transport_state()

    def close(self) -> None:
        return None

    def reconnect(self) -> dict[str, object]:
        raise ControllerUnavailableError("controller unavailable")

    def transport_state(self) -> dict[str, object]:
        return {
            "configured": self._configured,
            "connection": "offline" if self._configured else "not_configured",
            "bus": "unknown",
            "servos": "unknown",
            "motionState": "blocked",
            "torqueState": "unknown",
            "identity": None,
            "servoCount": None,
            "heartbeatAgeMs": None,
        }

    status = transport_state

    @staticmethod
    def _unavailable(*_: object, **__: object) -> dict[str, object]:
        raise ControllerUnavailableError("controller unavailable")

    scan = _unavailable
    assign_id = _unavailable
    declare_family = _unavailable
    declare_native_multi_turn_servos = _unavailable
    set_position_mode = _unavailable
    capture = _unavailable
    move_set = _unavailable
    follow_set = _unavailable
    follow_read = _unavailable
    home_multi_turn = _unavailable
    torque_lease = _unavailable
    set_hold_servos = _unavailable
    torque_off = _unavailable
    prepare_nudge = _unavailable
    execute_nudge = _unavailable
    stop = _unavailable
    reset = _unavailable


def _proposal_hash(payload: dict[str, object]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return f"sha256:{sha256(canonical).hexdigest()}"


class ReplayArmController:
    """Deterministic no-hardware controller for API and workflow tests."""

    def __init__(
        self,
        *,
        connected: bool,
        servos: list[dict[str, object]] | None = None,
    ) -> None:
        self._connected = connected
        self._faulted = False
        self._servos: dict[int, dict[str, object]] = {}
        self._lock = threading.RLock()
        self._failures: dict[str, list[str]] = {}
        self._proposals: dict[str, dict[str, object]] = {}
        # Raw register bytes a test wrote, so a read-back sees them. The real
        # controller has a servo on the other end; this stands in for it.
        self._registers: dict[int, dict[int, list[int]]] = {}
        self._leases: dict[int, float] = {}
        self._odometers: dict[int, dict[str, int | bool]] = {}
        # Signed Mode-0 coordinate owned by the native multi-turn controller.
        # Servo telemetry remains wrapped, just like real register 56.
        self._native_positions: dict[int, int] = {}
        self._multi_turn_servos: set[int] = set()
        self._declared_native_multi_turn_servos: set[int] = set()
        self._servo_families: dict[int, str] = {}
        self._stopped = False
        self._boot_id = "replay_boot_1"
        self._last_scan: dict[str, object] | None = None
        self.commands: list[dict[str, object]] = []
        for supplied in servos or []:
            normalized = self._normalize_servo(supplied)
            identifier = int(normalized["id"])
            self._servos[identifier] = normalized
            supplied_native = supplied.get("multiTurnPosition")
            self._native_positions[identifier] = (
                int(supplied_native)
                if isinstance(supplied_native, int)
                and not isinstance(supplied_native, bool)
                else int(normalized["rawPosition"])
            )

    @classmethod
    def connected(
        cls, *, servos: list[dict[str, object]] | None = None
    ) -> "ReplayArmController":
        return cls(connected=True, servos=servos)

    def start(self) -> dict[str, object]:
        if not self._connected:
            raise ControllerUnavailableError("controller unavailable")
        self._faulted = False
        return {
            "controllerId": "replay-hat-a",
            "bootId": self._boot_id,
            "firmwareVersion": "replay-1.0.0",
            "protocolVersion": 1,
            "state": "disarmed",
        }

    def close(self) -> None:
        with self._lock:
            self._leases.clear()
            self._proposals.clear()
            for servo in self._servos.values():
                servo["torqueState"] = "off"
                servo["operatingMode"] = 0

    def reconnect(self) -> dict[str, object]:
        with self._lock:
            self.close()
            return self.start()

    def fail_next(self, operation: str, message: str) -> None:
        with self._lock:
            self._failures.setdefault(operation.upper(), []).append(message)

    def simulate_reboot(self) -> None:
        """Rotate replay identity and discard volatile controller authority."""

        with self._lock:
            suffix = int(self._boot_id.rsplit("_", 1)[-1]) + 1
            self._boot_id = f"replay_boot_{suffix}"
            self._faulted = False
            self._leases.clear()
            self._proposals.clear()
            self._odometers.clear()
            self._stopped = False
            for servo in self._servos.values():
                servo["torqueState"] = "off"

    def _before(self, operation: str, **arguments: object) -> None:
        if not self._connected or self._faulted:
            raise ControllerUnavailableError("controller unavailable")
        self.commands.append({"operation": operation, **arguments})
        queued = self._failures.get(operation)
        if queued:
            queued.pop(0)
            self._faulted = True
            for servo in self._servos.values():
                servo["torqueState"] = "unknown"
            raise ControllerTransportError("controller transport failed")

    @staticmethod
    def _normalize_servo(supplied: dict[str, object]) -> dict[str, object]:
        servo = {
            "id": supplied.get("id"),
            "rawPosition": supplied.get("rawPosition", 2048),
            "speed": supplied.get("speed", 0),
            "load": supplied.get("load", 0),
            "voltageVolts": supplied.get("voltageVolts", 12.0),
            "temperatureC": supplied.get("temperatureC", 25),
            "moving": supplied.get("moving", False),
            "currentMilliamps": supplied.get("currentMilliamps", 0),
            "operatingMode": supplied.get("operatingMode", 0),
            "torqueState": supplied.get("torqueState", "off"),
            "packetAgeMs": supplied.get("packetAgeMs", 0),
            "errors": deepcopy(supplied.get("errors", [])),
        }
        # Replay accepts the firmware-only preflight evidence when a test
        # supplies it, but does not expand the long-standing public servo DTO
        # with default-only fields.
        for optional_field in ("online", "fresh", "statusError"):
            if optional_field in supplied:
                servo[optional_field] = supplied[optional_field]
        identifier = servo["id"]
        if isinstance(identifier, bool) or not isinstance(identifier, int):
            raise ValueError("servo id must be an integer")
        if not 0 <= identifier <= 253:
            raise ValueError("servo id must be between 0 and 253")
        return servo

    def _current_native_position(
        self, servo_id: int, servo: dict[str, object]
    ) -> int:
        """Update Replay's signed coordinate from the servo's wrapped sample."""

        current_raw = int(servo.get("rawPosition", 0))
        odometer = self._odometers.get(servo_id)
        if odometer is not None:
            delta = current_raw - int(odometer["lastRaw"])
            if delta > 2048:
                odometer["revolutions"] -= 1
            elif delta < -2048:
                odometer["revolutions"] += 1
            odometer["lastRaw"] = current_raw
            position = int(odometer["revolutions"]) * 4096 + current_raw
        else:
            previous = self._native_positions.get(servo_id, current_raw)
            previous_raw = previous % 4096
            delta = current_raw - previous_raw
            if delta > 2048:
                delta -= 4096
            elif delta < -2048:
                delta += 4096
            position = previous + delta
        self._native_positions[servo_id] = position
        return position

    def _refresh_leases(self) -> None:
        now = time.monotonic()
        expired = [identifier for identifier, expiry in self._leases.items() if now >= expiry]
        for identifier in expired:
            self._leases.pop(identifier, None)
            servo = self._servos.get(identifier)
            if servo is not None:
                servo["torqueState"] = "off"
                servo["operatingMode"] = 0

    def transport_state(self) -> dict[str, object]:
        with self._lock:
            self._refresh_leases()
            if not self._connected:
                return UnavailableArmController(configured=True).transport_state()
            if self._faulted:
                return {
                    "configured": True,
                    "connection": "faulted",
                    "bus": "unknown",
                    "servos": "unknown",
                    "motionState": "blocked",
                    "torqueState": "unknown",
                    "identity": {
                        "controllerId": "replay-hat-a",
                        "bootId": self._boot_id,
                        "firmwareVersion": "replay-1.0.0",
                        "protocolVersion": 1,
                        "capabilities": list(REPLAY_CAPABILITIES),
                    },
                    "servoCount": len(self._servos),
                    "heartbeatAgeMs": None,
                    "busBaud": 1_000_000,
                    "lastScan": deepcopy(self._last_scan),
                    "servosTelemetry": [
                        deepcopy(self._servos[identifier])
                        for identifier in sorted(self._servos)
                    ],
                    "hardwareEstop": "not_detected",
                }
            torque_states = {str(servo["torqueState"]) for servo in self._servos.values()}
            torque_state = (
                "on" if "on" in torque_states else "off" if torque_states else "unknown"
            )
            return {
                "configured": True,
                "connection": "online",
                "bus": "online" if self._servos else "unknown",
                "servos": "online" if self._servos else "none_found",
                # Matches the firmware's own `motionState()`, which distinguishes
                # a latched STOP from a generic block. The fake reporting
                # "blocked" for both hid a whole class of bug from every test.
                "motionState": "stopped" if self._stopped else "ready",
                "torqueState": torque_state,
                "identity": {
                    "controllerId": "replay-hat-a",
                    "bootId": self._boot_id,
                    "firmwareVersion": "replay-1.0.0",
                    "protocolVersion": 1,
                    "capabilities": list(REPLAY_CAPABILITIES),
                },
                "servoCount": len(self._servos),
                "heartbeatAgeMs": 0,
                "busBaud": 1_000_000,
                "lastScan": deepcopy(self._last_scan),
                "servosTelemetry": [
                    deepcopy(self._servos[identifier])
                    for identifier in sorted(self._servos)
                ],
                "hardwareEstop": "not_detected",
            }

    status = transport_state

    def scan(self, minimum_id: int, maximum_id: int) -> dict[str, object]:
        if minimum_id > maximum_id:
            raise ValueError("scan range minimum must not exceed maximum")
        with self._lock:
            self._before("SCAN", minId=minimum_id, maxId=maximum_id)
            self._proposals.clear()
            found = sorted(
                identifier
                for identifier in self._servos
                if minimum_id <= identifier <= maximum_id
            )
            result = {
                "foundIds": found,
                "completeRange": {"minId": minimum_id, "maxId": maximum_id},
                "collisionSuspected": False,
            }
            self._last_scan = {
                "minId": minimum_id,
                "maxId": maximum_id,
                "foundIds": found,
            }
            return result

    def assign_id(self, old_id: int, new_id: int) -> dict[str, object]:
        with self._lock:
            self._before("ASSIGN_ID", oldId=old_id, newId=new_id)
            if old_id not in self._servos:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if new_id in self._servos:
                raise ControllerCommandError("ID_IN_USE")
            servo = self._servos.pop(old_id)
            servo["id"] = new_id
            self._servos[new_id] = servo
            family = self._servo_families.pop(old_id, None)
            if family is not None:
                self._servo_families[new_id] = family
            self._proposals.clear()
            self._last_scan = None
            return {"oldId": old_id, "newId": new_id, "verified": True}

    def declare_family(self, servo_id: int, family: str) -> None:
        if isinstance(servo_id, bool) or not isinstance(servo_id, int):
            raise ValueError("servo id must be an integer")
        if not 0 <= servo_id <= 253:
            raise ValueError("servo id must be between 0 and 253")
        if family not in ("STS", "SCS"):
            raise ValueError("family must be STS or SCS")
        with self._lock:
            self._servo_families[servo_id] = family

    def declare_native_multi_turn_servos(self, servo_ids: list[int]) -> None:
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
        with self._lock:
            self._declared_native_multi_turn_servos = set(servo_ids)

    def set_position_mode(self, servo_id: int) -> dict[str, object]:
        with self._lock:
            self._before("SET_POSITION_MODE", servoId=servo_id)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if len(self._servos) != 1:
                raise ControllerCommandError("NOT_SINGLE_SERVO")
            previous_mode = int(servo.get("operatingMode", 0))
            servo["operatingMode"] = 0
            servo["torqueState"] = "off"
            servo["errors"] = [
                error
                for error in servo.get("errors", [])
                if error != "mode_not_position"
            ]
            self._proposals.clear()
            return {
                "servoId": servo_id,
                "previousOperatingMode": previous_mode,
                "operatingMode": 0,
                "verified": True,
                "locked": True,
                "torqueState": "off",
                "minimumPosition": 0,
                "maximumPosition": 4095,
            }

    def capture(self, servo_id: int) -> dict[str, object]:
        with self._lock:
            self._refresh_leases()
            self._before("CAPTURE", servoId=servo_id)
            self._proposals.clear()
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            odometer = self._odometers.get(servo_id)
            raw = int(servo.get("rawPosition", 0))
            multi_turn = (
                odometer["revolutions"] * 4096 + raw if odometer is not None else None
            )
            return {
                **deepcopy(servo),
                "sampleCount": 7,
                "odometerValid": odometer is not None,
                "revolutions": odometer["revolutions"] if odometer is not None else 0,
                "multiTurnPosition": multi_turn if multi_turn is not None else raw,
                "evidenceId": f"obs_{secrets.token_urlsafe(12)}",
            }

    def read_servo_registers(
        self, servo_id: int, address: int, length: int
    ) -> dict[str, object]:
        with self._lock:
            self._before("REG_READ", servoId=servo_id, address=address, length=length)
            if servo_id not in self._servos:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            return {
                "servoId": servo_id,
                "address": address,
                "length": length,
                "values": list(self._registers.get(servo_id, {}).get(address, [0] * length)),
            }

    def write_servo_registers(
        self, servo_id: int, address: int, values: list[int]
    ) -> dict[str, object]:
        with self._lock:
            self._before("REG_WRITE", servoId=servo_id, address=address, values=list(values))
            raise ControllerCommandError("REGISTER_WRITE_DISABLED")

    def move(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        with self._lock:
            self._refresh_leases()
            self._before("MOVE", servoId=servo_id, goal=goal, speed=speed)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            if servo_id not in self._leases:
                raise ControllerCommandError("NO_TORQUE_LEASE")
            servo["rawPosition"] = goal
            return {
                "servoId": servo_id,
                "goal": goal,
                "speed": speed,
                "acceleration": acceleration,
            }

    def move_multi_turn(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        """Same authority rules as move(); only the reachable range differs."""

        if not -30_719 <= goal <= 30_719:
            raise ValueError("goal must be between -30719 and 30719")
        with self._lock:
            self._refresh_leases()
            self._before("MOVE_MULTI_TURN", servoId=servo_id, goal=goal, speed=speed)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            if servo_id not in self._leases:
                raise ControllerCommandError("NO_TORQUE_LEASE")
            if (
                servo_id not in self._multi_turn_servos
                or servo.get("operatingMode") != 0
            ):
                raise ControllerCommandError("MODE_NOT_POSITION")
            # The servo's own feedback wraps; the odometer is what accumulates.
            servo["rawPosition"] = goal % 4096
            self._native_positions[servo_id] = goal
            self._odometers[servo_id] = {
                "revolutions": goal // 4096,
                "lastRaw": goal % 4096,
            }
            return {
                "servoId": servo_id,
                "goal": goal,
                "speed": speed,
                "acceleration": acceleration,
            }

    def move_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        """Apply one preflighted group in one deterministic replay mutation."""

        normalized = _normalize_move_set(moves)
        with self._lock:
            self._refresh_leases()
            rendered = [
                {
                    "servoId": servo_id,
                    "goal": goal,
                    "speed": speed,
                    "acceleration": acceleration,
                }
                for servo_id, goal, speed, acceleration in normalized
            ]
            self._before("MOVE_SET", moves=deepcopy(rendered))

            # Finish every state/authority/range check before touching any
            # servo. Firmware performs this before its first dialect SYNC_WRITE;
            # one bad member rejects the complete request without dispatch.
            prepared: list[tuple[dict[str, object], MoveSetTuple, bool]] = []
            for move in normalized:
                servo_id, goal, _speed, _acceleration = move
                servo = self._servos.get(servo_id)
                if servo is None:
                    raise ControllerCommandError("SERVO_NOT_FOUND")
                if self._stopped:
                    raise ControllerCommandError("STOPPED")
                if servo_id not in self._leases:
                    raise ControllerCommandError("NO_TORQUE_LEASE")
                multi_turn = servo_id in self._multi_turn_servos
                if servo.get("operatingMode") != 0:
                    raise ControllerCommandError("MODE_NOT_POSITION")
                if (
                    servo.get("online", True) is not True
                    or servo.get("fresh", True) is not True
                    or servo.get("statusError", 0) != 0
                    or servo.get("errors") != []
                ):
                    raise ControllerCommandError("SERVO_ERROR")
                if multi_turn:
                    odometer = self._odometers.get(servo_id)
                    if (
                        odometer is None
                        or odometer.get("tracking", True) is not True
                        or odometer.get("valid", True) is not True
                        or odometer.get("stepOutstanding", False) is not False
                        or odometer.get("countdownObserved", False) is not False
                        or odometer.get("resyncNeeded", False) is not False
                    ):
                        raise ControllerCommandError("ODOMETER_UNAVAILABLE")
                position_max = (
                    1_023 if self._servo_families.get(servo_id, "STS") == "SCS"
                    else 4_095
                )
                if not multi_turn and not 0 <= goal <= position_max:
                    raise ControllerCommandError("OUT_OF_RANGE")
                prepared.append((servo, move, multi_turn))

            for servo, move, multi_turn in prepared:
                servo_id, goal, _speed, _acceleration = move
                if multi_turn:
                    servo["rawPosition"] = goal % 4096
                    self._native_positions[servo_id] = goal
                    self._odometers[servo_id] = {
                        "revolutions": goal // 4096,
                        "lastRaw": goal % 4096,
                    }
                else:
                    servo["rawPosition"] = goal

            return {
                "controllerId": "replay-hat-a",
                "bootId": self._boot_id,
                "firmwareVersion": "replay-1.0.0",
                "protocolVersion": 1,
                "dispatch": "dialect_grouped_sync_write",
                "crossFamilyAtomic": False,
                "count": len(rendered),
                "moved": rendered,
            }

    def follow_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        """Apply exactly two STS goals and return their fresh compact feedback."""

        normalized = _normalize_follow_set(moves)
        with self._lock:
            self._refresh_leases()
            rendered = [
                {
                    "servoId": servo_id,
                    "goal": goal,
                    "speed": speed,
                    "acceleration": acceleration,
                }
                for servo_id, goal, speed, acceleration in normalized
            ]
            self._before("FOLLOW_SET", moves=deepcopy(rendered))

            prepared: list[tuple[dict[str, object], MoveSetTuple, bool]] = []
            for move in normalized:
                servo_id, goal, _speed, _acceleration = move
                if self._servo_families.get(servo_id, "STS") != "STS":
                    raise ControllerCommandError("UNSUPPORTED")
                servo = self._servos.get(servo_id)
                if servo is None:
                    raise ControllerCommandError("SERVO_NOT_FOUND")
                if self._stopped:
                    raise ControllerCommandError("STOPPED")
                if servo_id not in self._leases:
                    raise ControllerCommandError("NO_TORQUE_LEASE")
                multi_turn = servo_id in self._multi_turn_servos
                if servo.get("operatingMode") != 0:
                    raise ControllerCommandError("MODE_NOT_POSITION")
                if (
                    servo.get("online", True) is not True
                    or servo.get("fresh", True) is not True
                    or servo.get("statusError", 0) != 0
                    or servo.get("errors") != []
                ):
                    raise ControllerCommandError("SERVO_ERROR")
                if multi_turn:
                    odometer = self._odometers.get(servo_id)
                    if (
                        odometer is None
                        or odometer.get("tracking", True) is not True
                        or odometer.get("valid", True) is not True
                        or odometer.get("stepOutstanding", False) is not False
                        or odometer.get("countdownObserved", False) is not False
                        or odometer.get("resyncNeeded", False) is not False
                    ):
                        raise ControllerCommandError("ODOMETER_UNAVAILABLE")
                if not multi_turn and not 0 <= goal <= 4_095:
                    raise ControllerCommandError("OUT_OF_RANGE")
                prepared.append((servo, move, multi_turn))

            feedback: list[dict[str, object]] = []
            for servo, move, multi_turn in prepared:
                servo_id, goal, _speed, _acceleration = move
                raw_position = goal % 4096 if multi_turn else goal
                servo["rawPosition"] = raw_position
                servo["moving"] = False
                servo["packetAgeMs"] = 0
                if multi_turn:
                    self._native_positions[servo_id] = goal
                    self._odometers[servo_id] = {
                        "revolutions": goal // 4096,
                        "lastRaw": raw_position,
                    }
                feedback.append(
                    {
                        "servoId": servo_id,
                        "rawPosition": raw_position,
                        "moving": False,
                        "packetAgeMs": 0,
                        "voltageDeciVolts": int(
                            round(float(servo.get("voltageVolts", 0.0)) * 10)
                        ),
                        "temperatureC": int(
                            round(float(servo.get("temperatureC", 0)))
                        ),
                    }
                )

            return {
                "controllerId": "replay-hat-a",
                "bootId": self._boot_id,
                "firmwareVersion": "replay-1.0.0",
                "protocolVersion": 1,
                "dispatch": "dialect_grouped_sync_write",
                "crossFamilyAtomic": False,
                "count": 2,
                "moved": rendered,
                "feedback": feedback,
            }

    def follow_read(self, servo_ids: list[int]) -> dict[str, object]:
        """Return fresh compact state for an ordered STS pair without motion."""

        normalized = _normalize_follow_read(servo_ids)
        with self._lock:
            self._refresh_leases()
            self._before("FOLLOW_READ", servoIds=list(normalized))
            feedback: list[dict[str, object]] = []
            for servo_id in normalized:
                if self._servo_families.get(servo_id, "STS") != "STS":
                    raise ControllerCommandError("UNSUPPORTED")
                servo = self._servos.get(servo_id)
                if servo is None:
                    raise ControllerCommandError("SERVO_NOT_FOUND")
                if self._stopped:
                    raise ControllerCommandError("STOPPED")
                if servo_id not in self._leases:
                    raise ControllerCommandError("NO_TORQUE_LEASE")
                if (
                    servo.get("operatingMode") != 0
                    or servo.get("online", True) is not True
                    or servo.get("fresh", True) is not True
                    or servo.get("statusError", 0) != 0
                    or servo.get("errors") != []
                    or servo.get("torqueState") != "on"
                ):
                    raise ControllerCommandError("SERVO_ERROR")
                feedback.append(
                    {
                        "servoId": servo_id,
                        "rawPosition": int(servo.get("rawPosition", 0)),
                        "moving": bool(servo.get("moving", False)),
                        "packetAgeMs": 0,
                        "voltageDeciVolts": int(
                            round(float(servo.get("voltageVolts", 0.0)) * 10)
                        ),
                        "temperatureC": int(
                            round(float(servo.get("temperatureC", 0)))
                        ),
                    }
                )
            return {
                "controllerId": "replay-hat-a",
                "bootId": self._boot_id,
                "firmwareVersion": "replay-1.0.0",
                "protocolVersion": 1,
                "count": 2,
                "feedback": feedback,
            }

    def set_multi_turn(self, servo_id: int, enabled: bool) -> dict[str, object]:
        with self._lock:
            self._before("MULTI_TURN", servoId=servo_id, enabled=enabled)
            if servo_id not in self._servos:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            # Firmware mode/config writes deliberately invalidate counted truth;
            # ODO_ZERO is the only operation that establishes the new frame.
            self._odometers.pop(servo_id, None)
            if enabled:
                self._multi_turn_servos.add(servo_id)
                self._servos[servo_id]["operatingMode"] = 0
            else:
                self._multi_turn_servos.discard(servo_id)
                self._servos[servo_id]["operatingMode"] = 0
            self._leases.pop(servo_id, None)
            self._servos[servo_id]["torqueState"] = "off"
            return {
                "servoId": servo_id,
                "multiTurn": enabled,
                "operatingMode": 0,
                "angleMin": 0,
                "angleMax": 0 if enabled else 4095,
            }

    def odometer_zero(self, servo_id: int) -> dict[str, object]:
        with self._lock:
            self._before("ODO_ZERO", servoId=servo_id)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            position = self._current_native_position(servo_id, servo)
            revolutions, raw = divmod(position, 4096)
            servo["rawPosition"] = raw
            self._odometers[servo_id] = {
                "revolutions": revolutions,
                "lastRaw": raw,
                "tracking": True,
                "valid": True,
                "stepOutstanding": False,
                "countdownObserved": False,
                "resyncNeeded": False,
                "resyncCount": 0,
            }
            return {
                "servoId": servo_id,
                "tracking": True,
                "valid": True,
                "stepMode": False,
                "revolutions": revolutions,
                "rawPosition": raw,
                "multiTurnPosition": position,
                "sampleAgeMs": 0,
                "stepOutstanding": False,
                "countdownObserved": False,
                "resyncNeeded": False,
                "resyncCount": 0,
            }

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        with self._lock:
            self._before("ODO_READ", servoId=servo_id)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            odometer = self._odometers.get(servo_id)
            if odometer is not None:
                self._current_native_position(servo_id, servo)
            raw = (
                int(odometer["lastRaw"])
                if odometer is not None
                else int(servo.get("rawPosition", 0))
            )
            revolutions = int(odometer["revolutions"]) if odometer is not None else 0
            return {
                "servoId": servo_id,
                "tracking": (
                    bool(odometer.get("tracking", True))
                    if odometer is not None
                    else False
                ),
                "valid": (
                    bool(odometer.get("valid", True))
                    if odometer is not None
                    else False
                ),
                "stepMode": False,
                "revolutions": revolutions,
                "rawPosition": raw,
                "multiTurnPosition": revolutions * 4096 + raw,
                "sampleAgeMs": 0,
                "stepOutstanding": (
                    bool(odometer.get("stepOutstanding", False))
                    if odometer is not None
                    else False
                ),
                "countdownObserved": (
                    bool(odometer.get("countdownObserved", False))
                    if odometer is not None
                    else False
                ),
                "resyncNeeded": (
                    bool(odometer.get("resyncNeeded", False))
                    if odometer is not None
                    else False
                ),
                "resyncCount": (
                    int(odometer.get("resyncCount", 0))
                    if odometer is not None
                    else 0
                ),
            }

    def home_multi_turn(self, servo_id: int) -> dict[str, object]:
        """Atomically establish the operator-confirmed Base zero frame."""

        with self._lock:
            self._before("HOME_MULTI_TURN", servoId=servo_id)
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            position = self._current_native_position(servo_id, servo)
            revolutions, raw = divmod(position, 4096)
            servo["rawPosition"] = raw
            self._multi_turn_servos.add(servo_id)
            self._leases.pop(servo_id, None)
            servo["operatingMode"] = 0
            servo["torqueState"] = "off"
            self._odometers[servo_id] = {
                "revolutions": revolutions,
                "lastRaw": raw,
                "tracking": True,
                "valid": True,
                "stepOutstanding": False,
                "countdownObserved": False,
                "resyncNeeded": False,
                "resyncCount": 0,
            }
            return {
                "servoId": servo_id,
                "tracking": True,
                "valid": True,
                "stepMode": False,
                "revolutions": revolutions,
                "rawPosition": raw,
                "multiTurnPosition": position,
                "sampleAgeMs": 0,
                "stepOutstanding": False,
                "countdownObserved": False,
                "resyncNeeded": False,
                "resyncCount": 0,
            }

    def torque_lease(self, servo_id: int, lease_ms: int) -> dict[str, object]:
        with self._lock:
            self._before("TORQUE_LEASE", servoId=servo_id, leaseMs=lease_ms)
            self._proposals.clear()
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            self._leases[servo_id] = time.monotonic() + (lease_ms / 1000.0)
            servo["torqueState"] = "on"
            return {"servoId": servo_id, "leaseMs": lease_ms, "torqueState": "on"}

    def set_hold_servos(
        self, servo_ids: list[int], lease_ms: int = 1_500
    ) -> dict[str, object]:
        # Mirrors the firmware's MAX_HOLD_SERVOS, which is 4 as of arm-hat-2.0.0.
        # A fake that is stricter than the device makes tests fail on behaviour
        # the hardware allows; a fake that is looser lets a real refusal through
        # untested. It has to track the constant either way.
        if len(servo_ids) > 4 or len(set(servo_ids)) != len(servo_ids):
            raise ValueError("hold set must contain at most four unique servo ids")
        with self._lock:
            self._before("HOLD_SET", servoIds=list(servo_ids), leaseMs=lease_ms)
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            if any(identifier not in self._servos for identifier in servo_ids):
                raise ControllerCommandError("SERVO_NOT_FOUND")
            now = time.monotonic()
            desired = set(servo_ids)
            for identifier, servo in self._servos.items():
                if identifier in desired:
                    self._leases[identifier] = now + (lease_ms / 1000.0)
                    servo["torqueState"] = "on"
                else:
                    self._leases.pop(identifier, None)
                    servo["torqueState"] = "off"
                    servo["operatingMode"] = 0
            self._proposals.clear()
            return {
                "servoIds": list(servo_ids),
                "leaseMs": lease_ms,
                "torqueState": "on" if servo_ids else "off",
                "confirmed": True,
            }

    def torque_off(self, servo_id: int | None = None) -> dict[str, object]:
        with self._lock:
            self._before("TORQUE_OFF", servoId=servo_id)
            self._proposals.clear()
            identifiers = list(self._servos) if servo_id is None else [servo_id]
            if servo_id is not None and servo_id not in self._servos:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            for identifier in identifiers:
                self._leases.pop(identifier, None)
                self._servos[identifier]["torqueState"] = "off"
                self._servos[identifier]["operatingMode"] = 0
            return {
                "servoId": servo_id,
                "all": servo_id is None,
                "torqueState": "off",
            }

    def prepare_nudge(
        self,
        servo_id: int,
        delta_ticks: int,
        speed: int,
        acceleration: int,
    ) -> dict[str, object]:
        with self._lock:
            self._before(
                "PREPARE_NUDGE",
                servoId=servo_id,
                deltaTicks=delta_ticks,
                speed=speed,
                acceleration=acceleration,
            )
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            if self._stopped:
                raise ControllerCommandError("STOPPED")
            current = int(servo["rawPosition"])
            target = current + delta_ticks
            if not 0 <= target <= 4095:
                raise ControllerCommandError("NUDGE_OUT_OF_RANGE")
            proposal_id = f"nudge_{secrets.token_urlsafe(12)}"
            proposal = {
                "proposalId": proposal_id,
                "servoId": servo_id,
                "deltaTicks": delta_ticks,
                "startRawPosition": current,
                "currentRawPosition": current,
                "targetRawPosition": target,
                "speed": speed,
                "acceleration": acceleration,
                "expiresInMs": 10_000,
            }
            proposal["proposalHash"] = _proposal_hash(proposal)
            self._proposals[proposal_id] = deepcopy(proposal)
            return proposal

    def execute_nudge(
        self, proposal_id: str, proposal_hash: str | None = None
    ) -> dict[str, object]:
        with self._lock:
            self._before(
                "EXECUTE_NUDGE", proposalId=proposal_id, proposalHash=proposal_hash
            )
            proposal = self._proposals.pop(proposal_id, None)
            if proposal is None:
                raise ControllerCommandError("PROPOSAL_USED_OR_UNKNOWN")
            expected_hash = str(proposal["proposalHash"])
            if proposal_hash is not None and not secrets.compare_digest(
                expected_hash, proposal_hash
            ):
                raise ControllerCommandError("PROPOSAL_HASH_MISMATCH")
            servo_id = int(proposal["servoId"])
            servo = self._servos.get(servo_id)
            if servo is None:
                raise ControllerCommandError("SERVO_NOT_FOUND")
            servo["rawPosition"] = int(proposal["targetRawPosition"])
            servo["torqueState"] = "off"
            self._leases.pop(servo_id, None)
            return {
                "proposalId": proposal_id,
                "servoId": servo_id,
                "completed": True,
                "startRawPosition": proposal["startRawPosition"],
                "targetRawPosition": proposal["targetRawPosition"],
                "measuredDeltaTicks": proposal["deltaTicks"],
                "positionErrorTicks": 0,
                "rawPosition": servo["rawPosition"],
                "torqueState": "off",
                "evidenceId": f"obs_{secrets.token_urlsafe(12)}",
            }

    def stop(self) -> dict[str, object]:
        with self._lock:
            self._before("STOP")
            self._stopped = True
            self._leases.clear()
            self._proposals.clear()
            for servo in self._servos.values():
                servo["torqueState"] = "off"
                servo["operatingMode"] = 0
            return {"stopped": True, "torqueState": "off"}

    def reset(self, *, inspected: bool = False) -> dict[str, object]:
        with self._lock:
            self._before("RESET", inspected=inspected)
            self._stopped = False
            self._leases.clear()
            for servo in self._servos.values():
                servo["torqueState"] = "off"
                servo["operatingMode"] = 0
            return {"stopped": False, "torqueState": "off"}


__all__ = [
    "ArmController",
    "ArmControllerError",
    "ControllerCommandError",
    "ControllerProtocolError",
    "ControllerTransportError",
    "ControllerUnavailableError",
    "MoveSetTuple",
    "ReplayArmController",
    "TorqueState",
    "UnavailableArmController",
]
