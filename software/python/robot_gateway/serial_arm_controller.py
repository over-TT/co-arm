"""Strict A1 serial transport for the dedicated ESP32 arm controller."""

from __future__ import annotations

from copy import deepcopy
import json
import math
import re
import secrets
import threading
import time
from typing import Callable, Protocol

from .arm_controller import (
    ControllerCommandError,
    ControllerProtocolError,
    ControllerTransportError,
    ControllerUnavailableError,
    MoveSetTuple,
    _normalize_follow_read,
    _normalize_follow_set,
    _normalize_move_set,
)


PROTOCOL_TAG = "A1"
# Mirrors the firmware's MAX_HOLD_SERVOS (4 as of arm-hat-2.0.0). Named, and
# used on both the send and the parse side, because it was previously written
# out as a bare 3 in two places: raising the firmware and the API to four left
# those behind, so holding all four joints was refused with "at most three
# servos can be held" -- surfaced to the browser as "gateway unavailable".
MAX_HOLD_SERVOS = 4
PROTOCOL_VERSION = 1
DEFAULT_BAUDRATE = 115_200
DEFAULT_TIMEOUT_SECONDS = 0.5
HEARTBEAT_INTERVAL_SECONDS = 0.2
# Mirrors ODOMETER_MAX_REVOLUTIONS in the HAT firmware.
ODOMETER_MAX_REVOLUTIONS = 64

# The HAT owns the ST3215's counted coordinate. The Pi sends one signed absolute
# destination; the HAT compares it with live encoder samples and closes the move
# in Mode 0. PRESENT POSITION still wraps at 4095, so only ODO_READ may be used
# as multi-turn truth.
MULTI_TURN_GOAL_LIMIT = 30_719
# Mode 0 is the native absolute path. Mode 3 remains parse-compatible with an
# older HAT, but the service capability gate never drives Base through it.
HOLDING_MODES = frozenset({0, 3})
# The disabled diagnostic register-write operation still has a six-token A1
# parser budget; its id and address leave room for at most four data bytes.
MAX_REGISTER_WRITE_VALUES = 4
_REGISTER_ACCELERATION = 41
_REGISTER_PHASE = 18
_REGISTER_RESOLUTION = 30
_REGISTER_OPERATING_MODE = 33
_REGISTER_TORQUE_ENABLE = 40
_REGISTER_LOCK = 55
_REGISTER_PRESENT_POSITION = 56
_STS_PHASE_EXTENDED_POSITION = 0x10
_INSPECTION_SAFE_OPERATIONS = frozenset(
    {
        "CAPTURE",
        "HEARTBEAT",
        "ODO_READ",
        "REG_READ",
        "RESET",
        "SCAN",
        "STATUS",
        "STOP",
        "TORQUE_OFF",
    }
)
_MOVE_SET_PREFLIGHT_ERRORS = frozenset(
    {
        "BAD_ARGS",
        "HEARTBEAT_STALE",
        "NO_TORQUE_LEASE",
        "ODOMETER_UNAVAILABLE",
        "OUT_OF_RANGE",
        "SERVO_ERROR",
        "STOPPED",
        "UNSUPPORTED",
    }
)
_FOLLOW_READ_ERRORS = frozenset(
    {
        "BAD_ARGS",
        "FEEDBACK_UNAVAILABLE",
        "HEARTBEAT_STALE",
        "NO_TORQUE_LEASE",
        "OUT_OF_RANGE",
        "SERVO_ERROR",
        "STOPPED",
        "UNSUPPORTED",
    }
)
_CONTROLLER_STOP_REASONS = frozenset(
    {"MOVE_SET_FAILED", "EXPLICIT_STOP", "SAFETY_FAULT"}
)
_REGISTER_GOAL_POSITION = 42
_REGISTER_GOAL_SPEED = 46
_REGISTER_MIN_ANGLE = 9
_REGISTER_MAX_ANGLE = 11

# Motion policy for this arm, pushed to the controller on connect. These are the
# knobs that previously required a firmware flash to change. Each is clamped by a
# hard ceiling on the controller; the fail-safes (watchdog, torque lease, STOP
# latch, boot torque-off) are deliberately NOT settable from here.
MOTION_POLICY: dict[str, int] = {
    # Tuned for a bench arm an operator is standing next to with a STOP button,
    # not for an industrial cell. The fail-safes below still hold: watchdog,
    # torque lease, STOP latch, boot torque-off.
    "MAX_DELTA": 512,
    "MOTION_BUDGET_MS": 600,
    # A geared joint will not settle to a couple of ticks; this is a property of
    # the arm, so it belongs here rather than in the controller binary.
    "POS_TOLERANCE": 24,
    "EXEC_TIMEOUT_MS": 600,
    # MAX_SPEED gates MOVE as well as NUDGE, and MOVE is what the drive bar uses.
    # At 256 a full motor turn takes ~19 s, which reads as "the bar does nothing".
    # 2400 puts it near 2 s, which is what a drag is expected to feel like.
    "MAX_SPEED": 2400,
    # The installed ST3215 Shoulder and Elbow servos clamp register 41 to 50.
    # Advertising a larger value makes strict goal readback correctly reject
    # the command after dispatch, so 50 is the honest device limit.
    "MAX_ACCEL": 50,
    # A moving servo returns the odd transient status byte; revoking authority on
    # the first one latched STOP on every real move.
    "SUPERVISION_TOLERANCE": 6,
}
MAX_REQUEST_BYTES = 256
MAX_RESPONSE_BYTES = 4096
OPERATION_TIMEOUT_SECONDS = {
    # A 0..253 scan includes the full ping census, torque-off reconciliation,
    # telemetry refreshes, and JSON transmission. Keep it bounded without
    # mistaking a slow but valid commissioning exchange for a dead UART.
    "SCAN": 4.0,
    # ID assignment performs the full census plus verified EEPROM writes.
    "ASSIGN_ID": 1.5,
    # Position-mode restore performs the same census plus verified EEPROM writes.
    "SET_POSITION_MODE": 1.5,
    # Firmware bounds the physical nudge wait itself, then verifies readback
    # and torque-off before replying.
    "EXECUTE_NUDGE": 1.25,
    # Up to four addressed goal readbacks follow one or two no-ACK family
    # broadcasts; leave bounded headroom above the ordinary UART timeout.
    "MOVE_SET": 0.75,
    # Exactly two goal readbacks plus fresh telemetry/torque/mode reads.
    "FOLLOW_SET": 0.75,
    "FOLLOW_READ": 0.75,
}
_SAFE_OPERATION = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")
_SAFE_ERROR = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")
_SAFE_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_SAFE_PROPOSAL = re.compile(r"^[A-Za-z0-9_-]{8,128}$")


class SerialPort(Protocol):
    is_open: bool

    def write(self, payload: bytes) -> int: ...

    def flush(self) -> None: ...

    def reset_input_buffer(self) -> None: ...

    def readline(self, size: int = -1) -> bytes: ...

    def close(self) -> None: ...


SerialFactory = Callable[..., SerialPort]


def _default_serial_factory(**kwargs: object) -> SerialPort:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError:
        raise ControllerUnavailableError("serial support is unavailable") from None
    return serial.Serial(**kwargs)


def _reject_constant(_: str) -> None:
    raise ValueError("non-finite JSON number")


def _object_without_duplicates(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _integer(value: object, *, minimum: int, maximum: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ControllerProtocolError(f"invalid {name}")
    if not minimum <= value <= maximum:
        raise ControllerProtocolError(f"invalid {name}")
    return value


class SerialArmController:
    """One-request-at-a-time host transport with a fail-closed heartbeat."""

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        serial_factory: SerialFactory = _default_serial_factory,
    ) -> None:
        if not isinstance(port, str) or not port or len(port) > 512:
            raise ValueError("serial port must be a non-empty string")
        if any(ord(character) < 32 for character in port):
            raise ValueError("serial port contains control characters")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if isinstance(timeout_seconds, bool) or not isinstance(timeout_seconds, (int, float)):
            raise ValueError("timeout_seconds must be a number")
        if not 0.05 <= float(timeout_seconds) <= 5.0:
            raise ValueError("timeout_seconds must be between 0.05 and 5 seconds")
        self._port = port
        self._baudrate = baudrate
        self._timeout_seconds = float(timeout_seconds)
        self._serial_factory = serial_factory
        self._serial: SerialPort | None = None
        self._sequence = 0
        self._io_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._identity: dict[str, object] | None = None
        self._connection = "offline"
        self._bus = "unknown"
        self._servos = "unknown"
        self._motion_state = "blocked"
        self._torque_state = "unknown"
        self._servo_count: int | None = None
        self._last_heartbeat: float | None = None
        self._last_scan: dict[str, object] | None = None
        self._telemetry: dict[int, dict[str, object]] = {}
        self._bus_baud = 1_000_000
        self._hardware_estop = "not_detected"
        self._prepared_proposals: dict[str, dict[str, object]] = {}
        self._active_hold_ids: list[int] = []
        self._servo_families: dict[int, str] = {}
        self._declared_native_multi_turn_servos: set[int] = set()
        self._operator_inspection_required = False
        self._safety_stop_reason: str | None = None
        self._last_motion_failure: dict[str, object] | None = None
        self._transmission_attempt = 0

    def start(self) -> dict[str, object]:
        with self._io_lock:
            if self._connection == "online" and self._identity is not None:
                return deepcopy(self._identity)
            try:
                self._serial = self._serial_factory(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=max(
                        self._timeout_seconds,
                        max(OPERATION_TIMEOUT_SECONDS.values()),
                    ),
                    write_timeout=self._timeout_seconds,
                )
                # UART0 may contain ESP32 ROM boot text. Discard it exactly
                # once before HELLO; after a request, framing remains strict
                # and arbitrary lines are never skipped.
                reset_input = getattr(self._serial, "reset_input_buffer", None)
                if not callable(reset_input):
                    raise OSError("serial input reset unavailable")
                reset_input()
            except ControllerUnavailableError:
                self._transport_fault()
                raise
            except Exception:
                self._transport_fault()
                raise ControllerUnavailableError("controller unavailable") from None
            try:
                hello = self._command_locked("HELLO")
                identity = self._validate_hello(hello)
            except Exception:
                self._mark_failed_for_current_exception()
                self._close_serial_locked()
                raise
            with self._state_lock:
                controller_inspection_required = (
                    identity.get("operatorInspectionRequired") is True
                )
                if controller_inspection_required:
                    self._operator_inspection_required = True
                    self._safety_stop_reason = str(identity["safetyStopReason"])
                self._identity = deepcopy(identity)
                self._connection = "online"
                # HELLO proves the controller hop only. The downstream servo
                # bus stays unknown until a valid addressed response arrives.
                self._bus = "unknown"
                self._servos = str(hello.get("servosState", "unknown"))
                self._motion_state = (
                    "stopped" if self._operator_inspection_required else "blocked"
                )
                # A boot broadcast is an attempted safe action, not addressed
                # torque readback from every responding servo.
                self._torque_state = "unknown"
                self._last_heartbeat = time.monotonic()
                supplied_count = hello.get("servoCount")
                self._servo_count = (
                    supplied_count
                    if isinstance(supplied_count, int) and not isinstance(supplied_count, bool)
                    and 0 <= supplied_count <= 254
                    else None
                )
            with self._state_lock:
                inspection_required = self._operator_inspection_required
            if not inspection_required:
                self._apply_motion_policy_locked(hello)
                self._apply_servo_families_locked(hello)
            self._start_heartbeat_locked()
            return deepcopy(hello)

    def declare_family(self, servo_id: int, family: str) -> None:
        """Record which register dialect a servo id speaks, and tell the controller.

        The SC09 is an SCS-family part sharing a bus with STS-family arm servos.
        Framing, PING and the ID register are common -- which is why it scans and
        re-IDs correctly against STS code -- but every 16-bit field is
        byte-swapped and three registers move or do not exist. Nothing on the
        wire distinguishes the families reliably, so the mapping is declared
        here and replayed after every reconnect rather than sniffed.
        """

        if family not in ("STS", "SCS"):
            raise ValueError("family must be STS or SCS")
        if isinstance(servo_id, bool) or not isinstance(servo_id, int):
            raise ValueError("servo id must be an integer")
        if not 0 <= servo_id <= 253:
            raise ValueError("servo id must be between 0 and 253")
        with self._io_lock:
            if self._servo_families.get(servo_id) == family:
                return
            # Calibration may legitimately change the desired family map while
            # a controller STOP is awaiting inspection. Persist that validated
            # intent locally, but never mutate the HAT while the safety gate is
            # closed. An inspected RESET replays every declaration and validates
            # its exact receipt before releasing the host-side motion gate.
            self._servo_families[servo_id] = family
            with self._state_lock:
                online = self._connection == "online"
                inspection_required = self._operator_inspection_required
                identity = deepcopy(self._identity)
            capabilities = (
                identity.get("capabilities", [])
                if isinstance(identity, dict)
                else []
            )
            if (
                online
                and not inspection_required
                and "servo_family" in capabilities
            ):
                self._send_family_locked(servo_id, family)

    def declare_native_multi_turn_servos(self, servo_ids: list[int]) -> None:
        """Replace the host's complete logical native-multi-turn ID set.

        This is host intent only. It sends no controller or servo command; Scan
        may use it to distinguish the configured Base from ordinary STS joints
        when an older HAT boot has forgotten its RAM-only decoder table.
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
        with self._io_lock:
            self._declared_native_multi_turn_servos = set(servo_ids)

    def _send_family_locked(self, servo_id: int, family: str) -> None:
        try:
            payload = self._command_locked("FAMILY", servo_id, family)
        except ControllerCommandError:
            # HELLO already advertised this operation. Continuing after a
            # rejection would let MOVE_SET encode this id using the wrong
            # address/length/endianness, so reject this controller session.
            self._protocol_fault()
            raise ControllerProtocolError("servo family declaration rejected") from None
        if payload != {"servoId": servo_id, "family": family.lower()}:
            self._protocol_fault()
            raise ControllerProtocolError("servo family declaration was not acknowledged")

    def _apply_servo_families_locked(self, hello: dict[str, object]) -> None:
        capabilities = hello.get("capabilities")
        if not isinstance(capabilities, list) or "servo_family" not in capabilities:
            return
        for servo_id, family in sorted(self._servo_families.items()):
            self._send_family_locked(servo_id, family)

    def _apply_motion_policy_locked(self, hello: dict[str, object]) -> None:
        """Push this arm's tuning to the controller on every connect.

        The controller boots with conservative defaults and takes its real
        numbers from here, so retuning the arm is a gateway change rather than a
        reflash. A controller without `runtime_config` keeps its own defaults.
        """

        capabilities = hello.get("capabilities")
        if not isinstance(capabilities, list) or "runtime_config" not in capabilities:
            return
        for key, value in MOTION_POLICY.items():
            try:
                self._command_locked("CONFIG", key, value)
            except ControllerCommandError:
                # An unknown or out-of-ceiling key must not stop the arm coming
                # up; the controller keeps its default for that one setting.
                continue

    def close(self) -> None:
        self._heartbeat_stop.set()
        thread = self._heartbeat_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._io_lock:
            self._close_serial_locked()
        with self._state_lock:
            self._prepared_proposals.clear()
            self._active_hold_ids.clear()
            if self._connection == "online":
                self._connection = "closed"
            self._motion_state = (
                "stopped" if self._operator_inspection_required else "blocked"
            )
            self._torque_state = "unknown"
        self._heartbeat_thread = None

    def reconnect(self) -> dict[str, object]:
        """Perform one explicit bounded reconnect; never retry automatically."""

        self.close()
        return self.start()

    def _close_serial_locked(self) -> None:
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def _start_heartbeat_locked(self) -> None:
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="arm-controller-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def _heartbeat_loop(self) -> None:
        while not self._heartbeat_stop.wait(HEARTBEAT_INTERVAL_SECONDS):
            try:
                payload = self._command("HEARTBEAT")
                self._ingest_status(payload, heartbeat=True)
            except Exception:
                # One delayed/corrupt exchange already makes the cached torque
                # state conservative inside `_command_locked`; it must not kill
                # supervision forever. The old one-shot exit meant RESET or a
                # transient lock timeout could leave an otherwise online
                # controller with no more heartbeats until a full reconnect.
                # Retry on the next bounded interval; close/reconnect still sets
                # the event and terminates this exact thread.
                continue

    def _next_sequence_locked(self) -> int:
        self._sequence = 1 if self._sequence >= 2_147_483_647 else self._sequence + 1
        return self._sequence

    def _command(self, operation: str, *arguments: object) -> dict[str, object]:
        self._require_operation_permitted(operation)
        with self._state_lock:
            if self._connection != "online":
                raise ControllerUnavailableError("controller unavailable")
        with self._io_lock:
            return self._command_locked(operation, *arguments)

    def _command_locked(self, operation: str, *arguments: object) -> dict[str, object]:
        if _SAFE_OPERATION.fullmatch(operation) is None:
            raise ValueError("invalid controller operation")
        serial_port = self._serial
        if serial_port is None or not bool(getattr(serial_port, "is_open", False)):
            self._transport_fault()
            raise ControllerUnavailableError("controller unavailable")
        rendered_arguments: list[str] = []
        for argument in arguments:
            rendered = str(argument)
            if not rendered or any(character.isspace() for character in rendered):
                raise ValueError("controller arguments must be single tokens")
            if not rendered.isascii() or any(ord(character) < 33 for character in rendered):
                raise ValueError("controller arguments must be printable ASCII tokens")
            rendered_arguments.append(rendered)
        sequence = self._next_sequence_locked()
        suffix = f" {' '.join(rendered_arguments)}" if rendered_arguments else ""
        request = f"{PROTOCOL_TAG} {sequence} {operation}{suffix}\n".encode("ascii")
        if len(request) > MAX_REQUEST_BYTES:
            raise ValueError(f"controller request exceeded {MAX_REQUEST_BYTES} bytes")
        try:
            # Mark before the write call: even a reported short write can have
            # put a complete request into a buffered transport or enough bytes
            # on the wire to make outcome ambiguous.
            self._transmission_attempt += 1
            written = serial_port.write(request)
            if written != len(request):
                raise OSError("short serial write")
            serial_port.flush()
            response = self._readline_with_operation_timeout(
                serial_port, operation
            )
        except Exception:
            self._transport_fault()
            raise ControllerTransportError("controller transport failed") from None
        if not isinstance(response, bytes):
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller frame")
        if len(response) > MAX_RESPONSE_BYTES:
            self._protocol_fault()
            raise ControllerProtocolError(
                f"controller response exceeded {MAX_RESPONSE_BYTES} bytes"
            )
        if not response:
            self._transport_fault()
            raise ControllerTransportError("controller response timed out")
        if not response.endswith(b"\n") or b"\r" in response:
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller frame")
        try:
            line = response[:-1].decode("utf-8", errors="strict")
        except UnicodeDecodeError:
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller frame") from None
        return self._parse_response(line, sequence)

    def _readline_with_operation_timeout(
        self, serial_port: SerialPort, operation: str
    ) -> bytes:
        timeout = max(
            self._timeout_seconds,
            OPERATION_TIMEOUT_SECONDS.get(operation, self._timeout_seconds),
        )
        sentinel = object()
        previous = getattr(serial_port, "timeout", sentinel)
        try:
            if previous is not sentinel:
                setattr(serial_port, "timeout", timeout)
            return serial_port.readline(MAX_RESPONSE_BYTES + 1)
        finally:
            if previous is not sentinel:
                try:
                    setattr(serial_port, "timeout", previous)
                except Exception:
                    # A transport that stops accepting configuration changes is
                    # treated by the next command as a transport failure.
                    pass

    def _parse_response(self, line: str, expected_sequence: int) -> dict[str, object]:
        ok_match = re.fullmatch(r"A1 ([1-9][0-9]*) OK (.+)", line)
        error_match = re.fullmatch(
            r"A1 ([1-9][0-9]*) ERR ([A-Z][A-Z0-9_]{0,31}) (.+)", line
        )
        if ok_match is None and error_match is None:
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller frame")
        match = ok_match if ok_match is not None else error_match
        assert match is not None
        try:
            sequence = int(match.group(1))
        except ValueError:
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller sequence") from None
        if sequence != expected_sequence:
            self._protocol_fault()
            raise ControllerProtocolError("controller sequence mismatch")
        payload_text = match.group(2) if ok_match is not None else match.group(3)
        try:
            payload = json.loads(
                payload_text,
                parse_constant=_reject_constant,
                object_pairs_hook=_object_without_duplicates,
            )
        except (json.JSONDecodeError, UnicodeError, ValueError):
            self._protocol_fault()
            raise ControllerProtocolError("invalid controller JSON") from None
        if not isinstance(payload, dict):
            self._protocol_fault()
            raise ControllerProtocolError("controller JSON must be an object")
        if error_match is not None:
            error_code = error_match.group(2)
            if _SAFE_ERROR.fullmatch(error_code) is None:
                self._protocol_fault()
                raise ControllerProtocolError("invalid controller error code")
            raise ControllerCommandError(error_code, payload)
        return payload

    @staticmethod
    def _validate_hello(payload: dict[str, object]) -> dict[str, object]:
        required_strings = ("controllerId", "bootId", "firmwareVersion")
        for name in required_strings:
            value = payload.get(name)
            if not isinstance(value, str) or _SAFE_IDENTITY.fullmatch(value) is None:
                raise ControllerProtocolError("invalid controller identity handshake")
        protocol = payload.get("protocolVersion")
        if isinstance(protocol, bool) or protocol != PROTOCOL_VERSION:
            raise ControllerProtocolError("unsupported controller protocol version")
        inspection_required = payload.get("operatorInspectionRequired", False)
        stop_reason = payload.get("safetyStopReason")
        controller_state = payload.get("state")
        motion_state = payload.get("motionState", "blocked")
        inspection_tuple_valid = (
            inspection_required is False
            and stop_reason is None
            and controller_state in {"disarmed", "blocked"}
            and motion_state in {"blocked", "ready"}
            and payload.get("safetyFault", False) is False
        ) or (
            inspection_required is False
            and stop_reason is None
            and controller_state == "faulted"
            and motion_state == "blocked"
            and payload.get("safetyFault") is True
        ) or (
            inspection_required is True
            and stop_reason in _CONTROLLER_STOP_REASONS
            and controller_state == "stopped_latched"
            and motion_state == "stopped"
            and payload.get("safetyFault") is True
        )
        if not inspection_tuple_valid:
            raise ControllerProtocolError("invalid controller inspection latch")
        if controller_state not in {
            "disarmed",
            "blocked",
            "faulted",
            "stopped_latched",
        }:
            raise ControllerProtocolError("controller did not start disarmed")
        reported_torque = payload.get("torqueState", "off")
        if reported_torque not in {"off", "unknown"}:
            raise ControllerProtocolError("controller did not confirm torque off")
        if reported_torque == "unknown" and payload.get("bootTorqueOffSent") is not True:
            raise ControllerProtocolError("controller boot torque-off attempt is unverified")
        extended_keys = {
            "requestMaxBytes",
            "responseMaxBytes",
            "host",
            "servoBus",
            "watchdogMs",
            "leaseMs",
            "capabilities",
            "bootTorqueOffSent",
        }
        if (extended_keys - {"bootTorqueOffSent"}) & set(payload):
            host = payload.get("host")
            servo_bus = payload.get("servoBus")
            lease = payload.get("leaseMs")
            capabilities = payload.get("capabilities")
            required_capabilities = {
                "scan",
                "single_servo_id",
                "capture",
                "torque_lease",
                "bounded_nudge",
                "telemetry",
                "stop",
            }
            if (
                payload.get("requestMaxBytes") != MAX_REQUEST_BYTES
                or payload.get("responseMaxBytes") != MAX_RESPONSE_BYTES
                or host != {"uart": "Serial0", "baud": DEFAULT_BAUDRATE}
                or servo_bus
                != {"uart": "Serial1", "baud": 1_000_000, "rx": 18, "tx": 19}
                or not isinstance(payload.get("watchdogMs"), int)
                or isinstance(payload.get("watchdogMs"), bool)
                or not 250 <= int(payload["watchdogMs"]) <= 2_000
                or lease != {"min": 100, "max": 2_000}
                or not isinstance(capabilities, list)
                or any(not isinstance(item, str) for item in capabilities)
                or not required_capabilities.issubset(set(capabilities))
                or payload.get("bootTorqueOffSent") is not True
                or payload.get("servoModel") != "user_confirmation_required"
                or payload.get("registerProfile") != "ST3215_candidate"
            ):
                raise ControllerProtocolError("controller capability handshake mismatch")
        raw_capabilities = payload.get("capabilities")
        capabilities = (
            sorted(set(raw_capabilities))
            if isinstance(raw_capabilities, list)
            and all(isinstance(item, str) for item in raw_capabilities)
            else []
        )
        if "move_set_v1" in capabilities and "servo_family" not in capabilities:
            raise ControllerProtocolError("controller capability dependency mismatch")
        if (
            "follow_set_feedback_v1" in capabilities
            and "servo_family" not in capabilities
        ):
            raise ControllerProtocolError("controller capability dependency mismatch")
        if (
            "follow_feedback_v1" in capabilities
            and "servo_family" not in capabilities
        ):
            raise ControllerProtocolError("controller capability dependency mismatch")
        return {
            "controllerId": payload["controllerId"],
            "bootId": payload["bootId"],
            "firmwareVersion": payload["firmwareVersion"],
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": capabilities,
            "operatorInspectionRequired": inspection_required,
            "safetyStopReason": stop_reason,
        }

    def _transport_fault(self) -> None:
        self._heartbeat_stop.set()
        with self._io_lock:
            self._close_serial_locked()
        with self._state_lock:
            self._prepared_proposals.clear()
            self._active_hold_ids.clear()
            self._connection = "faulted"
            self._bus = "unknown"
            self._servos = "unknown"
            self._motion_state = (
                "stopped" if self._operator_inspection_required else "blocked"
            )
            self._torque_state = "unknown"

    def _require_operation_permitted(self, operation: str) -> None:
        with self._state_lock:
            inspection_required = self._operator_inspection_required
        if inspection_required and operation not in _INSPECTION_SAFE_OPERATIONS:
            raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")

    def _require_declared_family_support(self, servo_ids: list[int]) -> None:
        """Refuse SCS data paths until the HAT can hold the exact dialect map."""

        with self._io_lock:
            requires_family = any(
                self._servo_families.get(servo_id) == "SCS"
                for servo_id in servo_ids
            )
            if not requires_family:
                return
            with self._state_lock:
                identity = deepcopy(self._identity)
        if not isinstance(identity, dict):
            return
        capabilities = identity.get("capabilities", [])
        if not isinstance(capabilities, list) or "servo_family" not in capabilities:
            # Without FAMILY the HAT defaults this ID to STS, which changes goal
            # address/length/endianness and makes even telemetry/capture unsafe
            # to trust. This check runs before STATUS, hold, torque, or motion.
            raise ControllerCommandError("UNSUPPORTED")

    def _mark_move_set_unconfirmed(self) -> None:
        with self._state_lock:
            self._prepared_proposals.clear()
            self._active_hold_ids.clear()
            self._motion_state = (
                "stopped" if self._operator_inspection_required else "blocked"
            )
            self._torque_state = "unknown"
            self._last_motion_failure = {
                "phase": "unconfirmed",
                "torqueState": "unknown",
                "latentCommands": False,
                "motionMayHaveStarted": True,
                "partialDispatchPossible": True,
            }

    @staticmethod
    def _validated_follow_feedback(
        feedback: object, expected_ids: list[int]
    ) -> list[dict[str, object]]:
        if not isinstance(feedback, list) or len(feedback) != len(expected_ids):
            raise ControllerProtocolError("invalid follow feedback")
        validated: list[dict[str, object]] = []
        for expected_id, candidate in zip(expected_ids, feedback, strict=True):
            if not isinstance(candidate, dict):
                raise ControllerProtocolError("invalid follow feedback")
            servo_id = _integer(
                candidate.get("servoId"), minimum=0, maximum=253,
                name="follow feedback servo id"
            )
            if servo_id != expected_id:
                raise ControllerProtocolError("follow feedback did not match request")
            if not isinstance(candidate.get("moving"), bool):
                raise ControllerProtocolError("invalid follow feedback moving flag")
            validated.append(
                {
                    "servoId": servo_id,
                    "rawPosition": _integer(
                        candidate.get("rawPosition"), minimum=0, maximum=4095,
                        name="follow feedback position"
                    ),
                    "moving": candidate["moving"],
                    "packetAgeMs": _integer(
                        candidate.get("packetAgeMs"), minimum=0,
                        maximum=86_400_000, name="follow feedback age"
                    ),
                    "voltageDeciVolts": _integer(
                        candidate.get("voltageDeciVolts"), minimum=0,
                        maximum=255, name="follow feedback voltage"
                    ),
                    "temperatureC": _integer(
                        candidate.get("temperatureC"), minimum=0, maximum=150,
                        name="follow feedback temperature"
                    ),
                }
            )
        return validated

    def _ingest_follow_feedback(
        self, feedback: list[dict[str, object]]
    ) -> None:
        with self._state_lock:
            for row in feedback:
                servo_id = int(row["servoId"])
                telemetry = deepcopy(self._telemetry.get(servo_id, {}))
                telemetry.update(
                    {
                        "id": servo_id,
                        "rawPosition": row["rawPosition"],
                        "speed": telemetry.get("speed", 0),
                        "load": telemetry.get("load", 0),
                        "voltageVolts": int(row["voltageDeciVolts"]) / 10.0,
                        "temperatureC": row["temperatureC"],
                        "moving": row["moving"],
                        "torqueState": "on",
                        "packetAgeMs": row["packetAgeMs"],
                        "errors": [],
                        "operatingMode": 0,
                        "online": True,
                        "fresh": True,
                        "statusError": 0,
                    }
                )
                self._telemetry[servo_id] = telemetry
            if self._servo_count is None:
                self._servo_count = len(self._telemetry)
            self._bus = "online"
            self._torque_state = "on"
            self._motion_state = (
                "moving" if any(bool(row["moving"]) for row in feedback)
                else "ready"
            )

    _protocol_fault = _transport_fault

    def _mark_failed_for_current_exception(self) -> None:
        # Handshake rejection and transport loss are equally fail-closed here.
        self._transport_fault()

    def _validate_response_identity(
        self, payload: dict[str, object], *, required: bool = False
    ) -> None:
        identity_fields = (
            "controllerId",
            "bootId",
            "firmwareVersion",
            "protocolVersion",
        )
        if not any(field in payload for field in identity_fields):
            if required:
                self._protocol_fault()
                raise ControllerProtocolError(
                    "controller status omitted boot identity; explicit reconnect required"
                )
            return
        with self._state_lock:
            expected = deepcopy(self._identity)
        if expected is None or any(
            field not in payload or payload[field] != expected.get(field)
            for field in identity_fields
        ):
            self._protocol_fault()
            raise ControllerProtocolError(
                "controller identity changed; explicit reconnect required"
            )

    def _ingest_status(
        self,
        payload: dict[str, object],
        *,
        heartbeat: bool = False,
        require_identity: bool = False,
    ) -> None:
        # STATUS carries the controller boot nonce. A response from a new boot
        # must never be accepted beneath HELLO identity or a prepared proposal
        # from the previous boot. HEARTBEAT currently omits identity, but this
        # also validates it fail-closed if a compatible firmware adds it.
        self._validate_response_identity(payload, required=require_identity)
        allowed_enums = {
            "torqueState": {"off", "on", "unknown"},
            "motionState": {"blocked", "ready", "moving", "stopped"},
            "busState": {"online", "offline", "unknown", "faulted"},
            "servosState": {"online", "none_found", "unknown", "faulted"},
            "hardwareEstop": {"not_detected", "active", "released", "unknown"},
        }
        for key, allowed in allowed_enums.items():
            if key in payload and payload[key] not in allowed:
                self._protocol_fault()
                raise ControllerProtocolError(f"invalid {key}")
        if (
            "operatorInspectionRequired" in payload
            or "safetyStopReason" in payload
        ):
            inspection_required = payload.get("operatorInspectionRequired")
            stop_reason = payload.get("safetyStopReason")
            inspection_tuple_valid = (
                inspection_required is False
                and stop_reason is None
                and payload.get("motionState") != "stopped"
                and payload.get("stopped") is not True
            ) or (
                inspection_required is True
                and stop_reason in _CONTROLLER_STOP_REASONS
                and payload.get("motionState") == "stopped"
                and payload.get("stopped") is True
                and payload.get("safetyFault", True) is True
            )
            if not inspection_tuple_valid:
                self._protocol_fault()
                raise ControllerProtocolError("invalid controller inspection latch")
        else:
            inspection_required = False
            stop_reason = None
        telemetry_raw = payload.get("servos")
        validated_telemetry: dict[int, dict[str, object]] | None = None
        if telemetry_raw is not None:
            if not isinstance(telemetry_raw, list) or len(telemetry_raw) > 16:
                self._protocol_fault()
                raise ControllerProtocolError("invalid servo telemetry")
            with self._io_lock:
                locally_declared_families = dict(self._servo_families)
                locally_declared_native_multi_turn = set(
                    self._declared_native_multi_turn_servos
                )
                locally_declared_scs = {
                    servo_id
                    for servo_id, family in locally_declared_families.items()
                    if family == "SCS"
                }
                with self._state_lock:
                    identity = deepcopy(self._identity)
            capabilities = (
                identity.get("capabilities", [])
                if isinstance(identity, dict)
                else []
            )
            if "servo_family" not in capabilities and any(
                isinstance(candidate, dict)
                and candidate.get("id") in locally_declared_scs
                for candidate in telemetry_raw
            ):
                # The HAT decoded this packet through its default STS profile.
                # Accepting the position/mode words would make calibration and
                # arrival proof depend on the wrong endian/register layout.
                self._protocol_fault()
                raise ControllerProtocolError(
                    "SCS telemetry requires servo_family capability"
                )
            validated_telemetry = {}
            for candidate in telemetry_raw:
                try:
                    candidate_id = (
                        candidate.get("id") if isinstance(candidate, dict) else None
                    )
                    validated = self._validated_servo(
                        candidate,
                        declared_family=locally_declared_families.get(candidate_id),
                        native_multi_turn_capable=(
                            candidate_id in locally_declared_native_multi_turn
                            and "multi_turn_absolute_v1" in capabilities
                        ),
                    )
                except ControllerProtocolError:
                    self._protocol_fault()
                    raise
                identifier = int(validated["id"])
                if identifier in validated_telemetry:
                    self._protocol_fault()
                    raise ControllerProtocolError("duplicate servo telemetry")
                validated_telemetry[identifier] = validated
        hold_set_raw = payload.get("holdSet")
        validated_hold_ids: list[int] | None = None
        if "holdSet" in payload:
            if hold_set_raw is None:
                validated_hold_ids = []
            elif isinstance(hold_set_raw, dict):
                raw_ids = hold_set_raw.get("servoIds")
                remaining_ms = hold_set_raw.get("remainingMs")
                if (
                    not isinstance(raw_ids, list)
                    or len(raw_ids) > MAX_HOLD_SERVOS
                    or any(
                        isinstance(identifier, bool)
                        or not isinstance(identifier, int)
                        or not 0 <= identifier <= 253
                        for identifier in raw_ids
                    )
                    or len(set(raw_ids)) != len(raw_ids)
                    or isinstance(remaining_ms, bool)
                    or not isinstance(remaining_ms, int)
                    or not 1 <= remaining_ms <= 2_000
                ):
                    self._protocol_fault()
                    raise ControllerProtocolError("invalid hold set status")
                validated_hold_ids = list(raw_ids)
            else:
                self._protocol_fault()
                raise ControllerProtocolError("invalid hold set status")
        if validated_hold_ids and validated_telemetry is not None and any(
            identifier not in validated_telemetry
            or validated_telemetry[identifier].get("torqueState") != "on"
            for identifier in validated_hold_ids
        ):
            self._protocol_fault()
            raise ControllerProtocolError("hold set telemetry mismatch")
        declared_count = payload.get("servoCount")
        if declared_count is not None and (
            isinstance(declared_count, bool)
            or not isinstance(declared_count, int)
            or not 0 <= declared_count <= 254
        ):
            self._protocol_fault()
            raise ControllerProtocolError("invalid servoCount")
        with self._state_lock:
            if inspection_required:
                self._operator_inspection_required = True
                self._safety_stop_reason = str(stop_reason)
                self._prepared_proposals.clear()
                self._active_hold_ids.clear()
            if heartbeat:
                self._last_heartbeat = time.monotonic()
            if validated_hold_ids is not None:
                self._active_hold_ids = validated_hold_ids
            torque = payload.get("torqueState")
            if torque in {"on", "unknown"}:
                self._torque_state = str(torque)
            motion = payload.get("motionState")
            if motion in {"blocked", "ready", "moving", "stopped"}:
                self._motion_state = (
                    "stopped"
                    if self._operator_inspection_required
                    else str(motion)
                )
            bus = payload.get("busState")
            if bus in {"online", "offline", "unknown", "faulted"}:
                self._bus = str(bus)
            servos = payload.get("servosState")
            if servos in {"online", "none_found", "unknown", "faulted"}:
                self._servos = str(servos)
            previous_count = self._servo_count
            if isinstance(declared_count, int):
                self._servo_count = declared_count
            baud = payload.get("busBaud")
            if isinstance(baud, int) and not isinstance(baud, bool) and 1 <= baud <= 4_000_000:
                self._bus_baud = baud
            hardware_estop = payload.get("hardwareEstop")
            if hardware_estop in {"not_detected", "active", "released", "unknown"}:
                self._hardware_estop = str(hardware_estop)
            if declared_count == 0 and self._bus == "online":
                self._bus = "unknown"
            if validated_telemetry is not None:
                self._telemetry = validated_telemetry
                expected_count = (
                    declared_count
                    if isinstance(declared_count, int)
                    else previous_count
                )
                if expected_count is None:
                    self._servo_count = len(validated_telemetry)
                    expected_count = len(validated_telemetry)
                complete_set = len(validated_telemetry) == expected_count
                self._servos = (
                    "online"
                    if validated_telemetry and complete_set
                    else "none_found"
                    if not validated_telemetry and expected_count == 0
                    else "faulted"
                )
                if not validated_telemetry and expected_count == 0 and self._bus == "online":
                    self._bus = "unknown"
                torque_values = {
                    str(item["torqueState"]) for item in validated_telemetry.values()
                }
                if "on" in torque_values:
                    self._torque_state = "on"
                elif "unknown" in torque_values or not torque_values:
                    self._torque_state = "unknown"
                elif torque_values == {"off"} and complete_set and expected_count > 0:
                    self._torque_state = "off"

    def _require_current_boot_status_locked(self) -> dict[str, object]:
        """Prove the open UART still belongs to the HELLO controller boot."""

        payload = self._command("STATUS")
        self._ingest_status(payload, require_identity=True)
        return payload

    def _command_with_identity_guard(
        self, operation: str, *arguments: object
    ) -> tuple[dict[str, object], dict[str, object]]:
        self._require_operation_permitted(operation)
        # Keep both boot proofs and the command adjacent on the host UART. If
        # the ESP32 reboots in either gap, success is never surfaced beneath
        # stale HELLO identity. There is deliberately no automatic retry.
        with self._io_lock:
            self._require_current_boot_status_locked()
            # STATUS can reveal a controller-side MOVE_SET inspection latch
            # that did not exist when this caller first queued behind the UART
            # lock. Recheck the durable host gate immediately before every
            # non-safe command so stale authorization cannot cross that queue.
            self._require_operation_permitted(operation)
            result = self._command(operation, *arguments)
            status_after = self._require_current_boot_status_locked()
            return result, status_after

    @staticmethod
    def _validated_servo(
        value: object,
        *,
        declared_family: str | None = None,
        native_multi_turn_capable: bool = False,
    ) -> dict[str, object]:
        if not isinstance(value, dict):
            raise ControllerProtocolError("invalid servo telemetry")
        identifier = _integer(value.get("id"), minimum=0, maximum=253, name="servo id")
        speed = value.get("speed")
        load = value.get("load")
        current = value.get("currentMilliamps")
        operating_mode = value.get("operatingMode")
        packet_age = value.get("packetAgeMs")
        if "operatingMode" in value and operating_mode is not None and (
            isinstance(operating_mode, bool)
            or not isinstance(operating_mode, int)
            or not 0 <= operating_mode <= 255
        ):
            raise ControllerProtocolError("invalid servo telemetry")
        encoded_position = _integer(
            value.get("rawPosition"), minimum=0, maximum=65_535, name="raw position"
        )
        if encoded_position <= 4095:
            raw_position = encoded_position
        else:
            # After a HAT reboot, older compatible builds can lack their
            # RAM-only multi-turn classification even though the ST3215 remains
            # in EEPROM-backed native Mode 0. SCAN can expose that mismatch
            # while rebuilding inventory and replaying FAMILY. In that one
            # evidenced case the position word is a signed-magnitude absolute
            # coordinate, not a corrupt single-turn sample. Restore only the
            # one-turn telemetry projection here; ODO_READ remains the sole
            # source of Base-frame trust and stays invalid until the operator
            # chooses Set zero.
            magnitude = encoded_position & 0x7FFF
            if (
                declared_family != "STS"
                or not native_multi_turn_capable
                or operating_mode != 0
                or magnitude > MULTI_TURN_GOAL_LIMIT
            ):
                raise ControllerProtocolError("invalid raw position")
            signed_position = (
                -magnitude if encoded_position & 0x8000 else magnitude
            )
            raw_position = signed_position % 4096
        if (
            isinstance(speed, bool)
            or not isinstance(speed, int)
            or not -32767 <= speed <= 32767
            or isinstance(load, bool)
            or not isinstance(load, int)
            or not -1000 <= load <= 1000
            or isinstance(packet_age, bool)
            or not isinstance(packet_age, int)
            or not 0 <= packet_age <= 86_400_000
        ):
            raise ControllerProtocolError("invalid servo telemetry")
        voltage = value.get("voltageVolts")
        temperature = value.get("temperatureC")
        if (
            isinstance(voltage, bool)
            or not isinstance(voltage, (int, float))
            or not math.isfinite(float(voltage))
            or not 0 <= float(voltage) <= 30
            or isinstance(temperature, bool)
            or not isinstance(temperature, (int, float))
            or not math.isfinite(float(temperature))
            or not -40 <= float(temperature) <= 150
            or not isinstance(value.get("moving"), bool)
            or value.get("torqueState") not in {"off", "on", "unknown"}
        ):
            raise ControllerProtocolError("invalid servo telemetry")
        errors = value.get("errors")
        if (
            not isinstance(errors, list)
            or len(errors) > 16
            or any(
                not isinstance(error, str)
                or not error
                or len(error) > 64
                or not error.isprintable()
                for error in errors
            )
        ):
            raise ControllerProtocolError("invalid servo telemetry")
        result: dict[str, object] = {
            "id": identifier,
            "rawPosition": raw_position,
            "speed": speed,
            "load": load,
            "voltageVolts": float(voltage),
            "temperatureC": float(temperature),
            "moving": value["moving"],
            "torqueState": value["torqueState"],
            "packetAgeMs": packet_age,
            "errors": list(errors),
        }
        if current is not None:
            if (
                isinstance(current, bool)
                or not isinstance(current, int)
                or not -20_000 <= current <= 20_000
            ):
                raise ControllerProtocolError("invalid servo telemetry")
            result["currentMilliamps"] = current
        if "operatingMode" in value:
            if operating_mode is None:
                result["operatingMode"] = None
            else:
                result["operatingMode"] = operating_mode
        return result

    def transport_state(self) -> dict[str, object]:
        with self._state_lock:
            age_ms = None
            if self._last_heartbeat is not None:
                age_ms = max(0, int((time.monotonic() - self._last_heartbeat) * 1000))
            return {
                "configured": True,
                "connection": self._connection,
                "bus": self._bus,
                "servos": self._servos,
                "motionState": self._motion_state,
                "torqueState": self._torque_state,
                "identity": deepcopy(self._identity),
                "servoCount": self._servo_count,
                "heartbeatAgeMs": age_ms,
                "busBaud": self._bus_baud,
                "lastScan": deepcopy(self._last_scan),
                "servosTelemetry": [
                    deepcopy(self._telemetry[identifier])
                    for identifier in sorted(self._telemetry)
                ],
                "hardwareEstop": self._hardware_estop,
                "heldServoIds": list(self._active_hold_ids),
                "operatorInspectionRequired": self._operator_inspection_required,
                "safetyStopReason": self._safety_stop_reason,
                "lastMotionFailure": deepcopy(self._last_motion_failure),
            }

    def status(self) -> dict[str, object]:
        payload = self._command("STATUS")
        self._ingest_status(payload, require_identity=True)
        return self.transport_state()

    def _read_scan_recovery_registers_locked(
        self, servo_id: int, address: int, length: int
    ) -> tuple[list[int], int]:
        """Read one exact proof block without adding another identity-guard pair.

        Scan already owns the UART lane between same-boot STATUS receipts.  A
        valid controller rejection is recoverable evidence (the caller leaves
        telemetry unavailable), while a malformed OK receipt is still protocol
        corruption and must fault the link.
        """

        payload = self._command("REG_READ", servo_id, address, length)
        values = payload.get("values")
        status_error = payload.get("statusError")
        if (
            payload.get("servoId") != servo_id
            or payload.get("address") != address
            or payload.get("length") != length
            or not isinstance(values, list)
            or len(values) != length
            or any(
                isinstance(item, bool)
                or not isinstance(item, int)
                or not 0 <= item <= 255
                for item in values
            )
            or isinstance(status_error, bool)
            or not isinstance(status_error, int)
            or not 0 <= status_error <= 255
        ):
            self._protocol_fault()
            raise ControllerProtocolError(
                "scan recovery register read was not verified"
            )
        return list(values), status_error

    def _native_multi_turn_configuration_proven_locked(self, servo_id: int) -> bool:
        """Prove MULTITURN ON needs no EEPROM or configuration-register writes."""

        try:
            limits_and_phase, first_error = (
                self._read_scan_recovery_registers_locked(
                    servo_id, _REGISTER_MIN_ANGLE, 16
                )
            )
            mode_and_torque, second_error = (
                self._read_scan_recovery_registers_locked(
                    servo_id, _REGISTER_RESOLUTION, 11
                )
            )
            lock_and_position, third_error = (
                self._read_scan_recovery_registers_locked(
                    servo_id, _REGISTER_LOCK, 3
                )
            )
        except ControllerCommandError:
            # BUS_ERROR, STOPPED, or another well-formed controller refusal is
            # not permission to infer configuration.  Preserve the successful
            # PING evidence and leave telemetry unavailable.
            return False

        minimum = limits_and_phase[0] | (limits_and_phase[1] << 8)
        maximum = limits_and_phase[2] | (limits_and_phase[3] << 8)
        phase = limits_and_phase[_REGISTER_PHASE - _REGISTER_MIN_ANGLE]
        resolution = mode_and_torque[0]
        operating_mode = mode_and_torque[
            _REGISTER_OPERATING_MODE - _REGISTER_RESOLUTION
        ]
        torque_enabled = mode_and_torque[
            _REGISTER_TORQUE_ENABLE - _REGISTER_RESOLUTION
        ]
        locked = lock_and_position[0]
        encoded_position = (
            lock_and_position[_REGISTER_PRESENT_POSITION - _REGISTER_LOCK]
            | (
                lock_and_position[
                    _REGISTER_PRESENT_POSITION - _REGISTER_LOCK + 1
                ]
                << 8
            )
        )
        magnitude = encoded_position & 0x7FFF
        return (
            first_error == 0
            and second_error == 0
            and third_error == 0
            and minimum == 0
            and maximum == 0
            and phase & _STS_PHASE_EXTENDED_POSITION != 0
            and resolution == 1
            and operating_mode == 0
            and torque_enabled == 0
            and locked == 1
            and encoded_position > 4095
            and magnitude <= MULTI_TURN_GOAL_LIMIT
        )

    def _restore_native_multi_turn_decoder_locked(self, servo_id: int) -> bool:
        """Restore only the HAT's RAM classification; never establish Base zero."""

        try:
            payload = self._command("MULTITURN", servo_id, "ON")
        except ControllerCommandError:
            return False
        phase = payload.get("phase")
        if (
            payload.get("servoId") != servo_id
            or payload.get("multiTurn") is not True
            or payload.get("angleMin") != 0
            or payload.get("angleMax") != 0
            or payload.get("operatingMode") != 0
            or payload.get("resolution") != 1
            or isinstance(phase, bool)
            or not isinstance(phase, int)
            or not 0 <= phase <= 255
            or phase & _STS_PHASE_EXTENDED_POSITION == 0
        ):
            self._protocol_fault()
            raise ControllerProtocolError(
                "scan recovery multi-turn mode was not acknowledged"
            )
        # The existing HAT operation deliberately clears odometer tracking and
        # truth even on its idempotent no-configuration-write path.  It may
        # repeat torque OFF, but never enables torque or moves the servo.  Scan
        # must never call ODO_ZERO: the operator still establishes physical zero.
        return True

    def scan(self, minimum_id: int, maximum_id: int) -> dict[str, object]:
        if any(isinstance(value, bool) or not isinstance(value, int) for value in (minimum_id, maximum_id)):
            raise ValueError("scan range values must be integers")
        if not 0 <= minimum_id <= 253 or not 0 <= maximum_id <= 253 or minimum_id > maximum_id:
            raise ValueError("scan range must be ordered between 0 and 253")
        self._clear_prepared_proposals()
        # Keep the guarded scan and its one bounded family-recovery pass on the
        # same UART lane. A heartbeat cannot land between the first STATUS,
        # FAMILY replay, and final same-boot STATUS.
        with self._io_lock:
            payload, status_after = self._command_with_identity_guard(
                "SCAN", minimum_id, maximum_id
            )
            found = payload.get("foundIds")
            expected_range = {"minId": minimum_id, "maxId": maximum_id}
            if (
                not isinstance(found, list)
                or any(
                    isinstance(identifier, bool)
                    or not isinstance(identifier, int)
                    or not minimum_id <= identifier <= maximum_id
                    for identifier in found
                )
                or len(set(found)) != len(found)
                or payload.get("completeRange") != expected_range
            ):
                self._protocol_fault()
                raise ControllerProtocolError("invalid scan evidence")
            post_scan_telemetry = status_after.get("servos")
            if not isinstance(post_scan_telemetry, list):
                self._protocol_fault()
                raise ControllerProtocolError("scan status omitted servo telemetry")
            first_post_scan_ids = {
                int(item["id"])
                for item in post_scan_telemetry
                if isinstance(item, dict)
                and isinstance(item.get("id"), int)
                and not isinstance(item.get("id"), bool)
                and minimum_id <= int(item["id"]) <= maximum_id
            }
            missing_declared_ids = sorted(
                identifier
                for identifier in self._servo_families
                if minimum_id <= identifier <= maximum_id
                and identifier not in first_post_scan_ids
            )
            with self._state_lock:
                identity = deepcopy(self._identity)
            capabilities = (
                identity.get("capabilities", [])
                if isinstance(identity, dict)
                else []
            )
            if missing_declared_ids and "servo_family" in capabilities:
                for identifier in missing_declared_ids:
                    self._send_family_locked(
                        identifier, self._servo_families[identifier]
                    )
                status_after = self._require_current_boot_status_locked()
                post_scan_telemetry = status_after.get("servos")
                if not isinstance(post_scan_telemetry, list):
                    self._protocol_fault()
                    raise ControllerProtocolError(
                        "scan status omitted servo telemetry"
                    )

            # An older compatible HAT forgets its RAM-only native-position
            # decoder table on reboot even though the ST3215's persisted Mode-0
            # configuration survives.  Repair only a declared native STS servo
            # that PING proved present but fresh STATUS still omitted.  The
            # three read-only blocks prove the existing controller command needs
            # no EEPROM/configuration writes.  Its repeated torque-OFF write is
            # idempotent and can neither enable torque nor move the servo.
            ping_found_ids = set(found)
            telemetry_ids_after_family = {
                int(item["id"])
                for item in post_scan_telemetry
                if isinstance(item, dict)
                and isinstance(item.get("id"), int)
                and not isinstance(item.get("id"), bool)
                and minimum_id <= int(item["id"]) <= maximum_id
            }
            recovery_candidates = sorted(
                identifier
                for identifier in ping_found_ids - telemetry_ids_after_family
                if identifier in self._declared_native_multi_turn_servos
                and self._servo_families.get(identifier) == "STS"
                and "multi_turn_absolute_v1" in capabilities
            )
            recovery_attempted = False
            for identifier in recovery_candidates:
                # From the first REG_READ onward the controller/bus state may
                # have changed even when exact configuration proof fails or a
                # well-formed command is rejected.  Fence every such attempt
                # with one final same-boot STATUS before surfacing a successful
                # Scan so the returned aggregate state is never the pre-attempt
                # snapshot.
                recovery_attempted = True
                if not self._native_multi_turn_configuration_proven_locked(
                    identifier
                ):
                    continue
                self._restore_native_multi_turn_decoder_locked(identifier)
            if recovery_attempted:
                # This same-boot STATUS is the identity fence for every
                # well-formed attempt and, on success, the only evidence that
                # the RAM repair actually restored telemetry.
                status_after = self._require_current_boot_status_locked()
                post_scan_telemetry = status_after.get("servos")
                if not isinstance(post_scan_telemetry, list):
                    self._protocol_fault()
                    raise ControllerProtocolError(
                        "scan status omitted servo telemetry"
                    )

        post_scan_ids = {
            int(item["id"])
            for item in post_scan_telemetry
            if isinstance(item, dict)
            and isinstance(item.get("id"), int)
            and not isinstance(item.get("id"), bool)
            and minimum_id <= int(item["id"]) <= maximum_id
        }
        telemetry_unavailable_ids = sorted(ping_found_ids - post_scan_ids)
        telemetry_recovered_ids = sorted(
            (post_scan_ids - ping_found_ids)
            | (post_scan_ids - first_post_scan_ids)
        )
        found_ids = sorted(ping_found_ids | post_scan_ids)
        payload["pingFoundIds"] = sorted(ping_found_ids)
        payload["foundIds"] = found_ids
        payload["telemetryUnavailableIds"] = telemetry_unavailable_ids
        payload["telemetryRecoveredIds"] = telemetry_recovered_ids
        with self._state_lock:
            # PING and telemetry are separate time-bounded proofs. Preserve the
            # sorted union as presence evidence, while the guarded post-STATUS
            # remains the fresh aggregate state (including tracked IDs outside
            # a partial range); do not overwrite its count/state with the union.
            for identifier in tuple(self._telemetry):
                if minimum_id <= identifier <= maximum_id and identifier not in found_ids:
                    self._telemetry.pop(identifier, None)
            self._last_scan = {
                "minId": minimum_id,
                "maxId": maximum_id,
                "foundIds": list(found_ids),
                "pingFoundIds": sorted(ping_found_ids),
                "telemetryUnavailableIds": telemetry_unavailable_ids,
                "telemetryRecoveredIds": telemetry_recovered_ids,
                # Kept so the panel can explain an empty bus. The controller
                # reports a corrupt ping reply -- two servos sharing one id
                # answering together -- and dropping that here meant the only
                # place it ever surfaced was a scan's own HTTP response.
                "collisionSuspected": bool(payload.get("collisionSuspected")),
                "collisionId": payload.get("collisionId"),
                "busJammed": bool(payload.get("busJammed")),
                # The byte census behind a jam. "All zeroes" is a line held low
                # -- a power or wiring fault no protocol change can fix --
                # whereas framed wreckage is servos talking over each other.
                "busNoise": payload.get("busNoise"),
            }
        return payload

    def assign_id(self, old_id: int, new_id: int) -> dict[str, object]:
        self._validate_servo_id_argument(old_id)
        self._validate_servo_id_argument(new_id)
        if old_id == new_id:
            raise ValueError("new servo id must differ from old id")
        self._clear_prepared_proposals()
        payload, _ = self._command_with_identity_guard(
            "ASSIGN_ID", old_id, new_id, "SINGLE_SERVO"
        )
        if (
            payload.get("oldId") != old_id
            or payload.get("newId") != new_id
            or payload.get("verified") is not True
        ):
            self._protocol_fault()
            raise ControllerProtocolError("ID assignment was not verified")
        with self._state_lock:
            self._telemetry.clear()
            self._last_scan = None
            self._servo_count = None
            self._servos = "unknown"
        return payload

    def set_position_mode(self, servo_id: int) -> dict[str, object]:
        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "set_position_mode" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        self._clear_prepared_proposals()
        payload, status_after = self._command_with_identity_guard(
            "SET_POSITION_MODE", servo_id, "SINGLE_SERVO", "ST3215"
        )
        previous_mode = payload.get("previousOperatingMode")
        minimum_position = payload.get("minimumPosition")
        maximum_position = payload.get("maximumPosition")
        if (
            payload.get("servoId") != servo_id
            or isinstance(previous_mode, bool)
            or not isinstance(previous_mode, int)
            or not 0 <= previous_mode <= 3
            or payload.get("operatingMode") != 0
            or payload.get("verified") is not True
            or payload.get("locked") is not True
            or payload.get("torqueState") != "off"
            or isinstance(minimum_position, bool)
            or not isinstance(minimum_position, int)
            or isinstance(maximum_position, bool)
            or not isinstance(maximum_position, int)
            or not 0 <= minimum_position < maximum_position <= 4095
        ):
            self._protocol_fault()
            raise ControllerProtocolError("position mode restore was not verified")
        servos = status_after.get("servos")
        target = (
            next(
                (
                    item
                    for item in servos
                    if isinstance(item, dict) and item.get("id") == servo_id
                ),
                None,
            )
            if isinstance(servos, list)
            else None
        )
        if (
            not isinstance(target, dict)
            or target.get("operatingMode") != 0
            or target.get("torqueState") != "off"
            or target.get("errors") != []
        ):
            self._protocol_fault()
            raise ControllerProtocolError("position mode telemetry was not verified")
        with self._state_lock:
            self._last_scan = {
                "minId": 0,
                "maxId": 253,
                "foundIds": [servo_id],
            }
        return payload

    def capture(self, servo_id: int) -> dict[str, object]:
        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        self._clear_prepared_proposals()
        payload, _ = self._command_with_identity_guard("CAPTURE", servo_id)
        try:
            if payload.get("id") != servo_id:
                raise ControllerProtocolError("capture servo identity mismatch")
            _integer(
                payload.get("rawPosition"),
                minimum=0,
                maximum=4095,
                name="raw position",
            )
            sample_count = _integer(
                payload.get("sampleCount"),
                minimum=5,
                maximum=100,
                name="sample count",
            )
            if sample_count < 5 or payload.get("torqueState") != "off":
                raise ControllerProtocolError("invalid capture evidence")
            telemetry = self._validated_servo(payload)
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        with self._state_lock:
            self._telemetry[servo_id] = telemetry
            self._servo_count = len(self._telemetry)
            self._servos = "online"
        return payload

    def _validated_odometer(self, servo_id: int, payload: dict[str, object]) -> dict[str, object]:
        if payload.get("servoId") != servo_id:
            raise ControllerProtocolError("odometer servo identity mismatch")
        revolutions = _integer(
            payload.get("revolutions"),
            minimum=-ODOMETER_MAX_REVOLUTIONS,
            maximum=ODOMETER_MAX_REVOLUTIONS,
            name="revolutions",
        )
        raw_position = _integer(
            payload.get("rawPosition"), minimum=0, maximum=4095, name="raw position"
        )
        multi_turn = _integer(
            payload.get("multiTurnPosition"),
            minimum=-(ODOMETER_MAX_REVOLUTIONS + 1) * 4096,
            maximum=(ODOMETER_MAX_REVOLUTIONS + 1) * 4096,
            name="multi-turn position",
        )
        tracking = payload.get("tracking")
        valid = payload.get("valid")
        # Explicit even in Mode 0 so a mixed deployment cannot blur native
        # encoder truth with a retired Mode-3 countdown response.
        step_mode = payload.get("stepMode")
        if (
            not isinstance(tracking, bool)
            or not isinstance(valid, bool)
            or not isinstance(step_mode, bool)
        ):
            raise ControllerProtocolError("odometer flags must be booleans")
        if multi_turn != revolutions * 4096 + raw_position:
            raise ControllerProtocolError("odometer multi-turn position is inconsistent")
        if valid and not tracking:
            raise ControllerProtocolError("odometer cannot be valid while untracked")
        step_outstanding = payload.get("stepOutstanding", False)
        countdown_observed = payload.get("countdownObserved", False)
        resync_needed = payload.get("resyncNeeded", False)
        if not all(
            isinstance(value, bool)
            for value in (step_outstanding, countdown_observed, resync_needed)
        ):
            raise ControllerProtocolError("multi-turn truth flags must be booleans")
        resync_count = _integer(
            payload.get("resyncCount", 0),
            minimum=0,
            maximum=2_147_483_647,
            name="multi-turn resync count",
        )
        return {
            "servoId": servo_id,
            "tracking": tracking,
            "valid": valid,
            "stepMode": step_mode,
            "revolutions": revolutions,
            "rawPosition": raw_position,
            "multiTurnPosition": multi_turn,
            "sampleAgeMs": _integer(
                payload.get("sampleAgeMs"), minimum=0, maximum=86_400_000, name="sample age"
            ),
            "stepOutstanding": step_outstanding,
            "countdownObserved": countdown_observed,
            "resyncNeeded": resync_needed,
            "resyncCount": resync_count,
        }

    def read_servo_registers(self, servo_id: int, address: int, length: int) -> dict[str, object]:
        """Read any servo register. Diagnostic only; never energises anything."""

        self._validate_servo_id_argument(servo_id)
        if not isinstance(address, int) or isinstance(address, bool) or not 0 <= address <= 255:
            raise ValueError("register address must be between 0 and 255")
        if not isinstance(length, int) or isinstance(length, bool) or not 1 <= length <= 16:
            raise ValueError("register length must be between 1 and 16")
        payload, _ = self._command_with_identity_guard("REG_READ", servo_id, address, length)
        values = payload.get("values")
        if (
            payload.get("servoId") != servo_id
            or payload.get("address") != address
            or not isinstance(values, list)
            or len(values) != length
            or any(
                isinstance(item, bool) or not isinstance(item, int) or not 0 <= item <= 255
                for item in values
            )
        ):
            self._protocol_fault()
            raise ControllerProtocolError("register read was not verified")
        return {"servoId": servo_id, "address": address, "length": length, "values": values}

    def write_servo_registers(self, servo_id: int, address: int, values: list[int]) -> dict[str, object]:
        self._validate_servo_id_argument(servo_id)
        if not isinstance(address, int) or isinstance(address, bool) or not 0 <= address <= 255:
            raise ValueError("register address must be between 0 and 255")
        if (
            not isinstance(values, list)
            or not 1 <= len(values) <= MAX_REGISTER_WRITE_VALUES
            or any(
                isinstance(item, bool) or not isinstance(item, int) or not 0 <= item <= 255
                for item in values
            )
        ):
            raise ValueError(
                f"register values must be 1 to {MAX_REGISTER_WRITE_VALUES} bytes"
            )
        payload, _ = self._command_with_identity_guard("REG_WRITE", servo_id, address, *values)
        if payload.get("servoId") != servo_id or payload.get("written") != len(values):
            self._protocol_fault()
            raise ControllerProtocolError("register write was not verified")
        return payload

    def move(self, servo_id: int, goal: int, speed: int, acceleration: int) -> dict[str, object]:
        """Direct goal write. Requires a live torque lease on the controller, so a
        silent host still de-energises the joint."""

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        for name, value, high in (("goal", goal, 4095), ("speed", speed, 4095), ("acceleration", acceleration, 255)):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= high:
                raise ValueError(f"{name} must be between 0 and {high}")
        payload, _ = self._command_with_identity_guard("MOVE", servo_id, goal, speed, acceleration)
        if payload.get("servoId") != servo_id or payload.get("goal") != goal:
            self._protocol_fault()
            raise ControllerProtocolError("move was not acknowledged")
        return payload

    def move_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        """Dispatch 1..4 goals with one boot-bound A1 request.

        MOVE_SET deliberately does not use the ordinary STATUS/command/STATUS
        sandwich. The firmware receipt itself carries the complete HELLO boot
        identity. Each register dialect is sent in one SYNC_WRITE; a mixed set
        has a bounded two-packet gap and is explicitly not cross-family atomic.
        """

        normalized = _normalize_move_set(moves)
        self._require_declared_family_support(
            [servo_id for servo_id, _, _, _ in normalized]
        )
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "move_set_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        tokens = [
            f"{servo_id},{goal},{speed},{acceleration}"
            for servo_id, goal, speed, acceleration in normalized
        ]
        transmission_attempted = False
        try:
            self._require_operation_permitted("MOVE_SET")
            with self._state_lock:
                if self._connection != "online":
                    raise ControllerUnavailableError("controller unavailable")
            with self._io_lock:
                transmission_before = self._transmission_attempt
                try:
                    payload = self._command_locked("MOVE_SET", *tokens)
                finally:
                    transmission_attempted = (
                        self._transmission_attempt != transmission_before
                    )
        except ControllerCommandError as error:
            if error.code == "MOVE_SET_FAILED":
                failure = error.payload
                stopped = failure.get("stopped")
                if (
                    not isinstance(stopped, bool)
                    or failure.get("latentCommands") is not False
                    or failure.get("motionMayHaveStarted") is not True
                    or failure.get("partialDispatchPossible") is not True
                    or failure.get("phase") not in {"dispatch", "verify", "feedback"}
                    or failure.get("torqueState") not in {"off", "unknown"}
                    or isinstance(failure.get("failedIndex"), bool)
                    or not isinstance(failure.get("failedIndex"), int)
                    or not 0 <= int(failure["failedIndex"]) < len(normalized)
                    or isinstance(failure.get("dispatchedFamilyCount"), bool)
                    or not isinstance(failure.get("dispatchedFamilyCount"), int)
                    or not 0 <= int(failure["dispatchedFamilyCount"]) <= 2
                    or (
                        failure.get("phase") == "dispatch"
                        and (
                            int(failure["failedIndex"]) != 0
                            or int(failure["dispatchedFamilyCount"]) > 1
                        )
                    )
                    or (
                        failure.get("phase") in {"verify", "feedback"}
                        and int(failure["dispatchedFamilyCount"]) < 1
                    )
                ):
                    self._protocol_fault()
                    if transmission_attempted:
                        self._mark_move_set_unconfirmed()
                    raise ControllerProtocolError(
                        "invalid move set failure receipt"
                    ) from error
                # New firmware reports a recoverable authority revocation. An
                # older compatible build may still report the physical latch it
                # created; reflect that legacy fact so the service can apply its
                # narrowly proved automatic-reset policy instead of hiding it.
                with self._state_lock:
                    self._prepared_proposals.clear()
                    self._active_hold_ids.clear()
                    if stopped:
                        self._operator_inspection_required = True
                        self._safety_stop_reason = "MOVE_SET_FAILED"
                    self._motion_state = "stopped" if stopped else "blocked"
                    self._torque_state = "unknown"
                    self._last_motion_failure = {
                        key: failure[key]
                        for key in (
                            "phase",
                            "failedIndex",
                            "torqueState",
                            "latentCommands",
                            "motionMayHaveStarted",
                            "partialDispatchPossible",
                            "dispatchedFamilyCount",
                        )
                        if key in failure
                    }
            elif error.code not in _MOVE_SET_PREFLIGHT_ERRORS:
                self._protocol_fault()
                if transmission_attempted:
                    self._mark_move_set_unconfirmed()
                raise ControllerProtocolError(
                    "unexpected move set error receipt"
                ) from error
            raise
        except (
            ControllerProtocolError,
            ControllerTransportError,
            ControllerUnavailableError,
        ):
            if transmission_attempted:
                self._mark_move_set_unconfirmed()
            raise
        try:
            self._validate_response_identity(payload, required=True)
            moved = payload.get("moved")
            if (
                payload.get("dispatch") != "dialect_grouped_sync_write"
                or payload.get("crossFamilyAtomic") is not False
            ):
                raise ControllerProtocolError("invalid move set dispatch receipt")
            if (
                isinstance(payload.get("count"), bool)
                or payload.get("count") != len(normalized)
                or not isinstance(moved, list)
                or len(moved) != len(normalized)
            ):
                raise ControllerProtocolError("move set was not acknowledged")
            for expected, candidate in zip(normalized, moved, strict=True):
                if not isinstance(candidate, dict):
                    raise ControllerProtocolError("move set was not acknowledged")
                actual = (
                    _integer(candidate.get("servoId"), minimum=0, maximum=253,
                             name="move set servo id"),
                    _integer(candidate.get("goal"), minimum=-MULTI_TURN_GOAL_LIMIT,
                             maximum=MULTI_TURN_GOAL_LIMIT, name="move set goal"),
                    _integer(candidate.get("speed"), minimum=1, maximum=4095,
                             name="move set speed"),
                    _integer(candidate.get("acceleration"), minimum=1, maximum=255,
                             name="move set acceleration"),
                )
                if actual != expected:
                    raise ControllerProtocolError("move set was not acknowledged")
        except ControllerProtocolError:
            self._protocol_fault()
            self._mark_move_set_unconfirmed()
            raise
        return dict(payload)

    def follow_set(self, moves: list[MoveSetTuple]) -> dict[str, object]:
        """Dispatch exactly two STS goals and ingest their fresh feedback."""

        normalized = _normalize_follow_set(moves)
        servo_ids = [servo_id for servo_id, _, _, _ in normalized]
        self._require_declared_family_support(servo_ids)
        with self._io_lock:
            if any(self._servo_families.get(servo_id, "STS") != "STS"
                   for servo_id in servo_ids):
                raise ControllerCommandError("UNSUPPORTED")
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "follow_set_feedback_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        tokens = [
            f"{servo_id},{goal},{speed},{acceleration}"
            for servo_id, goal, speed, acceleration in normalized
        ]
        transmission_attempted = False
        try:
            self._require_operation_permitted("FOLLOW_SET")
            with self._state_lock:
                if self._connection != "online":
                    raise ControllerUnavailableError("controller unavailable")
            with self._io_lock:
                transmission_before = self._transmission_attempt
                try:
                    payload = self._command_locked("FOLLOW_SET", *tokens)
                finally:
                    transmission_attempted = (
                        self._transmission_attempt != transmission_before
                    )
        except ControllerCommandError as error:
            if error.code == "MOVE_SET_FAILED":
                failure = error.payload
                stopped = failure.get("stopped")
                if (
                    not isinstance(stopped, bool)
                    or failure.get("latentCommands") is not False
                    or failure.get("motionMayHaveStarted") is not True
                    or failure.get("partialDispatchPossible") is not True
                    or failure.get("phase") not in {"dispatch", "verify", "feedback"}
                    or failure.get("torqueState") not in {"off", "unknown"}
                    or isinstance(failure.get("failedIndex"), bool)
                    or not isinstance(failure.get("failedIndex"), int)
                    or not 0 <= int(failure["failedIndex"]) < 2
                    or isinstance(failure.get("dispatchedFamilyCount"), bool)
                    or not isinstance(failure.get("dispatchedFamilyCount"), int)
                    or not 0 <= int(failure["dispatchedFamilyCount"]) <= 1
                    or (
                        failure.get("phase") == "dispatch"
                        and (
                            int(failure["failedIndex"]) != 0
                            or int(failure["dispatchedFamilyCount"]) != 0
                        )
                    )
                    or (
                        failure.get("phase") in {"verify", "feedback"}
                        and int(failure["dispatchedFamilyCount"]) != 1
                    )
                ):
                    self._protocol_fault()
                    if transmission_attempted:
                        self._mark_move_set_unconfirmed()
                    raise ControllerProtocolError(
                        "invalid follow set failure receipt"
                    ) from error
                with self._state_lock:
                    self._prepared_proposals.clear()
                    self._active_hold_ids.clear()
                    if stopped:
                        self._operator_inspection_required = True
                        self._safety_stop_reason = "MOVE_SET_FAILED"
                    self._motion_state = "stopped" if stopped else "blocked"
                    self._torque_state = "unknown"
                    self._last_motion_failure = {
                        key: failure[key]
                        for key in (
                            "phase",
                            "failedIndex",
                            "stopped",
                            "torqueState",
                            "latentCommands",
                            "motionMayHaveStarted",
                            "partialDispatchPossible",
                            "dispatchedFamilyCount",
                        )
                        if key in failure
                    }
            elif error.code not in _MOVE_SET_PREFLIGHT_ERRORS:
                self._protocol_fault()
                if transmission_attempted:
                    self._mark_move_set_unconfirmed()
                raise ControllerProtocolError(
                    "unexpected follow set error receipt"
                ) from error
            raise
        except (
            ControllerProtocolError,
            ControllerTransportError,
            ControllerUnavailableError,
        ):
            if transmission_attempted:
                self._mark_move_set_unconfirmed()
            raise

        try:
            self._validate_response_identity(payload, required=True)
            moved = payload.get("moved")
            feedback = payload.get("feedback")
            if (
                payload.get("dispatch") != "dialect_grouped_sync_write"
                or payload.get("crossFamilyAtomic") is not False
                or payload.get("count") != 2
                or isinstance(payload.get("count"), bool)
                or not isinstance(moved, list)
                or len(moved) != 2
                or not isinstance(feedback, list)
                or len(feedback) != 2
            ):
                raise ControllerProtocolError("follow set was not acknowledged")

            validated_feedback: list[dict[str, object]] = []
            for expected, candidate, feedback_candidate in zip(
                normalized, moved, feedback, strict=True
            ):
                if not isinstance(candidate, dict) or not isinstance(
                    feedback_candidate, dict
                ):
                    raise ControllerProtocolError("follow set was not acknowledged")
                actual = (
                    _integer(candidate.get("servoId"), minimum=0, maximum=253,
                             name="follow set servo id"),
                    _integer(candidate.get("goal"), minimum=-MULTI_TURN_GOAL_LIMIT,
                             maximum=MULTI_TURN_GOAL_LIMIT,
                             name="follow set goal"),
                    _integer(candidate.get("speed"), minimum=1,
                             maximum=MOTION_POLICY["MAX_SPEED"],
                             name="follow set speed"),
                    _integer(candidate.get("acceleration"), minimum=1,
                             maximum=MOTION_POLICY["MAX_ACCEL"],
                             name="follow set acceleration"),
                )
                if actual != expected:
                    raise ControllerProtocolError("follow set was not acknowledged")
                feedback_servo_id = _integer(
                    feedback_candidate.get("servoId"), minimum=0, maximum=253,
                    name="follow feedback servo id"
                )
                if feedback_servo_id != expected[0]:
                    raise ControllerProtocolError("follow feedback did not match request")
                if not isinstance(feedback_candidate.get("moving"), bool):
                    raise ControllerProtocolError("invalid follow feedback moving flag")
                validated_feedback.append(
                    {
                        "servoId": feedback_servo_id,
                        "rawPosition": _integer(
                            feedback_candidate.get("rawPosition"), minimum=0,
                            maximum=4095, name="follow feedback position"
                        ),
                        "moving": feedback_candidate["moving"],
                        "packetAgeMs": _integer(
                            feedback_candidate.get("packetAgeMs"), minimum=0,
                            maximum=86_400_000, name="follow feedback age"
                        ),
                        "voltageDeciVolts": _integer(
                            feedback_candidate.get("voltageDeciVolts"), minimum=0,
                            maximum=255, name="follow feedback voltage"
                        ),
                        "temperatureC": _integer(
                            feedback_candidate.get("temperatureC"), minimum=0,
                            maximum=150, name="follow feedback temperature"
                        ),
                    }
                )
        except ControllerProtocolError:
            self._protocol_fault()
            self._mark_move_set_unconfirmed()
            raise

        with self._state_lock:
            for row in validated_feedback:
                servo_id = int(row["servoId"])
                telemetry = deepcopy(self._telemetry.get(servo_id, {}))
                telemetry.update(
                    {
                        "id": servo_id,
                        "rawPosition": row["rawPosition"],
                        "speed": telemetry.get("speed", 0),
                        "load": telemetry.get("load", 0),
                        "voltageVolts": int(row["voltageDeciVolts"]) / 10.0,
                        "temperatureC": row["temperatureC"],
                        "moving": row["moving"],
                        "torqueState": "on",
                        "packetAgeMs": row["packetAgeMs"],
                        "errors": [],
                        "operatingMode": 0,
                        "online": True,
                        "fresh": True,
                        "statusError": 0,
                    }
                )
                self._telemetry[servo_id] = telemetry
            if self._servo_count is None:
                self._servo_count = len(self._telemetry)
            self._bus = "online"
            self._torque_state = "on"
            self._motion_state = (
                "moving" if any(bool(row["moving"]) for row in validated_feedback)
                else "ready"
            )
        return dict(payload)

    def follow_read(self, servo_ids: list[int]) -> dict[str, object]:
        """Read an ordered STS pair without re-dispatching its current goal."""

        normalized = _normalize_follow_read(servo_ids)
        self._require_declared_family_support(normalized)
        with self._io_lock:
            if any(
                self._servo_families.get(servo_id, "STS") != "STS"
                for servo_id in normalized
            ):
                raise ControllerCommandError("UNSUPPORTED")
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
            online = self._connection == "online"
        if "follow_feedback_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        if not online:
            raise ControllerUnavailableError("controller unavailable")

        self._require_operation_permitted("FOLLOW_READ")
        try:
            with self._io_lock:
                payload = self._command_locked("FOLLOW_READ", *normalized)
        except ControllerCommandError as error:
            if error.code not in _FOLLOW_READ_ERRORS:
                self._protocol_fault()
                raise ControllerProtocolError(
                    "unexpected follow read error receipt"
                ) from error
            raise

        try:
            self._validate_response_identity(payload, required=True)
            if (
                isinstance(payload.get("count"), bool)
                or payload.get("count") != 2
            ):
                raise ControllerProtocolError("follow read was not acknowledged")
            feedback = self._validated_follow_feedback(
                payload.get("feedback"), normalized
            )
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        self._ingest_follow_feedback(feedback)
        return dict(payload)

    def set_multi_turn(self, servo_id: int, enabled: bool) -> dict[str, object]:
        """Switch a servo between single-turn and multi-turn.

        The controller owns this rather than the gateway poking angle limits
        itself: it has to change the servo AND its own goal encoding AND drop
        any hold captured under the old convention, and those three must not be
        able to disagree. Mode and limits are EPROM-backed; the controller reads
        them back every time rather than assuming which values survived a reset.
        """

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "multi_turn_absolute_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        payload, _ = self._command_with_identity_guard(
            "MULTITURN", servo_id, "ON" if enabled else "OFF"
        )
        if (
            payload.get("servoId") != servo_id
            or payload.get("multiTurn") is not enabled
            or payload.get("operatingMode") != 0
        ):
            self._protocol_fault()
            raise ControllerProtocolError("multi-turn was not acknowledged")
        return dict(payload)

    def move_multi_turn(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        """Drive to a goal that may lie outside one turn.

        MOVE cannot express this: it bounds the goal to 0..4095 because that is
        all a single-turn servo can name. Here the goal is sign-magnitude across
        +/-30719 and goes straight into the goal block.

        Goes through MOVE, not raw register writes, so it keeps every check MOVE
        carries: STOP, a fresh heartbeat, and live torque authority for this
        servo. Driving the goal block directly would have bypassed all three.
        """

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "multi_turn_absolute_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        for name, value, low, high in (
            ("goal", goal, -MULTI_TURN_GOAL_LIMIT, MULTI_TURN_GOAL_LIMIT),
            ("speed", speed, 1, 4095),
            ("acceleration", acceleration, 1, 255),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f"{name} must be between {low} and {high}")
        payload, _ = self._command_with_identity_guard(
            "MOVE", servo_id, goal, speed, acceleration
        )
        if payload.get("servoId") != servo_id or payload.get("goal") != goal:
            self._protocol_fault()
            raise ControllerProtocolError("move was not acknowledged")
        return dict(payload)

    def odometer_zero(self, servo_id: int) -> dict[str, object]:
        """Arm tracking while preserving the servo's signed current coordinate."""

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        payload, _ = self._command_with_identity_guard("ODO_ZERO", servo_id)
        try:
            state = self._validated_odometer(servo_id, payload)
            if not state["tracking"] or not state["valid"]:
                raise ControllerProtocolError("odometer did not arm at zero")
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        return state

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        """Read the wrap-counted position. Never moves anything."""

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        payload, _ = self._command_with_identity_guard("ODO_READ", servo_id)
        try:
            return self._validated_odometer(servo_id, payload)
        except ControllerProtocolError:
            self._protocol_fault()
            raise

    def home_multi_turn(self, servo_id: int) -> dict[str, object]:
        """Configure native Mode-0 multi-turn and capture the current Base zero."""

        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "multi_turn_absolute_v1" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")
        with self._io_lock:
            self._require_current_boot_status_locked()
            try:
                enabled = self._command("MULTITURN", servo_id, "ON")
                if (
                    enabled.get("servoId") != servo_id
                    or enabled.get("multiTurn") is not True
                    or enabled.get("operatingMode") != 0
                ):
                    raise ControllerProtocolError(
                        "native absolute multi-turn mode was not acknowledged"
                    )

                zeroed = self._validated_odometer(
                    servo_id, self._command("ODO_ZERO", servo_id)
                )
                if (
                    zeroed["tracking"] is not True
                    or zeroed["valid"] is not True
                    or zeroed["stepMode"] is not False
                ):
                    raise ControllerProtocolError("odometer did not arm at physical zero")

                state = self._validated_odometer(
                    servo_id, self._command("ODO_READ", servo_id)
                )
                if state["tracking"] is not True or state["valid"] is not True:
                    raise ControllerProtocolError("odometer frame was not retained")
                if state["stepMode"] is not False:
                    raise ControllerProtocolError("Base did not remain in encoder mode")
                self._require_current_boot_status_locked()
                return state
            except ControllerProtocolError:
                self._protocol_fault()
                raise

    def torque_lease(self, servo_id: int, lease_ms: int) -> dict[str, object]:
        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        if (
            isinstance(lease_ms, bool)
            or not isinstance(lease_ms, int)
            or not 100 <= lease_ms <= 2_000
        ):
            raise ValueError("torque lease must be between 100 and 2000 ms")
        self._clear_prepared_proposals()
        payload, status_after = self._command_with_identity_guard(
            "TORQUE_LEASE", servo_id, lease_ms
        )
        if (
            payload.get("servoId") != servo_id
            or payload.get("torqueState") != "on"
            or payload.get("leaseMs") != lease_ms
            or payload.get("confirmed") is not True
        ):
            self._protocol_fault()
            raise ControllerProtocolError("torque lease was not verified")
        raw_servos = status_after.get("servos")
        raw_target = (
            next(
                (
                    item
                    for item in raw_servos
                    if isinstance(item, dict) and item.get("id") == servo_id
                ),
                None,
            )
            if isinstance(raw_servos, list)
            else None
        )
        with self._state_lock:
            target = self._telemetry.get(servo_id)
            post_state_verified = (
                self._torque_state == "on"
                and target is not None
                and target.get("torqueState") == "on"
                and target.get("operatingMode") in HOLDING_MODES
                and target.get("errors") == []
                and isinstance(target.get("packetAgeMs"), int)
                and int(target["packetAgeMs"]) <= 250
            )
        post_state_verified = post_state_verified and (
            isinstance(raw_target, dict)
            and raw_target.get("online") is True
            and raw_target.get("fresh") is True
            and raw_target.get("statusError") == 0
        )
        if not post_state_verified:
            self._protocol_fault()
            raise ControllerProtocolError("torque lease was not verified")
        return payload

    def set_hold_servos(
        self, servo_ids: list[int], lease_ms: int = 1_500
    ) -> dict[str, object]:
        if not isinstance(servo_ids, list):
            raise ValueError("servo ids must be a list")
        if len(servo_ids) > MAX_HOLD_SERVOS:
            raise ValueError(f"at most {MAX_HOLD_SERVOS} servos can be held")
        for servo_id in servo_ids:
            self._validate_servo_id_argument(servo_id)
        if len(set(servo_ids)) != len(servo_ids):
            raise ValueError("held servo ids must be unique")
        self._require_declared_family_support(servo_ids)
        if (
            isinstance(lease_ms, bool)
            or not isinstance(lease_ms, int)
            or not 100 <= lease_ms <= 2_000
        ):
            raise ValueError("hold lease must be between 100 and 2000 ms")
        with self._state_lock:
            capabilities = (
                self._identity.get("capabilities", [])
                if isinstance(self._identity, dict)
                else []
            )
        if "hold_set" not in capabilities:
            raise ControllerCommandError("UNSUPPORTED")

        self._clear_prepared_proposals()
        payload, status_after = self._command_with_identity_guard(
            "HOLD_SET", lease_ms, *servo_ids
        )
        holds = payload.get("holds")
        if (
            payload.get("servoIds") != servo_ids
            or payload.get("leaseMs") != lease_ms
            or payload.get("confirmed") is not True
            or not isinstance(holds, list)
            or len(holds) != len(servo_ids)
        ):
            self._protocol_fault()
            raise ControllerProtocolError("hold set was not verified")
        hold_positions: dict[int, int] = {}
        try:
            for hold in holds:
                if not isinstance(hold, dict):
                    raise ControllerProtocolError("hold set was not verified")
                identifier = _integer(
                    hold.get("servoId"),
                    minimum=0,
                    maximum=253,
                    name="held servo id",
                )
                # The captured hold is expressed in the servo's own GOAL frame,
                # which for a multi-turn joint is the wrap-counted position and
                # is legitimately negative or past one turn. Bounding it to a
                # single turn faulted the link on every hold once the base left
                # the 0..4095 window -- and because holds are renewed
                # continuously, that made the arm unusable past about 90 deg.
                # The honest bound is the goal range the servo itself accepts;
                # the controller already range-checks per servo.
                position = _integer(
                    hold.get("holdRawPosition"),
                    minimum=-MULTI_TURN_GOAL_LIMIT,
                    maximum=MULTI_TURN_GOAL_LIMIT,
                    name="hold raw position",
                )
                if identifier in hold_positions:
                    raise ControllerProtocolError("hold set was not verified")
                hold_positions[identifier] = position
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        if list(hold_positions) != servo_ids:
            self._protocol_fault()
            raise ControllerProtocolError("hold set was not verified")

        raw_hold_set = status_after.get("holdSet")
        if servo_ids:
            status_verified = (
                isinstance(raw_hold_set, dict)
                and raw_hold_set.get("servoIds") == servo_ids
                and isinstance(raw_hold_set.get("remainingMs"), int)
                and not isinstance(raw_hold_set.get("remainingMs"), bool)
                and 1 <= int(raw_hold_set["remainingMs"]) <= lease_ms
            )
        else:
            status_verified = raw_hold_set is None
        raw_servos = status_after.get("servos")
        telemetry_by_id = (
            {
                int(item["id"]): item
                for item in raw_servos
                if isinstance(item, dict)
                and isinstance(item.get("id"), int)
                and not isinstance(item.get("id"), bool)
            }
            if isinstance(raw_servos, list)
            else {}
        )
        status_verified = status_verified and all(
            identifier in telemetry_by_id
            and telemetry_by_id[identifier].get("torqueState") == "on"
            and telemetry_by_id[identifier].get("operatingMode") in HOLDING_MODES
            and telemetry_by_id[identifier].get("errors") == []
            for identifier in servo_ids
        )
        if not status_verified:
            self._protocol_fault()
            raise ControllerProtocolError("hold set was not verified")
        return payload

    def torque_off(self, servo_id: int | None = None) -> dict[str, object]:
        if servo_id is not None:
            self._validate_servo_id_argument(servo_id)
        self._clear_prepared_proposals()
        payload, status_after = self._command_with_identity_guard(
            "TORQUE_OFF", "ALL" if servo_id is None else servo_id
        )
        expected_all = servo_id is None
        if (
            payload.get("torqueState") != "off"
            or payload.get("confirmed") is not True
            or payload.get("all") is not expected_all
            or payload.get("servoId") != servo_id
        ):
            self._protocol_fault()
            raise ControllerProtocolError("torque off was not verified")
        with self._state_lock:
            if servo_id is None:
                verified_post_state = (
                    isinstance(self._servo_count, int)
                    and self._servo_count > 0
                    and len(self._telemetry) == self._servo_count
                    and all(
                        item.get("torqueState") == "off"
                        for item in self._telemetry.values()
                    )
                )
            else:
                target = self._telemetry.get(servo_id)
                verified_post_state = (
                    target is not None
                    and target.get("torqueState") == "off"
                    and isinstance(status_after.get("servos"), list)
                )
                # status_after was already ingested by the identity guard. It
                # is the authoritative aggregate: another servo can remain on.
        if not verified_post_state:
            self._protocol_fault()
            raise ControllerProtocolError("torque off was not verified")
        if servo_id is None:
            with self._state_lock:
                self._torque_state = "off"
        return payload

    def prepare_nudge(
        self,
        servo_id: int,
        delta_ticks: int,
        speed: int,
        acceleration: int,
    ) -> dict[str, object]:
        self._validate_servo_id_argument(servo_id)
        self._require_declared_family_support([servo_id])
        if (
            isinstance(delta_ticks, bool)
            or not isinstance(delta_ticks, int)
            or delta_ticks == 0
            or not -64 <= delta_ticks <= 64
            or isinstance(speed, bool)
            or not isinstance(speed, int)
            or not 1 <= speed <= 256
            or isinstance(acceleration, bool)
            or not isinstance(acceleration, int)
            or not 1 <= acceleration <= 20
        ):
            raise ValueError("nudge parameters exceed bounded commissioning limits")
        self._clear_prepared_proposals()
        payload, _ = self._command_with_identity_guard(
            "PREPARE_NUDGE", servo_id, delta_ticks, speed, acceleration
        )
        proposal_id = payload.get("proposalId")
        try:
            if (
                not isinstance(proposal_id, str)
                or _SAFE_PROPOSAL.fullmatch(proposal_id) is None
            ):
                raise ControllerProtocolError("invalid nudge proposal")
            start = _integer(
                payload.get("startRawPosition"),
                minimum=0,
                maximum=4095,
                name="nudge start position",
            )
            target = _integer(
                payload.get("targetRawPosition"),
                minimum=0,
                maximum=4095,
                name="nudge target position",
            )
            if (
                payload.get("servoId") != servo_id
                or payload.get("deltaTicks") != delta_ticks
                or payload.get("speed") != speed
                or payload.get("acceleration") != acceleration
                or target - start != delta_ticks
            ):
                raise ControllerProtocolError("invalid nudge proposal")
            expires_ms = _integer(
                payload.get("expiresInMs"),
                minimum=1,
                maximum=30_000,
                name="nudge proposal expiry",
            )
            proposal_hash = payload.get("proposalHash")
            if proposal_hash is not None and (
                not isinstance(proposal_hash, str)
                or re.fullmatch(r"sha256:[0-9a-f]{64}", proposal_hash) is None
            ):
                raise ControllerProtocolError("invalid nudge proposal hash")
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        with self._state_lock:
            self._prepared_proposals.clear()
            self._prepared_proposals[proposal_id] = {
                "proposalId": proposal_id,
                "proposalHash": proposal_hash,
                "servoId": servo_id,
                "deltaTicks": delta_ticks,
                "startRawPosition": start,
                "targetRawPosition": target,
                "expiresAt": time.monotonic() + (expires_ms / 1000.0),
            }
        return payload

    def execute_nudge(
        self, proposal_id: str, proposal_hash: str | None = None
    ) -> dict[str, object]:
        if not isinstance(proposal_id, str) or _SAFE_PROPOSAL.fullmatch(proposal_id) is None:
            raise ValueError("proposal id is invalid")
        if proposal_hash is not None and re.fullmatch(r"sha256:[0-9a-f]{64}", proposal_hash) is None:
            raise ValueError("proposal hash is invalid")
        with self._state_lock:
            proposal = self._prepared_proposals.pop(proposal_id, None)
        if proposal is None:
            raise ControllerCommandError("PROPOSAL_USED_OR_UNKNOWN")
        self._require_declared_family_support([int(proposal["servoId"])])
        if time.monotonic() >= float(proposal["expiresAt"]):
            raise ControllerCommandError("PROPOSAL_EXPIRED")
        stored_hash = proposal.get("proposalHash")
        if (
            proposal_hash is not None
            and isinstance(stored_hash, str)
            and not secrets.compare_digest(proposal_hash, stored_hash)
        ):
            raise ControllerCommandError("PROPOSAL_HASH_MISMATCH")
        # The firmware proposal ID is already an opaque digest-bound capability.
        # proposalHash is checked by the API/controller response when supplied,
        # but never changes the wire command agreed with firmware.
        payload, _ = self._command_with_identity_guard("EXECUTE_NUDGE", proposal_id)
        try:
            start = _integer(
                payload.get("startRawPosition"),
                minimum=0,
                maximum=4095,
                name="nudge start position",
            )
            target = _integer(
                payload.get("targetRawPosition"),
                minimum=0,
                maximum=4095,
                name="nudge target position",
            )
            final = _integer(
                payload.get("rawPosition"),
                minimum=0,
                maximum=4095,
                name="nudge final position",
            )
            measured = payload.get("measuredDeltaTicks")
            position_error = payload.get("positionErrorTicks")
            evidence_id = payload.get("evidenceId")
            if (
                isinstance(measured, bool)
                or not isinstance(measured, int)
                or not -2048 <= measured <= 2047
                or isinstance(position_error, bool)
                or not isinstance(position_error, int)
                or not -2048 <= position_error <= 2047
                or not isinstance(evidence_id, str)
                or _SAFE_PROPOSAL.fullmatch(evidence_id) is None
            ):
                raise ControllerProtocolError("invalid nudge completion evidence")
            calculated_delta = final - start
            calculated_error = final - target
            expected_delta = int(proposal["deltaTicks"])
            same_direction = (calculated_delta > 0) == (expected_delta > 0)
            if (
                payload.get("proposalId") != proposal_id
                or payload.get("servoId") != proposal["servoId"]
                or payload.get("completed") is not True
                or start != proposal["startRawPosition"]
                or target != proposal["targetRawPosition"]
                or measured != calculated_delta
                or position_error != calculated_error
                or not same_direction
                or abs(calculated_delta - expected_delta) > 8
                or abs(calculated_error) > 8
                or payload.get("torqueState") != "off"
            ):
                raise ControllerProtocolError("nudge completion was not safely verified")
        except ControllerProtocolError:
            self._protocol_fault()
            raise
        if proposal_hash is not None and payload.get("proposalHash", proposal_hash) != proposal_hash:
            self._protocol_fault()
            raise ControllerProtocolError("nudge proposal hash mismatch")
        return payload

    def _clear_prepared_proposals(self) -> None:
        with self._state_lock:
            self._prepared_proposals.clear()

    @staticmethod
    def _validate_servo_id_argument(servo_id: object) -> None:
        if (
            isinstance(servo_id, bool)
            or not isinstance(servo_id, int)
            or not 0 <= servo_id <= 253
        ):
            raise ValueError("servo id must be between 0 and 253")

    def stop(self) -> dict[str, object]:
        self._clear_prepared_proposals()
        payload = self._command("STOP")
        if not self._stop_receipt_is_consistent(payload):
            self._protocol_fault()
            raise ControllerProtocolError("STOP was not safely verified")
        torque_state = payload.get("torqueState")
        self._ingest_status({**payload, "motionState": "stopped"})
        with self._state_lock:
            # stopped is a controller latch. It blocks motion even when the
            # actuator electrical state could not be read back.
            if not self._operator_inspection_required:
                self._operator_inspection_required = True
                self._safety_stop_reason = "EXPLICIT_STOP"
            self._motion_state = "stopped"
            self._torque_state = str(torque_state)
        return payload

    @staticmethod
    def _stop_receipt_is_consistent(payload: dict[str, object]) -> bool:
        torque_state = payload.get("torqueState")
        confirmed = payload.get("confirmed")
        broadcast_sent = payload.get("torqueOffBroadcastSent")
        stop_evidence_is_consistent = (
            confirmed is True
            and torque_state == "off"
            and isinstance(broadcast_sent, bool)
        ) or (
            confirmed is False
            and torque_state == "unknown"
            and isinstance(broadcast_sent, bool)
        )
        return payload.get("stopped") is True and stop_evidence_is_consistent

    def reset(self, *, inspected: bool = False) -> dict[str, object]:
        with self._state_lock:
            inspection_required = self._operator_inspection_required
            if inspection_required and not inspected:
                raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
        self._clear_prepared_proposals()
        self._require_operation_permitted("RESET")
        with self._io_lock:
            status_before = self._require_current_boot_status_locked()
            controller_inspection_required = (
                status_before.get("operatorInspectionRequired") is True
            )
            controller_stop_latched = (
                controller_inspection_required
                or status_before.get("stopped") is True
                or status_before.get("motionState") == "stopped"
            )
            with self._state_lock:
                if controller_stop_latched and not self._operator_inspection_required:
                    # Old compatible firmware may omit the additive reason
                    # tuple. A same-boot STOP/fault is still never permission
                    # for an automatic RESET INSPECTED.
                    self._operator_inspection_required = True
                    self._safety_stop_reason = (
                        str(status_before.get("safetyStopReason"))
                        if status_before.get("safetyStopReason")
                        in _CONTROLLER_STOP_REASONS
                        else "SAFETY_FAULT"
                    )
                inspection_required = self._operator_inspection_required
            if inspection_required and not inspected:
                # The same-boot STATUS above may be the first observation of a
                # MOVE_SET failure, including when RESET queued behind it. No
                # RESET token may cross the wire without explicit inspection.
                raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
            if inspection_required and inspected and not controller_inspection_required:
                # The previous RESET may have succeeded on the HAT while its OK
                # or post-STATUS was lost. The Pi must retain its local latch,
                # so rebuild a controller STOP with fresh same-boot proof before
                # retrying RESET. This is idempotent and sends no torque-enable
                # or motion command.
                stop_payload = self._command("STOP")
                if not self._stop_receipt_is_consistent(stop_payload):
                    self._protocol_fault()
                    raise ControllerProtocolError("STOP was not safely verified")
                self._ingest_status({**stop_payload, "motionState": "stopped"})
                self._require_current_boot_status_locked()
            if inspection_required:
                # Family byte order/addressing is safety-critical to MOVE_SET.
                # Replay deferred calibration intent while the HAT is still
                # physically STOP-latched, so a rejected or malformed FAMILY
                # receipt can never leave an unlatched controller behind.
                with self._state_lock:
                    identity = deepcopy(self._identity)
                if isinstance(identity, dict):
                    self._apply_servo_families_locked(identity)
            payload = self._command("RESET", "INSPECTED")
            self._require_current_boot_status_locked()
        reset_receipt_is_exact = (
            payload.get("stopped") is False
            and payload.get("torqueState") == "off"
            and payload.get("torqueOffConfirmed") is True
            and payload.get("reset") is True
        )
        if not reset_receipt_is_exact:
            self._protocol_fault()
            raise ControllerProtocolError("reset did not remain disarmed")
        self._ingest_status({**payload, "motionState": "blocked"})
        # A reconnect performed under the inspection latch deliberately skipped
        # mutable runtime policy. Restore it while the host-side gate is still
        # closed. Family declarations were already replayed before RESET so a
        # family failure could not clear the durable HAT STOP latch.
        if inspection_required:
            with self._state_lock:
                identity = deepcopy(self._identity)
            if isinstance(identity, dict):
                with self._io_lock:
                    self._apply_motion_policy_locked(identity)
        with self._state_lock:
            self._torque_state = "off"
            self._operator_inspection_required = False
            self._safety_stop_reason = None
            self._last_motion_failure = None
            self._motion_state = "blocked"
        return payload


__all__ = [
    "DEFAULT_BAUDRATE",
    "DEFAULT_TIMEOUT_SECONDS",
    "HEARTBEAT_INTERVAL_SECONDS",
    "MAX_REQUEST_BYTES",
    "MAX_RESPONSE_BYTES",
    "MOTION_POLICY",
    "ODOMETER_MAX_REVOLUTIONS",
    "OPERATION_TIMEOUT_SECONDS",
    "PROTOCOL_TAG",
    "PROTOCOL_VERSION",
    "SerialArmController",
]
