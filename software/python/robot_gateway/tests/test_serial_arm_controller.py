from __future__ import annotations

import json
import threading
import time
from copy import deepcopy

import pytest

from robot_gateway.arm_controller import (
    ControllerCommandError,
    ControllerProtocolError,
    ControllerTransportError,
    ReplayArmController,
)
from robot_gateway.serial_arm_controller import SerialArmController


class ScriptedSerial:
    def __init__(self, replies: list[bytes]) -> None:
        self.replies = replies
        self.writes: list[bytes] = []
        self.is_open = True
        self.input_resets = 0

    def write(self, payload: bytes) -> int:
        self.writes.append(payload)
        return len(payload)

    def flush(self) -> None:
        return None

    def reset_input_buffer(self) -> None:
        self.input_resets += 1

    def readline(self, _: int = -1) -> bytes:
        return self.replies.pop(0) if self.replies else b""

    def close(self) -> None:
        self.is_open = False


class TimeoutTrackingSerial(ScriptedSerial):
    def __init__(self, replies: list[bytes]) -> None:
        super().__init__(replies)
        self.timeout = 99.0
        self.read_timeouts: list[float] = []

    def readline(self, size: int = -1) -> bytes:
        self.read_timeouts.append(self.timeout)
        return super().readline(size)


class BlockingMoveSetSerial(ScriptedSerial):
    def __init__(self, replies: list[bytes]) -> None:
        super().__init__(replies)
        self.move_set_waiting = threading.Event()
        self.release_move_set = threading.Event()
        self._blocked_move_set = False

    def readline(self, size: int = -1) -> bytes:
        if (
            not self._blocked_move_set
            and self.writes
            and b" MOVE_SET " in self.writes[-1]
        ):
            self._blocked_move_set = True
            self.move_set_waiting.set()
            assert self.release_move_set.wait(timeout=2)
        return super().readline(size)


class ObservableRLock:
    def __init__(self, watched_thread_name: str) -> None:
        self._lock = threading.RLock()
        self._watched_thread_name = watched_thread_name
        self.watched_thread_waiting = threading.Event()

    def __enter__(self) -> ObservableRLock:
        if threading.current_thread().name == self._watched_thread_name:
            self.watched_thread_waiting.set()
        self._lock.acquire()
        return self

    def __exit__(self, *_: object) -> None:
        self._lock.release()


def _ok(seq: int, payload: dict[str, object]) -> bytes:
    return f"A1 {seq} OK {json.dumps(payload, separators=(',', ':'))}\n".encode()


def _err(seq: int, code: str, payload: dict[str, object]) -> bytes:
    return f"A1 {seq} ERR {code} {json.dumps(payload, separators=(',', ':'))}\n".encode()


def _servo(
    identifier: int,
    *,
    raw_position: int = 2048,
    torque_state: str = "off",
    operating_mode: int = 0,
    errors: list[str] | None = None,
) -> dict[str, object]:
    return {
        "id": identifier,
        "rawPosition": raw_position,
        "speed": 0,
        "load": 0,
        "voltageVolts": 12.0,
        "temperatureC": 28,
        "moving": False,
        "torqueState": torque_state,
        "packetAgeMs": 0,
        "errors": list(errors or []),
        "operatingMode": operating_mode,
        "online": True,
        "fresh": True,
        "statusError": 0,
    }


def _hello(
    *,
    position_mode_capability: bool = True,
    firmware_version: str = "arm-hat-1.0.2",
) -> dict[str, object]:
    capabilities = [
        "scan",
        "single_servo_id",
        "capture",
        "hold_set",
        "torque_lease",
        "bounded_nudge",
        "telemetry",
        "stop",
    ]
    if position_mode_capability:
        capabilities.append("set_position_mode")
    return {
        "controllerId": "hat-a-001",
        "bootId": "boot_abc",
        "firmwareVersion": firmware_version,
        "protocolVersion": 1,
        "state": "disarmed",
        "motionState": "blocked",
        "torqueState": "unknown",
        "busState": "unknown",
        "bootTorqueOffSent": True,
        "servosState": "unknown",
        "servoCount": None,
        "requestMaxBytes": 256,
        "responseMaxBytes": 4096,
        "host": {"uart": "Serial0", "baud": 115_200},
        "servoBus": {"uart": "Serial1", "baud": 1_000_000, "rx": 18, "tx": 19},
        "watchdogMs": 750,
        "leaseMs": {"min": 100, "max": 2_000},
        "servoModel": "user_confirmation_required",
        "registerProfile": "ST3215_candidate",
        "capabilities": capabilities,
    }


def _status(
    *,
    boot_id: str = "boot_abc",
    firmware_version: str = "arm-hat-1.0.0",
    servos: list[dict[str, object]] | None = None,
    motion_state: str = "blocked",
) -> dict[str, object]:
    telemetry = list(servos or [])
    torque_states = {str(item["torqueState"]) for item in telemetry}
    torque_state = (
        "on" if "on" in torque_states else "off" if telemetry else "unknown"
    )
    return {
        "controllerId": "hat-a-001",
        "bootId": boot_id,
        "firmwareVersion": firmware_version,
        "protocolVersion": 1,
        "motionState": motion_state,
        "torqueState": torque_state,
        "busState": "online" if telemetry else "unknown",
        "servosState": "online" if telemetry else "none_found",
        "servoCount": len(telemetry),
        "servos": telemetry,
    }


def _register_read_receipt(
    servo_id: int,
    address: int,
    values: list[int],
    *,
    status_error: int = 0,
) -> dict[str, object]:
    return {
        "servoId": servo_id,
        "address": address,
        "length": len(values),
        "statusError": status_error,
        "values": values,
    }


def _native_multi_turn_scan_proof() -> tuple[list[int], list[int], list[int]]:
    limits_and_phase = [0] * 16
    limits_and_phase[18 - 9] = 0x1C
    mode_and_torque = [0] * 11
    mode_and_torque[0] = 1
    # Lock is 1. Present Position is exact live evidence 0x831E: ST3215
    # signed-magnitude -798, whose one-turn projection is raw 3298.
    lock_and_position = [1, 0x1E, 0x83]
    return limits_and_phase, mode_and_torque, lock_and_position


def _native_multi_turn_receipt(servo_id: int = 1) -> dict[str, object]:
    return {
        "servoId": servo_id,
        "multiTurn": True,
        "angleMin": 0,
        "angleMax": 0,
        "operatingMode": 0,
        "phase": 0x1C,
        "resolution": 1,
    }


def test_serial_controller_uses_versioned_bounded_command_and_validates_sequence() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            )
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    hello = controller.start()

    assert serial.writes == [b"A1 1 HELLO\n"]
    assert serial.input_resets == 1
    assert hello["controllerId"] == "hat-a-001"
    assert hello["protocolVersion"] == 1


def test_serial_controller_rejects_corrupt_or_mismatched_replies() -> None:
    serial = ScriptedSerial([b"debug text must not be on this link\n"])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="invalid controller frame"):
        controller.start()
    assert controller.transport_state()["torqueState"] == "unknown"


def test_clean_ready_hello_connects_but_host_stays_blocked_until_status() -> None:
    hello = _hello(firmware_version="arm-hat-2.6.0")
    hello.update(
        {
            "motionState": "ready",
            "safetyFault": False,
            "operatorInspectionRequired": False,
            "safetyStopReason": None,
        }
    )
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    identity = controller.start()
    state = controller.transport_state()
    controller.close()

    assert identity["firmwareVersion"] == "arm-hat-2.6.0"
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["bus"] == "unknown"
    assert state["torqueState"] == "unknown"


def test_ready_hello_with_safety_fault_is_rejected() -> None:
    hello = _hello(firmware_version="arm-hat-2.6.0")
    hello.update(
        {
            "motionState": "ready",
            "safetyFault": True,
            "operatorInspectionRequired": False,
            "safetyStopReason": None,
        }
    )
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="inspection latch"):
        controller.start()


def test_recoverable_automatic_fault_hello_connects_without_inspection_latch() -> None:
    hello = _hello(firmware_version="arm-hat-2.6.0")
    hello.update(
        {
            "state": "faulted",
            "motionState": "blocked",
            "safetyFault": True,
            "operatorInspectionRequired": False,
            "safetyStopReason": None,
        }
    )
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    identity = controller.start()
    state = controller.transport_state()
    controller.close()

    assert identity["operatorInspectionRequired"] is False
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["operatorInspectionRequired"] is False


def test_serial_controller_rejects_oversized_lines_without_parsing_them() -> None:
    serial = ScriptedSerial([b"A" * 4097 + b"\n"])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="exceeded"):
        controller.start()
    assert controller.transport_state()["connection"] == "faulted"


def test_scan_arguments_are_checked_before_serial_write() -> None:
    serial = ScriptedSerial([])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ValueError, match="scan range"):
        controller.scan(21, 20)

    assert serial.writes == []


def test_full_bus_operations_receive_a_longer_bounded_read_deadline() -> None:
    serial = TimeoutTrackingSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status()),
            _ok(
                3,
                {
                    "foundIds": [],
                    "completeRange": {"minId": 0, "maxId": 253},
                },
            ),
            _ok(4, _status()),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    controller.start()
    controller.scan(0, 253)
    controller.close()

    assert serial.read_timeouts == [0.5, 0.5, 4.0, 0.5]
    assert serial.timeout == 99.0


def test_scan_keeps_link_online_when_ping_responder_lacks_fresh_telemetry() -> None:
    firmware = "arm-hat-2.7.0"
    all_servos = [_servo(identifier) for identifier in (1, 2, 3, 4)]
    post_status = _status(
        firmware_version=firmware,
        servos=[_servo(identifier) for identifier in (2, 3, 4)],
        motion_state="blocked",
    )
    # Firmware keeps the PING responder in its tracked inventory while omitting
    # telemetry that is not fresh enough to support pose or motion.
    post_status.update({"servoCount": 4, "servosState": "faulted"})
    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware)),
            _ok(
                2,
                _status(
                    firmware_version=firmware,
                    servos=all_servos,
                    motion_state="ready",
                ),
            ),
            _ok(
                3,
                {
                    "foundIds": [1, 2, 3, 4],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, post_status),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.scan(0, 10)

    assert result["foundIds"] == [1, 2, 3, 4]
    assert result["pingFoundIds"] == [1, 2, 3, 4]
    assert result["telemetryUnavailableIds"] == [1]
    assert result["telemetryRecoveredIds"] == []
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["bus"] == "online"
    assert state["servos"] == "faulted"
    assert state["servoCount"] == 4
    assert [servo["id"] for servo in state["servosTelemetry"]] == [2, 3, 4]
    assert state["lastScan"]["telemetryUnavailableIds"] == [1]
    assert serial.is_open is True
    controller.close()


def test_scan_returns_union_when_post_status_recovers_a_ping_miss() -> None:
    firmware = "arm-hat-2.7.0"
    post_servos = [_servo(identifier) for identifier in (1, 2, 3, 4)]
    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware)),
            _ok(
                2,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(identifier) for identifier in (1, 2, 3)],
                    motion_state="ready",
                ),
            ),
            _ok(
                3,
                {
                    "foundIds": [1, 2, 3],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(
                4,
                _status(
                    firmware_version=firmware,
                    servos=post_servos,
                    motion_state="ready",
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.scan(0, 10)

    assert result["foundIds"] == [1, 2, 3, 4]
    assert result["pingFoundIds"] == [1, 2, 3]
    assert result["telemetryUnavailableIds"] == []
    assert result["telemetryRecoveredIds"] == [4]
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert [servo["id"] for servo in state["servosTelemetry"]] == [1, 2, 3, 4]
    assert state["lastScan"]["foundIds"] == [1, 2, 3, 4]
    assert state["lastScan"]["telemetryRecoveredIds"] == [4]
    assert serial.is_open is True
    controller.close()


def test_scan_replays_missing_declared_family_then_recovers_telemetry() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("servo_family")
    all_servos = [_servo(identifier) for identifier in (1, 2, 3, 4)]
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, {"servoId": 1, "family": "sts"}),
            _ok(
                3,
                _status(
                    firmware_version=firmware,
                    servos=all_servos,
                    motion_state="ready",
                ),
            ),
            _ok(
                4,
                {
                    "foundIds": [2, 3, 4],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(
                5,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(identifier) for identifier in (2, 3, 4)],
                    motion_state="ready",
                ),
            ),
            _ok(6, {"servoId": 1, "family": "sts"}),
            _ok(
                7,
                _status(
                    firmware_version=firmware,
                    servos=all_servos,
                    motion_state="ready",
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.start()

    result = controller.scan(0, 10)

    assert result["pingFoundIds"] == [2, 3, 4]
    assert result["foundIds"] == [1, 2, 3, 4]
    assert result["telemetryUnavailableIds"] == []
    assert result["telemetryRecoveredIds"] == [1]
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["servos"] == "online"
    assert [servo["id"] for servo in state["servosTelemetry"]] == [1, 2, 3, 4]
    assert state["lastScan"]["telemetryRecoveredIds"] == [1]
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 FAMILY 1 STS\n",
        b"A1 3 STATUS\n",
        b"A1 4 SCAN 0 10\n",
        b"A1 5 STATUS\n",
        b"A1 6 FAMILY 1 STS\n",
        b"A1 7 STATUS\n",
    ]
    controller.close()


def test_scan_family_replay_does_not_invent_absent_telemetry() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("servo_family")
    visible_servos = [_servo(identifier) for identifier in (2, 3, 4)]
    final_status = _status(
        firmware_version=firmware,
        servos=visible_servos,
        motion_state="blocked",
    )
    final_status.update({"servoCount": 4, "servosState": "faulted"})
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, {"servoId": 1, "family": "sts"}),
            _ok(
                3,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(identifier) for identifier in (1, 2, 3, 4)],
                    motion_state="ready",
                ),
            ),
            _ok(
                4,
                {
                    "foundIds": [2, 3, 4],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(
                5,
                _status(
                    firmware_version=firmware,
                    servos=visible_servos,
                    motion_state="ready",
                ),
            ),
            _ok(6, {"servoId": 1, "family": "sts"}),
            _ok(7, final_status),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    result = controller.scan(0, 10)

    assert result["pingFoundIds"] == [2, 3, 4]
    assert result["foundIds"] == [2, 3, 4]
    assert result["telemetryUnavailableIds"] == []
    assert result["telemetryRecoveredIds"] == []
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["servos"] == "faulted"
    assert state["servoCount"] == 4
    assert [servo["id"] for servo in state["servosTelemetry"]] == [2, 3, 4]
    assert serial.writes[-2:] == [
        b"A1 6 FAMILY 1 STS\n",
        b"A1 7 STATUS\n",
    ]
    assert serial.is_open is True
    controller.close()


def test_scan_restores_old_hat_native_decoder_without_establishing_base_truth() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    visible = [_servo(identifier) for identifier in (2, 3, 4)]
    incomplete_status = _status(
        firmware_version=firmware,
        servos=visible,
        motion_state="blocked",
    )
    incomplete_status.update({"servoCount": 4, "servosState": "faulted"})
    limits_and_phase, mode_and_torque, lock_and_position = (
        _native_multi_turn_scan_proof()
    )
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, incomplete_status),
            _ok(
                3,
                {
                    "foundIds": [1, 2, 3, 4],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, incomplete_status),
            _ok(5, _register_read_receipt(1, 9, limits_and_phase)),
            _ok(6, _register_read_receipt(1, 30, mode_and_torque)),
            _ok(7, _register_read_receipt(1, 55, lock_and_position)),
            _ok(8, _native_multi_turn_receipt()),
            _ok(
                9,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(1, raw_position=0x831E), *visible],
                    motion_state="blocked",
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    result = controller.scan(0, 10)

    assert result["pingFoundIds"] == [1, 2, 3, 4]
    assert result["foundIds"] == [1, 2, 3, 4]
    assert result["telemetryUnavailableIds"] == []
    assert result["telemetryRecoveredIds"] == [1]
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["lastScan"]["telemetryRecoveredIds"] == [1]
    assert [servo["id"] for servo in state["servosTelemetry"]] == [1, 2, 3, 4]
    assert state["servosTelemetry"][0]["rawPosition"] == 3298
    operations = [write.decode().split()[2] for write in serial.writes]
    assert operations == [
        "HELLO",
        "STATUS",
        "SCAN",
        "STATUS",
        "REG_READ",
        "REG_READ",
        "REG_READ",
        "MULTITURN",
        "STATUS",
    ]
    assert "ODO_ZERO" not in operations
    assert "ODO_READ" not in operations
    controller.close()


def test_status_rejects_extended_sts_position_without_native_id_declaration() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(
                2,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(1, raw_position=0x831E, operating_mode=0)],
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.start()

    with pytest.raises(ControllerProtocolError, match="invalid raw position"):
        controller.status()

    assert controller.transport_state()["connection"] == "faulted"


@pytest.mark.parametrize(
    "mismatch",
    [
        "minimum",
        "maximum",
        "phase",
        "resolution",
        "operating_mode",
        "torque",
        "lock",
        "single_turn_position",
        "position_out_of_range",
        "servo_status_error",
    ],
)
def test_scan_native_decoder_recovery_requires_every_exact_register_proof(
    mismatch: str,
) -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    limits_and_phase, mode_and_torque, lock_and_position = (
        _native_multi_turn_scan_proof()
    )
    status_errors = [0, 0, 0]
    if mismatch == "minimum":
        limits_and_phase[0] = 1
    elif mismatch == "maximum":
        limits_and_phase[2] = 1
    elif mismatch == "phase":
        limits_and_phase[18 - 9] &= ~0x10
    elif mismatch == "resolution":
        mode_and_torque[0] = 2
    elif mismatch == "operating_mode":
        mode_and_torque[33 - 30] = 3
    elif mismatch == "torque":
        mode_and_torque[40 - 30] = 1
    elif mismatch == "lock":
        lock_and_position[0] = 0
    elif mismatch == "single_turn_position":
        lock_and_position[1:] = [0x00, 0x08]
    elif mismatch == "position_out_of_range":
        lock_and_position[1:] = [0x00, 0xF8]
    elif mismatch == "servo_status_error":
        status_errors[2] = 1
    else:  # pragma: no cover - pytest owns the finite parameter set
        raise AssertionError(mismatch)
    missing_status = _status(firmware_version=firmware, servos=[])
    missing_status.update({"servoCount": 1, "servosState": "faulted"})
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, missing_status),
            _ok(
                3,
                {
                    "foundIds": [1],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, missing_status),
            _ok(
                5,
                _register_read_receipt(
                    1, 9, limits_and_phase, status_error=status_errors[0]
                ),
            ),
            _ok(
                6,
                _register_read_receipt(
                    1, 30, mode_and_torque, status_error=status_errors[1]
                ),
            ),
            _ok(
                7,
                _register_read_receipt(
                    1, 55, lock_and_position, status_error=status_errors[2]
                ),
            ),
            _ok(8, missing_status),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    result = controller.scan(0, 10)

    assert result["telemetryUnavailableIds"] == [1]
    assert result["telemetryRecoveredIds"] == []
    assert controller.transport_state()["connection"] == "online"
    assert not any(b" MULTITURN " in write for write in serial.writes)
    assert serial.writes[-1] == b"A1 8 STATUS\n"
    controller.close()


@pytest.mark.parametrize("rejected_operation", ["first", "second", "third", "arm"])
def test_scan_keeps_successful_ping_evidence_when_decoder_recovery_is_rejected(
    rejected_operation: str,
) -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    missing_status = _status(firmware_version=firmware, servos=[])
    missing_status.update({"servoCount": 1, "servosState": "faulted"})
    limits_and_phase, mode_and_torque, lock_and_position = (
        _native_multi_turn_scan_proof()
    )
    replies = [
        _ok(1, hello),
        _ok(2, missing_status),
        _ok(
            3,
            {
                "foundIds": [1],
                "completeRange": {"minId": 0, "maxId": 10},
            },
        ),
        _ok(4, missing_status),
    ]
    proof_replies = [
        _ok(5, _register_read_receipt(1, 9, limits_and_phase)),
        _ok(6, _register_read_receipt(1, 30, mode_and_torque)),
        _ok(7, _register_read_receipt(1, 55, lock_and_position)),
        _ok(8, _native_multi_turn_receipt()),
    ]
    rejected_index = {"first": 0, "second": 1, "third": 2, "arm": 3}[
        rejected_operation
    ]
    proof_replies[rejected_index] = _err(
        5 + rejected_index,
        "BUS_ERROR" if rejected_operation != "arm" else "MULTI_TURN_CONFIG_FAILED",
        {},
    )
    replies.extend(proof_replies[: rejected_index + 1])
    degraded_status = dict(missing_status)
    degraded_status.update({"busState": "faulted", "servosState": "faulted"})
    replies.append(_ok(6 + rejected_index, degraded_status))
    serial = ScriptedSerial(replies)
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    result = controller.scan(0, 10)

    assert result["foundIds"] == [1]
    assert result["telemetryUnavailableIds"] == [1]
    assert result["telemetryRecoveredIds"] == []
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["bus"] == "faulted"
    assert state["servos"] == "faulted"
    assert state["servoCount"] == 1
    assert state["servosTelemetry"] == []
    assert serial.writes[-1] == f"A1 {6 + rejected_index} STATUS\n".encode()
    assert serial.is_open is True
    controller.close()


@pytest.mark.parametrize(
    ("native_ids", "family", "with_capability", "ping_ids", "has_telemetry"),
    [
        ([], "STS", True, [1], False),
        ([2], "STS", True, [1], False),
        ([1], "SCS", True, [1], False),
        ([1], "STS", False, [1], False),
        ([1], "STS", True, [], False),
        ([1], "STS", True, [1], True),
    ],
)
def test_scan_decoder_recovery_is_tightly_gated(
    native_ids: list[int],
    family: str,
    with_capability: bool,
    ping_ids: list[int],
    has_telemetry: bool,
) -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    if with_capability:
        hello["capabilities"] = [
            *hello["capabilities"],
            "multi_turn_absolute_v1",
        ]
    servos = [_servo(1)] if has_telemetry else []
    status = _status(firmware_version=firmware, servos=servos)
    if ping_ids and not has_telemetry:
        status.update({"servoCount": 1, "servosState": "faulted"})
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, status),
            _ok(
                3,
                {
                    "foundIds": ping_ids,
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, status),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, family)
    controller.declare_native_multi_turn_servos([1])
    # The declaration is a full-set replacement, so this also proves a Base-ID
    # change removes the old ID from recovery eligibility.
    controller.declare_native_multi_turn_servos(native_ids)
    controller.start()

    result = controller.scan(0, 10)

    assert not any(b" REG_READ " in write for write in serial.writes)
    assert not any(b" MULTITURN " in write for write in serial.writes)
    assert result["telemetryUnavailableIds"] == (
        [1] if ping_ids and not has_telemetry else []
    )
    controller.close()


def test_scan_decoder_recovery_propagates_a_malformed_register_receipt() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    missing_status = _status(firmware_version=firmware, servos=[])
    missing_status.update({"servoCount": 1, "servosState": "faulted"})
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, missing_status),
            _ok(
                3,
                {
                    "foundIds": [1],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, missing_status),
            _ok(
                5,
                {
                    "servoId": 1,
                    "address": 9,
                    "length": 16,
                    "statusError": 0,
                    "values": [0] * 15,
                },
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    with pytest.raises(ControllerProtocolError, match="register read"):
        controller.scan(0, 10)

    assert controller.transport_state()["connection"] == "faulted"


def test_scan_decoder_recovery_requires_an_exact_multiturn_receipt() -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "multi_turn_absolute_v1",
    ]
    missing_status = _status(firmware_version=firmware, servos=[])
    missing_status.update({"servoCount": 1, "servosState": "faulted"})
    limits_and_phase, mode_and_torque, lock_and_position = (
        _native_multi_turn_scan_proof()
    )
    malformed_receipt = _native_multi_turn_receipt()
    malformed_receipt["phase"] = 0x0C
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, missing_status),
            _ok(
                3,
                {
                    "foundIds": [1],
                    "completeRange": {"minId": 0, "maxId": 10},
                },
            ),
            _ok(4, missing_status),
            _ok(5, _register_read_receipt(1, 9, limits_and_phase)),
            _ok(6, _register_read_receipt(1, 30, mode_and_torque)),
            _ok(7, _register_read_receipt(1, 55, lock_and_position)),
            _ok(8, malformed_receipt),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    with pytest.raises(ControllerProtocolError, match="multi-turn mode"):
        controller.scan(0, 10)

    assert controller.transport_state()["connection"] == "faulted"


def test_multi_turn_home_arms_native_absolute_mode_before_capturing_zero() -> None:
    """Native Mode 0 is configured first; its current coordinate becomes zero."""

    firmware = "arm-hat-2.3.0"
    odometer = {
        "servoId": 1,
        "tracking": True,
        "valid": True,
        "stepMode": False,
        "revolutions": 2,
        "rawPosition": 3179,
        "multiTurnPosition": 11_371,
        "sampleAgeMs": 0,
        "stepOutstanding": False,
        "countdownObserved": False,
        "resyncNeeded": False,
        "resyncCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    **_hello(firmware_version=firmware),
                    "capabilities": [
                        *_hello(firmware_version=firmware)["capabilities"],
                        "multi_turn_absolute_v1",
                    ],
                },
            ),
            _ok(2, _status(firmware_version=firmware, servos=[_servo(1)])),
            _ok(3, {"servoId": 1, "multiTurn": True, "operatingMode": 0}),
            _ok(4, odometer),
            _ok(
                5,
                odometer,
            ),
            _ok(
                6,
                _status(
                    firmware_version=firmware,
                    servos=[_servo(1, operating_mode=0)],
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.home_multi_turn(1)

    assert result["valid"] is True
    assert result["stepMode"] is False
    assert result["multiTurnPosition"] == 11_371
    operations = [write.decode().split()[2] for write in serial.writes]
    assert operations == [
        "HELLO",
        "STATUS",
        "MULTITURN",
        "ODO_ZERO",
        "ODO_READ",
        "STATUS",
    ]
    assert serial.writes[2].decode().strip().endswith("MULTITURN 1 ON")


def _inspection_latched_status(
    *,
    firmware_version: str = "arm-hat-2.5.0",
    servos: list[dict[str, object]] | None = None,
    reason: str = "MOVE_SET_FAILED",
) -> dict[str, object]:
    payload = _status(
        firmware_version=firmware_version,
        servos=servos,
        motion_state="stopped",
    )
    payload.update(
        {
            "stopped": True,
            "safetyFault": True,
            "operatorInspectionRequired": True,
            "safetyStopReason": reason,
        }
    )
    return payload


def _move_set_receipt(
    moves: list[tuple[int, int, int, int]], *, boot_id: str = "boot_abc"
) -> dict[str, object]:
    return {
        "controllerId": "hat-a-001",
        "bootId": boot_id,
        "firmwareVersion": "arm-hat-2.5.0",
        "protocolVersion": 1,
        "dispatch": "dialect_grouped_sync_write",
        "crossFamilyAtomic": False,
        "count": len(moves),
        "moved": [
            {
                "servoId": servo_id,
                "goal": goal,
                "speed": speed,
                "acceleration": acceleration,
            }
            for servo_id, goal, speed, acceleration in moves
        ],
    }


def _enable_move_set(hello: dict[str, object]) -> None:
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.extend(["servo_family", "move_set_v1"])


def _enable_follow_set(hello: dict[str, object]) -> None:
    _enable_move_set(hello)
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.extend(["follow_set_feedback_v1", "follow_feedback_v1"])


def _follow_set_receipt(
    moves: list[tuple[int, int, int, int]],
    feedback: list[dict[str, object]],
) -> dict[str, object]:
    receipt = _move_set_receipt(moves)
    receipt["firmwareVersion"] = "arm-hat-2.7.0"
    receipt["feedback"] = feedback
    return receipt


def _follow_read_receipt(
    feedback: list[dict[str, object]], *, boot_id: str = "boot_abc"
) -> dict[str, object]:
    return {
        "controllerId": "hat-a-001",
        "bootId": boot_id,
        "firmwareVersion": "arm-hat-2.7.0",
        "protocolVersion": 1,
        "count": 2,
        "feedback": feedback,
    }


def _assert_unconfirmed_move_set_state(state: dict[str, object]) -> None:
    assert state["connection"] == "faulted"
    assert state["motionState"] == "blocked"
    assert state["torqueState"] == "unknown"
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert state["lastMotionFailure"] == {
        "phase": "unconfirmed",
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
    }


def test_move_set_capability_requires_servo_family_at_handshake() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("move_set_v1")
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="capability dependency"):
        controller.start()

    assert controller.transport_state()["connection"] == "faulted"
    assert serial.writes == [b"A1 1 HELLO\n"]
    assert not any(b"FAMILY" in write or b"MOVE_SET" in write for write in serial.writes)


def test_follow_set_capability_requires_servo_family_at_handshake() -> None:
    hello = _hello(firmware_version="arm-hat-2.7.0")
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("follow_set_feedback_v1")
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="capability dependency"):
        controller.start()

    assert controller.transport_state()["connection"] == "faulted"
    assert serial.writes == [b"A1 1 HELLO\n"]


def test_follow_feedback_capability_requires_servo_family_at_handshake() -> None:
    hello = _hello(firmware_version="arm-hat-2.7.0")
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("follow_feedback_v1")
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="capability dependency"):
        controller.start()

    assert controller.transport_state()["connection"] == "faulted"
    assert serial.writes == [b"A1 1 HELLO\n"]


def test_scs_dialect_sensitive_operations_require_servo_family_capability() -> None:
    hello = _hello(firmware_version="arm-hat-2.4.0")
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)
    controller.declare_family(4, "SCS")

    operations = (
        lambda: controller.capture(4),
        lambda: controller.set_position_mode(4),
        lambda: controller.move(4, 512, 100, 10),
        lambda: controller.torque_lease(4, 500),
        lambda: controller.set_hold_servos([4]),
        lambda: controller.prepare_nudge(4, 8, 80, 8),
    )
    for operation in operations:
        with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
            operation()

    assert serial.writes == [b"A1 1 HELLO\n"]


def test_scs_telemetry_is_rejected_when_hat_cannot_hold_family_map() -> None:
    hello = _hello(firmware_version="arm-hat-2.4.0")
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(
                2,
                _status(
                    firmware_version="arm-hat-2.4.0",
                    servos=[_servo(4, raw_position=512)],
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)
    controller.declare_family(4, "SCS")

    with pytest.raises(ControllerProtocolError, match="SCS telemetry"):
        controller.status()

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["servosTelemetry"] == []
    assert serial.writes == [b"A1 1 HELLO\n", b"A1 2 STATUS\n"]


@pytest.mark.parametrize(
    "reason",
    ["MOVE_SET_FAILED", "EXPLICIT_STOP", "SAFETY_FAULT"],
)
def test_fresh_process_ingests_controller_stop_latch(reason: str) -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    hello.update(
        {
            "state": "stopped_latched",
            "motionState": "stopped",
            "safetyFault": True,
            "operatorInspectionRequired": True,
            "safetyStopReason": reason,
        }
    )
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "stopped"
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == reason
    with pytest.raises(ControllerCommandError, match="OPERATOR_INSPECTION_REQUIRED"):
        controller.move(2, 900, 100, 10)
    assert serial.writes == [b"A1 1 HELLO\n"]


@pytest.mark.parametrize(
    "invalid_update",
    [
        {"operatorInspectionRequired": True, "safetyStopReason": None},
        {"operatorInspectionRequired": False, "safetyStopReason": "MOVE_SET_FAILED"},
        {
            "operatorInspectionRequired": True,
            "safetyStopReason": "MOVE_SET_FAILED",
            "state": "disarmed",
        },
        {
            "operatorInspectionRequired": True,
            "safetyStopReason": "MOVE_SET_FAILED",
            "motionState": "blocked",
        },
        {
            "operatorInspectionRequired": True,
            "safetyStopReason": "UNKNOWN_STOP",
            "state": "stopped_latched",
            "motionState": "stopped",
            "safetyFault": True,
        },
    ],
)
def test_hello_rejects_inconsistent_controller_inspection_latch(
    invalid_update: dict[str, object],
) -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    hello.update(invalid_update)
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="inspection latch"):
        controller.start()

    assert controller.transport_state()["connection"] == "faulted"


@pytest.mark.parametrize(
    "reason",
    ["MOVE_SET_FAILED", "EXPLICIT_STOP", "SAFETY_FAULT"],
)
def test_status_and_heartbeat_restore_controller_inspection_latch(reason: str) -> None:
    controller = SerialArmController("loop", serial_factory=lambda **_: ScriptedSerial([]))
    with controller._state_lock:
        controller._identity = {
            "controllerId": "hat-a-001",
            "bootId": "boot_abc",
            "firmwareVersion": "arm-hat-2.5.0",
            "protocolVersion": 1,
            "capabilities": ["servo_family", "move_set_v1"],
        }
        controller._connection = "online"

    reported = {
        "operatorInspectionRequired": True,
        "safetyStopReason": reason,
        "motionState": "stopped",
        "torqueState": "unknown",
        "stopped": True,
        "safetyFault": True,
    }
    controller._ingest_status(reported, heartbeat=True)

    state = controller.transport_state()
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == reason
    assert state["motionState"] == "stopped"


@pytest.mark.parametrize(
    "reported",
    [
        {"operatorInspectionRequired": True, "safetyStopReason": None},
        {"operatorInspectionRequired": False, "safetyStopReason": "MOVE_SET_FAILED"},
        {
            "operatorInspectionRequired": True,
            "safetyStopReason": "MOVE_SET_FAILED",
            "motionState": "blocked",
        },
        {
            "operatorInspectionRequired": True,
            "safetyStopReason": "UNKNOWN_STOP",
            "motionState": "stopped",
            "stopped": True,
            "safetyFault": True,
        },
    ],
)
def test_status_rejects_inconsistent_controller_inspection_latch(
    reported: dict[str, object],
) -> None:
    controller = SerialArmController("loop", serial_factory=lambda **_: ScriptedSerial([]))
    with controller._state_lock:
        controller._identity = {
            "controllerId": "hat-a-001",
            "bootId": "boot_abc",
            "firmwareVersion": "arm-hat-2.5.0",
            "protocolVersion": 1,
            "capabilities": ["servo_family", "move_set_v1"],
        }
        controller._connection = "online"

    with pytest.raises(ControllerProtocolError, match="inspection latch"):
        controller._ingest_status(reported)

    assert controller.transport_state()["connection"] == "faulted"


@pytest.mark.parametrize(
    "reason",
    ["MOVE_SET_FAILED", "EXPLICIT_STOP", "SAFETY_FAULT"],
)
def test_reset_rechecks_status_latch_before_sending_reset(reason: str) -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _inspection_latched_status(reason=reason))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="OPERATOR_INSPECTION_REQUIRED"):
        controller.reset()

    assert serial.writes == [b"A1 1 HELLO\n", b"A1 2 STATUS\n"]
    assert controller.transport_state()["operatorInspectionRequired"] is True


def test_reset_does_not_manufacture_inspection_from_transient_safety_fault() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    faulted_status = _status(firmware_version="arm-hat-2.5.0")
    faulted_status.update(
        {
            "stopped": False,
            "safetyFault": True,
            "operatorInspectionRequired": False,
            "safetyStopReason": None,
        }
    )
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, faulted_status),
            _err(
                3,
                "RESET_PRECONDITION",
                {"inspectedToken": True, "torqueOffConfirmed": False},
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="RESET_PRECONDITION"):
        controller.reset()

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 RESET INSPECTED\n",
    ]


def test_family_intent_declared_while_stopped_is_replayed_before_reset_opens_gate() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    hello.update(
        {
            "state": "stopped_latched",
            "motionState": "stopped",
            "safetyFault": True,
            "operatorInspectionRequired": True,
            "safetyStopReason": "EXPLICIT_STOP",
        }
    )
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, _inspection_latched_status(reason="EXPLICIT_STOP")),
            _ok(3, {"servoId": 7, "family": "scs"}),
            _ok(
                4,
                {
                    "stopped": False,
                    "torqueState": "off",
                    "torqueOffConfirmed": True,
                    "reset": True,
                },
            ),
            _ok(5, _status(firmware_version="arm-hat-2.5.0")),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    controller.declare_family(7, "SCS")
    assert serial.writes == [b"A1 1 HELLO\n"]

    controller.reset(inspected=True)

    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 FAMILY 7 SCS\n",
        b"A1 4 RESET INSPECTED\n",
        b"A1 5 STATUS\n",
    ]
    assert controller.transport_state()["operatorInspectionRequired"] is False


def test_failed_deferred_family_receipt_keeps_reset_motion_gate_closed() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    hello.update(
        {
            "state": "stopped_latched",
            "motionState": "stopped",
            "safetyFault": True,
            "operatorInspectionRequired": True,
            "safetyStopReason": "SAFETY_FAULT",
        }
    )
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, _inspection_latched_status(reason="SAFETY_FAULT")),
            _err(3, "BAD_ARGS", {}),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)
    controller.declare_family(7, "SCS")

    with pytest.raises(ControllerProtocolError, match="family declaration rejected"):
        controller.reset(inspected=True)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "SAFETY_FAULT"
    assert serial.writes[-1] == b"A1 3 FAMILY 7 SCS\n"
    assert not any(b" RESET " in write for write in serial.writes)
    assert not any(b" MOVE " in write or b" MOVE_SET " in write for write in serial.writes)


def test_identity_guard_rechecks_status_latch_before_unsafe_mutation() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _inspection_latched_status())]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="OPERATOR_INSPECTION_REQUIRED"):
        controller.assign_id(1, 2)

    assert serial.writes == [b"A1 1 HELLO\n", b"A1 2 STATUS\n"]
    assert not any(b"ASSIGN_ID" in write for write in serial.writes)


def test_uninspected_reset_queued_behind_move_set_failure_sends_zero_reset() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
    }
    serial = BlockingMoveSetSerial(
        [
            _ok(1, hello),
            _err(2, "MOVE_SET_FAILED", failure),
            _ok(3, _inspection_latched_status()),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    move_errors: list[BaseException] = []
    reset_errors: list[BaseException] = []
    observed_lock = ObservableRLock("reset-race")
    controller._io_lock = observed_lock

    def run_move_set() -> None:
        try:
            controller.move_set([(2, 1_000, 2_000, 40)])
        except BaseException as error:  # captured for deterministic thread proof
            move_errors.append(error)

    def run_reset() -> None:
        try:
            controller.reset()
        except BaseException as error:  # captured for deterministic thread proof
            reset_errors.append(error)

    move_thread = threading.Thread(target=run_move_set)
    move_thread.start()
    assert serial.move_set_waiting.wait(timeout=2)
    reset_thread = threading.Thread(target=run_reset, name="reset-race")
    reset_thread.start()
    assert observed_lock.watched_thread_waiting.wait(timeout=2)
    serial.release_move_set.set()
    move_thread.join(timeout=2)
    reset_thread.join(timeout=2)

    assert not move_thread.is_alive()
    assert not reset_thread.is_alive()
    assert len(move_errors) == 1
    assert isinstance(move_errors[0], ControllerCommandError)
    assert len(reset_errors) == 1
    assert isinstance(reset_errors[0], ControllerCommandError)
    assert str(reset_errors[0]) == "OPERATOR_INSPECTION_REQUIRED"
    assert not any(b"RESET" in write for write in serial.writes)
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 MOVE_SET 2,1000,2000,40\n",
        b"A1 3 STATUS\n",
    ]


def test_move_set_is_one_boot_bound_command_without_status_sandwich() -> None:
    moves = [(1, -5_000, 2_000, 40), (4, 512, 1_000, 30)]
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("multi_turn_absolute_v1")
    serial = ScriptedSerial([_ok(1, hello), _ok(2, _move_set_receipt(moves))])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.move_set(moves)

    assert result["moved"] == _move_set_receipt(moves)["moved"]
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 MOVE_SET 1,-5000,2000,40 4,512,1000,30\n",
    ]


def test_move_set_rejects_wrong_boot_receipt_and_faults_transport() -> None:
    moves = [(2, 1_000, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _move_set_receipt(moves, boot_id="boot_other"))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="identity changed"):
        controller.move_set(moves)

    _assert_unconfirmed_move_set_state(controller.transport_state())


def test_move_set_timeout_faults_without_inspection_and_recovers_after_reconnect() -> None:
    moves = [(2, 1_000, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    first = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: first)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerTransportError, match="timed out"):
        controller.move_set(moves)

    _assert_unconfirmed_move_set_state(controller.transport_state())

    second = ScriptedSerial([_ok(3, hello), _ok(4, _move_set_receipt(moves))])
    controller._serial_factory = lambda **_: second
    controller.reconnect()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None

    assert controller.move_set(moves)["moved"] == _move_set_receipt(moves)["moved"]
    assert second.writes == [
        b"A1 3 HELLO\n",
        b"A1 4 MOVE_SET 2,1000,2000,40\n",
    ]
    assert not any(b"STOP" in write or b"RESET" in write for write in second.writes)


@pytest.mark.parametrize(
    ("field", "value"),
    [("dispatch", "atomic_action"), ("crossFamilyAtomic", True)],
)
def test_move_set_rejects_receipt_that_overstates_grouped_dispatch(
    field: str, value: object
) -> None:
    moves = [(2, 1_000, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    receipt = _move_set_receipt(moves)
    receipt[field] = value
    serial = ScriptedSerial([_ok(1, hello), _ok(2, receipt)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="dispatch receipt"):
        controller.move_set(moves)

    _assert_unconfirmed_move_set_state(controller.transport_state())


def test_move_set_failure_aborts_without_manual_stop_latch() -> None:
    moves = [(2, 1_000, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _err(
                2,
                "MOVE_SET_FAILED",
                {
                    "phase": "verify",
                    "failedIndex": 0,
                    "stopped": False,
                    "torqueState": "unknown",
                    "latentCommands": False,
                    "motionMayHaveStarted": True,
                    "partialDispatchPossible": True,
                    "dispatchedFamilyCount": 1,
                },
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    # This test isolates the synchronous MOVE_SET error transition. If the
    # daemon consumes the intentionally exhausted ScriptedSerial afterward,
    # that is a genuine *new* transport loss and correctly supersedes the
    # cached STOP receipt with faulted/blocked/unknown state.
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.move_set(moves)

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["torqueState"] == "unknown"
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert state["lastMotionFailure"] == {
        "phase": "verify",
        "failedIndex": 0,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 1,
    }

    # A later healthy sample recovers normally; transport loss remains blocked
    # but must not manufacture an operator-clear requirement.
    controller._ingest_status({"motionState": "ready"})
    assert controller.transport_state()["motionState"] == "ready"
    controller._transport_fault()
    assert controller.transport_state()["motionState"] == "blocked"
    assert controller.transport_state()["operatorInspectionRequired"] is False


def test_move_set_failure_requires_explicit_inspection_before_reset() -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "off",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _err(2, "MOVE_SET_FAILED", failure),
            _ok(3, _inspection_latched_status(firmware_version=firmware)),
            _ok(
                4,
                {
                    "stopped": False,
                    "torqueState": "off",
                    "torqueOffConfirmed": True,
                    "reset": True,
                },
            ),
            _ok(5, _status(firmware_version=firmware)),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.move_set([(2, 1_000, 2_000, 40)])
    writes_before_reset = list(serial.writes)

    with pytest.raises(ControllerCommandError, match="OPERATOR_INSPECTION_REQUIRED"):
        controller.reset()
    assert serial.writes == writes_before_reset

    controller.reset(inspected=True)
    state = controller.transport_state()
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert state["lastMotionFailure"] is None
    assert state["motionState"] == "blocked"
    assert b"A1 4 RESET INSPECTED\n" in serial.writes


def test_inspected_reset_rejects_unproven_torque_receipt_and_keeps_latch() -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _err(2, "MOVE_SET_FAILED", failure),
            _ok(3, _inspection_latched_status(firmware_version=firmware)),
            _ok(
                4,
                {
                    "stopped": False,
                    "torqueState": "unknown",
                    "torqueOffConfirmed": False,
                    "reset": True,
                },
            ),
            _ok(5, _status(firmware_version=firmware, motion_state="stopped")),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.move_set([(2, 1_000, 2_000, 40)])
    with pytest.raises(ControllerProtocolError, match="remain disarmed"):
        controller.reset(inspected=True)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "MOVE_SET_FAILED"
    assert state["motionState"] == "stopped"


def test_reset_precondition_error_keeps_inspection_latched() -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _err(2, "MOVE_SET_FAILED", failure),
            _ok(3, _inspection_latched_status(firmware_version=firmware)),
            _err(
                4,
                "RESET_PRECONDITION",
                {"inspectedToken": True, "torqueOffConfirmed": False},
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.move_set([(2, 1_000, 2_000, 40)])
    with pytest.raises(ControllerCommandError, match="RESET_PRECONDITION"):
        controller.reset(inspected=True)

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "MOVE_SET_FAILED"
    assert state["motionState"] == "stopped"


@pytest.mark.parametrize("post_reset_status_was_lost", [False, True])
def test_inspected_reset_converges_after_success_proof_is_lost(
    post_reset_status_was_lost: bool,
) -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
    }
    latched_status = _inspection_latched_status(
        firmware_version=firmware,
        servos=[_servo(2)],
    )
    first_replies = [
        _ok(1, hello),
        _err(2, "MOVE_SET_FAILED", failure),
        _ok(3, latched_status),
    ]
    if post_reset_status_was_lost:
        first_replies.append(
            _ok(
                4,
                {
                    "stopped": False,
                    "torqueState": "off",
                    "torqueOffConfirmed": True,
                    "reset": True,
                },
            )
        )
        # Sequence 5 post-STATUS is lost.
    # Otherwise firmware accepts sequence 4 and clears its own latch, but the
    # RESET OK itself is lost. Either way the Pi cannot prove completion.
    first = ScriptedSerial(first_replies)
    controller = SerialArmController("loop", serial_factory=lambda **_: first)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.move_set([(2, 1_000, 2_000, 40)])
    with pytest.raises(ControllerTransportError, match="timed out"):
        controller.reset(inspected=True)
    assert controller.transport_state()["operatorInspectionRequired"] is True

    unlatched_status = _status(
        firmware_version=firmware,
        servos=[_servo(2)],
        motion_state="blocked",
    )
    stopped_status = _status(
        firmware_version=firmware,
        servos=[_servo(2)],
        motion_state="stopped",
    )
    reconnect_sequence = 6 if post_reset_status_was_lost else 5
    second = ScriptedSerial(
        [
            _ok(reconnect_sequence, hello),
            _ok(reconnect_sequence + 1, unlatched_status),
            _ok(
                reconnect_sequence + 2,
                {
                    "stopped": True,
                    "torqueState": "off",
                    "confirmed": True,
                    "torqueOffBroadcastSent": True,
                    "torqueOffPending": False,
                },
            ),
            _ok(reconnect_sequence + 3, stopped_status),
            _ok(
                reconnect_sequence + 4,
                {
                    "stopped": False,
                    "torqueState": "off",
                    "torqueOffConfirmed": True,
                    "reset": True,
                },
            ),
            _ok(reconnect_sequence + 5, unlatched_status),
        ]
    )
    controller._serial_factory = lambda **_: second
    controller.reconnect()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    controller.reset(inspected=True)

    state = controller.transport_state()
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert state["motionState"] == "blocked"
    assert second.writes == [
        f"A1 {reconnect_sequence} HELLO\n".encode(),
        f"A1 {reconnect_sequence + 1} STATUS\n".encode(),
        f"A1 {reconnect_sequence + 2} STOP\n".encode(),
        f"A1 {reconnect_sequence + 3} STATUS\n".encode(),
        f"A1 {reconnect_sequence + 4} RESET INSPECTED\n".encode(),
        f"A1 {reconnect_sequence + 5} STATUS\n".encode(),
    ]
    assert not any(
        b"HOLD_SET" in write or b"MOVE " in write or b"TORQUE_LEASE" in write
        for write in second.writes
    )


@pytest.mark.parametrize(
    "invalid_update",
    [
        {"motionMayHaveStarted": False},
        {"partialDispatchPossible": False},
        {"phase": "stage"},
        {"failedIndex": 1},
        {"torqueState": "on"},
        {"dispatchedFamilyCount": 3},
        {"phase": "dispatch", "dispatchedFamilyCount": 2},
        {"phase": "verify", "dispatchedFamilyCount": 0},
    ],
)
def test_move_set_rejects_understated_or_malformed_failure_receipt(
    invalid_update: dict[str, object],
) -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    failure = {
        "phase": "dispatch",
        "failedIndex": 0,
        "stopped": True,
        "torqueState": "unknown",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 0,
        **invalid_update,
    }
    serial = ScriptedSerial(
        [_ok(1, hello), _err(2, "MOVE_SET_FAILED", failure)]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerProtocolError, match="failure receipt"):
        controller.move_set([(2, 1_000, 2_000, 40)])

    _assert_unconfirmed_move_set_state(controller.transport_state())


def test_move_set_unknown_post_transmission_error_faults_without_inspection() -> None:
    firmware = "arm-hat-2.5.0"
    hello = _hello(firmware_version=firmware)
    _enable_move_set(hello)
    serial = ScriptedSerial([_ok(1, hello), _err(2, "BUS_ERROR", {})])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerProtocolError, match="unexpected move set error"):
        controller.move_set([(2, 1_000, 2_000, 40)])

    _assert_unconfirmed_move_set_state(controller.transport_state())


def test_move_set_validates_every_tuple_before_serial_write() -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    _enable_move_set(hello)
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    invalid_sets = [
        [],
        [(1, 100, 100, 10)] * 5,
        [(1, 100, 100, 10), (1, 200, 100, 10)],
        [(2, 30_720, 100, 10)],
        [(2, 100, 0, 10)],
        [(2, 100, 100, 0)],
    ]
    for invalid in invalid_sets:
        with pytest.raises(ValueError):
            controller.move_set(invalid)

    assert serial.writes == [b"A1 1 HELLO\n"]


def test_follow_set_is_one_command_and_updates_compact_telemetry_cache() -> None:
    moves = [(2, 1_000, 2_400, 50), (3, 1_500, 2_400, 50)]
    feedback = [
        {
            "servoId": 2,
            "rawPosition": 998,
            "moving": True,
            "packetAgeMs": 4,
            "voltageDeciVolts": 121,
            "temperatureC": 29,
        },
        {
            "servoId": 3,
            "rawPosition": 1_497,
            "moving": False,
            "packetAgeMs": 1,
            "voltageDeciVolts": 120,
            "temperatureC": 30,
        },
    ]
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _follow_set_receipt(moves, feedback))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.follow_set(moves)

    assert result["feedback"] == feedback
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 FOLLOW_SET 2,1000,2400,50 3,1500,2400,50\n",
    ]
    state = controller.transport_state()
    assert state["bus"] == "online"
    assert state["motionState"] == "moving"
    cached = {item["id"]: item for item in state["servosTelemetry"]}
    assert cached[2]["rawPosition"] == 998
    assert cached[2]["voltageVolts"] == 12.1
    assert cached[2]["packetAgeMs"] == 4
    assert cached[3]["temperatureC"] == 30


@pytest.mark.parametrize("stopped", [False, True])
def test_follow_set_accepts_feedback_phase_fail_closed_receipt(stopped: bool) -> None:
    moves = [(2, 1_000, 2_000, 40), (3, 1_500, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    failure = {
        "phase": "feedback",
        "failedIndex": 1,
        "stopped": stopped,
        "torqueState": "off",
        "latentCommands": False,
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "dispatchedFamilyCount": 1,
    }
    serial = ScriptedSerial(
        [_ok(1, hello), _err(2, "MOVE_SET_FAILED", failure)]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="MOVE_SET_FAILED"):
        controller.follow_set(moves)

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["lastMotionFailure"]["phase"] == "feedback"
    assert state["lastMotionFailure"]["failedIndex"] == 1
    assert state["lastMotionFailure"]["stopped"] is stopped
    assert state["motionState"] == ("stopped" if stopped else "blocked")


def test_follow_set_rejects_malformed_feedback_and_marks_motion_unconfirmed() -> None:
    moves = [(2, 1_000, 2_000, 40), (3, 1_500, 2_000, 40)]
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    feedback = [
        {
            "servoId": 2,
            "rawPosition": 1_000,
            "moving": False,
            "packetAgeMs": 0,
            "voltageDeciVolts": 120,
            "temperatureC": 29,
        },
        {
            "servoId": 2,
            "rawPosition": 1_500,
            "moving": False,
            "packetAgeMs": 0,
            "voltageDeciVolts": 120,
            "temperatureC": 29,
        },
    ]
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _follow_set_receipt(moves, feedback))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="did not match"):
        controller.follow_set(moves)

    _assert_unconfirmed_move_set_state(controller.transport_state())


def test_follow_set_validates_count_policy_and_sts_family_before_write() -> None:
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    invalid = [
        [(2, 1_000, 2_000, 40)],
        [(2, 1_000, 2_401, 40), (3, 1_500, 2_000, 40)],
        [(2, 1_000, 2_000, 81), (3, 1_500, 2_000, 40)],
    ]
    for moves in invalid:
        with pytest.raises(ValueError):
            controller.follow_set(moves)
    controller._servo_families[3] = "SCS"
    with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
        controller.follow_set(
            [(2, 1_000, 2_000, 40), (3, 1_500, 2_000, 40)]
        )

    assert serial.writes == [b"A1 1 HELLO\n"]


def test_replay_follow_set_returns_feedback_and_rejects_scs_member() -> None:
    controller = ReplayArmController.connected(
        servos=[
            _servo(2, raw_position=500, torque_state="on"),
            _servo(3, raw_position=600, torque_state="on"),
        ]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}

    result = controller.follow_set(
        [(2, 700, 2_000, 40), (3, 800, 2_000, 40)]
    )

    assert [row["rawPosition"] for row in result["feedback"]] == [700, 800]
    assert [item["rawPosition"] for item in controller.transport_state()["servosTelemetry"]] == [700, 800]
    controller.declare_family(3, "SCS")
    with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
        controller.follow_set(
            [(2, 900, 2_000, 40), (3, 900, 2_000, 40)]
        )


def test_follow_read_is_one_boot_bound_read_and_updates_telemetry_cache() -> None:
    feedback = [
        {
            "servoId": 2,
            "rawPosition": 1_050,
            "moving": True,
            "packetAgeMs": 3,
            "voltageDeciVolts": 121,
            "temperatureC": 29,
        },
        {
            "servoId": 3,
            "rawPosition": 1_460,
            "moving": True,
            "packetAgeMs": 1,
            "voltageDeciVolts": 120,
            "temperatureC": 30,
        },
    ]
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _follow_read_receipt(feedback))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.follow_read([2, 3])

    assert result["feedback"] == feedback
    assert serial.writes == [b"A1 1 HELLO\n", b"A1 2 FOLLOW_READ 2 3\n"]
    state = controller.transport_state()
    assert state["motionState"] == "moving"
    cached = {item["id"]: item for item in state["servosTelemetry"]}
    assert cached[2]["rawPosition"] == 1_050
    assert cached[2]["packetAgeMs"] == 3
    assert cached[3]["voltageVolts"] == 12.0


def test_follow_read_feedback_failure_is_a_read_error_not_motion_ambiguity() -> None:
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _err(2, "FEEDBACK_UNAVAILABLE", {"failedIndex": 1})]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller._heartbeat_stop.set()
    if controller._heartbeat_thread is not None:
        controller._heartbeat_thread.join(timeout=1)

    with pytest.raises(ControllerCommandError, match="FEEDBACK_UNAVAILABLE"):
        controller.follow_read([2, 3])

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["lastMotionFailure"] is None
    assert state["operatorInspectionRequired"] is False


def test_follow_read_rejects_wrong_order_feedback_and_faults_transport() -> None:
    feedback = [
        {
            "servoId": 3,
            "rawPosition": 1_050,
            "moving": True,
            "packetAgeMs": 1,
            "voltageDeciVolts": 120,
            "temperatureC": 29,
        },
        {
            "servoId": 2,
            "rawPosition": 1_460,
            "moving": True,
            "packetAgeMs": 1,
            "voltageDeciVolts": 120,
            "temperatureC": 30,
        },
    ]
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial(
        [_ok(1, hello), _ok(2, _follow_read_receipt(feedback))]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="did not match"):
        controller.follow_read([2, 3])

    assert controller.transport_state()["connection"] == "faulted"


def test_follow_read_validates_exact_unique_sts_pair_before_write() -> None:
    hello = _hello(firmware_version="arm-hat-2.7.0")
    _enable_follow_set(hello)
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    for invalid in ([], [2], [2, 2], [2, 3, 4], [2, 254]):
        with pytest.raises(ValueError):
            controller.follow_read(invalid)
    controller._servo_families[3] = "SCS"
    with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
        controller.follow_read([2, 3])

    assert serial.writes == [b"A1 1 HELLO\n"]


def test_replay_follow_read_is_read_only_and_preserves_order() -> None:
    controller = ReplayArmController.connected(
        servos=[
            {**_servo(2, raw_position=700, torque_state="on"), "moving": True},
            _servo(3, raw_position=800, torque_state="on"),
        ]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}
    before = deepcopy(controller.transport_state()["servosTelemetry"])

    result = controller.follow_read([3, 2])

    assert [row["servoId"] for row in result["feedback"]] == [3, 2]
    assert [row["rawPosition"] for row in result["feedback"]] == [800, 700]
    assert controller.transport_state()["servosTelemetry"] == before
    commands = [
        command for command in controller.commands
        if command["operation"] == "FOLLOW_READ"
    ]
    assert commands == [{"operation": "FOLLOW_READ", "servoIds": [3, 2]}]


def test_replay_move_set_preflights_all_moves_before_mutating_any_joint() -> None:
    controller = ReplayArmController.connected(
        servos=[
            _servo(2, raw_position=500, torque_state="on"),
            _servo(3, raw_position=600, torque_state="on"),
        ]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}

    with pytest.raises(ControllerCommandError, match="SERVO_NOT_FOUND"):
        controller.move_set([(2, 700, 200, 10), (9, 800, 200, 10)])

    telemetry = controller.transport_state()["servosTelemetry"]
    assert [item["rawPosition"] for item in telemetry] == [500, 600]
    move_set_commands = [
        command for command in controller.commands if command["operation"] == "MOVE_SET"
    ]
    assert len(move_set_commands) == 1


@pytest.mark.parametrize(
    "unsafe_fields",
    [
        {"operatingMode": 1},
        {"errors": ["overload"]},
        {"online": False},
        {"fresh": False},
        {"statusError": 1},
    ],
)
def test_replay_move_set_rejects_any_unusable_member_before_mutation(
    unsafe_fields: dict[str, object]
) -> None:
    unsafe = {**_servo(3, raw_position=600, torque_state="on"), **unsafe_fields}
    controller = ReplayArmController.connected(
        servos=[_servo(2, raw_position=500, torque_state="on"), unsafe]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}
    before = deepcopy(controller.transport_state()["servosTelemetry"])

    with pytest.raises(ControllerCommandError):
        controller.move_set([(2, 700, 100, 10), (3, 800, 100, 10)])

    assert controller.transport_state()["servosTelemetry"] == before


@pytest.mark.parametrize(
    "odometer",
    [
        None,
        {"revolutions": 0, "lastRaw": 600, "tracking": False, "valid": True},
        {"revolutions": 0, "lastRaw": 600, "tracking": True, "valid": False},
        {
            "revolutions": 0,
            "lastRaw": 600,
            "tracking": True,
            "valid": True,
            "stepOutstanding": True,
        },
        {
            "revolutions": 0,
            "lastRaw": 600,
            "tracking": True,
            "valid": True,
            "resyncNeeded": True,
        },
    ],
)
def test_replay_move_set_requires_complete_multi_turn_truth_before_mutation(
    odometer: dict[str, int | bool] | None,
) -> None:
    controller = ReplayArmController.connected(
        servos=[
            _servo(2, raw_position=500, torque_state="on"),
            _servo(3, raw_position=600, torque_state="on"),
        ]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}
    controller._multi_turn_servos.add(3)
    if odometer is not None:
        controller._odometers[3] = odometer
    before = deepcopy(controller.transport_state()["servosTelemetry"])

    with pytest.raises(ControllerCommandError, match="ODOMETER_UNAVAILABLE"):
        controller.move_set([(2, 700, 100, 10), (3, 8_000, 100, 10)])

    assert controller.transport_state()["servosTelemetry"] == before


def test_replay_move_set_applies_declared_scs_goal_range_before_mutation() -> None:
    controller = ReplayArmController.connected(
        servos=[
            _servo(2, raw_position=500, torque_state="on"),
            _servo(3, raw_position=600, torque_state="on"),
        ]
    )
    controller.declare_family(3, "SCS")
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}
    before = deepcopy(controller.transport_state()["servosTelemetry"])

    with pytest.raises(ControllerCommandError, match="OUT_OF_RANGE"):
        controller.move_set([(2, 700, 100, 10), (3, 2_000, 100, 10)])

    assert controller.transport_state()["servosTelemetry"] == before


def test_replay_move_set_mutates_together_and_logs_one_command() -> None:
    controller = ReplayArmController.connected(
        servos=[
            _servo(2, raw_position=500, torque_state="on"),
            _servo(3, raw_position=600, torque_state="on"),
        ]
    )
    controller._leases = {2: time.monotonic() + 1, 3: time.monotonic() + 1}

    result = controller.move_set([(2, 700, 200, 10), (3, 800, 250, 12)])

    assert result["count"] == 2
    assert result["bootId"] == "replay_boot_1"
    telemetry = controller.transport_state()["servosTelemetry"]
    assert [item["rawPosition"] for item in telemetry] == [700, 800]
    move_set_commands = [
        command for command in controller.commands if command["operation"] == "MOVE_SET"
    ]
    assert len(move_set_commands) == 1


def test_multi_turn_home_refuses_a_controller_without_native_absolute_capability() -> None:
    hello = _hello(firmware_version="arm-hat-2.3.2")
    serial = ScriptedSerial([_ok(1, hello)])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
        controller.home_multi_turn(1)

    assert [write.decode().split()[2] for write in serial.writes] == ["HELLO"]


def test_native_multi_turn_configuration_rejects_a_mode_three_response() -> None:
    firmware = "arm-hat-3.0.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [*hello["capabilities"], "multi_turn_absolute_v1"]
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, _status(firmware_version=firmware, servos=[_servo(1)])),
            _ok(3, {"servoId": 1, "multiTurn": True, "operatingMode": 3}),
            _ok(4, _status(firmware_version=firmware, servos=[_servo(1)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="multi-turn was not acknowledged"):
        controller.set_multi_turn(1, True)


def test_odometer_zero_preserves_a_nonzero_signed_native_coordinate() -> None:
    firmware = "arm-hat-3.0.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [*hello["capabilities"], "multi_turn_absolute_v1"]
    odometer = {
        "servoId": 1,
        "tracking": True,
        "valid": True,
        "stepMode": False,
        "revolutions": -2,
        "rawPosition": 3192,
        "multiTurnPosition": -5000,
        "sampleAgeMs": 0,
        "stepOutstanding": False,
        "countdownObserved": False,
        "resyncNeeded": False,
        "resyncCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, _status(firmware_version=firmware, servos=[_servo(1)])),
            _ok(3, odometer),
            _ok(4, _status(firmware_version=firmware, servos=[_servo(1)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    result = controller.odometer_zero(1)

    assert result["multiTurnPosition"] == -5000
    assert result["revolutions"] == -2


def test_serial_controller_uses_exact_commissioning_operation_tokens() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status()),
            _ok(
                3,
                {
                    "foundIds": [1],
                    "completeRange": {"minId": 0, "maxId": 20},
                },
            ),
            _ok(4, _status(servos=[_servo(1)])),
            _ok(5, _status(servos=[_servo(1)])),
            _ok(6, {"oldId": 1, "newId": 2, "verified": True}),
            _ok(7, _status(servos=[_servo(2)])),
            _ok(8, _status(servos=[_servo(2)])),
            _ok(
                9,
                {
                    "servoId": 2,
                    "leaseMs": 500,
                    "torqueState": "on",
                    "confirmed": True,
                },
            ),
            _ok(10, _status(servos=[_servo(2, torque_state="on")])),
            _ok(11, _status(servos=[_servo(2, torque_state="on")])),
            _ok(
                12,
                {
                    "servoId": 2,
                    "all": False,
                    "torqueState": "off",
                    "confirmed": True,
                },
            ),
            _ok(13, _status(servos=[_servo(2)])),
            _ok(14, _status(servos=[_servo(2)])),
            _ok(
                15,
                {
                    "servoId": None,
                    "all": True,
                    "torqueState": "off",
                    "confirmed": True,
                },
            ),
            _ok(16, _status(servos=[_servo(2)])),
            _ok(17, _status(servos=[_servo(2)])),
            _ok(
                18,
                {
                    "stopped": False,
                    "torqueState": "off",
                    "torqueOffConfirmed": True,
                    "reset": True,
                },
            ),
            _ok(19, _status(servos=[_servo(2)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    controller.scan(0, 20)
    controller.assign_id(1, 2)
    controller.torque_lease(2, 500)
    controller.torque_off(2)
    controller.torque_off()
    controller.reset()
    controller.close()

    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 SCAN 0 20\n",
        b"A1 4 STATUS\n",
        b"A1 5 STATUS\n",
        b"A1 6 ASSIGN_ID 1 2 SINGLE_SERVO\n",
        b"A1 7 STATUS\n",
        b"A1 8 STATUS\n",
        b"A1 9 TORQUE_LEASE 2 500\n",
        b"A1 10 STATUS\n",
        b"A1 11 STATUS\n",
        b"A1 12 TORQUE_OFF 2\n",
        b"A1 13 STATUS\n",
        b"A1 14 STATUS\n",
        b"A1 15 TORQUE_OFF ALL\n",
        b"A1 16 STATUS\n",
        b"A1 17 STATUS\n",
        b"A1 18 RESET INSPECTED\n",
        b"A1 19 STATUS\n",
    ]


def test_position_mode_restore_uses_fixed_tokens_and_verified_post_status() -> None:
    firmware_version = "arm-hat-1.0.2"
    mode_one = _servo(
        1,
        operating_mode=1,
        errors=["mode_not_position"],
    )
    mode_zero = _servo(1)
    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware_version)),
            _ok(
                2,
                _status(
                    firmware_version=firmware_version,
                    servos=[mode_one],
                ),
            ),
            _ok(
                3,
                {
                    "servoId": 1,
                    "previousOperatingMode": 1,
                    "operatingMode": 0,
                    "verified": True,
                    "locked": True,
                    "torqueState": "off",
                    "minimumPosition": 0,
                    "maximumPosition": 4095,
                },
            ),
            _ok(
                4,
                _status(
                    firmware_version=firmware_version,
                    servos=[mode_zero],
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    controller.start()
    result = controller.set_position_mode(1)
    controller.close()

    assert result["previousOperatingMode"] == 1
    assert result["operatingMode"] == 0
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 SET_POSITION_MODE 1 SINGLE_SERVO ST3215\n",
        b"A1 4 STATUS\n",
    ]


def test_hold_set_add_renew_remove_and_clear_use_one_desired_set() -> None:
    firmware_version = "arm-hat-1.1.0"

    def held_status(ids: list[int]) -> dict[str, object]:
        status = _status(
            firmware_version=firmware_version,
            servos=[
                _servo(1, torque_state="on" if 1 in ids else "off"),
                _servo(2, torque_state="on" if 2 in ids else "off"),
            ],
        )
        status["holdSet"] = (
            {"servoIds": ids, "remainingMs": 1_400} if ids else None
        )
        return status

    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware_version)),
            _ok(2, held_status([])),
            _ok(
                3,
                {
                    "servoIds": [1, 2],
                    "leaseMs": 1_500,
                    "holds": [
                        {"servoId": 1, "holdRawPosition": 1200},
                        {"servoId": 2, "holdRawPosition": 2200},
                    ],
                    "confirmed": True,
                },
            ),
            _ok(4, held_status([1, 2])),
            _ok(5, held_status([1, 2])),
            _ok(
                6,
                {
                    "servoIds": [1, 2],
                    "leaseMs": 1_500,
                    "holds": [
                        {"servoId": 1, "holdRawPosition": 1200},
                        {"servoId": 2, "holdRawPosition": 2200},
                    ],
                    "confirmed": True,
                },
            ),
            _ok(7, held_status([1, 2])),
            _ok(8, held_status([1, 2])),
            _ok(
                9,
                {
                    "servoIds": [2],
                    "leaseMs": 1_500,
                    "holds": [{"servoId": 2, "holdRawPosition": 2200}],
                    "confirmed": True,
                },
            ),
            _ok(10, held_status([2])),
            _ok(11, held_status([2])),
            _ok(
                12,
                {
                    "servoIds": [],
                    "leaseMs": 1_500,
                    "holds": [],
                    "confirmed": True,
                },
            ),
            _ok(13, held_status([])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    first = controller.set_hold_servos([1, 2])
    renewed = controller.set_hold_servos([1, 2])
    removed = controller.set_hold_servos([2])
    cleared = controller.set_hold_servos([])
    state = controller.transport_state()
    controller.close()

    assert first["holds"] == renewed["holds"]
    assert removed["servoIds"] == [2]
    assert cleared["servoIds"] == []
    assert state["heldServoIds"] == []
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 HOLD_SET 1500 1 2\n",
        b"A1 4 STATUS\n",
        b"A1 5 STATUS\n",
        b"A1 6 HOLD_SET 1500 1 2\n",
        b"A1 7 STATUS\n",
        b"A1 8 STATUS\n",
        b"A1 9 HOLD_SET 1500 2\n",
        b"A1 10 STATUS\n",
        b"A1 11 STATUS\n",
        b"A1 12 HOLD_SET 1500\n",
        b"A1 13 STATUS\n",
    ]


def test_hold_set_rejects_duplicates_before_serial_io() -> None:
    serial = ScriptedSerial([])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ValueError, match="unique"):
        controller.set_hold_servos([1, 1])

    assert serial.writes == []


def test_background_heartbeat_does_not_renew_a_hold_set() -> None:
    firmware_version = "arm-hat-1.1.0"
    off_status = _status(
        firmware_version=firmware_version,
        servos=[_servo(1)],
    )
    off_status["holdSet"] = None
    on_status = _status(
        firmware_version=firmware_version,
        servos=[_servo(1, torque_state="on")],
    )
    on_status["holdSet"] = {"servoIds": [1], "remainingMs": 1_400}
    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware_version)),
            _ok(2, off_status),
            _ok(
                3,
                {
                    "servoIds": [1],
                    "leaseMs": 1_500,
                    "holds": [{"servoId": 1, "holdRawPosition": 2048}],
                    "confirmed": True,
                },
            ),
            _ok(4, on_status),
            _ok(
                5,
                {
                    "watchdogFresh": True,
                    "hostAgeMs": 0,
                    "stopped": False,
                    "motionState": "ready",
                    "torqueState": "on",
                    "busState": "online",
                    "servosState": "online",
                    "servoCount": 1,
                },
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller.set_hold_servos([1])

    deadline = time.monotonic() + 1.0
    while b"A1 5 HEARTBEAT\n" not in serial.writes and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()

    assert b"A1 5 HEARTBEAT\n" in serial.writes
    assert sum(b" HOLD_SET " in request for request in serial.writes) == 1


def test_position_mode_restore_rejects_old_firmware_without_writing() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                _hello(
                    position_mode_capability=False,
                    firmware_version="arm-hat-1.0.0",
                ),
            )
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    controller.start()
    with pytest.raises(ControllerCommandError, match="UNSUPPORTED"):
        controller.set_position_mode(1)
    controller.close()

    assert serial.writes == [b"A1 1 HELLO\n"]


def test_background_heartbeat_is_sent_within_250_ms() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, {"torqueState": "off"}),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    deadline = time.monotonic() + 0.249
    while len(serial.writes) < 2 and time.monotonic() < deadline:
        time.sleep(0.005)
    controller.close()

    assert serial.writes[:2] == [b"A1 1 HELLO\n", b"A1 2 HEARTBEAT\n"]


def test_one_refused_heartbeat_does_not_kill_the_background_loop() -> None:
    """A command-level refusal is transient; supervision must try again."""

    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-2.3.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            b"A1 2 ERR BUS_ERROR {}\n",
            _ok(3, {"torqueState": "off"}),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    deadline = time.monotonic() + 0.7
    while len(serial.writes) < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.close()

    assert serial.writes[:3] == [
        b"A1 1 HELLO\n",
        b"A1 2 HEARTBEAT\n",
        b"A1 3 HEARTBEAT\n",
    ]


def test_transport_timeout_makes_torque_unknown() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            )
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(Exception, match="timed out"):
        controller.scan(0, 20)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["torqueState"] == "unknown"
    assert serial.is_open is False
    controller.close()


def test_controller_accepts_honest_unknown_boot_torque_only_after_boot_off_attempt() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "blocked",
                    "torqueState": "unknown",
                    "bootTorqueOffSent": True,
                },
            )
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    controller.start()

    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "blocked"
    assert state["torqueState"] == "unknown"
    controller.close()


def test_controller_rejects_unknown_boot_torque_without_boot_off_attempt() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "blocked",
                    "torqueState": "unknown",
                },
            )
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    with pytest.raises(ControllerProtocolError, match="boot torque-off"):
        controller.start()

    assert serial.is_open is False


def test_fault_requires_explicit_reconnect_and_uses_a_new_transport() -> None:
    first = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_first",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            )
        ]
    )
    second = ScriptedSerial(
        [
            _ok(
                3,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_second",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "blocked",
                    "torqueState": "unknown",
                    "bootTorqueOffSent": True,
                },
            )
        ]
    )
    transports = iter((first, second))
    controller = SerialArmController(
        "loop", serial_factory=lambda **_: next(transports)
    )
    controller.start()

    with pytest.raises(Exception, match="timed out"):
        controller.scan(0, 20)

    time.sleep(0.21)
    assert first.is_open is False
    assert second.writes == []

    hello = controller.reconnect()

    assert hello["bootId"] == "boot_second"
    assert second.input_resets == 1
    assert second.writes == [b"A1 3 HELLO\n"]
    controller.close()


def test_status_carries_operating_mode_and_omits_unscaled_current_raw() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(
                2,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "motionState": "ready",
                    "torqueState": "off",
                    "busState": "online",
                    "servosState": "online",
                    "servoCount": 1,
                    "servos": [
                        {
                            "id": 1,
                            "rawPosition": 2048,
                            "speed": 0,
                            "load": 0,
                            "voltageVolts": 12.0,
                            "temperatureC": 28,
                            "moving": False,
                            "currentRaw": 37,
                            "torqueState": "off",
                            "packetAgeMs": 4,
                            "errors": [],
                            "operatingMode": 0,
                        }
                    ],
                },
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    state = controller.status()

    assert state["servosTelemetry"][0]["operatingMode"] == 0
    assert "currentRaw" not in state["servosTelemetry"][0]
    assert "currentMilliamps" not in state["servosTelemetry"][0]
    controller.close()


@pytest.mark.parametrize(
    ("encoded_position", "expected_raw_position"),
    [
        (0x831E, 3298),  # signed-magnitude -798, exact live Base failure
        (5000, 904),     # positive native coordinate beyond one turn
    ],
)
def test_status_normalizes_declared_sts_native_position_without_inventing_odometer_truth(
    encoded_position: int,
    expected_raw_position: int,
) -> None:
    firmware = "arm-hat-2.7.0"
    hello = _hello(firmware_version=firmware)
    hello["capabilities"] = [
        *hello["capabilities"],
        "servo_family",
        "multi_turn_absolute_v1",
    ]
    native_status = _status(
        firmware_version=firmware,
        servos=[_servo(1, raw_position=encoded_position, operating_mode=0)],
        motion_state="ready",
    )
    untrusted_odometer = {
        "servoId": 1,
        "tracking": False,
        "valid": False,
        "stepMode": False,
        "revolutions": 0,
        "rawPosition": 0,
        "multiTurnPosition": 0,
        "sampleAgeMs": 5_350_000,
        "stepOutstanding": False,
        "countdownObserved": False,
        "resyncNeeded": False,
        "resyncCount": 0,
    }
    serial = ScriptedSerial(
        [
            _ok(1, hello),
            _ok(2, {"servoId": 1, "family": "sts"}),
            _ok(3, native_status),
            _ok(4, native_status),
            _ok(5, untrusted_odometer),
            _ok(6, native_status),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.declare_family(1, "STS")
    controller.declare_native_multi_turn_servos([1])
    controller.start()

    state = controller.status()
    assert state["servosTelemetry"][0]["rawPosition"] == expected_raw_position

    truth = controller.odometer_read(1)
    assert truth["tracking"] is False
    assert truth["valid"] is False
    assert truth["multiTurnPosition"] == 0
    assert controller.transport_state()["servosTelemetry"][0]["rawPosition"] == expected_raw_position
    controller.close()


@pytest.mark.parametrize(
    ("declared_family", "native_capability", "operating_mode", "encoded_position"),
    [
        (None, True, 0, 0x831E),
        ("SCS", True, 0, 0x831E),
        ("STS", False, 0, 0x831E),
        ("STS", True, 3, 0x831E),
        ("STS", True, 0, 0xF800),  # magnitude 30720 exceeds native limit
    ],
)
def test_extended_status_position_requires_complete_native_sts_evidence(
    declared_family: str | None,
    native_capability: bool,
    operating_mode: int,
    encoded_position: int,
) -> None:
    with pytest.raises(ControllerProtocolError, match="invalid raw position"):
        SerialArmController._validated_servo(
            _servo(1, raw_position=encoded_position, operating_mode=operating_mode),
            declared_family=declared_family,
            native_multi_turn_capable=native_capability,
        )


def test_nudge_requires_controller_measured_direction_and_torque_off_evidence() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(2)])),
            _ok(
                3,
                {
                    "proposalId": "proposal_abc123",
                    "servoId": 2,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "deltaTicks": 23,
                    "speed": 80,
                    "acceleration": 8,
                    "expiresInMs": 10000,
                },
            ),
            _ok(4, _status(servos=[_servo(2)])),
            _ok(5, _status(servos=[_servo(2)])),
            _ok(
                6,
                {
                    "proposalId": "proposal_abc123",
                    "servoId": 2,
                    "completed": True,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "rawPosition": 2022,
                    "measuredDeltaTicks": 22,
                    "positionErrorTicks": -1,
                    "torqueState": "off",
                    "evidenceId": "obs_nudge_abc123",
                },
            ),
            _ok(7, _status(servos=[_servo(2, raw_position=2022)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    proposal = controller.prepare_nudge(2, 23, 80, 8)
    result = controller.execute_nudge(str(proposal["proposalId"]))
    controller.close()

    assert result["completed"] is True
    assert result["measuredDeltaTicks"] == 22
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 PREPARE_NUDGE 2 23 80 8\n",
        b"A1 4 STATUS\n",
        b"A1 5 STATUS\n",
        b"A1 6 EXECUTE_NUDGE proposal_abc123\n",
        b"A1 7 STATUS\n",
    ]


def test_prepare_nudge_rejects_a_target_that_wraps_across_raw_boundary() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(2)])),
            _ok(
                3,
                {
                    "proposalId": "proposal_abc123",
                    "servoId": 2,
                    "startRawPosition": 4090,
                    "targetRawPosition": 17,
                    "deltaTicks": 23,
                    "speed": 80,
                    "acceleration": 8,
                    "expiresInMs": 10000,
                },
            ),
            _ok(4, _status(servos=[_servo(2)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="invalid nudge proposal"):
        controller.prepare_nudge(2, 23, 80, 8)

    assert controller.transport_state()["connection"] == "faulted"
    assert serial.is_open is False


def test_nudge_rejects_reversed_measured_motion_even_if_controller_says_complete() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(2)])),
            _ok(
                3,
                {
                    "proposalId": "proposal_abc123",
                    "servoId": 2,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "deltaTicks": 23,
                    "speed": 80,
                    "acceleration": 8,
                    "expiresInMs": 10000,
                },
            ),
            _ok(4, _status(servos=[_servo(2)])),
            _ok(5, _status(servos=[_servo(2)])),
            _ok(
                6,
                {
                    "proposalId": "proposal_abc123",
                    "servoId": 2,
                    "completed": True,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "rawPosition": 1990,
                    "measuredDeltaTicks": -10,
                    "positionErrorTicks": -33,
                    "torqueState": "off",
                    "evidenceId": "obs_nudge_abc123",
                },
            ),
            _ok(7, _status(servos=[_servo(2, raw_position=1990)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller.prepare_nudge(2, 23, 80, 8)

    with pytest.raises(ControllerProtocolError, match="safely verified"):
        controller.execute_nudge("proposal_abc123")

    assert controller.transport_state()["torqueState"] == "unknown"
    controller.close()


def test_prepared_nudge_ledger_cannot_survive_controller_reboot_status() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_first",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(boot_id="boot_first", servos=[_servo(2)])),
            _ok(
                3,
                {
                    "proposalId": "proposal_before_reboot",
                    "servoId": 2,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "deltaTicks": 23,
                    "speed": 80,
                    "acceleration": 8,
                    "expiresInMs": 10000,
                },
            ),
            _ok(4, _status(boot_id="boot_second", servos=[_servo(2)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    with pytest.raises(ControllerProtocolError, match="explicit reconnect"):
        controller.prepare_nudge(2, 23, 80, 8)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["motionState"] == "blocked"
    assert state["torqueState"] == "unknown"
    assert serial.is_open is False
    with pytest.raises(ControllerCommandError, match="PROPOSAL_USED_OR_UNKNOWN"):
        controller.execute_nudge("proposal_before_reboot")
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 PREPARE_NUDGE 2 23 80 8\n",
        b"A1 4 STATUS\n",
    ]


def test_stop_preserves_latch_when_torque_readback_is_unknown() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(2)])),
            _ok(
                3,
                {
                    "proposalId": "proposal_before_stop",
                    "servoId": 2,
                    "startRawPosition": 2000,
                    "targetRawPosition": 2023,
                    "deltaTicks": 23,
                    "speed": 80,
                    "acceleration": 8,
                    "expiresInMs": 10000,
                },
            ),
            _ok(4, _status(servos=[_servo(2)])),
            _ok(
                5,
                {
                    "stopped": True,
                    "torqueState": "unknown",
                    "confirmed": False,
                    "torqueOffBroadcastSent": True,
                },
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    controller.prepare_nudge(2, 23, 80, 8)

    stopped = controller.stop()

    assert stopped == {
        "stopped": True,
        "torqueState": "unknown",
        "confirmed": False,
        "torqueOffBroadcastSent": True,
    }
    state = controller.transport_state()
    assert state["connection"] == "online"
    assert state["motionState"] == "stopped"
    assert state["torqueState"] == "unknown"
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "EXPLICIT_STOP"
    with pytest.raises(ControllerCommandError, match="PROPOSAL_USED_OR_UNKNOWN"):
        controller.execute_nudge("proposal_before_stop")
    with pytest.raises(ControllerCommandError, match="OPERATOR_INSPECTION_REQUIRED"):
        controller.move(2, 2023, 80, 8)
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 PREPARE_NUDGE 2 23 80 8\n",
        b"A1 4 STATUS\n",
        b"A1 5 STOP\n",
    ]
    controller.close()


def test_empty_completed_scan_removes_stale_servo_telemetry() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(
                2,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "motionState": "ready",
                    "torqueState": "off",
                    "busState": "online",
                    "servosState": "online",
                    "servoCount": 1,
                    "servos": [
                        {
                            "id": 1,
                            "rawPosition": 2048,
                            "speed": 0,
                            "load": 0,
                            "voltageVolts": 12.0,
                            "temperatureC": 28,
                            "moving": False,
                            "torqueState": "off",
                            "packetAgeMs": 4,
                            "errors": [],
                            "operatingMode": 0,
                        }
                    ],
                },
            ),
            _ok(3, _status(servos=[_servo(1)])),
            _ok(
                4,
                {
                    "foundIds": [],
                    "completeRange": {"minId": 0, "maxId": 253},
                    "collisionSuspected": False,
                },
            ),
            _ok(5, _status()),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    assert [servo["id"] for servo in controller.status()["servosTelemetry"]] == [1]

    controller.scan(0, 253)

    state = controller.transport_state()
    assert state["servosTelemetry"] == []
    assert state["servoCount"] == 0
    assert state["servos"] == "none_found"
    assert state["bus"] == "unknown"
    assert state["torqueState"] == "unknown"
    controller.close()


def test_capture_evidence_is_rejected_if_controller_reboots_before_post_status() -> None:
    capture = {
        **_servo(3, raw_position=2300),
        "sampleCount": 5,
        "variationTicks": 2,
        "positionMedian": 2300,
        "evidenceId": "obs_capture_before_reboot",
    }
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_first",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(boot_id="boot_first", servos=[_servo(3)])),
            _ok(3, capture),
            _ok(4, _status(boot_id="boot_second", servos=[_servo(3)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="explicit reconnect"):
        controller.capture(3)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["servosTelemetry"][0]["rawPosition"] == 2048
    assert serial.is_open is False
    assert serial.writes == [
        b"A1 1 HELLO\n",
        b"A1 2 STATUS\n",
        b"A1 3 CAPTURE 3\n",
        b"A1 4 STATUS\n",
    ]


def test_capture_rejects_noncanonical_torque_on_success() -> None:
    capture = {
        **_servo(3, torque_state="on"),
        "sampleCount": 5,
        "variationTicks": 1,
        "positionMedian": 2048,
        "evidenceId": "obs_capture_torque_on",
    }
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(3)])),
            _ok(3, capture),
            _ok(4, _status(servos=[_servo(3, torque_state="on")])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="invalid capture evidence"):
        controller.capture(3)

    assert controller.transport_state()["connection"] == "faulted"


def test_partial_empty_scan_preserves_fresh_out_of_range_inventory() -> None:
    outside = _servo(100)
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[outside], motion_state="ready")),
            _ok(
                3,
                {
                    "foundIds": [],
                    "completeRange": {"minId": 0, "maxId": 20},
                    "collisionSuspected": False,
                },
            ),
            _ok(4, _status(servos=[outside], motion_state="ready")),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    controller.scan(0, 20)

    state = controller.transport_state()
    # The collision verdict rides along so the panel can distinguish an empty bus
    # from two servos sharing an id, which look identical from the outside.
    assert state["lastScan"] == {
        "minId": 0,
        "maxId": 20,
        "foundIds": [],
        "pingFoundIds": [],
        "telemetryUnavailableIds": [],
        "telemetryRecoveredIds": [],
        "collisionSuspected": False,
        "collisionId": None,
        "busJammed": False,
        # Absent on a controller too old to report the byte census.
        "busNoise": None,
    }
    assert state["servoCount"] == 1
    assert state["servos"] == "online"
    assert state["bus"] == "online"
    assert state["torqueState"] == "off"
    assert [item["id"] for item in state["servosTelemetry"]] == [100]
    controller.close()


def test_addressed_torque_off_keeps_other_servo_on_in_aggregate() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(
                2,
                _status(
                    servos=[
                        _servo(1, torque_state="on"),
                        _servo(2, torque_state="on"),
                    ]
                ),
            ),
            _ok(
                3,
                {
                    "servoId": 1,
                    "all": False,
                    "torqueState": "off",
                    "confirmed": True,
                },
            ),
            _ok(
                4,
                _status(
                    servos=[_servo(1), _servo(2, torque_state="on")],
                    motion_state="blocked",
                ),
            ),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    controller.torque_off(1)

    state = controller.transport_state()
    assert state["torqueState"] == "on"
    assert state["motionState"] == "blocked"
    assert [item["torqueState"] for item in state["servosTelemetry"]] == ["off", "on"]
    controller.close()


def test_torque_lease_rejects_on_payload_when_post_status_is_off() -> None:
    serial = ScriptedSerial(
        [
            _ok(
                1,
                {
                    "controllerId": "hat-a-001",
                    "bootId": "boot_abc",
                    "firmwareVersion": "arm-hat-1.0.0",
                    "protocolVersion": 1,
                    "state": "disarmed",
                },
            ),
            _ok(2, _status(servos=[_servo(2)])),
            _ok(
                3,
                {
                    "servoId": 2,
                    "leaseMs": 500,
                    "torqueState": "on",
                    "confirmed": True,
                },
            ),
            _ok(4, _status(servos=[_servo(2)])),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()

    with pytest.raises(ControllerProtocolError, match="torque lease was not verified"):
        controller.torque_lease(2, 500)

    state = controller.transport_state()
    assert state["connection"] == "faulted"
    assert state["torqueState"] == "unknown"


def test_connect_pushes_the_gateway_motion_policy_to_a_runtime_config_controller() -> None:
    from robot_gateway.serial_arm_controller import MOTION_POLICY

    hello = _hello(firmware_version="arm-hat-2.0.0")
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("runtime_config")

    replies = [_ok(1, hello)]
    for index, _ in enumerate(MOTION_POLICY, start=2):
        replies.append(_ok(index, {"positionToleranceTicks": 8}))
    serial = ScriptedSerial(replies)
    controller = SerialArmController(port="test-port", serial_factory=lambda **_: serial)
    controller.start()
    controller.close()

    sent = [write.decode() for write in serial.writes]
    # Every tunable that used to require a reflash is pushed from the gateway.
    for key, value in MOTION_POLICY.items():
        assert any(f"CONFIG {key} {value}" in line for line in sent), key


def test_connect_skips_policy_push_when_the_controller_lacks_runtime_config() -> None:
    serial = ScriptedSerial([_ok(1, _hello(firmware_version="arm-hat-1.2.0"))])
    controller = SerialArmController(port="test-port", serial_factory=lambda **_: serial)
    controller.start()
    controller.close()

    sent = [write.decode() for write in serial.writes]
    assert not any("CONFIG" in line for line in sent)


def test_declared_servo_family_is_replayed_after_a_reconnect() -> None:
    """The SC09's dialect must survive the controller rebooting.

    Family lives in the controller's RAM, so a reset silently drops back to
    reading the SC09 as an ST3215 -- byte-swapped position, a telemetry read
    that runs off the end of its register table. Nothing reports that as an
    error, so the declaration has to be re-pushed on every connect.
    """

    hello = _hello(firmware_version="arm-hat-2.0.0")
    capabilities = hello["capabilities"]
    assert isinstance(capabilities, list)
    capabilities.append("servo_family")

    serial = ScriptedSerial([_ok(1, hello), _ok(2, {"servoId": 4, "family": "scs"})])
    controller = SerialArmController(port="test-port", serial_factory=lambda **_: serial)
    controller.start()
    controller.declare_family(4, "SCS")
    assert any(b"FAMILY 4 SCS" in write for write in serial.writes)

    controller.close()
    reconnected = ScriptedSerial(
        [_ok(3, hello), _ok(4, {"servoId": 4, "family": "scs"})]
    )
    controller._serial_factory = lambda **_: reconnected
    controller.start()
    controller.close()

    assert any(b"FAMILY 4 SCS" in write for write in reconnected.writes)


def test_servo_family_is_not_pushed_to_a_controller_that_cannot_take_it() -> None:
    serial = ScriptedSerial([_ok(1, _hello(firmware_version="arm-hat-1.2.0"))])
    controller = SerialArmController(port="test-port", serial_factory=lambda **_: serial)
    controller.declare_family(4, "SCS")
    controller.start()
    controller.close()

    assert not any(b"FAMILY" in write for write in serial.writes)


@pytest.mark.parametrize(
    "family_reply",
    [
        _err(2, "BAD_ARGS", {}),
        _ok(2, {"servoId": 4, "family": "sts"}),
        _ok(2, {"servoId": 5, "family": "scs"}),
    ],
)
def test_advertised_servo_family_requires_an_exact_receipt(
    family_reply: bytes,
) -> None:
    hello = _hello(firmware_version="arm-hat-2.5.0")
    hello["capabilities"] = [
        *hello["capabilities"],
        "servo_family",
        "move_set_v1",
    ]
    serial = ScriptedSerial([_ok(1, hello), family_reply])
    controller = SerialArmController(port="test-port", serial_factory=lambda **_: serial)
    controller.declare_family(4, "SCS")

    with pytest.raises(ControllerProtocolError, match="servo family"):
        controller.start()

    assert controller.transport_state()["connection"] == "faulted"
    assert not any(b"MOVE_SET" in write for write in serial.writes)


def test_the_hold_limit_matches_the_firmware_on_both_send_and_parse() -> None:
    """One number, not three copies of it.

    The firmware's MAX_HOLD_SERVOS and the API's MAX_HELD were raised to four
    while this transport still said three, hard-coded in two separate places --
    so holding all four joints was refused with "at most three servos can be
    held", and the browser proxy relabelled the resulting 500 as "gateway
    unavailable", pointing every investigation at the network.
    """

    from robot_gateway.serial_arm_controller import MAX_HOLD_SERVOS
    from robot_gateway.simple_arm_api import MAX_HELD

    assert MAX_HOLD_SERVOS == 4
    assert MAX_HELD == MAX_HOLD_SERVOS

    serial = ScriptedSerial([])
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)

    # One past the limit is still refused, and still before any serial IO.
    with pytest.raises(ValueError, match="at most 4"):
        controller.set_hold_servos([1, 2, 3, 4, 5])

    # Four is NOT refused on the count: it gets past the length check and stops
    # later, on this controller never having handshaken. That is the regression.
    with pytest.raises(ControllerCommandError):
        controller.set_hold_servos([1, 2, 3, 4])

    assert serial.writes == []


def test_hold_set_accepts_a_multi_turn_servo_reporting_step_mode() -> None:
    """A multi-turn joint holds in step mode (3), not position mode (0).

    Regression: this verifier demanded operatingMode == 0, so arming the base
    for multi-turn made every hold fail AFTER the controller had already
    energised the servo -- torque on, hold unregistered, link faulted. Wheel
    mode still has to be refused, which is what the second half asserts.
    """

    firmware_version = "arm-hat-2.1.0"

    def held_status(mode: int) -> dict[str, object]:
        status = _status(
            firmware_version=firmware_version,
            servos=[_servo(1, torque_state="on", operating_mode=mode)],
        )
        status["holdSet"] = {"servoIds": [1], "remainingMs": 1_400}
        return status

    def hold_payload() -> dict[str, object]:
        return {
            "servoIds": [1],
            "leaseMs": 1_500,
            "holds": [{"servoId": 1, "holdRawPosition": 1200}],
            "confirmed": True,
        }

    serial = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware_version)),
            _ok(2, _status(firmware_version=firmware_version, servos=[_servo(1)])),
            _ok(3, hold_payload()),
            _ok(4, held_status(3)),
            _ok(5, held_status(3)),
        ]
    )
    controller = SerialArmController("loop", serial_factory=lambda **_: serial)
    controller.start()
    result = controller.set_hold_servos([1])
    controller.close()
    assert result["servoIds"] == [1]

    wheel = ScriptedSerial(
        [
            _ok(1, _hello(firmware_version=firmware_version)),
            _ok(2, _status(firmware_version=firmware_version, servos=[_servo(1)])),
            _ok(3, hold_payload()),
            _ok(4, held_status(1)),
            _ok(5, held_status(1)),
        ]
    )
    spinning = SerialArmController("loop", serial_factory=lambda **_: wheel)
    spinning.start()
    with pytest.raises(ControllerProtocolError):
        spinning.set_hold_servos([1])
    spinning.close()


def test_hold_set_accepts_a_wrap_counted_hold_position() -> None:
    """A multi-turn joint's hold is in its GOAL frame, not one turn of encoder.

    Regression: the base past ~90 deg reports a negative or >4095 wrap-counted
    hold position. Bounding it to 0..4095 raised a protocol fault on every hold
    -- and holds renew continuously, so the arm died a second after it worked.
    """

    firmware_version = "arm-hat-2.1.0"

    def held_status() -> dict[str, object]:
        status = _status(
            firmware_version=firmware_version,
            servos=[_servo(1, torque_state="on", operating_mode=3)],
        )
        status["holdSet"] = {"servoIds": [1], "remainingMs": 1_400}
        return status

    def script(hold_position: int) -> ScriptedSerial:
        return ScriptedSerial(
            [
                _ok(1, _hello(firmware_version=firmware_version)),
                _ok(2, _status(firmware_version=firmware_version, servos=[_servo(1)])),
                _ok(
                    3,
                    {
                        "servoIds": [1],
                        "leaseMs": 1_500,
                        "holds": [{"servoId": 1, "holdRawPosition": hold_position}],
                        "confirmed": True,
                    },
                ),
                _ok(4, held_status()),
                _ok(5, held_status()),
            ]
        )

    for hold_position in (-1282, 5461):
        controller = SerialArmController("loop", serial_factory=lambda **_: script(hold_position))
        controller.start()
        result = controller.set_hold_servos([1])
        controller.close()
        assert result["holds"][0]["holdRawPosition"] == hold_position

    # Beyond the servo's own goal range is still garbage and must fault.
    beyond = SerialArmController("loop", serial_factory=lambda **_: script(40_000))
    beyond.start()
    with pytest.raises(ControllerProtocolError):
        beyond.set_hold_servos([1])
    beyond.close()
