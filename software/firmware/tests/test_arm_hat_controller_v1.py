import json
import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest
from copy import deepcopy


ROOT = pathlib.Path(__file__).resolve().parents[2]
FIRMWARE_ROOT = ROOT / "firmware"
TEST_SOURCE = FIRMWARE_ROOT / "tests" / "arm_hat_controller_host_test.cpp"
RUNTIME_TEST_SOURCE = (
    FIRMWARE_ROOT / "tests" / "arm_hat_runtime_host_test.cpp"
)
ARDUINO_STUB_INCLUDE_DIR = FIRMWARE_ROOT / "tests" / "arduino_stubs"
LIBRARY_SOURCE = (
    FIRMWARE_ROOT
    / "libraries"
    / "ArmHatController"
    / "src"
    / "ArmHatProtocol.cpp"
)
INCLUDE_DIR = LIBRARY_SOURCE.parent
FIXTURE_PATH = (
    FIRMWARE_ROOT / "tests" / "fixtures" / "arm_hat_controller_v1.json"
)
RUNTIME_SOURCE = INCLUDE_DIR / "ArmHatController.cpp"
RUNTIME_HEADER = INCLUDE_DIR / "ArmHatController.h"
SKETCH_SOURCE = (
    FIRMWARE_ROOT
    / "esp32"
    / "arm_hat_controller"
    / "arm_hat_controller.ino"
)


class ArmHatControllerHostTest(unittest.TestCase):
    def test_move_set_uses_no_latent_grouped_sync_write_and_is_boot_bound(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        header = RUNTIME_HEADER.read_text(encoding="utf-8")

        hello_start = source.index("void ArmHatRuntime::handleHello")
        hello_end = source.index("void ArmHatRuntime::handleHeartbeat", hello_start)
        hello = source[hello_start:hello_end]
        self.assertIn("move_set_v1", hello)

        move_set_start = source.index("void ArmHatRuntime::handleMoveSet")
        move_set_end = source.index("void ArmHatRuntime::handleOdometerZero", move_set_start)
        move_set = source[move_set_start:move_set_end]
        self.assertIn("syncWritePositions", move_set)
        self.assertIn("dialect_grouped_sync_write", move_set)
        self.assertIn("crossFamilyAtomic", move_set)
        self.assertIn("verifyPositionCommand", move_set)
        self.assertIn('\\"bootId\\"', move_set)
        self.assertIn("failMoveSetClosed", move_set)
        flush_start = source.index("bool ArmHatRuntime::stopMoveSetMembersAndConfirm")
        flush_end = source.index("void ArmHatRuntime::failMoveSetClosed", flush_start)
        flush = source[flush_start:flush_end]
        self.assertIn("torqueOffAndConfirm", flush)
        self.assertNotIn("triggerRegisteredWrites", flush)

        protocol = (INCLUDE_DIR / "ArmHatProtocol.h").read_text(encoding="utf-8")
        self.assertIn("SCS_INSTRUCTION_SYNC_WRITE", protocol)
        self.assertNotIn("SCS_INSTRUCTION_REG_WRITE", protocol)
        self.assertNotIn("SCS_INSTRUCTION_ACTION", protocol)
        self.assertIn("syncWritePositions", header)
        self.assertNotIn("stagePosition", header)
        self.assertNotIn("triggerRegisteredWrites", header)

    def test_native_absolute_multiturn_is_the_only_base_drive_capability(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        hello_start = source.index("void ArmHatRuntime::handleHello")
        hello_end = source.index("void ArmHatRuntime::handleHeartbeat", hello_start)
        hello = source[hello_start:hello_end]
        self.assertIn("multi_turn_absolute_v1", hello)
        self.assertNotIn("multi_turn_step_drive", hello)
        self.assertNotIn("multi_turn_step_truth", hello)

        move_start = source.index("void ArmHatRuntime::handleMove")
        move_end = source.index("void ArmHatRuntime::handleOdometerZero", move_start)
        move = source[move_start:move_end]
        self.assertIn("servo->operatingMode != 0", move)
        self.assertIn("bus_.writePosition", move)
        self.assertNotIn("setMultiTurnGoal", move)

    def test_sketch_idle_loop_yields_after_servicing_safety(self):
        source = SKETCH_SOURCE.read_text(encoding="utf-8")
        loop = source[source.index("void loop()") :]

        poll_at = loop.index("armController.poll()")
        tick_at = loop.index("armController.tick()")
        yield_at = loop.index("delay(1)")

        self.assertLess(poll_at, tick_at)
        self.assertLess(tick_at, yield_at)

    def test_normal_multiturn_sample_gaps_do_not_destroy_position_truth(self):
        """One slow STATUS must not turn a healthy Base into UNKNOWN.

        Live STATUS traffic takes roughly 122 ms. The former 25 ms free-mode
        cutoff and single-read-failure invalidation therefore guaranteed that a
        normally resynchronised Base would lose truth immediately after a move.
        Native mode 0 has signed extended feedback. A missed sample makes that
        observation stale, not unknowable; an actual offline transition is
        handled separately by refreshServo.
        """

        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        start = source.index("void ArmHatRuntime::sampleOdometers")
        end = source.index("bool ArmHatRuntime::confirmTorqueOff", start)
        sampler = source[start:end]

        self.assertNotIn("ODOMETER_FREE_MAX_GAP_MS", sampler)
        self.assertIn(
            "servo.operatingMode == 0",
            sampler,
        )
        failed_read = sampler[
            sampler.index("if (result != BusResult::OK"):
            sampler.index("const uint16_t raw", sampler.index("if (result != BusResult::OK"))
        ]
        self.assertNotIn("forgetMultiTurnTruth", failed_read)
        self.assertNotIn("servo.odometerValid = false", failed_read)

    def test_native_feedback_is_decoded_as_signed_magnitude(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        decoder_start = source.index("bool decodeExtendedPosition")
        decoder_end = source.index("bool nativeExtendedPositionContinuous", decoder_start)
        decoder = source[decoder_start:decoder_end]
        self.assertIn("decodeSignedMagnitude16", decoder)
        self.assertIn("ST3215_MULTI_TURN_MAX", decoder)
        guard_start = decoder_end
        guard_end = source.index("bool storeExtendedPosition", guard_start)
        guard = source[guard_start:guard_end]
        self.assertIn("NATIVE_MULTI_TURN_REVOLUTION_MIN_MS", guard)
        self.assertIn("MULTI_TURN_ENCODER_TICKS", guard)

    def test_native_absolute_multiturn_runs_through_the_production_runtime(self):
        """Signed feedback, idempotent goals, config and frame loss are real."""

        with tempfile.TemporaryDirectory(prefix="arm-hat-runtime-") as temporary:
            temporary_path = pathlib.Path(temporary)
            executable = temporary_path / (
                "arm_hat_runtime_test.exe" if os.name == "nt" else "arm_hat_runtime_test"
            )
            sources = (RUNTIME_TEST_SOURCE, RUNTIME_SOURCE, LIBRARY_SOURCE)

            if os.name == "nt":
                vcvars = pathlib.Path(
                    r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
                    r"\VC\Auxiliary\Build\vcvars64.bat"
                )
                if not vcvars.exists():
                    self.skipTest("MSVC Build Tools 2022 are not installed")
                command = (
                    f'call "{vcvars}" >nul && '
                    f'cl /nologo /std:c++17 /EHsc /I"{ARDUINO_STUB_INCLUDE_DIR}" '
                    f'/I"{INCLUDE_DIR}" '
                    + " ".join(f'"{source}"' for source in sources)
                    + f' /Fe:"{executable}"'
                )
                compile_result = subprocess.run(
                    command,
                    cwd=temporary,
                    text=True,
                    capture_output=True,
                    check=False,
                    shell=True,
                )
            else:
                compiler = shutil.which("c++") or shutil.which("g++")
                if compiler is None:
                    self.skipTest("No C++17 compiler is installed")
                compile_result = subprocess.run(
                    [
                        compiler,
                        "-std=c++17",
                        "-Wall",
                        "-Wextra",
                        f"-I{ARDUINO_STUB_INCLUDE_DIR}",
                        f"-I{INCLUDE_DIR}",
                        *(str(source) for source in sources),
                        "-o",
                        str(executable),
                    ],
                    cwd=temporary,
                    text=True,
                    capture_output=True,
                    check=False,
                )

            self.assertEqual(
                compile_result.returncode,
                0,
                msg=compile_result.stdout + compile_result.stderr,
            )
            run_result = subprocess.run(
                [str(executable)],
                cwd=temporary,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(
                run_result.returncode,
                0,
                msg=run_result.stdout + run_result.stderr,
            )

    def test_protocol_parser_packet_and_safety_state_machine(self):
        with tempfile.TemporaryDirectory(prefix="arm-hat-protocol-") as temporary:
            temporary_path = pathlib.Path(temporary)
            executable = temporary_path / (
                "arm_hat_protocol_test.exe" if os.name == "nt" else "arm_hat_protocol_test"
            )

            if os.name == "nt":
                vcvars = pathlib.Path(
                    r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
                    r"\VC\Auxiliary\Build\vcvars64.bat"
                )
                if not vcvars.exists():
                    self.skipTest("MSVC Build Tools 2022 are not installed")
                command = (
                    f'call "{vcvars}" >nul && '
                    f'cl /nologo /std:c++17 /EHsc /I"{INCLUDE_DIR}" '
                    f'"{TEST_SOURCE}" "{LIBRARY_SOURCE}" /Fe:"{executable}"'
                )
                compile_result = subprocess.run(
                    command,
                    cwd=temporary,
                    text=True,
                    capture_output=True,
                    check=False,
                    shell=True,
                )
            else:
                compiler = shutil.which("c++") or shutil.which("g++")
                if compiler is None:
                    self.skipTest("No C++17 compiler is installed")
                compile_result = subprocess.run(
                    [
                        compiler,
                        "-std=c++17",
                        "-Wall",
                        "-Wextra",
                        f"-I{INCLUDE_DIR}",
                        str(TEST_SOURCE),
                        str(LIBRARY_SOURCE),
                        "-o",
                        str(executable),
                    ],
                    cwd=temporary,
                    text=True,
                    capture_output=True,
                    check=False,
                )

            self.assertEqual(
                compile_result.returncode,
                0,
                msg=compile_result.stdout + compile_result.stderr,
            )
            run_result = subprocess.run(
                [str(executable)],
                cwd=temporary,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(
                run_result.returncode,
                0,
                msg=run_result.stdout + run_result.stderr,
            )

    def test_canonical_payloads_are_accepted_by_the_pi_transport(self):
        from robot_gateway.serial_arm_controller import SerialArmController

        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        SerialArmController._validate_hello(fixture["hello"])
        SerialArmController._validated_servo(fixture["status"]["servos"][0])
        SerialArmController._validated_servo(fixture["capture"])

        class FixtureSerial:
            def __init__(self, fixture):
                self.is_open = True
                self.timeout = 0.5
                self.fixture = fixture
                self.writes = []
                self.last_operation = None

            def _status(self):
                status = deepcopy(self.fixture["status"])
                operation = self.last_operation
                if operation == "TORQUE_LEASE":
                    status["state"] = "lease_active"
                    status["torqueState"] = "on"
                    status["lease"] = {"servoId": 3, "remainingMs": 1000}
                    status["torqueOffPending"] = True
                    status["torqueOffPendingCount"] = 1
                    status["torqueOffRetryServoId"] = 3
                    status["servos"][0]["torqueState"] = "on"
                elif operation == "PREPARE_NUDGE":
                    prepared = self.fixture["prepareNudge"]
                    status["state"] = "nudge_prepared"
                    status["proposal"] = {
                        "proposalId": prepared["proposalId"],
                        "servoId": prepared["servoId"],
                        "targetRawPosition": prepared["targetRawPosition"],
                        "remainingMs": prepared["expiresInMs"],
                    }
                elif operation == "EXECUTE_NUDGE":
                    status["servos"][0]["rawPosition"] = self.fixture[
                        "executeNudge"
                    ]["rawPosition"]
                elif operation == "SET_POSITION_MODE":
                    status["servos"][0]["operatingMode"] = 0
                    status["servos"][0]["errors"] = []
                elif operation == "STOP":
                    status["state"] = "stopped_latched"
                    status["motionState"] = "stopped"
                    status["stopped"] = True
                    status["safetyFault"] = True
                    status["operatorInspectionRequired"] = True
                    status["safetyStopReason"] = "EXPLICIT_STOP"
                return status

            def _payload_for(self, operation):
                mapping = {
                    "SCAN": "scan",
                    "ASSIGN_ID": "assignId",
                    "SET_POSITION_MODE": "setPositionMode",
                    "CAPTURE": "capture",
                    "TORQUE_LEASE": "torqueLease",
                    "TORQUE_OFF": (
                        "torqueOffAll"
                        if self.writes[-1].decode("ascii").strip().endswith("ALL")
                        else "torqueOffOne"
                    ),
                    "PREPARE_NUDGE": "prepareNudge",
                    "EXECUTE_NUDGE": "executeNudge",
                    "STOP": "stop",
                    "RESET": "reset",
                }
                return deepcopy(self.fixture[mapping[operation]])

            def write(self, payload):
                self.writes.append(payload)
                return len(payload)

            def flush(self):
                return None

            def readline(self, _size=-1):
                tokens = self.writes[-1].decode("ascii").strip().split()
                sequence = int(tokens[1])
                operation = tokens[2]
                if operation == "STATUS":
                    payload = self._status()
                else:
                    payload = self._payload_for(operation)
                    self.last_operation = operation
                compact = json.dumps(payload, separators=(",", ":"))
                return f"A1 {sequence} OK {compact}\n".encode("ascii")

            def close(self):
                self.is_open = False

        serial = FixtureSerial(fixture)
        controller = SerialArmController("fixture", serial_factory=lambda **_: serial)
        controller._serial = serial
        controller._connection = "online"
        controller._identity = SerialArmController._validate_hello(fixture["hello"])

        controller.status()
        controller.scan(0, 20)
        controller.assign_id(1, 3)
        controller.set_position_mode(3)
        controller.capture(3)
        controller.torque_lease(3, 1000)
        controller.torque_off(3)
        controller.torque_off()
        prepared = controller.prepare_nudge(3, 23, 80, 10)
        controller.execute_nudge(prepared["proposalId"])
        controller.stop()
        controller.reset(inspected=True)
        operations = [
            write.decode("ascii").strip().split()[2] for write in serial.writes
        ]
        self.assertIn("SCAN", operations)
        self.assertIn("EXECUTE_NUDGE", operations)
        self.assertEqual(operations[-2:], ["RESET", "STATUS"])

    def test_firmware_handlers_emit_the_canonical_field_names(self):
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        def section(start, end):
            return source[source.index(start) : source.index(end, source.index(start))]

        sections = {
            "hello": section("void ArmHatRuntime::handleHello", "void ArmHatRuntime::handleHeartbeat"),
            "status": section("void ArmHatRuntime::sendStatusJson", "void ArmHatRuntime::handleHello"),
            "scan": section("void ArmHatRuntime::handleScan", "void ArmHatRuntime::handleAssignId"),
            "assignId": section("void ArmHatRuntime::handleAssignId", "void ArmHatRuntime::handleSetPositionMode"),
            "setPositionMode": section("void ArmHatRuntime::handleSetPositionMode", "void ArmHatRuntime::handleCapture"),
            "capture": section("void ArmHatRuntime::handleCapture", "void ArmHatRuntime::handleTorqueLease"),
            "torqueLease": section("void ArmHatRuntime::handleTorqueLease", "void ArmHatRuntime::handleTorqueOff"),
            "torqueOffOne": section("void ArmHatRuntime::handleTorqueOff", "void ArmHatRuntime::handlePrepareNudge"),
            "torqueOffAll": section("void ArmHatRuntime::handleTorqueOff", "void ArmHatRuntime::handlePrepareNudge"),
            "prepareNudge": section("void ArmHatRuntime::handlePrepareNudge", "void ArmHatRuntime::handleExecuteNudge"),
            "executeNudge": section("void ArmHatRuntime::handleExecuteNudge", "void ArmHatRuntime::handleStop"),
            "stop": section("void ArmHatRuntime::handleStop", "void ArmHatRuntime::handleReset"),
            "reset": section("void ArmHatRuntime::handleReset", "void ArmHatRuntime::tick"),
        }
        servo_fields = section("void ArmHatRuntime::sendServoFields", "void ArmHatRuntime::sendServoJson")
        for payload_name, handler_source in sections.items():
            keys = fixture[payload_name].keys()
            if payload_name == "capture":
                handler_source += servo_fields
            for key in keys:
                self.assertIn(
                    f'\\"{key}\\"',
                    handler_source,
                    msg=f"{payload_name} firmware response lost canonical field {key}",
                )

    def test_every_torque_enable_is_prearmed_and_cancellation_never_forgets_it(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        def section(start, end):
            start_at = source.index(start)
            return source[start_at : source.index(end, start_at)]

        torque_lease = section(
            "void ArmHatRuntime::handleTorqueLease",
            "void ArmHatRuntime::handleTorqueOff",
        )
        execute = section(
            "void ArmHatRuntime::handleExecuteNudge",
            "void ArmHatRuntime::handleStop",
        )
        for handler in (torque_lease, execute):
            arm_at = handler.index("armTorqueOffObligation")
            enable_at = handler.index("setTorque(", arm_at)
            self.assertLess(arm_at, enable_at)
            self.assertIn("torqueOffAndConfirm", handler[enable_at:])

        self.assertNotIn("clearTorqueLease", source)
        for handler_name, next_handler in (
            ("handleScan", "handleAssignId"),
            ("handleAssignId", "handleSetPositionMode"),
            ("handleSetPositionMode", "handleCapture"),
            ("handleCapture", "handleTorqueLease"),
            ("handleTorqueOff", "handlePrepareNudge"),
            ("handlePrepareNudge", "handleExecuteNudge"),
            ("handleStop", "handleReset"),
        ):
            handler = section(
                f"void ArmHatRuntime::{handler_name}",
                f"void ArmHatRuntime::{next_handler}",
            )
            self.assertIn(
                "resolvePendingTorqueOff",
                handler,
                msg=f"{handler_name} can cancel authority without retaining off proof",
            )

    def test_discovery_capture_and_explicit_off_queue_addressed_proof(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        def section(start, end):
            start_at = source.index(start)
            return source[start_at : source.index(end, start_at)]

        scan = section(
            "void ArmHatRuntime::handleScan",
            "void ArmHatRuntime::handleAssignId",
        )
        capture = section(
            "void ArmHatRuntime::handleCapture",
            "void ArmHatRuntime::handleTorqueLease",
        )
        torque_off = section(
            "void ArmHatRuntime::handleTorqueOff",
            "void ArmHatRuntime::handlePrepareNudge",
        )
        self.assertIn("queueTorqueOffObligation", scan)
        self.assertIn("reconcileCompletedScan", scan)
        self.assertIn("queueTorqueOffObligation(id)", capture)
        self.assertIn("torqueOffAndConfirm(id)", capture)
        self.assertIn("torqueOffAndConfirm(id)", torque_off)

        confirm = section(
            "bool ArmHatRuntime::confirmTorqueOff",
            "bool ArmHatRuntime::confirmAllTrackedTorqueOff",
        )
        self.assertIn("findServo(id)", confirm)
        self.assertNotIn("rememberServo(id)", confirm)

    def test_family_declaration_registers_configured_inventory_without_bus_mutation(self):
        """Configured IDs must become visible without a torque-off scan.

        The Pi replays FAMILY for every configured joint after connect. Before
        this regression was fixed, FAMILY only updated already-known IDs, while
        a fresh HAT inventory was empty; STATUS therefore had nothing to poll
        and Base stayed absent until the operator pressed SCAN.
        """

        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        start = source.index("void ArmHatRuntime::handleFamily")
        end = source.index("void ArmHatRuntime::handleConfig", start)
        handler = source[start:end]

        self.assertIn("rememberServo(id)", handler)
        self.assertIn('sendError(sequence, "CAPACITY")', handler)
        self.assertNotIn("refreshServo", handler)
        self.assertNotIn("bus_.ping", handler)
        self.assertNotIn("broadcastTorqueOff", handler)
        self.assertNotIn("torqueOffAndConfirm", handler)

    def test_active_lease_is_supervised_without_status_polling(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        def section(start, end):
            start_at = source.index(start)
            return source[start_at : source.index(end, start_at)]

        supervisor = section(
            "bool ArmHatRuntime::superviseActiveLease",
            "void ArmHatRuntime::invalidateProposal",
        )
        tick = section(
            "void ArmHatRuntime::tick",
            "}  // namespace armhat",
        )
        self.assertIn("LEASE_SUPERVISION_INTERVAL_MS", supervisor)
        self.assertIn("refreshServo(id)", supervisor)
        self.assertIn("validActiveLeaseTelemetry", supervisor)
        self.assertIn("queueTorqueOffObligation(id)", supervisor)
        self.assertIn("safety_.revokeAuthority()", supervisor)
        self.assertIn("resolvePendingTorqueOff()", supervisor)
        self.assertIn("broadcastTorqueOff()", supervisor)
        self.assertIn("superviseActiveLease(nowMs)", tick)

    def test_unexpected_torque_and_reassigned_id_are_fail_closed(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")

        def section(start, end):
            start_at = source.index(start)
            return source[start_at : source.index(end, start_at)]

        refresh = section(
            "bool ArmHatRuntime::refreshServo",
            "bool ArmHatRuntime::confirmTorqueOff",
        )
        assign = section(
            "void ArmHatRuntime::handleAssignId",
            "void ArmHatRuntime::handleSetPositionMode",
        )
        self.assertIn("torqueAuthorizedFor", refresh)
        self.assertIn("queueTorqueOffObligation(id)", refresh)
        self.assertIn("safety_.revokeAuthority()", refresh)
        self.assertIn("torqueOffAndConfirm(id)", refresh)
        self.assertIn("torqueOffAndConfirm(newId)", assign)
        self.assertIn("refreshServo(newId)", assign)
        self.assertIn("newIdFresh", assign)

    def test_only_explicit_stop_can_latch_automatic_faults(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        protocol_header = (INCLUDE_DIR / "ArmHatProtocol.h").read_text(
            encoding="utf-8"
        )
        protocol_source = LIBRARY_SOURCE.read_text(encoding="utf-8")

        stop_start = source.index("void ArmHatRuntime::handleStop")
        stop_end = source.index("void ArmHatRuntime::handleReset", stop_start)
        stop_handler = source[stop_start:stop_end]
        outside_stop = source[:stop_start] + source[stop_end:]

        self.assertEqual(source.count("safety_.latchStop();"), 1)
        self.assertIn("safety_.latchStop();", stop_handler)
        self.assertNotIn("safety_.latchStop();", outside_stop)
        self.assertNotIn("safety_.stop(", source)
        self.assertIn("void revokeAuthority();", protocol_header)
        self.assertIn("void latchStop();", protocol_header)
        self.assertNotIn("void stop(", protocol_header)
        self.assertIn("SafetyStateMachine::revokeAuthority()", protocol_source)
        self.assertIn("SafetyStateMachine::latchStop()", protocol_source)

    def test_position_mode_restore_is_fixed_guarded_and_verified(self):
        source = RUNTIME_SOURCE.read_text(encoding="utf-8")
        start = source.index("void ArmHatRuntime::handleSetPositionMode")
        handler = source[start : source.index("void ArmHatRuntime::handleCapture", start)]

        self.assertIn('"SINGLE_SERVO"', handler)
        self.assertIn('"ST3215"', handler)
        self.assertIn("candidate <= 253", handler)
        self.assertIn("count != 1", handler)
        self.assertIn("torqueOffAndConfirm(id)", handler)
        self.assertIn("writeLock(id, false)", handler)
        self.assertIn("writePositionMode(id)", handler)
        self.assertIn("writeLock(id, true)", handler)
        self.assertIn("readPositionLimits", handler)
        self.assertIn("previousMode > ST3215_OPERATING_MODE_MAX", handler)
        self.assertIn("verifiedMinimum == previousMinimum", handler)
        self.assertIn("verifiedMaximum == previousMaximum", handler)
        self.assertNotIn("writeRegister", handler)

        lock_reader_start = source.index("BusResult St3215Bus::readLock")
        lock_reader = source[
            lock_reader_start : source.index(
                "BusResult St3215Bus::readOperatingMode", lock_reader_start
            )
        ]
        self.assertIn("value > 1", lock_reader)
        self.assertIn("return BusResult::CORRUPT", lock_reader)

    def test_worst_case_eight_servo_status_fits_the_wire_buffer(self):
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        status = dict(fixture["status"])
        template = dict(status["servos"][0])
        template.update(
            {
                "rawPosition": 4095,
                "speed": -32767,
                "load": -1000,
                "voltageVolts": 25.5,
                "temperatureC": 150,
                "moving": False,
                "currentRaw": 65535,
                "torqueState": "unknown",
                "packetAgeMs": 86400000,
                "errors": [
                    "servo_fault_0xFF",
                    "status_error_0xFF",
                    "bus_offline",
                    "torque_unknown",
                    "mode_not_position",
                ],
                "statusError": 255,
                "online": False,
                "fresh": False,
                "operatingMode": 255,
            }
        )
        status["servoCount"] = 8
        status["state"] = "nudge_prepared"
        status["motionState"] = "blocked"
        status["torqueState"] = "unknown"
        status["servosState"] = "faulted"
        status["hostAgeMs"] = 4294967295
        status["lease"] = {"servoId": 253, "remainingMs": 2000}
        status["torqueOffPending"] = True
        status["torqueOffPendingCount"] = 254
        status["torqueOffRetryServoId"] = 253
        status["proposal"] = {
            "proposalId": "pTEST_ONLY_MAX_SEQUENCE",
            "servoId": 253,
            "targetRawPosition": 4095,
            "remainingMs": 15000,
        }
        status["servos"] = [{**template, "id": identifier} for identifier in range(8)]
        compact = json.dumps(status, separators=(",", ":"))
        frame = f"A1 4294967295 OK {compact}\n".encode("ascii")
        self.assertLessEqual(len(frame), 4096)
        # Mirrors the portable static allocation proof in ArmHatProtocol.h.
        self.assertLessEqual(len(frame), 768 + 8 * 384 + 32)


if __name__ == "__main__":
    unittest.main()
