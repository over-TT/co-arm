from __future__ import annotations

import asyncio
from copy import deepcopy
from pathlib import Path

from fastapi import FastAPI
from fastapi.testclient import TestClient
import httpx

from robot_gateway.arm_controller import ReplayArmController, UnavailableArmController
import robot_gateway.physical_arm_api as physical_arm_api
import robot_gateway.simple_arm_api as simple_arm_api_module
from robot_gateway.physical_arm_api import create_physical_arm_router
from robot_gateway.runtime import create_app
from robot_gateway.simple_arm_api import ArmService, JointStore, create_simple_arm_router


TOKEN = "physical-arm-token-" + ("p" * 40)


def _headers() -> dict[str, str]:
    return {"Authorization": f"Bearer {TOKEN}"}


def _client(tmp_path: Path, controller: object) -> TestClient:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    return TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=tmp_path / "state",
        )
    )


def test_same_event_loop_physical_request_is_rejected_while_first_awaits_body(
    monkeypatch,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 2, "rawPosition": 2048, "torqueState": "off"}]
    )
    original_strict_body = physical_arm_api._strict_body

    async def scenario() -> tuple[httpx.Response, httpx.Response, list[dict[str, object]]]:
        first_entered = asyncio.Event()
        release_first = asyncio.Event()
        parse_calls = 0

        async def pause_first_body(http_request, contract):
            nonlocal parse_calls
            parse_calls += 1
            if parse_calls == 1:
                first_entered.set()
                await release_first.wait()
            return await original_strict_body(http_request, contract)

        monkeypatch.setattr(physical_arm_api, "_strict_body", pause_first_body)
        app = FastAPI()
        app.include_router(create_physical_arm_router(controller))
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://gateway.test"
        ) as client:
            first_task = asyncio.create_task(
                client.post(
                    "/api/robot/physical/arm/servos/torque-off",
                    json={"servoId": 2},
                )
            )
            await asyncio.wait_for(first_entered.wait(), timeout=1.0)
            second = await client.post(
                "/api/robot/physical/arm/servos/torque-off",
                json={"servoId": 2},
            )
            commands_before_release = deepcopy(controller.commands)
            release_first.set()
            first = await asyncio.wait_for(first_task, timeout=1.0)
        return first, second, commands_before_release

    first, second, commands_before_release = asyncio.run(scenario())

    assert second.status_code == 409, second.text
    assert second.json()["detail"] == "Another physical arm operation is in progress."
    assert commands_before_release == []
    assert first.status_code == 200, first.text
    assert [command["operation"] for command in controller.commands] == ["TORQUE_OFF"]


def test_same_loop_simple_release_cannot_be_overtaken_by_older_physical_body(
    tmp_path: Path,
    monkeypatch,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 2, "rawPosition": 2048, "torqueState": "off"}]
    )
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    original_strict_body = physical_arm_api._strict_body

    async def scenario() -> tuple[httpx.Response, httpx.Response, httpx.Response]:
        first_entered = asyncio.Event()
        release_first = asyncio.Event()
        parse_calls = 0

        async def pause_first_physical_body(http_request, contract):
            nonlocal parse_calls
            parse_calls += 1
            if parse_calls == 1:
                first_entered.set()
                await release_first.wait()
            return await original_strict_body(http_request, contract)

        monkeypatch.setattr(
            physical_arm_api, "_strict_body", pause_first_physical_body
        )
        app = FastAPI()
        app.include_router(
            create_physical_arm_router(
                controller,
                operation_lock=service.operation_lock,
                admission_lock=service.operation_admission_lock,
                mutation_context=service.commissioning_mutation,
                clear_mutation_context=service.commissioning_clear_mutation,
                stop_callback=service.stop_for_commissioning,
                reset_callback=service.clear_stop_for_commissioning,
            )
        )
        app.include_router(create_simple_arm_router(controller, arm_service=service))
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://gateway.test"
        ) as client:
            physical_task = asyncio.create_task(
                client.post(
                    "/api/robot/physical/arm/servos/hold-set",
                    json={
                        "servoIds": [2],
                        "leaseMs": 1500,
                        "acknowledgedPhysicalPowerCut": True,
                        "confirmedServoModel": "ST3215",
                    },
                )
            )
            await asyncio.wait_for(first_entered.wait(), timeout=1.0)
            blocked_release = await client.post(
                "/api/robot/arm/torque", json={"hold": []}
            )
            assert controller.commands == []
            release_first.set()
            physical = await asyncio.wait_for(physical_task, timeout=1.0)
            completed_release = await client.post(
                "/api/robot/arm/torque", json={"hold": []}
            )
        return blocked_release, physical, completed_release

    try:
        blocked_release, physical, completed_release = asyncio.run(scenario())
    finally:
        service.close()

    assert blocked_release.status_code == 409, blocked_release.text
    assert physical.status_code == 200, physical.text
    assert completed_release.status_code == 200, completed_release.text
    release_index = max(
        index
        for index, command in enumerate(controller.commands)
        if command["operation"] == "HOLD_SET" and command.get("servoIds") == []
    )
    assert not any(
        command["operation"] == "HOLD_SET" and command.get("servoIds")
        for command in controller.commands[release_index + 1 :]
    )
    assert controller.transport_state()["torqueState"] == "off"


def test_physical_status_names_each_connection_layer_when_hat_is_not_configured(
    tmp_path: Path,
) -> None:
    with _client(tmp_path, UnavailableArmController(configured=False)) as client:
        response = client.get("/api/robot/physical/arm/status", headers=_headers())

    assert response.status_code == 200
    body = response.json()
    assert body["mode"] == "physical"
    assert body["motionState"] == "blocked"
    assert body["torqueState"] == "unknown"
    assert body["connections"]["pi"]["state"] == "online"
    assert body["connections"]["controller"]["state"] == "not_configured"
    assert body["connections"]["bus"]["state"] == "unknown"
    assert body["connections"]["servos"]["state"] == "unknown"
    assert "Configure the HAT controller serial port" in body["nextAction"]


def test_physical_action_is_fail_closed_when_hat_is_offline(tmp_path: Path) -> None:
    with _client(tmp_path, UnavailableArmController(configured=True)) as client:
        response = client.post(
            "/api/robot/physical/arm/bus/scan",
            headers=_headers(),
            json={"minId": 0, "maxId": 20},
        )

    assert response.status_code == 503
    assert response.json() == {
        "detail": "The HAT controller is not responding. Motion is blocked."
    }


def test_connected_status_returns_full_normalized_live_servo_dto(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {
                "id": 1,
                "rawPosition": 2112,
                "speed": -3,
                "load": 7,
                "voltageVolts": 12.1,
                "temperatureC": 29,
                "moving": False,
                "currentMilliamps": 18,
                "operatingMode": 0,
                "torqueState": "off",
                "packetAgeMs": 12,
                "errors": [],
            }
        ]
    )
    with _client(tmp_path, controller) as client:
        response = client.get("/api/robot/physical/arm/status", headers=_headers())

    assert response.status_code == 200
    body = response.json()
    assert body["motionState"] == "ready_disarmed"
    assert body["connections"]["pi"]["detail"]
    assert body["connections"]["controller"]["controllerId"] == "replay-hat-a"
    assert body["connections"]["bus"]["baud"] == 1_000_000
    assert body["connections"]["servos"]["state"] == "online"
    assert body["servos"] == [
        {
            "id": 1,
            "rawPosition": 2112,
            "speed": -3,
            "load": 7,
            "voltageVolts": 12.1,
            "temperatureC": 29,
            "moving": False,
            "currentMilliamps": 18,
            "operatingMode": 0,
            "torqueState": "off",
            "packetAgeMs": 12,
            "errors": [],
        }
    ]
    assert body["stop"]["state"] == "not_latched"
    assert body["hardwareEstop"] == "not_detected"


def test_configured_offline_status_has_recovery_details_and_unknown_stop(
    tmp_path: Path,
) -> None:
    with _client(tmp_path, UnavailableArmController(configured=True)) as client:
        response = client.get("/api/robot/physical/arm/status", headers=_headers())

    body = response.json()
    assert body["motionState"] == "blocked"
    assert body["torqueState"] == "unknown"
    assert body["connections"]["controller"]["state"] == "offline"
    assert body["connections"]["controller"]["detail"]
    assert body["connections"]["bus"]["detail"]
    assert body["connections"]["servos"]["detail"]
    assert body["stop"]["state"] == "unknown"
    assert body["stop"]["detail"]
    assert all(blocker["recovery"] for blocker in body["blockers"])


def test_zero_reply_scan_keeps_bus_unverified_and_reports_no_servos(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(servos=[])
    with _client(tmp_path, controller) as client:
        scanned = client.post(
            "/api/robot/physical/arm/bus/scan",
            headers=_headers(),
            json={"minId": 0, "maxId": 253},
        )
        status_response = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )

    assert scanned.status_code == 200
    assert scanned.json()["foundIds"] == []
    body = status_response.json()
    assert body["connections"]["bus"]["state"] == "unknown"
    assert body["connections"]["servos"]["state"] == "none"


def test_partial_scan_does_not_replace_global_responding_ids(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 100, "torqueState": "off", "operatingMode": 0}]
    )
    with _client(tmp_path, controller) as client:
        scanned = client.post(
            "/api/robot/physical/arm/bus/scan",
            headers=_headers(),
            json={"minId": 0, "maxId": 20},
        )
        status_response = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )

    assert scanned.status_code == 200
    assert scanned.json()["foundIds"] == []
    body = status_response.json()
    assert body["connections"]["servos"]["respondingIds"] == [100]
    assert body["connections"]["servos"]["state"] == "online"
    assert body["connections"]["bus"]["lastScan"] == {
        "minId": 0,
        "maxId": 20,
        "foundIds": [],
    }


def test_scan_and_capture_return_controller_observations_not_browser_values(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {
                "id": 1,
                "rawPosition": 2112,
                "speed": 0,
                "load": 7,
                "voltageVolts": 12.1,
                "temperatureC": 29,
                "moving": False,
                "currentMilliamps": 18,
                "torqueState": "off",
                "packetAgeMs": 12,
                "errors": [],
            }
        ]
    )
    with _client(tmp_path, controller) as client:
        scanned = client.post(
            "/api/robot/physical/arm/bus/scan",
            headers=_headers(),
            json={"minId": 0, "maxId": 20},
        )
        captured = client.post(
            "/api/robot/physical/arm/servos/capture",
            headers=_headers(),
            json={"servoId": 1},
        )

    assert scanned.status_code == 200
    assert scanned.json()["foundIds"] == [1]
    assert scanned.json()["completeRange"] == {"minId": 0, "maxId": 20}
    assert captured.status_code == 200
    assert captured.json()["rawPosition"] == 2112
    assert captured.json()["sampleCount"] >= 5
    assert captured.json()["evidenceId"].startswith("cap_")


def test_id_assignment_requires_one_servo_acknowledgement(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(servos=[])
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/physical/arm/servos/assign-id",
            headers=_headers(),
            json={
                "oldId": 1,
                "newId": 2,
                "acknowledgedSingleServo": False,
                "confirmedServoModel": "ST3215",
            },
        )

    assert response.status_code == 422
    assert controller.commands == []


def test_position_mode_restore_is_explicit_and_unblocks_capture(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {
                "id": 1,
                "rawPosition": 4084,
                "torqueState": "off",
                "operatingMode": 1,
                "errors": ["mode_not_position"],
            }
        ]
    )
    with _client(tmp_path, controller) as client:
        rejected = client.post(
            "/api/robot/physical/arm/servos/set-position-mode",
            headers=_headers(),
            json={
                "servoId": 1,
                "acknowledgedSingleServo": False,
                "confirmedServoModel": "ST3215",
            },
        )
        restored = client.post(
            "/api/robot/physical/arm/servos/set-position-mode",
            headers=_headers(),
            json={
                "servoId": 1,
                "acknowledgedSingleServo": True,
                "confirmedServoModel": "ST3215",
            },
        )
        status_response = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )
        captured = client.post(
            "/api/robot/physical/arm/servos/capture",
            headers=_headers(),
            json={"servoId": 1},
        )

    assert rejected.status_code == 422
    assert restored.status_code == 200
    assert restored.json() == {
        "servoId": 1,
        "previousOperatingMode": 1,
        "operatingMode": 0,
        "verified": True,
        "locked": True,
        "torqueState": "off",
        "minimumPosition": 0,
        "maximumPosition": 4095,
    }
    assert controller.commands[0] == {"operation": "SET_POSITION_MODE", "servoId": 1}
    servo = status_response.json()["servos"][0]
    assert servo["operatingMode"] == 0
    assert servo["torqueState"] == "off"
    assert servo["errors"] == []
    assert captured.status_code == 200
    assert captured.json()["rawPosition"] == 4084


def test_nudge_is_two_phase_bounded_and_one_use(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 2, "rawPosition": 2000, "torqueState": "off"}]
    )
    with _client(tmp_path, controller) as client:
        zero = client.post(
            "/api/robot/physical/arm/servos/capture",
            headers=_headers(),
            json={"servoId": 2},
        ).json()
        prepared = client.post(
            "/api/robot/physical/arm/tests/prepare-nudge",
            headers=_headers(),
            json={
                "servoId": 2,
                "deltaTicks": 23,
                "speed": 80,
                "acceleration": 8,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
                "zeroEvidenceId": zero["evidenceId"],
            },
        )
        assert prepared.status_code == 200
        proposal = prepared.json()
        assert proposal["expiresInMs"] == 10_000
        assert "startRawPosition" not in proposal
        assert "targetRawPosition" not in proposal

        missing_execute_ack = client.post(
            "/api/robot/physical/arm/tests/execute-nudge",
            headers=_headers(),
            json={"proposalId": proposal["proposalId"]},
        )
        executed = client.post(
            "/api/robot/physical/arm/tests/execute-nudge",
            headers=_headers(),
            json={
                "proposalId": proposal["proposalId"],
                "acknowledgedPhysicalPowerCut": True,
            },
        )
        replayed = client.post(
            "/api/robot/physical/arm/tests/execute-nudge",
            headers=_headers(),
            json={
                "proposalId": proposal["proposalId"],
                "acknowledgedPhysicalPowerCut": True,
            },
        )

    assert missing_execute_ack.status_code == 422
    assert executed.status_code == 200
    assert executed.json()["torqueState"] == "off"
    assert replayed.status_code == 409


def test_hold_set_selectively_holds_and_releases_servos(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {"id": identifier, "rawPosition": 2048, "torqueState": "off"}
            for identifier in (1, 2, 3)
        ]
    )
    request_body = {
        "servoIds": [2, 3],
        "leaseMs": 1_500,
        "acknowledgedPhysicalPowerCut": True,
        "confirmedServoModel": "ST3215",
    }
    with _client(tmp_path, controller) as client:
        held = client.post(
            "/api/robot/physical/arm/servos/hold-set",
            headers=_headers(),
            json=request_body,
        )
        released_two = client.post(
            "/api/robot/physical/arm/servos/hold-set",
            headers=_headers(),
            json={**request_body, "servoIds": [3]},
        )
        released_all = client.post(
            "/api/robot/physical/arm/servos/hold-set",
            headers=_headers(),
            json={**request_body, "servoIds": []},
        )

    assert held.status_code == 200
    assert held.json()["servoIds"] == [2, 3]
    assert held.json()["confirmed"] is True
    assert released_two.status_code == 200
    assert released_two.json()["servoIds"] == [3]
    assert released_all.status_code == 200
    assert released_all.json()["servoIds"] == []
    assert [command for command in controller.commands if command["operation"] == "HOLD_SET"] == [
        {"operation": "HOLD_SET", "servoIds": [2, 3], "leaseMs": 1_500},
        {"operation": "HOLD_SET", "servoIds": [3], "leaseMs": 1_500},
        {"operation": "HOLD_SET", "servoIds": [], "leaseMs": 1_500},
    ]
    assert controller.transport_state()["torqueState"] == "off"


def test_hold_set_schema_rejects_invalid_sets_before_controller_io(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 1, "rawPosition": 2048, "torqueState": "off"}]
    )
    common = {
        "leaseMs": 1_500,
        "acknowledgedPhysicalPowerCut": True,
        "confirmedServoModel": "ST3215",
    }
    with _client(tmp_path, controller) as client:
        responses = [
            client.post(
                "/api/robot/physical/arm/servos/hold-set",
                headers=_headers(),
                json={**common, **invalid},
            )
            for invalid in (
                {"servoIds": [1, 1]},
                {"servoIds": [1, 2, 3, 4]},
                {"servoIds": [1], "acknowledgedPhysicalPowerCut": 1},
            )
        ]

    assert all(response.status_code == 422 for response in responses)
    assert controller.commands == []


def test_prepare_nudge_is_withheld_and_torque_is_forced_off_when_not_near_zero(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 2, "rawPosition": 2000, "torqueState": "off"}]
    )
    with _client(tmp_path, controller) as client:
        zero = client.post(
            "/api/robot/physical/arm/servos/capture",
            headers=_headers(),
            json={"servoId": 2},
        ).json()
        controller._servos[2]["rawPosition"] = 2050
        prepared = client.post(
            "/api/robot/physical/arm/tests/prepare-nudge",
            headers=_headers(),
            json={
                "servoId": 2,
                "deltaTicks": 23,
                "speed": 80,
                "acceleration": 8,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
                "zeroEvidenceId": zero["evidenceId"],
            },
        )

    assert prepared.status_code == 409
    assert "within 8 raw ticks" in prepared.json()["detail"]
    assert [command["operation"] for command in controller.commands[-2:]] == [
        "PREPARE_NUDGE",
        "TORQUE_OFF",
    ]
    assert controller.transport_state()["torqueState"] == "off"


def test_write_and_motion_routes_require_exact_model_and_power_cut_assertions(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": 1, "rawPosition": 2048, "torqueState": "off"}]
    )
    with _client(tmp_path, controller) as client:
        missing_model = client.post(
            "/api/robot/physical/arm/servos/assign-id",
            headers=_headers(),
            json={"oldId": 1, "newId": 2, "acknowledgedSingleServo": True},
        )
        wrong_model = client.post(
            "/api/robot/physical/arm/servos/assign-id",
            headers=_headers(),
            json={
                "oldId": 1,
                "newId": 2,
                "acknowledgedSingleServo": True,
                "confirmedServoModel": "ST3235",
            },
        )
        missing_cut = client.post(
            "/api/robot/physical/arm/servos/torque-lease",
            headers=_headers(),
            json={"servoId": 1, "leaseMs": 500, "confirmedServoModel": "ST3215"},
        )
        non_boolean_cut = client.post(
            "/api/robot/physical/arm/tests/prepare-nudge",
            headers=_headers(),
            json={
                "servoId": 1,
                "deltaTicks": 12,
                "speed": 40,
                "acceleration": 4,
                "acknowledgedPhysicalPowerCut": 1,
                "confirmedServoModel": "ST3215",
                "zeroEvidenceId": "cap_fake_zero",
            },
        )

    assert {response.status_code for response in (missing_model, wrong_model, missing_cut, non_boolean_cut)} == {422}
    assert controller.commands == []


def test_explicit_reconnect_recovers_a_faulted_replay_controller(
    tmp_path: Path, monkeypatch,
) -> None:
    # This route test owns the reconnect. Keep the background boot-order retry
    # from racing the intervening faulted-state assertion under a loaded suite.
    monkeypatch.setattr(simple_arm_api_module, "SERVICE_TICK_SECONDS", 60.0)
    controller = ReplayArmController.connected(servos=[])
    controller.fail_next("SCAN", "serial timeout")
    with _client(tmp_path, controller) as client:
        failed = client.post(
            "/api/robot/physical/arm/bus/scan",
            headers=_headers(),
            json={"minId": 0, "maxId": 20},
        )
        faulted = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )
        reconnected = client.post(
            "/api/robot/physical/arm/controller/reconnect", headers=_headers()
        )

    assert failed.status_code == 503
    assert faulted.json()["connections"]["controller"]["state"] == "faulted"
    assert reconnected.status_code == 200
    assert reconnected.json()["connections"]["controller"]["state"] == "online"


def test_status_blocks_stale_faulted_or_non_position_servo_telemetry(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {
                "id": 1,
                "torqueState": "off",
                "operatingMode": 3,
                "packetAgeMs": 400,
                "errors": ["overload"],
            }
        ]
    )
    with _client(tmp_path, controller) as client:
        response = client.get("/api/robot/physical/arm/status", headers=_headers())

    body = response.json()
    assert body["motionState"] == "blocked"
    assert body["servos"][0]["operatingMode"] == 3
    codes = {blocker["code"] for blocker in body["blockers"]}
    assert {"SERVO_ERRORS", "SERVO_TELEMETRY_STALE", "OPERATING_MODE_NOT_POSITION"} <= codes


def test_status_and_motion_block_when_operating_mode_is_unknown(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {
                "id": 1,
                "torqueState": "off",
                "operatingMode": None,
                "packetAgeMs": 4,
                "errors": [],
            }
        ]
    )
    with _client(tmp_path, controller) as client:
        status_response = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )
        prepared = client.post(
            "/api/robot/physical/arm/tests/prepare-nudge",
            headers=_headers(),
            json={
                "servoId": 1,
                "deltaTicks": 12,
                "speed": 40,
                "acceleration": 4,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
                "zeroEvidenceId": "cap_fake_zero",
            },
        )

    body = status_response.json()
    assert body["motionState"] == "blocked"
    assert any(
        blocker["code"] == "OPERATING_MODE_UNKNOWN"
        for blocker in body["blockers"]
    )
    assert prepared.status_code == 409
    assert not any(
        command["operation"] == "PREPARE_NUDGE" for command in controller.commands
    )


def test_stop_delivery_failure_never_claims_that_hardware_stopped(tmp_path: Path) -> None:
    controller = ReplayArmController.connected(servos=[])
    controller.fail_next("STOP", "serial timeout")
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/physical/arm/stop",
            headers=_headers(),
        )

    assert response.status_code == 503
    assert response.json()["detail"] == (
        "STOP delivery is unknown. Cut servo power before touching the arm."
    )


def test_status_preserves_accepted_stop_latch_with_unknown_electrical_state(
    tmp_path: Path,
) -> None:
    class LatchedUnknownStopController(ReplayArmController):
        def stop(self) -> dict[str, object]:
            with self._lock:
                self._before("STOP")
                self._proposals.clear()
                self._leases.clear()
                self._stopped = True
                for servo in self._servos.values():
                    servo["torqueState"] = "unknown"
                return {
                    "stopped": True,
                    "torqueState": "unknown",
                    "confirmed": False,
                    "torqueOffBroadcastSent": True,
                }

        def transport_state(self) -> dict[str, object]:
            state = super().transport_state()
            if self._stopped and state.get("connection") == "online":
                state["motionState"] = "stopped"
                state["torqueState"] = "unknown"
                for servo in state.get("servosTelemetry", []):
                    if isinstance(servo, dict):
                        servo["torqueState"] = "unknown"
            return state

        status = transport_state

    controller = LatchedUnknownStopController(
        connected=True,
        servos=[{"id": 1, "torqueState": "off", "operatingMode": 0}],
    )
    with _client(tmp_path, controller) as client:
        stopped = client.post("/api/robot/physical/arm/stop", headers=_headers())
        status_response = client.get(
            "/api/robot/physical/arm/status", headers=_headers()
        )

    assert stopped.status_code == 200
    assert stopped.json()["confirmed"] is False
    body = status_response.json()
    assert body["motionState"] == "stopped"
    assert body["torqueState"] == "unknown"
    assert body["stop"]["state"] == "unknown"
    assert body["stop"]["detail"] == (
        "Controller STOP latch is accepted, but actuator torque removal is unproven."
    )
    assert "RESET INSPECTED" in body["nextAction"]


def test_physical_stop_and_inspected_reset_converge_simple_arm_stop_state(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[
            {"id": 1, "rawPosition": 2048, "torqueState": "off", "operatingMode": 0},
            {"id": 2, "rawPosition": 2048, "torqueState": "off", "operatingMode": 0},
            {"id": 3, "rawPosition": 2048, "torqueState": "off", "operatingMode": 0},
        ]
    )
    with _client(tmp_path, controller) as client:
        stopped = client.post("/api/robot/physical/arm/stop", headers=_headers())
        simple_stopped = client.get("/api/robot/arm/state", headers=_headers())
        reset = client.post(
            "/api/robot/physical/arm/reset",
            headers=_headers(),
            json={"acknowledgedPhysicalInspection": True},
        )
        simple_cleared = client.get("/api/robot/arm/state", headers=_headers())

    assert stopped.status_code == 200
    assert simple_stopped.json()["stopped"] is True
    assert reset.status_code == 200, reset.text
    assert simple_cleared.json()["stopped"] is False
    reset_commands = [
        command for command in controller.commands if command["operation"] == "RESET"
    ]
    assert reset_commands[-1]["inspected"] is True


def _profile_payload() -> dict[str, object]:
    joints: dict[str, object] = {}
    for index, logical_id in enumerate(("joint_1", "joint_2", "joint_3"), start=1):
        joints[logical_id] = {
            "logicalId": logical_id,
            "name": ("Base", "Shoulder", "Elbow")[index - 1],
            "servoId": index,
            "rawZero": 2048,
            "direction": 1,
            "negativeLimitTicks": -900,
            "positiveLimitTicks": 900,
            "limitMarginTicks": 40,
            "maxVelocityDegreesPerSecond": 25.0,
            "maxAccelerationDegreesPerSecond2": 50.0,
            "evidenceIds": [f"obs_setup_{index}"],
        }
    return {
        "expectedControllerBootId": "replay_boot_1",
        "confirmedServoModel": "ST3215",
        "joints": joints,
        "geometry": {
            "baseHeightMm": 80.0,
            "upperArmMm": 150.0,
            "forearmMm": 140.0,
            "toolOffsetMm": 20.0,
        },
        "acknowledgedRemainDisarmed": True,
    }


def _record_profile_evidence(
    client: TestClient,
    controller: ReplayArmController,
    payload: dict[str, object],
) -> None:
    joints = payload["joints"]
    assert isinstance(joints, dict)
    for joint in joints.values():
        assert isinstance(joint, dict)
        servo_id = int(joint["servoId"])
        raw_zero = int(joint["rawZero"])
        direction = int(joint["direction"])
        margin = int(joint["limitMarginTicks"])
        observed_negative = int(joint["negativeLimitTicks"]) - margin
        observed_positive = int(joint["positiveLimitTicks"]) + margin
        evidence_ids: list[str] = []
        for raw_position in (
            raw_zero,
            raw_zero + direction * observed_negative,
            raw_zero + direction * observed_positive,
        ):
            controller._servos[servo_id]["rawPosition"] = raw_position
            captured = client.post(
                "/api/robot/physical/arm/servos/capture",
                headers=_headers(),
                json={"servoId": servo_id},
            )
            assert captured.status_code == 200, captured.text
            evidence_ids.append(captured.json()["evidenceId"])
        controller._servos[servo_id]["rawPosition"] = raw_zero
        delta = 23 * direction
        prepared = client.post(
            "/api/robot/physical/arm/tests/prepare-nudge",
            headers=_headers(),
            json={
                "servoId": servo_id,
                "deltaTicks": delta,
                "speed": 80,
                "acceleration": 8,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
                "zeroEvidenceId": evidence_ids[0],
            },
        )
        assert prepared.status_code == 200, prepared.text
        executed = client.post(
            "/api/robot/physical/arm/tests/execute-nudge",
            headers=_headers(),
            json={
                "proposalId": prepared.json()["proposalId"],
                "acknowledgedPhysicalPowerCut": True,
            },
        )
        assert executed.status_code == 200, executed.text
        evidence_ids.append(executed.json()["evidenceId"])
        joint["evidenceIds"] = evidence_ids


def test_profile_commit_uses_fresh_off_telemetry_and_persists_atomically(
    tmp_path: Path,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    state_dir = tmp_path / "state"
    controller = ReplayArmController.connected(
        servos=[
            {"id": identifier, "rawPosition": 2048, "torqueState": "off", "packetAgeMs": 4}
            for identifier in (1, 2, 3)
        ]
    )
    with TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=state_dir,
        )
    ) as client:
        profile_payload = _profile_payload()
        profile_payload["joints"]["joint_1"]["motorTurnsPerJointTurn"] = 4
        _record_profile_evidence(client, controller, profile_payload)
        commands_before_commit = len(controller.commands)
        before = client.get(
            "/api/robot/physical/arm/calibration/profile", headers=_headers()
        )
        committed = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
                json=profile_payload,
        )

    assert before.json() == {
        "mode": "physical",
        "committed": False,
        "motionState": "ready_disarmed",
        "profileRevision": 0,
        "profileHash": None,
        "profile": None,
    }
    assert committed.status_code == 200
    body = committed.json()
    assert body["committed"] is True
    assert body["motionState"] == "ready_disarmed"
    assert body["profileRevision"] == 1
    assert body["profileHash"].startswith("sha256:")
    assert body["profile"]["schemaVersion"] == 1
    assert body["profile"]["servoModel"] == "ST3215"
    assert body["profile"]["servoLabelsConfirmed"] is True
    assert body["profile"]["expectedServoIds"] == [1, 2, 3]
    assert body["profile"]["joints"]["joint_1"]["motorTurnsPerJointTurn"] == 4.0
    assert body["profile"]["joints"]["joint_2"]["motorTurnsPerJointTurn"] == 1.0
    assert body["profile"]["controller"] == {
        "controllerId": "replay-hat-a",
        "firmwareVersion": "replay-1.0.0",
        "protocolVersion": 1,
    }
    assert (state_dir / "physical-arm-profile.json").is_file()
    assert not any(
        command["operation"] in {"TORQUE_LEASE", "PREPARE_NUDGE", "EXECUTE_NUDGE"}
        for command in controller.commands[commands_before_commit:]
    )
    reloaded_controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with TestClient(
        create_app(
            token_file=token_file,
            arm_controller=reloaded_controller,
            arm_state_dir=state_dir,
        )
    ) as client:
        reloaded = client.get(
            "/api/robot/physical/arm/calibration/profile", headers=_headers()
        )
    assert reloaded.status_code == 200
    assert reloaded.json()["profileRevision"] == 1
    assert reloaded.json()["profileHash"] == body["profileHash"]
    assert reloaded.json()["profile"] == body["profile"]


def test_profile_commit_rejects_any_unconfirmed_off_torque_before_writing(
    tmp_path: Path,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    state_dir = tmp_path / "state"
    controller = ReplayArmController.connected(
        servos=[
            {"id": 1, "torqueState": "on"},
            {"id": 2, "torqueState": "off"},
            {"id": 3, "torqueState": "off"},
        ]
    )
    with TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=state_dir,
        )
    ) as client:
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=_profile_payload(),
        )

    assert response.status_code == 409
    assert "torque" in response.json()["detail"].lower()
    assert not (state_dir / "physical-arm-profile.json").exists()
    assert controller.commands == []


def test_profile_rejects_a_servo_range_that_wraps_past_raw_encoder_bounds(
    tmp_path: Path,
) -> None:
    payload = _profile_payload()
    payload["joints"]["joint_1"]["rawZero"] = 4070
    payload["joints"]["joint_1"]["positiveLimitTicks"] = 100

    controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=payload,
        )

    assert response.status_code == 422
    assert controller.commands == []


def test_profile_rejects_invalid_motor_turns_ratio_before_controller_io(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with _client(tmp_path, controller) as client:
        responses = []
        for ratio in (0, 65, True):
            payload = _profile_payload()
            payload["joints"]["joint_1"]["motorTurnsPerJointTurn"] = ratio
            responses.append(
                client.post(
                    "/api/robot/physical/arm/calibration/profile",
                    headers=_headers(),
                    json=payload,
                )
            )

    assert all(response.status_code == 422 for response in responses)
    assert controller.commands == []


def test_profile_rejects_browser_forged_evidence_before_bus_commands(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=_profile_payload(),
        )

    assert response.status_code == 409
    assert "forged, stale, or other-servo" in response.json()["detail"]
    assert controller.commands == []


def test_profile_rejects_other_servo_and_prior_boot_evidence(
    tmp_path: Path,
) -> None:
    controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with _client(tmp_path, controller) as client:
        other_servo_payload = _profile_payload()
        _record_profile_evidence(client, controller, other_servo_payload)
        joints = other_servo_payload["joints"]
        assert isinstance(joints, dict)
        joint_1 = joints["joint_1"]
        joint_2 = joints["joint_2"]
        assert isinstance(joint_1, dict) and isinstance(joint_2, dict)
        joint_2["evidenceIds"][0] = joint_1["evidenceIds"][0]
        before_other_servo = len(controller.commands)
        other_servo = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=other_servo_payload,
        )
        after_other_servo = len(controller.commands)

        stale_payload = _profile_payload()
        _record_profile_evidence(client, controller, stale_payload)
        controller.simulate_reboot()
        stale_payload["expectedControllerBootId"] = "replay_boot_2"
        before_stale = len(controller.commands)
        stale = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=stale_payload,
        )

    assert other_servo.status_code == 409
    assert "other-servo" in other_servo.json()["detail"]
    assert after_other_servo == before_other_servo
    assert stale.status_code == 409
    assert "stale" in stale.json()["detail"]
    assert len(controller.commands) == before_stale


def _record_multi_turn_evidence(
    client: TestClient,
    controller: ReplayArmController,
    joint: dict[str, object],
    *,
    arm_odometer: bool = True,
) -> None:
    """Capture zero and both endpoints for a joint whose travel wraps the encoder."""

    servo_id = int(joint["servoId"])
    raw_zero = int(joint["rawZero"])
    direction = int(joint["direction"])
    margin = int(joint["limitMarginTicks"])
    observed_negative = int(joint["negativeLimitTicks"]) - margin
    observed_positive = int(joint["positiveLimitTicks"]) + margin

    controller._servos[servo_id]["rawPosition"] = raw_zero
    if arm_odometer:
        armed = client.post(
            "/api/robot/physical/arm/servos/odometer/zero",
            headers=_headers(),
            json={"servoId": servo_id},
        )
        assert armed.status_code == 200, armed.text
        assert armed.json()["multiTurnPosition"] == raw_zero

    evidence_ids: list[str] = []
    for offset in (0, direction * observed_negative, direction * observed_positive):
        target = raw_zero + offset
        revolutions, raw = divmod(target, 4096)
        controller._servos[servo_id]["rawPosition"] = raw
        if arm_odometer:
            controller._odometers[servo_id] = {"revolutions": revolutions, "lastRaw": raw}
        captured = client.post(
            "/api/robot/physical/arm/servos/capture",
            headers=_headers(),
            json={"servoId": servo_id},
        )
        assert captured.status_code == 200, captured.text
        if arm_odometer:
            assert captured.json()["multiTurnPosition"] == target
        evidence_ids.append(captured.json()["evidenceId"])

    controller._servos[servo_id]["rawPosition"] = raw_zero
    if arm_odometer:
        controller._odometers[servo_id] = {"revolutions": 0, "lastRaw": raw_zero}
    prepared = client.post(
        "/api/robot/physical/arm/tests/prepare-nudge",
        headers=_headers(),
        json={
            "servoId": servo_id,
            "deltaTicks": 23 * direction,
            "speed": 80,
            "acceleration": 8,
            "acknowledgedPhysicalPowerCut": True,
            "confirmedServoModel": "ST3215",
            "zeroEvidenceId": evidence_ids[0],
        },
    )
    assert prepared.status_code == 200, prepared.text
    executed = client.post(
        "/api/robot/physical/arm/tests/execute-nudge",
        headers=_headers(),
        json={
            "proposalId": prepared.json()["proposalId"],
            "acknowledgedPhysicalPowerCut": True,
        },
    )
    assert executed.status_code == 200, executed.text
    evidence_ids.append(executed.json()["evidenceId"])
    joint["evidenceIds"] = evidence_ids


def _multi_turn_base_payload() -> dict[str, object]:
    payload = _profile_payload()
    base = payload["joints"]["joint_1"]
    assert isinstance(base, dict)
    base["multiTurn"] = True
    base["motorTurnsPerJointTurn"] = 8.0
    base["negativeLimitTicks"] = -8000
    base["positiveLimitTicks"] = 8000
    return payload


def test_multi_turn_joint_accepts_a_range_spanning_several_motor_turns(
    tmp_path: Path,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    state_dir = tmp_path / "state"
    controller = ReplayArmController.connected(
        servos=[
            {"id": identifier, "rawPosition": 2048, "torqueState": "off", "packetAgeMs": 4}
            for identifier in (1, 2, 3)
        ]
    )
    payload = _multi_turn_base_payload()
    with TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=state_dir,
        )
    ) as client:
        _record_multi_turn_evidence(client, controller, payload["joints"]["joint_1"])
        _record_profile_evidence(
            client,
            controller,
            {"joints": {key: payload["joints"][key] for key in ("joint_2", "joint_3")}},
        )
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=payload,
        )

    assert response.status_code == 200, response.text
    assert (state_dir / "physical-arm-profile.json").exists()


def test_multi_turn_range_is_still_rejected_without_the_multi_turn_flag(
    tmp_path: Path,
) -> None:
    payload = _multi_turn_base_payload()
    payload["joints"]["joint_1"]["multiTurn"] = False

    controller = ReplayArmController.connected(
        servos=[{"id": identifier, "torqueState": "off"} for identifier in (1, 2, 3)]
    )
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=payload,
        )

    assert response.status_code == 422
    assert controller.commands == []


def test_multi_turn_joint_is_rejected_when_its_zero_lacks_wrap_counted_evidence(
    tmp_path: Path,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    controller = ReplayArmController.connected(
        servos=[
            {"id": identifier, "rawPosition": 2048, "torqueState": "off", "packetAgeMs": 4}
            for identifier in (1, 2, 3)
        ]
    )
    payload = _multi_turn_base_payload()
    # Narrow enough to survive schema validation without the odometer armed, so
    # the failure proves the evidence check fired rather than the bounds check.
    payload["joints"]["joint_1"]["negativeLimitTicks"] = -900
    payload["joints"]["joint_1"]["positiveLimitTicks"] = 900
    with _client(tmp_path, controller) as client:
        _record_multi_turn_evidence(
            client, controller, payload["joints"]["joint_1"], arm_odometer=False
        )
        _record_profile_evidence(
            client,
            controller,
            {"joints": {key: payload["joints"][key] for key in ("joint_2", "joint_3")}},
        )
        response = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=payload,
        )

    assert response.status_code == 409
    assert "wrap-counted" in response.json()["detail"]


def test_declared_limits_commit_without_endpoint_captures_but_still_need_zero_and_nudge(
    tmp_path: Path,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    state_dir = tmp_path / "state"
    controller = ReplayArmController.connected(
        servos=[
            {"id": identifier, "rawPosition": 2048, "torqueState": "off", "packetAgeMs": 4}
            for identifier in (1, 2, 3)
        ]
    )
    payload = _profile_payload()
    for joint in payload["joints"].values():
        joint["declaredLimits"] = True

    with TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=state_dir,
        )
    ) as client:
        # Only zero and the direction nudge are captured; neither endpoint is.
        for joint in payload["joints"].values():
            servo_id = int(joint["servoId"])
            raw_zero = int(joint["rawZero"])
            controller._servos[servo_id]["rawPosition"] = raw_zero
            captured = client.post(
                "/api/robot/physical/arm/servos/capture",
                headers=_headers(),
                json={"servoId": servo_id},
            )
            assert captured.status_code == 200, captured.text
            zero_evidence = captured.json()["evidenceId"]
            prepared = client.post(
                "/api/robot/physical/arm/tests/prepare-nudge",
                headers=_headers(),
                json={
                    "servoId": servo_id,
                    "deltaTicks": 23 * int(joint["direction"]),
                    "speed": 80,
                    "acceleration": 8,
                    "acknowledgedPhysicalPowerCut": True,
                    "confirmedServoModel": "ST3215",
                    "zeroEvidenceId": zero_evidence,
                },
            )
            assert prepared.status_code == 200, prepared.text
            executed = client.post(
                "/api/robot/physical/arm/tests/execute-nudge",
                headers=_headers(),
                json={
                    "proposalId": prepared.json()["proposalId"],
                    "acknowledgedPhysicalPowerCut": True,
                },
            )
            assert executed.status_code == 200, executed.text
            joint["evidenceIds"] = [zero_evidence, executed.json()["evidenceId"]]

        accepted = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=payload,
        )

        # The same profile without the flag is still refused for lacking endpoints.
        proven = deepcopy(payload)
        for joint in proven["joints"].values():
            joint["declaredLimits"] = False
        refused = client.post(
            "/api/robot/physical/arm/calibration/profile",
            headers=_headers(),
            json=proven,
        )

    assert accepted.status_code == 200, accepted.text
    assert accepted.json()["profile"]["joints"]["joint_1"]["declaredLimits"] is True
    assert refused.status_code == 409
    assert "endpoint evidence" in refused.json()["detail"]
