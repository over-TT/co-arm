from __future__ import annotations

import threading
import time
from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from robot_gateway.arm_controller import ReplayArmController
from robot_gateway.runtime import create_app
import robot_gateway.simple_arm_api as arm_api
from robot_gateway.simple_arm_api import ArmService, JointStore, SequencePreviewRequest
from robot_gateway.tests.test_arm_execute_and_capture import (
    BaseTruthLostAfterDispatchController,
    FakeProvider,
    TransientlyStaleOdometerController,
    _frame,
)


TOKEN = "arm-sequence-token-" + ("q" * 40)
AUTH = {"Authorization": f"Bearer {TOKEN}"}


def _servo(servo_id: int, raw: int) -> dict[str, object]:
    return {
        "id": servo_id,
        "rawPosition": raw,
        "speed": 0,
        "load": 0,
        "voltageVolts": 12.0,
        "temperatureC": 30.0,
        "moving": False,
        "operatingMode": 0,
        "torqueState": "off",
        "packetAgeMs": 1,
        "errors": [],
    }


def _controller(
    kind: type[ReplayArmController] = ReplayArmController,
) -> ReplayArmController:
    controller = kind(
        connected=True,
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ],
    )
    controller.set_multi_turn(1, True)
    controller.odometer_zero(1)
    controller.commands.clear()
    return controller


def _client(
    tmp_path: Path,
    controller: ReplayArmController,
    *,
    provider: FakeProvider | None = None,
) -> TestClient:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    return TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=tmp_path / "state",
            camera_provider=provider,
        )
    )


def _calibrate(client: TestClient) -> None:
    for name, zero, maximum in (
        ("joint_1", 2048, 4095),
        ("joint_2", 2048, 4095),
        ("joint_3", 2048, 4095),
        ("joint_4", 512, 1023),
    ):
        response = client.post(
            f"/api/robot/arm/joints/{name}/calibrate",
            headers=AUTH,
            json={
                "rawZero": zero,
                "rawMin": 0,
                "rawMax": maximum,
                "ratio": 1,
            },
        )
        assert response.status_code == 200, response.text


def _preview(
    client: TestClient,
    waypoints: list[dict[str, object]] | None = None,
) -> dict[str, object]:
    response = client.post(
        "/api/robot/arm/sequences/preview",
        headers=AUTH,
        json={
            "waypoints": waypoints
            or [
                {
                    "label": "left contact",
                    "targets": {"joint_2": 4.0, "joint_3": 4.0},
                },
                {
                    "label": "right sweep",
                    "targets": {"joint_2": 8.0, "joint_3": 8.0},
                },
            ]
        },
    )
    assert response.status_code == 200, response.text
    return response.json()


def _execute_payload(preview: dict[str, object]) -> dict[str, object]:
    return {
        "sequenceId": preview["sequenceId"],
        "sequenceDigest": preview["sequenceDigest"],
        "arrivalTimeoutMs": 2500,
    }


def test_sequence_preview_is_strict_motionless_and_carries_each_resolved_pose(
    tmp_path: Path,
) -> None:
    controller = _controller()
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        controller.commands.clear()
        preview = _preview(
            client,
            [
                {"label": "lift-1", "targets": {"joint_2": 5.0}},
                {"label": "aim 2", "targets": {"joint_4": 10.0}},
            ],
        )

        one = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={"waypoints": [{"targets": {"joint_2": 1.0}}]},
        )
        nine = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={
                "waypoints": [
                    {"targets": {"joint_2": float(index)}} for index in range(9)
                ]
            },
        )
        unsafe_label = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={
                "waypoints": [
                    {"label": "bad/slash", "targets": {"joint_2": 1.0}},
                    {"targets": {"joint_2": 2.0}},
                ]
            },
        )
        extra = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={
                "waypoints": [
                    {"targets": {"joint_2": 1.0}, "unknown": True},
                    {"targets": {"joint_2": 2.0}},
                ]
            },
        )

    assert str(preview["sequenceId"]).startswith("armseq_")
    assert str(preview["sequenceDigest"]).startswith("sha256:")
    assert preview["expiresInMs"] == 120_000
    assert preview["waypointCount"] == 2
    assert isinstance(preview["previewDurationMs"], int)
    assert preview["lowestClearanceMm"] == min(
        row["lowestClearanceMm"] for row in preview["waypoints"]
    )
    rows = preview["waypoints"]
    assert isinstance(rows, list)
    assert rows[0]["startPose"] == preview["measuredPose"]
    assert rows[1]["startPose"] == rows[0]["resolvedPose"]
    assert rows[1]["resolvedPose"]["joint_2"] == rows[0]["resolvedPose"]["joint_2"]
    assert rows[1]["resolvedPose"]["joint_4"] != rows[1]["startPose"]["joint_4"]
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )
    assert one.status_code == 422
    assert nine.status_code == 422
    assert unsafe_label.status_code == 422
    assert extra.status_code == 422


def test_sequence_digest_binds_every_reviewed_segment_field(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = _controller()
    store = JointStore(tmp_path / "digest-state")
    for name, zero, maximum in (
        ("joint_1", 2048, 4095),
        ("joint_2", 2048, 4095),
        ("joint_3", 2048, 4095),
        ("joint_4", 512, 1023),
    ):
        joint = store.get(name)
        joint.rawZero = zero
        joint.rawMin = 0
        joint.rawMax = maximum
    captured_payloads: list[dict[str, object]] = []
    original = arm_api._sha256

    def record(value: object) -> str:
        if isinstance(value, dict) and "sequenceId" in value:
            captured_payloads.append(value)
        return original(value)

    monkeypatch.setattr(arm_api, "_sha256", record)
    service = ArmService(controller, store)
    try:
        preview = service.preview_sequence(
            SequencePreviewRequest.model_validate(
                {
                    "waypoints": [
                        {"targets": {"joint_2": 3.0}},
                        {"targets": {"joint_4": 6.0}},
                    ]
                }
            )
        )
    finally:
        service.close()

    digest_waypoints = captured_payloads[-1]["waypoints"]
    assert isinstance(digest_waypoints, list)
    reviewed_waypoints = preview["waypoints"]
    assert isinstance(reviewed_waypoints, list)
    for bound, reviewed in zip(digest_waypoints, reviewed_waypoints, strict=True):
        assert bound["warnings"] == reviewed["warnings"]
        assert bound["lowestClearanceMm"] == reviewed["lowestClearanceMm"]
        assert bound["scene"] == reviewed["scene"]
        assert bound["ik"] == reviewed.get("ik")


def test_sequence_preview_rejects_an_unsafe_later_segment_without_motion(
    tmp_path: Path,
) -> None:
    controller = _controller()
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        controller._servos[2]["rawPosition"] = 1024  # Shoulder -90 degrees.
        controller._servos[3]["rawPosition"] = 1024  # Elbow -90 degrees.
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={
                "waypoints": [
                    {"label": "safe start", "targets": {"joint_3": -85.0}},
                    {"label": "direct retreat", "targets": {"joint_3": 85.0}},
                ]
            },
        )

    assert response.status_code == 409
    detail = response.json()["detail"]
    assert "Transition into waypoint 2 (direct retreat)" in detail
    assert "independently timed" in detail
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_sequence_execute_proves_every_arrival_and_replays_without_motion(
    tmp_path: Path,
) -> None:
    controller = _controller()
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)
        controller.commands.clear()

        first = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )
        repeated = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )
        wrong_mode = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )

    assert first.status_code == 200, first.text
    result = first.json()
    assert result["outcome"] == "completed"
    assert result["executed"] is True
    assert result["waypointCount"] == 2
    assert result["completedWaypointCount"] == 2
    assert result["failedWaypointIndex"] is None
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert len(result["waypointResults"]) == 2
    assert all(row["outcome"] == "arrived" for row in result["waypointResults"])
    assert all(row["arrival"]["proved"] is True for row in result["waypointResults"])
    assert all(
        isinstance(row["dispatchMs"], (int, float)) and row["dispatchMs"] > 0
        for row in result["waypointResults"]
    )
    assert all(isinstance(row["arrivalMs"], int) for row in result["waypointResults"])
    assert all(isinstance(row["durationMs"], int) for row in result["waypointResults"])
    assert isinstance(result["durationMs"], int)
    assert result["finalMeasuredPose"]["joint_2"] == result["resolvedPose"]["joint_2"]
    assert repeated.status_code == 200
    assert repeated.json() == result
    assert wrong_mode.status_code == 409
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE_SET"]
    ) == 2


def test_sequence_base_waypoint_waits_for_fresh_odometer_after_stale_sample(
    tmp_path: Path,
) -> None:
    controller = _controller(TransientlyStaleOdometerController)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(
            client,
            [
                {"label": "base one", "targets": {"joint_1": 4.0}},
                {"label": "base two", "targets": {"joint_1": 8.0}},
            ],
        )
        response = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json={**_execute_payload(preview), "arrivalTimeoutMs": 5000},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert controller.stale_post_dispatch_samples == 1
    assert result["outcome"] == "completed"
    assert result["completedWaypointCount"] == 2
    assert all(row["outcome"] == "arrived" for row in result["waypointResults"])
    assert all(row["arrival"]["proved"] is True for row in result["waypointResults"])


class NeverSettlesFirstWaypoint(ReplayArmController):
    def move_set(self, moves):
        result = super().move_set(moves)
        for servo_id, _goal, _speed, _acceleration in moves:
            self._servos[servo_id]["moving"] = True
        return result


class NeverSettlesSecondWaypoint(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.move_set_calls = 0

    def move_set(self, moves):
        result = super().move_set(moves)
        self.move_set_calls += 1
        if self.move_set_calls == 2:
            for servo_id, _goal, _speed, _acceleration in moves:
                self._servos[servo_id]["moving"] = True
        return result


class PauseBeforeSecondWaypoint(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.first_move_sent = False
        self.statuses_after_move = 0
        self.before_second = threading.Event()
        self.release_second = threading.Event()

    def move(self, servo_id: int, goal: int, speed: int, acceleration: int):
        result = super().move(servo_id, goal, speed, acceleration)
        self.first_move_sent = True
        return result

    def status(self) -> dict[str, object]:
        if self.first_move_sent:
            self.statuses_after_move += 1
            if self.statuses_after_move == 4:
                self.before_second.set()
                assert self.release_second.wait(3.0)
        return super().status()


def test_sequence_timeout_aborts_without_stop_or_later_dispatch(
    tmp_path: Path,
) -> None:
    controller = _controller(NeverSettlesFirstWaypoint)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = {
            **_execute_payload(preview),
            "arrivalTimeoutMs": 500,
        }
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "arrival_timeout"
    assert result["completedWaypointCount"] == 0
    assert result["failedWaypointIndex"] == 0
    assert result["waypointResults"][0]["outcome"] == "arrival_timeout"
    assert state["stopped"] is False
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE_SET"]
    ) == 1
    assert not any(
        command["operation"] == "STOP" for command in controller.commands
    )


def test_typed_arrival_fault_aborts_without_emergency_stop(
    tmp_path: Path,
) -> None:
    controller = _controller(BaseTruthLostAfterDispatchController)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(
            client,
            [
                {"label": "base one", "targets": {"joint_1": 4.0}},
                {"label": "base two", "targets": {"joint_1": 8.0}},
            ],
        )
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json={**_execute_payload(preview), "arrivalTimeoutMs": 2500},
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "arrival_fault"
    assert result["completedWaypointCount"] == 0
    assert result["failedWaypointIndex"] == 0
    assert result["waypointResults"][0]["arrival"]["reason"] == (
        "joint_1_position_untrusted"
    )
    assert state["stopped"] is False
    assert len(
        [
            command
            for command in controller.commands
            if command["operation"] == "MOVE_MULTI_TURN"
        ]
    ) == 1
    assert not any(
        command["operation"] == "STOP" for command in controller.commands
    )


def test_final_capture_timeout_aborts_without_stop_or_picture(
    tmp_path: Path,
) -> None:
    controller = _controller(NeverSettlesSecondWaypoint)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-after-timeout\xff\xd9"))
    with _client(tmp_path, controller, provider=provider) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = {**_execute_payload(preview), "arrivalTimeoutMs": 2500}
        controller.commands.clear()
        controller.move_set_calls = 0
        response = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "arrival_timeout"
    assert result["completedWaypointCount"] == 1
    assert result["failedWaypointIndex"] == 1
    assert result["waypointResults"][1]["outcome"] == "arrival_timeout"
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0
    assert state["stopped"] is False
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE_SET"]
    ) == 2
    assert not any(
        command["operation"] == "STOP" for command in controller.commands
    )


def test_operator_stop_preempts_sequence_and_prevents_later_dispatch(
    tmp_path: Path,
) -> None:
    controller = _controller(NeverSettlesFirstWaypoint)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = {**_execute_payload(preview), "arrivalTimeoutMs": 5000}
        controller.commands.clear()
        finished = threading.Event()
        received: dict[str, object] = {}

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        thread = threading.Thread(target=execute)
        thread.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not any(
            command["operation"] == "MOVE_SET" for command in controller.commands
        ):
            time.sleep(0.01)
        stopped = client.post("/api/robot/arm/stop", headers=AUTH)
        assert stopped.status_code == 200, stopped.text
        assert finished.wait(2.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "stopped"
    assert result["completedWaypointCount"] == 0
    assert result["failedWaypointIndex"] == 0
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE_SET"]
    ) == 1


def test_post_dispatch_arrival_exception_stops_and_replays_without_motion(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = _controller()
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)

        def fail_after_dispatch(*_args: object, **_kwargs: object) -> object:
            raise RuntimeError("injected arrival read failure")

        monkeypatch.setattr(ArmService, "_arrival_sample", fail_after_dispatch)
        controller.commands.clear()
        first = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )
        commands_after_first = list(controller.commands)
        repeated = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert first.status_code == 200, first.text
    result = first.json()
    assert result["outcome"] == "arrival_fault"
    assert result["waypointCount"] == 2
    assert result["completedWaypointCount"] == 0
    assert result["failedWaypointIndex"] == 0
    failed = result["waypointResults"][0]
    assert failed["dispatched"] is True
    assert failed["arrival"]["reason"] == "post_dispatch_arrival_check_failed"
    assert state["stopped"] is True
    assert len(
        [command for command in commands_after_first if command["operation"] == "MOVE_SET"]
    ) == 1
    assert len(
        [command for command in commands_after_first if command["operation"] == "STOP"]
    ) == 1
    assert repeated.status_code == 200
    assert repeated.json() == result
    assert controller.commands == commands_after_first


def test_operator_stop_between_arrived_waypoints_is_typed_without_second_stop(
    tmp_path: Path,
) -> None:
    controller = _controller(PauseBeforeSecondWaypoint)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(
            client,
            [
                {"targets": {"joint_2": 4.0}},
                {"targets": {"joint_2": 8.0}},
            ],
        )
        controller.commands.clear()
        received: dict[str, object] = {}
        finished = threading.Event()

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/sequences/execute",
                headers=AUTH,
                json={**_execute_payload(preview), "arrivalTimeoutMs": 5000},
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        thread = threading.Thread(target=execute)
        thread.start()
        assert controller.before_second.wait(3.0)
        stopped = client.post("/api/robot/arm/stop", headers=AUTH)
        assert stopped.status_code == 200, stopped.text
        controller.release_second.set()
        assert finished.wait(3.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "stopped"
    assert result["completedWaypointCount"] == 1
    assert result["failedWaypointIndex"] == 1
    assert result["waypointResults"][1]["dispatched"] is False
    assert result["waypointResults"][1]["arrival"]["reason"] == (
        "stop_latched_before_waypoint"
    )
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE"]
    ) == 1
    assert len(
        [command for command in controller.commands if command["operation"] == "STOP"]
    ) == 1


class DriftsBeforeSecondWaypoint(ReplayArmController):
    def move(self, servo_id: int, goal: int, speed: int, acceleration: int):
        result = super().move(servo_id, goal, speed, acceleration)
        if servo_id == 4:
            self._servos[2]["rawPosition"] += 128
        return result


class MissingServoFamilyCapability(ReplayArmController):
    def transport_state(self) -> dict[str, object]:
        state = super().transport_state()
        identity = state.get("identity")
        capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
        if isinstance(capabilities, list):
            identity["capabilities"] = [
                capability
                for capability in capabilities
                if capability != "servo_family"
            ]
        return state


class DriftsInheritedShoulderBeforeSecondWaypoint(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._first_move_sent = False
        self._statuses_after_move = 0

    def move(self, servo_id: int, goal: int, speed: int, acceleration: int):
        result = super().move(servo_id, goal, speed, acceleration)
        if servo_id == 2:
            self._first_move_sent = True
        return result

    def status(self) -> dict[str, object]:
        result = super().status()
        if self._first_move_sent:
            self._statuses_after_move += 1
            if self._statuses_after_move == 4:
                self._servos[2]["rawPosition"] += 128
                result = self.transport_state()
        return result


def test_sequence_revalidates_the_actual_start_before_each_waypoint(
    tmp_path: Path,
) -> None:
    controller = _controller(DriftsBeforeSecondWaypoint)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(
            client,
            [
                {"targets": {"joint_4": 5.0}},
                {"targets": {"joint_2": 5.0, "joint_3": 5.0}},
            ],
        )
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json=_execute_payload(preview),
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "refused"
    assert result["completedWaypointCount"] == 1
    assert result["failedWaypointIndex"] == 1
    assert result["waypointResults"][1]["dispatched"] is False
    assert result["waypointResults"][1]["dispatchMs"] is None
    assert result["waypointResults"][1]["arrival"]["reason"] == "start_pose_changed"
    assert result["waypointResults"][1]["durationMs"] > 0
    assert state["stopped"] is False
    assert len(
        [
            command
            for command in controller.commands
            if command["operation"] in {"MOVE", "MOVE_SET"}
        ]
    ) == 1


def test_sequence_revalidates_an_inherited_shoulder_before_base_camera_move(
    tmp_path: Path,
) -> None:
    controller = _controller(DriftsInheritedShoulderBeforeSecondWaypoint)
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(
            client,
            [
                {"targets": {"joint_2": 5.0}},
                {"targets": {"joint_1": 3.0, "joint_4": 5.0}},
            ],
        )
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json=_execute_payload(preview),
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "refused"
    assert result["completedWaypointCount"] == 1
    assert result["failedWaypointIndex"] == 1
    assert result["waypointResults"][1]["arrival"]["reason"] == "start_pose_changed"
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE"]
    ) == 1
    assert not any(
        command["operation"] in {"MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_capture_preflight_does_not_consume_or_move_then_final_capture_replays(
    tmp_path: Path,
) -> None:
    controller = _controller()
    with _client(tmp_path, controller) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)
        controller.commands.clear()
        unavailable = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        no_motion = [
            command
            for command in controller.commands
            if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        ]
        plain = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )

    assert unavailable.status_code == 503
    assert no_motion == []
    assert plain.status_code == 200, plain.text
    assert plain.json()["outcome"] == "completed"

    captured_controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8sequence-final-frame\xff\xd9"))
    capture_path = tmp_path / "capture"
    capture_path.mkdir()
    with _client(capture_path, captured_controller, provider=provider) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)
        captured_controller.commands.clear()
        captured = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        replay = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        wrong_mode = client.post(
            "/api/robot/arm/sequences/execute", headers=AUTH, json=payload
        )

    assert captured.status_code == 200, captured.text
    result = captured.json()
    assert result["outcome"] == "captured"
    assert result["completedWaypointCount"] == 2
    assert result["captureAttempted"] is True
    assert result["capture"]["frameId"]
    assert result["waypointResults"][-1]["dispatchMs"] > 0
    assert provider.capture_calls == 1
    assert replay.status_code == 200
    assert replay.json() == result
    assert wrong_mode.status_code == 409
    assert len(
        [
            command
            for command in captured_controller.commands
            if command["operation"] == "MOVE_SET"
        ]
    ) == 2


def test_capture_sequence_requires_camera_servo_family_before_consuming_or_moving(
    tmp_path: Path,
) -> None:
    controller = _controller(MissingServoFamilyCapability)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture\xff\xd9"))
    with _client(tmp_path, controller, provider=provider) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        commands_after_refusal = list(controller.commands)
        plain = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json=payload,
        )

    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "SERVO_FAMILY_UNAVAILABLE"
    assert not any(
        command["operation"]
        in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in commands_after_refusal
    )
    assert provider.capture_calls == 0
    assert plain.status_code == 200, plain.text
    assert plain.json()["outcome"] == "completed"


def test_final_capture_unknown_exception_after_dispatch_stops_and_replays(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8must-not-be-used\xff\xd9"))

    def fail_after_final_dispatch(
        self: ArmService,
        request: arm_api.PlanExecuteCaptureRequest,
        _camera: object,
    ) -> dict[str, object]:
        execution = self.execute_plan(
            arm_api.PlanExecuteRequest(
                planId=request.planId,
                planDigest=request.planDigest,
            )
        )
        assert execution["executed"] is True
        raise RuntimeError("injected failure after final dispatch")

    monkeypatch.setattr(ArmService, "execute_and_capture", fail_after_final_dispatch)
    with _client(tmp_path, controller, provider=provider) as client:
        _calibrate(client)
        preview = _preview(client)
        payload = _execute_payload(preview)
        controller.commands.clear()
        first = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        commands_after_first = list(controller.commands)
        repeated = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert first.status_code == 200, first.text
    result = first.json()
    assert result["outcome"] == "arrival_fault"
    assert result["completedWaypointCount"] == 1
    assert result["failedWaypointIndex"] == 1
    failed = result["waypointResults"][1]
    assert failed["dispatched"] is True
    assert failed["arrival"]["reason"] == "post_dispatch_capture_check_failed"
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0
    assert state["stopped"] is True
    assert len(
        [command for command in commands_after_first if command["operation"] == "MOVE_SET"]
    ) == 2
    assert len(
        [command for command in commands_after_first if command["operation"] == "STOP"]
    ) == 1
    assert repeated.status_code == 200
    assert repeated.json() == result
    assert controller.commands == commands_after_first
