from __future__ import annotations

import asyncio
import itertools
import json
import time
import threading
from pathlib import Path

import httpx
import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

import robot_gateway.simple_arm_api as simple_arm_api_module
from robot_gateway.arm_controller import ControllerCommandError, ReplayArmController
from robot_gateway.runtime import create_app
from robot_gateway.simple_arm_api import (
    JointState,
    JointStore,
    LiveFollowStartRequest,
)

TOKEN = "simple-arm-token-" + ("s" * 40)
_LIVE_FOLLOW_START_ATTEMPTS = itertools.count(1)


def _headers() -> dict[str, str]:
    return {"Authorization": f"Bearer {TOKEN}"}


def _servo(servo_id: int, raw: int, torque: str = "off") -> dict[str, object]:
    return {
        "id": servo_id,
        "rawPosition": raw,
        "speed": 0,
        "load": 0,
        "voltageVolts": 12.0,
        "temperatureC": 32.0,
        "moving": False,
        "operatingMode": 0,
        "torqueState": torque,
        "packetAgeMs": 4,
        "errors": [],
    }


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


def _connected() -> ReplayArmController:
    controller = ReplayArmController.connected(
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    return controller


def _planning_controller() -> ReplayArmController:
    controller = ReplayArmController.connected(
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ]
    )
    _establish_base_frame(controller)
    return controller


def _calibrate_for_plans(client: TestClient) -> None:
    for name in ("joint_1", "joint_2", "joint_3"):
        response = client.post(
            f"/api/robot/arm/joints/{name}/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 0, "rawMax": 4095, "ratio": 1},
        )
        assert response.status_code == 200, response.text
    camera = client.post(
        "/api/robot/arm/joints/joint_4/calibrate",
        headers=_headers(),
        json={"rawZero": 512, "rawMin": 0, "rawMax": 1023, "ratio": 1},
    )
    assert camera.status_code == 200, camera.text


def _live_follow_settings(
    *,
    start_attempt_id: str | None = None,
    joint_2_speed: int = 1200,
    joint_2_accel: int = 40,
    joint_2_span: int = 30,
    joint_3_speed: int = 1200,
    joint_3_accel: int = 40,
    joint_3_span: int = 30,
) -> dict[str, object]:
    if start_attempt_id is None:
        start_attempt_id = (
            f"test_start_attempt_{next(_LIVE_FOLLOW_START_ATTEMPTS):08d}"
        )
    return {
        "startAttemptId": start_attempt_id,
        "joint_2": {
            "speed": joint_2_speed,
            "accel": joint_2_accel,
            "maxDeltaDegrees": joint_2_span,
        },
        "joint_3": {
            "speed": joint_3_speed,
            "accel": joint_3_accel,
            "maxDeltaDegrees": joint_3_span,
        },
    }


def _establish_base_frame(controller: ReplayArmController) -> None:
    """Model the explicit physical home that production requires after HAT boot."""

    # Most tests model a Pi-process restart while the HAT's already-established
    # multi-turn frame remains alive. Tests for a HAT reboot clear this explicitly.
    controller.set_multi_turn(1, True)
    controller.odometer_zero(1)
    controller.commands.clear()


# ---- the four numbers ------------------------------------------------------


def test_direction_flips_the_sense_of_every_angle() -> None:
    rising = JointState(1)
    rising.rawZero = 2048
    falling = JointState(2)
    falling.rawZero, falling.direction = 2048, -1

    assert rising.degrees_at(2048 + 512) == falling.degrees_at(2048 - 512)
    assert rising.raw_for(45) == 2560
    assert falling.raw_for(45) == 1536


def test_a_limit_can_be_typed_in_degrees_instead_of_captured(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "ratio": 1},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"minDegrees": -30, "maxDegrees": 30},
        ).json()

    base = body["joints"][0]
    # 30 degrees at 1:1 is 341 ticks either side of the zero.
    assert (base["rawMin"], base["rawMax"]) == (1707, 2389)


def test_ratio_in_the_same_request_applies_before_a_typed_limit(tmp_path: Path) -> None:
    """Otherwise typing a ratio and a limit together silently uses the old scale."""

    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": 2048},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"ratio": 8, "maxDegrees": 5},
        ).json()

    assert body["joints"][0]["rawMax"] == 2048 + round(5 * 4096 * 8 / 360)


def test_a_geared_joint_may_declare_more_travel_than_one_motor_turn() -> None:
    """The operator can see the gearbox; this module cannot.

    A geared joint's declared limits are allowed to name travel past one motor
    turn. Refusing them meant storing a number nobody asked for and then arguing
    about the gearing with the person looking at it.
    """

    joint = JointState(1)
    joint.rawZero, joint.ratio = 2048, 8.0
    joint.rawMin, joint.rawMax = -14336, 18432  # +/- 180 deg at 8:1

    minimum, maximum = joint.degree_bounds()

    assert (round(minimum), round(maximum)) == (-180, 180)
    # The wire is still a single-turn goal, so nothing outside 0..4095 goes out.
    assert joint.goal_for(-180) == 0
    assert joint.goal_for(180) == 4095


def test_reach_reports_the_travel_one_motor_turn_actually_covers() -> None:
    """Declared limits and reachable limits are different questions.

    A joint whose gearing outruns one motor turn accepts its limits and then
    stops short of them, which looks exactly like the drive being broken unless
    the reachable window is stated somewhere.
    """

    joint = JointState(1)
    joint.rawZero, joint.ratio, joint.direction = 610, 8.0, -1
    joint.rawMin, joint.rawMax = joint.raw_for(-180), joint.raw_for(180)

    declared = joint.degree_bounds()
    low, high = joint.reach_bounds()

    assert (round(declared[0]), round(declared[1])) == (-180, 180)
    # 4096 ticks at 8:1 is 45 degrees, and this zero sits 610 ticks from one end.
    assert (round(low, 1), round(high, 1)) == (-38.3, 6.7)


def test_reach_equals_the_limits_when_the_gearing_covers_them() -> None:
    joint = JointState(1)
    joint.rawZero, joint.ratio = 2048, 1.0
    joint.rawMin, joint.rawMax = 1048, 3048

    assert joint.reach_bounds() == joint.degree_bounds()


def test_target_is_clamped_into_the_limits_rather_than_refused() -> None:
    joint = JointState(1)
    joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 1048, 3048, 1.0

    assert joint.goal_for(0) == 2048
    assert joint.goal_for(45) == 2560
    assert joint.goal_for(9_000) == 3048
    assert joint.goal_for(-9_000) == 1048


# ---- the five routes -------------------------------------------------------


def test_calibrate_sets_one_number_at_a_time_and_survives_a_restart(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "ratio": 8},
        )
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawMin": 1048},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawMax": 3048},
        ).json()

    base = body["joints"][0]
    assert (base["rawZero"], base["rawMin"], base["rawMax"], base["ratio"]) == (2048, 1048, 3048, 8)
    assert base["calibrated"] is True

    # A second gateway over the same state directory must see the same numbers.
    with _client(tmp_path, _connected()) as client:
        reloaded = client.get("/api/robot/arm/state", headers=_headers()).json()
    assert reloaded["joints"][0]["rawZero"] == 2048


def test_driving_needs_a_zero_and_nothing_else(tmp_path: Path) -> None:
    controller = _connected()
    with _client(tmp_path, controller) as client:
        refused = client.post(
            "/api/robot/arm/joints/joint_2/target", headers=_headers(), json={"degrees": 10}
        )
        assert refused.status_code == 409

        client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
        )
        client.post(
            "/api/robot/arm/joints/joint_3/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
        )
        # No torque lease, no proposal, no acknowledgement literal: a joint
        # inside its own limits may simply move.
        accepted = client.post(
            "/api/robot/arm/joints/joint_2/target", headers=_headers(), json={"degrees": 45}
        )

    assert accepted.status_code == 200
    assert accepted.json()["moved"] == [
        {"joint": "joint_2", "servoId": 2, "goal": 2560, "degrees": 45.0}
    ]


def test_reading_state_never_touches_the_serial_port(tmp_path: Path) -> None:
    """The browser polls this several times a second.

    A three-servo STATUS is ~2 KB, which is ~175 ms of wire time at 115200 baud;
    issuing one per poll saturated the link and left HOLD_SET and MOVE queued
    behind it. Only the Pi's own service loop may refresh telemetry.
    """

    class CountingController(ReplayArmController):
        refreshes = 0

        def status(self) -> dict[str, object]:
            type(self).refreshes += 1
            return self.transport_state()

    controller = CountingController(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    with _client(tmp_path, controller) as client:
        for _ in range(5):
            body = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert CountingController.refreshes == 0
    assert "refreshMs" in body


def test_releasing_torque_tells_the_controller_the_hold_set_is_empty(tmp_path: Path) -> None:
    """Skipping the empty HOLD_SET left the controller holding a set the Pi
    believed it had dropped."""

    sets: list[list[int]] = []

    class RecordingController(ReplayArmController):
        def set_hold_servos(self, servo_ids: list[int], lease_ms: int = 1_500) -> dict[str, object]:
            sets.append(list(servo_ids))
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = RecordingController(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        client.post("/api/robot/arm/torque", headers=_headers(), json={"hold": [1, 2, 3]})
        client.post("/api/robot/arm/torque", headers=_headers(), json={"hold": []})

    assert sets[0] == [1, 2, 3]
    assert sets[-1] == []


def test_a_stranded_energised_servo_is_dropped_before_being_held(tmp_path: Path) -> None:
    """Otherwise it can never be held or driven again without a power cycle.

    The firmware only adds a servo to the hold set when that servo reads
    torque-off, so a servo left energised with no controller authority makes
    every HOLD_SET fail with TORQUE_UNCONFIRMED.
    """

    order: list[str] = []

    class StrandedController(ReplayArmController):
        def torque_off(self, servo_id: int | None = None) -> dict[str, object]:
            order.append(f"off:{servo_id}")
            return super().torque_off(servo_id)

        def set_hold_servos(self, servo_ids: list[int], lease_ms: int = 1_500) -> dict[str, object]:
            order.append(f"hold:{sorted(servo_ids)}")
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = StrandedController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
    )
    _establish_base_frame(controller)
    controller._servos[1]["torqueState"] = "on"
    with _client(tmp_path, controller) as client:
        client.post("/api/robot/arm/torque", headers=_headers(), json={"hold": [1]})

    assert order[0] == "off:1"
    assert order[1] == "hold:[1]"


def test_a_latched_stop_is_reported_and_can_be_cleared(tmp_path: Path) -> None:
    """`transport_state()` has no "stop" key; reading one made every latched STOP
    invisible while the controller refused every command."""

    controller = _connected()
    with _client(tmp_path, controller) as client:
        client.post("/api/robot/arm/stop", headers=_headers())
        stopped = client.get("/api/robot/arm/state", headers=_headers()).json()
        assert stopped["stopped"] is True

        cleared = client.post("/api/robot/arm/clear-stop", headers=_headers()).json()

    assert cleared["stopped"] is False


def test_one_request_drives_every_joint_and_takes_torque_once(tmp_path: Path) -> None:
    """Three separate calls meant three round trips and three HOLD_SETs per frame
    of a drag, and the joints visibly started at different times."""

    holds = 0

    class CountingController(ReplayArmController):
        def set_hold_servos(self, servo_ids: list[int], lease_ms: int = 1_500) -> dict[str, object]:
            nonlocal holds
            holds += 1
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = CountingController(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        for name in ("joint_1", "joint_2", "joint_3"):
            client.post(
                f"/api/robot/arm/joints/{name}/calibrate",
                headers=_headers(),
                json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
            )
        body = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_1": 10, "joint_2": -10, "joint_3": 20},
        ).json()

    assert [entry["servoId"] for entry in body["moved"]] == [1, 2, 3]
    assert holds == 1
    assert len(
        [command for command in controller.commands if command["operation"] == "MOVE_SET"]
    ) == 1
    assert not any(
        command["operation"] in {"MOVE", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_live_follow_coalesces_latest_shoulder_elbow_frame_and_reports_feedback(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Keep the background owner asleep until the explicit orderly End below.
    # Otherwise either correct scheduling outcome is possible: the first frame
    # may dispatch before the second request arrives, or the second may replace
    # it. This test specifically proves the latter, so make that ordering exact.
    monkeypatch.setattr(simple_arm_api_module, "SERVICE_TICK_SECONDS", 60.0)
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        assert state["controller"]["liveFollowV1"] is True
        controller.commands.clear()
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json={
                "startAttemptId": "coalesce_start_attempt_0001",
                "joint_2": {
                    "speed": 2400,
                    "accel": 43,
                    "maxDeltaDegrees": 75,
                },
                "joint_3": {
                    "speed": 333,
                    "accel": 7,
                    "maxDeltaDegrees": 46,
                },
            },
        )
        assert started.status_code == 200, started.text
        receipt = started.json()
        session_id = receipt["sessionId"]
        assert receipt["schema"] == "arm-live-follow-v2"
        assert receipt["state"] == "active"
        assert receipt["settings"] == {
            "joint_2": {
                "speed": 2400,
                "accel": 43,
                "maxDeltaDegrees": 75,
            },
            "joint_3": {
                "speed": 333,
                "accel": 7,
                "maxDeltaDegrees": 46,
            },
        }
        assert receipt["envelope"]["maxDeltaDegrees"] == {
            "joint_2": 75,
            "joint_3": 46,
        }
        assert set(receipt["measured"]) == {"joint_2", "joint_3"}

        first = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 1,
                "joint_2": -4,
                "joint_3": 4,
            },
        )
        latest = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 2,
                "joint_2": -8,
                "joint_3": 8,
            },
        )
        assert first.status_code == latest.status_code == 200
        assert latest.json()["stats"]["lastAcceptedSequence"] == 2
        assert latest.json()["stats"]["coalescedFrames"] >= 1

        ended = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": True},
        )
        assert ended.status_code == 200, ended.text
        assert ended.json()["state"] == "ended"
        assert ended.json()["stats"]["dispatchedFrames"] == 1
        assert isinstance(
            ended.json()["measured"]["joint_2"]["velocityDegreesPerSecond"], float
        )
        assert (
            ended.json()["stats"]["lastAcceptedSequence"]
            == ended.json()["stats"]["lastDispatchedSequence"]
            == 2
        )

    holds = [
        command["servoIds"] for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert holds[0] == [2, 3]
    assert [] not in holds
    follow_commands = [
        command for command in controller.commands
        if command["operation"] == "FOLLOW_SET"
    ]
    assert len(follow_commands) == 1
    assert follow_commands[0]["moves"] == [
        {
            "servoId": 2,
            "goal": follow_commands[0]["moves"][0]["goal"],
            "speed": 2400,
                "acceleration": 43,
        },
        {
            "servoId": 3,
            "goal": follow_commands[0]["moves"][1]["goal"],
            "speed": 333,
            "acceleration": 7,
        },
    ]

@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        (
            {
                "joint_2": {"speed": 1, "accel": 1},
                "joint_3": {"speed": 2400, "accel": 50},
            },
            {
                "joint_2": {
                    "speed": 1,
                    "accel": 1,
                    "maxDeltaDegrees": 30,
                },
                "joint_3": {
                    "speed": 2400,
                    "accel": 50,
                    "maxDeltaDegrees": 30,
                },
            },
        ),
        (
            {
                "joint_2": {
                    "speed": 2400,
                    "accel": 50,
                    "maxDeltaDegrees": 90,
                },
                "joint_3": {
                    "speed": 1,
                    "accel": 1,
                    "maxDeltaDegrees": 1,
                },
            },
            {
                "joint_2": {
                    "speed": 2400,
                    "accel": 50,
                    "maxDeltaDegrees": 90,
                },
                "joint_3": {
                    "speed": 1,
                    "accel": 1,
                    "maxDeltaDegrees": 1,
                },
            },
        ),
    ],
)
def test_live_follow_accepts_independent_setting_endpoints_without_persisting(
    tmp_path: Path,
    payload: dict[str, object],
    expected: dict[str, object],
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        before = client.get("/api/robot/arm/state", headers=_headers()).json()
        persisted_before = {
            row["id"]: (row["speed"], row["accel"])
            for row in before["joints"]
            if row["id"] in {"joint_2", "joint_3"}
        }
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json={
                "startAttemptId": (
                    f"endpoint_start_attempt_{next(_LIVE_FOLLOW_START_ATTEMPTS):08d}"
                ),
                **payload,
            },
        )
        assert started.status_code == 200, started.text
        body = started.json()
        assert body["settings"] == expected
        assert "speed" not in body and "accel" not in body
        during = client.get("/api/robot/arm/state", headers=_headers()).json()
        cancelled = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": body["sessionId"], "flushPending": False},
        )
        after = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert cancelled.status_code == 200, cancelled.text
    for snapshot in (during, after):
        assert {
            row["id"]: (row["speed"], row["accel"])
            for row in snapshot["joints"]
            if row["id"] in {"joint_2", "joint_3"}
        } == persisted_before


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"joint_2": {"speed": 100, "accel": 10}},
        {"joint_3": {"speed": 100, "accel": 10}},
        {
            "joint_2": {"accel": 10},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 0, "accel": 10},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10},
            "joint_3": {"speed": 2401, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 0},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10},
            "joint_3": {"speed": 100, "accel": 51},
        },
        {
            "joint_2": {"speed": True, "accel": 10},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": "10"},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100.0, "accel": 10},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10.0},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10, "maxDeltaDegrees": 0},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10},
            "joint_3": {"speed": 100, "accel": 10, "maxDeltaDegrees": 91},
        },
        {
            "joint_2": {"speed": 100, "accel": 10, "extra": 1},
            "joint_3": {"speed": 100, "accel": 10},
        },
        {
            "joint_2": {"speed": 100, "accel": 10},
            "joint_3": {"speed": 100, "accel": 10},
            "extra": 1,
        },
    ],
)
def test_live_follow_start_rejects_missing_extra_out_of_range_and_wrong_types(
    tmp_path: Path,
    payload: dict[str, object],
) -> None:
    controller = _planning_controller()
    controller.commands.clear()
    with _client(tmp_path, controller) as client:
        refused = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json={
                "startAttemptId": (
                    f"invalid_start_attempt_{next(_LIVE_FOLLOW_START_ATTEMPTS):08d}"
                ),
                **payload,
            },
        )

    assert refused.status_code == 422
    assert not any(
        command["operation"] == "HOLD_SET" for command in controller.commands
    )


@pytest.mark.parametrize("nonfinite", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize("field", ["speed", "accel", "maxDeltaDegrees"])
def test_live_follow_start_contract_rejects_nonfinite_numbers(
    field: str,
    nonfinite: float,
) -> None:
    payload = _live_follow_settings()
    payload["joint_2"][field] = nonfinite  # type: ignore[assignment]
    with pytest.raises(ValueError):
        LiveFollowStartRequest.model_validate(payload)


def test_live_follow_start_attempt_contract_is_strict_and_bounded() -> None:
    payload = _live_follow_settings()
    payload.update(
        {
            "startAttemptId": "attempt_12345678",
            "startTimeoutMs": 1500,
        }
    )

    request = LiveFollowStartRequest.model_validate(payload)

    assert request.startAttemptId == "attempt_12345678"
    assert request.startTimeoutMs == 1500
    assert (
        LiveFollowStartRequest.model_validate(_live_follow_settings()).startTimeoutMs
        == 1500
    )
    for invalid in (
        {"startAttemptId": "short", "startTimeoutMs": 1500},
        {"startAttemptId": "attempt with spaces", "startTimeoutMs": 1500},
        {"startAttemptId": "attempt_12345678", "startTimeoutMs": 0},
        {"startAttemptId": "attempt_12345678", "startTimeoutMs": 1501},
        {"startAttemptId": "attempt_12345678", "startTimeoutMs": 1500.0},
    ):
        candidate = _live_follow_settings()
        candidate.update(invalid)
        with pytest.raises(ValueError):
            LiveFollowStartRequest.model_validate(candidate)


def test_live_follow_start_requires_a_caller_owned_attempt_before_any_hold(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service_calls = 0
    original_start = simple_arm_api_module.ArmService.start_live_follow

    def tracked_start(service: object, request: LiveFollowStartRequest) -> dict[str, object]:
        nonlocal service_calls
        service_calls += 1
        return original_start(service, request)  # type: ignore[arg-type]

    monkeypatch.setattr(
        simple_arm_api_module.ArmService,
        "start_live_follow",
        tracked_start,
    )
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        payload = _live_follow_settings()
        payload.pop("startAttemptId")

        refused = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=payload,
        )

    assert refused.status_code == 422
    assert service_calls == 0
    assert not any(
        command["operation"] == "HOLD_SET" for command in controller.commands
    )


def test_live_follow_start_releases_hold_when_post_hold_health_read_raises(
    tmp_path: Path,
) -> None:
    """Once HOLD_SET succeeds, every failed start exit must revoke it."""

    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class PostHoldHealthFailure(ReplayArmController):
        fail_post_hold_health = False

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            result = super().set_hold_servos(servo_ids, lease_ms)
            if servo_ids:
                self.fail_post_hold_health = True
            return result

        def transport_state(self) -> dict[str, object]:
            if self.fail_post_hold_health:
                self.fail_post_hold_health = False
                raise ControllerTransportError("private serial failure detail")
            return super().transport_state()

    controller = PostHoldHealthFailure(
        connected=True,
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    controller.commands.clear()

    with pytest.raises(ControllerTransportError):
        service.start_live_follow(
            LiveFollowStartRequest.model_validate(_live_follow_settings())
        )

    with service._lock:
        assert service._live_follow is None
        assert service._held == []
    assert sorted(controller._leases) == []
    hold_sets = [
        command["servoIds"]
        for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert hold_sets == [[2, 3], []]


def test_live_follow_start_cannot_commit_after_stop_wins_the_state_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """STOP between post-hold health and commit must prevent session install."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    original_digest = service._configuration_digest
    stopped_at_commit = False

    def stop_immediately_before_commit() -> str:
        nonlocal stopped_at_commit
        if not stopped_at_commit:
            stopped_at_commit = True
            service.stop_for_commissioning()
        return original_digest()

    monkeypatch.setattr(service, "_configuration_digest", stop_immediately_before_commit)
    controller.commands.clear()

    with pytest.raises(HTTPException) as refused:
        service.start_live_follow(
            LiveFollowStartRequest.model_validate(_live_follow_settings())
        )

    assert refused.value.status_code == 409
    assert "continuity" in str(refused.value.detail).lower()
    with service._lock:
        assert service._live_follow is None
        assert service._held == []
        assert service._operator_stopped is True
    assert sorted(controller._leases) == []
    assert any(command["operation"] == "STOP" for command in controller.commands)


def test_live_follow_start_deadline_prevents_a_late_post_hold_commit(
    tmp_path: Path,
) -> None:
    """A client-abandoned start cannot install a session after its Pi deadline."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class PausedPostHoldHealth(ReplayArmController):
        pause_next_health = False

        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[
                    _servo(1, 2048),
                    _servo(2, 2048),
                    _servo(3, 2048),
                    _servo(4, 512),
                ],
            )
            self.health_entered = threading.Event()
            self.allow_health = threading.Event()

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            result = super().set_hold_servos(servo_ids, lease_ms)
            if servo_ids:
                self.pause_next_health = True
            return result

        def transport_state(self) -> dict[str, object]:
            if self.pause_next_health:
                self.pause_next_health = False
                self.health_entered.set()
                assert self.allow_health.wait(2.0)
            return super().transport_state()

    controller = PausedPostHoldHealth()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    payload = _live_follow_settings()
    payload.update(
        {
            "startAttemptId": "deadline_attempt_1234",
            "startTimeoutMs": 50,
        }
    )
    request = LiveFollowStartRequest.model_validate(payload)
    results: list[dict[str, object]] = []
    failures: list[BaseException] = []

    def start() -> None:
        try:
            results.append(service.start_live_follow(request))
        except BaseException as error:
            failures.append(error)

    thread = threading.Thread(target=start)
    try:
        thread.start()
        assert controller.health_entered.wait(1.0)
        time.sleep(0.1)
        controller.allow_health.set()
        thread.join(timeout=2.0)
    finally:
        controller.allow_health.set()
        thread.join(timeout=2.0)

    assert thread.is_alive() is False
    assert results == []
    assert len(failures) == 1
    assert isinstance(failures[0], HTTPException)
    assert failures[0].status_code == 409  # type: ignore[union-attr]
    assert "deadline" in str(failures[0].detail).lower()  # type: ignore[union-attr]
    with service._lock:
        assert service._live_follow is None
        assert service._held == []
    assert sorted(controller._leases) == []


def test_live_follow_start_attempt_id_reconciles_one_committed_session(
    tmp_path: Path,
) -> None:
    """A duplicate attempt ID returns its receipt and never takes another hold."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    payload = _live_follow_settings()
    payload.update(
        {
            "startAttemptId": "committed_attempt_1234",
            "startTimeoutMs": 1500,
        }
    )
    request = LiveFollowStartRequest.model_validate(payload)
    controller.commands.clear()

    first = service.start_live_follow(request)
    duplicate = service.start_live_follow(request)

    assert first["startAttemptId"] == "committed_attempt_1234"
    assert duplicate["startAttemptId"] == "committed_attempt_1234"
    assert duplicate["sessionId"] == first["sessionId"]
    hold_sets = [
        command["servoIds"]
        for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert hold_sets == [[2, 3]]


def test_live_follow_start_cancel_tombstones_an_attempt_before_it_arrives(
    tmp_path: Path,
) -> None:
    """A delayed proxy request cannot reuse an ID cancelled ahead of arrival."""

    from robot_gateway.simple_arm_api import ArmService, JointStore, create_simple_arm_router

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    app = FastAPI()
    app.include_router(create_simple_arm_router(controller, arm_service=service))
    payload = _live_follow_settings()
    payload.update(
        {
            "startAttemptId": "cancelled_attempt_1234",
            "startTimeoutMs": 1500,
        }
    )
    controller.commands.clear()

    with TestClient(app) as client:
        cancelled = client.post(
            "/api/robot/arm/live-follow/start/cancel",
            json={"startAttemptId": "cancelled_attempt_1234"},
        )
        refused = client.post(
            "/api/robot/arm/live-follow/start",
            json=payload,
        )

    assert cancelled.status_code == 200, cancelled.text
    assert cancelled.json() == {
        "schema": "arm-live-follow-v2",
        "startAttemptId": "cancelled_attempt_1234",
        "cancelled": True,
        "inputLeaseMs": 400,
    }
    assert refused.status_code == 409, refused.text
    assert "no longer reusable" in str(refused.json()["detail"])
    assert not any(
        command["operation"] == "HOLD_SET" for command in controller.commands
    )


def test_live_follow_start_cancel_waits_for_in_flight_hold_cleanup(
    tmp_path: Path,
) -> None:
    """A cancel confirmation means a post-HOLD start has finished releasing."""

    from robot_gateway.simple_arm_api import (
        ArmService,
        JointStore,
        LiveFollowStartCancelRequest,
    )

    class PausedPostHoldHealth(ReplayArmController):
        pause_next_health = False

        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[
                    _servo(1, 2048),
                    _servo(2, 2048),
                    _servo(3, 2048),
                    _servo(4, 512),
                ],
            )
            self.health_entered = threading.Event()
            self.allow_health = threading.Event()

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            result = super().set_hold_servos(servo_ids, lease_ms)
            if servo_ids:
                self.pause_next_health = True
            return result

        def transport_state(self) -> dict[str, object]:
            if self.pause_next_health:
                self.pause_next_health = False
                self.health_entered.set()
                assert self.allow_health.wait(2.0)
            return super().transport_state()

    controller = PausedPostHoldHealth()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    payload = _live_follow_settings()
    payload.update(
        {
            "startAttemptId": "inflight_cancel_attempt_1234",
            "startTimeoutMs": 1500,
        }
    )
    request = LiveFollowStartRequest.model_validate(payload)
    cancel_request = LiveFollowStartCancelRequest(
        startAttemptId="inflight_cancel_attempt_1234"
    )
    start_failures: list[BaseException] = []
    cancel_results: list[dict[str, object]] = []

    def start() -> None:
        try:
            service.start_live_follow(request)
        except BaseException as error:
            start_failures.append(error)

    def cancel() -> None:
        cancel_results.append(service.cancel_live_follow_start(cancel_request))

    start_thread = threading.Thread(target=start)
    cancel_thread = threading.Thread(target=cancel)
    try:
        start_thread.start()
        assert controller.health_entered.wait(1.0)
        cancel_thread.start()
        time.sleep(0.05)
        assert cancel_thread.is_alive() is True
        controller.allow_health.set()
        start_thread.join(timeout=2.0)
        cancel_thread.join(timeout=2.0)
    finally:
        controller.allow_health.set()
        start_thread.join(timeout=2.0)
        cancel_thread.join(timeout=2.0)

    assert start_thread.is_alive() is cancel_thread.is_alive() is False
    assert len(start_failures) == 1
    assert isinstance(start_failures[0], HTTPException)
    assert "cancelled" in str(start_failures[0].detail).lower()  # type: ignore[union-attr]
    assert cancel_results == [
        {
            "schema": "arm-live-follow-v2",
            "startAttemptId": "inflight_cancel_attempt_1234",
            "cancelled": True,
            "inputLeaseMs": 400,
        }
    ]
    with service._lock:
        assert service._live_follow is None
        assert service._held == []
    assert sorted(controller._leases) == []
    with pytest.raises(HTTPException) as reused:
        service.start_live_follow(request)
    assert reused.value.status_code == 409


def test_live_follow_start_cancel_releases_a_matching_committed_session(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        payload = _live_follow_settings()
        payload.update(
            {
                "startAttemptId": "committed_cancel_attempt_1234",
                "startTimeoutMs": 1500,
            }
        )
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=payload,
        )
        cancelled = client.post(
            "/api/robot/arm/live-follow/start/cancel",
            headers=_headers(),
            json={"startAttemptId": "committed_cancel_attempt_1234"},
        )

    assert started.status_code == 200, started.text
    assert cancelled.status_code == 200, cancelled.text
    body = cancelled.json()
    assert body["sessionId"] == started.json()["sessionId"]
    assert body["startAttemptId"] == "committed_cancel_attempt_1234"
    assert body["state"] == "ended"
    assert body["reason"] == "Fast Follow start attempt cancelled by the client."
    assert sorted(controller._leases) == []


def test_live_follow_state_fails_closed_when_compact_readback_is_not_advertised(
    tmp_path: Path,
) -> None:
    class MotionFeedbackOnlyController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            if isinstance(identity, dict):
                capabilities = identity.get("capabilities")
                if isinstance(capabilities, list):
                    identity["capabilities"] = [
                        capability
                        for capability in capabilities
                        if capability != "follow_feedback_v1"
                    ]
            return reported

    controller = MotionFeedbackOnlyController(
        connected=True,
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        assert state["controller"]["liveFollowV1"] is False
        refused = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(),
        )

    assert refused.status_code == 409
    assert "does not provide bounded Fast Follow feedback" in refused.json()["detail"]


def test_live_follow_refuses_a_fresh_start_below_the_floor_guard(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        # Elbow -90 degrees puts the measured tip at 20 mm. Admission must
        # fail before Fast Follow takes a Shoulder/Elbow hold.
        controller._servos[3]["rawPosition"] = 1024
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(),
        )

    assert refused.status_code == 409
    assert "below the 40 mm floor guard" in refused.json()["detail"]
    assert not any(
        command["operation"] == "HOLD_SET" for command in controller.commands
    )


def test_live_follow_refuses_a_start_pose_outside_calibrated_reach(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        narrowed = client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawMin": 1800, "rawMax": 2300},
        )
        assert narrowed.status_code == 200, narrowed.text
        controller._servos[2]["rawPosition"] = 1600
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(),
        )

    assert refused.status_code == 409, refused.text
    assert "outside its calibrated reachable limits" in refused.json()["detail"]
    assert not any(
        command["operation"] == "HOLD_SET" for command in controller.commands
    )


def test_live_follow_intersects_requested_span_with_calibrated_reach(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        narrowed = client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawMin": 1800, "rawMax": 2300},
        )
        assert narrowed.status_code == 200, narrowed.text
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_span=90,
                joint_3_span=45,
            ),
        ).json()
        bounded = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": started["sessionId"],
                "sequence": 1,
                "joint_2": 80,
                "joint_3": 20,
            },
        )
        cancelled = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": started["sessionId"], "flushPending": False},
        )

    assert bounded.status_code == 200, bounded.text
    body = bounded.json()
    calibrated_high = (2300 - 2048) * 360 / 4096
    assert body["commanded"]["joint_2"] <= calibrated_high + 0.1
    assert body["commanded"]["joint_2"] >= calibrated_high - 0.1
    assert body["commanded"]["joint_3"] == pytest.approx(20, abs=0.1)
    assert body["envelope"]["lowestPointMm"] >= 40
    assert body["stats"]["clampedFrames"] == 1
    assert cancelled.status_code == 200


def test_live_follow_cumulative_floor_proof_clamps_an_asymmetric_span(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_span=90,
                joint_3_span=45,
            ),
        ).json()
        first = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": started["sessionId"],
                "sequence": 1,
                "joint_2": -40,
                "joint_3": 15,
            },
        )
        second = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": started["sessionId"],
                "sequence": 2,
                "joint_2": -50,
                "joint_3": 15,
            },
        )
        cancelled = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": started["sessionId"], "flushPending": False},
        )

    assert first.status_code == second.status_code == 200
    assert first.json()["envelope"]["lowestPointMm"] >= 40
    body = second.json()
    assert body["commanded"]["joint_2"] > -50
    assert body["commanded"]["joint_3"] == pytest.approx(15, abs=0.1)
    assert body["envelope"]["lowestPointMm"] >= 40
    assert body["stats"]["clampedFrames"] == 1
    assert cancelled.status_code == 200


def test_live_follow_end_flushes_the_newest_pending_target_once(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=1600,
                joint_3_speed=1600,
            ),
        ).json()
        session_id = started["sessionId"]
        accepted = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 1,
                "joint_2": -12,
                "joint_3": 12,
            },
        )
        assert accepted.status_code == 200
        ended = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": True},
        )

    assert ended.status_code == 200, ended.text
    assert ended.json()["state"] == "ended"
    assert ended.json()["stats"]["lastAcceptedSequence"] == 1
    assert ended.json()["stats"]["lastDispatchedSequence"] == 1
    assert ended.json()["stats"]["dispatchedFrames"] == 1
    assert len(
        [command for command in controller.commands if command["operation"] == "FOLLOW_SET"]
    ) == 1


def test_live_follow_same_target_keepalive_renews_without_dispatch(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=900,
                joint_3_speed=900,
            ),
        ).json()
        session_id = started["sessionId"]
        target = started["commanded"]
        latest = started
        for sequence in range(1, 6):
            time.sleep(0.12)
            response = client.post(
                "/api/robot/arm/live-follow/frame",
                headers=_headers(),
                json={
                    "sessionId": session_id,
                    "sequence": sequence,
                    "joint_2": target["joint_2"],
                    "joint_3": target["joint_3"],
                },
            )
            assert response.status_code == 200, response.text
            latest = response.json()

        assert latest["state"] == "active"
        assert latest["stats"]["receivedFrames"] == 5
        assert latest["stats"]["lastAcceptedSequence"] == 5
        assert latest["stats"]["dispatchedFrames"] == 0
        assert latest["stats"]["coalescedFrames"] == 0
        assert not any(
            command["operation"] == "FOLLOW_SET" for command in controller.commands
        )
        ended = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": True},
        ).json()

    assert ended["state"] == "ended"
    assert ended["stats"]["lastAcceptedSequence"] == 5
    assert ended["stats"]["lastDispatchedSequence"] == 5
    assert ended["stats"]["dispatchedFrames"] == 0
    assert not any(
        command["operation"] == "FOLLOW_SET" for command in controller.commands
    )


def test_live_follow_heartbeat_renews_lease_without_touching_motion_stats(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=900,
                joint_3_speed=900,
            ),
        ).json()
        session_id = started["sessionId"]
        expected_stats = started["stats"]

        latest = started
        for _ in range(5):
            time.sleep(0.12)
            heartbeat = client.post(
                "/api/robot/arm/live-follow/heartbeat",
                headers=_headers(),
                json={"sessionId": session_id},
            )
            assert heartbeat.status_code == 200, heartbeat.text
            latest = heartbeat.json()

        assert latest["state"] == "active"
        assert latest["stats"] == expected_stats
        assert not any(
            command["operation"] == "FOLLOW_SET" for command in controller.commands
        )

        ended = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": False},
        )
        terminal_heartbeat = client.post(
            "/api/robot/arm/live-follow/heartbeat",
            headers=_headers(),
            json={"sessionId": session_id},
        )
        wrong_session = client.post(
            "/api/robot/arm/live-follow/heartbeat",
            headers=_headers(),
            json={"sessionId": "armfollow_wrong_session_1234"},
        )

    assert ended.status_code == 200, ended.text
    assert ended.json()["state"] == "ended"
    assert terminal_heartbeat.status_code == 200, terminal_heartbeat.text
    assert terminal_heartbeat.json()["state"] == "ended"
    assert wrong_session.status_code == 409


def test_live_follow_heartbeat_bypasses_a_frame_waiting_for_its_http_body(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The independent deadman lane must not queue behind frame admission."""

    from robot_gateway.simple_arm_api import ArmService, JointStore, create_simple_arm_router

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    original_strict_body = simple_arm_api_module._strict_body

    async def scenario() -> tuple[
        httpx.Response,
        httpx.Response,
        list[httpx.Response],
        float,
        float,
    ]:
        frame_body_entered = asyncio.Event()
        release_frame_body = asyncio.Event()

        async def pause_frame_body(http_request, contract):
            if contract is simple_arm_api_module.LiveFollowFrameRequest:
                frame_body_entered.set()
                await release_frame_body.wait()
            return await original_strict_body(http_request, contract)

        app = FastAPI()
        app.include_router(create_simple_arm_router(controller, arm_service=service))
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://gateway.test"
        ) as client:
            for name in ("joint_1", "joint_2", "joint_3"):
                calibrated = await client.post(
                    f"/api/robot/arm/joints/{name}/calibrate",
                    json={"rawZero": 2048, "rawMin": 0, "rawMax": 4095, "ratio": 1},
                )
                assert calibrated.status_code == 200, calibrated.text
            camera = await client.post(
                "/api/robot/arm/joints/joint_4/calibrate",
                json={"rawZero": 512, "rawMin": 0, "rawMax": 1023, "ratio": 1},
            )
            assert camera.status_code == 200, camera.text
            started = await client.post(
                "/api/robot/arm/live-follow/start",
                json=_live_follow_settings(),
            )
            assert started.status_code == 200, started.text
            session_id = started.json()["sessionId"]
            with service._lock:
                active = service._live_follow
                assert active is not None
                # Make renewal observable without relying on the host clock
                # advancing between two adjacent requests. Windows may expose
                # the same monotonic tick even after a short asyncio sleep.
                active["lastInputMonotonic"] = (
                    float(active["lastInputMonotonic"])
                    - simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS / 2
                )
                before = float(active["lastInputMonotonic"])

            monkeypatch.setattr(simple_arm_api_module, "_strict_body", pause_frame_body)
            frame_task = asyncio.create_task(
                client.post(
                    "/api/robot/arm/live-follow/frame",
                    json={
                        "sessionId": session_id,
                        "sequence": 1,
                        "joint_2": -5,
                        "joint_3": 5,
                    },
                )
            )
            await asyncio.wait_for(frame_body_entered.wait(), timeout=1.0)
            await asyncio.sleep(0.01)
            heartbeat = await client.post(
                "/api/robot/arm/live-follow/heartbeat",
                json={"sessionId": session_id},
            )
            blocked = [
                await client.post(
                    "/api/robot/arm/live-follow/start",
                    json=_live_follow_settings(),
                ),
                await client.post(
                    "/api/robot/arm/live-follow/frame",
                    json={
                        "sessionId": session_id,
                        "sequence": 2,
                        "joint_2": -6,
                        "joint_3": 6,
                    },
                ),
                await client.post(
                    "/api/robot/arm/live-follow/end",
                    json={"sessionId": session_id, "flushPending": False},
                ),
            ]
            with service._lock:
                active = service._live_follow
                assert active is not None
                after = float(active["lastInputMonotonic"])
            release_frame_body.set()
            frame = await asyncio.wait_for(frame_task, timeout=1.0)
        return heartbeat, frame, blocked, before, after

    try:
        heartbeat, frame, blocked, before, after = asyncio.run(scenario())
    finally:
        service.close()

    assert heartbeat.status_code == 200, heartbeat.text
    assert heartbeat.json()["state"] == "active"
    assert after > before
    assert [response.status_code for response in blocked] == [409, 409, 409]
    assert all(
        response.json()["detail"] == "Another physical arm operation is in progress."
        for response in blocked
    )
    assert frame.status_code == 200, frame.text


@pytest.mark.parametrize("input_kind", ["heartbeat", "frame"])
def test_late_live_follow_input_release_does_not_block_stop_on_the_event_loop(
    tmp_path: Path,
    input_kind: str,
) -> None:
    """A stalled empty HOLD_SET must not prevent FastAPI from scheduling STOP."""

    from robot_gateway.simple_arm_api import ArmService, JointStore, create_simple_arm_router

    class BlockingReleaseController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[
                    _servo(1, 2048),
                    _servo(2, 2048),
                    _servo(3, 2048),
                    _servo(4, 512),
                ],
            )
            self.block_empty_release = False
            self.release_entered = threading.Event()
            self.allow_release = threading.Event()
            self.on_release_entered = lambda: None
            self.stop_before_release = False

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            if self.block_empty_release and not servo_ids:
                self.release_entered.set()
                self.on_release_entered()
                assert self.allow_release.wait(3.0)
            return super().set_hold_servos(servo_ids, lease_ms)

        def stop(self) -> dict[str, object]:
            self.stop_before_release = not self.allow_release.is_set()
            return super().stop()

    controller = BlockingReleaseController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    with service._lock:
        active = service._live_follow
        assert active is not None
        active["lastInputMonotonic"] = (
            time.monotonic()
            - simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS
            - 0.01
        )
    controller.block_empty_release = True

    async def scenario() -> tuple[httpx.Response, httpx.Response]:
        app = FastAPI()
        app.include_router(create_simple_arm_router(controller, arm_service=service))
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://gateway.test"
        ) as client:
            loop = asyncio.get_running_loop()
            stop_tasks: list[asyncio.Task[httpx.Response]] = []

            def schedule_stop() -> None:
                loop.call_soon_threadsafe(
                    lambda: stop_tasks.append(
                        asyncio.create_task(client.post("/api/robot/arm/stop"))
                    )
                )

            controller.on_release_entered = schedule_stop
            safety_release = threading.Timer(1.5, controller.allow_release.set)
            safety_release.start()
            try:
                if input_kind == "heartbeat":
                    live_input = await client.post(
                        "/api/robot/arm/live-follow/heartbeat",
                        json={"sessionId": session_id},
                    )
                else:
                    live_input = await client.post(
                        "/api/robot/arm/live-follow/frame",
                        json={
                            "sessionId": session_id,
                            "sequence": 1,
                            "joint_2": -5,
                            "joint_3": 5,
                        },
                    )
                deadline = time.monotonic() + 2.0
                while not stop_tasks and time.monotonic() < deadline:
                    await asyncio.sleep(0.01)
                assert stop_tasks, "STOP was never scheduled"
                stopped = await asyncio.wait_for(stop_tasks[0], timeout=2.0)
                return live_input, stopped
            finally:
                controller.allow_release.set()
                safety_release.cancel()

    live_input, stopped = asyncio.run(scenario())

    assert controller.release_entered.is_set() is True
    assert live_input.status_code == 200, live_input.text
    assert stopped.status_code == 200, stopped.text
    assert controller.stop_before_release is True


def test_live_follow_expiry_revalidates_a_concurrent_heartbeat_at_finish_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A stale service-tick snapshot must not expire a lease renewed meanwhile."""

    from robot_gateway.simple_arm_api import (
        ArmService,
        JointStore,
        LiveFollowHeartbeatRequest,
    )

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    prior_input = time.monotonic()
    with service._lock:
        active = service._live_follow
        assert active is not None
        active["lastInputMonotonic"] = prior_input

    # Model a service tick whose expiry snapshot is slightly ahead of the
    # current request handler. The heartbeat itself still arrives comfortably
    # before the prior input's real 400 ms deadline.
    time.sleep(0.02)
    stale_tick_now = (
        prior_input + simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS + 0.005
    )

    original_finish = service._finish_live_follow
    renewal_observed = False

    def renew_immediately_before_finish(*args, **kwargs):
        nonlocal renewal_observed
        if not renewal_observed:
            renewed = service.live_follow_heartbeat(
                LiveFollowHeartbeatRequest(sessionId=session_id)
            )
            assert renewed["state"] == "active"
            with service._lock:
                active = service._live_follow
                assert active is not None
                renewed_at = float(active["lastInputMonotonic"])
            assert renewed_at < (
                prior_input + simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS
            )
            assert renewed_at > (
                stale_tick_now - simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS
            )
            renewal_observed = True
        return original_finish(*args, **kwargs)

    monkeypatch.setattr(service, "_finish_live_follow", renew_immediately_before_finish)

    service._dispatch_live_follow(stale_tick_now)

    assert renewal_observed is True
    with service._lock:
        assert service._live_follow is not None
        assert service._live_follow["sessionId"] == session_id
        assert service._live_follow["state"] == "active"
    assert service._last_live_follow is None


@pytest.mark.parametrize("input_kind", ["heartbeat", "frame"])
def test_live_follow_late_input_expires_without_queueing_or_dispatching(
    tmp_path: Path,
    input_kind: str,
) -> None:
    """Input after its prior 400 ms deadline cannot revive the session."""

    from robot_gateway.simple_arm_api import (
        ArmService,
        JointStore,
        LiveFollowFrameRequest,
        LiveFollowHeartbeatRequest,
    )

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    with service._lock:
        active = service._live_follow
        assert active is not None
        active["lastInputMonotonic"] = (
            time.monotonic()
            - simple_arm_api_module.LIVE_FOLLOW_INPUT_LEASE_SECONDS
            - 0.01
        )
    controller.commands.clear()

    if input_kind == "heartbeat":
        terminal = service.live_follow_heartbeat(
            LiveFollowHeartbeatRequest(sessionId=session_id)
        )
    else:
        terminal = service.live_follow_frame(
            LiveFollowFrameRequest(
                sessionId=session_id,
                sequence=1,
                joint_2=-5,
                joint_3=5,
            )
        )

    assert terminal["state"] == "expired"
    assert terminal["reason"] == "Fast Follow input lease expired."
    assert terminal["stats"]["receivedFrames"] == 0
    assert terminal["stats"]["lastAcceptedSequence"] == 0
    assert not any(
        command["operation"] == "FOLLOW_SET" for command in controller.commands
    )
    with service._lock:
        assert service._live_follow is None
        assert service._held == []
    assert sorted(controller._leases) == []


def test_stale_old_flush_end_cannot_dispatch_a_new_sessions_pending_frame(
    tmp_path: Path,
) -> None:
    """End must bind its pending/flush work to the requested session id."""

    from robot_gateway.simple_arm_api import (
        ArmService,
        JointStore,
        LiveFollowEndRequest,
        LiveFollowFrameRequest,
    )

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    old_request = LiveFollowStartRequest.model_validate(_live_follow_settings())
    old = service.start_live_follow(old_request)
    old_session_id = str(old["sessionId"])
    old_terminal = service._finish_live_follow(
        old_session_id,
        state_name="expired",
        reason="Fast Follow input lease expired.",
        release=True,
    )
    assert old_terminal is not None
    new_request = LiveFollowStartRequest.model_validate(_live_follow_settings())
    new = service.start_live_follow(new_request)
    new_session_id = str(new["sessionId"])
    service.live_follow_frame(
        LiveFollowFrameRequest(
            sessionId=new_session_id,
            sequence=1,
            joint_2=-5,
            joint_3=5,
        )
    )
    controller.commands.clear()

    with pytest.raises(HTTPException) as stale_end:
        service.end_live_follow(
            LiveFollowEndRequest(
                sessionId=old_session_id,
                flushPending=True,
            )
        )

    assert stale_end.value.status_code == 409
    assert not any(
        command["operation"] == "FOLLOW_SET" for command in controller.commands
    )
    with service._lock:
        assert service._live_follow is not None
        assert service._live_follow["sessionId"] == new_session_id
        pending = service._live_follow["pending"]
        assert isinstance(pending, dict)
        assert pending["sequence"] == 1
        assert service._held == [2, 3]


def test_old_live_follow_finisher_cannot_revoke_a_new_session_hold(
    tmp_path: Path,
) -> None:
    """Session removal and physical release share authority ownership."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class TrackingAuthorityLock:
        def __init__(self) -> None:
            self._inner = threading.RLock()
            self._meta = threading.Lock()
            self.owner: str | None = None
            self.old_acquired = threading.Event()
            self.new_acquired = threading.Event()

        def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
            if timeout == -1:
                acquired = self._inner.acquire(blocking)
            else:
                acquired = self._inner.acquire(blocking, timeout)
            if acquired:
                name = threading.current_thread().name
                with self._meta:
                    self.owner = name
                if name == "old-finisher":
                    self.old_acquired.set()
                elif name == "new-starter":
                    self.new_acquired.set()
            return acquired

        def release(self) -> None:
            with self._meta:
                self.owner = None
            self._inner.release()

        def __enter__(self):
            self.acquire()
            return self

        def __exit__(self, *_args: object) -> None:
            self.release()

    controller = _planning_controller()
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    first_request = LiveFollowStartRequest.model_validate(_live_follow_settings())
    next_request = LiveFollowStartRequest.model_validate(_live_follow_settings())
    first = service.start_live_follow(first_request)
    first_session_id = str(first["sessionId"])

    authority = TrackingAuthorityLock()
    service._authority_lock = authority  # type: ignore[assignment]
    finish_paused = threading.Event()
    allow_finish = threading.Event()

    class PausingCondition(threading.Condition):
        def notify_all(self) -> None:
            if threading.current_thread().name == "old-finisher":
                finish_paused.set()
                assert allow_finish.wait(2.0)
            super().notify_all()

    service._telemetry_condition = PausingCondition(service._lock)
    finished: list[dict[str, object] | None] = []
    started: list[dict[str, object]] = []
    failures: list[BaseException] = []

    def finish_old() -> None:
        try:
            finished.append(
                service._finish_live_follow(
                    first_session_id,
                    state_name="expired",
                    reason="Fast Follow input lease expired.",
                    release=True,
                )
            )
        except BaseException as error:
            failures.append(error)

    def start_new() -> None:
        try:
            started.append(service.start_live_follow(next_request))
        except BaseException as error:
            failures.append(error)

    finisher = threading.Thread(target=finish_old, name="old-finisher")
    starter = threading.Thread(target=start_new, name="new-starter")
    try:
        finisher.start()
        assert finish_paused.wait(1.0)
        starter.start()
        acquired = authority.old_acquired.wait(0.2) or authority.new_acquired.wait(1.0)
        assert acquired is True
        assert authority.owner == "old-finisher"
        assert authority.new_acquired.is_set() is False
    finally:
        allow_finish.set()
        finisher.join(timeout=2.0)
        starter.join(timeout=2.0)

    assert finisher.is_alive() is False
    assert starter.is_alive() is False
    assert failures == []
    assert len(finished) == len(started) == 1
    assert finished[0] is not None
    assert started[0]["sessionId"] != first_session_id
    with service._lock:
        assert service._live_follow is not None
        assert service._live_follow["sessionId"] == started[0]["sessionId"]
        assert service._held == [2, 3]
    assert sorted(controller._leases) == [2, 3]
    hold_sets = [
        command["servoIds"]
        for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert hold_sets[-2:] == [[], [2, 3]]


@pytest.mark.parametrize(
    "recovery_state",
    [
        {"motionState": "blocked"},
        {"motionState": "ready", "safetyFault": True},
        {"motionState": "ready", "operatorInspectionRequired": True},
    ],
    ids=("blocked", "safety-fault", "inspection-required"),
)
def test_live_follow_idle_session_faults_on_controller_recovery_state(
    tmp_path: Path,
    recovery_state: dict[str, object],
) -> None:
    """No pending pointer frame may hide a controller recovery block."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    fault_fields: dict[str, object] = {}

    class RecoveryStateController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            reported.update(fault_fields)
            return reported

    controller = RecoveryStateController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    fault_fields.update(recovery_state)

    service._dispatch_live_follow(time.monotonic())
    terminal = service.live_follow_heartbeat(
        simple_arm_api_module.LiveFollowHeartbeatRequest(sessionId=session_id)
    )

    assert terminal["state"] == "faulted"
    assert terminal["reason"] == "Fast Follow lost safe controller continuity."
    assert terminal["fault"] == {"code": "FAST_FOLLOW_FAILED"}
    with service._lock:
        assert service._live_follow is None
        assert service._held == []
    assert sorted(controller._leases) == []


def test_live_follow_terminal_receipt_preserves_only_safe_controller_fault_evidence(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """A useful MOVE_SET failure reaches the UI without raw payload leakage."""

    from robot_gateway.simple_arm_api import ArmService, JointStore, LiveFollowFrameRequest

    class FailingFollowController(ReplayArmController):
        def follow_set(self, moves):
            raise ControllerCommandError(
                "MOVE_SET_FAILED",
                {
                    "phase": "verify",
                    "failedIndex": 0,
                    "stopped": False,
                    "privateDiagnostic": "never expose this raw controller detail",
                },
            )

    controller = FailingFollowController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    service.live_follow_frame(
        LiveFollowFrameRequest(
            sessionId=session_id,
            sequence=1,
            joint_2=-5,
            joint_3=5,
        )
    )

    service._dispatch_live_follow(time.monotonic(), force=True)
    terminal = service.live_follow_frame(
        LiveFollowFrameRequest(
            sessionId=session_id,
            sequence=2,
            joint_2=-6,
            joint_3=6,
        )
    )

    assert terminal["state"] == "faulted"
    assert terminal["reason"] == "Fast Follow lost safe controller feedback."
    assert terminal["fault"] == {
        "code": "MOVE_SET_FAILED",
        "phase": "verify",
        "failedIndex": 0,
        "stopped": False,
    }
    assert terminal["stats"]["lastAttemptedSequence"] == 1
    assert terminal["stats"]["lastDispatchedSequence"] == 0
    assert terminal["stats"]["lastAttemptLatencyMs"] >= 0
    matching_logs = [
        json.loads(record.message)
        for record in caplog.records
        if '"event":"arm_fast_follow_dispatch_failed"' in record.message
    ]
    assert len(matching_logs) == 1
    assert matching_logs[0] == {
        "event": "arm_fast_follow_dispatch_failed",
        "fault": {
            "code": "MOVE_SET_FAILED",
            "failedIndex": 0,
            "phase": "verify",
            "stopped": False,
        },
        "latencyMs": terminal["stats"]["lastAttemptLatencyMs"],
        "pendingSequence": 1,
        "sessionId": session_id,
        "settings": started["settings"],
    }
    assert "privateDiagnostic" not in repr(terminal)
    assert "privateDiagnostic" not in repr(matching_logs)


def test_live_follow_idle_readback_updates_without_redispatching_target(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=1800,
                joint_3_speed=1800,
            ),
        ).json()
        session_id = started["sessionId"]
        target = {"joint_2": -8, "joint_3": 8}
        accepted = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={"sessionId": session_id, "sequence": 1, **target},
        )
        assert accepted.status_code == 200, accepted.text

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            operations = [command["operation"] for command in controller.commands]
            if "FOLLOW_SET" in operations and "FOLLOW_READ" in operations:
                break
            time.sleep(0.01)

        observed = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={"sessionId": session_id, "sequence": 2, **target},
        )
        assert observed.status_code == 200, observed.text
        ended = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": True},
        )

    operations = [command["operation"] for command in controller.commands]
    assert operations.count("FOLLOW_SET") == 1
    assert operations.count("FOLLOW_READ") >= 1
    assert observed.json()["stats"]["dispatchedFrames"] == 1
    assert ended.status_code == 200, ended.text


def test_live_follow_tolerates_bounded_idle_feedback_gaps_then_recovers(
    tmp_path: Path,
) -> None:
    """Moving-servo read noise must not end Live mode on the first miss."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class TransientIdleFeedbackController(ReplayArmController):
        feedback_failures_remaining = 0

        def follow_read(self, servo_ids: list[int]) -> dict[str, object]:
            with self._lock:
                if self.feedback_failures_remaining > 0:
                    self._before("FOLLOW_READ", servoIds=list(servo_ids))
                    self.feedback_failures_remaining -= 1
                    raise ControllerCommandError(
                        "FEEDBACK_UNAVAILABLE", {"failedIndex": 0}
                    )
            return super().follow_read(servo_ids)

    controller = TransientIdleFeedbackController(
        connected=True,
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1

    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(
            _live_follow_settings(
                joint_2_speed=2400,
                joint_3_speed=2400,
                joint_2_accel=50,
                joint_3_accel=50,
                joint_2_span=90,
                joint_3_span=90,
            )
        )
    )
    session_id = str(started["sessionId"])
    controller.feedback_failures_remaining = (
        simple_arm_api_module.LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE
    )

    for expected_misses in range(
        1, simple_arm_api_module.LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE + 1
    ):
        with service._lock:
            assert service._live_follow is not None
            service._live_follow["lastFeedbackMonotonic"] = 0.0
        assert service._dispatch_live_follow(time.monotonic()) is True
        with service._lock:
            assert service._live_follow is not None
            assert (
                service._live_follow["consecutiveIdleFeedbackMisses"]
                == expected_misses
            )

    with service._lock:
        assert service._live_follow is not None
        service._live_follow["lastFeedbackMonotonic"] = 0.0
    assert service._dispatch_live_follow(time.monotonic()) is True

    active = service.live_follow_heartbeat(
        simple_arm_api_module.LiveFollowHeartbeatRequest(
            sessionId=session_id
        )
    )
    assert active["state"] == "active"
    with service._lock:
        assert service._live_follow is not None
        assert service._live_follow["consecutiveIdleFeedbackMisses"] == 0


def test_live_follow_faults_after_sustained_idle_feedback_loss(
    tmp_path: Path,
) -> None:
    """The transient allowance stays bounded and preserves fail-closed loss."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class MissingIdleFeedbackController(ReplayArmController):
        fail_feedback = False

        def follow_read(self, servo_ids: list[int]) -> dict[str, object]:
            if not self.fail_feedback:
                return super().follow_read(servo_ids)
            with self._lock:
                self._before("FOLLOW_READ", servoIds=list(servo_ids))
            raise ControllerCommandError(
                "FEEDBACK_UNAVAILABLE", {"failedIndex": 1}
            )

    controller = MissingIdleFeedbackController(
        connected=True,
        servos=[
            _servo(1, 2048),
            _servo(2, 2048),
            _servo(3, 2048),
            _servo(4, 512),
        ],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    for name in ("joint_1", "joint_2", "joint_3"):
        joint = service._store.get(name)
        joint.rawZero, joint.rawMin, joint.rawMax, joint.ratio = 2048, 0, 4095, 1
    camera = service._store.get("joint_4")
    camera.rawZero, camera.rawMin, camera.rawMax, camera.ratio = 512, 0, 1023, 1

    # Admission needs one valid compact read; fail only after the session exists.
    started = service.start_live_follow(
        LiveFollowStartRequest.model_validate(_live_follow_settings())
    )
    session_id = str(started["sessionId"])
    controller.fail_feedback = True

    for _ in range(
        simple_arm_api_module.LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE + 1
    ):
        with service._lock:
            if service._live_follow is None:
                break
            service._live_follow["lastFeedbackMonotonic"] = 0.0
        assert service._dispatch_live_follow(time.monotonic()) is True

    with service._lock:
        assert service._live_follow is None
        assert service._last_live_follow is not None
        assert service._last_live_follow["sessionId"] == session_id
        assert service._last_live_follow["state"] == "faulted"
        assert service._last_live_follow["fault"] == {
            "code": "FEEDBACK_UNAVAILABLE",
            "failedIndex": 1,
        }


def test_live_follow_cancel_discards_pending_and_releases_hold(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Keep the background dispatch lane asleep long enough to prove the HTTP
    # cancellation path itself never flushes the accepted target.
    monkeypatch.setattr(simple_arm_api_module, "SERVICE_TICK_SECONDS", 1.0)
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        session_id = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=1800,
                joint_3_speed=1800,
            ),
        ).json()["sessionId"]
        missing_cancel_policy = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id},
        )
        assert missing_cancel_policy.status_code == 422
        accepted = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 1,
                "joint_2": -12,
                "joint_3": 12,
            },
        )
        assert accepted.status_code == 200, accepted.text
        cancelled = client.post(
            "/api/robot/arm/live-follow/end",
            headers=_headers(),
            json={"sessionId": session_id, "flushPending": False},
        )

    assert cancelled.status_code == 200, cancelled.text
    assert cancelled.json()["state"] == "ended"
    assert cancelled.json()["reason"] == "Fast Follow cancelled by the client."
    assert cancelled.json()["stats"]["lastAcceptedSequence"] == 1
    assert cancelled.json()["stats"]["lastDispatchedSequence"] == 0
    assert not any(
        command["operation"] == "FOLLOW_SET" for command in controller.commands
    )
    holds = [
        command["servoIds"]
        for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert holds == [[2, 3], []]


def test_live_follow_clamps_span_rejects_other_mutations_and_stop_stays_live(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        started = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_span=70,
                joint_3_span=15,
            ),
        ).json()
        session_id = started["sessionId"]

        bounded = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 1,
                "joint_2": 100,
                "joint_3": 100,
            },
        )
        assert bounded.status_code == 200, bounded.text
        body = bounded.json()
        assert abs(body["commanded"]["joint_2"] - started["commanded"]["joint_2"]) <= 70.1
        assert abs(body["commanded"]["joint_3"] - started["commanded"]["joint_3"]) <= 15.1
        assert body["stats"]["clampedFrames"] == 1

        refused = client.post("/api/robot/arm/scan", headers=_headers())
        assert refused.status_code == 409
        assert "Fast Follow owns" in refused.json()["detail"]
        assert client.get("/api/robot/arm/state", headers=_headers()).status_code == 200

        stopped = client.post("/api/robot/arm/stop", headers=_headers())
        assert stopped.status_code == 200, stopped.text
        terminal = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 2,
                "joint_2": 0,
                "joint_3": 0,
            },
        )
        assert terminal.status_code == 200
        assert terminal.json()["state"] == "stopped"


def test_live_follow_input_deadman_expires_and_releases_hold(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        session_id = client.post(
            "/api/robot/arm/live-follow/start",
            headers=_headers(),
            json=_live_follow_settings(
                joint_2_speed=1000,
                joint_3_speed=1000,
            ),
        ).json()["sessionId"]
        time.sleep(0.65)
        expired = client.post(
            "/api/robot/arm/live-follow/frame",
            headers=_headers(),
            json={
                "sessionId": session_id,
                "sequence": 1,
                "joint_2": 0,
                "joint_3": 0,
            },
        )
        assert expired.status_code == 200, expired.text
        assert expired.json()["state"] == "expired"

    holds = [
        command["servoIds"] for command in controller.commands
        if command["operation"] == "HOLD_SET"
    ]
    assert holds[-1] == []


def test_multi_target_falls_back_safely_when_move_set_is_not_advertised(
    tmp_path: Path,
) -> None:
    class LegacyController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            if isinstance(identity, dict):
                capabilities = identity.get("capabilities")
                if isinstance(capabilities, list):
                    identity["capabilities"] = [
                        value for value in capabilities if value != "move_set_v1"
                    ]
            return reported

    controller = LegacyController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        for name in ("joint_1", "joint_2", "joint_3"):
            response = client.post(
                f"/api/robot/arm/joints/{name}/calibrate",
                headers=_headers(),
                json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
            )
            assert response.status_code == 200
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -10, "joint_3": 20},
        )

    assert response.status_code == 200, response.text
    operations = [command["operation"] for command in controller.commands]
    assert "MOVE_SET" not in operations
    assert operations.count("MOVE") == 2


@pytest.mark.parametrize(
    "targets",
    [
        {"joint_4": 5},
        {"joint_2": -5, "joint_4": 5},
    ],
)
def test_scs_motion_without_servo_family_capability_fails_before_torque(
    tmp_path: Path,
    targets: dict[str, int],
) -> None:
    class MissingFamilyController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            capabilities = (
                identity.get("capabilities") if isinstance(identity, dict) else None
            )
            if isinstance(capabilities, list):
                identity["capabilities"] = [
                    value for value in capabilities if value != "servo_family"
                ]
            return reported

    controller = MissingFamilyController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json=targets,
        )

    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "SERVO_FAMILY_UNAVAILABLE"
    assert not any(
        command["operation"]
        in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_sts_only_motion_remains_compatible_without_servo_family_capability(
    tmp_path: Path,
) -> None:
    class LegacyStsController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            capabilities = (
                identity.get("capabilities") if isinstance(identity, dict) else None
            )
            if isinstance(capabilities, list):
                identity["capabilities"] = [
                    value
                    for value in capabilities
                    if value not in {"servo_family", "move_set_v1"}
                ]
            return reported

    controller = LegacyStsController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        moved = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -5, "joint_3": 5},
        )

    assert moved.status_code == 200, moved.text
    operations = [command["operation"] for command in controller.commands]
    assert operations.count("HOLD_SET") == 1
    assert operations.count("MOVE") == 2
    assert "MOVE_SET" not in operations


def test_legacy_multi_move_failure_drops_authority_without_stop_latch(
    tmp_path: Path,
) -> None:
    class FailsSecondLegacyMove(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
            if isinstance(capabilities, list):
                identity["capabilities"] = [
                    value for value in capabilities if value != "move_set_v1"
                ]
            return reported

        def move(self, servo_id, goal, speed, acceleration):
            if servo_id == 3:
                raise ControllerCommandError("BUS_ERROR")
            return super().move(servo_id, goal, speed, acceleration)

    controller = FailsSecondLegacyMove(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        failed = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -5, "joint_3": 5},
        )
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        later = client.post(
            "/api/robot/arm/joints/joint_2/target",
            headers=_headers(),
            json={"degrees": -3},
        )

    assert failed.status_code == 409
    operations = [command["operation"] for command in controller.commands]
    assert operations.count("MOVE") == 2
    assert "STOP" not in operations
    assert "RESET" not in operations
    assert any(
        command["operation"] == "HOLD_SET" and command.get("servoIds") == []
        for command in controller.commands
    )
    assert state["stopped"] is False
    assert state["operatorInspectionRequired"] is False
    assert later.status_code == 200, later.text


def test_legacy_first_move_known_refusal_does_not_false_latch_inspection(
    tmp_path: Path,
) -> None:
    class RefusesBeforeDispatch(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            identity = reported.get("identity")
            capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
            if isinstance(capabilities, list):
                identity["capabilities"] = [
                    value for value in capabilities if value != "move_set_v1"
                ]
            return reported

        def move(self, servo_id, goal, speed, acceleration):
            raise ControllerCommandError("SERVO_NOT_FOUND")

    controller = RefusesBeforeDispatch(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -5, "joint_3": 5},
        )
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "SERVO_NOT_FOUND"
    assert not any(
        command["operation"] in {"MOVE", "MOVE_MULTI_TURN", "STOP"}
        for command in controller.commands
    )
    assert state["operatorInspectionRequired"] is False


def test_pi_only_legacy_ambiguity_clear_establishes_stop_before_inspected_reset(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService

    class FirstStopLostController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.lose_next_stop = True

        def stop(self) -> dict[str, object]:
            if self.lose_next_stop:
                self.lose_next_stop = False
                raise ControllerTransportError("STOP delivery lost")
            return super().stop()

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            if self.transport_state().get("motionState") != "stopped":
                raise ControllerCommandError("RESET_PRECONDITION")
            return super().reset(inspected=inspected)

    controller = FirstStopLostController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)

    service._latch_inspection_stop()
    assert controller.transport_state()["motionState"] != "stopped"
    controller.commands.clear()

    service.clear_stop_for_commissioning()

    operations = [command["operation"] for command in controller.commands]
    assert operations[:2] == ["STOP", "RESET"]
    reset = next(command for command in controller.commands if command["operation"] == "RESET")
    assert reset["inspected"] is True
    assert service.state()["operatorInspectionRequired"] is False
    assert controller.transport_state()["motionState"] != "stopped"


def test_calibration_rejects_a_servo_id_already_owned_by_another_joint(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/arm/joints/joint_3/calibrate",
            headers=_headers(),
            json={"servoId": 2},
        )

    assert response.status_code == 409
    assert "already assigned to Shoulder" in response.json()["detail"]


@pytest.mark.parametrize("advertise_move_set", [True, False])
def test_duplicate_logical_servo_mapping_never_takes_torque_or_dispatches(
    tmp_path: Path,
    advertise_move_set: bool,
) -> None:
    class Controller(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            if not advertise_move_set:
                identity = reported.get("identity")
                capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
                if isinstance(capabilities, list):
                    identity["capabilities"] = [
                        value for value in capabilities if value != "move_set_v1"
                    ]
            return reported

    controller = Controller(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    store = JointStore(tmp_path / "state")
    for name in ("joint_2", "joint_3"):
        joint = store.get(name)
        joint.servoId = 2
        joint.rawZero = 2048
        joint.rawMin = 0
        joint.rawMax = 4095
        joint.ratio = 1
    store.save()

    with _client(tmp_path, controller) as client:
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -5, "joint_3": 5},
        )

    assert response.status_code == 409
    assert "same servo ID" in response.json()["detail"]
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_move_set_failure_aborts_without_manual_clear_and_surfaces_safe_evidence(
    tmp_path: Path,
) -> None:
    class AmbiguousOnceController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[
                    _servo(1, 2048),
                    _servo(2, 2048),
                    _servo(3, 2048),
                    _servo(4, 512),
                ],
            )
            self.failed = False

        def move_set(self, moves):
            if not self.failed:
                self.failed = True
                with self._lock:
                    self._before("MOVE_SET", moves=moves)
                raise ControllerCommandError(
                    "MOVE_SET_FAILED",
                    {
                        "phase": "verify",
                        "failedIndex": 1,
                        "stopped": False,
                        "torqueState": "unknown",
                        "motionMayHaveStarted": True,
                        "partialDispatchPossible": True,
                        "dispatchedFamilyCount": 1,
                        "privateDriverDetail": "must-not-leak",
                    },
                )
            return super().move_set(moves)

    controller = AmbiguousOnceController()
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()
        failed = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -5, "joint_3": 5},
        )
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        motion_before_retry = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )
        recovered = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_2": -4, "joint_3": 4},
        )
        motion_after_retry = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )

    assert failed.status_code == 409
    assert failed.json()["detail"] == {
        "code": "MOVE_SET_FAILED",
        "phase": "verify",
        "motionMayHaveStarted": True,
        "partialDispatchPossible": True,
        "stopped": False,
        "torqueState": "unknown",
        "dispatchedFamilyCount": 1,
        "failedIndex": 1,
    }
    assert "must-not-leak" not in failed.text
    assert state["stopped"] is False
    assert state["operatorInspectionRequired"] is False
    assert state["safetyStopReason"] is None
    assert recovered.status_code == 200, recovered.text
    assert motion_after_retry > motion_before_retry
    reset_commands = [
        command for command in controller.commands if command["operation"] == "RESET"
    ]
    assert reset_commands == []


class _RestartLatchedController(ReplayArmController):
    def __init__(self, stop_reason: str, *, unsafe_telemetry: bool = False) -> None:
        super().__init__(
            connected=True,
            servos=[
                _servo(1, 2048),
                _servo(2, 2048),
                _servo(3, 2048),
                _servo(4, 512),
            ],
        )
        self.stop_reason = stop_reason
        self.unsafe_telemetry = unsafe_telemetry
        self.inspection_required = True
        self.reset_attempts: list[bool] = []
        self._stopped = True

    def transport_state(self) -> dict[str, object]:
        state = super().transport_state()
        if self.inspection_required:
            state["operatorInspectionRequired"] = True
            state["safetyStopReason"] = self.stop_reason
            state["motionState"] = "stopped"
        if self.unsafe_telemetry:
            telemetry = state.get("servosTelemetry")
            if isinstance(telemetry, list) and telemetry:
                first = telemetry[0]
                if isinstance(first, dict):
                    first["torqueState"] = "on"
            state["torqueState"] = "on"
        return state

    def reset(self, *, inspected: bool = False) -> dict[str, object]:
        self.reset_attempts.append(inspected)
        if self.inspection_required and not inspected:
            raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
        result = super().reset(inspected=inspected)
        self.inspection_required = False
        return {
            **result,
            "torqueOffConfirmed": True,
            "reset": True,
        }


@pytest.mark.parametrize("stop_reason", ["MOVE_SET_FAILED", "SAFETY_FAULT"])
def test_controller_reported_automatic_stop_recovers_a_fresh_service(
    tmp_path: Path,
    stop_reason: str,
) -> None:
    """Safe automatic faults survive restart but do not require a human clear."""

    controller = _RestartLatchedController(stop_reason)
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not controller.reset_attempts:
            time.sleep(0.02)
        assert controller.reset_attempts == [True]
        _calibrate_for_plans(client)
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        recovered = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_4": 1},
        )
        final_state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["stopped"] is False
    assert state["operatorInspectionRequired"] is False
    assert recovered.status_code == 200, recovered.text
    assert controller.reset_attempts == [True]
    assert any(
        command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )
    assert final_state["operatorInspectionRequired"] is False


@pytest.mark.parametrize("stop_reason", ["MOVE_SET_FAILED", "SAFETY_FAULT"])
def test_controller_reported_automatic_stop_waits_for_safe_telemetry(
    tmp_path: Path,
    stop_reason: str,
) -> None:
    controller = _RestartLatchedController(stop_reason, unsafe_telemetry=True)
    _establish_base_frame(controller)

    with _client(tmp_path, controller) as client:
        time.sleep(0.4)
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert controller.reset_attempts == []
    assert state["stopped"] is True
    assert state["operatorInspectionRequired"] is True


def test_controller_reported_explicit_stop_blocks_until_explicit_clear(
    tmp_path: Path,
) -> None:
    """A Pi restart must not forget a same-boot explicit operator STOP."""

    controller = _RestartLatchedController("EXPLICIT_STOP")
    _establish_base_frame(controller)

    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        time.sleep(0.4)
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        refused = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_4": 1},
        )
        commands_before_clear = list(controller.commands)
        reset_attempts_before_clear = list(controller.reset_attempts)
        cleared = client.post("/api/robot/arm/clear-stop", headers=_headers())
        recovered = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_4": 1},
        )
        final_state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["stopped"] is True
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "EXPLICIT_STOP"
    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "STOPPED"
    assert reset_attempts_before_clear == []
    assert not any(
        command["operation"]
        in {"RESET", "HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in commands_before_clear
    )
    assert cleared.status_code == 200, cleared.text
    assert recovered.status_code == 200, recovered.text
    assert controller.reset_attempts == [True]
    assert final_state["operatorInspectionRequired"] is False


@pytest.mark.parametrize("fail_family_after_reset", [False, True])
def test_stopped_camera_id_change_redeclares_family_before_motion_or_stays_closed(
    tmp_path: Path,
    fail_family_after_reset: bool,
) -> None:
    from robot_gateway.arm_controller import ControllerTransportError

    class StoppedFamilyController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[
                    _servo(1, 2048),
                    _servo(2, 2048),
                    _servo(3, 2048),
                    _servo(7, 512),
                ],
            )
            self.inspection_required = True
            self.fail_family_after_reset = fail_family_after_reset
            self.events: list[str] = []
            self._stopped = True

        def transport_state(self) -> dict[str, object]:
            state = super().transport_state()
            if self.inspection_required:
                state["operatorInspectionRequired"] = True
                state["safetyStopReason"] = "EXPLICIT_STOP"
                state["motionState"] = "stopped"
            return state

        def declare_family(self, servo_id: int, family: str) -> None:
            self.events.append(f"family:{servo_id}:{family}")
            if self.inspection_required:
                raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
            if self.fail_family_after_reset:
                raise ControllerTransportError("family replay failed")
            super().declare_family(servo_id, family)

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            if self.inspection_required and not inspected:
                raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
            result = super().reset(inspected=inspected)
            self.inspection_required = False
            return result

        def move(self, servo_id, goal, speed, acceleration):
            self.events.append(f"move:{servo_id}")
            return super().move(servo_id, goal, speed, acceleration)

    controller = StoppedFamilyController()
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        changed = client.post(
            "/api/robot/arm/joints/joint_4/calibrate",
            headers=_headers(),
            json={
                "servoId": 7,
                "rawZero": 512,
                "rawMin": 0,
                "rawMax": 1023,
                "ratio": 1,
            },
        )
        controller.events.clear()
        cleared = client.post("/api/robot/arm/clear-stop", headers=_headers())
        moved = client.post(
            "/api/robot/arm/joints/joint_4/target",
            headers=_headers(),
            json={"degrees": 1},
        )
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert changed.status_code == 200, changed.text
    camera = next(joint for joint in changed.json()["joints"] if joint["id"] == "joint_4")
    assert camera["servoId"] == 7
    if fail_family_after_reset:
        assert cleared.status_code == 502, cleared.text
        assert moved.status_code == 409, moved.text
        assert not any(event.startswith("move:") for event in controller.events)
        assert state["stopped"] is True
    else:
        assert cleared.status_code == 200, cleared.text
        assert moved.status_code == 200, moved.text
        assert controller.events.index("family:7:SCS") < controller.events.index("move:7")


def test_plan_preview_resolves_direct_targets_without_torque_or_motion(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller.commands.clear()

        response = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_1": 10, "joint_4": 5}},
        )

    assert response.status_code == 200, response.text
    body = response.json()
    assert body["planId"].startswith("armplan_")
    assert body["planDigest"].startswith("sha256:")
    assert body["expiresInMs"] == 30_000
    assert body["measuredPose"]["joint_1"] == pytest.approx(0, abs=0.1)
    assert body["resolvedPose"]["joint_1"] == pytest.approx(10, abs=0.1)
    assert body["resolvedPose"]["joint_4"] == pytest.approx(5, abs=0.4)
    assert body["scene"]["measured"]["points"]["tipCamera"]
    operations = [entry["operation"] for entry in controller.commands]
    assert "MOVE" not in operations
    assert "HOLD_SET" not in operations


def test_planar_tip_ik_uses_the_dashboard_servo_offsets(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        # Start at the same safe horizontal pose so this test isolates the IK
        # frame offsets rather than asking independently timed joints to sweep
        # through the unsafe shoulder-first intermediate configuration.
        controller._servos[2]["rawPosition"] = 1024
        controller._servos[3]["rawPosition"] = 3072
        response = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={
                "targets": {},
                "tip": {"radialMm": 400, "heightMm": 60},
                "elbowPreference": "nearest",
            },
        )

    assert response.status_code == 200, response.text
    body = response.json()
    # Model shoulder/elbow are both zero for a horizontal straight arm. Servo
    # degrees therefore use the same +90/-90 offsets as SimpleArm.
    assert body["resolvedPose"]["joint_2"] == pytest.approx(-90, abs=0.2)
    assert body["resolvedPose"]["joint_3"] == pytest.approx(90, abs=0.2)
    assert body["ik"]["selectedBranch"] in {"up", "down"}
    assert body["ik"]["projected"] is False
    assert body["lowestClearanceMm"] == pytest.approx(20, abs=0.3)


def test_planar_tip_ik_rejects_joint_limit_clamping_that_would_move_the_endpoint(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        response = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={
                "targets": {},
                "tip": {"radialMm": -70, "heightMm": 75},
                "elbowPreference": "nearest",
            },
        )

    assert response.status_code == 409
    assert "cannot reach" in response.json()["detail"]


def test_apply_consumes_the_exact_prepared_pose_once(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 10, "joint_3": 20}},
        ).json()
        controller.commands.clear()
        payload = {
            "planId": preview["planId"],
            "planDigest": preview["planDigest"],
        }

        executed = client.post(
            "/api/robot/arm/plans/execute", headers=_headers(), json=payload
        )
        replay = client.post(
            "/api/robot/arm/plans/execute", headers=_headers(), json=payload
        )

    assert executed.status_code == 200, executed.text
    body = executed.json()
    assert body["executed"] is True
    assert body["planId"] == preview["planId"]
    assert body["resolvedPose"] == preview["resolvedPose"]
    assert [entry["servoId"] for entry in body["moved"]] == [2, 3]
    assert replay.status_code == 409
    move_sets = [
        entry for entry in controller.commands if entry["operation"] == "MOVE_SET"
    ]
    assert len(move_sets) == 1
    assert [entry["servoId"] for entry in move_sets[0]["moves"]] == [2, 3]


def test_apply_rejects_start_pose_drift_and_consumes_the_plan(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 25}},
        ).json()
        # Move the measured encoder by about 18 degrees without using the plan.
        controller._servos[2]["rawPosition"] = 2253
        payload = {
            "planId": preview["planId"],
            "planDigest": preview["planDigest"],
        }
        refused = client.post(
            "/api/robot/arm/plans/execute", headers=_headers(), json=payload
        )
        replay = client.post(
            "/api/robot/arm/plans/execute", headers=_headers(), json=payload
        )

    assert refused.status_code == 409
    assert "moved after preview" in refused.json()["detail"]
    assert replay.status_code == 409
    assert not [entry for entry in controller.commands if entry["operation"] == "MOVE"]


def test_apply_rejects_changed_floor_protection_and_controller_boot(tmp_path: Path) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        floor_plan = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 5}},
        ).json()
        client.post(
            "/api/robot/arm/floor-guard",
            headers=_headers(),
            json={"enabled": False},
        )
        floor_refusal = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={
                "planId": floor_plan["planId"],
                "planDigest": floor_plan["planDigest"],
            },
        )

        client.post(
            "/api/robot/arm/floor-guard",
            headers=_headers(),
            json={"enabled": True},
        )
        boot_plan = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 5}},
        ).json()
        controller.simulate_reboot()
        boot_refusal = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={
                "planId": boot_plan["planId"],
                "planDigest": boot_plan["planDigest"],
            },
        )

    assert floor_refusal.status_code == 409
    assert "floor protection changed" in floor_refusal.json()["detail"]
    assert boot_refusal.status_code == 409
    assert "boot changed" in boot_refusal.json()["detail"]


def test_plan_rejects_independent_joint_sweep_and_direct_move_clamps_it(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import FLOOR_MM, swept_lowest_point_mm

    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        # Both endpoints clear the 40 mm plane, but with Shoulder already at
        # -90 the Elbow passes through 0 degrees and puts the tip at -160 mm.
        controller._servos[2]["rawPosition"] = 1024
        controller._servos[3]["rawPosition"] = 1024
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_3": 85}},
        )
        direct = client.post(
            "/api/robot/arm/target",
            headers=_headers(),
            json={"joint_3": 85},
        )

    assert swept_lowest_point_mm(-90, -90, -90, 85) < FLOOR_MM
    assert preview.status_code == 409
    assert "independently timed" in preview.json()["detail"]
    assert direct.status_code == 200, direct.text
    sent = direct.json()["moved"][0]["degrees"]
    assert sent < 85
    assert swept_lowest_point_mm(-90, -90, -90, sent) >= FLOOR_MM - 1e-6


def test_plan_requires_a_new_status_and_fresh_required_servo_packets(
    tmp_path: Path,
) -> None:
    status_calls = 0

    class CountingStatusController(ReplayArmController):
        def status(self) -> dict[str, object]:
            nonlocal status_calls
            status_calls += 1
            return self.transport_state()

    controller = CountingStatusController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        calls_before_preview = status_calls
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 5}},
        ).json()
        controller._servos[2]["packetAgeMs"] = 251
        calls_after_preview = status_calls
        refused = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={"planId": preview["planId"], "planDigest": preview["planDigest"]},
        )

    assert calls_after_preview == calls_before_preview + 1
    assert status_calls == calls_after_preview + 1
    assert refused.status_code == 409
    assert "Fresh measured telemetry" in refused.json()["detail"]
    assert not [entry for entry in controller.commands if entry["operation"] == "MOVE"]


def test_base_plan_requires_fresh_odometer_proof_at_preview_and_execute(
    tmp_path: Path,
) -> None:
    stale_odometer = True

    class OdometerAgeController(ReplayArmController):
        def odometer_read(self, servo_id: int) -> dict[str, object]:
            result = super().odometer_read(servo_id)
            if stale_odometer:
                result["sampleAgeMs"] = 251
            return result

    controller = OdometerAgeController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        stale_preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_1": 10}},
        )
        stale_odometer = False
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_1": 10}},
        ).json()
        stale_odometer = True
        stale_execute = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={"planId": preview["planId"], "planDigest": preview["planDigest"]},
        )

    assert stale_preview.status_code == 409
    assert stale_execute.status_code == 409
    assert not [
        entry
        for entry in controller.commands
        if entry["operation"] in {"MOVE", "MOVE_MULTI_TURN"}
    ]


def test_invalid_planning_odometer_proof_clears_cached_base_position(
    tmp_path: Path,
) -> None:
    """An explicit invalid ODO result must invalidate every Pi-side Base cache."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    invalid = False

    class InvalidOdometerController(ReplayArmController):
        def odometer_read(self, servo_id: int) -> dict[str, object]:
            result = super().odometer_read(servo_id)
            if servo_id == 1 and invalid:
                result.update(
                    tracking=True,
                    valid=False,
                    stepMode=False,
                    multiTurnPosition=None,
                    sampleAgeMs=0,
                    resyncNeeded=True,
                )
            return result

    controller = InvalidOdometerController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "state"))
    service.close()
    service._service.join(timeout=1.0)
    base = service._store.get("joint_1")
    base.rawZero, base.rawMin, base.rawMax, base.ratio = 2048, -7000, 12_000, 4
    service._refresh_multi_turn()
    assert service._multi_turn_raw[1] == 2048
    assert 1 in service._multi_turn_armed

    invalid = True
    with pytest.raises(HTTPException):
        service._planning_snapshot({"joint_1"})

    assert 1 not in service._multi_turn_raw
    assert 1 not in service._multi_turn_armed


def test_plan_requires_fresh_companion_but_ignores_unrelated_stale_camera(
    tmp_path: Path,
) -> None:
    stale_companion = _planning_controller()
    companion_path = tmp_path / "companion"
    companion_path.mkdir()
    with _client(companion_path, stale_companion) as client:
        _calibrate_for_plans(client)
        stale_companion._servos[3]["packetAgeMs"] = 251
        refused = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 5}},
        )

    unrelated_camera = _planning_controller()
    camera_path = tmp_path / "camera"
    camera_path.mkdir()
    with _client(camera_path, unrelated_camera) as client:
        _calibrate_for_plans(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_2": 5}},
        ).json()
        unrelated_camera._servos[4]["packetAgeMs"] = 251
        applied = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={"planId": preview["planId"], "planDigest": preview["planDigest"]},
        )

    assert refused.status_code == 409
    assert "Elbow" in refused.json()["detail"]
    assert applied.status_code == 200, applied.text


def test_apply_rejects_small_allowed_drift_that_makes_the_sweep_unsafe(
    tmp_path: Path,
) -> None:
    controller = _planning_controller()
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        controller._servos[2]["rawPosition"] = 1024  # shoulder -90 degrees
        controller._servos[3]["rawPosition"] = 1024  # elbow -90 degrees
        preview_response = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_3": -85}},
        )
        assert preview_response.status_code == 200, preview_response.text
        preview = preview_response.json()
        # About +1.5 degrees: inside the 2-degree drift tolerance, but enough
        # for the independently timed rectangle to cross below 40 mm.
        controller._servos[2]["rawPosition"] = 1041
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/plans/execute",
            headers=_headers(),
            json={"planId": preview["planId"], "planDigest": preview["planDigest"]},
        )

    assert refused.status_code == 409
    assert "sweep is no longer floor-safe" in refused.json()["detail"]
    assert not [entry for entry in controller.commands if entry["operation"] == "MOVE"]


def test_base_plan_rejects_missing_native_odometer_capability(tmp_path: Path) -> None:
    hide_capability = False

    class MissingCapabilityController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            state = super().transport_state()
            if hide_capability:
                identity = state.get("identity")
                if isinstance(identity, dict):
                    identity["capabilities"] = [
                        capability
                        for capability in identity.get("capabilities", [])
                        if capability != "multi_turn_absolute_v1"
                    ]
            return state

        def status(self) -> dict[str, object]:
            return self.transport_state()

    controller = MissingCapabilityController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    _establish_base_frame(controller)
    hide_capability = True
    with _client(tmp_path, controller) as client:
        _calibrate_for_plans(client)
        refused = client.post(
            "/api/robot/arm/plans/preview",
            headers=_headers(),
            json={"targets": {"joint_1": 10}},
        )

    assert refused.status_code == 409
    assert "Base odometer frame" in refused.json()["detail"]
    assert not [
        entry
        for entry in controller.commands
        if entry["operation"] in {"MOVE", "MOVE_MULTI_TURN", "HOLD_SET"}
    ]


def test_plan_paths_refuse_outstanding_goals_and_chase_uses_motion_lock(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import (
        ArmService,
        JointStore,
        PlanExecuteRequest,
        PlanPreviewRequest,
        PlanTargets,
    )

    controller = _planning_controller()
    store = JointStore(tmp_path / "direct-service")
    for name, zero, high in (
        ("joint_1", 2048, 4095),
        ("joint_2", 2048, 4095),
        ("joint_3", 2048, 4095),
        ("joint_4", 512, 1023),
    ):
        joint = store.get(name)
        joint.rawZero = zero
        joint.rawMin = 0
        joint.rawMax = high
    service = ArmService(controller, store)
    try:
        with service._lock:
            service._goals["joint_2"] = (2100, 1, time.monotonic() + 5)
        with pytest.raises(HTTPException, match="earlier move is still settling"):
            service.preview_plan(PlanPreviewRequest(targets=PlanTargets(joint_4=5)))

        with service._lock:
            service._goals.clear()
        preview = service.preview_plan(PlanPreviewRequest(targets=PlanTargets(joint_4=5)))
        with service._lock:
            service._goals["joint_2"] = (2100, 1, time.monotonic() + 5)
        with pytest.raises(HTTPException, match="earlier move is still settling"):
            service.execute_plan(
                PlanExecuteRequest(
                    planId=str(preview["planId"]),
                    planDigest=str(preview["planDigest"]),
                )
            )

        done = threading.Event()
        service._motion_lock.acquire()
        worker = threading.Thread(target=lambda: (service._chase_goals(), done.set()))
        worker.start()
        assert not done.wait(0.05)
        service._motion_lock.release()
        assert done.wait(1)
        worker.join(timeout=1)
    finally:
        service.close()


def test_execute_sends_digest_bound_raw_goal_without_second_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from robot_gateway.simple_arm_api import (
        ArmService,
        JointState,
        JointStore,
        PlanExecuteRequest,
        PlanPreviewRequest,
        PlanTargets,
    )

    controller = _planning_controller()
    store = JointStore(tmp_path / "exact-service")
    for name, zero, high in (
        ("joint_1", 2048, 4095),
        ("joint_2", 2048, 4095),
        ("joint_3", 2048, 4095),
        ("joint_4", 512, 1023),
    ):
        joint = store.get(name)
        joint.rawZero = zero
        joint.rawMin = 0
        joint.rawMax = high
    service = ArmService(controller, store)
    try:
        preview = service.preview_plan(
            PlanPreviewRequest(targets=PlanTargets(joint_2=5))
        )
        stored = service._plans[str(preview["planId"])]
        expected_goal = stored["moveGoals"]["joint_2"]

        def no_second_guard(_: dict[str, float]) -> dict[str, float]:
            raise AssertionError("execute re-ran floor resolution")

        def no_second_quantization(*_: object, **__: object) -> int:
            raise AssertionError("execute re-ran goal quantization")

        monkeypatch.setattr(service, "_guard_floor", no_second_guard)
        monkeypatch.setattr(JointState, "goal_for", no_second_quantization)
        controller.commands.clear()
        result = service.execute_plan(
            PlanExecuteRequest(
                planId=str(preview["planId"]),
                planDigest=str(preview["planDigest"]),
            )
        )
    finally:
        service.close()

    moves = [entry for entry in controller.commands if entry["operation"] == "MOVE"]
    assert [entry["goal"] for entry in moves] == [expected_goal]
    assert result["moved"][0]["degrees"] == pytest.approx(
        preview["resolvedPose"]["joint_2"], abs=1e-6
    )


def test_floor_guard_refuses_direct_move_without_fresh_planar_telemetry() -> None:
    from robot_gateway.simple_arm_api import ArmService

    class _Service:
        _floor_guard = True

        def __init__(self) -> None:
            self._lock = threading.RLock()

        def _measured_degrees(self, _: str) -> None:
            return None

    with pytest.raises(HTTPException, match="Fresh Shoulder and Elbow telemetry"):
        ArmService._guard_floor(_Service(), {"joint_2": -10})


def test_a_limit_can_be_typed_before_any_zero_exists(tmp_path: Path) -> None:
    """The zero is derived from the limits, so demanding one first made the very
    first typed limit impossible."""

    with _client(tmp_path, _connected()) as client:
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"minDegrees": -30},
        )

    assert body.status_code == 200
    base = body.json()["joints"][0]
    # Measured from wherever the joint was standing, raw 2048.
    assert base["rawZero"] == 2048
    assert base["rawMin"] == 2048 - round(30 * 4096 / 360)


def test_a_typed_limit_is_stored_as_typed_however_it_is_geared(tmp_path: Path) -> None:
    """The limit the operator typed is the limit that gets stored.

    A geared joint may need more than one motor turn to reach its declared
    limits. Whether the gearing delivers that is visible on the arm, not from
    here, so clamping it was this module overruling the person holding it.
    """

    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "ratio": 8},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"minDegrees": -180},
        ).json()

    # 180 deg at 8:1 is 16384 ticks below the zero — four motor turns, kept whole.
    assert body["joints"][0]["rawMin"] == 2048 - 16384
    assert round(body["joints"][0]["minDegrees"]) == -180
    assert "note" not in body


def test_zero_can_be_centred_between_the_limits(tmp_path: Path) -> None:
    """Zero belongs halfway between min and max, so travel is symmetric."""

    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawMin": 1000, "rawMax": 3000, "ratio": 1},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"zeroFromLimits": True},
        ).json()

    base = body["joints"][0]
    assert base["rawZero"] == 2000
    assert base["minDegrees"] == -base["maxDegrees"]


def test_centring_the_zero_needs_both_limits(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_2/calibrate", headers=_headers(), json={"rawMin": 1000}
        )
        refused = client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"zeroFromLimits": True},
        )

    assert refused.status_code == 409


def test_a_target_past_the_limit_moves_to_the_limit(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1024, "rawMax": 3072, "ratio": 1},
        )
        client.post(
            "/api/robot/arm/joints/joint_3/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1900, "rawMax": 2200, "ratio": 1},
        )
        body = client.post(
            "/api/robot/arm/joints/joint_3/target", headers=_headers(), json={"degrees": 180}
        ).json()

    assert body["moved"][0]["goal"] == 2200


def test_assigning_an_id_carries_the_joint_that_pointed_at_it(tmp_path: Path) -> None:
    """Renaming a servo must not silently unbind the joint that drives it.

    The store names servos by id, so leaving a joint on the old number would
    leave it reading offline with a perfectly good calibration and nothing to
    say why.
    """

    with _client(tmp_path, _connected()) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "ratio": 2},
        )
        body = client.post(
            "/api/robot/arm/servo-id", headers=_headers(), json={"oldId": 1, "newId": 5}
        )

    assert body.status_code == 200
    payload = body.json()
    assert payload["assigned"] == {"oldId": 1, "newId": 5, "verified": True}
    base = payload["joints"][0]
    assert base["servoId"] == 5
    # The calibration is untouched: it describes the joint, not the servo.
    assert base["rawZero"] == 2048
    assert base["ratio"] == 2


def test_assigning_an_id_survives_a_restart(tmp_path: Path) -> None:
    controller = _connected()
    with _client(tmp_path, controller) as client:
        client.post(
            "/api/robot/arm/joints/joint_2/calibrate", headers=_headers(), json={"servoId": 2}
        )
        client.post(
            "/api/robot/arm/servo-id", headers=_headers(), json={"oldId": 2, "newId": 7}
        )
    with _client(tmp_path, controller) as client:
        joints = client.get("/api/robot/arm/state", headers=_headers()).json()["joints"]

    assert joints[1]["servoId"] == 7


def test_base_id_mapping_replaces_the_native_multi_turn_scan_declaration(
    tmp_path: Path,
) -> None:
    class NativeIdRecordingController(ReplayArmController):
        def __init__(self, **kwargs) -> None:
            super().__init__(**kwargs)
            self.native_id_sets: list[list[int]] = []

        def declare_native_multi_turn_servos(self, servo_ids: list[int]) -> None:
            self.native_id_sets.append(list(servo_ids))
            super().declare_native_multi_turn_servos(servo_ids)

    controller = NativeIdRecordingController(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
    )
    _establish_base_frame(controller)

    with _client(tmp_path, controller) as client:
        assert controller.native_id_sets[-1] == [1]
        changed = client.post(
            "/api/robot/arm/servo-id",
            headers=_headers(),
            json={"oldId": 1, "newId": 5},
        )

    assert changed.status_code == 200, changed.text
    assert controller.native_id_sets[-1] == [5]
    assert controller._declared_native_multi_turn_servos == {5}


def test_an_id_that_is_already_taken_is_refused_without_touching_the_store(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        body = client.post(
            "/api/robot/arm/servo-id", headers=_headers(), json={"oldId": 1, "newId": 2}
        )
        joints = client.get("/api/robot/arm/state", headers=_headers()).json()["joints"]

    assert body.status_code == 409
    assert joints[0]["servoId"] == 1


def test_assign_id_rejects_an_offline_destination_owned_by_another_logical_joint(
    tmp_path: Path,
) -> None:
    class FamilyCountingController(ReplayArmController):
        def __init__(self, **kwargs) -> None:
            super().__init__(**kwargs)
            self.family_calls: list[tuple[int, str]] = []

        def declare_family(self, servo_id: int, family: str) -> None:
            self.family_calls.append((servo_id, family))
            super().declare_family(servo_id, family)

    controller = FamilyCountingController(
        connected=True,
        # ID 4 is absent physically but still belongs to logical Camera.
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        before = client.get("/api/robot/arm/state", headers=_headers()).json()[
            "joints"
        ]
        controller.commands.clear()
        controller.family_calls.clear()
        refused = client.post(
            "/api/robot/arm/servo-id",
            headers=_headers(),
            json={"oldId": 1, "newId": 4},
        )
        after = client.get("/api/robot/arm/state", headers=_headers()).json()[
            "joints"
        ]

    assert refused.status_code == 409, refused.text
    assert "already assigned to Camera" in refused.json()["detail"]
    assert [joint["servoId"] for joint in after] == [
        joint["servoId"] for joint in before
    ]
    assert controller.family_calls == []
    assert not any(
        command["operation"]
        in {"ASSIGN_ID", "HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in controller.commands
    )


def test_renaming_a_servo_to_its_own_id_is_refused(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        body = client.post(
            "/api/robot/arm/servo-id", headers=_headers(), json={"oldId": 1, "newId": 1}
        )

    assert body.status_code == 400


def test_registers_are_read_only_on_the_bench(tmp_path: Path) -> None:
    """Diagnostics may inspect registers but cannot bypass guarded operations."""

    with _client(tmp_path, _connected()) as client:
        written = client.post(
            "/api/robot/arm/registers",
            headers=_headers(),
            json={"servoId": 1, "address": 9, "values": [0, 0, 0, 0]},
        )
        read = client.post(
            "/api/robot/arm/registers",
            headers=_headers(),
            json={"servoId": 1, "address": 9, "length": 4},
        )

    assert written.status_code == 409
    assert written.json()["detail"] == {"code": "REGISTER_WRITE_DISABLED"}
    assert read.json()["values"] == [0, 0, 0, 0]


def test_a_register_request_must_say_whether_it_reads_or_writes(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        neither = client.post(
            "/api/robot/arm/registers", headers=_headers(), json={"servoId": 1, "address": 9}
        )
        both = client.post(
            "/api/robot/arm/registers",
            headers=_headers(),
            json={"servoId": 1, "address": 9, "length": 2, "values": [0, 0]},
        )

    assert neither.status_code == 400
    assert both.status_code == 400


def test_the_camera_joint_is_calibrated_and_driven_like_any_other(tmp_path: Path) -> None:
    """It carries the camera, so it is a servo with four numbers -- nothing more.

    What it is not is part of the arm: it has no link length and contributes
    nothing to where the tool ends up, which is why the twin never sees it.
    """

    controller = ReplayArmController.connected(
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)]
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()
        assert [joint["id"] for joint in state["joints"]] == [
            "joint_1", "joint_2", "joint_3", "joint_4",
        ]
        camera = state["joints"][3]
        assert (camera["name"], camera["servoId"]) == ("Camera", 4)

        client.post(
            "/api/robot/arm/joints/joint_4/calibrate",
            headers=_headers(),
            json={"rawZero": 512, "rawMin": 0, "rawMax": 1023, "ratio": 0.3},
        )
        moved = client.post(
            "/api/robot/arm/joints/joint_4/target", headers=_headers(), json={"degrees": 30}
        )

    assert moved.status_code == 200
    assert moved.json()["moved"][0]["servoId"] == 4


def test_all_four_joints_can_be_held_at_once(tmp_path: Path) -> None:
    """arm-hat-2.0.0 raised MAX_HOLD_SERVOS to 4, so the camera no longer has to
    displace an arm joint to be energised."""

    controller = ReplayArmController.connected(
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)]
    )
    _establish_base_frame(controller)
    with _client(tmp_path, controller) as client:
        for name, servo in (("joint_1", 1), ("joint_2", 2), ("joint_3", 3), ("joint_4", 4)):
            client.post(
                f"/api/robot/arm/joints/{name}/calibrate",
                headers=_headers(),
                json={
                    "rawZero": 512 if servo == 4 else 2048,
                    "rawMin": 0,
                    "rawMax": 1023 if servo == 4 else 4095,
                },
            )
        client.post("/api/robot/arm/torque", headers=_headers(), json={"hold": [1, 2, 3]})
        client.post("/api/robot/arm/target", headers=_headers(), json={"joint_4": 5})
        held = client.get("/api/robot/arm/state", headers=_headers()).json()["held"]

    assert sorted(held) == [1, 2, 3, 4]


def test_a_trimmed_hold_set_keeps_the_joint_being_driven() -> None:
    """Independent of MAX_HELD's current value.

    Whenever a hold set has to be trimmed, the servos just asked for come first.
    Truncating a sorted union instead would evict whichever id sorted lowest --
    including the joint the operator is moving at that moment.
    """

    from robot_gateway.simple_arm_api import MAX_HELD

    previous = [10, 11, 12, 13, 14][:MAX_HELD]
    requested = [99]
    wanted = list(dict.fromkeys([*requested, *previous]))[:MAX_HELD]

    assert wanted[0] == 99
    assert len(wanted) == MAX_HELD
    assert previous[-1] not in wanted  # the oldest is what gets dropped


def test_a_late_controller_boot_stop_still_requires_explicit_clear(tmp_path: Path) -> None:
    """Offline startup must not hide or auto-reset a later stopped HAT."""

    controller = ReplayArmController.connected(servos=[_servo(1, 2048)])
    controller.stop()
    offline = {"connection": "offline"}
    calls: list[str] = []

    original = controller.transport_state

    def late(*args: object, **kwargs: object) -> dict[str, object]:
        calls.append("read")
        if len(calls) <= 3:
            return dict(offline)
        return original(*args, **kwargs)

    controller.transport_state = late  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        # An offline read reports stopped=False (there is no motionState at all),
        # so waiting on that field would exit before the controller ever arrives.
        # Wait on the controller actually being consulted instead.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and len(calls) <= 8:
            time.sleep(0.05)
        stopped = client.get("/api/robot/arm/state", headers=_headers()).json()["stopped"]
        resets_before_clear = [
            command
            for command in controller.commands
            if command["operation"] == "RESET"
        ]
        cleared = client.post("/api/robot/arm/clear-stop", headers=_headers())
        stopped_after_clear = client.get(
            "/api/robot/arm/state", headers=_headers()
        ).json()["stopped"]

    assert len(calls) > 3, "the offline reads should not have consumed the one-shot clear"
    assert stopped is True
    assert resets_before_clear == []
    assert cleared.status_code == 200, cleared.text
    assert stopped_after_clear is False
    reset = next(
        command for command in controller.commands if command["operation"] == "RESET"
    )
    assert reset["inspected"] is True


def test_a_scan_returns_its_own_evidence_not_just_the_state(tmp_path: Path) -> None:
    """The scan payload carries the collision verdict; state() alone does not."""

    with _client(tmp_path, _connected()) as client:
        body = client.post("/api/robot/arm/scan", headers=_headers()).json()

    assert "joints" in body
    assert "scan" in body, "the controller's own scan evidence was being discarded"


def test_a_scan_returns_the_post_scan_inventory_in_the_same_response(tmp_path: Path) -> None:
    """The button response must not briefly repaint the pre-scan offline state."""

    controller = ReplayArmController.connected(servos=[])

    def discover(minimum_id: int, maximum_id: int) -> dict[str, object]:
        assert (minimum_id, maximum_id) == (0, 10)
        with controller._lock:  # type: ignore[attr-defined]
            controller._servos[1] = _servo(1, 2048)  # type: ignore[attr-defined]
        return {
            "foundIds": [1],
            "completeRange": {"minId": minimum_id, "maxId": maximum_id},
            "collisionSuspected": False,
        }

    controller.scan = discover  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        body = client.post("/api/robot/arm/scan", headers=_headers()).json()

    base = next(joint for joint in body["joints"] if joint["id"] == "joint_1")
    assert base["online"] is True
    assert body["scan"]["foundIds"] == [1]


def test_last_scan_presence_evidence_survives_normal_state_polls(tmp_path: Path) -> None:
    """The next 150 ms UI poll must not erase the Scan button's evidence."""

    with _client(tmp_path, _connected()) as client:
        scanned = client.post("/api/robot/arm/scan", headers=_headers()).json()
        polled = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert scanned["scan"]["foundIds"] == [1, 2, 3]
    assert polled["lastScan"] == {
        "minId": 0,
        "maxId": 10,
        "foundIds": [1, 2, 3],
        "collisionSuspected": False,
        "collisionId": None,
    }
    assert "scan" not in polled


def test_ping_presence_is_preserved_without_inventing_base_telemetry(tmp_path: Path) -> None:
    """PING proves presence; only fresh aggregate telemetry proves a pose."""

    controller = ReplayArmController.connected(
        servos=[_servo(2, 2048), _servo(3, 2048), _servo(4, 512)]
    )
    original = controller.transport_state

    def with_scan(*args: object, **kwargs: object) -> dict[str, object]:
        reported = dict(original(*args, **kwargs))
        reported["lastScan"] = {
            "minId": 0,
            "maxId": 10,
            "foundIds": [1, 2, 3, 4],
            "pingFoundIds": [1, 2, 3, 4],
            "telemetryUnavailableIds": [1],
            "telemetryRecoveredIds": [],
            "collisionSuspected": False,
            "collisionId": None,
            "busJammed": False,
            "busNoise": {"bytes": 999_999, "sample": "do-not-forward"},
            "unboundedControllerField": "do-not-forward",
        }
        return reported

    controller.transport_state = with_scan  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    base = next(joint for joint in state["joints"] if joint["id"] == "joint_1")
    assert base["online"] is False
    assert base["rawPosition"] is None
    assert base["positionTrusted"] is False
    assert state["lastScan"] == {
        "minId": 0,
        "maxId": 10,
        "foundIds": [1, 2, 3, 4],
        "pingFoundIds": [1, 2, 3, 4],
        "telemetryUnavailableIds": [1],
        "telemetryRecoveredIds": [],
        "collisionSuspected": False,
        "collisionId": None,
    }


def test_ping_backed_base_telemetry_recovery_survives_the_next_state_poll(
    tmp_path: Path,
) -> None:
    """Recovered means fresh after the first STATUS, not absent from PING."""

    controller = _connected()
    original = controller.transport_state

    def with_recovered_scan(*args: object, **kwargs: object) -> dict[str, object]:
        reported = dict(original(*args, **kwargs))
        reported["lastScan"] = {
            "minId": 0,
            "maxId": 10,
            "foundIds": [1, 2, 3],
            "pingFoundIds": [1, 2, 3],
            "telemetryUnavailableIds": [],
            "telemetryRecoveredIds": [1],
            "collisionSuspected": False,
            "collisionId": None,
        }
        return reported

    controller.transport_state = with_recovered_scan  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["lastScan"] == {
        "minId": 0,
        "maxId": 10,
        "foundIds": [1, 2, 3],
        "pingFoundIds": [1, 2, 3],
        "telemetryUnavailableIds": [],
        "telemetryRecoveredIds": [1],
        "collisionSuspected": False,
        "collisionId": None,
    }


def test_a_suspected_id_collision_reaches_the_panel(tmp_path: Path) -> None:
    """Two servos on one id looks exactly like an empty bus from the outside.

    The controller can tell the difference -- a corrupt ping reply rather than
    silence -- so the panel must be told, or the operator sees a blank list and
    no reason for it.
    """

    controller = _connected()
    original = controller.transport_state

    def with_collision(*args: object, **kwargs: object) -> dict[str, object]:
        reported = dict(original(*args, **kwargs))
        reported["lastScan"] = {
            "minId": 0, "maxId": 10, "foundIds": [],
            "collisionSuspected": True, "collisionId": 1, "busJammed": True,
        }
        return reported

    controller.transport_state = with_collision  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["collisionSuspected"] is True
    assert state["collisionId"] == 1


def test_no_collision_reported_when_the_bus_is_merely_empty(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["collisionSuspected"] is False
    assert state["collisionId"] is None


def test_a_faulted_bus_is_explained_in_words_the_panel_can_print(tmp_path: Path) -> None:
    """Keyed on the controller's own vocabulary, not the translated one.

    `physical_arm_api` renames "faulted" to "degraded" for its own status shape.
    Keying this table on the translated word made it dead code against real
    hardware, which reports "faulted".
    """

    controller = _connected()
    original = controller.transport_state

    def faulted(*args: object, **kwargs: object) -> dict[str, object]:
        reported = dict(original(*args, **kwargs))
        reported["bus"] = "faulted"
        return reported

    controller.transport_state = faulted  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["busTrouble"] is not None
    assert "telemetry" in state["busTrouble"]
    assert "Run Scan" in state["busTrouble"]
    assert "Do not change servo IDs" in state["busTrouble"]
    assert "two servos sharing" not in state["busTrouble"]


def test_a_healthy_bus_has_nothing_to_explain(tmp_path: Path) -> None:
    with _client(tmp_path, _connected()) as client:
        state = client.get("/api/robot/arm/state", headers=_headers()).json()

    assert state["busTrouble"] is None


def test_the_camera_joint_uses_the_sc09_encoder_and_not_the_st3215_one() -> None:
    """An SC09 is 1024 counts over 300 degrees, not 4096 over 360.

    Sharing the ST3215 geometry produces goals above 1023, which the controller
    rejects outright as OUT_OF_RANGE, and reports angles scaled by 3.3x. Neither
    failure announces itself: the joint just refuses to move, or moves wrong.
    """

    camera = JointState(4, scs=True)
    elbow = JointState(3)

    assert camera.raw_max == 1023
    assert elbow.raw_max == 4095
    # 1024 vs 4096 counts over the same full revolution -- the camera turns
    # four times further per count. Measured on the arm: 530 counts reads as
    # ~180 degrees, which is 360 (186) and not 300 (155).
    assert round(camera.ticks_per_degree, 4) == round(1024 / 360.0, 4)
    assert round(elbow.ticks_per_degree, 4) == round(4096 / 360.0, 4)

    # An uncalibrated joint offers its own full travel, not the ST3215's.
    assert camera.raw_bounds() == (0, 1023)

    # A goal past the encoder's end is pulled back onto it rather than being
    # sent to a controller that would refuse the whole move.
    camera.rawZero = 512
    assert camera.goal_for(360.0) == 1023
    assert camera.goal_for(-360.0) == 0


def test_here_is_resolved_from_a_reading_taken_when_the_request_lands(tmp_path: Path) -> None:
    """"Here" must not mean "wherever the browser last saw the joint".

    The panel polls, and the Pi refreshes telemetry on its own cadence, so the
    browser's rawPosition trails the servo by up to half a second. Sending that
    number stored a position the joint had already left -- invisible on a geared
    arm joint moved slowly, tens of degrees on the camera servo spun by hand.
    """

    controller = _connected()
    with _client(tmp_path, controller) as client:
        moved = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"here": ["rawMin"]},
        )
        assert moved.status_code == 200
        joint = next(j for j in moved.json()["joints"] if j["id"] == "joint_1")
        # Taken from the controller's live telemetry, not from anything the
        # caller supplied -- the request carried no position at all.
        assert joint["rawMin"] == joint["rawPosition"]


def test_here_refuses_rather_than_storing_a_guess_when_the_joint_is_silent(tmp_path: Path) -> None:
    controller = _connected()
    with _client(tmp_path, controller) as client:
        client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"servoId": 99},
        )
        refused = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"here": ["rawMin"]},
        )

    assert refused.status_code == 409
    assert "not reporting a position" in refused.json()["detail"]


def test_a_bad_value_is_reported_as_a_bad_value_not_as_a_dead_gateway(tmp_path: Path) -> None:
    """A rejected argument must not masquerade as the gateway being down.

    The controller validates goal/speed/accel itself and raises ValueError
    naming the field. That fell through to the 500 catch-all, and the browser
    proxy turns any 5xx into "Robot gateway is unavailable" -- so a number out
    of range read as a network fault and sent debugging in the wrong direction.
    """

    controller = _connected()

    def refuse(*_args: object, **_kwargs: object) -> dict[str, object]:
        raise ValueError("goal must be between 0 and 4095")

    # joint_2, deliberately: joint_1 is multi-turn and drives through
    # move_multi_turn, so stubbing move() there would test nothing.
    controller.move = refuse  # type: ignore[method-assign]
    with _client(tmp_path, controller) as client:
        client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1024, "rawMax": 3072},
        )
        client.post(
            "/api/robot/arm/joints/joint_3/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1024, "rawMax": 3072},
        )
        response = client.post(
            "/api/robot/arm/joints/joint_2/target", headers=_headers(), json={"degrees": 0.0}
        )

    assert response.status_code == 422
    assert "goal must be between 0 and 4095" in response.json()["detail"]


def test_the_base_can_be_driven_past_a_single_motor_turn() -> None:
    """The whole point of multi-turn: a goal that one turn cannot name.

    A geared base at 4:1 covers only 90 degrees per motor turn, so clamping its
    goal to 0..4095 capped the joint there no matter what limits were declared.
    """

    from robot_gateway.simple_arm_api import JointState

    base = JointState(1, multi_turn=True)
    elbow = JointState(3)

    assert base.raw_max == 30_719 and base.raw_min == -30_719
    assert elbow.raw_max == 4095 and elbow.raw_min == 0
    # Uncalibrated, a multi-turn joint offers the servo's whole multi-turn span
    # rather than one turn of it.
    assert base.raw_bounds() == (-30_719, 30_719)

    # At 4:1 one motor turn is 90 degrees at the joint, so 180 degrees is two
    # turns out -- a goal a single-turn joint physically cannot express.
    base.ratio = 4.0
    base.rawZero = 2048
    beyond = base.goal_for(180.0)
    assert beyond > 4095, f"expected a goal past one turn, got {beyond}"
    assert base.goal_for(-180.0) < 0


def test_a_multi_turn_joint_reports_the_wrap_counted_position(tmp_path: Path) -> None:
    """rawPosition must be the counted position, not the position within a turn.

    The servo's own feedback wraps at 4095, so it reads identically at 10
    degrees and at 370. Reporting that straight through would make the panel
    show a base that teleports every time it crosses the seam.
    """

    from robot_gateway.simple_arm_api import ArmService, JointStore

    service = ArmService(_connected(), JointStore(tmp_path))
    try:
        # What the odometer would have counted after two turns and change. Set
        # directly so this holds whether or not joint_1 is currently multi-turn.
        service._multi_turn_raw[1] = 9000
        state = service.state()
    finally:
        service.close()

    base = next(j for j in state["joints"] if j["id"] == "joint_1")
    assert base["rawPosition"] == 9000
    elbow = next(j for j in state["joints"] if j["id"] == "joint_3")
    # Untouched joints still report straight from the servo.
    assert elbow["rawPosition"] != 9000


def test_an_invalid_multi_turn_position_is_never_replaced_by_wrapping_feedback(
    tmp_path: Path,
) -> None:
    """The regression behind the reported false zero.

    The servo telemetry field wraps each turn. If counted HAT truth is absent,
    the API must publish UNKNOWN instead of substituting that plausible number.
    """

    from robot_gateway.simple_arm_api import ArmService, JointStore

    service = ArmService(_connected(), JointStore(tmp_path))
    service.close()
    service._multi_turn_raw.pop(1, None)
    state = service.state()

    base = next(joint for joint in state["joints"] if joint["id"] == "joint_1")
    assert base["online"] is True
    assert base["rawPosition"] is None
    assert base["degrees"] is None
    assert base["positionTrusted"] is False


def test_a_pi_restart_preserves_an_already_valid_hat_multi_turn_frame(
    tmp_path: Path,
) -> None:
    """A Pi-only restart must not destroy the HAT's still-valid turn count."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    controller._odometers[1] = {"revolutions": 2, "lastRaw": 808}
    controller._servos[1]["rawPosition"] = 808
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    controller.commands.clear()
    service._multi_turn_armed.clear()
    service._multi_turn_raw.clear()

    service._arm_multi_turn()

    operations = [entry["operation"] for entry in controller.commands]
    assert "MULTI_TURN" not in operations
    assert "ODO_ZERO" not in operations
    assert service._multi_turn_raw[1] == 9000


def test_a_direct_target_after_pi_restart_adopts_valid_absolute_mode_without_reconfiguring(
    tmp_path: Path,
) -> None:
    """MULTITURN ON invalidates ODO, so valid Mode-0 truth is readiness proof."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    base = service._store.get("joint_1")
    base.rawZero, base.rawMin, base.rawMax, base.ratio = 2048, -7000, 12_000, 4.0
    service._multi_turn_armed.clear()
    service._multi_turn_raw.clear()
    controller.commands.clear()

    moved = service.targets({"joint_1": 30.0})

    operations = [entry["operation"] for entry in controller.commands]
    assert moved["moved"][0]["goal"] == 2048 + round(30 * 4096 * 4 / 360)
    assert "MULTI_TURN" not in operations
    assert operations[-2:] == ["HOLD_SET", "MOVE_MULTI_TURN"]


def test_a_hat_restart_keeps_the_four_to_one_base_unknown_until_physical_home(
    tmp_path: Path,
) -> None:
    """Wrapped encoder truth alone cannot distinguish positions 90 degrees apart."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    controller._servos[1]["rawPosition"] = 808
    controller._odometers.clear()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    controller.commands.clear()

    service._arm_multi_turn()

    operations = [entry["operation"] for entry in controller.commands]
    assert "MULTI_TURN" not in operations
    assert "ODO_ZERO" not in operations
    assert service.live_raw(1) is None
    base = next(joint for joint in service.state()["joints"] if joint["id"] == "joint_1")
    assert base["positionTrusted"] is False
    configured = service._store.get("joint_1")
    configured.rawZero, configured.rawMin, configured.rawMax, configured.ratio = (
        2048,
        -7000,
        12_000,
        4.0,
    )
    with pytest.raises(HTTPException) as failure:
        service.targets({"joint_1": 30.0})
    assert failure.value.status_code == 409


def test_a_valid_native_absolute_frame_is_trusted_and_drive_ready(
    tmp_path: Path,
) -> None:
    """Native absolute control stays in Mode 0 whether Base is free or held."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    controller.commands.clear()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._multi_turn_armed.clear()
    service._multi_turn_raw.clear()

    service._refresh_multi_turn()

    assert service._multi_turn_raw[1] == 2048
    assert service.live_raw(1) == 2048
    assert 1 in service._multi_turn_armed


def test_base_absolute_truth_is_published_from_the_existing_odo_cache_only(
    tmp_path: Path,
) -> None:
    """Dashboard reads must expose proof without adding serial transactions."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._refresh_multi_turn()
    controller.commands.clear()

    first = service.state()
    second = service.state()

    assert controller.commands == []
    assert first["controller"] == {
        "controllerId": "replay-hat-a",
        "bootId": "replay_boot_1",
        "firmwareVersion": "replay-1.0.0",
            "protocolVersion": 1,
            "multiTurnTruthV3": False,
            "multiTurnAbsoluteV1": True,
            "liveFollowV1": True,
        }
    base = next(joint for joint in first["joints"] if joint["id"] == "joint_1")
    assert base["positionTrusted"] is True
    truth = dict(base["multiTurnTruth"])
    sample_age = truth.pop("sampleAgeMs")
    assert truth == {
        "tracking": True,
        "valid": True,
        "stepMode": False,
        "stepOutstanding": False,
        "countdownObserved": False,
        "resyncNeeded": False,
        "resyncCount": 0,
    }
    assert sample_age >= 0
    second_base = next(joint for joint in second["joints"] if joint["id"] == "joint_1")
    assert second_base["multiTurnTruth"]["sampleAgeMs"] >= base["multiTurnTruth"]["sampleAgeMs"]


def test_base_absolute_truth_never_reports_a_relative_countdown(
    tmp_path: Path,
) -> None:
    """Normal absolute motion keeps encoder truth valid; continuity loss does not."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    odometer = controller._odometers[1]
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._refresh_multi_turn()

    active = next(
        joint for joint in service.state()["joints"] if joint["id"] == "joint_1"
    )
    active_truth = dict(active["multiTurnTruth"])
    assert active_truth.pop("sampleAgeMs") >= 0
    assert active_truth == {
        "tracking": True,
        "valid": True,
        "stepMode": False,
        "stepOutstanding": False,
        "countdownObserved": False,
        "resyncNeeded": False,
        "resyncCount": 0,
    }

    odometer.update({
        "valid": False,
        "stepOutstanding": False,
        "resyncNeeded": True,
        "resyncCount": 3,
    })
    controller._multi_turn_servos.discard(1)
    service._refresh_multi_turn()
    unknown = next(
        joint for joint in service.state()["joints"] if joint["id"] == "joint_1"
    )

    assert unknown["positionTrusted"] is False
    assert unknown["multiTurnTruth"]["valid"] is False
    assert unknown["multiTurnTruth"]["stepMode"] is False
    assert unknown["multiTurnTruth"]["resyncNeeded"] is True
    assert unknown["multiTurnTruth"]["resyncCount"] == 3


def test_base_truth_cache_is_discarded_on_boot_or_capability_discontinuity(
    tmp_path: Path,
) -> None:
    """A stale diagnostic may never look like proof from the current firmware."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._refresh_multi_turn()
    assert service._multi_turn_truth

    controller.simulate_reboot()
    after_boot = service.state()
    base = next(joint for joint in after_boot["joints"] if joint["id"] == "joint_1")
    assert base["multiTurnTruth"] is None
    assert service._multi_turn_truth == {}

    _establish_base_frame(controller)
    service._refresh_multi_turn()
    assert service._multi_turn_truth
    original = controller.transport_state

    def legacy_capabilities() -> dict[str, object]:
        reported = original()
        identity = dict(reported["identity"])
        identity["capabilities"] = [
            capability
            for capability in identity["capabilities"]
                if capability != "multi_turn_absolute_v1"
        ]
        reported["identity"] = identity
        return reported

    controller.transport_state = legacy_capabilities  # type: ignore[method-assign]
    after_downgrade = service.state()
    base = next(
        joint for joint in after_downgrade["joints"] if joint["id"] == "joint_1"
    )
    assert after_downgrade["controller"]["multiTurnTruthV3"] is False
    assert after_downgrade["controller"]["multiTurnAbsoluteV1"] is False
    assert base["multiTurnTruth"] is None
    assert service._multi_turn_truth == {}
    configured = service._store.get("joint_1")
    configured.rawZero, configured.rawMin, configured.rawMax, configured.ratio = (
        2048,
        -7000,
        12_000,
        4.0,
    )
    with pytest.raises(HTTPException) as refusal:
        service.targets({"joint_1": 30.0})
    assert refusal.value.status_code == 409


def test_a_free_base_tracks_encoder_motion_without_leaving_absolute_mode(
    tmp_path: Path,
) -> None:
    """Hand back-driving while free must update count instead of preserving a lie."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    controller._odometers[1] = {"revolutions": 2, "lastRaw": 808}
    controller._servos[1]["rawPosition"] = 808
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    assert service.live_raw(1) == 9000

    controller.torque_off(1)
    controller._servos[1]["rawPosition"] = 1200
    service._refresh_multi_turn()

    assert service.live_raw(1) == 9392
    assert 1 in service._multi_turn_armed
    service._prepare_multi_turn_drive(1)
    assert service.live_raw(1) == 9392
    assert 1 in service._multi_turn_armed


def test_a_new_hat_boot_invalidates_cached_base_position_before_service_tick(
    tmp_path: Path,
) -> None:
    """No stale pre-reboot count may pass target validation in the polling race."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    controller._odometers[1] = {"revolutions": 2, "lastRaw": 808}
    controller._servos[1]["rawPosition"] = 808
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    assert service.live_raw(1) == 9000
    configured = service._store.get("joint_1")
    configured.rawZero, configured.rawMin, configured.rawMax, configured.ratio = (
        2048,
        -7000,
        12_000,
        4.0,
    )

    controller.simulate_reboot()

    state = service.state()
    base = next(joint for joint in state["joints"] if joint["id"] == "joint_1")
    assert base["positionTrusted"] is False
    assert base["rawPosition"] is None
    with pytest.raises(HTTPException) as failure:
        service.targets({"joint_1": 30.0})
    assert failure.value.status_code == 409


def test_base_drive_reports_temporary_controller_unavailability_without_claiming_reboot(
    tmp_path: Path,
) -> None:
    """A transport outage is not evidence that the HAT boot identity changed."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)
    service._store.get("joint_1").rawZero = 2048
    service._arm_multi_turn()
    service._refresh_multi_turn()
    assert service._multi_turn_raw[1] == 2048
    assert 1 in service._multi_turn_armed
    original = controller.transport_state

    def offline() -> dict[str, object]:
        state = original()
        state["connection"] = "offline"
        return state

    controller.transport_state = offline  # type: ignore[method-assign]
    with pytest.raises(HTTPException) as refusal:
        service._prepare_multi_turn_drive(1)

    assert refusal.value.status_code == 409
    assert "temporarily unavailable" in str(refusal.value.detail)
    assert "identity changed" not in str(refusal.value.detail)
    assert service._multi_turn_raw == {}
    assert service._multi_turn_armed == set()


def test_base_drive_reports_an_actual_hat_boot_identity_change(
    tmp_path: Path,
) -> None:
    """A new boot is named explicitly and still invalidates Base truth."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)
    service._store.get("joint_1").rawZero = 2048
    service._arm_multi_turn()
    service._refresh_multi_turn()
    assert service._multi_turn_raw[1] == 2048
    assert 1 in service._multi_turn_armed

    controller.simulate_reboot()
    with pytest.raises(HTTPException) as refusal:
        service._prepare_multi_turn_drive(1)

    assert refusal.value.status_code == 409
    assert "boot identity changed" in str(refusal.value.detail)
    assert service._multi_turn_raw == {}
    assert service._multi_turn_armed == set()


def test_an_offline_same_boot_observation_discards_cached_base_until_fresh_odo(
    tmp_path: Path,
) -> None:
    """A reconnect may reuse a boot id, but never a pre-disconnect Pi cache."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._refresh_multi_turn()
    assert service.live_raw(1) == 2048
    original = controller.transport_state

    def offline() -> dict[str, object]:
        state = original()
        state["connection"] = "offline"
        return state

    controller.transport_state = offline  # type: ignore[method-assign]
    disconnected = service.state()
    base = next(joint for joint in disconnected["joints"] if joint["id"] == "joint_1")
    assert base["rawPosition"] is None
    assert service._multi_turn_raw == {}
    assert service._multi_turn_armed == set()

    controller.transport_state = original  # type: ignore[method-assign]
    reconnected = service.state()
    base = next(joint for joint in reconnected["joints"] if joint["id"] == "joint_1")
    assert base["rawPosition"] is None
    service._arm_multi_turn()
    assert service.live_raw(1) == 2048


def test_set_base_zero_here_explicitly_rehomes_the_lost_multi_turn_frame(
    tmp_path: Path,
) -> None:
    """The operator's physical-zero confirmation is what resolves turn ambiguity."""

    from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore

    controller = _connected()
    controller._odometers.clear()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    controller.commands.clear()

    state = service.calibrate("joint_1", CalibrateRequest(here=["rawZero"]))

    operations = [entry["operation"] for entry in controller.commands]
    assert operations[0] == "HOME_MULTI_TURN"
    base = next(joint for joint in state["joints"] if joint["id"] == "joint_1")
    assert base["rawZero"] == 2048
    assert base["positionTrusted"] is True
    assert base["multiTurnTruth"]["stepMode"] is False
    assert 1 in service._multi_turn_armed

    # It is still free; encoder movement updates the same native absolute frame.
    controller._servos[1]["rawPosition"] = 2300
    service._refresh_multi_turn()
    assert service.live_raw(1) == 2300
    service._prepare_multi_turn_drive(1)
    assert service.live_raw(1) == 2300
    assert 1 in service._multi_turn_armed


def test_rehoming_base_shifts_raw_limits_without_changing_degree_bounds(
    tmp_path: Path,
) -> None:
    """A new controller frame must not silently make saved Base travel asymmetric."""

    from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    base = service._store.get("joint_1")
    base.rawZero, base.ratio = 2048, 4.0
    base.rawMin, base.rawMax = -6144, 10_240
    before = base.degree_bounds()
    controller._servos[1]["rawPosition"] = 3000

    service.calibrate("joint_1", CalibrateRequest(here=["rawZero"]))

    assert base.rawZero == 3000
    assert (base.rawMin, base.rawMax) == (-5192, 11_192)
    assert base.degree_bounds() == before


def test_typed_signed_zero_is_allowed_only_for_the_native_multi_turn_base(
    tmp_path: Path,
) -> None:
    with _client(tmp_path, _connected()) as client:
        base = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"rawZero": -5000},
        )
        shoulder = client.post(
            "/api/robot/arm/joints/joint_2/calibrate",
            headers=_headers(),
            json={"rawZero": -1},
        )

    assert base.status_code == 200
    assert base.json()["joints"][0]["rawZero"] == -5000
    assert shoulder.status_code == 422


def test_controller_command_errors_are_structured_for_the_proxy(tmp_path: Path) -> None:
    """The laptop can only explain a refusal if the Pi preserves its code."""

    from robot_gateway.arm_controller import ControllerCommandError

    controller = _connected()
    controller.home_multi_turn = lambda _servo_id: (_ for _ in ()).throw(  # type: ignore[method-assign]
        ControllerCommandError("ODOMETER_UNAVAILABLE", {"phase": "multi_turn_frame"})
    )
    with _client(tmp_path, controller) as client:
        response = client.post(
            "/api/robot/arm/joints/joint_1/calibrate",
            headers=_headers(),
            json={"here": ["rawZero"]},
        )

    assert response.status_code == 409
    assert response.json()["detail"] == {
        "code": "ODOMETER_UNAVAILABLE",
        "phase": "multi_turn_frame",
    }


def test_a_failed_base_send_is_not_chased_later(tmp_path: Path) -> None:
    """A command that never got accepted is not an outstanding operator goal."""

    from robot_gateway.arm_controller import ControllerCommandError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    base = service._store.get("joint_1")
    base.rawZero, base.rawMin, base.rawMax, base.ratio = 2048, -7000, 12_000, 4.0
    service._multi_turn_raw[1] = 2048

    def fail(*_args: object, **_kwargs: object) -> dict[str, object]:
        raise ControllerCommandError("BUS_ERROR")

    controller.move_multi_turn = fail  # type: ignore[method-assign]
    with pytest.raises(ControllerCommandError, match="BUS_ERROR"):
        service.targets({"joint_1": 120.0})

    assert "joint_1" not in service._goals


def test_an_untrusted_base_position_is_not_chased_from_wrapping_feedback(
    tmp_path: Path,
) -> None:
    """The chase loop must not reinterpret one-turn servo feedback as counted."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._multi_turn_raw.pop(1, None)
    service._held = [1]
    service._goals["joint_1"] = (7000, 2, time.monotonic() + 5)
    moved: list[int] = []

    def record_move(
        _servo_id: int, goal: int, _speed: int, _acceleration: int
    ) -> dict[str, object]:
        moved.append(goal)
        return {}

    controller.move_multi_turn = record_move  # type: ignore[method-assign]

    service._chase_goals()

    assert moved == []
    assert "joint_1" in service._goals


def test_a_multi_turn_move_hands_the_controller_one_goal() -> None:
    """The host must not walk the joint itself.

    Aiming steps from here meant correcting against telemetry a refresh old, by
    which time the joint had moved several hundred counts -- so corrections
    pointed backwards as often as forwards and the base hunted left-right. The
    controller samples position every 2 ms and does the stepping there; this
    side sends the destination once.
    """

    from robot_gateway.simple_arm_api import ArmService, JointStore

    assert not hasattr(ArmService, "_step_multi_turn"), "host stepping was removed"
    assert not hasattr(ArmService, "_drive_multi_turn"), "host stepping was removed"
    del JointStore


def test_base_zero_and_targets_use_native_absolute_mode_end_to_end(tmp_path: Path) -> None:
    """Zero is an offset; every later target is one counted absolute destination."""

    from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore

    controller = ReplayArmController.connected(
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    service = ArmService(controller, JointStore(tmp_path))
    service.close()

    service.calibrate("joint_1", CalibrateRequest(here=["rawZero"], ratio=4.0))
    base = service._store.get("joint_1")
    base.rawMin, base.rawMax = -10_000, 14_000
    controller.commands.clear()

    first = service.targets({"joint_1": 30.0})
    repeated = service.targets({"joint_1": 30.0})
    reverse = service.targets({"joint_1": -30.0})
    two_turns_positive = service.targets({"joint_1": 180.0})
    two_turns_negative = service.targets({"joint_1": -180.0})

    expected_delta = round(30.0 * 4096 * 4.0 / 360.0)
    goals = [
        entry["goal"]
        for entry in controller.commands
        if entry["operation"] == "MOVE_MULTI_TURN"
    ]
    assert first["moved"][0]["goal"] == 2048 + expected_delta
    assert repeated["moved"][0]["goal"] == 2048 + expected_delta
    assert reverse["moved"][0]["goal"] == 2048 - expected_delta
    assert goals == [
        2048 + expected_delta,
        2048 + expected_delta,
        2048 - expected_delta,
        2048 + 8192,
        2048 - 8192,
    ]
    assert two_turns_positive["moved"][0]["goal"] == 2048 + 8192
    assert two_turns_negative["moved"][0]["goal"] == 2048 - 8192
    assert controller.transport_state()["servosTelemetry"][0]["operatingMode"] == 0
    truth = controller.odometer_read(1)
    assert truth["multiTurnPosition"] == 2048 - 8192
    assert truth["stepMode"] is False


def test_replay_native_multi_turn_configuration_survives_torque_release() -> None:
    """The fake must model Mode-0 configuration independently from authority."""

    from robot_gateway.arm_controller import ControllerCommandError

    controller = ReplayArmController.connected(servos=[_servo(1, 2048)])
    controller.set_hold_servos([1])
    with pytest.raises(ControllerCommandError, match="MODE_NOT_POSITION"):
        controller.move_multi_turn(1, 6000, 1000, 20)

    configured = controller.set_multi_turn(1, True)
    zeroed = controller.odometer_zero(1)
    controller.set_hold_servos([1])
    controller.move_multi_turn(1, 6000, 1000, 20)
    controller.set_hold_servos([])

    assert configured["operatingMode"] == 0
    assert zeroed["stepMode"] is False
    assert controller.transport_state()["servosTelemetry"][0]["operatingMode"] == 0
    controller.set_hold_servos([1])
    assert controller.move_multi_turn(1, -3000, 1000, 20)["goal"] == -3000


def test_replay_multiturn_configuration_invalidates_odo_until_zero() -> None:
    """Replay must expose the same ON -> UNKNOWN -> ODO_ZERO sequence as firmware."""

    controller = ReplayArmController.connected(servos=[_servo(1, 2048)])
    controller.set_multi_turn(1, True)
    controller.odometer_zero(1)
    assert controller.odometer_read(1)["valid"] is True

    controller.set_multi_turn(1, True)

    assert controller.odometer_read(1)["valid"] is False
    zeroed = controller.odometer_zero(1)
    assert zeroed["valid"] is True
    assert zeroed["stepMode"] is False


def test_replay_home_preserves_the_exact_signed_native_coordinate() -> None:
    controller = ReplayArmController.connected(servos=[_servo(1, 2048)])
    controller.set_multi_turn(1, True)
    controller.odometer_zero(1)
    controller.set_hold_servos([1])
    controller.move_multi_turn(1, -5000, 1000, 20)
    controller.torque_off(1)

    homed = controller.home_multi_turn(1)

    assert controller.transport_state()["servosTelemetry"][0]["rawPosition"] == 3192
    assert homed["multiTurnPosition"] == -5000
    assert homed["revolutions"] == -2
    assert homed["rawPosition"] == 3192


def test_the_height_model_agrees_with_the_arm_in_servo_degrees() -> None:
    """Servo degrees, not the twin's frame, because that is what callers hold.

    Getting this wrong is not a rounding error. The servo zeros sit mid-travel,
    so the upper arm is straight UP at servo shoulder 0 and the arm is straight
    at servo elbow 90. Reading those as the twin's frame put the elbow 180 mm
    below where it physically was and made the guard refuse most forward reach.
    """

    from robot_gateway.simple_arm_api import (
        BASE_HEIGHT_MM,
        DISTAL_MM,
        UPPER_ARM_MM,
        lowest_point_mm,
    )

    # Straight up: elbow one upper-arm above the pivot, tip a forearm above that.
    assert lowest_point_mm(0.0, 90.0) == pytest.approx(BASE_HEIGHT_MM + UPPER_ARM_MM)
    # Stretched horizontally forward: the whole arm sits level with the pivot.
    assert lowest_point_mm(-90.0, 90.0) == pytest.approx(BASE_HEIGHT_MM)
    # Upper arm horizontal, forearm pointing straight down: this is the pose
    # that actually drives through the table, and it is the tip that does it.
    assert lowest_point_mm(-90.0, 0.0) == pytest.approx(BASE_HEIGHT_MM - DISTAL_MM)


def test_floor_guard_pulls_a_below_floor_pose_up_to_the_plane() -> None:
    """The keep-out plane is enforced on the Pi, not just drawn in the browser.

    Clamped, never refused: dragging too low must stop the arm at the plane, not
    error at the operator.
    """

    from robot_gateway.simple_arm_api import ArmService, FLOOR_MM, lowest_point_mm

    class _Service:
        _floor_guard = True

        def __init__(self) -> None:
            import threading

            self._lock = threading.RLock()

        def _measured_degrees(self, name: str) -> float:
            # Standing straight up, comfortably clear of the plane.
            return 0.0 if name == "joint_2" else 90.0

    service = _Service()
    # Reaching out and folding the forearm down: 60 - 220 = -160 mm at the tip.
    result = ArmService._guard_floor(service, {"joint_2": -90.0, "joint_3": 0.0})

    guarded = sorted(result.items())
    assert lowest_point_mm(result["joint_2"], result["joint_3"]) >= FLOOR_MM - 1e-6, guarded
    # It moved toward the request rather than refusing or freezing.
    assert result["joint_2"] < 0.0


def test_the_guard_leaves_a_reachable_forward_pose_alone() -> None:
    """The bug this pins: a pose well clear of the desk was being clamped.

    With the frames conflated, the elbow term went under the plane for any
    servo shoulder below about -6 degrees, so ordinary forward reach came back
    pulled up and the operator saw a move that stopped short of what they asked.
    """

    from robot_gateway.simple_arm_api import ArmService

    class _Service:
        _floor_guard = True

        def __init__(self) -> None:
            import threading

            self._lock = threading.RLock()

        def _measured_degrees(self, name: str) -> float:
            return 0.0 if name == "joint_2" else 90.0

    wanted = {"joint_2": -45.0, "joint_3": 60.0}
    assert ArmService._guard_floor(_Service(), dict(wanted)) == wanted

    # Guard off: even a pose that would bury the tip passes through untouched.
    off = _Service()
    off._floor_guard = False
    assert ArmService._guard_floor(off, {"joint_2": -90.0, "joint_3": 0.0}) == {
        "joint_2": -90.0,
        "joint_3": 0.0,
    }


class LegacyAutomaticStopController(ReplayArmController):
    """Replay the narrow, reasonless STOP signature emitted by firmware 2.4."""

    def __init__(self, *, blocker: str | None = None) -> None:
        super().__init__(
            connected=True,
            servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
        )
        self.blocker = blocker

    def trigger_automatic_stop(self) -> None:
        with self._lock:
            self._stopped = True

    def transport_state(self) -> dict[str, object]:
        reported = super().transport_state()
        identity = reported.get("identity")
        if isinstance(identity, dict):
            reported["identity"] = {
                **identity,
                "firmwareVersion": "arm-hat-2.4.0",
            }
        reported["operatorInspectionRequired"] = False
        reported["safetyStopReason"] = None
        reported["hardwareEstop"] = (
            "active" if self.blocker == "hardware_estop" else "not_detected"
        )
        telemetry = reported.get("servosTelemetry")
        if isinstance(telemetry, list) and telemetry:
            first = telemetry[0]
            if isinstance(first, dict):
                if self.blocker == "torque_on":
                    first["torqueState"] = "on"
                    reported["torqueState"] = "on"
                elif self.blocker == "moving":
                    first["moving"] = True
                elif self.blocker == "stale":
                    first["packetAgeMs"] = 10_000
                elif self.blocker == "servo_error":
                    first["errors"] = ["overload"]
            if self.blocker == "incomplete":
                reported["servosTelemetry"] = telemetry[:-1]
        return reported


def _stopped_legacy_service(
    tmp_path: Path, *, blocker: str | None = None
) -> tuple[LegacyAutomaticStopController, object]:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = LegacyAutomaticStopController(blocker=blocker)
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)
    controller.trigger_automatic_stop()
    controller.commands.clear()
    return controller, service


def test_legacy_24_reasonless_automatic_stop_recovers_once(tmp_path: Path) -> None:
    controller, service = _stopped_legacy_service(tmp_path)
    try:
        before = controller.transport_state()
        assert before["motionState"] == "stopped"
        assert before["operatorInspectionRequired"] is False
        assert before["safetyStopReason"] is None

        service._recover_stop()
        service._recover_stop()

        resets = [
            command
            for command in controller.commands
            if command["operation"] == "RESET"
        ]
        assert resets == [{"operation": "RESET", "inspected": True}]
        assert controller.transport_state()["motionState"] != "stopped"
        assert service.state()["stopped"] is False
    finally:
        service.close()


def test_local_operator_stop_is_never_treated_as_legacy_automatic_stop(
    tmp_path: Path,
) -> None:
    controller, service = _stopped_legacy_service(tmp_path)
    try:
        service.stop_for_commissioning()
        controller.commands.clear()

        service._recover_stop()

        assert not any(
            command["operation"] == "RESET" for command in controller.commands
        )
        assert controller.transport_state()["motionState"] == "stopped"
        assert service.state()["safetyStopReason"] == "EXPLICIT_STOP"
    finally:
        service.close()


def test_explicit_stop_marker_survives_restart_and_blocks_legacy_auto_reset(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    state_dir = tmp_path / "explicit-stop-marker"
    controller = LegacyAutomaticStopController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)
    service.stop_for_commissioning()
    service.close()

    fresh = ArmService(controller, JointStore(state_dir))
    try:
        time.sleep(0.15)
        assert not any(
            command["operation"] == "RESET" for command in controller.commands
        )
        assert fresh.state()["stopped"] is True
        assert fresh.state()["safetyStopReason"] == "EXPLICIT_STOP"
    finally:
        fresh.close()


def test_operator_stop_wins_if_it_lands_during_legacy_automatic_reset(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class ResetPausedLegacyController(LegacyAutomaticStopController):
        def __init__(self) -> None:
            super().__init__()
            self.reset_entered = threading.Event()
            self.release_reset = threading.Event()

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            self.reset_entered.set()
            assert self.release_reset.wait(3.0)
            return super().reset(inspected=inspected)

    controller = ResetPausedLegacyController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path / "auto-reset-race"))
    service.close()
    service._service.join(timeout=1.0)
    controller.trigger_automatic_stop()
    controller.commands.clear()
    recovery = threading.Thread(target=service._recover_stop)

    try:
        recovery.start()
        assert controller.reset_entered.wait(2.0)
        service.stop_for_commissioning()
        controller.release_reset.set()
        recovery.join(timeout=2.0)

        operations = [command["operation"] for command in controller.commands]
        assert recovery.is_alive() is False
        assert operations[-1] == "STOP"
        assert controller.transport_state()["motionState"] == "stopped"
        assert service.state()["safetyStopReason"] == "EXPLICIT_STOP"
    finally:
        controller.release_reset.set()
        recovery.join(timeout=2.0)
        service.close()


@pytest.mark.parametrize(
    "blocker",
    [
        "hardware_estop",
        "torque_on",
        "moving",
        "stale",
        "servo_error",
        "incomplete",
    ],
)
def test_legacy_automatic_stop_does_not_reset_with_live_safety_blocker(
    tmp_path: Path,
    blocker: str,
) -> None:
    controller, service = _stopped_legacy_service(tmp_path, blocker=blocker)
    try:
        service._recover_stop()

        assert not any(
            command["operation"] == "RESET" for command in controller.commands
        )
        assert controller.transport_state()["motionState"] == "stopped"
    finally:
        service.close()


def test_controller_stop_without_automatic_signature_requires_explicit_clear(
    tmp_path: Path,
) -> None:
    """An unclassified STOP is never guessed to be an automatic legacy fault."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    try:
        controller.stop()
        controller.commands.clear()
        assert controller.transport_state()["motionState"] == "stopped"
        service._recover_stop()
        assert service._clear_unwanted_stop() is False
        assert controller.transport_state()["motionState"] == "stopped"
        assert not any(
            command["operation"] == "RESET" for command in controller.commands
        )

        service.clear_stop()
        assert controller.transport_state()["motionState"] != "stopped"
        reset = next(
            command for command in controller.commands if command["operation"] == "RESET"
        )
        assert reset["inspected"] is True
    finally:
        service.close()


def test_operator_stop_wins_if_it_lands_during_explicit_clear_reset(
    tmp_path: Path,
) -> None:
    """A newer STOP must physically win over an in-flight explicit RESET."""

    from robot_gateway.simple_arm_api import ArmService

    class ResetPausedController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.reset_entered = threading.Event()
            self.release_reset = threading.Event()

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            self.reset_entered.set()
            assert self.release_reset.wait(3.0)
            return super().reset(inspected=inspected)

    controller = ResetPausedController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)
    result: dict[str, object] = {}

    def recover() -> None:
        result["cleared"] = service.clear_stop_for_commissioning()

    recovery_thread = threading.Thread(target=recover)
    try:
        # Model the stopped latch that the explicit clear request intends to
        # release, then race a newer operator STOP into that RESET.
        controller.stop()
        controller.commands.clear()
        recovery_thread.start()
        assert controller.reset_entered.wait(2.0)

        # This is a newer, deliberate operator STOP. It bypasses every normal
        # operation lane and must remain the final physical controller state.
        service.stop_for_commissioning()
        controller.release_reset.set()
        recovery_thread.join(timeout=2.0)

        operations = [command["operation"] for command in controller.commands]
        assert recovery_thread.is_alive() is False
        assert operations[-1] == "STOP"
        assert operations.count("RESET") == 1
        assert controller.transport_state()["motionState"] == "stopped"
        with pytest.raises(ControllerCommandError, match="STOPPED"):
            controller.set_hold_servos([2])
    finally:
        controller.release_reset.set()
        recovery_thread.join(timeout=2.0)


def test_failed_final_stop_persists_across_restart_and_blocks_physical_mutation(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.physical_arm_api import create_physical_arm_router
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class FinalStopLostController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.reset_entered = threading.Event()
            self.release_reset = threading.Event()
            self.fail_stop_after_reset = False

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            self.reset_entered.set()
            assert self.release_reset.wait(3.0)
            result = super().reset(inspected=inspected)
            self.fail_stop_after_reset = True
            return result

        def stop(self) -> dict[str, object]:
            if self.fail_stop_after_reset:
                self.fail_stop_after_reset = False
                raise ControllerTransportError("final STOP receipt was lost")
            return super().stop()

    state_dir = tmp_path / "durable-state"
    controller = FinalStopLostController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)
    errors: list[BaseException] = []

    def clear() -> None:
        try:
            service.clear_stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            errors.append(error)

    controller.stop()
    controller.commands.clear()
    thread = threading.Thread(target=clear)
    try:
        thread.start()
        assert controller.reset_entered.wait(2.0)
        service.stop_for_commissioning()
        controller.release_reset.set()
        thread.join(timeout=2.0)
    finally:
        controller.release_reset.set()
        thread.join(timeout=2.0)
        service.close()

    assert len(errors) == 1
    assert isinstance(errors[0], ControllerCommandError)
    assert errors[0].code == "STOP_DELIVERY_UNKNOWN"
    assert controller.transport_state()["motionState"] != "stopped"
    marker = state_dir / "arm-clear-required.json"
    assert marker.exists()

    # Model the new Pi process against the same still-running, now-unlatched
    # HAT. The disk marker, not RAM or controller state, must close both APIs.
    fresh = ArmService(controller, JointStore(state_dir))
    fresh.close()
    fresh._service.join(timeout=1.0)
    controller.commands.clear()
    app = FastAPI()
    app.include_router(
        create_physical_arm_router(
            controller,
            operation_lock=fresh.operation_lock,
            admission_lock=fresh.operation_admission_lock,
            mutation_context=fresh.commissioning_mutation,
            clear_mutation_context=fresh.commissioning_clear_mutation,
            stop_callback=fresh.stop_for_commissioning,
            reset_callback=fresh.clear_stop_for_commissioning,
        )
    )
    with TestClient(app) as client:
        refused = client.post(
            "/api/robot/physical/arm/servos/hold-set",
            json={
                "servoIds": [2],
                "leaseMs": 1500,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
            },
        )
        guarded_commands = list(controller.commands)
        cleared = client.post(
            "/api/robot/physical/arm/reset",
            json={"acknowledgedPhysicalInspection": True},
        )
        allowed = client.post(
            "/api/robot/physical/arm/servos/hold-set",
            json={
                "servoIds": [2],
                "leaseMs": 1500,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
            },
        )

    assert fresh.state()["operatorInspectionRequired"] is False
    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "STOPPED"
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "TORQUE_LEASE"}
        for command in guarded_commands
    )
    assert cleared.status_code == 200, cleared.text
    assert marker.exists() is False
    assert allowed.status_code == 200, allowed.text


def test_inspected_clear_without_durable_state_refuses_before_reset() -> None:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore())
    service.close()
    service._service.join(timeout=1.0)
    controller.stop()
    controller.commands.clear()

    with pytest.raises(
        ControllerCommandError, match="SAFETY_LATCH_PERSISTENCE_FAILED"
    ):
        service.clear_stop_for_commissioning()

    assert not any(
        command["operation"] == "RESET" for command in controller.commands
    )
    assert controller.transport_state()["motionState"] == "stopped"


def test_reset_receipt_loss_leaves_write_ahead_marker_for_fresh_service(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class LosesFirstResetReceipt(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.lose_reset_receipt = True

        def reset(self, *, inspected: bool = False) -> dict[str, object]:
            result = super().reset(inspected=inspected)
            if self.lose_reset_receipt:
                self.lose_reset_receipt = False
                raise ControllerTransportError("RESET receipt lost")
            return result

    state_dir = tmp_path / "write-ahead-state"
    controller = LosesFirstResetReceipt()
    _establish_base_frame(controller)
    controller.stop()
    service = ArmService(controller, JointStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)

    with pytest.raises(ControllerTransportError, match="RESET receipt lost"):
        service.clear_stop_for_commissioning()

    marker = state_dir / "arm-clear-required.json"
    assert marker.exists()
    assert controller.transport_state()["motionState"] != "stopped"
    service.close()

    fresh = ArmService(controller, JointStore(state_dir))
    fresh.close()
    fresh._service.join(timeout=1.0)
    controller.commands.clear()
    with pytest.raises(ControllerCommandError, match="STOPPED"):
        fresh.torque([2])
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "TORQUE_LEASE"}
        for command in controller.commands
    ), controller.commands

    fresh.clear_stop_for_commissioning()
    assert marker.exists() is False
    fresh.torque([2])
    assert any(
        command["operation"] == "HOLD_SET" and command.get("servoIds") == [2]
        for command in controller.commands
    )


def test_marker_clear_failure_reasserts_physical_stop_before_returning_error(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class DeletesThenFailsStore(JointStore):
        def clear_safety_latch(self) -> None:
            super().clear_safety_latch()
            raise OSError("directory fsync failed after unlink")

    state_dir = tmp_path / "marker-clear-failure"
    controller = _connected()
    controller.stop()
    service = ArmService(controller, DeletesThenFailsStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)
    controller.commands.clear()

    with pytest.raises(
        ControllerCommandError, match="SAFETY_LATCH_PERSISTENCE_FAILED"
    ):
        service.clear_stop_for_commissioning()

    assert (state_dir / "arm-clear-required.json").exists() is False
    assert [command["operation"] for command in controller.commands][-1] == "STOP"
    assert controller.transport_state()["motionState"] == "stopped"

    fresh = ArmService(controller, JointStore(state_dir))
    fresh.close()
    fresh._service.join(timeout=1.0)
    controller.commands.clear()
    with pytest.raises(ControllerCommandError, match="STOPPED"):
        fresh.torque([2])
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "TORQUE_LEASE"}
        for command in controller.commands
    ), controller.commands


def test_operator_stop_reaches_controller_while_marker_clear_is_blocked(
    tmp_path: Path,
) -> None:
    """Filesystem durability work must never delay the emergency STOP bus call."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class BlockingClearStore(JointStore):
        def __init__(self, state_dir: Path) -> None:
            super().__init__(state_dir)
            self.clear_started = threading.Event()
            self.release_clear = threading.Event()

        def clear_safety_latch(self) -> None:
            self.clear_started.set()
            assert self.release_clear.wait(3.0)
            super().clear_safety_latch()

    class StopObservedController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.stop_delivered = threading.Event()

        def stop(self) -> dict[str, object]:
            result = super().stop()
            self.stop_delivered.set()
            return result

    state_dir = tmp_path / "blocked-marker-clear"
    store = BlockingClearStore(state_dir)
    controller = StopObservedController()
    _establish_base_frame(controller)
    controller.stop()
    service = ArmService(controller, store)
    service.close()
    service._service.join(timeout=1.0)
    controller.commands.clear()
    controller.stop_delivered.clear()
    clear_errors: list[BaseException] = []
    stop_errors: list[BaseException] = []

    def clear() -> None:
        try:
            service.clear_stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            clear_errors.append(error)

    def stop() -> None:
        try:
            service.stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            stop_errors.append(error)

    clear_thread = threading.Thread(target=clear)
    stop_thread = threading.Thread(target=stop)
    try:
        clear_thread.start()
        assert store.clear_started.wait(2.0)
        stop_thread.start()

        # The marker fsync is still blocked, but the physical STOP must already
        # have reached the controller. This is the emergency-latency contract.
        assert controller.stop_delivered.wait(0.5)
        assert store.release_clear.is_set() is False
    finally:
        store.release_clear.set()
        clear_thread.join(timeout=2.0)
        stop_thread.join(timeout=2.0)
        service.close()

    assert clear_thread.is_alive() is False
    assert stop_thread.is_alive() is False
    assert clear_errors == []
    assert stop_errors == []
    assert [command["operation"] for command in controller.commands][-1] == "STOP"
    assert controller.transport_state()["motionState"] == "stopped"


def test_newer_failed_stop_marker_cannot_be_deleted_by_older_clear(
    tmp_path: Path,
) -> None:
    """A STOP marker persisted after the precheck must win the disk commit order."""

    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class StopMarkerStore(JointStore):
        def __init__(self, state_dir: Path) -> None:
            super().__init__(state_dir)
            self.stop_marker_persisted = threading.Event()

        def persist_safety_latch(self, reason: str) -> None:
            super().persist_safety_latch(reason)
            if reason == "STOP_DELIVERY_UNKNOWN":
                self.stop_marker_persisted.set()

    class StopFirstSafetyLock:
        """Pause the clear's second disk transaction until STOP has persisted."""

        def __init__(self, marker_persisted: threading.Event) -> None:
            self._lock = threading.Lock()
            self._marker_persisted = marker_persisted
            self._clear_acquisitions = 0
            self.clear_waiting = threading.Event()

        def __enter__(self):
            if threading.current_thread().name == "clear-marker":
                self._clear_acquisitions += 1
                if self._clear_acquisitions == 2:
                    self.clear_waiting.set()
                    assert self._marker_persisted.wait(3.0)
            self._lock.acquire()
            return self

        def __exit__(self, _type, _value, _traceback) -> None:
            self._lock.release()

    class LosesNextStopController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.lose_next_stop = False

        def stop(self) -> dict[str, object]:
            if self.lose_next_stop:
                self.lose_next_stop = False
                raise ControllerTransportError("STOP delivery was not confirmed")
            return super().stop()

    state_dir = tmp_path / "newer-stop-marker"
    store = StopMarkerStore(state_dir)
    controller = LosesNextStopController()
    _establish_base_frame(controller)
    controller.stop()
    service = ArmService(controller, store)
    service.close()
    service._service.join(timeout=1.0)
    safety_lock = StopFirstSafetyLock(store.stop_marker_persisted)
    service._safety_latch_lock = safety_lock
    clear_errors: list[BaseException] = []
    stop_errors: list[BaseException] = []

    def clear() -> None:
        try:
            service.clear_stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            clear_errors.append(error)

    def stop() -> None:
        try:
            service.stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            stop_errors.append(error)

    clear_thread = threading.Thread(target=clear, name="clear-marker")
    stop_thread = threading.Thread(target=stop, name="newer-stop")
    try:
        clear_thread.start()
        assert safety_lock.clear_waiting.wait(2.0)
        controller.lose_next_stop = True
        stop_thread.start()
        assert store.stop_marker_persisted.wait(2.0)
        clear_thread.join(timeout=2.0)
        stop_thread.join(timeout=2.0)
    finally:
        clear_thread.join(timeout=2.0)
        stop_thread.join(timeout=2.0)
        service.close()

    assert clear_thread.is_alive() is False
    assert stop_thread.is_alive() is False
    assert clear_errors == []
    assert len(stop_errors) == 1
    assert isinstance(stop_errors[0], ControllerTransportError)
    marker = state_dir / "arm-clear-required.json"
    assert marker.exists()

    # Model a fresh Pi against a controller that cannot supply the old RAM
    # latch. The surviving marker must still reject all torque before a write.
    fresh_controller = _connected()
    fresh = ArmService(fresh_controller, JointStore(state_dir))
    fresh.close()
    fresh._service.join(timeout=1.0)
    fresh_controller.commands.clear()
    with pytest.raises(ControllerCommandError, match="STOPPED"):
        fresh.torque([2])
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "TORQUE_LEASE"}
        for command in fresh_controller.commands
    )


def test_a_faulted_controller_reconnects_in_the_background_without_motion(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    controller.fail_next("SCAN", "link dropped")
    with pytest.raises(ControllerTransportError):
        controller.scan(1, 4)
    controller.commands.clear()
    service._last_controller_reconnect = 0.0

    assert service._recover_controller() is True
    assert controller.transport_state()["connection"] == "online"
    assert all(
        command["operation"]
        not in {"SCAN", "HOLD_SET", "TORQUE_LEASE", "MOVE", "ODO_ZERO", "RESET"}
        for command in controller.commands
    )


def test_background_controller_reconnect_is_bounded_and_ignores_unconfigured(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerUnavailableError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    class OfflineController:
        def __init__(self, configured: bool = True) -> None:
            self.configured = configured
            self.reconnects = 0

        def transport_state(self) -> dict[str, object]:
            return {
                "configured": self.configured,
                "connection": "offline" if self.configured else "not_configured",
            }

        def reconnect(self) -> dict[str, object]:
            self.reconnects += 1
            raise ControllerUnavailableError("still booting")

    offline = OfflineController()
    service = ArmService(offline, JointStore(tmp_path))  # type: ignore[arg-type]
    service.close()
    service._last_controller_reconnect = 0.0
    assert service._recover_controller() is False
    assert service._recover_controller() is False
    assert offline.reconnects == 1

    absent = OfflineController(configured=False)
    absent_service = ArmService(absent, JointStore(tmp_path / "absent"))  # type: ignore[arg-type]
    absent_service.close()
    assert absent_service._recover_controller() is False
    assert absent.reconnects == 0


# ---- a goal outlives the lease that was carrying it -------------------------


def _calibrated(client: TestClient, *names: str) -> None:
    for name in names:
        client.post(
            f"/api/robot/arm/joints/{name}/calibrate",
            headers=_headers(),
            json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
        )


def test_a_move_the_controller_abandoned_part_way_is_sent_again(tmp_path: Path) -> None:
    """The failure this exists for, in the shape it was measured in.

    A MOVE only lives as long as torque authority does, and authority dies the
    moment the controller's 750 ms host watchdog runs dry -- a whole turn of the
    2 s hold lease early. The joint stops wherever it had got to, the next hold
    capture writes that position as its hold, and the HTTP call has already
    answered 200. Measured on hardware: the elbow was sent to 2427, was pinned at
    1392 seven tenths of a second later, and was still at 1391 seven seconds on.
    """

    class PinsTheFirstMove(ReplayArmController):
        def __init__(self, **arguments: object) -> None:
            super().__init__(**arguments)  # type: ignore[arg-type]
            self.pinned = False

        def move(self, servo_id: int, goal: int, speed: int, acceleration: int) -> dict[str, object]:
            result = super().move(servo_id, goal, speed, acceleration)
            if not self.pinned:
                self.pinned = True
                # Authority lapsed under it: the servo stopped short and the
                # hold capture pinned it there.
                self._servos[servo_id]["rawPosition"] = 2048 + 40
            return result

    controller = PinsTheFirstMove(
        connected=True, servos=[_servo(2, 2048), _servo(3, 2048)]
    )
    with _client(tmp_path, controller) as client:
        _calibrated(client, "joint_2", "joint_3")
        client.post("/api/robot/arm/target", headers=_headers(), json={"joint_2": 45})

        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            position = controller.transport_state()["servosTelemetry"][0]["rawPosition"]
            if position == 2048 + 512:
                break
            time.sleep(0.05)

    assert controller.pinned, "the test never exercised a pinned move"
    assert position == 2048 + 512, "the abandoned goal was never re-sent"


def test_a_joint_that_will_not_reach_its_goal_is_not_driven_at_it_forever(tmp_path: Path) -> None:
    """A joint stalled on something solid must not be re-commanded without end."""

    from robot_gateway.simple_arm_api import ARRIVAL_ATTEMPTS

    class NeverArrives(ReplayArmController):
        def move(self, servo_id: int, goal: int, speed: int, acceleration: int) -> dict[str, object]:
            result = super().move(servo_id, goal, speed, acceleration)
            self._servos[servo_id]["rawPosition"] = 2048
            return result

    controller = NeverArrives(
        connected=True, servos=[_servo(2, 2048), _servo(3, 2048)]
    )
    with _client(tmp_path, controller) as client:
        _calibrated(client, "joint_2")
        _calibrated(client, "joint_3")
        client.post("/api/robot/arm/target", headers=_headers(), json={"joint_2": 45})
        time.sleep(3.0)
        moves = [entry for entry in controller.commands if entry["operation"] == "MOVE"]

    assert len(moves) == 1 + ARRIVAL_ATTEMPTS, [entry["goal"] for entry in moves]


def test_a_hold_older_than_the_renewal_interval_is_re_sent_before_a_move(tmp_path: Path) -> None:
    """Bookkeeping is not authority.

    `self._held` is what the Pi intends to hold; the controller's lease behind it
    expires on its own clock. Skipping HOLD_SET on the strength of the
    bookkeeping alone is how a move came back NO_TORQUE_LEASE with nothing
    whatever wrong -- measured, two of four multi-joint moves.
    """

    from robot_gateway.simple_arm_api import HOLD_RENEW_SECONDS, ArmService, JointStore

    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    # The service loop is the other thing that keeps the hold fresh, and this is
    # about the request path deciding for itself.
    service.close()
    time.sleep(0.2)
    try:
        joint = service._store.get("joint_2")
        joint.rawZero, joint.rawMin, joint.rawMax = 2048, 1048, 3048
        elbow = service._store.get("joint_3")
        elbow.rawZero, elbow.rawMin, elbow.rawMax = 2048, 1048, 3048
        service.torque([1, 2, 3])
        controller.commands.clear()

        service.targets({"joint_2": 10})
        assert not [
            entry for entry in controller.commands if entry["operation"] == "HOLD_SET"
        ], "a hold taken a moment ago does not need re-sending"

        time.sleep(HOLD_RENEW_SECONDS + 0.1)
        service.targets({"joint_2": 20})
        stale = [entry for entry in controller.commands if entry["operation"] == "HOLD_SET"]
        assert len(stale) == 1
        assert sorted(stale[0]["servoIds"]) == [1, 2, 3], "the whole set has to be re-asserted"
    finally:
        service.close()


def test_clear_stop_waits_for_inflight_hold_renewal_and_reset_stays_disarmed(
    tmp_path: Path,
) -> None:
    """A stale renewal must never land after RESET's torque-off receipt."""

    from robot_gateway.simple_arm_api import ArmService

    class BlockingRenewalController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.block_next_hold = False
            self.renewal_entered = threading.Event()
            self.release_renewal = threading.Event()

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            if self.block_next_hold:
                self.block_next_hold = False
                self.renewal_entered.set()
                assert self.release_renewal.wait(3.0)
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = BlockingRenewalController()
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    clear_finished = threading.Event()
    clear_errors: list[BaseException] = []

    def clear_stop() -> None:
        try:
            service.clear_stop_for_commissioning()
        except BaseException as error:  # pragma: no cover - asserted below
            clear_errors.append(error)
        finally:
            clear_finished.set()

    clear_thread = threading.Thread(target=clear_stop)
    try:
        service.torque([2])
        controller.commands.clear()
        controller.block_next_hold = True
        service._last_hold = 0.0
        assert controller.renewal_entered.wait(2.0)

        # STOP remains preemptive while the background renewal is stalled. The
        # following explicit clear/reset must wait behind that renewal boundary.
        service.stop_for_commissioning()
        clear_thread.start()
        clear_was_serialized = not clear_finished.wait(0.2)

        controller.release_renewal.set()
        assert clear_finished.wait(2.0)
        clear_thread.join(timeout=2.0)

        operations = [command["operation"] for command in controller.commands]
        reset_index = max(
            index for index, operation in enumerate(operations) if operation == "RESET"
        )
        held_after_reset = [
            operation for operation in operations[reset_index + 1 :] if operation == "HOLD_SET"
        ]
        reported = controller.transport_state()

        assert clear_was_serialized is True
        assert clear_errors == []
        assert held_after_reset == []
        assert reported.get("heldServoIds", []) == []
        assert reported["torqueState"] == "off"
        assert service.state()["held"] == []
    finally:
        controller.release_renewal.set()
        clear_thread.join(timeout=2.0)
        service.close()


@pytest.mark.parametrize("release_route", ["simple", "physical"])
def test_torque_release_waits_for_inflight_renewal_and_cannot_be_reenergized(
    tmp_path: Path,
    release_route: str,
) -> None:
    class BlockingRenewalController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.block_next_hold = False
            self.renewal_entered = threading.Event()
            self.release_renewal = threading.Event()

        def set_hold_servos(
            self, servo_ids: list[int], lease_ms: int = 1_500
        ) -> dict[str, object]:
            if self.block_next_hold:
                self.block_next_hold = False
                self.renewal_entered.set()
                assert self.release_renewal.wait(3.0)
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = BlockingRenewalController()
    _establish_base_frame(controller)
    finished = threading.Event()
    received: dict[str, object] = {}
    thread: threading.Thread | None = None
    try:
        with _client(tmp_path, controller) as client:
            held = client.post(
                "/api/robot/arm/torque",
                headers=_headers(),
                json={"hold": [2]},
            )
            assert held.status_code == 200, held.text
            controller.commands.clear()
            controller.block_next_hold = True
            assert controller.renewal_entered.wait(2.0)

            def release() -> None:
                if release_route == "simple":
                    response = client.post(
                        "/api/robot/arm/torque",
                        headers=_headers(),
                        json={"hold": []},
                    )
                else:
                    response = client.post(
                        "/api/robot/physical/arm/servos/torque-off",
                        headers=_headers(),
                        json={"servoId": 2},
                    )
                received["status"] = response.status_code
                received["body"] = response.json()
                finished.set()

            thread = threading.Thread(target=release)
            thread.start()
            release_was_serialized = not finished.wait(0.15)
            controller.release_renewal.set()
            assert finished.wait(2.0)
            thread.join(timeout=1.0)
            time.sleep(0.2)

        assert release_was_serialized is True
        assert received["status"] == 200
        operations = [command["operation"] for command in controller.commands]
        if release_route == "simple":
            release_index = max(
                index
                for index, command in enumerate(controller.commands)
                if command["operation"] == "HOLD_SET"
                and command.get("servoIds") == []
            )
        else:
            release_index = max(
                index
                for index, operation in enumerate(operations)
                if operation == "TORQUE_OFF"
            )
        assert not any(
            command["operation"] == "HOLD_SET" and command.get("servoIds")
            for command in controller.commands[release_index + 1 :]
        )
        reported = controller.transport_state()
        assert reported.get("heldServoIds", []) == []
        assert reported["torqueState"] == "off"
    finally:
        controller.release_renewal.set()
        if thread is not None:
            thread.join(timeout=2.0)


def test_physical_move_waits_for_chase_then_cancels_the_old_simple_goal(
    tmp_path: Path,
) -> None:
    class BlockingChaseController(ReplayArmController):
        def __init__(self) -> None:
            super().__init__(
                connected=True,
                servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)],
            )
            self.block_next_move = False
            self.chase_entered = threading.Event()
            self.release_chase = threading.Event()

        def move(self, servo_id, goal, speed, acceleration):
            if self.block_next_move:
                self.block_next_move = False
                self.chase_entered.set()
                assert self.release_chase.wait(3.0)
            return super().move(servo_id, goal, speed, acceleration)

    controller = BlockingChaseController()
    _establish_base_frame(controller)
    finished = threading.Event()
    received: dict[str, object] = {}
    thread: threading.Thread | None = None
    try:
        with _client(tmp_path, controller) as client:
            calibrated = client.post(
                "/api/robot/arm/joints/joint_2/calibrate",
                headers=_headers(),
                json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
            )
            assert calibrated.status_code == 200, calibrated.text
            elbow = client.post(
                "/api/robot/arm/joints/joint_3/calibrate",
                headers=_headers(),
                json={"rawZero": 2048, "rawMin": 1048, "rawMax": 3048, "ratio": 1},
            )
            assert elbow.status_code == 200, elbow.text
            initial = client.post(
                "/api/robot/arm/joints/joint_2/target",
                headers=_headers(),
                json={"degrees": 10},
            )
            assert initial.status_code == 200, initial.text
            old_goal = initial.json()["moved"][0]["goal"]
            controller.commands.clear()
            controller._servos[2]["rawPosition"] = 2048
            controller._servos[2]["moving"] = False
            controller.block_next_move = True
            assert controller.chase_entered.wait(2.0)

            def physical_move() -> None:
                response = client.post(
                    "/api/robot/physical/arm/servos/move",
                    headers=_headers(),
                    json={
                        "servoId": 2,
                        "goal": 2200,
                        "speed": 500,
                        "acceleration": 20,
                        "acknowledgedPhysicalPowerCut": True,
                        "confirmedServoModel": "ST3215",
                    },
                )
                received["status"] = response.status_code
                received["body"] = response.json()
                finished.set()

            thread = threading.Thread(target=physical_move)
            thread.start()
            physical_was_serialized = not finished.wait(0.15)
            controller.release_chase.set()
            assert finished.wait(2.0)
            thread.join(timeout=1.0)
            time.sleep(0.4)

        moves = [
            command for command in controller.commands if command["operation"] == "MOVE"
        ]
        assert physical_was_serialized is True
        assert received["status"] == 200
        assert [command["goal"] for command in moves] == [old_goal, 2200]
        assert controller._servos[2]["rawPosition"] == 2200
    finally:
        controller.release_chase.set()
        if thread is not None:
            thread.join(timeout=2.0)


def test_a_failed_hold_renewal_keeps_the_set_the_operator_asked_for(tmp_path: Path) -> None:
    """Dropping it de-energised joints nobody was moving.

    The set is the operator's standing instruction, not a record of what the
    controller managed this second. Wiping it meant the next move re-armed from
    empty and sent a HOLD_SET naming only the joints it was driving -- which
    torque-offs every joint left out of it, base and camera included.
    """

    from robot_gateway.arm_controller import ControllerTransportError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    calls = []

    class RefusesTheFirstRenewal(ReplayArmController):
        def set_hold_servos(self, servo_ids: list[int], lease_ms: int = 1_500) -> dict[str, object]:
            calls.append(list(servo_ids))
            if len(calls) == 2:
                raise ControllerTransportError("controller transport failed")
            return super().set_hold_servos(servo_ids, lease_ms)

    controller = RefusesTheFirstRenewal(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    try:
        service.torque([1, 2, 3])
        deadline = time.monotonic() + 5
        while len(calls) < 3 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert len(calls) >= 3, "the renewal was never retried after it failed"
        assert service.state()["held"] == [1, 2, 3]
        assert sorted(calls[2]) == [1, 2, 3], "the retry has to name the whole set"
    finally:
        service.close()


def test_an_in_flight_absolute_base_goal_keeps_valid_truth_during_renewal(
    tmp_path: Path,
) -> None:
    """Native absolute motion never turns encoder position into a countdown."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class ReportsHeldBase(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            state = super().transport_state()
            state["heldServoIds"] = sorted(self._leases)
            return state

    controller = ReportsHeldBase(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    time.sleep(0.2)
    try:
        service.torque([1, 2, 3])
        controller.commands.clear()

        service._apply_hold([1, 2, 3], isolate_multi_turn_failures=True)

        operations = [entry["operation"] for entry in controller.commands]
        assert operations == ["ODO_READ", "HOLD_SET"]
        assert "MULTI_TURN" not in operations
        assert controller.commands[-1]["servoIds"] == [1, 2, 3]
        service._refresh_multi_turn()
        state = service.state()
        assert state["held"] == [1, 2, 3]
        base = next(joint for joint in state["joints"] if joint["id"] == "joint_1")
        assert base["positionTrusted"] is True
    finally:
        service.close()


def test_lost_base_truth_does_not_block_other_hold_renewals(tmp_path: Path) -> None:
    """A Base preparation refusal is local to Base, not a whole-arm lease failure."""

    from robot_gateway.simple_arm_api import ArmService, JointStore

    class LosesBaseTruth(ReplayArmController):
        truth_lost = False

        def transport_state(self) -> dict[str, object]:
            state = super().transport_state()
            state["heldServoIds"] = sorted(self._leases)
            return state

        def odometer_read(self, servo_id: int) -> dict[str, object]:
            result = super().odometer_read(servo_id)
            if servo_id == 1 and self.truth_lost:
                result.update(
                    valid=False,
                    multiTurnPosition=None,
                    stepOutstanding=False,
                    countdownObserved=False,
                    resyncNeeded=True,
                )
            return result

    controller = LosesBaseTruth(
        connected=True, servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048)]
    )
    _establish_base_frame(controller)
    service = ArmService(controller, JointStore(tmp_path))
    try:
        service.torque([1, 2, 3])
        controller.commands.clear()
        controller.truth_lost = True

        deadline = time.monotonic() + 3
        reduced_hold = None
        while time.monotonic() < deadline:
            reduced_hold = next(
                (
                    entry
                    for entry in controller.commands
                    if entry["operation"] == "HOLD_SET" and entry["servoIds"] == [2, 3]
                ),
                None,
            )
            if reduced_hold is not None:
                break
            time.sleep(0.02)

        assert reduced_hold is not None, "Base truth loss skipped renewal for Shoulder and Elbow"
        assert service.state()["held"] == [2, 3]
        telemetry = {
            row["id"]: row for row in controller.transport_state()["servosTelemetry"]
        }
        assert telemetry[1]["torqueState"] == "off"
        assert telemetry[2]["torqueState"] == "on"
        assert telemetry[3]["torqueState"] == "on"
    finally:
        service.close()
