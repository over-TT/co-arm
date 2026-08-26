from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import threading
import time

from fastapi.testclient import TestClient
import pytest

from robot_gateway.arm_controller import (
    ControllerCommandError,
    ControllerTransportError,
    ReplayArmController,
)
from robot_gateway.camera_api import CameraEvidenceService
from robot_gateway.camera_profiles import CAPTURE_PROFILE_DETAIL
from robot_gateway.pi_camera import AutofocusStatus, CapturedFrame, FocusQuality, JPEG_MIME_TYPE
from robot_gateway.runtime import create_app
from robot_gateway.simple_arm_api import ArmService


TOKEN = "arm-observation-token-" + ("o" * 40)
AUTH = {"Authorization": f"Bearer {TOKEN}"}
FIXED_FOCUS = AutofocusStatus(capability="unsupported", mode="fixed", state="fixed")


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


def _frame(payload: bytes) -> CapturedFrame:
    return CapturedFrame(
        data=payload,
        mime_type=JPEG_MIME_TYPE,
        width=640,
        height=480,
        captured_at_utc=datetime(2026, 8, 14, 12, 0, tzinfo=timezone.utc),
        captured_monotonic=10.0,
        sha256=hashlib.sha256(payload).hexdigest(),
        camera_id="rpi-camera-0",
        sensor_model="ov5647",
        autofocus=FIXED_FOCUS,
        focus_quality=FocusQuality(status="unavailable"),
        _monotonic_clock=lambda: 10.1,
    )


class FakeProvider:
    camera_id = "rpi-camera-0"
    sensor_model = "ov5647"
    autofocus_status = FIXED_FOCUS

    def __init__(self, frame: CapturedFrame, *, on_capture=None) -> None:
        self.frame = frame
        self.max_frame_bytes = max(1, len(frame.data))
        self.on_capture = on_capture
        self.capture_calls = 0
        self.capture_profiles: list[str] = []

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame:
        self.capture_calls += 1
        self.capture_profiles.append(profile)
        if self.on_capture is not None:
            self.on_capture()
        return replace(
            self.frame,
            captured_monotonic=time.monotonic(),
            _monotonic_clock=time.monotonic,
        )


class NeverSettlesController(ReplayArmController):
    def move(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        result = super().move(servo_id, goal, speed, acceleration)
        self._servos[servo_id]["moving"] = True
        return result


class SlowOdometerController(ReplayArmController):
    """Model firmware 2.4's guarded STATUS -> ODO_READ -> STATUS latency."""

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        result = super().odometer_read(servo_id)
        time.sleep(0.3)
        return result


class TransientlyStaleOdometerController(ReplayArmController):
    """Return one late-but-valid Base sample immediately after dispatch."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._base_move_dispatched = False
        self.stale_post_dispatch_samples = 0

    def move_set(self, moves):
        result = super().move_set(moves)
        if any(servo_id == 1 for servo_id, *_ in moves):
            self._base_move_dispatched = True
        return result

    def move_multi_turn(
        self, servo_id: int, goal: int, speed: int, acceleration: int
    ) -> dict[str, object]:
        result = super().move_multi_turn(servo_id, goal, speed, acceleration)
        if servo_id == 1:
            self._base_move_dispatched = True
        return result

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        result = super().odometer_read(servo_id)
        if (
            servo_id == 1
            and self._base_move_dispatched
            and self.stale_post_dispatch_samples == 0
        ):
            result["sampleAgeMs"] = 251
            self.stale_post_dispatch_samples += 1
        return result


class PersistentlyStaleOdometerController(TransientlyStaleOdometerController):
    """Keep returning valid Base truth that is too old to prove arrival."""

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        result = ReplayArmController.odometer_read(self, servo_id)
        if servo_id == 1 and self._base_move_dispatched:
            result["sampleAgeMs"] = 251
            self.stale_post_dispatch_samples += 1
        return result


class BaseTruthLostAfterDispatchController(TransientlyStaleOdometerController):
    """Invalidate the counted Base frame after a Base command is dispatched."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.invalid_post_dispatch_samples = 0

    def odometer_read(self, servo_id: int) -> dict[str, object]:
        result = ReplayArmController.odometer_read(self, servo_id)
        if servo_id == 1 and self._base_move_dispatched:
            result["valid"] = False
            self.invalid_post_dispatch_samples += 1
        return result


class MoveSetFailsController(ReplayArmController):
    def move_set(self, moves):
        with self._lock:
            self._before("MOVE_SET", moves=moves)
            self._stopped = True
        raise ControllerCommandError(
            "MOVE_SET_FAILED",
            {
                "phase": "dispatch",
                "failedIndex": 0,
                "stopped": True,
                "torqueState": "unknown",
                "motionMayHaveStarted": True,
                "partialDispatchPossible": False,
                "dispatchedFamilyCount": 1,
            },
        )


class MoveSetUnconfirmedController(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.inspection_required = False

    def move_set(self, moves):
        with self._lock:
            self._before("MOVE_SET", moves=moves)
            self._stopped = True
            self.inspection_required = True
        raise ControllerTransportError("serial timeout")

    def transport_state(self) -> dict[str, object]:
        state = super().transport_state()
        if self.inspection_required:
            state["operatorInspectionRequired"] = True
            state["safetyStopReason"] = "MOVE_SET_UNCONFIRMED"
            state["lastMotionFailure"] = {
                "phase": "unconfirmed",
                "torqueState": "unknown",
                "motionMayHaveStarted": True,
                "partialDispatchPossible": True,
            }
        return state


class PreexistingInspectionController(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.inspection_required = False
        self.inspection_reason = "MOVE_SET_UNCONFIRMED"

    def transport_state(self) -> dict[str, object]:
        state = super().transport_state()
        if self.inspection_required:
            state["operatorInspectionRequired"] = True
            state["safetyStopReason"] = self.inspection_reason
            state["lastMotionFailure"] = {
                "phase": "unconfirmed",
                "torqueState": "unknown",
                "motionMayHaveStarted": True,
                "partialDispatchPossible": True,
            }
        return state


class LegacyFirstMoveUnconfirmedController(ReplayArmController):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.receipt_lost = False

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
        result = super().move(servo_id, goal, speed, acceleration)
        if not self.receipt_lost:
            self.receipt_lost = True
            raise ControllerTransportError("MOVE receipt lost after dispatch")
        return result


def _controller(kind: type[ReplayArmController] = ReplayArmController) -> ReplayArmController:
    controller = kind(
        connected=True,
        servos=[_servo(1, 2048), _servo(2, 2048), _servo(3, 2048), _servo(4, 512)],
    )
    controller.set_multi_turn(1, True)
    controller.odometer_zero(1)
    controller.commands.clear()
    return controller


def _client(
    tmp_path: Path, controller: ReplayArmController, provider: FakeProvider
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


def _client_with_camera_service(
    tmp_path: Path,
    controller: ReplayArmController,
    camera_service: CameraEvidenceService,
) -> TestClient:
    token_file = tmp_path / "gateway.token"
    token_file.write_text(TOKEN, encoding="utf-8")
    return TestClient(
        create_app(
            token_file=token_file,
            arm_controller=controller,
            arm_state_dir=tmp_path / "state",
            camera_service=camera_service,
        )
    )


def _calibrate_and_plan(
    client: TestClient, targets: dict[str, float] | None = None
) -> dict[str, str]:
    for name, zero, maximum in (
        ("joint_1", 2048, 4095),
        ("joint_2", 2048, 4095),
        ("joint_3", 2048, 4095),
        ("joint_4", 512, 1023),
    ):
        response = client.post(
            f"/api/robot/arm/joints/{name}/calibrate",
            headers=AUTH,
            json={"rawZero": zero, "rawMin": 0, "rawMax": maximum, "ratio": 1},
        )
        assert response.status_code == 200, response.text
    preview = client.post(
        "/api/robot/arm/plans/preview",
        headers=AUTH,
        json={"targets": targets or {"joint_2": 5.0}},
    )
    assert preview.status_code == 200, preview.text
    body = preview.json()
    return {"planId": body["planId"], "planDigest": body["planDigest"]}


def test_execute_and_capture_proves_arrival_and_returns_the_exact_bound_frame(
    tmp_path: Path,
) -> None:
    controller = _controller()
    jpeg = b"\xff\xd8bound-arm-frame\xff\xd9"
    provider = FakeProvider(_frame(jpeg))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

        assert response.status_code == 200, response.text
        result = response.json()
        assert result["executed"] is True
        assert result["outcome"] == "captured"
        assert result["arrival"]["proved"] is True
        assert result["arrival"]["samples"] >= 3
        assert result["arrival"]["spanMs"] >= 250
        assert result["capture"]["frameId"]
        assert result["capture"]["dimensions"] == {"width": 640, "height": 480}
        assert result["measuredPoseNearCapture"]["before"]["telemetryGeneration"]
        assert result["measuredPoseNearCapture"]["after"]["telemetryGeneration"]

        repeated = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        assert repeated.status_code == 200
        assert repeated.json() == result
        assert provider.capture_calls == 1
        assert provider.capture_profiles == ["detail"]

        frame = client.get(result["capture"]["transferUrl"], headers=AUTH)
        assert frame.status_code == 200
        assert frame.content == jpeg
        assert client.get(result["capture"]["transferUrl"], headers=AUTH).status_code == 404


def test_composed_plan_and_sequence_forward_the_requested_capture_profile(
    tmp_path: Path,
) -> None:
    plan_controller = _controller()
    plan_provider = FakeProvider(_frame(b"\xff\xd8survey-plan-frame\xff\xd9"))
    plan_path = tmp_path / "plan"
    plan_path.mkdir()
    with _client(plan_path, plan_controller, plan_provider) as client:
        plan = _calibrate_and_plan(client)
        captured = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500, "captureProfile": "survey"},
        )

    assert captured.status_code == 200, captured.text
    assert captured.json()["capture"]["captureProfile"] == "survey"
    assert plan_provider.capture_profiles == ["survey"]

    sequence_controller = _controller()
    sequence_provider = FakeProvider(_frame(b"\xff\xd8detail-sequence-frame\xff\xd9"))
    sequence_path = tmp_path / "sequence"
    sequence_path.mkdir()
    with _client(sequence_path, sequence_controller, sequence_provider) as client:
        _calibrate_and_plan(client)
        preview = client.post(
            "/api/robot/arm/sequences/preview",
            headers=AUTH,
            json={
                "waypoints": [
                    {"label": "first", "targets": {"joint_2": 4.0}},
                    {"label": "second", "targets": {"joint_2": 8.0}},
                ]
            },
        )
        assert preview.status_code == 200, preview.text
        payload = {
            "sequenceId": preview.json()["sequenceId"],
            "sequenceDigest": preview.json()["sequenceDigest"],
            "arrivalTimeoutMs": 2500,
        }
        motion_only = client.post(
            "/api/robot/arm/sequences/execute",
            headers=AUTH,
            json={**payload, "captureProfile": "survey"},
        )
        captured = client.post(
            "/api/robot/arm/sequences/execute-and-capture",
            headers=AUTH,
            json=payload,
        )

    assert motion_only.status_code == 422
    assert captured.status_code == 200, captured.text
    assert captured.json()["capture"]["captureProfile"] == "detail"
    assert sequence_provider.capture_profiles == ["detail"]


def test_base_arrival_uses_the_fresh_status_after_a_slow_guarded_odometer_read(
    tmp_path: Path,
) -> None:
    controller = _controller(SlowOdometerController)
    provider = FakeProvider(_frame(b"\xff\xd8slow-odometer-arrival\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client, {"joint_1": 5.0})
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 5000},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "captured"
    assert result["arrival"]["proved"] is True
    assert result["capture"] is not None
    assert provider.capture_calls == 1


def test_base_arrival_waits_for_a_fresh_odometer_after_a_valid_stale_sample(
    tmp_path: Path,
) -> None:
    controller = _controller(TransientlyStaleOdometerController)
    provider = FakeProvider(_frame(b"\xff\xd8stale-then-fresh-base-arrival\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client, {"joint_1": 5.0})
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 5000},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert controller.stale_post_dispatch_samples == 1
    assert result["outcome"] == "captured"
    assert result["arrival"]["proved"] is True
    assert result["capture"] is not None
    assert provider.capture_calls == 1


def test_base_arrival_times_out_when_valid_odometer_samples_stay_stale(
    tmp_path: Path,
) -> None:
    controller = _controller(PersistentlyStaleOdometerController)
    provider = FakeProvider(_frame(b"\xff\xd8persistently-stale-base-arrival\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client, {"joint_1": 5.0})
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 500},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert controller.stale_post_dispatch_samples >= 1
    assert result["outcome"] == "arrival_timeout"
    assert result["arrival"]["proved"] is False
    assert result["arrival"]["reason"] == "arrival_timeout"
    assert result["arrival"]["lastSampleReason"] == "joint_1_odometer_stale"
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_base_arrival_faults_when_counted_position_truth_is_lost(
    tmp_path: Path,
) -> None:
    controller = _controller(BaseTruthLostAfterDispatchController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-untrusted-base\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client, {"joint_1": 5.0})
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 5000},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert controller.invalid_post_dispatch_samples >= 1
    assert result["outcome"] == "arrival_fault"
    assert result["arrival"]["proved"] is False
    assert result["arrival"]["reason"] == "joint_1_position_untrusted"
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_execute_and_capture_times_out_without_taking_a_picture(
    tmp_path: Path,
) -> None:
    controller = _controller(NeverSettlesController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-be-used\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 500},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["executed"] is True
    assert result["outcome"] == "arrival_timeout"
    assert result["arrival"]["proved"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_execute_and_capture_returns_recoverable_abort_when_atomic_dispatch_fails(
    tmp_path: Path,
) -> None:
    controller = _controller(MoveSetFailsController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-after-failed-set\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        _calibrate_and_plan(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=AUTH,
            json={"targets": {"joint_2": 5.0, "joint_3": 5.0}},
        )
        assert preview.status_code == 200, preview.text
        preview_body = preview.json()
        plan = {
            "planId": preview_body["planId"],
            "planDigest": preview_body["planDigest"],
        }
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        repeated = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 409, response.text
    assert response.json()["detail"] == {
        "code": "MOVE_SET_FAILED",
        "phase": "dispatch",
        "motionMayHaveStarted": True,
        "partialDispatchPossible": False,
        "stopped": True,
        "torqueState": "unknown",
        "dispatchedFamilyCount": 1,
        "failedIndex": 0,
    }
    assert repeated.status_code == 409
    assert repeated.json()["detail"] == (
        "That plan is unavailable, expired, or already used. Prepare it again."
    )
    assert state["operatorInspectionRequired"] is False
    assert not any(command["operation"] == "STOP" for command in controller.commands)
    assert provider.capture_calls == 0


def test_execute_and_capture_returns_typed_link_failure_for_unconfirmed_atomic_receipt(
    tmp_path: Path,
) -> None:
    controller = _controller(MoveSetUnconfirmedController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-unconfirmed\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        _calibrate_and_plan(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=AUTH,
            json={"targets": {"joint_2": 4.0, "joint_3": 4.0}},
        ).json()
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={
                "planId": preview["planId"],
                "planDigest": preview["planDigest"],
                "arrivalTimeoutMs": 2500,
            },
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 502, response.text
    assert response.json() == {"detail": {"code": "CONTROLLER_LINK_UNHEALTHY"}}
    result = response.json()
    assert provider.capture_calls == 0
    assert state["operatorInspectionRequired"] is True
    assert state["safetyStopReason"] == "MOVE_SET_UNCONFIRMED"
    assert not any(command["operation"] == "STOP" for command in controller.commands)


def test_legacy_first_move_receipt_loss_drops_authority_before_next_move_or_capture(
    tmp_path: Path,
) -> None:
    controller = _controller(LegacyFirstMoveUnconfirmedController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-legacy-ambiguity\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        _calibrate_and_plan(client)
        preview = client.post(
            "/api/robot/arm/plans/preview",
            headers=AUTH,
            json={"targets": {"joint_2": -4.0, "joint_3": 4.0}},
        ).json()
        controller.commands.clear()
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={
                "planId": preview["planId"],
                "planDigest": preview["planDigest"],
                "arrivalTimeoutMs": 2500,
            },
        )
        state = client.get("/api/robot/arm/state", headers=AUTH).json()

    assert response.status_code == 502, response.text
    assert response.json() == {"detail": {"code": "CONTROLLER_LINK_UNHEALTHY"}}
    operations = [command["operation"] for command in controller.commands]
    assert operations.count("MOVE") == 1
    assert "STOP" not in operations
    assert provider.capture_calls == 0
    assert state["stopped"] is False
    assert state["operatorInspectionRequired"] is False


@pytest.mark.parametrize(
    "stop_reason",
    ["MOVE_SET_FAILED", "EXPLICIT_STOP", "SAFETY_FAULT"],
)
def test_preexisting_stop_latch_is_not_mislabeled_or_cached_as_executed(
    tmp_path: Path,
    stop_reason: str,
) -> None:
    controller = _controller(PreexistingInspectionController)
    provider = FakeProvider(_frame(b"\xff\xd8must-not-capture-preexisting-latch\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        controller.inspection_required = True
        controller.inspection_reason = stop_reason
        controller.commands.clear()
        first = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        replay = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert first.status_code == 409, first.text
    assert first.json()["detail"]["code"] == "OPERATOR_INSPECTION_REQUIRED"
    assert replay.status_code == 409, replay.text
    assert replay.json()["detail"]["code"] == "OPERATOR_INSPECTION_REQUIRED"
    assert replay.json().get("executed") is not True
    assert provider.capture_calls == 0
    assert not any(
        command["operation"]
        in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN", "STOP"}
        for command in controller.commands
    )


def test_execute_and_capture_discards_a_frame_if_the_arm_boot_changes_during_capture(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(
        _frame(b"\xff\xd8invalidated-frame\xff\xd9"),
        on_capture=controller.simulate_reboot,
    )

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        camera_status = client.get("/api/camera/status", headers=AUTH)

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["executed"] is True
    assert result["outcome"] == "capture_invalidated"
    assert result["arrival"]["proved"] is True
    assert result["capture"] is None
    assert provider.capture_calls == 1
    assert camera_status.json()["latestFrameId"] is None


def test_operator_stop_preempts_arrival_wait_and_never_captures(tmp_path: Path) -> None:
    controller = _controller(NeverSettlesController)
    provider = FakeProvider(_frame(b"\xff\xd8stop-means-no-frame\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        finished = threading.Event()
        received: dict[str, object] = {}

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/plans/execute-and-capture",
                headers=AUTH,
                json={**plan, "arrivalTimeoutMs": 5000},
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        thread = threading.Thread(target=execute)
        thread.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not any(
            command["operation"] == "MOVE" for command in controller.commands
        ):
            time.sleep(0.01)
        stopped = client.post("/api/robot/arm/stop", headers=AUTH)
        assert stopped.status_code == 200
        assert finished.wait(2.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "stopped"
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_stop_after_arrival_proof_wins_before_shutter_and_takes_no_picture(
    tmp_path: Path,
    monkeypatch,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8must-not-open-shutter\xff\xd9"))
    pose_proved = threading.Event()
    release_pose = threading.Event()
    original = ArmService._trusted_view_pose
    blocked_once = False

    def pause_after_first_trusted_pose(self, snapshot):
        nonlocal blocked_once
        result = original(self, snapshot)
        if result[0] is not None and not blocked_once:
            blocked_once = True
            pose_proved.set()
            assert release_pose.wait(3.0)
        return result

    monkeypatch.setattr(ArmService, "_trusted_view_pose", pause_after_first_trusted_pose)
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        finished = threading.Event()
        received: dict[str, object] = {}

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/plans/execute-and-capture",
                headers=AUTH,
                json={**plan, "arrivalTimeoutMs": 2500},
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        thread = threading.Thread(target=execute)
        thread.start()
        assert pose_proved.wait(3.0)
        stopped = client.post("/api/robot/arm/stop", headers=AUTH)
        assert stopped.status_code == 200, stopped.text
        release_pose.set()
        assert finished.wait(3.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "stopped"
    assert result["arrival"]["reason"] == "operator_stop_before_shutter"
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_stop_while_composed_capture_waits_for_camera_lock_never_opens_shutter(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8camera-lock-stop-race\xff\xd9"))
    preflight_complete = threading.Event()
    release_preflight = threading.Event()
    transfer_entered = threading.Event()

    class LockContendedCameraService(CameraEvidenceService):
        def transfer_available(self) -> bool:
            available = super().transfer_available()
            preflight_complete.set()
            assert release_preflight.wait(3.0)
            return available

        def capture_transfer(
            self,
            *,
            profile: str = CAPTURE_PROFILE_DETAIL,
            shutter_guard=None,
        ) -> dict[str, object]:
            transfer_entered.set()
            return super().capture_transfer(profile=profile, shutter_guard=shutter_guard)

    camera_service = LockContendedCameraService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
    )
    with _client_with_camera_service(tmp_path, controller, camera_service) as client:
        plan = _calibrate_and_plan(client)
        finished = threading.Event()
        received: dict[str, object] = {}

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/plans/execute-and-capture",
                headers=AUTH,
                json={**plan, "arrivalTimeoutMs": 2500},
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        # Model an ordinary capture/autofocus already owning the serialized
        # camera lane when the composed operation reaches its shutter step.
        thread = threading.Thread(target=execute)
        try:
            thread.start()
            assert preflight_complete.wait(3.0)
            camera_service._lock.acquire()
            release_preflight.set()
            assert transfer_entered.wait(3.0)
            stopped = client.post("/api/robot/arm/stop", headers=AUTH)
            assert stopped.status_code == 200, stopped.text
        finally:
            release_preflight.set()
            camera_service._lock.release()

        assert finished.wait(3.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "stopped"
    assert result["arrival"]["reason"] == "operator_stop_before_shutter"
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_uncommanded_camera_motion_during_shutter_invalidates_the_frame(
    tmp_path: Path,
) -> None:
    controller = _controller()

    def move_camera() -> None:
        controller._servos[4]["rawPosition"] = 620

    provider = FakeProvider(
        _frame(b"\xff\xd8camera-moved-during-shutter\xff\xd9"),
        on_capture=move_camera,
    )
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        status = client.get("/api/camera/status", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "capture_invalidated"
    assert result["arrival"]["proved"] is True
    assert result["capture"] is None
    assert status["latestFrameId"] is None


def test_untrusted_uncommanded_camera_prevents_the_shutter(tmp_path: Path) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8must-not-bind-to-unknown-pose\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        controller._servos.pop(4)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["executed"] is True
    assert result["arrival"]["proved"] is True
    assert result["outcome"] == "view_pose_untrusted"
    assert result["captureAttempted"] is False
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_untrusted_uncommanded_base_odometer_prevents_the_shutter(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8must-not-bind-without-base-truth\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        controller._odometers.pop(1, None)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["executed"] is True
    assert result["arrival"]["proved"] is True
    assert result["outcome"] == "view_pose_untrusted"
    assert result["viewPoseReason"] == "joint_1_view_pose_untrusted"
    assert result["capture"] is None
    assert provider.capture_calls == 0


def test_slow_uncommanded_camera_motion_during_shutter_invalidates_the_frame(
    tmp_path: Path,
) -> None:
    controller = _controller()

    def start_slow_camera_move() -> None:
        controller._servos[4]["rawPosition"] = 513
        controller._servos[4]["moving"] = True

    provider = FakeProvider(
        _frame(b"\xff\xd8slow-moving-camera\xff\xd9"),
        on_capture=start_slow_camera_move,
    )
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        status = client.get("/api/camera/status", headers=AUTH).json()

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "capture_invalidated"
    assert result["arrival"]["proved"] is True
    assert result["capture"] is None
    assert status["latestFrameId"] is None


def test_cached_capture_downgrades_after_its_one_use_transfer_is_consumed(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8bounded-history-frame\xff\xd9"))

    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        first = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        assert first.status_code == 200, first.text
        first_result = first.json()
        assert first_result["outcome"] == "captured"
        transfer_url = first_result["capture"]["transferUrl"]

        for _ in range(8):
            assert client.post("/api/camera/captures", headers=AUTH).status_code == 200
        assert client.get(transfer_url, headers=AUTH).status_code == 200
        motion_count = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )

        replay = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert replay.status_code == 200, replay.text
    result = replay.json()
    assert result["executed"] is True
    assert result["arrival"]["proved"] is True
    assert result["outcome"] == "capture_unavailable"
    assert result["capture"] is None
    assert result["captureUnavailableReason"] == "exact_frame_transfer_unavailable"
    assert len(
        [
            command
            for command in controller.commands
            if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        ]
    ) == motion_count
    assert provider.capture_calls == 9


def test_combined_transfer_survives_intervening_camera_history_captures(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8reserved-combined-frame\xff\xd9"))

    class InterveningCaptureService(CameraEvidenceService):
        def capture_transfer(
            self,
            *,
            profile: str = CAPTURE_PROFILE_DETAIL,
            shutter_guard=None,
        ) -> dict[str, object]:
            metadata = super().capture_transfer(
                profile=profile,
                shutter_guard=shutter_guard,
            )
            # Model unrelated captures landing after the combined shutter but
            # before its post-shutter physical telemetry proof finishes.
            for _ in range(3):
                super().capture(profile=profile)
            return metadata

    camera_service = InterveningCaptureService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
        max_frames=2,
    )
    with _client_with_camera_service(tmp_path, controller, camera_service) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

        assert response.status_code == 200, response.text
        result = response.json()
        assert result["outcome"] == "captured"
        exact_frame = client.get(result["capture"]["transferUrl"], headers=AUTH)

    assert exact_frame.status_code == 200
    assert exact_frame.content == b"\xff\xd8reserved-combined-frame\xff\xd9"
    assert provider.capture_calls == 4


def test_cached_provider_frame_outside_pose_bracket_is_invalidated(
    tmp_path: Path,
) -> None:
    controller = _controller()

    class StaleProvider(FakeProvider):
        def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame:
            self.capture_calls += 1
            self.capture_profiles.append(profile)
            return self.frame

    provider = StaleProvider(_frame(b"\xff\xd8stale-provider-frame\xff\xd9"))
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        response = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert response.status_code == 200, response.text
    result = response.json()
    assert result["outcome"] == "capture_invalidated"
    assert result["captureInvalidationReason"] == (
        "capture_timestamp_outside_pose_bracket"
    )
    assert result["capture"] is None


def test_full_transfer_lane_rejects_before_plan_execution_or_motion(
    tmp_path: Path,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8bounded-transfer\xff\xd9"))
    camera_service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
    )
    camera_service.start()
    held_transfers = [camera_service.capture_transfer() for _ in range(4)]

    with _client_with_camera_service(tmp_path, controller, camera_service) as client:
        plan = _calibrate_and_plan(client)
        motion_before = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )
        refused = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        motion_after = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )
        assert refused.status_code == 503, refused.text
        assert motion_after == motion_before
        assert provider.capture_calls == 4

        # Releasing one one-use handoff makes the same still-unconsumed plan
        # executable, proving the refusal happened before dispatch.
        assert client.get(held_transfers[0]["transferUrl"], headers=AUTH).status_code == 200
        retry = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert retry.status_code == 200, retry.text
    assert retry.json()["outcome"] == "captured"
    assert provider.capture_calls == 5


def test_full_transfer_byte_reservation_rejects_before_motion_with_count_slot_free(
    tmp_path: Path,
) -> None:
    controller = _controller()
    jpeg = b"\xff\xd8full-byte-pool\xff\xd9"
    provider = FakeProvider(_frame(jpeg))
    provider.max_frame_bytes = len(jpeg)
    camera_service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
        max_transfers=2,
        max_transfer_bytes=len(jpeg),
    )
    camera_service.start()
    held = camera_service.capture_transfer()
    assert camera_service.transfer_available() is False

    with _client_with_camera_service(tmp_path, controller, camera_service) as client:
        plan = _calibrate_and_plan(client)
        motion_before = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )
        refused = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        motion_after = len(
            [
                command
                for command in controller.commands
                if command["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
            ]
        )

        assert refused.status_code == 503, refused.text
        assert motion_after == motion_before
        assert provider.capture_calls == 1
        assert client.get(held["transferUrl"], headers=AUTH).status_code == 200
        retry = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert retry.status_code == 200, retry.text
    assert retry.json()["outcome"] == "captured"


@pytest.mark.parametrize("bound_case", ["missing", "invalid", "oversize"])
def test_untrusted_provider_frame_bound_rejects_before_motion_or_shutter(
    tmp_path: Path,
    bound_case: str,
) -> None:
    controller = _controller()
    provider = FakeProvider(_frame(b"\xff\xd8x\xff\xd9"))
    if bound_case == "missing":
        del provider.max_frame_bytes
    elif bound_case == "invalid":
        provider.max_frame_bytes = True
    else:
        provider.max_frame_bytes = 65
    camera_service = CameraEvidenceService(
        provider,
        status_callback=lambda: {"stateRevision": 1},
        max_transfer_bytes=64,
    )

    with _client_with_camera_service(tmp_path, controller, camera_service) as client:
        plan = _calibrate_and_plan(client)
        guarded_operations = {
            "RESET",
            "HOLD_SET",
            "MOVE",
            "MOVE_SET",
            "MOVE_MULTI_TURN",
        }
        before = [
            command
            for command in controller.commands
            if command["operation"] in guarded_operations
        ]
        refused = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        after = [
            command
            for command in controller.commands
            if command["operation"] in guarded_operations
        ]

    assert camera_service.transfer_available() is False
    assert refused.status_code == 503, refused.text
    assert after == before
    assert provider.capture_calls == 0


def test_composed_observation_requires_servo_family_before_consuming_arm_only_plan(
    tmp_path: Path,
) -> None:
    hide_family = True

    class MissingFamilyController(ReplayArmController):
        def transport_state(self) -> dict[str, object]:
            reported = super().transport_state()
            if hide_family:
                identity = reported.get("identity")
                capabilities = (
                    identity.get("capabilities")
                    if isinstance(identity, dict)
                    else None
                )
                if isinstance(capabilities, list):
                    identity["capabilities"] = [
                        value
                        for value in capabilities
                        if value != "servo_family"
                    ]
            return reported

    controller = _controller(MissingFamilyController)
    provider = FakeProvider(_frame(b"\xff\xd8family-proof-required\xff\xd9"))
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        controller.commands.clear()
        refused = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )
        commands_after_refusal = list(controller.commands)
        captures_after_refusal = provider.capture_calls

        # The capability refusal must not consume the one-use reviewed plan.
        hide_family = False
        retry = client.post(
            "/api/robot/arm/plans/execute-and-capture",
            headers=AUTH,
            json={**plan, "arrivalTimeoutMs": 2500},
        )

    assert refused.status_code == 409, refused.text
    assert refused.json()["detail"]["code"] == "SERVO_FAMILY_UNAVAILABLE"
    assert not any(
        command["operation"]
        in {"HOLD_SET", "MOVE", "MOVE_SET", "MOVE_MULTI_TURN"}
        for command in commands_after_refusal
    )
    assert captures_after_refusal == 0
    assert retry.status_code == 200, retry.text
    assert retry.json()["outcome"] == "captured"


def test_commissioning_mutation_is_excluded_while_physical_stop_still_preempts(
    tmp_path: Path,
) -> None:
    controller = _controller()
    capture_entered = threading.Event()
    release_capture = threading.Event()

    class BlockingProvider(FakeProvider):
        def capture(self, *, profile: str = CAPTURE_PROFILE_DETAIL) -> CapturedFrame:
            capture_entered.set()
            assert release_capture.wait(3.0)
            return super().capture(profile=profile)

    provider = BlockingProvider(_frame(b"\xff\xd8blocked-operation\xff\xd9"))
    with _client(tmp_path, controller, provider) as client:
        plan = _calibrate_and_plan(client)
        finished = threading.Event()
        received: dict[str, object] = {}

        def execute() -> None:
            response = client.post(
                "/api/robot/arm/plans/execute-and-capture",
                headers=AUTH,
                json={**plan, "arrivalTimeoutMs": 2500},
            )
            received["status"] = response.status_code
            received["body"] = response.json()
            finished.set()

        thread = threading.Thread(target=execute)
        thread.start()
        assert capture_entered.wait(3.0)

        blocked_move = client.post(
            "/api/robot/physical/arm/servos/move",
            headers=AUTH,
            json={
                "servoId": 2,
                "goal": 2200,
                "speed": 500,
                "acceleration": 20,
                "acknowledgedPhysicalPowerCut": True,
                "confirmedServoModel": "ST3215",
            },
        )
        stop_started = time.monotonic()
        stopped = client.post("/api/robot/physical/arm/stop", headers=AUTH)
        stop_elapsed = time.monotonic() - stop_started

        assert blocked_move.status_code == 409
        assert stopped.status_code == 200, stopped.text
        assert stop_elapsed < 0.5
        assert finished.is_set() is False
        release_capture.set()
        assert finished.wait(3.0)
        thread.join(timeout=1.0)

    assert received["status"] == 200
    result = received["body"]
    assert isinstance(result, dict)
    assert result["outcome"] == "capture_invalidated"
    assert result["capture"] is None
    assert not any(
        command["operation"] == "MOVE"
        and command.get("servoId") == 2
        and command.get("goal") == 2200
        for command in controller.commands
    )
