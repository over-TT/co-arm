from __future__ import annotations

import hashlib
from pathlib import Path
from threading import Thread
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from arm_sim.bridge.client import BridgeClient
from arm_sim.bridge.engine import InMemoryBridgeEngine
from arm_sim.bridge.server import LoopbackBridgeServer
from arm_sim.bridge.sim_gateway import (
    _ensure_simulator_state,
    _install_simulator_health,
)
from robot_gateway.arm_controller import ArmController, ControllerCommandError
from robot_gateway.isaac_bridge import (
    IsaacArmController,
    IsaacCameraProvider,
    create_isaac_camera_service,
    default_servo_mappings,
)
from robot_gateway.pi_camera import CapturedFrame
from robot_gateway.runtime import create_app


TOKEN = "a" * 32


class IsaacBridgeAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.server = LoopbackBridgeServer(
            engine=InMemoryBridgeEngine(), token=TOKEN, port=0
        )
        self.thread = Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = BridgeClient(token=TOKEN, port=self.server.address[1])

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def test_arm_adapter_implements_protocol_and_translates_raw_goals(self) -> None:
        controller = IsaacArmController(self.client)
        self.assertIsInstance(controller, ArmController)
        controller.declare_native_multi_turn_servos([1])
        with self.assertRaises(ControllerCommandError) as mismatch:
            controller.declare_native_multi_turn_servos([2])
        self.assertEqual(mismatch.exception.code, "NATIVE_MULTI_TURN_ID_MISMATCH")
        started = controller.start()
        self.assertEqual(started["backendId"], "sim")
        self.assertTrue(started["simulated"])
        instance_id = started["backendInstanceId"]

        mappings = {mapping.servo_id: mapping for mapping in default_servo_mappings()}
        shoulder_goal = mappings[2].raw_from_degrees(-20.0)
        camera_goal = mappings[4].raw_from_degrees(15.0)
        controller.set_hold_servos([2, 4], 2_000)
        receipt = controller.move_set(
            [(2, shoulder_goal, 1_000, 20), (4, camera_goal, 400, 10)]
        )
        self.assertEqual(receipt["backendInstanceId"], instance_id)
        self.assertEqual(receipt["count"], 2)

        state = self.client.get_state().result
        self.assertAlmostEqual(
            state["jointPositionsDegrees"]["joint_2"],
            -20.0,
            delta=0.5 / mappings[2].ticks_per_degree,
        )
        self.assertAlmostEqual(
            state["jointPositionsDegrees"]["joint_4"],
            15.0,
            delta=0.5 / mappings[4].ticks_per_degree,
        )
        reported = controller.status()
        self.assertEqual(reported["motionState"], "ready")
        self.assertEqual(reported["torqueState"], "on")

        controller.stop()
        with self.assertRaises(ControllerCommandError) as raised:
            controller.move(2, shoulder_goal, 1_000, 20)
        self.assertEqual(raised.exception.code, "STOPPED")
        reset = controller.reset(inspected=True)
        self.assertEqual(reset["motionState"], "ready")

    def test_camera_adapter_returns_gateway_captured_frame(self) -> None:
        camera = IsaacCameraProvider(self.client)
        service = create_isaac_camera_service(camera)
        self.assertEqual(camera.last_state_revision, 0)
        self.assertEqual(camera.max_frame_bytes, 16 * 1024 * 1024)
        service.start()
        status = service.status()
        self.assertEqual(
            status["cameraProfile"]["id"],
            "rpi-camera-module-3-wide-imx708-nominal-v1",
        )
        frame = camera.capture()
        self.assertIsInstance(frame, CapturedFrame)
        self.assertEqual(frame.mime_type, "image/jpeg")
        self.assertEqual((frame.width, frame.height), (1280, 720))
        self.assertEqual(frame.sensor_model, "isaac-imx708-wide-provisional")
        self.assertEqual(frame.sha256, hashlib.sha256(frame.data).hexdigest())
        self.assertEqual(camera.backend_identity["backendId"], "sim")
        profile = camera.backend_identity["cameraProfile"]
        self.assertEqual(
            profile["id"],
            "rpi-camera-module-3-wide-imx708-nominal-v1",
        )
        with patch.object(
            self.client,
            "capture",
            wraps=self.client.capture,
        ) as bridge_capture:
            survey = camera.capture(profile="survey")
        bridge_capture.assert_called_once_with(profile="survey")
        self.assertEqual((survey.width, survey.height), (1280, 720))
        evidence = service.capture()
        self.assertTrue(evidence["simulated"])
        self.assertEqual(evidence["source"], "isaac_rgb")
        self.assertEqual(evidence["sensorModel"], "isaac-imx708-wide-provisional")
        self.assertEqual(
            evidence["cameraProfile"]["id"],
            "rpi-camera-module-3-wide-imx708-nominal-v1",
        )
        self.assertEqual(evidence["stateRevision"], camera.last_state_revision)
        attempt = camera.autofocus()
        self.assertFalse(attempt.attempted)
        self.assertEqual(attempt.result, "unsupported")
        self.assertEqual(attempt.autofocus.mode, "unknown")
        self.assertEqual(attempt.autofocus.state, "not_started")
        service.close()

    def test_existing_gateway_runtime_accepts_both_simulator_adapters(self) -> None:
        controller = IsaacArmController(self.client)
        camera = IsaacCameraProvider(self.client)
        camera_service = create_isaac_camera_service(camera)
        gateway_token = "g" * 32
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            token_file = root / "gateway.token"
            token_file.write_text(gateway_token + "\n", encoding="utf-8")
            state_dir = _ensure_simulator_state(root / "sim-state")
            app = create_app(
                token_file=token_file,
                arm_controller=controller,
                camera_service=camera_service,
                arm_state_dir=state_dir,
            )
            _install_simulator_health(app, self.client)
            headers = {"Authorization": f"Bearer {gateway_token}"}
            with TestClient(app) as http:
                health = http.get("/healthz")
                self.assertEqual(health.status_code, 200, health.text)
                self.assertEqual(health.json()["backendId"], "sim")
                self.assertEqual(
                    health.json()["backendInstanceId"],
                    self.client.backend_instance_id,
                )
                arm_state = http.get("/api/robot/arm/state", headers=headers)
                self.assertEqual(arm_state.status_code, 200, arm_state.text)
                self.assertEqual(arm_state.json()["connection"], "online")
                self.assertEqual(
                    arm_state.json()["controller"]["bootId"],
                    health.json()["backendInstanceId"],
                )
                self.assertTrue(
                    all(joint["calibrated"] for joint in arm_state.json()["joints"])
                )
                preview = http.post(
                    "/api/robot/arm/plans/preview",
                    headers=headers,
                    json={"targets": {"joint_4": 5.0}},
                )
                self.assertEqual(preview.status_code, 200, preview.text)
                plan = preview.json()
                execute = http.post(
                    "/api/robot/arm/plans/execute",
                    headers=headers,
                    json={
                        "planId": plan["planId"],
                        "planDigest": plan["planDigest"],
                    },
                )
                self.assertEqual(execute.status_code, 200, execute.text)
                self.assertTrue(execute.json()["executed"])
                self.assertAlmostEqual(
                    self.client.get_state().result["jointPositionsDegrees"][
                        "joint_4"
                    ],
                    5.0,
                    delta=0.5,
                )
                capture = http.post("/api/camera/captures", headers=headers)
                self.assertEqual(capture.status_code, 200, capture.text)
                self.assertTrue(capture.json()["simulated"])
                self.assertEqual(capture.json()["source"], "isaac_rgb")


if __name__ == "__main__":
    unittest.main()
