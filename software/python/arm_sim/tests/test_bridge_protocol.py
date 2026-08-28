from __future__ import annotations

import base64
import hashlib
import math
import unittest

from arm_sim.bridge.protocol import (
    PROTOCOL_NAME,
    ProtocolViolation,
    capture_result,
    validate_request,
    validate_result,
)
from arm_sim.camera_profile import load_camera_profile


TOKEN = "b" * 32


def request(command: str, params: dict[str, object]) -> dict[str, object]:
    return {
        "protocol": PROTOCOL_NAME,
        "requestId": "req_contract_123",
        "token": TOKEN,
        "command": command,
        "params": params,
    }


class BridgeProtocolTests(unittest.TestCase):
    def test_only_declared_commands_and_exact_fields_are_accepted(self) -> None:
        validated = validate_request(request("health", {}))
        self.assertEqual(validated.command, "health")

        with self.assertRaises(ProtocolViolation):
            validate_request({**request("health", {}), "path": "C:/private.usd"})
        with self.assertRaises(ProtocolViolation):
            validate_request(request("eval", {"expression": "1 + 1"}))
        with self.assertRaises(ProtocolViolation):
            validate_request(request("capture", {"path": "render.jpg"}))

    def test_capture_profile_is_bounded_and_defaults_to_detail(self) -> None:
        self.assertEqual(
            validate_request(request("capture", {})).params,
            {"profile": "detail"},
        )
        self.assertEqual(
            validate_request(request("capture", {"profile": "survey"})).params,
            {"profile": "survey"},
        )
        for profile in ("native", "DETAIL", "", None, 1):
            with self.subTest(profile=profile), self.assertRaises(ProtocolViolation):
                validate_request(request("capture", {"profile": profile}))

    def test_health_accepts_only_the_bounded_camera_profile_contract(self) -> None:
        result = {
            "status": "ok",
            "engine": "isaac_sim_6_rtx",
            "calibrationStatus": "provisional",
            "cameraProfile": load_camera_profile().report_metadata(),
        }
        validated = validate_result("health", result)
        self.assertEqual(
            validated["cameraProfile"]["profileId"],
            "rpi-camera-module-3-wide-imx708-nominal-v1",
        )
        self.assertEqual(
            validated["cameraProfile"]["projection"]["nominalFitAxis"],
            "horizontal",
        )
        tampered = dict(result)
        tampered_profile = dict(result["cameraProfile"])
        tampered_profile["privatePrimPath"] = "/World/Target"
        tampered["cameraProfile"] = tampered_profile
        with self.assertRaises(ProtocolViolation):
            validate_result("health", tampered)

    def test_joint_targets_reject_extra_ids_nonfinite_and_out_of_range(self) -> None:
        valid = validate_request(
            request(
                "set_joint_targets",
                {"targets": {"joint_2": -25.5}, "durationMs": 250},
            )
        )
        self.assertEqual(valid.params["targets"], {"joint_2": -25.5})

        for targets in (
            {"joint_5": 0.0},
            {"joint_2": math.inf},
            {"joint_2": -91.0},
            {},
        ):
            with self.subTest(targets=targets), self.assertRaises(ProtocolViolation):
                validate_request(
                    request(
                        "set_joint_targets",
                        {"targets": targets, "durationMs": 0},
                    )
                )

    def test_capture_builder_binds_bytes_to_digest(self) -> None:
        data = b"\xff\xd8test\xff\xd9"
        built = capture_result(
            frame_id="simframe_contract",
            data=data,
            width=640,
            height=480,
            captured_at="2026-08-18T10:00:00.000Z",
            state_revision=3,
            joint_positions_degrees={
                "joint_1": 1.0,
                "joint_2": 2.0,
                "joint_3": 3.0,
                "joint_4": 4.0,
            },
        )
        self.assertEqual(
            built["sha256"], f"sha256:{hashlib.sha256(data).hexdigest()}"
        )
        self.assertEqual(base64.b64decode(str(built["dataBase64"])), data)

        tampered = dict(built)
        tampered["dataBase64"] = base64.b64encode(b"different").decode("ascii")
        with self.assertRaises(ProtocolViolation):
            validate_result("capture", tampered)


if __name__ == "__main__":
    unittest.main()
