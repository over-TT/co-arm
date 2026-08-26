"""Standard-library contract tests for the primitive desk-arm URDF asset."""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ARM_SIM = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ARM_SIM / "config" / "arm_model.json"


def _floats(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.split())


class DeskCameraArmAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.urdf_path = (MANIFEST_PATH.parent / cls.manifest["urdf"]).resolve()
        cls.root = ET.parse(cls.urdf_path).getroot()
        cls.links = {node.attrib["name"]: node for node in cls.root.findall("link")}
        cls.joints = {node.attrib["name"]: node for node in cls.root.findall("joint")}

    def test_manifest_points_to_fixed_base_urdf(self) -> None:
        self.assertEqual(self.manifest["schema"], "arm-sim.model.v1")
        self.assertTrue(self.manifest["fixedBase"])
        self.assertTrue(self.urdf_path.is_file())
        self.assertEqual(self.root.tag, "robot")
        self.assertEqual(self.root.attrib["name"], self.manifest["modelId"])
        fixed = self.joints["base_fixed_joint"]
        self.assertEqual(fixed.attrib["type"], "fixed")
        self.assertEqual(fixed.find("parent").attrib["link"], "world")
        self.assertEqual(fixed.find("child").attrib["link"], "base_link")

    def test_link_and_joint_topology_is_explicit(self) -> None:
        expected_links = {
            "world",
            "base_link",
            "shoulder_mount_link",
            "upper_arm_link",
            "forearm_link",
            "tool_offset_link",
            "camera_link",
            "camera_optical_frame",
        }
        self.assertEqual(set(self.links), expected_links)
        revolute = {
            name for name, joint in self.joints.items() if joint.attrib["type"] == "revolute"
        }
        self.assertEqual(
            revolute,
            {
                "base_yaw_joint",
                "shoulder_pitch_joint",
                "elbow_pitch_joint",
                "camera_pitch_joint",
            },
        )
        self.assertEqual(len(self.joints), 7)
        manifest_joint_names = {joint["name"] for joint in self.manifest["joints"]}
        self.assertEqual(manifest_joint_names, revolute)

    def test_measured_dimensions_match_urdf_chain(self) -> None:
        dimensions = self.manifest["kinematics"]["dimensionsM"]
        base_origin = _floats(self.joints["base_yaw_joint"].find("origin").attrib["xyz"])
        elbow_origin = _floats(self.joints["elbow_pitch_joint"].find("origin").attrib["xyz"])
        tool_origin = _floats(self.joints["tool_offset_fixed_joint"].find("origin").attrib["xyz"])
        camera_origin = _floats(self.joints["camera_pitch_joint"].find("origin").attrib["xyz"])

        self.assertAlmostEqual(base_origin[2], dimensions["basePivotHeight"])
        self.assertAlmostEqual(elbow_origin[2], dimensions["upperArm"])
        self.assertAlmostEqual(tool_origin[0], dimensions["forearm"])
        self.assertAlmostEqual(camera_origin[0], dimensions["toolOffset"])
        self.assertAlmostEqual(
            tool_origin[0] + camera_origin[0], dimensions["elbowToCameraPivot"]
        )
        self.assertAlmostEqual(
            dimensions["upperArm"] + dimensions["elbowToCameraPivot"],
            dimensions["maximumPlanarReachFromShoulder"],
        )

        dimensions_mm = self.manifest["kinematics"]["dimensionsMm"]
        for key, value_m in dimensions.items():
            self.assertAlmostEqual(dimensions_mm[key] / 1000.0, value_m)

    def test_measured_dimensions_stay_in_sync_with_gateway_constants(self) -> None:
        gateway_path = ARM_SIM.parent / "robot_gateway" / "simple_arm_api.py"
        module = ast.parse(gateway_path.read_text(encoding="utf-8"))
        wanted = {"BASE_HEIGHT_MM", "UPPER_ARM_MM", "DISTAL_MM", "FLOOR_MM"}
        constants: dict[str, float] = {}
        for statement in module.body:
            if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
                continue
            target = statement.targets[0]
            if isinstance(target, ast.Name) and target.id in wanted:
                constants[target.id] = float(ast.literal_eval(statement.value))

        self.assertEqual(set(constants), wanted)
        dimensions = self.manifest["kinematics"]["dimensionsMm"]
        self.assertEqual(dimensions["basePivotHeight"], constants["BASE_HEIGHT_MM"])
        self.assertEqual(dimensions["upperArm"], constants["UPPER_ARM_MM"])
        self.assertEqual(dimensions["elbowToCameraPivot"], constants["DISTAL_MM"])
        self.assertEqual(dimensions["defaultFloorKeepOut"], constants["FLOOR_MM"])
        self.assertEqual(
            dimensions["forearm"] + dimensions["toolOffset"], constants["DISTAL_MM"]
        )

    def test_joint_axes_and_limits_match_manifest(self) -> None:
        for declared in self.manifest["joints"]:
            joint = self.joints[declared["name"]]
            self.assertEqual(joint.attrib["type"], declared["type"])
            self.assertEqual(_floats(joint.find("axis").attrib["xyz"]), tuple(declared["axis"]))
            limit = joint.find("limit")
            self.assertAlmostEqual(float(limit.attrib["lower"]), declared["lowerRad"])
            self.assertAlmostEqual(float(limit.attrib["upper"]), declared["upperRad"])
            self.assertGreater(float(limit.attrib["effort"]), 0.0)
            self.assertGreater(float(limit.attrib["velocity"]), 0.0)
            self.assertEqual(declared["limitStatus"], "provisional")

        calibrated = self.manifest["realSimCalibration"]["parameters"]["jointLimitsDeg"]
        for declared in self.manifest["joints"]:
            lower_deg, upper_deg = calibrated[declared["servoId"]]
            self.assertAlmostEqual(math.radians(lower_deg), declared["lowerRad"])
            self.assertAlmostEqual(math.radians(upper_deg), declared["upperRad"])
        self.assertEqual(calibrated["status"], "provisional_simulation_envelope")

    def test_camera_optical_frame_is_parent_ready(self) -> None:
        frames = self.manifest["frames"]
        self.assertEqual(self.manifest["cameraProfile"], "camera_profile.json")
        self.assertEqual(frames["cameraSensorParent"], "camera_optical_frame")
        optical = self.joints["camera_optical_joint"]
        self.assertEqual(optical.attrib["type"], "fixed")
        self.assertEqual(optical.find("parent").attrib["link"], "camera_link")
        self.assertEqual(optical.find("child").attrib["link"], "camera_optical_frame")
        origin = optical.find("origin")
        self.assertEqual(
            _floats(origin.attrib["xyz"]),
            tuple(frames["cameraOpticalFixedTranslationM"]),
        )
        actual_rpy = _floats(origin.attrib["rpy"])
        for actual, expected in zip(actual_rpy, frames["cameraOpticalFixedRpyRad"]):
            self.assertAlmostEqual(actual, expected)

        extrinsics = self.manifest["realSimCalibration"]["parameters"][
            "cameraExtrinsics"
        ]
        self.assertIsNone(extrinsics["translationM"])
        self.assertIsNone(extrinsics["rpyRad"])
        self.assertEqual(extrinsics["status"], "required_from_final_physical_mount")

    def test_physical_links_have_positive_provisional_inertia(self) -> None:
        for name, link in self.links.items():
            if name in {"world", "camera_optical_frame"}:
                continue
            self.assertIsNotNone(link.find("visual"), name)
            self.assertIsNotNone(link.find("collision"), name)
            inertial = link.find("inertial")
            self.assertIsNotNone(inertial, name)
            self.assertGreater(float(inertial.find("mass").attrib["value"]), 0.0, name)
            inertia = inertial.find("inertia").attrib
            for diagonal in ("ixx", "iyy", "izz"):
                self.assertGreater(float(inertia[diagonal]), 0.0, f"{name}.{diagonal}")

        provisional = set(self.manifest["evidenceBoundary"]["provisional"])
        self.assertIn("link masses, centers of mass, and inertias", provisional)
        self.assertIn("joint travel limits", provisional)
        self.assertIn("camera body dimensions", provisional)
        self.assertIn(
            "camera intrinsics, lens distortion, optical center, and mount extrinsics",
            provisional,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
