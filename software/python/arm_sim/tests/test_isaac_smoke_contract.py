"""Static contract tests for the Isaac smoke scene.

These run under ordinary Python; importing Isaac Sim is intentionally not
required for source validation.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SMOKE_PATH = ROOT / "python" / "arm_sim" / "isaac" / "smoke_scene.py"


class IsaacSmokeContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SMOKE_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(SMOKE_PATH))

    def test_source_parses(self) -> None:
        self.assertIsInstance(self.tree, ast.Module)

    def test_uses_current_experimental_rtx_api(self) -> None:
        self.assertIn("isaacsim.sensors.experimental.rtx", self.source)
        self.assertIn("RtxCamera", self.source)
        self.assertIn("CameraSensor", self.source)
        self.assertNotIn("isaacsim.sensors.camera", self.source)

    def test_robot_camera_is_mounted_below_optical_frame(self) -> None:
        self.assertIn("camera_optical_frame", self.source)
        self.assertIn('CAMERA_PATH = f"{CAMERA_MOUNT}/RGB"', self.source)
        self.assertIn("Imported arm is missing its camera optical frame", self.source)

    def test_module_3_wide_profile_owns_resolution_and_optics(self) -> None:
        self.assertIn("load_camera_profile", self.source)
        self.assertIn("CAMERA_PROFILE.survey.width_px", self.source)
        self.assertIn("CAMERA_PROFILE.survey.height_px", self.source)
        self.assertIn("projection.horizontal_aperture_mm", self.source)
        self.assertIn("projection.vertical_aperture_mm", self.source)
        self.assertNotIn('default=640', self.source)
        self.assertNotIn('default=480', self.source)

    def test_outputs_and_evidence_boundary_are_explicit(self) -> None:
        for filename in ("smoke_scene.usda", "smoke_rgb.png", "smoke_report.json"):
            self.assertIn(filename, self.source)
        self.assertIn('"physicalProof": False', self.source)
        self.assertIn("simulator pixels are not live-device", self.source)
        self.assertIn("CAMERA_PROFILE.report_metadata()", self.source)

    def test_scene_contains_desk_target_and_lights(self) -> None:
        for marker in ("/World/Desk", "/World/Target", "DomeLight", "RectLight"):
            self.assertIn(marker, self.source)

    def test_no_live_arm_or_network_imports(self) -> None:
        imported_roots = set()
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".")[0])
        self.assertTrue(
            imported_roots.isdisjoint(
                {"requests", "socket", "urllib", "robot_gateway", "arm_alliance"}
            )
        )


if __name__ == "__main__":
    unittest.main()
