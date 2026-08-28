from __future__ import annotations

import ast
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from arm_sim.bridge import sim_gateway
from arm_sim.isaac import scene_factory
from robot_gateway.simple_arm_api import JOINT_IDS, JointStore


ROOT = Path(__file__).resolve().parents[3]
ISAAC_ENTRYPOINT = ROOT / "python" / "arm_sim" / "isaac" / "bridge_server.py"
SCENE_FACTORY = ROOT / "python" / "arm_sim" / "isaac" / "scene_factory.py"
GATEWAY_ENTRYPOINT = ROOT / "python" / "arm_sim" / "bridge" / "sim_gateway.py"
VIEW_ENTRYPOINT = ROOT / "python" / "arm_sim" / "isaac" / "view_scene.py"


class BridgeEntrypointContractTests(unittest.TestCase):
    def test_isaac_entrypoint_is_one_persistent_app_over_existing_scene(self) -> None:
        source = ISAAC_ENTRYPOINT.read_text(encoding="utf-8")
        tree = ast.parse(source)
        self.assertEqual(source.count("SimulationApp("), 1)
        self.assertIn("open_stage(str(scene_path))", source)
        self.assertIn("DEFAULT_SCENE_USD", source)
        self.assertIn("smoke_scene_is_current(scene_path)", source)
        self.assertIn("build_smoke_scene(self._app, scene_path)", source)
        self.assertIn("RtxCamera(", source)
        self.assertIn("CameraSensor(", source)
        self.assertIn("CAMERA_PROFILE.detail.width_px", source)
        self.assertIn("CAMERA_PROFILE.detail.height_px", source)
        self.assertIn('def capture(self, profile: str = "detail")', source)
        self.assertIn("reset_xform_op_properties=False", source)
        self.assertIn("camera_xform.GetOrderedXformOps()", source)
        self.assertIn("camera_xform.SetXformOpOrder", source)
        self.assertIn("Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0))", source)
        self.assertIn("cv2.imencode", source)
        self.assertIn("cv2.imdecode", source)
        decode_index = source.index("decoded = cv2.imdecode")
        decoded_quality_index = source.index(
            "decoded_quality = assess_frame_quality(decoded)"
        )
        self.assertGreater(decoded_quality_index, decode_index)
        self.assertIn("if not decoded_quality.passed", source)
        self.assertIn("np.max(np.abs(desired - current))", source)
        self.assertIn("interpolation_step_count(", source)
        self.assertIn('set_joint_targets(targets={"joint_4": 5.0}', source)
        self.assertIn('engine._set_positions_immediate({"joint_4": 40.0})', source)
        self.assertIn("os._exit(exit_code)", source)
        self.assertIn("LoopbackBridgeServer(", source)
        self.assertIn('BRIDGE_HOST = "127.0.0.1"', source)
        self.assertIn("BRIDGE_PORT = 8790", source)
        self.assertIn("--token-file", source)
        self.assertIn(
            "service_action=None if ARGS.headless else _service_visible_application",
            source,
        )
        self.assertIn(
            "poll_interval=0.1 if ARGS.headless else GUI_SERVER_POLL_SECONDS",
            source,
        )
        self.assertIn("SIMULATION_APP.update()", source)

        engine = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "IsaacSimBridgeEngine"
        )
        methods = {
            node.name for node in engine.body if isinstance(node, ast.FunctionDef)
        }
        self.assertTrue(
            {"health", "reset", "get_state", "set_joint_targets", "capture"}
            <= methods
        )
        for forbidden in (
            "robot_gateway",
            "subprocess",
            "requests.",
            "serial.",
            "eval(",
            "exec(",
        ):
            self.assertNotIn(forbidden, source)

    def test_clean_checkout_scene_factory_is_deterministic_and_single_app(self) -> None:
        source = SCENE_FACTORY.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn("SimulationApp(", source)
        self.assertIn("URDFImporterConfig()", source)
        self.assertIn("config.merge_fixed_joints = False", source)
        self.assertNotIn("np.random", source)
        self.assertIn('UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")', source)
        self.assertIn('UsdGeom.Cylinder.Define(stage, "/World/Target")', source)
        self.assertIn("UsdPhysics.MassAPI.Apply(target.GetPrim()).CreateMassAttr(0.014)", source)
        self.assertIn("UsdGeom.Camera.Define(stage, CAMERA_PATH)", source)
        self.assertIn('camera.GetPrim().ApplyAPI("OmniSensorAPI")', source)
        self.assertIn("stage.GetRootLayer().Export", source)
        self.assertIn("source_digest_before = scene_source_digest()", source)
        self.assertIn("Simulator authored inputs changed during scene generation", source)
        self.assertIn("robotArtifactDigest", source)
        for forbidden in ("subprocess", "eval(", "exec(", "robot_gateway"):
            self.assertNotIn(forbidden, source)

    def test_gateway_entrypoint_has_fixed_separate_loopback_endpoints(self) -> None:
        source = GATEWAY_ENTRYPOINT.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('GATEWAY_HOST = "127.0.0.1"', source)
        self.assertIn("GATEWAY_PORT = 8788", source)
        self.assertIn('BRIDGE_HOST = "127.0.0.1"', source)
        self.assertIn("BRIDGE_PORT = 8790", source)
        self.assertIn("--bridge-token-file", source)
        self.assertIn("--gateway-token-file", source)
        self.assertIn("--state-dir", source)
        self.assertIn("create_isaac_camera_service", source)
        self.assertNotIn("isaacsim", source)

    def test_manual_scene_viewer_rebuilds_missing_or_stale_scene(self) -> None:
        source = VIEW_ENTRYPOINT.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn("if not SCENE.is_file()", source)
        self.assertIn('os.environ.get("ISAAC_EXPERIENCE")', source)
        self.assertIn('"--experience"', source)
        self.assertIn("EXPERIENCE = ARGS.experience.resolve()", source)
        self.assertIn("experience=str(EXPERIENCE)", source)
        self.assertIn("smoke_scene_is_current(SCENE)", source)
        self.assertIn("build_smoke_scene(APP, SCENE)", source)

    def test_simulator_state_is_deterministic_calibrated_and_provisional(self) -> None:
        with TemporaryDirectory() as temporary:
            state_dir = sim_gateway._ensure_simulator_state(Path(temporary))
            document = json.loads(
                (state_dir / sim_gateway.SIM_STATE_FILENAME).read_text(encoding="utf-8")
            )
            self.assertEqual(document["_meta"]["backendId"], "sim")
            self.assertTrue(document["_meta"]["simulated"])
            self.assertEqual(document["_meta"]["calibrationStatus"], "provisional")

            store = JointStore(state_dir)
            for joint_id in JOINT_IDS:
                joint = store.get(joint_id)
                self.assertIsNotNone(joint.rawZero)
                self.assertIsNotNone(joint.rawMin)
                self.assertIsNotNone(joint.rawMax)

            # A second launch verifies rather than rewriting the mapping.
            self.assertEqual(sim_gateway._ensure_simulator_state(state_dir), state_dir)

    def test_scene_provenance_rejects_missing_corrupt_and_stale_artifacts(self) -> None:
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = root / "smoke_scene.usda"
            robot = root / "desk_camera_arm" / "robot.usda"
            robot.parent.mkdir()
            scene.write_text("#usda 1.0\n", encoding="utf-8")
            robot.write_text("#usda 1.0\n", encoding="utf-8")
            sidecar = scene_factory.scene_provenance_path(scene)

            self.assertFalse(scene_factory.smoke_scene_is_current(scene))
            sidecar.write_text("not-json", encoding="utf-8")
            self.assertFalse(scene_factory.smoke_scene_is_current(scene))

            current = {
                "schema": scene_factory.PROVENANCE_SCHEMA,
                "sourceDigest": scene_factory.scene_source_digest(),
                "sceneFile": scene.name,
                "sceneSha256": scene_factory._file_digest(scene),
                "robotAsset": robot.relative_to(root).as_posix(),
                "robotArtifactRoot": robot.parent.relative_to(root).as_posix(),
                "robotArtifactDigest": scene_factory._tree_digest(robot.parent),
            }
            sidecar.write_text(json.dumps(current), encoding="utf-8")
            self.assertTrue(scene_factory.smoke_scene_is_current(scene))

            scene.write_text("#usda 1.0\n# tampered\n", encoding="utf-8")
            self.assertFalse(scene_factory.smoke_scene_is_current(scene))
            scene.write_text("#usda 1.0\n", encoding="utf-8")
            robot.write_text("#usda 1.0\n# tampered\n", encoding="utf-8")
            self.assertFalse(scene_factory.smoke_scene_is_current(scene))
            robot.write_text("#usda 1.0\n", encoding="utf-8")
            self.assertTrue(scene_factory.smoke_scene_is_current(scene))

            current["sourceDigest"] = "0" * 64
            sidecar.write_text(json.dumps(current), encoding="utf-8")
            self.assertFalse(scene_factory.smoke_scene_is_current(scene))

            current["sourceDigest"] = scene_factory.scene_source_digest()
            current["robotAsset"] = "../escaped.usda"
            sidecar.write_text(json.dumps(current), encoding="utf-8")
            self.assertFalse(scene_factory.smoke_scene_is_current(scene))

    def test_scene_source_digest_includes_future_transitive_assets(self) -> None:
        with TemporaryDirectory() as temporary:
            assets = Path(temporary)
            (assets / "desk_camera_arm.urdf").write_text(
                '<robot><mesh filename="meshes/link.stl"/></robot>\n',
                encoding="utf-8",
            )
            mesh = assets / "meshes" / "link.stl"
            mesh.parent.mkdir()
            mesh.write_bytes(b"solid first")
            with patch.object(scene_factory, "ASSET_ROOT", assets):
                before = scene_factory.scene_source_digest()
                mesh.write_bytes(b"solid second")
                after = scene_factory.scene_source_digest()
            self.assertNotEqual(before, after)

    def test_scene_source_digest_includes_camera_profile(self) -> None:
        with TemporaryDirectory() as temporary:
            profile = Path(temporary) / "camera_profile.json"
            profile.write_text('{"version":1}\n', encoding="utf-8")
            with patch.object(scene_factory, "CAMERA_PROFILE_PATH", profile):
                before = scene_factory.scene_source_digest()
                profile.write_text('{"version":2}\n', encoding="utf-8")
                after = scene_factory.scene_source_digest()
            self.assertNotEqual(before, after)

    def test_mismatched_state_is_refused(self) -> None:
        with TemporaryDirectory() as temporary:
            state_dir = Path(temporary)
            (state_dir / sim_gateway.SIM_STATE_FILENAME).write_text(
                '{"joint_1":{"rawZero":999}}\n', encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "does not match"):
                sim_gateway._ensure_simulator_state(state_dir)


if __name__ == "__main__":
    unittest.main()
