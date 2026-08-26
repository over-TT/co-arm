r"""Run one persistent Isaac Sim 6 process behind the strict Arm bridge.

Persistent mode requires an explicit simulator-only token file and binds the
fixed endpoint ``127.0.0.1:8790``.  The process opens the generated smoke scene
once, wraps its existing four-DOF articulation and robot-mounted RTX camera,
then services the five authenticated public commands defined by
``arm_sim.bridge.protocol``.

Run with Isaac's Python, not the project's ordinary interpreter::

    C:\isaacsim\python.bat arm_sim\isaac\bridge_server.py \
        --token-file runtime\arm-sim\bridge.token

``--self-test`` exercises the same engine without opening a socket or requiring
a token.  It is simulator proof only and never touches the Raspberry Pi arm.
"""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from arm_sim.camera_profile import load_camera_profile


CAMERA_PROFILE = load_camera_profile()
RUNTIME_ROOT = PROJECT_ROOT.parent / "runtime" / "arm-sim"
DEFAULT_SCENE_USD = RUNTIME_ROOT / "smoke_scene.usda"
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 8790
GUI_SERVER_POLL_SECONDS = 1.0 / 60.0
ARTICULATION_ROOT = "/World/DeskCameraArm/Geometry/world"
CAMERA_PATH = (
    "/World/DeskCameraArm/Geometry/world/base_link/shoulder_mount_link/"
    "upper_arm_link/forearm_link/tool_offset_link/camera_link/"
    "camera_optical_frame/SmokeCameraMount/RGB"
)
TARGET_PATH = "/World/Target"
JOINT_TO_DOF = {
    "joint_1": "base_yaw_joint",
    "joint_2": "shoulder_pitch_joint",
    "joint_3": "elbow_pitch_joint",
    "joint_4": "camera_pitch_joint",
}
PHYSICS_HZ = 60
ARRIVAL_TOLERANCE_DEGREES = 1.0
MAX_SETTLE_STEPS = 300
RESET_MAX_SETTLE_STEPS = 240
RESET_STABLE_STEPS = 30
RESET_RENDER_UPDATES = 24
RESET_OVERVIEW_CAMERA_DEGREES = 55.0
RESET_OVERVIEW_MIN_TARGET_PIXELS = 64
RESET_OVERVIEW_MIN_CATCH_PIXELS = 256
CAPTURE_RENDER_UPDATES = 90
CAPTURE_HEIGHT = CAMERA_PROFILE.detail.height_px
CAPTURE_WIDTH = CAMERA_PROFILE.detail.width_px


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Host the persistent four-axis Arm Isaac simulator bridge."
    )
    parser.add_argument(
        "--token-file",
        type=Path,
        help="explicit simulator-only bridge token file (required in server mode)",
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE_USD)
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="initialize, reset, capture a JPEG, print bounded evidence, and exit",
    )
    args = parser.parse_known_args()[0]
    if not args.self_test and args.token_file is None:
        parser.error("--token-file is required in persistent server mode")
    return args


ARGS = _parse_args()
SCENE_USD = ARGS.scene.resolve()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "width": CAPTURE_WIDTH,
        "height": CAPTURE_HEIGHT,
        "multi_gpu": False,
    }
)

# Kit/Omniverse imports must occur after the single SimulationApp is created.
import cv2
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from omni.physics.core import ContactEventType, get_physics_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics

from arm_sim.bridge.engine import BridgeEngine
from arm_sim.bridge.protocol import JOINT_IDS, capture_result
from arm_sim.bridge.server import LoopbackBridgeServer
from arm_sim.build_identity import simulator_source_digest
from arm_sim.isaac.scene_factory import (
    build_smoke_scene,
    smoke_scene_is_current,
)
def _timestamp() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _init_phase(name: str) -> None:
    print(f"[arm-sim:init] {name}", flush=True)


def _failure_exit_code(scope: str, error: BaseException) -> int:
    print(
        f"[arm-sim] {scope} failed: {type(error).__name__}: {error!r}",
        file=sys.stderr,
        flush=True,
    )
    if isinstance(error, KeyboardInterrupt):
        return 130
    if isinstance(error, SystemExit) and isinstance(error.code, int) and error.code:
        return error.code
    return 1


class _VisibleApplicationClosed(Exception):
    """Signal that the operator closed the GUI-mode Kit application."""


def _service_visible_application() -> None:
    if not SIMULATION_APP.is_running():
        raise _VisibleApplicationClosed
    SIMULATION_APP.update()


def _read_token_file(path: Path) -> str:
    resolved = path.resolve()
    if not resolved.is_file():
        raise ValueError("Simulator bridge token file is unavailable.")
    try:
        with resolved.open("r", encoding="utf-8") as handle:
            token = handle.read(258).strip()
    except (OSError, UnicodeError):
        raise ValueError("Simulator bridge token file is unreadable.") from None
    if (
        not 32 <= len(token) <= 256
        or any(character.isspace() or not character.isprintable() for character in token)
    ):
        raise ValueError(
            "Simulator bridge token must contain 32 to 256 printable non-space characters."
        )
    return token


def _rgb_to_u8(raw_rgb: Any) -> np.ndarray:
    raw = np.asarray(raw_rgb)[:, :, :3]
    clean = np.nan_to_num(raw, nan=0.0, posinf=255.0, neginf=0.0)
    if np.issubdtype(clean.dtype, np.floating):
        maximum = float(clean.max()) if clean.size else 0.0
        minimum = float(clean.min()) if clean.size else 0.0
        if minimum >= -0.01 and maximum <= 1.01:
            clean = clean * 255.0
    elif np.issubdtype(clean.dtype, np.integer) and clean.size and int(clean.max()) <= 1:
        clean = clean * 255
    return np.rint(np.clip(clean, 0.0, 255.0)).astype(np.uint8)


class IsaacSimBridgeEngine(BridgeEngine):
    """Physical-equivalent engine backed by the already-open Isaac stage."""

    def __init__(self, application: SimulationApp, scene_path: Path) -> None:
        self._app = application
        _init_phase("scene-check")
        if not smoke_scene_is_current(scene_path):
            _init_phase("scene-build-begin")
            build_smoke_scene(self._app, scene_path)
            _init_phase("scene-build-complete")
        # Capture the identity of the complete loaded simulator stack once at
        # engine initialization.  This is intentionally the same canonical
        # source-set digest used by the supervisor, not merely the generated
        # scene's authored-input digest.
        self._provenance_sha256 = simulator_source_digest(PROJECT_ROOT)
        self._revision = 0
        self._moving = False
        self._stopped = False
        self._last_seed = 7

        _init_phase("stage-open-begin")
        opened, stage = open_stage(str(scene_path))
        _init_phase("stage-open-returned")
        if not opened or stage is None:
            raise RuntimeError("Isaac could not open the generated arm scene")
        while is_stage_loading():
            self._app.update()
        _init_phase("stage-load-complete")
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac returned no active USD stage")
        for prim_path in (ARTICULATION_ROOT, CAMERA_PATH):
            if not self._stage.GetPrimAtPath(prim_path).IsValid():
                raise RuntimeError("Generated arm scene is missing a required prim")
        _init_phase("required-prims-valid")

        SimulationManager.set_physics_dt(1.0 / PHYSICS_HZ)
        _init_phase("physics-configured")
        self._articulation = Articulation(ARTICULATION_ROOT)
        _init_phase("articulation-created")
        # The generated smoke scene aimed this nested camera with a world-space
        # look-at transform.  That image is valid at the authored zero pose but
        # becomes a bad local transform when the parent articulation moves.
        # Normalize the existing camera beneath the URDF optical frame instead:
        # ROS optical +Z is forward/+Y down, while USD Camera -Z is forward/+Y
        # up, so a local 180-degree X rotation is the exact frame conversion.
        camera_xform = UsdGeom.Xformable(self._stage.GetPrimAtPath(CAMERA_PATH))
        existing_ops = {
            operation.GetOpType(): operation
            for operation in camera_xform.GetOrderedXformOps()
        }
        translate_op = existing_ops.get(UsdGeom.XformOp.TypeTranslate)
        if translate_op is None:
            translate_op = camera_xform.AddTranslateOp(
                UsdGeom.XformOp.PrecisionDouble
            )
        orient_op = existing_ops.get(UsdGeom.XformOp.TypeOrient)
        if orient_op is None:
            orient_op = camera_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        scale_op = existing_ops.get(UsdGeom.XformOp.TypeScale)
        if scale_op is None:
            scale_op = camera_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        translate_op.Set(
            Gf.Vec3d(0.0, 0.0, 0.0)
            if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble
            else Gf.Vec3f(0.0, 0.0, 0.0)
        )
        orient_op.Set(
            Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0))
            if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble
            else Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0))
        )
        scale_op.Set(
            Gf.Vec3d(1.0, 1.0, 1.0)
            if scale_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble
            else Gf.Vec3f(1.0, 1.0, 1.0)
        )
        camera_xform.SetXformOpOrder([translate_op, orient_op, scale_op])
        _init_phase("camera-transform-normalized")
        camera = RtxCamera(
            CAMERA_PATH,
            tick_rate=None,
            reset_xform_op_properties=False,
        )
        self._sensor = CameraSensor(
            camera,
            resolution=(CAPTURE_HEIGHT, CAPTURE_WIDTH),
            annotators=["rgb"],
        )
        _init_phase("camera-sensor-created")

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        for _ in range(4):
            self._step()
        _init_phase("timeline-warmup-complete")
        available = list(self._articulation.dof_names)
        expected = list(JOINT_TO_DOF.values())
        if set(expected) != set(available) or len(available) != 4:
            raise RuntimeError("Imported arm articulation does not expose the expected four DOFs")
        self._dof_indices = [available.index(name) for name in expected]
        _init_phase("articulation-dofs-valid")
        self._set_positions_immediate(
            {joint_id: 0.0 for joint_id in JOINT_IDS},
        )
        _init_phase("initial-pose-applied")

    def health(self) -> dict[str, object]:
        if not self._app.is_running() or not self._timeline.is_playing():
            raise RuntimeError("Isaac simulation is not running")
        return {
            "status": "ok",
            "engine": "isaac_sim_6_rtx",
            "calibrationStatus": "provisional",
            "cameraProfile": CAMERA_PROFILE.report_metadata(),
        }

    def reset(self, *, seed: int | None) -> dict[str, object]:
        self._last_seed = self._last_seed if seed is None else seed
        self._stopped = False
        self._moving = False
        self._revision += 1
        self._set_positions_immediate({joint_id: 0.0 for joint_id in JOINT_IDS})
        return self._state()

    def get_state(self) -> dict[str, object]:
        return self._state()

    def set_joint_targets(
        self, *, targets: Mapping[str, float], duration_ms: int
    ) -> dict[str, object]:
        if self._stopped:
            raise RuntimeError("simulator is stopped")
        current = self._read_all_dof_radians()
        desired = current.copy()
        for joint_id, degrees in targets.items():
            desired[self._dof_indices[JOINT_IDS.index(joint_id)]] = math.radians(
                float(degrees)
            )
        interpolation_steps = max(
            1,
            int(math.ceil(duration_ms * PHYSICS_HZ / 1000.0)),
        )
        self._revision += 1
        self._moving = True
        try:
            for step in range(1, interpolation_steps + 1):
                blend = step / interpolation_steps
                command = current + (desired - current) * blend
                self._articulation.set_dof_position_targets(command)
                self._step()
            tolerance = math.radians(ARRIVAL_TOLERANCE_DEGREES)
            for _ in range(MAX_SETTLE_STEPS):
                measured = self._read_all_dof_radians()
                if float(np.max(np.abs(measured - desired))) <= tolerance:
                    break
                self._articulation.set_dof_position_targets(desired)
                self._step()
            else:
                raise RuntimeError("Isaac articulation did not reach its target")
        finally:
            self._moving = False
        return self._state()

    def capture(self, profile: str = "detail") -> dict[str, object]:
        render_profile = CAMERA_PROFILE.render(profile)
        if (
            render_profile.width_px != CAPTURE_WIDTH
            or render_profile.height_px != CAPTURE_HEIGHT
        ):
            raise RuntimeError(
                "The requested camera profile does not match the active Isaac sensor"
            )
        final_rgb: np.ndarray | None = None
        for _ in range(CAPTURE_RENDER_UPDATES):
            self._step()
            frame, _ = self._sensor.get_data("rgb")
            if frame is not None:
                candidate = frame.numpy()
                if candidate.ndim == 3 and candidate.shape[2] >= 3:
                    final_rgb = candidate
        if final_rgb is None:
            raise RuntimeError("Isaac RTX camera returned no RGB frame")
        rgb_u8 = _rgb_to_u8(final_rgb)
        non_black_fraction = float((np.max(rgb_u8, axis=2) >= 8).mean())
        variance = float(rgb_u8.var())
        if variance < 4.0 or non_black_fraction < 0.02:
            raise RuntimeError(
                "Isaac RTX camera returned a blank or uniform frame "
                f"(min={int(rgb_u8.min())}, max={int(rgb_u8.max())}, "
                f"variance={variance:.3f}, nonBlackFraction={non_black_fraction:.5f})"
            )
        success, encoded = cv2.imencode(
            ".jpg",
            cv2.cvtColor(rgb_u8, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), 90],
        )
        if not success:
            raise RuntimeError("Isaac RGB frame could not be encoded as JPEG")
        jpeg = encoded.tobytes()
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[:2] != rgb_u8.shape[:2]:
            raise RuntimeError("Encoded Isaac JPEG failed a decode round trip")
        state = self._state()
        return capture_result(
            frame_id=f"isaacrgb_{self._revision}_{time.time_ns()}",
            data=jpeg,
            width=int(rgb_u8.shape[1]),
            height=int(rgb_u8.shape[0]),
            captured_at=state["capturedAt"],
            state_revision=self._revision,
            joint_positions_degrees=state["jointPositionsDegrees"],
        )

    def close(self) -> None:
        if self._timeline.is_playing():
            self._timeline.stop()

    def _step(self) -> None:
        self._app.update()

    def _read_all_dof_radians(self) -> np.ndarray:
        positions = np.asarray(self._articulation.get_dof_positions().numpy(), dtype=np.float64)
        if positions.ndim == 2 and positions.shape[0] == 1:
            positions = positions[0]
        if positions.ndim != 1 or positions.size != 4 or not np.isfinite(positions).all():
            raise RuntimeError("Isaac articulation returned invalid DOF positions")
        return positions

    def _set_positions_immediate(
        self, targets: Mapping[str, float]
    ) -> None:
        radians = self._read_all_dof_radians()
        for joint_id, degrees in targets.items():
            radians[self._dof_indices[JOINT_IDS.index(joint_id)]] = math.radians(
                float(degrees)
            )
        self._articulation.set_dof_positions(radians)
        self._articulation.set_dof_position_targets(radians)
        self._step()

    def _state(self) -> dict[str, object]:
        radians = self._read_all_dof_radians()
        degrees_by_joint = {
            joint_id: float(
                math.degrees(radians[self._dof_indices[index]])
            )
            for index, joint_id in enumerate(JOINT_IDS)
        }
        return {
            "stateRevision": self._revision,
            "jointPositionsDegrees": degrees_by_joint,
            "moving": self._moving,
            "stopped": self._stopped,
            "capturedAt": _timestamp(),
        }


def _self_test(engine: IsaacSimBridgeEngine) -> int:
    _init_phase("self-test-reset-begin")
    reset = engine.reset(seed=7)
    _init_phase("self-test-reset-complete")
    moved = engine.set_joint_targets(targets={"joint_4": 5.0}, duration_ms=100)
    _init_phase("self-test-motion-complete")
    # Zero means the camera looks horizontally along the forearm. The smoke
    # target sits on the desk below it, so establish a deterministic downward
    # test viewpoint before proving the RTX path. Immediate placement here is
    # the same reset-only operation used during engine initialization; the
    # public set_joint_targets command was exercised immediately above.
    engine._set_positions_immediate({"joint_4": 40.0})
    _init_phase("self-test-viewpoint-complete")
    capture = engine.capture(profile="detail")
    _init_phase("self-test-capture-complete")
    report = {
        "status": "ok",
        "physicalProof": False,
        "engine": engine.health()["engine"],
        "cameraProfile": CAMERA_PROFILE.report_metadata(),
        "stateRevision": moved["stateRevision"],
        "resetJointPositionsDegrees": reset["jointPositionsDegrees"],
        "movedJointPositionsDegrees": moved["jointPositionsDegrees"],
        "capture": {
            "mimeType": capture["mimeType"],
            "width": capture["width"],
            "height": capture["height"],
            "jointPositionsDegrees": capture["jointPositionsDegrees"],
            "jpegByteCount": len(base64.b64decode(capture["dataBase64"])),
            "encodedBase64Length": len(capture["dataBase64"]),
            "sha256": capture["sha256"],
        },
        "limitations": [
            "simulator-only evidence",
            "articulation drives, contact, camera calibration, and timing remain provisional",
            "autofocus, lens distortion, depth of field, and focus convergence are not modeled",
            "detail currently renders the same 1280 x 720 image as survey",
        ],
    }
    print(json.dumps(report, sort_keys=True, separators=(",", ":")), flush=True)
    return 0


def main() -> int:
    print("[arm-sim] initializing persistent engine", flush=True)
    try:
        engine = IsaacSimBridgeEngine(SIMULATION_APP, SCENE_USD)
    except BaseException as error:
        return _failure_exit_code("engine initialization", error)
    print("[arm-sim] persistent engine initialized", flush=True)
    if ARGS.self_test:
        try:
            try:
                return _self_test(engine)
            except BaseException as error:
                return _failure_exit_code("self-test", error)
        finally:
            engine.close()
    assert ARGS.token_file is not None
    token = _read_token_file(ARGS.token_file)
    server = LoopbackBridgeServer(
        engine=engine,
        token=token,
        host=BRIDGE_HOST,
        port=BRIDGE_PORT,
        service_action=None if ARGS.headless else _service_visible_application,
    )
    print(
        json.dumps(
            {
                "status": "ready",
                "backendId": "sim",
                "backendInstanceId": server.backend_instance_id,
                "simulated": True,
                "host": BRIDGE_HOST,
                "port": BRIDGE_PORT,
            },
            sort_keys=True,
            separators=(",", ":"),
        ),
        flush=True,
    )
    try:
        # In GUI mode the server's same-thread service action pumps Kit between
        # requests so Windows paint/input events remain responsive. Engine
        # commands and Isaac/PhysX updates remain serialized on this thread.
        server.serve_forever(
            poll_interval=0.1 if ARGS.headless else GUI_SERVER_POLL_SECONDS
        )
    except (KeyboardInterrupt, _VisibleApplicationClosed):
        return 0
    finally:
        server.server_close()
        engine.close()
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        try:
            exit_code = main()
        except BaseException as error:
            exit_code = _failure_exit_code("server", error)
    finally:
        if exit_code:
            # SimulationApp.close() normalizes some Kit failures to process
            # status zero. A failed startup/self-test has nothing left to
            # service, so terminate this exact process with the real failure
            # code before Kit can hide it.
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
        try:
            SIMULATION_APP.close()
        finally:
            # Kit's shutdown path can normalize a pending SystemExit to zero.
            # Flush the bounded evidence first, then terminate with the status
            # that main actually returned so supervisors cannot accept a
            # failed self-test as success.
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
