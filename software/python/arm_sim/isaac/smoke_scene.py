"""Render a deterministic, robot-mounted-camera smoke scene in Isaac Sim 6.

Run this module with the bundled Isaac runtime, after :mod:`import_arm` has
created the USD asset::

    C:\\isaacsim\\python.bat arm_sim\\isaac\\smoke_scene.py --headless

The scene is deliberately small enough for a first local RTX check.  It never
imports the live gateway, opens the network, or commands physical hardware.
Its report is simulator evidence only; ``physicalProof`` is always ``false``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

from isaacsim import SimulationApp


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from arm_sim.camera_profile import load_camera_profile


CAMERA_PROFILE = load_camera_profile()
GENERATED_DIR = PROJECT_ROOT.parent / "runtime" / "arm-sim"
DEFAULT_ROBOT_USD = (
    GENERATED_DIR
    / "desk_camera_arm"
    / "desk_camera_arm"
    / "desk_camera_arm.usda"
)
DEFAULT_SCENE_USD = GENERATED_DIR / "smoke_scene.usda"
DEFAULT_RGB_PATH = GENERATED_DIR / "smoke_rgb.png"
DEFAULT_REPORT_PATH = GENERATED_DIR / "smoke_report.json"

ROBOT_ROOT = "/World/DeskCameraArm"
CAMERA_OPTICAL_FRAME = (
    f"{ROBOT_ROOT}/Geometry/world/base_link/shoulder_mount_link/upper_arm_link/"
    "forearm_link/tool_offset_link/camera_link/camera_optical_frame"
)
CAMERA_MOUNT = f"{CAMERA_OPTICAL_FRAME}/SmokeCameraMount"
CAMERA_PATH = f"{CAMERA_MOUNT}/RGB"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and render the desk-camera-arm Isaac smoke scene."
    )
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT_USD)
    parser.add_argument("--scene-usd", type=Path, default=DEFAULT_SCENE_USD)
    parser.add_argument("--rgb", type=Path, default=DEFAULT_RGB_PATH)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT_PATH)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--frames", type=int, default=48)
    parser.add_argument("--width", type=int, default=CAMERA_PROFILE.survey.width_px)
    parser.add_argument("--height", type=int, default=CAMERA_PROFILE.survey.height_px)
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run without an Isaac Sim window (default: true).",
    )
    args = parser.parse_known_args()[0]
    if args.frames < 1:
        parser.error("--frames must be at least 1")
    if args.width < 64 or args.height < 64:
        parser.error("--width and --height must each be at least 64")
    return args


ARGS = _parse_args()
SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "width": ARGS.width,
        "height": ARGS.height,
    }
)


def _set_display_color(geometry: Any, rgb: tuple[float, float, float]) -> None:
    """Set a simple Hydra-visible display color without a material graph."""

    geometry.CreateDisplayColorAttr([rgb])


def _create_box(
    stage: Any,
    path: str,
    *,
    center: tuple[float, float, float],
    dimensions: tuple[float, float, float],
    color: tuple[float, float, float],
) -> Any:
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*(dimension / 2.0 for dimension in dimensions)))
    _set_display_color(cube, color)
    return cube


def _create_scene(stage: Any, *, rng: np.random.Generator) -> dict[str, Any]:
    """Author the minimal desk, target, and lighting and return target truth."""

    from pxr import Gf, UsdGeom, UsdLux

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    _create_box(
        stage,
        "/World/Desk",
        center=(0.35, 0.0, -0.025),
        dimensions=(0.90, 0.70, 0.05),
        color=(0.28, 0.18, 0.11),
    )

    # The seed changes only a small, camera-visible placement window.  The
    # target remains a generic lightweight object rather than a canned route.
    target_x = float(rng.uniform(0.455, 0.495))
    target_y = float(rng.uniform(-0.035, 0.035))
    target_radius = 0.033
    target_height = 0.120
    target = UsdGeom.Cylinder.Define(stage, "/World/Target")
    target.CreateAxisAttr(UsdGeom.Tokens.z)
    target.CreateRadiusAttr(target_radius)
    target.CreateHeightAttr(target_height)
    target.AddTranslateOp().Set(Gf.Vec3d(target_x, target_y, target_height / 2.0))
    _set_display_color(target, (0.86, 0.12, 0.05))
    target.GetPrim().SetCustomDataByKey("nominalMassKg", 0.014)
    target.GetPrim().SetCustomDataByKey("role", "generic lightweight target")

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.88, 1.0))

    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(1600.0)
    key.CreateWidthAttr(1.0)
    key.CreateHeightAttr(0.7)
    key.AddTranslateOp().Set(Gf.Vec3d(0.35, 0.0, 1.0))

    return {
        "prim": "/World/Target",
        "positionM": [target_x, target_y, target_height / 2.0],
        "radiusM": target_radius,
        "heightM": target_height,
        "nominalMassKg": 0.014,
    }


def _mount_camera(
    stage: Any, *, target_world_m: list[float]
) -> tuple[Any, dict[str, Any]]:
    """Create an RTX RGB camera beneath the imported optical-frame prim.

    ``ViewportManager.set_camera_view`` is the bundled Isaac 6 look-at helper.
    It authors the camera transform relative to its parent, so the camera stays
    mounted in the arm hierarchy while the commissioning frame is aimed at the
    generated target instead of relying on a hand-derived axis convention.
    """

    from isaacsim.core.rendering_manager import ViewportManager
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from pxr import Usd, UsdGeom

    optical_prim = stage.GetPrimAtPath(CAMERA_OPTICAL_FRAME)
    if not optical_prim.IsValid():
        raise RuntimeError(
            "Imported arm is missing its camera optical frame at "
            f"{CAMERA_OPTICAL_FRAME}"
        )

    UsdGeom.Xform.Define(stage, CAMERA_MOUNT)

    camera = RtxCamera(CAMERA_PATH, tick_rate=30.0)
    projection = CAMERA_PROFILE.projection
    camera.camera.set_focal_lengths(projection.focal_length_mm)
    camera.camera.set_apertures(
        horizontal_apertures=projection.horizontal_aperture_mm,
        vertical_apertures=projection.vertical_aperture_mm,
    )
    camera.camera.set_clipping_ranges(0.01, 5.0)

    optical_transform = UsdGeom.Xformable(optical_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    eye = optical_transform.ExtractTranslation()
    eye_world_m = [float(eye[0]), float(eye[1]), float(eye[2])]
    ViewportManager.set_camera_view(
        CAMERA_PATH,
        eye=eye_world_m,
        target=target_world_m,
    )

    sensor = CameraSensor(
        camera,
        resolution=(ARGS.height, ARGS.width),
        annotators=["rgb"],
    )
    return sensor, {
        "orientationStrategy": "ViewportManager look-at from optical frame",
        "eyeWorldM": eye_world_m,
        "lookAtWorldM": target_world_m,
    }


def _array_stats(array: np.ndarray[Any, Any]) -> dict[str, Any]:
    """Return JSON-safe diagnostics without hiding NaN or empty data."""

    values = np.asarray(array)
    finite = values[np.isfinite(values)]
    return {
        "dtype": str(values.dtype),
        "shape": list(values.shape),
        "finiteFraction": float(finite.size / values.size) if values.size else 0.0,
        "min": float(finite.min()) if finite.size else None,
        "max": float(finite.max()) if finite.size else None,
        "mean": float(finite.mean()) if finite.size else None,
        "variance": float(finite.var()) if finite.size else None,
    }


def _rgb_to_u8(raw_rgb: np.ndarray[Any, Any]) -> tuple[np.ndarray[Any, Any], str]:
    """Preserve uint8 RGB and correctly encode normalized float RGB."""

    raw = np.asarray(raw_rgb[:, :, :3])
    clean = np.nan_to_num(raw, nan=0.0, posinf=255.0, neginf=0.0)
    if np.issubdtype(clean.dtype, np.floating):
        # Replicator's current ``rgb`` annotator is uint8, but this makes the
        # proof writer robust to a float [0, 1] backend or future API variant.
        finite_max = float(clean.max()) if clean.size else 0.0
        finite_min = float(clean.min()) if clean.size else 0.0
        if finite_min >= -0.01 and finite_max <= 1.01:
            encoded = np.rint(np.clip(clean, 0.0, 1.0) * 255.0).astype(np.uint8)
            return encoded, "float_0_1_to_uint8"
        encoded = np.rint(np.clip(clean, 0.0, 255.0)).astype(np.uint8)
        return encoded, "float_0_255_to_uint8"

    if np.issubdtype(clean.dtype, np.integer) and clean.size and int(clean.max()) <= 1:
        return (clean.astype(np.uint8) * 255), "integer_0_1_to_uint8"
    return np.clip(clean, 0, 255).astype(np.uint8), "integer_to_uint8"


def _validate_visible_frame(rgb_u8: np.ndarray[Any, Any]) -> dict[str, Any]:
    """Require actual scene contrast plus centered target/desk-like pixels."""

    rgb = rgb_u8.astype(np.float32)
    red, green, blue = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    non_black = np.max(rgb, axis=2) >= 8.0
    target_like = (
        (red >= 24.0)
        & (red >= green * 1.30 + 2.0)
        & (red >= blue * 1.30 + 2.0)
    )
    desk_like = (
        (red >= 15.0)
        & (green >= 7.0)
        & (red >= green * 1.05)
        & (green >= blue * 1.02)
        & (red <= green * 4.0 + 4.0)
    )

    total = int(rgb.shape[0] * rgb.shape[1])
    target_count = int(target_like.sum())
    desk_count = int(desk_like.sum())
    non_black_count = int(non_black.sum())
    if target_count:
        ys, xs = np.nonzero(target_like)
        target_centroid = [
            float(xs.mean() / max(1, rgb.shape[1] - 1)),
            float(ys.mean() / max(1, rgb.shape[0] - 1)),
        ]
        target_is_centered = (
            0.15 <= target_centroid[0] <= 0.85
            and 0.15 <= target_centroid[1] <= 0.85
        )
    else:
        target_centroid = None
        target_is_centered = False

    encoded_variance = float(rgb.var())
    checks = {
        "nonBlack": non_black_count >= max(1000, int(total * 0.02)),
        "hasVariance": encoded_variance >= 4.0,
        "targetColorVisible": target_count >= 64,
        "targetCentered": target_is_centered,
        "deskColorVisible": desk_count >= 256,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "nonBlackPixels": non_black_count,
        "nonBlackFraction": float(non_black_count / total),
        "targetLikePixels": target_count,
        "targetLikeCentroidNormalized": target_centroid,
        "deskLikePixels": desk_count,
        "encodedVariance": encoded_variance,
    }


def _write_report(report: dict[str, Any]) -> None:
    ARGS.report.resolve().parent.mkdir(parents=True, exist_ok=True)
    compact = json.dumps(report, separators=(",", ":"), sort_keys=True)
    ARGS.report.resolve().write_text(compact + "\n", encoding="utf-8")
    print(compact, flush=True)


def main() -> int:
    import cv2
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.utils import stage as stage_utils
    from pxr import UsdGeom

    robot_usd = ARGS.robot_usd.resolve()
    scene_usd = ARGS.scene_usd.resolve()
    rgb_path = ARGS.rgb.resolve()
    if not robot_usd.is_file():
        raise FileNotFoundError(
            f"Imported robot USD not found: {robot_usd}. Run import_arm.py first."
        )

    for output in (scene_usd, rgb_path, ARGS.report.resolve()):
        output.parent.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(ARGS.seed)
    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    target_truth = _create_scene(stage, rng=rng)
    stage_utils.add_reference_to_stage(usd_path=str(robot_usd), path=ROBOT_ROOT)
    while stage_utils.is_stage_loading():
        SIMULATION_APP.update()

    sensor, camera_pose = _mount_camera(
        stage,
        target_world_m=target_truth["positionM"],
    )
    if not stage.GetRootLayer().Export(str(scene_usd)):
        raise RuntimeError(f"Failed to export smoke scene: {scene_usd}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    first_frame_index: int | None = None
    saved_frame_index: int | None = None
    final_rgb: np.ndarray[Any, Any] | None = None
    try:
        for frame_index in range(ARGS.frames):
            SIMULATION_APP.update()
            frame, _ = sensor.get_data("rgb")
            if frame is not None:
                if first_frame_index is None:
                    first_frame_index = frame_index
                final_rgb = frame.numpy()
                saved_frame_index = frame_index
    finally:
        timeline.stop()

    if final_rgb is None:
        raise RuntimeError(
            f"RTX camera returned no RGB pixels in {ARGS.frames} render updates"
        )
    if final_rgb.ndim != 3 or final_rgb.shape[2] < 3:
        raise RuntimeError(f"Unexpected RGB array shape: {final_rgb.shape}")

    raw_rgb = np.asarray(final_rgb[:, :, :3])
    raw_stats = _array_stats(raw_rgb)
    rgb_u8, conversion = _rgb_to_u8(raw_rgb)
    encoded_stats = _array_stats(rgb_u8)
    validation = _validate_visible_frame(rgb_u8)
    if not cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb_u8, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Failed to save RGB image: {rgb_path}")

    report = {
        "status": "ok" if validation["passed"] else "error",
        "physicalProof": False,
        "seed": ARGS.seed,
        "headless": ARGS.headless,
        "resolution": [ARGS.height, ARGS.width],
        "requestedRenderUpdates": ARGS.frames,
        "firstRgbUpdate": first_frame_index,
        "savedRgbUpdate": saved_frame_index,
        "robotUsd": str(robot_usd),
        "sceneUsd": str(scene_usd),
        "rgbFrame": str(rgb_path),
        "camera": {
            "prim": CAMERA_PATH,
            "parentOpticalFrame": CAMERA_OPTICAL_FRAME,
            "profile": CAMERA_PROFILE.report_metadata(),
            "focalLengthMmPinholeApproximation": (
                CAMERA_PROFILE.projection.focal_length_mm
            ),
            "horizontalApertureMmPinholeApproximation": (
                CAMERA_PROFILE.projection.horizontal_aperture_mm
            ),
            "verticalApertureMmPinholeApproximation": (
                CAMERA_PROFILE.projection.vertical_aperture_mm
            ),
            **camera_pose,
        },
        "rawRgbStats": raw_stats,
        "encodedRgbStats": encoded_stats,
        "rgbConversion": conversion,
        "frameValidation": validation,
        "target": target_truth,
        "limitations": [
            "camera intrinsics, distortion, extrinsics, and mount pitch are uncalibrated",
            "autofocus and depth of field are not modeled",
            "detail and survey currently share the same 1280 x 720 render",
            "arm inertias, servo dynamics, backlash, friction, and contact are not calibrated",
            "simulator pixels are not live-device or physical-outcome proof",
        ],
    }
    _write_report(report)
    return 0 if validation["passed"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        failure = {
            "status": "error",
            "physicalProof": False,
            "seed": ARGS.seed,
            "errorType": type(exc).__name__,
            "error": str(exc),
        }
        try:
            _write_report(failure)
        except Exception:
            print(json.dumps(failure, separators=(",", ":")), file=sys.stderr)
        raise
    finally:
        SIMULATION_APP.close()
