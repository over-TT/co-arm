"""Deterministically build the bridge smoke USD inside one SimulationApp.

There are no Isaac imports at module-import time. ``build_smoke_scene`` runs
only after the persistent entrypoint creates its single ``SimulationApp``. It
is the clean-checkout fallback when ignored generated USD artifacts are absent;
later launches reuse the export only while its source-provenance sidecar still
matches this factory and the robot URDF.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any

from arm_sim.camera_profile import CAMERA_PROFILE_PATH, load_camera_profile
PROJECT_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = PROJECT_ROOT / "arm_sim" / "assets"
SOURCE_URDF = ASSET_ROOT / "desk_camera_arm.urdf"
ROBOT_ROOT = "/World/DeskCameraArm"
CAMERA_OPTICAL_FRAME = (
    f"{ROBOT_ROOT}/Geometry/world/base_link/shoulder_mount_link/upper_arm_link/"
    "forearm_link/tool_offset_link/camera_link/camera_optical_frame"
)
CAMERA_MOUNT = f"{CAMERA_OPTICAL_FRAME}/SmokeCameraMount"
CAMERA_PATH = f"{CAMERA_MOUNT}/RGB"
PROVENANCE_SCHEMA = "arm-sim-scene-provenance.v1"

# The URDF importer currently writes the requested angular stiffness through a
# radians-to-degrees conversion, while PhysX's articulation tensor controller
# consumes the authored drive value directly.  The resulting 0.436 stiffness
# cannot hold this arm's horizontal distal links against gravity.  Author the
# provisional servo drives in the stronger root layer so a commanded pose is a
# pose, not a slowly collapsing pendulum.  These remain workflow-simulation
# values until measured trajectories from the real arm replace them.
ARM_DRIVE_GAINS = {
    "base_yaw_joint": (150.0, 15.0, 20.0),
    "shoulder_pitch_joint": (150.0, 15.0, 20.0),
    "elbow_pitch_joint": (150.0, 15.0, 20.0),
    "camera_pitch_joint": (100.0, 10.0, 4.0),
}


def _update_digest_with_file(digest: Any, label: str, source: Path) -> None:
    if source.is_symlink() or not source.is_file():
        raise FileNotFoundError(f"Simulator source is unavailable: {label}")
    digest.update(b"\0")
    digest.update(label.encode("utf-8"))
    digest.update(b"\0")
    digest.update(source.read_bytes())


def _file_digest(source: Path) -> str:
    digest = hashlib.sha256()
    _update_digest_with_file(digest, source.name, source)
    return digest.hexdigest()


def _tree_digest(root: Path) -> str:
    resolved = root.resolve()
    if resolved.is_symlink() or not resolved.is_dir():
        raise FileNotFoundError("Simulator artifact tree is unavailable")
    files = sorted(
        (candidate for candidate in resolved.rglob("*") if candidate.is_file()),
        key=lambda candidate: candidate.relative_to(resolved).as_posix(),
    )
    if not files:
        raise FileNotFoundError("Simulator artifact tree is empty")
    digest = hashlib.sha256()
    for source in files:
        relative = source.relative_to(resolved).as_posix()
        _update_digest_with_file(digest, relative, source)
    return digest.hexdigest()


def scene_source_digest() -> str:
    """Hash every local authored input that changes the generated USD."""

    if not ASSET_ROOT.is_dir():
        raise FileNotFoundError("Simulator asset directory is unavailable")
    digest = hashlib.sha256()
    digest.update(PROVENANCE_SCHEMA.encode("ascii"))
    _update_digest_with_file(digest, "scene_factory.py", Path(__file__))
    _update_digest_with_file(
        digest, "config/camera_profile.json", CAMERA_PROFILE_PATH
    )
    assets = sorted(
        (candidate for candidate in ASSET_ROOT.rglob("*") if candidate.is_file()),
        key=lambda candidate: candidate.relative_to(ASSET_ROOT).as_posix(),
    )
    if not assets:
        raise FileNotFoundError("Simulator asset directory is empty")
    for source in assets:
        _update_digest_with_file(
            digest,
            f"assets/{source.relative_to(ASSET_ROOT).as_posix()}",
            source,
        )
    return digest.hexdigest()


def scene_provenance_path(scene_path: Path) -> Path:
    return scene_path.with_name(f"{scene_path.name}.provenance.json")


def smoke_scene_is_current(scene_path: Path) -> bool:
    """Return whether a generated scene exactly matches its authored inputs."""

    scene = scene_path.resolve()
    sidecar = scene_provenance_path(scene)
    if not scene.is_file() or not sidecar.is_file():
        return False
    try:
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            return False
        if document.get("schema") != PROVENANCE_SCHEMA:
            return False
        if document.get("sourceDigest") != scene_source_digest():
            return False
        if document.get("sceneFile") != scene.name:
            return False
        robot_asset = document.get("robotAsset")
        if not isinstance(robot_asset, str) or not robot_asset:
            return False
        robot_root = document.get("robotArtifactRoot")
        if not isinstance(robot_root, str) or not robot_root:
            return False
        relative_asset = Path(robot_asset)
        relative_root = Path(robot_root)
        if (
            relative_asset.is_absolute()
            or ".." in relative_asset.parts
            or relative_root.is_absolute()
            or ".." in relative_root.parts
        ):
            return False
        artifact_root = (scene.parent / relative_root).resolve()
        artifact = (scene.parent / relative_asset).resolve()
        artifact.relative_to(artifact_root)
        artifact_root.relative_to(scene.parent.resolve())
        if not artifact.is_file():
            return False
        scene_digest = document.get("sceneSha256")
        artifact_digest = document.get("robotArtifactDigest")
        if not isinstance(scene_digest, str) or not isinstance(artifact_digest, str):
            return False
        return (
            _file_digest(scene) == scene_digest
            and _tree_digest(artifact_root) == artifact_digest
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        return False


def _write_scene_provenance(
    scene_path: Path,
    imported_robot: Path,
    robot_artifact_root: Path,
    source_digest: str,
) -> None:
    destination = scene_path.resolve()
    try:
        relative_robot = imported_robot.resolve().relative_to(destination.parent)
        relative_root = robot_artifact_root.resolve().relative_to(destination.parent)
        imported_robot.resolve().relative_to(robot_artifact_root.resolve())
    except ValueError:
        raise RuntimeError("Imported robot asset escaped the generated scene directory") from None
    document = {
        "schema": PROVENANCE_SCHEMA,
        "sourceDigest": source_digest,
        "sceneFile": destination.name,
        "sceneSha256": _file_digest(destination),
        "robotAsset": relative_robot.as_posix(),
        "robotArtifactRoot": relative_root.as_posix(),
        "robotArtifactDigest": _tree_digest(robot_artifact_root),
    }
    payload = (json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8"
    )
    sidecar = scene_provenance_path(destination)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{sidecar.name}.", dir=sidecar.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, sidecar)
    except OSError:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            Path(temporary).unlink(missing_ok=True)
        except OSError:
            pass
        raise RuntimeError("Isaac scene provenance could not be recorded") from None


def _set_display_color(geometry: Any, rgb: tuple[float, float, float]) -> None:
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
    cube.AddScaleOp().Set(
        Gf.Vec3f(*(dimension / 2.0 for dimension in dimensions))
    )
    _set_display_color(cube, color)
    return cube


def _physics_material(
    stage: Any,
    path: str,
    *,
    static_friction: float,
    dynamic_friction: float,
    restitution: float,
) -> Any:
    from pxr import UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(static_friction)
    physics.CreateDynamicFrictionAttr(dynamic_friction)
    physics.CreateRestitutionAttr(restitution)
    return material


def _bind_physics_material(prim: Any, material: Any) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, "physics")


def _import_robot(output_dir: Path) -> Path:
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    if not SOURCE_URDF.is_file():
        raise FileNotFoundError("The simulator arm URDF source is unavailable.")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig()
    config.urdf_path = str(SOURCE_URDF)
    config.usd_path = str(output_dir)
    config.fix_base = True
    config.merge_fixed_joints = False
    config.merge_mesh = False
    config.allow_self_collision = False
    config.collision_from_visuals = False
    config.joint_drive_type = "force"
    config.joint_target_type = "position"
    config.override_joint_stiffness = 25.0
    config.override_joint_damping = 5.0
    imported = URDFImporter(config).import_urdf()
    if not imported:
        raise RuntimeError("Isaac URDF importer returned no simulator asset")
    imported_path = Path(imported).resolve()
    if not imported_path.is_file():
        raise RuntimeError("Isaac URDF importer did not create its reported asset")
    return imported_path


def build_smoke_scene(simulation_app: Any, scene_path: Path) -> Path:
    """Create and export the fixed desk, arm, target, lights, and RGB camera.

    The destination comes only from the local process CLI, never from IPC.
    Geometry, import settings, authored target pose, optics, and frame
    conversion are fixed here so an empty checkout produces the same
    provisional scene.
    """

    import omni.usd
    from isaacsim.core.experimental.utils import stage as stage_utils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    destination = scene_path.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    sidecar = scene_provenance_path(destination)
    try:
        sidecar.unlink(missing_ok=True)
    except OSError:
        raise RuntimeError("Stale Isaac scene provenance could not be removed") from None
    source_digest_before = scene_source_digest()
    camera_profile = load_camera_profile()
    projection = camera_profile.projection
    robot_artifact_root = destination.parent / "desk_camera_arm"
    imported_robot = _import_robot(robot_artifact_root)

    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac returned no stage for simulator scene generation")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateEnableEnhancedDeterminismAttr(True)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateSolverTypeAttr("TGS")

    desk_material = _physics_material(
        stage,
        "/World/PhysicsMaterials/Desk",
        static_friction=0.75,
        dynamic_friction=0.60,
        restitution=0.01,
    )
    target_material = _physics_material(
        stage,
        "/World/PhysicsMaterials/Prop",
        static_friction=0.55,
        dynamic_friction=0.40,
        restitution=0.01,
    )
    desk = _create_box(
        stage,
        "/World/Desk",
        center=(0.35, 0.0, -0.025),
        dimensions=(0.90, 0.70, 0.05),
        color=(0.28, 0.18, 0.11),
    )
    UsdPhysics.CollisionAPI.Apply(desk.GetPrim())
    _bind_physics_material(desk.GetPrim(), desk_material)

    target = UsdGeom.Cylinder.Define(stage, "/World/Target")
    target.CreateAxisAttr(UsdGeom.Tokens.z)
    target.CreateRadiusAttr(0.033)
    target.CreateHeightAttr(0.120)
    target.AddTranslateOp().Set(Gf.Vec3d(0.315, 0.0, 0.060))
    target.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
    )
    _set_display_color(target, (0.86, 0.12, 0.05))
    target.GetPrim().SetCustomDataByKey("nominalMassKg", 0.014)
    target.GetPrim().SetCustomDataByKey("role", "generic lightweight target")
    UsdPhysics.CollisionAPI.Apply(target.GetPrim())
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(target.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(target.GetPrim()).CreateMassAttr(0.014)
    PhysxSchema.PhysxRigidBodyAPI.Apply(target.GetPrim()).CreateEnableCCDAttr(True)
    _bind_physics_material(target.GetPrim(), target_material)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.88, 1.0))
    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(1600.0)
    key.CreateWidthAttr(1.0)
    key.CreateHeightAttr(0.7)
    key.AddTranslateOp().Set(Gf.Vec3d(0.35, 0.0, 1.0))

    stage_utils.add_reference_to_stage(
        usd_path=str(imported_robot), path=ROBOT_ROOT
    )
    while stage_utils.is_stage_loading():
        simulation_app.update()

    # Override the weak imported drives in the scene's root layer.  A missing
    # joint is a build error because silently leaving one soft makes reset and
    # contact outcomes dependent on how long rendering happens to take.
    for joint_name, (stiffness, damping, max_force) in ARM_DRIVE_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_ROOT}/Physics/{joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Imported arm is missing drive joint {joint_name}")
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateMaxForceAttr().Set(max_force)

    if not stage.GetPrimAtPath(CAMERA_OPTICAL_FRAME).IsValid():
        raise RuntimeError("Imported arm is missing its camera optical frame")

    UsdGeom.Xform.Define(stage, CAMERA_MOUNT)
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    if not camera.GetPrim().ApplyAPI("OmniSensorAPI"):
        raise RuntimeError("Isaac could not apply OmniSensorAPI to the RGB camera")
    # UsdGeom stores focal length and aperture in tenths of a stage unit. The
    # experimental camera wrapper's millimetre setters apply this same factor.
    # These values are a manufacturer-FOV-matched pinhole approximation, not
    # measured intrinsics or distortion for the installed physical camera.
    camera.CreateFocalLengthAttr(projection.focal_length_mm * 10.0)
    camera.CreateHorizontalApertureAttr(
        projection.horizontal_aperture_mm * 10.0
    )
    camera.CreateVerticalApertureAttr(projection.vertical_aperture_mm * 10.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 5.0))
    camera_xform = UsdGeom.Xformable(camera.GetPrim())
    camera_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    camera_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0))
    )
    camera_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(1.0, 1.0, 1.0)
    )

    if not stage.GetRootLayer().Export(str(destination)):
        raise RuntimeError("Isaac could not export the simulator scene")
    if not destination.is_file():
        raise RuntimeError("Isaac reported scene export without a scene file")
    if scene_source_digest() != source_digest_before:
        raise RuntimeError("Simulator authored inputs changed during scene generation")
    _write_scene_provenance(
        destination,
        imported_robot,
        robot_artifact_root,
        source_digest_before,
    )
    if scene_source_digest() != source_digest_before:
        try:
            sidecar.unlink(missing_ok=True)
        except OSError:
            pass
        raise RuntimeError("Simulator authored inputs changed during provenance recording")
    return destination


__all__ = [
    "build_smoke_scene",
    "scene_provenance_path",
    "scene_source_digest",
    "smoke_scene_is_current",
]
