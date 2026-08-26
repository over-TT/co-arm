"""Import the desk-camera arm URDF with the bundled Isaac Sim 6 runtime.

Run this file with ``C:\\isaacsim\\python.bat``, not the project's ordinary
Python interpreter.  It deliberately preserves fixed joints so the authored
camera optical frame remains available for an RTX camera sensor.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = PROJECT_ROOT / "arm_sim" / "assets" / "desk_camera_arm.urdf"
RUNTIME_ROOT = PROJECT_ROOT.parent / "runtime" / "arm-sim"
DEFAULT_OUTPUT_DIR = RUNTIME_ROOT / "desk_camera_arm"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Import the four-axis desk-camera arm into an Isaac USD asset."
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run without an Isaac Sim window (default: true).",
    )
    return parser.parse_known_args()[0]


ARGS = _parse_args()
SIMULATION_APP = SimulationApp({"headless": ARGS.headless})


def main() -> int:
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    urdf_path = ARGS.urdf.resolve()
    output_dir = ARGS.output_dir.resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    output_dir.mkdir(parents=True, exist_ok=True)

    config = URDFImporterConfig()
    config.urdf_path = str(urdf_path)
    config.usd_path = str(output_dir)
    config.fix_base = True
    config.merge_fixed_joints = False
    config.merge_mesh = False
    config.allow_self_collision = False
    config.collision_from_visuals = False
    config.joint_drive_type = "force"
    config.joint_target_type = "position"
    # These gains are intentionally provisional commissioning values.  The
    # real servo dynamics are recorded separately in arm_model.json and must
    # be fitted from measurements before sim-to-real claims are made.
    config.override_joint_stiffness = 25.0
    config.override_joint_damping = 5.0

    imported_path = URDFImporter(config).import_urdf()
    if not imported_path:
        raise RuntimeError("Isaac URDF importer returned no USD path")

    print(
        json.dumps(
            {
                "status": "ok",
                "sourceUrdf": str(urdf_path),
                "importedUsd": str(Path(imported_path).resolve()),
                "fixedBase": True,
                "fixedJointsPreserved": True,
                "physicalProof": False,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        SIMULATION_APP.close()
