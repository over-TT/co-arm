"""Open the generated arm smoke scene in the full Isaac Sim editor.

This is a human-viewing helper. It opens only a local USD and never imports the
robot gateway, opens a network connection, or commands physical hardware.
Close the Isaac Sim window to end the process.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from isaacsim import SimulationApp


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = PROJECT_ROOT.parent / "runtime" / "arm-sim" / "smoke_scene.usda"
DEFAULT_EXPERIENCE = Path(
    os.environ.get("ISAAC_EXPERIENCE")
    or r"C:\isaacsim\apps\isaacsim.exp.full.kit"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View the local arm scene in Isaac Sim.")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--experience",
        type=Path,
        default=DEFAULT_EXPERIENCE,
        help=(
            "Isaac Sim .kit experience (default: ISAAC_EXPERIENCE or the "
            "standard Windows install path)."
        ),
    )
    return parser.parse_known_args()[0]


ARGS = _parse_args()
SCENE = ARGS.scene.resolve()
EXPERIENCE = ARGS.experience.resolve()
if not EXPERIENCE.is_file():
    raise FileNotFoundError(f"Isaac editor experience not found: {EXPERIENCE}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

APP = SimulationApp(
    {
        "headless": False,
        "hide_ui": False,
        "create_new_stage": True,
        "multi_gpu": False,
        "width": 1280,
        "height": 720,
        "window_width": 1440,
        "window_height": 900,
        "display_options": 3286,
    },
    experience=str(EXPERIENCE),
)


def main() -> int:
    from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
    from isaacsim.core.rendering_manager import ViewportManager
    from arm_sim.isaac.scene_factory import build_smoke_scene, smoke_scene_is_current

    if not smoke_scene_is_current(SCENE):
        print("Generated arm scene is missing or stale; rebuilding it now.", flush=True)
        build_smoke_scene(APP, SCENE)

    opened, stage = open_stage(str(SCENE))
    if not opened or stage is None:
        raise RuntimeError(f"Isaac could not open the local scene: {SCENE}")
    while is_stage_loading():
        APP.update()

    # Start on an external overview so the primitive arm, desk, and target are
    # visible. The robot-mounted RGB camera remains authored in the stage and
    # can be selected from the Stage tree by the user.
    active_camera = ViewportManager.get_camera()
    ViewportManager.set_camera_view(
        active_camera,
        eye=[0.72, -0.72, 0.52],
        target=[0.28, 0.0, 0.12],
    )

    print(
        f"Opened local arm scene for viewing: {stage.GetRootLayer().identifier}",
        flush=True,
    )
    while APP.is_running():
        APP.update()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        APP.close()
