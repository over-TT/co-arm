"""Smoke the installed co-arm wheel from outside its source tree.

CI runs this file with the isolated wheel environment's interpreter after the
wheel is installed. Keeping the probe outside the packaged modules prevents a
source checkout from hiding missing package data.
"""

from __future__ import annotations

from importlib.metadata import version
from importlib.resources import files
from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from arm_sim.camera_profile import load_camera_profile
from web_backend.runtime import main as dashboard_main


def main() -> int:
    assert version("co-arm-stack") == "0.1.0"

    package_root = files("arm_sim")
    model_resource = package_root.joinpath("config", "arm_model.json")
    camera_resource = package_root.joinpath("config", "camera_profile.json")
    urdf_resource = package_root.joinpath("assets", "desk_camera_arm.urdf")

    model = json.loads(model_resource.read_text(encoding="utf-8"))
    camera = json.loads(camera_resource.read_text(encoding="utf-8"))
    urdf = ET.fromstring(urdf_resource.read_text(encoding="utf-8"))

    assert model["schema"] == "arm-sim.model.v1"
    assert model["urdf"] == "../assets/desk_camera_arm.urdf"
    assert camera["schema"] == "arm-sim.camera-profile.v1"
    assert urdf.tag == "robot"
    assert load_camera_profile().profile_id == camera["profileId"]
    assert find_spec("arm_mcp.tests") is None
    assert find_spec("arm_sim.tests") is None
    assert find_spec("robot_gateway.tests") is None
    assert find_spec("web_backend.tests") is None

    with tempfile.TemporaryDirectory() as directory:
        missing_dashboard = Path(directory) / "dist"
        assert dashboard_main(
            [
                "--static-directory",
                str(missing_dashboard),
                "--no-access-log",
            ]
        ) == 2

    print("Installed wheel smoke passed: Python modules and Arm simulator resources load.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
