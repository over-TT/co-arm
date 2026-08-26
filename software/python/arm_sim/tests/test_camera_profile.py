"""Truth-boundary tests for the provisional Module 3 Wide Isaac profile."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from arm_sim.camera_profile import (
    CAMERA_PROFILE_PATH,
    CameraProfileError,
    load_camera_profile,
)


def test_profile_preserves_official_nominal_module_3_wide_facts() -> None:
    profile = load_camera_profile()
    reference = profile.document["referenceHardware"]

    assert reference["module"] == "Camera Module 3 Wide"
    assert reference["sensorModel"] == "imx708"
    assert reference["sensorResolutionPx"] == {"width": 4608, "height": 2592}
    assert reference["focalLengthMm"] == 2.75
    assert reference["fieldOfViewDeg"] == {
        "horizontal": 102.0,
        "vertical": 67.0,
        "diagonal": 120.0,
    }
    assert reference["minimumFocusDistanceMm"] == 50.0
    assert reference["focusCapability"] == "powered_autofocus"
    assert reference["evidenceStatus"] == "manufacturer_nominal_not_live_verified"


def test_sim_identity_render_profiles_and_focus_limit_are_explicit() -> None:
    profile = load_camera_profile()

    assert profile.camera_id == "isaac-sim-camera"
    assert profile.sensor_model == "isaac-imx708-wide-provisional"
    assert profile.identity_confidence == "simulated_model"
    assert (profile.survey.width_px, profile.survey.height_px) == (1280, 720)
    assert (profile.detail.width_px, profile.detail.height_px) == (1280, 720)
    assert "same 1280 x 720" in (profile.detail.limitation or "")
    assert profile.physical_focus_capability == "powered_autofocus"
    assert profile.simulated_focus_capability == "unsupported"
    assert profile.report_metadata()["focusModel"]["status"] == "not_modeled"


def test_pinhole_apertures_match_nominal_horizontal_and_vertical_fov() -> None:
    projection = load_camera_profile().projection
    horizontal = math.degrees(
        2.0
        * math.atan(
            projection.horizontal_aperture_mm / (2.0 * projection.focal_length_mm)
        )
    )
    vertical = math.degrees(
        2.0
        * math.atan(
            projection.vertical_aperture_mm / (2.0 * projection.focal_length_mm)
        )
    )

    assert horizontal == pytest.approx(102.0, abs=0.01)
    assert vertical == pytest.approx(67.0, abs=0.01)
    assert "not a calibrated model" in projection.limitation


def test_uncalibrated_intrinsics_distortion_and_extrinsics_remain_empty(
    tmp_path: Path,
) -> None:
    document = json.loads(CAMERA_PROFILE_PATH.read_text(encoding="utf-8"))
    calibration = document["calibration"]
    assert set(calibration["intrinsics"].values()) >= {
        None,
        "required_from_installed_camera_per_capture_mode",
    }
    assert calibration["distortion"]["model"] is None
    assert calibration["distortion"]["coefficients"] == []
    assert calibration["extrinsics"]["translationM"] is None
    assert calibration["extrinsics"]["rpyRad"] is None

    document["calibration"]["intrinsics"]["fxPx"] = 1.0
    invalid = tmp_path / "invalid-camera-profile.json"
    invalid.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(CameraProfileError, match="intrinsics must remain null"):
        load_camera_profile(invalid)
