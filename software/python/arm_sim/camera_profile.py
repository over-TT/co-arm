"""Validated access to the canonical provisional Isaac camera profile.

The JSON deliberately separates Raspberry Pi's manufacturer facts from the
numeric pinhole approximation Isaac needs and from calibration fields that are
still unknown.  Keeping that boundary here prevents simulator convenience
values from silently becoming physical calibration claims.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping


CAMERA_PROFILE_PATH = Path(__file__).resolve().parent / "config" / "camera_profile.json"
CAMERA_PROFILE_SCHEMA = "arm-sim.camera-profile.v1"
_SAFE_ID = re.compile(r"^[a-z0-9][a-z0-9._-]{1,95}$")


class CameraProfileError(ValueError):
    """The authored simulator camera profile is missing or inconsistent."""


@dataclass(frozen=True, slots=True)
class CameraRenderProfile:
    name: str
    width_px: int
    height_px: int
    limitation: str | None = None


@dataclass(frozen=True, slots=True)
class CameraProjection:
    model: str
    focal_length_mm: float
    horizontal_aperture_mm: float
    vertical_aperture_mm: float
    horizontal_fov_deg: float
    vertical_fov_deg: float
    status: str
    limitation: str


@dataclass(frozen=True, slots=True)
class CameraProfile:
    profile_id: str
    description: str
    camera_id: str
    sensor_model: str
    identity_confidence: str
    reference_sensor_model: str
    physical_focus_capability: str
    simulated_focus_capability: str
    calibration_status: str
    survey: CameraRenderProfile
    detail: CameraRenderProfile
    projection: CameraProjection
    document: Mapping[str, Any]

    def render(self, name: str) -> CameraRenderProfile:
        if name == "survey":
            return self.survey
        if name == "detail":
            return self.detail
        raise CameraProfileError("camera render profile must be survey or detail")

    def report_metadata(self) -> dict[str, object]:
        return {
            "profileId": self.profile_id,
            "cameraId": self.camera_id,
            "sensorModel": self.sensor_model,
            "referenceSensorModel": self.reference_sensor_model,
            "identityConfidence": self.identity_confidence,
            "calibrationStatus": self.calibration_status,
            "renderProfiles": {
                "survey": {
                    "widthPx": self.survey.width_px,
                    "heightPx": self.survey.height_px,
                },
                "detail": {
                    "widthPx": self.detail.width_px,
                    "heightPx": self.detail.height_px,
                    "limitation": self.detail.limitation,
                },
            },
            "projection": {
                "model": self.projection.model,
                "horizontalFovDeg": self.projection.horizontal_fov_deg,
                "verticalFovDeg": self.projection.vertical_fov_deg,
                "status": self.projection.status,
                "limitation": self.projection.limitation,
            },
            "focusModel": {
                "physicalTargetCapability": self.physical_focus_capability,
                "reportedCapability": self.simulated_focus_capability,
                "status": "not_modeled",
            },
        }


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or not all(isinstance(key, str) for key in value):
        raise CameraProfileError(f"{label} must be an object")
    return value


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 512:
        raise CameraProfileError(f"{label} must be bounded text")
    return value.strip()


def _identifier(value: object, label: str) -> str:
    rendered = _text(value, label)
    if _SAFE_ID.fullmatch(rendered) is None:
        raise CameraProfileError(f"{label} must be a safe identifier")
    return rendered


def _positive_number(value: object, label: str, *, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CameraProfileError(f"{label} must be a number")
    converted = float(value)
    if not math.isfinite(converted) or not 0.0 < converted <= maximum:
        raise CameraProfileError(f"{label} is outside its allowed range")
    return converted


def _positive_int(value: object, label: str, *, maximum: int = 16_384) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= maximum:
        raise CameraProfileError(f"{label} must be a positive bounded integer")
    return value


def _render_profile(value: object, name: str) -> CameraRenderProfile:
    profile = _mapping(value, f"simulation.renderProfiles.{name}")
    limitation = profile.get("limitation")
    return CameraRenderProfile(
        name=name,
        width_px=_positive_int(profile.get("widthPx"), f"{name}.widthPx"),
        height_px=_positive_int(profile.get("heightPx"), f"{name}.heightPx"),
        limitation=(
            _text(limitation, f"{name}.limitation") if limitation is not None else None
        ),
    )


def _projection(value: object) -> CameraProjection:
    projection = _mapping(value, "simulation.projection")
    matched = _mapping(
        projection.get("matchedNominalFieldOfViewDeg"),
        "simulation.projection.matchedNominalFieldOfViewDeg",
    )
    model = _identifier(projection.get("model"), "simulation.projection.model")
    if model != "pinhole":
        raise CameraProfileError("the current Isaac camera supports only pinhole projection")
    horizontal_fov = _positive_number(
        matched.get("horizontal"), "projection.horizontalFovDeg", maximum=179.0
    )
    vertical_fov = _positive_number(
        matched.get("vertical"), "projection.verticalFovDeg", maximum=179.0
    )
    result = CameraProjection(
        model=model,
        focal_length_mm=_positive_number(
            projection.get("focalLengthMm"), "projection.focalLengthMm", maximum=1_000.0
        ),
        horizontal_aperture_mm=_positive_number(
            projection.get("horizontalApertureMm"),
            "projection.horizontalApertureMm",
            maximum=1_000.0,
        ),
        vertical_aperture_mm=_positive_number(
            projection.get("verticalApertureMm"),
            "projection.verticalApertureMm",
            maximum=1_000.0,
        ),
        horizontal_fov_deg=horizontal_fov,
        vertical_fov_deg=vertical_fov,
        status=_identifier(projection.get("status"), "projection.status"),
        limitation=_text(projection.get("limitation"), "projection.limitation"),
    )
    calculated_horizontal = math.degrees(
        2.0
        * math.atan(
            result.horizontal_aperture_mm / (2.0 * result.focal_length_mm)
        )
    )
    calculated_vertical = math.degrees(
        2.0
        * math.atan(result.vertical_aperture_mm / (2.0 * result.focal_length_mm))
    )
    if (
        abs(calculated_horizontal - horizontal_fov) > 0.01
        or abs(calculated_vertical - vertical_fov) > 0.01
    ):
        raise CameraProfileError("pinhole apertures do not match the declared nominal FOV")
    return result


def load_camera_profile(path: Path = CAMERA_PROFILE_PATH) -> CameraProfile:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        raise CameraProfileError("camera profile is unavailable or invalid JSON") from None
    root = _mapping(document, "camera profile")
    if root.get("schema") != CAMERA_PROFILE_SCHEMA:
        raise CameraProfileError("camera profile schema is unsupported")
    reference = _mapping(root.get("referenceHardware"), "referenceHardware")
    simulation = _mapping(root.get("simulation"), "simulation")
    render_profiles = _mapping(simulation.get("renderProfiles"), "simulation.renderProfiles")
    focus_model = _mapping(simulation.get("focusModel"), "simulation.focusModel")
    calibration = _mapping(root.get("calibration"), "calibration")
    intrinsics = _mapping(calibration.get("intrinsics"), "calibration.intrinsics")
    distortion = _mapping(calibration.get("distortion"), "calibration.distortion")
    extrinsics = _mapping(calibration.get("extrinsics"), "calibration.extrinsics")

    if any(
        intrinsics.get(field) is not None
        for field in ("widthPx", "heightPx", "fxPx", "fyPx", "cxPx", "cyPx")
    ):
        raise CameraProfileError("uncalibrated intrinsics must remain null")
    if distortion.get("model") is not None or distortion.get("coefficients") != []:
        raise CameraProfileError("uncalibrated distortion must remain null and empty")
    if extrinsics.get("translationM") is not None or extrinsics.get("rpyRad") is not None:
        raise CameraProfileError("uncalibrated extrinsics must remain null")

    return CameraProfile(
        profile_id=_identifier(root.get("profileId"), "profileId"),
        description=_text(root.get("description"), "description"),
        camera_id=_identifier(simulation.get("cameraId"), "simulation.cameraId"),
        sensor_model=_identifier(simulation.get("sensorModel"), "simulation.sensorModel"),
        identity_confidence=_identifier(
            simulation.get("identityConfidence"), "simulation.identityConfidence"
        ),
        reference_sensor_model=_identifier(
            reference.get("sensorModel"), "referenceHardware.sensorModel"
        ),
        physical_focus_capability=_identifier(
            reference.get("focusCapability"), "referenceHardware.focusCapability"
        ),
        simulated_focus_capability=_identifier(
            focus_model.get("reportedCapability"),
            "simulation.focusModel.reportedCapability",
        ),
        calibration_status=_identifier(
            calibration.get("status"), "calibration.status"
        ),
        survey=_render_profile(render_profiles.get("survey"), "survey"),
        detail=_render_profile(render_profiles.get("detail"), "detail"),
        projection=_projection(simulation.get("projection")),
        document=dict(root),
    )


__all__ = [
    "CAMERA_PROFILE_PATH",
    "CAMERA_PROFILE_SCHEMA",
    "CameraProfile",
    "CameraProfileError",
    "CameraProjection",
    "CameraRenderProfile",
    "load_camera_profile",
]
