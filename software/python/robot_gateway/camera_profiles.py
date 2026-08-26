"""Bounded optical and capture profiles for supported Raspberry Pi cameras.

The sensor can report its own model and pixel-array size through Picamera2,
but it cannot distinguish the standard and wide IMX708 lens variants.  The
deployment therefore selects the lens profile explicitly and the provider
cross-checks every driver-reported fact that is available.
"""

from __future__ import annotations

from dataclasses import dataclass


CAPTURE_PROFILE_SURVEY = "survey"
CAPTURE_PROFILE_DETAIL = "detail"
CAPTURE_PROFILES = frozenset({CAPTURE_PROFILE_SURVEY, CAPTURE_PROFILE_DETAIL})

MODULE3_WIDE_PROFILE_ID = "module3-wide"
OV5647_PROFILE_ID = "ov5647"
AUTO_PROFILE_ID = "auto"


@dataclass(frozen=True, slots=True)
class CaptureModeProfile:
    width: int
    height: int
    autofocus_ranges: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {"width": self.width, "height": self.height}


@dataclass(frozen=True, slots=True)
class CameraHardwareProfile:
    profile_id: str
    product_name: str
    sensor_model: str
    lens_variant: str
    native_width: int
    native_height: int
    nominal_focal_length_mm: float
    nominal_horizontal_fov_degrees: float
    nominal_vertical_fov_degrees: float
    survey: CaptureModeProfile
    detail: CaptureModeProfile

    def mode(self, capture_profile: str) -> CaptureModeProfile:
        normalized = normalize_capture_profile(capture_profile)
        return self.survey if normalized == CAPTURE_PROFILE_SURVEY else self.detail

    def as_dict(self) -> dict[str, object]:
        return {
            "id": self.profile_id,
            "productName": self.product_name,
            "sensorModel": self.sensor_model,
            "lensVariant": self.lens_variant,
            "nativeDimensions": {
                "width": self.native_width,
                "height": self.native_height,
            },
            "nominalFocalLengthMm": self.nominal_focal_length_mm,
            "nominalFieldOfViewDegrees": {
                "horizontal": self.nominal_horizontal_fov_degrees,
                "vertical": self.nominal_vertical_fov_degrees,
            },
            "captureProfiles": {
                CAPTURE_PROFILE_SURVEY: self.survey.as_dict(),
                CAPTURE_PROFILE_DETAIL: self.detail.as_dict(),
            },
        }


MODULE3_WIDE_PROFILE = CameraHardwareProfile(
    profile_id=MODULE3_WIDE_PROFILE_ID,
    product_name="Raspberry Pi Camera Module 3 Wide",
    sensor_model="imx708",
    lens_variant="wide",
    native_width=4608,
    native_height=2592,
    nominal_focal_length_mm=2.75,
    nominal_horizontal_fov_degrees=102.0,
    nominal_vertical_fov_degrees=67.0,
    survey=CaptureModeProfile(
        width=2304,
        height=1296,
        autofocus_ranges=("normal", "full", "unavailable"),
    ),
    detail=CaptureModeProfile(
        width=4608,
        height=2592,
        autofocus_ranges=("macro", "full", "normal", "unavailable"),
    ),
)

OV5647_PROFILE = CameraHardwareProfile(
    profile_id=OV5647_PROFILE_ID,
    product_name="Raspberry Pi Camera Module 1",
    sensor_model="ov5647",
    lens_variant="standard_fixed_focus",
    native_width=2592,
    native_height=1944,
    nominal_focal_length_mm=3.60,
    nominal_horizontal_fov_degrees=53.5,
    nominal_vertical_fov_degrees=41.41,
    survey=CaptureModeProfile(
        width=1296,
        height=972,
        autofocus_ranges=("unavailable",),
    ),
    detail=CaptureModeProfile(
        width=2592,
        height=1944,
        autofocus_ranges=("unavailable",),
    ),
)

CAMERA_PROFILES = {
    MODULE3_WIDE_PROFILE_ID: MODULE3_WIDE_PROFILE,
    OV5647_PROFILE_ID: OV5647_PROFILE,
}
CAMERA_PROFILE_CHOICES = (MODULE3_WIDE_PROFILE_ID, OV5647_PROFILE_ID, AUTO_PROFILE_ID)


def normalize_capture_profile(value: str) -> str:
    if not isinstance(value, str) or value not in CAPTURE_PROFILES:
        raise ValueError("capture profile must be survey or detail")
    return value


def camera_profile(value: str) -> CameraHardwareProfile:
    if not isinstance(value, str):
        raise ValueError("camera profile must be a string")
    try:
        return CAMERA_PROFILES[value]
    except KeyError:
        raise ValueError("camera profile is not supported") from None


def camera_profile_for_sensor(sensor_model: str) -> CameraHardwareProfile | None:
    if not isinstance(sensor_model, str):
        return None
    normalized = sensor_model.strip().lower()
    if normalized.startswith(("imx708_", "imx708-")):
        normalized = MODULE3_WIDE_PROFILE.sensor_model
    elif normalized.startswith(("ov5647_", "ov5647-")):
        normalized = OV5647_PROFILE.sensor_model
    if normalized == MODULE3_WIDE_PROFILE.sensor_model:
        # The driver reports IMX708 but not which lens assembly is fitted.  The
        # current deployment's auto fallback is intentionally the Wide profile.
        return MODULE3_WIDE_PROFILE
    if normalized == OV5647_PROFILE.sensor_model:
        return OV5647_PROFILE
    return None


__all__ = [
    "AUTO_PROFILE_ID",
    "CAMERA_PROFILE_CHOICES",
    "CAMERA_PROFILES",
    "CAPTURE_PROFILE_DETAIL",
    "CAPTURE_PROFILE_SURVEY",
    "CAPTURE_PROFILES",
    "CameraHardwareProfile",
    "CaptureModeProfile",
    "MODULE3_WIDE_PROFILE",
    "MODULE3_WIDE_PROFILE_ID",
    "OV5647_PROFILE",
    "OV5647_PROFILE_ID",
    "camera_profile",
    "camera_profile_for_sensor",
    "normalize_capture_profile",
]
