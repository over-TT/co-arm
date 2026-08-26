"""Narrow server-side client for the loopback robot gateway.

The browser never supplies an upstream URL/path and never receives the bearer token.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re
import secrets
from typing import Annotated, Literal
from urllib.parse import urlsplit

import httpx
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


MAX_REQUEST_BYTES = 65_536
MAX_RESPONSE_BYTES = 262_144
MAX_CAMERA_FRAME_BYTES = 16 * 1024 * 1024
CAMERA_CAPTURE_READ_TIMEOUT_SECONDS = 20.0
LIVE_FOLLOW_START_TIMEOUT_MS_MAX = 1_500
LIVE_FOLLOW_ADMISSION_READ_TIMEOUT_SECONDS = 2.0
LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS = 0.5
_CAMERA_CAPTURE_PROFILES = frozenset({"survey", "detail"})
_REAL_CAMERA_IDENTITY_CONFIDENCE = frozenset(
    {"configured_candidate", "driver_reported"}
)
_ALLOWED_CALLS = frozenset(
    {
        ("GET", "/healthz"),
        ("GET", "/api/camera/status"),
        ("GET", "/api/camera/observations/latest"),
        ("POST", "/api/camera/captures"),
        ("POST", "/api/camera/autofocus"),
        ("GET", "/api/robot/physical/arm/status"),
        ("POST", "/api/robot/physical/arm/controller/reconnect"),
        ("POST", "/api/robot/physical/arm/bus/scan"),
        ("POST", "/api/robot/physical/arm/servos/assign-id"),
        ("POST", "/api/robot/physical/arm/servos/set-position-mode"),
        ("POST", "/api/robot/physical/arm/servos/capture"),
        ("POST", "/api/robot/physical/arm/servos/move"),
        ("POST", "/api/robot/physical/arm/servos/registers/read"),
        ("POST", "/api/robot/physical/arm/servos/odometer/zero"),
        ("POST", "/api/robot/physical/arm/servos/odometer/read"),
        ("POST", "/api/robot/physical/arm/servos/torque-lease"),
        ("POST", "/api/robot/physical/arm/servos/hold-set"),
        ("POST", "/api/robot/physical/arm/servos/torque-off"),
        ("POST", "/api/robot/physical/arm/tests/prepare-nudge"),
        ("POST", "/api/robot/physical/arm/tests/execute-nudge"),
        ("POST", "/api/robot/physical/arm/stop"),
        ("POST", "/api/robot/physical/arm/reset"),
        ("GET", "/api/robot/physical/arm/calibration/profile"),
        ("POST", "/api/robot/physical/arm/calibration/profile"),
        # The simple arm: four numbers per joint, clamped once on the Pi.
        ("GET", "/api/robot/arm/state"),
        ("POST", "/api/robot/arm/scan"),
        ("POST", "/api/robot/arm/torque"),
        ("POST", "/api/robot/arm/servo-id"),
        ("POST", "/api/robot/arm/target"),
        ("POST", "/api/robot/arm/live-follow/start"),
        ("POST", "/api/robot/arm/live-follow/start/cancel"),
        ("POST", "/api/robot/arm/live-follow/frame"),
        ("POST", "/api/robot/arm/live-follow/heartbeat"),
        ("POST", "/api/robot/arm/live-follow/end"),
        ("POST", "/api/robot/arm/plans/preview"),
        ("POST", "/api/robot/arm/plans/execute"),
        ("POST", "/api/robot/arm/sequences/preview"),
        ("POST", "/api/robot/arm/sequences/execute"),
        ("POST", "/api/robot/arm/sequences/execute-and-capture"),
        ("POST", "/api/robot/arm/stop"),
        ("POST", "/api/robot/arm/clear-stop"),
        # Generated across every joint rather than hand-listed. Written out one
        # by one, this stopped at joint_3 while the router, the request models
        # and the gateway had all moved to four -- so every camera calibrate and
        # target 404ed here, at the one hop with no reason to know about it.
        *(
            ("POST", f"/api/robot/arm/joints/{joint}/{action}")
            for joint in ("joint_1", "joint_2", "joint_3", "joint_4")
            for action in ("calibrate", "target")
        ),
    }
)
_PROFILE_HASH_PATTERN = r"^sha256:[0-9a-f]{64}$"
_CAMERA_CONTENT_SHA_PATTERN = re.compile(r"^sha256:[0-9a-f]{64}$")
_CAMERA_SHA_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_SAFE_FRAME_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")


class RobotGatewayConfigurationError(ValueError):
    """Safe-to-display local proxy configuration failure."""


class RobotGatewayError(RuntimeError):
    """A sanitized upstream failure with a browser-safe status code."""

    def __init__(self, message: str, *, status_code: int) -> None:
        super().__init__(message)
        self.status_code = status_code


@dataclass(frozen=True, slots=True)
class CameraFrameResponse:
    content: bytes
    media_type: str
    headers: dict[str, str]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


class ArmGeometry(StrictModel):
    baseHeightMm: float
    upperArmMm: float
    forearmMm: float
    toolOffsetMm: float

    @field_validator("baseHeightMm", "upperArmMm", "forearmMm", "toolOffsetMm", mode="before")
    @classmethod
    def finite_geometry(cls, value: object, info: object) -> float:
        number = _finite(value, getattr(info, "field_name", "geometry"))
        if not 0.0 < number <= 1000.0:
            raise ValueError("geometry measurement must be above 0 and at most 1000 mm")
        return number

    @model_validator(mode="after")
    def combined_forearm_is_supported(self) -> "ArmGeometry":
        if self.forearmMm + self.toolOffsetMm > 1000.0:
            raise ValueError("combined forearm and tool offset must not exceed 1000 mm")
        return self


JointId = Literal["joint_1", "joint_2", "joint_3"]
LogicalName = Literal["base_yaw", "shoulder_pitch", "elbow_pitch"]
RawPosition = Annotated[int, Field(strict=True, ge=0, le=4095)]


class SimpleArmPlanTargets(StrictModel):
    joint_1: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_2: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_3: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_4: Annotated[float, Field(ge=-3600, le=3600)] | None = None


class SimpleArmLiveFollowJointSettings(StrictModel):
    speed: Annotated[int, Field(strict=True, ge=1, le=2400)]
    accel: Annotated[int, Field(strict=True, ge=1, le=50)]
    maxDeltaDegrees: Annotated[
        int, Field(strict=True, ge=1, le=90)
    ] = 30


class SimpleArmLiveFollowStartRequest(StrictModel):
    startAttemptId: Annotated[
        str,
        Field(
            min_length=8,
            max_length=128,
            pattern=r"^[A-Za-z0-9][A-Za-z0-9_-]{7,127}$",
        ),
    ]
    startTimeoutMs: Annotated[
        int,
        Field(strict=True, ge=1, le=LIVE_FOLLOW_START_TIMEOUT_MS_MAX),
    ] = LIVE_FOLLOW_START_TIMEOUT_MS_MAX
    joint_2: SimpleArmLiveFollowJointSettings
    joint_3: SimpleArmLiveFollowJointSettings


class SimpleArmLiveFollowStartCancelRequest(StrictModel):
    startAttemptId: Annotated[
        str,
        Field(
            min_length=8,
            max_length=128,
            pattern=r"^[A-Za-z0-9][A-Za-z0-9_-]{7,127}$",
        ),
    ]


class SimpleArmLiveFollowFrameRequest(StrictModel):
    sessionId: Annotated[
        str, Field(pattern=r"^armfollow_[A-Za-z0-9_-]{16,64}$")
    ]
    sequence: Annotated[int, Field(strict=True, ge=1)]
    joint_2: Annotated[float, Field(ge=-3600, le=3600)]
    joint_3: Annotated[float, Field(ge=-3600, le=3600)]


class SimpleArmLiveFollowHeartbeatRequest(StrictModel):
    sessionId: Annotated[
        str, Field(pattern=r"^armfollow_[A-Za-z0-9_-]{16,64}$")
    ]


class SimpleArmLiveFollowEndRequest(StrictModel):
    sessionId: Annotated[
        str, Field(pattern=r"^armfollow_[A-Za-z0-9_-]{16,64}$")
    ]
    flushPending: Annotated[bool, Field(strict=True)]


class SimpleArmPlanTip(StrictModel):
    radialMm: Annotated[float, Field(ge=-450, le=450)]
    heightMm: Annotated[float, Field(ge=-400, le=600)]


class SimpleArmPlanPreviewRequest(StrictModel):
    targets: SimpleArmPlanTargets
    tip: SimpleArmPlanTip | None = None
    elbowPreference: Literal["nearest", "up", "down"] = "nearest"

    @model_validator(mode="after")
    def one_pose_source(self) -> "SimpleArmPlanPreviewRequest":
        supplied = self.targets.model_dump(exclude_none=True)
        if self.tip is None and not supplied:
            raise ValueError("name at least one joint target or provide a tip")
        if self.tip is not None and any(name in supplied for name in ("joint_2", "joint_3")):
            raise ValueError("tip inverse kinematics owns joint_2 and joint_3")
        return self


class SimpleArmPlanExecuteRequest(StrictModel):
    planId: Annotated[
        str, Field(pattern=r"^armplan_[A-Za-z0-9_-]{8,64}$")
    ]
    planDigest: Annotated[str, Field(pattern=_PROFILE_HASH_PATTERN)]


class SimpleArmSequenceWaypoint(SimpleArmPlanPreviewRequest):
    label: Annotated[
        str, Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,39}$")
    ] | None = None


class SimpleArmSequencePreviewRequest(StrictModel):
    waypoints: Annotated[
        list[SimpleArmSequenceWaypoint], Field(min_length=2, max_length=8)
    ]


class SimpleArmSequenceExecuteRequest(StrictModel):
    sequenceId: Annotated[
        str, Field(pattern=r"^armseq_[A-Za-z0-9_-]{8,64}$")
    ]
    sequenceDigest: Annotated[str, Field(pattern=_PROFILE_HASH_PATTERN)]
    arrivalTimeoutMs: Annotated[
        int, Field(strict=True, ge=500, le=12_000)
    ] = 10_000


class SimpleArmSequenceExecuteCaptureRequest(SimpleArmSequenceExecuteRequest):
    """A reviewed sequence whose final evidence uses one explicit camera profile."""

    captureProfile: Literal["survey", "detail"] = "detail"


ServoId = Annotated[int, Field(strict=True, ge=0, le=253)]


class PhysicalScanRequest(StrictModel):
    minId: ServoId = 0
    maxId: ServoId = 20

    @model_validator(mode="after")
    def ordered_range(self) -> "PhysicalScanRequest":
        if self.minId > self.maxId:
            raise ValueError("minId must not exceed maxId")
        return self


class AssignServoIdRequest(StrictModel):
    oldId: ServoId
    newId: ServoId
    acknowledgedSingleServo: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedSingleServo", mode="before")
    @classmethod
    def exact_single_servo_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedSingleServo must be the boolean true")
        return True

    @model_validator(mode="after")
    def distinct_ids(self) -> "AssignServoIdRequest":
        if self.oldId == self.newId:
            raise ValueError("newId must differ from oldId")
        return self


class SetServoPositionModeRequest(StrictModel):
    servoId: ServoId
    acknowledgedSingleServo: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedSingleServo", mode="before")
    @classmethod
    def exact_single_servo_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedSingleServo must be the boolean true")
        return True


class ServoCaptureRequest(StrictModel):
    servoId: ServoId


class ServoOdometerRequest(StrictModel):
    servoId: ServoId


class ServoMoveRequest(StrictModel):
    servoId: ServoId
    goal: Annotated[int, Field(strict=True, ge=0, le=4095)]
    speed: Annotated[int, Field(strict=True, ge=1, le=4095)]
    acceleration: Annotated[int, Field(strict=True, ge=1, le=255)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]


class ServoRegisterReadRequest(StrictModel):
    servoId: ServoId
    address: Annotated[int, Field(strict=True, ge=0, le=255)]
    length: Annotated[int, Field(strict=True, ge=1, le=16)]


class TorqueLeaseRequest(StrictModel):
    servoId: ServoId
    leaseMs: Annotated[int, Field(strict=True, ge=100, le=2000)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True


class TorqueOffRequest(StrictModel):
    servoId: ServoId | None = None


class HoldSetRequest(StrictModel):
    servoIds: Annotated[list[ServoId], Field(min_length=0, max_length=3)]
    leaseMs: Annotated[int, Field(strict=True, ge=100, le=2_000)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True

    @model_validator(mode="after")
    def unique_servo_ids(self) -> "HoldSetRequest":
        if len(set(self.servoIds)) != len(self.servoIds):
            raise ValueError("servoIds must be unique")
        return self


class PrepareNudgeRequest(StrictModel):
    servoId: ServoId
    # Mirrors the gateway's own bounds; a tighter proxy would reject a legal
    # command with a confusing "invalid parameters" instead of the real reason.
    deltaTicks: Annotated[int, Field(strict=True, ge=-512, le=512)]
    speed: Annotated[int, Field(strict=True, ge=1, le=2_400)]
    acceleration: Annotated[int, Field(strict=True, ge=1, le=50)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]
    zeroEvidenceId: Annotated[
        str,
        Field(strict=True, min_length=8, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
    ]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True

    @model_validator(mode="after")
    def nonzero_delta(self) -> "PrepareNudgeRequest":
        if self.deltaTicks == 0:
            raise ValueError("deltaTicks must not be zero")
        return self


class ExecuteNudgeRequest(StrictModel):
    proposalId: Annotated[
        str,
        Field(strict=True, min_length=8, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
    ]
    proposalHash: Annotated[str, Field(strict=True, pattern=_PROFILE_HASH_PATTERN)] | None = None
    acknowledgedPhysicalPowerCut: Literal[True]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True


class PhysicalResetRequest(StrictModel):
    acknowledgedPhysicalInspection: Literal[True]


EvidenceId = Annotated[
    str,
    Field(strict=True, min_length=4, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
]


class PhysicalJointProfile(StrictModel):
    logicalId: JointId
    name: LogicalName
    servoId: ServoId
    rawZero: RawPosition
    direction: Literal[-1, 1]
    motorTurnsPerJointTurn: float = 1.0
    negativeLimitTicks: Annotated[int, Field(strict=True, ge=-4095, le=-1)]
    positiveLimitTicks: Annotated[int, Field(strict=True, ge=1, le=4095)]
    limitMarginTicks: Annotated[int, Field(strict=True, ge=1, le=512)]
    maxVelocityDegreesPerSecond: float
    maxAccelerationDegreesPerSecond2: float
    evidenceIds: Annotated[list[EvidenceId], Field(min_length=3, max_length=32)]

    @field_validator(
        "motorTurnsPerJointTurn",
        "maxVelocityDegreesPerSecond",
        "maxAccelerationDegreesPerSecond2",
        mode="before",
    )
    @classmethod
    def finite_motion_limits(cls, value: object, info: object) -> float:
        field_name = getattr(info, "field_name", "motion limit")
        number = _finite(value, field_name)
        if field_name == "motorTurnsPerJointTurn" and not 0.01 <= number <= 64.0:
            raise ValueError("motorTurnsPerJointTurn must be between 0.01 and 64")
        return number

    @model_validator(mode="after")
    def safe_position_mode_range(self) -> "PhysicalJointProfile":
        if abs(self.negativeLimitTicks) <= self.limitMarginTicks:
            raise ValueError("negative range must remain after the inward margin")
        if self.positiveLimitTicks <= self.limitMarginTicks:
            raise ValueError("positive range must remain after the inward margin")
        negative_raw = self.rawZero + self.direction * self.negativeLimitTicks
        positive_raw = self.rawZero + self.direction * self.positiveLimitTicks
        if not (0 <= negative_raw <= 4095 and 0 <= positive_raw <= 4095):
            raise ValueError(
                "ST3215 position-mode limits must remain inside raw 0 through 4095"
            )
        if not 0.0 < self.maxVelocityDegreesPerSecond <= 60.0:
            raise ValueError("commissioning velocity must be above 0 and at most 60 deg/s")
        if not 0.0 < self.maxAccelerationDegreesPerSecond2 <= 120.0:
            raise ValueError("commissioning acceleration must be above 0 and at most 120 deg/s^2")
        if len(set(self.evidenceIds)) != len(self.evidenceIds):
            raise ValueError("evidence identifiers must be distinct")
        return self


class PhysicalProfileJoints(StrictModel):
    joint_1: PhysicalJointProfile
    joint_2: PhysicalJointProfile
    joint_3: PhysicalJointProfile

    @model_validator(mode="after")
    def fixed_identity_and_unique_servos(self) -> "PhysicalProfileJoints":
        expected = {
            "joint_1": "base_yaw",
            "joint_2": "shoulder_pitch",
            "joint_3": "elbow_pitch",
        }
        ids: list[int] = []
        for logical_id, name in expected.items():
            profile = getattr(self, logical_id)
            if profile.logicalId != logical_id or profile.name != name:
                raise ValueError(f"{logical_id} identity mapping does not match")
            ids.append(profile.servoId)
        if len(set(ids)) != 3:
            raise ValueError("all three physical servo IDs must be distinct")
        return self


class PhysicalCalibrationProfileRequest(StrictModel):
    expectedControllerBootId: Annotated[
        str,
        Field(strict=True, min_length=4, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
    ]
    joints: PhysicalProfileJoints
    geometry: ArmGeometry
    acknowledgedRemainDisarmed: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedRemainDisarmed", mode="before")
    @classmethod
    def exact_disarmed_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedRemainDisarmed must be the boolean true")
        return True


def _validated_base_url(value: str) -> str:
    try:
        parsed = urlsplit(value)
        port = parsed.port
    except (TypeError, ValueError):
        raise RobotGatewayConfigurationError("Robot gateway URL is invalid.") from None
    if (
        parsed.scheme != "http"
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in ("", "/")
        or port is None
        or port < 1
    ):
        raise RobotGatewayConfigurationError(
            "Robot gateway URL must be an HTTP loopback origin with an explicit port."
        )
    host = parsed.hostname
    is_loopback = host == "localhost"
    if host is not None and not is_loopback:
        try:
            is_loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            is_loopback = False
    if not is_loopback or host not in {"127.0.0.1", "::1", "localhost"}:
        raise RobotGatewayConfigurationError("Robot gateway URL must use loopback only.")
    rendered_host = f"[{host}]" if host == "::1" else host
    return f"http://{rendered_host}:{port}"


def _load_token(path: str | Path) -> str:
    token_path = Path(path)
    try:
        if token_path.stat().st_size > 1024:
            raise OSError
        token = token_path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        raise RobotGatewayConfigurationError("Unable to read the robot gateway token file.") from None
    if not 32 <= len(token) <= 256 or any(
        character.isspace() or not character.isprintable() for character in token
    ):
        raise RobotGatewayConfigurationError("Robot gateway token file is invalid.")
    return token


def _number(value: object, label: str) -> float:
    try:
        return _finite(value, label)
    except ValueError:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502) from None


def _integer(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return value


def _bounded_integer(value: object, label: str, *, minimum: int, maximum: int) -> int:
    number = _integer(value)
    if not minimum <= number <= maximum:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return number


def _camera_text(value: object, *, maximum: int = 256) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum or not value.isprintable():
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return value


def _camera_capture_profile(value: object) -> str:
    if value not in _CAMERA_CAPTURE_PROFILES:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    assert isinstance(value, str)
    return value


def _camera_identity_confidence(value: object, *, simulated: bool) -> str:
    expected = {"simulated_model"} if simulated else _REAL_CAMERA_IDENTITY_CONFIDENCE
    if value not in expected:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    assert isinstance(value, str)
    return value


def _camera_dimensions(value: object, label: str) -> dict[str, int]:
    if not isinstance(value, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "width": _bounded_integer(
            value.get("width"), f"{label}.width", minimum=1, maximum=65_535
        ),
        "height": _bounded_integer(
            value.get("height"), f"{label}.height", minimum=1, maximum=65_535
        ),
    }


def _camera_hardware_profile(value: object) -> dict[str, object]:
    """Strip a camera profile to the public, bounded cross-layer contract."""

    if not isinstance(value, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    capture_profiles = value.get("captureProfiles")
    field_of_view = value.get("nominalFieldOfViewDegrees")
    if not isinstance(capture_profiles, dict) or not isinstance(field_of_view, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    horizontal = _number(
        field_of_view.get("horizontal"), "cameraProfile.nominalFieldOfViewDegrees.horizontal"
    )
    vertical = _number(
        field_of_view.get("vertical"), "cameraProfile.nominalFieldOfViewDegrees.vertical"
    )
    focal_length = _number(
        value.get("nominalFocalLengthMm"), "cameraProfile.nominalFocalLengthMm"
    )
    if not 0 < horizontal < 180 or not 0 < vertical < 180 or not 0 < focal_length <= 1_000:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "id": _camera_text(value.get("id"), maximum=64),
        "productName": _camera_text(value.get("productName"), maximum=128),
        "sensorModel": _camera_text(value.get("sensorModel"), maximum=64),
        "lensVariant": _camera_text(value.get("lensVariant"), maximum=64),
        "nativeDimensions": _camera_dimensions(
            value.get("nativeDimensions"), "cameraProfile.nativeDimensions"
        ),
        "nominalFocalLengthMm": focal_length,
        "nominalFieldOfViewDegrees": {
            "horizontal": horizontal,
            "vertical": vertical,
        },
        "captureProfiles": {
            profile: _camera_dimensions(
                capture_profiles.get(profile), f"cameraProfile.captureProfiles.{profile}"
            )
            for profile in ("survey", "detail")
        },
    }


def _camera_frame_id(value: object) -> str:
    if not isinstance(value, str) or _SAFE_FRAME_ID.fullmatch(value) is None:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return value


def _camera_autofocus(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    capability = value.get("capability")
    mode = value.get("mode")
    state = value.get("state")
    focus_range = value.get("range")
    if capability not in {"unknown", "supported", "unsupported"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if mode not in {"unknown", "continuous", "fixed"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if state not in {
        "not_started",
        "configured",
        "configuration_failed",
        "fixed",
        "idle",
        "scanning",
        "focused",
        "failed",
        "closed",
    }:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if focus_range not in {"unavailable", "normal", "full", "macro"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    sanitized: dict[str, object] = {
        "capability": capability,
        "mode": mode,
        "state": state,
        "range": focus_range,
    }
    if "lensPosition" in value:
        sanitized["lensPosition"] = _number(value.get("lensPosition"), "lensPosition")
    return sanitized


def _camera_autofocus_attempt(
    document: dict[str, object], *, expected_simulated: bool = False
) -> dict[str, object]:
    if (
        document.get("simulated") is not expected_simulated
        or document.get("physicalArmMotion") is not False
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    attempted = document.get("attempted")
    result = document.get("result")
    if not isinstance(attempted, bool) or result not in {
        "focused",
        "not_focused",
        "unsupported",
        "unavailable",
        "timed_out",
    }:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    expected_attempted = result in {"focused", "not_focused", "timed_out"}
    if attempted is not expected_attempted:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    autofocus = _camera_autofocus(document.get("autofocus"))
    capability = autofocus["capability"]
    if result == "unsupported" and capability != "unsupported":
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if result in {"focused", "not_focused", "timed_out"} and capability != "supported":
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if result == "unavailable" and capability == "unsupported":
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "simulated": expected_simulated,
        "physicalArmMotion": False,
        "attempted": attempted,
        "result": result,
        "autofocus": autofocus,
    }


def _camera_focus_quality(value: object) -> dict[str, object]:
    """Sanitize libcamera's relative-only same-frame focus indication."""

    if not isinstance(value, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if (
        value.get("metric") != "libcamera_focus_fom"
        or value.get("status") not in {"measured", "unavailable"}
        or value.get("higherIsSharper") is not True
        or value.get("comparison") != "same_subject_similar_framing_only"
        or "value" not in value
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)

    status_value = value["status"]
    measured = value["value"]
    if status_value == "unavailable":
        if measured is not None:
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        bounded_value: int | None = None
    else:
        bounded_value = _bounded_integer(
            measured,
            "focusQuality.value",
            minimum=0,
            maximum=2_147_483_647,
        )

    return {
        "metric": "libcamera_focus_fom",
        "status": status_value,
        "value": bounded_value,
        "higherIsSharper": True,
        "comparison": "same_subject_similar_framing_only",
    }


def _camera_status(
    document: dict[str, object], *, expected_simulated: bool = False
) -> dict[str, object]:
    if (
        document.get("simulated") is not expected_simulated
        or document.get("readOnly") is not True
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    state = document.get("state")
    if state not in {"new", "started", "unavailable", "closed"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    available = document.get("available")
    if not isinstance(available, bool) or available is not (state == "started"):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    latest = document.get("latestFrameId")
    if latest is not None:
        latest = _camera_frame_id(latest)
    history_size = _bounded_integer(document.get("historySize"), "historySize", minimum=0, maximum=8)
    history_limit = _bounded_integer(document.get("historyLimit"), "historyLimit", minimum=1, maximum=8)
    retained = _bounded_integer(
        document.get("retainedBytes"), "retainedBytes", minimum=0, maximum=64 * 1024 * 1024
    )
    byte_limit = _bounded_integer(
        document.get("byteLimit"), "byteLimit", minimum=1, maximum=64 * 1024 * 1024
    )
    if history_size > history_limit or retained > byte_limit:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    identity_confidence = _camera_identity_confidence(
        document.get("identityConfidence"), simulated=expected_simulated
    )
    result: dict[str, object] = {
        "simulated": expected_simulated,
        "readOnly": True,
        "cameraId": _camera_text(document.get("cameraId")),
        "sensorModel": _camera_text(document.get("sensorModel")),
        "identityConfidence": identity_confidence,
        "state": state,
        "available": available,
        "latestFrameId": latest,
        "historySize": history_size,
        "historyLimit": history_limit,
        "retainedBytes": retained,
        "byteLimit": byte_limit,
    }
    if "autofocus" in document:
        result["autofocus"] = _camera_autofocus(document.get("autofocus"))
    if "cameraProfile" in document:
        result["cameraProfile"] = _camera_hardware_profile(document.get("cameraProfile"))
    return result


def _camera_observation(
    document: dict[str, object],
    *,
    expected_simulated: bool = False,
    expected_source: str = "picamera2",
    expected_identity_confidence: str = "configured_candidate",
) -> dict[str, object]:
    if "captureProfile" not in document:
        raise RobotGatewayError(
            (
                "The simulator camera response did not identify its capture profile. "
                "Restart the matching simulator stack."
                if expected_simulated
                else "The Pi camera package is older than this dashboard and did not identify "
                "the capture profile. Deploy the current Module 3 Wide gateway package, then "
                "restart arm-gateway.service."
            ),
            status_code=502,
        )
    if (
        document.get("simulated") is not expected_simulated
        or document.get("readOnly") is not True
        or document.get("source") != expected_source
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    frame_id = _camera_frame_id(document.get("frameId"))
    dimensions = _camera_dimensions(document.get("dimensions"), "dimensions")
    identity_confidence = _camera_identity_confidence(
        document.get("identityConfidence"), simulated=expected_simulated
    )
    age_seconds = _number(document.get("ageSeconds"), "ageSeconds")
    if age_seconds < 0:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    digest = document.get("sha256")
    content_digest = document.get("contentSha256")
    if (
        not isinstance(digest, str)
        or _CAMERA_SHA_PATTERN.fullmatch(digest) is None
        or not isinstance(content_digest, str)
        or _CAMERA_CONTENT_SHA_PATTERN.fullmatch(content_digest) is None
        or not secrets.compare_digest(content_digest, f"sha256:{digest}")
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    result: dict[str, object] = {
        "simulated": expected_simulated,
        "readOnly": True,
        "source": expected_source,
        "frameId": frame_id,
        "cameraId": _camera_text(document.get("cameraId")),
        "sensorModel": _camera_text(document.get("sensorModel")),
        "identityConfidence": identity_confidence,
        "capturedAt": _camera_text(document.get("capturedAt"), maximum=64),
        "ageSeconds": age_seconds,
        "dimensions": dimensions,
        "captureProfile": _camera_capture_profile(document.get("captureProfile")),
        "byteCount": _bounded_integer(
            document.get("byteCount"),
            "byteCount",
            minimum=1,
            maximum=MAX_CAMERA_FRAME_BYTES,
        ),
        "sha256": digest,
        "contentSha256": content_digest,
        "stateRevision": _integer(document.get("stateRevision")),
        "frameUrl": f"/api/camera/frames/{frame_id}",
    }
    if "autofocus" in document:
        result["autofocus"] = _camera_autofocus(document.get("autofocus"))
    if "focusQuality" in document:
        result["focusQuality"] = _camera_focus_quality(document.get("focusQuality"))
    if "cameraProfile" in document:
        result["cameraProfile"] = _camera_hardware_profile(document.get("cameraProfile"))
    return result


def _bounded_signed_integer(value: object, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return value


def _physical_text(value: object, *, maximum: int = 512) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > maximum
        or not value.isprintable()
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return value


def _id_list(value: object, *, maximum_items: int = 254) -> list[int]:
    if not isinstance(value, list) or len(value) > maximum_items:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    result = [
        _bounded_integer(item, "servoId", minimum=0, maximum=253)
        for item in value
    ]
    if len(set(result)) != len(result):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return result


def _physical_scan_result(document: dict[str, object]) -> dict[str, object]:
    complete = document.get("completeRange")
    if not isinstance(complete, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    minimum = _bounded_integer(complete.get("minId"), "minId", minimum=0, maximum=253)
    maximum = _bounded_integer(complete.get("maxId"), "maxId", minimum=0, maximum=253)
    if minimum > maximum:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    found = _id_list(document.get("foundIds"))
    if any(item < minimum or item > maximum for item in found):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {"foundIds": found, "completeRange": {"minId": minimum, "maxId": maximum}}


def _physical_position_mode_result(document: dict[str, object]) -> dict[str, object]:
    servo_id = _bounded_integer(document.get("servoId"), "servoId", minimum=0, maximum=253)
    previous_mode = _bounded_integer(
        document.get("previousOperatingMode"),
        "previousOperatingMode",
        minimum=0,
        maximum=3,
    )
    minimum = _bounded_integer(
        document.get("minimumPosition"), "minimumPosition", minimum=0, maximum=4095
    )
    maximum = _bounded_integer(
        document.get("maximumPosition"), "maximumPosition", minimum=0, maximum=4095
    )
    if (
        minimum >= maximum
        or document.get("operatingMode") != 0
        or document.get("verified") is not True
        or document.get("locked") is not True
        or document.get("torqueState") != "off"
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "servoId": servo_id,
        "previousOperatingMode": previous_mode,
        "operatingMode": 0,
        "verified": True,
        "locked": True,
        "torqueState": "off",
        "minimumPosition": minimum,
        "maximumPosition": maximum,
    }


def _physical_capture_result(document: dict[str, object]) -> dict[str, object]:
    evidence_id = document.get("evidenceId")
    if not isinstance(evidence_id, str) or _SAFE_FRAME_ID.fullmatch(evidence_id) is None:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    result: dict[str, object] = {
        "servoId": _bounded_integer(document.get("servoId"), "servoId", minimum=0, maximum=253),
        "rawPosition": _bounded_integer(
            document.get("rawPosition"), "rawPosition", minimum=0, maximum=4095
        ),
        "sampleCount": _bounded_integer(
            document.get("sampleCount"), "sampleCount", minimum=5, maximum=100
        ),
        "variationTicks": _bounded_integer(
            document.get("variationTicks", 0), "variationTicks", minimum=0, maximum=4095
        ),
        "evidenceId": evidence_id,
    }
    if document.get("odometerValid") is True:
        result["odometerValid"] = True
        result["revolutions"] = _bounded_integer(
            document.get("revolutions"), "revolutions", minimum=-64, maximum=64
        )
        result["multiTurnPosition"] = _bounded_integer(
            document.get("multiTurnPosition"),
            "multiTurnPosition",
            minimum=-266_240,
            maximum=266_240,
        )
    else:
        result["odometerValid"] = False
    captured_at = document.get("capturedAt")
    if captured_at is not None:
        result["capturedAt"] = _physical_text(captured_at, maximum=64)
    return result


def _physical_odometer_result(document: dict[str, object]) -> dict[str, object]:
    tracking = document.get("tracking")
    valid = document.get("valid")
    if not isinstance(tracking, bool) or not isinstance(valid, bool):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "servoId": _bounded_integer(document.get("servoId"), "servoId", minimum=0, maximum=253),
        "tracking": tracking,
        "valid": valid,
        "revolutions": _bounded_integer(
            document.get("revolutions"), "revolutions", minimum=-64, maximum=64
        ),
        "rawPosition": _bounded_integer(
            document.get("rawPosition"), "rawPosition", minimum=0, maximum=4095
        ),
        "multiTurnPosition": _bounded_integer(
            document.get("multiTurnPosition"),
            "multiTurnPosition",
            minimum=-266_240,
            maximum=266_240,
        ),
        "sampleAgeMs": _bounded_integer(
            document.get("sampleAgeMs"), "sampleAgeMs", minimum=0, maximum=86_400_000
        ),
    }


def _physical_nudge_result(
    document: dict[str, object], request: ExecuteNudgeRequest
) -> dict[str, object]:
    if (
        document.get("proposalId") != request.proposalId
        or document.get("completed") is not True
        or document.get("torqueState") != "off"
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    start = _bounded_integer(
        document.get("startRawPosition"), "startRawPosition", minimum=0, maximum=4095
    )
    target = _bounded_integer(
        document.get("targetRawPosition"), "targetRawPosition", minimum=0, maximum=4095
    )
    final = _bounded_integer(
        document.get("rawPosition"), "rawPosition", minimum=0, maximum=4095
    )
    measured = _bounded_signed_integer(
        document.get("measuredDeltaTicks"), minimum=-64, maximum=64
    )
    error = _bounded_signed_integer(
        document.get("positionErrorTicks"), minimum=-64, maximum=64
    )
    planned = target - start
    if (
        planned == 0
        or abs(planned) > 64
        or measured != final - start
        or error != final - target
        or measured * planned <= 0
        or abs(measured - planned) > 8
        or abs(error) > 8
    ):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    result: dict[str, object] = {
        "proposalId": request.proposalId,
        "completed": True,
        "startRawPosition": start,
        "targetRawPosition": target,
        "rawPosition": final,
        "measuredDeltaTicks": measured,
        "positionErrorTicks": error,
        "torqueState": "off",
    }
    evidence_id = document.get("evidenceId")
    if not isinstance(evidence_id, str) or _SAFE_FRAME_ID.fullmatch(evidence_id) is None:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    result["evidenceId"] = evidence_id
    return result


def _physical_servo(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    torque = value.get("torqueState")
    if torque not in {"off", "on", "unknown"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    errors = value.get("errors")
    if not isinstance(errors, list) or len(errors) > 16:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    normalized_errors = [_physical_text(item, maximum=64) for item in errors]
    moving = value.get("moving")
    if not isinstance(moving, bool):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    current = value.get("currentMilliamps")
    normalized: dict[str, object] = {
        "id": _bounded_integer(value.get("id"), "servoId", minimum=0, maximum=253),
        "rawPosition": _bounded_integer(
            value.get("rawPosition"), "rawPosition", minimum=0, maximum=4095
        ),
        "speed": _bounded_signed_integer(value.get("speed"), minimum=-32767, maximum=32767),
        "load": _bounded_signed_integer(value.get("load"), minimum=-1000, maximum=1000),
        "voltageVolts": _number(value.get("voltageVolts"), "voltageVolts"),
        "temperatureC": _number(value.get("temperatureC"), "temperatureC"),
        "moving": moving,
        "torqueState": torque,
        "packetAgeMs": _bounded_integer(
            value.get("packetAgeMs"), "packetAgeMs", minimum=0, maximum=86_400_000
        ),
        "errors": normalized_errors,
    }
    if not 0.0 <= float(normalized["voltageVolts"]) <= 30.0:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if not -40.0 <= float(normalized["temperatureC"]) <= 150.0:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    if current is not None:
        normalized["currentMilliamps"] = _bounded_signed_integer(
            current, minimum=-20_000, maximum=20_000
        )
    if "currentRaw" in value:
        normalized["currentRaw"] = _bounded_integer(
            value.get("currentRaw"), "currentRaw", minimum=0, maximum=65_535
        )
    if "statusError" in value:
        normalized["statusError"] = _bounded_integer(
            value.get("statusError"), "statusError", minimum=0, maximum=255
        )
    for field in ("online", "fresh"):
        if field in value:
            flag = value.get(field)
            if not isinstance(flag, bool):
                raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
            normalized[field] = flag
    if "operatingMode" in value:
        operating_mode = value.get("operatingMode")
        if operating_mode is None:
            normalized["operatingMode"] = None
        else:
            normalized["operatingMode"] = _bounded_integer(
                operating_mode, "operatingMode", minimum=0, maximum=255
            )
    return normalized


def _physical_status(document: dict[str, object]) -> dict[str, object]:
    if document.get("mode") != "physical":
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    motion = document.get("motionState")
    torque = document.get("torqueState")
    if motion not in {
        "blocked", "disarmed", "commissioning", "holding", "stopped", "faulted",
        "ready_disarmed",
    } or torque not in {"off", "on", "unknown"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    connections = document.get("connections")
    if not isinstance(connections, dict):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)

    def base_connection(layer: str, states: set[str]) -> tuple[dict[str, object], dict[str, object]]:
        raw = connections.get(layer)
        if not isinstance(raw, dict) or raw.get("state") not in states:
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        return raw, {
            "state": raw["state"],
            "detail": _physical_text(raw.get("detail")),
        }

    _, pi = base_connection("pi", {"online"})
    controller_raw, controller = base_connection(
        "controller", {"online", "offline", "not_configured", "faulted"}
    )
    for key in ("controllerId", "bootId", "firmwareVersion"):
        candidate = controller_raw.get(key)
        if candidate is not None:
            controller[key] = _physical_text(candidate, maximum=128)
    if controller_raw.get("protocolVersion") is not None:
        controller["protocolVersion"] = _bounded_integer(
            controller_raw.get("protocolVersion"), "protocolVersion", minimum=1, maximum=255
        )
    if controller_raw.get("lastSeenMs") is not None:
        controller["lastSeenMs"] = _bounded_integer(
            controller_raw.get("lastSeenMs"), "lastSeenMs", minimum=0, maximum=86_400_000
        )

    bus_raw, bus = base_connection("bus", {"online", "offline", "unknown", "degraded"})
    bus["baud"] = _bounded_integer(bus_raw.get("baud"), "baud", minimum=1, maximum=4_000_000)
    last_scan = bus_raw.get("lastScan")
    if last_scan is not None:
        if not isinstance(last_scan, dict):
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        bus["lastScan"] = _physical_scan_result(
            {"foundIds": last_scan.get("foundIds"), "completeRange": last_scan}
        )["completeRange"] | {"foundIds": _id_list(last_scan.get("foundIds"))}

    servo_link_raw, servo_link = base_connection(
        "servos", {"online", "partial", "none", "unknown", "collision"}
    )
    servo_link["expectedIds"] = _id_list(servo_link_raw.get("expectedIds"))
    servo_link["respondingIds"] = _id_list(servo_link_raw.get("respondingIds"))

    servos_raw = document.get("servos")
    if not isinstance(servos_raw, list) or len(servos_raw) > 16:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    servos = [_physical_servo(item) for item in servos_raw]
    if len({item["id"] for item in servos}) != len(servos):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)

    blockers_raw = document.get("blockers")
    if not isinstance(blockers_raw, list) or len(blockers_raw) > 32:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    blockers: list[dict[str, str]] = []
    for item in blockers_raw:
        if not isinstance(item, dict):
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        blockers.append(
            {
                "code": _physical_text(item.get("code"), maximum=64),
                "message": _physical_text(item.get("message")),
                "recovery": _physical_text(item.get("recovery")),
            }
        )

    stop_raw = document.get("stop")
    if not isinstance(stop_raw, dict) or stop_raw.get("state") not in {
        "confirmed", "not_latched", "unknown"
    }:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    hardware_estop = document.get("hardwareEstop")
    if hardware_estop not in {"not_detected", "active", "released", "unknown"}:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    last_update = document.get("lastUpdateMs")
    if last_update is not None:
        last_update = _bounded_integer(
            last_update, "lastUpdateMs", minimum=0, maximum=86_400_000
        )
    return {
        "mode": "physical",
        "motionState": motion,
        "torqueState": torque,
        "lastUpdateMs": last_update,
        "connections": {
            "pi": pi,
            "controller": controller,
            "bus": bus,
            "servos": servo_link,
        },
        "servos": servos,
        "blockers": blockers,
        "nextAction": _physical_text(document.get("nextAction")),
        "stop": {
            "state": stop_raw["state"],
            "detail": _physical_text(stop_raw.get("detail")),
        },
        "hardwareEstop": hardware_estop,
    }


def _physical_profile_response(document: dict[str, object]) -> dict[str, object]:
    if document.get("mode") != "physical" or document.get("motionState") not in {
        "blocked", "faulted", "disarmed", "ready_disarmed"
    }:
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    committed = document.get("committed", False)
    if not isinstance(committed, bool):
        raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    revision = _bounded_integer(
        document.get("profileRevision"), "profileRevision", minimum=0, maximum=2**31 - 1
    )
    profile = document.get("profile")
    profile_hash = document.get("profileHash")
    if profile is None:
        if profile_hash is not None:
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    else:
        if not isinstance(profile, dict):
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        if not isinstance(profile_hash, str) or re.fullmatch(
            _PROFILE_HASH_PATTERN, profile_hash
        ) is None:
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
    return {
        "mode": "physical",
        "committed": committed,
        "motionState": document["motionState"],
        "profileRevision": revision,
        "profileHash": profile_hash,
        "profile": profile,
    }


# The controller reports precisely why it refused; a single generic conflict message
# sends the operator looking for a browser problem instead of the real bus condition.
_CONFLICT_MESSAGES = {
    "NOT_SINGLE_SERVO": (
        "The controller needs exactly one servo on the bus for this command. "
        "Disconnect the other servos, run it for this servo alone, then reconnect them."
    ),
    "STOPPED": "STOP is latched. Reset STOP before running this command.",
    "HEARTBEAT_STALE": "The controller heartbeat went stale. Reconnect the controller and retry.",
    "TORQUE_UNCONFIRMED": "The controller could not confirm torque-off. Check servo power and retry.",
    "BUS_CORRUPT": "The servo bus returned corrupt data. Check wiring and servo power, then rescan.",
    "PROPOSAL_EXPIRED": "That movement proposal expired. Prepare the test again.",
    "BOOT_CHANGED": "The controller restarted. Rescan the bus before continuing.",
    "UNSUPPORTED": "The controller firmware does not support this command.",
    "NUDGE_TIMING_UNSAFE": (
        "The controller refused that test move: it would take longer than the "
        "350 ms motion budget. Use a higher test speed for this step size."
    ),
    "NUDGE_UNSAFE": "The controller refused that test move as out of bounds.",
    "NUDGE_INCOMPLETE": (
        "The joint did not land on target within tolerance. Torque is off and "
        "STOP is not latched - try a smaller step, or check the joint moves freely."
    ),
    "MODE_NOT_POSITION": "That servo is not in position mode. Fix its mode, then retry.",
    "NO_TORQUE_LEASE": (
        "The controller has no live torque lease for that servo, so it refused to "
        "move it. Grab the drive bar again to take one."
    ),
    "REGISTER_PROTECTED": (
        "That servo register is only writable through its dedicated command, "
        "which keeps ids unique and torque supervised."
    ),
    "UNKNOWN_KEY": "The controller does not have that configuration key.",
    "SERVO_NOT_FOUND": "That servo id is not on the bus. Rescan and try again.",
    "ODOMETER_UNAVAILABLE": (
        "Base position is no longer trusted because continuous absolute encoder "
        "feedback was lost. Keep the Base free on the physical zero mark, then press "
        "Set Base zero here again."
    ),
    "MULTI_TURN_NOT_SET": (
        "The Base servo did not confirm native absolute multi-turn mode. Keep it free, "
        "check servo power and wiring, then set Base zero again."
    ),
    "MULTI_TURN_CONFIG_FAILED": (
        "The Base servo did not verify its native absolute multi-turn configuration. "
        "No movement was sent; keep it free, check servo power and the bus, then retry "
        "Set Base zero here."
    ),
    "BUS_ERROR": (
        "The ESP32 reached the servo bus, but the addressed servo operation did not "
        "return a valid acknowledgement. Check servo power and the bus connector, then retry."
    ),
}

_MULTI_TURN_CONFIG_PHASE_MESSAGES = {
    "preflight_readback": (
        "The Base servo did not return its current lock, Phase, resolution, limits, "
        "and mode registers. No movement was sent; check servo power and the bus."
    ),
    "unlock_write": "The Base servo did not accept the configuration unlock. No movement was sent.",
    "unlock_readback": "The Base servo did not confirm the configuration unlock. No movement was sent.",
    "phase_read": "The Base servo Phase register could not be read. No movement was sent.",
    "phase_write": "The Base servo did not accept extended-position Phase BIT4. No movement was sent.",
    "phase_readback": "The Base servo did not confirm extended-position Phase BIT4. No movement was sent.",
    "resolution_readback": (
        "The Base servo angle-resolution register is not the required value 1. "
        "No movement was sent."
    ),
    "limits_write": "The Base servo did not accept the 0/0 multi-turn limits. No movement was sent.",
    "limits_readback": "The Base servo did not confirm the 0/0 multi-turn limits. No movement was sent.",
    "operating_mode_readback": "The Base servo did not confirm absolute position mode 0. No movement was sent.",
    "relock_write": "The Base servo could not be relocked after configuration. No movement was sent.",
    "relock_readback": "The Base servo did not confirm that configuration was relocked. No movement was sent.",
}

_UPSTREAM_FAILURE_MESSAGES = {
    "CONTROLLER_UNAVAILABLE": (
        "The Pi gateway is online, but the ESP32 arm controller is not connected. "
        "Check HAT logic power, the Pi UART/header connection, and that laptop USB is "
        "disconnected. No arm command was confirmed."
    ),
    "CONTROLLER_LINK_UNHEALTHY": (
        "The Pi gateway is online, but its UART exchange with the ESP32 arm controller "
        "was missing or invalid. Make sure only the Pi owns UART0, check the HAT connection, "
        "then reconnect the controller. No arm command was confirmed."
    ),
}


_FIELD_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
_BOUND_LABELS = {"ge": "at least", "gt": "greater than", "le": "at most", "lt": "less than"}


def _validation_detail(detail: object) -> str | None:
    """Name the field and the bound a FastAPI 422 actually complained about.

    Rebuilt from the structured `loc`/`ctx` fields only, never from the upstream
    `msg` string, so this stays a mapper rather than an echo. Without it every
    schema rejection read as one generic sentence, which is indistinguishable
    from a browser bug and hides the common real cause: a gateway running older
    contracts than the laptop that is talking to it.
    """

    if not isinstance(detail, list) or not detail:
        return None
    first = detail[0]
    if not isinstance(first, dict):
        return None
    location = first.get("loc")
    field = next(
        (
            part
            for part in reversed(location)
            if isinstance(part, str) and _FIELD_NAME.fullmatch(part) and part != "body"
        ),
        None,
    ) if isinstance(location, list) else None
    if field is None:
        return None
    sent = first.get("input")
    sent_text = f" (sent {sent})" if isinstance(sent, (int, float)) and not isinstance(sent, bool) else ""
    context = first.get("ctx")
    bounds = [
        f"{_BOUND_LABELS[key]} {value}"
        for key, value in (context.items() if isinstance(context, dict) else [])
        if key in _BOUND_LABELS
        and isinstance(value, (int, float))
        and not isinstance(value, bool)
    ]
    if bounds:
        return (
            f"The robot gateway refused '{field}'{sent_text}: it accepts "
            f"{' and '.join(sorted(bounds))}. If this laptop allows a wider range "
            f"than the gateway, the Pi is running older gateway code — redeploy it."
        )
    return f"The robot gateway refused the '{field}' value{sent_text}."


def _rejected_detail(body: bytes) -> str:
    """Same mapped-code passthrough as 409, so a controller veto is not reported
    to the operator as if the browser had sent a malformed request."""

    fallback = "Robot gateway rejected invalid arm parameters."
    try:
        document = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return fallback
    detail = document.get("detail") if isinstance(document, dict) else None
    code = detail.get("code") if isinstance(detail, dict) else None
    if not isinstance(code, str):
        return _validation_detail(detail) or fallback
    return _CONFLICT_MESSAGES.get(code, fallback)


def _transport_detail(error: Exception) -> str:
    """Name the layer that actually failed, not just "the gateway".

    These three read identically to an operator and have completely different
    fixes, and conflating them cost a whole debugging session chasing the
    gateway while the gateway was answering 2181 of 2187 requests with 200:

    * nothing listening on the loopback port -> the SSH tunnel is down, which is
      a laptop-side problem and the most common one by far;
    * connected but silent -> the tunnel is up and the Pi is not answering,
      so the Pi is off, wedged, or its gateway process is gone;
    * the connection dropped mid-request -> the tunnel died while in flight.
    """

    if isinstance(error, httpx.ConnectError):
        return (
            "The SSH tunnel to the Pi is down, so nothing is listening locally. "
            "Restart the tunnel (scripts/arm-tunnel.sh); the Pi itself may be fine."
        )
    if isinstance(error, httpx.TimeoutException):
        return (
            "The tunnel is open but the Pi did not answer in time. "
            "Check the Pi is powered and arm-gateway.service is running."
        )
    return (
        "The connection to the Pi dropped mid-request. "
        "Usually the SSH tunnel restarting; it should recover within a minute."
    )


def _conflict_detail(body: bytes) -> str:
    """Preserve Pi-authored operator guidance or map a controller reason code."""

    fallback = "Robot gateway state changed; refresh and try again."
    try:
        document = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return fallback
    detail = document.get("detail") if isinstance(document, dict) else None
    # The simple arm API writes these strings itself; they are the actionable
    # reason for a 409 (for example, Base homing is required). Replacing them
    # with the fallback made a healthy Pi look disconnected. Keep the boundary
    # bounded and printable before returning the text to the browser.
    if isinstance(detail, str):
        return detail if 0 < len(detail) <= 512 and detail.isprintable() else fallback
    code = detail.get("code") if isinstance(detail, dict) else None
    if not isinstance(code, str):
        return fallback
    if code == "MULTI_TURN_CONFIG_FAILED":
        phase = detail.get("phase")
        if isinstance(phase, str) and phase in _MULTI_TURN_CONFIG_PHASE_MESSAGES:
            return _MULTI_TURN_CONFIG_PHASE_MESSAGES[phase]
    return _CONFLICT_MESSAGES.get(code, fallback)


def _upstream_failure_detail(
    status_code: int, body: bytes, override: str | None = None
) -> str:
    if override is not None:
        return override
    try:
        document = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError):
        document = None
    detail = document.get("detail") if isinstance(document, dict) else None
    code = detail.get("code") if isinstance(detail, dict) else None
    if isinstance(code, str) and code in _UPSTREAM_FAILURE_MESSAGES:
        return _UPSTREAM_FAILURE_MESSAGES[code]
    if status_code == 502:
        return (
            "The Pi gateway answered, but its ESP32 controller link returned an invalid "
            "or incomplete response. Check the HAT UART connection, then reconnect the controller."
        )
    if status_code == 503:
        return (
            "The Pi gateway answered, but the ESP32 arm controller is unavailable. "
            "Check HAT power and the Pi UART/header connection."
        )
    return f"The Pi gateway answered with an internal error (HTTP {status_code})."


class RobotGatewayClient:
    def __init__(
        self,
        base_url: str,
        token_file: str | Path,
        *,
        transport: httpx.BaseTransport | None = None,
        expected_simulated: bool = False,
        camera_source: str = "picamera2",
        camera_identity_confidence: str = "configured_candidate",
    ) -> None:
        if not isinstance(expected_simulated, bool):
            raise RobotGatewayConfigurationError(
                "Robot gateway simulated identity must be a boolean."
            )
        if (
            not isinstance(camera_source, str)
            or not camera_source
            or len(camera_source) > 64
            or not camera_source.replace("_", "").replace("-", "").isalnum()
        ):
            raise RobotGatewayConfigurationError("Robot gateway camera source is invalid.")
        if (
            not isinstance(camera_identity_confidence, str)
            or not camera_identity_confidence
            or len(camera_identity_confidence) > 64
            or not camera_identity_confidence.isprintable()
        ):
            raise RobotGatewayConfigurationError(
                "Robot gateway camera identity confidence is invalid."
            )
        token = _load_token(token_file)
        self._expected_simulated = expected_simulated
        self._camera_source = camera_source
        self._camera_identity_confidence = camera_identity_confidence
        self._backend_identity: dict[str, object] | None = None
        self._client = httpx.Client(
            base_url=_validated_base_url(base_url),
            headers={"Authorization": f"Bearer {token}", "Accept": "application/json"},
            timeout=httpx.Timeout(connect=1.0, read=5.0, write=2.0, pool=1.0),
            follow_redirects=False,
            trust_env=False,
            transport=transport,
        )

    def close(self) -> None:
        self._client.close()

    def backend_identity(self, *, refresh: bool = False) -> dict[str, object] | None:
        """Return a validated upstream process identity without inventing one.

        SIM exposes the exact persistent bridge instance on ``/healthz``.  A
        deployed physical gateway may expose the same fields; older physical
        gateways fall back to the controller boot identity when available.
        ``refresh=False`` is cache-only so inventory rendering never stalls on
        an offline backend.
        """

        if not refresh:
            return dict(self._backend_identity) if self._backend_identity else None
        document = self._request("GET", "/healthz")
        backend_id = document.get("backendId")
        instance_id = document.get("backendInstanceId")
        simulated = document.get("simulated")
        if (
            backend_id in {"sim", "real"}
            and isinstance(instance_id, str)
            and re.fullmatch(r"[A-Za-z0-9_.:-]{8,256}", instance_id) is not None
            and isinstance(simulated, bool)
            and simulated is self._expected_simulated
            and (backend_id == "sim") is self._expected_simulated
        ):
            identity = {
                "backendId": backend_id,
                "backendInstanceId": instance_id,
                "simulated": simulated,
            }
            self._backend_identity = identity
            return dict(identity)
        if self._expected_simulated:
            raise RobotGatewayError(
                "The simulator gateway did not prove its bridge instance.",
                status_code=502,
            )
        # Compatibility for the currently deployed physical gateway: its
        # controller boot id is real upstream provenance, while a Pi-side plan
        # store restart still rejects any missing one-use plan itself.
        state = self.arm_state()
        controller = state.get("controller")
        boot_id = controller.get("bootId") if isinstance(controller, dict) else None
        if (
            isinstance(boot_id, str)
            and re.fullmatch(r"[A-Za-z0-9_.:-]{1,256}", boot_id) is not None
        ):
            identity = {
                "backendId": "real",
                "backendInstanceId": f"controller:{boot_id}",
                "simulated": False,
            }
            self._backend_identity = identity
            return dict(identity)
        return None

    def observe_backend_identity(self, value: object) -> dict[str, object] | None:
        """Cache provenance carried by the response being decorated.

        This keeps high-rate state polling to one upstream request. Older Pi
        gateways may omit process provenance while the controller is offline;
        those diagnostic responses remain usable with a null instance instead
        of being hidden behind a 502.
        """

        if isinstance(value, CameraFrameResponse):
            backend_id = value.headers.get("X-Arm-Backend-Id")
            instance_id = value.headers.get("X-Arm-Backend-Instance")
            simulated_text = value.headers.get("X-Simulated")
            simulated: object = (
                True if simulated_text == "true" else False if simulated_text == "false" else None
            )
        elif isinstance(value, dict):
            backend_id = value.get("backendId")
            instance_id = value.get("backendInstanceId")
            simulated = value.get("simulated")
            if not self._expected_simulated and not isinstance(instance_id, str):
                controller = value.get("controller")
                boot_id = controller.get("bootId") if isinstance(controller, dict) else None
                if (
                    isinstance(boot_id, str)
                    and re.fullmatch(r"[A-Za-z0-9_.:-]{1,256}", boot_id) is not None
                ):
                    backend_id = "real"
                    instance_id = f"controller:{boot_id}"
                    simulated = False
        else:
            return None
        if (
            backend_id in {"sim", "real"}
            and isinstance(instance_id, str)
            and re.fullmatch(r"[A-Za-z0-9_.:-]{8,256}", instance_id) is not None
            and isinstance(simulated, bool)
            and simulated is self._expected_simulated
            and (backend_id == "sim") is self._expected_simulated
        ):
            identity = {
                "backendId": backend_id,
                "backendInstanceId": instance_id,
                "simulated": simulated,
            }
            self._backend_identity = identity
            return dict(identity)
        return None

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, object] | None = None,
        *,
        not_found_detail: str | None = None,
        bad_gateway_detail: str | None = None,
        unavailable_detail: str | None = None,
        timeout: httpx.Timeout | None = None,
        retry_remote_protocol_error: bool = False,
    ) -> dict[str, object]:
        if (method, path) not in _ALLOWED_CALLS:
            raise RobotGatewayError("Robot gateway operation is not allowlisted.", status_code=500)
        content: bytes | None = None
        headers: dict[str, str] | None = None
        if payload is not None:
            try:
                content = json.dumps(
                    payload, sort_keys=True, separators=(",", ":"), allow_nan=False
                ).encode("utf-8")
            except (TypeError, ValueError):
                raise RobotGatewayError("Robot request could not be encoded safely.", status_code=422) from None
            if len(content) > MAX_REQUEST_BYTES:
                raise RobotGatewayError("Robot request is too large.", status_code=413)
            headers = {"Content-Type": "application/json"}
        try:
            with self._client.stream(
                method,
                path,
                content=content,
                headers=headers,
                timeout=timeout if timeout is not None else self._client.timeout,
            ) as response:
                body = bytearray()
                for chunk in response.iter_bytes():
                    body.extend(chunk)
                    if len(body) > MAX_RESPONSE_BYTES:
                        raise RobotGatewayError("Robot gateway response exceeded the safety limit.", status_code=502)
                if 300 <= response.status_code < 400:
                    raise RobotGatewayError("Robot gateway redirects are not accepted.", status_code=502)
                if response.status_code >= 400:
                    if response.status_code == 404 and not_found_detail is not None:
                        raise RobotGatewayError(not_found_detail, status_code=404)
                    if response.status_code == 409:
                        raise RobotGatewayError(_conflict_detail(bytes(body)), status_code=409)
                    if response.status_code == 422:
                        raise RobotGatewayError(_rejected_detail(bytes(body)), status_code=422)
                    if response.status_code >= 500:
                        raise RobotGatewayError(
                            _upstream_failure_detail(
                                response.status_code,
                                bytes(body),
                                bad_gateway_detail
                                if response.status_code == 502 and bad_gateway_detail is not None
                                else unavailable_detail,
                            ),
                            status_code=503,
                        )
                    raise RobotGatewayError("Robot gateway rejected the fixed proxy operation.", status_code=502)
        except RobotGatewayError:
            raise
        except httpx.RemoteProtocolError as error:
            if retry_remote_protocol_error:
                # Sequence execution is keyed by the exact id, digest, and
                # route mode. The Pi caches its terminal receipt, so one retry
                # after a dropped HTTP response recovers that receipt without
                # replaying motion. No other mutation opts into this path.
                return self._request(
                    method,
                    path,
                    payload,
                    not_found_detail=not_found_detail,
                    bad_gateway_detail=bad_gateway_detail,
                    unavailable_detail=unavailable_detail,
                    timeout=timeout,
                    retry_remote_protocol_error=False,
                )
            raise RobotGatewayError(
                unavailable_detail or _transport_detail(error), status_code=503
            ) from None
        except (httpx.TimeoutException, httpx.NetworkError) as error:
            raise RobotGatewayError(
                unavailable_detail or _transport_detail(error), status_code=503
            ) from None
        except httpx.HTTPError:
            raise RobotGatewayError("Robot gateway response was invalid.", status_code=502) from None
        try:
            document = json.loads(bytes(body))
        except (json.JSONDecodeError, UnicodeDecodeError):
            raise RobotGatewayError("Robot gateway returned invalid JSON.", status_code=502) from None
        if not isinstance(document, dict):
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        return document

    def camera_status(self) -> dict[str, object]:
        return _camera_status(
            self._request("GET", "/api/camera/status"),
            expected_simulated=self._expected_simulated,
        )

    def capture_camera(self, profile: Literal["survey", "detail"] = "detail") -> dict[str, object]:
        capture_profile = _camera_capture_profile(profile)
        observation = _camera_observation(
            self._request(
                "POST",
                "/api/camera/captures",
                {"captureProfile": capture_profile},
                bad_gateway_detail="Camera capture failed.",
                unavailable_detail="Camera is unavailable.",
                timeout=httpx.Timeout(
                    connect=1.0,
                    read=CAMERA_CAPTURE_READ_TIMEOUT_SECONDS,
                    write=2.0,
                    pool=1.0,
                ),
            ),
            expected_simulated=self._expected_simulated,
            expected_source=self._camera_source,
            expected_identity_confidence=self._camera_identity_confidence,
        )
        if observation.get("captureProfile") != capture_profile:
            raise RobotGatewayError(
                "Robot gateway returned a different camera capture profile.",
                status_code=502,
            )
        return observation

    def autofocus_camera(self) -> dict[str, object]:
        status = self.camera_status()
        autofocus = status.get("autofocus")
        if isinstance(autofocus, dict) and autofocus.get("capability") == "unsupported":
            return {
                "simulated": self._expected_simulated,
                "physicalArmMotion": False,
                "attempted": False,
                "result": "unsupported",
                "autofocus": autofocus,
            }
        return _camera_autofocus_attempt(
            self._request(
                "POST",
                "/api/camera/autofocus",
                bad_gateway_detail="Camera autofocus test failed.",
                unavailable_detail="Camera is unavailable.",
                timeout=httpx.Timeout(connect=1.0, read=15.0, write=2.0, pool=1.0),
            ),
            expected_simulated=self._expected_simulated,
        )

    def latest_camera_observation(self) -> dict[str, object]:
        return _camera_observation(
            self._request(
                "GET",
                "/api/camera/observations/latest",
                not_found_detail="No camera observation is available.",
            ),
            expected_simulated=self._expected_simulated,
            expected_source=self._camera_source,
            expected_identity_confidence=self._camera_identity_confidence,
        )

    def camera_frame(self, frame_id: str) -> CameraFrameResponse:
        return self._camera_jpeg(frame_id, transfer=False)

    def camera_transfer(self, transfer_token: str) -> CameraFrameResponse:
        return self._camera_jpeg(transfer_token, transfer=True)

    def _camera_jpeg(self, identifier: str, *, transfer: bool) -> CameraFrameResponse:
        if _SAFE_FRAME_ID.fullmatch(identifier) is None:
            label = "transfer token" if transfer else "frame identifier"
            raise RobotGatewayError(f"Camera {label} is invalid.", status_code=422)
        path = (
            f"/api/camera/transfers/{identifier}"
            if transfer
            else f"/api/camera/frames/{identifier}"
        )
        try:
            with self._client.stream("GET", path, headers={"Accept": "image/jpeg"}) as response:
                if 300 <= response.status_code < 400:
                    raise RobotGatewayError("Robot gateway redirects are not accepted.", status_code=502)
                if response.status_code == 404:
                    raise RobotGatewayError(
                        (
                            "Camera transfer was not found or is no longer available."
                            if transfer
                            else "Camera frame was not found in bounded history."
                        ),
                        status_code=404,
                    )
                if response.status_code >= 500:
                    raise RobotGatewayError("Robot gateway is unavailable.", status_code=503)
                if response.status_code >= 400:
                    raise RobotGatewayError(
                        "Robot gateway rejected the fixed proxy operation.", status_code=502
                    )
                media_type = response.headers.get("content-type", "").split(";", 1)[0].strip().lower()
                if media_type != "image/jpeg":
                    raise RobotGatewayError("Robot gateway returned an invalid camera frame.", status_code=502)
                declared_length = response.headers.get("content-length")
                if declared_length is not None:
                    try:
                        parsed_length = int(declared_length)
                    except ValueError:
                        raise RobotGatewayError(
                            "Robot gateway returned an invalid camera frame.", status_code=502
                        ) from None
                    if parsed_length < 0:
                        raise RobotGatewayError(
                            "Robot gateway returned an invalid camera frame.", status_code=502
                        )
                    if parsed_length > MAX_CAMERA_FRAME_BYTES:
                        raise RobotGatewayError(
                            "Robot gateway camera frame exceeded the safety limit.", status_code=502
                        )
                body = bytearray()
                for chunk in response.iter_bytes():
                    body.extend(chunk)
                    if len(body) > MAX_CAMERA_FRAME_BYTES:
                        raise RobotGatewayError(
                            "Robot gateway camera frame exceeded the safety limit.", status_code=502
                        )
                if declared_length is not None and parsed_length != len(body):
                    raise RobotGatewayError(
                        "Robot gateway returned an invalid camera frame.", status_code=502
                    )
                upstream_frame_id = response.headers.get("x-frame-id")
                content_digest = response.headers.get("x-content-sha256")
                etag = response.headers.get("etag")
                simulated = response.headers.get("x-simulated")
                read_only = response.headers.get("x-read-only")
                camera_source = response.headers.get("x-camera-source")
                one_use_transfer = response.headers.get("x-one-use-transfer")
        except RobotGatewayError:
            raise
        except (httpx.TimeoutException, httpx.NetworkError):
            raise RobotGatewayError("Robot gateway is unavailable.", status_code=503) from None
        except httpx.HTTPError:
            raise RobotGatewayError("Robot gateway response was invalid.", status_code=502) from None

        content = bytes(body)
        actual_digest = f"sha256:{hashlib.sha256(content).hexdigest()}"
        if (
            not content
            or not content.startswith(b"\xff\xd8")
            or not content.endswith(b"\xff\xd9")
            or not isinstance(upstream_frame_id, str)
            or _SAFE_FRAME_ID.fullmatch(upstream_frame_id) is None
            or (not transfer and upstream_frame_id != identifier)
            or (transfer and one_use_transfer != "true")
            or not isinstance(content_digest, str)
            or _CAMERA_CONTENT_SHA_PATTERN.fullmatch(content_digest) is None
            or not secrets.compare_digest(content_digest, actual_digest)
            or etag != f'"{content_digest}"'
            or simulated != str(self._expected_simulated).lower()
            or read_only != "true"
            or (
                self._expected_simulated
                and camera_source != self._camera_source
            )
            or (
                camera_source is not None
                and camera_source != self._camera_source
            )
        ):
            raise RobotGatewayError("Robot gateway returned an invalid camera frame.", status_code=502)
        headers = {
            "Cache-Control": "no-store",
            "ETag": f'"{content_digest}"',
            "X-Frame-Id": upstream_frame_id,
            "X-Simulated": str(self._expected_simulated).lower(),
            "X-Read-Only": "true",
            "X-Content-SHA256": content_digest,
            "X-Content-Type-Options": "nosniff",
        }
        if camera_source is not None:
            headers["X-Camera-Source"] = camera_source
        if transfer:
            headers["X-One-Use-Transfer"] = "true"
        return CameraFrameResponse(
            content=content,
            media_type="image/jpeg",
            headers=headers,
        )

    def physical_status(self) -> dict[str, object]:
        return _physical_status(
            self._request("GET", "/api/robot/physical/arm/status")
        )

    def physical_reconnect(self) -> dict[str, object]:
        return _physical_status(
            self._request(
                "POST", "/api/robot/physical/arm/controller/reconnect"
            )
        )

    def physical_scan(self, request: PhysicalScanRequest) -> dict[str, object]:
        return _physical_scan_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/bus/scan",
                request.model_dump(),
            )
        )

    def physical_assign_id(self, request: AssignServoIdRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/servos/assign-id",
            request.model_dump(),
        )

    def physical_set_position_mode(
        self, request: SetServoPositionModeRequest
    ) -> dict[str, object]:
        return _physical_position_mode_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/servos/set-position-mode",
                request.model_dump(),
            )
        )

    def physical_capture(self, request: ServoCaptureRequest) -> dict[str, object]:
        return _physical_capture_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/servos/capture",
                request.model_dump(),
            )
        )

    def physical_move(self, request: ServoMoveRequest) -> dict[str, object]:
        return self._request(
            "POST", "/api/robot/physical/arm/servos/move", request.model_dump()
        )

    def physical_read_registers(self, request: ServoRegisterReadRequest) -> dict[str, object]:
        return self._request(
            "POST", "/api/robot/physical/arm/servos/registers/read", request.model_dump()
        )

    def physical_odometer_zero(self, request: ServoOdometerRequest) -> dict[str, object]:
        return _physical_odometer_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/servos/odometer/zero",
                request.model_dump(),
            )
        )

    def physical_odometer_read(self, request: ServoOdometerRequest) -> dict[str, object]:
        return _physical_odometer_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/servos/odometer/read",
                request.model_dump(),
            )
        )

    # ---- The simple arm -----------------------------------------------------
    # Deliberately thin. The Pi already clamps every target into the joint's own
    # limits, so a second opinion here can only disagree with it — which is the
    # exact failure mode that made a legal command read as "invalid parameters".

    def arm_state(self) -> dict[str, object]:
        return self._request("GET", "/api/robot/arm/state")

    def arm_scan(self) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/scan")

    def arm_calibrate(self, joint: str, payload: dict[str, object]) -> dict[str, object]:
        return self._request("POST", f"/api/robot/arm/joints/{joint}/calibrate", payload)

    def arm_target(self, joint: str, degrees: float) -> dict[str, object]:
        return self._request("POST", f"/api/robot/arm/joints/{joint}/target", {"degrees": degrees})

    def arm_targets(self, payload: dict[str, object]) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/target", payload)

    def arm_live_follow_start(
        self, request: SimpleArmLiveFollowStartRequest
    ) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/live-follow/start",
            request.model_dump(),
            unavailable_detail=(
                "Live-follow start was not confirmed before the proxy deadline. "
                "Cancel this start attempt before recovering the controls."
            ),
            timeout=httpx.Timeout(
                connect=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
                read=LIVE_FOLLOW_ADMISSION_READ_TIMEOUT_SECONDS,
                write=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
                pool=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
            ),
        )

    def arm_live_follow_start_cancel(
        self, request: SimpleArmLiveFollowStartCancelRequest
    ) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/live-follow/start/cancel",
            request.model_dump(),
            unavailable_detail=(
                "Live-follow start cancellation is unconfirmed. Keep controls locked "
                "until the Pi input lease and watchdog have elapsed."
            ),
            timeout=httpx.Timeout(
                connect=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
                read=LIVE_FOLLOW_ADMISSION_READ_TIMEOUT_SECONDS,
                write=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
                pool=LIVE_FOLLOW_ADMISSION_STAGE_TIMEOUT_SECONDS,
            ),
        )

    def arm_live_follow_frame(
        self, request: SimpleArmLiveFollowFrameRequest
    ) -> dict[str, object]:
        return self._request(
            "POST", "/api/robot/arm/live-follow/frame", request.model_dump()
        )

    def arm_live_follow_heartbeat(
        self, request: SimpleArmLiveFollowHeartbeatRequest
    ) -> dict[str, object]:
        return self._request(
            "POST", "/api/robot/arm/live-follow/heartbeat", request.model_dump()
        )

    def arm_live_follow_end(
        self, request: SimpleArmLiveFollowEndRequest
    ) -> dict[str, object]:
        return self._request(
            "POST", "/api/robot/arm/live-follow/end", request.model_dump()
        )

    def arm_plan_preview(self, request: SimpleArmPlanPreviewRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/plans/preview",
            request.model_dump(exclude_none=True),
        )

    def arm_plan_execute(self, request: SimpleArmPlanExecuteRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/plans/execute",
            request.model_dump(),
        )

    def arm_sequence_preview(
        self, request: SimpleArmSequencePreviewRequest
    ) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/sequences/preview",
            request.model_dump(exclude_none=True),
        )

    def arm_sequence_execute(
        self, request: SimpleArmSequenceExecuteRequest
    ) -> dict[str, object]:
        read_timeout = max(25.0, request.arrivalTimeoutMs * 8 / 1000.0 + 20.0)
        return self._request(
            "POST",
            "/api/robot/arm/sequences/execute",
            request.model_dump(exclude_none=True),
            timeout=httpx.Timeout(connect=1.0, read=read_timeout, write=2.0, pool=1.0),
            retry_remote_protocol_error=True,
        )

    def arm_sequence_execute_and_capture(
        self, request: SimpleArmSequenceExecuteCaptureRequest
    ) -> dict[str, object]:
        read_timeout = max(25.0, request.arrivalTimeoutMs * 8 / 1000.0 + 20.0)
        return self._request(
            "POST",
            "/api/robot/arm/sequences/execute-and-capture",
            request.model_dump(exclude_none=True),
            timeout=httpx.Timeout(connect=1.0, read=read_timeout, write=2.0, pool=1.0),
            retry_remote_protocol_error=True,
        )

    def arm_torque(self, hold: list[int]) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/torque", {"hold": hold})

    def arm_floor_guard(self, enabled: bool) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/floor-guard", {"enabled": enabled})

    def arm_assign_id(self, old_id: int, new_id: int) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/servo-id", {"oldId": old_id, "newId": new_id})

    def arm_stop(self) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/arm/stop",
            unavailable_detail="STOP delivery is unknown. Cut servo power before touching the arm.",
        )

    def arm_clear_stop(self) -> dict[str, object]:
        return self._request("POST", "/api/robot/arm/clear-stop")

    def physical_torque_lease(self, request: TorqueLeaseRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/servos/torque-lease",
            request.model_dump(),
        )

    def physical_torque_off(self, request: TorqueOffRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/servos/torque-off",
            request.model_dump(),
            unavailable_detail=(
                "Torque-off delivery is unknown. Cut servo power before touching the arm."
            ),
        )

    def physical_hold_set(self, request: HoldSetRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/servos/hold-set",
            request.model_dump(),
        )

    def physical_prepare_nudge(self, request: PrepareNudgeRequest) -> dict[str, object]:
        prepared = self._request(
            "POST",
            "/api/robot/physical/arm/tests/prepare-nudge",
            request.model_dump(),
        )
        proposal_id = prepared.get("proposalId")
        if not isinstance(proposal_id, str) or _SAFE_FRAME_ID.fullmatch(proposal_id) is None:
            raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
        result: dict[str, object] = {
            "proposalId": proposal_id,
            "servoId": _bounded_integer(
                prepared.get("servoId", request.servoId), "servoId", minimum=0, maximum=253
            ),
            "deltaTicks": _bounded_signed_integer(
                prepared.get("deltaTicks", request.deltaTicks), minimum=-64, maximum=64
            ),
        }
        proposal_hash = prepared.get("proposalHash")
        if proposal_hash is not None:
            if not isinstance(proposal_hash, str) or re.fullmatch(
                _PROFILE_HASH_PATTERN, proposal_hash
            ) is None:
                raise RobotGatewayError("Robot gateway returned an invalid response.", status_code=502)
            result["proposalHash"] = proposal_hash
        result["expiresInMs"] = _bounded_integer(
            prepared.get("expiresInMs"),
            "expiresInMs",
            minimum=1,
            maximum=15_000,
        )
        return result

    def physical_execute_nudge(self, request: ExecuteNudgeRequest) -> dict[str, object]:
        return _physical_nudge_result(
            self._request(
                "POST",
                "/api/robot/physical/arm/tests/execute-nudge",
                request.model_dump(exclude_none=True),
                unavailable_detail=(
                    "Nudge delivery is unknown. Cut servo power before touching the arm."
                ),
            ),
            request,
        )

    def physical_stop(self) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/stop",
            unavailable_detail=(
                "STOP delivery is unknown. Cut servo power before touching the arm."
            ),
        )

    def physical_reset(self, request: PhysicalResetRequest) -> dict[str, object]:
        return self._request(
            "POST",
            "/api/robot/physical/arm/reset",
            request.model_dump(),
        )

    def physical_profile(self) -> dict[str, object]:
        return _physical_profile_response(
            self._request("GET", "/api/robot/physical/arm/calibration/profile")
        )

    def commit_physical_profile(
        self, request: PhysicalCalibrationProfileRequest
    ) -> dict[str, object]:
        response = _physical_profile_response(
            self._request(
                "POST",
                "/api/robot/physical/arm/calibration/profile",
                request.model_dump(),
            )
        )
        if response["committed"] is not True or response["motionState"] not in {
            "disarmed", "ready_disarmed"
        }:
            raise RobotGatewayError(
                "Physical calibration was not committed in a disarmed state.",
                status_code=502,
            )
        return response


__all__ = [
    "AssignServoIdRequest", "CameraFrameResponse", "ExecuteNudgeRequest",
    "MAX_CAMERA_FRAME_BYTES", "RobotGatewayClient", "RobotGatewayConfigurationError",
    "PhysicalCalibrationProfileRequest", "PhysicalResetRequest", "PhysicalScanRequest",
    "PrepareNudgeRequest",
    "RobotGatewayError", "ServoCaptureRequest", "TorqueLeaseRequest",
    "TorqueOffRequest",
]
