"""Authenticated physical-arm commissioning routes.

Authentication is supplied by :mod:`robot_gateway.runtime`.  This router keeps
the browser-facing surface deliberately smaller than the controller protocol:
there is no raw register, bus packet, absolute goal, or arbitrary trajectory
endpoint.
"""

from __future__ import annotations

from copy import deepcopy
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from functools import wraps
from hashlib import sha256
import inspect
import json
import math
import os
from pathlib import Path
import secrets
import tempfile
import threading
import time
from typing import Annotated, Literal

from fastapi import APIRouter, HTTPException, Request, status
from pydantic import Field, field_validator, model_validator

from .request_validation import strict_body as _strict_body
from .arm_controller import (
    ArmController,
    ControllerCommandError,
    ControllerProtocolError,
    ControllerTransportError,
    ControllerUnavailableError,
)
from .strict_contract import StrictContract
from .serial_arm_controller import HOLDING_MODES, MOTION_POLICY


ServoId = Annotated[int, Field(strict=True, ge=0, le=253)]
# A wrap-counted joint may travel several motor turns. 8 turns covers the Base
# ring gear (~8 motor turns per joint turn) with margin, and stays far below the
# firmware odometer's own 64-revolution fault ceiling.
MAX_MULTI_TURN_TICKS = 8 * 4096
ProposalId = Annotated[
    str,
    Field(strict=True, min_length=8, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
]
ProposalHash = Annotated[
    str, Field(strict=True, pattern=r"^sha256:[0-9a-f]{64}$")
]
EvidenceId = Annotated[
    str,
    Field(strict=True, min_length=8, max_length=128, pattern=r"^[A-Za-z0-9_-]+$"),
]


class ScanRequest(StrictContract):
    minId: ServoId
    maxId: ServoId

    @model_validator(mode="after")
    def ordered_range(self) -> "ScanRequest":
        if self.minId > self.maxId:
            raise ValueError("minId must not exceed maxId")
        return self


class AssignIdRequest(StrictContract):
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
    def changed_id(self) -> "AssignIdRequest":
        if self.oldId == self.newId:
            raise ValueError("newId must differ from oldId")
        return self


class SetPositionModeRequest(StrictContract):
    servoId: ServoId
    acknowledgedSingleServo: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedSingleServo", mode="before")
    @classmethod
    def exact_single_servo_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedSingleServo must be the boolean true")
        return True


class CaptureRequest(StrictContract):
    servoId: ServoId


class OdometerRequest(StrictContract):
    servoId: ServoId


class RegisterReadRequest(StrictContract):
    servoId: ServoId
    address: Annotated[int, Field(strict=True, ge=0, le=255)]
    length: Annotated[int, Field(strict=True, ge=1, le=16)]


class MoveRequest(StrictContract):
    servoId: ServoId
    goal: Annotated[int, Field(strict=True, ge=0, le=4095)]
    speed: Annotated[int, Field(strict=True, ge=1, le=4095)]
    acceleration: Annotated[int, Field(strict=True, ge=1, le=50)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]


class TorqueLeaseRequest(StrictContract):
    servoId: ServoId
    leaseMs: Annotated[int, Field(strict=True, ge=100, le=2_000)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True


class TorqueOffRequest(StrictContract):
    servoId: ServoId | None


class HoldSetRequest(StrictContract):
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
    def unique_ids(self) -> "HoldSetRequest":
        if len(set(self.servoIds)) != len(self.servoIds):
            raise ValueError("servoIds must be unique")
        return self


class PrepareNudgeRequest(StrictContract):
    servoId: ServoId
    # Bounded to match MOTION_POLICY rather than the old firmware constants: a
    # geared joint needs hundreds of motor ticks to move a visible amount at the
    # joint, and a 64-tick ceiling made the direction test invisible on the base.
    deltaTicks: Annotated[int, Field(strict=True, ge=-512, le=512)]
    speed: Annotated[int, Field(strict=True, ge=1, le=2_400)]
    acceleration: Annotated[int, Field(strict=True, ge=1, le=50)]
    acknowledgedPhysicalPowerCut: Literal[True]
    confirmedServoModel: Literal["ST3215"]
    zeroEvidenceId: EvidenceId

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


class ExecuteNudgeRequest(StrictContract):
    proposalId: ProposalId
    proposalHash: ProposalHash | None = None
    acknowledgedPhysicalPowerCut: Literal[True]

    @field_validator("acknowledgedPhysicalPowerCut", mode="before")
    @classmethod
    def exact_power_cut_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedPhysicalPowerCut must be the boolean true")
        return True


class ResetRequest(StrictContract):
    acknowledgedPhysicalInspection: Literal[True]


class PhysicalJointProfile(StrictContract):
    logicalId: Literal["joint_1", "joint_2", "joint_3"]
    name: Annotated[str, Field(strict=True, min_length=1, max_length=48)]
    servoId: ServoId
    rawZero: Annotated[int, Field(strict=True, ge=0, le=4095)]
    direction: Literal[-1, 1]
    motorTurnsPerJointTurn: float = 1.0
    # multiTurn joints are wrap-counted by the HAT odometer, so their working
    # range may span several motor turns. Driving past one turn is NOT supported
    # yet: prepare/execute nudge still validate against raw 0..4095.
    multiTurn: bool = False
    # Limits the operator typed rather than physically drove the joint to. Zero
    # and the direction nudge still need real controller evidence; only the two
    # endpoint captures are waived, and the profile records that it happened.
    declaredLimits: bool = False
    negativeLimitTicks: Annotated[
        int, Field(strict=True, ge=-MAX_MULTI_TURN_TICKS, le=-1)
    ]
    positiveLimitTicks: Annotated[
        int, Field(strict=True, ge=1, le=MAX_MULTI_TURN_TICKS)
    ]
    limitMarginTicks: Annotated[int, Field(strict=True, ge=1, le=512)]
    maxVelocityDegreesPerSecond: float
    maxAccelerationDegreesPerSecond2: float
    evidenceIds: Annotated[
        list[
            Annotated[
                str,
                Field(
                    strict=True,
                    min_length=8,
                    max_length=128,
                    pattern=r"^[A-Za-z0-9_-]+$",
                ),
            ]
        ],
        Field(min_length=1, max_length=16),
    ]

    @field_validator(
        "motorTurnsPerJointTurn",
        "maxVelocityDegreesPerSecond",
        "maxAccelerationDegreesPerSecond2",
        mode="before",
    )
    @classmethod
    def finite_positive_limits(cls, value: object, info: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{getattr(info, 'field_name', 'value')} must be a number")
        number = float(value)
        if not math.isfinite(number) or number <= 0:
            raise ValueError(f"{getattr(info, 'field_name', 'value')} must be finite and positive")
        field_name = getattr(info, "field_name", "")
        if field_name == "motorTurnsPerJointTurn":
            maximum = 64.0
        else:
            maximum = 90.0 if field_name == "maxVelocityDegreesPerSecond" else 180.0
        if number > maximum:
            raise ValueError(f"{getattr(info, 'field_name', 'value')} exceeds the commissioning limit")
        return number

    @model_validator(mode="after")
    def safe_offsets(self) -> "PhysicalJointProfile":
        if self.negativeLimitTicks >= -self.limitMarginTicks:
            raise ValueError("negativeLimitTicks must extend beyond the safety margin")
        if self.positiveLimitTicks <= self.limitMarginTicks:
            raise ValueError("positiveLimitTicks must extend beyond the safety margin")
        if self.multiTurn:
            # Wrap counting makes the recorded range honest, but the servo is
            # still commanded inside one turn, so a multi-turn joint must say so
            # rather than silently claiming a drivable range.
            if not 1 <= self.motorTurnsPerJointTurn <= 64:
                raise ValueError("a multi-turn joint needs a gear ratio of at least 1")
        else:
            negative_raw = self.rawZero + self.direction * self.negativeLimitTicks
            positive_raw = self.rawZero + self.direction * self.positiveLimitTicks
            if not (0 <= negative_raw <= 4095 and 0 <= positive_raw <= 4095):
                raise ValueError(
                    "The ST3215 position-mode working range must not cross raw 0 or 4095"
                )
        if len(set(self.evidenceIds)) != len(self.evidenceIds):
            raise ValueError("evidenceIds must be unique")
        if not self.name.isprintable() or not self.name.strip():
            raise ValueError("name must be printable")
        return self


class PhysicalProfileJoints(StrictContract):
    joint_1: PhysicalJointProfile
    joint_2: PhysicalJointProfile
    joint_3: PhysicalJointProfile

    @model_validator(mode="after")
    def identities_match_slots(self) -> "PhysicalProfileJoints":
        for logical_id in ("joint_1", "joint_2", "joint_3"):
            if getattr(self, logical_id).logicalId != logical_id:
                raise ValueError(f"{logical_id}.logicalId must match its joint slot")
        identifiers = [self.joint_1.servoId, self.joint_2.servoId, self.joint_3.servoId]
        if len(set(identifiers)) != 3:
            raise ValueError("each physical joint must use a unique servoId")
        return self


class PhysicalArmGeometry(StrictContract):
    baseHeightMm: Annotated[float, Field(ge=0, le=1000)]
    upperArmMm: Annotated[float, Field(gt=0, le=1000)]
    forearmMm: Annotated[float, Field(gt=0, le=1000)]
    toolOffsetMm: Annotated[float, Field(ge=0, le=500)]

    @field_validator("baseHeightMm", "upperArmMm", "forearmMm", "toolOffsetMm", mode="before")
    @classmethod
    def finite_geometry(cls, value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("geometry values must be JSON numbers")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("geometry values must be finite")
        return number


class SavePhysicalProfileRequest(StrictContract):
    expectedControllerBootId: Annotated[
        str,
        Field(strict=True, min_length=1, max_length=128, pattern=r"^[A-Za-z0-9._-]+$"),
    ]
    joints: PhysicalProfileJoints
    geometry: PhysicalArmGeometry
    acknowledgedRemainDisarmed: Literal[True]
    confirmedServoModel: Literal["ST3215"]

    @field_validator("acknowledgedRemainDisarmed", mode="before")
    @classmethod
    def exact_disarmed_acknowledgement(cls, value: object) -> bool:
        if value is not True:
            raise ValueError("acknowledgedRemainDisarmed must be the boolean true")
        return True


class PhysicalProfileConflict(RuntimeError):
    """Live controller evidence does not match the proposed profile."""


class PhysicalProfileStoreError(RuntimeError):
    """The private profile state could not be loaded or atomically stored."""


class CommissioningEvidenceLedger:
    """Process-local, controller-boot-bound evidence for profile commits.

    IDs supplied by the browser never create evidence.  A record exists only
    after this process verified a controller capture or completed nudge.  The
    ledger intentionally disappears on gateway restart and clears when a new
    controller boot is observed.
    """

    MAX_RECORDS = 256
    MAX_FRESH_PACKET_AGE_MS = 250

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._boot_id: str | None = None
        self._records: dict[str, dict[str, object]] = {}

    def _record(
        self, boot_id: str, kind: Literal["capture", "nudge"], payload: dict[str, object]
    ) -> str:
        if not isinstance(boot_id, str) or not boot_id:
            raise ControllerProtocolError("controller boot identity is unavailable")
        with self._lock:
            if self._boot_id is not None and not secrets.compare_digest(
                self._boot_id, boot_id
            ):
                self._records.clear()
            self._boot_id = boot_id
            prefix = "cap" if kind == "capture" else "nudge"
            evidence_id = f"{prefix}_{secrets.token_urlsafe(18)}"
            self._records[evidence_id] = {
                "evidenceId": evidence_id,
                "kind": kind,
                "bootId": boot_id,
                "observedAtUnixMs": int(time.time() * 1000),
                **deepcopy(payload),
            }
            while len(self._records) > self.MAX_RECORDS:
                oldest = next(iter(self._records))
                self._records.pop(oldest, None)
            return evidence_id

    def record_capture(
        self, boot_id: str, servo_id: int, observation: dict[str, object]
    ) -> str:
        raw = observation.get("rawPosition")
        packet_age = observation.get("packetAgeMs")
        sample_count = observation.get("sampleCount")
        errors = observation.get("errors")
        operating_mode = observation.get("operatingMode")
        if (
            isinstance(raw, bool)
            or not isinstance(raw, int)
            or not 0 <= raw <= 4095
            or isinstance(packet_age, bool)
            or not isinstance(packet_age, int)
            or not 0 <= packet_age <= self.MAX_FRESH_PACKET_AGE_MS
            or isinstance(sample_count, bool)
            or not isinstance(sample_count, int)
            or not 5 <= sample_count <= 100
            or observation.get("torqueState") != "off"
            or errors != []
            or operating_mode != 0
        ):
            raise ControllerProtocolError(
                "capture lacks fresh position-mode torque-off evidence"
            )
        # Wrap-counted position, when the HAT odometer was armed for this servo.
        # Absent or invalid means the capture is single-turn evidence only.
        odometer_valid = observation.get("odometerValid") is True
        multi_turn = observation.get("multiTurnPosition")
        if odometer_valid and (
            isinstance(multi_turn, bool)
            or not isinstance(multi_turn, int)
            or abs(multi_turn) > MAX_MULTI_TURN_TICKS + 4096
            or multi_turn % 4096 != raw % 4096
        ):
            raise ControllerProtocolError("capture multi-turn position is inconsistent")
        return self._record(
            boot_id,
            "capture",
            {
                "servoId": servo_id,
                "rawPosition": raw,
                "multiTurnPosition": multi_turn if odometer_valid else None,
                "torqueState": "off",
                "packetAgeMs": packet_age,
                "sampleCount": sample_count,
                "operatingMode": 0,
            },
        )

    def record_nudge(
        self, boot_id: str, completion: dict[str, object]
    ) -> str:
        controller_evidence_id = completion.get("evidenceId")
        if (
            not isinstance(controller_evidence_id, str)
            or not 8 <= len(controller_evidence_id) <= 128
            or not controller_evidence_id.replace("_", "").replace("-", "").isalnum()
        ):
            raise ControllerProtocolError("nudge controller evidence is incomplete")
        required_ints = (
            "servoId",
            "startRawPosition",
            "targetRawPosition",
            "rawPosition",
            "measuredDeltaTicks",
            "positionErrorTicks",
        )
        values: dict[str, int] = {}
        for key in required_ints:
            value = completion.get(key)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ControllerProtocolError("nudge completion evidence is incomplete")
            values[key] = value
        commanded = completion.get("commandedDeltaTicks")
        if isinstance(commanded, bool) or not isinstance(commanded, int):
            raise ControllerProtocolError("nudge command evidence is incomplete")
        if (
            completion.get("completed") is not True
            or completion.get("torqueState") != "off"
            or not 0 <= values["servoId"] <= 253
            or any(
                not 0 <= values[key] <= 4095
                for key in (
                    "startRawPosition",
                    "targetRawPosition",
                    "rawPosition",
                )
            )
            or commanded == 0
            or not -64 <= commanded <= 64
            or values["targetRawPosition"] - values["startRawPosition"] != commanded
            or values["measuredDeltaTicks"]
            != values["rawPosition"] - values["startRawPosition"]
            or values["positionErrorTicks"]
            != values["rawPosition"] - values["targetRawPosition"]
            or (values["measuredDeltaTicks"] > 0) != (commanded > 0)
            or abs(values["measuredDeltaTicks"] - commanded) > 8
            or abs(values["positionErrorTicks"]) > 8
        ):
            raise ControllerProtocolError("nudge completion evidence is not safe")
        return self._record(
            boot_id,
            "nudge",
            {
                **values,
                "deltaTicks": commanded,
                "torqueState": "off",
                "completed": True,
                "controllerEvidenceId": controller_evidence_id,
            },
        )

    def verify_joint(
        self, boot_id: str, joint: PhysicalJointProfile
    ) -> None:
        with self._lock:
            records: list[dict[str, object]] = []
            for evidence_id in joint.evidenceIds:
                record = self._records.get(evidence_id)
                if (
                    record is None
                    or record.get("bootId") != boot_id
                    or record.get("servoId") != joint.servoId
                ):
                    raise PhysicalProfileConflict(
                        f"{joint.logicalId} contains forged, stale, or other-servo evidence."
                    )
                records.append(record)

        captures = [record for record in records if record.get("kind") == "capture"]
        nudges = [record for record in records if record.get("kind") == "nudge"]
        if not any(record.get("rawPosition") == joint.rawZero for record in captures):
            raise PhysicalProfileConflict(
                f"{joint.logicalId} rawZero does not match a live capture."
            )

        # A multi-turn joint travels past the 4095 wrap, so its endpoint evidence
        # is only meaningful in wrap-counted coordinates.
        zero_multi_turn: int | None = None
        if joint.multiTurn:
            for record in captures:
                if (
                    record.get("rawPosition") == joint.rawZero
                    and isinstance(record.get("multiTurnPosition"), int)
                ):
                    zero_multi_turn = int(record["multiTurnPosition"])
                    break
            if zero_multi_turn is None:
                raise PhysicalProfileConflict(
                    f"{joint.logicalId} is multi-turn but its zero lacks wrap-counted evidence."
                )

        def logical_capture(record: dict[str, object]) -> int | None:
            if joint.multiTurn:
                position = record.get("multiTurnPosition")
                if not isinstance(position, int):
                    return None
                return (position - (zero_multi_turn or 0)) * joint.direction
            return (int(record["rawPosition"]) - joint.rawZero) * joint.direction

        observed = [
            value for value in (logical_capture(record) for record in captures)
            if value is not None
        ]
        negative_observed = joint.negativeLimitTicks - joint.limitMarginTicks
        positive_observed = joint.positiveLimitTicks + joint.limitMarginTicks
        if not joint.declaredLimits:
            if not any(value <= negative_observed for value in observed):
                raise PhysicalProfileConflict(
                    f"{joint.logicalId} lacks negative endpoint evidence beyond its margin."
                )
            if not any(value >= positive_observed for value in observed):
                raise PhysicalProfileConflict(
                    f"{joint.logicalId} lacks positive endpoint evidence beyond its margin."
                )

        negative_raw = joint.rawZero + joint.direction * joint.negativeLimitTicks
        positive_raw = joint.rawZero + joint.direction * joint.positiveLimitTicks
        working_min = min(negative_raw, positive_raw)
        working_max = max(negative_raw, positive_raw)
        if joint.multiTurn:
            # The recorded range spans several turns but the servo is still only
            # commandable inside one. Verify the nudge against what can actually
            # be driven rather than against the wider recorded travel.
            working_min = max(working_min, 0)
            working_max = min(working_max, 4095)
        nudge_verified = False
        for record in nudges:
            commanded = int(record["deltaTicks"])
            measured = int(record["measuredDeltaTicks"])
            start = int(record["startRawPosition"])
            target = int(record["targetRawPosition"])
            if (
                commanded * joint.direction > 0
                and measured * joint.direction > 0
                # Must not be tighter than the controller's own settling
                # tolerance, or a nudge the controller reported as complete gets
                # rejected here at save time with an unrelated-sounding reason.
                and abs(measured - commanded) <= MOTION_POLICY["POS_TOLERANCE"]
                and working_min <= start <= working_max
                and working_min <= target <= working_max
            ):
                nudge_verified = True
                break
        if not nudge_verified:
            raise PhysicalProfileConflict(
                f"{joint.logicalId} lacks a correct in-range direction nudge."
            )

    def verified_capture_position(
        self, evidence_id: str, boot_id: str, servo_id: int
    ) -> int:
        with self._lock:
            record = self._records.get(evidence_id)
            if (
                record is None
                or record.get("kind") != "capture"
                or record.get("bootId") != boot_id
                or record.get("servoId") != servo_id
                or record.get("torqueState") != "off"
            ):
                raise PhysicalProfileConflict(
                    "The selected zero capture is forged, stale, or belongs to another servo."
                )
            raw_position = record.get("rawPosition")
            if isinstance(raw_position, bool) or not isinstance(raw_position, int):
                raise PhysicalProfileConflict("The selected zero capture is invalid.")
            return raw_position


def _profile_digest(profile: dict[str, object]) -> str:
    canonical = json.dumps(
        profile, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return f"sha256:{sha256(canonical).hexdigest()}"


def _reject_profile_constant(_: str) -> None:
    raise ValueError("non-finite JSON value")


def _profile_object_without_duplicates(
    pairs: list[tuple[str, object]],
) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _validate_stored_profile(profile: dict[str, object]) -> None:
    if set(profile) != {
        "schemaVersion",
        "servoModel",
        "servoLabelsConfirmed",
        "controller",
        "expectedServoIds",
        "joints",
        "geometry",
    } or profile.get("schemaVersion") != 1:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.")
    if profile.get("servoModel") != "ST3215" or profile.get("servoLabelsConfirmed") is not True:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.")
    controller = profile.get("controller")
    if not isinstance(controller, dict) or set(controller) != {
        "controllerId",
        "firmwareVersion",
        "protocolVersion",
    }:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.")
    for key in ("controllerId", "firmwareVersion"):
        value = controller.get(key)
        if not isinstance(value, str) or not value or len(value) > 128 or not value.isprintable():
            raise PhysicalProfileStoreError("Stored physical profile is invalid.")
    protocol = controller.get("protocolVersion")
    if isinstance(protocol, bool) or not isinstance(protocol, int) or protocol != 1:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.")
    try:
        joints = PhysicalProfileJoints.model_validate(profile.get("joints"))
        PhysicalArmGeometry.model_validate(profile.get("geometry"))
    except Exception:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.") from None
    expected = profile.get("expectedServoIds")
    actual = sorted((joints.joint_1.servoId, joints.joint_2.servoId, joints.joint_3.servoId))
    if expected != actual:
        raise PhysicalProfileStoreError("Stored physical profile is invalid.")


class PhysicalArmProfileStore:
    """Small revisioned profile store with optional atomic disk persistence."""

    _FILENAME = "physical-arm-profile.json"

    def __init__(self, state_dir: str | os.PathLike[str] | None = None) -> None:
        self._lock = threading.RLock()
        self._state_dir = Path(state_dir) if state_dir is not None else None
        self._path = self._state_dir / self._FILENAME if self._state_dir is not None else None
        self._revision = 0
        self._hash: str | None = None
        self._profile: dict[str, object] | None = None
        if self._state_dir is not None:
            try:
                self._state_dir.mkdir(parents=True, mode=0o700, exist_ok=True)
                if self._path is not None and self._path.is_symlink():
                    raise PhysicalProfileStoreError("Profile state path must not be a symbolic link.")
                if self._path is not None and self._path.exists():
                    self._load_locked()
            except PhysicalProfileStoreError:
                raise
            except OSError:
                raise PhysicalProfileStoreError("Unable to initialize physical profile state.") from None

    def snapshot(self) -> tuple[int, str | None, dict[str, object] | None]:
        with self._lock:
            return self._revision, self._hash, deepcopy(self._profile)

    def commit(self, profile: dict[str, object]) -> tuple[int, str, dict[str, object]]:
        digest = _profile_digest(profile)
        with self._lock:
            if self._revision >= 2_147_483_647:
                raise PhysicalProfileStoreError("Physical profile revision limit reached.")
            revision = self._revision + 1
            envelope: dict[str, object] = {
                "profileRevision": revision,
                "profileHash": digest,
                "profile": deepcopy(profile),
            }
            if self._path is not None and self._state_dir is not None:
                self._write_atomic(envelope)
            self._revision = revision
            self._hash = digest
            self._profile = deepcopy(profile)
            return revision, digest, deepcopy(profile)

    def _load_locked(self) -> None:
        assert self._path is not None
        try:
            document = json.loads(
                self._path.read_text(encoding="utf-8"),
                parse_constant=_reject_profile_constant,
                object_pairs_hook=_profile_object_without_duplicates,
            )
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
            raise PhysicalProfileStoreError("Stored physical profile is invalid.") from None
        if not isinstance(document, dict):
            raise PhysicalProfileStoreError("Stored physical profile is invalid.")
        revision = document.get("profileRevision")
        digest = document.get("profileHash")
        profile = document.get("profile")
        if (
            isinstance(revision, bool)
            or not isinstance(revision, int)
            or not 1 <= revision <= 2_147_483_647
            or not isinstance(digest, str)
            or not isinstance(profile, dict)
            or not secrets.compare_digest(digest, _profile_digest(profile))
        ):
            raise PhysicalProfileStoreError("Stored physical profile is invalid.")
        _validate_stored_profile(profile)
        self._revision = revision
        self._hash = digest
        self._profile = deepcopy(profile)

    def _write_atomic(self, envelope: dict[str, object]) -> None:
        assert self._path is not None and self._state_dir is not None
        encoded = (
            json.dumps(envelope, sort_keys=True, separators=(",", ":"), allow_nan=False)
            + "\n"
        ).encode("utf-8")
        descriptor = -1
        temporary_path: str | None = None
        try:
            descriptor, temporary_path = tempfile.mkstemp(
                prefix=".physical-arm-profile-", dir=self._state_dir
            )
            if os.name == "posix" and hasattr(os, "fchmod"):
                os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "wb") as handle:
                descriptor = -1
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self._path)
            temporary_path = None
            if os.name == "posix":
                os.chmod(self._path, 0o600)
        except OSError:
            raise PhysicalProfileStoreError("Unable to store the physical profile atomically.") from None
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            if temporary_path is not None:
                try:
                    os.unlink(temporary_path)
                except OSError:
                    pass


class PhysicalArmProfileService:
    """Commits profiles only from a fresh, fully disarmed controller census."""

    MAX_FRESH_PACKET_AGE_MS = 250

    def __init__(
        self,
        controller: ArmController,
        store: PhysicalArmProfileStore,
        evidence: CommissioningEvidenceLedger,
    ) -> None:
        self._controller = controller
        self._store = store
        self._evidence = evidence
        self._lock = threading.RLock()

    def status(self) -> dict[str, object]:
        revision, digest, profile = self._store.snapshot()
        transport = self._controller.transport_state()
        disarmed = (
            transport.get("connection") == "online"
            and transport.get("torqueState") == "off"
        )
        ready = (
            disarmed
            and transport.get("bus") == "online"
            and transport.get("servos") == "online"
        )
        return {
            "mode": "physical",
            "committed": False,
            "motionState": "ready_disarmed" if ready else ("disarmed" if disarmed else "blocked"),
            "profileRevision": revision,
            "profileHash": digest,
            "profile": profile,
        }

    def commit(self, request: SavePhysicalProfileRequest) -> dict[str, object]:
        with self._lock:
            before = self._verified_disarmed_status(request.expectedControllerBootId)
            identity = before["identity"]
            assert isinstance(identity, dict)
            joint_models = {
                logical_id: getattr(request.joints, logical_id)
                for logical_id in ("joint_1", "joint_2", "joint_3")
            }
            expected_ids = sorted(joint.servoId for joint in joint_models.values())
            boot_id = str(identity["bootId"])
            for joint in joint_models.values():
                self._evidence.verify_joint(boot_id, joint)
            scan = self._controller.scan(0, 253)
            found = scan.get("foundIds")
            if not isinstance(found, list) or sorted(found) != expected_ids:
                raise PhysicalProfileConflict(
                    "The responding servo IDs do not exactly match the proposed joint map."
                )

            captured: dict[int, dict[str, object]] = {}
            fresh_evidence_ids: dict[int, str] = {}
            for servo_id in expected_ids:
                observation = self._controller.capture(servo_id)
                self._validate_fresh_off_observation(servo_id, observation)
                captured[servo_id] = observation
                fresh_evidence_ids[servo_id] = self._evidence.record_capture(
                    boot_id, servo_id, observation
                )

            after = self._verified_disarmed_status(request.expectedControllerBootId)
            after_identity = after.get("identity")
            if not isinstance(after_identity, dict) or after_identity != identity:
                raise PhysicalProfileConflict("Controller identity changed during profile verification.")

            joint_payload: dict[str, object] = {}
            for logical_id, joint in joint_models.items():
                data = joint.model_dump()
                evidence = list(data["evidenceIds"])
                fresh_id = fresh_evidence_ids[joint.servoId]
                if fresh_id not in evidence:
                    evidence = evidence[-15:]
                    evidence.append(fresh_id)
                data["evidenceIds"] = evidence
                joint_payload[logical_id] = data
            profile: dict[str, object] = {
                "schemaVersion": 1,
                "servoModel": request.confirmedServoModel,
                "servoLabelsConfirmed": True,
                "controller": {
                    "controllerId": identity["controllerId"],
                    "firmwareVersion": identity["firmwareVersion"],
                    "protocolVersion": identity["protocolVersion"],
                },
                "expectedServoIds": expected_ids,
                "joints": joint_payload,
                "geometry": request.geometry.model_dump(),
            }
            revision, digest, stored = self._store.commit(profile)
            return {
                "mode": "physical",
                "committed": True,
                "motionState": "ready_disarmed",
                "profileRevision": revision,
                "profileHash": digest,
                "profile": stored,
            }

    def _verified_disarmed_status(self, expected_boot_id: str) -> dict[str, object]:
        state = self._controller.status()
        if state.get("connection") != "online" or state.get("torqueState") != "off":
            raise PhysicalProfileConflict("All controller and servo torque must be confirmed off.")
        identity = state.get("identity")
        if not isinstance(identity, dict):
            raise PhysicalProfileConflict("Controller identity is unavailable.")
        required = ("controllerId", "bootId", "firmwareVersion", "protocolVersion")
        if any(key not in identity for key in required):
            raise PhysicalProfileConflict("Controller identity is incomplete.")
        boot_id = identity.get("bootId")
        if not isinstance(boot_id, str) or not secrets.compare_digest(boot_id, expected_boot_id):
            raise PhysicalProfileConflict("Controller boot changed; capture fresh evidence.")
        return state

    def _validate_fresh_off_observation(
        self, servo_id: int, observation: dict[str, object]
    ) -> None:
        observed_id = observation.get("id", observation.get("servoId"))
        packet_age = observation.get("packetAgeMs")
        samples = observation.get("sampleCount")
        errors = observation.get("errors")
        evidence_id = observation.get("evidenceId")
        if (
            observed_id != servo_id
            or isinstance(packet_age, bool)
            or not isinstance(packet_age, int)
            or not 0 <= packet_age <= self.MAX_FRESH_PACKET_AGE_MS
            or isinstance(samples, bool)
            or not isinstance(samples, int)
            or not 5 <= samples <= 100
            or observation.get("torqueState") != "off"
            or errors != []
            or observation.get("operatingMode") != 0
            or not isinstance(evidence_id, str)
            or not 8 <= len(evidence_id) <= 128
        ):
            raise PhysicalProfileConflict(
                f"Servo {servo_id} lacks fresh, fault-free, torque-off telemetry."
            )


def _controller_offline() -> HTTPException:
    return HTTPException(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        detail="The HAT controller is not responding. Motion is blocked.",
    )


def _translate(error: Exception) -> HTTPException:
    if isinstance(error, (ControllerUnavailableError, ControllerTransportError)):
        return _controller_offline()
    if isinstance(error, ControllerProtocolError):
        return HTTPException(
            status_code=status.HTTP_502_BAD_GATEWAY,
            detail="The HAT controller response could not be verified. Motion is blocked.",
        )
    if isinstance(error, ControllerCommandError):
        conflicts = {
            "ID_IN_USE",
            "NOT_SINGLE_SERVO",
            "MODE_VERIFY_FAILED",
            "UNSUPPORTED",
            "STOPPED",
            "PROPOSAL_USED_OR_UNKNOWN",
            "PROPOSAL_HASH_MISMATCH",
            "PROPOSAL_EXPIRED",
            "BOOT_CHANGED",
        }
        code = status.HTTP_409_CONFLICT if error.code in conflicts else status.HTTP_422_UNPROCESSABLE_ENTITY
        return HTTPException(
            status_code=code,
            detail={"code": error.code, "message": "The controller rejected the bounded command."},
        )
    return HTTPException(
        status_code=status.HTTP_502_BAD_GATEWAY,
        detail="The HAT controller request failed. Motion is blocked.",
    )


def _state(controller: ArmController) -> dict[str, object]:
    try:
        reported = controller.status()
    except Exception:
        reported = controller.transport_state()
    return reported


def _current_boot_id(controller: ArmController) -> str:
    reported = controller.transport_state()
    identity = reported.get("identity")
    boot_id = identity.get("bootId") if isinstance(identity, dict) else None
    if reported.get("connection") != "online" or not isinstance(boot_id, str) or not boot_id:
        raise ControllerUnavailableError("controller boot identity is unavailable")
    return boot_id


def _public_status(
    controller: ArmController, profile_store: PhysicalArmProfileStore
) -> dict[str, object]:
    reported = _state(controller)
    connection = str(reported.get("connection", "faulted"))
    internal_bus = str(reported.get("bus", "unknown"))
    internal_servos = str(reported.get("servos", "unknown"))
    torque = reported.get("torqueState")
    if torque not in {"off", "on", "unknown"}:
        torque = "unknown"
    identity = reported.get("identity")
    identity = identity if isinstance(identity, dict) else {}
    telemetry_raw = reported.get("servosTelemetry")
    telemetry = deepcopy(telemetry_raw) if isinstance(telemetry_raw, list) else []
    responding_ids = sorted(
        item["id"]
        for item in telemetry
        if isinstance(item, dict)
        and isinstance(item.get("id"), int)
        and not isinstance(item.get("id"), bool)
        and 0 <= int(item["id"]) <= 253
    )
    last_scan = reported.get("lastScan")
    _, _, profile = profile_store.snapshot()
    expected_ids_raw = profile.get("expectedServoIds") if isinstance(profile, dict) else []
    expected_ids = (
        sorted(identifier for identifier in expected_ids_raw if isinstance(identifier, int))
        if isinstance(expected_ids_raw, list)
        else []
    )

    bus_state = {
        "online": "online",
        "offline": "offline",
        "faulted": "degraded",
        "unknown": "unknown",
    }.get(internal_bus, "unknown")
    if internal_servos == "none_found":
        servo_state = "none"
    elif internal_servos == "faulted":
        # A generic controller fault does not prove an ID collision.
        servo_state = "unknown"
    elif internal_servos == "online":
        if expected_ids and set(responding_ids) != set(expected_ids):
            servo_state = "partial"
        else:
            servo_state = "online"
    else:
        servo_state = "unknown"

    internal_motion = str(reported.get("motionState", "blocked"))
    if connection != "online":
        motion_state = "faulted" if connection == "faulted" else "blocked"
    elif internal_motion == "stopped":
        motion_state = "stopped"
    elif torque == "unknown":
        motion_state = "blocked"
    elif torque == "on":
        motion_state = "commissioning" if internal_motion == "moving" else "holding"
    elif internal_motion == "ready":
        motion_state = "ready_disarmed"
    else:
        motion_state = "disarmed"

    blockers: list[dict[str, str]] = []
    if connection == "not_configured":
        next_action = "Configure the HAT controller serial port, then restart the gateway."
        blockers.append(
            {
                "code": "HAT_NOT_CONFIGURED",
                "message": "No HAT controller serial port is configured.",
                "recovery": next_action,
            }
        )
    elif connection != "online":
        next_action = "Check controller power, UART selection, wiring, and the configured serial port."
        blockers.append(
            {
                "code": "HAT_OFFLINE" if connection == "offline" else "HAT_FAULTED",
                "message": "The HAT controller is offline." if connection == "offline" else "The HAT controller transport is faulted.",
                "recovery": next_action,
            }
        )
    elif bus_state != "online":
        next_action = "Verify the controller firmware identity and servo-bus power before scanning."
        blockers.append(
            {
                "code": "BUS_UNVERIFIED",
                "message": "The servo bus is not verified online.",
                "recovery": next_action,
            }
        )
    elif servo_state == "none":
        next_action = "Connect one unloaded servo and run a bounded ID scan."
        blockers.append(
            {
                "code": "NO_SERVOS",
                "message": "No bus servos responded to the last scan.",
                "recovery": next_action,
            }
        )
    elif servo_state == "partial":
        missing = sorted(set(expected_ids) - set(responding_ids))
        next_action = "Check servo power and cabling, then scan the full bus again."
        blockers.append(
            {
                "code": "SERVOS_MISSING",
                "message": f"Expected servo IDs are missing: {', '.join(map(str, missing))}.",
                "recovery": next_action,
            }
        )
    else:
        next_action = "Review live telemetry and keep torque off until a supervised test."
    if torque == "unknown":
        blockers.append(
            {
                "code": "TORQUE_UNKNOWN",
                "message": "Servo torque state is unknown.",
                "recovery": "Cut servo power before touching the arm and restore controller telemetry.",
            }
        )

    telemetry_by_id = {
        int(item["id"]): item
        for item in telemetry
        if isinstance(item, dict)
        and isinstance(item.get("id"), int)
        and not isinstance(item.get("id"), bool)
    }
    unsafe_telemetry_recoveries: list[str] = []
    if connection == "online" and internal_servos == "faulted":
        recovery = (
            "Keep torque off, inspect each servo packet/error, then refresh status; "
            "do not assume an ID collision without scan evidence."
        )
        blockers.append(
            {
                "code": "SERVO_TELEMETRY_FAULT",
                "message": "The controller reports a generic servo telemetry fault.",
                "recovery": recovery,
            }
        )
        unsafe_telemetry_recoveries.append(recovery)

    mode_unknown = sorted(
        identifier
        for identifier in responding_ids
        if identifier not in telemetry_by_id
        or telemetry_by_id[identifier].get("operatingMode") is None
    )
    mode_not_position = sorted(
        identifier
        for identifier, item in telemetry_by_id.items()
        if item.get("operatingMode") is not None and item.get("operatingMode") != 0
    )
    stale = sorted(
        identifier
        for identifier, item in telemetry_by_id.items()
        if isinstance(item.get("packetAgeMs"), int)
        and not isinstance(item.get("packetAgeMs"), bool)
        and int(item["packetAgeMs"]) > PhysicalArmProfileService.MAX_FRESH_PACKET_AGE_MS
    )
    faulted_ids = sorted(
        identifier
        for identifier, item in telemetry_by_id.items()
        if isinstance(item.get("errors"), list) and bool(item["errors"])
    )
    if mode_unknown:
        recovery = (
            "Read each servo operating-mode register with torque off; only ST3215 "
            "position mode 0 may be commissioned."
        )
        blockers.append(
            {
                "code": "OPERATING_MODE_UNKNOWN",
                "message": f"Operating mode is unknown for servo IDs: {', '.join(map(str, mode_unknown))}.",
                "recovery": recovery,
            }
        )
        unsafe_telemetry_recoveries.append(recovery)
    if mode_not_position:
        recovery = (
            "Keep torque off and restore the confirmed ST3215 servos to position mode 0 "
            "before any hold or nudge."
        )
        blockers.append(
            {
                "code": "OPERATING_MODE_NOT_POSITION",
                "message": f"Servo IDs are not in position mode 0: {', '.join(map(str, mode_not_position))}.",
                "recovery": recovery,
            }
        )
        unsafe_telemetry_recoveries.append(recovery)
    if stale:
        recovery = "Keep torque off and restore fresh servo packets (250 ms or newer)."
        blockers.append(
            {
                "code": "SERVO_TELEMETRY_STALE",
                "message": f"Servo telemetry is stale for IDs: {', '.join(map(str, stale))}.",
                "recovery": recovery,
            }
        )
        unsafe_telemetry_recoveries.append(recovery)
    if faulted_ids:
        recovery = "Keep torque off, inspect the reported servo faults, and recapture clean telemetry."
        blockers.append(
            {
                "code": "SERVO_ERRORS",
                "message": f"Servo errors are present for IDs: {', '.join(map(str, faulted_ids))}.",
                "recovery": recovery,
            }
        )
        unsafe_telemetry_recoveries.append(recovery)
    if unsafe_telemetry_recoveries and motion_state not in {"stopped", "faulted"}:
        motion_state = "blocked"
        next_action = unsafe_telemetry_recoveries[0]
    if motion_state == "stopped":
        next_action = (
            "Keep the STOP latch active, restore addressed torque-off read-back, "
            "physically inspect the arm, then use RESET INSPECTED."
        )

    heartbeat_age = reported.get("heartbeatAgeMs")
    last_update_ms = (
        heartbeat_age
        if isinstance(heartbeat_age, int)
        and not isinstance(heartbeat_age, bool)
        and 0 <= heartbeat_age <= 86_400_000
        else None
    )
    packet_ages = [
        item.get("packetAgeMs")
        for item in telemetry
        if isinstance(item, dict)
        and isinstance(item.get("packetAgeMs"), int)
        and not isinstance(item.get("packetAgeMs"), bool)
    ]
    if packet_ages:
        last_update_ms = max(0, min(86_400_000, max(packet_ages)))
    bus_detail = {
        "online": "Servo bus is responding.",
        "offline": "Servo bus power or communication is offline.",
        "degraded": "Servo bus responses are corrupt or incomplete.",
        "unknown": "Servo bus has not been verified.",
    }[bus_state]
    if expected_ids:
        servo_detail = f"{len(set(responding_ids) & set(expected_ids))} of {len(expected_ids)} expected servos are responding."
    elif responding_ids:
        servo_detail = f"{len(responding_ids)} unbound servo IDs are responding."
    else:
        servo_detail = "No verified servo census is available."
    controller_details = {
        "online": "HAT safety controller handshake verified.",
        "offline": "HAT controller is not responding.",
        "not_configured": "HAT controller serial port is not configured.",
        "faulted": "HAT controller transport or protocol is faulted.",
        "closed": "HAT controller transport is closed.",
    }
    controller_connection = connection if connection in {"online", "offline", "not_configured", "faulted"} else "faulted"
    controller_layer: dict[str, object] = {
        "state": controller_connection,
        "detail": controller_details.get(connection, controller_details["faulted"]),
    }
    for key in ("controllerId", "bootId", "firmwareVersion", "protocolVersion"):
        if key in identity:
            controller_layer[key] = identity[key]
    if last_update_ms is not None:
        controller_layer["lastSeenMs"] = last_update_ms
    bus_layer: dict[str, object] = {
        "state": bus_state,
        "detail": bus_detail,
        "baud": reported.get("busBaud", 1_000_000),
    }
    if isinstance(last_scan, dict):
        bus_layer["lastScan"] = deepcopy(last_scan)
    stop_confirmed = motion_state == "stopped" and torque == "off"
    stop_state = "confirmed" if stop_confirmed else ("not_latched" if torque == "off" else "unknown")
    stop_detail = (
        "Controller STOP is latched and torque is confirmed off."
        if stop_confirmed
        else (
            "Controller STOP latch is accepted, but actuator torque removal is unproven."
            if motion_state == "stopped"
            else (
            "Motion is disarmed; software STOP is not latched."
            if stop_state == "not_latched"
            else "STOP delivery and actuator torque are not confirmed."
            )
        )
    )
    hardware_estop = reported.get("hardwareEstop", "not_detected")
    if hardware_estop not in {"not_detected", "active", "released", "unknown"}:
        hardware_estop = "unknown"
    return {
        "mode": "physical",
        "motionState": motion_state,
        "torqueState": torque,
        "lastUpdateMs": last_update_ms,
        "connections": {
            "pi": {"state": "online", "detail": "Gateway process is running."},
            "controller": controller_layer,
            "bus": bus_layer,
            "servos": {
                "state": servo_state,
                "detail": servo_detail,
                "expectedIds": expected_ids,
                "respondingIds": responding_ids,
            },
        },
        "servos": telemetry,
        "blockers": blockers,
        "nextAction": next_action,
        "stop": {"state": stop_state, "detail": stop_detail},
        "hardwareEstop": hardware_estop,
    }


def _require_online(controller: ArmController) -> None:
    if controller.transport_state().get("connection") != "online":
        raise _controller_offline()


# A servo that is already moving under the operator's hand reports transient
# status bits and its telemetry is one bus round behind. Gating continuous drive
# on the same evidence as a commissioning capture rejected commands mid-drag, so
# the live paths get their own looser bound. STOP, the host watchdog, and lease
# expiry are what actually de-energise the arm; this check only ever decided
# whether a command was worth sending.
LIVE_MOTION_PACKET_AGE_MS = 1_000


def _servo_telemetry(controller: ArmController, servo_id: int) -> dict[str, object] | None:
    telemetry = controller.transport_state().get("servosTelemetry")
    candidates = (
        [
            item
            for item in telemetry
            if isinstance(item, dict) and item.get("id") == servo_id
        ]
        if isinstance(telemetry, list)
        else []
    )
    return candidates[0] if len(candidates) == 1 else None


def _fresh_within(servo: dict[str, object] | None, limit_ms: int) -> bool:
    packet_age = servo.get("packetAgeMs") if isinstance(servo, dict) else None
    return (
        not isinstance(packet_age, bool)
        and isinstance(packet_age, int)
        and 0 <= packet_age <= limit_ms
    )


def _require_motion_safe_servo(
    controller: ArmController, servo_id: int, *, require_torque_off: bool = True
) -> None:
    """Fresh, fault-free, position-mode telemetry before torque or motion.

    `require_torque_off` is the right precondition for preparing a move from
    rest, but wrong for a direct MOVE: that already runs under live torque
    authority, so the servo is legitimately energised and demanding torque-off
    would make the command impossible to satisfy. That same flag now selects the
    looser live-motion bound, because a MOVE is only ever sent while the joint is
    already under the operator's control.
    """

    servo = _servo_telemetry(controller, servo_id)
    if require_torque_off:
        acceptable = (
            servo is not None
            and servo.get("operatingMode") in HOLDING_MODES
            and servo.get("torqueState") == "off"
            and servo.get("errors") == []
            and _fresh_within(servo, PhysicalArmProfileService.MAX_FRESH_PACKET_AGE_MS)
        )
    else:
        acceptable = (
            servo is not None
            and servo.get("operatingMode") in HOLDING_MODES
            and servo.get("torqueState") != "unknown"
            and _fresh_within(servo, LIVE_MOTION_PACKET_AGE_MS)
        )
    if not acceptable:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=(
                "Fresh, fault-free, torque-off ST3215 position-mode telemetry is "
                "required before torque or motion. Capture the servo again."
                if require_torque_off
                else "That servo is not reporting live position-mode telemetry. "
                "Reconnect the controller and try again."
            ),
        )


def _require_hold_safe_servo(controller: ArmController, servo_id: int) -> None:
    servo = _servo_telemetry(controller, servo_id)
    if not (
        servo is not None
        and servo.get("operatingMode") in HOLDING_MODES
        and servo.get("torqueState") in {"off", "on"}
        and _fresh_within(servo, LIVE_MOTION_PACKET_AGE_MS)
    ):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Servo {servo_id} is not reporting live position-mode telemetry, so it cannot be held.",
        )


def create_physical_arm_router(
    controller: ArmController,
    *,
    profile_store: PhysicalArmProfileStore | None = None,
    operation_lock: threading.RLock | None = None,
    admission_lock: threading.Lock | None = None,
    mutation_context: Callable[[], AbstractContextManager[object]] | None = None,
    clear_mutation_context: Callable[[], AbstractContextManager[object]] | None = None,
    stop_callback: Callable[[], dict[str, object]] | None = None,
    reset_callback: Callable[[], dict[str, object]] | None = None,
) -> APIRouter:
    configured_store = profile_store or PhysicalArmProfileStore()
    evidence = CommissioningEvidenceLedger()
    profile_service = PhysicalArmProfileService(controller, configured_store, evidence)
    prepared_lock = threading.RLock()
    prepared_http_proposals: dict[str, dict[str, object]] = {}
    shared_operation_lock = operation_lock or threading.RLock()
    # RLock ownership is an OS-thread property, not an asyncio-task property.
    # Two concurrent async requests on the same event-loop thread could
    # otherwise both re-enter `shared_operation_lock` while the first awaited
    # its request body. This plain admission gate gives requests distinct,
    # non-reentrant ownership; the shared RLock still excludes SimpleArm's
    # composed operation running in its worker thread.
    physical_admission_lock = admission_lock or threading.Lock()

    def _guard_operation(endpoint, *, clearing: bool):
        """Own all SimpleArm writer lanes for a commissioning operation."""

        def physical_mutation_context() -> AbstractContextManager[object]:
            selected = clear_mutation_context if clearing else mutation_context
            return selected() if selected is not None else nullcontext()

        if inspect.iscoroutinefunction(endpoint):
            @wraps(endpoint)
            async def guarded_async(*args, **kwargs):
                if not physical_admission_lock.acquire(blocking=False):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Another physical arm operation is in progress.",
                    )
                operation_acquired = False
                try:
                    operation_acquired = shared_operation_lock.acquire(blocking=False)
                    if not operation_acquired:
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Another physical arm operation is in progress.",
                        )
                    with physical_mutation_context():
                        return await endpoint(*args, **kwargs)
                except HTTPException:
                    raise
                except Exception as error:
                    raise _translate(error) from error
                finally:
                    if operation_acquired:
                        shared_operation_lock.release()
                    physical_admission_lock.release()

            return guarded_async

        @wraps(endpoint)
        def guarded_sync(*args, **kwargs):
            if not physical_admission_lock.acquire(blocking=False):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Another physical arm operation is in progress.",
                )
            operation_acquired = False
            try:
                operation_acquired = shared_operation_lock.acquire(blocking=False)
                if not operation_acquired:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Another physical arm operation is in progress.",
                    )
                with physical_mutation_context():
                    return endpoint(*args, **kwargs)
            except HTTPException:
                raise
            except Exception as error:
                raise _translate(error) from error
            finally:
                if operation_acquired:
                    shared_operation_lock.release()
                physical_admission_lock.release()

        return guarded_sync

    def guard_operation(endpoint):
        return _guard_operation(endpoint, clearing=False)

    def guard_clear_operation(endpoint):
        return _guard_operation(endpoint, clearing=True)

    def invalidate_prepared_http_proposals() -> None:
        with prepared_lock:
            prepared_http_proposals.clear()
    router = APIRouter(
        prefix="/api/robot/physical/arm", tags=["physical-arm-commissioning"]
    )

    @router.get("/status")
    def controller_status() -> dict[str, object]:
        return _public_status(controller, configured_store)

    @router.post("/controller/reconnect")
    @guard_operation
    def reconnect_controller() -> dict[str, object]:
        invalidate_prepared_http_proposals()
        try:
            controller.reconnect()
            return _public_status(controller, configured_store)
        except Exception as error:
            raise _translate(error) from error

    @router.get("/calibration/profile")
    def profile_status() -> dict[str, object]:
        return profile_service.status()

    @router.post("/calibration/profile")
    @guard_operation
    async def commit_profile(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, SavePhysicalProfileRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return profile_service.commit(request)
        except PhysicalProfileConflict as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except PhysicalProfileStoreError:
            raise HTTPException(
                status_code=503,
                detail="The physical calibration profile could not be stored safely.",
            ) from None
        except Exception as error:
            raise _translate(error) from error

    @router.post("/bus/scan")
    @guard_operation
    async def scan(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, ScanRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return controller.scan(request.minId, request.maxId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/assign-id")
    @guard_operation
    async def assign_id(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, AssignIdRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return controller.assign_id(request.oldId, request.newId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/set-position-mode")
    @guard_operation
    async def set_position_mode(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, SetPositionModeRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return controller.set_position_mode(request.servoId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/capture")
    @guard_operation
    async def capture(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, CaptureRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            observation = controller.capture(request.servoId)
            observed_id = observation.get("id", observation.get("servoId"))
            if observed_id != request.servoId:
                raise ControllerProtocolError("capture servo identity mismatch")
            evidence_id = evidence.record_capture(
                _current_boot_id(controller), request.servoId, observation
            )
            result: dict[str, object] = {
                "servoId": request.servoId,
                "rawPosition": observation.get("rawPosition"),
                "sampleCount": observation.get("sampleCount"),
                "variationTicks": observation.get("variationTicks", 0),
                "evidenceId": evidence_id,
            }
            if observation.get("odometerValid") is True:
                result["odometerValid"] = True
                result["revolutions"] = observation.get("revolutions")
                result["multiTurnPosition"] = observation.get("multiTurnPosition")
            else:
                result["odometerValid"] = False
            if "capturedAt" in observation:
                result["capturedAt"] = observation["capturedAt"]
            return result
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/odometer/zero")
    @guard_operation
    async def odometer_zero(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, OdometerRequest)
        _require_online(controller)
        try:
            return controller.odometer_zero(request.servoId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/odometer/read")
    @guard_operation
    async def odometer_read(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, OdometerRequest)
        _require_online(controller)
        try:
            return controller.odometer_read(request.servoId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/registers/read")
    @guard_operation
    async def read_registers(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, RegisterReadRequest)
        _require_online(controller)
        try:
            return controller.read_servo_registers(
                request.servoId, request.address, request.length
            )
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/move")
    @guard_operation
    async def move_servo(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, MoveRequest)
        _require_online(controller)
        _require_motion_safe_servo(controller, request.servoId, require_torque_off=False)
        try:
            return controller.move(
                request.servoId, request.goal, request.speed, request.acceleration
            )
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/torque-lease")
    @guard_operation
    async def torque_lease(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, TorqueLeaseRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        _require_motion_safe_servo(controller, request.servoId)
        try:
            return controller.torque_lease(request.servoId, request.leaseMs)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/torque-off")
    @guard_operation
    async def torque_off(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, TorqueOffRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return controller.torque_off(request.servoId)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servos/hold-set")
    @guard_operation
    async def hold_set(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, HoldSetRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        for servo_id in request.servoIds:
            _require_hold_safe_servo(controller, servo_id)
        try:
            return controller.set_hold_servos(request.servoIds, request.leaseMs)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/tests/prepare-nudge")
    @guard_operation
    async def prepare_nudge(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, PrepareNudgeRequest)
        _require_online(controller)
        _require_motion_safe_servo(controller, request.servoId)
        try:
            boot_id = _current_boot_id(controller)
            zero_raw = evidence.verified_capture_position(
                request.zeroEvidenceId, boot_id, request.servoId
            )
            prepared = controller.prepare_nudge(
                request.servoId,
                request.deltaTicks,
                request.speed,
                request.acceleration,
            )
            proposal_id = prepared.get("proposalId")
            expires_ms = prepared.get("expiresInMs")
            if (
                not isinstance(proposal_id, str)
                or isinstance(expires_ms, bool)
                or not isinstance(expires_ms, int)
                or not 1 <= expires_ms <= 30_000
                or prepared.get("servoId") != request.servoId
                or prepared.get("deltaTicks") != request.deltaTicks
            ):
                raise ControllerProtocolError("invalid nudge proposal")
            start_raw = prepared.get("startRawPosition")
            if (
                isinstance(start_raw, bool)
                or not isinstance(start_raw, int)
                or not 0 <= start_raw <= 4095
                or abs(start_raw - zero_raw) > 8
            ):
                # PREPARE is read-only. Explicit torque-off also invalidates
                # the controller proposal before withholding its token.
                controller.torque_off(request.servoId)
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "The servo is not within 8 raw ticks of the selected live "
                        "zero capture. Reposition with torque off and capture zero again."
                    ),
                )
            result: dict[str, object] = {
                "proposalId": proposal_id,
                "servoId": request.servoId,
                "deltaTicks": request.deltaTicks,
                "expiresInMs": expires_ms,
            }
            proposal_hash = prepared.get("proposalHash")
            if proposal_hash is not None:
                result["proposalHash"] = proposal_hash
            with prepared_lock:
                prepared_http_proposals.clear()
                prepared_http_proposals[proposal_id] = {
                    "servoId": request.servoId,
                    "deltaTicks": request.deltaTicks,
                    "bootId": boot_id,
                    "expiresAt": time.monotonic() + (expires_ms / 1000.0),
                }
            return result
        except PhysicalProfileConflict as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except HTTPException:
            raise
        except Exception as error:
            raise _translate(error) from error

    @router.post("/tests/execute-nudge")
    @guard_operation
    async def execute_nudge(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, ExecuteNudgeRequest)
        _require_online(controller)
        try:
            with prepared_lock:
                prepared = prepared_http_proposals.pop(request.proposalId, None)
            if prepared is None:
                raise ControllerCommandError("PROPOSAL_USED_OR_UNKNOWN")
            if time.monotonic() >= float(prepared["expiresAt"]):
                raise ControllerCommandError("PROPOSAL_EXPIRED")
            boot_id = _current_boot_id(controller)
            if prepared.get("bootId") != boot_id:
                raise ControllerCommandError("BOOT_CHANGED")
            completed = controller.execute_nudge(
                request.proposalId, request.proposalHash
            )
            if completed.get("servoId") != prepared.get("servoId"):
                raise ControllerProtocolError("nudge servo identity mismatch")
            evidence_id = evidence.record_nudge(
                boot_id,
                {
                    **completed,
                    "commandedDeltaTicks": prepared["deltaTicks"],
                },
            )
            return {
                "proposalId": completed.get("proposalId"),
                "completed": completed.get("completed"),
                "startRawPosition": completed.get("startRawPosition"),
                "targetRawPosition": completed.get("targetRawPosition"),
                "rawPosition": completed.get(
                    "rawPosition"
                ),
                "measuredDeltaTicks": completed.get("measuredDeltaTicks"),
                "positionErrorTicks": completed.get("positionErrorTicks"),
                "torqueState": completed.get("torqueState"),
                "evidenceId": evidence_id,
            }
        except Exception as error:
            raise _translate(error) from error

    @router.post("/stop")
    def stop_arm() -> dict[str, object]:
        invalidate_prepared_http_proposals()
        try:
            return stop_callback() if stop_callback is not None else controller.stop()
        except (
            ControllerUnavailableError,
            ControllerTransportError,
            ControllerProtocolError,
            ControllerCommandError,
        ):
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="STOP delivery is unknown. Cut servo power before touching the arm.",
            ) from None
        except Exception as error:
            raise _translate(error) from error

    @router.post("/reset")
    @guard_clear_operation
    async def reset_arm(http_request: Request) -> dict[str, object]:
        await _strict_body(http_request, ResetRequest)
        _require_online(controller)
        invalidate_prepared_http_proposals()
        try:
            return reset_callback() if reset_callback is not None else controller.reset(
                inspected=True
            )
        except Exception as error:
            raise _translate(error) from error

    return router


__all__ = [
    "AssignIdRequest",
    "CaptureRequest",
    "ExecuteNudgeRequest",
    "PhysicalArmProfileService",
    "PhysicalArmProfileStore",
    "PhysicalProfileConflict",
    "PhysicalProfileStoreError",
    "PrepareNudgeRequest",
    "ResetRequest",
    "SavePhysicalProfileRequest",
    "ScanRequest",
    "TorqueLeaseRequest",
    "TorqueOffRequest",
    "create_physical_arm_router",
]
