"""The whole arm, in four numbers per joint.

Design brief, verbatim from the operator: the ESP32 sends telemetry up and takes
positions down, nothing else. The Pi owns limits, state and every decision. Per
servo it needs a zero, a min, a max and a gear ratio; inside that range the joint
is free to move regardless of torque, pose or anything else, because a joint that
cannot leave its limits cannot do damage.

So this module is the whole physical arm API. The direct target routes remain
simple and compatible, while the optional plan/apply routes hold a short-lived,
one-use physical pose for explicit review before motion. Both paths resolve
limits and the floor guard here on the Pi. STOP still cuts everything, and the
ESP32 keeps its own fail-safes (boot torque-off, host watchdog, hold expiry)
because those cost the operator nothing and are what make walking away safe.

Deliberately parallel to `physical_arm_api.py` rather than carved out of it: the
commissioning flow stays reachable until this one has driven the real arm.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Mapping
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import hmac
import inspect
import json
import logging
import math
import os
import re
import secrets
from pathlib import Path
import tempfile
import threading
import time
from functools import wraps
from typing import Annotated, Literal

from fastapi import APIRouter, HTTPException, Request, status
from pydantic import Field, model_validator
from starlette.concurrency import run_in_threadpool

from .request_validation import strict_body as _strict_body
from .arm_controller import (
    ArmController,
    ControllerCommandError,
    ControllerProtocolError,
    ControllerTransportError,
    ControllerUnavailableError,
)
from .strict_contract import StrictContract
from .camera_api import (
    CameraEvidenceCaptureCancelledError,
    CameraEvidenceCaptureError,
    CameraEvidenceService,
    CameraEvidenceTransferUnavailableError,
    CameraUnavailableError,
)
from .camera_profiles import CAPTURE_PROFILE_DETAIL
from .serial_arm_controller import MAX_HOLD_SERVOS, MOTION_POLICY, MULTI_TURN_GOAL_LIMIT

# joint_4 is not part of the arm: it carries the camera, so it is driven and
# calibrated like any other joint but never enters the kinematic chain.
JOINT_IDS = ("joint_1", "joint_2", "joint_3", "joint_4")
JointId = Literal["joint_1", "joint_2", "joint_3", "joint_4"]
DEFAULT_SERVO_ID = {"joint_1": 1, "joint_2": 2, "joint_3": 3, "joint_4": 4}
JOINT_NAMES = {
    "joint_1": "Base",
    "joint_2": "Shoulder",
    "joint_3": "Elbow",
    "joint_4": "Camera",
}
# The camera servo is a Waveshare SC09, an SCS-family part; the three arm servos
# are ST3215s, which are STS-family. They share framing and the ID register but
# not byte order or register layout, and nothing on the wire tells them apart
# reliably -- so which is which is declared here and pushed to the controller.
SCS_JOINTS = frozenset({"joint_4"})
# The base drives a ring gear, so one motor turn is only 360/ratio degrees at
# the joint, and a single-turn goal cannot reach past it.
#
# The HAT owns one counted absolute frame and closes the Base goal against live
# encoder samples in position mode. The Pi only converts joint degrees through
# the gearbox and names a destination in that frame. This avoids Mode-3 relative
# countdowns entirely: repeating a goal is idempotent and retargeting never has
# to finish an obsolete hop first.
MULTI_TURN_JOINTS: frozenset[str] = frozenset({"joint_1"})
MULTI_TURN_REQUIRED_CAPABILITIES = frozenset({"multi_turn_absolute_v1"})

MAX_HELD = MAX_HOLD_SERVOS

# Measured off the physical arm 2026-08-07, millimetres. The 2 cm the two link
# plates are offset by is already inside these lengths, not a separate term.
# DISTAL is elbow-to-tip with the camera fully extended forward, the worst case
# for hitting something.
BASE_HEIGHT_MM = 60.0
UPPER_ARM_MM = 180.0
DISTAL_MM = 220.0
# Keep-out plane above the floor. NOT a calibration limit: calibration is each
# joint's range of travel, this is a region of space the whole arm is kept out
# of, and the two are enforced independently. Reach is 400 mm from a pivot only
# 60 mm up, so the arm can drive itself through the table without this.
FLOOR_MM = 40.0
# joint_1 is a yaw and joint_4 rides on the arm, so neither can change a height.
FLOOR_JOINTS = ("joint_2", "joint_3")
# Servo frame to twin frame. These are the same two numbers as MODEL_OFFSET in
# the 2D view and must stay in step with it: servo zeros sit mid-travel, so the
# upper arm is straight up at servo shoulder 0 and the arm is straight at servo
# elbow 90, while the height formula is written for a frame where shoulder 0 is
# horizontal and elbow 0 is collinear.
SHOULDER_MODEL_OFFSET_DEGREES = 90.0
ELBOW_MODEL_OFFSET_DEGREES = -90.0

# A prepared physical pose is intentionally brief and one-use. Thirty seconds
# is long enough for a dashboard confirmation or one model reasoning turn, but
# short enough that a forgotten plan cannot become a later surprise.
PLAN_TTL_MS = 30_000
PLAN_CACHE_LIMIT = 32
SEQUENCE_TTL_MS = 120_000
SEQUENCE_CACHE_LIMIT = 32
SEQUENCE_RESULT_CACHE_LIMIT = 32
SEQUENCE_MIN_WAYPOINTS = 2
SEQUENCE_MAX_WAYPOINTS = 8
PLAN_START_POSE_TOLERANCE_DEGREES = 2.0
PLAN_MAX_HEARTBEAT_AGE_MS = 2_000
PLAN_MAX_SERVO_PACKET_AGE_MS = 250
PLAN_MAX_ODOMETER_SAMPLE_AGE_MS = 250
SCENE_RAY_MM = 140.0
_PLAN_ID_PATTERN = r"^armplan_[A-Za-z0-9_-]{8,64}$"
_PLAN_DIGEST_PATTERN = r"^sha256:[0-9a-f]{64}$"
_SEQUENCE_ID_PATTERN = r"^armseq_[A-Za-z0-9_-]{8,64}$"
_SEQUENCE_LABEL_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,39}$"
_LIVE_FOLLOW_ID_PATTERN = r"^armfollow_[A-Za-z0-9_-]{16,64}$"
_LIVE_FOLLOW_ATTEMPT_ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{7,127}$"


def lowest_point_mm(shoulder_degrees: float, elbow_degrees: float) -> float:
    """Height of the lowest part of the arm above the floor, in millimetres.

    Takes SERVO degrees, which is what every caller has, and converts. The two
    frames are not the same and conflating them was a real bug: the servo zeros
    sit mid-travel, so servo shoulder 0 points the upper arm straight UP and the
    arm is straight at servo elbow 90. The twin's frame, in which the usual
    sin(shoulder) / sin(shoulder + elbow) formula holds, is offset by the same
    MODEL_OFFSET the 2D view uses -- shoulder +90, elbow -90.

    Substituting those offsets, the tip term is unchanged because the two
    offsets cancel, but the elbow term becomes cos(servo_shoulder). Using
    sin there put the elbow 180 mm low at a typical pose (measured: computed
    60 mm while the joint was physically at 240 mm), which dragged the guard
    below the plane for any servo shoulder under about -6 degrees and refused
    most forward reach while the arm was nowhere near the desk.

    Both links are straight segments, so a segment's minimum height is always at
    one of its endpoints: testing the elbow and the tip is exhaustive. The
    shoulder pivot is fixed at BASE_HEIGHT_MM and never decides the answer.

    Guarding only the tip would be wrong -- with the upper arm swung down the
    ELBOW is the buried point while the tip folds back up clear.
    """

    shoulder = math.radians(shoulder_degrees + SHOULDER_MODEL_OFFSET_DEGREES)
    elbow_angle = math.radians(elbow_degrees + ELBOW_MODEL_OFFSET_DEGREES)
    elbow = BASE_HEIGHT_MM + UPPER_ARM_MM * math.sin(shoulder)
    tip = elbow + DISTAL_MM * math.sin(shoulder + elbow_angle)
    return min(elbow, tip)


def _periodic_angles(
    low_degrees: float,
    high_degrees: float,
    offset_degrees: float,
    period_degrees: float,
) -> list[float]:
    """Return every offset + k*period angle inside a closed interval."""

    low, high = sorted((float(low_degrees), float(high_degrees)))
    first = math.ceil((low - offset_degrees) / period_degrees)
    last = math.floor((high - offset_degrees) / period_degrees)
    return [offset_degrees + index * period_degrees for index in range(first, last + 1)]


def swept_lowest_point_mm(
    start_shoulder_degrees: float,
    start_elbow_degrees: float,
    target_shoulder_degrees: float,
    target_elbow_degrees: float,
) -> float:
    """Exact minimum height over independently timed shoulder/elbow motion.

    MOVE commands are issued close together, but the two servos do not promise
    synchronized progress.  The physically possible sweep is therefore the
    whole rectangle between the two start/target angles, not merely the line
    obtained by blending both joints with one shared time value.

    The endpoint heights are trigonometric and smooth.  Their global minimum on
    a closed rectangle can only occur at a corner, a stationary point on one of
    the four edges, or an interior stationary point.  Enumerating those finite
    candidates is conservative without the holes introduced by sampled motion.
    """

    shoulder_low, shoulder_high = sorted(
        (float(start_shoulder_degrees), float(target_shoulder_degrees))
    )
    elbow_low, elbow_high = sorted(
        (float(start_elbow_degrees), float(target_elbow_degrees))
    )
    candidates: set[tuple[float, float]] = {
        (shoulder, elbow)
        for shoulder in (shoulder_low, shoulder_high)
        for elbow in (elbow_low, elbow_high)
    }

    # The upper-link endpoint is BASE + L1*cos(servo shoulder).  Its minima are
    # at shoulder = 180 + 360k and do not depend on elbow.
    for shoulder in _periodic_angles(shoulder_low, shoulder_high, 180.0, 360.0):
        candidates.add((shoulder, elbow_low))

    # On a fixed-shoulder edge, the tip term varies as sin(shoulder + elbow).
    for shoulder in (shoulder_low, shoulder_high):
        for combined in _periodic_angles(
            shoulder + elbow_low,
            shoulder + elbow_high,
            -90.0,
            360.0,
        ):
            candidates.add((shoulder, combined - shoulder))

    # On a fixed-elbow edge the tip is A*cos(shoulder)+B*sin(shoulder).
    # Its minimum occurs at phase + 180 degrees, repeated every full turn.
    for elbow in (elbow_low, elbow_high):
        elbow_radians = math.radians(elbow)
        coefficient_cos = UPPER_ARM_MM + DISTAL_MM * math.sin(elbow_radians)
        coefficient_sin = DISTAL_MM * math.cos(elbow_radians)
        phase = math.degrees(math.atan2(coefficient_sin, coefficient_cos))
        for shoulder in _periodic_angles(
            shoulder_low,
            shoulder_high,
            phase + 180.0,
            360.0,
        ):
            candidates.add((shoulder, elbow))

    # Interior tip stationary points satisfy shoulder = 180k and
    # shoulder+elbow = 90+180m.  Some are maxima or saddles, but evaluating all
    # of them is cheap and keeps the proof straightforward.
    for shoulder in _periodic_angles(shoulder_low, shoulder_high, 0.0, 180.0):
        for combined in _periodic_angles(
            shoulder + elbow_low,
            shoulder + elbow_high,
            90.0,
            180.0,
        ):
            candidates.add((shoulder, combined - shoulder))

    return min(lowest_point_mm(shoulder, elbow) for shoulder, elbow in candidates)


def _planar_inverse(
    radial_mm: float, height_mm: float
) -> tuple[list[dict[str, object]], dict[str, float], bool]:
    """Solve both elbow branches for a side-view point.

    Input height is absolute above the physical floor, matching ``arm_scene``.
    Returned shoulder/elbow values are servo degrees, not model degrees.
    A point outside the two-link annulus is projected to its nearest edge so the
    preview follows the same "as far as the arm can go" rule as the dashboard.
    """

    vertical = height_mm - BASE_HEIGHT_MM
    distance = math.hypot(radial_mm, vertical)
    minimum = abs(UPPER_ARM_MM - DISTAL_MM) + 1e-6
    maximum = UPPER_ARM_MM + DISTAL_MM
    held = min(maximum, max(minimum, distance))
    projected = not math.isclose(held, distance, abs_tol=1e-6)
    if distance < 1e-6:
        on_ring_radial, on_ring_vertical = 0.0, held
    else:
        ratio = held / distance
        on_ring_radial = radial_mm * ratio
        on_ring_vertical = vertical * ratio
    cosine = max(
        -1.0,
        min(
            1.0,
            (held * held - UPPER_ARM_MM * UPPER_ARM_MM - DISTAL_MM * DISTAL_MM)
            / (2.0 * UPPER_ARM_MM * DISTAL_MM),
        ),
    )
    magnitude = math.acos(cosine)
    candidates: list[dict[str, object]] = []
    for elbow_model, branch in ((magnitude, "down"), (-magnitude, "up")):
        shoulder_model = math.atan2(on_ring_vertical, on_ring_radial) - math.atan2(
            DISTAL_MM * math.sin(elbow_model),
            UPPER_ARM_MM + DISTAL_MM * math.cos(elbow_model),
        )
        candidates.append(
            {
                "branch": branch,
                "servoPose": {
                    "joint_2": math.degrees(shoulder_model)
                    - SHOULDER_MODEL_OFFSET_DEGREES,
                    "joint_3": math.degrees(elbow_model)
                    - ELBOW_MODEL_OFFSET_DEGREES,
                },
            }
        )
    return (
        candidates,
        {
            "radialMm": round(on_ring_radial, 3),
            "heightMm": round(on_ring_vertical + BASE_HEIGHT_MM, 3),
        },
        projected,
    )


def _scene_points(pose: dict[str, float | None]) -> dict[str, object] | None:
    """Side-view points for a measured or planned servo-degree pose."""

    shoulder_degrees = pose.get("joint_2")
    elbow_degrees = pose.get("joint_3")
    if not isinstance(shoulder_degrees, (int, float)) or not isinstance(
        elbow_degrees, (int, float)
    ):
        return None
    shoulder_model = shoulder_degrees + SHOULDER_MODEL_OFFSET_DEGREES
    elbow_model = elbow_degrees + ELBOW_MODEL_OFFSET_DEGREES
    forearm_absolute = shoulder_model + elbow_model
    shoulder_rad = math.radians(shoulder_model)
    forearm_rad = math.radians(forearm_absolute)
    elbow = {
        "radialMm": round(UPPER_ARM_MM * math.cos(shoulder_rad), 3),
        "heightMm": round(BASE_HEIGHT_MM + UPPER_ARM_MM * math.sin(shoulder_rad), 3),
    }
    tip = {
        "radialMm": round(elbow["radialMm"] + DISTAL_MM * math.cos(forearm_rad), 3),
        "heightMm": round(elbow["heightMm"] + DISTAL_MM * math.sin(forearm_rad), 3),
    }
    points: dict[str, object] = {
        "base": {"radialMm": 0.0, "heightMm": 0.0},
        "shoulder": {"radialMm": 0.0, "heightMm": BASE_HEIGHT_MM},
        "elbow": elbow,
        "tipCamera": tip,
    }
    camera_degrees = pose.get("joint_4")
    camera_absolute: float | None = None
    if isinstance(camera_degrees, (int, float)):
        camera_absolute = forearm_absolute - camera_degrees
        camera_rad = math.radians(camera_absolute)
        points["cameraRayEnd"] = {
            "radialMm": round(tip["radialMm"] + SCENE_RAY_MM * math.cos(camera_rad), 3),
            "heightMm": round(tip["heightMm"] + SCENE_RAY_MM * math.sin(camera_rad), 3),
        }
    return {
        "servoDegrees": {
            name: round(float(value), 3) if isinstance(value, (int, float)) else None
            for name, value in pose.items()
        },
        "modelDegrees": {
            "shoulder": round(shoulder_model, 3),
            "elbow": round(elbow_model, 3),
            "forearmAbsolute": round(forearm_absolute, 3),
            "cameraRayAbsolute": round(camera_absolute, 3)
            if camera_absolute is not None
            else None,
        },
        "points": points,
    }


def _sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def _expires_at(seconds_since_epoch: float) -> str:
    return (
        datetime.fromtimestamp(seconds_since_epoch, timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )

_LOG = logging.getLogger(__name__)

TICKS_PER_TURN = 4096
RAW_MAX = 4095
# The SC09 is a different encoder on the same arc: 1024 counts per revolution,
# against the ST3215's 4096. Sharing the ST3215 count would let the panel name a
# goal past 1023, which the controller refuses outright.
#
# 360 and not the 300 the SC-series datasheets quote: this unit's pot wraps
# 1023 -> 0 and carries on turning, so its sensed arc IS one whole revolution.
# Measured against the arm, 530 counts reads as ~180 degrees, which fits 360
# (186) and does not fit 300 (155).
SCS_TICKS_PER_TURN = 1024
SCS_SPAN_DEGREES = 360.0
STS_SPAN_DEGREES = 360.0
# Kept under the policy the Pi pushes to the controller, so a stale controller
# policy can never turn a legal drive command into OUT_OF_RANGE.
DEFAULT_SPEED = min(2000, MOTION_POLICY["MAX_SPEED"])
# A drag is now one goal, sent when the handle is let go, so the servo makes a
# single long move instead of chasing a goal that keeps shifting a few ticks.
# The gentle ramp that smoothed the old stream just makes one move feel heavy.
DEFAULT_ACCEL = min(40, MOTION_POLICY["MAX_ACCEL"])
# Fast Follow is deliberately narrower than ordinary direct motion. The Pi is
# the session owner: the browser may replace the latest target, but it cannot
# extend these physical bounds or queue an unbounded trail of stale poses.
LIVE_FOLLOW_SCHEMA = "arm-live-follow-v2"
LIVE_FOLLOW_JOINTS = ("joint_2", "joint_3")
LIVE_FOLLOW_CAPABILITY = "follow_set_feedback_v1"
LIVE_FOLLOW_READ_CAPABILITY = "follow_feedback_v1"
LIVE_FOLLOW_MAX_DURATION_SECONDS = 30.0
LIVE_FOLLOW_INPUT_LEASE_SECONDS = 0.4
LIVE_FOLLOW_START_TIMEOUT_MS = 1_500
LIVE_FOLLOW_MIN_DISPATCH_SECONDS = 0.05
LIVE_FOLLOW_FEEDBACK_SECONDS = 0.05
# The HAT's ordinary lease supervisor already treats a short run of bad samples
# from a moving servo as transient.  Mirror that bounded tolerance for the
# read-only idle feedback lane: FOLLOW_READ does not dispatch motion, so one
# FEEDBACK_UNAVAILABLE receipt is not motion ambiguity and must not tear down an
# otherwise healthy live-follow session.  A sustained run still faults closed.
LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE = MOTION_POLICY[
    "SUPERVISION_TOLERANCE"
]
LIVE_FOLLOW_DEFAULT_MAX_DELTA_DEGREES = 30
LIVE_FOLLOW_MAX_DELTA_CEILING_DEGREES = 90
LIVE_FOLLOW_START_LIMIT_TOLERANCE_DEGREES = 0.1
# The controller's hold expires on its own; the Pi renews it while torque is
# meant to be on. Renewing at roughly half the lease survives one lost round.
HOLD_LEASE_MS = 2_000
HOLD_RENEW_SECONDS = 0.8
# One Pi-side loop owns every periodic serial command. The browser used to drive
# STATUS itself, once per poll: a three-servo STATUS is up to ~2 KB, which is
# ~175 ms of wire time at 115200 baud, so polling at 150 ms saturated the link
# and left HOLD_SET and MOVE queued behind it. A single sequential loop cannot
# oversubscribe the port no matter how fast the browser asks.
REFRESH_SECONDS = 0.25
SERVICE_TICK_SECONDS = 0.05
# These are controller-reported STOP reason tokens. Automatic reasons may be
# cleared only by the complete all-off proof below; EXPLICIT_STOP is never
# eligible for that path.
CLEAR_REQUIRED_STOP_REASONS = frozenset(
    {
        "MOVE_SET_FAILED",
        "MOVE_SET_UNCONFIRMED",
        "EXPLICIT_STOP",
        "SAFETY_FAULT",
    }
)
PERSISTED_CLEAR_REQUIRED_REASONS = CLEAR_REQUIRED_STOP_REASONS | frozenset(
    {"STOP_DELIVERY_UNKNOWN"}
)
# The Pi can finish booting before the HAT. The HTTP gateway deliberately stays
# available in that case, so systemd never restarts it and nothing else used to
# retry the serial handshake. One bounded attempt every two seconds makes boot
# order irrelevant without turning a dashboard poll into controller I/O.
CONTROLLER_RECONNECT_INTERVAL_SECONDS = 2.0
AUTOMATIC_STOP_RECOVERY_INTERVAL_SECONDS = 2.0
# Firmware 2.4 can turn a host-watchdog/torque-proof interruption into a
# reasonless STOP even after every actuator is visibly off.  Recovery is only
# allowed from a complete, recent all-servo sample; this is deliberately looser
# than arrival telemetry because a stopped HAT may need one full bus sweep.
AUTOMATIC_STOP_RECOVERY_MAX_PACKET_AGE_MS = 1_000
AUTOMATIC_STOP_REASONS = frozenset(
    {"MOVE_SET_FAILED", "MOVE_SET_UNCONFIRMED", "SAFETY_FAULT"}
)
# A drag is a stream of MOVEs and it owns the link while it lasts. Telemetry
# going briefly stale during a deliberate motion is invisible; a laggy joint is
# not.
MOVE_QUIET_SECONDS = 0.25
# Only HEARTBEAT and HOLD_SET refresh the controller's 750 ms host watchdog, and
# a stale watchdog voids `holdSetActive_` outright -- authority dies on the spot,
# a whole turn of the hold lease early. The heartbeat thread wants the wire every
# 200 ms but shares one unfair lock with commands that cost a STATUS either side,
# so a burst of MOVEs re-takes the lock before the heartbeat can get in. Measured
# on the bench: a two-joint move refused the SECOND MOVE with NO_TORQUE_LEASE
# 0.97 s after a confirmed HOLD_SET, with the controller reporting an empty hold
# set. Standing aside for a beat between MOVEs is what lets the heartbeat land.
MOVE_YIELD_SECONDS = 0.05
# Close enough to call a joint arrived. A healthy move lands inside 0.3 deg;
# the ones a lapsed lease cut short sat tens of degrees out, so a degree of slack
# separates the two without re-driving a joint that is simply where it was asked
# to be.
ARRIVAL_DEGREES = 1.0
# The lower-resolution SC09 camera joint can settle several encoder counts
# away from its requested raw goal while already reporting stationary.  The
# loaded Module 3 Wide mount was measured settling 16 counts / 5.625 degrees
# short after all bounded chase attempts. Six degrees rounds to a 17-count
# Camera-only arrival/chase tolerance. Joints 1-3, and the separate capture
# drift guard, remain on ARRIVAL_DEGREES.
CAMERA_ARRIVAL_DEGREES = 6.0


def _arrival_tolerance_degrees(joint_name: str) -> float:
    return CAMERA_ARRIVAL_DEGREES if joint_name == "joint_4" else ARRIVAL_DEGREES


def _arrival_tolerance_ticks(joint_name: str, ticks_per_degree: float) -> int:
    tolerance_degrees = _arrival_tolerance_degrees(joint_name)
    return max(1, round(tolerance_degrees * ticks_per_degree))


# How many times a goal is re-sent before the Pi accepts the joint is not going
# to get there. Bounded because a joint stalled on something solid must not be
# driven at it forever.
ARRIVAL_ATTEMPTS = 3
# How long a goal stays worth chasing. A 180 deg swing at DEFAULT_SPEED is a
# couple of seconds; past this the command is history and re-sending it would
# move the arm long after the operator stopped expecting it to.
ARRIVAL_SECONDS = 8.0
# A photographed pose is stronger evidence than a command acknowledgement.  It
# therefore needs multiple fresh controller generations, not a sleep followed
# by a read of whatever happened to be cached.
ARRIVAL_SAMPLE_COUNT = 3
ARRIVAL_STABLE_SECONDS = 0.25
ARRIVAL_TIMEOUT_DEFAULT_MS = 10_000
ARRIVAL_TIMEOUT_MIN_MS = 500
ARRIVAL_TIMEOUT_MAX_MS = 12_000
OBSERVATION_RESULT_CACHE_LIMIT = 32


class CalibrateRequest(StrictContract):
    """Every field optional: the UI sets one number at a time.

    Limits can arrive as raw encoder counts or as degrees at the joint; the
    conversion lives here rather than in the browser so there is exactly one
    place that knows how ticks become angles.
    """

    # Native Base zero is a signed counted coordinate; ordinary joints are
    # narrowed back to one turn by `calibrate()`. Limits declare where the joint may go
    # and are allowed to name more travel than one motor turn spans — see
    # `raw_bounds`.
    rawZero: Annotated[
        int, Field(strict=True, ge=-MULTI_TURN_GOAL_LIMIT, le=MULTI_TURN_GOAL_LIMIT)
    ] | None = None
    rawMin: Annotated[int, Field(strict=True, ge=-64 * TICKS_PER_TURN, le=64 * TICKS_PER_TURN)] | None = None
    rawMax: Annotated[int, Field(strict=True, ge=-64 * TICKS_PER_TURN, le=64 * TICKS_PER_TURN)] | None = None
    minDegrees: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    maxDegrees: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    ratio: Annotated[float, Field(gt=0.01, le=64)] | None = None
    direction: Literal[-1, 1] | None = None
    speed: Annotated[int, Field(strict=True, ge=1, le=MOTION_POLICY["MAX_SPEED"])] | None = None
    accel: Annotated[int, Field(strict=True, ge=1, le=MOTION_POLICY["MAX_ACCEL"])] | None = None
    servoId: Annotated[int, Field(strict=True, ge=0, le=253)] | None = None
    # Put the zero exactly halfway between the two limits, so the joint has the
    # same travel either side of it and the reported angle is symmetric.
    zeroFromLimits: bool | None = None
    clear: list[Literal["rawZero", "rawMin", "rawMax"]] | None = None
    # "Here" resolved on the Pi, from a reading taken as the request lands.
    # The browser's copy of rawPosition is a poll plus a refresh old — up to
    # half a second — and it used to send that number. On a geared arm joint
    # moved slowly that is invisible; on the camera servo, which is small and
    # spins freely under a hand, half a second is tens of degrees, so pressing
    # "here" stored somewhere the joint had already left.
    here: list[Literal["rawZero", "rawMin", "rawMax"]] | None = None


class TargetRequest(StrictContract):
    degrees: float


class MultiTargetRequest(StrictContract):
    """Whole-arm goals in one request.

    Three separate calls meant three HTTP round trips and three HOLD_SET checks
    per frame of a drag, and the joints started at visibly different times. One
    request is one round trip and one authority check for the whole pose.
    """

    joint_1: float | None = None
    joint_2: float | None = None
    joint_3: float | None = None
    joint_4: float | None = None


class LiveFollowJointSettings(StrictContract):
    speed: Annotated[
        int, Field(strict=True, ge=1, le=MOTION_POLICY["MAX_SPEED"])
    ]
    accel: Annotated[
        int, Field(strict=True, ge=1, le=MOTION_POLICY["MAX_ACCEL"])
    ]
    maxDeltaDegrees: Annotated[
        int,
        Field(
            strict=True,
            ge=1,
            le=LIVE_FOLLOW_MAX_DELTA_CEILING_DEGREES,
        ),
    ] = LIVE_FOLLOW_DEFAULT_MAX_DELTA_DEGREES


class LiveFollowStartRequest(StrictContract):
    joint_2: LiveFollowJointSettings
    joint_3: LiveFollowJointSettings
    startAttemptId: Annotated[str, Field(pattern=_LIVE_FOLLOW_ATTEMPT_ID_PATTERN)]
    startTimeoutMs: Annotated[
        int, Field(strict=True, ge=1, le=LIVE_FOLLOW_START_TIMEOUT_MS)
    ] = LIVE_FOLLOW_START_TIMEOUT_MS


class LiveFollowStartCancelRequest(StrictContract):
    startAttemptId: Annotated[str, Field(pattern=_LIVE_FOLLOW_ATTEMPT_ID_PATTERN)]


class LiveFollowFrameRequest(StrictContract):
    sessionId: Annotated[str, Field(pattern=_LIVE_FOLLOW_ID_PATTERN)]
    sequence: Annotated[int, Field(strict=True, ge=1)]
    joint_2: Annotated[float, Field(ge=-3600, le=3600)]
    joint_3: Annotated[float, Field(ge=-3600, le=3600)]


class LiveFollowHeartbeatRequest(StrictContract):
    sessionId: Annotated[str, Field(pattern=_LIVE_FOLLOW_ID_PATTERN)]


class LiveFollowEndRequest(StrictContract):
    sessionId: Annotated[str, Field(pattern=_LIVE_FOLLOW_ID_PATTERN)]
    # Explicit orderly End flushes the final target. Pointer-cancel, blur,
    # visibility loss, and unmount are fail-safe cancellation: the Pi must
    # discard any target it accepted but has not dispatched yet.
    flushPending: Annotated[bool, Field(strict=True)]


class PlanTargets(StrictContract):
    """Servo-degree targets reviewed as one coordinated pose."""

    joint_1: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_2: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_3: Annotated[float, Field(ge=-3600, le=3600)] | None = None
    joint_4: Annotated[float, Field(ge=-3600, le=3600)] | None = None

    def supplied(self) -> dict[str, float]:
        return {
            name: float(value)
            for name in JOINT_IDS
            if (value := getattr(self, name)) is not None
        }


class PlanTip(StrictContract):
    """Side-view tip point in the same floor frame as ``arm_scene``."""

    radialMm: Annotated[float, Field(ge=-450, le=450)]
    heightMm: Annotated[float, Field(ge=-400, le=600)]


class PlanPreviewRequest(StrictContract):
    targets: PlanTargets
    tip: PlanTip | None = None
    elbowPreference: Literal["nearest", "up", "down"] = "nearest"

    @model_validator(mode="after")
    def one_pose_source(self) -> "PlanPreviewRequest":
        supplied = self.targets.supplied()
        if self.tip is None and not supplied:
            raise ValueError("name at least one joint target or provide a tip")
        if self.tip is not None and any(name in supplied for name in FLOOR_JOINTS):
            raise ValueError(
                "tip inverse kinematics owns joint_2 and joint_3; only joint_1 and joint_4 may accompany it"
            )
        return self


class PlanExecuteRequest(StrictContract):
    planId: Annotated[str, Field(pattern=_PLAN_ID_PATTERN)]
    planDigest: Annotated[str, Field(pattern=_PLAN_DIGEST_PATTERN)]


class PlanExecuteCaptureRequest(PlanExecuteRequest):
    """Execute one reviewed plan and publish a frame only after proved arrival."""

    arrivalTimeoutMs: Annotated[
        int,
        Field(
            strict=True,
            ge=ARRIVAL_TIMEOUT_MIN_MS,
            le=ARRIVAL_TIMEOUT_MAX_MS,
        ),
    ] = ARRIVAL_TIMEOUT_DEFAULT_MS
    captureProfile: Literal["survey", "detail"] = CAPTURE_PROFILE_DETAIL


class SequenceWaypoint(PlanPreviewRequest):
    """One reviewed pose in a bounded Pi-local physical sequence."""

    label: Annotated[
        str,
        Field(
            strict=True,
            min_length=1,
            max_length=40,
            pattern=_SEQUENCE_LABEL_PATTERN,
        ),
    ] | None = None


class SequencePreviewRequest(StrictContract):
    waypoints: Annotated[
        list[SequenceWaypoint],
        Field(min_length=SEQUENCE_MIN_WAYPOINTS, max_length=SEQUENCE_MAX_WAYPOINTS),
    ]


class SequenceExecuteRequest(StrictContract):
    sequenceId: Annotated[str, Field(pattern=_SEQUENCE_ID_PATTERN)]
    sequenceDigest: Annotated[str, Field(pattern=_PLAN_DIGEST_PATTERN)]
    arrivalTimeoutMs: Annotated[
        int,
        Field(
            strict=True,
            ge=ARRIVAL_TIMEOUT_MIN_MS,
            le=ARRIVAL_TIMEOUT_MAX_MS,
        ),
    ] = ARRIVAL_TIMEOUT_DEFAULT_MS


class SequenceExecuteCaptureRequest(SequenceExecuteRequest):
    """Execute a reviewed route and bind one requested final camera profile."""

    captureProfile: Literal["survey", "detail"] = CAPTURE_PROFILE_DETAIL


class AssignIdRequest(StrictContract):
    """Burn a new ID into a servo's EEPROM.

    The controller refuses this unless exactly one servo answers the bus, which
    is the only thing that makes it safe: IDs are written by addressing the old
    one, so a second servo already holding the new ID would be shadowed and a
    mis-typed old ID would hit whatever does hold it. A factory-fresh Waveshare
    servo is ID 1, so plugging one into a populated bus collides immediately —
    set its ID on its own first.
    """

    oldId: Annotated[int, Field(strict=True, ge=0, le=253)]
    newId: Annotated[int, Field(strict=True, ge=0, le=253)]


class RegisterRequest(StrictContract):
    """Read-only raw register access, for the bench.

    Deliberately not wired into the panel. This exists so a servo's own memory
    table can be inspected without reflashing the controller. Writes are
    refused because mode, limits and goal registers can all bypass the guarded
    motion state machine even when torque itself is protected.
    """

    servoId: Annotated[int, Field(strict=True, ge=0, le=253)]
    address: Annotated[int, Field(strict=True, ge=0, le=255)]
    length: Annotated[int, Field(strict=True, ge=1, le=16)] | None = None
    values: list[Annotated[int, Field(strict=True, ge=0, le=255)]] | None = None


class TorqueRequest(StrictContract):
    """The servos that should be energised. Everything else is released.

    A whole set rather than a per-joint toggle: freeing one joint to position it
    by hand must not drop the two holding the arm up.
    """

    hold: Annotated[list[Annotated[int, Field(strict=True, ge=0, le=253)]], Field(max_length=len(JOINT_IDS))]


class FloorGuardRequest(StrictContract):
    """Whether the keep-out plane is enforced.

    Separate from calibration on purpose: calibration is how far each joint can
    travel, this is a region of space the whole arm is kept out of.
    """

    enabled: Annotated[bool, Field(strict=True)]


# What the controller says about bus health, in words the panel can print.
# `faulted` is intentionally generic: the controller uses it for ordinary
# telemetry/read/write failures as well as corrupt replies. A duplicate ID is
# only a separate, scan-backed `collisionSuspected` result.
# Keyed on the controller's OWN bus vocabulary. `physical_arm_api` translates
# "faulted" -> "degraded" for its status shape; keying on the translated word
# meant this never fired against the real thing, which reports "faulted".
BUS_TROUBLE = {
    "faulted": (
        "Servo-bus communication failed, so live joint telemetry is unavailable. "
        "Support the arm. Run Scan for a fresh diagnosis. If Scan does not "
        "report an ID collision, check the 12 V servo rail and bus cabling. Do not "
        "change servo IDs without scan-backed collision evidence."
    ),
    "offline": (
        "The servo bus is not responding at all. Check the 12 V rail and that the "
        "servo cables are seated — silence means nothing is answering, which is a "
        "different fault from an id clash."
    ),
}


def _collision_of(last_scan: object) -> dict[str, object]:
    if not isinstance(last_scan, dict) or not last_scan.get("collisionSuspected"):
        return {"collisionSuspected": False, "collisionId": None}
    identifier = last_scan.get("collisionId")
    return {
        "collisionSuspected": True,
        "collisionId": identifier if isinstance(identifier, int) and identifier > 0 else None,
    }


def _safe_last_scan(last_scan: object) -> dict[str, object] | None:
    """Whitelist bounded servo-presence evidence from the controller cache.

    A Scan PING is useful even when the immediately following aggregate
    telemetry sample omits that servo. Keep that distinction available to
    normal state polls without forwarding arbitrary controller payload fields
    or turning presence into position evidence.
    """

    if not isinstance(last_scan, dict):
        return None
    minimum_id = last_scan.get("minId")
    maximum_id = last_scan.get("maxId")
    if (
        isinstance(minimum_id, bool)
        or not isinstance(minimum_id, int)
        or isinstance(maximum_id, bool)
        or not isinstance(maximum_id, int)
        or not 0 <= minimum_id <= maximum_id <= 253
    ):
        return None

    def ids_of(field: str, *, required: bool = False) -> list[int] | None:
        value = last_scan.get(field)
        if value is None and not required:
            return None
        if (
            not isinstance(value, list)
            or any(
                isinstance(identifier, bool)
                or not isinstance(identifier, int)
                or not minimum_id <= identifier <= maximum_id
                for identifier in value
            )
            or len(set(value)) != len(value)
        ):
            raise ValueError(field)
        return sorted(value)

    try:
        found_ids = ids_of("foundIds", required=True)
        ping_found_ids = ids_of("pingFoundIds")
        unavailable_ids = ids_of("telemetryUnavailableIds")
        recovered_ids = ids_of("telemetryRecoveredIds")
    except ValueError:
        return None
    assert found_ids is not None
    found = set(found_ids)
    ping = set(ping_found_ids or [])
    unavailable = set(unavailable_ids or [])
    recovered = set(recovered_ids or [])
    if ping_found_ids is not None and not ping <= found:
        return None
    if unavailable_ids is not None and not unavailable <= ping:
        return None
    # A Scan recovery can restore fresh telemetry for a servo PING already
    # proved present. Recovered therefore means "missing from the first guarded
    # STATUS, present in the final one", not "absent from PING". It must be in
    # the reported union and cannot simultaneously remain unavailable.
    if recovered_ids is not None and (
        not recovered <= found or recovered & unavailable
    ):
        return None

    evidence: dict[str, object] = {
        "minId": minimum_id,
        "maxId": maximum_id,
        "foundIds": found_ids,
        "collisionSuspected": last_scan.get("collisionSuspected") is True,
        "collisionId": None,
    }
    for field, identifiers in (
        ("pingFoundIds", ping_found_ids),
        ("telemetryUnavailableIds", unavailable_ids),
        ("telemetryRecoveredIds", recovered_ids),
    ):
        if identifiers is not None:
            evidence[field] = identifiers
    collision_id = last_scan.get("collisionId")
    if (
        evidence["collisionSuspected"] is True
        and isinstance(collision_id, int)
        and not isinstance(collision_id, bool)
        and minimum_id <= collision_id <= maximum_id
    ):
        evidence["collisionId"] = collision_id
    return evidence


def _safe_motion_failure(payload: object) -> dict[str, object]:
    """Whitelist only bounded MOVE_SET failure evidence safe for API callers."""

    if not isinstance(payload, dict):
        return {}
    evidence: dict[str, object] = {}
    phase = payload.get("phase")
    if (
        isinstance(phase, str)
        and len(phase) <= 64
        and re.fullmatch(r"[a-z][a-z0-9_]*", phase) is not None
    ):
        evidence["phase"] = phase
    for field in ("motionMayHaveStarted", "partialDispatchPossible", "stopped"):
        value = payload.get(field)
        if isinstance(value, bool):
            evidence[field] = value
    torque_state = payload.get("torqueState")
    if torque_state in {"off", "on", "unknown"}:
        evidence["torqueState"] = torque_state
    for field, maximum in (("dispatchedFamilyCount", 2), ("failedIndex", 3)):
        value = payload.get(field)
        if (
            isinstance(value, int)
            and not isinstance(value, bool)
            and 0 <= value <= maximum
        ):
            evidence[field] = value
    return evidence


def _safe_live_follow_fault(error: Exception) -> dict[str, object]:
    """Reduce a terminal Fast Follow error to bounded public evidence."""

    if isinstance(error, ControllerCommandError):
        code = error.code
        if (
            not isinstance(code, str)
            or len(code) > 64
            or re.fullmatch(r"[A-Z][A-Z0-9_]*", code) is None
        ):
            code = "CONTROLLER_COMMAND_FAILED"
        return {"code": code, **_safe_motion_failure(error.payload)}
    if isinstance(error, ControllerTransportError):
        return {"code": "CONTROLLER_LINK_UNHEALTHY"}
    if isinstance(error, ControllerUnavailableError):
        return {"code": "CONTROLLER_UNAVAILABLE"}
    if isinstance(error, ControllerProtocolError):
        return {"code": "CONTROLLER_LINK_UNHEALTHY"}
    return {"code": "FAST_FOLLOW_FAILED"}


class JointState:
    __slots__ = (
        "servoId", "rawZero", "rawMin", "rawMax", "ratio", "direction", "speed", "accel",
        "steps", "spanDegrees", "multiTurn",
    )

    def __init__(self, servo_id: int, *, scs: bool = False, multi_turn: bool = False) -> None:
        self.servoId = servo_id
        self.multiTurn = multi_turn
        # Encoder geometry, not calibration: a property of which servo model this
        # joint carries, so it is fixed at construction and never loaded from the
        # store. Nothing the operator types can make an SC09 have 4096 counts.
        self.steps = SCS_TICKS_PER_TURN if scs else TICKS_PER_TURN
        self.spanDegrees = SCS_SPAN_DEGREES if scs else STS_SPAN_DEGREES
        self.rawZero: int | None = None
        self.rawMin: int | None = None
        self.rawMax: int | None = None
        self.ratio: float = 1.0
        # Explicit rather than inferred from which end was called max: the
        # operator asked to flip it by hand, and inferring it made typing a limit
        # in degrees circular (the sign depended on the value being set).
        self.direction: int = 1
        self.speed: int = DEFAULT_SPEED
        self.accel: int = DEFAULT_ACCEL

    def as_dict(self) -> dict[str, object]:
        return {
            "servoId": self.servoId,
            "rawZero": self.rawZero,
            "rawMin": self.rawMin,
            "rawMax": self.rawMax,
            "ratio": self.ratio,
            "direction": self.direction,
            "speed": self.speed,
            "accel": self.accel,
        }

    def load(self, value: object) -> None:
        if not isinstance(value, dict):
            return
        for key in ("servoId", "rawZero", "rawMin", "rawMax"):
            candidate = value.get(key)
            if isinstance(candidate, int) and not isinstance(candidate, bool):
                setattr(self, key, candidate)
        ratio = value.get("ratio")
        if isinstance(ratio, (int, float)) and not isinstance(ratio, bool) and 0.01 < ratio <= 64:
            self.ratio = float(ratio)
        if value.get("direction") in (-1, 1):
            self.direction = int(value["direction"])  # type: ignore[arg-type]
        speed = value.get("speed")
        if isinstance(speed, int) and not isinstance(speed, bool) and 1 <= speed <= MOTION_POLICY["MAX_SPEED"]:
            self.speed = speed
        accel = value.get("accel")
        if isinstance(accel, int) and not isinstance(accel, bool) and 1 <= accel <= MOTION_POLICY["MAX_ACCEL"]:
            self.accel = accel

    @property
    def raw_max(self) -> int:
        """The largest goal this joint can name.

        A multi-turn joint is not bounded by its encoder: the servo accepts
        +/-30719 and the odometer counts the wraps, so clamping to one turn here
        is what used to cap the base at 360/ratio degrees.
        """

        return MULTI_TURN_GOAL_LIMIT if self.multiTurn else self.steps - 1

    @property
    def raw_min(self) -> int:
        return -MULTI_TURN_GOAL_LIMIT if self.multiTurn else 0

    @property
    def ticks_per_degree(self) -> float:
        return self.steps * self.ratio / self.spanDegrees

    def raw_for(self, degrees: float) -> int | None:
        """Degrees at the joint -> an encoder count, unclamped."""

        if self.rawZero is None:
            return None
        return round(self.rawZero + degrees * self.ticks_per_degree * self.direction)

    def degrees_at(self, raw: int) -> float | None:
        if self.rawZero is None:
            return None
        return (raw - self.rawZero) * self.direction / self.ticks_per_degree

    def raw_bounds(self) -> tuple[int, int]:
        """The raw window the operator declared, in encoder counts.

        Deliberately *not* intersected with 0..4095. A geared joint is allowed to
        declare more travel than one motor turn spans; whether the gearing
        actually delivers it is a question about the arm, not about this number,
        and the operator is the one who can see the arm. `goal_for` is where the
        single-turn goal is enforced, because that is a property of the wire.
        """

        edges = [value for value in (self.rawMin, self.rawMax) if value is not None]
        if not edges:
            return self.raw_min, self.raw_max
        return min(edges), max(edges)

    def degree_bounds(self) -> tuple[float | None, float | None]:
        if self.rawZero is None:
            return None, None
        low, high = self.raw_bounds()
        edges = sorted((self.degrees_at(low) or 0.0, self.degrees_at(high) or 0.0))
        return edges[0], edges[1]

    def reach_bounds(self) -> tuple[float | None, float | None]:
        """Where the servo can actually be sent, as opposed to where it may go.

        `degree_bounds` is the operator's declaration; this is that window
        intersected with the one motor turn a single-turn goal can name. They
        differ whenever the gearing needs more than one turn to cover the
        declared travel, and when they differ the difference is the whole reason
        a joint stops short of a number that was accepted without complaint.
        """

        if self.rawZero is None:
            return None, None
        low, high = self.raw_bounds()
        edges = sorted((
            self.degrees_at(max(self.raw_min, min(self.raw_max, low))) or 0.0,
            self.degrees_at(max(self.raw_min, min(self.raw_max, high))) or 0.0,
        ))
        return edges[0], edges[1]

    def goal_for(self, degrees: float) -> int:
        """Degrees at the joint -> a raw goal that is always inside the limits."""

        assert self.rawZero is not None
        low, high = self.raw_bounds()
        raw = round(self.rawZero + degrees * self.ticks_per_degree * self.direction)
        # Declared limits first, then the wire. Native multi-turn uses the signed
        # extended range; ordinary joints remain inside one encoder turn.
        return max(self.raw_min, min(self.raw_max, max(low, min(high, raw))))


class JointStore:
    """Four numbers per joint in one JSON file. A corrupt file starts over."""

    _FILENAME = "arm-joints.json"
    _SAFETY_LATCH_FILENAME = "arm-clear-required.json"

    def __init__(self, state_dir: str | os.PathLike[str] | None = None) -> None:
        self._lock = threading.RLock()
        self._joints = {
            name: JointState(
                DEFAULT_SERVO_ID[name],
                scs=name in SCS_JOINTS,
                multi_turn=name in MULTI_TURN_JOINTS,
            )
            for name in JOINT_IDS
        }
        self._state_dir = Path(state_dir) if state_dir is not None else None
        self._path = self._state_dir / self._FILENAME if self._state_dir is not None else None
        self._safety_latch_path = (
            self._state_dir / self._SAFETY_LATCH_FILENAME
            if self._state_dir is not None
            else None
        )
        if self._path is not None and self._path.exists():
            try:
                document = json.loads(self._path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError):
                document = None
            if isinstance(document, dict):
                for name in JOINT_IDS:
                    self._joints[name].load(document.get(name))

    def snapshot(self) -> dict[str, JointState]:
        with self._lock:
            return dict(self._joints)

    def get(self, name: str) -> JointState:
        with self._lock:
            return self._joints[name]

    def save(self) -> None:
        if self._path is None or self._state_dir is None:
            return
        with self._lock:
            payload = {name: joint.as_dict() for name, joint in self._joints.items()}
        encoded = (json.dumps(payload, indent=1, sort_keys=True) + "\n").encode("utf-8")
        try:
            self._state_dir.mkdir(parents=True, exist_ok=True)
            descriptor, temporary = tempfile.mkstemp(prefix=".arm-joints-", dir=self._state_dir)
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(encoded)
            os.replace(temporary, self._path)
        except OSError:
            # Losing the file costs a recalibration, not safety. Never take the
            # arm offline because a disk write failed.
            pass

    def safety_latch_reason(self) -> str | None:
        """Read the fail-closed Pi latch left by an unconfirmed final STOP."""

        path = self._safety_latch_path
        if path is None or not path.exists():
            return None
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            # Presence with unreadable contents is still evidence that a prior
            # process could not prove its final STOP.
            return "STOP_DELIVERY_UNKNOWN"
        if not isinstance(document, dict) or document.get("clearRequired") is not True:
            return "STOP_DELIVERY_UNKNOWN"
        reason = document.get("reason")
        return (
            reason
            if reason in PERSISTED_CLEAR_REQUIRED_REASONS
            else "STOP_DELIVERY_UNKNOWN"
        )

    def persist_safety_latch(self, reason: str) -> None:
        """Atomically make a host-only clear requirement survive Pi restart."""

        if self._safety_latch_path is None or self._state_dir is None:
            raise OSError("arm_state_dir is required for a durable safety latch")
        if reason not in PERSISTED_CLEAR_REQUIRED_REASONS:
            reason = "STOP_DELIVERY_UNKNOWN"
        payload = {
            "version": 1,
            "clearRequired": True,
            "reason": reason,
        }
        encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
        self._state_dir.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=".arm-clear-required-", dir=self._state_dir
        )
        try:
            with os.fdopen(descriptor, "wb") as handle:
                descriptor = -1
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self._safety_latch_path)
            if os.name == "posix":
                directory = os.open(self._state_dir, os.O_RDONLY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            try:
                Path(temporary).unlink(missing_ok=True)
            except OSError:
                pass

    def clear_safety_latch(self) -> None:
        """Remove the marker only after the explicit STOP -> RESET proof."""

        if self._safety_latch_path is None:
            raise OSError("arm_state_dir is required for a durable safety latch")
        self._safety_latch_path.unlink(missing_ok=True)
        if os.name == "posix" and self._state_dir is not None:
            directory = os.open(self._state_dir, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)


class ArmService:
    def __init__(
        self,
        controller: ArmController,
        store: JointStore,
        *,
        operation_lock: threading.RLock | None = None,
    ) -> None:
        self._controller = controller
        self._store = store
        self._lock = threading.RLock()
        # Request/task ownership must not rely on RLock: asyncio tasks sharing
        # one event-loop thread are distinct operations even though RLock sees
        # the same thread owner. Both HTTP routers share this admission gate.
        self._operation_admission_lock = threading.Lock()
        # Serializes durable marker commit/removal against a concurrent STOP.
        # It is deliberately separate from controller I/O and the telemetry
        # condition so the physical STOP call itself is never queued behind a
        # long operation.
        self._safety_latch_lock = threading.Lock()
        # A composed physical observation owns the user-operation lane from
        # reviewed execution through arrival proof and capture.  Ordinary
        # mutations take this lock; STOP deliberately does not.  The background
        # chase loop also deliberately does not, because it may be required to
        # finish the very move the observation is waiting for.
        self._operation_lock = operation_lock or threading.RLock()
        # Linearizes STOP against the decision to begin a camera shutter. STOP
        # never waits for this gate: if capture already owns it, STOP still goes
        # straight to the controller and the resulting frame is invalidated.
        self._capture_gate = threading.Lock()
        # Serialises the read/revalidate/send boundary for prepared and direct
        # motion. A direct target that lands between plan validation and execute
        # would otherwise make "apply this exact preview" untrue.
        self._motion_lock = threading.RLock()
        # Serialises background HOLD_SET renewal against every RESET boundary.
        # STOP intentionally bypasses this gate; a generation check after RESET
        # makes any overlapping STOP the final physical controller operation.
        self._authority_lock = threading.RLock()
        self._plan_lock = threading.RLock()
        self._plans: dict[str, dict[str, object]] = {}
        self._sequences: dict[str, dict[str, object]] = {}
        self._sequence_results: OrderedDict[
            tuple[str, str], dict[str, object]
        ] = OrderedDict()
        # Serialise the Base's mode/frame transactions separately from ordinary
        # state bookkeeping. A target, re-home, and background refresh must not
        # interleave OFF -> ODO -> ON across separate HTTP/service threads.
        self._multi_turn_lock = threading.RLock()
        self._held: list[int] = []
        self._last_hold = 0.0
        # Give controller.start()/the HTTP lifespan one refresh interval to
        # settle. An immediate background STATUS made the first cache-only
        # state reads race serial I/O at process startup.
        self._last_status = time.monotonic()
        self._last_move = 0.0
        self._status_ms = 0
        self._telemetry_generation = 0
        self._telemetry_observed_at = 0.0
        self._stop_generation = 0
        self._telemetry_condition = threading.Condition(self._lock)
        self._observation_results: OrderedDict[
            tuple[str, str], dict[str, object]
        ] = OrderedDict()
        # Pi-local mirrors are reserved for explicit/uncertain operator stops.
        # Automatic controller faults remain controller-reported recovery state.
        durable_safety_reason = self._store.safety_latch_reason()
        self._operator_stopped = durable_safety_reason is not None
        self._inspection_required = durable_safety_reason is not None
        self._local_safety_stop_reason = durable_safety_reason
        self._last_controller_reconnect = 0.0
        self._last_stop_recovery = 0.0
        self._stop_service = threading.Event()
        # joint name -> (raw goal, re-sends left, give up after). The last thing
        # each joint was told to do, kept because a MOVE the controller accepted
        # can still be abandoned mid-travel: any lapse of torque authority
        # cancels the controller's active goal and makes the next hold capture pin
        # the joint wherever it had got to. `targets()` has returned 200 by then,
        # so without this nobody ever finishes the move.
        self._goals: dict[str, tuple[int, int, float]] = {}
        # A live-follow session is one Pi-owned latest-value stream for Shoulder
        # and Elbow only. It is intentionally not represented in `_goals`:
        # ordinary goals are chased after a missed arrival, while a follow frame
        # is obsolete the instant a newer pointer frame arrives.
        self._live_follow: dict[str, object] | None = None
        self._last_live_follow: dict[str, object] | None = None
        # Start attempt IDs are process-lifetime one-shot tokens. The gateway is
        # authenticated and starts are human-rate, so retaining these small
        # records is preferable to ever letting a delayed/retried old request
        # create a second physical session.
        self._live_follow_start_attempts: dict[str, str] = {}
        # servoId -> wrap-counted position, refreshed from the controller's
        # odometer. The servo's own feedback wraps at 4095, so for a geared
        # multi-turn joint this is the only honest answer to "where is it".
        self._multi_turn_raw: dict[int, int] = {}
        self._multi_turn_armed: set[int] = set()
        self._multi_turn_boot_id: object | None = None
        # Diagnostic proof from the last ODO result the service loop already
        # fetched. The browser's state route reads this cache only; it must never
        # turn a dashboard poll into another serial transaction.
        self._multi_turn_truth: dict[int, tuple[dict[str, object], float]] = {}
        self._families_dirty = True
        # On by default: the arm can reach 340 mm below its own pivot, so the
        # safe state is the guarded one. The operator can switch it off to drive
        # past the plane deliberately.
        self._floor_guard = True
        self._declare_servo_families()
        self._service = threading.Thread(target=self._service_loop, daemon=True)
        self._service.start()

    @property
    def operation_lock(self) -> threading.RLock:
        """The shared user-operation lane used by every physical API router."""

        return self._operation_lock

    @property
    def operation_admission_lock(self) -> threading.Lock:
        """The non-reentrant HTTP operation gate shared by both arm routers."""

        return self._operation_admission_lock

    @property
    def live_follow_active(self) -> bool:
        with self._lock:
            return self._live_follow is not None

    def evidence_status(self) -> dict[str, int]:
        """Return the bounded four-joint telemetry revision used by captures."""

        with self._lock:
            return {"stateRevision": self._telemetry_generation}

    def require_live_follow_idle(self) -> None:
        """Keep every unrelated mutation out of an active follow session."""

        if self.live_follow_active:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "Fast Follow owns Shoulder and Elbow. End it or press STOP "
                    "before another physical arm operation."
                ),
            )

    @contextmanager
    def commissioning_mutation(self):
        """Exclude every background/user writer while commissioning takes over.

        Commissioning calls the controller directly. Clearing SimpleArm's
        standing authority and goals after both background lanes are acquired
        prevents a queued renewal or chase from undoing the returned physical
        command. STOP deliberately bypasses this context and remains
        preemptive.
        """

        with self._commissioning_lane(allow_clear_required=False):
            yield

    @contextmanager
    def commissioning_clear_mutation(self):
        """Own the commissioning lanes while allowing explicit stop clear."""

        with self._commissioning_lane(allow_clear_required=True):
            yield

    @contextmanager
    def _commissioning_lane(self, *, allow_clear_required: bool):
        with self._operation_lock, self._motion_lock, self._authority_lock:
            self.require_live_follow_idle()
            reported = self._controller.transport_state()
            with self._lock:
                clear_required = (
                    self._operator_stopped
                    or self._inspection_required
                    or reported.get("motionState") == "stopped"
                    or self._reported_clear_required_stop(reported)
                )
                if clear_required and not allow_clear_required:
                    raise ControllerCommandError("STOPPED")
                self._held = []
                self._goals.clear()
                self._last_hold = time.monotonic()
            yield

    def _declare_servo_families(self, *, strict: bool = False) -> bool:
        """Push the joint -> servo family mapping down to the controller.

        Idempotent and cheap when nothing changed, so it is safe to call after
        anything that can move a joint onto a different servo id. The controller
        keeps its own copy and replays it after a reconnect. The complete
        native-multi-turn ID set is declared beside the family map so Scan can
        distinguish the configured Base from an ordinary STS joint after an
        older HAT boot loses only its RAM-side decoder classification.
        """

        with self._lock:
            self._families_dirty = True
        declare = getattr(self._controller, "declare_family", None)
        declare_native_multi_turn = getattr(
            self._controller, "declare_native_multi_turn_servos", None
        )
        if not callable(declare) or not callable(declare_native_multi_turn):
            if strict:
                raise RuntimeError(
                    "Controller cannot declare servo families and native "
                    "multi-turn IDs."
                )
            return False
        native_multi_turn_ids = sorted(
            {self._store.get(name).servoId for name in MULTI_TURN_JOINTS}
        )
        try:
            # Replacement, not accumulation: a Base ID change must not leave
            # the old ID eligible for the tightly gated Scan recovery path.
            declare_native_multi_turn(native_multi_turn_ids)
        except Exception:
            if strict:
                raise
            return False
        for name in JOINT_IDS:
            try:
                declare(
                    self._store.get(name).servoId,
                    "SCS" if name in SCS_JOINTS else "STS",
                )
            except Exception:
                # A controller that is not up yet gets the mapping on connect.
                if strict:
                    raise
                return False
        with self._lock:
            self._families_dirty = False
        return True

    def _sync_multi_turn_identity(self, reported: dict[str, object]) -> bool:
        """Bind every cached Base count to the exact live HAT boot."""

        identity = reported.get("identity")
        boot_id = identity.get("bootId") if isinstance(identity, dict) else None
        connection_online = reported.get("connection") == "online"
        if not connection_online:
            with self._lock:
                self._multi_turn_raw.clear()
                self._multi_turn_armed.clear()
                self._multi_turn_truth.clear()
                for name in MULTI_TURN_JOINTS:
                    self._goals.pop(name, None)
            return False
        if not isinstance(boot_id, str) or not boot_id:
            with self._lock:
                self._multi_turn_truth.clear()
                # Online without a stable identity is not a frame to which an
                # absolute multi-turn count can safely be attached.
                self._multi_turn_raw.clear()
                self._multi_turn_armed.clear()
                for name in MULTI_TURN_JOINTS:
                    self._goals.pop(name, None)
            return False
        with self._lock:
            if self._multi_turn_boot_id is None:
                self._multi_turn_boot_id = boot_id
            elif boot_id != self._multi_turn_boot_id:
                self._multi_turn_boot_id = boot_id
                self._multi_turn_armed.clear()
                self._multi_turn_raw.clear()
                self._multi_turn_truth.clear()
                for name in MULTI_TURN_JOINTS:
                    self._goals.pop(name, None)
        return connection_online

    @staticmethod
    def _multi_turn_truth_supported(reported: dict[str, object]) -> bool:
        identity = reported.get("identity")
        capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
        if capabilities is None:
            capabilities = reported.get("capabilities")
        return isinstance(capabilities, list) and MULTI_TURN_REQUIRED_CAPABILITIES.issubset(
            capabilities
        )

    @staticmethod
    def _reported_capabilities(reported: Mapping[str, object]) -> frozenset[str]:
        identity = reported.get("identity")
        capabilities = (
            identity.get("capabilities") if isinstance(identity, dict) else None
        )
        if capabilities is None:
            capabilities = reported.get("capabilities")
        if not isinstance(capabilities, list) or not all(
            isinstance(capability, str) for capability in capabilities
        ):
            return frozenset()
        return frozenset(capabilities)

    def _require_servo_family_ready(
        self, reported: Mapping[str, object]
    ) -> None:
        """Prove the HAT knows each servo dialect before SCS evidence/motion."""

        if "servo_family" not in self._reported_capabilities(reported):
            raise ControllerCommandError("SERVO_FAMILY_UNAVAILABLE")
        with self._lock:
            dirty = self._families_dirty
        if not dirty:
            return
        try:
            self._declare_servo_families(strict=True)
        except Exception:
            raise ControllerCommandError("SERVO_FAMILY_UNAVAILABLE") from None

    def _multi_turn_frame_current(self, reported: dict[str, object]) -> bool:
        """Return whether cached Base truth belongs to this live v3 frame."""

        current = self._sync_multi_turn_identity(reported)
        supported = self._multi_turn_truth_supported(reported)
        if not current or not supported:
            # A diagnostic from a previous boot, an offline transport, or an
            # older capability set is worse than no diagnostic: it looks live.
            with self._lock:
                self._multi_turn_truth.clear()
        return current and supported

    def _cache_multi_turn_truth(
        self, servo_id: int, state: dict[str, object]
    ) -> None:
        """Keep only bounded, display-safe fields from an existing ODO read."""

        truth: dict[str, object] = {}
        for field in (
            "tracking",
            "valid",
            "stepMode",
            "stepOutstanding",
            "countdownObserved",
            "resyncNeeded",
        ):
            value = state.get(field)
            if isinstance(value, bool):
                truth[field] = value
        resync_count = state.get("resyncCount")
        if (
            isinstance(resync_count, int)
            and not isinstance(resync_count, bool)
            and resync_count >= 0
        ):
            truth["resyncCount"] = resync_count
        sample_age = state.get("sampleAgeMs")
        if (
            isinstance(sample_age, int)
            and not isinstance(sample_age, bool)
            and sample_age >= 0
        ):
            truth["sampleAgeMs"] = sample_age
        with self._lock:
            self._multi_turn_truth[servo_id] = (truth, time.monotonic())

    def _multi_turn_truth_snapshot(self, servo_id: int) -> dict[str, object] | None:
        """Copy cached proof and age it without performing controller I/O."""

        with self._lock:
            cached = self._multi_turn_truth.get(servo_id)
            if cached is None:
                return None
            truth, cached_at = cached
            snapshot = dict(truth)
        sample_age = snapshot.get("sampleAgeMs")
        if isinstance(sample_age, int) and not isinstance(sample_age, bool):
            snapshot["sampleAgeMs"] = sample_age + max(
                0, int((time.monotonic() - cached_at) * 1000)
            )
        return snapshot

    def _controller_identity(self, reported: dict[str, object]) -> dict[str, object]:
        """Select the harmless identity fields the compact dashboard needs."""

        identity = reported.get("identity")
        source = identity if isinstance(identity, dict) else {}
        selected: dict[str, object] = {
            # Keep the old field for dashboard compatibility, but never present
            # native absolute control as the retired Mode-3 truth protocol.
            "multiTurnTruthV3": False,
            "multiTurnAbsoluteV1": self._multi_turn_truth_supported(reported),
            # This is deliberately one bounded feature bit rather than the
            # controller's raw capability list. The browser must not offer
            # Fast Follow unless both motion-with-feedback and compact
            # feedback-only reads are available end to end.
            "liveFollowV1": {
                LIVE_FOLLOW_CAPABILITY,
                LIVE_FOLLOW_READ_CAPABILITY,
            }.issubset(self._reported_capabilities(reported)),
        }
        for field in ("controllerId", "bootId", "firmwareVersion"):
            value = source.get(field)
            if isinstance(value, str) and value:
                selected[field] = value
        protocol_version = source.get("protocolVersion")
        if isinstance(protocol_version, int) and not isinstance(protocol_version, bool):
            selected["protocolVersion"] = protocol_version
        return selected

    def _arm_multi_turn(self) -> None:
        """Adopt an already-valid HAT multi-turn frame without redefining it.

        A Pi-process restart does not reboot the HAT, so its counted frame can
        remain valid and must be preserved. A HAT/servo continuity loss is
        different: the one-turn encoder behind the Base's 4:1 gearing leaves four
        physical joint angles with the same wrapped value. That state remains
        UNKNOWN until the operator puts the Base at its physical zero and uses
        "Set zero here", which calls `_home_multi_turn` explicitly.
        """

        read = getattr(self._controller, "odometer_read", None)
        if not callable(read):
            return
        reported = self._controller.transport_state()
        if not self._multi_turn_frame_current(reported):
            # No capability list, or a retired Mode-3 capability set, is treated
            # the same: never arm a counted Base on an unknown controller.
            return
        for name in MULTI_TURN_JOINTS:
            servo_id = self._store.get(name).servoId
            if servo_id in self._multi_turn_armed:
                continue
            try:
                state = read(servo_id)
            except Exception:
                continue
            self._cache_multi_turn_truth(servo_id, state)
            position = state.get("multiTurnPosition") if state.get("valid") else None
            if not isinstance(position, int) or isinstance(position, bool):
                # Wrapped encoder truth is not enough to pick a 90-degree Base
                # turn. Never make that choice autonomously.
                continue
            self._multi_turn_raw[servo_id] = position
            if state.get("stepMode") is False:
                self._multi_turn_armed.add(servo_id)
            else:
                self._multi_turn_armed.discard(servo_id)

    def _home_multi_turn(self, servo_id: int) -> int:
        """Create a new Base frame after an explicit physical-zero confirmation."""

        arm = getattr(self._controller, "set_multi_turn", None)
        home = getattr(self._controller, "home_multi_turn", None)
        if not callable(arm) or not callable(home):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="This controller cannot establish a trustworthy multi-turn Base frame.",
            )
        reported = self._controller.transport_state()
        if not self._sync_multi_turn_identity(reported):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The HAT must be online with a stable boot identity before Base homing.",
            )
        if not self._multi_turn_truth_supported(reported):
            with self._lock:
                self._multi_turn_truth.clear()
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The connected HAT firmware cannot prove multi-turn Base position.",
            )
        with self._lock:
            if servo_id in self._held:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Free the Base before setting its physical zero.",
                )
            self._goals.pop("joint_1", None)
            with self._multi_turn_lock:
                self._multi_turn_raw.pop(servo_id, None)
                self._multi_turn_armed.discard(servo_id)
                try:
                    # The controller keeps native Mode-0 multi-turn setup,
                    # physical-frame capture, and verification adjacent on UART.
                    state = home(servo_id)
                    self._cache_multi_turn_truth(servo_id, state)
                except Exception:
                    try:
                        arm(servo_id, False)
                    except Exception:
                        pass
                    raise
                position = (
                    state.get("multiTurnPosition")
                    if state.get("valid") and state.get("stepMode") is False
                    else None
                )
                if not isinstance(position, int) or isinstance(position, bool):
                    try:
                        arm(servo_id, False)
                    except Exception:
                        pass
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="The Base encoder frame could not be verified in free mode after homing.",
                    )
                self._multi_turn_raw[servo_id] = position
                self._multi_turn_armed.add(servo_id)
                return position

    def _prepare_multi_turn_drive(self, servo_id: int) -> None:
        """Verify native absolute multi-turn mode before the Base is energised."""

        read = getattr(self._controller, "odometer_read", None)
        if not callable(read):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="This controller cannot prepare trustworthy multi-turn drive.",
            )
        with self._multi_turn_lock:
            reported = self._controller.transport_state()
            identity = reported.get("identity")
            reported_boot_id = (
                identity.get("bootId") if isinstance(identity, dict) else None
            )
            with self._lock:
                previous_boot_id = self._multi_turn_boot_id
            if not self._sync_multi_turn_identity(reported):
                if reported.get("connection") != "online":
                    detail = (
                        "The controller is temporarily unavailable; Base drive was not "
                        "prepared. Wait for the same HAT boot to reconnect."
                    )
                elif not isinstance(reported_boot_id, str) or not reported_boot_id:
                    detail = (
                        "The online controller has no stable boot identity; Base drive "
                        "was not prepared."
                    )
                else:
                    detail = "The controller cannot prove its current Base identity."
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=detail,
                )
            if (
                isinstance(previous_boot_id, str)
                and previous_boot_id
                and isinstance(reported_boot_id, str)
                and reported_boot_id != previous_boot_id
            ):
                # `_sync_multi_turn_identity` already discarded every cached
                # count. Name the evidence accurately: unlike an offline read,
                # this is a proven new HAT frame and requires a fresh zero.
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "The HAT boot identity changed; put the Base at physical zero "
                        "and choose Set zero here before driving it."
                    ),
                )
            if not self._multi_turn_truth_supported(reported):
                with self._lock:
                    self._multi_turn_truth.clear()
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The connected HAT firmware cannot prove multi-turn Base position.",
                )
            state = read(servo_id)
            self._cache_multi_turn_truth(servo_id, state)
            position = state.get("multiTurnPosition") if state.get("valid") else None
            if (
                not isinstance(position, int)
                or isinstance(position, bool)
                or state.get("stepMode") is not False
            ):
                self._multi_turn_raw.pop(servo_id, None)
                self._multi_turn_armed.discard(servo_id)
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Put the Base at physical zero and choose Set zero here before driving it.",
                )
            self._multi_turn_raw[servo_id] = position
            self._multi_turn_armed.add(servo_id)

    def _refresh_multi_turn(self) -> bool:
        read = getattr(self._controller, "odometer_read", None)
        if not callable(read):
            return False
        reported = self._controller.transport_state()
        if not self._multi_turn_frame_current(reported):
            return False
        refreshed = False
        for name in MULTI_TURN_JOINTS:
            servo_id = self._store.get(name).servoId
            try:
                state = read(servo_id)
            except Exception:
                continue
            refreshed = True
            self._cache_multi_turn_truth(servo_id, state)
            if not state.get("valid"):
                # Never fall back to wrapping servo feedback. An invalid HAT
                # counted frame stays UNKNOWN until the operator confirms zero.
                self._multi_turn_raw.pop(servo_id, None)
                self._multi_turn_armed.discard(servo_id)
                continue
            position = state.get("multiTurnPosition")
            if isinstance(position, int) and not isinstance(position, bool):
                self._multi_turn_raw[servo_id] = position
                if state.get("stepMode") is False:
                    self._multi_turn_armed.add(servo_id)
                else:
                    self._multi_turn_armed.discard(servo_id)
        return refreshed

    @staticmethod
    def _reported_clear_required_stop(reported: Mapping[str, object]) -> bool:
        return (
            reported.get("operatorInspectionRequired") is True
            or reported.get("safetyStopReason")
            in CLEAR_REQUIRED_STOP_REASONS
        )

    def _automatic_stop_is_safely_resettable(
        self, reported: Mapping[str, object]
    ) -> bool:
        """Recognize only an automatic latch with complete all-off proof.

        Firmware 2.4 did not identify its automatic watchdog/supervision STOP,
        so its narrow reasonless signature is version-gated. Later firmware can
        name an automatic reason. An explicit STOP, an unknown reason, missing
        telemetry, or any live actuator/e-stop concern is never reset here.
        """

        identity = reported.get("identity")
        firmware = identity.get("firmwareVersion") if isinstance(identity, dict) else None
        reason = reported.get("safetyStopReason")
        reasoned_automatic = reason in AUTOMATIC_STOP_REASONS
        reasonless_legacy = (
            isinstance(firmware, str)
            and firmware.startswith("arm-hat-2.4.")
            and reason is None
            and reported.get("operatorInspectionRequired") is not True
        )
        if not (reasoned_automatic or reasonless_legacy):
            return False
        if (
            reported.get("connection") != "online"
            or reported.get("motionState") != "stopped"
            or reported.get("hardwareEstop") != "not_detected"
            or reported.get("torqueState") != "off"
            or reported.get("servos") != "online"
        ):
            return False
        count = reported.get("servoCount")
        rows = reported.get("servosTelemetry")
        if (
            isinstance(count, bool)
            or not isinstance(count, int)
            or count <= 0
            or not isinstance(rows, list)
            or len(rows) != count
        ):
            return False
        identifiers: set[int] = set()
        for row in rows:
            if not isinstance(row, dict):
                return False
            identifier = row.get("id")
            packet_age = row.get("packetAgeMs")
            if (
                isinstance(identifier, bool)
                or not isinstance(identifier, int)
                or identifier in identifiers
                or row.get("torqueState") != "off"
                or row.get("moving") is not False
                or row.get("operatingMode") != 0
                or row.get("errors") != []
                or isinstance(packet_age, bool)
                or not isinstance(packet_age, int)
                or not 0 <= packet_age <= AUTOMATIC_STOP_RECOVERY_MAX_PACKET_AGE_MS
            ):
                return False
            identifiers.add(identifier)
        return True

    def _recover_stop(self) -> bool:
        """Clear only a proved-safe automatic latch, never an operator STOP."""

        # Do not cut across a reviewed/user motion transaction. The next service
        # tick will retry after that transaction observes and reports the abort.
        if not self._motion_lock.acquire(blocking=False):
            return False
        try:
            with self._authority_lock:
                reported = self._controller.transport_state()
                now = time.monotonic()
                with self._telemetry_condition:
                    if self._operator_stopped or self._inspection_required:
                        return False
                    if (
                        now - self._last_stop_recovery
                        < AUTOMATIC_STOP_RECOVERY_INTERVAL_SECONDS
                    ):
                        return False
                    if not self._automatic_stop_is_safely_resettable(reported):
                        return False
                    stop_generation = self._stop_generation
                    self._last_stop_recovery = now
                    self._held = []
                    self._goals.clear()
                    self._last_hold = now
                    self._stop_generation += 1
                    recovery_generation = self._stop_generation
                    self._telemetry_condition.notify_all()
                try:
                    self._controller.reset(inspected=True)
                except Exception as error:
                    _LOG.warning("Automatic controller STOP recovery failed: %r", error)
                    return False
                with self._telemetry_condition:
                    # A concurrent explicit STOP increments the generation before
                    # touching the controller. Whichever command reaches the bus
                    # last therefore wins physically; never clear its Pi latch.
                    interrupted = (
                        self._operator_stopped
                        or self._inspection_required
                        or self._stop_generation != recovery_generation
                        or stop_generation + 1 != recovery_generation
                    )
                    if not interrupted:
                        self._last_status = 0.0
                if interrupted:
                    # RESET may have queued behind a newer explicit STOP and
                    # cleared it. Reassert STOP so the explicit user action is
                    # always the final physical state, just as in manual clear.
                    try:
                        self._controller.stop()
                    except Exception:
                        self._latch_stop_delivery_unknown()
                    return False
                return True
        finally:
            self._motion_lock.release()

    def _recover_controller(self) -> bool:
        """Retry a configured controller handshake when boot order left it offline.

        This performs HELLO/config/status only through the controller's existing
        bounded reconnect path. It never scans, takes torque, clears STOP, homes,
        or moves a joint.
        """

        reported = self._controller.transport_state()
        if reported.get("configured") is not True:
            return False
        connection = reported.get("connection")
        if connection == "online" or connection not in {"offline", "faulted", "closed"}:
            return False
        reconnect = getattr(self._controller, "reconnect", None)
        if not callable(reconnect):
            return False
        now = time.monotonic()
        with self._lock:
            if now - self._last_controller_reconnect < CONTROLLER_RECONNECT_INTERVAL_SECONDS:
                return False
            self._last_controller_reconnect = now
        try:
            reconnect()
        except Exception:
            return False
        # A successful reconnect may have a new HAT boot identity. Drop/adopt
        # multi-turn truth through the normal identity path on this same tick.
        self._declare_servo_families()
        self._last_status = 0.0
        return True

    def _service_loop(self) -> None:
        """The only periodic user of the serial port.

        Renewing the hold takes priority over refreshing telemetry: a late
        renewal de-energises the arm, a late reading is merely stale. Both yield
        to an active drag.
        """

        while not self._stop_service.wait(SERVICE_TICK_SECONDS):
            # An active follow session is continuity-bound. Do not hide a link
            # loss or automatic STOP by reconnecting/resetting underneath it;
            # the cached health check below ends and releases that session.
            if not self.live_follow_active:
                self._recover_controller()
                self._recover_stop()
            now = time.monotonic()
            with self._lock:
                last_move = self._last_move
                renewal_due = bool(self._held) and (
                    now - self._last_hold >= HOLD_RENEW_SECONDS
                )
            if renewal_due:
                # RESET and renewal share the authority gate. A reset therefore
                # lands after any in-flight renewal, or a later renewal sees the
                # cleared hold set and is discarded before HOLD_SET. This gate is
                # intentionally separate from the long-lived observation lane so
                # hold leases can still be renewed while arrival is being proved.
                with self._authority_lock:
                    reported = self._controller.transport_state()
                    with self._lock:
                        held = list(self._held)
                        stop_generation = self._stop_generation
                        renewal_current = (
                            bool(held)
                            and time.monotonic() - self._last_hold
                            >= HOLD_RENEW_SECONDS
                            and not self._operator_stopped
                            and not self._inspection_required
                            and not self._reported_clear_required_stop(reported)
                        )
                    if renewal_current:
                        # Revalidate the exact snapshot at the send boundary.
                        # STOP may still preempt after this check; the controller's
                        # STOP latch then refuses the queued HOLD_SET.
                        with self._lock:
                            renewal_current = (
                                stop_generation == self._stop_generation
                                and held == self._held
                                and not self._operator_stopped
                                and not self._inspection_required
                            )
                            if renewal_current:
                                self._last_hold = time.monotonic()
                    if renewal_current:
                        try:
                            self._apply_hold(held, isolate_multi_turn_failures=True)
                        except Exception as error:
                            # Torque drops on its own when the hold expires, which
                            # is the behaviour we want anyway; the next read reports
                            # it. The set itself is KEPT: it is the operator's
                            # standing request and is retried on the next tick.
                            _LOG.warning(
                                "Arm hold renewal failed for %s: %r", held, error
                            )
                continue
            # Fast Follow is the only motion path whose latest target is owned
            # by this loop. It runs after lease renewal, never in place of it;
            # command traffic refreshes the heartbeat but does not extend the
            # controller's independent torque lease.
            if self._dispatch_live_follow(time.monotonic()):
                continue
            if self.live_follow_active:
                # Compact FOLLOW_SET feedback is the live telemetry while this
                # session owns the link. A full ~175 ms STATUS here would add
                # latency and could consume most of the 400 ms input deadman.
                continue
            if now - self._last_status < REFRESH_SECONDS or now - last_move < MOVE_QUIET_SECONDS:
                continue
            self._last_status = now
            refreshed = False
            status_observed_at = 0.0
            try:
                # Refreshes the controller's own telemetry cache, which is what
                # `transport_state()` then hands back for free.
                self._controller.status()
                status_observed_at = time.monotonic()
                self._status_ms = int((status_observed_at - now) * 1000)
                refreshed = True
            except Exception:
                pass
            # A gateway may host the physical commissioning API before this
            # simple-arm profile has ever been calibrated. In that state the
            # Base odometer cannot produce a joint angle or support motion, so
            # periodic ODO traffic is pure contention. Direct commissioning
            # helpers remain available; background tracking begins with a zero.
            multi_turn_refreshed = False
            if self._store.get("joint_1").rawZero is not None:
                self._arm_multi_turn()
                multi_turn_refreshed = self._refresh_multi_turn()
            self._chase_goals()
            if multi_turn_refreshed:
                # ODO_READ and any legacy chase resend are guarded by a fresh
                # STATUS on both sides. On firmware 2.4 those exchanges can
                # exceed the 250 ms arrival freshness bound, so publish the
                # generation from the final guarded STATUS instead of the
                # earlier standalone STATUS timestamp.
                status_observed_at = time.monotonic()
            if refreshed:
                with self._telemetry_condition:
                    self._telemetry_generation += 1
                    self._telemetry_observed_at = status_observed_at
                    self._telemetry_condition.notify_all()

    def live_raw(self, servo_id: int) -> int | None:
        """Where the joint is, in counts, wraps included.

        For a multi-turn joint the servo's own rawPosition is only the position
        within the current turn, so it reads the same at 10 degrees and at 370.
        The odometer's count is the one that can be compared against a limit.
        """

        multi_turn_servo = any(
            self._store.get(name).servoId == servo_id for name in MULTI_TURN_JOINTS
        )
        reported = self._controller.transport_state()
        frame_current = self._multi_turn_frame_current(reported)
        counted = self._multi_turn_raw.get(servo_id)
        if counted is not None and frame_current:
            return counted
        if multi_turn_servo:
            if not frame_current:
                return None
            read = getattr(self._controller, "odometer_read", None)
            if not callable(read):
                return None
            try:
                state = read(servo_id)
            except Exception:
                return None
            self._cache_multi_turn_truth(servo_id, state)
            if not state.get("valid"):
                # A Pi-only restart may be able to adopt the HAT's existing
                # frame. A HAT restart cannot: `_arm_multi_turn` deliberately
                # leaves that 90-degree-ambiguous state unknown until the user
                # confirms physical zero.
                self._arm_multi_turn()
                try:
                    state = read(servo_id)
                except Exception:
                    return None
                self._cache_multi_turn_truth(servo_id, state)
            position = (
                state.get("multiTurnPosition")
                if state.get("valid")
                else None
            )
            if isinstance(position, int) and not isinstance(position, bool):
                self._multi_turn_raw[servo_id] = position
                return position
            return None
        servo = self.telemetry_of(reported).get(servo_id)
        raw = servo.get("rawPosition") if isinstance(servo, dict) else None
        return raw if isinstance(raw, int) and not isinstance(raw, bool) else None

    def telemetry_of(self, reported: dict[str, object]) -> dict[int, dict[str, object]]:
        rows = reported.get("servosTelemetry")
        if not isinstance(rows, list):
            return {}
        return {
            row["id"]: row
            for row in rows
            if isinstance(row, dict) and isinstance(row.get("id"), int)
        }

    def state(self) -> dict[str, object]:
        # A pure cache read. The service loop is what keeps it fresh, so the
        # browser can poll as fast as it likes without touching the serial port.
        reported = self._controller.transport_state()
        last_scan = _safe_last_scan(reported.get("lastScan"))
        frame_current = self._multi_turn_frame_current(reported)
        telemetry = self.telemetry_of(reported)
        joints = []
        for name in JOINT_IDS:
            joint = self._store.get(name)
            servo = telemetry.get(joint.servoId)
            raw = servo.get("rawPosition") if isinstance(servo, dict) else None
            counted = self._multi_turn_raw.get(joint.servoId) if frame_current else None
            position_trusted = not joint.multiTurn and isinstance(raw, int)
            if joint.multiTurn:
                raw = counted if counted is not None and servo is not None else None
                position_trusted = raw is not None
            minimum, maximum = joint.degree_bounds()
            reach_low, reach_high = joint.reach_bounds()
            low, high = joint.raw_bounds()
            rendered = {
                "id": name,
                "name": JOINT_NAMES[name],
                **joint.as_dict(),
                "online": servo is not None,
                "rawPosition": raw,
                "degrees": joint.degrees_at(raw) if isinstance(raw, int) else None,
                "positionTrusted": position_trusted,
                "minDegrees": minimum,
                "maxDegrees": maximum,
                # What the servo can be sent to, which is narrower than the
                # declared limits whenever the gearing outruns one motor turn.
                "reachMin": reach_low,
                "reachMax": reach_high,
                "rawLow": low,
                "rawHigh": high,
                "direction": joint.direction,
                "calibrated": joint.rawZero is not None,
                "torque": servo.get("torqueState") if isinstance(servo, dict) else "unknown",
                "moving": servo.get("moving") if isinstance(servo, dict) else None,
                "packetAgeMs": servo.get("packetAgeMs") if isinstance(servo, dict) else None,
                "temperatureC": servo.get("temperatureC") if isinstance(servo, dict) else None,
                "voltageVolts": servo.get("voltageVolts") if isinstance(servo, dict) else None,
            }
            if joint.multiTurn:
                rendered["multiTurnTruth"] = (
                    self._multi_turn_truth_snapshot(joint.servoId)
                    if frame_current
                    else None
                )
            joints.append(rendered)
        now = time.monotonic()
        with self._lock:
            held = list(self._held)
            telemetry_generation = self._telemetry_generation
            telemetry_observed_at = self._telemetry_observed_at
            local_inspection_required = self._inspection_required
            local_operator_stopped = self._operator_stopped
            local_safety_stop_reason = self._local_safety_stop_reason
        telemetry_age_ms = (
            max(0, int((now - telemetry_observed_at) * 1000))
            if telemetry_generation > 0 and telemetry_observed_at > 0
            else None
        )
        return {
            "connection": reported.get("connection"),
            "bus": reported.get("bus"),
            "controller": self._controller_identity(reported),
            "joints": joints,
            "held": held,
            "telemetryGeneration": telemetry_generation,
            "telemetryAgeMs": telemetry_age_ms,
            # `transport_state()` has no "stop" key — that one is assembled by the
            # commissioning status route. Reading it here meant a latched STOP
            # always reported as false, so every command failed with STOPPED
            # while the screen showed nothing wrong.
            "stopped": (
                local_operator_stopped
                or reported.get("motionState") == "stopped"
            ),
            "operatorInspectionRequired": (
                local_inspection_required
                or reported.get("operatorInspectionRequired") is True
            ),
            "safetyStopReason": (
                reported.get("safetyStopReason")
                if reported.get("safetyStopReason")
                in CLEAR_REQUIRED_STOP_REASONS
                else local_safety_stop_reason
                if local_safety_stop_reason is not None
                else "MOVE_SET_FAILED"
                if local_inspection_required
                else None
            ),
            "lastMotionFailure": (
                _safe_motion_failure(reported.get("lastMotionFailure")) or None
            ),
            # Persistent, bounded presence evidence. In particular, PING can
            # prove that a servo exists even when the post-Scan aggregate
            # telemetry sample cannot yet provide a trusted position.
            "lastScan": last_scan,
            # Carried from the last scan so the panel can say WHY a bus is empty.
            # An id clash is the one failure that looks identical to "nothing is
            # plugged in", and it is the one the operator can actually fix.
            **_collision_of(last_scan),
            # Set whenever the controller can describe the bus fault, even if no
            # scan ever completed. A collision refuses the scan at the
            # torque-off stage, so waiting for scan evidence would mean the one
            # case that most needs explaining never gets explained.
            "busTrouble": BUS_TROUBLE.get(str(reported.get("bus"))),
            # How long the last telemetry refresh took on the wire. Surfaced so a
            # saturated serial link is visible instead of being guessed at.
            "refreshMs": self._status_ms,
            # The keep-out plane and the geometry it is measured against, so the
            # panel draws and enforces the same barrier this service does rather
            # than keeping a second copy of the numbers that can drift out of
            # step. A drifted floor is a floor you land on.
            "floorGuard": {
                "enabled": self._floor_guard,
                "floorMm": FLOOR_MM,
                "geometry": {
                    "baseHeightMm": BASE_HEIGHT_MM,
                    "upperArmMm": UPPER_ARM_MM,
                    "distalMm": DISTAL_MM,
                },
            },
        }

    def calibrate(self, name: str, request: CalibrateRequest) -> dict[str, object]:
        with self._operation_lock, self._motion_lock:
            return self._calibrate_locked(name, request)

    def _calibrate_locked(self, name: str, request: CalibrateRequest) -> dict[str, object]:
        joint = self._store.get(name)
        if request.servoId is not None:
            duplicate_name = next(
                (
                    candidate
                    for candidate in JOINT_IDS
                    if candidate != name
                    and self._store.get(candidate).servoId == request.servoId
                ),
                None,
            )
            if duplicate_name is not None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        f"Servo ID {request.servoId} is already assigned to "
                        f"{JOINT_NAMES[duplicate_name]}."
                    ),
                )
            joint.servoId = request.servoId
            # The dialect follows the id, not the joint: pointing the camera at a
            # different servo has to re-declare, or the new id is read as STS.
            self._declare_servo_families()
        # Ratio and direction land before the limits, so a request that sets a
        # scale and a typed limit together converts with the new scale.
        if request.ratio is not None:
            joint.ratio = request.ratio
        if request.direction is not None:
            joint.direction = request.direction
        if request.speed is not None:
            joint.speed = request.speed
        if request.accel is not None:
            joint.accel = request.accel
        # Read once, here, so every field captured in this request agrees with a
        # single instant on the wire rather than with whatever the browser last
        # happened to poll.
        if request.here:
            previous_zero = joint.rawZero
            live = (
                self._home_multi_turn(joint.servoId)
                if joint.multiTurn and "rawZero" in request.here
                else self.live_raw(joint.servoId)
            )
            if live is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{JOINT_NAMES[name]} is not reporting a position, so there is no 'here' to store.",
                )
            if (
                joint.multiTurn
                and "rawZero" in request.here
                and previous_zero is not None
            ):
                # Re-homing changes the counted coordinate, not the physical
                # range. Translate saved limits so their degree bounds survive.
                frame_shift = live - previous_zero
                if joint.rawMin is not None:
                    joint.rawMin += frame_shift
                if joint.rawMax is not None:
                    joint.rawMax += frame_shift
            for field in request.here:
                setattr(joint, field, live)
        if request.rawZero is not None:
            if not joint.raw_min <= request.rawZero <= joint.raw_max:
                raise HTTPException(
                    status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                    detail=(
                        f"{JOINT_NAMES[name]} zero must be between "
                        f"{joint.raw_min} and {joint.raw_max}."
                    ),
                )
            joint.rawZero = request.rawZero
        # A degree is only meaningful relative to a zero, but the zero is now
        # derived from the limits, so refusing here made the first typed limit
        # impossible. Adopt wherever the joint is standing as a provisional zero
        # instead; `zeroFromLimits` re-centres it once both limits exist.
        if joint.rawZero is None and (request.minDegrees is not None or request.maxDegrees is not None):
            joint.rawZero = self.live_raw(joint.servoId)
            if joint.rawZero is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{JOINT_NAMES[name]} is not reporting a position, so a typed limit has nothing to measure from.",
                )
        for field, raw, degrees in (
            ("rawMin", request.rawMin, request.minDegrees),
            ("rawMax", request.rawMax, request.maxDegrees),
        ):
            if raw is not None:
                setattr(joint, field, raw)
            elif degrees is not None:
                # Stored as typed. This used to be clamped into one motor turn,
                # which stored a different number than the one asked for and then
                # had to explain itself; the operator knows what gearing is on the
                # arm and how far it swings, and a limit the servo cannot reach
                # costs nothing — `goal_for` still cannot send it past 4095.
                converted = joint.raw_for(degrees)
                assert converted is not None  # a zero exists by now
                setattr(joint, field, converted)
        if request.zeroFromLimits:
            if joint.rawMin is None or joint.rawMax is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{JOINT_NAMES[name]} needs both a min and a max before its zero can be centred.",
                )
            joint.rawZero = round((joint.rawMin + joint.rawMax) / 2)
        for field in request.clear or []:
            setattr(joint, field, None)
        self._store.save()
        return self.state()

    def _apply_hold(
        self, wanted: list[int], *, isolate_multi_turn_failures: bool = False
    ) -> None:
        """Set the controller's hold set, recovering from a stranded servo.

        The firmware only adds a servo to the hold set if that servo currently
        reads torque-off. A servo left energised while the controller holds no
        authority for it is therefore unacquirable — every HOLD_SET returns
        TORQUE_UNCONFIRMED and the joint can never be held or driven again
        without a power cycle. Dropping it first costs nothing: the controller
        was going to de-energise it on the next tick anyway, since it has an
        outstanding torque-off obligation and no authority.
        """

        reported = self._controller.transport_state()
        current = reported.get("heldServoIds")
        current = current if isinstance(current, list) else []
        physically_held = {
            servo_id
            for servo_id in current
            if isinstance(servo_id, int) and not isinstance(servo_id, bool)
        }
        telemetry = self.telemetry_of(reported)
        for servo_id in wanted:
            if servo_id in physically_held:
                continue
            servo = telemetry.get(servo_id)
            if isinstance(servo, dict) and servo.get("torqueState") == "on":
                try:
                    self._controller.torque_off(servo_id)
                except Exception:
                    # Let HOLD_SET report the real reason rather than masking it.
                    pass
        multi_turn_ids = {
            self._store.get(name).servoId for name in MULTI_TURN_JOINTS
        }
        ready = list(wanted)
        failed_multi_turn: list[tuple[int, Exception]] = []
        for servo_id in wanted:
            if servo_id in multi_turn_ids:
                try:
                    self._prepare_multi_turn_drive(servo_id)
                except Exception as error:
                    if not isolate_multi_turn_failures:
                        raise
                    ready.remove(servo_id)
                    failed_multi_turn.append((servo_id, error))
        if failed_multi_turn:
            failed_ids = {servo_id for servo_id, _ in failed_multi_turn}
            # `_held` is rendered as the physical hold state. Once Base cannot
            # be prepared, the reduced HOLD_SET below explicitly releases it;
            # leaving it here painted a free Base as held and caused every
            # renewal tick to repeat the same whole-arm failure.
            with self._lock:
                self._held = [servo_id for servo_id in self._held if servo_id not in failed_ids]
            for servo_id, error in failed_multi_turn:
                _LOG.warning(
                    "Dropping multi-turn servo %d from hold renewal: %r", servo_id, error
                )
        self._controller.set_hold_servos(ready, HOLD_LEASE_MS)

    def _ensure_held(self, servo_ids: list[int]) -> None:
        """Asking a joint to move is the same thing as asking for torque.

        The controller still refuses MOVE without authority — that is its own
        fail-safe and stays — but the operator should never have to arrange it.
        Only `torque()` and STOP ever give it up. Takes the whole set at once so
        a three-joint move costs one HOLD_SET, not three.

        Matching the bookkeeping is not enough to skip the HOLD_SET: `self._held`
        is what the Pi intends to hold, and the controller's authority behind it
        expires on its own clock. Skipping while that clock has run down is how a
        move gets refused with NO_TORQUE_LEASE when nothing is wrong, so the
        short circuit also requires the last HOLD_SET to be newer than the
        renewal interval -- which in normal running it always is, because the
        service loop keeps it so.
        """

        with self._lock:
            if (
                all(servo_id in self._held for servo_id in servo_ids)
                and time.monotonic() - self._last_hold < HOLD_RENEW_SECONDS
            ):
                return
            previous = list(self._held)
            # Requested first, then as much of the previous set as still fits.
            # Truncating a sorted union instead would have evicted whichever
            # joint happened to have the lowest id, including one mid-move.
            wanted = list(dict.fromkeys([*servo_ids, *previous]))[:MAX_HELD]
            self._held = wanted
            self._last_hold = time.monotonic()
        try:
            self._apply_hold(wanted)
        except Exception:
            with self._lock:
                self._held = previous
            raise

    def target(self, name: str, degrees: float) -> dict[str, object]:
        return self.targets({name: degrees})

    def _measured_degrees(self, name: str) -> float | None:
        joint = self._store.get(name)
        telemetry = self.telemetry_of(self._controller.transport_state())
        servo = telemetry.get(joint.servoId)
        if not isinstance(servo, dict) or not self._fresh_age(
            servo.get("packetAgeMs"), PLAN_MAX_SERVO_PACKET_AGE_MS
        ):
            return None
        raw = servo.get("rawPosition") if isinstance(servo, dict) else None
        counted = self._multi_turn_raw.get(joint.servoId)
        if joint.multiTurn:
            raw = counted if counted is not None and servo is not None else None
        elif counted is not None and servo is not None:
            raw = counted
        return joint.degrees_at(raw) if isinstance(raw, int) else None

    def _guard_floor(self, wanted: dict[str, float]) -> dict[str, float]:
        """Pull a requested pose up to the nearest one that clears the floor.

        Clamps rather than refuses, so nothing ever errors at the operator for
        dragging too low -- the arm simply stops at the plane. The UI applies the
        same constraint to the drag itself; this is the authority behind it, so a
        request that never went through the UI still cannot bury the arm.

        Joints the caller did not name are filled from measured position: a
        single-joint command still has to be judged against the whole pose,
        because the shoulder alone decides where the elbow ends up. If a needed
        angle cannot be measured freshly the guard refuses the move. Guessing a
        start pose is how a spatial guard silently drives through its keep-out.
        """

        with self._lock:
            enabled = self._floor_guard
        if not enabled or not any(name in wanted for name in FLOOR_JOINTS):
            return wanted
        pose: dict[str, float] = {}
        for name in FLOOR_JOINTS:
            value = wanted.get(name)
            if value is None:
                value = self._measured_degrees(name)
            if value is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Fresh Shoulder and Elbow telemetry is required for the floor guard.",
                )
            pose[name] = float(value)
        current: dict[str, float] = {}
        for name in FLOOR_JOINTS:
            measured = self._measured_degrees(name)
            if measured is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Fresh Shoulder and Elbow telemetry is required for the floor guard.",
                )
            current[name] = float(measured)

        def lowest(candidate: dict[str, float]) -> float:
            return lowest_point_mm(candidate["joint_2"], candidate["joint_3"])

        # Never below where the arm already is, so a pose that starts under the
        # plane can still be driven out of it, and the bar ratchets back up to
        # FLOOR_MM as it climbs.
        bar = min(FLOOR_MM, lowest(current))
        def swept_lowest(candidate: dict[str, float]) -> float:
            return swept_lowest_point_mm(
                current["joint_2"],
                current["joint_3"],
                candidate["joint_2"],
                candidate["joint_3"],
            )

        if swept_lowest(pose) >= bar:
            return wanted
        blocked, reachable = 1.0, 0.0
        for _ in range(14):
            middle = (reachable + blocked) / 2
            blended = {
                name: current[name] + (pose[name] - current[name]) * middle
                for name in FLOOR_JOINTS
            }
            if swept_lowest(blended) >= bar:
                reachable = middle
            else:
                blocked = middle
        guarded = dict(wanted)
        for name in FLOOR_JOINTS:
            if name in wanted:
                guarded[name] = round(
                    current[name] + (pose[name] - current[name]) * reachable, 3
                )
        return guarded

    def _configuration_digest(self) -> str:
        with self._lock:
            floor_guard = self._floor_guard
        joints = {}
        for name in JOINT_IDS:
            joint = self._store.get(name)
            joints[name] = {
                **joint.as_dict(),
                "steps": joint.steps,
                "spanDegrees": joint.spanDegrees,
                "multiTurn": joint.multiTurn,
            }
        return _sha256(
            {
                "joints": joints,
                "floorGuard": floor_guard,
                "floorMm": FLOOR_MM,
                "geometry": {
                    "baseHeightMm": BASE_HEIGHT_MM,
                    "upperArmMm": UPPER_ARM_MM,
                    "distalMm": DISTAL_MM,
                },
            }
        )

    @staticmethod
    def _require_plan_health(reported: dict[str, object]) -> str:
        if reported.get("connection") != "online":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller link is not online; no physical plan was prepared.",
            )
        if reported.get("bus") != "online":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The servo bus is not online; no physical plan was prepared.",
            )
        motion = reported.get("motionState")
        if motion == "stopped":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="STOP is latched. Clear it before preparing or applying a plan.",
            )
        if motion == "moving":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The arm is still moving. Wait for measured telemetry to settle, then plan again.",
            )
        if motion not in {"ready", "blocked"}:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller motion state is not safe for planning.",
            )
        heartbeat_age = reported.get("heartbeatAgeMs")
        if (
            isinstance(heartbeat_age, bool)
            or not isinstance(heartbeat_age, (int, float))
            or not math.isfinite(float(heartbeat_age))
            or heartbeat_age < 0
            or heartbeat_age > PLAN_MAX_HEARTBEAT_AGE_MS
        ):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Controller telemetry is stale; wait for a fresh heartbeat and plan again.",
            )
        if reported.get("hardwareEstop") == "active":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The hardware E-stop is active.",
            )
        if _collision_of(reported.get("lastScan"))["collisionSuspected"]:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The servo bus reports a possible ID collision.",
            )
        identity = reported.get("identity")
        boot_id = identity.get("bootId") if isinstance(identity, dict) else None
        if not isinstance(boot_id, str) or not boot_id:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller has no stable boot identity; no physical plan was prepared.",
            )
        return boot_id

    @staticmethod
    def _fresh_age(value: object, maximum_ms: int) -> bool:
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
            and 0 <= float(value) <= maximum_ms
        )

    def _planning_snapshot(
        self, required_joints: set[str]
    ) -> tuple[str, dict[str, float | None]]:
        """Read a motionless pose backed by fresh per-joint telemetry.

        The Base is special: its wrapping servo packet proves only that the
        servo answered.  A fresh, valid native odometer read from the same HAT
        boot is additionally required before Base can participate in a plan.
        """

        # STATUS is an explicit, bounded, read-only controller transaction. A
        # low age in an old cache is not sufficient evidence for a new plan.
        status_started = time.monotonic()
        fresh_reported = self._controller.status()
        status_finished = time.monotonic()
        # An explicit planning STATUS is also a real refresh. Advance the
        # periodic cadence so the service loop does not immediately duplicate
        # this relatively expensive serial transaction after the request lets
        # go of the motion lane.
        with self._lock:
            self._last_status = status_finished
            self._status_ms = int((status_finished - status_started) * 1000)
        if not isinstance(fresh_reported, dict):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller did not return a fresh telemetry generation.",
            )
        boot_id = self._require_plan_health(fresh_reported)

        base_proof: dict[str, object] | None = None
        base_joint = self._store.get("joint_1")
        read_odometer = getattr(self._controller, "odometer_read", None)
        if "joint_1" in required_joints and callable(read_odometer):
            try:
                with self._multi_turn_lock:
                    candidate = read_odometer(base_joint.servoId)
                if isinstance(candidate, dict):
                    self._cache_multi_turn_truth(base_joint.servoId, candidate)
                    base_proof = candidate
                    position = candidate.get("multiTurnPosition")
                    structurally_valid = (
                        candidate.get("tracking") is True
                        and candidate.get("valid") is True
                        and candidate.get("stepMode") is False
                        and candidate.get("stepOutstanding") is False
                        and candidate.get("countdownObserved") is False
                        and candidate.get("resyncNeeded") is False
                        and isinstance(position, int)
                        and not isinstance(position, bool)
                    )
                    if not structurally_valid:
                        # A fresh explicit UNKNOWN/invalid frame supersedes the
                        # old Pi cache immediately. The later planning checks
                        # still decide whether this request may proceed.
                        with self._lock:
                            self._multi_turn_raw.pop(base_joint.servoId, None)
                            self._multi_turn_armed.discard(base_joint.servoId)
            except Exception:
                base_proof = None

        # Bind the pose to the health/identity after the non-motion odometer
        # transaction.  A reboot during that read invalidates the whole snapshot.
        after = self._controller.transport_state()
        after_boot = self._require_plan_health(after)
        if after_boot != boot_id:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller boot changed while the plan was being prepared.",
            )
        if "joint_1" in required_joints and not self._multi_turn_frame_current(after):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller cannot prove the current Base odometer frame.",
            )
        telemetry = self.telemetry_of(after)
        pose: dict[str, float | None] = {}
        stale: list[str] = []
        for name in JOINT_IDS:
            joint = self._store.get(name)
            servo = telemetry.get(joint.servoId)
            packet_fresh = isinstance(servo, dict) and self._fresh_age(
                servo.get("packetAgeMs"), PLAN_MAX_SERVO_PACKET_AGE_MS
            )
            raw: int | None = None
            if joint.multiTurn:
                proof_fresh = (
                    isinstance(base_proof, dict)
                    and base_proof.get("tracking") is True
                    and base_proof.get("valid") is True
                    and base_proof.get("stepMode") is False
                    and self._fresh_age(
                        base_proof.get("sampleAgeMs"),
                        PLAN_MAX_ODOMETER_SAMPLE_AGE_MS,
                    )
                    and isinstance(base_proof.get("multiTurnPosition"), int)
                    and not isinstance(base_proof.get("multiTurnPosition"), bool)
                )
                if packet_fresh and proof_fresh:
                    raw = int(base_proof["multiTurnPosition"])
                    with self._lock:
                        self._multi_turn_raw[joint.servoId] = raw
                        self._multi_turn_armed.add(joint.servoId)
            elif packet_fresh:
                candidate_raw = servo.get("rawPosition")
                if isinstance(candidate_raw, int) and not isinstance(candidate_raw, bool):
                    raw = candidate_raw
            if name in required_joints and raw is None:
                stale.append(name)
            degrees = joint.degrees_at(raw) if raw is not None else None
            pose[name] = round(float(degrees), 6) if degrees is not None else None
        if stale:
            labels = ", ".join(JOINT_NAMES[name] for name in stale)
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "Fresh measured telemetry is unavailable for: "
                    f"{labels}. Wait for new servo packets and plan again."
                ),
            )
        return boot_id, pose

    def _resolve_plan_targets(
        self,
        wanted: dict[str, float],
        measured: dict[str, float | None],
    ) -> tuple[dict[str, float], dict[str, int], list[str], float | None]:
        """Quantize once, then fail closed if its independent sweep is unsafe."""

        warnings: list[str] = []
        resolved: dict[str, float] = {}
        goals: dict[str, int] = {}
        for name, requested in wanted.items():
            joint = self._store.get(name)
            if joint.rawZero is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{JOINT_NAMES[name]} has no zero yet. Set its zero before planning.",
                )
            goal = joint.goal_for(requested)
            degrees = joint.degrees_at(goal)
            assert degrees is not None
            goals[name] = goal
            resolved[name] = round(float(degrees), 6)
            if abs(resolved[name] - requested) > 0.05:
                warnings.append(
                    f"{JOINT_NAMES[name]} was limited from {requested:.2f} to {resolved[name]:.2f} degrees."
                )
        clearance: float | None = None
        if any(name in resolved for name in FLOOR_JOINTS):
            start = {name: measured.get(name) for name in FLOOR_JOINTS}
            if not all(isinstance(start[name], (int, float)) for name in FLOOR_JOINTS):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Fresh Shoulder and Elbow telemetry is required for floor-safe planning.",
                )
            target = {
                name: resolved.get(name, float(start[name])) for name in FLOOR_JOINTS
            }
            swept = swept_lowest_point_mm(
                float(start["joint_2"]),
                float(start["joint_3"]),
                float(target["joint_2"]),
                float(target["joint_3"]),
            )
            clearance = round(swept - FLOOR_MM, 3)
            with self._lock:
                floor_guard = self._floor_guard
            allowed_floor = min(
                FLOOR_MM,
                lowest_point_mm(float(start["joint_2"]), float(start["joint_3"])),
            )
            if floor_guard and swept < allowed_floor - 1e-6:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "The independently timed Shoulder/Elbow sweep crosses the floor "
                        "keep-out plane. Choose another pose."
                    ),
                )
            if not floor_guard and swept < FLOOR_MM:
                warnings.append(
                    "Floor guard is off and the independently timed arm sweep crosses "
                    "the normal keep-out plane."
                )
        return resolved, goals, warnings, clearance

    def _planar_targets(
        self,
        tip: PlanTip,
        preference: Literal["nearest", "up", "down"],
        measured: dict[str, float | None],
    ) -> tuple[dict[str, float], dict[str, object], list[str]]:
        current_shoulder = measured.get("joint_2")
        current_elbow = measured.get("joint_3")
        if not isinstance(current_shoulder, (int, float)) or not isinstance(
            current_elbow, (int, float)
        ):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Shoulder and Elbow need trusted measured angles before tip IK can be planned.",
            )
        raw_candidates, projected_target, projected = _planar_inverse(
            tip.radialMm, tip.heightMm
        )
        candidates: list[dict[str, object]] = []
        for raw_candidate in raw_candidates:
            requested = raw_candidate["servoPose"]
            assert isinstance(requested, dict)
            bounded: dict[str, float] = {}
            violations: list[str] = []
            clamp_cost = 0.0
            for name in FLOOR_JOINTS:
                value = float(requested[name])
                low, high = self._store.get(name).reach_bounds()
                if low is None or high is None:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=f"{JOINT_NAMES[name]} has no zero yet. Set its zero before planning tip IK.",
                    )
                held = max(low, min(high, value))
                bounded[name] = held
                clamp_cost += abs(held - value)
                if value < low - 1e-6:
                    violations.append(f"{name} below its reachable minimum")
                elif value > high + 1e-6:
                    violations.append(f"{name} above its reachable maximum")
            distance = math.hypot(
                bounded["joint_2"] - float(current_shoulder),
                bounded["joint_3"] - float(current_elbow),
            )
            candidates.append(
                {
                    "branch": raw_candidate["branch"],
                    "requestedPose": {
                        name: round(float(requested[name]), 3) for name in FLOOR_JOINTS
                    },
                    "boundedPose": {
                        name: round(value, 3) for name, value in bounded.items()
                    },
                    "withinLimits": not violations,
                    "violations": violations,
                    "distanceFromMeasured": round(distance, 3),
                    "clampCost": round(clamp_cost, 3),
                    "_score": clamp_cost + 0.02 * distance,
                }
            )
        eligible = (
            [candidate for candidate in candidates if candidate["branch"] == preference]
            if preference != "nearest"
            else candidates
        )
        eligible_within_limits = [
            candidate for candidate in eligible if candidate["withinLimits"] is True
        ]
        if not eligible_within_limits:
            alternate_exists = preference != "nearest" and any(
                candidate["withinLimits"] is True for candidate in candidates
            )
            detail = (
                f"The requested {preference} IK branch is outside calibrated joint limits. "
                "Choose the other branch or nearest."
                if alternate_exists
                else (
                    "Tip IK cannot reach the requested radial_mm and height_mm within "
                    "calibrated joint limits. Choose a different endpoint."
                )
            )
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=detail,
            )
        selected = min(
            eligible_within_limits,
            key=lambda candidate: float(candidate["_score"]),
        )
        selected_pose = selected["boundedPose"]
        assert isinstance(selected_pose, dict)
        warnings: list[str] = []
        if projected:
            warnings.append(
                "The requested tip was outside geometric reach and was projected to the nearest reach boundary."
            )
        for candidate in candidates:
            candidate.pop("_score", None)
        return (
            {name: float(selected_pose[name]) for name in FLOOR_JOINTS},
            {
                "target": {
                    "radialMm": round(tip.radialMm, 3),
                    "heightMm": round(tip.heightMm, 3),
                },
                "projectedTarget": projected_target,
                "projected": projected,
                "elbowPreference": preference,
                "selectedBranch": selected["branch"],
                "candidates": candidates,
            },
            warnings,
        )

    def _prune_plans(self, now: float) -> None:
        expired = [
            plan_id
            for plan_id, plan in self._plans.items()
            if float(plan["expiresMonotonic"]) <= now
        ]
        for plan_id in expired:
            self._plans.pop(plan_id, None)
        while len(self._plans) >= PLAN_CACHE_LIMIT:
            self._plans.pop(next(iter(self._plans)))

    def _prune_sequences(self, now: float) -> None:
        expired = [
            sequence_id
            for sequence_id, sequence in self._sequences.items()
            if float(sequence["expiresMonotonic"]) <= now
        ]
        for sequence_id in expired:
            self._sequences.pop(sequence_id, None)
        while len(self._sequences) >= SEQUENCE_CACHE_LIMIT:
            self._sequences.pop(next(iter(self._sequences)))

    def _resolve_sequence_waypoint(
        self,
        waypoint: SequenceWaypoint,
        start_pose: dict[str, float | None],
        index: int,
    ) -> tuple[dict[str, object], dict[str, object]]:
        """Resolve one segment from the prior resolved endpoint without motion."""

        wanted = waypoint.targets.supplied()
        # Every finite coordinate in the reviewed start is part of the segment
        # contract, even when this waypoint does not command that joint. A
        # carried-forward Shoulder/Base/Camera drifting between waypoints must
        # stale the segment instead of silently changing its physical sweep.
        required_measured = {
            name
            for name in JOINT_IDS
            if isinstance(start_pose.get(name), (int, float))
            and not isinstance(start_pose.get(name), bool)
        }
        required_measured.update(wanted)
        if waypoint.tip is not None or any(name in wanted for name in FLOOR_JOINTS):
            required_measured.update(FLOOR_JOINTS)
        warnings: list[str] = []
        ik: dict[str, object] | None = None
        if waypoint.tip is not None:
            planar, ik, ik_warnings = self._planar_targets(
                waypoint.tip,
                waypoint.elbowPreference,
                start_pose,
            )
            wanted.update(planar)
            warnings.extend(ik_warnings)
        missing = [
            name
            for name in sorted(required_measured)
            if not isinstance(start_pose.get(name), (int, float))
        ]
        if missing:
            labels = ", ".join(JOINT_NAMES[name] for name in missing)
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"Trusted measured angles are unavailable for: {labels}.",
            )
        resolved_targets, move_goals, limit_warnings, clearance = (
            self._resolve_plan_targets(wanted, start_pose)
        )
        warnings.extend(limit_warnings)
        resolved_pose = dict(start_pose)
        resolved_pose.update(resolved_targets)
        if clearance is None:
            shoulder = resolved_pose.get("joint_2")
            elbow = resolved_pose.get("joint_3")
            if isinstance(shoulder, (int, float)) and isinstance(elbow, (int, float)):
                clearance = round(lowest_point_mm(shoulder, elbow) - FLOOR_MM, 3)
        with self._lock:
            floor_guard = self._floor_guard
        measured_scene = _scene_points(start_pose)
        resolved_scene = _scene_points(resolved_pose)
        scene = (
            {
                "view": "side",
                "geometryMm": {
                    "baseHeightMm": BASE_HEIGHT_MM,
                    "upperArmMm": UPPER_ARM_MM,
                    "distalMm": DISTAL_MM,
                },
                "floorGuard": {"enabled": floor_guard, "floorMm": FLOOR_MM},
                "measured": measured_scene,
                "resolved": resolved_scene,
            }
            if measured_scene is not None and resolved_scene is not None
            else None
        )
        public: dict[str, object] = {
            "index": index,
            "startPose": start_pose,
            "resolvedPose": resolved_pose,
            "warnings": warnings,
            "lowestClearanceMm": clearance,
        }
        if waypoint.label is not None:
            public["label"] = waypoint.label
        if scene is not None:
            public["scene"] = scene
        if ik is not None:
            public["ik"] = ik
        stored = {
            **public,
            "moveTargets": resolved_targets,
            "moveGoals": move_goals,
            "requiredJoints": sorted(required_measured),
        }
        return public, stored

    def preview_sequence(self, request: SequencePreviewRequest) -> dict[str, object]:
        """Prepare 2..8 exact poses from one live start without moving the arm."""

        preview_started = time.perf_counter()
        with self._operation_lock, self._motion_lock:
            self._require_no_outstanding_goals()
            required_initial: set[str] = set()
            for waypoint in request.waypoints:
                wanted = waypoint.targets.supplied()
                required_initial.update(wanted)
                if waypoint.tip is not None or any(
                    name in wanted for name in FLOOR_JOINTS
                ):
                    required_initial.update(FLOOR_JOINTS)
            boot_id, measured = self._planning_snapshot(required_initial)
            public_waypoints: list[dict[str, object]] = []
            stored_waypoints: list[dict[str, object]] = []
            aggregate_warnings: list[str] = []
            start_pose = dict(measured)
            for index, waypoint in enumerate(request.waypoints):
                try:
                    public, stored = self._resolve_sequence_waypoint(
                        waypoint,
                        start_pose,
                        index,
                    )
                except HTTPException as error:
                    label = (
                        f" ({waypoint.label})" if waypoint.label is not None else ""
                    )
                    raise HTTPException(
                        status_code=error.status_code,
                        detail=(
                            f"Transition into waypoint {index + 1}{label} was rejected: "
                            f"{error.detail}"
                        ),
                        headers=error.headers,
                    ) from None
                public_waypoints.append(public)
                stored_waypoints.append(stored)
                for warning in public["warnings"]:
                    aggregate_warnings.append(f"Waypoint {index + 1}: {warning}")
                resolved = stored["resolvedPose"]
                assert isinstance(resolved, dict)
                start_pose = dict(resolved)

            sequence_id = "armseq_" + secrets.token_urlsafe(12)
            now_monotonic = time.monotonic()
            expires_monotonic = now_monotonic + SEQUENCE_TTL_MS / 1000.0
            expires_at = _expires_at(time.time() + SEQUENCE_TTL_MS / 1000.0)
            configuration_digest = self._configuration_digest()
            digest_payload = {
                "sequenceId": sequence_id,
                "bootId": boot_id,
                "configurationDigest": configuration_digest,
                "measuredPose": measured,
                "waypoints": [
                    {
                        "index": waypoint["index"],
                        "label": waypoint.get("label"),
                        "startPose": waypoint["startPose"],
                        "resolvedPose": waypoint["resolvedPose"],
                        "moveTargets": waypoint["moveTargets"],
                        "moveGoals": waypoint["moveGoals"],
                        "requiredJoints": waypoint["requiredJoints"],
                        "warnings": waypoint.get("warnings", []),
                        "lowestClearanceMm": waypoint.get("lowestClearanceMm"),
                        "scene": waypoint.get("scene"),
                        "ik": waypoint.get("ik"),
                    }
                    for waypoint in stored_waypoints
                ],
                "expiresAt": expires_at,
            }
            sequence_digest = _sha256(digest_payload)
            numeric_clearances = [
                float(waypoint["lowestClearanceMm"])
                for waypoint in public_waypoints
                if isinstance(waypoint.get("lowestClearanceMm"), (int, float))
                and not isinstance(waypoint.get("lowestClearanceMm"), bool)
            ]
            response: dict[str, object] = {
                "sequenceId": sequence_id,
                "sequenceDigest": sequence_digest,
                "expiresAt": expires_at,
                "expiresInMs": SEQUENCE_TTL_MS,
                "previewDurationMs": int(
                    (time.perf_counter() - preview_started) * 1000
                ),
                "measuredPose": measured,
                "waypointCount": len(public_waypoints),
                "warnings": aggregate_warnings,
                "lowestClearanceMm": (
                    round(min(numeric_clearances), 3)
                    if numeric_clearances
                    else None
                ),
                "waypoints": public_waypoints,
            }
            sequence = {
                **response,
                "bootId": boot_id,
                "configurationDigest": configuration_digest,
                "storedWaypoints": stored_waypoints,
                "expiresMonotonic": expires_monotonic,
            }
            with self._plan_lock:
                self._prune_sequences(now_monotonic)
                self._sequences[sequence_id] = sequence
            return response

    def _require_no_outstanding_goals(self) -> None:
        with self._lock:
            outstanding = sorted(self._goals)
        if outstanding:
            labels = ", ".join(JOINT_NAMES.get(name, name) for name in outstanding)
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    f"An earlier move is still settling ({labels}). Wait for fresh "
                    "measured telemetry before preparing or applying a plan."
                ),
            )

    def preview_plan(self, request: PlanPreviewRequest) -> dict[str, object]:
        """Prepare an authoritative, motionless, one-use physical pose."""

        with self._operation_lock, self._motion_lock:
            wanted = request.targets.supplied()
            required_measured = set(wanted)
            if request.tip is not None or any(name in wanted for name in FLOOR_JOINTS):
                required_measured.update(FLOOR_JOINTS)
            self._require_no_outstanding_goals()
            boot_id, measured = self._planning_snapshot(required_measured)
            warnings: list[str] = []
            ik: dict[str, object] | None = None
            if request.tip is not None:
                planar, ik, ik_warnings = self._planar_targets(
                    request.tip, request.elbowPreference, measured
                )
                wanted.update(planar)
                warnings.extend(ik_warnings)
            missing = [name for name in sorted(required_measured) if measured.get(name) is None]
            if missing:
                labels = ", ".join(JOINT_NAMES[name] for name in missing)
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"Trusted measured angles are unavailable for: {labels}.",
                )
            (
                resolved_targets,
                move_goals,
                limit_warnings,
                swept_clearance,
            ) = self._resolve_plan_targets(wanted, measured)
            warnings.extend(limit_warnings)
            resolved_pose = dict(measured)
            resolved_pose.update(resolved_targets)
            clearance: float | None = swept_clearance
            shoulder = resolved_pose.get("joint_2")
            elbow = resolved_pose.get("joint_3")
            if (
                clearance is None
                and isinstance(shoulder, (int, float))
                and isinstance(elbow, (int, float))
            ):
                clearance = round(lowest_point_mm(shoulder, elbow) - FLOOR_MM, 3)
            with self._lock:
                floor_guard = self._floor_guard
            measured_scene = _scene_points(measured)
            resolved_scene = _scene_points(resolved_pose)
            scene = (
                {
                    "view": "side",
                    "geometryMm": {
                        "baseHeightMm": BASE_HEIGHT_MM,
                        "upperArmMm": UPPER_ARM_MM,
                        "distalMm": DISTAL_MM,
                    },
                    "floorGuard": {"enabled": floor_guard, "floorMm": FLOOR_MM},
                    "measured": measured_scene,
                    "resolved": resolved_scene,
                }
                if measured_scene is not None and resolved_scene is not None
                else None
            )
            plan_id = "armplan_" + secrets.token_urlsafe(12)
            now_monotonic = time.monotonic()
            expires_monotonic = now_monotonic + PLAN_TTL_MS / 1000.0
            expires_wall = time.time() + PLAN_TTL_MS / 1000.0
            configuration_digest = self._configuration_digest()
            digest_payload = {
                "planId": plan_id,
                "bootId": boot_id,
                "configurationDigest": configuration_digest,
                "measuredPose": measured,
                "resolvedPose": resolved_pose,
                "moveTargets": resolved_targets,
                "moveGoals": move_goals,
                "requiredJoints": sorted(required_measured),
                "expiresAt": _expires_at(expires_wall),
            }
            plan_digest = _sha256(digest_payload)
            response: dict[str, object] = {
                "planId": plan_id,
                "planDigest": plan_digest,
                "expiresAt": digest_payload["expiresAt"],
                "expiresInMs": PLAN_TTL_MS,
                "measuredPose": measured,
                "resolvedPose": resolved_pose,
                "warnings": warnings,
                "lowestClearanceMm": clearance,
            }
            if scene is not None:
                response["scene"] = scene
            if ik is not None:
                response["ik"] = ik
            plan = {
                **response,
                "bootId": boot_id,
                "configurationDigest": configuration_digest,
                "moveTargets": resolved_targets,
                "moveGoals": move_goals,
                "requiredJoints": sorted(required_measured),
                "expiresMonotonic": expires_monotonic,
            }
            with self._plan_lock:
                self._prune_plans(now_monotonic)
                self._plans[plan_id] = plan
            return response

    def execute_plan(self, request: PlanExecuteRequest) -> dict[str, object]:
        """Consume exactly one prepared pose after revalidating live safety."""

        with self._operation_lock, self._motion_lock:
            with self._plan_lock:
                plan = self._plans.pop(request.planId, None)
            if plan is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="That plan is unavailable, expired, or already used. Prepare it again.",
                )
            expected_digest = plan.get("planDigest")
            if not isinstance(expected_digest, str) or not hmac.compare_digest(
                request.planDigest, expected_digest
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The plan digest does not match the reviewed pose.",
                )
            if time.monotonic() >= float(plan["expiresMonotonic"]):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="That plan expired. Read the current arm and prepare it again.",
                )
            self._require_no_outstanding_goals()
            required = plan.get("requiredJoints")
            assert isinstance(required, list)
            boot_id, measured = self._planning_snapshot(set(required))
            if boot_id != plan.get("bootId"):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The controller boot changed after preview; the plan is stale.",
                )
            if self._configuration_digest() != plan.get("configurationDigest"):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Calibration, limits, geometry, or floor protection changed after preview.",
                )
            start_pose = plan.get("measuredPose")
            assert isinstance(start_pose, dict)
            for name in required:
                start = start_pose.get(name)
                if not isinstance(start, (int, float)):
                    continue
                current = measured.get(name)
                if (
                    not isinstance(current, (int, float))
                    or abs(current - start) > PLAN_START_POSE_TOLERANCE_DEGREES
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=(
                            f"{JOINT_NAMES.get(name, name)} moved after preview; the plan is stale."
                        ),
                    )
            resolved_pose = plan.get("resolvedPose")
            assert isinstance(resolved_pose, dict)
            move_targets = plan.get("moveTargets")
            move_goals = plan.get("moveGoals")
            assert isinstance(move_targets, dict)
            assert isinstance(move_goals, dict)

            # A small accepted start-pose tolerance must not create a newly
            # unsafe rectangle. Revalidate safety only; never alter the reviewed
            # target or run it through the clamping resolver a second time.
            if any(name in move_targets for name in FLOOR_JOINTS):
                current_shoulder = measured.get("joint_2")
                current_elbow = measured.get("joint_3")
                target_shoulder = resolved_pose.get("joint_2")
                target_elbow = resolved_pose.get("joint_3")
                if not all(
                    isinstance(value, (int, float))
                    for value in (
                        current_shoulder,
                        current_elbow,
                        target_shoulder,
                        target_elbow,
                    )
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fresh Shoulder and Elbow telemetry is required before apply.",
                    )
                with self._lock:
                    floor_guard = self._floor_guard
                swept = swept_lowest_point_mm(
                    float(current_shoulder),
                    float(current_elbow),
                    float(target_shoulder),
                    float(target_elbow),
                )
                allowed_floor = min(
                    FLOOR_MM,
                    lowest_point_mm(float(current_shoulder), float(current_elbow)),
                )
                if floor_guard and swept < allowed_floor - 1e-6:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="The live independently timed sweep is no longer floor-safe.",
                    )

            prepared: list[tuple[str, JointState, int]] = []
            for name in JOINT_IDS:
                raw_goal = move_goals.get(name)
                if raw_goal is None:
                    continue
                if (
                    name not in JOINT_IDS
                    or not isinstance(raw_goal, int)
                    or isinstance(raw_goal, bool)
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="The prepared raw goals are invalid; prepare the plan again.",
                    )
                joint = self._store.get(name)
                reviewed_degrees = move_targets.get(name)
                exact_degrees = joint.degrees_at(raw_goal)
                if (
                    not isinstance(reviewed_degrees, (int, float))
                    or exact_degrees is None
                    or abs(round(float(exact_degrees), 6) - float(reviewed_degrees)) > 1e-6
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="The prepared raw goals no longer match the reviewed pose.",
                    )
                prepared.append((name, joint, raw_goal))
            moved = self._drive_prepared_goals_locked(prepared)
            return {
                "executed": True,
                "planId": request.planId,
                "resolvedPose": resolved_pose,
                "moved": moved,
            }

    @staticmethod
    def _pose_from_state(snapshot: object) -> dict[str, float | None] | None:
        if not isinstance(snapshot, dict):
            return None
        rows = snapshot.get("joints")
        if not isinstance(rows, list):
            return None
        by_name = {
            row.get("id"): row
            for row in rows
            if isinstance(row, dict) and row.get("id") in JOINT_IDS
        }
        pose: dict[str, float | None] = {}
        for name in JOINT_IDS:
            row = by_name.get(name)
            degrees = row.get("degrees") if isinstance(row, dict) else None
            pose[name] = (
                round(float(degrees), 6)
                if isinstance(degrees, (int, float)) and not isinstance(degrees, bool)
                else None
            )
        return pose

    def _cache_sequence_result(
        self,
        key: tuple[str, str],
        mode: Literal["execute", "capture"],
        result: dict[str, object],
    ) -> dict[str, object]:
        with self._plan_lock:
            self._sequence_results[key] = {
                "mode": mode,
                "result": deepcopy(result),
            }
            self._sequence_results.move_to_end(key)
            while len(self._sequence_results) > SEQUENCE_RESULT_CACHE_LIMIT:
                self._sequence_results.popitem(last=False)
        return result

    def _replay_sequence_result(
        self,
        key: tuple[str, str],
        mode: Literal["execute", "capture"],
        camera: CameraEvidenceService | None,
    ) -> dict[str, object] | None:
        with self._plan_lock:
            cached = self._sequence_results.get(key)
        if cached is None:
            return None
        if cached.get("mode") != mode:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "That sequence already ran in a different execution mode; "
                    "prepare a new sequence."
                ),
            )
        value = cached.get("result")
        if not isinstance(value, dict):
            raise RuntimeError("cached sequence result was invalid")
        replay = deepcopy(value)
        if mode == "capture" and replay.get("outcome") == "captured":
            capture = replay.get("capture")
            transfer_token = (
                capture.get("transferToken") if isinstance(capture, dict) else None
            )
            if (
                camera is None
                or not isinstance(transfer_token, str)
                or not camera.has_transfer(transfer_token)
            ):
                replay["outcome"] = "capture_unavailable"
                replay["capture"] = None
                replay["captureUnavailableReason"] = (
                    "exact_frame_transfer_unavailable"
                )
                return self._cache_sequence_result(key, mode, replay)
        return replay

    def _revalidate_sequence_waypoint(
        self,
        sequence: dict[str, object],
        waypoint: dict[str, object],
    ) -> tuple[str, str, dict[str, float | None], list[tuple[str, JointState, int]]]:
        """Recheck one reviewed segment from fresh actual telemetry."""

        self._require_no_outstanding_goals()
        required = waypoint.get("requiredJoints")
        if not isinstance(required, list) or any(name not in JOINT_IDS for name in required):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The prepared waypoint is invalid; prepare the sequence again.",
            )
        boot_id, measured = self._planning_snapshot(set(required))
        if boot_id != sequence.get("bootId"):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller boot changed after sequence preview.",
            )
        configuration = self._configuration_digest()
        if configuration != sequence.get("configurationDigest"):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "Calibration, limits, geometry, or floor protection changed "
                    "after sequence preview."
                ),
            )
        start_pose = waypoint.get("startPose")
        if not isinstance(start_pose, dict):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The prepared waypoint has no valid start pose.",
            )
        for name in required:
            start = start_pose.get(name)
            current = measured.get(name)
            if (
                not isinstance(start, (int, float))
                or not isinstance(current, (int, float))
                or abs(float(current) - float(start))
                > PLAN_START_POSE_TOLERANCE_DEGREES
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        f"{JOINT_NAMES.get(name, name)} moved away from the reviewed "
                        "waypoint start."
                    ),
                )

        resolved_pose = waypoint.get("resolvedPose")
        move_targets = waypoint.get("moveTargets")
        move_goals = waypoint.get("moveGoals")
        if not all(
            isinstance(value, dict)
            for value in (resolved_pose, move_targets, move_goals)
        ):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The prepared waypoint is invalid; prepare the sequence again.",
            )
        assert isinstance(resolved_pose, dict)
        assert isinstance(move_targets, dict)
        assert isinstance(move_goals, dict)
        if any(name in move_targets for name in FLOOR_JOINTS):
            current_shoulder = measured.get("joint_2")
            current_elbow = measured.get("joint_3")
            target_shoulder = resolved_pose.get("joint_2")
            target_elbow = resolved_pose.get("joint_3")
            if not all(
                isinstance(value, (int, float))
                for value in (
                    current_shoulder,
                    current_elbow,
                    target_shoulder,
                    target_elbow,
                )
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Fresh Shoulder and Elbow telemetry is required before apply.",
                )
            with self._lock:
                floor_guard = self._floor_guard
            swept = swept_lowest_point_mm(
                float(current_shoulder),
                float(current_elbow),
                float(target_shoulder),
                float(target_elbow),
            )
            allowed_floor = min(
                FLOOR_MM,
                lowest_point_mm(float(current_shoulder), float(current_elbow)),
            )
            if floor_guard and swept < allowed_floor - 1e-6:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The live waypoint sweep is no longer floor-safe.",
                )

        prepared: list[tuple[str, JointState, int]] = []
        for name in JOINT_IDS:
            raw_goal = move_goals.get(name)
            if raw_goal is None:
                continue
            if not isinstance(raw_goal, int) or isinstance(raw_goal, bool):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The prepared waypoint goals are invalid.",
                )
            joint = self._store.get(name)
            reviewed_degrees = move_targets.get(name)
            exact_degrees = joint.degrees_at(raw_goal)
            if (
                not isinstance(reviewed_degrees, (int, float))
                or exact_degrees is None
                or abs(round(float(exact_degrees), 6) - float(reviewed_degrees))
                > 1e-6
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The prepared waypoint goals no longer match review.",
                )
            prepared.append((name, joint, raw_goal))
        return boot_id, configuration, measured, prepared

    def _execute_sequence_waypoint(
        self,
        sequence: dict[str, object],
        waypoint: dict[str, object],
        timeout_seconds: float,
    ) -> tuple[dict[str, object], dict[str, object] | None]:
        waypoint_started = time.perf_counter()
        with self._motion_lock:
            try:
                boot_id, configuration, _measured, prepared = (
                    self._revalidate_sequence_waypoint(sequence, waypoint)
                )
            except Exception as error:
                return (
                    self._sequence_revalidation_failure(
                        waypoint,
                        error,
                        waypoint_started,
                    ),
                    None,
                )
            dispatch_started = time.perf_counter()
            try:
                moved = self._drive_prepared_goals_locked(prepared)
            except Exception:
                dispatch_ms = round(
                    (time.perf_counter() - dispatch_started) * 1000,
                    6,
                )
                try:
                    reported = self._controller.transport_state()
                except Exception:
                    # The original dispatch exception is already an ambiguous
                    # motion boundary. A second telemetry failure must not
                    # escape before the sequence can latch STOP and cache its
                    # terminal evidence.
                    reported = {}
                with self._lock:
                    stopped = self._operator_stopped or self._inspection_required
                outcome = (
                    "stopped"
                    if stopped
                    or reported.get("motionState") == "stopped"
                    or self._reported_clear_required_stop(reported)
                    else "arrival_fault"
                )
                row: dict[str, object] = {
                    "index": waypoint["index"],
                    "outcome": outcome,
                    "dispatched": True,
                    "resolvedPose": waypoint["resolvedPose"],
                    "moved": [],
                    "arrival": {
                        "proved": False,
                        "elapsedMs": 0,
                        "toleranceDegrees": ARRIVAL_DEGREES,
                        "sampleCount": 0,
                        "samples": 0,
                        "reason": "waypoint_dispatch_failed",
                    },
                    "dispatchMs": dispatch_ms,
                    "arrivalMs": 0,
                    "durationMs": int(
                        (time.perf_counter() - waypoint_started) * 1000
                    ),
                }
                if "label" in waypoint:
                    row["label"] = waypoint["label"]
                return row, None
            dispatch_ms = round(
                (time.perf_counter() - dispatch_started) * 1000,
                6,
            )
            with self._lock:
                dispatch_generation = self._telemetry_generation
                stop_generation = self._stop_generation

        arrival_started = time.perf_counter()
        try:
            outcome, arrival, sample = self._wait_for_arrival(
                moved=moved,
                expected_boot_id=boot_id,
                expected_configuration=configuration,
                after_generation=dispatch_generation,
                stop_generation=stop_generation,
                timeout_seconds=timeout_seconds,
            )
        except Exception as error:
            _LOG.exception(
                "Post-dispatch sequence arrival check failed at waypoint %s: %r",
                waypoint.get("index"),
                error,
            )
            return (
                self._sequence_post_dispatch_failure(
                    waypoint,
                    waypoint_started,
                    reason="post_dispatch_arrival_check_failed",
                    moved=moved,
                    dispatch_ms=dispatch_ms,
                    arrival_ms=round(
                        (time.perf_counter() - arrival_started) * 1000,
                        6,
                    ),
                ),
                None,
            )
        if outcome == "arrived":
            self._forget_goals(
                [
                    str(command["joint"])
                    for command in moved
                    if command.get("joint") in JOINT_IDS
                ]
            )
        row = {
            "index": waypoint["index"],
            "outcome": outcome,
            "dispatched": True,
            "resolvedPose": waypoint["resolvedPose"],
            "moved": moved,
            "arrival": arrival,
            "dispatchMs": dispatch_ms,
            "arrivalMs": int(arrival.get("elapsedMs", 0)),
            "durationMs": int((time.perf_counter() - waypoint_started) * 1000),
        }
        if "label" in waypoint:
            row["label"] = waypoint["label"]
        return row, sample

    def _sequence_post_dispatch_failure(
        self,
        waypoint: dict[str, object],
        started: float,
        *,
        reason: str,
        moved: list[dict[str, object]] | None = None,
        dispatch_ms: float | int | None = None,
        arrival_ms: float | int | None = None,
    ) -> dict[str, object]:
        """Render a post-send abort without claiming arrival or issuing STOP."""

        elapsed_ms = round((time.perf_counter() - started) * 1000, 6)
        arrival_elapsed = arrival_ms if arrival_ms is not None else elapsed_ms
        row: dict[str, object] = {
            "index": waypoint["index"],
            "outcome": "arrival_fault",
            "dispatched": True,
            "resolvedPose": waypoint.get("resolvedPose"),
            "moved": moved or [],
            "arrival": {
                "proved": False,
                "elapsedMs": arrival_elapsed,
                "toleranceDegrees": ARRIVAL_DEGREES,
                "sampleCount": 0,
                "samples": 0,
                "reason": reason,
            },
            "dispatchMs": dispatch_ms,
            "arrivalMs": arrival_ms,
            "durationMs": elapsed_ms,
        }
        if "label" in waypoint:
            row["label"] = waypoint["label"]
        return row

    def _sequence_revalidation_failure(
        self,
        waypoint: dict[str, object],
        error: Exception,
        started: float,
    ) -> dict[str, object]:
        """Render a no-dispatch refusal without pretending arrival was attempted."""

        try:
            reported = self._controller.transport_state()
        except Exception:
            reported = {}
        with self._lock:
            stopped = self._operator_stopped or self._inspection_required
        stopped = (
            stopped
            or reported.get("motionState") == "stopped"
            or self._reported_clear_required_stop(reported)
        )
        detail = error.detail if isinstance(error, HTTPException) else None
        detail_text = detail if isinstance(detail, str) else ""
        if stopped or (
            isinstance(error, ControllerCommandError)
            and error.code in {"STOPPED", "OPERATOR_INSPECTION_REQUIRED"}
        ):
            outcome = "stopped"
            reason = "stop_latched_before_waypoint"
        else:
            outcome = "refused"
            if "moved away from the reviewed" in detail_text:
                reason = "start_pose_changed"
            elif "boot changed" in detail_text:
                reason = "controller_boot_changed"
            elif "floor" in detail_text:
                reason = "waypoint_sweep_unsafe"
            elif "Calibration, limits, geometry" in detail_text:
                reason = "arm_configuration_changed"
            elif "telemetry" in detail_text:
                reason = "telemetry_unavailable"
            elif isinstance(
                error,
                (
                    ControllerProtocolError,
                    ControllerTransportError,
                    ControllerUnavailableError,
                ),
            ):
                reason = "controller_link_unhealthy"
            else:
                reason = "waypoint_revalidation_failed"
        arrival: dict[str, object] = {
            "proved": False,
            "elapsedMs": 0,
            "toleranceDegrees": ARRIVAL_DEGREES,
            "sampleCount": 0,
            "samples": 0,
            "reason": reason,
        }
        if detail_text:
            arrival["detail"] = detail_text
        row: dict[str, object] = {
            "index": waypoint["index"],
            "outcome": outcome,
            "dispatched": False,
            "resolvedPose": waypoint.get("resolvedPose"),
            "moved": [],
            "arrival": arrival,
            "dispatchMs": None,
            "arrivalMs": None,
            "durationMs": round(
                (time.perf_counter() - started) * 1000,
                6,
            ),
        }
        if "label" in waypoint:
            row["label"] = waypoint["label"]
        return row

    @staticmethod
    def _sequence_failure_requires_stop(row: dict[str, object]) -> bool:
        """Reserve STOP for failures where dispatch itself is ambiguous."""

        arrival = row.get("arrival")
        reason = arrival.get("reason") if isinstance(arrival, dict) else None
        return reason in {
            "waypoint_dispatch_failed",
            "post_dispatch_arrival_check_failed",
            "post_dispatch_capture_check_failed",
        }

    def _stop_ambiguous_sequence_failure(self) -> None:
        """Latch STOP only when the Pi cannot prove what a dispatch did."""

        try:
            self.stop_for_commissioning()
        except Exception:
            # stop_for_commissioning persists STOP_DELIVERY_UNKNOWN itself.
            pass

    def _install_sequence_capture_plan(
        self,
        sequence: dict[str, object],
        waypoint: dict[str, object],
        timeout_ms: int,
        capture_profile: Literal["survey", "detail"],
    ) -> PlanExecuteCaptureRequest:
        plan_id = "armplan_" + secrets.token_urlsafe(12)
        plan_digest = _sha256(
            {
                "sequenceId": sequence["sequenceId"],
                "sequenceDigest": sequence["sequenceDigest"],
                "waypointIndex": waypoint["index"],
                "moveGoals": waypoint["moveGoals"],
            }
        )
        now = time.monotonic()
        expires_monotonic = now + timeout_ms / 1000.0 + 5.0
        plan = {
            "planId": plan_id,
            "planDigest": plan_digest,
            "expiresAt": _expires_at(time.time() + timeout_ms / 1000.0 + 5.0),
            "expiresInMs": timeout_ms + 5_000,
            "measuredPose": waypoint["startPose"],
            "resolvedPose": waypoint["resolvedPose"],
            "warnings": waypoint.get("warnings", []),
            "lowestClearanceMm": waypoint.get("lowestClearanceMm"),
            "bootId": sequence["bootId"],
            "configurationDigest": sequence["configurationDigest"],
            "moveTargets": waypoint["moveTargets"],
            "moveGoals": waypoint["moveGoals"],
            "requiredJoints": waypoint["requiredJoints"],
            "expiresMonotonic": expires_monotonic,
        }
        with self._plan_lock:
            self._prune_plans(now)
            self._plans[plan_id] = plan
        return PlanExecuteCaptureRequest(
            planId=plan_id,
            planDigest=plan_digest,
            arrivalTimeoutMs=timeout_ms,
            captureProfile=capture_profile,
        )

    def execute_sequence(
        self,
        request: SequenceExecuteRequest,
    ) -> dict[str, object]:
        return self._execute_sequence(request, camera=None, mode="execute")

    def execute_sequence_and_capture(
        self,
        request: SequenceExecuteCaptureRequest,
        camera: CameraEvidenceService | None,
    ) -> dict[str, object]:
        return self._execute_sequence(request, camera=camera, mode="capture")

    def _execute_sequence(
        self,
        request: SequenceExecuteRequest | SequenceExecuteCaptureRequest,
        *,
        camera: CameraEvidenceService | None,
        mode: Literal["execute", "capture"],
    ) -> dict[str, object]:
        operation_started = time.perf_counter()
        key = (request.sequenceId, request.sequenceDigest)
        with self._operation_lock:
            replay = self._replay_sequence_result(key, mode, camera)
            if replay is not None:
                return replay
            if mode == "capture":
                if camera is None or camera.status().get("available") is not True:
                    raise HTTPException(
                        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                        detail="Camera is unavailable; the reviewed sequence was not executed.",
                    )
                if not camera.transfer_available():
                    raise HTTPException(
                        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                        detail=(
                            "Exact camera-transfer capacity is unavailable; "
                            "the reviewed sequence was not executed."
                        ),
                    )
                reported_before_sequence = self._controller.transport_state()
                with self._lock:
                    inspection_before_sequence = self._inspection_required
                if (
                    inspection_before_sequence
                    or self._reported_clear_required_stop(reported_before_sequence)
                ):
                    raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
                # A final photograph is evidence from all four view-defining
                # joints. Refuse before consuming the sequence or moving its
                # first waypoint when the Camera servo dialect is not proven.
                self._require_servo_family_ready(reported_before_sequence)
            with self._plan_lock:
                sequence = self._sequences.pop(request.sequenceId, None)
            if sequence is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "That sequence is unavailable, expired, or already used. "
                        "Prepare it again."
                    ),
                )
            expected_digest = sequence.get("sequenceDigest")
            if not isinstance(expected_digest, str) or not hmac.compare_digest(
                request.sequenceDigest,
                expected_digest,
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The sequence digest does not match the reviewed waypoints.",
                )
            if time.monotonic() >= float(sequence["expiresMonotonic"]):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="That sequence expired. Read the current arm and prepare it again.",
                )
            stored_waypoints = sequence.get("storedWaypoints")
            if not isinstance(stored_waypoints, list) or not stored_waypoints:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The prepared sequence is invalid; prepare it again.",
                )
            operation_id = "armseqop_" + secrets.token_urlsafe(12)
            waypoint_results: list[dict[str, object]] = []
            completed = 0
            final_measured: dict[str, float | None] | None = None
            final_resolved = stored_waypoints[-1].get("resolvedPose")

            def finish(
                outcome: str,
                failed_index: int | None,
                **extra: object,
            ) -> dict[str, object]:
                result: dict[str, object] = {
                    "operationId": operation_id,
                    "sequenceId": request.sequenceId,
                    "waypointCount": len(stored_waypoints),
                    "executed": any(
                        row.get("dispatched") is True for row in waypoint_results
                    ),
                    "outcome": outcome,
                    "completedWaypointCount": completed,
                    "failedWaypointIndex": failed_index,
                    "waypointResults": waypoint_results,
                    "resolvedPose": final_resolved,
                    "finalMeasuredPose": final_measured,
                    "durationMs": int(
                        (time.perf_counter() - operation_started) * 1000
                    ),
                    "captureAttempted": False,
                    "capture": None,
                }
                result.update(extra)
                return self._cache_sequence_result(key, mode, result)

            manual_count = (
                len(stored_waypoints) - 1
                if mode == "capture"
                else len(stored_waypoints)
            )
            for index in range(manual_count):
                waypoint = stored_waypoints[index]
                if not isinstance(waypoint, dict):
                    if waypoint_results:
                        return finish("arrival_fault", index)
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="The prepared sequence is invalid.",
                    )
                row, sample = self._execute_sequence_waypoint(
                    sequence,
                    waypoint,
                    request.arrivalTimeoutMs / 1000.0,
                )
                waypoint_results.append(row)
                state = sample.get("state") if isinstance(sample, dict) else None
                pose = self._pose_from_state(state)
                if pose is not None:
                    final_measured = pose
                if row.get("outcome") != "arrived":
                    outcome = str(row.get("outcome"))
                    if self._sequence_failure_requires_stop(row):
                        self._stop_ambiguous_sequence_failure()
                    return finish(outcome, index)
                completed += 1

            if mode == "capture":
                final_index = len(stored_waypoints) - 1
                final_waypoint = stored_waypoints[final_index]
                assert isinstance(final_waypoint, dict)
                capture_started = time.perf_counter()
                try:
                    internal_request = self._install_sequence_capture_plan(
                        sequence,
                        final_waypoint,
                        request.arrivalTimeoutMs,
                        request.captureProfile,
                    )
                    capture_result = self.execute_and_capture(internal_request, camera)
                except Exception as error:
                    known_predispatch = isinstance(error, HTTPException) or (
                        isinstance(error, ControllerCommandError)
                        and error.code
                        in {
                            "STOPPED",
                            "OPERATOR_INSPECTION_REQUIRED",
                            "SERVO_FAMILY_UNAVAILABLE",
                        }
                    )
                    if known_predispatch:
                        failed_row = self._sequence_revalidation_failure(
                            final_waypoint,
                            error,
                            capture_started,
                        )
                    else:
                        # Once the composed final operation is entered, an
                        # untyped exception cannot prove that no MOVE_SET acted.
                        # Treat that ambiguity as post-dispatch, STOP, and cache
                        # the terminal result so a retry cannot repeat motion.
                        _LOG.exception(
                            "Unknown final sequence capture failure at waypoint %s: %r",
                            final_index,
                            error,
                        )
                        failed_row = self._sequence_post_dispatch_failure(
                            final_waypoint,
                            capture_started,
                            reason="post_dispatch_capture_check_failed",
                        )
                    waypoint_results.append(failed_row)
                    if self._sequence_failure_requires_stop(failed_row):
                        self._stop_ambiguous_sequence_failure()
                    return finish(str(failed_row["outcome"]), final_index)
                arrival = capture_result.get("arrival")
                arrival_dict = arrival if isinstance(arrival, dict) else {}
                arrival_proved = arrival_dict.get("proved") is True
                motion_outcome = "arrived" if arrival_proved else str(
                    capture_result.get("outcome", "arrival_fault")
                )
                final_row: dict[str, object] = {
                    "index": final_index,
                    "outcome": motion_outcome,
                    "dispatched": capture_result.get("executed") is True,
                    "resolvedPose": final_waypoint.get("resolvedPose"),
                    "moved": capture_result.get("moved", []),
                    "arrival": arrival_dict,
                    "dispatchMs": capture_result.get("dispatchMs"),
                    "arrivalMs": int(arrival_dict.get("elapsedMs", 0)),
                    "durationMs": int(
                        (time.perf_counter() - capture_started) * 1000
                    ),
                }
                if "label" in final_waypoint:
                    final_row["label"] = final_waypoint["label"]
                waypoint_results.append(final_row)
                near_capture = capture_result.get("measuredPoseNearCapture")
                if isinstance(near_capture, dict):
                    final_measured = self._pose_from_state(
                        near_capture.get("after") or near_capture.get("before")
                    )
                if arrival_proved:
                    completed += 1
                capture_extra = {
                    name: capture_result[name]
                    for name in (
                        "captureAttempted",
                        "capture",
                        "measuredPoseNearCapture",
                        "inspectionRequired",
                        "motionFailure",
                        "captureUnavailableReason",
                        "captureInvalidationReason",
                        "viewPoseReason",
                    )
                    if name in capture_result
                }
                return finish(
                    str(capture_result.get("outcome", "arrival_fault")),
                    None if arrival_proved else final_index,
                    **capture_extra,
                )

            return finish("completed", None)

    @staticmethod
    def _boot_id_of(reported: dict[str, object]) -> str | None:
        identity = reported.get("identity")
        boot_id = identity.get("bootId") if isinstance(identity, dict) else None
        return boot_id if isinstance(boot_id, str) and boot_id else None

    def _next_telemetry_generation(
        self,
        *,
        after: int,
        stop_generation: int,
        deadline: float,
    ) -> tuple[int, float] | Literal["stopped", "timed_out"]:
        """Wait for service-loop evidence without issuing STATUS from HTTP."""

        with self._telemetry_condition:
            while (
                self._telemetry_generation <= after
                and self._stop_generation == stop_generation
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return "timed_out"
                self._telemetry_condition.wait(remaining)
            if self._stop_generation != stop_generation:
                return "stopped"
            if self._telemetry_generation <= after:
                return "timed_out"
            return self._telemetry_generation, self._telemetry_observed_at

    def _arrival_sample(
        self,
        *,
        generation: int,
        observed_at: float,
        moved: list[dict[str, object]],
        expected_boot_id: str,
        expected_configuration: str,
    ) -> tuple[bool, str | None, dict[str, object]]:
        """Evaluate one already-refreshed generation against the raw goals."""

        reported = self._controller.transport_state()
        snapshot = self.state()
        tolerance_evidence = self._arrival_tolerance_evidence(moved)
        with self._lock:
            current_generation = self._telemetry_generation
        if current_generation != generation:
            # A newer STATUS landed while this snapshot was being assembled.
            # Do not mix its state with the older generation marker.
            return False, None, {"superseded": True}

        detail: dict[str, object] = {
            "generation": generation,
            "observedMonotonic": observed_at,
            "state": snapshot,
            **tolerance_evidence,
        }
        if self._boot_id_of(reported) != expected_boot_id:
            detail["reason"] = "controller_boot_changed"
            return False, "arrival_fault", detail
        if self._configuration_digest() != expected_configuration:
            detail["reason"] = "arm_configuration_changed"
            return False, "arrival_fault", detail
        if reported.get("connection") != "online" or reported.get("bus") != "online":
            detail["reason"] = "controller_or_bus_unhealthy"
            return False, "arrival_fault", detail
        if reported.get("motionState") == "stopped":
            detail["reason"] = "stop_latched"
            return False, "stopped", detail
        if reported.get("hardwareEstop") == "active":
            detail["reason"] = "hardware_estop_active"
            return False, "stopped", detail
        if _collision_of(reported.get("lastScan"))["collisionSuspected"]:
            detail["reason"] = "servo_id_collision"
            return False, "arrival_fault", detail
        if reported.get("motionState") not in {"ready", "moving"}:
            detail["reason"] = "controller_motion_state_unhealthy"
            return False, "arrival_fault", detail
        if not self._fresh_age(reported.get("heartbeatAgeMs"), PLAN_MAX_HEARTBEAT_AGE_MS):
            detail["reason"] = "controller_heartbeat_stale"
            return False, "arrival_fault", detail

        rows = snapshot.get("joints")
        by_name = {
            row.get("id"): row
            for row in rows
            if isinstance(rows, list) and isinstance(row, dict)
        } if isinstance(rows, list) else {}
        measured_pose: dict[str, float] = {}
        measured_raw: dict[str, int] = {}
        target_pose: dict[str, float] = {}
        errors: dict[str, float] = {}
        telemetry_age = snapshot.get("telemetryAgeMs")
        for command in moved:
            name = command.get("joint")
            goal = command.get("goal")
            if name not in JOINT_IDS or isinstance(goal, bool) or not isinstance(goal, int):
                detail["reason"] = "executed_goal_shape_invalid"
                return False, "arrival_fault", detail
            row = by_name.get(name)
            if not isinstance(row, dict) or row.get("online") is not True:
                detail["reason"] = f"{name}_offline"
                return False, "arrival_fault", detail
            packet_age = row.get("packetAgeMs")
            if (
                not self._fresh_age(packet_age, PLAN_MAX_SERVO_PACKET_AGE_MS)
                or not self._fresh_age(telemetry_age, PLAN_MAX_SERVO_PACKET_AGE_MS)
                or float(packet_age) + float(telemetry_age)
                > PLAN_MAX_SERVO_PACKET_AGE_MS
            ):
                detail["reason"] = f"{name}_telemetry_stale"
                return False, None, detail
            if row.get("positionTrusted") is not True:
                detail["reason"] = f"{name}_position_untrusted"
                return False, "arrival_fault", detail
            if name == "joint_1":
                truth = row.get("multiTurnTruth")
                sample_age = truth.get("sampleAgeMs") if isinstance(truth, dict) else None
                if (
                    not isinstance(truth, dict)
                    or truth.get("valid") is not True
                    or truth.get("stepMode") is not False
                ):
                    detail["reason"] = "joint_1_odometer_untrusted"
                    return False, "arrival_fault", detail
                if not self._fresh_age(
                    sample_age, PLAN_MAX_ODOMETER_SAMPLE_AGE_MS
                ):
                    # A valid Base frame can temporarily outlive its latest
                    # wire sample while firmware 2.4 services guarded STATUS,
                    # ODO_READ, and legacy move traffic. This is missing fresh
                    # arrival evidence, not proof that the reference was lost.
                    # Keep waiting for a later generation within the caller's
                    # bounded deadline; genuinely invalid truth above remains
                    # a terminal fault.
                    detail["reason"] = "joint_1_odometer_stale"
                    return False, None, detail
            raw = row.get("rawPosition")
            degrees = row.get("degrees")
            if (
                isinstance(raw, bool)
                or not isinstance(raw, int)
                or isinstance(degrees, bool)
                or not isinstance(degrees, (int, float))
            ):
                detail["reason"] = f"{name}_measurement_missing"
                return False, None, detail
            joint = self._store.get(str(name))
            tolerance_by_joint = tolerance_evidence["toleranceByJoint"]
            assert isinstance(tolerance_by_joint, dict)
            tolerance = tolerance_by_joint.get(str(name))
            assert isinstance(tolerance, dict)
            threshold = tolerance["ticks"]
            assert isinstance(threshold, int)
            measured_raw[str(name)] = raw
            measured_pose[str(name)] = round(float(degrees), 6)
            target_degrees = command.get("degrees")
            if isinstance(target_degrees, (int, float)) and not isinstance(target_degrees, bool):
                target_pose[str(name)] = round(float(target_degrees), 6)
            errors[str(name)] = round((raw - goal) / joint.ticks_per_degree, 6)
            if abs(raw - goal) > threshold or row.get("moving") is not False:
                detail.update(
                    {
                        "targets": target_pose,
                        "measuredPose": measured_pose,
                        "measuredRaw": measured_raw,
                        "errorDegrees": errors,
                        "reason": f"{name}_still_moving_or_outside_tolerance",
                    }
                )
                return False, None, detail
        if reported.get("motionState") != "ready":
            detail["reason"] = "controller_still_moving"
            return False, None, detail
        detail.update(
            {
                "targets": target_pose,
                "measuredPose": measured_pose,
                "measuredRaw": measured_raw,
                "errorDegrees": errors,
                "reason": None,
            }
        )
        return True, None, detail

    def _arrival_tolerance_evidence(
        self, moved: list[dict[str, object]]
    ) -> dict[str, object]:
        """Describe the exact per-joint thresholds used by arrival and chase."""

        tolerance_by_joint: dict[str, dict[str, float | int]] = {}
        for command in moved:
            name = command.get("joint")
            if name not in JOINT_IDS:
                continue
            joint_name = str(name)
            joint = self._store.get(joint_name)
            tolerance_by_joint[joint_name] = {
                "degrees": _arrival_tolerance_degrees(joint_name),
                "ticks": _arrival_tolerance_ticks(
                    joint_name, joint.ticks_per_degree
                ),
            }
        distinct_degrees = {
            float(tolerance["degrees"])
            for tolerance in tolerance_by_joint.values()
        }
        # Preserve the legacy scalar when every commanded joint uses one value.
        # A mixed Camera/arm move is explicitly per-joint instead of presenting
        # either tolerance as if it applied to the whole move.
        tolerance_degrees: float | None = (
            next(iter(distinct_degrees))
            if len(distinct_degrees) == 1
            else ARRIVAL_DEGREES
            if not distinct_degrees
            else None
        )
        return {
            "toleranceDegrees": tolerance_degrees,
            "toleranceByJoint": tolerance_by_joint,
        }

    def _wait_for_arrival(
        self,
        *,
        moved: list[dict[str, object]],
        expected_boot_id: str,
        expected_configuration: str,
        after_generation: int,
        stop_generation: int,
        timeout_seconds: float,
        sample_count: int = ARRIVAL_SAMPLE_COUNT,
        stable_seconds: float = ARRIVAL_STABLE_SECONDS,
    ) -> tuple[str, dict[str, object], dict[str, object] | None]:
        started = time.monotonic()
        deadline = started + timeout_seconds
        tolerance_evidence = self._arrival_tolerance_evidence(moved)
        last_generation = after_generation
        stable: list[tuple[float, dict[str, object]]] = []
        latest: dict[str, object] | None = None
        while True:
            marker = self._next_telemetry_generation(
                after=last_generation,
                stop_generation=stop_generation,
                deadline=deadline,
            )
            if marker == "stopped":
                return (
                    "stopped",
                    {
                        "proved": False,
                        "elapsedMs": int((time.monotonic() - started) * 1000),
                        **tolerance_evidence,
                        "sampleCount": len(stable),
                        "samples": len(stable),
                        "reason": "operator_stop",
                    },
                    latest,
                )
            if marker == "timed_out":
                arrival = {
                    "proved": False,
                    "elapsedMs": int((time.monotonic() - started) * 1000),
                    **tolerance_evidence,
                    "sampleCount": len(stable),
                    "samples": len(stable),
                    "reason": "arrival_timeout",
                }
                if latest is not None:
                    for key in ("targets", "measuredPose", "errorDegrees"):
                        if key in latest:
                            arrival[key] = latest[key]
                    arrival["lastSampleReason"] = latest.get("reason")
                return "arrival_timeout", arrival, latest
            generation, observed_at = marker
            last_generation = generation
            qualifies, terminal, sample = self._arrival_sample(
                generation=generation,
                observed_at=observed_at,
                moved=moved,
                expected_boot_id=expected_boot_id,
                expected_configuration=expected_configuration,
            )
            if sample.get("superseded"):
                continue
            latest = sample
            if terminal is not None:
                arrival = {
                    "proved": False,
                    "elapsedMs": int((time.monotonic() - started) * 1000),
                    **tolerance_evidence,
                    "sampleCount": len(stable),
                    "samples": len(stable),
                    "reason": sample.get("reason"),
                }
                for key in ("targets", "measuredPose", "errorDegrees"):
                    if key in sample:
                        arrival[key] = sample[key]
                return terminal, arrival, latest
            if not qualifies:
                stable.clear()
                continue
            stable.append((observed_at, sample))
            span = observed_at - stable[0][0]
            if len(stable) < sample_count or span + 1e-9 < stable_seconds:
                continue
            arrival = {
                "proved": True,
                "elapsedMs": int((time.monotonic() - started) * 1000),
                **tolerance_evidence,
                "stableForMs": int(span * 1000),
                "spanMs": int(span * 1000),
                "sampleCount": len(stable),
                "samples": len(stable),
                "targets": sample.get("targets", {}),
                "measuredPose": sample.get("measuredPose", {}),
                "errorDegrees": sample.get("errorDegrees", {}),
                "bootId": expected_boot_id,
                "telemetryGeneration": generation,
            }
            return "arrived", arrival, sample

    def _trusted_view_pose(
        self, snapshot: dict[str, object]
    ) -> tuple[dict[str, object] | None, str | None]:
        """Require fresh, stationary pose truth for every camera-defining joint."""

        telemetry_generation = snapshot.get("telemetryGeneration")
        telemetry_age = snapshot.get("telemetryAgeMs")
        if (
            isinstance(telemetry_generation, bool)
            or not isinstance(telemetry_generation, int)
            or telemetry_generation <= 0
            or not self._fresh_age(telemetry_age, PLAN_MAX_SERVO_PACKET_AGE_MS)
        ):
            return None, "view_pose_generation_stale"
        rows = snapshot.get("joints")
        by_name = (
            {
                row.get("id"): row
                for row in rows
                if isinstance(row, dict)
            }
            if isinstance(rows, list)
            else {}
        )
        raw_pose: dict[str, int] = {}
        degrees_pose: dict[str, float] = {}
        for name in JOINT_IDS:
            row = by_name.get(name)
            if not isinstance(row, dict) or row.get("online") is not True:
                return None, f"{name}_view_pose_offline"
            if row.get("positionTrusted") is not True:
                return None, f"{name}_view_pose_untrusted"
            if row.get("moving") is not False:
                return None, f"{name}_view_pose_moving"
            packet_age = row.get("packetAgeMs")
            if (
                not self._fresh_age(packet_age, PLAN_MAX_SERVO_PACKET_AGE_MS)
                or float(packet_age) + float(telemetry_age)
                > PLAN_MAX_SERVO_PACKET_AGE_MS
            ):
                return None, f"{name}_view_pose_stale"
            raw = row.get("rawPosition")
            degrees = row.get("degrees")
            if (
                isinstance(raw, bool)
                or not isinstance(raw, int)
                or isinstance(degrees, bool)
                or not isinstance(degrees, (int, float))
            ):
                return None, f"{name}_view_pose_missing"
            if name == "joint_1":
                truth = row.get("multiTurnTruth")
                sample_age = truth.get("sampleAgeMs") if isinstance(truth, dict) else None
                if (
                    not isinstance(truth, dict)
                    or truth.get("valid") is not True
                    or truth.get("stepMode") is not False
                    or not self._fresh_age(
                        sample_age, PLAN_MAX_ODOMETER_SAMPLE_AGE_MS
                    )
                ):
                    return None, "joint_1_view_pose_odometer_untrusted"
            raw_pose[name] = raw
            degrees_pose[name] = round(float(degrees), 6)
        return (
            {
                "telemetryGeneration": telemetry_generation,
                "telemetryAgeMs": telemetry_age,
                "raw": raw_pose,
                "degrees": degrees_pose,
            },
            None,
        )

    def _cache_observation_result(
        self, key: tuple[str, str], result: dict[str, object]
    ) -> dict[str, object]:
        with self._lock:
            self._observation_results[key] = deepcopy(result)
            self._observation_results.move_to_end(key)
            while len(self._observation_results) > OBSERVATION_RESULT_CACHE_LIMIT:
                self._observation_results.popitem(last=False)
        return result

    def execute_and_capture(
        self,
        request: PlanExecuteCaptureRequest,
        camera: CameraEvidenceService | None,
    ) -> dict[str, object]:
        """Execute, prove measured arrival, then return one pose-bound frame."""

        key = (request.planId, request.planDigest)
        with self._operation_lock:
            with self._lock:
                cached = self._observation_results.get(key)
            if cached is not None:
                replay = deepcopy(cached)
                if replay.get("outcome") == "captured":
                    capture = replay.get("capture")
                    transfer_token = (
                        capture.get("transferToken")
                        if isinstance(capture, dict)
                        else None
                    )
                    if (
                        camera is None
                        or not isinstance(transfer_token, str)
                        or not camera.has_transfer(transfer_token)
                    ):
                        replay["outcome"] = "capture_unavailable"
                        replay["capture"] = None
                        replay["captureUnavailableReason"] = (
                            "exact_frame_transfer_unavailable"
                        )
                        return self._cache_observation_result(key, replay)
                return replay
            if camera is None or camera.status().get("available") is not True:
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail="Camera is unavailable; the reviewed plan was not executed.",
                )
            if not camera.transfer_available():
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail=(
                        "Exact camera-transfer capacity is unavailable; "
                        "the reviewed plan was not executed."
                    ),
                )

            reported_before_execution = self._controller.transport_state()
            with self._lock:
                inspection_before_execution = self._inspection_required
            preexisting_inspection = (
                inspection_before_execution
                or self._reported_clear_required_stop(reported_before_execution)
            )
            if preexisting_inspection:
                raise ControllerCommandError("OPERATOR_INSPECTION_REQUIRED")
            # Even an arm-only observation binds the image to Camera telemetry.
            # Without the negotiated FAMILY command the HAT defaults that SCS
            # servo to the STS dialect, so neither its pose nor a later camera
            # move is trustworthy. Refuse before consuming the reviewed plan.
            self._require_servo_family_ready(reported_before_execution)

            # Bind the composed operation to the identity/configuration that
            # execute_plan is about to revalidate. Capturing these *after*
            # dispatch would leave a tiny reboot window in which a new boot
            # could be mistaken for the one that accepted the reviewed move.
            expected_configuration = self._configuration_digest()
            expected_boot_id = self._boot_id_of(reported_before_execution)
            operation_id = "armobs_" + secrets.token_urlsafe(12)
            dispatch_started = time.perf_counter()
            try:
                execution = self.execute_plan(
                    PlanExecuteRequest(
                        planId=request.planId, planDigest=request.planDigest
                    )
                )
            except Exception as error:
                dispatch_ms = round(
                    (time.perf_counter() - dispatch_started) * 1000,
                    6,
                )
                reported_failure = self._controller.transport_state()
                with self._lock:
                    inspection_after_execution = self._inspection_required
                inspection_required = (
                    not preexisting_inspection
                    and (
                        inspection_after_execution
                        or reported_failure.get("operatorInspectionRequired") is True
                    )
                    and reported_failure.get("safetyStopReason")
                    not in AUTOMATIC_STOP_REASONS
                )
                if not inspection_required:
                    raise
                failure_payload = (
                    error.payload
                    if isinstance(error, ControllerCommandError)
                    else reported_failure.get("lastMotionFailure")
                )
                return self._cache_observation_result(
                    key,
                    {
                        "operationId": operation_id,
                        "planId": request.planId,
                        "executed": True,
                        "outcome": "stopped",
                        "resolvedPose": None,
                        "moved": [],
                        "captureAttempted": False,
                        "capture": None,
                        "dispatchMs": dispatch_ms,
                        "measuredPoseNearCapture": None,
                        "inspectionRequired": True,
                        "motionFailure": _safe_motion_failure(failure_payload),
                        "arrival": {
                            "proved": False,
                            "toleranceDegrees": ARRIVAL_DEGREES,
                            "sampleCount": 0,
                            "samples": 0,
                            "reason": (
                                "legacy_move_unconfirmed_inspection_required"
                                if isinstance(
                                    error,
                                    (ControllerTransportError, ControllerProtocolError),
                                )
                                else "move_set_failed_inspection_required"
                            ),
                        },
                    },
                )
            dispatch_ms = round(
                (time.perf_counter() - dispatch_started) * 1000,
                6,
            )
            moved_value = execution.get("moved")
            moved = (
                [dict(value) for value in moved_value if isinstance(value, dict)]
                if isinstance(moved_value, list)
                else []
            )
            with self._lock:
                dispatch_generation = self._telemetry_generation
                stop_generation = self._stop_generation

            base: dict[str, object] = {
                "operationId": operation_id,
                "planId": request.planId,
                "executed": True,
                "resolvedPose": execution.get("resolvedPose"),
                "moved": moved,
                "dispatchMs": dispatch_ms,
                "captureAttempted": False,
                "capture": None,
                "measuredPoseNearCapture": None,
            }
            capture_attempted = False
            captured_transfer_token: str | None = None
            if expected_boot_id is None or not moved:
                return self._cache_observation_result(
                    key,
                    {
                        **base,
                        "outcome": "arrival_fault",
                        "arrival": {
                            "proved": False,
                            "toleranceDegrees": ARRIVAL_DEGREES,
                            "sampleCount": 0,
                            "samples": 0,
                            "reason": "execution_evidence_incomplete",
                        },
                    },
                )

            try:
                outcome, arrival, before_sample = self._wait_for_arrival(
                    moved=moved,
                    expected_boot_id=expected_boot_id,
                    expected_configuration=expected_configuration,
                    after_generation=dispatch_generation,
                    stop_generation=stop_generation,
                    timeout_seconds=request.arrivalTimeoutMs / 1000.0,
                )
                if outcome != "arrived" or before_sample is None:
                    return self._cache_observation_result(
                        key,
                        {**base, "outcome": outcome, "arrival": arrival},
                    )

                before_state = before_sample.get("state")
                before_generation = before_sample.get("generation")
                if not isinstance(before_state, dict) or not isinstance(before_generation, int):
                    raise RuntimeError("arrival state was incomplete")
                before_view_pose, before_view_reason = self._trusted_view_pose(
                    before_state
                )
                if before_view_pose is None:
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "view_pose_untrusted",
                            "arrival": arrival,
                            "viewPoseReason": before_view_reason,
                            "measuredPoseNearCapture": {
                                "before": before_state,
                                "after": None,
                            },
                        },
                    )
                def shutter_is_blocked() -> bool:
                    with self._telemetry_condition:
                        stopped_locally = (
                            self._stop_generation != stop_generation
                            or self._operator_stopped
                            or self._inspection_required
                        )
                    reported = self._controller.transport_state()
                    return (
                        stopped_locally
                        or reported.get("motionState") == "stopped"
                        or reported.get("hardwareEstop") == "active"
                        or self._reported_clear_required_stop(reported)
                    )

                shutter_authorized = False

                @contextmanager
                def authorize_shutter():
                    nonlocal shutter_authorized
                    # This gate is entered by CameraEvidenceService only after
                    # its camera lock and capacity checks. STOP therefore wins
                    # while a composed request is queued behind ordinary
                    # camera I/O; once this gate is held, shutter start wins and
                    # STOP still reaches the controller without waiting.
                    with self._capture_gate:
                        if shutter_is_blocked():
                            raise CameraEvidenceCaptureCancelledError(
                                "Camera capture was cancelled before shutter start."
                            )
                        shutter_authorized = True
                        yield

                def stopped_before_shutter_result() -> dict[str, object]:
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "stopped",
                            "arrival": {
                                **arrival,
                                "proved": False,
                                "reason": "operator_stop_before_shutter",
                            },
                            "captureAttempted": False,
                            "measuredPoseNearCapture": {
                                "before": before_state,
                                "after": None,
                            },
                        },
                    )

                if shutter_is_blocked():
                    return stopped_before_shutter_result()

                capture_attempted = False
                try:
                    capture = camera.capture_transfer(
                        profile=request.captureProfile,
                        shutter_guard=authorize_shutter()
                    )
                    capture_attempted = shutter_authorized
                except CameraEvidenceCaptureCancelledError:
                    return stopped_before_shutter_result()
                except CameraUnavailableError:
                    capture_attempted = shutter_authorized
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "camera_unavailable",
                            "arrival": arrival,
                            "captureAttempted": capture_attempted,
                            "measuredPoseNearCapture": {"before": before_state, "after": None},
                        },
                    )
                except CameraEvidenceCaptureError:
                    capture_attempted = shutter_authorized
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "capture_failed",
                            "arrival": arrival,
                            "captureAttempted": capture_attempted,
                            "measuredPoseNearCapture": {"before": before_state, "after": None},
                        },
                    )
                except CameraEvidenceTransferUnavailableError:
                    capture_attempted = shutter_authorized
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "capture_unavailable",
                            "arrival": arrival,
                            "captureAttempted": capture_attempted,
                            "captureUnavailableReason": "transfer_capacity_unavailable",
                            "measuredPoseNearCapture": {
                                "before": before_state,
                                "after": None,
                            },
                        },
                    )
                transfer_token_value = capture.get("transferToken")
                captured_transfer_token = (
                    transfer_token_value
                    if isinstance(transfer_token_value, str)
                    else None
                )
                captured_monotonic = (
                    camera.transfer_captured_monotonic(captured_transfer_token)
                    if captured_transfer_token is not None
                    else None
                )

                post_outcome, _, after_sample = self._wait_for_arrival(
                    moved=moved,
                    expected_boot_id=expected_boot_id,
                    expected_configuration=expected_configuration,
                    after_generation=before_generation,
                    stop_generation=stop_generation,
                    timeout_seconds=max(0.5, min(2.0, request.arrivalTimeoutMs / 1000.0)),
                    sample_count=1,
                    stable_seconds=0.0,
                )
                after_state = after_sample.get("state") if isinstance(after_sample, dict) else None
                before_observed = before_sample.get("observedMonotonic")
                after_observed = (
                    after_sample.get("observedMonotonic")
                    if isinstance(after_sample, dict)
                    else None
                )
                after_view_pose, after_view_reason = (
                    self._trusted_view_pose(after_state)
                    if isinstance(after_state, dict)
                    else (None, "post_capture_view_pose_missing")
                )
                drift_ok = after_view_pose is not None
                capture_bracketed = (
                    isinstance(before_observed, (int, float))
                    and not isinstance(before_observed, bool)
                    and isinstance(after_observed, (int, float))
                    and not isinstance(after_observed, bool)
                    and isinstance(captured_monotonic, (int, float))
                    and not isinstance(captured_monotonic, bool)
                    and math.isfinite(float(before_observed))
                    and math.isfinite(float(after_observed))
                    and math.isfinite(float(captured_monotonic))
                    and float(before_observed)
                    <= float(captured_monotonic)
                    <= float(after_observed)
                )
                before_raw = before_view_pose["raw"]
                after_raw = (
                    after_view_pose.get("raw")
                    if isinstance(after_view_pose, dict)
                    else None
                )
                if not isinstance(before_raw, dict) or not isinstance(after_raw, dict):
                    drift_ok = False
                # Arrival is judged only for commanded joints. The photograph,
                # however, is bound to all four joints that define its view.
                for name in JOINT_IDS if drift_ok else ():
                    first = before_raw.get(name)
                    second = after_raw.get(name)
                    joint = self._store.get(name)
                    threshold = max(1, round(ARRIVAL_DEGREES * joint.ticks_per_degree))
                    if (
                        not isinstance(first, int)
                        or isinstance(first, bool)
                        or not isinstance(second, int)
                        or isinstance(second, bool)
                        or abs(first - second) > threshold
                    ):
                        drift_ok = False
                        break
                if (
                    post_outcome != "arrived"
                    or not isinstance(after_state, dict)
                    or not drift_ok
                    or not capture_bracketed
                ):
                    if captured_transfer_token is not None:
                        camera.discard_transfer(captured_transfer_token)
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "capture_invalidated",
                            "arrival": arrival,
                            "captureAttempted": True,
                            "captureInvalidationReason": (
                                after_view_reason
                                or (
                                    "capture_timestamp_outside_pose_bracket"
                                    if not capture_bracketed
                                    else "view_pose_drifted_during_capture"
                                )
                            ),
                            "measuredPoseNearCapture": {
                                "before": before_state,
                                "after": after_state,
                            },
                        },
                    )
                if (
                    captured_transfer_token is None
                    or not camera.has_transfer(captured_transfer_token)
                ):
                    return self._cache_observation_result(
                        key,
                        {
                            **base,
                            "outcome": "capture_unavailable",
                            "arrival": arrival,
                            "captureAttempted": True,
                            "captureUnavailableReason": (
                                "exact_frame_transfer_unavailable"
                            ),
                            "measuredPoseNearCapture": {
                                "before": before_state,
                                "after": after_state,
                            },
                        },
                    )
                return self._cache_observation_result(
                    key,
                    {
                        **base,
                        "outcome": "captured",
                        "arrival": arrival,
                        "captureAttempted": True,
                        "capture": capture,
                        "measuredPoseNearCapture": {
                            "before": before_state,
                            "after": after_state,
                        },
                    },
                )
            except Exception as error:
                _LOG.exception("Post-dispatch arm observation failed: %r", error)
                if captured_transfer_token is not None:
                    camera.discard_transfer(captured_transfer_token)
                return self._cache_observation_result(
                    key,
                    {
                        **base,
                        "outcome": (
                            "capture_invalidated"
                            if captured_transfer_token
                            else "arrival_fault"
                        ),
                        "captureAttempted": capture_attempted,
                        "arrival": {
                            "proved": False,
                            "toleranceDegrees": ARRIVAL_DEGREES,
                            "sampleCount": 0,
                            "samples": 0,
                            "reason": "post_dispatch_observation_failed",
                        },
                    },
                )

    def _clear_unwanted_stop(self) -> bool:
        """Retry only after the same complete proof used by background recovery."""

        return self._recover_stop()

    def _retake_hold(self) -> bool:
        """Re-send the hold set after the controller's lease lapsed underneath us.

        `_ensure_held` skips HOLD_SET whenever its own bookkeeping already lists
        every servo asked for, but the controller's lease only lasts
        HOLD_LEASE_MS. Any pause longer than that -- a settle, an operator
        thinking, an agent reading telemetry between moves -- leaves the Pi
        believing it holds authority the controller has already dropped, and the
        next MOVE comes back NO_TORQUE_LEASE even though nothing is wrong.
        Measured on the bench: two of four multi-joint moves were refused this
        way, with the joint still sitting where the last command left it.

        Re-sending the set the Pi already intends to hold costs one frame and
        asks for nothing new, so it is safe to do on the refusal itself.
        """

        with self._lock:
            wanted = list(self._held)
        if not wanted:
            return False
        try:
            self._apply_hold(wanted)
            return True
        except Exception:
            return False

    def _without_refusal(self, action):
        """Retry once past an expired torque lease, never past a STOP.

        A lapsed lease is the controller and Pi disagreeing about authority and
        can be restored with the same absolute hold set. A legacy automatic STOP
        is reset only by the recovery helper after complete all-off proof.

        Retrying is safe because the actions wrapped here are idempotent -- a
        hold set and a MOVE both name an absolute end state, so repeating one
        asks for exactly what was already asked for.

        STOP is never cleared here, so it still refuses and still says so.
        """

        reported = self._controller.transport_state()
        with self._lock:
            if (
                self._operator_stopped
                or self._inspection_required
                or reported.get("motionState") == "stopped"
                or self._reported_clear_required_stop(reported)
            ):
                raise ControllerCommandError("STOPPED")
        try:
            return action()
        except ControllerCommandError as error:
            code = str(getattr(error, "code", ""))
            if code == "STOPPED":
                if not self._clear_unwanted_stop():
                    raise
            elif code == "NO_TORQUE_LEASE":
                if not self._retake_hold():
                    raise
            else:
                raise
            return action()

    # ---- bounded Shoulder/Elbow Fast Follow -------------------------------

    @staticmethod
    def _live_follow_motion_health(reported: Mapping[str, object]) -> str:
        """Require live motion health without rejecting `moving` itself."""

        if reported.get("connection") != "online":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller link is not online; Fast Follow cannot continue.",
            )
        if reported.get("bus") != "online":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The servo bus is not online; Fast Follow cannot continue.",
            )
        if (
            reported.get("safetyFault") is True
            or reported.get("operatorInspectionRequired") is True
            or reported.get("inspectionRequired") is True
        ):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="A controller safety fault or inspection block ended Fast Follow.",
            )
        # Firmware 2.7 reports `blocked` while automatic safety recovery is
        # pending. It is not a harmless idle state and must never retain live
        # authority. Only genuinely ready or currently moving states are valid.
        if reported.get("motionState") not in {"ready", "moving"}:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="STOP or a controller fault ended Fast Follow.",
            )
        if reported.get("hardwareEstop") == "active":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The hardware E-stop ended Fast Follow.",
            )
        identity = reported.get("identity")
        boot_id = identity.get("bootId") if isinstance(identity, dict) else None
        if not isinstance(boot_id, str) or not boot_id:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="The controller boot identity is unavailable.",
            )
        return boot_id

    def _live_follow_measurement(
        self,
        rows: list[dict[str, object]],
        *,
        previous_raw: Mapping[str, int] | None,
        previous_at: float | None,
        observed_at: float,
    ) -> tuple[dict[str, dict[str, object]], dict[str, int]]:
        """Validate compact firmware feedback and derive joint velocity."""

        by_id: dict[int, dict[str, object]] = {}
        for row in rows:
            servo_id = row.get("servoId")
            if (
                isinstance(servo_id, int)
                and not isinstance(servo_id, bool)
                and servo_id not in by_id
            ):
                by_id[servo_id] = row
        expected_ids = {
            self._store.get(name).servoId for name in LIVE_FOLLOW_JOINTS
        }
        if set(by_id) != expected_ids:
            raise ControllerProtocolError(
                "Fast Follow feedback did not cover exactly Shoulder and Elbow."
            )

        measured: dict[str, dict[str, object]] = {}
        latest_raw: dict[str, int] = {}
        elapsed = (
            observed_at - previous_at
            if isinstance(previous_at, (int, float)) and observed_at > previous_at
            else None
        )
        for name in LIVE_FOLLOW_JOINTS:
            joint = self._store.get(name)
            row = by_id[joint.servoId]
            raw = row.get("rawPosition")
            packet_age = row.get("packetAgeMs")
            moving = row.get("moving")
            if (
                isinstance(raw, bool)
                or not isinstance(raw, int)
                or not joint.raw_min <= raw <= joint.raw_max
                or not self._fresh_age(packet_age, PLAN_MAX_SERVO_PACKET_AGE_MS)
                or not isinstance(moving, bool)
            ):
                raise ControllerProtocolError(
                    "Fast Follow feedback was incomplete or stale."
                )
            degrees = joint.degrees_at(raw)
            if degrees is None:
                raise ControllerProtocolError(
                    "Fast Follow feedback cannot be converted without calibration."
                )
            velocity = 0.0
            before = previous_raw.get(name) if previous_raw is not None else None
            if isinstance(before, int) and elapsed is not None and elapsed > 0:
                before_degrees = joint.degrees_at(before)
                if before_degrees is not None:
                    velocity = (float(degrees) - float(before_degrees)) / elapsed
            measured[name] = {
                "degrees": round(float(degrees), 3),
                "rawPosition": raw,
                "velocityDegreesPerSecond": round(velocity, 3),
                "moving": moving,
                "packetAgeMs": int(float(packet_age)),
            }
            latest_raw[name] = raw
        return measured, latest_raw

    @staticmethod
    def _live_follow_response_locked(session: Mapping[str, object]) -> dict[str, object]:
        response: dict[str, object] = {
            "schema": session.get("schema", LIVE_FOLLOW_SCHEMA),
            "sessionId": session["sessionId"],
            "state": session["state"],
            "reason": session.get("reason"),
            "settings": deepcopy(session["settings"]),
            "envelope": deepcopy(session["envelope"]),
            "commanded": deepcopy(session["commanded"]),
            "measured": deepcopy(session["measured"]),
            "stats": deepcopy(session["stats"]),
            "timing": deepcopy(session["timing"]),
        }
        fault = session.get("fault")
        if isinstance(fault, Mapping):
            response["fault"] = deepcopy(dict(fault))
        start_attempt_id = session.get("startAttemptId")
        if isinstance(start_attempt_id, str):
            response["startAttemptId"] = start_attempt_id
        return response

    def _live_follow_response(self, session: Mapping[str, object]) -> dict[str, object]:
        with self._lock:
            return self._live_follow_response_locked(session)

    def _live_follow_session(self, session_id: str) -> dict[str, object]:
        with self._lock:
            session = self._live_follow
            if session is not None and session.get("sessionId") == session_id:
                return session
            terminal = self._last_live_follow
            if terminal is not None and terminal.get("sessionId") == session_id:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=self._live_follow_response_locked(terminal),
                )
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="That Fast Follow session is not active.",
        )

    def _finish_live_follow(
        self,
        session_id: str,
        *,
        state_name: Literal["ended", "expired", "faulted", "stopped"],
        reason: str,
        release: bool,
        fault: Mapping[str, object] | None = None,
        input_expiry_cutoff: float | None = None,
    ) -> dict[str, object] | None:
        # A releasing finish owns the motion and authority suffix of the global
        # operation -> motion -> authority -> state order before it removes the
        # active session, and keeps both through the matching empty HOLD_SET.
        # Without that one boundary, either a pending frame can dispatch while
        # expiry is being committed or a new start can install its hold before
        # the old finisher reaches the controller and then be revoked by it.
        motion_acquired = False
        authority_acquired = False
        try:
            if release:
                self._motion_lock.acquire()
                motion_acquired = True
                self._authority_lock.acquire()
                authority_acquired = True
            with self._telemetry_condition:
                session = self._live_follow
                if session is None or session.get("sessionId") != session_id:
                    return None
                if input_expiry_cutoff is not None:
                    # The heartbeat endpoint intentionally bypasses global physical
                    # admission. Revalidate its lease timestamp under the same lock
                    # that converts the active session into a terminal receipt.
                    # Otherwise a service tick can expire a heartbeat that landed
                    # after the tick's initial snapshot but before this boundary.
                    last_input = session.get("lastInputMonotonic")
                    if (
                        isinstance(last_input, (int, float))
                        and not isinstance(last_input, bool)
                        and float(last_input) > input_expiry_cutoff
                    ):
                        return None
                session["state"] = state_name
                session["reason"] = reason
                if fault is not None:
                    session["fault"] = deepcopy(dict(fault))
                session["pending"] = None
                terminal = self._live_follow_response_locked(session)
                self._last_live_follow = terminal
                self._live_follow = None
                if release:
                    self._held = []
                    self._goals.clear()
                    self._last_hold = time.monotonic()
                    self._stop_generation += 1
                self._telemetry_condition.notify_all()
            if release:
                try:
                    self._controller.set_hold_servos([], HOLD_LEASE_MS)
                except Exception:
                    # The 750 ms watchdog and 2 s hold lease still revoke
                    # authority. Never retry an expired/faulted follow command.
                    pass
            return terminal
        finally:
            if authority_acquired:
                self._authority_lock.release()
            if motion_acquired:
                self._motion_lock.release()

    def start_live_follow(self, request: LiveFollowStartRequest) -> dict[str, object]:
        start_attempt_id = request.startAttemptId
        start_deadline = time.monotonic() + (request.startTimeoutMs / 1000.0)
        try:
            return self._start_live_follow_attempt(
                request,
                start_attempt_id=start_attempt_id,
                start_deadline=start_deadline,
            )
        except Exception:
            with self._telemetry_condition:
                attempt_state = self._live_follow_start_attempts.get(start_attempt_id)
                if attempt_state == "cancel_requested":
                    self._live_follow_start_attempts[start_attempt_id] = "cancelled"
                elif attempt_state == "starting":
                    self._live_follow_start_attempts[start_attempt_id] = "failed"
                self._telemetry_condition.notify_all()
            raise

    def _start_live_follow_attempt(
        self,
        request: LiveFollowStartRequest,
        *,
        start_attempt_id: str,
        start_deadline: float,
    ) -> dict[str, object]:
        with self._operation_lock, self._motion_lock, self._authority_lock:
            with self._lock:
                for existing in (self._live_follow, self._last_live_follow):
                    if (
                        existing is not None
                        and existing.get("startAttemptId") == start_attempt_id
                    ):
                        return self._live_follow_response_locked(existing)
                if start_attempt_id in self._live_follow_start_attempts:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="That Fast Follow start attempt is no longer reusable.",
                    )
                self._live_follow_start_attempts[start_attempt_id] = "starting"
            self.require_live_follow_idle()
            self._require_no_outstanding_goals()
            with self._lock:
                if not self._floor_guard:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow requires the floor guard to stay on.",
                    )
                stop_generation = self._stop_generation

            follow_set = getattr(self._controller, "follow_set", None)
            follow_read = getattr(self._controller, "follow_read", None)
            reported = self._controller.transport_state()
            capabilities = self._reported_capabilities(reported)
            if (
                not callable(follow_set)
                or not callable(follow_read)
                or LIVE_FOLLOW_CAPABILITY not in capabilities
                or LIVE_FOLLOW_READ_CAPABILITY not in capabilities
            ):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "The connected HAT firmware does not provide bounded "
                        "Fast Follow feedback."
                    ),
                )

            boot_id, pose = self._planning_snapshot(set(LIVE_FOLLOW_JOINTS))
            reported = self._controller.transport_state()
            if self._live_follow_motion_health(reported) != boot_id:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="The controller boot changed while Fast Follow was starting.",
                )
            telemetry = self.telemetry_of(reported)
            servo_ids: list[int] = []
            feedback_rows: list[dict[str, object]] = []
            for name in LIVE_FOLLOW_JOINTS:
                joint = self._store.get(name)
                if (
                    joint.rawZero is None
                    or joint.rawMin is None
                    or joint.rawMax is None
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=(
                            f"{JOINT_NAMES[name]} needs calibrated zero, minimum, "
                            "and maximum limits before Fast Follow."
                        ),
                    )
                servo_ids.append(joint.servoId)
                row = telemetry.get(joint.servoId)
                if (
                    not isinstance(row, dict)
                    or row.get("operatingMode") != 0
                    or row.get("errors") != []
                    or not self._fresh_age(
                        row.get("packetAgeMs"), PLAN_MAX_SERVO_PACKET_AGE_MS
                    )
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=(
                            f"{JOINT_NAMES[name]} needs fresh, healthy mode-0 "
                            "telemetry before Fast Follow."
                        ),
                    )
                feedback_rows.append(
                    {
                        "servoId": joint.servoId,
                        "rawPosition": row.get("rawPosition"),
                        "moving": row.get("moving"),
                        "packetAgeMs": row.get("packetAgeMs"),
                    }
                )
            if len(set(servo_ids)) != len(LIVE_FOLLOW_JOINTS):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Shoulder and Elbow must use two distinct STS servo IDs.",
                )

            start_pose = {
                name: float(pose[name])
                for name in LIVE_FOLLOW_JOINTS
                if isinstance(pose.get(name), (int, float))
            }
            if len(start_pose) != len(LIVE_FOLLOW_JOINTS):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Fresh Shoulder and Elbow telemetry is required.",
                )
            for name in LIVE_FOLLOW_JOINTS:
                joint = self._store.get(name)
                logical_low, logical_high = joint.degree_bounds()
                reachable_low, reachable_high = joint.reach_bounds()
                if (
                    logical_low is None
                    or logical_high is None
                    or reachable_low is None
                    or reachable_high is None
                    or not logical_low - LIVE_FOLLOW_START_LIMIT_TOLERANCE_DEGREES
                    <= start_pose[name]
                    <= logical_high + LIVE_FOLLOW_START_LIMIT_TOLERANCE_DEGREES
                    or not reachable_low - LIVE_FOLLOW_START_LIMIT_TOLERANCE_DEGREES
                    <= start_pose[name]
                    <= reachable_high + LIVE_FOLLOW_START_LIMIT_TOLERANCE_DEGREES
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=(
                            f"{JOINT_NAMES[name]} is outside its calibrated "
                            "reachable limits."
                        ),
                    )
            observed_at = time.monotonic()
            measured, measured_raw = self._live_follow_measurement(
                feedback_rows,
                previous_raw=None,
                previous_at=None,
                observed_at=observed_at,
            )
            start_lowest = lowest_point_mm(
                start_pose["joint_2"], start_pose["joint_3"]
            )
            if start_lowest < FLOOR_MM - 1e-6:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        "Fast Follow cannot start while the measured arm is "
                        "below the 40 mm floor guard."
                    ),
                )

            # Fast Follow intentionally owns only Shoulder and Elbow. Replacing
            # rather than unioning the standing hold prevents Base/Camera from
            # being energised by this two-axis test mode.
            with self._lock:
                if self._live_follow_start_attempts.get(start_attempt_id) != "starting":
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow start attempt was cancelled.",
                    )
                if time.monotonic() >= start_deadline:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow start deadline expired.",
                    )
                if (
                    stop_generation != self._stop_generation
                    or self._operator_stopped
                    or self._inspection_required
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow lost controller continuity while starting.",
                    )
                self._held = list(servo_ids)
                self._goals.clear()
                self._last_hold = time.monotonic()
            committed = False
            try:
                self._apply_hold(list(servo_ids))
                with self._lock:
                    if (
                        self._live_follow_start_attempts.get(start_attempt_id)
                        != "starting"
                    ):
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Fast Follow start attempt was cancelled.",
                        )
                    if time.monotonic() >= start_deadline:
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Fast Follow start deadline expired.",
                        )
                    if (
                        stop_generation != self._stop_generation
                        or self._operator_stopped
                        or self._inspection_required
                    ):
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail=(
                                "Fast Follow lost controller continuity while "
                                "starting."
                            ),
                        )
                after = self._controller.transport_state()
                if (
                    self._live_follow_motion_health(after) != boot_id
                    or LIVE_FOLLOW_CAPABILITY
                    not in self._reported_capabilities(after)
                    or LIVE_FOLLOW_READ_CAPABILITY
                    not in self._reported_capabilities(after)
                ):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow lost controller continuity while starting.",
                    )

                started_monotonic = time.monotonic()
                started_epoch = time.time()
                session_id = "armfollow_" + secrets.token_urlsafe(18)
                settings = {
                    name: getattr(request, name).model_dump()
                    for name in LIVE_FOLLOW_JOINTS
                }
                session: dict[str, object] = {
                    "sessionId": session_id,
                    "startAttemptId": start_attempt_id,
                    "state": "active",
                    "reason": None,
                    "settings": settings,
                    "bootId": boot_id,
                    "configurationDigest": self._configuration_digest(),
                    "startedMonotonic": started_monotonic,
                    "lastInputMonotonic": started_monotonic,
                    "lastDispatchMonotonic": 0.0,
                    "lastFeedbackMonotonic": observed_at,
                    "consecutiveIdleFeedbackMisses": 0,
                    "startPose": dict(start_pose),
                    "floorBarMm": FLOOR_MM,
                    "pending": None,
                    "commanded": dict(start_pose),
                    "measured": measured,
                    "measuredRaw": measured_raw,
                    "measuredAt": observed_at,
                    "envelope": {
                        "maxDeltaDegrees": {
                            name: settings[name]["maxDeltaDegrees"]
                            for name in LIVE_FOLLOW_JOINTS
                        },
                        "floorMm": FLOOR_MM,
                        "lowestPointMm": round(start_lowest, 3),
                        "joint_2": {
                            "minDegrees": round(start_pose["joint_2"], 3),
                            "maxDegrees": round(start_pose["joint_2"], 3),
                        },
                        "joint_3": {
                            "minDegrees": round(start_pose["joint_3"], 3),
                            "maxDegrees": round(start_pose["joint_3"], 3),
                        },
                    },
                    "stats": {
                        "receivedFrames": 0,
                        "dispatchedFrames": 0,
                        "coalescedFrames": 0,
                        "clampedFrames": 0,
                        "lastAcceptedSequence": 0,
                        "lastDispatchedSequence": 0,
                        "lastAttemptedSequence": 0,
                        "lastDispatchLatencyMs": None,
                        "lastAttemptLatencyMs": None,
                    },
                    "timing": {
                        "inputLeaseMs": round(
                            LIVE_FOLLOW_INPUT_LEASE_SECONDS * 1000
                        ),
                        "maxDurationMs": round(
                            LIVE_FOLLOW_MAX_DURATION_SECONDS * 1000
                        ),
                        "minDispatchIntervalMs": round(
                            LIVE_FOLLOW_MIN_DISPATCH_SECONDS * 1000
                        ),
                        "startedAt": _expires_at(started_epoch),
                        "expiresAt": _expires_at(
                            started_epoch + LIVE_FOLLOW_MAX_DURATION_SECONDS
                        ),
                    },
                }
                with self._lock:
                    # STOP deliberately bypasses the long motion/authority locks.
                    # This state boundary is therefore the linearization point:
                    # either the session installs first and STOP removes it, or
                    # STOP wins and this start can never resurrect the session.
                    if (
                        self._live_follow_start_attempts.get(start_attempt_id)
                        != "starting"
                    ):
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Fast Follow start attempt was cancelled.",
                        )
                    if time.monotonic() >= start_deadline:
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Fast Follow start deadline expired.",
                        )
                    if (
                        stop_generation != self._stop_generation
                        or self._operator_stopped
                        or self._inspection_required
                    ):
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail=(
                                "Fast Follow lost controller continuity while "
                                "starting."
                            ),
                        )
                    response = self._live_follow_response_locked(session)
                    # Installing ``_live_follow`` is the commit write. Keep it
                    # last so every earlier assignment is still covered by the
                    # post-HOLD cleanup path if it unexpectedly raises.
                    self._live_follow_start_attempts[start_attempt_id] = "committed"
                    self._last_live_follow = None
                    self._live_follow = session
                    committed = True
                return response
            except Exception:
                if not committed:
                    self._abort_motion_recoverably()
                raise

    def cancel_live_follow_start(
        self, request: LiveFollowStartCancelRequest
    ) -> dict[str, object]:
        """Tombstone a one-shot start, releasing its session if commit won."""

        start_attempt_id = request.startAttemptId
        while True:
            with self._telemetry_condition:
                active = self._live_follow
                if (
                    active is not None
                    and active.get("startAttemptId") == start_attempt_id
                ):
                    session_id = str(active["sessionId"])
                    self._live_follow_start_attempts[start_attempt_id] = (
                        "cancel_requested"
                    )
                    break
                terminal = self._last_live_follow
                if (
                    terminal is not None
                    and terminal.get("startAttemptId") == start_attempt_id
                ):
                    self._live_follow_start_attempts[start_attempt_id] = "cancelled"
                    return self._live_follow_response_locked(terminal)
                attempt_state = self._live_follow_start_attempts.get(start_attempt_id)
                if attempt_state is None:
                    # Cancellation can legitimately beat a delayed proxy start.
                    # Retain the ID so that request can never take physical hold.
                    self._live_follow_start_attempts[start_attempt_id] = "cancelled"
                    self._telemetry_condition.notify_all()
                    return {
                        "schema": LIVE_FOLLOW_SCHEMA,
                        "startAttemptId": start_attempt_id,
                        "cancelled": True,
                        "inputLeaseMs": round(
                            LIVE_FOLLOW_INPUT_LEASE_SECONDS * 1000
                        ),
                    }
                if attempt_state == "starting":
                    self._live_follow_start_attempts[start_attempt_id] = (
                        "cancel_requested"
                    )
                    attempt_state = "cancel_requested"
                if attempt_state == "cancel_requested":
                    # Confirmation is returned only after the start thread has
                    # unwound any post-HOLD failure and completed its one release
                    # attempt. This wait runs in a worker thread at the route.
                    self._telemetry_condition.wait()
                    continue
                self._live_follow_start_attempts[start_attempt_id] = "cancelled"
                return {
                    "schema": LIVE_FOLLOW_SCHEMA,
                    "startAttemptId": start_attempt_id,
                    "cancelled": True,
                    "inputLeaseMs": round(LIVE_FOLLOW_INPUT_LEASE_SECONDS * 1000),
                }

        terminal = self._finish_live_follow(
            session_id,
            state_name="ended",
            reason="Fast Follow start attempt cancelled by the client.",
            release=True,
        )
        with self._telemetry_condition:
            self._live_follow_start_attempts[start_attempt_id] = "cancelled"
            self._telemetry_condition.notify_all()
            if terminal is not None:
                return terminal
        # The session may have reached another terminal state while cancellation
        # waited for the motion/authority boundary. Reconcile that exact receipt.
        with self._lock:
            latest = self._last_live_follow
            if (
                latest is not None
                and latest.get("startAttemptId") == start_attempt_id
            ):
                return self._live_follow_response_locked(latest)
        return {
            "schema": LIVE_FOLLOW_SCHEMA,
            "startAttemptId": start_attempt_id,
            "cancelled": True,
            "inputLeaseMs": round(LIVE_FOLLOW_INPUT_LEASE_SECONDS * 1000),
        }

    def _bounded_live_follow_target(
        self,
        session: Mapping[str, object],
        requested: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, int], dict[str, object], bool]:
        start = session["startPose"]
        commanded = session["commanded"]
        envelope = session["envelope"]
        assert isinstance(start, dict)
        assert isinstance(commanded, dict)
        assert isinstance(envelope, dict)

        def quantized(candidate: Mapping[str, float]) -> tuple[dict[str, float], dict[str, int]]:
            resolved: dict[str, float] = {}
            goals: dict[str, int] = {}
            settings = session["settings"]
            assert isinstance(settings, dict)
            for name in LIVE_FOLLOW_JOINTS:
                joint = self._store.get(name)
                selected = settings[name]
                assert isinstance(selected, dict)
                max_delta = float(selected["maxDeltaDegrees"])
                logical_low, logical_high = joint.degree_bounds()
                reachable_low, reachable_high = joint.reach_bounds()
                assert (
                    logical_low is not None
                    and logical_high is not None
                    and reachable_low is not None
                    and reachable_high is not None
                )
                # Intersect the deliberate start-relative laboratory span with
                # both the operator-calibrated logical window and the physical
                # goal range before quantization. `goal_for` remains a final
                # defence, not the first place this contract discovers a limit.
                low = max(
                    float(start[name]) - max_delta,
                    float(logical_low),
                    float(reachable_low),
                )
                high = min(
                    float(start[name]) + max_delta,
                    float(logical_high),
                    float(reachable_high),
                )
                if low > high:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail=(
                            f"{JOINT_NAMES[name]} no longer has a calibrated "
                            "reachable Fast Follow range."
                        ),
                    )
                bounded = max(low, min(high, float(candidate[name])))
                goal = joint.goal_for(bounded)
                degrees = joint.degrees_at(goal)
                assert degrees is not None
                goals[name] = goal
                resolved[name] = round(float(degrees), 6)
            return resolved, goals

        def expanded(candidate: Mapping[str, float]) -> tuple[dict[str, object], float]:
            settings = session["settings"]
            assert isinstance(settings, dict)
            result: dict[str, object] = {
                "maxDeltaDegrees": {
                    name: settings[name]["maxDeltaDegrees"]
                    for name in LIVE_FOLLOW_JOINTS
                    if isinstance(settings.get(name), dict)
                },
                "floorMm": FLOOR_MM,
            }
            for name in LIVE_FOLLOW_JOINTS:
                existing = envelope[name]
                assert isinstance(existing, dict)
                result[name] = {
                    "minDegrees": min(
                        float(existing["minDegrees"]), float(candidate[name])
                    ),
                    "maxDegrees": max(
                        float(existing["maxDegrees"]), float(candidate[name])
                    ),
                }
            shoulder = result["joint_2"]
            elbow = result["joint_3"]
            assert isinstance(shoulder, dict) and isinstance(elbow, dict)
            lowest = swept_lowest_point_mm(
                float(shoulder["minDegrees"]),
                float(elbow["minDegrees"]),
                float(shoulder["maxDegrees"]),
                float(elbow["maxDegrees"]),
            )
            result["lowestPointMm"] = round(lowest, 3)
            return result, lowest

        resolved, goals = quantized(requested)
        next_envelope, lowest = expanded(resolved)
        floor_bar = float(session["floorBarMm"])
        if lowest < floor_bar - 1e-6:
            safe = dict(commanded)
            safe_goals = {
                name: self._store.get(name).goal_for(float(commanded[name]))
                for name in LIVE_FOLLOW_JOINTS
            }
            safe_envelope, _ = expanded(safe)
            reachable, blocked = 0.0, 1.0
            for _ in range(18):
                fraction = (reachable + blocked) / 2
                candidate = {
                    name: float(commanded[name])
                    + (float(requested[name]) - float(commanded[name])) * fraction
                    for name in LIVE_FOLLOW_JOINTS
                }
                candidate, candidate_goals = quantized(candidate)
                candidate_envelope, candidate_lowest = expanded(candidate)
                if candidate_lowest >= floor_bar - 1e-6:
                    reachable = fraction
                    safe = candidate
                    safe_goals = candidate_goals
                    safe_envelope = candidate_envelope
                else:
                    blocked = fraction
            resolved, goals, next_envelope = safe, safe_goals, safe_envelope
        clamped = any(
            abs(float(resolved[name]) - float(requested[name])) > 0.05
            for name in LIVE_FOLLOW_JOINTS
        )
        return resolved, goals, next_envelope, clamped

    def live_follow_frame(
        self, request: LiveFollowFrameRequest
    ) -> dict[str, object]:
        arrived = time.monotonic()
        input_expiry_cutoff = arrived - LIVE_FOLLOW_INPUT_LEASE_SECONDS
        while True:
            with self._lock:
                session = self._live_follow
                if session is None or session.get("sessionId") != request.sessionId:
                    # Produce the terminal receipt (expired/faulted/stopped) rather
                    # than making the browser guess why its once-valid id vanished.
                    terminal = self._last_live_follow
                    if (
                        terminal is not None
                        and terminal.get("sessionId") == request.sessionId
                    ):
                        return self._live_follow_response_locked(terminal)
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="That Fast Follow session is not active.",
                    )
                last_input = float(session["lastInputMonotonic"])
                if last_input <= input_expiry_cutoff:
                    expired = True
                else:
                    expired = False
                    stats = session["stats"]
                    assert isinstance(stats, dict)
                    if request.sequence <= int(stats["lastAcceptedSequence"]):
                        raise HTTPException(
                            status_code=status.HTTP_409_CONFLICT,
                            detail="Fast Follow frame sequence must increase monotonically.",
                        )
                    requested = {
                        "joint_2": float(request.joint_2),
                        "joint_3": float(request.joint_3),
                    }
                    previous_commanded = session["commanded"]
                    assert isinstance(previous_commanded, dict)
                    resolved, goals, envelope, clamped = self._bounded_live_follow_target(
                        session, requested
                    )
                    stats["receivedFrames"] = int(stats["receivedFrames"]) + 1
                    if clamped:
                        stats["clampedFrames"] = int(stats["clampedFrames"]) + 1
                    stats["lastAcceptedSequence"] = request.sequence
                    session["lastInputMonotonic"] = max(last_input, arrived)
                    same_quantized_target = all(
                        math.isclose(
                            float(resolved[name]),
                            float(previous_commanded[name]),
                            rel_tol=0.0,
                            abs_tol=1e-9,
                        )
                        for name in LIVE_FOLLOW_JOINTS
                    )
                    if same_quantized_target:
                        # This is the browser's deadman keepalive. Advancing sequence
                        # and receipt stats proves the client is present; retaining any
                        # already-pending first command, but creating no new command,
                        # prevents a steady pointer from repeatedly restarting ramps.
                        return self._live_follow_response_locked(session)
                    if session.get("pending") is not None:
                        stats["coalescedFrames"] = int(stats["coalescedFrames"]) + 1
                    session["commanded"] = resolved
                    session["envelope"] = envelope
                    session["pending"] = {
                        "sequence": request.sequence,
                        "targets": resolved,
                        "goals": goals,
                        "acceptedMonotonic": time.monotonic(),
                    }
                    return self._live_follow_response_locked(session)
            if expired:
                terminal = self._finish_live_follow(
                    request.sessionId,
                    state_name="expired",
                    reason="Fast Follow input lease expired.",
                    release=True,
                    input_expiry_cutoff=input_expiry_cutoff,
                )
                if terminal is not None:
                    return terminal
                # A request that arrived before the old deadline may have renewed
                # the same session while this request waited for the finish boundary.
                # Re-evaluate this request against that timestamp without replacing
                # it with a later wall-clock sample.

    def live_follow_heartbeat(
        self, request: LiveFollowHeartbeatRequest
    ) -> dict[str, object]:
        """Renew browser presence without touching ordered motion state."""

        arrived = time.monotonic()
        input_expiry_cutoff = arrived - LIVE_FOLLOW_INPUT_LEASE_SECONDS
        while True:
            with self._lock:
                session = self._live_follow
                if session is not None and session.get("sessionId") == request.sessionId:
                    last_input = float(session["lastInputMonotonic"])
                    if last_input > input_expiry_cutoff:
                        session["lastInputMonotonic"] = max(last_input, arrived)
                        return self._live_follow_response_locked(session)
                else:
                    terminal = self._last_live_follow
                    if (
                        terminal is not None
                        and terminal.get("sessionId") == request.sessionId
                    ):
                        return self._live_follow_response_locked(terminal)
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="That Fast Follow session is not active.",
                    )
            terminal = self._finish_live_follow(
                request.sessionId,
                state_name="expired",
                reason="Fast Follow input lease expired.",
                release=True,
                input_expiry_cutoff=input_expiry_cutoff,
            )
            if terminal is not None:
                return terminal
            # A pre-deadline request can renew the same session at the finish
            # boundary. Re-evaluate this heartbeat using its original arrival.

    def end_live_follow(self, request: LiveFollowEndRequest) -> dict[str, object]:
        with self._operation_lock, self._motion_lock:
            with self._lock:
                terminal = self._last_live_follow
                session = self._live_follow
                if session is None or session.get("sessionId") != request.sessionId:
                    if (
                        terminal is not None
                        and terminal.get("sessionId") == request.sessionId
                    ):
                        return self._live_follow_response_locked(terminal)
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="That Fast Follow session is not active.",
                    )
                pending = session.get("pending")
                wait_seconds = max(
                    0.0,
                    LIVE_FOLLOW_MIN_DISPATCH_SECONDS
                    - (time.monotonic() - float(session["lastDispatchMonotonic"])),
                )
            if not request.flushPending:
                # Losing the browser's pointer/dead-man boundary is not an
                # orderly release. End the session and revoke its exact J2/J3
                # hold without ever transmitting an already-accepted target.
                cancelled = self._finish_live_follow(
                    request.sessionId,
                    state_name="ended",
                    reason="Fast Follow cancelled by the client.",
                    release=True,
                )
                if cancelled is None:
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="That Fast Follow session is not active.",
                    )
                return cancelled
            if isinstance(pending, dict):
                # Pointer-up is a bounded flush, not cancellation. Wait at most
                # one dispatch interval, then prove the newest accepted pose
                # through compact feedback before returning an ended receipt.
                if wait_seconds > 0:
                    time.sleep(wait_seconds)
                self._dispatch_live_follow(time.monotonic(), force=True)
            with self._lock:
                session = self._live_follow
                if session is None or session.get("sessionId") != request.sessionId:
                    terminal = self._last_live_follow
                    if (
                        terminal is not None
                        and terminal.get("sessionId") == request.sessionId
                    ):
                        return self._live_follow_response_locked(terminal)
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Fast Follow lost continuity while ending.",
                    )
                stats = session["stats"]
                assert isinstance(stats, dict)
                if session.get("pending") is not None:
                    # A forced flush may only leave a pending target after a
                    # continuity fault; never claim an orderly end in that case.
                    faulted = True
                else:
                    faulted = False
                    # A keepalive can have a newer sequence than the physical
                    # dispatch while naming the exact same quantized target.
                    # Mark that latest sequence as physically satisfied without
                    # inventing another dispatch or incrementing its count.
                    stats["lastDispatchedSequence"] = stats["lastAcceptedSequence"]
            if faulted:
                failed = self._finish_live_follow(
                    request.sessionId,
                    state_name="faulted",
                    reason="Fast Follow could not flush its final target.",
                    release=True,
                )
                assert failed is not None
                return failed
            ended = self._finish_live_follow(
                request.sessionId,
                state_name="ended",
                reason="Fast Follow ended by the client.",
                release=False,
            )
            if ended is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="That Fast Follow session is not active.",
                )
            return ended

    def _dispatch_live_follow(self, now: float, *, force: bool = False) -> bool:
        with self._lock:
            session = self._live_follow
            if session is None:
                return False
            session_id = str(session["sessionId"])
            started = float(session["startedMonotonic"])
            last_input = float(session["lastInputMonotonic"])
            if now - started >= LIVE_FOLLOW_MAX_DURATION_SECONDS:
                terminal_state = "expired"
                terminal_reason = "Fast Follow reached its 30 second maximum."
                input_expiry_cutoff = None
            elif now - last_input >= LIVE_FOLLOW_INPUT_LEASE_SECONDS:
                terminal_state = "expired"
                terminal_reason = "Fast Follow input lease expired."
                input_expiry_cutoff = now - LIVE_FOLLOW_INPUT_LEASE_SECONDS
            else:
                terminal_state = None
                terminal_reason = None
                input_expiry_cutoff = None
            pending = session.get("pending")
            dispatch_due = (
                isinstance(pending, dict)
                and (
                    force
                    or now - float(session["lastDispatchMonotonic"])
                    >= LIVE_FOLLOW_MIN_DISPATCH_SECONDS
                )
            )
        if terminal_state is not None and terminal_reason is not None:
            terminal = self._finish_live_follow(
                session_id,
                state_name="expired",
                reason=terminal_reason,
                release=True,
                input_expiry_cutoff=input_expiry_cutoff,
            )
            if terminal is not None:
                return True
            # A same-session heartbeat may have renewed the input lease after
            # the snapshot above. Re-read the pending/dispatch boundary instead
            # of returning from a tick that no longer owns expiry.
            with self._lock:
                current = self._live_follow
                if current is None or current.get("sessionId") != session_id:
                    return True
                pending = current.get("pending")
                dispatch_due = (
                    isinstance(pending, dict)
                    and (
                        force
                        or now - float(current["lastDispatchMonotonic"])
                        >= LIVE_FOLLOW_MIN_DISPATCH_SECONDS
                    )
                )
        if not dispatch_due:
            try:
                reported = self._controller.transport_state()
                with self._lock:
                    current = self._live_follow
                    if current is None or current.get("sessionId") != session_id:
                        return True
                    healthy = (
                        self._floor_guard
                        and not self._operator_stopped
                        and not self._inspection_required
                        and self._live_follow_motion_health(reported)
                        == str(current["bootId"])
                        and self._configuration_digest()
                        == str(current["configurationDigest"])
                        and LIVE_FOLLOW_CAPABILITY
                        in self._reported_capabilities(reported)
                        and LIVE_FOLLOW_READ_CAPABILITY
                        in self._reported_capabilities(reported)
                    )
                    pending_now = current.get("pending")
                    feedback_due = (
                        now - float(current["lastFeedbackMonotonic"])
                        >= LIVE_FOLLOW_FEEDBACK_SECONDS
                    )
                if not healthy:
                    raise ControllerCommandError("LIVE_FOLLOW_CONTINUITY_LOST")
                if isinstance(pending_now, dict) or not feedback_due:
                    # A pending target always wins the next service slot. Do
                    # not spend even a compact read transaction in front of it.
                    return False
                with self._motion_lock:
                    with self._lock:
                        current = self._live_follow
                        if current is None or current.get("sessionId") != session_id:
                            return True
                        if current.get("pending") is not None:
                            return False
                        measured_raw = dict(current["measuredRaw"])
                        measured_at = float(current["measuredAt"])
                        current["lastFeedbackMonotonic"] = time.monotonic()
                    follow_read = getattr(self._controller, "follow_read")
                    receipt = follow_read(
                        [
                            self._store.get(name).servoId
                            for name in LIVE_FOLLOW_JOINTS
                        ]
                    )
                    observed_at = time.monotonic()
                    if not isinstance(receipt, dict) or not isinstance(
                        receipt.get("feedback"), list
                    ):
                        raise ControllerProtocolError(
                            "Fast Follow readback did not return compact feedback."
                        )
                    feedback = [
                        row for row in receipt["feedback"] if isinstance(row, dict)
                    ]
                    measured, latest_raw = self._live_follow_measurement(
                        feedback,
                        previous_raw=measured_raw,
                        previous_at=measured_at,
                        observed_at=observed_at,
                    )
                    with self._telemetry_condition:
                        current = self._live_follow
                        if current is None or current.get("sessionId") != session_id:
                            return True
                        current["measured"] = measured
                        current["measuredRaw"] = latest_raw
                        current["measuredAt"] = observed_at
                        current["lastFeedbackMonotonic"] = observed_at
                        current["consecutiveIdleFeedbackMisses"] = 0
                        self._telemetry_generation += 1
                        self._telemetry_observed_at = observed_at
                        self._telemetry_condition.notify_all()
                return True
            except Exception as error:
                if (
                    isinstance(error, ControllerCommandError)
                    and error.code == "FEEDBACK_UNAVAILABLE"
                ):
                    with self._lock:
                        current = self._live_follow
                        if (
                            current is not None
                            and current.get("sessionId") == session_id
                        ):
                            misses = int(
                                current.get("consecutiveIdleFeedbackMisses", 0)
                            ) + 1
                            current["consecutiveIdleFeedbackMisses"] = misses
                        else:
                            misses = LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE + 1
                    if misses <= LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE:
                        _LOG.info(
                            "Fast Follow session %s deferred transient idle "
                            "feedback miss %d/%d",
                            session_id,
                            misses,
                            LIVE_FOLLOW_IDLE_FEEDBACK_MISS_TOLERANCE,
                        )
                        return True
                _LOG.warning(
                    "Fast Follow session %s lost idle continuity: %r",
                    session_id,
                    error,
                )
                self._finish_live_follow(
                    session_id,
                    state_name="faulted",
                    reason="Fast Follow lost safe controller continuity.",
                    release=True,
                    fault=_safe_live_follow_fault(error),
                )
                return True
            return False

        with self._motion_lock:
            with self._lock:
                session = self._live_follow
                if session is None or session.get("sessionId") != session_id:
                    return True
                pending = session.get("pending")
                if not isinstance(pending, dict):
                    return False
                session["pending"] = None
                session["lastDispatchMonotonic"] = time.monotonic()
                boot_id = str(session["bootId"])
                config_digest = str(session["configurationDigest"])
                settings = deepcopy(session["settings"])
                assert isinstance(settings, dict)
                measured_raw = dict(session["measuredRaw"])
                measured_at = float(session["measuredAt"])
            dispatch_started: float | None = None
            try:
                reported = self._controller.transport_state()
                with self._lock:
                    floor_guard = self._floor_guard
                    locally_stopped = self._operator_stopped or self._inspection_required
                if (
                    locally_stopped
                    or not floor_guard
                    or self._live_follow_motion_health(reported) != boot_id
                    or self._configuration_digest() != config_digest
                    or LIVE_FOLLOW_CAPABILITY not in self._reported_capabilities(reported)
                    or LIVE_FOLLOW_READ_CAPABILITY
                    not in self._reported_capabilities(reported)
                ):
                    raise ControllerCommandError("LIVE_FOLLOW_CONTINUITY_LOST")
                goals = pending["goals"]
                assert isinstance(goals, dict)
                moves = [
                    (
                        self._store.get(name).servoId,
                        int(goals[name]),
                        int(settings[name]["speed"]),
                        int(settings[name]["accel"]),
                    )
                    for name in LIVE_FOLLOW_JOINTS
                ]
                follow_set = getattr(self._controller, "follow_set")
                dispatch_started = time.monotonic()
                with self._lock:
                    current = self._live_follow
                    if current is not None and current.get("sessionId") == session_id:
                        stats = current["stats"]
                        assert isinstance(stats, dict)
                        stats["lastAttemptedSequence"] = int(pending["sequence"])
                receipt = follow_set(moves)
                observed_at = time.monotonic()
                if not isinstance(receipt, dict) or not isinstance(
                    receipt.get("feedback"), list
                ):
                    raise ControllerProtocolError(
                        "Fast Follow did not return compact feedback."
                    )
                feedback = [
                    row for row in receipt["feedback"] if isinstance(row, dict)
                ]
                measured, latest_raw = self._live_follow_measurement(
                    feedback,
                    previous_raw=measured_raw,
                    previous_at=measured_at,
                    observed_at=observed_at,
                )
                with self._telemetry_condition:
                    current = self._live_follow
                    if current is None or current.get("sessionId") != session_id:
                        return True
                    stats = current["stats"]
                    assert isinstance(stats, dict)
                    stats["dispatchedFrames"] = int(stats["dispatchedFrames"]) + 1
                    stats["lastDispatchedSequence"] = int(pending["sequence"])
                    stats["lastDispatchLatencyMs"] = round(
                        (observed_at - dispatch_started) * 1000, 3
                    )
                    stats["lastAttemptLatencyMs"] = stats["lastDispatchLatencyMs"]
                    current["measured"] = measured
                    current["measuredRaw"] = latest_raw
                    current["measuredAt"] = observed_at
                    current["lastFeedbackMonotonic"] = observed_at
                    current["consecutiveIdleFeedbackMisses"] = 0
                    self._last_move = observed_at
                    self._telemetry_generation += 1
                    self._telemetry_observed_at = observed_at
                    self._telemetry_condition.notify_all()
                return True
            except Exception as error:
                observed_at = time.monotonic()
                dispatch_latency_ms = (
                    round((observed_at - dispatch_started) * 1000, 3)
                    if dispatch_started is not None
                    else None
                )
                fault = _safe_live_follow_fault(error)
                with self._lock:
                    current = self._live_follow
                    if current is not None and current.get("sessionId") == session_id:
                        stats = current["stats"]
                        assert isinstance(stats, dict)
                        stats["lastAttemptedSequence"] = int(pending["sequence"])
                        stats["lastAttemptLatencyMs"] = dispatch_latency_ms
                _LOG.warning(
                    json.dumps(
                        {
                            "event": "arm_fast_follow_dispatch_failed",
                            "sessionId": session_id,
                            "settings": settings,
                            "pendingSequence": int(pending["sequence"]),
                            "fault": fault,
                            "latencyMs": dispatch_latency_ms,
                        },
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                )
                self._finish_live_follow(
                    session_id,
                    state_name="faulted",
                    reason="Fast Follow lost safe controller feedback.",
                    release=True,
                    fault=fault,
                )
                return True

    def floor_guard(self, enabled: bool) -> dict[str, object]:
        with self._operation_lock, self._motion_lock:
            return self._floor_guard_locked(enabled)

    def _floor_guard_locked(self, enabled: bool) -> dict[str, object]:
        with self._lock:
            self._floor_guard = bool(enabled)
        return self.state()

    def targets(self, wanted: dict[str, float]) -> dict[str, object]:
        with self._operation_lock, self._motion_lock:
            return self._targets_locked(wanted)

    def _targets_locked(self, wanted: dict[str, float]) -> dict[str, object]:
        """Drive several joints from one request.

        Torque for the whole set is taken first. A capable HAT receives one
        host MOVE_SET and emits one grouped write per servo family; mixed-family
        sets therefore have a small bounded cross-family skew, not true atomic
        simultaneous starts. Older HATs use the guarded sequential fallback.

        Returning is not the same as arriving, and the controller accepting a
        MOVE is not a promise that it will finish one. Each goal is remembered
        so `_chase_goals` can see the move through from the service loop; this
        call still answers as soon as the goals are on the wire.
        """

        wanted = self._guard_floor(wanted)
        planned: list[tuple[str, JointState, int]] = []
        for name, degrees in wanted.items():
            joint = self._store.get(name)
            if joint.rawZero is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{JOINT_NAMES[name]} has no zero yet. Set its zero first.",
                )
            if joint.multiTurn and self.live_raw(joint.servoId) is None:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=(
                        f"{JOINT_NAMES[name]} position is untrusted after controller or "
                        "servo continuity was lost. Put the Base at physical zero and choose "
                        "Set zero here."
                    ),
                )
            planned.append((name, joint, joint.goal_for(degrees)))
        planned = self._keep_quantized_floor_sweep_safe(wanted, planned)
        return {"moved": self._drive_prepared_goals_locked(planned)}

    def _keep_quantized_floor_sweep_safe(
        self,
        wanted: dict[str, float],
        planned: list[tuple[str, JointState, int]],
    ) -> list[tuple[str, JointState, int]]:
        """Back a direct clamp off if tick rounding crossed its safe boundary."""

        with self._lock:
            floor_guard = self._floor_guard
        if not floor_guard or not any(name in wanted for name in FLOOR_JOINTS):
            return planned
        current = {name: self._measured_degrees(name) for name in FLOOR_JOINTS}
        if not all(isinstance(current[name], (int, float)) for name in FLOOR_JOINTS):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Fresh Shoulder and Elbow telemetry is required for the floor guard.",
            )
        start = {name: float(current[name]) for name in FLOOR_JOINTS}
        bar = min(FLOOR_MM, lowest_point_mm(start["joint_2"], start["joint_3"]))

        def sweep(candidate: list[tuple[str, JointState, int]]) -> float:
            target = dict(start)
            for name, joint, goal in candidate:
                if name in FLOOR_JOINTS:
                    degrees = joint.degrees_at(goal)
                    assert degrees is not None
                    target[name] = float(degrees)
            return swept_lowest_point_mm(
                start["joint_2"],
                start["joint_3"],
                target["joint_2"],
                target["joint_3"],
            )

        if sweep(planned) >= bar - 1e-6:
            return planned

        # The rectangle at a smaller fraction is a subset of the larger one, so
        # this quantized predicate remains monotone despite raw-tick plateaus.
        def at_fraction(fraction: float) -> list[tuple[str, JointState, int]]:
            candidate: list[tuple[str, JointState, int]] = []
            for name, joint, goal in planned:
                if name in FLOOR_JOINTS:
                    degrees = start[name] + (float(wanted[name]) - start[name]) * fraction
                    goal = joint.goal_for(degrees)
                candidate.append((name, joint, goal))
            return candidate

        safe = at_fraction(0.0)
        if sweep(safe) < bar - 1e-6:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Fresh joint ticks cannot represent a floor-safe direct move.",
            )
        reachable, blocked = 0.0, 1.0
        for _ in range(18):
            middle = (reachable + blocked) / 2
            candidate = at_fraction(middle)
            if sweep(candidate) >= bar - 1e-6:
                reachable = middle
                safe = candidate
            else:
                blocked = middle
        return safe

    def _drive_prepared_goals_locked(
        self, planned: list[tuple[str, JointState, int]]
    ) -> list[dict[str, object]]:
        """Send already-resolved raw goals without clamping or re-quantizing."""

        if not planned:
            return []
        servo_ids = [joint.servoId for _, joint, _ in planned]
        if len(set(servo_ids)) != len(servo_ids):
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    "Two logical joints resolve to the same servo ID; "
                    "correct calibration before motion."
                ),
            )
        now = time.monotonic()
        with self._lock:
            self._last_move = now

        def drive() -> list[dict[str, object]]:
            reported = self._controller.transport_state()
            if any(name in SCS_JOINTS for name, _, _ in planned):
                # SCS defaults cannot be inferred safely. This capability and
                # declaration proof precedes HOLD_SET so a refusal has no
                # torque or motion side effect.
                self._require_servo_family_ready(reported)
            self._ensure_held([joint.servoId for _, joint, _ in planned])
            sent: list[dict[str, object]] = []
            identity = reported.get("identity")
            capabilities = identity.get("capabilities") if isinstance(identity, dict) else None
            move_set = getattr(self._controller, "move_set", None)
            use_move_set = (
                len(planned) > 1
                and callable(move_set)
                and isinstance(capabilities, list)
                and "move_set_v1" in capabilities
            )
            if use_move_set:
                # One host transaction for the coordinated pose. Firmware
                # preflights the complete group before any servo write, then
                # emits one grouped packet per dialect; mixed-family groups
                # therefore have bounded cross-family skew. Older deployed HATs
                # under-advertise and stay on the guarded MOVE sequence below.
                move_set(
                    [
                        (joint.servoId, goal, joint.speed, joint.accel)
                        for _, joint, goal in planned
                    ]
                )
                accepted_at = time.monotonic()
                with self._lock:
                    for name, _joint, goal in planned:
                        self._goals[name] = (
                            goal,
                            ARRIVAL_ATTEMPTS,
                            accepted_at + ARRIVAL_SECONDS,
                        )
                return [
                    {
                        "joint": name,
                        "servoId": joint.servoId,
                        "goal": goal,
                        "degrees": joint.degrees_at(goal),
                    }
                    for name, joint, goal in planned
                ]
            try:
                for index, (name, joint, goal) in enumerate(planned):
                    if index:
                        # Between MOVEs, not before the first: the heartbeat thread is
                        # already queued on the lock by the time one has gone out, and
                        # the watchdog it feeds is what keeps the rest of the burst
                        # authorised. See MOVE_YIELD_SECONDS.
                        time.sleep(MOVE_YIELD_SECONDS)
                    self._move(joint, goal)
                    # Remember only a command the controller actually accepted. A
                    # failed first send must not be resurrected later by the chase
                    # loop as if the operator had asked twice.
                    with self._lock:
                        self._goals[name] = (
                            goal,
                            ARRIVAL_ATTEMPTS,
                            time.monotonic() + ARRIVAL_SECONDS,
                        )
                    sent.append(
                        {
                            "joint": name,
                            "servoId": joint.servoId,
                            "goal": goal,
                            "degrees": joint.degrees_at(goal),
                        }
                    )
            except (ControllerTransportError, ControllerProtocolError):
                # Legacy sequential MOVE has no atomic receipt. Once the call
                # is entered, a transport/protocol failure cannot prove whether
                # even the first packet acted. Revoke intent and torque authority;
                # the interrupted request fails, but this is not an operator STOP.
                raise
            except Exception:
                if sent:
                    self._abort_motion_recoverably()
                raise
            return sent

        try:
            return self._without_refusal(drive)
        except ControllerCommandError as error:
            if str(getattr(error, "code", "")) == "MOVE_SET_FAILED":
                self._abort_motion_recoverably()
            raise
        except (ControllerTransportError, ControllerProtocolError):
            # MOVE_SET is one host transaction, so loss of its receipt is still
            # a recoverable abort: firmware/watchdog removes authority and no
            # automatic path is allowed to impersonate the red STOP button.
            self._abort_motion_recoverably()
            raise
        except Exception:
            reported = self._controller.transport_state()
            if reported.get("safetyStopReason") in AUTOMATIC_STOP_REASONS:
                self._abort_motion_recoverably()
            raise

    def _move(self, joint: JointState, goal: int) -> None:
        if joint.multiTurn:
            # One signed absolute goal. The HAT closes it against its counted
            # encoder frame; the Pi never manufactures relative corrections.
            with self._multi_turn_lock:
                self._controller.move_multi_turn(
                    joint.servoId, goal, joint.speed, joint.accel
                )
        else:
            self._controller.move(joint.servoId, goal, joint.speed, joint.accel)

    def _forget_goals(self, names: list[str]) -> None:
        with self._lock:
            for name in names:
                self._goals.pop(name, None)

    def _chase_goals(self) -> None:
        """Finish a move the controller accepted and then abandoned.

        A MOVE only survives while torque authority does. Lose it -- and it goes
        the instant the 750 ms host watchdog runs dry, not when the 2 s hold
        lease does -- and the HAT drops the active goal, while the next
        hold capture writes the joint's CURRENT position as its hold, pinning it
        where it stopped. Measured: the elbow was sent to 2427, the hold captured
        it at 1392 seven tenths of a second later, and it sat at 1391 for the
        next seven seconds. The HTTP call had already returned 200.

        So the goal is re-sent, from the one loop that owns the serial port, once
        the joint has come to rest short of it. Re-sending an absolute goal asks
        for exactly what was already asked for, which is why it is safe to do
        without the operator; the attempt count is what stops a joint stalled
        against something solid being driven at it forever.
        """

        # A retry is a physical MOVE just like an HTTP target. Keep the entire
        # observe/decide/send sequence in the same lane as direct and planned
        # motion so a plan cannot be validated between a chase decision and its
        # delayed resend. STOP intentionally does not take this lock and remains
        # able to preempt the arm immediately.
        with self._motion_lock:
            with self._lock:
                outstanding = dict(self._goals)
                held = set(self._held)
            if not outstanding:
                return
            reported = self._controller.transport_state()
            telemetry = self.telemetry_of(reported)
            now = time.monotonic()
            finished: list[str] = []
            for name, (goal, attempts, expires) in outstanding.items():
                joint = self._store.get(name)
                servo = telemetry.get(joint.servoId)
                if not isinstance(servo, dict) or joint.servoId not in held or now >= expires:
                    # Offline, released by the operator, or simply old news. None of
                    # those is a move worth finishing on the arm's behalf.
                    finished.append(name)
                    continue
                counted = self._multi_turn_raw.get(joint.servoId)
                if joint.multiTurn and counted is None:
                    # Never chase from the servo's wrapping one-turn telemetry. Only
                    # the HAT's valid counted coordinate can close an absolute goal.
                    continue
                raw = counted if joint.multiTurn else servo.get("rawPosition")
                if not isinstance(raw, int) or isinstance(raw, bool):
                    continue
                if abs(raw - goal) <= _arrival_tolerance_ticks(
                    name, joint.ticks_per_degree
                ):
                    finished.append(name)
                    continue
                if servo.get("moving"):
                    continue
                if attempts <= 0:
                    _LOG.warning(
                        "%s stopped %d counts short of %d and would not close it.",
                        JOINT_NAMES[name], goal - raw, goal,
                    )
                    finished.append(name)
                    continue
                with self._lock:
                    self._goals[name] = (goal, attempts - 1, expires)
                    self._last_move = time.monotonic()
                try:
                    self._without_refusal(lambda: self._move(joint, goal))
                except Exception as error:
                    _LOG.warning("Could not re-send %s to %d: %r", JOINT_NAMES[name], goal, error)
                    finished.append(name)
            self._forget_goals(finished)

    def torque(self, hold: list[int]) -> dict[str, object]:
        # Truncate rather than refuse: "hold everything" is a reasonable thing to
        # ask for even when the controller can only hold MAX_HELD, and the state
        # this returns names exactly which servos ended up energised, so the
        # panel shows the leftover as free instead of lying about it.
        with self._operation_lock, self._motion_lock, self._authority_lock:
            wanted = sorted(set(hold))[:MAX_HELD]
            with self._lock:
                previous = list(self._held)
                self._held = wanted
                self._last_hold = time.monotonic()
            try:
                # Always sent, empty set included: HOLD_SET is the statement of which
                # servos have authority, so skipping it on release left the
                # controller holding a set the Pi believed it had dropped.
                self._without_refusal(lambda: self._apply_hold(wanted))
                for servo_id in previous:
                    if servo_id not in wanted:
                        self._controller.torque_off(servo_id)
                with self._lock:
                    released = set(previous) - set(wanted)
                    for name in list(self._goals):
                        if self._store.get(name).servoId in released:
                            self._goals.pop(name, None)
            except Exception:
                with self._lock:
                    self._held = previous
                raise
            return self.state()

    def assign_id(self, old_id: int, new_id: int) -> dict[str, object]:
        with self._operation_lock, self._motion_lock:
            return self._assign_id_locked(old_id, new_id)

    def scan(self) -> dict[str, object]:
        """Run the commissioning scan outside an active pose/capture operation."""

        with self._operation_lock, self._motion_lock:
            scan_result = self._controller.scan(0, 10)
            return {**self.state(), "scan": scan_result}

    def _assign_id_locked(self, old_id: int, new_id: int) -> dict[str, object]:
        """Rename a servo, then follow it with every joint that pointed at it.

        Leaving the joints behind would silently unbind them: the store would
        still name an ID that no longer answers, and the joint would read offline
        with a perfectly good calibration.
        """

        conflict = next(
            (
                name
                for name, joint in self._store.snapshot().items()
                if joint.servoId == new_id and joint.servoId != old_id
            ),
            None,
        )
        if conflict is not None:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=(
                    f"Servo ID {new_id} is already assigned to "
                    f"{JOINT_NAMES[conflict]}."
                ),
            )

        result = self._controller.assign_id(old_id, new_id)
        for name in JOINT_IDS:
            joint = self._store.get(name)
            if joint.servoId == old_id:
                joint.servoId = new_id
        self._store.save()
        self._declare_servo_families()
        return {**self.state(), "assigned": result}

    def _latch_operator_stop(self) -> None:
        with self._telemetry_condition:
            session = self._live_follow
            if session is not None:
                session["state"] = "stopped"
                session["reason"] = "STOP ended Fast Follow."
                session["pending"] = None
                self._last_live_follow = self._live_follow_response_locked(session)
                self._live_follow = None
            self._held = []
            # An outstanding goal is a move still owed to the operator, and STOP
            # is them saying they no longer want it. Forgetting it here rather
            # than letting it age out is what stops the arm finishing the move
            # the moment torque comes back.
            self._goals.clear()
            # Deliberate, so only the explicit clear-stop path may release it.
            self._operator_stopped = True
            if self._local_safety_stop_reason is None:
                self._local_safety_stop_reason = "EXPLICIT_STOP"
            self._stop_generation += 1
            self._telemetry_condition.notify_all()

    def _abort_motion_recoverably(self) -> None:
        """Drop Pi motion intent without manufacturing an operator STOP.

        The controller/firmware owns broadcast and addressed torque-off proof.
        Clearing the desired hold and every owed goal here prevents a background
        renewal or chase from resurrecting an interrupted move. Failure to send
        the empty hold is safe: a transport lapse/watchdog also revokes authority,
        and fresh telemetry must prove recovery before later motion is admitted.
        """

        with self._authority_lock:
            with self._telemetry_condition:
                self._held = []
                self._goals.clear()
                self._last_hold = time.monotonic()
                self._stop_generation += 1
                self._telemetry_condition.notify_all()
            try:
                self._controller.set_hold_servos([], HOLD_LEASE_MS)
            except Exception:
                pass

    def _latch_inspection_stop(self) -> None:
        """Make ambiguous group dispatch a durable operator-cleared Pi stop."""

        with self._telemetry_condition:
            self._held = []
            self._goals.clear()
            self._operator_stopped = True
            self._inspection_required = True
            if self._local_safety_stop_reason is None:
                self._local_safety_stop_reason = "MOVE_SET_UNCONFIRMED"
            self._stop_generation += 1
            self._telemetry_condition.notify_all()
        try:
            self._controller.stop()
        except Exception:
            # MOVE_SET_FAILED already carries a validated firmware STOP receipt.
            # The extra STOP is reinforcement; Pi-side refusal remains latched.
            self._latch_stop_delivery_unknown()

    def _latch_stop_delivery_unknown(self) -> None:
        """Persist a Pi-authoritative stop when firmware STOP is unproved."""

        with self._telemetry_condition:
            self._held = []
            self._goals.clear()
            self._operator_stopped = True
            self._inspection_required = True
            self._local_safety_stop_reason = "STOP_DELIVERY_UNKNOWN"
            self._telemetry_condition.notify_all()
        try:
            with self._safety_latch_lock:
                self._store.persist_safety_latch("STOP_DELIVERY_UNKNOWN")
        except OSError as error:
            _LOG.critical("Could not persist the arm safety latch: %r", error)

    def stop_for_commissioning(self) -> dict[str, object]:
        """Deliver the same preemptive STOP while preserving legacy response shape."""

        won_capture_gate = self._capture_gate.acquire(blocking=False)
        try:
            # If this acquired the gate, STOP is ordered before any future
            # shutter check. If it did not, capture has already begun; motor
            # STOP is still delivered without waiting on camera I/O.
            self._latch_operator_stop()
        finally:
            if won_capture_gate:
                self._capture_gate.release()
        try:
            result = self._controller.stop()
        except Exception:
            # The bus attempt remains first and immediate. Persist only its
            # failed/unknown result so SD-card latency can never delay STOP.
            self._latch_stop_delivery_unknown()
            raise
        try:
            # Persist only after the physical STOP receipt, so filesystem latency
            # can never delay the emergency action. This is what lets a Pi restart
            # distinguish a legacy firmware-2.4 operator STOP from its otherwise
            # identical reasonless automatic watchdog latch.
            with self._safety_latch_lock:
                self._store.persist_safety_latch("EXPLICIT_STOP")
        except OSError:
            self._latch_stop_delivery_unknown()
            raise ControllerCommandError(
                "SAFETY_LATCH_PERSISTENCE_FAILED"
            ) from None
        return result

    def stop(self) -> dict[str, object]:
        self.stop_for_commissioning()
        return self.state()

    def clear_stop(self) -> dict[str, object]:
        self.clear_stop_for_commissioning()
        return self.state()

    def clear_stop_for_commissioning(self) -> dict[str, object]:
        """Apply an explicitly inspected RESET and converge both API latches."""

        with self._operation_lock, self._motion_lock, self._authority_lock:
            with self._lock:
                self._held = []
                self._goals.clear()
                stop_generation = self._stop_generation
                local_clear_required = (
                    self._inspection_required or self._operator_stopped
                )
            reported = self._controller.transport_state()
            controller_clear_required = (
                reported.get("motionState") == "stopped"
                or self._reported_clear_required_stop(reported)
            )
            if local_clear_required and not controller_clear_required:
                # A failed reinforcing STOP can leave a Pi-only legacy-motion
                # ambiguity latch after reconnect. Establish a verified physical
                # STOP first so inspected RESET has a real latch to clear.
                stop_result = self._controller.stop()
                stopped_report = self._controller.transport_state()
                if not (
                    (
                        isinstance(stop_result, dict)
                        and stop_result.get("stopped") is True
                    )
                    or stopped_report.get("motionState") == "stopped"
                ):
                    raise ControllerCommandError("STOP_UNCONFIRMED")
                controller_clear_required = True
            inspection_required = local_clear_required or controller_clear_required
            reset = self._controller.reset
            durable_clear_started = False
            if inspection_required:
                reason = (
                    self._local_safety_stop_reason
                    or reported.get("safetyStopReason")
                    or "SAFETY_FAULT"
                )
                try:
                    # Write-ahead before RESET: from here until the final
                    # generation/family commit, a crash must make the next Pi
                    # process refuse motion even if RESET cleared the HAT latch.
                    with self._safety_latch_lock:
                        self._store.persist_safety_latch(str(reason))
                except OSError:
                    with self._telemetry_condition:
                        self._operator_stopped = True
                        self._inspection_required = True
                        self._local_safety_stop_reason = "STOP_DELIVERY_UNKNOWN"
                        self._telemetry_condition.notify_all()
                    raise ControllerCommandError(
                        "SAFETY_LATCH_PERSISTENCE_FAILED"
                    ) from None
                durable_clear_started = True
                with self._telemetry_condition:
                    self._operator_stopped = True
                    self._inspection_required = True
                    self._local_safety_stop_reason = str(reason)
                try:
                    result = reset(inspected=True)
                except TypeError:
                    # Replay/legacy controller implementations predate the
                    # explicit inspection argument; this is still the explicit
                    # operator clear-stop path, never automatic recovery.
                    result = reset()
            else:
                result = reset()
            with self._lock:
                interrupted = self._stop_generation != stop_generation
            if not interrupted:
                try:
                    # A servo ID may have changed while the durable STOP gate
                    # correctly refused FAMILY. Prove every current mapping now,
                    # while all motion lanes are still closed, before releasing.
                    self._declare_servo_families(strict=True)
                except Exception:
                    self._latch_inspection_stop()
                    raise
                marker_error = False
                # STOP increments this generation before its controller call.
                # Check on both sides of marker I/O, but never hold the
                # condition while unlink/fsync runs: emergency STOP must reach
                # the bus even if the state filesystem stalls indefinitely.
                with self._telemetry_condition:
                    interrupted = self._stop_generation != stop_generation
                if not interrupted and durable_clear_started:
                    try:
                        with self._safety_latch_lock:
                            # A failed newer STOP persists under this same lock.
                            # Recheck only after we own it so an older clear can
                            # never unlink that newer STOP_DELIVERY_UNKNOWN file.
                            with self._telemetry_condition:
                                interrupted = (
                                    self._stop_generation != stop_generation
                                )
                            if not interrupted:
                                self._store.clear_safety_latch()
                    except OSError:
                        marker_error = True
                with self._telemetry_condition:
                    interrupted = self._stop_generation != stop_generation
                    if marker_error:
                        self._operator_stopped = True
                        self._inspection_required = True
                        self._local_safety_stop_reason = (
                            "STOP_DELIVERY_UNKNOWN"
                        )
                    elif not interrupted:
                        self._operator_stopped = False
                        self._inspection_required = False
                        self._local_safety_stop_reason = None
                if marker_error:
                    try:
                        stop_result = self._controller.stop()
                        stopped_report = self._controller.transport_state()
                    except Exception:
                        self._latch_stop_delivery_unknown()
                        raise ControllerCommandError(
                            "STOP_DELIVERY_UNKNOWN"
                        ) from None
                    if not (
                        (
                            isinstance(stop_result, dict)
                            and stop_result.get("stopped") is True
                        )
                        or stopped_report.get("motionState") == "stopped"
                    ):
                        self._latch_stop_delivery_unknown()
                        raise ControllerCommandError(
                            "STOP_DELIVERY_UNKNOWN"
                        ) from None
                    raise ControllerCommandError(
                        "SAFETY_LATCH_PERSISTENCE_FAILED"
                    ) from None
            if interrupted:
                # A later STOP always wins over the clear request that was
                # already in flight, including when it reached firmware before
                # this RESET and would otherwise have been silently undone.
                try:
                    result = self._controller.stop()
                except Exception:
                    self._latch_stop_delivery_unknown()
                    raise ControllerCommandError("STOP_DELIVERY_UNKNOWN") from None
            return result if isinstance(result, dict) else self._controller.transport_state()

    def close(self) -> None:
        self._stop_service.set()


def _translate(error: Exception) -> HTTPException:
    if isinstance(error, ControllerCommandError):
        detail: dict[str, object] = {
            "code": error.code,
            **_safe_motion_failure(error.payload),
        }
        return HTTPException(status_code=409, detail=detail)
    if isinstance(error, (ControllerProtocolError, ControllerTransportError)):
        # Keep the browser response bounded, but retain the exact local cause.
        # A bare 502 made a scan evidence mismatch indistinguishable from a
        # physically missing UART connection in the Pi journal.
        _LOG.warning(
            "Arm controller link failure (%s): %s",
            type(error).__name__,
            error,
        )
        return HTTPException(status_code=502, detail={"code": "CONTROLLER_LINK_UNHEALTHY"})
    if isinstance(error, ControllerUnavailableError):
        return HTTPException(status_code=503, detail={"code": "CONTROLLER_UNAVAILABLE"})
    if isinstance(error, HTTPException):
        return error
    if isinstance(error, ValueError):
        # The controller validates its own arguments and says exactly which one
        # is wrong. That is a bad number, not a broken gateway -- and reported as
        # a 500 the proxy relabelled it "gateway unavailable", which sent every
        # investigation at the network instead of at the value.
        _LOG.warning("Arm command rejected a value: %s", error)
        return HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"That command had an unusable value: {error}.",
        )
    # Anything reaching here is a bug rather than a known refusal, and the
    # browser is deliberately told nothing useful. Log it: without this the only
    # trace of a real fault was a bare 500 that the proxy relabelled "gateway
    # unavailable", which points at the network and is exactly wrong.
    _LOG.exception("Unhandled arm gateway failure: %r", error)
    return HTTPException(status_code=500, detail="The arm gateway failed to run that command.")


def create_simple_arm_router(
    controller: ArmController,
    *,
    store: JointStore | None = None,
    arm_service: ArmService | None = None,
    camera_service: CameraEvidenceService | None = None,
) -> APIRouter:
    if arm_service is not None and store is not None:
        raise ValueError("Supply either an arm service or a joint store, not both.")
    service = arm_service or ArmService(controller, store or JointStore())
    router = APIRouter(prefix="/api/robot/arm", tags=["arm"])
    admission_lock = service.operation_admission_lock

    def _guard_operation(endpoint, *, allow_live_follow: bool):
        """Give each HTTP task distinct non-reentrant physical ownership."""

        if inspect.iscoroutinefunction(endpoint):
            @wraps(endpoint)
            async def guarded_async(*args, **kwargs):
                if not admission_lock.acquire(blocking=False):
                    raise HTTPException(
                        status_code=status.HTTP_409_CONFLICT,
                        detail="Another physical arm operation is in progress.",
                    )
                try:
                    if not allow_live_follow:
                        service.require_live_follow_idle()
                    return await endpoint(*args, **kwargs)
                finally:
                    admission_lock.release()

            return guarded_async

        @wraps(endpoint)
        def guarded_sync(*args, **kwargs):
            if not admission_lock.acquire(blocking=False):
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="Another physical arm operation is in progress.",
                )
            try:
                if not allow_live_follow:
                    service.require_live_follow_idle()
                return endpoint(*args, **kwargs)
            finally:
                admission_lock.release()

        return guarded_sync

    def guard_operation(endpoint):
        return _guard_operation(endpoint, allow_live_follow=False)

    def guard_live_follow_operation(endpoint):
        return _guard_operation(endpoint, allow_live_follow=True)

    def _joint(name: str) -> str:
        if name not in JOINT_IDS:
            raise HTTPException(status_code=404, detail="No such joint.")
        return name

    @router.get("/state")
    def read_state() -> dict[str, object]:
        try:
            return service.state()
        except Exception as error:
            raise _translate(error) from error

    @router.post("/live-follow/start")
    @guard_live_follow_operation
    async def start_live_follow(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, LiveFollowStartRequest)
        try:
            return await run_in_threadpool(service.start_live_follow, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/live-follow/start/cancel")
    async def cancel_live_follow_start(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, LiveFollowStartCancelRequest)
        try:
            return await run_in_threadpool(service.cancel_live_follow_start, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/live-follow/frame")
    @guard_live_follow_operation
    async def live_follow_frame(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, LiveFollowFrameRequest)
        try:
            return await run_in_threadpool(service.live_follow_frame, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/live-follow/heartbeat")
    async def live_follow_heartbeat(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, LiveFollowHeartbeatRequest)
        try:
            return await run_in_threadpool(service.live_follow_heartbeat, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/live-follow/end")
    @guard_live_follow_operation
    async def end_live_follow(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, LiveFollowEndRequest)
        try:
            return await run_in_threadpool(service.end_live_follow, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/plans/preview")
    @guard_operation
    async def preview_plan(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, PlanPreviewRequest)
        try:
            return service.preview_plan(request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/plans/execute")
    @guard_operation
    async def execute_plan(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, PlanExecuteRequest)
        try:
            return service.execute_plan(request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/plans/execute-and-capture")
    @guard_operation
    async def execute_and_capture(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, PlanExecuteCaptureRequest)
        try:
            # Arrival can legitimately take seconds. Keep FastAPI's event loop
            # free while the condition waits for the service-loop generations.
            return await run_in_threadpool(
                service.execute_and_capture,
                request,
                camera_service,
            )
        except Exception as error:
            raise _translate(error) from error

    @router.post("/sequences/preview")
    @guard_operation
    async def preview_sequence(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, SequencePreviewRequest)
        try:
            return service.preview_sequence(request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/sequences/execute")
    @guard_operation
    async def execute_sequence(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, SequenceExecuteRequest)
        try:
            return await run_in_threadpool(service.execute_sequence, request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/sequences/execute-and-capture")
    @guard_operation
    async def execute_sequence_and_capture(
        http_request: Request,
    ) -> dict[str, object]:
        request = await _strict_body(http_request, SequenceExecuteCaptureRequest)
        try:
            return await run_in_threadpool(
                service.execute_sequence_and_capture,
                request,
                camera_service,
            )
        except Exception as error:
            raise _translate(error) from error

    @router.post("/scan")
    @guard_operation
    def scan() -> dict[str, object]:
        try:
            return service.scan()
        except Exception as error:
            raise _translate(error) from error

    @router.post("/joints/{name}/calibrate")
    @guard_operation
    async def calibrate(name: str, http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, CalibrateRequest)
        try:
            return service.calibrate(_joint(name), request)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/target")
    @guard_operation
    async def multi_target(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, MultiTargetRequest)
        wanted = {
            name: value
            for name in JOINT_IDS
            if (value := getattr(request, name)) is not None
        }
        try:
            return service.targets(wanted)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/joints/{name}/target")
    @guard_operation
    async def target(name: str, http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, TargetRequest)
        try:
            return service.target(_joint(name), request.degrees)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/torque")
    @guard_operation
    async def torque(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, TorqueRequest)
        try:
            return service.torque(request.hold)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/floor-guard")
    @guard_operation
    async def floor_guard(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, FloorGuardRequest)
        try:
            return service.floor_guard(request.enabled)
        except Exception as error:
            raise _translate(error) from error

    @router.post("/registers")
    @guard_operation
    async def registers(http_request: Request) -> dict[str, object]:
        """Read when `length` is given; raw writes are deliberately disabled."""

        request = await _strict_body(http_request, RegisterRequest)
        if (request.length is None) == (request.values is None):
            raise HTTPException(
                status_code=400,
                detail="Give length to read or values to write, not both and not neither.",
            )
        try:
            if request.values is not None:
                raise ControllerCommandError("REGISTER_WRITE_DISABLED")
            return controller.read_servo_registers(request.servoId, request.address, request.length or 1)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        except Exception as error:
            raise _translate(error) from error

    @router.post("/servo-id")
    @guard_operation
    async def assign_id(http_request: Request) -> dict[str, object]:
        request = await _strict_body(http_request, AssignIdRequest)
        if request.oldId == request.newId:
            raise HTTPException(status_code=400, detail="That servo already has that ID.")
        try:
            return service.assign_id(request.oldId, request.newId)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        except Exception as error:
            raise _translate(error) from error

    @router.post("/stop")
    def stop() -> dict[str, object]:
        try:
            return service.stop()
        except Exception as error:
            raise _translate(error) from error

    @router.post("/clear-stop")
    @guard_operation
    def clear_stop() -> dict[str, object]:
        try:
            return service.clear_stop()
        except Exception as error:
            raise _translate(error) from error

    return router
