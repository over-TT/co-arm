"""Pure runtime policies for Isaac motion and camera evidence.

This module deliberately imports no Isaac or Omniverse packages so the
production bridge policies can be exercised by the ordinary Python test suite.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


# At the bridge's 60 Hz physics rate this limits successive articulation target
# changes to one degree, rather than teleporting a large command in one update.
# This is a simulator integration bound, not a physical-servo speed claim.
MAX_JOINT_DEGREES_PER_PHYSICS_STEP = 1.0

NON_BLACK_VALUE_THRESHOLD = 8.0
MIN_NON_BLACK_FRACTION = 0.02
MIN_FRAME_VARIANCE = 4.0


def interpolation_step_count(
    *,
    maximum_angular_delta_degrees: float,
    duration_ms: int,
    physics_hz: int,
    maximum_degrees_per_step: float = MAX_JOINT_DEGREES_PER_PHYSICS_STEP,
) -> int:
    """Return a deterministic target-ramp length for one joint command.

    Requested duration is a lower bound.  Angular travel supplies an
    independent lower bound, and every nonzero move receives at least two
    physics updates even when its travel is smaller than the per-step limit.
    """

    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
        raise ValueError("duration_ms must be a nonnegative integer")
    if duration_ms < 0:
        raise ValueError("duration_ms must be a nonnegative integer")
    if isinstance(physics_hz, bool) or not isinstance(physics_hz, int):
        raise ValueError("physics_hz must be a positive integer")
    if physics_hz <= 0:
        raise ValueError("physics_hz must be a positive integer")

    angular_delta = float(maximum_angular_delta_degrees)
    per_step = float(maximum_degrees_per_step)
    if not math.isfinite(angular_delta) or angular_delta < 0.0:
        raise ValueError(
            "maximum_angular_delta_degrees must be finite and nonnegative"
        )
    if not math.isfinite(per_step) or per_step <= 0.0:
        raise ValueError("maximum_degrees_per_step must be finite and positive")

    duration_steps = max(1, (duration_ms * physics_hz + 999) // 1000)
    if angular_delta == 0.0:
        angular_steps = 1
    else:
        angular_steps = max(2, int(math.ceil(angular_delta / per_step)))
    return max(duration_steps, angular_steps)


@dataclass(frozen=True, slots=True)
class FrameQuality:
    """Bounded evidence that a decoded RGB-like frame is not blank/uniform."""

    minimum: float
    maximum: float
    variance: float
    non_black_fraction: float

    @property
    def passed(self) -> bool:
        return (
            self.variance >= MIN_FRAME_VARIANCE
            and self.non_black_fraction >= MIN_NON_BLACK_FRACTION
        )

    def summary(self) -> str:
        return (
            f"min={self.minimum:.0f}, max={self.maximum:.0f}, "
            f"variance={self.variance:.3f}, "
            f"nonBlackFraction={self.non_black_fraction:.5f}"
        )


def assess_frame_quality(rgb_like: Any) -> FrameQuality:
    """Measure the bounded nonblank/nonuniform gate on decoded pixels."""

    values = np.asarray(rgb_like)
    if (
        values.ndim != 3
        or values.shape[2] < 3
        or values.shape[0] < 1
        or values.shape[1] < 1
    ):
        raise ValueError(
            "frame must contain at least one row, one column, and three channels"
        )
    rgb = values[:, :, :3]
    if not np.issubdtype(rgb.dtype, np.number):
        raise ValueError("frame pixels must be numeric")
    finite = np.asarray(rgb, dtype=np.float64)
    if not np.isfinite(finite).all():
        raise ValueError("frame pixels must be finite")

    non_black_fraction = float(
        (np.max(finite, axis=2) >= NON_BLACK_VALUE_THRESHOLD).mean()
    )
    # Use spatial variance inside each channel.  Variance across a constant
    # pixel's RGB components would otherwise make a solid-colour frame look
    # nonuniform even though it contains no spatial evidence.
    spatial_variance = max(
        float(finite[:, :, channel].var()) for channel in range(3)
    )
    return FrameQuality(
        minimum=float(finite.min()),
        maximum=float(finite.max()),
        variance=spatial_variance,
        non_black_fraction=non_black_fraction,
    )


__all__ = [
    "FrameQuality",
    "MAX_JOINT_DEGREES_PER_PHYSICS_STEP",
    "MIN_FRAME_VARIANCE",
    "MIN_NON_BLACK_FRACTION",
    "NON_BLACK_VALUE_THRESHOLD",
    "assess_frame_quality",
    "interpolation_step_count",
]
