from __future__ import annotations

import cv2
import numpy as np
import pytest

from arm_sim.isaac.runtime_policy import (
    MAX_JOINT_DEGREES_PER_PHYSICS_STEP,
    assess_frame_quality,
    interpolation_step_count,
)


def test_zero_duration_nonzero_motion_never_collapses_to_one_physics_step() -> None:
    assert (
        interpolation_step_count(
            maximum_angular_delta_degrees=0.001,
            duration_ms=0,
            physics_hz=60,
        )
        == 2
    )


def test_motion_steps_bound_each_interpolated_angular_increment() -> None:
    steps = interpolation_step_count(
        maximum_angular_delta_degrees=30.25,
        duration_ms=0,
        physics_hz=60,
    )

    assert steps == 31
    assert 30.25 / steps <= MAX_JOINT_DEGREES_PER_PHYSICS_STEP


def test_requested_duration_remains_an_independent_lower_bound() -> None:
    assert (
        interpolation_step_count(
            maximum_angular_delta_degrees=3.0,
            duration_ms=1_000,
            physics_hz=60,
        )
        == 60
    )
    assert (
        interpolation_step_count(
            maximum_angular_delta_degrees=0.0,
            duration_ms=0,
            physics_hz=60,
        )
        == 1
    )


@pytest.mark.parametrize(
    "frame",
    [
        np.zeros((8, 8, 3), dtype=np.uint8),
        np.full((8, 8, 3), 127, dtype=np.uint8),
    ],
)
def test_decoded_blank_or_uniform_frame_fails_quality_gate(frame: np.ndarray) -> None:
    assert assess_frame_quality(frame).passed is False


def test_decoded_nonblank_nonuniform_frame_passes_quality_gate() -> None:
    frame = np.full((16, 16, 3), 32, dtype=np.uint8)
    frame[:, 8:, :] = 192

    quality = assess_frame_quality(frame)

    assert quality.passed is True
    assert quality.non_black_fraction == 1.0
    assert quality.variance > 4.0


def test_jpeg_round_trip_can_collapse_raw_variation_and_must_be_rechecked() -> None:
    checker = (np.indices((64, 64)).sum(axis=0) % 2).astype(np.int16)
    raw = np.full((64, 64, 3), 128, dtype=np.int16)
    raw += checker[:, :, None] * np.array([-6, -6, 3], dtype=np.int16)
    raw = raw.astype(np.uint8)
    assert assess_frame_quality(raw).passed is True

    encoded_ok, encoded = cv2.imencode(
        ".jpg",
        raw,
        [int(cv2.IMWRITE_JPEG_QUALITY), 90],
    )
    assert encoded_ok is True
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    assert decoded is not None
    assert assess_frame_quality(decoded).passed is False
