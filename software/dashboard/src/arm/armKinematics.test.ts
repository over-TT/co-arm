import { describe, expect, it } from "vitest";
import { forwardKinematics, inverseKinematics, lowestPointMm } from "./armKinematics";
import { DEFAULT_CALIBRATION, DEFAULT_GEOMETRY, type ArmPose } from "./armTypes";

describe("arm kinematics", () => {
  it("round-trips a reachable pose through Cartesian space", () => {
    const pose: ArmPose = { joint_1: 12, joint_2: 20, joint_3: -30 };
    const target = forwardKinematics(pose, DEFAULT_CALIBRATION.geometry);
    const preview = inverseKinematics(target, pose, DEFAULT_CALIBRATION);

    expect(preview.reachable).toBe(true);
    expect(preview.candidates).toHaveLength(2);
    const selected = preview.selectedCandidate === null ? null : preview.candidates[preview.selectedCandidate];
    expect(selected?.withinLimits).toBe(true);
    expect(selected?.distanceFromCurrent).toBeLessThan(0.01);
  });

  it("rejects a point outside the measured reach", () => {
    const preview = inverseKinematics(
      { x: 2_000, y: 0, z: 2_000 },
      { joint_1: 0, joint_2: 0, joint_3: 0 },
      DEFAULT_CALIBRATION,
    );

    expect(preview.reachable).toBe(false);
    expect(preview.candidates).toEqual([]);
    expect(preview.reason).toMatch(/outside/i);
  });

  it("reports the elbow, not the tip, when the elbow is what is buried", () => {
    // Shoulder straight down folds the tip back up: tip is well above the
    // floor while the elbow is 120 mm underground. A tip-only guard passes
    // this pose, which is the whole reason the predicate takes the minimum.
    const pose: ArmPose = { joint_1: 0, joint_2: -90, joint_3: 90 };
    const elbowZ = DEFAULT_GEOMETRY.baseHeightMm - DEFAULT_GEOMETRY.upperArmMm;

    expect(elbowZ).toBe(-120);
    expect(lowestPointMm(pose, DEFAULT_GEOMETRY)).toBeCloseTo(elbowZ, 3);
  });

  it("ignores base rotation, which cannot change any height", () => {
    const geometry = DEFAULT_GEOMETRY;
    const flat = lowestPointMm({ joint_1: 0, joint_2: 10, joint_3: -20 }, geometry);

    for (const joint_1 of [-175, -90, 45, 175]) {
      expect(lowestPointMm({ joint_1, joint_2: 10, joint_3: -20 }, geometry)).toBeCloseTo(flat, 6);
    }
  });

  it("agrees with forward kinematics about the tip when the tip is lowest", () => {
    const pose: ArmPose = { joint_1: 0, joint_2: 0, joint_3: -60 };
    const tipZ = forwardKinematics(pose, DEFAULT_GEOMETRY).z;

    expect(tipZ).toBeLessThan(DEFAULT_GEOMETRY.baseHeightMm);
    expect(lowestPointMm(pose, DEFAULT_GEOMETRY)).toBeCloseTo(tipZ, 3);
  });

  it("distinguishes geometric reach from calibrated soft limits", () => {
    const target = forwardKinematics(
      { joint_1: 90, joint_2: 0, joint_3: 0 },
      DEFAULT_CALIBRATION.geometry,
    );
    const preview = inverseKinematics(target, { joint_1: 0, joint_2: 0, joint_3: 0 }, DEFAULT_CALIBRATION);

    expect(preview.reachable).toBe(false);
    expect(preview.candidates).toHaveLength(2);
    expect(preview.reason).toMatch(/soft limits/i);
  });
});
