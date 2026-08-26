import { describe, expect, it } from "vitest";
import { drivableRange, goalForDegrees, ticksPerDegree, type PhysicalCalibrationProfileJoint } from "./armTypes";

function joint(overrides: Partial<PhysicalCalibrationProfileJoint> = {}): PhysicalCalibrationProfileJoint {
  return {
    logicalId: "joint_1",
    name: "base_yaw",
    servoId: 1,
    rawZero: 2048,
    direction: 1,
    motorTurnsPerJointTurn: 1,
    multiTurn: false,
    declaredLimits: false,
    negativeLimitTicks: -1000,
    positiveLimitTicks: 1000,
    limitMarginTicks: 32,
    maxVelocityDegreesPerSecond: 20,
    maxAccelerationDegreesPerSecond2: 40,
    evidenceIds: ["evidence-1"],
    ...overrides,
  };
}

describe("physical joint drive maths", () => {
  it("scales ticks per degree by the gear ratio", () => {
    expect(ticksPerDegree(1)).toBeCloseTo(11.378, 3);
    expect(ticksPerDegree(8)).toBeCloseTo(91.022, 3);
  });

  it("keeps min below max when the encoder counts backwards", () => {
    const range = drivableRange(joint({ direction: -1 }));
    expect(range.minDegrees).toBeLessThan(0);
    expect(range.maxDegrees).toBeGreaterThan(0);
    expect(range.rawMin).toBe(1048);
    expect(range.rawMax).toBe(3048);
  });

  it("clamps a geared joint to the single turn the servo can actually be commanded to", () => {
    // 8:1 with a saved range of four motor turns: only the part inside 0..4095
    // is reachable by a position-mode goal, so that is all the UI may offer.
    const range = drivableRange(joint({ motorTurnsPerJointTurn: 8, negativeLimitTicks: -8000, positiveLimitTicks: 8000 }));
    expect(range.rawMin).toBe(0);
    expect(range.rawMax).toBe(4095);
    expect(range.minDegrees).toBeCloseTo(-2048 / ticksPerDegree(8), 3);
    expect(range.maxDegrees).toBeCloseTo(2047 / ticksPerDegree(8), 3);
  });

  it("converts degrees to a raw goal and mirrors it with direction", () => {
    expect(goalForDegrees(joint(), 45)).toBe(2560);
    expect(goalForDegrees(joint({ direction: -1 }), 45)).toBe(1536);
    expect(goalForDegrees(joint(), 0)).toBe(2048);
  });

  it("clamps out-of-range demands instead of producing an unreachable goal", () => {
    expect(goalForDegrees(joint(), 500)).toBe(3048);
    expect(goalForDegrees(joint(), -500)).toBe(1048);
  });
});
