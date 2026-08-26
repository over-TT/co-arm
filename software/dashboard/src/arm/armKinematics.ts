import {
  ARM_JOINTS,
  type ArmGeometry,
  type ArmPose,
  type CalibrationDraft,
  type CartesianTarget,
  type IkCandidate,
  type IkPreview,
} from "./armTypes";

const radians = (degrees: number) => (degrees * Math.PI) / 180;
const degrees = (value: number) => (value * 180) / Math.PI;
const finite = (value: number) => Number.isFinite(value);
const round = (value: number, places = 3) => Number(value.toFixed(places));

export function forwardKinematics(pose: ArmPose, geometry: ArmGeometry): CartesianTarget {
  const base = radians(pose.joint_1);
  const shoulder = radians(pose.joint_2);
  const elbow = radians(pose.joint_3);
  const distalLength = geometry.forearmMm + geometry.toolOffsetMm;
  const radial =
    geometry.upperArmMm * Math.cos(shoulder) +
    distalLength * Math.cos(shoulder + elbow);
  const z =
    geometry.baseHeightMm +
    geometry.upperArmMm * Math.sin(shoulder) +
    distalLength * Math.sin(shoulder + elbow);
  return {
    x: round(radial * Math.cos(base)),
    y: round(radial * Math.sin(base)),
    z: round(z),
  };
}

/**
 * Height of the LOWEST part of the arm above the floor, in millimetres.
 *
 * Both links are straight segments, so a segment's minimum height is always at
 * one of its endpoints. Testing the elbow and the tip is therefore exhaustive --
 * no sampling along the links. The shoulder pivot is fixed at baseHeightMm and
 * cannot move, so it never decides the answer.
 *
 * joint_1 is absent because base rotation is a yaw: it swings the arm around
 * without changing any height.
 *
 * Guarding only the tip would be wrong. At shoulder -90 the ELBOW sits at
 * baseHeight - upperArm (buried) while the tip can be back above the floor.
 */
export function lowestPointMm(pose: ArmPose, geometry: ArmGeometry): number {
  const distal = geometry.forearmMm + geometry.toolOffsetMm;
  const elbow = geometry.baseHeightMm + geometry.upperArmMm * Math.sin(radians(pose.joint_2));
  const tip = elbow + distal * Math.sin(radians(pose.joint_2 + pose.joint_3));
  return round(Math.min(elbow, tip));
}

function poseDistance(a: ArmPose, b: ArmPose) {
  return Math.sqrt(
    ARM_JOINTS.reduce((sum, joint) => sum + (a[joint] - b[joint]) ** 2, 0),
  );
}

function candidate(
  branch: IkCandidate["branch"],
  pose: ArmPose,
  current: ArmPose,
  calibration: CalibrationDraft,
): IkCandidate {
  const violations = ARM_JOINTS.flatMap((joint) => {
    const configured = calibration.joints[joint];
    if (pose[joint] < configured.logicalMinDegrees - 1e-6) return [`${joint} below its soft minimum`];
    if (pose[joint] > configured.logicalMaxDegrees + 1e-6) return [`${joint} above its soft maximum`];
    return [];
  });
  const singular = Math.abs(Math.sin(radians(pose.joint_3))) < 0.025;
  return {
    branch,
    pose: {
      joint_1: round(pose.joint_1),
      joint_2: round(pose.joint_2),
      joint_3: round(pose.joint_3),
    },
    withinLimits: violations.length === 0,
    singular,
    distanceFromCurrent: round(poseDistance(pose, current)),
    violations,
  };
}

export function inverseKinematics(
  target: CartesianTarget,
  current: ArmPose,
  calibration: CalibrationDraft,
): IkPreview {
  if (![target.x, target.y, target.z].every(finite)) {
    return { reachable: false, reason: "Target coordinates must be finite.", target, candidates: [], selectedCandidate: null };
  }
  const geometry = calibration.geometry;
  const radial = Math.hypot(target.x, target.y);
  const vertical = target.z - geometry.baseHeightMm;
  const upper = geometry.upperArmMm;
  const distal = geometry.forearmMm + geometry.toolOffsetMm;
  const distanceSquared = radial ** 2 + vertical ** 2;
  const cosineElbow = (distanceSquared - upper ** 2 - distal ** 2) / (2 * upper * distal);
  if (cosineElbow < -1 - 1e-8 || cosineElbow > 1 + 1e-8) {
    return {
      reachable: false,
      reason: "The target is outside the measured arm reach.",
      target,
      candidates: [],
      selectedCandidate: null,
    };
  }
  const base = degrees(Math.atan2(target.y, target.x));
  const elbowMagnitude = Math.acos(Math.max(-1, Math.min(1, cosineElbow)));
  const candidates = [elbowMagnitude, -elbowMagnitude].map((elbow, index) => {
    const shoulder =
      Math.atan2(vertical, radial) -
      Math.atan2(distal * Math.sin(elbow), upper + distal * Math.cos(elbow));
    return candidate(
      index === 0 ? "elbow_down" : "elbow_up",
      { joint_1: base, joint_2: degrees(shoulder), joint_3: degrees(elbow) },
      current,
      calibration,
    );
  });
  const viable = candidates
    .map((entry, index) => ({ entry, index }))
    .filter(({ entry }) => entry.withinLimits)
    .sort((a, b) => a.entry.distanceFromCurrent - b.entry.distanceFromCurrent);
  return {
    reachable: viable.length > 0,
    reason: viable.length > 0 ? null : "The arm can reach that point geometrically, but not inside the calibrated soft limits.",
    target,
    candidates,
    selectedCandidate: viable[0]?.index ?? null,
  };
}

export interface PlanarSolution {
  shoulder: number;
  elbow: number;
}

/**
 * Two-link inverse kinematics in the shoulder's own plane, same frame as
 * `forwardKinematics`. Both elbow branches, nearest first is the caller's job.
 *
 * Deliberately cannot refuse. `inverseKinematics` reports an out-of-reach target
 * because a commissioning flow needs to know; a hand dragging the tool wants the
 * arm to follow as far as it goes, so a target outside the annulus is pulled onto
 * the nearest edge of it and solved there.
 *
 * `radial` and `height` are millimetres from the shoulder pivot, not from the
 * floor — the base height is the caller's frame.
 */
export function planarInverse(
  radial: number,
  height: number,
  geometry: ArmGeometry,
): [PlanarSolution, PlanarSolution] {
  const upper = geometry.upperArmMm;
  const distal = geometry.forearmMm + geometry.toolOffsetMm;
  const distance = Math.hypot(radial, height);
  // The dead zone at the centre is as real as the outer limit: with unequal
  // links there is a hole the tool cannot enter.
  const held = Math.min(upper + distal, Math.max(Math.abs(upper - distal) + 1e-6, distance));
  const ratio = distance < 1e-6 ? 0 : held / distance;
  // Straight up out of a dead centre, rather than an undefined atan2.
  const onRing = distance < 1e-6 ? { r: 0, h: held } : { r: radial * ratio, h: height * ratio };
  const cosine = Math.max(-1, Math.min(1, (held ** 2 - upper ** 2 - distal ** 2) / (2 * upper * distal)));
  const magnitude = Math.acos(cosine);
  return [magnitude, -magnitude].map((elbow) => ({
    elbow: round(degrees(elbow)),
    shoulder: round(degrees(
      Math.atan2(onRing.h, onRing.r) -
      Math.atan2(distal * Math.sin(elbow), upper + distal * Math.cos(elbow)),
    )),
  })) as [PlanarSolution, PlanarSolution];
}

export function clampTarget(target: CartesianTarget): CartesianTarget {
  const clamp = (value: number, minimum: number, maximum: number) =>
    Math.min(maximum, Math.max(minimum, finite(value) ? value : 0));
  return {
    x: round(clamp(target.x, -450, 450), 1),
    y: round(clamp(target.y, -450, 450), 1),
    z: round(clamp(target.z, 0, 520), 1),
  };
}
