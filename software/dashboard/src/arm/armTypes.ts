export type ArmJointId = "joint_1" | "joint_2" | "joint_3";
/** Driven and calibrated, but outside the kinematic chain. */
export type AuxJointId = "joint_4";
export type SimpleJointId = ArmJointId | AuxJointId;
export type ArmLogicalName = "base_yaw" | "shoulder_pitch" | "elbow_pitch";
export type ArmMode = "simulator" | "physical";

export interface ArmPose {
  joint_1: number;
  joint_2: number;
  joint_3: number;
}

export interface CartesianTarget {
  x: number;
  y: number;
  z: number;
}

export interface ArmGeometry {
  baseHeightMm: number;
  upperArmMm: number;
  forearmMm: number;
  toolOffsetMm: number;
}

export interface JointCalibration {
  logicalId: ArmJointId;
  name: ArmLogicalName;
  servoId: number;
  rawZero: number;
  direction: 1 | -1;
  rawMin: number;
  rawMax: number;
  logicalMinDegrees: number;
  logicalMaxDegrees: number;
  maxVelocityDegreesPerSecond: number;
  maxAccelerationDegreesPerSecond2: number;
  directionVerified: boolean;
}

export interface CalibrationDraft {
  joints: Record<ArmJointId, JointCalibration>;
  geometry: ArmGeometry;
}

export interface IkCandidate {
  branch: "elbow_up" | "elbow_down";
  pose: ArmPose;
  withinLimits: boolean;
  singular: boolean;
  distanceFromCurrent: number;
  violations: string[];
}

export interface IkPreview {
  reachable: boolean;
  reason: string | null;
  target: CartesianTarget;
  candidates: IkCandidate[];
  selectedCandidate: number | null;
}

export interface ArmGatewayStatus {
  mode: ArmMode;
  connection: "online" | "offline" | "demo";
  state: "calibration_required" | "disarmed" | "ready" | "stopped" | "faulted";
  stateRevision: number;
  calibrationRevision: number;
  calibrated: boolean;
  profileHash: string | null;
  currentPose: ArmPose;
  geometry: ArmGeometry;
  source: "gateway" | "browser_demo";
}

export interface CameraAutofocus {
  capability: "unknown" | "unsupported" | "supported";
  mode: "unknown" | "fixed" | "continuous";
  state: "not_started" | "fixed" | "configured" | "configuration_failed" | "closed" | "idle" | "scanning" | "focused" | "failed";
  range: "unavailable" | "normal" | "full" | "macro";
  lensPosition?: number;
}

export type CameraCaptureProfile = "survey" | "detail";

export interface CameraDimensions {
  width: number;
  height: number;
}

export interface CameraHardwareProfile {
  id: string;
  productName: string;
  sensorModel: string;
  lensVariant: string;
  nativeDimensions: CameraDimensions;
  nominalFocalLengthMm: number;
  nominalFieldOfViewDegrees: {
    horizontal: number;
    vertical: number;
  };
  captureProfiles: Record<CameraCaptureProfile, CameraDimensions>;
}

export type CameraAutofocusResult = "focused" | "not_focused" | "unsupported" | "unavailable" | "timed_out";

export interface CameraAutofocusResponse {
  simulated: boolean;
  physicalArmMotion: false;
  attempted: boolean;
  result: CameraAutofocusResult;
  autofocus: CameraAutofocus;
}

export interface CameraStatus {
  simulated: boolean;
  readOnly: boolean;
  cameraId: string;
  sensorModel: string;
  identityConfidence: string;
  state: string;
  available: boolean;
  latestFrameId: string | null;
  historySize: number;
  historyLimit: number;
  retainedBytes: number;
  byteLimit: number;
  autofocus: CameraAutofocus;
  cameraProfile?: CameraHardwareProfile;
}

export interface CameraObservation {
  simulated: boolean;
  readOnly: boolean;
  source: string;
  frameId: string;
  cameraId: string;
  sensorModel: string;
  identityConfidence: string;
  capturedAt: string;
  ageSeconds: number;
  dimensions: CameraDimensions;
  byteCount: number;
  sha256: string;
  contentSha256: string;
  stateRevision: number;
  frameUrl: string;
  autofocus: CameraAutofocus;
  captureProfile: CameraCaptureProfile;
  cameraProfile?: CameraHardwareProfile;
}

export type PhysicalMotionState = "blocked" | "disarmed" | "commissioning" | "holding" | "stopped" | "faulted" | "ready_disarmed";
export type PhysicalTorqueState = "off" | "on" | "unknown";
export type PhysicalControllerState = "online" | "offline" | "not_configured" | "faulted";
export type PhysicalBusState = "online" | "offline" | "unknown" | "degraded";
export type PhysicalServoCensusState = "online" | "partial" | "none" | "unknown" | "collision";

export interface PhysicalArmBlocker {
  code: string;
  message: string;
  recovery: string;
}

export interface PhysicalServoTelemetry {
  id: number;
  rawPosition: number;
  speed: number;
  load: number;
  voltageVolts: number;
  temperatureC: number;
  moving: boolean;
  operatingMode?: number | null;
  currentMilliamps?: number;
  currentRaw?: number;
  statusError?: number;
  online?: boolean;
  fresh?: boolean;
  torqueState: PhysicalTorqueState;
  packetAgeMs: number;
  errors: string[];
}

export interface PhysicalArmStatus {
  mode: "physical";
  motionState: PhysicalMotionState;
  torqueState: PhysicalTorqueState;
  lastUpdateMs: number | null;
  connections: {
    pi: { state: "online"; detail: string };
    controller: {
      state: PhysicalControllerState;
      detail: string;
      controllerId?: string;
      bootId?: string;
      firmwareVersion?: string;
      protocolVersion?: number;
      lastSeenMs?: number;
    };
    bus: {
      state: PhysicalBusState;
      detail: string;
      baud: number;
      lastScan?: { minId: number; maxId: number; foundIds: number[] };
    };
    servos: {
      state: PhysicalServoCensusState;
      detail: string;
      expectedIds: number[];
      respondingIds: number[];
    };
  };
  servos: PhysicalServoTelemetry[];
  blockers: PhysicalArmBlocker[];
  nextAction: string;
  stop: { state: "confirmed" | "not_latched" | "unknown"; detail: string };
  hardwareEstop: "not_detected" | "active" | "released" | "unknown";
}

export interface PhysicalJointCommissioning {
  servoId: number;
  rawZero: number | null;
  zeroEvidenceId: string | null;
  direction: 1 | -1 | null;
  motorTurnsPerJointTurn: number;
  ratioConfirmed: boolean;
  // Native signed absolute feedback, so this joint's range may span several
  // motor turns and each command still names one idempotent destination.
  multiTurn: boolean;
  zeroMultiTurnPosition: number | null;
  // A limit the operator typed rather than physically visited. Recorded so the
  // stored profile never claims a range the arm has actually been driven to.
  declaredLimits: boolean;
  observedNegativeTicks: number | null;
  observedPositiveTicks: number | null;
  negativeLimitTicks: number | null;
  positiveLimitTicks: number | null;
  limitMarginTicks: number;
  maxVelocityDegreesPerSecond: number;
  maxAccelerationDegreesPerSecond2: number;
  evidenceIds: string[];
  torqueHoldPassed: boolean;
  nudgePassed: boolean;
}

export interface PhysicalCommissioningDraft {
  joints: Record<ArmJointId, PhysicalJointCommissioning>;
  geometry: ArmGeometry;
  modelLabelConfirmed: boolean;
  mappingConfirmed: boolean;
  savedAt: string | null;
}

export interface PhysicalCalibrationProfileJoint {
  logicalId: ArmJointId;
  name: ArmLogicalName;
  servoId: number;
  rawZero: number;
  direction: 1 | -1;
  motorTurnsPerJointTurn: number;
  multiTurn: boolean;
  declaredLimits: boolean;
  negativeLimitTicks: number;
  positiveLimitTicks: number;
  limitMarginTicks: number;
  maxVelocityDegreesPerSecond: number;
  maxAccelerationDegreesPerSecond2: number;
  evidenceIds: string[];
}

export interface PhysicalCalibrationProfilePayload {
  expectedControllerBootId: string;
  confirmedServoModel: "ST3215";
  joints: Record<ArmJointId, PhysicalCalibrationProfileJoint>;
  geometry: ArmGeometry;
  acknowledgedRemainDisarmed: true;
}

export interface SimpleMultiTurnTruth {
  /** These are cached from the service loop's existing ODO read. Optional
   * fields keep a newer dashboard honest against an older gateway response. */
  tracking?: boolean;
  valid?: boolean;
  stepMode?: boolean;
  stepOutstanding?: boolean;
  countdownObserved?: boolean;
  resyncNeeded?: boolean;
  resyncCount?: number;
  sampleAgeMs?: number;
}

export interface SimpleControllerIdentity {
  controllerId?: string;
  bootId?: string;
  firmwareVersion?: string;
  protocolVersion?: number;
  /** Native extended absolute Base positioning is available end to end. */
  multiTurnAbsoluteV1?: boolean;
  /** True only when both the step-drive and truth-v3 capabilities are live. */
  multiTurnTruthV3?: boolean;
  /** True only when bounded Fast Follow commands and compact feedback are live. */
  liveFollowV1?: boolean;
}

/** A joint is four numbers: a zero, a min, a max and a gear ratio. */
export interface SimpleJoint {
  id: SimpleJointId;
  name: string;
  servoId: number;
  rawZero: number | null;
  rawMin: number | null;
  rawMax: number | null;
  ratio: number;
  direction: 1 | -1;
  speed: number;
  accel: number;
  online: boolean;
  rawPosition: number | null;
  degrees: number | null;
  /** Optional live telemetry facts exposed by newer gateways. */
  moving?: boolean | null;
  packetAgeMs?: number | null;
  /** False when a multi-turn command ledger lost encoder continuity. Controls
   * stay disabled until the controller performs a torque-off mode-0 resync. */
  positionTrusted: boolean;
  /** Present only for the multi-turn Base and never populated by a browser read. */
  multiTurnTruth?: SimpleMultiTurnTruth | null;
  minDegrees: number | null;
  maxDegrees: number | null;
  /** The declared limits intersected with the one motor turn a goal can name.
   *  Narrower than min/max whenever the gearing outruns a single turn. */
  reachMin?: number | null;
  reachMax?: number | null;
  rawLow: number;
  rawHigh: number;
  calibrated: boolean;
  torque: "off" | "on" | "unknown";
  temperatureC: number | null;
  voltageVolts: number | null;
}

export interface SimpleArmScanEvidence {
  foundIds: number[];
  /** Raw PING responders before the post-Scan telemetry proof is merged. */
  pingFoundIds?: number[];
  /** PING answered, but the immediate full telemetry sample was incomplete. */
  telemetryUnavailableIds?: number[];
  /** Fresh final telemetry recovered an ID missing from the first guarded
   *  post-Scan sample; the ID may already have answered PING. */
  telemetryRecoveredIds?: number[];
  /** Persistent controller-cache shape returned by normal state polls. */
  minId?: number;
  maxId?: number;
  /** Direct Scan-response shape. */
  completeRange?: { minId: number; maxId: number };
  collisionSuspected?: boolean;
  collisionId?: number | null;
}

export interface SimpleArmState {
  connection: string;
  bus: string;
  controller?: SimpleControllerIdentity;
  joints: SimpleJoint[];
  held: number[];
  stopped: boolean;
  /** Independent recovery-image motion gate. Clear STOP cannot retire it. */
  baseReferenceRequired?: boolean;
  /** Gateway-bounded reason; arbitrary marker content is normalized to invalid. */
  baseReferenceReason?:
    | "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED"
    | "BASE_REFERENCE_MARKER_INVALID"
    | null;
  /** Wire time of the last telemetry refresh; makes a saturated link visible. */
  refreshMs?: number;
  telemetryGeneration?: number;
  telemetryAgeMs?: number;
  /** The last scan suspected two servos answering one id. This is the only
   *  field that authorizes collision-specific recovery copy. */
  collisionSuspected?: boolean;
  collisionId?: number | null;
  /** Generic bus-health recovery copy. It must not infer an ID collision when
   *  the scan-backed collision fields above are absent. */
  busTrouble?: string | null;
  /** Sanitized evidence from the most recent completed Scan. Presence does not
   *  establish a live position or authorize torque/motion. */
  lastScan?: SimpleArmScanEvidence | null;
  /** Present on the direct Scan response. It is diagnostic evidence only and
   *  does not establish a trusted joint pose or home the Base. */
  scan?: SimpleArmScanEvidence;
  /** The keep-out plane, owned by the Pi so the drawing, the drag constraint and
   *  the server-side clamp all read one number. A second hand-maintained copy
   *  would drift, and a drifted floor is a floor you land on. */
  floorGuard?: {
    enabled: boolean;
    floorMm: number;
    geometry: { baseHeightMm: number; upperArmMm: number; distalMm: number };
  };
}

export type ArmLiveFollowJointId = "joint_2" | "joint_3";
export type ArmLiveFollowState = "active" | "ended" | "expired" | "faulted" | "stopped";

export interface ArmLiveFollowTargets {
  joint_2: number;
  joint_3: number;
}

export interface ArmLiveFollowJointSettings {
  /** Temporary raw ST3215 register units for this live-follow session. */
  speed: number;
  /** Temporary raw ST3215 register units for this live-follow session. */
  accel: number;
  /** Requested symmetric software envelope around the fresh session origin. */
  maxDeltaDegrees: number;
}

export type ArmLiveFollowSettings = Record<ArmLiveFollowJointId, ArmLiveFollowJointSettings>;

export interface ArmLiveFollowMeasuredJoint {
  degrees: number;
  rawPosition: number;
  /** Controller estimate only. The dashboard derives its displayed speed from
   * timestamped state-poll angle deltas instead of treating this as proof. */
  velocityDegreesPerSecond: number;
  moving: boolean;
  packetAgeMs: number;
}

export interface ArmLiveFollowReceipt {
  schema: "arm-live-follow-v2";
  sessionId: string;
  state: ArmLiveFollowState;
  reason: string | null;
  settings: ArmLiveFollowSettings;
  envelope: {
    maxDeltaDegrees: Record<ArmLiveFollowJointId, number>;
    floorMm: number;
    lowestPointMm: number;
    joint_2: { minDegrees: number; maxDegrees: number };
    joint_3: { minDegrees: number; maxDegrees: number };
  };
  commanded: ArmLiveFollowTargets;
  measured: Record<ArmLiveFollowJointId, ArmLiveFollowMeasuredJoint>;
  stats: {
    receivedFrames: number;
    dispatchedFrames: number;
    coalescedFrames: number;
    clampedFrames: number;
    lastAcceptedSequence: number;
    lastDispatchedSequence: number;
    lastDispatchLatencyMs: number | null;
  };
  timing: {
    inputLeaseMs: number;
    maxDurationMs: number;
    minDispatchIntervalMs: number;
    startedAt: string;
    expiresAt: string;
  };
}

/** A non-moving, one-time arm plan prepared by the Raspberry Pi. All angles in
 * these payloads are logical servo degrees (the same numbers shown by the joint
 * cards), not the 2D twin's shoulder/elbow frame. */
export interface SimpleArmPlan {
  planId: string;
  planDigest: string;
  expiresAt: string | null;
  expiresInMs: number | null;
  measuredPose: Partial<Record<SimpleJointId, number>>;
  resolvedPose: Partial<Record<SimpleJointId, number>>;
  warnings: string[];
  lowestClearanceMm: number | null;
  /** Geometry is optional so the dashboard remains compatible with a planner
   * that has not started returning the richer camera-ray scene yet. */
  scene?: Record<string, unknown> | null;
}

export interface SimpleArmPlanExecution {
  executed: true;
  planId: string;
  resolvedPose: Partial<Record<SimpleJointId, number>>;
  moved: SimpleJointId[];
}

export interface SimpleArmSequenceWaypointInput {
  targets: Partial<Record<SimpleJointId, number>>;
  tip?: { radialMm: number; heightMm: number };
  elbowPreference?: "nearest" | "up" | "down";
  label?: string;
}

export interface SimpleArmSequencePlanWaypoint {
  index: number;
  label?: string;
  startPose: Partial<Record<SimpleJointId, number>>;
  resolvedPose: Partial<Record<SimpleJointId, number>>;
  warnings: string[];
  lowestClearanceMm: number | null;
  scene?: Record<string, unknown> | null;
  ik?: Record<string, unknown> | null;
}

export interface SimpleArmSequencePlan {
  sequenceId: string;
  sequenceDigest: string;
  expiresAt: string | null;
  expiresInMs: number | null;
  previewDurationMs: number | null;
  measuredPose: Partial<Record<SimpleJointId, number>>;
  waypointCount: number;
  waypoints: SimpleArmSequencePlanWaypoint[];
  warnings: string[];
  lowestClearanceMm: number | null;
}

export interface SimpleArmSequenceWaypointResult {
  index: number;
  label?: string;
  outcome: string;
  dispatched: boolean;
  moved?: Array<{
    joint: SimpleJointId;
    servoId: number;
    goal: number;
    degrees: number;
  }>;
  arrival: Record<string, unknown>;
  dispatchMs?: number | null;
  arrivalMs?: number | null;
  durationMs?: number;
}

export interface SimpleArmSequenceExecution {
  operationId: string;
  sequenceId: string;
  executed: boolean;
  outcome: string;
  waypointCount: number;
  completedWaypointCount: number;
  failedWaypointIndex: number | null;
  waypointResults: SimpleArmSequenceWaypointResult[];
  resolvedPose?: Partial<Record<SimpleJointId, number>>;
  finalMeasuredPose?: Partial<Record<SimpleJointId, number>>;
  durationMs?: number;
  captureAttempted?: boolean;
  capture?: Record<string, unknown> | null;
}

export interface SimpleCalibratePatch {
  rawZero?: number;
  rawMin?: number;
  rawMax?: number;
  minDegrees?: number;
  maxDegrees?: number;
  ratio?: number;
  direction?: 1 | -1;
  speed?: number;
  accel?: number;
  /** Which servo this joint drives. Not the same as burning an ID into a servo
   *  — that is `assignId`, and it needs a bus with one servo on it. */
  servoId?: number;
  zeroFromLimits?: boolean;
  clear?: Array<"rawZero" | "rawMin" | "rawMax">;
  /** Store where the joint is *now*, read on the Pi as the request lands. The
   *  browser's rawPosition is up to a poll and a refresh stale, which on the
   *  camera servo is tens of degrees. */
  here?: Array<"rawZero" | "rawMin" | "rawMax">;
  /** Current acknowledgement from the operator's explicit Set zero here action. */
  confirmedPhysicalBaseZero?: true;
}

export interface PhysicalProfileEnvelope {
  mode: "physical";
  committed: boolean;
  motionState: PhysicalMotionState;
  profileRevision: number;
  profileHash: string | null;
  profile: {
    joints: Record<ArmJointId, PhysicalCalibrationProfileJoint>;
    geometry: ArmGeometry;
  } | null;
}

/** Motor ticks the joint travels for one degree at the joint itself. */
export function ticksPerDegree(motorTurnsPerJointTurn: number) {
  return 4096 * motorTurnsPerJointTurn / 360;
}

/**
 * The raw window a joint can actually be commanded to. The ST3215 takes a goal
 * inside one turn, so a geared joint's saved range may be wider than what a
 * single MOVE can reach; clamping here is what stops the UI promising travel the
 * servo will silently refuse.
 */
export function drivableRange(joint: PhysicalCalibrationProfileJoint) {
  const perDegree = ticksPerDegree(joint.motorTurnsPerJointTurn);
  const negativeRaw = joint.rawZero + joint.direction * joint.negativeLimitTicks;
  const positiveRaw = joint.rawZero + joint.direction * joint.positiveLimitTicks;
  const rawMin = Math.max(0, Math.min(negativeRaw, positiveRaw));
  const rawMax = Math.min(4095, Math.max(negativeRaw, positiveRaw));
  // Back out of raw space through the joint's own sign so min/max stay ordered
  // in degrees regardless of which way the encoder counts.
  const edges = [rawMin, rawMax].map((raw) => (raw - joint.rawZero) * joint.direction / perDegree);
  return {
    rawMin,
    rawMax,
    minDegrees: Math.max(-180, Math.min(...edges)),
    maxDegrees: Math.min(180, Math.max(...edges)),
  };
}

/** Degrees at the joint -> a raw goal the controller will accept, or null. */
export function goalForDegrees(joint: PhysicalCalibrationProfileJoint, degrees: number) {
  const range = drivableRange(joint);
  const bounded = Math.max(range.minDegrees, Math.min(range.maxDegrees, degrees));
  const goal = Math.round(joint.rawZero + bounded * ticksPerDegree(joint.motorTurnsPerJointTurn) * joint.direction);
  if (!Number.isFinite(goal) || goal < 0 || goal > 4095) return null;
  return goal;
}

export function signedServoDelta(rawPosition: number, rawZero: number) {
  const normalized = ((rawPosition - rawZero + 2048) % 4096 + 4096) % 4096;
  return normalized - 2048;
}

export const ARM_JOINTS: readonly ArmJointId[] = ["joint_1", "joint_2", "joint_3"];

/**
 * Joints that are driven and calibrated but are not part of the arm.
 *
 * The camera joint sits on the end of the arm rather than in it, so it has no
 * link length and contributes nothing to where the tool ends up. Keeping it out
 * of `ARM_JOINTS` is what stops it reaching the kinematics, the 3D twin and the
 * side elevation — all of which are about the chain. It still gets a full joint
 * card and its own dial.
 */
export const AUX_JOINTS: readonly AuxJointId[] = ["joint_4"];
export const ALL_JOINTS: readonly SimpleJointId[] = [...ARM_JOINTS, ...AUX_JOINTS];

export const JOINT_LABELS: Record<ArmJointId, string> = {
  joint_1: "Base yaw",
  joint_2: "Shoulder",
  joint_3: "Elbow",
};

// Measured off the physical arm 2026-08-07. The 2 cm the two link plates are
// offset by is already inside these lengths -- it is not a separate term.
// forearm + toolOffset is the elbow-to-tip distance with the camera fully
// extended forward, i.e. the worst case for hitting anything.
export const DEFAULT_GEOMETRY: ArmGeometry = {
  baseHeightMm: 60,
  upperArmMm: 180,
  forearmMm: 180,
  toolOffsetMm: 40,
};

/**
 * Keep-out plane, millimetres above the floor. Nothing on the arm may go below
 * it while the floor guard is on.
 *
 * Deliberately NOT a calibration limit: calibration is each joint's range of
 * travel, this is a region in space the whole arm is kept out of, and the two
 * are enforced independently. Reach is 180+220 = 400 mm from a pivot only
 * 60 mm up, so the arm can comfortably drive itself through the table -- this
 * plane is doing real work.
 */
export const DEFAULT_FLOOR_MM = 40;

export const DEFAULT_POSE: ArmPose = {
  joint_1: 0,
  joint_2: 18,
  joint_3: -32,
};

export const DEFAULT_CALIBRATION: CalibrationDraft = {
  joints: {
    joint_1: {
      logicalId: "joint_1",
      name: "base_yaw",
      servoId: 1,
      rawZero: 2047,
      direction: 1,
      rawMin: 1100,
      rawMax: 2994,
      logicalMinDegrees: -45,
      logicalMaxDegrees: 45,
      maxVelocityDegreesPerSecond: 15,
      maxAccelerationDegreesPerSecond2: 30,
      directionVerified: false,
    },
    joint_2: {
      logicalId: "joint_2",
      name: "shoulder_pitch",
      servoId: 2,
      rawZero: 2047,
      direction: 1,
      rawMin: 1535,
      rawMax: 2559,
      logicalMinDegrees: -45,
      logicalMaxDegrees: 45,
      maxVelocityDegreesPerSecond: 12,
      maxAccelerationDegreesPerSecond2: 24,
      directionVerified: false,
    },
    joint_3: {
      logicalId: "joint_3",
      name: "elbow_pitch",
      servoId: 3,
      rawZero: 2047,
      direction: 1,
      rawMin: 1535,
      rawMax: 2559,
      logicalMinDegrees: -45,
      logicalMaxDegrees: 45,
      maxVelocityDegreesPerSecond: 12,
      maxAccelerationDegreesPerSecond2: 24,
      directionVerified: false,
    },
  },
  geometry: DEFAULT_GEOMETRY,
};
