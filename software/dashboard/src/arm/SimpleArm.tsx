import { useCallback, useEffect, useMemo, useRef, useState, type ChangeEvent, type KeyboardEvent } from "react";
import { Arm2D, type JointLimit } from "./Arm2D";
import { ArmApiError, createCameraApi, createSimpleArmApi, loadActionToken } from "./armApi";
import { forwardKinematics, inverseKinematics } from "./armKinematics";
import { ArmViewport } from "./ArmViewport";
import {
  ALL_JOINTS,
  ARM_JOINTS,
  AUX_JOINTS,
  DEFAULT_FLOOR_MM,
  DEFAULT_GEOMETRY,
  type ArmJointId,
  type SimpleJointId,
  type ArmPose,
  type CalibrationDraft,
  type CameraAutofocusResult,
  type CameraCaptureProfile,
  type CameraHardwareProfile,
  type CameraObservation,
  type CameraStatus,
  type CartesianTarget,
  type SimpleArmPlan,
  type SimpleArmSequenceExecution,
  type SimpleArmSequencePlan,
  type SimpleArmSequenceWaypointInput,
  type SimpleArmState,
  type SimpleCalibratePatch,
  type SimpleJoint,
} from "./armTypes";
import "./simple-arm.css";

const POLL_MS = 150;
const IDLE_POLLS_BEFORE_RETIRING_MISSED_GOAL = 6;
const BASE_TEST_ANGLES = [-120, -30, 0, 30, 120] as const;

type BaseAction = {
  kind: "home" | "range" | "move";
  phase: "sending" | "accepted" | "failed";
  target?: number;
  detail?: string;
};

type PlanPhase = "idle" | "draft" | "planning" | "ready" | "applying" | "executing" | "error";
type MotionMode = "single" | "sequence";
type SequencePhase = "idle" | "draft" | "planning" | "ready" | "applying" | "complete" | "error";
type SequenceDraft = { id: number; targets: Partial<Record<SimpleJointId, number>> };
type PreparedSingleMotion =
  | { kind: "plan"; value: SimpleArmPlan }
  | { kind: "floor-route"; value: SimpleArmSequencePlan };
type CameraBusyState = "status" | "capture-survey" | "capture-detail" | "autofocus" | null;

const FLOOR_SWEEP_REFUSAL = "The independently timed Shoulder/Elbow sweep crosses the floor keep-out plane. Choose another pose.";

const routeAngle = (value: number) => Math.round(value * 1_000_000) / 1_000_000;

/**
 * Candidate geometry only. The browser never decides whether one is safe:
 * every leg goes through the Pi's authoritative sequence preview before the UI
 * can display or apply it.
 */
function floorRouteCandidates(
  measured: Partial<Record<SimpleJointId, number>>,
  requested: Partial<Record<SimpleJointId, number>>,
): SimpleArmSequenceWaypointInput[][] {
  const startShoulder = measured.joint_2;
  const startElbow = measured.joint_3;
  const finalShoulder = requested.joint_2 ?? startShoulder;
  const finalElbow = requested.joint_3 ?? startElbow;
  if (![startShoulder, startElbow, finalShoulder, finalElbow].every((value) => typeof value === "number" && Number.isFinite(value))) return [];

  const shoulder = finalShoulder as number;
  const elbow = finalElbow as number;
  const finalTargets = { ...requested, joint_2: shoulder, joint_3: elbow };
  const candidates: SimpleArmSequenceWaypointInput[][] = [
    [
      { targets: { joint_3: elbow }, label: "Floor-safe Elbow first" },
      { targets: finalTargets, label: "Destination" },
    ],
    [
      { targets: { joint_2: shoulder }, label: "Floor-safe Shoulder first" },
      { targets: finalTargets, label: "Destination" },
    ],
  ];
  for (let legs = 2; legs <= 8; legs += 1) {
    candidates.push(Array.from({ length: legs }, (_, index) => {
      const step = index + 1;
      if (step === legs) return { targets: finalTargets, label: "Destination" };
      const fraction = step / legs;
      return {
        targets: {
          joint_2: routeAngle((startShoulder as number) + (shoulder - (startShoulder as number)) * fraction),
          joint_3: routeAngle((startElbow as number) + (elbow - (startElbow as number)) * fraction),
        },
        label: `Floor-safe leg ${step}`,
      };
    }));
  }

  const seen = new Set<string>();
  return candidates.filter((candidate) => {
    const key = JSON.stringify(candidate.map((waypoint) => waypoint.targets));
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
}

function isInitialFloorSweepRefusal(error: unknown) {
  return error instanceof ArmApiError && error.status === 409 && error.message === FLOOR_SWEEP_REFUSAL;
}

function isRoutedFloorSweepRefusal(error: unknown) {
  return error instanceof ArmApiError && error.status === 409 && error.message.endsWith(FLOOR_SWEEP_REFUSAL);
}

const CAMERA_AUTOFOCUS_MESSAGES: Record<CameraAutofocusResult, string> = {
  focused: "Camera reported focus. Take a picture to check whether the subject looks sharper.",
  not_focused: "The camera could not lock focus. Try again with more light or more detail in the frame.",
  unsupported: "This camera source reports no motorized autofocus support.",
  timed_out: "Autofocus timed out before the camera reported a result. Try again.",
  unavailable: "Autofocus could not run. The autofocus controls or camera may be unavailable. Check camera status before trying again.",
};
/**
 * Where each joint's own zero sits in the twin's frame.
 *
 * The twin measures the shoulder from horizontal and the elbow as a deviation
 * from straight; the arm reads zero at the middle of each joint's travel. On the
 * real arm both joints at zero stand the upper arm up with the forearm out at a
 * right angle — an L — so the shoulder's zero is the twin's 90° and the elbow's
 * zero is the twin's −90°. Nothing else in the app knows about this offset.
 */
export const MODEL_OFFSET: Record<ArmJointId, number> = { joint_1: 0, joint_2: 90, joint_3: -90 };

export const toModel = (id: ArmJointId, degrees: number) => degrees + MODEL_OFFSET[id];
export const fromModel = (id: ArmJointId, degrees: number) => degrees - MODEL_OFFSET[id];

// Internal fallback for kinematic math only. The UI never renders this as a
// measured pose unless every arm joint has fresh, trusted telemetry.
const ZERO_POSE: ArmPose = { joint_1: 0, joint_2: MODEL_OFFSET.joint_2, joint_3: MODEL_OFFSET.joint_3 };

// Compatibility boundary for older Pi deployments, which incorrectly mapped
// every generic `bus=faulted` state to a duplicate-ID diagnosis. The browser
// only shows collision instructions from the separate scan-backed fields.
const FAULTED_BUS_TROUBLE = "Servo-bus communication failed, so live joint telemetry is unavailable. Support the arm. Run Scan for a fresh diagnosis. If Scan does not report an ID collision, check the 12 V servo rail and bus cabling. Do not change servo IDs without scan-backed collision evidence.";

function byId(state: SimpleArmState | null) {
  const table = {} as Record<SimpleJointId, SimpleJoint | undefined>;
  for (const joint of state?.joints ?? []) table[joint.id] = joint;
  return table;
}

function poseOf(state: SimpleArmState | null): ArmPose {
  const table = byId(state);
  return ARM_JOINTS.reduce<ArmPose>((pose, id) => {
    const degrees = table[id]?.degrees;
    if (typeof degrees !== "number" || !Number.isFinite(degrees)) return pose;
    pose[id] = toModel(id, degrees);
    return pose;
  }, { ...ZERO_POSE });
}

/**
 * The twin's calibration shape, rebuilt from the four numbers. Only the IK
 * solver wants this; nothing is stored in it and nothing is committed from it.
 */
function draftOf(state: SimpleArmState | null): CalibrationDraft {
  const table = byId(state);
  return {
    geometry: DEFAULT_GEOMETRY,
    joints: ARM_JOINTS.reduce((accumulator, id) => {
      const joint = table[id];
      accumulator[id] = {
        logicalId: id,
        name: id === "joint_1" ? "base_yaw" : id === "joint_2" ? "shoulder_pitch" : "elbow_pitch",
        servoId: joint?.servoId ?? 0,
        rawZero: joint?.rawZero ?? 2048,
        direction: (joint?.direction ?? 1) >= 0 ? 1 : -1,
        rawMin: joint?.rawLow ?? 0,
        rawMax: joint?.rawHigh ?? 4095,
        logicalMinDegrees: toModel(id, joint?.minDegrees ?? -90),
        logicalMaxDegrees: toModel(id, joint?.maxDegrees ?? 90),
        maxVelocityDegreesPerSecond: 90,
        maxAccelerationDegreesPerSecond2: 180,
        directionVerified: true,
      };
      return accumulator;
    }, {} as CalibrationDraft["joints"]),
  };
}

/**
 * What a control may actually be dragged to.
 *
 * The declared min/max is where the operator says the joint may go; `reach` is
 * that intersected with the one motor turn a single-turn goal can name. Driving
 * against the declared limits meant the far end of every bar was dead: the goal
 * went out, got clamped at the gateway, and the joint stopped somewhere in the
 * middle with nothing on screen admitting why.
 */
function reachOf(joint: SimpleJoint | undefined) {
  return {
    min: joint?.reachMin ?? joint?.minDegrees ?? -90,
    max: joint?.reachMax ?? joint?.maxDegrees ?? 90,
  };
}

/** True when the gearing cannot cover what the limits declare. */
function isPinched(joint: SimpleJoint | undefined) {
  if (joint?.reachMin == null || joint.reachMax == null || joint.minDegrees == null || joint.maxDegrees == null) return false;
  return joint.maxDegrees - joint.minDegrees - (joint.reachMax - joint.reachMin) > 1;
}

function fixed(value: number | null | undefined, digits = 1) {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function servoPoseOf(state: SimpleArmState | null) {
  return (state?.joints ?? []).reduce<Partial<Record<SimpleJointId, number>>>((pose, joint) => {
    if (typeof joint.degrees === "number" && Number.isFinite(joint.degrees)) pose[joint.id] = joint.degrees;
    return pose;
  }, {});
}

function backendInstanceOf(value: unknown): string | null {
  if (!value || typeof value !== "object") return null;
  const instanceId = (value as Record<string, unknown>).backendInstanceId;
  return typeof instanceId === "string" && instanceId ? instanceId : null;
}

function motionEvidenceOf(state: SimpleArmState): "moving" | "idle" | "unknown" {
  const aggregate = (state as SimpleArmState & { moving?: unknown }).moving;
  if (aggregate === true) return "moving";
  if (aggregate === false) return "idle";
  if (state.joints.length > 0 && state.joints.every((joint) => typeof joint.moving === "boolean")) {
    return state.joints.some((joint) => joint.moving === true) ? "moving" : "idle";
  }
  return "unknown";
}

function cameraWords(value: string) {
  return value.replaceAll("_", " ");
}

function cameraTimestamp(value: string) {
  const parsed = new Date(value);
  if (Number.isNaN(parsed.valueOf())) return value;
  return new Intl.DateTimeFormat(undefined, { dateStyle: "medium", timeStyle: "medium" }).format(parsed);
}

function cameraBytes(value: number) {
  if (!Number.isFinite(value) || value < 0) return "unknown";
  if (value < 1024) return `${Math.round(value)} B`;
  return `${(value / 1024).toFixed(value < 10 * 1024 ? 1 : 0)} KB`;
}

function cameraFocus(value: CameraStatus["autofocus"] | undefined) {
  if (!value || value.capability === "unknown") return "unknown";
  if (value.capability === "unsupported") return "motorized autofocus unavailable";
  const state = cameraWords(value.state);
  return value.range === "unavailable" ? state : `${state} · ${value.range}`;
}

function cameraDetailIsNative(profile: CameraHardwareProfile | undefined) {
  if (!profile) return false;
  return profile.captureProfiles.detail.width === profile.nativeDimensions.width
    && profile.captureProfiles.detail.height === profile.nativeDimensions.height;
}

function CameraPanel({
  status,
  observation,
  frameUrl,
  busy,
  error,
  autofocusMessage,
  canCapture,
  onCapture,
  onAutofocus,
}: {
  status: CameraStatus | null;
  observation: CameraObservation | null;
  frameUrl: string | null;
  busy: CameraBusyState;
  error: string | null;
  autofocusMessage: string | null;
  canCapture: boolean;
  onCapture: (profile: CameraCaptureProfile) => void;
  onAutofocus: () => void;
}) {
  const hardwareProfile = observation?.cameraProfile ?? status?.cameraProfile;
  const detailStill = cameraDetailIsNative(hardwareProfile)
    ? "native-detail still"
    : "detail-profile still";
  const sensor = hardwareProfile?.productName ?? observation?.sensorModel ?? status?.sensorModel ?? "Camera";
  const focus = observation?.autofocus ?? status?.autofocus;
  const stateLabel = busy === "status"
    ? "Checking"
    : busy === "capture-survey" || busy === "capture-detail"
      ? "Capturing"
      : busy === "autofocus"
        ? "Trying autofocus"
      : error
        ? "Attention"
        : observation && frameUrl
          ? "Picture ready"
          : status?.available
            ? "Ready"
            : "Unavailable";
  const stateClass = error
    ? " is-error"
    : busy
      ? " is-loading"
      : observation && frameUrl
        ? " is-ready"
        : "";

  return (
    <section className="simplearm-camera" aria-labelledby="simplearm-camera-title" aria-busy={busy !== null}>
      <header>
        <div>
          <span>Manual evidence</span>
          <h2 id="simplearm-camera-title">Desk camera</h2>
        </div>
        <strong className={`simplearm-camera-state${stateClass}`}>{stateLabel}</strong>
      </header>

      <p className="simplearm-camera-intro">Take a fast wide survey or a {detailStill}, or ask the camera to try one autofocus cycle. Nothing happens until you press a button, and none of these actions moves the arm.</p>
      <div className="simplearm-camera-actions">
        <button
          type="button"
          className="simplearm-camera-capture"
          disabled={!canCapture || busy !== null}
          onClick={() => onCapture("survey")}
        >{busy === "capture-survey" ? "Taking survey picture\u2026" : "Take survey picture"}</button>
        <button
          type="button"
          className="simplearm-camera-capture"
          disabled={!canCapture || busy !== null}
          onClick={() => onCapture("detail")}
        >{busy === "capture-detail" ? "Taking detail picture\u2026" : "Take detail picture"}</button>
        <button
          type="button"
          className="simplearm-camera-autofocus"
          disabled={!canCapture || busy !== null}
          onClick={onAutofocus}
        >{busy === "autofocus" ? "Trying autofocus\u2026" : "Try autofocus"}</button>
      </div>

      {error ? <p className="simplearm-camera-error" role="alert">{error}</p> : null}
      {autofocusMessage ? <p className="simplearm-camera-result" role="status" aria-live="polite">{autofocusMessage}</p> : null}

      {observation && frameUrl ? (
        <div className="simplearm-camera-evidence">
          <figure>
            <img
              src={frameUrl}
              alt={`Desk camera picture captured ${cameraTimestamp(observation.capturedAt)} with ${observation.sensorModel}`}
            />
            <figcaption>
              <span>Captured</span>
              <time dateTime={observation.capturedAt}>{cameraTimestamp(observation.capturedAt)}</time>
            </figcaption>
          </figure>
          <dl>
            <div><dt>Sensor</dt><dd>{observation.sensorModel}</dd></div>
            {hardwareProfile ? <div><dt>Camera</dt><dd>{hardwareProfile.productName} · {cameraWords(hardwareProfile.lensVariant)}</dd></div> : null}
            {hardwareProfile ? <div><dt>Nominal FOV</dt><dd>{fixed(hardwareProfile.nominalFieldOfViewDegrees.horizontal, 0)}° {"\u00d7"} {fixed(hardwareProfile.nominalFieldOfViewDegrees.vertical, 0)}° nominal</dd></div> : null}
            <div><dt>Profile</dt><dd>{cameraWords(observation.captureProfile)}</dd></div>
            <div><dt>Frame</dt><dd>{observation.dimensions.width} {"\u00d7"} {observation.dimensions.height}</dd></div>
            <div><dt>Size</dt><dd>{cameraBytes(observation.byteCount)}</dd></div>
            <div><dt>Focus</dt><dd>{cameraFocus(focus)}</dd></div>
            <div><dt>Identity</dt><dd>{cameraWords(observation.identityConfidence)}</dd></div>
            <div><dt>Source</dt><dd>{cameraWords(observation.source)}</dd></div>
          </dl>
          <p className="simplearm-camera-frame" title={observation.frameId}>Frame {observation.frameId}</p>
        </div>
      ) : busy === "status" ? (
        <div className="simplearm-camera-empty" aria-live="polite">
          <strong>Checking camera\u2026</strong>
          <span>Looking for retained evidence. The camera will not take a new picture.</span>
        </div>
      ) : (
        <div className="simplearm-camera-empty">
          <strong>{status?.available ? "No picture yet." : "Camera unavailable."}</strong>
          <span>
            {status?.available
              ? canCapture
                ? `Use Survey for a fast wide still or Detail for a ${detailStill}.`
                : "A local action session is required before taking a picture."
              : `${sensor} · ${status?.state ? cameraWords(status.state) : "not connected"}`}
          </span>
        </div>
      )}

      <p className="simplearm-camera-boundary">Manual camera actions · no stream · no arm movement</p>
    </section>
  );
}

function BaseTruthStrip({
  joint,
  absolute,
  scanDetected,
}: {
  joint: SimpleJoint;
  absolute: boolean;
  scanDetected: boolean;
}) {
  const truth = joint.multiTurnTruth;
  // This is the gateway's authoritative drive gate. The richer ODO row can be
  // temporarily absent/stale without rewriting a still-trusted cached position.
  const trusted = joint.positionTrusted === true;
  const mode = truth?.stepMode === false
    ? "mode 0 · free encoder"
    : truth?.stepMode === true
      ? "mode 3 · step drive"
      : "mode unknown";
  const step = truth?.resyncNeeded === true
    ? "resync required"
    : truth?.stepOutstanding === true
      ? truth.countdownObserved === true
        ? "step active · countdown seen"
        : truth.countdownObserved === false
          ? "step active · waiting for countdown"
          : "step active · countdown unknown"
      : truth?.stepOutstanding === false
        ? "step idle"
        : "step unknown";
  const resyncs = Number.isInteger(truth?.resyncCount) ? truth?.resyncCount : "unknown";
  const sample = typeof truth?.sampleAgeMs === "number" && Number.isFinite(truth.sampleAgeMs)
    ? `sample ${Math.max(0, Math.round(truth.sampleAgeMs))}ms`
    : null;

  // Stored calibration/truth can outlive a current servo reply. No cached
  // value turns an absent Base into a homing instruction or live position.
  if (!joint.online) {
    if (scanDetected) {
      return (
        <div
          className="simplearm-truth is-unknown"
          aria-label="Base multi-turn truth"
        >
          <strong>HOME REQUIRED</strong>
          <span>{`servo ID ${joint.servoId} detected by Scan`}</span>
          <span>telemetry unavailable · place at zero</span>
        </div>
      );
    }
    return (
      <div
        className="simplearm-truth is-unknown"
        aria-label="Base multi-turn truth"
      >
        <strong>BASE NOT DETECTED</strong>
        <span>{`waiting for servo ID ${joint.servoId}`}</span>
        <span>check servo power and bus link</span>
      </div>
    );
  }

  if (!trusted && !truth) {
    return (
      <div
        className="simplearm-truth is-unknown"
        aria-label="Base multi-turn truth"
      >
        <strong>HOME REQUIRED</strong>
        <span>Base not sampled</span>
        <span>place at zero · press zero here</span>
      </div>
    );
  }

  if (
    !trusted && truth?.tracking === false && truth.valid === false &&
    truth.resyncNeeded !== true
  ) {
    return (
      <div
        className="simplearm-truth is-unknown"
        aria-label="Base multi-turn truth"
      >
        <strong>HOME REQUIRED</strong>
        <span>{absolute ? "absolute encoder" : "mode 0 · free encoder"}</span>
        <span>place at zero · press zero here</span>
      </div>
    );
  }

  if (absolute) {
    return (
      <div
        className={`simplearm-truth${trusted ? " is-trusted" : " is-unknown"}`}
        aria-label="Base multi-turn truth"
      >
        <strong>{trusted ? "TRUSTED" : "UNKNOWN"}</strong>
        <span>absolute encoder</span>
        {sample ? <span>{sample}</span> : null}
      </div>
    );
  }

  return (
    <div
      className={`simplearm-truth${trusted ? " is-trusted" : " is-unknown"}${truth?.resyncNeeded === true ? " is-resync" : ""}`}
      aria-label="Base multi-turn truth"
    >
      <strong>{trusted ? "TRUSTED" : "UNKNOWN"}</strong>
      <span className={truth?.stepMode === false ? "is-mode-free" : undefined}>{mode}</span>
      <span>{step}</span>
      <span>resyncs {resyncs}</span>
      {sample ? <span>{sample}</span> : null}
    </div>
  );
}

function formatTarget(degrees: number) {
  return `${degrees > 0 ? "+" : ""}${degrees}\u00b0`;
}

function formatDurationMs(milliseconds: number) {
  return milliseconds > 0 && milliseconds < 1 ? "<1 ms" : `${Math.round(milliseconds)} ms`;
}

function BaseSelfService({
  joint,
  scanDetected,
  free,
  stopped,
  apiReady,
  action,
  onHome,
  onRestoreRange,
  onMove,
}: {
  joint: SimpleJoint;
  scanDetected: boolean;
  free: boolean;
  stopped: boolean;
  apiReady: boolean;
  action: BaseAction | null;
  onHome: () => void;
  onRestoreRange: () => void;
  onMove: (degrees: number) => void;
}) {
  const trusted = joint.positionTrusted === true;
  const busy = action?.phase === "sending";
  const canHome = apiReady && (joint.online || scanDetected) && free && !stopped && !busy;
  const { min, max } = reachOf(joint);
  const canMove = apiReady && joint.online && joint.calibrated && trusted && !stopped && !busy && max - min >= 1;
  const fullTestRange = min <= -120 && max >= 120;
  const measured = `${fixed(joint.degrees)}\u00b0`;

  let status = trusted
    ? `Measured ${measured}. Choose a test position.`
    : joint.online
      ? free
        ? "Turn the Base by hand to the marked zero, then set zero here."
        : "Free the Base before setting its zero."
      : scanDetected
        ? free
          ? `Servo ID ${joint.servoId} answered Scan. Turn the Base by hand to the marked zero, then set zero here.`
          : "Free the Base before setting its zero."
        : `Waiting for servo ID ${joint.servoId}.`;
  let statusTone = "";

  if (action?.phase === "sending") {
    status = action.kind === "home"
      ? "Reading the encoder and setting this position as zero..."
      : action.kind === "range"
        ? "Centering the saved Base range on zero..."
        : `Sending ${formatTarget(action.target ?? 0)}...`;
  } else if (action?.phase === "failed") {
    status = action.detail ?? "The Base action was refused.";
    statusTone = " is-error";
  } else if (action?.kind === "home" && action.phase === "accepted") {
    status = trusted
      ? `Zero set. Measured ${measured}.`
      : "Zero was accepted; waiting for a trusted Base reading.";
  } else if (action?.kind === "range" && action.phase === "accepted") {
    status = "Base range set to -180\u00b0 through +180\u00b0.";
    statusTone = " is-success";
  } else if (action?.kind === "move" && action.phase === "accepted") {
    const target = action.target ?? 0;
    if (!trusted) {
      status = `Position became unknown after ${formatTarget(target)} was accepted. Put the Base on the mark and set zero again.`;
      statusTone = " is-error";
    } else if (typeof joint.degrees === "number" && Math.abs(joint.degrees - target) < 1) {
      status = `Reached ${formatTarget(target)}. Measured ${measured}.`;
      statusTone = " is-success";
    } else {
      status = `Target ${formatTarget(target)} accepted. Measured ${measured}.`;
    }
  }

  return (
    <div className={`simplearm-base-self${trusted ? " is-ready" : " is-home"}`} aria-label="Base setup and test">
      <div className="simplearm-base-home">
        <div>
          <span>Base setup</span>
          <strong>{trusted ? "Reference ready" : "Place Base on the zero mark"}</strong>
        </div>
        <button
          type="button"
          className="simplearm-base-home-button"
          aria-label="Set Base zero here"
          disabled={!canHome}
          onClick={onHome}
        >{busy && action?.kind === "home" ? "Setting..." : trusted ? "Re-set zero here" : "Set zero here"}</button>
      </div>

      <div className="simplearm-base-presets">
        <span>Test positions</span>
        <div role="group" aria-label="Base test positions">
          {BASE_TEST_ANGLES.map((target) => (
            <button
              type="button"
              key={target}
              disabled={!canMove || target < min || target > max}
              onClick={() => onMove(target)}
            >{formatTarget(target)}</button>
          ))}
        </div>
      </div>

      {trusted && !fullTestRange ? (
        <div className="simplearm-base-range-fix">
          <span>Saved range is {fixed(min, 0)}\u00b0 to {fixed(max, 0)}\u00b0, so some tests are locked.</span>
          <button
            type="button"
            aria-label="Set Base range to minus 180 through plus 180 degrees"
            disabled={!apiReady || busy}
            onClick={onRestoreRange}
          >Use -180\u00b0 ... +180\u00b0</button>
        </div>
      ) : null}

      <p className={`simplearm-base-status${statusTone}`} role={statusTone === " is-error" ? "alert" : "status"} aria-live="polite">
        {status}
      </p>
    </div>
  );
}

export function SimpleArm({
  request = ((...args: Parameters<typeof fetch>) => globalThis.fetch(...args)) as typeof fetch,
  onControlBusyChange,
}: {
  request?: typeof fetch;
  onControlBusyChange?: (busy: boolean) => void;
}) {
  const [actionToken, setActionToken] = useState<string | null>(null);
  const [state, setState] = useState<SimpleArmState | null>(null);
  // Sticky: only a new action clears it. The poll used to null this out every
  // 150 ms, so no failure was ever on screen long enough to read.
  const [message, setMessage] = useState<string | null>(null);
  const [connecting, setConnecting] = useState(true);
  const [scanBusy, setScanBusy] = useState(false);
  const [selected, setSelected] = useState<SimpleJointId>("joint_1");
  // Flat by default: it is the one that drives.
  const [view, setView] = useState<"flat" | "twin">("flat");
  // Camera takes the joint panel's column rather than adding another one, so
  // the arm scene stays visible at the same useful size.
  const [panel, setPanel] = useState<"joints" | "camera">("joints");
  const [renaming, setRenaming] = useState(false);
  const [renameFrom, setRenameFrom] = useState("1");
  const [renameTo, setRenameTo] = useState("4");
  const [commanded, setCommanded] = useState<Partial<Record<SimpleJointId, number>>>({});
  const [baseAction, setBaseAction] = useState<BaseAction | null>(null);
  const [preparedSingle, setPreparedSingle] = useState<PreparedSingleMotion | null>(null);
  const [planPhase, setPlanPhase] = useState<PlanPhase>("idle");
  const [requestedTargets, setRequestedTargets] = useState<Partial<Record<SimpleJointId, number>>>({});
  const [motionMode, setMotionMode] = useState<MotionMode>("single");
  const [sequenceWaypoints, setSequenceWaypoints] = useState<SequenceDraft[]>([]);
  const [editingWaypoint, setEditingWaypoint] = useState<number | null>(null);
  const [sequenceDraftDirty, setSequenceDraftDirty] = useState(false);
  const [preparedSequence, setPreparedSequence] = useState<SimpleArmSequencePlan | null>(null);
  const [sequencePhase, setSequencePhase] = useState<SequencePhase>("idle");
  const [sequenceResult, setSequenceResult] = useState<SimpleArmSequenceExecution | null>(null);
  const [sequencePreviewDurationMs, setSequencePreviewDurationMs] = useState<number | null>(null);
  const [manualMotionInFlight, setManualMotionInFlight] = useState(false);
  const [cameraStatus, setCameraStatus] = useState<CameraStatus | null>(null);
  const [cameraObservation, setCameraObservation] = useState<CameraObservation | null>(null);
  const [cameraFrameUrl, setCameraFrameUrl] = useState<string | null>(null);
  const [cameraBusy, setCameraBusy] = useState<CameraBusyState>("status");
  const [cameraError, setCameraError] = useState<string | null>(null);
  const [cameraAutofocusMessage, setCameraAutofocusMessage] = useState<string | null>(null);
  const pendingRef = useRef<Partial<Record<SimpleJointId, number>>>({});
  const draftRef = useRef<Partial<Record<SimpleJointId, number>>>({});
  const executingRef = useRef<Partial<Record<SimpleJointId, number>>>({});
  const planRequestRef = useRef(0);
  const applyInFlightRef = useRef(false);
  const sequenceApplyInFlightRef = useRef(false);
  const sequenceRequestRef = useRef(0);
  const nextWaypointIdRef = useRef(1);
  const cameraFrameUrlRef = useRef<string | null>(null);
  const cameraMountedRef = useRef(true);
  // Every state-producing mutation advances this epoch. A poll that started
  // before that mutation may still finish later, but it is no longer allowed
  // to replace the newer authoritative response.
  const stateGenerationRef = useRef(0);
  const scanBusyRef = useRef(false);
  const lastBackendInstanceRef = useRef<string | null>(null);
  const executionBackendInstanceRef = useRef<string | null>(null);
  const executionObservedMotionRef = useRef(false);
  const executionIdlePollsRef = useRef(0);
  const manualDispatchPendingRef = useRef(false);

  const retireManualExecution = useCallback((detail?: string) => {
    executingRef.current = {};
    executionBackendInstanceRef.current = null;
    executionObservedMotionRef.current = false;
    executionIdlePollsRef.current = 0;
    manualDispatchPendingRef.current = false;
    setManualMotionInFlight(false);
    setPlanPhase((current) => (
      current === "executing" || (detail && current === "applying")
        ? detail ? "error" : "idle"
        : current
    ));
    if (detail) setMessage(detail);
  }, []);

  const commitMutationState = useCallback((next: SimpleArmState) => {
    stateGenerationRef.current += 1;
    setState(next);
  }, []);

  const api = useMemo(() => actionToken ? createSimpleArmApi(request, actionToken) : null, [actionToken, request]);
  const cameraApi = useMemo(() => createCameraApi(request, actionToken), [actionToken, request]);
  const preparedPlan = preparedSingle?.kind === "plan" ? preparedSingle.value : null;
  const preparedFloorRoute = preparedSingle?.kind === "floor-route" ? preparedSingle.value : null;
  const joints = byId(state);
  const measured = poseOf(state);
  const draft = useMemo(() => draftOf(state), [state]);

  const replaceCameraFrame = useCallback((frame: Blob | null) => {
    const nextUrl = frame ? URL.createObjectURL(frame) : null;
    if (cameraFrameUrlRef.current) URL.revokeObjectURL(cameraFrameUrlRef.current);
    cameraFrameUrlRef.current = nextUrl;
    setCameraFrameUrl(nextUrl);
  }, []);

  useEffect(() => {
    cameraMountedRef.current = true;
    return () => {
      cameraMountedRef.current = false;
      if (cameraFrameUrlRef.current) URL.revokeObjectURL(cameraFrameUrlRef.current);
      cameraFrameUrlRef.current = null;
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    void (async () => {
      try {
        const token = await loadActionToken(request);
        if (!cancelled) setActionToken(token);
      } catch {
        if (!cancelled) setMessage("No local session. Restart the desktop app.");
      }
    })();
    return () => { cancelled = true; };
  }, [request]);

  useEffect(() => {
    let cancelled = false;
    void (async () => {
      setCameraBusy("status");
      setCameraError(null);

      let nextStatus: CameraStatus;
      try {
        nextStatus = await cameraApi.status();
      } catch (caught) {
        if (!cancelled) {
          setCameraStatus(null);
          setCameraError(`Camera status could not be loaded: ${caught instanceof Error ? caught.message : "unknown error"}`);
          setCameraBusy(null);
        }
        return;
      }

      if (cancelled) return;
      setCameraStatus(nextStatus);
      if (!nextStatus.latestFrameId) {
        setCameraObservation(null);
        replaceCameraFrame(null);
        setCameraBusy(null);
        return;
      }
      // Status is read-only and may be read before the action token arrives.
      // Retained evidence is authenticated, so a token change reruns this
      // effect and loads it without taking a new picture.
      if (!actionToken) {
        setCameraBusy(null);
        return;
      }

      let latest: CameraObservation;
      try {
        latest = await cameraApi.latest();
      } catch (caught) {
        if (!cancelled) {
          setCameraError(`Latest picture metadata could not be loaded: ${caught instanceof Error ? caught.message : "unknown error"}`);
          setCameraBusy(null);
        }
        return;
      }

      if (cancelled) return;
      try {
        const frame = await cameraApi.frame(latest.frameId);
        if (cancelled) return;
        setCameraObservation(latest);
        replaceCameraFrame(frame);
      } catch (caught) {
        if (!cancelled) {
          setCameraObservation(latest);
          replaceCameraFrame(null);
          setCameraError(`Latest picture was found, but its JPEG could not be loaded: ${caught instanceof Error ? caught.message : "unknown error"}`);
        }
      } finally {
        if (!cancelled) setCameraBusy(null);
      }
    })();
    return () => { cancelled = true; };
  }, [actionToken, cameraApi, replaceCameraFrame]);

  useEffect(() => {
    if (!api) return;
    let cancelled = false;
    let timer: number | null = null;
    let hasReadState = state !== null;
    const read = async () => {
      const generation = stateGenerationRef.current;
      try {
        const next = await api.state();
        if (cancelled || generation !== stateGenerationRef.current) return;
        hasReadState = true;
        setState(next);
        const nextBackendInstance = backendInstanceOf(next);
        const previousBackendInstance = lastBackendInstanceRef.current;
        const backendRestarted = Boolean(
          previousBackendInstance
          && nextBackendInstance
          && previousBackendInstance !== nextBackendInstance
        );
        if (nextBackendInstance) lastBackendInstanceRef.current = nextBackendInstance;
        if (backendRestarted) {
          planRequestRef.current += 1;
          sequenceRequestRef.current += 1;
          setPreparedSingle(null);
          setPreparedSequence(null);
          setSequencePreviewDurationMs(null);
          setSequencePhase((current) => current === "idle" ? current : "draft");
          retireManualExecution("The arm backend restarted, so the old move and reviewed plans are no longer active. Prepare a fresh move.");
        }
        // A STOP latched outside this dashboard invalidates the same reviewed
        // artifact as the local STOP button. Do not let Clear STOP silently
        // re-arm a path that was reviewed before the interruption.
        if (next.stopped && !sequenceApplyInFlightRef.current) {
          sequenceRequestRef.current += 1;
          setPreparedSequence(null);
          setSequencePreviewDurationMs(null);
          setSequencePhase((current) => current === "ready" || current === "planning" ? "draft" : current);
        }
        if (next.stopped && !applyInFlightRef.current) {
          planRequestRef.current += 1;
          setPreparedSingle(null);
          retireManualExecution();
          setPlanPhase((current) => current === "ready" || current === "planning" || current === "executing" ? "idle" : current);
        }
        if (connecting) setConnecting(false);
        // Retire an applied plan only when telemetry reaches it. Drafts and
        // prepared plans remain visible even when torque is currently free:
        // they are proposals, not claims that the arm has already moved.
        const executingTargets = { ...executingRef.current };
        const executionFinished = Object.entries(executingTargets).length > 0 &&
          Object.entries(executingTargets).every(([id, target]) => {
            const joint = next.joints.find((candidate) => candidate.id === id);
            return typeof target === "number" && typeof joint?.degrees === "number" && Math.abs(joint.degrees - target) < 1;
          });
        const executionInstanceChanged = Boolean(
          executionBackendInstanceRef.current
          && nextBackendInstance
          && executionBackendInstanceRef.current !== nextBackendInstance
        );
        const motionEvidence = motionEvidenceOf(next);
        if (motionEvidence === "moving") {
          executionObservedMotionRef.current = true;
          executionIdlePollsRef.current = 0;
        } else if (
          Object.keys(executingTargets).length > 0
          && motionEvidence === "idle"
          && !manualDispatchPendingRef.current
          && !applyInFlightRef.current
        ) {
          executionIdlePollsRef.current += 1;
        } else if (motionEvidence === "unknown") {
          executionIdlePollsRef.current = 0;
        }
        const executionEndedWithoutArrival = Object.keys(executingTargets).length > 0
          && motionEvidence === "idle"
          && !manualDispatchPendingRef.current
          && !applyInFlightRef.current
          && (
            executionObservedMotionRef.current
            || executionIdlePollsRef.current >= IDLE_POLLS_BEFORE_RETIRING_MISSED_GOAL
          );
        if (executionFinished) {
          retireManualExecution();
        } else if (executionInstanceChanged || executionEndedWithoutArrival) {
          retireManualExecution(
            executionInstanceChanged
              ? "The arm backend restarted before that target was verified. Prepare a fresh move."
              : "The controller reports no active motion, but the target was not reached. Prepare a fresh move.",
          );
        }
        // `commanded` is written as "last thing asked for" but read as "where the
        // arm is" — by previewPose and by the bars — so leaving it set forever
        // froze the drawing at the last goal and, worse, made the elbow measure
        // itself against a shoulder that had moved on.
        setCommanded((current) => {
          const kept = { ...current };
          for (const joint of next.joints) {
            const wanted = kept[joint.id];
            if (typeof wanted !== "number") continue;
            const executing = executingTargets[joint.id];
            // Drafts and plans do not expire merely because the physical joint
            // is free. Only targets actually handed to execute are retired.
            if (typeof executing !== "number") continue;
            // A servo parks a few tenths off its goal, so exact equality would
            // never retire. Widen if the arm settles sloppier than this.
            const arrived = typeof joint.degrees === "number" && Math.abs(joint.degrees - wanted) < 1;
            if (arrived) kept[joint.id] = undefined;
          }
          return kept;
        });
      } catch (error) {
        // A dropped poll is not evidence the arm changed; keep the last reading.
        if (!cancelled && !hasReadState) {
          // The proxy names the layer that failed (tunnel down vs Pi silent vs
          // dropped mid-request); pass it through rather than flattening every
          // cause into one sentence that fits none of them.
          setMessage(error instanceof Error ? error.message : "Could not reach the Pi, and the reason was not reported.");
        }
      } finally {
        // Schedule from completion instead of a wall-clock interval. A slow Pi
        // response must never build a queue of older snapshots that can land
        // out of order after a newer state or mutation.
        if (!cancelled) timer = window.setTimeout(() => void read(), POLL_MS);
      }
    };
    void read();
    return () => {
      cancelled = true;
      if (timer !== null) window.clearTimeout(timer);
    };
  }, [api, retireManualExecution]);

  const takeCameraPicture = async (profile: CameraCaptureProfile) => {
    if (!actionToken || !cameraStatus?.available || cameraBusy !== null) return;
    setCameraBusy(profile === "survey" ? "capture-survey" : "capture-detail");
    setCameraError(null);

    let observation: CameraObservation;
    try {
      observation = await cameraApi.capture(profile);
    } catch (caught) {
      if (cameraMountedRef.current) {
        setCameraError(`Picture capture failed: ${caught instanceof Error ? caught.message : "unknown error"}`);
        setCameraBusy(null);
      }
      return;
    }
    if (!cameraMountedRef.current) return;

    try {
      const frame = await cameraApi.frame(observation.frameId);
      if (!cameraMountedRef.current) return;
      setCameraObservation(observation);
      replaceCameraFrame(frame);
      setCameraStatus((current) => current ? { ...current, latestFrameId: observation.frameId } : current);
    } catch (caught) {
      // The shutter succeeded, but displaying unverified or missing bytes as a
      // successful picture would collapse two materially different failures.
      if (cameraMountedRef.current) {
        setCameraObservation(observation);
        replaceCameraFrame(null);
        setCameraError(`Picture was captured, but its JPEG could not be loaded: ${caught instanceof Error ? caught.message : "unknown error"}`);
      }
    } finally {
      if (cameraMountedRef.current) setCameraBusy(null);
    }
  };

  const tryCameraAutofocus = async () => {
    if (!actionToken || !cameraStatus?.available || cameraBusy !== null) return;
    setCameraBusy("autofocus");
    setCameraError(null);
    setCameraAutofocusMessage(null);

    try {
      const response = await cameraApi.autofocus();
      if (!cameraMountedRef.current) return;
      setCameraStatus((current) => current ? { ...current, autofocus: response.autofocus } : current);
      setCameraAutofocusMessage(CAMERA_AUTOFOCUS_MESSAGES[response.result]);
    } catch (caught) {
      if (cameraMountedRef.current) {
        setCameraError(`Autofocus test failed: ${caught instanceof Error ? caught.message : "unknown error"}`);
      }
    } finally {
      if (cameraMountedRef.current) setCameraBusy(null);
    }
  };

  /** Dragging is local. A new edit invalidates any older one-time plan. */
  const preview = useCallback((next: Partial<Record<SimpleJointId, number>>) => {
    // A slider, typed target, or 2D drag supersedes any old preset result.
    if (next.joint_1 !== undefined) setBaseAction(null);
    setCommanded((current) => ({ ...current, ...next }));
    pendingRef.current = { ...pendingRef.current, ...next };
    draftRef.current = { ...draftRef.current, ...next };
    planRequestRef.current += 1;
    setPreparedSingle(null);
    setPlanPhase("draft");
    setRequestedTargets({ ...draftRef.current });
    if (motionMode === "sequence") {
      setSequenceDraftDirty(true);
      setPreparedSequence(null);
      setSequenceResult(null);
      setSequencePreviewDurationMs(null);
      setSequencePhase("draft");
    }
  }, [motionMode]);

  /**
   * Release prepares one authoritative, non-moving plan. The plan resolver on
   * the Pi owns final clamping, floor clearance, start-pose freshness, and the
   * one-time digest. Only `applyPlan` below can consume it.
   *
   * This used to stream a goal every ~45 ms for the whole drag. Each one landed
   * a few ticks from the last, so the servo spent the entire gesture ramping out
   * of a standstill under a deliberately gentle acceleration and never reached
   * speed — the drag felt heavy, and letting go left a queue of stale goals still
   * arriving. Sending once means a full-speed move to the place the handle
   * actually ended up, and moving again retargets instead of queueing.
   */
  const preparePlan = useCallback(() => {
    const pending = pendingRef.current;
    pendingRef.current = {};
    if (Object.keys(pending).length === 0) return;
    if (motionMode === "sequence") {
      setSequenceDraftDirty(true);
      return;
    }
    const targets = { ...draftRef.current };
    const requestId = ++planRequestRef.current;
    setPlanPhase("planning");
    setPreparedSingle(null);
    setRequestedTargets(targets);
    if (!api) {
      setPlanPhase("error");
      setMessage("The arm planner is unavailable. The draft did not move the arm.");
      return;
    }
    void (async () => {
      try {
        const plan = await api.previewPlan(targets);
        if (planRequestRef.current !== requestId) return;
        const resolved = { ...targets, ...plan.resolvedPose };
        draftRef.current = resolved;
        setCommanded(resolved);
        setPreparedSingle({ kind: "plan", value: { ...plan, resolvedPose: resolved } });
        setPlanPhase("ready");
        setMessage(null);
      } catch (error) {
        if (planRequestRef.current !== requestId) return;
        if (!isInitialFloorSweepRefusal(error)) throw error;

        const candidates = floorRouteCandidates(servoPoseOf(state), targets);
        for (const waypoints of candidates) {
          try {
            const route = await api.previewSequence(waypoints);
            if (planRequestRef.current !== requestId) return;
            if (route.waypointCount !== waypoints.length || route.waypoints.length !== waypoints.length) {
              throw new Error("The arm planner returned a different route length. The move was not armed.");
            }
            const final = route.waypoints.at(-1)?.resolvedPose;
            if (!final) throw new Error("The arm planner returned a route without a destination. The move was not armed.");
            const resolved = { ...targets, ...final };
            draftRef.current = resolved;
            setCommanded(resolved);
            setPreparedSingle({ kind: "floor-route", value: route });
            setPlanPhase("ready");
            setMessage(null);
            return;
          } catch (routeError) {
            if (planRequestRef.current !== requestId) return;
            if (isRoutedFloorSweepRefusal(routeError)) continue;
            throw routeError;
          }
        }
        throw new Error("No floor-safe route was found. Adjust the destination and try again.");
      }
    })().catch((error) => {
      if (planRequestRef.current !== requestId) return;
      setPreparedSingle(null);
      setPlanPhase("error");
      setMessage(error instanceof Error ? error.message : "That plan was refused. The arm did not move.");
    });
  }, [api, motionMode, state]);

  const drive = useCallback((next: Partial<Record<SimpleJointId, number>>) => {
    preview(next);
    preparePlan();
  }, [preparePlan, preview]);

  const cancelPlan = useCallback(() => {
    planRequestRef.current += 1;
    pendingRef.current = {};
    draftRef.current = {};
    setPreparedSingle(null);
    setRequestedTargets({});
    setCommanded((current) => ({
      ...current,
      ...Object.fromEntries(Object.keys(current).filter((id) => executingRef.current[id as SimpleJointId] === undefined).map((id) => [id, undefined])),
    }));
    setPlanPhase(Object.keys(executingRef.current).length > 0 ? "executing" : "idle");
  }, []);

  const applyPlan = useCallback(async () => {
    const prepared = preparedSingle;
    if (!api || !prepared || planPhase === "applying" || applyInFlightRef.current) return;
    applyInFlightRef.current = true;
    setPlanPhase("applying");
    // A missing response is ambiguous: never offer a blind retry of the same
    // one-time plan. Telemetry remains the source of truth after this call.
    try {
      let resolved: Partial<Record<SimpleJointId, number>>;
      let executionInstance = lastBackendInstanceRef.current;
      if (prepared.kind === "plan") {
        const result = await api.executePlan(prepared.value);
        executionInstance = backendInstanceOf(result) ?? executionInstance;
        resolved = { ...prepared.value.resolvedPose, ...(result.resolvedPose ?? {}) };
      } else {
        const result = await api.executeSequence(prepared.value);
        executionInstance = backendInstanceOf(result) ?? executionInstance;
        if (result.outcome !== "completed") {
          const failed = result.failedWaypointIndex === null ? "" : ` Failed at leg ${result.failedWaypointIndex + 1}.`;
          throw new Error(`Floor-safe route ${result.outcome} after ${result.completedWaypointCount} of ${result.waypointCount} legs.${failed}`);
        }
        resolved = {
          ...(prepared.value.waypoints.at(-1)?.resolvedPose ?? {}),
          ...(result.resolvedPose ?? {}),
          ...(result.finalMeasuredPose ?? {}),
        };
      }
      executingRef.current = resolved;
      executionBackendInstanceRef.current = executionInstance;
      executionObservedMotionRef.current = false;
      executionIdlePollsRef.current = 0;
      setManualMotionInFlight(Object.keys(resolved).length > 0);
      setCommanded(resolved);
      setMessage(null);
      setPlanPhase("executing");
    } catch (error) {
      setCommanded({});
      retireManualExecution();
      setPlanPhase("error");
      setMessage(error instanceof Error ? error.message : "Plan execution is unknown. Read telemetry and prepare a fresh plan.");
    } finally {
      applyInFlightRef.current = false;
      planRequestRef.current += 1;
      pendingRef.current = {};
      draftRef.current = {};
      setPreparedSingle(null);
      setRequestedTargets({});
    }
  }, [api, planPhase, preparedSingle, retireManualExecution]);

  const sequenceRunning = sequencePhase === "applying" || (planPhase === "applying" && preparedFloorRoute !== null);
  const controlBusy = scanBusy || manualMotionInFlight || planPhase === "applying" || planPhase === "executing" || sequenceRunning;

  useEffect(() => {
    onControlBusyChange?.(controlBusy);
  }, [controlBusy, onControlBusyChange]);

  useEffect(() => {
    return () => onControlBusyChange?.(false);
  }, [onControlBusyChange]);

  const invalidateSequenceReview = (nextCount = sequenceWaypoints.length) => {
    sequenceRequestRef.current += 1;
    setPreparedSequence(null);
    setSequenceResult(null);
    setSequencePreviewDurationMs(null);
    setSequencePhase(nextCount > 0 ? "draft" : "idle");
  };

  const sequenceSnapshot = () => {
    const live = servoPoseOf(state);
    const tail = sequenceWaypoints.at(-1)?.targets ?? live;
    return ALL_JOINTS.reduce<Partial<Record<SimpleJointId, number>>>((pose, id) => {
      const value = draftRef.current[id] ?? commanded[id] ?? tail[id] ?? live[id];
      if (typeof value === "number" && Number.isFinite(value)) pose[id] = value;
      return pose;
    }, {});
  };

  const storeWaypoint = () => {
    if (sequenceRunning || (!sequenceDraftDirty && editingWaypoint === null)) return;
    const targets = sequenceSnapshot();
    if (Object.keys(targets).length !== ALL_JOINTS.length) {
      setMessage("Every joint needs trusted telemetry before a sequence waypoint can be stored.");
      return;
    }
    const next = editingWaypoint === null
      ? sequenceWaypoints.length >= 8
        ? sequenceWaypoints
        : [...sequenceWaypoints, { id: nextWaypointIdRef.current++, targets }]
      : sequenceWaypoints.map((waypoint, index) => index === editingWaypoint ? { ...waypoint, targets } : waypoint);
    if (next === sequenceWaypoints) return;
    setSequenceWaypoints(next);
    setEditingWaypoint(null);
    setSequenceDraftDirty(false);
    draftRef.current = { ...targets };
    pendingRef.current = {};
    setCommanded({ ...targets });
    setRequestedTargets({ ...targets });
    invalidateSequenceReview(next.length);
    setMessage(null);
  };

  const editSequenceWaypoint = (index: number) => {
    if (sequenceRunning) return;
    const waypoint = sequenceWaypoints[index];
    if (!waypoint) return;
    invalidateSequenceReview(sequenceWaypoints.length);
    setEditingWaypoint(index);
    setSequenceDraftDirty(false);
    draftRef.current = { ...waypoint.targets };
    pendingRef.current = {};
    setCommanded({ ...waypoint.targets });
    setRequestedTargets({ ...waypoint.targets });
  };

  const moveSequenceWaypoint = (index: number, direction: -1 | 1) => {
    if (sequenceRunning) return;
    const destination = index + direction;
    if (destination < 0 || destination >= sequenceWaypoints.length) return;
    const next = [...sequenceWaypoints];
    [next[index], next[destination]] = [next[destination], next[index]];
    const tail = next.at(-1)?.targets ?? servoPoseOf(state);
    setSequenceWaypoints(next);
    setEditingWaypoint(null);
    setSequenceDraftDirty(false);
    draftRef.current = { ...tail };
    pendingRef.current = {};
    setCommanded({ ...tail });
    setRequestedTargets({ ...tail });
    invalidateSequenceReview(next.length);
  };

  const removeSequenceWaypoint = (index: number) => {
    if (sequenceRunning) return;
    const next = sequenceWaypoints.filter((_, candidate) => candidate !== index);
    setSequenceWaypoints(next);
    setEditingWaypoint(null);
    setSequenceDraftDirty(false);
    const tail = next.at(-1)?.targets ?? servoPoseOf(state);
    draftRef.current = { ...tail };
    pendingRef.current = {};
    setCommanded({ ...tail });
    setRequestedTargets({ ...tail });
    invalidateSequenceReview(next.length);
  };

  const clearSequence = () => {
    if (sequenceRunning) return;
    setSequenceWaypoints([]);
    setEditingWaypoint(null);
    setSequenceDraftDirty(false);
    draftRef.current = {};
    pendingRef.current = {};
    setCommanded({});
    invalidateSequenceReview(0);
  };

  const previewSequence = async () => {
    if (
      !api
      || sequenceRunning
      || sequencePhase === "planning"
      || sequenceDraftDirty
      || editingWaypoint !== null
      || sequenceWaypoints.length < 2
      || sequenceWaypoints.length > 8
    ) return;
    const requestId = ++sequenceRequestRef.current;
    const waypoints: SimpleArmSequenceWaypointInput[] = sequenceWaypoints.map(({ targets }) => ({ targets }));
    setPreparedSequence(null);
    setSequenceResult(null);
    setSequencePreviewDurationMs(null);
    setSequencePhase("planning");
    try {
      const plan = await api.previewSequence(waypoints);
      if (sequenceRequestRef.current !== requestId) return;
      if (plan.waypointCount !== waypoints.length || plan.waypoints.length !== waypoints.length) {
        throw new Error("The arm planner returned a different waypoint count. The sequence was not armed.");
      }
      setPreparedSequence(plan);
      setSequencePreviewDurationMs(plan.previewDurationMs);
      setSequencePhase("ready");
      setMessage(null);
    } catch (error) {
      if (sequenceRequestRef.current !== requestId) return;
      setPreparedSequence(null);
      setSequencePreviewDurationMs(null);
      setSequencePhase("error");
      setMessage(error instanceof Error ? error.message : "The sequence was refused. The arm did not move.");
    }
  };

  const applySequence = async () => {
    const sequence = preparedSequence;
    if (!api || !sequence || sequencePhase !== "ready" || sequenceApplyInFlightRef.current) return;
    sequenceApplyInFlightRef.current = true;
    setSequencePhase("applying");
    setSequenceResult(null);
    try {
      const result = await api.executeSequence(sequence);
      setSequenceResult(result);
      setSequencePhase("complete");
      setCommanded({});
      setMessage(null);
    } catch (error) {
      setSequencePhase("error");
      setSequenceResult(null);
      setCommanded({});
      setMessage(error instanceof Error ? error.message : "Sequence execution is unknown. Read telemetry and preview it again.");
    } finally {
      sequenceApplyInFlightRef.current = false;
      sequenceRequestRef.current += 1;
      setPreparedSequence(null);
      setEditingWaypoint(null);
      setSequenceDraftDirty(false);
      draftRef.current = {};
      pendingRef.current = {};
      setRequestedTargets({});
    }
  };

  const chooseMotionMode = (next: MotionMode) => {
    if (sequenceRunning || motionMode === next) return;
    cancelPlan();
    setMotionMode(next);
    setEditingWaypoint(null);
    setSequenceDraftDirty(false);
    if (next === "sequence") {
      const tail = sequenceWaypoints.at(-1)?.targets ?? servoPoseOf(state);
      draftRef.current = { ...tail };
      setCommanded({ ...tail });
      setRequestedTargets({ ...tail });
    } else {
      draftRef.current = {};
      setCommanded({});
      setRequestedTargets({});
      invalidateSequenceReview(sequenceWaypoints.length);
    }
  };

  const held = state?.held ?? [];
  const stopped = state?.stopped ?? false;
  const busReady = state?.connection === "online" && state.bus === "online" && !stopped;
  const expectedJointCount = state?.joints.length ?? ALL_JOINTS.length;
  const liveJointCount = state?.joints.filter((joint) =>
    joint.online === true
      && joint.positionTrusted === true
      && typeof joint.degrees === "number"
      && Number.isFinite(joint.degrees)).length ?? 0;
  const allJointsOnline = state !== null
    && state.joints.length === ALL_JOINTS.length
    && ALL_JOINTS.every((id) => joints[id]?.online === true);
  const scanEvidence = state?.lastScan ?? state?.scan;
  const scanPingIds = scanEvidence?.pingFoundIds ?? scanEvidence?.foundIds ?? [];
  const baseServoId = joints.joint_1?.servoId;
  const baseDetectedByScan = typeof baseServoId === "number" && scanPingIds.includes(baseServoId);
  const livePoseAvailable = ARM_JOINTS.every((id) => {
    const joint = joints[id];
    return joint?.online === true
      && joint.positionTrusted === true
      && typeof joint.degrees === "number"
      && Number.isFinite(joint.degrees);
  });
  // The Pi owns the plane. Falling back to the local default only matters
  // against a gateway too old to report one; guarding by default is the safe
  // way round, since the arm reaches well below its own pivot.
  const floorGuardOn = state?.floorGuard?.enabled ?? true;
  const floorMm = floorGuardOn ? state?.floorGuard?.floorMm ?? DEFAULT_FLOOR_MM : null;
  const genericBusTrouble = state?.bus === "faulted" ? FAULTED_BUS_TROUBLE : state?.busTrouble;

  const setHold = async (ids: number[]) => {
    if (!api) return;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    try { commitMutationState(await api.torque(ids)); } catch (error) {
      setMessage(error instanceof Error ? error.message : "The controller would not change torque.");
    }
  };

  // Free one joint, hold the rest: the arm must not collapse while a joint is
  // being positioned by hand.
  const freeOnly = (joint: SimpleJoint) => setHold(
    (state?.joints ?? []).filter((other) => other.id !== joint.id).map((other) => other.servoId),
  );
  const holdAll = () => setHold((state?.joints ?? []).map((joint) => joint.servoId));

  const apply = async (joint: SimpleJoint, patch: SimpleCalibratePatch) => {
    if (!api) return;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    try {
      const next = await api.calibrate(joint.id, patch);
      commitMutationState(next);
      setCommanded((current) => ({ ...current, [joint.id]: undefined }));
      setMessage(null);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : "That value was not stored.");
    }
  };

  /** "Here" — store wherever the joint is standing right now.
   *
   *  The Pi resolves it, not this. Sending our own rawPosition stored a reading
   *  up to a poll and a refresh old; on the camera, small and free-spinning
   *  under a hand, that is tens of degrees and made "here" land somewhere the
   *  joint had already left. */
  const captureHere = (joint: SimpleJoint, field: "rawZero" | "rawMin" | "rawMax") => {
    void apply(joint, { here: [field] });
  };

  const scanArm = async () => {
    if (!api || scanBusyRef.current || sequenceRunning) return;
    scanBusyRef.current = true;
    setScanBusy(true);
    // Any poll that began before this hardware census is stale by definition.
    stateGenerationRef.current += 1;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    try {
      const nextState = await api.scan();
      commitMutationState(nextState);
      const foundIds = nextState.scan?.foundIds;
      const unavailableIds = nextState.scan?.telemetryUnavailableIds ?? [];
      if (Array.isArray(foundIds)) {
        if (unavailableIds.length > 0) {
          setMessage(`Scan found servo IDs ${foundIds.join(", ")}; waiting for full telemetry from ${unavailableIds.join(", ")}.`);
        } else {
          setMessage(foundIds.length
            ? `Scan complete: servo IDs ${foundIds.join(", ")} answered.`
            : "Scan complete: no servo IDs answered from 0–10. Check the dedicated 12 V servo rail and bus cabling; do not change IDs based on silence.");
        }
      } else {
        setMessage("Scan completed, but this gateway did not return an ID inventory.");
      }
    } catch (error) {
      const failure = error instanceof Error ? `Scan failed: ${error.message}` : "Scan failed.";
      // The failed request may still have changed the HAT census. Read it once
      // now so an older polling response cannot repaint pre-scan inventory.
      try {
        commitMutationState(await api.state());
      } catch {
        // Keep the last visible state and the original bounded Scan error.
      }
      setMessage(failure);
    } finally {
      scanBusyRef.current = false;
      setScanBusy(false);
    }
  };

  const homeBase = async (joint: SimpleJoint) => {
    if (!api) return;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    setMessage(null);
    setBaseAction({ kind: "home", phase: "sending" });
    try {
      const next = await api.calibrate(joint.id, { here: ["rawZero"] });
      commitMutationState(next);
      setCommanded((current) => ({ ...current, [joint.id]: undefined }));
      setBaseAction({ kind: "home", phase: "accepted" });
    } catch (error) {
      const detail = error instanceof Error ? error.message : "The Base zero was refused.";
      setBaseAction({ kind: "home", phase: "failed", detail });
    }
  };

  const moveBase = async (degrees: number) => {
    if (!api) return;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    setMessage(null);
    setCommanded((current) => ({ ...current, joint_1: degrees }));
    executingRef.current = { joint_1: degrees };
    executionBackendInstanceRef.current = lastBackendInstanceRef.current;
    executionObservedMotionRef.current = false;
    executionIdlePollsRef.current = 0;
    manualDispatchPendingRef.current = true;
    setManualMotionInFlight(true);
    setPlanPhase("executing");
    setBaseAction({ kind: "move", phase: "sending", target: degrees });
    try {
      const result = await api.targets({ joint_1: degrees });
      executionBackendInstanceRef.current = backendInstanceOf(result) ?? executionBackendInstanceRef.current;
      setBaseAction({ kind: "move", phase: "accepted", target: degrees });
    } catch (error) {
      const detail = error instanceof Error ? error.message : "That Base move was refused.";
      setCommanded((current) => ({ ...current, joint_1: undefined }));
      retireManualExecution();
      setPlanPhase("idle");
      setBaseAction({ kind: "move", phase: "failed", target: degrees, detail });
    } finally {
      manualDispatchPendingRef.current = false;
    }
  };

  const restoreBaseRange = async (joint: SimpleJoint) => {
    if (!api) return;
    cancelPlan();
    invalidateSequenceReview(sequenceWaypoints.length);
    setMessage(null);
    setBaseAction({ kind: "range", phase: "sending" });
    try {
      const next = await api.calibrate(joint.id, { minDegrees: -180, maxDegrees: 180 });
      commitMutationState(next);
      setBaseAction({ kind: "range", phase: "accepted" });
    } catch (error) {
      const detail = error instanceof Error ? error.message : "The Base range was not changed.";
      setBaseAction({ kind: "range", phase: "failed", detail });
    }
  };

  // Typed fields keep their own draft text so a half-typed "-" or "1." is not
  // rewritten under the cursor by the 150 ms poll, and commit on Enter or blur.
  //
  // They used to commit on every keystroke, which stored the number on the way
  // to the number — typing "-45" set min to -4 first — and raced its own
  // replies, so the explanation for a value the joint cannot reach was wiped by
  // the reply to the previous keystroke. Enter then looked like it did nothing,
  // because by then everything had already been sent.
  const [drafts, setDrafts] = useState<Record<string, string>>({});
  const draftKey = (joint: SimpleJoint, field: string) => `${joint.id}:${field}`;
  const typed = (joint: SimpleJoint, field: string, live: number | null) => {
    const key = draftKey(joint, field);
    return drafts[key] ?? (live === null ? "" : String(Math.round(live * 10) / 10));
  };
  /** Committing always drops the draft, so the field falls back to what was
   *  actually stored — a value snapping back is the answer to "did it take?". */
  const commitTyped = (joint: SimpleJoint, field: keyof SimpleCalibratePatch) => {
    const key = draftKey(joint, field);
    const raw = drafts[key];
    setDrafts((current) => {
      const next = { ...current };
      delete next[key];
      return next;
    });
    if (raw === undefined) return;
    const value = Number(raw);
    if (raw.trim() === "" || !Number.isFinite(value)) return;
    void apply(joint, { [field]: value } as SimpleCalibratePatch);
  };
  const editDraft = (joint: SimpleJoint, field: string) => (event: ChangeEvent<HTMLInputElement>) => {
    // Read before the updater runs: `currentTarget` is null by the time React
    // applies a lazy state update.
    const raw = event.currentTarget.value;
    setDrafts((current) => ({ ...current, [draftKey(joint, field)]: raw }));
  };
  const numberField = (joint: SimpleJoint, field: keyof SimpleCalibratePatch, live: number | null) => ({
    value: typed(joint, field, live),
    disabled: sequenceRunning,
    onChange: editDraft(joint, field),
    onKeyDown: (event: KeyboardEvent<HTMLInputElement>) => {
      if (event.key === "Enter") commitTyped(joint, field);
    },
    onBlur: () => commitTyped(joint, field),
  });

  /** "Go to" — the same numbers, but driving the joint instead of storing one.
   *  Enter and the button only: committing on blur would move the arm because
   *  the operator clicked somewhere else. */
  const goTo = (joint: SimpleJoint) => {
    const key = draftKey(joint, "goto");
    const raw = drafts[key];
    setDrafts((current) => {
      const next = { ...current };
      delete next[key];
      return next;
    });
    const value = Number(raw);
    if (raw === undefined || raw.trim() === "" || !Number.isFinite(value)) return;
    drive({ [joint.id]: value });
  };

  const target = useMemo<CartesianTarget>(() => forwardKinematics(measured, DEFAULT_GEOMETRY), [measured.joint_1, measured.joint_2, measured.joint_3]);
  const ik = useMemo(() => inverseKinematics(target, measured, draft), [draft, measured, target]);

  // The flat view hands back twin-frame angles, one joint per handle. Nothing to
  // solve and nothing to refuse, which is why it drives and the 3D twin watches.
  const modelLimits = ARM_JOINTS.reduce((table, id) => {
    const joint = joints[id];
    const edges = [toModel(id, reachOf(joint).min), toModel(id, reachOf(joint).max)];
    table[id] = { min: Math.min(...edges), max: Math.max(...edges) };
    return table;
  }, {} as Record<ArmJointId, JointLimit>);

  // The flat view speaks the twin's frame for the arm joints; the extra dials are
  // already in servo degrees, since nothing about the camera is kinematic.
  const previewModel = (next: Partial<Record<SimpleJointId, number>>) =>
    preview(Object.fromEntries(
      Object.entries(next).map(([id, degrees]) => [
        id,
        ARM_JOINTS.includes(id as ArmJointId) ? fromModel(id as ArmJointId, degrees as number) : (degrees as number),
      ]),
    ));

  const previewPose = ARM_JOINTS.reduce<ArmPose>((pose, id) => {
    const value = commanded[id];
    if (typeof value !== "number") return pose;
    pose[id] = toModel(id, value);
    return pose;
  }, { ...measured });
  const planVisible = planPhase !== "idle";
  const plannedTip = forwardKinematics(previewPose, DEFAULT_GEOMETRY);
  const measuredTip = forwardKinematics(measured, DEFAULT_GEOMETRY);
  const preparedSingleResolvedPose = preparedPlan?.resolvedPose ?? preparedFloorRoute?.waypoints.at(-1)?.resolvedPose ?? {};
  const preparedSingleWarnings = preparedPlan?.warnings ?? preparedFloorRoute?.warnings ?? [];
  const preparedSingleClearance = preparedPlan?.lowestClearanceMm ?? preparedFloorRoute?.lowestClearanceMm ?? null;
  const preparedSingleExpiresInMs = preparedPlan?.expiresInMs ?? preparedFloorRoute?.expiresInMs ?? null;
  const planRows = ALL_JOINTS.flatMap((id) => {
    const planned = commanded[id];
    const current = joints[id]?.degrees;
    if (typeof planned !== "number" || typeof current !== "number") return [];
    return [{ id, name: joints[id]?.name ?? id, current, planned }];
  });
  const clampWarnings = preparedSingle ? Object.entries(requestedTargets).flatMap(([id, requested]) => {
    const resolved = preparedSingleResolvedPose[id as SimpleJointId];
    if (typeof requested !== "number" || typeof resolved !== "number" || Math.abs(requested - resolved) < .05) return [];
    return [`${joints[id as SimpleJointId]?.name ?? id} was clamped from ${requested.toFixed(1)} degrees to ${resolved.toFixed(1)} degrees.`];
  }) : [];
  const planWarnings = [...preparedSingleWarnings, ...clampWarnings];
  const basePlanStatus = {
    idle: "Drag a grip or slider, then release to prepare a plan.",
    draft: "Draft only. Release the active control to check it on the Pi.",
    planning: "Checking limits, reach, floor clearance, and live start pose...",
    ready: "Plan ready. Review the dashed pose, then apply it.",
    applying: "Applying the exact displayed plan once...",
    executing: "Move accepted. Solid telemetry will catch up to the dashed pose.",
    error: "No move was started from this draft. Adjust it or prepare a fresh plan.",
  }[planPhase];
  const planStatus = planPhase === "ready" && preparedFloorRoute
    ? `Floor-safe route ready · ${preparedFloorRoute.waypointCount} checked legs`
    : planPhase === "applying" && preparedFloorRoute
      ? `Applying ${preparedFloorRoute.waypointCount} checked floor-safe legs on the Pi...`
      : basePlanStatus;
  const sequencePoses = (preparedSequence?.waypoints ?? []).map((waypoint) =>
    ARM_JOINTS.reduce<ArmPose>((pose, id) => {
      const degrees = waypoint.resolvedPose[id];
      if (typeof degrees === "number") pose[id] = toModel(id, degrees);
      return pose;
    }, { ...measured }),
  );
  const singleRoutePoses = (preparedFloorRoute?.waypoints ?? []).map((waypoint) =>
    ARM_JOINTS.reduce<ArmPose>((pose, id) => {
      const degrees = waypoint.resolvedPose[id];
      if (typeof degrees === "number") pose[id] = toModel(id, degrees);
      return pose;
    }, { ...measured }),
  );
  const sequenceStatus = sequencePhase === "applying"
    ? `Executing ${sequenceWaypoints.length} reviewed waypoints on the Pi…`
    : sequencePhase === "ready"
      ? "Sequence ready. Review the whole path, then apply all."
      : sequencePhase === "planning"
        ? "Checking every sweep, clearance, and inherited start pose on the Pi…"
        : sequencePhase === "complete" && sequenceResult
          ? sequenceResult.outcome === "completed"
            ? `Completed ${sequenceResult.completedWaypointCount} of ${sequenceWaypoints.length} waypoints.`
            : `${sequenceResult.outcome === "stopped" ? "Stopped" : "Sequence ended"} after ${sequenceResult.completedWaypointCount} of ${sequenceWaypoints.length} waypoints.${sequenceResult.failedWaypointIndex === null ? "" : ` Failed at waypoint ${sequenceResult.failedWaypointIndex + 1}.`}`
          : sequencePhase === "error"
            ? "The sequence is not armed. Review the error and preview all again."
            : sequenceWaypoints.length < 2
              ? "Add at least two waypoints, then preview the whole path once."
              : "Draft only. Preview all to check every segment on the Pi.";
  const controller = state?.controller;
  const bootSuffix = controller?.bootId ? controller.bootId.slice(-8) : null;
  const truthVersion = controller?.multiTurnAbsoluteV1 === true
    ? "absolute-v1"
    : controller?.multiTurnTruthV3 === true
      ? "truth-v3"
      : controller?.firmwareVersion
        ? "upgrade needed"
        : "truth unknown";
  const truthCurrent = truthVersion === "absolute-v1" || truthVersion === "truth-v3";

  return (
    <div className="simplearm">
      <header className="simplearm-bar">
        <div>
          <strong>Arm</strong>
          <span>{state ? `controller ${state.connection} · bus ${state.bus} · ${liveJointCount}/${expectedJointCount} joints live${state.refreshMs ? ` · refresh ${state.refreshMs} ms` : ""}` : "connecting"}</span>
          {state ? (
            <span className="simplearm-controller" aria-label="Controller firmware" title={controller?.controllerId ? `controller ${controller.controllerId}` : undefined}>
              firmware {controller?.firmwareVersion ?? "unknown"} · <b className={truthCurrent ? "is-v3" : truthVersion === "upgrade needed" ? "is-upgrade" : undefined}>{truthVersion}</b>{bootSuffix ? ` · boot ${bootSuffix}` : ""}
            </span>
          ) : null}
        </div>
        <div className="simplearm-bar-actions">
          <button type="button" disabled={sequenceRunning || scanBusy} onClick={() => void scanArm()}>{scanBusy ? "Scanning…" : "Scan"}</button>
          <button type="button" disabled={sequenceRunning} aria-expanded={renaming} onClick={() => setRenaming((open) => !open)}>Servo ID</button>
          <button type="button" disabled={sequenceRunning || (!held.length && (!busReady || !allJointsOnline))} onClick={() => void (held.length ? setHold([]) : holdAll())}>{held.length ? "Release all" : "Hold all"}</button>
          {/* The keep-out plane, not a range of motion. Off lets the arm be
              driven below the floor deliberately; the Pi holds the flag, so the
              drag constraint and the server-side clamp switch together. */}
          <button
            type="button"
            className={`simplearm-floorguard${floorGuardOn ? " is-active" : ""}`}
            aria-pressed={floorGuardOn}
            disabled={sequenceRunning}
            title={floorGuardOn
              ? "The arm cannot be dragged below the floor. Click to allow it."
              : "The arm CAN be driven below the floor. Click to protect it."}
            onClick={() => {
              cancelPlan();
              invalidateSequenceReview(sequenceWaypoints.length);
              void api?.floorGuard(!floorGuardOn).then(commitMutationState).catch(() => setMessage("Floor guard did not change."));
            }}
          >Floor guard{floorGuardOn ? " on" : " off"}</button>
          <button type="button" className="simplearm-stop" onClick={() => {
            if (!sequenceRunning) {
              cancelPlan();
              invalidateSequenceReview(sequenceWaypoints.length);
            }
            void api?.stop().then(commitMutationState).catch(() => setMessage("STOP delivery unknown — cut servo power before touching the arm."));
          }}>STOP</button>
        </div>
      </header>

      {stopped ? (
        <p className="simplearm-note" role="alert">
          STOP is latched — the controller is refusing every command. Check the arm, then
          <button type="button" disabled={sequenceRunning} onClick={() => void api?.clearStop().then(commitMutationState).catch((error) => setMessage(error instanceof Error ? error.message : "STOP would not clear."))}>Clear STOP</button>
        </p>
      ) : null}
      {/* "Gateway unreachable" used to be shown for this, which sent everyone
          looking at the Pi while the Pi was answering every request. The
          gateway replied — it is the serial link to the HAT that is down, and
          the fix is a reconnect, not a restart. */}
      {state && state.connection !== "online" ? (
        <p className="simplearm-note" role="alert">
          {state.connection === "faulted"
            ? "The Pi is fine and answering — its serial link to the arm controller is faulted."
            : "The Pi is answering, but no arm controller is connected. Check the HAT's power and ribbon."}
          <button
            type="button"
            disabled={sequenceRunning}
            onClick={() => {
              cancelPlan();
              invalidateSequenceReview(sequenceWaypoints.length);
              void api?.reconnectController().then(commitMutationState).catch((error) => setMessage(error instanceof Error ? error.message : "Reconnect failed."));
            }}
          >Reconnect controller</button>
        </p>
      ) : null}
      {/* An id clash presents as a bus with nothing on it, which is the same
          thing a loose plug looks like. The controller can tell them apart, so
          say which one it is rather than leaving an empty panel to interpret. */}
      {/* Shown without needing a scan: a collision refuses the scan at the
          torque-off stage, so the case that most needs explaining is exactly
          the one that never produces scan evidence. */}
      {!state?.collisionSuspected && genericBusTrouble ? (
        <p className="simplearm-note" role="alert">{genericBusTrouble}</p>
      ) : null}
      {state?.collisionSuspected ? (
        <p className="simplearm-note" role="alert">
          Two servos are answering to id {state.collisionId ?? "the same number"} — they talk over
          each other, so neither can be reached. A new Waveshare servo ships as id 1, the same as
          the base. Unplug all but one, then use <b>Servo ID</b> to give it its own number.
        </p>
      ) : null}
      {message ? <p className="simplearm-note" role="alert">{message}<button type="button" onClick={() => setMessage(null)}>Dismiss</button></p> : null}

      {/* A bench operation, not a control: it burns an ID into a servo's EEPROM,
          and the controller refuses unless exactly one servo answers the bus. A
          factory-fresh Waveshare servo is ID 1, same as the base, so a new one
          has to be renamed before it can join. */}
      {renaming ? (
        <form
          className="simplearm-rename"
          onSubmit={(event) => {
            event.preventDefault();
            const from = Number(renameFrom);
            const to = Number(renameTo);
            if (!Number.isInteger(from) || !Number.isInteger(to)) { setMessage("Servo IDs are whole numbers."); return; }
            cancelPlan();
            invalidateSequenceReview(sequenceWaypoints.length);
            void api?.assignId(from, to)
              .then((next) => { commitMutationState(next); setRenaming(false); setMessage(`Servo ${from} is now ID ${to}.`); })
              .catch((error) => setMessage(error instanceof Error ? error.message : "The ID was not written."));
          }}
        >
          <strong>Set servo ID</strong>
          <span>Only one servo may be on the bus. Unplug the others first.</span>
          <label>from<input type="number" step="1" min="0" max="253" aria-label="Current servo id" disabled={sequenceRunning} value={renameFrom} onChange={(event) => setRenameFrom(event.currentTarget.value)} /></label>
          <label>to<input type="number" step="1" min="0" max="253" aria-label="New servo id" disabled={sequenceRunning} value={renameTo} onChange={(event) => setRenameTo(event.currentTarget.value)} /></label>
          <button type="submit" disabled={sequenceRunning}>Write</button>
          <button type="button" onClick={() => setRenaming(false)}>Cancel</button>
        </form>
      ) : null}
      {connecting && !state ? <p className="simplearm-note">Connecting…</p> : null}

      <div className="simplearm-body">
        <div className="simplearm-stage">
          <div className="simplearm-viewtabs" role="group" aria-label="Arm view">
            <button type="button" className={view === "flat" ? "is-active" : ""} aria-pressed={view === "flat"} onClick={() => setView("flat")}>2D</button>
            <button type="button" className={view === "twin" ? "is-active" : ""} aria-pressed={view === "twin"} onClick={() => setView("twin")}>3D</button>
          </div>
          {!livePoseAvailable ? (
            <section className="simplearm-pose-unavailable" role="status" aria-label="Live arm pose unavailable">
              <strong>Live pose unavailable</strong>
              <span>Waiting for fresh servo telemetry. The dashboard will not invent joint angles from stored calibration.</span>
              <small>Run Scan to identify responding servo IDs. Motion remains disabled; STOP stays available.</small>
            </section>
          ) : view === "flat" ? (
            <Arm2D
              pose={previewPose} measuredPose={measured} planned={planVisible}
              sequencePoses={motionMode === "sequence" ? sequencePoses : singleRoutePoses}
              disabled={sequenceRunning || !busReady}
              limits={modelLimits} geometry={DEFAULT_GEOMETRY}
              floorMm={floorMm}
              onPose={previewModel} onRelease={preparePlan}
              /* Servo degrees, not the twin's frame: the number beside a joint
                 has to be the same one its panel shows, or they contradict
                 each other on screen. Commanded wins while a move is in
                 flight, so the label leads the drawing the same way it does. */
              readouts={{
                joint_1: commanded.joint_1 ?? joints.joint_1?.degrees ?? null,
                joint_2: commanded.joint_2 ?? joints.joint_2?.degrees ?? null,
                joint_3: commanded.joint_3 ?? joints.joint_3?.degrees ?? null,
              }}
              /* Not in the chain, so no model offset and no link length: the
                 camera is its own dial, and its wedge is whatever calibration
                 measured rather than a fixed sweep. */
              /* Shown before calibration too, just inert. Hiding it until a
                 zero existed meant the dial was simply absent at exactly the
                 moment someone went looking for it, with nothing on screen
                 saying why -- and an uncalibrated joint has no angle to drive
                 to, so drawing it live would be a lie. */
              extras={AUX_JOINTS.flatMap((id) => {
                const joint = joints[id];
                if (!joint) return [];
                return [{
                  id,
                  name: joint.name,
                    degrees: commanded[id] ?? joint.degrees ?? 0,
                    measuredDegrees: joint.degrees ?? 0,
                  limit: reachOf(joint),
                  pending: !joint.calibrated,
                }];
              })}
            />
          ) : (
            <ArmViewport
              currentPose={measured}
              previewPose={previewPose}
              geometry={DEFAULT_GEOMETRY}
              target={target}
              targetReachable={ik.reachable}
            />
          )}
          <section
            className={`simplearm-plan${motionMode === "sequence" ? " is-sequence" : ""} is-${motionMode === "sequence" ? sequencePhase : planPhase}`}
            aria-label={motionMode === "sequence" ? "Waypoint sequence plan" : "2D move plan"}
            aria-live="polite"
          >
            <div className="simplearm-plan-heading">
              <div>
                <strong>{motionMode === "sequence" ? "Waypoint sequence" : "2D move plan"}</strong>
                <span>{motionMode === "sequence" ? (sequencePhase === "ready" ? "checked on Pi" : sequencePhase) : (planPhase === "ready" ? "checked on Pi" : planPhase)}</span>
              </div>
              <div className="simplearm-motion-mode" role="group" aria-label="Motion mode">
                <button type="button" aria-label="Single move mode" aria-pressed={motionMode === "single"} disabled={sequenceRunning} onClick={() => chooseMotionMode("single")}>Single</button>
                <button type="button" aria-label="Sequence mode" aria-pressed={motionMode === "sequence"} disabled={sequenceRunning} onClick={() => chooseMotionMode("sequence")}>Sequence</button>
              </div>
              <p>{motionMode === "sequence" ? sequenceStatus : planStatus}</p>
            </div>

            {motionMode === "sequence" ? (
              <div className="simplearm-sequence-details">
                {sequenceWaypoints.length ? (
                  <ol className="simplearm-sequence-list" aria-label="Waypoint sequence">
                    {sequenceWaypoints.map((waypoint, index) => {
                      const reviewed = preparedSequence?.waypoints[index];
                      const receipt = sequenceResult?.waypointResults.find((candidate) => candidate.index === index);
                      return (
                        <li key={waypoint.id} className={editingWaypoint === index ? "is-editing" : undefined}>
                          <span className="simplearm-sequence-index">{index + 1}</span>
                          <span className="simplearm-sequence-pose">
                            {ALL_JOINTS.map((id) => `${joints[id]?.name ?? id} ${waypoint.targets[id]?.toFixed(1) ?? "—"}°`).join(" · ")}
                          </span>
                          {reviewed?.lowestClearanceMm != null ? <span className={reviewed.lowestClearanceMm < 10 ? "is-warning" : undefined}>{reviewed.lowestClearanceMm.toFixed(1)} mm clear</span> : null}
                          {receipt ? <span className={`simplearm-sequence-result is-${receipt.outcome}`}>{receipt.outcome}</span> : null}
                          {receipt && [receipt.dispatchMs, receipt.arrivalMs, receipt.durationMs].some((value) => typeof value === "number") ? (
                            <span className="simplearm-sequence-timing">
                              {[
                                typeof receipt.dispatchMs === "number" ? `${formatDurationMs(receipt.dispatchMs)} dispatch` : null,
                                typeof receipt.arrivalMs === "number" ? `${formatDurationMs(receipt.arrivalMs)} arrival` : null,
                                typeof receipt.durationMs === "number" ? `${formatDurationMs(receipt.durationMs)} total` : null,
                              ].filter(Boolean).join(" · ")}
                            </span>
                          ) : null}
                          <span className="simplearm-sequence-row-actions">
                            <button type="button" aria-label={`Edit waypoint ${index + 1}`} disabled={sequenceRunning} onClick={() => editSequenceWaypoint(index)}>Edit</button>
                            <button type="button" aria-label={`Move waypoint ${index + 1} up`} disabled={sequenceRunning || index === 0} onClick={() => moveSequenceWaypoint(index, -1)}>↑</button>
                            <button type="button" aria-label={`Move waypoint ${index + 1} down`} disabled={sequenceRunning || index === sequenceWaypoints.length - 1} onClick={() => moveSequenceWaypoint(index, 1)}>↓</button>
                            <button type="button" aria-label={`Remove waypoint ${index + 1}`} disabled={sequenceRunning} onClick={() => removeSequenceWaypoint(index)}>Remove</button>
                          </span>
                          {reviewed?.warnings.length ? <ul className="simplearm-plan-warnings">{reviewed.warnings.map((warning, warningIndex) => <li key={warningIndex}>{warning}</li>)}</ul> : null}
                        </li>
                      );
                    })}
                  </ol>
                ) : null}
                {preparedSequence?.lowestClearanceMm != null ? (
                  <p className={`simplearm-plan-clearance${preparedSequence.lowestClearanceMm < 10 ? " is-warning" : ""}`}>
                    Whole-path lowest clearance {preparedSequence.lowestClearanceMm.toFixed(1)} mm
                  </p>
                ) : null}
                {sequencePreviewDurationMs !== null ? <p className="simplearm-sequence-overall">Preview {formatDurationMs(sequencePreviewDurationMs)}</p> : null}
                {sequenceResult && typeof sequenceResult.durationMs === "number" ? <p className="simplearm-sequence-overall">Overall {formatDurationMs(sequenceResult.durationMs)}</p> : null}
                {preparedSequence?.warnings.length ? <ul className="simplearm-plan-warnings" aria-label="Sequence warnings">{preparedSequence.warnings.map((warning, index) => <li key={index}>{warning}</li>)}</ul> : null}
              </div>
            ) : planRows.length ? (
              <div className="simplearm-plan-details">
                <dl aria-label="Current and planned joint angles">
                  {planRows.map((row) => (
                    <div key={row.id}>
                      <dt>{row.name}</dt>
                      <dd>{row.current.toFixed(1)}{"\u00b0"} <span aria-hidden="true">&rarr;</span> {row.planned.toFixed(1)}{"\u00b0"}</dd>
                    </div>
                  ))}
                </dl>
                {ARM_JOINTS.some((id) => typeof commanded[id] === "number") ? (
                  <p className="simplearm-plan-tip">
                    Tip radial {Math.hypot(measuredTip.x, measuredTip.y).toFixed(0)} &rarr; {Math.hypot(plannedTip.x, plannedTip.y).toFixed(0)} mm
                    <span>height {measuredTip.z.toFixed(0)} &rarr; {plannedTip.z.toFixed(0)} mm</span>
                  </p>
                ) : null}
                {preparedSingleClearance != null ? (
                  <p className={`simplearm-plan-clearance${preparedSingleClearance < 10 ? " is-warning" : ""}`}>
                    Lowest clearance {preparedSingleClearance.toFixed(1)} mm
                  </p>
                ) : null}
                {planWarnings.length ? (
                  <ul className="simplearm-plan-warnings" aria-label="Plan warnings">
                    {planWarnings.map((warning, index) => <li key={`${warning}-${index}`}>{warning}</li>)}
                  </ul>
                ) : null}
              </div>
            ) : null}

            <div className="simplearm-plan-actions">
              {motionMode === "sequence" ? (
                <>
                  <button type="button" disabled={sequenceRunning || (editingWaypoint === null && (!sequenceDraftDirty || sequenceWaypoints.length >= 8))} onClick={storeWaypoint}>{editingWaypoint === null ? "Add waypoint" : "Update waypoint"}</button>
                  <button type="button" disabled={sequenceRunning || sequencePhase === "planning" || sequenceDraftDirty || editingWaypoint !== null || sequenceWaypoints.length < 2 || sequenceWaypoints.length > 8} onClick={() => void previewSequence()}>Preview all</button>
                  <button type="button" className="simplearm-plan-apply" disabled={sequenceRunning || !preparedSequence || sequencePhase !== "ready" || !busReady || !livePoseAvailable} onClick={() => void applySequence()}>Apply all</button>
                  <button type="button" disabled={sequenceRunning || sequenceWaypoints.length === 0} onClick={clearSequence}>Clear all</button>
                  {preparedSequence?.expiresInMs != null ? <span>reviewed for {Math.max(0, Math.ceil(preparedSequence.expiresInMs / 1000))}s</span> : null}
                </>
              ) : (
                <>
                  <button type="button" className="simplearm-plan-apply" disabled={!preparedSingle || planPhase !== "ready" || !busReady || !livePoseAvailable} onClick={() => void applyPlan()}>Apply move</button>
                  <button type="button" disabled={planPhase === "idle" || planPhase === "applying" || planPhase === "executing"} onClick={cancelPlan}>Cancel</button>
                  {preparedSingleExpiresInMs != null ? <span>prepared with {Math.max(0, Math.ceil(preparedSingleExpiresInMs / 1000))}s validity</span> : null}
                </>
              )}
            </div>
          </section>
        </div>

        {/* Tabs, not an action button. Switching what the side panel shows is a
            view choice, and mixing it in beside Scan and STOP made it read as
            something that does a thing to the arm. */}
        <div className="simplearm-side">
          <div className="simplearm-sidetabs" role="tablist" aria-label="Side panel">
            <button
              type="button" role="tab" id="arm-tab-joints"
              aria-selected={panel === "joints"} aria-controls="arm-panel-joints"
              className={panel === "joints" ? "is-active" : ""}
              onClick={() => setPanel("joints")}
            >Joints</button>
            <button
              type="button" role="tab" id="arm-tab-camera"
              aria-selected={panel === "camera"} aria-controls="arm-panel-camera"
              className={panel === "camera" ? "is-active" : ""}
              onClick={() => setPanel("camera")}
            >Camera</button>
          </div>
        {panel === "camera" ? (
          <div className="simplearm-sidepanel simplearm-camera-panel" role="tabpanel" id="arm-panel-camera" aria-labelledby="arm-tab-camera">
            <CameraPanel
              status={cameraStatus}
              observation={cameraObservation}
              frameUrl={cameraFrameUrl}
              busy={cameraBusy}
              error={cameraError}
              autofocusMessage={cameraAutofocusMessage}
              canCapture={Boolean(actionToken && cameraStatus?.available && !sequenceRunning)}
              onCapture={(profile) => void takeCameraPicture(profile)}
              onAutofocus={() => void tryCameraAutofocus()}
            />
          </div>
        ) : (
        <div className="simplearm-joints" role="tabpanel" id="arm-panel-joints" aria-labelledby="arm-tab-joints">
          {ALL_JOINTS.map((id) => {
            const joint = joints[id];
            if (!joint) return null;
            const free = !held.includes(joint.servoId);
            // Fail closed across partial/stale deployments: the Base moves only
            // when the backend explicitly asserts that its multi-turn frame is
            // trusted. An omitted field is not evidence of position.
            const positionTrusted = joint.positionTrusted === true;
            const { min, max } = reachOf(joint);
            const value = commanded[id] ?? joint.degrees ?? 0;
            return (
              <section key={id} className={`simplearm-joint${selected === id ? " is-selected" : ""}${free ? " is-free" : ""}`} onFocus={() => setSelected(id)}>
                <header>
                  <strong>{joint.name}</strong>
                  <output>{fixed(joint.degrees)}°</output>
                  <button type="button" disabled={sequenceRunning || (free && (!busReady || !joint.online))} className={`simplearm-torque${free ? " is-free" : ""}`} onClick={() => void (free ? holdAll() : freeOnly(joint))}>{free ? "Hold" : "Free"}</button>
                </header>

                {id === "joint_1" ? (
                  <BaseTruthStrip
                    joint={joint}
                    absolute={controller?.multiTurnAbsoluteV1 === true}
                    scanDetected={baseDetectedByScan}
                  />
                ) : null}

                {id === "joint_1" ? (
                  <BaseSelfService
                    joint={joint}
                    scanDetected={baseDetectedByScan}
                    free={free}
                    stopped={stopped}
                    apiReady={api !== null && !sequenceRunning && !scanBusy && !manualMotionInFlight && busReady}
                    action={baseAction}
                    onHome={() => void homeBase(joint)}
                    onRestoreRange={() => void restoreBaseRange(joint)}
                    onMove={(degrees) => void moveBase(degrees)}
                  />
                ) : null}

                <input
                  className="simplearm-drive"
                  type="range" step="0.1"
                  aria-label={`Drive ${joint.name}`}
                  min={min} max={max} value={value}
                  disabled={sequenceRunning || !busReady || !joint.calibrated || !joint.online || !positionTrusted || max - min < 1}
                  // Same contract as the flat view: the bar moves freely, the
                  // arm gets one goal when it is let go.
                  onChange={(event) => preview({ [id]: Number(event.currentTarget.value) })}
                  onPointerUp={preparePlan}
                  onPointerCancel={preparePlan}
                  onKeyUp={preparePlan}
                />

                {/* Dragging is fine for finding a pose; typing is the only way
                    to ask for a number you already know. */}
                <label className="simplearm-goto">
                  <span>go to</span>
                  <input
                    type="number" step="any"
                    aria-label={`Move ${joint.name} to`}
                    disabled={sequenceRunning || !busReady || !joint.calibrated || !joint.online || !positionTrusted || max - min < 1}
                    value={typed(joint, "goto", joint.degrees ?? null)}
                    onChange={editDraft(joint, "goto")}
                    onKeyDown={(event) => { if (event.key === "Enter") goTo(joint); }}
                  />
                  <i>°</i>
                  <button
                    type="button"
                    disabled={sequenceRunning || !busReady || !joint.calibrated || !joint.online || !positionTrusted || max - min < 1}
                    onClick={() => goTo(joint)}
                  >go</button>
                </label>

                {/* Every number is typeable; "here" stores wherever the joint
                    is standing, for when moving it by hand is easier. */}
                <div className="simplearm-fields">
                  {([
                    { field: "minDegrees" as const, capture: "rawMin" as const, label: "min", unit: "°", live: joint.minDegrees },
                    { field: "rawZero" as const, capture: "rawZero" as const, label: "zero", unit: "raw", live: joint.rawZero },
                    { field: "maxDegrees" as const, capture: "rawMax" as const, label: "max", unit: "°", live: joint.maxDegrees },
                  ]).map((entry) => (
                    <label key={entry.field}>
                      <span>{entry.label}</span>
                      {/* step="any" for degrees: a live angle is shown to one
                          decimal, and a browser refuses to accept anything in a
                          field whose value is off its own step grid — 38.3 with
                          step .5 rejected every edit, including whole numbers. */}
                      <input
                        type="number" step={entry.unit === "raw" ? "1" : "any"}
                        aria-label={`${joint.name} ${entry.label}`}
                        {...numberField(joint, entry.field, entry.live)}
                      />
                      {/* min and max are typed in degrees, zero in encoder
                          counts. Unlabelled they look interchangeable. */}
                      <i>{entry.unit}</i>
                      {joint.id === "joint_1" && entry.capture === "rawZero" ? null : (
                        <button
                          type="button"
                          aria-label={`Set ${joint.name} ${entry.label} here`}
                          disabled={sequenceRunning || !positionTrusted}
                          onClick={() => captureHere(joint, entry.capture)}
                        >here</button>
                      )}
                      {/* Zero belongs halfway between the limits, so the joint has
                          the same travel either side of it. */}
                      {entry.capture === "rawZero" ? (
                        <button
                          type="button"
                          aria-label={`Centre ${joint.name} zero between min and max`}
                          disabled={sequenceRunning || joint.rawMin === null || joint.rawMax === null}
                          onClick={() => void apply(joint, { zeroFromLimits: true })}
                        >mid</button>
                      ) : null}
                    </label>
                  ))}
                </div>

                <div className="simplearm-tuning">
                  <label>
                    <span>ratio</span>
                    <input
                      type="number" step="any" min="0.02" max="64"
                      aria-label={`${joint.name} ratio`}
                      {...numberField(joint, "ratio", joint.ratio)}
                    />
                  </label>
                  <label>
                    <span>speed</span>
                    <input
                      type="number" step="1" min="1" max="2400"
                      aria-label={`${joint.name} speed`}
                      {...numberField(joint, "speed", joint.speed)}
                    />
                  </label>
                  <label>
                    <span>accel</span>
                    <input
                      type="number" step="1" min="1" max="50"
                      aria-label={`${joint.name} acceleration`}
                      {...numberField(joint, "accel", joint.accel)}
                    />
                  </label>
                  {/* Which servo this joint drives. Separate from burning an ID
                      into a servo — that is the bench tool in the header. */}
                  <label>
                    <span>id</span>
                    <input
                      type="number" step="1" min="0" max="253"
                      aria-label={`${joint.name} servo id`}
                      {...numberField(joint, "servoId", joint.servoId)}
                    />
                  </label>
                  <label>
                    <span>dir</span>
                    <button
                      type="button" className="simplearm-direction"
                      disabled={sequenceRunning}
                      aria-label={`${joint.name} direction, currently ${joint.direction === 1 ? "positive" : "reversed"}`}
                      onClick={() => void apply(joint, { direction: joint.direction === 1 ? -1 : 1 })}
                    >{joint.direction === 1 ? "+ →" : "− ←"}</button>
                  </label>
                </div>

                {!positionTrusted ? (
                  <p className="simplearm-pinned" role="alert">
                    Base position is unknown after controller continuity was lost. Keep it free, place it
                    exactly on the marked physical zero, then press <b>zero here</b>. Drive and limit
                    capture stay locked until that explicit home is verified.
                  </p>
                ) : joint.calibrated && max - min < 1 ? (
                  <p className="simplearm-pinned">
                    Min and max are {((max - min) * 60).toFixed(0)}′ apart — this joint is pinned.
                    Free it, move it to one end and press <b>here</b> on min, then the other end for max.
                  </p>
                ) : isPinched(joint) ? (
                  /* The limits were accepted, so nothing looked wrong — but the
                     servo has one turn of travel and at this ratio that is not
                     enough to cover them. Without this the far end of the bar is
                     simply dead and the joint stops for no stated reason. */
                  <p className="simplearm-pinned">
                    One motor turn at {joint.ratio}:1 only spans {fixed((joint.reachMax ?? 0) - (joint.reachMin ?? 0), 0)}°, so
                    this joint can only reach <b>{fixed(joint.reachMin, 0)}° … {fixed(joint.reachMax, 0)}°</b> of
                    its {fixed(joint.minDegrees, 0)}° … {fixed(joint.maxDegrees, 0)}° limits.
                    Press <b>mid</b> to centre that window, or lower the ratio if the joint really turns further.
                  </p>
                ) : null}

                <footer>
                  <span>raw {joint.rawPosition ?? "—"}</span>
                  {/* A min and max at the same encoder count is a zero-wide
                      window: the bar renders but the joint can never leave the
                      spot, which reads as the drive being broken. */}
                  <span className={isPinched(joint) ? "is-warning" : undefined}>{fixed(min, 0)}° … {fixed(max, 0)}°</span>
                  <span>{free ? "free" : "held"}</span>
                </footer>
              </section>
            );
          })}
          <p className="simplearm-hint">Free a joint, move it by hand, then press <b>here</b> on min and max. Then press <b>mid</b> to drop the zero exactly between them. That is the whole calibration. Typed numbers apply on <b>Enter</b>; if one snaps back, the joint cannot reach it and the banner says why.</p>
        </div>
        )}
        </div>
      </div>
    </div>
  );
}
