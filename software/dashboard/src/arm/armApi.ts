import type { ArmLiveFollowJointSettings, ArmLiveFollowReceipt, ArmLiveFollowSettings, ArmLiveFollowTargets, CameraAutofocusResponse, CameraCaptureProfile, CameraObservation, CameraStatus, PhysicalArmStatus, PhysicalCalibrationProfilePayload, PhysicalProfileEnvelope, SimpleArmPlan, SimpleArmPlanExecution, SimpleArmSequenceExecution, SimpleArmSequencePlan, SimpleArmSequenceWaypointInput, SimpleArmState, SimpleCalibratePatch, SimpleJointId } from "./armTypes";

export class ArmApiError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
    this.name = "ArmApiError";
  }
}

export interface LiveFollowStartAttempt {
  startAttemptId: string;
  startTimeoutMs: number;
  signal?: AbortSignal;
}

export interface LiveFollowStartCancelledReceipt {
  schema: "arm-live-follow-v2";
  startAttemptId: string;
  cancelled: true;
  inputLeaseMs: number;
}

const CAMERA_FRAME_ID = /^[A-Za-z0-9_-]{1,128}$/;
const MAX_CAMERA_FRAME_BYTES = 16 * 1024 * 1024;
export const LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS = 200;
export const LIVE_FOLLOW_START_TIMEOUT_MS = 1_500;
const LIVE_FOLLOW_START_ATTEMPT_ID = /^[A-Za-z0-9][A-Za-z0-9_-]{7,127}$/;
let liveFollowStartAttemptSequence = 0;

export function createLiveFollowStartAttemptId() {
  const uuid = globalThis.crypto?.randomUUID?.();
  if (uuid) return uuid;
  liveFollowStartAttemptSequence += 1;
  return `browser_${Date.now().toString(36)}_${liveFollowStartAttemptSequence.toString(36)}`;
}

function errorMessage(status: number, body: unknown) {
  if (body && typeof body === "object" && "detail" in body && typeof (body as { detail?: unknown }).detail === "string") {
    return String((body as { detail: string }).detail);
  }
  return `Arm gateway returned HTTP ${status}.`;
}

async function json(request: typeof fetch, url: string, actionToken: string | null, init?: RequestInit) {
  const response = await request(url, {
    ...init,
    headers: {
      Accept: "application/json",
      ...(init?.body ? { "Content-Type": "application/json" } : {}),
      ...(actionToken ? { "X-Co-Arm-Token": actionToken } : {}),
      ...init?.headers,
    },
  });
  let body: unknown = null;
  try { body = await response.json(); } catch { body = null; }
  if (!response.ok) throw new ArmApiError(errorMessage(response.status, body), response.status);
  return body as Record<string, unknown>;
}

export async function loadActionToken(request: typeof fetch) {
  const response = await request("/api/session", { headers: { Accept: "application/json" } });
  const body = await response.json() as { actionToken?: unknown };
  if (!response.ok || typeof body.actionToken !== "string" || body.actionToken.length < 32) {
    throw new ArmApiError("The local arm action session is unavailable.", response.status);
  }
  return body.actionToken;
}

const isRecord = (value: unknown): value is Record<string, unknown> => value !== null && typeof value === "object" && !Array.isArray(value);
const isFiniteNumber = (value: unknown): value is number => typeof value === "number" && Number.isFinite(value);
const isNonNegativeInteger = (value: unknown): value is number => Number.isInteger(value) && (value as number) >= 0;
const validLiveFollowJointSettings = (value: unknown): value is ArmLiveFollowJointSettings => isRecord(value)
  && Number.isInteger(value.speed)
  && (value.speed as number) >= 1
  && (value.speed as number) <= 2400
  && Number.isInteger(value.accel)
  && (value.accel as number) >= 1
  && (value.accel as number) <= 50
  && Number.isInteger(value.maxDeltaDegrees)
  && (value.maxDeltaDegrees as number) >= 1
  && (value.maxDeltaDegrees as number) <= 90;

const validLiveFollowSettings = (value: unknown): value is ArmLiveFollowSettings => isRecord(value)
  && validLiveFollowJointSettings(value.joint_2)
  && validLiveFollowJointSettings(value.joint_3);

function parseLiveFollowReceipt(body: unknown, expectedStartAttemptId?: string, expectedSessionId?: string): ArmLiveFollowReceipt {
  if (!isRecord(body)) throw new ArmApiError("The arm returned an invalid live-follow receipt.", 502);
  const envelope = body.envelope;
  const commanded = body.commanded;
  const measured = body.measured;
  const settings = body.settings;
  const stats = body.stats;
  const timing = body.timing;
  const states = new Set(["active", "ended", "expired", "faulted", "stopped"]);
  const validLimit = (value: unknown) => isRecord(value)
    && isFiniteNumber(value.minDegrees)
    && isFiniteNumber(value.maxDegrees)
    && value.minDegrees <= value.maxDegrees;
  const validTargets = (value: unknown) => isRecord(value)
    && isFiniteNumber(value.joint_2)
    && isFiniteNumber(value.joint_3);
  const validMeasuredJoint = (value: unknown) => isRecord(value)
    && isFiniteNumber(value.degrees)
    && isFiniteNumber(value.rawPosition)
    && isFiniteNumber(value.velocityDegreesPerSecond)
    && typeof value.moving === "boolean"
    && isFiniteNumber(value.packetAgeMs)
    && value.packetAgeMs >= 0;
  const validStats = isRecord(stats)
    && ["receivedFrames", "dispatchedFrames", "coalescedFrames", "clampedFrames", "lastAcceptedSequence", "lastDispatchedSequence"]
      .every((key) => isNonNegativeInteger(stats[key]))
    && (stats.lastDispatchLatencyMs === null
      || (isFiniteNumber(stats.lastDispatchLatencyMs) && stats.lastDispatchLatencyMs >= 0));
  const validTiming = isRecord(timing)
    && ["inputLeaseMs", "maxDurationMs", "minDispatchIntervalMs"].every((key) => isFiniteNumber(timing[key]) && (timing[key] as number) > 0)
    && typeof timing.startedAt === "string"
    && timing.startedAt.length > 0
    && typeof timing.expiresAt === "string"
    && timing.expiresAt.length > 0;
  const validEnvelope = isRecord(envelope)
    && isRecord(envelope.maxDeltaDegrees)
    && Number.isInteger(envelope.maxDeltaDegrees.joint_2)
    && (envelope.maxDeltaDegrees.joint_2 as number) > 0
    && (envelope.maxDeltaDegrees.joint_2 as number) <= 90
    && Number.isInteger(envelope.maxDeltaDegrees.joint_3)
    && (envelope.maxDeltaDegrees.joint_3 as number) > 0
    && (envelope.maxDeltaDegrees.joint_3 as number) <= 90
    && isFiniteNumber(envelope.floorMm)
    && isFiniteNumber(envelope.lowestPointMm)
    && validLimit(envelope.joint_2)
    && validLimit(envelope.joint_3);
  const validMeasured = isRecord(measured)
    && validMeasuredJoint(measured.joint_2)
    && validMeasuredJoint(measured.joint_3);
  if (
    body.schema !== "arm-live-follow-v2"
    || typeof body.sessionId !== "string"
    || body.sessionId.length === 0
    || (expectedStartAttemptId !== undefined && body.startAttemptId !== expectedStartAttemptId)
    || (expectedSessionId !== undefined && body.sessionId !== expectedSessionId)
    || typeof body.state !== "string"
    || !states.has(body.state)
    || !(body.reason === null || typeof body.reason === "string")
    || !validLiveFollowSettings(settings)
    || !validEnvelope
    || !validTargets(commanded)
    || !validMeasured
    || !validStats
    || !validTiming
  ) {
    throw new ArmApiError("The arm returned an invalid live-follow receipt.", 502);
  }
  return body as unknown as ArmLiveFollowReceipt;
}

function parseLiveFollowStartCancelReceipt(body: unknown, expectedAttemptId: string): LiveFollowStartCancelledReceipt | ArmLiveFollowReceipt {
  if (!isRecord(body) || body.startAttemptId !== expectedAttemptId) {
    throw new ArmApiError("The arm returned a mismatched live-follow start cancellation receipt.", 502);
  }
  if (
    body.schema === "arm-live-follow-v2"
    && LIVE_FOLLOW_START_ATTEMPT_ID.test(expectedAttemptId)
    && body.cancelled === true
    && isFiniteNumber(body.inputLeaseMs)
    && body.inputLeaseMs > 0
  ) return body as unknown as LiveFollowStartCancelledReceipt;
  return parseLiveFollowReceipt(body, expectedAttemptId);
}

export function createCameraApi(request: typeof fetch, actionToken: string | null) {
  return {
    async status() {
      return await json(request, "/api/camera/status", null) as unknown as CameraStatus;
    },
    async capture(profile: CameraCaptureProfile = "detail") {
      if (!actionToken) throw new ArmApiError("A local action session is required to capture an observation.", 401);
      if (profile !== "survey" && profile !== "detail") throw new ArmApiError("The camera capture profile is invalid.", 422);
      return await json(request, `/api/camera/captures?profile=${profile}`, actionToken, { method: "POST" }) as unknown as CameraObservation;
    },
    async autofocus() {
      if (!actionToken) throw new ArmApiError("A local action session is required to try camera autofocus.", 401);
      return await json(request, "/api/camera/autofocus", actionToken, { method: "POST" }) as unknown as CameraAutofocusResponse;
    },
    async latest() {
      if (!actionToken) throw new ArmApiError("A local action session is required to read camera evidence.", 401);
      return await json(request, "/api/camera/observations/latest", actionToken) as unknown as CameraObservation;
    },
    async frame(frameId: string) {
      if (!actionToken) throw new ArmApiError("A local action session is required to read camera evidence.", 401);
      if (!CAMERA_FRAME_ID.test(frameId)) throw new ArmApiError("The camera frame identifier is invalid.", 422);
      const response = await request(`/api/camera/frames/${frameId}`, {
        headers: {
          Accept: "image/jpeg",
          "X-Co-Arm-Token": actionToken,
        },
      });
      if (!response.ok) {
        let body: unknown = null;
        try { body = await response.json(); } catch { body = null; }
        throw new ArmApiError(errorMessage(response.status, body), response.status);
      }
      if (response.headers.get("Content-Type")?.split(";", 1)[0]?.trim().toLowerCase() !== "image/jpeg") {
        throw new ArmApiError("The camera proxy did not return verified JPEG evidence.", 502);
      }
      const frame = await response.blob();
      if (frame.size === 0 || frame.size > MAX_CAMERA_FRAME_BYTES) {
        throw new ArmApiError("The camera frame is empty or exceeds the evidence limit.", 502);
      }
      return frame;
    },
  };
}

/** The Pi-backed arm controls. `targets` remains for explicit commissioning
 * presets and compatibility; ordinary 2D, slider, and typed movement goes
 * through previewPlan -> executePlan so the pose shown is the pose consumed. */
export function createSimpleArmApi(request: typeof fetch, actionToken: string) {
  const post = (path: string, body?: unknown, init?: RequestInit) => json(request, `/api/arm/simple${path}`, actionToken, {
    ...init,
    method: "POST",
    ...(body === undefined ? {} : { body: JSON.stringify(body) }),
  });
  return {
    async state() {
      return await json(request, "/api/arm/simple/state", actionToken) as unknown as SimpleArmState;
    },
    async scan() {
      return await post("/scan") as unknown as SimpleArmState;
    },
    async calibrate(joint: SimpleJointId, patch: SimpleCalibratePatch) {
      return await post(`/joints/${joint}/calibrate`, patch) as unknown as SimpleArmState;
    },
    /** Every joint in one round trip; the controller dispatches servo families back to back. */
    async targets(pose: Partial<Record<SimpleJointId, number>>) {
      return await post("/target", pose);
    },
    /** Open one bounded REAL-only Shoulder/Elbow streaming lease. */
    async startLiveFollow(settings: ArmLiveFollowSettings, attempt: LiveFollowStartAttempt) {
      if (!validLiveFollowSettings(settings)) {
        throw new ArmApiError("Each live-follow joint needs integer speed 1 to 2400, acceleration 1 to 50, and travel 1 to 90 degrees.", 422);
      }
      if (
        !LIVE_FOLLOW_START_ATTEMPT_ID.test(attempt.startAttemptId)
        || !Number.isInteger(attempt.startTimeoutMs)
        || attempt.startTimeoutMs < 1
        || attempt.startTimeoutMs > LIVE_FOLLOW_START_TIMEOUT_MS
      ) throw new ArmApiError("The live-follow start attempt is invalid.", 422);
      return parseLiveFollowReceipt(await post("/live-follow/start", {
        joint_2: { ...settings.joint_2 },
        joint_3: { ...settings.joint_3 },
        startAttemptId: attempt.startAttemptId,
        startTimeoutMs: attempt.startTimeoutMs,
      }, attempt.signal ? { signal: attempt.signal } : undefined), attempt.startAttemptId);
    },
    /** Tombstone one start attempt. This terminal request is never retried. */
    async cancelLiveFollowStart(startAttemptId: string, signal?: AbortSignal) {
      if (!LIVE_FOLLOW_START_ATTEMPT_ID.test(startAttemptId)) {
        throw new ArmApiError("The live-follow start attempt is invalid.", 422);
      }
      return parseLiveFollowStartCancelReceipt(await post(
        "/live-follow/start/cancel",
        { startAttemptId },
        signal ? { signal } : undefined,
      ), startAttemptId);
    },
    /** Latest-wins frame acceptance. A receipt acknowledges dispatch, not arrival. */
    async sendLiveFollowFrame(sessionId: string, sequence: number, targets: ArmLiveFollowTargets, signal?: AbortSignal) {
      if (!sessionId || !Number.isInteger(sequence) || sequence < 1 || !isFiniteNumber(targets.joint_2) || !isFiniteNumber(targets.joint_3)) {
        throw new ArmApiError("The live-follow frame is invalid.", 422);
      }
      return parseLiveFollowReceipt(await post("/live-follow/frame", {
        sessionId,
        sequence,
        joint_2: targets.joint_2,
        joint_3: targets.joint_3,
      }, signal ? { signal } : undefined), undefined, sessionId);
    },
    /** Renew browser presence independently from the ordered motion-frame lane. */
    async heartbeatLiveFollow(sessionId: string) {
      if (!sessionId) throw new ArmApiError("The live-follow heartbeat request is invalid.", 422);
      const controller = new AbortController();
      let timeout: ReturnType<typeof globalThis.setTimeout> | null = null;
      const deadline = new Promise<never>((_resolve, reject) => {
        timeout = globalThis.setTimeout(() => {
          // Reject first so an AbortError from fetch cannot hide which safety
          // deadline expired. A heartbeat timeout is terminal and is never
          // retried: the session owner must fail closed instead.
          reject(new ArmApiError(
            `Live heartbeat timed out after ${LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS} ms. The session is ending without retrying.`,
            408,
          ));
          controller.abort();
        }, LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS);
      });
      try {
        return parseLiveFollowReceipt(await Promise.race([
          post("/live-follow/heartbeat", { sessionId }, { signal: controller.signal }),
          deadline,
        ]), undefined, sessionId);
      } finally {
        if (timeout !== null) globalThis.clearTimeout(timeout);
      }
    },
    /** End the lease. A deliberate flush retains the final bounded hold;
     * cancellation discards pending input and releases Shoulder/Elbow authority. */
    async endLiveFollow(sessionId: string, flushPending: boolean, signal?: AbortSignal) {
      if (!sessionId || typeof flushPending !== "boolean") throw new ArmApiError("The live-follow end request is invalid.", 422);
      return parseLiveFollowReceipt(await post("/live-follow/end", { sessionId, flushPending }, signal ? { signal } : undefined), undefined, sessionId);
    },
    /** Resolve and guard a pose without acquiring torque or moving hardware. */
    async previewPlan(targets: Partial<Record<SimpleJointId, number>>) {
      const body = await post("/plans/preview", { targets });
      // `digest` was the short-lived development name. Accept it at this edge
      // while exposing one stable name to the rest of the dashboard.
      const planDigest = typeof body.planDigest === "string"
        ? body.planDigest
        : typeof body.digest === "string"
          ? body.digest
          : "";
      if (typeof body.planId !== "string" || !planDigest) {
        throw new ArmApiError("The arm planner returned an incomplete plan.", 502);
      }
      return {
        ...body,
        planId: body.planId,
        planDigest,
        expiresAt: typeof body.expiresAt === "string" ? body.expiresAt : null,
        expiresInMs: typeof body.expiresInMs === "number" ? body.expiresInMs : null,
        measuredPose: body.measuredPose && typeof body.measuredPose === "object" ? body.measuredPose : {},
        resolvedPose: body.resolvedPose && typeof body.resolvedPose === "object" ? body.resolvedPose : {},
        warnings: Array.isArray(body.warnings)
          ? body.warnings.map((warning) => typeof warning === "string"
            ? warning
            : warning && typeof warning === "object" && "message" in warning
              ? String((warning as { message: unknown }).message)
              : String(warning))
          : [],
        lowestClearanceMm: typeof body.lowestClearanceMm === "number" ? body.lowestClearanceMm : null,
      } as SimpleArmPlan;
    },
    /** Consume exactly the plan that was displayed. The Pi rejects stale,
     * replayed, or changed-boot plans; no target values are recalculated here. */
    async executePlan(plan: Pick<SimpleArmPlan, "planId" | "planDigest">) {
      return await post("/plans/execute", {
        planId: plan.planId,
        planDigest: plan.planDigest,
      }) as unknown as SimpleArmPlanExecution;
    },
    /** Resolve every segment from the previous waypoint without moving hardware. */
    async previewSequence(waypoints: SimpleArmSequenceWaypointInput[]) {
      const body = await post("/sequences/preview", { waypoints });
      if (typeof body.sequenceId !== "string" || typeof body.sequenceDigest !== "string") {
        throw new ArmApiError("The arm planner returned an incomplete sequence.", 502);
      }
      const rows = Array.isArray(body.waypoints) ? body.waypoints : [];
      return {
        ...body,
        sequenceId: body.sequenceId,
        sequenceDigest: body.sequenceDigest,
        expiresAt: typeof body.expiresAt === "string" ? body.expiresAt : null,
        expiresInMs: typeof body.expiresInMs === "number" ? body.expiresInMs : null,
        previewDurationMs: typeof body.previewDurationMs === "number" ? body.previewDurationMs : null,
        measuredPose: body.measuredPose && typeof body.measuredPose === "object" ? body.measuredPose : {},
        waypointCount: typeof body.waypointCount === "number" ? body.waypointCount : rows.length,
        waypoints: rows,
        warnings: Array.isArray(body.warnings) ? body.warnings.map(String) : [],
        lowestClearanceMm: typeof body.lowestClearanceMm === "number" ? body.lowestClearanceMm : null,
      } as SimpleArmSequencePlan;
    },
    /** Consume the exact reviewed multi-waypoint artifact once. The Pi owns all
     * intermediate dispatch, arrival checks, STOP handling, and the receipt. */
    async executeSequence(
      sequence: Pick<SimpleArmSequencePlan, "sequenceId" | "sequenceDigest" | "waypointCount">,
      arrivalTimeoutMs = 10_000,
    ) {
      const body = await post("/sequences/execute", {
        sequenceId: sequence.sequenceId,
        sequenceDigest: sequence.sequenceDigest,
        arrivalTimeoutMs,
      });
      const completedValue = body.completedWaypointCount;
      const completed = typeof completedValue === "number" && Number.isInteger(completedValue)
        ? completedValue
        : -1;
      const failedValue = body.failedWaypointIndex;
      const failed = failedValue === null
        ? null
        : typeof failedValue === "number" && Number.isInteger(failedValue)
          ? failedValue
          : -1;
      const outcome = typeof body.outcome === "string" ? body.outcome : "";
      const hasResults = Array.isArray(body.waypointResults);
      const results: unknown[] = Array.isArray(body.waypointResults) ? body.waypointResults : [];
      const expected = sequence.waypointCount;
      const failureOutcomes = new Set(["stopped", "arrival_timeout", "arrival_fault", "refused"]);
      const structurallyValid = Number.isInteger(expected)
        && expected >= 2
        && expected <= 8
        && body.sequenceId === sequence.sequenceId
        && typeof body.operationId === "string"
        && body.operationId.length > 0
        && typeof body.executed === "boolean"
        && body.waypointCount === expected
        && (outcome === "completed" || failureOutcomes.has(outcome))
        && completed >= 0
        && completed <= expected
        && (failed === null || (failed >= 0 && failed < expected))
        && hasResults
        && results.every((row: unknown, index: number) => {
          if (row === null || typeof row !== "object") return false;
          const candidate = row as {
            index?: unknown;
            outcome?: unknown;
            dispatched?: unknown;
            arrival?: unknown;
          };
          const arrival = candidate.arrival;
          const rowOutcome = typeof candidate.outcome === "string" ? candidate.outcome : "";
          return candidate.index === index
            && index < expected
            && (rowOutcome === "arrived" || failureOutcomes.has(rowOutcome))
            && typeof candidate.dispatched === "boolean"
            && arrival !== null
            && typeof arrival === "object"
            && typeof (arrival as { proved?: unknown }).proved === "boolean"
            && ((rowOutcome === "arrived") === ((arrival as { proved: boolean }).proved))
            && (rowOutcome !== "arrived" || candidate.dispatched === true)
            && (rowOutcome !== "refused" || candidate.dispatched === false)
            && (!["arrival_timeout", "arrival_fault"].includes(rowOutcome) || candidate.dispatched === true);
        });
      const dispatchedAny = results.some((row) => row !== null
        && typeof row === "object"
        && (row as { dispatched?: unknown }).dispatched === true);
      const completedReceipt = structurallyValid
        && outcome === "completed"
        && body.executed === true
        && completed === expected
        && failed === null
        && results.length === expected
        && results.every((row) => (row as { outcome?: unknown }).outcome === "arrived");
      const partialReceipt = structurallyValid
        && failureOutcomes.has(outcome)
        && completed < expected
        && failed === completed
        && results.length === completed + 1
        && results.slice(0, completed).every((row) => (row as { outcome?: unknown }).outcome === "arrived")
        && (results[completed] as { outcome?: unknown } | undefined)?.outcome === outcome
        && body.executed === dispatchedAny;
      if (
        !structurallyValid
        || (!completedReceipt && !partialReceipt)
      ) {
        throw new ArmApiError("The arm returned an invalid sequence execution receipt.", 502);
      }
      return body as unknown as SimpleArmSequenceExecution;
    },
    async torque(hold: number[]) {
      return await post("/torque", { hold }) as unknown as SimpleArmState;
    },
    /** Burns a new ID into the servo's EEPROM. One servo on the bus only. */
    async assignId(oldId: number, newId: number) {
      return await post("/servo-id", { oldId, newId }) as unknown as SimpleArmState;
    },
    async stop(signal?: AbortSignal) {
      return await post("/stop", undefined, signal ? { signal } : undefined) as unknown as SimpleArmState;
    },
    async clearStop() {
      return await post("/clear-stop") as unknown as SimpleArmState;
    },
    /** Turns the keep-out plane on or off. The Pi owns it, so the 2D scene and
     *  the server-side clamp switch together instead of disagreeing. */
    async floorGuard(enabled: boolean) {
      return await post("/floor-guard", { enabled }) as unknown as SimpleArmState;
    },
    /** Re-opens the Pi's serial link to the HAT. A faulted controller never
     *  retries on its own -- by design, so a fault cannot be papered over --
     *  which means the panel has to offer the one action that fixes it. */
    async reconnectController() {
      await json(request, "/api/arm/physical/controller/reconnect", actionToken, { method: "POST" });
      return await json(request, "/api/arm/simple/state", actionToken) as unknown as SimpleArmState;
    },
  };
}

export interface PhysicalServoCapture {
  servoId: number;
  rawPosition: number;
  sampleCount?: number;
  variationTicks?: number;
  evidenceId?: string;
  capturedAt?: string;
  odometerValid?: boolean;
  revolutions?: number;
  multiTurnPosition?: number;
}

export interface PhysicalServoOdometer {
  servoId: number;
  tracking: boolean;
  valid: boolean;
  revolutions: number;
  rawPosition: number;
  multiTurnPosition: number;
  sampleAgeMs: number;
}

export interface PreparedPhysicalNudge {
  proposalId: string;
  proposalHash?: string;
  expiresAt?: string;
  servoId: number;
  deltaTicks: number;
  expiresInMs: number;
}

export interface ExecutedPhysicalNudge {
  completed: boolean;
  startRawPosition: number;
  targetRawPosition: number;
  rawPosition: number;
  measuredDeltaTicks: number;
  positionErrorTicks: number;
  torqueState: "off" | "on" | "unknown";
  evidenceId?: string;
}

export function createPhysicalArmApi(request: typeof fetch, actionToken: string) {
  return {
    async status() {
      return await json(request, "/api/arm/physical/status", actionToken) as unknown as PhysicalArmStatus;
    },
    async reconnectController() {
      return await json(request, "/api/arm/physical/controller/reconnect", actionToken, { method: "POST" }) as unknown as PhysicalArmStatus;
    },
    async scanBus(minId: number, maxId: number) {
      return await json(request, "/api/arm/physical/bus/scan", actionToken, {
        method: "POST",
        body: JSON.stringify({ minId, maxId }),
      });
    },
    async assignServoId(oldId: number, newId: number, acknowledgedSingleServo: boolean, confirmedServoModel: "ST3215") {
      return await json(request, "/api/arm/physical/servos/assign-id", actionToken, {
        method: "POST",
        body: JSON.stringify({ oldId, newId, acknowledgedSingleServo, confirmedServoModel }),
      });
    },
    async setServoPositionMode(servoId: number, acknowledgedSingleServo: boolean, confirmedServoModel: "ST3215") {
      return await json(request, "/api/arm/physical/servos/set-position-mode", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId, acknowledgedSingleServo, confirmedServoModel }),
      });
    },
    async captureServo(servoId: number) {
      return await json(request, "/api/arm/physical/servos/capture", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId }),
      }) as unknown as PhysicalServoCapture;
    },
    async moveServo(servoId: number, goal: number, speed: number, acceleration: number) {
      return await json(request, "/api/arm/physical/servos/move", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId, goal, speed, acceleration, acknowledgedPhysicalPowerCut: true, confirmedServoModel: "ST3215" }),
      });
    },
    async readServoRegisters(servoId: number, address: number, length: number) {
      return await json(request, "/api/arm/physical/servos/registers/read", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId, address, length }),
      }) as unknown as { servoId: number; address: number; length: number; values: number[] };
    },
    async odometerZero(servoId: number) {
      return await json(request, "/api/arm/physical/servos/odometer/zero", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId }),
      }) as unknown as PhysicalServoOdometer;
    },
    async odometerRead(servoId: number) {
      return await json(request, "/api/arm/physical/servos/odometer/read", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId }),
      }) as unknown as PhysicalServoOdometer;
    },
    async startTorqueLease(servoId: number, leaseMs: number, acknowledgedPhysicalPowerCut: boolean, confirmedServoModel: "ST3215") {
      return await json(request, "/api/arm/physical/servos/torque-lease", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId, leaseMs, acknowledgedPhysicalPowerCut, confirmedServoModel }),
      });
    },
    async setTorqueHoldSet(servoIds: number[], leaseMs: number, acknowledgedPhysicalPowerCut: boolean, confirmedServoModel: "ST3215") {
      return await json(request, "/api/arm/physical/servos/hold-set", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoIds, leaseMs, acknowledgedPhysicalPowerCut, confirmedServoModel }),
      });
    },
    async torqueOff(servoId: number | null) {
      return await json(request, "/api/arm/physical/servos/torque-off", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId }),
      });
    },
    async prepareNudge(servoId: number, zeroEvidenceId: string, deltaTicks: number, speed: number, acceleration: number, acknowledgedPhysicalPowerCut: boolean, confirmedServoModel: "ST3215") {
      return await json(request, "/api/arm/physical/tests/prepare-nudge", actionToken, {
        method: "POST",
        body: JSON.stringify({ servoId, zeroEvidenceId, deltaTicks, speed, acceleration, acknowledgedPhysicalPowerCut, confirmedServoModel }),
      }) as unknown as PreparedPhysicalNudge;
    },
    async executeNudge(proposal: PreparedPhysicalNudge, acknowledgedPhysicalPowerCut: true) {
      return await json(request, "/api/arm/physical/tests/execute-nudge", actionToken, {
        method: "POST",
        body: JSON.stringify({
          proposalId: proposal.proposalId,
          ...(proposal.proposalHash ? { proposalHash: proposal.proposalHash } : {}),
          acknowledgedPhysicalPowerCut,
        }),
      }) as unknown as ExecutedPhysicalNudge;
    },
    async calibrationProfile() {
      return await json(request, "/api/arm/physical/calibration/profile", actionToken) as unknown as PhysicalProfileEnvelope;
    },
    async commitCalibrationProfile(profile: PhysicalCalibrationProfilePayload) {
      return await json(request, "/api/arm/physical/calibration/profile", actionToken, {
        method: "POST",
        body: JSON.stringify(profile),
      });
    },
    async stop() {
      return await json(request, "/api/arm/physical/stop", actionToken, { method: "POST" });
    },
    async resetAfterInspection() {
      return await json(request, "/api/arm/physical/reset", actionToken, {
        method: "POST",
        body: JSON.stringify({ acknowledgedPhysicalInspection: true }),
      });
    },
  };
}
