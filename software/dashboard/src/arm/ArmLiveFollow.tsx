import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Arm2D, type JointLimit } from "./Arm2D";
import {
  createLiveFollowStartAttemptId,
  createSimpleArmApi,
  LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS,
  LIVE_FOLLOW_START_TIMEOUT_MS,
  loadActionToken,
} from "./armApi";
import { fromModel, toModel } from "./SimpleArm";
import {
  DEFAULT_GEOMETRY,
  type ArmJointId,
  type ArmLiveFollowJointId,
  type ArmLiveFollowJointSettings,
  type ArmLiveFollowReceipt,
  type ArmLiveFollowSettings,
  type ArmLiveFollowTargets,
  type ArmPose,
  type SimpleArmState,
  type SimpleJoint,
  type SimpleJointId,
} from "./armTypes";
import "./arm-live-follow.css";

const LIVE_JOINTS = ["joint_2", "joint_3"] as const;
const SPEED_PRESETS = [400, 800, 1400, 2000, 2400] as const;
const TRAVEL_PRESETS = [15, 30, 45, 60, 90] as const;
const SETTING_LIMITS = {
  speed: { min: 1, max: 2400 },
  accel: { min: 1, max: 50 },
  maxDeltaDegrees: { min: 1, max: 90 },
} as const;
const DEFAULT_SETTINGS: ArmLiveFollowSettings = {
  joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
  joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
};
const POLL_MS = 125;
const ACTIVE_POLL_MS = 50;
const MIN_FRAME_INTERVAL_MS = 50;
const DEFAULT_INPUT_LEASE_MS = 400;
const TERMINAL_RECOVERY_MARGIN_MS = (LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS * 2) + 100;
const START_UNCONFIRMED_RECOVERY_MS = LIVE_FOLLOW_START_TIMEOUT_MS + 4_250;
const TERMINAL_UNCONFIRMED_RECOVERY_MS = START_UNCONFIRMED_RECOVERY_MS;
const EMPTY_POSE: ArmPose = { joint_1: 0, joint_2: 90, joint_3: -90 };

type Phase = "idle" | "starting" | "active" | "ending";
type EndReason = "release" | "cancel" | "blur" | "hidden" | "manual";
type ArmApi = ReturnType<typeof createSimpleArmApi>;
type SettingKey = keyof ArmLiveFollowJointSettings;
type SettingDrafts = Record<ArmLiveFollowJointId, Record<SettingKey, string>>;

interface LiveStartAttempt {
  id: string;
  generation: number;
  startedAt: number;
  api: ArmApi;
  controller: AbortController;
  cancelController: AbortController | null;
  cancelStarted: boolean;
}

interface RunSummary {
  id: number;
  settings: ArmLiveFollowSettings;
  durationMs: number;
  peak: ArmLiveFollowTargets;
  maxError: number;
  dispatched: number;
  coalesced: number;
  reason: EndReason | "stop" | "fault";
}

export interface ArmLiveFollowProps {
  request?: typeof fetch;
  backendId: "sim" | "real";
  onControlBusyChange?: (busy: boolean) => void;
}

const defaultRequest: typeof fetch = (...arguments_) => globalThis.fetch(...arguments_);
const finite = (value: unknown): value is number => typeof value === "number" && Number.isFinite(value);
const degrees = (value: number | null | undefined) => finite(value) ? `${value.toFixed(1)}°` : "n/a";
const speedText = (value: number | null | undefined) => finite(value) ? `${value.toFixed(0)}°/s` : "n/a";
const durationText = (milliseconds: number) => `${(milliseconds / 1000).toFixed(2)}s`;
const messageOf = (error: unknown) => error instanceof Error ? error.message : "Live follow failed.";
const jointLabel = (joint: ArmLiveFollowJointId) => joint === "joint_2" ? "Shoulder" : "Elbow";
const cloneSettings = (settings: ArmLiveFollowSettings): ArmLiveFollowSettings => ({
  joint_2: { ...settings.joint_2 },
  joint_3: { ...settings.joint_3 },
});
const draftsFor = (settings: ArmLiveFollowSettings): SettingDrafts => ({
  joint_2: {
    speed: String(settings.joint_2.speed),
    accel: String(settings.joint_2.accel),
    maxDeltaDegrees: String(settings.joint_2.maxDeltaDegrees),
  },
  joint_3: {
    speed: String(settings.joint_3.speed),
    accel: String(settings.joint_3.accel),
    maxDeltaDegrees: String(settings.joint_3.maxDeltaDegrees),
  },
});
const boundedInteger = (key: SettingKey, raw: string, fallback: number) => {
  const parsed = Number(raw.trim());
  if (!Number.isFinite(parsed)) return fallback;
  const { min, max } = SETTING_LIMITS[key];
  return Math.min(max, Math.max(min, Math.round(parsed)));
};
const normalizedSettings = (settings: ArmLiveFollowSettings, drafts: SettingDrafts): ArmLiveFollowSettings => ({
  joint_2: {
    speed: boundedInteger("speed", drafts.joint_2.speed, settings.joint_2.speed),
    accel: boundedInteger("accel", drafts.joint_2.accel, settings.joint_2.accel),
    maxDeltaDegrees: boundedInteger("maxDeltaDegrees", drafts.joint_2.maxDeltaDegrees, settings.joint_2.maxDeltaDegrees),
  },
  joint_3: {
    speed: boundedInteger("speed", drafts.joint_3.speed, settings.joint_3.speed),
    accel: boundedInteger("accel", drafts.joint_3.accel, settings.joint_3.accel),
    maxDeltaDegrees: boundedInteger("maxDeltaDegrees", drafts.joint_3.maxDeltaDegrees, settings.joint_3.maxDeltaDegrees),
  },
});

function jointsOf(state: SimpleArmState | null) {
  const joints = {} as Partial<Record<SimpleJointId, SimpleJoint>>;
  for (const joint of state?.joints ?? []) joints[joint.id] = joint;
  return joints;
}

function measuredTargets(state: SimpleArmState | null): ArmLiveFollowTargets | null {
  const joints = jointsOf(state);
  const shoulder = joints.joint_2?.degrees;
  const elbow = joints.joint_3?.degrees;
  return finite(shoulder) && finite(elbow) ? { joint_2: shoulder, joint_3: elbow } : null;
}

function modelPose(state: SimpleArmState | null): ArmPose {
  const joints = jointsOf(state);
  return {
    joint_1: toModel("joint_1", finite(joints.joint_1?.degrees) ? joints.joint_1.degrees : 0),
    joint_2: toModel("joint_2", finite(joints.joint_2?.degrees) ? joints.joint_2.degrees : 0),
    joint_3: toModel("joint_3", finite(joints.joint_3?.degrees) ? joints.joint_3.degrees : 0),
  };
}

function modelLimit(joint: ArmJointId, minimum: number, maximum: number): JointLimit {
  const a = toModel(joint, minimum);
  const b = toModel(joint, maximum);
  return { min: Math.min(a, b), max: Math.max(a, b) };
}

function readiness(state: SimpleArmState | null) {
  if (!state) return "Waiting for current arm telemetry.";
  if (state.connection !== "online") return "The REAL controller is offline.";
  if (state.bus !== "online") return "The servo bus is not healthy.";
  if (state.stopped) return "STOP is latched. Clear it from Control before enabling live follow.";
  if (state.baseReferenceRequired === true) return "Recovery requires physical Base alignment and Set zero here in Control before enabling live follow.";
  if (state.controller?.liveFollowV1 !== true) return "Controller firmware 2.7+ with Fast Follow feedback support is required.";
  if (state.floorGuard?.enabled !== true) return "Floor guard must be on for live follow.";
  const joints = jointsOf(state);
  for (const id of ["joint_2", "joint_3"] as const) {
    const joint = joints[id];
    if (!joint?.online) return `${joint?.name ?? id} is offline.`;
    if (!joint.calibrated) return `${joint.name} is not calibrated.`;
    if (!joint.positionTrusted) return `${joint.name} telemetry is not trusted.`;
    if (!finite(joint.degrees)) return `${joint.name} has no measured angle.`;
  }
  return null;
}

function baseReferenceUnknown(state: SimpleArmState | null) {
  if (!state) return false;
  const base = jointsOf(state).joint_1;
  return !base
    || !base.online
    || !finite(base.rawZero)
    || base.positionTrusted !== true
    || !finite(base.rawPosition)
    || !finite(base.degrees);
}

export function ArmLiveFollow({ request = defaultRequest, backendId, onControlBusyChange }: ArmLiveFollowProps) {
  const [phase, setPhase] = useState<Phase>("idle");
  const [state, setState] = useState<SimpleArmState | null>(null);
  const [receipt, setReceipt] = useState<ArmLiveFollowReceipt | null>(null);
  const [settings, setSettings] = useState<ArmLiveFollowSettings>(() => cloneSettings(DEFAULT_SETTINGS));
  const [settingDrafts, setSettingDrafts] = useState<SettingDrafts>(() => draftsFor(DEFAULT_SETTINGS));
  const [ghost, setGhost] = useState<ArmPose>(EMPTY_POSE);
  const [target, setTarget] = useState<ArmLiveFollowTargets | null>(null);
  const [sessionOrigin, setSessionOrigin] = useState<ArmLiveFollowTargets | null>(null);
  const [velocity, setVelocity] = useState<ArmLiveFollowTargets>({ joint_2: 0, joint_3: 0 });
  const [rttMs, setRttMs] = useState<number | null>(null);
  const [dispatched, setDispatched] = useState(0);
  const [coalesced, setCoalesced] = useState(0);
  const [message, setMessage] = useState<string | null>(null);
  const [history, setHistory] = useState<RunSummary[]>([]);

  const mounted = useRef(true);
  const apiRef = useRef<ArmApi | null>(null);
  const tokenPromise = useRef<Promise<ArmApi> | null>(null);
  const phaseRef = useRef<Phase>("idle");
  const sessionRef = useRef<string | null>(null);
  const generationRef = useRef(0);
  const sequenceRef = useRef(0);
  const pendingRef = useRef<ArmLiveFollowTargets | null>(null);
  const inFlightRef = useRef(false);
  const heartbeatInFlightRef = useRef(false);
  const lastFrameStartRef = useRef<number | null>(null);
  const pumpTimerRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const terminalFrameAbortTimerRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const terminalWatchdogRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const endRequestRef = useRef<{ reason: EndReason; flush: boolean } | null>(null);
  const endCallRef = useRef(false);
  const frameAbortRef = useRef<AbortController | null>(null);
  const endAbortRef = useRef<AbortController | null>(null);
  const startAttemptRef = useRef<LiveStartAttempt | null>(null);
  const startDeadlineTimerRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const startRecoveryTimerRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const stopAbortRef = useRef<AbortController | null>(null);
  const stopWatchdogRef = useRef<ReturnType<typeof globalThis.setTimeout> | null>(null);
  const stopCallRef = useRef(false);
  const inputLeaseMsRef = useRef(DEFAULT_INPUT_LEASE_MS);
  const ghostRef = useRef<ArmPose>(EMPTY_POSE);
  const targetRef = useRef<ArmLiveFollowTargets | null>(null);
  const dispatchedRef = useRef(0);
  const coalescedRef = useRef(0);
  const sampleRef = useRef<Record<ArmLiveFollowJointId, { at: number; degrees: number } | null>>({ joint_2: null, joint_3: null });
  const statePollInFlightRef = useRef(false);
  const runRef = useRef<{ id: number; startedAt: number; settings: ArmLiveFollowSettings; peak: ArmLiveFollowTargets; maxError: number; dispatched: number; coalesced: number } | null>(null);
  const nextRunId = useRef(1);
  const pumpRef = useRef<() => void>(() => undefined);

  const setCurrentPhase = useCallback((next: Phase) => {
    phaseRef.current = next;
    if (mounted.current) setPhase(next);
  }, []);

  const ensureApi = useCallback(async () => {
    if (apiRef.current) return apiRef.current;
    if (!tokenPromise.current) {
      tokenPromise.current = loadActionToken(request).then((token) => {
        const api = createSimpleArmApi(request, token);
        apiRef.current = api;
        return api;
      }).finally(() => {
        tokenPromise.current = null;
      });
    }
    return await tokenPromise.current;
  }, [request]);

  const recordRun = useCallback((reason: RunSummary["reason"]) => {
    const run = runRef.current;
    if (!run) return;
    runRef.current = null;
    const row: RunSummary = {
      id: run.id,
      settings: cloneSettings(run.settings),
      durationMs: Math.max(0, performance.now() - run.startedAt),
      peak: run.peak,
      maxError: run.maxError,
      dispatched: run.dispatched,
      coalesced: run.coalesced,
      reason,
    };
    if (mounted.current) setHistory((current) => [row, ...current].slice(0, 5));
  }, []);

  const closeLocal = useCallback((reason: RunSummary["reason"]) => {
    if (pumpTimerRef.current !== null) {
      globalThis.clearTimeout(pumpTimerRef.current);
      pumpTimerRef.current = null;
    }
    if (terminalFrameAbortTimerRef.current !== null) {
      globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
      terminalFrameAbortTimerRef.current = null;
    }
    if (terminalWatchdogRef.current !== null) {
      globalThis.clearTimeout(terminalWatchdogRef.current);
      terminalWatchdogRef.current = null;
    }
    if (startDeadlineTimerRef.current !== null) {
      globalThis.clearTimeout(startDeadlineTimerRef.current);
      startDeadlineTimerRef.current = null;
    }
    if (startRecoveryTimerRef.current !== null) {
      globalThis.clearTimeout(startRecoveryTimerRef.current);
      startRecoveryTimerRef.current = null;
    }
    startAttemptRef.current?.controller.abort();
    startAttemptRef.current?.cancelController?.abort();
    startAttemptRef.current = null;
    if (stopWatchdogRef.current !== null) {
      globalThis.clearTimeout(stopWatchdogRef.current);
      stopWatchdogRef.current = null;
    }
    stopAbortRef.current?.abort();
    stopAbortRef.current = null;
    stopCallRef.current = false;
    frameAbortRef.current?.abort();
    frameAbortRef.current = null;
    endAbortRef.current?.abort();
    endAbortRef.current = null;
    sessionRef.current = null;
    pendingRef.current = null;
    endRequestRef.current = null;
    endCallRef.current = false;
    inFlightRef.current = false;
    heartbeatInFlightRef.current = false;
    setCurrentPhase("idle");
    recordRun(reason);
  }, [recordRun, setCurrentPhase]);

  const cancelStartAttempt = useCallback((attempt: LiveStartAttempt, reason: string) => {
    if (startAttemptRef.current !== attempt || attempt.cancelStarted) return;
    attempt.cancelStarted = true;
    attempt.controller.abort();
    setCurrentPhase("ending");
    if (mounted.current) setMessage(reason);

    const remaining = Math.max(0, attempt.startedAt + START_UNCONFIRMED_RECOVERY_MS - performance.now());
    startRecoveryTimerRef.current = globalThis.setTimeout(() => {
      startRecoveryTimerRef.current = null;
      if (startAttemptRef.current !== attempt || generationRef.current !== attempt.generation) return;
      generationRef.current += 1;
      if (mounted.current) {
        setMessage("Start cancellation was not confirmed. Local controls recovered only after the proxy, start-deadline, and input-lease safety window; no command was retried.");
      }
      closeLocal(endRequestRef.current?.reason ?? "fault");
    }, remaining);

    const controller = new AbortController();
    attempt.cancelController = controller;
    void attempt.api.cancelLiveFollowStart(attempt.id, controller.signal).then((next) => {
      if (startAttemptRef.current !== attempt || generationRef.current !== attempt.generation) return;
      inputLeaseMsRef.current = "cancelled" in next ? next.inputLeaseMs : next.timing.inputLeaseMs;
      if (!("cancelled" in next) && mounted.current) setReceipt(next);
      generationRef.current += 1;
      if (mounted.current) setMessage("The bounded live-follow start attempt was cancelled; no start request was retried.");
      closeLocal(endRequestRef.current?.reason ?? "fault");
    }).catch((error) => {
      if (startAttemptRef.current !== attempt || generationRef.current !== attempt.generation) return;
      if (mounted.current) {
        setMessage(`Start cancellation was not confirmed: ${messageOf(error)} Waiting for the full safety window; no command will be retried.`);
      }
    });
  }, [closeLocal, setCurrentPhase]);

  const finishEnd = useCallback(async (reason: EndReason, flushPending: boolean) => {
    if (endCallRef.current) return;
    const sessionId = sessionRef.current;
    if (!sessionId) {
      closeLocal(reason);
      return;
    }
    endCallRef.current = true;
    setCurrentPhase("ending");
    const generation = generationRef.current;
    const controller = new AbortController();
    endAbortRef.current = controller;
    let confirmed = false;
    try {
      const api = await ensureApi();
      const next = await api.endLiveFollow(sessionId, flushPending, controller.signal);
      inputLeaseMsRef.current = next.timing.inputLeaseMs;
      if (generation === generationRef.current && runRef.current) {
        runRef.current.dispatched = next.stats.dispatchedFrames;
        runRef.current.coalesced = next.stats.coalescedFrames;
      }
      if (mounted.current && generation === generationRef.current) setReceipt(next);
      confirmed = true;
    } catch (error) {
      if (mounted.current && generation === generationRef.current) {
        setMessage(`Live session end was not confirmed: ${messageOf(error)} Waiting for the safety lease window; no command will be retried.`);
      }
    } finally {
      if (endAbortRef.current === controller) endAbortRef.current = null;
      if (confirmed && generation === generationRef.current) closeLocal(reason);
    }
  }, [closeLocal, ensureApi, setCurrentPhase]);

  const armTerminalFrameDeadline = useCallback((controller: AbortController, generation: number) => {
    if (terminalFrameAbortTimerRef.current !== null) globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
    terminalFrameAbortTimerRef.current = globalThis.setTimeout(() => {
      terminalFrameAbortTimerRef.current = null;
      if (
        generation === generationRef.current
        && phaseRef.current === "ending"
        && frameAbortRef.current === controller
      ) {
        const endRequest = endRequestRef.current;
        pendingRef.current = null;
        if (endRequest?.flush) endRequestRef.current = { ...endRequest, flush: false };
        if (mounted.current) setMessage("Final target confirmation timed out. Ending once without flushing or retrying ambiguous input.");
        controller.abort();
      }
    }, LIVE_FOLLOW_HEARTBEAT_TIMEOUT_MS);
  }, []);

  const scheduleTerminalWatchdog = useCallback((reason: EndReason) => {
    if (terminalWatchdogRef.current !== null) return;
    const generation = generationRef.current;
    terminalWatchdogRef.current = globalThis.setTimeout(() => {
      terminalWatchdogRef.current = null;
      if (generation !== generationRef.current || phaseRef.current !== "ending") return;

      // A request can be accepted by the Pi even when its browser response is
      // lost. Wait beyond the final possible renewal plus the controller lease,
      // then recover only local ownership. Never replay a frame, heartbeat, or
      // end request whose outcome is ambiguous.
      generationRef.current += 1;
      frameAbortRef.current?.abort();
      endAbortRef.current?.abort();
      if (mounted.current) {
        setMessage("Pi end confirmation did not arrive. Local controls recovered after the full unconfirmed input-chain safety window; no command was retried.");
      }
      closeLocal(reason);
    }, TERMINAL_UNCONFIRMED_RECOVERY_MS);
  }, [closeLocal]);

  const requestEnd = useCallback((reason: EndReason, flush: boolean) => {
    if (phaseRef.current === "idle") return;
    const previousPhase = phaseRef.current;
    endRequestRef.current = { reason, flush };
    if (!flush) pendingRef.current = null;
    // Close the input lane synchronously. A deliberate End may still dispatch
    // the one value that was already pending, but no later pointer event can
    // enter the pump while that flush or an in-flight frame drains.
    setCurrentPhase("ending");
    if (previousPhase === "starting") {
      const attempt = startAttemptRef.current;
      if (attempt) cancelStartAttempt(attempt, "Live-follow start was cancelled before the backend attempt completed.");
      return;
    }
    scheduleTerminalWatchdog(reason);

    if (!flush) {
      frameAbortRef.current?.abort();
      void finishEnd(reason, false);
      return;
    }
    if (frameAbortRef.current) armTerminalFrameDeadline(frameAbortRef.current, generationRef.current);
    pumpRef.current();
  }, [armTerminalFrameDeadline, cancelStartAttempt, finishEnd, scheduleTerminalWatchdog, setCurrentPhase]);

  pumpRef.current = () => {
    if (inFlightRef.current || endCallRef.current) return;
    const pending = pendingRef.current;
    const endRequest = endRequestRef.current;
    if (!pending) {
      if (endRequest) void finishEnd(endRequest.reason, endRequest.flush);
      return;
    }
    const api = apiRef.current;
    const sessionId = sessionRef.current;
    const endingFlush = phaseRef.current === "ending" && endRequest?.flush === true;
    if (!api || !sessionId || (phaseRef.current !== "active" && !endingFlush)) return;

    const now = performance.now();
    const lastStarted = lastFrameStartRef.current;
    const wait = lastStarted === null ? 0 : MIN_FRAME_INTERVAL_MS - (now - lastStarted);
    if (wait > 0) {
      if (pumpTimerRef.current === null) {
        pumpTimerRef.current = globalThis.setTimeout(() => {
          pumpTimerRef.current = null;
          pumpRef.current();
        }, wait);
      }
      return;
    }
    if (pumpTimerRef.current !== null) {
      globalThis.clearTimeout(pumpTimerRef.current);
      pumpTimerRef.current = null;
    }

    pendingRef.current = null;
    inFlightRef.current = true;
    lastFrameStartRef.current = now;
    const sequence = ++sequenceRef.current;
    const generation = generationRef.current;
    const controller = new AbortController();
    frameAbortRef.current = controller;
    if (endingFlush) armTerminalFrameDeadline(controller, generation);
    const sentAt = performance.now();
    dispatchedRef.current += 1;
    if (mounted.current) setDispatched(dispatchedRef.current);
    void api.sendLiveFollowFrame(sessionId, sequence, pending, controller.signal).then((next) => {
      if (generation !== generationRef.current) return;
      inputLeaseMsRef.current = next.timing.inputLeaseMs;
      if (mounted.current) {
        setReceipt(next);
        setRttMs(performance.now() - sentAt);
      }
      if (runRef.current) {
        runRef.current.dispatched = next.stats.dispatchedFrames;
        runRef.current.coalesced = next.stats.coalescedFrames;
      }
      if (next.state !== "active") {
        if (mounted.current) setMessage(next.reason ?? `Live follow ended with state ${next.state}.`);
        generationRef.current += 1;
        closeLocal(next.state === "stopped" ? "stop" : "fault");
      }
    }).catch((error) => {
      if (generation !== generationRef.current) return;
      pendingRef.current = null;
      if (!controller.signal.aborted && !endRequestRef.current) {
        if (mounted.current) setMessage(messageOf(error));
        requestEnd("cancel", false);
      }
    }).finally(() => {
      if (generation !== generationRef.current) return;
      if (frameAbortRef.current === controller) {
        frameAbortRef.current = null;
        if (terminalFrameAbortTimerRef.current !== null) {
          globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
          terminalFrameAbortTimerRef.current = null;
        }
      }
      inFlightRef.current = false;
      pumpRef.current();
    });
  };

  const updateMeasuredState = useCallback((next: SimpleArmState) => {
    const now = performance.now();
    const measured = measuredTargets(next);
    if (measured) {
      const derived: ArmLiveFollowTargets = { joint_2: 0, joint_3: 0 };
      for (const joint of LIVE_JOINTS) {
        const previous = sampleRef.current[joint];
        const current = measured[joint];
        // State polling can observe the same cached controller packet several
        // times. Preserve the last DISTINCT sample time so the next angle
        // change is divided across its real browser-observed interval instead
        // of an arbitrary final 50 ms poll.
        if (previous && current !== previous.degrees) {
          const elapsedSeconds = (now - previous.at) / 1000;
          if (elapsedSeconds > 0) derived[joint] = Math.abs(current - previous.degrees) / elapsedSeconds;
          sampleRef.current[joint] = { at: now, degrees: current };
        } else if (!previous) {
          sampleRef.current[joint] = { at: now, degrees: current };
        }
      }
      if (mounted.current) setVelocity(derived);
      const run = runRef.current;
      if (run) {
        run.peak.joint_2 = Math.max(run.peak.joint_2, derived.joint_2);
        run.peak.joint_3 = Math.max(run.peak.joint_3, derived.joint_3);
      }
      const desired = targetRef.current;
      if (runRef.current && desired) {
        runRef.current.maxError = Math.max(
          runRef.current.maxError,
          Math.abs(desired.joint_2 - measured.joint_2),
          Math.abs(desired.joint_3 - measured.joint_3),
        );
      }
      if (!targetRef.current && phaseRef.current === "idle") {
        targetRef.current = measured;
        ghostRef.current = modelPose(next);
        if (mounted.current) {
          setTarget(measured);
          setGhost(ghostRef.current);
        }
      }
    }
    if (mounted.current) setState(next);
  }, []);

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      if (statePollInFlightRef.current) return;
      statePollInFlightRef.current = true;
      try {
        const api = await ensureApi();
        const next = await api.state();
        if (!cancelled) updateMeasuredState(next);
      } catch (error) {
        if (!cancelled && phaseRef.current === "idle") setMessage(messageOf(error));
      } finally {
        statePollInFlightRef.current = false;
      }
    };
    void poll();
    const timer = globalThis.setInterval(() => void poll(), phase === "active" ? ACTIVE_POLL_MS : POLL_MS);
    return () => {
      cancelled = true;
      globalThis.clearInterval(timer);
    };
  }, [ensureApi, phase, updateMeasuredState]);

  useEffect(() => {
    const busy = phase !== "idle";
    onControlBusyChange?.(busy);
    return () => onControlBusyChange?.(false);
  }, [onControlBusyChange, phase]);

  /* Motion frames remain single-flight so targets cannot arrive out of order.
   * The deliberately short Pi lease uses a separate one-in-flight heartbeat:
   * one slow motion receipt must never starve the 400 ms deadman renewal. */
  useEffect(() => {
    if (phase !== "active") return;
    let cancelled = false;
    const heartbeat = async () => {
      if (cancelled || heartbeatInFlightRef.current || endCallRef.current || endRequestRef.current) return;
      const sessionId = sessionRef.current;
      if (!sessionId) return;
      const generation = generationRef.current;
      heartbeatInFlightRef.current = true;
      try {
        const api = await ensureApi();
        const next = await api.heartbeatLiveFollow(sessionId);
        if (cancelled || generation !== generationRef.current || sessionRef.current !== sessionId) return;
        inputLeaseMsRef.current = next.timing.inputLeaseMs;
        if (next.state !== "active") {
          if (mounted.current) setMessage(next.reason ?? `Live follow ended with state ${next.state}.`);
          generationRef.current += 1;
          closeLocal(next.state === "stopped" ? "stop" : "fault");
        }
      } catch (error) {
        if (cancelled || generation !== generationRef.current || sessionRef.current !== sessionId) return;
        if (mounted.current) setMessage(`Live heartbeat failed: ${messageOf(error)}`);
        requestEnd("cancel", false);
      } finally {
        if (generation === generationRef.current) heartbeatInFlightRef.current = false;
      }
    };
    const keepAlive = globalThis.setInterval(() => void heartbeat(), POLL_MS);
    return () => {
      cancelled = true;
      globalThis.clearInterval(keepAlive);
    };
  }, [closeLocal, ensureApi, phase, requestEnd]);

  useEffect(() => {
    const onBlur = () => requestEnd("blur", false);
    const onVisibility = () => {
      if (document.visibilityState === "hidden") requestEnd("hidden", false);
    };
    globalThis.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      globalThis.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
    };
  }, [requestEnd]);

  useEffect(() => {
    // StrictMode replays setup after cleanup on the same component instance.
    // Restore UI updates without restoring any cancelled motion authority.
    mounted.current = true;
    return () => {
      mounted.current = false;
      generationRef.current += 1;
      if (pumpTimerRef.current !== null) globalThis.clearTimeout(pumpTimerRef.current);
      if (terminalFrameAbortTimerRef.current !== null) globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
      if (terminalWatchdogRef.current !== null) globalThis.clearTimeout(terminalWatchdogRef.current);
      if (startDeadlineTimerRef.current !== null) globalThis.clearTimeout(startDeadlineTimerRef.current);
      if (startRecoveryTimerRef.current !== null) globalThis.clearTimeout(startRecoveryTimerRef.current);
      if (stopWatchdogRef.current !== null) globalThis.clearTimeout(stopWatchdogRef.current);
      const startAttempt = startAttemptRef.current;
      if (startAttempt && !startAttempt.cancelStarted) {
        startAttempt.cancelStarted = true;
        startAttempt.controller.abort();
        void startAttempt.api.cancelLiveFollowStart(startAttempt.id).catch(() => undefined);
      }
      startAttempt?.cancelController?.abort();
      stopAbortRef.current?.abort();
      frameAbortRef.current?.abort();
      endAbortRef.current?.abort();
      const sessionId = sessionRef.current;
      if (sessionId && apiRef.current && !endCallRef.current) {
        void apiRef.current.endLiveFollow(sessionId, false).catch(() => undefined);
      }
    };
  }, []);

  const readinessMessage = readiness(state);
  const simBlocked = backendId !== "real";
  const canStart = !simBlocked && phase === "idle" && !readinessMessage;
  const showBaseReferenceError = !simBlocked && baseReferenceUnknown(state);

  const setExactSetting = (joint: ArmLiveFollowJointId, key: SettingKey, value: number) => {
    const bounded = boundedInteger(key, String(value), settings[joint][key]);
    setSettings((current) => ({
      ...current,
      [joint]: { ...current[joint], [key]: bounded },
    }));
    setSettingDrafts((current) => ({
      ...current,
      [joint]: { ...current[joint], [key]: String(bounded) },
    }));
  };

  const setSettingDraft = (joint: ArmLiveFollowJointId, key: SettingKey, value: string) => {
    setSettingDrafts((current) => ({
      ...current,
      [joint]: { ...current[joint], [key]: value },
    }));
  };

  const commitSettingDraft = (joint: ArmLiveFollowJointId, key: SettingKey) => {
    setExactSetting(joint, key, boundedInteger(key, settingDrafts[joint][key], settings[joint][key]));
  };

  const begin = async () => {
    if (!canStart) return;
    // Readiness comes from a state request made through this exact API. Never
    // enter `starting` and take shell ownership while waiting on a new token.
    const api = apiRef.current;
    if (!api) {
      setMessage("The local action session is not ready. Refresh telemetry before enabling live control.");
      return;
    }
    const requestedSettings = normalizedSettings(settings, settingDrafts);
    setSettings(requestedSettings);
    setSettingDrafts(draftsFor(requestedSettings));
    const generation = ++generationRef.current;
    setCurrentPhase("starting");
    setMessage(null);
    setReceipt(null);
    setRttMs(null);
    sequenceRef.current = 0;
    lastFrameStartRef.current = null;
    inputLeaseMsRef.current = DEFAULT_INPUT_LEASE_MS;
    if (pumpTimerRef.current !== null) {
      globalThis.clearTimeout(pumpTimerRef.current);
      pumpTimerRef.current = null;
    }
    if (terminalFrameAbortTimerRef.current !== null) {
      globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
      terminalFrameAbortTimerRef.current = null;
    }
    if (terminalWatchdogRef.current !== null) {
      globalThis.clearTimeout(terminalWatchdogRef.current);
      terminalWatchdogRef.current = null;
    }
    if (startDeadlineTimerRef.current !== null) {
      globalThis.clearTimeout(startDeadlineTimerRef.current);
      startDeadlineTimerRef.current = null;
    }
    if (startRecoveryTimerRef.current !== null) {
      globalThis.clearTimeout(startRecoveryTimerRef.current);
      startRecoveryTimerRef.current = null;
    }
    startAttemptRef.current?.controller.abort();
    startAttemptRef.current?.cancelController?.abort();
    startAttemptRef.current = null;
    frameAbortRef.current?.abort();
    frameAbortRef.current = null;
    endAbortRef.current?.abort();
    endAbortRef.current = null;
    pendingRef.current = null;
    endRequestRef.current = null;
    endCallRef.current = false;
    inFlightRef.current = false;
    heartbeatInFlightRef.current = false;
    dispatchedRef.current = 0;
    coalescedRef.current = 0;
    sampleRef.current = { joint_2: null, joint_3: null };
    setVelocity({ joint_2: 0, joint_3: 0 });
    setDispatched(0);
    setCoalesced(0);
    let startAttempt: LiveStartAttempt | null = null;
    try {
      if (generation !== generationRef.current || !mounted.current) return;
      const controller = new AbortController();
      const attempt: LiveStartAttempt = {
        id: createLiveFollowStartAttemptId(),
        generation,
        startedAt: performance.now(),
        api,
        controller,
        cancelController: null,
        cancelStarted: false,
      };
      startAttempt = attempt;
      startAttemptRef.current = attempt;
      startDeadlineTimerRef.current = globalThis.setTimeout(() => {
        startDeadlineTimerRef.current = null;
        if (startAttemptRef.current !== attempt || generation !== generationRef.current) return;
        cancelStartAttempt(attempt, `Live start timed out after ${LIVE_FOLLOW_START_TIMEOUT_MS} ms. Cancelling the exact attempt without retrying.`);
      }, LIVE_FOLLOW_START_TIMEOUT_MS);

      const next = await api.startLiveFollow(requestedSettings, {
        startAttemptId: attempt.id,
        startTimeoutMs: LIVE_FOLLOW_START_TIMEOUT_MS,
        signal: controller.signal,
      });
      if (startAttemptRef.current !== attempt || attempt.cancelStarted || generation !== generationRef.current || !mounted.current) {
        if (!attempt.cancelStarted) {
          attempt.cancelStarted = true;
          void api.cancelLiveFollowStart(attempt.id).catch(() => undefined);
        }
        return;
      }
      if (startDeadlineTimerRef.current !== null) {
        globalThis.clearTimeout(startDeadlineTimerRef.current);
        startDeadlineTimerRef.current = null;
      }
      startAttemptRef.current = null;
      if (next.state !== "active") throw new Error(next.reason ?? `Live follow returned ${next.state}.`);
      sessionRef.current = next.sessionId;
      inputLeaseMsRef.current = next.timing.inputLeaseMs;
      setReceipt(next);
      setSettings(cloneSettings(next.settings));
      setSettingDrafts(draftsFor(next.settings));
      // This pose and the live-follow lease were captured atomically by the Pi;
      // a browser state poll may already be one cycle older.
      const initial = { ...next.commanded };
      targetRef.current = initial;
      setTarget(initial);
      setSessionOrigin({ ...next.commanded });
      ghostRef.current = {
        ...modelPose(state),
        joint_2: toModel("joint_2", initial.joint_2),
        joint_3: toModel("joint_3", initial.joint_3),
      };
      setGhost(ghostRef.current);
      runRef.current = {
        id: nextRunId.current++,
        startedAt: performance.now(),
        settings: cloneSettings(next.settings),
        peak: { joint_2: 0, joint_3: 0 },
        maxError: 0,
        dispatched: next.stats.dispatchedFrames,
        coalesced: next.stats.coalescedFrames,
      };
      setCurrentPhase(endRequestRef.current ? "ending" : "active");
      if (endRequestRef.current) pumpRef.current();
    } catch (error) {
      if (startAttempt?.cancelStarted) return;
      if (generation === generationRef.current && mounted.current) {
        if (startAttemptRef.current === startAttempt && startAttempt) {
          cancelStartAttempt(startAttempt, `Live start was not confirmed: ${messageOf(error)} Cancelling the exact attempt without retrying.`);
        } else {
          setMessage(messageOf(error));
          closeLocal("fault");
        }
      }
    }
  };

  const handlePose = (patch: Partial<Record<ArmJointId, number>>) => {
    if (phaseRef.current !== "active" || endRequestRef.current) return;
    setMessage(null);
    const nextGhost = { ...ghostRef.current, ...patch };
    ghostRef.current = nextGhost;
    setGhost(nextGhost);
    const nextTarget = {
      joint_2: fromModel("joint_2", nextGhost.joint_2),
      joint_3: fromModel("joint_3", nextGhost.joint_3),
    };
    targetRef.current = nextTarget;
    setTarget(nextTarget);
    if (pendingRef.current) {
      coalescedRef.current += 1;
      setCoalesced(coalescedRef.current);
    }
    pendingRef.current = nextTarget;
    pumpRef.current();
  };

  const handleInteractionEnd = (reason: "release" | "cancel") => {
    if (endRequestRef.current || phaseRef.current !== "active") return;
    if (reason === "cancel") {
      requestEnd("cancel", false);
      return;
    }
    // Pointer release is a dead-man boundary for new input, not a session end.
    // The newest absolute target may finish while the independent heartbeat
    // keeps the lease alive and telemetry shows the measured arrival.
    pumpRef.current();
    setMessage("Newest target flushed. Session stays LIVE so measured arrival remains visible.");
  };

  const stop = async () => {
    if (stopCallRef.current) return;
    const stopStartedAt = performance.now();
    const supersededSession = phaseRef.current === "active" || phaseRef.current === "ending";
    const startAttempt = startAttemptRef.current;
    if (startAttempt && !startAttempt.cancelStarted) {
      cancelStartAttempt(startAttempt, "STOP cancelled the in-flight live-follow start attempt.");
    }
    const stopGeneration = ++generationRef.current;
    pendingRef.current = null;
    endRequestRef.current = null;
    sessionRef.current = null;
    endCallRef.current = false;
    inFlightRef.current = false;
    heartbeatInFlightRef.current = false;
    if (pumpTimerRef.current !== null) {
      globalThis.clearTimeout(pumpTimerRef.current);
      pumpTimerRef.current = null;
    }
    if (terminalFrameAbortTimerRef.current !== null) {
      globalThis.clearTimeout(terminalFrameAbortTimerRef.current);
      terminalFrameAbortTimerRef.current = null;
    }
    if (terminalWatchdogRef.current !== null) {
      globalThis.clearTimeout(terminalWatchdogRef.current);
      terminalWatchdogRef.current = null;
    }
    frameAbortRef.current?.abort();
    frameAbortRef.current = null;
    endAbortRef.current?.abort();
    endAbortRef.current = null;
    setReceipt(null);
    setSessionOrigin(null);
    setCurrentPhase("ending");
    recordRun("stop");
    stopCallRef.current = true;
    const controller = new AbortController();
    stopAbortRef.current = controller;
    const leaseMs = Math.max(DEFAULT_INPUT_LEASE_MS, Math.ceil(inputLeaseMsRef.current));
    const leaseSafeAt = stopStartedAt + leaseMs + TERMINAL_RECOVERY_MARGIN_MS;
    const supersededSessionSafeAt = supersededSession
      ? stopStartedAt + START_UNCONFIRMED_RECOVERY_MS
      : 0;
    const delayedStartSafeAt = startAttempt
      ? startAttempt.startedAt + START_UNCONFIRMED_RECOVERY_MS
      : 0;
    const watchdogDelay = Math.max(
      0,
      Math.max(leaseSafeAt, supersededSessionSafeAt, delayedStartSafeAt) - performance.now(),
    );
    stopWatchdogRef.current = globalThis.setTimeout(() => {
      stopWatchdogRef.current = null;
      if (stopGeneration !== generationRef.current || stopAbortRef.current !== controller) return;
      generationRef.current += 1;
      controller.abort();
      stopCallRef.current = false;
      if (mounted.current) {
        setMessage("STOP confirmation did not arrive. Local controls recovered only after the safety lease window; the STOP request was not retried.");
      }
      closeLocal("stop");
    }, watchdogDelay);

    let confirmed = false;
    try {
      const api = await ensureApi();
      if (stopGeneration !== generationRef.current || stopAbortRef.current !== controller) return;
      const next = await api.stop(controller.signal);
      if (stopGeneration !== generationRef.current || stopAbortRef.current !== controller) return;
      updateMeasuredState(next);
      if (mounted.current) setMessage("STOP latched. Live follow ended.");
      confirmed = true;
    } catch (error) {
      if (stopGeneration === generationRef.current && stopAbortRef.current === controller && mounted.current) {
        setMessage(`STOP request was not confirmed: ${messageOf(error)} Waiting for the safety lease window; the request will not be retried.`);
      }
    } finally {
      if (stopGeneration !== generationRef.current || stopAbortRef.current !== controller) return;
      if (confirmed) {
        stopCallRef.current = false;
        closeLocal("stop");
      }
    }
  };

  const joints = useMemo(() => jointsOf(state), [state]);
  const measured = measuredTargets(state);
  const measuredModel = modelPose(state);
  const limitFrom = (joint: ArmJointId) => {
    const current = joints[joint];
    const minimum = current?.reachMin ?? current?.minDegrees ?? -90;
    const maximum = current?.reachMax ?? current?.maxDegrees ?? 90;
    const origin = joint === "joint_2" ? sessionOrigin?.joint_2 : joint === "joint_3" ? sessionOrigin?.joint_3 : null;
    const span = joint === "joint_2" || joint === "joint_3" ? receipt?.settings[joint].maxDeltaDegrees : null;
    return modelLimit(
      joint,
      finite(origin) && finite(span) ? Math.max(minimum, origin - span) : minimum,
      finite(origin) && finite(span) ? Math.min(maximum, origin + span) : maximum,
    );
  };
  const limits: Record<ArmJointId, JointLimit> = {
    joint_1: limitFrom("joint_1"),
    joint_2: limitFrom("joint_2"),
    joint_3: limitFrom("joint_3"),
  };
  const errors = measured && target ? {
    joint_2: Math.abs(target.joint_2 - measured.joint_2),
    joint_3: Math.abs(target.joint_3 - measured.joint_3),
  } : null;
  const moving = {
    joint_2: joints.joint_2?.moving === true || velocity.joint_2 >= 1,
    joint_3: joints.joint_3?.moving === true || velocity.joint_3 >= 1,
  };
  const blockedCopy = simBlocked
    ? "Speed Lab is REAL-only. Simulated timing cannot measure physical servo speed."
    : readinessMessage;

  return (
    <section className="arm-live-follow" aria-labelledby="arm-live-follow-title">
      <header className="arm-live-follow-header">
        <div>
          <span className="arm-live-follow-kicker">REAL · SPEED LAB</span>
          <h1 id="arm-live-follow-title">Shoulder + Elbow live follow</h1>
          <p>Drag the shoulder or tool tip. The cyan ghost follows your pointer locally; the solid arm comes only from fresh telemetry.</p>
        </div>
        <button
          type="button"
          className="arm-live-follow-stop"
          disabled={simBlocked}
          title={simBlocked ? "STOP is available here only when REAL is selected." : undefined}
          onClick={() => void stop()}
        >STOP</button>
      </header>

      <section className="arm-live-follow-settings" aria-labelledby="arm-live-follow-settings-title">
        <header className="arm-live-follow-settings-header">
          <div>
            <h2 id="arm-live-follow-settings-title">Session tuning</h2>
            <p id="arm-live-follow-settings-help">Temporary raw ST3215 settings for Shoulder and Elbow. They lock when the session starts and never rewrite calibration.</p>
          </div>
          <div className="arm-live-follow-session-actions">
            <span>{phase === "idle" ? "Editable before Enable" : "Settings locked for this session"}</span>
            {phase === "idle" ? (
              <button type="button" className="arm-live-follow-enable" disabled={!canStart} onClick={() => void begin()}>
                Enable live control
              </button>
            ) : (
              <button type="button" className="arm-live-follow-end" onClick={() => requestEnd("manual", true)}>
                End session
              </button>
            )}
          </div>
        </header>

        <fieldset className="arm-live-follow-settings-grid" disabled={phase !== "idle" || simBlocked} aria-describedby="arm-live-follow-settings-help arm-live-follow-travel-help">
          <legend className="arm-live-follow-sr-only">Per-joint live-follow settings</legend>
          {LIVE_JOINTS.map((joint) => {
            const label = jointLabel(joint);
            const row = settings[joint];
            const drafts = settingDrafts[joint];
            const speedId = `arm-live-follow-${joint}-speed`;
            const accelId = `arm-live-follow-${joint}-accel`;
            const travelId = `arm-live-follow-${joint}-travel`;
            return (
              <div className="arm-live-follow-setting-row" key={joint}>
                <div className="arm-live-follow-setting-joint">
                  <strong>{label}</strong>
                  <span>{finite(joints[joint]?.servoId) ? `ID ${joints[joint]?.servoId}` : "ID n/a"}</span>
                </div>

                <div className="arm-live-follow-setting is-speed">
                  <label htmlFor={speedId}>Speed <span>1 to 2400 raw</span></label>
                  <div className="arm-live-follow-setting-line">
                    <input
                      type="range"
                      min={SETTING_LIMITS.speed.min}
                      max={SETTING_LIMITS.speed.max}
                      step={1}
                      value={row.speed}
                      aria-label={`${label} speed slider, raw ST3215 units`}
                      aria-valuetext={`${row.speed} raw ST3215 speed`}
                      onChange={(event) => setExactSetting(joint, "speed", Number(event.currentTarget.value))}
                    />
                    <input
                      id={speedId}
                      name={`${joint}-speed`}
                      type="number"
                      inputMode="numeric"
                      min={SETTING_LIMITS.speed.min}
                      max={SETTING_LIMITS.speed.max}
                      step={1}
                      value={drafts.speed}
                      aria-label={`${label} speed, raw ST3215 units`}
                      onChange={(event) => setSettingDraft(joint, "speed", event.currentTarget.value)}
                      onBlur={() => commitSettingDraft(joint, "speed")}
                      onKeyDown={(event) => {
                        if (event.key === "Enter") {
                          event.preventDefault();
                          commitSettingDraft(joint, "speed");
                          event.currentTarget.blur();
                        }
                      }}
                    />
                  </div>
                  <div className="arm-live-follow-presets" role="group" aria-label={`${label} speed presets`}>
                    {SPEED_PRESETS.map((preset) => (
                      <button
                        key={preset}
                        type="button"
                        aria-label={`${label} speed preset ${preset}`}
                        aria-pressed={row.speed === preset}
                        onClick={() => setExactSetting(joint, "speed", preset)}
                      >{preset}</button>
                    ))}
                  </div>
                </div>

                <div className="arm-live-follow-setting">
                  <label htmlFor={accelId}>Acceleration <span>1 to 50 raw · servo limit</span></label>
                  <div className="arm-live-follow-setting-line">
                    <input
                      type="range"
                      min={SETTING_LIMITS.accel.min}
                      max={SETTING_LIMITS.accel.max}
                      step={1}
                      value={row.accel}
                      aria-label={`${label} acceleration slider, raw ST3215 units`}
                      aria-valuetext={`${row.accel} raw ST3215 acceleration`}
                      onChange={(event) => setExactSetting(joint, "accel", Number(event.currentTarget.value))}
                    />
                    <input
                      id={accelId}
                      name={`${joint}-acceleration`}
                      type="number"
                      inputMode="numeric"
                      min={SETTING_LIMITS.accel.min}
                      max={SETTING_LIMITS.accel.max}
                      step={1}
                      value={drafts.accel}
                      aria-label={`${label} acceleration, raw ST3215 units`}
                      onChange={(event) => setSettingDraft(joint, "accel", event.currentTarget.value)}
                      onBlur={() => commitSettingDraft(joint, "accel")}
                      onKeyDown={(event) => {
                        if (event.key === "Enter") {
                          event.preventDefault();
                          commitSettingDraft(joint, "accel");
                          event.currentTarget.blur();
                        }
                      }}
                    />
                  </div>
                </div>

                <div className="arm-live-follow-setting is-travel">
                  <label htmlFor={travelId}>Travel <span>±1° to ±90°</span></label>
                  <div className="arm-live-follow-setting-line">
                    <input
                      type="range"
                      min={SETTING_LIMITS.maxDeltaDegrees.min}
                      max={SETTING_LIMITS.maxDeltaDegrees.max}
                      step={1}
                      value={row.maxDeltaDegrees}
                      aria-label={`${label} session travel slider, degrees from start`}
                      aria-valuetext={`plus or minus ${row.maxDeltaDegrees} degrees from session start`}
                      onChange={(event) => setExactSetting(joint, "maxDeltaDegrees", Number(event.currentTarget.value))}
                    />
                    <input
                      id={travelId}
                      name={`${joint}-travel`}
                      type="number"
                      inputMode="numeric"
                      min={SETTING_LIMITS.maxDeltaDegrees.min}
                      max={SETTING_LIMITS.maxDeltaDegrees.max}
                      step={1}
                      value={drafts.maxDeltaDegrees}
                      aria-label={`${label} session travel, degrees from start`}
                      onChange={(event) => setSettingDraft(joint, "maxDeltaDegrees", event.currentTarget.value)}
                      onBlur={() => commitSettingDraft(joint, "maxDeltaDegrees")}
                      onKeyDown={(event) => {
                        if (event.key === "Enter") {
                          event.preventDefault();
                          commitSettingDraft(joint, "maxDeltaDegrees");
                          event.currentTarget.blur();
                        }
                      }}
                    />
                  </div>
                  <div className="arm-live-follow-presets" role="group" aria-label={`${label} travel presets`}>
                    {TRAVEL_PRESETS.map((preset) => (
                      <button
                        key={preset}
                        type="button"
                        title={preset === 90 ? "Maximum Speed Lab software envelope; not a safe or full physical range." : undefined}
                        aria-label={`${label} travel preset plus or minus ${preset} degrees${preset === 90 ? ", maximum software envelope" : ""}`}
                        aria-pressed={row.maxDeltaDegrees === preset}
                        onClick={() => setExactSetting(joint, "maxDeltaDegrees", preset)}
                      >±{preset}°</button>
                    ))}
                  </div>
                </div>
              </div>
            );
          })}
        </fieldset>

        <p className="arm-live-follow-settings-note" id="arm-live-follow-travel-help">
          First physical proof: use 400 speed, 10 acceleration, and ±30° or less on both joints. Then change one setting on one joint at a time. The installed ST3215 servos store at most 50 acceleration; larger values are not offered. ±90° is only the Speed Lab software ceiling; calibrated reachable limits and cumulative floor guard can reduce the actual envelope. This is not obstacle, cable, self-contact, or overshoot sensing.
        </p>
      </section>

      {showBaseReferenceError ? (
        <div className="arm-live-follow-base-reference" role="alert">
          <strong>BASE REFERENCE UNKNOWN</strong>
          <p>
            No live Base angle is available; this screen does not show or assume 0°. Live remains
            Shoulder + Elbow only. In <b>Control</b>, restore Base telemetry if needed, keep it free,
            place it exactly on the marked physical zero, then choose <b>Set zero here</b>.
          </p>
        </div>
      ) : null}
      {blockedCopy ? <p className="arm-live-follow-blocked">{blockedCopy}</p> : null}
      <p className="arm-live-follow-status" role="status" aria-live="polite">
        {message ?? (phase === "starting" ? "Opening bounded live-follow session…" : phase === "active" ? "LIVE: release flushes the newest target; End session when the run is complete." : phase === "ending" ? "Ending live-follow session…" : "Tune each joint, then explicitly enable one session.")}
      </p>

      <div className="arm-live-follow-workspace">
        <div className="arm-live-follow-canvas">
          <Arm2D
            pose={ghost}
            measuredPose={measuredModel}
            planned
            disabled={phase !== "active"}
            limits={limits}
            geometry={DEFAULT_GEOMETRY}
            floorMm={state?.floorGuard?.enabled ? state.floorGuard.floorMm : null}
            onPose={handlePose}
            onRelease={() => undefined}
            onInteractionEnd={handleInteractionEnd}
            showDials={false}
            emptyReadout="n/a"
            interactionCopy="Drag streams only Shoulder and Elbow. Release flushes the newest target and keeps this run LIVE for measured arrival. Pointer cancel discards pending input and ends the session."
            readouts={{ joint_1: null, joint_2: target?.joint_2 ?? null, joint_3: target?.joint_3 ?? null }}
          />
        </div>

        <aside className="arm-live-follow-inspector" aria-label="Live follow telemetry">
          <div className="arm-live-follow-session-line">
            <span className={`arm-live-follow-dot is-${phase}`} aria-hidden="true" />
            <strong>{phase === "active" ? "LIVE" : phase.toUpperCase()}</strong>
            <code>{receipt?.sessionId ? receipt.sessionId.slice(-8) : "no session"}</code>
          </div>

          <table className="arm-live-follow-joints">
            <caption>Fresh measured joint feedback</caption>
            <thead><tr><th>Joint</th><th>Target</th><th>Measured</th><th>Error</th><th>Moving</th><th>Approx.</th></tr></thead>
            <tbody>
              {(["joint_2", "joint_3"] as const).map((id) => (
                <tr key={id}>
                  <th scope="row">{id === "joint_2" ? "Shoulder" : "Elbow"}</th>
                  <td>{degrees(target?.[id])}</td>
                  <td>{degrees(measured?.[id])}</td>
                  <td>{degrees(errors?.[id])}</td>
                  <td>{moving[id] ? "yes" : "no"}</td>
                  <td>{speedText(velocity[id])}</td>
                </tr>
              ))}
            </tbody>
          </table>

          <dl className="arm-live-follow-metrics">
            <div><dt>Round trip</dt><dd>{rttMs == null ? "n/a" : `${rttMs.toFixed(0)} ms`}</dd></div>
            <div><dt>HTTP frames</dt><dd>{dispatched}</dd></div>
            <div><dt>Client overwrite</dt><dd>{coalesced}</dd></div>
            <div><dt>Pi dispatched</dt><dd>{receipt?.stats.dispatchedFrames ?? 0}</dd></div>
            <div><dt>Pi coalesced</dt><dd>{receipt?.stats.coalescedFrames ?? 0}</dd></div>
            <div><dt>Backend clamp</dt><dd>{receipt?.stats.clampedFrames ?? 0}</dd></div>
          </dl>

          <p className="arm-live-follow-caveat">
            Speed and acceleration are temporary raw ST3215 settings, not degrees/second or degrees/second². “Approx.” comes from timestamped active-session angle changes and is useful for comparison, not maximum-speed proof or precision metrology.
          </p>

          <section className="arm-live-follow-history" aria-labelledby="arm-live-follow-history-title">
            <h2 id="arm-live-follow-history-title">Last runs</h2>
            {history.length ? (
              <table>
                <thead><tr><th>Speed S / E</th><th>Accel S / E</th><th>Travel S / E</th><th>Time</th><th>Approx. peak S / E</th><th>Max err.</th><th>Pi disp. / coal.</th></tr></thead>
                <tbody>{history.map((run) => (
                  <tr key={run.id} title={`Ended: ${run.reason}`}>
                    <td>{run.settings.joint_2.speed} / {run.settings.joint_3.speed}</td>
                    <td>{run.settings.joint_2.accel} / {run.settings.joint_3.accel}</td>
                    <td>±{run.settings.joint_2.maxDeltaDegrees}° / ±{run.settings.joint_3.maxDeltaDegrees}°</td>
                    <td>{durationText(run.durationMs)}</td>
                    <td>{run.peak.joint_2.toFixed(0)} / {run.peak.joint_3.toFixed(0)}</td>
                    <td>{run.maxError.toFixed(1)}°</td>
                    <td>{run.dispatched} / {run.coalesced}</td>
                  </tr>
                ))}</tbody>
              </table>
            ) : <p>No completed run yet.</p>}
          </section>
        </aside>
      </div>
    </section>
  );
}
