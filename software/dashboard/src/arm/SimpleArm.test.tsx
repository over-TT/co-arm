import { act, fireEvent, render, screen, waitFor, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { describe, expect, it, vi } from "vitest";
import { SimpleArm, fromModel, toModel } from "./SimpleArm";
import { forwardKinematics } from "./armKinematics";
import type { CameraAutofocusResponse, CameraAutofocusResult, CameraObservation, CameraStatus, SimpleArmScanEvidence, SimpleControllerIdentity, SimpleMultiTurnTruth } from "./armTypes";

/** Typed fields commit on Enter, not on every keystroke. */
function type(name: string, value: string) {
  const field = screen.getByRole("spinbutton", { name });
  fireEvent.change(field, { target: { value } });
  fireEvent.keyDown(field, { key: "Enter" });
}

function jsonResponse(body: unknown, status = 200) {
  return new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });
}

function deferred<T>() {
  let resolve!: (value: T | PromiseLike<T>) => void;
  let reject!: (reason?: unknown) => void;
  const promise = new Promise<T>((accept, decline) => {
    resolve = accept;
    reject = decline;
  });
  return { promise, resolve, reject };
}

const FLOOR_SWEEP_REFUSAL = "The independently timed Shoulder/Elbow sweep crosses the floor keep-out plane. Choose another pose.";

function joint(id: string, name: string, servoId: number, raw: number, calibrated = true) {
  return {
    id, name, servoId,
    rawZero: calibrated ? 2048 : null,
    rawMin: calibrated ? 1048 : null,
    rawMax: calibrated ? 3048 : null,
    ratio: 1, direction: 1, speed: 1400, accel: 12,
    online: true,
    rawPosition: raw,
    degrees: calibrated ? (raw - 2048) * 360 / 4096 : null,
    positionTrusted: true,
    minDegrees: calibrated ? -87.9 : null,
    maxDegrees: calibrated ? 87.9 : null,
    rawLow: 1048, rawHigh: 3048,
    calibrated, torque: "off",
    temperatureC: 32, voltageVolts: 12,
  };
}

function harness(overrides: {
  held?: number[];
  connection?: "online" | "faulted" | "disconnected";
  bus?: "online" | "faulted" | "offline";
  busTrouble?: string | null;
  collisionSuspected?: boolean;
  collisionId?: number | null;
  lastScan?: SimpleArmScanEvidence | null;
  allJointsOffline?: boolean;
  baseRaw?: number;
  calibrated?: boolean;
  positionTrusted?: boolean;
  controller?: SimpleControllerIdentity | null;
  multiTurnTruth?: SimpleMultiTurnTruth | null;
  baseOnline?: boolean;
  baseMinDegrees?: number;
  baseMaxDegrees?: number;
  stopped?: boolean;
  baseReferenceRequired?: boolean;
  baseReferenceReason?: "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED" | "BASE_REFERENCE_MARKER_INVALID" | null;
  planWarnings?: string[];
  planResolvedPose?: Record<string, unknown>;
  planPreviewFailure?: { status: number; detail: string };
  sequencePreviewResponse?: (body: { waypoints: Array<{ targets: Record<string, number>; label?: string }> }, attempt: number) => Response | Promise<Response>;
  sequenceOutcome?: string;
  sequenceCompletedCount?: number;
  sequenceFailedWaypointIndex?: number | null;
  sequenceExecuteResponse?: Promise<Response>;
  } = {}) {
  const calls: Array<{ url: string; body: unknown }> = [];
  let sequencePreviewAttempt = 0;
  const base = {
    ...joint("joint_1", "Base", 1, overrides.baseRaw ?? 2300, overrides.calibrated ?? true),
    positionTrusted: overrides.positionTrusted ?? true,
    online: overrides.baseOnline ?? true,
    minDegrees: overrides.baseMinDegrees ?? -87.9,
    maxDegrees: overrides.baseMaxDegrees ?? 87.9,
    ...(overrides.multiTurnTruth === null ? {} : {
      multiTurnTruth: overrides.multiTurnTruth ?? {
        tracking: true,
        valid: true,
        stepMode: false,
        stepOutstanding: false,
        countdownObserved: false,
        resyncNeeded: false,
        resyncCount: 0,
        sampleAgeMs: 12,
      },
    }),
  };
  const liveJoints = [
    base,
    joint("joint_2", "Shoulder", 2, 2048),
    joint("joint_3", "Elbow", 3, 2048),
    joint("joint_4", "Camera", 4, 2048),
  ];
  const state = {
    connection: overrides.connection ?? "online", bus: overrides.bus ?? "online", held: overrides.held ?? [], stopped: overrides.stopped ?? false,
    baseReferenceRequired: overrides.baseReferenceRequired ?? false,
    baseReferenceReason: overrides.baseReferenceReason ?? null,
    busTrouble: overrides.busTrouble ?? null,
    collisionSuspected: overrides.collisionSuspected ?? false,
    collisionId: overrides.collisionId ?? null,
    ...(overrides.lastScan === undefined ? {} : { lastScan: overrides.lastScan }),
    ...(overrides.controller === null ? {} : {
      controller: overrides.controller ?? {
        controllerId: "armhat-test-a",
        bootId: "boot-test-00000001",
        firmwareVersion: "arm-hat-2.3.0",
        protocolVersion: 1,
        multiTurnTruthV3: true,
      },
    }),
    joints: overrides.allJointsOffline
      ? liveJoints.map((candidate) => ({
          ...candidate,
          online: false,
          rawPosition: null,
          degrees: null,
          positionTrusted: false,
          torque: "unknown" as const,
        }))
      : liveJoints,
  };
  const request = vi.fn<typeof fetch>(async (input, init) => {
    const url = String(input);
    if (url === "/api/session") return jsonResponse({ actionToken: "z".repeat(40) });
    if (init?.method === "POST") {
      const body = init.body ? JSON.parse(String(init.body)) : null;
      calls.push({ url, body });
      if (url.endsWith("/plans/preview")) {
        if (overrides.planPreviewFailure) {
          return jsonResponse({ detail: overrides.planPreviewFailure.detail }, overrides.planPreviewFailure.status);
        }
        const measuredPose = Object.fromEntries(state.joints.map((candidate) => [candidate.id, candidate.degrees]));
        return jsonResponse({
          planId: "plan_test_1",
          planDigest: `sha256:${"a".repeat(64)}`,
          expiresAt: "2099-01-01T00:00:00Z",
          expiresInMs: 15_000,
          measuredPose,
          resolvedPose: overrides.planResolvedPose ?? body.targets,
          warnings: overrides.planWarnings ?? [],
          lowestClearanceMm: 42.5,
        });
      }
      if (url.endsWith("/plans/execute")) {
        return jsonResponse({ executed: true, planId: "plan_test_1", resolvedPose: {}, moved: [] });
      }
      if (url.endsWith("/sequences/preview")) {
        sequencePreviewAttempt += 1;
        if (overrides.sequencePreviewResponse) {
          return await overrides.sequencePreviewResponse(body, sequencePreviewAttempt);
        }
        const measuredPose = Object.fromEntries(state.joints.map((candidate) => [candidate.id, candidate.degrees]));
        let startPose = measuredPose;
        const waypoints = body.waypoints.map((waypoint: { targets: Record<string, number> }, index: number) => {
          const resolvedPose = { ...startPose, ...waypoint.targets };
          const row = {
            index,
            startPose,
            resolvedPose,
            warnings: [],
            lowestClearanceMm: 40 - index * 5,
          };
          startPose = resolvedPose;
          return row;
        });
        return jsonResponse({
          sequenceId: "armseq_test_1234",
          sequenceDigest: `sha256:${"d".repeat(64)}`,
          expiresAt: "2099-01-01T00:00:00Z",
          expiresInMs: 120_000,
          previewDurationMs: 12,
          waypointCount: waypoints.length,
          measuredPose,
          waypoints,
          warnings: [],
          lowestClearanceMm: 35,
        });
      }
      if (url.endsWith("/sequences/execute")) {
        if (overrides.sequenceExecuteResponse) return await overrides.sequenceExecuteResponse;
        const completed = overrides.sequenceCompletedCount ?? body.waypoints?.length ?? 2;
        return jsonResponse({
          executed: overrides.sequenceOutcome === undefined || overrides.sequenceOutcome === "completed",
          sequenceId: "armseq_test_1234",
          operationId: "armop_test_1234",
          outcome: overrides.sequenceOutcome ?? "completed",
          waypointCount: 2,
          completedWaypointCount: completed,
          failedWaypointIndex: overrides.sequenceFailedWaypointIndex ?? null,
          waypointResults: Array.from({ length: 2 }, (_, index) => ({
            index,
            outcome: index < completed ? "arrived" : index === overrides.sequenceFailedWaypointIndex ? overrides.sequenceOutcome : "not_started",
            dispatched: index <= (overrides.sequenceFailedWaypointIndex ?? completed - 1),
            arrival: { proved: index < completed },
            moved: ["joint_1"],
            dispatchMs: index === 0 ? 0.4 : 110,
            arrivalMs: 300 + index * 10,
            durationMs: 400 + index * 20,
          })),
          resolvedPose: {},
          finalMeasuredPose: {},
          durationMs: 1200,
        });
      }
      return jsonResponse(state);
    }
    if (url === "/api/arm/simple/state") return jsonResponse(state);
    return jsonResponse({ detail: "not found" }, 404);
  });
  return { request, calls };
}

const CAMERA_STATUS: CameraStatus = {
  simulated: false,
  readOnly: true,
  cameraId: "rpi-camera-0",
  sensorModel: "ov5647",
  identityConfidence: "configured_candidate",
  state: "started",
  available: true,
  latestFrameId: null,
  historySize: 0,
  historyLimit: 8,
  retainedBytes: 0,
  byteLimit: 67_108_864,
  autofocus: {
    capability: "unsupported",
    mode: "fixed",
    state: "fixed",
    range: "unavailable",
  },
};

function cameraObservation(frameId = "camera_frame_1"): CameraObservation {
  return {
    simulated: false,
    readOnly: true,
    source: "picamera2",
    frameId,
    cameraId: "rpi-camera-0",
    sensorModel: "ov5647",
    identityConfidence: "configured_candidate",
    capturedAt: "2026-08-03T20:15:30Z",
    ageSeconds: 0,
    dimensions: { width: 2592, height: 1944 },
    captureProfile: "detail",
    byteCount: 24_576,
    sha256: "a".repeat(64),
    contentSha256: `sha256:${"b".repeat(64)}`,
    stateRevision: 12,
    frameUrl: `/api/camera/frames/${frameId}`,
    autofocus: {
      capability: "unsupported",
      mode: "fixed",
      state: "fixed",
      range: "unavailable",
    },
  };
}

function cameraAutofocusResponse(result: CameraAutofocusResult): CameraAutofocusResponse {
  const unsupported = result === "unsupported";
  return {
    simulated: false,
    physicalArmMotion: false,
    attempted: !unsupported && result !== "unavailable",
    result,
    autofocus: {
      capability: unsupported ? "unsupported" : "supported",
      mode: unsupported ? "fixed" : "continuous",
      state: result === "focused" ? "focused" : result === "not_focused" ? "failed" : unsupported ? "fixed" : "idle",
      range: unsupported ? "unavailable" : "macro",
    },
  };
}

function jpegResponse() {
  return new Response(new Blob(["jpeg"], { type: "image/jpeg" }), {
    status: 200,
    headers: { "Content-Type": "image/jpeg", "Cache-Control": "no-store" },
  });
}

function cameraHarness(options: {
  status?: Partial<CameraStatus>;
  latest?: CameraObservation;
  captured?: CameraObservation;
  autofocus?: CameraAutofocusResponse;
  onCapture?: () => Response | Promise<Response>;
  onAutofocus?: () => Response | Promise<Response>;
  onFrame?: (frameId: string) => Response | Promise<Response>;
} = {}) {
  const arm = harness();
  const captured = options.captured ?? cameraObservation();
  const request = vi.fn<typeof fetch>(async (input, init) => {
    const url = String(input);
    if (url === "/api/camera/status") return jsonResponse({ ...CAMERA_STATUS, ...options.status });
    if (url === "/api/camera/observations/latest") return jsonResponse(options.latest ?? captured);
    if (url.startsWith("/api/camera/captures?profile=")) {
      return options.onCapture ? await options.onCapture() : jsonResponse(captured);
    }
    if (url === "/api/camera/autofocus") {
      return options.onAutofocus
        ? await options.onAutofocus()
        : jsonResponse(options.autofocus ?? cameraAutofocusResponse("unsupported"));
    }
    if (url.startsWith("/api/camera/frames/")) {
      const frameId = url.slice(url.lastIndexOf("/") + 1);
      return options.onFrame ? await options.onFrame(frameId) : jpegResponse();
    }
    return arm.request(input, init);
  });
  return { request, captured };
}

function stubCameraUrls(...urls: string[]) {
  const createObjectURL = vi.fn(() => urls.shift() ?? "blob:camera-frame");
  const revokeObjectURL = vi.fn();
  const TestURL = class extends URL {};
  Object.assign(TestURL, { createObjectURL, revokeObjectURL });
  vi.stubGlobal("URL", TestURL);
  return { createObjectURL, revokeObjectURL };
}

describe("SimpleArm", () => {
  it("keeps state polling single-flight while one Pi response is unresolved", async () => {
    vi.useFakeTimers();
    const firstState = deferred<Response>();
    const stateRequests: string[] = [];
    const request = vi.fn<typeof fetch>(async (input) => {
      const url = String(input);
      if (url === "/api/session") return jsonResponse({ actionToken: "z".repeat(40) });
      if (url === "/api/arm/simple/state") {
        stateRequests.push(url);
        return await firstState.promise;
      }
      if (url === "/api/camera/status") return jsonResponse({ available: false, state: "unavailable", latestFrameId: null });
      return jsonResponse({ detail: "not found" }, 404);
    });

    const view = render(<SimpleArm request={request} />);
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(stateRequests).toHaveLength(1);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_000); });
    expect(stateRequests).toHaveLength(1);

    view.unmount();
    firstState.resolve(jsonResponse({}));
    vi.useRealTimers();
  });

  it("does not let an older untrusted poll erase a successful Base-zero response", async () => {
    vi.useFakeTimers();
    const { request: fixtureRequest } = harness({ positionTrusted: false });
    const untrusted = await (await fixtureRequest("/api/arm/simple/state")).json();
    const trusted = {
      ...untrusted,
      joints: (untrusted as { joints: Array<Record<string, unknown>> }).joints.map((candidate) => (
        candidate.id === "joint_1" ? { ...candidate, positionTrusted: true } : candidate
      )),
    };
    const stalePoll = deferred<Response>();
    let stateCall = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return jsonResponse({ actionToken: "z".repeat(40) });
      if (url === "/api/camera/status") return jsonResponse({ available: false, state: "unavailable", latestFrameId: null });
      if (url === "/api/arm/simple/state") {
        stateCall += 1;
        if (stateCall === 1) return jsonResponse(untrusted);
        if (stateCall === 2) return await stalePoll.promise;
        return jsonResponse(trusted);
      }
      if (init?.method === "POST" && url.endsWith("/joints/joint_1/calibrate")) return jsonResponse(trusted);
      return jsonResponse({ detail: `Unexpected request: ${url}` }, 404);
    });

    const view = render(<SimpleArm request={request} />);
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText(/Base position is unknown/)).toBeInTheDocument();

    await act(async () => { await vi.advanceTimersByTimeAsync(150); });
    expect(stateCall).toBe(2);
    fireEvent.click(screen.getByRole("button", { name: "Set Base zero here" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(within(screen.getByLabelText("Base setup and test")).getByText("Reference ready")).toBeInTheDocument();

    await act(async () => {
      stalePoll.resolve(jsonResponse(untrusted));
      await Promise.resolve();
      await Promise.resolve();
    });
    expect(within(screen.getByLabelText("Base setup and test")).getByText("Reference ready")).toBeInTheDocument();

    view.unmount();
    vi.useRealTimers();
  });

  it("never captures on mount and sends both exact profiles only after their explicit clicks", async () => {
    const user = userEvent.setup();
    const { request } = cameraHarness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const takeSurvey = within(panel).getByRole("button", { name: "Take survey picture" });
    const takeDetail = within(panel).getByRole("button", { name: "Take detail picture" });
    await waitFor(() => expect(takeSurvey).toBeEnabled());
    expect(request.mock.calls.some(([input]) => String(input).startsWith("/api/camera/captures?profile="))).toBe(false);
    expect(request.mock.calls.some(([input]) => String(input) === "/api/camera/autofocus")).toBe(false);

    await user.click(takeSurvey);

    await waitFor(() => expect(request.mock.calls.filter(([input]) => String(input) === "/api/camera/captures?profile=survey")).toHaveLength(1));
    expect(request).toHaveBeenCalledWith(
      "/api/camera/captures?profile=survey",
      expect.objectContaining({ method: "POST" }),
    );
    await user.click(takeDetail);
    await waitFor(() => expect(request.mock.calls.filter(([input]) => String(input) === "/api/camera/captures?profile=detail")).toHaveLength(1));
    expect(request).toHaveBeenCalledWith(
      "/api/camera/captures?profile=detail",
      expect.objectContaining({ method: "POST" }),
    );
    expect(request.mock.calls.some(([input]) => String(input) === "/api/camera/autofocus")).toBe(false);
  });

  it("keeps fixed-lens autofocus enabled for a manual try and makes exactly one autofocus call", async () => {
    const user = userEvent.setup();
    const { request } = cameraHarness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const autofocus = within(panel).getByRole("button", { name: "Try autofocus" });
    await waitFor(() => expect(autofocus).toBeEnabled());
    expect(within(panel).getByText("Take a fast wide survey or a detail-profile still, or ask the camera to try one autofocus cycle. Nothing happens until you press a button, and none of these actions moves the arm.")).toBeInTheDocument();
    expect(within(panel).getByText("Manual camera actions · no stream · no arm movement")).toBeInTheDocument();
    expect(request.mock.calls.some(([input]) => String(input) === "/api/camera/autofocus")).toBe(false);

    await user.click(autofocus);

    await waitFor(() => expect(
      request.mock.calls.filter(([input]) => String(input) === "/api/camera/autofocus"),
    ).toHaveLength(1));
    const autofocusCalls = request.mock.calls.filter(([input]) => String(input) === "/api/camera/autofocus");
    expect(autofocusCalls[0]).toEqual([
      "/api/camera/autofocus",
      {
        method: "POST",
        headers: {
          Accept: "application/json",
          "X-Co-Arm-Token": "z".repeat(40),
        },
      },
    ]);
    expect(request.mock.calls.filter(([input]) => String(input).startsWith("/api/camera/captures?profile="))).toHaveLength(0);
    expect(await within(panel).findByRole("status")).toHaveTextContent(
      "This camera source reports no motorized autofocus support.",
    );
    expect(within(panel).queryByRole("alert")).not.toBeInTheDocument();
  });

  it("marks autofocus busy and disables both camera actions until it settles", async () => {
    const user = userEvent.setup();
    let settleAutofocus!: (response: Response) => void;
    const pendingAutofocus = new Promise<Response>((resolve) => { settleAutofocus = resolve; });
    const { request } = cameraHarness({ onAutofocus: () => pendingAutofocus });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const autofocus = within(panel).getByRole("button", { name: "Try autofocus" });
    const takeSurvey = within(panel).getByRole("button", { name: "Take survey picture" });
    const takeDetail = within(panel).getByRole("button", { name: "Take detail picture" });
    await waitFor(() => expect(autofocus).toBeEnabled());
    await user.click(autofocus);

    const region = within(panel).getByRole("region", { name: "Desk camera" });
    expect(region).toHaveAttribute("aria-busy", "true");
    expect(within(panel).getByText("Trying autofocus")).toBeInTheDocument();
    expect(within(panel).getByRole("button", { name: "Trying autofocus…" })).toBeDisabled();
    expect(takeSurvey).toBeDisabled();
    expect(takeDetail).toBeDisabled();

    settleAutofocus(jsonResponse(cameraAutofocusResponse("timed_out")));
    expect(await within(panel).findByRole("status")).toHaveTextContent(
      "Autofocus timed out before the camera reported a result. Try again.",
    );
    expect(region).toHaveAttribute("aria-busy", "false");
  });

  it.each([
    ["focused", "Camera reported focus. Take a picture to check whether the subject looks sharper."],
    ["not_focused", "The camera could not lock focus. Try again with more light or more detail in the frame."],
    ["timed_out", "Autofocus timed out before the camera reported a result. Try again."],
    ["unavailable", "Autofocus could not run. The autofocus controls or camera may be unavailable. Check camera status before trying again."],
  ] as const)("announces the %s autofocus outcome as polite status", async (result, expected) => {
    const user = userEvent.setup();
    const { request } = cameraHarness({ autofocus: cameraAutofocusResponse(result) });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const autofocus = within(panel).getByRole("button", { name: "Try autofocus" });
    await waitFor(() => expect(autofocus).toBeEnabled());
    await user.click(autofocus);

    expect(await within(panel).findByRole("status")).toHaveTextContent(expected);
    expect(within(panel).queryByRole("alert")).not.toBeInTheDocument();
  });

  it("shows an autofocus transport failure without replacing the existing JPEG or same-frame focus evidence", async () => {
    const user = userEvent.setup();
    const { createObjectURL, revokeObjectURL } = stubCameraUrls("blob:camera-existing");
    const latest = cameraObservation("camera_existing");
    const { request } = cameraHarness({
      status: { latestFrameId: latest.frameId },
      latest,
      onAutofocus: () => jsonResponse({ detail: "focus proxy offline" }, 503),
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const image = await within(panel).findByRole("img");
    expect(image).toHaveAttribute("src", "blob:camera-existing");
    expect(within(panel).getByText("motorized autofocus unavailable")).toBeInTheDocument();

    await user.click(within(panel).getByRole("button", { name: "Try autofocus" }));

    expect(await within(panel).findByRole("alert")).toHaveTextContent(
      "Autofocus test failed: focus proxy offline",
    );
    expect(within(panel).getByRole("img")).toHaveAttribute("src", "blob:camera-existing");
    expect(within(panel).getByText("motorized autofocus unavailable")).toBeInTheDocument();
    expect(createObjectURL).toHaveBeenCalledTimes(1);
    expect(revokeObjectURL).not.toHaveBeenCalled();
  });

  it("keeps manual capture disabled when the camera is unavailable", async () => {
    const user = userEvent.setup();
    const { request } = cameraHarness({ status: { available: false, state: "not_configured" } });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    expect(await within(panel).findByText("Camera unavailable.")).toBeInTheDocument();
    expect(within(panel).getByRole("button", { name: "Take survey picture" })).toBeDisabled();
    expect(within(panel).getByRole("button", { name: "Take detail picture" })).toBeDisabled();
    expect(within(panel).getByRole("button", { name: "Try autofocus" })).toBeDisabled();
    expect(within(panel).getByText(/not configured/i)).toBeInTheDocument();
  });

  it("serializes both profiles and labels a pending detail capture", async () => {
    const user = userEvent.setup();
    let settleCapture!: (response: Response) => void;
    const pendingCapture = new Promise<Response>((resolve) => { settleCapture = resolve; });
    const { request } = cameraHarness({ onCapture: () => pendingCapture });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const takeDetail = within(panel).getByRole("button", { name: "Take detail picture" });
    await waitFor(() => expect(takeDetail).toBeEnabled());
    await user.click(takeDetail);

    const pendingButton = within(panel).getByRole("button", { name: "Taking detail picture…" });
    expect(pendingButton).toBeDisabled();
    expect(within(panel).getByRole("button", { name: "Take survey picture" })).toBeDisabled();
    expect(within(panel).getByRole("button", { name: "Try autofocus" })).toBeDisabled();

    settleCapture(jsonResponse({ detail: "capture cancelled" }, 503));
    expect(await within(panel).findByRole("alert")).toHaveTextContent("Picture capture failed: capture cancelled");
  });

  it("displays the authenticated JPEG with its timestamp and compact evidence metadata", async () => {
    const user = userEvent.setup();
    const { createObjectURL } = stubCameraUrls("blob:camera-frame");
    const { request, captured } = cameraHarness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const takePicture = within(panel).getByRole("button", { name: "Take detail picture" });
    await waitFor(() => expect(takePicture).toBeEnabled());
    await user.click(takePicture);

    const image = await within(panel).findByRole("img", { name: /desk camera picture captured.*ov5647/i });
    expect(image).toHaveAttribute("src", "blob:camera-frame");
    expect(panel.querySelector("time")).toHaveAttribute("datetime", captured.capturedAt);
    expect(within(panel).getByText("2592 × 1944")).toBeInTheDocument();
    expect(within(panel).getByText("24 KB")).toBeInTheDocument();
    expect(within(panel).getByText("motorized autofocus unavailable")).toBeInTheDocument();
    expect(within(panel).getByText("configured candidate")).toBeInTheDocument();
    expect(within(panel).getByText("picamera2")).toBeInTheDocument();
    expect(createObjectURL).toHaveBeenCalledTimes(1);
    expect(request).toHaveBeenCalledWith(
      `/api/camera/frames/${captured.frameId}`,
      expect.objectContaining({
        headers: expect.objectContaining({ "X-Co-Arm-Token": "z".repeat(40) }),
      }),
    );
  });

  it("shows Module 3 Wide capability metadata and labels its field of view nominal", async () => {
    const user = userEvent.setup();
    stubCameraUrls("blob:module3-wide-frame");
    const cameraProfile = {
      id: "module3-wide",
      productName: "Raspberry Pi Camera Module 3 Wide",
      sensorModel: "imx708",
      lensVariant: "wide",
      nativeDimensions: { width: 4608, height: 2592 },
      nominalFocalLengthMm: 2.75,
      nominalFieldOfViewDegrees: { horizontal: 102, vertical: 67 },
      captureProfiles: {
        survey: { width: 2304, height: 1296 },
        detail: { width: 4608, height: 2592 },
      },
    };
    const captured: CameraObservation = {
      ...cameraObservation("camera_module3_wide"),
      sensorModel: "imx708",
      identityConfidence: "driver_reported",
      dimensions: { width: 4608, height: 2592 },
      cameraProfile,
      autofocus: {
        capability: "supported",
        mode: "continuous",
        state: "focused",
        range: "normal",
      },
    };
    const { request } = cameraHarness({
      captured,
      status: {
        sensorModel: "imx708",
        identityConfidence: "driver_reported",
        cameraProfile,
        autofocus: captured.autofocus,
      },
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    await user.click(await within(panel).findByRole("button", { name: "Take detail picture" }));

    expect(await within(panel).findByText("Raspberry Pi Camera Module 3 Wide · wide")).toBeInTheDocument();
    expect(within(panel).getByText("102° × 67° nominal")).toBeInTheDocument();
    expect(within(panel).getByText("focused · normal")).toBeInTheDocument();
    expect(within(panel).getAllByText(/native-detail still/).length).toBeGreaterThan(0);
  });

  it("does not call a downsampled SIM detail profile native", async () => {
    const user = userEvent.setup();
    const simulatedProfile = {
      id: "module3-wide",
      productName: "Raspberry Pi Camera Module 3 Wide (simulated)",
      sensorModel: "imx708",
      lensVariant: "wide",
      nativeDimensions: { width: 4608, height: 2592 },
      nominalFocalLengthMm: 2.75,
      nominalFieldOfViewDegrees: { horizontal: 102, vertical: 67 },
      captureProfiles: {
        survey: { width: 960, height: 540 },
        detail: { width: 1280, height: 720 },
      },
    };
    const { request } = cameraHarness({
      status: {
        simulated: true,
        identityConfidence: "simulated_model",
        cameraProfile: simulatedProfile,
      },
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });

    expect(within(panel).getAllByText(/detail-profile still/).length).toBeGreaterThan(0);
    expect(within(panel).queryByText(/native-detail still/)).not.toBeInTheDocument();
  });

  it("reports a capture failure separately from a captured frame that cannot be loaded", async () => {
    const user = userEvent.setup();
    const { request } = cameraHarness({
      onFrame: () => jsonResponse({ detail: "JPEG missing" }, 502),
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const takePicture = within(panel).getByRole("button", { name: "Take detail picture" });
    await waitFor(() => expect(takePicture).toBeEnabled());
    await user.click(takePicture);

    const alert = await within(panel).findByRole("alert");
    expect(alert).toHaveTextContent("Picture was captured, but its JPEG could not be loaded: JPEG missing");
    expect(within(panel).queryByRole("img")).not.toBeInTheDocument();
  });

  it("revokes camera blob URLs when a picture is replaced and when the panel unmounts", async () => {
    const user = userEvent.setup();
    const { createObjectURL, revokeObjectURL } = stubCameraUrls("blob:camera-old", "blob:camera-new");
    const latest = cameraObservation("camera_old");
    const captured = cameraObservation("camera_new");
    const { request } = cameraHarness({
      status: { latestFrameId: latest.frameId },
      latest,
      captured,
    });
    const view = render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("tab", { name: "Camera" }));
    const panel = screen.getByRole("tabpanel", { name: "Camera" });
    const oldImage = await within(panel).findByRole("img");
    expect(oldImage).toHaveAttribute("src", "blob:camera-old");
    expect(createObjectURL).toHaveBeenCalledTimes(1);

    await user.click(within(panel).getByRole("button", { name: "Take detail picture" }));
    await waitFor(() => expect(within(panel).getByRole("img")).toHaveAttribute("src", "blob:camera-new"));
    expect(revokeObjectURL).toHaveBeenCalledWith("blob:camera-old");

    view.unmount();
    expect(revokeObjectURL).toHaveBeenCalledWith("blob:camera-new");
  });

  it("shows trusted truth-v3 with healthy mode-0 free encoder diagnostics", async () => {
    const { request } = harness();
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("TRUSTED");
    expect(truth).toHaveTextContent("mode 0 · free encoder");
    expect(truth).toHaveTextContent("step idle");
    expect(truth).toHaveTextContent("resyncs 0");
    expect(truth).toHaveTextContent("sample 12ms");
    const firmware = screen.getByLabelText("Controller firmware");
    expect(firmware).toHaveTextContent("firmware arm-hat-2.3.0");
    expect(firmware).toHaveTextContent("truth-v3");
    expect(firmware).toHaveTextContent("boot 00000001");
  });

  it("labels native absolute multi-turn without legacy step diagnostics", async () => {
    const { request } = harness({
      controller: {
        controllerId: "armhat-test-a",
        bootId: "boot-test-00000001",
        firmwareVersion: "arm-hat-2.4.0",
        protocolVersion: 1,
        multiTurnTruthV3: true,
        multiTurnAbsoluteV1: true,
      } as SimpleControllerIdentity,
    });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("TRUSTED");
    expect(truth).toHaveTextContent("absolute encoder");
    expect(truth).not.toHaveTextContent("mode 0 Â· free encoder");
    expect(truth).not.toHaveTextContent("step idle");

    const firmware = screen.getByLabelText("Controller firmware");
    expect(firmware).toHaveTextContent("firmware arm-hat-2.4.0");
    expect(firmware).toHaveTextContent("absolute-v1");
    expect(firmware).not.toHaveTextContent("truth-v3");
  });

  it("distinguishes an active step waiting for countdown from one observed", async () => {
    const { request } = harness({
      multiTurnTruth: {
        tracking: true, valid: true, stepMode: true, stepOutstanding: true,
        countdownObserved: false, resyncNeeded: false, resyncCount: 0,
      },
    });
    const view = render(<SimpleArm request={request} />);
    expect(await screen.findByLabelText("Base multi-turn truth"))
      .toHaveTextContent("step active · waiting for countdown");

    view.unmount();
    const seen = harness({
      multiTurnTruth: {
        tracking: true, valid: true, stepMode: true, stepOutstanding: true,
        countdownObserved: true, resyncNeeded: false, resyncCount: 0,
      },
    });
    render(<SimpleArm request={seen.request} />);
    expect(await screen.findByLabelText("Base multi-turn truth"))
      .toHaveTextContent("step active · countdown seen");
  });

  it("shows unknown and resync required without treating mode 0 as the fault", async () => {
    const { request } = harness({
      positionTrusted: false,
      multiTurnTruth: {
        tracking: true, valid: false, stepMode: false, stepOutstanding: false,
        countdownObserved: true, resyncNeeded: true, resyncCount: 3,
      },
    });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("UNKNOWN");
    expect(truth).toHaveTextContent("mode 0 · free encoder");
    expect(truth).toHaveTextContent("resync required");
    expect(truth).toHaveTextContent("resyncs 3");
  });

  it("falls back honestly when an older gateway omits identity and truth fields", async () => {
    const { request } = harness({ controller: null, multiTurnTruth: null });
    render(<SimpleArm request={request} />);

    const firmware = await screen.findByLabelText("Controller firmware");
    expect(firmware).toHaveTextContent("firmware unknown");
    expect(firmware).toHaveTextContent("truth unknown");
    const truth = screen.getByLabelText("Base multi-turn truth");
    // positionTrusted remains the authoritative drive gate even if a mixed
    // deployment has not learned the richer diagnostic row yet.
    expect(truth).toHaveTextContent("TRUSTED");
    expect(truth).toHaveTextContent("mode unknown");
    expect(truth).toHaveTextContent("step unknown");
    expect(truth).toHaveTextContent("resyncs unknown");
  });

  it("turns an untrusted Base with no first sample into one home action", async () => {
    const { request } = harness({ positionTrusted: false, multiTurnTruth: null });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("HOME REQUIRED");
    expect(truth).toHaveTextContent("place at zero");
    expect(truth).not.toHaveTextContent("mode unknown");
    expect(truth).not.toHaveTextContent("step unknown");
    expect(truth).not.toHaveTextContent("resyncs unknown");
  });

  it("reports an undiscovered Base directly instead of a wall of unknown fields", async () => {
    const { request } = harness({
      positionTrusted: false,
      multiTurnTruth: null,
      baseOnline: false,
    });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("BASE NOT DETECTED");
    expect(truth).toHaveTextContent("waiting for servo ID 1");
    expect(truth).not.toHaveTextContent("mode unknown");
    expect(truth).not.toHaveTextContent("step unknown");
    expect(truth).not.toHaveTextContent("resyncs unknown");
    expect(screen.getByRole("button", { name: "Set Base zero here" })).toBeDisabled();
  });

  it("uses persisted Scan PING evidence for Base homing without enabling torque or motion", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({
      positionTrusted: false,
      multiTurnTruth: null,
      baseOnline: false,
      lastScan: {
        minId: 0,
        maxId: 10,
        foundIds: [1, 2, 3, 4],
        pingFoundIds: [1, 2, 3, 4],
        telemetryUnavailableIds: [1],
        telemetryRecoveredIds: [],
        collisionSuspected: false,
        collisionId: null,
      },
    });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("HOME REQUIRED");
    expect(truth).toHaveTextContent("servo ID 1 detected by Scan");
    expect(truth).toHaveTextContent("telemetry unavailable");
    expect(truth).not.toHaveTextContent("BASE NOT DETECTED");

    const controls = screen.getByLabelText("Base setup and test");
    const home = within(controls).getByRole("button", { name: "Set Base zero here" });
    expect(home).toBeEnabled();
    expect(within(controls).getByText(/Servo ID 1 answered Scan/i)).toBeInTheDocument();
    for (const preset of within(controls).getByRole("group", { name: "Base test positions" }).querySelectorAll("button")) {
      expect(preset).toBeDisabled();
    }
    expect(screen.getByRole("slider", { name: "Drive Base" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Hold all" })).toBeDisabled();
    const baseSection = truth.closest("section");
    expect(baseSection).not.toBeNull();
    expect(within(baseSection as HTMLElement).getByRole("button", { name: "Hold" })).toBeDisabled();

    await user.click(home);
    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));
    expect(calls.find((call) => call.url.endsWith("/joints/joint_1/calibrate"))?.body)
      .toEqual({ here: ["rawZero"] });
  });

  it("keeps Scan-backed Base homing disabled while stopped or not free", async () => {
    const evidence: SimpleArmScanEvidence = {
      minId: 0,
      maxId: 10,
      foundIds: [1, 2, 3, 4],
      pingFoundIds: [1, 2, 3, 4],
      telemetryUnavailableIds: [1],
    };
    for (const blocked of [{ stopped: true }, { held: [1] }]) {
      const { request } = harness({
        ...blocked,
        positionTrusted: false,
        multiTurnTruth: null,
        baseOnline: false,
        lastScan: evidence,
      });
      const view = render(<SimpleArm request={request} />);
      expect(await screen.findByRole("button", { name: "Set Base zero here" })).toBeDisabled();
      view.unmount();
    }
  });

  it("turns a discovered but unseeded Base into the same home action", async () => {
    const { request } = harness({
      positionTrusted: false,
      multiTurnTruth: {
        tracking: false,
        valid: false,
        stepMode: false,
        stepOutstanding: false,
        countdownObserved: false,
        resyncNeeded: false,
        resyncCount: 0,
      },
    });
    render(<SimpleArm request={request} />);

    const truth = await screen.findByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("HOME REQUIRED");
    expect(truth).toHaveTextContent("place at zero · press zero here");
    expect(truth).not.toHaveTextContent("UNKNOWN");
  });

  it("reserves upgrade needed for a known older firmware", async () => {
    const { request } = harness({
      controller: { firmwareVersion: "arm-hat-2.2.0", multiTurnTruthV3: false },
      multiTurnTruth: null,
    });
    render(<SimpleArm request={request} />);

    const firmware = await screen.findByLabelText("Controller firmware");
    expect(firmware).toHaveTextContent("firmware arm-hat-2.2.0");
    expect(firmware).toHaveTextContent("upgrade needed");
  });

  it("shows each joint's live angle and its four numbers", async () => {
    const { request } = harness();
    render(<SimpleArm request={request} />);

    expect(await screen.findByText("Base")).toBeInTheDocument();
    // Scoped to the panel: the flat view shows the base angle too, and an
    // unscoped query cannot tell which one it found.
    const panel = within(screen.getByRole("tabpanel", { name: "Joints" }));
    expect(panel.getByText("Shoulder")).toBeInTheDocument();
    expect(panel.getByText("Elbow")).toBeInTheDocument();
    // 2300 - 2048 = 252 ticks at 1:1 is 22.1 degrees.
    expect(panel.getByText("22.1°")).toBeInTheDocument();
    expect(panel.getByText("raw 2300")).toBeInTheDocument();
  });

  it("calibrates by storing wherever the joint is standing right now", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({ baseRaw: 2300 });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "Set Base zero here" }));

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));
    const calibrate = calls.find((call) => call.url.endsWith("/joints/joint_1/calibrate"));
    // The field to capture, not a position. Sending our own rawPosition stored a
    // reading up to a poll and a refresh old, so "here" landed where the joint
    // had been rather than where it was; the Pi now reads it as the request lands.
    expect(calibrate?.body).toEqual({ here: ["rawZero"] });
  });

  it("shows the independent recovery gate and confirms only its explicit zero action", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({
      baseReferenceRequired: true,
      baseReferenceReason: "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
    });
    render(<SimpleArm request={request} />);

    expect(await screen.findByText(/Recovery Base reference is untrusted/i)).toBeInTheDocument();
    expect(screen.getByText(/Clear STOP will not unlock motion/i)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Hold all" })).toBeDisabled();
    expect(screen.getByRole("slider", { name: "Drive Shoulder" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Set Base zero here" })).toBeEnabled();

    await user.click(screen.getByRole("button", { name: "Set Base zero here" }));
    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));
    expect(calls.find((call) => call.url.endsWith("/joints/joint_1/calibrate"))?.body)
      .toEqual({
        here: ["rawZero"],
        confirmedPhysicalBaseZero: true,
      });
  });

  it("keeps malformed recovery state locked without offering a zero action", async () => {
    const { request, calls } = harness({
      baseReferenceRequired: true,
      baseReferenceReason: "BASE_REFERENCE_MARKER_INVALID",
    });
    render(<SimpleArm request={request} />);

    expect(await screen.findByText(/Recovery Base-reference marker is invalid/i)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Hold all" })).toBeDisabled();
    expect(screen.getByRole("slider", { name: "Drive Shoulder" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Set Base zero here" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();
    expect(calls.filter((call) => call.url.endsWith("/calibrate"))).toHaveLength(0);
  });

  it("keeps drive and limits locked but allows explicit Base homing when position is unknown", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({ positionTrusted: false });
    render(<SimpleArm request={request} />);
    await screen.findByText(/Base position is unknown/);

    expect(screen.getByRole("slider", { name: "Drive Base" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Set Base min here" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Set Base max here" })).toBeDisabled();
    const home = screen.getByRole("button", { name: "Set Base zero here" });
    expect(home).toBeEnabled();

    await user.click(home);

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));
    expect(calls.find((call) => call.url.endsWith("/joints/joint_1/calibrate"))?.body)
      .toEqual({ here: ["rawZero"] });
  });

  it("puts Base homing and repeatable test positions in one self-service control", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({ baseMinDegrees: -181, baseMaxDegrees: 181 });
    render(<SimpleArm request={request} />);

    const controls = await screen.findByLabelText("Base setup and test");
    expect(within(controls).getByText("Reference ready")).toBeInTheDocument();
    expect(within(controls).getByText(/Measured 22\.1/)).toBeInTheDocument();

    const presets = within(controls).getByRole("group", { name: "Base test positions" });
    expect(within(presets).getAllByRole("button")).toHaveLength(5);
    expect(within(presets).getByRole("button", { name: /\+120/ })).toBeEnabled();
    await user.click(within(presets).getByRole("button", { name: /\+120/ }));

    await waitFor(() => expect(calls.some((call) => (
      call.url.endsWith("/simple/target") && JSON.stringify(call.body) === JSON.stringify({ joint_1: 120 })
    ))).toBe(true));
    expect(within(controls).getByText(/Target \+120.*accepted/i)).toBeInTheDocument();
  });

  it("keeps Base test positions locked until zero is trusted", async () => {
    const { request } = harness({ positionTrusted: false });
    render(<SimpleArm request={request} />);

    const controls = await screen.findByLabelText("Base setup and test");
    expect(within(controls).getByRole("button", { name: "Set Base zero here" })).toBeEnabled();
    for (const preset of within(controls).getByRole("group", { name: "Base test positions" }).querySelectorAll("button")) {
      expect(preset).toBeDisabled();
    }
    expect(within(controls).getByText(/Turn the Base by hand to the marked zero/i)).toBeInTheDocument();
  });

  it("offers a one-click range repair when the saved Base range cannot reach both test sides", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);

    const controls = await screen.findByLabelText("Base setup and test");
    const repair = within(controls).getByRole("button", {
      name: "Set Base range to minus 180 through plus 180 degrees",
    });
    expect(repair).toBeEnabled();
    await user.click(repair);

    await waitFor(() => expect(calls.some((call) => (
      call.url.endsWith("/joints/joint_1/calibrate") &&
      JSON.stringify(call.body) === JSON.stringify({ minDegrees: -180, maxDegrees: 180 })
    ))).toBe(true));
    expect(within(controls).getByText(/Base range set to -180.*through \+180/i)).toBeInTheDocument();
  });

  it("shows a refused preset move beside the Base controls with the real reason", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness();
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (init?.method === "POST" && url.endsWith("/simple/target")) {
        return jsonResponse({
          detail: "Base position became unknown before the move. Put it on the mark and set zero here.",
        }, 409);
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);

    const controls = await screen.findByLabelText("Base setup and test");
    await user.click(within(controls).getByRole("button", { name: /-30/ }));

    const refusal = await within(controls).findByRole("alert");
    expect(refusal).toHaveTextContent("Base position became unknown before the move");
    expect(refusal).not.toHaveTextContent("Robot gateway state changed");
  });

  it("shows the detailed Base homing refusal instead of a generic gateway banner", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ positionTrusted: false });
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (init?.method === "POST" && url.endsWith("/joints/joint_1/calibrate")) {
        return jsonResponse({
          detail: "Base position is no longer trusted because continuous absolute encoder feedback was lost. Keep Base free on the physical zero mark, then press Here again.",
        }, 409);
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText(/Base position is unknown/);

    await user.click(screen.getByRole("button", { name: "Set Base zero here" }));

    expect(
      await screen.findByText(/continuous absolute encoder feedback was lost/i),
    ).toBeInTheDocument();
    expect(screen.getByText(/Keep Base free on the physical zero mark/i)).toBeInTheDocument();
  });

  it("takes a limit typed in degrees, without moving the joint there first", async () => {
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    type("Base max", "35");

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));
    expect(calls.at(-1)?.body).toEqual({ maxDegrees: 35 });
  });

  it("sends the number that was typed, not every number on the way to it", async () => {
    // Per-keystroke commits stored "-4" on the way to "-45", and the reply to
    // one keystroke overwrote the reply to the next — which is what made a
    // rejected value look like Enter doing nothing.
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const field = screen.getByRole("spinbutton", { name: "Base min" });
    fireEvent.change(field, { target: { value: "-4" } });
    fireEvent.change(field, { target: { value: "-45" } });
    expect(calls.filter((call) => call.url.endsWith("/calibrate"))).toHaveLength(0);

    fireEvent.keyDown(field, { key: "Enter" });
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ minDegrees: -45 }));
    expect(calls.filter((call) => call.url.endsWith("/calibrate"))).toHaveLength(1);
  });

  it("flips direction and sets speed without touching anything else", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: /Base direction/i }));
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ direction: -1 }));

    type("Base speed", "600");
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ speed: 600 }));
  });

  it("prepares from slider release without moving, then applies the exact plan once", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({
      planWarnings: ["Near the configured reach limit."],
      planResolvedPose: { joint_1: 25, joint_3: 12 },
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const base = screen.getByRole("slider", { name: "Drive Base" });
    fireEvent.change(base, { target: { value: "30" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Elbow" }), { target: { value: "12" } });
    // Dragging a bar is local; no plan and, critically, no physical target.
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
    expect(calls.filter((call) => call.url.endsWith("/plans/preview"))).toHaveLength(0);
    fireEvent.pointerUp(base);

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/plans/preview"))).toBe(true));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
    expect(calls.find((call) => call.url.endsWith("/plans/preview"))?.body).toEqual({
      targets: { joint_1: 30, joint_3: 12 },
    });
    expect(screen.getByText("Plan ready. Review the dashed pose, then apply it.")).toBeInTheDocument();
    expect(screen.getByTestId("arm2d-planned-pose")).toBeInTheDocument();
    expect(screen.getByText("Lowest clearance 42.5 mm")).toBeInTheDocument();
    expect(screen.getByText("Near the configured reach limit.")).toBeInTheDocument();
    expect(within(screen.getByLabelText("Current and planned joint angles")).getByText(/22\.1°.*25\.0°/)).toBeInTheDocument();
    expect(screen.getByText(/Base was clamped from 30\.0 degrees to 25\.0 degrees/)).toBeInTheDocument();

    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/plans/execute"))).toHaveLength(1));
    expect(calls.find((call) => call.url.endsWith("/plans/execute"))?.body).toEqual({
      planId: "plan_test_1",
      planDigest: `sha256:${"a".repeat(64)}`,
    });
    expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(0);
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
  });

  it("releases a stale manual-control lock when the selected SIM instance restarts", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ planResolvedPose: { joint_1: 30 } });
    let executed = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({
          ...body,
          backendInstanceId: executed ? "sim-instance-new" : "sim-instance-old",
          moving: false,
        });
      }
      if (init?.method === "POST" && url.endsWith("/plans/execute")) {
        const response = await normal(input, init);
        const body = await response.json();
        executed = true;
        return jsonResponse({ ...body, backendInstanceId: "sim-instance-old" });
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    await screen.findByText("Base");

    const base = screen.getByRole("slider", { name: "Drive Base" });
    fireEvent.change(base, { target: { value: "30" } });
    fireEvent.pointerUp(base);
    await screen.findByText("Plan ready. Review the dashed pose, then apply it.");
    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(onControlBusyChange).toHaveBeenCalledWith(true));

    expect(await screen.findByRole("alert", {}, { timeout: 2_500 })).toHaveTextContent(/backend restarted/i);
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(false));
  });

  it("does not report a restart for a normal REAL move on one stable backend instance", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ planResolvedPose: { joint_1: 30 } });
    let executed = false;
    let postExecutePolls = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        if (executed) postExecutePolls += 1;
        const arrived = executed && postExecutePolls >= 2;
        return jsonResponse({
          ...body,
          backendInstanceId: "real-instance-stable",
          moving: executed && !arrived,
          joints: body.joints.map((candidate: { id: string }) => (
            candidate.id === "joint_1" && executed
              ? { ...candidate, degrees: arrived ? 30 : 26 }
              : candidate
          )),
        });
      }
      if (init?.method === "POST" && url.endsWith("/plans/execute")) {
        const response = await normal(input, init);
        const body = await response.json();
        executed = true;
        return jsonResponse({ ...body, backendInstanceId: "real-instance-stable" });
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    await screen.findByText("Base");

    const base = screen.getByRole("slider", { name: "Drive Base" });
    fireEvent.change(base, { target: { value: "30" } });
    fireEvent.pointerUp(base);
    await screen.findByText("Plan ready. Review the dashed pose, then apply it.");
    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(true));

    await waitFor(() => expect(postExecutePolls).toBeGreaterThanOrEqual(2));
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(false));
    expect(screen.queryByText("The arm backend restarted before that target was verified. Prepare a fresh move.")).not.toBeInTheDocument();
  });

  it("keeps a quiet Camera target active until it settles inside its six-degree tolerance", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({
      // The physical Base preview can be intentionally null while its
      // multi-turn position remains trusted. A non-numeric preview entry must
      // never become an execution target that blocks Camera retirement.
      planResolvedPose: { joint_1: null, joint_4: 40 },
    });
    let executed = false;
    let postExecutePolls = 0;
    let cameraArrived = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        if (executed) postExecutePolls += 1;
        return jsonResponse({
          ...body,
          backendInstanceId: "real-camera-instance-stable",
          // Another joint may briefly provide aggregate moving evidence. That
          // must not bypass the Camera-specific quiet pursuit window.
          moving: executed && postExecutePolls === 1,
          joints: body.joints.map((candidate: { id: string }) => (
            candidate.id === "joint_4" && executed
              ? { ...candidate, degrees: cameraArrived ? 34.4 : 30, moving: false }
              : candidate
          )),
        });
      }
      if (init?.method === "POST" && url.endsWith("/plans/execute")) {
        const response = await normal(input, init);
        const body = await response.json();
        executed = true;
        return jsonResponse({ ...body, backendInstanceId: "real-camera-instance-stable" });
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    await screen.findByText("Base");

    const camera = screen.getByRole("slider", { name: "Drive Camera" });
    fireEvent.change(camera, { target: { value: "40" } });
    fireEvent.pointerUp(camera);
    await screen.findByText("Plan ready. Review the dashed pose, then apply it.");
    await user.click(screen.getByRole("button", { name: "Apply move" }));

    await waitFor(() => expect(onControlBusyChange).toHaveBeenCalledWith(true));
    await waitFor(() => expect(postExecutePolls).toBeGreaterThanOrEqual(8), { timeout: 3_000 });
    expect(onControlBusyChange).toHaveBeenLastCalledWith(true);
    expect(screen.queryByText(/target was not reached/i)).not.toBeInTheDocument();
    cameraArrived = true;
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(false));
    expect(screen.queryByText("The controller reports no active motion, but the target was not reached. Prepare a fresh move.")).not.toBeInTheDocument();
  });

  it("does not declare a quiet Camera miss before the Pi pursuit window expires", async () => {
    vi.useFakeTimers();
    const { request: normal } = harness({ planResolvedPose: { joint_1: null, joint_4: 40 } });
    let executed = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({
          ...body,
          backendInstanceId: "real-camera-grace-stable",
          moving: false,
          joints: body.joints.map((candidate: { id: string }) => (
            candidate.id === "joint_4" && executed
              ? { ...candidate, degrees: 30, moving: false }
              : candidate
          )),
        });
      }
      if (init?.method === "POST" && url.endsWith("/plans/execute")) {
        const response = await normal(input, init);
        const body = await response.json();
        executed = true;
        return jsonResponse({ ...body, backendInstanceId: "real-camera-grace-stable" });
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    const view = render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    try {
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      const camera = screen.getByRole("slider", { name: "Drive Camera" });
      fireEvent.change(camera, { target: { value: "40" } });
      fireEvent.pointerUp(camera);
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      expect(screen.getByText("Plan ready. Review the dashed pose, then apply it.")).toBeInTheDocument();

      fireEvent.click(screen.getByRole("button", { name: "Apply move" }));
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      expect(onControlBusyChange).toHaveBeenLastCalledWith(true);

      await act(async () => { await vi.advanceTimersByTimeAsync(8_999); });
      expect(onControlBusyChange).toHaveBeenLastCalledWith(true);
      expect(screen.queryByText(/target was not reached/i)).not.toBeInTheDocument();

      await act(async () => { await vi.advanceTimersByTimeAsync(1); });
      expect(screen.getByRole("alert")).toHaveTextContent(/target was not reached/i);
      expect(onControlBusyChange).toHaveBeenLastCalledWith(false);
    } finally {
      view.unmount();
      vi.useRealTimers();
    }
  });

  it.each([
    { camera: 34, shoulder: 30, arrived: false },
    { camera: 34.001, shoulder: 30, arrived: true },
    { camera: 45.999, shoulder: 30, arrived: true },
    { camera: 46, shoulder: 30, arrived: false },
    { camera: 34.4, shoulder: 29, arrived: false },
    { camera: 34.4, shoulder: 29.001, arrived: true },
  ])("checks each mixed target with its own arrival tolerance: $camera / $shoulder", async ({ camera, shoulder, arrived }) => {
    vi.useFakeTimers();
    const { request: normal } = harness({ planResolvedPose: { joint_1: null, joint_2: 30, joint_4: 40 } });
    let executed = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const response = await normal(input, init);
      if (String(input) === "/api/arm/simple/state") {
        const body = await response.json();
        return jsonResponse({
          ...body,
          moving: false,
          joints: body.joints.map((candidate: { id: string }) => {
            if (!executed) return candidate;
            if (candidate.id === "joint_2") return { ...candidate, degrees: shoulder, moving: false };
            if (candidate.id === "joint_4") return { ...candidate, degrees: camera, moving: false };
            return candidate;
          }),
        });
      }
      if (String(input).endsWith("/plans/execute")) executed = true;
      return response;
    });
    const onControlBusyChange = vi.fn();
    const view = render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    try {
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      const cameraSlider = screen.getByRole("slider", { name: "Drive Camera" });
      fireEvent.change(cameraSlider, { target: { value: "40" } });
      fireEvent.change(screen.getByRole("slider", { name: "Drive Shoulder" }), { target: { value: "30" } });
      fireEvent.pointerUp(cameraSlider);
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      fireEvent.click(screen.getByRole("button", { name: "Apply move" }));
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      expect(onControlBusyChange).toHaveBeenLastCalledWith(true);
      await act(async () => { await vi.advanceTimersByTimeAsync(150); });
      expect(onControlBusyChange).toHaveBeenLastCalledWith(!arrived);
      expect(screen.queryByText(/target was not reached/i)).not.toBeInTheDocument();
    } finally {
      view.unmount();
      vi.useRealTimers();
    }
  });

  it("retires a Camera move immediately on a real gateway restart during pursuit", async () => {
    vi.useFakeTimers();
    const { request: normal } = harness({ planResolvedPose: { joint_1: null, joint_4: 40 } });
    let executed = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const response = await normal(input, init);
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const body = await response.json();
        return jsonResponse({
          ...body,
          backendInstanceId: executed ? "gateway:new-process" : "gateway:old-process",
          moving: false,
        });
      }
      if (url.endsWith("/plans/execute")) {
        const body = await response.json();
        executed = true;
        return jsonResponse({ ...body, backendInstanceId: "gateway:old-process" });
      }
      return response;
    });
    const onControlBusyChange = vi.fn();
    const view = render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    try {
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      const camera = screen.getByRole("slider", { name: "Drive Camera" });
      fireEvent.change(camera, { target: { value: "40" } });
      fireEvent.pointerUp(camera);
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      fireEvent.click(screen.getByRole("button", { name: "Apply move" }));
      await act(async () => { await vi.advanceTimersByTimeAsync(0); });
      expect(onControlBusyChange).toHaveBeenLastCalledWith(true);
      await act(async () => { await vi.advanceTimersByTimeAsync(150); });
      expect(screen.getByRole("alert")).toHaveTextContent(/backend restarted/i);
      expect(onControlBusyChange).toHaveBeenLastCalledWith(false);
    } finally {
      view.unmount();
      vi.useRealTimers();
    }
  });

  it.each([null, false, "40", Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY])(
    "refuses an explicitly invalid requested Camera target: %s", async (value) => {
    const { request, calls } = harness({ planResolvedPose: { joint_4: value } });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const camera = screen.getByRole("slider", { name: "Drive Camera" });
    fireEvent.change(camera, { target: { value: "40" } });
    fireEvent.pointerUp(camera);

    expect(await screen.findByRole("alert")).toHaveTextContent(
      "The arm planner returned an invalid joint_4 target. The move was not armed.",
    );
    expect(screen.getByRole("button", { name: "Apply move" })).toBeDisabled();
    expect(calls.filter((call) => call.url.endsWith("/plans/execute"))).toHaveLength(0);
  });

  it("releases a missed-goal lock after authoritative telemetry stays idle", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ planResolvedPose: { joint_1: 30 } });
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (String(input) === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, backendInstanceId: "sim-instance-stable", moving: false });
      }
      if (init?.method === "POST" && String(input).endsWith("/plans/execute")) {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, backendInstanceId: "sim-instance-stable" });
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    await screen.findByText("Base");

    const base = screen.getByRole("slider", { name: "Drive Base" });
    fireEvent.change(base, { target: { value: "30" } });
    fireEvent.pointerUp(base);
    await screen.findByText("Plan ready. Review the dashed pose, then apply it.");
    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(true));

    expect(await screen.findByRole("alert", {}, { timeout: 3_000 })).toHaveTextContent(/no active motion/i);
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(false));
  });

  it("turns the exact independent floor-sweep refusal into a reviewed Single-mode route", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({
      planPreviewFailure: { status: 409, detail: FLOOR_SWEEP_REFUSAL },
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "30" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Elbow" }), { target: { value: "20" } });
    fireEvent.pointerUp(shoulder);

    expect(await screen.findByText("Floor-safe route ready · 2 checked legs")).toBeInTheDocument();
    expect(screen.getByTestId("arm2d-sequence-path")).toBeInTheDocument();
    expect(screen.queryByText(FLOOR_SWEEP_REFUSAL)).not.toBeInTheDocument();
    expect(calls.find((call) => call.url.endsWith("/sequences/preview"))?.body).toEqual({
      waypoints: [
        { targets: { joint_3: 20 }, label: "Floor-safe Elbow first" },
        { targets: { joint_2: 30, joint_3: 20 }, label: "Destination" },
      ],
    });

    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(1));
    expect(calls.find((call) => call.url.endsWith("/sequences/execute"))?.body).toEqual({
      sequenceId: "armseq_test_1234",
      sequenceDigest: `sha256:${"d".repeat(64)}`,
      arrivalTimeoutMs: 10_000,
    });
    expect(calls.filter((call) => call.url.endsWith("/plans/execute"))).toHaveLength(0);
  });

  it("keeps omitted Camera tracking through a floor route with nullable Base output", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({
      planPreviewFailure: { status: 409, detail: FLOOR_SWEEP_REFUSAL },
      sequencePreviewResponse: (body) => {
        const waypoints = body.waypoints.map((waypoint, index) => ({
          index,
          startPose: {},
          resolvedPose: index === body.waypoints.length - 1
            ? { joint_1: null, joint_2: 30 }
            : waypoint.targets,
          warnings: [],
          lowestClearanceMm: 35,
        }));
        return jsonResponse({
          sequenceId: "armseq_test_1234",
          sequenceDigest: `sha256:${"d".repeat(64)}`,
          expiresAt: "2099-01-01T00:00:00Z",
          expiresInMs: 120_000,
          previewDurationMs: 12,
          waypointCount: waypoints.length,
          measuredPose: {},
          waypoints,
          warnings: [],
          lowestClearanceMm: 35,
        });
      },
    });
    let executed = false;
    let postExecutePolls = 0;
    let cameraArrived = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/arm/simple/state") {
        const response = await normal(input, init);
        const body = await response.json();
        if (executed) postExecutePolls += 1;
        return jsonResponse({
          ...body,
          moving: false,
          joints: body.joints.map((candidate: { id: string }) => {
            if (!executed) return candidate;
            if (candidate.id === "joint_2") return { ...candidate, degrees: 30, moving: false };
            if (candidate.id === "joint_3") return { ...candidate, degrees: 20, moving: false };
            if (candidate.id === "joint_4") {
              return { ...candidate, degrees: cameraArrived ? 36.5 : 30, moving: false };
            }
            return candidate;
          }),
        });
      }
      if (init?.method === "POST" && url.endsWith("/sequences/execute")) {
        const response = await normal(input, init);
        executed = true;
        return response;
      }
      return normal(input, init);
    });
    const onControlBusyChange = vi.fn();
    render(<SimpleArm request={request} onControlBusyChange={onControlBusyChange} />);
    await screen.findByText("Base");

    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "30" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Elbow" }), { target: { value: "20" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Camera" }), { target: { value: "40" } });
    fireEvent.pointerUp(shoulder);

    expect(await screen.findByText("Floor-safe route ready · 2 checked legs")).toBeInTheDocument();
    await user.click(screen.getByRole("button", { name: "Apply move" }));
    await waitFor(() => expect(postExecutePolls).toBeGreaterThanOrEqual(2), { timeout: 2_500 });
    expect(onControlBusyChange).toHaveBeenLastCalledWith(true);
    cameraArrived = true;
    await waitFor(() => expect(onControlBusyChange).toHaveBeenLastCalledWith(false));
    expect(screen.queryByText(/target was not reached/i)).not.toBeInTheDocument();
  });

  it("invalidates a ready automatic floor route when STOP is observed externally", async () => {
    const { request: normal } = harness({
      planPreviewFailure: { status: 409, detail: FLOOR_SWEEP_REFUSAL },
    });
    let externallyStopped = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (!init?.method && String(input).endsWith("/api/arm/simple/state") && externallyStopped) {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, stopped: true });
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "30" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Elbow" }), { target: { value: "20" } });
    fireEvent.pointerUp(shoulder);
    await screen.findByText("Floor-safe route ready · 2 checked legs");
    expect(screen.getByRole("button", { name: "Apply move" })).toBeEnabled();

    externallyStopped = true;
    await screen.findByRole("button", { name: "Clear STOP" });
    expect(screen.getByRole("button", { name: "Apply move" })).toBeDisabled();
    expect(screen.queryByTestId("arm2d-sequence-path")).not.toBeInTheDocument();
  });

  it("preserves unrelated planning conflicts instead of treating them as floor routing", async () => {
    const { request, calls } = harness({
      planPreviewFailure: { status: 409, detail: "Shoulder moved after preview; the plan is stale." },
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "30" } });
    fireEvent.pointerUp(shoulder);

    expect(await screen.findByRole("alert")).toHaveTextContent("Shoulder moved after preview; the plan is stale.");
    expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(0);
    expect(screen.getByRole("button", { name: "Apply move" })).toBeDisabled();
  });

  it("keeps the arm idle and gives a concise error when every floor route is refused", async () => {
    const { request, calls } = harness({
      planPreviewFailure: { status: 409, detail: FLOOR_SWEEP_REFUSAL },
      sequencePreviewResponse: () => jsonResponse({
        detail: `Transition into waypoint 1 was rejected: ${FLOOR_SWEEP_REFUSAL}`,
      }, 409),
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "30" } });
    fireEvent.change(screen.getByRole("slider", { name: "Drive Elbow" }), { target: { value: "20" } });
    fireEvent.pointerUp(shoulder);

    const alert = await screen.findByRole("alert");
    expect(alert).toHaveTextContent("No floor-safe route was found. Adjust the destination and try again.");
    expect(alert).not.toHaveTextContent(FLOOR_SWEEP_REFUSAL);
    expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(9);
    expect(calls.filter((call) => call.url.endsWith("/plans/execute") || call.url.endsWith("/sequences/execute"))).toHaveLength(0);
    expect(screen.getByRole("button", { name: "Apply move" })).toBeDisabled();
  });

  it("builds, previews, and applies a reviewed waypoint sequence in one call each", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    fireEvent.change(base, { target: { value: "30" } });
    fireEvent.pointerUp(base);
    expect(calls.filter((call) => call.url.endsWith("/plans/preview"))).toHaveLength(0);
    expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(0);
    await user.click(screen.getByRole("button", { name: "Add waypoint" }));

    fireEvent.change(base, { target: { value: "-20" } });
    fireEvent.pointerUp(base);
    await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    expect(within(screen.getByLabelText("Waypoint sequence")).getAllByRole("listitem")).toHaveLength(2);

    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(1));
    expect(calls.find((call) => call.url.endsWith("/sequences/preview"))?.body).toEqual({
      waypoints: [
        { targets: { joint_1: 30, joint_2: 0, joint_3: 0, joint_4: 0 } },
        { targets: { joint_1: -20, joint_2: 0, joint_3: 0, joint_4: 0 } },
      ],
    });
    expect(screen.getByText("Sequence ready. Review the whole path, then apply all.")).toBeInTheDocument();
    expect(screen.getByTestId("arm2d-sequence-path")).toBeInTheDocument();

    await user.click(screen.getByRole("button", { name: "Apply all" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(1));
    expect(calls.find((call) => call.url.endsWith("/sequences/execute"))?.body).toEqual({
      sequenceId: "armseq_test_1234",
      sequenceDigest: `sha256:${"d".repeat(64)}`,
      arrivalTimeoutMs: 10_000,
    });
    expect(await screen.findByText("Completed 2 of 2 waypoints.")).toBeInTheDocument();
    expect(screen.getByText("Preview 12 ms")).toBeInTheDocument();
    expect(screen.getByText("Overall 1200 ms")).toBeInTheDocument();
    expect(screen.getByText("<1 ms dispatch · 300 ms arrival · 400 ms total")).toBeInTheDocument();
    expect(screen.getByText("110 ms dispatch · 310 ms arrival · 420 ms total")).toBeInTheDocument();
    expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(1);
  });

  it("invalidates the reviewed sequence after edit, reorder, removal, or clear", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await screen.findByText("Sequence ready. Review the whole path, then apply all.");
    expect(screen.getByRole("button", { name: "Apply all" })).toBeEnabled();

    await user.click(screen.getByRole("button", { name: "Move waypoint 2 up" }));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    expect(screen.queryByTestId("arm2d-sequence-path")).not.toBeInTheDocument();
    await user.click(screen.getByRole("button", { name: "Edit waypoint 1" }));
    fireEvent.change(base, { target: { value: "15" } });
    fireEvent.pointerUp(base);
    await user.click(screen.getByRole("button", { name: "Update waypoint" }));
    await user.click(screen.getByRole("button", { name: "Remove waypoint 2" }));
    expect(within(screen.getByLabelText("Waypoint sequence")).getAllByRole("listitem")).toHaveLength(1);
    expect(screen.getByRole("button", { name: "Preview all" })).toBeDisabled();
    await user.click(screen.getByRole("button", { name: "Clear all" }));
    expect(screen.queryByLabelText("Waypoint sequence")).not.toBeInTheDocument();
    expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(0);
  });

  it("inherits a newly reordered tail when the next waypoint is added", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }

    // [20, -10] -> [-10, 20], so the next draft must inherit Base=20.
    await user.click(screen.getByRole("button", { name: "Move waypoint 2 up" }));
    const shoulder = screen.getByRole("slider", { name: "Drive Shoulder" });
    fireEvent.change(shoulder, { target: { value: "15" } });
    fireEvent.pointerUp(shoulder);
    await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    await user.click(screen.getByRole("button", { name: "Preview all" }));

    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(1));
    const preview = calls.find((call) => call.url.endsWith("/sequences/preview"));
    expect((preview?.body as { waypoints: Array<{ targets: Record<string, number> }> }).waypoints[2]?.targets).toEqual({
      joint_1: 20,
      joint_2: 15,
      joint_3: 0,
      joint_4: 0,
    });
  });

  it("does not preview a stored route while an edited waypoint is unsaved", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }

    await user.click(screen.getByRole("button", { name: "Edit waypoint 1" }));
    expect(screen.getByRole("button", { name: "Preview all" })).toBeDisabled();
    fireEvent.change(base, { target: { value: "15" } });
    fireEvent.pointerUp(base);
    expect(screen.getByRole("button", { name: "Preview all" })).toBeDisabled();
    fireEvent.click(screen.getByRole("button", { name: "Preview all" }));
    expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(0);

    await user.click(screen.getByRole("button", { name: "Update waypoint" }));
    expect(screen.getByRole("button", { name: "Preview all" })).toBeEnabled();
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/sequences/preview"))).toHaveLength(1));
  });

  it("invalidates a reviewed sequence before floor, calibration, and servo ID changes", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }

    const previewAndExpectReady = async () => {
      await user.click(screen.getByRole("button", { name: "Preview all" }));
      await screen.findByText("Sequence ready. Review the whole path, then apply all.");
      expect(screen.getByRole("button", { name: "Apply all" })).toBeEnabled();
    };

    await previewAndExpectReady();
    await user.click(screen.getByRole("button", { name: "Floor guard on" }));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    expect(calls.filter((call) => call.url.endsWith("/floor-guard"))).toHaveLength(1);

    await previewAndExpectReady();
    type("Base speed", "600");
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/joints/joint_1/calibrate"))).toBe(true));

    await previewAndExpectReady();
    await user.click(screen.getByRole("button", { name: "Servo ID" }));
    await user.click(screen.getByRole("button", { name: "Write" }));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/servo-id"))).toHaveLength(1));
  }, 10_000);

  it("invalidates a reviewed sequence before torque or direct Base motion", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    const previewAndExpectReady = async () => {
      await user.click(screen.getByRole("button", { name: "Preview all" }));
      await screen.findByText("Sequence ready. Review the whole path, then apply all.");
      expect(screen.getByRole("button", { name: "Apply all" })).toBeEnabled();
    };

    await previewAndExpectReady();
    await user.click(screen.getByRole("button", { name: "Hold all" }));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/torque"))).toHaveLength(1));

    await previewAndExpectReady();
    const baseSetup = screen.getByLabelText("Base setup and test");
    await user.click(within(baseSetup).getByRole("button", { name: "0°" }));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/simple/target"))).toBe(true));
  }, 10_000);

  it("invalidates a reviewed sequence before reconnecting the controller", async () => {
    const user = userEvent.setup();
    const { request: normal, calls } = harness();
    let controllerFaulted = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (!init?.method && String(input).endsWith("/api/arm/simple/state") && controllerFaulted) {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, connection: "faulted" });
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await screen.findByText("Sequence ready. Review the whole path, then apply all.");
    expect(screen.getByRole("button", { name: "Apply all" })).toBeEnabled();

    controllerFaulted = true;
    expect(await screen.findByRole("button", { name: "Reconnect controller" })).toBeEnabled();
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await user.click(screen.getByRole("button", { name: "Reconnect controller" }));

    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/controller/reconnect"))).toHaveLength(1));
  });

  it("invalidates a reviewed sequence when STOP is pressed before apply", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await screen.findByText("Sequence ready. Review the whole path, then apply all.");
    expect(screen.getByRole("button", { name: "Apply all" })).toBeEnabled();

    await user.click(screen.getByRole("button", { name: "STOP" }));
    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/simple/stop"))).toHaveLength(1));
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    expect(screen.queryByTestId("arm2d-sequence-path")).not.toBeInTheDocument();
    expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(0);
  });

  it("keeps an externally stopped sequence invalid after Clear STOP", async () => {
    const user = userEvent.setup();
    const { request: normal, calls } = harness();
    let externallyStopped = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (!init?.method && String(input).endsWith("/api/arm/simple/state") && externallyStopped) {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, stopped: true });
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await screen.findByText("Sequence ready. Review the whole path, then apply all.");

    externallyStopped = true;
    expect(await screen.findByText(/STOP is latched/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    externallyStopped = false;
    await user.click(screen.getByRole("button", { name: "Clear STOP" }));
    await waitFor(() => expect(screen.queryByText(/STOP is latched/)).not.toBeInTheDocument());
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(0);
  });

  it("locks sequence editing while execution is pending, keeps STOP live, and shows the terminal partial receipt", async () => {
    const user = userEvent.setup();
    let settleExecution!: (response: Response) => void;
    const sequenceExecuteResponse = new Promise<Response>((resolve) => { settleExecution = resolve; });
    const { request: normal, calls } = harness({ sequenceExecuteResponse });
    let externallyStopped = false;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (!init?.method && String(input).endsWith("/api/arm/simple/state") && externallyStopped) {
        const response = await normal(input, init);
        const body = await response.json();
        return jsonResponse({ ...body, stopped: true });
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");
    await user.click(screen.getByRole("button", { name: "Sequence mode" }));
    const base = screen.getByRole("slider", { name: "Drive Base" });
    for (const target of [20, -10]) {
      fireEvent.change(base, { target: { value: String(target) } });
      fireEvent.pointerUp(base);
      await user.click(screen.getByRole("button", { name: "Add waypoint" }));
    }
    await user.click(screen.getByRole("button", { name: "Preview all" }));
    await screen.findByText("Sequence ready. Review the whole path, then apply all.");
    await user.click(screen.getByRole("button", { name: "Apply all" }));

    expect(await screen.findByText("Executing 2 reviewed waypoints on the Pi…")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Add waypoint" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Clear all" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();
    expect(screen.getByRole("button", { name: "Scan" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Floor guard on" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Hold all" })).toBeDisabled();
    expect(screen.queryByRole("button", { name: "Reconnect controller" })).not.toBeInTheDocument();
    expect(screen.getByRole("button", { name: "2D" })).toBeEnabled();
    expect(screen.getByRole("button", { name: "Arm tip, drag to move the whole arm" })).toHaveAttribute("aria-disabled", "true");
    expect(base).toBeDisabled();

    externallyStopped = true;
    expect(await screen.findByText(/STOP is latched/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Clear STOP" })).toBeDisabled();
    externallyStopped = false;

    settleExecution(jsonResponse({
      executed: true,
      sequenceId: "armseq_test_1234",
      operationId: "armop_test_1234",
      outcome: "stopped",
      waypointCount: 2,
      completedWaypointCount: 1,
      failedWaypointIndex: 1,
      waypointResults: [
        { index: 0, outcome: "arrived", dispatched: true, arrival: { proved: true }, moved: ["joint_1"] },
        { index: 1, outcome: "stopped", dispatched: false, arrival: { proved: false }, moved: [] },
      ],
      resolvedPose: { joint_1: 20 },
      finalMeasuredPose: { joint_1: 20 },
      durationMs: 800,
    }));
    expect(await screen.findByText("Stopped after 1 of 2 waypoints. Failed at waypoint 2.")).toBeInTheDocument();
    expect(screen.getByText("arrived")).toBeInTheDocument();
    expect(screen.getByText("stopped")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Apply all" })).toBeDisabled();
    expect(calls.filter((call) => call.url.endsWith("/sequences/execute"))).toHaveLength(1);
  }, 10_000);

  it("uses keyboard release to prepare, never to send a physical target", async () => {
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    const shoulder = await screen.findByRole("slider", { name: "Shoulder angle" });

    fireEvent.keyDown(shoulder, { key: "ArrowUp" });
    expect(calls.filter((call) => call.url.endsWith("/plans/preview"))).toHaveLength(0);
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
    fireEvent.keyUp(shoulder, { key: "ArrowUp" });

    await waitFor(() => expect(calls.filter((call) => call.url.endsWith("/plans/preview"))).toHaveLength(1));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
  });

  it("keeps the other joints stiff when one is freed", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness({ held: [1, 2, 3] });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getAllByRole("button", { name: "Free" })[0]);

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/simple/torque"))).toBe(true));
    // Everything but the freed joint. The gateway trims to what the controller
    // can actually hold and reports back which servos ended up energised.
    expect(calls.find((call) => call.url.endsWith("/simple/torque"))?.body).toEqual({ hold: [2, 3, 4] });
  });

  it("accepts a limit typed in degrees and one captured from the joint", async () => {
    // Both ways, every field: type a number, or press "here" to store wherever
    // the joint is standing.
    const user = userEvent.setup();
    const { request, calls } = harness({ baseRaw: 2300 });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    type("Base min", "-20");
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ minDegrees: -20 }));

    await user.click(screen.getByRole("button", { name: "Set Base max here" }));
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ here: ["rawMax"] }));

    type("Base zero", "2100");
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ rawZero: 2100 }));
  });

  it("centres the zero between min and max on request", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: /Centre Base zero/i }));

    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ zeroFromLimits: true }));
  });

  it("draws an L when the shoulder and the elbow both read zero", () => {
    // The twin measures the shoulder from horizontal and the elbow from
    // straight; the arm reads zero mid-travel on both. Both at zero is upper arm
    // up, forearm out — so the twin needs +90 and -90, or it draws a fold that
    // is not there and swings the forearm below the floor.
    const upright = { joint_1: 0, joint_2: toModel("joint_2", 0), joint_3: toModel("joint_3", 0) };
    const geometry = { baseHeightMm: 100, upperArmMm: 200, forearmMm: 150, toolOffsetMm: 0 };
    const tip = forwardKinematics(upright, geometry);

    // Straight up by the upper arm, then straight out by the forearm.
    expect(tip.z).toBeCloseTo(300, 3);
    expect(tip.x).toBeCloseTo(150, 3);
    expect(fromModel("joint_3", toModel("joint_3", 33))).toBeCloseTo(33, 6);
    expect(toModel("joint_1", 33)).toBe(33);
  });

  it("prepares a typed angle on Enter or go, but never moves or plans on blur", async () => {
    // Blur-to-commit is right for a calibration number and wrong for a move:
    // clicking somewhere else must not swing the arm.
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const field = screen.getByRole("spinbutton", { name: "Move Base to" });
    // Angles are shown to one decimal, so a fractional step grid puts the field
    // permanently off its own step and the browser then rejects every edit,
    // whole numbers included. Degrees take any value; only raw counts are whole.
    for (const name of ["Move Base to", "Base min", "Base max"]) {
      expect(screen.getByRole("spinbutton", { name })).toHaveAttribute("step", "any");
    }
    expect(screen.getByRole("spinbutton", { name: "Base zero" })).toHaveAttribute("step", "1");

    fireEvent.change(field, { target: { value: "-30" } });
    fireEvent.blur(field);
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);

    fireEvent.change(field, { target: { value: "-30" } });
    fireEvent.keyDown(field, { key: "Enter" });
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ targets: { joint_1: -30 } }));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);

    fireEvent.change(screen.getByRole("spinbutton", { name: "Move Elbow to" }), { target: { value: "12" } });
    await user.click(screen.getAllByRole("button", { name: "go" })[2]);
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ targets: expect.objectContaining({ joint_3: 12 }) }));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
  });

  it("opens on the flat view and prepares a bounded plan from its grips", async () => {
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const grip = screen.getByRole("slider", { name: "Base rotation" });
    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: 20, clientY: 110 });
    fireEvent.pointerUp(grip, { pointerId: 1 });

    // Zero is at the bottom of the dial, so the left of it is a quarter turn:
    // -90. This joint's calibrated limit is -87.9, and the limit is what goes
    // into the authoritative plan — release itself cannot command hardware.
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ targets: { joint_1: -87.9 } }));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);
  });

  it("gives the 3D view nothing to drag — it watches, the flat view drives", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "3D" }));

    expect(screen.queryByRole("button", { name: /Target handle/i })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Move target" })).not.toBeInTheDocument();
    // Orbiting is a view change, so it must never reach the arm.
    const canvas = screen.getByRole("application");
    fireEvent.pointerDown(canvas, { pointerId: 1, clientX: 100, clientY: 100 });
    fireEvent.pointerMove(canvas, { pointerId: 1, clientX: 180, clientY: 140 });
    expect(calls.some((call) => call.url.endsWith("/simple/target"))).toBe(false);
  });

  it("drives against what the servo can reach, and says so when that is less than the limits", async () => {
    // The real case: base at 8:1 with its zero 610 ticks from the end of the
    // motor's single turn. The limits say +/-180 and were accepted; the servo has
    // 45 degrees. Driving against the limits left most of the bar dead.
    const state = {
      connection: "online", bus: "online", held: [1], stopped: false,
      joints: [{
        ...joint("joint_1", "Base", 1, 2), rawZero: 610, ratio: 8, direction: -1,
        degrees: 6.7, minDegrees: -180, maxDegrees: 180, reachMin: -38.3, reachMax: 6.7,
      }],
    };
    const request = vi.fn<typeof fetch>(async (input) => {
      const url = String(input);
      if (url === "/api/session") return jsonResponse({ actionToken: "z".repeat(40) });
      return jsonResponse(state);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const bar = screen.getByRole("slider", { name: "Drive Base" });
    expect(bar).toHaveAttribute("min", "-38.3");
    expect(bar).toHaveAttribute("max", "6.7");
    expect(screen.getByText(/only spans 45°/)).toBeInTheDocument();
  });

  it("writes a servo ID, and warns that the bus must hold only that servo", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "Servo ID" }));
    // A fresh Waveshare servo is ID 1, the same as the base. Writing an ID is
    // addressed to the old one, so a populated bus makes it ambiguous.
    expect(screen.getByText(/Only one servo may be on the bus/)).toBeInTheDocument();

    fireEvent.change(screen.getByRole("spinbutton", { name: "Current servo id" }), { target: { value: "1" } });
    fireEvent.change(screen.getByRole("spinbutton", { name: "New servo id" }), { target: { value: "4" } });
    await user.click(screen.getByRole("button", { name: "Write" }));

    await waitFor(() => expect(calls.some((call) => call.url.endsWith("/simple/servo-id"))).toBe(true));
    expect(calls.at(-1)?.body).toEqual({ oldId: 1, newId: 4 });
  });

  it("points a joint at a different servo without burning an ID into anything", async () => {
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    type("Base servo id", "4");

    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ servoId: 4 }));
    expect(calls.at(-1)?.url).toContain("/joints/joint_1/calibrate");
  });

  it("gives the camera a full joint card and its own dial, but keeps it out of the arm", async () => {
    const user = userEvent.setup();
    const { request, calls } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    // A joint card like any other: calibratable, drivable, its own servo id.
    const panel = within(screen.getByRole("tabpanel", { name: "Joints" }));
    expect(panel.getByText("Camera")).toBeInTheDocument();
    expect(screen.getByRole("slider", { name: "Drive Camera" })).toBeInTheDocument();
    expect(screen.getByRole("spinbutton", { name: "Camera zero" })).toBeInTheDocument();

    // Its own dial in the flat view, alongside the base's.
    const dial = screen.getByRole("slider", { name: "Camera" });
    fireEvent.pointerDown(dial, { pointerId: 1 });
    fireEvent.pointerMove(dial, { pointerId: 1, clientX: 20, clientY: 110 });
    fireEvent.pointerUp(dial, { pointerId: 1 });
    // No model offset: the camera is not in the chain, so its dial degrees are
    // the servo's degrees, unlike the shoulder and elbow.
    await waitFor(() => expect(calls.at(-1)?.body).toEqual({ targets: { joint_4: -87.9 } }));
    expect(calls.filter((call) => call.url.endsWith("/simple/target"))).toHaveLength(0);

    // ...and the twin never draws it.
    await user.click(screen.getByRole("button", { name: "3D" }));
    expect(screen.queryByRole("slider", { name: "Camera" })).not.toBeInTheDocument();
  });

  it("names an id collision instead of showing an unexplained empty panel", async () => {
    // Two servos answering one id and nothing being plugged in look identical
    // from the browser. The controller can tell them apart, so the panel must.
    const state = {
      connection: "online", bus: "faulted", held: [], stopped: false,
      collisionSuspected: true, collisionId: 1,
      joints: [joint("joint_1", "Base", 1, 2300)],
    };
    const request = vi.fn<typeof fetch>(async (input) => {
      if (String(input) === "/api/session") return jsonResponse({ actionToken: "z".repeat(40) });
      return jsonResponse(state);
    });
    render(<SimpleArm request={request} />);

    expect(await screen.findByText(/Two servos are answering to id 1/)).toBeInTheDocument();
    expect(screen.getByText(/ships as id 1, the same as/)).toBeInTheDocument();
  });

  it("renders a faulted REAL bus as unavailable without inventing a duplicate id or live pose", async () => {
    const { request } = harness({
      bus: "faulted",
      // A stale Pi may still send the old, unsupported diagnosis. The browser
      // must normalize it until the Pi package is deployed too.
      busTrouble: "The servo bus is answering with corrupt frames — the signature of two servos sharing an id and talking over each other.",
      allJointsOffline: true,
      positionTrusted: false,
      multiTurnTruth: {
        tracking: false,
        valid: false,
        stepMode: false,
        stepOutstanding: false,
        countdownObserved: false,
        resyncNeeded: false,
        resyncCount: 0,
      },
    });
    render(<SimpleArm request={request} />);

    expect(await screen.findByText(/Servo-bus communication failed/)).toBeInTheDocument();
    expect(screen.queryByText(/two servos sharing an id/i)).not.toBeInTheDocument();
    expect(screen.getByText(/controller online · bus faulted · 0\/4 joints live/i)).toBeInTheDocument();

    const truth = screen.getByLabelText("Base multi-turn truth");
    expect(truth).toHaveTextContent("BASE NOT DETECTED");
    expect(truth).not.toHaveTextContent("HOME REQUIRED");

    expect(screen.getByRole("status", { name: "Live arm pose unavailable" })).toHaveTextContent(/waiting for fresh servo telemetry/i);
    expect(screen.queryByLabelText("Flat arm control")).not.toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Hold all" })).toBeDisabled();
    for (const button of screen.getAllByRole("button", { name: "Hold" })) {
      expect(button).toBeDisabled();
    }
    expect(screen.getByRole("button", { name: "Scan" })).toBeEnabled();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();
  });

  it("shows the bounded gateway reason when Scan fails", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ allJointsOffline: true });
    const detail = "The Pi gateway answered, but its ESP32 controller link returned an invalid or incomplete response. Check the HAT UART connection, then reconnect the controller.";
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (String(input) === "/api/arm/simple/scan") {
        return jsonResponse({ detail }, 502);
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "Scan" }));

    expect(await screen.findByText(`Scan failed: ${detail}`, { selector: ".simplearm-note" })).toBeInTheDocument();
  });

  it("keeps Scan single-flight and disables the button until its census returns", async () => {
    const gate = deferred<void>();
    const { request: normal } = harness({ allJointsOffline: true });
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (String(input) === "/api/arm/simple/scan") {
        await gate.promise;
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    const scan = screen.getByRole("button", { name: "Scan" });
    fireEvent.click(scan);
    fireEvent.click(scan);

    expect(await screen.findByRole("button", { name: "Scanning…" })).toBeDisabled();
    expect(request.mock.calls.filter(([input]) => String(input) === "/api/arm/simple/scan")).toHaveLength(1);

    gate.resolve();
    expect(await screen.findByRole("button", { name: "Scan" })).toBeEnabled();
  });

  it("reports the servo inventory when Scan succeeds without treating it as pose proof", async () => {
    const user = userEvent.setup();
    const { request: normal } = harness({ allJointsOffline: true });
    const request = vi.fn<typeof fetch>(async (input, init) => {
      if (String(input) === "/api/arm/simple/scan") {
        const response = await normal(input, init);
        const state = await response.json();
        return jsonResponse({
          ...state,
          scan: {
            foundIds: [1, 2, 3, 4],
            completeRange: { minId: 0, maxId: 10 },
            collisionSuspected: false,
          },
        });
      }
      return normal(input, init);
    });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    await user.click(screen.getByRole("button", { name: "Scan" }));

    expect(await screen.findByText("Scan complete: servo IDs 1, 2, 3, 4 answered.", { selector: ".simplearm-note" })).toBeInTheDocument();
    expect(screen.getByRole("status", { name: "Live arm pose unavailable" })).toBeInTheDocument();
  });

  it("stays quiet about collisions on a healthy bus", async () => {
    const { request } = harness();
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    expect(screen.queryByText(/Two servos are answering/)).not.toBeInTheDocument();
  });

  it("does not offer to drive a joint that has no zero yet", async () => {
    const { request } = harness({ calibrated: false });
    render(<SimpleArm request={request} />);
    await screen.findByText("Base");

    expect(screen.getByRole("slider", { name: "Drive Base" })).toBeDisabled();
    expect(screen.getByRole("slider", { name: "Drive Shoulder" })).toBeEnabled();
  });
});
