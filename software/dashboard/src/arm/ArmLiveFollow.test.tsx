import axe from "axe-core";
import { act, fireEvent, render, screen, waitFor, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import { ArmLiveFollow } from "./ArmLiveFollow";

vi.mock("./Arm2D", () => ({
  Arm2D: ({ disabled, showDials, interactionCopy, limits, readouts, onPose, onInteractionEnd }: {
    disabled?: boolean;
    showDials?: boolean;
    interactionCopy?: string;
    limits?: Record<string, { min: number; max: number }>;
    readouts?: Partial<Record<"joint_1" | "joint_2" | "joint_3", number | null>>;
    onPose: (pose: { joint_2: number; joint_3: number }) => void;
    onInteractionEnd?: (reason: "release" | "cancel") => void;
  }) => (
    <div
      data-testid="arm2d-live"
      data-show-dials={String(showDials)}
      data-shoulder-min={limits?.joint_2?.min}
      data-shoulder-max={limits?.joint_2?.max}
      data-base-readout={readouts?.joint_1 === null ? "n/a" : String(readouts?.joint_1)}
    >
      <p>{interactionCopy}</p>
      <button type="button" disabled={disabled} onClick={() => onPose({ joint_2: 100, joint_3: -100 })}>Target one</button>
      <button type="button" disabled={disabled} onClick={() => onPose({ joint_2: 105, joint_3: -105 })}>Target two</button>
      <button type="button" disabled={disabled} onClick={() => onPose({ joint_2: 110, joint_3: -110 })}>Target three</button>
      <button type="button" onClick={() => onPose({ joint_2: 115, joint_3: -115 })}>Late target</button>
      <button type="button" disabled={disabled} onClick={() => {
        for (let index = 0; index < 100; index += 1) onPose({ joint_2: 90 + index / 10, joint_3: -90 - index / 10 });
      }}>Burst 100 targets</button>
      <button type="button" disabled={disabled} onClick={() => onInteractionEnd?.("release")}>Release drag</button>
      <button type="button" disabled={disabled} onClick={() => onInteractionEnd?.("cancel")}>Cancel drag</button>
    </div>
  ),
}));

function response(body: unknown, status = 200) {
  return new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });
}

const armState = {
  connection: "online",
  bus: "online",
  controller: { firmwareVersion: "arm-hat-2.7.0", liveFollowV1: true },
  stopped: false,
  held: [2, 3],
  joints: [
    { id: "joint_1", name: "Base", servoId: 1, rawZero: 2048, rawMin: 0, rawMax: 4095, ratio: 1, direction: 1, speed: 800, accel: 40, online: true, rawPosition: 2048, degrees: 0, positionTrusted: true, minDegrees: -180, maxDegrees: 180, reachMin: -180, reachMax: 180, rawLow: 0, rawHigh: 4095, calibrated: true, torque: "on", temperatureC: 30, voltageVolts: 12 },
    { id: "joint_2", name: "Shoulder", servoId: 2, rawZero: 2048, rawMin: 0, rawMax: 4095, ratio: 1, direction: 1, speed: 800, accel: 40, online: true, rawPosition: 2048, degrees: 0, moving: false, positionTrusted: true, minDegrees: -90, maxDegrees: 90, reachMin: -90, reachMax: 90, rawLow: 0, rawHigh: 4095, calibrated: true, torque: "on", temperatureC: 30, voltageVolts: 12 },
    { id: "joint_3", name: "Elbow", servoId: 3, rawZero: 2048, rawMin: 0, rawMax: 4095, ratio: 1, direction: 1, speed: 800, accel: 40, online: true, rawPosition: 2048, degrees: 0, moving: false, positionTrusted: true, minDegrees: -120, maxDegrees: 120, reachMin: -120, reachMax: 120, rawLow: 0, rawHigh: 4095, calibrated: true, torque: "on", temperatureC: 30, voltageVolts: 12 },
  ],
  floorGuard: { enabled: true, floorMm: 40, geometry: { baseHeightMm: 86, upperArmMm: 220, distalMm: 235 } },
};

function liveReceipt(overrides: Record<string, unknown> = {}) {
  return {
    schema: "arm-live-follow-v2",
    sessionId: "live_frontend",
    state: "active",
    reason: null,
    settings: {
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    },
    envelope: {
      maxDeltaDegrees: { joint_2: 30, joint_3: 30 },
      floorMm: 40,
      lowestPointMm: 80,
      joint_2: { minDegrees: 0, maxDegrees: 0 },
      joint_3: { minDegrees: 0, maxDegrees: 0 },
    },
    commanded: { joint_2: 0, joint_3: 0 },
    measured: {
      joint_2: { degrees: 0, rawPosition: 2048, velocityDegreesPerSecond: 0, moving: false, packetAgeMs: 8 },
      joint_3: { degrees: 0, rawPosition: 2048, velocityDegreesPerSecond: 0, moving: false, packetAgeMs: 8 },
    },
    stats: { receivedFrames: 0, dispatchedFrames: 0, coalescedFrames: 0, clampedFrames: 0, lastAcceptedSequence: 0, lastDispatchedSequence: 0, lastDispatchLatencyMs: null },
    timing: { inputLeaseMs: 400, maxDurationMs: 30_000, minDispatchIntervalMs: 50, startedAt: "2026-08-21T00:00:00Z", expiresAt: "2026-08-21T00:00:30Z" },
    ...overrides,
  };
}

function liveStartReceipt(init?: RequestInit, overrides: Record<string, unknown> = {}) {
  const body = JSON.parse(String(init?.body)) as { startAttemptId?: unknown };
  return liveReceipt({ ...overrides, startAttemptId: body.startAttemptId });
}

function defaultRequest() {
  return vi.fn<typeof fetch>(async (input, init) => {
    const url = String(input);
    if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
    if (url.endsWith("/state") || url.endsWith("/stop")) return response(armState);
    if (url.endsWith("/start")) {
      const settings = JSON.parse(String(init?.body)) as ReturnType<typeof liveReceipt>["settings"] & { startAttemptId: string };
      return response(liveReceipt({
        startAttemptId: settings.startAttemptId,
        settings,
        envelope: {
          ...liveReceipt().envelope,
          maxDeltaDegrees: {
            joint_2: settings.joint_2.maxDeltaDegrees,
            joint_3: settings.joint_3.maxDeltaDegrees,
          },
        },
      }));
    }
    if (url.endsWith("/frame")) return response(liveReceipt());
    if (url.endsWith("/heartbeat")) return response(liveReceipt());
    if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
    throw new Error(`Unexpected request: ${url}`);
  });
}

afterEach(() => {
  vi.useRealTimers();
  vi.restoreAllMocks();
});

describe("ArmLiveFollow", () => {
  it("is REAL-only, keeps the planar surface visible, and marks STOP unavailable on SIM", async () => {
    const request = defaultRequest();
    render(<ArmLiveFollow request={request} backendId="sim" />);

    expect(await screen.findByText(/Speed Lab is REAL-only/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP" })).toHaveAttribute("title", expect.stringMatching(/only when REAL/));
    expect(screen.getByTestId("arm2d-live")).toHaveAttribute("data-show-dials", "false");
  });

  it("gates Enable on firmware 2.7 Fast Follow feedback capability", async () => {
    const unsupported = { ...armState, controller: { firmwareVersion: "arm-hat-2.6.0", liveFollowV1: false } };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(unsupported);
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmLiveFollow request={request} backendId="real" />);

    expect(await screen.findByText(/Controller firmware 2.7\+/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeDisabled();
    expect(request.mock.calls.some(([url]) => String(url).endsWith("/start"))).toBe(false);
  });

  it("clearly reports an unknown Base reference without inventing 0 or blocking Shoulder and Elbow", async () => {
    const baseUnknown = {
      ...armState,
      joints: armState.joints.map((joint) => joint.id === "joint_1" ? {
        ...joint,
        rawPosition: null,
        degrees: null,
        positionTrusted: false,
      } : joint),
    };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(baseUnknown);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmLiveFollow request={request} backendId="real" />);

    const alert = await screen.findByRole("alert");
    expect(alert).toHaveTextContent("BASE REFERENCE UNKNOWN");
    expect(alert).toHaveTextContent("does not show or assume 0°");
    expect(alert).toHaveTextContent("Live remains Shoulder + Elbow only");
    expect(alert).toHaveTextContent("Set zero here");
    expect(screen.getByTestId("arm2d-live")).toHaveAttribute("data-base-readout", "n/a");

    const enable = screen.getByRole("button", { name: "Enable live control" });
    expect(enable).toBeEnabled();
    fireEvent.click(enable);
    await waitFor(() => expect(request.mock.calls.some(([url]) => String(url).endsWith("/start"))).toBe(true));
  });

  it("bounds a hung start and releases ownership only after the exact attempt is tombstoned", async () => {
    vi.useFakeTimers();
    let startAborts = 0;
    let resolveCancel!: (value: Response) => void;
    const cancelled = new Promise<Response>((resolve) => { resolveCancel = resolve; });
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start/cancel")) return await cancelled;
      if (url.endsWith("/start")) {
        return await new Promise<Response>((_resolve, reject) => {
          init?.signal?.addEventListener("abort", () => {
            startAborts += 1;
            reject(new DOMException("The operation was aborted.", "AbortError"));
          }, { once: true });
        });
      }
      throw new Error(`Unexpected request: ${url}`);
    });
    const onBusy = vi.fn();
    render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    const start = request.mock.calls.find(([url]) => String(url).endsWith("/live-follow/start"));
    const startBody = JSON.parse(String(start?.[1]?.body)) as { startAttemptId?: string; startTimeoutMs?: number };
    expect(startBody.startAttemptId).toEqual(expect.any(String));
    expect(startBody.startTimeoutMs).toBe(1_500);
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_499); });
    expect(startAborts).toBe(0);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/start/cancel"))).toHaveLength(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => { await vi.advanceTimersByTimeAsync(1); });
    const cancel = request.mock.calls.find(([url]) => String(url).endsWith("/start/cancel"));
    expect(startAborts).toBe(1);
    expect(JSON.parse(String(cancel?.[1]?.body))).toEqual({ startAttemptId: startBody.startAttemptId });
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => {
      resolveCancel(response({
        schema: "arm-live-follow-v2",
        startAttemptId: startBody.startAttemptId,
        cancelled: true,
        inputLeaseMs: 400,
      }));
      await vi.advanceTimersByTimeAsync(0);
    });

    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();
    expect(onBusy).toHaveBeenLastCalledWith(false);
    await act(async () => { await vi.advanceTimersByTimeAsync(10_000); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/live-follow/start"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/start/cancel"))).toHaveLength(1);
  });

  it("labels, bounds, clamps, and locks exact per-joint session settings", async () => {
    const request = defaultRequest();
    const user = userEvent.setup();
    render(<ArmLiveFollow request={request} backendId="real" />);

    await waitFor(() => expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled());
    expect(screen.getByText(/Temporary raw ST3215 settings/)).toBeInTheDocument();
    expect(screen.getByText(/never rewrite calibration/)).toBeInTheDocument();
    expect(screen.getByText(/±90° is only the Speed Lab software ceiling/)).toBeInTheDocument();
    expect(screen.getByText(/not obstacle, cable, self-contact, or overshoot sensing/)).toBeInTheDocument();
    expect(screen.getAllByText("1 to 2400 raw")).toHaveLength(2);
    expect(screen.getAllByText("1 to 50 raw · servo limit")).toHaveLength(2);

    const shoulderSpeed = screen.getByRole("spinbutton", { name: "Shoulder speed, raw ST3215 units" });
    const shoulderAccel = screen.getByRole("spinbutton", { name: "Shoulder acceleration, raw ST3215 units" });
    const shoulderTravel = screen.getByRole("spinbutton", { name: "Shoulder session travel, degrees from start" });
    const elbowSpeed = screen.getByRole("spinbutton", { name: "Elbow speed, raw ST3215 units" });
    const elbowAccel = screen.getByRole("spinbutton", { name: "Elbow acceleration, raw ST3215 units" });
    const elbowTravel = screen.getByRole("spinbutton", { name: "Elbow session travel, degrees from start" });
    expect(shoulderSpeed).toHaveValue(400);
    expect(shoulderAccel).toHaveValue(10);
    expect(shoulderTravel).toHaveValue(30);
    expect(elbowSpeed).toHaveValue(400);
    expect(elbowAccel).toHaveValue(10);
    expect(elbowTravel).toHaveValue(30);

    const typeAndCommit = (input: HTMLElement, value: string) => {
      fireEvent.change(input, { target: { value } });
      fireEvent.blur(input);
    };
    typeAndCommit(shoulderSpeed, "0");
    typeAndCommit(shoulderAccel, "0");
    typeAndCommit(shoulderTravel, "0");
    typeAndCommit(elbowSpeed, "2401");
    typeAndCommit(elbowAccel, "51");
    typeAndCommit(elbowTravel, "91");
    expect(shoulderSpeed).toHaveValue(1);
    expect(shoulderAccel).toHaveValue(1);
    expect(shoulderTravel).toHaveValue(1);
    expect(elbowSpeed).toHaveValue(2400);
    expect(elbowAccel).toHaveValue(50);
    expect(elbowTravel).toHaveValue(90);
    fireEvent.change(shoulderSpeed, { target: { value: "777" } });
    fireEvent.keyDown(shoulderSpeed, { key: "Enter" });
    expect(shoulderSpeed).toHaveValue(777);

    typeAndCommit(shoulderSpeed, "1234");
    typeAndCommit(shoulderAccel, "17.6");
    typeAndCommit(shoulderTravel, "44");
    typeAndCommit(elbowSpeed, "2345");
    typeAndCommit(elbowAccel, "49");
    typeAndCommit(elbowTravel, "89");
    expect(shoulderAccel).toHaveValue(18);

    await user.click(screen.getByRole("button", { name: "Enable live control" }));
    await screen.findByText(/LIVE: release flushes/);
    const start = request.mock.calls.find(([url]) => String(url).endsWith("/start"));
    expect(JSON.parse(String(start?.[1]?.body))).toEqual(expect.objectContaining({
      joint_2: { speed: 1234, accel: 18, maxDeltaDegrees: 44 },
      joint_3: { speed: 2345, accel: 49, maxDeltaDegrees: 89 },
      startAttemptId: expect.any(String),
      startTimeoutMs: 1_500,
    }));
    expect(shoulderSpeed).toBeDisabled();
    expect(elbowAccel).toBeDisabled();
    expect(screen.getByRole("button", { name: "Shoulder speed preset 800" })).toBeDisabled();
    expect(screen.getByText("Settings locked for this session")).toBeInTheDocument();
  });

  it("opens the first drag to the requested start-relative span, not the zero-width visited envelope", async () => {
    const request = defaultRequest();
    const user = userEvent.setup();
    render(<ArmLiveFollow request={request} backendId="real" />);

    await waitFor(() => expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled());
    await user.click(screen.getByRole("button", { name: "Enable live control" }));
    const surface = await screen.findByTestId("arm2d-live");
    const minimum = Number(surface.getAttribute("data-shoulder-min"));
    const maximum = Number(surface.getAttribute("data-shoulder-max"));
    expect(maximum).toBeGreaterThan(minimum);
    expect(maximum - minimum).toBeCloseTo(60, 5);

    await user.click(screen.getByRole("button", { name: "Target one" }));
    await waitFor(() => expect(request.mock.calls.some(([url]) => String(url).endsWith("/frame"))).toBe(true));
  });

  it("keeps one frame in flight, overwrites one pending target, and flushes it while the run stays live", async () => {
    const firstFrame = { resolve: null as ((value: Response) => void) | null };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state") || url.endsWith("/stop")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) {
        const frameCalls = request.mock.calls.filter(([candidate]) => String(candidate).endsWith("/frame")).length;
        if (frameCalls === 1) return await new Promise<Response>((resolve) => { firstFrame.resolve = resolve; });
        return response(liveReceipt({ stats: { ...liveReceipt().stats, receivedFrames: 2, dispatchedFrames: 2, lastAcceptedSequence: 2, lastDispatchedSequence: 2 } }));
      }
      if (url.endsWith("/end")) return response(liveReceipt({
        state: "ended",
        stats: { ...liveReceipt().stats, receivedFrames: 2, dispatchedFrames: 2, lastAcceptedSequence: 2, lastDispatchedSequence: 2 },
      }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const user = userEvent.setup();
    const onBusy = vi.fn();
    render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await waitFor(() => expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled());
    await user.click(screen.getByRole("button", { name: "Enable live control" }));
    await screen.findByText(/LIVE: release flushes/);
    await user.click(screen.getByRole("button", { name: "Target one" }));
    await user.click(screen.getByRole("button", { name: "Target two" }));
    await user.click(screen.getByRole("button", { name: "Target three" }));
    await user.click(screen.getByRole("button", { name: "Release drag" }));

    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(screen.getByText("HTTP frames").parentElement).toHaveTextContent("HTTP frames1");
    firstFrame.resolve?.(response(liveReceipt({ stats: { ...liveReceipt().stats, receivedFrames: 1, dispatchedFrames: 1, lastAcceptedSequence: 1, lastDispatchedSequence: 1 } })));

    await waitFor(() => expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame")).length).toBeGreaterThanOrEqual(2));
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(0);
    const frames = request.mock.calls.filter(([url]) => String(url).endsWith("/frame"));
    const frameBodies = frames.map(([, init]) => JSON.parse(String(init?.body)) as { sequence: number; joint_2: number; joint_3: number });
    expect(frameBodies.map((body) => body.sequence)).toEqual(frameBodies.map((_, index) => index + 1));
    expect(frameBodies.some((body) => body.joint_2 === 15 || body.joint_3 === -15)).toBe(false);
    expect(frameBodies.at(-1)).toMatchObject({ joint_2: 20, joint_3: -20 });
    await user.click(screen.getByRole("button", { name: "End session" }));
    await waitFor(() => expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1));
    const manualEnd = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(manualEnd?.[1]?.body))).toMatchObject({ flushPending: true });
    expect(screen.getByText("400 / 400", { selector: ".arm-live-follow-history td" })).toBeInTheDocument();
    expect(onBusy).toHaveBeenCalledWith(true);
    expect(onBusy).toHaveBeenLastCalledWith(false);
  });

  it("discards a pending target on cancel and always routes STOP through the action token", async () => {
    const request = defaultRequest();
    const user = userEvent.setup();
    render(<ArmLiveFollow request={request} backendId="real" />);

    await waitFor(() => expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled());
    await user.click(screen.getByRole("button", { name: "Enable live control" }));
    await user.click(screen.getByRole("button", { name: "Cancel drag" }));
    await waitFor(() => expect(request.mock.calls.some(([url]) => String(url).endsWith("/end"))).toBe(true));
    const frames = request.mock.calls.filter(([url]) => String(url).endsWith("/frame"));
    expect(frames.every(([, init]) => {
      const body = JSON.parse(String(init?.body)) as { joint_2: number; joint_3: number };
      return body.joint_2 === 0 && body.joint_3 === 0;
    })).toBe(true);
    const cancelEnd = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(cancelEnd?.[1]?.body))).toMatchObject({ flushPending: false });

    await user.click(screen.getByRole("button", { name: "STOP" }));
    await waitFor(() => expect(request.mock.calls.some(([url]) => String(url).endsWith("/stop"))).toBe(true));
    const stopCall = request.mock.calls.find(([url]) => String(url).endsWith("/stop"));
    expect(stopCall?.[1]?.headers).toEqual(expect.objectContaining({ "X-Co-Arm-Token": "t".repeat(40) }));
  });

  it("keeps control ownership fail-closed through the full superseded-session window when STOP hangs", async () => {
    vi.useFakeTimers();
    let stopAborts = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/stop")) {
        return await new Promise<Response>((_resolve, reject) => {
          init?.signal?.addEventListener("abort", () => {
            stopAborts += 1;
            reject(new DOMException("The operation was aborted.", "AbortError"));
          }, { once: true });
        });
      }
      throw new Error(`Unexpected request: ${url}`);
    });
    const onBusy = vi.fn();
    render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();

    fireEvent.click(screen.getByRole("button", { name: "STOP" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/stop"))).toHaveLength(1);
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_201); });
    expect(stopAborts).toBe(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(4_548); });
    expect(stopAborts).toBe(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(1); });

    expect(stopAborts).toBe(1);
    expect(screen.getByText(/STOP confirmation did not arrive.*safety lease window/i)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();
    expect(onBusy).toHaveBeenLastCalledWith(false);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/stop"))).toHaveLength(1);
  });

  it("keeps the full ownership window when STOP supersedes an unconfirmed terminal end", async () => {
    vi.useFakeTimers();
    let endAborts = 0;
    let stopAborts = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/end")) {
        return await new Promise<Response>((_resolve, reject) => {
          init?.signal?.addEventListener("abort", () => {
            endAborts += 1;
            reject(new DOMException("The operation was aborted.", "AbortError"));
          }, { once: true });
        });
      }
      if (url.endsWith("/stop")) {
        return await new Promise<Response>((_resolve, reject) => {
          init?.signal?.addEventListener("abort", () => {
            stopAborts += 1;
            reject(new DOMException("The operation was aborted.", "AbortError"));
          }, { once: true });
        });
      }
      throw new Error(`Unexpected request: ${url}`);
    });
    const onBusy = vi.fn();
    render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "End session" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);

    fireEvent.click(screen.getByRole("button", { name: "STOP" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(endAborts).toBe(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/stop"))).toHaveLength(1);
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_201); });
    expect(stopAborts).toBe(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(4_548); });
    expect(stopAborts).toBe(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(1); });

    expect(stopAborts).toBe(1);
    expect(onBusy).toHaveBeenLastCalledWith(false);
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/stop"))).toHaveLength(1);
  });

  it("ignores a stale STOP completion after watchdog recovery and a later run starts", async () => {
    vi.useFakeTimers();
    let resolveStop!: (value: Response) => void;
    const delayedStop = new Promise<Response>((resolve) => { resolveStop = resolve; });
    let startCount = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) {
        startCount += 1;
        return response(liveStartReceipt(init, { sessionId: `live_${startCount}` }));
      }
      if (url.endsWith("/heartbeat")) return response(liveReceipt({ sessionId: `live_${startCount}` }));
      if (url.endsWith("/stop")) return await delayedStop;
      throw new Error(`Unexpected request: ${url}`);
    });
    const onBusy = vi.fn();
    render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "STOP" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(1_201); });
    expect(screen.getByRole("button", { name: "End session" })).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(4_548); });
    expect(screen.getByRole("button", { name: "End session" })).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(1); });
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();

    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(startCount).toBe(2);
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => {
      resolveStop(response({ ...armState, stopped: true }));
      await vi.advanceTimersByTimeAsync(0);
    });

    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    expect(screen.queryByText("STOP latched. Live follow ended.")).not.toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/stop"))).toHaveLength(1);
  });

  it("discards a rate-limited pending target on window blur and ends without flushing", async () => {
    vi.useFakeTimers();
    const firstFrame = { resolve: null as ((value: Response) => void) | null };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) return await new Promise<Response>((resolve) => { firstFrame.resolve = resolve; });
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    fireEvent.click(screen.getByRole("button", { name: "Target two" }));
    act(() => { globalThis.dispatchEvent(new Event("blur")); });
    await act(async () => {
      firstFrame.resolve?.(response(liveReceipt()));
      await vi.advanceTimersByTimeAsync(200);
    });

    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    const end = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(end?.[1]?.body))).toMatchObject({ flushPending: false });
    view.unmount();
  });

  it("closes input immediately on manual End and flushes only the target already pending", async () => {
    vi.useFakeTimers();
    const firstFrame = { resolve: null as ((value: Response) => void) | null };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) {
        const frameCalls = request.mock.calls.filter(([candidate]) => String(candidate).endsWith("/frame")).length;
        if (frameCalls === 1) return await new Promise<Response>((resolve) => { firstFrame.resolve = resolve; });
        return response(liveReceipt({
          stats: { ...liveReceipt().stats, receivedFrames: 2, dispatchedFrames: 2, lastAcceptedSequence: 2, lastDispatchedSequence: 2 },
        }));
      }
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    fireEvent.click(screen.getByRole("button", { name: "Target two" }));
    fireEvent.click(screen.getByRole("button", { name: "End session" }));
    fireEvent.click(screen.getByRole("button", { name: "Late target" }));

    await act(async () => {
      firstFrame.resolve?.(response(liveReceipt({
        stats: { ...liveReceipt().stats, receivedFrames: 1, dispatchedFrames: 1, lastAcceptedSequence: 1, lastDispatchedSequence: 1 },
      })));
      await vi.advanceTimersByTimeAsync(100);
    });

    const frames = request.mock.calls.filter(([url]) => String(url).endsWith("/frame"));
    expect(frames).toHaveLength(2);
    const frameBodies = frames.map(([, init]) => JSON.parse(String(init?.body)) as { sequence: number; joint_2: number; joint_3: number });
    expect(frameBodies).toEqual([
      expect.objectContaining({ sequence: 1, joint_2: 10, joint_3: -10 }),
      expect.objectContaining({ sequence: 2, joint_2: 15, joint_3: -15 }),
    ]);
    expect(frameBodies.some((body) => body.joint_2 === 25 || body.joint_3 === -25)).toBe(false);
    const end = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(end?.[1]?.body))).toMatchObject({ flushPending: true });
    view.unmount();
  });

  it("downgrades an ambiguous terminal-frame timeout to one no-flush end without dispatching the pending frame", async () => {
    vi.useFakeTimers();
    let resolveFrame!: (value: Response) => void;
    const frameSignals: AbortSignal[] = [];
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) {
        if (init?.signal) frameSignals.push(init.signal);
        return await new Promise<Response>((resolve) => { resolveFrame = resolve; });
      }
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    fireEvent.click(screen.getByRole("button", { name: "Target two" }));
    fireEvent.click(screen.getByRole("button", { name: "End session" }));

    await act(async () => { await vi.advanceTimersByTimeAsync(200); });
    expect(frameSignals[0]?.aborted).toBe(true);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(0);

    await act(async () => {
      resolveFrame(response(liveReceipt()));
      await vi.advanceTimersByTimeAsync(0);
    });

    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    const endCalls = request.mock.calls.filter(([url]) => String(url).endsWith("/end"));
    expect(endCalls).toHaveLength(1);
    expect(JSON.parse(String(endCalls[0]?.[1]?.body))).toEqual({ sessionId: "live_frontend", flushPending: false });
    view.unmount();
  });

  it("closes input immediately on cancel and never sends the pending or later target", async () => {
    vi.useFakeTimers();
    const firstFrame = { resolve: null as ((value: Response) => void) | null };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) return await new Promise<Response>((resolve) => { firstFrame.resolve = resolve; });
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    fireEvent.click(screen.getByRole("button", { name: "Target two" }));
    fireEvent.click(screen.getByRole("button", { name: "Cancel drag" }));
    fireEvent.click(screen.getByRole("button", { name: "Late target" }));

    await act(async () => {
      firstFrame.resolve?.(response(liveReceipt()));
      await vi.advanceTimersByTimeAsync(100);
    });

    const frames = request.mock.calls.filter(([url]) => String(url).endsWith("/frame"));
    expect(frames).toHaveLength(1);
    expect(JSON.parse(String(frames[0]?.[1]?.body))).toMatchObject({ sequence: 1, joint_2: 10, joint_3: -10 });
    const end = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(end?.[1]?.body))).toMatchObject({ flushPending: false });
    view.unmount();
  });

  it("keeps the lease alive when one motion frame stalls beyond the 400 ms deadman", async () => {
    vi.useFakeTimers();
    const firstFrame = { resolve: null as ((value: Response) => void) | null };
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/frame")) {
        const frameCalls = request.mock.calls.filter(([candidate]) => String(candidate).endsWith("/frame")).length;
        if (frameCalls === 1) return await new Promise<Response>((resolve) => { firstFrame.resolve = resolve; });
        return response(liveReceipt());
      }
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));

    await act(async () => { await vi.advanceTimersByTimeAsync(625); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat")).length).toBeGreaterThanOrEqual(4);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(0);
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();

    await act(async () => {
      firstFrame.resolve?.(response(liveReceipt()));
      await vi.advanceTimersByTimeAsync(0);
    });

    fireEvent.click(screen.getByRole("button", { name: "End session" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    view.unmount();
    vi.useRealTimers();
  });

  it("shows a heartbeat deadline failure, ends once, and never retries ambiguous input", async () => {
    vi.useFakeTimers();
    let heartbeatAborts = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) {
        return await new Promise<Response>((_resolve, reject) => {
          init?.signal?.addEventListener("abort", () => {
            heartbeatAborts += 1;
            reject(new DOMException("The operation was aborted.", "AbortError"));
          }, { once: true });
        });
      }
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      if (url.endsWith("/frame")) return response(liveReceipt());
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();

    await act(async () => { await vi.advanceTimersByTimeAsync(350); });
    expect(screen.getByText(/Live heartbeat failed: Live heartbeat timed out after 200 ms/i)).toBeInTheDocument();
    expect(heartbeatAborts).toBe(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(0);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_000); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    view.unmount();
  });

  it("treats a heartbeat 409 as terminal instead of retrying the lease", async () => {
    vi.useFakeTimers();
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response({ detail: "The live-follow session no longer owns the controller." }, 409);
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    await act(async () => { await vi.advanceTimersByTimeAsync(125); });

    expect(screen.getByText(/no longer owns the controller/i)).toBeInTheDocument();
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);

    await act(async () => { await vi.advanceTimersByTimeAsync(1_000); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    view.unmount();
  });

  it("ends independently when heartbeat fails while a motion frame never settles", async () => {
    vi.useFakeTimers();
    const frameSignals: AbortSignal[] = [];
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/frame")) {
        if (init?.signal) frameSignals.push(init.signal);
        return await new Promise<Response>(() => undefined);
      }
      if (url.endsWith("/heartbeat")) return response({ detail: "Heartbeat ownership was lost." }, 409);
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(125); });

    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    expect(frameSignals).toHaveLength(1);
    expect(frameSignals[0]?.aborted).toBe(true);
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();

    await act(async () => { await vi.advanceTimersByTimeAsync(1_000); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    view.unmount();
  });

  it("retains ownership for the full input chain and ignores late old heartbeat, frame, and end receipts", async () => {
    vi.useFakeTimers();
    let startCount = 0;
    let resolveHeartbeat!: (response: Response) => void;
    let resolveFrame!: (response: Response) => void;
    let resolveEnd!: (response: Response) => void;
    const heartbeat = new Promise<Response>((resolve) => { resolveHeartbeat = resolve; });
    const frame = new Promise<Response>((resolve) => { resolveFrame = resolve; });
    const end = new Promise<Response>((resolve) => { resolveEnd = resolve; });
    const frameSignals: AbortSignal[] = [];
    let endAborts = 0;
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) {
        startCount += 1;
        return response(liveStartReceipt(init, { sessionId: startCount === 1 ? "live_old" : "live_new" }));
      }
      if (url.endsWith("/heartbeat")) return await heartbeat;
      if (url.endsWith("/frame")) {
        if (init?.signal) frameSignals.push(init.signal);
        return await frame;
      }
      if (url.endsWith("/end")) {
        return await new Promise<Response>((resolve) => {
          init?.signal?.addEventListener("abort", () => {
            endAborts += 1;
          }, { once: true });
          void end.then(resolve);
        });
      }
      throw new Error(`Unexpected request: ${url}`);
    });
    const onBusy = vi.fn();
    const view = render(<ArmLiveFollow request={request} backendId="real" onControlBusyChange={onBusy} />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Target one" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(125); });
    await act(async () => { await vi.advanceTimersByTimeAsync(200); });

    expect(frameSignals).toHaveLength(1);
    expect(frameSignals[0]?.aborted).toBe(true);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    const firstEnd = request.mock.calls.find(([url]) => String(url).endsWith("/end"));
    expect(JSON.parse(String(firstEnd?.[1]?.body))).toMatchObject({ sessionId: "live_old", flushPending: false });
    expect(screen.getByRole("button", { name: "End session" })).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);

    await act(async () => { await vi.advanceTimersByTimeAsync(901); });
    expect(screen.getByRole("button", { name: "End session" })).toBeInTheDocument();
    expect(endAborts).toBe(0);
    expect(onBusy).toHaveBeenLastCalledWith(true);
    await act(async () => { await vi.advanceTimersByTimeAsync(4_848); });
    expect(screen.getByRole("button", { name: "End session" })).toBeInTheDocument();
    expect(endAborts).toBe(0);
    await act(async () => { await vi.advanceTimersByTimeAsync(1); });

    expect(endAborts).toBe(1);
    expect(screen.getByText(/local controls recovered after the full unconfirmed input-chain safety window/i)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Enable live control" })).toBeEnabled();
    expect(onBusy).toHaveBeenLastCalledWith(false);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);

    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(startCount).toBe(2);
    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);

    resolveHeartbeat(response(liveReceipt({ sessionId: "live_old", state: "stopped", reason: "stale heartbeat" })));
    resolveFrame(response(liveReceipt({ sessionId: "live_old", state: "stopped", reason: "stale frame" })));
    resolveEnd(response(liveReceipt({ sessionId: "live_old", state: "stopped", reason: "stale end" })));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });

    expect(screen.getByText(/LIVE: release flushes/)).toBeInTheDocument();
    expect(onBusy).toHaveBeenLastCalledWith(true);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/heartbeat"))).toHaveLength(1);
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/end"))).toHaveLength(1);
    view.unmount();
  });

  it("derives Approx peak across repeated cached polls instead of only the final 50 ms", async () => {
    vi.useFakeTimers();
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) {
        const shoulderDegrees = performance.now() >= 150 ? 9 : 0;
        return response({
          ...armState,
          joints: armState.joints.map((joint) => joint.id === "joint_2" ? { ...joint, degrees: shoulderDegrees } : joint),
        });
      }
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/frame") || url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    await act(async () => { await vi.advanceTimersByTimeAsync(150); });

    const shoulderRow = screen.getByRole("row", { name: /Shoulder/ });
    expect(within(shoulderRow).getByText("60°/s")).toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "End session" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(screen.getByText("60 / 0", { selector: ".arm-live-follow-history td" })).toBeInTheDocument();
    view.unmount();
  });

  it("caps a 100-sample pointer burst at 20 Hz and sends only the newest pending target", async () => {
    vi.useFakeTimers();
    const frameStarts: number[] = [];
    const request = vi.fn<typeof fetch>(async (input, init) => {
      const url = String(input);
      if (url === "/api/session") return response({ actionToken: "t".repeat(40) });
      if (url.endsWith("/state")) return response(armState);
      if (url.endsWith("/start")) return response(liveStartReceipt(init));
      if (url.endsWith("/heartbeat")) return response(liveReceipt());
      if (url.endsWith("/frame")) {
        frameStarts.push(performance.now());
        return response(liveReceipt());
      }
      if (url.endsWith("/end")) return response(liveReceipt({ state: "ended" }));
      throw new Error(`Unexpected request: ${url}`);
    });
    const view = render(<ArmLiveFollow request={request} backendId="real" />);

    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Enable live control" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    fireEvent.click(screen.getByRole("button", { name: "Burst 100 targets" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);

    await act(async () => { await vi.advanceTimersByTimeAsync(49); });
    expect(request.mock.calls.filter(([url]) => String(url).endsWith("/frame"))).toHaveLength(1);
    await act(async () => { await vi.advanceTimersByTimeAsync(1); });

    const frames = request.mock.calls.filter(([url]) => String(url).endsWith("/frame"));
    expect(frames).toHaveLength(2);
    expect(frameStarts[1]! - frameStarts[0]!).toBeGreaterThanOrEqual(50);
    expect(JSON.parse(String(frames[1]?.[1]?.body))).toMatchObject({ sequence: 2 });
    expect(JSON.parse(String(frames[1]?.[1]?.body)).joint_2).toBeCloseTo(9.9, 5);
    expect(JSON.parse(String(frames[1]?.[1]?.body)).joint_3).toBeCloseTo(-9.9, 5);

    fireEvent.click(screen.getByRole("button", { name: "Cancel drag" }));
    await act(async () => { await vi.advanceTimersByTimeAsync(0); });
    view.unmount();
  });

  it("has no detectable axe violations in the disabled SIM state", async () => {
    const { container } = render(<ArmLiveFollow request={defaultRequest()} backendId="sim" />);
    await screen.findByText(/Speed Lab is REAL-only/);
    expect((await axe.run(container)).violations).toEqual([]);
  });
});
