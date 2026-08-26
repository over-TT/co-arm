import axe from "axe-core";
import { render, screen, waitFor, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { beforeEach, describe, expect, it, vi } from "vitest";
import App from "../App";
import { ARM_BACKEND_SESSION_KEY, ArmControlCenter } from "./ArmControlCenter";

const busyCallbackHistory = vi.hoisted(() => ({
  live: [] as Array<((busy: boolean) => void) | undefined>,
}));

vi.mock("./SimpleArm", () => ({
  SimpleArm: ({ request, onControlBusyChange }: { request: typeof fetch; onControlBusyChange?: (busy: boolean) => void }) => (
    <div data-testid="simple-arm-surface">
      Existing SimpleArm surface
      <button type="button" onClick={() => void request("/api/arm/simple/state")}>Probe selected backend</button>
      <button type="button" onClick={() => onControlBusyChange?.(true)}>Begin mocked motion</button>
      <button type="button" onClick={() => onControlBusyChange?.(false)}>Finish mocked motion</button>
      <button type="button">STOP</button>
    </div>
  ),
}));

vi.mock("./ArmLiveFollow", () => ({
  ArmLiveFollow: ({ request, onControlBusyChange, backendId }: { request: typeof fetch; onControlBusyChange?: (busy: boolean) => void; backendId: "real" | "sim" }) => {
    busyCallbackHistory.live.push(onControlBusyChange);
    return (
      <div data-testid="live-follow-surface" data-backend-id={backendId}>
        <h1>Shoulder + Elbow live follow</h1>
        <button type="button" onClick={() => void request("/api/arm/simple/live-follow/start")}>Probe Live backend</button>
        <button type="button" onClick={() => onControlBusyChange?.(true)}>Begin live session</button>
        <button type="button" onClick={() => onControlBusyChange?.(false)}>Finish live session</button>
        <button type="button">STOP Live</button>
      </div>
    );
  },
}));

function jsonResponse(body: unknown, status = 200) {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}

const backends = {
  backends: [
    {
      backendId: "sim",
      backendInstanceId: "sim-instance-7",
      displayName: "Isaac Simulator",
      simulated: true,
      configured: true,
    },
    {
      backendId: "real",
      backendInstanceId: "real-instance-2",
      displayName: "Desk Arm",
      simulated: false,
      configured: true,
    },
  ],
  defaultBackendId: "real",
  selectionScope: "request",
};

const manifest = {
  schema: "arm-control-center.manifest.v1",
  generatedAt: "2026-08-18T12:30:00Z",
  status: "ready",
  project: {
    name: "Arm Alliance",
    sourceIndex: {
      status: "ready",
      canonicalEntry: "software/SOURCE_INDEX.json",
      entryCount: 45,
      entries: [
        { id: "sharedMcp", path: "software/python/arm_mcp/server.py" },
        { id: "simulation.readme", path: "software/python/arm_sim/README.md" },
      ],
    },
  },
  simulation: {
    status: "provisional",
    model: { status: "ready", description: "Four-joint model and mounted camera." },
    latestResult: { status: "passed", passed: true },
  },
  checks: { passed: 1, total: 1, label: "Latest simulator frame validation" },
  calibration: { completed: 1, total: 2, label: "Physical measurement jobs" },
  actions: [
    {
      id: "open_isaac_scene",
      label: "Open Isaac scene",
      description: "Open the latest saved desk scene.",
      available: true,
      runtime: { status: "idle" },
    },
  ],
  evidenceBoundary: { physicalGatewayAccessed: false, physicalArmMotion: false, physicalProof: false },
};

function createRequest() {
  return vi.fn(async (input: RequestInfo | URL, init?: RequestInit) => {
    const url = String(input);
    if (url === "/api/arm/backends") return jsonResponse(backends);
    if (url === "/api/arm/control-center/manifest") return jsonResponse(manifest);
    if (url === "/api/session") return jsonResponse({ actionToken: "control-center-action-token-000001" });
    if (url === "/api/arm/control-center/actions/open_isaac_scene") return jsonResponse({ status: "started" });
    if (url === "/api/arm/simple/state") return jsonResponse({ ok: true });
    throw new Error(`Unexpected request: ${url} ${init?.method ?? "GET"}`);
  });
}

beforeEach(() => {
  busyCallbackHistory.live.length = 0;
  sessionStorage.clear();
  window.history.replaceState(null, "", "/");
});

describe("Arm Control Center", () => {
  it("keeps the Live ownership callback stable when ownership itself rerenders the shell", async () => {
    const user = userEvent.setup();
    window.history.replaceState(null, "", "/#arm/live");
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    await screen.findByTestId("live-follow-surface");
    const before = busyCallbackHistory.live.at(-1);
    expect(before).toBeTypeOf("function");

    await user.click(screen.getByRole("button", { name: "Begin live session" }));
    expect(screen.getByRole("status")).toHaveTextContent("Active live follow is bound to sim");
    expect(busyCallbackHistory.live.at(-1)).toBe(before);
  });

  it("defaults to SIM and persists an explicit REAL selection for this browser session", async () => {
    const user = userEvent.setup();
    const requestMock = createRequest();
    const { unmount } = render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("Isaac Simulator")).toBeInTheDocument();
    expect(screen.getAllByText("Configured · not probed").length).toBeGreaterThan(0);
    expect(screen.getByRole("button", { name: "Use simulator" })).toHaveAttribute("aria-pressed", "true");
    const environment = screen.getByRole("region", { name: "Isaac Simulator" });
    expect(within(environment).getByText("sim-instance-7")).toBeInTheDocument();
    expect(within(environment).getByText("Local Isaac simulation")).toBeInTheDocument();

    await user.click(screen.getByRole("button", { name: "Use real arm" }));

    expect(screen.getByRole("button", { name: "Use real arm" })).toHaveAttribute("aria-pressed", "true");
    expect(await screen.findByText("Desk Arm")).toBeInTheDocument();
    expect(JSON.parse(sessionStorage.getItem(ARM_BACKEND_SESSION_KEY) ?? "null")).toEqual({ id: "real", mode: "real" });

    unmount();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);
    expect(await screen.findByText("Desk Arm")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Use real arm" })).toHaveAttribute("aria-pressed", "true");
  });

  it("shows only a disabled placeholder while backend inventory is still loading", async () => {
    let resolveInventory!: (response: Response) => void;
    const inventory = new Promise<Response>((resolve) => { resolveInventory = resolve; });
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return inventory;
      if (url === "/api/arm/control-center/manifest") return jsonResponse(manifest);
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(screen.getByRole("button", { name: "Arm backends loading" })).toBeDisabled();
    expect(screen.queryByRole("button", { name: "Use simulator" })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Use real arm" })).not.toBeInTheDocument();
    expect(screen.getByRole("tab", { name: "Control" })).toBeDisabled();

    resolveInventory(jsonResponse(backends));
    expect(await screen.findByRole("button", { name: "Use simulator" })).toBeEnabled();
  });

  it("keeps controls unconfigured and disabled when backend inventory fails", async () => {
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return jsonResponse({ detail: "Backend registry could not load" }, 503);
      if (url === "/api/arm/control-center/manifest") return jsonResponse(manifest);
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("Backend inventory offline: Backend registry could not load. Controls remain disabled.")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "No arm backend configured" })).toBeDisabled();
    expect(screen.queryByRole("button", { name: "Use simulator" })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Use real arm" })).not.toBeInTheDocument();
    expect(screen.getByRole("tab", { name: "Control" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Arm control unavailable" })).toBeDisabled();
  });

  it("recovers a stale stored backend through a backend-neutral inventory request", async () => {
    sessionStorage.setItem(ARM_BACKEND_SESSION_KEY, JSON.stringify({ id: "retired-sim", mode: "sim" }));
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("Isaac Simulator")).toBeInTheDocument();
    const inventoryCall = requestMock.mock.calls.find(([input]) => String(input) === "/api/arm/backends");
    expect(new Headers(inventoryCall?.[1]?.headers).has("X-Arm-Backend-Id")).toBe(false);
    await waitFor(() => expect(JSON.parse(sessionStorage.getItem(ARM_BACKEND_SESSION_KEY) ?? "null")).toEqual({ id: "sim", mode: "sim" }));
  });

  it("renders only the backends that the server reports as configured", async () => {
    const realOnly = { ...backends, backends: [backends.backends[1]] };
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return jsonResponse(realOnly);
      if (url === "/api/arm/control-center/manifest") return jsonResponse(manifest);
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("Desk Arm")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Use real arm" })).toHaveAttribute("aria-pressed", "true");
    expect(screen.queryByRole("button", { name: "Use simulator" })).not.toBeInTheDocument();
    expect(JSON.parse(sessionStorage.getItem(ARM_BACKEND_SESSION_KEY) ?? "null")).toEqual({ id: "real", mode: "real" });
  });

  it("does not turn an empty successful inventory into fabricated SIM or REAL targets", async () => {
    const user = userEvent.setup();
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return jsonResponse({ backends: [] });
      if (url === "/api/arm/control-center/manifest") return jsonResponse(manifest);
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("No backend configured")).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Use simulator" })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Use real arm" })).not.toBeInTheDocument();
    expect(screen.getByRole("button", { name: "No arm backend configured" })).toBeDisabled();
    expect(screen.getByText("No arm backend is configured. Add SIM or REAL in the local service before opening controls.")).toBeInTheDocument();
    expect(screen.getByRole("tab", { name: "Control" })).toBeDisabled();
    expect(screen.getByRole("tab", { name: "Live" })).toBeDisabled();
    expect(screen.getByRole("tab", { name: "Guide" })).toBeEnabled();

    await user.click(screen.getByRole("tab", { name: "Guide" }));
    expect(screen.getByRole("heading", { name: "Arm reference" })).toBeInTheDocument();
  });

  it("opens direct tab URLs and updates the URL when navigation changes", async () => {
    const user = userEvent.setup();
    window.history.replaceState(null, "", "/#arm/guide");
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(screen.getByRole("tab", { name: "Guide" })).toHaveAttribute("aria-selected", "true");
    expect(screen.getByRole("heading", { name: "Arm reference" })).toBeInTheDocument();
    await screen.findByRole("button", { name: "Use simulator" });

    await user.click(screen.getByRole("tab", { name: "Live" }));
    expect(window.location.hash).toBe("#arm/live");
    expect(screen.getByRole("heading", { name: "Shoulder + Elbow live follow" })).toBeInTheDocument();

    const liveTab = screen.getByRole("tab", { name: "Live" });
    liveTab.focus();
    await user.keyboard("{ArrowLeft}");
    expect(screen.getByRole("tab", { name: "Control" })).toHaveAttribute("aria-selected", "true");
    expect(screen.getByRole("tab", { name: "Control" })).toHaveFocus();
    expect(window.location.hash).toBe("#arm/control");
  });

  it("replaces legacy removed routes with the arm overview", async () => {
    for (const legacyRoute of ["retired", "unknown"]) {
      window.history.replaceState(null, "", `/#arm/${legacyRoute}`);
      const requestMock = createRequest();
      const view = render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

      expect(screen.getByRole("tab", { name: "Overview" })).toHaveAttribute("aria-selected", "true");
      await waitFor(() => expect(window.location.hash).toBe("#arm/overview"));
      view.unmount();
    }
  });

  it("reuses SimpleArm and binds every control request to the selected backend", async () => {
    const user = userEvent.setup();
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);
    await screen.findByText("Isaac Simulator");

    await user.click(screen.getByRole("tab", { name: "Control" }));
    expect(screen.getByTestId("simple-arm-surface")).toHaveTextContent("Existing SimpleArm surface");
    expect(screen.getByText("SIMULATOR")).toBeInTheDocument();
    await user.click(screen.getByRole("button", { name: "Probe selected backend" }));

    await waitFor(() => {
      const probe = requestMock.mock.calls.find(([input]) => String(input) === "/api/arm/simple/state");
      expect(new Headers(probe?.[1]?.headers).get("X-Arm-Backend-Id")).toBe("sim");
    });

    await user.click(screen.getByRole("button", { name: "Use real arm" }));
    expect(screen.getByText("REAL ARM")).toBeInTheDocument();
    await user.click(screen.getByRole("button", { name: "Probe selected backend" }));

    await waitFor(() => {
      const probes = requestMock.mock.calls.filter(([input]) => String(input) === "/api/arm/simple/state");
      expect(new Headers(probes.at(-1)?.[1]?.headers).get("X-Arm-Backend-Id")).toBe("real");
    });
  });

  it("opens the Live tab on its direct URL, binds requests, and locks its owner session", async () => {
    const user = userEvent.setup();
    window.history.replaceState(null, "", "/#arm/live");
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(screen.getByRole("tab", { name: "Live" })).toHaveAttribute("aria-selected", "true");
    await waitFor(() => expect(screen.getByTestId("live-follow-surface")).toHaveAttribute("data-backend-id", "sim"));
    await user.click(screen.getByRole("button", { name: "Probe Live backend" }));
    await waitFor(() => {
      const probe = requestMock.mock.calls.find(([input]) => String(input) === "/api/arm/simple/live-follow/start");
      expect(new Headers(probe?.[1]?.headers).get("X-Arm-Backend-Id")).toBe("sim");
    });

    await user.click(screen.getByRole("button", { name: "Begin live session" }));
    expect(screen.getByRole("tab", { name: "Live" })).toBeEnabled();
    expect(screen.getByRole("tab", { name: "Control" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Use real arm" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP Live" })).toBeEnabled();
    expect(screen.getByRole("status")).toHaveTextContent("Active live follow is bound to sim");
    expect(screen.getByRole("status")).toHaveTextContent("press STOP in Live");

    window.history.pushState(null, "", "/#arm/overview");
    window.dispatchEvent(new HashChangeEvent("hashchange"));
    await waitFor(() => expect(window.location.hash).toBe("#arm/live"));
    await user.click(screen.getByRole("button", { name: "Finish live session" }));
    expect(screen.getByRole("tab", { name: "Overview" })).toBeEnabled();
  });

  it("locks backend and page switching while any bound manual control is active", async () => {
    const user = userEvent.setup();
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);
    await screen.findByText("Isaac Simulator");
    await user.click(screen.getByRole("tab", { name: "Control" }));

    await user.click(screen.getByRole("button", { name: "Begin mocked motion" }));

    expect(screen.getByRole("button", { name: "Use simulator" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Use real arm" })).toBeDisabled();
    expect(screen.getByRole("tab", { name: "Overview" })).toBeDisabled();
    expect(screen.getByRole("tab", { name: "Guide" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Open Isaac Sim" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();
    expect(screen.getByRole("status")).toHaveTextContent("Active control is bound to sim");
    expect(screen.getByRole("status")).toHaveTextContent("press STOP in Control");

    await user.click(screen.getByRole("button", { name: "Finish mocked motion" }));
    expect(screen.getByRole("button", { name: "Use real arm" })).toBeEnabled();
    expect(screen.getByRole("tab", { name: "Overview" })).toBeEnabled();
    expect(screen.getByRole("button", { name: "Open Isaac Sim" })).toBeEnabled();
  });

  it("restores Control when hash or browser-history navigation tries to unmount an active motion", async () => {
    const user = userEvent.setup();
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);
    await screen.findByText("Isaac Simulator");
    await user.click(screen.getByRole("tab", { name: "Control" }));
    await user.click(screen.getByRole("button", { name: "Begin mocked motion" }));

    window.history.pushState(null, "", "/#arm/overview");
    window.dispatchEvent(new HashChangeEvent("hashchange"));
    await waitFor(() => expect(window.location.hash).toBe("#arm/control"));
    expect(screen.getByRole("tab", { name: "Control" })).toHaveAttribute("aria-selected", "true");
    expect(screen.getByTestId("simple-arm-surface")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();

    window.history.pushState(null, "", "/#arm/guide");
    window.dispatchEvent(new PopStateEvent("popstate"));
    await waitFor(() => expect(window.location.hash).toBe("#arm/control"));
    expect(screen.getByTestId("simple-arm-surface")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "STOP" })).toBeEnabled();
  });

  it("promotes one Isaac scene CTA and runs it against the selected SIM identity", async () => {
    const user = userEvent.setup();
    const requestMock = createRequest();
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    const isaacButton = await screen.findByRole("button", { name: "Open Isaac Sim" });
    expect(screen.getAllByRole("button", { name: "Open Isaac Sim" })).toHaveLength(1);
    expect(screen.queryByRole("group", { name: "Project launch actions" })).not.toBeInTheDocument();
    expect(screen.queryByText("Everything for the arm, in one place.")).not.toBeInTheDocument();

    await user.click(isaacButton);
    expect(await screen.findByText("Isaac Sim is opening in a separate window.")).toBeInTheDocument();
    const actionCall = requestMock.mock.calls.find(([input]) => String(input).endsWith("/actions/open_isaac_scene"));
    expect(actionCall?.[1]?.method).toBe("POST");
    expect(new Headers(actionCall?.[1]?.headers).get("X-Arm-Backend-Id")).toBe("sim");
    expect(new Headers(actionCall?.[1]?.headers).get("X-Co-Arm-Token")).toBe("control-center-action-token-000001");
    expect(actionCall?.[1]?.body).toBeUndefined();
  });

  it("keeps the promoted Isaac scene action visible but unavailable when the manifest omits it", async () => {
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return jsonResponse(backends);
      if (url === "/api/arm/control-center/manifest") return jsonResponse({ ...manifest, actions: [] });
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByRole("button", { name: "Isaac unavailable" })).toBeDisabled();
    expect(screen.queryByRole("button", { name: "Open Isaac Sim" })).not.toBeInTheDocument();
  });

  it("polls an active action runtime until a delayed failure and stderr tail are visible", async () => {
    let manifestCalls = 0;
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url === "/api/arm/backends") return jsonResponse(backends);
      if (url === "/api/arm/control-center/manifest") {
        manifestCalls += 1;
        const failed = manifestCalls >= 3;
        return jsonResponse({
          ...manifest,
          actions: [{
            ...manifest.actions[0],
            runtime: failed
              ? { status: "failed", lastExitCode: 1, lastError: "Isaac startup crashed\nGPU context unavailable" }
              : { status: "running", pid: 4412 },
          }],
        });
      }
      throw new Error(`Unexpected request: ${url}`);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("RUNNING")).toBeInTheDocument();
    await waitFor(() => expect(screen.getByText(/Isaac startup crashed/)).toBeInTheDocument(), { timeout: 4_000 });
    expect(screen.getByText("FAILED")).toBeInTheDocument();
    expect(screen.getByText("EXIT 1")).toBeInTheDocument();
    expect(manifestCalls).toBeGreaterThanOrEqual(3);
  });

  it("keeps a useful manual visible when the live manifest is offline", async () => {
    const user = userEvent.setup();
    const requestMock = vi.fn(async (input: RequestInfo | URL) => {
      if (String(input) === "/api/arm/backends") return jsonResponse(backends);
      return jsonResponse({ detail: "Manifest bridge is not running" }, 503);
    });
    render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByText("Manifest unavailable.")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Open control" })).toBeEnabled();
    await user.click(screen.getByRole("button", { name: "Guide" }));
    expect(screen.getByText("Real-arm stack")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: /Control the simulated arm/i })).toBeEnabled();
    expect(screen.getByText(/open the repository root in Codex/i)).toBeInTheDocument();
    expect(screen.getByText("docs/SETUP_WITH_CODEX.md")).toBeInTheDocument();
  });

  it("has no automated accessibility violations on the overview", async () => {
    const requestMock = createRequest();
    const { container } = render(<ArmControlCenter request={requestMock as unknown as typeof fetch} />);
    await screen.findByText("Isaac Simulator");
    const results = await axe.run(container);
    expect(results.violations).toEqual([]);
  });

  it("mounts the Control Center directly from an arm URL in the standalone app", async () => {
    window.history.replaceState(null, "", "/#arm/guide");
    const requestMock = createRequest();
    render(<App request={requestMock as unknown as typeof fetch} />);

    expect(await screen.findByRole("heading", { name: "Arm reference" })).toBeInTheDocument();
    expect(document.querySelector(".arm-dashboard-shell")).toBeInTheDocument();
    expect(requestMock.mock.calls.some(([input]) => String(input) === "/api/devices")).toBe(false);
  });
});
