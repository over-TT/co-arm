import { useCallback, useEffect, useMemo, useRef, useState, type KeyboardEvent, type ReactNode } from "react";
import { loadActionToken } from "./armApi";
import { ArmLiveFollow } from "./ArmLiveFollow";
import { SimpleArm } from "./SimpleArm";
import "./arm-control-center.css";

export type ArmBackendMode = "sim" | "real";
export type ArmControlCenterTab = "overview" | "control" | "live" | "guide";
export type ArmControlOwner = "control" | "live" | null;
export type ArmBackendStatus = "ready" | "starting" | "offline" | "error" | "configured" | "unknown";

export interface ArmBackendDescriptor {
  id: string;
  mode: ArmBackendMode;
  displayName: string;
  identity: string;
  description?: string;
  status: ArmBackendStatus;
  simulated: boolean;
  version?: string;
}

export interface ArmControlCenterAction {
  id: string;
  label: string;
  description: string;
  enabled: boolean;
  tone?: "primary" | "neutral";
  modes?: ArmBackendMode[];
  runtime?: {
    status: string;
    pid?: number;
    startedAt?: string;
    finishedAt?: string;
    lastExitCode?: number;
    lastError?: string;
  };
}

export interface ArmBuildItem {
  id: string;
  label: string;
  description: string;
  status: "ready" | "partial" | "planned" | "unknown";
  detail?: string;
  path?: string;
}

export interface ArmControlCenterManifest {
  schemaVersion?: string;
  generatedAt?: string;
  projectName?: string;
  summary?: string;
  checks?: {
    passed: number;
    total: number;
    label?: string;
  };
  calibration?: {
    completed: number;
    total: number;
    label?: string;
  };
  buildItems: ArmBuildItem[];
  actions: ArmControlCenterAction[];
  sourceEntries: Array<{ label: string; path: string }>;
}

export interface ArmControlCenterProps {
  request?: typeof fetch;
}

interface BackendPreference {
  id: string | null;
  mode: ArmBackendMode;
}

const defaultRequest: typeof fetch = (...arguments_) => globalThis.fetch(...arguments_);

export const ARM_BACKEND_SESSION_KEY = "arm-control-center.backend.v1";

const TABS: Array<{ id: ArmControlCenterTab; label: string }> = [
  { id: "overview", label: "Overview" },
  { id: "control", label: "Control" },
  { id: "live", label: "Live" },
  { id: "guide", label: "Guide" },
];

const ISAAC_SCENE_ACTION_ID = "open_isaac_scene";

function handleTabKeyDown(event: KeyboardEvent<HTMLButtonElement>) {
  if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
  const tabs = Array.from(
    event.currentTarget.parentElement?.querySelectorAll<HTMLButtonElement>('[role="tab"]:not(:disabled)') ?? [],
  );
  const currentIndex = tabs.indexOf(event.currentTarget);
  if (currentIndex < 0 || tabs.length === 0) return;

  event.preventDefault();
  const nextIndex = event.key === "Home"
    ? 0
    : event.key === "End"
      ? tabs.length - 1
      : (currentIndex + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) % tabs.length;
  tabs[nextIndex]?.focus();
  tabs[nextIndex]?.click();
}

const UNCONFIGURED_BACKEND: ArmBackendDescriptor = {
  id: "__unconfigured__",
  mode: "sim",
  displayName: "No backend configured",
  identity: "Configure SIM or REAL in the local service",
  description: "The backend inventory returned no configured arm targets.",
  status: "offline",
  simulated: true,
};

const FALLBACK_BUILD_ITEMS: ArmBuildItem[] = [
  {
    id: "arm-stack",
    label: "Real-arm stack",
    description: "Dashboard, laptop service, Pi gateway, ESP32 controller, and measured telemetry.",
    status: "unknown",
    path: "software/SOURCE_INDEX.json",
  },
  {
    id: "sim-lab",
    label: "Isaac arm simulation",
    description: "Four-joint digital twin, mounted camera, and a local SIM backend for arm setup and control.",
    status: "unknown",
    path: "software/python/arm_sim/README.md",
  },
  {
    id: "tool-contract",
    label: "Shared arm tools",
    description: "The same state, scene, camera, plan, sequence, and apply vocabulary in either mode.",
    status: "unknown",
    path: "software/python/arm_mcp/server.py",
  },
];

function isRecord(value: unknown): value is Record<string, unknown> {
  return Boolean(value) && typeof value === "object" && !Array.isArray(value);
}

function text(value: unknown, fallback = "") {
  return typeof value === "string" && value.trim() ? value.trim() : fallback;
}

function finiteNumber(value: unknown, fallback = 0) {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function backendMode(value: unknown): ArmBackendMode | null {
  if (value === "sim" || value === "simulator") return "sim";
  if (value === "real" || value === "physical") return "real";
  return null;
}

function backendStatus(value: unknown): ArmBackendStatus {
  if (value === "faulted") return "error";
  return ["ready", "starting", "offline", "error", "configured"].includes(String(value))
    ? value as ArmBackendStatus
    : "unknown";
}

function normalizeBackends(value: unknown): ArmBackendDescriptor[] {
  const root = isRecord(value) ? value : {};
  const candidates = Array.isArray(root.backends) ? root.backends : Array.isArray(value) ? value : [];
  const normalized = candidates.flatMap((candidate): ArmBackendDescriptor[] => {
    if (!isRecord(candidate)) return [];
    const id = text(candidate.id ?? candidate.backendId);
    const mode = backendMode(candidate.mode ?? candidate.backendMode)
      ?? (candidate.simulated === true ? "sim" : candidate.simulated === false ? "real" : backendMode(id));
    if (!mode || !id) return [];
    return [{
      id,
      mode,
      displayName: text(candidate.displayName ?? candidate.label, mode === "sim" ? "Simulator" : "Real arm"),
      identity: text(candidate.identity ?? candidate.backendInstanceId ?? candidate.endpointLabel, id),
      description: text(candidate.description) || undefined,
      status: backendStatus(candidate.status ?? candidate.health ?? (candidate.configured === true ? "configured" : undefined)),
      simulated: typeof candidate.simulated === "boolean" ? candidate.simulated : mode === "sim",
      version: text(candidate.version) || undefined,
    }];
  });
  return normalized;
}

function buildStatus(value: unknown): ArmBuildItem["status"] {
  return ["ready", "partial", "planned"].includes(String(value))
    ? value as ArmBuildItem["status"]
    : "unknown";
}

function normalizeManifest(value: unknown): ArmControlCenterManifest {
  const root = isRecord(value) ? value : {};
  const project = isRecord(root.project) ? root.project : {};
  const simulation = isRecord(root.simulation) ? root.simulation : {};
  const sourceIndex = isRecord(project.sourceIndex) ? project.sourceIndex : {};
  const model = isRecord(simulation.model) ? simulation.model : {};
  const latestResult = isRecord(simulation.latestResult) ? simulation.latestResult : {};
  const rawBuildItems = Array.isArray(root.buildItems) ? root.buildItems : [];
  let buildItems = rawBuildItems.flatMap((item, index): ArmBuildItem[] => {
    if (!isRecord(item)) return [];
    return [{
      id: text(item.id, `build-${index}`),
      label: text(item.label ?? item.title, "Project component"),
      description: text(item.description ?? item.summary, "No description supplied."),
      status: buildStatus(item.status),
      detail: text(item.detail) || undefined,
      path: text(item.path) || undefined,
    }];
  });
  if (!buildItems.length && Object.keys(simulation).length) {
    const simulationStatus = text(simulation.status, "unknown");
    const sourceStatus = text(sourceIndex.status, "unknown");
    buildItems = [
      {
        id: "arm-stack",
        label: "Arm Alliance source map",
        description: `${finiteNumber(sourceIndex.entryCount)} indexed paths connect the dashboard, laptop service, Pi gateway, firmware, tools, and simulator.`,
        status: sourceStatus === "ready" ? "ready" : sourceStatus === "missing" || sourceStatus === "invalid" ? "partial" : "unknown",
        path: text(sourceIndex.canonicalEntry, "software/SOURCE_INDEX.json"),
      },
      {
        id: "sim-lab",
        label: "Isaac simulation lab",
        description: text(model.description, "Four-joint model, mounted camera, deterministic desk scenes, and guarded SIM control."),
        status: simulationStatus === "calibrated" ? "ready" : simulationStatus === "provisional" ? "partial" : simulationStatus === "incomplete" ? "partial" : "unknown",
        detail: simulationStatus,
        path: "software/python/arm_sim/README.md",
      },
      {
        id: "tool-contract",
        label: "Shared arm tools",
        description: "Typed state, scene, camera, plan, sequence, apply, release, and floor-guard tools for SIM and REAL.",
        status: sourceStatus === "ready" ? "ready" : "unknown",
        path: "software/python/arm_mcp/server.py",
      },
    ];
  }
  const rawActions = Array.isArray(root.actions) ? root.actions : [];
  const actions = rawActions.flatMap((item, index): ArmControlCenterAction[] => {
    if (!isRecord(item)) return [];
    const id = text(item.id, `action-${index}`);
    const modes = Array.isArray(item.modes)
      ? item.modes.flatMap((mode) => backendMode(mode) ?? [])
      : undefined;
    const runtime = isRecord(item.runtime) ? item.runtime : null;
    return [{
      id,
      label: text(item.label ?? item.title, id),
      description: text(item.description ?? item.summary, "Run this local project action."),
      enabled: item.enabled !== false && item.available !== false,
      tone: item.tone === "primary" ? "primary" : "neutral",
      modes: modes?.length ? modes : undefined,
      runtime: runtime ? {
        status: text(runtime.status, "idle"),
        pid: typeof runtime.pid === "number" ? runtime.pid : undefined,
        startedAt: text(runtime.startedAt) || undefined,
        finishedAt: text(runtime.finishedAt) || undefined,
        lastExitCode: typeof runtime.lastExitCode === "number" ? runtime.lastExitCode : undefined,
        lastError: text(runtime.lastError) || undefined,
      } : undefined,
    }];
  });
  const rawSources = Array.isArray(root.sourceEntries)
    ? root.sourceEntries
    : Array.isArray(sourceIndex.entries)
      ? sourceIndex.entries
      : [];
  const sourceEntries = rawSources.flatMap((item): Array<{ label: string; path: string }> => {
    if (!isRecord(item)) return [];
    const path = text(item.path);
    if (!path) return [];
    return [{ label: text(item.label ?? item.id, path), path }];
  });
  let checks = isRecord(root.checks)
    ? {
        passed: finiteNumber(root.checks.passed),
        total: finiteNumber(root.checks.total),
        label: text(root.checks.label) || undefined,
      }
    : undefined;
  if (!checks && Object.keys(latestResult).length) {
    checks = {
      passed: latestResult.passed === true || latestResult.status === "passed" ? 1 : 0,
      total: 1,
      label: "Latest simulator frame validation",
    };
  }
  const calibration = isRecord(root.calibration)
    ? {
        completed: finiteNumber(root.calibration.completed),
        total: finiteNumber(root.calibration.total),
        label: text(root.calibration.label) || undefined,
      }
    : undefined;
  return {
    schemaVersion: text(root.schemaVersion ?? root.schema) || undefined,
    generatedAt: text(root.generatedAt) || undefined,
    projectName: text(root.projectName ?? project.name) || undefined,
    summary: text(root.summary) || undefined,
    checks,
    calibration,
    buildItems: buildItems.length ? buildItems : FALLBACK_BUILD_ITEMS,
    actions,
    sourceEntries,
  };
}

function readBackendPreference(): BackendPreference {
  try {
    const raw = globalThis.sessionStorage?.getItem(ARM_BACKEND_SESSION_KEY);
    if (!raw) return { id: null, mode: "sim" };
    const parsed: unknown = JSON.parse(raw);
    if (!isRecord(parsed)) return { id: null, mode: "sim" };
    return {
      id: text(parsed.id) || null,
      mode: backendMode(parsed.mode) ?? "sim",
    };
  } catch {
    return { id: null, mode: "sim" };
  }
}

function writeBackendPreference(backend: ArmBackendDescriptor) {
  try {
    globalThis.sessionStorage?.setItem(ARM_BACKEND_SESSION_KEY, JSON.stringify({ id: backend.id, mode: backend.mode }));
  } catch {
    // The selector still works when storage is unavailable; it simply resets on reload.
  }
}

export function armTabFromHash(hash: string): ArmControlCenterTab | null {
  const match = /^#arm\/(overview|control|live|guide)\/?$/.exec(hash.toLowerCase());
  return match?.[1] as ArmControlCenterTab | undefined ?? null;
}

function replaceArmHash(tab: ArmControlCenterTab, replace = false) {
  const next = `#arm/${tab}`;
  if (globalThis.location.hash === next) return;
  const url = `${globalThis.location.pathname}${globalThis.location.search}${next}`;
  if (replace) globalThis.history.replaceState(null, "", url);
  else globalThis.history.pushState(null, "", url);
}

export function withArmBackend(request: typeof fetch, backendId: string): typeof fetch {
  return ((input: RequestInfo | URL, init?: RequestInit) => {
    const inherited = typeof Request !== "undefined" && input instanceof Request ? input.headers : undefined;
    const headers = new Headers(inherited);
    new Headers(init?.headers).forEach((value, key) => headers.set(key, value));
    headers.set("X-Arm-Backend-Id", backendId);
    return request(input, { ...init, headers });
  }) as typeof fetch;
}

async function json(request: typeof fetch, url: string, init?: RequestInit) {
  const response = await request(url, init);
  const body: unknown = await response.json().catch(() => null);
  if (!response.ok) {
    const detail = isRecord(body) ? text(body.detail ?? body.message) : "";
    throw new Error(detail || `The local service returned HTTP ${response.status}.`);
  }
  return body;
}

function statusCopy(status: ArmBackendStatus, loading: boolean) {
  if (loading) return "Checking";
  if (status === "ready") return "Ready";
  if (status === "starting") return "Starting";
  if (status === "offline") return "Offline";
  if (status === "error") return "Needs attention";
  if (status === "configured") return "Configured · not probed";
  return "Status unknown";
}

function SectionHeading({ eyebrow, title, children }: { eyebrow: string; title: string; children?: ReactNode }) {
  return (
    <header className="armcc-section-heading">
      <div>
        <p>{eyebrow}</p>
        <h2>{title}</h2>
      </div>
      {children ? <div>{children}</div> : null}
    </header>
  );
}

function ActionRuntimeDetail({ action }: { action: ArmControlCenterAction }) {
  const runtime = action.runtime;
  if (!runtime || runtime.status === "idle") return null;
  return (
    <div className={`armcc-action-runtime is-${runtime.status}`} aria-live={runtime.status === "failed" ? "assertive" : "polite"}>
      <span>{runtime.status.toUpperCase()}</span>
      {runtime.pid ? <small>PID {runtime.pid}</small> : null}
      {runtime.lastExitCode !== undefined ? <small>EXIT {runtime.lastExitCode}</small> : null}
      {runtime.lastError ? <code>{runtime.lastError}</code> : null}
    </div>
  );
}

function Overview({
  backend,
  manifest,
  manifestLoading,
  manifestError,
  actionMessage,
  onTab,
  onRetry,
}: {
  backend: ArmBackendDescriptor;
  manifest: ArmControlCenterManifest;
  manifestLoading: boolean;
  manifestError: string | null;
  actionMessage: string | null;
  onTab: (tab: ArmControlCenterTab) => void;
  onRetry: () => void;
}) {
  const backendConfigured = backend.id !== UNCONFIGURED_BACKEND.id;
  const checks = manifest.checks;
  const calibration = manifest.calibration;
  const isaacSceneAction = manifest.actions.find((action) => action.id === ISAAC_SCENE_ACTION_ID);
  const environmentKind = backendConfigured
    ? backend.simulated ? "Local Isaac simulation" : "Raspberry Pi gateway"
    : "No request target";
  return (
    <div className="armcc-page armcc-overview-page">
      <header className="armcc-page-header">
        <div>
          <h1 id="armcc-overview-title" data-stage-heading tabIndex={-1}>Arm workspace</h1>
          <p>Choose an environment, then open its controls or reference guide.</p>
        </div>
        <div className="armcc-page-actions">
          <button type="button" disabled={!backendConfigured} onClick={() => onTab("control")}>{backendConfigured ? "Open control" : "Arm control unavailable"}</button>
          <button type="button" className="armcc-button-secondary" onClick={() => onTab("guide")}>Guide</button>
        </div>
      </header>

      <section className="armcc-environment-panel" aria-labelledby="armcc-environment-title">
        <header className="armcc-environment-header">
          <div className="armcc-environment-status">
            <span className={`armcc-status-dot is-${backend.status}${manifestLoading ? " is-loading" : ""}`} aria-hidden="true" />
            <div>
              <h2 id="armcc-environment-title">{backend.displayName}</h2>
              <p>{statusCopy(backend.status, manifestLoading)}</p>
            </div>
          </div>
          <span className="armcc-environment-mode">{backendConfigured ? backend.mode.toUpperCase() : "NONE"}</span>
        </header>

        <dl className="armcc-environment-identity">
          <div><dt>Backend</dt><dd><code>{backendConfigured ? backend.id : "Not configured"}</code></dd></div>
          <div><dt>Process identity</dt><dd><code title={backend.identity}>{backend.identity}</code></dd></div>
          <div><dt>Version</dt><dd><code>{backend.version ?? "Not reported"}</code></dd></div>
          <div><dt>Provenance</dt><dd>{environmentKind}</dd></div>
        </dl>

        <div className="armcc-environment-rows" aria-label="Workspace status">
          <div>
            <span><strong>Checks</strong><small>{checks?.label ?? (manifestLoading ? "Loading manifest" : "No report loaded")}</small></span>
            <code>{checks && checks.total > 0 ? `${checks.passed}/${checks.total}` : "Not reported"}</code>
          </div>
          <div>
            <span><strong>Calibration</strong><small>{calibration?.label ?? "Measured jobs completed"}</small></span>
            <code>{calibration && calibration.total > 0 ? `${calibration.completed}/${calibration.total}` : "Not reported"}</code>
          </div>
        </div>

        <footer className="armcc-environment-footer">
          <button type="button" className="armcc-text-action" onClick={() => onTab("guide")}>Open full guide</button>
          {manifestError ? (
            <span className="armcc-environment-error" role="status">
              <strong>Manifest unavailable.</strong> {manifestError}
              <button type="button" className="armcc-text-action" onClick={onRetry}>Retry</button>
            </span>
          ) : actionMessage ? <span className="armcc-environment-message" role="status">{actionMessage}</span> : null}
          {isaacSceneAction ? <ActionRuntimeDetail action={isaacSceneAction} /> : null}
        </footer>
      </section>
    </div>
  );
}

function Guide({
  manifest,
  backend,
  actionBusy,
  actionMessage,
  onAction,
  onTab,
}: {
  manifest: ArmControlCenterManifest;
  backend: ArmBackendDescriptor;
  actionBusy: string | null;
  actionMessage: string | null;
  onAction: (action: ArmControlCenterAction) => void;
  onTab: (tab: ArmControlCenterTab) => void;
}) {
  const backendConfigured = backend.id !== UNCONFIGURED_BACKEND.id;
  const actions = manifest.actions.filter((action) => !action.modes || action.modes.includes(backend.mode));
  return (
    <div className="armcc-page armcc-guide-page">
      <SectionHeading eyebrow="Guide" title="Arm reference" />
      <p className="armcc-page-intro">Choose what you want to do. For agent-guided setup, open the repository root in Codex, follow <code>AGENTS.md</code> and <code>docs/SETUP_WITH_CODEX.md</code>, then configure <code>software/plugin/plugins/arm-alliance/</code>.</p>

      <section className="armcc-guide-start" aria-label="Common tasks">
        <button type="button" disabled={!backendConfigured} onClick={() => onTab("control")}><span>MOVE</span><strong>{backendConfigured ? `Control the ${backend.mode === "sim" ? "simulated" : "real"} arm` : "Arm control unavailable"}</strong><small>Open measured state, camera, calibration, and reviewed planning.</small></button>
        <button type="button" onClick={() => onTab("overview")}><span>OPEN</span><strong>Open the workspace</strong><small>Review the active environment and its recorded status.</small></button>
      </section>

      <div className="armcc-guide-columns">
        <section>
          <h3>How work is routed</h3>
          <dl className="armcc-route-list">
            <div><dt>ACTION</dt><dd>Change physical state with the fewest useful views and one deliberate effect-producing route.</dd></div>
            <div><dt>INSPECTION</dt><dd>Move the viewpoint until labels, connectors, chips, or other requested evidence are actually visible.</dd></div>
            <div><dt>HYBRID</dt><dd>Inspect only until the target and route are clear, then switch immediately to action.</dd></div>
          </dl>
        </section>
        <section>
          <h3>What the evidence means</h3>
          <ul className="armcc-proof-list">
            <li><strong>Source checks</strong><span>Prove code and contracts, not physical behavior.</span></li>
            <li><strong>Measured state</strong><span>Proves current telemetry, not requested arrival.</span></li>
            <li><strong>Camera pixels</strong><span>Prove only what the returned frame visibly contains.</span></li>
            <li><strong>Simulation</strong><span>Improves workflows; calibration decides how well it transfers.</span></li>
          </ul>
        </section>
      </div>

      <section className="armcc-section">
        <SectionHeading eyebrow="Components" title="Implementation inventory" />
        <div className="armcc-guide-inventory">
          {manifest.buildItems.map((item) => (
            <article key={item.id}><span>{item.status === "unknown" ? "INDEXED" : item.status.toUpperCase()}</span><h3>{item.label}</h3><p>{item.description}</p>{item.path ? <code>{item.path}</code> : null}</article>
          ))}
        </div>
        {manifest.sourceEntries.length ? (
          <dl className="armcc-source-map">
            {manifest.sourceEntries.map((entry) => <div key={`${entry.label}-${entry.path}`}><dt>{entry.label}</dt><dd><code>{entry.path}</code></dd></div>)}
          </dl>
        ) : null}
      </section>

      {actions.length ? (
        <section className="armcc-section">
          <SectionHeading eyebrow={`${backend.mode} tools`} title="Project actions" />
          <div className="armcc-guide-actions">
            {actions.map((action) => (
              <button type="button" key={action.id} disabled={!action.enabled || actionBusy !== null} onClick={() => onAction(action)}>
                <strong>{actionBusy === action.id ? "Opening…" : action.label}</strong><span>{action.description}</span><ActionRuntimeDetail action={action} />
              </button>
            ))}
          </div>
          {actionMessage ? <p className="armcc-action-message" role="status">{actionMessage}</p> : null}
        </section>
      ) : null}
    </div>
  );
}

export function ArmControlCenter({ request = defaultRequest }: ArmControlCenterProps) {
  const [tab, setTab] = useState<ArmControlCenterTab>(() => armTabFromHash(globalThis.location.hash) ?? "overview");
  const initialPreference = useMemo(readBackendPreference, []);
  const [backends, setBackends] = useState<ArmBackendDescriptor[]>([]);
  const [selectedBackendId, setSelectedBackendId] = useState(() => initialPreference.id ?? (initialPreference.mode === "real" ? "real" : "sim"));
  const [backendsLoading, setBackendsLoading] = useState(true);
  const [backendsError, setBackendsError] = useState<string | null>(null);
  const [manifest, setManifest] = useState<ArmControlCenterManifest>(() => normalizeManifest(null));
  const [manifestLoading, setManifestLoading] = useState(true);
  const [manifestError, setManifestError] = useState<string | null>(null);
  const [manifestRevision, setManifestRevision] = useState(0);
  const [actionBusy, setActionBusy] = useState<string | null>(null);
  const [actionMessage, setActionMessage] = useState<string | null>(null);
  const [actionToken, setActionToken] = useState<string | null>(null);
  const [controlOwner, setControlOwner] = useState<ArmControlOwner>(null);
  const manifestPollAttemptRef = useRef(0);

  const selectedBackend = useMemo(() => {
    return backends.find((backend) => backend.id === selectedBackendId)
      ?? backends.find((backend) => backend.mode === initialPreference.mode)
      ?? backends.find((backend) => backend.mode === "sim")
      ?? backends[0]
      ?? UNCONFIGURED_BACKEND;
  }, [backends, initialPreference.mode, selectedBackendId]);

  const backendConfigured = selectedBackend.id !== UNCONFIGURED_BACKEND.id;
  const backendRequest = useMemo(() => backendConfigured ? withArmBackend(request, selectedBackend.id) : request, [backendConfigured, request, selectedBackend.id]);

  const chooseTab = useCallback((next: ArmControlCenterTab) => {
    if (controlOwner && next !== controlOwner) return;
    if (!backendConfigured && (next === "control" || next === "live")) return;
    setTab(next);
    replaceArmHash(next);
  }, [backendConfigured, controlOwner]);

  const setOwnerBusy = useCallback((owner: Exclude<ArmControlOwner, null>, busy: boolean) => {
    setControlOwner((current) => busy ? owner : current === owner ? null : current);
  }, []);
  // These callbacks participate in child acquire/release effects. Keeping them
  // stable prevents a shell rerender from looking like an owner unmount and
  // briefly releasing an active motion or live-follow session.
  const setControlBusy = useCallback((busy: boolean) => setOwnerBusy("control", busy), [setOwnerBusy]);
  const setLiveBusy = useCallback((busy: boolean) => setOwnerBusy("live", busy), [setOwnerBusy]);

  useEffect(() => {
    if (!armTabFromHash(globalThis.location.hash)) replaceArmHash(tab, true);
    const onHashChange = () => {
      const next = armTabFromHash(globalThis.location.hash);
      if (controlOwner && next !== controlOwner) {
        setTab(controlOwner);
        replaceArmHash(controlOwner, true);
        return;
      }
      if (next) setTab(next);
    };
    globalThis.addEventListener("hashchange", onHashChange);
    globalThis.addEventListener("popstate", onHashChange);
    return () => {
      globalThis.removeEventListener("hashchange", onHashChange);
      globalThis.removeEventListener("popstate", onHashChange);
    };
  }, [controlOwner, tab]);

  useEffect(() => {
    let cancelled = false;
    const controller = new AbortController();
    setBackendsLoading(true);
    void json(request, "/api/arm/backends", {
      headers: { Accept: "application/json" },
      signal: controller.signal,
    }).then((body) => {
      if (cancelled) return;
      const available = normalizeBackends(body);
      const preference = readBackendPreference();
      const next = available.find((backend) => backend.id === preference.id)
        ?? available.find((backend) => backend.mode === preference.mode)
        ?? available.find((backend) => backend.mode === "sim")
        ?? available[0];
      setBackends(available);
      setSelectedBackendId(next?.id ?? UNCONFIGURED_BACKEND.id);
      if (next) writeBackendPreference(next);
      setBackendsError(null);
    }).catch((caught) => {
      if (cancelled || controller.signal.aborted) return;
      setBackends([]);
      setSelectedBackendId(UNCONFIGURED_BACKEND.id);
      setBackendsError(caught instanceof Error ? caught.message : "Backend inventory is unavailable.");
    }).finally(() => {
      if (!cancelled) setBackendsLoading(false);
    });
    return () => { cancelled = true; controller.abort(); };
  }, [request]);

  useEffect(() => {
    let cancelled = false;
    const controller = new AbortController();
    setManifestLoading(true);
    setManifestError(null);
    void json(backendRequest, "/api/arm/control-center/manifest", {
      headers: { Accept: "application/json" },
      signal: controller.signal,
    }).then((body) => {
      if (cancelled) return;
      setManifest(normalizeManifest(body));
    }).catch((caught) => {
      if (cancelled || controller.signal.aborted) return;
      setManifest(normalizeManifest(null));
      setManifestError(caught instanceof Error ? caught.message : "Build manifest is unavailable.");
    }).finally(() => {
      if (!cancelled) setManifestLoading(false);
    });
    return () => { cancelled = true; controller.abort(); };
  }, [backendRequest, manifestRevision]);

  useEffect(() => {
    const runtimeActive = manifest.actions.some((action) => action.runtime?.status === "running" || action.runtime?.status === "starting");
    if (!runtimeActive) {
      manifestPollAttemptRef.current = 0;
      return;
    }
    const delay = Math.min(1_000 * (2 ** Math.min(manifestPollAttemptRef.current, 3)), 8_000);
    const timer = window.setTimeout(() => {
      manifestPollAttemptRef.current += 1;
      setManifestRevision((revision) => revision + 1);
    }, delay);
    return () => window.clearTimeout(timer);
  }, [backendRequest, manifest.actions]);

  const chooseBackend = (backend: ArmBackendDescriptor) => {
    if (controlOwner || backend.id === selectedBackend.id) return;
    writeBackendPreference(backend);
    setSelectedBackendId(backend.id);
    setActionBusy(null);
    setActionMessage(null);
  };

  const runAction = async (action: ArmControlCenterAction) => {
    if (!action.enabled || actionBusy !== null) return;
    const actionLabel = action.id === ISAAC_SCENE_ACTION_ID ? "Open scene in Isaac Sim" : action.label;
    setActionBusy(action.id);
    setActionMessage(null);
    try {
      const token = actionToken ?? await loadActionToken(backendRequest);
      if (token !== actionToken) setActionToken(token);
      const body = await json(backendRequest, `/api/arm/control-center/actions/${encodeURIComponent(action.id)}`, {
        method: "POST",
        headers: { Accept: "application/json", "X-Co-Arm-Token": token },
      });
      const message = isRecord(body) ? text(body.message ?? body.detail) : "";
      const status = isRecord(body) ? text(body.status) : "";
      const fallbackMessage = action.id === ISAAC_SCENE_ACTION_ID
        ? status === "already_running"
          ? "Isaac Sim is already open."
          : "Isaac Sim is opening in a separate window."
        : status === "already_running"
          ? `${actionLabel} is already running.`
          : `${actionLabel} started.`;
      setActionMessage(message || fallbackMessage);
      setManifestRevision((revision) => revision + 1);
    } catch (caught) {
      setActionMessage(caught instanceof Error ? caught.message : `${actionLabel} could not be opened.`);
    } finally {
      setActionBusy(null);
    }
  };

  const isaacSceneAction = manifest.actions.find((action) => action.id === ISAAC_SCENE_ACTION_ID);
  const isaacSceneBusy = actionBusy === ISAAC_SCENE_ACTION_ID;
  const isaacSceneUnavailable = !manifestLoading && (!isaacSceneAction || !isaacSceneAction.enabled);
  const isaacSceneLabel = manifestLoading
    ? "Checking Isaac"
    : isaacSceneUnavailable
      ? "Isaac unavailable"
      : isaacSceneBusy
        ? "Opening Isaac"
        : "Open Isaac Sim";

  return (
    <div className="armcc" data-mode={selectedBackend.mode}>
      <header className="armcc-header">
        <div className="armcc-title-lockup">
          <strong>Arm Control Center</strong>
        </div>

        <nav className="armcc-tabs" role="tablist" aria-label="Arm Control Center">
          {TABS.map((item) => (
            <button
              type="button"
              role="tab"
              id={`armcc-tab-${item.id}`}
              aria-selected={tab === item.id}
              aria-controls="armcc-tabpanel"
              tabIndex={tab === item.id ? 0 : -1}
              disabled={(controlOwner !== null && item.id !== controlOwner) || (!backendConfigured && (item.id === "control" || item.id === "live"))}
              key={item.id}
              onKeyDown={handleTabKeyDown}
              onClick={() => chooseTab(item.id)}
            >{item.label}</button>
          ))}
        </nav>

        <span className="armcc-toolbar-spacer" aria-hidden="true" />

        <button
          type="button"
          className="armcc-isaac-launch"
          aria-label={isaacSceneLabel}
          aria-busy={isaacSceneBusy}
          aria-describedby="armcc-isaac-launch-description"
          disabled={manifestLoading || isaacSceneUnavailable || actionBusy !== null || controlOwner !== null}
          onClick={() => isaacSceneAction && void runAction(isaacSceneAction)}
          title={isaacSceneUnavailable
            ? "The local Isaac scene viewer is not available in this build."
            : "Opens the generated scene in a separate visible Isaac editor; it does not attach to the live headless bridge."}
        >
          <span className="armcc-isaac-label-long">{isaacSceneLabel}</span>
          <span className="armcc-isaac-label-short" aria-hidden="true">Isaac</span>
        </button>
        <span id="armcc-isaac-launch-description" className="armcc-visually-hidden">
          Opens the generated desk-arm scene in a separate visible Isaac editor. This is not a live attachment to the headless simulator bridge.
        </span>

        <div className="armcc-backend">
          <div className={`armcc-segmented${backends.length ? "" : " is-empty"}`} role="group" aria-label="Arm environment" aria-describedby={controlOwner ? "armcc-backend-lock" : undefined}>
            {!backends.length ? <button type="button" disabled aria-label={backendsLoading ? "Arm backends loading" : "No arm backend configured"}>NONE</button> : null}
            {[...backends].sort((left, right) => Number(left.mode === "real") - Number(right.mode === "real")).map((backend) => {
              const mode = backend.mode;
              return (
                <button
                  type="button"
                  key={backend.id}
                  aria-label={mode === "sim" ? "Use simulator" : "Use real arm"}
                  aria-pressed={selectedBackend.id === backend.id}
                  disabled={controlOwner !== null}
                  onClick={() => chooseBackend(backend)}
                >{mode === "sim" ? "SIM" : "REAL"}</button>
              );
            })}
          </div>
          <div className="armcc-backend-status" aria-live="polite" aria-label={`Backend status: ${statusCopy(selectedBackend.status, backendsLoading)}`}>
            <span className={`armcc-status-dot is-${selectedBackend.status}${backendsLoading ? " is-loading" : ""}`} aria-hidden="true" />
            <strong>{statusCopy(selectedBackend.status, backendsLoading)}</strong>
          </div>
        </div>
      </header>

      <div className="armcc-notices">
        {tab === "control" && isaacSceneAction?.runtime && isaacSceneAction.runtime.status !== "idle" ? (
          <div className="armcc-isaac-runtime-notice" aria-label="Isaac scene viewer status">
            <ActionRuntimeDetail action={isaacSceneAction} />
          </div>
        ) : null}
        {controlOwner ? <div className="armcc-backend-lock" id="armcc-backend-lock" role="status">Active {controlOwner === "live" ? "live follow" : "control"} is bound to <strong>{selectedBackend.id}</strong>. Finish the operation or press STOP in {controlOwner === "live" ? "Live" : "Control"} before switching backend or page.</div> : null}
        {!backendsLoading && !backendsError && !backends.length ? <div className="armcc-backend-note" role="status">No arm backend is configured. Add SIM or REAL in the local service before opening controls.</div> : null}
        {backendsError ? <div className="armcc-backend-note" role="status">Backend inventory offline: {backendsError}. Controls remain disabled.</div> : null}
        {actionMessage && tab !== "overview" && tab !== "guide" ? <p className="armcc-action-message armcc-global-action-message" role="status" aria-atomic="true">{actionMessage}</p> : null}
      </div>

      <main className={`armcc-main is-${tab}`}>
        <section role="tabpanel" id="armcc-tabpanel" aria-labelledby={`armcc-tab-${tab}`} className="armcc-panel">
          {tab === "overview" ? (
            <Overview
              backend={selectedBackend}
              manifest={manifest}
              manifestLoading={manifestLoading}
              manifestError={manifestError}
              actionMessage={actionMessage}
              onTab={chooseTab}
              onRetry={() => setManifestRevision((revision) => revision + 1)}
            />
          ) : tab === "control" ? (
            selectedBackend === UNCONFIGURED_BACKEND ? (
              <div className="armcc-control-unconfigured" role="status"><strong>No arm backend is configured.</strong><span>Add SIM or REAL in the local service, then reload the inventory.</span></div>
            ) : (
              <div className="armcc-control" key={selectedBackend.id}>
                <div className="armcc-control-ribbon"><strong>{selectedBackend.mode === "sim" ? "SIMULATOR" : "REAL ARM"}</strong><span>Every request is bound to <code>{selectedBackend.id}</code></span></div>
                <SimpleArm request={backendRequest} onControlBusyChange={setControlBusy} />
              </div>
            )
          ) : tab === "live" ? (
            selectedBackend === UNCONFIGURED_BACKEND ? (
              <div className="armcc-control-unconfigured" role="status"><strong>No arm backend is configured.</strong><span>Add REAL in the local service before opening Speed Lab.</span></div>
            ) : (
              <ArmLiveFollow
                key={selectedBackend.id}
                request={backendRequest}
                backendId={selectedBackend.mode}
                onControlBusyChange={setLiveBusy}
              />
            )
          ) : (
            <Guide manifest={manifest} backend={selectedBackend} actionBusy={actionBusy} actionMessage={actionMessage} onAction={(action) => void runAction(action)} onTab={chooseTab} />
          )}
        </section>
      </main>
    </div>
  );
}
