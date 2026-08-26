import { useMemo, useRef, useState, type KeyboardEvent, type PointerEvent } from "react";
import { clampTarget, forwardKinematics } from "./armKinematics";
import type { ArmGeometry, ArmPose, CartesianTarget } from "./armTypes";
import "./arm-viewport.css";

interface ArmViewportProps {
  currentPose: ArmPose;
  previewPose: ArmPose;
  geometry: ArmGeometry;
  target: CartesianTarget;
  targetReachable: boolean;
  /** Omit to get a view-only twin: no target handle, no plane picker, orbit only.
   *  Driving the arm by dragging a point in 3D means solving IK, which can refuse
   *  a reachable-looking point; the flat view drives joints directly instead. */
  onTargetChange?: (target: CartesianTarget) => void;
}

interface Point3 {
  x: number;
  y: number;
  z: number;
}

interface Point2 {
  x: number;
  y: number;
}

type InteractionMode = "orbit" | "target";
type TargetPlane = "XZ" | "YZ" | "XY";

interface DragState {
  pointerId: number;
  x: number;
  y: number;
  kind: InteractionMode;
  captureTarget: SVGElement;
  target?: CartesianTarget;
  plane?: TargetPlane;
}

const VIEW_BOX_WIDTH = 720;
const VIEW_BOX_HEIGHT = 520;
const ARM_DRAW_SCALE = 0.86;
const TARGET_PLANES: readonly TargetPlane[] = ["XZ", "YZ", "XY"];

function radians(degrees: number) {
  return (degrees * Math.PI) / 180;
}

function armPoints(pose: ArmPose, geometry: ArmGeometry): Point3[] {
  const yaw = radians(pose.joint_1);
  const shoulder = radians(pose.joint_2);
  const elbow = radians(pose.joint_3);
  const direction = (radius: number, pitch: number): Point3 => ({
    x: radius * Math.cos(pitch) * Math.cos(yaw),
    y: radius * Math.cos(pitch) * Math.sin(yaw),
    z: radius * Math.sin(pitch),
  });
  const base = { x: 0, y: 0, z: 0 };
  const shoulderPoint = { x: 0, y: 0, z: geometry.baseHeightMm };
  const upper = direction(geometry.upperArmMm, shoulder);
  const elbowPoint = {
    x: shoulderPoint.x + upper.x,
    y: shoulderPoint.y + upper.y,
    z: shoulderPoint.z + upper.z,
  };
  const distal = direction(geometry.forearmMm + geometry.toolOffsetMm, shoulder + elbow);
  return [
    base,
    shoulderPoint,
    elbowPoint,
    { x: elbowPoint.x + distal.x, y: elbowPoint.y + distal.y, z: elbowPoint.z + distal.z },
  ];
}

function path(points: Point2[]) {
  return points.map((point, index) => `${index === 0 ? "M" : "L"}${point.x.toFixed(1)},${point.y.toFixed(1)}`).join(" ");
}

export function ArmViewport({
  currentPose,
  previewPose,
  geometry,
  target,
  targetReachable,
  onTargetChange,
}: ArmViewportProps) {
  const canTarget = typeof onTargetChange === "function";
  const [azimuth, setAzimuth] = useState(-38);
  const [elevation, setElevation] = useState(24);
  const [mode, setMode] = useState<InteractionMode>("target");
  const interaction: InteractionMode = canTarget ? mode : "orbit";
  const [targetPlane, setTargetPlane] = useState<TargetPlane>("XZ");
  const drag = useRef<DragState | null>(null);
  const project = useMemo(() => {
    const az = radians(azimuth);
    const el = radians(elevation);
    return (point: Point3): Point2 => {
      const horizontal = point.x * Math.cos(az) - point.y * Math.sin(az);
      const depth = point.x * Math.sin(az) + point.y * Math.cos(az);
      const vertical = point.z * Math.cos(el) - depth * Math.sin(el);
      return { x: 360 + horizontal * ARM_DRAW_SCALE, y: 430 - vertical * ARM_DRAW_SCALE };
    };
  }, [azimuth, elevation]);
  const currentPoints = armPoints(currentPose, geometry).map(project);
  const previewPoints = armPoints(previewPose, geometry).map(project);
  const targetPoint = project(target);
  const actualTarget = forwardKinematics(currentPose, geometry);

  const movedTarget = (
    origin: CartesianTarget,
    horizontal: number,
    vertical: number,
    plane: TargetPlane,
  ) => {
    const next = { ...origin };
    if (plane === "XZ") {
      next.x += horizontal;
      next.z -= vertical;
    } else if (plane === "YZ") {
      next.y += horizontal;
      next.z -= vertical;
    } else {
      next.x += horizontal;
      next.y -= vertical;
    }
    return clampTarget(next);
  };

  const pointerDeltaInWorld = (canvas: SVGSVGElement, dx: number, dy: number) => {
    const bounds = canvas.getBoundingClientRect();
    const viewBox = canvas.viewBox?.baseVal;
    const viewWidth = viewBox?.width || VIEW_BOX_WIDTH;
    const viewHeight = viewBox?.height || VIEW_BOX_HEIGHT;
    const renderedWidth = bounds.width || viewWidth;
    const renderedHeight = bounds.height || viewHeight;
    return {
      horizontal: (dx * viewWidth) / renderedWidth / ARM_DRAW_SCALE,
      vertical: (dy * viewHeight) / renderedHeight / ARM_DRAW_SCALE,
    };
  };

  const pointerDown = (event: PointerEvent<SVGSVGElement>) => {
    if (interaction !== "orbit") return;
    drag.current = {
      pointerId: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      kind: "orbit",
      captureTarget: event.currentTarget,
    };
    event.currentTarget.setPointerCapture?.(event.pointerId);
  };

  const targetPointerDown = (event: PointerEvent<SVGCircleElement>) => {
    if (interaction !== "target") return;
    event.preventDefault();
    event.stopPropagation();
    drag.current = {
      pointerId: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      kind: "target",
      captureTarget: event.currentTarget,
      target: { ...target },
      plane: targetPlane,
    };
    event.currentTarget.setPointerCapture?.(event.pointerId);
  };

  const pointerMove = (event: PointerEvent<SVGSVGElement>) => {
    const active = drag.current;
    if (!active || active.pointerId !== event.pointerId) return;
    const dx = event.clientX - active.x;
    const dy = event.clientY - active.y;
    if (active.kind === "orbit") {
      drag.current = { ...active, x: event.clientX, y: event.clientY };
      setAzimuth((value) => value + dx * 0.45);
      setElevation((value) => Math.max(-10, Math.min(72, value - dy * 0.35)));
    } else {
      const delta = pointerDeltaInWorld(event.currentTarget, dx, dy);
      const next = movedTarget(
        active.target ?? target,
        delta.horizontal,
        delta.vertical,
        active.plane ?? targetPlane,
      );
      drag.current = { ...active, x: event.clientX, y: event.clientY, target: next };
      onTargetChange?.(next);
    }
  };

  const pointerUp = (event: PointerEvent<SVGSVGElement>) => {
    const active = drag.current;
    if (!active || active.pointerId !== event.pointerId) return;
    drag.current = null;
    if (active.captureTarget.hasPointerCapture?.(event.pointerId)) {
      active.captureTarget.releasePointerCapture?.(event.pointerId);
    }
  };

  const keyDown = (event: KeyboardEvent<SVGSVGElement>) => {
    if (!onTargetChange || interaction !== "target" || !event.key.startsWith("Arrow")) return;
    event.preventDefault();
    const step = event.shiftKey ? 10 : 2;
    if (event.key === "ArrowLeft") onTargetChange(movedTarget(target, -step, 0, targetPlane));
    if (event.key === "ArrowRight") onTargetChange(movedTarget(target, step, 0, targetPlane));
    if (event.key === "ArrowUp") onTargetChange(movedTarget(target, 0, -step, targetPlane));
    if (event.key === "ArrowDown") onTargetChange(movedTarget(target, 0, step, targetPlane));
  };

  const grid = Array.from({ length: 11 }, (_, index) => (index - 5) * 55);
  return (
    <section className="arm-viewport" aria-labelledby="arm-viewport-title">
      <header>
        <div>
          <span className="web-eyebrow">Digital twin</span>
          <h2 id="arm-viewport-title">Arm workspace</h2>
        </div>
        <div className="arm-view-tools" aria-label="Viewport interaction mode">
          {canTarget ? (
            <>
              <button type="button" className={interaction === "target" ? "is-active" : ""} onClick={() => setMode("target")} aria-pressed={interaction === "target"}>Move target</button>
              <button type="button" className={interaction === "orbit" ? "is-active" : ""} onClick={() => setMode("orbit")} aria-pressed={interaction === "orbit"}>Orbit view</button>
            </>
          ) : null}
          <button type="button" onClick={() => { setAzimuth(-38); setElevation(24); }}>Reset view</button>
        </div>
        {interaction === "target" ? (
          <div className="arm-view-planes" role="group" aria-label="Target movement plane">
            {TARGET_PLANES.map((plane) => (
              <button
                key={plane}
                type="button"
                className={targetPlane === plane ? "is-active" : ""}
                aria-pressed={targetPlane === plane}
                onClick={() => setTargetPlane(plane)}
              >
                {plane}
              </button>
            ))}
          </div>
        ) : null}
      </header>
      <div className="arm-canvas-wrap">
        <svg
          className={`arm-canvas is-${interaction}`}
          viewBox="0 0 720 520"
          role="application"
          aria-label={interaction === "target" ? `3D arm target. Drag the target handle to move in the ${targetPlane} plane. Arrow keys make fine adjustments.` : "3D arm view. Drag anywhere to orbit."}
          tabIndex={0}
          onPointerDown={pointerDown}
          onPointerMove={pointerMove}
          onPointerUp={pointerUp}
          onPointerCancel={pointerUp}
          onKeyDown={keyDown}
        >
          <defs>
            <linearGradient id="arm-floor-fade" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stopColor="#6d9ec5" stopOpacity=".14"/><stop offset="1" stopColor="#6d9ec5" stopOpacity="0"/></linearGradient>
            <filter id="arm-glow"><feGaussianBlur stdDeviation="3" result="blur"/><feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
          </defs>
          <rect width="720" height="520" fill="transparent" />
          <path d="M0 430 H720 V520 H0Z" fill="url(#arm-floor-fade)" />
          <g className="arm-grid" aria-hidden="true">
            {grid.map((offset) => {
              const a = project({ x: offset, y: -275, z: 0 });
              const b = project({ x: offset, y: 275, z: 0 });
              const c = project({ x: -275, y: offset, z: 0 });
              const d = project({ x: 275, y: offset, z: 0 });
              return <g key={offset}><line x1={a.x} y1={a.y} x2={b.x} y2={b.y}/><line x1={c.x} y1={c.y} x2={d.x} y2={d.y}/></g>;
            })}
          </g>
          <g className="arm-axes" aria-hidden="true">
            {([[[0,0,0],[120,0,0],"X"],[[0,0,0],[0,120,0],"Y"],[[0,0,0],[0,0,120],"Z"]] as const).map(([start, end, label]) => {
              const a = project({ x: start[0], y: start[1], z: start[2] });
              const b = project({ x: end[0], y: end[1], z: end[2] });
              return <g key={label}><line x1={a.x} y1={a.y} x2={b.x} y2={b.y}/><text x={b.x + 5} y={b.y - 5}>{label}</text></g>;
            })}
          </g>
          <g className="arm-current" aria-label="Measured pose">
            <path d={path(currentPoints)} />
            {currentPoints.map((point, index) => <circle key={index} cx={point.x} cy={point.y} r={index === 0 ? 16 : 8}/>) }
          </g>
          <g className="arm-preview" aria-label="Preview pose">
            <path d={path(previewPoints)} />
            {previewPoints.slice(1).map((point, index) => <circle key={index} cx={point.x} cy={point.y} r="6"/>) }
          </g>
          {canTarget ? (
            <g className={`arm-target${targetReachable ? " is-reachable" : " is-blocked"}`} transform={`translate(${targetPoint.x} ${targetPoint.y})`} filter="url(#arm-glow)">
              <circle
                className="arm-target-hit"
                r="34"
                fill="transparent"
                style={{ fill: "transparent", stroke: "transparent" }}
                role="button"
                aria-label={`Target handle for ${targetPlane} plane`}
                tabIndex={interaction === "target" ? 0 : -1}
                pointerEvents={interaction === "target" ? "all" : "none"}
                onPointerDown={targetPointerDown}
              />
              <circle r="13" pointerEvents="none"/><line x1="-22" x2="22" pointerEvents="none"/><line y1="-22" y2="22" pointerEvents="none"/>
            </g>
          ) : null}
        </svg>
        <div className="arm-pose-legend" aria-label="Pose legend">
          <span><i className="is-current"/>Measured</span>
          <span><i className="is-preview"/>Ghost preview</span>
          {canTarget ? <span><i className={targetReachable ? "is-target" : "is-blocked"}/>Target</span> : null}
        </div>
        <output className="arm-coordinate-readout" aria-live="polite">
          {canTarget ? <span>Target <b>X {target.x.toFixed(1)}</b> <b>Y {target.y.toFixed(1)}</b> <b>Z {target.z.toFixed(1)}</b> mm</span> : null}
          <span>Measured <b>X {actualTarget.x.toFixed(1)}</b> <b>Y {actualTarget.y.toFixed(1)}</b> <b>Z {actualTarget.z.toFixed(1)}</b> mm</span>
        </output>
      </div>
      <p className="arm-viewport-help">
        {!canTarget
          ? "Drag anywhere to orbit. This view shows the arm; the flat view drives it."
          : interaction === "target"
            ? `Choose a plane, then drag the large target handle. ${targetPlane[0]} moves horizontally and ${targetPlane[1]} moves vertically; arrow keys nudge the same axes. Only the ghost preview moves; hardware still requires a reviewed path and hold-to-move.`
            : "Drag anywhere in the workspace to orbit the view."}
      </p>
    </section>
  );
}
