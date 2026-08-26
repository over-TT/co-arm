import { useRef, type KeyboardEvent, type PointerEvent } from "react";
import { lowestPointMm, planarInverse, type PlanarSolution } from "./armKinematics";
import type { ArmGeometry, ArmJointId, ArmPose, SimpleJointId } from "./armTypes";
import "./arm-2d.css";

/**
 * The arm as two flat drawings you grab directly.
 *
 * The 3D twin answers "where is the tool", which needs full IK and can refuse a
 * point as unreachable. This answers "put the arm here", in the one plane the
 * shoulder and elbow actually move in — so the tip is dragged freely and the two
 * joints solve for it, and a point past the arm's reach straightens it out
 * instead of stopping. The base is a dial, because a yaw joint drawn on a side
 * elevation is a dot.
 *
 * Angles in and out are the twin's frame, same as ArmViewport. `SimpleArm` owns
 * the translation to what the servos read.
 */

export interface JointLimit {
  min: number;
  max: number;
}

export interface Arm2DProps {
  /** Draft/planned pose. Grips follow this pose so local IK remains immediate. */
  pose: ArmPose;
  /** Last measured pose from telemetry. It stays solid while a draft is shown. */
  measuredPose?: ArmPose;
  /** Draw `pose` as a dashed proposal over the solid measured arm. */
  planned?: boolean;
  /** Every Pi-reviewed waypoint. These are annotations only; the solid arm
   * remains measured telemetry and `pose` remains the editable selection. */
  sequencePoses?: ReadonlyArray<ArmPose>;
  /** Lock all physical-pose editing while a server-owned sequence is running. */
  disabled?: boolean;
  limits: Record<ArmJointId, JointLimit>;
  geometry: ArmGeometry;
  /** Keep-out plane in mm above the floor, or null when the guard is off.
   *  Independent of `limits`: those are each joint's travel, this is a region
   *  of space the whole arm is kept out of. */
  floorMm?: number | null;
  /** Fires continuously through a drag. Preview only — nothing should move. */
  onPose: (next: Partial<Record<ArmJointId, number>>) => void;
  /** Fires when the handle is let go. The owner prepares a non-moving plan. */
  onRelease: () => void;
  /** Optional lifecycle hooks for owners that stream during the gesture instead
   * of preparing one plan on release. Pointer cancellation stays distinct from
   * an intentional release so a live controller can discard pending input. */
  onInteractionStart?: () => void;
  onInteractionEnd?: (reason: "release" | "cancel") => void;
  /** A planar-only owner can omit Base/camera dials without maintaining a
   * second copy of the side-view IK and floor guard. */
  showDials?: boolean;
  /** Truthful owner-specific help. The normal planner copy remains the default. */
  interactionCopy?: string;
  /** Owner-specific empty-value copy; the planner keeps its compact dash. */
  emptyReadout?: string;
  /** Joints driven from here but outside the arm — the camera. Each gets its own
   *  dial; the wedge on it is whatever calibration measured, not a fixed sweep. */
  /** `pending` means the joint has no zero yet, so the dial is drawn but inert:
   *  there is no angle to drag to until calibration gives it one. */
  extras?: ReadonlyArray<{
    id: SimpleJointId; name: string; degrees: number; measuredDegrees?: number; limit: JointLimit; pending?: boolean;
  }>;
  /** What to print beside each joint. Supplied by the caller rather than taken
   *  from `pose`, because `pose` is in the twin's frame and the numbers on the
   *  drawing have to be the same ones the joint panel shows. */
  readouts?: Partial<Record<"joint_1" | "joint_2" | "joint_3", number | null>>;
}

type Handle = SimpleJointId | "tip";

const SIDE_W = 480;
const SIDE_H = 380;
const PIVOT_X = 240;
const PIVOT_Y = 342;
const DIAL = 220;
const DIAL_C = DIAL / 2;
const DIAL_R = 84;
// Zero points at the floor of the dial rather than out to the right, so the
// needle reads against the arm the way it is stood in front of you.
const DIAL_ZERO = -90;
const NUDGE = 2;
const NUDGE_COARSE = 10;
const NUDGE_MM = 6;
const NUDGE_MM_COARSE = 25;

const rad = (degrees: number) => (degrees * Math.PI) / 180;
const clamp = (value: number, limit: JointLimit) => Math.max(limit.min, Math.min(limit.max, value));
/** Into (-180, 180]. A raw atan2 difference can name the same pose two ways. */
const wrap = (degrees: number) => (((degrees + 180) % 360) + 360) % 360 - 180;

/** One decimal, or an em dash when the joint has no zero to measure from. */
const degreeText = (value: number | null | undefined, empty: string) =>
  typeof value === "number" && Number.isFinite(value) ? `${value.toFixed(1)}°` : empty;

/**
 * Park a dial angle at the end it is actually nearest, measured the short way.
 *
 * Plain numeric clamping is wrong on a circle. Dragging past the top of a
 * narrow wedge, `wrap` flips +179 to -179 in one pixel, and clamping that picks
 * the *smaller-numbered* end — so the needle jumped straight from max to min
 * mid-drag. The camera's wedge is only ~156 degrees wide, so the 204-degree
 * dead zone outside it contains that flip point and it was easy to land on.
 */
const clampAround = (value: number, limit: JointLimit) => {
  if (value >= limit.min && value <= limit.max) return value;
  const gap = (edge: number) => Math.abs(wrap(value - edge));
  return gap(limit.min) <= gap(limit.max) ? limit.min : limit.max;
};

/** Screen angle of a pointer about a pivot, in the twin's frame (y is up). */
const angleAt = (cx: number, cy: number, px: number, py: number) =>
  (Math.atan2(-(py - cy), px - cx) * 180) / Math.PI;

/**
 * Pointer to viewBox units.
 *
 * An SVG with no `preserveAspectRatio` renders `xMidYMid meet`: ONE uniform
 * scale, drawing centred, letterbox bands on the axis with room to spare. Mapping
 * the bounding box onto the viewBox with independent x and y factors therefore
 * lands off the cursor by however wide those bands are — which is most of the
 * time, since the canvas is flex-sized and its aspect ratio is nobody's choice.
 *
 * Deliberately not `getScreenCTM`, which jsdom does not implement. The zero-size
 * rect jsdom does report collapses this to the identity, which is what the tests
 * lean on — so a regression test has to stub a real rect.
 */
function inSvg(svg: SVGSVGElement, clientX: number, clientY: number, width: number, height: number) {
  const bounds = svg.getBoundingClientRect();
  const boxWidth = bounds.width || width;
  const boxHeight = bounds.height || height;
  const drawn = Math.min(boxWidth / width, boxHeight / height);
  return {
    x: (clientX - bounds.left - (boxWidth - width * drawn) / 2) / drawn,
    y: (clientY - bounds.top - (boxHeight - height * drawn) / 2) / drawn,
  };
}

function arc(cx: number, cy: number, r: number, from: number, to: number) {
  const at = (angle: number) =>
    `${(cx + r * Math.cos(rad(angle))).toFixed(1)},${(cy - r * Math.sin(rad(angle))).toFixed(1)}`;
  // Screen y runs down, so a counter-clockwise sweep in the twin's frame is
  // sweep-flag 0 here.
  return `M${cx},${cy} L${at(from)} A${r},${r} 0 ${Math.abs(to - from) > 180 ? 1 : 0} 0 ${at(to)} Z`;
}

export function Arm2D({ pose, measuredPose = pose, planned = false, sequencePoses = [], disabled = false, limits, geometry, floorMm = null, onPose, onRelease, onInteractionStart, onInteractionEnd, showDials = true, interactionCopy = "Drag to preview. Release prepares the dashed plan; the solid arm is measured telemetry. This 2D draft moves only when you press Apply move.", emptyReadout = "—", extras = [], readouts = {} }: Arm2DProps) {
  // Keyed by pointer id: a second finger landing on the canvas must not steal or
  // end the drag the first one is in the middle of.
  const drag = useRef<{ pointerId: number; handle: Handle } | null>(null);
  const keyboardActive = useRef(false);

  const distalMm = geometry.forearmMm + geometry.toolOffsetMm;
  const reachUp = geometry.baseHeightMm + geometry.upperArmMm + distalMm;
  const reachOut = geometry.upperArmMm + distalMm;
  // Fit whichever way the arm is pointing, so a long forearm cannot draw itself
  // off the canvas.
  const scale = Math.min((PIVOT_Y - 26) / reachUp, (SIDE_W / 2 - 26) / reachOut);

  const shoulder = { x: PIVOT_X, y: PIVOT_Y - geometry.baseHeightMm * scale };
  const pointsFor = (candidate: ArmPose) => {
    const elbow = {
      x: shoulder.x + geometry.upperArmMm * scale * Math.cos(rad(candidate.joint_2)),
      y: shoulder.y - geometry.upperArmMm * scale * Math.sin(rad(candidate.joint_2)),
    };
    return {
      elbow,
      tip: {
        x: elbow.x + distalMm * scale * Math.cos(rad(candidate.joint_2 + candidate.joint_3)),
        y: elbow.y - distalMm * scale * Math.sin(rad(candidate.joint_2 + candidate.joint_3)),
      },
    };
  };
  const { elbow, tip } = pointsFor(pose);
  const measuredPoints = pointsFor(measuredPose);
  const sequencePoints = sequencePoses.map((candidate) => ({
    pose: candidate,
    ...pointsFor(candidate),
  }));
  const needle = (degrees: number, radius: number) => ({
    x: DIAL_C + radius * Math.cos(rad(degrees + DIAL_ZERO)),
    y: DIAL_C - radius * Math.sin(rad(degrees + DIAL_ZERO)),
  });

  /**
   * Shoulder and elbow for a tip in millimetres from the shoulder pivot.
   *
   * Both elbow branches reach the same point; the one that needs least clamping
   * wins, and staying near the current pose breaks the tie so the elbow does not
   * flip sides halfway through a drag.
   */
  const solveTipRaw = (radial: number, height: number) => {
    const cost = (branch: PlanarSolution) =>
      Math.abs(clamp(branch.shoulder, limits.joint_2) - branch.shoulder) +
      Math.abs(clamp(branch.elbow, limits.joint_3) - branch.elbow) +
      0.02 * (Math.abs(branch.shoulder - pose.joint_2) + Math.abs(branch.elbow - pose.joint_3));
    const branches = planarInverse(radial, height, geometry);
    const best = cost(branches[0]) <= cost(branches[1]) ? branches[0] : branches[1];
    return {
      joint_2: clamp(best.shoulder, limits.joint_2),
      joint_3: clamp(best.elbow, limits.joint_3),
    };
  };

  /**
   * The floor guard, applied to the drag itself rather than to the command it
   * produces: the handle stops dead at the plane and slides along it, instead
   * of the arm taking the pose and something downstream complaining afterwards.
   *
   * Applied to every patch rather than only to tip drags, because dragging the
   * shoulder segment buries the elbow just as effectively as dragging the tip.
   * Patches that cannot change a height -- the base yaw and the camera -- pass
   * straight through, since `lowestPointMm` ignores those joints anyway.
   *
   * The bar is `min(floorMm, where the arm already is)`, so a pose already below
   * the plane -- guard switched on while low, or the arm moved by hand -- can
   * still be dragged, just never any lower, and the bar ratchets back up to the
   * plane as the arm climbs. A flat `>= floorMm` would lock the controls solid
   * exactly when they are needed to get out.
   *
   * Bisecting toward the current pose rather than solving the boundary
   * algebraically is deliberate: one predicate covers an elbow violation, a tip
   * violation and both at once, and it cannot return an illegal pose even if
   * the algebra would have been wrong.
   */
  const guardFloor = (patch: Partial<Record<SimpleJointId, number>>) => {
    if (floorMm == null) return patch;
    const blend = (t: number) =>
      Object.fromEntries(
        Object.entries(patch).map(([joint, value]) => {
          const from = pose[joint as ArmJointId];
          return [joint, from === undefined ? value : from + (value - from) * t];
        }),
      ) as Partial<Record<SimpleJointId, number>>;
    const lowest = (candidate: Partial<Record<SimpleJointId, number>>) =>
      lowestPointMm({ ...pose, ...candidate }, geometry);
    const bar = Math.min(floorMm, lowest({}));
    if (lowest(patch) >= bar) return patch;
    // t = 0 is where the arm already is, which meets the bar by construction,
    // so this always has an answer to fall back on.
    let reachable = 0;
    let blocked = 1;
    for (let step = 0; step < 12; step += 1) {
      const middle = (reachable + blocked) / 2;
      if (lowest(blend(middle)) >= bar) reachable = middle;
      else blocked = middle;
    }
    return blend(reachable);
  };

  const tipMm = { radial: (tip.x - shoulder.x) / scale, height: (shoulder.y - tip.y) / scale };

  /** Every dial behaves the same; only the base's angle comes from the pose. */
  const dialOf = (handle: Handle): { value: number; limit: JointLimit } | undefined => {
    if (handle === "joint_1") return { value: pose.joint_1, limit: limits.joint_1 };
    const extra = extras.find((candidate) => candidate.id === handle);
    return extra && { value: extra.degrees, limit: extra.limit };
  };

  const poseFor = (handle: Handle, point: { x: number; y: number }) => {
    if (handle === "tip") {
      return guardFloor(solveTipRaw((point.x - shoulder.x) / scale, (shoulder.y - point.y) / scale));
    }
    const dial = dialOf(handle);
    if (dial) return { [handle]: clampAround(wrap(angleAt(DIAL_C, DIAL_C, point.x, point.y) - DIAL_ZERO), dial.limit) };
    return guardFloor({ [handle]: clamp(angleAt(shoulder.x, shoulder.y, point.x, point.y), limits.joint_2) });
  };

  const startDrag = (handle: Handle) => (event: PointerEvent<SVGElement>) => {
    if (drag.current) return;
    event.preventDefault();
    event.stopPropagation();
    drag.current = { pointerId: event.pointerId, handle };
    onInteractionStart?.();
    event.currentTarget.setPointerCapture?.(event.pointerId);
    // preventDefault suppresses Chromium's focus-on-press, which left the arrow
    // keys this pane advertises unreachable for anyone using a pointer.
    event.currentTarget.focus?.();
  };

  const moveDrag = (width: number, height: number) => (event: PointerEvent<SVGSVGElement>) => {
    const active = drag.current;
    if (!active || active.pointerId !== event.pointerId) return;
    onPose(poseFor(active.handle, inSvg(event.currentTarget, event.clientX, event.clientY, width, height)));
  };

  const endDrag = (reason: "release" | "cancel") => (event: PointerEvent<SVGSVGElement>) => {
    const active = drag.current;
    // Guard on the pointer, not just on there being a drag: an unguarded release
    // would re-send the last goal on any stray click or second finger.
    if (!active || active.pointerId !== event.pointerId) return;
    drag.current = null;
    if (event.currentTarget.hasPointerCapture?.(event.pointerId)) {
      event.currentTarget.releasePointerCapture?.(event.pointerId);
    }
    onRelease();
    onInteractionEnd?.(reason);
  };

  const nudge = (handle: Handle) => (event: KeyboardEvent<SVGElement>) => {
    const step = { ArrowLeft: -1, ArrowDown: -1, ArrowRight: 1, ArrowUp: 1 }[event.key];
    if (step === undefined) return;
    event.preventDefault();
    if (!keyboardActive.current) {
      keyboardActive.current = true;
      onInteractionStart?.();
    }
    if (handle === "tip") {
      const out = event.key === "ArrowLeft" || event.key === "ArrowRight";
      const far = event.shiftKey ? NUDGE_MM_COARSE : NUDGE_MM;
      // Guarded like the drag: arrow keys are another way to ask for a pose,
      // not an exemption from the plane.
      onPose(guardFloor(solveTipRaw(tipMm.radial + (out ? step * far : 0), tipMm.height + (out ? 0 : step * far))));
      return;
    }
    const far = event.shiftKey ? NUDGE_COARSE : NUDGE;
    const dial = dialOf(handle);
    const from = dial ? dial.value : pose[handle as ArmJointId];
    const limit = dial ? dial.limit : limits[handle as ArmJointId];
    onPose(guardFloor({ [handle]: clamp(from + step * far, limit) }));
  };

  /** Held arrows preview the whole way and send once, same as a drag. */
  const release = (event: KeyboardEvent<SVGElement>) => {
    if (!event.key.startsWith("Arrow") || !keyboardActive.current) return;
    keyboardActive.current = false;
    onRelease();
    onInteractionEnd?.("release");
  };

  const grip = (handle: Handle, label: string) => disabled ? {
    tabIndex: -1,
    "aria-label": label,
    "aria-disabled": true,
  } : {
    tabIndex: 0,
    "aria-label": label,
    onPointerDown: startDrag(handle),
    onKeyDown: nudge(handle),
    onKeyUp: release,
  };

  const jointGrip = (joint: Handle, label: string, value: number, limit: JointLimit) => ({
    ...grip(joint, label),
    role: "slider",
    "aria-valuemin": Math.round(limit.min),
    "aria-valuemax": Math.round(limit.max),
    "aria-valuenow": Math.round(value),
    "aria-valuetext": `${value.toFixed(1)} degrees`,
  });

  return (
    <section className={`arm2d${disabled ? " is-disabled" : ""}${showDials ? "" : " is-planar-only"}`} aria-label="Flat arm control">
      <div className="arm2d-pane">
        <h3>Side view</h3>
        <svg
          className="arm2d-side"
          viewBox={`0 0 ${SIDE_W} ${SIDE_H}`}
          onPointerMove={moveDrag(SIDE_W, SIDE_H)}
          onPointerUp={endDrag("release")}
          onPointerCancel={endDrag("cancel")}
          onLostPointerCapture={endDrag("cancel")}
        >
          <line className="arm2d-floor" x1="20" y1={PIVOT_Y} x2={SIDE_W - 20} y2={PIVOT_Y} />
          {/* The keep-out region, drawn so the barrier is visible rather than
              just felt: an arm that stops for no shown reason reads as a bug. */}
          {floorMm === null ? null : (
            <g aria-hidden="true">
              <rect
                className="arm2d-keepout"
                x="20"
                y={PIVOT_Y - floorMm * scale}
                width={SIDE_W - 40}
                height={Math.max(0, SIDE_H - (PIVOT_Y - floorMm * scale))}
              />
              <line
                className="arm2d-keepout-edge"
                x1="20"
                y1={PIVOT_Y - floorMm * scale}
                x2={SIDE_W - 20}
                y2={PIVOT_Y - floorMm * scale}
              />
            </g>
          )}
          {/* Everywhere the tip can be put: outside the far ring the arm is
              straight, inside the near one the links cannot fold that tightly. */}
          <circle className="arm2d-reach" cx={shoulder.x} cy={shoulder.y} r={(geometry.upperArmMm + distalMm) * scale} />
          <circle className="arm2d-reach" cx={shoulder.x} cy={shoulder.y} r={Math.abs(geometry.upperArmMm - distalMm) * scale} />
          <path className="arm2d-range" d={arc(shoulder.x, shoulder.y, geometry.upperArmMm * scale, limits.joint_2.min, limits.joint_2.max)} />
          <line className="arm2d-column" x1={PIVOT_X} y1={PIVOT_Y} x2={shoulder.x} y2={shoulder.y} />
          <g data-testid="arm2d-measured-pose">
            <line className="arm2d-link" x1={shoulder.x} y1={shoulder.y} x2={measuredPoints.elbow.x} y2={measuredPoints.elbow.y} />
            <line className="arm2d-link" x1={measuredPoints.elbow.x} y1={measuredPoints.elbow.y} x2={measuredPoints.tip.x} y2={measuredPoints.tip.y} />
          </g>
          {sequencePoints.length ? (
            <g data-testid="arm2d-sequence-path" aria-label="Reviewed waypoint path">
              <polyline
                className="arm2d-sequence-path"
                points={[
                  `${measuredPoints.tip.x},${measuredPoints.tip.y}`,
                  ...sequencePoints.map((points) => `${points.tip.x},${points.tip.y}`),
                ].join(" ")}
                aria-hidden="true"
              />
              {sequencePoints.map((points, index) => (
                <g key={index} role="img" aria-label={`Waypoint ${index + 1}`}>
                  <line className="arm2d-sequence-link" x1={shoulder.x} y1={shoulder.y} x2={points.elbow.x} y2={points.elbow.y} />
                  <line className="arm2d-sequence-link" x1={points.elbow.x} y1={points.elbow.y} x2={points.tip.x} y2={points.tip.y} />
                  <circle className="arm2d-sequence-node" cx={points.tip.x} cy={points.tip.y} r="10" />
                  <text className="arm2d-sequence-number" x={points.tip.x} y={points.tip.y + 3.5} textAnchor="middle" aria-hidden="true">{index + 1}</text>
                </g>
              ))}
            </g>
          ) : null}
          {planned ? (
            <g data-testid="arm2d-planned-pose" aria-label="Planned arm pose">
              <line className="arm2d-plan-link" x1={shoulder.x} y1={shoulder.y} x2={elbow.x} y2={elbow.y} />
              <line className="arm2d-plan-link" x1={elbow.x} y1={elbow.y} x2={tip.x} y2={tip.y} />
            </g>
          ) : null}
          <circle className="arm2d-pivot" cx={shoulder.x} cy={shoulder.y} r="7" />
          <circle className="arm2d-grip" cx={elbow.x} cy={elbow.y} r="14" {...jointGrip("joint_2", "Shoulder angle", pose.joint_2, limits.joint_2)} />
          {/* The free one: drag it anywhere and both joints solve for it. */}
          <circle className="arm2d-grip is-tip" cx={tip.x} cy={tip.y} r="19" role="button" {...grip("tip", "Arm tip, drag to move the whole arm")} />
          {/* Angles on the drawing itself, so the pose can be read without
              looking across to the panel. Offset clear of each handle, and
              aria-hidden because the handles already carry the value. */}
          <text className="arm2d-readout" x={PIVOT_X + 16} y={PIVOT_Y - 12} aria-hidden="true">
            {degreeText(readouts.joint_1, emptyReadout)}
          </text>
          <text className="arm2d-readout" x={shoulder.x - 14} y={shoulder.y - 16} textAnchor="end" aria-hidden="true">
            {degreeText(readouts.joint_2, emptyReadout)}
          </text>
          <text className="arm2d-readout" x={elbow.x + 22} y={elbow.y - 16} aria-hidden="true">
            {degreeText(readouts.joint_3, emptyReadout)}
          </text>
        </svg>
        <p>{interactionCopy}</p>
      </div>

      {/* One scrolling column, however many dials there are. As direct grid
          children they were laid out down the page instead: with the camera
          added, its dial landed below the fold of a 720px window in a container
          that did not scroll, so it was rendered but simply unreachable. */}
      {showDials ? <div className="arm2d-dials">
      {[
        { id: "joint_1" as Handle, name: "Base rotation", degrees: pose.joint_1, measuredDegrees: measuredPose.joint_1, limit: limits.joint_1, pending: false },
        ...extras,
      ].map((dial) => (
        <div className={`arm2d-pane is-dial${dial.pending ? " is-pending" : ""}`} key={dial.id}>
          <h3>{dial.name}</h3>
          <svg
            className="arm2d-dial"
            viewBox={`0 0 ${DIAL} ${DIAL}`}
            onPointerMove={moveDrag(DIAL, DIAL)}
            onPointerUp={endDrag("release")}
            onPointerCancel={endDrag("cancel")}
            onLostPointerCapture={endDrag("cancel")}
          >
            {/* The wedge is whatever calibration measured for THIS joint, so a
                servo that only sweeps 300 degrees draws 300, not a full circle. */}
            <path className="arm2d-range" d={arc(DIAL_C, DIAL_C, DIAL_R, dial.limit.min + DIAL_ZERO, dial.limit.max + DIAL_ZERO)} />
            <circle className="arm2d-dial-face" cx={DIAL_C} cy={DIAL_C} r={DIAL_R} />
            <line className="arm2d-mark" x1={DIAL_C} y1={DIAL_C + DIAL_R - 9} x2={DIAL_C} y2={DIAL_C + DIAL_R + 9} />
            <text className="arm2d-mark-label" x={DIAL_C} y={DIAL_C + DIAL_R + 22} textAnchor="middle">0°</text>
            <line className="arm2d-needle" x1={DIAL_C} y1={DIAL_C} x2={needle(dial.measuredDegrees ?? dial.degrees, DIAL_R).x} y2={needle(dial.measuredDegrees ?? dial.degrees, DIAL_R).y} />
            {planned && Math.abs(dial.degrees - (dial.measuredDegrees ?? dial.degrees)) > .05 ? (
              <line className="arm2d-plan-needle" x1={DIAL_C} y1={DIAL_C} x2={needle(dial.degrees, DIAL_R).x} y2={needle(dial.degrees, DIAL_R).y} />
            ) : null}
            <circle
              className="arm2d-grip"
              cx={needle(dial.degrees, DIAL_R).x}
              cy={needle(dial.degrees, DIAL_R).y}
              r="15"
              {...(dial.pending ? {} : jointGrip(dial.id, dial.name, dial.degrees, dial.limit))}
            />
            <circle className="arm2d-pivot" cx={DIAL_C} cy={DIAL_C} r="6" />
          </svg>
          <output>{dial.pending ? "not calibrated" : `${dial.degrees.toFixed(1)}°`}</output>
        </div>
      ))}
      </div> : null}
    </section>
  );
}
