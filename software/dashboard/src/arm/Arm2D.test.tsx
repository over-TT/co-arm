import { fireEvent, render, screen, within } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { Arm2D } from "./Arm2D";
import { DEFAULT_GEOMETRY, type ArmJointId } from "./armTypes";

const WIDE = { min: -180, max: 180 };

function draw(overrides: {
  pose?: Partial<Record<ArmJointId, number>>;
  measuredPose?: Partial<Record<ArmJointId, number>>;
  planned?: boolean;
  sequencePoses?: Array<Record<ArmJointId, number>>;
  limits?: Partial<Record<ArmJointId, { min: number; max: number }>>;
  floorMm?: number | null;
  showDials?: boolean;
  interactionCopy?: string;
  emptyReadout?: string;
  onInteractionStart?: () => void;
  onInteractionEnd?: (reason: "release" | "cancel") => void;
} = {}) {
  const onPose = vi.fn();
  const onRelease = vi.fn();
  render(
    <Arm2D
      pose={{ joint_1: 0, joint_2: 90, joint_3: -90, ...overrides.pose }}
      measuredPose={{ joint_1: 0, joint_2: 90, joint_3: -90, ...overrides.measuredPose }}
      planned={overrides.planned}
      sequencePoses={overrides.sequencePoses}
      limits={{ joint_1: WIDE, joint_2: WIDE, joint_3: WIDE, ...overrides.limits }}
      geometry={DEFAULT_GEOMETRY}
      floorMm={overrides.floorMm ?? null}
      onPose={onPose}
      onRelease={onRelease}
      showDials={overrides.showDials}
      interactionCopy={overrides.interactionCopy}
      emptyReadout={overrides.emptyReadout}
      onInteractionStart={overrides.onInteractionStart}
      onInteractionEnd={overrides.onInteractionEnd}
    />,
  );
  return { onPose, onRelease };
}

// jsdom reports a zero-size box, so Arm2D falls back to a 1:1 viewBox scale and
// client coordinates are viewBox coordinates. The shoulder pivot sits on the
// centre line at x=240.
const SHOULDER_X = 240;

function dragTo(name: string, clientX: number, clientY: number) {
  // The tip is a button (it moves two joints, so it has no single value);
  // everything else is a slider.
  const grip = screen.getByRole(name.startsWith("Arm tip") ? "button" : "slider", { name });
  fireEvent.pointerDown(grip, { pointerId: 1 });
  fireEvent.pointerMove(grip, { pointerId: 1, clientX, clientY });
}

describe("Arm2D", () => {
  it("keeps telemetry solid and renders a planned pose as a separate dashed overlay", () => {
    draw({
      measuredPose: { joint_2: 90, joint_3: -90 },
      pose: { joint_2: 45, joint_3: -20 },
      planned: true,
    });

    const measured = screen.getByTestId("arm2d-measured-pose");
    const planned = screen.getByTestId("arm2d-planned-pose");
    expect(measured.querySelectorAll(".arm2d-link")).toHaveLength(2);
    expect(planned.querySelectorAll(".arm2d-plan-link")).toHaveLength(2);
    expect(planned).toHaveAttribute("aria-label", "Planned arm pose");
  });

  it("renders every reviewed waypoint as one numbered whole-path overlay", () => {
    draw({
      measuredPose: { joint_2: 90, joint_3: -90 },
      sequencePoses: [
        { joint_1: 10, joint_2: 70, joint_3: -60 },
        { joint_1: -15, joint_2: 35, joint_3: -10 },
      ],
    });

    const path = screen.getByTestId("arm2d-sequence-path");
    expect(path.querySelectorAll(".arm2d-sequence-link")).toHaveLength(4);
    expect(path.querySelectorAll(".arm2d-sequence-node")).toHaveLength(2);
    expect(within(path).getByLabelText("Waypoint 1")).toBeInTheDocument();
    expect(within(path).getByLabelText("Waypoint 2")).toBeInTheDocument();
    expect(screen.getByTestId("arm2d-measured-pose")).toBeInTheDocument();
  });

  it("turns a drag on the shoulder grip into that joint's angle and nothing else", () => {
    const { onPose } = draw();

    // Straight up from the shoulder pivot is 90 degrees in the twin's frame.
    dragTo("Shoulder angle", SHOULDER_X, 60);

    expect(onPose).toHaveBeenLastCalledWith({ joint_2: expect.closeTo(90, 1) });
  });

  it("solves both joints from the tip, so the arm is dragged as one thing", () => {
    const { onPose } = draw({ pose: { joint_2: 90, joint_3: -90 } });

    dragTo("Arm tip, drag to move the whole arm", SHOULDER_X + 60, 120);

    const [[sent]] = onPose.mock.calls.slice(-1);
    expect(Object.keys(sent).sort()).toEqual(["joint_2", "joint_3"]);
    // Whatever it chose has to actually put the tip where the pointer was.
    // Derived, not hard-coded: the scale falls out of the measured link
    // lengths, so a re-measured arm should not look like a broken drag solver.
    const distal = DEFAULT_GEOMETRY.forearmMm + DEFAULT_GEOMETRY.toolOffsetMm;
    const scale = Math.min(
      (342 - 26) / (DEFAULT_GEOMETRY.baseHeightMm + DEFAULT_GEOMETRY.upperArmMm + distal),
      (480 / 2 - 26) / (DEFAULT_GEOMETRY.upperArmMm + distal),
    );
    const shoulderY = 342 - DEFAULT_GEOMETRY.baseHeightMm * scale;
    const rad = (d: number) => (d * Math.PI) / 180;
    const x = SHOULDER_X
      + DEFAULT_GEOMETRY.upperArmMm * scale * Math.cos(rad(sent.joint_2))
      + distal * scale * Math.cos(rad(sent.joint_2 + sent.joint_3));
    const y = shoulderY
      - DEFAULT_GEOMETRY.upperArmMm * scale * Math.sin(rad(sent.joint_2))
      - distal * scale * Math.sin(rad(sent.joint_2 + sent.joint_3));
    expect(x).toBeCloseTo(SHOULDER_X + 60, 0);
    expect(y).toBeCloseTo(120, 0);
  });

  it("straightens the arm toward a tip dragged past its reach instead of refusing", () => {
    // A hand dragging the tool wants the arm to follow as far as it goes. The
    // 3D view's "out of reach" is the behaviour this replaced.
    const { onPose } = draw({ pose: { joint_2: 90, joint_3: -90 } });

    dragTo("Arm tip, drag to move the whole arm", SHOULDER_X + 470, 40);

    const [[sent]] = onPose.mock.calls.slice(-1);
    expect(sent.joint_3).toBeCloseTo(0, 1); // elbow straight
    expect(Number.isFinite(sent.joint_2)).toBe(true);
  });

  it("puts the dial's zero at the bottom", () => {
    const { onPose } = draw({ pose: { joint_1: 0 } });

    // Straight down from the dial centre now reads zero, not -90.
    dragTo("Base rotation", 110, 200);

    expect(onPose).toHaveBeenLastCalledWith({ joint_1: expect.closeTo(0, 1) });
  });

  it("clamps a drag to the joint's limits instead of sending an angle it cannot hold", () => {
    const { onPose } = draw({ limits: { joint_2: { min: -30, max: 45 } } });

    dragTo("Shoulder angle", SHOULDER_X, 60);

    expect(onPose).toHaveBeenLastCalledWith({ joint_2: 45 });
  });

  it("does not move a joint until a grip has actually been grabbed", () => {
    const { onPose } = draw();

    fireEvent.pointerMove(screen.getByRole("slider", { name: "Base rotation" }), { pointerId: 1, clientX: 10, clientY: 10 });

    expect(onPose).not.toHaveBeenCalled();
  });

  it("previews the whole drag but only asks for the move on release", () => {
    // The arm gets one goal, at the angle the handle was let go at. Streaming
    // every intermediate angle is what made the drag feel heavy.
    const { onPose, onRelease } = draw();
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: SHOULDER_X, clientY: 200 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: SHOULDER_X, clientY: 60 });
    expect(onPose.mock.calls.length).toBeGreaterThan(1);
    expect(onRelease).not.toHaveBeenCalled();

    fireEvent.pointerUp(grip, { pointerId: 1 });
    expect(onRelease).toHaveBeenCalledTimes(1);
  });

  it("supports a planar-only streaming owner with distinct release and cancel lifecycle", () => {
    const onInteractionStart = vi.fn();
    const onInteractionEnd = vi.fn();
    draw({
      showDials: false,
      interactionCopy: "Ghost follows locally. Solid arm is measured telemetry.",
      emptyReadout: "n/a",
      onInteractionStart,
      onInteractionEnd,
    });
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    expect(screen.queryByRole("slider", { name: "Base rotation" })).not.toBeInTheDocument();
    expect(screen.getByText("Ghost follows locally. Solid arm is measured telemetry.")).toBeInTheDocument();
    expect([...document.querySelectorAll(".arm2d-readout")].map((node) => node.textContent)).toEqual(["n/a", "n/a", "n/a"]);

    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerCancel(grip, { pointerId: 1 });
    expect(onInteractionStart).toHaveBeenCalledTimes(1);
    expect(onInteractionEnd).toHaveBeenLastCalledWith("cancel");

    fireEvent.pointerDown(grip, { pointerId: 2 });
    fireEvent.pointerUp(grip, { pointerId: 2 });
    expect(onInteractionEnd).toHaveBeenLastCalledWith("release");
  });

  it("cancels a drag when pointer capture is lost and accepts the next pointer", () => {
    const onInteractionStart = vi.fn();
    const onInteractionEnd = vi.fn();
    const { onPose, onRelease } = draw({ onInteractionStart, onInteractionEnd });
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: SHOULDER_X, clientY: 60 });
    fireEvent.lostPointerCapture(grip, { pointerId: 1 });

    expect(onInteractionEnd).toHaveBeenLastCalledWith("cancel");
    expect(onRelease).toHaveBeenCalledTimes(1);

    onPose.mockClear();
    fireEvent.pointerDown(grip, { pointerId: 2 });
    fireEvent.pointerMove(grip, { pointerId: 2, clientX: SHOULDER_X, clientY: 200 });
    fireEvent.pointerUp(grip, { pointerId: 2 });

    expect(onInteractionStart).toHaveBeenCalledTimes(2);
    expect(onPose).toHaveBeenCalled();
    expect(onInteractionEnd).toHaveBeenLastCalledWith("release");
  });

  it("does not re-send the last goal when the canvas is clicked without a drag", () => {
    const { onRelease } = draw();

    fireEvent.pointerUp(screen.getByRole("slider", { name: "Shoulder angle" }), { pointerId: 1 });

    expect(onRelease).not.toHaveBeenCalled();
  });

  it("sends once after a keyboard nudge, on key release", () => {
    const { onPose, onRelease } = draw({ pose: { joint_2: 20 } });
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.keyDown(grip, { key: "ArrowUp" });
    expect(onRelease).not.toHaveBeenCalled();
    fireEvent.keyUp(grip, { key: "ArrowUp" });

    expect(onPose).toHaveBeenCalledTimes(1);
    expect(onRelease).toHaveBeenCalledTimes(1);
  });

  it("drives the base from the dial, quarter turn from the bottom", () => {
    const { onPose } = draw({ pose: { joint_1: 0 } });

    // Left of the dial centre is a quarter turn on from the zero at the bottom.
    dragTo("Base rotation", 20, 110);

    expect(onPose).toHaveBeenLastCalledWith({ joint_1: expect.closeTo(-90, 1) });
  });

  it("is drivable from the keyboard, coarsely with shift", () => {
    const { onPose } = draw({ pose: { joint_2: 20 } });
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.keyDown(grip, { key: "ArrowUp" });
    expect(onPose).toHaveBeenLastCalledWith({ joint_2: 22 });

    fireEvent.keyDown(grip, { key: "ArrowDown", shiftKey: true });
    expect(onPose).toHaveBeenLastCalledWith({ joint_2: 10 });

    onPose.mockClear();
    fireEvent.keyDown(grip, { key: "Tab" });
    expect(onPose).not.toHaveBeenCalled();
  });

  it("maps the pointer through the letterbox, not through the bounding box", () => {
    // The canvas is flex-sized, so its box is almost never 480:380. With no
    // preserveAspectRatio the drawing renders "meet": one uniform scale, centred,
    // with bands down the roomy axis. Scaling x and y independently lands the
    // drag off the cursor by the width of those bands — invisible to every other
    // test here, because jsdom's zero-size rect collapses the maths to identity.
    const { onPose } = draw({ pose: { joint_2: 0 } });
    const svg = document.querySelector(".arm2d-side") as SVGSVGElement;
    // 960x380 box for a 480x380 viewBox: scale 1, 240px of band each side.
    vi.spyOn(svg, "getBoundingClientRect").mockReturnValue({
      left: 0, top: 0, width: 960, height: 380, right: 960, bottom: 380, x: 0, y: 0, toJSON: () => ({}),
    } as DOMRect);

    // Straight up from the shoulder pivot, in client pixels: viewBox x 240 is
    // drawn at 240 + 240 = 480.
    dragTo("Shoulder angle", 480, 60);

    expect(onPose).toHaveBeenLastCalledWith({ joint_2: expect.closeTo(90, 1) });
  });

  it("ignores a second finger instead of letting it hijack or end the drag", () => {
    const { onPose, onRelease } = draw();
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: SHOULDER_X, clientY: 60 });
    const claimed = onPose.mock.calls.length;

    // A stray second pointer: it must not steal the handle...
    fireEvent.pointerDown(screen.getByRole("button", { name: /Arm tip/ }), { pointerId: 2 });
    fireEvent.pointerMove(grip, { pointerId: 2, clientX: 20, clientY: 300 });
    expect(onPose.mock.calls.length).toBe(claimed);

    // ...nor end the drag, which would fire a goal the hand never asked for.
    fireEvent.pointerUp(grip, { pointerId: 2 });
    expect(onRelease).not.toHaveBeenCalled();

    fireEvent.pointerUp(grip, { pointerId: 1 });
    expect(onRelease).toHaveBeenCalledTimes(1);
  });

  it("focuses the grip it is given, so the arrow keys it advertises are reachable", () => {
    draw();
    const grip = screen.getByRole("slider", { name: "Shoulder angle" });

    fireEvent.pointerDown(grip, { pointerId: 1 });

    // preventDefault on pointerdown suppresses focus-on-press in Chromium, which
    // left the documented arrow keys dead for anyone using a mouse.
    expect(document.activeElement).toBe(grip);
  });

  it("reports each joint's range to assistive technology", () => {
    draw({ pose: { joint_2: 33 }, limits: { joint_2: { min: -30, max: 45 } } });

    const grip = screen.getByRole("slider", { name: "Shoulder angle" });
    expect(grip).toHaveAttribute("aria-valuemin", "-30");
    expect(grip).toHaveAttribute("aria-valuemax", "45");
    expect(grip).toHaveAttribute("aria-valuenow", "33");
  });
});

describe("Arm2D dials", () => {
  const NARROW = { min: -78, max: 78 };

  function drawWithCamera() {
    const onPose = vi.fn();
    render(
      <Arm2D
        pose={{ joint_1: 0, joint_2: 90, joint_3: -90 }}
        limits={{ joint_1: WIDE, joint_2: WIDE, joint_3: WIDE }}
        geometry={DEFAULT_GEOMETRY}
        onPose={onPose}
        onRelease={vi.fn()}
        extras={[{ id: "joint_4", name: "Camera", degrees: 60, limit: NARROW }]}
      />,
    );
    return { onPose };
  }

  it("parks a dial at the end it is nearest instead of flipping to the other one", () => {
    // The dial is 220 wide, centre (110,110), and 0 degrees points DOWN.
    // Dragging up-and-slightly-past the max end lands outside the +/-78 wedge.
    // Numeric clamping after wrap() sent that to the MIN end, because crossing
    // +/-180 turns +179 into -179 in one pixel -- the needle jumped the full
    // width of the wedge mid-drag.
    const { onPose } = drawWithCamera();

    // Straight up from the dial centre is 180 degrees from the zero at the
    // bottom: the exact point the old maths flipped on.
    const grip = screen.getByRole("slider", { name: "Camera" });
    fireEvent.pointerDown(grip, { pointerId: 1 });
    fireEvent.pointerMove(grip, { pointerId: 1, clientX: 110, clientY: 10 });

    const [[sent]] = onPose.mock.calls.slice(-1);
    // Whichever end it picks, it must be AN end -- never a value outside the
    // wedge, and never a silent jump across it.
    expect([NARROW.min, NARROW.max]).toContain(sent.joint_4);
  });

  it("keeps every dial in one scrolling column so an added joint stays reachable", () => {
    drawWithCamera();

    const strip = document.querySelector(".arm2d-dials");
    expect(strip).not.toBeNull();
    // Base and camera both live in the strip; as loose grid children the second
    // one flowed off the bottom of the window with nothing to scroll.
    expect(strip!.querySelectorAll(".arm2d-pane.is-dial")).toHaveLength(2);
  });
});

describe("Arm2D readouts", () => {
  it("prints each joint's angle on the drawing, in the numbers the panel shows", () => {
    render(
      <Arm2D
        pose={{ joint_1: 0, joint_2: 90, joint_3: -90 }}
        limits={{ joint_1: WIDE, joint_2: WIDE, joint_3: WIDE }}
        geometry={DEFAULT_GEOMETRY}
        onPose={vi.fn()}
        onRelease={vi.fn()}
        // Servo degrees, deliberately different from the twin-frame pose above:
        // a readout taken from `pose` would print 90 and -90 and disagree with
        // the joint panel sitting right next to it.
        readouts={{ joint_1: -12.5, joint_2: 0, joint_3: 33.25 }}
      />,
    );

    const texts = [...document.querySelectorAll(".arm2d-readout")].map((n) => n.textContent);
    expect(texts).toEqual(["-12.5°", "0.0°", "33.3°"]);
  });

  it("shows a dash rather than a number for a joint with no zero", () => {
    render(
      <Arm2D
        pose={{ joint_1: 0, joint_2: 90, joint_3: -90 }}
        limits={{ joint_1: WIDE, joint_2: WIDE, joint_3: WIDE }}
        geometry={DEFAULT_GEOMETRY}
        onPose={vi.fn()}
        onRelease={vi.fn()}
        readouts={{ joint_1: null, joint_2: 5, joint_3: null }}
      />,
    );

    const texts = [...document.querySelectorAll(".arm2d-readout")].map((n) => n.textContent);
    expect(texts).toEqual(["—", "5.0°", "—"]);
  });
});

describe("Arm2D floor guard", () => {
  const lowest = (patch: Record<string, number>, pose: Record<string, number>) => {
    const { baseHeightMm, upperArmMm, forearmMm, toolOffsetMm } = DEFAULT_GEOMETRY;
    const full = { ...pose, ...patch } as Record<string, number>;
    const rad = (d: number) => (d * Math.PI) / 180;
    const elbow = baseHeightMm + upperArmMm * Math.sin(rad(full.joint_2));
    return Math.min(elbow, elbow + (forearmMm + toolOffsetMm) * Math.sin(rad(full.joint_2 + full.joint_3)));
  };

  it("will not let a drag put any part of the arm below the plane", () => {
    const pose = { joint_1: 0, joint_2: 90, joint_3: -90 };
    const { onPose } = draw({ pose, floorMm: 40 });

    // Straight down, well past the floor.
    dragTo("Arm tip, drag to move the whole arm", SHOULDER_X, 900);

    expect(onPose).toHaveBeenCalled();
    for (const [patch] of onPose.mock.calls) {
      expect(lowest(patch, pose)).toBeGreaterThanOrEqual(40 - 1e-6);
    }
  });

  it("blocks the shoulder grip too, not just the tip", () => {
    const pose = { joint_1: 0, joint_2: 90, joint_3: -90 };
    const { onPose } = draw({ pose, floorMm: 40 });

    dragTo("Shoulder angle", SHOULDER_X, 900);

    expect(onPose).toHaveBeenCalled();
    for (const [patch] of onPose.mock.calls) {
      expect(lowest(patch, pose)).toBeGreaterThanOrEqual(40 - 1e-6);
    }
  });

  it("lets the arm go anywhere once the guard is off", () => {
    const pose = { joint_1: 0, joint_2: 90, joint_3: -90 };
    const { onPose } = draw({ pose, floorMm: null });

    dragTo("Arm tip, drag to move the whole arm", SHOULDER_X, 900);

    expect(onPose).toHaveBeenCalled();
    const wentLow = onPose.mock.calls.some(([patch]) => lowest(patch, pose) < 40);
    expect(wentLow).toBe(true);
  });
});
