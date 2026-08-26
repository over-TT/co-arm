import { fireEvent, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { describe, expect, it, vi } from "vitest";
import { ArmViewport } from "./ArmViewport";
import { DEFAULT_GEOMETRY, DEFAULT_POSE, type CartesianTarget } from "./armTypes";

const TARGET: CartesianTarget = { x: 100, y: 50, z: 200 };

function renderViewport() {
  const onTargetChange = vi.fn();
  render(
    <ArmViewport
      currentPose={DEFAULT_POSE}
      previewPose={DEFAULT_POSE}
      geometry={DEFAULT_GEOMETRY}
      target={TARGET}
      targetReachable
      onTargetChange={onTargetChange}
    />,
  );
  const canvas = screen.getByRole("application") as unknown as SVGSVGElement;
  Object.defineProperty(canvas, "getBoundingClientRect", {
    configurable: true,
    value: () => ({
      x: 0,
      y: 0,
      top: 0,
      left: 0,
      right: 360,
      bottom: 260,
      width: 360,
      height: 260,
      toJSON: () => ({}),
    }),
  });
  return { canvas, onTargetChange };
}

function dragTarget(canvas: SVGSVGElement, start: [number, number], end: [number, number]) {
  const handle = screen.getByRole("button", { name: /target handle/i });
  Object.defineProperty(handle, "setPointerCapture", { configurable: true, value: vi.fn() });
  Object.defineProperty(handle, "hasPointerCapture", { configurable: true, value: vi.fn(() => false) });
  Object.defineProperty(handle, "releasePointerCapture", { configurable: true, value: vi.fn() });
  fireEvent.pointerDown(handle, { pointerId: 7, clientX: start[0], clientY: start[1] });
  fireEvent.pointerMove(canvas, { pointerId: 7, clientX: end[0], clientY: end[1] });
  fireEvent.pointerUp(canvas, { pointerId: 7, clientX: end[0], clientY: end[1] });
}

describe("ArmViewport target controls", () => {
  it("maps scaled target-handle drags onto the selected world plane", async () => {
    const user = userEvent.setup();
    const { canvas, onTargetChange } = renderViewport();

    expect(screen.getByRole("button", { name: /target handle.*XZ plane/i })).toBeInTheDocument();
    dragTarget(canvas, [20, 20], [63, 41.5]);
    expect(onTargetChange).toHaveBeenLastCalledWith({ x: 200, y: 50, z: 150 });

    onTargetChange.mockClear();
    await user.click(screen.getByRole("button", { name: "YZ" }));
    expect(screen.getByRole("button", { name: /target handle.*YZ plane/i })).toBeInTheDocument();
    dragTarget(canvas, [20, 20], [63, 41.5]);
    expect(onTargetChange).toHaveBeenLastCalledWith({ x: 100, y: 150, z: 150 });

    onTargetChange.mockClear();
    await user.click(screen.getByRole("button", { name: "XY" }));
    dragTarget(canvas, [20, 20], [63, 41.5]);
    expect(onTargetChange).toHaveBeenLastCalledWith({ x: 200, y: 0, z: 200 });
  });

  it("does not begin a target drag from empty canvas space", () => {
    const { canvas, onTargetChange } = renderViewport();

    fireEvent.pointerDown(canvas, { pointerId: 3, clientX: 20, clientY: 20 });
    fireEvent.pointerMove(canvas, { pointerId: 3, clientX: 120, clientY: 120 });
    fireEvent.pointerUp(canvas, { pointerId: 3, clientX: 120, clientY: 120 });

    expect(onTargetChange).not.toHaveBeenCalled();
  });

  it("uses the selected plane for keyboard target adjustments", async () => {
    const user = userEvent.setup();
    const { canvas, onTargetChange } = renderViewport();

    await user.click(screen.getByRole("button", { name: "YZ" }));
    fireEvent.keyDown(canvas, { key: "ArrowRight" });
    expect(onTargetChange).toHaveBeenLastCalledWith({ x: 100, y: 52, z: 200 });

    onTargetChange.mockClear();
    fireEvent.keyDown(canvas, { key: "ArrowUp" });
    expect(onTargetChange).toHaveBeenLastCalledWith({ x: 100, y: 50, z: 202 });
  });
});
