# Mechanical design

This document describes the current co-arm mechanical model and the information
needed to turn it into a reproducible build. The exact printed-part geometry,
materials, hardware stack, and assembly tolerances are not yet exported, so this
is deliberately explicit about what is known and what still needs inspection.

## Axis layout

| Axis | Motion | Actuator | Logical ID | Notes |
| --- | --- | --- | ---: | --- |
| Base | Yaw around vertical | ST3215/STS-family | 1 | 4:1 external reduction |
| Shoulder | Upper-arm pitch | ST3215/STS-family | 2 | Drives the 180 mm upper link |
| Elbow | Distal-link pitch | ST3215/STS-family | 3 | Drives the 220 mm elbow-to-tip span |
| Camera | Camera pitch | SC09/SCS-family | 4 | Auxiliary axis; excluded from arm IK |

The base ratio is recorded as **4 motor turns : 1 output turn**. One motor turn
therefore corresponds to 90 degrees of base output motion. Record tooth counts,
gear pitch/module, gear centre distance, backlash, and retention method when the
native CAD is added; the ratio alone is not enough to reproduce the mechanism.

## Reference geometry

All public CAD and drawings use millimetres.

| Symbol | Dimension | Recorded value |
| --- | --- | ---: |
| `H_base` | Work surface to base/shoulder pivot reference | 60 mm |
| `L_upper` | Shoulder pivot to elbow pivot | 180 mm |
| `L_forearm` | Elbow pivot to distal offset origin | 180 mm |
| `L_tool` | Camera/tool offset | 40 mm |
| `L_distal` | Elbow pivot to camera/tool tip | 220 mm |

The software model treats the physical link-plate offset as already included in
these link lengths. Do not add a second offset term without a new measurement
and a coordinated model revision.

Ignoring joint limits, collision envelopes, base structure, and cable limits,
the two-link planar chain has:

- nominal maximum shoulder-to-tip distance: `180 + 220 = 400 mm`;
- nominal inner dead-zone radius: `|220 - 180| = 40 mm`.

Those are geometric calculations, not a certified workspace. Actual reach must
be derived from as-built joint limits, swept-volume checks, link thickness,
camera envelope, wiring, and base anchoring.

## Coordinate and zero conventions

Use a right-handed assembly coordinate system for new CAD:

- origin: centre of the base yaw axis at the work-surface reference plane;
- `+Z`: upward;
- `+X`: forward when Base is at its physical zero mark;
- `+Y`: completes the right-handed frame;
- positive Base motion: document with a drawing after viewing the arm from
  `+Z`; do not infer it from a motor-shaft view.

Current servo-space conventions:

- Base is yaw; its zero depends on the physical base mark.
- Shoulder `0 deg` points the upper arm straight up; negative values tilt it
  forward.
- With Shoulder `0 deg`, Elbow `0 deg` forms the recorded folded-middle/L
  posture; Shoulder `0 deg` plus Elbow `90 deg` is the straight convention used
  by the current model.
- Camera `0 deg` looks along the forearm; positive aims down and negative aims
  up.

These are control conventions, not a replacement for assembly mates. Each CAD
assembly must include named zero-reference planes and a drawing showing the
view direction for positive rotation.

## Mechanical datum and measurement plan

Before final export, create or confirm these datums:

1. `D0_WORK_SURFACE` — underside/base contact plane or the actual bench plane.
2. `A1_BASE_YAW` — Base output axis.
3. `A2_SHOULDER_PITCH` — Shoulder output axis.
4. `A3_ELBOW_PITCH` — Elbow output axis.
5. `A4_CAMERA_PITCH` — Camera-servo output axis.
6. `P_TOOL_TIP` — the camera/tool reference point used for the 220 mm distal
   dimension.
7. `P_BASE_ZERO` — visible mechanical Base-zero alignment mark.

For each measured dimension, record the tool and uncertainty in a drawing note.
Example: `180.0 mm nominal; verify centre-to-centre with calipers, +/-0.5 mm`.
Do not publish a tolerance until it has been chosen for the manufacturing method
and checked against the assembled arm.

## Part breakdown to preserve in CAD

The native assembly should separate at least these functional groups, even if
the exact part names differ:

- base mounting structure;
- Base motor mount, driven gear, pinion, output hub/shaft, bearings/bushings,
  and retaining hardware;
- shoulder servo mount and upper-link structure;
- elbow servo mount and distal-link structure;
- camera-servo mount and camera bracket;
- Raspberry Pi/HAT enclosure or mounting plate;
- power and cable-routing clips/guards;
- purchased hardware represented as reference components.

Do not fuse purchased actuators or fasteners into printable bodies. Suppress
them only for exports where the README explicitly says they are excluded.

## Structural and motion checks before release

- Confirm the base is positively anchored and cannot tip at maximum reach.
- Confirm shafts/horns are captured axially and fasteners cannot back out into a
  moving gear or link.
- Check every joint through its intended range with power removed and links
  supported.
- Measure hard-stop locations separately from software limits.
- Check the complete swept volume, including servo cases, screw heads, camera,
  ribbon cable, connectors, and wire loops.
- Confirm camera ribbon and servo cables retain slack without entering gears or
  pinch points.
- Check backlash and link deflection at multiple reaches before declaring a
  payload.
- Do not infer load capacity from an actuator's marketing torque figure.

## Open mechanical record

- **TODO (owner verification):** exact CAD source application and native file
  format.
- **TODO (owner verification):** printed-part list and which revision is
  currently installed.
- **TODO (owner verification):** filament/resin type, manufacturer, color, and
  any annealing or post-processing.
- **TODO (owner verification):** print orientation, nozzle, layer height, wall
  count, top/bottom layers, infill type/percentage, support settings, and fit
  compensation.
- **TODO (owner verification):** every fastener's standard, thread, length,
  head style, grade/material, quantity, washer/nut/insert, and torque where
  appropriate.
- **TODO (owner verification):** bearing/bushing, shaft, gear, servo-horn, and
  spacer specifications.
- **TODO (owner verification):** actual joint limits, hard-stop clearances,
  base footprint, mounting-hole pattern, overall stowed dimensions, mass, and
  centre of gravity.
- **TODO (owner verification):** repeatable payload and deflection test method.

Place native CAD, neutral STEP, printable STL, and drawings according to
[`CAD_AND_STL.md`](CAD_AND_STL.md).
