# Public AI context

This is the short project context for an agent working on co-arm. It contains
stable build facts, not the arm's current pose or a replacement for reading the
real joints and camera.

## First-read order

For setup, Raspberry Pi connection, assembly, or commissioning, begin with
[`SETUP_WITH_CODEX.md`](SETUP_WITH_CODEX.md). It defines the shared human/AI
checklist, the ARM-only source boundary, and the allowed read-only Pi discovery
steps.

An agent should read, in order:

1. [`SOFTWARE_ARCHITECTURE.md`](SOFTWARE_ARCHITECTURE.md)
2. [`CONTROL_AND_SAFETY.md`](CONTROL_AND_SAFETY.md)
3. [`VISION.md`](VISION.md) for any camera or physical-object task
4. [`COMMISSIONING.md`](COMMISSIONING.md) before setup, calibration, flashing,
   or a first hardware run
5. [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) when live behavior disagrees

Then it must read live telemetry through the typed interface. Documentation is
not the current pose.

## Stable project facts

- co-arm has four driven joints: Base, Shoulder, Elbow, and Camera.
- Base is yaw. Its 3D-printed 52:13 gear pair gives a 4:1 reduction, and the
  Arm HAT reads the servo's native absolute multi-turn position.
- Shoulder and Elbow define the radial/height kinematic chain.
- Camera is driven but excluded from endpoint IK.
- Shoulder servo `0 deg` points the upper arm straight up.
- The arm is straight at Shoulder `0 deg`, Elbow `90 deg`.
- Camera `0 deg` follows the forearm; positive Camera looks down.
- The camera is physically inverted; the Pi delivers new frames upright after
  a 180-degree transform.
- Reference geometry is 60 mm Base pivot height, 180 mm upper arm, and 220 mm
  Elbow-to-tip distance.
- The default floor keep-out plane is 40 mm.
- The current camera is Raspberry Pi Camera Module 3 Wide / IMX708 with powered
  autofocus and bounded survey/detail profiles.
- The Pi handles the camera, calibration, and move planning. The ESP32 talks to
  the servos, reads their positions, and drops the hold if its updates stop.

## Say what actually happened

When reporting a result, say which thing you actually checked:

- code, tests, or a build;
- the process running on the Pi;
- current joint readings;
- the calculated arm drawing;
- the picture from the camera;
- the physical result seen by the operator.

A test result, a joint reading, and a camera image answer different questions.
Use the one that matters for the task.

## Motion contract

Before motion, read current state and scene. Confirm a healthy controller and
bus, trusted required joints, known torque state, STOP clear, floor guard on,
and a clear planned sweep. Prefer `arm_plan` plus `arm_apply_plan`; review exact
angles, warnings, and swept clearance. After applying, settle and read measured
state.

Pause for a real blocker: STOP, offline/untrusted required joint, guard or
clamp, collision signal, unknown torque, lost Base continuity, or a person or
object inside the planned swept volume. A hand visible elsewhere in a camera
frame is not automatically inside that sweep. Stop immediately when the
operator says stop.

Once the user asks for a physical task, do not keep asking for the same task.
Use the normal arm tools and stop only for a real fault, missing physical fact,
or exact tool confirmation. Do not invent a raw serial or register path.

## Vision contract

For a desk search or identification:

1. Read live state and scene.
2. Plan/apply an elevated, retracted, downward-looking survey with Base exactly
   `0 deg`.
3. Verify measured arrival, then capture a survey.
4. Require pixels to show the useful desk; pose alone is not wide-view proof.
5. Crop the retained full source to localize without moving or recapturing.
6. Plan/apply a centered close pass, verify, capture, and inspect again.
7. Rank focus by subject-region detail and require same-capture AF/lens metadata
   for the current autofocus camera. A control request alone is not focus proof.
8. Repeat bounded reviewed viewpoints while evidence is improving. Stop when
   answered, progress is exhausted, a concrete blocker appears, or the operator
   stops.

Do not claim an exact board/object model from a wide candidate view. Require
readable markings, connectors, chips, or another distinctive close-frame cue.

## Durable memory policy

Safe durable memory contains only stable facts such as:

- coordinate and camera conventions;
- verified calibration procedure;
- safety and approval boundaries;
- a repeatable viewpoint as a **destination**, with its calibration/revision
  dependency and evidence date;
- concise operator-authored preferences that contain no personal or secret
  data.

Never store:

- current pose, torque, voltage, temperature, STOP, online/offline, or bus state;
- controller, session, plan, or image-frame identifiers;
- private filesystem paths, addresses, credentials, account metadata, or
  authentication/session databases;
- unverified object identity or guessed calibration;
- a viewpoint as if it described the live pose.

The public repository contains no private operator memory. If a local runtime
bridges a broader memory system, it should read only an explicit allowlisted
summary/registry, bound its size, reject symlink escapes, and never copy auth or
session state into the arm runtime.

## Local facts the operator must supply

The AI must discover or ask for these when relevant; it must not guess them:

- exact power supply, fuse, and physical servo-power cut arrangement;
- confirmed printed servo models and joint mapping;
- current calibration, limits, zero marks, and gear ratios;
- whether the arm is mechanically supported for torque release or restart;
- whether the intended swept volume is clear;
- local gateway endpoint and credentials;
- whether the current 3MF/CAD/STL files match the assembled hardware revision;
- whether a named viewpoint has been reverified after camera/bracket changes.

## Public-reporting style

Lead with the result. Keep routine updates concise and use normal builder
language. Say whether you checked code, ran tests, read the joints, or looked at
the camera. Do not turn routine arm work into a policy lecture.
