# Public AI context

This file is the public, privacy-safe operating context for an AI working with
co-arm. It preserves stable engineering facts and the required evidence model.
It intentionally excludes personal memory, credentials, private paths, network
addresses, controller/session/frame identifiers, and live hardware state.

## First-read order

For setup, Raspberry Pi connection, assembly, or commissioning, begin with
[`SETUP_WITH_CODEX.md`](SETUP_WITH_CODEX.md). It defines the shared human/AI
checklist, the current documentation-only software boundary, and the allowed
read-only Pi discovery steps.

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
- Base is yaw and uses a 4:1 geared, native absolute multi-turn path in the
  reference firmware 2.4 implementation.
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
- The Pi owns calibration, geometry, limits, floor policy, plans, and camera
  evidence. The ESP32 owns servo timing, watchdog, torque authority, STOP, and
  torque-off supervision.

## Required evidence language

Use these proof tiers explicitly:

| Claim | Required evidence |
| --- | --- |
| Source behavior | Current source inspection |
| Tested branch | Named test result |
| Built artifact | Successful build for that artifact |
| Deployed service | Verified running process/version after deployment |
| Live pose / status | Fresh `arm_state` telemetry |
| Calculated shape / ray | `arm_scene` from the same live state |
| Command acceptance | Successful move/apply response |
| Physical arrival | Fresh measured state after settling |
| Camera content | Pixels from the named capture |
| Crop content | Pixels from the retained source crop; no new view |
| Focus | Visible subject detail; AF-capable hardware also needs live AF metadata |
| Exact object identity | Close-frame discriminating evidence |
| Safety performance | Controlled physical test; software tests are insufficient |

Never collapse these tiers. In particular, a requested target is not arrival,
a rendered twin is not camera evidence, and a documentation statement is not a
live-device check.

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

Do not clear STOP, disable a guard, flash firmware, or move/restart unsupported
held hardware without explicit operator intent. Use typed Arm tools; do not
invent a raw serial, register, HTTP, or shell control path.

## Vision contract

For a desk search or identification:

1. Read live state and scene.
2. Plan/apply an elevated, retracted, downward-looking survey with Base exactly
   `0 deg`.
3. Verify measured arrival, then capture a survey.
4. Require pixels to show the useful desk; pose alone is not wide-view proof.
5. Crop the retained full source to localize without moving or recapturing.
6. Plan/apply a centered close pass, verify, capture, and inspect again.
7. Rank focus by subject-region detail. The installed reference camera is
   fixed-focus; moving closer is not autofocus and can make the image softer.
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
- whether exported CAD/STL files match the assembled hardware revision;
- whether a named viewpoint has been reverified after camera/bracket changes.

## Public-reporting style

Lead with the result. Keep routine updates concise. State exactly which layer
was checked and which physical/manual checks remain. Never imply endorsement,
certification, autonomy, or precision beyond the evidence.
