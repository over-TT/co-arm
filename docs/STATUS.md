# Status and evidence

Last documentation review: **2026-08-12**.

This page reports evidence, not aspiration. Historical statements are dated;
live state must always be refreshed before operating the arm.

## Current reference-build contract

| Area | Current documented state | Evidence tier |
| --- | --- | --- |
| Mechanism | Four driven joints: Base, Shoulder, Elbow, Camera | Source plus physical observations |
| Kinematics | Camera is auxiliary; Shoulder/Elbow form the planar IK chain | Source and geometry tests |
| Base | ST3215/STS, 4:1 external gearing, native Mode-0 signed absolute goals | Register readback plus operator motion acceptance |
| Shoulder / Elbow | ST3215/STS-family path | Source and live bus observations |
| Camera joint | SC09/SCS-family path, separate byte order and scale | Source tests plus live bus observations |
| Vision | OV5647 fixed-focus 2592 x 1944 still capture, delivered upright | Source tests plus real image capture |
| Firmware | `arm-hat-2.4.0`; advertises `multi_turn_absolute_v1` | Flash identity and live protocol evidence |
| Pi gateway | Authentication, state, camera, calibration, floor geometry, plan/apply | Source tests and deployed endpoint evidence |
| Codex tools | Nine bounded typed tools | Schema/source verification; individual physical proofs vary |
| Named overview | One Base-0 wide desk view was operator-confirmed from pixels | Image and operator evidence; not current-pose evidence |

## Latest read-only live snapshot

On 2026-08-12, a read-only `arm_state` call reported:

- controller connection online;
- servo bus online;
- STOP clear and collision flag clear;
- the floor guard enabled at 40 mm;
- all four joint endpoints online;
- no measured Base angle.

Because the Base angle was unavailable, that snapshot does **not** prove a
complete current pose or trusted Base coordinate. No move, release, plan apply,
capture, deployment, restart, or firmware action occurred during the snapshot.

## Physically verified

- The Pi, camera, HAT, and four servo IDs have operated together.
- The Base servo's native extended-position configuration was read back with
  Phase extended-position bit enabled, resolution 1, zero angle limits, Mode 0,
  EEPROM locked, and torque off.
- Firmware 2.4.0 and the matching Pi control path survived a complete stack
  power cycle.
- Normal Base zero, positive, negative, and return motion was accepted by the
  operator on 2026-08-10.
- Codex has previously moved joints through typed tools and captured real
  camera pixels.
- The OV5647 delivered a full-resolution still and correctly reported that it
  has fixed-focus optics.
- A Base-0 wide overview was confirmed from the returned image.

## Source/test verified, but not yet physical proof

- One-use plan preview and apply checks, including expiry, replay, telemetry
  freshness, start-pose drift, limits, and swept floor clearance.
- The corrected independently timed Shoulder/Elbow sweep calculation.
- Autofocus capability detection and per-frame metadata handling for a future
  motorized camera; the installed OV5647 cannot exercise this.
- The current dashboard Camera-tab behavior and its error states.
- Most gateway, firmware, MCP, and UI contracts under simulation or replay.

## Open physical gates

- Record a numeric Base arrival table across signed moves, seam crossings,
  endpoints, repeated targets, and mid-flight retargets on the final 2.4 path.
- Prove the final path over a longer coordinated reliability run.
- Physically measure the corrected floor-guard clearance model.
- Exercise `arm_plan` followed by `arm_apply_plan` on hardware and record
  measured arrival.
- Confirm a real production-dashboard Camera-tab click.
- Qualify servo-rail sag, current scaling, stall behavior, thermal envelope,
  and simultaneous-load limits.
- Confirm whether a dedicated physical emergency power cut is installed.
- Photograph exact servo, HAT, camera, and power-supply labels.
- Add CAD, STL, drawings, fastener inventory, materials, and print settings.

## Automated checks at export time

These checks ran against the private parent workspace during the export audit.
They are useful source evidence, but are not clean-checkout evidence for this
documentation repository and are not live-hardware proof.

| Check | Result | Scope note |
| --- | ---: | --- |
| Canonical Arm source-index validator | 32 paths passed | Parent source map |
| Raspberry Pi gateway tests | 261 passed | Focused `robot_gateway` suite |
| Arm MCP tests | 49 passed | Focused MCP compatibility suite |
| Arm HAT firmware host tests | 15 passed | Protocol/runtime host harness |
| Arm frontend tests | 158 passed across 10 files | Arm components inside the parent app |
| TypeScript project build | Passed | Whole parent application |
| Production frontend build | Passed | Whole parent application |
| Broad combined Python run | 492 passed, 2 failed | Not a green aggregate result |

The two broad-run failures were one order-sensitive legacy physical-arm API
expectation and one persistent embedded-memory context-size expectation. They
do not invalidate this documentation package, but they prevent a green parent
aggregate claim. A source release must run and pass its own allowlisted tests
from a fresh clean checkout.
