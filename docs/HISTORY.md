# Public engineering history

This is a distilled technical history. Failed approaches are included when
they explain the current design, but installation-specific and personal
details have been removed.

## 2026-07-31 to 2026-08-04 — simulator-first foundation

The project established a Raspberry Pi camera/evidence gateway, an ESP32 HAT
protocol, replay and simulation infrastructure, commissioning UI, and a local
dashboard. Early builds and tests proved software paths only; they did not
prove servo motion.

## 2026-08-04 — physical stack connected

The Pi service, camera, direct controller link, and reboot recovery were
observed. The controller was still in an early firmware line and the physical
commissioning path was incomplete.

## 2026-08-05 — bus and telemetry root causes

The primary early motion failure was traced to incorrect ST3215 load decoding:
the load-direction flag is bit 10, not bit 15. The fix removed false collision
interpretation. A separate bus failure was traced to a factory ID collision:
the SC09 camera servo initially shared ID 1 with the Base. It was isolated,
reassigned to ID 4, and the full bus then enumerated IDs 1 through 4 cleanly.

## 2026-08-07 — wide motion, then a reliability limit

An earlier Mode-3 relative-step approach produced real signed Base travel and
coordinated four-joint movement. The software gained STOP/watchdog handling,
floor geometry, hold leases, and richer telemetry. This was meaningful physical
progress, but Mode-3 depended on sampled continuity and step bookkeeping.

## 2026-08-08 — typed AI control and false-zero diagnosis

Codex moved the arm and captured real camera pixels through typed tools. A
serious Base false-zero event was then reproduced: software could report a
plausible coordinate after physical multi-turn truth had been lost. Motion was
paused and the failure was treated as a coordinate-continuity problem rather
than a UI or network glitch.

## 2026-08-09 — recovery and homing work

Atomic homing, Pi/HAT reconnection behavior, diagnostics, and hold-renewal
fixes were developed and deployed in stages. These reduced known failure modes,
but did not make the relative-step method an acceptable final architecture.

## 2026-08-10 — native absolute Base control

Firmware 2.4.0 replaced Mode-3 step bookkeeping with the ST3215's native
Mode-0 signed extended-position coordinate. Configuration became a guarded
read/modify/write/readback sequence. A physical Base readback proved the
extended-position phase bit, resolution, zero limits, operating mode, lock,
and torque-off state. The matching Pi source was deployed, the complete stack
was power-cycled, and the configuration persisted.

The operator then accepted normal zero, positive, negative, and return movement.
This was physical acceptance, not a complete numeric seam or endpoint table.

## 2026-08-10 — Arm Alliance

The arm gained a shared MCP server and bounded Codex integration. Live tools
were separated from project editing authority, and local AI context was
allowlisted rather than copying credentials, transcripts, or an entire agent
home.

## 2026-08-11 — explicit plan/apply and full-resolution vision

Motionless preview and one-use apply contracts were added. Preview resolves
joint targets or planar IK against fresh telemetry, limits, start-pose drift,
and independently timed swept floor clearance. A real preview was generated
without moving the arm. The apply path remains a recorded physical gate.

Camera capture moved to the OV5647's full still resolution. Focus handling now
states the actual optical limit: the installed sensor module is fixed focus.
For it, useful focus comes from choosing standoff and judging visible subject
detail, not pretending software can actuate a lens.

## 2026-08-22 to 2026-08-23 — Camera Module 3 and Live Follow

The reference camera changed to Camera Module 3 Wide / IMX708. Survey/detail
profiles, sensor/array validation, powered autofocus controls, and same-capture
AF evidence were added while retaining OV5647 only as a legacy compatibility
profile.

Shoulder/Elbow Live Follow gained a separate strict heartbeat, bounded grouped
dispatch, exact goal readback, dead-man/lease behavior, and start-relative
travel envelopes. After the installed servos were shown to store acceleration
`50` when a higher value was requested, every current layer adopted `1..50` as
the explicit arm-specific range. Five- and twelve-second unloaded runs then
completed on matching Arm HAT 2.7.2, Pi, and dashboard code. The raw settings
were not treated as calibrated speed.

## 2026-08-24 — ARM-only public source export

The standalone dashboard/backend, Pi gateway, Arm HAT firmware, core Isaac
simulator, MCP/plugin, operations helpers, and focused tests were extracted from
the broader development workspace. Machine-specific configuration, credentials,
runtime data, captures, reports, backups, generated artifacts, and unrelated UI
were excluded. Unrelated product and evaluation additions were then removed to
keep the public package focused on building, simulating, commissioning, and
running the arm. Clean-checkout tests remain source evidence, not a new hardware
verification.

## Current lesson

The project became more reliable whenever it replaced inferred state with
measured state and made evidence explicit: native signed coordinates instead
of reconstructed turns, plan/apply instead of implicit motion, retained source
crops instead of needless recaptures, and visible pixels instead of assuming a
named pose must show the desk.
