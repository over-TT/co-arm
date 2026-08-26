# Troubleshooting

Start by identifying the failed layer. Do not compensate for an unknown state
with a larger move, disabled guard, raw register write, or repeated command.

## Immediate stop conditions

Use STOP or the physical servo-power cut, as appropriate, if the arm moves in
the wrong direction, leaves the reviewed sweep, catches a cable/object, makes
an abnormal sound, reports an electrical fault, loses trustworthy telemetry,
or does not respond to STOP. If torque state becomes `unknown`, remove servo
power before touching the arm.

Support gravity-loaded links before release, restart, reconnect, or manual
repositioning.

## Read-only diagnosis order

1. Gateway process health
2. Controller connection and boot continuity
3. Servo-bus health
4. Non-motion bounded bus scan
5. Refreshed live arm state
6. Required joint freshness, faults, torque, and Base truth
7. Floor guard and calibrated limits
8. Only then, prepare a motionless plan

A gateway health result does not prove the controller or servo bus. A scan can
find devices while an older cached state still says offline; refresh state
before deciding.

## Connection and bus symptoms

| Symptom | Likely layer | Safe checks and response |
| --- | --- | --- |
| Dashboard cannot reach gateway | Local backend, tunnel, gateway service, or credentials | Check each process independently; do not restart a torque-holding arm without support |
| Gateway healthy, controller offline | Host UART, boot order, firmware mismatch, or controller power | Keep servo power/motion off; verify one host owns UART, then use the bounded reconnect path |
| Controller online, servo bus offline | Servo power, bus wiring/polarity, collision, noise, or wrong baud | Remove power, inspect wiring, reconnect one labelled servo, run bounded scan |
| Scan finds fewer/more devices | Disconnected servo, duplicate ID, unexpected device | Power off and isolate one servo; never rewrite IDs on a populated ambiguous bus |
| Torque says `unknown` | Readback missing, controller disappeared, or off confirmation failed | Remove servo power; do not touch until electrical state is known |
| State looks stale immediately after scan/reconnect | Cached telemetry has not refreshed | Wait for a new generation and read state again; do not move from the old snapshot |

## STOP, watchdog, and lease symptoms

| Symptom / error | Meaning | Response |
| --- | --- | --- |
| `STOPPED` | STOP is latched or controller reports stopped | Inspect the physical cause. Clear only on explicit operator decision after torque-off checks |
| `NO_TORQUE_LEASE` | Motion arrived without active authority, often after a delay or restart | Do not bypass it. Let the Pi retake a fresh hold only after state/clearance are rechecked |
| Hold drops during motion | Heartbeat/serial starvation, lease expiry, process interruption, or electrical fault | Support/STOP, inspect connection and packet timing, then replan from measured position |
| Arm moves again after authority returns | A stale goal was retained somewhere | STOP and power-cut if needed. The current Pi clears outstanding goals on deliberate STOP; verify deployed version before retesting |
| STOP will not clear | No fresh heartbeat, torque-off not confirmed, or an operator STOP still intentionally held | Resolve heartbeat/readback; never loop reset requests blindly |

Current reference limits are a 750 ms firmware watchdog, 100..2000 ms lease
range, a 2000 ms Pi hold renewed around 800 ms, and an approximately 200 ms Pi
heartbeat. Treat a mismatch between source and live handshake as deployment
drift.

## Plan/apply symptoms

| Symptom / message | Cause | Response |
| --- | --- | --- |
| Plan unavailable, expired, or already used | Plan exceeded its current 30 s lifetime or was consumed | Read state and prepare a new plan |
| Digest mismatch | Applied data is not the exact reviewed proposal | Discard it and plan again; never edit the digest or targets |
| Controller boot changed | Preview belongs to an older controller continuity frame | Re-read state; re-home Base if truth is not proven; plan again |
| Configuration changed | Calibration, limits, geometry, or floor policy differs from preview | Review the new configuration and plan again |
| Joint moved after preview | Start pose drift exceeded current 2 deg tolerance | Wait for settle or investigate unexpected motion, then replan |
| Earlier move is still settling | An outstanding bounded goal has not closed/expired | Read measured state; do not stack another plan |
| Fresh telemetry required | Needed joint packet, heartbeat, or Base odometer evidence is stale | Wait for a new measured generation; fix connection if it stays stale |
| Sweep crosses floor | Any independently timed Shoulder/Elbow combination can go below the allowed plane | Choose a different pose or safe waypoint. Do not disable the guard as an agent workaround |
| Direct move was clamped | Calibrated limit or floor boundary changed the command | Read the returned reason and measured state; revise the request rather than repeatedly pushing the boundary |

Plan/apply success proves command acceptance. If measured state differs after
settling, diagnose arrival rather than declaring the destination reached.

## Base multi-turn symptoms

### Base angle is numerically plausible but physically one or more turns wrong

Treat this as lost revolution continuity. STOP, support the arm, remove/confirm
torque off, align the physical Base zero mark, and use **Set zero here**. Never
choose an output revolution from a single-turn reading alone.

### `ODOMETER_UNAVAILABLE`, invalid Base truth, or missing native capability

Do not move Base. Verify Arm HAT 2.7.2/native capability, stable controller boot,
fresh native-frame evidence, Mode 0, and the verified multi-turn
configuration. Re-home if continuity cannot be proven.

### `MODE_NOT_POSITION`

Keep torque off. Use the typed Base configuration/restore workflow, which
verifies mode and related registers. Do not improvise raw writes while the arm
is assembled or energized.

### Base reaches the opposite or stale destination

STOP. Confirm that the deployed stack uses one native absolute destination,
not an older relative-step implementation. Check gateway, firmware, and client
versions separately; source changes do not update running processes.

## Floor and geometry symptoms

### Scene is rotated or height is obviously wrong

Verify the servo-to-model offsets: Shoulder `+90 deg`, Elbow `-90 deg`. Shoulder
servo zero must draw straight up, and Shoulder `0` plus Elbow `90` must draw a
straight arm. Confirm 60/180/220 mm geometry and the correct calibration.

### Guard clamps a physically high pose

Check fresh Shoulder/Elbow measurements, direction, zero, offsets, link lengths,
and floor reference. Do not lower the floor value until the model matches the
physical arm.

### Destination is above the floor but plan is refused

The intermediate independently timed sweep may cross the plane even when both
endpoints are safe. Inspect lowest swept clearance and use a safe waypoint.

### Base or Camera was clamped and message mentions travel

Those joints do not affect side-view floor height. Their clamp comes from the
joint's calibrated range; fix the target or calibration evidence, not the floor
guard.

## Camera and vision symptoms

| Symptom | Diagnosis | Response |
| --- | --- | --- |
| Image upside down | Pi transform missing or a second transform was applied | Verify exactly one 180-degree transform in the camera provider |
| Capture succeeds but UI image fails | Browser/object-URL/proxy loading stage, not shutter | Preserve the capture error boundary and debug the display path separately |
| Subject absent after moving | Target pose did not guarantee framing | Verify measured arrival, inspect scene ray, then make a bounded coordinated correction |
| Survey does not cover desk | Stored/guessed viewpoint or current geometry differs | Rebuild the Base-0 elevated survey from live state and pixels |
| Closer frame is softer | AF has not settled, macro/normal range is wrong, or standoff is unsuitable | Check same-frame AF/lens metadata and visible subject detail; change focus range or standoff |
| `FocusFoM` rises but subject is worse | Background texture dominates whole-frame score | Trust subject ROI readability and edges |
| AF is `configured` but not `focused` | Control request has no same-frame focus proof | Require valid capture AF metadata and improved subject detail |
| Detail crop source unavailable | Bounded retained-frame history expired or was not saved | Take a new survey; do not reuse an old runtime identifier |
| Timestamp looks old | Wall clock may be wrong or frame may be stale | Capture two comparison frames and inspect changing content/monotonic age |

## Source, deployment, and UI symptoms

- **Tests pass but hardware behaves differently:** tests are software proof.
  Verify which firmware, gateway, MCP server, and frontend bundle are actually
  running.
- **A new MCP tool is missing:** start a new client task/session after installing
  or updating local MCP configuration. Existing tasks do not hot-load tools.
- **Gateway edit has no effect:** deploy the reviewed package and restart the
  Pi service only after the arm is supported or in a verified clear rest pose.
- **MCP edit has no effect:** restart the MCP process/new task; the Pi service is
  a separate process.
- **Dashboard will not build:** work from `software/dashboard/`, use the pinned
  package manager/lockfile, and distinguish dependency-install failure from a
  TypeScript, test, or Vite failure.
- **Backend cannot find the frontend:** build the dashboard first and confirm
  the standalone runtime's static-directory path. Do not point it at a broader
  private application as a shortcut.
- **Agent tools are unavailable:** confirm the co-arm Python environment is the
  interpreter used by MCP, start a new task after plugin/config changes, and
  verify the selected SIM or REAL URL/token-file pair. This is separate from
  dashboard or gateway health.

## What to capture in a bug report

Include only privacy-safe data:

- commit/artifact versions for each running layer;
- operation and exact public error code/message;
- whether motion occurred;
- controller/bus/STOP/floor/torque trust states, with live identifiers removed;
- requested, resolved, and measured angles only when sharing them is safe;
- calibration and hardware revision;
- redacted photo/video if it materially proves the symptom;
- source-test, build, deployment, live telemetry, and pixel evidence as separate
  sections.

Do not include gateway credentials, private addresses/paths, account/session
state, controller/session/frame identifiers, or unrelated desk content.
