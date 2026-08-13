# Commissioning

This runbook brings a newly assembled co-arm from unpowered hardware to a
documented first motion. It does not authorize flashing or movement by itself;
the operator remains responsible for power isolation, mechanical support, and
the physical work envelope.

Start with [`SETUP_WITH_CODEX.md`](SETUP_WITH_CODEX.md) when a human and Codex
are preparing the checkout or Raspberry Pi together. Return here once the
hardware facts and software release boundary are understood.

Software tests and a successful firmware build do not prove wiring, servo
identity, supply capacity, torque removal, direction, clearance, or motion on
the assembled arm.

## Critical operator inputs

Resolve and record these before the corresponding stage. Do not let software or
an AI guess them.

| Input | Why it is critical |
| --- | --- |
| Exact supply voltage/current rating, fuse, conductor size, and polarity | A logic connection can work while the servo rail is unsafe or undersized |
| Reachable, tested physical servo-power cut | Software STOP depends on powered, functioning electronics |
| Printed model of every servo | STS- and SCS-family devices share a bus but differ in register layout and encoder scale |
| Which physical servo drives each joint | A bus reply does not prove mechanical mapping |
| Base gearbox ratio and a durable physical zero mark | Multi-turn output position cannot be reconstructed after continuity loss without them |
| Safe manual endpoint and inward margin for every joint | Prevents using a hard stop as a calibration instrument |
| Camera/bracket orientation and hardware revision | Determines upright transform, ray convention, and reusable viewpoints |
| Whether the arm is supported during torque release/restart | Shoulder and Elbow can fall when authority expires |

If any answer is not known, stop at the non-motion stage that can still be
completed safely.

## 0. Prepare the work area

1. Remove the servo-rail supply.
2. Support all gravity-loaded links.
3. Clear the full possible sweep and route cables with slack.
4. Install and test a physical servo-power cut with the rail unloaded.
5. Mark the Base mechanical zero visibly and durably.
6. Photograph wiring and labels for the build record, excluding private
   information.
7. Confirm the Pi and ESP32 logic-power arrangement will not back-power another
   host connection.

Do not count a dashboard button as the physical power cut.

## 1. Review the software export

Read [`../software/README.md`](../software/README.md). Confirm that:

- firmware, gateway, MCP, and dashboard source status is understood;
- local credentials and endpoint configuration are outside Git;
- toolchain versions are recorded locally;
- source tests can be run before deployment;
- the extracted dashboard/backend's standalone packaging limitations are
  accepted.

Run the relevant source tests and firmware compile described by each software
folder. Record the command, toolchain version, commit, and result. This is the
**software proof tier only**.

## 2. Wire with servo power off

The intended control path is:

```text
dashboard / MCP client
-> laptop proxy or direct local MCP configuration
-> authenticated Raspberry Pi gateway
-> ESP32 Arm HAT host UART
-> 1 Mbps TTL servo bus
-> smart servos
```

The Pi camera connects to the Pi and is not in the safety loop. Confirm common
ground, host TX/RX crossover, servo-bus polarity, supply polarity, and connector
orientation against the exact board revision.

Only one host may drive the ESP32 host UART while flashing. Disconnect or
isolate the Pi serial side before connecting a USB flashing host. Keep servo
power off for the flash.

## 3. Build and flash firmware 2.4

1. After the source release is added, build the exported ESP32 sketch against
   its Arm HAT controller library. Until then, use the reviewed private source
   package and do not treat this documentation repository as flashable.
2. Save the build log and artifact hash locally.
3. Confirm the exact target board and flash port.
4. Isolate competing UART hosts.
5. Flash only after explicit operator approval.
6. Disconnect the flashing host or otherwise restore the intended Pi UART
   topology before the controller handshake.

On boot, firmware should begin disarmed and attempt a servo-bus torque-off
broadcast. A boot message saying an attempt was sent is not the same as
addressed torque-off confirmation from every servo.

## 4. Establish a non-motion controller handshake

With the servo rail still off, start the Pi gateway and confirm a compatible,
typed controller handshake. It must report:

- expected protocol and firmware generation;
- disarmed/blocked startup rather than active motion;
- boot torque-off attempt status;
- the watchdog and lease range;
- required capabilities, including native absolute multi-turn support for the
  Base path;
- the expected host-UART and servo-bus configuration.

Do not commit controller identifiers, network addresses, or credential paths to
the public repository.

## 5. Assign and verify servo-bus identities one at a time

Factory servos may share an identifier. Never connect several uncommissioned
same-ID servos and try to repair the collision in software.

For each servo:

1. Remove servo power.
2. Connect exactly one servo to the bus and mechanically unload/support it.
3. Read and photograph the printed model label.
4. Power the rail and perform a bounded scan.
5. Confirm exactly one response.
6. Assign the reference mapping: Base `1`, Shoulder `2`, Elbow `3`, Camera `4`.
7. Power-cycle and scan again before applying a physical label.
8. Remove power before connecting the next servo.

The reference arm uses STS-family behavior for Base/Shoulder/Elbow and
SCS-family behavior for Camera. Do not infer family from a successful ping.

## 6. Assemble the bus and verify readback

With all four labelled servos connected and the arm supported:

1. Power on without requesting torque.
2. Perform a non-motion scan.
3. Refresh state after the scan; a first cached state can lag the bus result.
4. Confirm all expected joints are present and mapped.
5. Confirm torque is `off` or otherwise safely known.
6. Inspect voltage, temperature, moving flag, fault flags, packet age, and Base
   truth/capability.

If the controller disappears after torque has ever been enabled, treat torque
as `unknown` and remove servo power before touching the arm.

## 7. Calibrate each joint with torque off

Keep the arm supported and calibrate one joint at a time.

For each joint:

1. Confirm the physical joint mapping by moving only that joint manually and
   observing which encoder changes.
2. Place it at the chosen neutral/zero and use server-owned **Set zero here**;
   do not send an old browser-cached encoder value.
3. Determine the positive physical direction with a manual torque-off move.
4. Capture comfortable negative and positive endpoints manually.
5. Apply an inward safety margin; never drive into a hard stop.
6. Record encoder scale, direction, gear ratio, and calibrated degree limits.
7. Re-read the saved configuration and its revision.

The Camera servo may use a different encoder scale and byte/register family
from the arm joints. It still needs its own zero and safe travel limits.

## 8. Configure and home the Base absolute frame

The reference firmware 2.4 Base path uses ST3215 native Mode-0 absolute
multi-turn operation. The verified reference profile expects:

- Phase register BIT4 enabled (the reference value was `28`);
- resolution register set to `1`;
- internal angle-limit registers cleared for the multi-turn path;
- operating mode `0` (position mode);
- native capability `multi_turn_absolute_v1` visible to the Pi.

Use the typed commissioning workflow that performs torque-off staging,
read-modify-write/readback, and configuration verification. Do not hand-issue
raw register writes from this guide.

Then:

1. Remove/confirm torque off and align the output to the physical Base zero
   mark.
2. Use **Set zero here** to bind the native signed motor frame to output zero.
3. Confirm the Pi reports valid native-frame truth from the same controller
   boot.
4. If continuity becomes unknown at any later time, repeat this physical
   re-home. Never infer an output revolution from a single-turn reading.

## 9. Verify geometry and floor policy without moving

Confirm the configured model matches the assembled revision:

- Base pivot height `60 mm`;
- upper arm `180 mm`;
- Elbow-to-tip distance `220 mm`;
- Shoulder `0 deg` straight up;
- straight arm at Elbow `90 deg`;
- Camera `0 deg` along the forearm, positive down;
- floor guard enabled at the intended floor reference (reference default
  `40 mm`).

Render a measured scene and compare it to the physical arm. A 90-degree frame
error here makes the floor calculation unsafe even if every joint angle is
numerically fresh.

## 10. First controlled motion

Before each first-motion test, confirm the physical power cut, mechanical
support, and clear sweep again.

1. Prepare a very small single-joint plan at low speed.
2. Review measured and planned angles, warnings, and swept clearance.
3. Apply the one-use plan.
4. Read measured state after settling; do not judge success from the apply
   response.
5. STOP immediately for wrong direction, unexpected sound/current, cable pull,
   structural flex, telemetry loss, or motion outside the reviewed sweep.
6. Test return to the starting point before increasing range.

Suggested progression:

1. Camera small positive/negative tilt while the distal assembly is supported.
2. Shoulder small positive/negative motion with the Elbow supported.
3. Elbow small positive/negative motion with the distal link supported.
4. Base zero -> small positive -> zero -> small negative -> zero.
5. Larger single-joint points inside conservative limits.
6. Slow coordinated poses with floor guard and plan/apply.
7. A controlled physical STOP test.

Record requested, resolved, and measured angles separately.

## 11. Commission the camera

1. Capture a fresh frame without motion.
2. Confirm the delivered image is upright.
3. Confirm the reported sensor capability: the reference OV5647 is fixed-focus.
4. Verify Camera positive physically moves the view downward.
5. Build a Base-`0 deg`, elevated/retracted desk survey through plan/apply.
6. Save a named viewpoint only after measured arrival and pixel coverage are
   both verified.
7. Reverify that view after any bracket, camera, geometry, zero, or limit change.

Do not publish an unreviewed desk capture. See [`VISION.md`](VISION.md).

## 12. Restart and fail-safe checks

Place the arm in a verified clear rest pose or support it mechanically before a
service/controller restart. A restart can outlast the heartbeat and hold lease,
causing a held arm to sag.

Verify separately:

- gateway restart does not command motion;
- controller boot starts disarmed and attempts torque off;
- loss of heartbeat revokes torque authority within the watchdog contract;
- lease expiry removes hold authority;
- deliberate STOP remains latched until inspected reset;
- reset does not resume the old goal;
- Base truth is preserved only when continuity is explicitly proven, otherwise
  re-home is required.

These are live hardware tests. Run them with support, clearance, and a physical
power cut.

## Commissioning record

For each stage, record:

- hardware/CAD revision;
- firmware and software commit/artifact hashes from the actual source package;
- supply and protection configuration;
- confirmed servo labels and joint mapping;
- calibration revision and limits;
- requested/resolved/measured motion;
- voltage, current/load where supported, temperature, faults, and packet loss;
- STOP/lease/watchdog test method and result;
- camera evidence filename after privacy review;
- operator, date, result, and remaining proof debt.

Completion should read: **commissioned for the tested envelope; motion remains
disarmed until explicitly requested**. It must not imply certification or safe
operation outside the recorded configuration.
