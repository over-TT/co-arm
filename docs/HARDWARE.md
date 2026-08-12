# Hardware overview

This page records the public-safe hardware facts currently known for co-arm. It
is an as-built record in progress, not a purchasing guarantee or a validated
replication guide. Items marked **TODO (owner verification)** must be checked
against the physical arm, a label photograph, a drawing, or a measurement
before they are treated as final.

## Current configuration

| Function | Recorded hardware | Current confidence |
| --- | --- | --- |
| Host computer | Raspberry Pi 4 | Recorded; RAM capacity and board revision still need a label/photo |
| Bus controller | Waveshare Bus Servo Driver HAT (A), SKU 27577, with integrated ESP32 | Recorded; photograph the front and back labels |
| Base actuator | ST3215/STS-family serial-bus servo, logical ID 1 | Working identification; exact label and suffix need owner verification |
| Shoulder actuator | ST3215/STS-family serial-bus servo, logical ID 2 | Working identification; exact label and suffix need owner verification |
| Elbow actuator | ST3215/STS-family serial-bus servo, logical ID 3 | Working identification; exact label and suffix need owner verification |
| Camera actuator | SC09/SCS-family serial-bus servo, logical ID 4 | Working identification; exact label and voltage rating need owner verification |
| Camera | Raspberry Pi Camera Module 1 Rev 1.3, OV5647 sensor | Recorded from the live camera stack; add label photographs |
| Candidate servo supply | 12 V, 7.5 A (90 W nominal) | Candidate only; exact model, ratings, connector, polarity, and measured margins are not approved yet |

The hardware bill of materials lives in
[`hardware/bom/bom.csv`](../hardware/bom/bom.csv). Update that file whenever an
exact label, quantity, supplier part number, or revision is confirmed.

## Mechanical architecture

co-arm is a four-actuator desk arm:

1. **Base** — yaw about the vertical axis. The external reduction is 4:1,
   meaning four motor turns produce one output turn.
2. **Shoulder** — pitch of the upper arm.
3. **Elbow** — pitch of the distal link.
4. **Camera** — aims the camera independently and is not part of the planar
   shoulder/elbow inverse-kinematics chain.

Recorded model dimensions, in millimetres:

| Dimension | Value | Meaning |
| --- | ---: | --- |
| Base pivot height | 60 mm | Work-surface reference to the shoulder/base model pivot |
| Upper arm | 180 mm | Shoulder pivot to elbow pivot |
| Forearm | 180 mm | Elbow pivot to camera/tool-offset origin |
| Camera/tool offset | 40 mm | Distal offset beyond the forearm |
| Elbow to camera/tool tip | 220 mm | Forearm plus camera/tool offset |
| Maximum planar link length | 400 mm | 180 + 220, before joint limits and collisions |

These values are the software model dimensions and are suitable reference
dimensions for CAD alignment. They are not manufacturing tolerances. See
[`MECHANICAL.md`](MECHANICAL.md) for frames, zero conventions, and measurements.

## Electrical architecture

The intended logical chain is:

```text
Raspberry Pi 4
  |-- CSI camera interface --> OV5647 camera
  `-- host UART --> ESP32 on Waveshare Bus Servo Driver HAT (A)
                         `-- serial servo bus --> IDs 1, 2, 3, and 4
```

The Pi-to-controller host UART runs at 115200 baud. The serial-servo bus runs at
1 Mbps. Those communication values describe the current software/firmware
contract; they do not define connector pin order. Never infer a physical pinout
from this document.

The 12 V, 7.5 A supply is only a candidate for the servo power rail. It must not
be treated as approved merely because its nominal rating is 90 W. Peak/stall
current, wire gauge, connector ratings, voltage drop, regulator capacity,
thermal rise, and simultaneous-axis motion still need measurement. In
particular, do not assume the SC09/SCS-family camera servo can accept the same
12 V rail as the ST3215/STS-family actuators.

See [`ELECTRONICS.md`](ELECTRONICS.md) before adding a wiring diagram or
powering a replicated build.

## Camera facts

- Sensor: OV5647, fixed-focus, approximately 5 MP.
- Full still-capture resolution used by the current stack: 2592 x 1944.
- The module is physically inverted on the arm; software applies a 180-degree
  transform so newly captured images are upright.
- The installed module has no autofocus actuator. Focus is adjusted by camera
  standoff/framing, not by claiming a software autofocus result.

## Control and calibration facts that affect hardware work

- Logical servo IDs are Base `1`, Shoulder `2`, Elbow `3`, Camera `4`.
- A factory-fresh bus servo may arrive on an ID already in use. Assign or verify
  IDs with only the intended device connected to the bus; two devices answering
  the same address can corrupt communication.
- Base position continuity matters because the motor makes multiple turns for
  one output turn. After a true position-continuity loss, align the physical
  zero mark and deliberately establish zero again.
- The software floor keep-out plane defaults to 40 mm. It is a software guard,
  not a physical stop, not a measurement of table thickness, and not a promise
  that every mounted part clears the desk.
- Software build/test success is not evidence of correct wiring, adequate power,
  structural strength, or physical clearance.

## Unresolved owner-verification items

The following are intentionally not guessed:

- **TODO (owner verification):** photograph the exact label on every servo,
  including model suffix, voltage, serial number if appropriate, and any gear
  or horn markings. Redact serial numbers before public upload if desired.
- **TODO (owner verification):** record the Raspberry Pi board revision and RAM
  capacity.
- **TODO (owner verification):** photograph and transcribe the power supply
  input/output label, connector type, connector polarity, and cable rating.
- **TODO (owner verification):** document the camera-servo regulator or
  step-down converter, including its measured output under load.
- **TODO (owner verification):** document the power-distribution topology,
  common-ground arrangement, fuse locations/ratings, wire gauges, connector
  current ratings, and strain relief.
- **TODO (owner verification):** state whether a physical E-stop, latched power
  disconnect, or other hardware torque-removal device exists. A software STOP
  is not equivalent to a hardware E-stop.
- **TODO (owner verification):** identify all bearings, shafts, gears, horns,
  brackets, fasteners, heat-set inserts, washers, spacers, and cable-management
  parts.
- **TODO (owner verification):** record printed-part material, printer/nozzle,
  layer height, wall count, infill, support, orientation, and post-processing.
- **TODO (owner verification):** measure the mounting footprint, base anchoring,
  moving clearances, joint hard stops, cable slack, and payload.

Do not remove a TODO merely because a CAD model contains a value. Close it with
as-built evidence and update the BOM, drawing, and relevant document together.
