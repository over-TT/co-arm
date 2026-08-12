# Electronics and wiring

This page captures the known electronic architecture and defines what must be
verified before a public wiring guide can be called complete. No connector
pinout is published yet because the as-built harness has not been traced and
photographed end to end.

## Known functional blocks

| Block | Recorded device | Role |
| --- | --- | --- |
| Gateway | Raspberry Pi 4 | Network/API host and camera capture |
| Camera | Raspberry Pi Camera Module 1 Rev 1.3 (`OV5647`) | Fixed-focus image source over the Pi camera interface |
| Motion controller | Waveshare Bus Servo Driver HAT (A), SKU 27577 | Integrated ESP32, host-UART endpoint, and serial-servo bus control |
| Main joints | 3 x ST3215/STS-family serial-bus servos | Base, Shoulder, and Elbow |
| Camera joint | SC09/SCS-family serial-bus servo | Camera pitch |
| Candidate servo supply | 12 V, 7.5 A, 90 W nominal | Candidate source for the high-power servo rail only |

Exact model labels, board revisions, connector variants, and voltage limits must
be transcribed from the physical components into the BOM.

## Logical signal topology

```text
Raspberry Pi 4
  +-- camera interface ---------------- OV5647 camera module
  |
  `-- UART, 115200 baud ---------------- ESP32 on Waveshare HAT (A)
                                              |
                                              `-- half-duplex serial servo bus,
                                                  1 Mbps
                                                    +-- ID 1 Base
                                                    +-- ID 2 Shoulder
                                                    +-- ID 3 Elbow
                                                    `-- ID 4 Camera
```

This is a logical topology. The connector order, wire colors, pin numbers,
physical daisy-chain order, branch locations, and ground paths are **TODO (owner
verification)**. Do not turn the diagram above into a cable without consulting
the board/servo documentation and checking the physical harness with power off.

## Servo addressing

The current logical assignment is:

| ID | Joint | Working family/model |
| ---: | --- | --- |
| 1 | Base | ST3215/STS-family |
| 2 | Shoulder | ST3215/STS-family |
| 3 | Elbow | ST3215/STS-family |
| 4 | Camera | SC09/SCS-family |

Serial-bus servos may ship with the same default ID. Two devices answering one
address can make the bus unreadable. Verify or change an ID with only one
uncommissioned servo connected, then label both the servo and its cable before
adding it to the shared bus.

The camera servo belongs to a related but not necessarily identical protocol
family. Its exact label, voltage, byte ordering, and compatible control profile
must be verified rather than inferred from the three main servos.

## Power domains

Treat these as separate domains until the as-built circuit proves otherwise:

1. Raspberry Pi logic power.
2. ESP32/HAT logic power.
3. 12 V candidate rail for the ST3215/STS-family actuators.
4. Camera-servo rail, potentially regulated to a different voltage.
5. Signal/common reference between the controller and servo bus.

Never apply the candidate 12 V rail directly to the Raspberry Pi. Do not apply
12 V to the SC09/SCS-family servo until its exact label and electrical rating
are confirmed. The presence, model, setting, and load capability of any
step-down converter remain undocumented.

### Candidate supply is not yet an approved supply

`12 V x 7.5 A = 90 W` is only the nameplate nominal value. Approval requires:

- exact supply model and trustworthy output rating;
- connector type and verified centre/pin polarity;
- measured no-load and loaded voltage;
- simultaneous-axis peak current or a conservative measured bound;
- suitable wire gauge, connector current rating, branch distribution, and
  temperature rise;
- fuse/protection strategy and fault-current path;
- headroom for transient/stall conditions without voltage collapse;
- confirmation that logic power remains stable during joint starts/stops.

Do not size the system by adding advertised servo torque values or by assuming
all actuators draw the same current.

## STOP and emergency isolation

The software stack has a STOP state and torque-control safeguards. Those are
important controls, but they are not proof of a physical emergency stop. A
public build guide must distinguish:

- **software STOP:** a controller/gateway state that commands or prevents
  motion while electronics remain powered;
- **hardware E-stop/disconnect:** a reachable, intentional method that removes
  actuator power or otherwise puts the machine in a defined safe state without
  depending on software or communications.

- **TODO (owner verification):** state whether a physical E-stop or latched
  servo-power disconnect exists and document exactly what conductors it opens.
- **TODO (owner verification):** document what happens to each link after
  actuator power is removed; gravity may make an elevated arm fall or sag.

## Required wiring documentation

Add final files under:

- [`hardware/electronics/schematics/`](../hardware/electronics/schematics/) for
  electrical intent, power domains, protection, and connector pin numbers;
- [`hardware/electronics/wiring/`](../hardware/electronics/wiring/) for the
  physical harness, cable IDs, lengths, routing, and photographs/diagrams.

Every connector in a released drawing should have:

- unique reference designator (`J1`, `J2`, ...);
- mating connector family and part number;
- pin number, signal name, nominal voltage, maximum expected current, and wire
  color/gauge;
- polarity/keying view and an explicit statement of the viewing side;
- destination reference and cable label;
- revision and date.

## Power-off wiring verification checklist

1. Disconnect all external power and mechanically support raised links.
2. Photograph the complete system before moving wires.
3. Trace one conductor at a time; do not rely only on color.
4. Confirm continuity and absence of shorts with an appropriate meter.
5. Record connector views as `mating face` or `wire side`.
6. Verify supply polarity at the disconnected load connector.
7. Verify the regulator output independently before attaching a servo.
8. Reconnect one power domain at a time with current limiting when appropriate.
9. Check idle voltage/current and temperature before enabling motion.
10. Update the schematic, wiring drawing, BOM, and photo captions together.

## Unresolved electrical record

- **TODO (owner verification):** Pi 4 board revision, RAM, storage, power
  supply, and cooling.
- **TODO (owner verification):** exact HAT revision and all switch/jumper
  positions.
- **TODO (owner verification):** complete connector pinout and physical bus
  order.
- **TODO (owner verification):** exact labels and electrical ratings of all
  four servos.
- **TODO (owner verification):** supply brand/model, mains input rating,
  output tolerance, DC connector, and polarity.
- **TODO (owner verification):** camera-servo regulator/step-down model,
  adjustment, measured output, current capability, and thermal margin.
- **TODO (owner verification):** fuse type/rating/location, over-current and
  reverse-polarity protection, grounding, wire gauge, strain relief, and cable
  shielding/routing.
- **TODO (owner verification):** hardware E-stop/disconnect and safe-state
  behavior.
- **TODO (owner verification):** measured idle, single-axis, coordinated, and
  worst-observed current/voltage data.
