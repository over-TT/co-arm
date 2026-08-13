# co-arm

This is my four-joint desktop camera arm project. Basically, I wanted Codex to
be able to actually look around my desk, understand what it can see, and move
the camera when it needs a better angle. I am documenting the whole build here
as I go, including the hardware, electronics, software, CAD files, tests, and
the things that are still not fully proved yet.

The Raspberry Pi handles the camera, API, geometry, and motion rules. An ESP32
on the servo HAT handles the real-time servo bus, watchdogs, STOP state, torque
leases, and torque-off behavior.

> **Important:** This is a working prototype, not a certified robot, safety
> controller, finished product, or plug-and-play kit. I keep it supervised,
> support the links whenever torque might turn off, keep the movement area
> clear, and keep a physical servo-power cut within reach. The software STOP is
> useful, but it is not the same as physically cutting power.

## What is on the arm right now

| Part | Current reference build |
| --- | --- |
| Driven joints | Base, Shoulder, Elbow, and Camera |
| Main servos | 3 x ST3215/STS-family serial-bus servos |
| Camera servo | SC09/SCS-family serial-bus servo |
| Base transmission | 4:1 external gearing |
| Gateway | Raspberry Pi 4 |
| Servo controller | ESP32 on a Waveshare Bus Servo Driver HAT (A) |
| Camera | Raspberry Pi Camera Module 1 Rev 1.3 / OV5647, fixed focus |
| Measured geometry | 60 mm pivot height, 180 mm upper arm, 220 mm elbow-to-tip |
| Current controller line | `arm-hat-2.4.0`, native Mode-0 absolute Base control |

The Camera joint is the fourth driven joint. It aims the sensor, but it is not
part of the two-link Shoulder and Elbow inverse kinematics.

## How it is connected

```mermaid
flowchart LR
    Operator["Me or an AI client"] --> MCP["Typed arm tools"]
    Dashboard["Arm dashboard"] --> Backend["Local backend"]
    MCP --> Pi["Raspberry Pi gateway"]
    Backend --> Pi
    Pi --> Camera["OV5647 camera"]
    Pi --> HAT["ESP32 Arm HAT"]
    HAT --> Bus["1 Mbps serial-servo bus"]
    Bus --> Joints["Base, Shoulder, Elbow, Camera"]
```

The basic path is the laptop or AI tools to the Raspberry Pi, then the Pi talks
to the ESP32 Arm HAT, and the HAT controls the serial-bus servos. The Pi also
owns the camera and the higher-level movement checks.

## What I have actually proved so far

I want to keep this honest, because a software test is not the same thing as a
real arm test.

- Automated tests prove parts of the software, not the physical arm.
- Live telemetry proves what the controller measured at that moment.
- Sending a target does not automatically prove that the arm reached it.
- A camera frame only proves what is actually visible in that frame.
- Seeing the arm move is real physical evidence, but it is not a proper
  calibration table unless the numbers were recorded too.

By 2026-08-10, I had read back the Base configuration for native absolute
multi-turn control and physically checked normal zero, positive, negative,
and return movement on the reference arm.

By 2026-08-11, the motionless plan and preview system, plus the one-use apply
contract, had been deployed. The preview path was exercised, but the newer
plan/apply movement path still needs a properly recorded physical run before I
call that fully proved.

The full proof list, including what still needs a real device test, is in
[Current status](docs/STATUS.md).

## What is in this repo

- [`docs/`](docs/README.md) has the detailed arm documentation, current status,
  hardware, architecture, controls, safety, vision, assembly, commissioning,
  history, troubleshooting, and release notes.
- [`hardware/`](hardware/README.md) has the BOM and the prepared folders for
  native CAD, STEP, STL, drawings, schematics, and wiring.
- [`software/`](software/README.md) explains the clean software release plan and
  has prepared folders for the firmware, Pi gateway, MCP server, and dashboard.
- [`media/`](media/README.md) is where the approved photos, diagrams, and video
  links will go.
- [`tools/check_repo.py`](tools/check_repo.py) checks the repo for missing files,
  broken links, private paths, device IDs, secrets, oversized files, and common
  things that should not be pushed.

The working software currently lives in a larger private workspace. I am not
copying that whole workspace here because it also contains unrelated projects,
local runtime files, generated builds, and private machine details. The clean,
portable arm code will be added to the prepared software folders once its
public scope and licenses are decided.

## Where I will add the CAD, STL files, pictures, and videos

I made the folders now so I do not have to reorganize everything later:

1. Editable CAD goes in `hardware/cad/native/`.
2. Millimetre STEP exports go in `hardware/cad/step/`.
3. Oriented, print-ready millimetre STL files go in
   `hardware/stl/print-ready/`.
4. Dimensioned PDF or DXF drawings go in `hardware/drawings/`.
5. New unreviewed photos and videos first go in the ignored `media/raw/`
   folder.
6. After checking the background, metadata, labels, and anything personal, the
   approved files go into the matching tracked folder under `media/photos/`,
   `media/video/`, or `media/diagrams/`.

The exact file naming and export setup is in
[CAD and STL](docs/CAD_AND_STL.md). The picture and video shot list is in the
[Media guide](docs/MEDIA_GUIDE.md). There is also a simple upload checklist in
[Owner handoff](docs/OWNER_HANDOFF.md).

## If you want to reproduce it

Start with these pages:

1. [Current status and evidence](docs/STATUS.md)
2. [Control and safety boundaries](docs/CONTROL_AND_SAFETY.md)
3. [Hardware inventory](docs/HARDWARE.md)
4. [Electronics and power](docs/ELECTRONICS.md)
5. [Assembly](docs/ASSEMBLY.md)
6. [Commissioning](docs/COMMISSIONING.md)

Do not blindly copy the reference calibration. Every build needs its own
geometry measurements, joint zeros, limits, gearing checks, servo checks, and a
real physical way to cut servo power.

## What is still missing

The documentation structure is ready, but I still need to add the actual CAD,
STL files, drawings, approved pictures and videos, exact fastener and print
details, and the cleaned software exports. I also still need to decide the
public attribution and licenses.

Those remaining questions are tracked in
[Open questions](docs/OPEN_QUESTIONS.md). Until a license is added, this repo
does not grant an open-source or open-hardware license. See
[Licensing](LICENSING.md) before copying, manufacturing from, or redistributing
anything.
