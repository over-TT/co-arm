# Set up co-arm with Codex

This is the shared setup runbook for a human and an AI agent. It is designed so
the same steps still make sense when a Raspberry Pi is already connected, when
the hardware is only partly assembled, or when someone is only exploring the
documentation.

The human owns the physical setup, power, work area, and approvals. Codex owns
the checklist, source inspection, read-only discovery, explanations, and exact
next-step proposals. The Raspberry Pi and ESP32 remain separate policy and
real-time control layers.

## Current release boundary

This checkout is currently documentation-first. It contains the architecture,
hardware record, BOM, assembly and commissioning order, AI rules, release
checks, and folders for future CAD, media, and software.

It does **not** yet contain runnable firmware, the Raspberry Pi gateway, the MCP
server, or a standalone dashboard. It also does not yet contain the final CAD,
STL files, schematics, verified wiring drawings, or a complete fastener list.

That means:

- guided planning and Raspberry Pi discovery can start now;
- wiring and hardware facts can be audited now;
- setup records and missing-file lists can be prepared now;
- this checkout cannot yet be treated as an installable or flashable package;
- Codex must stop instead of inventing an install, service, flash, or motion
  command when the required source is absent.

Read [`../software/README.md`](../software/README.md) for the exact software
release status.

## Who does what

| Human | Codex | Pi / ESP32 stack |
| --- | --- | --- |
| Identifies hardware and checks labels | Reads the repo and maintains the checklist | Reports live state through the approved interface |
| Supports the arm and clears the sweep | Explains each command before it runs | Enforces authentication, limits, watchdogs, STOP, and torque rules |
| Owns servo power and the physical disconnect | Begins with read-only checks | Never turns a requested target into proof of arrival |
| Approves wiring, flashing, deployment, and motion | Stops when a critical fact or authority is missing | Returns telemetry and camera evidence when those services exist |
| Confirms what physically happened | Separates source, live, and physical proof | Cannot replace the human's physical inspection |

## 1. Clone and check the documentation

Install Git and Python 3.10 or newer. The repository checker uses only the
Python standard library.

```bash
git clone https://github.com/over-TT/co-arm.git
cd co-arm
python tools/check_repo.py
```

On systems where Python 3 is named `python3`, use:

```bash
python3 tools/check_repo.py
```

A passing result means the documentation structure, local Markdown links, file
sizes, and common privacy rules passed. It does not prove wiring, firmware,
deployment, calibration, or physical motion.

## 2. Start the Codex guide

Open the cloned `co-arm` folder as the Codex workspace. `AGENTS.md` tells Codex
which project contracts to read and what it must not assume.

Paste this starter prompt:

```text
Help me set up co-arm from this checkout. Read AGENTS.md,
docs/SETUP_WITH_CODEX.md, docs/STATUS.md, docs/KNOWN_LIMITATIONS.md,
docs/CONTROL_AND_SAFETY.md, and docs/COMMISSIONING.md before acting.

My Raspberry Pi is [not connected / connected over SSH at <host>]. My arm is
[not assembled / partly assembled / assembled]. Servo power is [off / on / I
do not know]. Start by checking what files and hardware are actually available.
Begin with read-only, no-motion checks. Ask me only for critical missing
hardware facts. Do not move the arm, release torque, change wiring, flash
firmware, deploy services, or disable a guard until you show me the exact step
and I approve it. Keep source checks, live telemetry, command acceptance,
physical arrival, and camera proof separate. Maintain a setup checklist as we
go.
```

Do not paste passwords, tokens, private keys, private addresses, serial numbers,
or personal camera frames into the prompt or the repository.

## 3. Codex first-read and inventory

Before suggesting commands, Codex should:

1. Read `AGENTS.md` and this guide.
2. Read [`STATUS.md`](STATUS.md), [`KNOWN_LIMITATIONS.md`](KNOWN_LIMITATIONS.md),
   [`CONTROL_AND_SAFETY.md`](CONTROL_AND_SAFETY.md), and
   [`COMMISSIONING.md`](COMMISSIONING.md).
3. Read [`../software/README.md`](../software/README.md) and inspect the actual
   `software/` files instead of assuming the source has been released.
4. Read [`HARDWARE.md`](HARDWARE.md), [`ELECTRONICS.md`](ELECTRONICS.md), and
   [`ASSEMBLY.md`](ASSEMBLY.md) for a physical build.
5. Run `python tools/check_repo.py` from the repository root.
6. Create a checklist with `confirmed`, `unknown`, `blocked`, and `not yet
   applicable` states.
7. State the current proof tier before making any claim.

If the software folders contain only their README placeholders, Codex should
say that clearly and continue with documentation, hardware intake, or Pi
discovery. It must not make up package names or deployment commands.

## 4. Critical hardware intake

The setup can proceed only as far as the known facts allow. Codex should ask for
these when the matching stage becomes relevant:

- exact printed model and voltage on all four servos;
- exact servo HAT model, revision, jumpers, and switches;
- exact power-supply model, voltage, current, connector, and polarity;
- camera-servo regulator model, setting, and measured output;
- fuse, reverse-polarity, and over-current protection;
- reachable physical servo-power disconnect or E-stop behavior;
- connector pinouts, wire gauges, and common-ground layout;
- Base tooth counts or confirmed ratio and a durable zero mark;
- mechanical support behavior when torque is removed;
- actual geometry, travel limits, cable limits, and hard stops;
- ownership and licenses of CAD or vendor models.

Unknown information stays marked unknown. A successful bus response must never
be used to guess a servo model, voltage, or mechanical joint mapping.

## 5. Prepare a Raspberry Pi without moving the arm

### Human preparation

1. Remove servo-rail power.
2. Support any gravity-loaded links.
3. Keep a tested physical way to remove servo power within reach.
4. Clear the possible sweep even though no movement is planned.
5. Connect only the Raspberry Pi logic power and camera needed for discovery.
6. For a new Pi, use
   [Raspberry Pi Imager](https://www.raspberrypi.com/documentation/computers/getting-started.html#install-using-imager)
   to install Raspberry Pi OS and enable SSH. Key-based authentication is
   preferred for a long-lived headless setup.

### Connect over SSH

From the computer running Codex:

```bash
ssh <pi-user>@<pi-host>
```

Keep the real username, hostname, address, and key path in local configuration.
Do not add them to the repo, screenshots, issues, or setup logs.

See Raspberry Pi's official [remote access guide](https://www.raspberrypi.com/documentation/computers/remote-access.html#ssh)
for current SSH setup options.

### Read-only discovery

Run these on the Pi, one block at a time, and let Codex explain what each result
does and does not prove:

```bash
uname -a
cat /etc/os-release
python3 --version
rpicam-hello --list-cameras
```

Expected proof boundaries:

| Check | What it proves | What it does not prove |
| --- | --- | --- |
| SSH login | This computer reached that SSH account | Servo power, gateway, or controller health |
| `uname -a` / OS release | Current kernel and OS information | Compatibility with unreleased arm software |
| Python version | A Python interpreter is available | Required packages or services are installed |
| Camera listing | The current camera stack detected a camera | Correct orientation, focus, framing, or a usable arm capture |

Camera preview or capture is a separate step because pixels may contain private
desk details. Review where any image will be stored before taking it.
Raspberry Pi's official [camera software guide](https://www.raspberrypi.com/documentation/computers/camera_software.html)
documents `rpicam-hello` and the current `rpicam-*` utilities.

## 6. Build a setup plan

Codex should now return a short plan split into these lanes:

1. **Available now:** documentation review, Pi discovery, parts audit, BOM
   updates, CAD/media placement, and missing-evidence collection.
2. **Waiting for files:** firmware build, gateway install, MCP configuration,
   dashboard start, and clean-checkout software tests.
3. **Waiting for hardware facts:** wiring, voltage, servo addressing,
   calibration, and power qualification.
4. **Waiting for explicit approval:** wiring changes, flashing, deployment,
   torque changes, STOP reset, guard changes, and motion.

Every proposed action should name:

- where it runs: laptop, Pi, ESP32, or physical arm;
- whether it writes or moves anything;
- the exact expected result;
- what success would prove;
- the rollback or stop condition.

## 7. Continue when the software source is released

Once executable source is actually present, Codex should verify the release
manifest and follow this order:

1. Inspect source and configuration templates.
2. Install dependencies in a clean environment using the released manifests.
3. Run the source tests that belong to that package.
4. Build firmware with the documented board target and toolchain.
5. Keep credentials, endpoints, and machine paths in local ignored files.
6. With servo power still off, deploy the Pi gateway and verify process health.
7. Establish the typed ESP32 handshake without motion.
8. Commission one isolated servo at a time.
9. Assemble the labelled bus and refresh live state.
10. Calibrate each supported joint with torque off.
11. Verify geometry and floor policy without moving.
12. Prepare a small motionless plan and review it.
13. Ask for deliberate operator approval before the first apply.
14. Read measured state after settling and record physical observations
    separately.

The exact commands must come from the released source and manifests. Until
those exist, use [`COMMISSIONING.md`](COMMISSIONING.md) as the sequence, not as
permission to improvise commands.

## 8. Non-motion gate before any move

Before proposing motion, Codex and the human should both confirm:

- the physical servo-power cut is reachable and tested;
- gravity-loaded links are supported;
- the full planned sweep is clear;
- the controller and bus are online;
- required joints have fresh, trusted telemetry;
- torque state is known;
- STOP is clear by deliberate operator decision;
- the floor guard is enabled;
- Base continuity is valid, or Base is not part of the move;
- the exact plan, sweep, clearance, and warnings were reviewed.

If one item is unknown, stop on that item. Do not solve a failed guard by
turning the guard off.

## 9. Evidence and progress format

Codex should keep a compact table like this during setup:

| Stage | State | Evidence | Next owner action |
| --- | --- | --- | --- |
| Repo check | confirmed | Named command and result | None |
| Pi SSH | unknown | No live check yet | Provide local host or connect manually |
| Camera discovery | blocked | Pi not connected | Connect camera with Pi powered off |
| Software export | not yet applicable | Placeholder folders only | Wait for source release |
| Servo power | unknown | No label or schematic evidence | Photograph and transcribe supply path |

Use these evidence labels consistently:

- **source:** current file inspection;
- **test:** named command and result;
- **build:** successful artifact build;
- **deployed:** verified running version after deployment;
- **live:** fresh telemetry from the device;
- **accepted:** a command was accepted;
- **arrived:** fresh measured state after settling;
- **image:** visible pixels from a named capture;
- **operator:** a clearly recorded physical observation.

## 10. Rules for the AI guide

- Never treat documentation as the current pose or live state.
- Never infer an output revolution from a wrapped Base reading.
- Never clear STOP automatically.
- Never disable the floor guard to get around a failed plan.
- Never flash while the Pi UART and USB host can both drive the ESP32 serial
  lines.
- Never use raw HTTP, serial, or register commands to bypass typed controls.
- Never call a fixed-focus camera autofocus-capable.
- Never store live pose, credentials, device identifiers, private network
  details, or personal images in public context.
- Never promote a passing software check into physical proof.
- Stop immediately when the operator says stop.

## Where to continue

- Physical build: [`ASSEMBLY.md`](ASSEMBLY.md)
- Power and wiring: [`ELECTRONICS.md`](ELECTRONICS.md)
- First hardware run: [`COMMISSIONING.md`](COMMISSIONING.md)
- Control rules: [`CONTROL_AND_SAFETY.md`](CONTROL_AND_SAFETY.md)
- Pi, firmware, MCP, and UI architecture:
  [`SOFTWARE_ARCHITECTURE.md`](SOFTWARE_ARCHITECTURE.md)
- Camera tasks: [`VISION.md`](VISION.md)
- Failure diagnosis: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)
- Missing owner facts: [`OPEN_QUESTIONS.md`](OPEN_QUESTIONS.md)
