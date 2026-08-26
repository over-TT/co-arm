---
name: arm-alliance-setup
description: Set up, test, simulate, deploy, or commission the co-arm Raspberry Pi, dashboard, ESP32 Arm HAT, and printable build from a co-arm checkout.
---

# Arm Alliance Setup

Work from a co-arm checkout. Read `AGENTS.md` and
`docs/SETUP_WITH_CODEX.md`, then use `software/SOURCE_INDEX.json` to find the
source.

## Pick the path

- **Source only:** install the locked Python package, run the repository and
  Python checks, then test/build the dashboard.
- **Simulation:** start the Isaac bridge and simulator gateway, then connect the
  dashboard or MCP with `ARM_EXPECTED_BACKEND=sim`.
- **Existing arm:** inspect the Pi, camera, Arm HAT, servo bus, and current
  joints, then deploy or commission the part the user asked for.
- **New build:** use `hardware/3mf/`, `docs/ASSEMBLY.md`, and the BOM. Fill in
  the still-missing bearing, gear-file, insert, screw, print, wiring, and power
  details instead of guessing them.

## Work through it

1. Check the checkout, tool versions, and current Git diff.
2. Run the source checks before involving hardware.
3. For simulation, use the user's Isaac Python, start the bridge/gateway, open
   the dashboard or MCP, and stop it with the repo launcher when finished.
4. For an existing arm, begin with the Pi/camera/controller/bus status that is
   relevant to the request. `/healthz` only says the gateway answered; it does
   not say every servo is online.
5. When the user asks to deploy, flash, restart, calibrate, or move, identify
   the exact source and target, do the requested action, and return the result.
   Do not keep asking for the same permission. Ask only when the target is
   unclear, a real fault blocks it, or a tool needs exact confirmation.
6. Commission one joint at a time before coordinated motion. If the Base loses
   its place, line up the physical zero mark before setting zero again.
7. Configure `arm_mcp` only after choosing the simulator or real gateway and
   setting the matching `ARM_EXPECTED_BACKEND=sim|real`.

Torque off makes Shoulder and Elbow go limp, so hold them if they are raised.
Keep progress updates short and describe the physical result in normal words.
