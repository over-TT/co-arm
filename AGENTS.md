# Agent instructions for co-arm

This is the focused repository for the four-joint camera arm. Before changing
behavior or making a setup claim, read:

1. `README.md`;
2. `docs/SETUP_WITH_CODEX.md`;
3. `docs/STATUS.md` and `docs/KNOWN_LIMITATIONS.md`;
4. `docs/CONTROL_AND_SAFETY.md`;
5. `docs/COMMISSIONING.md` for any physical path;
6. `software/SOURCE_INDEX.json` to locate implementation source.

## Supported scope

Keep only these product planes:

- `software/dashboard/` — standalone arm control, calibration, camera, and
  bounded Live Follow;
- `software/python/web_backend/` — loopback dashboard runtime and SIM/REAL
  routing;
- `software/python/robot_gateway/` — Raspberry Pi gateway and Camera Module 3
  path;
- `software/python/arm_sim/` — core Isaac Sim digital twin and gateway;
- `software/python/arm_mcp/` and `software/plugin/` — supported external agent
  integration;
- `software/firmware/` — ESP32 Arm HAT firmware and protocol;
- `software/operations/` — deployment, diagnostics, recovery, build, and
  simulator helpers.

Do not reintroduce unrelated product features, evaluation harnesses, parent-app
UI, generic board firmware, Tauri surfaces, private reports, or duplicate
source trees. The dashboard deliberately does not bundle an embedded Codex
runtime; a repository-aware agent plus the external MCP plugin is the supported
path.

## Working style

Keep the conversation short and physical. Say what the arm is doing in normal
words: look, preview, move, read the joints, take a picture.

- A clear request to move or use the configured arm is the authorization for
  that task. Do not turn every waypoint into another permission question.
- Start from current state, show the planned move, run it, then check what the
  joints or camera say happened.
- Torque off makes Shoulder and Elbow go limp. Say that once when it matters;
  do not wrap it in a generic safety lecture.
- If the Base loses its place, the missing fact is physical: line up the zero
  mark before using **Set zero here**.
- Ask again only when a missing fact changes the action, a real fault blocks it,
  or the tool itself requires an exact confirmation.

## Setup workflow

Maintain a checklist with `confirmed`, `unknown`, `blocked`, and
`not-applicable` states. Advance in this order:

1. **Checkout:** inventory files, tool versions, source manifest, and Git diff.
2. **Host validation:** create/use an explicit Python environment, install the
   local package, run Python tests, then install/test/build the dashboard.
3. **Simulation:** verify the Isaac version/launcher supplied by the user,
   create local simulator tokens, start the persistent bridge/gateway, prove
   its exact identity and camera smoke path, then stop it through the
   authenticated launcher.
4. **Physical inventory:** resolve Pi/HAT/camera/servo labels, supply,
   protection, wiring, mechanical support, zero marks, and physical power cut.
5. **Read-only Pi discovery:** inspect OS/Python/camera, gateway process,
   controller link, servo bus, STOP, torque, Base continuity, and telemetry.
   `/healthz` alone is not arm health.
6. **Build/deploy/flash:** when the user asks for it, identify the exact source
   and target, do the requested write, and return the result. Ask only if the
   target is genuinely unclear or the tool requires an exact confirmation.
7. **Commissioning:** identify and calibrate one joint at a time, then verify
   small measured motion, STOP/watchdog/lease/restart behavior, camera delivery,
   and finally coordinated motion.
8. **Operation:** use fresh state/scene/pixels and reviewed plan/apply tools;
   report only the strongest evidence returned.

All 12 current printed parts are included. A complete new build still needs the
bearing details, inserts/screws, print settings, wiring, and power details. Name
the missing item instead of guessing it.

## Change and evidence rules

- State where each command runs and whether it reads, writes, flashes,
  restarts, enables torque, or moves hardware, but keep it concise.
- Treat `arm_status.py --scan` as a commissioning state/electrical mutation,
  not a read-only ping; the default status read is the non-mutating path.
- Say whether a result came from reading code, running tests, building it,
  reading the real joints, or looking at the camera.
- Documentation or a named viewpoint is never the current pose.
- Command acceptance is not measured arrival.
- Configure MCP with an explicit `ARM_EXPECTED_BACKEND=sim|real`; the loopback
  URL or port is not backend identity. A lost write response is an unknown
  outcome, not permission to replay the action.
- The reference camera is Raspberry Pi Camera Module 3 Wide / IMX708.
  Autofocus needs same-capture metadata plus visible subject detail.
- Base continuity loss requires physical alignment and explicit re-zeroing;
  never reconstruct output turns from a wrapped encoder.
- Preserve the floor guard and the Pi/ESP32 safety split.
- STOP, stale required telemetry, controller/bus faults, untrusted Base truth,
  or a refused sweep block motion.
- Do not commit credentials, endpoints, SSH material, runtime databases,
  captures, logs, backups, toolchains, generated scenes/builds, or operator
  memory.

Before proposing a commit after software changes:

```powershell
python tools/build_source_manifest.py
python tools/build_source_manifest.py --check
python tools/check_repo.py
```

Run the affected Python, dashboard, firmware, and PowerShell checks as well.
Passing software checks never proves a physical arm. No public reuse terms
exist until explicit licenses are added.
