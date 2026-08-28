# Status and evidence

Last public-source review: **2026-08-28**.

This page reports evidence, not aspiration. Historical hardware statements are
dated. Nothing in the repository is fresh live state.

## Current public source snapshot

| Area | Included state | Evidence tier |
| --- | --- | --- |
| Dashboard | Standalone React/Vite Overview, guarded Control, bounded Live Follow, camera/calibration controls, Guide, Control Center, and SIM/REAL selection | Source inspection; unchanged in the 2026-08-28 camera/runtime port; the previous 180 tests, 42-module production build, and high-severity dependency audit passed |
| Local backend | Loopback-only FastAPI dashboard runtime, browser session boundary, Control Center, camera, and explicit SIM/REAL proxy routing | Source inspection; included in the 725-test Python run |
| Pi gateway | Bearer authentication, four-joint state/calibration, floor guard, plan/apply/sequence, Live Follow, camera evidence, physical commissioning, and Arm HAT client | Source inspection; included in the 725-test Python run |
| Arm MCP/plugin | Typed Arm tools, explicit SIM/REAL identity verification, structured MCP output, repo-local Codex marketplace, setup skill, and configured-arm runtime skill | 49 focused MCP/index tests passed; the plugin and both skills validated; credentials and installed MCP configuration are intentionally local |
| Isaac Sim | Four-joint URDF/model, deterministic desk/camera scene, bounded interpolation and frame-quality policy, persistent bridge, simulator HTTP gateway, and supervised launcher | Source/contracts tested; effective square-pixel Camera Module 3 Wide projection is explicit; Isaac runtime and vendor assets are not bundled |
| Arm HAT | `arm-hat-2.7.2` ESP32 controller source, shared library, and protocol fixtures | 17 host tests passed; device build/flash remains a separate tier |
| Operations | Pi service templates, deployment, reversible physical-UART mode control, status, backup/restore/imaging, firmware compile helper, and Isaac launcher | Source/contracts tested; six retained PowerShell scripts parsed |
| Printable parts | All 12 current printed parts: both Base gears, the Base stack, arm links, servo mounts/covers, and camera holder/cover | Files inspected; final print settings are not included |

Current camera/runtime export verification completed on 2026-08-28. Dashboard,
packaging, PowerShell, and dependency files were unchanged; their prior
2026-08-26 evidence remains listed separately below:

- gateway, backend, MCP, Isaac contract, deployment/recovery, and source tests:
  **725 passed**; four upstream Starlette
  deprecation warnings remain visible;
- unchanged dashboard evidence from 2026-08-26: **180 passed**, followed by a successful TypeScript/Vite
  production build with 42 modules transformed;
- an isolated PEP 517 build produced and cleanly installed the
  `co_arm_stack-0.1.0-py3-none-any.whl` artifact; `pip check` and a
  source-isolated smoke loaded the packaged simulator URDF/JSON, confirmed test
  suites were excluded, and confirmed that the dashboard CLI exits honestly
  when the separately built frontend assets are absent;
- no-gateway dashboard smoke: HTML plus both generated assets returned 200,
  the browser session issued a bounded action token, zero arm backends were
  configured, and the fixed Control Center manifest loaded;
- Arm HAT host/protocol suite: **17 passed**;
- all **6** retained PowerShell operations scripts parsed without syntax
  errors;
- the `co-arm` plugin manifest and marketplace validated, and both setup and
  runtime skills passed their skill validators;
- the exact Python lock, Raspberry Pi requirements, and complete dashboard
  dependency graph returned no known vulnerabilities from their current audit
  services;
- the source manifest is current across **153 software files**, and the
  repository checker reviewed **227 files** with only the documented missing
  license and approved-media warnings. The earlier missing-CAD warning is gone
  because all 12 current 3MF files are now included.

The checked-in GitHub workflow now repeats the source, test, build, package,
PowerShell, and dependency-audit gates, and Dependabot covers Python, Raspberry
Pi requirements, pnpm, and GitHub Actions. A GitHub-hosted run still requires
the exact candidate to be pushed; local inspection does not prove that remote
run.

The deterministic manifest and repository checker are the final tree-level
gate after any edit. These checks prove only the exported source and local build
environment; they do not prove a deployed service, Isaac runtime, flashed
controller, or physical arm. Re-run them for the exact commit being published.

## Agent-guided setup boundary

`AGENTS.md`, `docs/SETUP_WITH_CODEX.md`, `software/SOURCE_INDEX.json`, and the
repo-local setup skill give an agent an ordered path through checkout validation,
dashboard startup, Isaac setup, blank-Pi prerequisites, gateway deployment,
firmware compilation, tunnel/MCP configuration, commissioning, and operation.

That path is complete for source validation and is actionable for a supplied
Isaac installation or an existing physical arm. A reproducible new mechanical
build is now partly documented by all 12 printable parts. It still needs the
bearing details, full fastener and print schedule, as-built wiring/pinout, and
power details before it becomes a complete copy-and-build package.

## Current reference-build contract

| Area | Reference state | Evidence boundary |
| --- | --- | --- |
| Mechanism | Four driven joints: Base, Shoulder, Elbow, Camera | Source plus earlier physical observation |
| Geometry | 60 mm pivot, 180 mm upper arm, 220 mm elbow-to-tip | Configured/measured reference values, not manufacturing tolerances |
| Gateway | Raspberry Pi 4, Camera Module 3 Wide / IMX708, ESP32 Arm HAT | Source plus dated deployed evidence |
| Base | ST3215/STS plus 3D-printed 52:13 gear pair for 4:1 reduction; native Mode-0 signed absolute goals | Owner-supplied Base 3MF metadata plus earlier register readback and physical motion acceptance |
| Shoulder / Elbow | ST3215/STS-family path; grouped bounded Live Follow available | Source plus dated unloaded physical runs |
| Camera joint | SC09/SCS-family path, excluded from endpoint IK | Source tests plus earlier bus/motion observations |
| Vision | 2304 x 1296 survey and 4608 x 2592 detail profiles | Source plus dated deployed capture/status evidence |

## Previously recorded reference-arm evidence

These facts were recorded on the private reference system before this public
export. They are historical device evidence, not live verification made from
this checkout:

- The Pi, Camera Module 3 Wide, Arm HAT, and four servo IDs operated together.
- Native Base extended-position configuration and signed motion were read back
  and physically exercised after the earlier Mode-3 path was retired.
- Camera Module 3 Wide reported the IMX708 sensor/array and delivered upright
  survey/detail captures with powered autofocus controls. Focus still requires
  same-capture AF metadata plus visible subject detail.
- On 2026-08-23, matching Arm HAT `2.7.2`, Pi, and dashboard code completed
  unloaded Shoulder/Elbow Live Follow runs at raw speed `2400`, acceleration
  `50`, and a `±90 deg` start-relative allowance for five and twelve seconds.
- The twelve-second run recorded 223 accepted input frames, 52 verified grouped
  dispatches, 89 heartbeats, and an orderly client release.

That Live Follow evidence proves only the tested unloaded path. Raw speed is not
degrees per second, acceleration `50` is the installed-servo policy limit, and
the run does not establish every pose, load, overshoot, collision, cable, power,
or thermal condition.

## Still requiring live/device/operator proof

- An installed Isaac Python launcher, compatible GPU/driver stack, and a passing
  bridge/camera self-test; Isaac was not launched during this source review.
- Arduino CLI/ESP32-core compilation, upload, readback, and installed firmware
  identity; Arduino CLI was unavailable during this review.
- Pi deployment, service restart/hold behavior, loopback tunnel, camera, UART,
  controller, bus, and servo state on the target machine.
- Fresh STOP, torque, Base-continuity, calibration, telemetry, measured arrival,
  floor-clearance, cable/obstacle, power-sag, current, thermal, and stall checks.
- Remaining CAD and drawings; bearing details; fasteners; materials and print
  settings; as-built wiring/pinout; power details; and reviewed public media.

## Distribution boundary

No license has been selected, so public reuse terms are not granted. Runtime
state, captures, private reports, credentials, backups, toolchains, installed
agent configuration, and unrelated workspace code are intentionally absent.
The dashboard contains no embedded agent runtime; agent operation uses the
external MCP server/plugin with machine-local configuration.
