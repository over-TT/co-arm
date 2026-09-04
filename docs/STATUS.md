# Status and evidence

Last public-source review: **2026-09-04**.

This page reports evidence, not aspiration. Historical hardware statements are
dated. Nothing in the repository is fresh live state.

Release assessment: a source/reference-build candidate, not a complete
copy-and-build kit. The exported physical recovery path has unresolved
Base-reference and archive/provenance blockers; see [Release review](RELEASE_REVIEW.md).

## Current public source snapshot

| Area | Included state | Evidence tier |
| --- | --- | --- |
| Dashboard | Standalone React/Vite Overview, guarded Control, bounded Live Follow, camera/calibration controls, Guide, Control Center, and SIM/REAL selection | Fresh locked install; 197 tests; 42-module production build; dependency audit; disconnected browser smoke |
| Local backend | Loopback-only FastAPI runtime, browser session boundary, Control Center, explicit SIM/REAL routing, and response-bound process identity | Source inspection; included in the 767-test Python run and installed-wheel smoke |
| Pi gateway | Four-joint state/calibration, floor guard, plan/apply/sequence, Live Follow, camera evidence, commissioning, Camera arrival tuning, and process-stable response identity | Source inspection; included in the 767-test Python run; no deployment in this review |
| Arm MCP/plugin | Typed Arm tools, explicit SIM/REAL identity verification, structured MCP output, repo-local marketplace, setup skill, and configured-arm runtime skill | MCP/index tests included in the full Python run; plugin/marketplace and both skills validated; no installed configuration changed |
| Isaac Sim | Four-joint URDF/model, deterministic desk/camera scene, bounded interpolation and frame-quality policy, persistent bridge, simulator HTTP gateway, and supervised launcher | Source/contracts tested; effective square-pixel Camera Module 3 Wide projection is explicit; Isaac runtime and vendor assets are not bundled |
| Arm HAT | `arm-hat-2.7.2` ESP32 controller source, shared library, and protocol fixtures | 17 host tests passed; device build/flash remains a separate tier |
| Operations | Pi service templates, deployment, physical-UART mode control, status, backup/restore/imaging, firmware compile helper, and Isaac launcher | Six scripts parsed; source/contracts tested; portable backup/restore has unresolved release blockers |
| Printable parts | All 12 current printed parts: both Base gears, the Base stack, arm links, servo mounts/covers, and camera holder/cover | Files inspected; final print settings are not included |

Current candidate verification completed on 2026-09-04 using Windows, Python
3.11.9, Node 24.14.1, and pnpm 11.19.0. Python used a new virtual environment;
dashboard dependencies were installed from the frozen lock in a separate copy
of the candidate source, without replacing an existing installation:

- gateway, backend, MCP, Isaac contract, deployment/recovery, and source tests:
  **767 passed, plus 9 nested subtests**; four Starlette deprecation warnings
  remain visible. Pytest also reported a Windows permission warning while
  cleaning an older temporary fixture; it did not fail a test;
- dashboard: **197 passed**, followed by a successful TypeScript/Vite
  production build with 42 modules transformed. jsdom reported its known
  missing-canvas implementation during an accessibility test;
- an isolated PEP 517 build produced and cleanly installed the
  `co_arm_stack-0.1.0-py3-none-any.whl` artifact; `pip check` and a
  source-isolated smoke loaded the packaged simulator URDF/JSON, confirmed test
  suites were excluded, proved package imports came from the isolated wheel
  environment, and confirmed that the dashboard CLI exits honestly
  when the separately built frontend assets are absent;
- no-gateway browser smoke: the built Overview and Guide loaded, no backend
  was configured, and Control/Live stayed disabled. A separate loopback test
  server was used; no real-arm service or simulator was started;
- Arm HAT host/protocol suite: **17 passed**;
- all **6** retained PowerShell operations scripts parsed without syntax
  errors;
- the `co-arm` plugin manifest and marketplace validated, and both setup and
  runtime skills passed their skill validators;
- the exact Python lock, Raspberry Pi requirements, and dashboard dependency
  graph returned no known vulnerabilities from their audit services;
- the source manifest is current across **155 software files**, and the
  repository checker reviewed **232 files** with only the documented missing
  license and approved-media warnings. The earlier missing-CAD warning is gone
  because all 12 current 3MF files are included.

The fresh Python run first failed collection because NumPy/OpenCV were absent
from the declared test dependencies. The test extra and regenerated lock now
include them; the complete suite above passed after the correction. This is
why an earlier already-populated interpreter is not clean-install evidence.

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

That path covers source validation and connection to an existing commissioned
arm. A supplied Isaac installation still needs its own runtime verification.
The portable physical recovery branch is incomplete and must not be treated
as ready for an SD-card loss. A reproducible new mechanical
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
