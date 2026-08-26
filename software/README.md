# ARM software

This is the executable, ARM-only source boundary. Its project root is this
`software/` directory; runtime state belongs in ignored `software/runtime/`.

| Folder | Contents |
| --- | --- |
| `dashboard/` | Standalone Vite/React Control Center, calibration, camera, and bounded Live Follow |
| `python/web_backend/` | Loopback-only FastAPI host and explicit SIM/REAL routing |
| `python/robot_gateway/` | Raspberry Pi gateway, camera, serial controller, calibration, motion policy, and tests |
| `python/arm_sim/` | Core authenticated Isaac bridge, digital-twin assets, viewer, gateway, and tests |
| `python/arm_mcp/` | Typed ARM MCP server |
| `firmware/` | Arm HAT firmware/library, protocol fixtures, and host tests |
| `operations/` | Parameterized deployment, recovery, status, service, firmware-build, and simulator helpers |
| `plugin/` | Repo-local Codex marketplace, setup/runtime skills, and machine-local MCP setup guidance |

[`SOURCE_INDEX.json`](SOURCE_INDEX.json) maps the canonical entrypoints.
[`SOURCE_MANIFEST.json`](SOURCE_MANIFEST.json) records every releasable software
file and digest; regenerate it from the repository root with
`python tools/build_source_manifest.py` after an intentional source change.

Quick verification from the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --requirement ".\software\python\requirements.lock"
.\.venv\Scripts\python.exe -m pip install --no-deps --editable ".\software\python"

Set-Location software\python
..\..\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
..\..\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider ..\firmware\tests\test_arm_hat_controller_v1.py

Set-Location ..\dashboard
pnpm install --frozen-lockfile
pnpm test
pnpm build
Set-Location ..\..
```

Run the dashboard with `.\.venv\Scripts\python.exe -m web_backend`. A clean
checkout starts without a physical or simulated gateway; pass an explicit
loopback URL and local token-file pair to enable one. The supported agent path
is the repository contract plus external plugin; Codex binaries, authentication,
installed plugin configuration, live credentials, retained frames, databases,
device backups, generated scenes, and toolchains are deliberately not bundled.
