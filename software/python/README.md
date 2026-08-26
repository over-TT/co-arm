# Python stack

This directory contains only the ARM-specific Python packages:

- `robot_gateway`: Raspberry Pi API, camera, calibration, safety, motion, and
  live-follow control;
- `arm_sim`: source-only Isaac Sim model, bridge, scene, and contract tests;
- `arm_mcp`: typed MCP tools that talk to a configured gateway;
- `web_backend`: the ARM-only local dashboard proxy and supporting services.

From the repository root, create the canonical project environment, install the
stack, and run its tests with that exact interpreter:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --requirement ".\software\python\requirements.lock"
.\.venv\Scripts\python.exe -m pip install --no-deps --editable ".\software\python"
Set-Location software\python
..\..\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
Set-Location ..\..
```

Build the frontend in `../dashboard`, then start the local UI with no live
backend:

```powershell
.\.venv\Scripts\python.exe -m web_backend
```

Add one or both complete credential pairs to expose REAL or SIM. Token files
stay outside Git and are never sent to the browser:

```powershell
.\.venv\Scripts\python.exe -m web_backend `
  --real-url http://127.0.0.1:8787 `
  --real-token-file <absolute-owner-only-token-path>
```

The server binds only to `127.0.0.1`, validates the browser action token, and
requires each selected backend on every ARM request. Agent access uses the
separate `arm_mcp` package and an explicit machine-local MCP configuration.
Isaac Sim is started by `../operations/scripts/run_arm_sim.py`; its installation
path, tokens, logs, and generated scene remain local and outside version control.
