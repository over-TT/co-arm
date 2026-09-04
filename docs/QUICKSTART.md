# Open the dashboard

This gets the standalone co-arm dashboard running on your computer. You can
explore the interface without a Raspberry Pi, servos, camera, Codex, or Isaac
Sim. Movement and camera controls become available after a gateway is connected.

## 1. Install the prerequisites

These commands use Windows PowerShell. Install Git, Python **3.11**, Node.js
**20.19.x or 22.12+**, and **pnpm 11.19.0**, then check:

```powershell
git --version
python --version
node --version
pnpm --version
```

The Python lock was validated on 3.11. Node 21 and Node 22.0–22.11 are outside
the dashboard's supported range. The pnpm version is pinned in
[`package.json`](../software/dashboard/package.json).

## 2. Get and build the software

If you already have the checkout, open PowerShell in its root and skip the
first two lines. Run each stage only after the preceding command succeeds.

```powershell
git clone https://github.com/over-TT/co-arm.git
Set-Location co-arm

python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install --requirement .\software\python\requirements.lock
.\.venv\Scripts\python.exe -m pip install --no-deps --editable .\software\python

pnpm --dir .\software\dashboard install --frozen-lockfile
pnpm --dir .\software\dashboard build
```

This creates a local Python environment and downloads the locked dependencies.
The frontend build goes in `software/dashboard/dist/`. Keep both in this
checkout; the Python package alone does not contain the built dashboard.

## 3. Open it

From the repository root:

```powershell
.\.venv\Scripts\python.exe -m web_backend
```

Open [http://127.0.0.1:8765/](http://127.0.0.1:8765/) and keep the terminal
running. The interface should load with no arm backend configured. Missing
joint readings, an unavailable camera, and unavailable movement controls are
expected at this stage. The Control Center can still offer local setup actions.

Press **Ctrl+C** in that terminal to stop this dashboard. To open it again,
repeat only the launch command. If port 8765 is already occupied, use
`--port 8766` and open `http://127.0.0.1:8766/`.

## Choose the next step

| You want to… | Continue here |
| --- | --- |
| Check the code and builds | [Source validation](SETUP_WITH_CODEX.md#1-clone-and-validate-without-hardware) |
| Move a simulated arm | [Isaac Sim setup](SETUP_WITH_CODEX.md#3-run-the-isaac-sim-arm); requires a separate Isaac installation |
| Connect an already commissioned arm | [Existing-arm path](SETUP_WITH_CODEX.md#existing-arm-short-path) |
| Build a new arm | [Assembly](ASSEMBLY.md), then [commissioning](COMMISSIONING.md); some physical build details remain unfinished |
| Let Codex use a connected arm | [External MCP setup](SETUP_WITH_CODEX.md#9-configure-the-external-mcp-agent) |

Opening the dashboard verifies the local interface. Simulator operation and
real-arm operation each need their own checks; see [status and evidence](STATUS.md).

## If it does not start

- **“No module named web_backend”:** use this checkout's `.venv` interpreter
  and repeat the editable-package install in step 2.
- **Missing dashboard assets:** run the dashboard build in step 2. A Python
  wheel installed elsewhere needs `--static-directory` pointing to that build's
  `dist` directory.
- **Dependency or build error:** confirm the prerequisite versions first and
  keep the lockfiles unchanged. Use the first failed command's error as the
  starting point.

The longer [setup runbook](SETUP_WITH_CODEX.md) covers source validation,
simulation, Pi deployment, hardware commissioning, and agent configuration.
