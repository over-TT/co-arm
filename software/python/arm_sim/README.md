# Arm simulation lab

`arm_sim` is the source-only Isaac Sim workspace for the four-axis desk camera
arm. It provides the URDF, provisional model and camera manifests, deterministic
desk/camera scene construction, an authenticated loopback bridge, and Isaac-free
contract tests. Generated USDs, captures, logs, tokens, and runtime state are
deliberately excluded.

The simulator shares the public gateway contract, but simulation results never
prove physical-arm behavior. Geometry, optics, drive dynamics, backlash,
compliance, desk registration, and contact properties remain provisional until
measured on a particular build.

## Layout

- `assets/desk_camera_arm.urdf` — four-joint source robot description.
- `config/` — provisional model and camera contracts.
- `bridge/` — authenticated loopback protocol and simulator HTTP gateway.
- `isaac/` — URDF importer, persistent bridge, desk/camera scene, and viewer.
- `tests/` — source and protocol checks that do not launch Isaac.

## Checks

From `software/`:

```powershell
$env:PYTHONPATH = "python"
python -m pytest -q python/arm_sim/tests
python -m compileall -q python/arm_sim
```

## Run Isaac

Pass the installed Isaac Python launcher explicitly; no machine-specific Isaac
path or bundled runtime is committed:

```powershell
$env:PYTHONPATH = "python"
python operations/scripts/run_arm_sim.py --isaac-python <path-to-isaac-python>
```

Use `--gui` for the interactive window, `--stop` for the authenticated local
supervisor stop, or `--ensure-tokens` to create the two simulator-only token
files: one for the private Isaac bridge and one for the simulator HTTP gateway.
The bridge, gateway, and supervisor bind only to `127.0.0.1` on ports 8790,
8788, and 8791 respectively. The generated scene, tokens, state, and logs stay
below the ignored `software/runtime/arm-sim/` directory.
