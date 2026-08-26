# Raspberry Pi gateway package

`robot_gateway` contains the authenticated HTTP API, camera evidence path,
serial Arm HAT controller, calibration/state stores, Simple Arm safety policy,
live-follow control, and Isaac adapters. It is exported as a complete package
because those modules share safety and state contracts. The Pi runtime does
not host a second synthetic simulator; SIM is the separate Isaac gateway.

The systemd templates and deployment helpers live in `software/operations/`.
They require explicit host, SSH identity, known-hosts, network, and controller
inputs. Runtime tokens, calibration profiles, logs, captures, and backups are
intentionally absent.

Run the package tests from `software/`:

```powershell
$env:PYTHONPATH = "python"
python -m pytest -q python/robot_gateway/tests
```

These tests are source evidence only; they do not contact or move an arm.
