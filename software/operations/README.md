# ARM operations

This directory contains the source-only deployment and recovery tools for the
Raspberry Pi arm gateway, core Isaac Sim launcher, and ESP32 Arm HAT firmware
build.

All installation-specific values are inputs. Supply the target host, service
account/group, SSH identity, pinned known-hosts file, controller/firmware
identity, and network addresses explicitly; do not commit those values or the
generated runtime artifacts. The scripts keep tokens, manifests, backups, disk
images, logs, and build output under an ignored runtime directory or a caller-
selected build directory.

Run scripts from any working directory; paths to source files are resolved from
the script location. The shared source-root convention is `software/`, with
Python packages in `software/python/`, operational files here, and firmware in
`software/firmware/`.

The deployment scripts configure only the arm gateway. The base service is
loopback-only and starts with the physical UART dormant. First activation and
its dormant rollback use the separate, approval-gated gateway-mode operation.

`scripts/arm_status.py` is read-only unless `--scan` is supplied. A scan is a
commissioning mutation: it clears outstanding motion proposals, may request
torque-off, and can invalidate Base truth until it is re-established.
Its bearer-token gateway URL is restricted to plain HTTP on an explicit local
loopback port; it rejects URL suffixes and bypasses redirects and proxies.

## Tool map

- `scripts/run_arm_sim.py` creates simulator-only tokens, supervises the core
  Isaac bridge and HTTP gateway, and writes its scene, state, and logs only
  under `software/runtime/arm-sim/`.
- `scripts/deploy-arm-gateway.ps1` installs the reviewed gateway package and
  loopback-only service. It stops and restarts the service; a live hold can
  expire and a gravity-loaded arm can sag, so support the mechanism first.
- `scripts/set-arm-gateway-mode.ps1` reversibly activates or deactivates only
  the reviewed physical-UART drop-in on an existing prepared Pi. It requires
  current supported-arm confirmation, preserves the fail-closed latch, and
  proves the requested mode without clearing STOP, taking torque, or moving.
- `scripts/arm_status.py` separates gateway, controller, bus, STOP, and joint
  state. Use the default read-only form before considering `--scan`.
- `scripts/restore-arm-pi.ps1`, `scripts/backup-arm-pi.ps1`, and
  `scripts/image-arm-pi-sd.ps1` implement the bounded Pi recovery path.
- `scripts/compile-firmware.ps1` builds only the ESP32 Arm HAT source and never
  flashes it.

For the ordered agent-assisted setup flow, start at
[`../../docs/SETUP_WITH_CODEX.md`](../../docs/SETUP_WITH_CODEX.md).
