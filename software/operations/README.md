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
  the reviewed physical-UART drop-in on an existing prepared Pi. It preserves
  the fail-closed latch and accepts activation only after authenticated live
  proof of the exact controller, firmware, required capabilities, and four
  fresh stationary torque-off joints. It never clears STOP, takes torque, or
  moves.
- `scripts/arm_status.py` separates gateway, controller, bus, STOP, and joint
  state. Use the default read-only form before considering `--scan`.
- `scripts/restore-arm-pi.ps1`, `scripts/backup-arm-pi.ps1`, and
  `scripts/image-arm-pi-sd.ps1` contain the Pi recovery tooling. Portable
  recovery is incomplete; see the release blocker below.
- `scripts/compile-firmware.ps1` builds only the ESP32 Arm HAT source and never
  flashes it.

For the ordered agent-assisted setup flow, start at
[`../../docs/SETUP_WITH_CODEX.md`](../../docs/SETUP_WITH_CODEX.md).

## Pi source and snapshot contract

Make reusable gateway changes in this checkout, validate them, and deploy those
exact bytes. `backup-arm-pi.ps1` computes a canonical SHA-256 contract for the
reviewed 13 gateway modules plus `requirements-pi.txt` and refuses the snapshot
if the frozen deployed copies differ. A live-Pi edit is therefore drift to fix,
not a new source of truth.

Take a protected snapshot before each reachable persistent Pi mutation when a
useful prior state exists, then always take and verify another after the change.
The parameterized archive retains same-device calibration, recovery gates,
service/network/boot evidence, optional Base and recovery markers, credentials,
and media binding under ignored `software/runtime/robot-gateway/pi-backups/`.
It is private recovery state for that Pi, not a template for another person's
credentials, Base frame, network, or host identity.

## Recovery release blocker

Do not use the exported `restore-arm-pi.ps1` for physical recovery until the
scripts and gateway are fixed together and a full backup-to-restore round trip
passes. A valid archive checksum proves saved bytes, not a restorable kit.

- Fresh deployment does not create the recovery manifest/provenance files
  required by `backup-arm-pi.ps1`.
- Normal calibration updates `arm-joints.json` but can leave its recovery
  provenance hash stale. Restore requires separate token, calibration, and
  provenance inputs and rejects that mismatch; it has no archive import path.
- Restored Base calibration needs an enforced invalid-reference gate until
  physical re-zeroing. The gateway can currently pair an old restored zero with
  a surviving valid HAT frame. Its generic STOP latch can be cleared without
  establishing that new reference. A cold HAT continuity loss still blocks Base.

The snapshot requirement above remains in force. Do not fabricate provenance
or clear STOP to bypass these gaps. Keep existing protected archives and consult
the [release review](../../docs/RELEASE_REVIEW.md) for the coordinated fix and
verification work still required.
