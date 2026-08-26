# ARM firmware

This directory contains the ESP32 Arm HAT firmware and its host-side tests:

- `esp32/arm_hat_controller/` — the ESP32 Arm HAT sketch;
- `libraries/ArmHatController/` — bounded servo protocol and safety runtime;
- `tests/` — Arm HAT C++ host tests, Arduino stubs, and sanitized fixtures;
- `ARM_HAT_CONTROLLER_V1.md` — the current host/HAT protocol contract.

Generated BIN, ELF, MAP, object, and factory-backup files are not part of the
source release. The compile helper uses a caller-supplied system Arduino CLI,
requires the pinned ESP32 core, writes to an explicit build directory, and
never uploads firmware:

```powershell
./operations/scripts/compile-firmware.ps1 `
  -ArduinoCli <path-to-arduino-cli> `
  -BuildRoot <temporary-build-directory>
```

Run the source/host tests from `software/`:

```powershell
$env:PYTHONPATH = "python"
python -m pytest -q firmware/tests/test_arm_hat_controller_v1.py
```

A passing build or host test is software evidence. It does not prove a flash,
controller identity, servo-bus health, torque state, or physical motion.
