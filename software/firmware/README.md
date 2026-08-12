# Arm HAT firmware source — pending

This folder is reserved for:

- the ESP32 Arm HAT sketch;
- the complete `ArmHatController` library;
- a current protocol document generated/reviewed against firmware 2.4;
- arm-only host tests and Arduino stubs;
- a reproducible compile script using a system Arduino CLI and pinned ESP32
  core;
- source-level third-party notices.

Do not add generated BIN/ELF/MAP/object files or the device's factory firmware
backup. Do not include legacy raw-register diagnostic scripts that can alter a
servo or move it outside the typed gateway workflow.

Firmware source will be added only after the owner confirms software licensing
and attribution. A build will remain software proof; flashing and motion need
separate controlled hardware evidence.
