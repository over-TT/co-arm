# Schematics

Put editable schematic sources and readable PDF/SVG exports here. Use names
such as:

```text
co-arm_electronics_power-distribution_r01.<source-extension>
co-arm_electronics_power-distribution_r01.pdf
co-arm_electronics_system-interfaces_r01.svg
```

A release schematic must show:

- Raspberry Pi, HAT/ESP32, camera, main-servo rail, and camera-servo rail;
- supply and regulator input/output ratings;
- common/reference grounds and isolation, if any;
- fuse/protection and hardware disconnect/E-stop boundaries;
- connector reference designators, pin numbers, signals, voltages, and expected
  current bounds;
- revision, units, source application, and verification state.

Do not publish a guessed connector pinout. Mark unknown fusing, polarity,
regulator settings, and supply margins as `TODO (owner verification)`.
