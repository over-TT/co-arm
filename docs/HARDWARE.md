# Hardware

This is the hardware used by the current co-arm build.

## Main parts

| Part | Current build | Notes |
| --- | --- | --- |
| Main computer | Raspberry Pi 4 | Runs the camera and arm gateway |
| Servo controller | Waveshare Bus Servo Driver HAT (A) with ESP32 | Connects the Pi to the servo bus |
| Base, Shoulder, Elbow | 3 x ST3215/STS-family serial servos | Exact label suffixes still need photos |
| Camera joint | SC09/SCS-family serial servo | Exact label still needs a photo |
| Camera | Raspberry Pi Camera Module 3 Wide / IMX708 | Mounted upside down; corrected in software |
| Printed mechanism | 12 current parts | Both Base gears, Base stack, links, mounts, and camera pieces |
| Servo power | 12 V, 7.5 A supply on the reference arm | Final power and wiring details are still being added |

The full parts list is in [`hardware/bom/bom.csv`](../hardware/bom/bom.csv).

## Four moving joints

1. **Base** turns the whole arm around the desk.
2. **Shoulder** moves the lower link.
3. **Elbow** moves the upper link.
4. **Camera** aims the camera without moving the rest of the arm.

The Base uses two printed gears: 52 teeth on the Base and 13 teeth on the servo.
That gives a 4:1 reduction for more output torque and finer movement. Both files
are in [`hardware/3mf/`](../hardware/3mf/), together with the other 10 printed
parts.

## Dimensions used by the software

| Measurement | Value |
| --- | ---: |
| Base pivot height | 60 mm |
| Shoulder to Elbow | 180 mm |
| Elbow to camera offset | 180 mm |
| Camera offset | 40 mm |
| Elbow to camera tip | 220 mm |

These are the dimensions used by the current arm software. I still need to
measure the final printed assembly and add the remaining hardware details.

## Connections

```text
Camera -> Raspberry Pi
Raspberry Pi -> ESP32 Arm HAT
Arm HAT -> Base / Shoulder / Elbow / Camera servos
```

The Pi talks to the HAT at 115200 baud. The HAT talks to the servos at 1 Mbps.
The servo IDs are `1 / 2 / 3 / 4` for Base, Shoulder, Elbow, and Camera.

See [`ELECTRONICS.md`](ELECTRONICS.md) for the current wiring notes and
[`ASSEMBLY.md`](ASSEMBLY.md) for the physical build order.

## Still to add

- exact servo labels and Raspberry Pi revision;
- large Base bearing details;
- heat-set insert and M3 screw list;
- print settings;
- final power and wiring diagram;
- tested joint limits and payload.
