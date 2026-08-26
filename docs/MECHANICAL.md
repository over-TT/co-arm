# Mechanical design

co-arm is a four-joint printed camera arm. The Base, gears, links, servo mounts,
covers, and camera holder are all part of the build.

## Joint layout

| Joint | What it moves | Servo ID |
| --- | --- | ---: |
| Base | Turns the whole arm | 1 |
| Shoulder | Lower arm link | 2 |
| Elbow | Upper arm link | 3 |
| Camera | Camera angle | 4 |

The Camera joint only aims the sensor. Shoulder and Elbow place it, and Base
turns the whole mechanism.

## Printed Base

The main Base print has the 52-tooth gear built into it. A separate 13-tooth
gear fits the Base servo, giving the arm its 4:1 reduction.

From bottom to top, the Base stack is:

1. Base with the large gear;
2. bearing-bottom piece;
3. large bearing;
4. Shaft Base;
5. Level 2 platform with the lower-arm servo and mount.

## Arm and camera

The lower link runs from the lower-arm servo to the next joint. The upper link
runs from there to the camera end. The top-servo cover closes the upper servo
area.

At the end of the upper link, the camera-servo mount holds the camera servo. The
camera holder attaches to that servo, and the camera cover closes the holder.

All 12 files are listed in
[`hardware/3mf/README.md`](../hardware/3mf/README.md).

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

## Joint zero positions

- Base zero lines up with the physical mark on the Base.
- Shoulder zero points the lower link straight up.
- Elbow zero is the folded-middle position used by the current arm model.
- Camera zero looks along the upper link.

With torque off, Shoulder and Elbow can sag. Hold the arm or rest it on
something before releasing torque.

## Still to add

- large Base bearing and remaining shaft/horn details;
- heat-set insert and M3 screw list;
- material and print settings;
- final joint limits, cable clearance, weight, and tested payload.
