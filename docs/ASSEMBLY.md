# Assembly

This is the current build order for co-arm. The 12 printed parts are in
[`hardware/3mf/`](../hardware/3mf/).

## What goes into the build

- Raspberry Pi 4 and Camera Module 3 Wide;
- Waveshare Bus Servo Driver HAT (A);
- three ST3215/STS-family servos and one SC09/SCS-family camera servo;
- the 12 printed parts;
- one large Base bearing;
- heat-set inserts, M3 screws, servo horns, washers, spacers, and cable ties.

The exact bearing details, insert list, screw lengths, and print settings are
still being added.

## 1. Build the Base

1. Start with `co-arm_base_main-body_r01.3mf`. The large Base gear is
   already part of this print.
2. Put `co-arm_base_bearing-bottom_r01.3mf` underneath the large bearing.
3. Fit the bearing into the Base.
4. Put `co-arm_base_shaft-base_r01.3mf` on top of the bearing.
5. Fit the lower-arm servo and
   `co-arm_shoulder_servo-mount_r01.3mf` before bolting
   `co-arm_base_level-2-platform_r01.3mf` to the Shaft Base.
6. Fit `co-arm_base_servo-gear_r01.3mf` to the Base servo and mesh it with the
   large gear. Turn the Base by hand and make sure it moves without a tight
   spot.

The two printed gears make the 4:1 Base reduction. They are part of the arm,
not temporary test pieces.

## 2. Build the arm

1. Attach `co-arm_shoulder_lower-link_r01.3mf` to the lower-arm servo.
2. Fit the next servo at the top of the lower link.
3. Attach `co-arm_elbow_upper-link_r01.3mf`.
4. Close the top-servo area with
   `co-arm_elbow_servo-cover_r01.3mf`.

With torque off, the Shoulder and Elbow can sag. Hold the link or rest it on
something before releasing torque.

## 3. Build the camera end

1. Mount `co-arm_camera_servo-mount_r01.3mf` at the end of the upper link.
2. Put the camera servo inside it.
3. Attach `co-arm_camera_holder_r01.3mf` to the servo.
4. Put the Camera Module 3 Wide in the holder.
5. Close it with `co-arm_camera_cover_r01.3mf`.

## 4. Wire it

The basic chain is:

```text
Camera -> Raspberry Pi
Raspberry Pi -> ESP32 Arm HAT
Arm HAT -> four serial servos
```

Use servo IDs `1 / 2 / 3 / 4` for Base, Shoulder, Elbow, and Camera. Leave
enough cable slack for every joint to move without pulling or entering the Base
gears. The final wiring diagram and power details are still being added; see
[`ELECTRONICS.md`](ELECTRONICS.md) for the current notes.

## 5. First setup

1. Move every joint by hand and check the gears, cables, and hard stops.
2. Mark the physical Base zero.
3. Power and test one servo at a time.
4. Check its ID, direction, and a small move before moving on.
5. Test Base, Shoulder, Elbow, then Camera.
6. Only then try a slow coordinated move.

Continue with [`COMMISSIONING.md`](COMMISSIONING.md) for the software setup and
joint calibration.

## Still to add

- large bearing details;
- heat-set insert and M3 screw list;
- print settings;
- final wiring and power diagram;
- step-by-step assembly photos.
