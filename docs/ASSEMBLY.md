# Assembly and commissioning record

This is a documentation-first assembly workflow for co-arm. It is not yet a
validated step-by-step reproduction manual because the native CAD, complete
BOM, fastener schedule, wiring drawings, print settings, and build photographs
have not been supplied. Preserve the order below while replacing every marked
gap with as-built evidence.

## Before disassembly or rebuilding

Capture the existing arm first:

- overall photos from front, rear, left, right, top, and a 45-degree view;
- close photos of every servo label, controller label, regulator, power supply,
  connector, gear mesh, bearing/shaft retention, and base anchoring;
- a slow video around the unpowered arm showing cable routing;
- photos of every joint at its physical zero mark;
- a ruler/caliper reference in dimension photos where practical.

Do not publish private network labels, account details, keys, QR codes, home
interiors beyond the intended crop, or serial numbers you do not want public.

## Tools and consumables

The exact tool list is not yet known.

- **TODO (owner verification):** driver/hex sizes, spanners, torque tools,
  calipers, thread-locking compound, lubricant, cable ties, heat-shrink, and
  electrical test equipment actually used.
- **TODO (owner verification):** note where thread locker must or must not be
  used, especially near plastic, bearings, and serviceable joints.

## Provisional mechanical assembly order

Keep all actuators unpowered and support each link during assembly.

1. **Prepare printed and purchased parts.** Check part revision, remove support
   material, inspect holes and layer lines, and test inserts without forcing
   them. Record any drilling, reaming, tapping, or heat-set operation.
2. **Build and anchor the Base.** Install the output support, bearings/bushings,
   shaft/hub, driven gear, motor pinion, and motor mount according to the future
   assembly drawing. Confirm the recorded 4:1 ratio from tooth counts before
   closing the housing.
3. **Set Base gear mesh.** It should rotate through the intended mechanical
   range without binding. Record backlash and axial retention. Do not enable the
   motor to overcome a tight mesh.
4. **Install the Shoulder.** Attach the shoulder actuator and upper link while
   the link is supported. Establish the shoulder zero-reference planes before
   tightening the output horn/hub.
5. **Install the Elbow.** Attach the distal link and establish the elbow
   zero-reference posture. Confirm 180 mm shoulder-to-elbow and 220 mm
   elbow-to-tip model dimensions using pivot centres, not case edges.
6. **Install the camera axis and module.** Mount the camera servo and bracket,
   then the OV5647 module. The current camera is physically inverted and the
   software rotates images 180 degrees; document the lens-facing direction in a
   drawing.
7. **Route cables.** Leave a service loop at every moving joint. Move the arm by
   hand through its intended range and verify no cable becomes taut, pinched,
   scraped, or able to enter a gear.
8. **Mount the Raspberry Pi and HAT.** Preserve access to connectors, airflow,
   status indicators, storage, and the servo-bus/service connections.
9. **Apply witness marks and labels.** Label Base/Shoulder/Elbow/Camera, servo
   IDs 1-4, cable ends, power domains, polarity, and mechanical zero marks.

Exact fasteners, tightening torques, fit classes, gear clearances, and part
orientation are **TODO (owner verification)** and must be added to drawings
before this becomes a repeatable assembly procedure.

## Electrical assembly order

1. Mechanically support the arm so loss of torque cannot drop a link.
2. Leave all external power disconnected.
3. Verify the Raspberry Pi and HAT orientation and all jumper/switch positions
   against the exact board revision.
4. Trace the power distribution from source to each load and complete the
   schematic before energizing it.
5. Verify connector pinout and polarity from the mating-face view and again at
   the wire-side harness.
6. Verify any step-down/regulator output without a servo connected.
7. Connect and identify one previously uncommissioned bus servo at a time.
   Prevent duplicate bus IDs; reserve 1/2/3/4 for Base/Shoulder/Elbow/Camera.
8. Reassemble the shared bus only after each servo is labeled with its confirmed
   ID and voltage.
9. Check continuity, shorts, ground references, cable strain relief, and fuse or
   protection placement.
10. First-power each domain independently with an appropriate current limit and
    no commanded motion.

The candidate 12 V, 7.5 A supply is not approved until its exact model,
polarity, harness ratings, fusing, regulator path, and measured margins are
recorded. Do not assume the camera servo accepts 12 V.

## First mechanical checks after wiring

Before enabling torque:

- base anchored and work area clear;
- links supported against gravity;
- all fasteners present and witness-marked where useful;
- gears, horns, shafts, and bearings retained;
- cables clear through the full hand-moved range;
- correct logical ID returned for each isolated actuator;
- controller communications healthy;
- supply voltage stable and polarity correct;
- hardware disconnect/E-stop behavior known;
- software STOP clear only when intentionally beginning commissioning.

## Commission one axis at a time

1. Start with the arm mechanically supported and the other axes unable to make
   an unexpected movement.
2. Read the live servo identity/position before commanding anything.
3. Verify the physical zero mark and direction using a deliberately small,
   bounded move.
4. Verify measured arrival; a requested target is not proof that the joint
   moved there.
5. Record comfortable software limits inside physical hard stops and cable
   limits.
6. Remove torque and confirm the link's gravity behavior before moving to the
   next axis.
7. Repeat for Base, Shoulder, Elbow, then Camera.
8. Only after the four individual axes are documented should coordinated motion
   be tested at low speed with clearance monitoring.

The Base's 4:1 reduction makes output position continuity especially important.
After a true continuity loss, align the physical Base-zero mark and deliberately
re-establish zero; do not infer output turns from one single-turn reading.

## Assembly evidence to add later

Place media in the repository's media folders and use descriptive captions. A
useful minimum set is:

- `media/photos/overview/` — completed arm and scale/context views;
- `media/photos/details/` — labels, gears, joints, controller, power, and
  connectors;
- `media/photos/assembly/` — one photo per meaningful assembly step;
- `media/video/` — short mechanism, cable-routing, and controlled-motion clips.

For each assembly photo, note the matching part revision and step. Avoid
embedding huge original videos in Git history without deciding on repository
size policy; a hosted link plus a small poster/preview may be preferable.

## Completion gate for a reproducible build

The assembly guide is not complete until all of the following exist:

- exact BOM with supplier/manufacturer identifiers;
- released native CAD, STEP, STL, and dimensioned drawings;
- fastener and purchased-hardware schedule;
- confirmed material and print profile per printed part;
- complete power schematic and point-to-point wiring/harness drawing;
- confirmed supply margins, fusing/protection, and emergency isolation;
- labelled assembly photos and a revision-matched build sequence;
- axis-by-axis zero, direction, limits, and first-motion checks;
- a final as-built verification against the released revision.
