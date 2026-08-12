# Known limitations

## Mechanical and electrical

- This is a prototype, not a certified mechanism.
- Exact fasteners, bearing types, printed materials, print settings, wire
  gauges, connector part numbers, fuse value, and physical E-stop/power-cut
  installation have not yet been documented.
- The candidate 12 V / 7.5 A supply may be undersized for simultaneous stall
  demand from three large servos. Avoid stalls; do not infer a safe current
  envelope from nominal ratings.
- Torque release, a controller restart, or a lost hold can let elevated links
  sag under gravity.
- Current/load telemetry has not been independently calibrated into certified
  protection thresholds.

## Motion and geometry

- The floor guard models a two-link side view and a horizontal keep-out plane.
  It is not full 3D collision avoidance.
- It does not identify people, cables, loose objects, self-collisions, or every
  obstacle in the sweep.
- The reference geometry and calibration belong only to the reference arm.
- The final 2.4 Base path still lacks a published endurance, seam, endpoint,
  retarget, and power-loss numeric table.
- The explicit one-use plan/apply path has been previewed but not yet recorded
  moving the physical arm.

## Vision and AI

- The OV5647 is fixed focus and has no lens actuator.
- A whole-frame focus score can favor background texture; visible subject
  detail is the stronger signal.
- A camera frame is not a personnel-safety sensor.
- A wide image can suggest an object family but should not support exact board
  identity without discriminating close detail.
- Named viewpoints are destinations, not current pose or guaranteed framing.
- AI tools operate within software contracts; they do not turn the prototype
  into an autonomous or safety-rated robot.

## Software distribution

- The dashboard and embedded Arm Chat are coupled to a larger private parent
  application and are not yet a clean standalone export.
- Licensing and public attribution are still undecided.
- A clean source release must run its own tests; parent-workspace test counts
  are historical evidence only.
