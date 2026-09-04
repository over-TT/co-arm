# Known limitations

- co-arm is still a working prototype.
- It positions a camera. No gripper or tool actuator is installed.
- All 12 current printed parts are here, but the bearing, fastener, print,
  wiring, and power details are not finished yet.
- With torque off or power removed, the Shoulder and Elbow can sag.
- The desk/floor check is a 2D software check, not full 3D collision sensing.
- The current dimensions and calibration come from the reference arm; another
  build will need its own measurements and zero positions.
- Camera autofocus and wide framing still need a close look when small details
  matter.
- On a real arm without a calibrated camera/mount/desk projection, pixels are
  framing evidence. They do not give reliable desk coordinates or a precise
  contact point. The kinematic camera tip is not a calibrated contact surface.
- Isaac Sim, Arduino/ESP32 tools, and device drivers are separate installs.
- The no-hardware quickstart opens the dashboard; it does not start Isaac or
  provide simulated joint readings or camera images.
- The Python wheel and dashboard build are separate artifacts. The quickstart
  uses an editable checkout so the backend can find the local frontend build.
- The setup and deployment examples are Windows/PowerShell-first. Other host
  environments need their own verification.
- Portable Pi recovery is incomplete and has no validated backup-to-restore
  round trip. Metadata/provenance gaps and missing restored-Base invalidation
  block physical use of the exported restore script. See the
  [release review](RELEASE_REVIEW.md) before planning recovery.
- The dashboard uses the external Codex tools; it does not contain Codex by
  itself.
- No public license has been selected yet.

See [Status and evidence](STATUS.md) for dated software and reference-arm
results, the [release review](RELEASE_REVIEW.md) for unresolved findings, and the
[release checklist](RELEASE_CHECKLIST.md) for publication work.
