# Release review — engineering 2026-09-05; publication scope 2026-09-13

The first release should describe a working four-axis camera-arm prototype,
with source and printable parts. It is not yet a complete copy-and-build kit.
Do not publish the current recovery tools as a supported physical recovery path.

On 2026-09-13 the owner authorized publishing this prototype source with
PolyForm Noncommercial 1.0.0 software and CC BY-NC 4.0 designs/documentation/media.
The exact single-bearing specification, full parts schedule, and approved demo
are follow-up milestones. The outstanding device tests below remain required
before claiming supported recovery or exact-stack physical acceptance.

This review covered the exported gateway/backend, dashboard, MCP boundary,
simulator contracts, firmware host tests, installation, packaging, operations,
documentation, and release checks. It is a focused engineering review, not a
formal security certification or a new physical-arm acceptance test.

## Fixed in this candidate

- Live Follow recovers correctly from React StrictMode's development effect
  replay, which previously left it waiting permanently for telemetry. A
  lifecycle regression covers telemetry, start, and end. Physical Live behavior
  still needs a fresh test on the exact installed stack.
- Recovery now has a separate Base-reference gate, dashboard/MCP reporting,
  explicit zero confirmation, durable calibration readback, and final-dispatch
  checks. Disk failures and a gate appearing during hold cannot unlock motion.
- Fresh backups permit absent historical recovery metadata and record whether
  older provenance matches the frozen calibration. Portable Prepare verifies a
  protected archive and creates archive-bound provenance plus the Base gate;
  it does not assert physical continuity or silently trust an older hash.
- Camera arrival and goal pursuit now use the reviewed 6-degree / 17-tick
  tolerance. Base, Shoulder, and Elbow retain their 1-degree checks; the
  shutter-time pose-drift check also stays at 1 degree.
- The dashboard allows 9 seconds for a quiet Camera target to settle. It
  rejects invalid requested angles, ignores an unrequested nullable Base,
  and keeps the real destination when a partial floor route is returned.
- The gateway exposes a process-stable response identity. The backend checks
  restart identity across reviewed requests, rejects malformed negotiated
  identity, and keeps camera evidence bound to the response that produced it.
- The Python test extra declares NumPy and headless OpenCV for the simulator
  pixel/JPEG policy tests. A fresh environment previously failed collection.
- The source manifest now matches the LF bytes in a Git archive. The release
  checker rejects CRLF text so Windows line endings cannot silently make a
  locally passing manifest fail in a fresh checkout.
- The [quickstart](QUICKSTART.md) is separate from the full setup runbook.
  Reconnecting an existing arm no longer reads like a full rebuild.

See [Status](STATUS.md) for the exact verification results and their limits.

## Remaining acceptance and demo work

### 1. Recovery still needs device round-trip acceptance

The following source-level findings are addressed by local regressions. They
are not a claim that a replacement Pi has been restored successfully:

1. **Restored Base calibration can be paired with the wrong valid frame.**
   Bootstrap restores the archived `rawZero`. The gateway can adopt a HAT
   frame that remained valid while the Pi was replaced/restarted, but it does
   not bind that frame to the archived calibration. If the Base was re-zeroed
   after the snapshot and the HAT remains powered, clearing the generic STOP
   latch can expose mismatched calibration as trusted. A true cold HAT/servo
   continuity loss still blocks Base; this finding is not a claim that every
   restore immediately permits motion. The new independent gate hides Base
   truth and blocks motion until explicit re-zero and durable readback succeed.
2. **A verified archive may contain stale calibration provenance.** Normal
   calibration changes the calibration JSON without refreshing the older
   recovery-provenance hash. Backup can preserve both while restore rejects
   that pair. Prepare now derives fresh provenance from the verified archive's
   actual frozen calibration; any separately supplied pair must match exactly.
3. **Fresh installs lack required recovery metadata.** Backup requires files
   created by the restore workflow, not by ordinary initial deployment. The
   script also took separately prepared inputs rather than an archive. Old
   metadata is now optional; protected archive input is required for restore.

Relevant implementation: [restore script](../software/operations/scripts/restore-arm-pi.ps1),
[backup script](../software/operations/scripts/backup-arm-pi.ps1), and
[calibration/Base state](../software/python/robot_gateway/simple_arm_api.py).

Remaining device acceptance: exercise fresh deployment, calibration change, snapshot, archive
input conversion, dormant restore, retained-valid HAT frame, cold HAT frame,
explicit physical re-zero, and verified post-change snapshot. The restored
Base must remain untrusted until the new reference is confirmed. Include
tampered/stale provenance and failed-write cases. Do not fabricate metadata or
clear STOP to make a recovery check pass.

### 2. Public terms selected; media and detailed provenance remain

The owner selected the noncommercial license split and existing over-TT
attribution in [Licensing](../LICENSING.md). The owner-supplied printable parts
are included under that scope; third-party rights remain unchanged. Record
any adapted-material provenance and approve photos/clip before adding them.
No approved public photos or videos are included yet.

### 3. New builders still need physical facts

The 12 printable parts are present. A reproducible build still needs the
bearing specification, inserts and screw lengths/counts, shaft/horn details,
tested materials/print settings, and as-built wiring and power/protection
details. Use the [open questions](OPEN_QUESTIONS.md) and
[BOM](../hardware/bom/bom.csv); do not guess these from source code.

### 4. The exact released stack needs its own evidence

Local tests do not prove an installed Pi, flashed controller, running Isaac
scene, or physical arrival. Validate the exact candidate on the reference arm
with a bounded supervised commissioning/demo run. After an explicitly approved
push, check the GitHub workflow for that exact commit and inspect the rendered
documentation. The authorized public prototype snapshot does not claim that
the unfinished physical acceptance work passed; a release tag is a separate step.

## Recommended next scope

1. Complete the physical recovery round trip using the tested source candidate.
2. Make one inspection demo repeatable: request a viewpoint, preview, measured
   arrival, and a camera image that visibly answers the request. Record several
   attempts and report the actual successes/failures instead of a broad
   reliability percentage.
3. Capture a short real clip and one clean overview photo. Put the clip and
   a one-sentence description near the top of the README after approval.
4. Finish the missing assembly/BOM/wiring facts, then ask a new reader to follow
   the quickstart and build instructions without oral corrections.

The local disconnected dashboard opens correctly and keeps Control/Live
disabled. Its next useful usability improvement is an inline connection/setup
entry point in that empty state; users currently have to leave the interface
for the runbook. Keep new features out of this release until setup, recovery,
and one good demo are dependable.

## Publication boundary

The owner authorized the noncommercial licenses, Git push, and public
repository visibility on 2026-09-13. That authorization covers the documented
prototype source snapshot. It does not claim completed hardware acceptance,
approved demo media, a tagged release, or a social post. The
[launch draft](LAUNCH_DRAFT.md) remains a draft for the later demo.
