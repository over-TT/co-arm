# Release review — 2026-09-04

The first release should describe a working four-axis camera-arm prototype,
with source and printable parts. It is not yet a complete copy-and-build kit.
Do not publish the current recovery tools as a supported physical recovery path.

This review covered the exported gateway/backend, dashboard, MCP boundary,
simulator contracts, firmware host tests, installation, packaging, operations,
documentation, and release checks. It is a focused engineering review, not a
formal security certification or a new physical-arm acceptance test.

## Fixed in this candidate

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

## Remaining release blockers

### 1. Recovery needs a coordinated fix

The portable operations scripts do not yet form a proven backup-to-restore
round trip. Three source-level findings need to be addressed together:

1. **Restored Base calibration can be paired with the wrong valid frame.**
   Bootstrap restores the archived `rawZero`. The gateway can adopt a HAT
   frame that remained valid while the Pi was replaced/restarted, but it does
   not bind that frame to the archived calibration. If the Base was re-zeroed
   after the snapshot and the HAT remains powered, clearing the generic STOP
   latch can expose mismatched calibration as trusted. A true cold HAT/servo
   continuity loss still blocks Base; this finding is not a claim that every
   restore immediately permits motion.
2. **A verified archive may contain stale calibration provenance.** Normal
   calibration changes the calibration JSON without refreshing the older
   recovery-provenance hash. Backup can preserve both while restore rejects
   that pair. Archive integrity alone does not prove restore compatibility.
3. **Fresh installs lack required recovery metadata.** Backup requires files
   created by the restore workflow, not by ordinary initial deployment. The
   script also takes separately prepared inputs rather than an archive.

Relevant implementation: [restore script](../software/operations/scripts/restore-arm-pi.ps1),
[backup script](../software/operations/scripts/backup-arm-pi.ps1), and
[calibration/Base state](../software/python/robot_gateway/simple_arm_api.py).

Acceptance: exercise fresh deployment, calibration change, snapshot, archive
input conversion, dormant restore, retained-valid HAT frame, cold HAT frame,
explicit physical re-zero, and verified post-change snapshot. The restored
Base must remain untrusted until the new reference is confirmed. Include
tampered/stale provenance and failed-write cases. Do not fabricate metadata or
clear STOP to make a recovery check pass.

### 2. Public terms and provenance need owner decisions

Choose the licenses and attribution described in [Licensing](../LICENSING.md),
review CAD/media provenance, and approve the actual photos/clip. No approved
public photos or videos are included yet. Drafting a post is not publishing it.

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
push, require the GitHub workflow to pass for that exact commit and inspect
the rendered documentation before changing visibility or tagging a release.

## Recommended next scope

1. Close the recovery findings and add the round-trip regressions.
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

This pass prepares local source and documentation. It does not push, change
repository visibility, choose a license, upload media, tag a release, or post
to X. The [launch draft](LAUNCH_DRAFT.md) is for owner review after the gates
above are satisfied.
