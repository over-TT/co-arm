# Changelog

All notable public repository changes will be recorded here.

## Unreleased

- Selected PolyForm Noncommercial 1.0.0 for software and firmware, and
  CC BY-NC 4.0 for designs, documentation, and media. Added full license texts,
  attribution, contribution terms, and Python/dashboard/firmware metadata.
- Prepared the public prototype source release; the full purchased-parts
  schedule and demo media remain follow-up work.
- Fixed Live startup after React StrictMode replay and added locally tested
  recovery Base-reference gates, archive preparation, provenance handling,
  and Windows recovery regressions. Exact-stack physical acceptance remains pending.
- Fixed source-manifest drift between Windows files and committed Git exports,
  and added an LF line-ending release gate with regressions.
- Ported Camera arrival/pursuit tuning, strict dashboard target handling,
  process-stable gateway identity, and response-bound camera provenance with
  boundary and restart-race regressions.
- Added the missing NumPy/headless OpenCV test dependencies so a fresh Python
  environment can collect the simulator camera-policy tests.
- Added a short no-hardware quickstart, existing-arm setup routing, a dated
  release review, and draft release/X copy.
- Documented the remaining portable recovery device-acceptance work after the
  Base-reference, provenance, and fresh-install metadata fixes.
- Rewrote the README around the physical build and why it exists, using the
  same direct voice as Rock and DOT instead of leading with software policy.
- Added all 12 current 3MF parts: both printed Base gears, bearing-bottom piece,
  Shaft Base, Level 2 platform, servo mount/covers, lower and upper links, and
  the camera mount/holder/cover.
- Added the current physical assembly map, one-row-per-part BOM entries, and a
  short list of the bearing, inserts, screws, and print details still needed.
- Made Codex verify whether it is connected to the simulator or the real arm,
  return plain structured results, and avoid repeating a physical write after
  a lost response.
- Made the source index root unambiguous and extended the repository checker to
  cover CI, Dependabot, locked dependencies, and installed-wheel smoke.
- Updated the Python and Pi dependency sets to current patched compatible
  versions, forced the patched dashboard `nanoid`, and added Python/pnpm audit
  gates to CI.
- Packaged the simulator URDF/JSON in the Python wheel, excluded test suites,
  and added a source-isolated installed-wheel smoke with honest dashboard-asset
  failure behavior.
- Corrected the Arm HAT 2.7.2 goal-readback documentation to four attempts with
  5 ms gaps while preserving the distinct compact-feedback retry contract.
- Added clean-checkout CI and weekly Dependabot coverage for Python, Pi
  requirements, pnpm, and GitHub Actions.
- Promoted the repository from a documentation scaffold to an ARM-only source
  export.
- Added the standalone React/Vite Arm dashboard and loopback FastAPI runtime.
- Added the Raspberry Pi gateway, Arm MCP server, core Isaac Sim stack,
  portable Codex plugin, deployment helpers, and focused tests.
- Added Arm HAT `2.7.2`, shared controller libraries, and host-side protocol
  tests.
- Updated the public reference build to Raspberry Pi Camera Module 3 Wide /
  IMX708 and documented the survey/detail autofocus evidence contract.
- Documented bounded Shoulder/Elbow Live Follow without treating raw servo
  settings as measured speed.
- Removed unrelated parent-workspace product/evaluation/runtime branches and
  the superseded three-joint synthetic gateway so the release stays focused on
  the arm, Pi, Arm HAT, Isaac Sim, dashboard, and external agent workflow.
- Reworked the source boundary and privacy checks to exclude credentials,
  runtime state, captures, reports, backups, generated artifacts, and unrelated
  parent-application code.
- Added a canonical source index and deterministic per-file SHA-256 manifest
  with a stale-manifest verification command.
- Kept source/build evidence distinct from previously recorded reference-arm
  evidence; no live hardware verification was performed for this export.
- Replaced the earlier no-license status with explicit noncommercial terms.
