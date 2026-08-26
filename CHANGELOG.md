# Changelog

All notable public repository changes will be recorded here.

## Unreleased

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
- Retained the explicit no-license warning. Public reuse terms remain pending
  an owner license decision.
