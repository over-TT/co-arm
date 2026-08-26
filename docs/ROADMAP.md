# Roadmap

## R0 — public documentation scaffold (complete)

- Complete the public-safe documentation scaffold.
- Established the ARM-only repository boundary and privacy checker.
- Run privacy, link, size, and required-file checks.
- Push the first private review commit.

## R1 — reproducible hardware package

- Added all 12 current printable parts and a concrete Base-to-camera assembly
  map.
- Recorded the printed 52:13 Base gears and 4:1 reduction.
- Named the lower and upper links and recorded the bearing-bottom position.
- Next: add the large bearing details, heat-set inserts, and M3 screw schedule.
- Add verified BOM and labeled hardware photographs.
- Add editable CAD, STEP, revision-matched STL, drawings, and print settings.
- Add a reviewed wiring diagram, power tree, fuse, and physical power-cut
  documentation.
- Record assembly and commissioning photographs.

## R2 — clean ARM software source export (source-complete; publication gates remain)

- Exported firmware, Pi gateway, MCP/plugin, simulator, operations helpers,
  ARM-only FastAPI backend, and standalone dashboard.
- Replaced installation-specific plugin/backend configuration with portable
  local inputs.
- Added locked dependency graphs, vulnerability gates, package resources,
  source-isolated wheel smoke, focused tests, clean-checkout CI, and Dependabot.
- Made the Codex tools identify SIM versus the real arm and read current state
  instead of blindly repeating a move whose reply was lost.
- Remaining publication work: choose licenses, finish the third-party license
  review, run CI for the exact pushed commit, and complete the owner launch
  assets.

## R3 — final-path physical qualification

- Record numeric signed Base arrivals, seams, endpoints, repeated targets,
  mid-flight retargets, and re-home after continuity loss.
- Record a longer coordinated four-joint run.
- Measure physical floor clearance against the corrected model.
- Exercise plan/apply and record measured arrival.
- Characterize servo power, sag, temperature, and current behavior.

## R4 — repeatable active-vision demo

- Add one clean three-quarter hero photograph and one dashboard plan-preview
  screenshot.
- Record a 10-20 second `state -> preview -> move -> joint readback -> camera
  result` clip.
- Capture a proven wide survey, retained-source crop, and centered close pass.
- Define image-based success criteria for framing and visible detail.
- Rehearse repeatable object localization without claiming exact identity until
  close evidence supports it.
- Add a reviewed social-preview image, then publish the media with raw/private
  originals kept out of Git history.
