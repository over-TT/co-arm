# Open owner questions

These are the decisions or observations that should be supplied by the project
owner. Unknown values are intentionally not guessed.

## Blocking the first public release

- [ ] Keep the first GitHub push private, or make the repository public?
- [ ] Include only documentation/CAD/media, or also publish the extracted
  firmware, Raspberry Pi gateway, MCP server, and dashboard?
- [ ] Choose the public creator/organization name, or choose no public credit.
- [ ] Choose licenses for software, hardware/CAD, documentation, and media.
- [ ] Confirm whether future CAD/STLs are entirely original or derived from
  vendor/community files, and record every source.

## Hardware identification

- [ ] Photograph the labels on all three main servos.
- [ ] Photograph the SC09 camera servo label.
- [ ] Photograph both sides and the printed revision of the servo HAT.
- [ ] Photograph the power-supply label.
- [ ] Confirm the camera board/revision label.
- [ ] Confirm the actual Base gear tooth counts or measured ratio.

## Reproducible mechanical build

- [ ] List every printed part and its revision.
- [ ] List every fastener, nut, washer, spacer, bearing, insert, and cable tie.
- [ ] Record material, nozzle, layer height, wall count, infill, support, and
  orientation for every printed part.
- [ ] Record the actual arm mass and maximum intended payload.
- [ ] Export native CAD, STEP, STL, and dimensioned drawings.

## Electrical and safety

- [ ] Confirm connector types, wire gauges, polarity, and wiring colors.
- [ ] Confirm fuse or circuit-protection details.
- [ ] Confirm whether a dedicated physical emergency power cut is installed.
- [ ] Record the normal and worst observed supply current/voltage.
- [ ] Confirm how Pi power and servo-rail power are isolated to avoid backfeed.

## Media

- [ ] Take the overview, detail, assembly, wiring, label, and motion shots from
  the [media guide](MEDIA_GUIDE.md).
- [ ] Decide whether long video uses Git LFS, GitHub Releases, or external
  hosting. The default recommendation is short compressed clips in Releases
  and linked long-form video elsewhere.
