# CAD and print-file standard

Use this convention for every mechanical file added to co-arm. Its purpose is
to keep native design intent, neutral exchange files, printable meshes, and
manufacturing drawings synchronized without relying on filenames such as
`final-final-v2`.

## Where files go

| Content | Repository folder | What belongs there |
| --- | --- | --- |
| Editable native CAD | `hardware/cad/native/` | Original parametric parts and assemblies in the authoring application's format |
| Neutral solid CAD | `hardware/cad/step/` | Revision-matched `.step`/`.stp` exports |
| Current 3MF parts | `hardware/3mf/` | The printed parts used by the current arm |
| General STL meshes | `hardware/stl/` | Revision-matched individual printable-part meshes in millimetres |
| Approved print-ready STL | `hardware/stl/print-ready/` | Manifold, checked, intentionally oriented STL selected for the documented print process |
| Dimensioned drawings | `hardware/drawings/` | PDF plus editable/source drawing where available |
| Electrical schematics | `hardware/electronics/schematics/` | Source and PDF/SVG exports of circuit/power intent |
| Physical wiring | `hardware/electronics/wiring/` | Point-to-point harness drawings, connector views, and wiring tables |

Do not place deployable firmware or application code in `hardware/`. Do not put
machine-specific G-code in `print-ready/`; G-code depends on printer, firmware,
material, nozzle, and slicer state and can outlive the assumptions that made it
safe.

## Units and geometry

- Master mechanical unit: **millimetre (mm)**.
- Drawing angles: **degrees (deg)**.
- CAD origin: centre of Base yaw at the work-surface reference plane unless a
  part drawing defines a local manufacturing datum.
- Assembly axes: `+Z` up, `+X` forward at Base zero, `+Y` right-handed.
- Mesh unit assumption: STL carries no reliable unit metadata, so every STL is
  exported numerically in millimetres and its README/drawing must say so.
- 3MF files must declare millimetres in their model metadata. Still check their
  bounding boxes before printing.
- Do not rescale STL on import. A 20 mm calibration cube should import as 20 mm.

The reference assembly should reproduce the recorded kinematic dimensions:

- base pivot height: 60 mm;
- shoulder-to-elbow: 180 mm;
- elbow-to-tip: 220 mm, made from 180 mm forearm plus 40 mm camera/tool offset;
- Base external ratio: 4 motor turns to 1 output turn.

## Naming convention

Use lowercase ASCII, hyphens between words, and underscores between structured
fields:

```text
co-arm_<subsystem>_<part-name>[_<variant>]_rNN.<ext>
```

Rules:

- project prefix is always `co-arm`;
- subsystem is one of `base`, `shoulder`, `elbow`, `camera`, `electronics`,
  `cable`, `tool`, or `assembly`;
- part name describes function, not color or an informal nickname;
- optional variant is used only for intentional alternatives such as
  `left`, `right`, or a documented hardware variant;
- revision is two digits beginning at `r01`;
- extension is lowercase;
- native, STEP, 3MF, STL, and drawing exports of the same geometry use the same
  base name and revision.

Examples:

```text
co-arm_base_motor-mount_r01.f3d
co-arm_base_motor-mount_r01.step
co-arm_base_motor-mount_r01.stl
co-arm_base_motor-mount_r01.pdf
co-arm_camera_bracket_left_r02.step
co-arm_assembly_full-arm_r01.step
```

If the native application requires multiple linked files, keep the package in a
same-named directory, for example:

```text
hardware/cad/native/co-arm_assembly_full-arm_r01/
```

Do not encode dates, `latest`, `new`, or `final` in released filenames. Git
records history; `rNN` records compatibility between exported artifacts.

## Revision rules

Increment `rNN` when a change can affect fit, strength, assembly, motion,
wiring clearance, manufacturing, or the resulting mesh. Examples include a
hole-size change, changed insert, moved datum, altered fillet, new print
orientation that changes required geometry, or modified cable clearance.

A metadata-only correction may keep the same part revision when geometry is
unchanged, but the commit message should say so. Never replace a released STL
with different geometry under the same filename.

For each geometry revision:

1. update native CAD;
2. regenerate STEP;
3. regenerate STL if printable;
4. regenerate the dimensioned drawing;
5. update the BOM/assembly revision references;
6. verify the exports by reopening them in a different viewer;
7. commit the matched set together.

## Native CAD requirements

- Preserve editable sketches, features, constraints, component origins, joints,
  and named datums.
- Name components and bodies; remove hidden experiments and unrelated meshes.
- Represent purchased components as reference geometry and identify their
  source/model in the BOM. Do not claim authorship or apply a license that has
  not been established.
- Keep the four actuator axes and the tool/camera reference point explicit.
- Include a top-level assembly with zero-position mates and joint ranges only
  after those ranges are physically confirmed.
- Run the authoring application's health/rebuild check before export.

## STEP export requirements

- Prefer STEP AP242; AP214 is acceptable if the authoring tool cannot produce a
  reliable AP242 export.
- Export solids, not only surface shells.
- Preserve assembly/component names when supported.
- Use millimetres and the canonical coordinate frame.
- Exclude construction sketches and irrelevant hidden prototypes.
- Reopen the exported STEP and check body count, orientation, scale, missing
  faces, and assembly placement.
- State the actual STEP application protocol in the commit or folder README if
  it differs from AP242.

## STL export requirements

- One printable manufactured part per STL unless a multi-part print is
  deliberately designed and documented.
- Export as binary STL in millimetre scale.
- Use a chordal/linear tolerance of **0.10 mm or finer** and an angular tolerance
  of **1 degree or finer** as an initial export target. Record the actual export
  settings; tighten them for small gears or curved fits if faceting affects
  function.
- Mesh must be manifold/watertight, with outward normals, no self-intersections,
  no duplicate shells, and no zero-area triangles.
- Confirm the imported bounding box against the part drawing.
- Do not include supports, rafts, brims, or machine-specific G-code in the STL.

`hardware/stl/` stores the canonical geometry export. Copy a checked artifact
to `hardware/stl/print-ready/` only when its intended bed orientation and print
profile are documented. If reorientation alone changes the file bytes, keep
the same geometry revision but add a clear `_print` variant and record that the
solid geometry is unchanged.

## 3MF parts

Use clear part names, check the units and part count, and add the part to the
short list in [`hardware/3mf/README.md`](../hardware/3mf/README.md). All 12
current arm parts are indexed there.

## Print-ready release record

For every file promoted to `print-ready/`, document:

- source CAD/STL revision and checksum if a release process adds one;
- intended bed face and orientation screenshot;
- printer and nozzle diameter;
- material brand/type;
- layer height, wall/perimeter count, top/bottom layers;
- infill type and percentage;
- support type, interface, and critical blocked/enforced regions;
- dimensional compensation, inserts, drilling/reaming, and post-processing;
- quantity and mirrored copies;
- observed fit result and which physical build tested it.

Those print-process values are still **TODO (owner verification)** for the
current 3MF set. Until they are filled, a 3MF or STL is a printable geometry
candidate, not a verified print profile.

## Drawing requirements

Each released part drawing should include:

- exact matching part name and `rNN` revision;
- units, projection method, sheet scale, and page count;
- material and manufacturing process, when confirmed;
- overall dimensions and functional datums;
- hole/thread/insert specifications;
- tolerances and critical fit dimensions;
- orientation and quantity;
- mass if meaningful and confirmed;
- drawing author/date fields without personal information that should remain
  private;
- a note that CAD governs only if that policy is intentionally adopted.

Assembly drawings should additionally include an exploded view, BOM balloons,
fastener stack order, cable routing, zero-position views, and gear timing/mesh.

## Export checklist

- [ ] Physical part/assembly identity confirmed.
- [ ] Exact revision chosen; no `final` naming.
- [ ] Native CAD rebuilt without errors.
- [ ] Millimetres and canonical axes verified.
- [ ] Reference dimensions are 60/180/220 mm where applicable.
- [ ] Base drive represents 4:1 reduction where applicable.
- [ ] STEP reopened and checked.
- [ ] 3MF units, bounding box, part count, and original filename recorded.
- [ ] STL mesh and bounding box checked.
- [ ] Drawing regenerated from the same revision.
- [ ] Print process fields completed or explicitly marked TODO.
- [ ] BOM and assembly references updated.
- [ ] No credentials, private paths, personal metadata, or unwanted serial
      numbers included.
- [ ] Ownership/attribution reviewed; no license invented.
