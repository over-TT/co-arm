# Owner handoff

This page is the practical drop-off list for completing the repository. The
folders already exist so files can be added without redesigning the tree.

## Where each file goes

| What you have | Put it here | Notes |
| --- | --- | --- |
| Editable/native CAD assembly and parts | `hardware/cad/native/` | Keep the main assembly and its external references together |
| Neutral CAD for reuse | `hardware/cad/step/` | Export STEP AP242 or AP214 in millimetres |
| Printable 3MF parts | `hardware/3mf/` | Use the clean part names already established there |
| Slicer-ready meshes | `hardware/stl/print-ready/` | One oriented binary STL per printable part, in millimetres |
| Dimensioned drawings | `hardware/drawings/` | PDF plus DXF when useful |
| Exact parts and quantities | `hardware/bom/bom.csv` | Replace TODO rows; do not invent supplier IDs |
| Electrical schematics | `hardware/electronics/schematics/` | Prefer editable source plus PDF/SVG export |
| Point-to-point wiring and harness drawings | `hardware/electronics/wiring/` | Include connector mating view, polarity, gauge, and labels |
| New unreviewed photos/videos | local `media/raw/` | Ignored by Git; never stage this folder |
| Approved whole-arm photos | `media/photos/overview/` | Remove metadata and personal background details |
| Approved build-step photos | `media/photos/assembly/` | Match filenames/captions to assembly steps |
| Approved label/detail photos | `media/photos/details/` | Blur serials/QR codes if they are not needed |
| Original diagrams | `media/diagrams/` | Prefer editable SVG plus PNG fallback |
| Small approved clips or video links | `media/video/` | Use Releases/external hosting for long video |

Read [`CAD_AND_STL.md`](CAD_AND_STL.md) before exporting geometry and
[`MEDIA_GUIDE.md`](MEDIA_GUIDE.md) before adding images.

## Minimum useful CAD package

All 12 current printed parts are now in `hardware/3mf/`. To finish the
mechanical package, add:

1. the editable assembly and part sources;
2. a full neutral STEP assembly;
3. individual STEP parts;
4. one print-ready STL per printed part;
5. dimensioned drawings for critical interfaces;
6. a revision-matched BOM;
7. material and print settings;
8. a short change note in `CHANGELOG.md`.

Use the same two-digit revision in every matching filename. Do not overwrite a
released revision with different geometry.

## Minimum useful photo package

Start with these twelve shots:

1. complete arm, front three-quarter;
2. complete arm, side;
3. complete arm, folded/rest pose;
4. Base gear and zero mark;
5. Base servo label;
6. Shoulder servo label;
7. Elbow servo label;
8. camera servo and camera-module labels;
9. HAT top;
10. HAT bottom/revision;
11. power-supply label and power-disconnect path;
12. full wiring/cable-routing overview with power off.

The raw originals stay outside Git. Only the reviewed exports enter the tracked
folders.

## Before staging new material

```powershell
python tools/check_repo.py
git status --short
```

Then inspect exactly what will be staged:

```powershell
git add <explicit-file-or-folder>
git diff --cached --stat
git diff --cached
```

Avoid using a broad add command when raw media or unrelated local files are
nearby. The repository check catches common release mistakes, but visual review
is still required.

## Owner decisions still required

Before the first public release, answer the items in
[`OPEN_QUESTIONS.md`](OPEN_QUESTIONS.md). The critical ones are repository
visibility, public attribution, license split, and CAD provenance. The ARM-only
software scope is now recorded in `SOURCE_RELEASE_PLAN.md`.
