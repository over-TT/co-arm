# Media guide

The goal is to make the arm understandable without publishing faces, screens,
addresses, account details, serial numbers, QR codes, unrelated desk contents,
or embedded location metadata.

## Intake workflow

1. Copy new originals into untracked `media/raw/`.
2. Keep the originals outside Git history.
3. Review the full frame for personal and device-specific details.
4. Crop or blur anything unrelated to the project.
5. Remove EXIF/XMP/GPS metadata.
6. Export an approved web copy with a descriptive filename.
7. Place it in `media/photos/`, `media/diagrams/`, or `media/video/`.
8. Add a caption and provenance row to the local folder README.
9. Run `python tools/check_repo.py` before staging.

## Photo shot list

### Overview

- front three-quarter view of the complete arm;
- side view showing Base, Shoulder, Elbow, and Camera joints;
- folded/rest pose;
- scale reference that does not reveal personal information.

### Hardware detail

- Base gear and mechanical zero mark;
- each servo label;
- HAT top and bottom, including printed model/revision;
- Raspberry Pi and camera mounting;
- power-supply label;
- physical power-cut or E-stop, if installed.

### Assembly

- parts laid out by stage;
- Base assembly;
- Shoulder and Elbow assembly;
- camera bracket and cable routing;
- wiring and strain relief;
- final mechanical checks.

### Documentation evidence

- clean dashboard overview with private account/device data hidden;
- measured vs. planned 2D scene;
- one wide camera frame and one detail frame, only after desk privacy review.

## Video shot list

- slow physical overview with power off;
- controlled boot and read-only telemetry check;
- one bounded joint movement at a time;
- coordinated move with the sweep visibly clear;
- wide-view to close-view active-perception sequence;
- physical STOP/power-cut demonstration only after its procedure is defined.

## File conventions

- Photos: `YYYY-MM-DD_subject_view_rNN.jpg`
- Diagrams: `subject_diagram_rNN.svg` plus a PNG fallback when useful
- Short clips: `YYYY-MM-DD_subject_rNN.mp4`
- Use lower-case ASCII, hyphens, and a two-digit revision.
- Prefer JPEG/WebP for photos, SVG for diagrams, and H.264 MP4 for short clips.
- Keep individual tracked media comfortably below GitHub's size limit. Use
  Releases or external hosting for long videos.

Do not use Git LFS until the repository owner deliberately chooses it; LFS
changes clone and storage behavior for every contributor.
