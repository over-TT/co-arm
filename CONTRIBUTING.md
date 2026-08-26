# Contributing

`co-arm` is a working prototype. Keep changes focused and explain what you
actually tried.

## Before you open a change

1. Read [Status](docs/STATUS.md) and the document for the part you are changing.
2. Update the matching test, BOM row, drawing, or setup step.
3. Keep raw logs and unreviewed media out of the commit.
4. Run `python tools/check_repo.py`.

## Say what you checked

Tell us whether you:

- read the code;
- ran tests;
- built the software or firmware;
- tried it on the real arm;
- checked a photo or camera frame;
- have not checked it yet.

Use normal sentences. A passing test does not mean the real arm moved, and a
move command does not mean the joints reached the target.

## Hardware changes

For a mechanical or electrical change, include:

- the affected part and revision;
- the reason for the change and the failure mode it addresses;
- a drawing or photo;
- material and manufacturing details;
- the measurement method and result;
- whether it was installed and physically tested;
- the rollback or previous revision when relevant.

For a printed part, also update `hardware/3mf/README.md` or the matching CAD/STL
record, the BOM row, and the assembly step it changes.

## Photos and video

Use the [media intake workflow](docs/MEDIA_GUIDE.md). Do not commit anything
from `media/raw/`; it is intentionally ignored.

## License boundary

Contributions cannot be accepted for redistribution until the repository owner
selects licenses and a contribution policy. Until then, coordinate privately
with the owner before submitting third-party or original source/CAD.
