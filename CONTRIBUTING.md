# Contributing

This repository is in a documentation-first release stage. Small, reviewable
changes with explicit evidence are preferred.

## Before opening a change

1. Read [Status](docs/STATUS.md), [Control and safety](docs/CONTROL_AND_SAFETY.md),
   and [Provenance](docs/PROVENANCE.md).
2. Do not add credentials, raw logs, personal paths, unreviewed media, device
   identifiers, or factory firmware backups.
3. Do not describe a build/test result as physical proof.
4. Do not copy the reference calibration into another arm and move it.
5. Run `python tools/check_repo.py`.

## Evidence labels

Use one of these labels when making a claim:

- `source`: direct inspection of source or configuration;
- `automated-test`: a named test and result;
- `build`: a named compiler/build and result;
- `live-telemetry`: a dated read from connected hardware;
- `image`: visible evidence from a dated frame;
- `operator-observed`: a dated physical observation;
- `unverified`: a candidate or open assumption.

## Hardware changes

For a mechanical or electrical change, include:

- affected part/revision;
- reason and failure mode addressed;
- drawing or photo;
- material and manufacturing details;
- measurement method and result;
- whether the change was installed and physically tested;
- rollback or previous revision when relevant.

## Media

Use the [media intake workflow](docs/MEDIA_GUIDE.md). Do not commit anything
from `media/raw/`; it is intentionally ignored.

## License boundary

Contributions cannot be accepted for redistribution until the repository owner
selects licenses and a contribution policy. Until then, coordinate privately
with the owner before submitting third-party or original source/CAD.
