# Provenance and privacy

## Source boundary

This clean repository is an allowlisted export from a larger private robotics
workspace. It deliberately excludes unrelated application code, generated
toolchains, build outputs, firmware backups, tokens, SSH material, service logs,
retained camera frames, screenshots, local caches, account state, and the raw
chronological agent handoff.

Generated engineering reports are also excluded until they can be regenerated
without personal paths and with fonts/assets whose redistribution terms are
understood. A PDF or screenshot is not made public merely because its
underlying source is public.

The public history was rebuilt from the current arm contract and dated proof
records. Installation-specific values were removed rather than replaced with
plausible-looking examples.

## Data never intended for this repository

- credentials, bearer tokens, private keys, or known-hosts files;
- personal absolute paths or usernames;
- fixed private-network addresses, host fingerprints, or service credentials;
- exact controller, USB, MAC, boot, plan, or camera-frame identifiers;
- raw desk-camera media or unreviewed screenshots;
- factory firmware backups or generated vendor binaries;
- account tier, model catalog, transcripts, or personal AI memory;
- logs containing command lines, paths, or network topology.

## Hardware and CAD provenance

Vendor and product names identify compatibility targets and remain the property
of their owners.

All 12 current 3MF parts were supplied directly by the project owner on
2026-08-26. Their archives contain model titles/revisions and geometry but no
author name, source URL, or third-party license metadata. Before publishing or
licensing them, confirm whether every mesh is fully original or adapted from
another source. If anything was adapted, record its source, author, license,
changes, and revision in `hardware/3mf/README.md`.
