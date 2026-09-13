# Release checklist

## Owner decisions

- [x] Public prototype source release authorized on 2026-09-13.
- [x] Attribution: co-arm by over-TT.
- [x] PolyForm Noncommercial 1.0.0 for software; CC BY-NC 4.0 for designs,
  documentation, and media; see `LICENSING.md`.
- [x] ARM-only software-export scope recorded in `SOURCE_RELEASE_PLAN.md`.
- [ ] CAD and media provenance confirmed.
- [ ] Hero photo, dashboard screenshot, short proof clip, and social preview
  approved for public use.

## Documentation

Full build reproduction and approved demo media remain follow-up milestones;
they are not claimed by this public prototype source snapshot.

- [ ] No `TODO(owner)` needed for a claim presented as reproducible.
- [x] Status date and evidence tiers are current.
- [ ] BOM matches photographs and labels.
- [ ] Geometry matches the exported CAD revision.
- [x] All relative links pass.
- [x] Prototype and physical limitations remain documented.

## Privacy and secrets

- [x] No personal absolute paths or machine-account names found by the release checks.
- [x] No installation-specific IPs, host fingerprints, private-key names, or
  credential/token paths; any documented private subnet is an explicit
  portable default/example rather than reference-machine state.
- [x] No controller, MAC, USB serial, boot, or frame IDs from the reference
  installation.
- [x] No raw logs, camera frames, screenshots, caches, backups, or build trees.
- [ ] Media was reviewed and metadata removed.
- [x] `python tools/check_repo.py` passes.
- [x] `python tools/build_source_manifest.py --check` passes.

## Source release

- [x] Configuration is templated and secrets are environment/file inputs.
- [x] Dependencies and versions are documented.
- [x] Applicable license texts and package metadata are present; third-party notices are retained.
- [x] Python gateway/backend/MCP/simulator suites, dashboard tests/build, and
  applicable firmware host tests pass from the local release tree; see `STATUS.md`.
- [ ] The GitHub Source validation workflow passes for the exact pushed commit.
- [x] Generated binaries are excluded unless redistribution was reviewed.
- [x] Source/build evidence is not described as physical proof.

## Git and GitHub

- [ ] Review `git status --short` and `git diff --cached`.
- [ ] Stage only the clean `co-arm` repository.
- [ ] Confirm no file is unexpectedly large.
- [ ] Commit message describes the ARM source export.
- [ ] Push to the intended private/public repository.
- [ ] Review the rendered README and Mermaid diagram on GitHub.
- [ ] Add the GitHub description, homepage/demo link, and focused topics.
- [ ] Upload the reviewed social-preview image and create a tagged release.
- [ ] Configure GitHub private vulnerability reporting or publish a private
  security contact before inviting outside testing.
- [ ] Enable branch protection and secret scanning where available.
