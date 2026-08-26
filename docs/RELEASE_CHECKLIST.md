# Release checklist

## Owner decisions

- [ ] Visibility confirmed: private review or public.
- [ ] Attribution confirmed.
- [ ] Software, hardware/CAD, documentation, and media licenses confirmed.
- [x] ARM-only software-export scope recorded in `SOURCE_RELEASE_PLAN.md`.
- [ ] CAD and media provenance confirmed.
- [ ] Hero photo, dashboard screenshot, short proof clip, and social preview
  approved for public use.

## Documentation

- [ ] No `TODO(owner)` needed for a claim presented as reproducible.
- [ ] Status date and evidence tiers are current.
- [ ] BOM matches photographs and labels.
- [ ] Geometry matches the exported CAD revision.
- [ ] All relative links pass.
- [ ] Safety limitations remain prominent.

## Privacy and secrets

- [ ] No personal absolute paths or usernames.
- [ ] No installation-specific IPs, host fingerprints, private-key names, or
  credential/token paths; any documented private subnet is an explicit
  portable default/example rather than reference-machine state.
- [ ] No controller, MAC, USB serial, boot, or frame IDs from the reference
  installation.
- [ ] No raw logs, camera frames, screenshots, caches, backups, or build trees.
- [ ] Media was reviewed and metadata removed.
- [ ] `python tools/check_repo.py` passes.
- [ ] `python tools/build_source_manifest.py --check` passes.

## Source release

- [ ] Configuration is templated and secrets are environment/file inputs.
- [ ] Dependencies and versions are documented.
- [ ] License headers and third-party notices are complete.
- [ ] Python gateway/backend/MCP/simulator suites, dashboard tests/build, and
  applicable firmware host tests pass from the clean checkout.
- [ ] The GitHub Source validation workflow passes for the exact pushed commit.
- [ ] Generated binaries are excluded unless redistribution was reviewed.
- [ ] Source/build evidence is not described as physical proof.

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
