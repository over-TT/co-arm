# Release checklist

## Owner decisions

- [ ] Visibility confirmed: private review or public.
- [ ] Attribution confirmed.
- [ ] Software, hardware/CAD, documentation, and media licenses confirmed.
- [ ] Software-export scope confirmed.
- [ ] CAD and media provenance confirmed.

## Documentation

- [ ] No `TODO(owner)` needed for a claim presented as reproducible.
- [ ] Status date and evidence tiers are current.
- [ ] BOM matches photographs and labels.
- [ ] Geometry matches the exported CAD revision.
- [ ] All relative links pass.
- [ ] Safety limitations remain prominent.

## Privacy and secrets

- [ ] No personal absolute paths or usernames.
- [ ] No IPs, host fingerprints, private-key names, or token paths.
- [ ] No controller, MAC, USB serial, boot, or frame IDs from the reference
  installation.
- [ ] No raw logs, camera frames, screenshots, caches, backups, or build trees.
- [ ] Media was reviewed and metadata removed.
- [ ] `python tools/check_repo.py` passes.

## Source release, when included

- [ ] Configuration is templated and secrets are environment/file inputs.
- [ ] Dependencies and versions are documented.
- [ ] License headers and third-party notices are complete.
- [ ] Firmware, gateway, MCP, and dashboard tests pass from the clean checkout.
- [ ] Generated binaries are excluded unless redistribution was reviewed.
- [ ] Source/build evidence is not described as physical proof.

## Git and GitHub

- [ ] Review `git status --short` and `git diff --cached`.
- [ ] Stage only the clean `co-arm` repository.
- [ ] Confirm no file is unexpectedly large.
- [ ] Commit message describes the documentation release.
- [ ] Push to the intended private/public repository.
- [ ] Review the rendered README and Mermaid diagram on GitHub.
- [ ] Enable branch protection and secret scanning where available.
