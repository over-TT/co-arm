# Public source boundary

The ARM implementation is now exported into this repository. This document
records the boundary so later updates do not copy the broader development
workspace or private runtime state back into Git.

## Included allowlist

The standalone source root is `software/`:

| Area | Public path |
| --- | --- |
| Dashboard | `software/dashboard/` |
| ARM-only FastAPI backend | `software/python/web_backend/` |
| Raspberry Pi gateway | `software/python/robot_gateway/` |
| MCP server | `software/python/arm_mcp/` |
| Core Isaac simulator/digital twin | `software/python/arm_sim/` |
| Arm HAT firmware | `software/firmware/` |
| Portable Codex plugin | `software/plugin/` |
| Deployment/diagnostic helpers | `software/operations/` |
| Public source map | `software/SOURCE_INDEX.json` |
| Deterministic source snapshot | `software/SOURCE_MANIFEST.json` |

Focused tests, fixtures, configuration templates, model assets, package
manifests, and lockfiles needed to build or test these areas are part of the
allowlist. Machine-specific values must be passed through local configuration,
environment variables, or ignored token/data files.

## Explicit exclusions

Never copy these from a development or deployed machine:

- tokens, credentials, private keys, SSH config/known-host state, or account
  metadata;
- runtime databases, WAL/SHM files, chats, transcripts, operator memory, or
  agent homes;
- camera captures, retained frames, screenshots, raw audio, or personal desk
  content unless separately reviewed for `media/`;
- backups, SD-card images, installed services, generated simulation output,
  caches, logs, `node_modules/`, build output, or bundled toolchains;
- private endpoints, usernames, device/controller/session/frame identifiers,
  live poses, or calibration state from the reference machine;
- unrelated UI, backend routes, products, or tests from the parent workspace;
- unrelated product features, embedded agent runtimes, and superseded simulator
  branches;
- generated reports until they are regenerated without personal paths and with
  redistributable fonts/assets.

The public plugin contains portable instructions only. The setup runbook
registers `python -m arm_mcp` with an absolute project-venv interpreter and
machine-local configuration, so no interpreter, endpoint, or token path is
copied into Git.

## Adaptations made for standalone use

- The complete Arm UI has its own Vite/React shell and package manifest.
- The FastAPI app contains only arm, camera, backend, session, and Control
  Center routes and serves the built dashboard on loopback.
- Real and simulated gateway configuration is request-scoped and fails closed
  when required local configuration is absent.
- The supported agent path is `AGENTS.md`, the setup runbook, and the external
  MCP/plugin; no embedded Codex runtime is bundled.
- Source-identity and Control Center paths are relative to this repository.
- The Pi gateway, MCP server, core Isaac simulator, Arm HAT firmware, plugin,
  and operations files preserve their ARM responsibility without duplicate
  source trees.

## Update procedure

1. Start from the canonical ARM source map in the private workspace.
2. Copy only the documented allowlist into the matching public path.
3. Reapply the standalone adaptations instead of copying parent-app entry
   points or private configuration.
4. Review the diff for new dependencies, routes, runtime-file types, identifiers,
   and non-ARM imports.
5. Update `software/SOURCE_INDEX.json`, public docs, changelog, and third-party
   notices, then regenerate `software/SOURCE_MANIFEST.json` with
   `python tools/build_source_manifest.py`.
6. Run `python tools/build_source_manifest.py --check`, the repository checker,
   Python suites, dashboard tests/build, firmware
   host tests, and any source-index/snapshot verifier.
7. Inspect the clean Git diff and history before pushing.

## Publication gates

A source-complete tree is not automatically ready for public reuse. Publication
still requires:

- green checks for the exact clean checkout;
- manual review of Git history and generated lockfiles/manifests;
- explicit software, hardware/CAD, documentation, and media license decisions;
- complete third-party notices for anything redistributed;
- privacy/provenance review of any added media, CAD, or reports;
- release notes that keep source/build results separate from dated live-device
  and physical evidence.
