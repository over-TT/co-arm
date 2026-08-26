# Co-Arm dashboard

This directory is the standalone React/Vite frontend for the Co-Arm itself. It
contains Overview, guarded Control, bounded Live Follow, camera, calibration,
commissioning, Guide, SIM/REAL selection, and the visible Isaac Sim launcher.

Agent assistance stays outside the dashboard. Open the repository root in
Codex, follow `AGENTS.md` and `docs/SETUP_WITH_CODEX.md`, then configure the
portable plugin at `software/plugin/plugins/arm-alliance/` to help build, set
up, and run the arm through the Raspberry Pi-backed stack.

The dashboard keeps the same-origin API contract as the integrated ARM
application. It expects the Co-Arm backend to serve the `/api/arm` and
`/api/camera` routes. The frontend deliberately remains useful
in its offline state when those routes are unavailable; it does not simulate a
successful hardware connection.

## Run locally

```sh
pnpm install --frozen-lockfile
pnpm dev
```

Open the URL printed by Vite. For live backend access, serve the built files
from the ARM backend or put both behind a same-origin reverse proxy.

## Verify

```sh
pnpm test
pnpm build
```

The production output is written to `dist/` and is intentionally ignored by
Git.
