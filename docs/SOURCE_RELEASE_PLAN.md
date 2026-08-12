# Software source release plan

The documentation tree is clean, but the working arm software originated in a
larger private application. Copying that whole workspace would publish unrelated
code, local state, generated binaries, and personal installation details. A
source release therefore needs an explicit allowlist and a clean-checkout test.

## Recommended first release boundary

For the first private review commit, publish this documentation, hardware/CAD
scaffold, media workflow, and repository checks. Add executable source only
after the owner confirms scope and licensing.

## Portable source candidates

The strongest candidates for a later allowlisted export are:

- the complete clean Raspberry Pi `robot_gateway` Python package and its tests;
- the ESP32 Arm HAT sketch and `ArmHatController` library;
- the arm-specific firmware host tests and a rewritten current protocol spec;
- the canonical Arm MCP server, with portable configuration templates;
- a small arm-only deployment template and dependency manifests.

Preserve complete packages rather than selecting individual Python modules:
the gateway runtime imports state, calibration, simulation, camera, physical
commissioning, and simple-arm modules across the package.

### Audited parent-source allowlist

If the owner approves a software release, begin from this reviewed relative
source set and sanitize it into the reserved `software/` tree:

| Parent source | Intended public destination | Notes |
| --- | --- | --- |
| `robot_gateway/**/*.py` and `robot_gateway/tests/**/*.py` | `software/gateway/robot_gateway/` | Copy the clean package as a unit; exclude caches |
| `firmware/esp32/arm_hat_controller/arm_hat_controller.ino` | `software/firmware/arm_hat_controller/` | Current ESP32 sketch |
| `firmware/libraries/ArmHatController/` | `software/firmware/libraries/ArmHatController/` | Fix author/maintainer metadata after attribution is chosen |
| Arm-specific firmware host tests and stubs under `protocol/tests/` | `software/firmware/tests/` | Exclude unrelated board/MCUCP tests |
| `ARM/mcp/server.py` and its focused tests | `software/mcp/` | Add portable config; never copy installed `.mcp.json` |
| `deploy/arm-gateway.service` | `software/gateway/deploy/` | Add a documented opt-in physical-UART drop-in template |
| Pi/web dependency manifests | relevant software folder | Split arm runtime dependencies from the parent app |

The old public-looking protocol and Pi setup pages in the parent workspace are
not safe to copy verbatim: they describe earlier firmware/deployment states.
Rewrite them against the exported 2.4 source.

## Components that are not standalone yet

### Dashboard

The Arm React components currently build inside a broader application entry
point. A public source release needs its own entry component, styles, package
metadata, test setup, and API configuration before it can be described as a
standalone dashboard.

### Local backend and embedded Arm Chat

The parent backend also owns board discovery, Arduino toolchains, a bundled
Codex runtime, application state, and frontend distribution. Copying its Arm
files alone does not create a runnable service. Extract an arm-only proxy and
chat supervisor, or publish a documented interface without claiming a working
standalone backend.

### Codex plugin

The installed plugin configuration contains installation-specific executable,
workspace, token-file, and frame-directory paths. Publish only a disabled or
placeholder template that requires explicit local configuration. Never publish
the installed configuration.

## Files and data excluded from any source export

- private chronological handoffs and agent transcripts;
- runtime tokens, sessions, known-hosts data, retained camera frames, and logs;
- device factory firmware backups;
- generated binaries, objects, maps, caches, and bundled toolchains;
- private network and controller identifiers;
- legacy raw-register diagnostic scripts that can change servo configuration;
- account state or personal AI memory.

## Clean source release gates

1. Define the copied paths in a machine-readable source manifest.
2. Replace local paths and fixed endpoints with environment/config templates.
3. Rewrite stale protocol and deployment documentation from current source.
4. Install dependencies from documented manifests in a fresh checkout.
5. Run gateway, MCP, firmware host, frontend, typecheck, and build checks that
   actually apply to the exported tree.
6. Compile the Arm HAT firmware with a documented system Arduino CLI and pinned
   ESP32 core; do not rely on the private bundled toolchain.
7. Run repository, secret, path, link, and file-size checks.
8. Report source/build evidence separately from live-device evidence.

Historical parent-workspace checks are summarized in [`STATUS.md`](STATUS.md),
but they are not a substitute for a green clean-checkout source release.
