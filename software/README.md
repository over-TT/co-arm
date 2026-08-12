# Software

This tree is a documented release boundary, not yet an executable source
release. The implementation is working in a larger private parent workspace,
but copying that workspace would also publish unrelated code, local runtime
state, generated artifacts, and installation-specific details.

| Folder | Intended contents | Current state |
| --- | --- | --- |
| `firmware/` | ESP32 Arm HAT sketch, controller/protocol library, arm-only host tests | Reserved; source pending scope and license |
| `gateway/` | Complete Raspberry Pi `robot_gateway` package, tests, service template, dependencies | Reserved; source pending scope and license |
| `mcp/` | Canonical typed Arm MCP server, tests, portable configuration example | Reserved; source pending scope and license |
| `dashboard/` | Standalone Arm Lab UI and an arm-only backend/proxy | Reserved; requires extraction work |

The recommended allowlist and release gates are in
[`docs/SOURCE_RELEASE_PLAN.md`](../docs/SOURCE_RELEASE_PLAN.md). Do not put
tokens, endpoints, sessions, retained frames, logs, factory backups, generated
binaries, bundled toolchains, or an installed plugin configuration here.

Historical source checks in the private parent workspace are recorded in
[`docs/STATUS.md`](../docs/STATUS.md). They are not clean-checkout proof for
this repository.
