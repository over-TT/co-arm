# Codex plugin and setup skill

`plugins/arm-alliance/` gives Codex two skills: one for setting up or building
co-arm, and one for using an arm that is already connected.

Open the co-arm checkout in Codex first; `AGENTS.md` and
`docs/SETUP_WITH_CODEX.md` work without installing the plugin. Install this
repo-local marketplace and plugin from the repository root:

```powershell
codex plugin marketplace add .\software\plugin
codex plugin add arm-alliance@co-arm
```

The plugin contains instructions only, so installation never needs a gateway
token or project interpreter. Use `arm-alliance-setup` for clone, validation,
Isaac, Pi, firmware, deployment, or commissioning requests.

Install the Python stack into a project virtual environment before enabling
live tools. Configure the MCP server machine-locally with the absolute virtual
environment interpreter and these values:

- `ARM_GATEWAY_URL` — the loopback gateway origin;
- `ARM_EXPECTED_BACKEND` — exactly `sim` or `real`, selected independently of
  the URL and port;
- `ARM_GATEWAY_TOKEN_FILE` — an absolute path to the local token file;
- `ARM_FRAME_DIR` — an ignored local directory for retained frames.

Use the exact `codex mcp add` flow in `docs/SETUP_WITH_CODEX.md`. Use
`arm-alliance` after the selected SIM or real gateway is running and
`arm_state` works. Start a new Codex task after installing or updating the
plugin so its skills reload.
