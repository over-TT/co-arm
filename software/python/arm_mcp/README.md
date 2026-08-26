# Arm MCP server

This package lets Codex use the same Raspberry Pi arm service as the dashboard.
The Pi works out the move and the ESP32 talks to the servos; MCP does not create
a second motor controller.

Set `ARM_GATEWAY_TOKEN_FILE` to an owner-readable token file, set
`ARM_EXPECTED_BACKEND` explicitly to `sim` or `real`, and optionally set
`ARM_GATEWAY_URL` to the local tunnel or simulator gateway origin. Before every
operational call, the client authenticates `/healthz` and refuses a different
backend kind. The origin must be plain HTTP on an explicit port with the host
exactly `localhost`,
`127.0.0.1`, or `::1`; paths, credentials, queries, fragments, redirects, and
environment proxies are not accepted for this bearer-token client. Then start
the stdio server from `software/` with:

```powershell
$env:PYTHONPATH = "python"
python -m arm_mcp
```

No installed Codex or MCP configuration is included in this repository. Keep
credential files and retained frames outside the checkout.

If the reply to a move is lost, MCP reads the arm before trying anything else.
It does not blindly send the move twice.
