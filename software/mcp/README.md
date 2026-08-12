# Arm MCP source — pending

This folder is reserved for the canonical typed MCP server, its tests, and a
portable disabled configuration template. The server should remain a thin
adapter over the Raspberry Pi policy boundary; it must not open the servo bus
or duplicate calibration/safety policy.

The public template must use explicit local configuration for the gateway URL,
credential file, retained-frame directory, and source entrypoint. Never copy
the installed plugin configuration from the reference machine.

The documented tools are `arm_state`, `arm_scene`, `arm_plan`,
`arm_apply_plan`, `arm_move`, `arm_look`, `arm_detail`, `arm_release`, and
`arm_floor_guard`. Source will be added after scope and licensing are confirmed.
