# Proof log

This compact log records the strongest public-safe evidence. It omits private
paths, device serials, network details, opaque frame IDs, credentials context,
and raw personal workspace history.

| Date | Claim | Evidence | What it does not prove |
| --- | --- | --- | --- |
| 2026-08-05 | ST3215 load-decode defect identified and fixed | Firmware analysis, tests, later hardware behavior | Final Base reliability |
| 2026-08-05 | SC09 ID collision isolated and camera servo assigned ID 4 | Isolation and clean four-ID scan | Long-duration bus reliability |
| 2026-08-07 | Earlier Mode-3 control moved all joints and reached wide Base angles | Live motion observations | Reliability of the later 2.4 control path |
| 2026-08-08 | Codex used typed tools to move and capture a real image | Live telemetry and pixels | General autonomous perception |
| 2026-08-08 | Earlier Base bookkeeping could report false zero | Controlled failure observation | A defect in the later native Mode-0 method |
| 2026-08-10 | Native extended-position configuration persisted | Physical register readback before and after power cycle | Numeric arrival across all travel |
| 2026-08-10 | Normal Base signed motion and return accepted | Operator physical observation | Seam, endpoint, retarget, or endurance table |
| 2026-08-11 | Full-resolution fixed-focus capture deployed | Live capture plus metadata | Autofocus or exact object identification |
| 2026-08-11 | Plan/apply preview deployed | Live motionless preview, zero warnings | Physical plan application or arrival |
| 2026-08-12 | Link and bus online; floor guard on; Base angle unavailable | Read-only live state | Complete current pose or Base truth |
| 2026-08-22 | Camera Module 3 Wide path reported IMX708 identity/array, upright capture, and powered AF controls | Deployed status/capture evidence | Wide lens variant from sensor ID alone, or sharpness from a control request alone |
| 2026-08-23 | Five- and twelve-second unloaded Shoulder/Elbow Live Follow runs completed at raw 2400/50 with a ±90 deg allowance | Matching deployed 2.7.2 stack, grouped goal readback, measured trace, heartbeat, orderly release | Degrees/second, every load/pose, collision sensing, or long-duration reliability |
| 2026-08-24 | ARM-only dashboard/backend, gateway, firmware, simulator, MCP/plugin, operations, and tests exported | Public source inspection and clean-checkout verification workflow | Deployment, current live state, or new physical behavior |
| 2026-08-25 | Focused four-joint arm/Pi/Arm HAT/Isaac/dashboard/agent source boundary validated | 689 Python tests, 180 dashboard tests plus build, isolated Python wheel-artifact build, no-gateway dashboard runtime smoke, 17 Arm HAT host tests, six parsed PowerShell scripts, current 147-file manifest, and 207-file repository check | A self-contained wheel runtime, Isaac launch, Arduino device build/flash, Pi deployment, or physical-arm proof |
| 2026-08-26 | Agent contract and local release candidate hardened | 714 Python tests plus 9 nested subtests, 49 focused MCP/index tests, 180 dashboard tests plus 42-module build, clean Python/Pi/dashboard dependency audits, source-isolated installed-wheel smoke, 17 Arm HAT host tests, six parsed PowerShell scripts, validated plugin and skills, current 151-file manifest, and 212-file repository check | A GitHub-hosted workflow run, Isaac launch, Arduino device build/flash, Pi deployment, fresh live state, or new physical-arm proof |
| 2026-08-26 | Current printable mechanical set added | 12 3MF files inspected and indexed, including both Base gears; 225-file repository check passed | Print settings, fit on the physical arm, bearing details, or a complete fastener schedule |

See [Status](STATUS.md) for the current proof matrix and [History](HISTORY.md)
for the engineering narrative.
