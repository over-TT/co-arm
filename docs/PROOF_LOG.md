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

See [Status](STATUS.md) for the current proof matrix and [History](HISTORY.md)
for the engineering narrative.
