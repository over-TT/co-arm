# Mechanical drawings

Put revision-matched part and assembly drawings here. Prefer a readable PDF for
every released drawing and include the editable/source drawing when available.

Use the same basename as CAD:

```text
co-arm_<subsystem>_<part-name>[_<variant>]_rNN.pdf
```

Part drawings should state units, projection, material/process when confirmed,
datums, overall and fit-critical dimensions, holes/threads/inserts, tolerances,
orientation, and quantity. Assembly drawings should add exploded views, BOM
balloons, fastener stack order, joint zero views, gear timing/mesh, and cable
routing.

Do not invent material, tolerance, fastener, or torque values. Leave an explicit
`TODO (owner verification)` note until the as-built part is inspected.
