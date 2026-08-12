# STL exports

Put revision-matched printable-part meshes here. STL has no dependable unit
metadata, so every mesh must be exported numerically in **millimetres** and
checked against its drawing/bounding box after re-import.

Use:

```text
co-arm_<subsystem>_<part-name>[_<variant>]_rNN.stl
```

Requirements:

- one manufactured part per file unless a multi-part print is intentional;
- binary STL;
- manifold/watertight mesh with outward normals;
- no self-intersections, duplicate shells, supports, raft, brim, or G-code;
- initial chordal tolerance 0.10 mm or finer and angular tolerance 1 degree or
  finer, with tighter settings where functional geometry needs it;
- reopened scale and bounding-box check.

These are geometry exports, not automatically proven print profiles. See
[`print-ready/`](print-ready/) and
[`../../docs/CAD_AND_STL.md`](../../docs/CAD_AND_STL.md).
