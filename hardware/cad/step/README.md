# STEP exports

Put neutral solid CAD exports here. Prefer STEP AP242; use AP214 only when the
authoring application cannot produce a reliable AP242 file.

Each STEP filename must match its native source revision:

```text
co-arm_<subsystem>_<part-name>[_<variant>]_rNN.step
```

Export in millimetres, preserve assembly names when supported, and reopen every
file to verify scale, axes, body count, placement, and missing faces. STEP files
are exchange artifacts; make design changes in `../native/` and regenerate the
export.
