# Native CAD sources

Put the editable files from the original CAD application here. Preserve
parametric history, sketches, constraints, components, joints, datums, and
design tables.

Use names such as:

```text
co-arm_base_motor-mount_r01.<native-extension>
co-arm_assembly_full-arm_r01/<linked-native-files>
```

If the design is a linked multi-file assembly, keep it in a same-named folder
and include any application-specific project/archive file needed to reopen it.
Do not flatten the only editable copy into STEP or STL.

Before adding files:

- remove unrelated experiments and hidden private metadata;
- confirm the file opens and rebuilds without missing references;
- check units are millimetres and axes match the project convention;
- note the authoring application/version in the commit or a package README;
- export the same `rNN` revision to `../step/`, `../../stl/`, and drawings as
  applicable.
