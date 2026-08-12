# CAD

Store editable sources in [`native/`](native/) and neutral solid exports in
[`step/`](step/). Do not mix STL meshes, slicer projects, drawings, or firmware
into these folders.

Follow [`../../docs/CAD_AND_STL.md`](../../docs/CAD_AND_STL.md) for the required
millimetre units, coordinate frame, naming, `rNN` revisions, STEP settings, and
matched-export checklist.

Minimum release set:

- top-level zero-position assembly;
- one file/component per manufactured part;
- purchased actuator/controller reference geometry where licensing permits;
- named Base, Shoulder, Elbow, and Camera axes;
- named work-surface and tool/camera datums;
- revision-matched STEP and drawings.

Do not infer ownership or apply a license to imported vendor models until their
source and terms are documented.
