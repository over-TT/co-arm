# Hardware files

This directory is the landing area for the co-arm physical design package.

```text
hardware/
|-- bom/                  Exact purchased and manufactured parts
|-- cad/
|   |-- native/           Editable source CAD
|   `-- step/             Neutral solid exports
|-- stl/
|   `-- print-ready/      Verified, intentionally oriented print candidates
|-- drawings/             Dimensioned part and assembly drawings
`-- electronics/
    |-- schematics/       Electrical intent and power distribution
    `-- wiring/           Point-to-point harness and connector documentation
```

Start with [`../docs/HARDWARE.md`](../docs/HARDWARE.md), then follow the export
and naming rules in [`../docs/CAD_AND_STL.md`](../docs/CAD_AND_STL.md).

## Upload order

1. Add label photographs and use them to finish `bom/bom.csv`.
2. Add editable CAD to `cad/native/`.
3. Export the matching revision to `cad/step/`.
4. Export printable parts to `stl/`; promote only checked parts to
   `stl/print-ready/`.
5. Add revision-matched drawings to `drawings/`.
6. Add electrical source files and readable exports to
   `electronics/schematics/` and `electronics/wiring/`.
7. Update the assembly documentation only after checking the files against the
   physical arm.

Do not add credentials, local network addresses, private filesystem paths,
unwanted serial numbers, or personal metadata. Do not add a license or
third-party attribution unless ownership and the exact terms have been
confirmed.
