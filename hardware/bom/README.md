# Bill of materials

[`bom.csv`](bom.csv) is the machine-readable as-built bill of materials. It is
seeded with the currently known electronics and intentionally incomplete
mechanical rows.

## Columns

| Column | Meaning |
| --- | --- |
| `item_id` | Stable public identifier such as `ACT-001`; never reuse an ID for a different part |
| `subsystem` | Functional group: compute, control, actuation, vision, power, mechanical, wiring, or hardware |
| `description` | Plain-language part description |
| `manufacturer` | Marked manufacturer; blank until confirmed |
| `manufacturer_part_number` | Exact model/MPN from label or datasheet |
| `supplier` | Supplier used for this build; blank if unknown |
| `supplier_sku` | Supplier catalog/SKU, not an order/account number |
| `quantity` | Quantity installed in one arm |
| `unit` | Usually `each`, plus `m`, `g`, or another explicit unit when needed |
| `revision` | Hardware/design revision where applicable |
| `status` | `recorded`, `owner-verification`, or `TODO` |
| `source_or_evidence` | Public-safe evidence note, drawing, photo, or official product reference |
| `notes` | Fit, variant, joint mapping, or unresolved question |

## Rules

- Transcribe exact labels; do not silently turn a working software profile into
  a confirmed manufacturer part number.
- Use one row per distinct part/specification. Split variants even when they
  have the same function.
- Do not store prices, personal order IDs, addresses, credentials, or private
  serial numbers.
- Add every printed part, bearing, shaft, gear, horn, fastener, insert, washer,
  spacer, connector, wire type, regulator, fuse, and mounting item before
  calling the BOM complete.
- Keep BOM revision references aligned with CAD and drawings.
- A blank field means unknown; `TODO` in `status` means the row itself is only a
  placeholder.
