# Electronics design files

This folder separates electrical intent from physical harness construction:

- [`schematics/`](schematics/) — power domains, protection, interfaces, and
  connector pin definitions;
- [`wiring/`](wiring/) — point-to-point cables, connector orientation, wire
  gauge/color/length, labels, and routing.

Read [`../../docs/ELECTRONICS.md`](../../docs/ELECTRONICS.md) first. The current
repository does not yet contain a verified pinout. Do not infer wiring from
software signal names, connector shape, or wire color.

The candidate 12 V, 7.5 A supply, camera-servo voltage/regulator, fusing,
grounding, polarity, wire ratings, and hardware E-stop/disconnect all require
owner verification before a reproducible wiring guide can be released.
