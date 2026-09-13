# Third-party notices

This repository contains project source and dependency manifests. Generated
dependency trees, vendor toolchains, firmware binaries, simulator installs, and
factory-firmware backups are not redistributed.

Project-owned material uses the category-specific noncommercial licenses in
[Licensing](LICENSING.md). Those licenses do not replace dependency or vendor
licenses. The license texts in `LICENSE` and `LICENSES/` are reproduced from
the official PolyForm Project and Creative Commons publications without changes.

## Python package dependencies

The standalone Python package declares FastAPI, HTTPX, Pillow, Pydantic,
pyserial, Starlette, and Uvicorn at runtime; setuptools and wheel build it, and
pytest, NumPy, and headless OpenCV run its host tests. Direct constraints are recorded in
`software/python/pyproject.toml`, while `software/python/requirements.lock`
records the exact Python 3.11 runtime/test graph used by source validation.
The Raspberry Pi deployment set is pinned separately in
`software/operations/requirements-pi.txt`. Dependency licenses and notices
still come from their upstream distributions.

## Dashboard dependencies

The dashboard declares React, React DOM, Vite, TypeScript, Vitest, jsdom,
Testing Library, axe-core, and their build/test support packages. The exact
resolved dependency graph is recorded in `software/dashboard/pnpm-lock.yaml`.
`node_modules/` and built output are local generated artifacts and must not be
committed.

## Firmware and simulation ecosystems

- ESP32 and Arduino-ESP32 identify the firmware target and toolchain family.
- Raspberry Pi, Picamera2, and libcamera are referenced for the gateway and
  Camera Module 3 Wide / IMX708 stack.
- Waveshare Bus Servo Driver HAT (A), ST3215, and SC09 identify reference
  hardware and compatible protocol families.
- NVIDIA Isaac Sim / Isaac Lab integration source is included, but NVIDIA
  software, assets, and toolchains are not bundled.

Product names and trademarks belong to their respective owners. Their use
describes compatibility and does not imply endorsement.

Before adding a copied source file, CAD model, image, binary, report asset, or
other third-party material, record its component, version, author, source,
license, modifications, and required notices here. Do not infer redistribution
permission from a public download or compatible product name.
