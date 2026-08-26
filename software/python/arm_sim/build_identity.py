"""Deterministic identity for the source that defines the local Arm simulator.

The digest deliberately excludes generated USDs, captures, logs, caches, and
tests.  A supervisor captures it before launching its children and exposes the
value through its authenticated control socket.  A later launcher recomputes
the digest from disk, so it cannot silently reuse a stack whose executable
source or authored simulator inputs have changed.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


_SCHEMA = b"arm-sim-source.v1\0"
_ARM_SIM_SUFFIXES = frozenset(
    {
        ".dae",
        ".ini",
        ".json",
        ".mtl",
        ".obj",
        ".py",
        ".stl",
        ".toml",
        ".urdf",
        ".usd",
        ".usda",
        ".usdc",
        ".yaml",
        ".yml",
    }
)
_EXCLUDED_ARM_SIM_DIRECTORIES = frozenset({"__pycache__", "generated", "tests"})
_EXCLUDED_ROBOT_GATEWAY_DIRECTORIES = frozenset({"__pycache__", "tests"})
_REQUIRED_EXTERNAL_SOURCES = (
    Path("operations/scripts/run_arm_sim.py"),
    Path("operations/requirements-pi.txt"),
)


def simulator_source_files(project_root: Path) -> tuple[Path, ...]:
    """Return the canonical, sorted source set for one simulator build."""

    root = project_root.resolve()
    arm_sim_root = root / "python" / "arm_sim"
    if not arm_sim_root.is_dir():
        raise FileNotFoundError("Arm simulator source directory is unavailable.")

    selected: set[Path] = set()
    for candidate in arm_sim_root.rglob("*"):
        if not candidate.is_file():
            continue
        relative_to_arm_sim = candidate.relative_to(arm_sim_root)
        if any(
            part in _EXCLUDED_ARM_SIM_DIRECTORIES
            for part in relative_to_arm_sim.parts[:-1]
        ):
            continue
        if candidate.suffix.lower() in _ARM_SIM_SUFFIXES:
            selected.add(candidate.resolve())

    robot_gateway_root = root / "python" / "robot_gateway"
    if not robot_gateway_root.is_dir():
        raise FileNotFoundError("Robot gateway source directory is unavailable.")
    for candidate in robot_gateway_root.rglob("*.py"):
        if not candidate.is_file():
            continue
        relative_to_gateway = candidate.relative_to(robot_gateway_root)
        if any(
            part in _EXCLUDED_ROBOT_GATEWAY_DIRECTORIES
            for part in relative_to_gateway.parts[:-1]
        ):
            continue
        selected.add(candidate.resolve())

    for relative in _REQUIRED_EXTERNAL_SOURCES:
        candidate = (root / relative).resolve()
        if not candidate.is_relative_to(root) or not candidate.is_file():
            raise FileNotFoundError(f"Required simulator source is unavailable: {relative.as_posix()}")
        selected.add(candidate)

    if not selected:
        raise FileNotFoundError("No Arm simulator source files were found.")
    return tuple(sorted(selected, key=lambda path: path.relative_to(root).as_posix()))


def simulator_source_digest(project_root: Path) -> str:
    """Hash canonical relative paths and exact bytes into a stable identity."""

    root = project_root.resolve()
    digest = hashlib.sha256(_SCHEMA)
    for path in simulator_source_files(root):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        payload = path.read_bytes()
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    return f"sha256:{digest.hexdigest()}"


__all__ = ["simulator_source_digest", "simulator_source_files"]
