"""Deterministic identity for the backend code and built dashboard it serves."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def build_identity(project_root: Path) -> str:
    root = project_root.resolve()
    candidates: list[Path] = []
    for directory_name in (
        "python/web_backend",
        "python/arm_mcp",
        "dashboard/src",
        "dashboard/dist",
    ):
        directory = root / directory_name
        if not directory.is_dir():
            continue
        candidates.extend(
            path
            for path in directory.rglob("*")
            if path.is_file()
            and "__pycache__" not in path.parts
            and "tests" not in path.relative_to(directory).parts
        )
    for filename in (
        "SOURCE_INDEX.json",
        "python/pyproject.toml",
        "dashboard/index.html",
        "dashboard/package.json",
        "dashboard/pnpm-lock.yaml",
        "dashboard/tsconfig.json",
        "dashboard/tsconfig.app.json",
        "dashboard/tsconfig.node.json",
        "dashboard/vite.config.ts",
    ):
        path = root / filename
        if path.is_file():
            candidates.append(path)
    digest = hashlib.sha256()
    for path in sorted(candidates, key=lambda candidate: candidate.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        content = path.read_bytes()
        digest.update(relative)
        digest.update(b"\0")
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    arguments = parser.parse_args()
    print(build_identity(arguments.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
