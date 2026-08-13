#!/usr/bin/env python3
"""Validate the clean co-arm release tree without third-party dependencies."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
MAX_FILE_BYTES = 25 * 1024 * 1024

REQUIRED = {
    "README.md",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "SECURITY.md",
    "LICENSING.md",
    "THIRD_PARTY_NOTICES.md",
    "docs/STATUS.md",
    "docs/SETUP_WITH_CODEX.md",
    "docs/CONTROL_AND_SAFETY.md",
    "docs/HARDWARE.md",
    "docs/ELECTRONICS.md",
    "docs/MECHANICAL.md",
    "docs/CAD_AND_STL.md",
    "docs/AI_CONTEXT.md",
    "docs/VISION.md",
    "docs/HISTORY.md",
    "docs/MEDIA_GUIDE.md",
    "docs/RELEASE_CHECKLIST.md",
    "hardware/bom/bom.csv",
    "media/README.md",
}

FORBIDDEN_PARTS = {
    ".git",
    ".claude",
    ".codex-tmp",
    ".playwright-cli",
    ".pnpm-store",
    ".shots",
    "__pycache__",
    "artifacts",
    "device-backups",
    "dist",
    "media/raw",
    "media/private",
    "node_modules",
    "output",
    "runtime",
    "target",
    "toolchains",
}

FORBIDDEN_SUFFIXES = {
    ".a",
    ".bin",
    ".eep",
    ".elf",
    ".hex",
    ".key",
    ".log",
    ".map",
    ".o",
    ".pem",
    ".pyc",
    ".token",
    ".uf2",
}

# These patterns detect actual-looking installation data, not prose explaining
# that such data must be kept private.
SENSITIVE_PATTERNS = {
    "personal Windows path": re.compile(
        r"(?i)\b[A-Z]:[\\/]+Users[\\/]+(?!example\b|your[-_ ]?name\b|username\b)[^\\/\s]+"
    ),
    "MSYS personal path": re.compile(
        r"(?i)(?<![A-Za-z0-9])/[a-z]/Users/(?!example\b|your[-_ ]?name\b|username\b)[^/\s]+"
    ),
    "private LAN address": re.compile(
        r"(?<!\d)(?:10\.\d{1,3}\.\d{1,3}\.\d{1,3}|192\.168\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})(?!\d)"
    ),
    "serial port identifier": re.compile(r"(?i)\bCOM\d+\b"),
    "USB hardware identifier": re.compile(r"(?i)USB[\\/]+VID_[0-9A-F]{4}"),
    "reference controller identifier": re.compile(r"(?i)\barmhat-[0-9a-f]{12,}\b"),
    "reference boot identifier": re.compile(r"(?i)\bboot-[0-9a-f]{12,}\b"),
    "opaque camera frame identifier": re.compile(r"\bcamera_[A-Za-z0-9_-]{16,}\b"),
    "private key block": re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "OpenAI API key": re.compile(r"\bsk-(?:proj-)?[A-Za-z0-9_-]{20,}\b"),
    "GitHub token": re.compile(r"\b(?:ghp|github_pat)_[A-Za-z0-9_]{20,}\b"),
    "AWS access key": re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
}

MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")
IMAGE_LINK = re.compile(r"!\[[^\]]*\]\(([^)]+)\)")


def relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def release_files() -> list[Path]:
    return sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(ROOT).parts
    )


def is_forbidden_path(path: Path) -> str | None:
    rel = relative(path)
    parts = path.relative_to(ROOT).parts
    for forbidden in FORBIDDEN_PARTS:
        if "/" in forbidden:
            if rel == forbidden or rel.startswith(forbidden + "/"):
                return forbidden
        elif forbidden in parts:
            return forbidden
    if path.suffix.lower() in FORBIDDEN_SUFFIXES:
        return path.suffix.lower()
    return None


def read_text(path: Path) -> str | None:
    data = path.read_bytes()
    if b"\x00" in data:
        return None
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


def local_link_target(source: Path, raw_target: str) -> Path | None:
    target = raw_target.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1]
    target = target.split(maxsplit=1)[0]
    if not target or target.startswith(("#", "http://", "https://", "mailto:")):
        return None
    target = unquote(target.split("#", 1)[0].split("?", 1)[0])
    if not target:
        return None
    return (source.parent / target).resolve()


def main() -> int:
    failures: list[str] = []
    warnings: list[str] = []
    files = release_files()

    present = {relative(path) for path in files}
    for required in sorted(REQUIRED - present):
        failures.append(f"missing required file: {required}")

    for path in files:
        rel = relative(path)
        forbidden = is_forbidden_path(path)
        if forbidden:
            failures.append(f"forbidden release path ({forbidden}): {rel}")

        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            failures.append(
                f"file exceeds {MAX_FILE_BYTES // (1024 * 1024)} MiB release cap: {rel} ({size} bytes)"
            )

        text = read_text(path)
        if text is None:
            if path.suffix.lower() not in {
                ".3mf",
                ".f3d",
                ".fcstd",
                ".jpeg",
                ".jpg",
                ".mp4",
                ".pdf",
                ".png",
                ".step",
                ".stl",
                ".webp",
            }:
                warnings.append(f"unreviewed binary file: {rel}")
            continue

        for label, pattern in SENSITIVE_PATTERNS.items():
            for match in pattern.finditer(text):
                line = text.count("\n", 0, match.start()) + 1
                failures.append(f"{label}: {rel}:{line}")

        if path.suffix.lower() == ".md":
            for pattern in (MARKDOWN_LINK, IMAGE_LINK):
                for match in pattern.finditer(text):
                    target = local_link_target(path, match.group(1))
                    if target is None:
                        continue
                    try:
                        target.relative_to(ROOT)
                    except ValueError:
                        failures.append(
                            f"Markdown link escapes repository: {rel} -> {match.group(1)}"
                        )
                        continue
                    if not target.exists():
                        line = text.count("\n", 0, match.start()) + 1
                        failures.append(
                            f"broken local Markdown link: {rel}:{line} -> {match.group(1)}"
                        )

    if not (ROOT / "LICENSE").exists():
        warnings.append(
            "LICENSE is not selected; LICENSING.md intentionally records the closed/default state"
        )
    if not any(path.suffix.lower() in {".step", ".stl", ".3mf", ".f3d", ".fcstd"} for path in files):
        warnings.append("no CAD/STEP/STL assets are present yet")
    if not any(path.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp", ".mp4"} for path in files):
        warnings.append("no approved photos or videos are present yet")

    for warning in warnings:
        print(f"WARN: {warning}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        print(f"\nRepository check failed: {len(failures)} issue(s).")
        return 1

    print(f"Repository check passed: {len(files)} files reviewed, {len(warnings)} warning(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
