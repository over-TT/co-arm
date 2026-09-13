#!/usr/bin/env python3
"""Validate the clean co-arm release tree without third-party dependencies."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path, PurePosixPath
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
MAX_FILE_BYTES = 25 * 1024 * 1024

REQUIRED = {
    "LICENSE",
    "NOTICE",
    "LICENSES/CC-BY-NC-4.0.txt",
    "software/python/LICENSE",
    "software/python/NOTICE",
    ".github/dependabot.yml",
    ".github/workflows/repo-check.yml",
    "README.md",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "SECURITY.md",
    "LICENSING.md",
    "THIRD_PARTY_NOTICES.md",
    "docs/STATUS.md",
    "docs/QUICKSTART.md",
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
    "software/README.md",
    "software/SOURCE_INDEX.json",
    "software/SOURCE_MANIFEST.json",
    "software/dashboard/package.json",
    "software/dashboard/src/arm/ArmControlCenter.tsx",
    "software/firmware/esp32/arm_hat_controller/arm_hat_controller.ino",
    "software/operations/requirements-pi.txt",
    "software/operations/scripts/set-arm-gateway-mode.ps1",
    "software/plugin/.agents/plugins/marketplace.json",
    "software/plugin/plugins/arm-alliance/.codex-plugin/plugin.json",
    "software/plugin/plugins/arm-alliance/skills/arm-alliance-setup/SKILL.md",
    "software/plugin/plugins/arm-alliance/skills/arm-alliance/SKILL.md",
    "software/python/pyproject.toml",
    "software/python/requirements.lock",
    "software/python/tests/installed_wheel_smoke.py",
    "software/python/arm_mcp/config.py",
    "software/python/arm_mcp/server.py",
    "software/python/arm_sim/config/arm_model.json",
    "software/python/robot_gateway/runtime.py",
    "software/python/web_backend/app.py",
}

FORBIDDEN_PARTS = {
    ".git",
    ".claude",
    ".codex",
    ".codex-tmp",
    ".pytest_cache",
    ".venv",
    ".playwright-cli",
    ".pnpm-store",
    ".shots",
    ".ssh",
    "__pycache__",
    "artifacts",
    "backups",
    "build",
    "device-backups",
    "dist",
    "media/raw",
    "media/private",
    "node_modules",
    "output",
    "pi-backups",
    "rendered",
    "reports",
    "runtime",
    "target",
    "toolchains",
    "venv",
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
    ".db",
    ".o",
    ".ndjson",
    ".pem",
    ".pyc",
    ".shm",
    ".sqlite",
    ".sqlite3",
    ".token",
    ".wal",
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
    "opaque camera frame identifier": re.compile(
        r"\bcamera_(?=[A-Za-z0-9_-]{16,}\b)(?=(?:[A-Za-z0-9_-]*\d){6})[A-Za-z0-9_-]+\b"
    ),
    "private key block": re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "OpenAI API key": re.compile(r"\bsk-(?:proj-)?[A-Za-z0-9_-]{20,}\b"),
    "GitHub token": re.compile(r"\b(?:ghp|github_pat)_[A-Za-z0-9_]{20,}\b"),
    "AWS access key": re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
}

MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")
IMAGE_LINK = re.compile(r"!\[[^\]]*\]\(([^)]+)\)")
SOURCE_MANIFEST = "software/SOURCE_MANIFEST.json"
CAPTURE_SUFFIXES = {".jpeg", ".jpg", ".mp4", ".png", ".webp"}

# Durable product-boundary guardrails for features removed from the focused
# co-arm export. Keep these markers specific so ordinary ARM terms remain valid.
FORBIDDEN_SCOPE_MARKERS = {
    "WristLink feature": re.compile(r"(?i)\bWristLink\b"),
    "WatchPanel feature": re.compile(r"\bWatchPanel\b"),
    "watch bridge": re.compile(r"(?i)\b(?:arm_watch|watch_wifi)\b"),
    "embedded agent supervisor": re.compile(r"(?i)\bcodex_supervisor\b"),
    "removed arm experiment": re.compile(r"(?i)\barm_experiment\b"),
    "embedded Arm Chat": re.compile(r"(?i)\bArm[_ -]?Chat\b"),
    "removed Test Lab": re.compile(r"(?i)\bArm Test Lab\b|\bTest Lab\b"),
    "removed can scenario": re.compile(r"(?i)\bcan[_ -](?:drop|tip)\b"),
    "legacy generic simulator": re.compile(r"\bRobotSimulator\b"),
}

ALLOWED_SOFTWARE_TOP_LEVEL = {
    "README.md",
    "SOURCE_INDEX.json",
    "SOURCE_MANIFEST.json",
    "dashboard",
    "firmware",
    "operations",
    "plugin",
    "python",
}

FORBIDDEN_SCOPE_PATH_FRAGMENTS = {
    "arm_can",
    "arm_chat",
    "arm_codex_chat",
    "arm_eval",
    "arm_experiment",
    "arm_watch",
    "benchmark",
    "can_drop",
    "can_tip",
    "codex_supervisor",
    "curriculum",
    "evaluator",
    "experiment_runner",
    "robot_simulator",
    "robotsimulator",
    "src_tauri",
    "test_lab",
    "watch_wifi",
    "watch_panel",
    "watchpanel",
    "waveshare_esp32_s3_lcd_1_69_watch",
    "wristlink",
}

ALLOWED_PYTHON_TOP_LEVEL = {
    "LICENSE",
    "NOTICE",
    "README.md",
    "arm_mcp",
    "arm_sim",
    "pyproject.toml",
    "requirements.lock",
    "robot_gateway",
    "tests",
    "web_backend",
}

SYNTHETIC_SENSITIVE_FIXTURES = {
    (
        "private LAN address",
        "software/python/robot_gateway/tests/test_runtime.py",
        ".".join(("192", "168", "1", "20")),
    ),
}


def relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def release_files() -> list[Path]:
    """Return tracked and unignored release candidates, never local build output."""

    try:
        result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=ROOT,
            check=True,
            capture_output=True,
        )
        candidates: list[Path] = []
        for raw in result.stdout.decode("utf-8").split("\0"):
            if not raw:
                continue
            candidate = (ROOT / raw).resolve()
            if candidate.is_file() and candidate.is_relative_to(ROOT):
                candidates.append(candidate)
        return sorted(set(candidates), key=relative)
    except (OSError, subprocess.SubprocessError, UnicodeDecodeError):
        return sorted(
            (
                path
                for path in ROOT.rglob("*")
                if path.is_file()
                and ".git" not in path.relative_to(ROOT).parts
                and is_forbidden_path(path) is None
            ),
            key=relative,
        )


def is_forbidden_path(path: Path) -> str | None:
    rel = relative(path)
    parts = path.relative_to(ROOT).parts
    lowered_rel = rel.lower()
    lowered_parts = tuple(part.lower() for part in parts)
    lowered_name = path.name.lower()
    if any(part.endswith(".egg-info") for part in lowered_parts):
        return "*.egg-info"
    if "known_hosts" in lowered_name or lowered_name in {
        "authorized_keys",
        "id_dsa",
        "id_ecdsa",
        "id_ed25519",
        "id_rsa",
    }:
        return "SSH identity material"
    for forbidden in FORBIDDEN_PARTS:
        if "/" in forbidden:
            if lowered_rel == forbidden or lowered_rel.startswith(forbidden + "/"):
                return forbidden
        elif forbidden in lowered_parts:
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


def sensitive_match_is_fixture(label: str, path: Path, value: str) -> bool:
    """Allow only reviewed synthetic values at their exact fixture location."""

    return (label, relative(path), value) in SYNTHETIC_SENSITIVE_FIXTURES


def validate_source_manifest(files: list[Path], failures: list[str]) -> None:
    path = ROOT / SOURCE_MANIFEST
    if not path.is_file():
        return
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        records = document["files"]
        if document.get("schema") != "co-arm.source-manifest.v1" or not isinstance(records, list):
            raise ValueError("invalid schema")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        failures.append(f"invalid source manifest: {SOURCE_MANIFEST}")
        return

    expected = {
        relative(candidate): candidate
        for candidate in files
        if relative(candidate).startswith("software/")
        and relative(candidate) != SOURCE_MANIFEST
    }
    recorded: dict[str, tuple[int, str]] = {}
    for record in records:
        if not isinstance(record, dict):
            failures.append(f"invalid source manifest record: {SOURCE_MANIFEST}")
            continue
        name = record.get("path")
        size = record.get("bytes")
        digest = record.get("sha256")
        if (
            not isinstance(name, str)
            or not name.startswith("software/")
            or isinstance(size, bool)
            or not isinstance(size, int)
            or not isinstance(digest, str)
            or not re.fullmatch(r"[0-9a-f]{64}", digest)
            or name in recorded
        ):
            failures.append(f"invalid source manifest record: {SOURCE_MANIFEST}")
            continue
        recorded[name] = (size, digest)

    for name in sorted(expected.keys() - recorded.keys()):
        failures.append(f"software file missing from source manifest: {name}")
    for name in sorted(recorded.keys() - expected.keys()):
        failures.append(f"source manifest names absent software file: {name}")
    for name in sorted(expected.keys() & recorded.keys()):
        candidate = expected[name]
        size, digest = recorded[name]
        payload = candidate.read_bytes()
        if len(payload) != size or hashlib.sha256(payload).hexdigest() != digest:
            failures.append(f"source manifest digest mismatch: {name}")


def validate_source_index(failures: list[str]) -> None:
    path = ROOT / "software" / "SOURCE_INDEX.json"
    if not path.is_file():
        return
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        paths = document["paths"]
        software_root_value = document["softwareRoot"]
        if (
            document.get("schema") != "co-arm.source-index.v2"
            or document.get("canonicalEntry") != "software/SOURCE_INDEX.json"
            or document.get("pathsRelativeTo") != "softwareRoot"
            or not isinstance(software_root_value, str)
            or not isinstance(paths, dict)
        ):
            raise ValueError("invalid schema")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        failures.append("invalid source index: software/SOURCE_INDEX.json")
        return

    def normalized_relative(value: str) -> bool:
        if not value or "\\" in value or ":" in value:
            return False
        parsed = PurePosixPath(value)
        return (
            not parsed.is_absolute()
            and parsed.as_posix() == value
            and all(part not in {"", ".", ".."} for part in parsed.parts)
        )

    if not normalized_relative(software_root_value):
        failures.append("source index softwareRoot must be repository-relative and normalized")
        return
    software_root = (ROOT / software_root_value).resolve()
    if not software_root.is_relative_to(ROOT) or software_root != path.parent.resolve():
        failures.append("source index softwareRoot does not resolve to software/")
        return
    for label, value in sorted(paths.items()):
        if (
            not isinstance(label, str)
            or not label
            or not isinstance(value, str)
            or not normalized_relative(value)
        ):
            failures.append("invalid source index entry: software/SOURCE_INDEX.json")
            continue
        candidate = (software_root / value).resolve()
        if not candidate.is_relative_to(software_root):
            failures.append(f"source index path escapes software root: {label}")
        elif not candidate.exists():
            failures.append(f"source index path is missing: {label} -> {value}")


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
        rel_parts = path.relative_to(ROOT).parts
        if (
            len(rel_parts) >= 2
            and rel_parts[0] == "software"
            and rel_parts[1] not in ALLOWED_SOFTWARE_TOP_LEVEL
        ):
            failures.append(f"software path is outside the ARM allowlist: {rel}")
        if (
            len(rel_parts) >= 3
            and rel_parts[:2] == ("software", "python")
            and rel_parts[2] not in ALLOWED_PYTHON_TOP_LEVEL
        ):
            failures.append(f"Python path is outside the ARM package allowlist: {rel}")
        normalized_rel = re.sub(r"[^a-z0-9]+", "_", rel.lower())
        for fragment in FORBIDDEN_SCOPE_PATH_FRAGMENTS:
            if fragment in normalized_rel:
                failures.append(
                    f"non-ARM scope path ({fragment}): {rel}"
                )
                break
        if path.is_symlink():
            failures.append(f"symbolic links are not allowed in the release: {rel}")
        forbidden = is_forbidden_path(path)
        if forbidden:
            failures.append(f"forbidden release path ({forbidden}): {rel}")

        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            failures.append(
                f"file exceeds {MAX_FILE_BYTES // (1024 * 1024)} MiB release cap: {rel} ({size} bytes)"
            )

        if path.suffix.lower() in CAPTURE_SUFFIXES and not rel.startswith("media/"):
            failures.append(f"captured media must be reviewed under media/: {rel}")

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

        if "\r\n" in text:
            failures.append(f"release text must use LF line endings: {rel}")

        for label, pattern in SENSITIVE_PATTERNS.items():
            for match in pattern.finditer(text):
                if sensitive_match_is_fixture(label, path, match.group(0)):
                    continue
                line = text.count("\n", 0, match.start()) + 1
                failures.append(f"{label}: {rel}:{line}")

        if rel != "tools/check_repo.py":
            for label, pattern in FORBIDDEN_SCOPE_MARKERS.items():
                match = pattern.search(text)
                if match is not None:
                    line = text.count("\n", 0, match.start()) + 1
                    failures.append(f"non-ARM scope marker ({label}): {rel}:{line}")

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

    validate_source_index(failures)
    validate_source_manifest(files, failures)

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
