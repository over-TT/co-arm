"""Prepare calibration-only recovery artifacts from a verified protected archive.

This never extracts an archive tree, restores credentials, or claims that an
archived Base frame remains physically valid. Only exact frozen calibration
bytes are retained; the mandatory marker requires measured physical re-zero.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
import tarfile

CALIBRATION_MEMBER = "var/lib/arm-gateway/arm-joints.json"
MAX_MEMBER_BYTES = 2 * 1024 * 1024


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def strict_json(data: bytes) -> object:
    def unique(pairs: list[tuple[str, object]]) -> dict:
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON field")
            result[key] = value
        return result

    def invalid_constant(_: str) -> None:
        raise ValueError("Non-finite JSON number")

    return json.loads(data.decode("utf-8"), object_pairs_hook=unique,
                      parse_constant=invalid_constant)


def validate_calibration(data: bytes) -> None:
    document = strict_json(data)
    names = {f"joint_{index}" for index in range(1, 5)}
    if not isinstance(document, dict) or set(document) != names:
        raise ValueError("Calibration must contain exactly four joints")
    integers = {"servoId", "rawZero", "rawMin", "rawMax", "direction", "speed", "accel"}
    for index in range(1, 5):
        joint = document[f"joint_{index}"]
        if not isinstance(joint, dict) or not integers | {"ratio"} <= joint.keys():
            raise ValueError("Calibration joint fields are incomplete")
        if any(type(joint[key]) is not int for key in integers):
            raise ValueError("Calibration requires JSON integers")
        ratio = joint["ratio"]
        if type(ratio) not in (int, float) or not math.isfinite(ratio) or not 0.01 < ratio <= 64:
            raise ValueError("Calibration ratio is invalid")
        if (joint["servoId"] != index or joint["direction"] not in (-1, 1)
                or not 1 <= joint["speed"] <= 2400 or not 1 <= joint["accel"] <= 50):
            raise ValueError("Calibration joint contract is invalid")
        lower, upper = (-30719, 30719) if index == 1 else (0, 1023 if index == 4 else 4095)
        if any(not lower <= joint[key] <= upper for key in ("rawMin", "rawZero", "rawMax")):
            raise ValueError("Calibration is outside the controller wire range")
        endpoints = sorted((joint["rawMin"], joint["rawMax"]))
        if endpoints[0] == endpoints[1] or not endpoints[0] <= joint["rawZero"] <= endpoints[1]:
            raise ValueError("Calibration zero must lie within distinct endpoints")


def safe_member_name(name: str) -> str:
    if name.startswith("./"):
        name = name[2:]
    if (not name or name.startswith("/") or "\\" in name or ":" in name
            or any(ord(char) < 32 for char in name)
            or any(part in ("", ".", "..") for part in name.rstrip("/").split("/"))):
        raise ValueError("Unsafe archive member path")
    return str(PurePosixPath(name))


def reject_links(path: Path) -> None:
    # Windows junctions and symlinks may exist above an otherwise regular file.
    for component in (path, *path.parents):
        try:
            info = component.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError("Recovery inputs and outputs cannot use links or reparse points")


def checksum_rows(data: bytes) -> dict[str, str]:
    checksums = {}
    for row in data.decode("utf-8").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", row)
        if match is None:
            raise ValueError("Malformed internal checksum row")
        name = safe_member_name(match[2])
        if name in checksums:
            raise ValueError("Duplicate internal checksum row")
        checksums[name] = match[1]
    return checksums


def read_calibration(archive: Path, expected_sha256: str, *,
                     expected_host: str | None = None, expected_user: str | None = None) -> bytes:
    if not re.fullmatch(r"[0-9a-f]{64}", expected_sha256):
        raise ValueError("An exact lowercase archive SHA256 is required")
    reject_links(archive)
    if bool(expected_host) != bool(expected_user):
        raise ValueError("Expected archive hostname and user must be supplied together")
    requested = {CALIBRATION_MEMBER, "metadata/SHA256SUMS"}
    if expected_host:
        requested.update(("metadata/snapshot.tsv", "metadata/METADATA_SHA256SUMS"))
    with archive.open("rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode):
            raise ValueError("Recovery archive must be a regular file")
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != expected_sha256:
            raise ValueError("Recovery archive SHA256 mismatch")
        stream.seek(0)
        selected: dict[str, bytes] = {}
        seen = set()
        with tarfile.open(fileobj=stream, mode="r:gz") as tar:
            for count, member in enumerate(tar, 1):
                if count > 100000:
                    raise ValueError("Recovery archive contains too many entries")
                # GNU tar --directory snapshot . emits a harmless root entry.
                if member.name in (".", "./") and member.isdir():
                    if "." in seen:
                        raise ValueError("Duplicate archive root directory")
                    seen.add(".")
                    continue
                name = safe_member_name(member.name)
                if name in seen:
                    raise ValueError("Duplicate archive member")
                seen.add(name)
                if not member.isfile() and not member.isdir():
                    raise ValueError("Recovery archive cannot contain links or special files")
                if name not in requested:
                    continue
                if not member.isfile() or not 0 < member.size <= MAX_MEMBER_BYTES:
                    raise ValueError("Selected recovery member has an invalid size or type")
                extracted = tar.extractfile(member)
                if extracted is None:
                    raise ValueError("Cannot read selected recovery member")
                selected[name] = extracted.read(MAX_MEMBER_BYTES + 1)
        stream.seek(0)
        if hashlib.file_digest(stream, "sha256").hexdigest() != expected_sha256:
            raise ValueError("Recovery archive changed during verification")
    if set(selected) != requested:
        raise ValueError("Recovery archive is missing calibration, identity metadata, or internal checksums")
    checksums = checksum_rows(selected["metadata/SHA256SUMS"])
    if expected_host:
        metadata_checksums = checksum_rows(selected["metadata/METADATA_SHA256SUMS"])
        for name in ("metadata/snapshot.tsv", "metadata/SHA256SUMS"):
            if metadata_checksums.get(name) != sha256(selected[name]):
                raise ValueError("Archive identity metadata checksum mismatch")
        identity = {}
        for row in selected["metadata/snapshot.tsv"].decode("utf-8").splitlines():
            key, separator, value = row.partition("\t")
            if not separator or key in identity or "\t" in value:
                raise ValueError("Malformed archive identity metadata")
            identity[key] = value
        if (identity.get("schema") not in ("arm-pi-recovery-backup.v1", "arm-pi-recovery-backup.v2",
                                           "arm-pi-recovery-backup.v3")
                or identity.get("hostname") != expected_host or identity.get("user") != expected_user):
            raise ValueError("Recovery archive belongs to a different hostname or user")
    data = selected[CALIBRATION_MEMBER]
    if checksums.get(CALIBRATION_MEMBER) != sha256(data):
        raise ValueError("Calibration does not match its internal archive checksum")
    validate_calibration(data)
    return data


def encoded(document: dict) -> bytes:
    return (json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def protect_outputs(root: Path, names: tuple[str, ...], *, mutate: bool) -> None:
    """Give only the current account access, including reused output files."""
    paths = [root, *(root / name for name in names if (root / name).exists())]
    for path in paths:
        reject_links(path)
    if os.name != "nt":
        for path in paths:
            expected_mode = 0o700 if path.is_dir() else 0o600
            if path.stat().st_uid != os.getuid():
                raise ValueError("Recovery output belongs to a different account")
            if mutate:
                os.chmod(path, expected_mode)
            elif stat.S_IMODE(path.stat().st_mode) != expected_mode or path.stat().st_uid != os.getuid():
                raise ValueError("Recovery output permissions are not owner-only")
        return
    environment = dict(os.environ)
    environment["ARM_RECOVERY_PRIVATE_PATHS_JSON"] = json.dumps([str(path) for path in paths])
    environment["ARM_RECOVERY_SET_PRIVATE_PERMISSIONS"] = "yes" if mutate else "no"
    windows_root = os.environ.get("SystemRoot")
    if not windows_root:
        raise ValueError("Windows system directory is unavailable for permission verification")
    powershell = Path(windows_root) / "System32/WindowsPowerShell/v1.0/powershell.exe"
    command = r"""
$ErrorActionPreference = 'Stop'
$sid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User
if ($null -eq $sid) { throw 'Current account SID is unavailable.' }
foreach ($path in (ConvertFrom-Json $env:ARM_RECOVERY_PRIVATE_PATHS_JSON)) {
    $directory = [System.IO.Directory]::Exists($path)
    $security = if ($directory) {
        New-Object System.Security.AccessControl.DirectorySecurity
    } else { New-Object System.Security.AccessControl.FileSecurity }
    $security.SetAccessRuleProtection($true, $false)
    $inheritance = if ($directory) {
        [System.Security.AccessControl.InheritanceFlags]::ContainerInherit -bor
        [System.Security.AccessControl.InheritanceFlags]::ObjectInherit
    } else { [System.Security.AccessControl.InheritanceFlags]::None }
    $rule = New-Object System.Security.AccessControl.FileSystemAccessRule(
        $sid, [System.Security.AccessControl.FileSystemRights]::FullControl,
        $inheritance, [System.Security.AccessControl.PropagationFlags]::None,
        [System.Security.AccessControl.AccessControlType]::Allow)
    [void]$security.AddAccessRule($rule)
    $sections = [System.Security.AccessControl.AccessControlSections]::Access -bor
        [System.Security.AccessControl.AccessControlSections]::Owner
    if ($directory) {
        if ($env:ARM_RECOVERY_SET_PRIVATE_PERMISSIONS -eq 'yes') {
            [System.IO.Directory]::SetAccessControl($path, $security)
        }
        $actual = [System.IO.Directory]::GetAccessControl($path, $sections)
    } else {
        if ($env:ARM_RECOVERY_SET_PRIVATE_PERMISSIONS -eq 'yes') {
            [System.IO.File]::SetAccessControl($path, $security)
        }
        $actual = [System.IO.File]::GetAccessControl($path, $sections)
    }
    $rules = @($actual.GetAccessRules($true, $true, [System.Security.Principal.SecurityIdentifier]))
    if (-not $actual.AreAccessRulesProtected -or $rules.Count -ne 1 -or
        $rules[0].IdentityReference.Value -cne $sid.Value -or
        $rules[0].FileSystemRights -ne [System.Security.AccessControl.FileSystemRights]::FullControl -or
        $rules[0].AccessControlType -ne [System.Security.AccessControl.AccessControlType]::Allow -or
        $actual.GetOwner([System.Security.Principal.SecurityIdentifier]).Value -cne $sid.Value) {
        throw 'Recovery output is not owner-only.'
    }
}
"""
    completed = subprocess.run([str(powershell), "-NoLogo", "-NoProfile", "-NonInteractive",
                                "-Command", command], env=environment, capture_output=True,
                               text=True, check=False, timeout=30)
    if completed.returncode:
        raise ValueError("Could not apply and verify owner-only recovery output permissions")


def prepare(archive: Path, archive_sha256: str, output_root: Path, *, create: bool,
            calibration: Path | None = None, provenance: Path | None = None,
            expected_host: str | None = None, expected_user: str | None = None) -> dict:
    data = read_calibration(archive, archive_sha256,
                            expected_host=expected_host, expected_user=expected_user)
    digest = sha256(data)
    if bool(calibration) != bool(provenance):
        raise ValueError("Legacy calibration and provenance must be supplied together")
    if calibration is not None and provenance is not None:
        reject_links(calibration)
        reject_links(provenance)
        # The archive is authoritative. Never bless a stale separately copied pair.
        supplied = strict_json(provenance.read_bytes())
        if (calibration.read_bytes() != data or not isinstance(supplied, dict)
                or supplied.get("schema") not in ("arm-joints-recovery-provenance.v1",
                                                  "arm-joints-recovery-provenance.v2")
                or not isinstance(supplied.get("artifact"), dict)
                or supplied["artifact"].get("sha256") != digest
                or type(supplied["artifact"].get("jointCount")) is not int
                or supplied["artifact"]["jointCount"] != 4):
            raise ValueError("Supplied calibration/provenance does not match the frozen archive")
    root = output_root.absolute() / ("archive-" + archive_sha256)
    reject_links(root)
    if create:
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
    elif not root.is_dir():
        raise ValueError("Recovery artifacts are absent; run laptop-only Prepare first")
    output_names = ("arm-joints.json", "arm-joints.provenance.json", "base-reference-required.json")
    protect_outputs(root, output_names, mutate=create)
    recovery = {
        "sourceKind": "verified-protected-pi-backup",
        "sourceArchiveSha256": archive_sha256,
        "sourceMemberPath": CALIBRATION_MEMBER,
        "sourceMemberSha256": digest,
        "baseContinuityClaimed": False,
        "physicalBaseRezeroRequired": True,
    }
    payloads = {
        "arm-joints.json": data,
        "arm-joints.provenance.json": encoded({
            "schema": "arm-joints-recovery-provenance.v2",
            "artifact": {"sha256": digest, "jointCount": 4}, "recovery": recovery,
        }),
        "base-reference-required.json": encoded({
            "schema": "arm-base-reference-required.v1",
            "baseReferenceRequired": True,
            "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
            "sourceArchiveSha256": archive_sha256,
            "calibrationSha256": digest,
        }),
    }
    for name, payload in payloads.items():
        target = root / name
        reject_links(target)
        if target.exists():
            if not target.is_file() or target.read_bytes() != payload:
                raise ValueError("Prepared recovery artifact differs from the selected archive")
        elif create:
            with target.open("xb") as output:
                output.write(payload)
                output.flush()
                os.fsync(output.fileno())
            os.chmod(target, 0o600)
        else:
            raise ValueError("Recovery artifact is absent; run laptop-only Prepare first")
    if create:
        protect_outputs(root, output_names, mutate=True)
    return {
        "calibrationPath": str(root / "arm-joints.json"),
        "provenancePath": str(root / "arm-joints.provenance.json"),
        "baseReferenceMarkerPath": str(root / "base-reference-required.json"),
        "archiveSha256": archive_sha256,
        "calibrationSha256": digest,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--sha256", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--create", action="store_true")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--provenance", type=Path)
    parser.add_argument("--expected-host")
    parser.add_argument("--expected-user")
    args = parser.parse_args()
    try:
        result = prepare(args.archive, args.sha256, args.output_root, create=args.create,
                         calibration=args.calibration, provenance=args.provenance,
                         expected_host=args.expected_host, expected_user=args.expected_user)
    except (OSError, ValueError, tarfile.TarError, subprocess.SubprocessError) as error:
        # Errors describe contracts, never token content or calibration values.
        print(f"Recovery preparation refused: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
