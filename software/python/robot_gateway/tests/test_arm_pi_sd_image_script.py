from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "operations" / "scripts" / "image-arm-pi-sd.ps1"


def _bash_path() -> str | None:
    discovered = shutil.which("bash")
    if discovered:
        return discovered
    git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
    return str(git_bash) if git_bash.is_file() else None


def test_sd_image_script_is_fixed_to_an_offline_sd_and_pinned_transport() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    for required in (
        "[Parameter(Mandatory)]",
        "[string] $ExpectedHostName",
        "[string] $ExpectedRootDevice",
        "[string] $ExpectedBootDevice",
        "$targetDevice = '/dev/mmcblk0'",
        "[ \"$root_source\" = \"$expected_root\" ]",
        "[ \"$boot_source\" = \"$expected_boot\" ]",
        "case \"$pi_model\" in Raspberry\\ Pi*",
        "[ \"$actual_user\" = \"$expected_user\" ]",
        "lsblk -nrpo MOUNTPOINTS",
        "[ \"$target\" != \"$root_disk\" ]",
        "[ \"$target\" != \"$boot_disk\" ]",
        "StrictHostKeyChecking=yes",
        "GlobalKnownHostsFile=",
        "IdentitiesOnly=yes",
        "BatchMode=yes",
        "PasswordAuthentication=no",
        "KbdInteractiveAuthentication=no",
        "UpdateHostKeys=no",
        "CheckHostIP=yes",
        "$IdentityAnchorFile",
        "arm-pi-replacement-sd-identity.v1",
        "^[0-9a-f]{32}$",
        "/sys/class/block/mmcblk0/device/cid",
        "TARGET_P1_PARTUUID_B64",
        "TARGET_P2_PARTUUID_B64",
    ):
        assert required in source

    for forbidden in (
        "StrictHostKeyChecking=no",
        "ssh-keygen -R",
        "sudo reboot",
        "systemctl reboot",
        "shutdown -r",
        "dd of=",
        "mkfs",
        "parted",
        "fdisk",
        "arm_move",
        "clear_stop",
    ):
        assert forbidden not in source


def test_sd_image_script_streams_validates_hashes_and_records_proof() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    for required in (
        "set -euo pipefail",
        "sudo dd if=/dev/mmcblk0 bs=4M iflag=fullblock status=none | gzip -1 -c",
        "arm-pi-sd-mmcblk0.img.gz",
        '"$finalImagePath.partial"',
        "System.IO.Compression.GZipStream",
        "Get-FileHash -LiteralPath $partialImagePath -Algorithm SHA256",
        "RawImageSha256",
        "DecompressedBytes",
        "Move-Item -LiteralPath $partialImagePath -Destination $finalImagePath",
        "runtime\\robot-gateway\\pi-disk-images",
        "Set-PrivateDirectory -Path $imageRoot",
        "Set-PrivateDirectory -Path $imageDirectory",
        "Remove-IncompleteImageDirectory",
        "lsblkIdentity",
        "lsblkTree",
        "targetIdentityRecheckedImmediatelyBeforeStream = $true",
        "targetIdentityRecheckedAfterStream = $true",
        "kernelWholeDeviceReadOnlyVerifiedImmediatelyBeforeRead = $true",
        "originalKernelReadOnlyStateRestoredBeforeSuccess = $true",
        "gzipValidatedToEofLocally = $true",
        "decompressedByteCountMatchesSource = $true",
        "remoteWritesToSource = $false",
        "secretContentsPrinted = $false",
        "[long] $freeSpaceMarginBytes = 1GB",
        "destinationFreeSpaceCheckedBeforeStream = $true",
        "destinationRequiredFreeBytes = $requiredFreeBytes",
        "destinationAvailableFreeBytesAtCheck = $availableFreeBytes",
    ):
        assert required in source

    assert "raw:$finalImageName" not in source


def test_stream_fails_closed_and_holds_a_trap_restored_kernel_ro_lock() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for required in (
        'root_source=$(findmnt -n -o SOURCE -- /) || refuse',
        'boot_source=$(findmnt -n -o SOURCE -- /boot/firmware) || refuse',
        'target_type=$(lsblk -dn -o TYPE -- "$target") || refuse',
        'mount_rows=$(lsblk -nrpo MOUNTPOINTS -- "$target") || refuse',
        '-o PATH,KNAME,TYPE,SIZE,MODEL,SERIAL,WWN,PTUUID,MAJ:MIN,RM -- "$target") || refuse',
        'actual_size=$(sudo blockdev --getsize64 "$target") || refuse',
        'actual_p1_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p1) || refuse',
        'actual_p2_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p2) || refuse',
        'trap restore_kernel_ro EXIT',
        'sudo blockdev --setro "$target" || refuse',
        'if ! sudo blockdev --setrw "$target"',
        'locked_ro=$(sudo blockdev --getro "$target") || refuse',
        "assert_target_identity\nrequire_unmounted",
        "remainingProofLimit =",
    ):
        assert required in source

    stream_start = source.index("$streamScript = @'")
    first_assert = source.index("assert_target_identity", source.index("# First gate", stream_start))
    lock = source.index('sudo blockdev --setro "$target"', first_assert)
    read = source.index("sudo dd if=/dev/mmcblk0", lock)
    post_assert = source.index("assert_target_identity", read)
    assert first_assert < lock < read < post_assert


def test_preflight_disk_and_mount_queries_also_fail_closed() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for required in (
        'root_source=$(findmnt -n -o SOURCE -- /) || identity_failure',
        'boot_source=$(findmnt -n -o SOURCE -- /boot/firmware) || identity_failure',
        'target_type=$(lsblk -dn -o TYPE -- "$target") || target_failure',
        'parent=$(lsblk -dn -o PKNAME -- "$source") || return 1',
        'mount_rows=$(lsblk -nrpo MOUNTPOINTS -- "$target") || target_failure',
        '-o PATH,KNAME,TYPE,SIZE,MODEL,SERIAL,WWN,PTUUID,MAJ:MIN,RM -- "$target") || exit 44',
        '-o PATH,TYPE,SIZE,FSTYPE,UUID,PARTUUID,MOUNTPOINTS -- "$target") || exit 44',
        'field_value=$(LC_ALL=C lsblk -bdnr -o "$1" -- "$target") || return 1',
    ):
        assert required in source


def test_identity_anchor_rejects_a_64_hex_cid_before_ssh(tmp_path: Path) -> None:
    shell = shutil.which("powershell") or shutil.which("powershell.exe")
    if shell is None:
        pytest.skip("Windows PowerShell is not available")

    identity = tmp_path / "identity"
    known_hosts = tmp_path / "known-hosts"
    anchor = tmp_path / "replacement-sd-identity.json"
    identity.write_text("test-only", encoding="utf-8")
    known_hosts.write_text("test-only", encoding="utf-8")
    anchor.write_text(
        json.dumps(
            {
                "schema": "arm-pi-replacement-sd-identity.v1",
                "device": "/dev/mmcblk0",
                "cid": "0" * 64,
                "sizeBytes": 15931539456,
                "partitions": {
                    "mmcblk0p1": "0" * 8 + "-01",
                    "mmcblk0p2": "0" * 8 + "-02",
                },
            }
        ),
        encoding="utf-8",
    )
    result = subprocess.run(
        [
            shell,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(SCRIPT),
            "-HostName",
            "127.0.0.1",
            "-UserName",
            "armtest",
            "-IdentityFile",
            str(identity),
            "-KnownHostsFile",
            str(known_hosts),
            "-IdentityAnchorFile",
            str(anchor),
            "-ExpectedHostName",
            "arm-test-host",
            "-ExpectedRootDevice",
            "/dev/test-root",
            "-ExpectedBootDevice",
            "/dev/test-boot",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    assert result.returncode != 0
    assert "exactly 32 lowercase hexadecimal" in (result.stdout + result.stderr)


def test_identity_anchor_accepts_the_captured_replacement_sd_shape(tmp_path: Path) -> None:
    shell = shutil.which("powershell") or shutil.which("powershell.exe")
    if shell is None:
        pytest.skip("Windows PowerShell is not available")
    anchor = tmp_path / "replacement-sd-identity.json"
    example_cid = "0" * 31 + "1"
    example_p1 = "0" * 8 + "-01"
    example_p2 = "0" * 8 + "-02"
    anchor.write_text(
        json.dumps(
            {
                "schema": "arm-pi-replacement-sd-identity.v1",
                "capturedAtUtc": "2026-08-22T16:27:39.2677689Z",
                "device": "/dev/mmcblk0",
                "cid": example_cid,
                "sizeBytes": 15931539456,
                "partitions": {
                    "mmcblk0p1": example_p1,
                    "mmcblk0p2": example_p2,
                },
            }
        ),
        encoding="utf-8",
    )
    script_literal = str(SCRIPT).replace("'", "''")
    anchor_literal = str(anchor).replace("'", "''")
    command = (
        f"$source=Get-Content -Raw -LiteralPath '{script_literal}'; "
        "$start=$source.IndexOf('function Read-ReplacementSdIdentityAnchor'); "
        "$finish=$source.IndexOf('function Read-GzipImageEvidence',$start); "
        "if($start -lt 0 -or $finish -le $start){exit 2}; "
        "Invoke-Expression $source.Substring($start,$finish-$start); "
        f"$a=Read-ReplacementSdIdentityAnchor -Path '{anchor_literal}'; "
        f"if($a.Cid -cne '{example_cid}' -or "
        "$a.SizeBytes -ne 15931539456 -or "
        f"$a.P1Partuuid -cne '{example_p1}' -or "
        f"$a.P2Partuuid -cne '{example_p2}'){{exit 3}}"
    )
    result = subprocess.run(
        [shell, "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", command],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("variable", ["preflightScript", "streamScript"])
def test_remote_image_shell_payloads_parse(variable: str) -> None:
    shell = _bash_path()
    if shell is None:
        pytest.skip("bash is not available")
    source = SCRIPT.read_text(encoding="utf-8")
    match = re.search(rf"\${variable}\s*=\s*@'\r?\n(.*?)\r?\n'@", source, re.S)
    assert match is not None
    result = subprocess.run(
        [shell, "-n"],
        input=match.group(1),
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_boot_identity_and_transport_files_are_explicit_parameters() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for name in (
        "$IdentityFile",
        "$KnownHostsFile",
        "$IdentityAnchorFile",
        "$ExpectedHostName",
        "$ExpectedRootDevice",
        "$ExpectedBootDevice",
    ):
        assert name in source
    assert "codex_arm_pi" not in source
    assert "pi-known-hosts" not in source


def test_unsafe_host_is_rejected_with_all_explicit_inputs(tmp_path: Path) -> None:
    shell = shutil.which("powershell") or shutil.which("powershell.exe")
    if shell is None:
        pytest.skip("Windows PowerShell is not available")

    identity = tmp_path / "identity"
    known_hosts = tmp_path / "known-hosts"
    anchor = tmp_path / "identity-anchor.json"
    identity.write_text("test-only", encoding="utf-8")
    known_hosts.write_text("test-only", encoding="utf-8")
    anchor.write_text("{}\n", encoding="utf-8")
    result = subprocess.run(
        [
            shell,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(SCRIPT),
            "-HostName",
            "arm-pi;touch-bad",
            "-UserName",
            "armtest",
            "-IdentityFile",
            str(identity),
            "-KnownHostsFile",
            str(known_hosts),
            "-IdentityAnchorFile",
            str(anchor),
            "-ExpectedHostName",
            "arm-test-host",
            "-ExpectedRootDevice",
            "/dev/test-root",
            "-ExpectedBootDevice",
            "/dev/test-boot",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )

    assert result.returncode != 0
    combined = result.stdout + result.stderr
    assert "HostName must be" in combined
    assert "Join-Path" not in combined


def test_sd_image_script_parses_as_powershell() -> None:
    shell = shutil.which("powershell") or shutil.which("powershell.exe")
    if shell is None:
        pytest.skip("Windows PowerShell is not available")
    script_literal = str(SCRIPT).replace("'", "''")
    command = (
        "$errors=$null; "
        "[void][System.Management.Automation.Language.Parser]::ParseFile("
        f"'{script_literal}', [ref]$null, [ref]$errors); "
        "if ($errors.Count -ne 0) { "
        "$errors | ForEach-Object { Write-Error $_.Message }; exit 1 }"
    )
    completed = subprocess.run(
        [shell, "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", command],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
