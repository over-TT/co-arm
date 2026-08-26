from __future__ import annotations

from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "operations" / "scripts" / "backup-arm-pi.ps1"


def _source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _bash_path() -> str | None:
    discovered = shutil.which("bash")
    if discovered:
        return discovered
    git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
    return str(git_bash) if git_bash.is_file() else None


def _explicit_parameters(identity: Path, known_hosts: Path) -> list[str]:
    return [
        "-UserName",
        "armtest",
        "-ServiceGroup",
        "armtest",
        "-IdentityFile",
        str(identity),
        "-KnownHostsFile",
        str(known_hosts),
        "-ExpectedHostName",
        "arm-test-host",
        "-ExpectedRootDevice",
        "/dev/test-root",
        "-ExpectedBootDevice",
        "/dev/test-boot",
        "-DirectLanAddressCidr",
        "198.51.100.2/24",
        "-DirectLanProfilePath",
        "/etc/netplan/99-arm-test.yaml",
    ]


def test_backup_uses_only_the_pinned_noninteractive_ssh_identity() -> None:
    source = _source()
    for required in (
        "UserKnownHostsFile=$sshKnownHostsPath",
        "GlobalKnownHostsFile=$emptyGlobalKnownHostsPath",
        "StrictHostKeyChecking=yes",
        "IdentitiesOnly=yes",
        "BatchMode=yes",
        "PasswordAuthentication=no",
        "KbdInteractiveAuthentication=no",
        "UpdateHostKeys=no",
    ):
        assert required in source
    assert "StrictHostKeyChecking=no" not in source
    assert "AcceptNew" not in source


def test_backup_rejects_an_unsafe_host_before_opening_ssh(tmp_path: Path) -> None:
    identity = tmp_path / "identity"
    known_hosts = tmp_path / "known-hosts"
    identity.write_text("test-only", encoding="utf-8")
    known_hosts.write_text("test-only", encoding="utf-8")

    completed = subprocess.run(
        [
            "powershell.exe",
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(SCRIPT),
            "-HostName",
            "arm-pi;touch-bad",
            *_explicit_parameters(identity, known_hosts),
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    assert completed.returncode != 0
    assert "HostName must be" in completed.stderr
    assert "Backup implementation is not complete" not in completed.stderr


def test_installation_identity_inputs_have_no_repository_defaults() -> None:
    completed = subprocess.run(
        [
            "powershell.exe",
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(SCRIPT),
            "-HostName",
            "arm-pi;touch-bad",
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    assert completed.returncode != 0
    assert "MissingMandatoryParameter" in completed.stderr
    assert "Join-Path" not in completed.stderr


def test_remote_snapshot_refuses_the_wrong_pi_or_boot_media_and_cleans_tmp() -> None:
    source = _source()
    for required in (
        "actual_hostname=$(hostname)",
        "actual_user=$(id -un)",
        "root_source=$(findmnt -n -o SOURCE -- /)",
        "boot_source=$(findmnt -n -o SOURCE -- /boot/firmware)",
        'case "$model" in',
        "Raspberry\\ Pi*)",
        'if [ "$actual_hostname" != "$expected_hostname" ]',
        'if [ "$actual_user" != "$expected_user" ]',
        'if [ "$root_source" != "$expected_root" ]',
        'if [ "$root_fstype" != "ext4" ]',
        'if [ "$boot_source" != "$expected_boot" ]',
        "exit 42",
        "mktemp -d /tmp/arm-pi-backup.XXXXXXXXXX",
        "trap cleanup EXIT HUP INT TERM",
        '/tmp/arm-pi-backup.*)',
        'sudo rm -rf -- "$remote_tmp"',
    ):
        assert required in source

    assert source.index("root_fstype=$(findmnt -n -o FSTYPE -- /)") < source.index(
        "remote_tmp=$(sudo mktemp -d /tmp/arm-pi-backup.XXXXXXXXXX)"
    )
    assert 'sudo chown "$expected_user:$service_group" "$remote_tmp"' in source
    assert 'sudo install -m 0600 -o "$expected_user" -g "$service_group" /dev/null "$file_list"' in source
    assert 'sudo install -m 0600 -o pi -g pi /dev/null "$file_list"' not in source
    assert 'sudo chmod 0600 "$archive_gz"' in source


def test_snapshot_curates_recovery_state_and_hashes_without_dumping_secrets() -> None:
    source = _source()
    for required_path in (
        "$install_relative/robot_gateway/$module",
        "$install_relative/requirements-pi.txt",
        "$install_relative/.robot-gateway.token",
        "var/lib/arm-gateway/arm-joints.json",
        "var/lib/arm-gateway/arm-joints.recovery-provenance.json",
        "var/lib/arm-gateway/recovery-manifest.json",
        "var/lib/arm-gateway/physical-arm-profile.json",
        "var/lib/arm-gateway/arm-clear-required.json",
        "etc/systemd/system/arm-gateway.service",
        "etc/systemd/system/arm-gateway.service.d",
        "etc/systemd/system/wayvnc.service.d",
        "etc/NetworkManager/system-connections",
        "etc/netplan",
        "etc/ssh",
        "$home_relative/.ssh/authorized_keys",
        "$home_relative/.config/wayvnc",
        "boot/firmware/config.txt",
        "boot/firmware/cmdline.txt",
        "$direct_lan_profile_relative",
        "$direct_lan_address_cidr",
    ):
        assert required_path in source

    for required in (
        "SHA256SUMS",
        "snapshot.tsv",
        "packages.tsv",
        "sha256sum",
        "tar --create",
        "gzip",
        "--no-recursion",
        "--dereference",
        "snapshotFilesDereferenced",
        "checksumsCoverSnapshotBytes",
        "physicalArmProfilePresent",
        "stopLatchPresent",
        "physicalUartDropInPresent",
        "20-arm-controller-uart.conf",
        "requiredDirectLanNetplanPresent",
        "require_regular_nonsymlink",
        "sudo grep -Fq -- 'eth0'",
        'sudo grep -Fq -- "$direct_lan_address_cidr"',
    ):
        assert required in source

    assert "Get-Content -LiteralPath $token" not in source
    assert 'cat "$install_root/.robot-gateway.token"' not in source
    assert "set -x" not in source
    assert 'add_tree "$install_relative/.venv"' not in source
    assert 'add_tree "$install_relative/backups"' not in source
    assert "codex-arm-lan.nmconnection" not in source
    assert "20-physical-uart.conf" not in source


def test_backup_requires_exactly_the_reviewed_13_gateway_modules() -> None:
    source = _source()
    expected = {
        "__init__.py",
        "__main__.py",
        "arm_controller.py",
        "camera_api.py",
        "camera_profiles.py",
        "isaac_bridge.py",
        "physical_arm_api.py",
        "pi_camera.py",
        "request_validation.py",
        "runtime.py",
        "serial_arm_controller.py",
        "simple_arm_api.py",
        "strict_contract.py",
    }
    local = {path.name for path in (ROOT / "python" / "robot_gateway").glob("*.py")}
    assert local == expected
    for name in expected:
        assert f"'{name}'" in source
    for required in (
            "$currentGatewayModuleNames.Count -ne 13",
            "$expectedGatewayModulesCsv",
            'cmp -s "$expected_modules" "$actual_modules" || incomplete',
            'cmp -s "$expected_modules" "$snapshot_modules" || incomplete',
            'cmp -s "$expected_modules" "$verify_modules" || incomplete',
    ):
        assert required in source


def test_backup_behaviorally_rejects_a_partial_local_gateway(tmp_path: Path) -> None:
    shell = shutil.which("powershell.exe") or shutil.which("powershell")
    if shell is None:
        pytest.skip("Windows PowerShell is not available")

    fake_root = tmp_path / "repo"
    scripts = fake_root / "operations" / "scripts"
    gateway = fake_root / "python" / "robot_gateway"
    scripts.mkdir(parents=True)
    gateway.mkdir(parents=True)
    copied_script = scripts / SCRIPT.name
    shutil.copyfile(SCRIPT, copied_script)
    for name in sorted(path.name for path in (ROOT / "python" / "robot_gateway").glob("*.py"))[:-1]:
        (gateway / name).write_text("# partial fixture\n", encoding="utf-8")

    completed = subprocess.run(
        [
            shell,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(copied_script),
            "-HostName",
            "arm-test.invalid",
            *_explicit_parameters(fake_root / "identity", fake_root / "known-hosts"),
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    assert completed.returncode != 0
    assert "exactly the reviewed 13 Python modules" in (completed.stdout + completed.stderr)


def test_remote_snapshot_hashes_frozen_bytes_and_self_verifies_final_archive() -> None:
    source = _source()
    assert source.index('sudo tar --create --file "$seed_tar"') < source.index(
        'metadata_dir="$snapshot_root/metadata"'
    )
    assert source.index('metadata_dir="$snapshot_root/metadata"') < source.index(
        'sha256sum -c metadata/SHA256SUMS'
    )
    for required in (
        'sudo find "$snapshot_root" -type f ! -path "$metadata_dir/*"',
        'xargs -0 -r sha256sum -- < "$2" > metadata/SHA256SUMS',
        "metadata/METADATA_SHA256SUMS",
        '$metadata_dir/gateway-modules.txt',
        'sudo tar --extract --gzip --file "$archive_gz" --directory "$verify_root"',
        "sha256sum -c metadata/SHA256SUMS",
        "sha256sum -c metadata/METADATA_SHA256SUMS",
        'sudo cat "$archive_gz"',
    ):
        assert required in source


def test_remote_backup_shell_payload_parses() -> None:
    shell = _bash_path()
    if shell is None:
        pytest.skip("bash is not available")
    match = re.search(r"\$remoteArchiveScript\s*=\s*@'\r?\n(.*?)\r?\n'@", _source(), re.S)
    assert match is not None
    completed = subprocess.run(
        [shell, "-n"],
        input=match.group(1),
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_local_result_is_timestamped_private_atomic_and_self_describing() -> None:
    source = _source()
    for required in (
        "runtime\\robot-gateway\\pi-backups",
        "yyyyMMddTHHmmssZ",
        "SetAccessRuleProtection($true, $false)",
        "[System.Security.AccessControl.FileSystemRights]::FullControl",
        "[System.Security.AccessControl.InheritanceFlags]::ContainerInherit",
        "[System.Security.AccessControl.InheritanceFlags]::ObjectInherit",
        ".partial",
        "StandardOutput.BaseStream.CopyTo",
        "StandardError.ReadToEndAsync",
        "Get-FileHash -LiteralPath $partialArchivePath -Algorithm SHA256",
        "tar --list --gzip --file",
        "Move-Item -LiteralPath $partialArchivePath",
        "metadata.json",
        "archive.sha256",
        "ConvertTo-Json",
        "Remove-IncompleteBackupDirectory",
        "$processStarted = $false",
        "$processStarted = $true",
        "if ($processStarted -and -not $process.HasExited)",
    ):
        assert required in source
    assert "[System.IO.Directory]::SetAccessControl($Path, $security)" in source
    assert "Set-Acl" not in source
    assert ".SetOwner(" not in source


def test_private_directory_acl_applies_without_security_privilege(
    tmp_path: Path,
) -> None:
    powershell = shutil.which("powershell.exe")
    if powershell is None:
        pytest.skip("Windows PowerShell is not available")

    match = re.search(
        r"(function Set-PrivateDirectory\s*\{.*?^\})",
        _source(),
        re.S | re.M,
    )
    assert match is not None
    directory = tmp_path / "private-backup"
    directory.mkdir()
    literal = str(directory).replace("'", "''")
    command = (
        f"{match.group(1)}\n"
        f"$path = '{literal}'\n"
        "Set-PrivateDirectory -Path $path\n"
        "Set-PrivateDirectory -Path $path\n"
        "$acl = [System.IO.Directory]::GetAccessControl($path, [System.Security.AccessControl.AccessControlSections]::Access)\n"
        "$sid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User\n"
        "if (-not $acl.AreAccessRulesProtected) { throw 'DACL is not protected.' }\n"
        "$rules = @($acl.Access | Where-Object { -not $_.IsInherited })\n"
        "if ($rules.Count -ne 1) { throw \"Expected one explicit ACE; got $($rules.Count).\" }\n"
        "$rule = $rules[0]\n"
        "$ruleSid = $rule.IdentityReference.Translate([System.Security.Principal.SecurityIdentifier])\n"
        "if ($ruleSid.Value -ne $sid.Value) { throw 'ACE is not for the current user.' }\n"
        "if ($rule.AccessControlType -ne [System.Security.AccessControl.AccessControlType]::Allow) { throw 'ACE is not Allow.' }\n"
        "$full = [System.Security.AccessControl.FileSystemRights]::FullControl\n"
        "if (($rule.FileSystemRights -band $full) -ne $full) { throw 'ACE is not FullControl.' }\n"
        "$expectedInheritance = [System.Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [System.Security.AccessControl.InheritanceFlags]::ObjectInherit\n"
        "if (($rule.InheritanceFlags -band $expectedInheritance) -ne $expectedInheritance) { throw 'ACE does not inherit to children.' }\n"
        "$child = Join-Path $path 'token-like-file'\n"
        "[System.IO.File]::WriteAllText($child, 'test-only')\n"
        "$foreign = @(\n"
        "  [System.IO.File]::GetAccessControl($child, [System.Security.AccessControl.AccessControlSections]::Access).Access | Where-Object {\n"
        "    $_.IdentityReference.Translate([System.Security.Principal.SecurityIdentifier]).Value -ne $sid.Value\n"
        "  }\n"
        ")\n"
        "if ($foreign.Count -ne 0) { throw 'Child inherited an unexpected identity.' }\n"
    )
    completed = subprocess.run(
        [
            powershell,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            command,
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_backup_script_parses_as_powershell() -> None:
    script_literal = str(SCRIPT).replace("'", "''")
    command = (
        "$errors=$null; "
        "[void][System.Management.Automation.Language.Parser]::ParseFile("
        f"'{script_literal}', [ref]$null, [ref]$errors); "
        "if ($errors.Count -ne 0) { "
        "$errors | ForEach-Object { Write-Error $_.Message }; exit 1 }"
    )
    completed = subprocess.run(
        [
            "powershell.exe",
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            command,
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
