from __future__ import annotations

from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "operations" / "scripts" / "set-arm-gateway-mode.ps1"


def _source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _powershell_path() -> str | None:
    return shutil.which("powershell.exe") or shutil.which("powershell")


def _bash_path() -> str | None:
    discovered = shutil.which("bash")
    if discovered:
        return discovered
    git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
    return str(git_bash) if git_bash.is_file() else None


def test_mode_operation_is_approval_and_pinned_transport_gated_without_support_prompt() -> None:
    source = _source()
    for required in (
        "SupportsShouldProcess = $true",
        "ConfirmImpact = 'High'",
        "ValidateSet('Activate', 'Deactivate')",
        "[ValidateRange(1, 65535)]",
        "[int] $Port = 22",
        "StrictHostKeyChecking=yes",
        "IdentitiesOnly=yes",
        "BatchMode=yes",
        "PasswordAuthentication=no",
        "KbdInteractiveAuthentication=no",
        "UpdateHostKeys=no",
        "GlobalKnownHostsFile=$emptyGlobalKnownHosts",
        "$sshArguments = @('-p', [string] $Port)",
        "$scpArguments = @('-q', '-P', [string] $Port)",
        '$remoteHost = if ($HostName.Contains(\':\')) { "[$HostName]" } else { $HostName }',
    ):
        assert required in source

    assert source.index("$PSCmdlet.ShouldProcess") < source.index(
        "$preparePayload | & $sshPath"
    )
    assert "StrictHostKeyChecking=no" not in source
    assert "ConfirmedArmSupported" not in source
    assert "support confirmation" not in source.lower()


def test_activate_verifies_reviewed_install_and_fail_closed_controller_state() -> None:
    source = _source()
    for required in (
        "arm-gateway.service",
        "arm-gateway-physical-uart.conf",
        "expected_base_sha256",
        "expected_drop_in_sha256",
        'sha256sum -- "$staged_base"',
        'sha256sum -- "$staged_drop_in"',
        'sha256sum -- "$base_unit"',
        "--property=FragmentPath",
        "--property=DropInPaths",
        "readlink -f /dev/serial0",
        "console=(serial0|ttyAMA0|ttyS0)",
        "enable_uart=1",
        "os.link(temporary, target, follow_symlinks=False)",
        "os.fchown(fd, uid, gid)",
        'document.get("clearRequired") is not True',
        'http://127.0.0.1:8787/healthz',
        'http://127.0.0.1:8787/api/robot/arm/state',
        'headers={"Authorization": f"Bearer {token}"}',
        "class NoRedirect(urllib.request.HTTPRedirectHandler)",
        "urllib.request.ProxyHandler({})",
        "with opener.open(request, timeout=2) as response",
        'controller.get("controllerId") != expected_controller_id',
        'controller.get("firmwareVersion") != expected_firmware_version',
        'controller.get("multiTurnAbsoluteV1") is not True',
        'controller.get("liveFollowV1") is not True',
        'state.get("connection") != "online"',
        'state.get("bus") != "online"',
        'state.get("stopped") is not True',
        'state.get("operatorInspectionRequired") is not True',
        'state.get("held") != []',
        'floor_guard.get("enabled") is not True',
        'state.get("collisionSuspected") is not False',
        'row.get("torque") != "off"',
        'row.get("moving") is not False',
        "packet_age + telemetry_age > 1000",
        "activation_rollback_required=1",
        "cleanup_and_rollback",
        "Activation did not reach its authenticated safety proof; restoring dormant mode.",
        "Automatic dormant rollback passed",
        "Physical UART mode was already correct",
    ):
        assert required in source

    mode_branch = source.rindex('case "$mode" in')
    activation = source[mode_branch : source.index("  Deactivate)", mode_branch)]
    assert source.index('sha256sum -- "$staged_drop_in"') < mode_branch
    assert source.index("ensure_fail_closed_latch") < source.index(
        "sudo install -m 0644 -o root -g root \"$staged_drop_in\""
    )
    assert activation.index("activation_rollback_required=1") < activation.index(
        "sudo systemctl stop arm-gateway.service"
    )
    assert "urllib.request.urlopen" not in source


def test_installed_service_files_require_strict_root_ownership_and_mode() -> None:
    source = _source()
    assert 'sudo stat -c \'%u:%g:%a\' -- "$base_unit"' in source
    assert 'sudo stat -c \'%u:%g:%a\' -- "$drop_in"' in source
    assert "= '0:0:644'" in source
    assert "root-owned, root-grouped, and mode 0644" in source


def test_deactivate_is_exact_reversible_and_idempotently_dormant() -> None:
    source = _source()
    for required in (
        '"$drop_in|1") deactivation_state=physical',
        'sha256sum -- "$drop_in"',
        'sudo rm -- "$drop_in"',
        "sudo systemctl daemon-reload",
        "sudo systemctl restart arm-gateway.service",
        "verify_dormant_mode",
        "Dormant mode still exposes a physical arm-controller port.",
        "UART deactivation did not retain the fail-closed latch.",
        "Dormant gateway mode was already correct; no restart was performed",
    ):
        assert required in source

    assert 'rm -- "$latch"' not in source
    assert 'unlink("/var/lib/arm-gateway/arm-clear-required.json")' not in source
    for forbidden in (
        "raspi-config",
        "systemctl reboot",
        "sudo reboot",
        "shutdown -r",
        "deploy-arm-gateway.ps1",
        "/api/robot/arm/move",
        "/api/robot/arm/stop/clear",
        "/api/camera",
    ):
        assert forbidden not in source


def test_mode_script_and_embedded_remote_shell_parse() -> None:
    powershell = _powershell_path()
    if powershell is None:
        pytest.skip("Windows PowerShell is not available")
    literal = str(SCRIPT).replace("'", "''")
    command = (
        "$tokens=$null; $errors=$null; "
        "[void][System.Management.Automation.Language.Parser]::ParseFile("
        f"'{literal}', [ref]$tokens, [ref]$errors); "
        "if ($errors.Count -ne 0) { "
        "$errors | ForEach-Object { Write-Error $_.Message }; exit 1 }"
    )
    parsed = subprocess.run(
        [powershell, "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", command],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert parsed.returncode == 0, parsed.stderr

    bash = _bash_path()
    if bash is None:
        pytest.skip("bash is not available")
    match = re.search(r"\$modeScript\s*=\s*@'\r?\n(.*?)\r?\n'@", _source(), re.S)
    assert match is not None
    shell_parsed = subprocess.run(
        [bash, "-n"],
        input=match.group(1),
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert shell_parsed.returncode == 0, shell_parsed.stderr
