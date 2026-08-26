from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "operations" / "scripts" / "restore-arm-pi.ps1"
DEPLOY_SCRIPT = ROOT / "operations" / "scripts" / "deploy-arm-gateway.ps1"
COMPILE_SCRIPT = ROOT / "operations" / "scripts" / "compile-firmware.ps1"
EXPECTED_GATEWAY_MODULES = {
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


def _powershell_path() -> str | None:
    return shutil.which("powershell.exe") or shutil.which("powershell")


def _bash_path() -> str | None:
    discovered = shutil.which("bash")
    if discovered:
        return discovered
    git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
    return str(git_bash) if git_bash.is_file() else None


def _expected_modules_declared_by(source: str) -> set[str]:
    match = re.search(
        r"\$expectedGatewayModules\s*=\s*@\((.*?)\r?\n\)", source, re.S
    )
    assert match is not None
    return set(re.findall(r"'([^']+\.py)'", match.group(1)))


def test_recovery_script_is_sd_bound_pinned_and_phase_separated() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    for required in (
        "ValidateSet('Prepare', 'Bootstrap', 'ConfigureUart', 'Activate', 'Deactivate', 'Verify')",
        "[ValidateRange(1, 65535)]",
        "[int] $Port = 22",
        "StrictHostKeyChecking=yes",
        "IdentitiesOnly=yes",
        "PasswordAuthentication=no",
        "expectedRootDevice = $ExpectedRootDevice",
        '[ "$root" = "$expected_root" ]',
        '[ "$boot" = "$expected_boot" ]',
        "[Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $CalibrationFile",
        "[Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $CalibrationProvenanceFile",
        "arm-gateway-physical-uart.conf",
        "do_serial_cons 1",
        "do_serial_hw 0",
        "-ConfirmedArmSupported",
        "20-arm-controller-uart.conf",
        "copiedIntoManifest = $false",
        "Assert-RecoveredCalibrationProvenance",
        "artifact.sha256",
        "New-RemoteDigestContract",
        "Invoke-RemoteDigestCheck",
        '$remoteHost = if ($HostName.Contains(\':\')) { "[$HostName]" } else { $HostName }',
        "SshArguments = @('-p', [string] $Port) + $shared",
        "ScpArguments = @('-q', '-P', [string] $Port) + $shared",
        "sshPort = $Port",
        "-Port $Port",
        "sha256sum --",
        "systemctl enable wayvnc.service",
        "NetworkManager-wait-online.service",
    ):
        assert required in source

    for forbidden in (
        "StrictHostKeyChecking=no",
        "ssh-keygen -R",
        "systemctl reboot",
        "sudo reboot",
        "shutdown -r",
        "arm_move",
        "clear_stop",
        "upload-arm-hat-controller",
        "/dev/sda2 ] ||",
        "systemctl start wayvnc.service",
        "systemctl restart wayvnc.service",
        "Physical UART activation is healthy",
        'echo "$actual"',
        'echo "$expected"',
        'cat "$installRoot/.robot-gateway.token"',
    ):
        assert forbidden not in source


def test_first_physical_activation_is_fail_closed_and_state_proven() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    for required in (
        'expected_drop_in_sha256=$4',
        'staged_sha256=$(sha256sum -- "$staged_drop_in"',
        '[ "$staged_sha256" = "$expected_drop_in_sha256" ]',
        'installed_sha256=$(sudo sha256sum -- "$drop_in"',
        'arm-clear-required.json',
        'os.link(temporary, target, follow_symlinks=False)',
        'os.fchown(fd, uid, gid)',
        'payload = b\'{"clearRequired":true,"reason":"STOP_DELIVERY_UNKNOWN","version":1}\\n\'',
        'stat.S_IMODE(result.st_mode) != 0o600',
        'result.st_uid != uid or result.st_gid != gid',
        'document.get("clearRequired") is not True',
        'document.get("reason") not in accepted_reasons',
        'Authorization": f"Bearer {token}',
        'http://127.0.0.1:8787/api/robot/arm/state',
        'controller.get("controllerId") != expected_controller_id',
        'controller.get("firmwareVersion") != expected_firmware_version',
        'f"{expected_firmware_version} controller and four torque-off stationary joints are online',
        'controller.get("multiTurnAbsoluteV1") is not True',
        'controller.get("liveFollowV1") is not True',
        'state.get("telemetryAgeMs")',
        'state.get("telemetryGeneration")',
        'packet_age + telemetry_age > 1000',
        'state.get("stopped") is not True',
        'state.get("operatorInspectionRequired") is not True',
        'state.get("held") != []',
        'floor_guard.get("enabled") is not True',
        'state.get("collisionSuspected") is not False',
        'row.get("torque") != "off"',
        'row.get("moving") is not False',
        'row.get("packetAgeMs")',
        'The Pi-local authenticated controller safety gate passed',
        'typed controller',
        'corroboration and camera/autofocus verification remain required',
        'no cached collision reported',
    ):
        assert required in source

    assert 'rm /var/lib/arm-gateway/arm-clear-required.json' not in source
    assert 'unlink("/var/lib/arm-gateway/arm-clear-required.json")' not in source


def test_physical_uart_deactivation_is_reviewed_fail_closed_and_dormant() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    deactivate = source[
        source.index("function Invoke-Deactivate") : source.index("function Invoke-Verify")
    ]

    for required in (
        "New-RemoteDigestContract -Scope Deactivate",
        "Pre-deactivation physical UART integrity verification",
        'drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)',
        '[ "$drop_ins" = "$drop_in" ]',
        'document.get("clearRequired") is not True',
        'sudo rm -- "$drop_in"',
        "sudo systemctl daemon-reload",
        "sudo systemctl restart arm-gateway.service",
        "current_drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)",
        "*'--arm-controller-port'*)",
        "Dormant restart did not retain the fail-closed latch.",
        "Physical UART deactivation passed",
        "issued no STOP clear, calibration action, torque, motion, or camera capture",
    ):
        assert required in deactivate

    assert deactivate.index("Pre-deactivation physical UART integrity verification") < deactivate.index(
        'sudo rm -- "$drop_in"'
    )
    assert 'rm -- "$latch"' not in deactivate
    assert 'unlink("/var/lib/arm-gateway/arm-clear-required.json")' not in deactivate


def test_bootstrap_and_verify_report_wayvnc_without_surprise_start() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "WayVNC system service is absent" in source
    assert "WayVNC is enabled for the next boot and was deliberately not started" in source
    assert "report_unit wayvnc.service" in source
    assert "report_unit NetworkManager-wait-online.service" in source
    assert "Dormant gateway process is active and /healthz is ready" in source
    assert "no physical controller or camera health is claimed" in source
    assert "This does not prove controller, servo-bus, or camera health" in source
    assert "[System.IO.File]::WriteAllText" in source
    assert '([string[]]$rows -join "`n") + "`n"' in source
    assert "[System.IO.File]::WriteAllLines" not in source


def test_bootstrap_and_verify_digest_the_complete_prepared_recovery_install() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    for required in (
        "A freshly prepared recovery manifest is required for the $Scope digest contract.",
        "Source = $requirementsPath",
        'Remote = "$installRoot/requirements-pi.txt"',
        "Label = 'requirements-pi'",
        "Source = $resolvedManifestPath",
        "Remote = '/var/lib/arm-gateway/recovery-manifest.json'",
        "Label = 'recovery-manifest'",
        "New-RemoteDigestContract -Scope Bootstrap -ManifestPath $ManifestPath",
        "New-RemoteDigestContract -Scope Verify -ManifestPath $ManifestPath",
        "Invoke-Verify -Context $context -ManifestPath $manifestPath",
    ):
        assert required in source

    assert "Resolve-RequiredFile -Path $ManifestPath" in source


def test_restore_and_deploy_require_the_exact_reviewed_gateway_module_names() -> None:
    local = {path.name for path in (ROOT / "python" / "robot_gateway").glob("*.py")}
    assert local == EXPECTED_GATEWAY_MODULES

    for script in (SCRIPT, DEPLOY_SCRIPT):
        source = script.read_text(encoding="utf-8")
        assert _expected_modules_declared_by(source) == EXPECTED_GATEWAY_MODULES
        assert "Expected exactly the reviewed 13 robot_gateway Python modules." in source
        assert "[string]::Join(" in source


def test_deploy_can_only_preserve_the_exact_reviewed_physical_uart_drop_in() -> None:
    source = DEPLOY_SCRIPT.read_text(encoding="utf-8")
    install = source[source.index("$installScript = @'") : source.index("$cleanupScript = @'")]

    for required in (
        "[switch] $PreservePhysicalUart",
        "arm-gateway-physical-uart.conf",
        "DeploymentMode = 'dormant'",
        "[ValidateSet('dormant', 'physical')]",
        'expected_drop_in=/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf',
        '[ "$drop_ins" = "$expected_drop_in" ]',
        'cmp -s "$stage/arm-gateway-physical-uart.conf" "$expected_drop_in"',
        "no active UART drop-in exists to preserve",
        "if ($PreservePhysicalUart) { 'physical' } else { 'dormant' }",
    ):
        assert required in source

    assert "stage=$1\ndeployment_mode=$2" in install.replace("\r\n", "\n")
    assert "sudo install" not in source[source.index("expected_drop_in=") : source.index("sudo systemctl stop")]


@pytest.mark.parametrize("script_name", [SCRIPT.name, DEPLOY_SCRIPT.name])
def test_restore_and_deploy_reject_a_13_file_name_substitution(
    tmp_path: Path, script_name: str
) -> None:
    powershell = _powershell_path()
    if powershell is None:
        pytest.skip("Windows PowerShell is not available")

    fixture_root = tmp_path / script_name.removesuffix(".ps1")
    scripts = fixture_root / "operations" / "scripts"
    gateway = fixture_root / "python" / "robot_gateway"
    deploy = fixture_root / "operations" / "deploy"
    shims = fixture_root / "shims"
    for directory in (scripts, gateway, deploy, shims):
        directory.mkdir(parents=True, exist_ok=True)

    source_script = ROOT / "operations" / "scripts" / script_name
    fixture_script = scripts / script_name
    shutil.copyfile(source_script, fixture_script)
    substituted = (EXPECTED_GATEWAY_MODULES - {"pi_camera.py"}) | {"unexpected.py"}
    assert len(substituted) == 13
    for name in substituted:
        (gateway / name).write_text("# fixture\n", encoding="utf-8")

    identity = fixture_root / "identity"
    known_hosts = fixture_root / "known-hosts"
    identity.write_text("fixture\n", encoding="utf-8")
    known_hosts.write_text("fixture\n", encoding="utf-8")
    (fixture_root / "operations" / "requirements-pi.txt").write_text(
        "fixture==1\n", encoding="utf-8"
    )
    (deploy / "arm-gateway.service").write_text(
        "[Service]\nUser=__SERVICE_USER__\nGroup=__SERVICE_GROUP__\n"
        "ExecStart=__INSTALL_ROOT__/.venv/bin/python -m robot_gateway serve\n",
        encoding="utf-8",
    )
    (deploy / "arm-gateway-physical-uart.conf").write_text(
        "[Service]\nExecStart=__INSTALL_ROOT__/.venv/bin/python "
        "-m robot_gateway serve --arm-controller-port /dev/serial0\n",
        encoding="utf-8",
    )
    (deploy / "wayvnc-network-online.conf").write_text("[Unit]\n", encoding="utf-8")
    for name in ("ssh.cmd", "scp.cmd"):
        (shims / name).write_text("@exit /b 0\n", encoding="ascii")
    environment = dict(os.environ)
    windows = Path(environment.get("SystemRoot", r"C:\Windows"))
    environment["PATH"] = os.pathsep.join(
        (str(shims), str(windows / "System32"), str(windows))
    )

    if script_name == SCRIPT.name:
        token = fixture_root / "token"
        calibration = fixture_root / "calibration.json"
        provenance = fixture_root / "provenance.json"
        token.write_text("T" * 64 + "\n", encoding="ascii")
        calibration.write_text("{}\n", encoding="utf-8")
        provenance.write_text("{}\n", encoding="utf-8")
        arguments = [
            "-Phase", "Prepare",
            "-HostName", "arm-test.invalid",
            "-UserName", "armtest",
            "-ServiceGroup", "armtest",
            "-IdentityFile", str(identity),
            "-KnownHostsFile", str(known_hosts),
            "-TokenFile", str(token),
            "-CalibrationFile", str(calibration),
            "-CalibrationProvenanceFile", str(provenance),
            "-ExpectedHostName", "arm-test-host",
            "-ExpectedRootDevice", "/dev/test-root",
            "-ExpectedBootDevice", "/dev/test-boot",
            "-ExpectedControllerId", "armhat-example-controller",
            "-ExpectedFirmwareVersion", "arm-hat-example-version",
            "-DirectLanAddressCidr", "198.51.100.2/24",
        ]
    else:
        arguments = [
            "-HostName",
            "192.0.2.1",
            "-UserName",
            "armtest",
            "-ServiceGroup",
            "armtest",
            "-IdentityFile",
            str(identity),
            "-KnownHostsFile",
            str(known_hosts),
        ]

    completed = subprocess.run(
        [
            powershell,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(fixture_script),
            *arguments,
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
        env=environment,
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "Expected exactly the reviewed 13 robot_gateway Python modules." in output
    assert "pi_camera.py" in output
    assert "unexpected.py" in output


def test_verify_asserts_rebuilt_pi_prerequisites_without_live_hardware_calls() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    verify = source[source.index("function Invoke-Verify") : source.index("$inputs = Get-RecoveryInputs")]

    for package in (
        "python3",
        "python3-venv",
        "python3-picamera2",
        "python3-libcamera",
        "libcamera0.7",
        "rpicam-apps",
        "wayvnc",
        "openssh-server",
        "network-manager",
        "raspi-config",
        "curl",
    ):
        assert f"  {package}" in verify

    for required in (
        "require_active_enabled_unit ssh.service",
        "require_active_enabled_unit arm-gateway.service",
        "systemctl cat wayvnc.service",
        "systemctl is-enabled wayvnc.service",
        "systemctl cat NetworkManager-wait-online.service",
        "--property=FragmentPath",
        "--property=DropInPaths",
        "20-arm-controller-uart.conf",
        "runtime_mode=dormant",
        "runtime_mode=physical",
        "--property=ExecStart",
        "--camera picamera2",
        "--camera-profile module3-wide",
        "--arm-controller-port /dev/serial0",
        "--property=NRestarts",
        "[ \"$restarts\" -eq 0 ]",
        "import fastapi, serial, uvicorn, picamera2, libcamera",
        "http://127.0.0.1:8787/healthz",
    ):
        assert required in verify

    for forbidden in (
        "/api/arm/controller",
        "/api/arm/state",
        "/api/camera",
        "autofocus",
        "libcamera-still",
        "rpicam-still",
        "--simulator",
    ):
        assert forbidden not in verify


def test_verify_remote_shell_payload_parses() -> None:
    bash = _bash_path()
    if bash is None:
        pytest.skip("bash is not available")
    source = SCRIPT.read_text(encoding="utf-8")
    verify = source[source.index("function Invoke-Verify") : source.index("$inputs = Get-RecoveryInputs")]
    match = re.search(r"\$script\s*=\s*@'\r?\n(.*?)\r?\n'@", verify, re.S)
    assert match is not None
    completed = subprocess.run(
        [bash, "-n"],
        input=match.group(1),
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


@pytest.mark.parametrize("script", [SCRIPT, DEPLOY_SCRIPT, COMPILE_SCRIPT])
def test_restore_and_deploy_scripts_parse_as_powershell(script: Path) -> None:
    powershell = _powershell_path()
    if powershell is None:
        pytest.skip("Windows PowerShell is not available")
    script_literal = str(script).replace("'", "''")
    command = (
        "$tokens=$null; $errors=$null; "
        "[void][System.Management.Automation.Language.Parser]::ParseFile("
        f"'{script_literal}', [ref]$tokens, [ref]$errors); "
        "if ($errors.Count -ne 0) { "
        "$errors | ForEach-Object { Write-Error $_.Message }; exit 1 }"
    )
    completed = subprocess.run(
        [
            powershell,
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


def test_repeated_verify_reuses_latest_manifest_without_repreparing_it(
    tmp_path: Path,
) -> None:
    powershell = shutil.which("powershell.exe") or shutil.which("powershell")
    if powershell is None:
        pytest.skip("Windows PowerShell is not available")

    fixture_root = tmp_path / "fixture"
    scripts = fixture_root / "operations" / "scripts"
    gateway = fixture_root / "python" / "robot_gateway"
    deploy = fixture_root / "operations" / "deploy"
    recovery = fixture_root / "runtime" / "robot-gateway" / "recovery"
    shims = tmp_path / "shims"
    for directory in (scripts, gateway, deploy, recovery, shims):
        directory.mkdir(parents=True, exist_ok=True)

    fixture_script = scripts / SCRIPT.name
    fixture_source = SCRIPT.read_text(encoding="utf-8").replace(
        "Set-StrictMode -Version Latest",
        "Import-Module Microsoft.PowerShell.Utility -ErrorAction Stop\n"
        "Set-StrictMode -Version Latest",
        1,
    )
    fixture_script.write_text(fixture_source, encoding="utf-8")
    (scripts / "deploy-arm-gateway.ps1").write_text("# fixture\n", encoding="utf-8")
    (fixture_root / "operations" / "requirements-pi.txt").write_text(
        "fixture==1\n",
        encoding="utf-8",
    )
    (deploy / "arm-gateway.service").write_text(
        "[Service]\nUser=__SERVICE_USER__\nGroup=__SERVICE_GROUP__\n"
        "ExecStart=__INSTALL_ROOT__/.venv/bin/python -m robot_gateway serve\n",
        encoding="utf-8",
    )
    (deploy / "arm-gateway-physical-uart.conf").write_text(
        "[Service]\nExecStart=__INSTALL_ROOT__/.venv/bin/python "
        "-m robot_gateway serve --arm-controller-port /dev/serial0\n",
        encoding="utf-8",
    )
    (deploy / "wayvnc-network-online.conf").write_text(
        "[Unit]\n", encoding="utf-8"
    )
    for source in sorted((ROOT / "python" / "robot_gateway").glob("*.py")):
        (gateway / source.name).write_text("# fixture\n", encoding="utf-8")

    joint = {
        "servoId": 1,
        "rawZero": 2048,
        "rawMin": 0,
        "rawMax": 4095,
        "ratio": 1,
        "direction": 1,
        "speed": 400,
        "accel": 10,
    }
    calibration = {
        f"joint_{index}": {**joint, "servoId": index} for index in range(1, 5)
    }
    calibration_path = recovery / "arm-joints.recovered.json"
    calibration_path.write_text(
        json.dumps(calibration, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    calibration_sha256 = hashlib.sha256(calibration_path.read_bytes()).hexdigest()
    provenance_path = recovery / "arm-joints.recovered.provenance.json"
    provenance_path.write_text(
        json.dumps(
            {
                "schema": "arm-joints-recovery-provenance.v1",
                "artifact": {"sha256": calibration_sha256, "jointCount": 4},
            },
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    token_path = fixture_root / "token"
    token_path.write_text("t" * 64 + "\n", encoding="ascii")
    identity_path = fixture_root / "identity"
    known_hosts_path = fixture_root / "known-hosts"
    identity_path.write_text("fixture\n", encoding="utf-8")
    known_hosts_path.write_text("fixture\n", encoding="utf-8")

    for name in ("ssh.cmd", "scp.cmd"):
        (shims / name).write_text("@exit /b 0\n", encoding="ascii")
    environment = dict(os.environ)
    windows = Path(environment.get("SystemRoot", r"C:\Windows"))
    environment["PATH"] = os.pathsep.join(
        (str(shims), str(windows / "System32"), str(windows))
    )

    common = [
        powershell,
        "-NoLogo",
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(fixture_script),
        "-IdentityFile",
        str(identity_path),
        "-KnownHostsFile",
        str(known_hosts_path),
        "-TokenFile",
        str(token_path),
        "-CalibrationFile",
        str(calibration_path),
        "-CalibrationProvenanceFile",
        str(provenance_path),
        "-HostName",
        "arm-test.invalid",
        "-UserName",
        "armtest",
        "-ServiceGroup",
        "armtest",
        "-ExpectedHostName",
        "arm-test-host",
        "-ExpectedRootDevice",
        "/dev/test-root",
        "-ExpectedBootDevice",
        "/dev/test-boot",
        "-ExpectedControllerId",
        "armhat-example-controller",
        "-ExpectedFirmwareVersion",
        "arm-hat-example-version",
        "-DirectLanAddressCidr",
        "198.51.100.2/24",
    ]
    prepared = subprocess.run(
        [*common, "-Phase", "Prepare"],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
        env=environment,
    )
    assert prepared.returncode == 0, prepared.stderr
    latest_path = recovery / "LATEST.json"
    latest_bytes = latest_path.read_bytes()
    prepared_files = sorted(recovery.glob("prepared-*.json"))
    assert len(prepared_files) == 1

    for _ in range(2):
        verified = subprocess.run(
            [*common, "-Phase", "Verify"],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
            env=environment,
        )
        assert verified.returncode == 0, verified.stderr
        assert latest_path.read_bytes() == latest_bytes
        assert sorted(recovery.glob("prepared-*.json")) == prepared_files


def test_physical_uart_drop_in_matches_the_reviewed_loopback_service() -> None:
    source = (ROOT / "operations" / "deploy" / "arm-gateway-physical-uart.conf").read_text(
        encoding="utf-8"
    )
    base_source = (ROOT / "operations" / "deploy" / "arm-gateway.service").read_text(
        encoding="utf-8"
    )
    assert "ExecStart=" in source
    assert "--arm-controller-port /dev/serial0" in source
    assert "--camera-profile module3-wide" in source
    assert "--host 127.0.0.1 --port 8787" in source
    assert "0.0.0.0" not in source
    assert "--simulator" not in source
    assert "--simulator" not in base_source
