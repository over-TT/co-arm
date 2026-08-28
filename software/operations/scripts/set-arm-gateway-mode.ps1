#Requires -Version 5.1

<#
.SYNOPSIS
Activates or deactivates the reviewed physical UART mode on a prepared arm Pi.

.DESCRIPTION
This high-impact, approval-gated operation is for an existing prepared gateway
installation. Activate verifies the pinned SSH transport, installed base unit,
UART prerequisites, reviewed drop-in digest, and durable fail-closed latch,
then performs one deliberate service restart. It accepts physical mode only
after an authenticated Pi-local state proves the expected controller, firmware,
required controller capabilities, and four fresh, stationary, torque-off joints
with STOP and operator inspection latched, floor guard enabled, and collision
explicitly clear.

Deactivate verifies the same installed drop-in and latch, removes only that
drop-in, deliberately restarts the dormant service, and proves that no physical
controller port or unexpected systemd drop-in remains. Already-correct modes
are verified without a restart. Neither mode changes UART boot configuration,
reboots, deploys source, clears STOP, enables torque, moves a joint, or captures
a camera frame.
#>

[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'High')]
param(
    [Parameter(Mandatory)]
    [ValidateSet('Activate', 'Deactivate')]
    [string] $Mode,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $HostName,

    [ValidateRange(1, 65535)]
    [int] $Port = 22,

    [Parameter(Mandatory)]
    [ValidatePattern('^[a-z_][a-z0-9_-]{0,31}$')]
    [string] $UserName,

    [Parameter(Mandatory)]
    [ValidatePattern('^[a-z_][a-z0-9_-]{0,31}$')]
    [string] $ServiceGroup,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $IdentityFile,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $KnownHostsFile,

    [Parameter(Mandatory)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._:-]{2,127}$')]
    [string] $ExpectedControllerId,

    [Parameter(Mandatory)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{2,63}$')]
    [string] $ExpectedFirmwareVersion
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($HostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$' -or $HostName.StartsWith('-')) {
    throw 'HostName must be a DNS name, IPv4 address, or unbracketed IPv6 literal.'
}
function Resolve-RequiredLeaf {
    param(
        [Parameter(Mandatory)] [string] $Path,
        [Parameter(Mandatory)] [string] $Label
    )

    $resolved = (Resolve-Path -LiteralPath $Path -ErrorAction Stop).Path
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
        throw "$Label must be an existing regular file."
    }
    return $resolved
}

function Get-Sha256Lower {
    param([Parameter(Mandatory)] [string] $Path)

    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256 -ErrorAction Stop).Hash.ToLowerInvariant()
}

$softwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$operationsRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$baseTemplatePath = Resolve-RequiredLeaf `
    -Path (Join-Path $operationsRoot 'deploy\arm-gateway.service') `
    -Label 'Base service template'
$dropInTemplatePath = Resolve-RequiredLeaf `
    -Path (Join-Path $operationsRoot 'deploy\arm-gateway-physical-uart.conf') `
    -Label 'Physical UART drop-in template'
$identityPath = Resolve-RequiredLeaf -Path $IdentityFile -Label 'IdentityFile'
$knownHostsPath = Resolve-RequiredLeaf -Path $KnownHostsFile -Label 'KnownHostsFile'
$installRoot = "/home/$UserName/arm-gateway"

$baseUnit = (Get-Content -Raw -LiteralPath $baseTemplatePath).
    Replace('__SERVICE_USER__', $UserName).
    Replace('__SERVICE_GROUP__', $ServiceGroup).
    Replace('__INSTALL_ROOT__', $installRoot)
$dropIn = (Get-Content -Raw -LiteralPath $dropInTemplatePath).
    Replace('__INSTALL_ROOT__', $installRoot)
foreach ($rendered in @($baseUnit, $dropIn)) {
    if ($rendered -match '__[A-Z0-9_]+__') {
        throw 'A deployment template contains an unresolved placeholder.'
    }
}

$operation = if ($Mode -eq 'Activate') {
    'install the reviewed physical UART drop-in and restart the gateway once'
}
else {
    'remove the reviewed physical UART drop-in and restart the dormant gateway once'
}
if (-not $PSCmdlet.ShouldProcess($HostName, $operation)) {
    return
}

$sshPath = (Get-Command ssh -CommandType Application -ErrorAction Stop).Source
$scpPath = (Get-Command scp -CommandType Application -ErrorAction Stop).Source
$remoteHost = if ($HostName.Contains(':')) { "[$HostName]" } else { $HostName }
$remoteTarget = "${UserName}@${remoteHost}"
$remoteStage = '/tmp/co-arm-gateway-mode-' + [Guid]::NewGuid().ToString('N')
$localStage = Join-Path ([System.IO.Path]::GetTempPath()) (
    'co-arm-gateway-mode-' + [Guid]::NewGuid().ToString('N')
)
$stagePrepared = $false
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)

try {
    New-Item -ItemType Directory -Path $localStage -ErrorAction Stop | Out-Null
    $renderedBasePath = Join-Path $localStage 'arm-gateway.service'
    $renderedDropInPath = Join-Path $localStage '20-arm-controller-uart.conf'
    [System.IO.File]::WriteAllText($renderedBasePath, $baseUnit, $utf8NoBom)
    [System.IO.File]::WriteAllText($renderedDropInPath, $dropIn, $utf8NoBom)
    $expectedBaseSha256 = Get-Sha256Lower -Path $renderedBasePath
    $expectedDropInSha256 = Get-Sha256Lower -Path $renderedDropInPath

    # OpenSSH interprets UserKnownHostsFile as a whitespace-separated list.
    # Stage the reviewed file under a transport path without whitespace so only
    # this project pin and an intentionally empty global file can authenticate.
    $transportKnownHosts = Join-Path $localStage 'known-hosts'
    $emptyGlobalKnownHosts = Join-Path $localStage 'empty-global-known-hosts'
    Copy-Item -LiteralPath $knownHostsPath -Destination $transportKnownHosts -Force
    [System.IO.File]::WriteAllText($emptyGlobalKnownHosts, '', $utf8NoBom)
    foreach ($transportPath in @($transportKnownHosts, $emptyGlobalKnownHosts)) {
        if ($transportPath -match '\s') {
            throw 'The temporary SSH transport path contains whitespace; choose a TEMP location without whitespace.'
        }
    }

    $sharedArguments = @(
        '-i', $identityPath,
        '-o', "UserKnownHostsFile=$transportKnownHosts",
        '-o', "GlobalKnownHostsFile=$emptyGlobalKnownHosts",
        '-o', 'StrictHostKeyChecking=yes',
        '-o', 'IdentitiesOnly=yes',
        '-o', 'BatchMode=yes',
        '-o', 'PasswordAuthentication=no',
        '-o', 'KbdInteractiveAuthentication=no',
        '-o', 'UpdateHostKeys=no',
        '-o', 'ConnectTimeout=8'
    )
    $sshArguments = @('-p', [string] $Port) + $sharedArguments
    $scpArguments = @('-q', '-P', [string] $Port) + $sharedArguments

    $prepareScript = @'
set -eu
stage=$1
case "$stage" in /tmp/co-arm-gateway-mode-[0-9a-f]*) ;; *) exit 2 ;; esac
umask 077
mkdir -- "$stage"
'@
    $preparePayload = ($prepareScript -replace "`r", '') + "`n#"
    $preparePayload | & $sshPath @sshArguments $remoteTarget "sh -s -- '$remoteStage'"
    if ($LASTEXITCODE -ne 0) {
        throw "Remote mode staging failed with exit code $LASTEXITCODE."
    }
    $stagePrepared = $true

    & $scpPath @scpArguments `
        $renderedBasePath `
        $renderedDropInPath `
        "${remoteTarget}:$remoteStage/"
    if ($LASTEXITCODE -ne 0) {
        throw "Reviewed service-file staging failed with exit code $LASTEXITCODE."
    }

    $modeScript = @'
set -eu
stage=$1
mode=$2
service_user=$3
service_group=$4
install_root=$5
expected_controller_id=$6
expected_firmware_version=$7
expected_base_sha256=$8
expected_drop_in_sha256=$9

case "$stage" in /tmp/co-arm-gateway-mode-[0-9a-f]*) ;; *) exit 2 ;; esac
case "$mode" in Activate|Deactivate) ;; *) exit 2 ;; esac
for digest in "$expected_base_sha256" "$expected_drop_in_sha256"; do
  case "$digest" in *[!0-9a-f]*|'') echo 'Expected service digest is invalid.' >&2; exit 3 ;; esac
  [ "${#digest}" -eq 64 ] || { echo 'Expected service digest has the wrong length.' >&2; exit 3; }
done

cleanup_stage() {
  case "$stage" in
    /tmp/co-arm-gateway-mode-[0-9a-f]*) rm -rf -- "$stage" ;;
    *) echo 'Refusing unexpected remote staging cleanup path.' >&2 ;;
  esac
}
activation_rollback_required=0
cleanup_and_rollback() {
  status=$?
  trap - EXIT HUP INT TERM
  if [ "$activation_rollback_required" -eq 1 ]; then
    echo 'Activation did not reach its authenticated safety proof; restoring dormant mode.' >&2
    sudo systemctl stop arm-gateway.service >/dev/null 2>&1 || true
    sudo rm -f -- /etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
    sudo systemctl daemon-reload >/dev/null 2>&1 || true
    sudo systemctl restart arm-gateway.service >/dev/null 2>&1 || true
    rollback_attempt=0
    rollback_ready=0
    while [ "$rollback_attempt" -lt 20 ]; do
      rollback_drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value 2>/dev/null || true)
      rollback_exec=$(sudo systemctl show arm-gateway.service --property=ExecStart --value 2>/dev/null || true)
      if [ -z "$rollback_drop_ins" ] &&
         ! printf '%s\n' "$rollback_exec" | grep -Fq -- '--arm-controller-port' &&
         curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null 2>&1; then
        rollback_ready=1
        break
      fi
      rollback_attempt=$((rollback_attempt + 1))
      sleep 1
    done
    if [ "$rollback_ready" -eq 1 ]; then
      echo 'Automatic dormant rollback passed; the fail-closed latch was retained.' >&2
    else
      echo 'Automatic dormant rollback could not prove a healthy dormant service; operator recovery is required.' >&2
    fi
    if [ "$status" -eq 0 ]; then status=70; fi
  fi
  cleanup_stage
  exit "$status"
}
trap cleanup_and_rollback EXIT
trap 'exit 130' HUP INT TERM

staged_base="$stage/arm-gateway.service"
staged_drop_in="$stage/20-arm-controller-uart.conf"
for file in "$staged_base" "$staged_drop_in"; do
  [ -f "$file" ] && [ ! -L "$file" ] || {
    echo 'A staged reviewed service file is missing or has the wrong type.' >&2
    exit 4
  }
done
[ "$(sha256sum -- "$staged_base" | awk '{print $1}')" = "$expected_base_sha256" ] || {
  echo 'The staged base service differs from the reviewed laptop artifact.' >&2
  exit 5
}
[ "$(sha256sum -- "$staged_drop_in" | awk '{print $1}')" = "$expected_drop_in_sha256" ] || {
  echo 'The staged UART drop-in differs from the reviewed laptop artifact.' >&2
  exit 5
}

sudo -n true
[ "$(id -un)" = "$service_user" ] || {
  echo 'The authenticated SSH account is not the expected gateway service user.' >&2
  exit 6
}
base_unit=/etc/systemd/system/arm-gateway.service
drop_in=/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
latch=/var/lib/arm-gateway/arm-clear-required.json
sudo test -f "$base_unit" && ! sudo test -L "$base_unit" || {
  echo 'The installed base service is missing or has the wrong type.' >&2
  exit 7
}
[ "$(sudo stat -c '%u:%g:%a' -- "$base_unit")" = '0:0:644' ] || {
  echo 'The installed base service must be root-owned, root-grouped, and mode 0644.' >&2
  exit 7
}
[ "$(sudo sha256sum -- "$base_unit" | awk '{print $1}')" = "$expected_base_sha256" ] || {
  echo 'The installed base service differs from reviewed source.' >&2
  exit 8
}
fragment=$(sudo systemctl show arm-gateway.service --property=FragmentPath --value)
[ "$fragment" = "$base_unit" ] || {
  echo "The effective base service fragment is unexpected: $fragment" >&2
  exit 9
}
sudo systemctl is-enabled --quiet arm-gateway.service || {
  echo 'arm-gateway.service is not enabled.' >&2
  exit 10
}

ensure_fail_closed_latch() {
  sudo install -d -m 0700 -o "$service_user" -g "$service_group" /var/lib/arm-gateway
  sudo python3 - "$service_user" "$service_group" <<'PY'
import grp
import json
import os
import pwd
import stat
import sys
import tempfile

service_user, service_group = sys.argv[1:]
uid = pwd.getpwnam(service_user).pw_uid
gid = grp.getgrnam(service_group).gr_gid
directory = "/var/lib/arm-gateway"
target = os.path.join(directory, "arm-clear-required.json")
payload = b'{"clearRequired":true,"reason":"STOP_DELIVERY_UNKNOWN","version":1}\n'
created = False
if not os.path.lexists(target):
    fd, temporary = tempfile.mkstemp(prefix=".arm-clear-required-", dir=directory)
    try:
        os.fchmod(fd, 0o600)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "wb", closefd=True) as handle:
            fd = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        try:
            os.link(temporary, target, follow_symlinks=False)
            created = True
        except FileExistsError:
            pass
    finally:
        if fd >= 0:
            os.close(fd)
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass

result = os.lstat(target)
if not stat.S_ISREG(result.st_mode) or stat.S_ISLNK(result.st_mode):
    raise SystemExit("durable safety latch has the wrong type")
if stat.S_IMODE(result.st_mode) != 0o600:
    raise SystemExit("durable safety latch mode is not 0600")
if result.st_uid != uid or result.st_gid != gid:
    raise SystemExit("durable safety latch owner/group does not match the service account")
with open(target, "rb") as handle:
    raw = handle.read(1025)
if len(raw) > 1024:
    raise SystemExit("durable safety latch is unexpectedly large")
try:
    document = json.loads(raw.decode("utf-8"))
except (UnicodeError, ValueError) as error:
    raise SystemExit("durable safety latch is not valid UTF-8 JSON") from error
accepted_reasons = {
    "STOP_DELIVERY_UNKNOWN",
    "EXPLICIT_STOP",
    "MOVE_SET_FAILED",
    "MOVE_SET_UNCONFIRMED",
    "SAFETY_FAULT",
}
if (
    not isinstance(document, dict)
    or document.get("clearRequired") is not True
    or document.get("version") != 1
    or document.get("reason") not in accepted_reasons
):
    raise SystemExit("durable safety latch is not fail-closed")
if created:
    directory_fd = os.open(directory, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
PY
}

wait_for_health() {
  attempt=0
  while [ "$attempt" -lt 20 ]; do
    if curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null; then
      sudo systemctl is-active --quiet arm-gateway.service
      return 0
    fi
    attempt=$((attempt + 1))
    sleep 1
  done
  return 1
}

verify_dormant_mode() {
  current_drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
  [ -z "$current_drop_ins" ] || {
    echo "Dormant mode has unexpected service drop-ins: $current_drop_ins" >&2
    return 1
  }
  effective_exec_start=$(sudo systemctl show arm-gateway.service --property=ExecStart --value)
  case "$effective_exec_start" in
    *'--arm-controller-port'*)
      echo 'Dormant mode still exposes a physical arm-controller port.' >&2
      return 1
      ;;
  esac
  wait_for_health
}

verify_physical_state() {
  python3 - "$install_root" "$expected_controller_id" "$expected_firmware_version" <<'PY'
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

install_root, expected_controller_id, expected_firmware_version = sys.argv[1:]
token = Path(install_root, ".robot-gateway.token").read_text(encoding="utf-8").strip()
if not 32 <= len(token) <= 256 or any(ord(char) < 0x21 or ord(char) > 0x7E for char in token):
    raise SystemExit("installed gateway bearer token is invalid")
request = urllib.request.Request(
    "http://127.0.0.1:8787/api/robot/arm/state",
    headers={"Authorization": f"Bearer {token}"},
    method="GET",
)


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, file_pointer, code, message, headers, new_url):
        return None


opener = urllib.request.build_opener(
    urllib.request.ProxyHandler({}),
    NoRedirect(),
)
accepted_reasons = {
    "STOP_DELIVERY_UNKNOWN",
    "EXPLICIT_STOP",
    "MOVE_SET_FAILED",
    "MOVE_SET_UNCONFIRMED",
    "SAFETY_FAULT",
}


def fresh(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and 0 <= value <= 1000


def failures(state):
    problems = []
    if not isinstance(state, dict):
        return ["response is not a JSON object"]
    controller = state.get("controller")
    if not isinstance(controller, dict):
        return ["controller identity is absent"]
    if controller.get("controllerId") != expected_controller_id:
        problems.append("controller identity does not match")
    if controller.get("firmwareVersion") != expected_firmware_version:
        problems.append("controller firmware does not match")
    if not isinstance(controller.get("bootId"), str) or not controller.get("bootId"):
        problems.append("controller boot identity is absent")
    if controller.get("multiTurnAbsoluteV1") is not True:
        problems.append("native absolute multi-turn capability is absent")
    if controller.get("liveFollowV1") is not True:
        problems.append("live-follow capability is absent")
    if state.get("connection") != "online":
        problems.append("controller connection is not online")
    if state.get("bus") != "online":
        problems.append("servo bus is not online")
    telemetry_age = state.get("telemetryAgeMs")
    if not fresh(telemetry_age):
        problems.append("gateway telemetry is absent or older than 1000 ms")
    generation = state.get("telemetryGeneration")
    if not isinstance(generation, int) or isinstance(generation, bool) or generation <= 0:
        problems.append("telemetry generation is absent or invalid")
    if state.get("stopped") is not True:
        problems.append("STOP is not latched")
    if state.get("operatorInspectionRequired") is not True:
        problems.append("operator inspection latch is absent")
    if state.get("safetyStopReason") not in accepted_reasons:
        problems.append("safety latch reason is not accepted")
    if state.get("held") != []:
        problems.append("one or more joints are held")
    floor_guard = state.get("floorGuard")
    if not isinstance(floor_guard, dict) or floor_guard.get("enabled") is not True:
        problems.append("floor guard is not enabled")
    if state.get("collisionSuspected") is not False:
        problems.append("collision state is not explicitly clear")

    rows = state.get("joints")
    if not isinstance(rows, list) or len(rows) != 4:
        problems.append("exact four-joint telemetry is absent")
        return problems
    by_name = {row.get("id"): row for row in rows if isinstance(row, dict)}
    for servo_id in range(1, 5):
        name = f"joint_{servo_id}"
        row = by_name.get(name)
        if not isinstance(row, dict):
            problems.append(f"{name} telemetry is absent")
            continue
        if row.get("servoId") != servo_id:
            problems.append(f"{name} servo identity does not match")
        if row.get("online") is not True:
            problems.append(f"{name} is not online")
        if row.get("calibrated") is not True:
            problems.append(f"{name} is not calibrated")
        if row.get("torque") != "off":
            problems.append(f"{name} torque is not off")
        if row.get("moving") is not False:
            problems.append(f"{name} is not explicitly stationary")
        packet_age = row.get("packetAgeMs")
        if not fresh(packet_age):
            problems.append(f"{name} telemetry is absent or older than 1000 ms")
        elif fresh(telemetry_age) and packet_age + telemetry_age > 1000:
            problems.append(f"{name} combined telemetry age is older than 1000 ms")
    return problems


last = ["no authenticated state received"]
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    try:
        with opener.open(request, timeout=2) as response:
            payload = json.loads(response.read(1_000_000))
        last = failures(payload)
        if not last:
            print(
                "Authenticated controller safety proof passed: expected controller, firmware, "
                "native multi-turn and live-follow capabilities, four fresh torque-off stationary "
                "joints, STOP and inspection latched, floor guard enabled, and collision explicitly clear."
            )
            raise SystemExit(0)
    except (OSError, UnicodeError, ValueError, urllib.error.URLError):
        last = ["authenticated state request was unavailable or invalid"]
    time.sleep(1)

print("Controller safety proof failed: " + "; ".join(last), file=sys.stderr)
raise SystemExit(60)
PY
}

drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
drop_in_present=0
if sudo test -L "$drop_in" || { sudo test -e "$drop_in" && ! sudo test -f "$drop_in"; }; then
  echo 'The physical UART drop-in path has the wrong type.' >&2
  exit 11
elif sudo test -f "$drop_in"; then
  drop_in_present=1
  [ "$(sudo stat -c '%u:%g:%a' -- "$drop_in")" = '0:0:644' ] || {
    echo 'The installed UART drop-in must be root-owned, root-grouped, and mode 0644.' >&2
    exit 11
  }
fi

case "$mode" in
  Activate)
    device=$(readlink -f /dev/serial0 2>/dev/null || true)
    [ -n "$device" ] || { echo '/dev/serial0 is unavailable.' >&2; exit 20; }
    if grep -Eq '(^| )(console=(serial0|ttyAMA0|ttyS0),[^ ]+)' /boot/firmware/cmdline.txt; then
      echo 'Refusing activation while a serial console remains.' >&2
      exit 21
    fi
    grep -Eq '^enable_uart=1([[:space:]]|$)' /boot/firmware/config.txt || {
      echo 'Refusing activation because enable_uart=1 is absent.' >&2
      exit 22
    }
    case "$drop_ins|$drop_in_present" in
      '|0') activation_state=dormant ;;
      "$drop_in|1") activation_state=physical ;;
      *) echo "Refusing activation with an unexpected drop-in state: $drop_ins" >&2; exit 23 ;;
    esac
    if [ "$drop_in_present" -eq 1 ]; then
      [ "$(sudo sha256sum -- "$drop_in" | awk '{print $1}')" = "$expected_drop_in_sha256" ] || {
        echo 'The installed UART drop-in differs from reviewed source.' >&2
        exit 24
      }
    fi
    ensure_fail_closed_latch
    activated_now=0
    if [ "$activation_state" = dormant ]; then
      activation_rollback_required=1
      sudo systemctl stop arm-gateway.service
      sudo install -d -m 0755 -o root -g root /etc/systemd/system/arm-gateway.service.d
      sudo install -m 0644 -o root -g root "$staged_drop_in" "$drop_in"
      sudo systemctl daemon-reload
      sudo systemctl restart arm-gateway.service
      activated_now=1
    fi
    current_drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
    if [ "$current_drop_ins" != "$drop_in" ] ||
       [ "$(sudo sha256sum -- "$drop_in" | awk '{print $1}')" != "$expected_drop_in_sha256" ]; then
      echo 'Physical mode did not load only the reviewed UART drop-in.' >&2
      exit 25
    fi
    if ! wait_for_health || ! verify_physical_state; then
      exit 60
    fi
    activation_rollback_required=0
    if [ "$activated_now" -eq 1 ]; then
      echo "Physical UART activation passed at $device after one deliberate restart."
    else
      echo "Physical UART mode was already correct at $device; no restart was performed."
    fi
    ;;
  Deactivate)
    case "$drop_ins|$drop_in_present" in
      '|0') deactivation_state=dormant ;;
      "$drop_in|1") deactivation_state=physical ;;
      *) echo "Refusing deactivation with an unexpected drop-in state: $drop_ins" >&2; exit 30 ;;
    esac
    ensure_fail_closed_latch
    if [ "$deactivation_state" = physical ]; then
      [ "$(sudo sha256sum -- "$drop_in" | awk '{print $1}')" = "$expected_drop_in_sha256" ] || {
        echo 'Refusing deactivation because the installed UART drop-in differs from reviewed source.' >&2
        exit 31
      }
      sudo systemctl stop arm-gateway.service
      sudo rm -- "$drop_in"
      sudo systemctl daemon-reload
      sudo systemctl restart arm-gateway.service
      if ! verify_dormant_mode; then
        echo 'Dormant gateway did not become healthy after UART deactivation.' >&2
        exit 32
      fi
      sudo test -f "$latch" && ! sudo test -L "$latch" || {
        echo 'UART deactivation did not retain the fail-closed latch.' >&2
        exit 33
      }
      echo 'Physical UART deactivation passed after one deliberate dormant restart; the fail-closed latch was retained.'
    else
      verify_dormant_mode || { echo 'Existing dormant mode failed verification.' >&2; exit 34; }
      echo 'Dormant gateway mode was already correct; no restart was performed and the fail-closed latch remains.'
    fi
    ;;
esac

echo 'No UART boot configuration, reboot, deployment, STOP clear, torque command, motion command, or camera action was issued.'
'@

    $modePayload = ($modeScript -replace "`r", '') + "`n#"
    $remoteCommand = (
        "sh -s -- '$remoteStage' '$Mode' '$UserName' '$ServiceGroup' " +
        "'$installRoot' '$ExpectedControllerId' '$ExpectedFirmwareVersion' " +
        "'$expectedBaseSha256' '$expectedDropInSha256'"
    )
    $modePayload | & $sshPath @sshArguments $remoteTarget $remoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "$Mode failed with exit code $LASTEXITCODE."
    }
}
finally {
    if ($stagePrepared) {
        $cleanupScript = @'
set -eu
stage=$1
case "$stage" in /tmp/co-arm-gateway-mode-[0-9a-f]*) rm -rf -- "$stage" ;; *) exit 2 ;; esac
'@
        try {
            $cleanupPayload = ($cleanupScript -replace "`r", '') + "`n#"
            $cleanupPayload | & $sshPath @sshArguments $remoteTarget "sh -s -- '$remoteStage'" 2>$null
        }
        catch {
            Write-Warning 'Remote mode staging cleanup could not be confirmed.'
        }
    }
    if (Test-Path -LiteralPath $localStage -PathType Container) {
        $temporaryRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath()).TrimEnd('\', '/')
        $localStageFull = [System.IO.Path]::GetFullPath($localStage)
        $temporaryPrefix = $temporaryRoot + [System.IO.Path]::DirectorySeparatorChar
        if (-not $localStageFull.StartsWith(
                $temporaryPrefix,
                [System.StringComparison]::OrdinalIgnoreCase
            )) {
            throw "Refusing to clean an unexpected local staging directory: $localStageFull"
        }
        Remove-Item -LiteralPath $localStageFull -Recurse -Force
    }
}
