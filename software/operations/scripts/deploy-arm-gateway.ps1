#Requires -Version 5.1

<#
.SYNOPSIS
Deploys the camera and disarmed physical commissioning gateway to the Pi.

.DESCRIPTION
Copies only robot_gateway/*.py, requirements-pi.txt, and the reviewed systemd
unit. The default service stays on Pi loopback and leaves physical UART dormant.
`-PreservePhysicalUart` permits an in-place Pi package refresh only when the
sole active drop-in is the byte-exact reviewed physical-UART file already on the
Pi; it never installs, removes, or changes that drop-in. The deployer prepares
`dialout`/persistent state and retains profiles under /var/lib/arm-gateway. It
never enables UART/GPIO, flashes firmware, enables torque, or prints the bearer
token. The pi account must have non-interactive sudo access.

The remote install stops and restarts arm-gateway.service. That interruption can
expire a live hold/watchdog and release torque, so a gravity-loaded mechanism
may sag. Put the physical arm in a supported safe state before deployment.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $HostName,

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

    [ValidateRange(1, 65535)]
    [int] $Port = 22,

    [switch] $PreservePhysicalUart
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($HostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$' -or $HostName.StartsWith('-')) {
    throw 'HostName must be a DNS name, IPv4 address, or unbracketed IPv6 literal.'
}
$installRoot = "/home/$UserName/arm-gateway"

$sshPath = (Get-Command ssh -CommandType Application -ErrorAction Stop).Source
$scpPath = (Get-Command scp -CommandType Application -ErrorAction Stop).Source
$identityPath = (Resolve-Path -LiteralPath $IdentityFile -ErrorAction Stop).Path
$knownHostsPath = (Resolve-Path -LiteralPath $KnownHostsFile -ErrorAction Stop).Path

foreach ($path in @($identityPath, $knownHostsPath)) {
    if ((Get-Item -LiteralPath $path).PSIsContainer) {
        throw "Expected a file, received a directory: $path"
    }
}

$softwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$operationsRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$packageDirectory = Join-Path $softwareRoot 'python\robot_gateway'
$requirementsPath = Join-Path $operationsRoot 'requirements-pi.txt'
$unitPath = Join-Path $operationsRoot 'deploy\arm-gateway.service'
$physicalUartDropInPath = Join-Path $operationsRoot 'deploy\arm-gateway-physical-uart.conf'
$renderRoot = Join-Path $env:TEMP ("arm-gateway-render-" + [Guid]::NewGuid().ToString('N'))
$renderedUnitPath = Join-Path $renderRoot 'arm-gateway.service'
$renderedPhysicalUartDropInPath = Join-Path $renderRoot 'arm-gateway-physical-uart.conf'
$expectedGatewayModules = @(
    '__init__.py',
    '__main__.py',
    'arm_controller.py',
    'camera_api.py',
    'camera_profiles.py',
    'isaac_bridge.py',
    'physical_arm_api.py',
    'pi_camera.py',
    'request_validation.py',
    'runtime.py',
    'serial_arm_controller.py',
    'simple_arm_api.py',
    'strict_contract.py'
)
$pythonFiles = @(Get-ChildItem -LiteralPath $packageDirectory -File -Filter '*.py' | Sort-Object Name)

$actualGatewayModules = @($pythonFiles | ForEach-Object { $_.Name })
$reviewedGatewayModules = @($expectedGatewayModules | Sort-Object)
if ($actualGatewayModules.Count -ne $reviewedGatewayModules.Count -or
    [string]::Join("`n", $actualGatewayModules) -cne
        [string]::Join("`n", $reviewedGatewayModules)) {
    throw (
        'Expected exactly the reviewed 13 robot_gateway Python modules. ' +
        "Expected: $($reviewedGatewayModules -join ', '). " +
        "Actual: $($actualGatewayModules -join ', ')."
    )
}
foreach ($path in @($requirementsPath, $unitPath, $physicalUartDropInPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required deployment input is missing: $path"
    }
}

New-Item -ItemType Directory -Path $renderRoot -ErrorAction Stop | Out-Null
$unitText = (Get-Content -Raw -LiteralPath $unitPath).
    Replace('__SERVICE_USER__', $UserName).
    Replace('__SERVICE_GROUP__', $ServiceGroup).
    Replace('__INSTALL_ROOT__', $installRoot)
$dropInText = (Get-Content -Raw -LiteralPath $physicalUartDropInPath).
    Replace('__INSTALL_ROOT__', $installRoot)
foreach ($renderedText in @($unitText, $dropInText)) {
    if ($renderedText -match '__[A-Z0-9_]+__') {
        throw 'A deployment template contains an unresolved placeholder.'
    }
}
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($renderedUnitPath, $unitText, $utf8NoBom)
[System.IO.File]::WriteAllText($renderedPhysicalUartDropInPath, $dropInText, $utf8NoBom)

$unpinned = @(
    Get-Content -LiteralPath $requirementsPath |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ -and -not $_.StartsWith('#') } |
        Where-Object {
            $_ -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]*(\[[A-Za-z0-9_,.-]+\])?==[^;\s]+(?:\s*;.+)?$'
        }
)
if ($unpinned.Count -gt 0) {
    throw 'requirements-pi.txt contains an unpinned or unsupported requirement.'
}

$remoteHost = if ($HostName.Contains(':')) { "[$HostName]" } else { $HostName }
$remoteTarget = "${UserName}@${remoteHost}"
$remoteStage = "/tmp/arm-gateway-deploy-$([Guid]::NewGuid().ToString('N'))"

# OpenSSH treats UserKnownHostsFile as a space-separated LIST of files, so a
# workspace path containing spaces is read as several nonexistent ones and every
# host key comes back unknown. Quoting inside the option value is the documented
# escape, but PowerShell 5.1 does not pass embedded quotes through to a native
# command intact -- so the file is staged somewhere without spaces instead.
$quotedKnownHostsPath = $knownHostsPath
$stagedKnownHostsPath = $null
if ($knownHostsPath -match '\s') {
    $stagedKnownHostsPath = Join-Path $env:TEMP ("known-hosts-" + [Guid]::NewGuid().ToString('N'))
    Copy-Item -LiteralPath $knownHostsPath -Destination $stagedKnownHostsPath -Force
    $quotedKnownHostsPath = $stagedKnownHostsPath
}

$sshArguments = @(
    '-p', $Port.ToString(),
    '-i', $identityPath,
    '-o', "UserKnownHostsFile=$quotedKnownHostsPath",
    '-o', 'StrictHostKeyChecking=yes',
    '-o', 'IdentitiesOnly=yes',
    '-o', 'BatchMode=yes',
    '-o', 'PasswordAuthentication=no'
)
$scpArguments = @(
    '-q',
    '-P', $Port.ToString(),
    '-i', $identityPath,
    '-o', "UserKnownHostsFile=$quotedKnownHostsPath",
    '-o', 'StrictHostKeyChecking=yes',
    '-o', 'IdentitiesOnly=yes',
    '-o', 'BatchMode=yes',
    '-o', 'PasswordAuthentication=no'
)

function Invoke-RemoteScript {
    param(
        [Parameter(Mandatory)] [string] $ScriptText,
        [Parameter(Mandatory)] [string] $Operation,
        [ValidateSet('dormant', 'physical')] [string] $DeploymentMode = 'dormant'
    )

    # Windows PowerShell appends CRLF when piping text to a native process.
    # Normalize embedded line endings and end on a comment so the appended CR
    # cannot become part of the final POSIX shell token.
    $remotePayload = ($ScriptText -replace "`r", '') + "`n#"
    $remotePayload | & $sshPath @sshArguments $remoteTarget (
        "sh -s -- '$remoteStage' '$DeploymentMode' '$installRoot' '$UserName' '$ServiceGroup'"
    )
    if ($LASTEXITCODE -ne 0) {
        throw "$Operation failed with exit code $LASTEXITCODE."
    }
}

$prepareScript = @'
set -eu
stage=$1
case "$stage" in
  /tmp/arm-gateway-deploy-[0-9a-f]*) ;;
  *) echo 'Refusing unexpected staging path.' >&2; exit 2 ;;
esac
umask 077
mkdir -- "$stage"
mkdir -- "$stage/robot_gateway"
'@

$installScript = @'
set -eu
stage=$1
deployment_mode=$2
root=$3
service_user=$4
service_group=$5
case "$stage" in
  /tmp/arm-gateway-deploy-[0-9a-f]*) ;;
  *) echo 'Refusing unexpected staging path.' >&2; exit 2 ;;
esac
case "$root" in
  /home/*/arm-gateway) ;;
  *) echo 'Refusing unexpected install root.' >&2; exit 2 ;;
esac
case "$service_user:$service_group" in
  *[!a-z0-9_:-]*) echo 'Refusing unexpected service identity.' >&2; exit 2 ;;
esac
package="$root/robot_gateway"
token="$root/.robot-gateway.token"
state=/var/lib/arm-gateway
service=arm-gateway.service

sudo -n true
sudo install -d -m 0750 -o "$service_user" -g "$service_group" "$root"
sudo install -d -m 0750 -o "$service_user" -g "$service_group" "$package"
sudo install -d -m 0700 -o "$service_user" -g "$service_group" "$state"
drop_ins=$(sudo systemctl show "$service" --property=DropInPaths --value 2>/dev/null || true)
if [ -n "$drop_ins" ]; then
  expected_drop_in=/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
  if [ "$deployment_mode" != physical ]; then
    echo "Refusing dormant base deployment while systemd drop-ins are active: $drop_ins" >&2
    echo 'Remove the reviewed physical-UART activation explicitly before redeploying dormant mode.' >&2
    exit 5
  fi
  [ "$drop_ins" = "$expected_drop_in" ] || {
    echo "Refusing physical refresh with unexpected systemd drop-ins: $drop_ins" >&2
    exit 6
  }
  [ -f "$expected_drop_in" ] && [ ! -L "$expected_drop_in" ] || {
    echo 'Refusing physical refresh because the UART drop-in is missing or has the wrong type.' >&2
    exit 7
  }
  cmp -s "$stage/arm-gateway-physical-uart.conf" "$expected_drop_in" || {
    echo 'Refusing physical refresh because the installed UART drop-in differs from reviewed source.' >&2
    exit 8
  }
elif [ "$deployment_mode" = physical ]; then
  echo 'Refusing physical refresh because no active UART drop-in exists to preserve.' >&2
  exit 9
fi
sudo systemctl stop "$service" 2>/dev/null || true
sudo find "$package" -maxdepth 1 -type f -name '*.py' -delete

for source in "$stage"/robot_gateway/*.py; do
  sudo install -m 0640 -o "$service_user" -g "$service_group" "$source" "$package/$(basename "$source")"
done
sudo install -m 0640 -o "$service_user" -g "$service_group" "$stage/requirements-pi.txt" "$root/requirements-pi.txt"

sudo -u "$service_user" -H python3 -m venv --system-site-packages "$root/.venv"
sudo -u "$service_user" -H "$root/.venv/bin/python" -m pip install \
  --disable-pip-version-check --requirement "$root/requirements-pi.txt"

cd "$root"
if [ ! -e "$token" ] && [ ! -L "$token" ]; then
  sudo -u "$service_user" -H "$root/.venv/bin/python" -m robot_gateway generate-token \
    --output "$token" >/dev/null
fi
if [ ! -f "$token" ] || [ -L "$token" ]; then
  echo 'Token path must be an existing regular file, not a symbolic link.' >&2
  exit 3
fi
sudo chown "$service_user:$service_group" "$token"
sudo chmod 0600 "$token"

sudo install -m 0644 -o root -g root "$stage/arm-gateway.service" \
  /etc/systemd/system/arm-gateway.service
sudo systemctl daemon-reload
sudo systemctl enable "$service" >/dev/null
sudo systemctl start "$service"

attempt=0
while [ "$attempt" -lt 20 ]; do
  if curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null; then
    sudo systemctl is-active --quiet "$service"
    echo 'Arm gateway is active and healthy on Pi loopback.'
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 1
done

echo 'Arm gateway did not become healthy on Pi loopback.' >&2
sudo systemctl --no-pager --full status "$service" >&2 || true
exit 4
'@

$cleanupScript = @'
set -eu
stage=$1
case "$stage" in
  /tmp/arm-gateway-deploy-[0-9a-f]*) ;;
  *) exit 2 ;;
esac
[ ! -d "$stage" ] || find "$stage" -depth -type f -delete
[ ! -d "$stage" ] || find "$stage" -depth -type d -empty -delete
'@

Write-Warning (
    'Deployment stops and restarts arm-gateway.service. A live hold/watchdog ' +
    'can expire and release torque; support any gravity-loaded mechanism first.'
)

$stagePrepared = $false
try {
    Invoke-RemoteScript -ScriptText $prepareScript -Operation 'Remote staging preparation'
    $stagePrepared = $true

    $pythonSources = @($pythonFiles | ForEach-Object { $_.FullName })
    $arguments = $scpArguments + $pythonSources + @("${remoteTarget}:$remoteStage/robot_gateway/")
    & $scpPath @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Python package upload failed.' }

    $arguments = $scpArguments + @($requirementsPath, "${remoteTarget}:$remoteStage/requirements-pi.txt")
    & $scpPath @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Requirements upload failed.' }

    $arguments = $scpArguments + @($renderedUnitPath, "${remoteTarget}:$remoteStage/arm-gateway.service")
    & $scpPath @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Systemd unit upload failed.' }

    $arguments = $scpArguments + @(
        $renderedPhysicalUartDropInPath,
        "${remoteTarget}:$remoteStage/arm-gateway-physical-uart.conf"
    )
    & $scpPath @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Physical UART verification input upload failed.' }

    $deploymentMode = if ($PreservePhysicalUart) { 'physical' } else { 'dormant' }
    Invoke-RemoteScript `
        -ScriptText $installScript `
        -Operation 'Remote gateway installation' `
        -DeploymentMode $deploymentMode
}
finally {
    if ($stagePrepared) {
        $cleanupPayload = ($cleanupScript -replace "`r", '') + "`n#"
        $cleanupPayload | & $sshPath @sshArguments $remoteTarget "sh -s -- '$remoteStage'" 2>$null
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Remote staging cleanup failed at $remoteStage."
        }
    }
    if ($stagedKnownHostsPath -and (Test-Path -LiteralPath $stagedKnownHostsPath)) {
        Remove-Item -LiteralPath $stagedKnownHostsPath -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path -LiteralPath $renderRoot -PathType Container) {
        $temporaryRoot = [System.IO.Path]::GetFullPath($env:TEMP).TrimEnd('\', '/')
        $renderFull = [System.IO.Path]::GetFullPath($renderRoot)
        if (-not $renderFull.StartsWith(
            $temporaryRoot + [System.IO.Path]::DirectorySeparatorChar,
            [System.StringComparison]::OrdinalIgnoreCase
        )) {
            throw "Refusing to clean an unexpected render directory: $renderFull"
        }
        Get-ChildItem -LiteralPath $renderFull -File | Remove-Item -Force
        Remove-Item -LiteralPath $renderFull -Force
    }
}
