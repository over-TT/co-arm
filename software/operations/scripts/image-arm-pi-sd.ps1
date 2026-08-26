#Requires -Version 5.1

<#
.SYNOPSIS
Creates a private, exact block image of the offline replacement arm-Pi SD card.

.DESCRIPTION
This command is intentionally narrower than a normal disk-imaging utility. It
works only when arm-pi is booted from the known old USB device (/dev/sda) and
the replacement SD (/dev/mmcblk0) has no mountpoint. The SD is read once with
dd, compressed on the Pi, streamed over pinned noninteractive SSH, validated
to gzip EOF on the laptop, hashed, and atomically published beneath the ignored
runtime directory.

The command never mounts or writes the media and never changes boot state,
services, firmware, or arm hardware. During the read it temporarily sets the
offline whole block device's kernel read-only flag, verifies the lock, and
trap-restores the original flag. If identity or offline proof changes between
preflight, lock, read, and postflight, it refuses to publish the image.

The durable identity anchor uses schema arm-pi-replacement-sd-identity.v1:
device is /dev/mmcblk0, cid is the kernel-reported 32-character lowercase MMC
CID, sizeBytes is the exact raw byte count, and partitions contains plain
string values named mmcblk0p1 and mmcblk0p2 for their expected PARTUUIDs.
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
    [ValidateNotNullOrEmpty()]
    [string] $IdentityFile,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $KnownHostsFile,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $IdentityAnchorFile,

    [ValidateRange(1, 65535)]
    [int] $Port = 22,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $ExpectedHostName,

    [Parameter(Mandatory)]
    [ValidatePattern('^/dev/[A-Za-z0-9._/+:-]+$')]
    [string] $ExpectedRootDevice,

    [Parameter(Mandatory)]
    [ValidatePattern('^/dev/[A-Za-z0-9._/+:-]+$')]
    [string] $ExpectedBootDevice
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$targetDevice = '/dev/mmcblk0'

if ($HostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$' -or $HostName.StartsWith('-')) {
    throw 'HostName must be a DNS name, IPv4 address, or unbracketed IPv6 literal.'
}
if ($ExpectedHostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$' -or $ExpectedHostName.StartsWith('-')) {
    throw 'ExpectedHostName must be a plain hostname.'
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

function Set-PrivateDirectory {
    param([Parameter(Mandatory)] [string] $Path)

    $currentSid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User
    if ($null -eq $currentSid) {
        throw 'Could not resolve the current Windows user SID.'
    }
    $security = New-Object System.Security.AccessControl.DirectorySecurity
    $security.SetOwner($currentSid)
    $security.SetAccessRuleProtection($true, $false)
    $inheritance = (
        [System.Security.AccessControl.InheritanceFlags]::ContainerInherit -bor
        [System.Security.AccessControl.InheritanceFlags]::ObjectInherit
    )
    $rule = New-Object System.Security.AccessControl.FileSystemAccessRule(
        $currentSid,
        [System.Security.AccessControl.FileSystemRights]::FullControl,
        $inheritance,
        [System.Security.AccessControl.PropagationFlags]::None,
        [System.Security.AccessControl.AccessControlType]::Allow
    )
    [void] $security.AddAccessRule($rule)
    Set-Acl -LiteralPath $Path -AclObject $security
}

function Remove-IncompleteImageDirectory {
    param(
        [Parameter(Mandatory)] [string] $Path,
        [Parameter(Mandatory)] [string] $ImageRoot
    )

    $rootFull = [System.IO.Path]::GetFullPath($ImageRoot).TrimEnd('\', '/')
    $candidateFull = [System.IO.Path]::GetFullPath($Path)
    $requiredPrefix = $rootFull + [System.IO.Path]::DirectorySeparatorChar
    if (-not $candidateFull.StartsWith(
        $requiredPrefix,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Refusing to clean an image path outside the private image root: $candidateFull"
    }
    if (Test-Path -LiteralPath $candidateFull -PathType Container) {
        Remove-Item -LiteralPath $candidateFull -Recurse -Force
    }
}

function ConvertTo-NativeArgument {
    param([Parameter(Mandatory)] [AllowEmptyString()] [string] $Argument)

    if ($Argument.Length -gt 0 -and $Argument -notmatch '[\s"]') {
        return $Argument
    }

    $builder = New-Object System.Text.StringBuilder
    [void] $builder.Append('"')
    $backslashes = 0
    foreach ($character in $Argument.ToCharArray()) {
        if ($character -eq [char] 92) {
            $backslashes += 1
            continue
        }
        if ($character -eq [char] 34) {
            [void] $builder.Append(('\' * (($backslashes * 2) + 1)))
            [void] $builder.Append('"')
            $backslashes = 0
            continue
        }
        if ($backslashes -gt 0) {
            [void] $builder.Append(('\' * $backslashes))
            $backslashes = 0
        }
        [void] $builder.Append($character)
    }
    if ($backslashes -gt 0) {
        [void] $builder.Append(('\' * ($backslashes * 2)))
    }
    [void] $builder.Append('"')
    return $builder.ToString()
}

function Start-SshScriptProcess {
    param(
        [Parameter(Mandatory)] [string] $SshPath,
        [Parameter(Mandatory)] [string[]] $SshArguments,
        [Parameter(Mandatory)] [string] $RemoteTarget,
        [Parameter(Mandatory)] [string] $RemoteCommand,
        [Parameter(Mandatory)] [string] $ScriptText
    )

    $arguments = @($SshArguments) + @($RemoteTarget, $RemoteCommand)
    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $SshPath
    $startInfo.Arguments = (
        $arguments | ForEach-Object { ConvertTo-NativeArgument -Argument $_ }
    ) -join ' '
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardInput = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo
    if (-not $process.Start()) {
        throw 'Could not start the pinned SSH process.'
    }
    try {
        $payload = ($ScriptText -replace "`r", '') + "`n#`n"
        $process.StandardInput.Write($payload)
        $process.StandardInput.Close()
    }
    catch {
        if (-not $process.HasExited) {
            $process.Kill()
            $process.WaitForExit()
        }
        $process.Dispose()
        throw
    }
    return $process
}

function Invoke-SshScriptText {
    param(
        [Parameter(Mandatory)] [string] $SshPath,
        [Parameter(Mandatory)] [string[]] $SshArguments,
        [Parameter(Mandatory)] [string] $RemoteTarget,
        [Parameter(Mandatory)] [string] $RemoteCommand,
        [Parameter(Mandatory)] [string] $ScriptText
    )

    $process = Start-SshScriptProcess `
        -SshPath $SshPath `
        -SshArguments $SshArguments `
        -RemoteTarget $RemoteTarget `
        -RemoteCommand $RemoteCommand `
        -ScriptText $ScriptText
    try {
        $outputTask = $process.StandardOutput.ReadToEndAsync()
        $errorTask = $process.StandardError.ReadToEndAsync()
        $process.WaitForExit()
        $output = $outputTask.GetAwaiter().GetResult()
        $remoteError = $errorTask.GetAwaiter().GetResult()
        if ($process.ExitCode -ne 0) {
            $message = switch ($process.ExitCode) {
                42 { 'The pinned SSH host is not the expected Raspberry Pi USB boot.' }
                43 { 'The replacement SD is absent, is not the exact expected disk, or is in use.' }
                44 { 'The replacement SD identity could not be captured safely.' }
                default { "Offline SD preflight failed with exit code $($process.ExitCode)." }
            }
            if ($remoteError -match 'Permission denied|Host key verification failed|Connection refused|timed out') {
                $message += ' Check the pinned host key, dedicated identity, SSH service, and direct Ethernet link.'
            }
            throw $message
        }
        return $output
    }
    finally {
        if (-not $process.HasExited) {
            $process.Kill()
            $process.WaitForExit()
        }
        $process.Dispose()
    }
}

function ConvertFrom-PreflightRecord {
    param([Parameter(Mandatory)] [string] $Text)

    $record = @{}
    foreach ($line in ($Text -split "`r?`n")) {
        if (-not $line) { continue }
        if ($line -notmatch '^([A-Z0-9_]+)=(.*)$') {
            throw 'The Pi returned an invalid offline-SD preflight record.'
        }
        $key = $Matches[1]
        if ($record.ContainsKey($key)) {
            throw "The Pi returned duplicate preflight field $key."
        }
        $record[$key] = $Matches[2]
    }
    foreach ($key in @(
        'SCHEMA', 'HOSTNAME_B64', 'USER_B64', 'PI_MODEL_B64',
        'ROOT_SOURCE_B64', 'BOOT_SOURCE_B64', 'ROOT_DISK_B64', 'BOOT_DISK_B64',
        'TARGET_PATH_B64', 'TARGET_KNAME_B64', 'TARGET_TYPE_B64', 'TARGET_SIZE',
        'TARGET_MODEL_B64', 'TARGET_SERIAL_B64', 'TARGET_WWN_B64',
        'TARGET_PTUUID_B64', 'TARGET_MAJMIN_B64', 'TARGET_RO', 'TARGET_RM',
        'TARGET_CID', 'TARGET_P1_PARTUUID_B64', 'TARGET_P2_PARTUUID_B64',
        'IDENTITY_RECORD_B64', 'IDENTITY_SHA256', 'LSBLK_TREE_B64',
        'CAPTURED_AT_B64'
    )) {
        if (-not $record.ContainsKey($key)) {
            throw "The Pi preflight record is missing $key."
        }
    }
    if ($record['SCHEMA'] -ne 'arm-pi-offline-sd-preflight.v2') {
        throw 'The Pi returned an unsupported preflight schema.'
    }
    if ($record['TARGET_SIZE'] -notmatch '^[1-9][0-9]+$') {
        throw 'The Pi returned an invalid SD byte size.'
    }
    if ($record['IDENTITY_SHA256'] -notmatch '^[0-9a-f]{64}$') {
        throw 'The Pi returned an invalid SD identity digest.'
    }
    if ($record['TARGET_CID'] -cnotmatch '^[0-9a-f]{32}$') {
        throw 'The Pi returned an invalid SD CID.'
    }
    return $record
}

function ConvertFrom-Base64Utf8 {
    param([Parameter(Mandatory)] [AllowEmptyString()] [string] $Value)

    if ($Value.Length -eq 0) { return '' }
    try {
        return [System.Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($Value))
    }
    catch {
        throw 'The Pi returned invalid base64 metadata.'
    }
}

function Read-ReplacementSdIdentityAnchor {
    param([Parameter(Mandatory)] [string] $Path)

    try {
        $anchor = Get-Content -Raw -LiteralPath $Path -ErrorAction Stop | ConvertFrom-Json
    }
    catch {
        throw 'The replacement-SD identity anchor is not valid JSON.'
    }
    if ($null -eq $anchor -or
        [string]$anchor.schema -cne 'arm-pi-replacement-sd-identity.v1' -or
        [string]$anchor.device -cne '/dev/mmcblk0') {
        throw 'The replacement-SD identity anchor has an unsupported schema or device.'
    }

    $cid = [string]$anchor.cid
    if ($cid -cnotmatch '^[0-9a-f]{32}$') {
        throw 'The replacement-SD identity anchor CID must be exactly 32 lowercase hexadecimal characters.'
    }
    try {
        $sizeBytes = [Convert]::ToInt64([string]$anchor.sizeBytes)
    }
    catch {
        throw 'The replacement-SD identity anchor contains an invalid sizeBytes value.'
    }
    if ($sizeBytes -le 0) {
        throw 'The replacement-SD identity anchor sizeBytes must be positive.'
    }
    if ($null -eq $anchor.partitions) {
        throw 'The replacement-SD identity anchor is missing partition PARTUUIDs.'
    }
    $p1Partuuid = ([string]$anchor.partitions.mmcblk0p1).ToLowerInvariant()
    $p2Partuuid = ([string]$anchor.partitions.mmcblk0p2).ToLowerInvariant()
    foreach ($partuuid in @($p1Partuuid, $p2Partuuid)) {
        if ($partuuid -notmatch '^[0-9a-f][0-9a-f-]{3,127}$') {
            throw 'The replacement-SD identity anchor contains an invalid partition PARTUUID.'
        }
    }
    if ($p1Partuuid -eq $p2Partuuid) {
        throw 'The replacement-SD identity anchor partition PARTUUIDs must be distinct.'
    }
    return [pscustomobject]@{
        Cid = $cid
        SizeBytes = $sizeBytes
        P1Partuuid = $p1Partuuid
        P2Partuuid = $p2Partuuid
    }
}

function Read-GzipImageEvidence {
    param([Parameter(Mandatory)] [string] $Path)

    $file = [System.IO.File]::Open(
        $Path,
        [System.IO.FileMode]::Open,
        [System.IO.FileAccess]::Read,
        [System.IO.FileShare]::Read
    )
    $gzip = $null
    $sha = $null
    try {
        $gzip = New-Object System.IO.Compression.GZipStream(
            $file,
            [System.IO.Compression.CompressionMode]::Decompress,
            $false
        )
        $sha = [System.Security.Cryptography.SHA256]::Create()
        $buffer = New-Object byte[] (4 * 1024 * 1024)
        [long] $total = 0
        while (($read = $gzip.Read($buffer, 0, $buffer.Length)) -gt 0) {
            [void] $sha.TransformBlock($buffer, 0, $read, $buffer, 0)
            $total += $read
        }
        [void] $sha.TransformFinalBlock((New-Object byte[] 0), 0, 0)
        $rawHash = ([BitConverter]::ToString($sha.Hash)).Replace('-', '').ToLowerInvariant()
        return [pscustomobject]@{
            DecompressedBytes = $total
            RawImageSha256 = $rawHash
        }
    }
    catch {
        throw 'The downloaded image failed local gzip validation.'
    }
    finally {
        if ($null -ne $gzip) { $gzip.Dispose() }
        else { $file.Dispose() }
        if ($null -ne $sha) { $sha.Dispose() }
    }
}

$sshPath = (Get-Command ssh -CommandType Application -ErrorAction Stop).Source
$identityPath = Resolve-RequiredLeaf -Path $IdentityFile -Label 'IdentityFile'
$knownHostsPath = Resolve-RequiredLeaf -Path $KnownHostsFile -Label 'KnownHostsFile'
$identityAnchorPath = Resolve-RequiredLeaf -Path $IdentityAnchorFile -Label 'IdentityAnchorFile'
$identityAnchor = Read-ReplacementSdIdentityAnchor -Path $identityAnchorPath
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$imageRoot = Join-Path $repositoryRoot 'runtime\robot-gateway\pi-disk-images'

$preflightScript = @'
set -eu

expected_hostname=$1
expected_root=$2
expected_boot=$3
expected_user=$4
expected_cid=$5
expected_size=$6
expected_p1_partuuid=$7
expected_p2_partuuid=$8
target=/dev/mmcblk0

identity_failure() {
  printf '%s\n' 'Remote identity is not the expected arm Pi USB boot.' >&2
  exit 42
}
target_failure() {
  printf '%s\n' 'The replacement SD is absent, unexpected, mounted, or backs the running OS.' >&2
  exit 43
}

actual_hostname=$(hostname) || identity_failure
actual_user=$(id -un) || identity_failure
root_source=$(findmnt -n -o SOURCE -- /) || identity_failure
boot_source=$(findmnt -n -o SOURCE -- /boot/firmware) || identity_failure
pi_model=$(tr -d '\000' < /proc/device-tree/model 2>/dev/null) || identity_failure

[ "$actual_hostname" = "$expected_hostname" ] || identity_failure
[ "$actual_user" = "$expected_user" ] || identity_failure
[ "$root_source" = "$expected_root" ] || identity_failure
[ "$boot_source" = "$expected_boot" ] || identity_failure
case "$pi_model" in Raspberry\ Pi*) ;; *) identity_failure ;; esac
sudo -n true

[ -b "$target" ] || target_failure
[ "$(readlink -f "$target")" = "$target" ] || target_failure
target_type=$(lsblk -dn -o TYPE -- "$target") || target_failure
target_type=$(printf '%s' "$target_type" | tr -d '[:space:]')
[ "$target_type" = disk ] || target_failure

backing_disk() {
  source=$1
  parent=$(lsblk -dn -o PKNAME -- "$source") || return 1
  parent=$(printf '%s' "$parent" | tr -d '[:space:]')
  if [ -n "$parent" ]; then printf '/dev/%s' "$parent"; else readlink -f "$source"; fi
}
root_disk=$(backing_disk "$root_source") || target_failure
boot_disk=$(backing_disk "$boot_source") || target_failure
[ "$target" != "$root_source" ] || target_failure
[ "$target" != "$boot_source" ] || target_failure
[ "$target" != "$root_disk" ] || target_failure
[ "$target" != "$boot_disk" ] || target_failure

mount_rows=$(lsblk -nrpo MOUNTPOINTS -- "$target") || target_failure
if printf '%s\n' "$mount_rows" | awk 'NF { found=1 } END { exit(found ? 0 : 1) }'; then
  target_failure
fi

identity_record=$(LC_ALL=C lsblk -bdn -P \
  -o PATH,KNAME,TYPE,SIZE,MODEL,SERIAL,WWN,PTUUID,MAJ:MIN,RM -- "$target") || exit 44
[ -n "$identity_record" ] || exit 44
identity_sha256=$(printf '%s' "$identity_record" | sha256sum | awk '{print $1}')
target_size=$(sudo blockdev --getsize64 "$target") || exit 44
target_size=$(printf '%s' "$target_size" | tr -d '[:space:]')
case "$target_size" in ''|*[!0-9]*) exit 44 ;; esac
[ "$target_size" -gt 0 ] || exit 44
target_ro=$(sudo blockdev --getro "$target") || exit 44
target_ro=$(printf '%s' "$target_ro" | tr -d '[:space:]')
case "$target_ro" in 0|1) ;; *) exit 44 ;; esac
target_cid=$(tr '[:upper:]' '[:lower:]' < /sys/class/block/mmcblk0/device/cid) || exit 44
target_cid=$(printf '%s' "$target_cid" | tr -d '[:space:]')
printf '%s' "$target_cid" | grep -Eq '^[0-9a-f]{32}$' || exit 44
[ -b /dev/mmcblk0p1 ] || exit 44
[ -b /dev/mmcblk0p2 ] || exit 44
target_p1_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p1) || exit 44
target_p2_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p2) || exit 44
target_p1_partuuid=$(printf '%s' "$target_p1_partuuid" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
target_p2_partuuid=$(printf '%s' "$target_p2_partuuid" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
[ -n "$target_p1_partuuid" ] || exit 44
[ -n "$target_p2_partuuid" ] || exit 44
[ "$target_cid" = "$expected_cid" ] || target_failure
[ "$target_size" = "$expected_size" ] || target_failure
[ "$target_p1_partuuid" = "$expected_p1_partuuid" ] || target_failure
[ "$target_p2_partuuid" = "$expected_p2_partuuid" ] || target_failure
lsblk_tree=$(LC_ALL=C lsblk -brn -P \
  -o PATH,TYPE,SIZE,FSTYPE,UUID,PARTUUID,MOUNTPOINTS -- "$target") || exit 44

disk_field() {
  field_value=$(LC_ALL=C lsblk -bdnr -o "$1" -- "$target") || return 1
  printf '%s' "$field_value" | sed 's/[[:space:]]*$//'
}
emit_b64() {
  key=$1
  value=$2
  encoded=$(printf '%s' "$value" | base64 | tr -d '\n')
  printf '%s=%s\n' "$key" "$encoded"
}

target_path=$(disk_field PATH) || exit 44
target_kname=$(disk_field KNAME) || exit 44
target_model=$(disk_field MODEL) || exit 44
target_serial=$(disk_field SERIAL) || exit 44
target_wwn=$(disk_field WWN) || exit 44
target_ptuuid=$(disk_field PTUUID) || exit 44
target_majmin=$(disk_field MAJ:MIN) || exit 44
target_rm=$(disk_field RM) || exit 44

printf 'SCHEMA=arm-pi-offline-sd-preflight.v2\n'
emit_b64 HOSTNAME_B64 "$actual_hostname"
emit_b64 USER_B64 "$actual_user"
emit_b64 PI_MODEL_B64 "$pi_model"
emit_b64 ROOT_SOURCE_B64 "$root_source"
emit_b64 BOOT_SOURCE_B64 "$boot_source"
emit_b64 ROOT_DISK_B64 "$root_disk"
emit_b64 BOOT_DISK_B64 "$boot_disk"
emit_b64 TARGET_PATH_B64 "$target_path"
emit_b64 TARGET_KNAME_B64 "$target_kname"
emit_b64 TARGET_TYPE_B64 "$target_type"
printf 'TARGET_SIZE=%s\n' "$target_size"
emit_b64 TARGET_MODEL_B64 "$target_model"
emit_b64 TARGET_SERIAL_B64 "$target_serial"
emit_b64 TARGET_WWN_B64 "$target_wwn"
emit_b64 TARGET_PTUUID_B64 "$target_ptuuid"
emit_b64 TARGET_MAJMIN_B64 "$target_majmin"
printf 'TARGET_RO=%s\n' "$target_ro"
printf 'TARGET_RM=%s\n' "$(printf '%s' "$target_rm" | tr -d '[:space:]')"
printf 'TARGET_CID=%s\n' "$target_cid"
emit_b64 TARGET_P1_PARTUUID_B64 "$target_p1_partuuid"
emit_b64 TARGET_P2_PARTUUID_B64 "$target_p2_partuuid"
emit_b64 IDENTITY_RECORD_B64 "$identity_record"
printf 'IDENTITY_SHA256=%s\n' "$identity_sha256"
emit_b64 LSBLK_TREE_B64 "$lsblk_tree"
emit_b64 CAPTURED_AT_B64 "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
'@

$streamScript = @'
set -euo pipefail

expected_hostname=$1
expected_root=$2
expected_boot=$3
expected_user=$4
expected_identity_sha256=$5
expected_size=$6
expected_cid=$7
expected_p1_partuuid=$8
expected_p2_partuuid=$9
target=/dev/mmcblk0

refuse() {
  printf '%s\n' 'Offline SD stream gate failed.' >&2
  exit 45
}

actual_hostname=$(hostname) || refuse
actual_user=$(id -un) || refuse
pi_model=$(tr -d '\000' < /proc/device-tree/model 2>/dev/null) || refuse
[ "$actual_hostname" = "$expected_hostname" ] || refuse
[ "$actual_user" = "$expected_user" ] || refuse
case "$pi_model" in
  Raspberry\ Pi*) ;;
  *) refuse ;;
esac
root_source=$(findmnt -n -o SOURCE -- /) || refuse
boot_source=$(findmnt -n -o SOURCE -- /boot/firmware) || refuse
[ "$root_source" = "$expected_root" ] || refuse
[ "$boot_source" = "$expected_boot" ] || refuse
sudo -n true
[ -b "$target" ] || refuse
[ "$(readlink -f "$target")" = "$target" ] || refuse
target_type=$(lsblk -dn -o TYPE -- "$target") || refuse
target_type=$(printf '%s' "$target_type" | tr -d '[:space:]')
[ "$target_type" = disk ] || refuse

backing_disk() {
  source=$1
  parent=$(lsblk -dn -o PKNAME -- "$source") || return 1
  parent=$(printf '%s' "$parent" | tr -d '[:space:]')
  if [ -n "$parent" ]; then printf '/dev/%s' "$parent"; else readlink -f "$source"; fi
}
root_disk=$(backing_disk "$root_source") || refuse
boot_disk=$(backing_disk "$boot_source") || refuse
[ "$target" != "$root_disk" ] || refuse
[ "$target" != "$boot_disk" ] || refuse

require_unmounted() {
  mount_rows=$(lsblk -nrpo MOUNTPOINTS -- "$target") || refuse
  if printf '%s\n' "$mount_rows" | awk 'NF { found=1 } END { exit(found ? 0 : 1) }'; then
    refuse
  fi
}

assert_target_identity() {
  [ -b "$target" ] || refuse
  identity_record=$(LC_ALL=C lsblk -bdn -P \
    -o PATH,KNAME,TYPE,SIZE,MODEL,SERIAL,WWN,PTUUID,MAJ:MIN,RM -- "$target") || refuse
  [ -n "$identity_record" ] || refuse
  actual_identity_sha256=$(printf '%s' "$identity_record" | sha256sum | awk '{print $1}')
  actual_size=$(sudo blockdev --getsize64 "$target") || refuse
  actual_size=$(printf '%s' "$actual_size" | tr -d '[:space:]')
  actual_cid=$(tr '[:upper:]' '[:lower:]' < /sys/class/block/mmcblk0/device/cid) || refuse
  actual_cid=$(printf '%s' "$actual_cid" | tr -d '[:space:]')
  printf '%s' "$actual_cid" | grep -Eq '^[0-9a-f]{32}$' || refuse
  [ -b /dev/mmcblk0p1 ] || refuse
  [ -b /dev/mmcblk0p2 ] || refuse
  actual_p1_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p1) || refuse
  actual_p2_partuuid=$(sudo blkid -s PARTUUID -o value /dev/mmcblk0p2) || refuse
  actual_p1_partuuid=$(printf '%s' "$actual_p1_partuuid" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
  actual_p2_partuuid=$(printf '%s' "$actual_p2_partuuid" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
  [ "$actual_identity_sha256" = "$expected_identity_sha256" ] || refuse
  [ "$actual_size" = "$expected_size" ] || refuse
  [ "$actual_cid" = "$expected_cid" ] || refuse
  [ "$actual_p1_partuuid" = "$expected_p1_partuuid" ] || refuse
  [ "$actual_p2_partuuid" = "$expected_p2_partuuid" ] || refuse
}

for required_command in gzip dd blockdev sha256sum lsblk findmnt blkid; do
  command -v "$required_command" >/dev/null 2>&1 || {
    printf '%s\n' 'A required offline-image command is unavailable.' >&2
    exit 46
  }
done

original_ro=''
ro_changed=0
restore_kernel_ro() {
  status=$?
  trap - EXIT HUP INT TERM
  if [ "$ro_changed" -eq 1 ]; then
    if ! sudo blockdev --setrw "$target"; then
      printf '%s\n' 'Failed to restore the replacement SD kernel read-only flag.' >&2
      exit 47
    fi
    restored_ro=$(sudo blockdev --getro "$target") || exit 47
    restored_ro=$(printf '%s' "$restored_ro" | tr -d '[:space:]')
    if [ "$restored_ro" != 0 ]; then
      printf '%s\n' 'The replacement SD kernel read-only flag did not return to read-write.' >&2
      exit 47
    fi
  fi
  exit "$status"
}
trap restore_kernel_ro EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

# First gate in this stream process, before changing any kernel state.
assert_target_identity
require_unmounted
original_ro=$(sudo blockdev --getro "$target") || refuse
original_ro=$(printf '%s' "$original_ro" | tr -d '[:space:]')
case "$original_ro" in 0|1) ;; *) refuse ;; esac
if [ "$original_ro" -eq 0 ]; then
  ro_changed=1
  sudo blockdev --setro "$target" || refuse
fi

# Recheck the durable anchor, the unmounted proof, and RO=1 immediately before
# the read. blockdev's BLKROSET is a kernel flag; it does not write the media.
locked_ro=$(sudo blockdev --getro "$target") || refuse
locked_ro=$(printf '%s' "$locked_ro" | tr -d '[:space:]')
[ "$locked_ro" = 1 ] || refuse
assert_target_identity
require_unmounted

sudo dd if=/dev/mmcblk0 bs=4M iflag=fullblock status=none | gzip -1 -c

# Endpoint postflight is fail-closed. It proves the same CID, size, partition
# identities, lsblk identity and mount state after the read; the metadata does
# not claim continuous observation of other mount namespaces between checks.
locked_ro=$(sudo blockdev --getro "$target") || refuse
locked_ro=$(printf '%s' "$locked_ro" | tr -d '[:space:]')
[ "$locked_ro" = 1 ] || refuse
assert_target_identity
require_unmounted

exit 0
'@

$transportDirectory = $null
$transportBase = $null
$imageDirectory = $null
$partialImagePath = $null
$streamProcess = $null
$completed = $false

try {
    $transportBase = @($env:TEMP, $env:TMP) |
        Where-Object {
            $_ -and $_ -notmatch '\s' -and
            (Test-Path -LiteralPath $_ -PathType Container)
        } |
        Select-Object -First 1
    if (-not $transportBase) {
        throw 'A writable temporary path without spaces is required for pinned OpenSSH files.'
    }
    $transportDirectory = Join-Path $transportBase (
        'arm-pi-image-transport-' + [Guid]::NewGuid().ToString('N')
    )
    New-Item -ItemType Directory -Path $transportDirectory -ErrorAction Stop | Out-Null
    $sshKnownHostsPath = Join-Path $transportDirectory 'pinned-known-hosts'
    $emptyGlobalKnownHostsPath = Join-Path $transportDirectory 'empty-global-known-hosts'
    Copy-Item -LiteralPath $knownHostsPath -Destination $sshKnownHostsPath -ErrorAction Stop
    [System.IO.File]::WriteAllBytes($emptyGlobalKnownHostsPath, [byte[]] @())

    $sshArguments = @(
        '-p', $Port.ToString(),
        '-i', $identityPath,
        '-o', "UserKnownHostsFile=$sshKnownHostsPath",
        '-o', "GlobalKnownHostsFile=$emptyGlobalKnownHostsPath",
        '-o', 'StrictHostKeyChecking=yes',
        '-o', 'IdentitiesOnly=yes',
        '-o', 'BatchMode=yes',
        '-o', 'PasswordAuthentication=no',
        '-o', 'KbdInteractiveAuthentication=no',
        '-o', 'PubkeyAuthentication=yes',
        '-o', 'UpdateHostKeys=no',
        '-o', 'CheckHostIP=yes',
        '-o', 'ConnectTimeout=10',
        '-o', 'ServerAliveInterval=10',
        '-o', 'ServerAliveCountMax=3'
    )

    $remoteHost = if ($HostName.Contains(':')) { "[$HostName]" } else { $HostName }
    $remoteTarget = "${UserName}@${remoteHost}"
    $preflightCommand = (
        "sh -s -- '$ExpectedHostName' '$ExpectedRootDevice' '$ExpectedBootDevice' " +
        "'$UserName' " +
        "'$($identityAnchor.Cid)' '$($identityAnchor.SizeBytes)' " +
        "'$($identityAnchor.P1Partuuid)' '$($identityAnchor.P2Partuuid)'"
    )
    $preflightText = Invoke-SshScriptText `
        -SshPath $sshPath `
        -SshArguments $sshArguments `
        -RemoteTarget $remoteTarget `
        -RemoteCommand $preflightCommand `
        -ScriptText $preflightScript
    $preflight = ConvertFrom-PreflightRecord -Text $preflightText

    $sourceBytes = [long]::Parse(
        $preflight['TARGET_SIZE'],
        [System.Globalization.CultureInfo]::InvariantCulture
    )
    $preflightP1Partuuid = (
        ConvertFrom-Base64Utf8 $preflight['TARGET_P1_PARTUUID_B64']
    ).ToLowerInvariant()
    $preflightP2Partuuid = (
        ConvertFrom-Base64Utf8 $preflight['TARGET_P2_PARTUUID_B64']
    ).ToLowerInvariant()
    if ((ConvertFrom-Base64Utf8 $preflight['HOSTNAME_B64']) -ne $ExpectedHostName -or
        (ConvertFrom-Base64Utf8 $preflight['USER_B64']) -ne $UserName -or
        (ConvertFrom-Base64Utf8 $preflight['ROOT_SOURCE_B64']) -ne $ExpectedRootDevice -or
        (ConvertFrom-Base64Utf8 $preflight['BOOT_SOURCE_B64']) -ne $ExpectedBootDevice -or
        (ConvertFrom-Base64Utf8 $preflight['TARGET_PATH_B64']) -ne '/dev/mmcblk0' -or
        (ConvertFrom-Base64Utf8 $preflight['TARGET_TYPE_B64']) -ne 'disk' -or
        $sourceBytes -ne $identityAnchor.SizeBytes -or
        $preflight['TARGET_CID'] -cne $identityAnchor.Cid -or
        $preflightP1Partuuid -cne $identityAnchor.P1Partuuid -or
        $preflightP2Partuuid -cne $identityAnchor.P2Partuuid) {
        throw 'The Pi preflight record did not preserve the fixed USB-to-offline-SD contract.'
    }

    New-Item -ItemType Directory -Path $imageRoot -Force -ErrorAction Stop | Out-Null
    Set-PrivateDirectory -Path $imageRoot
    [long] $freeSpaceMarginBytes = 1GB
    $requiredFreeBytes = $sourceBytes + $freeSpaceMarginBytes
    $driveRoot = [System.IO.Path]::GetPathRoot([System.IO.Path]::GetFullPath($imageRoot))
    $driveInfo = New-Object System.IO.DriveInfo($driveRoot)
    [long] $availableFreeBytes = $driveInfo.AvailableFreeSpace
    if ($availableFreeBytes -lt $requiredFreeBytes) {
        throw 'The laptop does not have enough free space for a worst-case compressed SD image.'
    }

    $imageStamp = (Get-Date).ToUniversalTime().ToString('yyyyMMddTHHmmssZ')
    $imageName = "$imageStamp-$([Guid]::NewGuid().ToString('N').Substring(0, 8))"
    $imageDirectory = Join-Path $imageRoot $imageName
    New-Item -ItemType Directory -Path $imageDirectory -ErrorAction Stop | Out-Null
    Set-PrivateDirectory -Path $imageDirectory

    $finalImageName = 'arm-pi-sd-mmcblk0.img.gz'
    $finalImagePath = Join-Path $imageDirectory $finalImageName
    $partialImagePath = "$finalImagePath.partial"
    $streamCommand = (
        "bash -s -- '$ExpectedHostName' '$ExpectedRootDevice' '$ExpectedBootDevice' " +
        "'$UserName' " +
        "'$($preflight['IDENTITY_SHA256'])' '$sourceBytes' " +
        "'$($identityAnchor.Cid)' '$($identityAnchor.P1Partuuid)' " +
        "'$($identityAnchor.P2Partuuid)'"
    )
    $streamProcess = Start-SshScriptProcess `
        -SshPath $sshPath `
        -SshArguments $sshArguments `
        -RemoteTarget $remoteTarget `
        -RemoteCommand $streamCommand `
        -ScriptText $streamScript
    $errorTask = $streamProcess.StandardError.ReadToEndAsync()
    $partialImage = [System.IO.File]::Open(
        $partialImagePath,
        [System.IO.FileMode]::CreateNew,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::None
    )
    try {
        $streamProcess.StandardOutput.BaseStream.CopyTo($partialImage)
    }
    finally {
        $partialImage.Dispose()
    }
    $streamProcess.WaitForExit()
    $streamError = $errorTask.GetAwaiter().GetResult()
    if ($streamProcess.ExitCode -ne 0) {
        $message = if ($streamProcess.ExitCode -eq 45) {
            'The Pi changed boot media, SD identity, or SD mount state after preflight; no image was accepted.'
        }
        elseif ($streamProcess.ExitCode -eq 46) {
            'The old USB boot is missing a command required for the validated offline image.'
        }
        elseif ($streamProcess.ExitCode -eq 47) {
            'The image was discarded because the SD kernel read-only flag could not be restored.'
        }
        else {
            "Offline SD image stream failed with exit code $($streamProcess.ExitCode)."
        }
        if ($streamError -match 'Permission denied|Host key verification failed|Connection refused|timed out') {
            $message += ' Check the pinned SSH transport and direct Ethernet link.'
        }
        throw $message
    }

    $partialInfo = Get-Item -LiteralPath $partialImagePath -ErrorAction Stop
    if ($partialInfo.Length -le 0) {
        throw 'The Pi returned an empty SD image stream.'
    }
    $compressedHash = (
        Get-FileHash -LiteralPath $partialImagePath -Algorithm SHA256 -ErrorAction Stop
    ).Hash.ToLowerInvariant()
    $gzipEvidence = Read-GzipImageEvidence -Path $partialImagePath
    if ($gzipEvidence.DecompressedBytes -ne $sourceBytes) {
        throw (
            "The validated image expands to $($gzipEvidence.DecompressedBytes) bytes; " +
            "the offline SD reported $sourceBytes bytes."
        )
    }

    Move-Item -LiteralPath $partialImagePath -Destination $finalImagePath -ErrorAction Stop
    $partialImagePath = $null

    $metadata = [ordered] @{
        schema = 'arm-pi-offline-sd-image.local.v1'
        completedAtUtc = (Get-Date).ToUniversalTime().ToString('o')
        sshTarget = $remoteTarget
        sshPort = $Port
        pi = [ordered] @{
            hostname = ConvertFrom-Base64Utf8 $preflight['HOSTNAME_B64']
            user = ConvertFrom-Base64Utf8 $preflight['USER_B64']
            model = ConvertFrom-Base64Utf8 $preflight['PI_MODEL_B64']
            rootSource = ConvertFrom-Base64Utf8 $preflight['ROOT_SOURCE_B64']
            bootSource = ConvertFrom-Base64Utf8 $preflight['BOOT_SOURCE_B64']
            rootBackingDisk = ConvertFrom-Base64Utf8 $preflight['ROOT_DISK_B64']
            bootBackingDisk = ConvertFrom-Base64Utf8 $preflight['BOOT_DISK_B64']
        }
        source = [ordered] @{
            path = ConvertFrom-Base64Utf8 $preflight['TARGET_PATH_B64']
            kname = ConvertFrom-Base64Utf8 $preflight['TARGET_KNAME_B64']
            type = ConvertFrom-Base64Utf8 $preflight['TARGET_TYPE_B64']
            bytes = $sourceBytes
            model = (ConvertFrom-Base64Utf8 $preflight['TARGET_MODEL_B64']).Trim()
            serial = (ConvertFrom-Base64Utf8 $preflight['TARGET_SERIAL_B64']).Trim()
            wwn = (ConvertFrom-Base64Utf8 $preflight['TARGET_WWN_B64']).Trim()
            partitionTableUuid = (ConvertFrom-Base64Utf8 $preflight['TARGET_PTUUID_B64']).Trim()
            majorMinor = (ConvertFrom-Base64Utf8 $preflight['TARGET_MAJMIN_B64']).Trim()
            readOnlyFlag = $preflight['TARGET_RO']
            removableFlag = $preflight['TARGET_RM']
            cid = $preflight['TARGET_CID']
            partition1Partuuid = $preflightP1Partuuid
            partition2Partuuid = $preflightP2Partuuid
            lsblkIdentity = ConvertFrom-Base64Utf8 $preflight['IDENTITY_RECORD_B64']
            lsblkTree = ConvertFrom-Base64Utf8 $preflight['LSBLK_TREE_B64']
            identitySha256 = $preflight['IDENTITY_SHA256']
            identityDigestExcludesTransientReadOnlyFlag = $true
            preflightCapturedAtUtc = ConvertFrom-Base64Utf8 $preflight['CAPTURED_AT_B64']
        }
        image = [ordered] @{
            file = $finalImageName
            compressedBytes = (Get-Item -LiteralPath $finalImagePath).Length
            compressedSha256 = $compressedHash
            decompressedBytes = $gzipEvidence.DecompressedBytes
            rawImageSha256 = $gzipEvidence.RawImageSha256
            format = 'gzip-compressed raw whole-device image'
        }
        consistencyProof = [ordered] @{
            piBootedFromExpectedUsb = $true
            expectedUsbRoot = $ExpectedRootDevice
            expectedUsbBoot = $ExpectedBootDevice
            offlineSdDevice = '/dev/mmcblk0'
            targetHadNoMountpointsAtPreflight = $true
            targetWasNotRootOrBootBackingDisk = $true
            targetIdentityRecheckedImmediatelyBeforeStream = $true
            targetIdentityRecheckedAfterStream = $true
            cidSizeAndPartuuidsMatchedDurableAnchorAtEveryGate = $true
            kernelWholeDeviceReadOnlyVerifiedImmediatelyBeforeRead = $true
            kernelWholeDeviceReadOnlyVerifiedAfterRead = $true
            kernelReadOnlyFlagChangedForRead = ($preflight['TARGET_RO'] -eq '0')
            originalKernelReadOnlyStateRestoredBeforeSuccess = $true
            kernelFlagIoctlsOnlyNoMediaWrite = $true
            mountStateCheckedBeforeLockImmediatelyBeforeReadAndAfterRead = $true
            remotePipefailEnabled = $true
            sourceReadCommand = 'sudo dd if=/dev/mmcblk0 bs=4M iflag=fullblock status=none'
            remoteWritesToSource = $false
            gzipValidatedToEofLocally = $true
            decompressedByteCountMatchesSource = $true
            rawImageSha256ComputedLocally = $true
            destinationFreeSpaceCheckedBeforeStream = $true
            destinationRequiredFreeBytes = $requiredFreeBytes
            destinationAvailableFreeBytesAtCheck = $availableFreeBytes
            destinationFreeSpaceMarginBytes = $freeSpaceMarginBytes
            snapshotClass = 'offline unmounted whole-block-device image'
            remainingProofLimit = 'Kernel RO prevents media writes; mount state is checked at endpoints, not continuously observed in every mount namespace during the read.'
        }
        transport = [ordered] @{
            pinnedKnownHostsSha256 = (
                Get-FileHash -LiteralPath $knownHostsPath -Algorithm SHA256 -ErrorAction Stop
            ).Hash.ToLowerInvariant()
            dedicatedIdentity = $true
            noninteractive = $true
            secretContentsPrinted = $false
        }
        identityAnchor = [ordered] @{
            file = $identityAnchorPath
            sha256 = (
                Get-FileHash -LiteralPath $identityAnchorPath -Algorithm SHA256 -ErrorAction Stop
            ).Hash.ToLowerInvariant()
            schema = 'arm-pi-replacement-sd-identity.v1'
        }
    }
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    $metadataPath = Join-Path $imageDirectory 'metadata.json'
    $hashPath = Join-Path $imageDirectory 'image.sha256'
    [System.IO.File]::WriteAllText(
        $metadataPath,
        (($metadata | ConvertTo-Json -Depth 8) + "`n"),
        $utf8NoBom
    )
    [System.IO.File]::WriteAllText(
        $hashPath,
        "$compressedHash  $finalImageName`n",
        $utf8NoBom
    )

    $completed = $true
    Write-Host "Private offline SD image completed: $imageDirectory"
    Write-Host "Compressed SHA-256: $compressedHash"
    Write-Host "Raw image SHA-256: $($gzipEvidence.RawImageSha256)"
}
finally {
    if ($null -ne $streamProcess -and -not $streamProcess.HasExited) {
        $streamProcess.Kill()
        $streamProcess.WaitForExit()
    }
    if ($null -ne $streamProcess) {
        $streamProcess.Dispose()
    }
    if ($partialImagePath -and (Test-Path -LiteralPath $partialImagePath -PathType Leaf)) {
        Remove-Item -LiteralPath $partialImagePath -Force
    }
    if (-not $completed -and $imageDirectory) {
        Remove-IncompleteImageDirectory -Path $imageDirectory -ImageRoot $imageRoot
    }
    if ($transportDirectory -and (Test-Path -LiteralPath $transportDirectory -PathType Container)) {
        $transportFull = [System.IO.Path]::GetFullPath($transportDirectory)
        $temporaryRoot = [System.IO.Path]::GetFullPath($transportBase).TrimEnd('\', '/')
        $temporaryPrefix = $temporaryRoot + [System.IO.Path]::DirectorySeparatorChar
        if (-not $transportFull.StartsWith(
            $temporaryPrefix,
            [System.StringComparison]::OrdinalIgnoreCase
        )) {
            throw "Refusing to clean an unexpected transport directory: $transportFull"
        }
        Get-ChildItem -LiteralPath $transportFull -Force -File | Remove-Item -Force
        Remove-Item -LiteralPath $transportFull -Force
    }
}
