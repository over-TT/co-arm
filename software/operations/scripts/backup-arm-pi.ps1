#Requires -Version 5.1

<#
.SYNOPSIS
Creates a private recovery backup of the restored Raspberry Pi arm gateway.

.DESCRIPTION
Authenticates with the dedicated SSH key and pinned project known-hosts file.
The completed implementation creates only a bounded temporary archive on the
Pi and stores the encrypted-transport result beneath the ignored project
runtime directory without printing credentials or file contents.
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

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $ExpectedHostName,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $ExpectedRootDevice,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $ExpectedBootDevice,

    [Parameter(Mandatory)]
    [ValidatePattern('^(?:[0-9]{1,3}\.){3}[0-9]{1,3}/(?:[0-9]|[12][0-9]|3[0-2])$')]
    [string] $DirectLanAddressCidr,

    [Parameter(Mandatory)]
    [ValidatePattern('^/etc/netplan/[A-Za-z0-9._-]+\.ya?ml$')]
    [string] $DirectLanProfilePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($HostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$' -or $HostName.StartsWith('-')) {
    throw 'HostName must be a DNS name, IPv4 address, or unbracketed IPv6 literal.'
}
if ($ExpectedHostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$' -or $ExpectedHostName.StartsWith('-')) {
    throw 'ExpectedHostName must be a plain hostname.'
}
foreach ($device in @($ExpectedRootDevice, $ExpectedBootDevice)) {
    if ($device -notmatch '^/dev/[A-Za-z0-9._/+:-]+$') {
        throw 'Expected root and boot devices must be absolute /dev paths.'
    }
}
$installRoot = "/home/$UserName/arm-gateway"

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
    [System.IO.Directory]::SetAccessControl($Path, $security)
}

function Remove-IncompleteBackupDirectory {
    param(
        [Parameter(Mandatory)] [string] $Path,
        [Parameter(Mandatory)] [string] $BackupRoot
    )

    $rootFull = [System.IO.Path]::GetFullPath($BackupRoot).TrimEnd('\', '/')
    $candidateFull = [System.IO.Path]::GetFullPath($Path)
    $requiredPrefix = $rootFull + [System.IO.Path]::DirectorySeparatorChar
    if (-not $candidateFull.StartsWith(
        $requiredPrefix,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Refusing to clean a backup path outside the private backup root: $candidateFull"
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

$softwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$requiredGatewayModuleNames = @(
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
$localGatewayDirectory = Join-Path $softwareRoot 'python\robot_gateway'
$currentGatewayModuleNames = @(
    Get-ChildItem -LiteralPath $localGatewayDirectory -File -Filter '*.py' -ErrorAction Stop |
        ForEach-Object { $_.Name } |
        Sort-Object
)
if ($currentGatewayModuleNames.Count -ne 13 -or
    (($currentGatewayModuleNames -join "`n") -cne (($requiredGatewayModuleNames | Sort-Object) -join "`n"))) {
    throw 'The laptop gateway source is incomplete or unexpected; exactly the reviewed 13 Python modules are required.'
}
$expectedGatewayModulesCsv = ($currentGatewayModuleNames -join ',')
$requirementsPath = Join-Path $softwareRoot 'operations\requirements-pi.txt'
$installRelative = $installRoot.TrimStart('/')
$expectedCanonicalSourceRows = @(
    foreach ($moduleName in $currentGatewayModuleNames) {
        $modulePath = Join-Path $localGatewayDirectory $moduleName
        $moduleSha256 = (
            Get-FileHash -LiteralPath $modulePath -Algorithm SHA256 -ErrorAction Stop
        ).Hash.ToLowerInvariant()
        "$moduleSha256  $installRelative/robot_gateway/$moduleName"
    }
    $requirementsSha256 = (
        Get-FileHash -LiteralPath $requirementsPath -Algorithm SHA256 -ErrorAction Stop
    ).Hash.ToLowerInvariant()
    "$requirementsSha256  $installRelative/requirements-pi.txt"
)
if ($expectedCanonicalSourceRows.Count -ne 14) {
    throw 'The canonical Pi source hash contract must contain exactly 13 modules and requirements-pi.txt.'
}
$expectedCanonicalSourceContract = ([string[]] $expectedCanonicalSourceRows -join "`n") + "`n"
$expectedCanonicalSourceContractBase64 = [Convert]::ToBase64String(
    [System.Text.Encoding]::UTF8.GetBytes($expectedCanonicalSourceContract)
)

$sshPath = (Get-Command ssh -CommandType Application -ErrorAction Stop).Source
$tarPath = (Get-Command tar -CommandType Application -ErrorAction Stop).Source
$identityPath = Resolve-RequiredLeaf -Path $IdentityFile -Label 'IdentityFile'
$knownHostsPath = Resolve-RequiredLeaf -Path $KnownHostsFile -Label 'KnownHostsFile'
$backupRoot = Join-Path $softwareRoot 'runtime\robot-gateway\pi-backups'

$remoteArchiveScript = @'
set -eu

expected_hostname=$1
expected_root=$2
expected_boot=$3
expected_modules_csv=$4
expected_user=$5
service_group=$6
install_root=$7
direct_lan_address_cidr=$8
direct_lan_profile=$9
expected_canonical_source_contract_base64=${10}
install_relative=${install_root#/}
home_root=${install_root%/arm-gateway}
home_relative=${home_root#/}
direct_lan_profile_relative=${direct_lan_profile#/}
case "$install_root" in
  /home/*/arm-gateway) ;;
  *) printf '%s\n' 'Unexpected gateway install root.' >&2; exit 42 ;;
esac
case "$direct_lan_profile" in
  /etc/netplan/*.yaml|/etc/netplan/*.yml) ;;
  *) printf '%s\n' 'Unexpected direct-LAN profile path.' >&2; exit 42 ;;
esac

actual_hostname=$(hostname)
actual_user=$(id -un)
root_source=$(findmnt -n -o SOURCE -- /)
root_fstype=$(findmnt -n -o FSTYPE -- /)
boot_source=$(findmnt -n -o SOURCE -- /boot/firmware)
model=$(tr -d '\000' < /proc/device-tree/model 2>/dev/null || true)

identity_failure() {
  printf '%s\n' 'The remote Raspberry Pi identity did not match the requested backup target.' >&2
  exit 42
}

if [ "$actual_hostname" != "$expected_hostname" ]; then identity_failure; fi
if [ "$actual_user" != "$expected_user" ]; then identity_failure; fi
if [ "$root_source" != "$expected_root" ]; then identity_failure; fi
if [ "$root_fstype" != "ext4" ]; then identity_failure; fi
if [ "$boot_source" != "$expected_boot" ]; then identity_failure; fi
case "$model" in
  Raspberry\ Pi*) ;;
  *) identity_failure ;;
esac

sudo -n true
remote_tmp=''
cleanup() {
  case "$remote_tmp" in
    /tmp/arm-pi-backup.*) sudo rm -rf -- "$remote_tmp" ;;
    '') ;;
    *) printf '%s\n' 'Refusing unexpected remote cleanup path.' >&2 ;;
  esac
}
trap cleanup EXIT HUP INT TERM
remote_tmp=$(sudo mktemp -d /tmp/arm-pi-backup.XXXXXXXXXX)
sudo chown "$expected_user:$service_group" "$remote_tmp"
sudo chmod 0700 "$remote_tmp"

file_list="$remote_tmp/files.nul"
expected_modules="$remote_tmp/expected-modules.txt"
actual_modules="$remote_tmp/actual-modules.txt"
actual_modules_raw="$remote_tmp/actual-modules.raw.txt"
snapshot_modules="$remote_tmp/snapshot-modules.txt"
snapshot_modules_raw="$remote_tmp/snapshot-modules.raw.txt"
verify_modules="$remote_tmp/verify-modules.txt"
verify_modules_raw="$remote_tmp/verify-modules.raw.txt"
snapshot_checksum_list="$remote_tmp/snapshot-checksum-files.nul"
metadata_checksum_list="$remote_tmp/metadata-checksum-files.nul"
expected_canonical_source="$remote_tmp/expected-canonical-source.sha256"
snapshot_canonical_source="$remote_tmp/snapshot-canonical-source.sha256"
snapshot_root="$remote_tmp/snapshot"
verify_root="$remote_tmp/verify"
seed_tar="$remote_tmp/snapshot-seed.tar"
archive_tar="$remote_tmp/arm-pi-recovery.tar"
archive_gz="$archive_tar.gz"
sudo install -d -m 0700 -o root -g root "$snapshot_root" "$verify_root"
sudo install -m 0600 -o "$expected_user" -g "$service_group" /dev/null "$file_list"

incomplete() {
  printf '%s\n' 'The Pi gateway installation is incomplete; refusing a misleading recovery backup.' >&2
  exit 43
}

source_drift() {
  printf '%s\n' 'The deployed Pi gateway source differs from the canonical laptop source; refusing the backup.' >&2
  exit 44
}

add_file() {
  relative=$1
  if sudo test -f "/$relative"; then
    printf '%s\000' "$relative" >> "$file_list"
  fi
}

require_file() {
  relative=$1
  sudo test -f "/$relative" || incomplete
  printf '%s\000' "$relative" >> "$file_list"
}

require_regular_nonsymlink() {
  relative=$1
  sudo test -f "/$relative" || incomplete
  sudo test ! -L "/$relative" || incomplete
  printf '%s\000' "$relative" >> "$file_list"
}

add_tree() {
  relative=$1
  if sudo test -d "/$relative"; then
    sudo find -P "/$relative" -xdev -type f -printf "$relative/%P\000" >> "$file_list"
  fi
}

# The expected source list is derived from the laptop's current reviewed tree.
# Require the remote directory to contain exactly those 13 modules: a partial
# deployment or an unexpected extra Python module is not a recovery backup.
printf '%s\n' "$expected_modules_csv" | tr ',' '\n' | LC_ALL=C sort > "$expected_modules"
[ "$(wc -l < "$expected_modules" | tr -d ' ')" -eq 13 ] || incomplete
sudo find -P "$install_root/robot_gateway" -xdev -maxdepth 1 -type f \
  -name '*.py' -printf '%f\n' > "$actual_modules_raw" 2>/dev/null || incomplete
LC_ALL=C sort "$actual_modules_raw" > "$actual_modules" || incomplete
[ "$(wc -l < "$actual_modules" | tr -d ' ')" -eq 13 ] || incomplete
cmp -s "$expected_modules" "$actual_modules" || incomplete
while IFS= read -r module; do
  case "$module" in
    ''|*[!A-Za-z0-9_.]*) incomplete ;;
  esac
  require_file "$install_relative/robot_gateway/$module"
done < "$expected_modules"

# These files are the minimum complete recovery contract.
require_file "$install_relative/requirements-pi.txt"
require_file "$install_relative/.robot-gateway.token"
require_file var/lib/arm-gateway/arm-joints.json
require_file var/lib/arm-gateway/arm-joints.recovery-provenance.json
require_file var/lib/arm-gateway/recovery-manifest.json
require_file etc/systemd/system/arm-gateway.service
require_file boot/firmware/config.txt
require_file boot/firmware/cmdline.txt
require_regular_nonsymlink "$direct_lan_profile_relative"
sudo grep -Fq -- 'eth0' "$direct_lan_profile" || incomplete
sudo grep -Fq -- "$direct_lan_address_cidr" "$direct_lan_profile" || incomplete

# Persistent calibration, accepted physical profile, and fail-closed STOP state.
add_file var/lib/arm-gateway/physical-arm-profile.json
add_file var/lib/arm-gateway/arm-clear-required.json
add_file var/lib/arm-gateway/base-reference-required.json
add_file var/lib/arm-gateway/headless-network-config-removed.json
add_tree var/lib/arm-gateway/recovery-source
add_tree var/lib/arm-gateway/recovery-boot-config

# Units and reviewed runtime activation/boot-order drop-ins.
add_tree etc/systemd/system/arm-gateway.service.d
add_tree etc/systemd/system/wayvnc.service.d
add_tree etc/systemd/system/ssh.service.d

# Direct-Ethernet, optional Wi-Fi, SSH identity, and local VNC configuration.
add_tree etc/NetworkManager/system-connections
add_tree etc/netplan
add_tree etc/ssh
add_file etc/fstab
add_file etc/fake-hwclock.data
add_file etc/arm-gateway/recovery-usb-media-binding.v1
add_file "$home_relative/.ssh/authorized_keys"
add_tree "$home_relative/.config/wayvnc"

# Boot/UART configuration and the minimum OS/account/package recovery context.
add_file boot/firmware/usercfg.txt
add_file boot/firmware/user-data
add_file boot/firmware/network-config
add_file boot/firmware/meta-data
add_file etc/hostname
add_file etc/hosts
add_file etc/machine-id
add_file etc/os-release
add_file etc/passwd
add_file etc/group
add_file etc/sudoers
add_tree etc/sudoers.d
add_tree etc/udev/rules.d
add_file var/lib/dpkg/status
add_file etc/apt/sources.list
add_tree etc/apt/sources.list.d

# De-duplicate the curated list, then materialize a dereferenced regular-file
# snapshot before hashing anything. Checksums below are therefore over the
# archived snapshot bytes, never over mutable live files.
LC_ALL=C sort -zu "$file_list" -o "$file_list"
sudo tar --create --file "$seed_tar" --numeric-owner --acls --xattrs \
  --dereference --no-recursion --directory / --null --files-from "$file_list"
sudo tar --extract --file "$seed_tar" --numeric-owner --acls --xattrs \
  --directory "$snapshot_root"
sudo rm -f -- "$seed_tar"
non_regular=$(sudo find "$snapshot_root" ! -type d ! -type f -print -quit) || incomplete
[ -z "$non_regular" ] || incomplete

while IFS= read -r module; do
  sudo test -f "$snapshot_root/$install_relative/robot_gateway/$module" || incomplete
done < "$expected_modules"
sudo find "$snapshot_root/$install_relative/robot_gateway" -maxdepth 1 \
  -type f -name '*.py' -printf '%f\n' > "$snapshot_modules_raw" || incomplete
LC_ALL=C sort "$snapshot_modules_raw" > "$snapshot_modules" || incomplete
cmp -s "$expected_modules" "$snapshot_modules" || incomplete
snapshot_source_count=13
case "$expected_canonical_source_contract_base64" in
  ''|*[!A-Za-z0-9+/=]*) source_drift ;;
esac
printf '%s' "$expected_canonical_source_contract_base64" \
  | base64 --decode > "$expected_canonical_source" || source_drift
[ "$(wc -l < "$expected_canonical_source" | tr -d ' ')" -eq 14 ] || source_drift
while IFS= read -r module; do
  digest=$(sudo sha256sum -- "$snapshot_root/$install_relative/robot_gateway/$module" \
    | awk '{print $1}') || source_drift
  printf '%s  %s/robot_gateway/%s\n' "$digest" "$install_relative" "$module"
done < "$expected_modules" > "$snapshot_canonical_source"
requirements_digest=$(sudo sha256sum -- "$snapshot_root/$install_relative/requirements-pi.txt" \
  | awk '{print $1}') || source_drift
printf '%s  %s/requirements-pi.txt\n' "$requirements_digest" "$install_relative" \
  >> "$snapshot_canonical_source"
cmp -s "$expected_canonical_source" "$snapshot_canonical_source" || source_drift
canonical_source_contract_sha256=$(sha256sum "$expected_canonical_source" | awk '{print $1}')
for required in \
  "$install_relative/.robot-gateway.token" \
  var/lib/arm-gateway/arm-joints.json \
  var/lib/arm-gateway/arm-joints.recovery-provenance.json \
  var/lib/arm-gateway/recovery-manifest.json \
  etc/systemd/system/arm-gateway.service \
  boot/firmware/config.txt \
  boot/firmware/cmdline.txt \
  "$direct_lan_profile_relative"
do
  sudo test -f "$snapshot_root/$required" || incomplete
done
snapshot_lan="$snapshot_root/$direct_lan_profile_relative"
sudo grep -Fq -- 'eth0' "$snapshot_lan" || incomplete
sudo grep -Fq -- "$direct_lan_address_cidr" "$snapshot_lan" || incomplete

metadata_dir="$snapshot_root/metadata"
sudo install -d -m 0700 -o root -g root "$metadata_dir"
sudo install -m 0600 -o root -g root "$expected_modules" "$metadata_dir/gateway-modules.txt"
sudo install -m 0600 -o root -g root "$expected_canonical_source" \
  "$metadata_dir/canonical-source.sha256"
sudo find "$snapshot_root" -type f ! -path "$metadata_dir/*" -printf '%P\000' \
  > "$snapshot_checksum_list" || incomplete
LC_ALL=C sort -zu "$snapshot_checksum_list" -o "$snapshot_checksum_list" || incomplete
sudo sh -c 'cd "$1" && xargs -0 -r sha256sum -- < "$2" > metadata/SHA256SUMS' \
  sh "$snapshot_root" "$snapshot_checksum_list"
sudo chmod 0600 "$metadata_dir/SHA256SUMS"

root_partuuid=$(lsblk -n -o PARTUUID -- "$root_source" | head -n 1 | tr -d ' ')
boot_partuuid=$(lsblk -n -o PARTUUID -- "$boot_source" | head -n 1 | tr -d ' ')
serial0_target=$(readlink -f /dev/serial0 2>/dev/null || true)
machine_id_sha256=$(sudo sha256sum "$snapshot_root/etc/machine-id" | awk '{print $1}')
file_count=$(tr -cd '\000' < "$snapshot_checksum_list" | wc -c | tr -d ' ')
captured_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)
physical_profile_present=no
stop_latch_present=no
base_reference_gate_present=no
physical_uart_dropin_present=no
headless_cleanup_marker_present=no
recovery_source_present=no
recovery_boot_config_present=no
usb_media_binding_present=no
fstab_present=no
fake_hwclock_present=no
sudo test -f "$snapshot_root/var/lib/arm-gateway/physical-arm-profile.json" && physical_profile_present=yes
sudo test -f "$snapshot_root/var/lib/arm-gateway/arm-clear-required.json" && stop_latch_present=yes
sudo test -f "$snapshot_root/var/lib/arm-gateway/base-reference-required.json" && base_reference_gate_present=yes
sudo test -f "$snapshot_root/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf" && physical_uart_dropin_present=yes
sudo test -f "$snapshot_root/var/lib/arm-gateway/headless-network-config-removed.json" && headless_cleanup_marker_present=yes
if sudo find "$snapshot_root/var/lib/arm-gateway/recovery-source" -type f -print -quit \
   2>/dev/null | grep -q .; then
  recovery_source_present=yes
fi
if sudo find "$snapshot_root/var/lib/arm-gateway/recovery-boot-config" -type f -print -quit \
   2>/dev/null | grep -q .; then
  recovery_boot_config_present=yes
fi
sudo test -f "$snapshot_root/etc/arm-gateway/recovery-usb-media-binding.v1" && usb_media_binding_present=yes
sudo test -f "$snapshot_root/etc/fstab" && fstab_present=yes
sudo test -f "$snapshot_root/etc/fake-hwclock.data" && fake_hwclock_present=yes

{
  printf 'schema\tarm-pi-recovery-backup.v3\n'
  printf 'capturedAtUtc\t%s\n' "$captured_at"
  printf 'hostname\t%s\n' "$actual_hostname"
  printf 'user\t%s\n' "$actual_user"
  printf 'model\t%s\n' "$model"
  printf 'rootSource\t%s\n' "$root_source"
  printf 'rootFstype\t%s\n' "$root_fstype"
  printf 'rootPartuuid\t%s\n' "$root_partuuid"
  printf 'bootSource\t%s\n' "$boot_source"
  printf 'bootPartuuid\t%s\n' "$boot_partuuid"
  printf 'serial0Target\t%s\n' "$serial0_target"
  printf 'machineIdSha256\t%s\n' "$machine_id_sha256"
  printf 'sourceModuleCount\t%s\n' "$snapshot_source_count"
  printf 'archivedFileCount\t%s\n' "$file_count"
  printf 'canonicalSourceFileCount\t14\n'
  printf 'canonicalSourceHashesVerified\tyes\n'
  printf 'canonicalSourceContractSha256\t%s\n' "$canonical_source_contract_sha256"
  printf 'physicalArmProfilePresent\t%s\n' "$physical_profile_present"
  printf 'stopLatchPresent\t%s\n' "$stop_latch_present"
  printf 'baseReferenceGatePresent\t%s\n' "$base_reference_gate_present"
  printf 'physicalUartDropInPresent\t%s\n' "$physical_uart_dropin_present"
  printf 'headlessCleanupMarkerPresent\t%s\n' "$headless_cleanup_marker_present"
  printf 'recoverySourcePresent\t%s\n' "$recovery_source_present"
  printf 'recoveryBootConfigPresent\t%s\n' "$recovery_boot_config_present"
  printf 'usbMediaBindingPresent\t%s\n' "$usb_media_binding_present"
  printf 'fstabPresent\t%s\n' "$fstab_present"
  printf 'fakeHwclockPresent\t%s\n' "$fake_hwclock_present"
  printf 'requiredCalibrationProvenancePresent\tyes\n'
  printf 'requiredRecoveryManifestPresent\tyes\n'
  printf 'requiredDirectLanNetplanPresent\tyes\n'
  printf 'directLanProfilePath\t%s\n' "$direct_lan_profile"
  printf 'directLanExpectedInterface\teth0\n'
  printf 'directLanExpectedAddress\t%s\n' "$direct_lan_address_cidr"
  printf 'snapshotFilesDereferenced\tyes\n'
  printf 'checksumsCoverSnapshotBytes\tyes\n'
  printf 'kernelRelease\t%s\n' "$(uname -r)"
} | sudo tee "$metadata_dir/snapshot.tsv" >/dev/null
sudo chmod 0600 "$metadata_dir/snapshot.tsv"

dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null \
  | LC_ALL=C sort | sudo tee "$metadata_dir/packages.tsv" >/dev/null
sudo chmod 0600 "$metadata_dir/packages.tsv"

for service in arm-gateway.service ssh.service wayvnc.service; do
  printf '[%s]\n' "$service"
  systemctl show "$service" --no-pager \
    --property=Id,LoadState,ActiveState,SubState,UnitFileState,FragmentPath,DropInPaths,NRestarts \
    2>/dev/null || true
done | sudo tee "$metadata_dir/services.txt" >/dev/null
sudo chmod 0600 "$metadata_dir/services.txt"

sudo find "$metadata_dir" -type f ! -name METADATA_SHA256SUMS -printf 'metadata/%f\000' \
  > "$metadata_checksum_list" || incomplete
LC_ALL=C sort -zu "$metadata_checksum_list" -o "$metadata_checksum_list" || incomplete
sudo sh -c 'cd "$1" && xargs -0 -r sha256sum -- < "$2" > metadata/METADATA_SHA256SUMS' \
  sh "$snapshot_root" "$metadata_checksum_list"
sudo chmod 0600 "$metadata_dir/METADATA_SHA256SUMS"

sudo tar --create --file "$archive_tar" --numeric-owner --acls --xattrs \
  --directory "$snapshot_root" .
sudo gzip -n "$archive_tar"
sudo test -s "$archive_gz"
sudo tar --list --gzip --file "$archive_gz" >/dev/null

# Self-extract the final bytes and verify both the frozen payload and its
# metadata before any byte is streamed to the laptop.
sudo tar --extract --gzip --file "$archive_gz" --directory "$verify_root"
sudo sh -c 'cd "$1" && sha256sum -c metadata/SHA256SUMS >/dev/null && \
  sha256sum -c metadata/METADATA_SHA256SUMS >/dev/null && \
  sha256sum -c metadata/canonical-source.sha256 >/dev/null' sh "$verify_root" || incomplete
sudo find "$verify_root/$install_relative/robot_gateway" -maxdepth 1 \
  -type f -name '*.py' -printf '%f\n' > "$verify_modules_raw" || incomplete
LC_ALL=C sort "$verify_modules_raw" > "$verify_modules" || incomplete
cmp -s "$expected_modules" "$verify_modules" || incomplete
sudo chmod 0600 "$archive_gz"
sudo cat "$archive_gz"
'@

$remoteHost = if ($HostName.Contains(':')) { "[$HostName]" } else { $HostName }
$remoteTarget = "${UserName}@${remoteHost}"
$remoteCommand = (
    "sh -s -- '$ExpectedHostName' '$ExpectedRootDevice' '$ExpectedBootDevice' " +
    "'$expectedGatewayModulesCsv' '$UserName' '$ServiceGroup' '$installRoot' " +
    "'$DirectLanAddressCidr' '$DirectLanProfilePath' '$expectedCanonicalSourceContractBase64'"
)
$transportDirectory = $null
$backupDirectory = $null
$partialArchivePath = $null
$process = $null
$processStarted = $false
$completed = $false
$transportBase = $null

try {
    $transportBase = @($env:TEMP, $env:TMP) |
        Where-Object { $_ -and $_ -notmatch '\s' -and (Test-Path -LiteralPath $_ -PathType Container) } |
        Select-Object -First 1
    if (-not $transportBase) {
        throw 'A writable temporary path without spaces is required for pinned OpenSSH files.'
    }
    $transportDirectory = Join-Path $transportBase (
        'arm-pi-backup-transport-' + [Guid]::NewGuid().ToString('N')
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

    New-Item -ItemType Directory -Path $backupRoot -Force -ErrorAction Stop | Out-Null
    Set-PrivateDirectory -Path $backupRoot
    $backupStamp = (Get-Date).ToUniversalTime().ToString('yyyyMMddTHHmmssZ')
    $backupName = "$backupStamp-$([Guid]::NewGuid().ToString('N').Substring(0, 8))"
    $backupDirectory = Join-Path $backupRoot $backupName
    New-Item -ItemType Directory -Path $backupDirectory -ErrorAction Stop | Out-Null
    Set-PrivateDirectory -Path $backupDirectory

    $archiveName = 'arm-pi-recovery.tar.gz'
    $archivePath = Join-Path $backupDirectory $archiveName
    $partialArchivePath = "$archivePath.partial"
    $processArguments = @($sshArguments) + @($remoteTarget, $remoteCommand)

    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $sshPath
    $startInfo.Arguments = (
        $processArguments | ForEach-Object { ConvertTo-NativeArgument -Argument $_ }
    ) -join ' '
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardInput = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo
    if (-not $process.Start()) {
        throw 'Could not start the pinned SSH backup process.'
    }
    $processStarted = $true
    $standardErrorTask = $process.StandardError.ReadToEndAsync()
    $remotePayload = ($remoteArchiveScript -replace "`r", '') + "`n#`n"
    $process.StandardInput.Write($remotePayload)
    $process.StandardInput.Close()

    $partialArchive = [System.IO.File]::Open(
        $partialArchivePath,
        [System.IO.FileMode]::CreateNew,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::None
    )
    try {
        $process.StandardOutput.BaseStream.CopyTo($partialArchive)
    }
    finally {
        $partialArchive.Dispose()
    }
    $process.WaitForExit()
    $remoteError = $standardErrorTask.GetAwaiter().GetResult()
    if ($process.ExitCode -ne 0) {
        $failure = if ($process.ExitCode -eq 42) {
            'The pinned SSH host answered, but its hostname, Pi model, user, or boot media did not match.'
        }
        elseif ($process.ExitCode -eq 43) {
            'The restored Pi gateway installation is incomplete.'
        }
        elseif ($process.ExitCode -eq 44) {
            'The deployed Pi gateway source or requirements differ from the canonical laptop source.'
        }
        else {
            "Remote backup failed with exit code $($process.ExitCode)."
        }
        if ($remoteError -match 'Permission denied|Host key verification failed|Connection refused|timed out') {
            $failure += ' Check the pinned host key, dedicated identity, SSH service, and direct Ethernet link.'
        }
        throw $failure
    }

    $partialInfo = Get-Item -LiteralPath $partialArchivePath -ErrorAction Stop
    if ($partialInfo.Length -le 0) {
        throw 'The Pi returned an empty recovery archive.'
    }
    & $tarPath --list --gzip --file $partialArchivePath *> $null
    if ($LASTEXITCODE -ne 0) {
        throw 'The downloaded recovery archive failed local gzip/tar validation.'
    }

    $archiveHash = (
        Get-FileHash -LiteralPath $partialArchivePath -Algorithm SHA256 -ErrorAction Stop
    ).Hash.ToLowerInvariant()
    Move-Item -LiteralPath $partialArchivePath -Destination $archivePath -ErrorAction Stop
    $partialArchivePath = $null

    $metadata = [ordered] @{
        schema = 'arm-pi-recovery-backup.local.v3'
        capturedAtUtc = (Get-Date).ToUniversalTime().ToString('o')
        sshTarget = $remoteTarget
        sshPort = $Port
        expectedHostName = $ExpectedHostName
        expectedRootDevice = $ExpectedRootDevice
        expectedBootDevice = $ExpectedBootDevice
        serviceUser = $UserName
        serviceGroup = $ServiceGroup
        installRoot = $installRoot
        directLanAddressCidr = $DirectLanAddressCidr
        directLanProfilePath = $DirectLanProfilePath
        archiveFile = $archiveName
        archiveBytes = (Get-Item -LiteralPath $archivePath).Length
        archiveSha256 = $archiveHash
        knownHostsSha256 = (
            Get-FileHash -LiteralPath $knownHostsPath -Algorithm SHA256 -ErrorAction Stop
        ).Hash.ToLowerInvariant()
        secretContentsPrinted = $false
        remoteWrites = 'randomized /tmp archive only; trap-cleaned'
    }
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    $metadataPath = Join-Path $backupDirectory 'metadata.json'
    $hashPath = Join-Path $backupDirectory 'archive.sha256'
    [System.IO.File]::WriteAllText(
        $metadataPath,
        (($metadata | ConvertTo-Json -Depth 4) + "`n"),
        $utf8NoBom
    )
    [System.IO.File]::WriteAllText(
        $hashPath,
        "$archiveHash  $archiveName`n",
        $utf8NoBom
    )

    $completed = $true
    Write-Host "Private Pi recovery backup completed: $backupDirectory"
    Write-Host "Archive SHA-256: $archiveHash"
}
finally {
    if ($processStarted -and -not $process.HasExited) {
        $process.Kill()
        $process.WaitForExit()
    }
    if ($partialArchivePath -and (Test-Path -LiteralPath $partialArchivePath -PathType Leaf)) {
        Remove-Item -LiteralPath $partialArchivePath -Force
    }
    if (-not $completed -and $backupDirectory) {
        Remove-IncompleteBackupDirectory -Path $backupDirectory -BackupRoot $backupRoot
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
        Get-ChildItem -LiteralPath $transportFull -Force -File |
            Remove-Item -Force
        Remove-Item -LiteralPath $transportFull -Force
    }
}
