#Requires -Version 5.1

<#
.SYNOPSIS
Prepares and restores the Raspberry Pi arm gateway after an SD-card loss.

.DESCRIPTION
The default Prepare phase is laptop-only. It validates and hashes the current
gateway source, private bearer token, recovered four-joint calibration, and
reviewed service files into an ignored recovery manifest.

Bootstrap may be run only after the replacement SD is proven to be the Pi root.
It privately seeds the existing token, invokes the dormant gateway deployment,
restores calibration, and installs the WayVNC ordering drop-in. It never enables
the physical UART, clears STOP, flashes firmware, takes torque, or moves a joint.

ConfigureUart backs up the SD boot configuration and enables the Pi UART without
rebooting. Activate is deliberately separate, seeds or retains a fail-closed
inspection latch, installs the reviewed UART drop-in, restarts once, and
authenticates a strict torque-off controller-state proof. Deactivate verifies
and removes only that reviewed drop-in, retains the latch, and deliberately
restarts in dormant mode. Verify does not mutate restored system state. No
phase reboots the Pi or performs an arm action.
#>

[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'High')]
param(
    [ValidateSet('Prepare', 'Bootstrap', 'ConfigureUart', 'Activate', 'Deactivate', 'Verify')]
    [string] $Phase = 'Prepare',

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $HostName,

    [Parameter(Mandatory)]
    [ValidatePattern('^[a-z_][a-z0-9_-]{0,31}$')]
    [string] $UserName,

    [Parameter(Mandatory)]
    [ValidatePattern('^[a-z_][a-z0-9_-]{0,31}$')]
    [string] $ServiceGroup,

    [Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $IdentityFile,
    [Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $KnownHostsFile,

    [ValidateRange(1, 65535)]
    [int] $Port = 22,

    [Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $TokenFile,
    [string] $CalibrationFile,
    [string] $CalibrationProvenanceFile,
    [string] $RecoveryArchivePath,
    [ValidatePattern('^[0-9a-f]{64}$')] [string] $RecoveryArchiveSha256,
    [ValidateNotNullOrEmpty()] [string] $PythonExecutable = 'python',
    [Parameter(Mandatory)] [ValidateNotNullOrEmpty()] [string] $ExpectedHostName,
    [Parameter(Mandatory)] [ValidatePattern('^/dev/[A-Za-z0-9._/+:-]+$')] [string] $ExpectedRootDevice,
    [Parameter(Mandatory)] [ValidatePattern('^/dev/[A-Za-z0-9._/+:-]+$')] [string] $ExpectedBootDevice,
    [Parameter(Mandatory)] [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._:-]{2,127}$')] [string] $ExpectedControllerId,
    [Parameter(Mandatory)] [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{2,63}$')] [string] $ExpectedFirmwareVersion,
    [Parameter(Mandatory)] [ValidatePattern('^(?:[0-9]{1,3}\.){3}[0-9]{1,3}/(?:[0-9]|[12][0-9]|3[0-2])$')] [string] $DirectLanAddressCidr
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$softwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$operationsRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$runtimeRoot = Join-Path $softwareRoot 'runtime\robot-gateway'
$recoveryRoot = Join-Path $runtimeRoot 'recovery'
$installRoot = "/home/$UserName/arm-gateway"
$requirementsPath = Join-Path $operationsRoot 'requirements-pi.txt'
$baseUnitTemplatePath = Join-Path $operationsRoot 'deploy\arm-gateway.service'
$uartDropInTemplatePath = Join-Path $operationsRoot 'deploy\arm-gateway-physical-uart.conf'
$wayVncDropInPath = Join-Path $operationsRoot 'deploy\wayvnc-network-online.conf'
$deployScriptPath = Join-Path $PSScriptRoot 'deploy-arm-gateway.ps1'
$recoveryHelperPath = Join-Path $PSScriptRoot 'prepare_arm_recovery.py'
$baseReferenceMarkerPath = $null
$protectedRecoveryState = $null
$packageDirectory = Join-Path $softwareRoot 'python\robot_gateway'
$renderRoot = Join-Path $recoveryRoot 'rendered-deployment'
$baseUnitPath = Join-Path $renderRoot 'arm-gateway.service'
$uartDropInPath = Join-Path $renderRoot 'arm-gateway-physical-uart.conf'
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

if ($HostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$' -or $HostName.StartsWith('-')) {
    throw 'HostName must be a DNS name, IPv4 address, or unbracketed IPv6 literal.'
}
if ($ExpectedHostName -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$' -or $ExpectedHostName.StartsWith('-')) {
    throw 'ExpectedHostName must be a plain hostname.'
}
function Assert-RecoveryPathAncestry {
    param([Parameter(Mandatory)] [string] $Path)
    $cursor = [System.IO.Path]::GetFullPath($Path)
    while ($null -ne $cursor) {
        if (Test-Path -LiteralPath $cursor) {
            $item = Get-Item -LiteralPath $cursor -Force -ErrorAction Stop
            if (($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw 'Recovery output ancestry cannot use reparse points.'
            }
        }
        $parent = [System.IO.Directory]::GetParent($cursor)
        $cursor = if ($null -eq $parent) { $null } else { $parent.FullName }
    }
}

function Assert-PrivateRecoveryPath {
    param([Parameter(Mandatory)] [string] $Path)

    Assert-RecoveryPathAncestry -Path $Path
    $item = Get-Item -LiteralPath $Path -Force -ErrorAction Stop
    if (($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'Recovery outputs cannot use reparse points.'
    }
    $sid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User
    $sections = [System.Security.AccessControl.AccessControlSections]::Access -bor
        [System.Security.AccessControl.AccessControlSections]::Owner
    if ($Phase -eq 'Prepare') {
        $security = if ($item.PSIsContainer) {
            New-Object System.Security.AccessControl.DirectorySecurity
        } else { New-Object System.Security.AccessControl.FileSecurity }
        $security.SetAccessRuleProtection($true, $false)
        $inheritance = if ($item.PSIsContainer) {
            [System.Security.AccessControl.InheritanceFlags]::ContainerInherit -bor
            [System.Security.AccessControl.InheritanceFlags]::ObjectInherit
        } else { [System.Security.AccessControl.InheritanceFlags]::None }
        $rule = New-Object System.Security.AccessControl.FileSystemAccessRule(
            $sid, [System.Security.AccessControl.FileSystemRights]::FullControl, $inheritance,
            [System.Security.AccessControl.PropagationFlags]::None,
            [System.Security.AccessControl.AccessControlType]::Allow)
        [void]$security.AddAccessRule($rule)
        if ($item.PSIsContainer) { [System.IO.Directory]::SetAccessControl($item.FullName, $security) }
        else { [System.IO.File]::SetAccessControl($item.FullName, $security) }
    }
    $actual = if ($item.PSIsContainer) { [System.IO.Directory]::GetAccessControl($item.FullName, $sections) }
        else { [System.IO.File]::GetAccessControl($item.FullName, $sections) }
    $rules = @($actual.GetAccessRules($true, $true, [System.Security.Principal.SecurityIdentifier]))
    if (-not $actual.AreAccessRulesProtected -or $rules.Count -ne 1 -or
        $rules[0].IdentityReference.Value -cne $sid.Value -or
        $rules[0].FileSystemRights -ne [System.Security.AccessControl.FileSystemRights]::FullControl -or
        $rules[0].AccessControlType -ne [System.Security.AccessControl.AccessControlType]::Allow -or
        $actual.GetOwner([System.Security.Principal.SecurityIdentifier]).Value -cne $sid.Value) {
        throw 'Recovery output permissions are not owner-only; run laptop-only Prepare to protect them.'
    }
}

function Write-RenderedDeploymentFiles {
    Assert-RecoveryPathAncestry -Path $renderRoot
    foreach ($template in @($baseUnitTemplatePath, $uartDropInTemplatePath)) {
        if (-not (Test-Path -LiteralPath $template -PathType Leaf)) {
            throw "Required deployment template is missing: $template"
        }
    }
    if ($Phase -eq 'Prepare') {
        [void][System.IO.Directory]::CreateDirectory($recoveryRoot)
        Assert-PrivateRecoveryPath -Path $recoveryRoot
        [void][System.IO.Directory]::CreateDirectory($renderRoot)
    }
    elseif (-not (Test-Path -LiteralPath $renderRoot -PathType Container)) {
        throw 'Rendered recovery inputs are absent; run laptop-only Prepare first.'
    }
    Assert-PrivateRecoveryPath -Path $recoveryRoot
    Assert-PrivateRecoveryPath -Path $renderRoot
    $unit = Get-Content -Raw -LiteralPath $baseUnitTemplatePath
    $unit = $unit.Replace('__SERVICE_USER__', $UserName)
    $unit = $unit.Replace('__SERVICE_GROUP__', $ServiceGroup)
    $unit = $unit.Replace('__INSTALL_ROOT__', $installRoot)
    $dropIn = Get-Content -Raw -LiteralPath $uartDropInTemplatePath
    $dropIn = $dropIn.Replace('__INSTALL_ROOT__', $installRoot)
    foreach ($rendered in @($unit, $dropIn)) {
        if ($rendered -match '__[A-Z0-9_]+__') {
            throw 'A deployment template contains an unresolved placeholder.'
        }
    }
    $utf8NoBom = [System.Text.UTF8Encoding]::new($false)
    foreach ($entry in @(@($baseUnitPath, $unit), @($uartDropInPath, $dropIn))) {
        if ($Phase -eq 'Prepare') {
            if (Test-Path -LiteralPath $entry[0]) { Assert-PrivateRecoveryPath -Path $entry[0] }
            [System.IO.File]::WriteAllText($entry[0], $entry[1], $utf8NoBom)
        }
        elseif (-not (Test-Path -LiteralPath $entry[0] -PathType Leaf) -or
            [System.IO.File]::ReadAllText($entry[0]) -cne $entry[1]) {
            throw 'Rendered recovery inputs changed; run laptop-only Prepare first.'
        }
        Assert-PrivateRecoveryPath -Path $entry[0]
    }
}

Write-RenderedDeploymentFiles

function Resolve-RequiredFile {
    param([Parameter(Mandatory)] [string] $Path)

    $resolved = (Resolve-Path -LiteralPath $Path -ErrorAction Stop).Path
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
        throw "Required recovery input is not a file: $Path"
    }
    return $resolved
}

function Test-ValidGatewayTokenFile {
    param([Parameter(Mandatory)] [string] $Path)

    try {
        $file = Get-Item -LiteralPath $Path -ErrorAction Stop
        if ($file.PSIsContainer -or $file.Length -gt 1024) {
            return $false
        }
        $strictUtf8 = [System.Text.UTF8Encoding]::new($false, $true)
        $token = [System.IO.File]::ReadAllText($file.FullName, $strictUtf8).Trim()
        return $token -cmatch '^[\x21-\x7e]{32,256}$'
    }
    catch {
        return $false
    }
}

function Assert-RecoveredCalibration {
    param([Parameter(Mandatory)] [string] $Path)

    $document = Get-Content -Raw -LiteralPath $Path | ConvertFrom-Json
    $names = @('joint_1', 'joint_2', 'joint_3', 'joint_4')
    $expectedIds = @(1, 2, 3, 4)
    $requiredFields = @(
        'servoId', 'rawZero', 'rawMin', 'rawMax', 'ratio', 'direction', 'speed', 'accel'
    )
    for ($index = 0; $index -lt $names.Count; $index++) {
        $name = $names[$index]
        $property = $document.PSObject.Properties[$name]
        if ($null -eq $property) {
            throw "Recovered calibration is missing $name."
        }
        $joint = $property.Value
        foreach ($field in $requiredFields) {
            if ($null -eq $joint.PSObject.Properties[$field]) {
                throw "Recovered calibration $name is missing $field."
            }
        }
        if ([int]$joint.servoId -ne $expectedIds[$index]) {
            throw "Recovered calibration $name has the wrong servo ID."
        }
        if ([int]$joint.direction -notin @(-1, 1)) {
            throw "Recovered calibration $name has an invalid direction."
        }
        if ([double]$joint.ratio -le 0.01 -or [double]$joint.ratio -gt 64) {
            throw "Recovered calibration $name has an invalid ratio."
        }
        if ([int]$joint.speed -lt 1 -or [int]$joint.speed -gt 2400) {
            throw "Recovered calibration $name has an invalid speed."
        }
        if ([int]$joint.accel -lt 1 -or [int]$joint.accel -gt 50) {
            throw "Recovered calibration $name has an invalid acceleration."
        }
    }
    $unexpected = @(
        $document.PSObject.Properties.Name | Where-Object { $_ -notin $names }
    )
    if ($unexpected.Count -gt 0) {
        throw "Recovered calibration contains unexpected top-level keys: $($unexpected -join ', ')"
    }
}

function Get-Sha256Lower {
    param([Parameter(Mandatory)] [string] $Path)

    # Use the framework implementation so recovery still works in minimal
    # PowerShell environments where Microsoft.PowerShell.Utility is absent.
    $stream = [System.IO.File]::OpenRead($Path)
    try {
        $sha256 = [System.Security.Cryptography.SHA256]::Create()
        try {
            $bytes = $sha256.ComputeHash($stream)
            return ([System.BitConverter]::ToString($bytes) -replace '-', '').ToLowerInvariant()
        }
        finally {
            $sha256.Dispose()
        }
    }
    finally {
        $stream.Dispose()
    }
}

function Get-ReviewedGatewayPythonFiles {
    $pythonFiles = @(
        Get-ChildItem -LiteralPath $packageDirectory -File -Filter '*.py' |
            Sort-Object Name
    )
    $actualNames = @($pythonFiles | ForEach-Object { $_.Name })
    $expectedNames = @($expectedGatewayModules | Sort-Object)
    if ($actualNames.Count -ne $expectedNames.Count -or
        [string]::Join("`n", $actualNames) -cne [string]::Join("`n", $expectedNames)) {
        throw (
            'Expected exactly the reviewed 13 robot_gateway Python modules. ' +
            "Expected: $($expectedNames -join ', '). Actual: $($actualNames -join ', ')."
        )
    }
    return $pythonFiles
}

function Assert-RecoveredCalibrationProvenance {
    param(
        [Parameter(Mandatory)] [string] $CalibrationPath,
        [Parameter(Mandatory)] [string] $ProvenancePath
    )

    $provenance = Get-Content -Raw -LiteralPath $ProvenancePath | ConvertFrom-Json
    if ([string]$provenance.schema -ne 'arm-joints-recovery-provenance.v2') {
        throw 'Recovered calibration provenance has an unexpected schema.'
    }
    if ($null -eq $provenance.artifact -or $null -eq $provenance.artifact.sha256) {
        throw 'Recovered calibration provenance is missing artifact.sha256.'
    }
    $expected = ([string]$provenance.artifact.sha256).ToLowerInvariant()
    if ($expected -notmatch '^[0-9a-f]{64}$') {
        throw 'Recovered calibration provenance contains an invalid artifact SHA256.'
    }
    $actual = Get-Sha256Lower -Path $CalibrationPath
    if ($actual -cne $expected) {
        throw 'Recovered calibration does not match its provenance SHA256.'
    }
    if ([int]$provenance.artifact.jointCount -ne 4) {
        throw 'Recovered calibration provenance does not describe four joints.'
    }
    if ([string]$provenance.recovery.sourceKind -cne 'verified-protected-pi-backup' -or
        [string]$provenance.recovery.sourceArchiveSha256 -cne $RecoveryArchiveSha256 -or
        [string]$provenance.recovery.sourceMemberPath -cne 'var/lib/arm-gateway/arm-joints.json' -or
        [string]$provenance.recovery.sourceMemberSha256 -cne $actual -or
        $provenance.recovery.baseContinuityClaimed -ne $false -or
        $provenance.recovery.physicalBaseRezeroRequired -ne $true) {
        throw 'Calibration provenance must bind the verified archive and require physical Base re-zero.'
    }
}

function Initialize-ArchiveRecovery {
    if ([string]::IsNullOrWhiteSpace($RecoveryArchivePath) -or
        [string]::IsNullOrWhiteSpace($RecoveryArchiveSha256)) {
        throw "$Phase requires exact -RecoveryArchivePath and -RecoveryArchiveSha256 values."
    }
    Resolve-RequiredFile -Path $recoveryHelperPath | Out-Null
    $python = (Get-Command $PythonExecutable -CommandType Application -ErrorAction Stop |
        Select-Object -First 1).Source
    $arguments = @(
        $recoveryHelperPath, '--archive', $RecoveryArchivePath,
        '--sha256', $RecoveryArchiveSha256, '--output-root', $recoveryRoot,
        '--expected-host', $ExpectedHostName, '--expected-user', $UserName
    )
    if ($Phase -eq 'Prepare') { $arguments += '--create' }
    if ($CalibrationFile) { $arguments += @('--calibration', $CalibrationFile) }
    if ($CalibrationProvenanceFile) { $arguments += @('--provenance', $CalibrationProvenanceFile) }
    $result = & $python @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Protected archive recovery preparation failed.' }
    $script:protectedRecoveryState = ($result -join "`n") | ConvertFrom-Json
    $script:CalibrationFile = [string]$protectedRecoveryState.calibrationPath
    $script:CalibrationProvenanceFile = [string]$protectedRecoveryState.provenancePath
    $script:baseReferenceMarkerPath = [string]$protectedRecoveryState.baseReferenceMarkerPath
}

function Get-RecoveryInputs {
    $pythonFiles = @(Get-ReviewedGatewayPythonFiles)
    Initialize-ArchiveRecovery

    $required = @(
        $requirementsPath,
        $baseUnitPath,
        $uartDropInPath,
        $wayVncDropInPath,
        $deployScriptPath,
        $TokenFile,
        $CalibrationFile,
        $CalibrationProvenanceFile,
        $baseReferenceMarkerPath,
        $recoveryHelperPath
    )
    foreach ($path in $required) {
        Resolve-RequiredFile -Path $path | Out-Null
    }
    if (-not (Test-ValidGatewayTokenFile -Path $TokenFile)) {
        throw 'The private laptop gateway token is absent or invalid.'
    }
    Assert-RecoveredCalibration -Path $CalibrationFile
    Assert-RecoveredCalibrationProvenance `
        -CalibrationPath $CalibrationFile `
        -ProvenancePath $CalibrationProvenanceFile

    return @($pythonFiles.FullName) + @(
        $requirementsPath,
        $baseUnitPath,
        $uartDropInPath,
        $wayVncDropInPath,
        $deployScriptPath,
        $CalibrationFile,
        $CalibrationProvenanceFile,
        $baseReferenceMarkerPath,
        $recoveryHelperPath,
        $PSCommandPath
    )
}

function New-RemoteDigestContract {
    param(
        [Parameter(Mandatory)]
        [ValidateSet('Bootstrap', 'ActivatePreflight', 'Activate', 'Deactivate', 'Verify')]
        [string] $Scope,

        [string] $ManifestPath
    )

    $entries = @()
    if ($Scope -in @('Bootstrap', 'ActivatePreflight', 'Activate', 'Deactivate', 'Verify')) {
        if ([string]::IsNullOrWhiteSpace($ManifestPath)) {
            throw "A freshly prepared recovery manifest is required for the $Scope digest contract."
        }
        $resolvedManifestPath = Resolve-RequiredFile -Path $ManifestPath
        if ((Get-Sha256Lower -Path $resolvedManifestPath) -cne $script:preparedManifestSha256) {
            throw 'The prepared recovery manifest changed before digest-contract creation.'
        }
        $prepared = Get-Content -Raw -LiteralPath $resolvedManifestPath | ConvertFrom-Json
        $pythonFiles = @(Get-ReviewedGatewayPythonFiles)
        foreach ($source in $pythonFiles) {
            $entries += [pscustomobject]@{
                Mode = 'required'
                Source = $source.FullName
                Remote = "$installRoot/robot_gateway/$($source.Name)"
                Label = "source/$($source.Name)"
            }
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $requirementsPath
            Remote = "$installRoot/requirements-pi.txt"
            Label = 'requirements-pi'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $CalibrationFile
            Remote = '/var/lib/arm-gateway/arm-joints.json'
            Label = 'calibration'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $CalibrationProvenanceFile
            Remote = '/var/lib/arm-gateway/arm-joints.recovery-provenance.json'
            Label = 'calibration-provenance'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $baseReferenceMarkerPath
            Remote = '/var/lib/arm-gateway/base-reference-required.json'
            Label = 'base-reference-required'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $baseUnitPath
            Remote = '/etc/systemd/system/arm-gateway.service'
            Label = 'arm-gateway-service'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $wayVncDropInPath
            Remote = '/etc/systemd/system/wayvnc.service.d/10-network-online.conf'
            Label = 'wayvnc-drop-in'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $TokenFile
            Remote = "$installRoot/.robot-gateway.token"
            Label = 'gateway-token'
        }
        $entries += [pscustomobject]@{
            Mode = 'required'
            Source = $resolvedManifestPath
            Remote = '/var/lib/arm-gateway/recovery-manifest.json'
            Label = 'recovery-manifest'
        }
    }
    if ($Scope -in @('ActivatePreflight', 'Activate', 'Deactivate', 'Verify')) {
        $entries += [pscustomobject]@{
            Mode = if ($Scope -in @('Activate', 'Deactivate')) { 'required' } else { 'optional' }
            Source = $uartDropInPath
            Remote = '/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf'
            Label = 'physical-uart-drop-in'
        }
    }

    $rows = foreach ($entry in $entries) {
        $sourcePath = Resolve-RequiredFile -Path $entry.Source
        if ($entry.Label -eq 'recovery-manifest') { $digest = $script:preparedManifestSha256 }
        elseif ($entry.Label -eq 'gateway-token') { $digest = [string]$prepared.privateToken.sha256 }
        else {
            $relative = try { [System.IO.Path]::GetRelativePath($softwareRoot, $sourcePath) }
                catch { $sourcePath }
            $key = $relative.Replace('\', '/')
            $preparedRows = @($prepared.inputs | Where-Object { [string]$_.path -ceq $key })
            if ($preparedRows.Count -ne 1) { throw 'Digest input is missing from the prepared manifest.' }
            $digest = [string]$preparedRows[0].sha256
        }
        if ($digest -notmatch '^[0-9a-f]{64}$' -or (Get-Sha256Lower -Path $sourcePath) -cne $digest) {
            throw 'Recovery input changed after the prepared manifest was validated.'
        }
        "$($entry.Mode)`t$digest`t$($entry.Remote)`t$($entry.Label)"
    }
    $temporaryPath = Join-Path $env:TEMP (
        'arm-pi-expected-digests-' + [Guid]::NewGuid().ToString('N') + '.tsv'
    )
    # The remote verifier is POSIX sh. Write LF explicitly so the final TSV
    # label never carries a Windows CR byte and fails the strict label parser.
    [System.IO.File]::WriteAllText(
        $temporaryPath,
        (([string[]]$rows -join "`n") + "`n"),
        [System.Text.UTF8Encoding]::new($false)
    )
    return $temporaryPath
}

function New-RecoveryManifest {
    param([Parameter(Mandatory)] [string[]] $Inputs)

    New-Item -ItemType Directory -Force -Path $recoveryRoot | Out-Null
    $generated = [DateTime]::UtcNow
    $rows = foreach ($path in $Inputs) {
        $item = Get-Item -LiteralPath $path
        $relative = try {
            [System.IO.Path]::GetRelativePath($softwareRoot, $item.FullName)
        }
        catch {
            $item.FullName
        }
        [ordered]@{
            path = $relative.Replace('\', '/')
            bytes = $item.Length
            sha256 = Get-Sha256Lower -Path $item.FullName
        }
    }
    $tokenItem = Get-Item -LiteralPath $TokenFile
    $manifest = [ordered]@{
        schema = 'arm-pi-recovery-kit.v3'
        generatedUtc = $generated.ToString('o')
        expectedHost = $HostName
        sshPort = $Port
        expectedUser = $UserName
        expectedHostName = $ExpectedHostName
        expectedRootDevice = $ExpectedRootDevice
        expectedBootDevice = $ExpectedBootDevice
        directLanAddressCidr = $DirectLanAddressCidr
        installRoot = $installRoot
        serviceGroup = $ServiceGroup
        controller = [ordered]@{
            identity = $ExpectedControllerId
            firmware = $ExpectedFirmwareVersion
        }
        privateToken = [ordered]@{
            path = 'runtime/robot-gateway/arm-pi.token'
            bytes = $tokenItem.Length
            sha256 = Get-Sha256Lower -Path $tokenItem.FullName
            copiedIntoManifest = $false
        }
        calibrationRecovery = [ordered]@{
            sourceArchiveSha256 = $RecoveryArchiveSha256
            calibrationSha256 = Get-Sha256Lower -Path $CalibrationFile
            baseReferenceMarkerSha256 = Get-Sha256Lower -Path $baseReferenceMarkerPath
            baseContinuityClaimed = $false
            physicalBaseRezeroRequired = $true
        }
        inputs = @($rows)
        phases = @(
            'Bootstrap restores the dormant gateway, token, calibration, and WayVNC ordering.',
            'ConfigureUart changes boot configuration but never reboots.',
            'Activate requires a supported arm, fail-closed latch, physical UART drop-in, and authenticated torque-off controller proof.',
            'Deactivate requires a supported arm, retains the fail-closed latch, and restores the dormant service.',
            'Verify is read-only; Base physical zero remains a separate operator-confirmed action.'
        )
    }

    $stamp = $generated.ToString('yyyyMMddTHHmmssZ')
    $manifestPath = Join-Path $recoveryRoot ("prepared-$stamp-" + [Guid]::NewGuid().ToString('N') + '.json')
    $encoded = $manifest | ConvertTo-Json -Depth 8
    [System.IO.File]::WriteAllText(
        $manifestPath,
        $encoded + [Environment]::NewLine,
        [System.Text.UTF8Encoding]::new($false)
    )
    $script:preparedManifestSha256 = Get-Sha256Lower -Path $manifestPath
    Assert-PrivateRecoveryPath -Path $manifestPath
    $latestPath = Join-Path $recoveryRoot 'LATEST.json'
    $temporaryLatest = Join-Path $recoveryRoot ('.LATEST-' + [Guid]::NewGuid().ToString('N') + '.json')
    [System.IO.File]::WriteAllText(
        $temporaryLatest,
        $encoded + [Environment]::NewLine,
        [System.Text.UTF8Encoding]::new($false)
    )
    Move-Item -LiteralPath $temporaryLatest -Destination $latestPath -Force
    Assert-PrivateRecoveryPath -Path $latestPath
    return $manifestPath
}

function Resolve-PreparedRecoveryManifest {
    param([Parameter(Mandatory)] [string] $Path)

    $resolved = Resolve-RequiredFile -Path $Path
    Assert-PrivateRecoveryPath -Path $resolved
    try {
        $strictUtf8 = [System.Text.UTF8Encoding]::new($false, $true)
        $document = [System.IO.File]::ReadAllText($resolved, $strictUtf8) | ConvertFrom-Json
    }
    catch {
        throw "The prepared recovery manifest is not valid UTF-8 JSON: $resolved"
    }
    if ([string]$document.schema -ne 'arm-pi-recovery-kit.v3') {
        throw 'The prepared recovery manifest has an unexpected schema.'
    }
    if ([string]$document.expectedHost -cne $HostName -or
        [int]$document.sshPort -ne $Port -or
        [string]$document.expectedUser -cne $UserName) {
        throw 'The prepared recovery manifest belongs to a different recovery target.'
    }
    if ([string]$document.expectedHostName -cne $ExpectedHostName -or
        [string]$document.expectedRootDevice -cne $ExpectedRootDevice -or
        [string]$document.expectedBootDevice -cne $ExpectedBootDevice -or
        [string]$document.directLanAddressCidr -cne $DirectLanAddressCidr -or
        [string]$document.installRoot -cne $installRoot -or
        [string]$document.serviceGroup -cne $ServiceGroup) {
        throw 'The prepared recovery manifest has an unexpected SD device contract.'
    }
    if ([string]$document.controller.identity -cne $ExpectedControllerId -or
        [string]$document.controller.firmware -cne $ExpectedFirmwareVersion) {
        throw 'The prepared recovery manifest has an unexpected controller contract.'
    }
    if ($null -eq $document.privateToken -or
        $document.privateToken.copiedIntoManifest -ne $false -or
        [string]$document.privateToken.sha256 -cne (Get-Sha256Lower -Path $TokenFile) -or
        [long]$document.privateToken.bytes -ne (Get-Item -LiteralPath $TokenFile).Length) {
        throw 'The prepared recovery manifest does not preserve the private-token boundary.'
    }
    if ([string]$document.calibrationRecovery.sourceArchiveSha256 -cne $RecoveryArchiveSha256 -or
        [string]$document.calibrationRecovery.calibrationSha256 -cne (Get-Sha256Lower -Path $CalibrationFile) -or
        [string]$document.calibrationRecovery.baseReferenceMarkerSha256 -cne (Get-Sha256Lower -Path $baseReferenceMarkerPath) -or
        $document.calibrationRecovery.baseContinuityClaimed -ne $false -or
        $document.calibrationRecovery.physicalBaseRezeroRequired -ne $true) {
        throw 'The prepared recovery manifest does not preserve the fail-closed Base-reference policy.'
    }
    $generated = [DateTimeOffset]::MinValue
    if (-not [DateTimeOffset]::TryParse(
            [string]$document.generatedUtc,
            [System.Globalization.CultureInfo]::InvariantCulture,
            [System.Globalization.DateTimeStyles]::RoundtripKind,
            [ref]$generated
        )) {
        throw 'The prepared recovery manifest has an invalid generatedUtc value.'
    }
    $inputs = @($document.inputs)
    $currentInputs = @($script:inputs)
    if ($inputs.Count -ne $currentInputs.Count) {
        throw 'The prepared recovery manifest does not contain the complete current input set.'
    }
    $expectedPaths = @{}
    foreach ($inputPath in $currentInputs) {
        $item = Get-Item -LiteralPath $inputPath
        $relative = try { [System.IO.Path]::GetRelativePath($softwareRoot, $item.FullName) }
            catch { $item.FullName }
        $expectedPaths[$relative.Replace('\', '/')] = $item
    }
    $seen = @{}
    foreach ($inputRow in $inputs) {
        if ([string]::IsNullOrWhiteSpace([string]$inputRow.path) -or
            [long]$inputRow.bytes -lt 0 -or
            [string]$inputRow.sha256 -notmatch '^[0-9a-f]{64}$') {
            throw 'The prepared recovery manifest contains an invalid input digest row.'
        }
        $key = [string]$inputRow.path
        if ($seen.ContainsKey($key) -or -not $expectedPaths.ContainsKey($key)) {
            throw 'The prepared recovery manifest has duplicate or unexpected input paths.'
        }
        $seen[$key] = $true
        $current = $expectedPaths[$key]
        if ([long]$inputRow.bytes -ne $current.Length -or
            [string]$inputRow.sha256 -cne (Get-Sha256Lower -Path $current.FullName)) {
            throw 'The prepared recovery manifest is stale for current recovery inputs.'
        }
    }
    $script:preparedManifestSha256 = Get-Sha256Lower -Path $resolved
    return $resolved
}

function New-SshContext {
    $sshPath = (Get-Command ssh -CommandType Application -ErrorAction Stop).Source
    $scpPath = (Get-Command scp -CommandType Application -ErrorAction Stop).Source
    $identityPath = Resolve-RequiredFile -Path $IdentityFile
    $knownHostsPath = Resolve-RequiredFile -Path $KnownHostsFile
    $stagedKnownHosts = Join-Path $env:TEMP ('arm-pi-known-hosts-' + [Guid]::NewGuid().ToString('N'))
    Copy-Item -LiteralPath $knownHostsPath -Destination $stagedKnownHosts
    $remoteHost = if ($HostName.Contains(':')) { "[$HostName]" } else { $HostName }
    $remoteTarget = "${UserName}@${remoteHost}"
    $shared = @(
        '-i', $identityPath,
        '-o', "UserKnownHostsFile=$stagedKnownHosts",
        '-o', 'StrictHostKeyChecking=yes',
        '-o', 'IdentitiesOnly=yes',
        '-o', 'BatchMode=yes',
        '-o', 'PasswordAuthentication=no',
        '-o', 'ConnectTimeout=8'
    )
    return [pscustomobject]@{
        SshPath = $sshPath
        ScpPath = $scpPath
        SshArguments = @('-p', [string] $Port) + $shared
        ScpArguments = @('-q', '-P', [string] $Port) + $shared
        RemoteTarget = $remoteTarget
        StagedKnownHosts = $stagedKnownHosts
    }
}

function Remove-SshContext {
    param([Parameter(Mandatory)] $Context)
    if (Test-Path -LiteralPath $Context.StagedKnownHosts) {
        Remove-Item -LiteralPath $Context.StagedKnownHosts -Force
    }
}

function Invoke-RemoteScript {
    param(
        [Parameter(Mandatory)] $Context,
        [Parameter(Mandatory)] [string] $ScriptText,
        [Parameter(Mandatory)] [string] $Operation,
        [string[]] $Arguments = @()
    )
    $payload = ($ScriptText -replace "`r", '') + "`n#"
    $payload | & $Context.SshPath @($Context.SshArguments) $Context.RemoteTarget 'sh -s --' @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Operation failed with exit code $LASTEXITCODE."
    }
}

function Copy-ToRemoteStage {
    param(
        [Parameter(Mandatory)] $Context,
        [Parameter(Mandatory)] [string] $Source,
        [Parameter(Mandatory)] [string] $RemotePath
    )
    & $Context.ScpPath @($Context.ScpArguments) $Source "$($Context.RemoteTarget):$RemotePath"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to stage $Source."
    }
}

function Invoke-RemoteDigestCheck {
    param(
        [Parameter(Mandatory)] $Context,
        [Parameter(Mandatory)] [string] $Stage,
        [Parameter(Mandatory)] [string] $Operation
    )

    $script = @'
set -eu
stage=$1
case "$stage" in
  /tmp/arm-pi-recovery-[0-9a-f]*|/tmp/arm-pi-uart-[0-9a-f]*|/tmp/arm-pi-verify-[0-9a-f]*) ;;
  *) echo 'Refusing unexpected integrity staging path.' >&2; exit 2 ;;
esac
contract="$stage/expected-digests.tsv"
[ -f "$contract" ] || { echo 'Expected-digest contract is missing.' >&2; exit 50; }

checked=0
optional_missing=0
tab=$(printf '\t')
while IFS="$tab" read -r mode expected path label; do
  case "$mode" in required|optional) ;; *) echo 'Digest contract mode is invalid.' >&2; exit 51 ;; esac
  case "$expected" in
    *[!0-9a-f]*|'') echo 'Digest contract SHA256 is invalid.' >&2; exit 52 ;;
  esac
  [ "${#expected}" -eq 64 ] || { echo 'Digest contract SHA256 has the wrong length.' >&2; exit 53; }
  case "$path" in /*) ;; *) echo 'Digest contract path is invalid.' >&2; exit 54 ;; esac
  case "$label" in *[!A-Za-z0-9._/-]*|'') echo 'Digest contract label is invalid.' >&2; exit 55 ;; esac

  if ! sudo test -f "$path" || sudo test -L "$path"; then
    if [ "$mode" = optional ] && ! sudo test -e "$path" && ! sudo test -L "$path"; then
      echo "Integrity note: $label is not installed (expected before physical activation)."
      optional_missing=$((optional_missing + 1))
      continue
    fi
    echo "Integrity mismatch: $label is missing or not a regular non-symlink file." >&2
    exit 56
  fi
  actual=$(sudo sha256sum -- "$path" 2>/dev/null | awk '{print $1}') || actual=
  [ -n "$actual" ] && [ "$actual" = "$expected" ] || {
    echo "Integrity mismatch: $label differs from the prepared laptop artifact." >&2
    exit 57
  }
  checked=$((checked + 1))
done < "$contract"

[ "$checked" -gt 0 ] || { echo 'Digest contract checked no installed artifacts.' >&2; exit 58; }
echo "Remote integrity verification passed for $checked installed artifacts; $optional_missing optional artifact(s) absent."
'@
    Invoke-RemoteScript `
        -Context $Context `
        -ScriptText $script `
        -Operation $Operation `
        -Arguments @($Stage)
}

function Invoke-AuthenticatedControllerSafetyGate {
    param([Parameter(Mandatory)] $Context)

    $script = @'
set -eu
install_root=$1
expected_controller_id=$2
expected_firmware_version=$3
python3 - "$install_root" "$expected_controller_id" "$expected_firmware_version" <<'PY'
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

install_root, expected_controller_id, expected_firmware_version = sys.argv[1:]
token = Path(install_root, ".robot-gateway.token").read_text(
    encoding="utf-8"
).strip()
request = urllib.request.Request(
    "http://127.0.0.1:8787/api/robot/arm/state",
    headers={"Authorization": f"Bearer {token}"},
    method="GET",
)
acceptable_reasons = {
    "STOP_DELIVERY_UNKNOWN",
    "EXPLICIT_STOP",
    "MOVE_SET_FAILED",
    "MOVE_SET_UNCONFIRMED",
    "SAFETY_FAULT",
}


def fresh(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and 0 <= value <= 1000
    )


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
    boot_id = controller.get("bootId")
    if not isinstance(boot_id, str) or not boot_id:
        problems.append("controller boot identity is absent")
    if controller.get("multiTurnAbsoluteV1") is not True:
        problems.append("multi-turn absolute capability is absent")
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
    if state.get("baseReferenceRequired") is not True:
        problems.append("recovered Base-reference gate is absent")
    if state.get("safetyStopReason") not in acceptable_reasons:
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
    by_name = {
        row.get("id"): row for row in rows if isinstance(row, dict)
    }
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
        with urllib.request.urlopen(request, timeout=2) as response:
            payload = json.loads(response.read(1_000_000))
        last = failures(payload)
        if not last:
            print(
                "Authenticated Pi-local controller safety gate passed: expected "
                f"{expected_firmware_version} controller and four torque-off stationary joints are online "
                "with fresh telemetry, STOP and inspection latched, floor guard on, "
                "and no cached collision reported."
            )
            raise SystemExit(0)
    except (OSError, UnicodeError, ValueError, urllib.error.URLError):
        last = ["authenticated state request was unavailable or invalid"]
    time.sleep(1)

print("Controller safety gate failed: " + "; ".join(last), file=sys.stderr)
raise SystemExit(60)
PY
'@
    Invoke-RemoteScript `
        -Context $Context `
        -ScriptText $script `
        -Operation 'Authenticated Pi-local controller safety verification' `
        -Arguments @($installRoot, $ExpectedControllerId, $ExpectedFirmwareVersion)
}

function Assert-ReplacementSdBoot {
    param([Parameter(Mandatory)] $Context)
    $script = @'
set -eu
expected_root=$1
expected_boot=$2
expected_hostname=$3
root=$(findmnt -n -o SOURCE /)
boot=$(findmnt -n -o SOURCE /boot/firmware)
[ "$root" = "$expected_root" ] || {
  echo "Refusing restore: root is $root, expected $expected_root." >&2
  exit 20
}
[ "$boot" = "$expected_boot" ] || {
  echo "Refusing restore: boot is $boot, expected $expected_boot." >&2
  exit 21
}
[ "$(hostname)" = "$expected_hostname" ] || {
  echo "Refusing restore: unexpected hostname $(hostname)." >&2
  exit 22
}
sudo -n true
'@
    Invoke-RemoteScript `
        -Context $Context `
        -ScriptText $script `
        -Operation 'Replacement SD identity check' `
        -Arguments @($ExpectedRootDevice, $ExpectedBootDevice, $ExpectedHostName)
}

function Invoke-Bootstrap {
    param(
        [Parameter(Mandatory)] $Context,
        [Parameter(Mandatory)] [string] $ManifestPath
    )
    Assert-ReplacementSdBoot -Context $Context
    $stage = '/tmp/arm-pi-recovery-' + [Guid]::NewGuid().ToString('N')
    $digestContractPath = New-RemoteDigestContract -Scope Bootstrap -ManifestPath $ManifestPath
    $stagePrepared = $false
    try {
        $prepare = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-recovery-[0-9a-f]*) ;; *) exit 2 ;; esac
umask 077
mkdir -- "$stage"
'@
        Invoke-RemoteScript -Context $Context -ScriptText $prepare -Operation 'Remote recovery staging' -Arguments @($stage)
        $stagePrepared = $true
        Copy-ToRemoteStage -Context $Context -Source $TokenFile -RemotePath "$stage/arm-pi.token"
        Copy-ToRemoteStage -Context $Context -Source $CalibrationFile -RemotePath "$stage/arm-joints.json"
        Copy-ToRemoteStage -Context $Context -Source $CalibrationProvenanceFile -RemotePath "$stage/arm-joints.provenance.json"
        Copy-ToRemoteStage -Context $Context -Source $baseReferenceMarkerPath -RemotePath "$stage/base-reference-required.json"
        Copy-ToRemoteStage -Context $Context -Source $wayVncDropInPath -RemotePath "$stage/wayvnc-network-online.conf"
        Copy-ToRemoteStage -Context $Context -Source $ManifestPath -RemotePath "$stage/recovery-manifest.json"
        Copy-ToRemoteStage -Context $Context -Source $digestContractPath -RemotePath "$stage/expected-digests.tsv"

        $seedToken = @'
set -eu
stage=$1
service_user=$2
service_group=$3
install_root=$4
case "$stage" in /tmp/arm-pi-recovery-[0-9a-f]*) ;; *) exit 2 ;; esac
case "$install_root" in /home/*/arm-gateway) ;; *) exit 2 ;; esac
sudo install -d -m 0750 -o "$service_user" -g "$service_group" "$install_root"
sudo install -m 0600 -o "$service_user" -g "$service_group" "$stage/arm-pi.token" "$install_root/.robot-gateway.token"
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $seedToken `
            -Operation 'Private token seeding' `
            -Arguments @($stage, $UserName, $ServiceGroup, $installRoot)

        & $deployScriptPath `
            -HostName $HostName `
            -UserName $UserName `
            -ServiceGroup $ServiceGroup `
            -IdentityFile $IdentityFile `
            -KnownHostsFile $KnownHostsFile `
            -Port $Port
        if ($LASTEXITCODE -ne 0) {
            throw 'Dormant gateway deployment failed.'
        }

        $installState = @'
set -eu
stage=$1
service_user=$2
service_group=$3
case "$stage" in /tmp/arm-pi-recovery-[0-9a-f]*) ;; *) exit 2 ;; esac
if ! systemctl cat wayvnc.service >/dev/null 2>&1; then
  echo 'WayVNC system service is absent; no-touch desktop autostart cannot be restored safely.' >&2
  exit 6
fi
sudo systemctl stop arm-gateway.service
sudo install -d -m 0700 -o "$service_user" -g "$service_group" /var/lib/arm-gateway
sudo install -m 0600 -o "$service_user" -g "$service_group" "$stage/base-reference-required.json" /var/lib/arm-gateway/base-reference-required.json
marker_expected=$(awk -F '\t' '$4 == "base-reference-required" {print $2}' "$stage/expected-digests.tsv")
sudo python3 - "$marker_expected" <<'PY_BASE_GATE_DURABLE'
import hashlib
import os
from pathlib import Path
import stat
import sys

marker = Path('/var/lib/arm-gateway/base-reference-required.json')
if not stat.S_ISREG(marker.lstat().st_mode):
    raise SystemExit('Base-reference marker is not a regular file')
with marker.open('rb') as source:
    if hashlib.sha256(source.read()).hexdigest() != sys.argv[1]:
        raise SystemExit('Base-reference marker differs from the prepared digest')
    os.fsync(source.fileno())
directory = os.open(marker.parent, os.O_RDONLY)
try:
    os.fsync(directory)
finally:
    os.close(directory)
PY_BASE_GATE_DURABLE
# Only replace calibration after its recovery gate is durable on disk.
sudo install -m 0600 -o "$service_user" -g "$service_group" "$stage/arm-joints.json" /var/lib/arm-gateway/arm-joints.json
sudo install -m 0600 -o "$service_user" -g "$service_group" "$stage/arm-joints.provenance.json" /var/lib/arm-gateway/arm-joints.recovery-provenance.json
sudo install -m 0600 -o "$service_user" -g "$service_group" "$stage/recovery-manifest.json" /var/lib/arm-gateway/recovery-manifest.json
sudo install -d -m 0755 -o root -g root /etc/systemd/system/wayvnc.service.d
sudo install -m 0644 -o root -g root "$stage/wayvnc-network-online.conf" /etc/systemd/system/wayvnc.service.d/10-network-online.conf
sudo systemctl daemon-reload
sudo systemctl enable wayvnc.service >/dev/null
sudo systemctl is-enabled --quiet wayvnc.service || {
  echo 'WayVNC exists but could not be enabled for no-touch boot.' >&2
  exit 7
}
if systemctl is-active --quiet wayvnc.service; then
  echo 'WayVNC was already active; Bootstrap left the running GUI session untouched.'
else
  echo 'WayVNC is enabled for the next boot and was deliberately not started in this session.'
fi
sudo systemctl start arm-gateway.service
attempt=0
while [ "$attempt" -lt 20 ]; do
  if curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null; then
    sudo systemctl is-active --quiet arm-gateway.service
    echo 'Dormant gateway process is active and /healthz is ready; no physical controller or camera health is claimed.'
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 1
done
echo 'Dormant gateway did not become healthy.' >&2
sudo systemctl --no-pager --full status arm-gateway.service >&2 || true
exit 4
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $installState `
            -Operation 'Calibration and service bootstrap' `
            -Arguments @($stage, $UserName, $ServiceGroup)
        Invoke-RemoteDigestCheck `
            -Context $Context `
            -Stage $stage `
            -Operation 'Bootstrap remote artifact integrity verification'
    }
    finally {
        if ($stagePrepared) {
            $cleanup = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-recovery-[0-9a-f]*) ;; *) exit 2 ;; esac
sudo find "$stage" -depth -type f -delete 2>/dev/null || true
sudo find "$stage" -depth -type d -empty -delete 2>/dev/null || true
'@
            try {
        Invoke-RemoteScript -Context $Context -ScriptText $cleanup -Operation 'Remote recovery cleanup' -Arguments @($stage)
            }
            catch {
                Write-Warning $_.Exception.Message
            }
        }
        if (Test-Path -LiteralPath $digestContractPath) {
            Remove-Item -LiteralPath $digestContractPath -Force
        }
    }
}

function Invoke-ConfigureUart {
    param([Parameter(Mandatory)] $Context)
    Assert-ReplacementSdBoot -Context $Context
    $script = @'
set -eu
stamp=$(date -u +%Y%m%dT%H%M%SZ)
backup="/var/lib/arm-gateway/recovery-boot-config/$stamp"
sudo install -d -m 0700 -o root -g root "$backup"
sudo cp -a /boot/firmware/cmdline.txt "$backup/cmdline.txt"
sudo cp -a /boot/firmware/config.txt "$backup/config.txt"
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
if grep -Eq '(^| )(console=(serial0|ttyAMA0|ttyS0),[^ ]+)' /boot/firmware/cmdline.txt; then
  echo 'Serial console is still present after configuration.' >&2
  exit 30
fi
grep -Eq '^enable_uart=1([[:space:]]|$)' /boot/firmware/config.txt || {
  echo 'enable_uart=1 is missing after configuration.' >&2
  exit 31
}
echo 'UART boot configuration is ready. A deliberate reboot is required; none was performed.'
'@
    Invoke-RemoteScript -Context $Context -ScriptText $script -Operation 'UART boot configuration'
}

function Invoke-Activate {
    param([Parameter(Mandatory)] $Context, [Parameter(Mandatory)] [string] $ManifestPath)
    Assert-ReplacementSdBoot -Context $Context
    $stage = '/tmp/arm-pi-uart-' + [Guid]::NewGuid().ToString('N')
    $digestContractPath = New-RemoteDigestContract -Scope Activate -ManifestPath $ManifestPath
    $preflightDigestContractPath = New-RemoteDigestContract -Scope ActivatePreflight -ManifestPath $ManifestPath
    $expectedDropInSha256 = Get-Sha256Lower -Path $uartDropInPath
    $stagePrepared = $false
    try {
        $prepare = @'
set -eu
stage=$1
service_user=$2
case "$stage" in /tmp/arm-pi-uart-[0-9a-f]*) ;; *) exit 2 ;; esac
umask 077
mkdir -- "$stage"
'@
        Invoke-RemoteScript -Context $Context -ScriptText $prepare -Operation 'UART staging' -Arguments @($stage)
        $stagePrepared = $true
        Copy-ToRemoteStage -Context $Context -Source $uartDropInPath -RemotePath "$stage/20-arm-controller-uart.conf"
        Copy-ToRemoteStage -Context $Context -Source $preflightDigestContractPath -RemotePath "$stage/expected-digests.tsv"
        Invoke-RemoteDigestCheck -Context $Context -Stage $stage -Operation 'Pre-activation full prepared-install integrity verification'
        $script = @'
set -eu
stage=$1
service_user=$2
service_group=$3
expected_drop_in_sha256=$4
case "$stage" in /tmp/arm-pi-uart-[0-9a-f]*) ;; *) exit 2 ;; esac
case "$expected_drop_in_sha256" in
  *[!0-9a-f]*|'') echo 'Expected UART drop-in digest is invalid.' >&2; exit 39 ;;
esac
[ "${#expected_drop_in_sha256}" -eq 64 ] || {
  echo 'Expected UART drop-in digest has the wrong length.' >&2
  exit 39
}
staged_drop_in="$stage/20-arm-controller-uart.conf"
[ -f "$staged_drop_in" ] && [ ! -L "$staged_drop_in" ] || {
  echo 'The staged UART drop-in is missing or has the wrong type.' >&2
  exit 39
}
staged_sha256=$(sha256sum -- "$staged_drop_in" | awk '{print $1}')
[ "$staged_sha256" = "$expected_drop_in_sha256" ] || {
  echo 'The staged UART drop-in differs from the reviewed laptop artifact.' >&2
  exit 39
}
device=$(readlink -f /dev/serial0 2>/dev/null || true)
[ -n "$device" ] || { echo '/dev/serial0 is unavailable.' >&2; exit 40; }
if grep -Eq '(^| )(console=(serial0|ttyAMA0|ttyS0),[^ ]+)' /boot/firmware/cmdline.txt; then
  echo 'Refusing physical activation while a serial console remains.' >&2
  exit 41
fi
grep -Eq '^enable_uart=1([[:space:]]|$)' /boot/firmware/config.txt || {
  echo 'Refusing physical activation because enable_uart=1 is absent.' >&2
  exit 42
}
drop_in=/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
latch=/var/lib/arm-gateway/arm-clear-required.json
fragment=$(sudo systemctl show arm-gateway.service --property=FragmentPath --value)
[ "$fragment" = /etc/systemd/system/arm-gateway.service ] || {
  echo "Refusing physical activation because the base service fragment is unexpected: $fragment" >&2
  exit 43
}
drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
case "$drop_ins" in
  ''|"$drop_in") ;;
  *) echo "Refusing physical activation with unexpected service drop-ins: $drop_ins" >&2; exit 43 ;;
esac
if sudo test -L "$drop_in" || { sudo test -e "$drop_in" && ! sudo test -f "$drop_in"; }; then
  echo 'Refusing physical activation because the UART drop-in path has the wrong type.' >&2
  exit 43
fi
if sudo test -f "$drop_in"; then
  installed_sha256=$(sudo sha256sum -- "$drop_in" | awk '{print $1}')
  [ "$installed_sha256" = "$expected_drop_in_sha256" ] || {
    echo 'Refusing physical activation because the installed UART drop-in differs from reviewed source.' >&2
    exit 43
  }
fi
if sudo test -L "$latch" || { sudo test -e "$latch" && ! sudo test -f "$latch"; }; then
  echo 'Refusing physical activation because the durable safety latch is a symlink or wrong type.' >&2
  exit 44
fi
sudo install -d -m 0700 -o "$service_user" -g "$service_group" /var/lib/arm-gateway
sudo python3 - "$service_user" "$service_group" <<'PY'
import grp
import json
import os
import pwd
import stat
import tempfile
import sys

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
echo 'The durable fail-closed latch is present, valid, and retained for physical activation.'
sudo systemctl stop arm-gateway.service
sudo install -d -m 0755 -o root -g root /etc/systemd/system/arm-gateway.service.d
sudo install -m 0644 -o root -g root "$stage/20-arm-controller-uart.conf" /etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
sudo systemctl daemon-reload
sudo systemctl restart arm-gateway.service
attempt=0
while [ "$attempt" -lt 20 ]; do
  if curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null; then
    sudo systemctl is-active --quiet arm-gateway.service
    echo "Gateway process is active and /healthz is ready with the reviewed UART drop-in at $device. This does not prove controller, servo-bus, or camera health."
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 1
done
echo 'Physical gateway did not become healthy.' >&2
sudo systemctl --no-pager --full status arm-gateway.service >&2 || true
exit 4
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $script `
            -Operation 'Physical UART activation' `
            -Arguments @($stage, $UserName, $ServiceGroup, $expectedDropInSha256)
        Copy-ToRemoteStage -Context $Context -Source $digestContractPath -RemotePath "$stage/expected-digests.tsv"
        Invoke-RemoteDigestCheck `
            -Context $Context `
            -Stage $stage `
            -Operation 'Physical UART drop-in integrity verification'
        Invoke-AuthenticatedControllerSafetyGate -Context $Context
        Write-Host (
            'Activation issued no STOP clear, calibration action, torque, motion, or camera capture. ' +
            'The Pi-local authenticated controller safety gate passed; typed controller ' +
            'corroboration and camera/autofocus verification remain required.'
        )
    }
    finally {
        if ($stagePrepared) {
            $cleanup = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-uart-[0-9a-f]*) ;; *) exit 2 ;; esac
sudo find "$stage" -depth -type f -delete 2>/dev/null || true
sudo find "$stage" -depth -type d -empty -delete 2>/dev/null || true
'@
            try {
                Invoke-RemoteScript -Context $Context -ScriptText $cleanup -Operation 'UART staging cleanup' -Arguments @($stage)
            }
            catch {
                Write-Warning $_.Exception.Message
            }
        }
        if (Test-Path -LiteralPath $digestContractPath) {
            Remove-Item -LiteralPath $digestContractPath -Force
        }
        if (Test-Path -LiteralPath $preflightDigestContractPath) {
            Remove-Item -LiteralPath $preflightDigestContractPath -Force
        }
    }
}

function Invoke-Deactivate {
    param([Parameter(Mandatory)] $Context)
    Assert-ReplacementSdBoot -Context $Context
    $stage = '/tmp/arm-pi-uart-' + [Guid]::NewGuid().ToString('N')
    $digestContractPath = New-RemoteDigestContract -Scope Deactivate
    $stagePrepared = $false
    try {
        $prepare = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-uart-[0-9a-f]*) ;; *) exit 2 ;; esac
umask 077
mkdir -- "$stage"
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $prepare `
            -Operation 'UART deactivation staging' `
            -Arguments @($stage)
        $stagePrepared = $true
        Copy-ToRemoteStage `
            -Context $Context `
            -Source $digestContractPath `
            -RemotePath "$stage/expected-digests.tsv"
        Invoke-RemoteDigestCheck `
            -Context $Context `
            -Stage $stage `
            -Operation 'Pre-deactivation physical UART integrity verification'

        $script = @'
set -eu
service_user=$1
service_group=$2
drop_in=/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf
latch=/var/lib/arm-gateway/arm-clear-required.json
fragment=$(sudo systemctl show arm-gateway.service --property=FragmentPath --value)
[ "$fragment" = /etc/systemd/system/arm-gateway.service ] || {
  echo "Refusing UART deactivation because the base service fragment is unexpected: $fragment" >&2
  exit 45
}
drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
[ "$drop_ins" = "$drop_in" ] || {
  echo "Refusing UART deactivation because the active drop-in set is unexpected: $drop_ins" >&2
  exit 46
}
sudo test -f "$drop_in" && ! sudo test -L "$drop_in" || {
  echo 'Refusing UART deactivation because the reviewed drop-in is missing or has the wrong type.' >&2
  exit 47
}
sudo test -f "$latch" && ! sudo test -L "$latch" || {
  echo 'Refusing UART deactivation because the fail-closed latch is missing or has the wrong type.' >&2
  exit 48
}
sudo python3 - "$service_user" "$service_group" <<'PY'
import grp
import json
import os
import pwd
import stat
import sys

service_user, service_group = sys.argv[1:]
target = "/var/lib/arm-gateway/arm-clear-required.json"
uid = pwd.getpwnam(service_user).pw_uid
gid = grp.getgrnam(service_group).gr_gid
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
if (
    not isinstance(document, dict)
    or document.get("clearRequired") is not True
    or document.get("version") != 1
):
    raise SystemExit("durable safety latch is not fail-closed")
PY
sudo systemctl stop arm-gateway.service
sudo rm -- "$drop_in"
sudo systemctl daemon-reload
sudo systemctl restart arm-gateway.service
attempt=0
while [ "$attempt" -lt 20 ]; do
  if curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz >/dev/null; then
    sudo systemctl is-active --quiet arm-gateway.service
    current_drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
    [ -z "$current_drop_ins" ] || {
      echo "Dormant restart retained unexpected service drop-ins: $current_drop_ins" >&2
      exit 49
    }
    effective_exec_start=$(sudo systemctl show arm-gateway.service --property=ExecStart --value)
    case "$effective_exec_start" in
      *'--arm-controller-port'*)
        echo 'Dormant restart still exposes a physical arm-controller port.' >&2
        exit 49
        ;;
    esac
    sudo test -f "$latch" && ! sudo test -L "$latch" || {
      echo 'Dormant restart did not retain the fail-closed latch.' >&2
      exit 49
    }
    echo 'Physical UART deactivation passed: reviewed drop-in removed, fail-closed latch retained, and dormant loopback gateway healthy.'
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 1
done
echo 'Dormant gateway did not become healthy after physical UART deactivation.' >&2
sudo systemctl --no-pager --full status arm-gateway.service >&2 || true
exit 4
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $script `
            -Operation 'Physical UART deactivation' `
            -Arguments @($UserName, $ServiceGroup)
        Write-Host (
            'Deactivation issued no STOP clear, calibration action, torque, motion, or camera capture. ' +
            'The reviewed physical UART drop-in was removed and the fail-closed latch was retained.'
        )
    }
    finally {
        if ($stagePrepared) {
            $cleanup = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-uart-[0-9a-f]*) ;; *) exit 2 ;; esac
sudo find "$stage" -depth -type f -delete 2>/dev/null || true
sudo find "$stage" -depth -type d -empty -delete 2>/dev/null || true
'@
            try {
                Invoke-RemoteScript `
                    -Context $Context `
                    -ScriptText $cleanup `
                    -Operation 'UART deactivation staging cleanup' `
                    -Arguments @($stage)
            }
            catch {
                Write-Warning $_.Exception.Message
            }
        }
        if (Test-Path -LiteralPath $digestContractPath) {
            Remove-Item -LiteralPath $digestContractPath -Force
        }
    }
}

function Invoke-Verify {
    param(
        [Parameter(Mandatory)] $Context,
        [Parameter(Mandatory)] [string] $ManifestPath
    )
    Assert-ReplacementSdBoot -Context $Context
    $stage = '/tmp/arm-pi-verify-' + [Guid]::NewGuid().ToString('N')
    $digestContractPath = New-RemoteDigestContract -Scope Verify -ManifestPath $ManifestPath
    $stagePrepared = $false
    try {
        $prepare = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-verify-[0-9a-f]*) ;; *) exit 2 ;; esac
umask 077
mkdir -- "$stage"
'@
        Invoke-RemoteScript -Context $Context -ScriptText $prepare -Operation 'Verification staging' -Arguments @($stage)
        $stagePrepared = $true
        Copy-ToRemoteStage -Context $Context -Source $digestContractPath -RemotePath "$stage/expected-digests.tsv"

        $script = @'
set -eu
direct_lan_address_cidr=$1
install_root=$2
report_unit() {
  unit=$1
  if systemctl cat "$unit" >/dev/null 2>&1; then
    enabled=$(systemctl is-enabled "$unit" 2>/dev/null || true)
    active=$(systemctl is-active "$unit" 2>/dev/null || true)
    [ -n "$enabled" ] || enabled=unknown
    [ -n "$active" ] || active=unknown
    echo "$unit|exists=yes|enabled=$enabled|active=$active"
  else
    echo "$unit|exists=no|enabled=absent|active=absent"
  fi
}
require_active_enabled_unit() {
  unit=$1
  systemctl cat "$unit" >/dev/null 2>&1 || {
    echo "Required service is absent: $unit" >&2
    exit 61
  }
  enabled=$(systemctl is-enabled "$unit" 2>/dev/null || true)
  active=$(systemctl is-active "$unit" 2>/dev/null || true)
  [ "$enabled" = enabled ] || {
    echo "Required service is not enabled: $unit ($enabled)" >&2
    exit 62
  }
  [ "$active" = active ] || {
    echo "Required service is not active: $unit ($active)" >&2
    exit 63
  }
}
require_installed_package() {
  package=$1
  status=$(dpkg-query -W -f='${db:Status-Status}' "$package" 2>/dev/null || true)
  version=$(dpkg-query -W -f='${Version}' "$package" 2>/dev/null || true)
  [ "$status" = installed ] && [ -n "$version" ] || {
    echo "Required Raspberry Pi OS package is not installed: $package" >&2
    exit 64
  }
  echo "$package|version=$version"
}
require_exec_token() {
  token=$1
  case "$effective_exec_start" in
    *"$token"*) ;;
    *) echo "Effective arm-gateway ExecStart is missing: $token" >&2; exit 65 ;;
  esac
}
echo '[devices]'
findmnt -n -o SOURCE,TARGET,FSTYPE /
findmnt -n -o SOURCE,TARGET,FSTYPE /boot/firmware
echo '[network]'
network_row=$(ip -4 -o address show dev eth0 scope global 2>/dev/null || true)
echo "$network_row"
printf '%s\n' "$network_row" | grep -Fq -- "$direct_lan_address_cidr" || {
  echo 'The requested direct-LAN address is absent from eth0.' >&2
  exit 66
}
echo '[uart]'
readlink -f /dev/serial0 2>/dev/null || true
echo '[services]'
require_active_enabled_unit ssh.service
require_active_enabled_unit arm-gateway.service
systemctl cat wayvnc.service >/dev/null 2>&1 || {
  echo 'Required service is absent: wayvnc.service' >&2
  exit 67
}
[ "$(systemctl is-enabled wayvnc.service 2>/dev/null || true)" = enabled ] || {
  echo 'Required service is not enabled: wayvnc.service' >&2
  exit 68
}
systemctl cat NetworkManager-wait-online.service >/dev/null 2>&1 || {
  echo 'Required service is absent: NetworkManager-wait-online.service' >&2
  exit 69
}
report_unit ssh.service
report_unit arm-gateway.service
report_unit wayvnc.service
report_unit NetworkManager-wait-online.service
echo '[drop-ins]'
fragment=$(sudo systemctl show arm-gateway.service --property=FragmentPath --value)
[ "$fragment" = /etc/systemd/system/arm-gateway.service ] || {
  echo "arm-gateway.service has an unexpected fragment: $fragment" >&2
  exit 70
}
drop_ins=$(sudo systemctl show arm-gateway.service --property=DropInPaths --value)
case "$drop_ins" in
  '') runtime_mode=dormant ;;
  '/etc/systemd/system/arm-gateway.service.d/20-arm-controller-uart.conf') runtime_mode=physical ;;
  *) echo "arm-gateway.service has unexpected drop-ins: $drop_ins" >&2; exit 71 ;;
esac
echo "runtimeMode=$runtime_mode|dropIns=$drop_ins"
echo '[effective-exec]'
effective_exec_start=$(sudo systemctl show arm-gateway.service --property=ExecStart --value)
require_exec_token "$install_root/.venv/bin/python"
require_exec_token '-m robot_gateway serve'
require_exec_token '--camera picamera2'
require_exec_token '--camera-profile module3-wide'
require_exec_token '--arm-state-dir /var/lib/arm-gateway'
require_exec_token "--token-file $install_root/.robot-gateway.token"
require_exec_token '--host 127.0.0.1 --port 8787'
case "$runtime_mode" in
  physical)
    require_exec_token '--arm-controller-port /dev/serial0'
    [ -n "$(readlink -f /dev/serial0 2>/dev/null || true)" ] || {
      echo 'Physical UART mode is installed but /dev/serial0 is unavailable.' >&2
      exit 72
    }
    ;;
  dormant)
    case "$effective_exec_start" in
      *'--arm-controller-port'*)
        echo 'Dormant mode unexpectedly exposes a physical arm-controller port.' >&2
        exit 73
        ;;
    esac
    ;;
esac
restarts=$(systemctl show arm-gateway.service --property=NRestarts --value)
case "$restarts" in
  ''|*[!0-9]*) echo "arm-gateway.service returned invalid NRestarts: $restarts" >&2; exit 74 ;;
esac
[ "$restarts" -eq 0 ] || {
  echo "arm-gateway.service has restarted unexpectedly: NRestarts=$restarts" >&2
  exit 75
}
echo "NRestarts=$restarts"
echo '[packages]'
for package in \
  python3 \
  python3-venv \
  python3-picamera2 \
  python3-libcamera \
  libcamera0.7 \
  rpicam-apps \
  wayvnc \
  openssh-server \
  network-manager \
  raspi-config \
  curl
do
  require_installed_package "$package"
done
echo '[python-imports]'
[ -x "$install_root/.venv/bin/python" ] || {
  echo 'The deployed arm-gateway virtual environment is absent.' >&2
  exit 76
}
"$install_root/.venv/bin/python" -c \
  'import fastapi, serial, uvicorn, picamera2, libcamera' || {
    echo 'A required gateway or Camera Module 3 Python import failed.' >&2
    exit 77
  }
echo 'fastapi|serial|uvicorn|picamera2|libcamera imports=ok'
echo '[health]'
curl --fail --silent --max-time 2 http://127.0.0.1:8787/healthz
'@
        Invoke-RemoteScript `
            -Context $Context `
            -ScriptText $script `
            -Operation 'Read-only recovery verification' `
            -Arguments @($DirectLanAddressCidr, $installRoot)
        Invoke-RemoteDigestCheck `
            -Context $Context `
            -Stage $stage `
            -Operation 'Read-only remote artifact integrity verification'
    }
    finally {
        if ($stagePrepared) {
            $cleanup = @'
set -eu
stage=$1
case "$stage" in /tmp/arm-pi-verify-[0-9a-f]*) ;; *) exit 2 ;; esac
sudo find "$stage" -depth -type f -delete 2>/dev/null || true
sudo find "$stage" -depth -type d -empty -delete 2>/dev/null || true
'@
            try {
                Invoke-RemoteScript -Context $Context -ScriptText $cleanup -Operation 'Verification staging cleanup' -Arguments @($stage)
            }
            catch {
                Write-Warning $_.Exception.Message
            }
        }
        if (Test-Path -LiteralPath $digestContractPath) {
            Remove-Item -LiteralPath $digestContractPath -Force
        }
    }
}

$inputs = Get-RecoveryInputs
if ($Phase -eq 'Prepare') {
    $manifestPath = New-RecoveryManifest -Inputs $inputs
    Write-Host "Prepared private recovery manifest: $manifestPath"
}
else {
    $manifestPath = Resolve-PreparedRecoveryManifest -Path (
        Join-Path $recoveryRoot 'LATEST.json'
    )
    Write-Host "Reusing exact prepared recovery manifest: $manifestPath"
}

if ($Phase -eq 'Prepare') {
    return
}

$context = New-SshContext
try {
    switch ($Phase) {
        'Bootstrap' {
            if ($PSCmdlet.ShouldProcess($HostName, 'Restore dormant arm gateway, token, calibration, and WayVNC ordering')) {
                Invoke-Bootstrap -Context $context -ManifestPath $manifestPath
            }
        }
        'ConfigureUart' {
            if ($PSCmdlet.ShouldProcess($HostName, 'Back up boot configuration and enable the Pi UART without rebooting')) {
                Invoke-ConfigureUart -Context $context
            }
        }
        'Activate' {
            if ($PSCmdlet.ShouldProcess($HostName, 'Install the reviewed physical UART drop-in and restart the gateway once')) {
                Invoke-Activate -Context $context -ManifestPath $manifestPath
            }
        }
        'Deactivate' {
            if ($PSCmdlet.ShouldProcess($HostName, 'Remove the reviewed physical UART drop-in and restart the dormant gateway once')) {
                Invoke-Deactivate -Context $context
            }
        }
        'Verify' {
            Invoke-Verify -Context $context -ManifestPath $manifestPath
        }
    }
}
finally {
    Remove-SshContext -Context $context
}
