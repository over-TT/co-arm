#Requires -Version 5.1

<#
.SYNOPSIS
Compiles the ESP32 Arm HAT firmware with a system Arduino CLI.

.DESCRIPTION
Builds only the ESP32 Arm HAT sketch. The caller supplies an absolute build
directory outside the co-arm repository, keeping generated binaries outside
the source tree. This script never uploads or flashes a device.
#>

[CmdletBinding()]
param(
    [string] $ArduinoCli = 'arduino-cli',

    [string] $ArduinoConfig,

    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [string] $BuildRoot,

    [ValidatePattern('^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$')]
    [string] $RequiredEsp32CoreVersion = '3.3.11'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$softwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $softwareRoot '..')).Path
$firmwareRoot = Join-Path $softwareRoot 'firmware'
$libraries = Join-Path $firmwareRoot 'libraries'
$armHatSketch = Join-Path $firmwareRoot 'esp32\arm_hat_controller'

if (-not [System.IO.Path]::IsPathRooted($BuildRoot)) {
    throw 'BuildRoot must be an explicit absolute path outside the co-arm repository.'
}
$resolvedBuildRoot = [System.IO.Path]::GetFullPath($BuildRoot)
$normalizedRepositoryRoot = [System.IO.Path]::GetFullPath($repositoryRoot).TrimEnd('\', '/')
$repositoryPrefix = $normalizedRepositoryRoot + [System.IO.Path]::DirectorySeparatorChar
$buildRootIsRepository = $resolvedBuildRoot.TrimEnd('\', '/').Equals(
    $normalizedRepositoryRoot,
    [System.StringComparison]::OrdinalIgnoreCase
)
$buildRootIsInsideRepository = $resolvedBuildRoot.StartsWith(
    $repositoryPrefix,
    [System.StringComparison]::OrdinalIgnoreCase
)
if ($buildRootIsRepository -or $buildRootIsInsideRepository) {
    throw 'BuildRoot must be outside the co-arm repository.'
}

$cliPath = (Get-Command $ArduinoCli -CommandType Application -ErrorAction Stop).Source

foreach ($path in @($libraries, $armHatSketch)) {
    if (-not (Test-Path -LiteralPath $path -PathType Container)) {
        throw "Required firmware source directory is missing: $path"
    }
}
New-Item -ItemType Directory -Force -Path $resolvedBuildRoot | Out-Null

$sharedArguments = @()
if (-not [string]::IsNullOrWhiteSpace($ArduinoConfig)) {
    $configPath = (Resolve-Path -LiteralPath $ArduinoConfig -ErrorAction Stop).Path
    $sharedArguments += @('--config-file', $configPath)
}

$coreJson = & $cliPath @sharedArguments core list --format json
if ($LASTEXITCODE -ne 0) {
    throw 'Could not inspect installed Arduino cores.'
}
$cores = $coreJson | ConvertFrom-Json
$esp32Core = @($cores | Where-Object { [string]$_.ID -eq 'esp32:esp32' })
if ($esp32Core.Count -ne 1 -or
    [string]$esp32Core[0].Installed -cne $RequiredEsp32CoreVersion) {
    throw (
        "Expected exactly esp32:esp32 $RequiredEsp32CoreVersion. " +
        'Install that core explicitly before compiling.'
    )
}

$targets = @(
    @{
        Name = 'arm-hat-controller'
        Fqbn = 'esp32:esp32:esp32'
        Sketch = $armHatSketch
        BuildProperties = @()
    }
)

foreach ($target in $targets) {
    $targetBuildRoot = Join-Path $resolvedBuildRoot $target.Name
    New-Item -ItemType Directory -Force -Path $targetBuildRoot | Out-Null
    $arguments = @($sharedArguments) + @(
        'compile'
        '--fqbn', $target.Fqbn
        '--libraries', $libraries
        '--build-path', $targetBuildRoot
        '--warnings', 'all'
    )
    foreach ($property in $target.BuildProperties) {
        $arguments += @('--build-property', $property)
    }
    $arguments += $target.Sketch
    & $cliPath @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware compile failed for $($target.Name)."
    }
}

Write-Host "ARM firmware compilation passed. Build output: $resolvedBuildRoot"
