<#
.SYNOPSIS
  Quarantine broken Ghidra Java scripts that prevent the script-directory bundle from compiling.

.DESCRIPTION
  Ghidra's Java script provider compiles the whole script directory as a bundle. One
  stale, Python-formatted, duplicate, or API-incompatible .java file can prevent an
  unrelated script such as TeslaFullCflashS19ExporterV16.java from loading. This
  repair script moves the known failing files reported by Ghidra into a timestamped
  backup directory, optionally copies the known-good exporter into place, and leaves
  the C:\Users\<user>\ghidra_scripts directory buildable.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\RepairGhidraScriptBundle.ps1 `
    -ScriptDir "$env:USERPROFILE\ghidra_scripts" `
    -ExporterSource .\TeslaFullCflashS19ExporterV16.java

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\RepairGhidraScriptBundle.ps1 -WhatIf
#>
[CmdletBinding(SupportsShouldProcess = $true)]
param(
  [string] $ScriptDir = (Join-Path $env:USERPROFILE "ghidra_scripts"),
  [string] $ExporterSource = "",
  [string] $BackupRoot = ""
)

$ErrorActionPreference = "Stop"

function Move-ToBackup {
  param(
    [Parameter(Mandatory = $true)] [string] $Path,
    [Parameter(Mandatory = $true)] [string] $BackupDir,
    [Parameter(Mandatory = $true)] [string] $Reason
  )

  if (-not (Test-Path -LiteralPath $Path)) {
    return
  }

  $destination = Join-Path $BackupDir (Split-Path -Leaf $Path)
  $suffix = 1
  while (Test-Path -LiteralPath $destination) {
    $destination = Join-Path $BackupDir ("{0}.{1}" -f (Split-Path -Leaf $Path), $suffix)
    $suffix++
  }

  if ($PSCmdlet.ShouldProcess($Path, "move to $destination ($Reason)")) {
    Move-Item -LiteralPath $Path -Destination $destination
    Write-Host ("Moved {0} -> {1} [{2}]" -f $Path, $destination, $Reason)
  }
}

function Get-PublicClassName {
  param([Parameter(Mandatory = $true)] [string] $Path)

  $match = Select-String -LiteralPath $Path -Pattern '^\s*public\s+class\s+([A-Za-z_$][A-Za-z0-9_$]*)' -Encoding UTF8 -ErrorAction SilentlyContinue | Select-Object -First 1
  if ($null -eq $match) {
    return ""
  }
  return $match.Matches[0].Groups[1].Value
}

if (-not (Test-Path -LiteralPath $ScriptDir -PathType Container)) {
  throw "Ghidra script directory does not exist: $ScriptDir"
}

if ([string]::IsNullOrWhiteSpace($BackupRoot)) {
  $BackupRoot = Join-Path $ScriptDir "_disabled_compile_failures"
}

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backupDir = Join-Path $BackupRoot $timestamp
if ($PSCmdlet.ShouldProcess($backupDir, "create backup directory")) {
  New-Item -ItemType Directory -Force -Path $backupDir | Out-Null
}

# Files named in the reported Ghidra bundle failures. These are quarantined so
# the exporter can compile/load without the entire script bundle failing first.
$knownBroken = @(
  "TrasnsportAPublicationV53Script.java",
  "TeslaFlexCanExactPathAndId398TraceV41.java",
  "TeslaBootVerifierTrace.java",
  "TeslaInternalDatFileCrcProofV26.java",
  "TeslaFlexCanExactPathAndId398TraceV42.java",
  "TeslaFullCflashS19ExporterV16-old.java",
  "TeslaAutopilotRuntimeCopySplitPlannerV46.java",
  "TeslaInternalDatVLERepairAndEntryStubAnalysis.java"
)

foreach ($fileName in $knownBroken) {
  Move-ToBackup -Path (Join-Path $ScriptDir $fileName) -BackupDir $backupDir -Reason "reported Ghidra Java compile failure"
}

# Also quarantine any remaining .java script that is actually Python/Markdown text,
# has a UTF-8 BOM before the Java source, or declares a public class whose name does
# not match the file name. These are common causes of bundle-wide script load failure.
Get-ChildItem -LiteralPath $ScriptDir -Filter "*.java" -File | ForEach-Object {
  $path = $_.FullName
  $fileName = $_.Name
  if ($fileName -eq "TeslaFullCflashS19ExporterV16.java") {
    return
  }

  $bytes = [System.IO.File]::ReadAllBytes($path)
  $firstLine = (Get-Content -LiteralPath $path -TotalCount 1 -ErrorAction SilentlyContinue)
  if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
    Move-ToBackup -Path $path -BackupDir $backupDir -Reason "UTF-8 BOM before Java source"
    return
  }
  if ($firstLine -match '^\s*#') {
    Move-ToBackup -Path $path -BackupDir $backupDir -Reason "Python/Markdown content saved with .java extension"
    return
  }

  $publicClass = Get-PublicClassName -Path $path
  $expectedClass = [System.IO.Path]::GetFileNameWithoutExtension($fileName)
  if ($publicClass -ne "" -and $publicClass -ne $expectedClass) {
    Move-ToBackup -Path $path -BackupDir $backupDir -Reason ("public class {0} does not match file name {1}" -f $publicClass, $expectedClass)
  }
}

if (-not [string]::IsNullOrWhiteSpace($ExporterSource)) {
  if (-not (Test-Path -LiteralPath $ExporterSource -PathType Leaf)) {
    throw "ExporterSource does not exist: $ExporterSource"
  }
  $target = Join-Path $ScriptDir "TeslaFullCflashS19ExporterV16.java"
  if ($PSCmdlet.ShouldProcess($target, "copy known-good exporter from $ExporterSource")) {
    Copy-Item -LiteralPath $ExporterSource -Destination $target -Force
    Write-Host ("Copied exporter to {0}" -f $target)
  }
}

Write-Host "Repair complete. Restart Ghidra or refresh Script Manager so the Java bundle rebuilds."
Write-Host ("Disabled scripts backup: {0}" -f $backupDir)
