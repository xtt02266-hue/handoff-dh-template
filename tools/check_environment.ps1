param([string]$KeilRoot = $env:KEIL_ROOT)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$projectPath = Join-Path $repoRoot 'MDK-ARM/CtrBoard.uvprojx'
[xml]$project = Get-Content -LiteralPath $projectPath -Raw
$missing = @()
foreach ($node in $project.SelectNodes('//FilePath')) {
    $path = Join-Path (Split-Path $projectPath -Parent) $node.InnerText
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { $missing += $node.InnerText }
}
foreach ($node in $project.SelectNodes('//IncludePath')) {
    foreach ($relative in ($node.InnerText -split ';')) {
        if ($relative -and -not (Test-Path -LiteralPath (Join-Path (Split-Path $projectPath -Parent) $relative) -PathType Container)) { $missing += $relative }
    }
}
if ($missing.Count) { throw "Missing project paths: $($missing -join ', ')" }
Write-Host 'OK: all Keil source, library and include paths exist.'
$target = $project.Project.Targets.Target
Write-Host "Target: $($target.TargetName); compiler: $($target.pCCUsed); pack: $($target.TargetOption.TargetCommonOption.PackID)"
$registeredArm = (Get-ItemProperty 'HKLM:\SOFTWARE\WOW6432Node\Keil\Products\MDK' -ErrorAction SilentlyContinue).Path
$registeredRoot = if ($registeredArm) { Split-Path $registeredArm.TrimEnd('\') -Parent }
$roots = @($KeilRoot, $registeredRoot, 'C:\Keil_v5', 'D:\Keil_v5') | Where-Object { $_ }
$found = $roots | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'UV4/UV4.exe') } | Select-Object -First 1
if (-not $found) { throw 'Keil not found. Install MDK or pass -KeilRoot / set KEIL_ROOT.' }
$compiler = Join-Path $found 'ARM/ARMCLANG/bin/armclang.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw "Arm Compiler not found: $compiler" }
Write-Host "Keil: $found"
$versionText = & $compiler --version
if ($LASTEXITCODE -ne 0) { throw 'Unable to query Arm Compiler version.' }
$versionText | Write-Host
if (($versionText -join '\n') -notmatch '6\.21\b') { throw 'This project requires Arm Compiler 6.21. Install/select the matching version in Keil.' }
$toolsIni = Join-Path $found 'TOOLS.INI'
$configuredPackRoot = if (Test-Path -LiteralPath $toolsIni) {
    $match = Select-String -LiteralPath $toolsIni -Pattern '^RTEPATH="?([^"\r\n]+)"?' | Select-Object -First 1
    if ($match) { $match.Matches[0].Groups[1].Value.Trim() }
}
$packRoots = @($env:CMSIS_PACK_ROOT, $configuredPackRoot, (Join-Path $env:LOCALAPPDATA 'Arm/Packs'), (Join-Path $found 'Packs'), (Join-Path $found 'ARM/PACK')) | Where-Object { $_ }
$pack = $packRoots | ForEach-Object { Join-Path $_ 'Keil/STM32F4xx_DFP/3.1.1' } | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $pack) { throw 'Pack 3.1.1 not found in common locations. Install it with Pack Installer, or set CMSIS_PACK_ROOT to your pack root.' }
Write-Host "Pack: $pack"
Write-Host 'Environment check complete. Rebuild in Keil to verify compilation; hardware is not checked.'
