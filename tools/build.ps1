param([string]$KeilRoot = $env:KEIL_ROOT)
$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'check_environment.ps1') -KeilRoot $KeilRoot
$repoRoot = Split-Path $PSScriptRoot -Parent
$registeredArm = (Get-ItemProperty 'HKLM:\SOFTWARE\WOW6432Node\Keil\Products\MDK' -ErrorAction SilentlyContinue).Path
$registeredRoot = if ($registeredArm) { Split-Path $registeredArm.TrimEnd('\') -Parent }
$roots = @($KeilRoot, $registeredRoot, 'C:\Keil_v5', 'D:\Keil_v5') | Where-Object { $_ }
$found = $roots | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'UV4/UV4.exe') } | Select-Object -First 1
$uv4 = Join-Path $found 'UV4/UV4.exe'
$projectPath = Join-Path $repoRoot 'MDK-ARM/CtrBoard.uvprojx'
$logPath = Join-Path $repoRoot 'MDK-ARM/build_check.log'
# Start-Process waits for the GUI-subsystem compiler process without opening a window.
$process = Start-Process -FilePath $uv4 -ArgumentList @('-r', ('"' + $projectPath + '"'), '-t', 'CtrBoard', '-o', ('"' + $logPath + '"')) -WindowStyle Hidden -Wait -PassThru
if (Test-Path -LiteralPath $logPath) { Get-Content -LiteralPath $logPath }
# uVision: 0 = success; 1 = warnings; 2+ = errors or failed invocation.
if ($process.ExitCode -ge 2) { throw "Keil build failed with exit code $($process.ExitCode). See $logPath" }
if ($process.ExitCode -eq 1) { Write-Warning 'Build completed with warnings; review the log.' }
Write-Host "Build complete: $(Join-Path $repoRoot 'MDK-ARM/CtrBoard/CtrBoard.hex')"
