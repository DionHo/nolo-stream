# Debug session script for NoloStream (Windows PowerShell)
# Usage:  .\debug-session.ps1 [-WsPort 12345] [-NoBuild]
#
# Kills any stale server/miniviz, starts fresh, and prints combined labelled output.
# Ctrl-C stops both processes and prints the log paths.

param(
    [int]$WsPort = 12345,
    [switch]$NoBuild
)

$Root       = Split-Path $PSScriptRoot -Parent
$ServerExe  = Join-Path $Root 'target\release\nolostream_server.exe'
$MinivizExe = Join-Path $Root 'target\release\miniviz.exe'
$ServerLog  = Join-Path $env:TEMP "nolo_server_${WsPort}.log"
$MinivizLog = Join-Path $env:TEMP "nolo_miniviz_${WsPort}.log"

function Write-Label {
    param([string]$Label, [string]$Msg, [ConsoleColor]$Color = 'White')
    Write-Host "[$Label] " -NoNewline -ForegroundColor $Color
    Write-Host $Msg
}

# ----- kill any stale processes -----------------------------------------------
foreach ($name in 'nolostream_server', 'miniviz') {
    $procs = Get-Process -Name $name -ErrorAction SilentlyContinue
    if ($procs) {
        Write-Label 'kill' "stopping existing $name process(es)" Yellow
        $procs | Stop-Process -Force
    }
}

# ----- build ------------------------------------------------------------------
if (-not $NoBuild) {
    Write-Label 'build' 'cargo build --release' Cyan
    Push-Location $Root
    cargo build --release
    $rc = $LASTEXITCODE
    Pop-Location
    if ($rc -ne 0) {
        Write-Label 'error' "build failed (exit $rc)" Red
        exit $rc
    }
}

foreach ($exe in $ServerExe, $MinivizExe) {
    if (-not (Test-Path $exe)) {
        Write-Label 'error' "binary not found: $exe  (run without -NoBuild)" Red
        exit 1
    }
}

# ----- start processes --------------------------------------------------------
'' | Set-Content $ServerLog
'' | Set-Content $MinivizLog

Write-Label 'start' "nolostream_server --ws-listen-at $WsPort --debug" Green
$ServerProc = Start-Process -FilePath $ServerExe `
    -ArgumentList '--ws-listen-at', $WsPort, '--debug' `
    -RedirectStandardError $ServerLog `
    -NoNewWindow -PassThru

Start-Sleep -Milliseconds 800

Write-Label 'start' "miniviz --connect ws://127.0.0.1:$WsPort" Green
$MinivizProc = Start-Process -FilePath $MinivizExe `
    -ArgumentList '--connect', "ws://127.0.0.1:${WsPort}" `
    -RedirectStandardError $MinivizLog `
    -NoNewWindow -PassThru

Write-Label 'logs ' "tailing $ServerLog and $MinivizLog -- Ctrl-C to stop" Yellow
Write-Host ''

# ----- tail both logs with labels ---------------------------------------------
$serverJob = Start-Job -ScriptBlock {
    Get-Content -Wait -Path $using:ServerLog |
        ForEach-Object { "[server ] $_" }
}
$minivizJob = Start-Job -ScriptBlock {
    Get-Content -Wait -Path $using:MinivizLog |
        ForEach-Object { "[miniviz] $_" }
}

try {
    while ($true) {
        Receive-Job $serverJob, $minivizJob | Write-Host
        if ($ServerProc.HasExited) {
            Write-Label 'warn' "server exited (code $($ServerProc.ExitCode))" Yellow
            break
        }
        Start-Sleep -Milliseconds 100
    }
}
finally {
    Write-Host ''
    Write-Label 'stop' 'killing processes...' Yellow
    Stop-Job   $serverJob,  $minivizJob  -ErrorAction SilentlyContinue
    Remove-Job $serverJob,  $minivizJob  -ErrorAction SilentlyContinue
    if (-not $ServerProc.HasExited) {
        Stop-Process -Id $ServerProc.Id -Force -ErrorAction SilentlyContinue
    }
    if ($MinivizProc -and -not $MinivizProc.HasExited) {
        Stop-Process -Id $MinivizProc.Id -Force -ErrorAction SilentlyContinue
    }
    Write-Label 'done' "logs saved to $ServerLog and $MinivizLog" Cyan
}
