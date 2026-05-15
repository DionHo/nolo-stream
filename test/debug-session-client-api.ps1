# Debug session script using NoloClientLib API (Windows PowerShell)
# Usage:  .\debug-session-client-api.ps1 [-WsPort 12345] [-NoloServerPath <path>] [-NoBuild]
#
# Starts NoloServer.exe, copies required DLLs, then launches nolostream_server
# with --client-api and miniviz, tailing combined labelled output.
# Ctrl-C stops all processes.

param(
    [int]$WsPort = 12345,
    [string]$NoloServerPath = '',
    [switch]$NoBuild
)

$Root          = Split-Path $PSScriptRoot -Parent
$ServerExe     = Join-Path $Root 'target\release\nolostream_server.exe'
$MinivizExe    = Join-Path $Root 'target\release\miniviz.exe'
$ServerLog     = Join-Path $env:TEMP "nolo_server_client_${WsPort}.log"
$MinivizLog    = Join-Path $env:TEMP "nolo_miniviz_client_${WsPort}.log"
$NoloLog       = Join-Path $env:TEMP "nolo_noloserver_${WsPort}.log"

$SdkLib64      = Join-Path $Root 'docs\reference\NoloDeviceSDK\NoloClient\lib64'
$DefaultNsPath = Join-Path $Root 'docs\reference\NoloDeviceSDK\NoloServer\NoloServer.exe'

if ($NoloServerPath -eq '') {
    $NoloServerPath = $DefaultNsPath
}

function Write-Label {
    param([string]$Label, [string]$Msg, [ConsoleColor]$Color = 'White')
    Write-Host "[$Label] " -NoNewline -ForegroundColor $Color
    Write-Host $Msg
}

# ----- kill any stale processes -----------------------------------------------
foreach ($name in 'nolostream_server', 'miniviz', 'NoloServer') {
    $procs = Get-Process -Name $name -ErrorAction SilentlyContinue
    if ($procs) {
        Write-Label 'kill' "stopping existing $name process(es)" Yellow
        $procs | Stop-Process -Force
    }
}

# ----- copy DLLs next to the server binary ------------------------------------
$ReleaseDir = Join-Path $Root 'target\release'
if (-not (Test-Path $ReleaseDir)) {
    New-Item -ItemType Directory -Path $ReleaseDir | Out-Null
}

foreach ($dll in @('NoloClientLib.dll', 'libzmq-64.dll')) {
    $src = Join-Path $SdkLib64 $dll
    $dst = Join-Path $ReleaseDir $dll
    if (Test-Path $src) {
        Copy-Item -Path $src -Destination $dst -Force
        Write-Label 'copy' "$dll -> target\release\" Cyan
    } else {
        Write-Label 'warn' "$src not found -- DLL may be missing at runtime" Yellow
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

foreach ($exe in @($ServerExe, $MinivizExe)) {
    if (-not (Test-Path $exe)) {
        Write-Label 'error' "binary not found: $exe  (run without -NoBuild)" Red
        exit 1
    }
}

if (-not (Test-Path $NoloServerPath)) {
    Write-Label 'error' "NoloServer.exe not found: $NoloServerPath" Red
    exit 1
}

# ----- start NoloServer -------------------------------------------------------
'' | Set-Content $NoloLog
Write-Label 'start' "NoloServer.exe -- $NoloServerPath" Green
$NoloProc = Start-Process -FilePath $NoloServerPath `
    -WorkingDirectory (Split-Path $NoloServerPath) `
    -RedirectStandardError $NoloLog `
    -NoNewWindow -PassThru

Write-Label 'wait' 'giving NoloServer 2 s to initialise...' Yellow
Start-Sleep -Seconds 2

if ($NoloProc.HasExited) {
    Write-Label 'error' "NoloServer.exe exited early (code $($NoloProc.ExitCode))" Red
    exit 1
}

# ----- start nolostream_server with --client-api ------------------------------
'' | Set-Content $ServerLog
'' | Set-Content $MinivizLog

Write-Label 'start' "nolostream_server --client-api --ws-listen-at $WsPort --debug" Green
$ServerProc = Start-Process -FilePath $ServerExe `
    -ArgumentList '--client-api', '--ws-listen-at', $WsPort, '--debug' `
    -RedirectStandardError $ServerLog `
    -NoNewWindow -PassThru

Start-Sleep -Milliseconds 800

Write-Label 'start' "miniviz --connect ws://127.0.0.1:$WsPort" Green
$MinivizProc = Start-Process -FilePath $MinivizExe `
    -ArgumentList '--connect', "ws://127.0.0.1:${WsPort}" `
    -RedirectStandardError $MinivizLog `
    -NoNewWindow -PassThru

Write-Label 'logs' "tailing logs -- Ctrl-C to stop" Yellow
Write-Host ''

# ----- tail logs with labels --------------------------------------------------
$noloJob = Start-Job -ScriptBlock {
    Get-Content -Wait -Path $using:NoloLog |
        ForEach-Object { "[nolo   ] $_" }
}
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
        Receive-Job $noloJob, $serverJob, $minivizJob | Write-Host
        if ($ServerProc.HasExited) {
            Write-Label 'warn' "server exited (code $($ServerProc.ExitCode))" Yellow
            break
        }
        Start-Sleep -Milliseconds 100
    }
} finally {
    Write-Host ''
    Write-Label 'stop' 'killing processes...' Yellow
    Stop-Job   $noloJob, $serverJob, $minivizJob -ErrorAction SilentlyContinue
    Remove-Job $noloJob, $serverJob, $minivizJob -ErrorAction SilentlyContinue
    foreach ($proc in @($NoloProc, $ServerProc, $MinivizProc)) {
        if ($proc -and -not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }
    }
    Write-Label 'done' "logs: $NoloLog | $ServerLog | $MinivizLog" Cyan
}
