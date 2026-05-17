# API Comparison session: runs NoloClientAPI and hidAPI simultaneously, logs all data to a single CSV.
# Usage:  .\debug-session-api-compare.ps1 [-WsPortA 12345] [-WsPortB 12346] [-CsvOut <path>]
#                                          [-NoloServerPath <path>] [-NoBuild]
#
# Output CSV:  <repo>\test\compare_output.csv  (cleared on each run)
# Follow up:   python test\analyze_comparison.py test\compare_output.csv

param(
    [int]$WsPortA        = 12345,    # client-api WebSocket port (0 = no WS, csv-only)
    [int]$WsPortB        = 12346,    # hidAPI WebSocket port     (0 = no WS, csv-only)
    [string]$CsvOut      = '',
    [string]$NoloServerPath = '',
    [switch]$NoBuild
)

$Root          = Split-Path $PSScriptRoot -Parent
$ServerExe     = Join-Path $Root 'target\release\nolostream_server.exe'
$SdkLib64      = Join-Path $Root 'docs\reference\NoloDeviceSDK\NoloClient\lib64'
$DefaultNsPath = Join-Path $Root 'docs\reference\NoloDeviceSDK\NoloServer\NoloServer.exe'

if ($NoloServerPath -eq '') { $NoloServerPath = $DefaultNsPath }
if ($CsvOut         -eq '') { $CsvOut         = Join-Path $Root 'test\compare_output.csv' }

$TempClientCsv  = Join-Path $env:TEMP "nolo_compare_client_api.csv"
$TempHidCsv     = Join-Path $env:TEMP "nolo_compare_hid.csv"
$NoloLog        = Join-Path $env:TEMP "nolo_compare_noloserver.log"
$ServerClientLog= Join-Path $env:TEMP "nolo_compare_server_client.log"
$ServerHidLog   = Join-Path $env:TEMP "nolo_compare_server_hid.log"

function Write-Label {
    param([string]$Label, [string]$Msg, [ConsoleColor]$Color = 'White')
    Write-Host "[$Label] " -NoNewline -ForegroundColor $Color
    Write-Host $Msg
}

# ----- kill stale processes ---------------------------------------------------
foreach ($name in 'nolostream_server', 'NoloServer') {
    $procs = Get-Process -Name $name -ErrorAction SilentlyContinue
    if ($procs) {
        Write-Label 'kill' "stopping existing $name" Yellow
        $procs | Stop-Process -Force
    }
}

# ----- copy DLLs --------------------------------------------------------------
$ReleaseDir = Join-Path $Root 'target\release'
if (-not (Test-Path $ReleaseDir)) { New-Item -ItemType Directory -Path $ReleaseDir | Out-Null }
foreach ($dll in @('NoloClientLib.dll', 'libzmq-64.dll')) {
    $src = Join-Path $SdkLib64 $dll
    $dst = Join-Path $ReleaseDir $dll
    if (Test-Path $src) {
        Copy-Item -Path $src -Destination $dst -Force
        Write-Label 'copy' "$dll -> target\release\" Cyan
    } else {
        Write-Label 'warn' "$src not found -- DLL may be missing" Yellow
    }
}

# ----- build ------------------------------------------------------------------
if (-not $NoBuild) {
    Write-Label 'build' 'cargo build --release' Cyan
    Push-Location $Root
    cargo build --release
    $rc = $LASTEXITCODE
    Pop-Location
    if ($rc -ne 0) { Write-Label 'error' "build failed (exit $rc)" Red; exit $rc }
}

if (-not (Test-Path $ServerExe)) {
    Write-Label 'error' "binary not found: $ServerExe  (run without -NoBuild)" Red; exit 1
}
if (-not (Test-Path $NoloServerPath)) {
    Write-Label 'error' "NoloServer.exe not found: $NoloServerPath" Red; exit 1
}

# ----- clear output CSV (requirement: cleared on each run) --------------------
'' | Set-Content $CsvOut
'' | Set-Content $TempClientCsv
'' | Set-Content $TempHidCsv
Write-Label 'csv' "Output: $CsvOut (cleared)" Cyan

# ----- start NoloServer -------------------------------------------------------
'' | Set-Content $NoloLog
Write-Label 'start' "NoloServer.exe" Green
$NoloProc = Start-Process -FilePath $NoloServerPath `
    -WorkingDirectory (Split-Path $NoloServerPath) `
    -RedirectStandardError $NoloLog -NoNewWindow -PassThru

Write-Label 'wait' 'giving NoloServer 2 s to initialise...' Yellow
Start-Sleep -Seconds 2
if ($NoloProc.HasExited) {
    Write-Label 'error' "NoloServer.exe exited early (code $($NoloProc.ExitCode))" Red; exit 1
}

# ----- start nolostream_server --client-api with CSV log ----------------------
'' | Set-Content $ServerClientLog
$ClientArgs = @('--client-api', '--csv-log', $TempClientCsv)
if ($WsPortA -gt 0) { $ClientArgs += '--ws-listen-at', $WsPortA }
Write-Label 'start' "nolostream_server --client-api --csv-log $TempClientCsv" Green
$ClientProc = Start-Process -FilePath $ServerExe `
    -ArgumentList $ClientArgs `
    -RedirectStandardError $ServerClientLog `
    -NoNewWindow -PassThru

Start-Sleep -Milliseconds 1000

# ----- start nolostream_server (HID) with CSV log ----------------------------
'' | Set-Content $ServerHidLog
$HidArgs = @('--csv-log', $TempHidCsv)
if ($WsPortB -gt 0) { $HidArgs += '--ws-listen-at', $WsPortB }
Write-Label 'start' "nolostream_server (hid) --csv-log $TempHidCsv" Green
$HidProc = Start-Process -FilePath $ServerExe `
    -ArgumentList $HidArgs `
    -RedirectStandardError $ServerHidLog `
    -NoNewWindow -PassThru

Write-Label 'info' "Both APIs running. Press Ctrl-C to stop and merge CSV." Yellow
Write-Host ''

# ----- tail logs --------------------------------------------------------------
$noloJob   = Start-Job { Get-Content -Wait $using:NoloLog        | % { "[nolo  ] $_" } }
$clientJob = Start-Job { Get-Content -Wait $using:ServerClientLog | % { "[capi  ] $_" } }
$hidJob    = Start-Job { Get-Content -Wait $using:ServerHidLog    | % { "[hid   ] $_" } }

try {
    while ($true) {
        Receive-Job $noloJob, $clientJob, $hidJob | Write-Host
        if ($ClientProc.HasExited) {
            Write-Label 'warn' "client-api server exited (code $($ClientProc.ExitCode))" Yellow
            break
        }
        if ($HidProc.HasExited) {
            Write-Label 'warn' "hid server exited (code $($HidProc.ExitCode))" Yellow
            break
        }
        Start-Sleep -Milliseconds 200
    }
} finally {
    Write-Host ''
    Write-Label 'stop' 'stopping processes...' Yellow
    Stop-Job   $noloJob, $clientJob, $hidJob -ErrorAction SilentlyContinue
    Remove-Job $noloJob, $clientJob, $hidJob -ErrorAction SilentlyContinue
    foreach ($proc in @($NoloProc, $ClientProc, $HidProc)) {
        if ($proc -and -not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }
    }

    # ----- merge both temp CSVs into single output CSV ------------------------
    Write-Label 'merge' "merging CSVs -> $CsvOut" Cyan

    $clientRows = if (Test-Path $TempClientCsv) { Get-Content $TempClientCsv } else { @() }
    $hidRows    = if (Test-Path $TempHidCsv)    { Get-Content $TempHidCsv    } else { @() }

    if ($clientRows.Count -gt 0 -or $hidRows.Count -gt 0) {
        # Write header from whichever file has one
        $header = if ($clientRows.Count -gt 0) { $clientRows[0] } else { $hidRows[0] }
        $header | Set-Content $CsvOut

        # Append data rows from client_api (skip header)
        if ($clientRows.Count -gt 1) {
            $clientRows[1..($clientRows.Count - 1)] |
                Where-Object { $_ -match '\S' } |
                Add-Content $CsvOut
        }

        # Append data rows from hid (skip header)
        if ($hidRows.Count -gt 1) {
            $hidRows[1..($hidRows.Count - 1)] |
                Where-Object { $_ -match '\S' } |
                Add-Content $CsvOut
        }

        $totalRows = (Get-Content $CsvOut).Count - 1
        Write-Label 'done' "$totalRows data rows written to: $CsvOut" Green
        Write-Label 'next' "Run:  python test\analyze_comparison.py `"$CsvOut`"" Cyan
    } else {
        Write-Label 'warn' 'No data collected -- both temp CSV files are empty.' Yellow
    }
}
