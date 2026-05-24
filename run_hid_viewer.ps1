# Activate venv and run HID bit viewer
$ErrorActionPreference = "Stop"

# Get the script's directory
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Activate the virtual environment
Write-Host "Activating virtual environment..."
& "$scriptDir\test\.venv\bin\Activate.ps1"

# Run the HID bit viewer
Write-Host "Starting HID bit viewer..." -ForegroundColor Cyan
python "$scriptDir\test\hid_bit_viewer.py"
