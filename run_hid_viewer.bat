@echo off
REM Activate venv and run HID bit viewer

echo Activating virtual environment...
call "%~dp0test\.venv\bin\activate.bat"

echo Starting HID bit viewer...
python "%~dp0test\hid_bit_viewer.py"
