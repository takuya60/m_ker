@echo off
setlocal

cd /d "%~dp0"
set "VENV=.build_venv"

if not exist "%VENV%\Scripts\python.exe" (
    py -3 -m venv "%VENV%"
    if errorlevel 1 exit /b 1
)

call "%VENV%\Scripts\activate.bat"
python -m pip install --upgrade pip
if errorlevel 1 exit /b 1
python -m pip install -r requirements-windows.txt
if errorlevel 1 exit /b 1

if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

python -m PyInstaller ^
    --noconfirm ^
    --clean ^
    --windowed ^
    --onedir ^
    --name encoder_tool ^
    --collect-all flet_desktop ^
    --hidden-import serial.tools.list_ports_windows ^
    --add-data "encoder_tool\hex;hex" ^
    encoder_tool.py
if errorlevel 1 exit /b 1

python -m PyInstaller ^
    --noconfirm ^
    --clean ^
    --console ^
    --onefile ^
    --name ker_flash_cli ^
    --distpath "dist\encoder_tool" ^
    --collect-all pymcuprog ^
    --collect-all pyedbglib ^
    --collect-all esptool ^
    --hidden-import serial.tools.list_ports_windows ^
    encoder_flash_cli.py
if errorlevel 1 exit /b 1

echo.
echo Build complete: %CD%\dist\encoder_tool
echo Copy the entire encoder_tool directory to the target Windows computer.
endlocal
