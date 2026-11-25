@echo off
cd /d "%~dp0"
py -m pip install -r requirements.txt pyinstaller
py -m PyInstaller --noconfirm --clean --onefile --windowed --name IMU_GUI_V6 --add-data "assets;assets" --collect-all vtkmodules IMU_GUI.py
if errorlevel 1 (echo Build failed & pause & exit /b 1)
echo Created: dist\IMU_GUI_V6.exe
pause
