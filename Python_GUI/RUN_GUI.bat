@echo off
cd /d "%~dp0"
py -m pip install -r requirements.txt
if errorlevel 1 (pause & exit /b 1)
py IMU_GUI_STM32_VTK_PRELOAD.py
if errorlevel 1 pause
