@echo off
powershell.exe -ExecutionPolicy Bypass -File "%~dp0Resources/launch_initial_system.ps1"
pause

wsl.exe --shutdown