@echo off
cd /d "%~dp0"
if exist "C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe" (
    "C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe" serve.py
) else (
    python serve.py
)
pause
