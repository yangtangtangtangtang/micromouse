@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo 未找到运行环境，请先运行 安装运行环境.cmd。
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -B verify.py
pause
