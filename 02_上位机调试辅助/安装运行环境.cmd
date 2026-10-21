@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" py -3.11 -m venv ".venv"
if not exist ".venv\Scripts\python.exe" python -m venv ".venv"
if not exist ".venv\Scripts\python.exe" (
  echo 请先安装Python 3.11，再运行本文件。
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m pip install --disable-pip-version-check --no-cache-dir -r requirements.txt
if errorlevel 1 (
  echo 安装失败，请保留错误信息。
) else (
  echo 运行环境准备完成，可以双击启动上位机。
)
pause
