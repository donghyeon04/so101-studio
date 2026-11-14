@echo off
rem Runs so101_studio in a conda env, from any folder (no conda activate needed).
rem Env name: SO101_ENV (default so101). Or give the env folder directly with SO101_ENV_DIR.
setlocal
if not defined SO101_ENV set "SO101_ENV=so101"
if not defined SO101_ENV_DIR (
  for %%R in ("C:\so101-tools\miniforge3" "C:\miniforge3" "%USERPROFILE%\miniforge3" "%USERPROFILE%\miniconda3" "%USERPROFILE%\anaconda3" "%LOCALAPPDATA%\miniforge3" "%ProgramData%\miniforge3" "%ProgramData%\miniconda3") do (
    if not defined SO101_ENV_DIR if exist "%%~R\envs\%SO101_ENV%\python.exe" set "SO101_ENV_DIR=%%~R\envs\%SO101_ENV%"
  )
)
if not defined SO101_ENV_DIR (
  echo conda env "%SO101_ENV%" not found. Create it: conda env create -f environment.yml  ^(or set SO101_ENV / SO101_ENV_DIR^)
  exit /b 1
)
set "PATH=%SO101_ENV_DIR%\Library\bin;%SO101_ENV_DIR%;%SO101_ENV_DIR%\Scripts;%PATH%"
cd /d "%~dp0"
rem Run this folder's code even if the editable install points at another folder (worktrees share one conda env).
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"
"%SO101_ENV_DIR%\python.exe" -m so101_studio %*
