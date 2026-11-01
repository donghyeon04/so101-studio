@echo off
rem Runs so101_studio with the so101 conda env, from any folder (no conda activate needed).
set "ENVDIR=%USERPROFILE%\miniforge3\envs\so101"
set "PATH=%ENVDIR%\Library\bin;%ENVDIR%;%ENVDIR%\Scripts;%PATH%"
cd /d "%~dp0"
"%ENVDIR%\python.exe" -m so101_studio %*
