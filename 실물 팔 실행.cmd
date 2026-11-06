@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"
echo 실물 팔로 GUI를 켭니다.
echo 팔은 클램프로 고정하고, 처음에는 속도 30%% 이하로, 전원 스위치 옆에서 지켜보세요. 주변 반경 50cm를 비웁니다.
echo 계속하려면 아무 키나 누르세요.
pause >nul
rem 켤 때마다 최신 코드로 (git 폴더일 때만, 실패해도 계속)
set "GIT=git"
where git >nul 2>&1
if errorlevel 1 (
  set "GIT="
  for %%R in ("C:\so101-tools\miniforge3" "C:\miniforge3" "%USERPROFILE%\miniforge3") do if not defined GIT if exist "%%~R\Library\cmd\git.exe" set "GIT=%%~R\Library\cmd\git.exe"
)
if exist ".git" if defined GIT (
  "%GIT%" pull --ff-only || echo [경고] 업데이트하지 못했습니다. 지금 있는 버전으로 계속합니다.
) else if exist ".git" echo [경고] git이 없어 업데이트를 건너뜁니다.
call "%~dp0so101.cmd" gui
if errorlevel 1 (
  echo.
  echo [오류] 실행하지 못했습니다. 먼저 "설치.cmd"를 실행했는지 확인하세요.
  pause
)
