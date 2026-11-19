@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"
echo.
echo === SO-101 Studio 설치 ===
echo.

rem 1) conda 찾기. 없으면 Miniforge를 영문 경로에 무인 설치한다 (한글 사용자 폴더는 펌웨어 링크 등이 실패한다, tooling-setup C6)
set "TOOLS=C:\so101-tools"
set "MF=%TOOLS%\miniforge3"
set "ROOT="
for %%R in ("%MF%" "C:\miniforge3" "%USERPROFILE%\miniforge3" "%USERPROFILE%\miniconda3" "%USERPROFILE%\anaconda3" "%LOCALAPPDATA%\miniforge3" "%ProgramData%\miniforge3" "%ProgramData%\miniconda3") do (
  if not defined ROOT if exist "%%~R\condabin\conda.bat" set "ROOT=%%~R"
)
if defined ROOT goto have_conda

echo conda가 없어 Miniforge를 설치합니다 (약 100MB 다운로드, 관리자 권한 필요 없음).
if not exist "%TOOLS%" mkdir "%TOOLS%"
if not exist "%TOOLS%" (
  echo [오류] %TOOLS% 폴더를 만들 수 없습니다. C: 드라이브 쓰기 권한을 확인하세요.
  goto fail
)
set "INSTALLER=%TOOLS%\Miniforge3-Windows-x86_64.exe"
curl -L --fail -o "%INSTALLER%" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86_64.exe
if errorlevel 1 (
  echo [오류] Miniforge를 받지 못했습니다. 인터넷 연결을 확인하세요.
  goto fail
)
start /wait "" "%INSTALLER%" /InstallationType=JustMe /RegisterPython=0 /S /D=%MF%
if not exist "%MF%\condabin\conda.bat" (
  echo [오류] Miniforge 설치가 끝나지 않았습니다. %INSTALLER% 를 직접 실행해 보세요.
  goto fail
)
set "ROOT=%MF%"
del "%INSTALLER%" >nul 2>&1
rem 제거.cmd가 이 표시 파일이 있을 때만 Miniforge를 지울지 묻는다
> "%TOOLS%\installed-by-so101.txt" echo 설치.cmd가 Miniforge를 %MF%에 설치했습니다. 제거.cmd가 이 파일을 보고 지울지 묻습니다.

:have_conda
echo conda 위치: %ROOT%

rem 2) git (켤 때마다 자동 업데이트에 쓴다). 없으면 conda-forge로 base 환경에 받는다
where git >nul 2>&1
if errorlevel 1 if not exist "%ROOT%\Library\cmd\git.exe" (
  echo git이 없어 설치합니다.
  call "%ROOT%\condabin\conda.bat" install -y -n base -c conda-forge git
  if errorlevel 1 echo [경고] git 설치에 실패했습니다. 자동 업데이트 없이 계속합니다.
)

rem 3) so101 환경 만들기 또는 갱신 (처음에는 약 3GB를 받는다)
if exist "%ROOT%\envs\so101\python.exe" (
  echo so101 환경을 갱신합니다.
  call "%ROOT%\condabin\conda.bat" env update -n so101 -f environment.yml --prune
) else (
  echo so101 환경을 만듭니다. 처음에는 약 3GB를 받아 시간이 걸립니다.
  call "%ROOT%\condabin\conda.bat" env create -f environment.yml
)
if errorlevel 1 (
  echo [오류] 환경을 만들지 못했습니다. 위 메시지를 확인하세요.
  goto fail
)

rem 4) MuJoCo(물리 시뮬레이션, 선택). tooling-setup C24 줄, numpy가 바뀌지 않게 --no-deps
set "ENVDIR=%ROOT%\envs\so101"
set "ANS="
set /p "ANS=MuJoCo(물리 시뮬레이션)도 설치할까요? [Y/n] "
if /i "%ANS%"=="n" goto skip_mujoco
set "PATH=%ENVDIR%\Library\bin;%ENVDIR%;%ENVDIR%\Scripts;%PATH%"
python -m pip install --no-deps mujoco==4.0.0 mujoco-core==4.0.0 mujoco-studio==4.0.0 glfw==2.11.0 pyopengl==3.1.10
if errorlevel 1 echo [경고] MuJoCo를 설치하지 못했습니다. 기구학 시뮬레이션은 그대로 됩니다.
set "NPV="
for /f "delims=" %%v in ('python -c "import numpy; print(numpy.__version__)"') do set "NPV=%%v"
if "%NPV%"=="2.2.6" (echo numpy 2.2.6 확인) else echo [경고] numpy가 2.2.6이 아닙니다: %NPV%. docs\guides\tooling-setup.md C24를 보세요.
:skip_mujoco

rem 5) 바로가기 "SO-101 Studio" (바탕화면·시작 메뉴, 작업 폴더는 레포). 제거.cmd가 같은 이름으로 지운다
set "SO101_LNK_TARGET=%~dp0SO-101 Studio 실행.cmd"
set "SO101_LNK_DIR=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$s=New-Object -ComObject WScript.Shell; foreach($f in 'Desktop','Programs'){ $l=$s.CreateShortcut((Join-Path ([Environment]::GetFolderPath($f)) 'SO-101 Studio.lnk')); $l.TargetPath=$env:SO101_LNK_TARGET; $l.WorkingDirectory=$env:SO101_LNK_DIR; $l.Save() }"
if errorlevel 1 echo [경고] 바로가기를 만들지 못했습니다. "SO-101 Studio 실행.cmd"를 직접 더블클릭하세요.

rem 6) 점검 (읽기 전용, 팔을 움직이지 않는다)
echo.
echo === 점검 ===
call "%~dp0so101.cmd" doctor --no-probe
echo.
echo 설치가 끝났습니다. 바탕화면의 "SO-101 Studio"(또는 이 폴더의 "SO-101 Studio 실행.cmd")를 더블클릭하세요.
pause
exit /b 0

:fail
echo.
echo 설치를 마치지 못했습니다.
pause
exit /b 1
