@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"
rem 설치.cmd가 만든 것을 지운다: conda 환경, 바로가기, (설치.cmd가 직접 깔았을 때만) Miniforge. 레포 폴더와 기록·설정은 두고 안내만 한다.
if not defined SO101_ENV set "SO101_ENV=so101"
set "TOOLS=C:\so101-tools"
set "MF=%TOOLS%\miniforge3"
set "MARK=%TOOLS%\installed-by-so101.txt"
set "ROOT="
for %%R in ("%MF%" "C:\miniforge3" "%USERPROFILE%\miniforge3" "%USERPROFILE%\miniconda3" "%USERPROFILE%\anaconda3" "%LOCALAPPDATA%\miniforge3" "%ProgramData%\miniforge3" "%ProgramData%\miniconda3") do (
  if not defined ROOT if exist "%%~R\condabin\conda.bat" set "ROOT=%%~R"
)
set "ENVDIR="
if defined ROOT if exist "%ROOT%\envs\%SO101_ENV%\python.exe" set "ENVDIR=%ROOT%\envs\%SO101_ENV%"

echo.
echo === SO-101 Studio 제거 ===
echo.
echo 지우는 것:
if defined ENVDIR echo   - conda 환경 "%SO101_ENV%": %ENVDIR%
if not defined ENVDIR echo   - conda 환경 "%SO101_ENV%": 찾지 못해 건너뜁니다
echo   - 바탕화면과 시작 메뉴의 "SO-101 Studio" 바로가기
if exist "%MARK%" echo   - Miniforge %MF%: 설치.cmd가 깔았으므로 따로 한 번 더 묻습니다
if not exist "%MARK%" echo   - Miniforge: 설치.cmd가 깐 것이 아니어서 지우지 않습니다
echo.
echo 지우지 않는 것:
echo   - 이 레포 폴더: %~dp0
echo   - 그 안의 runs\ 실행 기록과 config\local.yaml 이 PC 설정
echo   - LeRobot 캘리브레이션 캐시: %USERPROFILE%\.cache\huggingface\lerobot\calibration
echo.
set "ANS="
set /p "ANS=계속하려면 yes를 입력하세요: "
if /i not "%ANS%"=="yes" (
  echo 취소했습니다. 아무것도 지우지 않았습니다.
  goto end
)

if not defined ENVDIR goto env_done
echo conda 환경 "%SO101_ENV%"을 지웁니다.
call "%ROOT%\condabin\conda.bat" env remove -y -n "%SO101_ENV%"
if errorlevel 1 echo [경고] 환경을 지우지 못했습니다. GUI나 so101 명령이 켜져 있으면 끄고 다시 실행하세요.
:env_done

powershell -NoProfile -ExecutionPolicy Bypass -Command "foreach($f in 'Desktop','Programs'){ $p=Join-Path ([Environment]::GetFolderPath($f)) 'SO-101 Studio.lnk'; if(Test-Path $p){ Remove-Item $p; Write-Host ('  removed: ' + $p) } }"

if not exist "%MARK%" goto mf_done
set "ANS="
set /p "ANS=Miniforge(%MF%)도 지울까요? 다른 conda 환경도 함께 사라집니다. yes/no: "
if /i not "%ANS%"=="yes" (
  echo Miniforge는 남겨 둡니다.
  goto mf_done
)
if exist "%MF%\Uninstall-Miniforge3.exe" start /wait "" "%MF%\Uninstall-Miniforge3.exe" /S _?=%MF%
if exist "%MF%" rmdir /s /q "%MF%"
if exist "%MF%" (
  echo [경고] %MF% 를 다 지우지 못했습니다. 직접 지우세요.
  goto mf_done
)
del "%MARK%" >nul 2>&1
rd "%TOOLS%" >nul 2>&1
echo Miniforge를 지웠습니다.
:mf_done

echo.
echo 끝났습니다. 이 폴더는 직접 지우세요: %~dp0
echo 실행 기록 runs\ 와 config\local.yaml 이 그 안에 있으니 필요하면 먼저 옮기세요.
echo LeRobot 캘리브레이션 캐시도 필요 없으면 직접 지우세요: %USERPROFILE%\.cache\huggingface\lerobot\calibration
:end
pause
