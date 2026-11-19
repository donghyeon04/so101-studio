# so101-studio

카메라 없이 저가 초음파 센서로 물체의 위치·크기·방향을 인식하고, SO-101(SO-ARM101) 로봇팔이 그 물체를 집어 옮기는 시스템이다. 같은 작업을 카메라 방식과 비교해 각 센서가 유리한 환경을 밝히고, 전체를 웹 GUI로 제어·모니터링한다.

*Pick-and-place with an SO-101 arm using two low-cost ultrasonic sensors instead of a camera, with a web GUI, a kinematic and a MuJoCo physics simulator. Windows, Python 3.12. Docs are in Korean.*

## 지금 상태 (2026-11-19, v0.1)
- 시뮬레이션에서는 처음부터 끝까지 돈다: 초음파 인식, 작업 실행(가까운 순으로 집어 배출 통에 놓기), GUI. 물리 시뮬레이션(MuJoCo)은 시험장·학습 데이터 수집·정책 평가까지 되고, 시뮬레이션 시계로 실시간의 약 10배 빠르게 돈다.
- 하드웨어 담당 CAD를 반영했다: 판 425×375mm(40×80 프로파일 위), 새 J1 받침, 오른쪽 앞 귀퉁이의 배출 통, 레이더 받침(XL330 + OpenRB + HC-SR04)의 자리와 형상([`decisions/0067`](docs/decisions/0067-workbench-cad-values.md)). 판 높이·팔 원점은 CAD 값이라 실물 조립 뒤 실측으로 고친다.
- 병렬(로보나인) 손은 높이를 알면 기울여 집을 수 있다(`grasp_approach: auto`, 기본은 수직).
- 실물 팔로는 11-13까지 전원·자동 캘리브레이션·집기·놓기를 확인했다(옛 받침, 58mm 상자 5회 왕복 30%·50%에서 10/10). 새 작업대·레이더 조립과 실물 재확인은 다음 단계다.
- 학습(ACT 모방학습, 위·손목 카메라 + 관절)은 시뮬레이션에서 처음 보는 배치 50번 중 15~17번을 집어 놓는다(기준 15/50, [`decisions/0071`](docs/decisions/0071-act-second-collection.md)). 학습 씨앗에 따라 7~17/50로 흔들린다. GUI 정책 실행 화면에서 학습 배치를 놓고 돌려 볼 수 있다.
- 다음은 초음파 정확도다: 레이더 조립 → 실측 → 보정([`decisions/0072`](docs/decisions/0072-ultrasonic-accuracy-first.md)). 카메라 학습은 카메라를 산 뒤 한다.
- 현황 한 장은 [`docs/planning/project-status.md`](docs/planning/project-status.md), 진행 중인 일과 사용자 결정은 [`docs/planning/work-queue.md`](docs/planning/work-queue.md)에 있다.

## 무엇을 하나
| 부분 | 내용 |
|---|---|
| 초음파 인식 | 서보가 돌리는 HC-SR04 레이더로 후보를 찾는다. 팔 손목의 센서로 지나가며 위치·폭·방향을 확정한다(`CHECK`). 후보가 없으면 다시 찾는다(`RESCAN`). |
| 팔 | 관절·직교 조그, 좌표 이동, 물체 방향에 맞춘 집기와 대칭으로 벌리는 놓기, 교시(손으로 움직여 기록·재생). 자동 캘리브레이션은 측정·비교·저장과 0° 확인까지 한다. |
| 안전 | 관절 한계, 작업대 가장자리, 바닥 검사. E-STOP 감시 프로세스(heartbeat·단축키 F9). 과열·과부하 감시. 캘리브레이션이 맞지 않으면 움직이는 명령을 잠근다. |
| GUI | 브라우저에서 여는 화면 17개(작업 공간, 직접 조작, 교시, 프로그램, 초음파, 수동 입력, 팔, 캘리브레이션, 파라미터, 안전, 작업대, 시점, 정책 실행 등). 3D 보기, 운영자 화면, 같은 망의 다른 기기에서 보기 전용(`--share`). |
| 시뮬레이션 | `--sim`(기구학)으로 팔 없이 모든 기능을 돌린다. `--sim --sim-engine mujoco`(물리)는 턱이 물체 윗면을 치거나 미끄러지는 접촉을 보인다. |
| 학습 | MuJoCo 시험장(`mujoco_bench`)으로 스크립트 시연을 LeRobot 데이터셋으로 모으고, ACT 정책을 학습해 시뮬레이션에서 평가·실행한다(GUI 정책 실행 화면). |
| 기록 | 실행마다 `runs/날짜-시각/`에 `events.jsonl`·`status.jsonl`(전압 포함)을 남기고, 사고 묶음(`incident`)을 만든다. |

## 빠른 시작 (Windows)
1. 레포를 받는다.
   - `git clone https://github.com/donghyeon04/so101-studio`를 권한다(켤 때마다 자동으로 최신이 된다).
   - git이 없으면 GitHub의 Code → Download ZIP을 풀어도 된다. 그러면 자동 업데이트가 없어 새 버전은 ZIP을 다시 받아야 한다.
2. `설치.cmd`를 더블클릭한다.
   - conda가 없으면 Miniforge를 `C:\so101-tools\miniforge3`에 설치한다(관리자 권한 필요 없음, 한글 사용자 폴더 문제를 피하려고 영문 경로).
   - git이 없으면 받는다.
   - `so101` 환경을 만든 뒤 점검(`doctor`)까지 보여 준다.
   - MuJoCo(물리 시뮬레이션)도 설치할지 `Y/n`으로 묻는다.
   - 바탕화면과 시작 메뉴에 "SO-101 Studio" 바로가기를 만든다.
   - 처음 설치에 약 3GB를 받는다. 다시 실행하면 환경을 갱신한다.
3. 바탕화면의 "SO-101 Studio"(또는 `SO-101 Studio 실행.cmd`)를 더블클릭한다. 브라우저에서 `http://127.0.0.1:8080`으로 GUI가 열린다.
   - 실물 팔이 없으면 GUI가 시뮬레이션으로 켜고 화면에 알린다.
   - 실물 팔을 쓰려면 USB 시리얼 드라이버(CH340)를 먼저 설치한다([`docs/guides/arm-hardware-setup.md`](docs/guides/arm-hardware-setup.md)).
   - 캘리브레이션이 다르면 팔이 잠겨 읽기만 된다.
4. 지우려면 `제거.cmd`를 실행한다.
   - 지울 목록을 보여 주고, `yes`를 입력해야 진행한다.
   - `so101` 환경과 바로가기를 지운다.
   - Miniforge는 `설치.cmd`가 직접 깐 경우에만 한 번 더 묻고 지운다.
   - 레포 폴더(`runs/`·`config/local.yaml` 포함)와 LeRobot 캘리브레이션 캐시는 지우지 않으므로 직접 지운다.

`SO-101 Studio 실행.cmd`는 켤 때마다 `git pull --ff-only`를 하고(실패해도 지금 버전으로 계속), `so101.cmd gui`를 부른다. `so101.cmd`는 `C:\so101-tools\miniforge3`, `C:\miniforge3`, 사용자 폴더의 miniforge·miniconda에서 `so101` 환경을 찾는다(근거 [`decisions/0040`](docs/decisions/0040-team-local-install.md)).

v0.1의 배포는 위 배치 파일 셋(`설치.cmd`, `SO-101 Studio 실행.cmd`, `제거.cmd`)이다. 설치 프로그램(Setup.exe, Windows 앱 목록에서 제거)은 최종 배포 때 만든다.

직접 설치할 때는 `conda env create -f environment.yml` 뒤 `conda activate so101`을 한다. numpy는 conda 설치본을 쓰므로 `pip`로 다시 설치하지 않는다. MuJoCo는 선택이고, 설치 줄은 [`docs/guides/tooling-setup.md`](docs/guides/tooling-setup.md) C24에 있다.

## 명령줄
어느 폴더에서든 `so101.cmd [--sim] [--speed N] <명령>`으로 쓴다(원래 형태는 `python -m so101_studio ...`).

| 명령 | 하는 일 |
|---|---|
| `gui` | GUI 앱 |
| `doctor` | 읽기 전용 점검: 환경·포트·캘리브레이션·서보 응답·전압·온도. 팔을 움직이지 않는다. |
| `status`, `home`, `park`, `movej`, `goto`, `gripper` | 상태 읽기와 기본 동작 |
| `pick x y 각도 폭 [높이]`, `place` | 집기·놓기 |
| `run --source manual\|ultrasonic` | 자동 작업(인식 → 가까운 순으로 집기 → 배출 구역) |
| `keys` | 키보드 조그(관절 / 직교 / 좌표 입력) |
| `record`, `replay`, `info` | 교시 기록·재생 |
| `calibrate --auto --measure-only`, `--save`, `--verify-zero` | 자동 캘리브레이션 |
| `powertest`, `shaketest`, `servoinfo`, `linkwatch` | 진단 |
| `incident` | 지금까지의 기록으로 사고 묶음 생성 |

MuJoCo 시험장(물리 시뮬레이션, `python -m so101_studio.arm.mujoco_bench`):

| 명령 | 하는 일 |
|---|---|
| `--suite pick_grid --clock step --jobs 12 --viewer grid` | 고정 묶음 집기 시험(결과 표는 `runs/bench-mujoco/`). `--clock step`은 시뮬레이션 시계(빠르고 같은 씨앗이면 같은 결과), `--viewer grid`는 일꾼 화면을 창 하나에 |
| `--suite collect_pick1 --record <이름> --seeds 300` | 스크립트 시연을 LeRobot 데이터셋(`runs/datasets/<이름>/`)으로 |
| `--suite collect_pick1 --policy <체크포인트> --seed-start 10000 --seeds 50` | 학습한 ACT 정책 평가 |
| `python -m so101_studio.arm.mujoco_record --shift\|--clip <데이터셋>` | 데이터셋 사본(action 앞당김, 관절 한계 안쪽으로 자르기) |

전체 설명은 [`CLAUDE.md`](CLAUDE.md)의 "실행" 절에 있다.

## 하드웨어
- 팔: SO-ARM101 Follower(STS3215 서보 6개, 12V), 버스 서보 보드.
- 초음파: ESP32(`firmware/esp32_scanner`)가 HC-SR04 두 개를 읽는다. 하나는 서보가 돌리는 레이더 스테이션(OpenRB-150 + XL330, `firmware/openrb_servo`)에, 하나는 팔 손목에 단다.
- 카메라(비교 실험용): AI-Thinker ESP32-CAM 1대([`decisions/0058`](docs/decisions/0058-camera-esp32-cam.md)).
- 작업대: 판 425×375mm, 40×80 알루미늄 프로파일 두 개 위. 팔은 프로파일에 꽂는 J1 받침, 레이더는 앞 프로파일 위(결정 0067).
- 그리퍼: 로보나인 병렬 그리퍼(결정 0063), 손목에 초음파 센서 브래킷.

부품과 비용은 [`docs/planning/hardware.md`](docs/planning/hardware.md)에 있다. HC-SR04는 5V, ESP32는 3.3V라 Echo 핀은 분압 저항이나 레벨 시프터를 거쳐 연결한다.

## 안전
- 팔은 클램프로 고정하고, 주변 반경 50cm를 비운다.
- 처음 움직일 때는 속도 30% 이하로 하고, 사람이 전원 스위치 옆에서 지켜본다.
- 진단 명령(`powertest`, `shaketest`)은 팔을 크게 움직인다.
- 실물 절차는 [`docs/guides/real-arm-runbook.md`](docs/guides/real-arm-runbook.md)를 따른다.

## 구조
```
앱        CLI · GUI(NiceGUI + Viser 3D)          감시 프로세스(E-STOP)
서비스    Studio = ArmService + 작업 실행 + 인식 관리
모듈      arm(기구학·궤적·집기/놓기)   perception(초음파·수동 입력)   task
공통      core(타입·좌표·설정·이벤트·기록)
드라이버  실물 서보 / SimDriver(기구학) / MuJoCoDriver(물리)
```
- `arm`과 `perception`은 서로 모르고, 둘을 잇는 것은 작업 실행뿐이다.
- 앱(CLI, GUI)은 서비스만 부른다.
- 자세한 것은 [`docs/planning/system-architecture.md`](docs/planning/system-architecture.md)와 모듈 설계서(`arm-software-spec.md`, `mujoco-spec.md`, `ultrasonic-spec.md`, `gui-spec.md`)에 있다.

## 시험
```
python -m pytest -q -m "not slow and not measure"            # 개발 중(빠른 묶음)
python -m pytest -q -n auto -m "not measure" --ignore=tests/gui_visual   # 전체(약 12분, xdist)
python -m pytest -q tests/gui_visual                          # GUI 화면 비교(보드를 뽑고)
python -m ruff check src tests
```
GUI 시험은 고정 포트를 쓰므로 한 번에 하나만 돌린다.

## 문서
- 기획서: [`docs/planning/overview.md`](docs/planning/overview.md)
- 문서 색인: [`docs/README.md`](docs/README.md)
- 결정 기록: [`docs/decisions/`](docs/decisions/)

## 레퍼런스와 라이선스
이 프로젝트는 MIT 라이선스다([`LICENSE`](LICENSE)). 가져온 것은 원래 라이선스를 따른다.
- [LeRobot](https://github.com/huggingface/lerobot): 서보 버스·캘리브레이션·기구학(Apache-2.0)
- [SO-ARM100/101](https://github.com/TheRobotStudio/SO-ARM100): 팔 URDF·메쉬(Apache-2.0)
- [mujoco_menagerie `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie): MuJoCo 모델(Apache-2.0, [`assets/mujoco/so101/SOURCE.md`](assets/mujoco/so101/SOURCE.md))
- [Robonine 병렬 그리퍼](https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper): 메쉬(Apache-2.0 / CERN-OHL-P-2.0, [`assets/assets/parallel/NOTICE.txt`](assets/assets/parallel/NOTICE.txt))
- [WeGo-Robotics/wego-piper-web](https://github.com/WeGo-Robotics/wego-piper-web): GUI 메뉴 구성·E-stop 감시 프로세스 분리 참고

팀: 송민준(하드웨어), 김동현(소프트웨어), 이민우(지원). 2026 자동차센서공학 과목 프로젝트.
