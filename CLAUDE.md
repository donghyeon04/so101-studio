# so101-studio

SO-101(SO-ARM101) 로봇 암 프로젝트. 문서 색인은 `docs/README.md`에 있다.

## 작업 원칙
- 사용자와 다른 세션에게 쓰는 답·보고·질문, PR 본문, 커밋 메시지는 한국어로, 결론부터 짧게 쓴다. 코드·명령·식별자는 원래 표기 그대로 둔다.
- 기능 구현, 문제 해결, 기술·도구 선택은 처음부터 설계하지 않고 오픈소스·레퍼런스를 가져와 고친다. 결정하기 전에 `researching-references` skill로 조사하고, 사용자 승인을 받은 뒤 진행한다.
- 판단 요청("어떻게 할까", "고민이야")에는 바로 답하지 않고 질문을 한 번에 하나씩 던진다(소크라테스 모드). "정리해줘"나 "결론 내줘"라고 하면 그때 정리한다.
- 사용자가 `/grill-me`를 부르면 여러 질문을 추천 답과 함께 라운드로 묻는다. 레포의 `.claude/skills/`에 mattpocock/skills 11개가 이 프로젝트에 맞게 고쳐져 있다 (`decisions/0032`).
- 실행 요청("만들어줘", "작성해줘")은 바로 실행한다.
- 검증용 계산기·시뮬레이터는 주제마다 하나만, 레포 안 실제 코드 위에 둔다. 새 아이디어는 그 도구의 옵션·시나리오로 붙이고, 다른 세션(기획 포함)이 같은 주제의 계산기를 따로 만들지 않는다. 설계서의 성능 숫자는 그 도구의 고정 시나리오 결과만 쓴다 (`decisions/0037`).

## 세션 구분
- 원격 레포 세션 = 기획: 기획, 아키텍처, 레퍼런스 조사, 문서 작성. 프로젝트 코드는 쓰지 않는다.
- 로컬 세션 = 개발: 코드 작성, 시리얼 포트·ESP32·서보 테스트, 펌웨어 업로드. 하드웨어 작업은 로컬에서만 할 수 있다.
- 로컬은 세 세션으로 나눈다 (`decisions/0024`, `0036`). GUI 세션(총괄)은 `src/so101_studio/gui/`, `tests/gui/`만 고치고 `planning/gui-spec.md`를 따르며, 일 나누기·요청 대장·PR 검토를 맡는다. 초음파 세션은 `perception/ultrasonic/`, `firmware/`, `tests/perception/`, `config/ultrasonic.yaml`만 고치고 `planning/ultrasonic-spec.md`를 따른다. 팔 세션은 그 밖의 코드를 고치고 `planning/arm-software-spec.md`를 따른다. 상대 영역이 필요하면 직접 고치지 않고 요청서(REQ)를 보낸다. 역할, worktree, 요청 양식은 `docs/guides/session-workflow.md`, 요청 기록은 `docs/planning/session-requests.md`.
- 개발 세션은 `docs/planning/`의 확정 문서와 `docs/decisions/`를 근거로 작업한다.
- 다른 계정에서 작업을 시작하면 `docs/guides/tooling-setup.md`의 설치 목록부터 맞춘다.
- 이어서 작업할 때(압축·세션 교체 뒤 포함)는 `docs/planning/work-queue.md`(진행 중인 큰 지시와 사용자 결정 모음)부터 읽는다. 실물 팔 작업은 `docs/guides/real-arm-runbook.md`를 따른다.

## 개발 세션 규칙
- 작업 전에 `docs/planning/system-architecture.md`와 해당 모듈 설계서(`arm-software-spec.md` 등)를 읽고, 작업 순서표의 다음 작업부터 한다.
- `main`에서 `dev/<작업>` 브랜치를 따서 작업하고, 끝나면 PR로 합친다 (`decisions/0007`).
- 설계서와 다르게 구현해야 하면 멈추고 사용자에게 알린다. 설계서는 기획 세션에서 고친다.
- 하드웨어 없이 `--sim`으로 먼저 테스트한다. 실제 팔은 속도 30% 이하로 처음 움직이고, 사람이 전원 스위치 옆에서 대기한다.
- 측정값은 `docs/planning/measurements.md`에 기록한다.
- 환경: conda env `so101`(Python 3.12). 먼저 `conda activate so101`로 활성화한다. 활성화 없이 env의 python.exe를 바로 부르면 numpy 행렬곱에서 아무 메시지 없이 종료(exit 127)된다(MKL DLL 경로). Git Bash에서는 `export PATH="<miniforge>/envs/so101/Library/bin:<miniforge>/envs/so101:$PATH"`(`<miniforge>`는 PC마다 다르다: 노트북 `/c/miniforge3`, 예전 데스크톱 `/c/Users/c/miniforge3`).
- numpy는 conda 설치본(MKL, placo와 맞춤)이다. `pip install`로 numpy를 다시 설치하지 않는다.
- 테스트: `python -m pytest -q` (레포 루트에서, 약 45분). 개발 중에는 `python -m pytest -q -m "not slow"`, PR은 main과 합쳐 관련 테스트만, 전체는 하루 마칠 때·실물 시험 전에 총괄이 한 번(`decisions/0046`). 린트: `python -m ruff check src tests`.
- 검증(결정 0045): 파일을 고치면 `.claude/hooks/ruff_check.py` 훅이 그 파일만 ruff로 검사한다. GUI 테스트는 고정 포트를 써서 한 번에 한 세션만 돌린다(팔·초음파 세션은 `--ignore=tests/gui`, 총괄이 합칠 때 전체). 총괄은 PR을 합치기 전에 `code-review`(medium 이상)를 돌려 결과를 작성 세션에 보내고, GUI를 바꾼 PR은 Playwright 스크린샷으로 화면을 확인한다. 끝났다·통과했다는 보고에는 실행한 명령과 결과(통과 수, 실패한 테스트 이름)를 함께 적는다.
- 설계서 점검(결정 0048): 설계서 새 절은 구현 전에 새로 연 세션에서 `/egm:egm-review`를 Stage 9만 돌리고, 큰 합치기가 몰린 날 마칠 때 그날 설계서 하나에 `/egm:egm-drift-report`를 돌린다. EGM 스킬은 `disable-model-invocation`이라 **사용자가 슬래시 명령을 직접 입력해야** 한다. 세션은 스스로 부르거나 하위 에이전트로 흉내 내지 않고, 붙여 넣을 명령 문장(대상 절, GoldfishContext)만 준비한다. 결과의 꼭 고칠 것만 [기획 요청]으로 올린다.
  EGM artifact directory: runs/egm
- 실행(어느 폴더에서든, conda 활성화 없이): `so101.cmd [--sim] [--speed N] <명령>` (레포 루트의 런처가 so101 conda 환경을 직접 지정한다. PowerShell에서 레포 폴더 안이면 `.\so101 ...`). 원래 형태는 `python -m so101_studio ...`. 모든 명령은 `service/arm_service.py`의 `ArmService`를 거치고 `runs/날짜-시각/`에 `events.jsonl`, `status.csv`(전압 포함)를 남긴다.
  - 동작: `status|home|park|movej|goto|pick x y 각도 폭 [높이]|place|gripper`(폭·높이 0이면 모름), 자동 작업 `run --source manual [--step] [--dry-run]`(`config/manual_objects.yaml`의 물체를 가까운 순으로 집어 `workspace.yaml`의 배출 구역으로 옮긴다). `--source ultrasonic [--us-port COMx]`면 초음파: 후보마다 `CHECK`(팔이 센서로 지나가 확정), 후보가 없으면 `RESCAN`, 놓는 동안 레이더 `prefetch`, 집기 실패는 `note_failure`, 실패가 쌓이면 `give_up`으로 끝낸다(`ultrasonic-spec.md` 2절). `--sim`이면 가상 ESP32가 벤치마크 demo 배치(반경 120~220mm, 간격 120mm 이상)의 물체를 놓는다. 대기 자세는 `poses.radar_clear`(결정 0039·0041)이고 첫 사이클·`RESCAN`·집기 실패 뒤에만 간다(`PLACE` 뒤에는 미리 돌린 레이더 결과로 바로 `SCAN`). 실제 포트는 기준값(`config/ultrasonic_baseline.json`)이 먼저 있어야 한다.
  - `doctor [--json] [--no-probe] [--port P] [--arm 이름]`: 읽기 전용 점검(환경·패키지·포트·팔 파일·캘리브레이션 대조·서보 응답·전압·온도, `decisions/0027`). 서보 버스에는 PING/READ만 보내고 `connect`를 쓰지 않으며 움직이지 않는다. `--sim`이나 `--no-probe`면 포트를 열지 않고 목록만 본다. lerobot 없이도 돈다. `find-port`: USB를 뽑아 사라진 포트를 찾는다(LeRobot 방식).
  - `calibrate --auto --measure-only [--velocity 200] [--torque-limit 500]`: SO-101 자동 캘리브레이션을 서보 EEPROM에 쓰지 않고 측정만 해 기존 `config/calibration/<id>.json`과 관절별 범위·0° 위치 차이를 표로 비교하고 `runs/*-autocal/measure.json`에 남긴다(`decisions/0028`, LeRobot PR #4826 엔진). 전압 11.0V 미만·온도 60°C 초과면 중단하고 되돌린다. 기존 파일과 ±3° 이내면 1회 합격, 아니면 최근 24시간 같은 팔 측정 3개가 서로 1° 이내일 때 반복 합격이고(`반복 n/3` 표시, `decisions/0029`), `calibrate --save`가 합격한 결과(반복이면 관절별 중앙값, 기존 파일이 있으면 차이표와 `yes` 입력)를 `.bak.json`을 남기고 EEPROM·LeRobot 캐시·`config/calibration`에 쓴다. `calibrate --verify-zero`는 속도 20%로 전 관절을 0°로 보내고 실물과 3D의 0° 자세를 사람이 `yes`로 확인하면 팔 파일에 `calibration_checked`를 기록한다. 이 기록이 없거나 캘리브레이션 파일과 해시가 다르면 움직이는 명령은 `locked`다. 클램프로 고정하고 반경 50cm를 비우고 전원 스위치 옆에서만 한다.
  - 설정은 세 겹: `config/robot.yaml`(공통) ← `config/arms/<팔>.yaml`(팔별, git) ← `config/local.yaml`(PC별, git 제외, `local.example.yaml` 참고). 팔 고르기 `--arm` → local.yaml → 보드 시리얼 → 하나뿐이면 그것, 포트 고르기 `--port` → local.yaml → 보드 시리얼로 자동. 서보 캘리브레이션이 다르거나 팔 파일이 없으면 움직이는 명령은 `locked`로 거부되고 읽기만 된다. 설치는 `conda env create -f environment.yml`, 런처는 `so101.cmd`/`so101.sh`(환경 이름 `SO101_ENV`, 기본 so101).
  - `gui`: GUI 앱 실행(`so101_studio.gui.main(args)`를 부름, 팔은 GUI가 연결한다). GUI 모듈이나 의존성이 없으면 안내하고 끝난다 (`pip install -e .[gui]`).
  - `python -m so101_studio.watchdog [--port 8090] [--hotkey F9] [--sim]`: GUI의 E-STOP 감시 프로세스(`decisions/0026`). heartbeat가 1초 넘게 끊기거나 단축키(기본 F9)가 눌리면 팔 프로세스의 `StopListener`에 정지를 보내고, 0.3초 안에 응답이 없으면 그 프로세스를 끝낸다. `--sim`이면 기록만(`runs/watchdog-*.jsonl`).
  - `keys`: Tab으로 관절 / 직교 조그 / 좌표 입력 세 모드. 좌표: `x y 높이 [yaw]`, `d dx dy dz`, `frame table|robot|tool`, `tcp tip|ref`, `speed|jog|cart N`, 높이는 작업대 면 위 mm, 닿지 않으면 움직이지 않고 이유를 알림 (`decisions/0013`). 옵션 `--jog-speed`, `--cart-speed`, `--trace`(명령·실제 위치 기록과 관절별 떨림 요약). 키 상태를 창과 상관없이 읽으므로 실행 중 다른 창에 타이핑하지 않는다. 옛 `jog`(`--cartesian`)도 남아 있다.
  - 교시: `record <이름>`(홈으로 이동 -> 팔을 잡고 Enter -> 3초 뒤 토크가 꺼지고 손으로 움직인 자세와 시각을 기록(설정 `record_hz` 20Hz, `record_max_s` 90초, `--record-hz`, `--record-max-s`로 이번만 변경), `config/sequences/`에 길이·속도 요약과 함께 저장, `decisions/0018·0019`), `info <이름>`(길이, 평균·최대 속도, 멈춤/느림/빠름 구간 목록), `replay <이름>`(`--until-limit`, `--seg-speed 1=0.5,3=2`로 구간별 속도)·`seq <이름>`은 재생.
  - 기록·사고: `so101 incident [--last N]`(지금까지의 기록으로 `runs/incidents/<시각>-MANUAL/` 묶음 생성). 기록은 JSON Lines(`events.jsonl`, `status.jsonl`, 공통 머리 t(ns)·proc·pid·seq·level·code·kind·data, 읽기 함수는 `core/logread.py`), `decisions/0030`.
  - 진단: `powertest [반복]`(최대 200°/s로 크게 휘두르며 전압·전류·부하, 팔을 끝까지 폄), `shaketest [반복]`(가속 한계·속도별 이동마다 떨림 측정, m 키로 표시, `--speeds`와 가속 값 목록), `servoinfo`(서보 설정 레지스터 읽기), `linkwatch`(서보 1~6 응답이 끊기고 돌아오는 순간을 시각과 함께 표시, 접촉 불량 찾기). `--hz N`은 제어 주기를 이번만 바꿔 시험한다(50Hz가 가장 낫다).
  - 안전 설정: `--free`(작업대 가장자리만 끔), `--unsafe`(관절 한계·작업 영역·바닥 모두 끔, 한 주기 이동량 한계와 과열·과부하 감시는 유지), `--safety-margin N`(관절 한계를 N°만큼 줄이거나 음수면 실제 가동 범위까지 넓힘), `--box x0,x1,y0,y1,z0,z1`(XYZ 허용 범위, 작업대 좌표 mm, z는 작업대 면 위 턱 끝 높이). `keys` 좌표 모드에서는 `safety ...`. GUI는 `ArmService.get_safety()/set_safety()` (`decisions/0015`). `replay`는 `--until-limit`.
  - 팔은 클램프로 고정하고 전원 스위치 옆에서 지켜본다. 진단 명령은 팔을 크게 움직이므로 주변을 비운다.
- 코드가 더 생기면 실행 명령을 이 절에 추가한다.

## 문서 규칙
- 새 문서는 `docs/README.md`에 정해진 네 위치 중 하나에 만들고, 색인에 한 줄을 추가한다. 어느 위치에도 맞지 않는 문서는 만들지 않는다.
- 모든 문서 맨 위에 `상태: 초안 / 확정 / 대체됨(→ NNNN)`을 적는다.
- 결정 기록(`docs/decisions/`)은 수정하지 않는다. 결정이 바뀌면 새 기록을 만들고, 이전 기록의 상태만 바꾼다.
- 조사 원문, 1회성 메모, 중간 산출물은 커밋하지 않는다. 사용자가 승인한 결정만 결정 기록으로 남긴다.

## 주의사항
- HC-SR04는 5V, ESP32는 3.3V다. Echo 핀은 분압 저항이나 레벨 시프터를 거쳐 연결한다.
- 같은 실수를 두 번 바로잡았다면 여기에 한 줄로 추가한다.
- 로컬 폴더에서는 `git add -A`·`git commit -a`를 쓰지 않고, `git status --short`로 확인한 뒤 바꾼 파일만 이름으로 더한다. GUI 실행이 바꾼 설정·추적 안 한 백업·교시 기록이 커밋에 섞인 적이 두 번 있다(2026-11-06 GUI 폴더, 2026-11-11 #234).
