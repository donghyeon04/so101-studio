# so101-studio

SO-101(SO-ARM101) 로봇 암 프로젝트. 문서 색인은 `docs/README.md`에 있다.

## 작업 원칙
- 기능 구현, 문제 해결, 기술·도구 선택은 처음부터 설계하지 않고 오픈소스·레퍼런스를 가져와 고친다. 결정하기 전에 `researching-references` skill로 조사하고, 사용자 승인을 받은 뒤 진행한다.
- 판단 요청("어떻게 할까", "고민이야")에는 바로 답하지 않고 질문을 한 번에 하나씩 던진다(소크라테스 모드). "정리해줘"나 "결론 내줘"라고 하면 그때 정리한다.
- 실행 요청("만들어줘", "작성해줘")은 바로 실행한다.

## 세션 구분
- 원격 레포 세션 = 기획: 기획, 아키텍처, 레퍼런스 조사, 문서 작성. 프로젝트 코드는 쓰지 않는다.
- 로컬 세션 = 개발: 코드 작성, 시리얼 포트·ESP32·서보 테스트, 펌웨어 업로드. 하드웨어 작업은 로컬에서만 할 수 있다.
- 개발 세션은 `docs/planning/`의 확정 문서와 `docs/decisions/`를 근거로 작업한다.
- 다른 계정에서 작업을 시작하면 `docs/guides/tooling-setup.md`의 설치 목록부터 맞춘다.

## 개발 세션 규칙
- 작업 전에 `docs/planning/system-architecture.md`와 해당 모듈 설계서(`arm-software-spec.md` 등)를 읽고, 작업 순서표의 다음 작업부터 한다.
- `main`에서 `dev/<작업>` 브랜치를 따서 작업하고, 끝나면 PR로 합친다 (`decisions/0007`).
- 설계서와 다르게 구현해야 하면 멈추고 사용자에게 알린다. 설계서는 기획 세션에서 고친다.
- 하드웨어 없이 `--sim`으로 먼저 테스트한다. 실제 팔은 속도 30% 이하로 처음 움직이고, 사람이 전원 스위치 옆에서 대기한다.
- 측정값은 `docs/planning/measurements.md`에 기록한다.
- 환경: conda env `so101`(Python 3.12). 먼저 `conda activate so101`로 활성화한다. 활성화 없이 env의 python.exe를 바로 부르면 numpy 행렬곱에서 아무 메시지 없이 종료(exit 127)된다(MKL DLL 경로). Git Bash에서는 `export PATH="/c/Users/c/miniforge3/envs/so101/Library/bin:/c/Users/c/miniforge3/envs/so101:$PATH"`.
- numpy는 conda 설치본(MKL, placo와 맞춤)이다. `pip install`로 numpy를 다시 설치하지 않는다.
- 테스트: `python -m pytest -q` (레포 루트에서). 린트: `python -m ruff check src tests`.
- 실행(어느 폴더에서든, conda 활성화 없이): `so101.cmd [--sim] [--speed N] <명령>` (레포 루트의 런처가 so101 conda 환경을 직접 지정한다. PowerShell에서 레포 폴더 안이면 `.\so101 ...`). 원래 형태는 `python -m so101_studio ...`. 모든 명령은 `service/arm_service.py`의 `ArmService`를 거치고 `runs/날짜-시각/`에 `events.jsonl`, `status.csv`(전압 포함)를 남긴다.
  - 동작: `status|home|park|movej|goto|pick|place|gripper`, 자동 작업 `run --source manual [--step] [--dry-run]`(`config/manual_objects.yaml`의 물체를 가까운 순으로 집어 `workspace.yaml`의 배출 구역으로 옮긴다).
  - `keys`: Tab으로 관절 / 직교 조그 / 좌표 입력 세 모드. 좌표: `x y 높이 [yaw]`, `d dx dy dz`, `frame table|robot|tool`, `tcp tip|ref`, `speed|jog|cart N`, 높이는 작업대 면 위 mm, 닿지 않으면 움직이지 않고 이유를 알림 (`decisions/0013`). 옵션 `--jog-speed`, `--cart-speed`, `--trace`(명령·실제 위치 기록과 관절별 떨림 요약). 키 상태를 창과 상관없이 읽으므로 실행 중 다른 창에 타이핑하지 않는다. 옛 `jog`(`--cartesian`)도 남아 있다.
  - 교시: `record <이름>`(토크를 끄고 손으로 가르침, `config/sequences/`에 저장), `record <이름> --follow [--stiffness N]`(토크를 켠 채 약하게 두고 손으로 밀어 가르침, `decisions/0016`), `replay <이름>`, `seq <이름>`.
  - 진단: `powertest [반복]`(최대 200°/s로 크게 휘두르며 전압·전류·부하, 팔을 끝까지 폄), `shaketest [반복]`(가속 한계·속도별 이동마다 떨림 측정, m 키로 표시, `--speeds`와 가속 값 목록), `servoinfo`(서보 설정 레지스터 읽기). `--hz N`은 제어 주기를 이번만 바꿔 시험한다(50Hz가 가장 낫다).
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
