# GUI 개발 현황

상태: 초안 · 2026-11-07 밤 (로컬 총괄이 총정리하며 다시 씀. 지난 단계별 기록은 git 기록에 있다)

GUI 세션(총괄)의 인수인계 문서다. 기준 설계는 `planning/gui-spec.md`, 세션 절차는 `guides/session-workflow.md`, 할 일은 `planning/work-queue.md`.

## 지금 상태 (main)
- 페이지 14개(`gui/app.py`의 `PAGES`): 시작, 작업 공간, 직접 조작, 교시, 프로그램, 초음파, 수동 입력, 팔, ESP32, 파라미터, 안전, 작업대·좌표, 실행 기록, 로그. 카메라는 장치를 달 때까지 숨김(결정 0044). 운영자 화면 `/operator`는 틀 밖 별도 화면(시작 화면 링크, 새 탭).
- 공통 틀: 위 상태줄(팔·감시·버스·서보·센서·전압·온도·속도·조작권, 실행 방식, 다시 연결, 점검, 그리퍼 속도), 아래 조작 막대(손끝 좌표 등 + 속도·정지 해제·홈·조그 패널·E-STOP), 어디서나 여는 조그 패널(조이스틱 두 개·그리퍼).
- 주요 기능: 직접 조작(관절 값·좌표 이동·손끝 각도·미리보기·집어 옮기기·조이스틱·간편 이동), 교시(기록·원본/변환본 재생·구간 배속), 프로그램(자세 즐겨찾기 `config/waypoints.yaml` + 단계 목록 `config/programs/<이름>.yaml`, 한 단계씩·정지·현재 단계 강조, 실행 방식을 그대로 탐), 서보 건강 대시보드(`gui/health.py`), 장면 부품 3D(`config/scene.yaml`), 병렬 그리퍼 3D, 실행 기록 손실 알림, 운영자 화면(3D·지금 단계·물체·실행 버튼, 보기 전용은 버튼·E-STOP 숨김).
- 시험: `tests/gui` 약 355개(slow 30개 포함). `-m "not slow"` 약 5분, 전체 약 11분.

## 실행
- Git Bash: `export PATH="/c/miniforge3/envs/so101/Library/bin:/c/miniforge3/envs/so101:$PATH"` 뒤 `PYTHONPATH=src python -m so101_studio --sim gui` → `http://127.0.0.1:8080`. `--sim`을 빼면 실제 팔(보드 시리얼로 포트 자동).
- 시험: `PYTHONPATH=src python -m pytest -q -p no:cacheprovider tests/gui -m "not slow"`. PR은 관련 시험만, 전체는 하루 마칠 때 총괄이 한 번(결정 0046).
- 린트: `python -m ruff check src tests` (파일을 고치면 훅이 그 파일만 검사, 결정 0045).

## 알려진 함정
- GUI 시험은 고정 포트(8090·8091·8190 등)를 쓴다. 다른 세션과 동시에 돌리면 서로 깨진다. 시간에 민감한 시험(감시 heartbeat, 로그)은 부하에 흔들리니 실패하면 따로 다시 돌린다.
- 시뮬레이션 서버를 끄지 않고 다시 띄우면 포트 충돌로 바로 끝난다. 8080·8081·8090·8091을 먼저 비운다.
- 내장 브라우저 스크린샷이 화면 갱신보다 늦게 찍히는 일이 있다. 값은 `javascript_exec`로 다시 확인한다.
- 이 폴더에서 `git commit -a`를 쓰지 않는다(GUI 실행이 `config/`를 바꾼다). `config/sequences/test.json`·`config/waypoints.yaml`·`config/programs/`의 시험 파일은 커밋하지 않는다.
- PR 합치기는 `gh pr merge N --merge`(번호를 꼭 준다).
- `main` 브랜치는 `so101-studio-us`가 쓰고 있어 다른 폴더는 `origin/main`에 detached로 두거나 `dev/<작업>`을 딴다.

## 설계 메모 (코드만 봐서는 이유가 안 보이는 것)
- 조작권: 처음 연결된 브라우저(소켓 연결 시점)가 가진다. 끊기면 비우고 작업을 세운다. 자동으로 넘기지 않는다(조작 노트북을 새로고침하는 사이 프로젝터 탭이 가져가지 않게). 운영자 화면도 같다.
- E-STOP: 본 화면은 보기 전용도 누를 수 있다. 운영자 화면도 늘 보인다(결정 0047, #212). 다른 기기(`client.ip`가 루프백 아님)는 조작권을 받지 못한다(`gui/share.py`).
- `Backend`는 `sim`과 `real` 두 `ArmService`를 갖고 실행 방식(sim / real / both / sim_first)대로 `_dispatch`한다. 실제 팔 명령은 감시 프로세스가 살아 있을 때만(결정 0026). 자동 작업은 실제 팔이 있으면 "실제만"에서만.
- 프로그램 실행은 프로그램 전체를 팔 동작 하나(`programs.action`)로 만들어 `_dispatch`에 넘긴다. 그래서 실행 방식·안전 검사·시뮬레이션 먼저 확인을 그대로 쓴다. '동시' 방식에서 한 단계씩은 막는다.
- 운영자 화면의 "이번 실행"은 `State.run_t0`(시작 시각) 뒤 이벤트다. 수동 입력은 물체마다 PREPARE를 다시 거쳐 PREPARE로는 나눌 수 없다.
- 3D 메시는 `simplify()`로 2mm 격자(면 40만 → 2.8만). 3D 칸은 페이지마다 iframe을 새로 만든다.
- 상태 표시는 색 + 아이콘 + 글자(Carbon), 대비는 `tests/gui/test_theme.py`가 검사. 디자인 기본값은 gui-spec 5절.

## 남은 GUI 일 (순서는 `work-queue.md`)
1. ~~결정 0047~~ 끝(#212): `--share`, 원격 보기 전용, 운영자 E-STOP 항상.
2. 캘리브레이션 마법사: 1차 끝(#215, 측정 엔진은 REQ-052 대기). 2차(저장·0° 확인)는 #41(REQ-017) 합친 뒤, #41은 전원 보강 뒤 AC4 실물 합격을 기다린다.
3. 장면 부품 편집기: 사용자 STEP 파일을 받은 뒤.
4. 초음파 화면을 실제 작업 기록(`Studio.bus`의 `us.*`)으로, ESP32 페이지와 작업 실행의 포트 나누기: 초음파 실물(B1) 때.
5. ~~작업대 저장 뒤 3D 다시 그리기~~ 끝(#218). 감사의 "중복 시험(view_only·comm_loss)"은 화면마다 다른 버튼을 보는 시험이라 합치지 않기로 함(2026-11-10).
6. ~~놓기 부딪힘 판정에 수동 입력 물체~~ 끝(#217, `Backend._tell_known_objects`).
7. MuJoCo는 결정 0044 순서 마지막, menagerie `robotstudio_so101`부터(결정 0050).
