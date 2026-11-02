# GUI 개발 현황

상태: 초안 · 2026-11-02

GUI 세션의 인수인계 문서다. 기준 문서는 `planning/gui-spec.md`, 절차는 `guides/session-workflow.md`이다.

## 환경
- 폴더 `C:/Users/c/Desktop/so101-studio-gui`. PR은 `main` 대상이고 G 단위로 끊는다. G0~G1은 `dev/gui-shell`(PR #7), G2는 그 위의 `dev/gui-state`(PR #11), G2b는 그 위의 `dev/gui-watchdog`다. 이 순서로 합친다.
- 출발점은 2fc1293(기획 브랜치)이고, 이 커밋은 PR #3(e5f3a48)으로 `main`에 들어갔다. 다시 만들거나 rebase하지 않는다.
- 설치: viser 1.1.1, nicegui 3.18.0. numpy는 2.2.6 conda 설치본 그대로다.
- 실행: `$env:PYTHONPATH="C:/Users/c/Desktop/so101-studio-gui/src"` 후 `python -m so101_studio.gui --sim [--port 8080]`. 아직 `--sim`만 된다. 주소는 `127.0.0.1`로 고정했다(NiceGUI 기본값은 `0.0.0.0`).

## 작업 진행
| 작업 | 상태 | 확인 |
|---|---|---|
| G0 | 완료 | `import viser, nicegui` 성공, numpy conda 설치본, `pyproject.toml`에 `gui` 묶음 |
| G1 | 완료 | 앱 틀과 빈 페이지 13개, E-STOP. `python -m pytest -q` 179 passed(GUI 37개 포함), ruff 통과. 브라우저에서 메뉴 이동, 새로고침 후 경로 유지, 사이드바 접기, E-STOP 알림 확인(1280×720, 1920×1080) |
| G2 | 완료 | `backend`·`state`, 10Hz 상태줄, 조작권. `python -m pytest -q` 197 passed, ruff 통과. 브라우저 탭 2개로 확인: 팔이 움직이면 손끝 좌표와 배지가 바뀜, 둘째 탭은 보기 전용, 첫 탭을 닫으면 동작이 stopped로 멈추고 둘째 탭에 "조작권 받기"가 뜸, 새로고침해도 조작권 유지, E-STOP 후 배지 `정지` |
| G2b | 완료(REQ-011 합류 대기) | 감시 프로세스 함께 띄우기, 서버·브라우저 heartbeat, 등록, 상태 표시, 실제 팔 명령 막기. `dev/arm-req-watchdog`를 합친 임시 폴더에서 실제 감시 프로세스로 확인: 서버 루프 2초 멈춤과 탭 닫기가 `--sim` 기록에 `trip_ignored`로 남음, `--sim` 없이 탭 닫기는 `trip acked=true`로 시뮬레이션 팔에 정지 도착, GUI는 살아 있음 |
| G3~G10 | 대기 | |

## G1에서 정해 둔 것
- 사이드바는 1080p에서 스크롤 없이 다 보이고, 720p에서는 스크롤한다. 메뉴 높이 44px을 지키기 위해서다.
- 테스트는 pytest-asyncio를 더하지 않고 `asyncio.run`과 `user_simulation`으로 돈다. `pyproject.toml`의 pytest 설정은 팔 세션 영역이다.

## G2에서 정해 둔 것
- 비어 있는 칸: 센서(G6), 닿는지·오차(G4). 나머지 상태줄 값은 살아 있다.
- 조작권: 처음 연 브라우저가 가진다. 연결이 끊기면 `stop()` 후 비워 두고, 보기 전용 브라우저는 "조작권 받기"를 눌러야 가져간다. 자동으로 넘기면 조작 노트북을 새로고침하는 사이 프로젝터 탭이 조작권을 가져갈 수 있어서다.
- E-STOP은 보기 전용 브라우저도 누를 수 있다. 설계서의 "버튼이 꺼진 보기 전용"에서 E-STOP만 뺐다.
- 아직 팔을 움직이는 버튼이 없어, 조작권이 실제로 막는 버튼은 G4부터 생긴다.

## G2b에서 정해 둔 것
- 임시 우회: `watchdog_link.py`의 `# REQ-011 임시`. 감시 모듈이 `main`에 없으면 감시 칸을 `없음`으로 두고 시뮬레이션만 허용한다. `main`에 들어오면 이 분기와 `test_missing_module_reads_as_none`을 지운다.
- 설계서는 "실제 팔이 연결되면" `/arm`을 보낸다고 하지만, G2b 완료 기준이 시뮬레이션 기록이라 `--sim`에서도 보낸다. 감시 프로세스가 `--sim`이면 기록만 한다.
- `/arm`은 조작권을 가진 브라우저가 처음 생긴 뒤에 보낸다. 그 전에는 브라우저 heartbeat가 없어 곧바로 끊김으로 보이기 때문이다. **G10 확인 사항:** 실제 팔에서 조작 브라우저를 닫으면 감시가 작동해 `작동함`으로 남고, 다음 조작자가 정지 해제해야 다시 움직인다.
- 브라우저 heartbeat는 Web Worker에서 보낸다. 숨겨진 탭의 `setInterval`은 브라우저가 늦춘다. 탭을 숨긴 채 오래 두는 경우는 시험하지 않았다(G10에서 확인).
- GUI가 강제로 꺼지면 감시 프로세스가 남을 수 있다. 남은 프로세스의 `sim` 값이 이번 실행과 다르면 `꺼짐`으로 치고 실제 팔을 막는다.
- `Backend.resume()`은 만들었지만 "정지 해제" 버튼은 G4에서 붙인다. 그 전까지 E-STOP을 누르면 앱을 다시 켜야 한다.

## 디자인 도구 (decisions/0025)
- NiceGUI 문서: 설치본의 `nicegui/llms.md`를 기준으로 한다.
- 화면 확인: 앱 내장 브라우저. Viser 3D가 안 나오면 Playwright CLI로 바꾼다(G3에서 확인).
- 검토: design 플러그인의 `design-critique`, `accessibility-review`, frontend-design 스킬.
- 상태 표시: 색, 아이콘, 글자(Carbon). 대비는 `tests/gui/test_theme.py`가 검사한다.

## 보낸 요청
- REQ-009 `ArmService.resume()`: E-STOP 해제. G4에서 필요하고, 오기 전에는 `backend.py`에서 `arm.ctl.resume()`으로 임시 우회한다.
- REQ-001~008은 기획이 미리 등록한 것을 팔 세션이 처리했다.

## 팔 세션 REQ 상태
- REQ-001~008은 `main`에 합쳐졌고(8c002fe), `dev/gui-state`에 받았다. 임시 우회는 없었다. `python -m so101_studio --sim gui`로 REQ-005를 확인했고(HTTP 200), 나머지는 쓰는 작업에서 확인한다. 사용법은 `arm-dev-status.md` 14절이다.
- REQ-009·010은 PR #8로 `main`에 들어왔고 받았다. `Backend.resume()`이 `ArmService.resume()`을 쓴다. 임시 우회는 없다.
- REQ-011은 `dev/arm-req-watchdog`에 있고 아직 `main`에 없다. 위 임시 폴더 확인 결과 GUI 쪽 규약은 맞는다.
- G2·G6에 쓸 주의점: `studio.bus.subscribe("task", cb)`의 콜백은 작업 스레드에서 불리므로 큐로 넘긴다. `run_diagnostic`은 블로킹이고, `powertest`·`shaketest`는 확인 창이 필요하다.
