# GUI 개발 현황

상태: 초안 · 2026-11-02

GUI 세션의 인수인계 문서다. 기준 문서는 `planning/gui-spec.md`, 절차는 `guides/session-workflow.md`이다.

## 다음 할 일
- G4 진행 중(`dev/gui-manual`, PR #14까지 합쳐진 `main`에서 시작). 끝나면 G5부터 G9까지 `--sim`으로 이어 간다. PR은 올리되 합치지 않고, 다음 G는 앞 브랜치 위에 쌓는다. G10(실제 팔)은 사용자와 함께 한다.

## 환경
- 폴더 `C:/Users/c/Desktop/so101-studio-gui`. PR은 `main` 대상이고 G 단위로 끊는다. G0~G1은 `dev/gui-shell`(PR #7), G2는 그 위의 `dev/gui-state`(PR #11), G2b `dev/gui-watchdog`(PR #12), G3 `dev/gui-3d`(PR #14)까지 합쳐졌다.
- 출발점은 2fc1293(기획 브랜치)이고, 이 커밋은 PR #3(e5f3a48)으로 `main`에 들어갔다. 다시 만들거나 rebase하지 않는다.
- 설치: viser[urdf] 1.1.1, nicegui 3.18.0. numpy는 2.2.6 conda 설치본 그대로다.
- 실행: `$env:PYTHONPATH="C:/Users/c/Desktop/so101-studio-gui/src"` 후 `python -m so101_studio.gui --sim [--port 8080]`. 아직 `--sim`만 된다. 주소는 `127.0.0.1`로 고정했다(NiceGUI 기본값은 `0.0.0.0`).

## 작업 진행
| 작업 | 상태 | 확인 |
|---|---|---|
| G0 | 완료 | `import viser, nicegui` 성공, numpy conda 설치본, `pyproject.toml`에 `gui` 묶음 |
| G1 | 완료 | 앱 틀과 빈 페이지 13개, E-STOP. `python -m pytest -q` 179 passed(GUI 37개 포함), ruff 통과. 브라우저에서 메뉴 이동, 새로고침 후 경로 유지, 사이드바 접기, E-STOP 알림 확인(1280×720, 1920×1080) |
| G2 | 완료 | `backend`·`state`, 10Hz 상태줄, 조작권. `python -m pytest -q` 197 passed, ruff 통과. 브라우저 탭 2개로 확인: 팔이 움직이면 손끝 좌표와 배지가 바뀜, 둘째 탭은 보기 전용, 첫 탭을 닫으면 동작이 stopped로 멈추고 둘째 탭에 "조작권 받기"가 뜸, 새로고침해도 조작권 유지, E-STOP 후 배지 `정지` |
| G2b | 완료 | 감시 프로세스 함께 띄우기, 서버·브라우저 heartbeat, 등록, 상태 표시, 실제 팔 명령 막기. `dev/arm-req-watchdog`를 합친 임시 폴더에서 실제 감시 프로세스로 확인: 서버 루프 2초 멈춤과 탭 닫기가 `--sim` 기록에 `trip_ignored`로 남음, `--sim` 없이 탭 닫기는 `trip acked=true`로 시뮬레이션 팔에 정지 도착, GUI는 살아 있음 |
| G3 | 완료 | Viser를 직접 조작·작업 공간 페이지에 넣음, 실제 팔·계산 자세 두 개, 작업대·좌표축·물체·배출 구역. 내장 브라우저에서 WebGL 표시 확인(Playwright CLI 필요 없음). 지연: 측정 스크립트로 상태 갱신 -> 소켓 도착 중앙 25ms, 최대 34ms. 상태 주기 100ms와 화면 한 프레임을 더한 계산값이 0.2초 안이다. 브라우저 화면까지 직접 잰 값은 아니다(패널이 숨겨져 있었음) |
| G4~G10 | 대기 | |

## G1에서 정해 둔 것
- 사이드바는 1080p에서 스크롤 없이 다 보이고, 720p에서는 스크롤한다. 메뉴 높이 44px을 지키기 위해서다.
- 테스트는 pytest-asyncio를 더하지 않고 `asyncio.run`과 `user_simulation`으로 돈다. `pyproject.toml`의 pytest 설정은 팔 세션 영역이다.

## G2에서 정해 둔 것
- 비어 있는 칸: 센서(G6), 닿는지·오차(G4). 나머지 상태줄 값은 살아 있다.
- 조작권: 처음 연 브라우저가 가진다. 연결이 끊기면 `stop()` 후 비워 두고, 보기 전용 브라우저는 "조작권 받기"를 눌러야 가져간다. 자동으로 넘기면 조작 노트북을 새로고침하는 사이 프로젝터 탭이 조작권을 가져갈 수 있어서다.
- E-STOP은 보기 전용 브라우저도 누를 수 있다. 설계서의 "버튼이 꺼진 보기 전용"에서 E-STOP만 뺐다.
- 아직 팔을 움직이는 버튼이 없어, 조작권이 실제로 막는 버튼은 G4부터 생긴다.

## G3에서 정해 둔 것
- 메시: 원본 STL은 면 40만 개라 `simplify()`가 2mm 격자로 합쳐 2.8만 개로 줄인다. 새 의존성은 없다. 초기 전송량 0.74MB, 내장 브라우저에서 3초 안에 다 보인다. 줄이기 전에는 8초 넘게 비어 있었다.
- 계산 자세 팔은 `set_preview(q)`로 켠다. 평소에는 숨긴다(같은 자리에 겹치면 깜박인다). G4의 미리보기에 연결한다.
- 3D 칸은 `/`와 `/manual`에만 둔다. 페이지를 옮길 때마다 iframe을 다시 만들어 메시를 다시 받는다. 3초가 거슬리면 G4에서 하나를 살려 두는 방식을 검토한다.
- Viser 패널은 접어 둔다. 시점 버튼과 작업대 클릭, 끌기 손잡이는 G4에서 만든다.
- `pyproject.toml`의 `gui` 묶음을 `viser[urdf]`로 바꿨다. numpy는 그대로다.
- G2 버그 수정: 페이지를 그릴 때 조작권을 받아서 소켓을 열지 않는 HTTP 요청(curl 등)이 조작권을 가져갔다. 소켓이 연결될 때 받게 고치고 테스트를 넣었다.

## G2b에서 정해 둔 것
- REQ-011이 `main`에 들어와 임시 분기를 지웠다. 감시 칸의 `없음`은 이제 `Backend`를 감시 없이 만든 테스트에서만 나온다.
- 설계서는 "실제 팔이 연결되면" `/arm`을 보낸다고 하지만, G2b 완료 기준이 시뮬레이션 기록이라 `--sim`에서도 보낸다. 감시 프로세스가 `--sim`이면 기록만 한다.
- `/arm`은 조작권을 가진 브라우저가 처음 생긴 뒤에 보낸다. 그 전에는 브라우저 heartbeat가 없어 곧바로 끊김으로 보이기 때문이다. **G10 확인 사항:** 실제 팔에서 조작 브라우저를 닫으면 감시가 작동해 `작동함`으로 남고, 다음 조작자가 정지 해제해야 다시 움직인다.
- 브라우저 heartbeat는 Web Worker에서 보낸다. 숨겨진 탭의 `setInterval`은 브라우저가 늦춘다. 탭을 숨긴 채 오래 두는 경우는 시험하지 않았다(G10에서 확인).
- GUI가 강제로 꺼지면 감시 프로세스가 남을 수 있다. 시작할 때 이미 떠 있으면 그것을 쓴다(Windows에서는 같은 포트를 두 번 열 수 있어 요청이 아무 쪽으로나 간다). 남은 프로세스의 `sim` 값이 이번 실행과 다르면 `꺼짐`으로 치고 실제 팔을 막는다.
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
- REQ-011은 PR #13으로 `main`에 들어왔고 받았다. 임시 분기를 지웠다.
- G2·G6에 쓸 주의점: `studio.bus.subscribe("task", cb)`의 콜백은 작업 스레드에서 불리므로 큐로 넘긴다. `run_diagnostic`은 블로킹이고, `powertest`·`shaketest`는 확인 창이 필요하다.
