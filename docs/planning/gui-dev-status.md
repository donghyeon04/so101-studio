# GUI 개발 현황

상태: 초안 · 2026-11-02

GUI 세션의 인수인계 문서다. 기준 문서는 `planning/gui-spec.md`, 절차는 `guides/session-workflow.md`이다.

## 환경
- 폴더 `C:/Users/c/Desktop/so101-studio-gui`, 브랜치 `dev/gui-shell`. PR은 `main` 대상이고 G 단위로 끊는다.
- 출발점은 2fc1293(기획 브랜치)이고, 이 커밋은 PR #3(e5f3a48)으로 `main`에 들어갔다. 다시 만들거나 rebase하지 않는다.
- 설치: viser 1.1.1, nicegui 3.18.0. numpy는 2.2.6 conda 설치본 그대로다.
- 실행: `$env:PYTHONPATH="C:/Users/c/Desktop/so101-studio-gui/src"` 후 `python -m so101_studio.gui --sim [--port 8080]`. 아직 `--sim`만 된다. 주소는 `127.0.0.1`로 고정했다(NiceGUI 기본값은 `0.0.0.0`).

## 작업 진행
| 작업 | 상태 | 확인 |
|---|---|---|
| G0 | 완료 | `import viser, nicegui` 성공, numpy conda 설치본, `pyproject.toml`에 `gui` 묶음 |
| G1 | 완료 | 앱 틀과 빈 페이지 13개, E-STOP. `python -m pytest -q` 179 passed(GUI 37개 포함), ruff 통과. 브라우저에서 메뉴 이동, 새로고침 후 경로 유지, 사이드바 접기, E-STOP 알림 확인(1280×720, 1920×1080) |
| G2~G10 | 대기 | |

## G1에서 정해 둔 것
- 상태줄 값은 `—`로 비워 두었다. 팔 칸만 `시뮬레이션`이다. G2에서 연결한다.
- E-STOP은 G2 전까지 알림만 띄운다. `--sim` 없이는 실행을 거부하므로 실제 팔이 연결된 상태로 이 버튼을 쓰는 일은 없다.
- 페이지 머리의 상태 배지는 G2 전까지 `대기`로 고정이다.
- 사이드바는 1080p에서 스크롤 없이 다 보이고, 720p에서는 스크롤한다. 메뉴 높이 44px을 지키기 위해서다.
- 테스트는 pytest-asyncio를 더하지 않고 `asyncio.run`과 `user_simulation`으로 돈다. `pyproject.toml`의 pytest 설정은 팔 세션 영역이다.

## 디자인 도구 (사용자 승인 2026-11-02, 결정 기록은 기획 세션에 요청)
- NiceGUI 문서: 설치본의 `nicegui/llms.md`를 기준으로 한다.
- 화면 확인: 앱 내장 브라우저. Viser 3D가 안 나오면 Playwright CLI로 바꾼다(G3에서 확인).
- 검토: design 플러그인의 `design-critique`, `accessibility-review`, frontend-design 스킬. 설치는 했지만 이 세션에는 아직 안 올라와서 `SKILL.md`를 직접 읽어 따랐다.
- 상태 표시: 색, 모양, 글자 중 두 가지 이상(Carbon). 대비는 `tests/gui/test_theme.py`가 검사한다.

## 보낸 요청
없음. REQ-001~008은 기획이 미리 등록한 것을 팔 세션이 처리했다.

## 팔 세션 REQ 상태
- REQ-001~008 모두 구현됐다고 알려 왔다. 아직 `main`에 없고 GUI에서 확인하지 않았다.
- 브랜치는 합치는 순서대로 `dev/arm-req-workspace`(001·002·003·007), `dev/arm-req-gui-entry`(+005·008), `dev/arm-req-params-diag`(+004·006)이다.
- `main`에 합쳐지면 `git fetch origin && git merge origin/main`으로 받고 `backend.py`의 임시 우회를 지운 뒤 팔 세션에 "반영됨"을 알린다.
- G2·G6에 쓸 주의점: `studio.bus.subscribe("task", cb)`의 콜백은 작업 스레드에서 불리므로 큐로 넘긴다. `run_diagnostic`은 블로킹이고, `powertest`·`shaketest`는 확인 창이 필요하다.
