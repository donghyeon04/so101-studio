# GUI 개발 현황

상태: 초안 · 2026-11-02

GUI 세션의 인수인계 문서다. 기준 문서는 `planning/gui-spec.md`, 절차는 `guides/session-workflow.md`이다.

## 환경
- 폴더 `C:/Users/c/Desktop/so101-studio-gui`, 브랜치 `dev/gui-shell`.
- 시작점은 `origin/claude/hopeful-hamilton-amqaac`(2fc1293)이다. 가이드 2절은 `origin/main`을 쓰라고 하지만, 지금 `origin/main`에는 팔 코드와 GUI 설계 문서가 없다. 기획 브랜치는 `dev/arm-t0-t7`(ba53a03) 위에 문서를 얹은 것이라 둘 다 들어 있다. 기획·팔 브랜치가 `main`에 합쳐지면 `main`을 받는다.
- 설치: viser 1.1.1, nicegui 3.18.0. numpy는 2.2.6 conda 설치본 그대로다.
- 팔 세션: `ListAgents`에서 `ARM T0-T1 하드웨어 초기 설정 [e97873]`이 보인다(2026-11-02 시작 시점).

## 작업 진행
| 작업 | 상태 | 확인 |
|---|---|---|
| G0 | 완료 | `python -m pytest -q` 142 passed, `import viser, nicegui` 성공, numpy conda 설치본 |
| G1~G10 | 대기 | |

## 보낸 요청
없음.

## 받은 알림
- REQ-001~004: 팔 세션이 브랜치 `dev/arm-gui-api`(커밋 629d758, 아직 PR 아님)에 구현했다고 알려 왔다. REQ-005(`so101 gui` → `so101_studio.gui.main(args)`), REQ-006(`ArmService.run_diagnostic`)도 같은 브랜치에 완료로 알려 왔다. REQ-007(`studio.bus.subscribe("task", cb)`, 콜백이 작업 스레드에서 불리므로 GUI는 큐로 넘긴다)도 완료로 알려 왔다. GUI 쪽 확인은 아직 안 했다. REQ-008은 진행 중이다.
