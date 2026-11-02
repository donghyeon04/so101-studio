# 0025. GUI 화면 확인은 앱 내장 브라우저로, 디자인 기준은 NiceGUI llms.md와 Carbon 상태 표시 규칙

상태: 확정 · 2026-11-02 (0022를 대체)

## 맥락
GUI 세션이 G1~G2를 진행하며 기획 요청을 보냈고, 사용자가 2026-11-02에 승인했다. Microsoft는 코딩 에이전트에 Playwright MCP보다 Playwright CLI(`@playwright/cli`, 스킬 포함)를 권한다. Claude Code 앱의 내장 브라우저는 따로 설치할 것이 없다. 디자인 스킬은 React·웹사이트 기준이라, NiceGUI 화면에는 NiceGUI 공식 문서와 검증된 디자인 시스템의 규칙이 더 직접적이다.

## 결정
- 화면 확인은 Claude Code 앱 내장 브라우저로 한다. Viser 3D(WebGL)가 내장 브라우저에 나오지 않으면 Playwright CLI로 스크린샷을 찍는다. Playwright MCP는 쓰지 않는다.
- NiceGUI 구현 기준 문서는 NiceGUI의 `llms.md`(에이전트용 공식 요약)이고, Context7(`/zauberzeug/nicegui`)은 보조로 쓴다.
- 상태 표시(배지, 알림, 연결·오류 표시)는 IBM Carbon의 상태 표시 규칙을 따른다. 색만으로 구분하지 않고 아이콘과 글자를 같이 쓴다.
- `DESIGN.md` 같은 별도 디자인 문서는 만들지 않는다. 색과 크기는 `gui-spec.md` 5절과 `gui/theme.py`에만 둔다.
- 0022의 나머지는 유지한다: 기획 단계 Excalidraw 와이어프레임, design 플러그인(`/critique`, `/ux-copy`, `/accessibility`), frontend-design 스킬, 쓰지 않는 도구 목록, MuJoCo를 붙일 때 mjviser 검토.
