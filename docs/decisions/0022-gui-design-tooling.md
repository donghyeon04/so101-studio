# 0022. GUI 디자인 도구는 와이어프레임(기획)과 스크린샷 반복(개발)으로 나눈다

상태: 대체됨(→ 0025) · 2026-11-02

## 맥락
GUI를 디자인하고 구현할 때 쓸 스킬, MCP, 플러그인 20개를 조사했다. 별이 많은 디자인 스킬(ui-ux-pro-max, taste, impeccable, superdesign)은 React와 웹사이트 기준이라 Viser(결정 0021)에는 맞지 않는다. Figma는 유료 좌석이 필요하고, Figma 시안을 Viser 코드로 옮길 수도 없다. 로컬 세션은 다른 계정이라, 이 세션의 커넥터가 따라가지 않는다.

## 결정
- 기획 세션: 화면마다 Excalidraw 와이어프레임을 그려 승인받은 뒤 개발로 넘긴다. 설명 모드는 필요하면 HTML 시안으로 흐름을 확인한다.
- 로컬 세션 설치 목록: Context7(Viser 문서 `/viser-project/viser`), Playwright MCP, design 플러그인, frontend-design 스킬. 설치 방법은 `guides/tooling-setup.md` C-4에 있다.
- 개발 방식: 화면 하나를 구현할 때마다 실행 → Playwright 스크린샷 → `/critique`로 비전문가가 읽을 수 있는지 확인 → 수정을 반복한다.
- 쓰지 않는 것: ui-ux-pro-max, taste, impeccable, superdesign, Figma, Chrome DevTools MCP(Playwright와 겹침), theme-factory, canvas-design.
- MuJoCo를 붙일 때는 mjviser(Viser 기반 MuJoCo 뷰어)를 먼저 검토한다.
