# 0054. GUI 화면 버그는 Playwright 화면 시험 묶음으로 잡는다

상태: 확정 · 2026-11-12 (사용자 승인, `researching-references`로 후보 14개 비교, 로컬 총괄)

## 맥락
nicegui.testing의 User 시뮬레이션은 브라우저 없이 요소·글자만 봐서, 11-11 사용자가 찾은 화면 버그(조이스틱 겹침, 조그 패널을 열면 Viser 창 펼침, 3D 점을 끌 때 알림 쌓임)를 놓쳤다.

## 결정
1. `tests/gui_visual/`(slow): 시뮬레이션 GUI를 실제 팔 GUI와 겹치지 않는 포트(웹 8380, 3D 8381, 감시 8490)로 `--no-browser`로 띄우고 Python Playwright(Chromium)로 연다.
2. 사진 비교 `pytest-playwright-visual-snapshot`: 주요 페이지 × 두 화면 크기, 3D 캔버스·시각·숫자는 가린다.
3. 배치 점검(직접 작성): 보이는 조작 요소 겹침 0, 가로 넘침 없음, Viser 창 접힘, 알림 수 한도.
4. 몽키 시험 gremlins.js(MIT, 씨앗 고정)를 하루 마감 때. 접근성(axe)은 나중.
5. 버린 것: Lost Pixel(보관됨), SaaS(유료·외부 업로드), BackstopJS·reg-suit(Node 도구 따로), NiceGUI `screen`(Selenium을 새로 들임).
