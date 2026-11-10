# 툴링 셋업

상태: 확정 · 갱신 2026-11-06

> 기획 계정과 개발 계정 모두 같은 도구를 갖추기 위한 체크리스트다.

---

## A. 필수 — 모든 계정·세션에 설치

| # | 이름 | 종류 | 용도 | 설치 |
|---|---|---|---|---|
| A1 | **Superpowers** | 플러그인 | brainstorming → writing-plans → executing-plans → TDD → code-review까지 이어지는 작업 흐름. 기획 세션과 개발 세션이 같은 흐름을 쓰게 한다 | `/plugin marketplace add obra/superpowers-marketplace` 후 `/plugin install superpowers@superpowers-marketplace` |
| A2 | **Context7** | MCP + 플러그인 | LeRobot, ESP-IDF/Arduino, FastAPI 같은 라이브러리의 **최신 문서**를 가져온다. LeRobot은 API가 자주 바뀌므로 필수 | `/plugin marketplace add upstash/context7` 후 설치. 또는 `claude mcp add --transport http context7 https://mcp.context7.com/mcp` |
| A3 | **Hugging Face MCP** | MCP | LeRobot 데이터셋, 정책 모델, 관련 논문 검색. 레퍼런스 10개 이상 조사할 때 핵심 소스 | `claude mcp add hf-mcp-server -t http "https://huggingface.co/mcp?login"` |
| A4 | **GitHub MCP** | MCP | 오픈소스 레퍼런스의 코드, 이슈, 스타 수, 최근 커밋 확인 (정량 평가에 사용) | claude.ai 커넥터에서 GitHub 연결 |
| A5 | **Tavily** (또는 deep-research) | 플러그인/스킬 | 여러 출처를 묶어 인용이 붙은 조사 결과를 만든다. 레퍼런스 10개 이상 조사에 사용 | 현재 계정에 설치됨. 다른 계정에도 같은 플러그인 설치 |
| A6 | **claude-md-management** (Anthropic 공식) | 플러그인 | `claude-md-improver`: CLAUDE.md 품질 점수와 개선안 / `/revise-claude-md`: 세션 종료 시 배운 점을 반영 | `/plugin install claude-md-management@claude-plugins-official` |
| A7 | **skill-creator** | 스킬 | 레퍼런스 조사 같은 반복 절차를 프로젝트 skill(`.claude/skills/`)로 만든다. 프로젝트 skill은 레포에 있어서 계정을 바꿔도 따라온다 | 현재 계정에 있음 (anthropic-skills) |

### 내장 기능 (설치 불필요)
- `/doctor prompt-audit`: CLAUDE.md, rules, skills의 모순과 낡은 지침 점검 (v2.1.283 이상)
- `/context`: CLAUDE.md가 실제로 로드됐는지 확인

### 클라우드 세션 참고
- Context7과 Hugging Face는 **커넥터 버전**(claude.ai 커넥터)이 원격 세션에서 동작한다. 플러그인 버전은 프록시에 막혀 연결에 실패했으므로, 원격 세션에서는 커넥터를 쓴다.
- 원문 확인이 필요한 도메인(arxiv.org, humanlayer.dev 등)은 환경 설정의 Network access → Allowed domains에 추가해야 열린다.

## B. 기획 단계 — 기획 계정 위주

| # | 이름 | 용도 | 비고 |
|---|---|---|---|
| B1 | **product-management** 플러그인 (`brainstorm`, `write-spec`, `roadmap-update`) | 아이디에이션, 기능명세서(PRD), 로드맵 | 현재 계정에 설치됨 |
| B2 | **engineering** 플러그인 (`architecture`, `system-design`) | 아키텍처 결정을 ADR로 기록. PC ↔ ESP32 ↔ 서보 구조 설계 | 현재 계정에 설치됨 |
| B3 | **design** 플러그인 (`design-critique`, `ux-copy`, `design-handoff`) | 와이어프레임 리뷰, UI 문구, 개발 인계 문서 | 현재 계정에 설치됨 |
| B4 | ~~Figma 커넥터~~ | 쓰지 않음 (`decisions/0022`: 유료 좌석, Viser로 옮길 수 없음) | |
| B5 | **Excalidraw MCP** | GUI 화면별 와이어프레임 (`decisions/0022`), 메뉴 트리, 시스템 다이어그램 | `claude mcp add --transport http excalidraw https://mcp.excalidraw.com/mcp` |

## C. 개발 단계 — 로컬 세션 (Windows PC)

### C-1. 개발 도구 (플러그인 아님, 먼저 설치)
> 개발하지 않고 써 보기만 하는 팀원은 아래 표 대신 레포 루트의 `설치.cmd`를 한 번 실행하고 `시뮬레이션 실행.cmd`로 켠다 (`decisions/0040`, REQ-029).

| # | 이름 | 용도 | 비고 |
|---|---|---|---|
| C1 | Git for Windows | 레포 작업. Windows용 Claude Code도 Git Bash를 쓴다 | |
| C2 | Miniforge (conda) | Python 3.12 이상 가상환경. LeRobot 공식 문서가 conda를 권장한다 | LeRobot은 Python 3.12 이상 필요 |
| C3 | LeRobot | 팔 제어, 기구학 (`decisions/0003`) | `pip install "lerobot[feetech,kinematics]"`. `kinematics`가 설치하는 placo는 Windows 휠이 없어 실패할 수 있다 → T1에서 확인 |
| C4 | USB 시리얼 드라이버 (CH340 또는 CP210x) | 버스 서보 보드와 ESP32의 COM 포트 인식 | 장치 관리자에서 COM 번호 확인 |
| C5 | ruff, pytest | 코드 형식 검사, `--sim` 단위 테스트 | |
| C6 | Arduino IDE 2 또는 PlatformIO + ESP32 보드 패키지 3.x | ESP32 스캔 펌웨어 | Windows 사용자 폴더 이름이 한글이면 펌웨어 링크(xtensa ld)가 실패한다. core를 영문 경로에 설치한다: `ARDUINO_DIRECTORIES_DATA=C:\arduino15`, `ARDUINO_DIRECTORIES_USER=C:\arduino15\user`를 설정한 뒤 core 설치. 상세는 `us-dev-status.md` 알려진 함정 8번 |

### C-2. Claude Code 플러그인·MCP·스킬
| # | 이름 | 용도 | 비고 |
|---|---|---|---|
| C7 | Superpowers (A1) | 개발 흐름: `writing-plans` → `executing-plans` → `test-driven-development` → `systematic-debugging` → `verification-before-completion` → `requesting-code-review` | 설계서의 작업 T1~T11을 하나씩 계획→구현→검증으로 진행 |
| C8 | Context7 (A2) | LeRobot `SO101Follower`, `RobotKinematics` 등 최신 API 확인 | 로컬에서는 플러그인 버전도 동작한다 |
| C9 | engineering 플러그인 | `debug`, `code-review`, `testing-strategy` | |
| C10 | desktop-commander | Windows 셸, 시리얼 모니터, 오래 도는 프로세스 관리 | |
| C11 | so101-mcp (Tibe31, 선택) | Claude가 관절을 직접 움직여 보는 초기 탐색용 | 시리얼 포트는 한 번에 한 프로그램만 쓸 수 있다. 우리 코드를 실행하기 전에 반드시 끈다 |
| C12 | Ponytail (선택) | 코드를 적게 쓰도록 유도하는 규칙 모음 | `/plugin marketplace add DietrichGebert/ponytail` 후 `/plugin install ponytail@ponytail`. Node.js 필요. 제3자 측정에서는 약 15% 감소 |
| C13 | huggingface-skills (나중) | ACT 등 학습 정책을 할 때만 (선택 층) | 지금은 불필요 |

### C-3. 쓰지 않는 것
- ros-mcp-server: ROS 2를 쓰지 않기로 했다 (`decisions/0003`).
- GUI 디자인: ui-ux-pro-max, taste-skill, impeccable, superdesign, Figma, Chrome DevTools MCP, Playwright MCP, theme-factory, canvas-design, 별도 `DESIGN.md` (`decisions/0022`, `0025`).

### C-4. GUI 단계 (`decisions/0023`, `0025`)
| # | 이름 | 용도 | 설치 |
|---|---|---|---|
| C14 | **Viser** (`viser[urdf]`) | GUI와 3D 화면. URDF 표시는 yourdfpy가 맡는다 | so101 환경에서 먼저 `pip install --dry-run "viser[urdf]"`로 numpy가 다시 설치되지 않는지 확인한 뒤 `pip install "viser[urdf]"`. 설치 후 `conda list numpy`에서 채널이 conda-forge인지 확인한다 |
| C14b | **NiceGUI** | 앱 틀(사이드바, 페이지, 카드, 그래프, 상태줄). 3D는 C14 Viser를 페이지 안에 넣는다 (`decisions/0023`) | C14와 같은 방법으로 `pip install --dry-run nicegui` 확인 후 `pip install nicegui`. Context7 ID는 `/zauberzeug/nicegui` |
| C15 | **NiceGUI `llms.md`** + **Context7** (C8) | NiceGUI 구현 기준 문서는 `llms.md`, Context7은 보조(`/zauberzeug/nicegui`, Viser는 `/viser-project/viser`) | C8과 같음 |
| C16 | **앱 내장 브라우저** (기본) / **Playwright CLI** (예비) | 실행한 GUI(`http://localhost:8080`)를 열어 화면 확인. Viser 3D(WebGL)가 내장 브라우저에 안 나오면 Playwright CLI로 스크린샷 | 내장 브라우저는 설치 불필요. Playwright CLI: `npm install -g @playwright/cli@latest` 후 `playwright-cli install --skills`. Playwright MCP는 쓰지 않는다 (`decisions/0025`) |
| C17 | **design 플러그인** | `/critique`(비전문가 관점 검토), `/ux-copy`(버튼·설명 문구), `/accessibility`(대비·글자 크기) | `/plugin marketplace add anthropics/knowledge-work-plugins` 후 `/plugin install design@knowledge-work-plugins` |
| C18 | **frontend-design 스킬** (Anthropic 공식) | 색, 글자, 배치 원칙. Viser에서는 브랜드 색, `add_html` 설명 카드, plotly 그래프에 적용 | `/plugin install frontend-design@claude-plugins-official`. 목록에 없으면 `/plugin marketplace add anthropics/skills` 후 설치 |

화면 하나를 만들 때마다 실행 → C16으로 화면 확인 → C17 `/critique` → 수정 순서로 반복한다. 상태 표시는 IBM Carbon 상태 표시 규칙(색 + 아이콘 + 글자)을 따른다.

---

### C-5. mattpocock/skills (`decisions/0032`)
레포의 `.claude/skills/`에 들어 있어 따로 설치하지 않는다(플러그인 `mattpocock-skills`는 설치하지 않는다. 설치하면 겹친다). 원본 커밋 `4ae2317`, MIT.

| 스킬 | 부르는 법 | 쓸 곳 |
|---|---|---|
| `grill-me` → `grilling` | `/grill-me` | 계획·설계를 여러 질문 라운드로 다듬기 |
| `handoff` | `/handoff` | 세션 사이 인수인계 문서(임시 디렉터리) |
| `wait-what` | `/wait-what` | 설명이 어려울 때 쉬운 한국어로 다시 |
| `to-questionnaire` | `/to-questionnaire` | 팀원에게 물을 결정을 질문지로 |
| `retro` | `/retro` | 세션이 끝날 때 작업 환경 개선점 |
| `improve-codebase-architecture` | `/improve-codebase-architecture` | 로컬 세션에서 며칠에 한 번 코드 구조 점검 |
| `codebase-design`, `prototype`, `pr`, `writing-for-agents` | 자동(상황에 맞으면 에이전트가 부름) | 모듈 설계, 버리는 시안, PR 본문, CLAUDE.md·스킬 쓰기 |

쓰지 않는 것: `setup-matt-pocock-skills`, `grill-with-docs`, `domain-modeling`, `research`, `tdd`, `diagnosing-bugs`, `code-review`, `implement`, `implement-spec`, `triage`, `to-tickets`, `to-spec`, `wayfinder`, `ask-matt`, `teach`, `wizard` (규칙 충돌 또는 Superpowers와 겹침).

### C-6. 사용자 지정 추가 (2026-11-06)
| # | 이름 | 종류 | 용도 | 설치 |
|---|---|---|---|---|
| C19 | **Stageshow** (Benji Taylor, `benjitaylor/skills`, MIT) | 스킬 | 제품 데모 화면을 만들고 영상으로 찍어 다듬는다(빽빽하거나 비거나 갇힌 화면을 고침). 시연 영상과 GUI 화면 다듬기에 쓴다 | Node.js 22.20 이상에서 `npx skills add benjitaylor/skills --skill stageshow`. 레포가 아니라 사용자 범위(`~/.claude/skills/`)에 설치한다. 설치 전에 레포의 `SKILL.md`·`LICENSE`를 읽고 확인한다(2차 자료로만 확인함) |

## D. 레퍼런스 평가 기준

평가표와 조사 절차는 프로젝트 skill `.claude/skills/researching-references/SKILL.md`에 있다. 레포에 포함되어 있어서 어느 계정에서든 따로 설치할 필요가 없다.
