# 00. 툴링 셋업 & 레퍼런스 평가 기준

> 기획 계정과 개발 계정 모두 같은 도구를 갖추기 위한 체크리스트다.
> 마지막 갱신: 2026-10-31 (기획 세션)

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
| B4 | **Figma** 커넥터 | 와이어프레임과 디자인 | ⚠️ 현재 원격 세션에서는 프록시 403으로 연결 실패. 환경 네트워크 정책 확인 필요 |
| B5 | **Excalidraw MCP** (선택) | 손그림 스타일의 메뉴트리, 시스템 다이어그램, 저충실도 와이어프레임 | `claude mcp add --transport http excalidraw https://mcp.excalidraw.com/mcp` |

## C. 개발 단계 — 로컬 세션 전용

| # | 이름 | 용도 | 비고 |
|---|---|---|---|
| C1 | **so101-mcp** (Tibe31) | Claude가 SO-101 관절을 직접 움직이며 테스트. 관절별 안전 한계 포함 | LeRobot 캘리브레이션이 끝나야 실행됨. 시리얼 포트는 한 번에 한 클라이언트만 사용 |
| C2 | **ros-mcp-server** (robotmcp) | ROS2를 쓰기로 결정한 경우에만 | Apache-2.0. 실험용 |
| C3 | **Ponytail** (선택) | 코드를 적게 쓰도록 유도하는 규칙 모음 | `/plugin marketplace add DietrichGebert/ponytail` 후 `/plugin install ponytail@ponytail`. Node.js 필요. 효과 수치는 제작자가 직접 측정한 값이고, 제3자 측정에서는 약 15% 감소에 그쳤다 |
| C4 | **desktop-commander** | 로컬 셸, 시리얼 모니터, 장시간 실행 프로세스 | 현재 계정에 설치됨 |

---

## D. 레퍼런스 정량 평가 기준표 (모든 조사에 공통 적용)

각 항목을 1~5점으로 매기고 가중치를 곱해 합산한다(만점 100점).
가중치는 주제에 따라 조정할 수 있지만, 바꾸면 조사 문서에 그 사실을 적는다.

| 기준 | 가중치 | 5점 | 1점 |
|---|---|---|---|
| **하드웨어 적합도** — SO-101, STS3215, ESP32와 그대로 맞는가 | 5 | 같은 하드웨어 | 개념만 참고 가능 |
| **문제 일치도** — 내 문제와 얼마나 같은가 | 5 | 같은 문제를 해결 | 일부만 관련 |
| **유지보수 상태** — 최근 커밋, 이슈 응답 | 3 | 3개월 이내 활동 | 1년 넘게 방치 |
| **최신 LeRobot 호환성** | 3 | 최신 버전에서 동작 | 구버전 전용 (예: robot_MCP) |
| **문서·재현성** | 2 | README만으로 재현 가능 | 문서 없음 |
| **라이선스** | 1 | MIT / Apache-2.0 | 라이선스 없음 또는 비상업 |
| **커뮤니티 검증** — 스타 수, 사용 사례 | 1 | 많은 사람이 사용 | 개인 실험 |

조사 문서 템플릿: `docs/references/<주제>.md`
```
## 문제 정의
## 후보 (최소 10개) — 표: 이름 | 링크 | 기준별 점수 | 총점
## 상위 3개 비교
## 결정 및 근거
## 버린 후보와 이유
```
