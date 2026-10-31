# 레퍼런스 조사: CLAUDE.md 작성법

> 조사일: 2026-10-31 · 기획 세션
> 원칙: CLAUDE.md §3 (레퍼런스 10개 이상, 정량 평가)

## 문제 정의
이 프로젝트는 **여러 계정 × 원격/로컬 세션**을 오가며 진행한다. 세션이 바뀌어도 같은 규칙으로 일하게 하려면
CLAUDE.md(그리고 그 주변 장치)를 어떻게 써야 하는가?

고민할 요소
1. 무엇을 CLAUDE.md에 넣고 무엇을 뺄 것인가 (길이, 내용)
2. 절차(레퍼런스 조사 같은 것)를 어디에 둘 것인가 (CLAUDE.md, skill, rule, hook)
3. 계정이 바뀌어도 따라오게 하려면 무엇을 레포에 두어야 하는가
4. 시간이 지나도 낡지 않게 어떻게 유지보수할 것인가

## 평가 기준 (이 주제용으로 조정)
하드웨어 관련 기준은 이 주제와 맞지 않아 아래 기준으로 바꿨다. 가중치 합 20, 각 1~5점, 만점 100점.

| 기준 | 가중치 | 5점 | 1점 |
|---|---|---|---|
| 근거 수준 | 5 | 통제 실험 / 공식 스펙 | 개인 의견 |
| 우리 상황 적합도 (다중 계정, 기획/개발 분리, LeRobot) | 5 | 그대로 적용 | 일반론 |
| 최신성 (현재 Claude Code 기능 반영) | 3 | 2026년 기준 | 구버전 전제 |
| 실행 가능성 | 3 | 바로 따라 할 수 있는 지침이나 도구 | 추상적 |
| 원문 검증 여부 | 2 | 원문을 직접 읽음 | 2차 요약만 봄 |
| 커뮤니티 검증 | 2 | 널리 인용됨 | 단일 출처 |

## 후보 (13개)

| # | 레퍼런스 | 종류 | 근거 | 적합 | 최신 | 실행 | 검증 | 커뮤 | **총점** |
|---|---|---|---|---|---|---|---|---|---|
| R1 | [Claude Code Best practices (공식)](https://code.claude.com/docs/en/best-practices) | 공식 문서 | 4 | 5 | 5 | 5 | 5 | 5 | **95** |
| R2 | [Claude Code Memory (공식)](https://code.claude.com/docs/en/memory) | 공식 문서 | 4 | 4 | 5 | 5 | 5 | 5 | **90** |
| R3 | [LeRobot `AGENTS.md` + `CLAUDE.md`](https://github.com/huggingface/lerobot) | 실사례 (같은 도메인) | 3 | 5 | 5 | 4 | 5 | 5 | **87** |
| R4 | [claude-md-management 플러그인 (Anthropic 공식)](https://claude.com/ko/marketplace/plugins/claude-md-management) | 도구 | 3 | 4 | 4 | 5 | 3 | 4 | **76** |
| R5 | [Evaluating AGENTS.md — ETH Zurich, arXiv 2602.11988](https://arxiv.org/abs/2602.11988) | 통제 실험 | 5 | 3 | 4 | 3 | 3 | 4 | **75** |
| R6 | [Boris Cherny 팀 운영 방식 (compounding engineering)](https://paddo.dev/blog/how-boris-uses-claude-code) | 실무 사례 | 3 | 4 | 4 | 4 | 2 | 5 | **73** |
| R7 | [AGENTS.md 개방형 표준 (Linux Foundation)](https://codex.danielvaughan.com/2026/04/07/agents-md-open-standard-cross-tool-portability) | 표준 | 3 | 3 | 4 | 4 | 3 | 5 | **70** |
| R8 | [HumanLayer "Writing a good CLAUDE.md"](https://www.humanlayer.dev/blog/writing-a-good-claude-md) | 블로그 (영향력 큼) | 3 | 3 | 3 | 4 | 2 | 5 | **65** |
| R9 | [IFScale — arXiv 2507.11538 (NeurIPS 2025)](https://arxiv.org/abs/2507.11538) | 통제 실험 | 5 | 2 | 3 | 2 | 3 | 4 | **64** |
| R10 | [McMillan 2026 — arXiv 2605.10039 (Alex Dunlop 정리)](https://www.alexdunlop.com/writing/claude-md-best-practices) | 통제 실험 (미검증) | 4 | 3 | 5 | 3 | 1 | 1 | **63** |
| R11 | [DataCamp — Writing the Best CLAUDE.md](https://www.datacamp.com/tutorial/writing-the-best-claude-md) | 튜토리얼 | 2 | 3 | 4 | 4 | 3 | 3 | **61** |
| R12 | [claude-md-templates principles](https://github.com/abhishekray07/claude-md-templates/blob/main/principles.md) | 템플릿 | 2 | 2 | 3 | 4 | 2 | 2 | **49** |
| R13 | [awesome-claude-code](https://claudelog.com/addons/awesome-claude-code) | 큐레이션 목록 | 1 | 2 | 3 | 3 | 1 | 5 | **45** |

> 검증 2점 이하 항목은 클라우드 세션 네트워크 정책 때문에 원문(humanlayer.dev, arxiv.org, alexdunlop.com)을 열지 못하고 2차 요약만 본 것이다.

## 상위 3개에서 얻은 핵심

### R1·R2 공식 문서
- **넣을 것**: Claude가 코드를 읽어서는 알 수 없는 것만 넣는다. 검증 명령(테스트, 린트, 펌웨어 빌드), 오해하기 쉬운 지점, 레포에 없는 결정, **두 번 이상 교정한 것**.
- **뺄 것**: 아키텍처와 파일 구조 설명. Claude가 레포를 직접 읽는다. 공식 문서의 "그만둬도 되는 것" 표에 명시되어 있다.
- **길이**: 파일당 200줄 미만이 목표. `@import`는 정리용일 뿐 컨텍스트 비용은 줄이지 않는다.
- **강조어**: IMPORTANT나 대문자 강조는 쓰지 않는다. 최신 모델은 평이한 문장을 따르고, 강조하면 오히려 규칙을 과하게 적용한다.
- **역할 분담**
  - 여러 단계 절차 → **skill** (`.claude/skills/<name>/SKILL.md`). 필요할 때만 로드된다.
  - 특정 경로에만 해당하는 규칙 → **path-scoped rule** (`.claude/rules/*.md` + `paths:` frontmatter)
  - 예외가 없어야 하는 규칙 → **hook / deny rule**. CLAUDE.md는 "가이드라인"일 뿐이다.
  - 개인 선호 → `CLAUDE.local.md` (gitignore)
- **유지보수 도구**: `/init`, `/doctor`(삭제 후보 제안), `/doctor prompt-audit`(모순, 낡은 지침, 없는 파일 참조 탐지, v2.1.283 이상)

### R3 LeRobot (우리가 가져다 쓸 라이브러리)
- `AGENTS.md`(약 58줄)를 원본으로 두고 `CLAUDE.md`는 그것을 가리키는 심볼릭 링크로 둔다.
- 사용자용 가이드(`AGENT_GUIDE.md`)와 에이전트용 지침(`AGENTS.md`)을 분리한다.
- 구성: 개요 → 기술 스택 → 설정 명령 → 핵심 명령(테스트, 린트) → 모듈 설명 → 주의사항(Notes)
- 시사점: 개발 단계에서 우리 레포가 LeRobot을 포크하거나 확장하면 이 형식을 그대로 따르는 게 자연스럽다.

### R5 ETH 실험 (반대 근거)
- 컨텍스트 파일이 있으면 작업 성공률이 **소폭 떨어지고** 추론 비용은 **20% 넘게** 늘었다.
- 레포 개요는 도움이 되지 않았다. 효과가 있는 건 **비표준 관행**을 명시했을 때뿐이었다.
- R1의 "알 수 없는 것만 넣는다"는 원칙을 실험으로 뒷받침한다.

## 근거들 사이의 충돌
| 쟁점 | A 쪽 | B 쪽 | 판단 |
|---|---|---|---|
| 길이 상한 | HumanLayer: 60줄 이하가 이상적 / 공식: 200줄 미만 | McMillan: 25~500줄 사이에서 준수율 차이 없음 | **공식 기준 200줄**을 상한으로 두고 내용 기준(알 수 없는 것만)으로 줄인다. 길이보다 내용이 중요하다는 점에서 세 근거 모두 일치한다 |
| 명령 개수 한계 | IFScale: 150~200개에서 앞쪽 지시를 더 잘 따르는 편향이 가장 큼 | 이 실험은 키워드 포함 과제라서 코딩 에이전트와 거리가 있음 | 중요한 규칙을 **위쪽**에 둔다 (근거가 약해도 비용이 없음) |
| 누적 vs 초기화 | Cherny 팀: 실수할 때마다 추가 | Cherny 후기 발언: 주기적으로 지우고 다시 쓰기 | 누적하되 **단계 전환마다 prompt-audit**으로 정리 |

## 결정 및 근거 (제안 — 사용자 승인 대기)

| # | 제안 | 근거 |
|---|---|---|
| D1 | 레퍼런스 조사 절차와 평가표를 **프로젝트 skill**(`.claude/skills/reference-research/`)로 옮기고, CLAUDE.md에는 한 줄만 남긴다 | R1: 여러 단계 절차는 skill로. **skill은 레포에 있으므로 계정을 바꿔도 자동으로 따라온다** |
| D2 | 하드웨어 BOM은 `docs/planning/hardware.md`로 옮기고 CLAUDE.md에는 포인터만 남긴다 | R1, R5: 개요와 배경 정보는 매 세션 효과가 없다 |
| D3 | "Gotchas" 섹션을 만들어 두 번 교정한 실수를 쌓는다 | R1, R6 |
| D4 | "(가장 중요)" 같은 강조 표현을 없애고, 중요한 규칙을 위쪽으로 올린다 | R1, R9 |
| D5 | 개발 단계에 들어가면 `.claude/rules/firmware.md`(ESP32), `.claude/rules/python.md`(LeRobot)처럼 경로별 규칙을 둔다 | R2 |
| D6 | 원격 기획 세션에서 코드를 쓰지 않는다는 규칙은 지금은 CLAUDE.md에 둔다. 어기는 일이 반복되면 hook으로 옮긴다 | R1: 예외가 없어야 하는 규칙만 hook으로 |
| D7 | AGENTS.md로 전환하는 건 **보류**한다. Claude 외의 도구(Codex, Cursor)를 쓰게 되면 전환하고, 그때는 Windows 호환을 위해 심볼릭 링크 대신 `@AGENTS.md` import를 쓴다 | R2, R3, R7 |
| D8 | 세션 종료 시 `/revise-claude-md`, 단계 전환 시 `/doctor prompt-audit`를 실행한다 | R2, R4 |
| D9 | 대화 모드(사용자 선호)는 원래 `CLAUDE.local.md`에 둘 내용이지만, **계정 간 이동이 필수 요건**이라 레포 CLAUDE.md에 그대로 둔다 | R1 원칙보다 프로젝트 요건을 우선함 (명시적 예외) |

## 버린 후보와 이유
- R12, R13: 근거가 약하고 일반 템플릿 위주라 우리 상황과 거리가 있다. 예시가 필요할 때만 참고한다.
- R10: 결론은 흥미롭지만 원문을 확인하지 못했다. 결정 근거로 쓰지 않는다.
