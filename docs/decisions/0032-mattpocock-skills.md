# 0032. mattpocock/skills는 맞는 11개만 레포에 넣고 이 프로젝트 규칙에 맞게 고친다

상태: 확정 · 2026-11-04

## 맥락
사용자가 mattpocock/skills(MIT, Claude Code 플러그인)를 앞으로의 작업에 넣고 싶어 했다. 원문(README와 SKILL.md)을 확인해 보니 일부 스킬이 이 프로젝트 규칙과 충돌한다. `setup`·`grill-with-docs`·`domain-modeling`은 루트 `GLOSSARY.md`·`docs/adr/`를 만들고 ADR을 고치며, `research`는 조사 원문을 레포에 저장한다. `tdd`·`diagnosing-bugs`·`code-review`·`implement`는 Superpowers와 겹치고, `triage`·`to-tickets`·`to-spec`·`wayfinder`는 GitHub Issues 기반이다.

## 결정
- 플러그인으로 전부 설치하지 않는다. 원본 커밋 `4ae2317`에서 11개만 `.claude/skills/`로 가져온다: `grill-me`, `grilling`, `handoff`, `wait-what`, `writing-for-agents`, `to-questionnaire`, `improve-codebase-architecture`, `codebase-design`, `prototype`, `pr`, `retro`. 라이선스는 `.claude/skills/MATTPOCOCK-LICENSE.txt`.
- 가져올 때 고친 것: `GLOSSARY.md`·ADR 대신 `docs/planning/` 용어와 `docs/decisions/`를 가리킨다. 결정 기록은 기획 세션만 쓴다. 인수인계는 `session-workflow.md` 양식을 쓴다. 산출물(질문지, 아키텍처 보고서, 인수인계)은 임시 디렉터리에 두고, PR 형식은 0007을 따르며, 한국어로 쓴다. 각 SKILL.md 끝의 "이 프로젝트에서" 절에 모았다.
- 판단 요청에는 지금처럼 한 번에 한 질문(소크라테스 모드)으로 묻는다. 사용자가 `/grill-me`를 부르면 여러 질문을 추천 답과 함께 라운드로 묻는다(사용자가 시험해 보기로 함).
- 원본이 바뀌어도 자동으로 따라가지 않는다. 다시 가져올 때는 이 결정의 고친 점을 다시 적용한다.
