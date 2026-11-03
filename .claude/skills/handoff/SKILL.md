---
name: handoff
description: Compact the current conversation into a handoff document for another agent to pick up.
argument-hint: "What will the next session be used for?"
disable-model-invocation: true
---

Write a handoff document summarising the current conversation so a fresh agent can continue the work. Save to the temporary directory of the user's OS (`$TMPDIR`, else `/tmp`; `%TEMP%` on Windows) - not the current workspace.

Include a "suggested skills" section in the document, naming which skills the next agent should call the Skill tool for.

Do not duplicate content already captured in other artifacts (specs, plans, ADRs, issues, commits, diffs). Reference them by path or URL instead.

Redact any sensitive information, such as API keys, passwords, or personally identifiable information.

If the user passed arguments, treat them as a description of what the next session will focus on and tailor the doc accordingly.

## 이 프로젝트에서 (so101-studio)
- 이 문서는 임시 디렉터리에만 두고 커밋하지 않는다. 로컬 세션의 공식 인수인계는 `docs/planning/arm-dev-status.md`, `docs/planning/gui-dev-status.md`이고, 그 문서를 다시 쓰지 말고 경로로 가리킨다.
- 다른 세션에 일을 넘길 때는 `docs/guides/session-workflow.md` 4절(REQ 요청·완료)과 5절(기획 요청) 양식을 그대로 쓴다.
- 한국어로 쓴다.
