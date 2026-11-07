# 0045. 테스트·검증 방식: 느린 테스트 표시, 테스트 감사, PR 리뷰, ruff 훅, 화면 확인

상태: 확정(1번의 "합치기 전에는 전체" → 0046) · 2026-11-07 (사용자가 로컬 총괄과 정함, `researching-references`로 후보 16·17개 비교)

## 맥락
전체 pytest가 약 40분이고 GUI 테스트는 고정 포트를 써서 세션 하나만 돌릴 수 있다. 테스트가 허위이거나 약하지 않은지, PR을 눈으로만 검토해 놓치는 것이 없는지 사용자가 물었다.

## 결정
1. `--durations`로 느린 테스트를 재고 `@pytest.mark.slow`를 붙인다. 개발 중에는 `-m "not slow"`, 합치기 전에는 전체를 돌린다. 각 세션이 자기 영역 테스트에 붙인다.
2. Anthropic 공식 `pr-review-toolkit`의 `pr-test-analyzer`·`silent-failure-hunter`로 기존 테스트와 예외 처리를 감사하고, 고칠 것은 사용자가 고른다.
3. 총괄은 PR을 합치기 전에 `code-review`(medium 이상)를 돌려 결과를 작성 세션에 보낸다.
4. 프로젝트 훅(`.claude/settings.json`): 파일을 고치면 그 `.py` 파일만 ruff로 검사한다. pytest는 훅에 넣지 않는다(40분, 포트 충돌).
5. GUI를 바꾼 PR은 Playwright 스크린샷으로 확인하고, 완료 보고에는 실행 명령과 결과를 붙인다.
- 하지 않는 것: xdist 병렬·포트 분리, Cosmic Ray, testmon, pytest를 자동 실행하는 MCP·플러그인(필요해지면 다시 본다).
