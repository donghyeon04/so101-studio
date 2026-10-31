# 문서 색인

새 문서는 아래 네 위치 중 하나에 만들고, 이 표에 한 줄을 추가한다. 이 표에 없는 문서는 두지 않는다.

| 위치 | 유형 | 수명 |
|---|---|---|
| `README.md` | 프로젝트 개요 | 계속 갱신 |
| `docs/planning/` | 기획서, 기능명세서, 메뉴트리, 와이어프레임, 하드웨어 구성 | 같은 파일을 계속 고침 |
| `docs/decisions/` | 결정 기록 (`NNNN-slug.md`) | 수정하지 않고, 새 기록으로 대체 |
| `docs/guides/` | 설치·셋업 방법 | 계속 갱신 |

## 문서 목록

| 경로 | 유형 | 상태 | 설명 |
|---|---|---|---|
| `docs/planning/hardware.md` | 기획 | 초안 | 부품 구성(BOM), 보유 현황, 예상 비용, 전기적 주의사항 |
| `docs/planning/comparison-experiment.md` | 기획 | 초안 | 초음파·카메라 인식 비교 실험의 통제 변인, 평가 기준, 환경 조건 가설 |
| `docs/guides/tooling-setup.md` | 가이드 | 확정 | 계정마다 설치할 플러그인, MCP, 스킬 |
| `docs/decisions/0001-claude-md-authoring.md` | 결정 | 확정 | CLAUDE.md는 짧게, 절차는 skill로 |
| `docs/decisions/0002-docs-structure.md` | 결정 | 확정 | 문서 유형 4종, 색인, 상태값, 조사 원문 비보관 |
| `docs/decisions/0003-arm-control-stack.md` | 결정 | 확정 | LeRobot SOFollower + placo IK, MoveJ/L/P는 얇게 직접 구현 |
| `docs/decisions/0004-perception-arm-interface.md` | 결정 | 확정 | 인식과 팔을 공통 인터페이스(x, y, 각도, 폭, 길이, 신뢰도)로 분리 |
| `docs/decisions/0005-development-order.md` | 결정 | 확정 | 팔 먼저(구동 점검 → 동작 라이브러리 → 작업 공간), 센서는 그 뒤 |
| `docs/decisions/0006-device-setup.md` | 결정 | 확정 | Windows PC 1대, ESP32 1개 + HC-SR04 2개, 서보·카메라는 추후 구매 |
