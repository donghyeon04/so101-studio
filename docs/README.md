# 문서 색인

새 문서는 아래 네 위치 중 하나에 만들고, 이 표에 한 줄을 추가한다. 이 표에 없는 문서는 두지 않는다.

| 위치 | 유형 | 수명 |
|---|---|---|
| `README.md` | 프로젝트 개요 | 계속 갱신 |
| `docs/planning/` | 기획서, 기능명세서, 메뉴트리, 와이어프레임, 하드웨어 구성, 설계서, 측정 기록 | 같은 파일을 계속 고침 |
| `docs/decisions/` | 결정 기록 (`NNNN-slug.md`) | 수정하지 않고, 새 기록으로 대체 |
| `docs/guides/` | 설치·셋업 방법 | 계속 갱신 |

## 문서 목록

| 경로 | 유형 | 상태 | 설명 |
|---|---|---|---|
| `docs/planning/overview.md` | 기획 | 초안 | 기획서: 한 줄 정의, 일정·평가, 산출물, 역할, 기획 진행 순서, 범위, 주간 일정 |
| `docs/planning/features.md` | 기획 | 초안 | 기능명세서와 메뉴트리, 시연 시나리오, 기능별 범위 층 |
| `docs/planning/system-architecture.md` | 기획 | 확정 | 시스템 구성, 모듈 경계, 공통 타입, 인식 규격, 작업 실행 상태 머신, ESP32 통신 틀, 폴더 구조, 0~2단계 작업 순서 |
| `docs/planning/arm-software-spec.md` | 기획 | 확정 | 팔 모듈 계층 구조, 데이터 타입, 모듈 API, 설정·출력 형식, 구현 작업 순서(T1~T11) |
| `docs/planning/measurements.md` | 기획 | 초안 | 로컬 개발 세션의 측정 기록 양식 (T0~T7) |
| `docs/planning/arm-dev-status.md` | 기획 | 초안 | 팔 개발 현황: 한 일(T0~T11, C1, S1~S3), 설계서와 다른 점, 사용법, 남은 일, GUI 연결, 저장소 상태 |
| `docs/planning/hardware.md` | 기획 | 초안 | 부품 구성(BOM), 보유 현황, 예상 비용, 전기적 주의사항 |
| `docs/planning/comparison-experiment.md` | 기획 | 초안 | 초음파·카메라 인식 비교 실험의 통제 변인, 평가 기준, 환경 조건 가설 |
| `docs/guides/arm-hardware-setup.md` | 가이드 | 확정 | 팔 하드웨어 구성, COM 포트, 서보 ID 1~6, 캘리브레이션 파일 복사, 다른 PC에서 처음 연결하는 순서, 문제 해결 |
| `docs/guides/tooling-setup.md` | 가이드 | 확정 | 계정마다 설치할 플러그인, MCP, 스킬 |
| `docs/decisions/0001-claude-md-authoring.md` | 결정 | 확정 | CLAUDE.md는 짧게, 절차는 skill로 |
| `docs/decisions/0002-docs-structure.md` | 결정 | 확정 | 문서 유형 4종, 색인, 상태값, 조사 원문 비보관 |
| `docs/decisions/0003-arm-control-stack.md` | 결정 | 확정 | LeRobot SOFollower + placo IK, MoveJ/L/P는 얇게 직접 구현 |
| `docs/decisions/0004-perception-arm-interface.md` | 결정 | 확정 | 인식과 팔을 공통 인터페이스(x, y, 각도, 폭, 길이, 신뢰도)로 분리 |
| `docs/decisions/0005-development-order.md` | 결정 | 확정 | 팔 먼저(구동 점검 → 동작 라이브러리 → 작업 공간), 센서는 그 뒤 |
| `docs/decisions/0006-device-setup.md` | 결정 | 확정 | Windows PC 1대, ESP32 1개 + HC-SR04 2개, 서보·카메라는 추후 구매 |
| `docs/decisions/0007-branch-and-dev-workflow.md` | 결정 | 확정 | 기획 브랜치를 main에 합치고 개발은 dev/ 브랜치, 측정값·설계 변경의 되돌림 경로 |
| `docs/decisions/0008-single-jaw-grasp-compensation.md` | 결정 | 대체됨(→ 0009) | 고정 턱 1개 그리퍼는 턱 중심 보정과 닫기 전 가동 턱 쪽 이동으로 소프트웨어에서 보정 |
| `docs/decisions/0009-gripper-geometry-and-jaw-yaw.md` | 결정 | 확정 | 턱 방향은 URDF 기하로 정하고(고정 턱 면이 기준점, 가동 턱 방향 m), 집기 각도는 물체 각도 + 90° |
| `docs/decisions/0010-centered-closing-grasp.md` | 결정 | 확정 | 최대로 벌려 중심을 맞추고 닫으며 손을 같이 보내 닫는 내내 중심을 유지, 접촉에서 정지 |
| `docs/decisions/0011-tilted-symmetric-closing.md` | 결정 | 확정 | 턱 끝 기하로 중간점을 물체에 맞추고, 턱이 앞뒤로 벌어질 때 손을 기울여 V자를 대칭으로, 닫을수록 푼다 |
| `docs/decisions/0012-symmetric-v-tilt.md` | 결정 | 확정 | 기울기를 벌림 각도의 절반으로 해 보기에 대칭인 V자로(손목 한계 35°), 대칭 그리퍼 교체와 물체 교정은 하지 않음 |
| `docs/decisions/0013-coordinate-frames-and-keys-modes.md` | 결정 | 확정 | 좌표계는 월드(작업대)·베이스(로봇)·툴, 높이는 작업대 면 기준, keys는 관절/직교 조그/좌표 입력 세 모드 |
| `docs/decisions/0014-servo-acceleration-and-elbow-limit.md` | 결정 | 확정 | 팔 서보 가속 한계 10(그리퍼 25 유지), 제어 주기 50Hz 유지, 팔꿈치 접는 한계 95·home 93, 전원은 병목 아님 |
