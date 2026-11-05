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
| `docs/planning/project-status.md` | 기획 | 초안 | 프로젝트 현황 한 장(팀 공유·인수인계): 부분별 된 것과 남은 것, 가상 성능, 팀과 논의할 것, 일정 |
| `docs/planning/features.md` | 기획 | 초안 | 기능명세서와 메뉴트리, 시연 시나리오, 기능별 범위 층 |
| `docs/planning/system-architecture.md` | 기획 | 확정 | 시스템 구성, 모듈 경계, 공통 타입, 인식 규격, 작업 실행 상태 머신, ESP32 통신 틀, 폴더 구조, 0~2단계 작업 순서 |
| `docs/planning/arm-software-spec.md` | 기획 | 확정 | 팔 모듈 계층 구조, 데이터 타입, 모듈 API, 설정·출력 형식, 구현 작업 순서(T1~T11) |
| `docs/planning/measurements.md` | 기획 | 초안 | 로컬 개발 세션의 측정 기록 (T0~T11, 떨림·전원·그리퍼 시험) |
| `docs/planning/arm-dev-status.md` | 기획 | 초안 | 팔 개발 현황과 인수인계: 상태, 구현 목록, 이 세션에서 한 일, 현재 설정값, 설계서와 다른 점, 사용법, GUI 연결 API, 조사·측정 결과, 알려진 문제, 남은 일 |
| `docs/planning/gui-spec.md` | 기획 | 확정 | GUI 설계서(1차 기본 골자): 원칙, 공통 틀, 13개 페이지, 디자인 기본값, 범위, 폴더 구조, 실행 구조, 작업 순서 G0~G10 |
| `docs/planning/gui-dev-status.md` | 기획 | 초안 | GUI 개발 현황과 인수인계: 환경, 작업 진행(G0~G10), 보낸 요청 |
| `docs/planning/us-dev-status.md` | 기획 | 초안 | 초음파 개발 현황과 인수인계: 작업 진행(U0~U9), 측정, 보낸 요청 (초음파 세션이 처음 만든다) |
| `docs/planning/session-requests.md` | 기획 | 초안 | 세션 간 요청 대장 (REQ 번호, 보냄·받음, 상태). 받은 세션이 갱신 |
| `docs/planning/hardware.md` | 기획 | 초안 | 부품 구성(BOM), 보유 현황, 예상 비용, 전기적 주의사항 |
| `docs/planning/comparison-experiment.md` | 기획 | 초안 | 초음파·카메라 인식 비교 실험의 통제 변인, 평가 기준, 환경 조건 가설 |
| `docs/planning/ultrasonic-spec.md` | 기획 | 확정 | 초음파 인식 설계서: 레이더 + 팔 확인, 합격 기준, 하드웨어 치수·배선·서보 전원(A 충전기/B 12V 공유)·구매 목록·브래킷 레퍼런스, 스캔 서보 추천, 시간 줄이기 기법 T1~T11, 레이더·확인 처리, ESP32 통신 규약, PC 구조, 작업 순서 U0~U9, 시험 |
| `docs/guides/arm-hardware-setup.md` | 가이드 | 확정 | 팔 하드웨어 구성, COM 포트, 서보 ID 1~6, 캘리브레이션 파일 복사, 다른 PC에서 처음 연결하는 순서, 문제 해결 |
| `docs/guides/tooling-setup.md` | 가이드 | 확정 | 계정마다 설치할 플러그인, MCP, 스킬 |
| `docs/guides/session-workflow.md` | 가이드 | 확정 | 세션 역할(로컬 셋, GUI 세션 총괄), worktree와 브랜치, REQ 요청·완료 양식, 기획 요청, 포트 같이 쓰기, 질문과 승인은 총괄로, 새 계정·새 PC로 옮길 때 |
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
| `docs/decisions/0015-safety-settings.md` | 결정 | 확정 | 안전 검사를 실행 중 바꿀 수 있는 설정으로(normal/free/off, 관절 여유, XYZ 범위), GUI는 get_safety/set_safety |
| `docs/decisions/0016-follow-teach.md` | 결정 | 대체됨(→ 0017) | 직접 교시는 토크 제한을 낮춘 추종 방식(밀리면 목표가 따라감), 1단계 고정 프로필, 중력 모델은 2단계 |
| `docs/decisions/0017-teach-with-torque-off.md` | 결정 | 대체됨(→ 0018) | 직접 교시는 토크를 끈 `record`가 기본, 추종 교시와 중력 보정은 기어 마찰 때문에 실험 기능으로만 둠 |
| `docs/decisions/0018-teach-torque-off-reference-flow.md` | 결정 | 확정 | 직접 교시는 레퍼런스의 토크 끈 흐름(홈 이동, 잡기 확인, 카운트다운, 20Hz, 90초, 실제 시각 저장), 추종 교시 코드 제거 |
| `docs/decisions/0019-record-segments-and-parameters.md` | 결정 | 확정 | 기록 주기·최대 시간·구간 기준을 설정과 파라미터로, 기록에 속도 요약 저장, 구간별 속도 배율 재생(GUI용 record_info/segment_speeds) |
| `docs/decisions/0020-arm-phase1-freeze.md` | 결정 | 확정 | 팔 1차 완료 = API 동결, 하드웨어 실측·전원·그리퍼는 시뮬레이션·GUI와 병행 |
| `docs/decisions/0021-gui-viser-kinematic-twin.md` | 결정 | 대체됨(→ 0023) | GUI는 Viser 기반 Python 단일 프로세스(`so101 gui`), 실제·미리보기 팔 두 개, 시뮬레이션은 기구학 쌍둥이(MuJoCo는 보류), 연결 끊김 시 stop |
| `docs/decisions/0022-gui-design-tooling.md` | 결정 | 대체됨(→ 0025) | GUI 디자인은 기획에서 Excalidraw 와이어프레임, 개발에서 Context7·Playwright MCP·design 플러그인·frontend-design으로 스크린샷 반복 |
| `docs/decisions/0023-app-shell-nicegui-viser.md` | 결정 | 확정 | GUI는 완성형 앱: NiceGUI 앱 틀 + Viser 3D, 센서별 화면 등록(초음파 2D 평면, 카메라는 나중), 실행 방식 설정, 조작권 한 곳 |
| `docs/decisions/0024-session-roles-and-requests.md` | 결정 | 확정 | 로컬 개발을 GUI 세션과 팔 세션으로 나누고(worktree), 상대 영역은 요청서(REQ)로 주고받음, 설계 변경은 기획으로 |
| `docs/decisions/0025-gui-screen-check-and-design-refs.md` | 결정 | 확정 | GUI 화면 확인은 앱 내장 브라우저(예비 Playwright CLI), 기준 문서는 NiceGUI llms.md, 상태 표시는 Carbon 규칙, DESIGN.md 없음 (0022 대체) |
| `docs/decisions/0026-estop-watchdog-process.md` | 결정 | 확정 | GUI E-STOP은 별도 감시 프로세스: 브라우저·서버 heartbeat가 1초 끊기면 정지 명령 → 0.3초 무응답이면 팔 프로세스 종료, 단축키 E-STOP, 감시 없으면 실제 팔 명령 막음 |
| `docs/decisions/0027-arm-profiles-and-local-config.md` | 결정 | 확정 | 설정 세 겹(robot.yaml ← 보드 시리얼로 고르는 arms/<팔>.yaml ← git 제외 local.yaml), 포트 자동 찾기, 캘리브레이션 불일치면 읽기만, 읽기 전용 so101 doctor, environment.yml과 Windows·Linux 런처 |
| `docs/decisions/0028-auto-calibration.md` | 결정 | 확정 | 자동 캘리브레이션은 LeRobot PR #4826 엔진을 가져와 측정만 모드부터(EEPROM 불변, 기존 값과 비교), 실물 합격 뒤 --save(.bak 보관), 이 팔 범위로 계획 수정 |
| `docs/decisions/0029-autocal-acceptance-repeatability.md` | 결정 | 확정 | 기존 값이 없거나 다른 팔은 측정 3회 반복(차이 1° 이하, 중앙값 저장, 기존 파일 있으면 yes 확인)으로 합격, 저장 뒤 0° 자세 확인 전에는 잠금 (0028 보완) |
| `docs/decisions/0030-blackbox-logging.md` | 결정 | 확정 | 상태·이벤트 기록은 JSON Lines 블랙박스: 공통 머리, 토크·fault·레지스터 쓰기 기록, ARM-TORQUE-LOST, 기록 스레드, 10초 링 버퍼와 사고 묶음, 30일·2GB 보관, 성능 기준 |
| `docs/decisions/0031-safety-xy-return-and-comm-loss-test.md` | 결정 | 확정 | 작업대 XY 검사에 밖→안 복귀 허용, 작업대는 실측으로(임의 확대 안 함), table_z·손끝은 접촉 측정, 포트 빠짐 4경우 안전 시험, 토크 풀림은 status 값으로 판단 (0015 보완) |
| `docs/decisions/0032-mattpocock-skills.md` | 결정 | 확정 | mattpocock/skills는 11개만 `.claude/skills/`에 넣고 프로젝트 규칙에 맞게 고침(용어·결정 위치, 임시 디렉터리 산출물, 한국어), `/grill-me`는 여러 질문 라운드 |
| `docs/decisions/0033-gripper-hold-preload-and-place-compensation.md` | 결정 | 확정 | 그리퍼 접촉 뒤 예압으로 잡기, 조임 한계는 RAM Torque_Limit(닫기 300·들기 200·놓은 뒤 복원), no_object면 올라와서 열기, 그리퍼 부하 2단계 감시, 놓기 중심 보정 (0010~0012 보완) |
| `docs/decisions/0034-ultrasonic-radar-plus-arm-check.md` | 결정 | 확정 | 초음파 인식은 앞 변 가운데 레이더(서보 1개)로 후보를 찾고 팔 손목 센서가 십자·원형으로 확인하는 2단계, 작업 순서에 CHECK, 시간 줄이기 기법을 설정으로, 정확·빠른 모드(→ 0038로 하나) (0006 보완) |
| `docs/decisions/0035-scan-servo-and-power.md` | 결정 | 확정 | 스캔 서보 MG90S 2개(예비 1), 서보 전원은 기존 12V 5A를 Y 케이블 + 12V→5V 강압 모듈로, 브래킷은 모두 출력(손목은 SO-ARM100 손목 카메라 마운트 개조, 레이더는 공개 받침 + 새 클램프) (0034 보완) |
| `docs/decisions/0036-three-local-sessions-with-hub.md` | 결정 | 확정 | 로컬 세션을 GUI(Opus, 총괄)·팔(Sonnet)·초음파(Sonnet) 셋으로, 총괄이 일 나누기·요청 대장·Sonnet PR 검토, 진행 상황은 문서에 (0024 보완) |
| `docs/decisions/0037-single-shared-calculator.md` | 결정 | 확정 | 검증용 계산기는 주제마다 하나만, 레포 안 실제 코드 위에(초음파는 8.5절 벤치마크, 도달은 팔 세션 계산), 아이디어는 옵션으로, 기획은 가정을 정하고 검토, 설계서 숫자는 고정 시나리오 결과만 |
| `docs/decisions/0038-single-check-mode.md` | 결정 | 확정 | 초음파 확인 모드는 하나(빠른 모드 없앰, 벤치마크에서 시간 -3~+4%), T8은 옵션 `check.approach_line` |
