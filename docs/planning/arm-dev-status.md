# 팔 개발 현황과 인수인계 (로컬 개발 세션 → 기획 세션)

상태: 초안

작성: 2026-11-01. 브랜치 `dev/arm-t0-t7`. 측정 원본은 `measurements.md`, 결정은 `docs/decisions/0008~0019`, 하드웨어 연결은 `docs/guides/arm-hardware-setup.md`.

## 1. 한 줄 요약
SO-101 팔을 실제로 구동해 작업 순서표의 T0~T11, C1, S1~S3을 구현했다. 키보드·CLI로 이동, 직교·좌표 이동, 집기·놓기, 토크를 끈 손 교시(속도 구간 포함), 물체 없는 자동 작업을 실제 팔로 시연했다. 센서(초음파)·카메라·GUI·시뮬레이션 화면은 아직 시작하지 않았고, 물체를 실제로 집는 성공률과 위치 정확도의 줄자 실측도 못 했다.

## 2. 지금 상태
시뮬레이션 테스트 142개(`python -m pytest -q`, 약 5분)와 린트(`ruff`) 통과.

| 구분 | 항목 |
|---|---|
| 실제 팔로 확인함 | 구동·영점·방향 매핑, 이동(`movej`, `goto`, `home`), 안전 검사 동작, 집기·놓기 동작(물체 없이), `run --dry-run` 자동 작업, 키보드 관절·직교 조그, `record`/`replay`(안전 구간 재생, `--unsafe` 전체 재생, 한계 거부), `powertest`, `shaketest`, `servoinfo`, `linkwatch`, 서보 가속 한계 비교, 팔꿈치 한계 시험 |
| 시뮬레이션만 거침 | 좌표 입력 모드(`x y 높이`), 툴 기준 조그, `tcp tip/ref` 높이, 안전 설정(`--box`, `--safety-margin`, `keys`의 `safety`), 새 `record`(홈 이동·카운트다운·표본 시각·구간 분석·구간별 속도 재생), 과전류 감시 |
| 새 설정(가속 10, 팔꿈치 95, 홈 93)으로 아직 안 한 것 | `pick`/`place`와 `run --dry-run`의 접촉 감지 재확인(그리퍼는 가속 25 유지) |
| 못 한 것 | `table_z_mm`(−25, 육안 추정) 실측, 위치 정확도 줄자 실측(T6), 실물 집기 성공률(T7), 큰 벌림 폭 표 보정 |

## 3. 구현된 것 (작업 순서표 기준)
| 작업 | 내용 |
|---|---|
| T0~T2 | 모터 ID(1~6, 이미 설정됨), 영점 보정, 환경(conda `so101`, Python 3.12, LeRobot 0.6.1), `ArmDriver`/`SimDriver`, `status` |
| T3 | FK/IK (LeRobot `RobotKinematics` + placo 0.9.25), 방향 매핑, FK 원점을 어깨 회전축으로 보정 |
| T4~T5 | `move_j`, `MotionController`(정지·비상정지), `safety.py`(관절 한계, 작업 영역, 바닥, 한 주기 이동량, 과열·과부하·통신 끊김·과전류) |
| T6 | `move_l`, `goto`, 수직 하강 도달 영역. 줄자 실측은 못 함 |
| T7 | `skills`: home, park, pick, place. 고정 턱 1개 그리퍼 보정(결정 0008~0012) |
| T8 | 원격 조종: 옛 `jog`와 새 `keys`(관절/직교 조그/좌표 입력, 결정 0013) |
| T9 | 녹화·재생(`record`/`replay`/`seq`/`info`), 토크를 끈 방식(결정 0018·0019) |
| T10 | `ArmService`, `runs/`(events.jsonl, status.csv) 기록. CLI와 이후 GUI가 모두 이것을 거침 |
| T11 | `retarget` (Ruckig). 이동 중 목표를 바꿔도 끊기지 않음 |
| C1, S1~S3 | core, `ManualSource`, 작업 상태 머신, `Studio`, `run` |

## 4. 이 세션에서 한 일 (시간순)
1. **환경·하드웨어 설정**: Miniforge 환경, pip의 placo 설치 실패 → conda-forge로 해결, 캘리브레이션(4번째 시도에서 공식 영상 자세로 성공), 모터 ID 확인, 첫 구동에서 그리퍼가 작업대에 닿아 과부하 → 안전 검사 도입.
2. **기구학·동작**: FK/IK, 궤적(quintic, Ruckig 선택), `move_l`, 직교 이동, 안전 모듈.
3. **집기 보정**: 그리퍼가 고정 턱 1개라 한쪽으로 벌어짐 → 턱 기하(URDF) 기반 중심 맞춤, 대칭 V자 기울기, 닫는 내내 중심 유지, 접촉은 명령-실측 지연으로 감지(결정 0008~0012). 레퍼런스 조사로 LeRobot 도구(draccus 설정, `ensure_safe_goal_position`)와 Ruckig 도입.
4. **T8~T11, 자동 작업**: 원격 조종, 녹화·재생, `ArmService`, `retarget`, `ManualSource`/상태 머신/`run`, 런처 `so101.cmd`.
5. **키보드 모드와 좌표계**: 관절/직교 조그/좌표 입력 3모드(Tab 전환), 좌표계 table(world)/robot(base)/tool, 높이는 작업대 면 위, 끝점 tip/ref(결정 0013). 조그 속도 램프, 시간 기준 적분, 막힘 이유 표시.
6. **떨림 조사**: 조원 제안서(S-Curve 중심)를 코드와 대조 → 이미 5차 다항식을 쓰고 있음을 확인. 가속 한계·속도·제어 주기·전원을 측정으로 비교해 서보 가속 10, 50Hz 유지로 결정(결정 0014). 진단 명령 `powertest`, `shaketest`, `servoinfo`, `keys --trace` 추가.
7. **전원 사건**: 서보 6개가 동시에 토크를 잃는 일이 세 번 있었고, 한 번은 서보 전체가 응답하지 않았다. 12V 어댑터를 뽑아 불이 꺼진 뒤 다시 꽂자 복구되어 과전류 보호나 접촉 불량으로 추정한다. 과전류 감시(4A 연속 2회 멈춤), `status.csv`에 전압·전류 열, `linkwatch`(응답 끊김 감시)를 추가했다.
8. **안전 설정**: normal/free/off 모드, 관절 여유(안전율), XYZ 허용 범위, 한계 밖에서 빠져나오기, 경로 0.1초 검사(결정 0015). 레퍼런스(UR 안전 평면·복구 모드, Franka 가상 벽, MoveIt 소프트 한계)와 대응됨을 확인.
9. **직접 교시**: 토크를 켠 채 손으로 미는 추종 교시를 여섯 번 시험(토크 제한, P·D, 중력 보정, 방향 보조) → 모두 기어 마찰로 뻑뻑해 제거. 레퍼런스 26개 조사 결과 같은 서보 팔로워에서의 선례가 없고, LeRobot은 기어비가 낮은 리더를 따로 둔다. 토크를 끈 `record`로 확정(결정 0016~0018). 이어서 기록 설정 파라미터화와 속도 구간 분석·구간별 배율 재생(결정 0019).
10. **문서**: 하드웨어 연결 가이드, 결정 기록 0008~0019, 이 인수인계 문서, 측정 기록.

## 5. 현재 설정값 (`config/robot.yaml`)
| 항목 | 값 | 근거 |
|---|---|---|
| 통신 | COM3, 1Mbps, 서보 ID 1~6 | `arm-hardware-setup.md` |
| 서보 가속 한계 | 팔 10, 그리퍼 25 | 결정 0014. 그리퍼는 접촉 감지를 25에서 맞춰 따로 둠 |
| PID | P16 I0 D32 | 변경 시험에서 기본이 가장 나았음 |
| 제어 주기 | 50Hz | 100·200Hz와 0.1~0.2초 간격은 더 떨림 |
| 팔꿈치 한계 / 홈·파크 | 위쪽 95 / 팔꿈치 93 (`[0,-90,93,0,0,3]`) | 96이 J2 모터에 닿기 직전, 서보가 0.4° 더 감 |
| 영점 보정 | shoulder_pan −3.0° | |
| 작업대 높이 `table_z_mm` / 바닥 한계 `tip_floor_mm` | −25 / 5 | 육안 추정, **실측 필요** |
| 속도 | 기본 30%, 조그 관절 30°/s·직교 80mm/s | |
| 기록 | `record_hz` 20, `record_max_s` 90, 구간 기준 3·40°/s, 최소 0.5초 | 결정 0019 |
| 과전류 | 4A 연속 2회 → fault | 어댑터 5A 보호가 걸리기 전에 멈춤 |

## 6. 설계서(arm-software-spec.md)와 다르거나 설계서에 없는 것 (기획 세션에서 반영)
- 홈·파크 자세와 팔꿈치 한계(위 표), `table_z_mm`·`tip_floor_mm` 높이 기준, 서보 가속 한계 설정.
- 궤적: 기본 quintic에 Ruckig 선택(`trajectory_profile`)과 T11 `retarget`. 설정 읽기는 draccus, 한 주기 이동량 제한은 LeRobot `ensure_safe_goal_position`.
- `ToolPose.yaw`는 턱이 벌어지는 축의 방향이고, 집기 각도는 물체 각도 + 90°. 높이 z는 작업대 면 위(좌표 입력 기준).
- `ArmService`에 추가된 것: `point_pose`, `preview_tool`, `get_safety`/`set_safety`, `record_info`, `replay(..., until_limit, segment_speeds)`, `record_start(hz, max_s)`. `status`에 전류(`current_ma`) 추가, `status.csv`에 `voltage_v`, `current_a` 열.
- CLI: `keys`, `info`, `powertest`, `shaketest`, `servoinfo`, `linkwatch`, 안전 옵션(`--free`, `--unsafe`, `--safety-margin`, `--box`), `replay --until-limit --seg-speed`, `--hz`, `--speeds`, `--trace`.
- 안전: 모드 normal/free/off, 과전류 감시, 한계 밖에서 빠져나오기, `physical_limits_deg`(캘리브레이션 실제 범위).

## 7. 사용법
레포 밖에서는 `C:\Users\c\Desktop\so101-studio\so101.cmd`로 부른다(conda 활성화 불필요). 실제 팔은 클램프로 고정하고 속도 30% 이하, 전원 스위치 옆에서 지켜본다. 자세한 명령은 `CLAUDE.md`.
- 이동: `status`, `home`, `movej`, `goto`, `pick x y 각도 폭`, `place 구역`, `gripper`, 자동 작업 `run --source manual [--dry-run] [--step]`.
- 조종: `keys`(Tab으로 관절 → 직교 조그 → 좌표 입력. 좌표: `x y 높이 [yaw]`, `d dx dy dz`, `frame table|robot|tool`, `tcp tip|ref`, `safety ...`).
- 교시: `record <이름>`(홈 이동 → 팔 잡기 확인 → 3초 → 토크 꺼짐 → Enter로 끝), `info <이름>`(길이·속도·구간), `replay <이름> [--seg-speed 1=0.5,3=2] [--until-limit]`.
- 진단: `powertest`(전압·전류·부하), `shaketest`(가속·속도별 떨림), `servoinfo`, `linkwatch`(응답 끊김 위치 찾기).

## 8. GUI와 시뮬레이션에 붙이는 방법
GUI는 `ArmService`만 호출한다.
- 상태: `status()`, `status_stream(callback)`(10Hz).
- 이동: `move_joints`, `move_tool`, `home`, `park`, `pick_and_place`, `place`, `set_gripper`, `jog`, `retarget`, `stop`, `estop`, `set_speed`.
- 점 찍기: `point_pose(x, y, 높이, yaw, frame, tcp)` → `preview_tool(pose)`(움직이지 않고 가능 여부·이유·풀린 관절값) → `move_tool(pose)`.
- 안전율: `get_safety()` / `set_safety(mode, joint_margin_deg, box_mm)`(결정 0015).
- 교시: `record_start(hz, max_s)`, `record_stop(name)`, `record_info(name)`(구간 목록), `replay(name, speed, until_limit, segment_speeds)`.
- 시뮬레이션: URDF와 `SimDriver`가 있어 FK로 화면을 그릴 수 있다. 사용자는 시뮬레이션에서 GUI로 점을 찍는 것까지 가고 싶어 한다. 기술 선택은 미정.

## 9. 주요 조사·측정 결과
- **떨림**: 느리고 서보 가속이 높을수록 체감 떨림이 크다(254/20°/s가 가장 심함). 가속 10이 20·60·200°/s 모두 가장 낫거나 같다. 서보에 목표를 보내는 간격은 50Hz가 최선이고 5~10Hz도 100·200Hz도 더 떨린다. 남은 떨림은 느린 키보드 조그에서 체감될 정도(서보 내부 설정이나 기구 하중 의심).
- **전원(12V 5A)**: 거친 동작(최대 200°/s, 팔을 끝까지 폄)에서 전압 12.2 → 11.8V, 전류 합 최대 1.7A(5A의 34%), 어깨 들기 부하 100%가 0.25초. 병목이라는 증거는 없다. 그러나 서보 전체가 꺼지는 사건이 세 번 있어 전원 계통 점검이 필요하다.
- **직접 교시**: 교시 중 서보 부하는 작고 뻑뻑함은 1/345 기어 마찰이다. 정지 부하는 중력보다 접근 방향·마찰에 좌우되어 중력 모델과 맞지 않았다. 같은 서보에서 추종·중력 보상을 구현한 사례는 레퍼런스 26개에서 찾지 못했다.
- **녹화·재생**: 표본 시각을 저장하므로 빠른·느린 구간이 평균이 아니라 그대로 재생된다. 격한 45초 녹화(910점)는 한계·작업대 밖·바닥을 넘어 기본 재생이 거부되고, `--unsafe`로는 151초 동안 끝까지 정상 재생됐다. 끝 자세 오차는 최대 약 2.7°(팔꿈치).
- **안전 설정**: 관절 여유·XYZ 범위·모드·복구 규칙은 UR, Franka, MoveIt, Kinova 문서와 대응된다. 빠진 것은 속도 상한을 안전 설정에 넣는 것(MoveIt의 속도 스케일링 계수)이다.

## 10. 알려진 문제와 주의
- **전원/연결**: 서보 6개가 동시에 토크를 잃거나 응답이 사라지는 일이 반복됐다. 12V 어댑터를 뽑고 불이 꺼진 뒤 다시 꽂으면 복구됐다. 커넥터 접촉이나 어댑터 보호 회로가 의심된다. `linkwatch`를 켜 둔 채 커넥터를 흔들어 위치를 찾는다.
- **토크 꺼짐**: 전원이 끊기면 팔이 받침으로 떨어진다. 팔을 받치고 시작하고, 클램프로 고정한다.
- **`table_z_mm` 추정값**: 높이 0 근처, 바닥 안전 검사의 정확도가 이 값에 달려 있다. 종이 한 장이 끼일 때까지 내려서 재는 절차가 필요하다(자 필요).
- **그리퍼**: 앞 다리가 제대로 맞물리지 않아 물체를 못 집는다. 교체는 URDF와 설계를 다시 해야 해 이번 범위에서 제외(조원 합의).
- 키보드 `keys`는 창과 상관없이 키를 읽는다. 실행 중 다른 창에 타이핑하지 않는다.

## 11. 남은 일 (우선순위)
1. 전원 계통 점검(커넥터 재압착, 다른 12V 5A 이상 어댑터로 비교) — 안전과 직결.
2. `table_z_mm` 실측과 위치 정확도 줄자 실측(T6).
3. 새 설정으로 `pick`/`place`/`run --dry-run` 재시험(접촉 감지), 좌표 입력·툴 조그·새 `record` 실기 시험.
4. 실물 집기 성공률(T7)과 큰 벌림 폭 표 보정. 그리퍼 개선(조원 재설계).
5. 안전 설정에 속도 상한(`max_speed_pct`) 추가.
6. 서보 시동 힘(`Minimum_Startup_Force`) 시험(느린 조그 떨림).
7. GUI가 쓸 API: `session-requests.md`의 REQ-001~008(기획이 미리 등록, 모두 기존 동작을 바꾸지 않는 추가). GUI는 GUI 세션이 따로 만든다(`decisions/0024`).
8. 초음파(3단계), 카메라(4단계).

## 12. 기획 세션에 요청할 결정·반영
- 설계서(`arm-software-spec.md`)에 6절 항목 반영(홈 자세, 관절 한계, API 추가).
- 센서 연결 순서. (GUI 기술과 시뮬레이션 범위는 결정 0023·0024와 `gui-spec.md`로 정해짐)
- 전원 하드웨어(어댑터 교체) 여부, 그리퍼 개선 일정(조원), 발표 자료에 쓸 영상·측정 정리.

## 13. 저장소 상태
GitHub `donghyeon04/so101-studio`. `main`은 기획 문서 1차 병합(#1)까지이고, 개발은 `dev/arm-t0-t7` 브랜치에서 했다(커밋 약 100개). 결정 기록 0008~0019와 문서 색인은 `docs/README.md`.
