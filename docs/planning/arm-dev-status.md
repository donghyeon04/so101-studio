# 팔 개발 현황과 인수인계 (로컬 개발 세션 → 기획 세션)

상태: 초안

작성: 2026-11-01. 브랜치 `dev/arm-t0-t7`. 측정 원본은 `measurements.md`, 결정은 `docs/decisions/0008~0019`, 하드웨어 연결은 `docs/guides/arm-hardware-setup.md`.

## 1. 한 줄 요약
SO-101 팔의 동작 라이브러리(이동, 직교·좌표 이동, 집기·놓기, 교시·재생, 안전 검사, 자동 작업)와 서비스 API, 블랙박스 기록, 통신·그리퍼 안전 보완이 구현돼 `--sim`과 실제 팔에서 확인됐다. GUI는 GUI 세션이 따로 만들고(`decisions/0024`), 초음파 인식은 초음파 세션이 만든다(`decisions/0036`, `ultrasonic-spec.md`). 팔 쪽 남은 일은 실물 측정이고, 순서는 `measurements.md`의 "실물 시험 순서"를 따른다.

## 2. 지금 상태
시뮬레이션 테스트와 린트(`ruff`)는 `python -m pytest -q`(약 16분)와 `python -m ruff check src tests`로 확인한다. 실제 팔은 사용자가 있을 때만 쓰고 포트(COM3)는 총괄(GUI 세션)이 조율한다(`session-workflow.md` 6·7절).

| 구분 | 항목 |
|---|---|
| 실제 팔로 확인함 | 구동·영점·방향 매핑, 이동·집기·놓기(물체 없이), 안전 검사, `run --dry-run`, 키보드 조그, 교시·재생, 진단 명령(`powertest`, `shaketest`, `servoinfo`, `linkwatch`), G10(E-STOP 세 경우, 포트 빠짐, 재연결) |
| `--sim`만 거침 | 블랙박스 기록(15절), 통신 끊김·XY 복귀 안전(16절), 그리퍼 예압·부하 감시(17절), 오프라인 준비(18절), 과부하 추가 읽기(19절), `follow_path`·팔 링크 점·summary.json(20절) |
| 실물 측정 대기 | REQ-019 `table_z_mm` 접촉·격자·포트 빠짐 네 경우, REQ-020 집기·놓기 5회와 부하 기준, REQ-018 실제 팔 기록 성능, PR #41(자동 캘리브레이션 반복·`--verify-zero`)은 G10 뒤에 합친다 |

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
실물 시험의 순서와 포트 사용은 `measurements.md`의 "실물 시험 순서"가 기준이다. 아래는 팔 세션 몫만 적는다.
1. 실물 시험 3~6번: PR #41 합치기와 `--verify-zero`, `table_z_mm` 접촉·격자(16절), 집기·놓기 5회와 그리퍼 부하 기준(17절), 기록 성능(15절).
2. `table_z_mm`·`joint_offsets_deg`/URDF의 팔꿈치 약 5° 불일치·실물 집기 성공률(T7): 기획 쪽 결정이 필요한 것은 총괄을 통해 [기획 요청]으로 보낸다.
3. REQ-025(작업 실행 `CHECK` 상태, `run --source ultrasonic`)는 초음파 세션의 U2가 합쳐진 뒤 총괄이 보낸다.
4. 안전 설정에 속도 상한(`max_speed_pct`), 서보 시동 힘(`Minimum_Startup_Force`) 시험(느린 조그 떨림)은 우선순위가 낮다.

## 12. 기획 세션에 요청할 결정·반영
총괄(GUI 세션)이 [기획 요청](`session-workflow.md` 5절)으로 모아 사용자에게 준다. 지금 올라간 것: 확인 경로 도달 범위(`measurements.md` "확인 경로 도달 계산"), 속도 180mm/s와 `tool_v_max_mm_s`(결정됨: 설정은 그대로, 실제 속도 = min(요청, 설정 × 속도%)).

## 13. 저장소 상태
GitHub `donghyeon04/so101-studio`, 개발은 `main`에서 딴 `dev/<작업>` 브랜치와 PR로 합친다(`decisions/0007`). 요청 대장 `session-requests.md`는 총괄이 관리한다(팔 세션은 상태 변화를 총괄에게 알린다).

## 14. GUI용 추가 API 사용법 (REQ, 기존 동작은 바꾸지 않음)
기획 세션이 설계서에 옮긴다. 대장은 `session-requests.md`.
- **REQ-001 `Studio.step()`**: `st.start(step_mode=True)` 뒤 `st.step()`을 부를 때마다 한 상태씩 진행한다(어느 스레드에서든). `st.stop()`은 대기 중인 단계도 풀어 준다. CLI `--step`(Enter)은 `task.wait_step`을 바꿔 쓰므로 그대로다.
- **REQ-002 `Studio.perception`**: `list()`(이름 목록), `active`, `select(name)`(작업 중이면 RuntimeError, 모르는 이름은 ValueError), `calibrate()`, `scan()`(ScanResult), `register(source)`(초음파·카메라 모듈이 생기면 등록).
- **REQ-003 `ManualSource.add(obj)` / `remove(id)`**: 다음 `scan()`부터 반영, Lock으로 스레드 안전. id가 겹치면 ValueError, 없는 id를 지우면 KeyError.
- **REQ-007 작업 이벤트 상세**: `studio.bus.subscribe("task", cb)`로 `TaskEvent`를 받는다(콜백은 작업 스레드에서 불린다). `DECIDE`의 `detail`에 `detail_n`, `scan_s`, `objects`(id, x_mm, y_mm, angle_deg, width_mm, radial_mm, reachable), `chosen`, `reason`이 들어 있다. 결과가 있는 `PICK`/`PLACE` 이벤트에는 `duration_s`(실제로 실행한 동작 시간)가 있다.
- **REQ-005 `so101 gui`**: `so101.cmd [--sim] [--speed N] gui`가 `so101_studio.gui.main(args)`(argparse Namespace 하나)를 부른다. 팔은 연결하지 않으니 `gui.main`이 직접 `ArmService`를 만든다. GUI 모듈이 없으면 안내 후 종료, 의존성(nicegui·viser)이 없으면 `pip install -e .[gui]` 안내.
- **REQ-008 `ArmService.preview_path(pose, speed_pct=None, dt=0.05)`**: 움직이지 않고 `move_tool(pose)`와 같은 계획의 경로를 돌려준다. `{"ok", "reason"(안전 검사에 걸린 이유 | "ik_fail" | None), "duration_s", "joints": [{"t", "q"}], "tool": [{"t", "x_mm", "y_mm", "z_mm"}]}`. 좌표는 작업대 mm, z는 `move_tool`과 같은 모델 z(작업대 면 위 높이가 아니다). `pose`는 `point_pose`로 만든다.
- **REQ-004 파라미터·프리셋**: `ArmService.get_params()` -> `{이름: {value, default, timing}}`(`speed_pct` 포함, timing은 now | next_motion | reconnect). `set_params({이름: 값})`은 모두 검증한 뒤 적용하고 `{이름: 적용 시점}`을 돌려준다(하나라도 틀리면 ValueError, 아무것도 안 바뀜). `save_preset(name)`, `load_preset(name)`, `list_presets()`는 `config/presets/<이름>.yaml`을 쓴다. 다루는 값은 `RobotConfig`의 숫자·문자열 설정이다.
- **REQ-006 진단 함수**: `ArmService.run_diagnostic(name, progress=None, stop=None, **kw)` -> `{"name", "result", "aborted"}`(블로킹, GUI는 스레드에서). name은 `servoinfo | linkwatch | powertest | shaketest`. `progress(text)`는 진행 문장, `stop()`이 True면 다음 이동 전에 시작 자세로 돌아와 설정을 복원하고 끝난다. kw: powertest(repeats, speed_pct), shaketest(reps, accels, speeds), linkwatch(period_s). `powertest`·`shaketest`는 팔을 크게 움직인다(`diagnostics.BIG_MOVES`, GUI는 확인 창). `linkwatch`는 포트를 직접 열어 시뮬레이션이나 팔 연결 중에는 ValueError.

- **REQ-009 `ArmService.resume()`**: `stop()`·`estop()` 뒤(state `stopped`)와 `fault` 뒤에 상태를 `idle`로 되돌려 다시 움직일 수 있게 한다. 팔은 움직이지 않고 이벤트 `resume`을 기록한다. 인자 없음, 블로킹 아님.
- **REQ-011 E-STOP 감시 프로세스**: `python -m so101_studio.watchdog [--port 8090] [--hotkey F9] [--sim]`(127.0.0.1, 표준 라이브러리 HTTP, CORS 허용). `GET/POST /hb?src=browser|server`, `POST /arm {armed, pid, stop_port}`(등록·해제, 작동한 뒤에는 다시 `/arm`이 올 때까지 작동 상태 유지), `POST /estop`, `GET /status`(`{alive, armed, sim, tripped, reason, age_s}`). armed이고 browser나 server heartbeat가 1.0초 넘게 끊기면 ① UDP로 `{"cmd":"estop","reason"}`를 `stop_port`에 보내고 ② 0.3초 안에 `{"ack":"estop"}`가 없으면 `pid`를 끝낸다(Windows는 TerminateProcess). `--sim`이거나 armed가 아니면 기록만 한다. 단축키는 Windows `GetAsyncKeyState` 50Hz 폴링(기본 F9, `robot.yaml`의 `watchdog_hotkey`나 `--hotkey`)이고 Windows가 아니면 꺼진다. 팔 프로세스 쪽은 `service/stop_listener.py`의 `StopListener(arm_service, port=8091).start()/close()`: UDP를 받아 `arm_service.estop()`(토크 유지)을 부르고 ack로 답한다. GUI는 시작할 때 `StopListener`를 켜고 감시 프로세스에 `/arm`을 보낸다.
- **REQ-012 `ArmService.jog` 토크 켜기**: 시그니처는 그대로(`jog(mode, velocity) -> bool`). 처음 부를 때 토크가 꺼져 있으면 켜고, 그 뒤에는 버스를 읽지 않아 10Hz로 불러도 된다. 교시(`record_start`)로 토크가 꺼지면 다음 `jog`가 다시 확인한다. 전원 끊김처럼 밖에서 토크가 꺼진 경우는 알 수 없어서 `move_*`를 한 번 부르거나 다시 연결한다.
- **REQ-013 `Studio.pause()` / `resume()`**: `pause()`는 작업 중 어느 스레드에서든 불러 다음 상태 변화(이벤트 처리 직후)에서 멈추게 한다(단계별 모드로 바꾼다). `step()`으로 한 단계씩, `resume()`으로 단계별 모드를 풀고 끝까지 이어 간다(기다리는 중이면 풀어 준다). `start()`는 시작할 때 남은 step 신호를 지운다. 이전에는 `stop()` 뒤 `start(step_mode=True)`의 첫 단계가 기다리지 않고 지나갔다. CLI `--step`은 `wait_step`을 바꿔 쓰므로 영향 없다.
  - REQ-013 추가분: 단계별 모드에서 `stop()`을 부르면 대기가 풀리고 이후에도 기다리지 않아 작업 스레드가 끝난다(`result.end == "STOPPED"`). 작업 스레드(`start(blocking=False)`)에서 예외가 나면 이제 `FAULT` 이벤트(`result="exception"`, `detail.error`)를 내고 `result = {..., "end": "FAULT", "error": ...}`로 남긴다. `blocking=True`는 예전처럼 예외를 호출한 쪽에 던진다.
- **REQ-014 `MotionController.close()`**: 진행 중인 동작이 있으면 그 `MotionHandle`을 `stopped`로 끝낸다. 인자·반환값은 그대로다. 이전에는 제어 루프만 끄고 `wait()`하던 스레드가 영원히 남았다(`ArmService.disconnect`도 같은 효과).
- **틱 지연과 `step` 안전 정지(사용자 결정 A+D)**: 시뮬레이션 자동 작업이 약 30회 중 2회 `step` 안전 정지(FAULT)로 끊겼다. `MotionController._tick`이 벽시계로 궤적을 샘플링해, 한 틱이 늦으면 위치가 한 번에 건너뛰어 주기당 5° 한도에 걸린 것으로 보인다. 지금은 한 틱에 궤적 시간이 `MAX_TICK_ADVANCE`(2틱) 넘게 진행하지 않고, 늦은 시간만큼 동작이 늦어진다(안전 한도는 그대로). 누적 지연은 `MotionController.time_lost_s`로 볼 수 있다(크면 제어 주기가 밀리는 중이다). 실제 팔에서는 G10에서 `time_lost_s`와 FAULT 여부를 먼저 확인한다. GUI 시험은 시뮬레이션 속도를 낮춰 부하를 줄이는 쪽으로 피한다.
- **REQ-015 다른 PC·다른 팔 브링업** (`decisions/0027`, 설계서 7.1절):
  - **설정 세 겹**: `core.config.load_robot(path, arm=None, serials=(), ...)`가 `robot.yaml` ← `config/arms/<팔>.yaml` ← `config/local.yaml` 순으로 겹쳐 읽는다(키 단위로 통째로 바꿈, 폴더와 파일은 `robot.yaml` 옆에서 찾는다). `load_robot_ex`는 `(설정, {arm, reason, files, local})`을 돌려준다. `local.yaml`이 없으면 이전과 같다(옮긴 값이 팔 파일로 갔을 뿐이고 합친 값이 같은 것을 확인했다). 새 필드 `arm_name`, `board_serial`, `measured_on`. 팔 고르기에 실패하면 `ArmSelectionError(candidates)`. `config/arms/_template.yaml`은 팔이 아니다.
  - **팔 파일로 옮긴 값**: `calibration_id`, `joint_offsets_deg`, `joint_limits_deg`, `physical_limits_deg`, `table_z_mm`, `gripper_width_mm`, `grasp_bias_mm`(그리고 `board_serial`, `measured_on`). `robot.yaml`의 `port: COM3`은 포트 목록을 못 읽을 때의 마지막 대안으로 남겼다.
  - **포트 고르기**: `core.ports.pick_port(ports, board_serial, explicit, local_port, fallback)`. 등록된 보드가 안 보이면 다른 보드를 추측해 쓰지 않고 `PortChoiceError(candidates)`를 낸다. 블루투스 가상 COM 등 USB 시리얼 칩이 아닌 포트는 후보가 아니다. CLI는 시뮬레이션이 아닌 명령에서 이 순서로 고르고 `팔 ..., 포트 ...`를 한 줄 출력한다(`gui` 명령은 고르지 않는다).
  - **잠금**: 서보 캘리브레이션이 lerobot 캘리브레이션 파일과 다르거나(`is_calibrated`가 거짓. 새 PC에 파일이 없는 경우 포함) 팔 파일이 없으면 `ArmDriver.connect`는 서보에 아무것도 쓰지 않고 연결만 한다(`driver.locked_reason`). `ArmService.locked_reason`(문장 또는 None)이 있으면 `execute`·`retarget`은 `MotionResult(status="locked")`, `jog`는 False, `record_start`는 RuntimeError, `driver.write_joints`는 RuntimeError이고 `set_torque(True)`는 무시된다. 읽기(`status`, `get_safety`, 상태 스트림)는 된다. 시뮬레이션은 잠그지 않는다. GUI는 `svc.locked_reason`으로 배지를 띄운다.
  - **`so101 doctor [--json]`**: `doctor.doctor(cfg=None, probe=True, port=None, progress=None, config_path, arm=None) -> dict`(블로킹, GUI는 스레드에서). `{"ok": bool(실패 항목 없음), "checks": [{"name","state": "ok|warn|fail|skip","text","fix"}], "ports": [{"device","vid","pid","serial","description","role": "arm|esp32|unknown|busy","detail"}], "arm": None|{"port","serial","servos": [{"id","model","voltage_v","temp_c","torque","status","pos"}],"calibration_id","calibration_found","calibration_mismatch": [문장]}, "esp32": None}`. `--json`은 같은 dict를 ASCII 이스케이프 JSON으로 출력하고 실패가 있으면 종료 코드 1이다. 점검 항목은 환경, numpy(conda 설치본), lerobot, placo, pyserial, ruckig, pyyaml, GUI 의존성, 포트 목록, 팔 파일(고른 이유), 캘리브레이션 파일, 서보 응답, 전압(리더 팔 7.4V면 주의), 온도, 캘리브레이션 대조다. `probe=False`는 포트를 열지 않는다. 서보가 응답하는 포트가 둘 이상이면 고르지 않고 `--port`를 안내한다. 안전: `packet()`이 PING(0x01)·READ(0x02) 외의 명령 코드를 거부하고(테스트), 포트는 DTR/RTS를 내린 채 열고(테스트), USB 시리얼 칩이 아닌 포트와 ESP32(VID 303A)는 열지 않는다.
  - **`so101 find-port`**: USB를 뽑아 사라진 포트를 찾고 `local.yaml`의 `port:`와 팔 파일의 `board_serial:`에 적을 값을 안내한다.
  - **이 PC 보드 확인(2026-11-03, 포트를 열지 않고 `list_ports`로만)**: CH343 `1A86:55D3`이 USB 시리얼 번호 `5B8E114816`을 준다. `arms/so101_follower_01.yaml`의 `board_serial`에 넣었다.
  - **설치**: `environment.yml`(python 3.12, conda-forge numpy 2.2.6 MKL, placo 0.9.25, pip lerobot[feetech] 0.6.1과 `-e .[gui]`). `so101.cmd`는 고정 경로 대신 환경 이름 `SO101_ENV`(기본 so101)나 폴더 `SO101_ENV_DIR`로 찾는다. `so101.sh`는 Linux용이고 dialout 안내를 띄운다. `keys`·`jog`는 Windows 전용이라 다른 OS에서는 안내하고 끝난다. `pyserial`을 직접 의존성에 넣었다.
  - **새 팔 순서**: 캘리브레이션은 자동(`so101 calibrate --auto --measure-only` → 합격 뒤 `--save`, REQ-016) 또는 수동 `lerobot-calibrate --robot.id=<id>` 중 하나로 한다. 앞서 `lerobot-setup-motors`로 모터 ID를 정한다(모터에 쓰고 움직이므로 doctor는 대신 하지 않는다). 그다음 `so101 doctor`로 보드 시리얼을 확인하고 `arms/_template.yaml`을 복사해 채운다.
  - **확인하지 못한 것**: 실제 팔에서 `doctor`의 탐침(서보 PING/READ)은 이 PC에서 포트를 열지 않았다(사용자가 집에서 지시할 때 한다). 가짜 시리얼 테스트만 했다. `so101.sh`, `conda env create -f environment.yml`, lerobot 없는 PC에서의 `doctor`도 실행해 보지 못했다(이 PC에는 lerobot이 있다). `--sim`은 아직 lerobot을 import하므로 lerobot 없는 PC에서는 `doctor`와 `find-port`만 돈다.
- **REQ-016 자동 캘리브레이션** (`decisions/0028`, 설계서 11.1절). 엔진은 LeRobot PR #4826(Apache-2.0)을 가져와 쓴다. 코드 작업은 끝났고 실물 시험(AC4)은 사용자와 함께 한다.
  - **가져온 것**: `arm/autocal/engine.py`(원본 `motors/feetech/auto_calibration.py`), `plan.py`(`robots/so_follower/auto_calibration_plan.py`), 테스트 `tests/arm/autocal/test_engine.py`(원본 `tests/motors/test_feetech_auto_calibration.py`). 출처는 stepankandrushin/lerobot 브랜치 `auto-calibration`의 커밋 `d55915e`이고 각 파일 머리말에 Apache-2.0 표기와 "Adapted from huggingface/lerobot#4826"이 있다. LeRobot이 합치면 이 사본을 지우고 upstream을 쓴다. **엔진 동작은 바꾸지 않았다.** 바꾼 곳은 import 두 줄(`lerobot.motors...`), 계획의 관절 허용 범위(shoulder_pan 230~245°, gripper 118~152°, 원본은 185~205°·128~152°), 시험의 가짜 팔 범위(pan 239°, gripper 125°와 그에 맞춘 기대 문구)다. 0.6.1의 버스 API(`sync_read`·`sync_write`·`read`·`write`의 `normalize`·`num_retry`, `disable_torque`, `write_calibration`, 레지스터 이름)는 그대로 맞아 고친 것이 없다. 원본 시험 중 PR이 LeRobot 자체에 더한 기능을 쓰는 3개(`SO101Follower.auto_calibrate`로 저장, `lerobot-calibrate --auto` 옵션과 `--help`)는 0.6.1에 없어 뺐다. 나머지 48개가 통과한다.
  - **`so101 calibrate --auto --measure-only [--velocity 200] [--torque-limit 500]`**: 안내(시작 자세, 클램프, 반경 50cm, 전원 스위치 옆)를 띄우고 Enter를 받은 뒤 측정한다. `ArmService`와 `ArmDriver.connect`를 거치지 않고 LeRobot 로봇의 버스만 연다(connect는 PID·가속과 EEPROM을 쓴다). 실행 중 변경은 엔진이 `Lock=1`로 RAM에만 남기고, 끝나면(성공·실패·Ctrl+C) 이전 `Homing_Offset`·`Min/Max_Position_Limit`와 `Lock`을 RAM에 되돌리고 다시 읽어 확인한다(실패하면 전원을 껐다 켜라고 안내하고 오류). 시작 전과 실행 중 0.25초마다 전압(11.0V 미만)과 온도(60°C 초과)를 보고 넘으면 즉시 중단하고 되돌린다(`guard.py`가 엔진이 읽는 버스를 감싼다. 엔진은 고치지 않았다). 기존 `config/calibration/<id>.json`(LeRobot 캐시를 먼저, 없으면 레포 사본)과 관절별 범위(°)·0° 위치 차이(°)를 표로 출력하고 `runs/<날짜-시각>-autocal/measure.json`에 남긴다. 범위와 0° 위치 차이가 모두 ±3° 이내면 합격(`passed`)이다. 비교할 기존 파일이 없으면 합격이 아니다. 종료 코드는 합격 0, 그 밖 1, 사용법 오류 2. `--sim`에서는 쓸 수 없다.
  - **`so101 calibrate --save [--from <measure.json>]`**: 팔을 움직이지 않는다. 기본은 같은 `calibration_id`의 가장 최근 `runs/*/measure.json`(24시간 이내)이고 `passed`가 아니면 거부한다. 순서: 이전 파일을 `config/calibration/<id>.<YYYYmmdd-HHMM>.bak.json`으로 남김 → 토크를 끄고 `write_calibration`으로 EEPROM에 쓰기 → 다시 읽어 확인(어긋나면 이전 값으로 되돌리고 파일은 쓰지 않음) → LeRobot 캐시와 `config/calibration/<id>.json`에 쓰기 → 기록 `runs/autocal-save-<id>.json`. 팔 파일의 `physical_limits_deg`는 새 범위의 ±절반을 제안으로만 출력한다. 저장 뒤 `doctor`의 캘리브레이션 대조를 다시 하고 정상이어야 종료 코드 0이다.
  - **doctor 연동**: `doctor()` 결과에 키 `autocal`(위 저장 기록 또는 None)과 점검 항목 `자동 캘리브레이션`이 더해졌다. GUI는 이 키를 무시해도 된다.
  - **시험 방법(가짜 서보)**: `tests/arm/autocal/test_measure_save.py`가 EEPROM을 흉내 내는 가짜 팔로 측정만 모드의 EEPROM·RAM 불변, 비교표, 전압·온도 중단과 되돌리기, 되돌리기 실패, Ctrl+C, `--save`의 `.bak.json`·EEPROM 확인·롤백·거부를 확인한다. 실제 팔은 열지 않았다.
  - **열린 점**: 새 팔(기존 캘리브레이션 파일이 없는 팔)은 비교할 값이 없어 합격이 될 수 없으므로 `--save`로 저장할 수 없다. 새 팔의 첫 자동 캘리브레이션을 어떻게 승인할지는 기획에서 정해야 한다.
- **연결이 토크를 건드리지 않게 (2026-11-04, GUI 세션의 사고 보고)**: 팔이 0° 자세(곧게 뻗음)에서 GUI를 실제 팔로 켜자 토크가 풀려 팔이 주저앉았다. 원인은 확인하지 못했다(실물 재현 안 함). 연결 코드에서 토크를 끌 수 있던 유일한 곳은 `ArmDriver.connect`의 `bus.disable_torque()`(토크가 꺼져 있다고 읽을 때만)였다. 고친 것: (1) 토크 끄는 쓰기를 없앴다. 꺼져 있을 때도 EEPROM에 쓰려고 `Lock`만 푼다. (2) 토크가 꺼져 있다는 읽기는 한 번 더 읽어 확인한다(한 번 잘못 읽은 0을 믿지 않는다). (3) PID·가속·그리퍼 EEPROM 값은 이미 같으면 쓰지 않는다(뻗은 팔을 잡고 있는 서보 설정을 괜히 건드리지 않는다). (4) 연결 이벤트(`runs/*/events.jsonl`의 `connect`)에 `torque_on_at_connect`와 실제로 쓴 항목(`writes`)을 남긴다. 다음에 같은 일이 생기면 이 기록과 `ctl.fault`로 원인을 좁힌다. 가설(미확인): 팔이 뻗은 자세에서 서보 보호(과부하·과전류)가 토크를 끊었을 수도 있다. 실물 시험은 접힌 자세에서 시작하고 사람이 팔 아래를 받친 채 시작하는 것을 권한다.


## 15. 블랙박스 기록 읽기·쓰기 API (REQ-018, 결정 0030)
- **쓰는 쪽**: `ArmService(..., proc="cli"|"gui")`(없으면 환경변수 `SO101_PROC`, 기본 cli). 연결 전에 `runs/날짜-시각/`을 열어 `events.jsonl`, `status.jsonl`(10Hz, 연결 직후 첫 줄)을 쓴다. 모든 줄은 `{t(ns), proc, pid, seq, level, code, kind, data}`. GUI는 `svc.log_event(kind, data, level, code)`로 조작권·실행 방식·안전 설정 변경을 남기고, `svc.create_incident(reason, last_s, code="MANUAL")`(폴더 경로)로 사고 묶음을 만든다. 감시 프로세스는 `python -m so101_studio.watchdog --run-dir <svc.logger.dir>`로 같은 폴더의 `watchdog.jsonl`에 쓴다.
- **읽는 쪽**(`so101_studio.core.logread`): `list_runs(root)`, `read_events(run_dir)`, `read_status(run_dir)`(나눈 파일과 옛 `status.csv` 포함), `merge_timeline(root, start_ns, end_ns)`(여러 프로세스를 t로 합침), `list_incidents(root)`, `read_incident(path)`. CSV 내보내기는 `core.logging.status_to_csv(run_dir)`.
- **자동 기록**: connect/disconnect, 토크 켜기·끄기(부른 함수 포함, `ARM-TORQUE-ON/OFF`), 레지스터 쓰기(Goal_Position 제외, `ARM-REG-WRITE`), fault(`ARM-FAULT`), E-STOP, `WATCHDOG-TRIP`, 상태 루프 예외(첫 발생은 전체 traceback, 반복은 횟수), 처리 안 된 예외, `ARM-TORQUE-LOST`(2Hz로 읽은 Torque_Enable이 기대와 다름).
- **사고 묶음** `runs/incidents/<시각>-<코드>/`: ring.jsonl(50Hz 틱 10초), timeline.jsonl(앞뒤 60초), 설정 사본과 sha256, doctor.json, meta.json(git 커밋·패키지 버전). fault/E-STOP/감시 작동/TORQUE-LOST/예외마다 자동(같은 코드는 10초 안에 한 번). 자동으로 지우지 않는다. `so101 incident [--last N]`은 이미 쓰인 기록으로만 만든다(다른 프로세스의 링 버퍼는 못 읽음).
- **보관**: 시작할 때 30일·2GB를 넘으면 오래된 실행부터 지움(incidents 제외), status.jsonl은 50MB마다 `status.001.jsonl`로 나눔. 상태 줄은 큐가 차면 버리고 다음 줄의 `data.dropped`에 수를 남긴다.
- **zip**: 사고 묶음은 폴더 하나라 GUI가 `shutil.make_archive`로 묶는다(API 없음).

## 16. 안전 보완 (REQ-019, 결정 0031)
- **작업대 XY 복귀**: `Safety._workspace(q, ref)`가 출발점(ref)이 작업대 x·y 밖이면 축마다 "벗어난 거리가 출발점보다 늘지 않는" 점만 허용한다(ref가 안이면 예전과 같다). 0° 자세처럼 밖에서 `home`으로 돌아올 수 있다. 작업대 크기·팔 받침 위치는 T6 실측 전까지 `workspace.yaml`을 바꾸지 않고, 0° 자세 시험은 안전 모드 free로 한다.
- **통신 끊김**: 끊김 시간은 마지막으로 통신이 된 시각부터 0.5초로 잰다(`HealthMonitor.comm_ok(now)`). 정지 중에는 10Hz 상태 루프의 읽기 결과를 `MotionController.note_comm(ok)`로 알려 끊김을 보고, 컨트롤러 틱이 0.5초가 차는 즉시 `fault("comm_lost")`를 낸다(USB를 뽑은 뒤 약 0.5초). 이때 `ARM-FAULT` 대신 `ARM-COMM-LOST`(ERROR) 이벤트와 같은 이름의 사고 묶음을 남긴다.
- **fault 뒤**: 조그 입력을 비우고, 마지막 위치 유지 쓰기도 하지 않는다(통신 끊김일 때). `ArmService.home/park/move_joints/move_tool/pick_and_place/place/set_gripper`는 버스를 읽거나 쓰지 않고 `MotionResult("fault")`를 돌려준다. `resume()`은 통신 끊김 fault에서는 `False`(이전에는 None)를 돌려주고 풀지 않는다. 다시 꽂아도 `connect()`를 해야 풀린다.
- **다시 연결**: `ArmService.connect()`가 이미 연결된 상태면 먼저 정리한다. `driver.disconnect()`는 토크를 끄지 않고, 포트가 이미 사라져 예외가 나도 정리를 끝낸다. 새 컨트롤러는 현재 위치를 명령값으로 삼는다.
- **읽기만 실패하는 통신(REQ-026)**: 쓰기는 되고 읽기만 응답이 없을 때(서보 하나가 빠지거나 버스 잡음), 동작이 끝나는 틱에서 `_final_tool`이 읽다 예외를 내면 job은 이미 비워져 handle이 끝나지 못해 `wait()`가 영원히 안 돌아오던 문제를 고쳤다. `MotionController._result`는 이제 읽기에 실패해도 결과를 돌려주고(정상 끝이면 `status="fault"`, 손끝 위치는 0), 끊김 시계를 시작해 0.5초 안팎에 `comm_lost` fault로 이어지게 하고, `ARM-FINAL-READ-FAILED`(WARN)를 남긴다. `stop()`·`retarget`·`execute`의 결과 계산도 같은 함수를 써서 예외를 내지 않는다. 시험: `tests/test_comm_hang.py`.
- **실물 측정**(table_z_mm 접촉 4곳, 인쇄 격자 3점, 포트 빠짐 4경우, 토크 풀림 재현)은 사용자와 COM3으로 한다. 아직 안 했다.

## 17. 그리퍼 예압·조임 한계·부하 감시·놓기 보정 (REQ-020, 결정 0033)
- **조임 한계는 RAM**: `ArmDriver.set_gripper_torque_limit(v)`가 RAM `Torque_Limit`(48번)에 쓴다(EEPROM `Max_Torque_Limit`에는 쓰지 않는다). `gripper_torque_limit()`으로 읽는다. 닫는 동안 `gripper_squeeze_torque`(300), 들고 옮기는 동안 `gripper_hold_torque`(200), `place`가 끝나면(예외가 나도) 바꾸기 전 값으로 되돌린다. 접촉이 없었거나 닫기가 실패하면 바로 되돌린다. place를 부르지 않으면 200과 예압 고정이 남는다(다음 pick·place가 이어받는다).
- **예압**: 접촉하면 `ctl.hold_gripper(실제 - grip_preload_units)`로 그리퍼 목표를 고정하고, 이후 궤적이 무엇을 주든 매 틱 이 값으로 덮는다(위로 올라가고 옮기는 동안 명령이 실제보다 많이 닫히지 않는다). 부호는 닫는 쪽(값이 작아지는 쪽)이다(기획 확인됨, arm-software-spec.md 6.6절). 그리퍼를 여는 명령(`set_gripper`로 지금보다 연 값, place의 열기)은 예압 고정을 풀고 조임 한계를 원래 값으로 되돌린다. `stop`·`estop`은 고정과 200을 유지한다(E-STOP이 쓰는 목표도 예압 값).
- **no_object**: 접근 높이까지 올라온 뒤 `gripper_open`으로 연다(바닥에서는 열지 않는다). 결과는 `no_object`.
- **그리퍼 부하 두 단계**: `MotionController._check_grip`(2Hz 건강 읽기에서). `gripper_load_warn_pct`(60%)를 `gripper_load_s`(1초) 넘게 넘으면 목표를 지금 위치로 맞추고 `ARM-GRIP-OVERLOAD-WARN`(WARN), 1초 더 이어지면 그리퍼를 열고 `fault("grip_overload")`와 `ARM-GRIP-OVERLOAD`(ERROR). 기준은 실측으로 정한다(정상 집기 5회 최대값 x 1.3).
- **놓기 보정**: `Skills._opening_path()`가 `_closing_path`를 반대로(지금 값 -> `gripper_open`) 써서 열리는 동안 두 턱 끝의 중간점이 제자리에 머물게 손을 같이 보낸다. IK가 안 풀리면 예전처럼 제자리에서 연다.
- **설정**(robot.yaml): `grip_preload_units` 2, `gripper_hold_torque` 200, `gripper_load_warn_pct` 60, `gripper_load_s` 1.0.
- 실물 확인(집기 5회: 그리퍼 부하 최대값·과부하 오류 유무, 놓기 5회: 물체 밀림)은 사용자와 COM3으로 따로 한다. 아직 안 했다.

## 18. 오프라인 준비 (REQ-021, GUI 세션 요청)
- **끊긴 뒤 E-STOP**: `MotionController.estop()`이 버스를 못 읽어도 예외 없이 끝나고 `ARM-ESTOP`(data.comm_error)을 남긴다(`StopListener`도 정상 응답한다). 통신 끊김 fault(`comm_lost`)면 `estop`·`stop` 뒤에도 state는 fault로 남고 `resume()`은 False다. 끊긴 직후 아직 fault 전에 E-STOP을 하면 stopped가 되지만, 상태 루프가 끊김을 보고 곧 fault로 바꾼다. 다시 연결해야만 풀린다(결정 0031 보완 구현).
- **`ArmService.connect`**: `self.kin`이 있으면 다시 만들지 않는다(URDF 로딩 1.7초 멈춤 제거). 연결된 채 `connect`를 부르면 먼저 정리한다. 연결한 포트는 `svc.port`.
- **`MotionResult.reason`**: out_of_bounds일 때 안전 검사 이유(`workspace`, `z_floor`, `joint_limit:<관절>` 등). 다른 상태는 None.
- **`run_diagnostic(svc, "linkwatch", port=...)`**: 포트를 주지 않으면 마지막으로 연결한 포트(`svc.port`)를 쓰고, 없으면 `cfg.port`.

## 19. 과부하 추가 읽기와 감시 기록 폴더 갱신 (REQ-022)
- **`ARM-LOAD-PROBE`**: 상태 루프가 어느 관절의 부하가 50% 이상을 읽거나(`why: high_load`) 읽기 오류를 낼 때(`why: status_loop_error`), 2Hz 주기를 기다리지 않고 그 순간 `read_health()`를 한 번 더 읽어 `torque`, `status_bits`, `load_pct`, `current_ma_by_joint`를 WARN 이벤트로 남긴다(1초에 한 번). 읽기 자체가 실패하면 그 오류를 남긴다. 서보 보호로 출력이 끊겨도 `Torque_Enable` 값만으로는 안 보일 수 있어, 그 순간의 Status 비트를 잡으려는 것이다. 제어 틱 안에는 읽기를 넣지 않는다.
- **감시 기록 폴더**: `POST /arm` 본문에 `run_dir`(문자열)를 함께 주면 감시 프로세스가 `watchdog.jsonl`을 그 실행 폴더로 옮긴다(`WATCHDOG-RUN_DIR` 이벤트). 처음 `--run-dir`(없으면 `runs/`)와 같은 부모 바로 아래 폴더만 받고, 아니면 400이며 등록도 하지 않는다(브라우저가 부를 수 있는 입구로 임의 경로에 파일을 만들지 못하게). GUI의 `WatchdogLink`는 팔을 연결할 때마다 `svc.logger.dir`을 `run_dir`로 실어 등록한다.

## 20. 경로 따라가기, 팔 링크 점, 실행 요약 (REQ-023, `ultrasonic-spec.md` 8.2절)
- `ArmService.set_sensor_offset({x, y, z})`: 손목 센서가 고정 턱 끝에서 떨어진 위치(x·y는 도구 프레임 수평축, z는 턱 끝 위 높이). 값은 `config/ultrasonic.yaml`의 `arm_sensor_offset_mm`를 호출하는 쪽이 한 번 넘긴다(팔 쪽 설정에는 없다). `tool_position(q)`는 손끝, `sensor_position(q)`는 센서의 작업대 좌표(z는 작업대 면 위 높이).
- `follow_path(points_mm, tool_z_mm, speed_mm_s, yaw_deg=None) -> list[PoseSample]` (블로킹): 손끝 xy 꺾은선을 수직 아래 자세로 따라간다. `yaw_deg=None`이면 손목 회전은 지금 값. 움직이기 전에 접근과 모든 구간을 계획해 안전 검사까지 마치고, 하나라도 못 가면 아무것도 움직이지 않고 `[]`를 돌려준다(결과는 `svc.last_path`: `ik_fail`, `out_of_bounds`+`reason`, `ok`, `stopped`, `fault`…). 센서 오프셋을 안 넘겼으면 `ValueError`.
- 속도는 `min(요청, tool_v_max_mm_s × 속도%)`다(기본 30%면 최대 30mm/s). 한 번의 호출은 지금 손끝 위치에서 첫 점까지 가는 것까지 **서지 않고 이어서** 간다(처음과 끝에서만 가속·감속). 꺾이는 점은 모서리를 2.5mm 안으로 깎는 호로 감아 지나고, 원처럼 완만한 꺾임은 곡률 반지름 그대로 지나며, 속도는 구심 가속이 한도 안이 되게 줄인다(`junction_arcs`, `LookaheadShape`). 접선 가속과 구심 가속이 겹쳐도 합이 `tool_a_max_mm_s2`를 넘지 않게 둘 다 a/√2로 잡았다(원 r=45mm에서 합 가속 최대 119.5 ≤ 120). 180°로 되돌아가는 꺾임만 서서 돈다. 접선 가속이 바뀌는 순간의 저크는 제한하지 않는다(5차 프로파일보다 거칠 수 있어 실물에서 떨림을 본다).
- 지금 위치에서 첫 점까지 직선(5mm 간격 IK, 관절 도약 15° 이하)으로 갈 수 없으면 예전처럼 `plan_goto`의 안전 경로(관절 이동으로 위에 간 뒤 수직 하강)로 먼저 가고, 경로는 그다음에 따로 지난다(이때만 접근과 경로 사이에서 선다). `_plan_path`가 계획만 한다(움직이지 않음).
- `PoseSample(t_ns, tool_mm, sensor_mm)`: 50Hz 제어 틱마다 하나. `t_ns`는 `time.perf_counter_ns()`(PC 단조 시계)에서 그 틱의 읽기 시각이다. 위치는 그 틱에 버스에서 읽은 실제 관절값으로 계산한다. 이 읽기는 `follow_path`가 도는 동안만 틱마다 한 번 추가된다(`MotionController.tap`). 읽는 시간은 `follow_path` 이벤트(`events.jsonl`)의 `read_ms_max`·`read_ms_mean`에 남으므로, 실물에서 틱(20ms)을 못 지키면 이 값으로 알 수 있다(`--sim`은 읽기가 즉시라 이 값이 0에 가깝다).
- `ArmService.arm_links(q=None) -> [(x, y)]×4`: 어깨(위팔 관절)·팔꿈치·손목·고정 턱 끝의 작업대 xy. 레이더가 팔 자리를 지우는 `arm_links_fn`이다.
- 새 타입: `perception/base.py`의 `PathSegment`, `PoseSample`, `CheckProbe`, `CheckableSource`(설계서 8.2절 그대로).
- `task/summary.py`의 `RunSummary`: `Studio.start`가 끝나거나 멈출 때 `runs/<시각>/summary.json`을 쓴다(기록 폴더가 없으면 건너뜀). 항목: `source`, `end`, `objects`·`succeeded`·`failed`·`skipped`, `total_s`, `stages_s`(상태별 소요 초: PREPARE·SCAN·DECIDE·PICK·PLACE, 나중에 CHECK도 자동으로 들어간다), `per_object`(`id`, `result` ok/failed/skipped, `reason`, `stages_s`). 인식 방식이 물체별 시간을 더하려면 `studio.summary.note(object_id, scan_s=…)`를 부른다.
- 도달 확인(손끝 55mm, 반경 100~240mm, 각도 25~155°의 135점 모두 닿음)과 확인 경로 전체의 도달 계산(정확 모드 경로로는 후보 38/135만 끝까지 감)은 `measurements.md`.
