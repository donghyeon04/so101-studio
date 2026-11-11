# 로봇팔 소프트웨어 설계서

상태: 확정 · 갱신 2026-11-02 · 11.3 놓기 동작 모양 2026-11-07, 대칭 V자 개정 2026-11-10, 상승 최소·상자 폭·높이 모름 집기 2026-11-11

로컬 개발 세션이 이 문서만 보고 팔 모듈을 구현할 수 있도록, 구조·인터페이스·출력값·작업 순서를 정한다. 범위는 개발 0~2단계(구동 점검, 동작 라이브러리, 작업 공간 확정)다. 인식 모듈과 GUI는 팔 모듈과 맞닿는 인터페이스만 정한다.

상위 문서: `system-architecture.md` (모듈 경계, 공통 타입, 폴더 구조).

근거: `decisions/0003`(LeRobot 제어 스택), `0004`(인식–팔 분리), `0005`(개발 순서), `0006`(장치 구성), `features.md`. 구현 결과는 `decisions/0008~0019`, `arm-dev-status.md`, `measurements.md`에 있고 이 문서에 반영했다(2026-11-02). 설정의 실제 값은 `config/robot.yaml`이 기준이다.

## 1. 제어 방식 요약
- STS3215는 PWM 서보가 아니라 **시리얼 버스 서보**다. PC가 USB–버스 보드를 거쳐 각 모터에 "목표 위치" 숫자를 보낸다. 위치 분해능은 12비트(0~4095, 약 0.088°/단계)다.
- 위치 제어는 모터 내부 PID가 한다. LeRobot은 떨림을 줄이려고 P=16, I=0, D=32로 설정한다(기본 P=32).
- **부드러운 동작은 PC 쪽 궤적 보간으로 만든다.** 목표 위치를 한 번에 보내지 않고, 시간에 따라 매끄럽게 변하는 중간 목표를 50Hz로 계속 보낸다(5차 다항식 또는 사다리꼴 속도).
- 모든 모터 통신은 LeRobot `SO101Follower`를 거친다. 직접 레지스터를 쓰는 코드는 장치 계층 안에만 둔다.

## 2. 계층 구조
```
[앱]        CLI (지금)                    GUI (나중, 기술 미정)
              │                              │
[서비스]    ArmService ── GUI·CLI가 부르는 유일한 입구, 상태 스트림 발행
              │
[기능]      skills: home, park, pick, place, sequence    teleop: jog    teach: record, replay
              │
[동작]      MotionController (50Hz 실행 루프, 정지·선점)   trajectory: MoveJ, MoveL, MoveP
              │
[기구학]    Kinematics: FK, IK (placo 기본, 교체 가능)
              │
[장치]      ArmDriver (LeRobot SO101Follower)  |  SimDriver (하드웨어 없이 동작)

[공통]      types · frames(좌표 변환) · config · safety · logging
```
- 위 계층은 아래 계층만 부른다. GUI는 `ArmService` 외에는 아무것도 import하지 않는다.
- `SimDriver`는 명령받은 위치를 그대로 현재 위치로 돌려주는 가짜 장치다. 팔 없이 GUI·로직을 개발하고 테스트하는 데 쓴다.

## 3. 폴더 구조
레포 전체 구조는 `system-architecture.md` 9절을 따른다. 팔 모듈 파일은 아래와 같다.
```
src/so101_studio/arm/
  driver.py, sim_driver.py, kinematics.py, trajectory.py,
  motion.py, safety.py, skills.py, teleop.py, teach.py
src/so101_studio/service/arm_service.py
```

## 4. 좌표계와 단위
| 항목 | 규칙 |
|---|---|
| 작업대 좌표계 (`table` = world) | 원점 = 작업대 왼쪽 아래 모서리, x = 긴 변 방향, y = 짧은 변 방향, z = 위. 인식 모듈과 GUI는 이 좌표만 쓴다 |
| 로봇 좌표계 (`robot` = base) | 원점 = 어깨 회전축, x = 앞, y = 왼쪽. 작업대 좌표계와의 관계(x, y, 회전각)는 `workspace.yaml`에 둔다 |
| 툴 좌표계 (`tool`) | 상대 이동만 받는다. z = 도구 앞, x = 턱이 벌어지는 축. 5축이라 자세 변경(기울이기)은 받지 않는다 (`decisions/0013`) |
| 높이 기준 | 모든 좌표계에서 높이는 **작업대 면 위 mm**다. 작업대 면의 모델 높이는 `table_z_mm`(현재 −25, 육안 추정, 실측 필요) |
| 입력점 (TCP) | `tip` = 고정 턱 끝(기본), `ref` = 고정 턱 안쪽 면(URDF `gripper_frame` 점) |
| 외부 단위 | 길이 mm, 각도 도(°). 서비스·인식·GUI 사이의 모든 값 |
| 내부 단위 | 기구학·궤적 계산은 m, rad. 변환은 `frames.py`에서만 |
| 물체 각도 | 작업대 x축 기준 반시계 방향, 0~180° (0/45/90/135는 인식 모듈의 분류값) |
| 그리퍼 | 0(닫힘)~100(완전 열림), LeRobot 정규화 값. 폭(mm) 대응표 `gripper_width_mm`은 URDF 기하 + 카드 54mm 실측 1점(값 약 30). 20~30mm, 70~90mm는 미확인 |

배치가 아직 미정이므로 코드는 로봇 위치를 하드코딩하지 않고 `workspace.yaml`에서 읽는다. 배치가 바뀌어도 설정 파일만 고친다.

## 5. 데이터 타입 (`core/types.py`)
```python
@dataclass
class JointState:          # 관절 6개, 도 단위
    shoulder_pan: float; shoulder_lift: float; elbow_flex: float
    wrist_flex: float; wrist_roll: float; gripper: float  # gripper는 0~100

@dataclass
class ToolPose:            # 작업대 좌표계
    x_mm: float; y_mm: float; z_mm: float
    yaw_deg: float         # 턱이 벌어지는 축의 방향(위에서 본 각도). 집기 yaw = 물체 각도 + 90°
    pitch_deg: float = -90 # -90 = 수직 하강, 부호 유지(-115 = 25° 기울임)

# DetectedObject는 system-architecture.md 3절의 공통 타입을 쓴다

@dataclass
class ArmStatus:           # 상태 스트림, 10Hz
    t: float; joints: JointState; tool: ToolPose
    temperature_c: dict[str, float]; load_pct: dict[str, float]; voltage_v: float
    torque_enabled: bool; state: str   # "idle" | "moving" | "teleop" | "teach" | "stopped" | "fault"
    fault: str | None
    current_ma: float      # 서보 전류 합(추정)

@dataclass
class MotionResult:
    status: str            # "ok" | "ik_fail" | "out_of_bounds" | "stopped" | "preempted" | "fault" | "timeout"
    duration_s: float; final_tool: ToolPose; position_error_mm: float | None
```

## 6. 모듈 명세

### 6.1 장치: `ArmDriver` / `SimDriver`
| 메서드 | 동작 |
|---|---|
| `connect(port, calibration_id)` / `disconnect()` | 연결. 연결 시 PID(16/0/32)와 서보 가속 한계(팔 10, 그리퍼 25, `decisions/0014`) 적용. 시리얼 포트는 잠금으로 한 스레드만 쓴다 |
| `read_joints() -> JointState` | 현재 위치 (sync_read) |
| `write_joints(JointState)` | 목표 위치 (sync_write, 응답 대기 없음) |
| `read_health() -> dict` | 온도, 부하, 전압, 전류. 2Hz로만 호출 |
| `set_torque(on: bool)` | 토크 켜기/끄기 |

### 6.2 기구학: `Kinematics`
| 메서드 | 동작 |
|---|---|
| `fk(JointState) -> ToolPose` | 순기구학 |
| `ik(ToolPose, seed: JointState) -> JointState | None` | 역기구학. 해가 없거나 관절 한계를 넘으면 None |

- 구현: LeRobot `RobotKinematics` + placo 0.9.25. Windows에서는 pip가 실패해 conda-forge로 설치한다(`arm-hardware-setup.md`). FK 원점은 어깨 회전축으로 보정했다.
- IK에 우리 관절 한계(`joint_limits_deg`)를 넣어 한계 밖 해를 만들지 않는다. 방향 가중치는 0.1(1.0이면 처짐 자세에서 위치 해가 3mm 넘게 어긋남).
- 시뮬레이션에서 수직 하강 IK 왕복 오차는 1mm 이내다. 실제 끝 위치 정확도(T6 줄자 실측)는 아직 없어서, 해석적 IK 추가 여부는 그 뒤에 정한다.
- 수직 하강으로 닿는 영역(계산): 높이 0~20mm에서 회전축 앞 80~260mm, 최대 높이 약 80mm. 작업 영역 판단은 `workspace.yaml`의 `reach_mm`(90~250)을 쓴다.

### 6.3 궤적: `trajectory.py`
| 함수 | 동작 |
|---|---|
| `move_j(q_start, q_goal, v_max, a_max) -> Trajectory` | 관절 공간 5차 다항식(기본). `trajectory_profile: ruckig`이면 속도·가속도·저크 한계 시간 최적(같은 한계에서 약 20% 빠르지만 떨림이 15~25% 큼) |
| `move_l(pose_start, pose_goal, v_mm_s) -> Trajectory` | 직선 경로를 5mm 간격으로 나눠 각 점 IK. 한 점이라도 실패하면 전체 거부 |
| `move_p(waypoints, v) -> Trajectory` | 경유점 사이를 블렌딩해 멈추지 않고 통과 (시퀀스·춤용) |

`Trajectory`는 `sample(t) -> JointState`와 `duration_s`를 가진다. 참고: Robotics Toolbox for Python의 `jtraj`, `ctraj`, `mstraj`. 경로는 실행 전에 0.1초 간격으로 안전 검사한다. 직선이 안 되면 `goto`는 목표 위 접근 높이로 MoveJ한 뒤 수직으로 내려간다.

### 6.4 동작 실행: `MotionController`
- 별도 스레드에서 50Hz로 돈다. 매 주기마다 궤적을 샘플링하고, 안전 검사를 거쳐 `write_joints`를 호출한다.
- 장치는 이 스레드만 만진다. 다른 계층은 명령 큐로만 요청한다. 제어 주기는 50Hz가 최선이었다(100·200Hz와 5~10Hz 모두 더 떨림, `decisions/0014`).

| 메서드 | 동작 |
|---|---|
| `execute(traj) -> MotionHandle` | 실행 시작. `handle.wait() -> MotionResult` |
| `stop()` | 감속 0.2초로 정지 후 현재 위치 유지(실측 0.24~0.28초) |
| `estop()` | 즉시 현재 위치 고정, 큐 비움, 상태 `stopped`. 토크는 끄지 않는다(끄면 팔이 떨어짐) |
| `retarget(goal)` | 진행 중인 동작을 현재 위치·속도에서 새 목표로 매끄럽게 교체(Ruckig). 이전 동작의 결과는 `preempted` |
| `jog(...)` | 원격 조종 속도 입력을 실제 경과 시간으로 적분해 목표를 갱신(틱 간격 요동 보정) |

### 6.5 안전: `safety.py`
매 주기, 명령을 보내기 전에 검사한다. 하나라도 어기면 그 명령은 보내지 않고 `stop()`한다.
- 관절 한계: 캘리브레이션으로 잰 가동 범위에서 1~3° 안쪽(`joint_limits_deg`). 팔꿈치 접는 쪽은 95°(96°에서 J2 모터에 닿음).
- 작업 영역: 그리퍼 끝이 작업대 경계 안(여유 10mm). 바닥: 낮은 턱 끝이 작업대 면보다 `tip_floor_mm`(5mm) 위.
- 한 주기 이동량: 관절당 최대 5° (LeRobot `ensure_safe_goal_position`).
- 온도 60°C 이상 3회 연속(읽기 튐 걸러냄), 부하 90% 이상 3초, 통신 끊김 0.5초, **전류 합 4A 이상 2회 연속** → `fault`.
- 원격 조종 중 입력이 0.3초 동안 들어오지 않으면 정지 (데드맨).
- 한계 밖(받침에 기댄 팔꿈치 약 97° 등)에서 출발한 동작은 더 바깥으로 가지 않는 한 허용해 빠져나올 수 있다. 관절 한계, 바닥, **작업대 x·y 모두** 같다: 작업대 밖에서 출발하면 벗어난 거리가 출발점보다 늘지 않는 점만 허용한다(`decisions/0031`).
- 통신 끊김 뒤: fault 상태에서는 다시 [연결]하기 전까지 아무 명령도 보내지 않는다. 다시 연결하면 현재 위치와 토크를 읽고 명령값을 실제값에 맞춘 뒤에만 다음 동작을 받는다. 다시 연결할 때 토크를 끄지 않는다.

안전 설정 `SafetySettings`는 실행 중에 바꿀 수 있다 (`decisions/0015`).
| 항목 | 값 |
|---|---|
| `mode` | `normal`(관절 한계 + 작업대 + 바닥) / `free`(작업대 x, y만 끔) / `off`(관절 한계·작업 영역·바닥 끔). 한 주기 이동량, 과열·과부하·통신·과전류 감시는 모든 모드에서 켜짐 |
| `joint_margin_deg` | 양수 = 한계를 안쪽으로, 음수 = `physical_limits_deg`까지만 넓힘 (그리퍼 제외) |
| `box_mm` | 작업대 좌표 x, y와 작업대 면 위 턱 끝 높이 z의 허용 범위 |
| 남은 일 | 속도 상한(`max_speed_pct`)을 안전 설정에 넣기 |

### 6.6 기능: `skills.py`
| 함수 | 순서 |
|---|---|
| `home()` | 정해진 기본 자세로 MoveJ |
| `park()` | 센서 시야 밖 대기 자세로 MoveJ. 스캔 전에 반드시 호출 |
| `pick(obj: DetectedObject)` | 접근 높이(`approach_height_mm` 40 + 집는 높이 15)로 MoveJ → 턱 축을 물체 각도 + 90°로 돌리고 벌림 → 하강 MoveL → 닫으며 손을 같이 보내 중심 유지 → 접촉에서 정지하고 목표를 지금 위치 + 예압으로 → 들고 있는 동안 조임 한계 `gripper_hold_torque` → 수직 상승. 접촉이 없으면 올라온 뒤 그리퍼를 열고 `no_object` |
| `place(zone: str)` | 배출 구역 위로 MoveJ → 하강 → 열면서 손을 같이 보내 두 턱 중간점 유지(닫는 경로를 반대로) → 상승 → 조임 한계 원래 값 |
| `run_sequence(name)` | `config/sequences/<name>.json`의 경유점을 MoveP로 재생 (시연용 동작, 춤) |

- 그리퍼가 고정 턱 1개 + 가동 턱 1개라 아래 보정을 한다 (`decisions/0009~0012`).
  - 턱 축 = 물체 각도 + 90°. 180° 돌려도 같은 파지라 손목이 덜 도는 쪽을 고른다. 물체 각도가 None이면 0°.
  - 벌림: 크기와 상관없이 최대 `grasp_max_open_mm`(70mm)로 벌려 틈의 중심을 물체에 맞춘다(`grasp_open_policy: max`, 약 60mm 폭까지).
  - 닫을 때 틈이 d 줄면 손을 가동 턱 쪽으로 d/2 같이 보내 닫는 내내 중심을 유지한다.
  - 턱이 앞뒤로 벌어질 때만 손을 벌림 각도의 절반만큼 기울여(최대 35°) 보기에 대칭인 V자로 닫는다.
  - 접촉 판정: 실제 그리퍼 값이 명령보다 4값 넘게 앞서 멈춤(`contact_lag_units`).
  - 조임 (`decisions/0033`): 접촉하면 목표를 지금 위치에서 **닫는 쪽으로**(그리퍼 값이 작아지는 쪽, 0 = 닫힘) `grip_preload_units`(2)만큼 더 간 값으로 고정한다(결정 0033의 "지금 위치 + 예압"은 이 뜻이다). 그리퍼를 여는 명령(`set_gripper`, `place`의 열기, `no_object` 열기)이 오면 고정을 풀고 조임 한계를 원래 값으로 되돌린다. 정지·E-STOP에서는 물체를 놓치지 않게 고정과 200을 유지한다. 조임 한계는 RAM `Torque_Limit`(48번)으로 바꾸고 EEPROM(`Max_Torque_Limit`)에는 쓰지 않는다. 닫는 동안 `gripper_squeeze_torque` 300, 들고 옮기는 동안 `gripper_hold_torque` 200(연결 때 쓰는 그리퍼 `Overload_Torque` 25% 아래), 놓은 뒤 원래 값.
  - 그리퍼 부하 감시: `gripper_load_warn_pct`를 `gripper_load_s` 넘게 넘으면 먼저 목표를 지금 위치로 맞추고 WARN, 그래도 이어지면 열고 fault. 시작값은 정상 집기 5회 부하 최대값의 1.3배, 1초.
- 접근 높이를 80mm에서 40mm로 낮췄다. 수직 하강으로 닿는 최대 높이가 약 80mm이기 때문이다. 따라서 **집을 수 있는 물체 높이에 제약이 있다**(시험 물체 선정 때 확인).
- 물체 높이: `DetectedObject.height_mm`를 알면 높이에 맞춰 집는다(11.2). 모를 때(0)는 다음과 같다(결정 0052).
  - 기울여 집을 수 있으면(`tilt_ok`) 지금처럼 `grasp_height_mm`로 집는다.
  - **기울이지 못하고(`tilt_ok=False`) 높이도 모르면 집지 않는다.** 결과 `height_required`(이유 문장: "턱 축이 팔 방향과 어긋나 수직으로 집어야 해서 물체 높이가 필요합니다")를 돌려준다. 수직으로 높이를 모른 채 내려가면 턱이 물체 위 모서리를 친다(sim 384건 중 127건, 높이를 알면 381/384 ok, 11-10 실물 배출 구역 `no_object`의 원인).
  - 수동·CLI·GUI는 높이를 받는다. CLI `pick`은 5번째 값, GUI는 높이 칸이다.
  - 초음파 작업 실행은 높이를 사람에게 묻지 않는다. 대신 그 후보를 한 번 더 `CHECK`해 높이를 잰다. 그래도 모르면 `note_failure`로 넘긴다.
  - 초음파 작업 실행에서 수직 집기가 `no_object`면, 그 후보를 한 번 더 `CHECK`해 높이를 다시 재고 한 번만 다시 집는다. 두 번째도 실패하면 `note_failure`.
  - 서보 부하로 높이를 재는 방식(닿을 때까지 내려가기)은 실물 기준값이 필요하고 정밀도가 의심스러워 나중 실험 후보로만 둔다.
- 알려진 문제: 그리퍼 앞 다리가 제대로 맞물리지 않아 실제 물체를 집지 못한다. 조원이 그리퍼를 재설계한다.

### 6.7 원격 조종: `teleop.py`
- `jog(mode, velocity)`: `mode`는 `"joint"`(관절별 °/s) 또는 `"cartesian"`(x, y, z mm/s, yaw °/s). 입력 장치와 무관하게 이 명령 하나로 받는다. 기본 속도 관절 30°/s, 직교 80mm/s.
- 키보드 `keys`: Tab으로 관절 → 직교 조그(로봇/툴 기준) → 좌표 입력 세 모드. 좌표 입력은 `point_pose` → `preview_tool`로 먼저 검사해 닿지 않으면 움직이지 않고 이유를 알린다 (`decisions/0013`).
- 입력 어댑터: 키보드(지금, Windows `GetAsyncKeyState`), GUI 가상 조이스틱과 게임패드(나중). 모두 `jog`를 부른다.
- 느린 조그에서 체감 떨림이 남아 있다(서보 내부 설정·기구 하중 의심, `Minimum_Startup_Force` 시험이 남음).

### 6.8 동작 녹화·재생: `teach.py`
토크를 켠 채 손으로 미는 추종 교시는 기어 마찰(1/345) 때문에 실패해 제거했다. 토크를 끈 방식 하나다 (`decisions/0016~0019`).
- `record_start(hz, max_s)`: 홈 자세로 이동 → 팔을 잡았다는 확인 → 3초 카운트다운 → 토크를 끄고 관절값과 **실제 시각**을 기록(기본 20Hz, 최대 90초, `record_hz`·`record_max_s`).
- `record_stop(name)`: `config/sequences/<name>.json`에 저장(길이, 점 수, 평균·최대 속도, 관절별 최대 속도 `meta` 포함)하고 토크를 켠다.
- `record_info(name)`: 길이·속도와 구간 목록(멈춤 < 3°/s, 느림, 빠름 ≥ 40°/s, 최소 0.5초).
- `replay(name, speed, until_limit, segment_speeds)`: 이동평균으로 다듬어 기록 시각대로 재생. 구간별 배율 0.1~5(최대 220°/s 초과 시 거부). 한계를 넘는 기록은 거부하고, `until_limit`이면 한계 직전까지 재생한다.
- 주의: 토크를 끄면 팔이 중력으로 처지므로, 녹화 시작 전에 팔을 손으로 받친다.

### 6.9 서비스: `ArmService`
GUI·CLI가 부르는 메서드 목록이다. 이름과 입력·출력은 GUI 기술이 정해져도 바뀌지 않는다.

| 메서드 | 입력 | 출력 |
|---|---|---|
| `connect` / `disconnect` | 포트 (생략 시 설정값) | 성공 여부, 오류 메시지 |
| `status_stream(callback)` | 콜백 | `ArmStatus` 10Hz |
| `home` / `park` | | `MotionResult` |
| `move_joints` | `JointState`, 속도 % | `MotionResult` |
| `move_tool` | `ToolPose`, 속도 % | `MotionResult` |
| `status` | | `ArmStatus` 1회 |
| `point_pose` | x, y, 높이, yaw, 좌표계(`table`/`robot`/`tool`), TCP(`tip`/`ref`) | `ToolPose` |
| `preview_tool` | `ToolPose` | 움직이지 않고 가능 여부, 이유, 풀린 관절값 |
| `pick_and_place` | `DetectedObject`, 배출 구역 | `MotionResult` |
| `place` | 배출 구역 | `MotionResult` |
| `set_gripper` | 값 0~100 또는 폭 mm | `MotionResult` |
| `jog` | 모드, 속도 벡터 | 성공 여부 (데드맨 적용) |
| `record_start` | 기록 주기 Hz, 최대 시간 s | 없음 |
| `record_stop` / `record_info` | 이름 | 저장 결과 / 길이·속도·구간 |
| `replay` | 이름, 속도, `until_limit`, `segment_speeds` | `MotionResult` |
| `run_sequence` | 이름 | `MotionResult` |
| `retarget` | `JointState` | 없음 |
| `get_safety` / `set_safety` | 모드, 관절 여유, XYZ 범위 | 현재 안전 설정 (JSON 변환 가능) |
| `stop` / `estop` | | 없음 |
| `set_speed` | 0~100 % | 없음 |

GUI에서 점 찍기는 `point_pose` → `preview_tool` → `move_tool` 순서로 부른다.

## 7. 설정 파일 (주요 값, 실제 값은 `config/*.yaml`이 기준)
| 항목 | 값 | 근거 |
|---|---|---|
| 통신 | 1Mbps, 서보 ID 1~6. 포트는 7.1절 순서로 고른다(이 PC는 COM3) | `arm-hardware-setup.md`, `decisions/0027` |
| 제어 주기, PID | 50Hz, P16 I0 D32 | `decisions/0014` |
| 서보 가속 한계 | 팔 10, 그리퍼 25 | `decisions/0014` |
| 홈·파크 자세 | `[0, -90, 93, 0, 0, 3]` (파크는 센서 위치가 정해지면 바꿈) | 팔꿈치 한계 95 |
| 영점 보정 | shoulder_pan −3.0° (팔 파일 `arms/so101_follower_01.yaml`) | 육안 정렬 |
| 속도 | 기본 30%, 관절 최대 90°/s, 직교 최대 100mm/s, 조그 30°/s·80mm/s | |
| 궤적 | `trajectory_profile: quintic` (선택 `ruckig`) | measurements.md |
| 높이 | `table_z_mm` −25(육안 추정), `tip_floor_mm` 5, `approach_height_mm` 40, `grasp_height_mm` 15, `place_height_mm` 30 | 실측 필요 |
| 집기 | `grasp_open_policy: max`, `grasp_max_open_mm` 70, `grasp_tilt_mode: symmetric`, `grasp_tilt_deg` 35, `contact_lag_units` 4, `gripper_squeeze_torque` 300 | `decisions/0010~0012` |
| 교시 | `record_hz` 20, `record_max_s` 90, 구간 기준 3·40°/s, 최소 0.5초 | `decisions/0019` |
| 작업대 (`workspace.yaml`) | 600×400mm(임시), 로봇 원점 (300, 50) 90°(긴 변 가운데, 임시), 배출 구역 right (500, 150), 도달 반경 90~250mm | 배치 미정 |

### 7.1 설정 겹과 팔 고르기 (`decisions/0027`)
| 파일 | 내용 | git |
|---|---|---|
| `config/robot.yaml` | 모든 SO-101 공통 기본값(속도, 궤적, 집기 방식, 교시, 자세) | 올림 |
| `config/arms/<팔 이름>.yaml` | 팔별 값: `board_serial`, `calibration_id`, `joint_offsets_deg`, `joint_limits_deg`, `physical_limits_deg`, `table_z_mm`, `gripper_width_mm`, `grasp_bias_mm`, 잰 날짜 | 올림. 새 팔은 `arms/_template.yaml`을 복사 |
| `config/local.yaml` | PC별 값: `port`, `arm`, 그 밖에 이 PC에서만 덮을 값 | 제외(`.gitignore`). `local.example.yaml`을 올림 |
| `config/calibration/<calibration_id>.json` | LeRobot 캘리브레이션 | 올림(지금과 같음) |

- 읽는 순서는 `robot.yaml` → 팔 파일 → `local.yaml`이고, 뒤가 앞을 덮는다. `local.yaml`이 없으면 지금과 똑같이 동작해야 한다.
- 팔 고르기: `--arm` → `local.yaml`의 `arm` → 보드 시리얼이 같은 팔 파일 → 팔 파일이 하나뿐이면 그것 → 못 고르면 멈추고 묻는다.
- 포트 고르기: `--port` → `local.yaml`의 `port` → VID·PID와 시리얼로 자동 → 여러 개면 묻는다 → 못 찾으면 `so101 find-port`(USB를 뺐다 꽂아 전후 목록 비교, LeRobot 방식).
- 안전: 보드 시리얼은 맞는데 서보 캘리브레이션 값이 팔 파일과 다르면 움직이는 명령을 막는다(읽기만). 팔 파일이 없는 새 팔도 같다.
- `so101 doctor [--json]`: 읽기 전용. 항목마다 정상/주의/실패와 고치는 방법. 점검 항목은 conda 환경과 Python, numpy가 conda 설치본인지, lerobot·placo 버전, GUI 의존성(선택), 포트 목록(VID·PID·시리얼), 고른 팔 파일과 고른 이유, 캘리브레이션 파일 대조, 서보 1~6 응답, 전압, 온도다.
- 설치와 실행: `environment.yml`(Python 3.12, conda-forge numpy·placo 0.9.25, pip lerobot[feetech] 0.6.1과 `-e .[gui]`). 런처는 `so101.cmd`(Windows)와 `so101.sh`(Linux)이고, 고정 경로 대신 conda 환경 이름 `so101`(바꿀 수 있음)으로 찾는다. Linux는 `dialout` 그룹 안내를 띄운다.

## 8. 출력과 기록 (`decisions/0030`)
- 실행할 때마다 `runs/YYYYMMDD-HHMMSS/`를 만든다. 모든 기록 줄은 JSON Lines이고 공통 머리 `t`(벽시계 ns), `proc`(cli/gui/watchdog), `pid`, `seq`, `level`(DEBUG/INFO/WARN/ERROR/FATAL), `code`, `kind`, `data`를 가진다.
- `events.jsonl`: 명령과 결과, 연결·해제, 토크 켜기·끄기(부른 함수 이름), 레지스터 쓰기(`Goal_Position` 스트림 제외: 레지스터, 서보, 값, 부른 곳, 성공 여부), fault와 이유, E-STOP, 감시 작동, 조작권·실행 방식·안전 설정 변경, 캘리브레이션 저장·확인, 예외(첫 발생은 전체, 반복은 횟수).
- `status.jsonl`(10Hz, 지금 `status.csv`를 대신함. CSV는 내보내기로 만든다): 관절별 명령값·실제값·토크·온도·부하·전류(온도·부하·전류·토크·`Status`는 2Hz로 읽은 값을 이어 씀), 손끝 x·y·z, `state`, `fault`, 서보 `Status` 비트, 전압, 누적 `time_lost_s`, 직전 1초 틱 간격 최대값. 연결하자마자 첫 줄을 쓴다.
- 토크 풀림 감지: 2Hz로 읽은 `Torque_Enable`이 컨트롤러가 기대하는 값과 다르면 `ARM-TORQUE-LOST` ERROR 이벤트와 사고 묶음.
- 쓰는 방법: 제어 루프와 상태 루프는 큐에만 넣고, 기록 스레드가 쓴다. 줄마다 flush, WARN 이상은 fsync. 큐(기본 10,000줄)가 차면 상태 줄만 버리고 `dropped` 수를 다음 줄에 남긴다. 이벤트는 버리지 않는다.
- 링 버퍼: 50Hz 틱(명령, 실제, 틱 간격, 버스 오류)의 최근 10초를 메모리에 둔다. fault, E-STOP, 감시 작동, `ARM-TORQUE-LOST`, 처리 안 된 예외 때 사고 묶음으로 쓴다.
- 사고 묶음 `runs/incidents/<YYYYmmdd-HHMMSS>-<code>/`: `ring.jsonl`, 모든 프로세스 기록의 앞뒤 60초를 합친 `timeline.jsonl`, `robot.yaml`·팔 파일·`local.yaml`·캘리브레이션 사본과 sha256, `doctor.json`, git 커밋, 패키지 버전. `so101 incident [--last N초]`로 손으로도 만든다.
- 보관: 시작할 때 `runs/`가 30일 또는 2GB를 넘으면 오래된 실행부터 지운다. `status.jsonl`은 50MB마다 나눈다. `runs/incidents/`는 지우지 않는다.
- 감시 프로세스는 GUI가 넘긴 실행 폴더에 `watchdog.jsonl`을 쓴다.
- 성능 기준: 50Hz 틱 안의 버스 읽기는 늘리지 않는다. 바꾸기 전후 5분 실행에서 `time_lost_s` 증가 1% 미만, 틱 간격 p99 증가 2ms 이하.
- 이 기록이 F1(실행 로그)·F2(CSV 내보내기)의 원천이다.

## 9. CLI (GUI 전 개발·점검용)
실행은 레포 루트의 `so101.cmd [--sim] [--speed N] <명령>`이다(conda 활성화 불필요). 옵션 전체는 `CLAUDE.md` 개발 세션 규칙에 있다.

| 구분 | 명령 |
|---|---|
| 이동 | `status`, `home`, `park`, `movej`, `goto`, `pick x y 각도 폭 [--place 구역]`, `place`, `gripper` |
| 자동 작업 | `run --source manual [--step] [--dry-run]` |
| 조종 | `keys` (관절 / 직교 조그 / 좌표 입력), 옛 `jog` |
| 교시 | `record <이름>`, `info <이름>`, `replay <이름> [--seg-speed 1=0.5] [--until-limit]`, `seq <이름>` |
| 진단 | `powertest`, `shaketest`, `servoinfo`, `linkwatch`, `--hz`, `--trace` |
| 안전 | `--free`, `--unsafe`, `--safety-margin N`, `--box x0,x1,y0,y1,z0,z1` |

## 10. 하드웨어 초기 설정 (T0, 코드 작성 전 1회)
LeRobot 공식 SO-101 절차를 따른다. 순서를 바꾸지 않는다.

| 단계 | 내용 | 완료 기준 |
|---|---|---|
| T0-1 전원·배선 | 12V 전원 → 버스 서보 보드, 보드 → PC USB, 보드 → 모터 3핀 케이블. 보드에 USB/MCU 전환 점퍼가 있으면 USB 쪽으로 둔다(Waveshare 보드는 `B` 채널) | 보드 전원 LED 켜짐 |
| T0-2 포트 찾기 | `lerobot-find-port` (USB를 뺐다 꽂아 사라지는 포트) | COM 번호를 `robot.yaml`에 기록 |
| T0-3 모터 ID 설정 | `lerobot-setup-motors --robot.type=so101_follower --robot.port=COMx`. 안내에 따라 **한 번에 모터 하나만** 연결(그리퍼 6번 → 어깨 회전 1번 순). 이미 조립했으면 모터 사이 케이블을 하나씩 빼서 진행 | 6개 모두 ID·통신 속도 설정 완료 메시지. 키트에서 이미 설정됐는지는 6개를 모두 연결한 뒤 읽기가 되는지로 확인 |
| T0-4 캘리브레이션 | `lerobot-calibrate --robot.type=so101_follower --robot.port=COMx --robot.id=so101_follower_01`. 모든 관절을 가동 범위 가운데에 두고 Enter → 각 관절을 끝에서 끝까지 천천히 움직인 뒤 Enter | 캘리브레이션 파일 생성. 레포 `config/calibration/`에 사본 보관 |
| T0-5 확인 | 토크를 끈 채 각 관절을 손으로 움직이며 읽기값의 방향과 범위가 맞는지 확인 | 6개 관절 모두 정상. 이상 있으면 T0-4 재실행 |
| T0-6 첫 구동 | 속도 낮게, 관절 하나씩 ±10°만 움직여 본다. 사람이 전원 스위치 옆에서 대기 | 떨림·이상음·과열 없음 |

결과(2026-11-01): ✅ 완료. 모터 ID는 이미 설정되어 있었고, 캘리브레이션은 네 번째 시도에서 성공했다(`measurements.md` T0, `arm-hardware-setup.md`).

주의: 팔로워용 STS3215-C018은 12V 모터다. 다른 전압의 전원을 쓰지 않는다.

## 11. 구현 작업 순서 (로컬 세션)
결과 열은 2026-11-02 기준이다. ✅ 완료, △ 구현은 끝났고 실측이 남음.

| # | 작업 | 완료 기준 | 관련 기능 | 결과 |
|---|---|---|---|---|
| T1 | 환경 구성: conda, Python 3.12 이상, `lerobot[feetech,kinematics]` | `import placo` 성공 여부를 기록. 실패하면 대체안 결정 기록 작성 | — | ✅ placo는 conda-forge 0.9.25로 해결 |
| T2 | `ArmDriver`, `SimDriver`, CLI `status` | 관절값·온도·부하·전압 출력. 30초 정지 자세에서 떨림 관찰 기록 | A1, B7 | ✅ |
| T3 | `Kinematics` + FK 검증 | 자세 3개에서 FK 결과와 자로 잰 그리퍼 끝 위치 차이 기록 | — | △ 방향 매핑 확인, 자 실측 없음 |
| T4 | `trajectory.move_j`, `MotionController`, `stop`, `estop`, CLI `home`·`park`·`movej` | 각 관절 전체 범위를 저속 왕복, 끊김·떨림 없음. 정지 명령 0.3초 안에 멈춤 | B2, B3, B1 | ✅ 정지 0.24~0.28초 |
| T5 | `safety.py` | 한계 밖 명령이 모두 거부되는 단위 테스트 통과 | B6, B8 | ✅ 안전 설정으로 확장(0015) |
| T6 | `frames.py`, `move_l`, CLI `goto` | 작업대 위 9개 점(3×3)에 10회씩 이동, 평균 오차와 반복 편차(mm) 기록. 이 결과로 도달 범위와 작업대 배치를 확정 | B4 | △ 시뮬·엔코더 기준만, 줄자 실측 없음 |
| T7 | 그리퍼 폭 대응표, `pick`, `place`, CLI `pick` | 손으로 넣은 좌표로 0/45/90/135° 물체 각 5회, 성공률 기록 | B5 | △ 물체 없이 동작만, 실물 성공률 없음 |
| T8 | `teleop.jog` + 키보드 입력 | 관절·직교 조종 동작, 입력 끊기면 0.3초 안에 정지 | B9 | ✅ keys 3모드(0013) |
| T9 | `teach.py`, `run_sequence` | 손으로 가르친 동작을 재생, 시연용 동작 1개 저장 | B10, B11 | ✅ 토크 끈 교시(0018·0019) |
| T10 | `ArmService`, 실행 기록 | CLI가 모두 `ArmService`를 거치도록 정리, `runs/`에 기록 생성 | F1 | ✅ |
| T11 | `retarget` | 이동 중 목표를 바꾸면 멈추지 않고 새 목표로 이어서 이동 | B12 | ✅ Ruckig |

| AC1 | 자동 캘리브레이션 엔진 가져오기: `arm/autocal/engine.py`, `plan.py` (LeRobot PR #4826, `decisions/0028`) | 가져온 가짜 서보 테스트가 LeRobot 0.6.1에서 통과 | A2 | REQ-016 |
| AC2 | `so101 calibrate --auto --measure-only` | EEPROM 불변, 기존 파일과 관절별 차이표 출력, 전압·온도 중단 | A2 | REQ-016 |
| AC3 | `--save` | `.bak.json` 생성, EEPROM·LeRobot 캐시·`config/calibration`에 저장, 저장 뒤 `doctor` 캘리브레이션 대조 정상 | A2 | REQ-016 |
| AC4 | 실물 시험 (`decisions/0028` 절차) | 기존 값과 ±3° 이내, 3회 반복 차이 1° 이하, `measurements.md` 기록 | A2 | 사용자와 함께 |
| AC5 | 반복 합격과 0° 자세 확인 (`decisions/0029`) | 새 팔·기존 값과 다른 팔도 3회 반복으로 저장 가능, 저장 뒤 0° 자세 확인 전에는 움직이는 명령이 잠김 | A2 | REQ-017 |

T1~T7이 개발 1단계(동작 라이브러리)이고, T6 결과로 2단계(작업 공간 확정)를 한다. T8~T11은 센서를 기다리는 동안 할 수 있다. 팔 이외의 작업(C1, S1~S3)과 전체 순서는 `system-architecture.md` 11절에 있다.

### 11.1 자동 캘리브레이션 (`decisions/0028`)
- 원본: `github.com/stepankandrushin/lerobot` 브랜치 `auto-calibration`의 `motors/feetech/auto_calibration.py`, `robots/so_follower/auto_calibration_plan.py`, `tests/motors/test_feetech_auto_calibration.py`.
- 순서(원본 계획 그대로): 접힌 자세(그리퍼 닫고 아래로) → 손목·어깨·팔꿈치를 조금 펴기 → 어깨 들기+팔꿈치를 끝에서 끝까지 → 팔꿈치를 들고 손목 회전·그리퍼·손목 굽힘 → 접은 자세에서 어깨 회전 → 처음 자세. 약 2분.
- 끝단 판정: 목표보다 4° 넘게 뒤처지고 0.3초 동안 0.5° 미만으로 움직임. 목표는 관절보다 최대 12° 앞. 토크 한계 500(50%), 속도 200 steps/s.
- 멈추고 되돌리는 경우: 시간 초과, 펴지지 않는 관절, 허용 범위 밖, 다른 관절의 부하 변화(외부 접촉), 오류, Ctrl-C. 되돌리는 레지스터: `Operating_Mode`, `Torque_Limit`, `Acceleration`, `Goal_Velocity`, `P_Coefficient`. 실패하면 `Homing_Offset`, `Min/Max_Position_Limit`도 되돌린다. 끝나면 토크는 꺼진다.
- 이 팔의 허용 범위: shoulder_lift 200~225°, elbow_flex 187~208°, wrist_flex 195~218°, wrist_roll 325~360°는 원본 그대로. shoulder_pan은 실물 확인 전까지 230~245°, gripper는 118~152°.
- 팔꿈치의 접히는 끝은 J2 모터 몸체에 닿는 곳이다. 측정만 모드에서 이 끝의 부하와 소리를 따로 지켜본다.
- 합격 기준(`decisions/0029`): ① 기존 파일과 모든 관절 ±3° 이내면 1회로 합격. ② 그 밖에는 24시간 안의 측정 3개가 모두 엔진 검사를 통과하고 3회 사이 관절별 범위·0° 위치 차이가 1° 이하면 반복 합격, 저장 값은 관절별 중앙값, 기존 파일이 있으면 차이표를 보이고 `yes` 입력 뒤 저장.
- 저장 뒤 0° 자세 확인: `so101 calibrate --verify-zero`가 속도 20%로 전 관절을 0°로 보내고 사람의 확인을 받는다. 확인하면 팔 파일에 `calibration_checked: {date, sha256}`(캘리브레이션 파일 해시)를 쓴다. 해시가 지금 캘리브레이션 파일과 다르거나 없으면 움직이는 명령은 `locked`(이 확인 명령만 예외).

## 11.2 2026-11-06~07 추가 (결정 0042·0043, REQ-030~043)
세부 시그니처와 값은 코드와 `config/`가 기준이다. 설계 원칙만 여기에 둔다.
- **그리퍼 인터페이스** (`arm/gripper.py`의 `Gripper`): 폭↔서보값, 턱 끝 위치, 손 기준점 보정, 기울기, 턱이 위로 좁아지는 비율, 닫는 경로를 그리퍼마다 구현한다. `SingleJawGripper`(지금 것, 기본)와 `ParallelGripper`(Robonine, 두 턱 중앙이 기준점, 기울기 0). 고르기는 `config/arms/<팔>.yaml`의 `gripper`, 값은 `config/grippers/<종류>.yaml`. 접촉 감지·조임 토크는 그리퍼마다 따로 실측한다.
- **물체 높이에 맞춘 집기**: `DetectedObject.height_mm`를 알면 턱 끝 높이 t를 물체마다 정한다. 통과 조건은 단동식 W ≤ G − (H − t)·s(G) − 4, 병렬 W ≤ G − 4(W 집는 폭, G 최대 벌림, H 물체 높이, s 좁아지는 비율). 모르면 `grasp_height_mm`. 지원 물체는 이 조건을 만족하는 크기이고, 시연 물체(높이 약 40mm, 폭 30~58mm)는 `grasp_max_open_mm` 90에서 통과한다(REQ-033 실측으로 확인).
- **집기 범위와 움직임 범위**: `workspace.yaml`의 집기 범위(수직 하강, 작업 실행·물체 판정)와 움직임 범위(손끝 방향 자유, 직접 조작·안전 상자 표시)를 나눈다. 손끝 pitch를 받는다.
- **장면 충돌 검사**: 장면 부품(`system-architecture.md` 5.1)과 턱 메쉬를 coal로 검사한다. 턱 메쉬 가장 낮은 점이 작업 면(`scene_floor`) 아래로 가지 않게 한다.
- **교시 재생 변환본**: 녹화 원본에서 한 주기 이동량(5°/20ms)·가속 한도를 넘는 구간만 늦춘 재생 가능본을 만들고, 재생 때 원본/변환본을 고른다. 원본도 한 주기 이동량 검사는 끄지 않는다.
- **fault 원인 문장**: fault에 관절·값·한도를 담고 사람이 읽는 원인과 해결법을 붙인다(`service/targets.py`의 `reason_text`). 분류는 우리 코드의 예외(`internal_error`)와 서보 오류 비트(입력 전압·과열·과부하 등, 하드웨어 쪽)를 나눈다.
- **설정 바로 적용**: 작업대·좌표 설정을 저장하면 `ArmService`·`Skills`가 다시 읽는다. 작업 실행 중에는 저장을 막는다. 그리퍼 속도 `gripper_speed_pct`는 바로 적용, `gripper_acceleration`(접촉 감지 기준)은 그대로.
- **E-STOP·감시 상태**: `estop()` 응답은 정지가 실제로 먹혔는지(`ok`)를 돌려준다. 감시 프로세스는 스레드 생존과 `last_tick_age_s`로 살아 있음을 판단한다(스레드가 죽었는데 alive로 보이던 문제, REQ-043).

## 11.3 놓기 동작 모양 (2026-11-07 첫판 → 2026-11-10 개정: 대칭 V자, 사용자 요청 U2, EGM Stage 9 반영)
**목표**: 놓을 때도 집을 때처럼 두 턱이 수직에서 같은 각도로 벌어지는 V자가 보여야 한다.

### 11.3.1 첫판(#204)과 원인
- 11-10 실물 2회의 결과(`measurements.md` "놓기 재시험 2026-11-10"):
  - 기록 기준(고정 턱 이동 20mm 이상)은 통과했다. 고정 턱 이동 30.8·32.1mm, 두 턱 중간점 이동 4.5·5.3mm, 1.0초.
  - 영상 기준은 불합격이다. 고정 턱은 수직인 채 옆으로 비켜나고, 가동 턱만 거의 수평까지 돌아 올라간다.
- 첫판은 손을 수직(pitch −90°)에 둔 채 손을 옆으로만 옮겼다. 그래서 두 턱 끝 위치는 어느 정도 맞았지만 턱 **각도**는 대칭이 아니었다.
- 집을 때는 `grasp_tilt_mode: symmetric`으로 손을 θ/2만큼 기울인다(θ = 가동 턱이 닫힌 데서 벌어진 각도 = `VALUE_TO_Q[1]`·v, 손목 한계 `grasp_tilt_deg` 35°까지). 놓을 때는 이 기울기를 쓰지 않았다.
- 같은 이유로, 기울인 채 집은 물체는 첫판 놓기에서 수직 손으로 내려가 기운 채 놓였다.

### 11.3.2 용어
- **턱 각도**: 고정 턱 각도 α_f = τ(손 기울기 = −90° − 손 pitch), 가동 턱 각도 α_m = θ − τ. `tilt()`가 쓰는 정의와 같다. 둘 다 수직 아래에서 각 턱의 바깥쪽으로 잰다. **대칭 오차** = |α_f − α_m| = |2τ − θ|. 관절값(손 pitch는 FK, θ는 그리퍼 값)으로 계산한다.
- **집기 기억**: 집기가 접촉으로 끝나면 `Skills`가 두 값을 기억한다.
  - 설정 저장으로 `ArmService`가 `Skills`를 다시 만들어도 집기 기억을 새 `Skills`에 넘긴다(쥔 채 설정을 바꾸는 경우).
  - `grip_tilted`: 집기의 `tilt_ok`
  - `grip_tip_h`: 집을 때 낮은 턱 끝의 작업 면 위 높이 t_g(mm, 모델 z는 `table_z_mm` + t_g)
  - 다음에 지운다: 놓기가 끝날 때(성공·실패 모두), 집기 실패나 `no_object`, 새 집기를 시작할 때.
  - 물체 높이 `obj.height_mm`(0 = 모름)와 쥔 틈도 함께 기억한다.
- **시작 기울기** τ_s: `grip_tilted`이면 `tilt(지금 그리퍼 값)`, 아니면 0. 손에 쥔 그리퍼 값으로 계산하므로 경로 첫 단계에서 튀지 않는다(예압으로 집은 순간과 약 1° 다를 수 있으나 받아들인다).
- **대칭 상한 틈**: v_sym = 2·`grasp_tilt_deg`/`VALUE_TO_Q[1]`, 틈 = `_gap_mm(v_sym)`(약 90mm). 이보다 벌리면 손목 한계 때문에 대칭이 깨진다.

### 11.3.3 순서
1. **놓는 손 방향(yaw)**:
   - 단동식이면 놓는 점에서 팔이 뻗은 방향을 `_jaw_yaw`(손목이 덜 도는 쪽, 로봇 기준 ±90° 안)로 고른 값으로 한다. 지금은 yaw 0 고정이다. 배출 구역 (500, 150)은 yaw 0에서 약 27° 벗어나 있어 지금 방식으로는 기울일 수 없다.
   - 반대쪽(180°)은 시도하지 않는다.
   - 손목 회전은 접근 위치로 가는 관절 이동 중에 하고, 쥔 물체도 함께 돈다. 지금도 yaw 0으로 돌리고 있으므로 새로 생기는 위험은 아니다.
   - 병렬 그리퍼는 yaw를 바꾸지 않는다(기울기 0, 지금과 같음).
2. **내려가는 자세**:
   - release = `_hand_pose(x, y, yaw, 지금 그리퍼 값, grip_tilted, table_z_mm + t_g + place_drop_mm)`. 낮은 턱 끝이 작업 면 위 t_g + `place_drop_mm`(새 설정, 3mm)에 온다.
   - 쥔 동안 미끄러지지 않았다면 물체 바닥은 작업 면 위 약 3mm에서 놓인다. 높이 실측 오차가 ±3mm라 0~6mm로 본다.
   - hover는 release를 위로 올린 같은 기울기 자세다. 내려가는 동안 물체가 돌지 않게, 집기의 hover처럼 절반만 기울이지 않는다.
   - **hover 높이**(MoveIt approach의 desired/min 구조): 원하는 값 `approach_height_mm`(40)부터 5mm씩 낮춰 최소 10mm까지, IK가 닿는 첫 값을 쓴다. 지금 구현이 이렇다.
   - 이 두 자세를 IK가 못 풀면 **옛 방식**(yaw 0, 기울기 0, 손 기준점 z = `table_z_mm` + `place_height_mm`, 지금 코드 그대로)으로 다시 계산한다. 그래도 못 풀면 지금처럼 `ik_fail`이다.
   - 집기 기억이 없는 놓기도 옛 방식으로 간다. `place_height_mm`는 낮은 턱 끝이 아니라 **손 기준점** 높이다(지금 코드).
   - 옛 방식으로 가면, 기울여 집은 물체는 τ만큼 기운 채 놓인다. 받아들이고 이유를 남긴다.
3. **여는 경로** (`_opening_path`):
   - 중심과 낮은 턱 끝 높이는 −90° 고정 대신 지금 손 pitch(FK)로 계산한다.
   - 경로 종류를 아래 순서로 시도한다. 종류마다 여유를 `place_clearance_mm`(30)에서 5mm씩 줄여(최소 10mm) 처음 통과한 것을 쓴다.

| 순서 | 경로(`route`) | 언제 | 모양 |
|---|---|---|---|
| ① | `tilted` | `grip_tilted`이고 `tilt_allowed`가 참 | 닫는 경로를 거꾸로(`tilt_ok=True`). 기울기는 τ_s에서 `tilt(v)`로 커진다. 끝 틈 = min(쥔 틈 + 2·여유, 대칭 상한 틈) |
| ② | `released_then_tilted` | `grip_tilted`가 아니고 `tilt_allowed`가 참이며 물체 높이를 앎 | 두 구간. **A 놓기**: 기울기 0으로 틈을 쥔 틈 + `place_release_mm`(새 설정, 6mm)까지 벌린다(한쪽 3mm, 첫판과 같은 수직 대칭 이동), `place_open_s`의 25%. **B 기울이기**: 남은 벌림 동안 기울기 = `tilt(v)`·k, k는 그리퍼 값에 비례해 0→1, `place_open_s`의 75%. 끝 틈은 ①과 같다 |
| ③ | `fixed_tilt` | ①·②가 안 될 때(대칭 상한 틈 − 쥔 틈 < 2·10mm인 큰 물체 포함) | 기울기를 τ_s로 고정한 채 첫판처럼 중심을 지키며 벌린다. τ_s = 0이면 첫판과 같다. 끝 틈 = 쥔 틈 + 2·여유(그리퍼 최대 벌림까지) |
| ④ | 경로 없음 | ③도 안 될 때 | 지금처럼 제자리에서 `gripper_open`으로 열고 `ARM-OPENING-PATH-FALLBACK`(이유)를 남긴다. 이 이벤트는 ④에만 쓴다 |

   - ②를 만들려면 `Gripper.closing_path`에 기울기 배율 인자를 더한다. 단계마다 `hand_pose`의 `tilt_scale`에 넘길 값을 받는 `tilt_scale=(k0, k1)`이고, 그리퍼 값에 비례해 선형으로 바뀐다. 기본 (1, 1)이면 지금과 같다. A 구간은 `tilt_ok=False`로 부른다.
4. **경로 검사**(종류마다, 여유마다):
   - 경로의 **모든 단계**에서 `safety.check_goal(pts[i], pts[i-1])`(장면 부품 `scene:*`, `scene_floor`, 바닥, 작업 영역, 관절 한계)를 본다.
   - 두 턱 끝이 알려진 물체(`placed`, `known_objects`)의 원에 들어가는지도 단계마다 본다(지금의 시작-끝 선분 검사를 단계마다로).
   - ②는 하나를 더 본다. 쥔 물체 상자(폭 = 물체 폭을 알면 그 값(`obj.width_mm`), 모를 때만 쥔 틈. 쥔 틈을 쓰면 높이 40mm 물체가 이 검사에 걸려 ②로 놓이지 못한다, 높이 = `obj.height_mm`, 바닥 = 작업 면 위 `place_drop_mm`)의 옆면과 두 턱 선분이 단계마다 2mm 이상 떨어져야 한다. 턱 선분은 고정 턱이 손 기준점→고정 턱 끝, 가동 턱이 가동 턱 회전축→가동 턱 끝이고, 턱 축이 들어간 수직 평면에서 2D로 본다. 걸리면 이유 `tilt_hits_object`로 ③으로 간다. 물체 높이를 모르면 ②를 건너뛴다(이유 `height_unknown`).
5. **다 벌린 뒤**(첫판과 같음): 그 틈과 기울기를 유지한 채 지금 자리에서 `approach_height_mm`만큼 수직으로 올라간다. 원래 하강 경로를 거꾸로 타지 않는다(물체를 민다). 그리퍼는 연 채로 둔다. 놓은 물체는 `placed`에 더한다.
   - **상승 높이의 최소**: hover와 같은 방식(원하는 값부터 5mm씩)으로 올라가되, 최소 **물체 높이 + 10mm**(물체 높이를 모르면 t_g + 30mm)를 지켜야 한다. 그만큼 못 올라가면 다음 수평 이동을 하지 않는다. 그 자리에서 `stopped`(이유 `low_lift`, `ARM-PLACE-LOW-LIFT`를 WARN에서 ERROR로)로 멈추고 사람이 정한다. 지금은 경고만 남기고 다음 이동을 해서, 턱이 방금 놓은 물체를 쓸 수 있다.
6. **기록**:
   - `ARM-OPENING-PATH`에 키를 더한다.
     - `route`
     - `tilt_start_deg`, `tilt_end_deg`(계획)
     - `jaw_angle_fixed_deg`, `jaw_angle_moving_deg`([시작, 끝], 실제 관절로 계산)
     - `sym_err_max_deg`(계획 경로의 대칭 오차 최대. ②는 B 구간만), `actual_sym_err_end_deg`
     - `yaw_deg`, `skipped`(건너뛴 경로와 이유 목록)
   - 기존 키(`actual_fixed_tip_move_mm` 등)는 그대로 둔다.
   - 옛 방식으로 갔으면 `ARM-PLACE-LEGACY`(INFO, 이유)를 남긴다.

**바꿀 파일**: `arm/skills.py`(`place_at`, `_opening_path`, `_opening_blocked`, `_emit_opening`, 집기 기억), `arm/gripper.py`(`closing_path`의 `tilt_scale`), `config/robot.yaml`(새 키 `place_drop_mm: 3`, `place_release_mm: 6`)과 설정 스키마, 테스트.

### 11.3.4 완료 기준
- 시뮬레이션: 폭 30·45·58mm(높이 40mm), 놓는 곳 배출 구역과 (300, 200)에서 다음을 모두 만족한다.
  - 배출 구역은 반경 90~250mm 안에서 옮길 수 있다(`workspace.yaml`). 대칭 V자(①·②)는 sim 기준 반경 약 200mm까지 된다(넓힌 손목 한계면 폭 30mm는 225mm까지). 그보다 먼 곳의 넓은 물체가 ③`fixed_tilt`로 가는 것은 팔 구조의 한계로 받아들인다. 그래서 아래 기준은 반경 200mm 안의 놓는 곳에 적용한다.
  - ②의 기울기 배율은 k(v) = 1 − θ_A(1 − s)/θ(v)로 구현한다(θ_A는 A 구간 끝의 θ, s는 B 구간 진행 0→1). 이렇게 하면 "B 구간에서 대칭 오차가 줄어들기만 한다"는 기준이 지켜진다. 11.3.3의 "그리퍼 값에 비례해 0→1"을 이 식으로 바꾼다.
  - `tilted`: 벌리는 동안 대칭 오차 3° 이하, 끝에서 α_f·α_m 각각 15° 이상, 두 턱 중간점 수평 이동 3mm 이하
  - `released_then_tilted`: A 구간은 대칭을 재지 않는다. B 구간에서 대칭 오차가 줄어들기만 하고, 끝에서 3° 이하·각 15° 이상, 중간점 이동 3mm 이하
  - 물체 바닥 높이(= release의 낮은 턱 끝 높이 − 작업 면 − t_g)가 0~5mm
  - 배출 구역 점에서 놓는 턱 축 방향으로 ±70mm에 폭 58mm 물체 2개(알려진 물체)가 있을 때 3번째 놓기가 부딪힘 없이 끝난다. 여유가 줄어든 경우 `clearance_mm`에 남는다.
  - 높이를 모르는 물체, 대칭 상한을 넘는 큰 물체(폭 75mm)는 `fixed_tilt`로 가고 `skipped`에 이유가 남는다.
- 실물(runbook 2-2):
  - 놓기 5/5 성공
  - 놓은 물체가 5mm 넘게 밀리지 않고 쓰러지지 않음. 배출 구역에 mm 눈금 종이를 붙이고 위에서 고정한 휴대폰으로 놓기 전후를 찍어 잰다.
  - 사용자가 영상으로 "양쪽으로 대칭으로 벌어진다"를 확인

## 12. 미정 사항
- 팔 배치(모서리 / 긴 변 가운데)와 작업대 크기: T6 줄자 실측 후 확정. 코드는 `workspace.yaml`만 바뀐다. 허용 범위는 임의로 늘리지 않고 실측값을 넣는다(`decisions/0031`).
- `table_z_mm` 실측과 손끝 정확도(`decisions/0031`의 접촉 측정). 높이 0 근처와 바닥 안전 검사의 정확도가 이 값에 달려 있다.
  1. `keys` 좌표 모드로 손끝을 책상 위 4곳(작업대 네 모서리 근처)에 차례로 가져가 `d 0 0 -1`로 1mm씩 내린다. 종이 한 장이 끼여 빠지지 않는 순간 멈춘다(누르지 않는다). 바닥 검사에 걸리면 이번만 `--safety-margin`이 아니라 바닥 높이를 낮춘 `--box`를 쓴다.
  2. 토크로 자세를 유지한 채 관절값을 기록하고 FK로 그 점의 모델 z를 구한다. 네 점의 평균이 `table_z_mm`, 차이가 책상 기울기다.
  3. 손끝 앞 거리: 책상에 mm 격자를 인쇄해 붙이고, 같은 높이·같은 x에서 y만 50mm씩 다른 세 교차점에 손끝을 댄다. 실측 차이와 FK 차이를 비교한다(J1 축 기준점 오차가 상쇄된다). 각 점은 두 방향에서 접근해 두 번 잰다.
  4. 결과는 `measurements.md`에 남기고, 차이 비교가 2mm 이내면 운동학은 그대로, 일정한 δ°가 보이면 `joint_offsets_deg` 보정을 기획 세션에 올린다.
- 해석적 IK 추가 여부: T6 실측 결과로 결정(시뮬레이션 왕복 오차는 1mm 이내).
- 집는 높이, 시험 물체(높이 약 40mm 이하 제약 포함): 팀 논의 8번.
- 그리퍼 재설계(조원)와 실물 집기 성공률(T7).
- 전원 계통: 서보 6개 동시 토크 꺼짐 3회. 커넥터 점검과 어댑터 비교, 물리 비상 정지 스위치(12V 차단).
- 안전 설정에 속도 상한 추가, 서보 시동 힘 시험.
- shoulder_pan 가동 범위가 239°로 PR #4826의 SO-101 값(194°)보다 큰 이유(받침 스토퍼 차이 또는 수동 기록 오류). AC4 전에 실물로 확인.
- 보드(CH343)가 USB 시리얼 번호를 주는지 실물 확인(`doctor`). 주지 않으면 팔 고르기는 `local.yaml`의 `arm`이나 선택 메뉴로 한다.
