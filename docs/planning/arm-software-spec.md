# 로봇팔 소프트웨어 설계서

상태: 확정 · 갱신 2026-11-02 · 2026-11-14 13절을 mujoco-spec.md로 옮김 · 11.3 놓기 동작 모양 2026-11-07, 대칭 V자 개정 2026-11-10, 상승 최소·상자 폭·높이 모름 집기 2026-11-11, 13 MuJoCo 2026-11-11 · 2026-11-13 drift-report(main ec9fd73) 반영, 막힘 감지 문턱 속도별(결정 0061, 11-14), MuJoCo 시험장 계획(13.10 초안, 11-14), 집기 기울기 기본값 equal_tips(결정 0060), 막힘 감지(결정 0059, 6.5), M4 GUI는 `gui-spec.md` 12.8(13절 M4 부분은 초안, gui-spec 12.8과 함께 확정) · 2026-11-16 6.10 정책 실행(결정 0066, 초안) · 2026-11-16 밤 6.10 Stage 9 (나) 반영 · 2026-11-17 6.6 쥠 확인(닫힌 폭이 벌린 폭 − `grip_min_close_mm`보다 넓으면 `no_object`, M5 거짓 성공) · 2026-11-18 6.6.1 접근 각도 후보 고르기(결정 0069, 초안)

로컬 개발 세션이 이 문서만 보고 팔 모듈을 구현할 수 있도록, 구조·인터페이스·출력값·작업 순서를 정한다. 범위는 개발 0~2단계(구동 점검, 동작 라이브러리, 작업 공간 확정)다. 인식 모듈과 GUI는 팔 모듈과 맞닿는 인터페이스만 정한다.

상위 문서: `system-architecture.md` (모듈 경계, 공통 타입, 폴더 구조).

근거: `decisions/0003`(LeRobot 제어 스택), `0004`(인식–팔 분리), `0005`(개발 순서), `0006`(장치 구성), `features.md`. 구현 결과는 `decisions/0008~0019`, `arm-dev-status.md`, `measurements.md`에 있고 이 문서에 반영했다(2026-11-02). 설정의 실제 값은 `config/`가 기준이다(공통 값은 `robot.yaml`, 팔별 값은 팔 파일, 겹치는 순서는 7.1).

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
[기능]      skills: home, park, pick, place    teleop: 키보드 입력 어댑터    teach: record, replay, run_sequence
              │
[동작]      MotionController (50Hz 실행 루프, 정지·선점, jog)   trajectory: MoveJ, MoveL(직선·꺾은선·부드러운 선), 교시 재생
              │
[기구학]    Kinematics: FK, IK (placo 기본, 교체 가능)
              │
[장치]      ArmDriver (LeRobot SO101Follower)  |  SimDriver (기구학 엔진)  |  MuJoCoDriver (물리 엔진, 13절)

[공통]      types · frames(좌표 변환) · config · safety · logging
```
- 위 계층은 아래 계층만 부른다. GUI는 `ArmService` 외에는 아무것도 import하지 않는다.
- `SimDriver`는 명령받은 위치를 그대로 현재 위치로 돌려주는 가짜 장치다. 팔 없이 GUI·로직을 개발하고 테스트하는 데 쓴다.

## 3. 폴더 구조
레포 전체 구조는 `system-architecture.md` 9절을 따른다. 팔 모듈 파일은 아래와 같다.
```
src/so101_studio/arm/
  driver.py, logged_bus.py, reset_probe.py, sim_driver.py, mujoco_driver.py, mujoco_model.py,
  kinematics.py, trajectory.py, motion.py, safety.py, collision.py, gripper.py,
  skills.py, teleop.py, teach.py, fault_text.py, autocal/
src/so101_studio/service/
  arm_service.py, params.py, diagnostics.py, stop_listener.py, studio.py, targets.py,
  perception.py, ultrasonic.py
```

## 4. 좌표계와 단위
| 항목 | 규칙 |
|---|---|
| 작업대 좌표계 (`table` = world) | 원점 = 작업대 왼쪽 아래 모서리, x = 긴 변 방향, y = 짧은 변 방향, z = 위. 인식 모듈과 GUI는 이 좌표만 쓴다 |
| 로봇 좌표계 (`robot` = base) | 원점 = 어깨 회전축, x = 앞, y = 왼쪽. 작업대 좌표계와의 관계(x, y, 회전각)는 `workspace.yaml`에 둔다 |
| 툴 좌표계 (`tool`) | 상대 이동만 받는다. z = 도구 앞, x = 턱이 벌어지는 축. 5축이라 자세 변경(기울이기)은 받지 않는다 (`decisions/0013`) |
| 높이 기준 | 모든 좌표계에서 높이는 **작업대 면 위 mm**다. 작업대 면의 모델 높이는 `table_z_mm`(팔 파일, 현재 −15, 2026-11-07 실측) |
| 입력점 (TCP) | `tip` = 고정 턱 끝(기본), `ref` = 고정 턱 안쪽 면(URDF `gripper_frame` 점) |
| 외부 단위 | 길이 mm, 각도 도(°). 서비스·인식·GUI 사이의 모든 값 |
| 내부 단위 | 기구학·궤적 계산은 m, rad. 변환은 `frames.py`에서만 |
| 물체 각도 | 작업대 x축 기준 반시계 방향, 0~180° (0/45/90/135는 인식 모듈의 분류값) |
| 그리퍼 | 0(닫힘)~100(완전 열림), LeRobot 정규화 값. 폭(mm) 대응표 `gripper_width_mm`은 URDF 기하 + 카드 54mm 실측 2점(값 32.0, 27.7). 20~30mm, 70~90mm는 미확인 |

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
    fault: str | None = None
    current_ma: float = 0.0  # 서보 전류 합(추정)
    fault_text: str | None = None  # fault 원인 문장(11.2, arm/fault_text.py)

@dataclass
class MotionResult:
    status: str            # "ok" | "ik_fail" | "out_of_bounds" | "stopped" | "preempted" | "fault" | "timeout"
                           # | "height_required"(6.6) | "no_object"(집기에서 접촉 없음) | "locked"(캘리브레이션 확인 전, 11.1)
    duration_s: float; final_tool: ToolPose; position_error_mm: float | None = None
    reason: str | None = None  # 거부·멈춤 이유 문장. stopped의 이유는 앞머리로 구분한다(예: "low_hover: …", "low_lift: …", "drop_zone_full: …", "known_object: …")
```

## 6. 모듈 명세

### 6.1 장치: `ArmDriver` / `SimDriver`
| 메서드 | 동작 |
|---|---|
| `connect(port, calibration_id)` / `disconnect()` | 연결. 연결 시 PID(16/0/32)와 서보 가속 한계(팔 10, 그리퍼 25, `decisions/0014`) 적용. 시리얼 포트는 잠금으로 한 스레드만 쓴다 |
| `read_joints() -> JointState` | 현재 위치 (sync_read). 상태 루프가 10Hz로 부르고, 이 값으로 추종 오차(마지막 명령 − 실제)를 본다(6.5 막힘 감지, 결정 0059). 막힘 감지 때문에 버스 읽기를 늘리지 않는다 |
| `write_joints(JointState)` | 목표 위치 (sync_write, 응답 대기 없음) |
| `read_health() -> dict` | 온도, 부하, 전압, 전류, `Torque_Enable`, `Status`. 2Hz로만 호출 |
| `set_torque(on: bool)` | 토크 켜기/끄기 |

- 모든 메서드는 장치 잠금(`driver._lock`, `_locked` 데코레이터) 안에서 버스를 쓴다. 그래서 여러 스레드가 불러도 버스에는 한 번에 하나씩 간다(6.4).
- 하드웨어 없는 드라이버는 둘이다. `SimDriver`(기구학 엔진, 명령을 그대로 현재 위치로 돌려줌)와 `MuJoCoDriver`(물리 엔진, 13절)다. 둘 다 `simulated = True`다.

### 6.2 기구학: `Kinematics`
| 메서드 | 동작 |
|---|---|
| `fk(JointState) -> ToolPose` | 순기구학 |
| `ik(ToolPose, seed: JointState) -> JointState | None` | 역기구학(최대 30회 반복). 관절 한계는 풀이기에 넣어 한계 밖 해를 만들지 않고, 풀린 해의 위치 오차가 `tol_mm` 3mm를 넘을 때만 None |
| `step(...)` | 조그용 한 걸음. 방향 가중치를 0.1 → 0.01 → 0으로 낮춰 가며 푼다 |

- 구현: LeRobot `RobotKinematics` + placo 0.9.25. Windows에서는 pip가 실패해 conda-forge로 설치한다(`arm-hardware-setup.md`). FK 원점은 어깨 회전축으로 보정했다.
- IK에 우리 관절 한계(`joint_limits_deg`)를 넣는다. 방향 가중치는 0.1(1.0이면 처짐 자세에서 위치 해가 3mm 넘게 어긋남).
- IK 한계는 늘 `joint_limits_deg`다. 음수 `joint_margin_deg`(6.5)로 안전 한계를 넓혀도 IK 한계는 넓어지지 않는다.
- **잠금**(#285): placo는 스레드 사이에 안전하지 않다. 그래서 `Kinematics`의 공개 메서드는 `locked` 데코레이터로 `Kinematics.lock`(RLock)을 쥐고 풀이기 상태를 맞춘 뒤(`_sync`) 계산한다. 그리퍼(`arm/gripper.py`)도 같은 `kin.lock`을 쓴다. 잠금이 없을 때 GUI와 작업 스레드가 동시에 풀면 거짓 `ik_fail`이 났다.
- `Skills._ik`는 첫 시드가 실패하면 고정 시드 3개(`SEEDS`)로 다시 시도한다.
- 시뮬레이션에서 수직 하강 IK 왕복 오차는 1mm 이내다. 실제 끝 위치 정확도(T6 줄자 실측)는 아직 없어서, 해석적 IK 추가 여부는 그 뒤에 정한다.
- 수직 하강으로 닿는 영역(계산): 높이 0~20mm에서 회전축 앞 80~260mm, 최대 높이 약 80mm. 작업 영역 판단은 `workspace.yaml`의 `reach_mm`(90~250)을 쓴다.

### 6.3 궤적: `trajectory.py`
| 함수 | 동작 |
|---|---|
| `move_j(q_start, q_goal, v_max, a_max) -> Trajectory` | 관절 공간 5차 다항식(기본). `trajectory_profile: ruckig`이면 속도·가속도·저크 한계 시간 최적(같은 한계에서 약 20% 빠르지만 떨림이 15~25% 큼) |
| `move_l(kin, start, goal, seed, v_mm_s, a_mm_s2, step_mm, profile, j_mm_s3) -> Trajectory` | 직선 경로를 `step_mm`(기본 5mm) 간격으로 나눠 각 점 IK. 한 점이라도 실패하거나 이웃 점 사이에 관절이 `MAX_WAYPOINT_JUMP_DEG`(15°)보다 크게 튀면 전체 거부 |
| `polyline_l(...)`, `smooth_l(...)` | 꺾은선 경로와 모서리를 다듬은 경로(놓기·피하기 경로에 씀) |
| `SequenceTrajectory` | 여러 궤적을 이어 하나로 실행 |

- 경유점 사이를 블렌딩하는 `move_p`는 만들지 않았다. 교시 재생은 기록 시각대로 샘플을 따라간다(6.8).

`Trajectory`는 `sample(t) -> JointState`와 `duration_s`를 가진다. 참고: Robotics Toolbox for Python의 `jtraj`, `ctraj`, `mstraj`. 경로는 실행 전에 0.1초 간격으로 안전 검사한다. 직선이 안 되면 `goto`는 목표 위 접근 높이로 MoveJ한 뒤 수직으로 내려간다.

### 6.4 동작 실행: `MotionController`
- 별도 스레드에서 50Hz로 돈다. 매 주기마다 궤적을 샘플링하고, 안전 검사를 거쳐 `write_joints`를 호출한다.
- **장치 접근**(2026-11-13 정정, 판단 J3):
  - 쓰기(`write_joints`, 토크·가속·조임 한계)는 `MotionController`를 거친다. 틱 밖에서 쓰는 `stop`·`estop`·fault 처리·`hold_gripper`도 `MotionController`의 잠금 안에서 쓴다.
  - 읽기(`read_joints`, `read_health`)는 상태 루프, `ArmService`, `Skills`가 직접 부를 수 있다. 버스 직렬화는 장치 잠금(6.1)이 한다.
  - 다른 계층은 동작을 명령(`execute`·`stop`·`retarget`·`jog`)으로만 요청한다.
- 제어 주기는 50Hz가 최선이었다(100·200Hz와 5~10Hz 모두 더 떨림, `decisions/0014`).

| 메서드 | 동작 |
|---|---|
| `execute(traj) -> MotionHandle` | 실행 시작. `handle.wait() -> MotionResult` |
| `stop()` | 감속 0.2초로 정지 후 현재 위치 유지(실측 0.24~0.28초) |
| `estop()` | 즉시 현재 위치 고정, 큐 비움, 상태 `stopped`. 토크는 끄지 않는다(끄면 팔이 떨어짐) |
| `retarget(goal)` | 진행 중인 동작을 현재 위치·속도에서 새 목표로 매끄럽게 교체(Ruckig). 이전 동작의 결과는 `preempted` |
| `jog(...)` | 원격 조종 속도 입력을 실제 경과 시간으로 적분해 목표를 갱신(틱 간격 요동 보정). `teleop.py`는 키보드 입력을 이 명령으로 바꾸는 어댑터다 |

### 6.5 안전: `safety.py`
매 주기, 명령을 보내기 전에 검사한다. 하나라도 어기면 그 명령은 보내지 않고 `stop()`한다.
- 관절 한계: 캘리브레이션으로 잰 가동 범위에서 1~3° 안쪽(`joint_limits_deg`). 팔꿈치 접는 쪽은 95°(96°에서 J2 모터에 닿음).
- 작업 영역: 그리퍼 끝이 작업대 경계 안(여유 10mm). 바닥: 낮은 턱 끝이 작업대 면보다 `tip_floor_mm`(5mm) 위.
- 한 주기 이동량: 관절당 최대 5° (LeRobot `ensure_safe_goal_position`).
- 온도 60°C 이상 3회 연속(읽기 튐 걸러냄), 부하 90% 이상 3초, 통신 끊김 0.5초, **전류 합 4A 이상 2회 연속** → `fault`.
- 원격 조종 중 입력이 0.3초 동안 들어오지 않으면 정지 (데드맨).
- 한계 밖(받침에 기댄 팔꿈치 약 97° 등)에서 출발한 동작은 더 바깥으로 가지 않는 한 허용해 빠져나올 수 있다. 관절 한계, 바닥, **작업대 x·y 모두** 같다: 작업대 밖에서 출발하면 벗어난 거리가 출발점보다 늘지 않는 점만 허용한다(`decisions/0031`).
- 통신 끊김 뒤: fault 상태에서는 다시 [연결]하기 전까지 아무 명령도 보내지 않는다. 다시 연결하면 현재 위치와 토크를 읽고 명령값을 실제값에 맞춘 뒤에만 다음 동작을 받는다. 다시 연결할 때 토크를 끄지 않는다.

**막힘·접촉 감지 즉시 정지** (결정 0059, 문턱은 0061, 2026-11-14 코드(#307·REQ-067)에 맞춤). 위 부하 감시(관절 90% 3초, 그리퍼 60% 1초)로는 순간 접촉을 못 잡아서 더한다. 부하 감시는 두 번째 안전망으로 그대로 둔다.
- **신호**: 관절마다 추종 오차 e = 마지막 명령 − 실제 위치(°).
  - 실제 위치는 상태 루프가 10Hz로 읽는 값(6.1)이고, 명령은 그 순간 `MotionController`의 마지막 명령이다. 버스 읽기를 더하지 않는다.
  - `Present_Load`·`Present_Current`는 쓰지 않는다. STS3215 위치 모드에서 `Present_Load`는 e에 비례하고 작은 값은 0으로 떨어지며, `Present_Current`는 몇 단위뿐이다.
- **프로필**(지금 동작으로 고른다): `empty`(빈 손 이동), `held`(쥔 채 이동, 집기 기억이나 `hold_gripper`가 있음). 그리퍼 닫기(집기 밖)는 따로 본다.
- **구간**: 동작 시작 뒤와 끝 앞 0.2초는 가속(`accel`), 나머지는 정속(`cruise`)이다.
- **문턱**(설정, 관절 공통):

| 설정 | 값(30% 이하) | 실물 정상 최대 30% / 50% |
|---|---|---|
| `blocked_empty_accel_deg` | 10° | 5.9° / 7.1° |
| `blocked_empty_cruise_deg` | 15.5° | 6.6° / 13.8° |
| `blocked_held_accel_deg` | 8° | 2.8° / 3.1° |
| `blocked_held_cruise_deg` | 9° | 6.5° / 13.0° |
| `blocked_grip_units` | 10눈금 | 2.1 / 3.9 |

- **속도별 적용**(결정 0061, 동작 속도 s %):
  - s ≤ 30: 위 값 그대로다. 30% 아래에서는 정속 오차가 속도와 상관없이 비슷하다(서보 데드밴드·중력이 바닥값).
  - 30 < s ≤ 50: **정속 문턱만** s/30배로 늘린다(50%에서 쥔 채 15°, 빈 손 25.8°). 가속 구간과 그리퍼 닫기 문턱은 그대로다.
  - s > 50: 판정하지 않는다(막힘 감지가 꺼진다). 기록(`ARM-TRACKING`)만 남긴다. 그보다 빠른 속도의 문턱은 기록이 모인 뒤 다시 정한다. 그래서 시연은 50% 이하를 권장한다.
- **판정**: 같은 관절에서 |e| > 문턱이 연속 2회(약 0.2초)면 막힘이다.
- **반응**:
  - 팔 관절이 막히면 모든 관절의 목표를 지금 위치로 맞춰 멈춘다. 지금 동작 결과는 `stopped`(이유 `blocked: <관절> 추종 오차 x > 문턱 y (<프로필>)`)이고, WARN `ARM-BLOCKED`(관절, 오차, 문턱, 프로필)를 남긴다. 다음 동작은 사람이 고른다(자동 후퇴 없음).
  - 그리퍼: 집기 밖에서 닫을 때(`set_gripper`, 자세 이동에서 그리퍼가 닫히는 쪽으로 갈 때, 동작이 끝난 뒤 잠깐) 실제 − 명령이 `blocked_grip_units`를 넘으면 접촉으로 보고 결정 0033대로 "지금 위치 + 예압"으로 고정한다. 그래서 이미 쥔 물체 위로 다시 닫아도 더 조이지 않는다. 집기 안의 닫기는 `contact_lag_units`(4, 6.6)가 먼저 판단한다.
- **기록**(REQ-067): 동작이 끝날 때마다 INFO `ARM-TRACKING`(프로필, `speed_ratio`, 가속·정속 구간의 오차 최대와 관절, 그리퍼 닫기 오차)을 남긴다. 문턱은 이 기록의 정상 실행 최대 × 1.3 이상으로 정하고 `measurements.md`에 남긴다.
- **시험**: MuJoCo는 판정 논리만 본다(home 경로에 장면 상자를 두면 `stopped(blocked…)`, 빈 이동에서 오탐 0). MuJoCo 액추에이터의 추종 오차는 실물 서보와 달라서 문턱 숫자는 실물에서만 정한다.
- 모든 안전 모드(`--unsafe` 포함)에서 켜져 있다(50% 초과 제외). 토크가 꺼진 동안(교시 녹화)은 판정하지 않는다.

안전 설정 `SafetySettings`는 실행 중에 바꿀 수 있다 (`decisions/0015`).
| 항목 | 값 |
|---|---|
| `mode` | `normal`(관절 한계 + 작업대 + 바닥) / `free`(작업대 x, y만 끔) / `off`(관절 한계·작업 영역·바닥 끔). 한 주기 이동량, 과열·과부하·통신·과전류 감시는 모든 모드에서 켜짐 |
| `joint_margin_deg` | 양수 = 한계를 안쪽으로, 음수 = `physical_limits_deg`까지만 넓힘 (그리퍼 제외) |
| `box_mm` | 작업대 좌표 x, y와 작업대 면 위 턱 끝 높이 z의 허용 범위 |
| 검사(`set_safety`) | `joint_margin_deg`는 −15~30, `box_mm`의 z 아래쪽은 0 이상. None을 넘기면 기본값 |
| 남은 일 | 속도 상한(`max_speed_pct`)을 안전 설정에 넣기 |

### 6.6 기능: `skills.py`
| 함수 | 순서 |
|---|---|
| `home()` | 정해진 기본 자세로 MoveJ |
| `park()` | 센서 시야 밖 대기 자세로 MoveJ. 스캔 전에 반드시 호출 |
| `pick(obj: DetectedObject)` | 접근 높이(`approach_height_mm` 40 + 집는 높이 `grasp_height_mm` 20)로 MoveJ → 턱 축을 물체 각도 + 90°로 돌리고 벌림 → 하강 MoveL → 닫으며 손을 같이 보내 중심 유지 → 접촉에서 정지하고 목표를 지금 위치 + 예압으로 → 들고 있는 동안 조임 한계 `gripper_hold_torque` → 수직 상승. 접촉이 없으면 올라온 뒤 그리퍼를 열고 `no_object` |
| `place(zone)` / `place_at(x, y)` | 놓는 순서는 11.3. 배출 구역 이름이면 빈 칸을 고르고(11.3.5), 좌표면 그 점에 놓는다 |

- `run_sequence(name)`은 `skills.py`가 아니라 `teach.py`에 있고 `replay(name, 1.0)`과 같다(6.8).

- 그리퍼가 고정 턱 1개 + 가동 턱 1개라 아래 보정을 한다 (`decisions/0009~0012`).
  - 턱 축 = 물체 각도 + 90°. 180° 돌려도 같은 파지라 손목이 덜 도는 쪽을 고른다. 물체 각도가 None이면 0°.
  - 벌림(`grasp_open_policy`, 지금 설정 `max`):
    - `max`: 폭과 높이를 모두 알면 `_needed_gap`이 W + `gripper_margin_mm`(20)부터 2mm씩 늘려, 설정 집는 높이를 올리지 않고 11.2 통과 조건을 만족하는 가장 작은 틈을 고른다(REQ-036). 그 안에 없거나 폭·높이 중 하나라도 모르면 `grasp_max_open_mm`(90mm)다. 틈의 중심을 물체에 맞춘다.
    - `fit`: 폭을 알면 W + `gripper_margin_mm`, 모르면 최대.
    - 닫는 끝은 폭을 알면 폭 − 10mm(접촉 감지에 필요한 명령 차이), 모르면 끝까지다.
  - 닫을 때 틈이 d 줄면 손을 가동 턱 쪽으로 d/2 같이 보내 닫는 내내 중심을 유지한다.
  - 턱이 앞뒤로 벌어질 때만 손을 기울인다(`grasp_tilt_mode`, 최대 `grasp_tilt_deg` 35°). 기본 `equal_tips`(결정 0060)는 두 턱 끝 높이가 같아지는 기울기로, 두 턱 끝이 함께 `grasp_height_mm`에 와서 깊게 문다(58mm 상자에서 약 4.7°, 거의 닫히면 0). 옵션 `symmetric`(0012)은 벌림 각도의 절반만큼 기울여 보기에 대칭인 V자로 닫지만, 낮은 턱 끝만 `grasp_height_mm`에 와서 넓은 물체는 높은 턱이 얕게 문다(실물 58mm 상자에서 9mm, `no_object` 2/2).
  - **기울일 수 있는 방향**(5축 팔): 손은 반경 방향(팔이 뻗은 방향)으로만 기울일 수 있어, 턱 축이 반경에서 `grasp_tilt_tol_deg`(15°) 안일 때만 기울이고 그 밖에서는 수직으로 내려간다(가동 턱 하나만 크게 벌어짐).
    - **정사각형 물체**(폭 = 길이): 두 변 중 턱 축이 반경에 가까운 쪽을 고른다(2026-11-14 사용자 결정 B, 팔 PR). 그래서 물체 각도 75~90°(90° 정사각형은 0°와 같음)에서도 기울이기가 살아난다. 15~75°는 수직 집기다.
    - **45° 정사각형은 한계로 인정한다**(결정 A): 턱 축 후보 45°·135°가 둘 다 반경에서 45° 벗어나므로 수직으로 집는다. 높이를 모르면 결정 0052대로 `height_required`다.
  - 접촉 판정: 실제 그리퍼 값이 명령보다 4값 넘게 앞서 멈춤(`contact_lag_units`).
  - **쥠 확인**(2026-11-17, M5 거짓 성공): 접촉으로 멈췄어도(늦은 접촉 `ARM-PICK-LATE-CONTACT` 포함) 닫힌 뒤 턱 끝 벌림(폭 표로 바꾼 값)이 벌린 폭 − `grip_min_close_mm`(5mm)보다 넓으면 쥐지 못한 것으로 본다. 턱이 물체 윗면에 얹혀 거의 닫히지 못한 경우다(M5 `inject_grid` 병렬 58mm 상자, 높이 오차 0·+20mm, 벌림 축 어긋남 20mm: 벌린 폭 78mm, 닫힌 폭 약 81mm, 보고 `ok`·물리 `untouched` 4건).
    - 기준은 알려 준 물체 폭 W가 아니라 **벌린 폭**이다. 초음파 작업 실행의 W는 CHECK 추정값(구간 길이 − 빔 반경 가정)이라 10~20mm 틀릴 수 있어 W 기준이면 제대로 쥔 것도 실패로 본다. 벌린 폭은 W를 알면 W + 20mm 이상(`grasp_open_policy`)이라 제대로 쥐면 턱이 20mm 가까이 움직이고, W를 모르면 최대로 벌리므로 같은 규칙을 쓴다(W ≤ 벌린 폭 − 4mm인 물체만 집으므로 걸리지 않는 폭은 벌린 폭 − 5~4mm 사이뿐이다).
    - 결과: 접촉이 없을 때와 같이 처리한다. 조임 한계를 되돌리고 `known_objects`는 그대로 두고 `grip`(11.3.2 기억)은 비우고, 접근 높이까지 올라온 뒤 열고 `no_object`를 돌려준다. 새 상태 이름을 만들지 않는 것은 작업 실행(수직 집기 `no_object` 뒤 다시 CHECK, 결정 0052)·GUI 문구·sim 허용(`accept_no_object`)이 이미 `no_object`를 실패로 다루기 때문이다. WARN `ARM-PICK-NO-GRIP`(벌린 폭·닫힌 폭 mm)을 남기고 `last_pick_info`에 `no_grip: true`와 두 폭을 적는다.
    - 단동식·병렬 모두 같다(폭 표가 그리퍼별). 시작값 5mm는 M5 `pick_grid`·`inject_grid`를 다시 돌려 거짓 성공 0과 거짓 실패 0을 함께 확인한 값으로 고정한다. 거짓 실패가 나오면 값을 고치지 말고 그 에피소드를 보고한다. 실물은 runbook 2-10(로보나인 폭 표) 뒤 다시 본다.
    - 막힘 문턱(6.5)은 바꾸지 않는다. 반경 120mm ±40°의 `blocked` 24건과 맞물려 있어서다(총괄 안 (나)는 고르지 않음).
  - 조임 (`decisions/0033`): 접촉하면 목표를 지금 위치에서 **닫는 쪽으로**(그리퍼 값이 작아지는 쪽, 0 = 닫힘) `grip_preload_units`(2)만큼 더 간 값으로 고정한다(결정 0033의 "지금 위치 + 예압"은 이 뜻이다). 그리퍼를 여는 명령(`set_gripper`, `place`의 열기, `no_object` 열기)이 오면 고정을 풀고 조임 한계를 원래 값으로 되돌린다. 정지·E-STOP에서는 물체를 놓치지 않게 고정과 200을 유지한다. 조임 한계는 RAM `Torque_Limit`(48번)으로 바꾸고 EEPROM(`Max_Torque_Limit`)에는 쓰지 않는다. 닫는 동안 `gripper_squeeze_torque` 300, 들고 옮기는 동안 `gripper_hold_torque` 200(연결 때 쓰는 그리퍼 `Overload_Torque` 25% 아래), 놓은 뒤 원래 값.
  - 그리퍼 부하 감시: `gripper_load_warn_pct`를 `gripper_load_s` 넘게 넘으면 먼저 목표를 지금 위치로 맞추고 WARN, 그래도 이어지면 열고 fault. 시작값은 정상 집기 5회 부하 최대값의 1.3배, 1초.
- 접근 높이를 80mm에서 40mm로 낮췄다. 수직 하강으로 닿는 최대 높이가 약 80mm이기 때문이다. 따라서 **집을 수 있는 물체 높이에 제약이 있다**(시험 물체 선정 때 확인).
- 물체 높이: `DetectedObject.height_mm`를 알면 높이에 맞춰 집는다(11.2). 모를 때(0)는 다음과 같다(결정 0052). 이 검사는 **단동식 그리퍼에서만** 한다(위 모서리 문제는 V자 턱이 위로 좁아지는 데서 나온다. 병렬 그리퍼는 해당 없음).
  - 기울여 집을 수 있으면(`tilt_ok`) 지금처럼 `grasp_height_mm`로 집는다.
  - **기울이지 못하고(`tilt_ok=False`) 높이도 모르면 집지 않는다.** 결과 `height_required`(이유 문장: "턱 축이 팔 방향과 어긋나 수직으로 집어야 해서 물체 높이가 필요합니다")를 돌려준다. 수직으로 높이를 모른 채 내려가면 턱이 물체 위 모서리를 친다(sim 384건 중 127건, 높이를 알면 381/384 ok, 11-10 실물 배출 구역 `no_object`의 원인).
  - `tilt_ok`여도 기울인 계획이 `ik_fail`이면(멀리서 손목 한계) WARN `ARM-PICK-TILT-UNREACHABLE`을 남긴다. 그 뒤 높이를 알면 수직으로 틈·높이를 다시 계산해 한 번 더 하고(REQ-055), 모르면 `height_required`다.
  - 수동·CLI·GUI는 높이를 받는다. CLI `pick`은 5번째 값, GUI는 높이 칸이다.
  - 초음파 작업 실행은 높이를 사람에게 묻지 않는다. 대신 그 후보를 한 번 더 `CHECK`해 높이를 잰다. 그래도 모르면 `note_failure`로 넘긴다.
  - 초음파 작업 실행에서 수직 집기가 `no_object`면, 그 후보를 한 번 더 `CHECK`해 높이를 다시 재고 한 번만 다시 집는다. 두 번째도 실패하면 `note_failure`.
  - 서보 부하로 높이를 재는 방식(닿을 때까지 내려가기)은 실물 기준값이 필요하고 정밀도가 의심스러워 나중 실험 후보로만 둔다.
- **알려진 물체** (#267):
  - `Skills.known_objects`: 수동 입력 등 GUI가 채우는 (x, y, 반경) 목록. 재연결해도 유지된다.
  - `Skills.placed`: 놓은 물체 목록.
  - 둘 다 놓기의 장애물로 쓴다(11.3).
  - 집기를 시작하면 반경 40mm 안의 `placed`는 성공 여부와 상관없이 지운다(집어 갈 물체라서).
  - 집기에 성공하면 `known_objects`에서 40mm 안의 가장 가까운 항목 하나를 뺀다.
  - 높이는 담지 않는다. 이웃 물체 높이는 40mm로 가정한다(11.3.3, 판단 J2).
- 알려진 문제: 그리퍼 앞 다리가 제대로 맞물리지 않아 실제 물체를 집지 못한다. 조원이 그리퍼를 재설계한다.

#### 6.6.1 접근 각도 후보 고르기 (결정 0069, 2026-11-18 초안 — EGM Stage 9 전, 병렬 손만)
지금 병렬 손은 늘 수직(손끝 축이 아래)으로 집고 놓는다. 이것을 "기울기 후보마다 턱 방향을 계산하고, 통과한 것 가운데 고르기"로 바꾼다. 수직은 후보에 늘 들어 있어 대체가 된다. 단동식 손은 위 규칙(결정 0060 `equal_tips`) 그대로다.

**기하(5축)**: 손끝 축 a는 팔이 뻗은 방향의 세로 평면(그 물체 자리의 방위 평면) 안에서만 기울일 수 있다. a가 수직 아래에서 β만큼 팔 바깥쪽으로 기울었다고 하자(β = 0이 지금의 수직).
- 턱이 닫히는 방향 c는 a에 수직이고 손목 회전 φ로 돈다. 단위 벡터 둘을 t(작업대 수평, 방위 평면에 수직 = 접선)와 u(방위 평면 안, a에 수직, 수평에서 β 기운 것)로 두면 c = cos φ·t + sin φ·u다.
- c의 수평 성분이 접선 t와 이루는 각 ψ는 tan ψ = tan φ·cos β이고, c가 수평과 이루는 각(닫힘 기울기) γ는 sin γ = sin φ·sin β다.
- 물체의 한 면 쌍(폭 W 쪽 또는 길이 L 쪽)의 법선이 접선과 ψ만큼 돌아 있으면, 그 면을 물려면 φ = atan(tan ψ / cos β)이고 그때 γ가 정해진다. 면 법선이 접선과 나란하면(ψ = 0) γ = 0으로 수평으로 닫히고, 반경 방향이면(ψ = 90°) γ = β로 가장 기운다.

**후보와 거르기**
- 기울기 후보 `grasp_pitch_candidates_deg`: [0, 20, 35, 50](β, 시작값). 면 쌍 후보: 폭 쪽과 길이 쪽 둘(정사각형이면 둘 다 같은 폭). 각도를 모르면(None) 면 쌍 후보는 지금 기본 각도 하나다.
- 버리는 후보:
  - γ > `grasp_contact_tilt_max_deg`(25°, 시작값). 기운 닫힘이 면에서 미끄러지지 않을 한계로 잡은 값이고 M5에서 정한다.
  - 물리는 폭 W/cos γ가 `grasp_max_open_mm` − 4를 넘는 것(병렬 W ≤ G − 4를 기운 폭에 적용).
  - 낮은 쪽 턱 끝이 작업대 면 + `tip_floor_mm` 아래로 가는 것, 턱·손이 이웃 물체(`known_objects`, 11.3)나 작업대에 닿는 것.
  - 접근점(손끝에서 −a 방향으로 `approach_height_mm` 뒤)·물기 자세·들기 끝의 IK가 안 풀리거나 안전 검사(관절 한계·작업 영역·바닥)에 걸리는 것.
  - **물체 높이를 모르면(0) β > 0 후보는 모두 버린다**(기운 턱이 물체 윗모서리를 칠 수 있다. 결정 0052와 같은 이유). 그래서 초음파 작업 실행은 CHECK로 높이를 잰 뒤에만 기울인다.
- 고르기(통과한 것 가운데): ① γ가 작은 것(5° 단위로 묶음) ② `grasp_pitch_pref_deg`(35°)에 가까운 β ③ 손목 회전이 덜 도는 것. 수직(β = 0)은 다른 후보가 모두 버려질 때 남는다.
- 하강은 a 방향 직선이다(지금 수직 하강과 같은 `MoveL`). 닫기·접촉·조임·쥠 확인(6.6)은 그대로다. 벌림은 W/cos γ에 지금 규칙(`grasp_open_policy`)을 적용한다.

**놓기**: 집을 때의 β와 φ를 그대로 쓴다(그래야 물체 바닥이 평평하다). 놓을 자리의 방위가 달라도 β·φ는 손에 붙은 값이라 그대로 두면 된다. 그 β로 놓을 자리에 닿지 않으면 배출 구역의 다른 빈 칸(11.3.5)을 보고, 다 안 되면 `ik_fail`로 알린다(수직으로 바꾸면 물체가 기운다). 이 때문에 `auto`에서는 집기 후보를 고를 때 **배출 구역에서 같은 β로 닿는지도 함께 본다**(닿지 않는 후보는 버린다).

**설정과 기본값**
- `grasp_approach`: `vertical`(지금, 기본) | `auto`(위 규칙). `CHOICES`에 더한다(6.9.1).
- M5 비교: `pick_grid`·`inject_grid`를 두 값으로 돌린다. `auto`의 `placed_ok`가 `vertical` 이상이고 거짓 성공이 늘지 않으면 기본을 `auto`로 바꾼다(총괄이 표와 함께 기획에 올리고 기획이 이 절을 고친다). 반경 120mm ±40°의 `blocked`와 길이 80·100mm 90°의 `ik_fail`(M5 고정 실패)이 줄어드는지도 같은 표에 적는다.
- 기록: `last_pick_info`에 `approach: {beta, phi, gamma, face}`를 더하고, 기울인 집기는 INFO `ARM-PICK-APPROACH`를 남긴다.
- 화면: GUI 3D의 손 자세는 지금처럼 관절값으로 그리므로 바꿀 것이 없다. 미리보기 문구에 "기울여 집기 β°"를 더하는 것은 GUI 세션 몫이다.

**Stage 9 전에 확인할 것**(구현 REQ 전): ① β 후보에서 SO-101 병렬 손이 반경 120~250mm에 닿는 범위(팔 세션 기구학 계산) ② 기운 턱과 58mm 상자 윗모서리의 간섭(높이를 알 때) ③ 놓기 β 고정이 배출 구역 `right`에서 닿는지 ④ ACT 시연(결정 0068)이 `auto`로 바뀌면 다시 모아야 하는 일정.

### 6.7 원격 조종: `MotionController.jog` + `teleop.py`(입력 어댑터)
- `jog(mode, velocity)`(`MotionController`, 6.4): `mode`는 `"joint"`(관절별 °/s) 또는 `"cartesian"`(x, y, z mm/s, yaw °/s). 입력 장치와 무관하게 이 명령 하나로 받는다. 기본 속도 관절 30°/s, 직교 80mm/s.
- 키보드 `keys`: Tab으로 관절 → 직교 조그(로봇/툴 기준) → 좌표 입력 세 모드. 좌표 입력은 `point_pose` → `preview_tool`로 먼저 검사해 닿지 않으면 움직이지 않고 이유를 알린다 (`decisions/0013`).
- 입력 어댑터: 키보드(지금, Windows `GetAsyncKeyState`), GUI 가상 조이스틱과 게임패드(나중). 모두 `jog`를 부른다.
- 느린 조그에서 체감 떨림이 남아 있다(서보 내부 설정·기구 하중 의심, `Minimum_Startup_Force` 시험이 남음).

### 6.8 동작 녹화·재생: `teach.py`
토크를 켠 채 손으로 미는 추종 교시는 기어 마찰(1/345) 때문에 실패해 제거했다. 토크를 끈 방식 하나다 (`decisions/0016~0019`).
- `record_start(hz, max_s)`: 홈 자세로 이동 → 팔을 잡았다는 확인 → 3초 카운트다운 → 토크를 끄고 관절값과 **실제 시각**을 기록(기본 20Hz, 최대 90초, `record_hz`·`record_max_s`).
- `record_stop(name)`: `config/sequences/<name>.json`에 저장(길이, 점 수, 평균·최대 속도, 관절별 최대 속도 `meta` 포함)하고 토크를 켠다.
- `record_info(name, speed, segment_speeds)`: 길이·속도와 구간 목록(멈춤 < 3°/s, 느림, 빠름 ≥ 40°/s, 최소 0.5초). 반환값의 `fit`은 그 속도로 재생할 수 있는지와 변환본(11.2) 정보다.
- `replay(name, speed, until_limit, segment_speeds, mode)`: 이동평균으로 다듬어 기록 시각대로 재생. `mode`는 `"original"`(원본) 또는 `"fitted"`(변환본, 11.2). 구간별 배율 0.1~5(최대 220°/s 초과 시 거부). 한계를 넘는 기록은 거부하고, `until_limit`이면 한계 직전까지 재생한다.
- `run_sequence(name)`: `replay(name, 1.0)`(시연용 동작, 춤).
- 주의: 토크를 끄면 팔이 중력으로 처지므로, 녹화 시작 전에 팔을 손으로 받친다.

### 6.9 서비스: `ArmService`
GUI·CLI가 부르는 공개 메서드 목록이다(2026-11-13 코드 대조). 세부 인자와 기본값은 `service/arm_service.py`가 기준이다.

| 메서드 | 입력 | 출력 |
|---|---|---|
| **연결·상태** | | |
| `connect` / `disconnect` | 포트(생략 시 7.1 순서) / 없음 | (성공 여부, 메시지) / None |
| `status` / `status_stream(callback)` | / 콜백 | `ArmStatus` 1회 / 10Hz |
| `locked_reason` / `unlock_for_verification` / `relock_zero` | | 0° 확인 잠금(11.1): 이유 문장 또는 None / 확인 명령만 잠깐 풀기 / 다시 잠그기 |
| `enable_torque` | | 성공 여부 |
| `resume` | | stop·estop·fault 뒤 idle로 되돌린다(팔은 움직이지 않음, GUI의 [정지 해제]). 성공 여부. 통신 끊김 fault는 풀지 않고 False(다시 연결) |
| **이동** | | |
| `home` / `park` | | `MotionResult` |
| `move_joints` / `move_tool` | `JointState` 또는 `ToolPose`, 속도 %(생략 시 지금 속도) | `MotionResult` |
| `point_pose` | x, y, 높이, yaw(기본 90), 좌표계(`table`/`robot`/`tool`), TCP(`tip`/`ref`), pitch(기본 −90) | `ToolPose` |
| `preview_tool` / `preview_path` | `ToolPose`(, 속도 %, 샘플 간격) | 움직이지 않고 가능 여부·이유·관절값 / 경로 샘플 |
| `follow_path` | 점 목록(mm), 손끝 높이, 속도 mm/s, yaw | 자세 샘플 목록. 계획한 경로 시간이 비정상으로 길면(첫 구간이나 전체 길이가 1mm 이하일 때 났던 버그의 안전망, REQ-066) 움직이기 전에 `out_of_bounds`(이유 `plan_too_slow: …`)로 끝낸다. 점의 기준은 `tool_position`이다. 병렬이면 두 턱 끝 가운데(TCP, 0043-1)로 바꾼다(11-14 기획 결정, 팔 REQ, 지금 코드는 한쪽 턱 끝) |
| `pick_and_place` | `DetectedObject`, `zone`(선택), `place_at=(x, y)`(선택) | `MotionResult`. `zone`·`place_at` 둘 다 주면 ValueError. 둘 다 없으면 집기만 한다. 놓을 점이 범위 밖이면 움직이지 않고 `out_of_bounds` |
| `place` | `zone`과 `at=(x, y)` 가운데 정확히 하나 | `MotionResult` |
| `set_gripper` | 값 0~100, `as_width=True`면 폭 mm | `MotionResult` |
| `jog` | 모드, 속도 벡터 | 성공 여부(데드맨 적용) |
| `retarget` | `JointState` | `MotionHandle` |
| `stop` / `estop` | | None / 실패 이유 `str` 또는 None(None이면 정지가 먹힘) |
| `set_speed` | 0~100 % | None |
| **교시** | | |
| `record_start` | 기록 주기 Hz, 최대 시간 s | None |
| `record_stop` / `record_info` | 이름 / 이름, 속도, `segment_speeds` | 저장 결과 / 길이·속도·구간·`fit` |
| `replay` | 이름, 속도, `until_limit`, `segment_speeds`, `mode`(`original`/`fitted`) | `MotionResult` |
| `run_sequence` | 이름 | `MotionResult` |
| **설정** | | |
| `get_safety` / `set_safety` | 모드, 관절 여유, XYZ 범위(6.5 검사) | 지금 안전 설정(JSON 변환 가능) |
| `get_params` / `set_params` | / {이름: 값} | 6.9.1 |
| `save_preset` / `load_preset` / `list_presets` | 이름 | 6.9.1 |
| `reload_workspace` | 경로(기본 `config/workspace.yaml`) | 새 `WorkspaceConfig`. 작업 실행 중이거나 MuJoCo면 거부(13.5) |
| `set_sensor_offset` | 센서 오프셋 mm(기준점: 단동식 고정 턱 끝, 병렬 두 턱 끝 가운데, #358) | None |
| **계산·조회** | | |
| `tool_position` / `sensor_position` / `arm_links` | 관절값 | 손끝 xyz / 센서 xyz / 링크 점 목록(평면도용) |
| `reach_ranges` | pitch, 높이 목록, 간격 | 높이별 도달 반경 |
| `gripper_info` | | 지금 그리퍼 종류, 고를 수 있는 종류, 폭 대응표, 최대 벌림 |
| **진단·기록** | | |
| `run_diagnostic` | 이름(`powertest` 등), 진행 콜백, 정지 신호 | 결과 dict |
| `log_event` / `create_incident` | 이벤트 / 이유, 앞 시간 | None / 사고 묶음 경로 |
| `on_watchdog_stop` | 이유 | 감시 프로세스가 부른다(결정 0026) |

#### 6.9.1 파라미터와 프리셋 (`service/params.py`, #276·#285)
- `get_params()`는 {이름: {`value`, `default`, `saved`, `timing`}}를 돌려준다.
  - `value`: 지금 값
  - `default`: 코드 기본값
  - `saved`: 연결 때 설정 파일에서 읽은 값(겹친 cfg, `set_params` 전). 없으면 키가 없다. GUI의 [기본값으로]가 이 값으로 되돌린다(gui-spec 12.7)
  - `timing`: 적용 시점. `now`(바로, `NOW_KEYS`: 속도·그리퍼 속도), `reconnect`(다시 연결해야, `RECONNECT_KEYS`: `control_hz`, 서보·그리퍼 가속, `table_z_mm`, `tip_floor_mm`), 나머지는 `next_motion`
- `set_params(changes)`:
  - 모든 값을 먼저 검사하고, 하나라도 틀리면 ValueError를 내고 아무것도 바꾸지 않는다(원자적 적용).
  - 모르는 이름은 거부한다.
  - `CHOICES`에 있는 값(`trajectory_profile`, `grasp_open_policy`, `grasp_tilt_mode`)은 정해진 목록 안이어야 한다.
  - `LIMITS`에 있는 값은 양 끝을 포함한 범위 안이어야 하고 float로 받는다. 설정 파일에 정수로 적혀 있어도 2.5를 2로 자르지 않는다.
  - `INT_KEYS`(Hz, 서보 가속, 조임 토크)는 정수가 아니면 거부한다.
  - 반환값은 {이름: 적용 시점}이다.
  - `LIMITS`는 gui-spec 12.7.1 표의 허용 범위와 같다. 새 키를 설정에 더하면 `LIMITS`에도 더한다(`place_*`는 REQ-064).
- 프리셋: 이름은 글자·숫자·`_`·`-`만 쓴다. `config/presets/<이름>.yaml`에 지금 값을 저장하고, 읽을 때도 `set_params`와 같은 검사를 거친다.

GUI에서 점 찍기는 `point_pose` → `preview_tool` → `move_tool` 순서로 부른다.

### 6.10 정책 실행: 관절 목표 흘려 넣기 (결정 0066의 L4, 2026-11-16, EGM Stage 9 (나) 꼭 고칠 것 반영)
학습한 정책(ACT)은 주기마다 절대 관절 목표를 낸다. 이것을 입구 하나로 받는다. **12/4 전에는 MuJoCo 엔진에서만 쓴다**: 드라이버가 `simulated`가 아니면(실제 팔) `servo_joints`가 거부하고, `PolicyRunner`는 드라이버에 `render`가 없으면(실제 팔, 기구학 엔진) 시작하지 않는다(결정 0066의 4항).

**`MotionController.servo(goal: JointState) -> bool`, `ArmService.servo_joints(goal) -> bool`**
- 최신 목표 하나만 기억한다. 틱마다 지금 명령에서 그 목표 쪽으로 간다. 한 틱 이동 상한 = min(`joint_v_max_deg_s` × 속도 %/100 × dt, `max_step_deg`).
- 서보 중 상태는 조그와 같은 `"teleop"`이다. 궤적 동작이 도는 중이면 `False`(이유 `busy`)다. `stop()`은 서보 목표도 비운다.
- 검사는 `safety.check_goal(goal, ref=마지막 명령)`이다(관절 한계·작업 영역·바닥·장면 부품. **알려진 물체는 보지 않는다**: 보면 집으러 다가가는 목표가 모두 거부된다). 걸리면 조그와 달리 자르지 않고 목표를 버리고 제자리에 선 뒤 `False`를 돌려주고, WARN `ARM-SERVO-REJECT`(kind `servo_reject`, `{reason, count}`)를 같은 이유당 1초에 한 번 남긴다. 한 틱 보간 뒤의 `check_step`에 걸리면 조그와 같이 fault다.
- 새 목표가 0.5초 동안 없으면 지금 자리에서 선다(정책 스레드가 죽었을 때). `estop`·fault·부하·과열 감시는 그대로 듣는다.
- 그리퍼: 목표의 그리퍼 값이 직전 명령보다 닫히는 쪽이면 집기 밖 닫기와 같은 접촉 판단을 켠다(`_closes_gripper`, 6.5의 `blocked_grip_units`·예압 계산 그대로). 접촉이면 "지금 위치 + 예압"에서 멈춘다. 팔 다섯 관절의 막힘 판단은 서보에서 하지 않는다(조그와 같다. 막힘 판단은 궤적 동작에서만 돈다).

**`ArmStatus.cmd`**: `ArmStatus`에 `cmd: JointState`(지금 명령 관절값, 동작이 없으면 마지막 명령)를 더하고 `status()`가 채운다. 에피소드 기록기(`mujoco-spec.md` 13.10.4)가 `action`으로 읽는다. 기록용 훅은 따로 두지 않는다.

**`service/policy_runner.py`**(새 파일): `PolicyRunner(svc, driver, checkpoint_dir=None, fps=25, policy=None, max_s=20)`, `start()`, `stop()`.
- `policy`를 주면 그대로 쓴다(시험용. `select_action(obs) -> 6값`과 `reset()`만 있으면 된다). 없으면 lerobot 0.6.1의 ACT 정책을 `checkpoint_dir`에서 CPU로 열고 같은 폴더의 전후 처리기를 쓴다(설치본의 불러오기 함수 이름은 PR에 적는다). 체크포인트 폴더는 `runs/train/<이름>/checkpoints/<스텝>/pretrained_model`이다.
- 스레드 하나가 `fps`마다 관측(`driver.read_joints()`, `driver.render("top")`, `driver.render("wrist")`)을 모아 정책에 넣고 나온 목표를 `servo_joints`로 보낸다. 여러 스텝을 한 번에 내는 묶음은 정책의 `select_action` 큐에 맡긴다.
- 끝: `max_s`가 지나거나 `stop()`이면 끝난다. `servo_joints`가 `False`를 돌려주면(정지·E-STOP·fault·거부가 이어짐) 러너도 끝난다. 끝나고 1.0초 뒤 배치가 주어져 있으면 `judge`(13.10.4)를 불러 결과를 싣는다.
- 이벤트: `ARM-POLICY-START`(kind `policy_start`, 체크포인트·fps), `ARM-POLICY-STOP`(kind `policy_stop`, `{reason, outcome}`), 추론 시간 평균·최대. CPU 추론이 한 주기(40ms)를 넘으면 그 주기는 건너뛰고 건너뛴 수를 `ARM-POLICY-STOP`에 적는다. 25Hz를 지키는지 `measurements.md`에 남기고, 못 지키면 총괄을 거쳐 앱 환경의 torch를 바꿀지 사용자에게 묻는다.
- torch·lerobot을 못 불러오면 안내문으로 끝낸다(앱의 다른 기능은 그대로 돈다).
- 2차(자르기를 올린 것, 지금은 만들지 않는다): `pause()`(추론을 멈추고 서보 목표를 지금 명령값으로, `ARM-POLICY-PAUSE`)와 `resume()`(`policy.reset()` 뒤 새 관측으로), `ArmService.connect(fresh=True)`(다시 연결할 때 `Skills` 기억을 넘기지 않음, 세계 다시 짓기용), 게임패드 입력 어댑터(6.7의 "나중", `teleop.py`).

**시험**(팔, `tests/test_servo.py`·`tests/test_policy_runner.py`, `SimDriver`로)
- 범위 밖 목표는 거부되고 제자리에 선다. 0.5초 끊기면 선다. 궤적 동작 중이면 `busy`다. 실제 팔 드라이버(가짜)면 거부한다.
- 코드로 만든 25Hz 관절 목표 열을 `servo_joints`로 틀면 끝 자세가 목표와 1° 안이다(데이터셋을 다시 트는 시험은 MuJoCo 쪽 13.10.4에 있다).
- 닫히는 그리퍼 목표에서 접촉 판단이 켜진다.
- 가짜 정책(늘 홈 자세)으로 `PolicyRunner`를 `start()` → `stop()` 하면 이벤트 둘이 남는다. `render`가 없는 드라이버면 시작하지 않는다.

## 7. 설정 파일 (주요 값, 실제 값은 `config/*.yaml`이 기준)
| 항목 | 값 | 근거 |
|---|---|---|
| 통신 | 1Mbps, 서보 ID 1~6. 포트는 7.1절 순서로 고른다(PC마다 다르다) | `arm-hardware-setup.md`, `decisions/0027` |
| 제어 주기, PID | 50Hz, P16 I0 D32 | `decisions/0014` |
| 서보 가속 한계 | 팔 10, 그리퍼 25 | `decisions/0014` |
| 홈·파크 자세 | `[0, -90, 93, 0, 0, 3]` (파크는 센서 위치가 정해지면 바꿈) | 팔꿈치 한계 95 |
| 영점 보정 | shoulder_pan −3.0° (팔 파일 `arms/so101_follower_01.yaml`) | 육안 정렬 |
| 속도 | 기본 30%, 관절 최대 90°/s, 직교 최대 100mm/s, 조그 30°/s·80mm/s | |
| 궤적 | `trajectory_profile: quintic` (선택 `ruckig`) | measurements.md |
| 높이 | `table_z_mm` −15(팔 파일, 2026-11-07 실측), `tip_floor_mm` 5, `approach_height_mm` 40, `grasp_height_mm` 20(REQ-033), `place_height_mm` 30(옛 방식 놓기의 손 기준점 높이, 11.3.3) | `measurements.md` |
| 집기 | `grasp_open_policy: max`(6.6), `grasp_max_open_mm` 90(REQ-033), `gripper_margin_mm` 20, `grasp_tilt_mode: equal_tips`(옵션 `symmetric`), `grasp_tilt_deg` 35, `contact_lag_units` 4, `grip_min_close_mm` 5(6.6 쥠 확인), `gripper_squeeze_torque` 300 | `decisions/0010·0011·0060` |
| 교시 | `record_hz` 20, `record_max_s` 90, 구간 기준 3·40°/s, 최소 0.5초 | `decisions/0019` |
| 놓기 | `place_drop_mm` 3, `place_release_mm` 6, `place_clearance_mm` 30, `place_open_s` 1.0 | 11.3 |
| 작업대 (`workspace.yaml`) | 600×400mm(임시), 로봇 원점 (300, 50) 90°(긴 변 가운데, 임시), 배출 구역 `right: {x_mm: 480, y_mm: 150, w_mm: 100, h_mm: 80, slot_mm: 75}`(칸 격자, 11.3.5), `drop_zone_ignore_radius_mm` 75(점 구역을 초음파가 무시하는 반경, 11.3.5), 도달 반경 90~250mm | 배치 미정 |

### 7.1 설정 겹과 팔 고르기 (`decisions/0027`)
| 파일 | 내용 | git |
|---|---|---|
| `config/robot.yaml` | 모든 SO-101 공통 기본값(속도, 궤적, 집기 방식, 교시, 자세) | 올림 |
| `config/arms/<팔 이름>.yaml` | 팔별 값: `board_serial`, `calibration_id`, `joint_offsets_deg`, `joint_limits_deg`, `physical_limits_deg`, `table_z_mm`, `gripper_width_mm`, `grasp_bias_mm`, 잰 날짜 | 올림. 새 팔은 `arms/_template.yaml`을 복사 |
| `config/grippers/<종류>.yaml` | 그리퍼 종류별 값(폭 대응표, 조임 값 등). 팔 파일의 폭 표·조임 값은 단동식을 잰 것이라, 다른 종류에서는 이 파일이 이긴다 | 올림 |
| `config/local.yaml` | PC별 값: `port`, `arm`, `gripper`, 그 밖에 이 PC에서만 덮을 값 | 제외(`.gitignore`). `local.example.yaml`을 올림 |
| `config/calibration/<calibration_id>.json` | LeRobot 캘리브레이션 | 올림(지금과 같음) |
| 그 밖 | `workspace.yaml`(작업대·배출 구역), `scene.yaml`(장면 부품), `manual_objects.yaml`(수동 입력 물체), `sim_scenarios/`(MuJoCo 시나리오, 13.5), `ultrasonic.yaml`(초음파), `presets/`(파라미터 프리셋, 6.9.1), `sequences/`(교시) | 올림 |

- 읽는 순서는 `robot.yaml` → 팔 파일 → `grippers/<종류>.yaml` → `local.yaml`이고, 뒤가 앞을 덮는다(키 단위로 통째로). `local.yaml`이 없으면 지금과 똑같이 동작해야 한다.
- 그리퍼 종류 고르기: `local.yaml`의 `gripper` → 팔 파일의 `gripper` → `robot.yaml` → `single_jaw`. 모르는 종류면 ValueError다. 종류는 다시 연결(재시작)해야 바뀐다.
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
| 이동 | `status`, `home`, `park`, `movej`, `goto`, `pick x y 각도 폭 [높이] [--place 구역]`(5번째 값은 물체 높이 mm, 6.6), `place --place 구역`, `gripper [--width]` |
| 자동 작업 | `run --source manual [--step] [--dry-run]`, `run --source ultrasonic [--us-port COMx]` |
| 조종 | `keys` (관절 / 직교 조그 / 좌표 입력, `--jog-speed`·`--cart-speed`·`--trace`) |
| 교시 | `record <이름> [--record-hz] [--record-max-s]`, `info <이름>`, `replay <이름> [--seg-speed 1=0.5] [--until-limit] [--mode original\|fitted]`, `seq <이름>` |
| 진단 | `powertest`, `shaketest [--speeds]`, `servoinfo`, `linkwatch`, `--hz`, `--trace` |
| 점검·캘리브레이션 | `doctor [--json] [--no-probe]`, `find-port`, `calibrate --auto --measure-only` / `--save` / `--verify-zero`(11.1), `incident [--last N]` |
| 앱 | `gui [--share]`(gui-spec) |
| 공통 | `--sim`, `--sim-engine kinematic\|mujoco`, `--sim-scenario`(13.1), `--arm`, `--port`, `--config`, `--speed` |
| 집기 시험(이번만) | `--margin`, `--tilt`, `--max-open`, `--bias` |
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
T 행의 결과는 2026-11-02, AC·M 행은 2026-11-13 기준이다. ✅ 완료, △ 구현은 끝났고 실측이 남음.

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

| # | 작업 | 완료 기준 | 관련 기능 | 결과 |
|---|---|---|---|---|
| AC1 | 자동 캘리브레이션 엔진 가져오기: `arm/autocal/engine.py`, `plan.py` (LeRobot PR #4826, `decisions/0028`) | 가져온 가짜 서보 테스트가 LeRobot 0.6.1에서 통과 | A2 | ✅ REQ-016 |
| AC2 | `so101 calibrate --auto --measure-only` | EEPROM 불변, 기존 파일과 관절별 차이표 출력, 전압·온도 중단 | A2 | ✅ REQ-016 |
| AC3 | `--save` | `.bak.json` 생성, EEPROM·LeRobot 캐시·`config/calibration`에 저장, 저장 뒤 `doctor` 캘리브레이션 대조 정상 | A2 | ✅ REQ-016 |
| AC4 | 실물 시험 (`decisions/0028` 절차) | 기존 값과 ±3° 이내, 3회 반복 차이 1° 이하, `measurements.md` 기록 | A2 | ✅ 11-10(3회 0.26° 안, 중앙값 저장) |
| AC5 | 반복 합격과 0° 자세 확인 (`decisions/0029`) | 새 팔·기존 값과 다른 팔도 3회 반복으로 저장 가능, 저장 뒤 0° 자세 확인 전에는 움직이는 명령이 잠김 | A2 | ✅ REQ-017, 0° 확인 기록 11-10 |

MuJoCo 물리 시뮬레이션(결정 0053, 13절). M1부터 차례로 한다. GUI는 M4에서만 바뀐다.

| # | 작업 | 완료 기준 | 근거 | 결과 |
|---|---|---|---|---|
| M1 | 모델 가져오기(13.4), `arm/mujoco_model.py`(`load_model`, `JOINT_MAP`, 좌표 변환), 설치 확인(13.8·13.9) | 13.7 (d) | 0053, 13.1 | ✅ #260(관절 range 바꾸기는 M2에서) |
| M2 | `MuJoCoDriver`(13.2·13.3), 관절·그리퍼 range와 ctrlrange 넓히기(`mujoco_model.py`, 13.4), `make_sim_driver`, CLI `--sim-engine`과 명령 처리(13.1), `--dry-run` 거부, diagnostics 검사, `reload_workspace` 거부, `sim_state()`는 빈 값(13.6), tooling-setup 설치 줄과 버전 고정(13.8) | 13.7 (c)(e) | 0053, 13.1 | ✅ #275(11-13) |
| M3 | 장면·물체·시나리오(13.5), `sim_state()` 채우기(13.6) | 13.7 (a)(b) | 0053, 13.1 | ✅ #290(11-13) |
| M4 | 팔: `--sim-viewer`(13.1 뷰어), `sim_state()` `size_mm`, `failed_reason`, 물체 파일·겹침 검사(13.5), CLI `gui` 거부 해제. GUI: `gui-spec.md` 12.8(팔 세션이 REQ로 요청). 순서: 팔 M4 PR이 먼저 합쳐진 뒤 GUI M4를 시작한다. 총괄이 팔 M4 REQ를 `session-requests.md`에 올린다 | 13.7 (f) | 0053, 13.1, gui-spec 12.8 || ✅ 팔 #303, GUI #328(11-14) |
| M5~M9 | 시험장으로 키우기(13.10, 결정 0062): M7a 부품 사전 평가(✅ #335), M4b 로보나인 손 모델(0063), M5 집기 시험장·M6 초음파를 MuJoCo에(우선 1), M5b 빠른 모드·M5c 서보 맞추기(2), M8 GUI 시험장(3), M9 강화학습(12/4 뒤) | 13.10 | 0053, 0037 | |

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
- **그리퍼 인터페이스** (`arm/gripper.py`의 `Gripper`): 폭↔서보값, 턱 끝 위치, 손 기준점 보정, 기울기, 턱이 위로 좁아지는 비율, 닫는 경로를 그리퍼마다 구현한다. `SingleJawGripper`(지금 것, 기본)와 `ParallelGripper`(Robonine, 두 턱 중앙이 기준점, 기울기 0). 고르기는 7.1 순서(`local.yaml`의 `gripper`가 먼저, 그다음 팔 파일), 값은 `config/grippers/<종류>.yaml`. 접촉 감지·조임 토크는 그리퍼마다 따로 실측한다.
- **물체 높이에 맞춘 집기**: `DetectedObject.height_mm`를 알면 턱 끝 높이 t를 물체마다 정한다. 통과 조건은 단동식 W ≤ G − (H − t)·s(G) − 4, 병렬 W ≤ G − 4(W 집는 폭, G 최대 벌림, H 물체 높이, s 좁아지는 비율). 모르면 `grasp_height_mm`. 지원 물체는 이 조건을 만족하는 크기이고, 시연 물체(높이 약 40mm, 폭 30~58mm)는 `grasp_max_open_mm` 90에서 통과한다(REQ-033 실측으로 확인).
- **집기 범위와 움직임 범위**: `workspace.yaml`의 집기 범위(수직 하강, 작업 실행·물체 판정)와 움직임 범위(손끝 방향 자유, 직접 조작·안전 상자 표시)를 나눈다. 손끝 pitch를 받는다.
- **장면 충돌 검사**: 장면 부품(`system-architecture.md` 5.1)과 턱 메쉬를 coal로 검사한다. 턱 메쉬 가장 낮은 점이 작업 면(`scene_floor`) 아래로 가지 않게 한다.
- **교시 재생 변환본**: 녹화 원본에서 한 주기 이동량(5°/20ms)·가속 한도를 넘는 구간만 늦춘 재생 가능본을 만들고, 재생 때 원본/변환본을 고른다. 원본도 한 주기 이동량 검사는 끄지 않는다.
- **fault 원인 문장**: fault에 관절·값·한도를 담고 사람이 읽는 원인과 해결법을 붙인다(`arm/fault_text.py`의 `describe`. `service/targets.py`의 `reason_text`는 목표 거부 사유 문장이다). 분류는 우리 코드의 예외(`internal_error`)와 서보 오류 비트(입력 전압·과열·과부하 등, 하드웨어 쪽)를 나눈다.
- **설정 바로 적용**: 작업대·좌표 설정을 저장하면 `ArmService`·`Skills`가 다시 읽는다. 작업 실행 중에는 저장을 막는다. 그리퍼 속도 `gripper_speed_pct`는 바로 적용, `gripper_acceleration`(접촉 감지 기준)은 그대로.
- **E-STOP·감시 상태**: `ArmService.estop()`은 위치 고정을 서보에 쓰지 못했을 때 그 이유(`str`), 먹혔으면 None을 돌려준다. 감시 HTTP 응답의 `ok`는 이 값으로 만든다. 감시 프로세스는 스레드 생존과 `last_tick_age_s`로 살아 있음을 판단한다(스레드가 죽었는데 alive로 보이던 문제, REQ-043).
- **관절 한계**(REQ-054, 2026-11-11): `physical_limits_deg`는 2026-11-10 AC4 측정에서 제안된 값, `joint_limits_deg`는 그보다 1° 안쪽이다. 다만 elbow 위쪽은 95°를 유지한다.

## 11.3 놓기 동작 모양 (2026-11-07 첫판 → 2026-11-10 개정: 대칭 V자, 사용자 요청 U2, EGM Stage 9 반영)
**목표**: 놓을 때도 집을 때처럼 두 턱이 수직에서 같은 각도로 벌어지는 V자가 보여야 한다.

### 11.3.1 첫판(#204)과 원인
- 11-10 실물 2회의 결과(`measurements.md` "놓기 재시험 2026-11-10"):
  - 기록 기준(고정 턱 이동 20mm 이상)은 통과했다. 고정 턱 이동 30.8·32.1mm, 두 턱 중간점 이동 4.5·5.3mm, 1.0초.
  - 영상 기준은 불합격이다. 고정 턱은 수직인 채 옆으로 비켜나고, 가동 턱만 거의 수평까지 돌아 올라간다.
- 첫판은 손을 수직(pitch −90°)에 둔 채 손을 옆으로만 옮겼다. 그래서 두 턱 끝 위치는 어느 정도 맞았지만 턱 **각도**는 대칭이 아니었다.
- 당시 집기는 `grasp_tilt_mode: symmetric`으로 손을 θ/2만큼 기울였다(θ = 가동 턱이 닫힌 데서 벌어진 각도 = `VALUE_TO_Q[1]`·v, 손목 한계 `grasp_tilt_deg` 35°까지). 놓을 때는 이 기울기를 쓰지 않았다.
- 2026-11-13부터 집기 기본값은 `equal_tips`다(결정 0060, 6.6). `symmetric`은 옵션이다. 놓기는 집기 기울기를 그대로 이어받는다(판단 J5 = A, 11-13 사용자 결정). 그래서 아래 11.3.2~11.3.4의 대칭 V자 정의와 기준은 `symmetric`일 때만 쓰고, 기본값에서는 놓을 때 보기의 대칭을 맞추지 않는다(실물 equal_tips 놓기·떠나기 10/10, 손 기울기 5.0~5.3°→18.5°, `sym_err_max` 33.1°, runs/20261113-231006).
- 같은 이유로, 기울인 채 집은 물체는 첫판 놓기에서 수직 손으로 내려가 기운 채 놓였다.

### 11.3.2 용어
- **턱 각도**: 고정 턱 각도 α_f = τ(손 기울기 = −90° − 손 pitch), 가동 턱 각도 α_m = θ − τ. `tilt()`가 쓰는 정의와 같다. 둘 다 수직 아래에서 각 턱의 바깥쪽으로 잰다. **대칭 오차** = |α_f − α_m| = |2τ − θ|. 관절값(손 pitch는 FK, θ는 그리퍼 값)으로 계산한다.
- **집기 기억** `Skills.grip`(dict): 집기가 접촉으로 끝나면 다음 값을 기억한다.
  - `tilted`: 집기의 `tilt_ok`(아래 글에서 `grip_tilted`로 부른다)
  - `tip_h`: 집을 때 낮은 턱 끝의 작업 면 위 높이 t_g(mm, 모델 z는 `table_z_mm` + t_g)
  - `obj_h`: 물체 높이 `obj.height_mm`(0 = 모름), `obj_w`: 물체 폭, `held_gap`: 쥔 틈
  - **지우는 때**: 놓기에서 **벌리기 시작할 때**, 집기 실패나 `no_object`, 새 집기를 시작할 때. 벌리기 전에 끝난 놓기(정지, `ik_fail`, `low_hover`, `drop_zone_full`)는 기억을 남긴다. 그래서 다음 칸 시도(11.3.5)와 다시 놓기가 같은 기억으로 간다(`tests/test_req053.py`가 고정).
  - `Skills`는 연결·재연결 때 다시 만든다. 이때 집기 기억, `placed`, `known_objects`를 새 `Skills`에 넘긴다(`placed_height`는 넘기지 않는다). `reload_workspace`는 `ws`만 바꾸고 `Skills`를 다시 만들지 않는다.
- **시작 기울기** τ_s: `grip_tilted`이면 `tilt(지금 그리퍼 값)`, 아니면 0. 손에 쥔 그리퍼 값으로 계산하므로 경로 첫 단계에서 튀지 않는다(예압으로 집은 순간과 약 1° 다를 수 있으나 받아들인다).
- **대칭 상한 틈**: v_sym = 2·`grasp_tilt_deg`/`VALUE_TO_Q[1]`, 틈 = `_gap_mm(v_sym)`(약 90mm). 이보다 벌리면 손목 한계 때문에 대칭이 깨진다.

### 11.3.3 순서
1. **놓는 손 방향(yaw)**:
   - 단동식이면 `_place_yaw`가 놓는 점에서 팔이 뻗은 방향을 `_jaw_yaw`(손목이 덜 도는 쪽, 로봇 기준 ±90° 안)로 고른 값으로 한다(REQ-053 구현됨). 그 방향에서 기울일 수 없으면(`tilt_allowed` 거짓) 0이다. 예: 옛 점 (500, 150)은 26.6°로 기울여 놓는다(`tests/test_req053.py`).
   - 반대쪽(180°)은 시도하지 않는다.
   - 손목 회전은 접근 위치로 가는 관절 이동 중에 하고, 쥔 물체도 함께 돈다.
   - 병렬 그리퍼는 yaw를 바꾸지 않는다(기울기 0, 지금과 같음).
2. **내려가는 자세**:
   - release = `_hand_pose(x, y, yaw, 지금 그리퍼 값, grip_tilted, table_z_mm + t_g + place_drop_mm)`. 낮은 턱 끝이 작업 면 위 t_g + `place_drop_mm`(새 설정, 3mm)에 온다.
   - 쥔 동안 미끄러지지 않았다면 물체 바닥은 작업 면 위 약 3mm에서 놓인다. 높이 실측 오차가 ±3mm라 0~6mm로 본다.
   - hover는 release를 위로 올린 같은 기울기 자세다. 내려가는 동안 물체가 돌지 않게, 집기의 hover처럼 절반만 기울이지 않는다.
   - **hover 높이**(MoveIt approach의 desired/min 구조): 원하는 값 `approach_height_mm`(40)부터 5mm씩 낮춰 최소 10mm까지, IK가 닿는 첫 값을 쓴다. 지금 구현이 이렇다.
     - 40mm 아래 hover를 쓰는 것은 정식 규칙이다(2026-11-11 확정). 다만 hover로 가는 수평 이동 중에 **쥔 물체 바닥이 놓는 점 주변(반경 100mm)의 알려진 물체(`placed`, `known_objects`) 가운데 가장 높은 것보다 10mm 이상 위**여야 한다(`_hover_need`).
     - **이웃 높이**(판단 J2): `placed`는 놓을 때 기억한 높이(`placed_height`)를 쓰고, 모르면 40mm(`NEIGHBOUR_H_MM`)다. `known_objects`는 높이를 담지 않아 늘 40mm로 가정한다. 시연 물체(높이 약 40mm 이하, 11.2)는 이 가정 안이다. 40mm보다 높은 물체를 `known_objects`로 쓰면 과소평가되므로 쓰지 않는다.
     - 두 턱 끝과 알려진 물체의 원 검사(`_hits_objects`)도 같은 가정이다. 낮은 턱 끝이 40 + 10 = 50mm 이상이면 그 자세는 검사하지 않는다.
     - 상승 최소(아래 5번)와 같은 기준이다. 둘 다 "옆 물체 위로 10mm"를 지킨다.
   - **못 내려갈 때의 순서**(판단 J1, 코드와 시험이 굳힌 동작):
     1. 새 자세의 하강 계획(`_descent`: hover 하한을 지키고, 지나는 자세가 알려진 물체와 부딪히지 않는 hover 높이)이 없으면 **옛 방식**(yaw 0, 기울기 0, 손 기준점 z = `table_z_mm` + `place_height_mm`)으로 다시 계산한다. `ARM-PLACE-LEGACY`(이유 `ik_failed_new_pose`)를 남긴다.
     2. 옛 방식도 계획이 없을 때:
        - hover 하한이 10mm를 넘고 알려진 물체에 막힌 것이 아니면 `stopped`(이유 `low_hover`)로 멈추고, ERROR `ARM-PLACE-LOW-HOVER`를 남긴다.
        - 알려진 물체에 막혔으면 `ik_fail`(이유 `known_object: …`)이다.
        - 그 밖에는 `ik_fail`이다.
     - 옛 방식이 hover 하한을 지키는 낮은 자세를 찾을 수 있어서, `low_hover`보다 옛 방식을 먼저 해 본다.
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

   - ②를 위해 `Gripper.closing_path`가 기울기 배율 `tilt_scale`을 받는다. 그리퍼 값 v를 받아 배율을 돌려주는 함수(callable)이고, B 구간은 11.3.4의 k(v)를 넘긴다. 기본은 1이라 지금과 같다. A 구간은 `tilt_ok=False`로 부른다.
4. **경로 검사**(종류마다, 여유마다):
   - 경로의 **모든 단계**에서 `safety.check_goal(pts[i], pts[i-1])`(장면 부품 `scene:*`, `scene_floor`, 바닥, 작업 영역, 관절 한계)를 본다.
   - 두 턱 끝이 알려진 물체(`placed`, `known_objects`)의 원에 들어가는지도 단계마다 본다(지금의 시작-끝 선분 검사를 단계마다로).
   - ②는 하나를 더 본다. 쥔 물체 상자(폭 = 물체 폭을 알면 그 값(`obj.width_mm`), 모를 때만 쥔 틈, 높이 = `obj.height_mm`, 바닥 = 작업 면 위 `place_drop_mm`)의 옆면과 두 턱 선분이 단계마다 2mm 이상 떨어져야 한다. 턱 선분은 고정 턱이 손 기준점→고정 턱 끝, 가동 턱이 가동 턱 회전축→가동 턱 끝이고, 턱 축이 들어간 수직 평면에서 2D로 본다. 걸리면 이유 `tilt_hits_object`로 ③으로 간다. 물체 높이를 모르면 ②를 건너뛴다(이유 `height_unknown`).
   - 실제 결과(판단 J2): 높이 40mm 물체는 폭을 알아도(예: 45mm) 이 검사에 걸려 ③이 된다. 기울어지는 턱이 키 큰 물체의 윗모서리를 지나기 때문이다. ②는 낮은 물체(시험은 높이 25mm)에서 쓰인다. 40mm 물체는 기울여 집었으면 ①로 대칭 V자가 된다.
5. **다 벌린 뒤**: 원래 하강 경로를 거꾸로 타지 않는다(물체를 민다). 그리퍼는 연 채로 둔다. 놓은 물체는 `placed`에 더한다(높이를 알면 `placed_height`에도).
   - **상승**: 그 틈과 기울기를 유지한 채 지금 자리에서 곧게 올라간다(원하는 값 `approach_height_mm`부터 5mm씩 낮춘 높이 가운데 닿는 첫 값).
   - **상승 높이의 최소**: 최소 **물체 높이 + 10mm**(물체 높이를 모르면 t_g + 30mm)다.
   - 곧게는 그만큼 못 올라가면 `_untilt_goal`로 **기울기를 풀며** 마저 올라간다. 지나는 자세마다 쥐었던 물체 상자와 2mm, `known_objects`·`placed` 원, `safety.check_goal`을 검사한다.
   - 그래도 계획이 없으면 갈 수 있는 만큼 올라간 뒤 그 자리에서 `stopped`(이유 `low_lift`)로 멈춘다. ERROR `ARM-PLACE-LOW-LIFT`(올라간 높이, 필요한 높이, 거절한 검사 `rejected_by`)를 남기고, 다음 수평 이동은 하지 않는다. 사람이 정한다.
6. **기록**:
   - `ARM-OPENING-PATH`에 키를 더한다.
     - `route`
     - `tilt_start_deg`, `tilt_end_deg`(계획)
     - `jaw_angle_fixed_deg`, `jaw_angle_moving_deg`([시작, 끝], 실제 관절로 계산)
     - `sym_err_max_deg`(계획 경로의 대칭 오차 최대. ②는 B 구간만), `actual_sym_err_end_deg`
     - `yaw_deg`, `skipped`(건너뛴 경로와 이유 목록)
   - 기존 키(`actual_fixed_tip_move_mm` 등)는 그대로 둔다.
   - 옛 방식으로 갔으면 `ARM-PLACE-LEGACY`(이유)를 남긴다. 기울여 쥔 물체면 WARN, 아니면 INFO다.
   - 그 밖의 이벤트: `ARM-PLACE-LOW-HOVER`(ERROR, 2번), `ARM-PLACE-LOW-LIFT`(ERROR, 5번), `ARM-DROP-ZONE-FULL`(ERROR, 11.3.5), `ARM-PLACE-SAME-SPOT`(WARN, 11.3.5).
   - `skipped`의 이유: `height_unknown`, `tilt_hits_object`, `legacy_pose`, `not_tilted_grip`, `tilted_grip`, `tilt_not_allowed`, `gap_too_wide`, `clearance`, `no_lift_after_opening`.

**파일**(REQ-053·058 구현됨): `arm/skills.py`(`place`, `place_at`, `_descent`, `_scan_opening`·`_route_skip`·`_try_route`·`_build_route`(경로 고르기), `_leave`·`_untilt_goal`(상승), `_emit_opening`, 집기 기억), `arm/gripper.py`(`closing_path`의 `tilt_scale`), `config/robot.yaml`(`place_drop_mm: 3`, `place_release_mm: 6`, `place_clearance_mm: 30`, `place_open_s: 1.0`). `place_*` 키의 범위 검사(`service/params.py`의 `LIMITS`)는 REQ-064다.

### 11.3.4 완료 기준
- 아래 대칭 기준(①·②의 대칭 오차·각도)은 `grasp_tilt_mode: symmetric`일 때만 판정한다. 기본값 `equal_tips`(결정 0060, J5 = A)에서는 대칭을 판정하지 않고 `ARM-OPENING-PATH`의 `sym_err_*` 값만 기록한다. 이때 놓기 성공은 물체 바닥 높이, 이웃 물체(부딪힘 없음), `skipped` 이유, 떠나기 상승(`low_lift` 없이 최소 높이 도달)으로 본다. 이 기준들은 두 방식 모두에 적용한다.
- 시뮬레이션: 폭 30·45·58mm(높이 40mm), 놓는 곳 배출 구역과 (300, 200)에서 다음을 모두 만족한다.
  - 배출 구역은 반경 90~250mm 안에서 옮길 수 있다(`workspace.yaml`). 대칭 V자(①·②)는 sim 기준 반경 약 200mm까지 된다(넓힌 손목 한계면 폭 30mm는 225mm까지). 그보다 먼 곳의 넓은 물체가 ③`fixed_tilt`로 가는 것은 팔 구조의 한계로 받아들인다. 그래서 아래 기준은 반경 200mm 안의 놓는 곳에 적용한다.
  - ②의 기울기 배율은 k(v) = 1 − θ_A(1 − s)/θ(v)로 구현한다(θ_A는 A 구간 끝의 θ, s는 B 구간 진행 0→1). 이렇게 하면 "B 구간에서 대칭 오차가 줄어들기만 한다"는 기준이 지켜진다. 11.3.3의 "그리퍼 값에 비례해 0→1"을 이 식으로 바꾼다.
  - `tilted`: 벌리는 동안 대칭 오차 3° 이하, 끝에서 α_f·α_m 각각 15° 이상, 두 턱 중간점 수평 이동 3mm 이하
  - `released_then_tilted`: A 구간은 대칭을 재지 않는다. B 구간에서 대칭 오차가 줄어들기만 하고, 끝에서 3° 이하·각 15° 이상, 중간점 이동 3mm 이하
  - 물체 바닥 높이(= release의 낮은 턱 끝 높이 − 작업 면 − t_g)가 0~5mm
  - 배출 구역 점에서 놓는 턱 축 방향으로 ±70mm에 폭 58mm 물체 2개(알려진 물체)가 있을 때 3번째 놓기가 부딪힘 없이 끝난다. 여유가 줄어든 경우 `clearance_mm`에 남는다.
  - 높이를 모르는 물체, 대칭 상한을 넘는 큰 물체(폭 75mm)는 `fixed_tilt`로 가고 `skipped`에 이유가 남는다(이 항목을 확인하는 시험은 아직 없다).
  - ②`released_then_tilted`의 기준은 높이 25mm 물체로 잰다. 높이 40mm 물체는 11.3.3 4번대로 ③이 된다(2026-11-13 정정).
  - 시험은 옛 점 (500, 150)을 쓴다. 기본 사각형 구역의 칸에서 대칭 V자를 확인하는 시험은 아직 없다(팔 시험 보강 대상).
- 실물(runbook 2-2):
  - 놓기 5/5 성공
  - 놓은 물체가 5mm 넘게 밀리지 않고 쓰러지지 않음. 배출 구역에 mm 눈금 종이를 붙이고 위에서 고정한 휴대폰으로 놓기 전후를 찍어 잰다.
  - 사용자가 영상으로 "양쪽으로 대칭으로 벌어진다"를 확인

### 11.3.5 배출 구역 칸 (2026-11-11, #244 뒤 새 REQ)
- **문제**: 배출 구역이 점 하나라 두 번째 물체가 먼저 놓인 물체 자리로 내려간다. #244는 `ARM-PLACE-SAME-SPOT`(WARN)만 남기고 겹쳐 놓는다.
- **규칙**:
  - `workspace.yaml`의 배출 구역을 사각형으로 쓸 수 있게 한다: `right: {x_mm, y_mm, w_mm, h_mm, slot_mm}`(가운데·크기·칸 간격, 작업대 좌표).
  - 칸 가운데는 사각형 안에 `slot_mm` 간격 격자로 놓는다. 그 가운데 집기 범위(`reach_mm`) 밖의 칸은 뺀다.
  - 놓을 때는 비어 있는 칸(칸 가운데에서 `slot_mm`/2 안에 `placed`가 없는 칸) 가운데 **로봇 원점에 가까운 칸부터** 고른다. 가까울수록 대칭 V자(11.3.4, 약 200mm까지)가 잘 되기 때문이다.
  - **다음 칸으로 넘어가는 때**(판단 J1, 코드와 시험이 굳힌 동작): 그 칸의 하강 계획이 `ik_fail`이거나 `stopped(low_hover)`일 때만이다. 벌리기를 시작한 뒤에는 칸을 바꾸지 않는다. 그래서 열기 경로(11.3.3 3·4번)가 그 칸에서 ①~③ 모두 안 되면 다른 칸으로 가지 않고 ④ 제자리 열기를 한다.
  - 빈 칸이 없으면 놓지 않고 `stopped`(이유 `drop_zone_full`)로 멈춘다. 손에 쥔 채이므로 사람이 정한다.
  - `x_mm`·`y_mm`만 있는 옛 형식은 칸 하나로 보고 지금처럼 겹쳐 놓으며 경고한다(#244와 같음). 경고(`ARM-PLACE-SAME-SPOT`) 때 같은 자리의 옛 `placed` 항목은 지우고 새 것으로 바꾼다.
- **기본값**(시험값): `right: {x_mm: 480, y_mm: 150, w_mm: 100, h_mm: 80, slot_mm: 75}`. 폭 58mm 물체 + 여유 15mm를 생각한 값이다. 작업대 조립 뒤(runbook 2-A A-3) 실측 위치로 바꾼다.
- **초음파**: 배출 구역을 `ignore_zones`로 넘길 때(REQ-045)는 사각형을 덮는 원(반 대각선 + `safety_margin_mm`)으로 넘긴다. 점 구역은 반경 `drop_zone_ignore_radius_mm`다.
- **GUI**: 평면도가 사각형과 칸을 그린다(됨, `gui/sensors/manual_view.py`). 놓인 칸 표시만 남았다(GUI REQ, 나중).

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
- shoulder_pan 가동 범위가 PR #4826의 SO-101 값(194°)보다 큰 이유(받침 스토퍼 차이로 본다). AC4(11-10) 실측은 242.6°다. 자동 캘리브레이션 허용 범위 230~245°(`autocal/plan.py`)를 확정할지는 남아 있다.
- 보드(CH343) USB 시리얼 번호: 준다(`board_serial: "5B8E114816"`, 팔 파일). 해결됨.

## 13. MuJoCo 물리 시뮬레이션과 시험장 → `mujoco-spec.md`
2026-11-14 총정리로 이 절 전체(13.1~13.10)를 `docs/planning/mujoco-spec.md`로 옮겼다. 절 번호는 그대로라 "mujoco-spec 13.x"로 적힌 참조는 `mujoco-spec.md` 13.x로 읽는다.
