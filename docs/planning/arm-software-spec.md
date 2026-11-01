# 로봇팔 소프트웨어 설계서

상태: 초안 · 갱신 2026-11-01

로컬 개발 세션이 이 문서만 보고 팔 모듈을 구현할 수 있도록, 구조·인터페이스·출력값·작업 순서를 정한다. 범위는 개발 0~2단계(구동 점검, 동작 라이브러리, 작업 공간 확정)다. 인식 모듈과 GUI는 팔 모듈과 맞닿는 인터페이스만 정한다.

근거: `decisions/0003`(LeRobot 제어 스택), `0004`(인식–팔 분리), `0005`(개발 순서), `0006`(장치 구성), `features.md`.

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
```
src/so101_studio/
  core/        types.py, frames.py, config.py, logging.py
  arm/         driver.py, sim_driver.py, kinematics.py, trajectory.py,
               motion.py, safety.py, skills.py, teleop.py, teach.py
  service/     arm_service.py
  cli/         __main__.py
config/        robot.yaml, workspace.yaml, sequences/*.json
assets/        so101_new_calib.urdf (+ meshes)
runs/          실행 기록 (git 제외)
tests/         단위 테스트 (하드웨어 불필요)
```

## 4. 좌표계와 단위
| 항목 | 규칙 |
|---|---|
| 작업대 좌표계 | 원점 = 작업대 왼쪽 아래 모서리, x = 긴 변 방향, y = 짧은 변 방향, z = 위. 인식 모듈과 GUI는 이 좌표만 쓴다 |
| 로봇 좌표계 | 원점 = 어깨 회전축이 작업대와 만나는 점. 작업대 좌표계와의 관계(x, y, 회전각)는 `workspace.yaml`에 둔다 |
| 외부 단위 | 길이 mm, 각도 도(°). 서비스·인식·GUI 사이의 모든 값 |
| 내부 단위 | 기구학·궤적 계산은 m, rad. 변환은 `frames.py`에서만 |
| 물체 각도 | 작업대 x축 기준 반시계 방향, 0~180° (0/45/90/135는 인식 모듈의 분류값) |
| 그리퍼 | 0(닫힘)~100(완전 열림), LeRobot 정규화 값. 폭(mm)과의 대응표는 T7에서 측정 |

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
    yaw_deg: float         # 위에서 내려다본 그리퍼 회전각
    pitch_deg: float = -90 # -90 = 수직 하강

@dataclass
class DetectedObject:      # 인식 모듈 출력 (decisions/0004)
    x_mm: float; y_mm: float; angle_deg: float | None  # None = 판단 불가
    width_mm: float; length_mm: float; confidence: float  # 0~1
    source: str            # "ultrasonic" | "camera" | "manual"

@dataclass
class ArmStatus:           # 상태 스트림, 10Hz
    t: float; joints: JointState; tool: ToolPose
    temperature_c: dict[str, float]; load_pct: dict[str, float]; voltage_v: float
    torque_enabled: bool; state: str   # "idle" | "moving" | "teleop" | "teach" | "stopped" | "fault"
    fault: str | None

@dataclass
class MotionResult:
    status: str            # "ok" | "ik_fail" | "out_of_bounds" | "stopped" | "preempted" | "fault" | "timeout"
    duration_s: float; final_tool: ToolPose; position_error_mm: float | None
```

## 6. 모듈 명세

### 6.1 장치: `ArmDriver` / `SimDriver`
| 메서드 | 동작 |
|---|---|
| `connect(port, calibration_id)` / `disconnect()` | 연결. 연결 시 PID와 가속도 설정 적용 |
| `read_joints() -> JointState` | 현재 위치 (sync_read) |
| `write_joints(JointState)` | 목표 위치 (sync_write, 응답 대기 없음) |
| `read_health() -> dict` | 온도, 부하, 전압. 2Hz로만 호출 |
| `set_torque(on: bool)` | 토크 켜기/끄기 |

### 6.2 기구학: `Kinematics`
| 메서드 | 동작 |
|---|---|
| `fk(JointState) -> ToolPose` | 순기구학 |
| `ik(ToolPose, seed: JointState) -> JointState | None` | 역기구학. 해가 없거나 관절 한계를 넘으면 None |

- 기본 구현은 LeRobot `RobotKinematics`(placo)다. Windows에서 설치되지 않으면 같은 인터페이스로 다른 구현을 끼운다(`decisions/0003`).
- 5축 팔이라 "수직 하강 + 회전각" 자세에서 placo의 수치 해가 정확한지 T6에서 측정한다. 오차가 크면 수직 하강 전용 해석적 IK를 추가할지 정한다(미정 사항).

### 6.3 궤적: `trajectory.py`
| 함수 | 동작 |
|---|---|
| `move_j(q_start, q_goal, v_max, a_max) -> Trajectory` | 관절 공간 5차 다항식. 가장 많이 움직이는 관절 기준으로 시간 결정 |
| `move_l(pose_start, pose_goal, v_mm_s) -> Trajectory` | 직선 경로를 5mm 간격으로 나눠 각 점 IK. 한 점이라도 실패하면 전체 거부 |
| `move_p(waypoints, v) -> Trajectory` | 경유점 사이를 블렌딩해 멈추지 않고 통과 (시퀀스·춤용) |

`Trajectory`는 `sample(t) -> JointState`와 `duration_s`를 가진다. 참고: Robotics Toolbox for Python의 `jtraj`, `ctraj`, `mstraj`.

### 6.4 동작 실행: `MotionController`
- 별도 스레드에서 50Hz로 돈다. 매 주기마다 궤적을 샘플링하고, 안전 검사를 거쳐 `write_joints`를 호출한다.
- 장치는 이 스레드만 만진다. 다른 계층은 명령 큐로만 요청한다.

| 메서드 | 동작 |
|---|---|
| `execute(traj) -> MotionHandle` | 실행 시작. `handle.wait() -> MotionResult` |
| `stop()` | 0.3초 안에 감속 정지 후 현재 위치 유지 |
| `estop()` | 즉시 현재 위치 고정, 큐 비움, 상태 `stopped`. 토크는 끄지 않는다(끄면 팔이 떨어짐) |
| `retarget(goal)` | 진행 중인 동작을 현재 위치·속도에서 새 목표로 매끄럽게 교체. 결과는 `preempted` |

### 6.5 안전: `safety.py`
매 주기, 명령을 보내기 전에 검사한다. 하나라도 어기면 그 명령은 보내지 않고 `stop()`한다.
- 관절 한계: URDF 한계에서 5° 안쪽
- 작업 영역: 그리퍼 끝이 작업대 경계 안(여유 10mm), 높이는 작업대 위 5mm 이상
- 한 주기 이동량: 관절당 최대 5° (LeRobot `max_relative_target`과 같은 개념)
- 온도 60°C 이상, 부하 90% 이상 3초 지속, 통신 끊김 0.5초 이상 → `fault`
- 원격 조종 중 입력이 0.3초 동안 들어오지 않으면 정지 (데드맨)

### 6.6 기능: `skills.py`
| 함수 | 순서 |
|---|---|
| `home()` | 정해진 기본 자세로 MoveJ |
| `park()` | 센서 시야 밖 대기 자세로 MoveJ. 스캔 전에 반드시 호출 |
| `pick(obj: DetectedObject)` | 접근 위치(물체 위 80mm)로 MoveJ → 그리퍼 회전·열기(물체 폭 + 20mm) → 하강 MoveL → 닫기 → 상승 MoveL |
| `place(zone: str)` | 배출 구역 위로 MoveJ → 하강 → 열기 → 상승 |
| `run_sequence(name)` | `config/sequences/<name>.json`의 경유점을 MoveP로 재생 (시연용 동작, 춤) |

- 그리퍼 회전각 = 물체 각도 + 90° (짧은 변을 잡는다). 물체 각도가 None이면 0°.
- 물체 높이는 초음파로 알 수 없으므로, 집는 높이는 `robot.yaml`의 기본값(미정)을 쓴다.

### 6.7 원격 조종: `teleop.py`
- `jog(mode, velocity)`: `mode`는 `"joint"`(관절별 °/s) 또는 `"cartesian"`(x, y, z mm/s, yaw °/s). 입력 장치와 무관하게 이 명령 하나로 받는다.
- 입력 어댑터: 키보드(지금), GUI 가상 조이스틱과 게임패드(나중). 모두 `jog`를 부른다.

### 6.8 동작 녹화·재생: `teach.py`
- `record_start()`: 토크를 끄고, 사람이 손으로 팔을 움직이는 동안 관절값을 20Hz로 기록한다. 리더 암 없이 동작을 가르치는 방법이다.
- `record_stop(name)`: 기록을 `config/sequences/<name>.json`으로 저장하고 토크를 켠다.
- `replay(name, speed=1.0)`: 기록을 다듬어(이동평균) 재생한다. 시작 전 첫 자세로 MoveJ.
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
| `pick_and_place` | `DetectedObject`, 배출 구역 | `MotionResult` |
| `jog` | 모드, 속도 벡터 | 없음 (데드맨 적용) |
| `record_start` / `record_stop` / `replay` | 이름 | `MotionResult` |
| `run_sequence` | 이름 | `MotionResult` |
| `retarget` | `DetectedObject` | 없음 |
| `stop` / `estop` | | 없음 |
| `set_speed` | 0~100 % | 없음 |

## 7. 설정 파일 예시
```yaml
# config/robot.yaml
port: COM5
calibration_id: so101_follower_01
urdf: assets/so101_new_calib.urdf
control_hz: 50
pid: {p: 16, i: 0, d: 32}
speed_pct_default: 30
joint_v_max_deg_s: 90
tool_v_max_mm_s: 100
approach_height_mm: 80
grasp_height_mm: 15        # 미정: 시험 물체가 정해지면 확정
gripper_margin_mm: 20
poses:                     # 아래 값은 자리표시. T4에서 실제 자세를 측정해 바꾼다
  home: [0, -100, 90, 70, 0, 50]
  park: [90, -100, 90, 70, 0, 50]   # 센서 시야 밖
```
```yaml
# config/workspace.yaml
table_mm: {x: 600, y: 400}
robot_base_in_table: {x_mm: 300, y_mm: 50, yaw_deg: 90}   # 배치 확정 후 수정
drop_zones:
  right: {x_mm: 500, y_mm: 150}
safety_margin_mm: 10
```

## 8. 출력과 기록
- 실행할 때마다 `runs/YYYYMMDD-HHMMSS/`를 만든다.
- `events.jsonl`: 명령, 결과(`MotionResult`), 오류를 한 줄씩 기록한다.
- `status.csv`: `ArmStatus`를 10Hz로 기록한다 (시각, 관절 6개, 그리퍼 끝 x·y·z, 최고 온도, 최대 부하).
- 이 기록이 F1(실행 로그)·F2(CSV 내보내기)의 원천이다.

## 9. CLI (GUI 전 개발·점검용)
```
python -m so101_studio status                 # 상태 1회 출력
python -m so101_studio home | park
python -m so101_studio movej <6개 관절값>
python -m so101_studio goto <x> <y> <z> <yaw>  # 작업대 좌표 mm, 도
python -m so101_studio pick <x> <y> <angle> <width> --place right
python -m so101_studio jog                     # 키보드 원격 조종
python -m so101_studio record <name> | replay <name> | seq <name>
python -m so101_studio --sim ...               # SimDriver로 실행
```

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

주의: 팔로워용 STS3215-C018은 12V 모터다. 다른 전압의 전원을 쓰지 않는다.

## 11. 구현 작업 순서 (로컬 세션)
| # | 작업 | 완료 기준 | 관련 기능 |
|---|---|---|---|
| T1 | 환경 구성: conda, Python 3.12 이상, `lerobot[feetech,kinematics]` | `import placo` 성공 여부를 기록. 실패하면 대체안 결정 기록 작성 | — |
| T2 | `ArmDriver`, `SimDriver`, CLI `status` | 관절값·온도·부하·전압 출력. 30초 정지 자세에서 떨림 관찰 기록 | A1, B7 |
| T3 | `Kinematics` + FK 검증 | 자세 3개에서 FK 결과와 자로 잰 그리퍼 끝 위치 차이 기록 | — |
| T4 | `trajectory.move_j`, `MotionController`, `stop`, `estop`, CLI `home`·`park`·`movej` | 각 관절 전체 범위를 저속 왕복, 끊김·떨림 없음. 정지 명령 0.3초 안에 멈춤 | B2, B3, B1 |
| T5 | `safety.py` | 한계 밖 명령이 모두 거부되는 단위 테스트 통과 | B6, B8 |
| T6 | `frames.py`, `move_l`, CLI `goto` | 작업대 위 9개 점(3×3)에 10회씩 이동, 평균 오차와 반복 편차(mm) 기록. 이 결과로 도달 범위와 작업대 배치를 확정 | B4 |
| T7 | 그리퍼 폭 대응표, `pick`, `place`, CLI `pick` | 손으로 넣은 좌표로 0/45/90/135° 물체 각 5회, 성공률 기록 | B5 |
| T8 | `teleop.jog` + 키보드 입력 | 관절·직교 조종 동작, 입력 끊기면 0.3초 안에 정지 | B9 |
| T9 | `teach.py`, `run_sequence` | 손으로 가르친 동작을 재생, 시연용 동작 1개 저장 | B10, B11 |
| T10 | `ArmService`, 실행 기록 | CLI가 모두 `ArmService`를 거치도록 정리, `runs/`에 기록 생성 | F1 |
| T11 | `retarget` | 이동 중 목표를 바꾸면 멈추지 않고 새 목표로 이어서 이동 | B12 |

T1~T7이 개발 1단계(동작 라이브러리)이고, T6 결과로 2단계(작업 공간 확정)를 한다. T8~T11은 센서를 기다리는 동안 할 수 있다.

## 12. 미정 사항
- 팔 배치(모서리 / 긴 변 가운데): T6 실측 후 확정. 코드는 `workspace.yaml`만 바뀐다.
- 수직 하강 전용 해석적 IK 추가 여부: T6 결과로 결정.
- 집는 높이 기본값과 시험 물체: 팀 논의 8번.
- GUI 기술: 기능명세서 확정 후 조사.
- 물리 비상 정지 스위치(12V 전원 차단): 하드웨어 담당과 논의.
