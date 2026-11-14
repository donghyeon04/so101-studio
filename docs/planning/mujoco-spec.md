# MuJoCo 물리 시뮬레이션·시험장 설계서

상태: 확정 · 2026-11-14 (`arm-software-spec.md` 13절에서 옮김. 13.1~13.9는 M1~M4로 구현됨, 13.10 시험장 계획은 결정 0062)

로컬 세션이 MuJoCo 드라이버(M1~M4)와 시험장(M5~M9)을 구현하는 기준이다. 절 번호는 옮기기 전과 같다(13.x). 팔 쪽 기본 규약(타입, `ArmService`, 안전, 집기·놓기)은 `arm-software-spec.md`, GUI 쪽은 `gui-spec.md` 12.8이다. 근거 결정: 0053(MuJoCoDriver), 0062(시험장), 0037(주제마다 도구 하나).

## 13. MuJoCo 물리 시뮬레이션 (결정 0053, 2026-11-11, EGM Stage 9 2라운드 반영, M4 전 점검 2026-11-13 반영)
**목표**: 지금 기구학 엔진(`SimDriver`)이 `ok`로 넘기는 접촉 실패(턱이 윗모서리를 침, 물체 밀림·쓰러짐·미끄러짐)를 `--sim`에서 보이게 한다. 11-10 실물 실패(배출 구역 다시 집기 `no_object`)는 "턱이 물체를 실제보다 낮다고 믿고 내려가 윗면을 친다"는 물리였다. 완료 기준 (a)는 그 로그를 다시 돌리는 것이 아니라 같은 물리를 합성 시나리오로 재현한다.

**용어**: 기구학 엔진 = `--sim-engine kinematic` = `SimDriver`, MuJoCo 엔진 = `--sim-engine mujoco` = `MuJoCoDriver`. 사용자에게 보이는 새 문장은 "MuJoCo 시뮬레이션"으로 쓴다. M2에 들어간 "물리 시뮬레이션이 멈췄습니다"(13.2)와 "물리 시뮬레이션에서는 --dry-run을 쓰지 않는다"(13.6)는 그대로 두고, GUI의 물리 멈춤 문장은 13.2 문장을 다시 쓴다.

**범위 나누기**: M1~M3(완료)는 CLI와 시험만 바꿨다. GUI는 M4(완료, #303·#328)에서 한 번에 바꿨다. GUI 쪽 설계는 `gui-spec.md` 12.8이 기준이고, 이 절은 팔 쪽(드라이버·뷰어·CLI)만 정한다.
- GUI의 엔진은 화면이 아니라 시작 인자로만 고른다: `so101 --sim --sim-engine mujoco [--sim-viewer] gui`.
- MuJoCo는 실제 팔 없는 `--sim` GUI에서만 쓴다. 실제 팔과 같이 띄우는 쌍둥이 시뮬레이션은 늘 `SimDriver`다.
- M4도 `cli/__init__.py`의 `make_driver(sim, cfg)`는 그대로 두고, GUI가 `make_sim_driver`를 직접 부른다.
- gui-spec 12.8이 확정되기 전에는 M4의 GUI 작업을 시작하지 않는다.

### 13.1 파일
| 파일 | 내용 | 단계 | 세션 |
|---|---|---|---|
| `arm/mujoco_model.py` | 모델 읽기(`load_model(scene, objects)` → `MjModel`, MjSpec으로 장면·물체·턱 접촉값을 더함), 관절 대응표 `JOINT_MAP`, 좌표 변환 `table_to_model_mm`·`model_to_table_mm` | M1 | 팔 |
| `arm/mujoco_model.py`(고침) | `load_model(scene=None, objects=(), *, base, table_z_mm, joint_limits=None)` → `(MjModel, warnings)`. `joint_limits`가 있으면 관절 range와 액추에이터 ctrlrange를 그 값으로 넓힌다(13.4) | M2 | 팔 |
| `assets/mujoco/so101/` | menagerie `robotstudio_so101`에서 `load_model`이 실제로 읽는 파일만(MJCF·쓰는 메쉬)과 `LICENSE`(Apache-2.0), 가져온 menagerie 커밋을 적은 `SOURCE.md`. 원본 파일은 고치지 않는다. 우리가 더하는 것은 XML이 아니라 `load_model`(MjSpec)이 한다(13.4) | M1 | 팔 |
| `arm/mujoco_driver.py` | `MuJoCoDriver(cfg, ws, *, scene=None, objects=(), realtime=True, viewer=False, scenario=None)`. 모델은 `connect`에서 만든다. `sim_state()`는 M2에서 `{"t_s": 시간, "objects": {}, "contacts": [], "held": None}` | M2 | 팔 |
| `arm/sim_driver.py` | `make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False)` → `MuJoCoDriver`. M2는 `scene`만 넘겼고(M1 `load_model`이 이미 받음) `scenario`는 M3에서 쓰기 시작했다. `viewer`는 M4에서 쓴다. `SimDriver`와 `cli/__init__.py`의 `make_driver(sim, cfg)`는 그대로 둔다 | M2 | 팔 |
| `arm/sim_driver.py`(M3) | 시나리오와 `config/manual_objects.yaml` 읽기, 오류 처리(13.5) | M3 | 팔 |
| `cli/__init__.py` | M2: 인자 `--sim-engine {kinematic,mujoco}`(기본 kinematic). `main()`의 드라이버 만들기에서 `a.sim and a.sim_engine == "mujoco"`일 때만 `make_sim_driver`를 부르고, 아니면 지금처럼 `make_driver`. `mujoco` import 실패(ImportError)는 이 자리에서 잡아 13.8 문장(`MUJOCO_MISSING_TEXT`)을 출력하고 `sys.exit(2)`. M3에 `--sim-scenario`(13.5 문장). `--sim-viewer`는 아래 M4 행 | M2~M3 | 팔 |
| `service/diagnostics.py` | `isinstance(driver, SimDriver)` → `getattr(driver, "simulated", False)` | M2 | 팔 |
| `service/arm_service.py` | `reload_workspace`: 드라이버가 `physical_objects`면 거부(13.5) | M2 | 팔 |
| `config/sim_scenarios/<이름>.yaml` | 물체 시나리오(13.5). (a)용 `topedge.yaml`, (b)용 `carry30.yaml`·`carry45.yaml`·`carry58.yaml` | M3 | 팔 |
| `pyproject.toml` | 선택 의존성 `mujoco = ["mujoco>=4.0,<5"]` | M1 | 팔 |
| `environment.yml` | 기본에 넣지 않고 주석으로 설치법(13.8) | M1 | 팔 |
| `tests/test_mujoco_model.py` | (d) | M1 | 팔 |
| `tests/test_mujoco_driver.py` | M2 시험(13.7 (e) 목록) | M2 | 팔 |
| `tests/test_mujoco_scenarios.py` | (a)(b) | M3 | 팔 |
| `arm/mujoco_driver.py`·`arm/sim_driver.py`(M4) | 뷰어(아래 "뷰어"), `sim_state()` 물체에 `size_mm` 더하기(13.6), 읽기 전용 속성 `failed_reason`(물리가 멈췄으면 그 이유 문자열, 아니면 None, 13.2), 물체 겹침 검사(13.5) | M4 | 팔 |
| `config/manual_objects.yaml`(M4) | 기본 물체가 서로 겹치지 않게 고친다(13.5) | M4 | 팔 |
| `tests/test_mujoco_driver.py`(M4) | 13.7 (f) 팔 시험 | M4 | 팔 |
| `cli/__init__.py`(M4) | `gui` 거부 분기를 지운다. `--sim-viewer`(store_true)를 더하고, `--sim --sim-engine mujoco` 없이 주면 "--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다"로 종료 코드 1이다. `gui.main(a)`는 Namespace를 그대로 넘기므로 GUI가 `a.sim_engine`·`a.sim_viewer`를 읽는다 | M4 | 팔 |
| `gui/__main__.py`, `gui/app.py`, `gui/backend.py`, `gui/scene3d.py`, `gui/sensors/manual_view.py`, `gui/pages/table.py`, `tests/gui/test_mujoco_gui.py` | `gui-spec.md` 12.8이 기준이다 | M4 | GUI(팔 세션이 REQ로 요청) |
- 시험은 모두 `pytest.importorskip("mujoco")`로 없으면 건너뛰고, 10초 넘는 것은 `slow`로 표시한다.
- **`--sim-engine mujoco`일 때 명령 처리**(M2, 2026-11-13 코드 대조로 정정):
  - 먼저 검사한다. `--sim` 없이 `--sim-engine mujoco`면 "--sim-engine은 --sim과 함께 쓴다"(종료 코드 1, 13.7 (c)). `--sim-engine kinematic`은 `--sim` 없이 줘도 기본값이라 오류가 아니다.
  - 드라이버로 팔을 움직이는 명령(`status`, `home`, `park`, `movej`, `goto`, `pick`, `place`, `gripper`, `run`, `jog`, `keys`, `record`, `replay`, `seq`, `powertest`, `shaketest`, `servoinfo`)과 `info`는 MuJoCo 드라이버를 만들고 connect한다. 그래서 `info`도 mujoco가 없으면 종료 코드 2다.
  - `doctor`, `find-port`, `incident`는 위 검사만 거친 뒤 엔진을 쓰지 않고 돈다.
  - `calibrate`, `linkwatch`는 "<명령>은 시뮬레이션에서 쓸 수 없다"로 거부한다(종료 코드 1).
    - 기구학 엔진(`--sim`)에서는 다르다. `calibrate --auto`는 자동 캘리브레이션 쪽에서 거부하고(종료 코드 2), `calibrate --verify-zero`는 거부하지 않는다. `linkwatch`는 `cfg.port`를 연다(판단 필요: 시뮬레이션에서도 실포트를 여는 것이 맞는지).
  - `gui`: M3까지는 "GUI의 MuJoCo는 M4부터입니다"로 거부했다. M4에서 팔 세션이 이 분기를 지우고, `test_cli_gui_rejected`를 `test_cli_gui_passes_engine`으로 바꾼다. 이 시험은 `tests/test_cli_share.py`처럼 가짜 gui 모듈을 넣어 받은 `a.sim_engine == "mujoco"`·`a.sim_viewer`를 확인한다(팔 세션 시험이 nicegui를 import하지 않게).
- **뷰어**(M4, `--sim-viewer`):
  - `make_sim_driver`는 `viewer`를 `MuJoCoDriver(..., viewer=viewer)`로 넘긴다.
  - `viewer`가 참이면 `connect`가 모델을 만든 뒤 `mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)`를 연다. 메인 스레드가 아니어도 되는 passive 뷰어라 GUI의 `ui.run`과 같이 쓰려는 것이다(아래 첫 작업에서 확인).
  - 물리 스레드가 잠금을 쥔 채 1/30초마다 한 번 `handle.sync()`를 부른다. 뷰어의 마우스 힘 주기는 쓰지 않는다(sync 전에 `data.xfrc_applied`를 0으로).
  - `disconnect`가 `handle.close()`를 부른다(`connect`는 이미 먼저 `disconnect`를 부른다). 사용자가 창을 닫으면(`handle.is_running()`이 거짓) sync만 멈추고 물리는 계속한다.
  - `realtime=False`와 `viewer=True`를 함께 주면 `MuJoCoDriver.__init__`에서 `ValueError("뷰어는 실시간 모드에서만")`이다.
  - M4 첫 작업으로, 설치된 mujoco 4.0 메타 휠에 `mujoco.viewer.launch_passive`가 있는지와 Windows의 메인 아닌 스레드에서 열리는지를 확인해 13.9에 남긴다. 안 되면 멈추고 기획에 알린다.

### 13.2 드라이버 규약
- `SimDriver`의 하위 클래스로 만들지 않는다. 시험 전용 속성(`grip_block`, `grip_load`, `torque_limit_writes`, `simulated_reset`, `simulated_status`)은 갖지 않는다.
- **속성**: `connected`, `acceleration`(10), `simulated = True`, `locked_reason = None`, `event_sink = None`, `expect_torque`, `torque_limit`(`gripper_torque_limit()`과 같은 값), `physical_objects = True`, `connect_info = {"engine": "mujoco", "scenario": 이름 또는 None}`(`ArmService`가 connect 기록에 넣는다).
  - `simulated`가 없으면 `ArmService`가 모든 동작을 잠근다.
- **메서드**(`SimDriver`와 같은 이름·인자·반환 모양):
  - `connect(port="sim", calibration_id="sim")`: 두 위치 인자를 받고 쓰지 않는다. `ArmService`가 `driver.connect(port, calibration_id)`로 부른다.
  - `disconnect`, `read_joints`, `write_joints`, `torque_enabled`, `set_torque`, `set_acceleration`(값만 저장), `set_gripper_torque_limit`, `gripper_torque_limit`, `read_servo_info`({관절: {"Acceleration": 값}})
  - `read_health`: 7키. `temperature_c`(25.0, `SimDriver`와 같게), `load_pct`, `voltage_v`(12.0), `current_ma`, `current_ma_by_joint`, `torque`(관절별 0/1), `status_bits`(모두 0)
  - `read_power`: (12.0, 서보별 전류 mA, 서보별 부하 %)
  - `probe_reset`: 8키(`ping`, `torque`, `acceleration`, `torque_limit`, `status`, `voltage_dv`, `ids`, `expected_acceleration`), 늘 정상
- **토크와 목표**(실물 서보처럼):
  - `connect`는 모델을 `cfg.poses["home"]` 자세로 두고, 목표 = home, **토크 켬**으로 물리를 시작한다. 그래서 `MotionController`가 처음 읽는 값과 CLI `status`는 home이다(처지지 않는다).
  - `set_torque(on)`은 **모든 관절에 한 번에** 적용한다.
    - 끄면 모든 액추에이터 힘이 0이 되어 팔이 중력으로 처진다.
    - 켜면 그 순간의 관절값을 목표로 다시 잡는다(튀지 않게).
  - 토크가 꺼진 동안 `write_joints`는 목표를 저장만 하고 힘을 내지 않는다. 다시 켤 때는 저장한 값이 아니라 그 순간의 관절값을 목표로 쓴다.
  - 그리퍼 `Torque_Limit` v(0~1000)는 그리퍼 액추에이터 `forcerange`를 원래 값 × v/1000으로 줄인다.
- **목표 선형 보간**(M3, 2026-11-13): `write_joints`는 액추에이터 목표(ctrl)를 바로 바꾸지 않고, 다음 쓰기 간격(1/`control_hz`, 기본 20ms) 동안 이전 목표에서 새 목표로 선형으로 옮긴다. 서보 kp 998에서 50Hz 계단 목표는 액추에이터 힘을 한계까지 순간적으로 튀게 해 부하 감시(`ARM-GRIP-OVERLOAD-WARN`)를 걸고 그리퍼 열기를 멈췄다. 실물 서보도 목표를 부드럽게 따라가므로 이것에 맞춘다. 서보 원본 값(kp·kv)은 그대로다. 결과: 그리퍼 열기 부하 약 24%, (a) 대조 3회 연속 통과.
- **상태 값**(서보 모델이 없으므로 근사):
  - 부하 %: |액추에이터 힘| / **원래(줄이기 전)** `forcerange` × 100. 쥔 그리퍼는 운반 중 `gripper_hold_torque` 200이면 20% 이하, 닫는 중 `gripper_squeeze_torque` 300이면 30% 이하다. `gripper_load_warn_pct` 60 아래라 정상 집기에서 경고·fault가 나지 않는다.
    - 액추에이터 힘에는 kv·damping(속도 항)도 들어 있다. 그래서 목표를 계단으로 바꾸면 순간적으로 한계까지 튈 수 있는데, 이것은 아래 "목표 선형 보간"으로 막는다(M3, 2026-11-13). 한때 승인했던 "위치 오차 항만 세는" 정의 변경(f173c90)은 보간으로 원인이 풀려 쓰지 않는다.
  - 전류: 관절별 부하 % × 10mA(부하 100% = 1000mA), 합이 `current_ma`.
  - 부하·전류·과열 감시는 끄지 않는다. 정상 동작에서 걸리면 모델의 서보 값(kp·forcerange)이 실물과 다른 것으로 보고 모델을 고친다.
- **병렬 그리퍼**(M4b, 결정 0063): 팔 파일의 그리퍼가 `parallel`이면 `make_sim_driver`가 로보나인 손 모델(`arm/mujoco_hand.py`)을 쓴다. menagerie 단동식 손을 MjSpec에서 지우고 손 본체와 클램프 둘(slide, 한쪽 0~42mm)을 URDF 좌표로 붙인다. 왼쪽 클램프는 equality joint(solref 0.004)로 오른쪽의 반대 방향에 묶고 액추에이터는 `gripper` 하나다. 값↔이동은 `gripper_width_mm` 표(폭/2). 충돌은 메시 정점을 z로 자른 조각의 경계 상자(손 3개, 클램프 8개씩, 클램프 상자는 jaw 접촉값 condim 4·마찰 (1.0, 0.02, 0.001)·priority 1), 시각은 원본 메시. 접촉 역할 이름은 `clamp_right`·`clamp_left`. `gripper_frame`은 병렬 URDF origin(z −121.8mm). 두 클램프는 닫힌 자리에서 0.2mm 겹쳐 있어 서로의 접촉은 뺀다. 액추에이터 kp 10000 N/m·kv 20·힘 한계 10N(URDF effort)은 잠정값이고 실물 조임 힘 측정(runbook 2-10) 뒤 바꾼다. 13.7 (a)(b)·`sim_blocked`·부하 문턱은 이 손으로 다시 잰다(후속).
- **이벤트 통로**: 드라이버의 WARN·ERROR는 `event_sink(kind, level, code, data)`로 보낸다. `ArmService.connect`가 `driver.connect`를 부르기 직전에 `driver.event_sink`를 이어 준다(실물 `ArmDriver`와 같은 통로). `load_model`은 `MuJoCoDriver.connect` 안에서 돌므로 그 경고도 이 통로로 간다.
  - `event_sink`가 아직 None일 때 생긴 이벤트(예: `connect` 전 `load_model` 경고)는 드라이버가 목록에 모아 두었다가 처음 보낼 수 있을 때 함께 보낸다.
  - 코드: `SIM-JOINT-RANGE-DEFAULT`(WARN), `SIM-OBJECT-HEIGHT-DEFAULT`(WARN, M3), `SIM-PHYSICS-FAIL`(ERROR), `SIM-PHYSICS-DROP`(WARN, 13.3에서 버림이 생길 때마다).
- **물리 실패**: 스텝 중 예외가 나거나 MuJoCo 경고 카운터(`data.warning`의 `mjWARN_BADQACC` 등)가 늘거나 관절값이 NaN이면 다음과 같이 한다.
  - 스레드가 멈추고 `SIM-PHYSICS-FAIL`(이유)을 보낸다.
  - 그 뒤 `read_joints`·`write_joints`·`read_health`는 `RuntimeError("물리 시뮬레이션이 멈췄습니다: 이유")`를 낸다. `ArmService`는 이것을 실물의 통신 오류와 같이 다룬다(지금 통신 끊김 처리로 정지).
  - 이유는 읽기 전용 속성 `failed_reason`(M4)으로 읽는다. 멈추지 않았으면 None이고, 다시 `connect`하면 None으로 돌아간다. 물리가 멈추면 상태 루프의 `read_joints`가 예외를 내 `status_stream` 콜백이 돌지 않으므로, GUI는 이 속성으로 감지한다(`gui-spec.md` 12.8).
  - 되살리기는 `disconnect`·`connect`(모델을 새로 만듦)다.

### 13.3 시간
- 물리는 `MuJoCoDriver` 안의 **백그라운드 스레드**가 벽시계에 맞춰 계속 돌린다. `connect`에서 시작하고 `disconnect`에서 멈춘다.
- 물리 스텝은 2ms다. 스레드는 깨어날 때마다 시뮬레이션 시간이 벽시계를 따라잡을 만큼 스텝을 몰아서 진행하고, 1ms 쉰다. Windows의 `time.sleep` 해상도가 약 15ms여도 이 방식이면 시간은 맞는다.
- 따라잡아야 할 양이 100ms를 넘으면 넘는 만큼 버린다. **연결 직후**에는 `ArmService.connect`가 기구학을 읽는 동안 GIL을 쥐어 수 초가 버려질 수 있다(#275 실측: 약 3.9초 × 2회, `measurements.md` "MuJoCo 실시간 속도"). 팔은 연결 뒤에 움직이므로 받아들이고, 13.7 (e)의 측정은 연결이 끝난 뒤부터 한다. `dropped_count`(횟수)와 `dropped_s`(버린 시간 합)를 센다. 스텝 계산 시간의 평균은 `step_ms_avg`로 읽는다.
- 그래서 제어 주기(`control_hz`, `--hz`)와 상관이 없다. 쉬는 중, 토크를 끈 교시, 놓은 뒤, `estop`·fault·`hold_gripper` 같은 틱 밖 호출에도 물리가 흐른다.
- 읽기·쓰기는 스레드와 잠금 하나로 맞춘다.
- 시험용 `MuJoCoDriver(..., realtime=False)`: 스레드가 없고 `advance(seconds)`만 시간을 흘린다. `write_joints`는 목표만 바꾼다. `ArmService` 위 시험((a)(b))은 실시간 모드를 쓴다.

### 13.4 모델과 좌표
- **월드 좌표 = URDF `base_link` 프레임**(m). 우리 기구학 모델 좌표와 같다.
- **좌표 변환**(M1, `arm/mujoco_model.py`에 공개 함수로):
  - `table_to_model_mm(x, y, h, base, table_z_mm)`: 작업대 점(작업 면 위 높이 h) → 모델 좌표 mm. 순서는 `frames.table_to_robot`로 로봇 좌표(pan 축 원점)로 바꾸고, x에 `PAN_AXIS_X_MM`(38.8mm)를 더하고, z = `table_z_mm` + h.
  - `model_to_table_mm`는 그 역이다.
  - `kinematics.ik`·`collision.py`·`scene3d.py` 안의 같은 계산은 이번에 바꾸지 않는다. (d)가 두 경로가 같은지 확인한다.
  - `table_z_mm`는 `cfg.table_z_mm`(팔 파일까지 겹친 값), `base`는 `ws.base`다.
- **관절 대응표** `JOINT_MAP = {우리 관절: (MuJoCo 관절 이름, 부호 ±1, 오프셋 도)}`:
  - M1의 산출물로 `arm/mujoco_model.py` 상수에 둔다. 모델 파일은 고치지 않는다.
  - M1 결과: 관절 이름이 우리와 같고, 모두 부호 +1·오프셋 0이다. (d) 10자세 최대 차이 0.003mm.
  - 그리퍼 값 v ↔ MuJoCo 그리퍼 관절각은 `VALUE_TO_Q`(q = −10 + 1.214 v 도)에 부호·오프셋을 적용한 값이다.
- **더하는 것**: XML include로는 원본의 중첩 body 안에 site를 넣을 수 없어서, 모두 `load_model`이 MjSpec으로 더한다. 원본 파일은 고치지 않는다(M1 #260).
  - gripper body에 site `gripper_frame`을 둔다. 위치·방향은 우리 URDF `gripper_frame_joint`의 origin을 그대로 쓴다(`xyz="-0.0079 -0.000218121 -0.0981274" rpy="0 3.14159 0"`, gripper_link 좌표). 원본의 `gripperframe` site는 위치가 우리 URDF와 달라 쓰지 않는다.
  - 받침: 원본에 충돌 geom이 없다. base body에 visual 메쉬의 경계 상자를 충돌 geom `base_box`(상수 `BASE_BOX_POS`·`BASE_BOX_HALF`)로 더한다.
  - **관절 범위**: 원본 range는 우리 한계보다 좁다(예: pan ±110° vs ±120.3°, wrist_flex ±95° vs ±103.6°). **M2에서** `load_model(..., joint_limits=cfg.physical_limits_deg)`가 각 관절의 range와 **그 관절 액추에이터의 ctrlrange를 함께** `physical_limits_deg`(실물 AC4 측정, REQ-054)로 넓힌다. range만 넓히면 ctrlrange가 옛 범위라 목표가 잘린다(그리퍼는 약 90.6에서 멈춤).
    - 이유: 시뮬레이션의 기계적 끝은 실물과 같아야 한다. 좁은 range를 두면 우리 안전 한계(`joint_limits_deg`, 실물보다 1° 안쪽) 근처 명령이 MuJoCo에서만 막혀, 실물에는 없는 실패가 생긴다.
    - 안전 검사는 지금처럼 `joint_limits_deg`로 하므로 정상 명령은 MuJoCo range에 닿지 않는다.
    - 그리퍼는 `physical_limits_deg`의 그리퍼 값(0~100 단위)이 아니라 각도로 쓴다: range·ctrlrange = [`gripper_value_to_deg(0)` − 1°, `gripper_value_to_deg(100)` + 1°](라디안으로). 그래야 값 100까지 간다.
    - `joint_limits`가 None(팔 파일에 `physical_limits_deg`가 없음)이면 원본 range를 쓰고, `load_model`이 돌려주는 `warnings`에 `SIM-JOINT-RANGE-DEFAULT`를 담는다. 드라이버가 그것을 WARN 이벤트로 보낸다(13.2 이벤트). 일부 관절만 있으면 빠진 관절은 경고 없이 원본 range를 쓴다.
- **턱 접촉**:
  - `load_model`은 `gripper`·`moving_jaw_so101_v1` body에 바로 붙은 geom 가운데 원본 `collision_gripper`·`collision_gripper_mesh` 클래스(둘 다 priority 1, contype ≠ 0. 원본 condim 6, friction `1 0.005 0.0005`)인 것을 모두 `condim=4`, `friction="1.0 0.02 0.001"`로 바꾼다. priority 1이 남아 있어 턱–물체 접촉은 턱 쪽 값을 쓴다. 손목 서보 상자(`so101.xml`, priority 0)는 바꾸지 않는다(M3 #290 코드와 같음).
  - 전역 옵션은 `cone="elliptic" impratio="10" integrator="implicitfast" timestep="0.002"`이다(시험값). 원본 timestep 0.005(`so101.xml`)를 0.002로 바꾸고, `cone`·`impratio`·`integrator`는 원본과 같은 값이다.
  - 접촉을 보고할 때는 geom 이름 대신 **body의 역할 이름**을 쓴다: `jaw_fixed`(그리퍼 body), `jaw_moving`(가동 턱 body), `arm`(나머지 팔 body), `table`, `obj:<이름>`, `scene:<이름>`.
  - 물체와 장면 부품은 **body 이름 자체를** `obj:<이름>`·`scene:<이름>`으로 만들어, 역할 이름을 따로 표로 두지 않는다. 그 아래 geom 이름은 자유다.
  - 역할은 다음 순서로 정한다: geom `table`이면 `table`, body `gripper`의 priority 1 geom이면 `jaw_fixed`(손목 서보 상자처럼 priority 0인 geom은 `arm`), `moving_jaw_so101_v1`이면 `jaw_moving`, body 이름이 `obj:`·`scene:`으로 시작하면 그 이름, 나머지(`base`, `camera_mount` 포함)는 `arm`.
- 서보는 원본 값으로 시작한다: kp 998.22, kv 2.731, forcerange ±2.94, armature 0.028, damping 0.60, frictionloss 0.052(`so101.xml`). 공개 값은 kp 17.8~998로 약 56배 갈린다. 실물 `status.jsonl`로 맞추는 것은 M3 뒤 별도 REQ다.

### 13.5 장면과 물체
- `make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False)`. CLI는 `load_scene()`으로 읽은 장면을 넘긴다(`ArmService`와 같은 함수). home 자세는 드라이버가 `cfg.poses["home"]`에서 읽는다.
- **시나리오 읽기**(M3): 시나리오는 `make_sim_driver`가 읽는다. 이름이면 `config/sim_scenarios/<이름>.yaml`, `.yaml`로 끝나면 그 경로다. `objects`의 각 행을 `SimObject(**row)`로 만든다.
  - **경로 기준**(판단 J4): 이름과 `config/manual_objects.yaml`은 **레포 루트 기준**이다(`mujoco_model.py`처럼 `__file__`에서 레포 루트를 구한다). `so101.cmd`는 어느 폴더에서든 실행하기 때문이다. 지금 코드(`sim_driver.py`)는 작업 폴더 기준이라 팔 REQ로 고친다. `.yaml` 경로는 사용자가 준 그대로(작업 폴더 기준) 쓴다.
  - 파일이 없거나, 모르는 키·`name` 빠짐·이름 중복이면 `ValueError`를 내고, CLI는 그 문장을 출력하고 종료 코드 1로 끝난다.
  - 시나리오가 없으면 같은 함수가 `config/manual_objects.yaml`을 읽는다. 파일이 없으면 물체 없이 시작한다. 행의 키가 틀리면 시나리오와 같이 `ValueError`(지금은 TypeError traceback이라 팔 REQ로 고친다).
  - 만든 목록은 `MuJoCoDriver(cfg, ws, scene=scene, objects=objs, scenario=이름)`으로 넘기고, `scenario`(`connect_info`)에는 확장자를 뺀 파일 이름을 넣는다.
  - `--sim-scenario`를 `--sim --sim-engine mujoco` 없이 주면 "--sim-scenario는 --sim --sim-engine mujoco와 함께 쓴다"로 종료 코드 1이다.
- `connect`에서 `load_model`로 작업대(작업 면 평면)·장면 부품·물체를 더해 컴파일한다.
- **장면 부품**(`config/scene.yaml`):
  - `attach`가 작업대면 월드 고정 body, 링크면 그 링크 body의 자식이다. 모르는 링크 이름이면 `connect`에서 `ValueError`가 나고 CLI는 "연결 실패"로 끝난다. 링크에 붙는 부품 geom은 질량 0으로 둔다(팔 동역학을 바꾸지 않게. 지금 `scene.yaml`에는 링크 부품이 없다).
  - `collide: false`는 보이기만 하는 geom이다(`contype=0 conaffinity=0`).
  - `margin_mm`는 물리에 쓰지 않는다. `box_mm`가 있으면 상자, 없으면 `file`의 메쉬다(둘 다 있으면 `box_mm`). MuJoCo에서는 꼭짓점 8개·면 12개 메쉬만 box geom이고 나머지는 mesh geom(충돌은 볼록 껍질)이다. 파일이 없으면 CLI는 그 오류 문장으로 끝난다.
- **물체**:
  - 시나리오가 없으면 `config/manual_objects.yaml`의 물체를 `ManualSource`와 같은 이름 `obj-1`, `obj-2`, …(파일 순서, 1부터)로 놓는다. body는 `obj:obj-1`이다. M2는 물체를 넘기지 않았다(`objects=()`).
  - 시나리오 물체는 `name`이 필수이고 서로 달라야 한다. 시나리오 파일 이름과 물체 이름은 별개다.
  - 물체 행의 키는 `SimObject` 필드(`name, x_mm, y_mm, angle_deg, width_mm, length_mm, height_mm, mass_g`)이고 필수는 `name`·`x_mm`·`y_mm`이다. 기본값은 각도 0, 폭 30, 길이 100, 높이 0(→ 40), 질량 30g이다. 마찰은 `1.0 0.005 0.0001`(작업대와 같음)이다. 상자의 길이 축이 `angle_deg` 방향이다. 시험 시나리오는 길이를 늘 적는다(기본 100이면 막대가 된다).
  - 높이가 없는(0) 물체는 상자 높이 40mm로 놓는다. `load_model`의 warnings 항목은 `(code, data)`이고, 높이 0 물체마다 `("SIM-OBJECT-HEIGHT-DEFAULT", {"object": 이름, "height_mm": 40})`을 하나씩 담아 드라이버가 WARN으로 보낸다. 계획 쪽 물체 높이는 0으로 남는다. 그래서 기울이지 못하는 집기는 `height_required`로 거부된다(결정 0052).
  ```yaml
  # config/sim_scenarios/topedge.yaml (a)
  objects:
    - {name: box58, x_mm: 300, y_mm: 275, angle_deg: 45, width_mm: 58, length_mm: 58, height_mm: 39}
  ```
  ```yaml
  # config/sim_scenarios/carry30.yaml (b). carry45·carry58은 폭·길이만 45·58로 다르다(정사각 상자)
  objects:
    - {name: box, x_mm: 300, y_mm: 200, angle_deg: 0, width_mm: 30, length_mm: 30, height_mm: 40}
  ```
  - (300, 275)는 `robot_base_in_table` (300, 50)·yaw 90°에서 로봇 x축(정면)으로 225mm다. 팔 원점을 다시 재면(runbook 2-A A-1) 시나리오 YAML의 `x_mm`·`y_mm`와 13.7 (a)(b)의 좌표를 로봇 정면 225mm(`carry`는 150mm) 점으로 함께 고친다.
- **작업대 설정 다시 읽기**: MuJoCo 월드는 `connect` 때 만들어지므로, `ArmService.reload_workspace`는 드라이버가 `physical_objects`면 "MuJoCo 시뮬레이션은 작업대 설정을 바꾸면 다시 시작해야 합니다"로 거부한다.
- **물체 겹침**(M4, 2026-11-13): 물체끼리는 충돌하므로, 처음 자리에서 바닥 사각형(길이 × 폭, `angle_deg`로 돌림)이 서로 겹치면 MuJoCo가 첫 스텝에서 밀어내 계획 쪽 좌표와 어긋난다.
  - `make_sim_driver`는 시나리오와 `config/manual_objects.yaml` 모두에서 두 물체의 바닥 사각형이 겹치거나 닿으면(분리축 검사) "<파일>의 <이름>과 <이름>이 겹친다"로 `ValueError`를 낸다. CLI는 종료 코드 1, GUI는 "연결 실패"(gui-spec 12.8)다.
  - 지금 `config/manual_objects.yaml` 기본 물체는 obj-1(300,190,0°)과 obj-3(350,200,90°), obj-1과 obj-2(250,210,45°)가 겹친다. 팔 M4에서 이 값으로 고친다: obj-1 (300, 190, 0°), obj-2 (170, 170, 45°), obj-3 (400, 250, 90°). 모두 100×30, 높이 40이다. 조건: 서로 30mm 넘게 떨어지고, 로봇 원점에서 90~250mm이고, 알려진 물체 원(반지름 = 길이/2, GUI가 팔에 알리는 값)이 배출 구역 칸(`ws.zone_slots`), `tests/gui/test_manual.py:253`의 놓기 점 (420,120), gui-spec 12.8 시험 1의 빈 점 (300,275)에서 반지름 + 17mm 넘게 떨어진다(계산: 원점에서 140·177·224mm, obj-1과 obj-3은 x로 35mm, obj-3에서 칸 (442.5,187.5)까지 75.6mm·(420,120)까지 131mm·(300,275)까지 103mm, obj-1에서 (300,275)까지 85mm. IK로는 돌려 보지 않았다). 팔 M4 PR은 바꾼 뒤 `tests/gui/test_manual.py`·`tests/test_task.py`가 그대로 통과하는지 총괄에게 확인받고, 깨지면 합치기 전에 GUI REQ로 넘긴다. 실제 물체를 놓은 자리가 생기면 같은 조건으로 바꾼다.
- 초음파 가상 ESP32의 demo 배치와 같은 물체를 놓는 것은 나중이다(초음파 세션과 맞출 때).

### 13.6 시뮬레이션 전용 읽기와 `no_object` 규칙
- `sim_state()`(`MuJoCoDriver`에만 있다. M2는 빈 값이었고 M3 #290에서 채웠다):
  ```
  {"t_s": 시뮬레이션 시간,
   "objects": {이름: {"x_mm", "y_mm", "z_mm", "yaw_deg", "tilt_deg", "size_mm"}},
   "tool": {"pos_mm": [x_mm, y_mm, z_mm], "rot": 3×3 행렬},
   "contacts": [{"a": 역할 이름, "b": 역할 이름, "force_n": 법선 힘, "pos_mm": [x_mm, y_mm, z_mm]}],
   "held": 물체 이름 또는 None}
  ```
  - 좌표는 모두 작업대 좌표 mm이고, `z_mm`는 작업 면 위 높이(13.4의 h)다.
  - `objects`: 위치는 상자 바닥 가운데. `yaw_deg`는 시나리오 `angle_deg`와 같은 기준(작업대 x축에서 반시계, −180~180), `tilt_deg`는 상자 위쪽 축과 연직 사이 각. `size_mm`는 [길이, 폭, 높이](높이 0이던 물체는 40. M4에서 더한다. GUI 3D가 쓴다). 쓰러진 방향(회전 행렬)은 담지 않는다.
  - `tool`: `gripper_frame` site의 위치와 축(작업대 좌표·축). 미끄러짐 판정(13.7 (b))에 쓴다.
  - `contacts`: MuJoCo 접촉점 하나마다 한 항목이고, a·b 순서는 정하지 않는다(시험은 순서 없이 비교한다). `force_n`은 그 점의 법선 힘(N)이다. `a`·`b`는 역할 이름(`obj:<이름>` 포함)이고, `objects`의 키와 `held`는 접두어 없는 이름이다.
  - `held`: `jaw_fixed`와 `jaw_moving` 둘 다와 법선 힘 > 0으로 접촉한 물체의 이름(접두어 없이) 또는 None. 둘 이상이면 `force_n` 합이 큰 것.
  - M2에서는 `objects`·`contacts`가 빈 값, `tool`이 None, `held`가 None이었다.
- **`no_object`를 성공으로 치는 규칙**: 기구학 엔진(`SimDriver`, `--sim-engine kinematic`)은 쥘 물체가 없어서 `no_object`를 성공으로 친다. 드라이버의 `physical_objects`가 참이면 이 규칙을 쓰지 않는다.
  - CLI 작업 실행: `--dry-run`(→ `TaskRunner(accept_no_object=True)`)을 MuJoCo와 함께 주면 "물리 시뮬레이션에서는 --dry-run을 쓰지 않는다"로 거부한다(M2). `Studio`·`TaskRunner`는 고치지 않는다. `test_run_modes.py`·`test_task.py`는 기본 엔진이라 영향이 없다.
  - GUI(M4): `gui-spec.md` 12.8의 "`no_object` 규칙"을 따른다. 판정은 `getattr(driver, "physical_objects", False)`로 한다(`SimDriver`에는 이 속성이 없다).

### 13.7 완료 기준
- **(a)(b) 공통**: cfg는 `config/robot.yaml` + `config/arms/so101_follower_01.yaml`(local.yaml 없이)이다. 지금 시험은 `load_robot`이 `local.yaml`과 팔 고르기를 겹쳐 PC마다 결과가 달라질 수 있어 팔 REQ로 고친다. `driver = make_sim_driver(cfg, ws, None, scenario=이름)`, `svc = ArmService(driver, cfg, ws, log_root=tmp_path)`, `svc.connect()`. 집기는 `svc.pick_and_place(obj)`(놓을 곳 없이 집기만), 놓기는 `svc.place(at=(x, y))`다. 둘 다 `slow`로 표시한다.
- (a) **재현**(M3, 시나리오 `topedge`):
  - `obj = DetectedObject(id="box58", x_mm=300, y_mm=275, angle_deg=45, width_mm=58, length_mm=58, confidence=1.0, source="manual", height_mm=15)`를 넘긴다(실제 상자 높이 39).
    - 계획 쪽 계산: 각도 45°라 `tilt_ok=False`이고, 높이 15를 아니까 결정 0052의 `height_required`에 걸리지 않는다.
    - 기호는 11.2와 같다. W 물체 폭 58, H 계획에 넘긴 높이 15, t 낮은 턱 끝 높이, G 시작 벌림(`Skills._needed_gap`이 고름), s = `Gripper.narrowing(v(G), tilt_ok=False)`.
    - t = min(`grasp_height_mm` 20, H − `grasp_min_contact_mm` 8) = 7mm. G는 W + `gripper_margin_mm` = 78이고, 78에서 s ≈ 1.45라 (H − t)·s ≈ 11.6 ≤ G − W − 4 = 16이므로 계획대로 t = 7까지 내려간다.
    - 실제 윗면 39mm에서 턱 사이는 78 − (39 − 7)·1.45 ≈ 31.5mm < 58이라 턱이 윗모서리를 친다. 숫자는 `config/arms/so101_follower_01.yaml`의 `gripper_width_mm` 기준이다(EGM 리뷰 계산, 실행 전).
  - **기대**: `pick_and_place`가 도는 동안 시험 스레드가 `sim_state()`를 50Hz로 읽는다. {`jaw_fixed` 또는 `jaw_moving`, `obj:box58`} 접촉 가운데 `pos_mm`의 `z_mm`가 34~44mm(윗면 39 ±5)인 것이 한 번 이상 나온다.
  - 판정은 위의 **윗면 접촉 하나**다(2026-11-13 결정: 재현의 핵심 물리). 결과 status와 `held`는 판정하지 않고 시험 출력에만 남긴다.
    - 이유: M3 실측에서 턱이 윗모서리를 스친 뒤 계획 깊이(턱 끝 7mm)까지 내려가 결국 쥐었다(`held`=box58, 기울기 약 1°). 계획의 좁아짐 계수 s(URDF 메쉬 기준, 약 1.45)와 MuJoCo 턱 충돌 형상(`collision_gripper` geom)이 달라 시뮬레이션의 턱이 계획보다 덜 좁아진 것으로 본다.
    - 실물에서도 쥐는지는 놓기 실물 재시험(runbook 2-2) 때 같은 배치(정면 225mm, 폭 58·높이 39, 각도 45, 높이 15로 알려 줌)로 본다. 실물이 쥐지 못하면 턱 충돌 형상을 URDF 메쉬에 맞추는 작업(M3.5)을 따로 판단한다.
  - **대조**: 새 드라이버와 새 `ArmService`로 다시 connect한 뒤(물체가 처음 자리로 돌아온다) `height_mm=39`로 같은 호출을 하면 `ok`이고 `held`가 `box58`, `tilt_deg` < 10이다(같은 시점에 잰다). 이때 계획은 G 90, t 약 27mm(닿는 깊이 약 12mm)다. 실패하면 모델 접촉값 문제로 보고 기준은 바꾸지 않는다.
- (b) **집어 옮기기**(M3, 시나리오 `carry30`·`carry45`·`carry58`을 하나씩, 시나리오마다 새로 connect한다. 시험은 셋을 parametrize한다. 상자 이름은 `box`):
  - `svc.pick_and_place(obj)`(`height_mm=40`, 폭·길이는 시나리오와 같게) 뒤 `svc.place(at=(420, 170))`. 로봇 원점에서 약 170mm라 대칭 V자 범위(11.3.4, 약 200mm) 안이다. 배출 구역 칸(11.3.5)은 여기서 쓰지 않는다.
  - 시험 스레드가 `sim_state()`를 50Hz로 읽는다.
  - 미끄러짐: `held`가 `box`이고 그 `z_mm` > 5인 표본마다, `tool` 자세로 상자 바닥 가운데를 site 좌표계로 바꾼다. 그 값이 이 조건을 처음 만족한 표본의 값에서 3.5mm 이하다(2026-11-13: 3 → 3.5mm. M3 실측에서 carry45가 실행마다 2.95~3.03mm로 경계에 걸렸고, carry30·58은 3mm 안).
  - 놓기: `svc.place`가 끝나고 1.0초 뒤 잰다. 상자 바닥 가운데와 (420, 170)의 **수평 거리가 7mm 안**이고, `tilt_deg` < 10이다(2026-11-13: 5mm → 7mm. M3 실측 carry30·45·58 = 4.4·6.2·3.3mm. 실물 손끝 위치 정밀도 ±9mm(`measurements.md` 11-07)보다 작아 시뮬레이션 기준으로 충분하다).
  - 감시: 실행 동안 `events.jsonl`에 `ARM-GRIP-OVERLOAD-WARN`·`ARM-GRIP-OVERLOAD`가 0건이고, reason이 `overload`·`overcurrent`·`overheat`로 시작하는 `ARM-FAULT`가 0건이다. `ARM-LOAD-PROBE`는 세지 않고, 건수와 최대 `load_pct`만 시험 출력에 남긴다(M3 실측: 1~2건, 최대 100%는 순간값). 목표 선형 보간(13.2) 뒤에도 감시가 걸리면 서보 kv를 낮추는 안을 따로 올린다.
- (c) **기본값 유지**(M2): `--sim-engine`을 주지 않으면 지금 `SimDriver`이고, 지금 시험이 모두 그대로 통과한다. `--sim` 없이 `--sim-engine`만 주면 "`--sim`과 함께 쓴다"로 끝난다.
- (d) **모델 대조**(M1, `mujoco` 모듈과 `arm/mujoco_model.py`만 쓴다. 드라이버 없이):
  - `np.random.default_rng(0)`으로 그리퍼를 뺀 다섯 관절을 `joint_limits_deg`에서 5° 안쪽으로 고른 자세 10개를 만든다.
  - 각 자세를 `JOINT_MAP`으로 `qpos`에 넣고 `mj_kinematics`로 `gripper_frame` site 위치를 구한다(동역학·중력 없음). `model_to_table_mm`로 바꾼 값과 우리 `kin.fk`의 위치가 2mm 안이다.
  - 받침 충돌 geom이 있다(원본 것이든 더한 상자든).
- (e) **드라이버 동작과 속도**(M2):
  - 그리퍼 값 0·50·100을 `write_joints`로 주고 1초 뒤 읽은 값이 각각 2 안이다(물리 포함, 중력 그대로).
  - `connect` 직후 `read_joints`가 home과 1° 안이다.
  - shoulder_lift −30°·elbow_flex 60° 자세(중력이 걸리는 자세. home은 균형 자세라 처지지 않는다)에서 토크를 끄고 1초 뒤 shoulder_lift가 5° 넘게 처지고, 다시 켜면 그 자리를 지킨다(#275).
  - 토크가 꺼진 동안 `write_joints`로 다른 목표를 주어도 움직이지 않고, 켜면 그 순간 자세를 지킨다.
  - `realtime=False`에서 `advance` 없이는 시간이 흐르지 않는다.
  - 물리 실패를 일부러 일으키면(시험에서 qpos에 NaN) `SIM-PHYSICS-FAIL`이 `event_sink`로 가고 `read_joints`가 `RuntimeError`다.
  - `home`·`radar_clear` 두 자세에서 팔 body와 받침 상자(`base_box`)의 접촉이 없다. base는 월드에 고정된 body라 MuJoCo가 부모-자식 접촉을 자동으로 거르지 않으므로, `load_model`이 `exclude(base, shoulder)`를 더한다(받침 상자가 어깨 모터와 늘 닿기 때문, #275).
  - **실시간 속도**(손으로, 개발 노트북): 실시간 모드로 60초 동안(쉬기 30초 + `home`↔`radar_clear` 속도 30% 왕복 30초. `home`과 `park`는 지금 같은 자세라 쓰지 않음) 연결 직후 버림(13.3)을 뺀 `dropped_count`가 0이고 `step_ms_avg`가 1ms 안이다. pytest `slow` 시험 `test_realtime_60s`로 돌리고, 결과와 CPU 이름을 `measurements.md`에 남긴다.
  - **시험 이름**(`tests/test_mujoco_driver.py`): `test_attributes_and_shapes`(13.2 속성·7키·8키·`read_power`), `test_connect_home`, `test_torque_off_sags_and_holds`, `test_write_while_torque_off`, `test_gripper_0_50_100`, `test_not_realtime_needs_advance`, `test_physics_fail_event`, `test_base_box_no_contact`, `test_ranges_widened_and_default_warning`, `test_pending_events_flush_when_sink_is_set`, `test_cli_engine_requires_sim`, `test_cli_dry_run_rejected`, `test_cli_gui_rejected`(M4에서 `test_cli_gui_passes_engine`으로 바뀜), `test_cli_import_error_exit2`, `test_cli_status_mujoco`, `test_realtime_60s`(slow).
- (f) **보기**(M4):
  - 자동 시험(팔, `tests/test_mujoco_driver.py`):
    - `test_viewer_flag_passed`: `make_sim_driver(..., viewer=True)`의 드라이버 `viewer`가 참. `launch_passive`는 monkeypatch한다.
    - `test_viewer_requires_realtime`: `realtime=False`와 함께면 ValueError.
    - `test_cli_viewer_requires_engine`: 종료 코드 1.
    - `test_cli_gui_passes_engine`: 위 "명령 처리".
    - `test_sim_state_size`: `size_mm`가 시나리오 값과 같다. 시나리오 없이(`config/manual_objects.yaml`) connect하고 1초 뒤 물체마다 `sim_state()` 위치가 파일 값과 3mm 안이고 `tilt_deg` < 5다(기본 물체가 서로 밀어내지 않는다).
    - `test_failed_reason`: 정상이면 None이다. `qpos`에 NaN을 넣고 `advance` 뒤에는 "관절값이 NaN"이다.
    - `test_objects_overlap_rejected`: 바닥 사각형이 겹치는 시나리오는 `make_sim_driver`가 ValueError다.
  - 자동 시험(GUI): `gui-spec.md` 12.8.
  - 손 확인(`measurements.md`에 기록): `so101.cmd --sim --sim-engine mujoco --sim-viewer gui`에서 직접 조작 집어 옮기기 1회 동안 뷰어 창과 GUI 3D에서 같은 물체가 함께 움직인다.

### 13.8 설치 (M1 #260에서 확인)
- `mujoco` 4.0.0은 실제 내용이 `mujoco-core`·`mujoco-studio`에 있는 메타 휠이다. 그냥 `pip install`하면 pip가 numpy를 바꿀 수 있다. 그래서 **`--no-deps`로 필요한 것만** 설치한다:
  ```
  pip install --no-deps "mujoco>=4.0,<5" mujoco-core mujoco-studio glfw pyopengl
  ```
  M2의 첫 작업으로, #260에서 설치한 다섯 패키지의 버전을 `pip freeze`로 확인해 `==`로 고정한 줄을 `docs/guides/tooling-setup.md` 설치 목록에 둔다. `pyproject.toml`의 `mujoco>=4.0,<5`는 그대로 둔다.
- 설치한 뒤 `python -c "import numpy; print(numpy.__version__)"`가 2.2.6(conda 설치본)이어야 한다(#260에서 확인).
- `pyproject.toml`의 선택 의존성 `[mujoco]`는 목록 표시용으로 남긴다. 설치 안내에는 `pip install -e .[mujoco]`를 쓰지 않는다(numpy를 바꿀 수 있음).
- `mujoco`를 import하지 못하면: CLI는 "MuJoCo가 설치되지 않았습니다. docs/guides/tooling-setup.md의 MuJoCo 줄로 설치하세요"(`MUJOCO_MISSING_TEXT`)를 출력하고 종료 코드 2로 끝난다. GUI(M4)는 화면을 띄우기 전에 같은 문장을 터미널에 출력하고 종료 코드 2로 끝난다(`gui-spec.md` 12.8).

### 13.9 확인 결과 (M1 #260, M4 뷰어)
- 관절 이름: menagerie와 같다(`JOINT_MAP` 모두 부호 +1·오프셋 0).
- 받침 충돌 geom: 없다 → `base_box`를 더함.
- `gripper_frame`: 원본 site 위치가 달라 URDF origin으로 따로 더함.
- 라이선스: Apache-2.0.
- Windows 휠: 메타 휠이라 13.8처럼 설치하고, numpy 2.2.6이 유지됨을 확인.
- 관절 range: 원본이 우리 한계보다 좁다. `load_model(joint_limits=)`가 range·ctrlrange를 `physical_limits_deg`로 넓힌다(M2, 그리퍼는 각도로 [v0−1°, v100+1°]).
- 버전 고정(M2): `mujoco==4.0.0 mujoco-core==4.0.0 mujoco-studio==4.0.0 glfw==2.11.0 pyopengl==3.1.10`(`tooling-setup.md` C24).
- 모델 손질(M2, `load_model`): 받침 `base_box`와 어깨 사이 충돌 제외(`exclude(base, shoulder)`), 집게 geom은 priority==1만(`collision_gripper`·`collision_gripper_mesh`)을 condim 4·마찰 (1.0, 0.02, 0.001)로, 전역 옵션은 elliptic cone·impratio 10·implicitfast·timestep 0.002. 링크에 붙는 장면 부품은 density 0.
- 물리 스레드(M2): 벽시계를 따라 2 ms씩 진행하고 한 번에 따라잡는 양은 100 ms로 제한(`dropped_count`·`dropped_s`). i5-1240P에서 60초 동안 놓친 것 0, 스텝 평균 0.067 ms(`measurements.md`). 시험은 `realtime=False`와 `advance(초)`. 연결 직후 ArmService가 GIL을 오래 잡아 놓침이 2번 났으므로 놓침은 연결 뒤부터 센다.
- 목표 보간(13.2에 반영): ctrl을 새 목표로 바로 바꾸면 kp 998이 힘을 포화시켜 `ARM-GRIP-OVERLOAD-WARN`이 떴다. 이전 목표→새 목표를 `1/control_hz` 동안 직선으로 옮긴다. 부하 정의는 그대로.
- 물리 실패: 관절값 NaN은 스텝 전에 검사한다(MuJoCo가 NaN이면 data를 리셋해 사유가 사라짐). `failed_reason`, 이후 읽기·쓰기는 RuntimeError.
- 막힘 문턱(결정 0059·0061, #325): 로봇 설정 `sim_blocked`(`blocked_empty/held_accel/cruise_deg` 4개)는 MuJoCo일 때 판정 시점의 `ctl.blocked_cfg`(`_SimBlocked`)에만 덮는다. cfg·프리셋 값은 그대로. 모르는 키는 ValueError. 알려진 한계: GUI에서 이 4개 `blocked_*`를 고쳐도 MuJoCo 판정에는 안 먹는다.
- 뷰어(M4): `mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)`가 있고 Windows의 메인 아닌 스레드에서 열린다. 1/30초마다 sync하고 `xfrc_applied`는 0으로 둔다. `--sim-viewer`는 실시간 모드에서만.
- 시나리오(M3): `config/sim_scenarios/*.yaml`(이름 또는 .yaml 경로, 저장소 루트 기준), 수동 물체는 `config/manual_objects.yaml`(obj-1…). 모르는 키·이름 중복·바닥 사각형 겹침(분리축 검사)은 ValueError. 병렬 그리퍼는 ValueError.
- 재현 시험 기준: topedge는 윗면 접촉(z 34~44 mm)만 본다, 들고 가기 슬립 ≤3.5 mm·착지 <7 mm·기울기 <10°(`tests/test_mujoco_scenarios.py`).
- 함정: 토크 끈 처짐 시험은 홈 자세에서 균형이라 처지지 않는다(어깨 −30°·팔꿈치 60°로). `conda` PATH 없이 ruff·python을 부르면 MKL DLL 오류.

### 13.10 시험장으로 키우기 (M5~M9, 2026-11-14 확정 — 결정 0062)
사용자 요청(2026-11-14): MuJoCo를 실물 없이 반복하는 시험장으로 키운다. 당장은 **우리 알고리즘을 반복해서 재는 것**이 목적이고, 나중에는 같은 시험장을 **강화학습**에도 쓴다. 우리 알고리즘으로 안 되는 경우는 강화학습으로 밀어붙인다. 출력·구입 전 부품(브래킷 등)은 STEP/STL로 먼저 평가한다. 사용자가 승인했다(결정 0062). 단계마다 세부 설계를 이 절 아래에 펼치고 EGM Stage 9를 거쳐 REQ로 낸다.

**구조: 공통 층 하나 + 실행기 둘** (결정 0037 "주제마다 도구 하나"를 지키면서 강화학습으로 넓힐 수 있게)
- **공통 층**(시험장 본체, 도구 하나):
  - 세계 짓기: 장면 부품, CAD 부품(질량·관성 포함), 물체 묶음(격자 또는 씨앗 고정 무작위)을 MjSpec으로 짓는다.
  - 센서: MuJoCo 광선(`mj_multiRay`)으로 초음파 원뿔을 흉내 낸다. 센서 자리는 CAD 부품의 site에서 읽는다.
  - 판정: 성공과 실패 이유(`no_object`, `height_required`, `ik_fail`, 미끄러짐, 쓰러짐, `blocked`, 간섭).
  - 기록: 결과 표(JSON Lines·CSV), 실패만 영상(오프스크린 렌더러).
  - 겉모양은 Gymnasium 꼴(`reset(seed, options)` → 관측, `step`, `info`)로 둔다.
- **실행기 A (지금)**: 우리 알고리즘. `ArmService`·`Skills`를 `MuJoCoDriver` 위에서 그대로 돌린다. 한 시도 = 한 에피소드다. 알고리즘 비교(집기 기울기, 놓기, 막힘 감지, 순서 정책)는 이 실행기의 옵션이다.
- **실행기 B (나중, 강화학습)**: 같은 공통 층의 Gymnasium 환경을 시뮬레이션 시간으로 빠르게 돌린다. 행동은 관절 또는 손끝 변화량이다. 학습 틀은 새로 만들지 않고 가져온다(후보 so101-nexus·mjlab, 평가 뒤 정함).
- 초음파 성능은 지금처럼 초음파 벤치마크(`ultrasonic-spec.md` 8.5)가 맡는다. 벤치마크에 MuJoCo 엔진 옵션을 더하는 것이지 두 번째 도구가 아니다. 팔 도달 범위도 지금 도달 계산 그대로다.

| 단계 | 내용 | 사용자 요청 | 근거 레퍼런스 | 우선 |
|---|---|---|---|---|
| M7a 부품 사전 평가 ✅ #335(11-14) | 받은 STL(로보나인 + 브래킷 + HC-SR04)을 그리퍼 링크에 맞춰 장면 부품으로 올린다(3D에 보임). 질량·무게중심·관성(밀도 × 출력 채움 보정, 또는 슬라이서가 낸 무게), 손목·어깨 정적 토크 여유, 관절 끝 자세 간섭, 빔 원뿔 안 간섭, 출력 오버행을 한 보고서로 낸다. 센서 오프셋을 CAD에서 바로 구한다 | ④⑥ | trimesh 질량 특성, coal(지금 충돌 검사), MjSpec | 끝 (브래킷+센서 15.5g, 손목 끝 자세 36개 간섭 없음, 오프셋 {0.0, 66.5, 99.5}) |
| M4b 로보나인 손 모델(A, 결정 0063) ✅ #348·#350·#357(11-14, FK 0.002mm, carry 30/45/58 통과, kp·힘 잠정. 남은 것: 병렬 손으로 (a)(b)·`sim_blocked`·부하 문턱 다시 재기) | menagerie 팔(어깨~손목)은 그대로 두고 단동식 턱만 지운 뒤 로보나인 클램프 둘을 붙인다. ① 병렬 URDF `gripper_link`와 menagerie gripper body 좌표가 같은지 FK로 먼저 대조한다. ② 관절은 슬라이드 둘(0~42mm, 한쪽은 반대 방향)을 equality 또는 tendon으로 묶고 액추에이터는 하나다(MuJoCo는 URDF mimic을 무시한다). 접촉 때 한쪽만 밀리지 않게 solref를 맞춘다. ③ 그리퍼 단위가 힌지(°)에서 슬라이드(m)로 바뀌므로 서보값↔폭, kp·forcerange(N), 부하 % 정의, `Torque_Limit` 배율, 속도를 새로 정한다. 실물 조임 힘 측정(runbook 2-10)이 필요하다. ④ 충돌: 클램프·손의 볼록 껍질이 실제보다 4~6.5배 커서 시각은 원본 메쉬, 충돌은 상자 근사나 CoACD다. 클램프 안쪽에 얇은 패드 geom을 두고(치수는 CAD 확인), `jaw_*` 역할 이름과 `held` 판정·시험을 함께 고친다. ⑤ `gripper_frame` site는 병렬 값(z −121.8mm)으로 나눈다. ⑥ M7a의 브래킷 질량·`wrist_sonar` site(#336)를 같이 싣고, 로보나인 + 브래킷 + 센서 조립을 `--sim-viewer`로 띄워 사용자가 확인한다. 13.7 (a)(b)·`sim_blocked`·부하 문턱은 로보나인 손으로 다시 잰다 | 0063 | MjSpec, obj2mjcf/CoACD | **1 (M5·M6 앞)** |
| M5 집기 시험장 | 공통 층 + 실행기 A(로보나인 손 위, M4b 뒤). 크기(폭·길이·높이)·자리(반경·방위)·각도를 바꿔 집기·놓기를 반복하고 성공률·실패 이유 표. U6(`grasp_tilt_tol_deg`) 같은 결정의 근거를 낸다 | ①② | robosuite 배치 초기화기, ManiSkill SO-100 무작위화, so101-nexus | **1** |
| M6 초음파를 MuJoCo에 | (M4b 뒤) 레이더와 손목 센서(M7a의 CAD site, `wrist_sonar`)를 광선으로 흉내 내는 가상 ESP32. 찾기 → CHECK → 집기 → 놓기 전체를 물리로 돈다. 새 센서 높이(약 155mm)에 맞춰 확인 높이·경로 값을 벤치마크로 다시 고른다(알고리즘을 브래킷에 맞추기) | ⑤ | MuJoCo rangefinder·`mj_multiRay` | **1** |
| M5b 빠른 모드 | 제어기를 시뮬레이션 시간으로 돌려 벽시계보다 빠르게 한다. 대량 시험과 강화학습의 전제다. 첫 버전은 프로세스 여러 개를 나란히 돌린다 | ①, RL | so101-nexus(MuJoCo 병렬) | 2 |
| M5c 서보 맞추기(시스템 식별) | 실물 `status.jsonl`(명령·실제)을 다시 틀어 kp·감쇠·armature·마찰·힘 한계를 맞춘다. 시뮬레이션 성공률을 믿고 강화학습 결과를 실물로 옮기기 위한 전제다 | RL | ALOHA 2(MuJoCo 맞춤), BAM(STS3215 모델) | 2 |
| M8 GUI 시험장 | GUI에서 시나리오를 고르고 물체를 바꾸고(세계 다시 짓기) M5 결과 표·실패 영상을 본다 | ③ | mjviser | 3 |
| M9 강화학습(12/4 뒤) | 실행기 B. 우리 알고리즘이 구조상 못 하는 경우(예: 45° 정사각형 — 밀거나 돌린 뒤 집기, 초음파만 보고 다가가 집기)를 학습한다. 실물로는 시뮬레이션 체크포인트에서 시작해 실물에서 이어 배운다(HIL-SERL) | RL | so101-nexus, mjlab, LeRobot HIL-SERL | 4 |

**강화학습으로 넘어가는 문턱(제안)**: M5에서 어떤 물체 묶음의 성공률이 목표(예: 80%)에 못 미치고, 알고리즘 옵션을 바꿔도 오르지 않으며, 원인이 구조 한계(5축, 턱 모양)로 확인되면 그 묶음을 M9 후보로 올린다. 시뮬레이션만으로 학습한 정책을 실물에 바로 쓰면 SO-101 사례에서 실패가 많았다(집기는 실물 추가 학습이 필요했다). 그래서 M5c와 실물 이어 배우기를 함께 계획한다.

**정할 것과 위험**
- **실시간 제약**: 지금 `ArmService` 위 MuJoCo는 벽시계 실시간이라 집어 옮기기 한 번이 약 20초다(100회 = 약 33분). M5 첫 버전은 병렬 프로세스, 그 다음이 M5b다.
- **메쉬 충돌**: MuJoCo는 메쉬 충돌을 볼록 껍질로 한다. 오목한 브래킷은 CoACD(obj2mjcf `--decompose`)로 나누거나 상자 몇 개로 근사한다. 우리 충돌 검사(coal)는 메쉬를 그대로 쓴다.
- **질량**: 출력물은 채움 비율 때문에 메쉬 부피 × 재료 밀도보다 가볍다. 슬라이서가 낸 무게를 받으면 그것을, 없으면 채움 보정 값을 쓴다. 지금 링크 부품은 질량 0이다(13.5). M7a부터 실제 질량을 넣는다.
- **실물과의 차이**: 성공률 숫자는 "시뮬레이션 기준"으로만 쓰고, 실물 결과가 있으면 실물을 따른다(0037).
- **PC**: 지금 단계(M4b·M5·M6)는 노트북(CPU)으로 된다. M5b 대량 시험(수백 회)은 코어가 많은 데스크탑(RTX 5070 Ti PC)에서 병렬 프로세스로 돌린다. M9 강화학습의 GPU 병렬(MJX·MuJoCo Warp)은 NVIDIA GPU가 필요하고, JAX GPU는 리눅스(Windows는 WSL2)에서 돈다. 그래서 데스크탑이 필요한 시점은 M5b 대량 시험과 M9다.
- **마감**: 12/4까지는 M7a(끝) → M4b(끝) → M5·M6 → (여유가 있으면) M5b·M5c 순서다(결정 0063). 11/30부터 새 기능은 멈춘다. M8은 여유가 있을 때, M9는 12/4 뒤다.
