# MuJoCo 물리 시뮬레이션·시험장 설계서

상태: 확정 · 2026-11-14 (`arm-software-spec.md` 13절에서 옮김. 13.1~13.9는 M1~M4로 구현됨, 13.10 시험장 계획은 결정 0062. 2026-11-14 drift-report(main fcdcb0f, #348~#357) 반영: 13.9 병렬·뷰어, 13.10 M5 판정·시험장 기준 설정. 2026-11-15 총괄 밤 보고 반영: 시나리오는 그리퍼별, M5 설계 입력, M5b 시계 주입 계획. 2026-11-16 M5·M5b 첫 단계·M6 세부 설계 13.10.1~13.10.3(초안, EGM Stage 9 전)) · 2026-11-16 13.10.4 카메라·렌더·에피소드 기록(결정 0066, 초안. 2026-11-16 밤 Stage 9 (가)(나) 꼭 고칠 것 반영: 13.10.1·13.10.2·13.10.4)

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
  - 뷰어는 물리 `data`가 아니라 복사본(`_vdata`)을 본다(#354). 물리 스레드는 1/15초(`VIEWER_SYNC_S`)마다 `mj_copyData`로 복사만 하고, 별도 스레드 `mujoco-viewer-sync`가 잠금 밖에서 `handle.sync()`를 부른다. 뷰어의 마우스 힘은 복사본에만 들어가 물리에 닿지 않는다(`xfrc_applied`를 0으로 만드는 코드는 없다).
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
  - **시나리오는 그리퍼별이다**(2026-11-15). 손 모양이 달라 같은 배치가 다른 뜻이 된다(예: `blocker`의 벽 (300,200) 높이 120은 병렬 손이 home → `radar_clear`로 갈 때 쓸고 가는 자리(x 272~333, y 187 이상, 높이 86mm 이상)에 있어 시험하려던 동작 전에 막힌다). 이름 규칙: 접미사 없는 파일은 단동식 기준, 병렬은 `<이름>_parallel.yaml`(#370 `blocker_parallel`). 두 손에서 뜻이 같은 것(`carry30`·`carry45`·`carry58`)은 한 파일을 같이 쓴다. 시험과 시험장 결과 표에는 시나리오 이름과 그리퍼 종류를 함께 남긴다. 실물에도 같은 자리가 있다: 병렬 손에서는 높이 86mm가 넘는 물체가 x 270~335·y 187 이상에 있으면 `radar_clear`로 가는 길에 걸린다(시뮬레이션 충돌 형상 기준. 시연 물체는 40mm 이하라 해당 없다).
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
  - `held`: 두 턱 역할(단동식 `jaw_fixed`·`jaw_moving`, 병렬 `clamp_right`·`clamp_left`. 병렬 손 본체 상자는 `arm`) 둘 다와 법선 힘 > 0으로 접촉한 물체의 이름(접두어 없이) 또는 None. 둘 이상이면 `force_n` 합이 큰 것.
  - M2에서는 `objects`·`contacts`가 빈 값, `tool`이 None, `held`가 None이었다.
- **`no_object`를 성공으로 치는 규칙**: 기구학 엔진(`SimDriver`, `--sim-engine kinematic`)은 쥘 물체가 없어서 `no_object`를 성공으로 친다. 드라이버의 `physical_objects`가 참이면 이 규칙을 쓰지 않는다.
  - CLI 작업 실행: `--dry-run`(→ `TaskRunner(accept_no_object=True)`)을 MuJoCo와 함께 주면 "물리 시뮬레이션에서는 --dry-run을 쓰지 않는다"로 거부한다(M2). `Studio`·`TaskRunner`는 고치지 않는다. `test_run_modes.py`·`test_task.py`는 기본 엔진이라 영향이 없다.
  - GUI(M4): `gui-spec.md` 12.8의 "`no_object` 규칙"을 따른다. 판정은 `getattr(driver, "physical_objects", False)`로 한다(`SimDriver`에는 이 속성이 없다).

### 13.7 완료 기준
- **(a)(b) 공통**: cfg는 `config/robot.yaml` + `config/arms/so101_follower_01.yaml`(local.yaml 없이)이다. 지금 시험은 `load_robot`이 `local.yaml`과 팔 고르기를 겹쳐 PC마다 결과가 달라질 수 있어 팔 REQ로 고친다. `driver = make_sim_driver(cfg, ws, None, scenario=이름)`, `svc = ArmService(driver, cfg, ws, log_root=tmp_path)`, `svc.connect()`. 집기는 `svc.pick_and_place(obj)`(놓을 곳 없이 집기만), 놓기는 `svc.place(at=(x, y))`다. 둘 다 `slow`로 표시한다.
- (a) **재현**(M3, 시나리오 `topedge`. **단동식 비교 옵션 전용**, 2026-11-15 사용자 결정 U14: 윗모서리를 치는 것은 V자 턱이 위로 좁아지는 단동식의 물리라 병렬 손에서는 재현되지 않는다(면에 맞춰 내려가 `ok`·`held`). 이 시험은 그리퍼를 `single_jaw`로 명시해 돌린다. 병렬 손에서 대응하는 실패(위치가 어긋나 클램프가 물체 윗면에 내려앉음)는 M5 세부 설계의 실패 주입 항목으로 새로 정한다):
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
- 뷰어(M4, #354로 바뀜): `mujoco.viewer.launch_passive(model, 복사본 _vdata, show_left_ui=False, show_right_ui=False)`가 있고 Windows의 메인 아닌 스레드에서 열린다. 물리 스레드는 1/15초마다 `mj_copyData`만 하고 sync는 별도 스레드가 한다(13.1). 처음 방식(물리 `data`를 직접, 1/30초 sync)은 60초에 버림이 45~184번이어서 바꿨고 15Hz에서는 1~4번이다(`measurements.md`). `--sim-viewer`는 실시간 모드에서만.
- 시나리오(M3): `config/sim_scenarios/*.yaml`(이름 또는 .yaml 경로, 저장소 루트 기준), 수동 물체는 `config/manual_objects.yaml`(obj-1…). 모르는 키·이름 중복·바닥 사각형 겹침(분리축 검사)은 ValueError. 병렬 그리퍼는 M3에서는 ValueError였으나 M4b(#348)부터 거부하지 않는다: cfg의 그리퍼가 `parallel`이면 로보나인 손을 붙인다(13.2).
- 재현 시험 기준: topedge는 윗면 접촉(z 34~44 mm)만 본다, 들고 가기 슬립 ≤3.5 mm·착지 <7 mm·기울기 <10°(`tests/test_mujoco_scenarios.py`).
- 함정: 토크 끈 처짐 시험은 홈 자세에서 균형이라 처지지 않는다(어깨 −30°·팔꿈치 60°로). `conda` PATH 없이 ruff·python을 부르면 MKL DLL 오류.

### 13.10 시험장으로 키우기 (M5~M9, 2026-11-14 확정 — 결정 0062)
사용자 요청(2026-11-14): MuJoCo를 실물 없이 반복하는 시험장으로 키운다. 당장은 **우리 알고리즘을 반복해서 재는 것**이 목적이고, 나중에는 같은 시험장을 **강화학습**에도 쓴다. 우리 알고리즘으로 안 되는 경우는 강화학습으로 밀어붙인다. 출력·구입 전 부품(브래킷 등)은 STEP/STL로 먼저 평가한다. 사용자가 승인했다(결정 0062). 단계마다 세부 설계를 이 절 아래에 펼치고 EGM Stage 9를 거쳐 REQ로 낸다.

**구조: 공통 층 하나 + 실행기 둘** (결정 0037 "주제마다 도구 하나"를 지키면서 강화학습으로 넓힐 수 있게)
- **공통 층**(시험장 본체, 도구 하나):
  - 세계 짓기: 장면 부품, CAD 부품(질량·관성 포함), 물체 묶음(격자 또는 씨앗 고정 무작위)을 MjSpec으로 짓는다.
  - 센서: MuJoCo 광선(`mj_multiRay`)으로 초음파 원뿔을 흉내 낸다. 센서 자리는 CAD 부품의 site에서 읽는다.
  - 판정: 성공과 실패 이유(`no_object`, `ik_fail`, 미끄러짐, 쓰러짐, `blocked`, 간섭, `drop_zone_full`(쥔 물체가 겹치지 않고 들어갈 자리가 없음, #356)). 경고 `ARM-CARRY-LOW`(#352)와 `ARM-OPENING-PATH-FALLBACK`은 건수로 센다. `height_required`는 단동식 손에서만 나오므로(결정 0052·0063) 기준인 로보나인 손의 판정에는 없고, 단동식 비교 옵션에서만 센다.
  - 기록: 결과 표(JSON Lines·CSV), 실패만 영상(오프스크린 렌더러).
  - 겉모양은 Gymnasium 꼴(`reset(seed, options)` → 관측, `step`, `info`)로 둔다.
- **실행기 A (지금)**: 우리 알고리즘. `ArmService`·`Skills`를 `MuJoCoDriver` 위에서 그대로 돌린다. 한 시도 = 한 에피소드이고, 에피소드마다 드라이버와 `ArmService`를 새로 만든다(13.7처럼. 같은 `ArmService`로 다시 connect하면 `Skills`의 쥔 물체·놓은 자리·남은 물체 기억이 넘어가 에피소드가 섞인다). 알고리즘 비교(놓기, 막힘 감지, 순서 정책, 집는 높이·벌림 여유)는 이 실행기의 옵션이다. 집기 기울기는 단동식에만 있는 규칙이라 단동식 비교 옵션에서만 본다(결정 0063).
- **실행기 B (나중, 강화학습)**: 같은 공통 층의 Gymnasium 환경을 시뮬레이션 시간으로 빠르게 돌린다. 행동은 관절 또는 손끝 변화량이다. 학습 틀은 새로 만들지 않고 가져온다(후보 so101-nexus·mjlab, 평가 뒤 정함).
- 초음파 성능은 지금처럼 초음파 벤치마크(`ultrasonic-spec.md` 8.5)가 맡는다. 벤치마크에 MuJoCo 엔진 옵션을 더하는 것이지 두 번째 도구가 아니다. 팔 도달 범위도 지금 도달 계산 그대로다.

**M5 세부 설계 입력**(2026-11-15 MuJoCo 세션 메모. 13.10.1~13.10.3에 반영했고, Stage 9가 끝나면 이 문단을 지운다): ① 지금 시험은 덮어쓰기를 임시 `local.yaml`(`gripper: parallel` 한 줄, `load_robot(local_path=)`)로 한다. 팔 파일이 둘이 되면 팔도 지정해야 한다. ② 손목 부품은 임시 `scene.yaml`에 절대경로 `file`로 적어 `load_scene` → `make_sim_driver(cfg, ws, scene, scenario=)`로 넘긴다(상대경로는 yaml의 부모의 부모 기준). `ArmService`는 init에서 스스로 `load_scene()`(`config/scene.yaml`)을 읽어 계획 쪽 충돌 검사(`collision.set_scene`)에 쓰므로, 드라이버에만 넘긴 장면과는 별개다(#375 확인: 병렬 시험에서 드라이버 장면에는 손목 부품 둘이 있지만 `svc.scene`은 `radar_station`뿐이고 붙은 부품 목록은 비어 있다). 즉 지금 시험은 MuJoCo 물리에는 손목 부품이 실리지만 계획 쪽 충돌 검사는 보지 않는다. 시험장 기준 설정은 두 쪽이 같은 Scene을 보게 해야 한다(`svc.scene = sc; svc.collision.set_scene(sc, ws)`처럼. CLI `--sim-engine mujoco`는 `load_scene()`을 둘 다에 써서 같다). 맞추는 방법(ArmService에 scene 인자를 더할지 포함)은 M5 세부 설계에서 정한다. ③ 손목 부품 둘은 한 쌍으로만 온다(하나면 ValueError, 단동식이면 `SIM-PART-SKIPPED`). ④ 시험 하나가 연결부터 새로 해 실시간 20~27초다. ⑤ 병렬 carry 58mm의 미끄러짐이 실행마다 0.8~10mm로 갈린다(30·45mm는 0.7~1.4mm). 실시간 지터인지 물리 편차인지 못 갈랐고, 그래서 미끄러짐은 `measure` 시험(임시 4.0mm)으로 분리했다(#370). 미끄러짐 기준과 조임 힘(kp·forcerange)은 실물 2-10 뒤에 정한다(U15). 결정 스텝(M5b 첫 단계)으로 돌리면 지터인지 갈린다.

**로보나인 손 사양(저장소 원문 `docs/specifications.md`, 2026-11-16 기획 확인)**: 스트로크 84mm, 최대 조임 120N, 최대 속도 14mm/s, 반복 0.5mm, 조립 무게 170g(PLA 채움 30%), 랙·피니언 + 지름 6mm 봉 안내. 13.2의 잠정값(힘 한계 10N, kp 10000N/m)은 최대 조임 120N의 1/12이다. 우리는 `Torque_Limit`으로 조임을 30%(닫기)·20%(들기)로 낮춰 쓰므로 그 비율이면 약 36N·24N이 된다. 힘 한계·속도 한계(14mm/s)·손 질량(170g)을 이 값으로 맞출지는 실물 2-10의 조임·닫는 시간 실측과 함께 U15에서 정한다(그 전에는 잠정값을 바꾸지 않는다). 저장소의 MuJoCo 안내는 ROS2 `mujoco_ros2_control`로 URDF를 그대로 올리는 방식이라 가져올 액추에이터 값이 없다. 손목에 다는 방향(벌림 축이 가로인지 세로인지)의 기본값은 원문에 없다: 플랜지 구멍 넷으로 90°씩 돌려 달 수 있으므로 우리 병렬 URDF의 방향을 기준으로 하고 조립 때 맞춘다(runbook 2-10).

**시험장 기준 설정**(2026-11-14, drift-report 4번): 코드 기본값은 로보나인 교체(U5) 전까지 단동식이고(`gripper` 기본 `single_jaw`), 손목 브래킷·센서 부품은 `config/scene.yaml`에서 주석이며, `wrist_sonar` site는 병렬일 때만 생긴다. 그래서 M5·M6는 13.7 공통 cfg를 그대로 쓰지 않고 아래 기준 설정으로 돈다.
- cfg = `config/robot.yaml` + `config/arms/so101_follower_01.yaml` + **시험장 덮어쓰기(`gripper: parallel`)**. PC의 `config/local.yaml`은 읽지 않는다(PC마다 결과가 달라지지 않게). 지금 `tests/test_mujoco_parallel.py`가 임시 `local.yaml`로 하는 것과 같은 방식이다.
- 장면 = `config/scene.yaml`의 부품 + **손목 부품 둘(`wrist_hcsr04`, `wrist_bracket`)을 켠 것**. 브래킷 질량 15.5g과 `wrist_sonar`가 실린다.
- 덮어쓰기는 레포에 추적되는 파일 하나로 두고(MuJoCo 세션 영역), 결과 표에 그리퍼 종류·켠 부품·커밋을 함께 남긴다. 파일 자리와 형식은 M5 세부 설계에서 정한다.
- 단동식은 비교 옵션이다(덮어쓰기를 `single_jaw`로, 손목 부품 없이).
- U5 뒤 팔 파일 기본값이 `parallel`이 되고 `scene.yaml` 주석을 풀면 덮어쓰기는 비게 된다. 그때 13.7 (a)(b)의 단동식 시험은 그리퍼를 `single_jaw`로 명시하게 고친다(기본값에 기대지 않게).

| 단계 | 내용 | 사용자 요청 | 근거 레퍼런스 | 우선 |
|---|---|---|---|---|
| M7a 부품 사전 평가 ✅ #335(11-14) | 받은 STL(로보나인 + 브래킷 + HC-SR04)을 그리퍼 링크에 맞춰 장면 부품으로 올린다(3D에 보임). 질량·무게중심·관성(밀도 × 출력 채움 보정, 또는 슬라이서가 낸 무게), 손목·어깨 정적 토크 여유, 관절 끝 자세 간섭, 빔 원뿔 안 간섭, 출력 오버행을 한 보고서로 낸다. 센서 오프셋을 CAD에서 바로 구한다 | ④⑥ | trimesh 질량 특성, coal(지금 충돌 검사), MjSpec | 끝 (브래킷+센서 15.5g, 손목 끝 자세 36개 간섭 없음, 오프셋 {0.0, 66.5, 99.5}) |
| M4b 로보나인 손 모델(A, 결정 0063) ✅ #348·#350·#357(11-14, FK 0.002mm, carry 30/45/58은 `held`·놓은 자리 7mm·기울기 10°만 통과(미끄러짐 3.5mm와 과부하·fault 0건은 아직 안 봄), kp·힘 잠정. 계획과 달라진 것: 묶기는 equality, 클램프 안쪽 패드 geom은 만들지 않음(z 조각 경계 상자), 손 메쉬는 손목 카메라 팔을 자른 파생 STL `parallel_hand_nocam`. 남은 것: 병렬 손으로 (a)(b)·`sim_blocked`·부하 문턱 다시 재기) | menagerie 팔(어깨~손목)은 그대로 두고 단동식 턱만 지운 뒤 로보나인 클램프 둘을 붙인다. ① 병렬 URDF `gripper_link`와 menagerie gripper body 좌표가 같은지 FK로 먼저 대조한다. ② 관절은 슬라이드 둘(0~42mm, 한쪽은 반대 방향)을 equality 또는 tendon으로 묶고 액추에이터는 하나다(MuJoCo는 URDF mimic을 무시한다). 접촉 때 한쪽만 밀리지 않게 solref를 맞춘다. ③ 그리퍼 단위가 힌지(°)에서 슬라이드(m)로 바뀌므로 서보값↔폭, kp·forcerange(N), 부하 % 정의, `Torque_Limit` 배율, 속도를 새로 정한다. 실물 조임 힘 측정(runbook 2-10)이 필요하다. ④ 충돌: 클램프·손의 볼록 껍질이 실제보다 4~6.5배 커서 시각은 원본 메쉬, 충돌은 상자 근사나 CoACD다. 클램프 안쪽에 얇은 패드 geom을 두고(치수는 CAD 확인), `jaw_*` 역할 이름과 `held` 판정·시험을 함께 고친다. ⑤ `gripper_frame` site는 병렬 값(z −121.8mm)으로 나눈다. ⑥ M7a의 브래킷 질량·`wrist_sonar` site(#336)를 같이 싣고, 로보나인 + 브래킷 + 센서 조립을 `--sim-viewer`로 띄워 사용자가 확인한다. 13.7 (a)(b)·`sim_blocked`·부하 문턱은 로보나인 손으로 다시 잰다 | 0063 | MjSpec, obj2mjcf/CoACD | **1 (M5·M6 앞)** |
| M5 집기 시험장 | 공통 층 + 실행기 A(로보나인 손 위, M4b 뒤). 크기(폭·길이·높이)·자리(반경·방위)·각도를 바꿔 집기·놓기를 반복하고 성공률·실패 이유 표. 로보나인 손의 결정 근거를 낸다: 집을 수 있는 폭·높이 범위(W ≤ G − 4, 스트로크 84mm), 집는 높이와 벌림 여유, 여러 물체를 배출 구역에 놓을 때의 겹침·폴백. 단동식 전용 규칙(`grasp_tilt_tol_deg`, 45° 정사각형, `height_required`)은 기준에서 빼고 단동식 비교 옵션으로만 둔다(결정 0063, U6은 닫음) | ①② | robosuite 배치 초기화기, ManiSkill SO-100 무작위화, so101-nexus | **1** |
| M6 초음파를 MuJoCo에 | (M4b 뒤) 레이더와 손목 센서(M7a의 CAD site, `wrist_sonar`)를 광선으로 흉내 내는 가상 ESP32. 찾기 → CHECK → 집기 → 놓기 전체를 물리로 돈다. 새 센서 높이(약 155mm)에 맞춰 확인 높이·경로 값을 벤치마크로 다시 고른다(알고리즘을 브래킷에 맞추기) | ⑤ | MuJoCo rangefinder·`mj_multiRay` | **1** |
| M5b 빠른 모드 | 제어기를 시뮬레이션 시간으로 돌려 벽시계보다 빠르게 한다. 대량 시험과 강화학습의 전제다. 첫 버전은 프로세스 여러 개를 나란히 돌린다. **첫 단계 = 시계 주입**(2026-11-15 방향만 정함, 세부는 M5 세부 설계에서 M5와 한 묶음으로 쓰고 EGM Stage 9 뒤 REQ): `MotionController`·`Skills`·`ArmService`가 벽시계(`perf_counter`·`sleep`, 약 49곳)를 직접 부르지 않고 시계 객체(now·sleep)를 받는다. 기본은 지금과 같은 벽시계라 실제 팔·`SimDriver`·실시간 MuJoCo의 동작은 바뀌지 않는다. 시험·시험장용 `StepClock`은 sleep에서 `driver.advance`를 불러 `realtime=False` 드라이버 위에서 `ArmService`를 돌린다(13.3에 시험 전용 경로로 더한다). 조건: 안전 감시(막힘 0.2초, 부하 3초, 감시 heartbeat)의 시간 기준이 같은 시계를 쓰는지 목록으로 확인하고, 로그 시각(`time_ns`)은 벽시계 그대로 둔다. 스레드 셋을 스케줄러 하나로 모으는 완전 결정 실행은 그다음 단계다 | ①, RL | so101-nexus(MuJoCo 병렬) | 2 |
| M5c 서보 맞추기(시스템 식별) | 실물 `status.jsonl`(명령·실제)을 다시 틀어 kp·감쇠·armature·마찰·힘 한계를 맞춘다. 시뮬레이션 성공률을 믿고 강화학습 결과를 실물로 옮기기 위한 전제다 | RL | ALOHA 2(MuJoCo 맞춤), BAM(STS3215 모델) | 2 |
| M8 GUI 시험장 | GUI에서 시나리오를 고르고 물체를 바꾸고(세계 다시 짓기) M5 결과 표·실패 영상을 본다 | ③ | mjviser | 3 |
| M9 강화학습(12/4 뒤) | 실행기 B. 우리 알고리즘이 구조상 못 하는 경우(예: 45° 정사각형 — 밀거나 돌린 뒤 집기, 초음파만 보고 다가가 집기)를 학습한다. 실물로는 시뮬레이션 체크포인트에서 시작해 실물에서 이어 배운다(HIL-SERL) | RL | so101-nexus, mjlab, LeRobot HIL-SERL | 4 |

**강화학습으로 넘어가는 문턱(제안)**: M5에서 어떤 물체 묶음의 성공률이 목표(예: 80%)에 못 미치고, 알고리즘 옵션을 바꿔도 오르지 않으며, 원인이 구조 한계(5축, 턱 모양)로 확인되면 그 묶음을 M9 후보로 올린다. 시뮬레이션만으로 학습한 정책을 실물에 바로 쓰면 SO-101 사례에서 실패가 많았다(집기는 실물 추가 학습이 필요했다). 그래서 M5c와 실물 이어 배우기를 함께 계획한다.

**정할 것과 위험**
- **실시간 제약**: 지금 `ArmService` 위 MuJoCo는 벽시계 실시간이라 집어 옮기기 한 번이 약 20초다(100회 = 약 33분). M5 첫 버전은 병렬 프로세스, 그 다음이 M5b다.
- **메쉬 충돌**: MuJoCo는 메쉬 충돌을 볼록 껍질로 한다. 오목한 브래킷은 CoACD(obj2mjcf `--decompose`)로 나누거나 상자 몇 개로 근사한다. 우리 충돌 검사(coal)는 메쉬를 그대로 쓴다.
- **질량**: 출력물은 채움 비율 때문에 메쉬 부피 × 재료 밀도보다 가볍다. 슬라이서가 낸 무게를 받으면 그것을, 없으면 채움 보정 값을 쓴다. 지금 링크 부품은 질량 0이다(13.5). M7a부터 실제 질량을 넣는다.
- **M6에서 정할 것**(drift-report 13번): ① 손목 센서 자리의 기준. MuJoCo `wrist_sonar` site는 CAD x 0.0, `config/ultrasonic.yaml`의 `arm_sensor_offset_mm`은 x 0.6이다. ② 레이더는 MuJoCo site가 없고 `radar.station_mm`과 장면 상자뿐이다. ③ 가상 ESP32는 자체 `SimClock`으로, `MuJoCoDriver`는 벽시계 스레드로 돌아 시계가 다르고, 광선을 쏘는 공개 함수가 없다(잠금 `_lock`을 어떻게 쥘지 포함).
- **실물과의 차이**: 성공률 숫자는 "시뮬레이션 기준"으로만 쓰고, 실물 결과가 있으면 실물을 따른다(0037).
- **PC**: 지금 단계(M4b·M5·M6)는 노트북(CPU)으로 된다. M5b 대량 시험(수백 회)은 코어가 많은 데스크탑(RTX 5070 Ti PC)에서 병렬 프로세스로 돌린다. M9 강화학습의 GPU 병렬(MJX·MuJoCo Warp)은 NVIDIA GPU가 필요하고, JAX GPU는 리눅스(Windows는 WSL2)에서 돈다. 그래서 데스크탑이 필요한 시점은 M5b 대량 시험과 M9다.
- **마감**: 12/4까지는 M7a(끝) → M4b(끝) → M5·M6 → (여유가 있으면) M5b·M5c 순서다(결정 0063). 11/30부터 새 기능은 멈춘다. M8은 여유가 있을 때, M9는 12/4 뒤다.

#### 13.10.1 M5 집기 시험장 세부 설계 (2026-11-16, EGM Stage 9 (가) 꼭 고칠 것 8건 반영 — 구현 REQ를 낼 수 있다)
**무엇을 답하나**: 로보나인 손으로 어떤 크기·자리·각도의 물체를 집어 옮길 수 있는가, 그리고 **팔이 보고한 결과와 물리에서 실제로 일어난 일이 같은가**(기구학 엔진은 뒤쪽을 볼 수 없다).

**파일**(모두 MuJoCo 세션 영역)
| 파일 | 내용 |
|---|---|
| `arm/mujoco_bench.py` | 공통 층(세계 짓기·판정·기록)과 실행기 A. 실행: `python -m so101_studio.arm.mujoco_bench --suite <이름> [--seeds N] [--gripper parallel\|single_jaw] [--jobs N] [--only <에피소드 id>]` |
| `config/sim_scenarios/bench/robot.yaml` | 시험장 덮어쓰기. 지금 내용은 `gripper: parallel` 한 줄. `load_robot("config/robot.yaml", arm="so101_follower_01", local_path=이 파일)`로 읽어 PC의 `local.yaml`을 대신한다. U5 뒤 팔 파일 기본값이 `parallel`이 되면 빈 파일이 된다 |
| `config/sim_scenarios/bench/<suite>.yaml` | 묶음 정의(아래). 폴더가 달라 13.5의 시나리오 목록에는 섞이지 않는다 |
| `tests/test_mujoco_bench.py` | 아래 완료 기준 |

**장면을 하나로**(REQ-069 #412로 구현됨): 장면 파일은 `config/scene.yaml` 하나이고 부품의 선택 키 `gripper: parallel|single_jaw`(없으면 늘 씀)로 거른다. 거르는 것은 `Scene.for_gripper(cfg.gripper)`다. `ArmService`는 스스로 `load_scene().for_gripper(cfg.gripper)`를 읽는다. **시험장도 `load_scene().for_gripper(cfg.gripper)`를 드라이버에 넘긴다.** 그러면 물리와 계획 쪽 충돌 검사가 같은 부품을 본다. `mujoco_model.py`의 `SIM-PART-SKIPPED` 분기(단동식에 손목 부품이 오면 건너뜀)는 안전망으로 남긴다. 하드웨어 반영(H-2·H-3)의 작업대·레이더 부품도 같은 파일에 들어간다.

**에피소드**: 한 시도 = 한 에피소드이고 서로 아무것도 나누지 않는다.
1. 묶음에서 물체 참값과 "팔에게 알려 주는 값"을 만든다(씨앗 고정).
2. cfg는 `load_robot("config/robot.yaml", arm="so101_follower_01", local_path=덮어쓰기 파일)`이다. `--gripper`를 주면 에피소드 묶음 폴더에 `gripper: <값>` 한 줄짜리 임시 덮어쓰기 파일을 만들어 그것을 `local_path`로 쓴다(cfg를 읽은 뒤 `cfg.gripper`에 값을 넣는 것은 안 된다: 그리퍼 종류는 읽을 때 `config/grippers/<종류>.yaml` 층을 고른다). 드라이버는 `MuJoCoDriver(cfg, ws, scene=load_scene().for_gripper(cfg.gripper), objects=물체, realtime=, scenario=묶음 이름)`으로 **직접 만든다**(`make_sim_driver`에는 `realtime` 인자가 없다. 임시 시나리오 파일은 쓰지 않고, 겹침 검사는 `sim_driver._check_overlap`을 그대로 부른다). → 새 `ArmService(driver, cfg, ws, log_root=에피소드 폴더)`, `svc.incident_after_s = 0`(실패 주입이 사고 묶음과 60초 후속 스레드를 띄우지 않게) → `connect()`.
3. `svc.pick_and_place(given, zone 또는 place_at)`. 그동안 표본기가 `sim_state()`를 50Hz로 읽는다.
4. 끝나고 1.0초 뒤 판정하고 `disconnect()`.
- **표본과 기다림의 시계**: 지금(벽시계)은 `tests/test_mujoco_scenarios.py`의 `Sampler`(스레드, `time.sleep(0.02)`)와 `time.sleep(1.0)`이다. 시계 주입(13.10.2) 뒤 `StepClock`에서는 틱 스레드가 틱마다(`control_hz // 50`, 최소 1) 부르는 콜백으로 표본을 뜨고 기다림은 `clock.sleep(1.0)`이다. 이 둘을 고르는 자리도 드라이버·시계를 만드는 한 곳이다.
- `Sampler`, 미끄러짐 계산, 이벤트 읽기(`tests/test_mujoco_scenarios.py`, `tests/test_mujoco_parallel.py`에 흩어져 있다)는 `mujoco_bench.py`로 모으고 두 시험은 그것을 가져다 쓴다(13.7 (b) 정의를 한 곳에).
- 물리가 실패하면(`failed_reason`, 또는 `StepClock`의 5초 `RuntimeError`) 시험장이 잡아 `outcome: sim_fail`로 끝낸다.
- M5b 첫 단계(13.10.2) 전에는 실시간 모드로 돈다(에피소드 20~27초). 그 뒤에는 `realtime=False` + `StepClock`으로 바꾼다. 바뀌는 것은 드라이버·시계를 만드는 한 곳뿐이게 짠다.
- 겉모양은 Gymnasium 꼴로 둔다: `BenchEnv.reset(seed=, options=)`가 세계를 새로 짓고 관측(관절, 물체 참값, 알려 준 값)을 돌려준다. `step`은 실행기 B(M9) 몫이라 M5에서는 만들지 않는다. 실행기 A는 `reset` 뒤 위 3~4를 부르는 함수 하나다.

**묶음 정의**(YAML)
```yaml
name: pick_grid
objects:                      # 물체 한 개짜리 격자. 곱집합으로 에피소드를 만든다
  size_mm: [[30, 30, 40], [58, 58, 39], [100, 30, 40], [80, 40, 40], [50, 50, 40]]   # [길이, 폭, 높이]
  radius_mm: [120, 150, 180, 210, 240]     # 팔 원점에서
  bearing_deg: [-40, 0, 40]                # 팔 정면 기준
  angle_deg: [0, 45, 90]
place: {zone: right}                       # 또는 {at: [x, y]}
given: {}                                  # given(팔에게 알려 주는 값)을 참값에서 어긋나게 하는 격자(아래 실패 주입). 비면 참값 그대로
spread: {xy_mm: 0, angle_deg: 0}           # 씨앗마다 자리·각도를 흔드는 폭. 0이면 격자 그대로(씨앗 1개면 충분)
```
- 고정 묶음(결정 0037, 설계서·`measurements.md`에 적는 숫자는 이것만): `pick_grid`(위, 225 에피소드), `inject_grid`(아래), `pick_smoke`(시험용 12 에피소드: 크기 [30,30,40]·[58,58,39] × 반경 150·210 × 방위 0 × 각도 0·45·90).
- 에피소드 id는 `<묶음>-<격자 순번 4자리>-s<씨앗>`(예: `pick_grid-0042-s0`)이고 `--only`가 이 문자열을 받는다. `--seeds N`은 격자 칸마다 씨앗 0~N−1을 돌린다(`spread`가 0이면 N은 1로 본다). 격자 순번은 YAML에 적힌 순서의 곱집합이다(`size_mm`이 가장 바깥).
- 한 글자도 다른 뜻으로 쓰지 않는다: 참값 = MuJoCo에 놓는 물체, `given` = 팔에게 넘기는 `DetectedObject`.
- 물체 여러 개(배출 구역 겹침·폴백, 남은 물체 피하기)는 `objects`를 목록으로 주는 둘째 형식으로 M5 안에서 이어서 더한다(`multi3`: 13.5의 기본 물체 셋). 첫 PR은 한 개짜리만이다.

**판정**: 에피소드마다 "보고"와 "물리"를 따로 적고 둘을 맞춰 본다.
| 값 | 정의 |
|---|---|
| `reported` | `pick_and_place` 결과의 `status`와 `reason` 그대로. `drop_zone_full`은 독립 상태가 아니라 `stopped`의 reason 앞머리다 |
| `picked` | 물체가 `held`이면서 바닥 높이 `z_mm` > 5인 표본이 한 번이라도 있다 |
| `placed` | 끝나고 1.0초 뒤 물체 바닥 가운데가 **놓으려던 점**에서 수평 `place_tol_mm`(10) 안이고 `tilt_deg` < 10. 놓으려던 점은 구역 가운데가 아니라 `Skills`가 고른 칸이다(`svc.skills.last_place_xy`). 13.7 (b)의 7mm보다 넓은 것은 그쪽은 한 자리 회귀 시험이고 이쪽은 여러 반경·크기의 성공 판정이어서다(실물 손끝 정밀도 ±9mm) |
| `outcome` | 아래에서 처음 맞는 것 하나: `sim_fail`(물리 실패) → `rejected`(보고가 `out_of_bounds`·`height_required`·`locked`이거나, `ik_fail`이면서 도구가 1mm 미만 움직임) → `placed_ok`(`placed`) → `dropped`(`picked`였는데 `placed` 아님) → `toppled`(끝에 `tilt_deg` ≥ 10) → `pushed`(안 집혔고 처음 자리에서 10mm 넘게 밀림) → `untouched` |
| `agree` | `false_ok`(보고가 `ok`인데 `outcome`이 `placed_ok`가 아님 = 거짓 성공), `false_fail`(보고가 `ok`가 아닌데 `placed_ok`), `match`(나머지) |
| 재는 값(키) | `slip_mm`(13.7 (b)의 정의), `place_err_mm`, `tilt_end_deg`, `other_pushed_mm`(다른 물체가 밀린 최대 거리), `events`(코드별 건수: `ARM-BLOCKED`, `ARM-GRIP-OVERLOAD-WARN`, `ARM-GRIP-OVERLOAD`, `ARM-FAULT`, `ARM-CARRY-LOW`, `ARM-OPENING-PATH-FALLBACK`), `sim_s`, `wall_s`, `dropped_count`, `gap0_mm`·`jaw_yaw_deg`(아래 실패 주입) |
- 미끄러짐은 값만 적고 실패로 치지 않는다(기준과 조임 힘은 U15: 실물 2-10과 13.10.2의 결정 스텝 결과 뒤).
- 실시간 모드에서는 `connect()` 직후의 `dropped_count`를 빼고(연결 때는 늘 몇 번 놓친다, 13.3) 남는 값이 0보다 큰 에피소드에 `realtime_lag: true`를 달고 요약 표에서 따로 센다.

**실패 주입**(병렬 손의 topedge 대응, U14): 병렬 손은 높이를 잘못 알아도 윗모서리를 치지 않는다. 대신 **자리가 어긋나면 클램프가 물체 윗면에 내려앉는다.** `given`으로 팔에게 알려 주는 값을 참값에서 어긋나게 한다: `pos_offset_mm: [벌림 축, 그 수직]`, `width_error_mm`, `height_error_mm`, `angle_error_deg`.
- **G와 벌림 축**: 시험장이 `connect` 뒤 `given`으로 계산한다. G = `svc.skills._needed_gap(given, tilt_ok=False)`, 벌림 축 = `svc.skills._pick_yaw(given)` 방향의 단위벡터, 그 수직 = +90°. 둘을 `gap0_mm`·`jaw_yaw_deg`로 적는다(정사각형은 `_pick_yaw`가 두 방향 가운데 하나를 고르므로 계산해야 안다). 어긋남은 **참값 자리에 더해** `given`의 자리를 만든다. (G − W)/2의 W는 참값 폭이다.
- 고정 묶음 `inject_grid`: 58×58×39 상자(각도 0), 자리 셋(정면 150·225mm, 방위 40° 180mm) × 벌림 축 어긋남 {0, 5, 10, 15, 20}mm × 높이 오차 {−20, 0, +20}mm. 묶음 YAML은 곱집합 형식 대신 `positions: [[반경, 방위], …]`와 `given: {pos_offset_mm: [[0,0],[5,0],…], height_error_mm: [-20, 0, 20]}`처럼 목록을 주는 형식을 함께 받는다(목록끼리 곱집합).
- 보는 것: 어긋남이 (G − W)/2를 넘는 칸에서 **거짓 성공이 0건**인가(막힘 감지나 `no_object`로 끝나야 한다), 물체가 쓰러지지 않는가. 넘지 않는 칸은 `placed_ok`여야 한다. G는 그 에피소드의 시작 벌림(이벤트 기록에서 읽는다).
- 결과는 표로 보고하고, 거짓 성공이 나오면 [기획 요청]으로 올린다(막힘 문턱 `sim_blocked`나 알고리즘의 문제인지 가른다).

**기록**: `runs/bench-mujoco/<시각>-<커밋 7자>/`에 `results.jsonl`(에피소드마다 한 줄: 묶음, 에피소드 id, 씨앗, 그리퍼, 켠 장면 부품, 물체 참값, 알려 준 값, 위 판정 값 전부), `summary.json`, `summary.md`(크기 × 반경 성공률 표, `outcome`별 건수, 거짓 성공·거짓 실패 목록), 에피소드별 `events.jsonl`·`status.jsonl`(ArmService가 남기는 그대로). 실패 영상은 모델에 카메라가 들어간 뒤(H-3) `--video failed`로 더한다.

**나란히 돌리기**: `--jobs N`은 에피소드를 프로세스 N개에 나눠 주는 것뿐이다(대량 병렬·스케줄러는 M5b 둘째 단계). 실시간 모드에서는 서로 CPU를 빼앗아 물리가 밀리므로 N은 물리 코어 수의 절반까지로 하고 `realtime_lag` 건수를 본다. `pick_grid` 225개는 실시간 4개로 약 25분이다.

**완료 기준**(`tests/test_mujoco_bench.py`, 실시간이라 `slow`)
1. `pick_smoke`가 끝까지 돌고 `results.jsonl`이 12줄, 줄마다 판정 표의 키(`reported`, `picked`, `placed`, `outcome`, `agree`, 재는 값 키)가 다 있다.
2. 같은 묶음·씨앗이면 물체 참값과 알려 준 값이 두 번 돌려도 같다(세계가 씨앗으로 정해진다. 결과까지 같은 것은 13.10.2 뒤에 본다).
3. 한 에피소드의 `ArmService`가 다음 에피소드에 넘어가지 않는다(둘째 에피소드의 `Skills`에 놓은 자리·남은 물체 기억이 없다).
4. 덮어쓰기 `gripper: parallel`에서 드라이버 장면 부품과 `svc.scene` 부품 이름이 같고 손목 부품 둘이 들어 있다. `--gripper single_jaw`에서는 둘 다 없고 cfg의 그리퍼 층이 단동식이다(`cfg.grasp_tilt_deg`가 0이 아님).
5. 실패 주입 한 칸: 58 상자, 정면 180mm, 벌림 축 어긋남 = (G − W)/2 + 8mm → 클램프(`clamp_right` 또는 `clamp_left`)와 물체의 접촉 가운데 `z_mm`가 물체 높이 ±5mm인 것이 한 번 이상 나오고 `outcome`이 `placed_ok`가 아니다.
6. 손으로 한 번: `pick_grid` 전체 결과의 `summary.md` 표를 `measurements.md`에 커밋·묶음 이름과 함께 남긴다.

**파일 정리**: 만들기 = `arm/mujoco_bench.py`, `config/sim_scenarios/bench/robot.yaml`·`pick_grid.yaml`·`inject_grid.yaml`·`pick_smoke.yaml`, `tests/test_mujoco_bench.py`. 고치기 = `tests/test_mujoco_scenarios.py`·`tests/test_mujoco_parallel.py`(표본기·미끄러짐을 가져다 쓰게), `measurements.md`(기준 6). 팔 영역은 고치지 않는다(REQ-069는 끝났다). 시험장은 `Skills`의 비공개 함수 둘(`_needed_gap`, `_pick_yaw`)과 `last_place_xy`를 읽기만 한다.

#### 13.10.2 M5b 첫 단계: 시계 주입 (2026-11-16, Stage 9 (가) 지적 반영. **구현은 12/4 뒤(2차, 2026-11-17 사용자 승인, 결정 0066 상태 줄). 구현 전에 이 절만 Stage 9를 다시 돌린다**)
**왜**: 제어가 벽시계에 묶여 있어(`arm/motion.py`·`arm/skills.py`·`service/arm_service.py`의 `perf_counter`·`sleep`·`monotonic` 46줄) `ArmService` 위 시험과 시험장이 실시간보다 빨라지지 못한다. 병렬 58mm 미끄러짐이 실행마다 0.8~10mm로 갈리는 원인이 실시간 지터인지도 이것 없이는 가를 수 없다(원인은 아직 확인하지 않았다).

**미루는 이유**(2026-11-16 밤): 결과가 실행마다 같으려면 아래 "보조 맞추기"까지 해야 하고, 그것은 팔 핵심 세 파일의 46줄과 `ArmService` 공개 메서드 전부를 건드린다. 실물 팔이 오는 주에 이 변경을 넣는 위험이 크고, 12/4까지 필요한 수집·평가는 실시간으로도 된다(300 에피소드 약 2시간).

**설계**
- `core/clock.py`(팔 영역): `Clock` 규약은 `now() -> float`(단조 초), `sleep(seconds)`, `tick_sleep(seconds)`(제어 틱 스레드만 부른다), 읽기 전용 `drives_status: bool`이다. `WallClock`은 `time.perf_counter`·`time.sleep`을 그대로 부르고 `tick_sleep`은 `sleep`과 같으며 `drives_status`는 False다. 초음파 쪽에 이미 있는 `RealClock`·`SimClock`(`now_ns()`·`sleep()`, `perception/ultrasonic/model.py`·`fake_esp32.py`)은 이 파일로 옮겨 같은 규약 아래 둔다(13.10.3에서 어댑터가 필요 없게).
- `MotionController(..., clock=WallClock(), on_status=None)`, `Skills(..., clock=)`, `ArmService(..., clock=)`: 제어 시간에는 `time` 모듈을 직접 부르지 않는다. `ArmService`가 받은 시계를 둘에 넘긴다. **기본값에서는 동작이 지금과 같다**(실제 팔, `SimDriver`, 실시간 MuJoCo, GUI).
- 기록 시각(`time_ns`, 이벤트·상태 줄의 t)은 벽시계 그대로다. 단 `follow_path`가 돌려주는 `PoseSample.t_ns`는 받은 시계로 찍는다: `t_ns = int(clock.now() * 1e9)`.
- `StepClock(driver)`(`arm/mujoco_clock.py`, MuJoCo 영역): `now()`는 드라이버의 시뮬레이션 시간, `drives_status`는 True다. `tick_sleep(s)`만 `driver.advance(s)`를 부른다. 다른 스레드의 `sleep(s)`는 시뮬레이션 시간이 그만큼 흐를 때까지 기다린다(조건 변수. 벽시계 5초 동안 시간이 안 흐르면 `RuntimeError`).
- **보조 맞추기**(결과가 실행마다 같으려면 꼭 필요하다): 틱 스레드는 등록된 다른 스레드가 모두 `clock.sleep`이나 `MotionHandle.wait` 안에서 기다리는 동안에만 `advance`한다. "깨어 있는 스레드 수"를 센다: `ArmService` 공개 메서드에 들어가면 1을 더하고, 기다림에 들어가면 1을 빼고, 나오면 되돌린다. 수가 0보다 크면 틱 스레드도 기다린다. 이것이 없으면 호출 스레드가 IK·계획을 계산하거나 접촉 판단을 폴링하는 동안(`skills.py`의 `time.sleep(0.04)` 폴링) 시뮬레이션 시간이 얼마나 흐를지를 GIL과 OS 스케줄이 정해, 그리퍼가 멈추는 값이 실행마다 달라진다. MuJoCo 자체는 입력이 같으면 결과가 같다.
- 상태 루프(10Hz): `MotionController._loop`는 `with self._lock` 블록이 **끝난 뒤**, `clock.tick_sleep` 앞에서 `max(1, control_hz // 10)`틱마다 `on_status()`를 부른다(락 안에서 부르면 `check_blocked`·`note_comm`이 같은 락을 다시 잡아 멈춘다. 이 락은 재진입이 안 된다). `ArmService.connect`는 `clock.drives_status`이면 상태 스레드를 띄우지 않고 `on_status`에 상태 루프 한 바퀴 본문을 넘긴다. 벽시계에서는 지금 그대로다.
- `StepClock`은 `MuJoCoDriver(realtime=False)`와만 쓰고 뷰어와는 쓰지 않는다(만들 때 `driver.realtime`·`driver.viewer`를 검사해 `ValueError`). 13.3에는 "`realtime=False`는 `advance`로만 시간이 흐른다. `StepClock`이 그 `advance`를 부르는 시험·시험장 전용 경로다"를 더한다.

**같은 시계를 쓰는 것**(구현 PR이 하나씩 확인해 표로 남긴다): 막힘 판정 시간(`BLOCKED_ZONE_S`, `_blocked_until`), 통신 끊김 창(`HealthMonitor.comm_s`), 부하·과열 감시, 틱 따라잡기 `MAX_TICK_ADVANCE`, 조그 데드맨, `wait_idle`, 접촉 판단·놓은 뒤 기다림. **벽시계로 남는 것(허용 목록)**: 기록 시각, `ArmService`의 탐침·사고 묶음 쿨다운(`time.monotonic`), 기록 스레드, 감시 프로세스 heartbeat, 교시 기록(`record`).

**완료 기준**
1. 기본 시계에서 `python -m pytest -q -m "not slow and not measure"` 전체가 그대로 통과한다. 세 파일에 제어 시간을 `time` 모듈로 직접 부르는 곳이 남지 않았는지 보는 시험이 있다(위 허용 목록만 남는다). 시험 파일은 `tests/test_clock.py`(팔)와 `tests/test_mujoco_clock.py`(MuJoCo)다.
2. `carry45`(병렬 손, 시험장 실행기로)를 `realtime=False` + `StepClock`으로 돌리면 `placed_ok`이고 놓은 자리 7mm·기울기 10° 안이며, 벽시계로 실시간 모드의 1/5 이하다(목표. 못 미치면 측정값과 막는 곳을 보고).
3. 같은 씨앗을 두 번 돌리면 `results.jsonl`의 판정 값이 같고 미끄러짐·놓은 자리 오차가 0.01mm 안에서 같다.
4. `carry58`을 같은 씨앗으로 14번 돌린 미끄러짐 분포를 `measurements.md`에 남긴다. 범위가 0.01mm 안이면(기준 3이 통과한 상태에서) 실시간에서 갈린 것은 지터였다. 그다음 씨앗을 바꿔(자리 ±2mm) 14번 돌린 분포가 물리 쪽 편차다. U15의 입력이다.
- 스레드를 스케줄러 하나로 모으는 일과 대량 병렬 시험은 이 단계 뒤(M5b 둘째 단계)다.

**공유 파일**: 팔 영역 셋(`arm/motion.py`, `arm/skills.py`, `service/arm_service.py`)과 `core/clock.py`. 인자가 늘어나는 변경이라 팔 세션이 구현하고, `StepClock`과 시험은 MuJoCo 세션이 한다. 순서는 총괄이 정한다.

#### 13.10.3 M6 초음파를 MuJoCo에 세부 설계 (2026-11-16 초안, EGM Stage 9 전)
**무엇을 답하나**: 실제 브래킷·손·팔 형상에서 초음파 인식이 도는가. 지금 벤치마크의 가상 ESP32는 상자 목록과 수식으로 거리를 만들어, 빔이 자기 클램프나 팔에 걸리는 것, 레이더가 실제 팔 형상에 가리는 것, 인식한 자리로 **물리에서 실제로 집히는지**를 볼 수 없다.

**도구는 하나**(결정 0037): 새 도구가 아니라 초음파 벤치마크(`ultrasonic-spec.md` 8.5)에 `--engine analytic|mujoco`(기본 `analytic`) 옵션을 더한다. 레이더·확인·작업 실행 코드는 실제 코드 그대로다.

**순서 조건**: 13.10.2(시계 주입) 뒤에 한다. 가상 ESP32의 시계(`SimClock`)와 `MuJoCoDriver`의 시간이 하나여야 팔 자세 샘플과 센서 측정의 시각이 맞는다. `--engine mujoco`에서는 가상 ESP32가 `StepClock`과 같은 시뮬레이션 시간을 쓴다.

**설계**
- `arm/mujoco_sonar.py`(MuJoCo 영역) `SonarRays(driver)`: `range_mm(origin_mm, axis, half_angle_deg, max_mm, n_rays=19) -> float | None`. 원뿔 안에 광선을 고르게 쏘고(`mj_multiRay`, 드라이버 잠금 안에서) 가장 가까운 거리를 낸다(첫 메아리). 센서 자기 몸(센서 부품 geom)은 뺀다. 잡음·응답 없음·면 각도 한계는 `perception/ultrasonic/model.py`의 지금 모델 함수를 그대로 쓴다(값을 두 곳에 두지 않는다). 면 각도에 필요한 법선은 맞은 geom이 상자면 맞은 면에서 구한다(법선을 주는 광선 API가 MuJoCo 4.0에 있는지는 구현 때 문서로 확인하고 PR에 적는다).
- 가상 ESP32의 거리 계산을 갈아 끼울 수 있게 한다(초음파 영역 REQ): 지금 수식 계산을 기본 "거리 제공자"로 두고, `--engine mujoco`면 `SonarRays`를 쓰는 제공자를 넣는다. 규약은 `radar(angle_deg) -> mm | None`, `arm() -> mm | None` 둘이다. 인식 모듈이 MuJoCo를 import하지 않게 제공자는 벤치마크가 만들어 넘긴다.
- 손목 센서: 원점·축은 모델의 `wrist_sonar` site다(CAD 값, x 0.0). 인식 쪽은 지금처럼 `arm_sensor_offset_mm`(x 0.6)을 쓴다. **맞추지 않는다**: 0.6mm 차이는 실물의 조립 오차 구실을 하고, MuJoCo 엔진에서 A0 절차(10mm 막대 십자 확인)를 돌려 오프셋이 site 값으로 고쳐지는지로 A0 절차 자체를 시험한다.
- 레이더: 모델에 관절을 더하지 않는다. `radar.station_mm`·`radar.height_mm`·명령 각도로 센서 원점과 축을 계산해 광선을 쏜다(서보 지연 모델은 지금 것 그대로). 레이더 받침 형상은 장면 부품이다(H-3).
- 바닥: 광선이 맞을 판이 있어야 한다. H-3 전에는 `table_mm` 크기의 얇은 상자를 시험장이 임시로 놓는다.
- 물체: 벤치마크 시나리오의 상자를 MuJoCo 물체로 놓고(13.5의 겹침 검사 그대로), 판정의 참값은 **그 순간의 `sim_state()`**에서 읽는다(팔이 밀면 참값도 움직인다).
- 집기까지: `--engine mujoco --with-pick`이면 확인 뒤 `pick_and_place`를 물리로 돌리고 13.10.1의 판정(`placed_ok` 등)을 같은 결과 줄에 붙인다. 벤치마크의 "성공"(위치 15mm·방향) 정의는 그대로 두고 "물리로 놓임" 열을 더한다.

**완료 기준**
1. 정적 대조: 빈 판 위와 40mm 상자 위에서 손목 자세 20개의 `SonarRays` 거리가 수식 모델과 3mm 안이다(잡음을 끈 채).
2. 빈 판에서 레이더 한 바퀴가 수식 모델의 기준값과 각도마다 3mm 안이다(판 밖을 보는 각도는 둘 다 응답 없음).
3. 팔 가림: `radar_clear` 자세와 판 위로 뻗은 자세에서 레이더가 팔을 보는 각도 목록을 내고, 5절 "팔 지우기"가 그 각도를 모두 덮는지 본다(덮지 못하는 각도가 있으면 [기획 요청]).
4. 자기 간섭: 확인 높이 55mm, 그리퍼 닫힘·최대 벌림에서 손목 빔에 클램프·브래킷이 걸리는 자세가 있는지(M7a는 정적 CAD 평가였다).
5. `single` 씨앗 20을 `--engine mujoco`로 돌린 성공률을 수식 엔진의 병렬 실제 기구학 값과 나란히 보고한다. 10%p 넘게 다르면 원인(빔 모델, 가림, 물체 밀림)을 적는다. 이 값은 총괄이 다시 돌린 뒤 `ultrasonic-spec.md` 1절에 "MuJoCo 엔진" 줄로 적는다.
6. `demo` 씨앗 5를 `--with-pick`으로 끝(`DONE`)까지 돌려 "물리로 놓임" 수를 낸다.

**공유 파일**: 초음파 영역(`fake_esp32.py`의 거리 제공자, `benchmark.py`의 `--engine`)은 초음파 세션, `arm/mujoco_sonar.py`와 시험은 MuJoCo 세션이다.

**세 단계의 순서와 일정**: 13.10.1(M5, 실시간으로 바로 시작) → 13.10.2(시계 주입) → 13.10.3(M6). 하드웨어 반영 H-3(새 작업대·받침·카메라)은 어느 단계에서든 `config/scene.yaml`과 모델만 바꾸면 되고, 그 뒤 고정 묶음을 다시 돌려 표를 갱신한다. 11/30 동결 전에 M5와 시계 주입까지를 목표로 하고, M6는 초음파 실물(B1~I1)과 겹치면 실물이 먼저다(결정 0062-4).

#### 13.10.4 카메라, 화면 밖 렌더, 에피소드 기록 (결정 0066, 2026-11-16, EGM Stage 9 (나) 꼭 고칠 것 반영 — 1차 범위는 구현 REQ를 낼 수 있다)
약어: L1~L5는 결정 0066의 단계(다중 시점, 수집, ACT 학습, 정책 실행, 강화학습 데모), H-1·H-3은 `hardware.md`의 하드웨어 반영 단계다. **범위**: "1차"는 지금 REQ를 내는 것, "2차"는 12/4 뒤로 미룬 것(2026-11-17 사용자 승인, `learning-platform.md` 3a)으로 지금은 REQ를 내지 않는다.

**배치와 판정의 공개 규약**(1차, `arm/mujoco_bench.py`가 공개한다. GUI와 `PolicyRunner`는 이 둘만 import한다)
- **배치** = 묶음과 씨앗 하나가 정하는 물체 크기·자리·각도, 놓을 곳, 과제 문장이다. 배치 하나에 에피소드 하나.
- `layout(suite: str, seed: int) -> Layout(objects, place_xy_mm, task)`.
- `judge(driver, layout) -> dict(outcome, placed, place_err_mm, tilt_end_deg, …)`: `driver.sim_state()`와 `layout`만 읽고, `pick_and_place`의 보고 없이도 돈다(13.10.1 판정 표의 물리 쪽).
- 씨앗 범위: 전문가 수집 0~9999, 평가 10000~10049, GUI 수집(2차) 20000부터.

**모델 카메라**(1차, MuJoCo 세션. 판·받침 형상은 H-3 때 바뀌어도 카메라 이름과 규약은 그대로다): `load_model`이 카메라 둘을 더한다.
| 이름 | 붙는 곳 | 자리와 자세 | 비고 |
|---|---|---|---|
| `top` | 월드 고정 | **작업대 좌표**의 판 중심(`workspace.yaml` `table_mm` 직사각형의 가운데) 위, 판 면에서 550mm(결정 0058의 50~60cm). 월드 −z를 본다. 영상 위쪽이 작업대 +y(팔에서 먼 쪽)가 되게 돌린다. 시야각은 판 전체가 320×240에 들어오는 값 | 판이 모델에 들어오기(H-3) 전에는 `table_mm` 크기의 임시 판 기준이다. 판 크기가 바뀌면(H-1) 같이 고친다 |
| `wrist`(병렬 손) | MuJoCo body `gripper`(두 손 모두 이 이름이고 URDF의 `gripper_link`와 같은 프레임) | pos (0, +55, −75)mm, 바라보는 점 (0, 0, −140)mm, fovy 75°, 화면 가로 = 벌림 축(x) | 센서·브래킷이 −y 쪽이라 반대편 +y에 둔다. 쥘 때 클램프 끝(z −132)이 아래 모서리에서 조금 잘린다. MuJoCo 세션이 `carry58` 집기 중 후보 다섯을 비교해 고른 값 |
| `wrist`(단동식 손) | body `gripper` | 원본 `so101.xml`의 `wrist_cam` 값 그대로: pos (−3, 54, −53)mm, 방향(quat) 원본, fovy 61° | |
- 카메라는 물리에 영향이 없다(질량 0, 충돌 없음). **고정 자세**다(에피소드마다 바꾸지 않는다). 실물 손목 카메라는 없다(결정 0063).
- `load_model`이 `camera_mount`·`wrist_cam`을 지우는 것(`mujoco_model.py:111`, 질량·형상을 빼려는 것)은 그대로 두고, 카메라만 `wrist` 이름으로 더한다. 그 줄의 주석은 결정 0066에 맞게 고친다.

**렌더**(1차): `MuJoCoDriver.render(camera: str, width: int = 320, height: int = 240) -> numpy 배열(높이, 폭, 3) uint8`.
- 잠금 안에서는 `mj_copyData(self._rdata, model, data)`만 하고 그리기는 잠금 밖에서 한다(물리 스레드를 막지 않는다).
- `_rdata`와 렌더러는 `connect`·`disconnect`에서 버리고 다음 `render`에서 새 모델로 만든다(`connect`가 부를 때마다 모델을 새로 만들기 때문이다).
- 렌더러를 만든 스레드에서만 쓸 수 있으면(Windows에서 첫 PR에 확인) 드라이버 안 렌더 전용 스레드 하나가 요청 큐를 받는다. 이때도 `render`의 모양은 같다. MuJoCo 세션 확인: 메인 스레드에서 만들고 그 스레드에서 그리면 물리·`ArmService`가 도는 중에도 된다.
- 모르는 카메라 이름은 `ValueError`, 연결 전이면 `RuntimeError`. `SimDriver`에는 이 메서드가 없다(쓰는 쪽은 `hasattr(driver, "render")`로 가른다).
- 화면 밖 버퍼는 기본 640×480이다(`so101.xml`에 크기 지정이 없다). 320×240은 그 안이고, 더 키우려면 모델에 버퍼 크기를 적어야 한다.
- `--sim-viewer`와 함께 써도 된다. 재서 `measurements.md`에 남길 것: 한 장 시간, 화면 10fps만 돌릴 때와 화면 10fps + 기록 25fps를 함께 돌릴 때의 `dropped_count`(총괄 확인값: 320×240 한 장 1.1ms).

**명령 관절값의 공개 통로**(1차, 팔 REQ): 팔 세션이 `ArmStatus`에 `cmd: JointState`(지금 명령 관절값, 동작이 없으면 마지막 명령)를 더하고 `status()`가 채운다(`core/types.py`, `service/arm_service.py`). 기록기는 `svc.status().cmd`만 읽는다.

**에피소드 기록기**(1차, `arm/mujoco_record.py`, MuJoCo 세션)
- `EpisodeRecorder(svc, driver, dataset_dir, fps=25, cameras=("top", "wrist"), task="…")`, `start()`, `stop(save: bool)`, `close()`.
- 한 줄: `observation.state` = `driver.read_joints()`의 실제 관절 6개, `action` = `svc.status().cmd`의 명령 관절 6개, `observation.images.top`·`observation.images.wrist` = `render` 결과. 단위는 관절 1~5가 °, 그리퍼가 0~100(0 닫힘)이다. 이름은 `shoulder_pan.pos … gripper.pos`(`core/types.py` 순서)다. 키·차원·단위는 LeRobot `so101_follower`(각도 단위)와 같지만 영점까지 같다는 보장은 없다(실물로 옮기는 것은 12/4 뒤).
- 데이터셋: `dataset_dir`가 없으면 `LeRobotDataset.create(repo_id="local/<이름>", root=dataset_dir, fps=fps, robot_type="so101_follower", features=…, use_videos=True)`로 만든다. **이미 있으면 `ValueError`로 거부한다**(이어 쓰기는 2차). `stop(save=True)`는 에피소드를 저장하고(`save_episode`), `stop(save=False)`는 버리며(`clear_episode_buffer`), `close()`는 마무리한다. lerobot 0.6.1 설치본의 실제 함수 이름과 Windows에서 영상 인코딩이 되는지는 첫 PR에서 확인해 적는다. 영상이 안 되면 `use_videos=False`(그림 파일)로 바꾸고 보고한다.
- 과제 문장 `task`를 에피소드마다 적는다(결정 0066의 5항).
- 기록 구간은 `start()`부터 마지막 동작 뒤 판정 대기 1.0초까지다.
- 시계: `time.perf_counter`로 25Hz를 맞춘다(실시간 수집). 13.10.2가 들어오면 `ArmService`의 시계로 바꾼다. **기록기는 13.10.2를 기다리지 않는다.**
- `lerobot[dataset]`이 없으면 "lerobot[dataset]이 설치되지 않았습니다. docs/guides/tooling-setup.md C-8"로 끝낸다. 데이터셋 폴더는 `runs/datasets/<이름>/`(git 밖)이다. 이름은 글자·숫자·`_`·`-`만 받는다.

**스크립트 전문가**(1차, 13.10.1에 붙임): `python -m so101_studio.arm.mujoco_bench --suite collect_pick --record <데이터셋 이름> --seed-start 0 --seeds 300`. 에피소드마다 기록기를 켜고, **`outcome`이 `placed_ok`인 에피소드만 저장**한다(실패는 `results.jsonl`에만 남는다).
- 수집용 묶음 `collect_pick`: 58×58×39 상자 하나(시연 물체), 반경 120~220mm·방위 ±45°·각도 0~180°를 씨앗으로 고르게 뽑고 배출 구역 `right`에 놓는다. 300 에피소드가 첫 목표다(공개 사례: 300개·4만 스텝에 42~50%).
- 끝난 기준: 300 에피소드 데이터셋이 생긴다. LeRobot의 데이터셋 읽기로 길이·키·영상 크기를 확인하는 시험이 통과한다. 저장한 에피소드 넷을 골라 같은 씨앗의 배치에서 `action`을 `servo_joints`로 다시 틀면 `judge`가 `placed_ok`다(실시간이라 넷 중 셋 이상).

**학습과 평가**(1차, L3. 코드가 아니라 절차, `so101-train` 환경)
- LeRobot의 ACT 학습 명령을 그대로 쓰고 `--output_dir runs/train/<이름>`을 준다. `so101-train`의 lerobot도 0.6.1로 고정한다(체크포인트 형식이 앱 환경과 같아야 한다).
- 평가는 `mujoco_bench --suite collect_pick --policy <체크포인트 폴더> --seed-start 10000 --seeds 50`이다(`PolicyRunner`를 GUI 없이 쓴다, `arm-software-spec.md` 6.10). 체크포인트는 손실이 아니라 이 50개의 `placed_ok` 비율로 고른다.
- 한 번 추론으로 쓰는 스텝 수(`n_action_steps`, 기본 100이면 4초 동안 관측을 보지 않는다)는 L3에서 값을 정해(예: 25) 학습 명령·스텝 수·성공률 표와 함께 `measurements.md`에 남긴다. 성공률 목표는 두지 않는다(결정 0066).

**세계 다시 짓기**(2차: GUI 수집 화면과 정책 실행 화면의 [다음 배치]에만 필요하다)
- `Backend.rebuild_world(seed)` 하나로 한다. 순서: ① `PolicyRunner.stop()`과 `EpisodeRecorder.stop(save=…)` ② `MuJoCoDriver.set_objects(layout.objects)`(13.5 겹침 검사 그대로) ③ `backend.sim.connect(fresh=True)`: 새 `Skills`에 `placed`·`known_objects`·`grip` 기억을 넘기지 않는다(지금 `connect`는 넘긴다) ④ 수동 물체 목록과 3D를 바꾼다. 팔은 ③에서 홈 자세로 놓인다.
- `set_objects`는 MuJoCo 세션, `fresh` 인자는 팔 세션, `rebuild_world`는 GUI 세션이 만든다.

**강화학습 데모**(2차, L5, `so101-train` 환경): so101-nexus의 `MuJoCoPickLift-v1`과 그 예제 PPO 학습을 그대로 돌려 학습 곡선과 학습 전·후 영상을 남긴다. 우리 코드에 넣지 않고 `docs/guides/`에 실행 절차만 적는다. so101-nexus(`mujoco>=3.0`)가 우리 고정 버전 4.0.0과 도는지는 설치할 때 확인한다.
