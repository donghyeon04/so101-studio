상태: 초안 (EGM Stage 9 결과, gui-spec 12.8 + arm-software-spec M4, 2라운드, 2026-11-13)

# MuJoCo M4 구현 준비 점검 2라운드 (gui-spec 12.8 + arm-software-spec M4)

**결론: M4 기준 FAIL.** 🔴 3건, 🟡 8건, 🟢 12건이다.

- 1라운드 🔴 R1·R2는 해결됐다. R3·R4는 큰 틀이 정해졌지만 일부가 남았다. 팔 쪽(뷰어, `size_mm`, CLI `gui` 거부 해제)은 지금 문장대로 만들 수 있다.
- 새 🔴 셋은 모두 GUI 쪽이고, 셋 다 종합기가 코드를 열어 확인했다.
  - **B1** 물리가 멈추면 상태 콜백이 아예 돌지 않는다. 그래서 12.8의 "fault 문장 대신 …"을 GUI가 감지할 길이 없고, `<이유>`를 읽을 공개 속성도 없다(1라운드 A5가 풀리지 않았다).
  - **B2** "3D 끌기는 끈다"가 가리키는 물체 끌기는 3D에 없다. 3D의 끌기는 집기 점 손잡이(`/handle`) 하나뿐이라, 그대로 하면 직접 조작의 목표 점 고르기가 꺼진다.
  - **B3** GUI가 쓸 수 있는 유일한 물체 파일 `config/manual_objects.yaml`의 기본 물체가 서로 겹친다. MuJoCo는 시작하자마자 상자들을 밀어내고, `ManualSource` 좌표와 물리 위치가 처음부터 어긋난다.
- 모두 문장으로 고칠 수 있다. 아래에 붙여 넣을 문장을 적었다.

## 대상과 리뷰어

- 대상(main ebd36c2, 폴더 `<m4b>` = `C:\Users\김동현\AppData\Local\Temp\claude\aa7416ddebb701be\599578e8-9a81-4c89-b7db-e2e310eeeebc\scratchpad\m4b`):
  - `docs/planning/gui-spec.md` 12.8절(425–455행). 아래에서는 **G**로 적는다.
  - `docs/planning/arm-software-spec.md`의 M4 부분: 3행(상태 줄), 419행(작업 순서표 M4), 571–617행(13절 머리, 13.1 M4 행, 명령 처리 `gui`, 뷰어), 718–735행(13.6 `sim_state`·`size_mm`·`no_object`), 770–779행(13.7 (f)), 789행(13.8). 아래에서는 **A**로 적는다.
- 함께 읽은 코드: `src/so101_studio/gui/`(`__main__.py`, `app.py`, `backend.py`, `programs.py`, `scene3d.py`, 그리고 리뷰어가 따라간 `sensors/manual_view.py`, `pages/table.py`, `state.py`), `arm/mujoco_driver.py`, `arm/sim_driver.py`, `arm/mujoco_model.py`, `cli/__init__.py`, `service/`, `tests/gui/`, `tests/test_cli*.py`, `tests/test_mujoco_driver.py`, `config/manual_objects.yaml`. 코드 경로는 `src/so101_studio/` 아래 기준이다.
- GoldfishContext: `CLAUDE.md`, 결정 0053(3행에 M4 3D 표시 승인), 1라운드 보고(`runs/egm/2026-11-13-mujoco-m4/implementation_readiness_report.md`), 사용자 승인 결정 셋(11-13: ① 3D에 물리 물체 표시, ② GUI에서 `--sim-scenario` 막기·MuJoCo면 화면 물체 편집 거부, ③ 회전 행렬은 M4에 넣지 않음).

| 리뷰어 | 기준 | 상태 | M4 판정 |
| --- | --- | --- | --- |
| `egm-implementation-readiness` | implementation-readiness-standard | completed | FAIL (🔴 2) |
| `egm-codebase-details` | implementation-readiness-standard | completed | FAIL (🔴 3) |
| `egm-style` | 없음 | completed | 🔴 없음, 🟡 5 |

- 세 리뷰어 모두 한 번에 끝났고, 다시 돌린 것은 없다. 셋 다 읽기만 하고 아무것도 실행하지 않았다. style은 코드를 열지 않았다(맡긴 범위).
- 판단이 다르면 더 엄한 쪽을 따랐다. B3(물체 겹침)은 codebase-details만 짚었고 Blocker로 봤다. 종합기가 YAML 값과 `mujoco_model.py`의 상자 만들기를 대조해 겹침과 충돌을 확인하고 🔴로 두었다.
- 종합기가 직접 확인한 것:
  - `arm/mujoco_driver.py:115-118`(`_fail`이 `_failed`에 이유를 넣고 `SIM-PHYSICS-FAIL` 이벤트), `:173-181`(`_check`가 `RuntimeError("물리 시뮬레이션이 멈췄습니다: …")`, `read_joints`가 부름), `:44`(`_failed`는 비공개), `:29·71·89`(`connected`)
  - `service/arm_service.py:211-213`(`status_stream`), `:231-257`(`_status_loop`: `self.status()`가 `:244`에서 예외를 내면 `:247-251` 콜백을 건너뛰고 `:255` `note_comm(False)`)
  - `gui/backend.py:229-230`(`comm_lost()`는 `self.real`이 있어야 참), `:280-288`(`move_lock()`은 `real is None`이면 None)
  - `gui/scene3d.py:181-184`(`/handle` transform controls), `:326-338`(`_on_drag`), `:267`(`set_layer`), `:279`(`set_objects`)
  - `arm/sim_driver.py:29-45`(시나리오 없으면 `config/manual_objects.yaml`을 `obj-1`부터, 겹침 검사 없음), `arm/mujoco_model.py:156-158`(물체는 `angle_deg`로 돌린 상자, 길이 축이 각도 방향), `:179`(충돌 끄기는 장면 부품만, 물체는 서로 부딪힌다)
  - `config/manual_objects.yaml:5-7`

## 1라운드 지적 반영

| 1라운드 | 상태 | 근거·남은 것 |
| --- | --- | --- |
| 🔴 R1 GUI 설계 문서 없음 | 해결 | G12.8이 생겼고 A419·A576-580·A602·A735·A789가 그 절을 가리킨다. 절 번호는 12.9가 아니라 12.8이다. M4 REQ는 아직 `session-requests.md`에 없다(M4 참고). |
| 🔴 R2 엔진·시나리오·뷰어 값의 출처 | 해결 | 드라이버 주입 대신 `Backend(..., sim_engine=, sim_viewer=)`로 했다. `cfg`가 `Backend.__init__` 안(`backend.py:139`, `ws`는 `:158`)에서 읽히므로 맞다. 쌍둥이는 늘 `SimDriver`, CLI 거부 해제·시험 교체는 팔 일로 적혔고, 결정 0053 3행도 고쳤다. 생성자 기본값·import는 🟢 n5. |
| 🔴 R3 수동 물체와 MuJoCo 물체 | 일부 | 시나리오를 막고 편집을 거부하는 것은 정해졌다. 남은 것: 거부를 어떻게 알리는지와 `manual_view.py`(🟡 M1), "3D 끌기"(🔴 B2), 기본 물체 겹침(🔴 B3). |
| 🔴 R4 3D 표시 | 일부 | `/sim_objects` 새 노드, `size_mm`, 회전 없음은 정해졌다. 남은 것: `held` 전달(🟡 M2), 반투명·레이어(🟡 M3), 크기 축(🟡 M4). |
| 🟡 A1 `--sim-viewer` | 해결 | A611-617. 🟢 n10·n11만 남는다. |
| 🟡 A2 (f) 시험 계획 | 해결 | A770-779와 G447-454. 🟢 n6·n7·n8. |
| 🟡 A3 import 실패 | 해결 | G435, A789. |
| 🟡 A4 `no_object` 식 | 해결 | G436-437, A733-735. 주석 행은 🟢 n2. |
| 🟡 A5 물리 멈춤 | **미해결** | 문장은 생겼지만 감지할 길이 없다(🔴 B1). |
| 🟡 A6 파일 목록 | 일부 | `scene3d.py`가 들어가고 `programs.py`가 빠졌다. `gui/sensors/manual_view.py`가 빠졌다(🟡 M1·M6). |
| 🟡 A7 끝난 일을 계획처럼 | 해결 | A576-579. 비슷한 시제가 A589·A728·A732에 남았다(🟢 n12). |
| 🟡 A8 작업대 저장 안내 | 해결 | G444. 다만 빨간 오류로 보인다(🟡 M6). |
| 🟢 n1 (`cli` 168행) | 해결 | A591. |
| 🟢 n2 (서명 `scenario=None`) | 해결 | A588. |
| 🟢 n3 (사용자 문장의 "M4") | 해당 없음 | A610에서 과거형이 됐고 M4에서 분기를 지운다. |
| 🟢 n4 (용어) | 일부 | A574에 용어 줄이 생겼지만 사용자 문장 규칙과 실제 문장이 다르다(🟡 M7). A572에 "기구학 시뮬레이션"이 남았다. |
| 🟢 n5 (다시 시작 버튼) | 고쳐졌지만 틀림 | 그 버튼은 아직 없고, 실물 모드에서 누르는 버튼이라 엔진 인자가 성립하지 않는다(🟡 M5). |
| 🟢 n6 (상태줄 "시뮬레이션(MuJoCo)") | 해결 | G445. |
| 🟢 n7 (A586 "M3에서 채운다") | 해결 | A719 "M3 #290에서 채웠다". |
| 범위 밖 `test_port_pull.py:215,240` | 그대로 | `diagnostics.SimDriver`가 없는데 monkeypatch한다(codebase-details가 다시 확인). |

## 새 지적

### 🔴 Blocker

**B1. 물리가 멈추면 GUI는 그것을 감지할 수 없고, 12.8의 전제("통신 끊김처럼 fault, fault 문장 대신")가 코드와 다르다 (G443, A789 근처 13.2 참조)**
- **리뷰어:** implementation-readiness·codebase-details(둘 다 Blocker), style(Major, 문장 중복). 종합기가 코드로 확인했다.
- **문제:**
  - 물리가 멈추면 `read_joints`가 `RuntimeError`를 낸다(`mujoco_driver.py:178-181` → `:173-175`). `ArmService._status_loop`는 `self.status()`(`arm_service.py:244`)에서 예외가 나므로 콜백(`:247-251`)을 하나도 부르지 않는다. 그 뒤 `state.update`, 3D 갱신, G439의 새 콜백이 모두 멈춘다.
  - 그래서 화면은 마지막 상태(예: "실행 중")로 굳고 상태줄은 "N초째 상태가 오지 않습니다"만 보인다(`gui/state.py:86-88`, implementation-readiness 확인). 바꿀 "fault 문장"이 화면에 나오지 않는다.
  - fault 종류도 정해져 있지 않다. 상태 루프에서는 `note_comm(False)`(`arm_service.py:255`)로 `comm_lost`가 되고, 작업 중이면 제어 틱의 `write_joints` 예외로 `internal_error`가 될 수 있다(`arm/motion.py:38,268-274`, implementation-readiness 확인). `internal_error`는 [정지 해제]로 풀 수 있다.
  - `move_lock()`은 `real is None`이면 None이라(`backend.py:282-283`) 시뮬레이션 버튼은 잠기지 않는다. [다시 연결]은 `comm_lost()`가 `self.real`을 요구해(`backend.py:229-230`) 시뮬레이션에서는 이미 보이지 않으므로 "끈다"는 할 일이 없다.
  - `<이유>`는 비공개 `_failed`(`mujoco_driver.py:44`)에만 있다. GUI 세션은 `arm/`을 고칠 수 없다. 예외 문장을 그대로 넣으면 "물리 시뮬레이션이 멈췄습니다: 물리 시뮬레이션이 멈췄습니다: …"가 된다(style).
- **고칠 문장:**
  - A13.1 600행(팔, M4) 내용 칸 끝에 더한다: "`MuJoCoDriver.failed_reason`(읽기 전용 속성. 물리가 멈췄으면 `_failed`의 이유 문자열, 아니면 None)을 더한다. 시험 `test_failed_reason`: `qpos`에 NaN을 넣고 `advance` 뒤 값이 '관절값이 NaN'이다."
  - G443 행을 바꾼다: "물리 멈춤 | 물리가 멈추면 `read_joints`가 예외를 내 상태 콜백이 돌지 않으므로, 상태로 감지하지 않고 드라이버 속성으로 감지한다. `Backend.physics_failed()` = `getattr(self.sim.driver, "failed_reason", None)`. 값이 있으면 ① `move_lock()`이 `real is None`일 때도 '물리 시뮬레이션이 멈췄습니다: <failed_reason>. 앱을 다시 시작하세요'를 돌려줘 기존 잠금 띠와 움직이는 버튼 잠금이 이 문장을 쓴다, ② `app.refresh`의 상태 배지는 '오류', [정지 해제]는 숨긴다, ③ `_run_on`은 결과 문장 대신 이 문장을 돌려준다. [다시 연결]은 시뮬레이션에서 원래 보이지 않으므로 바꾸지 않는다. M4에서는 GUI 안에서 다시 연결하는 기능을 만들지 않는다 |"
  - G 시험 목록에 더한다: "7. `failed_reason`을 monkeypatch로 채우면 `move_lock()`이 '물리 시뮬레이션이 멈췄습니다'로 시작하고, 집어 옮기기가 그 문장을 돌려준다."
  - G419 쪽 파일 목록(A602)에 `gui/app.py`는 이미 있으므로 그대로 둔다.

**B2. "그 버튼과 3D 끌기는 끈다"가 집기 점 손잡이를 끄게 만든다 (G438)**
- **리뷰어:** implementation-readiness·codebase-details(둘 다 Blocker). 종합기가 코드로 확인했다.
- **문제:**
  - 3D에는 물체 끌기가 없다. 3D의 끌기는 `/handle` transform controls(`scene3d.py:181-184`) 하나이고, `_on_drag`(`:326-338`)가 직접 조작의 집기 목표 점을 고른다(`gui/pages/manual.py:191`, implementation-readiness 확인).
  - 문장대로 끄면 MuJoCo에서 직접 조작의 목표 점을 3D로 고를 수 없게 된다.
  - 물체 편집은 2D 평면도 클릭과 버튼(`gui/sensors/manual_view.py:101-163`)에서 들어오는데, 이 파일은 A602·G12.8 어디에도 없다.
- **고칠 문장 (G438 둘째 문장):** "그 조작은 `gui/sensors/manual_view.py`에서 끈다: `_physical()`이면 [처음 물체로]·[지우기]·[고른 물체에 이 높이]·조금씩 패드·'클릭하면' 토글을 끄고, 작업대 클릭(`clicked`)은 첫 줄에서 거부 문장을 알리고 끝낸다. 3D에는 물체 끌기가 없고, 집기 점 손잡이(`/handle`)는 그대로 둔다."
- A602 파일 목록에 `gui/sensors/manual_view.py`를 더한다.

**B3. GUI가 쓰는 유일한 물체 파일 `config/manual_objects.yaml`의 기본 물체가 서로 겹친다 (G434, A13.5)**
- **리뷰어:** codebase-details(Blocker). 종합기가 YAML 값과 모델 코드로 확인했다.
- **문제:**
  - G434(결정 ②)로 MuJoCo GUI의 물체는 늘 `config/manual_objects.yaml`(`sim_driver.py:42-44`)이다.
  - 물체는 `angle_deg`로 돌린 상자이고 길이 축이 각도 방향이다(`mujoco_model.py:156-158`). 물체끼리 충돌한다(충돌을 끄는 것은 장면 부품뿐, `:179`).
  - 지금 값(`config/manual_objects.yaml:5-7`): obj-1 (300,190) 0° 100×30 → x 250–350, y 175–205. obj-3 (350,200) 90° → x 335–365, y 150–250. 두 상자가 x 335–350, y 175–205에서 겹친다. obj-2 (250,210) 45°도 obj-1과 겹친다(codebase-details 계산).
  - `make_sim_driver`에는 겹침 검사가 없다(`sim_driver.py:11-26`은 이름 중복만 본다). MuJoCo는 첫 스텝에서 상자들을 밀어내고, 경고 카운터가 늘면 바로 `SIM-PHYSICS-FAIL`이 날 수도 있다.
  - G438은 "작업이 집는 좌표는 `ManualSource`의 처음 좌표"라 하므로, 시작부터 물리 위치와 어긋난 자리를 집는다. 12.8 시험 4와 13.7 (f) 손 확인(같은 물체가 함께 움직인다)이 첫 실행부터 틀린 결과를 낸다. M3의 CLI `run --source manual --sim-engine mujoco`에도 같은 문제가 이미 있다.
- **고칠 문장 (A13.1에 팔 M4 행, A13.5 끝에도 한 줄):**
  - "`config/manual_objects.yaml`(팔, M4): 기본 물체가 서로 겹치지 않게 고친다(지금 obj-1·obj-3, obj-1·obj-2가 겹친다). `make_sim_driver`는 물체 바닥 사각형이 서로 겹치면 '<파일>의 <이름>과 <이름>이 겹친다'로 `ValueError`를 낸다(시나리오에도 같이)."
  - "`test_sim_state_size`는 connect 1초 뒤 manual 물체마다 `sim_state()` 위치가 파일 값과 3mm 안이고 `tilt_deg` < 5인지도 본다."
  - G434 끝에 더한다: "GUI 쪽 `ValueError`는 시작 실패 행대로 '연결 실패: <문장>', 종료 코드 1이다."

### 🟡 Major

**M1. 편집 거부가 사용자에게 어떻게 전해지는지 없다 (G438, G450)**
- **리뷰어:** implementation-readiness·codebase-details
- **문제:** `add_object`는 id 문자열을, 나머지 다섯은 None을 돌려준다(`backend.py:665-715`). 부르는 쪽(`manual_view.py:104,109,117,122,127,160`)은 돌려준 값을 보지 않고 새로 그린다. "거부한다"만으로는 문장이 화면에 나오지 않고, 시험 3도 확인할 것이 없다.
- **고칠 문장 (G438):** "`backend.py`에 `OBJECTS_LOCKED_TEXT = "MuJoCo 시뮬레이션은 물체를 시작할 때 놓습니다. 바꾸려면 앱을 다시 시작하세요"`를 두고, 여섯 함수는 `_physical()`이면 첫 줄에서 `raise ValueError(OBJECTS_LOCKED_TEXT)`. `manual_view.py`는 B2대로 미리 막고 이 문장을 칸 위에 보인다. 시험 3은 `pytest.raises(ValueError)`와 `objects()` 불변을 본다."

**M2. `held`가 `set_sim_objects`에 오지 않고, 색 갱신이 막혀 있다 (G439-440)**
- **리뷰어:** 세 리뷰어 모두
- **문제:** 콜백은 `sim_state()["objects"]`만 넘기는데 `held`는 `sim_state()` 맨 위 키다(A725, `mujoco_driver.py:285`). G440 "그 뒤에는 위치·방향만 바꾼다"는 `held`·`tilt_deg`에 따른 색 바꾸기를 막는다. 콜백을 거는 자리와 이름도 없어 시험 4가 부를 대상이 없다.
- **고칠 문장 (G439):** "`backend.py:185`의 `if viser_port is not None:` 블록 안에서 `_physical()`이면 `self.sim.status_stream(self._on_sim_objects)`를 건다. `_on_sim_objects(_status)`는 `driver.connected`가 참이고 `failed_reason`이 없을 때만 `s = driver.sim_state()`를 읽어 `Scene3D.set_sim_objects(s["objects"], s["held"])`를 부른다."
- **고칠 문장 (G440):** "처음 한 번 만들고, 그 뒤에는 위치·방향·색만 바꾼다. 보통은 `theme.MUTED`, `tilt_deg` ≥ 10이면 `theme.WARN`, 이름이 `held`와 같으면 `theme.ACCENT`다." (색 상수 이름은 `gui/theme.py`의 실제 이름으로 맞춘다.)

**M3. 계획 쪽 물체 반투명과 레이어 함께 켜기에 필요한 Scene3D 변경이 없다 (G441)**
- **리뷰어:** implementation-readiness·codebase-details
- **문제:** `set_objects`(`scene3d.py:279`)에는 불투명도 인자가 없고 Scene3D는 물리 여부를 모른다. `set_layer("objects")`(`:267-274`)는 `_objects_root` 하나만 바꾼다.
- **고칠 문장 (G441):** "`Scene3D.plan_opacity`(기본 1.0)를 두고 `_physical()`이면 `Backend`가 0.4로 둔 뒤 `set_objects`를 부른다. `set_objects`는 `add_box(opacity=self.plan_opacity)`로 그린다. `set_sim_objects`는 처음에 `self._sim_root = add_frame("/sim_objects", visible=self.layers["objects"])`를 만들고, `set_layer`의 objects 가지는 두 루트를 함께 바꾼다."

**M4. 상자 크기의 축과 단위가 흐리다 (G440)**
- **리뷰어:** style(Major), implementation-readiness·codebase-details(Nit)
- **문제:** `size_mm`의 길이 축이 `yaw_deg` 방향이라는 말이 G12.8에 없어 상자를 90° 돌려 그릴 수 있다. "높이/2(작업 면 위, m)"는 단위 변환인지 기준면인지 흐리다.
- **고칠 문장:** "크기는 `size_mm`([길이, 폭, 높이] mm)이고, 길이 축을 `yaw_deg` 방향에 둔다(`set_objects`와 같은 순서). 상자 가운데는 (`x_mm`, `y_mm`, `z_mm` + 높이/2) mm이고, 그릴 때 1000으로 나눠 m로 바꾼다."

**M5. [시뮬레이션으로 다시 시작]에 엔진 인자를 붙이는 규칙이 성립하지 않는다 (G445)**
- **리뷰어:** 세 리뷰어 모두
- **문제:** 11절 A-3의 그 버튼은 2차 일이고(G154) `gui/`에 아직 없다. 실물 모드에서 누르는 버튼인데 실물 모드에는 `--sim-engine mujoco`가 있을 수 없다(`cli/__init__.py:132-134`). 반대로 [실물로 다시 시작]에 이 인자를 붙이면 종료 코드 1이다.
- **고칠 문장 (G445 둘째 문장):** "11절 A-3의 다시 시작 버튼은 M4에서 만들지 않는다. 2차에서 만들 때 [실물로 다시 시작]은 `--sim-engine`·`--sim-viewer`를 빼고 띄우고, [시뮬레이션으로 다시 시작]은 `so101.cmd --sim gui`(기구학 엔진)를 띄운다."

**M6. 파일 경계가 덜 정해졌다 (A602, A13.1)**
- **리뷰어:** implementation-readiness·codebase-details
- **문제:**
  - `gui/sensors/manual_view.py`가 빠졌다(B2·M1).
  - G444의 "저장했습니다…"는 `gui/pages/table.py:54`가 돌려받은 문자열을 모두 `negative` 알림으로 띄우므로 빨간 오류로 보인다.
  - A13.1에 `tests/test_mujoco_driver.py`의 M4 행이 없다(A773-777 시험이 들어갈 곳).
- **고칠 문장:**
  - A602 파일 칸: "`gui/__main__.py`, `gui/app.py`, `gui/backend.py`, `gui/scene3d.py`, `gui/sensors/manual_view.py`, `gui/pages/table.py`, `tests/gui/test_mujoco_gui.py`"
  - G444 끝: "이 문장은 `table.py`에서 `info` 알림으로 띄운다(`save_workspace`는 `(문장, "info")`를 돌려준다)." 빨간 알림을 받아들인다면 그렇게 적는다.
  - A13.1에 행 더하기: "| `tests/test_mujoco_driver.py`(M4) | 13.7 (f) 팔 시험 | M4 | 팔 |"

**M7. 사용자 문장 용어 규칙과 실제 문장이 다르다 (A574, G443, A734)**
- **리뷰어:** style
- **문제:** A574는 "사용자에게 보이는 문장은 'MuJoCo 시뮬레이션'으로 쓴다"인데, G443과 이미 들어간 A13.2·A734 문장은 "물리 시뮬레이션…"이다.
- **고칠 문장 (A574 둘째 문장):** "사용자에게 보이는 새 문장은 'MuJoCo 시뮬레이션'으로 쓴다. M2에 들어간 '물리 시뮬레이션이 멈췄습니다'(13.2)와 '물리 시뮬레이션에서는 --dry-run을 쓰지 않는다'(13.6)는 그대로 두고, GUI의 물리 멈춤 문장은 13.2 문장을 그대로 다시 쓴다."

**M8. 팔 M4와 GUI M4의 순서와 REQ가 없다 (A419, G440)**
- **리뷰어:** implementation-readiness
- **문제:** GUI 시험 4·`set_sim_objects`는 `size_mm`이 있어야 하는데 지금 코드에는 없다(`mujoco_driver.py:267-268`). B1의 `failed_reason`, B3의 겹침 검사도 팔 일이다. `session-requests.md`에 M4 REQ가 없다.
- **고칠 문장 (A419 끝):** "순서: 팔 M4 PR(`size_mm`·`failed_reason`·뷰어·CLI·물체 파일)이 먼저 합쳐진 뒤 GUI M4를 시작한다. 총괄이 팔 M4 REQ를 `session-requests.md`에 올린다."

### 🟢 Nit

- n1 (G434) "`getattr(args, "sim_scenario", None)`이 있으면 `WatchdogLink`를 만들기 전(`app.py:380` 앞)에 종료한다." `gui/__main__.py`는 `--sim-scenario`가 없어 argparse가 종료 코드 2로 거부한다는 것도 적는다. CLI의 `--sim-scenario` 검사(`cli/__init__.py:130`)는 `--sim --sim-engine mujoco`면 통과해 `gui.main(a)`까지 오므로 `app.main` 검사는 필요하다(두 리뷰어 확인).
- n2 (G437) "시뮬레이션에는 잡히는 물체가 없어"는 193행에만 있다. 516행 주석은 "시뮬레이션 팔은 쥘 물체가 없어…"이고 `programs.py:146`에도 비슷한 주석이 있다. 고칠 문장: "`backend.py` 193·516행 주석을 고친다(`programs.py:146` 주석은 그대로)."
- n3 (G432) `--sim-engine`·`--sim-viewer` 검사를 `gui/__main__.py`와 `app.main`에 둘 다 두지 말고 `app.main` 한 곳에서 하면 두 입구를 함께 덮는다. 고칠 문장: "두 검사는 `app.main` 첫 줄에서 `getattr`로 한다."
- n4 (G437, N9) "517행", "591행"은 arm-software-spec 행과 헷갈린다. "`backend.py:517`", "`backend.py:591`"로 쓴다.
- n5 (G433) 생성자 기본값과 import가 없다. 고칠 문장: "`Backend.__init__(..., sim_engine: str = "kinematic", sim_viewer: bool = False)`. `backend.py:16`을 `from ..arm.sim_driver import SimDriver, make_sim_driver`로 하고 `from ..core.scene import load_scene`을 더한다. `load_scene()`의 `ValueError`는 '연결 실패'로 끝난다(CLI와 같음)." 기존 `Backend(log_root=None)` 호출(`tests/gui/conftest.py:15`)이 그대로 돈다.
- n6 (G448, 시험 1) 정면이 아닌 점에서 높이 0이면 `height_required`가 먼저 나온다(결정 0052). 호출을 적는다: "`pick_place((300, 275), 0, 30, 30, None, height_mm=40)`".
- n7 (G449, 시험 2) "넘긴 판정이 거짓"을 "`self._no_object_ok(self.sim)`이 거짓"으로 쓴다. 시험 4는 `_on_sim_objects(None)`을 직접 부르고 viser 포트는 지금 쓰지 않는 8193을 쓴다(8190–8192 사용 중, `tests/gui/conftest.py:15`, `test_scene3d.py:25`, `test_port_pull.py:68`). 시험 5는 `so101_studio.gui.backend.make_sim_driver`를 monkeypatch한다.
- n8 (A610, A776) "`test_cli_gui_passes_engine`은 `tests/test_cli_share.py:10-12`처럼 가짜 gui 모듈을 쓴다." 아니면 팔 세션 시험이 nicegui를 import한다.
- n9 (A591, A601) A591 칸이 M2·M3·M4를 섞고 "처럼"으로 문장을 흐린다. A591 끝: "M3에 `--sim-scenario`(13.5 문장). `--sim-viewer`는 M4 행." A601에 정확한 문장: "`--sim-viewer`(store_true)를 `--sim --sim-engine mujoco` 없이 주면 '--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다'로 종료 코드 1."
- n10 (A616) `realtime=False`+`viewer=True`의 `ValueError`를 "`MuJoCoDriver.__init__`에서" 낸다고 적는다. (A615) `connect`가 이미 먼저 `self.disconnect()`를 부르므로(`mujoco_driver.py:53`) `handle.close()`는 `disconnect`에만 두면 된다.
- n11 (A613, A617, A791) "GUI의 `ui.run`과 같이 쓸 수 있다"는 아직 확인 전이다: "같이 쓰려는 것이다(아래 첫 작업에서 확인)". 13.9 제목을 "### 13.9 확인 결과 (M1 #260, M4 뷰어)"로 한다.
- n12 (A3, A571, A572, A589, A728, A732, G426, G455) 시제·표기 정리. A3 끝 "13절 M4 부분은 초안(`gui-spec.md` 12.8과 함께 확정)", A571 제목에 "M4 전 점검 2026-11-13 반영", A572 "기구학 시뮬레이션" → "기구학 엔진", A589 "M2는 … 넘겼고 … `scenario`는 M3에서 쓰기 시작했다", A728 "(M4에서 더한다…)", A732 "빈 값이었다", G426 "신규/고침 표기는 12절과 같고 이 절은 모두 신규다", G455 "① M4는 GUI 3D에 물리 물체를 표시한다(결정 0053 상태 줄), ② GUI는 `--sim-scenario`를 막는다(최소안), ③ 쓰러진 방향(회전 행렬)은 M4에 넣지 않는다." `ManualSource`가 작업 폴더 기준 경로(`perception/manual.py:15`)를 읽으므로 "GUI는 레포 루트에서 띄운다(`so101.cmd`)"도 G434에 적는다. A770의 시험 이름 목록에 이미 있는 `test_ranges_widened_and_default_warning`, `test_pending_events_flush_when_sink_is_set`, `test_cli_status_mujoco`가 빠졌다.

## 범위 밖

- `tests/gui/test_port_pull.py:215,240`이 없는 `diagnostics.SimDriver`를 monkeypatch한다(1라운드와 같음, `service/diagnostics.py:17`). GUI 세션이 먼저 확인할 일이다.
- `docs/planning/work-queue.md` 29·45행이 M4를 아직 "13.6"으로 가리킨다(style). "`gui-spec.md` 12.8, arm-software-spec 13.1 M4 행"으로 바꾼다.

## 꼭 고칠 것 (M4 시작 전)

1. **B1**: 팔 M4에 `MuJoCoDriver.failed_reason`을 더하고, G443을 "상태 콜백이 아니라 `failed_reason`으로 감지 → `move_lock()`·배지·`_run_on`"으로 바꾼다.
2. **B2**: G438의 "3D 끌기는 끈다"를 지우고 `gui/sensors/manual_view.py`에서 끄는 것으로 바꾼다. A602에 그 파일을 더한다.
3. **B3**: `config/manual_objects.yaml` 기본 물체 겹침을 고치고, `make_sim_driver`에 겹침 검사를 더한다(팔 M4).
4. **M1·M2·M3**: 거부는 `ValueError(OBJECTS_LOCKED_TEXT)`, 콜백은 `set_sim_objects(objects, held)`, 반투명·레이어는 `plan_opacity`·`_sim_root`로 적는다.
5. **M5·M8**: 다시 시작 버튼 규칙을 2차로 미루고, 팔 M4 PR이 GUI M4보다 먼저라고 A419에 적는다.

## 확인하지 않은 것

- 시험과 모델은 아무것도 돌리지 않았다. B1의 화면 결과("N초째 상태가 오지 않습니다", `internal_error` 경로)는 리뷰어의 코드 읽기이고, 종합기는 `_status_loop`·`read_joints`·`move_lock`·`comm_lost`만 직접 열었다.
- B3의 겹침은 YAML 값으로 계산했고, MuJoCo가 실제로 경고 카운터를 올리는지는 돌려 보지 않았다.
- viser `add_box`의 `opacity` 인자, `gui/theme.py`의 색 상수 이름, `mujoco.viewer.launch_passive`가 Windows의 메인 아닌 스레드와 NiceGUI·viser 옆에서 도는지는 확인하지 않았다(뷰어는 A617이 첫 작업으로 둔다).
- `session-requests.md`의 M4 REQ 유무는 implementation-readiness가 읽은 것이다.
