상태: 초안 (EGM Stage 9 결과, arm-software-spec 13절 M4 + gui-spec, M4 전, 2026-11-13)

# MuJoCo 설계서 13절 M4 구현 준비 점검 (M4 전)

**결론: M4 기준 FAIL.** 🔴 4건, 🟡 8건, 🟢 7건이다.

- 가장 큰 문제는 GUI 세션이 따를 문서가 없다는 것이다(R1). `gui-spec.md`에는 MuJoCo가 구현 순서의 한 단어(171행)로만 나온다. arm-software-spec은 GUI 일을 "GUI REQ"로 넘기는데, 그 REQ도 아직 없다.
- 그 밖의 🔴 셋도 GUI 쪽이다.
  - 엔진·시나리오·뷰어 값을 GUI가 어디서 받는지 정해지지 않았다. 결정 0053의 "화면은 그대로"와도 맞지 않는다(R2).
  - GUI의 수동 물체와 MuJoCo 물체가 따로 논다. 602행 규칙만 넣으면 집기가 `no_object` 실패로 바뀐다(R3).
  - 3D 물체 표시를 기존 `set_objects`에 넣으면 작업 이벤트마다 덮인다. `sim_state()`에는 크기도 없다(R4).
- 팔 쪽(`--sim-viewer`)은 방향은 분명하다. 다만 뷰어를 여는 API, 스레드, 잠금, 닫기, 인자 검사가 빠졌다(A1).
- 모두 문장으로 고칠 수 있다. 아래에 붙여 넣을 문장을 적었다. 몇 가지는 기획이 골라야 한다(각 항목에 "결정 필요"로 표시).

## 대상과 리뷰어

- 대상: `<drift>\docs\planning\arm-software-spec.md`(main ec9fd73)의 M4 부분.
  - 320행(작업 순서표 M4 행), 458행(13절 범위 나누기), 13.1 표 467·469·478행과 484행, 13.6 602행(`no_object` 규칙의 GUI 부분), 13.7 (f) 638행, 648행.
  - 13절 나머지(455–654행)는 맥락으로만 읽었다.
  - `<drift>` = `C:\Users\김동현\AppData\Local\Temp\claude\aa7416ddebb701be\599578e8-9a81-4c89-b7db-e2e310eeeebc\scratchpad\drift`
- 함께 본 설계서: `<drift>\docs\planning\gui-spec.md`
- GoldfishContext: `CLAUDE.md`, 결정 0053, 결정 0044
- 행 번호는 따로 적지 않으면 arm-software-spec의 행이다. 코드 근거는 `파일:행`이고, 경로는 `src/so101_studio/` 아래 기준이다.

| 리뷰어 | 기준 | 상태 | M4 판정 |
| --- | --- | --- | --- |
| `egm-implementation-readiness` | implementation-readiness-standard | completed | FAIL (🔴 4) |
| `egm-codebase-details` | implementation-readiness-standard | completed | FAIL (🔴 3) |
| `egm-style` | 없음 | completed | FAIL (🔴 없음, 🟡 7) |

- 세 리뷰어 모두 한 번에 끝났고, 다시 돌린 것은 없다. 셋 다 읽기만 하고 아무것도 실행하지 않았다.
- 판단이 다르면 더 엄한 쪽을 따랐다. R4(3D 표시)는 implementation-readiness가 Blocker, codebase-details가 Major로 봤다. 종합기가 코드를 열어 "그대로 넣으면 덮인다"를 확인하고 🔴로 올렸다.
- 종합기가 직접 확인한 것:
  - `gui-spec.md:171`(MuJoCo는 구현 순서의 한 단어뿐, 다른 곳에 `MuJoCo`·`sim_engine`·`physical_objects` 없음)
  - `gui/backend.py:161`(`SimDriver`를 직접 만듦), `:177`(`ManualSource`), `:183·611·620·663`(`set_objects`를 다시 부름), `:193`(`Studio(dry_run=self.real is None)`), `:517·591`(`arm is self.sim`), `:665-714`(물체 더하기·옮기기·돌리기·높이·지우기·되돌리기는 `ManualSource`만 바꿈)
  - `gui/app.py:372-389`(`sim`만 읽음, `RuntimeError`·`ArmSelectionError`·`PortChoiceError`만 잡음), `gui/__main__.py:7-15`(엔진 인자 없음)
  - `gui/scene3d.py:279-290`(`set_objects`가 `/objects`를 지우고 다시 만듦, 높이 고정 `OBJECT_H_M`, 바닥에 놓음, yaw만)
  - `cli/__init__.py:130-136`(시나리오 검사, `gui` 거부), `:177`(`gui.main(a)`), `:185-193`(엔진 분기, `MUJOCO_MISSING_TEXT`는 `:18`). `--sim-viewer` 인자는 아직 없다.
  - `arm/sim_driver.py:29-45`(`viewer`를 받지만 `MuJoCoDriver`에 넘기지 않음, 시나리오 없으면 `manual_objects.yaml`을 `obj-1`부터)
  - `arm/mujoco_driver.py:26-28`(`scenario` 인자 있음, `self.viewer = viewer  # M4`로 저장만), `:36`(`physical_objects = True`)
  - `service/diagnostics.py:17`, `tests/gui/test_port_pull.py:215,240`(아래 "범위 밖")
  - 결정 0053:10 "GUI는 드라이버를 만드는 곳과 인자 전달만 고친다(화면은 그대로)", 0053:3·30

## 새 지적

### 🔴 Blocker

**R1. GUI 세션이 따를 M4 설계가 `gui-spec.md`에 없다 (320·458·478·602·638·648행)**
- **리뷰어:** implementation-readiness·codebase-details(Blocker), style(Major). 종합기가 확인했다.
- **문제:**
  - `CLAUDE.md`에 따라 GUI 세션은 `gui/`·`tests/gui/`만 고치고 `gui-spec.md`를 따른다.
  - `gui-spec.md`에서 MuJoCo가 나오는 곳은 171행 구현 순서의 "→ MuJoCo"뿐이다. gui-spec 72행 인자 목록에 `--sim-engine`이 없고, 93행은 시뮬레이션 드라이버를 `SimDriver`로만 적는다.
  - arm-software-spec은 GUI 일(인자 전달, `no_object` 규칙, 물체 표시, import 실패, `gui` 거부 해제)을 320·458·478·602·638·648행 여섯 곳에 조각으로 두고 "GUI REQ"로 넘긴다. 478행이 가리키는 13.6에는 `no_object` 규칙만 있다.
  - `session-requests.md`에도 M4 GUI REQ가 없다(REQ-063은 M3).
  - 그래서 GUI 세션은 R2~R4를 스스로 정해야 한다.
- **고칠 문장:**
  - gui-spec 12절 끝에 새 절을 만든다: "### 12.9 MuJoCo 물리 시뮬레이션 (M4, 결정 0053, arm-software-spec 13절). 팔 쪽(드라이버·뷰어·CLI)은 arm-software-spec 13절이 기준이고, 이 절은 GUI 쪽만 정한다." R2~R4, A2~A5의 GUI 문장을 이 절에 넣는다.
  - 478행 표 칸: "`gui/__main__.py`, `gui/app.py`, `gui/backend.py`, `gui/scene3d.py`, `tests/gui/test_mujoco_gui.py` | `gui-spec.md` 12.9가 기준이다 | M4 | GUI(팔 세션이 REQ로 요청)"
  - 320행: "| M4 | 팔: `--sim-viewer`(13.1·A1), `gui` 거부 해제. GUI: `gui-spec.md` 12.9(REQ로 요청) | 13.7 (f) | 0053, 13.1 | |"
  - 13.6 끝에 한 줄: "GUI 쪽(M4)은 `gui-spec.md` 12.9를 따른다. 12.9가 확정되기 전에는 M4의 GUI 작업을 시작하지 않는다."

**R2. GUI가 엔진·시나리오·뷰어 값을 어디서 받는지 정해지지 않았고, 결정 0053과 맞지 않는다 (458·469·484·648행)**
- **리뷰어:** 세 리뷰어 모두(implementation-readiness·codebase-details Blocker). 종합기가 코드로 확인했다.
- **문제:**
  - 지금 흐름: `cli/__init__.py:135-136`이 `gui`를 거부한다. 거부를 지우면 `:177`이 Namespace 전체를 `gui.main(a)`에 넘기지만, `gui/app.py:374`는 `sim`만 읽는다. `gui/__main__.py:7-15`에는 엔진 인자가 없다. `gui/backend.py:161`은 `SimDriver`를 직접 만들고, 드라이버를 넣을 생성자 인자가 없다(`:136-138`).
  - 458행 "GUI에서 MuJoCo를 고를 수 없다"는 화면에서 고르는 장치가 있는 것처럼 읽힌다. 결정 0053:10은 "화면은 그대로"다. 그런데 638행(3D 표시)과 648행("띄운다")은 화면을 바꾸는 일이다.
  - 실제 팔과 같이 띄우는 쌍둥이 시뮬레이션(`backend.py:161`, `real_driver`가 있을 때)의 엔진도 정해지지 않았다. CLI는 `--sim` 없는 `--sim-engine`을 거부하므로(`cli:133-134`) CLI로는 생기지 않지만, `gui/__main__.py`는 따로 파서를 가진다.
  - 484행 거부를 지우는 쪽(팔 세션)과 `tests/test_mujoco_driver.py`의 `test_cli_gui_rejected`(13.7 (e) 637행 목록)를 어떻게 할지 없다.
- **고칠 문장:**
  - 458행: "**범위 나누기**: M1~M3(완료)는 CLI와 시험만 바꿨다. GUI는 M4에서 한 번에 바꾼다(`gui-spec.md` 12.9). GUI의 엔진은 화면이 아니라 시작 인자로만 고른다: `so101 --sim --sim-engine mujoco [--sim-viewer] gui`. MuJoCo는 실제 팔 없는 `--sim` GUI에서만 쓰고, 실제 팔과 같이 띄우는 쌍둥이 시뮬레이션은 늘 `SimDriver`다. M4도 `make_driver`는 그대로 두고 GUI가 `make_sim_driver`를 직접 부른다."
  - 484행: "`gui`는 M3까지 거부한다. M4에서 팔 세션이 이 거부(`cli/__init__.py`의 `a.command == "gui"` 분기)를 지우고, `test_cli_gui_rejected`를 `test_cli_gui_passes_engine`(`gui.main`을 monkeypatch해 받은 `a.sim_engine == "mujoco"`·`a.sim_viewer`를 확인)으로 바꾼다."
  - gui-spec 12.9: "`app.main(args)`는 `sim`이 참이고 `getattr(args, "sim_engine", "kinematic") == "mujoco"`일 때만 `make_sim_driver(cfg, load_workspace(), load_scene(), viewer=getattr(args, "sim_viewer", False))`로 드라이버를 만들어 `Backend(..., sim_driver=드라이버)`로 넘긴다. `Backend.__init__`은 `sim_driver=None`을 받고 161행을 `sim_driver or SimDriver(cfg.poses.get("home"))`로 한다. `gui/__main__.py`에 `--sim-engine {kinematic,mujoco}`·`--sim-viewer`를 CLI와 같은 이름·기본값으로 더하고, `--sim` 없이 주면 CLI와 같은 문장으로 종료 코드 1이다. 화면에는 엔진 고르기를 두지 않는다."
  - (결정 필요) 결정 0053:10 "화면은 그대로"와 3D 물체 표시(R4)가 어긋난다. 0053 머리말(3행)의 "13절이 바로잡는다" 목록에 "M4는 3D에 물리 물체 표시를 더한다"를 더할지 기획이 정한다.

**R3. GUI의 수동 물체와 MuJoCo 물체가 따로 논다. 602행 규칙만 넣으면 집기가 `no_object` 실패로 바뀐다 (602·638행)**
- **리뷰어:** implementation-readiness·codebase-details(Blocker). 종합기가 코드로 확인했다.
- **문제:**
  - MuJoCo 물체는 `connect` 때 한 번 놓인다(`sim_driver.py:38-45`, 13.5 582행). 시나리오가 없으면 `manual_objects.yaml`의 `obj-1`부터다.
  - GUI의 작업·직접 조작·3D는 `ManualSource`(`backend.py:177`)를 쓴다. 물체 더하기·옮기기·돌리기·높이·지우기·되돌리기(`backend.py:665-714`)는 `ManualSource`만 바꾸고 물리 세계는 그대로다. 옮긴 자리로 집으러 가면 물리 물체가 없다.
  - `--sim-scenario`로 GUI를 띄우면 시나리오 물체(`box58` 등)와 `ManualSource`(`obj-N`)는 이름도 자리도 다르다. 자동 작업이 빈 자리를 집는다.
  - 602행대로 `physical_objects`면 `no_object`를 실패로 보게 바꾸면, 위 두 경우 모두 집기가 실패로 끝난다. 13.5의 `reload_workspace` 거부 같은 막는 규칙이 물체에는 없다.
- **고칠 문장 (gui-spec 12.9, 602행 뒤에도 한 줄로 가리킨다):**
  - "M4의 GUI는 `--sim-scenario`를 받지 않는다(`app.main`이 'GUI는 config/manual_objects.yaml 물체만 씁니다'로 종료 코드 1). 그래서 MuJoCo 물체와 `ManualSource`는 같은 파일·같은 이름(`obj-1`, `obj-2`, …)이다."
  - "드라이버의 `physical_objects`가 참이면 수동 입력의 물체 더하기·옮기기·돌리기·높이 바꾸기·지우기·되돌리기는 'MuJoCo 시뮬레이션은 물체를 시작할 때 놓습니다. 바꾸려면 앱을 다시 시작하세요'로 거부하고, 버튼은 끈다."
  - "작업이 집는 좌표는 지금처럼 `ManualSource`의 처음 좌표다. 물리에서 밀린 자리를 따라가지 않는 것은 인식 없는 실물과 같다."
  - (결정 필요) 시나리오를 GUI에서도 쓰고 싶다면, 시나리오 물체로 `ManualSource`를 채우는 길을 따로 정해야 한다. 위 문장은 M4를 가장 작게 하는 안이다.

**R4. 3D 물체 표시를 그대로 구현하면 덮이고, 크기·주기·노드가 정해지지 않았다 (638·320행)**
- **리뷰어:** implementation-readiness(Blocker), codebase-details·style(Major). 종합기가 코드로 확인하고 🔴로 올렸다.
- **문제:**
  - 3D의 물체는 `Scene3D.set_objects`(`scene3d.py:279-290`)가 그린다. 부를 때마다 `/objects`를 지우고 다시 만든다. 높이는 고정 `OBJECT_H_M`, z는 바닥에 붙이고 yaw만 쓴다.
  - `set_objects`는 작업 이벤트(`backend.py:611`), 스캔(`:620`), 물체 편집(`:663`) 때 `ManualSource` 물체로 다시 불린다. `sim_state()` 물체를 같은 함수·같은 노드로 그리면 다음 이벤트에서 계획 쪽 물체로 덮인다. 바꿔 그릴지 함께 그릴지도 정해지지 않았다.
  - `sim_state()`의 `objects`에는 크기가 없다(13.6 589행). 크기는 `MuJoCoDriver.objects`(`mujoco_driver.py:27`)에만 있다. 기울기도 `tilt_deg` 크기뿐이라 쓰러진 상자의 방향은 그릴 수 없다. GUI 세션은 `arm/`을 고칠 수 없다.
  - 읽는 곳·주기·스레드가 없다. `sim_state()`는 물리 잠금을 쥐고 접촉을 모두 돈다. `connect` 전에는 `data`가 None이다.
- **고칠 문장 (gui-spec 12.9):**
  - "드라이버에 `sim_state`가 있으면 `Backend`가 `self.sim.status_stream`(10Hz, `backend.py:185`와 같은 통로)에 콜백을 하나 더 건다. 콜백은 `driver.connected`가 참일 때만 `sim_state()["objects"]`를 읽어 `Scene3D.set_sim_objects(objects, sizes)`(새 함수)를 부른다."
  - "`set_sim_objects`는 `/sim_objects/<이름>` 상자를 처음 한 번 만들고, 그 뒤에는 position·wxyz만 바꾼다. 크기는 `driver.objects`의 `length_mm`·`width_mm`·`height_mm`(0이면 40)이다. 위치는 `x_mm`·`y_mm`, 높이는 `z_mm` + 높이/2(작업 면 위, m), 방향은 `yaw_deg`다. `tilt_deg` ≥ 10이면 경고색으로 칠하고 기울기는 그리지 않는다. `held`인 물체는 ACCENT다."
  - "기존 `/objects`(수동 입력, 계획 쪽 위치)는 그대로 두고, MuJoCo일 때는 반투명으로 그린다. 둘 다 레이어 'objects'로 함께 켜고 끈다."
  - (결정 필요) 쓰러진 상자를 바르게 그리려면 팔 세션이 `sim_state()` `objects`에 `"rot"`(3×3)과 `"size_mm"`을 더해야 한다. M4 범위에 넣을지 기획이 정한다. 넣지 않으면 위 문장(경고색)으로 충분하다.

### 🟡 Major

**A1. `--sim-viewer` 구현이 정해지지 않았다 (466·467·469·638행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - `make_sim_driver`는 `viewer`를 받지만 `MuJoCoDriver`에 넘기지 않는다(`sim_driver.py:45`). `MuJoCoDriver`는 저장만 한다(`mujoco_driver.py:28`). CLI에는 `--sim-viewer` 인자가 아직 없다.
  - 정해지지 않은 것: 쓸 API(`mujoco.viewer`인지 `mujoco-studio`인지. 13.8은 둘 다 설치한다), 여는 시점, `sync()`를 부르는 스레드와 잠금(물리 스레드와 `data`를 같이 쓴다), `connect`가 모델을 새로 만들 때(`:53-56`)와 `disconnect` 때 열린 창, 사용자가 창을 닫을 때, 뷰어의 마우스 힘 주기, `realtime=False`일 때, 인자 검사.
  - GUI에서는 `ui.run`이 메인 스레드를 차지한다(`app.py:395`). 메인 스레드가 필요 없는 방식이어야 한다.
- **고칠 문장 (13.1에 `arm/mujoco_driver.py`·`arm/sim_driver.py` M4 행, 세션: 팔):**
  - "`make_sim_driver`는 `viewer`를 `MuJoCoDriver(..., viewer=viewer)`로 넘긴다. `viewer`가 참이면 `connect`가 모델을 만든 뒤 `mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)`를 연다(메인 스레드가 아니어도 되는 passive 뷰어)."
  - "물리 스레드가 `_lock`을 쥔 채 1/30초마다 한 번 `handle.sync()`를 부른다. 뷰어의 마우스 힘 주기는 쓰지 않는다(sync 전 `data.xfrc_applied`를 0으로)."
  - "`disconnect`와 다시 만드는 `connect`는 먼저 `handle.close()`를 부른다. 사용자가 창을 닫으면(`handle.is_running()`이 거짓) sync만 멈추고 물리는 계속한다. `realtime=False`와 `viewer=True`를 함께 주면 `ValueError('뷰어는 실시간 모드에서만')`이다."
  - "CLI `--sim-viewer`(store_true)를 `--sim --sim-engine mujoco` 없이 주면 '--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다'로 종료 코드 1이다(560행 `--sim-scenario`와 같은 방식)."
  - "M4 첫 작업으로 설치된 mujoco 4.0에서 `mujoco.viewer.launch_passive`가 있고 Windows의 메인 아닌 스레드에서 열리는지 확인해 13.9에 남긴다."

**A2. (f)는 판정할 수 없고, M4 시험 계획이 없다 (638·637행, 13.1 시험 행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:** (f)는 "보인다"로 끝난다. 명령·시나리오·자동/손 확인이 없다. 13.1에 M4 시험 파일 행이 없다. 484행 거부를 지우면 `test_cli_gui_rejected`가 깨진다. 602행 규칙과 R3 거부를 확인하는 시험도 없다.
- **고칠 문장 (638행):**
  - "(f) **보기**(M4). 자동 시험(팔, `tests/test_mujoco_driver.py`): `test_viewer_flag_passed`(`make_sim_driver(..., viewer=True)`의 드라이버 `viewer`가 참, `launch_passive`는 monkeypatch), `test_cli_viewer_requires_engine`, `test_cli_gui_passes_engine`(R2)."
  - "자동 시험(GUI, `tests/gui/test_mujoco_gui.py`, `importorskip("mujoco")`): ① `sim_driver=make_sim_driver(...)`로 만든 `Backend`에서 물체 없는 점의 집어 옮기기가 실패 문장을 돌려주고 놓기를 부르지 않는다. ② `programs.action`에 넘기는 `is_sim`이 거짓이다. ③ `Studio`의 `accept_no_object`가 거짓이다. ④ 수동 물체 더하기가 거부된다(R3). ⑤ 상태 콜백 한 번 뒤 `/sim_objects/obj-1` 노드가 있고 위치가 `sim_state()` 값과 1mm 안이다."
  - "손 확인(`measurements.md`에 기록): `so101.cmd --sim --sim-engine mujoco --sim-viewer gui`에서 직접 조작 집어 옮기기 1회 동안 뷰어 창과 GUI 3D에서 같은 물체가 함께 움직인다."
  - 637행 시험 이름 목록에서 `test_cli_gui_rejected`를 `test_cli_gui_passes_engine`으로 바꾼다.

**A3. 648행 "GUI는 같은 문장을 띄운다"는 어디에 어떻게인지 없고, 지금 코드는 traceback으로 끝난다 (648행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:** `Backend`는 `ui.run` 전에 만들어진다(`app.py:383-395`). 그래서 화면 알림으로는 띄울 수 없다. `app.py:387`은 `RuntimeError`·`ArmSelectionError`·`PortChoiceError`만 잡으므로 `ImportError`와 `make_sim_driver`의 `ValueError`(병렬 그리퍼, `sim_driver.py:37`)는 traceback으로 끝난다. 문장은 `cli/__init__.py:18`의 `MUJOCO_MISSING_TEXT`를 다시 쓸 수 있다.
- **고칠 문장 (648행 끝):** "GUI는 M4에서 `app.main`이 `make_sim_driver`의 `ImportError`를 잡아 `link.stop()` 뒤 `cli.MUJOCO_MISSING_TEXT`를 터미널에 출력하고 종료 코드 2로 끝난다(화면은 뜨지 않는다. 기구학 엔진으로 바꿔 띄우지 않는다). `ValueError`는 '연결 실패: <문장>'으로 종료 코드 1이다."

**A4. 602행 규칙에 식·문구가 없고, 그대로 읽으면 기구학 엔진에서 AttributeError가 난다 (602행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - 고칠 세 곳(`backend.py:517`, `:591` → `programs.py:146`, `:193`)은 코드와 맞다.
  - `SimDriver`에는 `physical_objects`가 없다(`sim_driver.py:48-61`). "드라이버의 `physical_objects`가 참이면"을 속성 접근으로 쓰면 기구학 엔진에서 깨진다. 470행처럼 `getattr`를 적어야 한다.
  - 같은 판정이 세 곳에 따로 생길 수 있다. 실패가 된 뒤 사용자에게 보일 문장이 없다(`MOTION_TEXT`에 `no_object`가 없다, `backend.py:67-76`, codebase-details 확인).
  - 193·516행 주석("시뮬레이션에는 잡히는 물체가 없어")도 맞지 않게 된다.
- **고칠 문장 (602행):** "GUI(M4): `backend.py`에 `_no_object_ok(arm)`을 두고 `arm is self.sim and not getattr(arm.driver, "physical_objects", False)`를 돌려준다. 집어 옮기기(517행)는 `arm is self.sim` 대신 이 함수를 쓰고, `run_program`(591행)은 `programs.action`에 `self._no_object_ok`를 넘긴다. `programs.py`는 고치지 않는다. `_make_studio`(193행)는 `dry_run=self.real is None and not getattr(self.arm.driver, "physical_objects", False)`로 한다. `MOTION_TEXT["no_object"] = "물체를 쥐지 못했습니다"`를 더하고, 193·516행 주석을 맞게 고친다."

**A5. GUI에서 물리가 멈추면 되살릴 길이 없다 (13.2 517행, GUI 범위)**
- **리뷰어:** implementation-readiness
- **문제:** 13.2는 되살리기를 `disconnect`·`connect`로 정했다. GUI의 다시 연결은 실제 팔만 다룬다(`comm_lost()`가 `self.real`을 요구, `backend.py:229-237`, `_connect_real` `:243-265`). `--sim` GUI에서 `SIM-PHYSICS-FAIL`이 나면 사용자는 무엇을 해야 할지 모른다.
- **고칠 문장 (gui-spec 12.9):** "MuJoCo에서 `SIM-PHYSICS-FAIL`이 나면 알림에 '물리 시뮬레이션이 멈췄습니다: <이유>. 앱을 다시 시작하세요'를 띄운다. M4에서는 GUI 안에서 다시 연결하는 기능을 만들지 않는다."

**A6. 478행 파일 목록이 빠지거나 넘치고, "GUI REQ"는 세션 이름이 아니다 (478·320행)**
- **리뷰어:** style, implementation-readiness
- **문제:** 3D를 그리는 `gui/scene3d.py`가 빠졌다(R4). A4대로면 `gui/programs.py`는 고치지 않는다. GUI 시험 파일 행이 없다. 320행 "GUI 인자 전달"은 어떤 인자인지 없다.
- **고칠 문장:** R1의 478행·320행 문장을 쓴다.

**A7. 458행이 끝난 일을 계획처럼 쓰고, M4에서 `make_driver`를 고쳐도 되는지 없다 (458·467행)**
- **리뷰어:** style
- **문제:** "그래서 M3까지는 … 고를 수 없다", "이렇게 하면 … 건드리지 않는다"는 끝난 M1~M3를 앞으로의 일처럼 쓴다. 467행은 `make_driver`를 "그대로 둔다"고 하는데, M4에 대해서는 말이 없다.
- **고칠 문장:** R2의 458행 문장("M1~M3(완료)는 …", "M4도 `make_driver`는 그대로 두고 …")을 쓴다.

**A8. MuJoCo GUI에서 작업대 설정을 저장할 때 안내가 맞지 않는다 (13.5 582행, GUI 범위)**
- **리뷰어:** codebase-details
- **문제:** `backend.py:789-796`이 `reload_workspace`의 MuJoCo 거부(`arm_service.py:456-457`)를 잡으면 "동작이 끝난 뒤 다시 저장하거나"가 섞인 안내를 낸다. MuJoCo에서는 기다려도 적용되지 않는다.
- **고칠 문장 (gui-spec 12.9):** "드라이버의 `physical_objects`가 참이면 작업대 설정 저장 뒤 '저장했습니다. MuJoCo 시뮬레이션은 앱을 다시 시작해야 적용됩니다'만 보인다."

### 🟢 Nit

- n1 (469행) "168행에서"는 낡았다. 지금 엔진 분기는 `cli/__init__.py:185`이고 168행은 `calibrate` 분기다. 고칠 문장: "`main()`의 드라이버 만들기에서 `a.sim and a.sim_engine == "mujoco"`일 때만 `make_sim_driver`를 부르고 … M3에 `--sim-scenario`, M4에 `--sim-viewer`(둘 다 `--sim --sim-engine mujoco` 없이 주면 종료 코드 1, 13.5·A1)."
- n2 (466행) 코드는 `MuJoCoDriver(cfg, ws, *, scene=None, objects=(), realtime=True, viewer=False, scenario=None)`다(`mujoco_driver.py:26`). 559행이 `scenario=`를 넘기므로 466행 서명에 `scenario=None`을 더한다.
- n3 (484행) 사용자에게 보이는 문장에 내부 단계 이름 "M4"가 있다. M4에서 이 문장은 없어지므로(R2) 그대로 두어도 되지만, 남긴다면 "GUI에서는 아직 MuJoCo를 쓸 수 없습니다"로 쓴다.
- n4 (456·600·582·601행) 같은 대상을 "기구학 시뮬레이션", "기구학 엔진", "시뮬레이션 팔"(0053)로, 사용자 문장을 "MuJoCo 시뮬레이션"·"물리 시뮬레이션"으로 섞어 부른다. 13절 머리에 한 줄: "용어: 기구학 엔진 = `--sim-engine kinematic` = `SimDriver`, MuJoCo 엔진 = `--sim-engine mujoco` = `MuJoCoDriver`. 사용자에게 보이는 문장은 'MuJoCo 시뮬레이션'으로 쓴다."
- n5 (gui-spec 154행) 2차 "다시 시작" 버튼이 띄우는 명령이 `so101.cmd [--sim] gui`라 엔진·뷰어 인자가 빠진다. gui-spec 12.9에 "다시 시작할 때 지금의 `--sim-engine`·`--sim-viewer`를 그대로 붙인다"를 더한다.
- n6 (GUI 상태줄) `backend.py:1188`의 "팔" 값은 "시뮬레이션"뿐이다. MuJoCo일 때 "시뮬레이션(MuJoCo)"로 보일지 gui-spec 12.9에 적는다(선택).
- n7 (586행, 범위 밖) "M2는 빈 값, M3에서 채운다"는 끝난 일이다. "M3 #290에서 채웠다"로 고친다.

## 범위 밖 (지금 main 문제일 수 있음)

- **`tests/gui/test_port_pull.py:215,240`이 없는 속성을 monkeypatch한다.** M2의 diagnostics 변경(470행)으로 `service/diagnostics.py`는 `getattr(svc.driver, "simulated", False)`(`:17`)를 쓰고 `diagnostics.SimDriver`는 없다. 그런데 두 시험은 `monkeypatch.setattr(diagnostics, "SimDriver", …)`를 한다. `raising` 기본값이 참이라 AttributeError로 실패할 것이다. 그 줄을 지워도 `PulledPort`는 `SimDriver` 하위 클래스(`simulated=True`)라 linkwatch가 거부된다. 종합기가 코드를 열어 확인했고, 실행은 하지 않았다. 팔·초음파 세션은 `--ignore=tests/gui`로 돌리므로 놓쳤을 수 있다. M4와 상관없이 GUI 세션이 먼저 확인할 일이다. 고칠 방향: 두 줄을 `monkeypatch.setattr(backend.real.driver, "simulated", False)`로 바꾸고 잠금 동작이 그대로인지 본다.

## 꼭 고칠 것 (M4 시작 전)

1. **R1**: `gui-spec.md`에 12.9 "MuJoCo 물리 시뮬레이션(M4)" 절을 만들고, GUI 일을 그 절로 모은다. arm-software-spec 320·478행과 13.6 끝은 그 절을 가리키게 한다. M4 GUI REQ를 `session-requests.md`에 올린다.
2. **R2**: GUI 엔진은 시작 인자로만 고르고(`sim_driver` 주입, `gui/__main__.py` 인자), 쌍둥이는 늘 `SimDriver`라고 적는다. 484행 거부 해제와 시험 교체를 팔 세션 일로 적는다. 결정 0053 "화면은 그대로"와 3D 표시의 관계를 기획이 정한다.
3. **R3**: GUI에서 `--sim-scenario`를 막고, `physical_objects`면 수동 물체 편집을 거부한다.
4. **R4**: 3D는 `/sim_objects` 새 노드로 10Hz 갱신하고, 크기는 `driver.objects`에서 가져온다. 기울기 방향(`rot`)을 M4에 넣을지 기획이 정한다.
5. **A1**: `--sim-viewer`의 API(`launch_passive`), 스레드·잠금, 닫기, 인자 검사를 13.1 M4 행에 적는다.
6. **A2·A3·A4**: (f)를 자동 시험 + 손 확인으로 바꾸고, import 실패는 터미널 출력·종료 코드 2로, 602행은 `getattr` 식 하나와 실패 문장으로 적는다.

## 확인하지 않은 것

- 시험과 모델은 아무것도 돌리지 않았다.
- mujoco 4.0 메타 휠에서 `mujoco.viewer.launch_passive`가 있는지, Windows의 메인 아닌 스레드에서 열리는지, NiceGUI·viser와 한 프로세스에서 같이 도는지는 확인하지 않았다(A1의 첫 작업으로 넣었다).
- `MOTION_TEXT`(`backend.py:67-76`), `comm_lost`(`:229-237`), `save_workspace`(`:789-796`), 상태줄(`:1188`)은 codebase-details·implementation-readiness가 읽은 것이고, 종합기는 다시 열지 않았다.
- `service/studio.py`·`task.py` 내부(`accept_no_object` 전달)는 리뷰어가 깊이 보지 않았다.
- 범위 밖 `test_port_pull.py` 실패는 코드 읽기로만 판단했다.
