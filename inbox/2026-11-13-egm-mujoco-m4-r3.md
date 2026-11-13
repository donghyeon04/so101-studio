상태: 초안 (EGM Stage 9 결과, gui-spec 12.8 + arm-software-spec M4, 3라운드, 2026-11-13)

# MuJoCo M4 구현 준비 점검 3라운드 (gui-spec 12.8 + arm-software-spec M4)

**결론: M4 기준 FAIL.** 🔴 2건, 🟡 7건, 🟢 12건이다.

- 2라운드 🔴 B1·B2는 해결됐다. B3는 검사·시험 문장은 생겼지만, 새로 정한 기본 물체 좌표가 다른 것을 깬다(새 🔴 C1).
- 2라운드 🟡 M1~M8은 모두 문장에 들어갔다.
- 새 🔴 둘은 종합기가 코드와 시험을 직접 열어 확인했다.
  - **C1** A721의 새 obj-3 (400,170,90°)는 배출 구역과 기존 GUI 시험의 놓기 점 (420,120) 옆에 놓인다. 지금 `tests/gui/test_manual.py:253`이 깨지고, 그 파일은 팔 세션이 고칠 수 없다. 그런데 팔 M4 PR이 먼저 합쳐진다(A419).
  - **C2** G436대로 `Backend`에 `sim_engine=`·`sim_viewer=`를 늘 넘기면 `tests/gui/test_pages.py:161·171`이 깨진다. 이것은 시험 6("지금 시험이 모두 그대로 통과")과 모순된다.
- 둘 다 문장 한두 줄로 고칠 수 있다. 아래에 붙여 넣을 문장을 적었다.

## 대상과 리뷰어

- 대상(main ff47875, 폴더 `<m4d>` = `C:\Users\김동현\AppData\Local\Temp\claude\aa7416ddebb701be\599578e8-9a81-4c89-b7db-e2e310eeeebc\scratchpad\m4d`):
  - `docs/planning/gui-spec.md` 12.8절(425–459행). 아래에서는 **G**로 적는다.
  - `docs/planning/arm-software-spec.md`의 M4 부분: 3행, 412–419행(작업 순서표 M4), 571–619행(13절 머리, 13.1 M4 행 600–604, 명령 처리 `gui` 612, 뷰어 613–619), 648–652행(13.2 물리 실패·`failed_reason`), 702–721행(13.5 물체·겹침 검사), 724–741행(13.6), 777–787행(13.7 (f)), 797행. 아래에서는 **A**로 적는다.
- 함께 읽은 코드: `src/so101_studio/gui/`(`__main__.py`, `app.py`, `backend.py`, `programs.py`, `scene3d.py`, `theme.py`, `state.py`, `sensors/manual_view.py`, `pages/table.py`, `pages/common.py`), `arm/mujoco_driver.py`, `arm/sim_driver.py`, `arm/mujoco_model.py`, `arm/skills.py`, `cli/__init__.py`, `service/arm_service.py`, `perception/manual.py`, `config/manual_objects.yaml`·`workspace.yaml`·`scene.yaml`, `tests/gui/`, `tests/test_cli*.py`, `tests/test_mujoco_*.py`. 코드 경로는 `src/so101_studio/` 아래 기준이다.
- GoldfishContext: `CLAUDE.md`, 결정 0053, 2라운드 보고(`runs/egm/2026-11-13-mujoco-m4-r2/implementation_readiness_report.md`), 사용자 승인 결정 셋(11-13: ① 3D에 물리 물체 표시, ② GUI에서 `--sim-scenario` 막기·MuJoCo면 화면 물체 편집 거부, ③ 회전 행렬은 M4에 넣지 않음).

| 리뷰어 | 기준 | 상태 | M4 판정 |
| --- | --- | --- | --- |
| `egm-implementation-readiness` | implementation-readiness-standard | completed (2번째 시도) | FAIL (🔴 1) |
| `egm-codebase-details` | implementation-readiness-standard | completed (2번째 시도) | FAIL (🔴 1) |
| `egm-style` | 없음 | completed | 🔴 없음, 🟡 2 |

- implementation-readiness와 codebase-details는 첫 시도가 사용량 한도(HTTP 429)로 끊겼다. 한도가 풀린 뒤 둘만 한 번씩 다시 돌렸고, 둘 다 끝까지 마쳤다. style은 첫 시도에 끝나 다시 돌리지 않았다.
- 셋 다 읽기만 하고 아무것도 실행하지 않았다. style은 맡긴 범위대로 코드를 열지 않았다.
- 판단이 다르면 더 엄한 쪽을 따랐다.
  - C1: implementation-readiness는 Blocker, codebase-details는 Major(배출 칸 근접)로 봤다. 종합기가 시험과 `skills.py`를 열어 확인하고 🔴로 두었다.
  - C2: codebase-details는 Blocker, implementation-readiness는 Major로 봤다. 종합기가 `test_pages.py`와 `app.py`를 열어 확인하고 🔴로 두었다.
- 종합기가 직접 확인한 것:
  - `tests/gui/test_pages.py:121`(`FakeBackend(..., **real)`), `:161`(`real == {}`), `:171-174`(real 인자 dict와 정확히 같음), `gui/app.py:383-386`(`Backend(...)` 호출, sim이면 `**{}`)
  - `tests/gui/test_manual.py:253`(`pick_place((300,200), …, (420,120))`이 None, 주석에 "기본 수동 물체(350,200, 반지름 50)와 겹치지 않게"), `gui/backend.py:523-528`(수동 물체를 반지름 = max(폭, 길이)/2인 `known_objects`로 팔에 알림), `arm/skills.py:419-421·511-530`(턱 끝·쥔 물체가 알려진 물체 원에 들어가면 `object@(x,y)`)
  - `tests/test_task.py:70-80·160-164`, `tests/test_studio_api.py:72`(`config/manual_objects.yaml`을 직접 읽는 시험. 개수·id만 보거나 셋 다 집어 놓기 ok를 본다)
  - `arm/mujoco_driver.py:115-118`(`_fail`), `:120-137`(`_step_n`: 예외 → "스텝 예외 …", NaN → "관절값이 NaN", 경고 카운터 → "MuJoCo 경고 카운터 증가"), `:173-175`(`_check`)
  - `gui/backend.py:315-316`(`_run_on`은 `@staticmethod`), `gui/scene3d.py:171·174·186·187·284`(루트 노드는 모두 `show_axes=False`)

## 2라운드 지적 반영

| 2라운드 | 상태 | 근거·남은 것 |
| --- | --- | --- |
| 🔴 B1 물리 멈춤 감지 | 해결 | A600·A651에 `failed_reason`, G438③ `physics_failed()`, G446 ①②③, G457 시험 7. 남은 것: `_run_on`이 staticmethod(🟡 D2), 시험 7 monkeypatch(🟡 D3), `test_failed_reason` 기대 문장(🟡 D4). |
| 🔴 B2 3D 끌기 | 해결 | G441이 `/handle`을 그대로 두고 `manual_view.py`에서 끈다. A604·G430에 파일이 들어갔다. 남은 것: 끈 버튼이 다시 켜짐(🟡 D1). |
| 🔴 B3 기본 물체 겹침 | 일부 | A601·A719-721·A783·A785·G437로 검사·시험·종료 코드는 정해졌다. 새 좌표는 서로 34mm 이상 떨어지고 받침·`radar_station`과도 닿지 않는다(codebase-details 계산). 그러나 obj-3이 기존 GUI 시험과 배출 구역을 막는다(🔴 C1). |
| 🟡 M1 편집 거부 알림 | 해결 | G440 `OBJECTS_LOCKED_TEXT`·`raise ValueError`, G441 `warning` 알림, G453 시험 3. |
| 🟡 M2 `held`·색 | 해결 | G442-443. `theme.MUTED`·`WARN`·`ACCENT`는 `theme.py:10-13`에 있다. 색은 hex 문자열이라 `theme.rgb(...)`를 거친다(🟢 p5). |
| 🟡 M3 반투명·레이어 | 해결 | G444. `add_box(opacity=)`는 `scene3d.py:242`에서 이미 쓴다. `/sim_objects`의 축 표시는 🟡 D5. |
| 🟡 M4 크기 축 | 해결 | G443. |
| 🟡 M5 다시 시작 버튼 | 해결 | G448. |
| 🟡 M6 파일 경계 | 해결 | G430, A602, A604, G447(`info` 알림). 다만 G430 목록에 시험 쪽 고칠 파일이 생길 수 있다(🔴 C1·C2). |
| 🟡 M7 용어 | 해결 | A574, G440·G447 "MuJoCo 시뮬레이션", G446이 13.2 문장을 다시 쓴다. A574와 G446의 끝말이 조금 다르다(🟢 p3). |
| 🟡 M8 순서·REQ | 해결 | A419, G426. `session-requests.md`의 REQ 유무는 열어 보지 않았다. |
| 🟢 n1 인자 검사 위치 | 해결 | G434-435. |
| 🟢 n2 주석 | 해결 | G439. 인용 문구는 두 줄이 다르다(🟢 p2). |
| 🟢 n3 검사 한 곳 | 해결 | G435. |
| 🟢 n4 행 번호 표기 | 일부 | G439는 고쳐졌다. G442의 `backend.py:185`는 `if` 줄이 아니다(🟢 p1). |
| 🟢 n5 생성자·import | 해결 | G436. 다만 늘 넘기는 방식이 시험을 깬다(🔴 C2). |
| 🟢 n6 시험 1 호출 | 해결 | G451. |
| 🟢 n7 시험 2·4·5 | 일부 | 포트 8193은 비어 있다. `Studio`에 `dry_run`이 없다(🟢 p4). |
| 🟢 n8 가짜 gui 모듈 | 해결 | A612. |
| 🟢 n9 A591·A601 | 해결 | A591, A603. |
| 🟢 n10 뷰어 `ValueError` 자리 | 해결 | A617-618. |
| 🟢 n11 뷰어 확인 전 | 해결 | A615, A619, A799 제목. |
| 🟢 n12 시제·표기 | 해결 | A3, A571-572, A589, A725, A738, G426, G434(레포 루트), G459, A776. 머리의 라운드 표기는 🟢 p8. |
| 범위 밖 `test_port_pull.py:215,240` | 확인 안 함 | 이번 리뷰어는 다시 보지 않았다. |

## 새 지적

### 🔴 Blocker

**C1. 새 기본 물체 obj-3 (400,170,90°)가 기존 GUI 시험을 깨고 배출 구역 옆에 놓인다 (A721, A601)**
- **리뷰어:** implementation-readiness(Blocker), codebase-details(Major). 종합기가 코드와 시험으로 확인했다.
- **문제:**
  - GUI는 수동 물체를 반지름 max(폭, 길이)/2 = 50mm인 알려진 물체 원으로 팔에 알린다(`gui/backend.py:523-528`). 팔은 놓을 때 턱 끝과 쥔 물체가 이 원에 들어가면 `object@(x,y)`로 거부한다(`arm/skills.py:419-421`, `:511-530`).
  - `tests/gui/test_manual.py:253`은 (420,120)에 놓기가 성공(None)하기를 기대한다. 주석에도 지금 obj-3 (350,200)을 피한 자리라고 적혀 있다. 새 obj-3 (400,170)에서 (420,120)까지는 53.9mm다. 반지름 50 원에 쥔 물체 반지름 약 15를 더하면 겹치므로 이 시험이 실패한다(implementation-readiness 계산).
  - 배출 구역 `right`의 칸 (442.5,112.5)·(442.5,187.5)(`config.py:108-119`, `workspace.yaml:6`)은 obj-3에서 71.5mm·46mm다. 둘째 칸은 원 안이라, GUI 자동 실행(알려진 물체를 넘김)의 둘째 놓기가 `known_object`로 막힐 수 있다. 첫 칸에 길이 100 물체를 x축 방향으로 놓으면 obj-3 바닥과 22.5×7.5mm 겹친다(codebase-details 계산).
  - A419·G426에 따라 팔 M4 PR이 먼저 합쳐진다. 그런데 CLAUDE.md상 팔 세션은 `tests/gui/`를 고칠 수 없다. 그래서 main이 GUI M4까지 깨진 채로 남는다.
  - `config/manual_objects.yaml`은 실물과 기구학 GUI도 쓰므로 영향이 모든 모드에 미친다. `tests/test_task.py:160-164`(기구학으로 셋 다 집어 놓기 ok)도 새 좌표로 다시 돌려 봐야 한다.
- **고칠 문장 (A721 셋째 문장을 바꾼다):** "팔 M4에서 이 값으로 고친다: obj-1 (300, 190, 0°), obj-2 (170, 170, 45°), obj-3 (400, 250, 90°). 모두 100×30, 높이 40이다. 조건: 서로 30mm 넘게 떨어지고, 로봇 원점에서 90~250mm이고, 알려진 물체 원(반지름 = 길이/2)이 배출 구역 칸(`ws.zone_slots`), `tests/gui/test_manual.py:253`의 놓기 점 (420,120), gui-spec 12.8 시험 1의 빈 점 (300,275)에서 반지름 + 17mm 넘게 떨어진다. 팔 M4 PR은 바꾼 뒤 `tests/gui/test_manual.py`·`tests/test_task.py`가 그대로 통과하는지 총괄에게 확인받고, 깨지면 합치기 전에 GUI REQ로 넘긴다. 실제 물체를 놓은 자리가 생기면 같은 조건으로 바꾼다."
  - 참고 계산(두 리뷰어, IK로 돌려 보지는 않음): obj-3 (400,250)은 로봇 원점에서 224mm다. 칸 (442.5,187.5)까지 75.6mm, (420,120)까지 131mm, (300,275)까지 103mm, obj-1과 x로 35mm 떨어진다. 다만 (300,275)는 원 반지름 50 + 17 = 67보다 멀지만 집기 시험 1은 빈 점이어야 하므로 G451과 함께 다시 확인한다.

**C2. `Backend`에 엔진 인자를 늘 넘기면 기존 `test_pages` 시험이 깨진다. 시험 6과 모순이다 (G436, G456)**
- **리뷰어:** codebase-details(Blocker), implementation-readiness(Major). 종합기가 코드로 확인했다.
- **문제:**
  - `tests/gui/test_pages.py:121`의 `FakeBackend`는 모르는 키워드를 `**real`로 받는다. `:161`은 sim 호출에서 `real == {}`, `:171-174`는 실물 호출에서 real dict가 정확히 네 키이기를 본다.
  - G436은 `app.main`이 `Backend(..., sim_engine=engine, sim_viewer=viewer)`를 늘 넘기라고 한다. 그러면 두 키가 `real`에 들어가 두 assert가 모두 실패한다.
  - G456 시험 6은 "기구학 엔진에서는 지금 시험이 모두 그대로 통과한다"이고, `test_pages.py`는 G430 고치는 파일 목록에 없다. 구현자가 시험을 고칠지 호출을 바꿀지 정할 수 없다.
- **고칠 문장 (G436 첫 문장을 바꾼다):** "`app.main`은 `engine = getattr(args, "sim_engine", "kinematic")`, `viewer = getattr(args, "sim_viewer", False)`를 읽고, `engine == "mujoco"`일 때만 `Backend(..., sim_engine=engine, sim_viewer=viewer)`로 넘긴다(`**({"sim_engine": engine, "sim_viewer": viewer} if engine == "mujoco" else {})`). 기구학 엔진과 실물의 호출 인자는 지금과 같아 `tests/gui/test_pages.py:161·171`이 바뀌지 않는다."

### 🟡 Major

**D1. 화면에서 끈 편집 조작이 0.5초 뒤 다시 켜진다 (G441)**
- **리뷰어:** implementation-readiness·codebase-details
- **문제:** [처음 물체로], "클릭하면" 토글, 조금씩 패드, [고른 물체에 이 높이]는 `controls.add`로 등록된다(`manual_view.py:132·142-144·153·163`). `tick()`(`:170-180`)이 0.5초마다 `controls.refresh()`(`pages/common.py:69-72`)를 불러 `enabled = allowed and valid`로 다시 켠다. [지우기]·[고른 물체에 이 높이]는 `:95·:173·:174`에서 따로 다시 계산된다. "끈다"만으로는 버튼이 되살아나고, 누르면 `ValueError`가 알림 없이 핸들러 밖으로 나간다.
- **고칠 문장 (G441 둘째 문장을 바꾼다):** "`page()` 첫머리에서 `locked = backend._physical()`을 둔다. `locked`이면 '클릭하면' 카드와 '고른 물체 조금씩' 카드를 만들지 않고, 그 자리에 `OBJECTS_LOCKED_TEXT` 라벨 하나를 둔다. [처음 물체로]·[지우기]·[고른 물체에 이 높이]는 `controls.add`에 넣지 않고 `.disable()`로 만든다. `:95·:173·:174`의 `enabled` 식에는 `and not locked`를 더한다."

**D2. `_run_on`은 staticmethod라 `physics_failed()`를 부를 수 없다 (G446 ③)**
- **리뷰어:** implementation-readiness·codebase-details. 종합기가 `backend.py:315-316`을 확인했다.
- **문제:** `self`가 없다. 동작 도중에 물리가 멈추면 `result.reason`에 원래 예외 문장이 실려 나온다(`backend.py:331-332`, implementation-readiness 확인).
- **고칠 문장 (G446 ③):** "③ `_run_on`(staticmethod 그대로)은 첫 줄과 `action` 뒤에 `reason = getattr(arm.driver, "failed_reason", None)`을 보고, 값이 있으면 `PHYSICS_FAILED_TEXT.format(reason)`을 돌려준다(동작 전이면 `action`을 부르지 않는다)."

**D3. 시험 7의 monkeypatch가 읽기 전용 속성에서 실패한다 (G457)**
- **리뷰어:** implementation-readiness·codebase-details
- **문제:** A600·A651은 `failed_reason`을 setter 없는 읽기 전용 속성으로 정했다. 인스턴스에 `monkeypatch.setattr(driver, "failed_reason", …)`을 하면 `AttributeError`다.
- **고칠 문장 (G457):** "7. `monkeypatch.setattr(backend.sim.driver, "_failed", "시험 이유")`로 채우면(물리 스레드도 멈춘다, `mujoco_driver.py:147`) `move_lock()`이 '물리 시뮬레이션이 멈췄습니다: 시험 이유'로 시작하고, `pick_place(...)`가 그 문장을 돌려준다."

**D4. `test_failed_reason`의 기대 문장 "관절값이 NaN"이 맞지 않을 수 있다 (A784)**
- **리뷰어:** implementation-readiness(Major). codebase-details는 `_fail("관절값이 NaN")`이 코드에 있다고만 확인했다.
- **문제:** MuJoCo `mj_step`은 시작할 때 qpos가 나쁘면 `mjWARN_BADQPOS`를 올리고 데이터를 리셋한다(autoreset). 그러면 `_step_n`(`mujoco_driver.py:134-137`)이 볼 때 qpos는 이미 유한하고, 이유는 "MuJoCo 경고 카운터 증가"가 된다. 기존 `test_physics_fail_event`는 이유 문장을 보지 않아 드러나지 않았다. MuJoCo 동작은 리뷰어의 기억이고 돌려 보지 않았다.
- **고칠 문장 (A784):** "`test_failed_reason`: 정상이면 None이다. `qpos`에 NaN을 넣고 `advance` 뒤에는 None이 아니고, `SIM-PHYSICS-FAIL` 이벤트의 `reason`과 같다. 다시 `connect`하면 None이다."

**D5. `/sim_objects` 루트에 좌표축이 그려진다 (G443)**
- **리뷰어:** codebase-details
- **문제:** 기존 루트 노드는 모두 `show_axes=False`다(`scene3d.py:171·174·186·187·284`). G443 문장대로면 viser 기본값으로 작업대 원점에 축이 보인다. `set_layer`가 처음 부르기 전(`_sim_root`가 없을 때)도 정해져 있지 않다.
- **고칠 문장 (G443 첫 문장):** "처음 부를 때 `self._sim_root = scene.add_frame("/sim_objects", show_axes=False, visible=self.layers["objects"])`를 만든다. `Scene3D.__init__`에 `self._sim_root = None`을 두고, `set_layer`는 None이 아닐 때만 바꾼다."

**D6. G446 칸 하나에 규칙이 너무 많다 (G446)**
- **리뷰어:** style
- **문제:** 감지 이유, 문장 상수, 효과 ①②③, 바꾸지 않는 것 둘이 한 칸에 이어져 있어 무엇이 어디에 걸리는지 바로 보이지 않는다. D2를 반영하면 더 길어진다.
- **고칠 문장:** 칸은 "`physics_failed()`로 감지한다(상태 콜백은 돌지 않는다). 문장과 효과는 표 아래 '물리 멈춤'."으로 줄이고, 표 아래에 목록을 둔다: 문장(`PHYSICS_FAILED_TEXT`), `move_lock()`, 페이지 `refresh()`, `_run_on`(D2), 바꾸지 않는 것.

**D7. 겹침 오류 문장의 조사가 이름에 따라 틀리고, 조건과 문장이 다르다 (A719-720, A785)**
- **리뷰어:** style
- **문제:** "<이름>과 <이름>이"에 obj-2를 넣으면 "obj-2과"가 된다. 사용자에게 보이는 문장이다(CLI 종료 코드 1, GUI "연결 실패"). 검사는 "겹치거나 닿으면"인데 문장은 "겹친다"만 말한다. `<파일>`이 전체 경로인지 이름인지도 없다(codebase-details).
- **고칠 문장 (A720):** "`make_sim_driver`는 시나리오와 `config/manual_objects.yaml` 모두에서 두 물체의 바닥 사각형이 겹치거나 닿으면(분리축 검사) '<파일 이름>: 물체 <이름>, <이름>의 바닥이 겹치거나 닿습니다'로 `ValueError`를 낸다(`<파일 이름>`은 `path.name`). CLI는 종료 코드 1, GUI는 '연결 실패: <문장>'(gui-spec 12.8)이다." A719 "겹치면" → "겹치거나 닿으면", A785 "겹치는" → "겹치거나 닿는".

### 🟢 Nit

- p1 (G442) `if viser_port is not None:`은 `backend.py:181`이다(185는 그 안의 `status_stream(self.scene.update_status)` 줄). "`backend.py:181`의 `if viser_port is not None:` 블록 안(`:185` 다음)"으로 쓴다. `driver`도 "`self.sim.driver`"로 쓴다(style).
- p2 (G439) 두 주석의 실제 문구가 다르다: 193행 "시뮬레이션에는 잡히는 물체가 없어", 516행 "시뮬레이션 팔은 쥘 물체가 없어". "두 주석의 '시뮬레이션'을 '기구학 엔진'으로 고친다."
- p3 (A574) "13.2 문장을 그대로 다시 쓴다"와 G446(뒤에 ". 앱을 다시 시작하세요"를 붙임)이 조금 다르다. "13.2 문장 뒤에 '. 앱을 다시 시작하세요'를 붙여 쓴다(gui-spec 12.8 `PHYSICS_FAILED_TEXT`)."
- p4 (G452 시험 2) `Studio`에는 `dry_run` 속성이 없다(`studio.py:18-22`). "`backend.studio.task.accept_no_object`가 거짓이다."
- p5 (G443) 테마 상수는 hex 문자열이다. "색은 `theme.rgb(theme.MUTED)`처럼(`set_objects`와 같게)". 만든 뒤 상자 `color`를 바꿀 수 있는지(설치된 viser 버전)는 확인하지 않았다. 안 되면 다시 만든다고 적는다.
- p6 (G446 ②) `app.refresh`는 모듈 함수가 아니라 `app.py:136`의 페이지 안 `refresh()`다. "배지 식(`:176`)에 `or backend.physics_failed()`, [정지 해제](`:185`)에 `and not backend.physics_failed()`."
- p7 (G448) "팔" 값의 자리: "`Backend.readout()`(`backend.py:1184`)의 '팔' 값".
- p8 (G425, A571) 머리 표기: G425 "M4 1~3라운드 반영", A571 "결정 0053 EGM 2라운드 반영, M4 전 점검 2026-11-13 1~3라운드 반영"(M4 라운드와 헷갈리지 않게).
- p9 (G435 ①) 거부 문장이 무엇을 거부하는지 없고 어미가 다른 둘과 다르다. "--sim-scenario는 gui에서 쓰지 않는다(GUI는 config/manual_objects.yaml 물체만 쓴다)".
- p10 (G454 시험 4) "1mm 안"의 비교 대상: "노드 위치 × 1000이 (`x_mm`, `y_mm`, `z_mm` + 높이/2)와 1mm 안이다." (G455 시험 5) `test_pages.py:141`처럼 `WatchdogLink`를 monkeypatch한다고 적는다.
- p11 (A783) `test_sim_state_size`의 "시나리오 값"이 어느 시나리오인지 없다. "`carry30`으로 connect하면 `box`의 `size_mm`가 [30, 30, 40]이다."를 앞에 둔다.
- p12 (A650, A721, A577) A650 "지금 통신 끊김 처리로 정지"는 작업 중 `internal_error`도 될 수 있다(2라운드 B1). "GUI는 이 구분에 기대지 않고 `failed_reason`으로 감지한다"로 쓴다. A721의 "안(…)"은 C1 문장으로 바뀐다. A577 명령 표기를 G434와 같은 `so101.cmd --sim --sim-engine mujoco [--sim-viewer] gui`로 맞춘다.

## 범위 밖

- 물리 멈춤과 상태 루프 사이에는 모순이 없다(implementation-readiness 확인). 상태 루프는 `note_comm(False)`만 반복하고(`arm_service.py:254-257`), GUI의 `comm_lost()`는 `self.real`이 있어야 참이다(`backend.py:229-230`). 상태줄에 "N초째 상태가 오지 않습니다"가 함께 보이는 것은 남는다.
- `tests/gui/test_port_pull.py:215,240`(없는 `diagnostics.SimDriver` monkeypatch)는 이번에 다시 보지 않았다.

## 꼭 고칠 것 (M4 시작 전)

1. **C1**: A721 기본 물체 좌표를 조건(배출 칸·시험 놓기 점과 거리)과 함께 다시 정하고, 팔 M4 PR이 `tests/gui/test_manual.py`·`tests/test_task.py`를 확인받게 적는다.
2. **C2**: G436을 "`engine == "mujoco"`일 때만 엔진 인자를 넘긴다"로 바꾼다.
3. **D1~D5**: 화면 잠금은 `locked`로 만들 때부터 끄고, `_run_on`은 `arm.driver.failed_reason`을 직접 읽고, 시험 7은 `_failed`를 채우고, `test_failed_reason`은 이벤트의 `reason`과 비교하고, `/sim_objects`는 `show_axes=False`로 만든다.

## 확인하지 않은 것

- 시험과 모델은 아무것도 돌리지 않았다. C1의 실패는 `skills.py`의 원 검사로 계산한 것이고, 집게 끝 위치·놓기 방향은 돌려 보지 않았다. 대안 좌표 (400,250)도 IK로 확인하지 않았다.
- D4의 MuJoCo autoreset 동작은 리뷰어의 기억이다.
- viser `add_frame` 기본 축 표시·상자 `color` 변경, `WatchdogLink` 생성자가 포트를 여는지, `session-requests.md`의 M4 REQ 유무는 확인하지 않았다.
