상태: 초안 (EGM Stage 9 결과, arm-software-spec 13.5·13.6·13.7 (a)(b), M3 전, 2026-11-13)

# MuJoCo 설계서 13.5·13.6·13.7 (a)(b) 구현 준비 점검 (5라운드, M3 전)

**결론: M3 기준 FAIL.** 🔴 1건, 🟡 11건, 🟢 14건이다.

- 🔴는 `carry` 시나리오 하나다(R1). 시나리오 파일은 하나인데 같은 점에 상자 셋을 놓으라고 하고, 상자 길이를 적지 않아 코드 기본값 100mm 막대가 놓인다.
- 4라운드에서 "M3 전"으로 넘긴 9개 가운데 반영된 것은 없다(부분 1, 미반영 8).
- 그 밖에 (a)(b) 시험을 쓰려면 구현자가 정해야 하는 것이 남아 있다: 호출할 API와 `DetectedObject` 인자 이름, 시나리오를 읽는 곳, 미끄러짐을 잴 손 자세, 감시할 이벤트 코드.
- 모두 문장 몇 줄로 고칠 수 있다. 아래에 붙여 넣을 문장을 적었다.

## 대상과 리뷰어

- 대상: `<mj5>\docs\planning\arm-software-spec.md`(main 6e5fdc0)의 13.5(550–570행), 13.6(572–584행), 13.7 (a)(b)(587–599행). 13절 나머지(455–633행)는 맥락으로만 읽었다.
  - `<mj5>` = `C:\Users\김동현\AppData\Local\Temp\claude\c91db8b570d4acfb\599578e8-9a81-4c89-b7db-e2e310eeeebc\scratchpad\mj5`
  - M2까지 합친 main이다.
- GoldfishContext: 결정 0053, `CLAUDE.md`, 3라운드 보고서(`runs/egm/2026-11-12-mujoco-r3/`), 4라운드 보고서(`runs/egm/2026-11-12-mujoco-r4/`)
- 행 번호는 모두 대상 파일의 행이다. 코드 근거는 `파일:행`이고, 경로는 `src/so101_studio/` 아래 기준이다.

| 리뷰어 | 기준 | 상태 | M3 판정 |
| --- | --- | --- | --- |
| `egm-implementation-readiness` | implementation-readiness-standard | completed | FAIL (🔴 1) |
| `egm-codebase-details` | implementation-readiness-standard | completed | FAIL (🔴 없음) |
| `egm-style` | 없음 | completed | FAIL (🔴 없음) |

- 세 리뷰어 모두 한 번에 끝났고, 다시 돌린 것은 없다. 셋 다 읽기만 하고 아무것도 실행하지 않았다.
- 판단이 다르면 더 엄한 쪽을 따랐다. `carry`는 implementation-readiness가 Blocker, codebase-details가 Major로 봤다. 종합기가 코드를 열어 확인하고 🔴로 올렸다.
- 종합기가 직접 확인한 것:
  - `arm/mujoco_model.py:48-58`(`SimObject` 기본값 length 100), `:114-118`(턱 group 필터), `:126-131`(받은 물체를 모두 놓음, 이름 중복 `ValueError`), `:156-157`(물체 마찰)
  - `arm/mujoco_driver.py:19,30,47,64-65`(`scenario`·`objects` 인자, warnings를 문자열로 보냄)
  - `core/types.py:48-58`(`DetectedObject` 필드)
  - `service/arm_service.py:714-743`(`pick_and_place`·`place`, `pick`·`place_at`은 없음), `:37,242-243,275-276`(`ARM-LOAD-PROBE` WARN)
  - `arm/skills.py:162-199`(`_grasp_height`, `_needed_gap`, G를 `_needed_gap`이 고름)
  - `perception/manual.py:18`(`obj-{i}`, 1부터)

## 4라운드 "M3 전" 목록

| # | 항목 | 판정 | 근거 |
| --- | --- | --- | --- |
| 1 | 3라운드 M3: (a) 식 기호(W·G·H·s·t), G=78 재계산 | 미반영 | 590행이 그대로다. 기호 정의도, 11.2 참조도 없고 "G 90에서 8·s < 28"이 남았다. 코드의 G는 `_needed_gap`이 고른 78이다(`skills.py:181-199`). A1 |
| 2 | 3라운드 M5: `carry` 시나리오 YAML | 미반영 | 594행이 산문 그대로이고, 471행 "(a)(b)용 두 개"와 맞지 않는다. R1 |
| 3 | 3라운드 M11: 턱 condim·priority·group 필터 | 미반영 | 544행은 "충돌 geom을 찾아 `condim=4`"뿐이다. 원본 condim 6·`priority=1`(`so101.xml:23-28`), 손목 서보 상자(`:176`)도 고쳐지는 것, camera 상자(`:214-217`)를 놓치는 것이 없다. A7 (codebase-details는 "부분") |
| 4 | 3라운드 M13: 물체 ID `manual0` 대 `obj-1` | 미반영 | 559행이 아직 `manual0`이다. `ManualSource`는 `obj-1`부터다(`perception/manual.py:18`). A6 |
| 5 | N5: kp·kv·forcerange·armature 원본 값, timestep 0.005→0.002 | 미반영 | 548행에 원본 값이 없다. 545행은 0.002만 적고 원본 0.005(`so101.xml:2`)에서 바꾼다는 말이 없다. n4 (codebase-details는 "부분") |
| 6 | N6: 상자/메쉬 기준 | 미반영 | 557행은 "STL이 있으면 메쉬"다. 코드는 꼭짓점 8개·면 12개로 가른다(`mujoco_model.py:178`). A10 |
| 7 | N8: 실제 body 이름 | 부분 | 536행에 `gripper`는 있다. 544·546행에 `moving_jaw_so101_v1`(`mujoco_model.py:41`)은 없다. A7 |
| 8 | N11: 걸러 낼 이벤트 코드 | 미반영 | 599행이 그대로다. A5 |
| 9 | 595행 11.3.5·11.3.4 순서 | 미반영(모양만) | 595행의 참조 자체는 맞다(200mm는 428행 11.3.4, 칸은 414행 11.3.5). 절 머리말 순서(414행이 426행보다 앞)가 그대로다. n14 |

반영 0, 부분 1, 미반영 8이다.

## 새 지적

### 🔴 Blocker

**R1. `carry`가 시나리오 파일 하나인데, 같은 점에 상자 셋을 "하나씩" 놓아야 한다. 길이도 없다 (M3, 471·594·597행)**
- **리뷰어:** implementation-readiness(Blocker), codebase-details·style(Major). 종합기가 코드로 확인했다.
- **문제:**
  - 471행은 시나리오 파일이 "(a)(b)용 두 개"라고 한다. 그러면 `carry.yaml`은 하나다.
  - 594행은 같은 점 (300, 200)에 폭·길이 30·45·58mm 상자를 하나씩 놓으라고 한다. 한 월드에서 같은 점에 상자 셋을 둘 수는 없다. 그리고 `load_model`은 받은 물체를 모두 놓는다(`arm/mujoco_model.py:126-131`). 시나리오에서 물체 하나만 고르는 방법은 없다.
  - "폭·길이 30·45·58mm"는 정사각 상자 셋으로도, 폭·길이 짝으로도 읽힌다. 길이를 적지 않으면 `SimObject.length_mm` 기본값 100(`mujoco_model.py:56`)이 쓰여 100mm 막대가 놓인다. 그러면 시험이 뜻한 물체와 다른 것을 집는다.
  - 상자 이름이 없는데 597행은 "그 상자"라고 부른다.
- **고칠 문장:**
  - 471행: "`config/sim_scenarios/<이름>.yaml` | 물체 시나리오(13.5). (a)용 `topedge.yaml`, (b)용 `carry30.yaml`·`carry45.yaml`·`carry58.yaml` | M3 | 팔"
  - 594행: "(b) **집어 옮기기**(M3, 시나리오 `carry30`·`carry45`·`carry58`를 하나씩, 시나리오마다 새로 connect한다. 시험은 셋을 parametrize한다):"
  - 594행 아래에 더한다:
    ```yaml
    # config/sim_scenarios/carry30.yaml (b). carry45·carry58은 이름과 폭·길이만 다르다
    objects:
      - {name: box, x_mm: 300, y_mm: 200, angle_deg: 0, width_mm: 30, length_mm: 30, height_mm: 40}
    ```
  - 597·598행의 "그 상자"를 "`box`"로 바꾼다.
  - 정사각이 아니라면 길이 값을 이 YAML에 적는다. 이것은 작성자가 정한다.

### 🟡 Major

**A1. (a)의 식 기호가 정의되지 않았고, G가 90이 아니라 78이다 (M3, 590행, 3라운드 M3)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - W·G·H·s·t가 이 줄에 정의되지 않았다. 11.2(337행)도 가리키지 않는다.
  - `grasp_open_policy: max`이고 W·H를 알면 G는 `Skills._needed_gap`이 고른다(`arm/skills.py:198-199`). W + `gripper_margin_mm` = 78부터 찾고, 78에서 바로 통과한다(`:181-191`). 그래서 G는 78이다.
  - G 78에서 그리퍼 값은 약 45.7, 벌림각은 약 55.5°, s ≈ 1.45다(`config/arms/so101_follower_01.yaml:27-35` 보간, `arm/gripper.py:171-174`). 리뷰어 계산이고 실행하지 않았다.
  - 결론(계획대로 7mm까지 내려간다)은 같다. 숫자만 틀렸다. 337행의 "G = 최대 벌림"도 코드와 다르다(범위 밖).
- **고칠 문장 (590행):** "기호는 11.2와 같다. W 물체 폭 58, H 계획에 넘긴 높이 15, t 낮은 턱 끝 높이, G 시작 벌림(`Skills._needed_gap`), s = `Gripper.narrowing(v(G), tilt_ok=False)`. t = min(`grasp_height_mm` 20, H − `grasp_min_contact_mm` 8) = 7mm. G는 W + `gripper_margin_mm` = 78이고, 78에서 s ≈ 1.45라 (H − t)·s ≈ 11.6 ≤ G − W − 4 = 16이므로 계획대로 t = 7까지 내려간다. 실제 윗면 39mm에서 턱 사이는 78 − (39 − 7)·1.45 ≈ 31.5mm < 58이라 턱이 윗모서리를 친다. 숫자는 `config/arms/so101_follower_01.yaml`의 `gripper_width_mm` 기준이다."

**A2. 시험 진입점과 `DetectedObject` 인자 이름이 코드와 다르다 (M3, 522·588·593·595행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - `DetectedObject`의 필드는 `id, x_mm, y_mm, angle_deg, width_mm, length_mm, confidence, source, height_mm`이다(`core/types.py:48-58`). 588행의 `x=`·`y=`·`angle=`·`width=`·`length=`를 그대로 쓰면 TypeError가 난다. 필수 필드 `id`·`confidence`·`source`도 빠졌다.
  - 522행은 (a)(b)가 `ArmService` 위 시험이라고 한다. 그런데 `ArmService`에는 `pick`·`place_at`이 없다. 있는 것은 `pick_and_place(obj, zone=None, place_at=None)`과 `place(zone=None, at=None)`이다(`service/arm_service.py:714,733`). `pick`·`place_at`은 `Skills`의 메서드다(`arm/skills.py:223,649`).
  - 시험에 쓸 cfg가 없다. 숫자가 팔 파일의 `gripper_width_mm`에 달려 있다. 팔 파일 없이 읽으면 폭 표가 달라 G·s가 바뀐다(`core/config.py:65`). `local.yaml`도 겹쳐 읽힌다.
- **고칠 문장:**
  - 587행 앞에 더한다: "(a)(b) 공통: cfg는 `config/robot.yaml` + `config/arms/so101_follower_01.yaml`(local.yaml 없이)이다. `driver = make_sim_driver(cfg, ws, None, scenario=이름)`, `svc = ArmService(driver, cfg, ws, log_root=tmp_path)`, `svc.connect()`. 집기는 `svc.pick_and_place(obj)`(놓을 곳 없이 집기만), 놓기는 `svc.place(at=(x, y))`다. 둘 다 `slow`로 표시한다."
  - 588행: "`obj = DetectedObject(id="box58", x_mm=300, y_mm=275, angle_deg=45, width_mm=58, length_mm=58, confidence=1.0, source="manual", height_mm=15)`를 넘긴다(실제 상자 높이 39)."
  - 595행: "`svc.pick_and_place(obj)`(`height_mm=40`) 뒤 `svc.place(at=(420, 170))`."

**A3. 시나리오를 어디서 읽는지와 오류 처리가 없다 (M3, 551·552·560행, 13.1)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - `make_sim_driver`는 아직 `scenario`를 쓰지 않는다(`arm/sim_driver.py:8-16`). `MuJoCoDriver`는 `objects=`·`scenario=`를 이미 받는다(`arm/mujoco_driver.py:19,30`).
  - 다음이 정해지지 않았다.
    - YAML을 `SimObject`로 바꾸는 곳(CLI, `make_sim_driver`, `mujoco_model` 가운데 어디인지)
    - 상대 경로의 기준
    - 파일 없음·모르는 키·`name` 빠짐·이름 중복일 때의 예외와 종료 코드
    - 경로로 줬을 때 `connect_info["scenario"]`에 넣을 값
    - `--sim-engine mujoco` 없이 `--sim-scenario`를 줬을 때의 처리
  - 지금은 이름이 겹치면 `connect` 안의 `load_model`에서 `ValueError`가 난다(`mujoco_model.py:128-129`). 이것은 `ArmService.connect`가 잡아 "연결 실패", 종료 코드 1이 된다.
  - 13.1 표에 `sim_driver.py`의 M3 행이 없다.
- **고칠 문장 (552행 뒤에 더한다):** "시나리오는 `make_sim_driver`가 읽는다. 이름이면 `config/sim_scenarios/<이름>.yaml`(레포 루트 기준), `.yaml`로 끝나면 그 경로다. `objects`의 각 행을 `SimObject(**row)`로 만든다. 파일이 없거나, 모르는 키·`name` 빠짐·이름 중복이면 `ValueError`를 내고, CLI는 그 문장을 출력하고 종료 코드 1로 끝난다. 시나리오가 없으면 같은 함수가 `config/manual_objects.yaml`을 읽는다. 만든 목록은 `MuJoCoDriver(cfg, ws, scene=scene, objects=objs, scenario=이름)`으로 넘기고, `scenario`에는 확장자를 뺀 파일 이름을 넣는다. `--sim-scenario`를 `--sim-engine mujoco` 없이 주면 '--sim-scenario는 --sim --sim-engine mujoco와 함께 쓴다'로 종료 코드 1이다." 13.1에 "`arm/sim_driver.py` | 시나리오·`manual_objects.yaml` 읽기(13.5) | M3 | 팔" 행을 더한다.

**A4. 미끄러짐 판정에 쓸 손 자세가 `sim_state()`에 없다 (M3, 576·597행)**
- **리뷰어:** 세 리뷰어 모두
- **문제:** 597행은 `gripper_frame` site 좌표계에서 상자 위치를 보라고 한다. 그런데 13.6의 `sim_state()`는 물체의 작업대 좌표만 준다. 시험이 직접 계산하려면 `driver.data`를 사설 잠금 `_lock` 밑에서 읽어야 한다(`arm/mujoco_driver.py:32-33`).
- **고칠 문장:**
  - 577행 뒤에 키를 더한다: "`"tool": {"pos_mm": [x_mm, y_mm, z_mm], "rot": 3×3 행렬}` = `gripper_frame` site의 자세(작업대 좌표·축)"
  - 597행: "미끄러짐: `held`가 `box`이고 그 `z_mm` > 5인 표본마다, `tool` 자세로 상자 바닥 가운데를 site 좌표계로 바꾼다. 그 값이 이 조건을 처음 만족한 표본의 값에서 3mm 안이다."

**A5. 감시할 이벤트 코드가 정해지지 않았다 (M3, 599행, N11)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - 코드에 "전류 감시 WARN"은 없다. 실제로 있는 것은 다음과 같다.
    - `ARM-FAULT`(ERROR): reason이 `overload:<관절>`·`overcurrent`·`overheat:<관절>`(`arm/motion.py:294,318-330`)
    - `ARM-GRIP-OVERLOAD-WARN`(WARN, `motion.py:360`), `ARM-GRIP-OVERLOAD`(ERROR, `motion.py:365`)
    - `ARM-LOAD-PROBE`(WARN): 어느 관절이든 부하가 50% 이상이면 남는다(`service/arm_service.py:37,242-243,275-276`)
  - "부하 감시의 WARN"에 `ARM-LOAD-PROBE`가 들어가는지에 따라 결과가 갈린다.
  - 추정(실행하지 않음): kp 998.22, forcerange ±2.94(`so101.xml:38`)면 오차 약 0.17°에서 힘이 한계에 닿는다. 그래서 움직이는 동안 부하가 50%를 넘어 `ARM-LOAD-PROBE`가 날 수 있다.
- **고칠 문장 (599행):** "감시: 실행 동안 `events.jsonl`에 `ARM-GRIP-OVERLOAD-WARN`·`ARM-GRIP-OVERLOAD`가 0건이고, reason이 `overload`·`overcurrent`·`overheat`로 시작하는 `ARM-FAULT`가 0건이다. `ARM-LOAD-PROBE`는 세지 않고, 건수와 최대 `load_pct`만 시험 출력에 남긴다."

**A6. 물체 이름이 `ManualSource` ID와 다르고, 시나리오 행의 키·기본값이 없다 (M3, 559·560·561행, 3라운드 M13)**
- **리뷰어:** 세 리뷰어 모두
- **문제:**
  - `ManualSource`는 `obj-{i}`이고 1부터 센다(`perception/manual.py:18`). 시나리오 없이 CLI로 돌리면 같은 상자가 작업 기록에는 `obj-1`, `sim_state`에는 `manual0`으로 남는다. M4에서 둘을 이어야 한다.
  - `SimObject` 기본값(angle 0, width 30, length 100, height 0, mass 30. `mujoco_model.py:48-58`) 가운데 문서에 있는 것은 질량뿐이다.
  - 4라운드 A10의 "M2는 물체 없음"도 13.5에는 아직 없다.
- **고칠 문장 (559–561행):** "시나리오가 없으면 `config/manual_objects.yaml`의 물체를 `ManualSource`와 같은 이름 `obj-1`, `obj-2`, …(파일 순서, 1부터)로 놓는다. body는 `obj:obj-1`이다. M2는 물체를 넘기지 않았다(`objects=()`). 물체 행의 키는 `SimObject` 필드(`name, x_mm, y_mm, angle_deg, width_mm, length_mm, height_mm, mass_g`)이고, 필수는 `name`·`x_mm`·`y_mm`이다. 기본값은 각도 0, 폭 30, 길이 100, 높이 0(→ 40), 질량 30g이다. 마찰은 `1.0 0.005 0.0001`(작업대와 같음)이다. 상자의 길이 축이 `angle_deg` 방향이다."

**A7. 턱 접촉에 쓰는 geom과 역할 이름 규칙이 코드와 다르다 (M3, 544·546행, 3라운드 M11·N8)**
- **리뷰어:** implementation-readiness(Major), style(Major), codebase-details(Nit). 544·546행은 범위 밖이지만 (a)의 접촉 판정에 바로 걸린다.
- **문제:**
  - 원본 `collision_gripper` 클래스는 condim 6, friction `1 5e-3 5e-4`, solref `0.01 1`, priority 1이다(`so101.xml:23-28`). priority 1이 남아 있어 턱–물체 접촉은 턱 값을 쓴다. 그런데 문서에 이 말이 없다.
  - 코드 필터는 `g.group in (3, 4)`다(`mujoco_model.py:114-118`). 그래서 두 가지가 생긴다.
    - `gripper` body의 손목 서보 상자(`so101.xml:176`)도 바뀌고, `jaw_fixed`로 보고된다.
    - `camera_mount` body의 `camera_box1·2`(`so101.xml:214-217`)는 바뀌지 않고 `arm`으로 보고된다.
  - 가동 턱 body 이름 `moving_jaw_so101_v1`이 문서에 없다.
- **고칠 문장:**
  - 544행: "`load_model`은 `gripper`·`moving_jaw_so101_v1` body에 바로 붙은 geom 가운데 원본 `collision_gripper` 클래스(priority 1, 원본 condim 6, friction 1 0.005 0.0005)인 것만 `condim=4`, `friction="1.0 0.02 0.001"`로 바꾼다. priority 1이 남아 있어 턱–물체 접촉은 턱 쪽 값을 쓴다. 손목 서보 상자(`so101.xml:176`, priority 0)는 바꾸지 않는다."
  - 546행 뒤에 더한다: "역할은 다음 순서로 정한다: geom `table`이면 `table`, body `gripper`면 `jaw_fixed`(손목 서보 상자는 `arm`), `moving_jaw_so101_v1`이면 `jaw_moving`, body 이름이 `obj:`·`scene:`으로 시작하면 그 이름, 나머지(`base`, `camera_mount` 포함)는 `arm`."

**A8. `contacts`·`held`의 규칙과 (a)의 표본 방법이 없다 (M3, 577·578·591·592행)**
- **리뷰어:** style, codebase-details, implementation-readiness
- **문제:**
  - `contacts`가 MuJoCo 접촉점마다 한 항목인지, 쌍마다 합치는지 정해지지 않았다. a·b의 순서와 `force_n`의 뜻도 없다. 그래서 591행 "접촉점 h가 34~44"를 한 점만 만족하면 되는지 알 수 없다.
  - 같은 물체를 `contacts`에서는 `obj:box58`, `objects`·`held`에서는 `box58`로 부른다.
  - `held`가 둘 이상일 때의 규칙이 없다.
  - (b)는 50Hz로 읽는다고 적었는데(596행), (a)의 "하강 중"에는 언제 몇 Hz로 읽는지가 없다.
- **고칠 문장:**
  - 577행 끝에 붙인다: "`contacts`는 MuJoCo 접촉점 하나마다 한 항목이고, a·b 순서는 정하지 않는다(시험은 순서 없이 비교한다). `force_n`은 그 점의 법선 힘(N)이다. `a`·`b`는 역할 이름(`obj:<이름>` 포함)이고, `objects`의 키와 `held`는 접두어 없는 이름이다."
  - 578행: "`"held"`: `jaw_fixed`와 `jaw_moving` 둘 다와 법선 힘 > 0으로 접촉한 물체의 이름(접두어 없이) 또는 None. 둘 이상이면 `force_n` 합이 큰 것."
  - 591행: "**기대**: `pick_and_place`가 도는 동안 시험 스레드가 `sim_state()`를 50Hz로 읽는다. {`jaw_fixed` 또는 `jaw_moving`, `obj:box58`} 접촉 가운데 `pos_mm`의 `z_mm`가 34~44mm(윗면 39 ±5)인 것이 한 번 이상 나온다."

**A9. (a)의 대조를 같은 월드에서 이어 하면 결과가 틀리고, 대조의 계획 값이 없다 (M3, 593행)**
- **리뷰어:** implementation-readiness, codebase-details
- **문제:**
  - 실패한 실행 뒤에는 상자가 밀렸거나 쓰러졌을 수 있다. fault로 끝났으면 `ArmService`도 fault 상태다. 물체는 `connect` 때만 놓인다(553행, `mujoco_driver.py:47`).
  - 대조(`height_mm=39`, 각도 45, 수직 집기)는 G 90, t ≈ 27mm(닿는 깊이 약 12mm)로 계획된다(`skills.py:173-191`, 리뷰어 계산). 이 얕은 집기가 `ok`가 되지 않을 때 모델을 고칠지 기준을 고칠지 정해지지 않았다.
- **고칠 문장 (593행):** "**대조**: 새 드라이버와 새 `ArmService`로 다시 connect한 뒤(물체가 처음 자리로 돌아온다), `height_mm=39`로 같은 호출을 하면 `ok`이고 `held`가 `box58`, `tilt_deg` < 10이다(같은 시점에 잰다). 이때 계획은 G 90, t 약 27mm(닿는 깊이 약 12mm)다. 실패하면 모델 접촉값 문제로 보고 기준은 바꾸지 않는다."

**A10. 장면 부품 형태 문장이 코드와 다르다 (M3, 557행, N6)**
- **리뷰어:** codebase-details, style
- **문제:**
  - `box_mm`가 있으면 `file`보다 먼저 쓴다(`core/scene.py:34-45`).
  - MuJoCo에서 상자 geom이 되는 기준은 꼭짓점 8개·면 12개다(`mujoco_model.py:178`).
  - 파일이 없으면 `ValueError`다. CLI는 그 오류로 끝나고(`cli/__init__.py:192-193`), `ArmService`는 빈 장면으로 넘어간다(`service/arm_service.py:71-75`).
- **고칠 문장 (557행):** "`margin_mm`는 물리에 쓰지 않는다. `box_mm`가 있으면 상자, 없으면 `file`의 메쉬다(둘 다 있으면 `box_mm`). MuJoCo에서는 꼭짓점 8개·면 12개 메쉬만 box geom이고 나머지는 mesh geom(충돌은 볼록 껍질)이다. 파일이 없으면 CLI는 그 오류 문장으로 끝난다."

**A11. 높이를 `z_mm`, `h`, "상자 h"로 섞어 부르고, `z_mm`의 기준이 없다 (M3, 576·577·591·597행)**
- **리뷰어:** style (3라운드 N7, 4라운드 n15)
- **문제:** `objects`는 `z_mm`에 "작업대 좌표"라고만 적어서, 0이 작업 면인지 알 수 없다. `contacts`는 `[x, y, h]`이고, 591행은 "접촉점 h", 597행은 "상자 h"다.
- **고칠 문장:**
  - 576행: "`"objects": {이름: {"x_mm", "y_mm", "z_mm", "yaw_deg", "tilt_deg"}}`. 위치는 상자 바닥 가운데의 작업대 좌표이고, `z_mm`는 작업 면 위 높이(13.4의 h)다. `yaw_deg`는 시나리오 `angle_deg`와 같은 기준(작업대 x축에서 반시계, −180~180)이고, `tilt_deg`는 상자 위쪽 축과 연직 사이 각이다."
  - 577행: `"pos_mm": [x_mm, y_mm, z_mm]`(같은 기준)으로 고친다.
  - 591·597행의 "h"를 "`z_mm`"로 바꾼다.

### 🟢 Nit

- n1 (551행) 코드는 `make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False)`다(`arm/sim_driver.py:8`). 고칠 문장: "`make_sim_driver(cfg, ws, scene=None, scenario=None, viewer=False)`. CLI는 `load_scene()`으로 읽은 장면을 넘긴다(`ArmService`와 같은 함수). home 자세는 드라이버가 `cfg.poses["home"]`에서 읽는다."
- n2 (573행) 제목 "(M3, …)"가 580행·코드(`mujoco_driver.py:232-235`, M2에 빈 값으로 있음)와 다르다. 고칠 문장: "`sim_state()`(`MuJoCoDriver`에만 있다. M2는 빈 값, M3에서 채운다):" (4라운드 n1)
- n3 (562행) 지금 `_add_object`는 경고 없이 40으로 둔다(`mujoco_model.py:152`). 드라이버는 warnings를 문자열로 받아 data `{}`로 보낸다(`mujoco_driver.py:64-65`). 고칠 문장: "`load_model`의 warnings 항목은 `(code, data)`이고, 높이 0 물체마다 `("SIM-OBJECT-HEIGHT-DEFAULT", {"object": 이름, "height_mm": 40})`을 하나씩 담는다." (4라운드 n3)
- n4 (545·548행, 범위 밖) 원본 값을 적는다. 고칠 문장: "원본 timestep 0.005(`so101.xml:2`)를 0.002로 바꾼다. `cone`·`impratio`·`integrator`는 원본과 같은 값이다. 서보는 원본 kp 998.22, kv 2.731, forcerange ±2.94, armature 0.028, damping 0.60, frictionloss 0.052(`so101.xml:31-39`)로 시작한다." (N5)
- n5 (555행) 모르는 링크 이름이면 `connect`에서 `ValueError`가 나고(`mujoco_model.py:171-173`) CLI는 "연결 실패"로 끝난다고 적는다. 링크에 붙는 부품 geom의 질량(지금은 부피로 생김, `:175-185`)을 0으로 둘지도 적는다. 지금 `scene.yaml`에는 링크 부품이 없다.
- n6 (562행) "결정 0052가 적용된다"는 결과를 말하지 않는다. 고칠 문장: "계획 쪽 물체 높이는 0으로 남는다. 그래서 기울이지 못하는 집기는 `height_required`로 거부된다(결정 0052)."
- n7 (568행) 팔 원점을 다시 재면 (a)뿐 아니라 (b)·`carry*.yaml`·`place(at=(420, 170))`의 좌표도 같이 옮긴다. 고칠 문장: "팔 원점을 다시 재면(runbook 2-A A-1) 시나리오 YAML의 `x_mm`·`y_mm`와 13.7 (a)(b)의 좌표를 로봇 정면 225mm(`carry`는 150mm) 점으로 함께 고친다."
- n8 (581행) 옛 엔진을 "시뮬레이션 팔"로 부른다. 고칠 문장: "기구학 엔진(`SimDriver`, `--sim-engine kinematic`)은 쥘 물체가 없어서 `no_object`를 성공으로 친다. 드라이버의 `physical_objects`가 참이면 이 규칙을 쓰지 않는다." (4라운드 n15)
- n9 (583행, M4) `programs.py`에는 `is_sim` 함수가 없다. `gui/backend.py:591`의 람다(`arm is self.sim`)를 넘겨 `programs.py:130`이 쓴다. "`backend.py:591`이 넘기는 `is_sim`"으로 고친다.
- n10 (584행) 458행 "범위 나누기"를 되풀이한다. 지운다.
- n11 (592행) 이중 부정이라 읽기 어렵다. 고칠 문장: "그리고 `pick_and_place`가 끝난 직후(놓기 전) `held != "box58"`이거나 `box58`의 `tilt_deg` ≥ 10이다(제대로 쥐지 못함)."
- n12 (597행) "최대 3mm 안"은 같은 뜻이 겹친다. "3mm 이하"로 고친다(A4 문장에 반영).
- n13 ((a)(b)) 두 시험 모두 10초를 넘을 것이다. `slow` 표시와 `log_root=tmp_path`를 적는다(A2 문장에 반영).
- n14 (595행·414–426행, 범위 밖) 11.3.5(414행)가 11.3.4(426행)보다 앞에 있다. 기획 세션에서 절 순서를 바로잡는다.

## 꼭 고칠 것 (M3 시작 전)

1. **R1**: `carry`를 `carry30`·`carry45`·`carry58` 세 파일로 나누고, 길이·이름을 YAML로 적는다. 471행을 같이 고친다.
2. **A1·A9**: 590행에 기호를 정의하고 G 78로 다시 계산한다. 대조는 새로 connect하고 계획 값(G 90, t ≈ 27)을 적는다.
3. **A2**: `svc.pick_and_place(obj)`·`svc.place(at=...)`, 실제 `DetectedObject` 필드, 시험 cfg를 적는다.
4. **A3·A6**: `make_sim_driver`가 시나리오와 `manual_objects.yaml`을 읽는다고 적고, 오류·종료 코드를 정한다. 물체 이름은 `obj-1`부터, `SimObject` 키·기본값을 적는다. 13.1에 `sim_driver.py` M3 행을 더한다.
5. **A4·A8·A11**: `sim_state()`에 `tool` 자세를 더하고, `contacts`·`held` 규칙과 `z_mm` 기준을 정한다. (a)도 50Hz 표본으로 판정한다.
6. **A5**: 599행 감시 코드를 정하고 `ARM-LOAD-PROBE`는 세지 않는다.
7. **A7·A10**: 턱 geom 필터(priority 1)와 역할 이름 순서, 장면 부품 형태 문장을 코드에 맞춘다.

## 확인하지 않은 것

- 시험과 모델은 아무것도 돌리지 않았다.
- G 78의 s ≈ 1.45, 대조의 G 90·t ≈ 27은 표 보간과 코드를 읽어 계산한 값이다. A5의 부하 포화와 `ARM-LOAD-PROBE` 발생도 추정이다.
- (b)의 5mm·3mm 기준을 MuJoCo가 실제로 지킬 수 있는지는 보지 않았다. A5 추정이 맞으면 (b)는 13.2의 부하 % 정의나 kp·forcerange를 고치기 전까지 감시 조건에서 흔들릴 수 있다(13.2는 이번 범위 밖).
- URDF 링크 프레임과 MJCF body 프레임이 같은지(링크에 붙는 장면 부품)는 확인하지 않았다.
- 결정 0053과 13절의 차이(4라운드 "설계서 밖" 항목)는 이번에 다시 보지 않았다.
