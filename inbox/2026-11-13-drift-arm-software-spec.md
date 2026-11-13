상태: 초안 (EGM drift-report, arm-software-spec, main ec9fd73, 2026-11-13)

# Drift Report: arm-software-spec.md

- 대상: `docs/planning/arm-software-spec.md` (main ec9fd73). 코드는 같은 스냅샷의 `src/so101_studio/`, `config/`, `tests/`를 읽어서 대조했다. 테스트는 실행하지 않았다.
- 리뷰어: `egm-codebase-details`를 절 단위로 세 번 띄웠다. 결과는 모두 completed다.
  - A: 1~10절 (1~289행)
  - B: 11~12절 (290~454행)
  - C: 13절과 M1~M4 행 (313~320행, 455~657행)
- 주요 항목은 종합자가 다시 확인했다: `robot.yaml` 값, `move_p` 없음, `skills.py:238`, `params.py` LIMITS.
- 표기:
  - 행 = 설계서 행 번호
  - 코드 경로는 `src/so101_studio/` 기준이다.
  - 고칠 쪽 = 설계서 / 코드 / 판단 필요
  - 심각도: 🔴 Blocker, 🟡 Major, 🟢 Nit
- 중복 두 건(estop 반환값, pick 시작 때 `placed` 정리)은 하나로 합쳤다.
- 합계 58건: 🔴 11, 🟡 24, 🟢 23.

## arm-software-spec.md

### 1~10절

- 🔴 **D1. 설정값이 옛값이다.**
  - 행 51·229 `table_z_mm` −25(육안): 실제는 −15(11-07 실측). 위치는 `config/arms/so101_follower_01.yaml:22`.
  - 행 155·229 `grasp_height_mm` 15: 실제는 20(REQ-033). 위치는 `config/robot.yaml:17`.
  - 행 161·230 `grasp_max_open_mm` 70: 실제는 90. 위치는 `config/robot.yaml:27`.
  - 고칠 쪽: 설계서
- 🔴 **D2. `move_p`와 "run_sequence = MoveP로 재생"이 코드에 없다** (행 25, 116, 157).
  - `arm/trajectory.py`에는 `move_j`(167), `polyline_l`(210), `smooth_l`(308), `move_l`(329), `SequenceTrajectory`(346)만 있다.
  - `run_sequence`는 `skills.py`가 아니라 `arm/teach.py:249`에 있고, 하는 일은 `replay(name, 1.0)`이다.
  - 고칠 쪽: 설계서
- 🔴 **D3. 벌림이 "크기와 상관없이 최대"가 아니다** (행 161).
  - 폭과 높이를 알면 `_needed_gap`이 W + `gripper_margin_mm`부터 2mm씩 늘려 필요한 만큼만 벌린다(REQ-036, `grasp_open_policy: fit`). 위치는 `arm/skills.py:181-191, 198-199`.
  - 고칠 쪽: 설계서
- 🔴 **D4. `height_required` 조건이 설계서와 다르다** (행 168-170).
  - 단동식 그리퍼(`single_jaw`)에서만 검사한다(`arm/skills.py:238`).
  - `tilt_ok=True`여도 기울인 계획이 `ik_fail`이고 높이를 모르면 `height_required`와 `ARM-PICK-TILT-UNREACHABLE` WARN을 남긴다(`skills.py:241-251`).
  - 높이를 알면 수직으로 다시 계획한다(REQ-055).
  - 고칠 쪽: 설계서
- 🔴 **D5. "장치는 제어 스레드만 만진다"(행 122)를 지키지 않는다.**
  - 다음 위치에서 `driver.read_joints()`를 직접 부른다:
    - `service/arm_service.py:436, 443, 546, 587, 688, 757`
    - `arm/skills.py:74`
  - 직렬화는 `driver._lock`(`arm/driver.py:43`)이 한다.
  - 고칠 쪽: 판단 필요
- 🟡 **D6. 5절 타입이 코드와 다르다** (행 76-86).
  - 설계서에 없는 필드:
    - `ArmStatus.fault_text`(`core/types.py:36`)
    - `MotionResult.reason`(`:45`). 거부 이유를 전하는 핵심 필드다.
  - 기본값 차이: `fault=None`, `current_ma=0.0`, `position_error_mm=None`
  - 고칠 쪽: 설계서
- 🟡 **D7. `MotionResult.status` 값이 빠졌다** (행 85).
  - 빠진 값: `height_required`(`skills.py:237`), `no_object`(`skills.py:291`), `locked`(`arm/motion.py:105, 151`, `service/arm_service.py:515`)
  - `core/types.py:41` 주석도 같은 값이 빠져 있다.
  - 고칠 쪽: 설계서와 코드 주석
- 🟡 **D8. 6.9 ArmService 표에 공개 메서드가 많이 빠졌다** (행 192-215). 행 192는 "GUI·CLI가 부르는 목록"이라고 하는데 실제와 다르다. 빠진 메서드는 `service/arm_service.py`에 있다:
  - `resume`(808): fault 뒤 idle로 되돌린다. GUI에 꼭 필요하다.
  - 파라미터·프리셋: `get_params`/`set_params`/`save_preset`/`load_preset`/`list_presets`(656-670)
  - 경로·작업대: `reach_ranges`(590), `preview_path`(684), `follow_path`(494), `reload_workspace`(449)
  - 장치·진단: `gripper_info`(622), `run_diagnostic`(673), `enable_torque`(399)
  - 센서·위치: `set_sensor_offset`(470), `tool_position`(475), `sensor_position`(481), `arm_links`(490)
  - 기록·감시: `create_incident`(375), `log_event`(353), `on_watchdog_stop`(344)
  - 0° 잠금: `unlock_for_verification`(166), `relock_zero`(171), `locked_reason`(175)
  - 고칠 쪽: 설계서
- 🟡 **D9. 파라미터 API(`service/params.py`, #276·#285)가 설계서 어디에도 없다.** 빠진 내용:
  - `get_params`가 돌려주는 `{value, default, saved, timing}`(45-55). `saved`는 연결 때의 파일 값이다(`arm_service.py:67`).
  - 적용 시점 `now`/`next_motion`/`reconnect`와 `RECONNECT_KEYS`·`NOW_KEYS`(15, 33)
  - `LIMITS`(17-29)와 `INT_KEYS`(31): 정수가 아니면 거부한다.
  - `CHOICES`(9-13)
  - 하나라도 틀리면 아무것도 바꾸지 않는 원자적 적용(82-88)
  - 프리셋 이름 규칙과 저장 위치 `config/presets`
  - 덧붙여 `ArmService.get_params` docstring(`arm_service.py:657`)에도 `saved`가 빠져 있다.
  - 고칠 쪽: 설계서(6.9 또는 7절). docstring은 코드.
- 🟡 **D10. 6.9 표의 입력·출력이 실제 시그니처와 다르다.**
  - 행 204 `pick_and_place`: `zone`은 선택이고 `place_at=(x,y)`도 받는다(둘 다 주면 ValueError). 범위 밖이면 움직이지 않고 `out_of_bounds`를 돌려준다(`arm_service.py:714-724`).
  - 행 205 `place`: `zone`과 `at` 중 정확히 하나를 받는다(733-736).
  - 행 188·210 `replay`: `mode="original"|"fitted"`가 빠졌다(788).
  - 행 209 `record_info`: `speed`·`segment_speeds` 인자와 반환값의 `fit`이 빠졌다(778-781).
  - 행 212 `retarget`: "없음"이 아니라 `MotionHandle`을 돌려준다(797-799).
  - 행 196 `disconnect`: 반환값이 None이다(179).
  - 행 213 `set_safety`: 검증 규칙이 빠졌다(`joint_margin_deg` −15~30, `box_mm.z` ≥ 0, None이면 기본값; 634-651).
  - 고칠 쪽: 설계서
- 🟡 **D11. `estop` 반환값이 맞지 않는다.**
  - 행 214는 "없음", 행 343은 `ok`라고 한다.
  - 코드는 실패 이유 `str | None`을 돌려준다(`arm_service.py:804-806`). `ok`는 감시 HTTP 응답에만 있다.
  - 고칠 쪽: 설계서
- 🟡 **D12. 3절 폴더 구조에 없는 파일이 많다** (행 39-42).
  - `arm/`: `autocal/`, `collision.py`, `fault_text.py`, `gripper.py`, `logged_bus.py`, `mujoco_driver.py`, `mujoco_model.py`, `reset_probe.py`
  - `service/`: `diagnostics.py`, `params.py`, `perception.py`, `stop_listener.py`, `studio.py`, `targets.py`, `ultrasonic.py`
  - 고칠 쪽: 설계서
- 🟡 **D13. 9절 CLI 표가 argparse와 다르다** (행 267-272).
  - `pick` 시그니처: 설계서는 `[--place 구역]`인데 실제 5번째 값은 높이다(`cli/__init__.py:241-245`). 같은 문서 행 171과도 맞지 않는다.
  - 빠진 명령: `gui`, `doctor`, `find-port`, `incident`, `calibrate`(`cli/__init__.py:15`)
  - 빠진 옵션: `run --source ultrasonic --us-port`(95-96), `--sim-engine`, `--sim-scenario`, `--arm`, `--port`, `--config`(85-87, 115-116), `replay --mode`(112)
  - 고칠 쪽: 설계서
- 🟡 **D14. 7.1 설정 겹에 그리퍼 파일 층이 빠졌다** (행 235-242).
  - 실제 순서는 `robot.yaml` → 팔 파일 → `config/grippers/<종류>.yaml` → `local.yaml`이다(`core/config.py:170, 184-191`).
  - 그리퍼 종류는 `local.gripper` → 팔 파일 → robot → `single_jaw` 순서로 고른다(181). 행 336("팔 파일의 `gripper`")에는 local 우선이 빠져 있다.
  - 표에 없는 설정 파일: `scene.yaml`, `manual_objects.yaml`, `sim_scenarios/`, `ultrasonic.yaml`, `presets/`
  - 고칠 쪽: 설계서
- 🟡 **D15. 배출 구역 설명이 옛 형식이다** (행 232).
  - 설계서: right (500, 150)
  - 실제: `{x_mm: 480, y_mm: 150, w_mm: 100, h_mm: 80, slot_mm: 75}` 칸 격자(`config/workspace.yaml:6`, `core/config.py:108-119`)
  - `drop_zone_ignore_radius_mm`(`core/config.py:95`)도 설계서에 없다.
  - 고칠 쪽: 설계서
- 🟡 **D16. 6.2 기구학 설명이 실제와 다르다** (행 100-109).
  - IK 잠금(#285)이 빠졌다: `locked` 데코레이터, `Kinematics.lock`(RLock), `_sync`(`arm/kinematics.py:39-51, 146-150`). placo가 스레드 사이에 안전하지 않아서 생긴 `ik_fail`을 막는다. 그리퍼도 `.kin.lock`을 같이 쓴다.
  - None 조건이 다르다: 행 104는 "관절 한계를 넘으면 None"이라고 한다. 실제로는 한계를 풀이기에 넣고, 위치 오차가 `tol_mm` 3mm를 넘을 때만 None이다(168-170). `max_iter=30`도 설계서에 없다.
  - 설계서에 없는 동작:
    - `step()`이 가중치를 0.1 → 0.01 → 0으로 낮춘다(132).
    - `Skills._ik`가 시드 3개로 다시 시도한다(`skills.py:30, 95-101`).
  - IK 한계는 늘 `joint_limits_deg`다(`arm_service.py:119`). 그래서 음수 `joint_margin_deg`로 한계를 넓혀도 IK 한계는 넓어지지 않는다.
  - 고칠 쪽: 설계서
- 🟡 **D17. `known_objects`(#267)가 6.6절에 없다.**
  - `Skills.known_objects`는 (x, y, 반경) 목록이다(`skills.py:42`). GUI가 채운다(`gui/backend.py:523-528`).
  - 집기에 성공하면 40mm 안의 가장 가까운 항목 하나를 뺀다(`skills.py:273-275`).
  - 놓을 때 장애물로 쓴다(419, 514, 539). 재연결해도 유지된다(`arm_service.py:130`).
  - 집기를 시작할 때 반경 40mm 안의 `placed`를 성공 여부와 상관없이 지운다(`skills.py:230`).
  - 고칠 쪽: 설계서
- 🟢 **D18.** 행 56 "카드 54mm 실측 1점(값 약 30)": 실제는 2점(32.0, 27.7)이다(`so101_follower_01.yaml:27`). 고칠 쪽: 설계서
- 🟢 **D19.** 행 280은 "COM 번호를 `robot.yaml`에 기록"이라고 하는데 7.1절은 `local.yaml`이라고 한다. 실제 `config/robot.yaml:1`에는 `port: COM3`가 남아 있다. 고칠 쪽: 판단 필요
- 🟢 **D20.** 행 9 "실제 값은 `robot.yaml`이 기준": 팔별 값은 팔 파일에 있다. 고칠 쪽: 설계서
- 🟢 **D21.** 6.7은 `jog`를 `teleop.py`에 둔 것처럼 쓴다. 실제는 `MotionController.jog`(`arm/motion.py:126`)다. 고칠 쪽: 설계서
- 🟢 **D22.** `arm/motion.py:52` 주석의 state 목록에 `teach`가 없다(`arm_service.py:208`에서 덧붙인다). 고칠 쪽: 코드 주석
- 🟢 **D23.** 행 127은 "감속 0.2초", `motion.py:171` docstring은 "0.3초 안에"다. 고칠 쪽: 판단 필요
- 🟢 **D24. `move_l` 시그니처가 다르다** (행 115).
  - 실제 인자: `kin, start, goal, seed, v_mm_s, a_mm_s2, step_mm, profile, j_mm_s3`(`trajectory.py:329`)
  - 관절이 15°보다 크게 튀면 거부하는 `MAX_WAYPOINT_JUMP_DEG`(179)가 설계서에 없다.
  - 고칠 쪽: 설계서

### 11~12절

- 🔴 **D25. 행 370의 "지금은 yaw 0 고정", "(500,150)은 기울일 수 없다"는 옛 내용이다.**
  - `_place_yaw`가 단동식이면 팔 방향으로 yaw를 정한다(`skills.py:499-505`). `tests/test_req053.py:87`은 (500,150)에서 26.6°로 기울여 놓는 것을 확인한다.
  - 기본 구역도 이제 (480,150) 사각형이다.
  - 고칠 쪽: 설계서
- 🔴 **D26. 집기 기억을 지우는 시점이 다르다** (행 363).
  - 설계서: 놓기가 끝날 때(성공·실패 모두)
  - 실제: 벌리기 시작할 때만 지운다(`skills.py:650, 695`). 벌리기 전 실패(정지, `ik_fail`, `low_hover`)에서는 기억을 유지하고, `test_req053.py:161-173`이 이것을 고정한다. 다음 칸 시도(`skills.py:494-495`)도 이 동작에 기댄다.
  - 고칠 쪽: 설계서
- 🔴 **D27. 놓은 뒤 상승 방식이 다르다** (행 400-401).
  - 설계서: 틈과 기울기를 유지하고 수직으로 올라간다.
  - 실제:
    - 곧게 올라가다 높이가 모자라면 `_untilt_goal`로 기울기를 풀며 마저 올라간다. 지나는 자세마다 상자 2mm, `known_objects`, `check_goal`을 검사한다(`skills.py:560-605, 622-647`).
    - 계획이 없으면 갈 수 있는 만큼 올라간 뒤 멈춘다(628-636).
    - "경고만 남기고 다음 이동"은 옛 내용이다. 지금은 ERROR와 `stopped(low_lift)`다(617-620).
  - 고칠 쪽: 설계서
- 🟡 **D28. hover를 지키지 못할 때의 순서가 다르다** (행 379, 381).
  - 실제 순서:
    - 새 `_descent`가 실패하면 먼저 옛 방식(`ARM-PLACE-LEGACY`)을 시도한다.
    - 그것도 실패하고 `hover_need_mm > 10`이며 `descent_blocked_by`가 비었을 때만 `low_hover`다.
    - 그 밖에는 `ik_fail`이고, 이유가 `known_object: …`일 수 있다(`skills.py:671-683`).
  - 고칠 쪽: 판단 필요
- 🟡 **D29. 이웃 물체 높이를 쓰지 않는다** (행 379, 398).
  - 설계서: "가장 높은 것보다 10mm 위"
  - 실제:
    - `known_objects`에는 높이가 없고 늘 40mm로 가정한다(`NEIGHBOUR_H_MM`, `skills.py:27, 538-540`).
    - 턱 끝이 50mm 이상이면 검사하지 않는다(`skills.py:520`).
    - 그래서 40mm보다 높은 물체는 과소평가된다.
  - 고칠 쪽: 판단 필요(높이를 넘기거나, 40mm 가정을 문서화)
- 🟡 **D30. 행 399가 실제 결과와 다르다.**
  - 설계서는 "쥔 틈을 쓰면 40mm 물체가 ②로 못 놓인다"고 읽힌다. 실제로는 폭 45를 알아도 높이 40mm면 `tilt_hits_object`로 ③이 된다(`test_req053.py:116-120`).
  - 그래서 행 419의 ② 기준(높이 40)은 충족되지 않는다. 시험은 높이 25mm로 한다(`test_req053.py:104`).
  - 고칠 쪽: 설계서(11.3.4 기준도 다시 정해야 함)
- 🟡 **D31. 행 422 완료 기준을 확인하는 테스트가 없다.**
  - 대상: 높이 모름 또는 폭 75mm일 때 `fixed_tilt`와 `skipped` 이유
  - 시뮬 테스트는 옛 점 (500,150)만 쓴다(`test_req053.py:18`). 기본 사각형 칸에서 대칭 V자를 확인하는 테스트도 없다.
  - 고칠 쪽: 코드(테스트)
- 🟡 **D32. 칸 선택 기준이 다르다** (행 433).
  - 설계서: 경로 검사(4번)를 통과한 첫 칸
  - 실제: 하강 계획이 `ik_fail`/`low_hover`일 때만 다음 칸으로 간다. 열기 경로 검사가 실패하면 같은 칸에서 ④ 제자리 열기를 한다(`skills.py:492-496`).
  - 고칠 쪽: 판단 필요
- 🟡 **D33.** 행 341의 fault 원인 문장은 `service/targets.py:26-31`의 `reason_text`가 아니라 `arm/fault_text.py:6`의 `describe`에 있다. `reason_text`는 목표 거부 사유다. 고칠 쪽: 설계서
- 🟡 **D34.** 행 410은 `ARM-PLACE-LEGACY`를 INFO라고 한다. 기울여 쥔 경우는 WARN이다(`skills.py:674-675`). 고칠 쪽: 설계서
- 🟡 **D35. 행 412의 `place_*` 새 키가 설정 스키마에 없다.**
  - `service/params.py:17-29`의 `LIMITS`에 `place_drop_mm`, `place_release_mm`, `place_clearance_mm`, `place_open_s`가 없어서 범위 검사 없이 바뀐다. 예: `place_open_s` = 0
  - `params.py:71`의 `type(cur[k])(v)` 때문에 yaml에 정수로 적힌 `place_drop_mm: 3` 등에 2.5를 넣으면 2로 잘린다. 30행 주석이 막으려던 바로 그 경우다.
  - 고칠 쪽: 코드
- 🟡 **D36. `place_height_mm` 주석이 다르다.** 행 382와 코드는 손 기준점 z로 쓰는데(`skills.py:677`), `config/robot.yaml:34` 주석은 "그리퍼 끝 높이"라고 한다. 고칠 쪽: 코드(설정 주석)
- 🟡 **D37. 11절 AC 표(행 307-311)에 문제가 있다.**
  - 헤더 행이 없어 표로 렌더링되지 않는다.
  - 결과 열에 결과 대신 REQ 번호가 있다.
  - AC4는 11-10에 끝났고(`so101_follower_01.yaml:15`, 행 344), AC5의 0° 확인도 기록됐다(`:37-39`).
  - 행 291 "2026-11-02 기준"은 M2·M3 행(11-13)과 맞지 않는다.
  - 고칠 쪽: 설계서
- 🟡 **D38. 12절 미정 사항 일부가 이미 해결됐다.**
  - 행 453 보드 시리얼: `board_serial: "5B8E114816"`로 해결됐다(`so101_follower_01.yaml:3`). 고칠 쪽: 설계서
  - 행 452 "AC4 전에 확인": AC4가 끝났고 실측은 242.6°다(`:16`). 고칠 쪽: 판단 필요
  - 행 329 shoulder_pan 230~245°(`autocal/plan.py:44`)를 확정할지: 고칠 쪽: 판단 필요
- 🟢 **D39.** 행 395의 `tilt_scale=(k0,k1)` 선형 설명이 행 417의 k(v)와 모순된다. 실제는 callable이다(`skills.py:398-399`, `gripper.py:190-195`). 고칠 쪽: 설계서
- 🟢 **D40. 기억 키 이름이 다르다** (행 361-364).
  - 설계서: `grip_tilted`, `grip_tip_h`
  - 실제: `self.grip`의 `tilted`, `tip_h`, `obj_h`, `obj_w`, `held_gap`(`skills.py:272`)
  - 고칠 쪽: 설계서
- 🟢 **D41. 행 360의 Skills 재생성 시점이 다르다.**
  - Skills는 설정 저장 때가 아니라 연결·재연결 때 다시 만든다(`arm_service.py:127-130`).
  - `reload_workspace`는 `ws`만 바꾸고 `placed_height`는 넘기지 않는다(463).
  - 고칠 쪽: 설계서
- 🟢 **D42.** 행 412의 `_opening_blocked`는 없다. 실제는 `_scan_opening`/`_route_skip`/`_try_route`/`_build_route`(`skills.py:338-429`)다. 고칠 쪽: 설계서
- 🟢 **D43.** 설계서에 없는 skip 이유가 있다: `legacy_pose`, `not_tilted_grip`, `tilted_grip`, `tilt_not_allowed`, `gap_too_wide`, `clearance`, `no_lift_after_opening`(`skills.py:339-356, 427-428`). 고칠 쪽: 설계서
- 🟢 **D44.** 설계서에 없는 이벤트가 있다: `ARM-PLACE-LOW-HOVER`, `ARM-DROP-ZONE-FULL`(ERROR, `skills.py:489, 680`). 고칠 쪽: 설계서
- 🟢 **D45.** 행 437: 초음파 `ignore_zones`는 사각형이 아니라 원(반대각선 + 여유)으로 넘긴다(`core/config.py:101-106`). 고칠 쪽: 설계서
- 🟢 **D46.** 행 438 "GUI 나중": 평면도의 사각형·칸 그리기는 이미 있다(`gui/sensors/manual_view.py:32`). 놓인 칸 표시만 없다. 고칠 쪽: 설계서
- 🟢 **D47.** SAME-SPOT 경고 때 같은 자리의 `placed` 항목을 지우는 동작이 설계서에 없다(`skills.py:660`). 고칠 쪽: 설계서
- 🟢 **D48.** dataclass 기본값 `approach_height_mm=80`, `grasp_height_mm=15`(`core/config.py:37-38`)가 yaml의 40·20과 다르다. yaml이 덮으므로 실행에는 영향이 없다. 고칠 쪽: 코드

### 13절 MuJoCo

- 🟡 **D49. 시나리오 경로 기준이 다르다** (행 556 "레포 루트 기준").
  - `sim_driver.py:39, 42`는 현재 작업 폴더 기준이다. `mujoco_model.py:15`는 `__file__` 기준이다.
  - 레포 루트 밖에서 실행하면 "파일 없음"이 된다. `so101.cmd`는 어느 폴더에서든 실행한다고 되어 있다.
  - 고칠 쪽: 판단 필요(코드를 레포 루트 기준으로 바꾸기 권장)
- 🟡 **D50. 행 483 "calibrate·linkwatch를 지금 --sim과 같이 거부"는 사실이 아니다.**
  - 일반 `--sim`에서 `linkwatch`는 `cfg.port`를 연다(`cli/__init__.py:179-183`). `calibrate --verify-zero`도 거부하지 않는다(167, 223-226).
  - 거부 문장과 종료 코드도 다르다:
    - 일반 sim: 2(`arm/autocal/cli.py:31-33`)
    - MuJoCo: 1(`cli/__init__.py:137-138`)
  - 고칠 쪽: 설계서(일반 `--sim`의 linkwatch가 실포트를 여는 것은 판단 필요)
- 🟡 **D51. `info`가 엔진을 무시하지 않는다** (행 482).
  - `info`는 MuJoCo 드라이버를 만들고 `connect`까지 한다(`cli/__init__.py:185-203, 290`). mujoco가 없으면 종료 코드 2다.
  - `doctor`/`find-port`/`incident`도 130-140행의 MuJoCo 검사를 먼저 거친다.
  - 고칠 쪽: 설계서(또는 코드)
- 🟡 **D52. (a)(b) 시험이 `local.yaml`을 빼지 않는다** (행 605).
  - `tests/test_mujoco_scenarios.py:16`이 `load_robot("config/robot.yaml")`을 부르고, `core/config.py:175-176`이 `local.yaml`과 팔 선택을 겹친다.
  - 개발 PC마다 결과가 달라질 수 있다.
  - 고칠 쪽: 코드
- 🟡 **D53. event_sink를 잇는 시점이 다르다** (행 511-512).
  - 설계서: 생성자에서 잇는다.
  - 실제: `connect` 안, `driver.connect` 직전에 잇는다(`service/arm_service.py:109`). `load_model`은 `MuJoCoDriver.connect` 안에서 돈다(`mujoco_driver.py:55`).
  - 고칠 쪽: 설계서
- 🟢 **D54.** 행 466의 `MuJoCoDriver` 시그니처에 `scenario=None`이 빠졌다(`mujoco_driver.py:26`). 행 559와도 맞지 않는다. 고칠 쪽: 설계서
- 🟢 **D55.** 행 469 "168행에서 …"의 행 번호가 낡았다. 실제는 `cli/__init__.py:185`다. 고칠 쪽: 설계서
- 🟢 **D56.** 행 485·623 "`--sim` 없이 `--sim-engine`이면 오류"는 `mujoco`일 때만 맞다(`cli/__init__.py:132-134`). 고칠 쪽: 설계서
- 🟢 **D57. 설계서에 없는 경계 동작이 있다.**
  - 행 540·652 `BASE_BOX`: 실제는 `BASE_BOX_POS`/`BASE_BOX_HALF`(`mujoco_model.py:33-34`)이고 geom 이름은 `base_box`(110)다.
  - `physical_limits_deg` 일부 관절만 있을 때: 빠진 관절은 경고 없이 원본 range를 쓴다(`mujoco_model.py:142-145`).
  - 시나리오가 없을 때 `manual_objects.yaml`:
    - 파일이 없으면 물체 없이 조용히 시작한다(`sim_driver.py:43`).
    - 키가 틀리면 TypeError traceback이 난다(`sim_driver.py:44`. `cli/__init__.py:195`는 ValueError만 잡는다).
  - 고칠 쪽: 설계서(TypeError 처리는 코드)
- 🟢 **D58. 시험 주석과 출력이 다르다.**
  - `tests/test_mujoco_scenarios.py:110` 주석은 "3mm인데 3.5로 둠(기획 확인 중)"인데, 행 620은 이미 3.5mm다.
  - 109행은 `ARM-LOAD-PROBE`의 최대 `load_pct`가 아니라 `s.max_load`를 출력한다. 행 622와 다르다.
  - 고칠 쪽: 코드(시험)

## 꼭 고칠 것 (구현자를 틀리게 이끄는 것)

| # | 항목 | 고칠 쪽 |
|---|---|---|
| 1 | D1 옛 설정값: `table_z_mm` −25→−15, `grasp_height_mm` 15→20, `grasp_max_open_mm` 70→90 (행 51, 155, 161, 229, 230) | 설계서 |
| 2 | D2 `move_p`/MoveP 재생은 없다. `run_sequence`는 `teach.replay` (행 25, 116, 157) | 설계서 |
| 3 | D3 벌림 `fit` 정책(`_needed_gap`)이 빠졌다 (행 161) | 설계서 |
| 4 | D4 `height_required`는 단동식에서만 나오고, 기울인 계획 `ik_fail`에서도 나온다 (행 168-170) | 설계서 |
| 5 | D6·D7 `MotionResult.reason`과 status 값 `height_required`/`no_object`/`locked`가 빠졌다 (행 76-86) | 설계서 + 코드 주석 |
| 6 | D8·D9 ArmService 표에 `resume`, 파라미터·프리셋 등이 빠졌고, `params.py` 규약(saved·LIMITS·INT_KEYS·원자적 적용)이 없다 | 설계서 |
| 7 | D13 CLI `pick` 5번째 값은 높이다. 명령 `doctor`/`calibrate`/`gui`/`find-port`/`incident`, `--source ultrasonic`, `--sim-engine`이 빠졌다 | 설계서 |
| 8 | D14 설정 겹에 `config/grippers/` 층과 `local.gripper` 우선이 빠졌다 | 설계서 |
| 9 | D16 IK 잠금(#285)이 빠졌고, IK None 조건(위치 오차 3mm)이 다르다 | 설계서 |
| 10 | D25·D26·D27 놓기의 yaw 0 고정은 옛 내용이다. 기억은 벌림 시작 때 지운다. 상승은 기울기를 풀며 올라가고 `stopped(low_lift)`로 끝난다 (행 363, 370, 400-401) | 설계서 |
| 11 | D28·D32 hover 실패 순서(옛 방식 먼저)와 칸 선택 기준이 테스트로 굳은 동작과 다르다 (행 379, 381, 433) | 판단 필요 |
| 12 | D29·D30 이웃 높이를 40mm로 가정하고, 11.3.4의 ② 기준(높이 40)은 충족되지 않는다 (행 379, 398, 399, 419) | 판단 필요 |
| 13 | D35 `place_*` 키가 `LIMITS`에 없어 범위 검사가 없고, 소수가 정수로 잘린다 (`service/params.py:17-29, 71`) | 코드 |
| 14 | D5 "장치는 제어 스레드만"과 실제 직접 읽기가 다르다 (행 122) | 판단 필요 |
| 15 | D49 시나리오 경로가 작업 폴더 기준이다 (행 556, `arm/sim_driver.py:39, 42`) | 판단 필요 |
| 16 | D50·D51 MuJoCo 거부 규칙 설명이 틀렸다: linkwatch/verify-zero는 일반 sim에서 거부되지 않고, `info`는 엔진을 쓴다 (행 482-483) | 설계서 |
| 17 | D53 event_sink는 생성자가 아니라 `connect`에서 잇는다 (행 511-512) | 설계서 |
