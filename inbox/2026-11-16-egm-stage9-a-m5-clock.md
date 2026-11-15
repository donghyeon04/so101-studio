# EGM Stage 9 (가) 구현 준비 시험 보고서

**대상:** `docs/planning/mujoco-spec.md` 301~389행 (13.10.1 M5 집기 시험장, 13.10.2 M5b 첫 단계 시계 주입). 254~300행은 맥락으로만 읽었다.
**기준 사본:** `C:\Users\c\Desktop\so101-gui-docs-tmp` (main c5db419)

## 결론

**아직 구현할 수 없다.** MuJoCo 세션이 이 절만 보고 묻지 않고 구현하기는 어렵다. 꼭 고칠 것이 8개다.

시계 주입은 벽시계 속도에 묶이는 문제는 없앤다. 하지만 실행마다 결과가 갈리는 지터는 없애지 못한다. 그 이유는 셋이다.
- 틱 스레드가 다른 스레드를 기다리지 않고 시간을 흘린다.
- 표본기와 "1.0초 뒤" 기다림이 아직 벽시계로 돈다.
- 감시 목록이 실제 코드와 다르다.

그래서 13.10.2 완료 기준 3(0.01mm 반복)은 지금 설계로는 통과할 수 없다. 기준 4의 해석("모이면 지터, 갈리면 물리")도 틀린 결론을 U15에 넣을 수 있다.

코드와 맞는 것은 다음과 같다(고칠 것 없음).
- `load_robot(path, arm=, local_path=)`
- `pick_and_place(obj, zone, place_at)`
- `MuJoCoDriver(realtime=False)`와 `advance`, `dropped_count`, `sim_state` 키
- 클램프 이름, `wrist_hcsr04`·`wrist_bracket`
- 배출 구역 `right`
- 연결할 때마다 `MotionController`·`Skills`를 새로 만드는 점

## 리뷰어 상태

| 리뷰어 | 상태 |
| --- | --- |
| `egm-implementation-readiness` | completed (1회차는 사용 한도로 끊겨 다시 돌렸다) |
| `egm-codebase-details` | completed (1회차는 사용 한도로 끊겨 다시 돌렸다) |
| `egm-style` | completed |

심각도별 개수(중복을 합친 뒤): 꼭 고칠 것 8, 고치면 좋은 것 13.

## 꼭 고칠 것

1. **375, 384~385행: `StepClock`이 보조를 맞추지 않아 결과가 실행마다 갈린다.**
   - 틱 스레드는 다른 스레드를 기다리지 않고 `advance`를 계속 부른다.
   - 그 사이 호출 스레드는 IK·계획(`skills.py:307~353`)을 계산한다. 접촉 판단도 `_close_until_contact`가 `time.sleep(0.04)` 폴링으로 하고 4회째에 `ctl.stop()`을 부른다(`skills.py:200~207`).
   - 이 동안 시뮬레이션 시간이 얼마나 흐를지는 GIL과 OS 스케줄이 정한다. 그래서 그리퍼가 멈추는 값이 실행마다 다르다.
   - MuJoCo는 입력이 같으면 결과도 같다. 그래서 `StepClock`에서 남는 흩어짐은 물리가 아니라 스레드 경쟁에서 나온다.
   - 제안 문장(375행 뒤): "`StepClock`에서 틱 스레드는 등록된 다른 스레드가 모두 `clock.sleep`이나 `MotionHandle.wait` 안에서 기다리는 동안에만 `advance`한다(조건 변수와 '깨어 있는 스레드 수' 카운터. `ArmService` 공개 메서드에 들어가면 1을 더하고, 기다림에 들어가면 1을 뺀다). 그 밖에는 틱 스레드도 기다린다."
   - 이번 단계에서 이렇게 하지 않으려면 384~385행을 이렇게 바꾼다: "결과가 같은지는 M5b 둘째 단계(스케줄러 하나)에서 본다. 이 단계에서는 같은 씨앗 10회의 미끄러짐 분포만 남기고 실시간 분포와 비교하며, 원인 판정은 하지 않는다."

2. **317~318, 342행: 표본기와 "1.0초 뒤"가 어느 시계인지 없다.**
   - "13.7과 같은 방법"은 `tests/test_mujoco_scenarios.py:30~41`의 `Sampler`다. 이것은 `time.sleep(0.02)` 벽시계로 표본을 뜬다. 1.0초 기다림(`:102`)도 벽시계다.
   - `StepClock`에서 이대로 두면 표본이 매번 다른 시뮬레이션 시각에 찍힌다. 그러면 `picked`, 미끄러짐 최대, 완료 기준 5의 클램프 접촉이 실행마다 달라진다.
   - 제안 문장: "`sim_state()` 표본과 끝난 뒤 기다림은 에피소드의 시계를 쓴다. `StepClock`에서는 틱 스레드가 `control_hz / 50`틱마다 부르는 콜백으로 표본을 뜨고, 기다림은 `clock.sleep(1.0)`이다(시뮬레이션 시간 기준)."

3. **372, 375~376행: `Clock` 규약만으로는 설계를 구현할 수 없고, 그대로 짜면 교착된다.**
   - `now`·`sleep` 둘만으로는 `StepClock`이 어느 스레드가 틱 스레드인지 알 수 없다. `MotionController`는 `__init__` 안에서 이름 없는 스레드를 만들어 바로 띄운다(`motion.py:100~101`).
   - 376행은 상태 루프를 틱 스레드가 부르라고 하지만, `MotionController`에는 `ArmService`를 부를 고리가 없다.
   - 틱 본문은 `with self._lock` 안에서 돈다(`motion.py:283`). 이 락은 재진입이 안 되는 `threading.Lock`이다(`:97`). 그런데 상태 루프 본문이 부르는 `check_blocked`(`:421`)와 `note_comm`(`:257`)이 같은 락을 다시 잡는다. 그래서 틱 본문 안에서 부르면 멈춰 버린다.
   - 제안 문장: "`Clock`은 `now()`, `sleep(s)`, `tick_sleep(s)`(제어 틱 스레드만 부른다. `WallClock`에서는 `sleep`과 같다), 그리고 읽기 전용 `drives_status: bool`을 가진다. `MotionController(..., clock=WallClock(), on_status=None)`의 `_loop`는 `with self._lock` 블록이 끝난 뒤, `clock.tick_sleep` 앞에서 `control_hz // 10`틱마다 `on_status()`를 부른다. `ArmService.connect`는 `clock.drives_status`이면 `_status_thread`를 띄우지 않고, `on_status`에 상태 루프 한 바퀴 본문을 넘긴다."

4. **379행, 382행 기준 1: 감시 목록이 코드와 다르다.**
   - "막힘 감지는 횟수 기준이라 그대로"는 틀렸다. `BLOCKED_ZONE_S`·`_blocked_until`은 `perf_counter`로 잰 시간 기준이다(`motion.py:432, 441~444, 535`).
   - 목록에 빠진 것:
     - 통신 끊김 0.5초(`HealthMonitor.comm_s`, `motion.py:116, 261, 263, 296, 531~532`)
     - 틱 따라잡기 `MAX_TICK_ADVANCE`(`motion.py:486~517`)
     - 조그 데드맨(`:544`)
     - `wait_idle` 2초(`:212~216`)
     - `ArmService`의 `time.monotonic` 쿨다운(`arm_service.py:260, 286, 330, 379`)
   - "동작 시간 초과"는 이 세 파일에 없다.
   - 이대로면 기준 1의 허용 목록 시험이 `monotonic` 때문에 실패한다. 아니면 구현자가 허용 목록을 마음대로 넓히게 된다.
   - 제안 문장: "같은 시계를 쓰는 것: 막힘 판정 시간(`BLOCKED_ZONE_S`, `_blocked_until`), 통신 끊김 창, 부하·과열 감시, `MAX_TICK_ADVANCE`, 조그 데드맨, `wait_idle`, 접촉·놓은 뒤 기다림. 벽시계로 남는 것(허용 목록): 기록 시각, `ArmService`의 탐침·사고 묶음 쿨다운(`time.monotonic`), 기록 스레드, 감시 프로세스 heartbeat, `record`."

5. **312, 362, 366행: 장면 거르기 자리가 정해져 있지 않고, "`ArmService`는 고치지 않는다"와 부딪친다.**
   - `load_scene(path)`에는 그리퍼 인자가 없다(`scene.py:48`). `ArmService`는 인자 없이 `load_scene()`을 부른다(`arm_service.py:85, 480`).
   - 시험장은 그리퍼를 `local.yaml`이 아닌 덮어쓰기 파일로 정한다. 그래서 `load_scene`은 혼자서 `parallel`을 알 수 없고, `ArmService`가 `cfg.gripper`를 넘겨야 한다.
   - `load_model`이 이미 손목 부품을 건너뛴다(`SIM-PART-SKIPPED`, `mujoco_model.py:179~185`). `tests/test_mujoco_model.py:87`이 이 동작을 확인한다. 이 분기를 남길지는 적혀 있지 않다.
   - 완료 기준 4는 이 결정에 달려 있는데, 그 결정을 "팔 세션이 정한다"로 미뤄 두었다.
   - 제안 문장(312행 끝과 366행): "`load_scene(path='config/scene.yaml', gripper: str | None = None)`이 읽을 때 거른다(부품에 `gripper` 키가 있고 값이 다르면 건너뛴다). `ArmService.__init__`과 `reload_workspace`, CLI(`cli/__init__.py:177`), GUI(`gui/backend.py:177`), 시험장은 `gripper=cfg.gripper`로 부른다. `mujoco_model.py`의 `SIM-PART-SKIPPED` 분기는 안전망으로 남긴다. 팔 REQ에는 `service/arm_service.py`의 두 줄이 들어간다."

6. **349, 351, 363행: G와 벌림 축을 얻을 수 없다.**
   - 시작 벌림 G를 남기는 이벤트가 없다. `last_pick_info`(`skills.py:334`)에도 벌림이 없다.
   - G는 `_plan_pick` 안에서 `_needed_gap`으로 정해진다(`skills.py:244~262`). 366행이 `Skills`를 못 고치게 하므로 이벤트를 더할 수도 없다.
   - 정사각형은 벌림 축이 `_pick_yaw`의 선택에 따라 두 방향 중 하나가 된다(`skills.py:173~181`). 그래서 "벌림 축으로 어긋나게"를 정할 수 없다.
   - 식 (G − W)/2의 W가 참값 폭인지 알려 준 폭인지도 없다.
   - 제안 문장: "G와 벌림 축은 시험장이 `connect` 뒤 알려 준 물체로 계산한다. G = `svc.skills._needed_gap(given_obj, tilt_ok=False)`, 벌림 축 = `svc.skills._pick_yaw(given_obj)` 방향의 단위벡터, 그 수직 = +90°. 둘 다 `results.jsonl`에 `gap0_mm`·`jaw_yaw_deg`로 적는다. (G − W)/2의 W는 참값 폭이다. 어긋남은 참값 자리에 더해 알려 준 값을 만든다."

7. **307~308, 316, 319행: 에피소드를 만드는 자리가 지금 코드로는 안 된다.**
   - `make_sim_driver`에는 `realtime` 인자가 없다(`sim_driver.py:57~69`). 그래서 "바뀌는 것은 한 곳뿐"을 지킬 수 없다.
   - `--gripper single_jaw`를 `cfg.gripper`에 나중에 대입해도 소용이 없다. 그리퍼 종류는 `load_robot_ex` 안에서 `config/grippers/<종류>.yaml` 층을 고르기 때문이다(`config.py:185~195`). 그러면 기준 4의 `single_jaw` 칸이 틀린 설정으로 돈다.
   - 제안 문장: "시험장은 `MuJoCoDriver(cfg, ws, scene=, objects=, realtime=, scenario=)`를 직접 만들고, 겹침 검사는 `sim_driver._check_overlap`을 재사용한다(임시 시나리오 파일을 쓰지 않는다). `--gripper`는 에피소드 폴더에 `gripper: <값>` 한 줄짜리 임시 덮어쓰기 파일을 만들어 `load_robot(..., local_path=)`로 읽는다."

8. **347, 356행: `jitter` 표시가 모든 에피소드에 붙고, 이름이 YAML 키와 겹친다.**
   - 드라이버는 연결할 때 프레임을 놓친다(13.3, `mujoco_driver.py:184~189`). 그래서 `dropped_count > 0` 조건은 모든 에피소드에서 참이 된다.
   - 같은 이름 `jitter`가 332행에서는 씨앗마다 흔드는 폭이라는 뜻의 YAML 키다.
   - 제안 문장: "`connect()` 직후의 `dropped_count`를 빼고 남는 값이 0보다 큰 에피소드에는 `realtime_lag: true`를 달고, 요약 표에서 따로 센다." 356행도 "`realtime_lag` 건수"로 바꾼다.

## 고치면 좋은 것

- **341~345, 359행: 판정 값의 키 이름이 없다.**
  - `agree`는 한글 표기(거짓 성공·거짓 실패·일치)뿐이고, "재는 값"에도 키가 없다. 그래서 "줄마다 위 값이 다 있다"를 시험할 수 없다.
  - 키를 정한다. `agree`는 `false_ok`·`false_fail`·`match`, 재는 값은 `slip_mm`·`place_err_mm`·`tilt_end_deg`·`other_pushed_mm`·`events`·`sim_s`·`wall_s`·`dropped_count`.
- **342~343행: 판정 기준이 덜 정해졌다.**
  - "놓으려던 점"은 구역 가운데가 아니라 `Skills`가 고른 칸이다. `svc.skills.last_place_xy`(`skills.py:60, 802`)에서 읽는다고 적는다.
  - `rejected`의 조건을 적는다. `ik_fail`은 움직인 뒤에도 난다(`skills.py:322, 359`). 예: "`out_of_bounds`·`height_required`·`locked`, 또는 `ik_fail`이면서 도구가 1mm 미만 움직임".
  - `drop_zone_full`은 독립 상태가 아니다. `stopped`의 reason 접두어다(`skills.py:574`).
- **334, 350, 307행: 고정 묶음과 에피소드가 덜 정해졌다.**
  - `pick_smoke`(12개)의 격자 내용이 없다.
  - `inject_grid`는 지금 YAML 형식(곱집합과 스칼라 `given`)으로는 적을 수 없다. 자리 목록과 `given` 격자를 담을 형식, 상자 `angle_deg`도 적는다.
  - `--only`가 받는 에피소드 id 형식과 `--seeds N`에서 에피소드별 씨앗을 만드는 규칙이 없다.
- **374행: 시각 단위가 맞지 않는다.** `PoseSample.t_ns`는 정수 ns이고 `now()`는 실수 초다. `t_ns = int(clock.now() * 1e9)`를 적는다.
  - 이미 `RealClock`/`SimClock`(`perception/ultrasonic/model.py:36`, `fake_esp32.py:10`)이 `now_ns()`·`sleep()` 규약을 쓴다. 이것을 `core/clock.py`로 옮겨 재사용하면 13.10.3에서 어댑터가 필요 없다.
- **376행:** `control_hz / 10`은 `--hz`가 10의 배수일 때만 맞는다. `//`로 쓰고 최소 1로 둔다.
- **377행:** 만들 때 할 수 있는 검사는 `driver.realtime`·`driver.viewer`뿐이다. GUI·감시 프로세스 검사 문구는 지운다.
- **물리가 실패하면:** `advance`가 아무것도 하지 않는다(`mujoco_driver.py:176~178`). 그러면 5초 뒤 `RuntimeError`가 난다. 시험장이 이것을 잡아 `outcome: sim_fail`로 끝낸다고 적는다.
- **366행:** 실패 주입은 사고 묶음과 60초 후속 스레드를 띄운다(`arm_service.py:92, 377~393`). 시험장에서는 `svc.incident_after_s = 0`으로 둔다.
- **재사용:** `Sampler`, 미끄러짐 계산(`test_mujoco_scenarios.py:104~111`, `test_mujoco_parallel.py:129~139`), `events()`를 `mujoco_bench.py`로 모으고, 시험은 그것을 가져다 쓴다. 그러면 13.7 (b) 정의가 세 군데로 복사되지 않는다.
- **305~310, 388행 파일 표:** 만들기·고치기 구분이 없다. 바뀔 파일도 빠져 있다(`core/scene.py`, `config/scene.yaml`, `service/arm_service.py`, `tests/test_mujoco_model.py`, `measurements.md`). 13.10.2 완료 기준에는 시험 파일 이름이 없다.
- **숫자와 단계가 서로 다르다.**
  - 호출 수가 285행은 약 49곳, 369행은 약 47곳이다. 실제 grep 결과는 46줄이다.
  - 369행은 "부하에 따라 흔들린다"를 사실처럼 쓰지만, 원인은 아직 확인하지 않았다.
  - 386행은 "프로세스 병렬은 이 단계 뒤"라고 하지만, 356행은 이미 `--jobs`를 M5에 둔다.
- **383, 385행:** 기준 2는 어느 시험이나 실행기로 돌리는지와 그리퍼를 적지 않았다. 병렬 손의 기준은 4.0mm `measure` 시험이다. 기준 4의 "한 값으로 모이면"에는 문턱(예: 범위 0.01mm)이 없다.
- **표현:** 382행 "빠른 묶음"을 `pytest -m "not slow and not measure"`로 바꾼다. 묶음(suite)과 헷갈린다. 315·320·331·349행의 "알려 주는 값", "알려 준 값", `given`, "어긋남"은 `given` 하나로 통일한다. `place_tol_mm` 10mm와 13.7 (b)의 7mm가 다른 이유를 적는다.

## 확인하지 못한 것

- 시험은 돌리지 않았다.
- `arm-software-spec.md` 13.5~13.7은 `mujoco-spec.md`로 옮겨졌다는 안내 줄만 확인했다.
- 결정 0062의 8행이 아직 `arm-software-spec` 13.10을 가리킨다. 0062는 결정 기록이라 고치지 않는다(참고만).
