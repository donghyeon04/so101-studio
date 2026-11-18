상태: 초안 (총괄 → 기획, 2026-11-18, main e2629a1. mujoco-spec 13.10.2 시계 주입 EGM Stage 9 보고. 사용자가 12/4 뒤로 미뤘던 시계 주입을 앞당겨 달라고 함(수집·평가 시간 단축). 원본 `runs/egm/2026-11-18-stage9-clock/`)

총괄 덧붙임: 사용자는 시뮬레이션을 늘 뷰어로 보길 원한다. MuJoCo 수동 뷰어는 StepClock과 같이 쓸 수 없다(13.10.2 386행). 하지만 MuJoCo 세션이 만든 `--viewer grid`는 화면 밖 render로 위 카메라 영상을 띄우므로, 틱 콜백(`every`)에서 몇 프레임마다 그리면 StepClock과도 같이 쓸 수 있을 것으로 본다(확인 안 함). 이것을 절에 넣을지 정해 달라.

# EGM Stage 9 구현 준비도 보고서: 13.10.2 M5b 첫 단계 시계 주입

**결론: 지금 문서로는 구현할 수 없다. 꼭 고칠 것은 9개다.**

- 가장 큰 구멍은 둘이다.
  - 384행 "보조 맞추기"(깨어 있는 스레드 수를 `ArmService` 공개 메서드로 센다)는 그대로 만들면 첫 실행에서 멈춘다. 틱 스레드가 상태 본문에서 `self.status()`를 부르기 때문이다(`arm_service.py:269`). 공개 메서드끼리 서로 부르는 것도 문제다(`pick_and_place` → `enable_torque`, `:754`).
  - `PolicyRunner`(25Hz 추론), ACT 기록기, 시험장 표본기·판정 전 기다림이 StepClock에서 어떻게 도는지가 절에 없다. 이것들은 지금 모두 벽시계 스레드라 "같은 씨앗이면 같은 결과"(기준 3)를 만족할 수 없다.
- 머리(374행)와 "미루는 이유"(377행)는 낡았다. 사용자가 일을 지금으로 앞당겼고, 목적 넷 가운데 둘만 적혀 있다.
- "46줄"은 지금 **50줄**이다(motion.py 39, skills.py 4, arm_service.py 7. `import time` 제외). 시계 목록·허용 목록에 빠진 줄이 많다.
- 절이 인용한 이름은 대부분 지금 코드에 있다: 재진입이 안 되는 락, 상태 스레드 위치, `advance`·`realtime`·`viewer`, `RealClock`·`SimClock`. 다만 `PoseSample.t_ns`는 `follow_path`가 아니라 `_tick`의 `tap`이 찍는다.
- 실물 팔 기본 동작(WallClock)이 같다는 것을 지키는 시험이 모자라다. 기존 시험 둘(`tests/test_motion.py:150, 178`)은 `motion.time`을 바꿔 끼워 시간을 조작한다. 시계로 바꾸면 깨진다.

대상: `docs/planning/mujoco-spec.md` 374~397행(main b06b689). 코드는 읽기만 했고 돌리지 않았다. 리뷰어 셋 모두 Goldfish로 따로 돌렸다.

## 리뷰어 상태
| 리뷰어 | 상태 | 찾은 것 |
|---|---|---|
| `egm-implementation-readiness` | completed | Blocker 2, Major 9, Nit 4, 시간 호출 다시 세기 표, 작업 나누기 |
| `egm-codebase-details` | completed | Blocker 5, Major 6, Nit 6, 시간 호출 다시 세기 표 |
| `egm-style` | completed | Blocker 4, Major 12, Nit 5 |

리뷰어끼리 엇갈린 점은 아래처럼 정했다.
- **참여 스레드를 세는 방법.** style은 공개 메서드 단위로 세고 중첩은 한 번만 세는 규칙을 냈다. readiness·codebase는 공개 메서드를 감싸지 말고, 시험장 주 스레드처럼 명시적으로 등록한 스레드만 세자고 했다. 뒤쪽을 쓴다. 공개 메서드 단위로 세면 틱 스레드가 자기 자신을 기다리고, 실물 경로에도 새 잠금이 들어간다(꼭 고칠 것 3).
- **PolicyRunner·기록기·표본기를 돌리는 자리.** readiness는 `StepClock.every(period_s, fn)`(MuJoCo 영역), codebase는 `MotionController.on_tick` 훅 목록(팔 영역)을 냈다. 결과는 같다. 앞쪽을 쓴다. 팔 파일 변경이 줄고, 상태 루프의 `on_status` 인자도 없앨 수 있다(꼭 고칠 것 4·5).
- **`StepClock.now()`가 읽는 값.** codebase는 `driver.data.time`, readiness는 시계가 스스로 센 시간을 냈다. 뒤쪽을 쓴다. `data`는 연결 전에 None이고 `connect`마다 0으로 돌아간다(`mujoco_driver.py:49, 69`). 대신 codebase의 반올림 지적(스텝 2ms의 정수배만 `advance`)을 함께 넣는다(꼭 고칠 것 6).
- **`RealClock`·`SimClock` 옮기기.** style은 옮기되 `now_ns()`를 남기자고 했다. readiness·codebase는 옮기지 말자고 했다. 옮기지 않는다. 가져다 쓰는 곳이 초음파·GUI 영역에 있고, `SimClock.sleep`은 기다리지 않고 자기 시각을 앞당겨서 뜻이 반대다(꼭 고칠 것 7).

---

## 꼭 고칠 것

### 1. 머리·"미루는 이유"가 낡았고 목적이 다 적혀 있지 않다 (374, 375, 377행)
- 374행은 "구현은 12/4 뒤"다. 같은 문서 299·425행은 이미 "11/30 동결 전 시계 주입"이라 서로 어긋난다. 사용자는 2026-11-18에 지금 하자고 했다.
- 377행은 지금 결정과 반대 논리다. "300 에피소드 약 2시간"도 466행(에피소드 중앙값 10초 목표), #443의 `--jobs`와 맞지 않는다.
- 375행은 목적 가운데 "실시간보다 빠르게"와 "지터인지 가르기"만 적었다.
- 결정 기록은 고치지 않는다. 앞당김은 새 결정으로 남기고 0066은 상태 줄만 바꾼다.
- 제안
  - 374행: "#### 13.10.2 M5b 첫 단계: 시계 주입 (2026-11-16 초안, 2026-11-18 앞당김: 지금 구현한다. 결정 NNNN. 구현 REQ 전에 이 절만 Stage 9를 다시 돌린다)"
  - 375행 대신: "**왜**: 제어가 벽시계에 묶여 있다. 목적은 넷이다. ① 수집·정책 평가·M5 비교를 실시간보다 빨리 돈다. ② `realtime_lag`로 버리는 에피소드를 없앤다. ③ `--jobs`를 물리 코어 절반(362행)보다 크게 쓴다. ④ 같은 씨앗이면 결과가 같다(58mm 미끄러짐 0.8~10mm 편차가 지터인지 물리인지 가른다). 목적마다 완료 기준이 하나씩 있다."
  - 377행 대신: "**받아들이는 위험**(2026-11-18): 팔 핵심 세 파일을 고친다. 기본 시계에서 동작이 같다는 것은 완료 기준 1과 실물 확인 한 번으로 본다(작업 나누기 ①)."

### 2. "46줄"이 50줄이고, 시계 목록·허용 목록을 줄 단위 표로 바꿔야 한다 (375, 382, 388행)
- 다시 센 결과(`import time` 제외, `from time import` 별칭은 없음). 두 리뷰어가 같은 값을 냈다.

| 파일:행 | 쓰임 | 분류 |
|---|---|---|
| motion.py:128, 207, 242 | execute·retarget·stop의 시작 시각 | 시계 |
| motion.py:163, 164, 171, 581, 582 | 조그 입력 시각, 데드맨, 조그 dt | 시계 |
| motion.py:187, 469, 572 | `_blocked_until` | 시계 |
| motion.py:478 | `check_blocked`의 `BLOCKED_ZONE_S` | 시계 |
| motion.py:191, 632 | 서보 입력 시각, `SERVO_TIMEOUT_S` | 시계(**절에 없음**) |
| motion.py:195 | 서보 거부 기록 1초 묶음 | 시계(**절에 없음**. 이벤트 수가 같아야 한다) |
| motion.py:120, 296, 298, 333, 377, 568, 569, 627 | `HealthMonitor` comm_ok·comm_failed | 시계 |
| motion.py:365 | `_check_health`(`HEALTH_PERIOD_S`, 부하·과열) | 시계 |
| motion.py:523 | `_tick`의 `MAX_TICK_ADVANCE` | 시계 |
| motion.py:249, 250, 251 | `wait_idle` | 시계 |
| motion.py:313, 316, 340 | 틱 간격, `time.sleep` | 시계(`tick_sleep`) |
| motion.py:125, 261, 352, 655 | `MotionResult.duration_s` | 시계(**절에 없음**) |
| motion.py:562 | `trace(perf_counter_ns(), q)` → PoseTrail → 초음파 `sensor_at` | 시계(**절에 없음**) |
| motion.py:564, 566 | `tap`: `(r0+r1)//2`가 `PoseSample.t_ns`, `r1−r0`가 읽기 걸린 ns | 시각은 시계, 걸린 ns는 벽시계 |
| motion.py:339 | `ring.add(time.time_ns(), …)` | 벽시계(기록 시각) |
| skills.py:347, 358 | 접촉 폴링 `sleep(0.04)`, 닫은 뒤 `sleep(0.15)` | 시계 |
| skills.py:997, 1000 | `ARM-OPENING-PATH`의 `actual_s` | 시계(**절에 없음**) |
| arm_service.py:229 | `ArmStatus.t = time.time()` | 벽시계(기록 시각) |
| arm_service.py:257, 284 | 상태 루프 간격 | 벽시계 전용(StepClock에서는 꼭 고칠 것 5) |
| arm_service.py:291, 384 | 탐침·사고 묶음 쿨다운 | 벽시계 |
| arm_service.py:265, 335 | `_volt_hist`, 토크 풀림 진단 10초 창 | 벽시계(**허용 목록에 없음**) |

- #444 접근 각도 코드(`skills.py:218-260`, `arm/approach.py`)에는 시간 호출이 없다. 계산만 길다(꼭 고칠 것 6의 5초 규칙과 관계).
- 388행 허용 목록의 "기록 스레드", "감시 heartbeat", "교시 `record`"는 세 파일 밖에 있다(`arm/teach.py:130-143`, `watchdog/`). 정적 검사가 볼 줄이 아니다.
- 제안
  - 375행의 "46줄" → "50줄(2026-11-18 main b06b689 기준. 아래 표)"
  - 382행 대신: "`PoseSample.t_ns`와 `trace`의 시각은 `MotionController._tick`에서 `clock.now_ns()`로 찍는다(`tap`의 첫 인자). `follow_path`는 그 값을 담기만 한다. `tap`의 셋째 값(읽기 걸린 ns)은 `time.perf_counter_ns()` 차이 그대로다."
  - 388행 대신 위 표를 절에 넣는다. 벽시계 줄마다 이유를 한마디 적는다(예: "쿨다운은 탐침·사고 묶음만 막고 제어 판단에 들어가지 않는다"). `perception/base.py:32`의 주석("time.perf_counter_ns")도 같이 고친다.

### 3. "보조 맞추기"를 공개 메서드로 세면 멈춘다. 명시적 참여로 바꾼다 (384행)
- 지금 코드
  - StepClock이면 상태 본문이 틱 스레드 안에서 돈다. 그 본문이 `self.status()`(공개 메서드)를 부른다(`arm_service.py:269`). 틱 스레드는 수가 0이 되기를 기다리며 자기 자신 때문에 멈춘다. 기록기도 `svc.status()`를 부른다(`mujoco_record.py:54`).
  - 공개 메서드는 서로 부른다(`pick_and_place` → `enable_torque`, `arm_service.py:754, 804` 등). 단순히 세면 2가 되고, 안쪽이 기다려도 1이 남아 멈춘다.
  - 일은 공개 메서드 밖에서도 한다: 기록기 `render`·`add_frame`(`mujoco_record.py:59, 66`), `PolicyRunner` 관측·추론(`policy_runner.py:98-104`), 시험장 `make_given`(`mujoco_bench.py:268`). 그동안은 수가 0이라 시뮬레이션이 OS 스케줄대로 흐른다. 이 설계가 없애려던 바로 그 문제다.
  - 등록된 스레드가 없으면 조건이 저절로 참이다. `connect`와 `pick_and_place` 사이에 시간이 아무만큼 흐른다.
  - `disconnect` → `ctl.close()`의 `join(timeout=1.0)`(`motion.py:650`)은 틱 스레드가 기다리는 동안 1초를 다 쓰고 스레드를 남긴다. 에피소드마다 하나씩 쌓인다.
  - `MotionHandle.wait`는 그냥 `Event.wait`다(`motion.py:33-42`). 기다림에 들어갔다는 것을 시계가 알 길이 없다.
- 제안(384행 대신)
  > "**보조 맞추기**: `StepClock`은 참여 스레드 집합을 갖는다. `StepClock`을 만든 스레드(시험장 주 스레드)는 저절로 참여하고, 다른 스레드는 `with clock.participate():`로만 참여한다. 참여 스레드는 `clock.sleep`·`clock.wait(event, timeout)` 안에 있을 때만 기다림으로 센다(스레드마다 깊이를 세므로 중첩해도 된다). 틱 스레드는 참여 스레드가 하나 이상이고 모두 기다리는 동안에만 `advance`하고, 기다리는 동안에도 `_quit`을 본다. `MotionHandle.wait`·`wait_idle`·`skills.py`의 폴링과 닫은 뒤 기다림은 모두 `clock.wait`·`clock.sleep`을 쓴다(`teach.py:244, 247`도 함께 맞춰진다). `ArmService` 공개 메서드는 감싸지 않는다. `WallClock.participate()`는 아무것도 하지 않는다. `MotionController.close`는 `_quit.set()` 뒤 `clock.wait(self._loop_done, 1.0)`으로 기다리고, `_loop`는 끝날 때 `_loop_done`을 세운다."
  - 동기 한 줄은 따로 남긴다: "이것이 없으면 IK 계산·접촉 폴링 동안 시뮬레이션 시간이 얼마나 흐를지를 OS 스케줄이 정해, 결과가 실행마다 달라진다."

### 4. `PolicyRunner`·ACT 기록기·시험장 표본기가 StepClock에서 어떻게 도는지 없다 (381~388행 사이, 13.10.4 461·479행)
- 지금 코드. 모두 벽시계 스레드다.
  - `PolicyRunner._run`(`policy_runner.py:86-121`): `max_s`, `SLOW_S`, 추론이 한 주기보다 길면 건너뜀(:99-105), `REJECT_S` 0.5초(:108-115), 25Hz 간격 `_quit.wait`(:116-117), `JUDGE_DELAY_S`(:121)
  - `EpisodeRecorder._run` 25fps(`mujoco_record.py:62-73`)
  - `Sampler._run` `time.sleep(SAMPLE_S)`(`mujoco_bench.py:46-53`)
  - 첫 프레임 기다림(:332-334), 판정 전 `time.sleep(settle_s)`(:341), 정책 결과 폴링 `time.sleep(0.2)`(:379)
- 388행은 "기록 스레드"를 벽시계 허용 목록에 넣었다. 461행은 "기록기를 `ArmService`의 시계로 바꾼다"다. 서로 어긋난다. `ArmService`에는 시계를 꺼낼 공개 속성도 없다.
- 추론 지연을 시뮬레이션 시간으로 어떻게 칠지(0, 고정값, 잰 값)는 모형 결정이다. 실물과 비교한 성공률이 달라지므로 문서가 정해야 한다.
- 제안(386행 뒤 새 묶음 "**시계를 쓰는 다른 곳**")
  > "`ArmService.clock`(읽기 전용)으로 받은 시계를 드러낸다. `StepClock.every(period_s, fn)`: 틱 스레드가 `tick_sleep`에서 `advance`한 직후, 시뮬레이션 시간이 `period_s`의 배수를 지날 때마다 `fn()`을 등록 순서대로 부른다(락 밖. 예외는 잡아 `SIM-CLOCK-CALLBACK-ERROR`로 남기고 계속한다). StepClock에서는 표본기(50Hz), 기록기(25fps, `frame()`과 `add_frame`), `PolicyRunner` 한 걸음(관측 → 추론 → `servo_joints`)을 모두 `every`로 돌리고 따로 스레드를 띄우지 않는다. 그래서 추론 지연은 시뮬레이션 시간으로 0이다. `infer_ms`는 벽시계로 기록만 하고, '한 주기 넘긴 추론 건너뛰기'와 `SLOW_S`는 StepClock에서 쓰지 않는다. `REJECT_S`·`max_s`·`JUDGE_DELAY_S`와 시험장의 `settle_s`·결과 폴링은 `clock.now()`·`clock.sleep`으로 잰다. 첫 프레임 기다림은 StepClock에서 뺀다. WallClock(GUI·실시간)에서는 지금 스레드 그대로다. 388행의 '기록 스레드'는 `events.jsonl`·`status.jsonl`을 쓰는 스레드만 뜻한다."
  - 461행은 이 묶음을 가리키게 고친다. `PolicyRunner`는 팔 영역이다(`service/`). 한 걸음을 메서드(`step()`)로 떼어 내는 일은 팔 세션이 한다.

### 5. 상태 루프 `on_status`를 생성자 인자로 넘기면 `connect`와 경합한다 (385행)
- 지금 코드
  - `MotionController`는 `__init__` 안에서 틱 스레드를 바로 시작한다(`motion.py:104-105`).
  - `ArmService.connect`는 `ctl`을 `:143`에서 만들고 `skills`는 `:145`, `teach`는 `:154`에서 넣는다. 첫 틱에서 `on_status` → `status()`가 아직 없는 `self.teach.recording`(`:232`)을 건드린다.
  - 상태 본문은 2Hz 건강 읽기를 위해 셈 `n`을 쓴다(`:257-283`).
  - `MotionController`에는 `control_hz`라는 이름이 없다. 인자는 `hz`, 저장은 `self.dt`다(`motion.py:64-65`). 10은 `STATUS_HZ`(`arm_service.py:35`)다.
- 제안(385행 대신)
  > "`_status_loop` 본문 한 바퀴를 `_status_once()`로 떼어 낸다(2Hz 셈은 `self._status_n`). `_status_loop`도 이것을 부른다. `connect`는 상태 스레드를 띄우던 자리(`_write_status` 다음 줄)에서 `clock.drives_status`이면 스레드를 띄우지 않고 `clock.every(1 / STATUS_HZ, self._status_once)`를 등록한다. `MotionController`에는 `on_status`를 두지 않는다. 벽시계에서는 지금 그대로다."

### 6. `StepClock.now()`의 출처, 틱 반올림, 5초 `RuntimeError`가 정해져 있지 않다 (383행)
- 드라이버에는 시뮬레이션 시간을 주는 공개 속성이 없다. `data`는 `connect`마다 새로 만들어져 0에서 다시 시작한다(`mujoco_driver.py:49, 69`). `_blocked_until`·`_next_health` 같은 시한은 시간이 앞으로만 간다고 가정한다.
- `advance`는 `round(seconds / 0.002)` 스텝이다(`mujoco_driver.py:221`). `control_hz`는 10~100에서 5씩 바뀐다(`params.py:21`). 30Hz면 한 틱이 2ms의 정수배가 아니라 오차가 쌓인다.
- "벽시계 5초 동안 시간이 안 흐르면 `RuntimeError`"는 참여 스레드가 오래 계산하면 잘못 걸린다: `auto` 접근 각도 IK 탐색, 첫 `render`(GL 만들기, `mujoco_driver.py:152`), `load_act`와 첫 추론, 영상 인코딩. 반대로 물리 실패는 이미 `advance`를 멈춘다(`:220`).
- 제안(383행 대신)
  > "`StepClock.now()`는 시계가 스스로 센 시간이다(실제로 `advance`한 스텝 수 × 스텝, 0에서 시작). `driver.data.time`을 읽지 않는다. `tick_sleep(s)`는 남은 몫을 쌓아 두고 정수 스텝만 `advance`한다. `now_ns()`는 `int(now() * 1e9)`다. 기다리는 스레드는 `driver.failed_reason`이 생기면 바로 `RuntimeError`(물리 시뮬레이션이 멈췄습니다: …)를 낸다. 벽시계 5초 규칙은 참여 스레드가 모두 기다리는데 틱 스레드가 5초 동안 `tick_sleep`에 한 번도 들어오지 않을 때만 걸린다(틱 스레드가 죽었거나 막힘). 참여 스레드가 계산하는 시간은 세지 않는다."

### 7. `RealClock`·`SimClock`은 옮기지 않는다 (380행)
- 규약이 다르다. 둘 다 `now_ns() -> int`다(`perception/ultrasonic/model.py:36-39`, `fake_esp32.py:10-18`). `SimClock.sleep`은 기다리지 않고 자기 시각을 앞당긴다. 이 절의 "다른 스레드의 `sleep`은 기다린다"와 반대다.
- 가져다 쓰는 곳이 다른 세션 영역이다: `service/ultrasonic.py:12`, `perception/ultrasonic/link.py:6`, `benchmark.py:32`, `gui/pages/esp32.py:14`, `gui/sensors/ultrasonic_view.py:16`(`:204`에서 `SimClock.sleep`을 묶지 않고 부름), `tests/perception/` 세 파일. CLAUDE.md는 상대 영역을 직접 고치지 말고 REQ를 보내라고 한다.
- 제안(380행 둘째 문장 대신)
  > "`Clock` 규약에 `now_ns() -> int`를 더한다(`WallClock.now_ns = time.perf_counter_ns`, `RealClock`과 같은 값). 규약은 `now`, `now_ns`, `sleep`, `wait(event, timeout)`, `tick_sleep`(틱 스레드만), `participate()`, `drives_status`다. 초음파의 `RealClock`·`SimClock`은 이 단계에서 옮기지 않는다. 13.10.3에서 가상 ESP32에 `StepClock`을 그대로 넘긴다."

### 8. 완료 기준이 실물 동작을 지키지 못하고, 목적 ②③④를 재지 않으며, 돌릴 수 없는 이름을 쓴다 (391~394행)
- 지금 코드
  - `tests/test_motion.py:150, 178`은 `monkeypatch.setattr(motion, "time", SimpleNamespace(perf_counter=…))`로 늦은 틱을 흉내 낸다. 시계로 바꾸면 이 둘은 시간을 못 움직인다. 기준 1 "그대로 통과"가 될 수 없다.
  - "직접 부르는 곳이 남지 않았는지 보는 시험"의 방법이 없다.
  - `carry45`·`carry58`은 13.5 시나리오 파일이다(`config/sim_scenarios/carry45.yaml`). `mujoco_bench --suite`는 `config/sim_scenarios/bench/`만 읽는다(`mujoco_bench.py:92`). 병렬 carry58은 지금 `tests/test_mujoco_parallel.py::carry_run`(101-130)으로 돌고 `results.jsonl`이 없다. "자리 ±2mm"를 줄 `spread` 묶음도 없다.
  - `mujoco_bench`에 시계를 고르는 옵션이 없다. `BenchEnv(realtime=…)`는 있지만 아무도 넘기지 않는다(`:242, 261, 320, 370, 619-625`).
  - "벽시계로 실시간 모드의 1/5 이하다(목표. 못 미치면 …)"는 합격선이 아니다.
  - 목적 ②(`realtime_lag` 0), ③(`--jobs` > 6), 정책 평가가 같은 씨앗에서 같은지를 재는 기준이 없다.
- 제안(391~394행 대신)
  > "1a. 기본 시계에서 `python -m pytest -q -m "not slow and not measure"`가 모두 통과한다. `tests/test_motion.py`의 늦은 틱 시험 둘은 `motion.time`을 바꾸는 대신 `MotionController(..., clock=ManualClock())`(시험 전용, `tests/test_clock.py`에 둔다)로 고친다.
  > 1b. `tests/test_clock.py`는 세 파일을 AST로 읽어 `time.*` 호출을 위 표의 벽시계 줄(파일, 함수, 속성)과 대조한다. 기본 시계의 `ArmService`(`SimDriver`)가 `connect` 뒤 상태 스레드를 띄우는지, `WallClock.now()`가 `time.perf_counter()`와 같은 축인지도 본다.
  > 1c. 실물 확인(팔 PR을 합친 뒤 한 번, runbook대로. 하드웨어라 사용자에게 먼저 묻는다): home, `pick_and_place` 한 번, 조그 키를 떼고 데드맨 정지, USB를 뽑아 `comm_lost`, E-STOP.
  > 2. `mujoco_bench`에 `--clock wall|step`(기본 `wall`)을 더한다. `step`이면 `BenchEnv`가 `realtime=False` 드라이버와 `StepClock`을 만들어 `ArmService(..., clock=)`에 넘기고, `--viewer`와 함께 주면 거부한다. 묶음 `bench/carry.yaml`(45·58 상자, 정면 (300, 200), `place: {at: [420, 170]}`, `spread: {xy_mm: 2}`, 병렬 손)을 만든다. `--suite carry --only carry45-… --clock step`이 `placed_ok`, `place_err_mm` < 7, `tilt_end_deg` < 10이고, `wall_s`가 같은 씨앗 `--clock wall`의 1/5 이하다. 넘으면 실패이고 PR에 두 `wall_s`와 막는 곳을 적는다.
  > 3. `pick_smoke`를 같은 씨앗으로 `--clock step --jobs 1`과 `--jobs 8`로 돌리면 `reported`·`outcome`·`agree`가 같고 `slip_mm`·`place_err_mm`가 0.01mm 안이다.
  > 4. `carry58` 같은 씨앗 14번과 `spread` 씨앗 14번의 `slip_mm` 최소·중앙·최대를 `measurements.md`에 남긴다(U15의 입력).
  > 5. `--clock step`에서는 `realtime_lag`가 늘 false다(`dropped_count`는 실시간 스레드에서만 오른다, `mujoco_driver.py:227-232`). 13.10.4의 'lag 에피소드 저장 안 함'은 걸리지 않는다.
  > 6. `collect_pick1` 50 에피소드를 `--clock step --jobs 12`로 돌린 `wall_s` 합계를 `measurements.md`에 남긴다.
  > 7. 정책 평가(`--policy`) 10 에피소드를 같은 씨앗·`--clock step`으로 두 번 돌리면 `outcome`이 같다(torch 스레드 수를 1로 고정한 상태에서)."
  - 해석 문장("범위가 0.01mm 안이면 지터였다")은 기준 목록 밖으로 뺀다.

### 9. 파일·주인·순서가 모자라다 (397행)
- 빠진 파일: `arm/mujoco_clock.py`, `arm/mujoco_bench.py`, `arm/mujoco_record.py`, `service/policy_runner.py`, `arm/teach.py`(`MotionHandle.wait`), `perception/base.py:32`(주석), `tests/test_motion.py`, `tests/test_mujoco_parallel.py`, `config/sim_scenarios/bench/carry.yaml`, 13.3 문장.
- "순서는 총괄이 정한다"는 정할 것을 남긴다.
- `Skills(..., clock=)`는 필요 없다. `Skills`와 `Teach`는 `self.ctl.clock`을 쓰면 서명 변경이 하나 준다.
- 제안(397행 대신)
  > "**파일과 주인**: 팔 세션 = 만들기 `core/clock.py`, `tests/test_clock.py`. 고치기 `arm/motion.py`, `arm/skills.py`, `arm/teach.py`, `service/arm_service.py`, `service/policy_runner.py`, `perception/base.py`(주석), `tests/test_motion.py`. MuJoCo 세션 = 만들기 `arm/mujoco_clock.py`, `tests/test_mujoco_clock.py`, `config/sim_scenarios/bench/carry.yaml`. 고치기 `arm/mujoco_bench.py`, `arm/mujoco_record.py`, `tests/test_mujoco_parallel.py`. 초음파 영역은 고치지 않는다. 순서는 아래 작업 나누기 ①→②→③이다."

---

## 작업 나누기와 순서
각 단계가 따로 합쳐져도 빌드·시험이 깨지지 않는다.

1. **① 팔 PR(동작 변화 없음)**, 팔 세션
   - `core/clock.py`: `Clock` 규약과 `WallClock`(그대로 넘겨 주기, `participate()`는 아무것도 안 함).
   - `MotionController(clock=WallClock())`와 꼭 고칠 것 2 표의 "시계" 줄 전부. `MotionHandle.wait`·`wait_idle`·`close`가 `clock.wait`를 쓴다.
   - `Skills`·`Teach`는 `ctl.clock`. `ArmService(clock=)`, `ArmService.clock`, `_status_once` 떼어 내기(상태 스레드는 그대로).
   - `PolicyRunner.step()` 떼어 내기(벽시계 스레드는 그대로 `step()`을 부름).
   - 시험: `tests/test_clock.py`(AST 대조, `ManualClock`), `tests/test_motion.py` 두 시험 고치기, 빠른 묶음 전체.
   - 합친 뒤 실물 확인 1c(사용자에게 먼저 묻는다).
2. **② MuJoCo PR: StepClock**, MuJoCo 세션
   - `arm/mujoco_clock.py`: 스스로 센 시간, 참여 스레드, `every`, 실패·5초 규칙, 닫기.
   - `tests/test_mujoco_clock.py`는 가짜 드라이버로 본다: 참여 스레드가 깨어 있으면 틱이 기다림, `advance` 뒤 `sleep`이 깨어남, 중첩 참여가 멈추지 않음, `close`가 걸리지 않음, `failed_reason`이면 바로 오류.
3. **③ MuJoCo PR: 시험장·기록기·평가 연결**, MuJoCo 세션(`PolicyRunner`는 ①에서 준비됨)
   - `--clock step`, 표본기·기록기·`PolicyRunner`를 `every`로, settle·폴링을 `clock.sleep`으로.
   - `bench/carry.yaml`, 완료 기준 2~7.

①은 실물 팔이 오는 주에 들어가므로 1b·1c를 빼고 합치지 않는다. ②는 ①과 나란히 만들 수 있다(가짜 드라이버만 쓴다).

---

## 고치면 좋은 것
- **뷰어와 StepClock**(386행): 뷰어 검사는 이미 `MuJoCoDriver`에 있다(`mujoco_driver.py:31-32`). StepClock은 `driver.realtime`만 보면 된다. 13.3에 더하려는 문장도 `mujoco_driver.py:4` 독스트링에 있다. 사용자는 시뮬레이션을 돌릴 때 뷰어를 늘 보고 싶어 한다. `--clock step`에서는 뷰어를 못 켠다는 것을 절에 한 줄 적고, 볼 때는 `--clock wall --viewer`로 한 에피소드를 다시 돌린다고 적는다.
- **`--jobs` 6 상한**: 코드에는 상한이 없다(`mujoco_bench.py:619`, 그냥 int). 362행의 "물리 코어 절반" 규칙뿐이다. "StepClock에서는 362행의 실시간 제한이 걸리지 않는다"를 한 줄 더한다.
- **GC 끄기·`encoder_threads=1`**(#443, `mujoco_record.py:46, 84-85, 96`): 실시간 지연을 피하려던 장치다. StepClock에서도 그대로 둔다고 적는다.
- **용어 정의**(381행 앞): "제어 시간 = 동작·판정·기다림의 길이와 시한. 기록 시각 = 로그·데이터 줄에 찍는 시각. 틱 = `_loop` 한 바퀴. 틱 스레드 = `_loop`를 도는 스레드." 380행의 "제어 틱 스레드"를 "틱 스레드"로 맞춘다.
- **틱 스레드 아닌 곳에서 `tick_sleep`**: "틱 스레드가 아닌 스레드가 부르면 `RuntimeError`다"를 더한다.
- **"보조 맞추기" 이름**: 이 절에서만 쓰는 말이다. "(lockstep)"을 붙이거나 "번갈아 돌기"로 바꾼다.
- **약어**(374행 아래): M5b, U15(`learning-platform.md`의 병렬 손 미끄러짐 기준), `carry45`·`carry58`, Stage 9 (가)를 한 줄로 풀어 쓴다.
- **395행**: 범위 밖 문장이다. 완료 기준 아래가 아니라 머리 아래 "**범위 밖**"으로 옮긴다.
- **384행 끝**: "MuJoCo 자체는 입력이 같으면 결과가 같다" → "MuJoCo는 같은 빌드·같은 기계에서 입력이 같으면 결과가 같다."
- **교차 참조**: 351행 "13.10.2의 결정 스텝 결과" → "13.10.2 완료 기준 4". 366행 "13.10.2 뒤에 본다", 428행 "2차 = 12/4 뒤"도 꼭 고칠 것 1에 맞춘다.
- **`int(perf_counter() * 1e9)`**는 `perf_counter_ns()`와 비트까지 같지 않다. `now_ns()`를 규약에 두면(꼭 고칠 것 7) 사라진다.
- **`task/runner.py`의 시간 호출 5곳**은 13.10.3(초음파를 MuJoCo에) 범위다. 이 절에서는 다루지 않는다고 적는다.

---
확인하지 않은 것
- 코드를 돌리지 않았다. 50줄은 리뷰어 둘이 grep으로 각각 센 값이고 서로 같다. 46이 #444(`skills.py:997, 1000`) 전 수라는 추정은 git으로 보지 않았다.
- GUI 백엔드가 `ArmService`를 어느 스레드에서 부르는지, `RunLogger`가 배경 스레드를 쓰는지는 보지 않았다. 둘 다 WallClock 경로라 이 단계 동작에는 영향이 없을 것으로 본다.
- 제안한 새 이름(`participate`, `every`, `_status_once`, `--clock`, `bench/carry.yaml`, `ManualClock`)은 리뷰어 제안이다. 기획 세션이 확정한다.
