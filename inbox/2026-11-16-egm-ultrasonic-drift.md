상태: 초안 (EGM drift-report 결과, ultrasonic-spec.md 1·5·6·8.3·11절 vs 11-15·16 PR(#378~#399), main 9a9a7ab, 2026-11-16)

# ultrasonic-spec drift-report (오늘 바뀐 절)

**결론: 꼭 고칠 것 4건, 모두 문장·표 머리 수준이다.** U16 설계를 쓸 때 같이 고치면 된다. 총괄이 3번(설정 값 75)과 GUI 항목을 코드로 확인했다.

1. 5절 5번 마지막 문장 "벤치마크는 이 거르기를 쓰지 않는다"는 지난 일이다. `benchmark.py`가 이미 `policy.choose(unconfirmed=True)`를 쓴다(#391).
2. 6절 "모서리 간격 조건을 넣은 `demo` 변형을 다음 갱신 때 잰다"도 지난 일이다. `demo80`이 있고(#384) 1절 표에 값이 있다.
3. 1절 표 머리가 "확인 전 후보 거르기 +60mm"의 출처를 #391로 적었다. 실제 동작은 #386(`policy.py`·`runner.py`·`workspace.yaml`), #391은 벤치마크 반영이다.
4. 병렬 표는 `tool_reach_mm` 70~250으로 잰 값인데 코드는 75~250이다(#399, `config/ultrasonic.yaml`). "75 반영 전 값"으로 적거나, 75로 다시 잰 값(같은 파일 benchmark-final 끝: 씨앗 60 demo 94/72)으로 바꾼다. "최소를 75로 올린다"는 "올렸다"로.

고치면 좋은 것(요약, 원문 아래):
- 6절에 `pick_yaw`가 지금 `yaw_deg`에 가까운 해를 고른다는 규칙이 없다. 8.3 설정 예에 `yaw_deg`·`servo`가 없다.
- `candidate_reach_margin_mm`이 `workspace.yaml`에 있고 최대 반경에만 더한다는 것을 설계서가 적지 않았다.
- #394 `_straighten`은 원·격자만이 아니라 잘린 다꼭짓점 경로 전체에 적용된다.
- 1절에 REQ-066이 #392로 고쳐졌고 #399가 벤치마크 우회 코드를 지웠다는 내용이 없다. 벤치마크 `--seed-timeout` 도움말은 아직 "placo가 멈춘다"고 적는다(초음파 세션 몫).
- 8.5절에 `demo80`·`pair` 시나리오와 `--pair-gap`·`--reach`·`--gripper`·`--trim-start`·`--seed-timeout`이 빠졌다.
- 5절 레이더 후보 90% 오차 56~63mm와 1절 56~66mm가 다르다.
- **GUI(총괄 몫, 설계서 변경 없음)**: 초음파 화면의 가상 실행(`gui/sensors/ultrasonic_view.py` `Demo._run`)이 `benchmark.build(boxes)`를 그리퍼 없이 불러, 병렬 팔에서도 단동식 도달 범위 60~255로 그린다. 표시용 가상 실행(REQ-025 shortcut)이라 작업 실행에는 영향 없다. 다음 총괄이 `gripper=backend.arm.cfg.gripper`를 넘기게 고친다.

돌린 방법: EGM drift-report 절차를 Sonnet 하위 에이전트 한 명이 읽고 따랐다(정식 리뷰어 무리 아님). 시험·python은 돌리지 않았다. 벤치마크 결과 JSON은 낙관 성공률만 대조해 일치했다.

---
원문:

# EGM drift-report 2026-11-16 (초음파 설계서)

기준: 설계서 = origin/main(9a9a7ab)의 `docs/planning/ultrasonic-spec.md`(줄 번호는 그 파일 기준), 코드 = 이 worktree HEAD 1011f60. 읽기만 했다(pytest·python 실행 없음). U16(경로 첫 점 당기기)은 구현 전이라 drift에서 뺐다.

## ultrasonic-spec.md 1·5·6·8.3·11절

### 꼭 고칠 것

- **5절 5번 마지막 문장이 이미 지난 일이다.** 설계서 198줄 "벤치마크도 같은 여유로 후보를 거르게 맞춘다(지금은 이 거르기를 쓰지 않아 …)". 코드는 `perception/ultrasonic/benchmark.py:349`에서 `policy.choose(..., unconfirmed=True)`를 쓰고, 확정 위치가 집기 범위 밖이면 그 후보를 빼는 것도 `runner.py:199`와 같게 한다(`benchmark.py` 같은 근처의 `policy.reachable`). 근거 PR #391(벤치마크 정책 통일), 동작 자체는 #386. 현재형으로 고친다.
- **6절 219줄 "`demo` 변형을 다음 벤치마크 갱신 때 함께 잰다"도 지난 일이다.** `demo80`은 `benchmark.py:52`에 있고(#384) 1절 표 14·15·24~25줄에 이미 값이 있다. 이 문장을 "`demo80`(모서리 80mm)으로 쟀다"로 바꾼다.
- **1절 표의 "확인 전 후보 거르기 +60mm(#391)" 출처 표기가 부정확하다.** 설계서 10줄(표 머리). 실제 동작(`policy.py:14-19`, `runner.py:175,199`, `workspace.yaml:8`, `core/config.py:103`)은 #386이고, #391은 벤치마크가 같은 정책을 쓰게 한 PR이다. "#386(작업 실행)·#391(벤치마크 반영)"로 적는다.
- **병렬 열의 조건 설명과 현재 설정이 어긋난다.** 설계서 19줄 "`tool_reach_mm` 70~250"으로 쟀고, 29줄 "병렬 최소값은 75로 올린다". 코드는 이미 75다(`config/ultrasonic.yaml:42`, #399). 19~25줄의 병렬 두 열은 70~250 값이므로 "75 반영 전 값, 재측정 안 함"이라고 적거나 다시 잰다. 29줄의 "올린다"는 "올렸다(#399)"로 고친다.

### 고치면 좋은 것

- **6절 202줄 yaw 설명에 코드에만 있는 규칙이 있다.** `check.py:22-37` `pick_yaw`: c를 ±L로 자르는 L은 `hypot(off.x, off.y)`(66.503, 설계서 66.5). 오프셋 방향이 팔→후보 방향에서 acos(c/L)만큼 벗어나게 푸는 yaw 해가 둘이고, **`check.yaw_deg`(90)에 가까운 쪽**을 쓴다. 그래서 `yaw_deg`는 끈 값일 뿐 아니라 해 고르기 기준으로도 남는다. 근거 PR #378. 설계서에 "두 해 중 `yaw_deg`에 가까운 쪽(손목이 덜 돈다)"를 한 줄 넣는다. 설정에서 `yaw_pivot_mm`이 없거나 null이어도 끈 것으로 본다(`check.py:28-29`, 설계서는 0만 적음).
- **8.3 설정 예(313~352줄)에 `check.yaw_deg: 90`이 없다.** 위 규칙과 `sensor_to_tip`(`check.py:14-19`)이 쓰는 키인데, `config/ultrasonic.yaml:29`에는 있다. `servo: {driver: pwm}`(yaml 6줄)도 8.3에 없다(3.2절에는 있음). `tool_reach_by_gripper`(yaml 40~42)는 350줄 주석이 "코드가 기준"이라 허용하지만, 형식(그리퍼 이름 `parallel`/`single_jaw` 아래 `{min, max}`, `model.py:24-33`이 `robot.yaml`의 `gripper` 값으로 고름, #390)을 한 줄 적으면 11절 407줄과 맞는다.
- **5절 5번에서 여유 설정 키의 위치가 안 적혀 있다.** 198줄 "이름은 코드가 기준". 코드는 `workspace.yaml`의 `candidate_reach_margin_mm: 60`(`workspace.yaml:8`)이고 `ultrasonic.yaml`이 아니다. 여유는 최대 반경에만 더한다(`policy.py:18-19`, 최소는 그대로). 키 이름·파일·"최대만"을 적는다.
- **#394 `_straighten`이 설계서보다 넓게 쓰인다.** 설계서 30줄은 "원·실패용 격자가 잘린 조각에서 일직선 위의 중간 점을 뺀다". 코드(`check.py:40-49`, `check.py:154`)는 도달 범위로 잘린 **모든** 다꼭짓점 경로에 쓴다(L3+L4를 이은 `L34`도 포함). 꼭짓점이 2개인 경로는 조각의 처음·끝 점만 남긴다. 일직선이어도 방향이 뒤집히는 점(내적 < 0)과 꺾이는 점은 남긴다. 전부 범위 안이면 원래 꼭짓점을 그대로 한 구간으로 보낸다(`check.py:146-147`). 6절 217줄(도달 범위로 자르기)에 이 규칙을 넣는다. 30줄의 "원·격자" 표현은 "다꼭짓점 경로 전체"로 바로잡는다.
- **1절 31줄이 REQ-066 수정을 안 적는다.** "원인은 `arm/trajectory.py` 경로 계획 버그(REQ-066)". 그 뒤 #392로 `trajectory.py`가 고쳐졌고 #399가 벤치마크의 임시 우회(첫 구간 1mm 이하 점 삭제)를 지웠다(`benchmark.py` `follow_path`의 `trim_start` 부분). "수정됨(#392), 벤치마크 우회 제거(#399)"를 붙인다.
- **벤치마크 쪽 설명이 31줄과 충돌한다.** `benchmark.py:545` `--seed-timeout` 도움말과 `_worker`/`_Guard`의 docstring(#381)은 "placo가 네이티브에서 멈추는 경우"라고 적는다. 설계서 31줄은 placo가 원인이 아니라고 한다. 둘 중 하나를 맞춘다(코드 주석은 초음파 세션 영역).
- **8.5절 벤치마크 설명(354~371줄, 이번 대상 밖이지만 1절이 인용함)이 옵션을 반영하지 않는다.** 코드 `benchmark.py:529-548`에는 시나리오 `demo80`·`pair`와 `--pair-gap`(#379), `--reach radius|kin`, `--gripper single_jaw|parallel`, `--trim-start`, `--tool-reach`, `--hover-mm`, `--seed-timeout`(기본 300초), `--set KEY=VAL`, `--servo`가 있다. 367줄 시나리오 표는 `spread90`·`spread120`·`demo`·`single`만 적는다. `--scenario all`은 `pair`·`demo80`을 뺀다(`benchmark.py:550`). 8.5절에 시나리오 2개와 옵션 요약을 넣는다. 364줄 "손끝 도달 50~265"는 `benchmark.py:39` `REACH`(반경 모델)와 맞다.
- **5절 198줄 "후보 오차 90%가 56~63mm"와 1절 12줄 "90% 56~66mm"가 서로 다르다.** 측정 시점이 다른 값으로 보인다(확인 못 함). 한쪽에 조건을 적거나 숫자를 맞춘다.

### 확인 못 함

- 1절 표 숫자는 `runs/bench/*-a703b28.json`(이 worktree에 있음)의 명령 줄과 `success_mean`만 대조했다. `single`/`demo`/`spread120`/`spread90` 낙관 성공률(97·100·97·92%)과 병렬 당기기 없음·있음 두 열(79/75/82%, 92/93/98%)은 맞았다. 비관 값, 인식 시간 평균·90%, 위치 오차, 방향 정답률은 보지 않았다(요약 JSON을 파싱할 도구를 못 썼다). 파일은 a703b28 커밋이라 설계서의 "main 3fba261"과 같은 코드인지는 확인하지 않았다.
- 28줄 "실제 `follow_path`는 한 점이라도 IK가 안 풀리면 움직이지 않고 그 단계를 건너뛴다"는 팔 쪽 `arm_service`·`trajectory.py` 동작이라 읽지 않았다. `check.py:89-97`은 빈 `poses`를 그렇게 처리한다.
- 확정 위치가 범위 밖일 때 건너뛰는 단위가 다르다: 작업 실행은 후보 id(`runner.py:206` `dropped.add(obj.id)`), 벤치마크는 자리 60mm(`benchmark.py:349`). 설계서는 단위를 적지 않았고, id가 스캔마다 안정적인지는 확인하지 않았다.
- GUI 가상 실행(`gui/sensors/ultrasonic_view.py:210` `benchmark.build(...)`, `:243,301` `load_config()`)은 그리퍼를 안 넘겨 병렬 손이어도 단동식 60~255를 쓴다. 의도인지(GUI 영역) 확인하지 않았다. `service/ultrasonic.py:39`의 실행 경로는 그리퍼를 넘긴다.
- 11절 406·408~411줄은 결정·조직 설명이라 코드 대조 대상이 아니다. 407줄(그리퍼별 75~250)은 코드와 같다.

### 설계서와 코드가 일치한 항목(참고, 차이 없음)

`yaw_pivot_mm` 150·c 식(반경 100 → 손끝 150, 240 → 173.5)·0이면 끔, `ring_r_mm` 60, `arm_sensor_offset_mm` {0.6, 66.5, 99.5}, `candidate_reach_margin_mm` 60, 확정 위치가 `reach_mm` 밖이면 건너뜀(`runner.py:199`), 병렬 75~250·단동식 60~255, `tool_reach_by_gripper` 우선 규칙, `check.clearance_mm` 15·`max_object_height_mm` 40(확인 높이 55).
