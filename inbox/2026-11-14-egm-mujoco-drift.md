상태: 초안 (EGM drift-report 결과, mujoco-spec.md 13.9·13.10 vs #348~#357, 2026-11-14 새 PC, main fcdcb0f)

# mujoco-spec 13.9·13.10 drift-report

**결론: 꼭 고칠 것 4건.** M5·M6 세부 설계를 쓰기 전에 13.9·13.10을 고쳐야 한다. 1·2·4는 총괄이 설계서 줄과 코드를 직접 열어 확인했고, 3은 보고서 근거(파일:줄)만 봤다.

1. **13.9 "병렬 그리퍼는 ValueError"(시나리오 줄)가 코드와 반대다.** #348이 `make_sim_driver`의 거부를 지웠다(`arm/sim_driver.py:57-70`). 병렬이면 로보나인 손을 붙인다.
2. **13.9 뷰어 설명이 다르다.** 설계서: 1/30초 sync, `launch_passive(model, data)`, `xfrc_applied` 0. 코드(#354 뒤): 15Hz(`VIEWER_SYNC_S = 1/15`), 복사본 `_vdata`를 뷰어에 넘기고 `mj_copyData`로 보낸다. `xfrc_applied`를 0으로 두는 코드는 없다(`arm/mujoco_driver.py:24, 92-93, 193-207`).
3. **13.10 M5 행과 판정 목록에 로보나인 손에 없는 규칙이 남아 있다.** `grasp_tilt_tol_deg`·`height_required`는 단동식에서만 쓴다(`arm/gripper.py:157`, `arm/skills.py:305`). 결정 0063 1항과 맞지 않는다.
4. **"로보나인 손 위"·"브래킷 질량 같이 싣고"는 기본 설정에서 켜지지 않는다.** 그리퍼 기본값은 `single_jaw`(`core/config.py:71·184`), 브래킷·센서 부품은 `config/scene.yaml:15-16`에서 주석, `wrist_sonar` site는 병렬일 때만 생긴다(`arm/mujoco_model.py:120-121`). 13.7 공통 cfg(local.yaml 없이)로 M5·M6를 돌리면 단동식 손에 브래킷이 없다. 시험장이 무엇으로 병렬·브래킷을 켜는지(시나리오 키, 시험 전용 cfg 등) 13.10에 정해야 한다.

M5·M6 설계에 바로 걸리는 "고치면 좋은 것" 둘:
- [11] 같은 `ArmService`로 다시 connect하면 `Skills` 기억이 넘어가 에피소드가 섞인다(`service/arm_service.py:143-148`, #352·#356이 넘기는 항목을 늘림).
- [13] 레이더는 MuJoCo site가 없고, 가상 ESP32는 자체 `SimClock`으로 돌아 벽시계로 도는 `MuJoCoDriver`와 시계가 다르다.

돌린 방법: EGM 플러그인은 새 PC에 방금 설치해 `egm:egm-synthesizer`가 세션에 아직 안 잡혔다. 일반 하위 에이전트에 EGM `agents/synthesizer.md`·`skills/egm-drift-report/SKILL.md`를 읽혀 한 번 돌렸다(리뷰어 무리 없이 한 명). 시험·python은 돌리지 않았다(전체 시험이 도는 중). 측정값(FK 0.002mm 등)은 "확인 못 함"이다.

아래는 보고서 원문이다(로컬 `runs/egm/2026-11-14-mujoco-drift/2026-11-14_DRIFT_REPORT.md`).

---

# Drift Report 2026-11-14

대상: `docs/planning/mujoco-spec.md` 13.9(235~251줄)·13.10(253~286줄), 결정 0053·0062·0063
코드 기준: origin/main `fcdcb0f`, PR #348~#357
방법: 파일 읽기·grep·`git diff`만 썼다. 시험·python은 돌리지 않았다.

## mujoco-spec.md 13.9·13.10

### 꼭 고칠 것

- **[1] 13.9 "병렬 그리퍼는 ValueError"가 지금 코드와 반대다.**
  - 설계서: 249줄 "병렬 그리퍼는 ValueError."
  - 코드: `src/so101_studio/arm/sim_driver.py:57-70` `make_sim_driver`에 그 거부가 없다. #348이 두 줄(`if cfg.gripper == "parallel": raise ValueError("MuJoCo는 단동식만 지원")`)을 지웠다. `cfg.gripper == "parallel"`이면 `mujoco_driver.py:34·63-64`가 `load_model(hand="parallel")`로 로보나인 손을 붙인다.
  - 같은 설계서 80줄(13.2 병렬 그리퍼)은 이미 새 동작으로 적혀 있어 13.9와 서로 어긋난다.
  - 근거 PR: #348

- **[2] 13.9 뷰어 동기 방식·주기가 다르다.**
  - 설계서: 248줄 "`launch_passive(model, data, ...)` … 1/30초마다 sync하고 `xfrc_applied`는 0으로 둔다." (같은 내용이 13.1의 52줄 "물리 스레드가 잠금을 쥔 채 1/30초마다 `handle.sync()`"에도 있다.)
  - 코드: `mujoco_driver.py:24` `VIEWER_SYNC_S = 1 / 15`. `:92-93` 뷰어는 물리 `data`가 아니라 복사본 `self._vdata`를 본다. `:193-197` 물리 스레드는 `mj_copyData`만 하고, `:200-207` 별도 스레드 `mujoco-viewer-sync`가 잠금 밖에서 `sync()`를 부른다. `xfrc_applied`를 0으로 만드는 코드는 없다(마우스 힘은 복사본에만 들어간다).
  - 측정: `measurements.md:479-483`(30Hz는 60초에 버림 45~64번, 15Hz 채택).
  - 근거 PR: #354

- **[3] 13.10 M5의 목적·판정 일부가 로보나인 손에는 없는 규칙이다.**
  - 설계서: 271줄 M5 "공통 층 + 실행기 A(로보나인 손 위) … U6(`grasp_tilt_tol_deg`) 같은 결정의 근거를 낸다", 260줄 판정 목록의 `height_required`, 263줄 "알고리즘 비교(집기 기울기 …)".
  - 코드: `grasp_tilt_tol_deg`는 `SingleJawGripper`에서만 쓴다(`arm/gripper.py:157`). `ParallelGripper`(`gripper.py:230-`)는 기울기 0이다(`config/grippers/parallel.yaml` `grasp_tilt_deg: 0`). `height_required` 거부는 `skills.py:305`에서 `self.gripper.name == "single_jaw"`일 때만 난다.
  - 결정 0063 1항도 "단동식 전용 규칙(기울여 집기, `grasp_tilt_tol_deg`)은 로보나인에는 해당 없다"고 적는다. M5 행은 0063 전(0062 때) 문장이 남은 것이다.
  - 그대로 M5 세부 설계를 쓰면 로보나인 손에서 나오지 않는 실패 이유와 쓰지 않는 파라미터를 재게 된다.
  - 근거 PR: #348(로보나인 손이 M5의 기준이 됨)

- **[4] "로보나인 손 위"·"브래킷 질량을 같이 싣고"는 기본 설정에서 켜지지 않는다. M5·M6가 무엇을 켜고 도는지 설계서에 없다.**
  - 설계서: 270줄 M4b ⑥ "M7a의 브래킷 질량·`wrist_sonar` site를 같이 싣고", 271줄 M5 "(로보나인 손 위)", 272줄 M6 "손목 센서(`wrist_sonar`)", 13.7 공통(180줄) "cfg는 `robot.yaml` + 팔 파일(local.yaml 없이)".
  - 코드:
    - 그리퍼 기본값은 `single_jaw`다(`core/config.py:71·184`, 팔 파일·`robot.yaml`에 `gripper:` 줄 없음). 병렬 손은 `config/local.yaml`의 `gripper: parallel`로만 켜진다(`tests/test_mujoco_parallel.py:26-32`는 임시 local.yaml을 만든다).
    - `wrist_sonar` site는 병렬일 때만 생긴다(`mujoco_model.py:120-121`).
    - 브래킷·센서 body와 질량(15.5g)은 `config/scene.yaml`의 두 부품이 있을 때만 들어가는데 지금 주석이다(`scene.yaml:15-16`, `mujoco_model.py:196-198`).
  - 그래서 13.7 공통의 cfg로 M5·M6를 돌리면 단동식 손, 브래킷 없음, `wrist_sonar` 없음이다.
  - 근거 PR: #348

### 고치면 좋은 것

- **[5] M4b 완료 표시의 "carry 30/45/58 통과"는 13.7 (b) 기준 전체가 아니다.**
  - 설계서: 270줄.
  - 코드: `tests/test_mujoco_parallel.py:84-107`은 `held == "box"`, 놓은 자리 7mm, 기울기 10°만 본다. 13.7 (b)의 미끄러짐 3.5mm와 `ARM-GRIP-OVERLOAD*`·fault 0건은 보지 않는다(`tests/test_mujoco_scenarios.py:107-112`는 단동식 cfg로만 돈다).
  - 행에 "남은 것: 다시 재기"가 있어 모순은 아니지만, 무엇이 통과한 것인지 적어 두는 편이 낫다. 근거 PR: #348

- **[6] 손 메쉬가 "원본"이 아니라 카메라 팔을 자른 파생 STL이다.**
  - 설계서: 80줄 "시각은 원본 메시", 270줄 완료 PR 목록 "#348·#350·#357"(#354 빠짐).
  - 코드: `mujoco_hand.py:14·65` `NOCAM_STL = assets/mujoco/parallel/parallel_hand_nocam.stl`(y < −36mm 평면 절단, `:49-58`). 충돌 상자도 이 메쉬에서 나온다(`:75-77`).
  - #357 뒤에는 URDF도 이 파일을 본다(`assets/so101_parallel.urdf:266·271`). 팔 쪽 기구학·충돌 검사가 MuJoCo 세션 영역(`assets/mujoco/`)의 파일에 기대게 됐는데 설계서에 없다.
  - 13.1 파일 표(18~40줄)에 `arm/mujoco_hand.py`, `assets/mujoco/parallel/`, `tests/test_mujoco_parallel.py`가 없다.
  - 근거 PR: #350, #354, #357

- **[7] M4b ④의 "클램프 안쪽 얇은 패드 geom"은 만들지 않았다.**
  - 설계서: 270줄 ④.
  - 코드: `mujoco_hand.py:108-125`는 z 조각 경계 상자만 붙인다(패드 없음). ②의 "equality 또는 tendon"은 equality로 정해졌다(`:126`).
  - 완료(✅) 행에 계획 문장이 그대로 남아 있다. 근거 PR: #348

- **[8] 접촉 역할·`held` 규칙이 13.4·13.6에는 단동식 이름만 있다.**
  - 설계서: 120·122줄(역할 `jaw_fixed`·`jaw_moving`), 173줄(`held` = `jaw_fixed`와 `jaw_moving` 둘 다). 80줄에만 `clamp_right`·`clamp_left`가 있다.
  - 코드: `mujoco_driver.py:23` `JAW_ROLES` 네 개, `mujoco_model.py:247-248` 병렬이면 클램프 body만 턱이고 손 본체 상자는 `arm`이다.
  - M5 판정(미끄러짐·`held`)을 13.6만 보고 쓰면 병렬 손에서 `held`가 늘 None인 것처럼 읽힌다. 근거 PR: #348

- **[9] `load_model` 인자와 그리퍼 범위 설명이 단동식 기준이다.**
  - 설계서: 22줄 `load_model(scene=None, objects=(), *, base, table_z_mm, joint_limits=None)`, 241줄 "그리퍼는 각도로 [v0−1°, v100+1°]".
  - 코드: `mujoco_model.py:91` 인자에 `parallel_hand`, `hand`, `gripper_width_mm`가 더 있다. 병렬이면 그리퍼는 slide(m)이고 범위는 `[-0.5mm, 42.5mm]` 고정이다(`mujoco_hand.py:23·123·133`, `STROKE_M = 0.042`).
  - 실물에서 폭 표(`gripper_width_mm`)를 다시 재 84mm를 넘으면 목표가 잘린다(표와 상수가 따로다).
  - 근거 PR: #348

- **[10] 13.4의 `camera_mount` 역할 문장이 지금 모델과 다르다.**
  - 설계서: 122줄 "나머지(`base`, `camera_mount` 포함)는 `arm`".
  - 코드: `mujoco_model.py:111`이 `camera_mount` body를 모든 손에서 지운다(커밋 378d95e, #348~#357 밖).
  - 13.10 M4b가 전제로 삼는 13.4라서 적는다.

- **[11] M5 "한 시도 = 한 에피소드"는 `ArmService`를 다시 쓰면 앞 시도의 기억이 남는다.**
  - 설계서: 262~263줄(`reset(seed, options)`, 한 시도 = 한 에피소드).
  - 코드: `service/arm_service.py:143-148`은 다시 connect할 때 `Skills`의 `grip`·`placed`·`known_objects`·`known_height`(#352)·`placed_size`(#356)·`placed_height`·`placed_angle`을 새 `Skills`로 넘긴다. MuJoCo 월드는 connect마다 새로 지어지므로(13.5), 같은 `ArmService`로 reset하면 없는 물체를 피하고 운반 높이(`skills.py:368` `_carry_lift`)와 놓을 칸(`skills.py:586` `_fits`)이 달라진다.
  - 13.7처럼 에피소드마다 새 `ArmService`를 만든다고 적어야 한다. 근거 PR: #352, #356

- **[12] 판정 목록에 #352·#356이 넣은 결과가 없다.**
  - 설계서: 260줄 판정 목록.
  - 코드: `skills.py:355` `ARM-CARRY-LOW`(WARN, 높이 못 올림 #352), `skills.py:573` `drop_zone_full`(이제 "칸이 다 참"이 아니라 "쥔 물체가 겹치지 않고 들어갈 자리가 없음", #356). 물체 여러 개를 배출 구역에 놓는 M5 묶음에서 나오는 실패 이유다.

- **[13] M6 "센서 자리는 CAD 부품의 site에서 읽는다"는 손목 센서 하나만 해당한다.**
  - 설계서: 259줄, 272줄.
  - 코드:
    - MuJoCo 모델의 센서 site는 `wrist_sonar` 하나다(`mujoco_model.py:121·235`, x 0.0). 초음파 설정은 `arm_sensor_offset_mm` x 0.6이다(`config/ultrasonic.yaml:39`). 어느 쪽이 기준인지 정해야 한다.
    - 레이더는 site가 없고 `radar.station_mm`(`ultrasonic.yaml:8`)과 장면 상자 `radar_station`(`scene.yaml:5`)뿐이다.
    - 지금 가상 ESP32(`perception/ultrasonic/fake_esp32.py:21-45`)는 자체 `SimClock`과 상자 목록으로 거리를 계산한다. `MuJoCoDriver`는 벽시계 스레드이고(`mujoco_driver.py:180-198`) 광선을 쏘는 공개 함수가 없다(`sim_state()`에 없음, `src`에 `mj_multiRay`·`mj_ray` 0건).
  - 구현 전 계획이라 drift는 아니고, M6 세부 설계가 시계와 잠금(`_lock`)을 어떻게 맞출지 정해야 한다는 뜻이다.

### 확인 못 함

- 270줄 "FK 0.002mm": 시험 기준은 0.1mm(`tests/test_mujoco_parallel.py:61`)이고 측정값 기록은 `measurements.md`에서 찾지 못했다.
- 80줄 "충돌 상자 손 3개, 클램프 8개씩": 손 3개는 `HAND_SLABS = 3`과 맞지만 클램프 개수는 STL 높이 ÷ 10mm라 코드를 돌려야 안다.
- 281줄 "집어 옮기기 한 번이 약 20초": MuJoCo 기준 기록을 찾지 못했다(`measurements.md:495`의 20초는 실물).
- 269줄 M7a 숫자 가운데 "손목 끝 자세 36개 간섭 없음": 보지 않았다(15.5g은 `tests/test_mujoco_parallel.py:120`과 맞음).
- 248줄 "Windows의 메인 아닌 스레드에서 열린다", 244줄 스텝 평균 0.067ms: 실행하지 않아 다시 재지 않았다(0.067ms는 `measurements.md:461`과 맞음).
- 병렬 손의 부하 %·`Torque_Limit` 배율(분모 10N, `mujoco_driver.py:69·260·271`)이 `gripper_load_warn_pct` 60 아래로 도는지: 설계서도 "다시 잰다"로 남겨 둔 것이고 측정 기록이 없다.
