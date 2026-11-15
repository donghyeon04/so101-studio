# EGM Stage 9 (나) 구현 준비도 보고서: 학습 플랫폼(결정 0066)

**결론: 지금 문서로는 세 세션이 묻지 않고 구현할 수 없다. 꼭 고칠 것은 10개다.** 가장 큰 구멍은 세 절이 맞물리는 곳에 있다. 세계를 다시 짓는 규약, 배치와 판정 함수, 기록기의 명령값 출처와 데이터셋 수명, `PolicyRunner`의 수명이 비어 있다. 이름 하나(`gripper_link`)도 지금 코드와 다르다. 12/4 전 전체 범위는 들어오지 않는다. 13.10.1(`mujoco_bench`)이 아직 없는데 세 절이 모두 그 위에 서 있어서다. 자를 후보는 아래 "고치면 좋은 것" 끝에 적었다.

- 대상: `docs/planning/mujoco-spec.md` 418~445(13.10.4), `docs/planning/gui-spec.md` 470~493(12.10), `docs/planning/arm-software-spec.md` 318~328(6.10). 기준 사본은 `C:\Users\c\Desktop\so101-gui-docs-tmp`(main c5db419)다.
- MuJoCo 세션이 보낸 모호한 곳 6개는 아래와 같이 처리했다.
  - ①④⑤⑥은 꼭 고칠 것 1·2·3·4에 합쳤다.
  - ②(13.10.1 기준 4와 REQ-069), ③(실패 주입 G의 이벤트 코드)은 13.10.1의 일이라 이번 범위 밖이다. 13.10.1에서 따로 정한다.

## 리뷰어 상태
| 리뷰어 | 상태 |
|---|---|
| `egm-implementation-readiness` | completed (첫 실행은 사용 한도로 끊겨 한 번 다시 돌림) |
| `egm-codebase-details` | completed (첫 실행은 사용 한도로 끊겨 한 번 다시 돌림) |
| `egm-style` | completed |

리뷰어끼리 엇갈린 점이 하나 있었다. LeRobot `so101_follower`의 관절 단위다. 설치본 `lerobot/robots/so_follower/config_so_follower.py:42`가 `use_degrees: bool = True`임을 직접 확인했다. 따라서 "키·단위가 `so101_follower`와 같다"(mujoco 435)는 맞다.

---

## 꼭 고칠 것

1. **손목 카메라를 붙일 body 이름이 틀렸고, 지금 코드가 손목 카메라를 지운다** (MuJoCo ① 합침): `mujoco-spec.md` 423~424
   - 지금 코드
     - MuJoCo body 이름은 두 손 모두 `gripper`다. `gripper_link`는 URDF 이름이다(`arm/mujoco_model.py:39`). 문서대로 `model.body("gripper_link")`를 부르면 KeyError가 난다.
     - `load_model`은 `camera_mount`와 `wrist_cam`을 일부러 지운다(`mujoco_model.py:111`, 주석 "결정 0035, 카메라는 탑뷰 하나").
     - 단동식 손의 값을 고를 근거가 문서에 없다.
   - 제안(423행 표 칸): "`wrist` | body `gripper`(두 손 같은 이름, URDF의 gripper_link) | 고정 자세 | `camera_mount` 지우기(mujoco_model.py:111)는 그대로 두고(질량·형상 제외) 카메라만 `add_camera(name="wrist")`로 더한다. 단동식은 원본 `wrist_cam` 값(pos (−3, 54, −53)mm, quat 원본, fovy 61, `so101.xml` 204~205)을 그대로 쓰고, 병렬 손 값은 첫 PR에서 이 칸에 숫자로 적는다. :111의 주석은 결정 0066에 맞게 고친다."

2. **탑 카메라 '판 가운데'의 좌표계가 없다** (MuJoCo ⑥): `mujoco-spec.md` 422
   - 제안: "자리: 작업대 좌표(`workspace.yaml`의 `table_mm` 직사각형)의 중심 위, 판 면에서 550mm. 축은 월드 −z를 본다. 영상 위쪽이 작업대 +y(팔에서 먼 쪽)가 되게 돌린다. H-3 전에는 13.10.3의 임시 판 기준이다."

3. **기록기 `action`이 읽을 명령 관절값의 공개 통로가 없다** (MuJoCo ④): `mujoco-spec.md` 434, `arm-software-spec.md` 326
   - 지금 코드
     - `ArmService.status()`의 `ArmStatus`에는 명령값이 없다(`service/arm_service.py:221-228`).
     - 명령값은 `status.jsonl`에 적을 때 비공개 `ctl._last`에서만 읽는다(`:238`).
     - 6.10의 "팔 쪽에 기록용 훅은 더하지 않는다"와 13.10.4의 "상태의 명령값을 읽는다"가 맞물리지 않는다.
   - 제안(arm 326 끝): "팔 세션이 `ArmStatus`에 `cmd: JointState`(지금 명령 관절값, 동작이 없으면 마지막 명령)를 더하고 `status()`가 채운다(공유 파일 `core/types.py`, `service/arm_service.py`). 기록기는 `svc.status().cmd`만 읽는다."
   - 제안(mujoco 434): "`observation.state`는 `driver.read_joints()`(25Hz), `action`은 `svc.status().cmd`."

4. **렌더러의 수명과 스레드 규칙이 다시 짓기와 맞지 않는다** (MuJoCo ⑤): `mujoco-spec.md` 427~428
   - 지금 코드
     - `connect()`는 부를 때마다 `model`·`data`를 새로 만든다(`mujoco_driver.py:61-65`). 그래서 "처음 부를 때 만든" 렌더러는 다시 짓기(gui 482·490) 뒤에 옛 모델을 가리킨다.
     - 뷰어는 물리 스레드 안에서 `_vdata`로 복사한다(`:196`). `render`는 부른 스레드에서 복사하므로 패턴이 다르다.
   - 제안: "`render`는 잠금 안에서 `mj_copyData(self._rdata, model, data)`만 하고, 그리기는 잠금 밖에서 한다. `_rdata`와 렌더러는 `connect`·`disconnect`에서 버리고 다음 `render`에서 새 모델로 만든다. 렌더러를 만든 스레드에서만 쓸 수 있으면(Windows에서 첫 PR에 확인) 드라이버 안 렌더 전용 스레드 하나가 요청 큐를 받는다. 이때도 `render`의 시그니처와 반환은 같다."

5. **'세계 다시 짓기' 규약이 없다**: `gui-spec.md` 482, 490 (`mujoco-spec.md` 13.10.4에 추가)
   - 지금 코드
     - 물체를 바꾸는 공개 수단이 없다. 물체는 생성자에서 정해진다(`mujoco_driver.py:32`).
     - 백엔드는 `make_sim_driver`를 시작할 때 한 번만 부르고(`gui/backend.py:177`), `reconnect()`는 실제 팔 전용이다(`:271-276`).
     - `ArmService.connect`는 다시 연결해도 `grip`·`placed`·`known_objects`를 새 `Skills`로 넘긴다(`arm_service.py:144-149`). 그래서 이전 에피소드의 기억이 남아 13.10.1 기준 3(에피소드 독립)과 부딪친다.
     - 다시 짓는 동안 기록기·정책 스레드가 `render`/`read_joints`를 부르면 `RuntimeError`가 난다.
   - 제안(mujoco 13.10.4에 한 단락): "세계 다시 짓기 = `Backend.rebuild_world(seed)` 하나다. 순서: (1) `PolicyRunner.stop()`과 `EpisodeRecorder.stop(save=…)` (2) `MuJoCoDriver.set_objects(layout.objects)`(13.5 겹침 검사 그대로) (3) `backend.sim.connect(fresh=True)`: `skills`의 `placed`·`known_objects`·`grip`을 비운다 (4) 수동 물체 목록과 3D(`scene.set_objects`)를 바꾼다. 팔은 (3)에서 홈으로 놓인다. 드라이버는 MuJoCo 세션, `fresh` 인자는 팔 세션, `rebuild_world`는 GUI 세션이 만든다."
   - 제안(gui 490): "[다음 배치]는 `rebuild_world(다음 씨앗)`을 부른다(팔은 다시 지을 때 홈에 놓이므로 따로 홈 동작을 하지 않는다)."

6. **'배치'와 판정 함수의 공개 규약이 없다**: `mujoco-spec.md` 439~441, `gui-spec.md` 481·489
   - 지금 코드: `arm/mujoco_bench.py`는 아직 없다(13.10.1 미구현). GUI 수집·정책 화면과 L3 평가가 부를 `collect_pick` 배치 생성과 `placed_ok` 판정의 이름·인자·모듈이 없다. "(4개 배치)"의 뜻도 모호하다.
   - 제안(13.10.4 첫머리): "배치 = 묶음과 씨앗 하나가 정하는 물체 크기·자리·각도, 놓을 곳, 과제 문장이다. 배치 하나에 에피소드 하나. `arm/mujoco_bench.py`는 `layout(suite: str, seed: int) -> Layout(objects, place_xy_mm, task)`와 `judge(driver, layout) -> dict(outcome, …)`를 공개한다. `judge`는 `driver.sim_state()`와 `layout`만 읽고 `pick_and_place`의 보고 없이도 돈다. GUI와 `PolicyRunner`는 이 둘만 import한다. 씨앗 범위는 전문가 수집 0~9999, 평가 10000~10049, GUI 수집 20000부터(이어 쓰면 기존 에피소드 수만큼 건너뜀)."
   - 제안(441행): "저장한 에피소드 4개를 골라 같은 씨앗의 배치에서 `action`을 `servo_joints`로 다시 틀면 `placed_ok`다(`StepClock`이 없으면 4개 중 3개 이상)."

7. **`servo_joints`의 안전 검사가 '조그와 같다'는데 지금 조그 코드와 다르다**: `arm-software-spec.md` 320~323
   - 지금 코드
     - 조그가 쓰는 `Safety.check_goal`(`arm/safety.py:71-94`)에는 "알려진 물체 충돌"이 없다. 넣으면 집으러 다가가는 목표가 모두 거부된다.
     - 조그는 관절 한계를 자른다(`motion.py:562-566`).
     - 막힘 감지 `check_blocked`는 궤적 동작(`_job`)에서만 돈다(`motion.py:427-439`). `_closes_gripper`도 `execute()`에서만 켜진다(`:142`). 그래서 조그 경로를 그대로 쓰면 323행의 "그리퍼 접촉 판단이 그대로 걸린다"가 사실이 아니게 된다.
     - 속도 한계 키(`joint_v_max_deg_s` 90인지 `jog_joint_max` 40인지), 서보 중 `ctl.state` 값, 궤적 동작 중 호출 처리도 없다.
   - 제안(321행): "검사는 `safety.check_goal(goal, ref=_last)`(관절 한계·작업 영역·장면 부품. 알려진 물체는 보지 않는다)다. 걸리면 자르지 않고 목표를 버리고 제자리에 선 뒤 WARN `ARM-SERVO-REJECT{reason, count}`를 같은 이유당 1초에 한 번 남긴다. 한 틱 보간 뒤 `check_step`은 조그와 같이 fault다."
   - 제안(320행): "한 틱 이동 상한 = min(`joint_v_max_deg_s` × 속도%/100 × dt, `max_step_deg`). 서보 중 상태는 `"teleop"`, 궤적 동작 중이면 `False`(이유 `busy`). `stop()`은 서보 목표도 비운다."
   - 제안(323행): "서보에서 그리퍼 목표가 직전 명령보다 닫히는 쪽이면 `_closes_gripper=True`로 두고, 6.5와 같은 계산(`blocked_grip_units`, `BLOCKED_N`, `_grip_contact_target`)으로 '지금 위치 + 예압'에서 멈춘다. 팔 다섯 관절의 막힘 판단은 서보에서 하지 않는다(조그와 같음)."

8. **데이터셋 수명(만들기·이어 쓰기·마무리)과 수집 화면 버튼의 연결이 없다**: `mujoco-spec.md` 433~437, `gui-spec.md` 480~483
   - 생성자에 새로 만들지 이어 열지가 없다. `repo_id`, 영상 저장 방식, 닫기 호출도 없다.
   - GUI에는 "에피소드를 시작하면"이 무엇인지(버튼인가 첫 입력인가)와 [정지]의 뜻이 없다.
   - 제안(mujoco 433 뒤): "`EpisodeRecorder`는 `dataset_dir`가 없으면 `LeRobotDataset.create(repo_id="local/<이름>", root=dataset_dir, fps, robot_type="so101_follower", features=FEATURES, use_videos=True)`, 있으면 열어서 `features`·`fps`가 같은지 보고 다르면 `ValueError`를 낸다. `stop(save=True)`는 `save_episode()`, `stop(save=False)`는 `clear_episode_buffer()`, `close()`는 마무리 호출이다(0.6.1 설치본의 실제 이름을 PR에 적는다). features 이름은 `shoulder_pan.pos … gripper.pos`(`core/types.py` 순서)다. 기록 구간은 `start()`부터 마지막 동작 뒤 판정 대기 1.0초까지다."
   - 제안(gui 480·482): "버튼은 [기록 시작]·[저장하고 다음]·[다시]·[정지]다. [기록 시작]=`start()`, [저장하고 다음]=`stop(True)` 뒤 `rebuild_world(다음 씨앗)`, [다시]=`stop(False)` 뒤 같은 씨앗으로 `rebuild_world`, [정지]=지금 에피소드를 버리고 `close()`. 다시 짓기가 끝나면 기록기는 꺼진 채 기다린다."

9. **`PolicyRunner`의 정책 불러오기·시험 주입·일시 정지·끝·적용 범위가 비어 있다**: `arm-software-spec.md` 319·324, `gui-spec.md` 488~491, `mujoco-spec.md` 443
   - GUI 시험(493 ③)의 "가짜 정책"을 넣을 인자가 없다.
   - 정책을 불러오는 법이 없다. 함수, CPU로 여는 법, 전후 처리기가 다 비어 있다. 체크포인트 폴더 구조(`<output_dir>/checkpoints/<step>/pretrained_model`)와 학습 출력 폴더도 정해지지 않았다.
   - `resume()`이 없고, 정책 에피소드가 언제 끝나 판정하는지(gui 489 "끝난 뒤")도 없다.
   - `simulated`는 기구학 `SimDriver`도 True다(`sim_driver.py:79`). 그래서 기구학 엔진에서 버튼이 보이고 `render`가 없어 실패한다.
   - 어느 `ArmService`(`backend.sim`)에 붙는지도 없다.
   - 제안(arm 324): "`PolicyRunner(svc, driver, checkpoint_dir=None, fps=25, policy=None, max_s=20)`. `policy`를 주면 그대로 쓰고(시험용, `select_action(obs) -> 6값`), 없으면 `ACTPolicy.from_pretrained(checkpoint_dir)`를 CPU로 열고 같은 폴더의 전후 처리기를 쓴다. 묶음은 `select_action`의 큐에 맡긴다. 드라이버에 `render`가 없으면 `start()`가 안내문으로 끝난다. `pause()`는 추론을 멈추고 서보 목표를 지금 명령값으로 둔다(`ARM-POLICY-PAUSE`). `resume()`은 `policy.reset()` 뒤 새 관측으로 다시 시작한다. `max_s`나 [정지]로 끝나면 1.0초 뒤 `judge`를 불러 결과를 `ARM-POLICY-STOP{reason, outcome}`에 싣는다. `servo_joints`가 `False`를 내면(정지·E-STOP·fault) 러너도 끝난다."
   - 제안(gui 488·491): "정책 실행과 수집은 늘 `backend.sim`에 붙는다. 체크포인트 목록은 `runs/train/*/checkpoints/*/pretrained_model`이다. 없을 때: 드라이버에 `render`가 없으면(실제 팔, 기구학 엔진) '학습한 정책 실행은 MuJoCo 시뮬레이션에서만 합니다'를 보이고 버튼을 숨긴다."
   - 제안(mujoco 443): "학습 명령에 `--output_dir runs/train/<이름>`을 준다. `so101-train`의 lerobot도 0.6.1로 고정한다(체크포인트 형식). 평가 50개는 `mujoco_bench --suite collect_pick --policy <폴더> --seed-start 10000 --seeds 50`으로 돌린다(`PolicyRunner`를 GUI 없이 쓴다)."

10. **기록기 시계가 아직 없는 13.10.2에 기댄다**: `mujoco-spec.md` 436
    - 지금 코드: `core/clock.py`가 없고 `ArmService`는 `time.perf_counter`를 바로 쓴다(`arm_service.py:252,279`). 이대로면 기록기 첫 PR이 13.10.2를 기다리는지 정할 수 없다.
    - 제안: "13.10.2가 들어오기 전에는 기록기가 `time.perf_counter`로 25Hz를 맞춘다(실시간 수집). `ArmService`에 시계가 생기면 그것으로 바꾼다. 기록기 첫 PR은 13.10.2를 기다리지 않는다."

---

## 고치면 좋은 것
- **재생 시험의 흔들림**(arm 327): 팔 시험이 기록기보다 먼저 생긴다. 팔 시험은 `SimDriver`에서 코드로 만든 25Hz 관절 목표 열로 끝 자세 1° 안을 보게 하고, 데이터셋 재생은 MuJoCo 시험으로 옮긴다.
- **ACT 묶음 크기와 추론 지연**(arm 324~325)
  - 기본 `n_action_steps` 100이면 4초 동안 관측을 보지 않는다. L3에서 값(예: 25)을 정해 `measurements.md`에 적는다.
  - CPU 추론이 40ms를 넘으면 프레임을 건너뛰고 그 수를 이벤트에 적는다.
- **렌더 부하 측정 조건**(mujoco 430): 10fps 화면만 재게 되어 있다. "화면 10fps와 기록·정책 25fps를 함께 돌릴 때"도 `dropped_count`를 남긴다.
- **오프스크린 버퍼**(mujoco 426): `so101.xml`에 offwidth/offheight가 없어 기본 640×480이다. 해상도를 키울 때의 한계를 한 줄 적는다.
- **GUI에서 다시 쓸 것**(gui 471~481)
  - 메뉴는 `gui/app.py:49-54` `PAGES`, 등록은 `:75` `BUILDERS`다.
  - 초음파 칸은 `gui/sensors/ultrasonic_view.py`의 `mini_view`를 쓴다.
  - 배치 평면은 `gui/sensors/manual_view.py`의 `plane_svg`를 그대로 쓴다고 적는다.
- **시험 판정을 재기 쉽게**(gui 493 ①): "그림이 달라진다"를 "관절을 30° 이상 움직인 전·후 픽셀 차이 평균이 0보다 크고, 같은 자세에서 두 번 그린 그림은 같다"로 바꾼다.
- **이벤트 kind 이름**(arm 321·325): `servo_reject`·`policy_start`·`policy_stop`·`policy_pause`를 적는다(`_emit(kind, level, code, data)` 규칙).
- **데이터셋 이름 규칙**(gui 483): 프리셋과 같은 규칙(글자·숫자·`_`·`-`)을 쓴다.
- **용어 통일**
  - 조건문에는 "MuJoCo 엔진 / 기구학 엔진 / 실제 팔"을 쓴다(mujoco-spec 10행 용어).
  - 화면 제목은 "탑뷰 (`top`)", "손목 (`wrist`)"로 고정한다.
  - 약어 L1~L5, H-1/H-3, 0066-4(=결정 0066의 4항)를 각 절 첫 줄에 한 번 풀어 쓴다.
- **단위 문장**(mujoco 434): "관절 1~5는 °, 그리퍼는 0~100(0 닫힘)"을 따로 쓴다. `so101_follower`와 키·차원·단위는 같지만 영점이 같다는 보장은 없다(실물 이전은 12/4 뒤)고 한 줄 덧붙인다.
- **환경**(arm 324, C-8)
  - 앱 환경에 이미 `lerobot[feetech]==0.6.1`(CPU torch)이 있어 ACT는 불러올 수 있을 것이다.
  - L5의 so101-nexus(`mujoco>=3.0`)가 4.0.0 고정과 맞는지 확인되지 않았다.

### 12/4 전 범위(오늘 11/16, 동결 11/30)
- **안 들어오는 이유**
  - MuJoCo 세션 한 곳에 13.10.1(미구현), 13.10.2, H-3, L1, L2, L3 평가, L5가 몰려 있다.
  - 실물 I1과 같은 주에 겹친다.
- **자를 후보**(먼저 자를 것부터)
  1. L5 강화학습 데모와 게임패드(arm 326): 둘 다 새 의존성을 골라야 하고, 결정 0066의 뒤 순서다.
  2. GUI ② 수집 화면 전체(gui 479~485): 전문가 수집은 명령줄로 된다. 자르면 꼭 고칠 것 5·8의 GUI 쪽 절반이 사라진다.
  3. 데이터셋 이어 쓰기: 같은 이름이면 거부한다.
  4. GUI ③을 줄인다: 체크포인트 고르기, [시작]·[정지], 두 카메라만 둔다. 일시 정지와 다시 짓기는 빼고, 판정은 명령줄 평가로 대신한다.
- **남길 것**: L1(① 시점), 카메라·렌더, 기록기, 전문가 수집, ACT 학습, 명령줄 평가, `servo_joints`, `PolicyRunner`.

---
확인하지 않은 것
- lerobot 0.6.1의 데이터셋 마무리·이어 쓰기·전후 처리기 함수 이름: 이 PC에 `lerobot[dataset]`이 없어 보지 못했다. 그래서 제안 문장에 "설치본에서 확인"을 붙였다.
- Windows에서 `lerobot[dataset]`의 영상 인코딩(torchcodec/av)이 되는지.
