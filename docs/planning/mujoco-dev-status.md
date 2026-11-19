# MuJoCo 개발 현황과 인수인계 (MuJoCo 세션 → 기획·다음 세션)

상태: 초안

갱신: 2026-11-19 (main 2194064 기준). 설계는 `mujoco-spec.md`(13.10.1~13.10.4)와 결정 0053·0062·0066~0071, 숫자는 `measurements.md`에 있다. 이 문서는 지금 상태·실행 방법·함정·남은 일만 남긴다.

## 인수인계
읽는 순서: `CLAUDE.md` → `docs/guides/session-workflow.md` → `work-queue.md` → 이 문서 → `mujoco-spec.md` 13.10. 영역: `arm/mujoco_*`, `assets/mujoco/`, `config/sim_scenarios/`, `tests/test_mujoco_*`(CLAUDE.md "세션 구분"). 폴더는 `so101-studio-sim` 하나이고 평소에는 origin/main을 detach로 따라간다. 데이터셋·학습·결과는 `runs/`(git 밖)에 있다.

**지금 상태**: 받은 REQ(-072, 077, 080, 083, 084, 087, 078 후속)가 모두 끝나 합쳐졌다. 다음 일은 사용자의 ACT 시연 방식 답(GUI에 학습 배치 / 명령줄 6칸 창)을 기다린다.

## 시험장 (`python -m so101_studio.arm.mujoco_bench`)
- 묶음(`config/sim_scenarios/bench/`):
  - 고정 묶음: `pick_grid`(223, 오른쪽 열 −20°, 통과 겹치는 두 칸 skip), `inject_grid`(45), `pick_smoke`(12), `carry`(45·58, 시계 기준)
  - 통 옆: `bin_side`(28)
  - 수집: `collect_pick`(옛), `collect_pick1`(1차), `collect_pick2`(2차, `start_pose`)
- 주요 옵션:
  - `--clock wall|step`: step은 StepClock. 실시간보다 약 10배 빠르고, 같은 씨앗이면 같은 결과, lag 없음
  - `--jobs N`
  - `--viewer`(MuJoCo 창, wall만) / `--viewer grid`(창 하나에 일꾼 칸, step이면 첫 일꾼만 10Hz)
  - `--record <이름>`(LeRobot 데이터셋, jobs면 part-NN을 합침)
  - `--policy <체크포인트>`(PolicyRunner 평가)
  - `--approach vertical|auto`
  - `--gripper`, `--grip-force`
- 묶음 키:
  - `robot`(수집 프리셋 `collect_fast.yaml`), `settle_s`, `start_pose`
  - `sample.angle_deg: along`
  - `skip`(격자 칸 빼기, 순번 유지)
  - 뽑는 묶음은 통에 걸리는 자리를 다시 뽑는다
- 데이터셋 도구 `python -m so101_studio.arm.mujoco_record`:
  - `--shift <이름> --k 5`(action 앞당김 사본)
  - `--clip <이름> [--margin 1]`(관절 한계 안쪽으로 자른 사본)
  - 사본 순서는 `meta/collect.json`의 `derived`에, 시작 자세는 `start_pose`에 적힌다.

## 지금 숫자 (자세한 것은 measurements.md)
- M5 새 장면(`--clock step`): `pick_grid` 149/223, `inject_grid` 30~32/45, `bin_side` 8/28. 모두 거짓 성공 0이다. 반경 240mm는 도달 밖(reach_mm 230)이다.
- 시계 주입: carry58 2.99초 대 실시간 32.36초다. jobs 1과 8의 결과가 같다. 같은 씨앗 미끄러짐 14번이 범위 0이다.
- ACT 2차: `pick2_v1`(314개, 시작 자세 [0,−30,30,75,0,50])을 clip해 k0를 60k 학습했다. 평가 **25/50**(`runs/train/act_pick2_k0clip/checkpoints/060000/pretrained_model`). 시뮬레이션 부하 판정이 예민한 상태의 값이다.
- 접근 각도: auto가 vertical보다 나빠(placed_ok 97 대 149, 거짓 성공 21) 기본은 vertical이다(결정 0069).

## 함정
- 녹화 중에는 GC를 끈다(`EpisodeRecorder`). 켜 두면 GC가 GIL을 0.1초 넘게 잡아 realtime_lag가 난다.
- `--record`를 `--jobs 12`로 돌리면 메모리가 모자란다(AV1 인코더). 녹화는 일꾼 6개 안팎으로 한다.
- 실시간 정책 평가는 다른 세션의 시험·벤치마크와 같이 돌면 추론이 느려져 too_slow가 는다. 숫자를 낼 때는 `--clock step`을 쓴다.
- 시뮬레이션 액추에이터는 kp 998·forcerange 2.94 N·m라 오차 0.17°에 부하 100%가 된다. 정책 실패의 fault(과부하·과전류)는 이 예민함 때문이다. M5c에서 실물로 맞출 때까지 그대로 둔다.
- StepClock: 에피소드마다 에피소드 스레드에서 새로 만든다. 참여 스레드가 시계 밖에서 기다리면(time.sleep 등) 30초 뒤 `SIM-CLOCK-HIDDEN-WAIT`가 남는다. 틱 스레드가 죽으면 바로 오류가 나고, 본문이 길면 60초까지 기다린다.
- 받침 충돌 상자는 `base_mount_h0.stl` 경계에서 계산한다(메쉬가 바뀌면 따라간다). 아래 핀(z −15mm)은 뺀다.

## 남은 일
1. ACT 시연 방식(사용자 답 대기). GUI 정책 실행은 체크포인트의 `train_config.json` → `dataset.root` → `meta/collect.json`의 `start_pose`·`preset_values`를 쓴다.
2. 600개·이어 학습(지금은 하지 않음, 총괄 판단).
3. M5c 서보 맞추기(실물 팔 뒤): STS3215 부하%·오차 관계를 재서 시뮬레이션 부하 판정을 맞춘다.
4. M6 초음파를 MuJoCo에(13.10.3). REQ-089(plan_goto 호버)가 영향을 준다.
