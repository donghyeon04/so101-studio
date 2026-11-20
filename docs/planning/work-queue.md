# 작업 대기열 (총괄이 유지)

상태: 초안 · 2026-11-20 기획(11-20 아침 사용자 결정 한 줄) · 2026-11-20 총괄(11-19 v0.1, REQ-096, 새 계정 설치) · 2026-11-19 총괄(11-18~19 합친 것, 세션별 다음 일, 폴더·브랜치 정리) · 2026-11-17 기획(3a 승인, U16 당기기 상한 85) 반영 · 2026-11-17 기획 #414 반영 · 2026-11-16 기획 #408 반영 · 2026-11-14 밤 (기획이 새 PC 이전 전에 다시 씀, main 299bea9. 첫날 순서는 새 PC 기획 세션이 절차를 따라가 보고 고침. 이후 총괄이 단계마다 갱신)

> 새 PC·새 세션은 `project-status.md`(총정리) → 이 문서 순서로 읽는다. 사용자 결정이 필요한 것은 "사용자 결정 대기"에 쌓고, 그 밖의 일은 기다리지 않고 진행한다.

- **11-20 기획 세션 로컬로**(사용자 지시, 결정 기록 없음): 기획은 이 PC의 로컬 세션 "기획 세션"(폴더 `so101-studio-plan`, 문서만)이다. 총괄↔기획은 `SendMessage`로 주고받고 사용자가 붙여 넣지 않는다. 기획은 일마다 `claude/<주제>` 브랜치(커밋 첫 줄 "[기획 → 총괄] …"), 합치기는 총괄(`session-workflow.md` 1·2·5·7a절).

## 새 PC 첫날 순서 (11-16 기준: 1·2·4 끝, 3만 남음)
1. ~~이전~~ 끝(11-14 밤).
2. ~~합친 main 전체 시험~~ 끝(11-15 fcdcb0f 1218 passed, 실패는 #369·#386으로 고침, 아래 시험 결과). 11-16 하루 마감 전체는 1011f60에서(`runs/fulltest-1114/full-1116.log`).
3. **실물 팔 첫 연결**(사용자 + 팔 세션, 2번이 끝난 뒤): 새 PC에는 LeRobot 캐시가 없다. `config/calibration/so101_follower_01.json`을 `%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\`로 복사 → `so101.cmd doctor` → `so101.cmd status` → `so101.cmd --speed 20 home`(클램프·전원 스위치 옆). 복사하지 않으면 "캘리브레이션 불일치"로 잠긴다. 0° 확인 기록은 git의 팔 파일에 있고 해시가 맞아(#344) `calibrate --verify-zero`를 다시 하지 않아도 움직인다(다시 하는 때: 캘리브레이션 파일이 바뀐 뒤, 로보나인 교체 뒤).
4. ~~EGM drift-report~~ 끝(mujoco-spec #366 반영, 11-16 ultrasonic-spec도 돌림 `inbox/2026-11-16-egm-ultrasonic-drift.md`).
5. 그다음은 아래 "이어지는 순서".

## 이어지는 순서
1. **로보나인으로 다시 맞추기**(결정 0063): 교체(U5) → 그리퍼 범위 캘리브레이션 → runbook 2-10(폭 표·접촉·조임) → 2-2 재시험 30%·50%(결정 0063 그대로, 사용자 확인 11-14. 0061 문턱 다시 재기, `runs/handoff-1114/real_22.py`) → 브래킷 출력(U7 결정: 손목 쪽 평면 +z를 바닥)·장착 → A0.
2. ~~CHECK 경로 기준점~~ 끝(REQ-064 #364, 레이더 가림 점도 #400).
3. **MuJoCo**: 병렬 손 재측정 끝(#367·#370·#375). 다음은 기획의 M5·M6 세부 설계(하드웨어 반영 계획과 한 묶음일 수 있음) → EGM Stage 9 → 구현.
4. **초음파**: CHECK yaw 반경별(결정 0065 #378), 그리퍼별 도달 범위(병렬 75~250 #399), 후보 거르기(#386·#391), 인식 시간 단축(#394) 끝. 다음은 U16 안 A(기획 설계 뒤 REQ) → 부품 뒤 B1~B4 → R1 → A0·A1 → I1.
5. **시험 시간 줄이기**(결정 0064): `slow` 기준 5초·`measure` 표시는 끝(#368·#369·#370). 남은 것(총괄): 시험 포트 0, xdist·loadgroup, testmon 설치·설정.
6. **하드웨어**: H1 레이더 부품, H2 OpenRB VIN, H5 작업대, H7 카메라 주문.
7. 비교 실험(11/23~29, 결정 0074: 시뮬레이션 둘 — 초음파 벤치마크 씨앗 200, MuJoCo 규칙 기반 vs ACT v2. 카메라 뺌, 실물이 오면 실물 열) → 마무리(11/30~12/4, `release-checklist.md`). 11/30부터 새 기능은 멈춘다.
- **A0·A1의 선행 조건**(11-15): REQ-066(경로 첫 구간·전체 길이 ≤ 1.0mm에서 계획 시간이 터지는 버그, `arm/trajectory.py`)이 닫혀 있어야 한다. 실제 `follow_path`(CHECK)가 같은 함수를 쓴다. **#392로 닫혔다(11-15)**: 뿌리 수정 + 안전망(`out_of_bounds`, 이유 `plan_too_slow`). 되돌리는 변경이 생기면 A0 전에 `tests/test_follow_path.py`의 REQ-066 시험을 다시 본다.
- 11-15·16 기획 결정(`ultrasonic-spec.md` 1절): `check.tool_reach_mm`은 그리퍼별(단동식 60~255, 병렬 **75~250**, #399), 인식 시간 기준 **100초**(임시). 병렬 표는 첫 점 당기기 없음/있음 두 열(U16 구현 뒤 한 열로).
- **I1과 시연 배치 조건**(11-15, 11-20 반경 150으로, `ultrasonic-spec.md` 6절): 반경 150~220mm, 물체 중심 사이 120mm 이상, **모서리(가장 가까운 두 변) 사이 80mm 이상**으로 놓는다. 센서 높이 154.5mm에서는 모서리 간격이 좁으면 이웃을 합쳐 본다(따로 보이는 한계: 낙관 50mm, 비관 80mm).
- 실물이 먼저다. 하드웨어를 기다리는 동안은 MuJoCo 시험장이 정식 작업이다(결정 0062).

## 기획 결정 (11-14 밤, 총괄 저녁 보고 7·9절)
- **CHECK 경로 기준점**: (a) 병렬일 때 `tool_position`·`follow_path` 기준을 두 턱 끝 가운데로 한다. 결정 0043-1이 이미 "병렬의 TCP는 두 턱 가운데"이고, #358로 센서 기준점도 가운데가 됐으므로 기준점을 하나로 맞춘다. 단동식은 그대로(고정 턱 끝). (b) "CHECK 전에 닫기"는 다른 `follow_path` 쓰임새의 어긋남을 남겨서 고르지 않는다. 팔 REQ로 낸다(GUI 현재 좌표 표시도 같이 바뀌는지 확인).
- **팀 CAD 원본**: 레포에 넣지 않는다. 팀 드라이브 한 곳을 기준으로 두고 `assets/scene/NOTICE.txt`에 그 위치(드라이브 폴더 이름)를 적는다. 레포에는 3D·충돌용 파생 STL만 둔다(`release-checklist.md` A절).
- **배출 구역 폭**: w100×h80 그대로(총괄 판단, 사용자 위임). 작업대(H5) 조립 때 다시 본다.
- **`real_22.py`**: 로보나인 2-2 재시험에서 다시 쓴다. 계속 쓰면 시험장(결정 0062)의 실물 실행기로 레포에 넣을지 그때 정한다.
- **`backup/tmp-merge-b59d933`**: 새 PC로 가져가지 않는다(main에 다 합쳐졌고 옛 임시 합치기).

## 기획 결정 (11-16, #408 합침)
- **결정 0066 학습 플랫폼(12/4 전)**: 사용자가 학습 플랫폼 질문 다섯을 모두 기획 제안대로 승인했다(11-16). 계획은 `learning-platform.md`, 설계는 `gui-spec.md` 12.10·`mujoco-spec.md` 13.10.4·`arm-software-spec.md` 6.10(초안, Stage 9 전), 설치는 `tooling-setup.md` C-8(사용자가 옆에 있을 때).
- **U16 안 A 반영**: `ultrasonic-spec.md` 6절 "첫 점 당기기"·8.2·8.3 → REQ-067(초음파)·REQ-068(팔).
- **하드웨어 반영 계획**: `hardware.md` H-0~H-6. H-0은 파일이 오면 총괄이 합친 STEP에서 값(판 크기 먼저)을 재서 inbox에 → 기획이 결정 0067. 하드웨어 담당 질문 아홉은 `hardware.md` 맨 아래.
- **M5·시계 주입·M6**: `mujoco-spec.md` 13.10.1~13.10.3(초안). EGM Stage 9 두 묶음 (가) 13.10.1+13.10.2 (나) 13.10.4+gui-spec 12.10+arm-software-spec 6.10 뒤에 구현. 13.10.1의 장면 부품 `gripper` 키는 작고 독립이라 먼저 REQ-069(팔). 13.10.3 M6는 시계 주입 뒤.
- 로보나인 원문 확인 항목은 runbook 2-10(U5 때 팔), 사양(최대 조임 120N, 14mm/s, 170g)은 mujoco-spec 13.10. 잠정 힘 한계 10N은 U15 전까지 그대로. 기획 모델은 Opus(`session-workflow.md`). 11/23 무렵 범위 점검은 [기획 요청]으로 올림(`project-status.md` 8절).

## 기획 결정 (11-17, Opus 기획 세션 `claude/determined-meitner-44rxlf`)
- **3a 범위 자르기 승인**(사용자 11-17): 시계 주입·M6·GUI ② 수집 화면 전체·이어 쓰기·세계 다시 짓기·GUI ③의 [일시 정지]·[다음 배치]·화면 판정·강화학습 데모는 12/4 뒤 2차. 결정 0066 상태 줄, `learning-platform.md` 3a. 11/23 점검은 그대로.
- **U16 당기기 상한 85mm 확정**(REQ-074 #417 + 총괄 재실행 02af7f1): `ultrasonic-spec.md` 1절 병렬 표를 한 열로, 6절·8.3을 `retry_pull_max_mm: 85`로. 설정 파일 기본값 20 → 85는 초음파 REQ(아래 "세션별 다음 일").

## 11-17~18 (총괄, 새 세션)
- 합침: #421 기획(3a 승인, U16 85), #422 REQ-075 대장, #423·#425 REQ-073 ②③(`servo_joints`, `PolicyRunner`, 리뷰 후속), #424 REQ-075(`retry_pull_max_mm` 85), #426 GUI ③ 정책 실행 줄인 판(`/policy`), #427 REQ-070 M5 시험장, #428 REQ-071 카메라·`render`(렌더 전용 스레드). main e0ae4fe.
- 환경: 이 PC `so101`에 pytest-xdist 3.8.0·testmon 2.3.0 설치(사용자 승인, `environment.yml` 고정 #423). `-n 4`에서 `tests/gui/test_pages.py::test_top_bar_run_mode_and_doctor`가 흔들린다(단독 통과, GUI 시험 포트 0 전까지는 GUI 시험을 순서대로).
- 화면: `--sim-engine mujoco gui`의 `/policy`에서 탑뷰·손목 그림을 확인했다(11-18).
- [기획 요청] `planning-inbox` `inbox/2026-11-18-m5-false-ok.md`: inject_grid 거짓 성공 4건 판정(추천 (가) 닫은 값 폭 검사), U15 표, 고정 실패 두 자리.
- MuJoCo 리뷰 후속(렌더 스레드 정리, 이벤트 flush, 예외 에피소드, GRIP_FORCE_N 되돌리기)은 REQ-072와 같이 낸다.

## 기획 결정 (11-18, 기획 `claude/determined-meitner-44rxlf`)
- **M5 거짓 성공**: 쥠 확인(벌린 폭 − 5mm보다 넓게 닫히면 `no_object`), `arm-software-spec.md` 6.6. 팔 REQ → 총괄 재측정.
- **결정 0068 ACT 수집 다시 짜기**: 1차 묶음(상자 면이 팔 방향과 나란, 반경 150~200, 방위 ±30°), 시연 10초 이하(sim 수집 프리셋), action k=0·k=5 사본 비교, 시작 자세 그대로, 씨앗 50개 중 15개 기준, `--jobs` 병렬 수집·평가(`realtime_lag` 에피소드는 버림). `mujoco-spec.md` 13.10.4 → MuJoCo REQ.
- **결정 0069 접근 각도 후보 고르기**(사용자: 수직 고정을 뜯어고치되 수직은 대체로): 병렬 손 `grasp_approach: vertical|auto`, `arm-software-spec.md` 6.6.1 초안. **Stage 9 뒤** 팔 REQ, M5에서 `auto`가 이기면 기본값.
- 총괄(11-18): #438로 합침. REQ-077(0068) → MuJoCo, 0069는 EGM Stage 9(`runs/egm/2026-11-18-stage9-approach/`) 진행 중, REQ-078(팔) 예정.

- **결정 0067 작업대 CAD 값**(H-1): 판 425×375, 배출 통, 시연 방위 45~135°.
- **결정 0070 시계 주입 앞당김**(사용자 요청): `mujoco-spec.md` 13.10.2에 Stage 9 꼭 고칠 것 9개 반영. ① 팔 Clock·WallClock(동작 변화 없음) → ② MuJoCo StepClock → ③ 연결(`--clock step`, `every`, 보기 창). ① 뒤 실물 확인은 사용자에게 먼저 묻는다.

- **결정 0071 ACT 2차 수집**(11-19, 1차 4/50): 수집 전용 시작 자세(MuJoCo 제안), 새 장면 300개 k0 → 모자라면 600·이어 학습, k5 뺌, PolicyRunner 한계 처리 그대로, 11/23 3단계 기준.

## 11-18~19 (총괄)
- 합침: #438·#441·#446 결정 0068·0069(6.6.1), #443·#454 REQ-077(1차 묶음·빠른 프리셋·병렬 수집·`--clip`·`--viewer grid`), #444 REQ-078 기울여 집기, #447 결정 0067(H-1), #448·#455 GUI(기울여 집기 설정, H-5 시험·기준 사진), #450 결정 0070, #452 REQ-083 StepClock, #453 REQ-079 H-2, #456 REQ-085 통 놓기 거짓 성공, #457 REQ-082 시계 ①, #458 REQ-086 PolicyRunner 공개 메서드, #459 결정 0071, #463 pick_grid −20°·bin_side.
- ACT: 1차 묶음(pick1_v1 319개) 최고 4/50(기준 15). 원인은 시작 자세가 관절 한계 근처 → 결정 0071(REQ-087, 시작 자세 `[0,-30,30,75,0,50]` 후보, 손목 카메라 상자 99%).
- 시계 주입: StepClock으로 실시간의 1/10.8, 같은 씨앗 같은 결과, lag 0(REQ-084, #462 검토 중).
- 정리(11-19 사용자 위임): 쓰지 않는 worktree 폴더 셋(`so101-gui-docs-tmp`, `so101-studio-sim-pol` ACT 1차 실험 체크포인트 12GB, `so101-studio-sim-cam` 옛 장면 데이터셋 930MB)과 `so101-이전-1114`(옮길 것은 GUI 폴더 `runs/archive-1114/`로)를 지웠다. 합쳐진 로컬 브랜치 23개와 원격 `dev/*` 브랜치 76개를 지웠다(session-workflow 2절 "원격 브랜치는 따로 지운다"). `claude/*`(기획)와 열린 PR 브랜치는 남겼다.

- **결정 0072 초음파 정확도 먼저**(11-19, 사용자): 레이더 조립 → B1~B4·R1 → 실측으로 규칙 보정(벤치마크·가상 ESP32 센서 모델) → 모자라면 2차 지도학습 추정기(R1·A1·I1에서 정답 쌍 기록, `ultrasonic-spec.md` 8.4). 팔 확인 위치 목표 중앙 5mm·90% 10mm(A1). ACT는 `pick2_v2`로 마무리, 카메라 ACT는 카메라 구매 뒤. M6는 2차 그대로.

## 기획 결정 (11-19, 기획 `claude/confident-einstein-5pzss0`)
- **결정 0073 세계 다시 짓기 1차**(사용자 결정, #487로 구현됨): `mujoco-spec.md` 13.10.4를 실제 모양 `rebuild_world(suite, seed)`에 맞추고, `gui-spec.md` 12.10 ③ 학습 배치·[다음 배치]·판정, `arm-software-spec.md` 6.10 `connect(fresh)`, `learning-platform.md` 3a 표를 1차로 옮겼다. 2차로 남는 것: GUI ② 수집 화면, 이어 쓰기, [일시 정지], L5.
- **배포 문서 대조**(v0.1, REQ-095): README 빠른 시작·release-checklist B절은 실행 파일 셋과 맞았다. 옛 이름(`시뮬레이션 실행.cmd`·`실물 팔 실행.cmd`)이 남은 `system-architecture.md` 9절 폴더 표와 `gui-spec.md` 시작 화면 줄을 고쳤다. README "지금 상태"를 v0.1·ACT 15~17/50·결정 0072로, release-checklist에 ZIP 줄·v0.1 태그·E절(Setup.exe 최종 배포)을 더했다.
- **ACT 11/23 기준 통과**(11-20, 기획 판단, #494): v2 100k 45/50 → 결정 0071 4항대로 L3·L4를 보조 시연에 넣는다(`learning-platform.md` 3a). 시연 문구는 새 씨앗 50개 값(MuJoCo 세션)으로, 15/50 미만이면 다시 본다. 필수 시연은 초음파(0072). README ACT 줄·평가 명령(`collect_pick2`, `--clock step`) 고침.
- **시뮬레이션 GUI 피드백 F1~F10**(11-20, 총괄 경유): 계획은 `feedback-1120-plan.md`(1차 F1·F4·F5·F9·F6, 2차 F3·F10·F8, 3차 F2·F7은 `ultrasonic-spec.md` 8.9·`gui-spec.md` 12.11 Stage 9 뒤). 사용자 질문 6개는 그 문서 3절.
- **사용자 결정 11-20 아침**(총괄 경유, 결정 기록 없음): ① 사용자에게 날짜·일정·하드웨어 질문을 하지 않는다(`project-status.md` 9.7은 소프트웨어·제품 선택만, 9.1은 "하드웨어 담당 확인 필요"). ② 시연·I1 최소 반경 120 → 150(`ultrasonic-spec.md` 6절·8.5 `show80`·8.6, 구현은 총괄이 REQ로). ③ 기본 그리퍼를 U5 전에 `parallel`로 앞당김(결정 0063 상태 줄, REQ-098 → 팔). ④ Astryx(GUI 디자인 값)는 보류.
- **R1 자리 (280,150)은 그대로**(11-20, 기획 판단, #497): R1은 무시 구역 없이 레이더 정확도를 재고, 통 옆 값으로 통 무시 반경 113mm를 줄일 수 있는지 본다. A1·I1 자리는 시연 배치 규칙(무시 원 둘레 50mm 밖)으로 고른다(`ultrasonic-spec.md` 11절). 8.4·10절·radar-bringup R1 명령은 `radar_check r1`로 고침.

## 11-19 (총괄, v0.1 릴리스)
- 합침: #486 MuJoCo 시험 runs/ 정리 끄기, #487 GUI 정책 실행(세계 다시 짓기·다음 배치·판정), #488 REQ-094 `prune_runs`는 실행 폴더만, #489 REQ-095 실행 파일 하나·`제거.cmd`, #490 GUI 실물 팔 없으면 시뮬레이션으로, #491 ACT 2차 다시 학습 기록(60k 17/50), #492 GUI 시작 페이지 안내, #493 결정 0072. main c380492.
- **v0.1 릴리스**: 레포 공개, 전체 시험 1325 passed·화면 비교 56 passed. 실행 파일은 `설치.cmd`·`SO-101 Studio 실행.cmd`·`제거.cmd`.
- 하드웨어 CAD는 모두 반영(결정 0067). 남은 것은 조립 뒤 H-6 실측.
- 11-20(새 계정): REQ-096(정답 쌍) → 초음파. REQ-088~095를 대장에 뒤늦게 적었다. 설치 목록 맞춤: superpowers·claude-md-management·engineering 플러그인, context7·hf-mcp-server·excalidraw MCP(user), playwright-cli와 스킬. #494 ACT 밤샘 결과 합침: v2(시연 988개) 100k 45/50, 시연용 체크포인트 `runs/train/act_pick2v2_k0clip/checkpoints/100000`(새 씨앗 확인은 MuJoCo). #496 기획 0073·배포 문서.

## 11-16 밤~11-17 (총괄)
- 합침: #410 REQ-067, #411 REQ-068, #412 REQ-069, #413 환경 파일(C-8 버전 고정), #414 기획(Stage 9 꼭 고칠 것 18건, 손목 카메라 값, 당기기 상한 측정 계획, 범위 안 3a, 하드웨어 질문 최종본). 하루 마감 전체 시험(1011f60) 1261 passed, 실패 0.
- EGM Stage 9: (가) 13.10.1·13.10.2, (나) 13.10.4·gui 12.10·arm 6.10 모두 "아직 구현할 수 없음" → 기획이 #414로 반영. 1차 범위는 Stage 9를 다시 돌리지 않고 REQ-070~074. 13.10.2 시계 주입은 12/4 뒤(3a, 11-17 사용자 승인)이고, 구현 전에 그 절만 Stage 9를 다시 돌린다.
- C-8 설치 끝(11-17): `so101`에 `lerobot[dataset]`(numpy 2.2.6 유지) + conda ffmpeg 7.1.1. 섞여 있던 pillow 12.3.0·13.0.0(11-01부터)을 12.3.0 하나로 다시 깔았다. `so101-train`은 torch 2.11.0+cu128 `cuda True`, lerobot 0.6.1, mujoco 4.0.0, so101-nexus-mujoco 0.3.12. 이 PC의 `so101`에 pytest-xdist·pytest-testmon이 없고 `environment.yml`에도 없다(설치는 사용자 결정).
- 실물 팔과 로보나인 그리퍼가 지금 없다(11-17 사용자). U5·U13은 물건이 올 때까지 미룬다. 세션별 다음 일은 `learning-platform.md` 3a "실물 없이 먼저 할 일" 표를 옮겼다.

## 사용자 결정 대기
| # | 무엇을 하면 되나(체크리스트) | 막는 것 |
|---|---|---|
| U5 로보나인 교체 날짜 | ☐ 로보나인 부품(출력물 + 기어·로드·나사)이 다 있는지 확인 ☐ 없으면 출력·구매할 것 목록을 하드웨어 담당에게 받기 ☐ 교체할 날을 정하기(교체 1시간 + 실측 반나절, 사용자가 팔 옆에 있는 날) ☐ 그날 runbook 2-10을 연다 | 2-10, 2-2 재시험, A0 |
| U4 OpenRB 전원·작업대 | ☐ OpenRB-150 보드의 VIN 단자에 무엇이 연결되는지 회로를 확인(12V 어댑터가 닿으면 XL330이 망가진다) ☐ 테스터로 VIN 전압을 재서 5V 근처인지 보기(전원은 5V 강압 모듈이나 USB) ☐ 결과를 총괄에게 알리기 ☐ 작업대 조립 날짜를 정하기(판 600×400, `workbench-hardware-requirements.md`) | 초음파 B1, 레이더 배선, runbook 2-A |
| U15 병렬 손 미끄러짐 기준·조임 힘(11-15) | ☐ MuJoCo 병렬 손 58mm 상자의 들고 가기 미끄러짐이 0.8mm 근처(절반)와 2~10mm로 갈린다(30·45mm는 0.7~1.4mm, `measurements.md`). 기준(임시 4.0mm, `measure` 시험)과 kp·forcerange 잠정값을 언제 정할지. **기획 안(11-16 밤, `learning-platform.md` 3a)**: 로보나인이 와서 runbook 2-10을 한 날 정한다. 그 전에 MuJoCo가 M5 뒤 `pick_grid`를 힘 한계 10·24·36N으로 돌려 민감도 표를 낸다. 그때까지 잠정 4.0mm·10N, 12/4까지 실물이 안 오면 시험장 표에 "조임 힘 잠정" | M5 판정 기준 |
| U13 실물 팔 첫 연결(11-15) | ☐ 보드가 오는 날 정하기. 순서: 캘리브레이션 파일을 LeRobot 캐시에 복사 → `doctor` → `status` → `--speed 20 home` | 실물 시험 전부 |
- 11-15 밤 보고와 기획 요청 원문: `planning-inbox` `inbox/2026-11-15-night-report.md`. 참고: 병렬 손 blocker 30% fault는 막힘 감지 문제가 아니라 시나리오 벽이 home→radar_clear 경로에 걸린 것이었다(MuJoCo #370, 병렬용 시나리오 추가).
- 알려진 실패(main): `tests/test_task_check.py::test_sim_three_objects_end_to_end`. U11(#378, 벤치마크 single 95/88%·demo 98/83%, 총괄 재실행 값은 `planning-inbox` `inbox/2026-11-15-check-yaw-benchmark.md`)로는 풀리지 않았다. 원인은 시험 배치의 모서리 간격 14·35mm와 확인 전 레이더 후보의 도달 거르기이고 REQ-065(팔)로 고친다. 흔들림: `tests/gui/test_watchdog_link.py::test_refused_run_folder_turns_the_watchdog_off`(부하 때 1회, 단독 통과).
- 11-15 사용자 승인: U11 CHECK yaw를 후보 반경별로 켠다(결정 0065. 켜는 PR에서 총괄이 벤치마크 고정 시나리오를 다시 돌려 남긴다), U12 nicegui·viser·playwright 계열을 `environment.yml`에 `==`로 고정(팔 세션 PR, `tooling-setup.md` C-4), U14 13.7 (a) topedge는 단동식 비교 옵션 전용(`mujoco-spec.md` 13.7, 병렬의 대응 실패는 M5에서).
- 11-15 사용자 승인: **U16 실제 CHECK에 첫 점 당기기는 안 A**(`ik_fail`이면 `Probe`가 첫 점을 안쪽으로 당겨 다시 시도, CHECK 한정). 순서: 기획이 `ultrasonic-spec.md` 6절·8.2를 고침 → 총괄이 REQ(초음파 `Probe`·벤치마크, 팔 작업 실행 연동) → 구현 → 총괄이 병렬 kin 벤치마크를 다시 돌려 1절 병렬 표 확정. 기획 반영 전에는 구현하지 않는다(`planning-inbox` `inbox/2026-11-15-benchmark-final.md`).
- U10(시험 계획)은 결정 0064로 승인했다(11-14).
- U7(브래킷 출력 방향: 손목 쪽 평면 +z 바닥, 6.4mm 걸림은 손목 카메라 팔 때문이었고 #357로 0), U8(시연 속도 30~40%), U9(NOTICE 이름 빼기, #347)는 11-14에 사용자가 승인했다. U6은 닫았다.

## 세션별 다음 일
| 세션 | 폴더 | 다음 일 | 기다리는 것 |
|---|---|---|---|
| 총괄 | `so101-studio-gui` | #461(H-3)·#462(REQ-084) 검토·합치기, 하루 마감 전체 시험, 실물 확인 1c 날 사용자 승인 받기 | - |
| 팔 | `so101-studio` | 대기. 실물 확인 1c(home·pick_and_place·데드맨·comm_lost·E-STOP), 로보나인 2-10·2-2 | 실물(U5·U13) |
| MuJoCo | `so101-studio-sim`(작업 worktree `-sim-h3`·`-sim-clock3`는 PR이 합쳐지면 지움) | M5 고정 묶음 표 새로(새 장면, `--clock step`) + `bin_side` → REQ-087 ACT 2차 수집 → `--approach` vertical·auto 비교 | #461·#462 |
| 초음파 | `so101-studio-us` | REQ-081 H-4(레이더 CAD 자리, 순차 뽑기 배치, 몸체·통 벽 겹침) PR | - |

## 시험 결과 (11-14, 총괄)
- 전체 `python -m pytest -q`(eded098, 1:38): 1192 passed, `tests/gui_visual` 실패 6·오류 4(실물 시험과 같이 돌아 CPU가 바빴던 때). ruff 0건.
- 합친 main `tests/gui`: 424 passed. 그 전 두 번의 119 실패는 다른 프로세스가 GUI 포트를 쥔 때라 무효.
- `tests/gui_visual`(5d359b2): 55 passed, 실패 1·오류 3. 보드 연결·로컬 교시 기록에 따라 바뀌는 부분 탓. 배포 전 보드를 뽑고 다시 돌린다.
- **새 PC 전체**(11-15, fcdcb0f = 합친 main과 코드 같음, 보드 없음, `python -m pytest -q --durations=50`, 1:17:21): 1218 passed, 6 failed, 12 errors. ruff 0건. 기준 시간은 이 1시간 17분이다.
  - `tests/gui/test_numeric_input.py` 5개: nicegui 3.18의 `user_simulation`이 `ui.notify`를 바꿔 넣어 monkeypatch 가로채기가 안 먹음 → `user.notify.messages`로 고침(28 passed).
  - `tests/gui_visual` 12개: 새 PC 글꼴 렌더링 차이(배치 같음) → `.mode-detail` `w-full`, 기준 사진 12장 다시 찍음(56 passed). 그 뒤 `tests/gui` 425 passed(10:10).
  - `tests/test_task_check.py::test_sim_three_objects_end_to_end`: picked 0, `give_up` → **풀림**(REQ-065 #386: 확인 전 후보 여유 + 시험 배치 모서리 간격).
  - nicegui·playwright 버전 → **고정함**(U12 #377).

## 나중에 할 일
- Inno Setup 설치 파일(Setup.exe, 앱 제거 등록)은 최종 배포 단계에서(사용자 결정 11-19). v0.1은 `설치.cmd`·`SO-101 Studio 실행.cmd`·`제거.cmd`(REQ-095).
- GUI 알림을 아래로 끌면 흐려지며 사라지게(총괄). 장면 부품 편집기(STEP 뒤), 초음파 화면 실제 기록(B1 때).
- MuJoCo 뷰어 이중 버퍼는 화면 찢어짐이 실제로 보일 때(#354 논의).
- 강화학습(M9)과 손목 USB 카메라(VLA)는 12/4 뒤.
