# 작업 대기열 (총괄이 유지)

상태: 초안 · 2026-11-14 밤 (기획이 새 PC 이전 전에 다시 씀, main 299bea9. 첫날 순서는 새 PC 기획 세션이 절차를 따라가 보고 고침. 이후 총괄이 단계마다 갱신)

> 새 PC·새 세션은 `project-status.md`(총정리) → 이 문서 순서로 읽는다. 사용자 결정이 필요한 것은 "사용자 결정 대기"에 쌓고, 그 밖의 일은 기다리지 않고 진행한다.

## 새 PC 첫날 순서
1. **이전**(`session-workflow.md` 8절 "새 PC로"): 클론, 환경(+ `python -m playwright install chromium`, MuJoCo는 `tooling-setup.md` C24), 폴더 넷(2절: 팔 폴더만 `main`, 나머지 셋은 `--detach origin/main`), `so101-이전-1114.zip` 풀기, 세션 다섯을 시작 프롬프트로.
2. **합친 main 전체 시험**(총괄): 11-14에 끝까지 못 돌렸다. **팔 보드를 꽂기 전에**, 다른 GUI·감시 프로세스를 끄고 `python -m pytest -q`(지금 방식 그대로, 약 1시간 40분. 결정 0064의 `-n auto`·`--testmon`·`measure` 표시는 아래 "이어지는 순서" 5번이 끝나야 쓸 수 있다), `python -m ruff check src tests`. 도는 동안 다른 세션은 `tests/gui`·`tests/gui_visual`을 돌리지 않는다(고정 포트). 통과 수와 실패 이름을 이 문서에 적는다.
3. **실물 팔 첫 연결**(사용자 + 팔 세션, 2번이 끝난 뒤): 새 PC에는 LeRobot 캐시가 없다. `config/calibration/so101_follower_01.json`을 `%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\`로 복사 → `so101.cmd doctor` → `so101.cmd status` → `so101.cmd --speed 20 home`(클램프·전원 스위치 옆). 복사하지 않으면 "캘리브레이션 불일치"로 잠긴다. 0° 확인 기록은 git의 팔 파일에 있고 해시가 맞아(#344) `calibrate --verify-zero`를 다시 하지 않아도 움직인다(다시 하는 때: 캘리브레이션 파일이 바뀐 뒤, 로보나인 교체 뒤).
4. **EGM drift-report**(총괄): 대상 `mujoco-spec.md`(13.9·13.10 vs M4b 코드 #348~#357). 결과의 꼭 고칠 것만 `planning-inbox` `inbox/`로.
5. 그다음은 아래 "이어지는 순서".

## 이어지는 순서
1. **로보나인으로 다시 맞추기**(결정 0063): 교체(U5) → 그리퍼 범위 캘리브레이션 → runbook 2-10(폭 표·접촉·조임) → 2-2 재시험 30%·50%(결정 0063 그대로, 사용자 확인 11-14. 0061 문턱 다시 재기, `runs/handoff-1114/real_22.py`) → 브래킷 출력(U7 결정: 손목 쪽 평면 +z를 바닥)·장착 → A0.
2. **CHECK 경로 기준점 REQ**(팔, 기획 결정 아래): 병렬일 때 `tool_position`·`follow_path`도 두 턱 끝 가운데(TCP) 기준으로.
3. **MuJoCo**: 병렬 손으로 `mujoco-spec.md` 13.7 (a)(b)·`sim_blocked`·부하 문턱 다시 재기 → 기획이 M5·M6 세부 설계(EGM 결과 뒤, EGM Stage 9).
4. **초음파**: 오프셋은 #359로 넣었다({0.6, 66.5, 99.5}, `ring_r_mm` 60). 새 센서 높이(약 154.5mm)에서 CHECK 높이·경로를 벤치마크로 다시 고른다. 부품 뒤 B1~B4 → R1 → A0·A1 → I1.
5. **시험 시간 줄이기**(결정 0064, 11-14 승인): 총괄은 시험 포트 0·xdist·loadgroup·testmon 설정·화면 비교 마스킹, 각 세션은 `--durations=50`으로 자기 영역 느린 시험 정리(측정 시험은 `measure` 표시). 첫 전체 시험(첫날 2번)은 지금 방식으로 돌려 기준 시간을 남긴다.
6. **하드웨어**: H1 레이더 부품, H2 OpenRB VIN, H5 작업대, H7 카메라 주문.
7. 비교 실험(11/23~29) → 마무리(11/30~12/4, `release-checklist.md`). 11/30부터 새 기능은 멈춘다.
- 실물이 먼저다. 하드웨어를 기다리는 동안은 MuJoCo 시험장이 정식 작업이다(결정 0062).

## 기획 결정 (11-14 밤, 총괄 저녁 보고 7·9절)
- **CHECK 경로 기준점**: (a) 병렬일 때 `tool_position`·`follow_path` 기준을 두 턱 끝 가운데로 한다. 결정 0043-1이 이미 "병렬의 TCP는 두 턱 가운데"이고, #358로 센서 기준점도 가운데가 됐으므로 기준점을 하나로 맞춘다. 단동식은 그대로(고정 턱 끝). (b) "CHECK 전에 닫기"는 다른 `follow_path` 쓰임새의 어긋남을 남겨서 고르지 않는다. 팔 REQ로 낸다(GUI 현재 좌표 표시도 같이 바뀌는지 확인).
- **팀 CAD 원본**: 레포에 넣지 않는다. 팀 드라이브 한 곳을 기준으로 두고 `assets/scene/NOTICE.txt`에 그 위치(드라이브 폴더 이름)를 적는다. 레포에는 3D·충돌용 파생 STL만 둔다(`release-checklist.md` A절).
- **배출 구역 폭**: w100×h80 그대로(총괄 판단, 사용자 위임). 작업대(H5) 조립 때 다시 본다.
- **`real_22.py`**: 로보나인 2-2 재시험에서 다시 쓴다. 계속 쓰면 시험장(결정 0062)의 실물 실행기로 레포에 넣을지 그때 정한다.
- **`backup/tmp-merge-b59d933`**: 새 PC로 가져가지 않는다(main에 다 합쳐졌고 옛 임시 합치기).

## 사용자 결정 대기
| # | 무엇을 하면 되나(체크리스트) | 막는 것 |
|---|---|---|
| U5 로보나인 교체 날짜 | ☐ 로보나인 부품(출력물 + 기어·로드·나사)이 다 있는지 확인 ☐ 없으면 출력·구매할 것 목록을 하드웨어 담당에게 받기 ☐ 교체할 날을 정하기(교체 1시간 + 실측 반나절, 사용자가 팔 옆에 있는 날) ☐ 그날 runbook 2-10을 연다 | 2-10, 2-2 재시험, A0 |
| U4 OpenRB 전원·작업대 | ☐ OpenRB-150 보드의 VIN 단자에 무엇이 연결되는지 회로를 확인(12V 어댑터가 닿으면 XL330이 망가진다) ☐ 테스터로 VIN 전압을 재서 5V 근처인지 보기(전원은 5V 강압 모듈이나 USB) ☐ 결과를 총괄에게 알리기 ☐ 작업대 조립 날짜를 정하기(판 600×400, `workbench-hardware-requirements.md`) | 초음파 B1, 레이더 배선, runbook 2-A |
| U13 실물 팔 첫 연결(11-15) | ☐ 보드가 오는 날 정하기. 순서: 캘리브레이션 파일을 LeRobot 캐시에 복사 → `doctor` → `status` → `--speed 20 home` | 실물 시험 전부 |
- 11-15 밤 보고와 기획 요청 원문: `planning-inbox` `inbox/2026-11-15-night-report.md`. 참고: 병렬 손 blocker 30% fault는 막힘 감지 문제가 아니라 시나리오 벽이 home→radar_clear 경로에 걸린 것이었다(MuJoCo #370, 병렬용 시나리오 추가).
- 알려진 실패(main): `tests/test_task_check.py::test_sim_three_objects_end_to_end`(U11 뒤 풀림). 흔들림: `tests/gui/test_watchdog_link.py::test_refused_run_folder_turns_the_watchdog_off`(부하 때 1회, 단독 통과).
- 11-15 사용자 승인: U11 CHECK yaw를 후보 반경별로 켠다(결정 0065. 켜는 PR에서 총괄이 벤치마크 고정 시나리오를 다시 돌려 남긴다), U12 nicegui·viser·playwright 계열을 `environment.yml`에 `==`로 고정(팔 세션 PR, `tooling-setup.md` C-4), U14 13.7 (a) topedge는 단동식 비교 옵션 전용(`mujoco-spec.md` 13.7, 병렬의 대응 실패는 M5에서).
- U10(시험 계획)은 결정 0064로 승인했다(11-14).
- U7(브래킷 출력 방향: 손목 쪽 평면 +z 바닥, 6.4mm 걸림은 손목 카메라 팔 때문이었고 #357로 0), U8(시연 속도 30~40%), U9(NOTICE 이름 빼기, #347)는 11-14에 사용자가 승인했다. U6은 닫았다.

## 세션별 다음 일
| 세션 | 다음 일 | 기다리는 것 |
|---|---|---|
| 총괄 | 전체 시험, EGM drift-report(mujoco-spec), gui_visual `.mode-detail` 가림 폭 고정 + 스냅숏(보드 뽑고), CHECK 기준점은 REQ-064로 냄(11-14) | - |
| 팔 | 새 PC 첫 연결(캐시 복사 → doctor → 20% home, 사용자와), CHECK 기준점 REQ-064(a). U5 뒤 로보나인 2-10·2-2 | U5 |
| MuJoCo | 병렬 손으로 13.7 (a)(b)·`sim_blocked`·부하 문턱 다시 재기 | - |
| 초음파 | CHECK 높이·경로 다시 고르기(벤치마크. 병렬 손은 손끝 55mm에서 `follow_path`가 ik_fail이라 `--sim run --source ultrasonic`이 CHECK unreachable로 끝난다, 80mm는 풀림: 팔 세션 #364 보고), 부품 뒤 B1~ | H1·H2·U4 |

## 시험 결과 (11-14, 총괄)
- 전체 `python -m pytest -q`(eded098, 1:38): 1192 passed, `tests/gui_visual` 실패 6·오류 4(실물 시험과 같이 돌아 CPU가 바빴던 때). ruff 0건.
- 합친 main `tests/gui`: 424 passed. 그 전 두 번의 119 실패는 다른 프로세스가 GUI 포트를 쥔 때라 무효.
- `tests/gui_visual`(5d359b2): 55 passed, 실패 1·오류 3. 보드 연결·로컬 교시 기록에 따라 바뀌는 부분 탓. 배포 전 보드를 뽑고 다시 돌린다.
- **새 PC 전체**(11-15, fcdcb0f = 합친 main과 코드 같음, 보드 없음, `python -m pytest -q --durations=50`, 1:17:21): 1218 passed, 6 failed, 12 errors. ruff 0건. 기준 시간은 이 1시간 17분이다.
  - `tests/gui/test_numeric_input.py` 5개: nicegui 3.18의 `user_simulation`이 `ui.notify`를 바꿔 넣어 monkeypatch 가로채기가 안 먹음 → `user.notify.messages`로 고침(28 passed).
  - `tests/gui_visual` 12개: 새 PC 글꼴 렌더링 차이(배치 같음) → `.mode-detail` `w-full`, 기준 사진 12장 다시 찍음(56 passed). 그 뒤 `tests/gui` 425 passed(10:10).
  - `tests/test_task_check.py::test_sim_three_objects_end_to_end`: picked 0, `give_up`. 초음파 세션이 원인 확인 중(#359 뒤 성능 저하로 추정).
  - nicegui·playwright 버전이 `environment.yml`에 고정돼 있지 않아 PC마다 결과가 달라진다(아침 결정).

## 나중에 할 일
- GUI 알림을 아래로 끌면 흐려지며 사라지게(총괄). 장면 부품 편집기(STEP 뒤), 초음파 화면 실제 기록(B1 때).
- MuJoCo 뷰어 이중 버퍼는 화면 찢어짐이 실제로 보일 때(#354 논의).
- 강화학습(M9)과 손목 USB 카메라(VLA)는 12/4 뒤.
