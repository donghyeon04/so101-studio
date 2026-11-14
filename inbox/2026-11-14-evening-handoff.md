# 총괄 저녁 보고: 새 PC 이전 전 마무리 (2026-11-14 저녁, main b6f2333)

상태: 초안 · 총괄(GUI) 작성, 기획 입력용

## 0. 한 줄
오늘 일은 모두 main에 합쳤고, 세션 넷(총괄·팔·MuJoCo·초음파)은 손에 쥔 일이 없다. 사용자는 **새 PC로 옮겨 세션을 새로 시작한다.** 기획이 할 일은 셋이다. ① 새 PC 이전 문서·프롬프트와 이후 계획을 짜 달라(3절). ② 시험이 너무 오래 걸리는 문제를 봐 달라(4절). ③ EGM을 돌릴 시점인지 판단해 달라(5절).

## 1. 오후 보고(adef581) 뒤에 합친 것 (#341~#357)
| PR | 내용 |
|---|---|
| #341 | #337 회귀: test_core가 COM3을 기대하던 것 |
| #342 | parts_eval.overhang 방향 이름 바로잡음(손목 쪽 평평한 면 +z를 바닥에 두면 0%) |
| #343 | 기획: 결정 0063, M4b, 총정리, 보고 체계, 시작 프롬프트, gui-spec 12.9 |
| #344 | 팔 파일 0° 확인 기록 11-14(이제 git에 있음) |
| #345 | work-queue: U7·U8·U9 사용자 승인 |
| #346 | CLI run도 남은 물체(known_objects)를 팔에 알림, 놓을 칸 순서 |
| #347 | NOTICE에서 팀원 이름 삭제(U9) |
| #348 | **M4b 로보나인 손 MuJoCo 모델**(FK 0.002mm 일치, 클램프 연동, 상자 충돌, carry 30/45/58 통과, kp·힘은 잠정) |
| #349 | **so101.cmd/sh가 그 폴더의 src를 쓰게**(전에는 editable 설치 때문에 어느 폴더에서 실행해도 팔 폴더 코드를 썼다) |
| #350 | MuJoCo 손에서 SO-101 손목 카메라 뺀 파생 STL(parallel_hand_nocam), scene.yaml 색 적용 |
| #352·#353 | 집은 뒤 상승 = max(40, 남은 물체 최고 높이 + 10), GUI도 높이를 넘김 |
| #354 | MuJoCo 뷰어 sync가 물리를 막지 않게(60초 버림 184번 → 1번) |
| #355 | **gui-spec 12.9**: 되돌리기 알림 줄이기(0.6초 안 연속 변경은 한 번), 수동 입력 물체도 Ctrl+Z, 실행하면 기록 비움 |
| #356 | 배출 칸 겹침: 놓을 물체 사각형과 놓인 물체 사각형 겹침 검사, 구역 안 5mm 후보. MuJoCo 기본 3개 병렬·단동식 모두 접촉 0 |
| #357 | URDF gripper_link도 nocam 메시. **M7a의 "손 메시가 센서 앞 6.4mm"는 SO-101 손목 카메라 팔 때문이었고 이제 0**(사용자 CAD 확인 필요 없어짐) |

사용자 확인: MuJoCo 뷰어로 로보나인 + 브래킷(주황) + HC-SR04(파랑) 조립을 봤다(#348·#350 뒤).

## 2. 새 PC로 옮길 때 알아야 할 것 (로컬 정리 결과)
- 원격 브랜치: `main`, `planning-inbox`, `claude/hopeful-hamilton-amqaac`만 남았다.
- 로컬 폴더(모두 main b6f2333 detach): `so101-studio`(팔), `so101-studio-gui`(총괄), `so101-studio-sim`(MuJoCo), `so101-studio-us`(초음파), `so101-studio-view`(**MuJoCo 확인용**, 로컬만: `config/local.yaml` `gripper: parallel`, `config/scene.yaml` 손목 부품 두 줄 주석 풀림). 임시 worktree는 모두 지웠다. 합쳐진 로컬 브랜치 50개도 지웠다.
- git 밖에만 있는 것(새 PC로 옮길지 정해 달라):
  - `so101-studio-gui/runs/handoff-1114/`: 실물 2-2 스크립트 `real_22.py`(아직 레포 도구가 아님. 넣을지 정해 달라), 브래킷 출력 방향 3D 페이지 `bracket-print.html`, 오전 보고, EGM 남은 패치
  - 실물 기록: `so101-studio-gui/runs/real-eval-20261113/`, `so101-studio/runs/from-armtmp/`, 각 폴더 `runs/`
  - 사용자 교시 기록: `so101-studio/config/sequences/1234.json`, `so101-studio-gui/config/sequences/test.json`(추적 안 함)
  - `so101-studio-gui/.claude/launch.json`(총괄 GUI 미리보기 설정)
  - 팀 CAD: `C:\Users\김동현\Downloads\브라켓 설계 초안\cad\Gripper(브라켓 짧은 버전).step/.stl`(기준 형상, 결정 0063)
  - 로컬 브랜치 `backup/tmp-merge-b59d933`(옛 임시 합치기, main에 없음. 버려도 되는지)
  - 팀 공유 3D 페이지: https://claude.ai/artifact/Vu2wdcf9dyPejksP9yv4aL (브래킷 출력 방향, 사용자 계정)
- 환경 주의: 새 PC는 `tooling-setup.md`로 맞춘다. MuJoCo 고정 버전은 C24. 실물 팔은 클론 뒤 `calibrate --verify-zero`로 0° 확인을 다시 해야 움직인다(캘리브레이션 파일이 PC의 LeRobot 캐시에도 있어야 한다).
- **부탁**: 새 PC 이전 문서, 세션별 시작 프롬프트, 이후 계획은 기획이 쓴다(사용자 지시). 총괄 새 세션이 그 문서로 가이드를 받아 시작한다.

## 3. 다음에 할 일 (새 세션 몫)
| 담당 | 일 |
|---|---|
| 기획 | M5 집기 시험장·M6 초음파 MuJoCo 세부 설계(로보나인 손 위에서). 로보나인 레퍼런스 조사(손목 기본 방향 가로/세로, 남들 초기값). 데스크탑 PC 필요 시점(M5b 대량 시험·M9 강화학습: 물리는 CPU, MJX/Warp는 NVIDIA GPU·리눅스) 기록. `hardware.md`·`overview.md` 팀원 이름 |
| MuJoCo | 병렬 손으로 13.7 (a)(b)·sim_blocked·부하 문턱 재측정 |
| 팔 | (U5 뒤) 교체 → 그리퍼 범위 캘리브레이션 → 2-10 → 2-2 재시험 |
| 초음파 | 센서 오프셋 REQ(오프라인이라 대기, 값 {0.6, 66.5, 99.5}, 높이 약 155mm), CHECK 높이·경로 다시 고르기 |
| 총괄 | gui_visual: `.mode-detail` 가림 폭 고정 + 스냅숏(보드 뽑고) |
| 사용자 | U5 교체 날짜, U4, 배출 구역 폭(아래) |
- **새 결정 거리**: 배출 구역(w100×h80)에 100×30 물체 3개를 넣으면 이웃 간격이 좁아 벌린 턱이 못 들어가 `ARM-OPENING-PATH-FALLBACK`(제자리 열기, 접촉은 없음)이 남는다. 없애려면 구역을 넓히는 작업대 배치 결정이 필요하다(#356).
- MuJoCo 뷰어 이중 버퍼는 화면 찢어짐이 실제로 보일 때 한다(#354 논의).

## 4. 문제 제기: 시험이 너무 오래 걸린다 (사용자 요청, 레퍼런스 조사 부탁)
- 지금: 전체 `python -m pytest -q` **1시간 38분**(eded098, 1192개), `tests/gui` 424개 26분, `tests/gui_visual` 12분, slow 표시가 330개 넘는다. 가장 느린 것: `test_task_check` 151초, `test_release_min_tip` 102초, `test_mujoco_driver::test_realtime_60s` 70초, MuJoCo·작업 실행 시뮬레이션 다수 40~70초.
- 겪은 문제: GUI 시험이 고정 포트라 다른 프로세스(사용자 GUI, 감시 프로세스)가 포트를 쥐면 119개가 줄줄이 RuntimeError로 넘어진다(코드는 멀쩡). `gui_visual`은 보드를 꽂았는지·로컬 교시 기록에 따라 흔들린다. 하루에 같은 묶음을 여러 번 다시 돌렸다.
- 사용자 질문: 모두 필요한 시험인가, 허수 시험(같은 것을 여러 번 보거나 실물과 무관한 것)은 없는가. 코드·GUI·MuJoCo 시험 전부에 대해 **레퍼런스 조사**(다른 로봇·GUI 프로젝트의 시험 구성, pytest-xdist 병렬, 시험 선택(변경 파일 기준 testmon 등), 포트 0 동적 할당, 시각 회귀 도구, MuJoCo 시뮬레이션 시험을 시뮬레이션 시간으로 빠르게(M5b와 겹침))를 돌려 바꿀 것·지울 것·합칠 것을 제안해 달라. 결정 0046(전체는 하루 마칠 때 총괄이 한 번)도 함께 다시 보자.

## 5. 질문: EGM을 돌릴 때인가
- 오늘 설계서가 크게 바뀌었다: mujoco-spec 분리·13.10(M4b·M7a), 결정 0062·0063, gui-spec 12.8 확정·12.9, project-status·work-queue 다시 씀, session-workflow 7a·8. 결정 0048·0056 규칙상 "큰 합치기가 몰린 날 마칠 때 설계서 하나에 drift-report"다. 새 PC로 옮기기 전에 한 번 돌릴지, 옮긴 뒤 첫 일로 할지, 대상 문서(mujoco-spec 13.10 또는 project-status)를 정해 달라. 돌리면 총괄이 `egm:egm-synthesizer`로 돌리고 결과는 planning-inbox로 보낸다.

## 6. 시험 상태 (이 보고 기준)
- 합친 main 관련 묶음: #355 GUI 67 passed, #356·#357 합본 test_scene3d·test_backend·known_objects·parts_eval 49 passed, MuJoCo 관련 37~43 passed. ruff 0건.
- **합친 main 전체 시험은 오늘 끝까지 다시 돌리지 않았다**(마지막 전체 eded098). 새 PC 첫 일로 돌린다.

## 7. 추가: 초음파 센서 오프셋 기준점 (초음파 세션, 결정 필요)
- 초음파 세션이 REQ 1단계를 확인했다. 축과 부호는 CAD와 같다(x 벌림 축, y 옆, z 손끝에서 손목 쪽). **원점이 다르다.** 코드의 "손끝"(`arm_service.tool_position` = `jaw_tip_offsets()[0]`)은 병렬일 때 **한쪽 턱 끝**(+x, TCP에서 gap/2)이다. CAD 값 {0.6, 66.5, 99.5}는 **두 턱 끝 가운데** 기준이다. 그대로 넣으면 x가 벌림/2(최대 42mm)만큼 어긋나고, CHECK 위치 오차로 바로 간다. z·y는 맞다(CHECK 높이 55에서 센서 약 154.5mm).
- 고치는 길: (a) 병렬일 때 tool_position(센서 기준)을 가운데로(팔 영역, arm_service·follow_path 영향, 팔 REQ) / (b) CHECK 중 그리퍼를 닫힘(0)으로 고정한다고 약속하고 CAD 값 그대로(가장 작음, runner가 실제로 닫는지 확인 필요).
- 총괄 추천 (a). 값은 바꾸지 않았다(초음파 세션 대기). 기획이 정해 다음 세션 REQ로.

## 8. 추가 (사용자 답, 11-14 저녁)
- **배출 구역 폭(총괄 판단, 사용자 위임)**: 지금 w100×h80 그대로 둔다. 100×30 물체 3개일 때 남는 `ARM-OPENING-PATH-FALLBACK`은 제자리 열기라 접촉이 없고(#356 MuJoCo 확인), 시연 물체(58 상자)는 한 칸에 하나씩 들어가 폴백이 없다. 작업대(H5)를 조립할 때 실제 공간을 보고 넓힐지 다시 정한다.
- **U4·U5**: 사용자가 "무엇을 하면 되는지 모르겠다"고 했다. 기획이 사용자가 할 일을 쉬운 체크리스트로 풀어 달라. 예) U5: 로보나인 그리퍼 부품이 있는지·출력했는지 → 교체할 날짜. U4: OpenRB 보드 VIN 단자에 12V가 들어가는지 테스터로 보기, 작업대 조립 날짜.
- **git 밖 파일 이전**: 바탕화면 `so101-이전-1114.zip`(6.4MB)에 모았다(팀 CAD 원본 폴더 `브라켓 설계 초안` 37MB 포함, README.txt에 각 파일이 새 PC 어디로 가는지). 레포 전체 `runs/`(약 700MB 실행 기록)는 넣지 않았다.
- **CAD 원본을 git에 올릴지(사용자 질문)**: 총괄 의견은 레포에 넣지 않는다. 배포 점검표 A절("STEP 원본은 레포 밖, 3D·충돌에 쓰는 STL만 assets/에")과 맞고, 36MB 바이너리가 모든 클론과 기록에 영원히 남는다. 레포에는 파생 STL이 이미 있다. 원본은 팀 드라이브(또는 따로 비공개 레포·GitHub Release 첨부) 한 곳을 기준으로 두고 `assets/scene/NOTICE.txt`에 그 위치를 적자. 기획이 정해 달라.
