# 작업 대기열 (총괄이 유지)

상태: 초안 · 2026-11-07 시작 (GUI 세션 = 총괄이 단계마다 갱신)

> 대화가 압축되거나 세션이 바뀌어도 이 문서부터 읽고 이어 간다. 사용자 지시(2026-11-07 새벽)의 큰 틀과 진행 상황을 한곳에 둔다. 사용자 결정이 필요한 것은 끝의 "결정 모음"에 쌓아 두고, 그 밖의 일은 기다리지 않고 진행한다. 사용 한도가 걸렸다가 풀리면 이 순서대로 이어 간다.

## 사용자 지시 (원문 요약)
1. 실물 없이 시뮬레이션으로 되는 범위와 먼저 개발할 수 있는 범위는 계속 진행한다(세 세션 모두).
2. GUI 전체를 Playwright와 frontend-design으로 평가·검수하고, 고칠 곳은 스스로 고친다.
3. 전체 문서 정리: GitHub PR, md 파일, 실제 개발 파일, 병목, 옛 정보와 새 정보 섞임, README 이상한 곳. 압축이 일어나도 이어지게 레포 안 현황을 계속 유지한다.
4. GUI·SO-101 팔·프로젝트 전체에 더하면 좋을 기능·화면·아이디어 10가지 이상. 같은 일을 하는 사람들의 레퍼런스를 조사해 근거로 든다.
5. 이후 작업 정리:
   1. 실물 팔이 있어야 하는 작업: 그대로 따라 할 수 있는 절차서.
   2. 초음파: 레이더 서보를 **OpenRB-150 + XL330-M288-T**(DYNAMIXEL)로 바꾼다. ESP32가 OpenRB-150에 연결되고 OpenRB-150이 XL330을 돌린다(배선은 하드웨어 담당 송민준이 확인 중). 즉 "모터를 ESP32에 붙여 쓴다"고 본다. 초음파 브래킷은 기존 카메라 브래킷 구조를 고쳐 쓴다. 배치는 지금과 같이 손목(위에서 보는 쪽) 하나와 책상에 하나. 초음파 세션의 코드·구성·계획을 이에 맞게 바꾼다.
   3. 그리퍼: 지금의 한쪽만 열리는 단동식 핑거 그리퍼와 **SO-ARM100/101/102 Parallel Gripper**를 둘 다 쓰고 고를 수 있게 한다. 송민준이 Parallel Gripper에 기능을 더하는 중이지만 원래 구조에서 크게 벗어나지 않는다. 그 구조·예제·레퍼런스로 그리퍼 선택 확장성을 팔·GUI에 넣는다. 초음파 브래킷 위치는 그대로라 초음파 쪽 영향은 다시 확인한다.
   4. STEP 파일: 직접 설계·소량 수정한 부품(바닥 초음파 브래킷, 그리퍼 등)을 시뮬레이션·3D에 넣기 어렵지 않은지. STEP 등 3D 파일을 URDF 등으로 바꾸는 레퍼런스를 조사해 제안한다.
6. 판단이 필요한 것은 모아 두었다가 사용자가 일어나서 한 번에 정한다.

## 진행 (위에서부터)
| # | 단계 | 맡는 세션 | 상태 | 메모 |
|---|---|---|---|---|
| 1 | 시뮬레이션 개발 계속: GUI의 REQ-037 화면(손끝 방향 pitch, 움직임 범위 링, 조이스틱 pitch), G13 조이스틱, G16 간편 이동, 팔 그리퍼 인터페이스(D3와 무관한 부분) | 팔, GUI | 진행 | #135·#137·#143(팔), #150(GUI REQ-034) 합침 |
| 2 | GUI 점검(Playwright 스크린샷, frontend-design 기준)과 수정 | GUI | 완료 | #152. 남김: E-STOP이 좁은 화면에서 조그 버튼 일부를 가림(스크롤로 보임), 작업대 저장 뒤 3D 그림은 다시 시작해야 바뀜, 초음파 화면은 아직 가상 실행만 그림 |
| 3 | 문서·PR·README 정리, 현황 유지 | GUI(총괄) | 진행 | #142(CLAUDE.md·README·가이드), #147(project-status·요청 대장), 브랜치·PR 정리는 D5 |
| 4 | 기능 제안 10+ (레퍼런스 조사) | GUI(총괄) | 완료 | 결정 모음 D2 |
| 5-1 | 실물 절차서 | GUI(총괄) | 완료(초안) | `docs/guides/real-arm-runbook.md`, 실물 시험 때마다 갱신 |
| 5-2 | 초음파 HW 변경(OpenRB-150 + XL330) 반영 | 초음파 | 진행 | 설계안 D4, 준비(서보 드라이버 분리) #144 합침, 본공사는 D4 결정 뒤 |
| 5-3 | 그리퍼 선택 확장성 | 팔, GUI | 진행 | 설계안 D3, 인터페이스·단동식 이사(동작 변경 0)는 REQ-037 뒤 |
| 5-4 | STEP → URDF 조사·제안 | GUI(총괄) | 완료 | 결정 모음 D1 |

## 결정 모음 (사용자가 일어나서 한 번에)
(단계를 진행하며 여기에 쌓는다. 항목마다 추천안과 이유.)

### D1. STEP·직접 설계 부품을 시뮬레이션·3D에 넣는 방법 (5-4, 조사 2026-11-07)
- 문제: 지금 3D(Viser)와 기구학은 SO-101 URDF(메쉬 포함)만 안다. 바닥 초음파 스테이션, 손목 브래킷, 바꾼 그리퍼 같은 직접 설계·소량 수정 부품은 보이지도 충돌 검사에 들어가지도 않는다.
- 후보: (A) CAD에서 부품마다 STL/OBJ로 내보내 "장면 부품 목록"(파일, 붙는 곳 = 작업대 또는 팔 링크, 위치·자세)으로 3D와 충돌 검사에 올리기. (B) 관절이 있는 부품(Parallel Gripper)은 URDF를 다시 만들기: Onshape이면 onshape-to-robot, Fusion이면 ExportURDF/FusionSDF, FreeCAD면 CROSS(freecad.cross)·RobotCAD, STEP 하나뿐이면 urdf-from-step. (C) 파이썬 자동화: cadquery(OCP)로 STEP→STL, trimesh로 단순화, CoACD로 충돌용 볼록 분해.
- 총괄 추천: (A)를 먼저(코드 작음, GUI·팔 공통 "장면 부품" 설정 하나), 그리퍼 두 종은 (B)로 URDF 변형 두 개(단동식·Parallel)를 두고 설정으로 고르기, (C)는 부품이 많아질 때. SO-ARM URDF가 어떤 도구로 만들어졌는지(Onshape 원본 여부)는 확인 필요.
- 출처: onshape-to-robot(github.com/Rhoban/onshape-to-robot), ExportURDF(github.com/daviddorf2023/ExportURDF), FusionSDF(discourse.openrobotics.org/t/fusionsdf…/39430), freecad.cross(github.com/galou/freecad.cross), RobotCAD(github.com/drfenixion/freecad.overcross), urdf-from-step(wiki.ros.org/urdf_from_step), CoACD(github.com/SarahWeiii/CoACD).
- 결정 필요: A부터 할지, 부품 CAD 원본이 어떤 도구(Onshape/Fusion/FreeCAD/SolidWorks)인지.

### D2. 더하면 좋은 기능 후보 (4단계, 조사 2026-11-07)
같은 SO-100/101로 프로그램을 만드는 프로젝트들: Hugging Face LeLab(브라우저에서 캘리브레이션·원격 조작·기록·학습·추론·재생·Hub 업로드), phosphobot(키보드·게임패드·리더 팔·Meta Quest 원격 조작, 데이터 기록, ACT·smolVLA 등 학습·추론, pybullet 시뮬), Beam Bots SO-101(시뮬레이션 대시보드, 3D와 관절 막대), Neuracore 예제(리더-팔로워, 웹에서 기록 시작·정지), LeRobot→Foxglove 실시간 보기, gym-soarm(MuJoCo 환경), ManiSkill SO-100.
| # | 기능 | 근거(누가 하나) | 우리에게 맞는 이유 | 크기 |
|---|---|---|---|---|
| 1 | 교시·작업 기록을 LeRobot 데이터셋 형식으로 저장, Hub 업로드 | LeLab, phosphobot | 4단계 학습 정책(ACT) 준비. 지금 교시 기록을 그대로 활용 | 중 |
| 2 | 리더 팔(두 번째 SO-101) 원격 조작 | LeLab, Neuracore, LeRobot | 사람 시연 데이터를 빠르게 모음 | 중(리더 팔 필요) |
| 3 | 게임패드 조작(브라우저 Gamepad API) | phosphobot | G13 조이스틱과 같은 입구, 시연 때 직관적 | 소 |
| 4 | 학습한 정책 실행 버튼(추론) | LeLab, phosphobot | 1·2 뒤 같은 화면에서 실행 | 중 |
| 5 | 물리 시뮬레이션(MuJoCo) 백엔드 | gym-soarm, ManiSkill | 지금 SimDriver는 기구학뿐이라 접촉·집기 실패를 못 봄(오늘 턱-책상 간섭 같은 것) | 대 |
| 6 | 장면 충돌 검사(작업대·물체·부품 메쉬) | MoveIt 방식, trossen sim | 턱 끝 간섭을 계획 단계에서 막음. D1의 장면 부품과 같이 | 중 |
| 7 | 웨이포인트·블록 프로그래밍(집기→이동→놓기 순서를 끌어 놓기로) | 교시형 로봇 HMI(lead-through 프로그래밍) | 직접 조작 집어 옮기기를 여러 단계 프로그램으로 확장 | 중 |
| 8 | 서보 건강 대시보드와 경고(전압·온도·통신 끊김·리셋 횟수 추이) | LeRobot 기록, 실물 경험 | 오늘 1번 서보 리셋 같은 문제를 일찍 알림 | 소 |
| 9 | 캘리브레이션 마법사(그림 안내, 측정 비교표) | LeLab 안내형 설정 | AC4·`--verify-zero`를 화면에서 | 중 |
| 10 | 실험 실행기(초음파 vs 카메라 비교 실험 자동 반복·지표·보고서) | 비교 실험 설계서 | 프로젝트 목표(비교)를 반복 가능하게 | 중 |
| 11 | 실시간 기록을 Foxglove/Rerun으로 내보내기 | LeRobot→Foxglove | 긴 실험 분석을 외부 도구로 | 소 |
| 12 | 장면 부품 편집기(STL 불러와 작업대에 배치) | D1 | 브래킷·스테이션을 화면에서 맞춤 | 중 |
| 13 | 팀 공유용 읽기 전용 보기(조작권 없이 상태·3D만) | LeLab 웹 | 시연·원격 확인 | 소 |
- 결정 필요: 다음 기간에 어떤 것을 할지(총괄 추천 순서: 8 → 3 → 6(D1 A와 함께) → 1 → 7).
- 출처: github.com/huggingface/lelab, github.com/phospho-app/phosphobot, docs.foxglove.dev/docs/getting-started/robots/so-100, hexdocs.pm/bb_so101, pypi.org/project/gym-soarm, maniskill.readthedocs.io(so100).

### D3. 그리퍼 두 종 선택 (5-3, 팔 세션 조사·설계 2026-11-07)
- 조사: Robonine SO-ARM100/101/102 Parallel Gripper(github.com/roboninecom/SO-ARM100-101-Parallel-Gripper) = STS3215 1개(ID 6) → 기어·랙 2개 → 두 턱이 로드 위를 대칭 이동. 행정 84mm, 파지력 최대 120N, 최대 14mm/s, 반복도 0.5mm. 부품·조립 STL 공개(CERN-OHL-P-2.0 / 코드 Apache-2.0), 그리퍼 단독 URDF·폭↔서보값 표는 없음(직접 재야 함). LeRobot은 그리퍼도 캘리브레이션 범위로만 다룸.
- 설계안: `Gripper` 인터페이스(폭↔값, 두 턱 끝·중간점, 보정 필요 여부, 기울기, 위로 좁아지는 비율, 닫기·열기 경로, 바닥 검사용 메쉬). 지금 단동식 전용 코드(kinematics·skills·safety·targets 곳곳)를 `SingleJawGripper`로 옮기고(동작 변경 0), `ParallelGripper`는 보정 0·기울기 0·높이 규칙 W ≤ G − 4. 설정 `config/arms/<팔>.yaml`의 `gripper: single_jaw | parallel`, 세부는 `config/grippers/<종류>.yaml`. 병렬 URDF는 gripper_link 아래만 바꾼 `assets/so101_parallel.urdf`(관절 이름 `gripper` 유지, 서보 ID 6 그대로).
- 진행: 인터페이스 + 단동식 이사(동작 변경 0)는 결정과 무관해 먼저 한다. 병렬 구현·설정 선택·GUI 선택 화면은 결정 뒤.
- 결정 필요 (팔 세션 추천): (a) 병렬의 TCP를 "두 턱 중앙"으로(추천, 보정이 필요 없어짐) (b) 병렬 URDF·메쉬를 D1(B)와 함께 만들기(추천) (c) 접촉 감지·조임 토크 기준은 그리퍼마다 따로 실측 (d) 단동식이 기본, 병렬은 설정으로만 켬(추천). 하드웨어 몫: 폭 게이지 5점으로 폭↔서보값 표, 캘리브레이션 범위.
- 초음파 영향: 초음파 코드는 그리퍼 종류를 모르고 손목 센서→집게 끝 오프셋(`sensor_to_tip`)과 `check.tool_reach_mm`만 쓴다. 집게 끝 위치가 바뀌면 그 설정값만 바꾸면 된다(초음파 세션이 코드로 확인, 실제로 돌려 보지는 않음).

### D4. 레이더 서보를 OpenRB-150 + XL330-M288-T로 (5-2, 초음파 세션 조사·설계 2026-11-07)
- **전원 위험(꼭 확인)**: XL330 입력은 3.7~6.0V(권장 5V). OpenRB-150의 DXL 포트 전원은 VIN에서 나오고 VIN은 12.6V까지 받으므로, 팔 어댑터 12V를 OpenRB VIN에 그대로 넣으면 XL330이 망가질 수 있다. 5V(3A급 강압 모듈 또는 USB 5V)로 먹인다. 배선 담당(송민준)에게 "VIN에 12V가 들어가는지" 먼저 확인. DXL 포트 전원은 OpenRB 펌웨어가 켜야 한다(핀 31). Serial2 UART 단자는 핀 헤더 납땜이 필요할 수 있다.
- 연결 방식 후보: A UART 브리지(ESP32 ─UART 3.3V, TX/RX 교차, GND 공통─ OpenRB Serial2, OpenRB가 Dynamixel2Arduino로 XL330 제어하고 실제 각도를 돌려줌) 82점 / B OpenRB를 USB-TTL 중계로 쓰고 ESP32가 DXL 프로토콜 직접 58 / C ESP32 직접 구동(반이중 회로) 45 / D OpenRB를 PC USB에 따로 50. 추천 A. ESP32↔OpenRB 연결 사례는 공식 문서에 없어 A는 "공식 부품의 일반 UART 연결"로 설계한 것.
- 효과: XL330은 12비트(0.088°) 엔코더로 측정 순간의 실제 각도를 읽어 샘플에 실을 수 있어, 지금 설계의 "서보 지연 ±3°"가 사실상 줄어든다(실측 B4로 확인). 통신 규약은 하위 호환 확장(`hello.servo = pwm|dxl`, 레이더 샘플에 선택 필드 `ang`). 손목 센서·PC 인식 코드는 거의 그대로.
- 진행: 결정과 무관한 준비(가상 서보 모델 pwm/dxl 옵션, `ang` 처리, 벤치마크 `--servo`, ESP32 펌웨어 `servoWrite` 분리)는 초음파 세션이 먼저 한다. 본공사(OpenRB 펌웨어 `firmware/openrb_servo/`, 설정 `servo.driver`)는 결정 뒤.
- 준비 결과(#144 합침, 60씨앗, pwm→dxl 성공률 %): demo 낙관 97→99, 비관 84→85 / single 낙관 91→93, 비관 78→78. 모두 ±5%p 안이라 "dxl이 성공률을 올린다"는 근거는 없다(dxl 지연 0.3°는 가정값). 교체 이유는 성공률이 아니라 해상도·실제 각도 피드백·떨림 확인(B4 실측)으로 본다. 레이더 중앙 오차 약 30mm는 서보가 아니라 빔 폭 때문.
- 결정 필요: (a) 연결 방식 A 채택 (b) XL330 전원 5V 확정(12V를 OpenRB VIN에 넣지 않음) (c) 구매 목록(ultrasonic-spec 3.5의 MG90S)을 OpenRB·XL330·5V 전원으로 갱신, 결정 0035(MG90S)를 대체하는 새 결정 기록. 출처: emanual.robotis.com(XL330-M288, OpenRB-150), github.com/ROBOTIS-GIT/Dynamixel2Arduino.

### D5. 레포 정리 (3단계, 총괄 2026-11-07)
- (a) 합쳐진 원격 브랜치 59개 지우기(`git branch -r --merged origin/main`, main 제외). 내용은 main에 모두 있다. 지운 뒤에도 PR 페이지에서 되살릴 수 있다.
- (b) `dev/gui-handoff-final` 지우기: 합쳐지지 않았지만 2026-11-06 인수인계 내용이고, 지금은 `project-status.md`·`work-queue.md`가 대신한다.
- (c) #68 기획 요청함 닫기: 기획 질문은 이제 프롬프트로 하고(`session-workflow.md` 5절) 요청함은 쓰지 않는다.
- 유지: #41(자동 캘리브레이션 반복 합격·`--verify-zero`) — AC4 실물 합격 뒤 다시 본다(runbook 2-4·2-5).
- 추천: (a)(b)(c) 모두. 지우기라 사용자 승인 뒤에만 한다.
