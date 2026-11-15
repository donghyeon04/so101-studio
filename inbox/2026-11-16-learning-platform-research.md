상태: 초안 (총괄 → 기획, 2026-11-16. 사용자 아이디어 "SO-101 Studio를 학습 플랫폼으로, 12/4 전에 한 번에"의 레퍼런스 조사와 이 PC 가능성 확인. 조사 원문은 커밋하지 않는 규칙이라 inbox에만 둔다)

# 학습 플랫폼 확장: 레퍼런스 조사와 가능성 확인

## 문제 정의
1. GUI에서 한 화면에 여러 시점(MuJoCo 탑뷰·손목 카메라, 초음파 빔·값, 실물 카메라)
2. 키보드·게임패드 조작으로 sim·실물 에피소드 수집(LeRobot 데이터셋)
3. 모방학습(ACT)·VLA 학습과 sim·실물 실행
4. 강화학습
5. 우리 구조(`ArmService`·`MuJoCoDriver`·시험장 결정 0062)에 붙이고, 12/4 전(11/30 기능 동결)에 들어가는지

## 이 PC에서 직접 확인한 것(총괄, main f72a161)
- **MuJoCo 화면 밖 렌더링이 된다**: 우리 모델(`mujoco_model.load_model`)을 `mujoco.Renderer` 320×240으로 찍으면 **1.1ms/장**. GUI 시점 패널은 성능 문제가 없다.
- 다만 우리 모델에는 **카메라가 0개**이고 작업대·바닥 모양이 없어 회색 배경에 팔만 보인다. 탑뷰·손목 카메라(손목은 `gripper_frame` site에 붙임)와 작업대 판(하드웨어 반영 계획의 새 작업대)을 모델에 더해야 한다.
- LeRobot 0.6.1의 `teleoperators.keyboard`·`teleoperators.gamepad`, `gymnasium`, `torch`는 설치돼 있다.
- **없는 것**: `lerobot[dataset]` 추가 패키지(`datasets`. `LeRobotDataset`을 불러오면 ImportError), ACT 모듈 확인 전(같은 extra 문제로 보임).
- **GPU**: RTX 5070 Ti 16GB가 있는데 **torch가 CPU 빌드**다(`cuda.is_available()` False). ACT·VLA 학습에는 CUDA torch 설치가 필요하다(5070 Ti는 Blackwell이라 CUDA 12.8 이상 빌드). 설치는 사용자 승인 일이다. numpy를 pip로 다시 깔지 않는 규칙과 충돌하지 않게 해야 한다.

## 평가 기준
기본표 그대로(하드웨어 적합도 5, 문제 일치도 5, 유지보수 3, LeRobot 호환 3, 문서 2, 라이선스 1, 커뮤니티 1). **모든 후보는 검색 요약(2차)만 봤고 원문을 읽지 않았다** → 점수는 잠정, 기획이 상위 후보 원문을 확인해야 한다.

## 후보 14개 (점수 잠정, 원문 미확인)
| # | 이름 | 무엇 | 하드웨어 | 문제 | 유지 | LeRobot | 문서 | 라이선스 | 커뮤니티 | 총점 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | so101-nexus (github johnsutor/so101-nexus, PyPI so101-nexus-mujoco 0.3.5) | MuJoCo SO-101(menagerie 같은 원본), 손목·위 카메라, Gymnasium API, LeRobot 데이터셋 출력, 과제 6개, 선택 GPU(Warp) | 5 | 5 | 4 | 4 | 3 | 5 | 3 | 85 |
| 2 | LeRobot gym_hil / HIL-SERL(공식 문서) | MuJoCo, 키보드·게임패드, LeRobotDataset 기록, 사람 개입 RL | 2(Franka) | 4 | 5 | 5 | 4 | 5 | 4 | 74 |
| 3 | aviadarn/so101-lerobot | LeRobot 0.6 + ACT, **스크립트 전문가로 MuJoCo 시연 자동 생성**, 300에피소드, 처음 보는 배치 42~68% | 5 | 5 | 3 | 5 | 3 | ? | 2 | 78 |
| 4 | sonson0510/so101-teleop-act-final | GUI 조작 → 25Hz RGB·관절 기록 → LeRobot 변환 → ACT → MuJoCo 평가 | 5 | 5 | 3 | 4 | 3 | ? | 2 | 74 |
| 5 | jangjunseo05/so101-il-dev-toolkit | sim·실물 조작 수집, 데이터셋 QA, ACT, MuJoCo 추론, Streamlit 대시보드 | 5 | 5 | 3 | 4 | 3 | ? | 2 | 74 |
| 6 | fport/robots-101 | 실습 과정, MuJoCo 장면, LeRobot 0.6.1 데이터·ACT | 5 | 4 | 3 | 5 | 4 | ? | 2 | 73 |
| 7 | fport/so101-flywheel | MuJoCo + LeRobot 조작·기록 데이터 플라이휠 | 5 | 4 | 3 | 4 | 3 | ? | 2 | 69 |
| 8 | EE5108-DigitalTwins/lerobot_mujoco_sim | 집어 상자에 넣기 한 과제, 수동 조작 수집, 학습은 Colab | 5 | 4 | 2 | 4 | 3 | ? | 1 | 65 |
| 9 | dev2703/real2sim | 실물 SO-101 조작 수집 → MuJoCo에서 ACT 평가 | 5 | 3 | 2 | 4 | 2 | ? | 1 | 58 |
| 10 | lucagianforcaro/so101-sim-to-real | 설정·MuJoCo·sim2real | 5 | 3 | 2 | 4 | 3 | ? | 1 | 60 |
| 11 | ManiSkill3 SO100 + lerobot-sim2real(Stone Tao) | GPU 병렬 시각 RL, 큐브 집기 실물 무학습 전이(2차 요약: 91.6%) | 3(SO100, SAPIEN 엔진) | 4 | 4 | 3 | 4 | 5 | 4 | 68 |
| 12 | Squint | SO-101 시각 RL sim2real(lerobot-sim2real 후속이라는 2차 요약) | 4 | 4 | 3 | 3 | 2 | ? | 2 | 61 |
| 13 | Isaac Lab SO-101 | 가장 큰 SO-101 sim 생태계(2차 요약), 다른 엔진·GPU 필수 | 4 | 3 | 4 | 2 | 3 | 4 | 4 | 63 |
| 14 | pick-101 | MuJoCo 큐브 들기 SAC(MIT) | 4 | 3 | 2 | 2 | 2 | 5 | 1 | 50 |
(? = 라이선스 미확인, 2점으로 계산)

## 상위 비교와 충돌
- **1(so101-nexus)**: 같은 menagerie 모델·카메라·gym·LeRobot 출력을 한 번에 가진 유일한 후보. 다만 우리 `ArmService`(안전 검사·집기 알고리즘·벽시계 제어)를 쓰지 않는 독립 환경이다.
- **3(aviadarn)**: "스크립트 전문가로 시연 자동 생성"이 우리에게 가장 맞는 아이디어다. 우리는 이미 집기·놓기 알고리즘(시험장 실행기 A)이 있어 사람이 조작하지 않고도 sim 시연 수백 개를 만들 수 있다.
- **2(gym_hil)**: 공식이고 키보드·게임패드·기록 흐름이 표준이지만 팔이 Franka다. 조작·기록 흐름만 가져온다.
- 충돌: RL은 빠르게 많이 돌려야 하는데(11·13은 GPU 병렬) 우리 제어는 벽시계라 실시간보다 빠르지 않다(M5b 시계 주입이 전제). 독립 gym 환경(1)으로 RL을 하면 빠르지만 우리 안전 검사·알고리즘과 갈라진다.

## 상황 맞춤 결정안 (승인 대기)
| 결정 | 이 상황에 맞는 이유 | 근거 |
|---|---|---|
| A. 시점 패널: 우리 모델에 탑뷰·손목 카메라와 작업대 판을 더하고 `mujoco.Renderer`로 GUI 패널에 그린다. 실물은 같은 틀에 USB/ESP32 카메라 | 이 PC에서 1.1ms/장 확인. 하드웨어 반영(새 작업대)과 같은 모델 작업 | 1, 직접 확인 |
| B. 수집: LeRobot 0.6.1의 `LeRobotDataset`(+ `lerobot[dataset]` 설치)과 `teleoperators.keyboard/gamepad`를 `ArmService` 조그·직교 조그에 연결. **우리 알고리즘을 스크립트 전문가로** 써서 sim 시연 자동 생성을 1순위로 | 처음부터 만들지 않고 공식 형식. 사람 조작 없이 수백 에피소드 | 2, 3, 4 |
| C. 학습: LeRobot ACT를 이 PC(RTX 5070 Ti)에서. CUDA torch 설치 필요(사용자 승인) | 공식 경로, GPU 있음 | 3, 4, 6 |
| D. 실행: 학습한 정책을 `MuJoCoDriver`와 실물에 같은 `ArmService` 입구로(새 "정책 실행" 동작). 안전 검사 그대로 | 안전 규칙을 지키며 sim·실물 같은 길 | 4, 5 |
| E. RL: 우리 모델·장면을 그대로 쓰는 **독립 gymnasium 환경**(so101-nexus 방식 참고, 설치해 쓰거나 구조만 가져옴)으로 먼저. M5b 시계 주입은 병행 | 벽시계 제어로는 RL이 느림. 같은 모델·장면이면 결과를 우리 시험장과 비교 가능 | 1, 2, 11 |
| F. 범위(12/4): A·B·C·D는 2주 안에 들어갈 수 있다. E는 "환경과 짧은 학습 데모"까지로 잡고, 성공률 목표는 두지 않는 것을 추천 | 11/30 동결, 실물·초음파 통합(I1)이 시연 필수 | - |

점수 1위(1)를 그대로 쓰지 않는 이유: 우리 앱의 핵심인 안전 검사·집기 알고리즘·GUI가 `ArmService` 위에 있어, 수집·실행은 그 위에 붙이고(B·D), 빠른 반복이 필요한 RL만 독립 환경으로 뺀다(E).

## 버린 후보
- 13 Isaac Lab, 11 ManiSkill: 엔진이 달라 우리 MuJoCo 모델·시험장과 두 벌이 된다. RL 결과 비교 참고로만.
- 14 pick-101: 과제가 너무 작다.

## 사용자 승인이 필요한 것
1. 위 결정안 A~F 방향
2. 설치: `lerobot[dataset]`, CUDA torch(RTX 5070 Ti용), 필요하면 so101-nexus. numpy 2.2.6 conda 설치본 유지가 조건
3. 범위: RL을 12/4 전에 어디까지(추천: 환경 + 짧은 학습 데모)
