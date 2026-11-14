상태: 초안 (총괄 → 기획, 2026-11-15 밤 자율 작업 보고. main 78a4dbe)

# 11-15 밤 보고와 기획 요청

## 1. 합친 것
- #363·#366 기획 브랜치(새 PC 절차, mujoco-spec drift 반영), #365 자율 진행·사후 보고 체계(session-workflow 7절, CLAUDE.md)·REQ-064 대장, #362 us-dev-status
- #364 REQ-064: 병렬이면 `tip_vector`·`tool_position`·`follow_path` 기준이 두 턱 끝 가운데. GUI 현재 좌표도 같은 함수라 같이 바뀜
- #368 팔 시험 21개 `slow` 표시(기준 5초), #369 GUI 시험 고침(nicegui 3.18 알림, 화면 기준 사진), `measure` 표시 등록

## 2. 새 PC 전체 시험(fcdcb0f, 1:17:21)
1218 passed, 6 failed, 12 errors. GUI 5개·화면 12개는 #369로 고침. 남은 실패는 `tests/test_task_check.py::test_sim_three_objects_end_to_end`(아래 3-1).

## 3. [기획 요청]

### 3-1. CHECK 손목 yaw를 후보 반경별로(ultrasonic-spec 6절 "yaw 90 고정" 변경) — 사용자 승인 필요
- 초음파 세션 분석: 센서가 손끝에서 66.5mm 떨어져(#359) yaw 90 고정이면 가깝거나 먼 물체에서 손끝 경로가 도달 범위(60~255)를 벗어난다. 성능 저하의 주원인은 센서 높이가 아니라 이것이다.
- 안: `check.yaw_pivot_mm`(150). 반경 r에서 센서가 손끝 바깥으로 나가는 양 c = clamp(r − 150, ±66.5)가 되게 yaw를 정한다.
- 벤치마크(초음파 세션 보고, 총괄 미확인): single 낙관 73→95%, 비관 62→88%. 병렬 + 실제 기구학 55mm 19→63%, trim_start까지 75%. CHECK 높이 80mm는 0~20%라 55mm 유지.
- 선택지: (a) 옵션을 기본으로 켜고 6절을 고친다 (b) 옵션만 두고 끈 채 둔다(지금 성능 62%·병렬 19%, 종단 시험 실패 유지). 옵션 코드(기본 끔) PR과 켜는 PR을 나눠 올리게 했다. 수치 표는 그 PR 본문·us-dev-status.

### 3-2. 13.7 (a) topedge를 병렬 손 기준으로 다시 정하기
- MuJoCo 세션: 병렬 손은 yaw 135°로 면에 맞춰 내려가 topedge가 재현되지 않는다(ok·held). (a)를 병렬용으로 다시 정하거나 단동식 비교 옵션 전용으로 표시해야 한다.

### 3-3. 시나리오가 단동식 기준이다
- blocker 벽(300,200, 높이 120)은 병렬 손의 home → radar_clear 이동 경로에 걸린다(병렬 손 최저점이 home에서 작업대 위 약 86mm, 벽 발자국 안). 그래서 병렬용 `blocker_parallel.yaml`(벽 300,140)을 따로 만들었다(#370). 13.10에 "시나리오는 그리퍼별"을 적을지.
- 실물에도 해당: 높이 120mm 안팎 물체가 x 270~335·y ≥ 187에 있으면 radar_clear로 가는 길에 걸린다(시뮬레이션 충돌 형상 기준).

### 3-4. M5 세부 설계용 메모(MuJoCo 세션, "시험장 기준 설정"을 지금 시험이 덮는 방법)
1. cfg: `load_robot("config/robot.yaml", local_path=<tmp>/local.yaml)`에 `gripper: parallel` 한 줄만 쓴 파일로 덮는다(없는 경로를 주면 local 무시). 팔 파일은 `config/arms`에 하나뿐이라 자동으로 골라진다(둘이 되면 지정 필요).
2. 장면: tmp의 `scene.yaml`에 `wrist_hcsr04`·`wrist_bracket`(attach `gripper_link`)을 절대경로 file로 적어 `load_scene`. 상대경로는 yaml의 부모의 부모 기준이라 tmp에서는 절대경로가 필요하다. `make_sim_driver(cfg, WS, scene, scenario=...)`로 넘기고 `ArmService`에는 scene 인자가 없다 → coal 장면 충돌 검사가 켜졌는지는 확인 안 함(M5에서 확인).
3. 손목 부품은 한 쌍으로 와야 한다(하나만 주면 ValueError). 단동식에서는 `SIM-PART-SKIPPED`, `wrist_sonar` site는 병렬일 때만.
4. 시험 하나가 연결부터 새로 해야 해(물체 초기화) 실시간 20~27초씩 걸린다. 빠르게 하려면 M5b.
5. 병렬 carry 58mm의 미끄러짐이 부하에 따라 2.0~9.6mm로 흔들린다(총괄 4회 중 3회 4.0mm 초과). 실시간 지터인지 물리 편차인지 MuJoCo 세션이 확인 중. kp·힘 잠정값과 함께 M5 판정 기준에 영향.

### 3-5. 환경 고정
- nicegui·playwright가 `environment.yml`에 버전 고정이 안 돼 새 PC(nicegui 3.18.0)에서 GUI 시험 5개가 깨졌다. 버전을 고정할지(팔 세션 영역 파일, tooling-setup 반영).

## 4. 그 밖
- code-review(#364) 남은 지적: `Kinematics.link_points`의 손끝 점은 병렬에서도 한쪽 턱 끝이다(레이더 가림 계산, 최대 약 42mm 차이). 병렬 분기를 `targets.py`가 그리퍼 이름 문자열로 한다. 급하지 않아 REQ로 내지 않았다.
- 병렬 손은 손끝 55mm에서 yaw 90 고정 CHECK 경로가 거의 안 닿는다(3-1로 해결).
