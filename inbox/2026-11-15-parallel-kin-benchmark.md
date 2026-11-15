상태: 초안 (총괄 → 기획, 2026-11-15. 병렬 손 + 실제 기구학 벤치마크를 총괄이 다시 돌린 값, main da8b586)

# 병렬 손 + 실제 기구학 벤치마크 재실행 (ultrasonic-spec 1절 "표의 한계" 갱신용)

조건: 새 PC, 씨앗 20, `--reach kin --gripper parallel --trim-start`, `check.yaw_pivot_mm` 150, 센서 높이 154.5mm, `check.tool_reach_mm` 60~255(바꾸지 않음).
커밋: 먼저 main 7596d8e에서 돌렸고(결과 `runs/bench/20261115-1121~-7596d8e.json`), #384(첫 구간 1.5mm 미만이면 둘째 점을 뺌 + demo80)를 main 위에 합친 5b3ac0c에서 다시 돌려 **demo·single이 같은 값**임을 확인했다. 단동식 기본 값(single 95/88, demo 98/83)도 #384 뒤 그대로다.

| 시나리오 | 명령(공통: `python -m so101_studio.perception.ultrasonic.benchmark … --seeds 20`) | 성공률 낙관 / 비관 | 인식 평균(초) | 90% | 최대 |
|---|---|---|---|---|---|
| demo, 병렬 kin | `--scenario demo --reach kin --gripper parallel --trim-start` | 78% / 62% | 70 / 71 | 80 / 88 | 116 / 112 |
| single, 병렬 kin | `--scenario single --reach kin --gripper parallel --trim-start` | 79% / 75% | 27 / 28 | 31 / 34 | 63 / 58 |
| demo80, 병렬 kin(5b3ac0c) | `--scenario demo80 --reach kin --gripper parallel --trim-start` | 68% / 63% | 66 / 69 | 78 / 78 | 88 / 92 |
| demo, 병렬 kin, 끔 | `… --set yaw_pivot_mm=0` | 43% / 28% | 44 / 47 | 63 / 69 | 67 / 77 |
| single, 병렬 kin, 끔(7596d8e) | `… --set yaw_pivot_mm=0` | 46% / 36% | 24 / 25 | 30 / 33 | 300 / 300 |
| demo80, 단동식 반경 모델(5b3ac0c) | `--scenario demo80` | 98% / 85% | 83 / 80 | 94 / 84 | 125 / 127 |

- single 병렬 kin 반경별 성공률(낙관): 100:60 120:80 140:90 160:70 180:85 200:100 220:80 240:70. 비관: 100:60 120:75 140:80 160:65 180:80 200:90 220:80 240:70.
- 확인 위치 오차 중앙·90%(병렬 kin): demo 2.5·15.7 / 4.2·18.7mm, single 2.4·11.9 / 2.5·17.8mm. 방향 정답률: demo 79% / 66%, single 88% / 82%.
- 초음파 세션 보고 값(demo 78/62, single 79/75, 끔 demo 43/28)과 같다.
- "single 끔"의 최대 300초는 씨앗 시간 상한(`--seed-timeout` 300)에 걸린 것이다. 원인은 아래 REQ-066 버그로 본다(#384 전 도구).
- demo80이 병렬 kin에서 demo보다 낮게(낙관 68 vs 78) 나온 것은 확인이 필요하다(씨앗 20의 잡음인지, 모서리 80mm 조건이 물체를 도달 띠 가장자리로 미는지). 초음파 세션에 묻는다.

## 같이 알릴 것
- **REQ-066(총괄 → 팔, 11-15, 버그)**: `arm/trajectory.py`의 `fillet_path` + `LookaheadShape`가 경로 첫 구간 ≤ 1.0mm에서 `duration_s`를 1e12초대로 낸다(총괄 재현: 첫 구간 1.2mm 5.9초, 1.0mm 2.0e12초). 앞서 "placo가 멈춘다"고 한 것의 진짜 원인이다. 실제 팔의 `follow_path`(CHECK)가 같은 함수를 써서, 손끝이 경로 첫 점에서 1mm 안쪽이면 팔이 선 채로 호출이 끝나지 않을 수 있다. 실제 실행 경로에서 나는지는 팔 세션이 확인 중. 설계서 변경은 필요 없다(버그 수정 + 계획 시간이 비정상이면 움직이기 전에 실패).
- 벤치마크의 60mm 후보 거르기(기획 지시 3-a)는 REQ-065 PR이 합쳐진 뒤 초음파 세션이 policy 함수를 그대로 불러 맞춘다. 그 뒤 값이 달라지면 다시 올린다.
