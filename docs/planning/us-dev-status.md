# 초음파 세션 현황

상태: 초안 · 2026-11-05

## 작업 진행
| # | 작업 | 상태 |
|---|---|---|
| U0 | 실측용 펌웨어 `firmware/esp32_scanner`, `perception/ultrasonic/bench.py` | 코드 작성, 펌웨어 컴파일·실물 미확인(Arduino CLI 없음). 사용자와 B1~B3 대기 |
| U2 | model, fake_esp32, radar, check, source, link, config | 코드·시험 완료(PR 대기). 가상 ESP32로 물체 3개 끝까지(팔 follow_path 비용 반영): 성공은 정확 3.0/3, 빠른 2.3/3이지만 시간은 정확 54.7초(기준 45), 빠른 T9 켬 73.9초·끔 47.4초(기준 30)로 기준 초과 — 기획 요청 대기 |
| U5 | 펌웨어 7절 전부 | 대기(서보 11/12 예정) |
| U6·U7·U9 | 총괄이 보냄 | - |

## 규약 추가(실측용)
- 메아리가 없으면 ECHO가 약 38ms HIGH로 남을 수 있어 B1·B2는 `--gap 60`으로 잰다. 펌웨어는 ECHO가 LOW가 될 때까지 기다린 뒤 트리거하고, gap은 트리거 시각부터 센다.
- `sync`는 실측용 펌웨어에 없다(U5에서 ns 숫자를 문자열로 돌려주는 방식으로 넣는다). U5 메모: emit의 `us`는 측정이 끝난 뒤 시각이라, 팔 위치와 맞출 때 기준 시점을 정해 2ms 목표에 넣는다.
- `{"cmd":"gap","ms":N}`: 측정 간격 변경 → `{"t":"gap","ms":N}`. 설계서 7절에 없어 정식 규약에 넣을지 기획 확인 필요.
- `once`의 `k`는 `"r"`, `"a"`, `"ra"`(번갈아 n회씩), `max_mm` 선택. 끝나면 `{"t":"done"}`.

## 실측 절차 (U0, 서보 없음)
실물 시험 전체 순서는 `measurements.md`의 "실물 시험 순서". 배선: 레이더 TRIG 25/ECHO 26, 팔 TRIG 32/ECHO 33, ECHO는 1k/2k 분압(5V→3.3V), 센서 전원 ESP32 5V. 펌웨어는 Arduino IDE 2에서 ESP32 보드로 올린다(업로드 중 시리얼 모니터는 닫는다).
- B1: 센서를 110mm 높이에 아래로 고정, 40mm 상자를 옆에서 5mm씩 밀어 넣는다. 자리마다 `bench --k a --n 20 --gap 60 --label B1-x<mm>`, 거리가 바뀌는 수평 거리를 기록(앞뒤·좌우).
- B2: 센서 20mm 높이 수평(`--k r`), 200mm 앞 상자를 5°씩 돌리며 자리마다 `--gap 60 --n 10 --label B2-<deg>`, 응답 없음이 늘어나는 각도를 기록.
- B3: 고정 물체 앞에 두 센서를 서로 비스듬히 두고 `bench --k ra --n 200 --gaps 20,25,29,33,40 --label B3`. 출력의 간격별 엉뚱한 값 비율이 낮아지는 최소 간격이 `ping_gap_ms`.
- 결과 CSV는 `runs/bench/`(git 제외). 값만 총괄에게 넘겨 `measurements.md`에 적는다.

## 측정
(없음)

## 보낸 요청
(없음)

## U2 구현 메모
- 시험은 가상 시계(`SimClock`)로 돈다. `FakeESP32.write/readlines`가 `SerialTransport`와 같은 모양이라 `Link`가 구분하지 못한다. 시험용 가상 팔은 `tests/perception/simlib.py`.
- 팔 확인 속도 100mm/s(기획 답). 빠른 모드는 `line_mm: 130`을 쓴다(U9 조정값).
- **시간 모델(팔 세션 측정)**: 가상 팔은 호출마다 정지에서 출발·정지로 끝, 가속 120mm/s², 호출 사이 0.25초. (모델: 180mm 직선 2.64초, 36mm 1.10초. 측정: 3.63초, 1.30초라 모델이 가볍다. `motion_scale=1.25`면 정확 65초.) 이 모델에서 물체 3개(10회 평균, 성공/인식 초): 정확 line180 3.0/54.7 · line130 2.5/47.7 · line100 1.8/43.4, 빠른 T9 끔 line180 3.0/54.3 · line130 2.3/47.4 · line100 1.7/43.3, 빠른 T9 켬(36mm 조각 45번 호출) line130 2.3/73.9. 가속 240mm/s²이면 빠른 T9 끔 line130 3.0/41.1, 정확 line180 2.8/48.4. 기준은 45·30초라 이 모델에서는 둘 다 못 맞춘다. 시간 시험은 xfail로 두고 성공만 확인한다.
- 원(R)은 follow_path 한 번(연속 37점 경로)이고, 정확 모드는 물체당 follow_path 5번(L1·L2·R·L3·L4)이다.
- 원 방향 추정은 호 길이 칸마다 한 번만 센다(팔이 호출 시작·끝에서 느려 같은 자리를 여러 번 재면 방향 평균이 쏠려서).
- **REQ-023 임시**: `PathSegment`, `PoseSample(t_ns, tool_mm, sensor_mm)`, `CheckProbe`, `CheckableSource`는 `model.py`에 임시 정의. `follow_path`는 못 가는 경로면 움직이지 않고 빈 목록을 돌려준다(이유는 `arm.last_path`). 8.2절 루프 그대로 `probe.feed([])`가 불리고, 그 구간은 데이터 없음으로 보고 다음 경로로 가며(신뢰도 -0.2, `probe.info["unreachable"]`), 전부 못 가면 `probe.status == "unreachable"`, 결과 None이고 후보는 지우지 않는다. 물체 없음은 `status == "empty"`이고 후보가 삭제된다(같은 자리는 다시 안 나온다).
- 시뮬레이션 도달(반경 100~240mm, 각도 25~155°)에서는 ±90mm 직선이 반경 방향이면 항상 일부가 범위를 벗어난다(폭 140mm < 직선 180mm). 그래서 반경 방향 선(L2 등)은 대부분 거부된다. `line_mm`·`ring_r_mm`은 설정값이라 기획 답에 따라 바꾼다.
- `check.py`의 `sensor_to_tip` 한 곳에서만 센서 선 → 손끝 점 변환(yaw로 돌린 오프셋). `check.yaw_deg` 설정(기본 0, 이 각도일 때의 `arm_sensor_offset_mm`).
- 빠른 모드의 T9(일찍 멈춤)는 선을 36mm 조각으로 나눠 `follow_path`를 여러 번 부른다. 조각 사이에 팔이 멈추면 시간이 늘 수 있다(가상 팔은 끊김 없이 이어진다고 가정). 팔 세션이 이어 부르는 구간을 부드럽게 이을 수 있는지 확인이 필요하다. **지금 보고한 시간(정확 38.5초, 빠른 29.7초)에는 구간 사이 멈춤 시간이 들어 있지 않다**(팔 세션이 REQ-023을 끝내면 받아 반영).
- 정지·E-STOP으로 `follow_path`가 빈 목록을 돌려주는 경우는 작업 실행(REQ-025)이 `arm.last_path`를 먼저 보고 끊는다는 가정이다(probe까지 오지 않는다). 시험에는 이 경우가 없다.
- 모든 값은 가상 ESP32 + 가상 팔이다. 도달 거부와 멈춤 시간은 실제 `follow_path`로 확인하지 않았다.
- 배출한 물체가 다음 스캔에 후보로 잡힐 수 있다. `UltrasonicSource(ignore_zones=[(x, y, 반경)])`로 뺀다(작업 실행이 배출 구역을 넘긴다).
- 기준값 파일 `config/ultrasonic_baseline.json`은 git 제외여야 한다(`.gitignore`는 내 영역 밖이라 총괄에 요청).

## 이벤트 내용 (Studio.bus, GUI는 REQ-024에서 사용)
- `us.sweep`: `{"pass": 몇 번째, "dir": 1|-1, "t_ns": PC 시계, "angles": [도], "d": [mm 또는 None]}` 한 방향 회전이 끝날 때마다.
- `us.candidates`: `{"objects": [DetectedObject를 dict로(angle_deg=None, confidence<=0.3)], "arcs": [{"first","last","center"(서보 각도),"d"}], "masked": [팔 때문에 지운 각도]}` scan()마다.
- `us.check_path`: `{"id": 후보 id, "stage": "L1"|"L2"|"R"|"L3"|"L4"|"GRID", "points_mm": 손끝 경로, "tool_z_mm", "points": [(x, y, h)] 센서 위치와 물체 높이(h = 센서 높이 - 거리)}` 구간이 끝날 때마다.
- `us.check_result`: `{"id", "object": DetectedObject dict 또는 None, "info": {"R", "h_mm", "stages", "check_s", "unreachable"?}, "status": "ok"|"empty"|"unreachable"}`.
