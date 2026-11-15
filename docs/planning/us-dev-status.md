# 초음파 세션 현황

상태: 초안 · 2026-11-14 (코드·문서를 main fcdcb0f에 맞춰 다시 씀, #359 반영)

## 지금 상태
**코드로 할 일은 끝났고, 남은 일은 전부 실물이다.** 부품(H1)이 11/16 도착 예정이고, OpenRB VIN 12V 여부(H2)를 확인하기 전에는 레이더 배선을 안내하지 않는다(`hardware.md` "지금 필요한 하드웨어"). 순서는 B1~B3 → B4 → R1 → A0·A1 → I1이고, 실측 결과는 `measurements.md`에 기록한다. 마감 12/4.

| # | 상태 |
|---|---|
| 인식 코드(model, fake_esp32, radar, check, source, link, config, benchmark) | 합쳐짐 |
| 작업 실행 연동(REQ-025, `run --source ultrasonic`), GUI 화면(REQ-024) | 합쳐짐 |
| 레이더 서보 OpenRB-150 + XL330(REQ-039, 결정 0043): `firmware/openrb_servo/`, `servo.driver` | 합쳐짐. **컴파일만 했고 업로드·실물 안 함** |
| `link.py` 감사 수정(REQ-044) | 합쳐짐 |
| 펌웨어 `esp32_scanner` | 컴파일만(arduino-cli 1.5.2, esp32:esp32 3.3.12). 업로드·실물 안 함 |
| B1~B4, R1, A0, A1, I1 | 대기(부품·H2) |
| 레이더 스테이션·손목 브래킷(H3·H4) | 하드웨어 담당 설계 중 |
| 센서 오프셋(#359, 합쳐짐) + CHECK yaw 반경별(결정 0065) | `arm_sensor_offset_mm` {0.6, 66.5, 99.5}, `ring_r_mm` 60. 11-14 성능 저하의 주원인은 센서 높이(154.5mm)가 아니라 **센서 옆 오프셋 66.5mm + yaw 90 고정**이었다. `check.yaw_pivot_mm` 150으로 후보 반경별 yaw를 정해 회복(아래 절). 실물은 A0로 확인 |

이 세션은 ESP32·서보 포트를 한 번도 연 적이 없다. 포트는 사용자가 옆에 있을 때만 열고, 열기 전과 반납 뒤에 총괄에게 알린다. COM3(팔)은 쓰지 않는다.

## CHECK yaw 반경별 (결정 0065, 2026-11-15)
**원인 정정**: 옛 오프셋 {0, 30, 55} → #359 오프셋으로 `single` 비관 성공이 78 → 26%로 떨어졌고(ring_r 60으로 62%), 센서 높이는 주원인이 아니다. yaw 90 고정이면 센서를 물체 위로 보내려 손끝이 늘 같은 방향으로 66.5mm 비켜나, 가깝거나 먼 물체에서 손끝 경로가 `check.tool_reach_mm`(60~255)을 벗어나 잘린다.

**식**(`check.pick_yaw`): 후보 반경 r(팔 원점 기준)에서 센서가 손끝보다 바깥으로 나가는 양 c = clamp(r − `yaw_pivot_mm`, ±L), L = 오프셋 크기(66.5mm). 오프셋 방향이 팔→후보 방향에서 acos(c/L)만큼 벗어나게 yaw를 풀고, 두 해 중 `yaw_deg`(90)에 가까운 쪽을 쓴다. 한 후보의 CHECK 동안 한 값이고(`PathSegment.yaw_deg`), 센서가 지나는 선은 그대로다. `yaw_pivot_mm`이 없거나 0이면 yaw 90 고정. 확인 높이는 55mm 그대로.

**pivot 150을 고른 이유**: `single`(씨앗 20, 반경 도달 모델) pivot 130/150/170/190에서 낙관 91/95/94/91%, 비관 82/88/89/82%. 150·170이 비슷하고, 병렬 + 실제 기구학(`--reach kin --gripper parallel`, 호버 포함)에서는 150이 63%, 170이 54%(비관)라 150.

**전·후(커밋 179ec33 위 작업본, 씨앗 20, 성공률 낙관/비관 · 인식 평균 s 낙관/비관)**: 설계서 1절 표에 쓰는 값은 총괄이 아래 명령으로 다시 낸 것만이다(결정 0037).

| 조건 | yaw 90 고정 | `yaw_pivot_mm: 150` |
|---|---|---|
| 단동식(반경 도달 60~255), `single` | 73 / 62% · 48 / 52s | **95 / 88%** · 32 / 32s |
| 단동식, `demo` | 85 / 63% · 143 / 147s | **98 / 83%** · 90 / 84s |
| 단동식, `spread120` | 77 / 58% · 153 / 154s | **95 / 88%** · 89 / 78s |
| 단동식, `spread90` | 68 / 53% · 127 / 158s | **92 / 72%** · 82 / 83s |
| 병렬 + 실제 기구학 + `trim_start`, `single` | 19%(비관, 호버 40mm 포함, trim 없음) | **79 / 75%** (trim 없으면 62 / 63%) |
| 병렬 + 실제 기구학 + `trim_start`, `demo` | 43 / 28% | **78 / 62%** · 70 / 71s |
| 병렬 + 실제 기구학 + `trim_start`, `spread120` | 35 / 22% | **78 / 67%** · 63 / 70s |
| 병렬 + 실제 기구학 + `trim_start`, `spread90` | 30 / 22% | **85 / 55%** · 68 / 69s |

**확인 높이 55mm 유지**: 병렬 기구학에서 80mm는 도달 띠가 100~220mm로 좁아져 0~21%(`single`, pivot 150·호버 0도 18~21%). 55mm는 70~250mm다. 팔 세션이 본 "80mm는 풀림"은 점 하나의 IK이고 경로 전체로는 55mm가 낫다.

**그리퍼별 `check.tool_reach_mm`**(`--reach kin --trim-start`, `single`, 씨앗 20, 성공률 낙관/비관, 커밋 7b0bcce 위): 병렬 손은 60~255 79/75%, **70~250 91/78%**, 80~240 86/76%, 70~220 88/72%. 단동식은 60~255 95/88%, 50~265 94/89%로 같다(씨앗 잡음 ±5%p). 병렬 손의 실제 도달 띠가 70~250이라 설정을 그에 맞추면 낙관 +12%p, 비관은 잡음 안(+3)이다. 제안(설계서 숫자라 기획 몫): 단동식 60~255 그대로, 병렬 70~250으로 그리퍼별 값을 둔다. 지금은 설정에 하나뿐이라(`check.tool_reach_mm`) 나누려면 `config/grippers/parallel.yaml` 겹치기 같은 구조가 필요하다.

**병렬 손 `tool_reach_mm` 60~255 대 70~250, `demo`·`demo80`**(`--reach kin --gripper parallel --trim-start --seed-timeout 180`, 씨앗 60, 커밋 da8b586 위, 성공률 · 인식 평균):

| 시나리오 | 모델 | 60~255(지금) | 70~250 |
|---|---|---|---|
| demo | 낙관 | 83% · 69s | **95%** · 93s |
| demo | 비관 | 67% · 69s | **72%** · 97s |
| demo80 | 낙관 | 74% · 67s | **93%** · 104s |
| demo80 | 비관 | 63% · 68s | **72%** · 91s |

병렬 손은 70~250이면 `demo80` 낙관의 손해(−9%p)가 사라진다(95 → 93). 인식 시간이 늘어나는 것은 60~255가 도달 밖 점을 건너뛰며 일찍 실패로 끝나던 사이클이 이제 끝까지 가서다. 설정은 바꾸지 않았다(그리퍼별 값은 기획이 정한다).

**`trim_start`**(벤치마크 옵션 `--trim-start`, 설계서에 없음): `--reach kin`에서 경로 첫 점이 호버까지 안 닿으면 첫 구간 위를 1mm씩 안쪽으로 가며 닿는 첫 점으로 당긴다(결정 0042 (다)안). 가상 팔 판정에만 있고 CHECK 코드·설정의 동작은 바꾸지 않는다. 실제 팔 `follow_path`가 같은 일을 하는지는 팔 세션 몫이다.

**yaw가 바뀌면 걸리는 것**
- 도달·손목 한계: pivot 150이면 경로 점 1056개(반경 100~240, 방위 25~155°, 선 ±90mm·원 60mm) 가운데 손끝 반경이 60~255 밖인 점이 4개(최대 262mm), IK 실패는 단동식 0개·병렬 11개(고정 yaw 90은 밖 216개, IK 실패 176·241개). 손목 회전은 단동식 −122~97°, 병렬 −125~101°로 한계(±168.8°) 안이다. `tool_reach_mm`은 바꾸지 않는다(밖의 점은 잘린 채 신뢰도만 −0.2). 병렬 손은 실제 띠가 70~250이라 아래 "남은 것"으로 둔다.
- yaw를 돌리는 시간: 가상 팔은 손목 회전 시간을 재지 않는다(손끝 이동 시간만). 후보마다 yaw가 달라 접근 이동에 최대 약 140° 회전이 더해질 수 있고 인식 시간에는 안 들어갔다. A1에서 잰다.
- 집기 직전 yaw와의 관계: 확인이 끝난 뒤 집기는 물체 각도로 yaw를 새로 정하므로(arm 세션 Skills) 확인 yaw와 상관없다. 이어붙이는 회전량만 늘 수 있다.
- 레이더 가림: 레이더 가림 계산(`links_at`, 팔 링크 4점)은 센서 끝점도 같은 yaw로 쓰므로 바뀐 yaw를 그대로 따른다. 가상 벤치마크(위 표)에는 이미 반영됐다.
- 알려진 문제(정정 11-15): 병렬 기구학 `spread90` 비관 시드 7의 멈춤은 placo가 아니다. `trim_start`가 경로 첫 구간을 0.87mm로 줄여, `arm/trajectory.py`의 `fillet_path`+`LookaheadShape`가 첫 구간 ≤ 1.0mm인 경로에서 `duration_s`를 약 1e12초로 계산해 50Hz 틱 루프가 끝나지 않았다(메모리 22GB). 총괄에 최소 재현을 보냈고 팔 세션이 실제 `smooth_l`에서도 나는지 확인한다(팔 영역). 벤치마크는 `trim_start` 뒤 첫 구간이 1.5mm 미만이면 둘째 점을 뺀다. `--reach kin`은 `--seed-timeout`(기본 300초)으로 씨앗마다 끊고, 메모리 상한은 없다.

**`test_sim_three_objects_end_to_end` (2026-11-15, 원인 둘, 아직 실패)**
1. **배치가 너무 촘촘하다**: `SPREAD` (420,190,0)·(330,150,0)은 모서리 간격 약 14mm, (330,250,90)과 35mm다. 센서 높이 154.5mm의 빔(물체 윗면에서 반경 약 15mm + 판정 15mm)이 이웃을 합쳐 본다(확인 폭 125mm, 각도 없음, 집기 실패). 중심 간격 기준으로는 spread90(90 이상)만 만족하고 demo·spread120(120 이상)은 아니다.
2. **DECIDE의 도달 필터가 레이더 후보에 너무 빡빡하다**(`task/policy.py reachable`, 팔 영역): 레이더 후보는 가까운 면 쪽으로 치우쳐(참값 (330,270) → 후보 (334,308), 반경 222 → 260) `pick_reach_mm` 최대 250을 넘어 매번 제외되고 `no_objects`로 끝난다. 확인(CHECK)이 위치를 바로잡기 전에 걸러지는 것이다. 벤치마크는 이 필터를 쓰지 않아 반경 240에서도 성공한다.
- 확인: 배치 [(180,170,0),(330,270,90),(330,150,0)](모서리 간격 50·55mm, 중심 120 이상, 배출 구역 무시 원(480,150,r74) 밖)에서 `policy.reachable`이 확인 안 된 레이더 후보(각도 None·높이 0)에 한해 최대 반경에 60mm 여유를 주면 **3개 모두 집는다**(`picked us-1, us-3, us-5`). 여유 없이는 2개에서 `no_objects`. 원래 배치는 여유를 줘도 3개 실패(1번).
- 재현: `tests/test_task_check.py`의 `sim_studio(tmp_path, spread=[…])`로 위 배치를 넣고 `st.start(blocking=True)`.
- 제안(팔 REQ): `reachable`을 후보·확정에 나눠, 레이더 후보(`angle_deg is None`)는 반경 여유(+60mm, 레이더 오차 90%값)를 두고, 확정된 물체만 `pick_reach_mm` 그대로 쓴다. 시험 배치는 위 [(180,170,0),(330,270,90),(330,150,0)]로.

**이웃 물체를 따로 보는 최소 간격**(벤치마크 `--scenario pair --pair-gap N`, 물체 둘의 모서리 사이 최소 거리 N, 씨앗 40, 반경 120~240, 성공 = 둘 다 15mm·방향 맞게 확정, 둘 중 하나라도 놓치면 그만큼 깎임):

| 모서리 간격 mm | 10 | 20 | 30 | 40 | 50 | 60 | 80 |
|---|---|---|---|---|---|---|---|
| 낙관 성공률 | 40% | 51% | 55% | 65% | 88% | 89% | 96% |
| 비관 성공률 | 46% | 48% | 52% | 56% | 51% | 49% | 86% |

모서리 간격 50mm 이상(낙관)·80mm 이상(비관)이면 따로 보인다. 설계서 1절의 "간격 120mm 이상"은 **중심 간격**이라, 길이 100mm 물체가 긴 축을 마주 보면 모서리 간격이 20mm까지 내려가 이 조건을 못 지킨다(벤치마크의 무작위 방향 배치는 평균적으로 넓다). 모서리 간격 ≥ 60~80mm 조건을 배치 기준에 더하길 제안한다(기획).

**총괄이 다시 돌릴 명령**(설계서 1절 표 값, 씨앗 20; 커밋 SHA는 JSON에 남는다):
```
python -m so101_studio.perception.ultrasonic.benchmark --scenario single --seeds 20
python -m so101_studio.perception.ultrasonic.benchmark --scenario demo --seeds 20
python -m so101_studio.perception.ultrasonic.benchmark --scenario spread120 --seeds 20
python -m so101_studio.perception.ultrasonic.benchmark --scenario spread90 --seeds 20
```
끈 값 비교는 `--set yaw_pivot_mm=0`. 병렬 손 + 기구학은 `--reach kin --gripper parallel --trim-start`(느리고 메모리를 쓴다). 설정 덮어쓰기 `--set 키=값`(check.<키>), 그리퍼 `--gripper single_jaw|parallel`은 이번에 추가했다.

## 알려진 함정
0. 벤치마크 시나리오: `spread90`·`spread120`·`demo`·`single`(`--scenario all`), `pair`(모서리 간격 `--pair-gap`), `demo80`(demo + 모서리 80mm 이상, 설계서 6절 권장 배치; 단동식 반경 모델 씨앗 20에서 낙관 98%·비관 85%, 인식 83/80초). 결과 표의 "방향 정답률 %" 열이 1절 방향 정답률이다.
1. 편집 설치가 원래 폴더를 가리킨다: 이 worktree(`../so101-studio-us`)에서 `python -m so101_studio...`는 `PYTHONPATH=src`가 필요하다(`pytest`는 설정으로 괜찮다).
2. 초음파 시험만 `python -m pytest -q tests/perception`(약 6초). 팔·초음파 세션은 `--ignore=tests/gui`. 벤치마크는 `python -m so101_studio.perception.ultrasonic.benchmark --seeds 20`(1~2분), 결과 JSON은 `runs/bench/`(git 제외라 새 PC에 안 온다). **성능 숫자는 이 도구의 고정 시나리오 결과만 쓴다**(결정 0037). 모델 값(`MODELS`, `DXL_LAG_DEG`)이 바뀌면 다시 내고, 비교가 ±5%p 근처면 씨앗 60개로 확인한다.
3. 벤치마크 사이클은 작업 실행을 흉내 낸 것이다(`poses.radar_clear`로 RESCAN, 집은 뒤 PLACE 시작에 prefetch, 빈 자리는 `note_failure`). 실제 작업 실행과 다르면 고친다. 가상 팔은 `arm/trajectory.py`의 `fillet_path`·`LookaheadShape`를 import한다(벤치마크 도구만의 예외, source·check·radar는 팔을 import하지 않는다). `test_fake_arm_matches_real_arm_times`가 `--sim` 실측 시간에 고정한다.
4. 사람 몫 가정(실측이 나오면 `MODELS`를 고친다): 확인 직선 180mm, 도달 clip 60~255mm, 서보 지연(pwm 3.0°, dxl 0.3°)·읽기 잡음 0.1°, 시간 기준 평균 70초.
5. git에 없는 것: `config/ultrasonic_baseline.json`(레이더 기준값, 첫 `calibrate()`가 만든다, `.gitignore`는 내 영역 밖), `config/local.yaml`, arduino-cli(PC마다 위치가 다르다. 새 PC는 Arduino IDE 2 + ESP32 보드 패키지 3.x).
6. 서보는 부팅·포트 열기마다 90°로 움직인다. 서보를 달 때 손을 대지 않고 회전 범위(10~170°)를 비운다.
7. **한글 사용자 폴더**(`C:\Users\<한글 이름>`): 기본 Arduino15 경로에서 ESP32 링크가 `cannot find -lcxx`로 실패한다. `C:\arduino15`에 core를 새로 설치하고 `ARDUINO_DIRECTORIES_DATA=C:\arduino15`, `ARDUINO_DIRECTORIES_USER=C:\arduino15\user`를 준 뒤 `arduino-cli compile --fqbn esp32:esp32:esp32 --build-path C:\tmp\fw firmware/esp32_scanner`.
8. OpenRB 펌웨어 컴파일: 보드 패키지 `OpenRB-150:samd`(인덱스 `https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json`) + Dynamixel2Arduino 0.8.2를 `C:\arduino15`에 설치하고 `arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --warnings all --build-path C:\tmpw2 firmware/openrb_servo`(경고 없음, 플래시 12%). XL330은 5V이고 OpenRB VIN에 12V를 넣지 않는다(H2).
9. 펌웨어 규약(`ultrasonic-spec.md` 7절): `sync`의 `pc`는 64비트 ns라 문자열 그대로 돌려준다. 측정 줄의 `us`는 트리거 시각이다(메아리 왕복 절반은 보정하지 않는다). stream이 켜지면 레이더 회전은 멈춘다. `gap`·`done`·`k:"ra"`·`driver`가 정식 규약이다. 외부 라이브러리 없음.
10. PR 열기: `gh`가 없으면 내장 브라우저로 `compare/main...<브랜치>?quick_pull=1`을 열어 제목·본문을 넣고 `form.requestSubmit()`(버튼이 안 눌리면 입력 뒤 1.5초 기다린다). 질문·승인은 총괄에게 `SendMessage`, 자기 `dev/` 브랜치 커밋·푸시·PR은 묻지 않는다(session-workflow 7절).
11. 알려진 한계: `follow_path`가 블로킹인 동안 `Link.poll()`이 안 불려 ESP32 끊김은 구간이 끝난 뒤에야 안다. 빠른 모드 T9(조각 호출)는 끈 상태라(`early_stop_cells: 0`) 구간 사이 멈춤 시간은 측정된 적 없다. 배출한 물체가 다음 스캔에 후보로 잡히는 것은 `UltrasonicSource(ignore_zones=...)`로 뺀다(작업 실행이 배출 구역을 넘긴다).
12. 호버 도달 띠는 그대로 둔다(결정 0042). A1 실측에서 `ik_fail`·성공률을 보고 다시 본다. 벤치마크 옵션 `--reach kin`, `--hover-mm`, `--tool-reach`, `--trim-start`.

## 실측 절차 (B1~B3, 서보 없음)
순서 전체는 `measurements.md` "실물 시험 순서". 사용자가 옆에 있어야 한다. 배선은 H2 확인(B4는 OpenRB 포함) 뒤에 안내한다.

**0. 배선**(전원을 끄고): 레이더 TRIG 25/ECHO 26, 팔 TRIG 32/ECHO 33, ECHO는 1kΩ/2kΩ 분압(5V → 3.3V), 센서 전원 ESP32 5V(VIN), GND 공통. ESP32는 USB 데이터 케이블로 PC에 연결.

**1. 포트 찾기**: `so101.cmd find-port`의 안내대로 USB를 뽑아 사라진 포트가 ESP32 포트다. 총괄에 알린다.

**2. 펌웨어 올리기**(`firmware/esp32_scanner`): Arduino IDE 2(보드 `ESP32 Dev Module`, **ESP32 보드 패키지 3.x**, 올리는 동안 시리얼 모니터 닫기) 또는 `arduino-cli upload -p COMx --fqbn esp32:esp32:esp32 firmware/esp32_scanner`. 올린 뒤 EN 버튼을 한 번 누른다.

**3. 연결 확인**: `conda activate so101`, `$env:PYTHONPATH="<레포>/src"`, 그리고 `python -m so101_studio.perception.ultrasonic.bench --port COMx --k a --n 5 --label check`. `hello`와 팔 센서 거리 5개가 보이면 된다(921600에서 안 되면 `--baud 115200`과 펌웨어 `Serial.begin`을 같이 바꾼다).

**4. 측정**(CSV는 `runs/bench/`, 값만 총괄에 보낸다):
- **B1 아래로 본 빔 반경**: 팔 센서를 약 155mm 높이(#359, 전 110mm)에 아래로 고정, 높이 40mm 상자를 옆에서 5mm씩 밀어 넣는다. 자리마다 `bench --port COMx --k a --n 20 --gap 60 --label B1-<앞뒤|좌우>-<mm>`. 각 자리의 `median`이 155mm에서 115mm 쪽으로 바뀌기 시작하는 수평 거리를 앞뒤·좌우로 적는다 → `beam_half_deg`.
- **B2 옆에서 본 면 각도 한계**: 레이더 센서 20mm 높이 수평, 200mm 앞 상자를 5°씩 돌리며 `--k r --n 10 --gap 60 --label B2-<각도>`. `outlier/none`이 0%에서 올라가기 시작하는 각도.
- **B3 최소 측정 간격**: 두 센서를 서로 비스듬히 두고 `--k ra --n 200 --gaps 20,25,29,33,40 --label B3`. 엉뚱한 값 비율이 0%에 가까운 가장 작은 간격 → `ping_gap_ms`.
- 간격 60ms 이상은 메아리가 없을 때 ECHO가 약 38ms HIGH로 남는 영향을 피하려는 것이다. `경고: ... done 못 받음`이 나오면 그 간격 결과는 쓰지 않는다(케이블·포트·펌웨어 확인).

**5. 총괄에 보낼 값**: 포트, 펌웨어 커밋, B1·B2·B3 표, 이상한 점. 올린 뒤 "ESP32 반납"을 알린다.

## B4 절차·기록 양식 (서보 지연·떨림, XL330)
준비: 레이더 서보를 작업대에 고정하고 센서면을 수직으로 맞춘다. 정면(90°) 300mm 앞에 폭 30mm 이상의 단단한 물체 하나(다른 물체 없음). 기준값을 먼저 만든다(`calibrate()`). `servo.driver: dxl`로 바꾸고 연결할 때 펌웨어에 `driver` 명령을 보낸다.

- **B4-1 지연**: `sweep from 10 to 170 step 4 bounce` 20바퀴. 같은 물체 호의 중심 각도를 갈 때(dir=+1)·올 때(dir=-1)로 구해 차이 d, 지연 = d/2. 합격은 |지연| 3° 이하(spec 10절). 벤치마크 `servo_lag_deg`를 이 값으로 바꾼다.
- **B4-2 정지 떨림**: 한 각도(예 90°)에 `servo`로 세우고 `once` r 200회. 폭 30mm 물체 모서리 쪽 20° 지점의 거리 표준편차·최대-최소.
- **B4-3 도착 시간**: 4°씩 명령한 뒤 33ms 안에 도착하는지, `once` 간격 10·20·33ms에서 거리가 안정되는 최소 간격(가정 약 7ms).
- **B4-4 ang 차이**: 읽은 각도 `ang`와 명령 각도 `a`의 평균·최대(갈 때·올 때 나눠). `ang`를 한 칸 늦춰 합쳐 후보 위치 오차가 변하는지로 UART 지연을 본다. `ang_noise_deg` 0.1을 이 값으로 바꾼다.

`measurements.md`에 쓸 항목: 날짜·서보·전원·보레이트·펌웨어 커밋 / 지연 d/2 중앙값·최대 / 정지 떨림 / 최소 안정 간격 / ang-a 평균·최대 / 판정(±3° 이하? 아니면 spec 10절 대안) / 원자료 `runs/b4-<시각>/`.

## 이벤트 (Studio.bus, GUI가 구독)
- `us.sweep`: `{"pass", "dir": 1|-1, "t_ns", "angles", "d"}` 한 방향 회전이 끝날 때마다.
- `us.candidates`: `{"objects", "arcs", "masked"}` scan()마다.
- `us.check_path`: `{"id", "stage": "L1"|"L2"|"R"|"L3"|"L4"|"GRID", "phase": "start"|"end", "points_mm", "tool_z_mm", "points", "unreachable"?}`.
- `us.check_result`: `{"id", "object", "info", "status": "ok"|"empty"|"unreachable"}`.

## 작업 실행이 부르는 것
`source.begin_cycle()`(사이클 시작, `calibrate()`도 부름), `probe.abort()`(정지·E-STOP·fault로 확인을 끝낼 때, 후보는 지우지 않음), `source.rescan()`, `source.prefetch`, `source.note_failure(x, y)`, `source.give_up`. 빈 목록이 `ik_fail`·`out_of_bounds`면 `abort`가 아니라 `probe.feed([])`다.
