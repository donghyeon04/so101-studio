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
| 센서 오프셋(#359, 합쳐짐) | `arm_sensor_offset_mm` {0.6, 66.5, 99.5}, `ring_r_mm` 60, 센서 높이 약 154.5mm(전 110mm, 결정 0063 로보나인+짧은 브래킷). `single` 비관 성공 78→62%(`ultrasonic-spec.md` 1절). **다음 일**: CHECK 높이·경로 다시 고르기, 1절 표·`check.tool_reach_mm`(60~255는 단동식 기준) 다시 내기. 실물은 A0로 확인 |

이 세션은 ESP32·서보 포트를 한 번도 연 적이 없다. 포트는 사용자가 옆에 있을 때만 열고, 열기 전과 반납 뒤에 총괄에게 알린다. COM3(팔)은 쓰지 않는다.

## 알려진 함정
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
