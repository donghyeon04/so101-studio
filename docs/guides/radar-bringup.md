# 레이더 조립·점검 안내 (사용자용)

상태: 초안 · 2026-11-19 (초음파 세션 작성, 결정 0043·0067, `ultrasonic-spec.md` 10절 B1~B4·R1, `real-arm-runbook.md` 3절의 실물 첫 단계) · 2026-11-20 R1 명령을 `radar_check r1`로(REQ-096)

부품만 출력한 상태에서 시작한다. 실물 작업은 사용자가 한다. 막히면 그 자리에서 멈추고 알린다. 팔은 이 안내에서 쓰지 않는다(전원 끔).

## 0단계. OpenRB VIN 전압 (이것 전에는 아무것도 배선하지 않는다)
XL330은 5V용(허용 3.7~6.0V)이다. **12V가 OpenRB VIN에 닿으면 XL330이 망가진다.**

| # | 할 일 | 합격 |
|---|---|---|
| 0-1 | 전원은 둘 중 하나: A) USB 5V 3A 충전기 + USB-A 단자 보드, B) 12V 어댑터 → 강압 모듈(출력 5.0V로 맞춤) | 12V 어댑터를 OpenRB에 직접 꽂지 않는다 |
| 0-2 | 멀티미터(DC 20V 칸)로 **전원 출력 단자**를 먼저 잰다. OpenRB는 아직 연결 안 함 | 4.8~5.2V |
| 0-3 | OpenRB VIN 단자(+/−)에 연결하고 점퍼를 **VIN(DXL)** 쪽에 꽂은 뒤, VIN 단자에서 다시 잰다. XL330은 아직 안 꽂음 | 4.8~5.2V |
| 0-4 | 470µF 16V 전해 커패시터를 VIN 단자 + / −에 붙인다(띠 표시 쪽이 −) | 극성 맞음 |

6V 넘게 나오면 즉시 뽑고 알린다.

## 1단계. 조립 (모터 → 가운데 맞추기 → 브래킷)
1. 모터 브래킷(`모터 브라켓`)을 프로파일에 고정하고 XL330을 그 위에 나사 4개로 단다. 혼(출력 축)이 판 앞(−y 쪽)을 향하는 쪽 끝에 오게(CAD: 몸체 가운데 y 398, 혼 축 y 405.5).
2. XL330을 OpenRB DXL 포트에 3핀 케이블로 꽂는다.
3. **서보를 가운데(눈금 2048 = 명령 90°)에 둔다.** 브래킷을 끼우기 전에 한다. 두 방법 중 하나:
   - 방법 A: Dynamixel Wizard 2.0 → 토크 켜고 Goal Position 2048(180°)
   - 방법 B: 아래 4·5단계의 펌웨어를 올린 뒤 `radar_check ... center`(5단계 명령)
4. **센서 브래킷을 혼에 끼운다.** 센서 앞면(초음파 눈 두 개)이 −y(판 안쪽, 팔이 있는 쪽)를 보게. 혼 나사 자리가 90° 간격 4곳이라 방향을 눈으로 확인하고 끼운다. 한 칸(90°) 틀리면 센서가 옆을 본다.
5. **손으로 10~170° 돌려 본다.** 전원을 끈 상태에서 천천히 좌우 80°씩. 회전 브래킷과 모터 브래킷 사이는 CAD상 1.2~2.8mm뿐이라 걸리면 걸린 자리를 줄로 살짝 다듬는다. 센서·브래킷은 판 위 11mm 이상 떠서 지나간다.

## 2단계. 배선 (표)
| 신호 | 한쪽 | 다른 쪽 | 비고 |
|---|---|---|---|
| 서보 전원 | 5V 전원 + / − | OpenRB VIN 단자 + / − | 점퍼 VIN(DXL), 470µF 병렬 |
| 서보 통신 | ESP32 GPIO 17 (TX2) | OpenRB Serial2 **RX** | 3.3V 직결, 분압 불필요 |
| 서보 통신 | ESP32 GPIO 16 (RX2) | OpenRB Serial2 **TX** | TX/RX 교차 |
| 공통 접지 | ESP32 GND | OpenRB GND, 충전기 GND | 전부 하나로 |
| 레이더 센서 | TRIG GPIO 25, ECHO GPIO 26 | HC-SR04 | **ECHO는 1kΩ/2kΩ 분압**(5V→3.3V) |
| 팔 센서 | TRIG GPIO 32, ECHO GPIO 33 | HC-SR04 | ECHO 분압 |
| 센서 전원 | ESP32 5V(VIN) | 두 HC-SR04 VCC | 서보 전원은 여기서 쓰지 않는다 |
| PC | ESP32 USB(데이터 케이블), OpenRB USB-C | PC | COM 포트 두 개 |

OpenRB Serial2 단자는 2mm 1x4 핀 헤더 소켓 납땜이 필요하다.

## 3단계. 펌웨어와 설정
ESP32와 OpenRB를 각각 올린다(한글 사용자 폴더면 `C:\arduino15`를 쓴다: `ARDUINO_DIRECTORIES_DATA=C:\arduino15`, `ARDUINO_DIRECTORIES_USER=C:\arduino15\user`).
```powershell
arduino-cli upload -p COM(ESP32) --fqbn esp32:esp32:esp32 firmware/esp32_scanner
arduino-cli upload -p COM(OpenRB) --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_servo
```
올리는 동안 시리얼 모니터를 닫는다. ESP32는 올린 뒤 EN 버튼을 한 번 누른다. 포트를 모르면 `so101.cmd find-port`(USB를 뽑아 사라진 포트가 그 보드).
PC 설정(이 PC에서만, git에 안 올라간다):
```powershell
Copy-Item config\ultrasonic.local.example.yaml config\ultrasonic.local.yaml   # servo.driver: dxl
```
`config/ultrasonic.local.yaml`의 `port:`를 ESP32 포트로 바꿔도 된다.

## 4단계. 첫 켜기와 가운데 확인
`conda activate so101` 뒤 레포 폴더에서(`radar_check`는 #477로 들어왔다). 센서 앞 20cm를 비우고 서보 회전 범위(10~170°)에 손을 넣지 않는다. **서보는 연결할 때마다 90°로 움직인다.**
```powershell
python -m so101_studio.perception.ultrasonic.radar_check --port COM(ESP32) center
```
센서가 −y(판 안쪽)를 정확히 보는지 본다. 틀리면 브래킷을 혼에서 빼고 한 구멍 돌려 다시 끼운다.

## 5단계. 측정 (순서대로, 각 기준은 ultrasonic-spec 10절)
| 시험 | 사용자가 하는 일 | PC 명령 | 성공 기준 |
|---|---|---|---|
| B3 최소 측정 간격(서보 없이, 센서 두 개를 서로 비스듬히 고정, 앞에 고정 물체) | 센서 두 개를 같은 물체 쪽으로 | `python -m so101_studio.perception.ultrasonic.bench --port COM(ESP32) --k ra --n 200 --gaps 20,25,29,33,40 --label B3` | 엉뚱한 값 비율이 0%에 가까운 가장 작은 간격(지금 `ping_gap_ms` 33). `경고: done 못 받음`이 나오면 그 간격은 버린다 |
| B1 아래로 본 빔 반경 | 팔 센서를 약 155mm 높이에서 아래로 고정, 40mm 상자를 옆에서 5mm씩 밀어 넣는다 | 자리마다 `bench --port COM --k a --n 20 --gap 60 --label B1-앞뒤-<mm>` (좌우도) | median이 155 → 115mm쪽으로 바뀌기 시작하는 수평 거리 → `beam_half_deg` |
| B2 면 각도 한계 | 레이더 센서를 작업대 면 위 수평으로, 200mm 앞 상자를 5°씩 돌린다 | `bench --port COM --k r --n 10 --gap 60 --label B2-<각도>` | 메아리가 끊기는(outlier/none이 올라가는) 각도 |
| B4-1 서보 지연 | 정면(90°) 약 300mm에 폭 30mm 이상 단단한 물체 하나만 | `radar_check --port COM lag` | \|지연\| 3° 이하 |
| B4-2 정지 떨림 | 같은 물체 앞 | `radar_check ... center` 뒤 `bench --port COM --k r --n 200 --label B4-jitter` | 표준편차가 작고 최대-최소가 몇 mm 안 |
| R1 레이더 단독 | 빈 판(팔은 시야 밖)에서 기준값, 그 뒤 물체 1개를 3×3 자리(x 150/215/280, y 150/190/230 mm, 작업대 좌표)에 3번씩 | 먼저 `radar_check --port COMx calibrate`(빈 판) → `radar_check --port COMx r1 --obj 폭x길이x높이`(예 `40x80x40`, 자리마다 물체를 놓고 Enter, 3회. 정답 쌍이 `runs/ultrasonic-truth/<시각>-R1/`에 남는다) | 후보가 실제 중심에서 90mm 안인 비율 95%(27개 중 26개) 이상 |

R1에서 후보가 좌우 거울이면(물체가 오른쪽인데 왼쪽으로 나옴) 펌웨어 `openrb_servo.ino`의 `DIR`을 +1/−1로 뒤집어 다시 올린다. 90°가 정면이 아니면 브래킷을 90° 단위로 다시 끼운다.

## 알릴 값
- 0단계: VIN 전압 두 개, 전원 방식(A/B)
- 포트 두 개, 펌웨어 커밋
- B3·B1·B2 표(CSV는 `runs/bench/`), B4 지연·떨림, R1 27개 오차
- 이상: 브래킷이 걸린 자리, 거울 여부
