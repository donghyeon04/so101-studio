# 팔 하드웨어 연결과 처음 실행 (다른 PC에서 바로 쓰기)

상태: 확정 · 2026-11-01

측정값의 출처는 `docs/planning/measurements.md` T0~T1이다. 이 문서는 새 PC나 새 계정에서 팔을 바로 쓰기 위한 요약이다.

## 1. 하드웨어 구성
| 항목 | 내용 |
|---|---|
| 팔 | SO-ARM101 Follower 1대 (리더 팔 없음) |
| 서보 | STS3215-C018 12V x 6 (Feetech, 모델 번호 777) |
| 드라이버 | Seeed Studio Bus Servo Driver Board 1개. PC와는 USB-C, 장치 이름 `USB-Enhanced-SERIAL CH343` |
| 전원 | 12V **5A** 어댑터 (조원이 확인). 거친 동작에서도 전류 합 1.7A, 전압 11.8V까지만 내려감 |
| 고정 | 팔은 반드시 작업대에 클램프로 고정한다 |
| 아직 연결 안 함 | ESP32, HC-SR04 초음파 2개, 카메라 |

## 2. 포트, 통신, 모터 ID
| 항목 | 값 |
|---|---|
| 포트 | **COM3** (Windows 장치 관리자의 "포트(COM & LPT)"에서 `USB-Enhanced-SERIAL CH343 (COMx)` 확인) |
| 속도 | 1,000,000 bps (1Mbps), Feetech STS 프로토콜 |
| 설정 위치 | `config/robot.yaml`의 `port`, `calibration_id` |

모터 ID는 **이미 설정되어 있어 새로 설정할 필요가 없다.** 베이스에서 그리퍼 쪽으로 이어지는 순서와 같다.

| ID | 관절 | 비고 |
|---|---|---|
| 1 | shoulder_pan | 영점 보정 −3.0° (`robot.yaml`의 `joint_offsets_deg`) |
| 2 | shoulder_lift | 팔 전체를 드는 서보. 팔을 편 자세에서 부하 100%까지 감 |
| 3 | elbow_flex | 접는 쪽 한계 95° (J2 모터에 닿음) |
| 4 | wrist_flex | |
| 5 | wrist_roll | |
| 6 | gripper | 가동 턱 1개 구조. 토크 한계 500, 가속 한계는 팔과 따로(`gripper_acceleration`) |

## 3. 다른 PC에서 처음 연결하는 순서
1. **환경**: Miniforge 설치 후 conda 환경 `so101` (Python 3.12). `pip install "lerobot[feetech]"`, `conda install -c conda-forge placo=0.9.25` (pip의 `lerobot[kinematics]`는 Windows에서 실패한다), 레포 루트에서 `pip install -e .`. 확인된 버전: lerobot 0.6.1, placo 0.9.25, numpy 2.2.6(conda 설치본), ruckig 0.19.4, draccus 0.11.6. **numpy를 pip로 다시 설치하지 않는다** (MKL 충돌로 조용히 종료).
2. **캘리브레이션 파일 복사**: 레포의 `config/calibration/so101_follower_01.json`을 `C:\Users\<사용자>\.cache\huggingface\lerobot\calibration\robots\so_follower\` 에 복사한다. 이 파일이 없거나 서보 값과 다르면 "캘리브레이션 불일치" 오류가 난다. 같은 팔이면 다시 캘리브레이션하지 않는다.
3. **포트 확인**: 장치 관리자에서 COM 번호를 보고 다르면 `config/robot.yaml`의 `port`를 고친다.
4. **전원 켜고 읽기만 해 본다** (팔은 움직이지 않는다):
   ```
   C:\경로\so101-studio\so101.cmd status
   ```
   관절 6개의 위치, 온도(30대), 전압(약 12V)이 나오면 된다. `so101.cmd`는 conda 활성화 없이 `so101` 환경을 직접 지정한다. 환경 경로가 다르면 `so101.cmd`의 `ENVDIR`을 고친다.
5. **하드웨어 없이 먼저**: `so101.cmd --sim status`, `python -m pytest -q`.
6. **첫 움직임은 낮은 속도로**, 전원 스위치 옆에서 지켜본다: `so101.cmd --speed 20 home`.

## 4. 알아 둘 것
- **토크가 꺼지면 팔이 받침 쪽으로 떨어진다.** 전원이 순간 끊겨도 6개가 동시에 꺼진다. 명령이 끝나도 토크는 유지된다. 팔 아래를 받치지 않는다.
- **받침에 기댄 자세**는 팔꿈치 약 97°이다. 설정 한계(95°)를 넘지만 출발점으로는 허용되고, 거기서 안쪽으로 움직일 수 있다. 홈·파크 자세는 `[0, -90, 93, 0, 0, 3]`.
- 실제 팔은 **속도 30% 이하**로 시작한다. `powertest`와 `shaketest`는 팔을 크게, 빠르게 움직이므로 주변을 비우고 한다.
- `keys`는 키 상태를 창과 상관없이 읽는다. 실행 중 다른 창에 타이핑하지 않는다.
- 작업대 높이 `table_z_mm`(−25)은 **육안 추정**이라 높이 0 근처는 아직 믿지 않는다.

## 5. 문제가 생기면
| 증상 | 확인 |
|---|---|
| `연결 실패`, 포트를 못 연다 | 다른 프로그램(다른 터미널의 so101, Arduino IDE 등)이 COM3을 쓰고 있는지, USB 케이블, `robot.yaml`의 `port` |
| `캘리브레이션 불일치` | 2단계 파일 복사. 같은 팔인데 계속 나면 서보를 교체했거나 EEPROM이 바뀐 것이다 |
| 일부 서보가 응답 없음 | 데이지 체인 케이블 접촉, 12V 전원. ID 1~6 중 빠진 번호를 확인 (`servoinfo`) |
| 서보 6개 전부 응답 없음(포트·케이블·전원 LED는 정상) | **12V 어댑터를 콘센트에서 뽑고 LED가 꺼질 때까지 기다린 뒤 다시 꽂는다.** 어댑터나 보드의 과전류 보호가 걸려 전원이 끊긴 상태일 수 있고, 2026-11-01에 이렇게 풀렸다. 서보를 다시 켜면 토크는 꺼진 채 시작한다(팔을 받친다) |
| 갑자기 토크가 풀림 | 12V 커넥터 접촉 불량(전원 순간 끊김)을 의심. 보호 상태 레지스터가 0이면 서보 보호가 아니다 |
| 온도가 한 번 90°C대로 튐 | 통신 읽기 오류. 3회 연속 60°C 이상일 때만 안전 정지 |
| 원인 불명의 이상 동작 | `runs/날짜-시각/`의 `events.jsonl`, `status.csv`(전압 포함) 확인 |
