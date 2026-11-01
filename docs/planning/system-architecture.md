# 시스템 설계서

상태: 초안 · 갱신 2026-11-01

시스템 전체가 어떻게 맞물리는지 정한다. 모듈별 상세 설계는 이 문서의 경계를 따른다. 팔 모듈은 `arm-software-spec.md`, 인식(초음파·카메라)과 GUI는 해당 개발 단계에서 따로 쓴다.

근거: `overview.md`, `features.md`, `decisions/0003`~`0006`.

## 1. 시스템 구성
```
                         Windows PC (Python 3.12, 프로세스 1개: so101_studio)
 ┌───────────────────────────────────────────────────────────────────────────┐
 │  앱      CLI (지금)                       GUI (나중, 기술 미정)            │
 │            └──────────────┬──────────────────────┘                        │
 │  서비스              Studio (ArmService + TaskService + PerceptionService) │
 │            ┌──────────────┼──────────────────────┐                        │
 │  모듈    task(작업 실행)   arm(팔)              perception(인식)            │
 │  공통    core: types · frames · config · events · logging                 │
 └────────────────┬──────────────────────────┬──────────────┬───────────────┘
                  │ USB (LeRobot)            │ USB 시리얼    │ USB (나중)
          버스 서보 보드 → SO-101      ESP32 → HC-SR04 ×2   카메라
                                       + 스캔 서보 ×2 (미구매)
```
- PC에서 도는 프로그램은 **하나**다. 모듈은 같은 프로세스 안에서 스레드로 나뉜다. 5주 일정과 Windows 1대 환경에서 프로세스 간 통신(Redis, Docker 등)은 쓰지 않는다.
- ESP32는 센서와 스캔 서보만 다룬다. 판단은 하지 않고, 각도별 거리값만 PC로 보낸다.

## 2. 모듈과 의존 방향
| 모듈 | 역할 | 부를 수 있는 모듈 | 상세 설계 |
|---|---|---|---|
| `core` | 데이터 타입, 좌표 변환, 설정, 이벤트, 기록 | 없음 | 이 문서 |
| `arm` | 팔 연결, 기구학, 궤적, 집기·놓기, 원격 조종, 녹화·재생 | `core` | `arm-software-spec.md` |
| `perception` | 물체 인식. 출력은 `DetectedObject` 목록 | `core` | 3단계(초음파), 4단계(카메라)에서 작성 |
| `task` | 작업 실행(판단): 스캔 → 우선순위 → 집기 → 반복 | `core`, `arm`, `perception`의 공개 인터페이스 | 이 문서 5절 |
| `service` | 앱이 부르는 유일한 입구. 상태·이벤트 발행 | 위 전부 | 이 문서 7절 |
| `cli`, `gui` | 사용자 화면 | `service`만 | GUI는 나중 |

- `arm`과 `perception`은 서로 모른다. 둘을 잇는 것은 `task`뿐이다 (`decisions/0004`).
- 위 표의 방향을 거스르는 import는 쓰지 않는다.

## 3. 공통 데이터 타입 (`core/types.py`)
팔 관련 타입(`JointState`, `ToolPose`, `ArmStatus`, `MotionResult`)은 `arm-software-spec.md` 5절을 따른다. 시스템 공통으로 아래를 추가한다.

```python
@dataclass
class DetectedObject:      # 모든 인식 방식의 공통 출력 (decisions/0004)
    id: str                # 스캔 안에서의 번호, 예: "obj-1"
    x_mm: float; y_mm: float          # 작업대 좌표계
    angle_deg: float | None           # 0~180, None = 판단 불가
    width_mm: float; length_mm: float
    confidence: float                 # 0~1
    source: str                       # "manual" | "ultrasonic" | "camera"

@dataclass
class ScanResult:
    objects: list[DetectedObject]
    duration_s: float                 # 인식 속도 비교에 쓴다
    raw: dict                         # 원자료 (초음파: 각도별 거리, 카메라: 이미지 경로)

@dataclass
class TaskEvent:
    t: float; state: str              # 5절의 상태 이름
    object_id: str | None
    result: str | None                # "ok" | "skipped" | "failed" | 오류 이름
    detail: dict
```

## 4. 인식 모듈 공통 규격 (`perception/`)
모든 인식 방식은 같은 규격을 구현한다. 작업 실행 모듈은 어떤 방식인지 모른다.

```python
class PerceptionSource(Protocol):
    name: str
    def connect(self) -> None: ...
    def calibrate(self) -> dict: ...          # 빈 작업대 기준값. 결과 요약을 돌려준다
    def scan(self) -> ScanResult: ...
    def close(self) -> None: ...
```

| 구현 | 단계 | 내용 |
|---|---|---|
| `ManualSource` | **지금** | 사람이 입력한 좌표 또는 `config/manual_objects.yaml`을 그대로 돌려준다. 센서 없이 전체 흐름(스캔 → 집기 → 반복)을 개발·시연할 수 있다 |
| `UltrasonicSource` | 3단계 | ESP32에 스캔 명령 → 각도별 거리 수신 → dip 검출, 좌표 변환, 매칭, 크기·방향 추정 |
| `CameraSource` | 4단계 | 영상 → 같은 `ScanResult` |

## 5. 작업 실행 모듈 (`task/`)
PPT 흐름도(캘리브레이션 → 스캔 → 판단 → 실행 → 남은 물체가 있으면 다시 스캔)를 상태 머신으로 옮긴다.

```
 IDLE ──시작──▶ PREPARE ──▶ SCAN ──▶ DECIDE ──물체 있음──▶ PICK ──▶ PLACE ─┐
   ▲            (팔 park)    │         │                                   │
   │                         │         └──물체 없음──▶ DONE ──▶ IDLE        │
   │                         └◀──────────── 다시 스캔 ◀────────────────────┘
   │
   └── STOPPED ◀── 정지/비상 정지 (어느 상태에서든)      FAULT ◀── 장치 오류 (어느 상태에서든)
```
| 상태 | 하는 일 | 다음 상태 |
|---|---|---|
| `PREPARE` | 팔을 센서 시야 밖 대기 자세로(`park`) | `SCAN` |
| `SCAN` | `perception.scan()` | `DECIDE` |
| `DECIDE` | 도달 범위 밖 물체 제외 → 우선순위(기본: 팔에서 가까운 순) → 1개 선택 | 있으면 `PICK`, 없으면 `DONE` |
| `PICK` | `arm.pick(obj)` | 성공 `PLACE`, 실패 시 재시도 1회 후 건너뛰고 `PREPARE` (재시도는 목표 층) |
| `PLACE` | `arm.place(zone)` | `PREPARE` (물체를 하나 옮길 때마다 다시 스캔) |

- **단계별 실행 모드(D3):** 상태가 바뀔 때마다 멈추고 `step()`을 기다린다. 시연 때 한 단계씩 설명하는 용도다.
- **사이클 상한:** 같은 물체를 3회 연속 실패하거나 사이클이 20회를 넘으면 `DONE`으로 끝낸다.
- 모든 상태 전환은 `TaskEvent`로 발행되고 기록된다.

## 6. PC ↔ ESP32 통신 규약 (틀만, 상세는 3단계)
- USB 시리얼 115200bps, 한 줄에 JSON 하나(줄바꿈으로 구분).
- PC → ESP32: `{"cmd":"ping"}`, `{"cmd":"scan","station":"A","from":10,"to":170,"step":2,"samples":5}`, `{"cmd":"stop"}`
- ESP32 → PC: `{"type":"sample","station":"A","angle":12,"dist_mm":[312,310,315,311,309]}` (측정값 원본 그대로, 필터는 PC에서) / `{"type":"done","station":"A"}` / `{"type":"error","msg":"..."}`
- 두 스테이션은 PC가 A → B 순서로 명령해 순차 측정한다(간섭 방지).

## 7. 서비스 계층과 이벤트
- 앱은 `Studio` 객체 하나만 만든다. `studio.arm`(`ArmService`, 팔 설계서 6.9절), `studio.task`, `studio.perception`을 가진다.
- `studio.task`: `start(source_name, step_mode)`, `step()`, `stop()`, `status()`
- `studio.perception`: `select(source_name)`, `calibrate()`, `scan()`
- `studio.estop()`: 작업 중단 → 팔 비상 정지 → 인식 중단을 한 번에 한다. 어느 화면에서든 부를 수 있다.
- 상태와 이벤트는 내부 이벤트 버스(`core/events.py`, 단순 발행·구독)로 퍼진다. GUI는 여기에 구독해 화면을 갱신한다.

## 8. 실행 구조 (스레드)
| 스레드 | 주기 | 역할 |
|---|---|---|
| 메인 | — | CLI 또는 GUI |
| 동작 루프 | 50Hz | 팔 궤적 실행, 안전 검사 (팔 설계서 6.4절) |
| 상태 수집 | 10Hz | `ArmStatus` 발행, 2Hz로 온도·부하 |
| 센서 수신 | 수신 즉시 | ESP32 시리얼 읽기 (3단계부터) |
| 작업 실행 | 상태 전환마다 | 상태 머신 |

장치 하나는 스레드 하나만 만진다. 다른 스레드는 큐로 요청한다.

## 9. 레포 폴더 구조
```
src/so101_studio/
  core/          types.py, frames.py, config.py, events.py, logging.py
  arm/           (팔 설계서 3절)
  perception/    base.py, manual.py, ultrasonic.py(3단계), camera.py(4단계)
  task/          runner.py, policy.py (우선순위·도달 범위 판단)
  service/       studio.py, arm_service.py
  cli/           __main__.py
  gui/           (기술 결정 후)
firmware/esp32_scanner/   (3단계)
config/          robot.yaml, workspace.yaml, manual_objects.yaml, sequences/, calibration/
assets/          URDF, 메쉬
tests/           하드웨어 없이 도는 테스트 (--sim, ManualSource)
runs/            실행 기록 (git 제외)
docs/
```

## 10. 실행 기록
실행할 때마다 `runs/YYYYMMDD-HHMMSS/`를 만든다.

| 파일 | 내용 | 쓰는 곳 |
|---|---|---|
| `events.jsonl` | 명령, `MotionResult`, `TaskEvent`, 오류 | 디버깅, 실행 로그(F1) |
| `status.csv` | `ArmStatus` 10Hz | 떨림·과열 분석 |
| `scans/NNN.json` | `ScanResult` 원본 | 스캔 그래프(E2), 비교 실험 |
| `summary.json` | 물체 수, 성공·실패, 단계별 소요 시간 | 결과표, CSV 내보내기(F2) |

## 11. 개발 단계와 모듈
| 개발 단계 (`decisions/0005`) | 만드는 것 |
|---|---|
| 0~2. 구동 점검, 동작 라이브러리, 작업 공간 | `core`, `arm`, `perception/manual`, `task`, `service`, `cli` |
| 3. 초음파 인식 | `firmware/esp32_scanner`, `perception/ultrasonic` |
| 4. 카메라 인식 | `perception/camera` |
| 5. 비교 실험·GUI | `gui`, 실험 기능(F3, F4) |

0~2단계가 끝나면 `ManualSource`로 "좌표 입력 → 자동으로 여러 물체 집어 옮기기"가 동작해야 한다. 센서가 붙으면 `ManualSource`를 `UltrasonicSource`로 바꾸기만 한다.

## 12. 0~2단계 작업 순서 (로컬 세션)
팔 작업(T0~T11)의 상세와 완료 기준은 `arm-software-spec.md` 10~11절에 있다. 전체 순서는 아래와 같다.

| 순서 | 작업 | 완료 기준 |
|---|---|---|
| 1 | T0 하드웨어 초기 설정, T1 환경 구성 | 캘리브레이션 파일 생성, placo 설치 결과 기록 |
| 2 | C1 `core`: types, frames, config, events, logging | 좌표 변환·설정 읽기 단위 테스트 통과 |
| 3 | T2~T7 팔 (장치 → 기구학 → MoveJ → 안전 → MoveL → 집기·놓기) | 각 작업의 완료 기준 |
| 4 | S1 `perception/manual` | `manual_objects.yaml`의 물체 3개를 `ScanResult`로 반환 |
| 5 | S2 `task` 상태 머신 + 우선순위 | `SimDriver` + `ManualSource`로 물체 3개 자동 사이클이 테스트에서 끝까지 돈다 |
| 6 | S3 `service`(Studio) + CLI `run --source manual [--step]` | 실제 팔로 물체 3개를 자동으로 집어 옮긴다 |
| 7 | T8~T11 원격 조종, 녹화·재생, 시연 동작, 추종 | 각 작업의 완료 기준 |

측정 결과는 `measurements.md`에 기록한다.
