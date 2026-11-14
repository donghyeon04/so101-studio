# 프로젝트 현황 (총정리 · 새 세션은 여기부터)

상태: 초안 · 2026-11-14 밤 (새 PC 이전 전 기획이 다시 씀, main 299bea9. 근거 `planning-inbox` `inbox/2026-11-14-afternoon-report.md`·`inbox/2026-11-14-evening-handoff.md`. 큰 단계가 끝날 때마다 기획이 고친다)

> 새 PC·새 계정·새 세션은 이 문서 하나로 시작한다. 새 PC 첫날 순서는 `work-queue.md` 맨 위다. 할 일과 사용자 결정 대기는 `work-queue.md`, 세션 운영(폴더·브랜치·보고 체계·시작 프롬프트)은 `../guides/session-workflow.md`에 있다.

## 1. 한 줄
SO-101 팔이 초음파로 물체를 찾아 집어 옮기는 시스템(12/4 시연)이다. 소프트웨어는 시뮬레이션에서 끝까지 돌고, 실물 팔은 단동식 그리퍼로 집어 옮기기 10/10(30%·50%)까지 확인했다. **11-14에 방향을 확정했다: 그리퍼는 로보나인 병렬, 손목에는 짧은 초음파 브래킷만(0°), 카메라는 탑뷰 하나(결정 0063).** 그래서 실물·MuJoCo·초음파를 로보나인 기준으로 다시 맞추는 것이 다음이다. 필수 시연의 실물(초음파·레이더·통합)은 아직 0%이고 부품·작업대·브래킷 출력을 기다린다.

## 2. 부분별 지금 상태 (main 299bea9 기준)
| 부분 | 된 것 | 남은 것 | 기준 문서 |
|---|---|---|---|
| 팔 | 동작·안전 검사·CLI·교시·자동 캘리브레이션(0° 확인까지), 작업 실행 `CHECK`·`RESCAN`, 그리퍼 두 종(단동식·병렬), 장면 부품 충돌, 막힘 감지(0059·0061), 포트 PC별 자동(#337), CLI `run`도 남은 물체를 팔에 알림(#346), 집은 뒤 상승이 남은 물체보다 높게(#352), 배출 칸 겹침 검사(#356), 병렬 센서 기준점 가운데(#358) | 새 PC에서 0° 확인 다시, CHECK 경로 기준점(병렬은 가운데, REQ), 로보나인 교체 뒤 2-10·2-2 재시험 | `arm-software-spec.md`, `arm-dev-status.md` |
| GUI | 페이지 15개 + 운영자 화면, `--share`, 캘리브레이션 마법사, 직접 조작(현재 좌표·되돌리기, 12.9 #355), MuJoCo GUI(M4) | 화면 비교 시험 안정화(`.mode-detail`), 장면 부품 편집기 | `gui-spec.md`, `gui-dev-status.md` |
| MuJoCo | M1~M4, **M4b 로보나인 손 모델**(#348·#350·#357, FK 0.002mm 일치, carry 30/45/58 통과, kp·힘은 잠정), 손목 브래킷·센서 조립을 뷰어로 사용자 확인 | 병렬 손으로 (a)(b)·문턱 다시 재기 → M5 집기 시험장·M6 초음파를 MuJoCo에 | `mujoco-spec.md` 13.10 |
| 초음파 | 레이더 + 팔 확인 인식, 가상 ESP32·벤치마크, OpenRB-150 + XL330 펌웨어(컴파일만), 센서 오프셋 {0.6, 66.5, 99.5}·`ring_r_mm` 60(#359) | 센서 높이 154.5mm로 성능이 떨어졌다(`ultrasonic-spec.md` 1절). CHECK 높이·경로 다시 고르기, 부품 뒤 B1~B4 → R1 → A0·A1 → I1 | `ultrasonic-spec.md`, `us-dev-status.md` |
| 하드웨어 | 손목 브래킷 설계·평가 끝(15.5g, 간섭 없음, #335), 출력 방향 결정(손목 쪽 평면 +z 바닥) | H1 레이더 부품, H2 OpenRB VIN 확인(U4), H5 작업대, H7 카메라 주문, 브래킷 출력, 로보나인 교체(U5) | `hardware.md`, `workbench-hardware-requirements.md` |
| 카메라 | ESP32-CAM 탑뷰로 결정(0058) | 주문·거치대, 인식 코드(`CameraSource`)는 초음파 실물 뒤 | `comparison-experiment.md` |

**실물 결과 요약**(`measurements.md`): 2-2(58×58×39 상자, 5회 왕복, 단동식) 30% 10/10 ×3, 50%(막힘 감지 켬) 10/10, 오탐 0. 50%에서 쥔 정속 추종 오차 13.5° vs 문턱 15°(여유 10%). 0° 확인 3D·GUI 실물 조작 정상.

## 3. 확정된 큰 결정 (전체 목록은 `../README.md` 색인)
| 결정 | 요지 |
|---|---|
| 0034·0043 | 초음파 = 레이더(XL330이 HC-SR04를 돌림) + 팔 손목 센서 확인 |
| 0053·0062 | MuJoCo는 드라이버 자리에 끼우는 `MuJoCoDriver`, 시험장은 공통 층 하나 + 실행기(우리 알고리즘 지금, 강화학습 12/4 뒤) |
| 0058 | 카메라는 ESP32-CAM 1대 탑뷰, ArUco + minAreaRect |
| 0059·0061 | 막힘 감지는 추종 오차, 문턱은 30% 이하 고정·50%까지 비례·50% 초과는 끔 |
| 0060 | 단동식 집기 기울기 기본 equal_tips(로보나인에는 해당 없음) |
| **0063** | **그리퍼 로보나인, 손목은 짧은 브래킷만(0°), 카메라 탑뷰, 다시 맞추는 순서** |
| 0037 | 검증 도구는 주제마다 하나(시험장, 초음파 벤치마크, 도달 계산) |
| 0045·0046·0048·0056 | 시험 규칙(관련 시험·전체는 하루 한 번), code-review, 설계서 새 절은 EGM Stage 9(총괄이 돌림) |

## 4. 다음 순서 (12/4까지)
1. **로보나인으로 다시 맞추기**(0063): 교체(U5) → 그리퍼 범위 캘리브레이션 → 폭 표·접촉·조임 실측(2-10) → 2-2 재시험 → 브래킷 출력(U7)·장착 → A0.
2. **MuJoCo**: M4b 로보나인 손 → 조립을 뷰어로 확인 → M5 집기 시험장·M6 초음파를 MuJoCo에(세부 설계는 기획이 M4b 뒤 13.10 아래에 펼치고 EGM Stage 9).
3. **초음파 실물**(H1·H2 뒤): B1~B4 → R1 → A0·A1 → I1.
4. **비교 실험**(11/23~29): 시연 반복 측정, 카메라 인식.
5. **마무리**(11/30~12/4): `../guides/release-checklist.md`, PPT·보고서, 리허설. 11/30부터 새 기능은 멈춘다.

사용자 결정 대기(U4·U5 체크리스트, U10 시험 계획)와 세션별 다음 일은 `work-queue.md`.

## 5. 문서 지도와 읽는 순서
| 문서 | 역할 | 누가 고치나 |
|---|---|---|
| 이 문서 | 지금 상태 한 장·결정 요약·다음 순서·git 밖 파일 | 기획(큰 단계마다) |
| `work-queue.md` | 할 일 순서, 사용자 결정 대기, 세션별 다음 일 | 총괄(매일) |
| `session-requests.md` | 세션 간 요청(REQ) 대장 | 보내는 쪽이 행 추가, 받는 쪽이 상태 갱신 |
| `*-dev-status.md` | 세션별 인수인계(그 세션이 읽고 고침) | 각 세션 |
| 설계서 `arm-software-spec`·`mujoco-spec`·`ultrasonic-spec`·`gui-spec`·`system-architecture` | 구현 기준 | 기획 |
| `measurements.md` | 실측 기록 | 실측한 세션 |
| `../decisions/` | 승인된 결정(고치지 않음) | 기획 |
| `../guides/` | 설치·세션 운영·실물 절차·배포 점검 | 기획·총괄 |

**읽는 순서**: `CLAUDE.md` → 이 문서 → `work-queue.md` → 자기 설계서 → 자기 `*-dev-status.md` → `session-requests.md`에서 자기 앞 REQ.

## 6. 세션과 폴더 (같은 PC, 같은 저장소의 worktree)
| 세션 | 폴더(바탕화면) | 영역·기준 문서 |
|---|---|---|
| 기획(원격) | 원격 저장소 | 설계서·결정·조사. 코드 안 씀. 승인 대기 브랜치 `claude/*`에 커밋 |
| 총괄(GUI) | `so101-studio-gui` | `gui/`, `tests/gui/`, 일 나누기·REQ 대장·PR 검토·합치기·EGM. 실물 시험도 이 폴더의 `so101.cmd`로 한다 |
| 팔 | `so101-studio` | 그 밖의 코드, `arm-software-spec.md` |
| MuJoCo | `so101-studio-sim` | `arm/mujoco_*`, `assets/mujoco/`, `config/sim_scenarios/`, `tests/test_mujoco_*`, `mujoco-spec.md` |
| 초음파 | `so101-studio-us` | `perception/ultrasonic/`, `firmware/`, `config/ultrasonic.yaml`, `ultrasonic-spec.md` |
- `so101.cmd`는 **그 폴더의 코드**를 실행한다. 세션을 시작할 때 `git status -sb`로 브랜치를 보고, 합쳐져 지워진 브랜치면 `main`으로 돌린다.
- 합칠 때 `gh pr merge --delete-branch`를 쓰지 않는다(worktree 폴더가 지워진다). 순서는 `session-workflow.md` 2절.
- 대화 기억은 계정·세션을 넘어가지 않는다. 넘어가야 할 것은 모두 git 안 문서(이 문서, `work-queue.md`, `*-dev-status.md`, `session-requests.md`, `measurements.md`, 결정)에 있어야 한다.

## 7. git 밖에만 있는 것 (새 PC로 옮기는 것)
옛 PC의 git 밖 파일은 바탕화면 `so101-이전-1114.zip`(약 6.4MB)에 모았다. 안의 `README.txt`가 각 파일이 새 PC 어디로 가는지 적고 있다. 실행 기록 `runs/` 전체(약 700MB)는 넣지 않았다.
| 무엇 | 새 PC에서 |
|---|---|
| `runs/handoff-1114/`(실물 2-2 스크립트 `real_22.py`, 브래킷 출력 방향 3D 페이지, 보고, EGM 남은 패치) | GUI 폴더 `runs/handoff-1114/`로. 로보나인 2-2 재시험에 `real_22.py`를 쓴다 |
| 실물 기록 `runs/real-eval-20261113/`, `runs/from-armtmp/` | 요약은 `measurements.md`에 있다. 필요하면 같은 자리로 |
| 사용자 교시 기록 `1234.json`, `test.json` | 각 폴더 `config/sequences/`로(추적 안 함). 시연에 쓸 것만 사용자가 골라 커밋 |
| 총괄 GUI 미리보기 설정 `.claude/launch.json` | GUI 폴더 `.claude/`로 |
| 팀 CAD 원본 `브라켓 설계 초안/`(약 37MB) | 레포에 넣지 않는다. 팀 드라이브 한 곳에 두고 `assets/scene/NOTICE.txt`에 위치를 적는다 |
| 팀 공유 3D 페이지(브래킷 출력 방향) | claude.ai 아티팩트(사용자 계정)라 옮길 것 없음 |
| `so101-studio-view`(MuJoCo 확인용 로컬 폴더) | 새 PC에는 만들지 않는다. 로보나인 확인은 MuJoCo 폴더에서 `--sim --sim-engine mujoco --sim-viewer`로 |
| 캘리브레이션 | 파일은 git(`config/calibration/`). 새 PC에서 0° 확인(`calibrate --verify-zero`)을 다시 해야 움직인다 |
