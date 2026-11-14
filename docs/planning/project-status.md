# 프로젝트 현황 (총정리 · 새 세션은 여기부터)

상태: 초안 · 2026-11-14 저녁 (기획이 총괄 오후 보고 `planning-inbox` `inbox/2026-11-14-afternoon-report.md`로 다시 씀. 큰 단계가 끝날 때마다 기획이 고친다)

> 새 계정·새 세션은 이 문서 하나로 시작한다. 할 일과 사용자 결정 대기는 `work-queue.md`, 세션 운영(폴더·브랜치·보고 체계·시작 프롬프트)은 `../guides/session-workflow.md`에 있다.

## 1. 한 줄
SO-101 팔이 초음파로 물체를 찾아 집어 옮기는 시스템(12/4 시연)이다. 소프트웨어는 시뮬레이션에서 끝까지 돌고, 실물 팔은 단동식 그리퍼로 집어 옮기기 10/10(30%·50%)까지 확인했다. **11-14에 방향을 확정했다: 그리퍼는 로보나인 병렬, 손목에는 짧은 초음파 브래킷만(0°), 카메라는 탑뷰 하나(결정 0063).** 그래서 실물·MuJoCo·초음파를 로보나인 기준으로 다시 맞추는 것이 다음이다. 필수 시연의 실물(초음파·레이더·통합)은 아직 0%이고 부품·작업대·브래킷 출력을 기다린다.

## 2. 부분별 지금 상태 (main 5a06ee9 기준)
| 부분 | 된 것 | 남은 것 | 기준 문서 |
|---|---|---|---|
| 팔 | 동작·안전 검사·CLI·교시·자동 캘리브레이션(0° 확인까지), 작업 실행 `CHECK`·`RESCAN`, 그리퍼 두 종(단동식·병렬), 장면 부품 충돌, 막힘 감지(0059·0061), 포트 PC별 자동(#337) | 로보나인 교체 뒤 그리퍼 값 실측(runbook 2-10)·2-2 재시험, CLI `run`이 남은 물체를 팔에 알리기(REQ) | `arm-software-spec.md`, `arm-dev-status.md` |
| GUI | 페이지 15개 + 운영자 화면, `--share`, 캘리브레이션 마법사, 직접 조작(현재 좌표·되돌리기), MuJoCo GUI(M4) | 되돌리기 알림 줄이기·3D에서만 바꾼 것도 되돌리기(`gui-spec.md` 12.9), 장면 부품 편집기 | `gui-spec.md`, `gui-dev-status.md` |
| MuJoCo | M1~M4(드라이버·장면·시나리오·뷰어·GUI), 손목 브래킷 질량·센서 site(#336, 병렬 손일 때만) | **M4b 로보나인 손 모델** → M5 집기 시험장·M6 초음파를 MuJoCo에(결정 0062·0063) | `mujoco-spec.md` 13.10 |
| 초음파 | 레이더 + 팔 확인 인식, 가상 ESP32·벤치마크, OpenRB-150 + XL330 펌웨어(컴파일만) | 센서 오프셋 REQ({0.0, 66.5, 99.5}, 받기 대기), 부품 뒤 B1~B4 → R1 → A0·A1 → I1 | `ultrasonic-spec.md`, `us-dev-status.md` |
| 하드웨어 | 손목 브래킷 설계·평가 끝(15.5g, 간섭 없음, #335) | H1 레이더 부품, H2 OpenRB VIN 확인, H5 작업대, H7 카메라 주문, 브래킷 출력(U7), 로보나인 교체(U5) | `hardware.md`, `workbench-hardware-requirements.md` |
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

사용자 결정 대기(U4·U5·U7·U8)와 세션별 다음 일은 `work-queue.md`.

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

## 7. git 밖에만 있는 것 (이 PC, 11-14 저녁 · 새 세션이 모르면 잃는다)
| 무엇 | 어디 | 할 일 |
|---|---|---|
| 0° 확인 기록 `calibration_checked`(11-14) | GUI 폴더 `config/arms/so101_follower_01.yaml`(커밋 안 함) | 팔 파일은 git이라 **PR로 커밋**한다(안 하면 다른 폴더의 `so101.cmd`는 잠김) |
| 실물 2-2 반복 스크립트 `real_22.py`, 브래킷 출력 방향 3D 페이지 `bracket-print.html`, 오전 보고, EGM 남은 패치 | GUI 폴더 `runs/handoff-1114/` | 그대로 둔다. 2-2 재시험(로보나인) 때 `real_22.py`를 쓴다. 계속 쓸 도구가 되면 시험장(0062)에 넣을지 정한다 |
| 실물 기록 모음 | GUI 폴더 `runs/real-eval-20261113/`, 팔 폴더 `runs/from-armtmp/` | 그대로 둔다(요약은 `measurements.md`에 있음) |
| 사용자 교시 기록 | 팔 폴더 `config/sequences/1234.json`, GUI 폴더 `test.json`(추적 안 함) | 시연에 쓸 것만 사용자가 골라 커밋 |
| 팀 CAD 원본 | `C:\Users\김동현\Downloads\브라켓 설계 초안\cad\Gripper(브라켓 짧은 버전).step/.stl` | 레포 밖(팀 드라이브)에 둔다. 3D·충돌에 쓰는 STL만 `assets/`에 출처와 함께 |
| 로컬 브랜치 `backup/tmp-merge-b59d933` | 저장소 | `git log main..backup/tmp-merge-b59d933`로 main에 없는 커밋을 확인하고, 필요 없으면 사용자 승인 뒤 지운다 |
| `config/local.yaml` | 각 폴더(없을 수 있음) | 포트는 보드 시리얼로 자동(#337)이라 없어도 된다 |
