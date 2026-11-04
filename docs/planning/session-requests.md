# 세션 간 요청 대장

상태: 초안 · 2026-11-02

절차와 양식은 `docs/guides/session-workflow.md`에 있다. 요청을 받은 세션이 행을 추가하고 상태를 갱신한다. 자세한 내용은 요청 메시지와 PR에 두고, 여기에는 한 줄만 남긴다.

상태: 예정(기획이 미리 등록) → 요청 → 진행 → 완료 → 반영됨(받는 쪽이 임시 우회를 지움) / 기획 대기 / 거절

| ID | 제목 | 보냄 → 받음 | 종류 | 상태 | PR·커밋 | 메모 |
|---|---|---|---|---|---|---|
| REQ-001 | `Studio.step()`: 단계별 모드에서 다음 단계로 진행. `start(step_mode=True)`와 짝 | 기획 → 팔 | 추가 | 완료 | #4 합쳐짐 | `TaskRunner.wait_step`을 `threading.Event`로 연결. 결정 0020 |
| REQ-002 | 인식 방식 선택·스캔: `Studio.perception.list()`, `select(name)`, `calibrate()`, `scan()` | 기획 → 팔 | 추가 | 완료 | #4 합쳐짐 | system-architecture 7절. 결정 0020 |
| REQ-003 | 실행 중 물체 추가: `ManualSource.add(obj)`, `remove(id)`, 스레드 안전, 다음 스캔에 반영 | 기획 → 팔 | 추가 | 완료 | #4 합쳐짐 | 결정 0020 |
| REQ-004 | `get_params()` / `set_params(changes)`: 값마다 적용 시점(바로/다음 동작부터/재연결)을 돌려줌. 프리셋 저장·불러오기 `config/presets/<이름>.yaml` | 기획 → 팔 | 추가 | 완료 | #6 합쳐짐 | gui-spec 파라미터 페이지 |
| REQ-005 | `so101 gui` 명령: CLI에 `gui` 하위 명령을 등록해 `so101_studio.gui.main(args)`를 부름. GUI 의존성이 없으면 설치 안내 | 기획 → 팔 | 추가 | 완료 | #5 합쳐짐 | 그 전에는 `python -m so101_studio.gui` |
| REQ-006 | 진단을 함수로: `servoinfo`, `linkwatch`, `powertest`, `shaketest`를 결과 dict, 진행 콜백, 중단과 함께 부를 수 있게 | 기획 → 팔 | 추가 | 완료 | #6 합쳐짐 | 장치 > 팔 페이지 |
| REQ-007 | 작업 이벤트 상세: `TaskEvent.detail`에 스캔한 물체 목록, 고른 물체와 이유, 계산한 경로 시간을 담음. `Studio.bus` 구독 방법 문서화 | 기획 → 팔 | 추가 | 완료 | #4 합쳐짐 | 작업 공간 진행 막대, 이벤트 기록 |
| REQ-008 | `preview_path(pose)`: 움직이지 않고 관절 궤적 샘플과 손끝 경로(작업대 좌표 점 목록)를 돌려줌 | 기획 → 팔 | 추가 | 완료 | #5 합쳐짐 | 3D 경로 선. 지금 `preview_tool`은 끝 관절값만 |
| REQ-009 | `ArmService.resume()`: stop·estop 뒤 상태를 idle로 되돌림(팔은 안 움직임) | GUI → 팔 | 추가 | 반영됨 | #8 합쳐짐 | E-STOP 뒤 "정지 해제" 버튼. fault에서도 idle로 돌아옴 |
| REQ-010 | `test_req005_gui_command_dispatches_or_explains`가 GUI 모듈이 있으면 실패함: 앞부분을 `sys.modules["so101_studio.gui"] = None`으로 바꿈 | GUI → 팔 | 테스트만 | 반영됨 | #8 합쳐짐 | GUI PR #7보다 먼저 합쳐야 함. dev/gui-state에서 통과 확인 |
| REQ-011 | E-STOP 감시 프로세스 `python -m so101_studio.watchdog`와 정지 수신 스레드 `StopListener` (결정 0026, 0026 본문의 REQ-009는 이 번호) | 기획 → 팔 | 추가 | 반영됨 | #13 합쳐짐 | gui-spec 8절 감시 프로세스. G2b |
| REQ-012 | `ArmService.jog`가 토크가 꺼져 있으면 먼저 켬(이미 켠 뒤에는 버스를 읽지 않음) | GUI → 팔 | 추가 | 반영됨 | #17 합쳐짐, GUI #24 | 직접 조작 페이지 조그. 10Hz 호출 |
| REQ-013 | `Studio.pause()`/`resume()`와 `start`에서 step 신호 지우기 | GUI → 팔 | 추가 | 반영됨 | #19 합쳐짐, GUI #24 | 작업 공간 일시정지·한 단계 버튼. 기존 `stop()`이 남긴 신호로 첫 단계가 지나가던 문제 |
| REQ-014 | `MotionController.close()`가 진행 중인 동작의 `MotionHandle`을 끝냄(status stopped) | GUI → 팔 | 추가 | 반영됨 | #25 합쳐짐, GUI 확인 | 동작 중 앱 종료 때 wait()하던 스레드가 남아 프로세스가 안 꺼지던 문제. 같은 PR에서 린트(test_teleop_teach RUF059) 수정 |
| REQ-015 | 다른 PC·다른 팔 브링업: 팔 파일·local.yaml 설정 세 겹, 포트 자동 찾기·find-port, so101 doctor, 캘리브레이션 불일치 잠금, environment.yml, 런처 (결정 0027) | 기획 → 팔 | 설정 파일 위치 변경 | 반영됨 | #32 합쳐짐, GUI #33 (실제 팔 잠금·doctor 탐침은 G10에서 확인) | GUI 장치 > 팔 점검 카드가 doctor 결과 dict를 쓴다 |
| REQ-016 | SO-101 자동 캘리브레이션: LeRobot PR #4826 엔진을 가져와 `so101 calibrate --auto --measure-only`와 `--save` (결정 0028) | 기획 → 팔 | 새 명령 | 완료(코드, 실물 시험 AC4는 사용자와) | dev/arm-autocal | 실물 시험 전에는 `--save`를 쓰지 않는다 |
| REQ-018 | 블랙박스 상태·이벤트 기록: `status.jsonl`(10Hz)·`events.jsonl`(공통 머리), 레지스터 쓰기·토크·fault·예외 기록, ARM-TORQUE-LOST, 사고 묶음과 `so101 incident`, 감시 `--run-dir`, 보관 정리 (결정 0030) | 기획 → 팔 | 기록 형식 변경(결정 0030 승인) | 완료(코드·시험·시뮬 성능 비교, 실제 팔 비교는 사용자와 나중에) | #45 합쳐짐 | G11이 기다림. 실제 팔 성능 비교와 토크 풀림 재현은 사용자와 함께 나중에 |
| REQ-019 | 안전 보완: 작업대 XY 복귀 허용, 통신 끊김 뒤 다시 연결 규칙, 실물 측정 (결정 0031) | 기획 → 팔 | 기존 동작 변경(XY 검사 예외, 결정 0031 승인) | 진행(코드·시험 완료 PR 전, 3번 실물 측정은 사용자와 COM3으로 대기) | dev/arm-safety-comm | 1·2번 코드, 4번은 workspace.yaml 그대로 |
| REQ-020 | 그리퍼 예압·조임 한계(RAM)·no_object 때 열기·부하 두 단계·놓기 보정 (결정 0033) | 기획 → 팔 | 기존 동작 변경(결정 0033 승인) | 진행(코드·시뮬 시험 완료, 실물 집기·놓기 5회와 부하 기준 실측은 사용자와 대기) | dev/arm-grip-preload | 과부하 즉시 추가 읽기와 감시 --run-dir 갱신은 사용자 답 뒤 따로 |
| REQ-021 | 오프라인 준비: 끊긴 뒤 E-STOP(예외 없음·comm_lost 유지), connect가 Kinematics 재사용, MotionResult.reason, linkwatch 포트 | GUI → 팔 | 1번은 결정 0031 구현 보완(기획 확인 필요), 2~4번은 추가 | 진행(코드·시험 완료 PR 전) | dev/arm-offline-prep | 과부하 추가 읽기·감시 --run-dir 갱신은 사용자 답 대기 |
| REQ-022 | 과부하 즉시 추가 읽기(ARM-LOAD-PROBE)와 감시 기록 폴더 갱신(POST /arm의 run_dir) | GUI → 팔(사용자 승인) | 추가 | 진행(코드·시험 완료 PR 전) | dev/arm-load-probe | 기록·감시 보강, 안전 동작 변경 없음 |
