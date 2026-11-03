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
| REQ-014 | `MotionController.close()`가 진행 중인 동작의 `MotionHandle`을 끝냄(status stopped) | GUI → 팔 | 추가 | 완료 | dev/arm-req-close | 동작 중 앱 종료 때 wait()하던 스레드가 남아 프로세스가 안 꺼지던 문제. 같은 PR에서 린트(test_teleop_teach RUF059) 수정 |
