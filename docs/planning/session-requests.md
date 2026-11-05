# 세션 간 요청 대장

상태: 초안 · 갱신 2026-11-05

절차와 양식은 `docs/guides/session-workflow.md`에 있다. GUI 세션(총괄)이 번호를 매기고 행과 상태를 관리한다(`decisions/0036`). 자세한 내용은 요청 메시지와 PR에 두고, 여기에는 한 줄만 남긴다.

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
| REQ-017 | 자동 캘리브레이션 반복 합격과 0° 자세 확인: `calibrate --save`(중앙값, `yes`), `--verify-zero`, 확인 기록이 없으면 `locked` (결정 0029) | 기획 → 팔 | 기존 동작 변경(지금 팔도 확인 전까지 잠김, 결정 0029 승인) | 진행(코드·시험 완료, 합치기 대기) | PR #41 열림, dev/arm-autocal-repeat | G10 실물 시험이 끝난 뒤 사용자 승인으로 합친다. 합치기 전에 `main`을 다시 받고 전체 시험. GUI는 합쳐진 뒤 0° 카드의 필드 없는 분기를 지운다 |
| REQ-018 | 블랙박스 상태·이벤트 기록: `status.jsonl`(10Hz)·`events.jsonl`(공통 머리), 레지스터 쓰기·토크·fault·예외 기록, ARM-TORQUE-LOST, 사고 묶음과 `so101 incident`, 감시 `--run-dir`, 보관 정리 (결정 0030) | 기획 → 팔 | 기록 형식 변경(결정 0030 승인) | 완료(코드·시험·시뮬 성능 비교, 실제 팔 비교는 사용자와 나중에) | #45 합쳐짐 | G11이 기다림. 실제 팔 성능 비교와 토크 풀림 재현은 사용자와 함께 나중에 |
| REQ-019 | 안전 보완: 작업대 XY 복귀 허용, 통신 끊김 뒤 다시 연결 규칙, 실물 측정 (결정 0031) | 기획 → 팔 | 기존 동작 변경(XY 검사 예외, 결정 0031 승인) | 진행(코드 합쳐짐, 실물 측정 대기) | #50 합쳐짐, GUI #53 | `measurements.md` 실물 시험 순서 2·4번 |
| REQ-020 | 그리퍼 예압·조임 한계(RAM)·no_object 때 열기·부하 두 단계·놓기 보정 (결정 0033) | 기획 → 팔 | 기존 동작 변경(결정 0033 승인) | 진행(코드 합쳐짐, 실물 측정 대기) | #52, #55 합쳐짐, GUI #56 | `measurements.md` 실물 시험 순서 5번. 측정 전에는 턱 사이에 손이나 물건을 넣지 않는다. 과부하 추가 읽기와 감시 기록 폴더는 REQ-022로 처리 |
| REQ-021 | 오프라인 준비: 끊긴 뒤 E-STOP(예외 없음·comm_lost 유지), connect가 Kinematics 재사용, MotionResult.reason, linkwatch 포트 | GUI → 팔 | 1번은 결정 0031 구현 보완(기획 확인 #57), 2~4번은 추가 | 반영됨 | #58 합쳐짐, GUI #60 | 가짜 드라이버로만 시험. 실제 USB는 실물 시험 순서 2번 |
| REQ-022 | 과부하 즉시 추가 읽기(ARM-LOAD-PROBE)와 감시 기록 폴더 갱신(POST /arm의 run_dir) | GUI → 팔(사용자 승인) | 추가 | 반영됨 | #59 합쳐짐, GUI #60 | 옛 코드의 감시가 떠 있으면 폴더를 무시한다. 실물 시험 전에 끈다 |
| REQ-023 | `ArmService.follow_path(points_mm, tool_z_mm, speed_mm_s) -> list[PoseSample]`(손끝 수직 아래, 50Hz 실제 관절값으로 팔 센서 위치·시각), 팔 링크 점 작업대 xy 함수, 손끝 55mm 확인 높이 도달 확인 | 기획 → 팔 | 추가 | 진행(PR 수정 중) | PR #73 열림, dev/arm-req-023 | `ultrasonic-spec.md` 8.2절, 9절 U3. 결정 0034. 100mm/s, 손끝 xy + `yaw_deg`, `PoseSample.tool_mm`(#70). 도달 범위 자르기와 연속 통과는 #71. 읽기 실패 멈춤 등 검토 수정 중 |
| REQ-024 | 초음파 화면: 레이더 부채꼴·각도별 값·후보·지운 각도, 확인 경로와 높이 점, 결과(사각형+방향), 스캔 그래프(E2), 기준값 다시 재기 | 기획 → GUI | 추가 | 진행(가상 값으로 화면 완료, 실제 연결은 REQ-025 뒤) | #83 | `ultrasonic-spec.md` 8.2절 이벤트, 9절 U4. `us.*` 이벤트로 그린다. 벤치마크 `run_cycle`을 빌려 쓰는 `Demo`(shortcut)는 REQ-025가 합쳐질 때 `Studio.bus` 구독으로 바꾸고 지운다 |
| REQ-026 | 읽기 실패 때 동작 명령이 돌아오지 않는 문제: `_tick` 끝 처리에서 `_final_tool`이 실패해도 handle을 끝낸다 | GUI → 팔(사용자 최우선 지시) | 버그 수정 | 완료 | #78 합쳐짐 | #73 접근 이동에서 재현, `main`의 다른 동작은 코드로 본 것이라 재현 필요. `main`에서 재현 후 수정. 전체 시험 624 통과, 실물 미확인. 움직이는 중 읽기 실패는 궤적 끝에서야 fault가 나고, `read_joints`만 실패하는 부분 고장은 fault까지 가지 않는다 |
| REQ-025 | 작업 실행 `CHECK` 상태(`CheckableSource`면 실행), 놓는 동안 레이더 미리 돌리기(`prefetch`), `run --source ultrasonic` | 기획 → 팔 | 기존 동작 변경(상태 추가, 결정 0034 승인) | 예정 | | `ultrasonic-spec.md` 2절, 8.2절, 9절 U8. 추가: RESCAN, `begin_cycle`, PICK 실패 때 `note_failure`, `give_up`이면 DONE(#77). `probe.abort`, 시각별 `arm_links_fn` 연결. #73 합쳐진 뒤 시작 |
