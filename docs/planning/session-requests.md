# 세션 간 요청 대장

상태: 초안 · 갱신 2026-11-10

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
| REQ-017 | 자동 캘리브레이션 반복 합격과 0° 자세 확인: `calibrate --save`(중앙값, `yes`), `--verify-zero`, 확인 기록이 없으면 `locked` (결정 0029) | 기획 → 팔 | 기존 동작 변경(지금 팔도 확인 전까지 잠김, 결정 0029 승인) | 완료(2026-11-10) | PR #41 합쳐짐(91d9a1a), AC4 저장·0° 확인 #226 | GUI 0° 카드의 필드 없는 분기는 지웠다(doctor가 늘 zero_check를 준다) |
| REQ-018 | 블랙박스 상태·이벤트 기록: `status.jsonl`(10Hz)·`events.jsonl`(공통 머리), 레지스터 쓰기·토크·fault·예외 기록, ARM-TORQUE-LOST, 사고 묶음과 `so101 incident`, 감시 `--run-dir`, 보관 정리 (결정 0030) | 기획 → 팔 | 기록 형식 변경(결정 0030 승인) | 완료(코드·시험·시뮬 성능 비교, 실제 팔 비교는 사용자와 나중에) | #45 합쳐짐 | G11이 기다림. 실제 팔 성능 비교와 토크 풀림 재현은 사용자와 함께 나중에 |
| REQ-019 | 안전 보완: 작업대 XY 복귀 허용, 통신 끊김 뒤 다시 연결 규칙, 실물 측정 (결정 0031) | 기획 → 팔 | 기존 동작 변경(XY 검사 예외, 결정 0031 승인) | 진행(코드 합쳐짐, 실물 측정 대기) | #50 합쳐짐, GUI #53 | `measurements.md` 실물 시험 순서 2·4번 |
| REQ-020 | 그리퍼 예압·조임 한계(RAM)·no_object 때 열기·부하 두 단계·놓기 보정 (결정 0033) | 기획 → 팔 | 기존 동작 변경(결정 0033 승인) | 진행(코드 합쳐짐, 실물 측정 대기) | #52, #55 합쳐짐, GUI #56 | `measurements.md` 실물 시험 순서 5번. 측정 전에는 턱 사이에 손이나 물건을 넣지 않는다. 과부하 추가 읽기와 감시 기록 폴더는 REQ-022로 처리 |
| REQ-021 | 오프라인 준비: 끊긴 뒤 E-STOP(예외 없음·comm_lost 유지), connect가 Kinematics 재사용, MotionResult.reason, linkwatch 포트 | GUI → 팔 | 1번은 결정 0031 구현 보완(기획 확인 #57), 2~4번은 추가 | 반영됨 | #58 합쳐짐, GUI #60 | 가짜 드라이버로만 시험. 실제 USB는 실물 시험 순서 2번 |
| REQ-022 | 과부하 즉시 추가 읽기(ARM-LOAD-PROBE)와 감시 기록 폴더 갱신(POST /arm의 run_dir) | GUI → 팔(사용자 승인) | 추가 | 반영됨 | #59 합쳐짐, GUI #60 | 옛 코드의 감시가 떠 있으면 폴더를 무시한다. 실물 시험 전에 끈다 |
| REQ-023 | `ArmService.follow_path(points_mm, tool_z_mm, speed_mm_s) -> list[PoseSample]`(손끝 수직 아래, 50Hz 실제 관절값으로 팔 센서 위치·시각), 팔 링크 점 작업대 xy 함수, 손끝 55mm 확인 높이 도달 확인 | 기획 → 팔 | 추가 | 완료 | #73 합쳐짐 | `ultrasonic-spec.md` 8.2절, 9절 U3. 결정 0034. 100mm/s, 손끝 xy + `yaw_deg`, `PoseSample.tool_mm`(#70). 도달 범위 자르기는 #71, 감속은 #77. 실물 미확인: 저크 미제한, `read_ms_max` |
| REQ-024 | 초음파 화면: 레이더 부채꼴·각도별 값·후보·지운 각도, 확인 경로와 높이 점, 결과(사각형+방향), 스캔 그래프(E2), 기준값 다시 재기 | 기획 → GUI | 추가 | 진행(가상 값으로 화면 완료, 실제 연결은 REQ-025 뒤) | #83 | `ultrasonic-spec.md` 8.2절 이벤트, 9절 U4. `us.*` 이벤트로 그린다. 벤치마크 `run_cycle`을 빌려 쓰는 `Demo`(shortcut)는 REQ-025가 합쳐질 때 `Studio.bus` 구독으로 바꾸고 지운다 |
| REQ-026 | 읽기 실패 때 동작 명령이 돌아오지 않는 문제: `_tick` 끝 처리에서 `_final_tool`이 실패해도 handle을 끝낸다 | GUI → 팔(사용자 최우선 지시) | 버그 수정 | 완료 | #78 합쳐짐 | #73 접근 이동에서 재현, `main`의 다른 동작은 코드로 본 것이라 재현 필요. `main`에서 재현 후 수정. 전체 시험 624 통과, 실물 미확인. 움직이는 중 읽기 실패는 궤적 끝에서야 fault가 나고, `read_joints`만 실패하는 부분 고장은 fault까지 가지 않는다 |
| REQ-027 | 잠긴 팔에는 어떤 명령도 토크를 켜지 않는다(`ArmService.enable_torque`). `follow_path`도 같은 입구로 | GUI → 팔(총괄 판단) | 버그 수정 | 완료 | #85, #89 합쳐짐 | 팔 파일 없음 잠금에서 토크가 켜지던 문제 |
| REQ-028 | ESP32 센서 보드 페이지: 포트와 연결 상태, 펌웨어 버전, 센서 2개와 서보 상태, 시계 맞추기 오차, 최근 JSON 메시지 | 기획 → GUI | 추가 | 완료(가상 ESP32로 확인, 실제 보드는 B1 때) | #99 합쳐짐
| REQ-025 | 작업 실행 `CHECK` 상태(`CheckableSource`면 실행), 놓는 동안 레이더 미리 돌리기(`prefetch`), `run --source ultrasonic` | 기획 → 팔 | 기존 동작 변경(상태 추가, 결정 0034 승인) | 완료(radar_clear 결정 0039·0041, #107 흐름 포함) | #109 합쳐짐
| REQ-029 | 팀원 배포: `설치.cmd`(conda 없으면 Miniforge를 영문 경로에 무인 설치, git도 conda로, `environment.yml`로 환경 만들기·갱신), `시뮬레이션 실행.cmd`·`실물 팔 실행.cmd`(켤 때마다 `git pull` 뒤 `so101.cmd [--sim] gui`), `so101.cmd`가 `C:\miniforge3` 등 설치 위치를 찾게 수정, README 설치 안내 | GUI → 팔(사용자 승인) | 추가 | 완료(새 PC 무인 설치는 사용자가 시험) | #110 합쳐짐
| REQ-030 | `plan_goto` 호버·목표 IK 여러 시드(radar_clear 출발 실패), Place 턱 가운데 보정(사용자 요청, 결정 0033 범위), 끊길 때 부하 확인 스레드 예외 처리 | GUI → 팔 | 버그 수정 + 기존 동작 보완 | 완료 | #117 합쳐짐 | 초음파 조사(#108), 실물 확인(2026-11-06), `gui-dev-status.md` 인수인계 |
| REQ-031 | 직접 조작 집어 옮기기(gui-spec 11절 B7, G14): `ArmService.pick_and_place`가 놓는 곳으로 배출 구역 이름 대신 작업대 점(x, y)도 받게(REQ-030의 Place 턱 가운데 보정 적용), 집기 범위 밖 점은 이유와 함께 거부 | GUI → 팔(기획 #116) | 추가 | 완료 | #117(팔), #129(GUI G14) 합쳐짐 | `gui-spec.md` 11절 B7 |
| REQ-032 | 실물 시험 결과 통신 고침: Write timeout 뒤 is_using이 남아 영구 "Port is in use"(scservo_sdk), 조그 중 enable_torque 반복, 동작 중 comm_lost 판정 지연(1.01초), 끊긴 동안 LOAD-PROBE 반복, AC4 guard 1회 잘못 읽은 값으로 중단, AC4 시작 자세 안내, table_z_mm 실측 반영, servoinfo 14·15·19·65번 | GUI → 팔(실물 시험 2026-11-06) | 버그 수정 | 완료 | #132 합쳐짐 | `measurements.md` 2026-11-06 절들 |
| REQ-033 | Place 들어 올리기 버그(연 뒤 원래 하강 경로를 거꾸로 가며 옆으로 되돌아가 물체를 밂), 집기 간섭 대응 시험값 grasp_height_mm 20·grasp_max_open_mm 90 | GUI → 팔(사용자 실물 확인) | 버그 수정 + 설정 | 완료 | #133 합쳐짐 | 58×58×39mm 물체 집기 5회 중 2회 실패, 팔 세션 간격 계산 |
| REQ-034 | 결정 0042 팔 몫: `DetectedObject.height_mm`와 높이에 맞춘 집는 높이, `gripper_speed_pct`(바로 적용), 교시 재생 변환본·원본 선택, fault 상세(관절·값·한도·이유 문장), 작업대 설정 다시 읽기 API | GUI → 팔(결정 0042) | 추가 | 완료 | #128·#135(팔), #150(GUI) 합쳐짐 | |
| REQ-035 | 결정 0042 초음파 몫: 확인 결과 실측 높이(h_mm)를 `DetectedObject.height_mm`에 | GUI → 초음파(결정 0042) | 추가 | 완료 | #130 합쳐짐 | |
| REQ-036 | 놓기 여는 경로(_opening_path)가 실패하면 조용히 그냥 열기로 대체되는 문제(가운데 보정 동작 빠짐), 기울인 V자 턱 끝이 책상까지 내려가는 간섭(바닥 검사가 실제 턱 끝을 보게, 벌림을 물체 폭에 맞추기) | GUI → 팔(사용자 실물 영상 2026-11-06) | 버그 수정 | 완료 | #137 합쳐짐 | 집기·놓기 5/5 성공 뒤 영상, 시뮬레이션 재현 |
| REQ-037 | 손끝 방향 자유(pitch·yaw)로 좌표 이동·직교 조그, 집기 범위와 움직임 범위 나누기(gui-spec 11절 B6·C9) | GUI → 팔(사용자 요청) | 추가 | 완료 | #143(팔), #154(GUI) 합쳐짐 | 0° 자세 수평 도달 실측 353mm |
| REQ-038 | 결정 0043-1 병렬 그리퍼: `ParallelGripper`(TCP 두 턱 중앙, 보정 0, W ≤ G − 4), `gripper: single_jaw\|parallel` 설정, `config/grippers/<종류>.yaml`, `assets/so101_parallel.urdf` | GUI → 팔(결정 0043) | 추가 | 진행 | #167 검토 | 폭↔서보값 표·접촉 기준은 실물 몫. GUI 선택 화면은 그 뒤 |
| REQ-039 | 결정 0043-2 레이더 서보 OpenRB-150 + XL330: `firmware/openrb_servo/`, `servo.driver`, ESP32 UART 연결, `ultrasonic-spec.md` 3.2·3.4·3.5 갱신안 | GUI → 초음파(결정 0043) | 추가 | 완료 | #165 합쳐짐 | XL330 5V, 12V를 VIN에 넣지 않음. 설계서 갱신은 총괄이 검토 |
| REQ-040 | 결정 0043-4·5 장면 부품: `config/scene.yaml`, `core/scene.py`의 `load_scene`·`ScenePart`(name, attach, T mm, mesh mm, collide, color), `ArmService.scene`, coal로 팔 링크와 작업대 부품 충돌 검사(`scene:<이름>`, margin_mm) | GUI → 팔(결정 0043) | 추가 | 진행 | #168 진행 | GUI는 `arm.scene`을 3D에 그림(링크 부품은 링크를 따라감). STL이 없는 동안 `box_mm` 상자 |
| REQ-041 | 병렬 그리퍼 URDF·메쉬: Robonine 공개 STL(CERN-OHL-P-2.0)을 `assets/`에, `assets/so101_parallel.urdf`(관절 `gripper`, ID 6), `parallel.yaml`의 `urdf:`, 병렬 `lowest_z`를 메쉬로 | GUI → 팔(사용자 승인 2026-11-07) | 추가 | 진행 | WIP 브랜치 | 사용자 부품 CAD는 Autodesk Fusion |
| REQ-042 | `doctor` 서보 온도 주의 55°C → 50°C(GUI 서보 건강 기준과 같게) | GUI → 팔(사용자 승인 2026-11-07) | 수정 | 완료 | #171 합쳐짐 | GUI 기준은 `gui/health.py` LIMITS |
| REQ-043 | 감사 수정(팔): 한쪽 None 허용 상자 TypeError, 감시 프로세스 스레드 생존·last_tick_age_s, E-STOP ack, 버그와 통신 실패 구분, 정지 쓰기 실패 숨김, 허위·약한 테스트 | GUI → 팔(사용자 승인 2026-11-07) | 수정 | 완료 | #180·#188 합쳐짐 | pr-test-analyzer·silent-failure-hunter 감사 |
| REQ-044 | 감사 수정(초음파): link.py dict 아닌 줄로 스레드 죽음, 포트 닫힘이 안 보임 | GUI → 초음파(사용자 승인 2026-11-07) | 수정 | 완료 | #179 합쳐짐 | |
| REQ-045 | 작업 실행이 배출 구역(drop_zones)을 초음파 `ignore_zones`로 넘김. 놓은 물체가 다음 스캔에서 다시 후보가 되는 버그 | 기획·사용자 → 팔(2026-11-07) | 버그 수정 | 완료 | #185 합쳐짐 | 초음파 실물 I1 전에 필요 |
| REQ-046 | `table_z_mm` −35 → 약 −15: 11-06 실측이 ToolPose 모델 z 명령이었는데 턱 끝 높이로 읽어 잘못 고침. 지금 값이면 집기 높이에서 턱이 바닥에 닿음 | GUI(실물 시험) → 팔(2026-11-07) | 버그 수정(안전) | 완료 | #195 합쳐짐, 실물 확인 37/40/40mm | `measurements.md` "실물 점검과 높이 다시 실측". 집기·2-9a 실물 전에 필요 |
| REQ-047 | 진단(powertest·shaketest)이 끝나거나 중단돼도 쉬는 자세로, 한계 밖 시작은 먼저 home, CLI 종료 때 부하 높으면 경고 | GUI(실물 시험) → 팔(2026-11-07) | 버그 수정(안전) | 완료 | #196 합쳐짐 | 편 자세에 남아 팔이 떨어짐 |
| REQ-048 | 서보 6개 동시 풀림: 동작을 순하게(사용자 방향). 조사 14개 뒤 사용자 승인은 D6(끊김 원인 기록)만 | 사용자 → 팔(2026-11-07) | 기능 | 완료(D6만) | #197 합쳐짐 | D1~D4 미승인, 전원 단계는 hardware.md |
| REQ-049 | 놓을 때 opening path 기록(ARM-OPENING-PATH): 실물에서 고정 턱이 제자리였음 | GUI(실물 시험) → 팔(2026-11-07) | 기록 | 완료 | #198 합쳐짐 | 실물: hand_move 18.76mm, 눈에 안 띔 → REQ-050 |
| REQ-050 | 놓기 동작 모양(arm-software-spec 11.3): 벌림 끝 = 쥔 틈+양쪽 30mm, 1.0초, 다 벌린 뒤 수직 상승, 부딪힘이면 여유 줄임, 실제 턱 이동 기록 | 기획·사용자 → 팔(2026-11-07) | 기능 | 완료 | #204 합쳐짐(시뮬레이션 고정 턱 30/30/26mm, 1.0초) | 뒤에 실물 2-2(놓기 5/5, 고정 턱 20mm 이상). 수동 입력 물체를 `Skills.known_objects`에 넣는 일은 GUI·작업 실행 몫으로 남음 |
| REQ-051 | `so101.cmd gui --share` 인자(store_true, 결정 0047·0049) | GUI → 팔(2026-11-08) | 기능 | 완료 | #209 합쳐짐 | GUI 구현 #212 |
| REQ-052 | 캘리브레이션 마법사용 `autocal.cli.measure_once(cfg, out, stop, runs_dir)`: 입력 대기 없이 측정, `stop`이면 되돌린 뒤 AutoCalibrationError | GUI → 팔(2026-11-10) | 기능 | 완료 | #214 합쳐짐, GUI 진행 표시 #220 | 마법사 1차가 실제 측정을 부른다(실물 측정은 AC4 때) |
| REQ-053 | 대칭 V자 놓기(arm-software-spec 11.3 개정, EGM Stage 9 반영): 놓는 yaw=`_jaw_yaw`, 내려갈 때 기울기 `tilt(지금 그리퍼 값)`·낮은 턱 끝 t_g+3mm, 여는 경로 ①tilted ②released_then_tilted ③fixed_tilt ④경로 없음(FALLBACK은 ④만), 단계별 `check_goal`·물체 검사, 대칭 상한 틈, 기록 키, 새 설정 `place_drop_mm`·`place_release_mm` | 기획·사용자 → 팔(2026-11-10) | 기능 | 완료(2026-11-11, sim까지. 실물 runbook 2-2 대기) | #229 합쳐짐(9259a9c) | 완료 기준 11.3.4(`--sim` 먼저, 그다음 runbook 2-2). REQ-050을 대체 |
| REQ-054 | 관절 한계를 새 캘리브레이션 실제 범위로 넓힘: `physical_limits_deg`는 `calibrate --save` 제안값(pan ±121.3, lift ±105.7, elbow ±98.7, wrist_flex ±104.6, wrist_roll ±169.8, gripper ±65.4), `joint_limits_deg`는 그보다 1° 안쪽, elbow 위쪽 95는 유지 | 사용자 → 팔(2026-11-11) | 설정 변경 | 요청 | - | 같이 정해진 것: 배출 구역은 집기 범위 전체(반경 90~250mm)에서 바꿀 수 있게 한다. 대칭 V자는 sim 기준 반경 약 200mm까지(넓힌 손목 한계면 폭 30mm는 225mm까지), 더 먼 곳의 넓은 물체는 ③fixed_tilt를 구조 한계로 받아들인다. 사용자 결정은 리뷰 준비 세션이 전함 |
| REQ-055 | 팔 방향과 어긋난 물체 집기 보완: 기울인 계획이 안 풀리면 수직으로 다시 시도, CLI `pick`에 물체 높이 인자, 높이를 모를 때의 처리는 [기획 요청] | 리뷰 준비(사용자 지시) → 팔(2026-11-11) | 기능 | 요청 | - | sim 근거: 기울이지 못하는 자세에서 높이를 알면 384건 중 381건 ok, 모르면 127건이 위 모서리를 친다. 기울여 집기는 반경 175mm까지, 그 밖은 IK 실패지만 수직은 풀린다. GUI는 수동 입력·집어 옮기기에서 높이를 이미 받는다(안내 문구 보강). `config/manual_objects.yaml`에는 높이가 없다(`height_mm`을 적을 수 있다) |
