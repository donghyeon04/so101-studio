# 0064. 시험은 네 층으로 나누고, 병렬·동적 포트·바뀐 곳 시험으로 줄인다

상태: 확정 · 2026-11-14 (사용자 승인, `researching-references`로 후보 12개 비교. 0046을 대체)

## 맥락
전체 `python -m pytest -q`가 1시간 38분(1192개)이 됐다. GUI 시험이 고정 포트라 사용자 GUI·감시 프로세스가 포트를 쥐면 119개가 줄줄이 실패했고, 화면 비교는 보드 연결·로컬 교시 기록에 따라 흔들렸다. 같은 묶음을 하루에 여러 번 다시 돌렸다.

## 결정
1. **네 층**:
   - 빠른 묶음 `python -m pytest -q -m "not slow and not measure"`(목표 5분 안): 각 세션이 수시로.
   - 바뀐 곳 시험 `pytest --testmon`: PR 확인. 공유 코드(core·service)를 바꿨거나 testmon 데이터가 없으면 빠른 묶음 전체.
   - 전체 `python -m pytest -q -n auto -m "not measure"`(pytest-xdist, GUI 시험은 `--dist loadgroup`으로 묶음): 하루 마칠 때·실물 시험 전에 총괄이 한 번.
   - 화면 비교(`tests/gui_visual`): GUI를 바꾼 PR과 배포 전에만. 보드를 뽑고, 보드·로컬 기록에 따라 바뀌는 칸은 가리고(mask), 작은 차이 허용(`maxDiffPixelRatio` 약 0.01).
2. **고정 포트를 없앤다**: 시험의 웹·viser·감시 포트는 0(OS가 빈 포트를 고름)으로 받는다. 포트 0을 못 받는 곳은 xdist 워커마다 포트 범위를 쓴다.
3. **GUI 시험은 NiceGUI `user` 픽스처를 기본**으로 하고, 브라우저가 꼭 필요한 시험만 남긴다.
4. **허수 시험 정리**: `--durations=50`과 testmon 데이터로 같은 코드를 보는 시험을 찾는다. 시뮬레이션 전체를 돌려 한 값만 보는 시험은 계획 함수의 단위 시험으로 바꾼다. 실시간 측정(예: `test_realtime_60s`)은 `measure` 표시로 전체에서도 빼고 필요할 때만 돌린다. REQ 번호 회귀 시험 파일(`test_req0xx`)은 주제별 파일로 합친다.
5. **MuJoCo 시험**은 되도록 `realtime=False` + `advance`로 돌리고, `ArmService` 위 시험은 M5b(시뮬레이션 시간 제어)와 함께 빠르게 한다.
6. 보고에는 지금처럼 실행한 명령과 결과(통과 수, 실패한 시험 이름)를 적는다. 일은 총괄(포트·GUI·화면 비교·xdist 설정)과 각 세션(자기 영역의 느린 시험 정리)이 나눠 PR로 한다.
