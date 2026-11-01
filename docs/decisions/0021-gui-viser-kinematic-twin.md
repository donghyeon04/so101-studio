# 0021. GUI는 Viser 기반 Python 단일 프로세스, 시뮬레이션은 기구학 쌍둥이

상태: 확정 · 2026-11-02

## 맥락
2차 단계(시뮬레이션·GUI)의 기술을 정하려고 레퍼런스 24개를 조사했다(PIPER Studio, LeLab, phosphobot, Viser, NiceGUI, Rerun, MuJoCo 등). SO-101 계열 웹 GUI는 대부분 React와 Python 백엔드로 나뉘어 있지만, 혼자 12/4까지 만들고 GitHub에 공개해야 하는 조건에서는 언어가 둘이 되고 빌드 도구가 늘어나는 부담이 크다. placo의 시각화 모듈은 이미 Viser를 쓴다.

## 결정
- GUI는 Viser(`viser[urdf]`)로 만들고, `ArmService`·`Studio`를 같은 Python 프로세스에서 호출한다. 실행은 `so101 gui`, 화면은 브라우저 localhost에서 연다. 프론트엔드 빌드는 쓰지 않는다.
- 3D 화면에는 ViserUrdf 팔 두 개를 띄운다. 하나는 실제(또는 SimDriver) 자세이고, 다른 하나는 `preview_tool` 결과를 보여 주는 반투명 미리보기 팔이다.
- 시뮬레이션은 기구학 쌍둥이(SimDriver + `assets/so101_new_calib.urdf`)로 한다. 물리 시뮬레이션(MuJoCo, menagerie SO-101)은 이번 단계에서 하지 않는다.
- 안전은 PIPER Studio의 방식을 따른다. 브라우저 연결이 끊기면 `stop()`을 부르고, 비상 정지 버튼은 모든 화면에 항상 보이게 둔다. 버튼은 사용할 수 있는 기능에 맞춰 켜거나 끈다.
- 화면 구성은 UFACTORY Studio를 참고한다(라이브 제어의 관절·TCP 조그, 기록·배속 재생).
- 앱 구성이 Viser로 부족하면 NiceGUI로 바꾼다. 둘 다 Python 단일 프로세스라 `ArmService` 쪽 코드는 그대로 쓴다.
