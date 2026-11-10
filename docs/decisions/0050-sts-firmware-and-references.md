# 0050. STS32XX 대체 펌웨어는 보류, 전원·시뮬레이션·서보 레지스터 레퍼런스 채택

상태: 확정 · 2026-11-10 (사용자 승인, `researching-references`로 후보 12개 비교)

## 맥락
사용자가 roboninecom/STS32XX-Firmware(GD32F130 STS3215용 대체 펌웨어, Apache-2.0, S자 가감속·PID·속도 피드포워드·중력 보상)를 찾았다. 우리 문제는 큰 동작 끝 서보 6개 동시 리셋(전원 처짐 추정, hardware.md "팔 전원 경로 점검과 보강")이다.

## 결정
1. R1: STS32XX-Firmware는 지금 굽지 않는다. 굽기 위험(서보 손상·캘리브레이션 삭제, 굽는 중 전원 끊김 금지), 우리 서보의 GD32F130 여부 미확인, 확장 모드는 드라이버의 프로토콜 v9 지원이 필요, 리셋 문제와 무관해서다. 전원 보강 확인 시험을 통과한 뒤 예비 서보 하나에 legacy 모드부터 다시 본다.
2. R2: 전원 보강 근거에 TheRobotStudio/SO-ARM100 이슈 #132(12V 5A 데이지체인 부족)를 더하고, 어깨 쪽 서보 전원 따로 넣기(급전 분기)를 보강 후보로 둔다.
3. R3: MuJoCo(결정 0044 순서 마지막)는 google-deepmind/mujoco_menagerie의 `robotstudio_so101` 모델에서 시작한다.
4. R4: 서보 상태·보호 레지스터 해석은 Feetech 공식 ftservo/FTServo_Python을 참고로 쓴다(리셋 판정 `arm/reset_probe.py` 확인용).
- 쓰지 않는 것: LeLab·phosphobot(GUI와 겹침), XLeRobot(다른 로봇), rerun(기능 11번 뺌), ruckig(실측 떨림 증가).
