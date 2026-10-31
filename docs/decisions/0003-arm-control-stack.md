# 0003. 로봇팔 제어 스택

상태: 확정 · 2026-10-31

## 맥락
SO-101 팔로워 암(STS3215 6개)을 Python으로 움직여야 한다. 필요한 것은 관절 제어, 역기구학(IK), MoveJ/MoveL/MoveP 같은 궤적 생성이다. 개발 PC는 Windows다.

## 검토한 대안
13개 후보를 채점했다. 상위 후보는 LeRobot `SOFollower` + `RobotKinematics`(93점), so101-control(84점), phosphobot(83점), placo 직접 사용(78점)이다. MoveIt 2 + lerobot_ws(73점)는 실기 하드웨어 인터페이스가 미완성이고 ROS 2 학습 비용이 커서 제외했다.

## 결정
- 관절 제어는 LeRobot `SOFollower`를 쓴다.
- IK는 LeRobot `RobotKinematics`(placo 기반)를 먼저 쓴다. placo는 PyPI에 Windows 휠이 없으므로, 첫 로컬 세션에서 설치를 확인한다. 설치되지 않으면 WSL2 + usbipd-win 또는 Pinocchio(conda-forge win-64)로 바꾸고 새 결정 기록을 남긴다.
- MoveJ(관절 보간), MoveL(직교 직선), MoveP(경유점 블렌딩)는 Robotics Toolbox for Python의 `jtraj`, `ctraj`, `mstraj` 공식을 참고해 얇은 래퍼로 직접 만든다.
- 동역학은 STS3215 내부 PID에 맡기고, 토크·전류·온도는 상태 모니터링에만 쓴다.

근거: LeRobot 공식 문서(Context7), SO-101 URDF(`so101_new_calib.urdf`), placo PyPI, Robotics Toolbox for Python 문서.
