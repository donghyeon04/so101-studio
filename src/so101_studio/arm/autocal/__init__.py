"""SO-101 자동 캘리브레이션 (decisions/0028, arm-software-spec.md 11.1절).

- engine.py, plan.py: LeRobot PR #4826(Apache-2.0)에서 가져온 엔진과 SO 계획. 엔진은 고치지 않는다(출처와 바뀐 곳은 각 파일 머리말).
- guard.py: 엔진이 쓰는 버스를 감싸 전압·온도를 확인한다(엔진을 고치지 않고 중단 조건을 더하는 곳).
- measure.py: `so101 calibrate --auto --measure-only`. EEPROM에 쓰지 않고 기존 캘리브레이션과 비교한다.
- save.py: `so101 calibrate --save`. 측정만 모드가 합격한 결과만 쓴다.
- cli.py: 명령 연결.
"""
