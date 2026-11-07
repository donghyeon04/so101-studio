# 병렬 그리퍼 메쉬 (REQ-041, 결정 0043-1)

상태: 확정

- 출처: https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper (`simulation/so_arm_101_description/meshes/visual/`), 커밋 시점 2026-11-07.
- 파일: `parallel_hand.stl`(= link5_1.stl, 손목 롤 출력과 본체가 한 몸, **Apache-2.0**), `clamp_right.stl`(= clamp_1.stl)·`clamp_left.stl`(= clamp_2.stl) (Robonine 집게 형상, **CERN-OHL-P-2.0**). 단위 mm(URDF에서 scale 0.001).
- `assets/so101_parallel.urdf`: `so101_new_calib.urdf`에서 gripper_link 아래만 바꿨다. 관절 `gripper`(서보 ID 6)는 오른쪽 클램프의 prismatic(0 = 닫힘, m), 왼쪽 클램프는 `gripper_left`가 mimic -1. `gripper_frame_link`(TCP)는 두 턱 중앙(손끝 10mm 위).
- 좌표 변환 (Robonine 시뮬레이션 URDF는 메쉬를 영점 세계 좌표에 두고 링크 원점만 옮긴다):
  p_이쪽 = M (p_저쪽 - o5), M = Rx(90°), o5 = link5 원점 (0.007062, -0.120627, 0.152066) m. 롤 축 y -> z, 손끝 방향 -y -> -z, 열리는 축 x -> x.
  손끝 접촉면은 롤 축에서 +0.44mm(x)에서 만난다.
- 확인 안 된 것: 폭<->관절 표(간격 = 2 x 관절 m), 손끝 패드 위치, TCP 높이는 실물에서 재서 `config/grippers/parallel.yaml`을 고친다. 사용자가 Fusion에서 고친 모델이 생기면 바꾼다.
