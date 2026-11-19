"""REQ-088(H-2 후속, 결정 0067): URDF base_link의 J1 받침 메쉬가 CAD 받침 몸통 자리에만 있다."""
import numpy as np
import pytest
import trimesh

from so101_studio.arm.kinematics import PAN_AXIS_X_MM
from so101_studio.core.config import load_robot, load_workspace


def test_base_mount_mesh_sits_on_the_profile_behind_the_board():
    """base_mount_h0.stl은 작업대 y -40~0(판 밖 프로파일 위)에만 있다. 작업대 z 24 위의 다른 부품(뒤로 y -59까지 가는 폭 6mm 판)은 넣지 않는다.
    CAD 값(높이 구간마다 0.1mm 안으로 같음): z<0 y -39.9~-0.9, z 0~10 y -39.9~0.9."""
    v = trimesh.load("assets/assets/base_mount_h0.stl", force="mesh").vertices * 1000.0  # base_link mm
    ws, cfg = load_workspace("config/workspace.yaml"), load_robot("config/robot.yaml")
    b = ws.base
    t = np.column_stack([b.x_mm - v[:, 1], b.y_mm + v[:, 0] - PAN_AXIS_X_MM, v[:, 2] - cfg.table_z_mm])  # yaw 90: 로봇 x -> 작업대 +y
    assert t.min(0).tolist() == pytest.approx([192.8, -40.0, -12.4], abs=0.6) and t.max(0).tolist() == pytest.approx([237.5, 1.7, 24.0], abs=0.6)
