"""3D 장면 (gui-spec G3). 브라우저 없이 서버 쪽 장면 값을 본다."""

import asyncio
import math
import time
from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("viser")

from nicegui.testing import user_simulation

from so101_studio.arm.kinematics import ARM_JOINTS
from so101_studio.core.frames import robot_to_table
from so101_studio.core.types import DetectedObject, JointState
from so101_studio.gui import app as gui_app
from so101_studio.gui.backend import Backend
from so101_studio.gui.scene3d import urdf_cfg


@pytest.fixture(scope="module")
def backend3d():
    backend = Backend(log_root=None, viser_port=8191)
    yield backend
    backend.close()


def _wait_for(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.02)
    return False




def test_urdf_cfg_follows_urdf_joint_order_and_gripper_mapping():
    q = JointState(10, 20, 30, 40, 50, 0)
    got = urdf_cfg(q, ("gripper", "wrist_roll", "shoulder_pan"))
    assert got == pytest.approx([math.radians(-10), math.radians(50), math.radians(10)])


def test_urdf_cfg_parallel_gripper_is_half_gap_in_meters():
    """REQ-041: 병렬 URDF의 gripper는 직선 관절(m). 값 50이면 폭 42mm -> 오른쪽 +0.021, 왼쪽 -0.021."""
    q = JointState(0, 0, 0, 0, 0, 50)
    got = urdf_cfg(q, ("gripper", "gripper_left", "shoulder_pan"), [[0, 0], [100, 84]])
    assert got == pytest.approx([0.021, -0.021, 0.0])


def test_urdf_cfg_covers_every_actuated_joint(backend3d):
    names = backend3d.scene._names
    assert set(names) == set(ARM_JOINTS) | {"gripper"}


def test_tool_frame_follows_status_on_the_same_table_coordinates(backend3d):
    assert _wait_for(lambda: backend3d.state.status is not None)
    st = backend3d.arm.status()
    backend3d.scene.update_status(st)
    t = st.tool
    x, y, z = t.x_mm, t.y_mm, t.z_mm - backend3d.arm.cfg.table_z_mm
    assert np.asarray(backend3d.scene._tool.position) * 1000 == pytest.approx([x, y, z], abs=1e-3)


def test_real_arm_moves_within_a_status_period(backend3d):
    assert _wait_for(lambda: backend3d.state.status is not None)
    q = backend3d.arm.status().joints
    before = np.array(backend3d.scene._tool.position)
    goal = replace(q, shoulder_pan=q.shoulder_pan + (10 if q.shoulder_pan < 0 else -10))
    assert backend3d.arm.move_joints(goal, 100).status == "ok"
    assert _wait_for(lambda: np.linalg.norm(np.array(backend3d.scene._tool.position) - before) > 0.01, 0.2)


def test_urdf_root_sits_where_kinematics_says_the_shoulder_is(backend3d):
    from so101_studio.arm.kinematics import PAN_AXIS_X_MM

    base = backend3d.arm.ws.base
    root = backend3d.scene.server.scene._handle_from_node_name["/real"]
    ox, oy = robot_to_table(-PAN_AXIS_X_MM, 0.0, base)
    assert np.asarray(root.position) * 1000 == pytest.approx([ox, oy, -backend3d.arm.cfg.table_z_mm])


def test_preview_arm_hidden_until_asked(backend3d):
    scene = backend3d.scene
    assert scene._preview_root.visible is False
    scene.set_preview(backend3d.arm.status().joints)
    assert scene._preview_root.visible is True
    scene.set_preview(None)
    assert scene._preview_root.visible is False


def test_objects_are_drawn_and_replaced(backend3d):
    scene = backend3d.scene
    handles = scene.server.scene._handle_from_node_name
    assert [n for n in handles if n.startswith("/objects/")]  # 시작할 때 수동 입력 물체를 그린다
    scene.set_objects([DetectedObject("only", 300, 200, 45, 30, 100, 1.0, "manual")])
    assert [n for n in handles if n.startswith("/objects/")] == ["/objects/only"]
    box = handles["/objects/only"]
    assert np.asarray(box.position) == pytest.approx([0.3, 0.2, 0.015])


def test_viser_binds_to_localhost_only(backend3d):
    assert backend3d.scene.url.startswith("http://127.0.0.1:")
    assert backend3d.scene.server._websock_server._host == "127.0.0.1"


@pytest.mark.parametrize("path,shown", [("/", True), ("/manual", True), ("/safety", True), ("/logs", False)])
def test_scene_iframe_only_on_3d_pages(backend3d, path, shown):
    backend3d.owner = None

    async def go():
        async with user_simulation(gui_app.make_root(backend3d)) as user:
            await user.open(path)
            await user.should_see(marker="page-title")
            if shown:
                await user.should_see(marker="scene3d")
            else:
                await user.should_not_see(marker="scene3d")

    asyncio.run(go())


def test_simplify_keeps_shape_with_far_fewer_faces():
    import trimesh

    from so101_studio.gui.scene3d import simplify

    mesh = trimesh.creation.icosphere(subdivisions=5, radius=0.05, face_colors=(255, 210, 0, 255))
    small = simplify(mesh, 0.005)
    assert len(small.faces) < len(mesh.faces) / 4
    assert small.extents == pytest.approx(mesh.extents, abs=0.005)
    assert tuple(small.visual.main_color) == (255, 210, 0, 255)  # URDF 재질 색을 잃지 않는다
    assert len(simplify(mesh, 1e-6).faces) == len(mesh.faces)  # 격자가 아주 작으면 그대로다


def test_layers_hide_and_show_their_nodes(backend3d):
    scene = backend3d.scene
    try:
        for name, nodes in scene._layer_nodes.items():
            scene.set_layer(name, False)
            assert all(node.visible is False for node in nodes), name
            scene.set_layer(name, True)
            assert all(node.visible is True for node in nodes), name
    finally:
        for name in scene.layers:
            scene.set_layer(name, True)


def test_preview_layer_stays_off_when_a_plan_arrives_and_returns_when_turned_on(backend3d):
    scene = backend3d.scene
    joints = backend3d.arm.status().joints
    try:
        scene.set_layer("preview", False)
        scene.set_preview(joints)
        assert scene._preview_root.visible is False
        scene.set_layer("preview", True)
        assert scene._preview_root.visible is True
        scene.set_preview(None)
        scene.set_layer("preview", True)
        assert scene._preview_root.visible is False  # 계산 자세가 없으면 켜도 보이지 않는다
    finally:
        scene.set_layer("preview", True)
        scene.set_preview(None)


def test_objects_layer_survives_redrawing_the_objects(backend3d):
    scene = backend3d.scene
    try:
        scene.set_layer("objects", False)
        scene.set_objects([])
        assert scene._objects_root.visible is False
        scene.set_layer("objects", True)
        assert scene._objects_root.visible is True
    finally:
        scene.set_layer("objects", True)


def test_scene_parts_on_table_and_on_link(backend3d):
    """결정 0043-4·REQ-040: 작업대 부품은 /parts 아래, 링크 부품은 그 링크 노드 아래(팔을 따라 움직임). 모르는 링크는 건너뛴다."""
    from types import SimpleNamespace

    import trimesh

    box = trimesh.creation.box((80.0, 60.0, 40.0))
    T = np.eye(4)
    T[:3, 3] = (100.0, 200.0, 20.0)
    parts = [SimpleNamespace(name="station", attach="table", T=T, mesh=box, color="#8a8f98"),
             SimpleNamespace(name="bracket", attach="wrist_link", T=np.eye(4), mesh=box, color=None),
             SimpleNamespace(name="ghost", attach="no_such_link", T=np.eye(4), mesh=box, color=None)]
    scene = backend3d.scene
    scene.set_parts(SimpleNamespace(parts=parts))
    nodes = scene.server.scene._handle_from_node_name
    assert "/parts/part_station" in nodes
    assert np.allclose(nodes["/parts/part_station"].position, (0.1, 0.2, 0.02))
    bracket = [n for n in nodes if n.endswith("/wrist_link/part_bracket")]
    assert len(bracket) == 1 and bracket[0].startswith("/real/")
    assert not any("ghost" in n for n in nodes)
    scene.set_parts(None)
    assert not any("part_" in n for n in scene.server.scene._handle_from_node_name)
