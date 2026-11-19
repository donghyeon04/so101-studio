"""MuJoCo GUI (gui-spec 12.8 시험 1~7). 실시간 드라이버라 slow다."""
import argparse
from pathlib import Path

import pytest

pytest.importorskip("mujoco")

from so101_studio.gui import app as gui_app
from so101_studio.gui import backend as backend_mod
from so101_studio.gui.backend import OBJECTS_LOCKED_TEXT, SAVED_RESTART_TEXT, Backend

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def mj():
    b = Backend(log_root=None, viser_port=8193, sim_engine="mujoco")
    yield b
    b.close()


def test_mujoco_pick_without_object_fails_and_does_not_place(mj, monkeypatch):
    placed = []
    monkeypatch.setattr(mj.sim, "place", lambda **kw: placed.append(kw))
    assert mj.pick_place((215, 250), 0, 30, 30, None, height_mm=40) == "물체를 쥐지 못했습니다"
    assert placed == []
    mj.sim.home()


def test_mujoco_is_not_dry_run(mj):
    assert mj._physical() and not mj._no_object_ok(mj.sim)
    assert mj.studio.task.accept_no_object is False
    assert mj.readout()["팔"] == "시뮬레이션(MuJoCo)"


def test_mujoco_objects_cannot_be_edited(mj):
    before = mj.objects()
    oid = before[0].id
    for call in (lambda: mj.add_object(200, 200), lambda: mj.move_object(oid, 200, 200), lambda: mj.nudge_object(oid, 5, 0),
                 lambda: mj.set_object_height(oid, 30), lambda: mj.remove_object(oid), mj.reset_objects):
        with pytest.raises(ValueError, match="MuJoCo 시뮬레이션은 물체를"):
            call()
    assert mj.objects() == before
    assert OBJECTS_LOCKED_TEXT.startswith("MuJoCo")


def test_mujoco_objects_drawn_from_physics(mj):
    mj._on_sim_objects(None)
    state = mj.sim.driver.sim_state()["objects"]["obj-1"]
    node, _ = mj.scene._sim_nodes["obj-1"]
    x, y, _z = node.position
    assert abs(x * 1000 - state["x_mm"]) < 1 and abs(y * 1000 - state["y_mm"]) < 1
    mj.scene.set_objects(mj.objects())  # 계획 쪽을 다시 그려도 물리 물체는 남는다
    assert "obj-1" in mj.scene._sim_nodes and mj.scene._sim_root is not None
    assert mj.scene.plan_opacity == 0.4


def test_mujoco_save_workspace_asks_restart(mj, monkeypatch, tmp_path):
    path = tmp_path / "workspace.yaml"
    path.write_text(Path(mj.workspace_path).read_text(encoding="utf-8"), encoding="utf-8")
    monkeypatch.setattr(mj, "workspace_path", str(path))
    values = {"table_x": 700, "table_y": 450, "base_x": 320, "base_y": 60, "base_yaw": 90, "zone_x": 520, "zone_y": 170, "margin": 12}
    assert mj.save_workspace(values, "right") == SAVED_RESTART_TEXT


def test_physics_failure_locks_moves(mj, monkeypatch):
    monkeypatch.setattr(type(mj.sim.driver), "failed_reason", property(lambda self: "발산"))
    assert mj.move_lock().startswith("물리 시뮬레이션이 멈췄습니다")
    assert mj.pick_place((215, 250), 0, 30, 30, None, height_mm=40).startswith("물리 시뮬레이션이 멈췄습니다")


def test_app_rejects_scenario_and_reports_missing_mujoco(monkeypatch, capsys):
    with pytest.raises(SystemExit, match="manual_objects"):
        gui_app.main(argparse.Namespace(sim=True, sim_engine="mujoco", sim_scenario="carry58"))
    with pytest.raises(SystemExit, match="--sim과 함께"):
        gui_app.main(argparse.Namespace(sim=False, sim_engine="mujoco"))
    with pytest.raises(SystemExit, match="--sim-viewer는"):
        gui_app.main(argparse.Namespace(sim=True, sim_viewer=True))

    def missing(*_a, **_k):
        raise ImportError("mujoco")

    class FakeLink:
        def __init__(self, sim, port=None, stop_port=None):
            pass

        def stop(self):
            pass

    monkeypatch.setattr(gui_app.port_check, "check_ports", lambda tcp, udp, url: None)
    monkeypatch.setattr(gui_app, "WatchdogLink", FakeLink)
    monkeypatch.setattr(backend_mod, "make_sim_driver", missing)
    with pytest.raises(SystemExit) as e:
        gui_app.main(argparse.Namespace(sim=True, sim_engine="mujoco", viser_port=8194))
    assert e.value.code == 2 and "MuJoCo가 설치되지 않았습니다" in capsys.readouterr().out


def test_rebuild_world_puts_the_training_layout_into_mujoco(mj):
    """mujoco-spec 13.10.4 세계 다시 짓기(결정 0071 시연): 학습 묶음의 씨앗 배치로 MuJoCo 물체와 수동 물체 목록이 같이 바뀐다."""
    try:
        lay = mj.rebuild_world("collect_pick2", 10000)
        (box,) = lay.objects
        objs = mj.sim.driver.sim_state()["objects"]
        assert len(objs) == 1 and abs(next(iter(objs.values()))["x_mm"] - box.x_mm) < 2.0
        assert [(round(o.x_mm), round(o.y_mm)) for o in mj.objects()] == [(round(box.x_mm), round(box.y_mm))]
        assert mj.sim.skills.placed == [] and mj.sim.skills.grip is None
    finally:
        # 다음 시험을 위해 처음 물체로: 수동 물체 파일대로 다시 짓는다
        from so101_studio.arm.mujoco_model import SimObject
        from so101_studio.perception.manual import ManualSource

        mj.sim.driver.set_objects([SimObject(o.id, o.x_mm, o.y_mm, o.angle_deg or 0.0, o.width_mm, o.length_mm, o.height_mm) for o in ManualSource().scan().objects])
        mj.sim.connect(fresh=True)


def test_rebuild_world_refuses_without_mujoco(backend):
    with pytest.raises(ValueError, match="MuJoCo"):
        backend.rebuild_world("collect_pick2", 10000)
