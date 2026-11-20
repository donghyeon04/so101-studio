"""REQ-113(피드백 F5): 교시 재생 때 기록 그리퍼가 지금과 다르면 거부(--any-gripper로 무시), 기록에 그리퍼가 없으면(옛 기록) 경고만."""
import json

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.service.arm_service import ArmService

CFG, WS = load_robot("config/robot.yaml", gripper="parallel"), load_workspace("config/workspace.yaml")


def svc_with(tmp_path, meta):
    s = ArmService(SimDriver(CFG.poses["home"]), CFG, WS, speed_pct=100, log_root=str(tmp_path / "runs"))
    assert s.connect()[0]
    s.teach.dir = tmp_path / "seq"
    s.teach.dir.mkdir()
    h = list(CFG.poses["home"])
    q = [[h[0] + k * 0.5, *h[1:]] for k in range(30)]  # 홈에서 pan만 천천히 15°
    (s.teach.dir / "r.json").write_text(json.dumps({"hz": 20, "t": [k / 20 for k in range(30)], "joints": q, **({"meta": meta} if meta is not None else {})}), encoding="utf-8")
    ev = []
    s.ctl.on_event = lambda kind, level, code, data: ev.append(code)
    return s, ev


def test_other_gripper_recording_is_refused_without_moving_and_any_gripper_plays_it(tmp_path):
    s, _ = svc_with(tmp_path, {"gripper": "single_jaw"})
    try:
        q0 = s.driver.read_joints()
        r = s.replay("r")
        assert r.status == "stopped" and r.reason.startswith("gripper_mismatch") and "--any-gripper" in r.reason
        assert s.driver.read_joints() == q0
        assert s.replay("r", any_gripper=True).status == "ok"
    finally:
        s.disconnect()


def test_same_gripper_plays_and_unknown_gripper_only_warns(tmp_path):
    s, ev = svc_with(tmp_path, {"gripper": "parallel"})
    try:
        assert s.replay("r").status == "ok" and "ARM-REPLAY-GRIPPER-UNKNOWN" not in ev
    finally:
        s.disconnect()
    s, ev = svc_with(tmp_path / "old", None)  # meta 없는 옛 기록 = 모름
    try:
        assert s.replay("r").status == "ok" and "ARM-REPLAY-GRIPPER-UNKNOWN" in ev
    finally:
        s.disconnect()
