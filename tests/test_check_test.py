"""REQ-097: A1 확인 실측(checktest a1)과 I1 정답 기록(run --truth-layout K), ultrasonic-spec 8.6."""
import json
import math
import types

import pytest
from test_task_check import StubArm, StubSource, cand, runner

from so101_studio.arm.sim_driver import SimDriver
from so101_studio.cli import run_checktest, run_task
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.perception.ultrasonic import truth
from so101_studio.perception.ultrasonic.benchmark import edge_gap, on_plate
from so101_studio.perception.ultrasonic.model import Box
from so101_studio.service.arm_service import ArmService
from so101_studio.service.check_test import (
    A1_N,
    a1_plan,
    a1_summary,
    i1_layout,
    i1_summary,
)

CFG = load_robot("config/robot.yaml")


def test_a1_plan_is_seeded_and_every_truth_body_is_on_the_plate():
    p = a1_plan(seed=3, reps=2)
    assert len(p) == 2 * A1_N and [i["n"] for i in p[:A1_N]] == list(range(1, A1_N + 1))
    assert [(i["cand"].x_mm, i["cand"].y_mm) for i in p] == [(i["cand"].x_mm, i["cand"].y_mm) for i in a1_plan(seed=3, reps=2)]
    assert [i["cand"].x_mm for i in p] != [i["cand"].x_mm for i in a1_plan(seed=4, reps=2)]
    ws = load_workspace()
    for i in p:
        t = i["truth"]
        assert math.dist((i["cand"].x_mm, i["cand"].y_mm), (t["x_mm"], t["y_mm"])) <= 30
        assert on_plate(Box(t["x_mm"], t["y_mm"], t["angle_deg"], t["width_mm"], t["length_mm"], t["height_mm"]), ws), i
    assert all(i["truth"]["angle_deg"] is None for i in a1_plan(obj=(50, 50, 40)))


def _rows(errs, angles=None, square=False, rep=1):
    return [{"rep": rep, "status": "ok" if e is not None else "empty", "err_mm": math.inf if e is None else e,
             "angle_ok": None if square else (angles[i] if angles else True), "square": square} for i, e in enumerate(errs)]


def test_a1_summary_verdicts():
    assert "합격" in a1_summary(_rows([3.0] * 11 + [None])) and "불합격" not in a1_summary(_rows([3.0] * 11 + [None]))
    fails = _rows([3.0] * 6 + [None] * 6)  # 실패를 +∞로 넣으면 중앙값이 +∞
    assert "불합격" in a1_summary(fails) and "중앙 -" in a1_summary(fails)
    assert "불합격" in a1_summary(_rows([3.0] * 12, angles=[True] * 9 + [False] * 3))
    sq = a1_summary(_rows([3.0] * 12, square=True))
    assert "합격" in sq and "방향" not in sq
    assert "미완료(5/12회)" in a1_summary(_rows([3.0] * 5))  # q로 일찍 끝남
    stopped = _rows([3.0] * 12)
    stopped[-1] = {**stopped[-1], "status": "aborted", "err_mm": math.inf}
    assert "미완료" in a1_summary(stopped)
    two = a1_summary(_rows([3.0] * 12) + _rows([None] * 12, rep=2))
    assert "1묶음: 합격" in two and "2묶음: 불합격" in two


@pytest.mark.slow  # 배치 하나에 10~40초(random_layout의 모서리 간격 계산)
def test_i1_layouts_1_to_10_keep_show80_rules_after_rounding():
    ws = load_workspace()
    base = (ws.robot_base_in_table["x_mm"], ws.robot_base_in_table["y_mm"])
    for k in range(1, 11):
        boxes = i1_layout(k)
        assert len(boxes) == 3
        assert all(b.x % 5 == 0 and b.y % 5 == 0 and b.angle_deg % 5 == 0 and on_plate(b, ws) for b in boxes), k
        assert all(150 <= math.dist((b.x, b.y), base) <= 220 for b in boxes), k  # show80 반경, 반올림 뒤에도
        assert all(edge_gap(a, b) >= 80 for i, a in enumerate(boxes) for b in boxes[i + 1:]), k


class CountingSource(StubSource):
    connects = 0

    def connect(self):
        self.connects += 1


def test_check_candidate_connects_once_and_skips_reach_filter():
    src, arm = CountingSource([]), StubArm()
    r = runner(src, arm)
    far = cand(1, 2000.0, 2000.0)  # 집기 범위 밖이어도 A1은 확인 결과를 그대로 돌려준다
    assert r.check_candidate(far)[1] == "ok" and r.check_candidate(cand(2))[1] == "ok"
    assert src.connects == 1 and src.calls.count("begin_cycle") == 1 and arm.offset is not None
    r._trail.detach()


def _svc(tmp_path):
    svc = ArmService(SimDriver(CFG.poses["home"]), CFG, load_workspace(), speed_pct=100, log_root=str(tmp_path / "runs"))
    assert svc.connect()[0]
    return svc


def _args(**kw):
    base = {"sim": True, "us_port": None, "obj": "40x80x40", "offset": 30.0, "seed": 0, "reps": 1, "note": "",
            "values": ["a1"], "source": "ultrasonic", "dry_run": True, "step": False, "truth_layout": None}
    return types.SimpleNamespace(**{**base, **kw})


def _one_run(root):
    (d,) = [p for p in root.iterdir() if p.is_dir()]
    return d, json.loads((d / "truth.json").read_text(encoding="utf-8")), truth.index(root)


def test_checktest_refuses_object_taller_than_check_height(tmp_path, monkeypatch):
    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    with pytest.raises(SystemExit, match="max_object_height_mm"):
        run_checktest(types.SimpleNamespace(), _args(obj="40x80x60"))


@pytest.mark.slow
def test_sim_checktest_a1_records_twelve_checks_without_scans(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    svc = _svc(tmp_path)
    try:
        run_checktest(svc, _args())
    finally:
        svc.disconnect()
    d, data, idx = _one_run(tmp_path / "truth")
    pl = data["placements"]
    assert len(pl) == A1_N and all(len(p["checks"]) == 1 and p["scans"] == [] for p in pl)
    assert not (d / "scans").exists()
    cands = [json.loads((d / p["checks"][0]).read_text(encoding="utf-8"))["candidate"] for p in pl]
    assert [(c["x_mm"], c["y_mm"]) for c in cands] == [(i["cand"].x_mm, i["cand"].y_mm) for i in a1_plan(seed=0)]
    assert idx == [{"run": d.as_posix(), "test": "A1", "commit": data["commit"], "objects": A1_N, "placements": A1_N, "sim": True}]
    out = capsys.readouterr().out
    assert "A1 1묶음:" in out and "미완료" not in out, out


@pytest.mark.slow
def test_sim_run_truth_layout_records_answer_and_summary_reads_it(tmp_path, monkeypatch):
    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    svc = _svc(tmp_path)
    try:
        run_task(svc, _args(values=[], truth_layout=1))
        log = svc.logger.dir
    finally:
        svc.disconnect()
    d, data, idx = _one_run(tmp_path / "truth")
    (p,) = data["placements"]
    assert [o["id"] for o in p["objects"]] == ["P1", "P2", "P3"] and p["checks"] and f"log={log}" in data["note"]
    assert [(o["x_mm"], o["y_mm"]) for o in p["objects"]] == [(b.x, b.y) for b in i1_layout(1)]
    assert len(idx) == 1 and idx[0]["test"] == "I1" and idx[0]["objects"] == 3
    s = i1_summary([str(d)])
    assert s.startswith("I1 1회:") and "summary 1/1회" in s, s


# ---- A0 센서 오프셋 보정 (ultrasonic-spec 8.7, REQ-101)
def test_a0_delta_recovers_an_offset_change_from_the_arm_sensor_position(tmp_path):
    """팔 좌표계와 식이 맞는지: ψ = 100° 수직 아래 자세에서 오프셋을 (3, -4, 2) 바꿨을 때 sensor_position 차이로 (3, -4)를 되찾는다."""
    from so101_studio.core.types import ToolPose
    from so101_studio.service.check_test import a0_delta

    svc = _svc(tmp_path)
    try:
        q = svc.skills._ik(ToolPose(189.0, 169.0, CFG.table_z_mm + 80, 100.0, -90.0))
        assert q is not None and abs(svc.kin.fk(q).yaw_deg - 100.0) < 1.0  # IK 자세 가중치 0.1
        o = {"x": 0.0, "y": 30.0, "z": 55.0}
        svc.set_sensor_offset(o)
        reported = svc.sensor_position(q)
        svc.set_sensor_offset({"x": 3.0, "y": 26.0, "z": 57.0})
        actual = svc.sensor_position(q)
    finally:
        svc.disconnect()
    e = (actual[0] - reported[0], actual[1] - reported[1])  # 막대(실제 센서 아래) - 팔이 보고한 위치
    dx, dy = a0_delta(e, 100.0)
    assert dx == pytest.approx(3.0, abs=1e-3) and dy == pytest.approx(-4.0, abs=1e-3) and actual[2] - reported[2] == pytest.approx(2.0)


def _a0_rows(deltas, status="ok"):
    """식(a0_delta의 역)으로 만든 가짜 행: δ를 넣으면 a0_offsets가 그 δ를 낸다."""
    from so101_studio.perception.ultrasonic.check import pick_yaw
    from so101_studio.perception.ultrasonic.model import load_config

    cfg, origin = load_config(), (215.2, 21.4)
    psi = math.radians(pick_yaw(cfg, (189, 169), origin))
    u, v = (math.cos(psi), math.sin(psi)), (-math.sin(psi), math.cos(psi))
    rows = []
    for dx, dy, dz in deltas:
        e = (-dx * u[0] + dy * v[0], -dx * u[1] + dy * v[1])
        rows.append({"status": status, "x_mm": 189 - e[0], "y_mm": 169 - e[1], "height_mm": 40 - dz, "truth_xy": (189, 169), "truth_height_mm": 40, "cand_xy": (189, 169)})
    return rows, cfg, origin


def test_a0_summary_four_cases():
    from so101_studio.service.check_test import a0_offsets, a0_summary

    rows, cfg, origin = _a0_rows([(0.5, -0.5, 1.0)] * 5)
    assert a0_offsets(rows, cfg, origin)[0] == pytest.approx((0.5, -0.5, 1.0), abs=1e-6)
    assert a0_summary(rows, cfg, origin).startswith("A0: 합격")
    assert a0_summary(rows[:4] + [{**rows[4], "status": "empty", "x_mm": None}], cfg, origin) == "A0: 미완료(ok 4/5회)"
    shaky, _, _ = _a0_rows([(0, 0, 0), (5, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    assert "불안정" in a0_summary(shaky, cfg, origin) and "arm_sensor_offset_mm" not in a0_summary(shaky, cfg, origin)
    fix, _, _ = _a0_rows([(3, -4, 2)] * 5)
    off = cfg.arm_sensor_offset_mm
    assert a0_summary(fix, cfg, origin).endswith(f"arm_sensor_offset_mm: {{x: {round(off.x + 3, 1):g}, y: {round(off.y - 4, 1):g}, z: {round(off.z + 2, 1):g}}}")


def test_checktest_a0_refuses_a_non_square_bar(tmp_path, monkeypatch):
    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    with pytest.raises(SystemExit, match="폭 = 길이"):
        run_checktest(types.SimpleNamespace(), _args(values=["a0"], obj="10x20x40"))


def _new_offset(out: str):
    import re

    m = re.search(r"arm_sensor_offset_mm: \{x: ([-\d.]+), y: ([-\d.]+), z: ([-\d.]+)\}", out)
    return None if m is None else tuple(map(float, m.groups()))


@pytest.mark.slow
def test_sim_checktest_a0_passes_with_five_checks(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    svc = _svc(tmp_path)
    try:
        run_checktest(svc, _args(values=["a0"], obj=None, seed=None, offset=None, reps=None))
    finally:
        svc.disconnect()
    _d, data, idx = _one_run(tmp_path / "truth")
    assert len(data["placements"]) == 5 and all(len(p["checks"]) == 1 for p in data["placements"]) and idx[0]["test"] == "A0"
    out = capsys.readouterr().out
    print(out)
    assert "A0: 합격" in out, out


@pytest.mark.slow
def test_sim_checktest_a0_finds_an_injected_offset_error(tmp_path, monkeypatch, capsys):
    """가상 센서를 팔이 보고한 위치보다 손 좌표 (3, -4)·위 2mm에 두면 고칠 값이 설정 + (3, -4, 2)에서 성분마다 1.5mm 안이다."""
    import numpy as np

    from so101_studio.perception.ultrasonic.model import load_config
    from so101_studio.service import ultrasonic

    monkeypatch.setattr(truth, "ROOT", tmp_path / "truth")
    orig = ultrasonic.make_ultrasonic_source

    def wrapped(arm, **kw):
        from so101_studio.perception.ultrasonic.check import pick_yaw

        src = orig(arm, **kw)
        base = src.esp.arm_sensor_fn
        psi = math.radians(pick_yaw(src.cfg, (189, 169), src.arm_origin))  # CHECK 동안 손은 수직 아래·yaw ψ 고정: 손 x축 (-cos, -sin), y축 (-sin, cos)
        axes = np.array([[-math.cos(psi), -math.sin(psi)], [-math.sin(psi), math.cos(psi)]]).T  # 열 = 손 x·y축(kin을 다른 스레드에서 부르지 않는다)
        d = axes @ np.array([3.0, -4.0])

        def shifted(t):
            p = base(t)
            return None if p is None else (p[0] + d[0], p[1] + d[1], p[2] + 2.0)

        src.esp.arm_sensor_fn = shifted
        return src

    monkeypatch.setattr(ultrasonic, "make_ultrasonic_source", wrapped)
    svc = _svc(tmp_path)
    try:
        run_checktest(svc, _args(values=["a0"], obj=None, seed=None, offset=None, reps=None))
    finally:
        svc.disconnect()
    out = capsys.readouterr().out
    print(out)
    got, off = _new_offset(out), load_config().arm_sensor_offset_mm
    assert got is not None, out
    assert got == pytest.approx((off.x + 3, off.y - 4, off.z + 2), abs=1.5), out
