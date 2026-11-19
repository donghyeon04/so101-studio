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
