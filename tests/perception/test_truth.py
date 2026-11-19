"""정답 쌍 기록 (ultrasonic-spec 8.4, REQ-096): truth.json + index.jsonl, scans·checks와 짝짓기."""
import json
import math

from so101_studio.perception.ultrasonic import radar_check, truth
from so101_studio.perception.ultrasonic.benchmark import build, do_check, make_source
from so101_studio.perception.ultrasonic.model import Box


def test_sim_r1_writes_truth_paired_with_scans(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(truth, "ROOT", tmp_path)
    radar_check.main(["--sim", "r1", "--reps", "1", "--obj", "40x80x40"])
    [line] = truth.index()
    assert line["test"] == "R1" and line["placements"] == 9 and line["objects"] == 9 and line["sim"] is True
    run = tmp_path / line["run"].rsplit("/", 1)[-1]
    assert json.loads((run / "truth.json").read_text(encoding="utf-8"))["config"]["radar"]["station_mm"]
    spots = set()
    for p, scans, checks in truth.pairs(run):
        [o], [scan] = p["objects"], scans
        assert checks == [] and o["width_mm"] == 40 and o["length_mm"] == 80 and o["height_mm"] == 40
        assert scan["passes"] and scan["baseline"]["d"] and scan["values"]
        err = min(math.dist((c["x_mm"], c["y_mm"]), (o["x_mm"], o["y_mm"])) for c in scan["candidates"])
        assert err < 90, (o, scan["candidates"])
        spots.add((o["x_mm"], o["y_mm"]))
    assert len(spots) == 9
    assert "R1: 9/9" in capsys.readouterr().out


def test_check_record_paired(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)], model=None, reach=None)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    run = truth.TruthRun("A1", cfg, sim=True, root=tmp_path)
    src.record_dir = run.dir
    cand = src.scan().objects[0]
    obj, _, _ = do_check(src, arm, cand)
    run.add([truth.obj(300, 170, 0, 30, 100, 40)], src.saved)
    run.close()
    run.close()  # 두 번 불러도 한 줄
    assert len(truth.index(tmp_path)) == 1
    [(_, [_], [check])] = list(truth.pairs(run.dir))  # 자리 하나에 scan 하나·check 하나
    assert check["status"] == "ok" and check["result"]["x_mm"] == obj.x_mm
    seg = check["segments"][0]
    assert seg["poses"][0]["sensor_mm"] and seg["segment"]["points_mm"] and check["arm_raw"]
    assert any(s["points"] for s in check["segments"])
    assert src._probe is None  # 다음 확인에 이번 구간이 안 섞인다


def test_empty_run_not_indexed(tmp_path):
    run = truth.TruthRun("R1", sim=True, root=tmp_path)
    run.close()
    assert truth.index(tmp_path) == [] and (run.dir / "truth.json").exists()


def test_no_record_dir_writes_nothing(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)], model=None, reach=None)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    do_check(src, arm, src.scan().objects[0])
    assert src.saved == [] and sorted(f.name for f in tmp_path.iterdir()) == ["baseline.json"]
