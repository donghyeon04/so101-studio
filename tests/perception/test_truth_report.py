"""정답 쌍 분석 (truth_report): --sim r1 기록으로 레이더 표, 다시 계산, 팔 확인 표."""
import json

import pytest

from so101_studio.perception.ultrasonic import radar_check, truth, truth_report
from so101_studio.perception.ultrasonic.benchmark import build, do_check, make_source
from so101_studio.perception.ultrasonic.model import Box


@pytest.fixture(scope="module")
def r1_run(tmp_path_factory):
    root = tmp_path_factory.mktemp("truth")
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(truth, "ROOT", root)
        radar_check.main(["--sim", "r1", "--reps", "1"])
    [line] = truth.index(root)
    return root, root / line["run"].rsplit("/", 1)[-1]


def test_radar_table_from_sim_r1(r1_run, capsys):
    root, _ = r1_run
    truth_report.main(["all", "--root", str(root)])
    out = capsys.readouterr().out
    assert "레이더: 9/9이 90mm 안 (100%, 합격 95% 통과)" in out
    assert out.count("반경 ") == 9 and "(280,150)" in out and "통 안" in out  # R1 자리 (280,150)은 통 무시 원 안이다


def test_recompute_same_config_matches_recorded(r1_run):
    _, run = r1_run
    cfg = truth_report._wrap(json.loads((run / "truth.json").read_text(encoding="utf-8"))["config"])
    for _, [scan], _ in truth.pairs(run):
        got = truth_report.recompute(scan, cfg)
        assert [v for c in got for v in c] == pytest.approx([c[k] for c in scan["candidates"] for k in ("x_mm", "y_mm")])


def test_recompute_with_override_changes_result(r1_run):
    _, run = r1_run
    rows = truth_report.radar_rows(run, {"dip_mm": 1000})  # 기준값보다 1m 가까워야 후보: 아무것도 안 나온다
    assert truth_report.radar_summary(rows)["none"] == 9
    assert "다시 계산: dip_mm=1000" in truth_report.report([run], {"dip_mm": 1000})


def test_set_rejects_keys_that_need_raw_arm_data(r1_run):
    root, _ = r1_run
    with pytest.raises(SystemExit):
        truth_report.main(["all", "--root", str(root), "--set", "beam_half_deg=10"])


def test_check_table(tmp_path):
    cfg, clk, esp, arm = build([Box(300, 170, 0)], model=None, reach=None)
    src = make_source(cfg, clk, esp, arm, tmp_path)
    run = truth.TruthRun("A1", cfg, sim=True, root=tmp_path)
    src.record_dir = run.dir
    do_check(src, arm, src.scan().objects[0])
    run.add([truth.obj(300, 170, 0, 30, 100, 40)], src.saved)
    [row] = truth_report.check_rows(run.dir)
    s = truth_report.check_summary([row])
    assert row["status"] == "ok" and s["measured"] == 1 and s["angle_n"] == 1 and s["median"] < 15
    assert "팔 확인: 1/1회 결과" in truth_report.report([run.dir])
