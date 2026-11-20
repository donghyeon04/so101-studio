"""REQ-107(피드백 F8, arm-software-spec 7.1 그리퍼별 캘리브레이션): calibration_id_by_gripper, id별 calibration_checked, --save가 다른 그리퍼 파일 1~5를 맞춤."""
import json
from datetime import date, datetime
from pathlib import Path

import yaml

from so101_studio.arm.autocal.save import sync_other_grippers
from so101_studio.core.calibration import (
    calibration_sha256,
    forget_checked,
    record_checked,
    zero_check,
)
from so101_studio.core.config import load_robot

ROBOT = Path("config/robot.yaml").resolve()
TABLE = "calibration_id_by_gripper: {single_jaw: arm_a, parallel: arm_a_parallel}\n"


def arm(tmp_path, extra=""):
    d = tmp_path / "arms"
    d.mkdir(exist_ok=True)
    (d / "arm_a.yaml").write_text("board_serial: X\ncalibration_id: arm_a\n" + extra, encoding="utf-8")
    return d


def load(tmp_path, gripper, extra=TABLE):
    return load_robot(ROBOT, arms_dir=arm(tmp_path, extra), local_path=tmp_path / "none.yaml", gripper=gripper)


def test_table_picks_the_gripper_file_and_without_a_table_the_id_stays(tmp_path):
    assert load(tmp_path, "parallel").calibration_id == "arm_a_parallel"
    assert load(tmp_path, "single_jaw").calibration_id == "arm_a"
    assert load(tmp_path, "parallel", extra="").calibration_id == "arm_a"


def test_old_checked_format_belongs_to_the_arm_file_calibration_id(tmp_path):
    cal = {"gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 1, "range_max": 2}}
    (tmp_path / "calibration").mkdir()
    for cid in ("arm_a", "arm_a_parallel"):
        (tmp_path / "calibration" / f"{cid}.json").write_text(json.dumps(cal), encoding="utf-8")
    old = f'calibration_checked:\n  date: "2026-11-14"\n  sha256: "{calibration_sha256(cal)}"\n'
    sj, par = load(tmp_path, "single_jaw", TABLE + old), load(tmp_path, "parallel", TABLE + old)
    assert sj.calibration_checked == {"arm_a": {"date": "2026-11-14", "sha256": calibration_sha256(cal)}}
    assert zero_check(sj, tmp_path)["state"] == "checked"
    assert zero_check(par, tmp_path)["state"] == "needed"  # 병렬 파일은 아직 확인 안 됨: 실물은 locked


def test_record_checked_by_id_keeps_the_other_ids_and_forget_removes_them(tmp_path):
    f = arm(tmp_path, TABLE + 'calibration_checked:   # 옛 기록\n  date: "2026-01-01"\n  sha256: "old"\ntable_z_mm: 3\n') / "arm_a.yaml"
    record_checked(f, "newhash", date(2026, 11, 20), "arm_a_parallel")
    data = yaml.safe_load(f.read_text(encoding="utf-8"))
    assert data["calibration_checked"] == {"arm_a": {"date": "2026-01-01", "sha256": "old"}, "arm_a_parallel": {"date": "2026-11-20", "sha256": "newhash"}}
    assert data["table_z_mm"] == 3 and f.read_text(encoding="utf-8").count("calibration_checked") == 1
    forget_checked(f, ["arm_a"])
    assert yaml.safe_load(f.read_text(encoding="utf-8"))["calibration_checked"] == {"arm_a_parallel": {"date": "2026-11-20", "sha256": "newhash"}}
    forget_checked(f, ["arm_a_parallel"])
    assert "calibration_checked" not in f.read_text(encoding="utf-8")


def _cal(offset, grip_min):
    out = {n: {"id": i, "drive_mode": 0, "homing_offset": offset, "range_min": 100, "range_max": 3900}
           for i, n in enumerate(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"), 1)}
    return {**out, "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": grip_min, "range_max": 2800}}


def test_save_fixes_servos_1_to_5_of_the_other_gripper_file_and_forgets_its_check(tmp_path):
    cfg = load(tmp_path, "single_jaw", TABLE + 'calibration_checked:\n  arm_a_parallel:\n    date: "2026-11-01"\n    sha256: "p"\n')
    (tmp_path / "calibration").mkdir()
    cache = tmp_path / "cache"
    cache.mkdir()
    par = tmp_path / "calibration" / "arm_a_parallel.json"
    par.write_text(json.dumps(_cal(0, 1500)), encoding="utf-8")
    (cache / "arm_a_parallel.json").write_text(json.dumps(_cal(0, 1500)), encoding="utf-8")
    measured = _cal(42, 1300)  # 단동식으로 잰 새 값: 서보 1~5 오프셋 42, 그리퍼 범위는 단동식 것
    said = []
    changed = sync_other_grippers(cfg, measured, config_dir=tmp_path, cache_dir=cache, arm_file=tmp_path / "arms" / "arm_a.yaml",
                                  now=datetime(2026, 11, 20, 9, 0), out=said.append)
    assert changed == ["arm_a_parallel"]
    for p in (par, cache / "arm_a_parallel.json"):
        new = json.loads(p.read_text(encoding="utf-8"))
        assert all(new[n]["homing_offset"] == 42 for n in new if n != "gripper") and new["gripper"]["range_min"] == 1500  # 서보 6은 그 그리퍼 값
    assert (tmp_path / "calibration" / "arm_a_parallel.20261120-0900.bak.json").is_file()
    assert "calibration_checked" not in (tmp_path / "arms" / "arm_a.yaml").read_text(encoding="utf-8")
