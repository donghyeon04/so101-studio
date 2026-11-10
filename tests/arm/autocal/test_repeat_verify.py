"""반복 합격, 중앙값 저장, 0° 자세 확인과 잠금 (REQ-017, decisions/0029). 가짜 서보·가짜 runs로 확인한다."""
import dataclasses
import json
import time
from datetime import date, datetime
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from lerobot.motors import MotorCalibration
from test_measure_save import CAL_ID, SaveBus, args, eeprom_arm, measure, opener_for

from so101_studio import doctor as dr
from so101_studio.arm.autocal import cli as acli
from so101_studio.arm.autocal import verify
from so101_studio.arm.autocal.engine import AutoCalibrationError, deg_to_steps
from so101_studio.arm.autocal.measure import compare
from so101_studio.arm.autocal.repeat import (
    median_calibration,
    progress_line,
    repeat_status,
    spread_deg,
)
from so101_studio.arm.sim_driver import SimDriver
from so101_studio.core import ports as ports_mod
from so101_studio.core.calibration import (
    calibration_sha256,
    find_calibration,
    record_checked,
    zero_check,
)
from so101_studio.core.config import load_robot, load_workspace
from so101_studio.core.types import JointState
from so101_studio.service.arm_service import ArmService

NOW = 1_000_000.0
H = 3600


@pytest.fixture(scope="module")
def base():
    """정상 측정 한 번의 관절별 캘리브레이션(가짜 팔)."""
    report, _ = measure(eeprom_arm())
    return report["measured"]


def put_run(runs: Path, name: str, measured: dict, *, age_h: float = 1.0, cal_id: str = CAL_ID, tweaks: dict | None = None) -> dict:
    """runs/<name>/measure.json. tweaks={(joint, field): 더할 steps}."""
    m = {j: dict(c) for j, c in measured.items()}
    for (joint, field), delta in (tweaks or {}).items():
        m[joint][field] += delta
    report = {"kind": "autocal-measure", "calibration_id": cal_id, "finished": NOW - age_h * H, "passed": False, "has_old": False, "measured": m, "rows": []}
    (runs / name).mkdir(parents=True)
    (runs / name / "measure.json").write_text(json.dumps(report), encoding="utf-8")
    return report


# ---- 반복 합격
def test_three_identical_runs_pass_and_the_median_is_the_value(base, tmp_path):
    for i in range(3):
        put_run(tmp_path, f"r{i}", base, age_h=3 - i)
    st = repeat_status(tmp_path, CAL_ID, NOW)
    assert st["passed"] and st["count"] == 3 and st["max_diff_deg"] == 0 and st["median"] == base
    assert progress_line(st) == "반복 3/3, 지금까지 최대 차이 0.00° (합격)"


@pytest.mark.parametrize("field", ["range_max", "homing_offset"])
def test_the_1_degree_boundary(base, tmp_path, field):
    # 1° = 11.38 steps: 11 steps (0.97°) passes, 12 steps (1.05°) does not. 범위와 0° 위치(호밍 오프셋) 모두 같은 기준
    put_run(tmp_path, "a", base, age_h=3)
    put_run(tmp_path, "b", base, age_h=2)
    put_run(tmp_path, "c", base, age_h=1, tweaks={("elbow_flex", field): 11})
    ok = repeat_status(tmp_path, CAL_ID, NOW)
    assert ok["passed"] and ok["max_diff_deg"] == pytest.approx(0.97, abs=0.02)
    bad_dir = tmp_path / "bad"
    for i, age in enumerate((3, 2)):
        put_run(bad_dir, f"a{i}", base, age_h=age)
    put_run(bad_dir, "c", base, age_h=1, tweaks={("elbow_flex", field): 12})
    bad = repeat_status(bad_dir, CAL_ID, NOW)
    assert not bad["passed"] and bad["median"] is None and bad["max_diff_deg"] == pytest.approx(1.05, abs=0.02)
    assert "넘게 달라" in progress_line(bad)


def test_fewer_than_three_runs_do_not_pass(base, tmp_path):
    put_run(tmp_path, "a", base, age_h=2)
    put_run(tmp_path, "b", base, age_h=1)
    st = repeat_status(tmp_path, CAL_ID, NOW)
    assert not st["passed"] and st["count"] == 2 and "3회가 모여야" in progress_line(st) and progress_line(st).startswith("반복 2/3")


def test_runs_older_than_24_hours_and_other_arms_are_left_out(base, tmp_path):
    put_run(tmp_path, "old", base, age_h=25)
    put_run(tmp_path, "other", base, age_h=1, cal_id="other_arm")
    put_run(tmp_path, "a", base, age_h=3)
    put_run(tmp_path, "b", base, age_h=2)
    assert repeat_status(tmp_path, CAL_ID, NOW)["count"] == 2  # 24시간 밖과 다른 팔은 세지 않는다
    put_run(tmp_path, "c", base, age_h=1)
    assert repeat_status(tmp_path, CAL_ID, NOW)["passed"]
    put_run(tmp_path, "edge", base, age_h=24.0)  # 정확히 24시간은 포함
    assert repeat_status(tmp_path, CAL_ID, NOW)["count"] == 4


def test_only_the_three_most_recent_runs_are_compared(base, tmp_path):
    put_run(tmp_path, "wild", base, age_h=20, tweaks={("gripper", "range_max"): 300})  # 오래된 이상한 측정은 최근 3개에 들지 않는다
    for i in range(3):
        put_run(tmp_path, f"r{i}", base, age_h=3 - i)
    st = repeat_status(tmp_path, CAL_ID, NOW)
    assert st["passed"] and st["count"] == 4 and len(st["used"]) == 3


def test_median_is_taken_per_joint_and_field(base):
    reports = [{"measured": {j: {**c, "homing_offset": c["homing_offset"] + d, "range_min": c["range_min"] + e} for j, c in base.items()}}
               for d, e in ((1, 100), (3, 105), (2, 103))]
    med = median_calibration(reports)
    assert med["shoulder_pan"]["homing_offset"] == base["shoulder_pan"]["homing_offset"] + 2 and med["shoulder_pan"]["range_min"] == base["shoulder_pan"]["range_min"] + 103
    assert med["gripper"]["range_max"] == base["gripper"]["range_max"] and med["gripper"]["id"] == base["gripper"]["id"]


def test_spread_wraps_the_zero_position_around_the_encoder_end(base):
    a = {"measured": {"x": {"id": 1, "drive_mode": 0, "homing_offset": 2047, "range_min": 1000, "range_max": 3000}}}
    b = {"measured": {"x": {"id": 1, "drive_mode": 0, "homing_offset": -2047 + 5, "range_min": 1000, "range_max": 3000}}}  # 인코더 끝을 넘어 7 steps 차이
    assert spread_deg([a, b]) == pytest.approx(7 * 360 / 4096, abs=0.01)


def test_measure_only_shows_the_repetition_progress(tmp_path):
    runs = tmp_path / "runs"
    lines = []
    for n in (1, 2):
        arm = eeprom_arm()
        from so101_studio.arm.autocal.measure import measure_only

        measure_only(arm, CAL_ID, None, out=lines.append, clock=arm.clock, sleep=arm.sleep, run_dir=runs / f"2026110{n}-autocal")
    reps = [x for x in lines if x.startswith("반복 ")]
    assert reps[0].startswith("반복 1/3, 지금까지 최대 차이 0.00°") and reps[1].startswith("반복 2/3, 지금까지 최대 차이 0.00°")
    saved = json.loads((runs / "20261102-autocal" / "measure.json").read_text(encoding="utf-8"))
    assert saved["repeat"]["count"] == 2 and not saved["repeat"]["passed"]


# ---- --save: 반복 합격
def _opener(base, tmp_path, with_old: bool):
    cache = tmp_path / "cache" / f"{CAL_ID}.json"
    old = {j: MotorCalibration(c["id"], 0, c["homing_offset"] + 40, c["range_min"] - 20, c["range_max"]) for j, c in base.items()}
    if with_old:
        cache.parent.mkdir(parents=True)
        cache.write_text(json.dumps({j: {"id": c.id, "drive_mode": 0, "homing_offset": c.homing_offset, "range_min": c.range_min, "range_max": c.range_max} for j, c in old.items()}), encoding="utf-8")
    bus = SaveBus({j: c["id"] for j, c in base.items()}, old)
    return bus, opener_for(bus, cache), cache


def _three_runs(base, runs):
    for i, (d, e) in enumerate(((0, 0), (2, 1), (1, -1))):  # 서로 1° 이내로 조금씩 다르다
        m = {j: dict(c) for j, c in base.items()}
        m["elbow_flex"]["homing_offset"] += d
        m["elbow_flex"]["range_min"] += e
        put_run(runs, f"2026110{i}-autocal", m, age_h=3 - i)
    return runs


def run_save(tmp_path, answers, with_old, base):
    runs = _three_runs(base, tmp_path / "runs")
    # 파일 시각을 NOW 기준으로 만들었으므로 now를 NOW로 준다
    bus, opener, cache = _opener(base, tmp_path, with_old)
    said, prompts = [], []
    it = iter(answers)
    code = acli.run_calibrate(args(auto=False, measure_only=False, save=True), SimpleNamespace(calibration_id=CAL_ID, port="COM_TEST"),
                              input_fn=lambda p: prompts.append(p) or next(it), out=said.append, opener=opener, now=lambda: datetime.fromtimestamp(NOW),
                              config_dir=tmp_path / "config", runs_dir=runs)
    return code, bus, said, prompts, cache


def test_repeat_pass_with_an_old_file_needs_a_typed_yes_and_saves_the_median(base, tmp_path, monkeypatch):
    monkeypatch.setattr("so101_studio.doctor.doctor", lambda **k: {"checks": [{"name": "캘리브레이션 대조", "state": "ok", "text": "", "fix": ""}]})
    code, bus, said, prompts, _ = run_save(tmp_path, ["yes"], True, base)
    text = "\n".join(said)
    assert code == 0 and bus.writes == 1 and "반복 합격" in text and "범위 차이" in text  # 차이표를 보여 주고 yes를 받았다
    assert "yes" in prompts[0]
    med = median_calibration([json.loads(p.read_text(encoding="utf-8")) for p in sorted((tmp_path / "runs").glob("*/measure.json"))])
    assert bus.eeprom["elbow_flex"].homing_offset == med["elbow_flex"]["homing_offset"] == base["elbow_flex"]["homing_offset"] + 1  # 중앙값
    assert bus.eeprom["elbow_flex"].range_min == base["elbow_flex"]["range_min"]
    assert list((tmp_path / "config" / "calibration").glob(f"{CAL_ID}.*.bak.json"))  # 이전 파일을 남겼다
    assert "--verify-zero" in text  # 저장한 뒤에는 0° 자세를 다시 확인해야 한다


@pytest.mark.parametrize("answer", ["y", "", "no", "예"])
def test_anything_but_yes_saves_nothing(base, tmp_path, answer):
    code, bus, said, _, _cache = run_save(tmp_path, [answer], True, base)
    assert code == 1 and bus.writes == 0 and "yes가 아니라서 저장하지 않았다" in "\n".join(said)
    assert not (tmp_path / "config" / "calibration").exists() and not (tmp_path / "runs" / f"autocal-save-{CAL_ID}.json").exists()


def test_repeat_pass_without_an_old_file_asks_only_for_enter(base, tmp_path, monkeypatch):
    monkeypatch.setattr("so101_studio.doctor.doctor", lambda **k: {"checks": [{"name": "캘리브레이션 대조", "state": "ok", "text": "", "fix": ""}]})
    code, bus, _, prompts, _ = run_save(tmp_path, [""], False, base)
    assert code == 0 and bus.writes == 1 and "기존 캘리브레이션 파일이 없다" in prompts[0]


def test_two_runs_or_a_wide_spread_cannot_be_saved(base, tmp_path):
    runs = tmp_path / "runs"
    put_run(runs, "a-autocal", base, age_h=2)
    put_run(runs, "b-autocal", base, age_h=1, tweaks={("wrist_flex", "range_max"): 40})  # 3.5° 차이이고 2개뿐
    bus, opener, _ = _opener(base, tmp_path, True)
    said = []
    code = acli.run_calibrate(args(auto=False, measure_only=False, save=True), SimpleNamespace(calibration_id=CAL_ID, port="COM_TEST"), input_fn=lambda _: "yes",
                              out=said.append, opener=opener, now=lambda: datetime.fromtimestamp(NOW), config_dir=tmp_path / "config", runs_dir=runs)
    assert code == 1 and bus.writes == 0 and "반복 합격" in "\n".join(said) and "아니다" in "\n".join(said)


def test_a_single_run_within_3_degrees_still_saves_without_repeating(base, tmp_path, monkeypatch):
    monkeypatch.setattr("so101_studio.doctor.doctor", lambda **k: {"checks": [{"name": "캘리브레이션 대조", "state": "ok", "text": "", "fix": ""}]})
    runs = tmp_path / "runs"
    report = put_run(runs, "a-autocal", base, age_h=1)
    report.update(passed=True, has_old=True, pass_deg=3.0, rows=compare(base, base))
    (runs / "a-autocal" / "measure.json").write_text(json.dumps(report), encoding="utf-8")
    bus, opener, _ = _opener(base, tmp_path, True)
    code = acli.run_calibrate(args(auto=False, measure_only=False, save=True), SimpleNamespace(calibration_id=CAL_ID, port="COM_TEST"), input_fn=lambda _: "",
                              out=lambda *_: None, opener=opener, now=lambda: datetime.fromtimestamp(NOW), config_dir=tmp_path / "config", runs_dir=runs)
    assert code == 0 and bus.writes == 1  # 1회 합격 갈래는 그대로(Enter만)


# ---- 0° 자세 확인과 잠금
CFG0 = dataclasses.replace(load_robot("config/robot.yaml"), calibration_id="test_arm", arm_name="test_arm", calibration_checked={})


@pytest.fixture
def conf(tmp_path):
    """tmp/config: 팔 파일과 캘리브레이션 파일(HF 캐시에 없는 id라 이 사본이 쓰인다)."""
    (tmp_path / "config" / "arms").mkdir(parents=True)
    (tmp_path / "config" / "calibration").mkdir()
    cal = {"shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -3, "range_min": 100, "range_max": 3900}}
    (tmp_path / "config" / "calibration" / "test_arm.json").write_text(json.dumps(cal, indent=4), encoding="utf-8")
    (tmp_path / "config" / "arms" / "test_arm.yaml").write_text("# 시험 팔\nboard_serial: \"X\"\ncalibration_id: test_arm\ntable_z_mm: 0\n", encoding="utf-8")
    return tmp_path / "config"


def service(tmp_path, conf, cfg=CFG0, hard=None):
    d = SimDriver(cfg.poses["home"])
    d.simulated, d.locked_reason = False, hard  # 실제 장치처럼 보이게(시뮬레이션은 잠그지 않는다)
    svc = ArmService(d, cfg, load_workspace("config/workspace.yaml"), speed_pct=50, log_root=str(tmp_path / "log"))
    svc.config_dir = conf
    ok, msg = svc.connect()
    assert ok, msg
    return svc


def test_unchecked_arm_is_locked_until_the_zero_pose_is_verified(tmp_path, conf):
    svc = service(tmp_path, conf)
    try:
        assert "0° 자세 확인 필요" in svc.locked_reason and "--verify-zero" in svc.locked_reason
        assert svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "locked" and svc.jog("joint", [5, 0, 0, 0, 0, 0]) is False
        assert svc.status().joints  # 읽기는 된다
    finally:
        svc.disconnect()


def test_hash_mismatch_locks_and_matching_hash_unlocks(tmp_path, conf):
    cal, _ = find_calibration("test_arm", conf)
    ok_cfg = dataclasses.replace(CFG0, calibration_checked={"date": "2026-11-04", "sha256": calibration_sha256(cal)})
    svc = service(tmp_path, conf, ok_cfg)
    try:
        assert svc.locked_reason is None and svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "ok"
    finally:
        svc.disconnect()
    cal["shoulder_pan"]["range_max"] = 3800  # 캘리브레이션 파일이 바뀌었다
    (conf / "calibration" / "test_arm.json").write_text(json.dumps(cal), encoding="utf-8")
    svc = service(tmp_path, conf, ok_cfg)
    try:
        assert "다르다" in svc.locked_reason and svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "locked"
    finally:
        svc.disconnect()
    assert zero_check(ok_cfg, conf)["state"] == "needed"


def test_the_hash_ignores_file_formatting(conf):
    cal, _ = find_calibration("test_arm", conf)
    (conf / "calibration" / "test_arm.json").write_text(json.dumps(cal, separators=(",", ":")), encoding="utf-8")  # 들여쓰기 없이 다시 저장
    assert calibration_sha256(find_calibration("test_arm", conf)[0]) == calibration_sha256(cal)


def test_verify_zero_moves_at_20_percent_asks_a_human_and_records_the_hash(tmp_path, conf):
    svc = service(tmp_path, conf)
    speeds, said, prompts = [], [], []
    real = svc.move_joints
    svc.move_joints = lambda goal, speed_pct=None: speeds.append(speed_pct) or real(goal, speed_pct)
    answers = iter(["", "yes"])
    try:
        code = verify.run_verify_zero(svc, CFG0, input_fn=lambda p: prompts.append(p) or next(answers), out=said.append, config_dir=conf, today=lambda: date(2026, 11, 5))
    finally:
        svc.disconnect()
    arm_file = conf / "arms" / "test_arm.yaml"
    rec = yaml.safe_load(arm_file.read_text(encoding="utf-8"))["calibration_checked"]
    assert code == 0 and speeds == [20] and str(rec["date"]) == "2026-11-05" and rec["sha256"] == calibration_sha256(find_calibration("test_arm", conf)[0])
    q = svc.driver.read_joints()
    assert (q.shoulder_pan, q.shoulder_lift, q.elbow_flex, q.wrist_flex, q.wrist_roll) == (0, 0, 0, 0, 0) and q.gripper == pytest.approx(verify.GRIPPER_ZERO)
    text = "\n".join(said)
    assert "3D" in text and "URDF" in text and "클램프" in text and "전원 스위치" in text  # 실물과 3D 0° 자세를 비교하라는 안내
    assert "# 시험 팔" in arm_file.read_text(encoding="utf-8")  # 다른 줄과 주석은 그대로
    reloaded = dataclasses.replace(CFG0, calibration_checked=rec)
    assert zero_check(reloaded, conf)["state"] == "checked"  # 다음 연결부터 움직인다


def test_verify_zero_without_yes_records_nothing(tmp_path, conf):
    svc = service(tmp_path, conf)
    arm_file = conf / "arms" / "test_arm.yaml"
    before = arm_file.read_text(encoding="utf-8")
    answers = iter(["", "no"])
    try:
        code = verify.run_verify_zero(svc, CFG0, input_fn=lambda _: next(answers), out=lambda *_: None, config_dir=conf)
        assert code == 1 and arm_file.read_text(encoding="utf-8") == before
        assert "0° 자세 확인 필요" in svc.locked_reason  # 확인하지 않았으니 다시 잠겨 있다
    finally:
        svc.disconnect()


def test_verify_zero_does_not_lift_a_calibration_mismatch_lock(tmp_path, conf):
    svc = service(tmp_path, conf, hard="캘리브레이션 불일치: x")
    moved = []
    svc.move_joints = lambda *a, **k: moved.append(1)
    said = []
    try:
        assert verify.run_verify_zero(svc, CFG0, input_fn=lambda _: "yes", out=said.append, config_dir=conf) == 1
    finally:
        svc.disconnect()
    assert moved == [] and "캘리브레이션 불일치" in "\n".join(said)
    assert "calibration_checked" not in (conf / "arms" / "test_arm.yaml").read_text(encoding="utf-8")


def test_verify_zero_needs_an_arm_file(tmp_path, conf):
    svc = service(tmp_path, conf)
    try:
        assert verify.run_verify_zero(svc, dataclasses.replace(CFG0, arm_name=""), input_fn=lambda _: "yes", out=lambda *_: None, config_dir=conf) == 1
    finally:
        svc.disconnect()


def test_record_checked_replaces_the_old_record(tmp_path):
    f = tmp_path / "a.yaml"
    f.write_text("calibration_id: x\ncalibration_checked:   # 옛 기록\n  date: \"2026-01-01\"\n  sha256: \"old\"\ntable_z_mm: 3\n", encoding="utf-8")
    record_checked(f, "newhash", date(2026, 11, 5))
    data = yaml.safe_load(f.read_text(encoding="utf-8"))
    assert data["calibration_checked"] == {"date": "2026-11-05", "sha256": "newhash"} and data["table_z_mm"] == 3 and f.read_text(encoding="utf-8").count("calibration_checked") == 1


def test_measure_only_is_not_blocked_by_the_zero_lock(base, tmp_path):
    """calibrate --auto(측정만)는 ArmService를 거치지 않으므로 0° 자세 확인 전에도 쓸 수 있다."""
    arm = eeprom_arm()
    import so101_studio.arm.autocal.cli as c

    real = c.measure_only
    said = []
    try:
        c.measure_only = lambda *a, **k: real(*a, **k, clock=arm.clock, sleep=arm.sleep)
        code = c.run_calibrate(args(), SimpleNamespace(calibration_id=CAL_ID, port="COM_TEST"), input_fn=lambda _: "", out=said.append, opener=opener_for(arm, tmp_path / "none.json"),
                               now=lambda: datetime(2026, 11, 5, 9, 0), config_dir=tmp_path, runs_dir=tmp_path / "runs")
    finally:
        c.measure_only = real
    assert code in (0, 1) and "반복 1/3" in "\n".join(said)  # 기존 파일이 없어 1회 합격은 아니지만 측정은 끝까지 돈다


# ---- doctor
def test_doctor_shows_the_zero_check_state(conf, monkeypatch):
    monkeypatch.setattr(ports_mod, "list_serial_ports", list)
    cal, _ = find_calibration("test_arm", conf)
    res = dr.doctor(cfg=CFG0, probe=False, config_path=conf / "robot.yaml")
    assert res["zero_check"]["state"] == "needed" and any(c["name"] == "0° 자세 확인" and c["state"] == "warn" and "--verify-zero" in c["fix"] for c in res["checks"])
    ok = dataclasses.replace(CFG0, calibration_checked={"date": "2026-11-05", "sha256": calibration_sha256(cal)})
    res = dr.doctor(cfg=ok, probe=False, config_path=conf / "robot.yaml")
    assert res["zero_check"]["state"] == "checked" and any(c["name"] == "0° 자세 확인" and c["state"] == "ok" for c in res["checks"])
    json.dumps(res)


def test_deg_to_steps_helper_matches_the_boundary_math():
    assert deg_to_steps(1.0) == 11 and deg_to_steps(1.05) == 12  # 위 경계 시험이 쓰는 11·12 steps
    with pytest.raises(AutoCalibrationError):
        raise AutoCalibrationError("x")
    assert time.time() > 0
