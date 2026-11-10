"""so101 calibrate --auto --measure-only / --save (REQ-016, decisions/0028): 가짜 서보로 확인한다. 실제 팔은 열지 않는다."""
import json
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from test_engine import SO101, SimArm, make_arm

from so101_studio.arm.autocal import cli as acli
from so101_studio.arm.autocal.engine import AutoCalibrationError
from so101_studio.arm.autocal.guard import GuardedBus, vitals_problem
from so101_studio.arm.autocal.measure import (
    CAL_REGISTERS,
    compare,
    measure_only,
    range_deg,
    restore_ram,
    zero_raw,
)
from so101_studio.arm.autocal.save import (
    latest_measure,
    physical_limits_suggestion,
    save_calibration,
)

CAL_ID = "so101_follower_01"


class EepromArm(SimArm):
    """SimArm에 EEPROM을 더한다: Lock=0일 때 쓴 캘리브레이션 레지스터만 EEPROM에 남고(전원을 껐다 켜도 유지), Lock=1이면 RAM에만 있다.
    전압·온도 레지스터도 있다."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.eeprom = {n: {r: j.regs[r] for r in CAL_REGISTERS} for n, j in self.joints.items()}
        for j in self.joints.values():
            j.regs["Present_Voltage"], j.regs["Present_Temperature"] = 120, 35

    def _put(self, name, register, value):
        unlocked = self.joints[name].regs["Lock"] == 0
        super()._put(name, register, value)  # 실패하면 여기서 던진다
        if register in CAL_REGISTERS and unlocked:
            self.eeprom[name][register] = value


def eeprom_arm(**kw) -> EepromArm:
    base = make_arm(**kw)
    arm = EepromArm(base.joints, base.interrupt_at)
    arm.initial = base.initial
    return arm


def measure(arm, old=None, tmp=None, **kw):
    lines: list[str] = []
    report = measure_only(arm, CAL_ID, old, out=lines.append, clock=arm.clock, sleep=arm.sleep, run_dir=tmp, **kw)
    return report, lines


@pytest.fixture(scope="module")
def reference():
    """정상 측정 한 번의 결과(같은 가짜 팔의 '기존 캘리브레이션')."""
    report, _ = measure(eeprom_arm())
    return report["measured"]


def assert_untouched(arm: EepromArm) -> None:
    for name, joint in arm.joints.items():
        assert arm.eeprom[name] == {r: arm.initial[name][r] for r in CAL_REGISTERS}, name  # EEPROM 불변
        for register in (*CAL_REGISTERS, "Lock"):
            assert joint.regs[register] == arm.initial[name][register], (name, register)  # RAM도 되돌렸다
        assert joint.regs["Torque_Enable"] == 0, name


# ---- 측정만
def test_measure_only_keeps_eeprom_and_registers_and_prints_the_table(reference, tmp_path):
    arm = eeprom_arm()
    report, lines = measure(arm, reference, tmp_path / "run")
    assert_untouched(arm)
    assert report["passed"] and all(r["ok"] for r in report["rows"]) and {r["joint"] for r in report["rows"]} == set(SO101)
    text = "\n".join(lines)
    assert "범위 차이" in text and "shoulder_pan" in text and "합격" in text
    saved = json.loads((tmp_path / "run" / "measure.json").read_text(encoding="utf-8"))
    assert saved["calibration_id"] == CAL_ID and saved["measured"].keys() == SO101.keys() and saved["passed"]
    pan = report["measured"]["shoulder_pan"]
    assert abs(range_deg(pan) - 239) < 1 and abs(range_deg(report["measured"]["gripper"]) - 125) < 1  # 이 팔의 범위(239°, 125°)


def test_a_joint_more_than_3_degrees_from_the_old_file_is_not_a_pass(reference):
    old = {n: dict(c) for n, c in reference.items()}
    old["elbow_flex"]["range_min"] += 80  # 약 7°
    old["wrist_flex"]["homing_offset"] += 60  # 0° 위치가 약 5° 다르다
    report, _ = measure(eeprom_arm(), old)
    rows = {r["joint"]: r for r in report["rows"]}
    assert not report["passed"] and not rows["elbow_flex"]["ok"] and not rows["wrist_flex"]["ok"]
    assert rows["elbow_flex"]["range_diff_deg"] == pytest.approx(7.0, abs=0.2) and abs(rows["wrist_flex"]["zero_diff_deg"]) == pytest.approx(5.3, abs=0.3)
    assert rows["shoulder_pan"]["ok"] and rows["gripper"]["ok"]


def test_without_an_old_file_nothing_passes(reference):
    report, _ = measure(eeprom_arm(), None)
    assert not report["passed"] and not report["has_old"] and all(r["ok"] is None for r in report["rows"])


def test_compare_zero_position_uses_the_range_middle():
    a = {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 1000, "range_max": 3000}
    b = {"id": 1, "drive_mode": 0, "homing_offset": 100, "range_min": 1000, "range_max": 3000}
    assert zero_raw(a) == 2000 and zero_raw(b) == 2100
    row = compare({"x": a}, {"x": b})[0]
    assert row["zero_diff_deg"] == pytest.approx(8.8, abs=0.1) and row["range_diff_deg"] == 0 and not row["ok"]


def test_low_voltage_at_start_aborts_before_anything_moves(reference):
    arm = eeprom_arm()
    arm.joints["shoulder_lift"].regs["Present_Voltage"] = 105  # 10.5V
    with pytest.raises(AutoCalibrationError, match="전압 10.5V"):
        measure(arm, reference)
    assert arm.writes == []  # 아무것도 쓰지 않았다


@pytest.mark.parametrize("register, value, message", [("Present_Voltage", 108, "전압 10.8V"), ("Present_Temperature", 65, "온도 65°C")])
def test_voltage_or_temperature_during_the_run_aborts_and_restores(reference, register, value, message):
    arm = eeprom_arm()
    sleep = arm.sleep

    def failing_sleep(dt):
        sleep(dt)
        if arm.t > 8:  # 한참 움직이는 중에 전원이 처지거나 모터가 뜨거워진다
            arm.joints["elbow_flex"].regs[register] = value

    arm.sleep = failing_sleep
    with pytest.raises(AutoCalibrationError, match=message):
        measure(arm, reference)
    assert 8 < arm.t < 40  # 끝까지 가지 않고 바로 멈췄다
    assert_untouched(arm)


def test_failed_ram_restore_is_reported_and_no_result_is_returned(reference):
    arm = eeprom_arm()
    arm.failing.append(("gripper", "Homing_Offset", arm.initial["gripper"]["Homing_Offset"], ConnectionError))
    with pytest.raises(AutoCalibrationError, match="되돌리지 못했다"):
        measure(arm, reference)
    assert all(arm.eeprom[n] == {r: arm.initial[n][r] for r in CAL_REGISTERS} for n in arm.joints)  # EEPROM은 그래도 그대로다


def test_ctrl_c_during_the_run_restores_the_registers(reference):
    arm = eeprom_arm(interrupt_at=15.0)
    with pytest.raises(KeyboardInterrupt):
        measure(arm, reference)
    assert_untouched(arm)


def test_restore_ram_uses_lock_so_nothing_reaches_the_eeprom():
    arm = eeprom_arm()
    saved = {r: {n: arm.initial[n][r] for n in arm.joints} for r in ("Lock", *CAL_REGISTERS)}
    for j in arm.joints.values():
        j.regs["Lock"], j.regs["Homing_Offset"] = 0, 999  # 엔진이 끝나며 Lock을 푼 상태
    assert restore_ram(arm, saved) == []
    assert_untouched(arm)


def test_guard_forwards_everything_and_trips_once():
    arm = eeprom_arm()
    guard = GuardedBus(arm, period_s=0, clock=arm.clock)
    assert guard.motors is arm.motors and guard.sync_read("Present_Position")
    arm.joints["gripper"].regs["Present_Voltage"] = 100
    assert guard.sync_read("Present_Position") and not guard.tripped  # 한 번 넘은 값만으로는 중단하지 않는다(연속 2회)
    with pytest.raises(AutoCalibrationError):
        guard.sync_read("Present_Position")
    assert guard.tripped and guard.sync_read("Present_Position")  # 정리하는 읽기는 다시 던지지 않는다
    assert vitals_problem(arm) and "전압" in vitals_problem(arm)


def test_guard_ignores_a_single_misread_temperature():
    """실물에서 온도 80°C로 한 번 잘못 읽힌 값(실제 34~39°C)으로 측정이 중단됐다."""
    arm = eeprom_arm()
    guard = GuardedBus(arm, period_s=0, clock=arm.clock)
    temp = arm.joints["gripper"].regs
    for _ in range(3):
        temp["Present_Temperature"] = 80
        assert guard.sync_read("Present_Position")  # 한 번 튄 값
        temp["Present_Temperature"] = 36
        assert guard.sync_read("Present_Position")  # 곧 정상으로 돌아온다: 연속이 아니니 세지 않는다
    assert not guard.tripped


# ---- 저장
class SaveBus:
    """write_calibration·read_calibration·disable_torque만 있는 가짜 버스. drop이면 쓴 값이 반영되지 않는다(쓰기 실패)."""

    def __init__(self, ids: dict[str, int], eeprom: dict[str, MotorCalibration], drop: bool = False):
        self.motors = {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for n, i in ids.items()}
        self.eeprom, self.drop, self.torque_off, self.writes = eeprom, drop, 0, 0

    def disable_torque(self):
        self.torque_off += 1

    def read_calibration(self):
        return dict(self.eeprom)

    def write_calibration(self, cal, cache=True):
        self.writes += 1
        if not self.drop:
            self.eeprom = dict(cal)


@pytest.fixture
def paths(tmp_path):
    return SimpleNamespace(cache=tmp_path / "cache" / f"{CAL_ID}.json", repo=tmp_path / "config" / "calibration" / f"{CAL_ID}.json", runs=tmp_path / "runs")


def passing_report(reference, **kw):
    return {"kind": "autocal-measure", "calibration_id": CAL_ID, "finished": 1000.0, "passed": True, "has_old": True, "pass_deg": 3.0,
            "measured": reference, "rows": compare(reference, reference), **kw}


def old_motor_cal(reference):
    return {n: MotorCalibration(c["id"], 0, c["homing_offset"] + 5, c["range_min"], c["range_max"]) for n, c in reference.items()}


def test_save_writes_eeprom_both_files_and_a_backup(reference, paths):
    for p in (paths.cache, paths.repo):
        p.parent.mkdir(parents=True)
        p.write_text('{"old": true}', encoding="utf-8")
    bus = SaveBus({n: c["id"] for n, c in reference.items()}, old_motor_cal(reference))
    said: list[str] = []
    record = save_calibration(bus, passing_report(reference), cache_path=paths.cache, repo_path=paths.repo, runs_dir=paths.runs, now=datetime(2026, 11, 4, 9, 30), out=said.append)
    backup = paths.repo.parent / f"{CAL_ID}.20261104-0930.bak.json"
    assert backup.read_text(encoding="utf-8") == '{"old": true}' and record["backup"] == str(backup)  # 이전 파일을 남겼다
    assert bus.torque_off == 1 and {n: (c.homing_offset, c.range_min, c.range_max) for n, c in bus.eeprom.items()} == {n: (c["homing_offset"], c["range_min"], c["range_max"]) for n, c in reference.items()}
    for p in (paths.cache, paths.repo):
        saved = json.loads(p.read_text(encoding="utf-8"))
        assert saved["shoulder_pan"] == {k: reference["shoulder_pan"][k] for k in ("id", "drive_mode", "homing_offset", "range_min", "range_max")}
    assert json.loads((paths.runs / f"autocal-save-{CAL_ID}.json").read_text(encoding="utf-8"))["saved_at"] == "2026-11-04T09:30:00"
    assert abs(record["physical_limits_deg_suggestion"]["shoulder_pan"][1] - range_deg(reference["shoulder_pan"]) / 2) < 0.1
    text = "\n".join(said)
    assert "physical_limits_deg" in text and "바꾸지 않는다" in text  # 제안만 출력한다


def test_save_refuses_a_measurement_that_did_not_pass(reference, paths):
    bus = SaveBus({n: c["id"] for n, c in reference.items()}, old_motor_cal(reference))
    for bad in (passing_report(reference, passed=False), passing_report(reference, passed=False, has_old=False)):
        with pytest.raises(AutoCalibrationError, match="합격하지 않았다"):
            save_calibration(bus, bad, cache_path=paths.cache, repo_path=paths.repo, runs_dir=paths.runs)
    assert bus.writes == 0 and not paths.repo.exists() and not paths.cache.exists()  # 아무것도 쓰지 않았다


def test_save_checks_servo_ids(reference, paths):
    bus = SaveBus({n: c["id"] + 10 for n, c in reference.items()}, old_motor_cal(reference))
    with pytest.raises(AutoCalibrationError, match="서보 ID"):
        save_calibration(bus, passing_report(reference), cache_path=paths.cache, repo_path=paths.repo, runs_dir=paths.runs)
    assert bus.writes == 0


def test_save_rolls_back_when_the_eeprom_does_not_take_the_values(reference, paths):
    old = old_motor_cal(reference)
    bus = SaveBus({n: c["id"] for n, c in reference.items()}, old, drop=True)
    with pytest.raises(AutoCalibrationError, match="다시 읽은 값과 다르다"):
        save_calibration(bus, passing_report(reference), cache_path=paths.cache, repo_path=paths.repo, runs_dir=paths.runs)
    assert bus.writes == 2 and bus.eeprom == old  # 이전 값으로 되돌리려고 한 번 더 썼다
    assert not paths.cache.exists() and not paths.repo.exists() and not (paths.runs / f"autocal-save-{CAL_ID}.json").exists()


def test_latest_measure_picks_this_arm_and_refuses_stale_results(tmp_path):
    def put(name, cal_id, finished):
        d = tmp_path / name
        d.mkdir()
        (d / "measure.json").write_text(json.dumps({"calibration_id": cal_id, "finished": finished}), encoding="utf-8")

    put("20261104-0900-autocal", CAL_ID, 1000.0)
    put("20261104-1000-autocal", "other_arm", 2000.0)
    assert latest_measure(tmp_path, CAL_ID, now=1000.0 + 3600).parent.name == "20261104-0900-autocal"
    with pytest.raises(AutoCalibrationError, match="오래됐다"):
        latest_measure(tmp_path, CAL_ID, now=1000.0 + 25 * 3600)
    with pytest.raises(AutoCalibrationError, match="측정 결과가 없다"):
        latest_measure(tmp_path, "nobody", now=1000.0)
    with pytest.raises(AutoCalibrationError):
        latest_measure(tmp_path / "none", CAL_ID)


def test_physical_limits_are_only_a_suggestion():
    assert physical_limits_suggestion({"x": {"range_min": 1000, "range_max": 3048}}) == {"x": [-90.0, 90.0]}


# ---- 명령 연결
def args(**kw):
    base = {"sim": False, "auto": True, "measure_only": True, "save": False, "velocity": 200, "torque_limit": None, "from_file": None, "config": "config/robot.yaml"}
    return SimpleNamespace(**{**base, **kw})


CFG = SimpleNamespace(calibration_id=CAL_ID, port="COM_TEST")


def opener_for(bus, fpath):
    robot = SimpleNamespace(bus=bus, calibration_fpath=fpath)
    bus.disconnect = lambda disable_torque=True: None
    return lambda cfg: robot


def test_cli_rejects_sim_and_bad_flag_combinations(paths):
    said: list[str] = []
    for a in (args(sim=True), args(auto=False, measure_only=False), args(save=True)):
        assert acli.run_calibrate(a, CFG, input_fn=lambda _: "", out=said.append, runs_dir=paths.runs) == 2


def test_cli_measure_only_asks_first_then_measures(reference, paths, monkeypatch):
    arm = eeprom_arm()
    prompts: list[str] = []
    paths.cache.parent.mkdir(parents=True)
    paths.cache.write_text(json.dumps({n: dict(c) for n, c in reference.items()}), encoding="utf-8")
    said: list[str] = []
    real = acli.measure_only
    monkeypatch.setattr(acli, "measure_only", lambda *a, **k: real(*a, **k, clock=arm.clock, sleep=arm.sleep))
    code = acli.run_calibrate(args(), CFG, input_fn=lambda p: prompts.append(p) or "", out=said.append, opener=opener_for(arm, paths.cache), now=lambda: datetime(2026, 11, 4, 9, 0), config_dir=paths.repo.parent.parent, runs_dir=paths.runs)
    text = "\n".join(said)
    assert code == 0 and len(prompts) == 1 and all(w in text for w in ("클램프", "반경 50cm", "전원 스위치", "홈 자세가 아니라", "아래로 늘어진 쉬는 자세"))  # 안내를 띄우고 Enter를 받았다
    assert (paths.runs / "20261104-090000-autocal" / "measure.json").is_file() and "합격" in text
    assert_untouched(arm)


def test_cli_save_without_a_measurement_fails_and_with_one_writes(reference, paths, monkeypatch):
    said: list[str] = []
    bus = SaveBus({n: c["id"] for n, c in reference.items()}, old_motor_cal(reference))
    bus.disconnect = lambda disable_torque=True: None
    opener = opener_for(bus, paths.cache)
    monkeypatch.setattr("so101_studio.doctor.doctor", lambda **k: {"checks": [{"name": "캘리브레이션 대조", "state": "ok", "text": "6개 서보가 파일과 같다", "fix": ""}]})
    code = acli.run_calibrate(args(auto=False, measure_only=False, save=True), CFG, input_fn=lambda _: "", out=said.append, opener=opener, config_dir=paths.repo.parent.parent, runs_dir=paths.runs)
    assert code == 1 and "측정 결과가 없다" in "\n".join(said) and bus.writes == 0  # 측정 없이는 저장하지 않는다
    run = paths.runs / "20261104-0900-autocal"
    run.mkdir(parents=True)
    import time

    (run / "measure.json").write_text(json.dumps(passing_report(reference, finished=time.time())), encoding="utf-8")
    code = acli.run_calibrate(args(auto=False, measure_only=False, save=True), CFG, input_fn=lambda _: "", out=said.append, opener=opener, now=lambda: datetime(2026, 11, 4, 9, 30), config_dir=paths.repo.parent.parent, runs_dir=paths.runs)
    assert code == 0 and bus.writes == 1 and paths.repo.is_file() and "저장 뒤 doctor 대조: ok" in "\n".join(said)


# ---- doctor: 저장 결과가 보인다
def test_doctor_shows_the_last_autocal_save(reference, tmp_path, monkeypatch):
    from so101_studio import doctor as dr
    from so101_studio.core import ports as ports_mod

    config = Path("config/robot.yaml").resolve()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(ports_mod, "list_serial_ports", list)
    assert dr.doctor(probe=False, config_path=config)["autocal"] is None
    (tmp_path / "runs").mkdir()
    (tmp_path / "runs" / f"autocal-save-{CAL_ID}.json").write_text(json.dumps({"saved_at": "2026-11-04T09:30:00", "rows": [1, 2, 3, 4, 5, 6], "backup": "x.bak.json"}), encoding="utf-8")
    res = dr.doctor(probe=False, config_path=config)
    assert res["autocal"]["saved_at"] == "2026-11-04T09:30:00" and any(c["name"] == "자동 캘리브레이션" and c["state"] == "ok" for c in res["checks"])
    json.dumps(res)


def _measure_once_setup(reference, paths, monkeypatch):
    arm = eeprom_arm()
    paths.cache.parent.mkdir(parents=True)
    paths.cache.write_text(json.dumps({n: dict(c) for n, c in reference.items()}), encoding="utf-8")
    real = acli.measure_only
    monkeypatch.setattr(acli, "measure_only", lambda *a, **k: real(*a, **k, clock=arm.clock, sleep=arm.sleep))
    return arm


def test_measure_once_returns_the_report_with_run_dir_and_progress_lines(reference, paths, monkeypatch):
    """REQ-052: GUI 마법사가 부르는 공개 함수. 입력을 기다리지 않고, 보고서에 run_dir를 더하고, 관절을 시작할 때 진행 한 줄을 낸다."""
    arm = _measure_once_setup(reference, paths, monkeypatch)
    said: list[str] = []
    report = acli.measure_once(CFG, out=said.append, opener=opener_for(arm, paths.cache), now=lambda: datetime(2026, 11, 4, 9, 0), config_dir=paths.repo.parent.parent, runs_dir=paths.runs)
    assert report["kind"] == "autocal-measure" and report["passed"] and report["run_dir"] == str(paths.runs / "20261104-090000-autocal")
    assert (paths.runs / "20261104-090000-autocal" / "measure.json").is_file()
    assert any(line.startswith("측정 중(") for line in said) and "합격" in "\n".join(said)
    assert_untouched(arm)


def test_measure_once_stop_ends_with_autocalibration_error_and_restores_registers(reference, paths, monkeypatch):
    arm = _measure_once_setup(reference, paths, monkeypatch)
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] >= 5  # 몇 틱 움직인 뒤 중단 버튼

    with pytest.raises(AutoCalibrationError, match="중단했습니다"):
        acli.measure_once(CFG, stop=stop, opener=opener_for(arm, paths.cache), now=lambda: datetime(2026, 11, 4, 9, 0), config_dir=paths.repo.parent.parent, runs_dir=paths.runs)
    assert calls["n"] >= 5
    assert_untouched(arm)  # 되돌린 뒤 EEPROM·캘리브레이션 레지스터가 처음 그대로
    assert not (paths.runs / "20261104-090000-autocal" / "measure.json").exists()  # 보고서는 남기지 않는다
