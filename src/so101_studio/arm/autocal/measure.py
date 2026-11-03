"""`so101 calibrate --auto --measure-only`: 서보 EEPROM에 쓰지 않고 측정만 한다 (decisions/0028).
실행 중 레지스터 변경은 엔진이 Lock=1로 RAM에만 남기고, 끝나면 여기서 이전 Homing_Offset·Min/Max_Position_Limit를 RAM에 되돌린다."""
import json
import time
from dataclasses import asdict
from pathlib import Path

from .engine import (
    FULL_TURN,
    AutoCalibrationConfig,
    AutoCalibrationError,
    FeetechAutoCalibrator,
    steps_to_deg,
    wrap,
)
from .guard import MAX_TEMP, MIN_VOLT, GuardedBus, vitals_problem
from .plan import SO_ARM_PLAN

CAL_REGISTERS = ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit")
PASS_DEG = 3.0  # 기존 값과의 차이 허용(°): 범위와 0° 위치 모두 (AC4)

GUIDE = """자동 캘리브레이션을 시작하기 전에 확인한다:
  1. 팔을 시작 자세로 둔다: 접힌 자세(윗팔을 뒤로 젖히고 아래팔을 그 위로 접음), 그리퍼는 닫고 아래로 향하게.
  2. 팔의 받침을 클램프로 책상에 단단히 고정한다.
  3. 팔 주변 반경 50cm를 비운다. 팔이 앞으로 뻗고 받침이 좌우로 약 100° 돈다.
  4. 전원 스위치 옆에 사람이 있다. 이상하면 바로 끈다 (Ctrl+C도 레지스터를 되돌린다).
약 2분 걸린다. 서보 EEPROM에는 쓰지 않는다(측정만)."""


def _read(bus, register: str) -> dict[str, int]:
    return bus.sync_read(register, normalize=False, num_retry=3)


def snapshot(bus) -> dict[str, dict[str, int]]:
    return {register: _read(bus, register) for register in ("Lock", *CAL_REGISTERS)}


def restore_ram(bus, saved: dict[str, dict[str, int]]) -> list[str]:
    """이전 캘리브레이션 레지스터와 Lock을 되돌리고 다시 읽어 확인한다. 실패한 항목 목록을 돌려준다(빈 목록이면 성공).
    Lock=1로 쓰므로 EEPROM에는 아무것도 쓰이지 않는다."""
    problems: list[str] = []
    names = list(bus.motors)
    for name in names:
        for register, value in (("Torque_Enable", 0), ("Lock", 1), *((r, saved[r][name]) for r in CAL_REGISTERS)):
            try:
                bus.write(register, name, value, normalize=False, num_retry=3)
            except Exception as e:  # noqa: BLE001 한 항목이 실패해도 나머지는 되돌린다
                problems.append(f"{name} {register}: {e}")
    for register in CAL_REGISTERS:
        try:
            now = _read(bus, register)
        except Exception as e:  # noqa: BLE001
            problems.append(f"{register} 읽기: {e}")
            continue
        problems += [f"{n} {register}: {now[n]} (원래 {saved[register][n]})" for n in names if now[n] != saved[register][n]]
    for name in names:
        try:
            bus.write("Lock", name, saved["Lock"][name], normalize=False, num_retry=3)
        except Exception as e:  # noqa: BLE001
            problems.append(f"{name} Lock: {e}")
    return problems


def range_deg(cal: dict) -> float:
    return (cal["range_max"] - cal["range_min"]) * 360 / FULL_TURN


def zero_raw(cal: dict) -> int:
    """관절 0°(범위의 가운데)에 해당하는 서보 원시 위치. Present = Actual - Homing_Offset 이므로 Actual = Present + Homing_Offset."""
    return round((cal["range_min"] + cal["range_max"]) / 2 + cal["homing_offset"]) % FULL_TURN


def compare(old: dict | None, new: dict[str, dict]) -> list[dict]:
    """관절별 비교표. old가 없으면 차이는 None. ok는 범위와 0° 위치 차이가 모두 PASS_DEG 이내인지."""
    rows = []
    for name, n in new.items():
        o = (old or {}).get(name)
        row = {"joint": name, "new_range_deg": round(range_deg(n), 1), "old_range_deg": None, "range_diff_deg": None, "zero_diff_deg": None, "ok": None}
        if o:
            row["old_range_deg"] = round(range_deg(o), 1)
            row["range_diff_deg"] = round(range_deg(n) - range_deg(o), 1)
            row["zero_diff_deg"] = round(steps_to_deg(wrap(zero_raw(n) - zero_raw(o))), 1)
            row["ok"] = abs(row["range_diff_deg"]) <= PASS_DEG and abs(row["zero_diff_deg"]) <= PASS_DEG
        rows.append(row)
    return rows


def format_table(rows: list[dict]) -> str:
    lines = [f"{'관절':<14}{'기존 범위':>10}{'새 범위':>10}{'범위 차이':>10}{'0° 차이':>9}  판정"]
    for r in rows:
        fmt = lambda v, w: f"{v:>{w}.1f}" if v is not None else f"{'-':>{w}}"
        mark = "-" if r["ok"] is None else ("합격" if r["ok"] else "벗어남")
        lines.append(f"{r['joint']:<14}{fmt(r['old_range_deg'], 10)}{fmt(r['new_range_deg'], 10)}{fmt(r['range_diff_deg'], 10)}{fmt(r['zero_diff_deg'], 9)}  {mark}")
    return "\n".join(lines)


def measure_only(bus, calibration_id: str, old_cal: dict | None, *, velocity: int = 200, torque_limit: int | None = None, out=print,
                 run_dir: Path | None = None, min_volt: float = MIN_VOLT, max_temp: float = MAX_TEMP, clock=time.monotonic, sleep=time.sleep) -> dict:
    """측정만 한다. 보고서 dict를 돌려주고 run_dir/measure.json에 남긴다. 실패하면 AutoCalibrationError(레지스터는 되돌린 뒤)."""
    if problem := vitals_problem(bus, min_volt, max_temp):
        raise AutoCalibrationError(problem)
    saved = snapshot(bus)
    guarded = GuardedBus(bus, min_volt, max_temp, clock=clock)
    cfg = AutoCalibrationConfig(velocity=velocity, torque_limit=torque_limit)
    calibrator = FeetechAutoCalibrator(guarded, SO_ARM_PLAN, cfg, clock=clock, sleep=sleep)
    started = time.time()
    try:
        result = calibrator.run()
    finally:
        out("측정 결과를 버리고 이전 캘리브레이션 레지스터를 되돌린다(RAM)")
        problems = restore_ram(bus, saved)
        if problems:
            out("되돌리기 실패: " + "; ".join(problems) + "\n전원을 껐다 켜면 서보의 이전 설정(EEPROM)이 돌아온다")
    if problems:
        raise AutoCalibrationError("측정은 끝났지만 레지스터를 되돌리지 못했다: " + "; ".join(problems))
    new = {name: {k: v for k, v in asdict(cal).items()} for name, cal in result.items()}
    rows = compare(old_cal, new)
    report = {"kind": "autocal-measure", "calibration_id": calibration_id, "started": started, "finished": time.time(),
              "velocity": velocity, "torque_limit": torque_limit or SO_ARM_PLAN.torque_limit, "measured": new, "rows": rows,
              "passed": old_cal is not None and all(r["ok"] for r in rows), "has_old": old_cal is not None, "pass_deg": PASS_DEG,
              "summary": calibrator.summary()}
    out(calibrator.summary())
    out(format_table(rows))
    out(f"1회 합격: 기존 캘리브레이션과 모든 관절이 ±{PASS_DEG:g}° 이내" if report["passed"] else f"1회 합격 아님: 기존 값과 {PASS_DEG:g}° 넘게 다르거나 비교할 기존 파일이 없다(3회 반복 합격으로 저장할 수 있다)")
    if run_dir:
        from . import (
            repeat,  # measure를 먼저 import하는 repeat와 서로 물리지 않게 여기서 가져온다
        )

        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "measure.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        out(f"기록: {run_dir / 'measure.json'}")
        status = repeat.repeat_status(run_dir.parent, calibration_id, report["finished"])  # 이 측정을 포함한다
        report["repeat"] = {k: status[k] for k in ("count", "max_diff_deg", "passed")}
        (run_dir / "measure.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        out(repeat.progress_line(status))
    return report
