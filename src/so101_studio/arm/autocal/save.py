"""`so101 calibrate --save`: 측정만 모드가 합격한 결과만 EEPROM·LeRobot 캐시·config/calibration에 쓴다 (decisions/0028).
팔을 움직이지 않는다(측정은 이미 끝났다). 팔 파일의 physical_limits_deg는 제안만 출력하고 덮지 않는다."""
import json
import time
from datetime import datetime
from pathlib import Path

from lerobot.motors.motors_bus import MotorCalibration

from .engine import AutoCalibrationError
from .measure import range_deg

MAX_AGE_H = 24.0  # 측정 결과가 이보다 오래되면 다시 측정한다
FIELDS = ("homing_offset", "range_min", "range_max")


def measure_files(runs_dir: Path) -> list[Path]:
    return sorted(runs_dir.glob("*/measure.json"), key=lambda p: p.stat().st_mtime) if runs_dir.is_dir() else []


def latest_measure(runs_dir: Path, calibration_id: str, max_age_h: float = MAX_AGE_H, now: float | None = None) -> Path:
    """가장 최근의 이 팔(calibration_id) 측정 결과. 없거나 너무 오래됐으면 AutoCalibrationError."""
    now = time.time() if now is None else now
    for p in reversed(measure_files(runs_dir)):
        data = json.loads(p.read_text(encoding="utf-8"))
        if data.get("calibration_id") != calibration_id:
            continue
        if now - data["finished"] > max_age_h * 3600:
            raise AutoCalibrationError(f"가장 최근 측정({p.parent.name})이 {max_age_h:g}시간보다 오래됐다. 측정만 모드를 다시 한다")
        return p
    raise AutoCalibrationError(f"{calibration_id}의 측정 결과가 없다. 먼저 so101 calibrate --auto --measure-only를 한다")


def physical_limits_suggestion(measured: dict[str, dict]) -> dict[str, list[float]]:
    """0°가 범위 가운데이므로 새 범위의 절반씩. 제안값일 뿐이다(팔 파일은 사람이 고친다)."""
    return {n: [round(-range_deg(c) / 2, 1), round(range_deg(c) / 2, 1)] for n, c in measured.items()}


def _dump(measured: dict[str, dict]) -> str:
    return json.dumps({n: {"id": c["id"], "drive_mode": c["drive_mode"], "homing_offset": c["homing_offset"], "range_min": c["range_min"], "range_max": c["range_max"]}
                       for n, c in measured.items()}, indent=4)


def save_calibration(bus, report: dict, *, cache_path: Path, repo_path: Path, runs_dir: Path, now: datetime | None = None, out=print) -> dict:
    """합격한 측정 결과를 쓴다. 순서: 검증 -> 이전 파일 .bak.json -> EEPROM 쓰기·다시 읽기 확인(어긋나면 이전 값으로 되돌리고 중단) -> 두 파일 쓰기 -> 기록."""
    cal_id = report["calibration_id"]
    if not report.get("passed"):
        why = "비교할 기존 캘리브레이션 파일이 없어" if not report.get("has_old", True) else f"기존 값과 차이가 허용(±{report.get('pass_deg', 3):g}°)을 넘어"
        raise AutoCalibrationError(f"측정만 모드가 합격하지 않았다({why}): 저장하지 않는다")
    measured = report["measured"]
    ids = {n: m.id for n, m in bus.motors.items()}
    if {n: c["id"] for n, c in measured.items()} != ids:
        raise AutoCalibrationError(f"측정 결과의 서보 ID가 연결된 팔과 다르다: {ids}")
    now = now or datetime.now()
    stamp = now.strftime("%Y%m%d-%H%M")
    previous = next((p for p in (cache_path, repo_path) if p.is_file()), None)
    backup = None
    if previous:
        backup = repo_path.parent / f"{cal_id}.{stamp}.bak.json"
        repo_path.parent.mkdir(parents=True, exist_ok=True)
        backup.write_text(previous.read_text(encoding="utf-8"), encoding="utf-8")
        out(f"이전 캘리브레이션을 {backup}에 남겼다")
    before = bus.read_calibration()
    new = {n: MotorCalibration(**c) for n, c in measured.items()}
    bus.disable_torque()  # 토크를 끄고 Lock을 푼다: EEPROM에 쓰려면 필요하다
    try:
        bus.write_calibration(new)
        back = bus.read_calibration()
        wrong = [f"{n} {f}: {getattr(back[n], f)} (쓴 값 {getattr(new[n], f)})" for n in new for f in FIELDS if getattr(back[n], f) != getattr(new[n], f)]
        if wrong:
            raise AutoCalibrationError("EEPROM에 쓴 값이 다시 읽은 값과 다르다: " + "; ".join(wrong))
    except Exception:
        out("EEPROM 쓰기를 확인하지 못해 이전 값으로 되돌린다")
        bus.write_calibration(before)
        raise
    for path in (cache_path, repo_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(_dump(measured), encoding="utf-8")
    suggestion = physical_limits_suggestion(measured)
    record = {"kind": "autocal-save", "calibration_id": cal_id, "saved_at": now.isoformat(timespec="seconds"), "backup": str(backup) if backup else None,
              "cache_path": str(cache_path), "repo_path": str(repo_path), "measured_at": report["finished"], "rows": report["rows"],
              "mode": report.get("mode", "single"), "runs": report.get("runs", []),
              "physical_limits_deg_suggestion": suggestion}
    runs_dir.mkdir(parents=True, exist_ok=True)
    text = json.dumps(record, indent=2, ensure_ascii=False)
    (runs_dir / f"autocal-save-{cal_id}.json").write_text(text, encoding="utf-8")  # so101 doctor가 읽는다
    out(f"저장했다: EEPROM, {cache_path}, {repo_path}")
    out("캘리브레이션 파일이 바뀌었으므로 0° 자세를 다시 확인해야 움직인다: so101 calibrate --verify-zero")
    out("팔 파일 physical_limits_deg 제안(자동으로 바꾸지 않는다, 사람이 확인해 고친다):\n" + "\n".join(f"  {n}: {v}" for n, v in suggestion.items()))
    return record
