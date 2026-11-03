"""반복 합격 (decisions/0029): 기존 파일이 없거나 틀린 팔은 최근 24시간의 같은 팔 측정 3개가 서로 1° 이내면 합격이다.
측정 결과 파일(measure.json)은 엔진의 관절 허용 범위 검사를 통과했을 때만 만들어지므로(엔진이 범위 밖이면 오류로 끝난다), 있다는 것이 곧 검사 통과다."""
import json
import statistics
import time
from pathlib import Path

from .engine import FULL_TURN, steps_to_deg, wrap
from .measure import range_deg, zero_raw

REPEAT_COUNT = 3
REPEAT_DEG = 1.0  # 3회 사이 관절별 범위와 0° 위치 차이가 모두 이 값 이하
WINDOW_H = 24.0
FIELDS = ("homing_offset", "range_min", "range_max")


def recent_measures(runs_dir: Path, calibration_id: str, now: float | None = None, window_h: float = WINDOW_H) -> list[tuple[Path, dict]]:
    """runs/*/measure.json 가운데 같은 calibration_id이고 now에서 window_h 시간 안인 것, 오래된 순서."""
    now = time.time() if now is None else now
    found = []
    for p in runs_dir.glob("*/measure.json") if runs_dir.is_dir() else []:
        data = json.loads(p.read_text(encoding="utf-8"))
        if data.get("calibration_id") == calibration_id and 0 <= now - data["finished"] <= window_h * 3600:
            found.append((p, data))
    return sorted(found, key=lambda x: x[1]["finished"])


def spread_deg(reports: list[dict]) -> float:
    """측정들 사이 관절별 범위 차이와 0° 위치 차이(°) 가운데 가장 큰 값."""
    worst = 0.0
    for joint in reports[0]["measured"]:
        cals = [r["measured"][joint] for r in reports]
        ranges = [range_deg(c) for c in cals]
        z0 = zero_raw(cals[0])
        zeros = [steps_to_deg(wrap(zero_raw(c) - z0)) for c in cals]
        worst = max(worst, max(ranges) - min(ranges), max(zeros) - min(zeros))
    return worst


def median_calibration(reports: list[dict]) -> dict[str, dict]:
    """관절별 homing_offset, range_min, range_max의 중앙값(id와 drive_mode는 마지막 측정의 값)."""
    last = reports[-1]["measured"]
    return {j: {**last[j], **{f: int(statistics.median(r["measured"][j][f] for r in reports)) for f in FIELDS}} for j in last}


def repeat_status(runs_dir: Path, calibration_id: str, now: float | None = None) -> dict:
    """{"count": 24시간 안의 측정 수, "used": 가장 최근 3개의 경로, "max_diff_deg": 그 3개 사이 최대 차이(3개 미만이면 있는 만큼),
    "passed": 3개 이상이고 차이가 모두 REPEAT_DEG 이하, "median": 합격이면 중앙값 캘리브레이션, "reports": 쓴 측정 결과들}."""
    found = recent_measures(runs_dir, calibration_id, now)
    used = found[-REPEAT_COUNT:]
    reports = [d for _, d in used]
    diff = spread_deg(reports) if len(reports) >= 2 else 0.0
    passed = len(reports) >= REPEAT_COUNT and diff <= REPEAT_DEG + 1e-9
    return {"count": len(found), "used": [p for p, _ in used], "max_diff_deg": round(diff, 2), "passed": passed,
            "median": median_calibration(reports) if passed else None, "reports": reports}


def progress_line(status: dict) -> str:
    n = min(status["count"], REPEAT_COUNT)
    tail = "합격" if status["passed"] else (f"{REPEAT_COUNT}회가 모여야 판정한다" if status["count"] < REPEAT_COUNT else f"{REPEAT_DEG:g}° 넘게 달라 합격이 아니다")
    return f"반복 {n}/{REPEAT_COUNT}, 지금까지 최대 차이 {status['max_diff_deg']:.2f}° ({tail})"


__all__ = ["FULL_TURN", "REPEAT_COUNT", "REPEAT_DEG", "median_calibration", "progress_line", "recent_measures", "repeat_status", "spread_deg"]
