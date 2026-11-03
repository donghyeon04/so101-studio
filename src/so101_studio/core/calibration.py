"""캘리브레이션 파일 찾기와 0° 자세 확인 기록 (decisions/0029). lerobot을 import하지 않는다."""
import hashlib
import json
import re
from datetime import date
from pathlib import Path

HF_CACHE = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"


def find_calibration(calibration_id: str, config_dir: Path) -> tuple[dict | None, str]:
    """lerobot이 쓰는 캘리브레이션 파일(HF 캐시)을 먼저, 없으면 레포 사본(config/calibration)."""
    for p in (HF_CACHE / f"{calibration_id}.json", config_dir / "calibration" / f"{calibration_id}.json"):
        if p.exists():
            return json.loads(p.read_text(encoding="utf-8")), str(p)
    return None, ""


def calibration_sha256(cal: dict) -> str:
    """캘리브레이션 내용의 해시. 파일의 들여쓰기·줄바꿈이 달라도 같은 값이 나오게 정규화한 JSON으로 만든다."""
    return hashlib.sha256(json.dumps(cal, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def zero_check(cfg, config_dir: Path) -> dict:
    """팔 파일의 calibration_checked가 지금 캘리브레이션 파일과 맞는지. {"state": "checked"|"needed", "reason", "date", "sha256"}.
    needed이면 움직이는 명령은 locked다(so101 calibrate --verify-zero로 확인한다)."""
    cal, _path = find_calibration(cfg.calibration_id, config_dir) if cfg.calibration_id else (None, "")
    checked = getattr(cfg, "calibration_checked", None) or {}
    if cal is None:
        return {"state": "needed", "reason": f"캘리브레이션 파일({cfg.calibration_id})이 없다", "date": None, "sha256": None}
    now = calibration_sha256(cal)
    if not checked.get("sha256"):
        return {"state": "needed", "reason": "0° 자세를 확인한 기록(calibration_checked)이 팔 파일에 없다", "date": None, "sha256": now}
    if checked["sha256"] != now:
        return {"state": "needed", "reason": "캘리브레이션 파일이 마지막으로 0° 자세를 확인한 때와 다르다", "date": str(checked.get("date")), "sha256": now}
    return {"state": "checked", "reason": None, "date": str(checked.get("date")), "sha256": now}


def record_checked(arm_file: Path, sha256: str, today: date | None = None) -> None:
    """팔 파일에 calibration_checked를 쓴다(이미 있으면 바꾼다). 다른 줄과 주석은 건드리지 않는다."""
    text = arm_file.read_text(encoding="utf-8").replace("\r\n", "\n")
    text = re.sub(r"(?m)^calibration_checked:[^\n]*\n(?:[ \t]+[^\n]*\n)*", "", text).rstrip("\n") + "\n"
    block = (f'calibration_checked:         # so101 calibrate --verify-zero가 기록 (decisions/0029). 캘리브레이션 파일이 바뀌면 다시 확인해야 움직인다\n'
             f'  date: "{(today or date.today()).isoformat()}"\n  sha256: "{sha256}"\n')
    arm_file.write_text(text + block, encoding="utf-8")
