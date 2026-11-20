"""캘리브레이션 파일 찾기와 0° 자세 확인 기록 (decisions/0029). lerobot을 import하지 않는다."""
import hashlib
import json
import re
from datetime import date
from pathlib import Path

import yaml

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
    checked = checked_record(cfg)
    if cal is None:
        return {"state": "needed", "reason": f"캘리브레이션 파일({cfg.calibration_id})이 없다", "date": None, "sha256": None}
    now = calibration_sha256(cal)
    if not checked.get("sha256"):
        return {"state": "needed", "reason": "0° 자세를 확인한 기록(calibration_checked)이 팔 파일에 없다", "date": None, "sha256": now}
    if checked["sha256"] != now:
        return {"state": "needed", "reason": "캘리브레이션 파일이 마지막으로 0° 자세를 확인한 때와 다르다", "date": str(checked.get("date")), "sha256": now}
    return {"state": "checked", "reason": None, "date": str(checked.get("date")), "sha256": now}


def checked_record(cfg) -> dict:
    """지금 calibration_id의 0° 확인 기록 {date, sha256}(없으면 {}). 옛 형식 {date, sha256} 하나면 그것(load_robot을 거치지 않은 설정)."""
    c = getattr(cfg, "calibration_checked", None) or {}
    return c if "sha256" in c else (c.get(cfg.calibration_id) or {})


HEAD = "calibration_checked:         # so101 calibrate --verify-zero가 기록 (decisions/0029). 캘리브레이션 파일이 바뀌면 다시 확인해야 움직인다\n"


def _arm_checked(text: str) -> dict:
    """팔 파일 글의 calibration_checked를 id별 dict로(옛 형식은 그 파일 calibration_id의 기록)."""
    data = yaml.safe_load(text) or {}
    c = data.get("calibration_checked") or {}
    return {str(data.get("calibration_id", "")): c} if "sha256" in c else dict(c)


def _write_checked(arm_file: Path, text: str, recs: dict | None, old: dict | None = None) -> None:
    """calibration_checked 블록을 지우고 새로 쓴다. recs = id별, old = 옛 형식 하나. 둘 다 없으면 키째 뺀다."""
    text = re.sub(r"(?m)^calibration_checked:[^\n]*\n(?:[ \t]+[^\n]*\n)*", "", text).rstrip("\n") + "\n"
    if old is not None:
        block = HEAD + f'  date: "{old["date"]}"\n  sha256: "{old["sha256"]}"\n'
    elif recs:
        block = HEAD + "".join(f'  {k}:\n    date: "{v.get("date")}"\n    sha256: "{v.get("sha256")}"\n' for k, v in recs.items())
    else:
        block = ""
    arm_file.write_text(text + block, encoding="utf-8")


def record_checked(arm_file: Path, sha256: str, today: date | None = None, calibration_id: str | None = None) -> None:
    """팔 파일에 calibration_checked를 쓴다(이미 있으면 바꾼다). 다른 줄과 주석은 건드리지 않는다.
    calibration_id를 주면 id별 형식 {<id>: {date, sha256}}으로 그 id만 바꾸고 다른 id의 기록은 남긴다(arm-software-spec 7.1 그리퍼별). 안 주면 옛 형식 하나."""
    text = arm_file.read_text(encoding="utf-8").replace("\r\n", "\n")
    rec = {"date": (today or date.today()).isoformat(), "sha256": sha256}
    if calibration_id is None:
        _write_checked(arm_file, text, None, rec)
    else:
        _write_checked(arm_file, text, {**_arm_checked(text), calibration_id: rec})


def forget_checked(arm_file: Path, calibration_ids) -> None:
    """이 id들의 0° 확인 기록을 지운다(다른 그리퍼 파일의 서보 1~5를 바꾼 뒤, 7.1). 남는 기록이 없으면 키째 지운다."""
    text = arm_file.read_text(encoding="utf-8").replace("\r\n", "\n")
    drop = set(calibration_ids)
    _write_checked(arm_file, text, {k: v for k, v in _arm_checked(text).items() if k not in drop})
