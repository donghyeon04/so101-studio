"""`so101 calibrate` 명령 연결. ArmService·ArmDriver.connect는 쓰지 않는다(connect는 PID·가속을 서보에 쓴다). LeRobot 로봇의 버스만 연다."""
import json
from datetime import datetime
from pathlib import Path

from .engine import AutoCalibrationError
from .measure import GUIDE, compare, format_table, measure_only
from .repeat import progress_line, repeat_status
from .save import latest_measure, save_calibration

USAGE = "calibrate는 --auto --measure-only(측정만), --save(합격한 측정 결과를 저장), --verify-zero(0° 자세 확인) 중 하나와 함께 쓴다"


def open_robot(cfg):
    """LeRobot SO-101 팔로워의 버스를 연다(읽기 연결만: 모터 확인). 캘리브레이션 파일 경로도 이 객체가 안다."""
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=cfg.port, id=cfg.calibration_id))
    robot.bus.connect()
    return robot


def _old_calibration(robot, repo_path: Path) -> dict | None:
    for p in (robot.calibration_fpath, repo_path):
        if Path(p).is_file():
            return json.loads(Path(p).read_text(encoding="utf-8"))
    return None


def run_calibrate(a, cfg, *, input_fn=input, out=print, opener=open_robot, now=datetime.now, config_dir: Path | None = None, runs_dir: Path = Path("runs")) -> int:
    if a.sim:
        out("calibrate는 실제 팔 전용이다(--sim에서는 쓸 수 없다)")
        return 2
    if not (a.auto and a.measure_only) and not a.save:
        out(USAGE)
        return 2
    if a.auto and a.measure_only and a.save:
        out("--measure-only와 --save는 같이 쓰지 않는다(측정이 합격한 뒤에 따로 저장한다)")
        return 2
    repo_path = (config_dir or Path("config")) / "calibration" / f"{cfg.calibration_id}.json"
    try:
        if a.save:
            return _save(a, cfg, input_fn, out, opener, now, repo_path, runs_dir)
        return _measure(a, cfg, input_fn, out, opener, now, repo_path, runs_dir)
    except AutoCalibrationError as e:
        out(f"중단: {e}")
        return 1
    except KeyboardInterrupt:
        out("Ctrl+C: 중단했다(레지스터는 되돌렸다)")
        return 1


def _measure(a, cfg, input_fn, out, opener, now, repo_path, runs_dir) -> int:
    out(GUIDE)
    input_fn("준비됐으면 Enter(시작), 그만두려면 Ctrl+C... ")
    robot = opener(cfg)
    try:
        report = measure_only(robot.bus, cfg.calibration_id, _old_calibration(robot, repo_path), velocity=a.velocity, torque_limit=a.torque_limit,
                              out=out, run_dir=runs_dir / f"{now():%Y%m%d-%H%M%S}-autocal")
    finally:
        robot.bus.disconnect(disable_torque=False)
    return 0 if report["passed"] or report.get("repeat", {}).get("passed") else 1  # 1회 합격 또는 반복 합격


def _is_yes(text: str) -> bool:
    return text.strip().lower() == "yes"


def _choose(a, cfg, runs_dir, old, out, input_fn, now) -> dict:
    """저장할 결과를 정한다. 1회 합격(기존 파일과 ±3° 이내)이면 그 측정 하나, 아니면 반복 합격(최근 24시간 3개가 서로 1° 이내)의 중앙값.
    반복 합격이고 기존 파일이 있으면 차이표를 보이고 사람이 yes를 입력해야 한다. 어느 쪽도 아니면 AutoCalibrationError."""
    path = Path(a.from_file) if a.from_file else latest_measure(runs_dir, cfg.calibration_id, now=now().timestamp())
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("passed"):
        out(f"저장할 측정 결과: {path} (1회 합격: 기존 파일과 ±{report.get('pass_deg', 3):g}° 이내)")
        input_fn("서보 EEPROM과 캘리브레이션 파일에 씁니다(이전 파일은 .bak.json으로 남김). Enter 계속, Ctrl+C 취소... ")
        return report
    status = repeat_status(runs_dir, cfg.calibration_id, now().timestamp())
    out(progress_line(status))
    if not status["passed"]:
        raise AutoCalibrationError("저장할 수 없다: 1회 합격(기존 파일과 ±3° 이내)도, 반복 합격(최근 24시간의 같은 팔 측정 3개가 서로 1° 이내)도 아니다. "
                                   "측정만 모드를 더 한다")
    merged = {"kind": "autocal-measure", "calibration_id": cfg.calibration_id, "finished": max(r["finished"] for r in status["reports"]), "passed": True,
              "has_old": old is not None, "pass_deg": 3.0, "measured": status["median"], "rows": compare(old, status["median"]), "mode": "repeat",
              "runs": [str(p) for p in status["used"]], "max_diff_deg": status["max_diff_deg"]}
    out(f"반복 합격: 최근 3개({', '.join(p.parent.name for p in status['used'])})의 관절별 중앙값을 저장한다")
    if old is not None:
        out(format_table(merged["rows"]))
        if not _is_yes(input_fn("기존 캘리브레이션과 위 표만큼 다릅니다. 서보 EEPROM과 파일을 바꾸려면 yes를 입력(이전 파일은 .bak.json으로 남김): ")):
            raise AutoCalibrationError("yes가 아니라서 저장하지 않았다")
    else:
        input_fn("기존 캘리브레이션 파일이 없다. 서보 EEPROM과 캘리브레이션 파일에 씁니다. Enter 계속, Ctrl+C 취소... ")
    return merged


def _save(a, cfg, input_fn, out, opener, now, repo_path, runs_dir) -> int:
    robot = opener(cfg)
    try:
        report = _choose(a, cfg, runs_dir, _old_calibration(robot, repo_path), out, input_fn, now)
        save_calibration(robot.bus, report, cache_path=Path(robot.calibration_fpath), repo_path=repo_path, runs_dir=runs_dir, now=now(), out=out)
    finally:
        robot.bus.disconnect(disable_torque=False)
    from ...doctor import doctor  # 저장한 뒤 서보 값과 파일을 doctor로 다시 대조한다

    res = doctor(port=cfg.port, config_path=Path(a.config))
    check = next((c for c in res["checks"] if c["name"] == "캘리브레이션 대조"), None)
    out(f"저장 뒤 doctor 대조: {check['state'] if check else '?'} {check['text'] if check else ''}")
    return 0 if check and check["state"] == "ok" else 1
