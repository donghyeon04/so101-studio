"""실측 정답 쌍 기록 (ultrasonic-spec 8.4, 결정 0072): 사람이 놓은 물체의 정답을 그 자리에서 잰 scans·checks와 짝지어 남긴다.

폴더 runs/ultrasonic-truth/<YYYYmmdd-HHMMSS>-<시험>/ (실행 폴더 이름 규칙 밖이라 prune_runs가 지우지 않는다, REQ-094)
  truth.json     {"format": 1, "test", "commit", "dirty", "sim", "started", "note", "config"(그때 초음파 설정),
                  "placements": [{"n", "rep", "objects": [{id, x_mm, y_mm, angle_deg, width_mm, length_mm, height_mm}],
                                  "scans": ["scans/001.json"], "checks": ["checks/001.json"]}]}
  scans/NNN.json  UltrasonicSource가 scan·rescan마다 남김(바퀴별 원자료, 기준값, 호, 후보, 지운 각도)
  checks/NNN.json UltrasonicSource가 확인마다 남김(구간, PoseSample, 팔 센서 측정, 점, 결과)
runs/ultrasonic-truth/index.jsonl  실행마다 한 줄 {"run", "test", "commit", "objects", "placements", "sim"}
좌표는 작업대 좌표(mm, 결정 0013), 높이는 작업대 면 위, angle_deg는 긴 축의 x축 기준 0~180(정사각형·모름은 null)."""
import json
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path("runs/ultrasonic-truth")
FORMAT = 1


def _git() -> tuple[str, bool]:
    try:
        run = lambda *a: subprocess.run(["git", *a], check=False, capture_output=True, text=True, timeout=5).stdout.strip()
        return run("rev-parse", "--short", "HEAD") or "?", bool(run("status", "--porcelain"))
    except OSError:
        return "?", False


class TruthRun:
    """한 실측 실행. source.record_dir에 self.dir을 주고, 물체를 놓고 잴 때마다 add()를 부른다. 끝나면 close()(index 한 줄)."""

    def __init__(self, test: str, cfg=None, *, sim=False, note="", root=None):
        self.root = Path(root or ROOT)
        self.dir = self.root / f"{datetime.now():%Y%m%d-%H%M%S}-{test}"
        self.dir.mkdir(parents=True, exist_ok=False)
        commit, dirty = _git()
        self.data = {"format": FORMAT, "test": test, "commit": commit, "dirty": dirty, "sim": sim,
                     "started": datetime.now().isoformat(timespec="seconds"), "note": note, "config": cfg, "placements": []}
        self._closed = False
        self._save()

    def add(self, objects: list[dict], files: list[str], rep: int = 1) -> dict:
        """objects = 정답 물체(위 키), files = 이번 자리에서 source가 새로 남긴 파일(source.saved의 증가분)."""
        p = {"n": len(self.data["placements"]) + 1, "rep": rep, "objects": objects,
             "scans": [f for f in files if f.startswith("scans/")], "checks": [f for f in files if f.startswith("checks/")]}
        self.data["placements"].append(p)
        self._save()  # 중간에 끊겨도 그때까지는 남는다
        return p

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        pl = self.data["placements"]
        if not pl:  # 자리 하나도 못 남긴 실행은 색인에 안 올린다(폴더와 truth.json은 남는다)
            return
        line = {"run": self.dir.as_posix(), "test": self.data["test"], "commit": self.data["commit"],
                "objects": sum(len(p["objects"]) for p in pl), "placements": len(pl), "sim": self.data["sim"]}
        with (self.root / "index.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(line, ensure_ascii=False) + "\n")

    def _save(self) -> None:
        (self.dir / "truth.json").write_text(json.dumps(self.data, ensure_ascii=False, indent=1), encoding="utf-8")


def obj(x_mm, y_mm, angle_deg=None, width_mm=0.0, length_mm=0.0, height_mm=0.0, id="P1") -> dict:
    return {"id": id, "x_mm": x_mm, "y_mm": y_mm, "angle_deg": angle_deg, "width_mm": width_mm,
            "length_mm": length_mm, "height_mm": height_mm}


def pairs(run_dir):
    """(자리, [scan dict], [check dict])를 차례로 낸다. 분석·2차 추정기 학습이 읽는 입구."""
    run_dir = Path(run_dir)
    truth = json.loads((run_dir / "truth.json").read_text(encoding="utf-8"))
    read = lambda f: json.loads((run_dir / f).read_text(encoding="utf-8"))
    for p in truth["placements"]:
        yield p, [read(f) for f in p["scans"]], [read(f) for f in p["checks"]]


def index(root=None) -> list[dict]:
    f = Path(root or ROOT) / "index.jsonl"
    return [json.loads(s) for s in f.read_text(encoding="utf-8").splitlines() if s.strip()] if f.exists() else []
