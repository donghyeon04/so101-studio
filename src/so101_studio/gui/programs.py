"""자세 즐겨찾기와 프로그램(결정 0044, UR PolyScope 프로그램 트리 방식): 저장한 자세와 순서 목록.
자세는 config/waypoints.yaml, 프로그램은 config/programs/<이름>.yaml. 실행은 프로그램 전체를 팔 동작 하나(action(arm) -> MotionResult)로 만들어
Backend의 실행 방식(시뮬레이션 먼저 등)과 안전 검사를 그대로 쓴다."""

import threading
from dataclasses import asdict, fields
from pathlib import Path

import yaml

from ..core.types import DetectedObject, JointState, MotionResult, ToolPose
from .param_meta import META

STEP_TEXT = {
    "waypoint": "자세로 이동", "move": "좌표로 이동", "gripper": "그리퍼", "pick_place": "집어 옮기기",
    "wait": "대기", "home": "홈", "replay": "교시 재생", "repeat": "반복",
}
# 단계 종류마다 필요한 값과 기본값. 화면이 새 단계를 만들 때도 쓴다
DEFAULTS = {
    "waypoint": {"name": ""},
    "move": {"x": 300.0, "y": 200.0, "h": 40.0, "yaw": 90.0, "pitch": -90.0},
    "gripper": {"value": 100.0},
    "pick_place": {"x": 300.0, "y": 200.0, "angle": 0.0, "width": 30.0, "length": 30.0, "height": 0.0, "place_x": 450.0, "place_y": 150.0},
    "wait": {"s": 1.0},
    "home": {},
    "replay": {"name": "", "speed": 1.0},
    "repeat": {"count": 2, "steps": []},
}
# 단계 값 -> 값 정보표 키 (gui-spec 12.7.1): 화면 칸과 validate가 같은 한계를 쓴다
STEP_META = {
    "x": "target.x", "y": "target.y", "place_x": "target.x", "place_y": "target.y", "h": "target.h", "yaw": "target.yaw", "pitch": "target.pitch",
    "value": "gripper.value", "angle": "obj.angle", "width": "obj.width", "length": "obj.length", "height": "obj.height",
    "s": "program.s", "speed": "replay.speed", "count": "program.count",
}
OK = MotionResult("ok", 0.0, ToolPose(0, 0, 0, 0))
JOINTS = [f.name for f in fields(JointState)]


def describe(step: dict) -> str:
    """목록에 보일 한 줄."""
    t = step["type"]
    text = STEP_TEXT.get(t, t)
    if t == "waypoint":
        return f"{text}: {step['name'] or '(고르기)'}"
    if t == "move":
        return f"{text}: x {step['x']:.0f}, y {step['y']:.0f}, 높이 {step['h']:.0f}"
    if t == "gripper":
        return f"{text}: {step['value']:.0f} ({'열기' if step['value'] >= 50 else '닫기'})"
    if t == "pick_place":
        return f"{text}: ({step['x']:.0f}, {step['y']:.0f}) → ({step['place_x']:.0f}, {step['place_y']:.0f})"
    if t == "wait":
        return f"{text}: {step['s']:g}초"
    if t == "replay":
        return f"{text}: {step['name'] or '(고르기)'} ×{step['speed']:g}"
    if t == "repeat":
        return f"{text}: {step['count']}번, 단계 {len(step['steps'])}개"
    return text


def validate(steps: list, waypoints: dict, depth: int = 0) -> str | None:
    """저장·실행 전에 확인한다. 문제가 있으면 사람이 읽을 이유."""
    for i, step in enumerate(steps, 1):
        t = step.get("type")
        if t not in DEFAULTS:
            return f"{i}번 단계: 모르는 종류 {t!r}"
        missing = [k for k in DEFAULTS[t] if k not in step]
        if missing:
            return f"{i}번 단계({STEP_TEXT[t]}): {', '.join(missing)} 값이 없습니다"
        for key, meta_key in STEP_META.items():  # 펼치지 않은 단계·파일에서 읽은 단계도 저장·실행 전에 막는다 (12.7.4)
            if key in step and key in DEFAULTS[t]:
                lo, hi = META[meta_key].hard
                try:
                    ok = lo <= float(step[key]) <= hi
                except (TypeError, ValueError):
                    ok = False
                if not ok:
                    return f"{i}번 단계: {key} 허용 {lo:g}~{hi:g} 밖"
        if t == "waypoint" and step["name"] not in waypoints:
            return f"{i}번 단계: 저장된 자세 '{step['name']}'이 없습니다"
        if t == "replay" and not step["name"]:
            return f"{i}번 단계: 재생할 교시를 고르세요"
        if t == "wait" and not 0 <= float(step["s"]) <= 600:
            return f"{i}번 단계: 대기는 0~600초"
        if t == "repeat":
            if depth:
                return f"{i}번 단계: 반복 안에 반복은 넣을 수 없습니다"
            if not 1 <= int(step["count"]) <= 1000:
                return f"{i}번 단계: 반복 횟수는 1~1000"
            reason = validate(step["steps"], waypoints, depth + 1)
            if reason:
                return f"{i}번 반복 안 {reason}"
    return None


def flatten(steps: list) -> list[tuple[int, int | None, dict]]:
    """실행 순서대로 (바깥 번호, 반복 안 번호 | None, 단계). 반복은 펼친다."""
    out = []
    for i, step in enumerate(steps):
        if step["type"] == "repeat":
            for _ in range(int(step["count"])):
                out += [(i, j, inner) for j, inner in enumerate(step["steps"])]
        else:
            out.append((i, None, step))
    return out


class Run:
    """실행 중인 프로그램의 진행·정지·한 단계씩. 화면 타이머가 index를 읽어 강조한다."""

    def __init__(self, step_mode: bool = False):
        self.index: tuple[int, int | None] | None = None
        self.done = 0
        self.total = 0
        self.stop = threading.Event()
        self.go = threading.Event()  # 한 단계씩: 다음 단계를 허락한다
        self.step_mode = step_mode
        self.reason: str | None = None
        self.active = False  # 팔이 이 프로그램을 돌리는 중(시뮬레이션 먼저의 실제 팔 확인 실행 포함)

    def next_step(self) -> None:
        self.go.set()


def action(steps: list, waypoints: dict, run: Run, is_sim):
    """프로그램 전체를 팔 하나에 거는 동작으로. 단계가 ok가 아니면 거기서 멈추고 그 결과를 돌려준다."""
    order = flatten(steps)

    def do(arm, step: dict) -> MotionResult:
        t = step["type"]
        if t == "waypoint":
            return arm.move_joints(JointState(**{k: float(v) for k, v in waypoints[step["name"]]["joints"].items()}))
        if t == "move":
            return arm.move_tool(arm.point_pose(step["x"], step["y"], step["h"], step["yaw"], pitch=min(0.0, max(-90.0, step["pitch"]))))
        if t == "gripper":
            return arm.set_gripper(float(step["value"]))
        if t == "home":
            return arm.home()
        if t == "replay":
            return arm.replay(step["name"], float(step["speed"]))
        if t == "wait":
            return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0)) if run.stop.wait(float(step["s"])) else OK
        if t == "pick_place":
            obj = DetectedObject("program-pick", float(step["x"]), float(step["y"]), float(step["angle"]), float(step["width"]),
                                 float(step["length"]), 1.0, "manual", height_mm=float(step["height"]))
            r = arm.pick_and_place(obj)
            if r.status == "ok" or (r.status == "no_object" and is_sim(arm)):  # 시뮬레이션은 쥘 물체가 없다(직접 조작 집어 옮기기와 같다)
                return arm.place(at=(float(step["place_x"]), float(step["place_y"])))
            return r
        raise ValueError(f"모르는 단계 {t}")

    def act(arm):
        run.active = True
        try:
            return steps_on(arm)
        finally:
            run.active = False

    def steps_on(arm):
        run.total = len(order)
        for n, (i, j, step) in enumerate(order):
            if run.stop.is_set():
                return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0))
            if run.step_mode and n:
                run.go.clear()
                while not run.go.wait(0.1):
                    if run.stop.is_set():
                        return MotionResult("stopped", 0.0, ToolPose(0, 0, 0, 0))
            run.index, run.done = (i, j), n
            r = do(arm, step)
            if r.status != "ok":
                return r
        run.done, run.index = len(order), None
        return OK

    return act


# ---- 저장
def load_waypoints(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else None
    return data or {}


def save_waypoints(path: Path, waypoints: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# 자세 즐겨찾기(결정 0044). GUI 프로그램 화면이 쓴다. joints는 도, 그리퍼는 값(0~100)\n"
                    + yaml.safe_dump(waypoints, allow_unicode=True, sort_keys=False), encoding="utf-8")


def pose_entry(joints: JointState, tool: ToolPose, favorite: bool = False) -> dict:
    return {"joints": {k: round(v, 2) for k, v in asdict(joints).items()},
            "tool": {"x": round(tool.x_mm, 1), "y": round(tool.y_mm, 1), "z": round(tool.z_mm, 1)}, "favorite": favorite}


def program_names(folder: Path) -> list[str]:
    return sorted(p.stem for p in folder.glob("*.yaml")) if folder.is_dir() else []


def load_program(folder: Path, name: str) -> list:
    data = yaml.safe_load((folder / f"{name}.yaml").read_text(encoding="utf-8")) or {}
    return data.get("steps") or []


def save_program(folder: Path, name: str, steps: list) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    (folder / f"{name}.yaml").write_text(
        f"# 프로그램 {name} (결정 0044). GUI 프로그램 화면에서 만들었다. 단계 종류: {', '.join(DEFAULTS)}\n"
        + yaml.safe_dump({"steps": steps}, allow_unicode=True, sort_keys=False), encoding="utf-8")
