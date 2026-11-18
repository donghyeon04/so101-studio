"""안전 검사 (arm-software-spec.md 6.5). 명령을 보내기 전에 검사하고, 하나라도 어기면 보내지 않는다."""
from contextlib import contextmanager
from dataclasses import asdict, dataclass

from lerobot.robots.utils import ensure_safe_goal_position

from ..core.config import WorkspaceConfig
from ..core.types import JointState

MODES = ("normal", "free", "off")


@dataclass
class SafetySettings:
    """사용자가 바꿀 수 있는 안전 설정. GUI가 그대로 JSON으로 주고받는다 (ArmService.get_safety / set_safety).
    mode: normal = 관절 한계 + 작업 영역 + 바닥 / free = 작업대 가장자리(x, y)만 끔 / off = 관절 한계·작업 영역·바닥 모두 끔(한 주기 이동량 한계와 과열·과부하 감시는 항상 켜짐).
    joint_margin_deg: 양수면 팔 관절 한계를 안쪽으로 그만큼 줄이고, 음수면 실제 가동 범위(physical_limits_deg)까지 넓힌다.
    box_mm: 작업대 좌표계 x, y 범위와 z(작업대 면 위 낮은 턱 끝 높이) 범위. None이면 작업대 크기에서 여유를 뺀 범위와 기본 바닥 높이."""

    mode: str = "normal"
    joint_margin_deg: float = 0.0
    box_mm: dict | None = None


class Safety:
    def __init__(self, joint_limits: dict[str, list[float]], ws: WorkspaceConfig | None = None, kin=None,
                 max_step_deg: float = 5.0, z_floor_mm: float = 4.0, table_z_mm: float = 0.0,
                 physical_limits: dict[str, list[float]] | None = None):
        self._base_limits, self.ws, self.kin = joint_limits, ws, kin
        self.physical = physical_limits or {}
        self.settings = SafetySettings()
        self.max_step_deg, self.z_floor_mm, self.table_z_mm = max_step_deg, z_floor_mm, table_z_mm
        self.collision = None  # SceneCollision (장면 부품과 팔의 거리). ArmService가 연결한다 (REQ-040)
        self.check_xy = True  # False면 작업대 가장자리 검사를 건너뛰고 바닥(턱 끝 높이)만 본다. 전원·부하 시험이 팔을 끝까지 펼 때만 임시로 끈다

    @property
    def limits(self) -> dict[str, list[float]]:
        """지금 적용되는 관절 한계(설정 파일 값에 joint_margin_deg를 반영). off 모드에서는 비어 있다."""
        if self.settings.mode == "off":
            return {}
        m = self.settings.joint_margin_deg
        out = {}
        for name, (lo, hi) in self._base_limits.items():
            if m and name != "gripper":
                if m > 0:
                    lo, hi = (lo + m, hi - m) if lo + m < hi - m else ((lo + hi) / 2, (lo + hi) / 2)
                elif name in self.physical:
                    lo, hi = max(self.physical[name][0], lo + m), min(self.physical[name][1], hi - m)
            out[name] = [lo, hi]
        return out

    def box(self) -> dict | None:
        """x, y, z 범위. 사용자가 정한 box_mm가 없으면 작업대 크기에서 여유를 뺀 x, y만 있고 z는 기본 바닥 높이부터."""
        if self.ws is None:
            return None
        m, t = self.ws.safety_margin_mm, self.ws.table_mm
        base = {"x": [m, t["x"] - m], "y": [m, t["y"] - m], "z": [self.z_floor_mm, None]}
        return {**base, **(self.settings.box_mm or {})}

    @contextmanager
    def relaxed(self, check_xy: bool = True, max_step_deg: float | None = None):
        """시험용으로 검사를 임시로 풀고, 어떤 이유로 끝나든 원래 값으로 되돌린다(오래 도는 GUI 프로세스에서 풀린 채로 남지 않게)."""
        prev = self.check_xy, self.max_step_deg
        self.check_xy = check_xy
        self.max_step_deg = prev[1] if max_step_deg is None else max_step_deg
        try:
            yield
        finally:
            self.check_xy, self.max_step_deg = prev

    def check_goal(self, q: JointState, ref: JointState | None = None) -> str | None:
        """ref(지금 위치)를 주면 한계 밖에서 출발한 관절은 한계 안쪽으로 돌아오는 방향의 움직임을 허용한다(조그용)."""
        if self.settings.mode == "off":
            return None
        for name, (lo, hi) in self.limits.items():
            if ref is not None:
                lo, hi = min(lo, getattr(ref, name)), max(hi, getattr(ref, name))
            if not lo <= getattr(q, name) <= hi:
                return f"joint_limit:{name}"
        return self._workspace(q, ref) or self._scene(q, ref)

    def _scene(self, q: JointState, ref: JointState | None = None) -> str | None:
        """장면 부품과 팔 링크 사이 거리가 margin_mm보다 가까우면 'scene:<이름>', 링크에 붙은 부품이 바닥(z_floor_mm)보다 낮으면 'scene_floor:<이름>'.
        이미 안에서 출발했다면 거리가 줄지 않고 더 낮아지지 않는 움직임은 허용해 빠져나올 수 있게 한다(_workspace와 같다)."""
        c = self.collision
        if c is None or c.scene is None:
            return None
        d, name, low_name, low_z = c.distance(q)
        r = c.distance(ref) if ref is not None else None
        if low_z < c.z_floor_mm and not (r is not None and r[3] < c.z_floor_mm and low_z >= r[3] - 1e-6):
            return f"scene_floor:{low_name}"
        if d >= c.scene.margin_mm or (r is not None and d >= r[0] - 1e-6):
            return None
        return f"scene:{name}"

    def _tip_height(self, q: JointState) -> float:
        """낮은 턱 끝의 작업대 면 위 높이(mm). 기준점(고정 턱 면)은 수직일 때 턱 끝보다 약 7mm 위라 기준점으로 재면 느슨하다."""
        p = self.kin.fk(q)
        f, mt = self.kin.jaw_tips_at(q)  # 병렬의 기운 자세(wrist_roll을 돌림)에서도 맞는 턱 끝 (6.6.1)
        h = min(float(f[2]), float(mt[2]), p.z_mm) - self.table_z_mm
        low = getattr(self.kin, "jaw_lowest_z", None)  # 기울기·벌림을 반영한 두 턱 메쉬의 가장 낮은 꼭짓점까지 본다 (REQ-036)
        return h if low is None else min(h, low(q) - self.table_z_mm)

    @staticmethod
    def _outside(v: float, lim) -> float:
        """범위 [lo, hi] 밖으로 벗어난 거리(안이면 0). 한쪽이 None이면 그쪽은 제한 없음."""
        lo, hi = lim
        return max(0.0 if lo is None else lo - v, 0.0 if hi is None else v - hi, 0.0)

    def _workspace(self, q: JointState, ref: JointState | None = None) -> str | None:
        if self.ws is None or self.kin is None:
            return None
        p, box = self.kin.fk(q), self.box()
        if self.check_xy and self.settings.mode == "normal":
            pr = self.kin.fk(ref) if ref is not None else None
            for ax, v, v0 in (("x", p.x_mm, pr and pr.x_mm), ("y", p.y_mm, pr and pr.y_mm)):
                # 작업대 밖에서 출발했다면(0° 자세 등) 벗어난 거리가 출발점보다 늘지 않는 점만 허용해 돌아올 수 있게 한다 (decisions/0031)
                if self._outside(v, box[ax]) > (self._outside(v0, box[ax]) if v0 is not None else 0.0) + 1e-6:
                    return "workspace"
        floor = box["z"][0]  # None이면 바닥 검사 없음(사용자가 명시)
        if floor is not None and ref is not None:  # 바닥 한계 아래에서 출발했다면(정지가 바닥 바로 앞에서 걸린 경우 등) 더 낮아지지 않는 동작은 허용해 빠져나올 수 있게 한다
            floor = min(floor, self._tip_height(ref))
        h = self._tip_height(q)
        if floor is not None and h < floor:
            return "z_floor"
        if box["z"][1] is not None and h > box["z"][1]:
            return "z_ceiling"
        return None

    def check_path(self, traj, n: int = 20) -> str | None:
        """목표와 경로 위 점들을 관절 한계와 작업 영역(바닥 포함)으로 검사한다. 시작점은 검사하지 않고(받침에 기댄 자세가 한계 밖일 수 있다),
        한계 밖에서 출발했다면 그 안쪽(시작점까지)으로 돌아오거나 머무는 점은 허용한다. 0.1초마다 이상 촘촘히 본다(긴 녹화에서 짧은 위반을 놓치지 않게)."""
        ref = traj.sample(0.0)
        n = max(n, int(traj.duration_s * 10))
        reason = self.check_goal(traj.sample(traj.duration_s), ref)
        for i in range(1, n):
            reason = reason or self.check_goal(traj.sample(traj.duration_s * i / n), ref)
        return reason

    def check_step(self, prev: JointState, nxt: JointState) -> str | None:
        """한 주기 이동량 상한. LeRobot의 ensure_safe_goal_position(max_relative_target과 같은 함수)으로 잘라 보고, 잘렸으면 거부한다."""
        goal, present = asdict(nxt), asdict(prev)
        safe = ensure_safe_goal_position({j: (goal[j], present[j]) for j in goal}, float(self.max_step_deg))
        return "step" if any(abs(safe[j] - goal[j]) > 1e-6 for j in goal) else None

    def step_detail(self, prev: JointState, nxt: JointState) -> dict:
        """check_step이 거부한 한 주기에서 가장 많이 움직이려 한 관절과 이동량·한계(fault 상세용)."""
        goal, present = asdict(nxt), asdict(prev)
        joint = max(goal, key=lambda j: abs(goal[j] - present[j]))
        return {"joint": joint, "delta_deg": round(abs(goal[joint] - present[joint]), 2), "limit_deg": float(self.max_step_deg)}


class HealthMonitor:
    """과열·과부하·통신 끊김 감시. 온도 읽기는 가끔 튀므로(T2) 연속 temp_count번 넘을 때만 fault로 본다."""

    def __init__(self, temp_limit_c: float = 60, temp_count: int = 3, load_limit_pct: float = 90,
                 load_s: float = 3.0, comm_s: float = 0.5, current_limit_ma: float = 4000, current_count: int = 2):
        self.temp_limit_c, self.temp_count = temp_limit_c, temp_count
        self.load_limit_pct, self.load_s, self.comm_s = load_limit_pct, load_s, comm_s
        self.current_limit_ma, self.current_count = current_limit_ma, current_count  # 어댑터(5A)의 과전류 보호가 걸려 6개가 한꺼번에 꺼지는 것을 막는다
        self._over_current = 0
        self._hot: dict[str, int] = {}
        self._over_since: dict[str, float] = {}
        self._down: dict[str, float | None] = {"read": None, "write": None}  # 끊김 시계는 읽기와 쓰기를 따로 센다: 쓰기가 성공해도 읽기 실패 시계를 지우지 못한다
        self._last_ok: dict[str, float | None] = {"read": None, "write": None}

    @property
    def _comm_down_since(self) -> float | None:
        """가장 먼저 시작된 끊김 시각(없으면 None)."""
        return min((v for v in self._down.values() if v is not None), default=None)

    def update_health(self, health: dict, now: float) -> str | None:
        for m, t in health["temperature_c"].items():
            self._hot[m] = self._hot.get(m, 0) + 1 if t >= self.temp_limit_c else 0
            if self._hot[m] >= self.temp_count:
                return f"overheat:{m}"
        self._over_current = self._over_current + 1 if health.get("current_ma", 0.0) >= self.current_limit_ma else 0
        if self._over_current >= self.current_count:
            return "overcurrent"
        for m, load in health["load_pct"].items():
            if load < self.load_limit_pct:
                self._over_since.pop(m, None)
            elif now - self._over_since.setdefault(m, now) >= self.load_s:
                return f"overload:{m}"
        return None

    def comm_ok(self, now: float | None = None, kind: str | None = None) -> None:
        """통신이 됐다. kind가 "read"나 "write"면 그 시계만 지우고, 없으면 둘 다 지운다. 이동 중 제어 틱의 쓰기 성공은 "write"만 지운다:
        서보가 쓰기는 받는데 읽기가 안 되는 상태를 놓치지 않으려는 것이다 (REQ-032)."""
        for k in ((kind,) if kind else ("read", "write")):
            self._down[k] = None
            self._last_ok[k] = now

    def comm_failed(self, now: float, kind: str = "read") -> str | None:
        """끊김 시간은 마지막으로 통신이 된 시각부터 잰다(처음 실패를 본 시각이 아니라): 읽기가 드문 정지 중에도 끊긴 뒤 0.5초쯤에 fault가 난다.
        읽기와 쓰기 중 어느 쪽이든 comm_s 넘게 이어지면 comm_lost."""
        if self._down[kind] is None:
            lo = self._last_ok[kind]
            self._down[kind] = lo if lo is not None and 0 <= now - lo < self.comm_s * 2 else now
        return self.comm_lost(now)

    def comm_lost(self, now: float) -> str | None:
        """새 실패를 알리지 않고, 이미 시작된 끊김이 comm_s를 넘었는지만 본다."""
        return "comm_lost" if any(now - v >= self.comm_s for v in self._down.values() if v is not None) else None
