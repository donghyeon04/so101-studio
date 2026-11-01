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
        return self._workspace(q, ref)

    def _tip_height(self, q: JointState) -> float:
        """낮은 턱 끝의 작업대 면 위 높이(mm). 기준점(고정 턱 면)은 수직일 때 턱 끝보다 약 7mm 위라 기준점으로 재면 느슨하다."""
        p = self.kin.fk(q)
        f, mt = self.kin.jaw_tip_offsets(p.pitch_deg, p.yaw_deg, q.gripper)
        return p.z_mm + min(f[2], mt[2], 0.0) - self.table_z_mm

    def _workspace(self, q: JointState, ref: JointState | None = None) -> str | None:
        if self.ws is None or self.kin is None:
            return None
        p, box = self.kin.fk(q), self.box()
        if self.check_xy and self.settings.mode == "normal" and not (
            box["x"][0] <= p.x_mm <= box["x"][1] and box["y"][0] <= p.y_mm <= box["y"][1]
        ):
            return "workspace"
        floor = box["z"][0]
        if ref is not None:  # 바닥 한계 아래에서 출발했다면(정지가 바닥 바로 앞에서 걸린 경우 등) 더 낮아지지 않는 동작은 허용해 빠져나올 수 있게 한다
            floor = min(floor, self._tip_height(ref))
        h = self._tip_height(q)
        if h < floor:
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


class HealthMonitor:
    """과열·과부하·통신 끊김 감시. 온도 읽기는 가끔 튀므로(T2) 연속 temp_count번 넘을 때만 fault로 본다."""

    def __init__(self, temp_limit_c: float = 60, temp_count: int = 3, load_limit_pct: float = 90,
                 load_s: float = 3.0, comm_s: float = 0.5):
        self.temp_limit_c, self.temp_count = temp_limit_c, temp_count
        self.load_limit_pct, self.load_s, self.comm_s = load_limit_pct, load_s, comm_s
        self._hot: dict[str, int] = {}
        self._over_since: dict[str, float] = {}
        self._comm_down_since: float | None = None

    def update_health(self, health: dict, now: float) -> str | None:
        for m, t in health["temperature_c"].items():
            self._hot[m] = self._hot.get(m, 0) + 1 if t >= self.temp_limit_c else 0
            if self._hot[m] >= self.temp_count:
                return f"overheat:{m}"
        for m, load in health["load_pct"].items():
            if load < self.load_limit_pct:
                self._over_since.pop(m, None)
            elif now - self._over_since.setdefault(m, now) >= self.load_s:
                return f"overload:{m}"
        return None

    def comm_ok(self) -> None:
        self._comm_down_since = None

    def comm_failed(self, now: float) -> str | None:
        self._comm_down_since = self._comm_down_since if self._comm_down_since is not None else now
        return "comm_lost" if now - self._comm_down_since >= self.comm_s else None
