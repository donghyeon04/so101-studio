"""관절 공간 궤적 (arm-software-spec.md 6.3). 지금은 move_j(5차 다항식)와 정지용 감속만."""
import math
from dataclasses import astuple, dataclass

import numpy as np

from ..core.types import JointState


def _arr(q: JointState) -> np.ndarray:
    return np.array(astuple(q), float)


def _joint_state(a: np.ndarray) -> JointState:
    return JointState(*a.tolist())


@dataclass
class Trajectory:
    q0: np.ndarray
    q1: np.ndarray
    duration_s: float

    def _s(self, t: float) -> tuple[float, float]:  # 위치 비율, 속도 비율(1/s)
        u = min(max(t / self.duration_s, 0.0), 1.0)
        return 10 * u**3 - 15 * u**4 + 6 * u**5, (30 * u**2 - 60 * u**3 + 30 * u**4) / self.duration_s

    def sample(self, t: float) -> JointState:
        return _joint_state(self.q0 + (self.q1 - self.q0) * self._s(t)[0])

    def velocity(self, t: float) -> np.ndarray:  # deg/s
        return (self.q1 - self.q0) * self._s(t)[1]


@dataclass
class StopTrajectory:
    """현재 속도에서 일정한 감속으로 멈춘다."""
    q0: np.ndarray
    v0: np.ndarray
    duration_s: float = 0.2

    def sample(self, t: float) -> JointState:
        t = min(max(t, 0.0), self.duration_s)
        return _joint_state(self.q0 + self.v0 * t - self.v0 * t * t / (2 * self.duration_s))

    def velocity(self, t: float) -> np.ndarray:
        return self.v0 * (1 - min(max(t, 0.0), self.duration_s) / self.duration_s)


class QuinticShape:
    """0→1 구간을 5차 다항식으로 지나는 시간 곡선."""

    def __init__(self, duration_s: float):
        self.duration_s = duration_s

    def frac(self, t: float) -> tuple[float, float]:  # 위치 비율, 속도 비율(1/s)
        u = min(max(t / self.duration_s, 0.0), 1.0)
        return 10 * u**3 - 15 * u**4 + 6 * u**5, (30 * u**2 - 60 * u**3 + 30 * u**4) / self.duration_s


class RuckigShape:
    """한 경로 길이 L을 Ruckig(속도·가속도·저크 한계)로 지나는 1축 시간 곡선."""

    def __init__(self, length: float, v_max: float, a_max: float, j_max: float):
        from ruckig import InputParameter, Ruckig, Trajectory

        inp = InputParameter(1)
        inp.current_position, inp.target_position = [0.0], [length]
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = [v_max], [a_max], [j_max]
        self._traj, self.length = Trajectory(1), length
        if Ruckig(1).calculate(inp, self._traj).value < 0:
            raise ValueError("Ruckig 궤적 계산 실패")
        self.duration_s = self._traj.duration

    def frac(self, t: float) -> tuple[float, float]:
        pos, vel, _ = self._traj.at_time(min(max(t, 0.0), self.duration_s))
        return pos[0] / self.length, vel[0] / self.length


class RuckigTrajectory:
    """6관절을 Ruckig로 동시에 도착시키는 관절 궤적(모든 관절이 같은 시각에 끝난다)."""

    def __init__(self, q0: np.ndarray, q1: np.ndarray, v_max: float, a_max: float, j_max: float,
                 v0: np.ndarray | None = None):
        from ruckig import InputParameter, Ruckig, Trajectory

        n = len(q0)
        inp = InputParameter(n)
        inp.current_position, inp.target_position = q0.tolist(), q1.tolist()
        if v0 is not None:  # 움직이는 중에 목표를 바꿀 때: 지금 속도에서 이어서 간다 (T11 retarget)
            inp.current_velocity = np.clip(v0, -v_max, v_max).tolist()
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = [v_max] * n, [a_max] * n, [j_max] * n
        self.q0, self.q1, self._traj = q0, q1, Trajectory(n)
        if Ruckig(n).calculate(inp, self._traj).value < 0:
            raise ValueError("Ruckig 궤적 계산 실패")
        self.duration_s = self._traj.duration

    def sample(self, t: float) -> JointState:
        return _joint_state(np.array(self._traj.at_time(min(max(t, 0.0), self.duration_s))[0]))

    def velocity(self, t: float) -> np.ndarray:
        return np.array(self._traj.at_time(min(max(t, 0.0), self.duration_s))[1])


def move_j(q_start: JointState, q_goal: JointState, v_max_deg_s: float, a_max_deg_s2: float,
           profile: str = "quintic", j_max_deg_s3: float = 300.0):
    """관절 공간 이동. quintic: 가장 많이 움직이는 관절이 속도·가속도 한계를 넘지 않도록 시간을 정한다
    (5차: 최대속도 1.875D/T, 최대가속도 5.77D/T²). ruckig: 속도·가속도·저크 한계 안에서 시간 최적."""
    a, b = _arr(q_start), _arr(q_goal)
    if profile == "ruckig":
        return RuckigTrajectory(a, b, v_max_deg_s, a_max_deg_s2, j_max_deg_s3)
    d = float(np.abs(b - a).max())
    T = max(1.875 * d / v_max_deg_s, math.sqrt(5.7735 * d / a_max_deg_s2), 0.2)
    return Trajectory(a, b, T)


MAX_WAYPOINT_JUMP_DEG = 15.0  # 5mm 간격 IK 해 사이에서 관절이 이만큼 넘게 바뀌면 해가 뒤집힌 것으로 본다


class LineTrajectory:
    """직선 경로를 따라 미리 푼 관절 경유점들 사이를 보간한다. 시간 곡선(shape)은 5차 다항식 또는 Ruckig."""

    def __init__(self, waypoints: np.ndarray, shape):
        self.pts, self.shape, self.duration_s = waypoints, shape, shape.duration_s

    def reversed(self, gripper: float | None = None) -> "LineTrajectory":
        """같은 경로를 거꾸로 간다. 내려간 길로 되돌아오는 것이므로 IK를 다시 풀 필요가 없다(그리퍼 값은 지금 상태로 바꿀 수 있다)."""
        pts = self.pts[::-1].copy()
        if gripper is not None:
            pts[:, 5] = gripper
        return LineTrajectory(pts, self.shape)

    def _at(self, s: float) -> np.ndarray:
        x = s * (len(self.pts) - 1)
        i = min(int(x), len(self.pts) - 2)
        return self.pts[i] + (self.pts[i + 1] - self.pts[i]) * (x - i)

    def sample(self, t: float) -> JointState:
        return _joint_state(self._at(self.shape.frac(t)[0]))

    def velocity(self, t: float) -> np.ndarray:
        s, ds = self.shape.frac(t)
        x = min(s * (len(self.pts) - 1), len(self.pts) - 1 - 1e-9)
        i = int(x)
        return (self.pts[i + 1] - self.pts[i]) * (len(self.pts) - 1) * ds


def move_l(kin, start, goal, seed: JointState, v_mm_s: float, a_mm_s2: float = 500.0,
           step_mm: float = 5.0, profile: str = "quintic", j_mm_s3: float = 3000.0) -> LineTrajectory | None:
    """start→goal(ToolPose)를 직선으로 잇는다. 점 하나라도 IK가 실패하거나 해가 튀면 전체를 거부(None)한다."""
    from dataclasses import replace

    p0, p1 = np.array([start.x_mm, start.y_mm, start.z_mm]), np.array([goal.x_mm, goal.y_mm, goal.z_mm])
    length = float(np.linalg.norm(p1 - p0))
    n = max(1, math.ceil(length / step_mm))
    pts, q = [_arr(seed)], seed
    for i in range(1, n + 1):
        f = i / n
        x, y, z = p0 + (p1 - p0) * f
        pose = replace(goal, x_mm=x, y_mm=y, z_mm=z, yaw_deg=start.yaw_deg + (goal.yaw_deg - start.yaw_deg) * f,
                       pitch_deg=start.pitch_deg + (goal.pitch_deg - start.pitch_deg) * f)
        q = kin.ik(pose, q)
        if q is None or np.abs(_arr(q) - pts[-1]).max() > MAX_WAYPOINT_JUMP_DEG:
            return None
        pts.append(_arr(q))
    if profile == "ruckig" and length > 1e-6:
        shape = RuckigShape(length, v_mm_s, a_mm_s2, j_mm_s3)
    else:
        shape = QuinticShape(max(1.875 * length / v_mm_s, math.sqrt(5.7735 * length / a_mm_s2), 0.2))
    return LineTrajectory(np.array(pts), shape)


def cfg_move_j(cfg, q0: JointState, q1: JointState, speed_pct: float):
    """설정(robot.yaml)의 속도·가속도·저크·프로파일로 만든 관절 이동."""
    return move_j(q0, q1, cfg.joint_v_max_deg_s * speed_pct / 100, cfg.a_max_deg_s2,
                  cfg.trajectory_profile, cfg.joint_jerk_max_deg_s3)


def cfg_move_l(cfg, kin, start, goal, seed: JointState, speed_pct: float):
    return move_l(kin, start, goal, seed, cfg.tool_v_max_mm_s * speed_pct / 100, cfg.tool_a_max_mm_s2,
                  profile=cfg.trajectory_profile, j_mm_s3=cfg.tool_jerk_max_mm_s3)


class SequenceTrajectory:
    """시간에 따라 기록된 관절 자세 목록(녹화·시연 동작)을 보간해 재생한다."""

    def __init__(self, times: np.ndarray, poses: np.ndarray):
        self.t, self.q, self.duration_s = np.asarray(times, float), np.asarray(poses, float), float(times[-1])

    def sample(self, t: float) -> JointState:
        t = min(max(t, 0.0), self.duration_s)
        return _joint_state(np.array([np.interp(t, self.t, self.q[:, j]) for j in range(self.q.shape[1])]))

    def velocity(self, t: float) -> np.ndarray:
        dt = 0.05
        return (np.array(astuple(self.sample(t + dt))) - np.array(astuple(self.sample(t - dt)))) / (min(t + dt, self.duration_s) - max(t - dt, 0.0) or 1.0)
