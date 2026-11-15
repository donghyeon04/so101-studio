"""관절 공간 궤적 (arm-software-spec.md 6.3). 지금은 move_j(5차 다항식)와 정지용 감속만."""
import bisect
import itertools
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


class LookaheadShape:
    """호 길이 L의 꺾은선을 처음에 서서 출발해 끝에서 서도록 지나는 시간 곡선. 경로를 ds 간격으로 나눈 점마다 속도 상한을 두고
    (곧은 구간은 v_max, 꼭짓점을 감는 호(vertices: (호 길이 위치, 속도, 호 길이 t))에서는 그 호의 속도, 되돌아가는 꺾임은 0, 처음과 끝은 0)
    앞뒤로 훑어 접선 가속이 a_max 이하가 되는 속도 곡선을 만든다. 점 사이는 등가속이다. 가속이 바뀌는 순간의 저크는 제한하지 않는다."""

    def __init__(self, length: float, vertices: list[tuple[float, float, float]], v_max: float, a_max: float, ds: float = 1.0):
        edges = [e for p, _, t in vertices if t > 1e-9 for e in (p - t, p + t)]  # 호의 양 끝도 점으로 둔다: 곡률이 끝나는 곳에서만 가속을 다시 시작하려고
        # 서는 점(처음·끝·서서 도는 꺾임) 둘이 ds 이하로 붙어 있으면 그 사이에 격자 점이 없어 두 점 모두 속도 0이 되고 구간 시간이 2d/0으로 터진다(REQ-066: 1e12초).
        # 가운데 점을 하나 넣어 반은 가속, 반은 감속하게 한다. ds보다 먼 쌍은 격자 점이 이미 사이에 있어 그대로다.
        stops = sorted({0.0, float(length), *[float(p) for p, v, t in vertices if v <= 1e-9 or t <= 1e-9]})
        mids = [(a + b) / 2 for a, b in itertools.pairwise(stops) if b - a <= ds]
        s = sorted({*np.arange(0.0, length, ds).tolist(), length, *[p for p, _, _ in vertices], *edges, *mids})
        s = np.array(s)
        lim = np.full(len(s), float(v_max))
        lim[0] = lim[-1] = 0.0
        for pos, v, t in vertices:
            if v <= 1e-9 or t <= 1e-9:  # 호 없이 서서 도는 꺾임
                lim[int(np.argmin(np.abs(s - pos)))] = 0.0
            else:
                lim[(s >= pos - t - 1e-9) & (s <= pos + t + 1e-9)] = np.minimum(lim[(s >= pos - t - 1e-9) & (s <= pos + t + 1e-9)], v)
        v2 = lim**2
        for i in range(1, len(s)):  # 앞으로 훑기: 앞 점에서 a_max로 가속해 닿을 수 있는 속도까지
            v2[i] = min(v2[i], v2[i - 1] + 2 * a_max * (s[i] - s[i - 1]))
        for i in range(len(s) - 2, -1, -1):  # 뒤로 훑기: 뒤 점에서 멈출 수 있게 a_max로 감속할 수 있는 속도까지
            v2[i] = min(v2[i], v2[i + 1] + 2 * a_max * (s[i + 1] - s[i]))
        self._s, self._v = s, np.sqrt(v2)
        d, v0, v1 = np.diff(s), self._v[:-1], self._v[1:]
        self._acc = (v1**2 - v0**2) / (2 * d)  # 점 사이 등가속
        dt = 2 * d / np.maximum(v0 + v1, 1e-12)
        self._t = np.r_[0.0, np.cumsum(dt)]
        self.length = float(length)
        self.duration_s = float(self._t[-1]) if self._t[-1] > 0 else 0.2
        self.corner_speeds = [float(self._v[int(np.argmin(np.abs(s - (pos + t if v > 1e-9 and t > 1e-9 else pos))))]) for pos, v, t in vertices]  # 꺾임마다 줄인 뒤 통과 속도(시험·기록용)

    def frac(self, t: float) -> tuple[float, float]:  # 위치 비율, 속도 비율(1/s)
        t = min(max(t, 0.0), self.duration_s)
        i = min(max(int(np.searchsorted(self._t, t, side="right")) - 1, 0), len(self._acc) - 1)
        tau = t - self._t[i]
        v = max(self._v[i] + self._acc[i] * tau, 0.0)
        pos = self._s[i] + self._v[i] * tau + 0.5 * self._acc[i] * tau * tau
        return min(float(pos) / self.length, 1.0), float(v) / self.length


def junction_arcs(P: list[np.ndarray], a_max: float, tol_mm: float = 2.5) -> list[tuple[float, float, float]]:
    """꺾은선 P의 안쪽 꼭짓점마다 (속도 mm/s, 접선 길이 t mm, 호 반지름 R mm). 꼭짓점을 반지름 R의 호로 감아 지난다고 보고 구심 가속 v²/R = a_max에서 v = sqrt(a_max·R), 호 길이는 꼭짓점 앞뒤로 t = R·tan(φ/2).
    R은 모서리를 깎는 거리가 tol_mm 이하(R ≤ tol·cos(φ/2)/(1-cos(φ/2)))이고 t가 짧은 쪽 구간의 절반 이하(R ≤ l/(2·tan(φ/2)))가 되게 잡는다. φ = 꺾인 각.
    φ가 작으면(원처럼 완만) 곡률 반지름 그대로, 180°로 되돌아가면 R=0이라 선다(속도 0, 호 없음). 거의 곧은 꼭짓점은 (inf, 0, 0)."""
    out = []
    for k in range(1, len(P) - 1):
        u, w = P[k] - P[k - 1], P[k + 1] - P[k]
        lu, lw = float(np.linalg.norm(u)), float(np.linalg.norm(w))
        phi = math.acos(float(np.clip(u @ w / (lu * lw), -1.0, 1.0))) if lu > 1e-9 and lw > 1e-9 else 0.0
        if phi < 1e-6:
            out.append((float("inf"), 0.0, 0.0))
            continue
        if phi > math.pi - 1e-6:
            out.append((0.0, 0.0, 0.0))
            continue
        c = math.cos(phi / 2)
        r = min(tol_mm * c / (1 - c), min(lu, lw) / (2 * math.tan(phi / 2)))
        out.append((math.sqrt(a_max * r), r * math.tan(phi / 2), r))
    return out


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


def polyline_l(kin, poses, seed: JointState, v_mm_s: float, a_mm_s2: float = 500.0,
               step_mm: float = 5.0, profile: str = "quintic", j_mm_s3: float = 3000.0) -> LineTrajectory | None:
    """poses(ToolPose 목록)를 직선으로 이어 한 번에 지나는 궤적(꺾이는 점은 그대로 꺾인다. 꺾임을 호로 감아 서지 않고 지나려면 smooth_l). 점 하나라도 IK가 실패하거나 해가 튀면 전체를 거부(None)한다.
    경유점은 전체 길이를 step_mm 이하로 균일하게 나눈 곳(호 길이 기준)이라 속도가 고르다.
    shortcut: 꺾이는 점의 모서리를 최대 step_mm/2만큼 깎는다. 정확히 지나야 하는 점이 있으면 step_mm를 줄인다."""
    from dataclasses import replace

    P = [np.array([p.x_mm, p.y_mm, p.z_mm]) for p in poses]
    seg = [float(np.linalg.norm(b - a)) for a, b in itertools.pairwise(P)]
    length = sum(seg)
    n = max(1, math.ceil(length / step_mm))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    pts, q = [_arr(seed)], seed
    for i in range(1, n + 1):
        d = length * i / n
        k = min(int(np.searchsorted(cum, d, side="right")) - 1, len(seg) - 1)
        f = 0.0 if seg[k] < 1e-9 else (d - cum[k]) / seg[k]
        a, b = poses[k], poses[k + 1]
        x, y, z = P[k] + (P[k + 1] - P[k]) * f
        pose = replace(b, x_mm=x, y_mm=y, z_mm=z, yaw_deg=a.yaw_deg + (b.yaw_deg - a.yaw_deg) * f,
                       pitch_deg=a.pitch_deg + (b.pitch_deg - a.pitch_deg) * f)
        q = kin.ik(pose, q)
        if q is None or np.abs(_arr(q) - pts[-1]).max() > MAX_WAYPOINT_JUMP_DEG:
            return None
        pts.append(_arr(q))
    if profile == "ruckig" and length > 1e-6:
        shape = RuckigShape(length, v_mm_s, a_mm_s2, j_mm_s3)
    else:
        shape = QuinticShape(max(1.875 * length / v_mm_s, math.sqrt(5.7735 * length / a_mm_s2), 0.2))
    return LineTrajectory(np.array(pts), shape)


class SplineTrajectory:
    """경유점(호 길이 s 기준)을 관절 공간 3차 스플라인(C2)으로 잇고, 시간 곡선(shape)이 호 길이를 따라 간다.
    경유점 사이를 선형으로 이으면 방향이 경유점마다 한 틱에 꺾여 가속이 튀므로 매끈하게 잇는다. 되돌아가는 지점(cusp)에서는 스플라인을 나눠 출렁이지 않게 한다."""

    def __init__(self, s: np.ndarray, waypoints: np.ndarray, shape, cusps: list[int] | None = None):
        from scipy.interpolate import CubicSpline

        self.shape, self.duration_s, self.length = shape, shape.duration_s, float(s[-1])
        cut = [0, *sorted(c for c in (cusps or []) if 0 < c < len(s) - 1), len(s) - 1]
        self._pieces = [(float(s[a]), float(s[b]), CubicSpline(s[a:b + 1], waypoints[a:b + 1], bc_type="natural")) for a, b in itertools.pairwise(cut)]
        self._los = [p[0] for p in self._pieces]

    def _at(self, pos: float, deriv: int = 0) -> np.ndarray:
        i = max(bisect.bisect_right(self._los, pos) - 1, 0)
        return self._pieces[min(i, len(self._pieces) - 1)][2](pos, deriv)

    def sample(self, t: float) -> JointState:
        return _joint_state(np.asarray(self._at(self.shape.frac(t)[0] * self.length)))

    def velocity(self, t: float) -> np.ndarray:  # deg/s
        frac, rate = self.shape.frac(t)
        return np.asarray(self._at(frac * self.length, 1)) * rate * self.length


def fillet_path(P: list[np.ndarray], a_eff: float, step_mm: float = 2.0, tol_mm: float = 2.5):
    """꺾은선 P의 안쪽 꼭짓점을 호(junction_arcs의 R, t)로 감아 step_mm 이하 간격으로 나눈 경로.
    반환: (점 N×3, 호 길이 s, 호 구간 [(가운데 s, 속도, 호 길이/2)], 원래 꼭짓점마다 s, 되돌아가는 점의 색인)."""
    n = len(P)
    unit = [(b - a) / max(float(np.linalg.norm(b - a)), 1e-12) for a, b in itertools.pairwise(P)]
    arcs_in = junction_arcs(P, a_eff, tol_mm)
    pts, arcs, vert_idx, cusps = [P[0].copy()], [], [0], []
    for i in range(n - 1):
        t0 = arcs_in[i - 1][1] if i >= 1 else 0.0  # 이 구간 시작 쪽 호의 접선 길이
        t1 = arcs_in[i][1] if i < n - 2 else 0.0
        a, b = P[i] + unit[i] * t0, P[i + 1] - unit[i] * t1
        length = float(np.linalg.norm(b - a))
        for k in range(1, math.ceil(length / step_mm) + 1) if length > 1e-9 else []:
            pts.append(a + (b - a) * k / math.ceil(length / step_mm))
        if i < n - 2:
            v, t, r = arcs_in[i]
            phi = math.acos(float(np.clip(unit[i] @ unit[i + 1], -1.0, 1.0)))
            if t > 1e-9 and r > 1e-9:
                nrm = unit[i + 1] - (unit[i + 1] @ unit[i]) * unit[i]
                nrm /= np.linalg.norm(nrm)
                center = b + nrm * r  # b = 꼭짓점 - u·t
                m = max(math.ceil(r * phi / step_mm), math.ceil(phi / math.radians(8.0)), 2)  # 호는 8° 이하 간격으로(작은 반지름 호가 두세 점으로 거칠어지지 않게)
                s_start = len(pts) - 1  # 호는 직전 점(곧은 구간의 끝)에서 시작한다
                for k in range(1, m + 1):
                    th = phi * k / m
                    pts.append(center + r * (-nrm * math.cos(th) + unit[i] * math.sin(th)))
                arcs.append((s_start, v, r * phi / 2))
                vert_idx.append(s_start + m // 2)
            elif math.isinf(v):  # 정확히 일직선인 꼭짓점(junction_arcs의 inf): 서지 않고 지나간다. 경로 점은 그대로, 속도 상한만 두지 않는다
                vert_idx.append(len(pts) - 1)
            else:  # 되돌아가는 꺾임: 그 점에서 선다
                if not np.allclose(pts[-1], P[i + 1]):
                    pts.append(P[i + 1].copy())
                arcs.append((len(pts) - 1, 0.0, 0.0))
                vert_idx.append(len(pts) - 1)
                if phi > math.pi - 1e-6:
                    cusps.append(len(pts) - 1)
    vert_idx.append(len(pts) - 1)
    pts = np.array(pts)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    arcs = [(float(s[a]) + (h if h > 0 else 0.0), v, h) for a, v, h in arcs]  # 호 시작 색인 -> 가운데 s
    return pts, s, arcs, s[vert_idx], cusps


def smooth_l(kin, poses, seed: JointState, v_mm_s: float, a_mm_s2: float, step_mm: float = 1.0) -> SplineTrajectory | None:
    """poses(ToolPose 목록)를 한 번에 서지 않고 지나는 궤적: 꼭짓점을 호로 감은 경로 위 경유점(step_mm 이하 간격)을 IK로 풀어 관절 공간 스플라인으로 잇고,
    속도는 처음과 끝에서만 서고 곡선에서는 구심 가속이 한도 안이 되게 줄인다(속도 v ≤ sqrt(a·R); 접선·구심 가속이 겹쳐도 합이 a_mm_s2를 넘지 않게 둘 다 a/√2).
    점 하나라도 IK가 실패하거나 해가 튀면 None. 180° 되돌아가는 꼭짓점은 서서 돈다. 모서리는 2.5mm 안으로 깎는다."""
    from dataclasses import replace

    a_eff = a_mm_s2 / math.sqrt(2)
    P = [np.array([p.x_mm, p.y_mm, p.z_mm]) for p in poses]
    pts, s, arcs, vert_s, cusps = fillet_path(P, a_eff, step_mm)
    yaw = np.interp(s, vert_s, [p.yaw_deg for p in poses]) if len(poses) > 1 else [poses[0].yaw_deg] * len(s)
    pitch = np.interp(s, vert_s, [p.pitch_deg for p in poses])
    wp, q = [_arr(seed)], seed
    for xyz, yw, pt in zip(pts[1:], yaw[1:], pitch[1:]):
        q = kin.ik(replace(poses[-1], x_mm=float(xyz[0]), y_mm=float(xyz[1]), z_mm=float(xyz[2]), yaw_deg=float(yw), pitch_deg=float(pt)), q, max_iter=100)  # 손목을 접은 자세(radar_clear)에서 출발하면 30번으로는 수렴하지 않는 점이 있다. 수렴하면 일찍 끝나서 느려지지 않는다
        if q is None or np.abs(_arr(q) - wp[-1]).max() > MAX_WAYPOINT_JUMP_DEG * step_mm / 5.0:
            return None
        wp.append(_arr(q))
    shape = LookaheadShape(float(s[-1]), arcs, v_mm_s, a_eff)
    return SplineTrajectory(s, np.array(wp), shape, cusps)


def move_l(kin, start, goal, seed: JointState, v_mm_s: float, a_mm_s2: float = 500.0,
           step_mm: float = 5.0, profile: str = "quintic", j_mm_s3: float = 3000.0) -> LineTrajectory | None:
    """start→goal(ToolPose)를 직선으로 잇는다. 점 하나라도 IK가 실패하거나 해가 튀면 전체를 거부(None)한다."""
    return polyline_l(kin, [start, goal], seed, v_mm_s, a_mm_s2, step_mm, profile, j_mm_s3)


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
