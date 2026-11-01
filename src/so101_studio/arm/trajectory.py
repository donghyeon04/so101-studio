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


def move_j(q_start: JointState, q_goal: JointState, v_max_deg_s: float, a_max_deg_s2: float) -> Trajectory:
    """가장 많이 움직이는 관절이 속도·가속도 한계를 넘지 않도록 시간을 정한다 (5차: 최대속도 1.875D/T, 최대가속도 5.77D/T²)."""
    a, b = _arr(q_start), _arr(q_goal)
    d = float(np.abs(b - a).max())
    T = max(1.875 * d / v_max_deg_s, math.sqrt(5.7735 * d / a_max_deg_s2), 0.2)
    return Trajectory(a, b, T)
