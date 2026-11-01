"""중력 보상 모델. 서보는 전류·토크 명령이 없으므로 "팔 무게를 버티는 데 필요한 위치 오차"를 목표에 미리 더해 준다.

URDF 질량으로 관절별 중력 토크(N·m)를 계산하고, 정지 상태에서 잰 부하(Present_Load, ±1000)와 맞춰 관절별 계수를 구한다:
    부하 = k x 중력토크 + b          (gravcal이 여러 자세에서 재서 최소제곱으로 구함)
    미리 더할 각도(°) = 부하 / s      (s = 위치 오차 1°당 서보가 내는 부하, 교시용 P 계수에서 잰 값)
계수는 config/gravity.json에 저장한다."""
import json
import math
from pathlib import Path

import numpy as np

from .kinematics import ARM_JOINTS

GRAVITY_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")  # 중력을 받는 관절. 어깨 회전·손목 회전·그리퍼는 무시한다
MAX_FF_DEG = 12.0  # 미리 더하는 각도의 상한(모델이 틀려도 팔이 크게 튀지 않게)
DEFAULT_PATH = "config/gravity.json"


class Gravity:
    def __init__(self, urdf_path: str, params: dict | None = None):
        import placo

        self._robot = placo.RobotWrapper(urdf_path)  # 기구학 풀이기와 따로 둔다(스레드에서 상태를 같이 쓰지 않게)
        self.params = params or {}

    def torques(self, q) -> np.ndarray:
        """5개 팔 관절의 중력 토크(N·m). q는 JointState 또는 관절각(°) 열."""
        vals = [getattr(q, j) for j in ARM_JOINTS] if hasattr(q, "shoulder_pan") else list(q)[:5]
        for j, v in zip(ARM_JOINTS, vals, strict=True):
            self._robot.set_joint(j, math.radians(v))
        self._robot.update_kinematics()
        return np.array(self._robot.generalized_gravity())[6:11]

    def load_at(self, q) -> dict[str, float]:
        """정지 자세에서 서보가 중력을 버티느라 내는 부하(±1000 단위)의 예측."""
        tau = self.torques(q)
        return {j: self.params[j]["k"] * tau[ARM_JOINTS.index(j)] + self.params[j]["b"] for j in GRAVITY_JOINTS if j in self.params}

    def ff_deg(self, q, gain: float = 1.0) -> np.ndarray:
        """6개 관절 각각에 목표로 미리 더할 각도(°). 중력 관절만 값이 있다."""
        out = np.zeros(6)
        for j, load in self.load_at(q).items():
            out[ARM_JOINTS.index(j)] = float(np.clip(gain * load / self.params[j]["s"], -MAX_FF_DEG, MAX_FF_DEG))
        return out


def fit(tau: np.ndarray, load: np.ndarray) -> dict:
    """한 관절의 (중력 토크, 정지 부하) 표본에서 부하 = k x 토크 + b 를 구한다. r2는 설명력(1에 가까울수록 좋음)."""
    A = np.column_stack([tau, np.ones(len(tau))])
    (k, b), *_ = np.linalg.lstsq(A, load, rcond=None)
    resid = load - (k * tau + b)
    ss_tot = float(np.sum((load - load.mean()) ** 2))
    r2 = 0.0 if ss_tot < 1e-6 else float(1 - np.sum(resid**2) / ss_tot)  # 부하가 자세에 따라 변하지 않으면 설명할 것이 없다
    return {"k": float(k), "b": float(b), "r2": r2, "resid_std": float(resid.std())}


def save(params: dict, path: str | Path | None = None) -> None:
    Path(path or DEFAULT_PATH).write_text(json.dumps(params, indent=1, ensure_ascii=False), encoding="utf-8")


def load_params(path: str | Path | None = None) -> dict | None:
    p = Path(path or DEFAULT_PATH)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None
