"""병렬 그리퍼의 접근 각도 후보 기하 (arm-software-spec 6.6.1, 결정 0069).

β: 손끝 축 기울기(0 = 수직, 양수 = 손끝이 밑동에서 멀어지는 쪽). φ: 닫힘 방향 c의 기하 각(90° = 턱 축이 방위 평면 안, 지금 ToolPose로 나타내는 자세).
γ: 닫힘 방향의 수평면 기울기. 벡터는 작업대 좌표(단위 벡터)."""
import math

import numpy as np

Z = np.array([0.0, 0.0, 1.0])


def fold90(deg: float) -> float:
    """각도를 -90~90°로 접는다(턱은 180° 대칭)."""
    return (deg + 90.0) % 180.0 - 90.0


def closing_tilt(psi_deg: float, beta_deg: float) -> tuple[float, float]:
    """ψ(면 법선과 닫힘 방향 수평 성분을 나란하게 하는 각)와 β -> (φ, γ)."""
    p, b = math.radians(psi_deg), math.radians(beta_deg)
    phi = math.atan2(math.sin(p), math.cos(p) * math.cos(b))
    return math.degrees(phi), math.degrees(math.asin(math.sin(phi) * math.sin(b)))


def axes(az_deg: float, beta_deg: float, phi_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """방위 az에서 (손끝 축 a, 닫힘 방향 c)."""
    t, b, f = (math.radians(v) for v in (az_deg, beta_deg, phi_deg))
    r = np.array([math.cos(t), math.sin(t), 0.0])
    e_t, e_u = np.cross(Z, r), math.cos(b) * r + math.sin(b) * Z
    return math.sin(b) * r - math.cos(b) * Z, math.cos(f) * e_t + math.sin(f) * e_u


def faces(angle_deg: float | None, width_mm: float, length_mm: float) -> list[tuple[str, float, float]]:
    """(면 쌍 이름, 무는 치수 D, 법선 n°). 각도가 None이면 0°로 보고 폭 쪽만. 치수를 모르는(0) 면 쌍은 뺀다."""
    a = angle_deg or 0.0
    out = [("width", width_mm, a + 90.0)]
    if angle_deg is not None:
        out.append(("length", length_mm, a))
    return [f for f in out if f[1] > 0]


def contact_gap(D: float, H: float, t_low: float, gamma_deg: float) -> float:
    """닿는 틈 = D·cosγ + h·sinγ (h = 낮은 턱 끝에서 물체 윗면까지)."""
    g = math.radians(abs(gamma_deg))
    return D * math.cos(g) + (H - t_low) * math.sin(g)
