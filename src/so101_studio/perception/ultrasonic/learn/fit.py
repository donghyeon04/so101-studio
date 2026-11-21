"""8.8 모델 후보 2: 규칙 결과 + 릿지 잔차 보정(numpy만). 위치는 규칙 중심에 (Δx, Δy)를 더하고, 방향은 정답 2θ의 cos·sin을 직접 맞춘다."""
import math

import numpy as np


class Ridge:
    def __init__(self, alpha: float = 10.0):
        self.alpha = alpha

    def fit(self, X, Y):
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        self.mu, self.sd = X.mean(0), X.std(0) + 1e-9
        Z = np.c_[np.ones(len(X)), (X - self.mu) / self.sd]
        reg = self.alpha * np.eye(Z.shape[1])
        reg[0, 0] = 0.0  # 절편은 줄이지 않는다
        self.w = np.linalg.solve(Z.T @ Z + reg, Z.T @ Y)
        return self

    def predict(self, X):
        Z = np.c_[np.ones(len(X)), (np.asarray(X, float) - self.mu) / self.sd]
        return Z @ self.w


def targets(rows) -> np.ndarray:
    """(정답 − 규칙 중심 Δx, Δy, 정답 cos 2θ, sin 2θ). 방향이 없는 정답(정사각형)은 방향 열을 0으로(방향 평가에서도 빠진다)."""
    def two(a):
        return (math.cos(math.radians(2 * a)), math.sin(math.radians(2 * a))) if a is not None else (0.0, 0.0)
    return np.array([[t[0] - r[0], t[1] - r[1], *two(t[2])] for r, t in ((row["rule"], row["truth"]) for row in rows)])


def fit(rows, alpha: float = 10.0) -> Ridge:
    return Ridge(alpha).fit([r["x"] for r in rows], targets(rows))


def predict(model: Ridge, rows) -> list[tuple[float, float, float]]:
    """(x, y, 각도°)."""
    out = []
    for r, p in zip(rows, model.predict([r["x"] for r in rows]), strict=True):
        out.append((r["rule"][0] + p[0], r["rule"][1] + p[1], math.degrees(math.atan2(p[3], p[2])) / 2 % 180))
    return out
