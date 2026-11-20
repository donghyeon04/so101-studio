"""레이더 받침 STL을 돌지 않는 받침(radar_base.stl)과 서보로 도는 머리(radar_head.stl)로 나눈다 (ultrasonic-spec 8.9 "레이더 머리", REQ-112).

실행: python tools/split_radar_station.py [--src assets/scene/radar_station.stl] [--out assets/scene] [--dry-run]
규칙(고정): 꼭짓점을 합친 뒤(merge_vertices(digits_vertex=2)) 연결 성분으로 나눈다(2026-11-20 실측 14성분).
머리 = 성분 12(센서 브래킷)·13(HC-SR04 둘) + bounds가 z >= 11mm이고 xy가 12·13의 xy 상자를 합친 안에 다 들어가는 성분(나사·스페이서). 받침 = 나머지.
성분 수가 14가 아니거나 12·13 bounds가 아래 값과 1mm 넘게 다르면 쓰지 않고 멈춘다(종료 코드 1). trimesh만 쓴다."""
import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh

N_PARTS = 14
HEAD = {12: ((-24.5, -38.5, 11.2), (24.5, 8.2, 35.2)), 13: (None, None)}  # 13은 y·z만 확인(아래 HEAD13)
HEAD13 = {"y": (-48.7, -31.8), "z": (13.2, 39.3)}
TOL_MM, HEAD_Z_MIN = 1.0, 11.0


def components(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    """꼭짓점 연결 성분(모서리로 이어진 꼭짓점 묶음)마다 하나. 면 인접(split)으로 나누면 볼록 껍질 면이 따로 놀아 2473조각이 된다."""
    m = mesh.copy()
    m.merge_vertices(merge_tex=True, merge_norm=True, digits_vertex=2)  # 면마다 법선이 달라 법선도 합쳐야 꼭짓점이 하나가 된다
    label = np.empty(len(m.vertices), int)
    for i, c in enumerate(trimesh.graph.connected_components(m.edges, nodes=np.arange(len(m.vertices)))):
        label[c] = i
    face_label = label[m.faces[:, 0]]
    return [m.submesh([np.flatnonzero(face_label == i)], append=True) for i in range(label.max() + 1)]


def check_reference(parts) -> str | None:
    """성분 수와 12·13 bounds가 규칙의 값과 맞는지. 다르면 이유."""
    if len(parts) != N_PARTS:
        return f"성분 수 {len(parts)} != {N_PARTS}"
    (lo, hi), b = HEAD[12], parts[12].bounds
    if np.abs(b - np.array([lo, hi])).max() > TOL_MM:
        return f"성분 12 bounds {np.round(b, 1).tolist()}가 {[lo, hi]}와 {TOL_MM}mm 넘게 다르다"
    b13 = parts[13].bounds
    for ax, i in (("y", 1), ("z", 2)):
        if abs(b13[0][i] - HEAD13[ax][0]) > TOL_MM or abs(b13[1][i] - HEAD13[ax][1]) > TOL_MM:
            return f"성분 13 {ax} {np.round(b13[:, i], 1).tolist()}가 {list(HEAD13[ax])}와 {TOL_MM}mm 넘게 다르다"
    return None


def classify(parts) -> list[bool]:
    """성분마다 머리면 True."""
    box = np.array([np.minimum(parts[12].bounds[0], parts[13].bounds[0]), np.maximum(parts[12].bounds[1], parts[13].bounds[1])])
    out = []
    for i, p in enumerate(parts):
        lo, hi = p.bounds
        inside = lo[0] >= box[0][0] - 1e-6 and lo[1] >= box[0][1] - 1e-6 and hi[0] <= box[1][0] + 1e-6 and hi[1] <= box[1][1] + 1e-6
        out.append(i in (12, 13) or (lo[2] >= HEAD_Z_MIN and inside))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", default="assets/scene/radar_station.stl")
    ap.add_argument("--out", default="assets/scene")
    ap.add_argument("--dry-run", action="store_true", help="표만 보이고 파일을 쓰지 않는다")
    a = ap.parse_args(argv)
    parts = components(trimesh.load(a.src, force="mesh", process=False))
    head = classify(parts) if len(parts) > 13 else [False] * len(parts)
    print(f"{'#':>2} {'꼭짓점':>6}  bounds min (x, y, z) / max (x, y, z)  판정")
    for i, (p, h) in enumerate(zip(parts, head, strict=True)):
        lo, hi = (", ".join(f"{v:6.1f}" for v in row) for row in p.bounds)
        print(f"{i:>2} {len(p.vertices):>6}  ({lo})  ({hi})  {'머리' if h else '받침'}")
    if why := check_reference(parts):
        print(f"멈춤: {why}. 원본 CAD에서 머리·받침 바디를 따로 내보낸다(ultrasonic-spec 8.9)", file=sys.stderr)
        return 1
    if a.dry_run:
        return 0
    out = Path(a.out)
    for name, keep in (("radar_head.stl", True), ("radar_base.stl", False)):
        trimesh.util.concatenate([p for p, h in zip(parts, head, strict=True) if h == keep]).export(out / name)
        print(f"썼다: {out / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
