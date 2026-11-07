"""장면 부품 목록 (REQ-040, 결정 0043-4): 팔 밖의 물체(레이더 스테이션, 케이블 지지대 등)를 STL·상자로 적어 둔다.
GUI가 3D로 그리고, 안전 검사가 충돌 거리를 잰다. 단위 mm."""
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
import yaml


@dataclass
class ScenePart:
    name: str
    attach: str  # "table" 또는 URDF 링크 이름
    T: np.ndarray  # 4x4, 붙는 곳 프레임 -> 부품 (이동은 mm)
    mesh: trimesh.Trimesh  # 부품 프레임, mm
    collide: bool = True
    color: str | None = None


@dataclass
class Scene:
    margin_mm: float = 10.0
    parts: list[ScenePart] = field(default_factory=list)


def _transform(xyz_mm, rpy_deg) -> np.ndarray:
    T = trimesh.transformations.euler_matrix(*np.radians(rpy_deg), axes="sxyz")  # URDF rpy: 고정축 x, y, z 순
    T[:3, 3] = xyz_mm
    return T


def _mesh(spec: dict, root: Path, name: str) -> trimesh.Trimesh:
    if "box_mm" in spec:
        w, d, h = (float(v) for v in spec["box_mm"])
        m = trimesh.creation.box(extents=[w, d, h])
        m.apply_translation([0, 0, h / 2])  # 가운데 아래면이 원점
        return m
    if "file" in spec:
        p = Path(spec["file"])
        p = p if p.is_absolute() else root / p
        if not p.exists():
            raise ValueError(f"장면 부품 {name}: 파일이 없다: {p}")
        return trimesh.load(p, force="mesh")
    raise ValueError(f"장면 부품 {name}: file 또는 box_mm가 있어야 한다")


def load_scene(path: str | Path = "config/scene.yaml") -> Scene:
    """파일이 없으면 빈 장면. 상대 경로의 file은 설정 폴더의 위(레포 루트) 기준."""
    path = Path(path)
    if not path.exists():
        return Scene()
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    root = path.resolve().parent.parent
    parts, seen = [], set()
    for spec in data.get("parts") or []:
        name = str(spec.get("name") or "")
        if not name or name in seen:
            raise ValueError(f"장면 부품 이름이 비었거나 겹친다: {name!r}")
        seen.add(name)
        parts.append(ScenePart(name, str(spec.get("attach", "table")), _transform(spec.get("xyz_mm", [0, 0, 0]), spec.get("rpy_deg", [0, 0, 0])),
                               _mesh(spec, root, name), bool(spec.get("collide", True)), spec.get("color")))
    return Scene(float(data.get("margin_mm", 10.0)), parts)
