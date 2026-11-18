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
    gripper: str | None = None  # parallel | single_jaw. None = 어느 그리퍼든 쓴다 (REQ-069)


@dataclass
class Scene:
    margin_mm: float = 10.0
    parts: list[ScenePart] = field(default_factory=list)

    def for_gripper(self, gripper: str) -> "Scene":
        """cfg 그리퍼와 다른 `gripper:` 부품을 뺀 장면. 장면을 쓰는 쪽(ArmService·시뮬레이션 드라이버)이 읽은 뒤 한 번 거른다."""
        return Scene(self.margin_mm, [p for p in self.parts if p.gripper in (None, gripper)])


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


BIN_WALL_T_MM = 3.0  # shortcut: 통 벽 두께는 정해지지 않아 출력물 가정값이다. 통 CAD·H-6 실측이 오면 workspace.yaml 키로 옮긴다


def bin_walls(ws) -> list[ScenePart]:
    """작업대 설정의 통 구역(type: bin, 결정 0067)마다 벽 네 개(높이 wall_mm, 두께 BIN_WALL_T_MM, 안쪽 w×h)를 충돌 부품으로."""
    out = []
    t = BIN_WALL_T_MM
    for name, z in (ws.drop_zones if ws is not None else {}).items():
        if z.get("type") != "bin":
            continue
        x, y, w, h, hw = (float(z[k]) for k in ("x_mm", "y_mm", "w_mm", "h_mm", "wall_mm"))
        for side, (dx, dy, bw, bd) in {"s": (0, -(h + t) / 2, w + 2 * t, t), "n": (0, (h + t) / 2, w + 2 * t, t),
                                       "w": (-(w + t) / 2, 0, t, h), "e": ((w + t) / 2, 0, t, h)}.items():
            m = trimesh.creation.box(extents=[bw, bd, hw])
            m.apply_translation([0, 0, hw / 2])
            out.append(ScenePart(f"bin_{name}_{side}", "table", _transform([x + dx, y + dy, 0.0], [0, 0, 0]), m, True, "#9aa5b1"))
    return out


def load_scene(path: str | Path = "config/scene.yaml", ws=None) -> Scene:
    """파일이 없으면 빈 장면. 상대 경로의 file은 설정 폴더의 위(레포 루트) 기준. ws를 주면 통 구역의 벽(bin_walls)을 더한다."""
    path = Path(path)
    if not path.exists():
        return Scene(parts=bin_walls(ws))
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    root = path.resolve().parent.parent
    parts, seen = [], set()
    for spec in data.get("parts") or []:
        name = str(spec.get("name") or "")
        if not name or name in seen:
            raise ValueError(f"장면 부품 이름이 비었거나 겹친다: {name!r}")
        seen.add(name)
        parts.append(ScenePart(name, str(spec.get("attach", "table")), _transform(spec.get("xyz_mm", [0, 0, 0]), spec.get("rpy_deg", [0, 0, 0])),
                               _mesh(spec, root, name), bool(spec.get("collide", True)), spec.get("color"), spec.get("gripper")))
    return Scene(float(data.get("margin_mm", 10.0)), parts + bin_walls(ws))
