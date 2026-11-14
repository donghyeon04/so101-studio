"""손목 초음파 브래킷(H4) 사전 평가 (M7a, mujoco-spec 13.10, 결정 0062): 출력·구입 전에 STL로 질량·토크·간섭·오버행·센서 오프셋을 잰다.

  python -m so101_studio.arm.parts_eval [--assembly "Gripper(...).stl" [--export]] [--json out.json]

조립 STL(로보나인 병렬 그리퍼 + 브래킷 + HC-SR04, 145개 바디, 이름 없음)의 0~43과 142는 그리퍼(이미 URDF에 있음, ICP로 손 메쉬에 맞음)이고
44~141·143은 HC-SR04 모듈(143은 기판), 144는 출력할 브래킷 판이다(바디 번호는 CAD 내보내기 순서라 조립이 바뀌면 다시 확인한다: 추정).
--export는 모듈과 브래킷을 gripper_link 좌표로 옮겨 assets/scene/에 쓴다. 평가는 그 파일과 URDF만으로 돈다.
좌표 변환(조립 -> gripper_link, mm)은 조립의 판 바디(0번)를 URDF의 parallel_hand 메쉬에 맞춰 구했다(99%가 1.6mm 안).
shortcut: placo가 한글 경로를 못 읽어 URDF를 상대 경로로 연다(레포 루트에서 실행). 질량은 출력 바디마다 벽 2겹(0.8mm)+채움 20% 보정, HC-SR04는 8.5g 고정(추정, 슬라이서 무게·실측이 오면 바꾼다)."""
import argparse
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "assets" / "so101_parallel.urdf"
SCENE_DIR = ROOT / "assets" / "scene"
ASM_TO_GL = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, -53.3], [0, 0, 0, 1]], float)  # 조립 -> gripper_link (mm)
BODIES = {"sensor": [*range(44, 142), 143], "bracket": [144]}
NAMES = {"sensor": "wrist_hcsr04", "bracket": "wrist_bracket"}
PLA_G_MM3 = 1.24e-3
SENSOR_G = 8.5  # HC-SR04 모듈 질량(추정)
WALL_MM, INFILL = 0.8, 0.20
G = 9.81
STALL_NM = 2.94  # STS3215 12V 정지 토크 30kg·cm. 연속 사용은 이것의 1/3 안팎으로 본다
TIP_Z_GL = -131.8  # 두 턱 끝(gripper_link 프레임 mm). TCP는 10mm 위


def export(assembly: str | Path) -> dict[str, Path]:
    """조립 STL에서 센서 모듈·브래킷 바디를 gripper_link 좌표 STL로 쓴다."""
    parts = trimesh.load(assembly).split(only_watertight=False)
    if len(parts) != 145:
        raise ValueError(f"조립의 바디 수가 145가 아니다({len(parts)}): BODIES 번호를 다시 확인한다")
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for key, ids in BODIES.items():
        m = trimesh.util.concatenate([parts[i] for i in ids])
        m.apply_transform(ASM_TO_GL)
        out[key] = SCENE_DIR / f"{NAMES[key]}.stl"
        m.export(out[key])
    return out


def load_parts() -> dict[str, trimesh.Trimesh]:
    return {k: trimesh.load(SCENE_DIR / f"{n}.stl", force="mesh") for k, n in NAMES.items()}


def _printed_mass_g(m: trimesh.Trimesh) -> float:
    v = abs(float(trimesh.convex.convex_hull(m).volume)) if not m.is_watertight else abs(float(m.volume))
    shell = min(v, m.area * WALL_MM)
    return PLA_G_MM3 * (shell + INFILL * (v - shell))


def mass_properties(parts) -> dict:
    """질량(g), 무게중심(gripper_link mm), 관성(g·mm², 무게중심 기준)."""
    br = parts["bracket"]
    items = [("bracket", br, _printed_mass_g(br)), ("sensor", parts["sensor"], SENSOR_G)]
    total = sum(w for _, _, w in items)
    com = sum(w * m.center_mass if m.is_watertight else w * m.convex_hull.center_mass for _, m, w in items) / total
    inertia = np.zeros((3, 3))
    per = {}
    for name, m, w in items:
        mm = m if m.is_watertight else m.convex_hull
        c = mm.center_mass
        I = mm.moment_inertia / abs(mm.volume) * w  # 밀도 균일 가정, 질량에 맞춰 환산
        d = c - com
        inertia += I + w * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        per[name] = {"mass_g": round(w, 1), "com_mm": c.round(1).tolist(), "volume_mm3": round(abs(float(mm.volume)))}
    return {"mass_g": round(total, 1), "com_mm": com.round(1).tolist(), "inertia_g_mm2": inertia.round(0).tolist(), "parts": per,
            "estimate": f"PLA {PLA_G_MM3 * 1000:.2f}g/cm³, 벽 {WALL_MM}mm + 채움 {INFILL:.0%}, HC-SR04 {SENSOR_G}g 고정"}


def _pose(k, q) -> None:
    for _ in range(2):  # placo는 첫 갱신에서 링크 변환이 낡은 채 나온다(collision.py _pose와 같다)
        k.forward_kinematics(np.array(q, float))
        k.robot.update_kinematics()


def _urdf_links():
    root = ET.parse(URDF).getroot()
    mass = {}
    for link in root.findall("link"):
        i = link.find("inertial")
        if i is not None:
            o = i.find("origin")
            mass[link.get("name")] = (float(i.find("mass").get("value")), np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()]))
    joints = {j.find("child").get("link"): (j.get("name"), np.array([float(x) for x in (j.find("axis").get("xyz") if j.find("axis") is not None else "0 0 0").split()])) for j in root.findall("joint")}
    return mass, joints


def _kin():
    from lerobot.model.kinematics import RobotKinematics

    from .kinematics import ARM_JOINTS
    return RobotKinematics(os.path.relpath(URDF), "gripper_frame_link", joint_names=ARM_JOINTS)


POSES = {  # 어깨·팔꿈치·손목 굽힘·손목 회전(°). 곧게 뻗은 0° 자세가 어깨와 손목 토크 최악이다.
    "0° 곧게 뻗음": [0, 0, 0, 0, 0],
    "손끝 아래(뻗음)": [0, 0, 0, 90, 0],
    "작업 자세(반경 약 250mm)": [0, -45, 50, 85, 0],
}
CHAIN = ["shoulder_lift", "elbow_flex", "wrist_flex"]
CHILD = {"shoulder_pan": "shoulder_link", "shoulder_lift": "upper_arm_link", "elbow_flex": "lower_arm_link", "wrist_flex": "wrist_link", "wrist_roll": "gripper_link"}


def static_torque(extra: dict, k=None) -> dict:
    """자세마다 어깨·팔꿈치·손목 굽힘 관절의 정적 중력 토크(N·m)와 부품을 달았을 때 늘어나는 양. 부품은 gripper_link에 붙는다."""
    k = k or _kin()
    mass, joints = _urdf_links()
    order = ["shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "clamp_right", "clamp_left"]
    out = {}
    for pose, q in POSES.items():
        _pose(k, q)
        T = {n: k.robot.get_T_world_frame(n) for n in order if n in mass or n == "gripper_link"}
        row = {}
        for jn in CHAIN:
            child = CHILD[jn]
            a_loc = joints[child][1]
            Tj = T[child]
            axis, p0 = Tj[:3, :3] @ a_loc, Tj[:3, 3]
            tot = add = 0.0
            for n in order[order.index(child):]:
                m, c = mass.get(n, (0.0, np.zeros(3)))
                r = (T[n] @ np.append(c, 1.0))[:3]
                tot += m * G * float(axis @ np.cross(r - p0, [0, 0, -1.0]))
                if n == "gripper_link":
                    ra = (T[n] @ np.append(np.array(extra["com_mm"]) / 1000.0, 1.0))[:3]
                    add = extra["mass_g"] / 1000.0 * G * float(axis @ np.cross(ra - p0, [0, 0, -1.0]))
            row[jn] = {"base_nm": round(abs(tot), 3), "added_nm": round(abs(add), 3), "with_nm": round(abs(tot + add), 3), "continuous_pct": round(100 * abs(tot + add) / (STALL_NM / 3), 1)}
        out[pose] = row
    return out


def joint_end_interference(parts, k=None) -> dict:
    """손목 굽힘 ±103.6°, 회전 ±168.8°와 어깨·팔꿈치 몇 자세에서 부품과 팔 링크 사이 최소 거리(mm). 음수·0이면 닿는다. 링크는 볼록 껍질 근사(안전한 쪽)."""
    import coal

    from .collision import _T, SceneCollision, _hull
    k = k or _kin()
    links = SceneCollision._load_links(URDF)
    own = {"gripper_link", "clamp_right", "clamp_left", "base_link", "shoulder_link"}  # 몸통 쪽 두 링크는 브래킷이 닿을 수 없다
    # 브래킷은 바디마다 볼록체(합치면 오목한 틈이 메워져 실제보다 크게 나온다), 센서 모듈은 한 덩어리
    hulls = [(NAMES["sensor"], _hull(np.asarray(parts["sensor"].vertices, float) / 1000.0))]
    hulls += [(f"{NAMES['bracket']}#{i}{np.round(b.extents).astype(int).tolist()}", _hull(np.asarray(b.vertices, float) / 1000.0)) for i, b in enumerate(parts["bracket"].split(only_watertight=False))]
    res = {}
    for wf in (-103.6, 0, 103.6):
        for wr in (-168.8, 0, 168.8):
            for ls, ef in ((0, 0), (-90, 93), (60, -90), (-100, 100)):
                _pose(k, [0, ls, ef, wf, wr])
                Tg = k.robot.get_T_world_frame("gripper_link")
                best = (math.inf, "")
                for pn, h in hulls:
                    for ln, lh in links:
                        if ln in own:
                            continue
                        req, rs = coal.DistanceRequest(), coal.DistanceResult()
                        coal.distance(h, _T(Tg), lh, _T(k.robot.get_T_world_frame(ln)), req, rs)
                        if rs.min_distance * 1000 < best[0]:
                            best = (rs.min_distance * 1000, f"{pn}-{ln}")
                res[f"wf{wf:g} wr{wr:g} lift{ls} elbow{ef}"] = (round(best[0], 1), best[1])
    worst = min(res.items(), key=lambda kv: kv[1][0])
    return {"min_mm": worst[1][0], "worst": worst[0], "pair": worst[1][1], "n_poses": len(res), "touching": [k2 for k2, v in res.items() if v[0] <= 0]}


def beam_clearance(parts, half_deg: float = 15.0, length_mm: float = 110.0) -> dict:
    """센서 면에서 원뿔(±15°, 110mm) 안에 그리퍼(손·턱)가 들어오는지. 광선 격자로 첫 충돌 거리를 잰다(턱 닫힘·최대 열림)."""
    root = ET.parse(URDF).getroot()
    sensor = parts["sensor"]
    # 센서 면 = 센서 모듈에서 턱 끝 쪽(-z) 끝에 있는 얇은 원판 중심 두 개의 가운데
    c = _sensor_face(sensor)
    meshes = []
    for link in root.findall("link"):
        if link.get("name") not in {"gripper_link", "clamp_right", "clamp_left"}:
            continue
        for vis in link.findall("visual"):
            msh = vis.find("geometry/mesh")
            o = vis.find("origin")
            xyz = np.array([float(x) for x in o.get("xyz").split()]) * 1000
            rpy = [float(x) for x in o.get("rpy").split()]
            m = trimesh.load(URDF.parent / msh.get("filename"), force="mesh")
            m.apply_transform(trimesh.transformations.euler_matrix(*rpy, axes="sxyz"))
            m.apply_translation(xyz)
            meshes.append((link.get("name"), m))
    out = {}
    for label, gap_mm in (("닫힘", 0.0), ("최대 열림", 84.0)):
        hits = {}
        for name, m in meshes:
            mm = m.copy()
            if name == "clamp_right":
                mm.apply_translation([gap_mm / 2, 0, 0])
            elif name == "clamp_left":
                mm.apply_translation([-gap_mm / 2, 0, 0])
            ri = trimesh.ray.ray_triangle.RayMeshIntersector(mm)
            d = [[math.sin(el) * math.cos(az), math.sin(el) * math.sin(az), -math.cos(el)] for az in np.linspace(0, 2 * math.pi, 24, endpoint=False) for el in np.radians([half_deg * 0.5, half_deg])]
            d = np.array(d + [[0, 0, -1.0]])
            loc, _, _ = ri.intersects_location(np.tile(c, (len(d), 1)), d, multiple_hits=False)
            h = float(np.min(np.linalg.norm(loc - c, axis=1))) if len(loc) else math.inf
            hits["손(URDF parallel_hand)" if name == "gripper_link" else "턱"] = min(h, hits.get("손(URDF parallel_hand)" if name == "gripper_link" else "턱", math.inf))
        out[label] = {k: (None if math.isinf(v) else round(v, 1)) for k, v in hits.items()}
    out["센서 면(gripper_link mm)"] = c.round(1).tolist()
    return out


def _sensor_face(sensor: trimesh.Trimesh) -> np.ndarray:
    """센서 면 = 가장 면이 많은(음파 막) 얇은 원판 두 개의 가운데 점."""
    thin = [p for p in sensor.split(only_watertight=False) if p.extents.min() < 0.5 and p.extents.max() > 14]
    thin = sorted(thin, key=lambda p: -len(p.faces))[:2]
    if len(thin) != 2 or len(thin[1].faces) < 5000:
        raise ValueError("센서 막 원판 2개를 못 찾았다: 조립이 바뀌었다")
    return np.mean([p.centroid for p in thin], axis=0)


def sensor_offset(parts) -> dict:
    """센서 면 중심을 두 턱 끝 가운데 기준(조립 좌표와 같은 방향: x 열림 축, y 옆, z 턱 끝에서 손목 쪽 높이)으로. 지금 값과 비교."""
    c = _sensor_face(parts["sensor"])
    tip = np.array([0.0, 0.0, TIP_Z_GL])
    v = c - tip
    off = {"x": round(float(v[0]), 1), "y": round(float(-v[1]), 1), "z": round(float(v[2]), 1)}  # gripper_link -> 조립 방향(y만 뒤집힘)
    now = {"x": 0.6, "y": 66.5, "z": 99.5}
    return {"cad_mm": off, "now_mm": now, "diff_mm": {k: round(off[k] - now[k], 1) for k in off}}


def overhang(parts) -> dict:
    """출력 방향 6가지(축 정렬)마다 서포트가 필요한 아랫면 면적 비율(법선이 수직 아래에서 45° 안, 바닥에 닿는 면 제외)."""
    out = {}
    for label, down in (("+x", [1, 0, 0]), ("-x", [-1, 0, 0]), ("+y", [0, 1, 0]), ("-y", [0, -1, 0]), ("+z", [0, 0, 1]), ("-z", [0, 0, -1])):
        m = parts["bracket"]
        n, a = m.face_normals, m.area_faces
        d = -np.array(down, float)  # 빌드 플레이트 쪽 방향
        facing = n @ d > math.cos(math.radians(45))
        base = m.triangles_center @ d >= (m.triangles_center @ d).max() - 0.3  # 맨 아래 접촉면
        out[label] = round(float(a[facing & ~base].sum() / a.sum()) * 100, 1)
    best = min(out, key=out.get)
    return {"support_area_pct_by_down_axis": out, "best_down_axis": best, "best_pct": out[best],
            "note": "아랫방향 축 = 빌드 플레이트에 닿는 쪽(gripper_link 좌표). 센서 앞쪽이 위인 방향은 U7"}


def hand_overlap(parts) -> dict:
    """URDF 손 메쉬(손목 쪽 형상 포함) 표면 점이 부품의 경계 상자 안에 몇 개나 들어오는지(30만 점 중). 0이 아니면 CAD 조립에 없는 형상과 겹칠 수 있다(상자라 거칠다)."""
    from scipy.spatial.transform import Rotation as Rot
    ph = trimesh.load(SCENE_DIR.parent / "assets" / "parallel" / "parallel_hand.stl", force="mesh")
    v = Rot.from_euler("x", 1.570796).apply(ph.vertices) + np.array([-7.062, 152.066, 120.627])
    pts = trimesh.Trimesh(v, ph.faces).sample(300000)
    return {k: int(((pts >= m.bounds[0]) & (pts <= m.bounds[1])).all(1).sum()) for k, m in parts.items()}


def report() -> dict:
    k = _kin()  # torch(lerobot)를 메쉬·scipy보다 먼저 읽어야 Windows에서 DLL 충돌이 없다
    parts = load_parts()
    mp = mass_properties(parts)
    return {"mass": mp, "torque": static_torque(mp, k), "interference": joint_end_interference(parts, k), "beam": beam_clearance(parts), "hand_overlap_pts": hand_overlap(parts), "offset": sensor_offset(parts), "overhang": overhang(parts)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--assembly", help="조립 STL(Gripper(브라켓 짧은 버전).stl 등)")
    ap.add_argument("--export", action="store_true", help="--assembly에서 부품 STL을 assets/scene/에 쓴다")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    import lerobot.model.kinematics  # noqa: F401  Windows에서 torch를 메쉬·scipy보다 먼저 읽어야 DLL 충돌이 없다
    if a.export:
        if not a.assembly:
            ap.error("--export는 --assembly가 필요하다")
        print("쓴 파일:", {k: str(v) for k, v in export(a.assembly).items()})
    r = report()
    text = json.dumps(r, ensure_ascii=False, indent=1)
    if a.json:
        Path(a.json).write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
