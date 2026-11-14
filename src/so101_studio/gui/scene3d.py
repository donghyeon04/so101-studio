"""Viser 3D 장면: URDF 팔 두 개(실제, 계산 자세), 작업대, 물체, 배출 구역, 좌표축 (gui-spec 4·8절).
장면 좌표는 작업대 좌표를 m로 옮긴 것이고, z = 0이 작업대 면이다."""

import logging
import math
import threading
import time
from pathlib import Path

import numpy as np
import trimesh
import viser
import viser.transforms as tf
import yourdfpy
from viser.extras import ViserUrdf

from ..arm.kinematics import ARM_JOINTS, PAN_AXIS_X_MM, VALUE_TO_Q
from ..core.frames import robot_to_table
from . import theme

HOST = "127.0.0.1"
DEFAULT_PORT = 8081
TABLE_T_M = 0.01
PEDESTAL_M = 0.1  # 받침판 한 변(m): SO-101 바닥판 크기쯤
OBJECT_H_M = 0.03
PREVIEW_ALPHA = 0.4
PART_ALPHA = 0.85  # 장면 부품: 뒤의 팔·물체가 조금 비치게
MESH_CELL_M = 0.002
ALLOWED_TOP_MM = 300.0  # 높이 제한이 없을 때 허용 상자를 그리는 높이
DEFAULT_PICK_H_MM = 40.0  # 작업대를 클릭했을 때의 높이. 0이면 턱 끝이 작업대에 닿는다
Z_UP = (0.0, 0.0, 1.0)
LAYERS = {
    "arm": "실제 팔", "preview": "계산 자세", "table": "작업대", "zones": "배출 구역", "objects": "물체",
    "axes": "좌표축", "marks": "목표·경로", "handle": "손잡이",
}  # 3D에서 켜고 끄는 묶음
ORBIT_DEG_PER_PX = 0.8  # 뷰큐브를 끌 때
ORBIT_MAX_ELEVATION_DEG = 87.0  # 위쪽 축을 넘어가면 화면이 뒤집힌다
VIEW_S_PER_180 = 0.75  # 시점 전환 시간은 회전 각도에 비례한다(Blender 방식). 180°에 AutoCAD 기본값 750ms
VIEW_MIN_S = 0.25  # 작은 회전이 너무 빨리 끝나지 않게 한다
VIEW_HZ = 60  # Viser가 메시지를 내보내는 주기와 같다
VIEW_SETTLE_S = 1.0  # 전환이 끝난 뒤 브라우저가 늦게 돌려주는 값을 무시하는 최대 시간. 사람이 3D를 직접 돌리면 이 뒤로는 따라간다


def table_hit(origin, direction) -> tuple[float, float] | None:
    """클릭 광선과 작업대 면(z = 0)의 교점을 작업대 mm로. 면을 향하지 않으면 None."""
    if direction[2] >= 0 or origin[2] <= 0:
        return None
    t = -origin[2] / direction[2]
    return (origin[0] + t * direction[0]) * 1000, (origin[1] + t * direction[1]) * 1000


def urdf_cfg(q, names, width_mm=None) -> np.ndarray:
    """JointState(도, 그리퍼는 값)를 URDF 관절값으로. names는 URDF의 관절 순서.
    width_mm([[값, 폭 mm], ...])를 주면 병렬 그리퍼(REQ-041): gripper는 직선 관절이라 +폭/2(m), gripper_left는 -폭/2."""
    out = {j: math.radians(getattr(q, j)) for j in ARM_JOINTS}
    if width_mm:
        pts = sorted(width_mm)
        half = float(np.interp(q.gripper, [p[0] for p in pts], [p[1] for p in pts])) / 2000.0
        out |= {"gripper": half, "gripper_left": -half}
    else:
        out["gripper"] = math.radians(VALUE_TO_Q[0] + VALUE_TO_Q[1] * q.gripper)
    return np.array([out.get(n, 0.0) for n in names])


def simplify(mesh, cell_m: float = MESH_CELL_M) -> trimesh.Trimesh:
    """격자 한 칸 안의 꼭짓점을 하나로 합친다. 원본 STL은 면이 40만 개라 브라우저로 보내는 데 오래 걸린다."""
    cells, index = np.unique(np.round(mesh.vertices / cell_m).astype(np.int64), axis=0, return_inverse=True)
    index = index.ravel()
    vertices = np.zeros((len(cells), 3))
    np.add.at(vertices, index, mesh.vertices)
    vertices /= np.bincount(index)[:, None]
    faces = index[mesh.faces]
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return trimesh.Trimesh(vertices, faces[keep], process=False, face_colors=mesh.visual.main_color)


def _load_urdf(path: Path) -> yourdfpy.URDF:
    """팔마다 따로 읽는다. ViserUrdf가 관절값을 URDF 객체에 써서, 나눠 쓰면 스레드끼리 섞인다."""
    urdf = yourdfpy.URDF.load(path, build_scene_graph=True, load_meshes=True, mesh_dir=str(path.parent))
    for name, mesh in list(urdf.scene.geometry.items()):
        urdf.scene.geometry[name] = simplify(mesh)
    return urdf


def link_node(urdf: yourdfpy.URDF, link: str, root: str) -> str:
    """URDF 링크가 ViserUrdf 안에서 갖는 장면 노드 이름(viser.extras._urdf와 같은 규칙). 이 노드 아래에 둔 것은 링크를 따라 움직인다."""
    graph = urdf.scene.graph
    frames = []
    while link != graph.base_frame:
        frames.append(link)
        link = graph.transforms.parents[link]
    return root + "/" + "/".join(reversed(frames))


def _yaw(deg: float) -> np.ndarray:
    return tf.SO3.from_z_radians(math.radians(deg)).wxyz


def up_for(direction) -> tuple[float, float, float]:
    """그 방향에서 볼 때의 위쪽. 곧장 위나 아래에서 보면 z를 위쪽으로 쓸 수 없어, 앞면이 위 시점에서는 아래쪽, 아래 시점에서는 위쪽에 오게 한다."""
    if direction[0] or direction[1]:
        return Z_UP
    return (0.0, 1.0, 0.0) if direction[2] > 0 else (0.0, -1.0, 0.0)


def orbit_offset(offset, up, dx_px: float, dy_px: float) -> np.ndarray:
    """끈 만큼 돌린 (카메라 - 보는 점). 오른쪽으로 끌면 큐브가 오른쪽으로 돌고(카메라는 왼쪽으로), 아래로 끌면 위에서 보게 된다. 돌린 뒤 위쪽은 z다."""
    offset = np.asarray(offset, float)
    distance = np.linalg.norm(offset)
    level = offset[:2]
    if np.linalg.norm(level) < 1e-3 * distance:  # 곧장 위나 아래: 수평 방향은 위쪽 벡터로 정해진다 (up_for와 짝)
        level = -np.sign(offset[2]) * np.asarray(up, float)[:2]
    azimuth = math.atan2(level[1], level[0]) - math.radians(dx_px * ORBIT_DEG_PER_PX)
    limit = math.radians(ORBIT_MAX_ELEVATION_DEG)
    elevation = float(np.clip(math.asin(np.clip(offset[2] / distance, -1, 1)) + math.radians(dy_px * ORBIT_DEG_PER_PX), -limit, limit))
    return distance * np.array((math.cos(elevation) * math.cos(azimuth), math.cos(elevation) * math.sin(azimuth), math.sin(elevation)))


def view_ease(t: float) -> float:
    """시점 전환의 속도 곡선. 천천히 시작해 천천히 멈춘다(Blender와 같은 식, 최고 속도는 평균의 1.5배). 느낌을 바꾸려면 이 식만 고친다."""
    return 3 * t * t - 2 * t * t * t


def camera_rotation(position, look_at, up) -> tf.SO3:
    """카메라 자세. 회전 행렬의 열이 (오른쪽, 위, 뒤)다."""
    back = np.asarray(position, float) - np.asarray(look_at, float)
    back /= np.linalg.norm(back)
    right = np.cross(np.asarray(up, float), back)
    if np.linalg.norm(right) < 1e-6:  # 위쪽 축을 곧장 보는 중: 시선과 가장 덜 겹치는 축을 위쪽 대신 쓴다
        right = np.cross(np.eye(3)[np.argmin(np.abs(back))], back)
    right /= np.linalg.norm(right)
    return tf.SO3.from_matrix(np.column_stack((right, np.cross(back, right), back)))


def view_path(start, goal) -> tuple:
    """(걸리는 초, 진행률 0~1을 (위치, 보는 점, 위쪽)으로 바꾸는 함수). start와 goal도 그 형식이다. 쿼터니언으로 최단 경로를 돌아 위·아래 시점을 지나도 뒤집히지 않는다."""
    begin, turn = camera_rotation(*start), None
    turn = (begin.inverse() @ camera_rotation(*goal)).log()
    look0, look1 = np.asarray(start[1], float), np.asarray(goal[1], float)
    dist0 = np.linalg.norm(np.asarray(start[0], float) - look0)
    dist1 = np.linalg.norm(np.asarray(goal[0], float) - look1)

    def at(t: float) -> tuple:
        if t >= 1.0:
            return goal
        s = view_ease(t)
        axes = (begin @ tf.SO3.exp(turn * s)).as_matrix()
        look = look0 + (look1 - look0) * s
        return look + axes[:, 2] * (dist0 + (dist1 - dist0) * s), look, axes[:, 1]

    return max(VIEW_MIN_S, VIEW_S_PER_180 * float(np.linalg.norm(turn)) / math.pi), at


class Scene3D:
    def __init__(self, cfg, ws, kin, port: int = DEFAULT_PORT, host: str = HOST):
        self.server = viser.ViserServer(host=host, port=port, label="SO-101 Studio", verbose=False)
        self.server.gui.configure_theme(
            dark_mode=False, show_logo=False, show_share_button=False, brand_color=theme.rgb(theme.ACCENT)
        )
        self.server.gui.main_panel.dock_right()  # 떠 있는 창은 3D 칸이 좁아지면(조그 패널을 열 때) 다시 펼쳐져 팔을 가린다: 오른쪽 띠로 붙여 접는다
        self.server.gui.main_panel.minimize()  # 조작은 NiceGUI 쪽에서 한다. 이 패널은 3D를 가리기만 한다
        self._kin, self._table_z = kin, cfg.table_z_mm
        self._width_mm = cfg.gripper_width_mm if getattr(cfg, "gripper", "single_jaw") == "parallel" else None  # 병렬 URDF는 그리퍼가 직선 관절
        self.layers = dict.fromkeys(LAYERS, True)  # 브라우저가 여럿이어도 3D는 하나라 함께 바뀐다
        self._preview_on = False
        scene = self.server.scene
        table, grid, zones, axes_robot, root = self._draw_workspace(ws)
        w, h = ws.table_mm["x"] / 1000, ws.table_mm["y"] / 1000
        axes_table = scene.add_frame("/axes/table", axes_length=0.06, axes_radius=0.002)
        self._tool = scene.add_frame("/axes/tool", axes_length=0.04, axes_radius=0.0015)
        urdf_path = Path(cfg.urdf)
        real_root = self._real_root = scene.add_frame("/real", show_axes=False, **root)
        self._real_urdf = _load_urdf(urdf_path)
        self._real = ViserUrdf(self.server, self._real_urdf, root_node_name="/real")
        self._preview_root = scene.add_frame("/preview", show_axes=False, visible=False, **root)
        self._preview = ViserUrdf(
            self.server, _load_urdf(urdf_path), root_node_name="/preview",
            mesh_color_override=(*(c / 255 for c in theme.rgb(theme.PREVIEW)), PREVIEW_ALPHA),
        )
        self._names = self._real.get_actuated_joint_names()
        self.picked = (0, 0.0, 0.0, 0.0)  # (번호, x, y, 높이). 번호가 바뀌면 새 점이다
        self._handle = scene.add_transform_controls(
            "/handle", scale=0.09, disable_rotations=True, depth_test=False, visible=True
        )
        self._handle.on_update(self._on_drag)
        self._objects_root = None
        self._sim_root = None  # MuJoCo 물리 물체(gui-spec 12.8). 계획 쪽 /objects와 따로 둔다
        self._sim_nodes: dict = {}  # 이름 -> (상자 노드, 지금 색)
        self.plan_opacity = 1.0  # MuJoCo면 Backend가 0.4로 둔다: 계획 물체는 흐리게, 물리 물체는 진하게
        marks = scene.add_frame("/marks", show_axes=False)
        parts = scene.add_frame("/parts", show_axes=False)
        self._part_nodes: list[str] = []
        self._layer_nodes = {
            "arm": [real_root], "table": [table, grid, parts], "zones": zones, "axes": [axes_table, axes_robot, self._tool],
            "marks": [marks], "handle": [self._handle],
        }
        scene.on_click()(self._on_click)
        cam = self.server.initial_camera
        cam.position, cam.look_at, cam.up = (w / 2, -0.22, 0.34), (w / 2, h / 2 - 0.02, 0.06), Z_UP
        self._home = ((w / 2, -0.22, 0.34), (w / 2, h / 2 - 0.02, 0.06), Z_UP)
        self.camera = self._home  # (위치, 보는 점, 위쪽). 뷰큐브가 읽는다. 3D 칸이 여럿이면 마지막으로 움직인 칸의 값이다
        self._camera_ready: set[int] = set()
        self._flight = 0  # 시점 전환 번호. 새 전환이나 끌기가 올리면 앞 전환이 멈춘다
        self._flying = False
        self._flight_thread: threading.Thread | None = None
        self._sent: dict[int, tuple] = {}  # 3D 칸마다 마지막으로 보낸 시점
        self._settling: dict[int, tuple] = {}  # 전환을 마치고 브라우저가 따라오길 기다리는 칸: (목표 위치, 기한)
        self.server.on_client_connect(self._watch_camera)
        self.server.on_client_disconnect(lambda client: self._camera_ready.discard(client.client_id))

    @property
    def url(self) -> str:
        return self.url_for(HOST)

    def url_for(self, hostname: str | None) -> str:
        """그 브라우저가 연 호스트로 3D 주소를 만든다(--share면 다른 기기는 PC의 IP로 연다, gui-spec 12.6)."""
        host = hostname or HOST
        return f"http://{'[' + host + ']' if ':' in host else host}:{self.server.get_port()}"

    def update_status(self, status) -> None:
        """status_stream 콜백. 실제 팔과 툴 좌표축을 지금 자세로 옮긴다."""
        t = status.tool
        with self.server.atomic():
            self._real.update_cfg(urdf_cfg(status.joints, self._names, self._width_mm))
            self._tool.position = (t.x_mm / 1000, t.y_mm / 1000, (t.z_mm - self._table_z) / 1000)
            self._tool.wxyz = tf.SO3.from_matrix(self._kin.tool_axes(status.joints)).wxyz

    def set_preview(self, joints) -> None:
        """계산 자세를 보여 준다. None이면 숨긴다."""
        self._preview_on = joints is not None
        self._preview_root.visible = self._preview_on and self.layers["preview"]
        if joints is not None:
            self._preview.update_cfg(urdf_cfg(joints, self._names, self._width_mm))

    def _draw_workspace(self, ws):
        """작업대·격자·배출 구역·로봇 좌표축을 그리고, 팔이 놓일 자리(URDF 원점)를 돌려준다. 같은 이름으로 다시 그리면 바뀐다."""
        scene = self.server.scene
        w, h = ws.table_mm["x"] / 1000, ws.table_mm["y"] / 1000
        table = scene.add_box(
            "/table", color=theme.rgb(theme.CARD), dimensions=(w, h, TABLE_T_M), position=(w / 2, h / 2, -TABLE_T_M / 2)
        )
        grid = scene.add_grid(
            "/grid", width=w, height=h, cell_size=0.05, section_size=0.1, position=(w / 2, h / 2, 0.0005),
            cell_color=theme.rgb(theme.LINE), section_color=theme.rgb(theme.CONTROL_BORDER),
        )
        zones = [scene.add_box(f"/zones/{name}", color=theme.rgb(theme.OK), dimensions=(zone.get("w_mm", 80) / 1000, zone.get("h_mm", 80) / 1000, 0.001), opacity=0.35,
                               position=(zone["x_mm"] / 1000, zone["y_mm"] / 1000, 0.001)) for name, zone in ws.drop_zones.items()]
        base = ws.base
        axes_robot = scene.add_frame("/axes/robot", axes_length=0.06, axes_radius=0.002,
                                     position=(base.x_mm / 1000, base.y_mm / 1000, 0.001), wxyz=_yaw(base.yaw_deg))
        # URDF 원점은 어깨 회전축에서 PAN_AXIS_X_MM 뒤에 있다 (Kinematics.fk와 같은 오프셋)
        ox, oy = robot_to_table(-PAN_AXIS_X_MM, 0.0, base)
        root = {"position": (ox / 1000, oy / 1000, -self._table_z / 1000), "wxyz": _yaw(base.yaw_deg)}
        lift = -self._table_z / 1000
        if lift > 0:  # table_z_mm(실측 보정, 처짐 포함)만큼 팔이 작업대 위에 떠 보인다: 실물은 받침이 책상에 붙어 있으니 그 틈을 받침판으로 채워 그린다(화면만, 계산은 그대로)
            scene.add_box("/pedestal", color=theme.rgb(theme.MUTED), dimensions=(PEDESTAL_M, PEDESTAL_M, lift),
                          position=(base.x_mm / 1000, base.y_mm / 1000, lift / 2), wxyz=_yaw(base.yaw_deg))
        return table, grid, zones, axes_robot, root

    def set_workspace(self, ws) -> None:
        """작업대·좌표를 저장한 뒤 다시 그린다(앱을 다시 켜지 않아도 된다). 지운 배출 구역은 지운다."""
        old_zones = self._layer_nodes["zones"]
        table, grid, zones, axes_robot, root = self._draw_workspace(ws)
        for node in old_zones:
            if node not in zones and node.name not in {z.name for z in zones}:
                node.remove()
        for frame in (self._real_root, self._preview_root):
            frame.position, frame.wxyz = root["position"], root["wxyz"]
        parts = self._layer_nodes["table"][2]
        self._layer_nodes.update(table=[table, grid, parts], zones=zones)
        self._layer_nodes["axes"][1] = axes_robot
        for name in ("table", "zones", "axes"):
            self.set_layer(name, self.layers[name])

    def set_layer(self, name: str, visible: bool) -> None:
        """묶음 하나를 켜거나 끈다. 자리만 바꾸므로 끈 동안의 자세·물체도 켜면 바로 맞게 보인다."""
        self.layers[name] = visible
        if name == "preview":
            self._preview_root.visible = self._preview_on and visible
        elif name == "objects":
            for root in (self._objects_root, self._sim_root):
                if root:
                    root.visible = visible
        else:
            for node in self._layer_nodes[name]:
                node.visible = visible

    def set_objects(self, objects, chosen: str | None = None) -> None:
        """물체 목록을 다시 그린다. objects는 DetectedObject들이고, chosen은 강조한다."""
        scene = self.server.scene
        with self.server.atomic():
            scene.remove_by_name("/objects")
            self._objects_root = scene.add_frame("/objects", show_axes=False, visible=self.layers["objects"])
            for o in objects:
                scene.add_box(
                    f"/objects/{o.id}", color=theme.rgb(theme.ACCENT if o.id == chosen else theme.MUTED),
                    dimensions=(o.length_mm / 1000, o.width_mm / 1000, OBJECT_H_M),
                    position=(o.x_mm / 1000, o.y_mm / 1000, OBJECT_H_M / 2), wxyz=_yaw(o.angle_deg or 0.0), opacity=self.plan_opacity,
                )

    def set_sim_objects(self, objects: dict, held: str | None) -> None:
        """MuJoCo 물리 물체(sim_state()의 objects, gui-spec 12.8). 상자는 처음 한 번 만들고 뒤로는 자리·방향·색만 바꾼다. 기울기는 그리지 않는다."""
        scene = self.server.scene
        with self.server.atomic():
            if self._sim_root is None:
                self._sim_root = scene.add_frame("/sim_objects", show_axes=False, visible=self.layers["objects"])
            for name, o in objects.items():
                length, width, height = o["size_mm"]
                color = theme.rgb(theme.ACCENT if name == held else theme.WARN if o.get("tilt_deg", 0.0) >= 10 else theme.MUTED)
                pos = (o["x_mm"] / 1000, o["y_mm"] / 1000, (o["z_mm"] + height / 2) / 1000)
                if name not in self._sim_nodes:
                    node = scene.add_box(f"/sim_objects/{name}", color=color, dimensions=(length / 1000, width / 1000, height / 1000),
                                         position=pos, wxyz=_yaw(o["yaw_deg"]))
                    self._sim_nodes[name] = (node, color)
                    continue
                node, old = self._sim_nodes[name]
                node.position, node.wxyz = pos, _yaw(o["yaw_deg"])
                if old != color:
                    node.color = color
                    self._sim_nodes[name] = (node, color)

    def set_parts(self, scene_def) -> None:
        """장면 부품(결정 0043-4, REQ-040)을 그린다. 작업대 부품은 작업대 좌표에, 링크 부품은 그 링크 노드 아래에 두어 팔을 따라 움직이게 한다.
        scene_def는 core.scene.Scene(parts: name, attach, T(4x4, mm), mesh(mm), color). None이면 지운다."""
        scene = self.server.scene
        with self.server.atomic():
            for name in self._part_nodes:
                scene.remove_by_name(name)
            self._part_nodes = []
            for part in getattr(scene_def, "parts", None) or []:
                if part.attach == "table":
                    parent = "/parts"
                elif part.attach in self._real_urdf.scene.graph.nodes:
                    parent = link_node(self._real_urdf, part.attach, "/real")
                else:
                    logging.getLogger(__name__).warning("장면 부품 %s: 모르는 링크 %s", part.name, part.attach)
                    continue
                T = np.asarray(part.T, float)
                name = f"{parent}/part_{part.name}"
                scene.add_mesh_simple(
                    name, np.asarray(part.mesh.vertices, np.float32) / 1000, np.asarray(part.mesh.faces, np.uint32),
                    color=theme.rgb(getattr(part, "color", None) or theme.MUTED), opacity=PART_ALPHA,
                    position=T[:3, 3] / 1000, wxyz=tf.SO3.from_matrix(T[:3, :3]).wxyz,
                )
                self._part_nodes.append(name)

    def pick(self, x_mm: float, y_mm: float, h_mm: float) -> None:
        """작업대 클릭이나 손잡이로 고른 점을 남긴다. Viser 스레드에서 불리고, 화면 타이머가 picked로 읽는다."""
        self.picked = (self.picked[0] + 1, x_mm, y_mm, h_mm)

    def _on_click(self, event) -> None:
        hit = table_hit(event.ray_origin, event.ray_direction)
        if hit is not None:
            self.pick(hit[0], hit[1], DEFAULT_PICK_H_MM)

    def _on_drag(self, _event) -> None:
        x, y, z = np.asarray(self._handle.position) * 1000
        self.pick(float(x), float(y), max(float(z), 0.0))

    def show_target(self, x_mm: float, y_mm: float, h_mm: float, ok: bool, tool_path=None) -> None:
        """고른 점(닿으면 강조색, 안 닿으면 위험색)과 손끝 경로(점선)를 그린다. tool_path는 preview_path의 tool 목록이다."""
        scene = self.server.scene
        position = (x_mm / 1000, y_mm / 1000, h_mm / 1000)
        with self.server.atomic():
            scene.add_icosphere(
                "/marks/target", radius=0.008, position=position, color=theme.rgb(theme.ACCENT if ok else theme.DANGER)
            )
            self._handle.position = position
            scene.remove_by_name("/marks/path")
            if tool_path:
                points = np.array([(p["x_mm"], p["y_mm"], p["z_mm"] - self._table_z) for p in tool_path]) / 1000
                dashes = np.stack([points[:-1:2], points[1::2][: len(points[:-1:2])]], axis=1)
                if len(dashes):
                    scene.add_line_segments(
                        "/marks/path", dashes, theme.rgb(theme.PREVIEW), thickness=3.0, thickness_units="screen"
                    )

    def show_trajectory(self, segments) -> None:
        """기록의 손끝 궤적을 구간 색으로 그린다. segments는 (라벨, [(x, y, z) 작업대 mm, z는 모델 z]) 목록이다."""
        scene = self.server.scene
        with self.server.atomic():
            scene.remove_by_name("/marks/trajectory")
            scene.add_frame("/marks/trajectory", show_axes=False)
            for i, (label, points) in enumerate(segments):
                if len(points) < 2:
                    continue
                xyz = (np.array(points, float) - (0.0, 0.0, self._table_z)) / 1000
                scene.add_line_segments(
                    f"/marks/trajectory/{i}", np.stack([xyz[:-1], xyz[1:]], axis=1), theme.rgb(theme.SEGMENT_LINE[label]),
                    thickness=3.0, thickness_units="screen",
                )

    def show_allowed(self, box: dict | None) -> None:
        """허용 상자를 반투명으로 그린다. box는 x, y, z의 [최소, 최대] mm이고 z 최대가 None이면 ALLOWED_TOP_MM까지 그린다. None이면 지운다."""
        scene = self.server.scene
        scene.remove_by_name("/allowed")
        if box is None:
            return
        (x0, x1), (y0, y1) = box["x"], box["y"]
        z0, z1 = box["z"][0] or 0.0, box["z"][1] if box["z"][1] is not None else ALLOWED_TOP_MM
        scene.add_box(
            "/allowed", color=theme.rgb(theme.ACCENT), opacity=0.12,
            dimensions=((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000),
            position=((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000),
        )

    def show_reach(self, ranges: dict | None) -> None:
        """REQ-037 두 범위를 팔 앞쪽 반원 호로 그린다. 집기 범위(작업대 면, 초록)와 고른 손끝 각도의 높이별 움직임 범위(강조색).
        ranges는 ArmService.reach_ranges 결과이고 None이면 지운다. 반경은 팔 원점(어깨 회전축) 기준이다."""
        scene = self.server.scene
        scene.remove_by_name("/marks/reach")
        if ranges is None:
            return
        base = self._kin.base
        angles = np.radians(np.linspace(-90.0, 90.0, 37))

        def arc(r_mm: float, z_mm: float) -> np.ndarray:
            pts = np.array([(*robot_to_table(r_mm * math.cos(a), r_mm * math.sin(a), base), z_mm) for a in angles]) / 1000
            return np.stack([pts[:-1], pts[1:]], axis=1)

        pick = ranges["pick"]
        pick_arcs = np.concatenate([arc(pick["min"], 1.0), arc(pick["max"], 1.0)])  # reach_mm 키 {min, max}
        motion = [arc(r, h["height_mm"]) for h in ranges["motion"] for seg in h["segments"] for r in seg]
        with self.server.atomic():
            scene.add_line_segments("/marks/reach/pick", pick_arcs, theme.rgb(theme.OK), thickness=2.0, thickness_units="screen")
            if motion:
                scene.add_line_segments("/marks/reach/motion", np.concatenate(motion), theme.rgb(theme.ACCENT), thickness=1.5, thickness_units="screen")

    def clear_target(self) -> None:
        scene = self.server.scene
        with self.server.atomic():
            scene.remove_by_name("/marks/target")
            scene.remove_by_name("/marks/path")
            scene.remove_by_name("/marks/trajectory")
            scene.remove_by_name("/allowed")
            scene.remove_by_name("/marks/reach")
            self.set_preview(None)

    def _watch_camera(self, client) -> None:
        def moved(camera) -> None:
            self._camera_ready.add(client.client_id)  # 첫 소식 전에는 Viser가 카메라 값을 읽지 못하게 한다
            if not self._is_late_echo(client.client_id, camera.position):
                self.camera = (camera.position, camera.look_at, camera.up_direction)

        client.camera.on_update(moved)

    def _is_late_echo(self, client_id: int, position) -> bool:
        """우리가 보낸 시점이 브라우저를 돌아 늦게 온 값인가. 그대로 쓰면 큐브가 지나온 길을 다시 돈다. 가려진 탭은 1초씩 늦기도 한다."""
        if self._flying:
            return True
        settling = self._settling.get(client_id)
        if settling is None:
            return False
        if np.linalg.norm(np.asarray(position) - settling[0]) < 1e-3 or time.monotonic() > settling[1]:
            del self._settling[client_id]
            return False
        return True

    def _clients(self):
        """카메라 값을 받은 3D 칸들."""
        return [c for c in self.server.get_clients().values() if c.client_id in self._camera_ready]

    def _aim(self, client, position, look_at, up) -> None:
        with client.atomic():
            client.camera.up_direction = up
            client.camera.position = position
            client.camera.look_at = look_at
        self.camera = self._sent[client.client_id] = (position, look_at, up)

    def _fly(self, goal_for) -> None:
        """열려 있는 3D 칸마다 goal_for(카메라)의 시점으로 부드럽게 옮긴다. 블로킹하지 않는다."""
        self._flight += 1
        paths = []
        for client in self._clients():
            camera = client.camera
            start = (camera.position, camera.look_at, camera.up_direction)
            if self._flying or client.client_id in self._settling:  # 브라우저 값은 늦다. 이어서 돌 때 뒤로 튀지 않게 보낸 값에서 시작한다
                start = self._sent.get(client.client_id, start)
            paths.append((client, *view_path(start, goal_for(camera))))
        if paths:
            self._flying = True
            self._flight_thread = threading.Thread(target=self._run_flight, args=(self._flight, paths), daemon=True)
            self._flight_thread.start()

    def _run_flight(self, flight: int, paths: list) -> None:
        begin = time.monotonic()
        try:
            while flight == self._flight:
                elapsed = time.monotonic() - begin
                for client, duration, at in paths:
                    self._aim(client, *at(min(elapsed / duration, 1.0)))
                if elapsed >= max(duration for _, duration, _ in paths):
                    self._settling = {c.client_id: (np.asarray(at(1.0)[0], float), time.monotonic() + VIEW_SETTLE_S) for c, _, at in paths}
                    return
                time.sleep(1 / VIEW_HZ)
        finally:
            if flight == self._flight:
                self._flying = False

    def look_from(self, direction) -> None:
        """그 방향에서 보는 점을 본다(거리는 그대로). 열려 있는 3D 칸 모두에 적용된다."""
        unit = np.asarray(direction, float) / np.linalg.norm(direction)

        def goal(camera) -> tuple:
            look_at = np.asarray(camera.look_at)
            return look_at + unit * np.linalg.norm(np.asarray(camera.position) - look_at), look_at, up_for(direction)

        self._fly(goal)

    def orbit(self, dx_px: float, dy_px: float) -> None:
        """뷰큐브를 끈 만큼 보는 점 둘레로 돈다. 끌기는 손을 바로 따라가므로 진행 중인 전환을 멈춘다."""
        self._flight += 1
        self._flying = False
        self._settling = {}
        for client in self._clients():
            look_at = np.asarray(client.camera.look_at)
            offset = orbit_offset(np.asarray(client.camera.position) - look_at, client.camera.up_direction, dx_px, dy_px)
            self._aim(client, look_at + offset, look_at, Z_UP)

    def home(self) -> None:
        self._fly(lambda _camera: self._home)

    def close(self) -> None:
        self._flight += 1
        self.server.stop()
