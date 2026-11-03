"""3D 칸 오른쪽 위의 뷰큐브 (Fusion 360의 ViewCube와 같은 모양과 동작). 면·모서리·꼭짓점을 누르면 그쪽에서 보고, 끌면 돌고, 집 단추는 처음 시점으로 간다.
3D는 다른 포트의 iframe이라 안쪽을 건드릴 수 없어 NiceGUI 쪽에 CSS 3D로 얹는다. 큐브의 좌표는 장면 좌표(x 오른쪽, y 뒤, z 위)와 같다."""

from itertools import product

import numpy as np
from nicegui import ui

from . import theme

# 글자: (글자의 오른쪽, 글자의 아래쪽, 바깥쪽). CSS 좌표는 왼손계라 이 순서의 행렬식이 -1이어야 글자가 뒤집히지 않는다
FACES = {
    "앞": ((1, 0, 0), (0, 0, -1), (0, -1, 0)),
    "뒤": ((-1, 0, 0), (0, 0, -1), (0, 1, 0)),
    "오른쪽": ((0, 1, 0), (0, 0, -1), (1, 0, 0)),
    "왼쪽": ((0, -1, 0), (0, 0, -1), (-1, 0, 0)),
    "위": ((1, 0, 0), (0, -1, 0), (0, 0, 1)),
    "아래": ((1, 0, 0), (0, 1, 0), (0, 0, -1)),
}
AXES = (("X", (1, 0, 0), theme.DANGER), ("Y", (0, 1, 0), theme.OK), ("Z", (0, 0, 1), theme.ACCENT))
AXIS_LEN = 1.45  # 큐브 한 변의 배수. 좌표축이 큐브 밖으로 나와 보이는 길이
CUBE_SYNC_S = 1 / 30  # 시점 전환 때 큐브가 끊겨 보이지 않을 만큼. 값이 그대로면 아무것도 보내지 않는다
DIRECTIONS = {d for d in product((-1, 0, 1), repeat=3) if any(d)}  # 면 6 + 모서리 12 + 꼭짓점 8

VIEW_CUBE_JS = """
<script>
// 눌렀다 떼면 cubepick, 끌면 cubedrag를 .viewcube에 보낸다. 포인터를 잡아 두어 3D iframe 위로 나가도 끌기가 이어진다
(() => {
  let down = null;
  const cellOf = (e) => e.target.closest?.('.vc-cell');
  const hot = (root, dir) => root.querySelectorAll('.vc-cell').forEach((c) => c.classList.toggle('hot', c.dataset.dir === dir));
  const send = (root, name, detail) => root.dispatchEvent(new CustomEvent(name, {detail}));
  const flush = () => {
    if (down && (down.dx || down.dy)) {
      send(down.root, 'cubedrag', {dx: down.dx, dy: down.dy});
      down.dx = down.dy = 0;
    }
  };
  addEventListener('pointerover', (e) => { const c = cellOf(e); if (c && !down) hot(c.closest('.viewcube'), c.dataset.dir); });
  addEventListener('pointerout', (e) => { const c = cellOf(e); if (c && !down) hot(c.closest('.viewcube'), null); });
  addEventListener('pointerdown', (e) => {
    const c = cellOf(e);
    if (!c) return;
    e.preventDefault();
    c.setPointerCapture(e.pointerId);
    down = {root: c.closest('.viewcube'), dir: c.dataset.dir, x: e.clientX, y: e.clientY, moved: false, dx: 0, dy: 0, timer: setInterval(flush, 40)};
  });
  addEventListener('pointermove', (e) => {
    if (!down) return;
    if (Math.abs(e.clientX - down.x) + Math.abs(e.clientY - down.y) > 3) down.moved = true;
    if (down.moved) {
      down.dx += e.movementX;
      down.dy += e.movementY;
    }
  });
  const up = () => {
    if (!down) return;
    clearInterval(down.timer);
    flush();
    if (!down.moved) send(down.root, 'cubepick', {dir: down.dir});
    hot(down.root, null);
    down = null;
  };
  addEventListener('pointerup', up);
  addEventListener('pointercancel', up);
  addEventListener('keydown', (e) => {
    const c = cellOf(e);
    if (c && (e.key === 'Enter' || e.key === ' ')) {
      e.preventDefault();
      send(c.closest('.viewcube'), 'cubepick', {dir: c.dataset.dir});
    }
  });
})();
</script>
"""


def _matrix(columns, translate=(0, 0, 0)) -> str:
    """CSS matrix3d. 세 축이 어디로 가는지(columns)와 이동을 받는다. 값은 열 순서로 쓴다."""
    values = [*columns[0], 0, *columns[1], 0, *columns[2], 0, *translate, 1]
    return "matrix3d(" + ",".join(f"{float(v):.4f}" for v in values) + ")"


def faces_html() -> str:
    """큐브 면 6개. 면마다 3x3 칸으로 나눠 가운데는 면, 변은 모서리, 귀퉁이는 꼭짓점 방향을 맡는다. 글자는 고정 문구다."""
    half = theme.CUBE_SIZE / 2
    parts = []
    for label, (right, down, out) in FACES.items():
        cells = []
        for j, i in product((-1, 0, 1), repeat=2):
            direction = ",".join(str(out[k] + i * right[k] + j * down[k]) for k in range(3))
            center = i == 0 and j == 0  # 키보드로는 면만 고른다. 칸 54개를 다 거치면 쓸 수 없다
            extra = f' tabindex="0" role="button" aria-label="{label}에서 보기"' if center else ""
            cells.append(f'<div class="vc-cell" data-dir="{direction}"{extra}></div>')
        transform = _matrix((right, down, out), tuple(v * half for v in out))
        parts.append(f'<div class="vc-face" style="transform:{transform}">{"".join(cells)}<span class="vc-label">{label}</span></div>')
    return "".join(parts)


def camera_axes(position, look_at, up) -> tuple | None:
    """카메라의 (오른쪽, 위, 앞). 위쪽 축을 곧장 내려다보면 정할 수 없어 None."""
    forward = np.asarray(look_at, float) - np.asarray(position, float)
    right = np.cross(forward, np.asarray(up, float))
    if np.linalg.norm(forward) < 1e-9 or np.linalg.norm(right) < 1e-9:
        return None
    forward, right = forward / np.linalg.norm(forward), right / np.linalg.norm(right)
    return right, np.cross(right, forward), forward


def cube_matrix(axes) -> str:
    """장면 좌표를 화면 좌표(x 오른쪽, y 아래, z 앞으로 나옴)로 돌리는 CSS 행렬."""
    right, top, forward = axes
    return _matrix([(right[k], -top[k], -forward[k]) for k in range(3)])


def triad_svg(axes) -> str:
    """큐브의 앞-왼쪽-아래 꼭짓점에서 나오는 좌표축. 큐브와 같은 시점으로 2D에 투영한다. 글자는 고정 문구다."""
    right, top, _ = axes
    size, center = theme.CUBE_SIZE, theme.CUBE_BOX / 2

    def screen(point) -> tuple[float, float]:
        return center + float(right @ point), center - float(top @ point)

    corner = np.array((-1.0, -1.0, -1.0)) * size / 2
    x0, y0 = screen(corner)
    parts = []
    for name, axis, color in AXES:
        x1, y1 = screen(corner + np.array(axis) * size * AXIS_LEN)
        parts.append(
            f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="{color}" stroke-width="2"/>'
            f'<text x="{x1:.1f}" y="{y1:.1f}" dy="4" dx="{3 if x1 >= x0 else -11}" fill="{color}" font-size="12" font-weight="700">{name}</text>'
        )
    return f'<svg width="{theme.CUBE_BOX}" height="{theme.CUBE_BOX}">{"".join(parts)}</svg>'


def parse_direction(text) -> tuple | None:
    """브라우저가 보낸 방향 글자를 확인한다. 큐브 칸이 낼 수 있는 26방향이 아니면 None."""
    try:
        direction = tuple(int(v) for v in str(text).split(","))
    except ValueError:
        return None
    return direction if direction in DIRECTIONS else None


def view_cube(scene) -> None:
    """지금 놓는 칸의 오른쪽 위에 뷰큐브를 만든다. 부모는 position: relative여야 한다."""

    def picked(event) -> None:
        direction = parse_direction(event.args["detail"]["dir"])
        if direction:
            scene.look_from(direction)

    def dragged(event) -> None:
        detail = event.args["detail"]
        scene.orbit(float(detail["dx"]), float(detail["dy"]))

    with ui.element("div").classes("viewcube").mark("viewcube") as root:
        cube = ui.html(faces_html(), sanitize=False).classes("vc-cube").mark("viewcube-cube")
        triad = ui.html("", sanitize=False).classes("vc-triad")
        ui.button(icon="home", on_click=scene.home).props('flat round dense size=sm aria-label="처음 시점으로"').classes("vc-home").mark("viewcube-home")
    root.on("cubepick", picked, ["detail"]).on("cubedrag", dragged, ["detail"])
    shown = [None]

    def sync() -> None:
        camera = scene.camera
        if camera is shown[0]:
            return
        shown[0] = camera
        axes = camera_axes(*camera)
        if axes:
            cube.style(f"transform: {cube_matrix(axes)}")
            triad.set_content(triad_svg(axes))

    sync()
    ui.timer(CUBE_SYNC_S, sync)
