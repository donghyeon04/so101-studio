"""직접 조작 페이지 (gui-spec 3절): 3D, 조그, 좌표 이동(미리보기 -> 이동), 그리퍼, 속도, 홈·파크."""

from nicegui import run, ui

from .. import theme
from ..backend import (
    ANGLE_AXES,
    CART_AXES,
    EDGE_TEXT,
    JOG_JOINTS,
    JOINT_TEXT,
    MOTION_TEXT,
)
from .common import (
    HEIGHT_HINT,
    Controls,
    jog_pads,
    meta_number,
    offer_free_edge,
    scene_frame,
)

STEPS = {0: "누르기", 1: "1", 5: "5", 10: "10"}
DEFAULT_TARGET = {"x": 300.0, "y": 200.0, "h": 40.0, "yaw": 90.0, "pitch": -90.0}
UNDO_MAX = 50  # 되돌리기로 기억하는 목표 좌표 수


def build(backend, client_id: str, actions) -> None:
    gate = Controls(backend, client_id, moves=True)
    control, guarded, owner = gate.add, gate.guard, lambda: gate.mine
    held: list = []  # 지금 누르고 있는 조그 (mode, index, sign)
    plan_state = {"dirty": True, "busy": False, "plan": None, "seq": backend.scene.picked[0]}
    target = dict(DEFAULT_TARGET)

    async def act(fn, *args) -> None:
        result = await run.io_bound(fn, *args)
        if result == EDGE_TEXT:
            offer_free_edge(backend, lambda: act(fn, *args))
        elif result:  # 실패 이유만 돌아온다
            ui.notify(result, type="negative")

    with actions:
        ui.button("정지", color="negative", on_click=lambda: run.io_bound(backend.stop_all)).props("outline no-caps").mark("stop")
        control(ui.button("홈", on_click=guarded(lambda: act(backend.home))).props("outline no-caps").mark("home"))
        control(ui.button("파크", on_click=guarded(lambda: act(backend.park))).props("outline no-caps").mark("park"))

    def mark_dirty() -> None:
        plan_state["dirty"] = True

    history: list[dict] = []  # 목표 좌표의 이전 값들 (Ctrl+Z·[되돌리기]). 팔은 움직이지 않는다

    def remember() -> None:
        if not history or history[-1] != target:
            history.append(dict(target))
            del history[:-UNDO_MAX]

    def undo() -> None:
        if not history:
            ui.notify("되돌릴 좌표가 없습니다", type="info")
            return
        for key, value in history.pop().items():  # 입력칸에 조용히 넣고(다시 기록하지 않게) 목표를 바꾼다. 3D 점과 계산이 따라온다
            fields[key].set(value, quiet=True)
            target[key] = value
        mark_dirty()

    def number(label: str, key: str):
        """좌표 칸 (gui-spec 12.7, target.*): 허용 값을 칸을 떠날 때·Enter·간격 버튼에서 받는다."""
        def changed(v) -> None:
            remember()
            target[key] = float(v)
            mark_dirty()

        ui.label(label).classes("stat-label")
        return meta_number(f"target.{key}", target[key], changed, label=label, mark=f"target-{key}", add=control)

    def joint_limits(i: int) -> tuple:
        """관절 각도로 칸의 한계: 고른 관절의 실제 가동 범위(없으면 설정 한계) (12.7.1 joint.deg)."""
        name = JOG_JOINTS[i]
        lim = backend.get_safety().get("physical_limits_deg") or {}
        return tuple(lim.get(name) or backend.arm.cfg.joint_limits_deg[name])

    async def move() -> None:
        plan = plan_state["plan"]
        if plan is None or not plan.ok:
            return
        move_button.disable()
        planned = plan_state.get("planned")
        if planned != target:  # 이동은 늘 미리보기(계산)한 점으로: 그 사이 다른 기기가 3D에서 점을 바꿨을 수 있다(gui-spec 12.6)
            plan_state["dirty"] = True
            ui.notify("점이 바뀌었습니다. 다시 계산한 뒤 [이동]을 누르세요", type="warning")
            return
        reason = await run.io_bound(backend.move_to, planned["x"], planned["y"], planned["h"], planned["yaw"], planned["pitch"])
        if reason == EDGE_TEXT:
            offer_free_edge(backend, move)
        elif reason:
            ui.notify(reason, type="negative")
        plan_state["dirty"] = True

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2 scene-column"):
            scene_frame(backend, classes="scene3d scene3d-manual")
            # 3D 칸은 따라 내려온다(sticky): 현재 좌표와 [이동]을 여기 두어 입력칸을 내려도 스크롤 없이 누른다 (사용자 요청 11-14)
            move_click: list = [lambda: None]
            # 폭 고정(w-full): 값 길이가 바뀌어도 스냅숏 가림 상자가 같다
            current = ui.label("현재 좌표  -").classes("page-body current-pose w-full").mark("current-pose")
            with ui.row().classes("w-full items-center no-wrap gap-2"):
                reach_icon = ui.icon("help", size="24px").mark("reach-icon")
                reach = ui.label("계산 중").classes("flex-1 reach-text").mark("reach")  # 예상 시간은 팔 자세에 따라 바뀐다(화면 시험은 가린다)
            edge_button = ui.button("작업대 가장자리 검사 끄기", on_click=lambda: offer_free_edge(backend, mark_dirty)).props(
                "outline no-caps dense color=warning").mark("edge-offer")
            edge_button.visible = False
            with ui.row().classes("w-full no-wrap gap-2"):
                move_button = ui.button("이동", on_click=lambda: move_click[0]()).props("no-caps").classes("flex-1").mark("move")
                ui.button("되돌리기", icon="undo", on_click=lambda: undo()).props("outline no-caps").tooltip(
                    "목표 좌표를 바로 전 값으로 되돌린다(Ctrl+Z). 팔은 움직이지 않는다").mark("target-undo")
                ui.button("점 처음 자리로", icon="restart_alt", on_click=lambda: reset_target()).props("outline no-caps").tooltip(
                    f"점을 x {DEFAULT_TARGET['x']:.0f}, y {DEFAULT_TARGET['y']:.0f}, 높이 {DEFAULT_TARGET['h']:.0f}, 아래 방향으로 되돌린다(팔은 움직이지 않는다)").mark("target-reset")
            ui.label("작업대를 클릭하거나 손잡이를 끌어 점을 고릅니다. 초록 호 = 집기 범위, 파란 호 = 고른 손끝 각도로 갈 수 있는 높이별 범위").classes("page-body")
        ui.keyboard(on_key=lambda e: undo() if e.action.keydown and not e.action.repeat and e.modifiers.ctrl and str(e.key).lower() == "z" else None)  # 입력칸 안의 Ctrl+Z는 브라우저가 그 칸을 되돌린다

        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("좌표 이동").classes("card-title")
                fields = {"x": number("x (mm)", "x"), "y": number("y (mm)", "y"), "h": number("높이 (mm)", "h"), "yaw": number("yaw (°)", "yaw"),
                          "pitch": number("손끝 각도 (°) — −90 아래 · 0 수평", "pitch")}
                coords = list(fields.values())
                move_click[0] = guarded(move, needs=coords)  # [이동]은 입력칸보다 먼저(3D 칸 밑에) 만들어서, 누를 때 칸 검사를 여기서 묶는다

                def reset_target() -> None:  # 3D 손잡이를 실수로 멀리 끌었을 때. 입력칸을 바꾸면 계산과 3D 점이 따라온다
                    remember()
                    for key, value in DEFAULT_TARGET.items():
                        fields[key].set(value, quiet=True)
                        target[key] = value
                    mark_dirty()

            with ui.card().classes("w-full gap-2"):
                ui.label("조그").classes("card-title")
                ui.label("간격 (° 또는 mm)").classes("stat-label")
                step = control(ui.toggle(STEPS, value=0).props("no-caps spread").classes("w-full").mark("jog-step"))
                with ui.tabs().classes("w-full").props("dense no-caps") as tabs:
                    joint_tab = ui.tab("관절")
                    cart_tab = ui.tab("직교")
                with ui.tab_panels(tabs, value=joint_tab).classes("w-full"):
                    with ui.tab_panel(joint_tab).classes("p-0 gap-1"):
                        for i, name in enumerate(JOG_JOINTS):
                            _jog_row(backend, JOINT_TEXT[name], ("joint", i), "°", held, step, control, guarded)
                        with ui.row().classes("w-full no-wrap items-center gap-1 mt-2"):
                            target_joint = control(ui.select({i: JOINT_TEXT[n] for i, n in enumerate(JOG_JOINTS)}, value=0)
                                                   .props("outlined dense").classes("flex-1").mark("joint-to-name"))
                            target_joint.on_value_change(lambda e: target_deg.set_hard(*joint_limits(int(e.value))))
                        target_deg = meta_number("joint.deg", 0.0, label="관절 각도", mark="joint-to-deg", hard=joint_limits(0), add=control)
                        with ui.row().classes("w-full no-wrap items-center gap-1"):

                            async def joint_to() -> None:
                                if target_deg.value is None:
                                    return
                                reason = await run.io_bound(backend.joint_to, int(target_joint.value), float(target_deg.value))
                                if reason:
                                    ui.notify(reason, type="negative")

                            control(ui.button("관절 이동", on_click=guarded(joint_to, needs=[target_deg])).props("outline no-caps").mark("joint-to"), needs=[target_deg])
                    with ui.tab_panel(cart_tab).classes("p-0 gap-1"):
                        jog_pads(backend, gate)
                        for i, name in enumerate(CART_AXES):
                            unit = "°" if name in ANGLE_AXES else "mm"
                            _jog_row(backend, name, ("cartesian", i), unit, held, step, control, guarded)

            with ui.card().classes("w-full gap-2"):
                ui.label("그리퍼").classes("card-title")
                meta_number("gripper.value", round(status.joints.gripper) if (status := backend.state.status) else 0,
                            guarded(lambda v: act(backend.gripper, float(v))), label="그리퍼 값", mark="grip-value", add=control)
                ui.label("폭 (mm)").classes("stat-label")
                width = meta_number("gripper.width_mm", 50.0, label="그리퍼 폭", mark="grip-width-mm", add=control)
                control(ui.button("폭으로", on_click=guarded(lambda: act(backend.gripper_width, float(width.value)), needs=[width]))
                        .props("outline no-caps").mark("grip-width"), needs=[width])

            with ui.card().classes("w-full gap-2").mark("pick-place"):
                ui.label("집어 옮기기").classes("card-title")
                points: dict = {"pick": None, "place": None}
                pick_text = ui.label("집기 점: 없음").classes("page-body").mark("pp-pick")
                place_text = ui.label("놓기 점: 배출 구역").classes("page-body").mark("pp-place")

                def set_point(kind: str) -> None:
                    points[kind] = (target["x"], target["y"])
                    (pick_text if kind == "pick" else place_text).text = f'{"집기" if kind == "pick" else "놓기"} 점: x {target["x"]:.0f}, y {target["y"]:.0f}'

                with ui.row().classes("w-full no-wrap gap-2"):
                    control(ui.button("지금 좌표를 집기 점으로", on_click=guarded(lambda: set_point("pick"), needs=coords))
                            .props("outline no-caps dense").mark("pp-set-pick"), needs=coords)
                    control(ui.button("놓기 점으로", on_click=guarded(lambda: set_point("place"), needs=coords))
                            .props("outline no-caps dense").mark("pp-set-place"), needs=coords)
                objs = {}
                for key, text, value in (("width", "폭 (mm, 0 = 모름)", 30.0), ("length", "길이 (mm, 0 = 모름)", 30.0), ("angle", "각도 (°)", 0.0),
                                         ("height", "높이 (mm, 0 = 모름)", 0.0)):
                    ui.label(text).classes("stat-label")
                    objs[key] = meta_number(f"obj.{key}", value, label=f"물체 {text.split(' ')[0]}", mark="pp-height" if key == "height" else f"pp-{key}", add=control)
                obj_w, obj_l, obj_a, obj_h = objs["width"], objs["length"], objs["angle"], objs["height"]
                obj_h.number.tooltip(HEIGHT_HINT)
                obj_list = list(objs.values())

                async def pick_place() -> None:
                    if points["pick"] is None:
                        ui.notify("먼저 집기 점을 고르세요(3D를 클릭하거나 좌표를 넣고 [지금 좌표를 집기 점으로])", type="warning")
                        return
                    reason = await run.io_bound(backend.pick_place, points["pick"], float(obj_a.value), float(obj_w.value),
                                                float(obj_l.value), points["place"], float(obj_h.value))
                    obj_h.set_error(reason if reason == MOTION_TEXT["height_required"] else None)  # 높이 칸 강조 (REQ-057)
                    if reason == EDGE_TEXT:
                        offer_free_edge(backend, pick_place)
                    else:
                        ui.notify(reason or "집어 옮겼습니다", type="negative" if reason else "positive")

                control(ui.button("집어 옮기기", on_click=guarded(pick_place, needs=obj_list)).props("no-caps").classes("w-full").mark("pp-run"), needs=obj_list)

            with ui.card().classes("w-full gap-2"):
                ui.label("속도 (%)").classes("card-title")
                meta_number("speed_pct", backend.arm.speed, guarded(lambda v: run.io_bound(backend.set_speed, float(v))), label="속도", mark="speed", add=control)

    async def refresh_plan() -> None:
        seq, x, y, h = backend.scene.picked
        if seq != plan_state["seq"]:  # 3D에서 고른 점을 입력칸에 옮긴다. 알림은 띄우지 않는다: 손잡이를 끄는 동안 수십 번 쌓여 화면을 가린다(입력칸과 닿는지 줄이 바로 바뀐다)
            plan_state["seq"] = seq
            point = {"x": round(x, 1), "y": round(y, 1), "h": round(h, 1)}
            inside = all(fields[k]._ok(v) for k, v in point.items())
            if inside:  # 셋 다 넣거나 하나도 넣지 않는다 (12.7.4)
                remember()  # 3D에서 끌어 옮긴 점도 되돌릴 수 있게
                for key, value in point.items():
                    fields[key].set(value, quiet=True)
                    target[key] = value
                plan_state["dirty"] = True
            elif not plan_state.get("out_warned"):
                ui.notify("점이 허용 범위 밖입니다", type="warning")
            plan_state["out_warned"] = not inside
        arm_state = backend.state.status.state if backend.state.status else None
        if arm_state == "idle" and plan_state.get("arm") != "idle":  # 팔이 움직였으면 경로와 시간이 달라진다
            plan_state["dirty"] = True
        plan_state["arm"] = arm_state
        # 3D의 점과 계산 자세는 하나뿐이라 조작권을 가진 브라우저만 계산한다
        if owner() and plan_state["dirty"] and not plan_state["busy"] and backend.state.status and backend.state.status.state == "idle":
            plan_state["dirty"], plan_state["busy"] = False, True
            planned = dict(target)
            try:
                plan_state["plan"] = await run.io_bound(backend.plan, planned["x"], planned["y"], planned["h"], planned["yaw"], planned["pitch"])
                plan_state["planned"] = planned
            finally:
                plan_state["busy"] = False
            plan = plan_state["plan"]
            reach.text = plan.text
            icon, color = theme.REACH_STYLES[plan.ok]
            reach_icon.name = icon
            reach_icon.props(f"color={color}")
            if not plan.ok and plan.text != plan_state.get("warned"):  # 손잡이를 끄는 동안 같은 이유가 수십 번 쌓이지 않게 바뀔 때만 알린다
                ui.notify(plan.text, type="warning")
            plan_state["warned"] = None if plan.ok else plan.text
            edge_button.visible = plan.reason == "workspace"  # 작업대 가장자리 때문에만 안 닿을 때 그 자리에서 끌 수 있게
        elif not owner() and plan_state["plan"] is None:
            reach.text = "보기 전용이라 계산하지 않습니다"
        if (pose := backend.tip_now()) is not None:
            current.text = "현재 좌표  x {x:.0f} · y {y:.0f} · 높이 {h:.0f} mm · yaw {yaw:.0f}° · 손끝 각도 {pitch:.0f}°".format(**pose)
        mine = gate.allowed
        gate.refresh()
        status = backend.state.status
        idle = status is not None and status.state == "idle"
        plan = plan_state["plan"]
        move_button.enabled = bool(mine and idle and plan and plan.ok and not plan_state["dirty"] and gate.valid(coords))
        if held:  # 데드맨: 이 호출이 끊기면 팔이 스스로 멈춘다
            backend.jog(*held[0])

    ui.timer(0.1, refresh_plan)


def _jog_row(backend, label, axis, unit, held, step, control, guarded) -> None:
    mode, index = axis

    def press(sign: int):
        async def pressed() -> None:
            if step.value:  # 간격 이동은 클릭에서 처리한다
                return
            held[:] = [(mode, index, sign)]
            backend.jog(mode, index, sign)

        return guarded(pressed)

    def release() -> None:
        if held:
            held.clear()
            backend.jog_release(mode)

    def click(sign: int):
        async def clicked() -> None:
            if not step.value:
                return
            reason = await run.io_bound(backend.jog_step, mode, index, sign * step.value)
            if reason:
                ui.notify(reason, type="negative")

        return guarded(clicked)

    with ui.row().classes("w-full no-wrap items-center gap-1"):
        ui.label(label).classes("flex-1")
        for sign, text in ((-1, "−"), (1, "+")):
            button = control(
                ui.button(text, on_click=click(sign)).props(f'outline no-caps aria-label="{label} {text}"')
                .classes("jog-button").mark(f"jog-{mode}-{index}-{sign}")
            )
            button.on("mousedown", press(sign))
            button.on("touchstart", press(sign))
            for event in ("mouseup", "mouseleave", "touchend", "touchcancel"):
                button.on(event, release)
        ui.label(unit).classes("stat-label jog-unit")
