"""직접 조작 페이지 (gui-spec 3절): 3D, 조그, 좌표 이동(미리보기 -> 이동), 그리퍼, 속도, 홈·파크."""

from nicegui import run, ui

from .. import theme
from ..backend import ANGLE_AXES, CART_AXES, EDGE_TEXT, JOG_JOINTS, JOINT_TEXT
from .common import Controls, jog_pads, offer_free_edge, scene_frame

STEPS = {0: "누르기", 1: "1", 5: "5", 10: "10"}
DEFAULT_TARGET = {"x": 300.0, "y": 200.0, "h": 40.0, "yaw": 90.0, "pitch": -90.0}


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

    def number(label: str, key: str, step: float) -> ui.number:
        def changed(e) -> None:
            if e.value is not None:
                target[key] = float(e.value)
                mark_dirty()

        return control(
            ui.number(label, value=target[key], step=step, on_change=changed).props("outlined dense").classes("flex-1").mark(f"target-{key}")
        )

    async def move() -> None:
        plan = plan_state["plan"]
        if plan is None or not plan.ok:
            return
        move_button.disable()
        reason = await run.io_bound(backend.move_to, target["x"], target["y"], target["h"], target["yaw"], target["pitch"])
        if reason == EDGE_TEXT:
            offer_free_edge(backend, move)
        elif reason:
            ui.notify(reason, type="negative")
        plan_state["dirty"] = True

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("flex-1 min-w-0 gap-2 scene-column"):
            scene_frame(backend)
            ui.label("작업대를 클릭하거나 손잡이를 끌어 점을 고릅니다. 초록 호 = 집기 범위, 파란 호 = 고른 손끝 각도로 갈 수 있는 높이별 범위").classes("page-body")

        with ui.column().classes("control-column gap-3"):
            with ui.card().classes("w-full gap-2"):
                ui.label("좌표 이동").classes("card-title")
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields = {"x": number("x (mm)", "x", 5), "y": number("y (mm)", "y", 5)}
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields |= {"h": number("높이 (mm)", "h", 5), "yaw": number("yaw (°)", "yaw", 15)}
                with ui.row().classes("w-full no-wrap gap-2 items-center"):
                    fields["pitch"] = number("손끝 각도 (°)", "pitch", 15).props("min=-90 max=0").tooltip("-90 = 수직 아래(집기), 0 = 수평")
                    ui.label("−90 아래 · 0 수평").classes("page-body")
                with ui.row().classes("w-full items-center no-wrap gap-2"):
                    reach_icon = ui.icon("help", size="24px").mark("reach-icon")
                    reach = ui.label("계산 중").classes("flex-1").mark("reach")
                edge_button = ui.button("작업대 가장자리 검사 끄기", on_click=lambda: offer_free_edge(backend, mark_dirty)).props(
                    "outline no-caps dense color=warning").mark("edge-offer")
                edge_button.visible = False
                move_button = ui.button("이동", on_click=guarded(move)).props("no-caps").classes("w-full").mark("move")

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
                            target_deg = control(ui.number("각도 (°)", value=0, step=1, format="%.1f").props("outlined dense").classes("w-24").mark("joint-to-deg"))

                            async def joint_to() -> None:
                                if target_deg.value is None:
                                    return
                                reason = await run.io_bound(backend.joint_to, int(target_joint.value), float(target_deg.value))
                                if reason:
                                    ui.notify(reason, type="negative")

                            control(ui.button("이동", on_click=guarded(joint_to)).props("outline no-caps").mark("joint-to"))
                    with ui.tab_panel(cart_tab).classes("p-0 gap-1"):
                        jog_pads(backend, gate)
                        for i, name in enumerate(CART_AXES):
                            unit = "°" if name in ANGLE_AXES else "mm"
                            _jog_row(backend, name, ("cartesian", i), unit, held, step, control, guarded)

            with ui.card().classes("w-full gap-2"):
                ui.label("그리퍼").classes("card-title")
                grip = control(
                    ui.slider(min=0, max=100, step=1, value=round(status.joints.gripper) if (status := backend.state.status) else 0)
                    .props("label-always").classes("mt-6").mark("grip-value")
                )
                grip.on("change", guarded(lambda: act(backend.gripper, float(grip.value))))
                with ui.row().classes("w-full no-wrap items-center gap-2"):
                    width = control(ui.number("폭 (mm)", value=50, min=0, max=111, step=5).props("outlined dense").classes("flex-1"))
                    control(
                        ui.button("폭으로", on_click=guarded(lambda: act(backend.gripper_width, float(width.value or 0))))
                        .props("outline no-caps").mark("grip-width")
                    )

            with ui.card().classes("w-full gap-2").mark("pick-place"):
                ui.label("집어 옮기기").classes("card-title")
                points: dict = {"pick": None, "place": None}
                pick_text = ui.label("집기 점: 없음").classes("page-body").mark("pp-pick")
                place_text = ui.label("놓기 점: 배출 구역").classes("page-body").mark("pp-place")

                def set_point(kind: str) -> None:
                    points[kind] = (target["x"], target["y"])
                    (pick_text if kind == "pick" else place_text).text = f'{"집기" if kind == "pick" else "놓기"} 점: x {target["x"]:.0f}, y {target["y"]:.0f}'

                with ui.row().classes("w-full no-wrap gap-2"):
                    control(ui.button("지금 좌표를 집기 점으로", on_click=lambda: set_point("pick")).props("outline no-caps dense").mark("pp-set-pick"))
                    control(ui.button("놓기 점으로", on_click=lambda: set_point("place")).props("outline no-caps dense").mark("pp-set-place"))
                with ui.row().classes("w-full no-wrap gap-2"):
                    obj_w = control(ui.number("폭 (mm)", value=30, min=0, step=5).props("outlined dense").classes("flex-1"))
                    obj_l = control(ui.number("길이 (mm)", value=30, min=0, step=5).props("outlined dense").classes("flex-1"))
                    obj_a = control(ui.number("각도 (°)", value=0, step=15).props("outlined dense").classes("flex-1"))
                    obj_h = control(ui.number("높이 (mm)", value=0, min=0, step=5).props("outlined dense").classes("flex-1")
                                    .tooltip("0이면 설정한 집는 높이로 집습니다").mark("pp-height"))

                async def pick_place() -> None:
                    if points["pick"] is None:
                        ui.notify("먼저 집기 점을 고르세요(3D를 클릭하거나 좌표를 넣고 [지금 좌표를 집기 점으로])", type="warning")
                        return
                    reason = await run.io_bound(backend.pick_place, points["pick"], float(obj_a.value or 0), float(obj_w.value or 0),
                                                float(obj_l.value or 0), points["place"], float(obj_h.value or 0))
                    if reason == EDGE_TEXT:
                        offer_free_edge(backend, pick_place)
                    else:
                        ui.notify(reason or "집어 옮겼습니다", type="negative" if reason else "positive")

                control(ui.button("집어 옮기기", on_click=guarded(pick_place)).props("no-caps").classes("w-full").mark("pp-run"))

            with ui.card().classes("w-full gap-2"):
                ui.label("속도 (%)").classes("card-title")
                speed = control(ui.slider(min=5, max=100, step=5, value=backend.arm.speed).props("label-always").classes("mt-6").mark("speed"))
                speed.on("change", guarded(lambda: run.io_bound(backend.set_speed, float(speed.value))))

    async def refresh_plan() -> None:
        seq, x, y, h = backend.scene.picked
        if seq != plan_state["seq"]:  # 3D에서 고른 점을 입력칸에 옮긴다
            plan_state["seq"] = seq
            for key, value in (("x", x), ("y", y), ("h", h)):
                target[key] = round(value, 1)
                fields[key].value = target[key]
            plan_state["dirty"] = True
        arm_state = backend.state.status.state if backend.state.status else None
        if arm_state == "idle" and plan_state.get("arm") != "idle":  # 팔이 움직였으면 경로와 시간이 달라진다
            plan_state["dirty"] = True
        plan_state["arm"] = arm_state
        # 3D의 점과 계산 자세는 하나뿐이라 조작권을 가진 브라우저만 계산한다
        if owner() and plan_state["dirty"] and not plan_state["busy"] and backend.state.status and backend.state.status.state == "idle":
            plan_state["dirty"], plan_state["busy"] = False, True
            try:
                plan_state["plan"] = await run.io_bound(backend.plan, target["x"], target["y"], target["h"], target["yaw"], target["pitch"])
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
        mine = gate.allowed
        gate.refresh()
        status = backend.state.status
        idle = status is not None and status.state == "idle"
        plan = plan_state["plan"]
        move_button.enabled = bool(mine and idle and plan and plan.ok and not plan_state["dirty"])
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
