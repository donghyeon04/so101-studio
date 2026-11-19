"""시작 화면 (gui-spec 11절 A1, G12): 모드 카드, 처음이면 할 순서, 페이지 안내, 최근 실행 기록."""

from nicegui import ui

from ...core.ports import is_usb_serial, list_serial_ports

FIRST_STEPS = (
    ("/manual", "1. 직접 조작에서 팔 움직여 보기"),
    ("/teach", "2. 교시에서 기록 재생하기"),
    ("/", "3. 작업 공간에서 수동 입력 물체로 자동 실행"),
)


def mode_text(backend, ports: list[dict]) -> tuple[str, str]:
    """(큰 글자, 설명). 시뮬레이션으로 켰는데 보드가 꽂혀 있으면 실물로 켜는 방법을 알린다."""
    if backend.real is None:
        if any(is_usb_serial(p) for p in ports):
            return "시뮬레이션", "포트에 팔 보드가 꽂혀 있습니다. 실물로 쓰려면 앱을 닫고 \"SO-101 Studio\"로 다시 켜세요(팔이 보이면 실물로 켜집니다)."
        return "시뮬레이션", "팔 없이 화면 속 팔로 움직입니다. 실물 팔은 USB를 꽂고 전원을 켠 뒤 \"SO-101 Studio\"로 다시 켜면 연결됩니다."
    lock = backend.move_lock()
    if lock:
        return "실물", f"실물 팔에 연결했지만 움직이는 버튼이 잠겨 있습니다: {lock}"
    return "실물", "실물 팔에 연결됨. 처음 움직일 때는 속도 30% 이하로, 전원 스위치 옆에서 지켜보세요."


def build(backend, client_id: str, actions, pages, descriptions) -> None:
    big, detail = mode_text(backend, list_serial_ports())
    with ui.card().classes("w-full gap-1").mark("start-mode"):
        ui.label(big).classes("text-2xl font-semibold").mark("start-mode-title")
        ui.label(detail).classes("page-body mode-detail w-full")  # 보드가 꽂혔는지에 따라 바뀐다(화면 시험은 가린다)

    ui.label("처음이면 이 순서로").classes("card-title")
    with ui.row().classes("w-full gap-2"):
        for path, text in FIRST_STEPS:
            ui.button(text, on_click=lambda p=path: ui.navigate.to(p)).props("outline no-caps").mark(f"start-step-{path}")

    with ui.row().classes("w-full items-center gap-2"):
        ui.link("운영자 화면 (시연용, 새 탭)", "/operator", new_tab=True).classes("font-semibold").mark("start-operator")
        ui.label("메뉴 없이 3D·지금 단계·물체·실행 버튼만 크게 보입니다").classes("page-body")

    ui.label("페이지 안내").classes("card-title")
    with ui.element("div").classes("w-full grid gap-3").style("grid-template-columns: repeat(auto-fill, minmax(260px, 1fr))"):
        for group, items in pages:
            with ui.card().classes("gap-1"):
                ui.label(group).classes("card-title")
                for path, title, _ in items:
                    ui.link(title, path).classes("font-semibold")
                    ui.label(descriptions.get(path, "")).classes("page-body page-desc")

    ui.label("최근 실행 기록").classes("card-title")
    names = backend.run_names()[:3]
    with ui.row().classes("w-full gap-2").mark("start-runs"):
        if not names:
            ui.label("아직 없습니다").classes("page-body")
        for name in names:
            ui.link(name, "/runs").classes("page-body")
