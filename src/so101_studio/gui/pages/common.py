"""페이지들이 같이 쓰는 조각: 조작권 게이트, 3D 칸."""

from nicegui import ui


class Controls:
    """조작권이 있어야 켜지는 요소를 모으고, 눌렸을 때 서버에서 한 번 더 막는다."""

    def __init__(self, backend, client_id: str, block_during_task: bool = True):
        self.backend, self.client_id, self.block_during_task = backend, client_id, block_during_task
        self.elements: list = []

    def add(self, element):
        self.elements.append(element)
        return element

    @property
    def mine(self) -> bool:
        return self.backend.owner == self.client_id

    @property
    def allowed(self) -> bool:
        """작업 실행 중에는 팔을 직접 움직이는 조작을 막는다."""
        return self.mine and not (self.block_during_task and self.backend.task_running())

    def guard(self, fn):
        async def call(*args):
            if not self.mine:
                ui.notify("보기 전용입니다", type="warning")
                return None
            if not self.allowed:
                ui.notify("작업 실행 중입니다", type="warning")
                return None
            return await fn(*args)

        return call

    def refresh(self) -> None:
        allowed = self.allowed
        for element in self.elements:
            element.enabled = allowed


def scene_frame(backend, classes: str = "scene3d") -> ui.element:
    return ui.element("iframe").props(f'src={backend.scene.url} title="3D 보기"').classes(classes).mark("scene3d")
