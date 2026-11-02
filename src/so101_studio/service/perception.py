"""인식 방식 선택·스캔 (system-architecture.md 7절, REQ-002). Studio.perception으로 쓴다."""


class PerceptionManager:
    def __init__(self, studio, source):
        self._studio = studio
        self._sources = {source.name: source}

    def register(self, source) -> None:
        """선택할 수 있는 인식 방식을 더한다 (초음파·카메라 모듈이 생기면 여기에 등록)."""
        self._sources[source.name] = source

    @property
    def active(self) -> str:
        return self._studio.source.name

    def list(self) -> list[str]:
        return list(self._sources)

    def select(self, name: str) -> None:
        if name not in self._sources:
            raise ValueError(f"알 수 없는 인식 방식: {name} (가능: {self.list()})")
        th = self._studio._thread
        if th is not None and th.is_alive():
            raise RuntimeError("작업이 실행 중이라 인식 방식을 바꿀 수 없다")
        self._studio.source = self._studio.task.source = self._sources[name]

    def calibrate(self) -> dict:
        return self._studio.source.calibrate()

    def scan(self):
        s = self._studio.source
        s.connect()
        return s.scan()
