"""SO-101 Studio GUI (gui-spec). 의존성이 없으면 import에서 실패해 `so101 gui`가 설치를 안내한다."""

from .app import main

__all__ = ["main"]
