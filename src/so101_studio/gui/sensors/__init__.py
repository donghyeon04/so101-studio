"""센서 화면 등록부 (gui-spec 7절). 인식 방식마다 자기 페이지와 작업 공간용 작은 화면을 등록한다.
page_fn이 None이면 메뉴에 자리만 있다. task는 그 자리를 채우는 작업이다."""

from collections.abc import Callable
from dataclasses import dataclass


@dataclass
class SensorView:
    name: str
    title: str
    path: str
    page_fn: Callable | None
    mini_view_fn: Callable | None
    task: str = ""


SENSORS: dict[str, SensorView] = {}


def register(name: str, title: str, path: str, page_fn=None, mini_view_fn=None, task: str = "") -> None:
    SENSORS[name] = SensorView(name, title, path, page_fn, mini_view_fn, task)


def menu() -> tuple:
    """사이드바 인식 그룹: (경로, 제목, 채우는 작업)."""
    return tuple((s.path, s.title, s.task) for s in SENSORS.values())


def pages() -> dict:
    """만들어진 센서 페이지: {경로: 페이지 함수}."""
    return {s.path: s.page_fn for s in SENSORS.values() if s.page_fn}


def mini_views() -> list[SensorView]:
    return [s for s in SENSORS.values() if s.mini_view_fn]


def pending_titles() -> list[str]:
    return [s.title for s in SENSORS.values() if not s.page_fn]


# 메뉴 순서는 gui-spec 3절을 따른다
register("ultrasonic", "초음파", "/ultrasonic", task="3단계")
register("camera", "카메라", "/camera", task="4단계")
from . import manual_view, ultrasonic_view

__all__ = ["SENSORS", "SensorView", "manual_view", "menu", "mini_views", "pages", "pending_titles", "register", "ultrasonic_view"]
