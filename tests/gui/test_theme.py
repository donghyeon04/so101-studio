"""theme.py의 색 대비가 WCAG AA를 넘는지 검사한다. 프로젝터에서는 더 낮아지므로 기준 아래로 내려가면 안 된다."""

import pytest

pytest.importorskip("nicegui")

from so101_studio.gui import theme


def _luminance(color: str) -> float:
    channels = [int(color[i : i + 2], 16) / 255 for i in (1, 3, 5)]
    r, g, b = [c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _contrast(a: str, b: str) -> float:
    hi, lo = sorted((_luminance(a), _luminance(b)), reverse=True)
    return (hi + 0.05) / (lo + 0.05)


def test_contrast_helper_matches_known_ratio():
    assert _contrast("#000000", "#ffffff") == pytest.approx(21)


@pytest.mark.parametrize("name", ["TEXT", "MUTED", "ACCENT", "OK", "WARN", "DANGER", "SENSOR", "PREVIEW"])
@pytest.mark.parametrize("background", ["BG", "CARD"])
def test_text_colors_are_readable(name, background):
    assert _contrast(getattr(theme, name), getattr(theme, background)) >= 4.5


@pytest.mark.parametrize("name", ["ACCENT", "DANGER"])
def test_white_label_on_filled_button_is_readable(name):
    assert _contrast("#ffffff", getattr(theme, name)) >= 4.5


@pytest.mark.parametrize("background", ["BG", "CARD"])
def test_control_border_meets_non_text_contrast(background):
    assert _contrast(theme.CONTROL_BORDER, getattr(theme, background)) >= 3


def test_watch_states_differ_by_icon_not_only_color():
    icons = [icon for icon, _ in theme.WATCH_STYLES.values()]
    assert len(set(icons)) == len(icons)


def test_control_height_meets_touch_size():
    assert theme.CONTROL_H >= 44
