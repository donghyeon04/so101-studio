"""화면 배치 점검(결정 0054): 페이지마다 두 화면 크기에서 조작 요소가 겹치지 않고, 가로로 넘치지 않고, 알림이 떠 있지 않고, 3D 위 Viser 창이 접혀 있다.
그리고 2026-11-11에 사람이 찾은 버그 셋(조이스틱 겹침, 조그 패널을 열면 Viser 창 펼침, 3D 점을 끌 때 알림 쌓임)을 다시 보지 않는다."""

import pytest
from visual_helpers import PAGES, SIZES, layout, open_page, viser_panel_open

pytestmark = pytest.mark.slow


@pytest.mark.parametrize("size", list(SIZES))
@pytest.mark.parametrize("path", PAGES)
def test_page_layout_is_sane(page, server, errors, path, size):
    open_page(page, server, path, size)
    out = layout(page)
    assert out["overlaps"] == [], f"{path} {size}: 겹치는 조작 요소 {out['overlaps']}"
    assert out["hscroll"] <= 1, f"{path} {size}: 가로로 {out['hscroll']}px 넘침"
    assert out["toasts"] == 0, f"{path} {size}: 열자마자 알림 {out['toasts']}개"
    assert not viser_panel_open(page), f"{path} {size}: 3D 위 Viser 설정 창이 펼쳐져 있다"
    assert errors == [], errors


@pytest.mark.parametrize("size", list(SIZES))
def test_jog_panel_opens_without_breaking_3d_or_the_sticks(page, server, errors, size):
    open_page(page, server, "/manual", size)
    page.get_by_role("button", name="조그 패널").click()
    page.wait_for_timeout(1500)
    out = layout(page)
    assert out["pads"] == [1, 1], f"조이스틱마다 손잡이 원은 하나여야 한다: {out['pads']}"
    assert out["overlaps"] == [] and out["hscroll"] <= 1, out
    assert not viser_panel_open(page), "조그 패널을 열자 3D 위 Viser 설정 창이 펼쳐졌다"
    pad = page.locator(".jog-pad").first.bounding_box()
    cx, cy = pad["x"] + pad["width"] / 2, pad["y"] + pad["height"] / 2
    page.mouse.move(cx, cy)
    page.mouse.down()
    page.mouse.move(cx + 30, cy - 20, steps=8)  # 끄는 동안(사용자 스크린샷의 상태)
    during = layout(page)
    page.mouse.up()
    assert during["pads"] == [1, 1], f"끄는 동안 조이스틱 원이 늘었다: {during['pads']}"
    assert errors == [], errors


def test_picking_points_in_3d_does_not_pile_up_toasts(page, server, errors):
    """3D 작업대를 여러 번 클릭해 점을 바꿔도(손잡이 끌기와 같은 길: scene.pick) 알림이 쌓이지 않는다."""
    open_page(page, server, "/manual")
    box = page.locator('iframe[title="3D 보기"]').bounding_box()
    for k in range(6):
        page.mouse.click(box["x"] + box["width"] * (0.35 + 0.05 * k), box["y"] + box["height"] * 0.7)
        page.wait_for_timeout(250)
    page.wait_for_timeout(800)
    assert layout(page)["toasts"] <= 1, "점을 옮길 때마다 알림이 쌓인다"
    assert errors == [], errors
