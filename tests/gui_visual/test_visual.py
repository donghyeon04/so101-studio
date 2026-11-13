"""화면 사진 비교(결정 0054): 주요 페이지를 찍어 기준 사진과 비교한다. 3D·상태 숫자·기록 폴더·이벤트 줄처럼 매번 바뀌는 곳은 가린다.
기준 사진을 새로 만들 때: pytest tests/gui_visual/test_visual.py --update-snapshots (바뀐 화면을 눈으로 확인한 뒤)."""

import pytest
from visual_helpers import LAYOUT_JS, open_page

pytestmark = pytest.mark.slow

MASKS = ['iframe[title="3D 보기"]', "div:has(> .stat-value)",  # 값 길이가 바뀌면 가림 상자 크기도 바뀌어 칸 전체를 가린다
          ".event-line", ".q-notification", ".viewcube", ".mode-detail", ".reach-text"]
SHOTS = [("/start", "wide"), ("/", "wide"), ("/manual", "wide"), ("/input", "wide"), ("/calibration", "wide"), ("/operator", "wide"),
         ("/table", "wide"), ("/params", "wide"), ("/safety", "wide"), ("/start", "pane"), ("/manual", "pane")]


@pytest.mark.parametrize(("path", "size"), SHOTS)
def test_page_looks_the_same(page, server, assert_snapshot, path, size):
    open_page(page, server, path, size)
    assert_snapshot(page, name=f"{path.strip('/') or 'workspace'}-{size}.png", mask_elements=MASKS)


def test_layout_check_catches_overlap_and_extra_stick(page):
    """배치 점검이 진짜 겹침과 조이스틱 원 늘어남을 잡는지(판정 자체의 시험, 서버 없이). 두 버튼은 서로의 가운데를 가리지 않게 겹친다."""
    page.set_content("""<div class="nicegui-content">
      <button class="q-btn" style="position:absolute;left:10px;top:10px;width:80px;height:40px">가</button>
      <button class="q-btn" style="position:absolute;left:70px;top:30px;width:80px;height:40px">나</button>
      <div class="jog-pad" style="position:absolute;left:200px;top:10px;width:120px;height:120px"><div class="back"></div><div class="back"></div></div>
    </div>""")
    out = page.evaluate(LAYOUT_JS)
    assert any({"가", "나"} == {a, b} for a, b, *_ in out["overlaps"]) and out["pads"] == [2]
