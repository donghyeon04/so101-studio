"""무작위 조작 시험(결정 0054): gremlins.js 2.2.0(MIT, vendor/)이 페이지마다 클릭·입력·스크롤을 정해진 씨앗으로 퍼붓는다.
그 뒤에도 페이지 스크립트 오류가 없고, 알림이 쌓이지 않고, 서버가 살아 있어야 한다.
레포 설정을 건드리지 않게 config/ 사본 폴더에서 띄운 서버(scratch_server)를 쓴다. 왼쪽 메뉴·머리 막대는 누르지 않는다(페이지를 벗어나면 gremlins가 끊긴다)."""

import urllib.request
from pathlib import Path

import pytest
from visual_helpers import layout, open_page

pytestmark = pytest.mark.slow

GREMLINS = (Path(__file__).parent / "vendor" / "gremlins.min.js").read_text(encoding="utf-8")
PAGES = ("/", "/manual", "/teach", "/program", "/input", "/params", "/safety", "/table")

UNLEASH_JS = """
async (seed) => {
  const inside = el => el.closest('.q-drawer, .q-header, iframe, a[href]');
  const horde = gremlins.createHorde({
    randomizer: new gremlins.Chance(seed),
    species: [
      gremlins.species.clicker({canClick: el => !inside(el)}),
      gremlins.species.toucher({canTouch: el => !inside(el)}),
      gremlins.species.formFiller({canFillElement: el => !inside(el)}),
      gremlins.species.scroller(),
    ],
    mogwais: [gremlins.mogwais.alert()],  // gizmo(오류 10개면 멈춤)는 뺀다: 오류는 Playwright가 모은다
    strategies: [gremlins.strategies.distribution({nb: 250, delay: 40})],
  });
  await horde.unleash();
}
"""


@pytest.mark.parametrize("path", PAGES)
def test_random_clicks_do_not_break_the_page(page, scratch_server, errors, path):
    page.add_init_script(GREMLINS)
    open_page(page, scratch_server, path)
    page.evaluate(UNLEASH_JS, 1234)
    page.wait_for_timeout(1500)
    assert errors == [], f"{path}: 페이지 스크립트 오류 {errors[:3]}"
    assert layout(page)["toasts"] <= 5, f"{path}: 알림이 쌓인다"
    with urllib.request.urlopen(scratch_server + "/start", timeout=5) as r:
        assert r.status == 200, "서버가 응답하지 않는다"
