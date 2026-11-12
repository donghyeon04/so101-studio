"""GUI 화면 시험(결정 0054) 공용 상수·함수. 시험 파일이 conftest를 불러올 수 없어 여기에 둔다."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WEB, VISER, WATCHDOG = 8380, 8381, 8490
BASE = f"http://127.0.0.1:{WEB}"
SIZES = {"wide": {"width": 1440, "height": 900}, "pane": {"width": 820, "height": 720}}  # 넓은 화면, 앱 안 브라우저 크기
PAGES = ("/start", "/", "/manual", "/teach", "/program", "/ultrasonic", "/input", "/arm", "/calibration", "/esp32", "/params", "/safety",
         "/table", "/runs", "/logs", "/operator")


def open_page(page, server: str, path: str, size: str = "wide") -> None:
    page.set_viewport_size(SIZES[size])
    page.goto(server + path, wait_until="load")  # networkidle은 오지 않는다: heartbeat가 0.2초마다 감시 프로세스에 간다
    page.wait_for_selector(".nicegui-content", timeout=20000)
    page.wait_for_timeout(2000)  # 0.1~0.5초 타이머가 몇 번 돌고 3D iframe이 붙게


LAYOUT_JS = """
() => {
  const vis = e => { const r = e.getBoundingClientRect(); const s = getComputedStyle(e);
    return r.width > 2 && r.height > 2 && s.visibility !== 'hidden' && s.display !== 'none' && r.bottom > 0 && r.right > 0 && r.top < innerHeight && r.left < innerWidth; };
  const els = [...document.querySelectorAll('.q-btn, input, .q-checkbox, .q-toggle, .q-slider, .jog-pad, .q-select, .q-tab')].filter(vis);
  const top = e => els.filter(o => o !== e && o.contains(e)).length === 0;
  const layer = e => { for (let n = e; n; n = n.parentElement) { const p = getComputedStyle(n).position; if (p === 'fixed' || p === 'sticky') return n; } return null; };
  // 고정 막대·패널 밑으로 지나가는 요소는 사용자에게 안 보이니 뺀다: 가운데 점에서 맨 위에 있는 요소가 자기(또는 그 안)일 때만 센다
  const onTop = e => { const r = e.getBoundingClientRect(); const h = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2); return h && (e === h || e.contains(h) || h.contains(e)); };
  const items = els.filter(top).filter(onTop).map(e => ({e, r: e.getBoundingClientRect()}));
  const overlaps = [];
  for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
    const a = items[i], b = items[j];
    if (a.e.contains(b.e) || b.e.contains(a.e)) continue;
    if (layer(a.e) !== layer(b.e)) continue;  // 스크롤되는 칸이 고정 막대·패널 밑을 지나는 겹침은 스크롤하면 풀린다. 같은 층 안의 겹침만 센다
    const w = Math.min(a.r.right, b.r.right) - Math.max(a.r.left, b.r.left), h = Math.min(a.r.bottom, b.r.bottom) - Math.max(a.r.top, b.r.top);
    if (w > 4 && h > 4) overlaps.push([(a.e.innerText || a.e.className).slice(0, 30), (b.e.innerText || b.e.className).slice(0, 30), Math.round(w), Math.round(h)]);
  }
  const pads = [...document.querySelectorAll('.jog-pad')].map(p => p.querySelectorAll('.back').length);
  return {overlaps, hscroll: document.scrollingElement.scrollWidth - innerWidth, toasts: document.querySelectorAll('.q-notification').length, pads};
}
"""


def layout(page) -> dict:
    """겹치는 조작 요소, 가로 스크롤 넘침(px), 떠 있는 알림 수, 조이스틱 받침마다 손잡이 원 수."""
    return page.evaluate(LAYOUT_JS)


def viser_panel_open(page) -> bool:
    """3D iframe 안 Viser 설정 창이 펼쳐져 있는가('Save Canvas' 버튼이 보이면 펼쳐진 것)."""
    frame = page.frame_locator('iframe[title="3D 보기"]')
    try:
        return frame.get_by_text("Save Canvas").first.is_visible(timeout=1500)
    except Exception:  # noqa: BLE001  3D가 없는 페이지
        return False
