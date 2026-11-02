"""GUI의 색과 크기 (gui-spec 5절). 색과 크기는 이 파일에만 둔다."""

from nicegui import ui

BG = "#f3f4f6"
CARD = "#ffffff"
LINE = "#d5dae0"  # 장식용 구분선. 입력칸 외곽선에는 대비(3:1)가 모자라 쓰지 않는다
TEXT = "#1b1f24"
MUTED = "#5b6470"
ACCENT = "#2457d6"
OK = "#1f6b3a"
WARN = "#8a4b00"
DANGER = "#c62828"
SENSOR = "#0b7285"
PREVIEW = "#6d4fc2"

STATE_COLORS = {"대기": "grey-8", "실행 중": "primary", "정지": "negative", "오류": "negative"}

CONTROL_H = 44
BODY_PX = 15
RADIUS = 10
GAP = 16
DRAWER_W = 240
ESTOP_D = 88

_FONTS = (
    '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?'
    "family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans+KR:wght@400;600&display=swap\">"
)
_CSS = f"""
body {{ font-family: 'IBM Plex Sans KR', 'Malgun Gothic', system-ui, sans-serif; font-size: {BODY_PX}px; color: {TEXT}; }}
.q-btn {{ min-height: {CONTROL_H}px; }}
.q-card {{ border-radius: {RADIUS}px; box-shadow: none; border: 1px solid {LINE}; }}
.q-drawer {{ background: {CARD}; border-right: 1px solid {LINE}; }}
.frame-bar {{ background: {CARD}; color: {TEXT}; }}
.frame-top {{ border-bottom: 1px solid {LINE}; }}
.frame-bottom {{ border-top: 1px solid {LINE}; }}
.stat-label {{ color: {MUTED}; font-size: 13px; line-height: 16px; }}
.stat-value {{ font-family: 'IBM Plex Mono', 'IBM Plex Sans KR', Consolas, 'Malgun Gothic', monospace; font-weight: 600; line-height: 20px; white-space: pre; }}
.page-state {{ font-size: {BODY_PX}px; padding: 4px 10px; }}
.nav-group {{ color: {MUTED}; font-size: 13px; font-weight: 600; padding: 8px {GAP}px 2px; }}
.nav-item {{ width: 100%; color: {TEXT}; font-size: {BODY_PX}px; }}
.nav-item.nav-active {{ color: {ACCENT}; background: {BG}; font-weight: 600; box-shadow: inset 3px 0 0 {ACCENT}; }}
.page-title {{ font-size: 24px; font-weight: 600; line-height: 32px; }}
.page-body {{ color: {MUTED}; }}
.estop {{ width: {ESTOP_D}px; height: {ESTOP_D}px; font-weight: 600; border: 3px solid {CARD}; outline: 2px solid {DANGER}; }}
.nicegui-content {{ padding: {GAP}px; gap: {GAP}px; padding-bottom: {ESTOP_D + 2 * GAP}px; }}
"""


def apply() -> None:
    """현재 페이지에 테마를 입힌다."""
    ui.colors(primary=ACCENT, positive=OK, negative=DANGER, warning=WARN, info=SENSOR, preview=PREVIEW)
    ui.add_head_html(_FONTS)  # 오프라인이면 _CSS의 시스템 글꼴을 쓴다
    ui.add_css(_CSS)
    ui.query("body").style(f"background: {BG}")
