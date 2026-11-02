"""GUI의 색과 크기 (gui-spec 5절). 색과 크기는 이 파일에만 둔다."""

from nicegui import ui

BG = "#f3f4f6"
CARD = "#ffffff"
LINE = "#d5dae0"  # 카드 테두리·구분선 같은 장식에만
CONTROL_BORDER = "#7d8794"  # 입력칸 외곽선 (WCAG 1.4.11의 3:1)
TEXT = "#1b1f24"
MUTED = "#5b6470"
ACCENT = "#2457d6"
OK = "#1f6b3a"
WARN = "#8a4b00"
DANGER = "#c62828"
SENSOR = "#0b7285"
PREVIEW = "#6d4fc2"

STATE_COLORS = {"대기": "grey-8", "실행 중": "primary", "정지": "negative", "오류": "negative"}
# 감시 상태: (아이콘, 색). 색 + 아이콘 + 글자 (decisions/0025)
WATCH_STYLES = {
    "정상": ("check_circle", "positive"),
    "꺼짐": ("error", "negative"),
    "작동함": ("report", "negative"),
    "없음": ("remove_circle_outline", "warning"),
}



def rgb(color: str) -> tuple[int, int, int]:
    """'#rrggbb'를 Viser가 받는 (r, g, b)로."""
    return tuple(int(color[i : i + 2], 16) for i in (1, 3, 5))


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
.scene3d {{ width: 100%; height: 62vh; min-height: 360px; border: 1px solid {LINE}; border-radius: {RADIUS}px; background: {CARD}; }}
.estop {{ width: {ESTOP_D}px; height: {ESTOP_D}px; font-weight: 600; border: 3px solid {CARD}; outline: 2px solid {DANGER}; }}
.nicegui-content {{ padding: {GAP}px; gap: {GAP}px; padding-bottom: {ESTOP_D + 2 * GAP}px; }}
.q-field--outlined .q-field__control:before {{ border-color: {CONTROL_BORDER}; }}
.q-field--outlined.q-field--focused .q-field__control:after {{ border-color: {ACCENT}; border-width: 2px; }}
:focus-visible {{ outline: 2px solid {ACCENT}; outline-offset: 2px; }}
"""


def apply() -> None:
    """현재 페이지에 테마를 입힌다."""
    ui.colors(primary=ACCENT, positive=OK, negative=DANGER, warning=WARN, info=SENSOR, preview=PREVIEW)
    ui.add_head_html(_FONTS)  # 오프라인이면 _CSS의 시스템 글꼴을 쓴다
    ui.add_css(_CSS)
    ui.query("body").style(f"background: {BG}")
