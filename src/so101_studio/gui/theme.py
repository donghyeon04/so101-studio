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
# 관절 그래프 계열. dataviz 기본 범주 팔레트의 앞 6색(validate_palette 통과). 상태 색과 겹치지 않는다
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
# 기록 구간: 한 색의 농도로 빠르기를 나타낸다. 글자를 함께 쓴다
SEGMENT_COLORS = {"멈춤": LINE, "느림": "#9db7ee", "빠름": ACCENT}
SEGMENT_INK = {"멈춤": TEXT, "느림": TEXT, "빠름": "#ffffff"}
SEGMENT_LINE = {"멈춤": CONTROL_BORDER, "느림": "#5f8ae4", "빠름": ACCENT}  # 3D 궤적. 밝은 작업대 위에서 보이게 진하다
# 점검 결과: (아이콘, 색, 글자). 색 + 아이콘 + 글자 (decisions/0025)
DOCTOR_STYLES = {
    "ok": ("check_circle", "positive", "정상"),
    "warn": ("warning", "warning", "주의"),
    "fail": ("error", "negative", "실패"),
    "skip": ("remove_circle_outline", "grey-8", "건너뜀"),
}
REACH_STYLES = {True: ("check_circle", "positive"), False: ("block", "negative")}



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
.card-title {{ font-weight: 600; }}
.diag-output {{ min-height: 120px; }}
.run-list {{ width: 280px; flex: none; }}
.run-chart {{ height: 24vh; min-height: 180px; }}
.run-events {{ height: 30vh; }}
.log-area {{ height: 66vh; background: {CARD}; border: 1px solid {LINE}; border-radius: {RADIUS}px; }}
.log-line {{ font-family: 'IBM Plex Mono', 'IBM Plex Sans KR', Consolas, monospace; font-size: 14px; line-height: 22px; white-space: pre-wrap; }}
.log-warning {{ color: {WARN}; font-weight: 600; }}
.log-error {{ color: {DANGER}; font-weight: 600; }}
.axis-label {{ width: 36px; flex: none; }}
.stat-danger {{ background: {DANGER}; color: #ffffff; padding: 0 6px; border-radius: 4px; }}
.param-row {{ padding: 2px 6px; border-radius: 4px; }}
.param-row.param-changed {{ background: #fff4d6; box-shadow: inset 3px 0 0 {WARN}; }}
.param-input {{ width: 112px; flex: none; }}
.progress-step {{ text-align: center; line-height: {CONTROL_H}px; background: {CARD}; color: {MUTED}; border: 1px solid {LINE}; border-radius: 6px; }}
.progress-step.progress-active {{ background: {ACCENT}; color: #ffffff; font-weight: 600; border-color: {ACCENT}; }}
.event-line {{ font-size: 14px; line-height: 22px; }}
.joint-name {{ width: 72px; flex: none; font-size: 14px; }}
.joint-value {{ width: 56px; flex: none; text-align: right; font-size: 14px; }}
.plane {{ border-radius: 6px; }}
.plane-large {{ max-width: 84vh; }}
.scene3d.scene3d-short {{ height: 44vh; min-height: 280px; }}
.joint-chart {{ height: 30vh; min-height: 220px; }}
.segment-bar {{ gap: 2px; }}
.segment {{ min-width: 0; height: {CONTROL_H}px; line-height: {CONTROL_H}px; text-align: center; overflow: hidden; border-radius: 4px; font-size: 13px; }}
.control-column {{ width: 380px; flex: none; }}
.scene-column {{ position: sticky; top: {GAP}px; }}
.jog-button {{ min-width: {CONTROL_H}px; padding: 0 8px; }}
.jog-unit {{ width: 24px; }}
.scene3d {{ width: 100%; height: 62vh; min-height: 360px; border: 1px solid {LINE}; border-radius: {RADIUS}px; background: {CARD}; }}
.q-page-sticky {{ z-index: 5000; }}
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
