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
CUBE_SIZE = 56  # 뷰큐브 한 변(px)
CUBE_BOX = 150  # 돌려도 좌표축 글자까지 잘리지 않는 칸
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
# 구성 요소 상태: (아이콘, 색, 글자). 색 + 아이콘 + 글자 (decisions/0025, 0030)
COMPONENT_STYLES = {
    "ok": ("check_circle", "positive", "정상"),
    "warn": ("warning", "warning", "주의"),
    "error": ("error", "negative", "오류"),
    "stale": ("sync_problem", "grey-8", "끊김"),
}
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
ESTOP_D = 64  # 아래 조작 막대 안에서 가장 큰 버튼 (결정 0044)
JOG_PANEL_W = 300

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
.page-desc {{ color: {MUTED}; margin-top: -4px; }}
.jog-pad {{ width: 120px; height: 120px; position: relative; border-radius: 50%; background: {BG}; border: 1px solid {CONTROL_BORDER}; }}
.jog-pad > div {{ position: absolute; inset: 0; }}  /* 조이스틱 라이브러리가 안쪽 요소에 'joystick' 클래스를 써서 같은 이름을 피한다 */
.nudge-pad {{ width: max-content; margin: 0 auto; }}
.nudge-button {{ width: 40px; height: 40px; }}
.top-stats {{ scrollbar-width: thin; }}
.stat-label {{ color: {MUTED}; font-size: 13px; line-height: 16px; white-space: nowrap; }}
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
.run-bands {{ height: 150px; }}
.log-link {{ cursor: pointer; }}
.log-link:hover {{ text-decoration: underline; }}
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
.meta-input {{ width: 112px; flex: none; }}
.meta-number {{ width: 100%; }}
.meta-soft-out .meta-hint {{ color: {WARN}; }}
.meta-soft-out .meta-input .q-field__control:before {{ border-color: {WARN}; border-width: 2px; }}
.meta-soft-band .q-slider__track {{ background: linear-gradient(to right, transparent var(--soft-a), rgba{(*rgb(ACCENT), 0.2)} var(--soft-a), rgba{(*rgb(ACCENT), 0.2)} var(--soft-b), transparent var(--soft-b)), rgba(0,0,0,0.1); }}
.meta-gap {{ min-width: 34px; }}
.progress-step {{ text-align: center; line-height: {CONTROL_H}px; background: {CARD}; color: {MUTED}; border: 1px solid {LINE}; border-radius: 6px; }}
.progress-step.progress-active {{ background: {ACCENT}; color: #ffffff; font-weight: 600; border-color: {ACCENT}; }}
.event-line {{ font-size: 14px; line-height: 22px; }}
.joint-name {{ width: 72px; flex: none; font-size: 14px; }}
.joint-value {{ width: 56px; flex: none; text-align: right; font-size: 14px; }}
.plane {{ border-radius: 6px; }}
.plane-large {{ max-width: 84vh; }}
.scene3d.scene3d-short {{ height: 44vh; min-height: 280px; }}
.scene3d.scene3d-manual {{ height: calc(100vh - 360px); min-height: 300px; }}  /* 직접 조작: 3D 밑의 현재 좌표·[이동] 줄이 스크롤 없이 보이게 (11-14) */
.joint-chart {{ height: 30vh; min-height: 220px; }}
.segment-bar {{ gap: 2px; }}
.segment {{ min-width: 0; height: {CONTROL_H}px; line-height: {CONTROL_H}px; text-align: center; overflow: hidden; border-radius: 4px; font-size: 13px; }}
.control-column {{ width: 380px; flex: none; }}
.operator-title {{ font-size: 28px; font-weight: 600; line-height: 36px; }}
.operator-card-title {{ font-size: 22px; font-weight: 600; }}
.operator-body {{ font-size: 20px; line-height: 30px; }}
.operator-stage {{ width: 340px; flex: none; }}
.operator-chip {{ font-size: 22px; line-height: 48px; padding: 0 16px; background: {CARD}; color: {MUTED}; border: 1px solid {LINE}; border-radius: 8px; }}
.operator-chip.operator-chip-active {{ background: {ACCENT}; color: #ffffff; font-weight: 600; border-color: {ACCENT}; }}
.operator-button {{ font-size: 20px; min-height: 56px; padding: 0 28px; }}
.operator-now {{ font-size: 32px; font-weight: 600; line-height: 40px; }}
.scene-column {{ position: sticky; top: {GAP}px; }}
@media (min-width: 700px) {{
  .scene3d {{ min-width: 600px; }}  /* Viser는 3D 칸이 약 576px보다 좁으면 좁은 화면 배치로 바뀌어 설정 창을 펼친다(조그 패널을 열 때). 폰은 보기 전용이라 둔다 */
  .nicegui-content .no-wrap:has(.scene3d) {{ flex-wrap: wrap; }}  /* 자리가 모자라면 오른쪽 조작 칸이 3D 아래로 내려간다 */
}}
.jog-button {{ min-width: {CONTROL_H}px; padding: 0 8px; }}
.jog-unit {{ width: 24px; }}
.scene3d {{ width: 100%; height: 62vh; min-height: 360px; border: 1px solid {LINE}; border-radius: {RADIUS}px; background: {CARD}; }}
.scene-wrap {{ position: relative; width: 100%; }}
.scene-wrap iframe {{ display: block; }}
.viewcube {{ position: absolute; top: -28px; right: 36px; width: {CUBE_BOX}px; height: {CUBE_BOX}px; pointer-events: none; user-select: none; touch-action: none; }}
.vc-cube {{ position: absolute; left: calc(50% - {CUBE_SIZE / 2}px); top: calc(50% - {CUBE_SIZE / 2}px); width: {CUBE_SIZE}px; height: {CUBE_SIZE}px; transform-style: preserve-3d; }}
.vc-face {{ position: absolute; inset: 0; display: grid; grid-template: 22% 56% 22% / 22% 56% 22%; background: #e9ebef; border: 1px solid {CONTROL_BORDER}; backface-visibility: hidden; }}
.vc-cell {{ pointer-events: auto; cursor: pointer; }}
.vc-cell:focus-visible {{ outline-offset: -2px; }}
.vc-cell.hot {{ background: {ACCENT}73; }}
.vc-label {{ position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; font-size: 12px; font-weight: 600; color: {TEXT}; pointer-events: none; }}
.vc-triad {{ position: absolute; inset: 0; pointer-events: none; }}
.vc-home {{ position: absolute; left: 0; bottom: 18px; pointer-events: auto; color: {MUTED}; }}
.scene-layers {{ position: absolute; top: 8px; left: 8px; }}
.q-page-sticky {{ z-index: 5000; }}
.program-step {{ border-radius: 6px; }}
.program-step-selected {{ background: {LINE}; outline: 2px solid {ACCENT}; }}
.estop {{ width: {ESTOP_D}px; height: {ESTOP_D}px; font-weight: 600; border: 3px solid {CARD}; outline: 2px solid {DANGER}; }}
.nicegui-content {{ padding: {GAP}px; gap: {GAP}px; }}
.q-field--outlined .q-field__control:before {{ border-color: {CONTROL_BORDER}; }}
.q-field--outlined.q-field--focused .q-field__control:after {{ border-color: {ACCENT}; border-width: 2px; }}
:focus-visible {{ outline: 2px solid {ACCENT}; outline-offset: 2px; }}
"""


SWIPE_DISMISS_JS = """
<script>
// 알림(Quasar Notify)을 아래로 끌면 따라 내려가며 흐려지고, 60px 넘게 끌고 놓으면 사라진다. 덜 끌면 제자리로 돌아온다.
// Vue가 관리하는 요소라 지우지 않고 숨긴다(시간이 되면 Quasar가 지운다, 시간이 없는 알림은 숨긴 채로 남는다)
(() => {
  let drag = null;
  addEventListener('pointerdown', (e) => {
    const el = e.target.closest?.('.q-notification');
    if (!el || e.target.closest('button')) return;
    drag = {el, y: e.clientY};
    el.style.transition = 'none';
  });
  addEventListener('pointermove', (e) => {
    if (!drag) return;
    const dy = Math.max(0, e.clientY - drag.y);
    drag.el.style.transform = `translateY(${dy}px)`;
    drag.el.style.opacity = String(Math.max(0.2, 1 - dy / 120));
  });
  addEventListener('pointerup', (e) => {
    if (!drag) return;
    const {el} = drag, dy = e.clientY - drag.y;
    drag = null;
    el.style.transition = 'transform .2s, opacity .2s';
    if (dy > 60) {
      el.style.transform = 'translateY(80px)'; el.style.opacity = '0';
      setTimeout(() => { el.style.display = 'none'; }, 200);
    } else {
      el.style.transform = ''; el.style.opacity = '';
    }
  });
})();
</script>
"""


def apply() -> None:
    """현재 페이지에 테마를 입힌다."""
    ui.colors(primary=ACCENT, positive=OK, negative=DANGER, warning=WARN, info=SENSOR, preview=PREVIEW)
    ui.add_head_html(_FONTS)  # 오프라인이면 _CSS의 시스템 글꼴을 쓴다
    ui.add_head_html(SWIPE_DISMISS_JS)
    ui.add_css(_CSS)
    ui.query("body").style(f"background: {BG}")
