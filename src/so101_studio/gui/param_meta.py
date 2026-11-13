"""숫자 칸 값 정보표 (gui-spec 12.7.1, 결정 0055): 칸마다 단위·허용 범위·한 칸·추천 범위와 그 근거."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Meta:
    unit: str
    hard: tuple[float, float]  # 받는 한계(양 끝 포함). 밖이면 막는다
    step: float  # 숫자 칸 화살표·휠 한 칸이자 슬라이더 눈금
    soft: tuple[float, float] | None = None  # 추천 범위. 근거(why)가 있을 때만
    why: str = ""
    widget: str | None = None  # 입력 방식 규칙(12.7.2)을 덮어쓸 때만: slider | gap | number
    allow_none: bool = False  # 빈 칸이 뜻을 갖는 값
    none_text: str = ""


def _m(unit, lo, hi, step, soft=None, why="", **kw) -> Meta:
    return Meta(unit, (float(lo), float(hi)), float(step), soft and (float(soft[0]), float(soft[1])), why, **kw)


META: dict[str, Meta] = {
    # 파라미터 (기본 = 설정 파일 값, get_params()의 saved)
    "speed_pct": _m("%", 5, 100, 5, (5, 30), "CLAUDE.md: 실물 첫 동작은 30% 이하"),
    "gripper_speed_pct": _m("%", 5, 100, 5),
    "joint_v_max_deg_s": _m("°/s", 10, 300, 5),
    "a_max_deg_s2": _m("°/s²", 10, 2000, 10),
    "joint_jerk_max_deg_s3": _m("°/s³", 50, 10000, 50),
    "tool_v_max_mm_s": _m("mm/s", 10, 500, 5),
    "tool_a_max_mm_s2": _m("mm/s²", 10, 3000, 10),
    "tool_jerk_max_mm_s3": _m("mm/s³", 50, 20000, 50),
    "servo_acceleration": _m("서보 단위", 1, 254, 1, (10, 25), "결정 0014, 측정 11-01(12는 끝점 오차 15mm)"),
    "gripper_acceleration": _m("서보 단위", 1, 254, 1),
    "control_hz": _m("Hz", 10, 100, 5, (50, 50), "결정 0014"),
    "table_z_mm": _m("mm", -100, 50, 1),
    "tip_floor_mm": _m("mm", 0, 50, 1),
    "approach_height_mm": _m("mm", 10, 150, 5),
    "grasp_height_mm": _m("mm", 0, 100, 1),
    "place_height_mm": _m("mm", 0, 150, 1),
    "gripper_margin_mm": _m("mm", 0, 60, 1),
    "gripper_open": _m("열림 눈금", 10, 100, 5),
    "gripper_squeeze_torque": _m("토크 눈금", 50, 1000, 10),
    "gripper_hold_torque": _m("토크 눈금", 50, 1000, 10),
    "grip_preload_units": _m("서보 단위", 0, 20, 1),
    "gripper_load_warn_pct": _m("%", 10, 100, 5),
    "gripper_load_s": _m("초", 0.2, 10, 0.1),
    "contact_lag_units": _m("서보 단위", 1, 30, 1),
    "grasp_max_open_mm": _m("mm", 20, 100, 1),
    "grasp_bias_mm": _m("mm", -20, 20, 0.5),
    "grasp_tilt_deg": _m("°", 0, 60, 1),
    "grasp_tilt_tol_deg": _m("°", 0, 45, 1),
    "grasp_min_contact_mm": _m("mm", 0, 50, 1),
    "record_hz": _m("Hz", 5, 50, 1),
    "record_max_s": _m("초", 5, 600, 5),
    "seg_pause_deg_s": _m("°/s", 0.5, 30, 0.5),
    "seg_fast_deg_s": _m("°/s", 5, 200, 1),
    "seg_min_s": _m("초", 0.1, 5, 0.1),
    "replay_a_max_deg_s2": _m("°/s²", 50, 3000, 50),
    # 좌표·물체 (직접 조작, 프로그램)
    "target.x": _m("mm", -600, 900, 5),
    "target.y": _m("mm", -600, 900, 5),
    "target.h": _m("mm", 0, 400, 5),
    "target.yaw": _m("°", -180, 360, 5),
    "target.pitch": _m("°", -90, 0, 5),
    "obj.angle": _m("°", -180, 180, 5),
    "obj.width": _m("mm", 0, 150, 1),
    "obj.length": _m("mm", 0, 150, 1),
    "obj.height": _m("mm", 0, 200, 1),
    "joint.deg": _m("°", -180, 180, 1),  # 실제 한계는 고른 관절의 physical_limits_deg를 set_hard()로
    "gripper.width_mm": _m("mm", 0, 111, 5),
    "gripper.value": _m("열림 눈금", 0, 100, 1),
    # 안전
    "safety.joint_margin_deg": _m("°", -15, 30, 1),
    "safety.box.xy": _m("mm", -600, 900, 5, allow_none=True, none_text="비우면 기본 범위"),
    "safety.box.z": _m("mm", 0, 500, 5, allow_none=True, none_text="비우면 기본 범위"),
    # 작업대
    "ws.table_x": _m("mm", 100, 3000, 10),
    "ws.table_y": _m("mm", 100, 3000, 10),
    "ws.margin": _m("mm", 0, 200, 1),
    "ws.base_x": _m("mm", -3000, 6000, 5),
    "ws.base_y": _m("mm", -3000, 6000, 5),
    "ws.zone_x": _m("mm", -3000, 6000, 5),
    "ws.zone_y": _m("mm", -3000, 6000, 5),
    "ws.base_yaw": _m("°", -360, 720, 15),
    # 교시·프로그램
    "replay.speed": _m("배", 0.05, 3, 0.05),
    "program.s": _m("초", 0, 600, 0.5),
    "program.count": _m("회", 1, 1000, 1),
}

SLIDER_UNITS = {"%", "열림 눈금", "토크 눈금", "Hz", "배"}
GAP_PREFIXES = ("target.", "obj.", "ws.", "safety.box.", "gripper.width_mm", "joint.deg")
GAPS = (-10, -5, -1, 1, 5, 10)


def widget_for(key: str) -> str:
    """입력 방식 규칙 (12.7.2, 위에서부터 처음 맞는 것): slider(슬라이더 + 숫자 칸) | gap(숫자 칸 + 간격 버튼) | number(숫자 칸만)."""
    m = META[key]
    if m.widget:
        return m.widget
    if m.unit in SLIDER_UNITS and (m.hard[1] - m.hard[0]) / m.step <= 200:
        return "slider"
    if key.startswith(GAP_PREFIXES):
        return "gap"
    return "number"


def fmt(v: float) -> str:
    """칸 아래 줄의 숫자: 정수면 소수점 없이."""
    return f"{v:g}"


def hint_text(m: Meta, default: float | None, hard: tuple[float, float] | None = None) -> str:
    """칸 아래 한 줄 (12.7.3): `기본 30 · 추천 5~30 · 허용 5~100 %`. 기본·추천이 없으면 뺀다."""
    lo, hi = hard or m.hard
    parts = [f"기본 {fmt(default)}"] if default is not None else []
    if m.soft:
        parts.append(f"추천 {fmt(m.soft[0])}~{fmt(m.soft[1])}")
    parts.append(f"허용 {fmt(lo)}~{fmt(hi)} {m.unit}")
    if m.allow_none:
        parts.append(m.none_text)
    return " · ".join(parts)
