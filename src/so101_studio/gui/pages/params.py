"""파라미터·프리셋 페이지 (gui-spec 3절): 카드 4개, 값마다 설명 한 줄, 바꾼 값 강조, 적용 시점, 프리셋 저장·불러오기."""

from nicegui import run, ui

from ...service.params import CHOICES
from ..backend import RUN_MODES
from ..param_meta import META
from .common import Controls, meta_number

TIMING_TEXT = {"now": "바로", "next_motion": "다음 동작부터", "reconnect": "재시작"}
# (제목, {파라미터: (이름, 설명)}). 목록에 없는 값은 "그 밖"에 설명 없이 보인다
CARDS = (
    ("움직임", {
        "speed_pct": ("속도 (%)", "동작 속도 비율. 실제 팔은 처음에 30% 이하로"),
        "joint_v_max_deg_s": ("관절 최대 속도 (°/s)", "속도 100%일 때의 관절 속도"),
        "a_max_deg_s2": ("관절 최대 가속도 (°/s²)", "클수록 빨리 출발하고 빨리 선다"),
        "joint_jerk_max_deg_s3": ("관절 최대 저크 (°/s³)", "ruckig 궤적에서 가속 변화의 한계"),
        "tool_v_max_mm_s": ("손끝 최대 속도 (mm/s)", "직선 이동의 속도"),
        "tool_a_max_mm_s2": ("손끝 최대 가속도 (mm/s²)", "직선 이동의 가속도"),
        "tool_jerk_max_mm_s3": ("손끝 최대 저크 (mm/s³)", "직선 이동의 가속 변화 한계"),
        "trajectory_profile": ("궤적 방식", "quintic 또는 ruckig"),
        "servo_acceleration": ("서보 가속 한계", "낮을수록 느린 동작의 떨림이 줄어든다"),
        "control_hz": ("제어 주기 (Hz)", "50Hz가 가장 낫다고 측정했다"),
        "table_z_mm": ("작업대 면 높이 (mm)", "작업대 면의 모델 z"),
        "tip_floor_mm": ("바닥 여유 (mm)", "턱 끝이 작업대 면 위로 지킬 높이"),
    }),
    ("집기", {
        "approach_height_mm": ("접근 높이 (mm)", "물체 위에서 내려오기 시작하는 높이"),
        "grasp_height_mm": ("집는 높이 (mm)", "낮은 턱 끝의 작업대 면 위 높이"),
        "place_height_mm": ("놓는 높이 (mm)", "놓을 때 그리퍼 끝 높이"),
        "gripper_margin_mm": ("그리퍼 여유 (mm)", "물체 폭에 더해 벌리는 여유"),
        "gripper_open": ("놓을 때 벌림 (0~100)", "놓을 때 여는 정도"),
        "gripper_speed_pct": ("그리퍼 속도 (%)", "열고 닫는 속도. 상단 막대와 같은 값"),
        "gripper_acceleration": ("그리퍼 가속 한계", "접촉 감지를 이 값에서 맞췄다"),
        "gripper_squeeze_torque": ("조임 힘 (0~1000)", "얇은 물체가 미끄러지지 않게 낮춘 값"),
        "gripper_hold_torque": ("옮길 때 조임 힘 (0~1000)", "집은 뒤 놓을 때까지 쓰는 조임 한계"),
        "grip_preload_units": ("예압", "접촉한 뒤 닫는 쪽으로 더 미는 양"),
        "gripper_load_warn_pct": ("그리퍼 부하 한계 (%)", "이 값을 오래 넘으면 조임을 멈춘다"),
        "gripper_load_s": ("부하 판단 시간 (초)", "이만큼 이어지면 조임을 멈추고, 한 번 더 이어지면 열고 멈춘다"),
        "contact_lag_units": ("접촉 판단 지연", "실제 값이 명령보다 이만큼 앞서 멈추면 닿은 것으로 본다"),
        "grasp_open_policy": ("벌림 방식", "max 또는 fit"),
        "grasp_max_open_mm": ("최대 벌림 (mm)", "최대로 벌릴 틈"),
        "grasp_bias_mm": ("턱 중심 보정 (mm)", "가동 턱 쪽이 +"),
        "grasp_tilt_mode": ("기울임 방식", "symmetric 또는 equal_tips"),
        "grasp_tilt_deg": ("기울임 한계 (°)", "손목 한계 때문에 이 각도까지만 기울인다"),
        "grasp_tilt_tol_deg": ("기울임 허용 각 (°)", "턱 축이 앞뒤 방향에서 이만큼 안쪽일 때만 기울인다"),
        "grasp_min_contact_mm": ("최소 접촉 높이 (mm)", "물체 높이로 집을 때 턱이 이만큼은 물체에 닿아야 한다"),
        "place_clearance_mm": ("놓을 때 벌림 여유 (mm)", "쥔 틈에 양쪽 이만큼 더 벌려 놓는다"),
        "place_open_s": ("놓기 벌림 시간 (초)", "놓을 때 벌림과 손 이동을 끝내는 시간"),
        "place_drop_mm": ("놓는 높이 여유 (mm)", "물체 바닥이 작업 면 위 이 높이에서 놓인다"),
        "place_release_mm": ("기울이기 전 벌림 (mm)", "기울여 놓을 때 먼저 수직으로 벌리는 틈"),
    }),
    ("막힘 감지", {
        "blocked_empty_accel_deg": ("빈 이동 출발·정지 (°)", "명령과 실제 위치 차이가 이보다 크면 막힌 것으로 보고 멈춘다(결정 0059, 실물 30% 측정의 최대 × 1.3, 잠정값)"),
        "blocked_empty_cruise_deg": ("빈 이동 정속 (°)", "정속 구간 문턱. 속도와 상관없이 이 값을 쓴다"),
        "blocked_held_accel_deg": ("쥔 채 출발·정지 (°)", "물체를 쥐고 움직일 때"),
        "blocked_held_cruise_deg": ("쥔 채 정속 (°)", "물체를 쥐고 움직일 때 정속 구간 문턱"),
        "blocked_grip_units": ("그리퍼 닫기 접촉", "집기 밖 닫기에서 실제가 명령보다 이만큼 앞서 멈추면 접촉으로 보고 그 자리에서 쥔다"),
    }),
    ("교시", {
        "record_hz": ("기록 주기 (Hz)", "손으로 움직인 자세를 읽는 주기"),
        "record_max_s": ("최대 기록 시간 (초)", "한 번에 기록하는 길이"),
        "seg_pause_deg_s": ("멈춤 기준 (°/s)", "이 속도 미만이면 멈춤 구간"),
        "seg_fast_deg_s": ("빠름 기준 (°/s)", "이 속도 이상이면 빠른 구간"),
        "seg_min_s": ("최소 구간 (초)", "이보다 짧은 구간은 이웃에 합친다"),
        "replay_a_max_deg_s2": ("변환본 가속 한도 (°/s²)", "변환본 재생이 맞추는 관절 가속"),
    }),
)
HIDDEN = {"speed_pct_default", "watchdog_hotkey", "arm_name", "board_serial", "measured_on"}  # 팔 파일의 신원 값은 여기서 바꾸지 않는다


def build(backend, client_id: str, actions) -> None:
    controls = Controls(backend, client_id)
    baseline = {k: v["value"] for k, v in backend.arm.get_params().items()}
    inputs: dict = {}
    rows: dict = {}

    metas: list = []

    def changes() -> dict:
        return {k: e.value for k, e in inputs.items() if e.value is not None and e.value != baseline[k]}

    def mark_changed(_v=None) -> None:
        changed = changes()
        for key, row in rows.items():
            row.classes(add="param-changed" if key in changed else "", remove="" if key in changed else "param-changed")
        count.text = f"바꾼 값 {len(changed)}개"
        apply_button.enabled = controls.allowed and bool(changed) and controls.valid(metas)

    def field(key: str, text: str, note: str, info: dict) -> None:
        with ui.column().classes("w-full gap-0 param-row").mark(f"row-{key}") as row:
            with ui.row().classes("w-full no-wrap items-center gap-2"):
                ui.label(text).classes("flex-1")
                ui.badge(TIMING_TEXT[info["timing"]], color="grey-8").props("outline").mark(f"timing-{key}")
                value = info["value"]
                if key in CHOICES and len(CHOICES[key]) <= 3:  # 정해진 보기 3개 이하는 토글 (12.7.2 규칙 2)
                    element = ui.toggle(list(CHOICES[key]), value=value, on_change=mark_changed).props("no-caps dense")
                elif key in CHOICES:
                    element = ui.select(list(CHOICES[key]), value=value, on_change=mark_changed).props("outlined dense")
                elif isinstance(value, str):
                    element = ui.input(value=value, on_change=mark_changed).props("outlined dense")
                elif key not in META:  # 표에 없는 숫자 값("그 밖")은 숫자 칸만
                    element = ui.number(value=value, on_change=mark_changed).props("outlined dense")
                else:
                    element = None
                if element is not None:
                    inputs[key] = controls.add(element.classes("param-input").mark(f"param-{key}"))
            if element is None:  # 값 정보표의 칸: 허용·추천·기본 줄과 입력 방식 (12.7)
                m = meta_number(key, value, mark_changed, label=text, default=info.get("saved"), mark=f"param-{key}")
                inputs[key] = controls.add(m)
                metas.append(m)
            if note:
                ui.label(note).classes("stat-label")
        rows[key] = row

    def mode_changed(e) -> None:
        reason = backend.set_run_mode(e.value)
        if reason:
            ui.notify(reason, type="warning")
            run_mode.value = backend.mode

    async def apply() -> None:
        reason, timings = await run.io_bound(backend.set_params, changes())
        if reason:
            ui.notify(reason, type="negative")
            return
        baseline.update({k: inputs[k].value for k in timings})
        mark_changed()
        ui.notify(f"{len(timings)}개를 적용했습니다", type="positive")

    def reload_values() -> None:
        current = backend.arm.get_params()
        for key, element in inputs.items():
            if hasattr(element, "load"):
                element.load(current[key]["value"])  # 프리셋 값을 처음 값으로: [적용]이 이전 값으로 되돌리지 않게 (12.7.4)
            else:
                element.value = current[key]["value"]
            baseline[key] = current[key]["value"]
        mark_changed()

    async def save_preset() -> None:
        name = (preset_name.value or "").strip()
        reason = await run.io_bound(backend.save_preset, name)
        ui.notify(reason or f"프리셋 {name}을 저장했습니다", type="negative" if reason else "positive")
        preset.options = backend.arm.list_presets()
        preset.update()

    async def load_preset() -> None:
        if not preset.value:
            return
        reason = await run.io_bound(backend.load_preset, preset.value)
        ui.notify(reason or f"프리셋 {preset.value}을 불러왔습니다", type="negative" if reason else "positive")
        reload_values()

    with actions:
        count = ui.label("").classes("page-body").mark("param-count")
        apply_button = ui.button("적용", on_click=controls.guard(apply, needs=metas)).props("no-caps").mark("param-apply")

    params = backend.arm.get_params()
    listed = {key for _, group in CARDS for key in group}
    with ui.grid(columns=2).classes("w-full gap-4 items-start"):
        for title, group in CARDS:
            with ui.card().classes("w-full gap-2"):
                ui.label(title).classes("card-title")
                for key, (text, note) in group.items():
                    if key in params:
                        field(key, text, note, params[key])
        with ui.column().classes("w-full gap-4"):
            with ui.card().classes("w-full gap-2"):
                ui.label("실행 방식").classes("card-title")
                run_mode = controls.add(
                    ui.toggle(RUN_MODES, value=backend.mode, on_change=mode_changed).props("no-caps spread").classes("w-full").mark("run-mode")
                )
                ui.label("직접 조작과 교시 재생에 적용됩니다. 위쪽 상태줄에 늘 보입니다").classes("page-body")
            with ui.card().classes("w-full gap-2"):
                ui.label("프리셋").classes("card-title")
                with ui.row().classes("w-full no-wrap items-center gap-2"):
                    preset = controls.add(ui.select(backend.arm.list_presets(), label="불러올 프리셋").props("outlined dense").classes("flex-1").mark("preset-select"))
                    controls.add(ui.button("불러오기", on_click=controls.guard(load_preset)).props("outline no-caps").mark("preset-load"))
                with ui.row().classes("w-full no-wrap items-center gap-2"):
                    preset_name = controls.add(ui.input("새 이름").props("outlined dense").classes("flex-1").mark("preset-name"))
                    controls.add(ui.button("저장", on_click=controls.guard(save_preset)).props("outline no-caps").mark("preset-save"))
            other = [k for k in params if k not in listed and k not in HIDDEN]
            if other:
                with ui.card().classes("w-full gap-2"):
                    ui.label("그 밖").classes("card-title")
                    for key in other:
                        field(key, key, "", params[key])

    def tick() -> None:
        controls.refresh()
        mark_changed()

    ui.timer(0.5, tick)
