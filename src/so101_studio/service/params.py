"""파라미터 읽기·쓰기와 프리셋 (REQ-004). ArmService.get_params/set_params/프리셋 함수가 부른다."""
import dataclasses
import re
from pathlib import Path

import yaml

NOW, NEXT, RECONNECT = "now", "next_motion", "reconnect"
CHOICES = {
    "trajectory_profile": ("quintic", "ruckig"),
    "grasp_open_policy": ("max", "fit"),
    "grasp_tilt_mode": ("symmetric", "equal_tips"),
    "grasp_approach": ("vertical", "auto"),
}
# 연결할 때 복사해 가는 값은 reconnect, 나머지는 Skills·궤적이 cfg를 그때그때 읽으므로 next_motion
RECONNECT_KEYS = {"control_hz", "servo_acceleration", "gripper_acceleration", "table_z_mm", "tip_floor_mm"}
# 서비스가 받는 한계(양 끝 포함, gui-spec 12.7.1 표의 허용과 같다. GUI META는 같거나 더 좁아야 한다). 0이면 위험한 값(0으로 나눔·제한 없음)을 막는 것이 첫째 목적
LIMITS = {
    "joint_v_max_deg_s": (10, 300), "a_max_deg_s2": (10, 2000), "joint_jerk_max_deg_s3": (50, 10000),
    "tool_v_max_mm_s": (10, 500), "tool_a_max_mm_s2": (10, 3000), "tool_jerk_max_mm_s3": (50, 20000),
    "servo_acceleration": (1, 254), "gripper_acceleration": (1, 254), "control_hz": (10, 100),
    "table_z_mm": (-100, 50), "tip_floor_mm": (0, 50), "approach_height_mm": (10, 150), "grasp_height_mm": (0, 100),
    "place_height_mm": (0, 150), "gripper_margin_mm": (0, 60), "gripper_open": (10, 100),
    "place_drop_mm": (0, 20), "bin_drop_clear_mm": (0, 50), "place_release_mm": (0, 30), "place_clearance_mm": (10, 100), "place_open_s": (0.2, 5),
    "blocked_empty_accel_deg": (0.5, 90), "blocked_empty_cruise_deg": (0.5, 90), "blocked_held_accel_deg": (0.5, 90), "blocked_held_cruise_deg": (0.5, 90),
    "blocked_grip_units": (1, 100),
    "gripper_squeeze_torque": (50, 1000), "gripper_hold_torque": (50, 1000), "grip_preload_units": (0, 20),
    "gripper_load_warn_pct": (10, 100), "gripper_load_s": (0.2, 10), "contact_lag_units": (1, 30),
    "grasp_max_open_mm": (20, 100), "grasp_bias_mm": (-20, 20), "grasp_tilt_deg": (0, 60), "grasp_tilt_tol_deg": (0, 45),
    "grasp_contact_tilt_max_deg": (0, 45), "grasp_approach_tilt_pref_deg": (0, 60),
    "grasp_min_contact_mm": (0, 50), "record_hz": (5, 50), "record_max_s": (5, 600), "seg_pause_deg_s": (0.5, 30),
    "seg_fast_deg_s": (5, 200), "seg_min_s": (0.1, 5), "replay_a_max_deg_s2": (50, 3000),
    "speed_pct": (1, 100), "gripper_speed_pct": (1, 100),
}
# 원래 정수인 값(Hz·서보 단위·토크 눈금): 정수가 아니면 거부한다. LIMITS의 나머지는 설정 파일이 0처럼 정수로 적혀 있어도 float로 받는다(0.5를 0으로 자르지 않게)
INT_KEYS = {"control_hz", "record_hz", "servo_acceleration", "gripper_acceleration", "gripper_squeeze_torque", "gripper_hold_torque"}
SPEED = "speed_pct"  # cfg가 아니라 ArmService.speed. 바로 적용된다
NOW_KEYS = {SPEED, "gripper_speed_pct"}  # 그리퍼 속도도 Skills가 그때그때 읽어 바로 적용된다


def _scalars(cfg) -> dict:
    return {f.name: getattr(cfg, f.name) for f in dataclasses.fields(cfg)
            if isinstance(getattr(cfg, f.name), (int, float, str)) and f.name not in ("port", "calibration_id", "urdf")}


def timing(name: str) -> str:
    return NOW if name in NOW_KEYS else RECONNECT if name in RECONNECT_KEYS else NEXT


def get_params(svc) -> dict:
    """{이름: {value, default, saved, timing}}. default는 코드의 기본값(없으면 None), saved는 연결 전 설정 파일 값(합친 cfg, set_params로 바꾸기 전. 없으면 키 없음)."""
    defaults = {f.name: f.default for f in dataclasses.fields(svc.cfg) if f.default is not dataclasses.MISSING}
    saved = getattr(svc, "saved_params", {})
    out = {SPEED: {"value": svc.speed, "default": defaults.get("speed_pct_default"), "timing": NOW}}
    for k, v in _scalars(svc.cfg).items():
        out[k] = {"value": v, "default": defaults.get(k), "timing": timing(k)}
    for k, v in out.items():
        if k in saved:
            v["saved"] = saved[k]
    return out


def _check(svc, changes: dict) -> dict:
    cur = {SPEED: svc.speed, **_scalars(svc.cfg)}
    new = {}
    for k, v in changes.items():
        if k not in cur:
            raise ValueError(f"바꿀 수 없는 파라미터: {k}")
        try:
            if k in INT_KEYS:
                f = float(v)
                if f != int(f):
                    raise ValueError
                v = int(f)
            else:
                v = float(v) if k in LIMITS else type(cur[k])(v)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f"{k}: {'정수' if k in INT_KEYS else type(cur[k]).__name__} 값이어야 한다 ({v!r})") from None
        if k in CHOICES and v not in CHOICES[k]:
            raise ValueError(f"{k}는 {CHOICES[k]} 중 하나")
        if k in LIMITS and not LIMITS[k][0] <= v <= LIMITS[k][1]:
            raise ValueError(f"{k}는 {LIMITS[k][0]}~{LIMITS[k][1]}")
        new[k] = v
    return new


def set_params(svc, changes: dict) -> dict:
    """모두 검증한 뒤 한꺼번에 적용한다(하나라도 틀리면 ValueError, 아무것도 안 바뀜). {이름: 적용 시점}을 돌려준다."""
    new = _check(svc, changes)
    for k, v in new.items():
        svc.set_speed(v) if k == SPEED else setattr(svc.cfg, k, v)
    svc._log("params", new)
    return {k: timing(k) for k in new}


def _path(svc, name: str) -> Path:
    if not re.fullmatch(r"[\w\-가-힣]+", name):
        raise ValueError(f"프리셋 이름은 글자·숫자·_·-만: {name!r}")
    return Path(svc.preset_dir) / f"{name}.yaml"


def save_preset(svc, name: str) -> Path:
    p = _path(svc, name)
    p.parent.mkdir(parents=True, exist_ok=True)
    values = {k: v["value"] for k, v in get_params(svc).items()}
    p.write_text(yaml.safe_dump(values, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return p


def load_preset(svc, name: str) -> dict:
    p = _path(svc, name)
    if not p.exists():
        raise FileNotFoundError(p)
    return set_params(svc, yaml.safe_load(p.read_text(encoding="utf-8")))


def list_presets(svc) -> list[str]:
    return sorted(p.stem for p in Path(svc.preset_dir).glob("*.yaml"))
