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
}
# 연결할 때 복사해 가는 값은 reconnect, 나머지는 Skills·궤적이 cfg를 그때그때 읽으므로 next_motion
RECONNECT_KEYS = {"control_hz", "servo_acceleration", "gripper_acceleration", "table_z_mm", "tip_floor_mm"}
SPEED = "speed_pct"  # cfg가 아니라 ArmService.speed. 바로 적용된다
NOW_KEYS = {SPEED, "gripper_speed_pct"}  # 그리퍼 속도도 Skills가 그때그때 읽어 바로 적용된다


def _scalars(cfg) -> dict:
    return {f.name: getattr(cfg, f.name) for f in dataclasses.fields(cfg)
            if isinstance(getattr(cfg, f.name), (int, float, str)) and f.name not in ("port", "calibration_id", "urdf")}


def timing(name: str) -> str:
    return NOW if name in NOW_KEYS else RECONNECT if name in RECONNECT_KEYS else NEXT


def get_params(svc) -> dict:
    """{이름: {value, default, timing}}. default는 config 파일이 아니라 코드의 기본값(없으면 None)."""
    defaults = {f.name: f.default for f in dataclasses.fields(svc.cfg) if f.default is not dataclasses.MISSING}
    out = {SPEED: {"value": svc.speed, "default": defaults.get("speed_pct_default"), "timing": NOW}}
    for k, v in _scalars(svc.cfg).items():
        out[k] = {"value": v, "default": defaults.get(k), "timing": timing(k)}
    return out


def _check(svc, changes: dict) -> dict:
    cur = {SPEED: svc.speed, **_scalars(svc.cfg)}
    new = {}
    for k, v in changes.items():
        if k not in cur:
            raise ValueError(f"바꿀 수 없는 파라미터: {k}")
        try:
            v = type(cur[k])(v)
        except (TypeError, ValueError):
            raise ValueError(f"{k}: {type(cur[k]).__name__} 값이어야 한다 ({v!r})") from None
        if k in CHOICES and v not in CHOICES[k]:
            raise ValueError(f"{k}는 {CHOICES[k]} 중 하나")
        if k == SPEED and not 1 <= v <= 100:
            raise ValueError("speed_pct는 1~100")
        if k == "gripper_speed_pct" and not 1 <= v <= 100:
            raise ValueError("gripper_speed_pct는 1~100")
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
