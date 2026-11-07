"""so101 doctor: 읽기 전용 점검 (decisions/0027, arm-software-spec.md 7.1). lerobot을 import하지 않는다(lerobot이 없는 PC에서도 "없음"이라고 알려야 한다).
안전: 서보 버스에는 PING(0x01)과 READ(0x02) 패킷만 보낸다(packet()이 다른 명령 코드를 거부한다). ArmDriver.connect는 쓰지 않는다
(connect는 PID·가속과 EEPROM에 쓴다). 포트는 DTR/RTS를 내린 채 열고(ESP32 보드는 이 신호로 리셋된다), USB 시리얼 칩으로 보이는 포트만 연다."""
import importlib.metadata as md
import json
import os
import sys
from pathlib import Path

from .core import ports as ports_mod
from .core.config import ArmSelectionError, arm_files, load_robot_ex

PING, READ = 0x01, 0x02
BAUD = 1_000_000
SERVO_IDS = range(1, 7)
TEMP_WARN_C = 50  # 서보 온도 주의 기준(°C). GUI 서보 건강 대시보드(gui/health.py)와 같다 (REQ-042)
MODEL_STS3215 = 777
# (주소, 바이트 수): STS3215 레지스터 중 읽기만 쓰는 것
REGS = {"model": (3, 2), "min": (9, 2), "max": (11, 2), "homing": (31, 2), "torque": (40, 1), "pos": (56, 2),
        "volt": (62, 1), "temp": (63, 1), "status": (65, 1)}
LEADER_VOLT_MAX = 9.0  # 리더 팔은 약 7.4V, 팔로워는 12V


def packet(sid: int, instruction: int, *params: int) -> bytes:
    """서보 패킷. 읽기 전용 점검이므로 PING과 READ 외의 명령 코드는 만들 수 없다."""
    if instruction not in (PING, READ):
        raise ValueError(f"읽기 전용: 명령 코드 {instruction:#x}는 보내지 않는다")
    body = [sid, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def open_port(device: str):
    """DTR/RTS를 내린 채 연다(open() 앞에서 설정해야 열 때 신호가 올라가지 않는다)."""
    import serial

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = device, BAUD, 0.05
    s.dtr = s.rts = False
    s.open()
    return s


_open = open_port  # 시험에서 가짜 시리얼로 바꾼다


def _ask(s, sid: int, instruction: int, *params: int, n: int = 6) -> bytes:
    s.reset_input_buffer()
    s.write(packet(sid, instruction, *params))
    s.flush()
    data = s.read(n)
    return data if len(data) >= n and data[:2] == b"\xff\xff" and data[2] == sid else b""


def _read_reg(s, sid: int, name: str) -> int | None:
    addr, size = REGS[name]
    d = _ask(s, sid, READ, addr, size, n=6 + size)
    return int.from_bytes(d[5:5 + size], "little") if d else None


def _sign_magnitude(raw: int, bit: int = 11) -> int:
    return -(raw & ((1 << bit) - 1)) if raw & (1 << bit) else raw


def probe_servos(device: str) -> dict:
    """한 포트를 열어 서보 1~6에 PING을 보내고 응답한 서보의 레지스터를 읽는다.
    {"state": "busy"|"error"|"ok", "error", "servos": [...], "missing": [id]}"""
    try:
        s = _open(device)
    except PermissionError as e:
        return {"state": "busy", "error": str(e), "servos": [], "missing": []}
    except Exception as e:  # noqa: BLE001 serial.SerialException 등: 열 수 없는 이유를 그대로 알린다
        return {"state": "busy" if "denied" in str(e).lower() or "access" in str(e).lower() else "error", "error": str(e), "servos": [], "missing": []}
    servos, missing = [], []
    try:
        for sid in SERVO_IDS:
            if not _ask(s, sid, PING):
                missing.append(sid)
                continue
            r = {k: _read_reg(s, sid, k) for k in REGS}
            servos.append({"id": sid, "model": r["model"], "voltage_v": None if r["volt"] is None else r["volt"] / 10,
                           "temp_c": r["temp"], "torque": None if r["torque"] is None else bool(r["torque"]), "status": r["status"],
                           "pos": r["pos"], "_homing": None if r["homing"] is None else _sign_magnitude(r["homing"]),
                           "_min": r["min"], "_max": r["max"]})
    finally:
        s.close()
    return {"state": "ok", "error": "", "servos": servos, "missing": missing}


def find_calibration(calibration_id: str, config_dir: Path) -> tuple[dict | None, str]:
    """lerobot이 쓰는 캘리브레이션 파일(HF 캐시)을 먼저, 없으면 레포 사본(config/calibration)."""
    for p in (Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower" / f"{calibration_id}.json",
              config_dir / "calibration" / f"{calibration_id}.json"):
        if p.exists():
            return json.loads(p.read_text(encoding="utf-8")), str(p)
    return None, ""


def compare_calibration(cal: dict, servos: list[dict]) -> list[str]:
    """서보 레지스터(호밍 오프셋, 가동 범위)와 캘리브레이션 파일을 대조한다. 다른 항목을 문장으로 돌려준다."""
    by_id = {v["id"]: (k, v) for k, v in cal.items()}
    out = []
    for sv in servos:
        if sv["id"] not in by_id:
            continue
        name, want = by_id[sv["id"]]
        for key, reg, label in (("homing_offset", "_homing", "호밍 오프셋"), ("range_min", "_min", "범위 최소"), ("range_max", "_max", "범위 최대")):
            if sv[reg] is not None and sv[reg] != want[key]:
                out.append(f"{name}: {label} 서보 {sv[reg]}, 파일 {want[key]}")
    return out


def _check(name: str, state: str, text: str, fix: str = "") -> dict:
    return {"name": name, "state": state, "text": text, "fix": fix}


def _pkg(name: str) -> str | None:
    try:
        return md.version(name)
    except md.PackageNotFoundError:
        return None


def _env_checks(conda_dir: Path | None = None) -> list[dict]:
    out = []
    v = sys.version_info
    in_conda = bool(os.environ.get("CONDA_DEFAULT_ENV")) or (Path(sys.prefix) / "conda-meta").is_dir()
    out.append(_check("환경", "ok" if (v >= (3, 12) and in_conda) else "warn",
                      f"Python {v.major}.{v.minor}.{v.micro}, conda 환경 {'맞음' if in_conda else '아님'} ({sys.prefix})",
                      "" if (v >= (3, 12) and in_conda) else "environment.yml로 conda 환경 so101을 만든다 (conda env create -f environment.yml)"))
    nm = _pkg("numpy")
    meta = Path(sys.prefix) / "conda-meta"
    conda_numpy = bool(nm) and meta.is_dir() and any(meta.glob("numpy-*.json"))
    out.append(_check("numpy", "ok" if conda_numpy else ("warn" if nm else "fail"),
                      f"numpy {nm}" + (" (conda 설치본)" if conda_numpy else " (conda 설치본이 아님)" if nm else " 없음"),
                      "" if conda_numpy else "pip로 numpy를 다시 설치하지 말고 conda로 설치한다 (placo와 MKL이 맞아야 한다)"))
    for pkg, need, fix in (("lerobot", True, "pip install \"lerobot[feetech]==0.6.1\""), ("placo", True, "conda install -c conda-forge placo"),
                           ("pyserial", True, "pip install pyserial"), ("ruckig", True, "pip install ruckig"), ("pyyaml", True, "conda install pyyaml")):
        ver = _pkg(pkg)
        out.append(_check(pkg, "ok" if ver else "fail", f"{pkg} {ver}" if ver else f"{pkg} 없음", "" if ver else fix))
    gui = {p: _pkg(p) for p in ("nicegui", "viser")}
    out.append(_check("GUI 의존성", "ok" if all(gui.values()) else "skip", ", ".join(f"{p} {v or '없음'}" for p, v in gui.items()),
                      "" if all(gui.values()) else "GUI를 쓰려면 pip install -e .[gui]"))
    return out


def doctor(cfg=None, probe: bool = True, port: str | None = None, progress=None, config_path: str | Path = "config/robot.yaml",
           arm: str | None = None) -> dict:
    """읽기 전용 점검. {"ok", "checks": [{name, state(ok|warn|fail|skip), text, fix}], "ports": [...], "arm": ..., "esp32": None}.
    probe=False면 포트를 열지 않고 OS가 주는 목록만 본다. port를 주면 그 포트만 탐침한다."""
    say = progress or (lambda _t: None)
    config_path = Path(config_path)
    config_dir = config_path.parent
    checks = _env_checks()
    say("환경·패키지 확인")
    port_list = ports_mod.list_serial_ports()
    arms = {n: (p, __import__("yaml").safe_load(p.read_text(encoding="utf-8")) or {}) for n, p in arm_files(config_dir / "arms").items()}
    serial_to_arm = {str(d.get("board_serial")): n for n, (_p, d) in arms.items() if d.get("board_serial")}
    usb = [p for p in port_list if ports_mod.is_usb_serial(p)]
    checks.append(_check("포트 목록", "ok" if usb else "fail", f"포트 {len(port_list)}개, USB 시리얼 {len(usb)}개" + "".join(
        f"\n  {p['device']} {ports_mod.fmt_id(p)} 시리얼 {p['serial']} {p['description']}" for p in port_list),
        "" if usb else "USB 케이블과 보드 전원을 확인한다 (리눅스는 dialout 그룹도)"))
    say("포트 목록 확인")

    # 팔 파일 고르기 (읽기만)
    chosen_serial = [p["serial"] for p in usb if p.get("serial") in serial_to_arm]
    info = {"arm": None, "reason": ""}
    try:
        cfg_now, info = load_robot_ex(config_path, arm=arm, serials=chosen_serial) if cfg is None else (cfg, {"arm": cfg.arm_name or None, "reason": "넘겨 받은 설정"})
        checks.append(_check("팔 파일", "ok" if info["arm"] else "warn", f"{info['arm'] or '없음'} ({info['reason']})",
                             "" if info["arm"] else "config/arms/_template.yaml을 복사해 팔 파일을 만든다"))
    except ArmSelectionError as e:
        cfg_now = load_robot_ex(config_path, arms_dir=config_dir / "_no_arms")[0] if cfg is None else cfg
        checks.append(_check("팔 파일", "fail", str(e), "--arm <이름>이나 config/local.yaml의 arm으로 고른다"))

    cal, cal_path = (find_calibration(cfg_now.calibration_id, config_dir) if cfg_now.calibration_id else (None, ""))
    checks.append(_check("캘리브레이션 파일", "ok" if cal else "fail", f"{cfg_now.calibration_id}: {cal_path}" if cal else f"{cfg_now.calibration_id or '(id 없음)'} 파일을 못 찾았다",
                         "" if cal else "lerobot-calibrate --robot.type=so101_follower --robot.port=<포트> --robot.id=<id> (모터에 쓰고 움직이니 직접 실행한다)"))

    autocal = None
    rec = Path("runs") / f"autocal-save-{cfg_now.calibration_id}.json"  # so101 calibrate --save가 남긴 기록
    if cfg_now.calibration_id and rec.is_file():
        autocal = json.loads(rec.read_text(encoding="utf-8"))
        checks.append(_check("자동 캘리브레이션", "ok", f"{autocal['saved_at']}에 저장({len(autocal['rows'])}개 관절), 이전 파일 {autocal['backup'] or '없음'}"))

    # 포트 역할과 탐침
    roles, arm_result, probed = [], None, {}
    candidates = [p for p in usb if p["device"] == port] if port else [p for p in usb if p.get("vid") != ports_mod.ESP32_VID]
    if probe:
        for p in candidates:
            say(f"{p['device']} 탐침 (PING/READ만)")
            probed[p["device"]] = probe_servos(p["device"])
    for p in port_list:
        dev = p["device"]
        r = probed.get(dev)
        if p.get("vid") == ports_mod.ESP32_VID:
            role, detail = "esp32", "ESP32 후보(VID 303A)"
        elif not ports_mod.is_usb_serial(p):
            role, detail = "unknown", "USB 시리얼 칩이 아니라 열지 않았다"
        elif r and r["state"] == "busy":
            role, detail = "busy", "다른 프로그램이 쓰는 중이다: " + r["error"]
        elif r and r["servos"]:
            role, detail = "arm", f"서보 {len(r['servos'])}개 응답"
        elif r:
            role, detail = "unknown", "서보 응답 없음" + (f" ({r['error']})" if r["error"] else "")
        else:
            role, detail = ("arm", f"보드 시리얼이 팔 파일 {serial_to_arm[p['serial']]}과 같다") if p.get("serial") in serial_to_arm else ("unknown", "탐침하지 않았다")
        roles.append({**p, "role": role, "detail": detail})

    responders = [d for d, r in probed.items() if r["servos"]]
    if not probe:
        for n in ("서보 응답", "전압", "온도", "캘리브레이션 대조"):
            checks.append(_check(n, "skip", "탐침하지 않았다 (포트를 열지 않았다)"))
    elif len(responders) > 1:
        checks.append(_check("서보 응답", "warn", f"서보가 응답하는 포트가 여럿이다: {responders}", "--port <포트>로 하나를 고른다 (리더 팔도 같은 모델·ID로 응답한다)"))
    elif not responders:
        busy = [d for d, r in probed.items() if r["state"] == "busy"]
        checks.append(_check("서보 응답", "fail", "서보가 응답하는 포트가 없다" + (f" (사용 중: {busy})" if busy else ""),
                             "12V 전원, 3핀 케이블, 보드 USB를 확인한다" + ("; 다른 프로그램(GUI, 터미널)이 포트를 쓰는지 본다" if busy else "")))
    else:
        dev = responders[0]
        r = probed[dev]
        sv = r["servos"]
        dev_info = next(p for p in port_list if p["device"] == dev)
        got = len(sv)
        checks.append(_check("서보 응답", "ok" if got == 6 and not any(s["model"] != MODEL_STS3215 for s in sv) else "warn",
                             f"{dev}: 서보 {got}/6 응답" + (f", 응답 없음 {r['missing']}" if r["missing"] else ""),
                             "" if got == 6 else "응답 없는 서보의 3핀 케이블 접촉을 확인한다 (so101 linkwatch)"))
        volts = [s["voltage_v"] for s in sv if s["voltage_v"] is not None]
        vmax = max(volts) if volts else None
        if vmax is None:
            checks.append(_check("전압", "fail", "전압을 못 읽었다", "서보 전원을 확인한다"))
        elif vmax < LEADER_VOLT_MAX:
            checks.append(_check("전압", "warn", f"{min(volts):.1f}~{vmax:.1f}V: 리더 팔(7.4V)일 수 있다", "팔로워는 12V 전원이다. 리더 팔이면 이 프로젝트의 팔이 아니다"))
        elif min(volts) < 11.0:
            checks.append(_check("전압", "warn", f"{min(volts):.1f}~{vmax:.1f}V: 12V보다 낮다", "전원 어댑터와 케이블을 확인한다"))
        else:
            checks.append(_check("전압", "ok", f"{min(volts):.1f}~{vmax:.1f}V"))
        temps = [s["temp_c"] for s in sv if s["temp_c"] is not None]
        hot = max(temps) if temps else None
        checks.append(_check("온도", "skip" if hot is None else ("warn" if hot >= TEMP_WARN_C else "ok"), "온도를 못 읽었다" if hot is None else f"최고 {hot}°C",
                             "" if hot is None or hot < TEMP_WARN_C else "식을 때까지 쉰다"))
        mismatch = compare_calibration(cal, sv) if cal else []
        if not cal:
            checks.append(_check("캘리브레이션 대조", "skip", "비교할 캘리브레이션 파일이 없다"))
        elif mismatch:
            checks.append(_check("캘리브레이션 대조", "fail", f"서보와 파일이 다르다 ({len(mismatch)}개): " + "; ".join(mismatch[:3]),
                                 "보드를 다른 팔로 옮겼거나 서보를 새로 캘리브레이션했다. 팔 파일(calibration_id)을 확인하고, 맞지 않으면 lerobot-calibrate를 다시 한다. 그동안 움직이는 명령은 막힌다"))
        else:
            checks.append(_check("캘리브레이션 대조", "ok", f"{len(sv)}개 서보의 호밍 오프셋·범위가 파일과 같다"))
        arm_result = {"port": dev, "serial": dev_info.get("serial"),
                      "servos": [{k: v for k, v in s.items() if not k.startswith("_")} for s in sv],
                      "calibration_id": cfg_now.calibration_id, "calibration_found": bool(cal), "calibration_mismatch": mismatch}
    return {"ok": not any(c["state"] == "fail" for c in checks), "checks": checks, "ports": roles, "arm": arm_result, "esp32": None, "autocal": autocal}


def format_report(res: dict) -> str:
    mark = {"ok": "정상", "warn": "주의", "fail": "실패", "skip": "건너뜀"}
    lines = []
    for c in res["checks"]:
        lines.append(f"[{mark[c['state']]}] {c['name']}: {c['text']}")
        if c["fix"] and c["state"] != "ok":
            lines.append(f"    고치기: {c['fix']}")
    lines.append("종합: " + ("이상 없음" if res["ok"] else "실패 항목이 있다"))
    return "\n".join(lines)
