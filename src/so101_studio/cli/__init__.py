"""CLI (arm-software-spec.md 9절). 모든 명령은 ArmService를 거친다."""
import argparse
import importlib
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

from ..arm.sim_driver import SimDriver
from ..core import ports as ports_mod
from ..core.config import ArmSelectionError, load_robot_ex, load_workspace
from ..core.types import DetectedObject, JointState, ToolPose
from ..service.arm_service import ArmService, plan_goto  # noqa: F401  plan_goto는 테스트가 이 경로로도 쓴다

COMMANDS = ["status", "home", "park", "movej", "goto", "pick", "place", "gripper", "run", "keys", "powertest", "shaketest", "servoinfo", "linkwatch", "info", "record", "replay", "seq", "gui", "doctor", "find-port", "incident", "calibrate", "checktest"]


MUJOCO_MISSING_TEXT = "MuJoCo가 설치되지 않았습니다. docs/guides/tooling-setup.md의 MuJoCo 줄로 설치하세요"


def make_driver(sim: bool, cfg):
    if sim:
        return SimDriver(cfg.poses.get("home"))
    from ..arm.driver import ArmDriver  # lerobot이 필요한 import는 실제 팔을 쓸 때만

    return ArmDriver(cfg.pid, cfg.joint_offsets_deg, cfg.servo_acceleration, cfg.gripper_acceleration)


def status_text(st) -> str:
    q = asdict(st.joints)
    lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
    lines += [f"{j:14s} {q[j]:8.1f} {st.temperature_c.get(j, 0):7.0f} {st.load_pct.get(j, 0):7.1f}" for j in q]
    lines.append(f"voltage {st.voltage_v:.1f} V  torque {'on' if st.torque_enabled else 'off'}")
    return "\n".join(lines)


def fmt(r) -> str:
    err = "-" if r.position_error_mm is None else f"{r.position_error_mm:.1f} mm"
    return f"{r.status}  {r.duration_s:.2f} s  끝점 오차 {err}"


def run_task(svc: ArmService, a) -> str:
    from ..perception.manual import ManualSource
    from ..service.studio import Studio

    k = getattr(a, "truth_layout", None)
    sim = getattr(a, "sim", False)
    if k is not None and getattr(a, "source", "manual") != "ultrasonic":
        sys.exit("--truth-layout은 --source ultrasonic과 함께 쓴다")
    boxes = None
    if k is not None:  # I1 정답 기록 (ultrasonic-spec 8.6)
        from ..service.check_test import i1_layout

        boxes = i1_layout(k)
        print(f"I1 배치 {k}:")
        for i, b in enumerate(boxes, 1):
            print(f"  P{i}: {b.width:g}x{b.length:g}x{b.height:g}mm, 중심 ({b.x:g}, {b.y:g}), 긴 축 {b.angle_deg % 180:g}°")
        if not sim:
            input("물체를 놓고 손을 뺀 뒤 Enter ")
    if getattr(a, "source", "manual") == "ultrasonic":
        from ..service.ultrasonic import make_ultrasonic_source

        source = make_ultrasonic_source(svc, sim=sim, port=getattr(a, "us_port", None),
                                        boxes=None if boxes is None else list(boxes))  # 가상 ESP32는 집어 간 물체를 이 목록에서 지운다: 정답은 따로 둔다
    else:
        source = ManualSource()
    run = None
    if k is not None:
        from ..perception.ultrasonic.truth import TruthRun

        run = TruthRun("I1", source.cfg, sim=sim, note=f"log={getattr(svc.logger, 'dir', None)} {a.note}".strip())
        source.record_dir = run.dir
    st = Studio(svc, source, dry_run=a.dry_run)
    st.bus.subscribe("task", lambda e: print(f"[{e.state}] {e.object_id or ''} {e.result or ''}".rstrip()))
    if a.step:
        st.task.wait_step = lambda: input("  (Enter: 다음 단계) ")
    svc.enable_torque()  # 잠겼으면 켜지 않고, 작업은 첫 동작에서 locked로 끝난다
    try:
        res = st.start(step_mode=a.step, blocking=True)
    finally:
        if run is not None:
            from ..service.check_test import i1_truth

            run.add(i1_truth(boxes), source.saved, 1)
            run.close()
            print(f"정답 기록: {run.dir}")
    return f"끝: {res['end']}  옮긴 물체 {len(res['picked'])}개 {res['picked']}  실패 {res['failed']}  사이클 {res['cycles']}"


def run_checktest(svc: ArmService, a) -> None:
    """checktest a1 (ultrasonic-spec 8.6): 정답 가까운 후보에서 CHECK만 12회. --sim이면 묻지 않고 가상 ESP32에 물체를 놓는다."""
    from ..perception.ultrasonic.model import Box, load_config
    from ..perception.ultrasonic.truth import TruthRun
    from ..service.check_test import A1_N, a1_plan, a1_summary, run_a1
    from ..service.studio import Studio
    from ..service.ultrasonic import make_ultrasonic_source

    if a.values[:1] != ["a1"]:
        sys.exit("checktest: 하위 명령은 a1뿐이다 (so101 checktest a1)")
    try:
        obj = tuple(float(x) for x in a.obj.lower().split("x"))
        assert len(obj) == 3
    except (ValueError, AssertionError):
        sys.exit("--obj는 폭x길이x높이(mm), 예: 40x80x40")
    limit = load_config().check.max_object_height_mm
    if obj[2] > limit:  # 확인 높이가 max_object_height_mm + clearance_mm로 고정이라 더 높은 물체에 닿는다
        sys.exit(f"--obj 높이 {obj[2]:g}mm가 check.max_object_height_mm({limit:g})보다 크다: 센서가 물체에 닿는다")
    svc.enable_torque()
    src = make_ultrasonic_source(svc, sim=a.sim, port=a.us_port)
    st = Studio(svc, src)
    run = TruthRun("A1", src.cfg, sim=a.sim, note=a.note)
    src.record_dir = run.dir

    def place(item):
        t = item["truth"]
        if a.sim:
            src.esp.boxes = [Box(t["x_mm"], t["y_mm"], t["angle_deg"] or 0.0, t["width_mm"], t["length_mm"], t["height_mm"])]
            return t
        print(f"{item['n']}/{A1_N}: {item['spot']} ({t['x_mm']}, {t['y_mm']})에 물체 중심, 긴 축 {t['angle_deg'] if t['angle_deg'] is not None else '-'}°. 놓고 손을 뺀 뒤 Enter")
        while True:
            s = input('(다르게 놓았으면 "x y 각도", q = 끝) ').strip()
            if s.lower() == "q":
                return None
            if not s:
                return t
            try:
                x, y, ang = (float(v) for v in s.split())
            except ValueError:
                print("  x y 각도 세 값을 띄어 쓴다")
                continue
            return {**t, "x_mm": x, "y_mm": y, "angle_deg": None if t["angle_deg"] is None else ang % 180}

    res = run_a1(svc, st.task, src, run, place, a1_plan(a.seed, a.offset, obj, a.reps))
    print(a1_summary(res))
    print(f"기록: {run.dir}")


GRIPPER_NAMES = {"parallel": "병렬", "single_jaw": "단동식", None: "모름"}


def print_info(info: dict) -> None:
    print(f"길이 {info['duration_s']:.1f}초, 점 {info['points']}개, 기록 주기 {info['hz_effective']:.1f}Hz, "
          f"평균 속도 {info['mean_deg_s']:.0f}°/s, 최대 {info['max_deg_s']:.0f}°/s" + (f", 기록 {info['recorded_at']}" if info.get("recorded_at") else "")
          + f", 그리퍼 {GRIPPER_NAMES.get(info.get('gripper'), info.get('gripper'))}")
    for s in info["segments"]:
        print(f"  구간 {s['index']}: {s['start_s']:5.1f}~{s['end_s']:5.1f}초  {s['label']}  평균 {s['mean_deg_s']:4.0f}°/s  최대 {s['max_deg_s']:4.0f}°/s")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio")
    ap.add_argument("--sim", action="store_true", help="SimDriver로 실행")
    ap.add_argument("--sim-engine", choices=["kinematic", "mujoco"], default="kinematic", help="--sim과 함께: kinematic(기본, SimDriver) 또는 mujoco(물리)")
    ap.add_argument("--sim-viewer", action="store_true", help="--sim --sim-engine mujoco와 함께: MuJoCo 뷰어 창을 띄운다")
    ap.add_argument("--sim-scenario", help="--sim --sim-engine mujoco와 함께: 물체 시나리오 이름(config/sim_scenarios/<이름>.yaml) 또는 .yaml 경로. 없으면 config/manual_objects.yaml")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--speed", type=float, help="속도 %% (기본: 설정값)")
    ap.add_argument("--place", help="pick 뒤에 놓을 배출 구역 이름 (place 명령은 필수)")
    ap.add_argument("--width", action="store_true", help="gripper: 값을 그리퍼 값이 아니라 벌림 폭(mm)으로 해석")
    ap.add_argument("--margin", type=float, help="pick: gripper_margin_mm를 이번만 바꿔 시험")
    ap.add_argument("--tilt", type=float, help="pick: 최대 기울기(°)를 이번만 바꿔 시험 (0이면 기울이지 않음)")
    ap.add_argument("--max-open", type=float, help="pick: 최대로 벌릴 틈(mm)을 이번만 바꿔 시험")
    ap.add_argument("--bias", type=float, help="pick: grasp_bias_mm를 이번만 바꿔 시험 (가동 턱 쪽이 +)")
    ap.add_argument("--source", default="manual", choices=["manual", "ultrasonic"], help="run: 인식 방식")
    ap.add_argument("--us-port", help="run --source ultrasonic: ESP32 시리얼 포트 (기본: config/ultrasonic.yaml의 port)")
    ap.add_argument("--dry-run", action="store_true", help="run: 물체 없이 동작만 (접촉이 없어도 성공으로 보고 이어간다)")
    ap.add_argument("--step", action="store_true", help="run: 상태가 바뀔 때마다 Enter를 기다린다")
    ap.add_argument("--jog-speed", type=float, default=30.0, help="keys: 관절 모드 이동 속도 (°/s)")
    ap.add_argument("--cart-speed", type=float, default=80.0, help="keys: 직교 조그 속도 (mm/s, 60 넘게도 가능)")
    ap.add_argument("--hz", type=int, help="제어 주기 Hz를 이번만 바꿔 시험 (기본: 설정값 50). 서보 목표를 더 자주 보내 떨림이 줄어드는지 보는 용도")
    ap.add_argument("--unsafe", action="store_true", help="관절 한계·작업 영역·바닥 검사를 모두 끈다 (한 주기 이동량 한계와 과열·과부하 감시는 유지). 매우 조심해서만")
    ap.add_argument("--safety-margin", type=float, help="팔 관절 한계를 이만큼(°) 안쪽으로 줄인다. 음수면 실제 가동 범위까지 넓힌다")
    ap.add_argument("--box", help="허용 범위(작업대 좌표 mm): x최소,x최대,y최소,y최대,z최소,z최대. z는 작업대 면 위 낮은 턱 끝 높이. 'none'이면 제한 없음")
    ap.add_argument("--record-hz", type=int, help="record: 기록 주기 Hz를 이번만 바꾼다 (기본: 설정 record_hz)")
    ap.add_argument("--record-max-s", type=float, help="record: 최대 기록 시간(초)을 이번만 바꾼다 (기본: 설정 record_max_s)")
    ap.add_argument("--seg-speed", help="replay: 구간별 속도 배율 '번호=배율,...' 예: 1=0.5,3=2 (구간 번호는 info로 본다)")
    ap.add_argument("--share", action="store_true", help="gui: 같은 네트워크의 다른 기기에서 보기 전용으로 보게 연다. 결정 0047")
    ap.add_argument("--free", action="store_true", help="작업대 가장자리 검사를 끈다 (바닥과 관절 한계는 유지). 손으로 움직이는 조종·재생용")
    ap.add_argument("--until-limit", action="store_true", help="replay: 한계나 바닥을 처음 넘는 자세 앞까지만 재생")
    ap.add_argument("--any-gripper", action="store_true", help="replay·seq: 기록한 그리퍼가 지금과 달라도 재생(기본은 거부, 기록에 그리퍼가 없으면 경고만)")
    ap.add_argument("--mode", choices=["original", "fitted"], default="original", help="replay: fitted면 한 주기 이동량·가속 한도를 넘는 구간만 늦춰 재생(info가 미리 보여 준다)")
    ap.add_argument("--trace", action="store_true", help="keys: 명령·실제 관절 위치를 빠르게 기록하고 떨림 요약 (runs/.../trace.csv)")
    ap.add_argument("--speeds", help="shaketest: 시험할 속도들(°/s, 쉼표로). 기본 20,60,200")
    ap.add_argument("--arm", help="팔 파일 이름(config/arms/<이름>.yaml). 기본: local.yaml -> 보드 시리얼 -> 하나뿐이면 그것")
    ap.add_argument("--port", help="시리얼 포트. 기본: local.yaml -> 보드 시리얼로 자동 찾기 (so101 find-port)")
    ap.add_argument("--json", action="store_true", help="doctor: JSON으로 출력")
    ap.add_argument("--no-probe", action="store_true", help="doctor: 포트를 열지 않고 목록만 본다")
    ap.add_argument("--auto", action="store_true", help="calibrate: 자동 캘리브레이션(--measure-only와 함께)")
    ap.add_argument("--measure-only", action="store_true", help="calibrate --auto: 서보 EEPROM에 쓰지 않고 측정만 해 기존 캘리브레이션과 비교")
    ap.add_argument("--save", action="store_true", help="calibrate: 합격한 측정 결과를 EEPROM·LeRobot 캐시·config/calibration에 저장")
    ap.add_argument("--from", dest="from_file", help="calibrate --save: 저장할 측정 결과 파일(기본: 가장 최근 runs/*/measure.json)")
    ap.add_argument("--verify-zero", action="store_true", help="calibrate: 속도 20%%로 전 관절을 0°로 보내고 실물과 3D의 0° 자세를 사람이 확인하면 팔 파일에 calibration_checked를 기록")
    ap.add_argument("--velocity", type=int, default=200, help="calibrate --auto: 끝단 찾기 속도(steps/s, 기본 200)")
    ap.add_argument("--torque-limit", type=int, help="calibrate --auto: 토크 한계(0~1000, 기본 500=50%%)")
    ap.add_argument("--last", type=float, default=60.0, help="incident: 지금부터 거슬러 올라갈 초 (기본 60)")
    ap.add_argument("--obj", default="40x80x40", help="checktest a1: 물체 폭x길이x높이 mm")
    ap.add_argument("--offset", type=float, default=30.0, help="checktest a1: 후보를 정답에서 떨어뜨리는 최대 거리 mm")
    ap.add_argument("--seed", type=int, default=0, help="checktest a1: 후보 씨앗")
    ap.add_argument("--reps", type=int, default=1, help="checktest a1: 12회 묶음 수")
    ap.add_argument("--note", default="", help="checktest a1·run --truth-layout: truth.json 메모")
    ap.add_argument("--truth-layout", type=int, choices=range(1, 11), metavar="K", help="run --source ultrasonic: I1 정답 배치 K(1~10)를 기록 (ultrasonic-spec 8.6)")
    ap.add_argument("command", choices=COMMANDS)
    ap.add_argument("values", nargs="*", help="movej: 관절 6개 / goto: x y z yaw (z는 ToolPose 모델 z: 기준점 높이이고 턱 끝의 작업대 위 높이가 아니다. 턱 끝 높이로 보내려면 GUI 좌표 이동이나 keys 좌표 모드) / pick: x y 각도 폭 [높이](폭·높이 0이면 모름. 팔 방향과 어긋난 물체는 높이를 모르면 위 모서리를 칠 수 있다) / gripper: 값 / record·replay·seq: 이름")
    a = ap.parse_args(argv)
    if a.sim_scenario and not (a.sim and a.sim_engine == "mujoco"):
        sys.exit("--sim-scenario는 --sim --sim-engine mujoco와 함께 쓴다")
    if a.sim_viewer and not (a.sim and a.sim_engine == "mujoco"):
        sys.exit("--sim-viewer는 --sim --sim-engine mujoco와 함께 쓴다")
    if a.sim_engine == "mujoco":
        if not a.sim:
            sys.exit("--sim-engine은 --sim과 함께 쓴다")
        if a.command in ("calibrate", "linkwatch"):
            sys.exit(f"{a.command}은 시뮬레이션에서 쓸 수 없다")
        if a.dry_run:
            sys.exit("물리 시뮬레이션에서는 --dry-run을 쓰지 않는다")
    if a.command in ("doctor", "find-port", "incident"):
        from ..bringup import run

        sys.exit(run(a))
    if a.command == "keys" and sys.platform != "win32":
        sys.exit("keys·jog는 Windows 전용이다(msvcrt). 다른 명령(status, movej, goto, run 등)은 쓸 수 있다")
    port_list = [] if a.sim else ports_mod.list_serial_ports()  # 목록만 읽는다(포트를 열지 않는다)
    try:
        cfg, info = load_robot_ex(a.config, arm=a.arm, serials=[p["serial"] for p in port_list])
    except ArmSelectionError as e:
        sys.exit(str(e))
    if not a.sim and a.command != "gui":  # gui는 팔을 직접 연결한다
        try:
            port, why = ports_mod.pick_port(port_list, cfg.board_serial, a.port, info["local"].get("port"), cfg.port)
        except ports_mod.PortChoiceError as e:
            sys.exit(f"{e}\n후보: " + (", ".join(f"{p['device']}({ports_mod.fmt_id(p)}, 시리얼 {p['serial']})" for p in e.candidates) or "없음"))
        cfg = replace(cfg, port=port)
        print(f"팔 {cfg.arm_name or '(팔 파일 없음)'} ({info['reason']}), 포트 {port} ({why})")
    cfg = replace(
        cfg,
        gripper_margin_mm=cfg.gripper_margin_mm if a.margin is None else a.margin,
        grasp_tilt_deg=cfg.grasp_tilt_deg if a.tilt is None else a.tilt,
        grasp_max_open_mm=cfg.grasp_max_open_mm if a.max_open is None else a.max_open,
        grasp_bias_mm=cfg.grasp_bias_mm if a.bias is None else a.bias,
        control_hz=cfg.control_hz if a.hz is None else a.hz,
    )
    if a.command == "calibrate" and not a.verify_zero:  # ArmService를 거치지 않는다: connect는 PID·가속을 서보에 쓴다 (REQ-016)
        from ..arm.autocal.cli import run_calibrate

        sys.exit(run_calibrate(a, cfg, config_dir=Path(a.config).parent))
    if a.command == "gui":  # 팔은 GUI가 연결한다. 여기서는 연결하지 않는다 (REQ-005)
        try:
            gui = importlib.import_module("so101_studio.gui")
        except ModuleNotFoundError as e:
            what = "GUI 모듈이 이 브랜치에 없다 (GUI 브랜치가 합쳐진 뒤 쓸 수 있다)" if e.name == "so101_studio.gui" else f"GUI 의존성 {e.name}이 없다: pip install -e .[gui]"
            sys.exit(what)
        gui.main(a)
        return
    if a.command == "linkwatch":  # 서보 연결 없이 포트만 쓴다
        from .linkwatch import run_linkwatch

        run_linkwatch(cfg.port)
        return
    ws = load_workspace()
    if a.sim and a.sim_engine == "mujoco":
        from ..core.scene import load_scene

        try:
            from ..arm.sim_driver import make_sim_driver

            driver = make_sim_driver(cfg, ws, load_scene(), scenario=a.sim_scenario, viewer=a.sim_viewer)
        except ImportError:
            print(MUJOCO_MISSING_TEXT)
            sys.exit(2)
        except ValueError as e:
            sys.exit(str(e))
    else:
        driver = make_driver(a.sim, cfg)
    svc = ArmService(driver, cfg, ws, a.speed or cfg.speed_pct_default)
    svc.config_dir = Path(a.config).parent  # 캘리브레이션 사본과 팔 파일을 robot.yaml 옆에서 찾는다
    ok, msg = svc.connect()
    if not ok:
        sys.exit(f"연결 실패: {msg}")
    if a.free or a.unsafe or a.safety_margin is not None or a.box:
        kw = {"mode": "off" if a.unsafe else "free"} if (a.unsafe or a.free) else {}
        if a.safety_margin is not None:
            kw["joint_margin_deg"] = a.safety_margin
        if a.box:
            n = [None if x in ("", "none") else float(x) for x in a.box.split(",")]
            if len(n) != 6:
                ap.error("--box는 x최소,x최대,y최소,y최대,z최소,z최대 (mm, 'none'이면 제한 없음)")
            kw["box_mm"] = {"x": n[0:2], "y": n[2:4], "z": n[4:6]}
        try:
            svc.set_safety(**kw)
        except ValueError as e:
            sys.exit(f"안전 설정 오류: {e}")
        if a.unsafe:
            print("경고: 관절 한계·작업 영역·바닥 검사를 모두 끕니다. 한 주기 이동량 한계와 과열·과부하 감시만 남습니다. 속도 30% 이하, 전원 스위치 옆에서.")
        elif a.free:
            print("작업대 가장자리 검사를 끕니다 (바닥과 관절 한계는 유지). 팔이 작업대 밖으로 나갈 수 있으니 주변을 비우세요.")
    try:
        c, v = a.command, a.values
        if c == "calibrate":  # --verify-zero (REQ-017). 나머지 calibrate는 위에서 처리했다
            from ..arm.autocal.verify import run_verify_zero

            sys.exit(run_verify_zero(svc, cfg, config_dir=svc.config_dir))
        nums = lambda n: [float(x) for x in v] if len(v) == n else ap.error(f"{c}: 값 {n}개가 필요하다")
        if c == "status":
            print(status_text(svc.status()))
        elif c == "home":
            print(fmt(svc.home()))
        elif c == "park":
            print(fmt(svc.park()))
        elif c == "movej":
            print(fmt(svc.move_joints(JointState(*nums(6)))))
        elif c == "goto":
            print(fmt(svc.move_tool(ToolPose(*nums(4)))))
        elif c == "gripper":
            print(fmt(svc.set_gripper(nums(1)[0], a.width)))
        elif c == "pick":
            x, y, angle, width, *rest = nums(len(v)) if len(v) in (4, 5) else ap.error("pick: 값 4개(x y 각도 폭) 또는 5개(x y 각도 폭 높이)가 필요하다")
            r = svc.pick_and_place(DetectedObject("cli", x, y, angle, width, width, 1.0, "manual", rest[0] if rest else 0.0), a.place)
            print(f"{r.status}  마지막 동작 {r.duration_s:.2f} s")
            if r.status == "height_required":
                print(f"{r.reason}. 5번째 값으로 높이(mm)를 넣는다: pick {x:g} {y:g} {angle:g} {width:g} <높이>")
        elif c == "place":
            print(fmt(svc.place(a.place or ap.error("place: --place <구역 이름>이 필요하다"))))
        elif c == "run":
            print(run_task(svc, a))
        elif c == "checktest":
            run_checktest(svc, a)
        elif c == "powertest":
            from .powertest import run_powertest

            run_powertest(svc, int(a.values[0]) if a.values else 3, a.speed or 100.0)
        elif c == "servoinfo":
            for m, regs in svc.driver.read_servo_info().items():
                print(m, regs)
        elif c == "shaketest":
            import msvcrt

            from .shaketest import run_shaketest

            def poll_mark():  # 떨림을 느끼면 m 키 (이동이 끝난 직후 확인)
                hit = False
                while msvcrt.kbhit():
                    hit |= msvcrt.getwch().lower() == "m"
                return hit

            run_shaketest(svc, int(a.values[0]) if a.values else 10, poll_mark=None if a.sim else poll_mark,
                          accels=[int(x) for x in a.values[1:]] or None, speeds=[float(x) for x in a.speeds.split(",")] if a.speeds else None)
        elif c == "keys":
            from .keys import run_keys

            run_keys(svc, a.jog_speed, a.trace, a.cart_speed)
        elif c == "record":
            name = v[0] if v else ap.error("record: 이름이 필요하다")
            print("먼저 홈 자세(받침 근처)로 이동합니다. 토크를 끄면 팔이 처지기 때문입니다.")
            print(fmt(svc.home()))
            input("팔을 손으로 단단히 잡으세요. 준비되면 Enter (3초 뒤 토크가 꺼집니다)...")
            for n in (3, 2, 1):
                print(f"  {n}...", flush=True)
                time.sleep(1)
            svc.record_start(a.record_hz, a.record_max_s)
            input("토크가 꺼졌습니다. 손으로 천천히 움직이세요. 끝나면 Enter... ")
            print("저장:", svc.record_stop(name))
            if svc.teach.truncated:
                print(f"기록은 최대 {svc.teach.max_s:.0f}초라 앞부분만 저장했습니다.")
            print_info(svc.record_info(name))
        elif c == "info":
            print_info(svc.record_info(v[0] if v else ap.error("info: 이름이 필요하다")))
        elif c in ("replay", "seq"):
            seg = {int(k): float(x) for k, x in (p.split("=") for p in a.seg_speed.split(","))} if a.seg_speed else None
            try:
                res = svc.replay(v[0] if v else ap.error(f"{c}: 이름이 필요하다"), a.speed / 100 if a.speed else 1.0, a.until_limit, seg, a.mode,
                                 any_gripper=a.any_gripper)
            except ValueError as e:
                sys.exit(f"재생 설정 오류: {e}")
            print(fmt(res))
            if svc.teach.last_cut:
                kept, total, why = svc.teach.last_cut
                print(f"한계 앞에서 잘랐습니다: {total}점 중 {kept}점 재생 ({why})")
    finally:
        from .rest import warn_if_loaded
        warn_if_loaded(svc)  # 부하가 큰 자세로 끝나면 알린다(움직이지는 않는다)
        svc.disconnect()
