"""CLI (arm-software-spec.md 9절). status, home, park, movej."""
import argparse
from dataclasses import asdict, replace

from ..arm.driver import ArmDriver
from ..arm.motion import MotionController
from ..arm.safety import Safety
from ..arm.sim_driver import SimDriver
from ..arm.skills import Skills
from ..arm.trajectory import cfg_move_j, cfg_move_l
from ..core.config import load_robot, load_workspace
from ..core.types import DetectedObject, JointState, ToolPose


def make_driver(sim: bool, cfg):
    return SimDriver(cfg.poses.get("home")) if sim else ArmDriver(cfg.pid, cfg.joint_offsets_deg, cfg.servo_acceleration)


def status(driver, cfg) -> str:
    q, h = asdict(driver.read_joints()), driver.read_health()
    lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
    lines += [f"{j:14s} {q[j]:8.1f} {h['temperature_c'][j]:7.0f} {h['load_pct'][j]:7.1f}" for j in q]
    lines.append(f"voltage {h['voltage_v']:.1f} V  torque {'on' if driver.torque_enabled() else 'off'}")
    return "\n".join(lines)


def plan_goto(kin, cfg, q0: JointState, goal: ToolPose, speed_pct: float):
    """직선(MoveL)으로 못 가면 목표 위 approach_height_mm 지점으로 MoveJ한 뒤 수직으로 내려간다(pick과 같은 순서)."""
    traj = cfg_move_l(cfg, kin, kin.fk(q0), goal, q0, speed_pct)
    if traj is not None:
        return [traj]
    hover = replace(goal, z_mm=goal.z_mm + cfg.approach_height_mm)
    q_hover = kin.ik(hover, q0)
    if q_hover is None:
        return None
    down = cfg_move_l(cfg, kin, hover, goal, q_hover, speed_pct)
    if down is None:
        return None
    return [cfg_move_j(cfg, q0, q_hover, speed_pct), down]


def run_skill(driver, cfg, a) -> str:
    from ..arm.kinematics import Kinematics

    ws = load_workspace()
    kin = Kinematics(cfg.urdf, ws.base)
    ctl = MotionController(driver, kin, Safety(cfg.joint_limits_deg, ws, kin), cfg.control_hz)
    cfg = replace(cfg, grasp_bias_mm=cfg.grasp_bias_mm if a.bias is None else a.bias,
                  gripper_margin_mm=cfg.gripper_margin_mm if a.margin is None else a.margin,
                  grasp_max_open_mm=cfg.grasp_max_open_mm if a.max_open is None else a.max_open)
    sk = Skills(ctl, kin, cfg, ws, a.speed or cfg.speed_pct_default)
    try:
        driver.set_torque(True)
        if a.command == "pick":
            x, y, angle, width = a.values
            r = sk.pick(DetectedObject("cli", x, y, angle, width, width, 1.0, "manual"))
            if r.status == "ok" and a.place:
                r = sk.place(a.place)
        else:
            r = sk.place(a.place)
        return f"{r.status}  마지막 동작 {r.duration_s:.2f} s"
    finally:
        ctl.close()


def open_gripper(driver, cfg, value: float, as_width: bool, speed_pct: float) -> str:
    """그리퍼만 움직인다. 벌림 폭(mm)을 눈으로 확인하는 데 쓴다 (예: 카드 폭 54mm)."""
    import numpy as np

    pts = sorted((mm, v) for v, mm in cfg.gripper_width_mm)
    v = float(np.interp(value, [p[0] for p in pts], [p[1] for p in pts])) if as_width else value
    lo, hi = cfg.joint_limits_deg["gripper"]
    v = min(max(v, lo), hi)
    ctl = MotionController(driver, None, Safety(cfg.joint_limits_deg), cfg.control_hz)
    try:
        driver.set_torque(True)
        q = driver.read_joints()
        r = ctl.execute(cfg_move_j(cfg, q, replace(q, gripper=v), speed_pct)).wait()
        return f"{r.status}  그리퍼 값 {v:.1f}"
    finally:
        ctl.close()


def move(driver, cfg, goal: JointState | ToolPose, speed_pct: float) -> str:
    from ..arm.kinematics import Kinematics

    ws = load_workspace()
    kin = Kinematics(cfg.urdf, ws.base)
    ctl = MotionController(driver, kin, Safety(cfg.joint_limits_deg, ws, kin), cfg.control_hz)
    try:
        q0 = driver.read_joints()
        if isinstance(goal, ToolPose):
            trajs = plan_goto(kin, cfg, q0, goal, speed_pct)
            if trajs is None:
                return "ik_fail  해가 없는 점이 있다 (움직이지 않음)"
        else:
            trajs = [cfg_move_j(cfg, q0, goal, speed_pct)]
        driver.set_torque(True)  # 현재 위치를 목표로 쓴 뒤 켜므로 튀지 않는다
        out = []
        for traj in trajs:
            r = ctl.execute(traj).wait()
            err = "-" if r.position_error_mm is None else f"{r.position_error_mm:.1f} mm"
            out.append(f"{r.status}  {r.duration_s:.2f} s (계획 {traj.duration_s:.2f} s)  끝점 오차 {err}")
            if r.status != "ok":
                break
        return "\n".join(out)
    finally:
        ctl.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio")
    ap.add_argument("--sim", action="store_true", help="SimDriver로 실행")
    ap.add_argument("--place", help="pick 뒤에 놓을 배출 구역 이름 (place 명령은 필수)")
    ap.add_argument("--width", action="store_true", help="gripper: 값을 그리퍼 값이 아니라 벌림 폭(mm)으로 해석")
    ap.add_argument("--margin", type=float, help="pick: gripper_margin_mm(물체 폭보다 더 벌리는 양)를 이번만 바꿔 시험")
    ap.add_argument("--max-open", type=float, help="pick: 최대로 벌릴 틈(mm)을 이번만 바꿔 시험")
    ap.add_argument("--bias", type=float, help="pick: grasp_bias_mm를 이번만 바꿔 시험 (가동 턱 쪽이 +)")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--speed", type=float, help="속도 %% (기본: 설정값)")
    ap.add_argument("command", choices=["status", "home", "park", "movej", "goto", "pick", "place", "gripper"])
    ap.add_argument("values", nargs="*", type=float, help="movej: 관절 6개 값(도, gripper 0~100) / goto: x y z yaw (작업대 좌표 mm, 도) / pick: x y 각도 폭")
    a = ap.parse_args(argv)
    cfg = load_robot(a.config)
    driver = make_driver(a.sim, cfg)
    driver.connect(cfg.port, cfg.calibration_id)
    try:
        if a.command == "status":
            print(status(driver, cfg))
            return
        speed = a.speed or cfg.speed_pct_default
        if a.command == "gripper":
            if len(a.values) != 1:
                ap.error("gripper: 값 1개(0~100, --width를 주면 벌림 mm)가 필요하다")
            print(open_gripper(driver, cfg, a.values[0], a.width, speed))
            return
        if a.command in ("pick", "place"):
            if a.command == "pick" and len(a.values) != 4:
                ap.error("pick: x y 각도 폭 4개가 필요하다 (작업대 좌표 mm, 도, mm)")
            if a.command == "place" and not a.place:
                ap.error("place: --place <구역 이름>이 필요하다")
            print(run_skill(driver, cfg, a))
            return
        if a.command == "goto":
            if len(a.values) != 4:
                ap.error("goto: x y z yaw 4개가 필요하다")
            print(move(driver, cfg, ToolPose(*a.values), speed))
            return
        vals = cfg.poses[a.command] if a.command in ("home", "park") else a.values
        if len(vals) != 6:
            ap.error("관절값 6개가 필요하다")
        print(move(driver, cfg, JointState(*vals), speed))
    finally:
        driver.disconnect()
