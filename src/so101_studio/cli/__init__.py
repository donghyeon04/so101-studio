"""CLI (arm-software-spec.md 9절). status, home, park, movej."""
import argparse
from dataclasses import asdict, replace

from ..arm.driver import ArmDriver
from ..arm.motion import MotionController
from ..arm.safety import Safety
from ..arm.sim_driver import SimDriver
from ..arm.trajectory import move_j, move_l
from ..core.config import load_robot, load_workspace
from ..core.types import JointState, ToolPose


def make_driver(sim: bool, cfg):
    return SimDriver(cfg.poses.get("home")) if sim else ArmDriver(cfg.pid)


def status(driver, cfg) -> str:
    q, h = asdict(driver.read_joints()), driver.read_health()
    lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
    lines += [f"{j:14s} {q[j]:8.1f} {h['temperature_c'][j]:7.0f} {h['load_pct'][j]:7.1f}" for j in q]
    lines.append(f"voltage {h['voltage_v']:.1f} V  torque {'on' if driver.torque_enabled() else 'off'}")
    return "\n".join(lines)


def plan_goto(kin, cfg, q0: JointState, goal: ToolPose, speed_pct: float):
    """직선(MoveL)으로 못 가면 목표 위 approach_height_mm 지점으로 MoveJ한 뒤 수직으로 내려간다(pick과 같은 순서)."""
    v = cfg.tool_v_max_mm_s * speed_pct / 100
    traj = move_l(kin, kin.fk(q0), goal, q0, v)
    if traj is not None:
        return [traj]
    hover = replace(goal, z_mm=goal.z_mm + cfg.approach_height_mm)
    q_hover = kin.ik(hover, q0)
    if q_hover is None:
        return None
    down = move_l(kin, hover, goal, q_hover, v)
    if down is None:
        return None
    return [move_j(q0, q_hover, cfg.joint_v_max_deg_s * speed_pct / 100, cfg.a_max_deg_s2), down]


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
            trajs = [move_j(q0, goal, cfg.joint_v_max_deg_s * speed_pct / 100, cfg.a_max_deg_s2)]
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
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--speed", type=float, help="속도 %% (기본: 설정값)")
    ap.add_argument("command", choices=["status", "home", "park", "movej", "goto"])
    ap.add_argument("values", nargs="*", type=float, help="movej: 관절 6개 값(도, gripper 0~100) / goto: x y z yaw (작업대 좌표 mm, 도)")
    a = ap.parse_args(argv)
    cfg = load_robot(a.config)
    driver = make_driver(a.sim, cfg)
    driver.connect(cfg.port, cfg.calibration_id)
    try:
        if a.command == "status":
            print(status(driver, cfg))
            return
        speed = a.speed or cfg.speed_pct_default
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
