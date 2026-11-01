"""CLI (arm-software-spec.md 9절). status, home, park, movej."""
import argparse
from dataclasses import asdict

from ..arm.driver import ArmDriver
from ..arm.motion import MotionController
from ..arm.safety import Safety
from ..arm.sim_driver import SimDriver
from ..arm.trajectory import move_j
from ..core.config import load_robot, load_workspace
from ..core.types import JointState


def make_driver(sim: bool, cfg):
    return SimDriver(cfg.poses.get("home")) if sim else ArmDriver(cfg.pid)


def status(driver, cfg) -> str:
    q, h = asdict(driver.read_joints()), driver.read_health()
    lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
    lines += [f"{j:14s} {q[j]:8.1f} {h['temperature_c'][j]:7.0f} {h['load_pct'][j]:7.1f}" for j in q]
    lines.append(f"voltage {h['voltage_v']:.1f} V  torque {'on' if driver.torque_enabled() else 'off'}")
    return "\n".join(lines)


def move(driver, cfg, goal: JointState, speed_pct: float) -> str:
    from ..arm.kinematics import Kinematics

    ws = load_workspace()
    kin = Kinematics(cfg.urdf, ws.base)
    ctl = MotionController(driver, kin, Safety(cfg.joint_limits_deg, ws, kin), cfg.control_hz)
    try:
        traj = move_j(driver.read_joints(), goal, cfg.joint_v_max_deg_s * speed_pct / 100, cfg.a_max_deg_s2)
        driver.set_torque(True)  # 현재 위치를 목표로 쓴 뒤 켜므로 튀지 않는다
        r = ctl.execute(traj).wait()
        err = "-" if r.position_error_mm is None else f"{r.position_error_mm:.1f} mm"
        return f"{r.status}  {r.duration_s:.2f} s (계획 {traj.duration_s:.2f} s)  끝점 오차 {err}"
    finally:
        ctl.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio")
    ap.add_argument("--sim", action="store_true", help="SimDriver로 실행")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--speed", type=float, help="속도 %% (기본: 설정값)")
    ap.add_argument("command", choices=["status", "home", "park", "movej"])
    ap.add_argument("values", nargs="*", type=float, help="movej: 관절 6개 값(도, gripper 0~100)")
    a = ap.parse_args(argv)
    cfg = load_robot(a.config)
    driver = make_driver(a.sim, cfg)
    driver.connect(cfg.port, cfg.calibration_id)
    try:
        if a.command == "status":
            print(status(driver, cfg))
            return
        vals = cfg.poses[a.command] if a.command in ("home", "park") else a.values
        if len(vals) != 6:
            ap.error("관절값 6개가 필요하다")
        print(move(driver, cfg, JointState(*vals), a.speed or cfg.speed_pct_default))
    finally:
        driver.disconnect()
