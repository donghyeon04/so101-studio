"""CLI (arm-software-spec.md 9절). 지금은 status만."""
import argparse
from dataclasses import asdict

from ..arm.driver import ArmDriver
from ..arm.sim_driver import SimDriver
from ..core.config import load_robot


def make_driver(sim: bool, cfg):
    return SimDriver(cfg.poses.get("home")) if sim else ArmDriver(cfg.pid)


def status(driver, cfg) -> str:
    driver.connect(cfg.port, cfg.calibration_id)
    try:
        q, h = asdict(driver.read_joints()), driver.read_health()
        lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
        lines += [f"{j:14s} {q[j]:8.1f} {h['temperature_c'][j]:7.0f} {h['load_pct'][j]:7.1f}" for j in q]
        lines.append(f"voltage {h['voltage_v']:.1f} V  torque {'on' if driver.torque_enabled() else 'off'}")
        return "\n".join(lines)
    finally:
        driver.disconnect()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio")
    ap.add_argument("--sim", action="store_true", help="SimDriver로 실행")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("command", choices=["status"])
    a = ap.parse_args(argv)
    cfg = load_robot(a.config)
    print(status(make_driver(a.sim, cfg), cfg))
