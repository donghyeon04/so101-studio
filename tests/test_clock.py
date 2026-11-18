"""시계 주입 ① (mujoco-spec 13.10.2, 결정 0070, REQ-082): 팔 핵심 세 파일의 시간 호출이 표대로이고, 기본 시계(WallClock)에서는 지금과 같다."""
import ast
import time
from pathlib import Path

import pytest

from so101_studio.core.clock import WallClock

SRC = Path("src/so101_studio")
# 받은 시계로 바꾸지 않고 벽시계로 두는 호출(파일, 함수, time 속성): 13.10.2 표의 "벽시계" 줄
WALL = {
    "arm/motion.py": {("_loop_body", "time_ns"),  # ring.add 기록 시각
                      ("_tick", "perf_counter_ns")},  # tap의 읽기 걸린 ns
    "arm/skills.py": set(),
    "service/arm_service.py": {("status", "time"),  # ArmStatus.t 기록 시각
                               ("_status_loop", "perf_counter"), ("_status_loop", "sleep"),  # 벽시계 전용 상태 루프 간격
                               ("_probe", "monotonic"), ("_trigger_incident", "monotonic"),  # 탐침·사고 묶음 쿨다운
                               ("_status_once", "monotonic"), ("_diagnose_torque_loss", "monotonic")},  # _volt_hist, 토크 풀림 진단 창
}


class ManualClock(WallClock):
    """시험 전용: now()는 손으로 앞당긴다(sleep·wait는 벽시계 그대로)."""

    def __init__(self, t: float = 1000.0):
        self.t = t

    def now(self) -> float:
        return self.t

    def now_ns(self) -> int:
        return int(self.t * 1e9)


def time_calls(rel: str) -> set[tuple[str, str]]:
    tree = ast.parse((SRC / rel).read_text(encoding="utf-8"))
    return {(fn.name, n.func.attr) for fn in ast.walk(tree) if isinstance(fn, ast.FunctionDef)
            for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and isinstance(n.func.value, ast.Name) and n.func.value.id == "time"}


def test_only_the_wall_clock_rows_call_time_directly():
    for rel, allowed in WALL.items():
        assert time_calls(rel) == allowed, rel


def test_wall_clock_is_the_perf_counter_axis():
    c = WallClock()
    assert abs(c.now() - time.perf_counter()) < 0.05 and abs(c.now_ns() - time.perf_counter_ns()) < 5e7
    assert c.drives_status is False
    with c.participate():
        pass


@pytest.mark.slow
def test_default_clock_service_runs_the_status_thread(tmp_path):
    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.core.config import load_robot, load_workspace
    from so101_studio.service.arm_service import ArmService

    cfg = load_robot("config/robot.yaml")
    s = ArmService(SimDriver(cfg.poses["home"]), cfg, load_workspace("tests/workspace_600x400.yaml"), log_root=str(tmp_path))
    try:
        assert isinstance(s.clock, WallClock) and s.connect()[0]
        assert s._status_thread is not None and s._status_thread.is_alive() and s.ctl.clock is s.clock
    finally:
        s.disconnect()
