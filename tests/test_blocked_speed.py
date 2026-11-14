"""결정 0061: 막힘 감지 문턱은 30% 이하 그대로, 30~50%는 정속만 s/0.3배, 50% 초과는 판정하지 않고 ARM-TRACKING만 남긴다."""
import pytest

from so101_studio.arm.motion import (
    BLOCKED_FULL_RATIO,
    BLOCKED_OFF_RATIO,
    blocked_threshold,
)
from so101_studio.core.config import load_robot

CFG = load_robot("config/robot.yaml")


@pytest.mark.parametrize("s", [0.1, 0.2, 0.30])
def test_up_to_30_percent_the_thresholds_are_unchanged(s):
    assert blocked_threshold(CFG, "empty", False, s) == pytest.approx(CFG.blocked_empty_cruise_deg)
    assert blocked_threshold(CFG, "held", False, s) == pytest.approx(CFG.blocked_held_cruise_deg)
    assert blocked_threshold(CFG, "empty", True, s) == CFG.blocked_empty_accel_deg and blocked_threshold(CFG, "held", True, s) == CFG.blocked_held_accel_deg


@pytest.mark.parametrize("s", [0.31, 0.40, 0.50])
def test_between_30_and_50_percent_only_the_cruise_threshold_scales(s):
    assert blocked_threshold(CFG, "empty", False, s) == pytest.approx(CFG.blocked_empty_cruise_deg * s / BLOCKED_FULL_RATIO)
    assert blocked_threshold(CFG, "held", False, s) == pytest.approx(CFG.blocked_held_cruise_deg * s / BLOCKED_FULL_RATIO)
    assert blocked_threshold(CFG, "empty", True, s) == CFG.blocked_empty_accel_deg and blocked_threshold(CFG, "held", True, s) == CFG.blocked_held_accel_deg  # 가속 문턱은 그대로


def test_the_documented_values_at_50_percent():
    assert blocked_threshold(CFG, "held", False, 0.5) == pytest.approx(15.0) and blocked_threshold(CFG, "empty", False, 0.5) == pytest.approx(25.83, abs=0.01)


@pytest.mark.parametrize("s", [0.51, 0.6, 1.0])
def test_above_50_percent_nothing_is_judged(s):
    for prof in ("empty", "held"):
        for zone in (True, False):
            assert blocked_threshold(CFG, prof, zone, s) is None
    assert BLOCKED_OFF_RATIO == 0.5 and BLOCKED_FULL_RATIO == 0.3
