"""REQ-048 D6: 서보가 풀린 직후 '리셋(전원)인지 개별 보호인지' 판정을 이벤트 한 줄로 남긴다."""
import time

from test_blackbox import make_svc

from so101_studio.arm.reset_probe import classify
from so101_studio.core.logread import read_events

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def info(acc=10, ping=True, status=0):
    return {"ping": {n: (ping if not callable(ping) else ping(n)) for n in NAMES}, "acceleration": {n: (acc(n) if callable(acc) else acc) for n in NAMES},
            "expected_acceleration": dict.fromkeys(NAMES, 10), "status": {n: (status(n) if callable(status) else status) for n in NAMES}}


def test_acceleration_back_to_default_means_power_reset():
    v, why = classify(info(acc=254), 6)
    assert v == "power_reset" and "Acceleration" in why[0]


def test_only_servo_1_answering_means_ids_reverted_and_power_reset():
    v, why = classify(info(ping=lambda n: n == NAMES[0]), 6)
    assert v == "power_reset" and any("ID가 1로" in w for w in why)


def test_overload_bit_with_intact_registers_is_individual_protection():
    v, why = classify(info(status=lambda n: 32 if n == "shoulder_lift" else 0), 1)
    assert v == "protection" and "overload" in why[0]


def test_nothing_answering_and_nothing_to_see():
    assert classify(info(ping=False), 6)[0] == "no_response"
    assert classify(info(), 6)[0] == "unknown"


def _wait_diag(s):
    deadline = time.time() + 8
    while time.time() < deadline:
        s.logger.flush()
        hits = [e for e in read_events(s.logger.dir) if e["code"] == "ARM-TORQUE-LOSS-DIAG"]
        if hits:
            return hits[0]
        time.sleep(0.1)
    raise AssertionError("ARM-TORQUE-LOSS-DIAG가 남지 않았다")


def test_torque_loss_logs_one_diag_line_with_the_reset_verdict(tmp_path):
    s = make_svc(tmp_path)
    try:
        s.driver.set_torque(True)
        time.sleep(0.6)  # 정상 건강 읽기 몇 번(전압 표본)
        s.driver.simulated_reset = True
        s.driver._torque = False  # 서보가 한꺼번에 리셋돼 토크가 0
        e = _wait_diag(s)["data"]
        assert e["verdict"] == "power_reset" and e["n_torque_off"] == 6 and e["responded"] == 6
        assert e["voltage_last_v"] == 12.0 and e["voltage_samples_10s"] >= 1
        assert set(e["acceleration"].values()) == {254} and set(e["expected_acceleration"].values()) == {10}
        assert sum(1 for x in read_events(s.logger.dir) if x["code"] == "ARM-TORQUE-LOSS-DIAG") == 1  # 한 줄
    finally:
        s.disconnect()


def test_torque_loss_with_a_protection_bit_is_not_called_a_reset(tmp_path):
    s = make_svc(tmp_path)
    try:
        s.driver.set_torque(True)
        s.driver.simulated_status = {"shoulder_lift": 32}
        s.driver._torque = False
        e = _wait_diag(s)["data"]
        assert e["verdict"] == "protection" and e["status_names"]["shoulder_lift"] == ["overload"]
    finally:
        s.disconnect()
