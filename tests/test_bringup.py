"""다른 PC·다른 팔 브링업 (REQ-015, decisions/0027): 설정 세 겹, 팔·포트 고르기, 읽기 전용 doctor, 캘리브레이션 불일치 잠금."""
import json
from pathlib import Path

import pytest
import yaml

from so101_studio import doctor as dr
from so101_studio.bringup import find_port
from so101_studio.core import ports as ports_mod
from so101_studio.core.config import (
    ArmSelectionError,
    load_robot,
    load_robot_ex,
    resolve_arm,
)

BASE = {"port": "COM9", "calibration_id": "base", "urdf": "u.urdf", "table_z_mm": 1.0, "control_hz": 50}


def write(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


# ---- 설정 세 겹
def test_layers_override_in_order_and_missing_files_change_nothing(tmp_path):
    robot = write(tmp_path / "robot.yaml", BASE)
    assert load_robot(robot).table_z_mm == 1.0 and load_robot(robot).arm_name == ""  # arms, local이 없으면 robot.yaml만
    write(tmp_path / "arms" / "a.yaml", {"calibration_id": "cal_a", "table_z_mm": 2.0, "board_serial": "S1"})
    cfg = load_robot(robot)
    assert (cfg.calibration_id, cfg.table_z_mm, cfg.arm_name, cfg.board_serial) == ("cal_a", 2.0, "a", "S1")  # 팔 파일이 robot.yaml을 덮는다
    write(tmp_path / "local.yaml", {"port": "COM5", "table_z_mm": 3.0})
    cfg = load_robot(robot)
    assert (cfg.port, cfg.table_z_mm, cfg.calibration_id) == ("COM5", 3.0, "cal_a")  # local.yaml이 마지막으로 덮는다


def test_template_is_not_an_arm_and_unknown_keys_are_still_rejected(tmp_path):
    robot = write(tmp_path / "robot.yaml", BASE)
    write(tmp_path / "arms" / "_template.yaml", {"calibration_id": "tpl"})
    cfg, info = load_robot_ex(robot)
    assert cfg.arm_name == "" and info["arm"] is None
    write(tmp_path / "local.yaml", {"prot": "typo"})
    from draccus.utils import DecodingError

    with pytest.raises(DecodingError):
        load_robot(robot)


def test_real_config_loads_the_same_values_as_before_the_split():
    cfg, info = load_robot_ex("config/robot.yaml")
    assert info["arm"] == "so101_follower_01" and cfg.arm_name == "so101_follower_01"
    assert cfg.calibration_id == "so101_follower_01" and cfg.joint_offsets_deg == {"shoulder_pan": -3.0}
    assert cfg.joint_limits_deg["elbow_flex"] == [-96, 95] and cfg.table_z_mm == -25 and len(cfg.gripper_width_mm) == 8
    assert cfg.port == "COM3" and cfg.board_serial


# ---- 팔 고르기
def test_arm_selection_order(tmp_path):
    arms = tmp_path / "arms"
    write(arms / "a.yaml", {"board_serial": "SA"})
    write(arms / "b.yaml", {"board_serial": "SB"})
    assert resolve_arm(arms, {}, arm="b", serials=["SA"])[0] == "b"  # --arm이 먼저
    assert resolve_arm(arms, {"arm": "a"}, serials=["SB"])[0] == "a"  # local.yaml의 arm
    assert resolve_arm(arms, {}, serials=["SX", "SB"]) == ("b", "연결된 보드 시리얼이 같은 팔 파일")
    with pytest.raises(ArmSelectionError) as e:  # 못 고르면 멈추고 묻는다
        resolve_arm(arms, {}, serials=["SX"])
    assert e.value.candidates == ["a", "b"]
    with pytest.raises(ArmSelectionError):
        resolve_arm(arms, {"arm": "nope"})
    write(arms / "c.yaml", {"board_serial": "SB"})
    with pytest.raises(ArmSelectionError):  # 같은 시리얼이 둘이면 추측하지 않는다
        resolve_arm(arms, {}, serials=["SB"])


def test_single_arm_file_is_chosen_and_no_files_means_none(tmp_path):
    write(tmp_path / "arms" / "only.yaml", {})
    assert resolve_arm(tmp_path / "arms", {}) == ("only", "팔 파일이 하나뿐")
    assert resolve_arm(tmp_path / "none", {})[0] is None


# ---- 포트 고르기 (가짜 포트 목록)
CH343 = {"device": "COM3", "vid": 0x1A86, "pid": 0x55D3, "serial": "5B8E114816", "description": "CH343"}
OTHER = {"device": "COM4", "vid": 0x1A86, "pid": 0x55D3, "serial": "SN2", "description": "CH343"}
BT = {"device": "COM1", "vid": None, "pid": None, "serial": None, "description": "Bluetooth"}
ESP = {"device": "COM7", "vid": 0x303A, "pid": 0x1001, "serial": "E1", "description": "ESP32"}


def test_port_order_and_serial_match():
    pick = ports_mod.pick_port
    assert pick([CH343], "5B8E114816", explicit="COM9") == ("COM9", "--port 옵션")
    assert pick([CH343], "5B8E114816", local_port="COM8")[0] == "COM8"
    assert pick([BT, CH343, OTHER], "SN2")[0] == "COM4"  # 시리얼이 같은 보드
    with pytest.raises(ports_mod.PortChoiceError) as e:  # 등록된 보드가 안 보이면 다른 보드를 추측해 쓰지 않는다
        pick([OTHER], "5B8E114816")
    assert e.value.candidates == [OTHER]


def test_new_arm_without_serial_needs_exactly_one_usb_serial_port():
    pick = ports_mod.pick_port
    assert pick([BT, CH343], "")[0] == "COM3"  # 블루투스 가상 COM은 후보가 아니다
    with pytest.raises(ports_mod.PortChoiceError):
        pick([CH343, OTHER], "")
    with pytest.raises(ports_mod.PortChoiceError):
        pick([BT], "")
    assert pick([], "", fallback="COM3")[0] == "COM3"  # 목록을 못 읽었을 때만 robot.yaml의 port
    with pytest.raises(ports_mod.PortChoiceError):
        pick([], "")


def test_find_port_reports_the_port_that_disappeared():
    seq = iter([[BT, CH343], [BT]])
    said = []
    gone = find_port(input_fn=lambda _: "", out=said.append, lister=lambda: next(seq))
    assert [p["device"] for p in gone] == ["COM3"] and any("COM3" in s for s in said)
    seq = iter([[BT, CH343], [BT, CH343]])
    assert find_port(input_fn=lambda _: "", out=said.append, lister=lambda: next(seq)) == []


# ---- doctor: 가짜 시리얼
class FakeSerial:
    """서보 버스를 흉내 낸다. 받은 명령 코드를 모두 기록한다."""

    def __init__(self, servos: dict[int, dict[int, int]], sent: list[int]):
        self.servos, self.sent, self.buf = servos, sent, b""

    def reset_input_buffer(self):
        self.buf = b""

    def flush(self):
        pass

    def close(self):
        pass

    @staticmethod
    def _reply(sid: int, data: bytes) -> bytes:
        body = [sid, len(data) + 2, 0, *data]
        return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])

    def write(self, data: bytes):
        sid, instr, params = data[2], data[4], data[5:-1]
        self.sent.append(instr)
        if sid not in self.servos:
            self.buf = b""
        elif instr == dr.PING:
            self.buf = self._reply(sid, b"")
        elif instr == dr.READ:
            addr, size = params
            self.buf = self._reply(sid, int(self.servos[sid].get(addr, 0)).to_bytes(size, "little"))

    def read(self, n: int) -> bytes:
        out, self.buf = self.buf[:n], self.buf[n:]
        return out


def _servo(name: str, cal: dict, volt=120, temp=36) -> dict[int, int]:
    c = cal[name]
    homing = (-c["homing_offset"] | 0x800) if c["homing_offset"] < 0 else c["homing_offset"]  # 부호-크기 방식(11번째 비트)
    return {3: 777, 9: c["range_min"], 11: c["range_max"], 31: homing, 40: 0, 56: 2048, 62: volt, 63: temp, 65: 0}


@pytest.fixture
def cal():
    data, _ = dr.find_calibration("so101_follower_01", Path("config"))
    assert data
    return data


@pytest.fixture
def bus(monkeypatch, cal):
    """포트별 서보 구성과 열린 포트·보낸 명령 기록. monkeypatch로 doctor가 쓰는 포트 목록과 시리얼을 바꾼다."""
    state = {"ports": [BT, CH343], "servos": {"COM3": {c["id"]: _servo(n, cal) for n, c in cal.items()}}, "sent": [], "opened": [], "busy": set()}

    def fake_open(device):
        state["opened"].append(device)
        if device in state["busy"]:
            raise PermissionError("access denied")
        return FakeSerial(state["servos"].get(device, {}), state["sent"])

    monkeypatch.setattr(ports_mod, "list_serial_ports", lambda: state["ports"])
    monkeypatch.setattr(dr, "_open", fake_open)
    return state


def states(res):
    return {c["name"]: c["state"] for c in res["checks"]}


def test_doctor_healthy_arm_reads_only_ping_and_read(bus):
    res = dr.doctor(config_path="config/robot.yaml")
    st = states(res)
    assert res["ok"] and st["서보 응답"] == "ok" and st["전압"] == "ok" and st["캘리브레이션 대조"] == "ok" and st["팔 파일"] == "ok"
    assert set(bus["sent"]) == {dr.PING, dr.READ}  # PING과 READ 외의 명령은 나가지 않았다
    assert bus["opened"] == ["COM3"]  # 블루투스 가상 COM은 열지 않았다
    arm = res["arm"]
    assert arm["port"] == "COM3" and arm["serial"] == "5B8E114816" and len(arm["servos"]) == 6 and arm["calibration_found"] and arm["calibration_mismatch"] == []
    assert arm["servos"][0]["voltage_v"] == 12.0 and arm["servos"][0]["model"] == 777 and arm["servos"][0]["torque"] is False
    roles = {p["device"]: p["role"] for p in res["ports"]}
    assert roles == {"COM1": "unknown", "COM3": "arm"}
    json.dumps(res)  # GUI와 --json이 그대로 쓴다


def test_doctor_without_probe_never_opens_a_port(bus):
    res = dr.doctor(probe=False)
    assert bus["opened"] == [] and bus["sent"] == [] and res["arm"] is None
    assert states(res)["서보 응답"] == "skip" and {p["device"]: p["role"] for p in res["ports"]}["COM3"] == "arm"  # 보드 시리얼로 역할만 짐작


def test_doctor_no_usb_serial_port_fails_with_a_fix(bus):
    bus["ports"] = [BT]
    res = dr.doctor()
    assert not res["ok"] and states(res)["포트 목록"] == "fail" and states(res)["서보 응답"] == "fail" and bus["opened"] == []
    assert any(c["fix"] for c in res["checks"] if c["state"] == "fail")


def test_doctor_busy_port_is_reported_and_skipped(bus):
    bus["busy"].add("COM3")
    res = dr.doctor()
    assert {p["device"]: p["role"] for p in res["ports"]}["COM3"] == "busy" and states(res)["서보 응답"] == "fail" and res["arm"] is None


def test_doctor_two_candidates_are_not_guessed(bus, cal):
    bus["ports"] = [CH343, OTHER]
    bus["servos"]["COM4"] = {c["id"]: _servo(n, cal) for n, c in cal.items()}
    res = dr.doctor()
    assert states(res)["서보 응답"] == "warn" and res["arm"] is None and "--port" in next(c["fix"] for c in res["checks"] if c["name"] == "서보 응답")
    only = dr.doctor(port="COM4")  # --port를 주면 그 포트만 본다
    assert only["arm"]["port"] == "COM4" and bus["opened"] == ["COM3", "COM4", "COM4"]  # --port를 주면 COM3은 다시 열지 않는다


def test_doctor_servo_without_response_and_leader_voltage(bus, cal):
    del bus["servos"]["COM3"][4]
    res = dr.doctor()
    sv = next(c for c in res["checks"] if c["name"] == "서보 응답")
    assert sv["state"] == "warn" and "[4]" in sv["text"] and len(res["arm"]["servos"]) == 5
    bus["servos"]["COM3"] = {c["id"]: _servo(n, cal, volt=74) for n, c in cal.items()}
    assert states(dr.doctor())["전압"] == "warn"  # 리더 팔(7.4V)일 수 있다


def test_doctor_calibration_mismatch_fails(bus, cal):
    bus["servos"]["COM3"][1][31] = 5  # 호밍 오프셋이 파일과 다르다
    res = dr.doctor()
    assert not res["ok"] and states(res)["캘리브레이션 대조"] == "fail"
    assert res["arm"]["calibration_mismatch"] and "shoulder_pan" in res["arm"]["calibration_mismatch"][0]


def test_esp32_board_is_not_probed(bus):
    bus["ports"] = [CH343, ESP]
    res = dr.doctor()
    assert "COM7" not in bus["opened"] and {p["device"]: p["role"] for p in res["ports"]}["COM7"] == "esp32"


def test_packet_builder_refuses_anything_but_ping_and_read():
    assert dr.packet(1, dr.PING)[4] == 1 and dr.packet(1, dr.READ, 56, 2)[4] == 2
    for bad in (0x03, 0x04, 0x05, 0x83):  # WRITE, REG_WRITE, ACTION, SYNC_WRITE
        with pytest.raises(ValueError):
            dr.packet(1, bad, 40, 1)


def test_serial_is_opened_with_dtr_and_rts_down(monkeypatch):
    import serial

    events = []

    class Spy:
        def __setattr__(self, k, v):
            events.append((k, v))

        def open(self):
            events.append(("open", None))

    monkeypatch.setattr(serial, "Serial", Spy)
    dr.open_port("COM3")
    assert events.index(("dtr", False)) < events.index(("open", None)) and events.index(("rts", False)) < events.index(("open", None))


def test_doctor_cli_json_in_sim(capsys):
    from so101_studio.cli import main

    with pytest.raises(SystemExit) as e:
        main(["--sim", "doctor", "--json"])
    res = json.loads(capsys.readouterr().out)
    assert e.value.code == 0 and res["ok"] and set(res) == {"ok", "checks", "ports", "arm", "esp32"}


# ---- 캘리브레이션 불일치·팔 파일 없음: 움직이는 명령 거부, 읽기는 허용
def _locked_service(tmp_path, reason=None, arm_name="so101_follower_01"):
    import dataclasses

    from so101_studio.arm.sim_driver import SimDriver
    from so101_studio.core.config import load_workspace
    from so101_studio.service.arm_service import ArmService

    d = SimDriver(load_robot("config/robot.yaml").poses["home"])
    d.simulated, d.locked_reason = False, reason  # 실제 장치처럼 보이게(시뮬레이션은 잠그지 않는다)
    writes = []
    real = d.write_joints
    d.write_joints = lambda q: (writes.append(q), real(q))[1]
    cfg = dataclasses.replace(load_robot("config/robot.yaml"), arm_name=arm_name)
    svc = ArmService(d, cfg, load_workspace("config/workspace.yaml"), speed_pct=50, log_root=str(tmp_path))
    ok, msg = svc.connect()
    assert ok, msg
    return svc, d, writes


def test_calibration_mismatch_blocks_every_move_but_allows_reading(tmp_path):
    from so101_studio.core.types import JointState

    svc, d, writes = _locked_service(tmp_path, reason="캘리브레이션 불일치: x")
    try:
        assert "캘리브레이션 불일치" in svc.locked_reason
        assert svc.status().joints == d.read_joints() and svc.get_safety()  # 읽기는 된다
        before = len(writes)
        assert svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "locked"
        assert svc.home().status == "locked" and svc.set_gripper(50).status == "locked"
        assert svc.jog("joint", [5, 0, 0, 0, 0, 0]) is False
        with pytest.raises(RuntimeError):
            svc.record_start()  # 토크를 끄는 것도 서보에 쓰는 일이다
        assert len(writes) == before  # 서보에 위치를 쓰지 않았다
    finally:
        svc.disconnect()


def test_missing_arm_file_blocks_moves_on_real_hardware_but_not_in_sim(tmp_path):
    from so101_studio.core.types import JointState

    svc, _d, _w = _locked_service(tmp_path, arm_name="")
    try:
        assert "팔 파일" in svc.locked_reason and svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "locked"
    finally:
        svc.disconnect()
    ok_svc, _d2, _w2 = _locked_service(tmp_path)  # 팔 파일이 있고 잠금 사유가 없으면 움직인다
    try:
        assert ok_svc.locked_reason is None and ok_svc.move_joints(JointState(10, -80, 90, 0, 0, 3)).status == "ok"
    finally:
        ok_svc.disconnect()
