"""so101 doctor / find-port (decisions/0027). lerobot 없이 돈다: 이 모듈은 doctor.py, core.ports, core.config만 쓴다."""
import argparse
import json
import sys

from .core import ports as ports_mod
from .doctor import doctor, format_report


def find_port(input_fn=input, out=print, lister=None) -> list[dict]:
    """LeRobot 방식: USB를 뽑았을 때 사라진 포트를 찾는다. 사라진 포트 목록을 돌려준다."""
    lister = lister or ports_mod.list_serial_ports
    before = lister()
    out("현재 포트: " + (", ".join(p["device"] for p in before) or "없음"))
    input_fn("보드의 USB 케이블을 뽑고 Enter (전원은 그대로)...")
    gone = ports_mod.diff_ports(before, lister())
    if len(gone) == 1:
        p = gone[0]
        out(f"찾았다: {p['device']} ({ports_mod.fmt_id(p)}, 시리얼 {p['serial']}). USB를 다시 꽂는다.")
        out(f"config/local.yaml에 `port: {p['device']}`를 적거나 --port {p['device']}로 쓴다. 팔 파일의 board_serial에는 {p['serial']}를 적는다.")
    elif not gone:
        out("사라진 포트가 없다. 케이블을 뽑았는지, 다른 보드가 아닌지 확인한다.")
    else:
        out("사라진 포트가 여럿이다: " + ", ".join(p["device"] for p in gone) + ". 다른 USB 장치는 빼 두고 다시 한다.")
    input_fn("USB를 다시 꽂은 뒤 Enter...")
    return gone


def run(a) -> int:
    if a.command == "find-port":
        find_port()
        return 0
    res = doctor(probe=not (a.sim or a.no_probe), port=a.port, config_path=a.config, arm=a.arm, progress=None if a.json else print)
    # --json은 ASCII 이스케이프(기본)라 콘솔 인코딩과 상관없이 파싱된다
    print(json.dumps(res, indent=2) if a.json else format_report(res))
    return 0 if res["ok"] else 1


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="so101")
    ap.add_argument("--sim", action="store_true", help="시뮬레이션: 포트를 열지 않고 목록만 본다")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--arm", help="팔 파일 이름(config/arms/<이름>.yaml)")
    ap.add_argument("--port", help="이 포트만 점검한다")
    ap.add_argument("--json", action="store_true", help="doctor: 결과를 JSON으로 출력")
    ap.add_argument("--no-probe", action="store_true", help="doctor: 포트를 열지 않고 목록만 본다")
    ap.add_argument("command", choices=["doctor", "find-port"])
    return ap


def main(argv=None) -> None:
    sys.exit(run(build_parser().parse_args(argv)))
