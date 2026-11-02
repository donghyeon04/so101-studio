"""python -m so101_studio.watchdog [--port 8090] [--hotkey F9] [--sim]"""
import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

from ..core.config import load_robot
from .core import Watchdog
from .hotkey import start_hotkey
from .server import make_server


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio.watchdog")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--hotkey", help="E-STOP 단축키 (기본: robot.yaml의 watchdog_hotkey, 없으면 F9)")
    ap.add_argument("--sim", action="store_true", help="시뮬레이션: 작동하지 않고 기록만 한다")
    ap.add_argument("--config", default="config/robot.yaml")
    a = ap.parse_args(argv)
    hotkey = a.hotkey or load_robot(a.config).watchdog_hotkey
    log_dir = Path("runs")
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"watchdog-{datetime.now():%Y%m%d-%H%M%S}.jsonl"

    def log(rec: dict) -> None:
        line = json.dumps({"t": time.time(), **rec}, ensure_ascii=False)
        print(line, flush=True)
        with path.open("a", encoding="utf-8") as f:
            f.write(line + "\n")

    wd = Watchdog(sim=a.sim, log=log)
    stop = threading.Event()
    srv = make_server(wd, a.port)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=wd.run, args=(stop,), daemon=True).start()
    hk = start_hotkey(hotkey, lambda: wd.trip(f"단축키 {hotkey}"), stop)
    log({"event": "start", "port": a.port, "sim": a.sim, "hotkey": hotkey if hk else None, "log": str(path)})
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        srv.shutdown()


if __name__ == "__main__":
    main()
