"""python -m so101_studio.watchdog [--port 8090] [--hotkey F9] [--sim] [--run-dir 실행폴더]"""
import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

from ..core.config import load_robot
from ..core.logging import JsonlFile
from .core import Watchdog
from .hotkey import start_hotkey
from .server import make_server


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio.watchdog")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--hotkey", help="E-STOP 단축키 (기본: robot.yaml의 watchdog_hotkey, 없으면 F9)")
    ap.add_argument("--sim", action="store_true", help="시뮬레이션: 작동하지 않고 기록만 한다")
    ap.add_argument("--run-dir", help="실행 폴더: 이 안에 watchdog.jsonl을 같은 머리(t ns, proc, pid, seq, level, code, kind, data)로 쓴다 (decisions/0030)")
    ap.add_argument("--config", default="config/robot.yaml")
    a = ap.parse_args(argv)
    hotkey = a.hotkey or load_robot(a.config).watchdog_hotkey
    path = Path(a.run_dir) / "watchdog.jsonl" if a.run_dir else Path("runs") / f"watchdog-{datetime.now():%Y%m%d-%H%M%S}.jsonl"
    jl = JsonlFile(path, "watchdog")

    def log(rec: dict) -> None:
        kind = rec.get("event", "log")
        data = {k: v for k, v in rec.items() if k != "event"}
        level = "ERROR" if kind == "trip" else "WARN" if kind == "trip_ignored" else "INFO"
        jl.write(kind, data, level, f"WATCHDOG-{kind.upper()}")
        print(json.dumps({"t": time.time(), **rec}, ensure_ascii=False), flush=True)

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
        jl.close()


if __name__ == "__main__":
    main()
