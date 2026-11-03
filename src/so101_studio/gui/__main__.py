import argparse

from . import scene3d
from .app import DEFAULT_PORT, main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="python -m so101_studio.gui")
    parser.add_argument("--sim", action="store_true", help="실제 팔 없이 시뮬레이션으로 실행. 없으면 실제 팔에 연결한다")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--arm", help="팔 파일 이름(config/arms/<이름>.yaml)")
    parser.add_argument("--viser-port", type=int, default=scene3d.DEFAULT_PORT)
    main(parser.parse_args())
