import argparse

from .app import DEFAULT_PORT, main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="python -m so101_studio.gui")
    parser.add_argument("--sim", action="store_true", help="실제 팔 없이 시뮬레이션으로 실행")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    main(parser.parse_args())
