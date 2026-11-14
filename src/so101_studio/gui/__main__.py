import argparse

from . import scene3d
from .app import DEFAULT_PORT, main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="python -m so101_studio.gui")
    parser.add_argument("--sim", action="store_true", help="실제 팔 없이 시뮬레이션으로 실행. 없으면 실제 팔에 연결한다")
    parser.add_argument("--port", dest="web_port", type=int, default=DEFAULT_PORT, help="웹 포트(이 입구에서만. so101.cmd gui의 --port는 서보 포트다)")
    parser.add_argument("--arm", help="팔 파일 이름(config/arms/<이름>.yaml)")
    parser.add_argument("--viser-port", type=int, default=scene3d.DEFAULT_PORT)
    parser.add_argument("--share", action="store_true", help="같은 네트워크의 다른 기기에서 보기 전용으로 보게 연다(결정 0047). 원격은 조작권이 없고 E-STOP만 누를 수 있다")
    parser.add_argument("--watchdog-port", type=int, help="감시 프로세스 포트(기본 8090, 정지 수신은 +1). 시험용 서버를 실제 팔 GUI와 같이 띄울 때")
    parser.add_argument("--sim-engine", choices=["kinematic", "mujoco"], default="kinematic", help="--sim과 함께: kinematic(기본) 또는 mujoco(물리)")
    parser.add_argument("--sim-viewer", action="store_true", help="--sim --sim-engine mujoco와 함께: MuJoCo 뷰어 창을 띄운다")
    parser.add_argument("--no-browser", action="store_true", help="띄울 때 기본 브라우저에 탭을 열지 않는다(환경 변수 SO101_GUI_NO_BROWSER=1과 같다)")
    main(parser.parse_args())
