"""ArmService·Studio를 감싸는 유일한 창구 (gui-spec 7·8절). 실행 방식, 조작권, REQ 임시 우회를 여기에 둔다."""

from ..arm.sim_driver import SimDriver
from ..core.config import load_robot, load_workspace
from ..service.arm_service import ArmService
from ..service.targets import tip_vector
from .state import EMPTY, SAFETY_TEXT, State, hottest_text
from .watchdog_link import WatchdogLink


class Backend:
    def __init__(self, speed: float | None = None, log_root: str | None = "runs", link: WatchdogLink | None = None):
        cfg = load_robot()
        self.state = State()
        self.link = link
        self._listener = None
        self.owner: str | None = None  # 조작권을 가진 브라우저. NiceGUI 이벤트 루프에서만 바꾼다
        self.arm = ArmService(SimDriver(cfg.poses.get("home")), cfg, load_workspace(), speed, log_root)
        ok, msg = self.arm.connect()
        if not ok:
            raise RuntimeError(msg)
        self.arm.status_stream(self.state.update)
        if link and link.available:  # 감시 프로세스의 정지 명령을 받는다
            from ..service.stop_listener import StopListener

            self._listener = StopListener(self.arm, link.stop_port).start()

    def claim(self, client: str) -> bool:
        """비어 있으면 조작권을 가져간다. 이 브라우저가 조작권을 가졌는지 돌려준다."""
        if self.owner is None:
            self.owner = client
            if self.link:
                self.link.want_armed = True
        return self.owner == client

    def real_allowed(self) -> bool:
        """감시 프로세스가 응답할 때만 실제 팔을 움직인다 (decisions/0026). 시뮬레이션은 상관없다."""
        return bool(self.link and self.link.alive)

    def resume(self) -> None:
        """비상 정지를 풀고 감시 프로세스에 다시 등록한다. 팔은 움직이지 않는다."""
        self.arm.resume()
        if self.link:
            self.link.rearm()

    def disconnected(self, client: str) -> None:
        """조작하던 브라우저의 연결이 끊기면 팔을 세운다(토크는 유지)."""
        if self.owner == client:
            self.owner = None
            self.arm.stop()

    def estop(self) -> None:
        self.arm.estop()

    def close(self) -> None:
        if self._listener:
            self._listener.close()
        self.arm.disconnect()
        if self.link:
            self.link.stop()

    def readout(self) -> dict[str, str]:
        """상태줄에 보일 값. 키는 상태줄 칸 이름이다."""
        arm = self.arm
        values = {
            "실행 방식": "시뮬레이션만",
            "감시": self.link.text() if self.link else "없음",
            "속도": f"{arm.speed:.0f} %",
            "안전 모드": SAFETY_TEXT.get(arm.safety.settings.mode, arm.safety.settings.mode),
            "기록 폴더": arm.logger.dir.name if arm.logger else EMPTY,
        }
        st = self.state.status
        if st is None:
            return values
        t = st.tool
        tip = tip_vector(arm.kin, t.pitch_deg, t.yaw_deg, st.joints.gripper, "tip")
        x, y, h = t.x_mm + tip[0], t.y_mm + tip[1], t.z_mm + tip[2] - arm.cfg.table_z_mm
        return values | {
            "전압": f"{st.voltage_v:.1f} V",
            "최고 온도": hottest_text(st),
            "손끝 좌표": f"x {x:4.0f}  y {y:4.0f}  높이 {h:4.0f} mm",  # 폭을 고정해 값이 바뀔 때 옆 칸이 흔들리지 않게 한다
            "토크": "켜짐" if st.torque_enabled else "꺼짐",
        }
