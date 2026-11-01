"""키보드 조종: 관절 모드(키를 누르는 동안 움직임)와 좌표 모드(x y z를 입력하면 이동), Tab으로 전환.
키 상태는 Windows GetAsyncKeyState로 읽어 누름/뗌을 정확히 안다(키 반복 지연 없음, 추가 설치 없음).
주의: 이 방식은 창과 상관없이 키를 읽으므로 관절 모드 동안 다른 창에 타이핑하면 팔이 움직인다. x로 끝낸다."""
import time

from ..arm.teleop import KEYS_JOINT
from ..service.targets import from_table, parse_target, reason_text

VK = {"tab": 0x09, "space": 0x20, "esc": 0x1B, **{c: ord(c.upper()) for c in "abcdefghijklmnopqrstuvwxyz"}}
HELP_JOINT = "[관절 모드] q/a w/s e/d r/f t/g y/h = 1~6번 관절 +/- (누르는 동안 이동)  Tab: 좌표 모드  space: 정지  x/Esc: 종료"
HELP_COORD = (
    "[좌표 모드] x y z [yaw] = 그 점으로 이동 | d dx dy dz = 지금 위치에서 상대 이동 | frame table|robot = 좌표계 | speed N(%) / jog N(°/s) = 속도 | "
    "where = 현재 위치 | j 또는 빈 줄 = 관절 모드 | x = 종료\n"
    "  table: 작업대 좌표(mm), robot: 앞/왼쪽(어깨 회전축 기준)/높이. 높이 z는 같고 그리퍼는 아래를 봅니다."
)


class KeysSession:
    def __init__(self, svc, speed_deg_s: float, key_down, read_line, out=print, tick_s: float = 0.02, frame: str = "table"):
        self.svc, self.speed, self.key_down, self.read_line, self.out = svc, speed_deg_s, key_down, read_line, out
        self.tick, self.frame, self.mode = tick_s, frame, "joint"

    # ---- 관절 모드
    def joint_mode(self) -> str:
        """다음 모드("coord" 또는 "quit")를 돌려준다."""
        self.out(HELP_JOINT)
        ctl, was_tab, jogging = self.svc.ctl, True, False  # 들어올 때 눌려 있던 Tab은 무시
        while True:
            if self.key_down("x") or self.key_down("esc"):
                ctl.jog_stop()
                return "quit"
            tab = self.key_down("tab")
            if tab and not was_tab:
                ctl.jog_stop()
                return "coord"
            was_tab = tab
            if self.key_down("space"):
                ctl.stop()
            v = [0.0] * 6
            for key, (i, d) in KEYS_JOINT.items():
                if self.key_down(key):
                    v[i] += d * self.speed
            if any(v):
                ctl.jog("joint", v)
                jogging = True
            elif jogging:  # 키를 떼면 바로 멈춘다(데드맨을 기다리지 않는다)
                ctl.jog_stop()
                jogging = False
            time.sleep(self.tick)

    # ---- 좌표 모드
    def where(self) -> str:
        st = self.svc.status()
        t, base = st.tool, self.svc.ws.base
        fx, fy = from_table(t.x_mm, t.y_mm, "robot", base)
        return (f"작업대 x={t.x_mm:.0f} y={t.y_mm:.0f} z={t.z_mm:.0f} yaw={t.yaw_deg:.0f} | "
                f"로봇 기준 앞={fx:.0f} 왼쪽={fy:.0f} z={t.z_mm:.0f} (현재 입력 좌표계: {self.frame})")

    def coord_line(self, line: str) -> str | None:
        """한 줄을 처리한다. "joint"/"quit"이면 모드 전환, 아니면 None."""
        text = line.strip()
        low = text.lower()
        if low in ("", "j", "joint"):
            return "joint"
        if low in ("x", "q", "quit", "exit"):
            return "quit"
        if low in ("h", "?", "help"):
            self.out(HELP_COORD)
        elif low == "where":
            self.out(self.where())
        elif low.startswith(("speed", "jog")):
            tok = low.split()
            try:
                if tok[0] == "speed":
                    self.svc.set_speed(float(tok[1]))
                else:
                    self.speed = float(tok[1])
            except (IndexError, ValueError):
                self.out("  사용법: speed 퍼센트 (좌표 이동) / jog 도/초 (관절 모드)")
            self.out(f"  좌표 이동 속도 {self.svc.speed:.0f}% | 관절 모드 {self.speed:.0f}°/s")
        elif low.startswith("frame"):
            arg = low.split()[1:] and low.split()[1]
            if arg in ("table", "robot"):
                self.frame = arg
            self.out(f"좌표계: {self.frame}")
        else:
            self.move_to(text)
        return None

    def move_to(self, text: str) -> None:
        svc = self.svc
        try:
            pose = parse_target(text, self.frame, svc.ws.base, svc.status().tool)
        except ValueError as e:
            self.out(f"  입력 오류: {e}")
            return
        reason, _ = svc.preview_tool(pose)
        if reason:
            self.out(f"  이동하지 않음: {reason_text(reason)}")
            return
        r = svc.move_tool(pose)
        self.out(f"  {r.status}  {r.duration_s:.1f}s  -> 작업대 ({pose.x_mm:.0f}, {pose.y_mm:.0f}, {pose.z_mm:.0f})")

    def coord_mode(self) -> str:
        self.out(HELP_COORD)
        self.out("  " + self.where())
        while True:
            nxt = self.coord_line(self.read_line("좌표> "))
            if nxt:
                return nxt

    def run(self) -> None:
        self.svc.driver.set_torque(True)
        mode = "joint"
        while mode != "quit":
            mode = self.joint_mode() if mode == "joint" else self.coord_mode()
            if mode == "coord":
                self.mode = "coord"
            elif mode == "joint":
                self.mode = "joint"


def run_keys(svc, speed_deg_s: float = 20.0) -> None:
    import ctypes
    import msvcrt

    user32 = ctypes.windll.user32

    def key_down(name: str) -> bool:
        return bool(user32.GetAsyncKeyState(VK[name]) & 0x8000)

    def flush() -> None:  # 키를 누르는 동안 콘솔 입력 버퍼에 쌓인 글자를 비운다
        while msvcrt.kbhit():
            msvcrt.getwch()

    def read_line(prompt: str) -> str:
        flush()
        return input(prompt)

    sess = KeysSession(svc, speed_deg_s, key_down, read_line)
    orig = sess.joint_mode

    def joint_mode() -> str:
        flush()
        return orig()

    sess.joint_mode = joint_mode
    sess.run()
