"""키보드 조종: 관절 모드, 직교 조그 모드(누르는 동안 움직임), 좌표 모드(x y 높이를 입력하면 이동). Tab으로 돌아가며 전환.
키 상태는 Windows GetAsyncKeyState로 읽어 누름/뗌을 정확히 안다(키 반복 지연 없음, 추가 설치 없음).
주의: 이 방식은 창과 상관없이 키를 읽으므로 키 모드 동안 다른 창에 타이핑하면 팔이 움직인다. x로 끝낸다."""
import time

import numpy as np

from ..arm.teleop import KEYS_CART, KEYS_JOINT, robot_to_table
from ..service.targets import FRAMES, from_table, parse_target, reason_text, tip_vector

VK = {"tab": 0x09, "space": 0x20, "esc": 0x1B, **{c: ord(c.upper()) for c in "abcdefghijklmnopqrstuvwxyz"}}
HELP_JOINT = "[관절 모드] q/a w/s e/d r/f t/g y/h = 1~6번 관절 +/- (누르는 동안 이동)  Tab: 직교 조그  space: 정지  x/Esc: 종료"
HELP_JOG = (
    "[직교 조그 {frame}] i/k 앞뒤, j/l 좌우, u/o 위아래, n/m 회전 (누르는 동안 이동)  f: 로봇 기준 <-> 툴 기준  Tab: 좌표 모드  space: 정지  x/Esc: 종료\n"
    "  툴 기준: i/k = 도구 앞(+z, 수직 자세에서는 아래)/뒤, j/l = 턱 벌어지는 축 x, u/o = y"
)
HELP_COORD = (
    "[좌표 모드] x y 높이 [yaw] = 그 점으로 이동 | d dx dy dz = 지금 위치에서 상대 이동 | frame table|robot|tool | "
    "tcp tip|ref | speed N(%) / jog N(°/s) / cart N(mm/s) | where | 빈 줄 = 관절 모드 | x = 종료\n"
    "  table(world): 작업대 왼쪽 아래 원점, robot(base): 어깨 회전축 원점 앞/왼쪽, tool: 상대 이동만(z = 도구 앞, x = 턱 벌어지는 축)\n"
    "  높이는 항상 작업대 면 위 mm. 입력점은 tcp: tip = 고정 턱 끝, ref = 고정 턱 안쪽 면. 그리퍼는 아래를 봅니다."
)


class KeysSession:
    def __init__(self, svc, speed_deg_s: float, key_down, read_line, out=print, tick_s: float = 0.02, frame: str = "table",
                 cart_mm_s: float = 40.0, yaw_deg_s: float = 20.0):
        self.svc, self.speed, self.key_down, self.read_line, self.out = svc, speed_deg_s, key_down, read_line, out
        self.tick, self.frame, self.tcp, self.jog_frame = tick_s, FRAMES[frame], "tip", "robot"
        self.cart, self.yaw_speed = cart_mm_s, yaw_deg_s

    # ---- 키 모드 공통 루프: 종료/Tab/정지 처리를 하고, 누른 키로 속도를 만들어 apply에 넘긴다
    def _hold_loop(self, next_mode: str, keys: dict, apply, extra=None) -> str:
        ctl, was_tab, jogging = self.svc.ctl, True, False  # 들어올 때 눌려 있던 Tab은 무시
        while True:
            if self.key_down("x") or self.key_down("esc"):
                ctl.jog_stop()
                return "quit"
            tab = self.key_down("tab")
            if tab and not was_tab:
                ctl.jog_stop()
                return next_mode
            was_tab = tab
            if extra:
                extra()
            if self.key_down("space"):
                ctl.stop()
            down = [d for k, d in keys.items() if self.key_down(k)]
            if down:
                apply(down)
                jogging = True
            elif jogging:  # 키를 떼면 바로 멈춘다(데드맨을 기다리지 않는다)
                ctl.jog_stop()
                jogging = False
            time.sleep(self.tick)

    # ---- 관절 모드
    def joint_mode(self) -> str:
        self.out(HELP_JOINT)

        def apply(down):
            v = [0.0] * 6
            for i, d in down:
                v[i] += d * self.speed
            self.svc.ctl.jog("joint", v)

        return self._hold_loop("jog", KEYS_JOINT, apply)

    # ---- 직교 조그 모드
    def jog_mode(self) -> str:
        self.out(HELP_JOG.format(frame=self.jog_frame))
        state = {"f": True}
        blocked = {"why": None}

        def toggle():  # 들어올 때 눌려 있던 f는 무시하고, 누를 때마다 한 번만 바꾼다
            f = self.key_down("f")
            if f and not state["f"]:
                self.jog_frame = "tool" if self.jog_frame == "robot" else "robot"
                self.out(f"  직교 조그 기준: {self.jog_frame}")
            state["f"] = f

        def apply(down):
            fwd, left, up, yaw = (sum(d[i] for d in down) for i in range(4))
            if self.jog_frame == "tool":
                ax = self.svc.kin.tool_axes(self.svc.driver.read_joints())
                vx, vy, vz = ax @ np.array([left, up, fwd]) * self.cart
                vyaw = yaw * self.yaw_speed
            else:
                vx, vy, vz, vyaw = robot_to_table((fwd * self.cart, left * self.cart, up * self.cart, yaw * self.yaw_speed),
                                                  self.svc.ws.base.yaw_deg)
            self.svc.ctl.jog("cartesian", [vx, vy, vz, vyaw])
            why = self.svc.ctl.jog_block
            if why and why != blocked["why"]:  # 막혔을 때 이유를 한 번 알려준다
                self.out(f"  막힘: {reason_text(why)}")
            blocked["why"] = why

        return self._hold_loop("coord", KEYS_CART, apply, toggle)

    # ---- 좌표 모드
    def _tip(self, pitch: float, yaw: float):
        return tip_vector(self.svc.kin, pitch, yaw, self.svc.status().joints.gripper, self.tcp)

    def where(self) -> str:
        st = self.svc.status()
        t, base, tz = st.tool, self.svc.ws.base, self.svc.cfg.table_z_mm
        tip = tip_vector(self.svc.kin, t.pitch_deg, t.yaw_deg, st.joints.gripper, self.tcp)
        x, y, h = t.x_mm + tip[0], t.y_mm + tip[1], t.z_mm + tip[2] - tz
        fx, fy = from_table(x, y, "robot", base)
        return (f"[{self.tcp}] table x={x:.0f} y={y:.0f} 높이={h:.0f} yaw={t.yaw_deg:.0f} | "
                f"robot 앞={fx:.0f} 왼쪽={fy:.0f} 높이={h:.0f} | 입력 좌표계: {self.frame}")

    def coord_line(self, line: str) -> str | None:
        """한 줄을 처리한다. "joint"/"quit"이면 모드 전환, 아니면 None."""
        text = line.strip()
        low = text.lower()
        tok = low.split()
        if low in ("", "j", "joint"):
            return "joint"
        if low in ("x", "q", "quit", "exit"):
            return "quit"
        if low in ("h", "?", "help"):
            self.out(HELP_COORD)
        elif low == "where":
            self.out(self.where())
        elif tok[0] in ("speed", "jog", "cart"):
            try:
                val = float(tok[1])
                if tok[0] == "speed":
                    self.svc.set_speed(val)
                elif tok[0] == "jog":
                    self.speed = val
                else:
                    self.cart = val
            except (IndexError, ValueError):
                self.out("  사용법: speed 퍼센트 (좌표 이동) / jog 도/초 (관절 모드) / cart mm/초 (직교 조그)")
            self.out(f"  좌표 이동 {self.svc.speed:.0f}% | 관절 모드 {self.speed:.0f}°/s | 직교 조그 {self.cart:.0f}mm/s")
        elif tok[0] == "frame":
            name = FRAMES.get(tok[1]) if len(tok) > 1 else None
            if name:
                self.frame = name
            self.out(f"좌표계: {self.frame}")
        elif tok[0] == "tcp":
            if len(tok) > 1 and tok[1] in ("tip", "ref"):
                self.tcp = tok[1]
            self.out(f"입력점(tcp): {self.tcp}")
        else:
            self.move_to(text)
        return None

    def move_to(self, text: str) -> None:
        svc = self.svc
        st = svc.status()
        try:
            pose = parse_target(text, self.frame, svc.ws.base, st.tool, svc.cfg.table_z_mm, self._tip(-90.0, st.tool.yaw_deg),
                                svc.kin.tool_axes(st.joints))
        except ValueError as e:
            self.out(f"  입력 오류: {e}")
            return
        reason, _ = svc.preview_tool(pose)
        if reason:
            self.out(f"  이동하지 않음: {reason_text(reason)}")
            return
        r = svc.move_tool(pose)
        self.out(f"  {r.status}  {r.duration_s:.1f}s  -> {self.where()}")

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
            mode = getattr(self, mode + "_mode")()


def run_keys(svc, speed_deg_s: float = 30.0, trace: bool = False, cart_mm_s: float = 80.0) -> None:
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

    sess = KeysSession(svc, speed_deg_s, key_down, read_line, cart_mm_s=cart_mm_s)
    svc.ctl.jog_tool_max = max(svc.ctl.jog_tool_max, cart_mm_s)  # 직교 조그 속도 상한(기본 60mm/s)을 넘겨 시험할 수 있게
    for name in ("joint_mode", "jog_mode"):
        orig = getattr(sess, name)
        setattr(sess, name, lambda orig=orig: (flush(), orig())[1])
    tracer = None
    if trace:
        from .trace import Tracer

        tracer = Tracer(svc)
        tracer.start()
    try:
        sess.run()
    finally:
        if tracer:
            tracer.stop()
