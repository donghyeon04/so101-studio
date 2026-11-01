"""원격 조종 입력 어댑터 (arm-software-spec.md 6.7). 입력 장치와 상관없이 MotionController.jog 하나로 받는다."""
import math
import time

# 관절 모드: 키 -> (관절 번호, 방향). 번호 0~5 = pan, lift, elbow, wrist_flex, wrist_roll, gripper
KEYS_JOINT = {"q": (0, 1), "a": (0, -1), "w": (1, 1), "s": (1, -1), "e": (2, 1), "d": (2, -1),
              "r": (3, 1), "f": (3, -1), "t": (4, 1), "g": (4, -1), "y": (5, 1), "h": (5, -1)}
# 직교 모드(로봇 기준): 키 -> (앞, 왼쪽, 위, yaw) 방향
KEYS_CART = {"i": (1, 0, 0, 0), "k": (-1, 0, 0, 0), "j": (0, 1, 0, 0), "l": (0, -1, 0, 0),
             "u": (0, 0, 1, 0), "o": (0, 0, -1, 0), "n": (0, 0, 0, 1), "m": (0, 0, 0, -1)}
HELP = "관절: q/a w/s e/d r/f t/g y/h = 1~6번 +/-   직교: i/k 앞뒤, j/l 좌우, u/o 위아래, n/m 회전   공백: 정지   x: 종료"


def robot_to_table(v_robot: tuple[float, float, float, float], base_yaw_deg: float) -> tuple[float, float, float, float]:
    """로봇 기준(앞, 왼쪽, 위, yaw) 속도를 작업대 좌표계 속도로 바꾼다."""
    c, s = math.cos(math.radians(base_yaw_deg)), math.sin(math.radians(base_yaw_deg))
    f, left, up, yaw = v_robot
    return c * f - s * left, s * f + c * left, up, yaw


def keyboard_loop(ctl, mode: str, base_yaw_deg: float, speed: float, get_key, out=print, tick_s: float = 0.02) -> None:
    """키를 누르고 있는 동안 움직인다. 키 입력이 0.3초 끊기면 컨트롤러의 데드맨이 멈춘다.
    get_key()는 대기 없이 키 한 글자(없으면 None)를 돌려준다. 'x'로 종료."""
    out(HELP)
    while True:
        key = get_key()
        if key == "x":
            ctl.stop()
            return
        if key == " ":
            ctl.stop()
        elif mode == "joint" and key in KEYS_JOINT:
            i, d = KEYS_JOINT[key]
            v = [0.0] * 6
            v[i] = d * speed
            ctl.jog("joint", v)
        elif mode == "cartesian" and key in KEYS_CART:
            f, left, up, yaw = KEYS_CART[key]
            vx, vy, vz, vyaw = robot_to_table((f * speed, left * speed, up * speed, yaw * speed), base_yaw_deg)
            ctl.jog("cartesian", [vx, vy, vz, vyaw])
        time.sleep(tick_s)
