"""원격 조종 입력 어댑터 (arm-software-spec.md 6.7). 입력 장치와 상관없이 MotionController.jog 하나로 받는다."""
import math

# 관절 모드: 키 -> (관절 번호, 방향). 번호 0~5 = pan, lift, elbow, wrist_flex, wrist_roll, gripper
KEYS_JOINT = {"q": (0, 1), "a": (0, -1), "w": (1, 1), "s": (1, -1), "e": (2, 1), "d": (2, -1),
              "r": (3, 1), "f": (3, -1), "t": (4, 1), "g": (4, -1), "y": (5, 1), "h": (5, -1)}
# 직교 모드(로봇 기준): 키 -> (앞, 왼쪽, 위, yaw) 방향
KEYS_CART = {"i": (1, 0, 0, 0), "k": (-1, 0, 0, 0), "j": (0, 1, 0, 0), "l": (0, -1, 0, 0),
             "u": (0, 0, 1, 0), "o": (0, 0, -1, 0), "n": (0, 0, 0, 1), "m": (0, 0, 0, -1)}


def robot_to_table(v_robot: tuple[float, float, float, float], base_yaw_deg: float) -> tuple[float, float, float, float]:
    """로봇 기준(앞, 왼쪽, 위, yaw) 속도를 작업대 좌표계 속도로 바꾼다."""
    c, s = math.cos(math.radians(base_yaw_deg)), math.sin(math.radians(base_yaw_deg))
    f, left, up, yaw = v_robot
    return c * f - s * left, s * f + c * left, up, yaw
