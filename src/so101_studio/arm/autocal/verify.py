"""`so101 calibrate --verify-zero` (decisions/0029): 속도 20%로 전 관절을 0°로 보내고, 실물과 3D(URDF)의 0° 자세를 사람이 비교해 확인하면
팔 파일에 calibration_checked: {date, sha256}를 쓴다. 이 기록이 없거나 해시가 지금 캘리브레이션 파일과 다르면 움직이는 명령은 locked다.
이 명령만 그 잠금에서 예외다(캘리브레이션 불일치·팔 파일 없음 잠금은 풀지 않는다)."""
from datetime import date
from pathlib import Path

from ...arm.kinematics import VALUE_TO_Q
from ...core.calibration import record_checked, zero_check
from ...core.types import JointState

SPEED_PCT = 20
# 그리퍼는 각도가 아니라 값(0~100)이다. URDF의 그리퍼 관절 0°에 해당하는 값(kinematics.VALUE_TO_Q: q = -10 + 1.214 v, 카드 시험에 맞춘 가정)
GRIPPER_ZERO = -VALUE_TO_Q[0] / VALUE_TO_Q[1]

GUIDE = f"""0° 자세 확인: 모든 관절을 속도 {SPEED_PCT}%로 0°로 보낸다.
  1. 팔의 받침을 클램프로 책상에 고정하고, 팔이 닿는 반경 50cm를 비운다. 0° 자세는 팔이 앞으로 곧게 뻗어 작업대 가장자리 밖으로 나간다.
  2. 전원 스위치 옆에 사람이 있다. 이상하면 바로 끈다.
  3. 시작 전에 팔을 손으로 받쳐 둘 필요는 없지만 지금 자세에서 0°까지 크게 움직일 수 있다."""

COMPARE = """팔이 0° 자세로 섰다. 실물과 3D(URDF) 모델의 0° 자세를 비교한다:
  - GUI의 3D 화면(so101 gui)에서 실제 팔과 같은 자세인지, 또는 assets/so101_new_calib.urdf의 모든 관절 0° 자세와 같은지 본다.
  - 관절마다 방향과 기울기가 같아야 한다(특히 shoulder_pan의 정면, wrist_roll, gripper)."""


def run_verify_zero(svc, cfg, *, input_fn=input, out=print, config_dir: Path | None = None, today=date.today) -> int:
    config_dir = config_dir or svc.config_dir
    if not cfg.arm_name:
        out("팔 파일이 없어 확인을 기록할 곳이 없다. config/arms/_template.yaml을 복사해 팔 파일을 만든다")
        return 1
    if not svc.unlock_for_verification():
        out(f"움직일 수 없다: {svc.locked_reason}")
        return 1
    out(GUIDE)
    input_fn("준비됐으면 Enter(시작), 그만두려면 Ctrl+C... ")
    try:
        # 0° 자세는 팔이 앞으로 곧게 뻗어 끝이 작업대 가장자리 밖으로 나간다(약 y 400mm, 작업대는 390mm까지). 이 이동 동안만 작업대 가장자리 검사를 끈다
        # (관절 한계와 바닥 검사는 그대로, powertest와 같다). 그래서 반경 50cm를 비우라고 안내한다
        with svc.safety.relaxed(check_xy=False):
            r = svc.move_joints(JointState(0, 0, 0, 0, 0, GRIPPER_ZERO), speed_pct=SPEED_PCT)
        if r.status != "ok":
            out(f"0° 자세로 가지 못했다: {r.status}. 기록하지 않았다")
            return 1
        out(COMPARE)
        if input_fn("실물이 3D의 0° 자세와 같으면 yes를 입력한다(아니면 아무 키나): ").strip().lower() != "yes":
            out("확인하지 않았다. 기록하지 않았으므로 움직이는 명령은 계속 locked다. 어긋나면 캘리브레이션(lerobot-calibrate나 calibrate --auto)을 다시 한다")
            return 1
        sha = zero_check(cfg, config_dir)["sha256"]
        if not sha:
            out("캘리브레이션 파일을 찾지 못해 기록하지 않았다")
            return 1
        arm_file = config_dir / "arms" / f"{cfg.arm_name}.yaml"
        record_checked(arm_file, sha, today())
        out(f"기록했다: {arm_file} calibration_checked (캘리브레이션 파일 해시 {sha[:12]}…). 다음에 연결할 때부터 움직인다")
        return 0
    finally:
        svc.relock_zero()
