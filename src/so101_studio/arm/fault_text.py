"""fault 한 건을 사람이 읽을 문장(원인과 해결법)으로 (REQ-034, 결정 0042). 이벤트·MotionResult.reason·상태가 같은 문장을 쓴다."""

SAFE = "팔은 현재 위치를 유지합니다(토크는 끄지 않음)."


def describe(reason: str, detail: dict | None = None) -> str:
    d = detail or {}
    name, _, joint = reason.partition(":")
    joint = d.get("joint", joint)
    if name == "step":
        return (f"{joint}가 한 제어 주기에 {d.get('delta_deg', 0):.1f}°(한계 {d.get('limit_deg', 0):.1f}°)를 넘게 움직이려 했습니다. "
                "목표가 너무 멀리 있거나 서보가 따라오지 못한 것입니다. " + SAFE + " '재개' 뒤 더 느린 속도로 다시 하세요.")
    if name == "comm_lost":
        return ("서보와의 통신이 0.5초 넘게 끊겼습니다. USB 케이블, 12V 전원 어댑터, 서보 커넥터를 확인한 뒤 팔을 다시 연결하세요. "
                "다시 연결하기 전에는 어떤 명령도 보내지 않습니다.")
    if name == "overload":
        return (f"{joint} 부하 {d.get('load_pct', 0):.0f}%가 {d.get('limit_pct', 0):.0f}%를 {d.get('seconds', 0):.0f}초 넘게 이어졌습니다. "
                "물체에 끼었거나 무리한 자세입니다. 막힌 것을 치우고 '재개' 하세요. " + SAFE)
    if name == "overheat":
        return (f"{joint} 온도 {d.get('temp_c', 0):.0f}°C가 한계 {d.get('limit_c', 0):.0f}°C에 닿았습니다. "
                "전원을 유지한 채 쉬게 해 식은 뒤 다시 하세요. " + SAFE)
    if name == "overcurrent":
        return (f"서보 전류 합 {d.get('current_ma', 0) / 1000:.1f}A가 {d.get('limit_ma', 0) / 1000:.1f}A를 넘었습니다(어댑터 5A 보호 직전). "
                "속도를 낮추거나 팔에 걸린 부하를 줄이세요. " + SAFE)
    if name == "grip_overload":
        return f"그리퍼 부하 {d.get('load_pct', 0):.0f}%가 한계 {d.get('limit_pct', 0):.0f}%를 넘어 그리퍼를 열었습니다. 물체를 너무 세게 쥐고 있었습니다."
    if name == "internal_error":
        return (f"제어 루프에서 통신이 아닌 예기치 않은 오류가 났습니다({d.get('error', '')}). 프로그램 문제일 수 있으니 사고 묶음의 traceback을 개발 세션에 알리세요. "
                + SAFE + " '재개' 하면 다시 시도합니다.")
    if name == "workspace":
        return "작업대 밖입니다(경계 안 여유 안쪽만 가능). 목표를 작업대 안쪽으로 옮기세요."
    if name == "z_floor":
        return "너무 낮습니다(낮은 턱 끝이 작업대에 닿음). 목표 높이를 올리세요."
    if name == "z_ceiling":
        return "너무 높습니다. 목표 높이를 낮추세요."
    if name == "joint_limit":
        return f"{joint} 관절 한계를 넘습니다. 목표 자세를 한계 안으로 옮기세요."
    if name == "locked":
        return (f"팔이 잠겨 읽기만 됩니다: {d.get('locked_reason') or '캘리브레이션 불일치 또는 팔 파일 없음'}. "
                "doctor로 캘리브레이션을 대조하고 맞춘 뒤 다시 연결하세요.")
    return f"fault: {reason}"
