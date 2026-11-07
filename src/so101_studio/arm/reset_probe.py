"""서보가 한꺼번에 풀렸을 때 '전원 순간 강하로 서보가 리셋됐나, 개별 보호가 걸렸나'를 가리는 판정 (REQ-048 D6).
단서: 서보가 리셋되면 RAM 레지스터가 기본값으로 돌아간다. 우리는 연결 때 Acceleration(41번, RAM)을 설정값(10 등)으로 써 두므로 읽은 값이 다르면 리셋된 것이다.
Torque_Enable도 0으로 돌아가고, ID는 EEPROM이라 유지되어야 하지만 잠깐 전원이 끊기면 ID가 1로 돌아간다는 보고가 있다(Spotpear 포럼, 2차 출처). 개별 보호면 Status 비트(전압 1·센서 2·과열 4·과전류 8·과부하 32)가 켜지고 가속은 그대로다."""

STATUS_BITS = {1: "voltage", 2: "sensor", 4: "overheat", 8: "overcurrent", 32: "overload"}


def decode_status(v: int | None) -> list[str]:
    return [name for bit, name in STATUS_BITS.items() if v and v & bit]


def classify(info: dict, n_torque_off: int) -> tuple[str, list[str]]:
    """info = ArmDriver.probe_reset() 결과. (판정, 이유들). 판정: power_reset | protection | no_response | unknown."""
    reasons, names = [], list(info["ping"])
    responded = [n for n in names if info["ping"][n]]
    acc_reset = [n for n in names if info["acceleration"].get(n) is not None and info["acceleration"][n] != info["expected_acceleration"].get(n)]
    flags = {n: decode_status(info["status"].get(n)) for n in names}
    flagged = [n for n in names if flags[n]]
    if acc_reset:
        reasons.append(f"Acceleration이 설정값이 아님(리셋 흔적): {acc_reset}")
    if responded == [names[0]] and len(names) > 1:
        reasons.append("1번만 응답하고 나머지가 응답하지 않음(ID가 1로 돌아간 모양)")
    if flagged:
        reasons.append(f"Status 비트: { {n: flags[n] for n in flagged} }")
    if not responded:
        return "no_response", ["어떤 서보도 응답하지 않음(USB·전원 차단 또는 통신 끊김)"] + reasons
    if acc_reset or (responded == [names[0]] and len(names) > 1):
        return "power_reset", reasons
    if flagged and n_torque_off <= max(2, len(names) // 2):
        return "protection", reasons + [f"토크가 풀린 서보 {n_torque_off}개, 리셋 흔적 없음"]
    if flagged:
        return "protection", reasons + [f"Status 비트가 켜져 있고 리셋 흔적은 없음(풀린 서보 {n_torque_off}개)"]
    return "unknown", reasons + ["리셋 흔적도 보호 비트도 없음"]
