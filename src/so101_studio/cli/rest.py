"""진단 명령(powertest, shaketest)이 끝나거나 중단되거나 실패해도 팔을 편 자세로 남기지 않게 하는 공통 부분 (REQ-047).
2026-11-07 실물 powertest가 처음 자세(쉬는 자세: 팔꿈치 96.4° > 소프트 한계 95°)로 돌아오는 이동을 한계 검사로 거부당해 편 자세에 토크를 켠 채 끝났고,
잠시 뒤 서보 과부하 보호로 토크가 꺼지며 팔이 떨어졌다."""
from ..arm.trajectory import move_j
from ..core.types import JointState

RETURN_V_DEG_S = 60.0  # 돌아올 때는 진단 속도와 상관없이 천천히
RETURN_A_DEG_S2 = 300.0
HIGH_LOAD_PCT = 40.0  # 명령이 끝날 때 이 부하 이상이면 팔이 무거운 자세로 남은 것으로 본다


def home_pose(svc) -> JointState:
    return JointState(*svc.cfg.poses["home"])


def prepare_rest(svc, out) -> JointState:
    """진단이 시작하고 끝날 자세를 돌려준다. 지금 자세가 소프트 한계 안이면 그 자세, 밖(쉬는 자세 등)이면 먼저 home으로 옮기고 home.
    한계 밖에서 출발하면 '처음 자세로 돌아가는 이동'이 한계 검사에 걸려 팔이 시험 자세에 남는다."""
    start = svc.driver.read_joints()
    if svc.safety.check_goal(start) is None:
        return start
    home = home_pose(svc)
    out(f"시작 자세가 소프트 한계 밖이라 시험 전에 home으로 옮깁니다 ({svc.safety.check_goal(start)}).")
    r = svc._run([move_j(start, home, RETURN_V_DEG_S, RETURN_A_DEG_S2)])
    if r.status != "ok":
        raise RuntimeError(f"home으로 옮기지 못해 시험하지 않았습니다: {r.status} {r.reason or ''}")
    return svc.driver.read_joints()


def go_rest(svc, rest: JointState, out) -> str:
    """시험이 어떻게 끝났든 rest로 돌아간다. 거부되거나 실패하면 home으로 대신 가고, 그것도 안 되면 큰 소리로 알린다. 마지막 이동의 상태를 돌려준다."""
    if svc.ctl.state in ("stopped", "fault"):
        out(f"경고: 컨트롤러가 {svc.ctl.state} 상태라 팔을 돌려보내지 못했습니다. 팔을 손으로 받치고 '재개'(또는 다시 연결)한 뒤 home을 실행하세요.")
        return svc.ctl.state
    here = svc.driver.read_joints()
    r = svc._run([move_j(here, rest, RETURN_V_DEG_S, RETURN_A_DEG_S2)])
    if r.status != "ok":
        home = home_pose(svc)
        out(f"처음 자세로 돌아가는 이동이 거절됐습니다 ({r.status} {r.reason or ''}). home으로 대신 갑니다.")
        r = svc._run([move_j(svc.driver.read_joints(), home, RETURN_V_DEG_S, RETURN_A_DEG_S2)])
        if r.status != "ok":
            out(f"경고: home으로도 돌아가지 못했습니다 ({r.status}). 팔이 시험 자세에 남아 있습니다. 손으로 받치고 home을 실행하세요.")
    return r.status


def warn_if_loaded(svc, out=print) -> None:
    """CLI가 끝나기 전, 서보 부하가 높은 자세(편 팔 등)에 토크를 켠 채 남으면 알린다. 움직이지 않는다(movej 같은 명령은 자세를 남기는 것이 목적이라 막지 않는다)."""
    try:
        load = svc.driver.read_health().get("load_pct", {})
    except Exception:  # noqa: BLE001 연결이 끊긴 뒤에는 읽을 수 없다. 경고만이라 조용히 넘어가도 안전에는 영향이 없다
        return
    worst = max(load.items(), key=lambda kv: kv[1], default=None)
    if worst and worst[1] >= HIGH_LOAD_PCT:
        out(f"주의: {worst[0]} 부하 {worst[1]:.0f}%인 자세로 끝납니다. 이 자세로 오래 두면 과부하 보호로 토크가 꺼져 팔이 떨어질 수 있습니다. 'so101 home' 또는 'so101 park'로 쉬는 자세로 옮기세요.")
