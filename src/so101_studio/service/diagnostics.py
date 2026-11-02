"""진단을 함수로 (REQ-006): servoinfo, linkwatch, powertest, shaketest. GUI 장치 > 팔 페이지가 부른다."""
from ..arm.sim_driver import SimDriver

NAMES = ("servoinfo", "linkwatch", "powertest", "shaketest")
BIG_MOVES = ("powertest", "shaketest")  # 팔을 크게 움직인다: GUI는 확인 창을 띄운다


def run_diagnostic(svc, name: str, progress=None, stop=None, **kw) -> dict:
    """진단을 블로킹으로 실행해 {"name", "result", "aborted"}를 돌려준다.
    progress(text): 진행 문장을 한 줄씩 받는다. stop(): True를 돌려주면 안전한 지점에서 멈춘다(GUI는 다른 스레드에서 Event로).
    kw: powertest(repeats, speed_pct), shaketest(reps, accels, speeds), linkwatch(period_s). linkwatch는 포트를 직접 쓰므로 실제 팔은 연결을 끊은 뒤에만."""
    out, stop = progress or (lambda _: None), stop or (lambda: False)
    if name not in NAMES:
        raise ValueError(f"알 수 없는 진단: {name} (가능: {NAMES})")
    if name == "servoinfo":
        res = svc.driver.read_servo_info()
    elif name == "linkwatch":
        if isinstance(svc.driver, SimDriver):
            raise ValueError("linkwatch는 실제 팔의 포트가 필요하다 (시뮬레이션에서는 쓸 수 없다)")
        if svc._connected:
            raise ValueError("linkwatch는 포트를 직접 열므로 팔 연결을 끊은 뒤에 실행한다")
        from ..cli.linkwatch import run_linkwatch

        res = run_linkwatch(svc.cfg.port, out=out, stop=stop, **kw)
    elif name == "powertest":
        from ..cli.powertest import run_powertest

        res = run_powertest(svc, out=out, stop=stop, **kw)
    else:
        from ..cli.shaketest import run_shaketest

        res = run_shaketest(svc, out=out, stop=stop, **kw)
    return {"name": name, "result": res, "aborted": bool(stop())}
