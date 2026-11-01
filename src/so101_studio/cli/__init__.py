"""CLI (arm-software-spec.md 9절). 모든 명령은 ArmService를 거친다."""
import argparse
import sys
import time
from dataclasses import asdict, replace

from ..arm.driver import ArmDriver
from ..arm.sim_driver import SimDriver
from ..arm.teach import MAX_RECORD_S
from ..core.config import load_robot, load_workspace
from ..core.types import DetectedObject, JointState, ToolPose
from ..service.arm_service import ArmService, plan_goto  # noqa: F401  plan_goto는 테스트가 이 경로로도 쓴다

COMMANDS = ["status", "home", "park", "movej", "goto", "pick", "place", "gripper", "run", "jog", "keys", "powertest", "shaketest", "servoinfo", "linkwatch", "record", "replay", "seq"]


def make_driver(sim: bool, cfg):
    return SimDriver(cfg.poses.get("home")) if sim else ArmDriver(cfg.pid, cfg.joint_offsets_deg, cfg.servo_acceleration, cfg.gripper_acceleration)


def status_text(st) -> str:
    q = asdict(st.joints)
    lines = [f"{'joint':14s} {'pos':>8s} {'temp C':>7s} {'load %':>7s}"]
    lines += [f"{j:14s} {q[j]:8.1f} {st.temperature_c.get(j, 0):7.0f} {st.load_pct.get(j, 0):7.1f}" for j in q]
    lines.append(f"voltage {st.voltage_v:.1f} V  torque {'on' if st.torque_enabled else 'off'}")
    return "\n".join(lines)


def fmt(r) -> str:
    err = "-" if r.position_error_mm is None else f"{r.position_error_mm:.1f} mm"
    return f"{r.status}  {r.duration_s:.2f} s  끝점 오차 {err}"


def run_task(svc: ArmService, a) -> str:
    from ..perception.manual import ManualSource
    from ..service.studio import Studio

    st = Studio(svc, ManualSource(), dry_run=a.dry_run)
    st.bus.subscribe("task", lambda e: print(f"[{e.state}] {e.object_id or ''} {e.result or ''}".rstrip()))
    if a.step:
        st.task.wait_step = lambda: input("  (Enter: 다음 단계) ")
    svc.driver.set_torque(True)
    res = st.start(step_mode=a.step, blocking=True)
    return f"끝: {res['end']}  옮긴 물체 {len(res['picked'])}개 {res['picked']}  실패 {res['failed']}  사이클 {res['cycles']}"


def run_jog(svc: ArmService, a) -> None:
    import msvcrt

    from ..arm.teleop import keyboard_loop

    def get_key():
        return msvcrt.getwch() if msvcrt.kbhit() else None

    mode = "cartesian" if a.cartesian else "joint"
    svc.driver.set_torque(True)
    keyboard_loop(svc.ctl, mode, svc.ws.base.yaw_deg, 20.0 if mode == "joint" else 40.0, get_key)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="so101_studio")
    ap.add_argument("--sim", action="store_true", help="SimDriver로 실행")
    ap.add_argument("--config", default="config/robot.yaml")
    ap.add_argument("--speed", type=float, help="속도 %% (기본: 설정값)")
    ap.add_argument("--place", help="pick 뒤에 놓을 배출 구역 이름 (place 명령은 필수)")
    ap.add_argument("--width", action="store_true", help="gripper: 값을 그리퍼 값이 아니라 벌림 폭(mm)으로 해석")
    ap.add_argument("--margin", type=float, help="pick: gripper_margin_mm를 이번만 바꿔 시험")
    ap.add_argument("--tilt", type=float, help="pick: 최대 기울기(°)를 이번만 바꿔 시험 (0이면 기울이지 않음)")
    ap.add_argument("--max-open", type=float, help="pick: 최대로 벌릴 틈(mm)을 이번만 바꿔 시험")
    ap.add_argument("--bias", type=float, help="pick: grasp_bias_mm를 이번만 바꿔 시험 (가동 턱 쪽이 +)")
    ap.add_argument("--source", default="manual", help="run: 인식 방식 (지금은 manual)")
    ap.add_argument("--dry-run", action="store_true", help="run: 물체 없이 동작만 (접촉이 없어도 성공으로 보고 이어간다)")
    ap.add_argument("--step", action="store_true", help="run: 상태가 바뀔 때마다 Enter를 기다린다")
    ap.add_argument("--cartesian", action="store_true", help="jog: 직교 모드(앞뒤·좌우·위아래)")
    ap.add_argument("--jog-speed", type=float, default=30.0, help="keys: 관절 모드 이동 속도 (°/s)")
    ap.add_argument("--cart-speed", type=float, default=80.0, help="keys: 직교 조그 속도 (mm/s, 60 넘게도 가능)")
    ap.add_argument("--hz", type=int, help="제어 주기 Hz를 이번만 바꿔 시험 (기본: 설정값 50). 서보 목표를 더 자주 보내 떨림이 줄어드는지 보는 용도")
    ap.add_argument("--unsafe", action="store_true", help="관절 한계·작업 영역·바닥 검사를 모두 끈다 (한 주기 이동량 한계와 과열·과부하 감시는 유지). 매우 조심해서만")
    ap.add_argument("--safety-margin", type=float, help="팔 관절 한계를 이만큼(°) 안쪽으로 줄인다. 음수면 실제 가동 범위까지 넓힌다")
    ap.add_argument("--box", help="허용 범위(작업대 좌표 mm): x최소,x최대,y최소,y최대,z최소,z최대. z는 작업대 면 위 낮은 턱 끝 높이. 'none'이면 제한 없음")
    ap.add_argument("--free", action="store_true", help="작업대 가장자리 검사를 끈다 (바닥과 관절 한계는 유지). 손으로 움직이는 조종·재생용")
    ap.add_argument("--until-limit", action="store_true", help="replay: 한계나 바닥을 처음 넘는 자세 앞까지만 재생")
    ap.add_argument("--trace", action="store_true", help="keys: 명령·실제 관절 위치를 빠르게 기록하고 떨림 요약 (runs/.../trace.csv)")
    ap.add_argument("--speeds", help="shaketest: 시험할 속도들(°/s, 쉼표로). 기본 20,60,200")
    ap.add_argument("command", choices=COMMANDS)
    ap.add_argument("values", nargs="*", help="movej: 관절 6개 / goto: x y z yaw / pick: x y 각도 폭(0이면 크기를 모름) / gripper: 값 / record·replay·seq: 이름")
    a = ap.parse_args(argv)
    cfg = load_robot(a.config)
    cfg = replace(
        cfg,
        gripper_margin_mm=cfg.gripper_margin_mm if a.margin is None else a.margin,
        grasp_tilt_deg=cfg.grasp_tilt_deg if a.tilt is None else a.tilt,
        grasp_max_open_mm=cfg.grasp_max_open_mm if a.max_open is None else a.max_open,
        grasp_bias_mm=cfg.grasp_bias_mm if a.bias is None else a.bias,
        control_hz=cfg.control_hz if a.hz is None else a.hz,
    )
    if a.command == "linkwatch":  # 서보 연결 없이 포트만 쓴다
        from .linkwatch import run_linkwatch

        run_linkwatch(cfg.port)
        return
    svc = ArmService(make_driver(a.sim, cfg), cfg, load_workspace(), a.speed or cfg.speed_pct_default)
    ok, msg = svc.connect()
    if not ok:
        sys.exit(f"연결 실패: {msg}")
    if a.free or a.unsafe or a.safety_margin is not None or a.box:
        kw = {"mode": "off" if a.unsafe else "free"} if (a.unsafe or a.free) else {}
        if a.safety_margin is not None:
            kw["joint_margin_deg"] = a.safety_margin
        if a.box:
            n = [None if x in ("", "none") else float(x) for x in a.box.split(",")]
            if len(n) != 6:
                ap.error("--box는 x최소,x최대,y최소,y최대,z최소,z최대 (mm, 'none'이면 제한 없음)")
            kw["box_mm"] = {"x": n[0:2], "y": n[2:4], "z": n[4:6]}
        try:
            svc.set_safety(**kw)
        except ValueError as e:
            sys.exit(f"안전 설정 오류: {e}")
        if a.unsafe:
            print("경고: 관절 한계·작업 영역·바닥 검사를 모두 끕니다. 한 주기 이동량 한계와 과열·과부하 감시만 남습니다. 속도 30% 이하, 전원 스위치 옆에서.")
        elif a.free:
            print("작업대 가장자리 검사를 끕니다 (바닥과 관절 한계는 유지). 팔이 작업대 밖으로 나갈 수 있으니 주변을 비우세요.")
    try:
        c, v = a.command, a.values
        nums = lambda n: [float(x) for x in v] if len(v) == n else ap.error(f"{c}: 값 {n}개가 필요하다")
        if c == "status":
            print(status_text(svc.status()))
        elif c == "home":
            print(fmt(svc.home()))
        elif c == "park":
            print(fmt(svc.park()))
        elif c == "movej":
            print(fmt(svc.move_joints(JointState(*nums(6)))))
        elif c == "goto":
            print(fmt(svc.move_tool(ToolPose(*nums(4)))))
        elif c == "gripper":
            print(fmt(svc.set_gripper(nums(1)[0], a.width)))
        elif c == "pick":
            x, y, angle, width = nums(4)
            r = svc.pick_and_place(DetectedObject("cli", x, y, angle, width, width, 1.0, "manual"), a.place)
            print(f"{r.status}  마지막 동작 {r.duration_s:.2f} s")
        elif c == "place":
            print(fmt(svc.place(a.place or ap.error("place: --place <구역 이름>이 필요하다"))))
        elif c == "run":
            print(run_task(svc, a))
        elif c == "jog":
            run_jog(svc, a)
        elif c == "powertest":
            from .powertest import run_powertest

            run_powertest(svc, int(a.values[0]) if a.values else 3, a.speed or 100.0)
        elif c == "servoinfo":
            for m, regs in svc.driver.read_servo_info().items():
                print(m, regs)
        elif c == "shaketest":
            import msvcrt

            from .shaketest import run_shaketest

            def poll_mark():  # 떨림을 느끼면 m 키 (이동이 끝난 직후 확인)
                hit = False
                while msvcrt.kbhit():
                    hit |= msvcrt.getwch().lower() == "m"
                return hit

            run_shaketest(svc, int(a.values[0]) if a.values else 10, poll_mark=None if a.sim else poll_mark,
                          accels=[int(x) for x in a.values[1:]] or None, speeds=[float(x) for x in a.speeds.split(",")] if a.speeds else None)
        elif c == "keys":
            from .keys import run_keys

            run_keys(svc, a.jog_speed, a.trace, a.cart_speed)
        elif c == "record":
            name = v[0] if v else ap.error("record: 이름이 필요하다")
            print("먼저 홈 자세(받침 근처)로 이동합니다. 토크를 끄면 팔이 처지기 때문입니다.")
            print(fmt(svc.home()))
            input("팔을 손으로 단단히 잡으세요. 준비되면 Enter (3초 뒤 토크가 꺼집니다)...")
            for n in (3, 2, 1):
                print(f"  {n}...", flush=True)
                time.sleep(1)
            svc.record_start()
            input("토크가 꺼졌습니다. 손으로 천천히 움직이세요. 끝나면 Enter... ")
            print("저장:", svc.record_stop(name))
            if svc.teach.truncated:
                print(f"기록은 최대 {MAX_RECORD_S:.0f}초라 앞부분만 저장했습니다.")
        elif c in ("replay", "seq"):
            print(fmt(svc.replay(v[0] if v else ap.error(f"{c}: 이름이 필요하다"), a.speed / 100 if a.speed else 1.0, a.until_limit)))
            if svc.teach.last_cut:
                kept, total, why = svc.teach.last_cut
                print(f"한계 앞에서 잘랐습니다: {total}점 중 {kept}점 재생 ({why})")
    finally:
        svc.disconnect()
