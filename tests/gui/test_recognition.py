"""작업 공간 인식 소스·설명 모드 (피드백 F2-e, gui-spec 12.11, ultrasonic-spec 8.9)."""

import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("nicegui")

from so101_studio.gui.backend import EXPLAIN_SPEED_PCT, US_BUILDING_TEXT


def _wait(check, timeout=10.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.05)
    return check()


def test_select_ultrasonic_sim_then_back_to_manual(backend):
    """초음파(가상)을 고르면 작업 실행 소스가 바뀌고 기록이 새 버스에 붙는다. 정답 목록(objects)은 그대로 수동이다. 수동으로 되돌릴 수 있다."""
    truth = [o.id for o in backend.objects()]
    try:
        assert backend.select_perception("sim") is None
        assert backend.perception_kind == "sim" and backend.perception_source is not backend.source
        assert backend.studio.task.source is backend.perception_source and backend.perception_source.bus is backend.studio.bus
        assert backend.us_log.snapshot()["baseline"]  # 기준값은 이벤트로 오지 않아 직접 넣는다
        assert [o.id for o in backend.objects()] == truth
        assert len(backend.perception_source.esp.boxes) == len(truth)
        assert backend.scan() == backend.objects()  # 초음파 소스는 [스캔]으로 레이더를 돌리지 않는다
        assert backend.move_lock() is None
    finally:
        assert backend.select_perception("manual") is None
    assert backend.perception_kind == "manual" and backend.perception_source is backend.source


def test_building_baseline_locks_moves_and_a_failure_stays_manual(backend, monkeypatch):
    """기준값을 재는 동안 move_lock이 막고, 만들기가 실패하면(radar_clear 실패 등) 수동으로 남는다."""
    seen = []

    def failing(arm, **kw):
        seen.append(backend.move_lock())
        raise RuntimeError("대기 자세(radar_clear)로 가지 못해 기준값을 재지 않았다")

    monkeypatch.setattr("so101_studio.service.ultrasonic.make_ultrasonic_source", failing)
    reason = backend.select_perception("sim")
    assert seen == [US_BUILDING_TEXT] and "radar_clear" in reason
    assert backend.perception_kind == "manual" and backend.move_lock() is None


def test_ultrasonic_sim_is_refused_with_a_real_arm(backend, monkeypatch):
    monkeypatch.setattr(backend, "real", SimpleNamespace(), raising=False)
    assert "실물 팔이 없을 때만" in backend.select_perception("sim")


def test_pace_waits_for_next_while_paused_and_a_stop_releases_it(backend):
    """설명 모드 고리: paused면 [다음](studio.step)까지 기다리고, 아니면 EXPLAIN_PAUSE_S 뒤 돌아온다. 정지하면 바로 풀린다."""
    backend.paused = True
    done = threading.Event()
    t = threading.Thread(target=lambda: (backend._pace("check", "L2"), done.set()), daemon=True)
    try:
        t.start()
        time.sleep(0.4)
        assert not done.is_set() and backend._flow == ("check", "L2")
        backend.studio.step()
        assert done.wait(1.0)
        done.clear()
        t = threading.Thread(target=lambda: (backend._pace("pick"), done.set()), daemon=True)
        t.start()
        time.sleep(0.3)
        assert not done.is_set()
        backend.studio.task._stop = True
        assert done.wait(1.0)
    finally:
        backend.studio.task._stop = False
        backend.paused = False


def test_explain_mode_slows_down_and_restores_the_speed(backend, monkeypatch):
    """설명 모드로 시작하면 속도 30%, 끝나면(DONE·STOPPED·FAULT) 원래 속도. 실행 중 사람이 바꾼 값이 있으면 그 값으로."""
    monkeypatch.setattr(backend.studio, "start", lambda step_mode=False: None)
    monkeypatch.setattr(backend, "task_running", lambda: False)
    backend.set_speed(55)
    backend.explain = True
    try:
        assert backend.start_task() is None
        assert backend.arm.speed == EXPLAIN_SPEED_PCT and backend.studio.task.prefetch_enabled is False
        backend.set_speed(70)
        backend.set_speed(EXPLAIN_SPEED_PCT)
        backend._on_task(SimpleNamespace(state="DONE", data={}, t=time.time()))
        assert backend.arm.speed == 70
        backend.explain = False
        assert backend.start_task() is None
        assert backend.studio.task.prefetch_enabled is True and backend.arm.speed == 70
        assert backend.studio.task.pace == backend._mark_flow  # 빠른 모드는 멈추지 않고 흐름 단계만 적는다
    finally:
        backend.explain = False
        backend._speed_before = None
        backend.set_speed(30)


@pytest.mark.slow
def test_ultrasonic_sim_fast_run_logs_candidates_checks_and_results(backend):
    """초음파(가상) 빠른 모드 자동 실행이 끝까지 돌고, 기록에 후보·확인 단계 추정·결과가 쌓인다(캡션도 흐름 단계를 말한다)."""
    try:
        assert backend.select_perception("sim") is None
        backend.set_speed(100)
        assert backend.start_task() is None
        captions = set()
        assert _wait(lambda: captions.add(backend.caption()) or not backend.task_running(), timeout=240)
        snap = backend.us_log.snapshot()
        assert snap["results"] and snap["estimates"] and snap["passes"]  # 마지막 스캔은 다 치운 뒤라 후보 0개가 맞다
        assert backend.studio.result["end"] == "no_objects" and len(backend.studio.result["picked"]) == 3
        assert {c[:1] for c in captions if c} >= {"①", "②", "③", "⑤"}  # 빠른 모드도 흐름 단계를 캡션으로(④는 쉬지 않아 곧 ⑤로 넘어간다)
    finally:
        backend.stop_task()
        backend.set_speed(30)
        backend.select_perception("manual")


def test_caption_names_each_flow_step(backend, monkeypatch):
    """캡션은 흐름 단계 ①~⑤를 쉬운 말로(gui-spec 12.11). 확인 단계 이름은 phase=start로 미리 안다."""
    monkeypatch.setattr(backend, "task_running", lambda: True)
    monkeypatch.setattr(backend, "perception_source", object())  # 초음파 소스처럼 기록을 읽게
    log = backend.us_log
    try:
        log.clear()
        backend._flow = ("idle", None)  # 같은 일꾼의 앞 시험이 남긴 흐름 단계
        log.on_event("us.sweep_progress", {"pass": 6, "total": 2})
        assert backend.caption() == "① 레이더가 돌며 거리를 잽니다 (2/2바퀴)"  # 바퀴 수는 넘치지 않는다
        log.on_event("us.candidates", {"objects": [{"id": "us-1"}, {"id": "us-2"}], "arcs": [], "masked": []})
        backend._mark_flow("scan")
        assert backend.caption() == "② 빈 작업대와 다른 곳: 후보 2개"
        log.on_event("us.check_path", {"id": "us-1", "stage": "R", "phase": "start"})
        backend._mark_flow("check", "L2")
        assert backend.caption() == "③ 팔 센서로 후보를 지나갑니다 (원)"
        log.on_event("us.check_result", {"id": "us-1", "object": {"x_mm": 212.4, "y_mm": 180.2, "angle_deg": 45.0}, "info": {}, "status": "ok"})
        backend._mark_flow("check_done")
        assert backend.caption() == "④ 위치 (212, 180), 긴 축 45°"
        backend._mark_flow("pick")
        assert backend.caption() == "⑤ 집습니다"
    finally:
        log.clear()
        backend._flow = ("idle", None)


def test_recognition_layers_are_drawn_from_the_log_and_the_radar_head_spins(backend, monkeypatch):
    """F2-f(gui-spec 12.11 3D 겹): 기록(us_log)에서 빔·후보 범위 원·확인 경로·좁혀지는 범위·결과 사각형을 /recognition/ 아래에 그린다.
    레이더 머리(scene spin: radar)는 서보 각도로 돈다. 수동으로 돌아가면 지운다."""
    import numpy as np

    from so101_studio.perception.ultrasonic.model import load_config

    cfg = load_config()
    monkeypatch.setattr(backend, "perception_source", SimpleNamespace(cfg=cfg))
    handles = backend.scene.server.scene._handle_from_node_name
    log = backend.us_log
    try:
        log.clear()
        log.on_event("us.sweep_progress", {"pass": 0, "total": 2, "dir": 1, "servo_deg": 60.0, "t_ns": 0, "angles": [50, 60], "d": [180.0, None]})
        log.on_event("us.candidates", {"objects": [{"id": "us-1", "x_mm": 210.0, "y_mm": 200.0, "search_r_mm": 90.0}], "arcs": [], "masked": []})
        log.on_event("us.check_path", {"id": "us-1", "stage": "L1", "phase": "end", "points_mm": [[180, 200], [240, 200]], "points": [[200, 200, 40.0]]})
        log.on_event("us.check_estimate", {"id": "us-1", "stage": "L2", "center": [212.0, 201.0], "spread_mm": 25.0})
        log.on_event("us.check_result", {"id": "us-1", "object": {"x_mm": 212.0, "y_mm": 201.0, "angle_deg": 30.0, "length_mm": 80.0, "width_mm": 40.0},
                                         "info": {}, "status": "ok"})
        heads = list(backend.scene._spin)
        before = [np.asarray(h.wxyz).copy() for h, _ in heads]
        backend.draw_recognition()
        names = set(handles)
        assert {"/recognition/beam", "/recognition/echo", "/recognition/cand/us-1", "/recognition/est/us-1", "/recognition/result/us-1"} <= names
        assert any(n.startswith("/recognition/path/us-1/") for n in names)
        assert heads and any(not np.allclose(np.asarray(h.wxyz), b) for (h, _), b in zip(heads, before, strict=True))  # 60°면 정면(90°)에서 돈다
        monkeypatch.setattr(backend, "perception_source", backend.source)
        backend.draw_recognition()
        assert not any(n.startswith("/recognition/") for n in handles)
    finally:
        log.clear()
        backend._drawn_version = None
