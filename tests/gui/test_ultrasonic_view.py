"""초음파 화면 (REQ-024). 가상 ESP32와 가상 팔이 낸 us.* 이벤트로 그린다."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.sensors import ultrasonic_view as uv
from so101_studio.perception.ultrasonic.model import Box, load_config

CFG = load_config()


def _sweep():
    return {"pass": 1, "dir": 1, "t_ns": 1, "angles": [10, 14, 18], "d": [400, None, 250]}


def _result(status="ok", obj=True):
    o = {"id": "us-1", "x_mm": 300.0, "y_mm": 200.0, "angle_deg": 45.0, "width_mm": 30.0, "length_mm": 100.0, "confidence": 0.8, "source": "ultrasonic"}
    return {"id": "us-1", "object": o if obj else None, "info": {"check_s": 12.34}, "status": status}


def test_log_replaces_started_path_with_measured_one():
    log = uv.UltrasonicLog()
    start = {"id": "us-1", "stage": "L1", "phase": "start", "points_mm": [(0, 0), (10, 0)], "tool_z_mm": 55, "points": [], "unreachable": False}
    log.on_event("us.check_path", start)
    log.on_event("us.check_path", {**start, "phase": "end", "points": [(5, 0, 40)]})
    log.on_event("us.check_path", {**start, "stage": "L2"})
    snap = log.snapshot()
    assert [(p["stage"], p["phase"], len(p["points"])) for p in snap["paths"]] == [("L1", "end", 1), ("L2", "start", 0)]


def test_snapshot_is_a_copy():
    log = uv.UltrasonicLog()
    log.on_event("us.sweep", _sweep())
    log.snapshot()["sweep"]["d"][0] = -1
    assert log.snapshot()["sweep"]["d"][0] == 400


def test_overlay_draws_fan_points_candidate_path_and_result():
    snap = uv.UltrasonicLog().snapshot()
    snap.update(sweep=_sweep(), masked=[50], results=[_result()],
                candidates=[{"id": "us<1", "x_mm": 300.0, "y_mm": 200.0}],
                paths=[{"id": "us-1", "stage": "L1", "phase": "end", "points_mm": [(250, 200), (350, 200)], "points": [(300, 200, 40), (260, 200, 2)], "unreachable": False}])
    svg = uv.radar_overlay(CFG, snap, 400, CFG.check.hit_mm)
    assert svg.count("<polygon") == 1 and "stroke-dasharray=\"4 6\"" in svg  # 부채꼴, 지운 각도
    assert "us&lt;1" in svg and "us<1" not in svg  # 후보 이름은 이스케이프한다
    assert svg.count("<polyline") == 1
    assert f'r="4" fill="{uv.theme.OK}"' in svg and f'r="2" fill="{uv.theme.MUTED}"' in svg  # 물체 위 점과 아닌 점
    assert "rotate(-45.0)" in svg


def test_overlay_draws_only_the_latest_candidates_paths():
    snap = uv.UltrasonicLog().snapshot()
    path = {"phase": "end", "points_mm": [(1, 1), (2, 2)], "points": [], "unreachable": False}
    snap["paths"] = [{**path, "id": "a", "stage": "L1"}, {**path, "id": "b", "stage": "L1"}, {**path, "id": "b", "stage": "L2"}]
    assert uv.radar_overlay(CFG, snap, 400, CFG.check.hit_mm).count("<polyline") == 2


def test_overlay_flips_y_and_marks_unreachable_path():
    snap = uv.UltrasonicLog().snapshot()
    snap["paths"] = [{"id": "a", "stage": "L2", "phase": "end", "points_mm": [(10, 30), (20, 30)], "points": [], "unreachable": True}]
    svg = uv.radar_overlay(CFG, snap, 400, CFG.check.hit_mm)
    assert '10.0,370.0 20.0,370.0' in svg and f'stroke="{uv.theme.DANGER}"' in svg


def test_result_without_object_and_without_direction():
    snap = uv.UltrasonicLog().snapshot()
    square = _result()
    square["object"]["angle_deg"] = None
    snap["results"] = [square, _result("empty", obj=False)]
    svg = uv.radar_overlay(CFG, snap, 400, CFG.check.hit_mm)
    assert "<line" not in svg.split("<g ")[-1]  # 방향이 없으면 축 선을 그리지 않는다
    rows = uv.result_rows(snap)
    assert (rows[0]["status"], rows[0]["angle"], rows[0]["size"], rows[0]["seconds"]) == ("확정", "없음", "100×30", "12.3")
    assert (rows[1]["status"], rows[1]["x"]) == ("물체 없음", "")


def test_chart_sorts_angles_and_keeps_gaps():
    snap = uv.UltrasonicLog().snapshot()
    snap["sweep"] = {**_sweep(), "angles": [18, 14, 10], "d": [250, None, 400]}
    snap["baseline"] = {"14": 500, "10": None}
    series = uv.chart_options(CFG, snap)["series"]
    assert series[0]["data"] == [[10, 400], [14, None], [18, 250]]
    assert series[1]["data"] == [[10, None], [14, 500]]


def test_demo_finds_an_object_and_never_writes_the_real_baseline():
    before = _baseline_stat()
    demo = uv.Demo(speed=1e6)
    demo.start([Box(300, 200, 0, 40, 80)])
    assert demo.wait(60), "가상 실행이 끝나지 않는다"
    assert demo.error is None
    snap = demo.log.snapshot()
    assert snap["baseline"] and snap["sweep"] and snap["paths"]
    found = [r for r in snap["results"] if r["object"]]
    assert found and abs(found[0]["object"]["x_mm"] - 300) < 30 and abs(found[0]["object"]["y_mm"] - 200) < 30
    assert _baseline_stat() == before


def _baseline_stat():
    from pathlib import Path

    p = Path("config/ultrasonic_baseline.json")
    return p.stat().st_mtime_ns if p.exists() else None


def test_demo_stops_when_asked():
    demo = uv.Demo(speed=1.0)
    demo.start([Box(300, 200, 0, 40, 80)])
    time.sleep(0.2)
    assert demo.running
    demo.stop()
    assert demo.wait(5)
    assert demo.error is None


def test_page_runs_the_demo_and_view_only_browser_cannot_start(backend):
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            with user.scope(marker="nav"):
                user.find("초음파").click()
            await user.should_see(marker="page-title", content="초음파")
            await user.should_see(marker="us-note")
            demo = uv.demo_of(backend)
            demo.speed = 1e6
            user.find(marker="us-start").click()
            end = time.monotonic() + 30
            while (demo.running or not demo.log.snapshot()["results"]) and time.monotonic() < end:
                await asyncio.sleep(0.05)
            assert not demo.running and demo.error is None
            await asyncio.sleep(0.4)  # 화면 타이머
            rows = next(iter(user.find(marker="us-results").elements)).rows
            assert any(r["status"] == "확정" for r in rows)
            backend.owner = "다른 브라우저"
            demo.log.clear()
            user.find(marker="us-start").click()
            await asyncio.sleep(0.1)
            assert not demo.running

    asyncio.run(go())


def test_workspace_mini_view_draws_the_radar_log(backend):
    """작업 공간 작은 화면: 가상 실행 기록이 바뀌면 다시 그린다."""
    demo = uv.demo_of(backend)
    demo.log.clear()

    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open("/")
            mini = next(iter(user.find(marker="us-mini").elements))
            assert "<polygon" in mini.content and "<polyline" not in mini.content
            demo.log.on_event("us.check_path", {"id": "us-1", "stage": "L1", "phase": "end", "points_mm": [(250, 200), (350, 200)], "points": [], "unreachable": False})
            for _ in range(20):
                if "<polyline" in mini.content:
                    break
                await asyncio.sleep(0.1)
            assert "<polyline" in mini.content

    asyncio.run(go())
    demo.log.clear()
