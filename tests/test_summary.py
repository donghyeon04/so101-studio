"""summary.json: TaskEvent에서 물체 수·결과·단계별 시간을 모은다 (system-architecture.md 10절)."""
import json
import types

from test_task import WS, ListSource, StubSkills, obj

from so101_studio.core.types import TaskEvent
from so101_studio.service.studio import Studio
from so101_studio.task.summary import RunSummary


def ev(t, state, oid=None, result=None, **detail):
    return TaskEvent(t, state, oid, result, detail)


def test_summary_counts_stage_times_and_per_object_results():
    s = RunSummary("ultrasonic")
    rows = [{"id": "a", "reachable": True}, {"id": "b", "reachable": True}, {"id": "c", "reachable": False}]
    for e in [ev(0.0, "PREPARE"), ev(1.0, "SCAN"), ev(3.0, "DECIDE", objects=rows), ev(3.1, "PICK", "a"),
              ev(5.1, "PLACE", "a"), ev(8.1, "PLACE", "a", "ok"), ev(8.1, "PREPARE"), ev(9.0, "SCAN"), ev(10.0, "DECIDE", objects=rows[1:]),
              ev(10.1, "PICK", "b"), ev(12.1, "PICK", "b", "no_object"), ev(12.1, "DONE", result="no_objects")]:
        s.on_event(e)
    s.note("a", scan_s=1.5)
    d = s.to_dict({"end": "no_objects"})
    assert (d["source"], d["end"], d["objects"], d["succeeded"], d["failed"], d["skipped"]) == ("ultrasonic", "no_objects", 3, 1, 1, 1)
    assert d["total_s"] == 12.1
    assert d["stages_s"]["PICK"] == 4.0 and d["stages_s"]["PLACE"] == 3.0 and d["stages_s"]["SCAN"] == 3.0  # PICK은 a·b 두 번(각 2초)
    by = {r["id"]: r for r in d["per_object"]}
    assert by["a"]["result"] == "ok" and by["a"]["scan_s"] == 1.5 and by["a"]["stages_s"]["PLACE"] == 3.0
    assert by["b"]["result"] == "failed" and by["b"]["reason"] == "no_object"
    assert by["c"]["result"] == "skipped" and by["c"]["reason"] == "out_of_reach"


def test_studio_writes_summary_json_into_the_run_dir(tmp_path):
    arm = types.SimpleNamespace(ctl=StubSkills().ctl, kin=None, cfg=None, ws=WS, skills=StubSkills(), _log=lambda *a: None,
                                logger=types.SimpleNamespace(dir=tmp_path))
    st = Studio(arm, ListSource([obj(1, 300, 170), obj(2, 300, 200)]))
    st.start(blocking=True)
    d = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert d["source"] == "list" and d["end"] == "no_objects" and d["succeeded"] == 2 and d["objects"] == 2
    assert {"SCAN", "PICK", "PLACE"} <= set(d["stages_s"])


def test_stop_also_writes_summary(tmp_path):
    arm = types.SimpleNamespace(ctl=StubSkills().ctl, kin=None, cfg=None, ws=WS, skills=StubSkills(), _log=lambda *a: None,
                                logger=types.SimpleNamespace(dir=tmp_path))
    st = Studio(arm, ListSource([obj(1, 300, 170)]))
    st.start(step_mode=True)
    import time
    time.sleep(0.2)
    st.stop()
    st._thread.join(2)
    assert json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))["end"] == "STOPPED"


def test_summary_failures_never_stop_the_task(tmp_path, monkeypatch):
    arm = types.SimpleNamespace(ctl=StubSkills().ctl, kin=None, cfg=None, ws=WS, skills=StubSkills(), _log=lambda *a: None,
                                logger=types.SimpleNamespace(dir=tmp_path))
    st = Studio(arm, ListSource([obj(1, 300, 170)]))

    def boom(*a, **k):
        raise RuntimeError("기록 실패")

    monkeypatch.setattr(RunSummary, "on_event", boom)  # 구독 콜백이 예외를 내도(start가 요약을 새로 만들므로 클래스에서 바꾼다)
    monkeypatch.setattr(RunSummary, "write", boom)  # 쓰기가 OSError 말고 다른 예외를 내도
    res = st.start(blocking=True)
    assert res["end"] == "no_objects" and res["picked"] == ["obj-1"]
