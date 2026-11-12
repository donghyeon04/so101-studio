"""설정 페이지 (gui-spec G7): 안전, 작업대·좌표, 파라미터."""

import asyncio
import pathlib
import time

import numpy as np
import pytest
import yaml

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.core.config import load_workspace
from so101_studio.gui import app as gui_app
from so101_studio.gui.backend import update_workspace_text
from so101_studio.gui.pages.params import CARDS, TIMING_TEXT

VALUES = {"table_x": 700, "table_y": 450, "base_x": 320, "base_y": 60, "base_yaw": 90, "zone_x": 520, "zone_y": 170, "margin": 12}


@pytest.fixture
def clean(backend):
    params = {k: v["value"] for k, v in backend.arm.get_params().items()}
    yield backend
    backend.arm.set_safety("normal", 0.0, None)
    backend.arm.set_params(params)
    if backend.scene:
        backend.scene.clear_target()


async def _until(check, timeout=5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        await asyncio.sleep(0.05)
    return False


def _one(user, marker):
    return next(iter(user.find(marker=marker).elements))


def in_app(backend, path, check) -> None:
    async def go():
        async with user_simulation(gui_app.make_root(backend)) as user:
            await user.open(path)
            await check(user)

    asyncio.run(go())


def test_safety_rejects_bad_values_and_changes_nothing(clean):
    before = clean.get_safety()
    assert "joint_margin" in clean.set_safety("normal", 99.0, None)
    assert clean.set_safety("normal", 0.0, {"x": [300, 100]})
    after = clean.get_safety()
    assert (after["joint_margin_deg"], after["box_mm"]) == (before["joint_margin_deg"], before["box_mm"])


def test_safety_box_appears_in_3d_with_right_size(clean):
    assert clean.set_safety("normal", 0.0, {"x": [100, 500], "y": [100, 300], "z": [10, 200]}) is None
    box = clean.scene.server.scene._handle_from_node_name["/allowed"]
    assert np.asarray(box.position) == pytest.approx([0.3, 0.2, 0.105])
    assert clean.arm.get_safety()["box_mm_effective"]["x"] == [100, 500]


def test_off_mode_removes_box_and_free_mode_opens_xy(clean):
    handles = clean.scene.server.scene._handle_from_node_name
    assert clean.set_safety("off", 0.0, None) is None
    assert "/allowed" not in handles
    assert clean.readout()["안전 모드"] == "끔"
    assert clean.set_safety("free", 0.0, None) is None
    box = handles["/allowed"]
    assert np.asarray(box.position)[:2] == pytest.approx([0.3, 0.2])
    assert clean.readout()["안전 모드"] == "자유"


def test_page_off_needs_confirmation_then_shows_red_badge(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        await user.should_not_see(marker="safety-off-badge")
        _one(user, "safety-mode").value = "off"
        await user.should_see(marker="off-confirm")
        assert clean.get_safety()["mode"] == "normal"  # 확인 전에는 바뀌지 않는다
        user.find(marker="off-cancel").click()
        assert await _until(lambda: _one(user, "safety-mode").value == "normal")
        assert clean.get_safety()["mode"] == "normal"
        _one(user, "safety-mode").value = "off"
        await user.should_see(marker="off-confirm")
        user.find(marker="off-confirm").click()
        assert await _until(lambda: clean.get_safety()["mode"] == "off")
        await user.should_see(marker="safety-off-badge")

    in_app(clean, "/safety", check)


def test_page_applies_margin_and_box(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        _one(user, "safety-margin").value = 5
        _one(user, "box-z-max").value = 150
        user.find(marker="safety-apply").click()
        assert await _until(lambda: clean.get_safety()["joint_margin_deg"] == 5)
        assert clean.get_safety()["box_mm"] == {"z": [None, 150]}

    in_app(clean, "/safety", check)


def test_one_sided_box_still_checks_moves(clean):
    """감사 2026-11-07: 높이 최대만 준 상자를 저장한 뒤에도 안전 검사가 예외 없이 판정해야 한다."""
    assert clean.set_safety("normal", 0.0, {"z": [None, 150]}) is None
    assert clean.arm.safety.check_goal(clean.arm.status().joints) is None


def test_view_only_cannot_change_safety(clean):
    clean.owner = "다른 브라우저"

    async def check(user):
        assert await _until(lambda: not _one(user, "safety-apply").enabled)
        assert not _one(user, "safety-mode").enabled

    in_app(clean, "/safety", check)


def test_workspace_text_keeps_comments_and_changes_values():
    text = pathlib.Path("config/workspace.yaml").read_text(encoding="utf-8")
    out = update_workspace_text(text, VALUES, "right")
    assert "배치 확정 후 수정" in out and "도달 밖 물체는" in out
    data = yaml.safe_load(out)
    assert data["table_mm"] == {"x": 700, "y": 450}
    assert data["robot_base_in_table"] == {"x_mm": 320, "y_mm": 60, "yaw_deg": 90}
    zone = data["drop_zones"]["right"]
    assert (zone["x_mm"], zone["y_mm"]) == (520, 170)  # 사각형 구역이면 w_mm·h_mm·slot_mm도 남는다(아래 시험)
    assert data["safety_margin_mm"] == 12
    assert data["reach_mm"] == {"min": 90, "max": 250}


def test_save_workspace_writes_and_validates(clean, tmp_path, monkeypatch):
    path = tmp_path / "workspace.yaml"
    path.write_text(pathlib.Path("config/workspace.yaml").read_text(encoding="utf-8"), encoding="utf-8")
    monkeypatch.setattr(clean, "workspace_path", str(path))
    try:
        assert clean.save_workspace(VALUES, "right") is None
        assert load_workspace(path).table_mm["x"] == 700
        assert clean.arm.ws.table_mm["x"] == 700 and clean.studio.ws.table_mm["x"] == 700  # REQ-034: 바로 적용
    finally:
        for arm in clean._arms():
            arm.reload_workspace("config/workspace.yaml")
        clean.studio.ws = clean.studio.task.ws = clean.arm.ws
    before = path.read_text(encoding="utf-8")
    assert "밖" in clean.save_workspace({**VALUES, "base_x": 9999}, "right")
    assert "밖" in clean.save_workspace({**VALUES, "zone_y": -1}, "right")
    assert "0보다" in clean.save_workspace({**VALUES, "table_x": 0}, "right")
    assert path.read_text(encoding="utf-8") == before


def test_table_page_shows_values_and_saves(clean, tmp_path, monkeypatch):
    path = tmp_path / "workspace.yaml"
    real_before = pathlib.Path("config/workspace.yaml").read_text(encoding="utf-8")
    path.write_text(real_before, encoding="utf-8")
    monkeypatch.setattr(clean, "workspace_path", str(path))

    async def check(user):
        await user.should_see(marker="ws-plane")
        await user.should_see("실측 전 값")
        assert await _until(lambda: clean.owner is not None)
        _one(user, "ws-zone_x").value = 480
        user.find(marker="ws-save").click()
        assert await _until(lambda: yaml.safe_load(path.read_text(encoding="utf-8"))["drop_zones"]["right"]["x_mm"] == 480)
        await user.should_see("저장했습니다")

    try:
        in_app(clean, "/table", check)
    finally:
        for arm in clean._arms():
            arm.reload_workspace("config/workspace.yaml")
        clean.studio.ws = clean.studio.task.ws = clean.arm.ws
    assert pathlib.Path("config/workspace.yaml").read_text(encoding="utf-8") == real_before  # 진짜 파일은 그대로다


def test_cards_cover_known_params(clean):
    params = clean.arm.get_params()
    listed = {key for _, group in CARDS for key in group}
    assert listed <= set(params)
    assert len(CARDS) == 3  # 실행 방식 카드는 따로 그린다
    assert set(TIMING_TEXT) == {v["timing"] for v in params.values()}


def test_set_params_all_or_nothing(clean):
    before = clean.arm.cfg.grasp_height_mm
    reason, timings = clean.set_params({"grasp_height_mm": 20, "trajectory_profile": "없는값"})
    assert reason and timings == {}
    assert clean.arm.cfg.grasp_height_mm == before
    reason, timings = clean.set_params({"grasp_height_mm": 20})
    assert reason is None and timings == {"grasp_height_mm": "next_motion"}


def test_page_highlights_changed_and_applies(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        await user.should_see("바로")
        await user.should_see("다음 동작부터")
        await user.should_see("재시작")
        apply_button = _one(user, "param-apply")
        assert await _until(lambda: not apply_button.enabled)
        _one(user, "param-grasp_height_mm").value = 22
        assert await _until(lambda: apply_button.enabled)
        assert "param-changed" in _one(user, "row-grasp_height_mm").classes
        assert "바꾼 값 1개" in _one(user, "param-count").text
        user.find(marker="param-apply").click()
        assert await _until(lambda: clean.arm.cfg.grasp_height_mm == 22)
        assert await _until(lambda: "param-changed" not in _one(user, "row-grasp_height_mm").classes)

    in_app(clean, "/params", check)


def test_speed_param_applies_now(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        _one(user, "param-speed_pct").value = 25
        assert await _until(lambda: _one(user, "param-apply").enabled)
        user.find(marker="param-apply").click()
        assert await _until(lambda: clean.arm.speed == 25)
        await user.should_see("25 %")

    in_app(clean, "/params", check)


def test_preset_round_trip(clean, tmp_path):
    clean.arm.preset_dir = str(tmp_path)
    clean.set_params({"grasp_height_mm": 33})
    assert clean.save_preset("시험") is None
    clean.set_params({"grasp_height_mm": 11})
    assert clean.load_preset("시험") is None
    assert clean.arm.cfg.grasp_height_mm == 33
    assert clean.save_preset("나쁜/이름")
    assert clean.load_preset("없음")


def test_table_pad_moves_the_chosen_item(clean, tmp_path, monkeypatch):
    """G16: 작업대 페이지 패드로 배출 구역을 5mm 옮긴다(저장 전이라 파일은 그대로)."""
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        before = _one(user, "ws-zone_y").value
        user.find(marker="nudge-위로").trigger("mousedown")
        user.find(marker="nudge-위로").trigger("mouseup")
        assert await _until(lambda: _one(user, "ws-zone_y").value == before + 5)

    in_app(clean, "/table", check)


def test_table_ignores_out_of_range_numbers(clean):
    """가로에 아주 큰 수를 쳐도 그림을 다시 그리지 않는다(격자선 수십만 개로 연결이 끊겼다, 무작위 조작 시험)."""
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        plane = _one(user, "ws-plane")
        before = plane.content
        _one(user, "ws-table_x").value = 10_000_000
        await asyncio.sleep(0.3)
        assert plane.content == before and len(plane.content) < 100_000
        _one(user, "ws-table_x").value = 700
        assert await _until(lambda: plane.content != before)

    in_app(clean, "/table", check)


def test_workspace_text_keeps_rectangle_zone_keys():
    """사각형 배출 구역(11.3.5, REQ-058)은 가운데 x·y만 바꾸고 w_mm·h_mm·slot_mm는 그대로 둔다."""
    text = "drop_zones:\n  right: {x_mm: 480, y_mm: 150, w_mm: 100, h_mm: 80, slot_mm: 75}\n"
    text = "table_mm: {x: 600, y: 400}\nrobot_base_in_table: {x_mm: 300, y_mm: 50, yaw_deg: 90}\n" + text + "safety_margin_mm: 10\n"
    data = yaml.safe_load(update_workspace_text(text, VALUES, "right"))
    assert data["drop_zones"]["right"] == {"x_mm": 520, "y_mm": 170, "w_mm": 100, "h_mm": 80, "slot_mm": 75}
