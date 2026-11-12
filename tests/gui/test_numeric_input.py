"""숫자 입력 (gui-spec 12.7, 결정 0055): 값 정보표, 입력 방식 규칙, MetaNumber 계약, 허용 밖 글자일 때 버튼 끄기·누를 때 다시 확인. 지금은 ① 파라미터 화면."""

import asyncio
import time

import pytest

pytest.importorskip("nicegui")

from nicegui.testing import user_simulation

from so101_studio.gui import app as gui_app
from so101_studio.gui.pages.common import Controls
from so101_studio.gui.param_meta import META, hint_text, widget_for
from so101_studio.service.params import CHOICES


@pytest.fixture
def clean(backend):
    params = {k: v["value"] for k, v in backend.arm.get_params().items()}
    yield backend
    backend.arm.set_params(params)


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


PARAM_KEYS = [k for k in META if "." not in k]


def test_meta_param_keys_exist_and_soft_has_reason(backend):
    params = backend.arm.get_params()
    assert set(PARAM_KEYS) <= set(params)
    for key, m in META.items():
        assert m.hard[0] < m.hard[1], key
        if m.soft:
            assert m.why, key
            assert m.hard[0] <= m.soft[0] <= m.soft[1] <= m.hard[1], key


def test_unlisted_numeric_params_are_printed(backend, capsys):
    """표에 없는 숫자 파라미터("그 밖")는 숫자 칸만 둔다. 목록을 출력해 하나씩 채운다 (12.7.1)."""
    from so101_studio.gui.pages.params import HIDDEN

    other = sorted(k for k, v in backend.arm.get_params().items()
                   if k not in META and k not in HIDDEN and isinstance(v["value"], (int, float)) and not isinstance(v["value"], bool))
    print("META에 없는 숫자 파라미터:", other)
    assert "speed_pct" not in other


def test_original_checks_match():
    assert META["safety.joint_margin_deg"].hard == (-15.0, 30.0)
    assert META["safety.box.z"].hard[0] == 0.0
    assert META["program.s"].hard == (0.0, 600.0)
    assert META["program.count"].hard == (1.0, 1000.0)


def test_widget_rules():
    assert widget_for("speed_pct") == "slider"
    assert widget_for("replay.speed") == "slider"
    assert widget_for("target.x") == "gap"
    assert widget_for("joint.deg") == "gap"
    assert widget_for("a_max_deg_s2") == "number"
    assert widget_for("servo_acceleration") == "number"
    assert all(len(v) <= 3 for v in CHOICES.values())  # 지금 보기는 모두 토글


def test_slider_can_pick_soft_and_hard_ends():
    m = META["speed_pct"]
    ticks = {m.hard[0] + i * m.step for i in range(int((m.hard[1] - m.hard[0]) / m.step) + 1)}
    assert {5.0, 30.0, 100.0} <= ticks


def test_hint_line():
    assert hint_text(META["speed_pct"], 30) == "기본 30 · 추천 5~30 · 허용 5~100 %"
    assert hint_text(META["grasp_height_mm"], None) == "허용 0~100 mm"
    assert "비우면 기본 범위" in hint_text(META["safety.box.z"], None)


def test_page_shows_hint_and_slider(clean):
    async def check(user):
        await user.should_see("추천 5~30 · 허용 5~100 %")
        assert _one(user, "param-speed_pct-slider") is not None
        assert _one(user, "param-a_max_deg_s2").meta.slider is None  # 큰 값은 숫자 칸만

    in_app(clean, "/params", check)


def test_typing_out_of_range_disables_apply_and_guard_rejects(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        before = clean.arm.cfg.grasp_height_mm
        number = _one(user, "param-grasp_height_mm")
        number.meta.set(22)
        assert await _until(lambda: _one(user, "param-apply").enabled)
        number.value = 1e7  # 사람이 친 글자(blur 전)
        assert not number.meta.valid and number.meta.value == 22  # 마지막 허용 값은 그대로
        assert await _until(lambda: not _one(user, "param-apply").enabled)
        user.find(marker="param-apply").click()  # 꺼진 버튼은 눌리지 않지만, 눌려도 guard가 막는다
        await asyncio.sleep(0.3)
        assert clean.arm.cfg.grasp_height_mm == before

    in_app(clean, "/params", check)


def test_on_change_only_on_commit_and_soft_warning(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        number = _one(user, "param-speed_pct")
        meta = number.meta
        seen = []
        meta.on_change = lambda v: seen.append(v)
        number.value = 5
        number.value = 50  # 글자를 칠 때마다는 보내지 않는다
        assert seen == []
        meta._commit()  # blur·Enter
        assert seen == [50]
        assert "meta-soft-out" in meta.box.classes  # 추천 5~30 밖
        meta._commit()  # 바뀐 것이 없으면 다시 보내지 않는다
        assert seen == [50]
        assert meta.set(30, quiet=True) and seen == [50]
        assert "meta-soft-out" not in meta.box.classes
        assert not meta.set(1000) and meta.value == 30
        meta._step(10)
        assert meta.value == 40 and seen == [50, 40]  # 간격 버튼은 누를 때 바로 보낸다

    in_app(clean, "/params", check)


def test_load_shows_out_of_range_value_without_sending(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        meta = _one(user, "param-grasp_height_mm").meta
        seen = []
        meta.on_change = lambda v: seen.append(v)
        meta.load(500)
        assert not meta.valid and meta.initial == 500 and seen == []
        meta.load(12)
        assert meta.valid and meta.value == 12 and seen == []

    in_app(clean, "/params", check)


def test_guard_runs_sync_and_async_and_checks_needs(clean, monkeypatch):
    from so101_studio.gui.pages import common

    class Field:
        valid, name = True, "칸"

    calls, notes = [], []
    monkeypatch.setattr(common.ui, "notify", lambda text, **kw: notes.append(text))

    async def go():
        clean.owner = "me"
        controls = Controls(clean, "me")
        field = Field()

        async def a():
            calls.append("a")

        def s():
            calls.append("s")
            return 1

        assert await controls.guard(s, needs=[field])() == 1  # 동기 함수도 감싼다
        await controls.guard(a, needs=[field])()
        field.valid = False
        assert await controls.guard(s, needs=[field])() is None  # 누른 순간 허용 밖이면 부르지 않는다
        assert calls == ["s", "a"] and notes == ["칸: 허용 밖이라 실행하지 않습니다"]

    asyncio.run(go())


def test_meta_hard_within_service_limits():
    from so101_studio.service.params import LIMITS

    for key in PARAM_KEYS:
        lo, hi = LIMITS[key]
        assert lo <= META[key].hard[0] and META[key].hard[1] <= hi, key


def test_reset_puts_saved_value_and_marks_changed(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        saved = clean.arm.get_params()["grasp_height_mm"]["saved"]
        meta = _one(user, "param-grasp_height_mm").meta
        assert meta.set(saved + 3)
        assert await _until(lambda: "param-changed" in _one(user, "row-grasp_height_mm").classes)
        user.find(marker="param-grasp_height_mm-reset").click()
        assert meta.value == saved
        assert await _until(lambda: "param-changed" not in _one(user, "row-grasp_height_mm").classes)
        await user.should_see(f"기본 {saved:g}")

    in_app(clean, "/params", check)


# ② 직접 조작

def test_manual_out_of_range_coordinate_disables_move_and_point_buttons(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        assert await _until(lambda: _one(user, "move").enabled, 10.0)
        x = _one(user, "target-x")
        x.value = 1e7  # 사람이 친 글자
        assert not x.meta.valid
        assert await _until(lambda: not _one(user, "move").enabled and not _one(user, "pp-set-pick").enabled)
        x.value = 300
        assert await _until(lambda: _one(user, "pp-set-pick").enabled)

    in_app(clean, "/manual", check)


def test_manual_joint_limit_follows_chosen_joint(clean):
    async def check(user):
        deg = _one(user, "joint-to-deg").meta
        lims = clean.get_safety()["physical_limits_deg"] or clean.arm.cfg.joint_limits_deg
        assert deg.hard == tuple(float(v) for v in lims["shoulder_pan"])
        _one(user, "joint-to-name").value = 1
        assert await _until(lambda: deg.hard == tuple(float(v) for v in lims["shoulder_lift"]))
        assert await _until(lambda: f"허용 {lims['shoulder_lift'][0]:g}~{lims['shoulder_lift'][1]:g}" in deg.hint.text)

    in_app(clean, "/manual", check)


def test_manual_3d_point_outside_limits_changes_nothing(clean, monkeypatch):
    from so101_studio.gui.pages import manual as manual_page

    toasts = []
    monkeypatch.setattr(manual_page.ui, "notify", lambda text, **k: toasts.append(text))

    async def check(user):
        assert await _until(lambda: _one(user, "reach").text != "계산 중")
        x0, h0 = _one(user, "target-x").value, _one(user, "target-h").value
        clean.scene.pick(300.0, 200.0, 1000.0)  # 높이만 허용(0~400) 밖
        clean.scene.pick(300.0, 200.0, 1200.0)
        await asyncio.sleep(0.5)
        assert _one(user, "target-x").value == x0 and _one(user, "target-h").value == h0  # 셋 다 넣지 않는다
        assert toasts.count("점이 허용 범위 밖입니다") == 1  # 한 번만 알린다

    in_app(clean, "/manual", check)


def test_manual_object_field_out_of_range_disables_pick_place(clean):
    async def check(user):
        assert await _until(lambda: clean.owner is not None)
        assert await _until(lambda: _one(user, "pp-run").enabled, 10.0)
        _one(user, "pp-height").value = 900
        assert await _until(lambda: not _one(user, "pp-run").enabled)

    in_app(clean, "/manual", check)
