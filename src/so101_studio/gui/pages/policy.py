"""정책 실행 페이지 1차 줄인 판 (gui-spec 12.10 ③): 체크포인트 고르기, [시작]·[정지], 탑뷰·손목 카메라와 3D, 끝난 뒤 추론 시간·건너뜀·거부 수.
늘 시뮬레이션 쪽 서비스(backend.sim)에 붙는다. 일시 정지·다음 배치·판정 표시는 2차다."""

import json
import time
from functools import partial
from pathlib import Path

from nicegui import run, ui

from ...core.logread import read_events
from ...core.types import JointState
from ...service.policy_runner import CAMERAS, PolicyRunner
from .common import CAMERA_TITLES, Controls, camera_tile, scene_frame, watch_visibility

TRAIN_ROOT = Path("runs/train")
NO_RENDER_TEXT = "학습한 정책 실행은 MuJoCo 시뮬레이션에서만 합니다"
OUTCOME_TEXT = {"placed_ok": "성공 (배출 통에 놓음)", "untouched": "상자를 건드리지 못함", "moved": "상자를 밀기만 함", "dropped": "들었다가 떨어뜨림",
                "toppled": "상자가 넘어짐", "placed_off": "다른 곳에 놓음"}
MAX_S = 45  # 전문가 시연(집어서 놓기)이 약 31초라 PolicyRunner 기본 20초면 놓기 전에 끊긴다. 명령줄 평가(--policy)와 같은 값


def checkpoints(root: Path) -> list[str]:
    """runs/train/*/checkpoints/*/pretrained_model, 최근 것 먼저."""
    found = sorted(root.glob("*/checkpoints/*/pretrained_model"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [str(p) for p in found]


def training_info(checkpoint: Path) -> dict:
    """체크포인트의 train_config.json → dataset.root → meta/collect.json(mujoco_bench가 수집 때 쓴 묶음·시작 자세). 없거나 못 읽으면 {}."""
    try:
        cfg = json.loads((Path(checkpoint) / "train_config.json").read_text(encoding="utf-8"))
        return json.loads((Path(cfg["dataset"]["root"]) / "meta" / "collect.json").read_text(encoding="utf-8"))
    except (OSError, KeyError, TypeError, ValueError):
        return {}


def start_pose_of(checkpoint: Path) -> list[float] | None:
    """학습 데이터의 시작 자세(결정 0071). 없으면 None(지금 자리에서 시작)."""
    pose = training_info(checkpoint).get("start_pose")
    return [float(v) for v in pose] if pose and len(pose) == 6 else None


def result_lines(result: dict | None, rejects: int | None) -> list[str]:
    if not result:
        return ["아직 실행하지 않았습니다"]
    ms = "-" if result["infer_ms_mean"] is None else f"평균 {result['infer_ms_mean']}ms · 최대 {result['infer_ms_max']}ms"
    lines = [f"끝난 이유: {result['reason']}", f"추론 시간: {ms}", f"보낸 주기 {result['steps']} · 건너뛴 주기 {result['skipped']}",
             f"거부된 목표: {'기록 없음' if rejects is None else rejects}"]
    if result.get("outcome"):  # 학습 배치로 돌렸으면 시험장 판정(mujoco_bench.judge)
        lines.insert(0, f"판정: {OUTCOME_TEXT.get(result['outcome'], result['outcome'])}")
    return lines


def count_rejects(svc, since_ns: int) -> int | None:
    """since_ns 뒤의 ARM-SERVO-REJECT 수(1초에 한 번 남기며 그 사이 수를 count에 담는다). 기록을 안 남기는 서비스면 None."""
    if svc.logger is None:
        return None
    return sum(e["data"].get("count", 1) for e in read_events(svc.logger.dir) if e["code"] == "ARM-SERVO-REJECT" and e["t"] >= since_ns)


def build(backend, client_id: str, actions) -> None:
    svc = backend.sim
    if not callable(getattr(svc.driver, "render", None)):  # 실제 팔·기구학 엔진에는 카메라가 없다
        ui.label(NO_RENDER_TEXT).classes("page-body").mark("policy-unavailable")
        return
    controls = Controls(backend, client_id)
    started_ns = [0]

    def running() -> bool:
        runner = backend.policy
        return backend.policy_loading or (runner is not None and runner._thread is not None and runner._thread.is_alive())

    async def start() -> None:
        path = (choice.value or "").strip()
        if not path:
            ui.notify("체크포인트를 고르거나 경로를 넣으세요", type="warning")
            return
        if running():  # 불러오는 동안 다시 눌러도 러너를 하나 더 만들지 않는다
            return
        judge = None
        suite = training_info(Path(path)).get("suite")
        if layout_on.value and suite and hasattr(svc.driver, "set_objects"):  # 학습 배치(mujoco-spec 13.10.4 세계 다시 짓기): 학습 때 묶음의 이 씨앗 자리에 상자를 놓고 끝나면 판정한다
            try:
                lay = await run.io_bound(backend.rebuild_world, suite, int(seed.value))
            except ValueError as e:
                ui.notify(str(e), type="warning")
                return
            from ...arm import (
                mujoco_bench,  # MuJoCo가 없는 설치에서도 페이지가 열리게 여기서 부른다
            )
            judge = partial(mujoco_bench.judge, layout=lay)
        backend.policy = runner = PolicyRunner(svc, svc.driver, checkpoint_dir=Path(path), max_s=MAX_S, judge=judge)
        backend.policy_loading, backend.policy_cancel = True, False
        started_ns[0] = time.time_ns()
        try:
            if (pose := start_pose_of(Path(path))) is not None:  # 학습 때와 같은 자세에서 시작해야 정책이 상자를 본다(결정 0071)
                r = await run.io_bound(svc.move_joints, JointState(*pose))
                if r.status != "ok":
                    ui.notify(f"시작 자세로 못 갔습니다: {r.reason or r.status}", type="warning")
                    return
            why = await run.io_bound(runner.start)  # torch·lerobot을 처음 불러오면 몇 초 걸린다
        finally:
            backend.policy_loading = False
        if why:
            ui.notify(why, type="warning")
        elif backend.policy_cancel:  # 불러오는 중에 [정지]를 눌렀다: start()가 _quit을 지우므로 여기서 다시 멈춘다
            await run.io_bound(runner.stop)

    async def next_layout() -> None:
        """씨앗을 하나 올리고 그 배치로 세계를 다시 짓는다(시작은 하지 않는다)."""
        suite = training_info(Path((choice.value or "").strip())).get("suite")
        if not suite:
            ui.notify("이 체크포인트의 학습 데이터에 묶음 정보(collect.json)가 없습니다", type="warning")
            return
        seed.value = int(seed.value) + 1
        try:
            await run.io_bound(backend.rebuild_world, suite, int(seed.value))
        except ValueError as e:
            ui.notify(str(e), type="warning")

    async def stop() -> None:
        backend.policy_cancel = True
        if backend.policy is not None:
            await run.io_bound(backend.policy.stop)

    with actions:
        stop_button = ui.button("정지", color="negative", on_click=stop).props("outline no-caps").mark("policy-stop")
        start_button = controls.add(ui.button("시작", on_click=controls.guard(start)).props("no-caps").mark("policy-start"))

    with ui.row().classes("w-full items-center gap-3"):
        choice = ui.select(checkpoints(TRAIN_ROOT), label="체크포인트 (runs/train/…/pretrained_model 또는 직접 입력)", with_input=True,
                           new_value_mode="add-unique").classes("flex-1").mark("policy-checkpoint")
        if choice.options:
            choice.value = choice.options[0]
        status = ui.label("").classes("stat-value").mark("policy-status")
    with ui.row().classes("w-full items-center gap-3"):
        layout_on = ui.switch("학습 배치로 (학습 때 묶음의 상자 자리)", value=True).mark("policy-layout")
        seed = ui.number("씨앗", value=10000, min=0, step=1, format="%d").classes("w-28").mark("policy-seed")
        controls.add(ui.button("다음 배치", on_click=controls.guard(next_layout)).props("outline no-caps").mark("policy-next"))

    visible = watch_visibility()
    with ui.row().classes("w-full no-wrap items-start gap-4"):
        tiles = []
        for cam in CAMERAS:
            with ui.card().classes("flex-1 min-w-0 gap-1"):
                tiles.append(camera_tile(svc.driver, cam, CAMERA_TITLES[cam]))
        with ui.column().classes("flex-1 min-w-0"):
            scene_frame(backend, "scene3d scene3d-short")
    with ui.card().classes("w-full gap-0"):
        ui.label("지난 실행").classes("card-title")
        summary = ui.column().classes("gap-0").mark("policy-result")
    shown: list = [None]

    async def tick() -> None:
        controls.refresh()
        busy = running()
        status.text = "불러오는 중" if backend.policy_loading else "실행 중" if busy else "멈춤"
        start_button.enabled = controls.allowed and not busy
        stop_button.enabled = controls.mine and busy
        result = None if backend.policy is None else backend.policy.result
        if result is not shown[0]:
            shown[0] = result
            rejects = await run.io_bound(count_rejects, svc, started_ns[0]) if result else None  # 기록 파일이 커도 화면을 멈추지 않게
            summary.clear()
            with summary:
                for line in result_lines(result, rejects):
                    ui.label(line).classes("page-body")

    async def draw() -> None:
        if visible():
            for tile in tiles:
                await tile()

    ui.timer(0.2, tick)
    ui.timer(0.1, draw)  # 10fps (12.10 ①과 같은 그리기)
