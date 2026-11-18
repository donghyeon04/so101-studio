"""에피소드 기록기 (mujoco-spec 13.10.4, 결정 0066 L2): MuJoCo 시뮬레이션을 LeRobot 데이터셋(lerobot 0.6.1)으로 남긴다.

한 줄 = `observation.state`(실제 관절 6개) + `action`(`svc.status().cmd`, 명령 관절 6개) + 카메라마다 `observation.images.<이름>`(render).
단위는 관절 1~5가 °, 그리퍼가 0~100(0 닫힘)이고 이름은 `shoulder_pan.pos … gripper.pos`다(LeRobot so101_follower와 같은 키·차원, 영점은 보장 안 함).
시계는 time.perf_counter로 fps를 맞춘다(실시간 수집, 13.10.2 뒤 ArmService 시계로)."""
import gc
import json
import re
import shutil
import threading
import time
from dataclasses import astuple
from pathlib import Path

import numpy as np

from .driver import JOINTS

NAMES = [f"{j}.pos" for j in JOINTS]
NOT_INSTALLED = "lerobot[dataset]이 설치되지 않았습니다. docs/guides/tooling-setup.md C-8"


def dataset_dir(name: str, root: Path) -> Path:
    """`runs/datasets/<이름>/`. 이름은 글자·숫자·`_`·`-`만."""
    if not re.fullmatch(r"[\w-]+", name):
        raise ValueError(f"데이터셋 이름은 글자·숫자·_·-만: {name!r}")
    return root / "runs" / "datasets" / name


class EpisodeRecorder:
    def __init__(self, svc, driver, dataset_dir, fps: int = 25, cameras=("top", "wrist"), task: str = "", width: int = 320, height: int = 240):
        """dataset_dir가 이미 있으면 ValueError(이어 쓰기는 2차). lerobot[dataset]이 없으면 RuntimeError(NOT_INSTALLED)."""
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as e:
            raise RuntimeError(NOT_INSTALLED) from e
        root = Path(dataset_dir)
        if root.exists():
            raise ValueError(f"데이터셋 폴더가 이미 있다(이어 쓰기는 2차): {root}")
        self.svc, self.driver, self.fps, self.cameras, self.task, self.size = svc, driver, fps, tuple(cameras), task, (width, height)
        vec = {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES}
        features = {"observation.state": vec, "action": dict(vec)}
        for c in self.cameras:
            features[f"observation.images.{c}"] = {"dtype": "video", "shape": (height, width, 3), "names": ["height", "width", "channels"]}
        self.ds = LeRobotDataset.create(repo_id=f"local/{root.name}", root=root, fps=fps, robot_type="so101_follower", features=features, use_videos=True,
                                         encoder_threads=1)  # 병렬 수집에서 다른 일꾼의 영상 인코딩이 CPU를 다 쓰면 물리가 밀린다(realtime_lag, 결정 0068)
        self.frames = 0  # 지금 에피소드의 줄 수
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None

    def frame(self) -> dict:
        """지금 순간의 한 줄."""
        st = self.svc.status()
        state = self.driver.read_joints()
        cmd = st.cmd if st.cmd is not None else state  # 아직 명령이 없으면 지금 자세가 곧 목표
        row = {"observation.state": np.asarray(astuple(state), np.float32), "action": np.asarray(astuple(cmd), np.float32), "task": self.task}
        for c in self.cameras:
            row[f"observation.images.{c}"] = self.driver.render(c, *self.size)
        return row

    def _run(self) -> None:
        period, t_next = 1.0 / self.fps, time.perf_counter()
        try:
            while not self._stop.is_set():
                self.ds.add_frame(self.frame())
                self.frames += 1
                t_next += period
                delay = t_next - time.perf_counter()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    t_next = time.perf_counter()  # 밀리면 따라잡지 않고 다음 칸부터(빈 칸은 생기지만 같은 순간을 겹쳐 적지 않는다)
        except BaseException as e:  # noqa: BLE001 - stop()에서 다시 올린다
            self._error = e

    def start(self, task: str | None = None) -> None:
        if self._thread is not None:
            raise RuntimeError("이미 기록 중")
        if task is not None:
            self.task = task
        self.frames, self._error = 0, None
        self._stop.clear()
        gc.collect()
        gc.disable()  # 기록 중 GC 2세대 수집이 GIL을 0.1초 넘게 잡아 물리가 밀렸다(realtime_lag, 결정 0068). stop()에서 다시 켠다
        try:
            self._thread = threading.Thread(target=self._run, name="episode-recorder", daemon=True)
            self._thread.start()
        except BaseException:
            self._thread = None
            gc.enable()
            raise

    def stop(self, save: bool) -> int:
        """기록을 멈추고 save면 에피소드로 저장, 아니면 버린다. 저장한 줄 수(버렸으면 0)."""
        t, self._thread = self._thread, None
        if t is None:
            return 0
        self._stop.set()
        t.join()
        gc.enable()
        if self._error is not None or not (save and self.frames):
            self.ds.clear_episode_buffer()
            if self._error is not None:
                raise RuntimeError(f"기록 중 오류: {self._error!r}") from self._error
            return 0
        self.ds.save_episode()
        return self.frames

    def close(self) -> None:
        if self._thread is not None:
            self.stop(save=False)
        self.ds.finalize()


def merge_parts(root: Path) -> int:
    """`root/part-NN/` 조각들을 lerobot 0.6.1 `aggregate_datasets`로 하나로 합쳐 `root`에 둔다(결정 0068). 합친 에피소드 수.
    빈 조각은 뺀다. 옆 폴더 `<이름>.merging`에 합친 뒤 `<이름>`(조각) → `<이름>.parts`, `.merging` → `<이름>` 순으로 바꿔 달고, 그다음에만 조각을 지운다."""
    from lerobot.datasets.aggregate import aggregate_datasets

    root = Path(root)
    parts = sorted(p for p in root.glob("part-*") if json.loads((p / "meta" / "info.json").read_text(encoding="utf-8"))["total_episodes"] > 0)
    if not parts:
        raise RuntimeError(f"저장한 에피소드가 없다: {root}")
    tmp, old = root.with_name(root.name + ".merging"), root.with_name(root.name + ".parts")
    if tmp.exists():
        shutil.rmtree(tmp)
    aggregate_datasets([f"local/{p.name}" for p in parts], f"local/{root.name}", roots=parts, aggr_root=tmp)
    _rename(root, old)
    _rename(tmp, root)
    shutil.rmtree(old, ignore_errors=True)
    return json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))["total_episodes"]


def _rename(a: Path, b: Path, tries: int = 20) -> None:
    """Windows에서 막 쓴 폴더는 잠깐 잠겨 rename이 PermissionError를 낸다(11-18 --jobs 4 수집에서 한 번): 0.5초씩 다시 한다."""
    for i in range(tries):
        try:
            a.rename(b)
            return
        except PermissionError:
            if i == tries - 1:
                raise
            time.sleep(0.5)


def _rewrite_actions(src: Path, dst: Path, fn, step: dict) -> Path:
    """action만 fn(action 배열, episode_index 배열) → 새 action으로 바꾼 사본을 dst에 만든다(--shift·--clip 공통).
    영상은 하드링크(안 되면 복사), meta/stats.json의 action 통계는 다시 계산하고, meta/collect.json의 derived에 step을 더한다.
    shortcut: meta/episodes의 에피소드별 action 통계는 원본 그대로다(학습 정규화는 stats.json을 쓴다). 에피소드별 통계를 쓰는 도구가 생기면 다시 계산한다."""
    import os

    import pyarrow as pa
    import pyarrow.parquet as pq

    if dst.exists():
        raise ValueError(f"사본 폴더가 이미 있다: {dst}")

    def link(a, b):
        try:
            os.link(a, b)
        except OSError:
            shutil.copy2(a, b)

    shutil.copytree(src / "meta", dst / "meta")
    if (src / "videos").exists():
        shutil.copytree(src / "videos", dst / "videos", copy_function=link)
    actions = []
    for f in sorted((src / "data").rglob("*.parquet")):
        t = pq.read_table(f)
        out = fn(np.stack(t.column("action").to_numpy(zero_copy_only=False)), t.column("episode_index").to_numpy())
        actions.append(out)
        col = pa.array(out.tolist(), type=t.schema.field("action").type)  # 합친 데이터셋은 고정 길이가 아닌 list일 수 있다: 원래 열 형식 그대로
        t = t.set_column(t.schema.get_field_index("action"), t.schema.field("action"), col)
        (dst / f.relative_to(src)).parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(t, dst / f.relative_to(src))
    a = np.concatenate(actions).astype(np.float64)
    stats_path = dst / "meta" / "stats.json"
    stats = json.loads(stats_path.read_text(encoding="utf-8"))
    q = {f"q{p:02d}": np.quantile(a, p / 100, axis=0).tolist() for p in (1, 10, 50, 90, 99)}
    stats["action"] = {"min": a.min(0).tolist(), "max": a.max(0).tolist(), "mean": a.mean(0).tolist(), "std": a.std(0).tolist(), "count": [len(a)],
                       **{key: v for key, v in q.items() if key in stats["action"]}}
    stats_path.write_text(json.dumps(stats, indent=4), encoding="utf-8")
    info_path = dst / "meta" / "collect.json"
    info = json.loads(info_path.read_text(encoding="utf-8")) if info_path.exists() else {}
    info["derived"] = [*info.get("derived", []), {"from": src.name, **step}]  # 원본 → shift → clip 순서
    info_path.write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
    return dst


def shift_actions(src: Path, k: int) -> Path:
    """`action[t] = cmd[t+k]`(에피소드 안에서, 끝 k프레임은 마지막 명령)인 사본 `<이름>-k<k>`를 만든다(결정 0068). 기록기는 그대로다."""
    if k < 1:
        raise ValueError("k는 1 이상")

    def fn(act, ep):
        out = act.copy()
        for e in np.unique(ep):
            idx = np.flatnonzero(ep == e)  # 한 파일 안 에피소드 줄은 frame_index 순서로 이어져 있다
            out[idx] = act[np.minimum(idx + k, idx[-1])]
        return out

    src = Path(src)
    return _rewrite_actions(src, src.with_name(f"{src.name}-k{k}"), fn, {"op": "shift", "k": k})


def clip_actions(src: Path, limits: dict, margin: float = 1.0) -> tuple[Path, dict]:
    """action을 관절별 [lo+margin, hi−margin]로 자른 사본 `<이름>-clip`(REQ-077, 총괄 결정 (c)). 전문가 그리퍼 명령이 한계(99)에 붙어 있어
    정책이 조금만 넘겨도 PolicyRunner가 거부했다. limits는 {관절: [lo, hi]}(시험장 cfg의 joint_limits_deg). 관절별 자른 프레임 수·비율도 돌려준다."""
    lo = np.array([limits[j][0] + margin for j in JOINTS])
    hi = np.array([limits[j][1] - margin for j in JOINTS])
    count, total = np.zeros(len(JOINTS), int), [0]

    def fn(act, ep):
        count[:] += ((act < lo) | (act > hi)).sum(0)
        total[0] += len(act)
        return np.clip(act, lo, hi)

    src = Path(src)
    dst = _rewrite_actions(src, src.with_name(f"{src.name}-clip"), fn, {"op": "clip", "margin": margin, "range": {j: [float(lo[i]), float(hi[i])] for i, j in enumerate(JOINTS)}})
    clipped = {j: {"frames": int(count[i]), "pct": round(100.0 * float(count[i]) / max(total[0], 1), 2)} for i, j in enumerate(JOINTS)}
    info_path = dst / "meta" / "collect.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    info["derived"][-1]["clipped"] = clipped
    info_path.write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
    return dst, clipped


def _limits_for(src: Path) -> dict:
    """데이터셋을 모은 시험장 cfg(meta/collect.json의 preset, 없으면 기본 덮어쓰기)의 관절 한계."""
    from .mujoco_bench import BENCH_DIR, OVERRIDE, load_cfg

    p = Path(src) / "meta" / "collect.json"
    preset = json.loads(p.read_text(encoding="utf-8")).get("preset") if p.exists() else None
    return load_cfg(BENCH_DIR / preset if preset else OVERRIDE).joint_limits_deg


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(prog="python -m so101_studio.arm.mujoco_record", description="데이터셋 도구(mujoco-spec 13.10.4, 결정 0068)")
    ap.add_argument("--shift", metavar="데이터셋", help="action을 k프레임 앞당긴 사본 <이름>-k<k>를 만든다(runs/datasets/<이름> 또는 폴더 경로)")
    ap.add_argument("--k", type=int, default=5)
    ap.add_argument("--clip", metavar="데이터셋", help="action을 관절 한계 안쪽 [lo+margin, hi−margin]으로 자른 사본 <이름>-clip을 만든다")
    ap.add_argument("--margin", type=float, default=1.0)
    a = ap.parse_args(argv)
    if bool(a.shift) == bool(a.clip):
        ap.error("--shift와 --clip 중 하나만")
    name = a.shift or a.clip
    src = Path(name) if Path(name).exists() else dataset_dir(name, Path(__file__).resolve().parents[3])
    if a.shift:
        print(f"사본: {shift_actions(src, a.k)}")
        return
    dst, clipped = clip_actions(src, _limits_for(src), a.margin)
    for j, c in clipped.items():
        print(f"{j}: {c['frames']}프레임 ({c['pct']}%)")
    print(f"사본: {dst}")


if __name__ == "__main__":
    main()
