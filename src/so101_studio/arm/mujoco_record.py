"""에피소드 기록기 (mujoco-spec 13.10.4, 결정 0066 L2): MuJoCo 시뮬레이션을 LeRobot 데이터셋(lerobot 0.6.1)으로 남긴다.

한 줄 = `observation.state`(실제 관절 6개) + `action`(`svc.status().cmd`, 명령 관절 6개) + 카메라마다 `observation.images.<이름>`(render).
단위는 관절 1~5가 °, 그리퍼가 0~100(0 닫힘)이고 이름은 `shoulder_pan.pos … gripper.pos`다(LeRobot so101_follower와 같은 키·차원, 영점은 보장 안 함).
시계는 time.perf_counter로 fps를 맞춘다(실시간 수집, 13.10.2 뒤 ArmService 시계로)."""
import re
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
        self.ds = LeRobotDataset.create(repo_id=f"local/{root.name}", root=root, fps=fps, robot_type="so101_follower", features=features, use_videos=True)
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
        self._thread = threading.Thread(target=self._run, name="episode-recorder", daemon=True)
        self._thread.start()

    def stop(self, save: bool) -> int:
        """기록을 멈추고 save면 에피소드로 저장, 아니면 버린다. 저장한 줄 수(버렸으면 0)."""
        t, self._thread = self._thread, None
        if t is None:
            return 0
        self._stop.set()
        t.join()
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
