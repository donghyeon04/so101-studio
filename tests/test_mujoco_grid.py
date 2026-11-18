"""--viewer grid(2026-11-18 사용자 요청): 일꾼의 Feeder가 위 카메라를 JPEG로 큐에 보내고, 끝날 때 결과 글이 붙은 마지막 그림을 보낸다. 창(tkinter)은 시험하지 않는다."""
import io
import queue
import time

import numpy as np
import pytest

pytest.importorskip("PIL")

from so101_studio.arm import mujoco_grid


class FakeDriver:
    def __init__(self):
        self.calls = 0

    def render(self, camera, width, height):
        assert camera == "view"
        self.calls += 1
        return np.zeros((height, width, 3), np.uint8)


def test_feeder_sends_jpeg_frames_and_final_label():
    from PIL import Image

    q, drv = queue.Queue(), FakeDriver()
    with mujoco_grid.Feeder(q, drv, "s1") as f:
        time.sleep(0.4)
        f.note("s1: placed_ok")
    items = []
    while not q.empty():
        items.append(q.get())
    assert len(items) >= 2 and items[0][2] == "s1" and items[-1][2] == "s1: placed_ok"
    assert Image.open(io.BytesIO(items[-1][1])).size == mujoco_grid.SIZE


def test_feeder_stops_quietly_when_driver_is_gone():
    class Gone:
        def render(self, *a):
            raise RuntimeError("연결 전에는 render할 수 없다")

    q = queue.Queue()
    with mujoco_grid.Feeder(q, Gone(), "x"):
        time.sleep(0.3)
    assert q.empty()


def test_feeder_drops_frames_when_the_queue_is_full_and_keeps_going():
    q, drv = queue.Queue(maxsize=1), FakeDriver()
    with mujoco_grid.Feeder(q, drv, "s1"):
        time.sleep(0.4)
    assert q.qsize() == 1 and drv.calls >= 3  # 꽉 찬 뒤에도 계속 그렸다(버리고 계속)
