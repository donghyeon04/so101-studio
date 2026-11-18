"""여러 일꾼의 화면을 창 하나에 격자로 모아 보기(`mujoco_bench --viewer grid`, 2026-11-18 사용자 요청).

일꾼 프로세스마다 `Feeder` 스레드가 보기용 시점(`view`, 그리퍼를 비스듬히 따라감)을 초당 FPS장 그려 JPEG로 큐에 넣고, 창 프로세스(`run_window`, tkinter)가
일꾼(pid)마다 칸 하나를 잡아 그 아래에 에피소드 이름과 결과를 적는다. MuJoCo 뷰어와 달리 시점은 고정이다."""
import contextlib
import io
import math
import os
import queue
import threading
import time

FPS = 20
SIZE = (480, 360)

_GRID = None  # 이 프로세스의 큐(일꾼은 풀 initializer가, --jobs 1은 run이 넣는다)


def set_queue(q) -> None:
    """일꾼 풀 initializer(이 프로세스의 큐)."""
    global _GRID
    _GRID = q


def current():
    return _GRID


class Feeder:
    """한 에피소드 동안 driver.render('view')(그리퍼를 따라가는 보기용 시점)를 큐로 보낸다. render는 드라이버의 렌더 전용 스레드가 그려 어느 스레드에서 불러도 된다."""

    def __init__(self, q, driver, label: str):
        self.q, self.driver, self.label = q, driver, label
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, name="grid-feeder", daemon=True)

    def _send(self) -> bool:
        from PIL import Image

        try:
            img = self.driver.render("view", *SIZE)
        except (RuntimeError, ValueError):  # 연결이 끊겼거나 카메라가 없다
            return False
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="JPEG", quality=80)
        try:
            self.q.put_nowait((os.getpid(), buf.getvalue(), self.label))
        except queue.Full:  # 창이 못 따라오면(창을 붙잡고 있으면) 이 그림만 버린다
            return True
        except Exception:  # noqa: BLE001 창 프로세스가 없어져도 일꾼은 계속 돈다
            return False
        return True

    def _run(self) -> None:
        while not self._stop.wait(1.0 / FPS) and self._send():
            pass

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        self._t.join(timeout=1.0)
        self._send()  # 결과 글(note)이 붙은 마지막 그림

    def note(self, label: str) -> None:
        """칸 아래 글을 바꾼다(결과가 나오면). 다음 그림부터 쓰인다."""
        self.label = label


def run_window(q, n: int, title: str = "MuJoCo 시험장") -> None:
    """창 프로세스 본체: 큐에서 (pid, jpeg, 글)을 받아 칸에 그린다. None을 받으면 끝난다. 창을 닫아도 큐는 계속 비운다."""
    import tkinter as tk

    from PIL import Image, ImageTk

    cols = max(1, math.ceil(math.sqrt(n)))
    root = tk.Tk()
    root.title(f"{title} (일꾼 {n})")
    slots: dict[int, tuple] = {}
    alive = {"window": True, "run": True}

    def on_close():
        alive["window"] = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    def slot(pid):
        if pid not in slots:
            i = len(slots)
            f = tk.Frame(root)
            f.grid(row=i // cols, column=i % cols, padx=2, pady=2)
            pic, text = tk.Label(f), tk.Label(f, font=("Malgun Gothic", 9))
            pic.pack()
            text.pack()
            slots[pid] = (pic, text, [None])
        return slots[pid]

    def poll():
        try:
            for _ in range(64):
                item = q.get_nowait()
                if item is None:
                    alive["run"] = False
                    on_close()
                    return
                pid, jpeg, label = item
                pic, text, keep = slot(pid)
                keep[0] = ImageTk.PhotoImage(Image.open(io.BytesIO(jpeg)))  # 참조를 잡아 두지 않으면 그림이 지워진다
                pic.configure(image=keep[0])
                text.configure(text=label)
        except queue.Empty:
            pass
        except (EOFError, BrokenPipeError, ConnectionError):  # 부모가 끝났다
            alive["run"] = False
            on_close()
            return
        root.after(30, poll)

    root.after(30, poll)
    root.mainloop()
    while alive["run"]:  # 창을 닫았으면 끝 신호(None)까지 버리기만 한다(일꾼 쪽 큐가 쌓이지 않게)
        try:
            if q.get(timeout=1.0) is None:
                return
        except queue.Empty:
            continue
        except (EOFError, BrokenPipeError, ConnectionError):
            return


class Grid:
    """부모 쪽: Manager 큐와 창 프로세스를 띄우고 닫는다. with Grid(n) as q: ProcessPoolExecutor(initializer=set_queue, initargs=(q,))."""

    def __init__(self, n: int, title: str = "MuJoCo 시험장"):
        self.n, self.title = n, title

    def __enter__(self):
        import multiprocessing as mp

        self._mgr = mp.Manager()
        self.q = self._mgr.Queue(maxsize=FPS * self.n)  # 1초 분량까지만 쌓는다
        self._p = mp.Process(target=run_window, args=(self.q, self.n, self.title), daemon=True)
        self._p.start()
        set_queue(self.q)
        return self.q

    def __exit__(self, *a):
        set_queue(None)
        with contextlib.suppress(Exception):  # 창 프로세스가 먼저 끝났어도 된다
            self.q.put(None)
        self._p.join(timeout=3.0)
        if self._p.is_alive():
            self._p.terminate()
        time.sleep(0.1)
        self._mgr.shutdown()
