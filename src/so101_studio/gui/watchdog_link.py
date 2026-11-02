"""감시 프로세스 연결 (decisions/0026): 함께 띄우기, 서버 heartbeat, 등록, 상태 읽기."""

import asyncio
import importlib.util
import os
import subprocess
import sys

import httpx

PORT = 8090
STOP_PORT = 8091
HB_S = 0.2
STATUS_EVERY = 5  # heartbeat 5번에 한 번(1초) 상태를 읽는다
TIMEOUT_S = 0.15

BROWSER_HB_JS = f"""
<script>
// Worker를 쓴다. 숨겨진 탭의 setInterval은 브라우저가 1초 이상으로 늦춰 감시가 끊김으로 본다
window.so101Hb = (on) => {{
  if (on && !window.so101HbWorker) {{
    const url = `http://${{location.hostname}}:{PORT}/hb?src=browser`;
    const code = `setInterval(() => fetch(${{JSON.stringify(url)}}).catch(() => {{}}), {int(HB_S * 1000)})`;
    window.so101HbWorker = new Worker(URL.createObjectURL(new Blob([code])));
  }} else if (!on && window.so101HbWorker) {{
    window.so101HbWorker.terminate();
    window.so101HbWorker = null;
  }}
}};
</script>
"""


class WatchdogLink:
    def __init__(self, sim: bool = True, port: int = PORT, stop_port: int = STOP_PORT):
        self.sim, self.port, self.stop_port = sim, port, stop_port
        self.available = True
        self.status: dict | None = None
        self.want_armed = False  # 조작할 브라우저가 생겨야 켠다. 그 전에는 브라우저 heartbeat가 없어 끊김으로 보인다
        self._armed = False
        self._proc: subprocess.Popen | None = None

    def start(self) -> None:
        # REQ-011 임시: 감시 프로세스 모듈이 main에 들어오기 전에는 "없음"으로 두고 시뮬레이션만 허용한다
        if importlib.util.find_spec("so101_studio.watchdog") is None:
            self.available = False
            return
        cmd = [sys.executable, "-m", "so101_studio.watchdog", "--port", str(self.port)] + (["--sim"] if self.sim else [])
        self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)

    def stop(self) -> None:
        if self._proc:
            self._proc.terminate()

    def text(self) -> str:
        if not self.available:
            return "없음"
        if self.status is None:
            return "꺼짐"
        return "작동함" if self.status.get("tripped") else "정상"

    @property
    def alive(self) -> bool:
        """감시가 응답하는가. 아니면 실제 팔 명령을 막는다."""
        return self.available and self.status is not None

    def rearm(self) -> None:
        """정지 해제 뒤 부른다. 감시 프로세스의 작동 상태는 다시 등록해야 풀린다."""
        self._armed = False

    async def run(self) -> None:
        """이벤트 루프에서 돈다. 루프가 멈추면 heartbeat도 멈추고, 그것을 감시 프로세스가 본다."""
        if not self.available:
            return
        base = f"http://127.0.0.1:{self.port}"
        n = 0
        async with httpx.AsyncClient(timeout=TIMEOUT_S) as client:
            while True:
                try:
                    await client.get(f"{base}/hb", params={"src": "server"})
                    if self.want_armed and not self._armed:
                        body = {"armed": True, "pid": os.getpid(), "stop_port": self.stop_port}
                        (await client.post(f"{base}/arm", json=body)).raise_for_status()
                        self._armed = True
                    if n % STATUS_EVERY == 0:
                        self.status = (await client.get(f"{base}/status")).json()
                except (httpx.HTTPError, ValueError):
                    self.status, self._armed = None, False  # 다시 살아나면 등록부터 다시 한다
                n += 1
                await asyncio.sleep(HB_S)
