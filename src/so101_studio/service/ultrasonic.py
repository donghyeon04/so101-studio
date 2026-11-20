"""초음파 인식 소스 만들기 (run --source ultrasonic, REQ-025). 인식 모듈은 팔을 모르므로 여기서 이어 준다.
--sim: 가상 ESP32(FakeESP32)가 시연 배치(show80: 반경 150~220mm·중심 간격 120mm·모서리 80mm 이상)의 물체를 작업대에 놓고, 팔 링크·센서 위치는 SimDriver가 보낸 관절 기록(PoseTrail)에서 읽는다.
실제: 시리얼 포트의 ESP32."""
import math
import random
import tempfile
from pathlib import Path

from ..perception.ultrasonic.benchmark import random_layout
from ..perception.ultrasonic.fake_esp32 import FakeESP32
from ..perception.ultrasonic.link import Link, SerialTransport
from ..perception.ultrasonic.model import load_config
from ..perception.ultrasonic.source import UltrasonicSource
from ..task.trail import PoseTrail


class SimUltrasonicSource(UltrasonicSource):
    """집어 간 물체를 가상 작업대에서도 치운다(실제로는 물체가 없어지니 레이더가 다시 보지 않는다)."""

    def __init__(self, *a, esp, **kw):
        super().__init__(*a, **kw)
        self.esp, self._found = esp, {}

    def end_check(self, cand=None, result=None, info=None, status="ok") -> None:
        if cand is not None and result is not None:
            self._found[cand.id] = (result.x_mm, result.y_mm)
        super().end_check(cand, result, info, status)

    def notify_picked(self, object_id: str) -> None:
        pos = self._found.get(object_id)
        if pos and self.esp.boxes:
            box = min(self.esp.boxes, key=lambda b: math.dist((b.x, b.y), pos))
            if math.dist((box.x, box.y), pos) <= 50:
                self.esp.boxes.remove(box)


def make_ultrasonic_source(arm, *, sim: bool, port: str | None = None, radar: dict | None = None, boxes=None):
    """UltrasonicSource를 만든다. bus는 TaskRunner가 Studio.bus로 이어 준다. 센서 오프셋은 TaskRunner가 시작할 때 arm에 넘긴다."""
    cfg = load_config(radar=radar, gripper=getattr(getattr(arm, "cfg", None), "gripper", None))  # 그리퍼별 check.tool_reach_mm
    if not sim:
        port = port or cfg.port
        if port in (None, "auto"):
            raise SystemExit("초음파 포트를 모른다: --us-port COMx 로 지정한다")
        return UltrasonicSource(Link(SerialTransport(port, cfg.baud)), cfg)  # 배출 구역(ignore_zones)은 TaskRunner가 실행을 시작할 때 넘긴다 (REQ-045)
    boxes = boxes if boxes is not None else random_layout(random.Random("demo-None-0"), "show80")  # 시연 배치 규칙(반경 150~220, REQ-102)
    trail, clk = PoseTrail(arm), arm.clock  # 팔 자세 기록(PoseTrail)이 팔 시계로 남으므로 레이더도 같은 시계(8.9)
    arm.trail = trail  # TaskRunner가 이 기록에 팔 자세를 남긴다
    esp = FakeESP32(clk, boxes, cfg, arm_sensor_fn=trail.sensor_at, arm_links_fn=trail.links_at)
    src = SimUltrasonicSource(Link(esp, clk), cfg, esp=esp, arm_links_fn=trail.links_at, baseline_path=Path(tempfile.mkdtemp()) / "baseline.json")
    src.connect()
    if (r := arm.skills.radar_clear()).status != "ok":  # 기준값에 팔이 찍히지 않게 대기 자세에서 잰다. 못 가면 재지 않는다
        raise RuntimeError(f"대기 자세(radar_clear)로 가지 못해 기준값을 재지 않았다: {r.status} {r.reason or ''}".strip())
    esp.boxes = []  # 빈 작업대로 기준값
    src.calibrate()
    esp.boxes = boxes
    return src
