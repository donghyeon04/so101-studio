"""시리얼 포트 목록과 고르기 (decisions/0027). lerobot을 import하지 않는다. 목록 읽기는 포트를 열지 않는다."""

# USB 시리얼 칩 제조사 번호. 이 칩이 아닌 포트(블루투스 가상 COM 등)는 열지 않는다
USB_SERIAL_VIDS = {0x1A86: "CH340/CH343", 0x10C4: "CP210x", 0x0403: "FTDI", 0x303A: "Espressif(ESP32)"}
ESP32_VID = 0x303A


class PortChoiceError(Exception):
    """포트를 하나로 고르지 못했다. candidates는 후보 포트 목록."""

    def __init__(self, message: str, candidates: list[dict] | None = None):
        super().__init__(message)
        self.candidates = candidates or []


def list_serial_ports() -> list[dict]:
    """OS가 주는 포트 목록. 포트를 열지 않는다. pyserial이 없으면 빈 목록."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [{"device": p.device, "vid": p.vid, "pid": p.pid, "serial": p.serial_number, "description": p.description or ""}
            for p in list_ports.comports()]


def is_usb_serial(port: dict) -> bool:
    return port.get("vid") in USB_SERIAL_VIDS


def fmt_id(port: dict) -> str:
    vid, pid = port.get("vid"), port.get("pid")
    return f"{vid:04X}:{pid:04X}" if vid is not None and pid is not None else "-"


def diff_ports(before: list[dict], after: list[dict]) -> list[dict]:
    """before에는 있고 after에는 없는 포트(so101 find-port: USB를 뽑았을 때 사라진 포트)."""
    now = {p["device"] for p in after}
    return [p for p in before if p["device"] not in now]


def pick_port(ports: list[dict], board_serial: str = "", explicit: str | None = None, local_port: str | None = None,
              fallback: str | None = None) -> tuple[str, str]:
    """포트 고르기: --port -> local.yaml의 port -> 보드 시리얼이 같은 USB 시리얼 포트 -> (시리얼을 모르는 새 팔이면) USB 시리얼 포트가 하나뿐일 때.
    목록을 못 읽었을 때만 robot.yaml의 port(fallback)를 쓴다. 못 고르면 PortChoiceError."""
    if explicit:
        return explicit, "--port 옵션"
    if local_port:
        return local_port, "local.yaml의 port"
    if not ports:
        if fallback:
            return fallback, "포트 목록을 못 읽어 robot.yaml의 port 사용"
        raise PortChoiceError("포트 목록이 비어 있다. 보드가 연결됐는지, pyserial이 있는지 확인한다")
    usb = [p for p in ports if is_usb_serial(p)]
    if board_serial:
        hit = [p for p in usb if p.get("serial") == board_serial]
        if len(hit) == 1:
            return hit[0]["device"], f"보드 시리얼 {board_serial}이 같은 포트"
        raise PortChoiceError(f"시리얼 {board_serial}인 보드가 {'여러 개 보인다' if hit else '안 보인다'}. --port <포트>로 직접 고르거나 so101 find-port를 쓴다", hit or usb)
    if len(usb) == 1:
        return usb[0]["device"], "USB 시리얼 포트가 하나뿐"
    raise PortChoiceError("포트를 고르지 못했다. --port <포트>로 직접 고르거나 so101 find-port를 쓴다", usb)
