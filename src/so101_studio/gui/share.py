"""다른 기기에서 보기(결정 0047·0049, gui-spec 12.6): 원격 판정과 이 PC의 IP."""

import socket

LOOPBACK = {"127.0.0.1", "::1", "::ffff:127.0.0.1", "localhost"}
SHARE_HOST = "0.0.0.0"
LOCAL_HOST = "127.0.0.1"
NO_IP_TEXT = "IP를 찾지 못했습니다. `ipconfig`로 이 PC의 IPv4 주소를 확인하세요"


def is_remote(ip: str | None) -> bool:
    """접속한 쪽 IP가 루프백이 아니면 원격이다. 같은 PC라도 LAN IP 주소로 열면 원격이다. 모르면(빈 값) 원격으로 본다(조작권을 주지 않는 쪽이 안전)."""
    return (ip or "") not in LOOPBACK


def local_ip() -> str | None:
    """이 PC의 바깥 쪽 IPv4 하나. UDP 소켓을 connect만 하고(패킷은 보내지 않음) 고른 주소를 읽는다. 못 찾으면 None."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
    except OSError:
        return None
    return None if ip in LOOPBACK or ip.startswith("127.") else ip


def share_url(web_port: int) -> str:
    """운영자 화면 주소. IP를 못 찾으면 안내 문장."""
    ip = local_ip()
    return f"http://{ip}:{web_port}/operator" if ip else NO_IP_TEXT


def share_warning(web_port: int) -> str:
    return (f"같은 네트워크의 누구나 화면을 볼 수 있고 E-STOP을 누를 수 있습니다: {share_url(web_port)} "
            "(Windows가 방화벽 허용을 물으면 '개인 네트워크'만 허용하세요)")
