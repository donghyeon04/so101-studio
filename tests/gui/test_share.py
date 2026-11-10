"""다른 기기에서 보기(결정 0047·0049, gui-spec 12.6): 원격 판정, 주소 안내, 3D 주소. 포트를 쓰지 않는 단위 시험."""

from so101_studio.gui import share
from so101_studio.gui.share import is_remote


def test_only_loopback_is_local():
    for ip in ("127.0.0.1", "::1", "::ffff:127.0.0.1"):
        assert not is_remote(ip)
    for ip in ("192.168.0.12", "10.0.0.3", "", None):  # 같은 PC라도 LAN IP로 열면 원격, 모르는 값도 원격(조작권을 주지 않는 쪽)
        assert is_remote(ip)


def test_share_url_says_how_to_find_the_ip_when_unknown(monkeypatch):
    monkeypatch.setattr(share, "local_ip", lambda: "192.168.0.12")
    assert share.share_url(8080) == "http://192.168.0.12:8080/operator"
    assert "개인 네트워크" in share.share_warning(8080) and "E-STOP" in share.share_warning(8080)
    monkeypatch.setattr(share, "local_ip", lambda: None)
    assert "ipconfig" in share.share_url(8080)
