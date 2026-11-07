"""전역 단축키 E-STOP: Windows GetAsyncKeyState 50Hz 폴링 (cli/keys.py와 같은 방식). Windows가 아니면 끈다."""
import sys
import threading


def vk(name: str) -> int:
    n = name.upper()
    if len(n) >= 2 and n[0] == "F" and n[1:].isdigit() and 1 <= int(n[1:]) <= 12:
        return 0x70 + int(n[1:]) - 1
    if len(n) == 1 and (n.isalpha() or n.isdigit()):
        return ord(n)
    raise ValueError(f"지원하지 않는 단축키: {name} (F1~F12 또는 영문자·숫자 한 글자)")


def start_hotkey(name: str, on_press, stop: threading.Event, on_error=None) -> threading.Thread | None:
    code = vk(name)
    if sys.platform != "win32":
        return None
    import ctypes

    user32 = ctypes.windll.user32

    def loop() -> None:
        was = False
        while not stop.wait(0.02):
            down = bool(user32.GetAsyncKeyState(code) & 0x8000)
            if down and not was:
                try:
                    on_press()
                except Exception as e:  # noqa: BLE001 콜백 하나의 예외로 단축키 스레드가 죽으면 이후 E-STOP 키가 조용히 먹통이 된다
                    if on_error:
                        on_error(e)
            was = down

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    return th
