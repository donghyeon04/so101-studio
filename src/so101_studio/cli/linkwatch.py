"""연결 감시: 서보 1~6에 계속 ping을 보내 응답이 끊기고 돌아오는 순간을 시각과 함께 보여 준다.
커넥터나 케이블을 만져 보며 어디가 접촉 불량인지 찾는 용도다. 서보 연결 없이 포트만 열어 쓴다(팔이 안 보이는 상태에서도 동작). Ctrl+C로 끝."""
import time


def _packet(sid: int, *params: int, instruction: int) -> bytes:
    body = [sid, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def run_linkwatch(port: str = "COM3", baud: int = 1_000_000, period_s: float = 0.25, out=print, stop=lambda: False) -> dict:
    import serial

    s = serial.Serial(port, baud, timeout=0.05)
    last, drops, t0, volt = None, 0, time.perf_counter(), None
    out(f"{port} {baud}bps, 서보 1~6 감시 시작. 커넥터를 하나씩 만져 보세요 (Ctrl+C로 끝).")
    try:
        while not stop():
            alive = []
            for sid in range(1, 7):
                s.reset_input_buffer()
                s.write(_packet(sid, instruction=0x01))
                s.flush()
                if s.read(6):
                    alive.append(sid)
            if alive:  # 응답하는 서보 하나에서 전압(레지스터 62)을 읽는다
                s.reset_input_buffer()
                s.write(_packet(alive[0], 0x3E, 0x01, instruction=0x02))
                data = s.read(7)
                volt = data[5] / 10 if len(data) >= 7 else volt
            if alive != last:
                if last is not None and len(alive) < len(last):
                    drops += 1
                missing = [i for i in range(1, 7) if i not in alive]
                out(f"{time.strftime('%H:%M:%S')}  {len(alive)}/6 응답" + (f"  끊김: {missing}" if missing else "") + (f"  전압 {volt:.1f}V" if volt else ""))
                last = alive
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    out(f"끝. {time.perf_counter() - t0:.0f}초 동안 응답이 줄어든 횟수 {drops}번")
    return {"drops": drops}
