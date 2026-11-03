"""버스 쓰기를 기록하는 얇은 감싸개 (decisions/0030). 레지스터 쓰기(Goal_Position 스트림 제외)와 토크 켜기·끄기를 레지스터·서보·값·부른 곳·성공 여부와 함께 sink에 알린다.
읽기와 Goal_Position 쓰기는 그대로 통과시킨다(50Hz 제어 틱 안에서는 비교 한 번만 늘어난다)."""
import sys


def _callers(skip_file: str, depth: int = 3) -> list[str]:
    """이 감싸개 밖에서 부른 쪽 함수 이름을 가까운 것부터 depth개."""
    out, f = [], sys._getframe(2)
    while f and len(out) < depth:
        if f.f_code.co_filename != skip_file:
            out.append(f"{f.f_globals.get('__name__', '?').rsplit('.', 1)[-1]}.{f.f_code.co_name}:{f.f_lineno}")
        f = f.f_back
    return out


class LoggedBus:
    def __init__(self, bus, sink):
        self._bus = bus
        self._sink = sink  # sink(kind, level, code, data)

    def __getattr__(self, name):
        return getattr(self._bus, name)

    def _emit(self, kind, level, code, data) -> None:
        sink = self._sink() if callable(self._sink) else self._sink
        if sink:
            try:
                sink(kind, level, code, data)
            except Exception:  # noqa: BLE001, S110 기록 실패가 팔 제어를 막지 않는다
                pass

    def _write(self, register: str, motors, value, call, kw, caller_depth=3):
        callers = _callers(__file__, caller_depth)
        try:
            call()
        except Exception as e:
            self._emit("register_write", "ERROR", "ARM-REG-WRITE-FAILED", {"register": register, "motors": motors, "value": value, "ok": False, "error": repr(e), "caller": callers})
            raise
        torque = register == "Torque_Enable"
        code = ("ARM-TORQUE-ON" if value else "ARM-TORQUE-OFF") if torque and isinstance(value, int) else "ARM-REG-WRITE"
        self._emit("torque" if torque else "register_write", "WARN" if torque and not value else "INFO", code, {"register": register, "motors": motors, "value": value, "ok": True, "caller": callers})

    def write(self, register, motor, value, *a, **kw):
        if register == "Goal_Position":
            return self._bus.write(register, motor, value, *a, **kw)
        return self._write(register, [motor], value, lambda: self._bus.write(register, motor, value, *a, **kw), kw)

    def sync_write(self, register, values, *a, **kw):
        if register == "Goal_Position":  # 제어 틱의 스트림은 기록하지 않는다
            return self._bus.sync_write(register, values, *a, **kw)
        motors = list(values) if isinstance(values, dict) else None
        return self._write(register, motors, values, lambda: self._bus.sync_write(register, values, *a, **kw), kw)

    def enable_torque(self, motors=None, *a, **kw):
        return self._torque("ARM-TORQUE-ON", True, motors, lambda: self._bus.enable_torque(motors, *a, **kw))

    def disable_torque(self, motors=None, *a, **kw):
        return self._torque("ARM-TORQUE-OFF", False, motors, lambda: self._bus.disable_torque(motors, *a, **kw))

    def _torque(self, code, on, motors, call):
        callers = _callers(__file__)
        try:
            call()
        except Exception as e:
            self._emit("torque", "ERROR", "ARM-TORQUE-FAILED", {"on": on, "motors": motors, "ok": False, "error": repr(e), "caller": callers})
            raise
        self._emit("torque", "INFO" if on else "WARN", code, {"on": on, "motors": motors, "ok": True, "caller": callers})
