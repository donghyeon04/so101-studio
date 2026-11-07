import pytest

pytest.importorskip("nicegui")

from so101_studio.gui.backend import Backend


def pytest_configure(config):
    # 결정 0045·0046: 개발과 PR 확인은 -m "not slow", 하루 마칠 때 전체. pyproject 등록은 팔 세션 몫이라 GUI는 여기서 등록한다
    config.addinivalue_line("markers", "slow: 10초 넘게 걸리는 시험. PR 확인은 -m 'not slow'로 건너뛴다")


@pytest.fixture(scope="module")
def shared_backend():
    backend = Backend(log_root=None, viser_port=8190)
    yield backend
    backend.close()


@pytest.fixture
def backend(shared_backend):
    """시뮬레이션 팔은 만드는 데 시간이 걸려 공유하고, 조작권과 정지 상태만 되돌린다."""
    shared_backend.owner = None
    shared_backend.arm.ctl.resume()
    shared_backend.paused = False
    shared_backend.mode, shared_backend.pending = "sim", None
    return shared_backend
