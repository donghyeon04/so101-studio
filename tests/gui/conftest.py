import pytest

pytest.importorskip("nicegui")

from so101_studio.gui.backend import Backend


@pytest.fixture(scope="module")
def shared_backend():
    backend = Backend(log_root=None)
    yield backend
    backend.close()


@pytest.fixture
def backend(shared_backend):
    """시뮬레이션 팔은 만드는 데 시간이 걸려 공유하고, 조작권과 정지 상태만 되돌린다."""
    shared_backend.owner = None
    shared_backend.arm.ctl.resume()
    return shared_backend
