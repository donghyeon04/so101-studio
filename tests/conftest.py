"""모든 시험 공통 고정구."""
import pytest


@pytest.fixture(autouse=True)
def _keep_real_runs(monkeypatch):
    """시험이 만드는 실행 기록(RunLogger)이 진짜 runs/의 보관 정리를 부르지 않게 한다(REQ-094, 2026-11-19 runs/train 체크포인트가 지워졌다).
    prune_runs 자체 시험은 core.retention에서 바로 불러 쓰므로 영향이 없다."""
    monkeypatch.setattr("so101_studio.core.logging.prune_runs", lambda *a, **k: [])
