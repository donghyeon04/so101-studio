"""REQ-051: `so101 gui --share`는 argparse가 받아 gui.main(args)에 그대로 넘기고, 다른 명령에는 영향이 없다 (결정 0047)."""
import importlib
import types

from so101_studio.cli import main


def test_gui_share_flag_reaches_gui_main_and_defaults_to_false(monkeypatch):
    got = []
    fake = types.SimpleNamespace(main=got.append)
    real = importlib.import_module
    monkeypatch.setattr(importlib, "import_module", lambda name, *a, **k: fake if name == "so101_studio.gui" else real(name, *a, **k))
    main(["gui", "--share"])
    main(["gui"])
    assert getattr(got[0], "share", False) is True and getattr(got[1], "share", False) is False
    assert got[0].command == "gui" and got[0].port is None  # --port는 서보 시리얼 포트로 그대로 남는다


def test_share_does_not_change_other_commands(capsys):
    main(["--sim", "status"])
    out = capsys.readouterr().out
    assert "shoulder_pan" in out
