import pytest


@pytest.fixture(autouse=True)
def _private_side_calibrations(tmp_path, monkeypatch):
    """fly-escape stores per-robot side checks in the user's folder: never the real one here."""
    monkeypatch.setenv("FLY_ESCAPE_SIDES", str(tmp_path / "fly-escape-sides.json"))
