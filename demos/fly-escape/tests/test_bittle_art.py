"""The standalone demo must serve the same packaged art used by the app bundle."""

import hashlib
import json
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest
from flybrain_demo_api.web import DashboardServer, FrameHub, bittle_asset


def test_packaged_art_and_http_allowlist():
    manifest = json.loads(bittle_asset("/bittle/manifest.json")[0])
    server = DashboardServer(FrameHub(), lambda _: {}).start()
    try:
        for name, entry in manifest.items():
            with urlopen(server.url + "bittle/" + name) as response:
                data = response.read()
                assert len(data) == entry["bytes"]
                assert hashlib.sha256(data).hexdigest() == entry["sha256"]
                if name.endswith(".webp"):
                    assert response.headers["Content-Type"] == "image/webp"
                    assert data[:4] == b"RIFF" and data[8:12] == b"WEBP"
        for path in ("../web.py", "%2e%2e/web.py", "missing.webp"):
            with pytest.raises(HTTPError) as error:
                urlopen(server.url + "bittle/" + path)
            assert error.value.code == 404
    finally:
        server.stop()
