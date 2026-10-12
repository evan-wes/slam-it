"""The server and the real worker together: a recording uploaded, run, and its result viewed.

The server runs under uvicorn on a free local port, with folder storage whose links point at it;
the worker (:mod:`slam_it.orchestration.worker`) claims the job over HTTP as any worker
would, and runs the import, the C++ pipeline and the bake on the room's Stray Scanner zip.
"""

from __future__ import annotations

import json
import socket
import threading
import time
import urllib.request
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest
import uvicorn

from slam_it.orchestration import worker
from slam_it_server.app import make_app
from slam_it_server.config import Settings
from tests.conftest import FIXTURES
from tests.edge import room
from tests.edge.support import FRAMES
from tests.eval.recordings import stray_zip
from tests.orchestration.conftest import require_program
from tests.server.conftest import CODE, TOKEN, make_settings


@pytest.fixture
def live(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Settings]:
    """Serve the app under uvicorn on a free local port until the test ends.

    The worker the test runs reads the fixture lens profiles.
    """
    monkeypatch.setenv("SLAM_IT_LENS_PROFILES", str(FIXTURES / "eval" / "lens_profiles"))
    listening = socket.socket()
    listening.bind(("127.0.0.1", 0))
    settings = make_settings(tmp_path, f"http://127.0.0.1:{listening.getsockname()[1]}")
    server = uvicorn.Server(uvicorn.Config(make_app(settings), log_level="warning"))
    thread = threading.Thread(target=server.run, kwargs={"sockets": [listening]}, daemon=True)
    thread.start()
    deadline = time.monotonic() + 10.0
    while not server.started and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.started
    yield settings
    server.should_exit = True
    thread.join(10.0)
    listening.close()


def _call(url: str, body: dict[str, Any] | None = None) -> Any:  # noqa: ANN401 -- JSON
    """GET a URL, or POST JSON to it; return the JSON answer."""
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data, {"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=30.0) as response:
        return json.loads(response.read())


def test_a_recording_uploaded_is_run_and_viewable(live: Settings, tmp_path: Path) -> None:
    """Made, uploaded, confirmed; claimed and run by the worker; done, its bundle served."""
    require_program("cpp")
    url = live.public_url
    # Stray Scanner's 30 fps setting, so that the import keeps every frame
    recording = stray_zip(tmp_path / "room.zip", room.sequence(FRAMES), divider=2).read_bytes()
    job = _call(f"{url}/api/jobs", {"code": CODE, "upload": "stray", "bytes": len(recording)})
    put = urllib.request.Request(job["upload_url"], recording, method="PUT")
    urllib.request.urlopen(put, timeout=30.0).close()
    assert _call(f"{url}/api/jobs/{job['token']}/uploaded", {}) == {"state": "queued"}

    worker.serve(worker.Client(url, TOKEN), tmp_path / "work", once=True)

    status = _call(f"{url}/api/jobs/{job['token']}")
    assert (status["state"], status["reason"]) == ("done", None)
    assert status["summary"]["frames"] == FRAMES
    assert status["summary"]["intrinsics"] == "stray-scanner"
    info = _call(f"{url}{status['bundle']}bundle.json")  # redirected to storage's link
    assert (info["truth"], info["backends"]["cpp"]["alignment"]) == (True, "sim3")
    files = Path(live.storage.removeprefix("folder:"))
    assert list(files.glob("jobs/*/upload")) + list(files.glob("jobs/*/import")) == []  # imported
