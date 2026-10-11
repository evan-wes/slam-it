"""The worker, against a stand-in for the upload server: its protocol on a local port.

The stand-in serves one job's upload, takes the bundle's files by PUT, and records every
report. The job runs the real import, C++ pipeline and bake, on the room.
"""

from __future__ import annotations

import json
import threading
import urllib.parse
from collections.abc import Iterator
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import pytest

from slam_it_all.eval import video
from slam_it_all.foundations.errors import InvalidInputError
from slam_it_all.orchestration import worker
from tests.edge import room
from tests.edge.support import FRAMES
from tests.eval.recordings import stray_zip
from tests.orchestration.conftest import require_program

TOKEN = "a-worker-token"


@dataclass
class Server:
    """What the stand-in holds and was told."""

    upload: bytes
    kind: str = "stray"
    cancel_at: str | None = None
    jobs: list[str] = field(default_factory=lambda: ["job-1"])
    stages: list[str] = field(default_factory=list)
    files: dict[str, bytes] = field(default_factory=dict)
    summary: dict[str, Any] | None = None
    reason: str | None = None
    url: str = ""


def _handler(state: Server) -> type[BaseHTTPRequestHandler]:
    """Return a request handler speaking the worker's side of the protocol."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_: object) -> None:
            """Stay quiet."""

        def _answer(self, status: int, body: object = None) -> None:
            data = b"" if body is None else json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self) -> bytes:
            return self.rfile.read(int(self.headers.get("Content-Length", "0")))

        def do_GET(self) -> None:
            """Serve the upload."""
            self.send_response(200)
            self.send_header("Content-Length", str(len(state.upload)))
            self.end_headers()
            self.wfile.write(state.upload)

        def do_PUT(self) -> None:
            """Take one of the bundle's files."""
            path = urllib.parse.unquote(self.path.removeprefix("/put/"))
            state.files[path] = self._body()
            self._answer(200)

        def do_POST(self) -> None:
            """Answer a claim or a report."""
            if self.headers.get("Authorization") != f"Bearer {TOKEN}":
                self._answer(401)
                return
            body = json.loads(self._body() or b"{}")
            if self.path == "/api/worker/claim":
                if not state.jobs:
                    self._answer(204)
                    return
                job = state.jobs.pop()
                upload = {"job": job, "kind": "vo", "upload": state.kind}
                self._answer(200, {**upload, "download": f"{state.url}/upload"})
                return
            action = self.path.rsplit("/", 1)[-1]
            if action == "progress":
                state.stages.append(body["stage"])
                self._answer(409 if body["stage"] == state.cancel_at else 200, {})
            elif action == "files":
                urls = {
                    f["path"]: f"{state.url}/put/{urllib.parse.quote(f['path'])}"
                    for f in body["files"]
                }
                self._answer(200, {"urls": urls})
            elif action == "done":
                state.summary = body["summary"]
                self._answer(200, {})
            else:
                state.reason = body["reason"]
                self._answer(200, {})

    return Handler


@pytest.fixture
def serving() -> Iterator[Server]:
    """Start the stand-in on a free local port; stop it after the test."""
    state = Server(upload=b"")
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), _handler(state))
    state.url = f"http://127.0.0.1:{httpd.server_address[1]}"
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield state
    httpd.shutdown()
    httpd.server_close()


@pytest.fixture(scope="module")
def recording(tmp_path_factory: pytest.TempPathFactory) -> bytes:
    """Return the room as a Stray Scanner zip's bytes."""
    folder = tmp_path_factory.mktemp("recording")
    return stray_zip(folder / "room.zip", room.sequence(FRAMES)).read_bytes()


def test_a_job_runs_end_to_end(serving: Server, recording: bytes, tmp_path: Path) -> None:
    """Downloaded, imported, run in C++, baked, uploaded and reported, every stage in turn."""
    require_program("cpp")
    serving.upload = recording
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason is None
    assert serving.stages == list(worker.STAGES)
    assert serving.summary is not None
    assert serving.summary["frames"] == FRAMES
    assert serving.summary["intrinsics"] == "stray-scanner"
    info = json.loads(serving.files["bundle.json"])
    assert list(info["backends"]) == ["cpp"]
    assert (info["truth"], info["backends"]["cpp"]["alignment"]) == (True, "sim3")
    assert {"times.npy", "cpp/records.json", "cpp/feature_records.npy"} <= set(serving.files)
    assert list((tmp_path / "work").iterdir()) == []  # the job's folder removed


def test_an_upload_refused_fails_with_its_reason_and_no_path(
    serving: Server, tmp_path: Path
) -> None:
    """Not a video: failed with the import's own message, the worker's paths taken out."""
    serving.upload, serving.kind = b"not a video", "video"
    work = tmp_path / "work"
    worker.serve(worker.Client(serving.url, TOKEN), work, once=True)
    assert serving.summary is None
    assert serving.reason is not None
    assert "not a readable video" in serving.reason
    assert str(work) not in serving.reason
    assert serving.stages == ["downloading", "importing"]


def test_a_job_past_its_time_fails(serving: Server, recording: bytes, tmp_path: Path) -> None:
    """A child process still going at the deadline is stopped, and the job failed."""
    serving.upload = recording
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True, timeout=0.5)
    assert serving.reason == f"took longer than {0.5 / 60:g} minutes"
    assert serving.summary is None


def test_a_job_the_server_lets_go_is_dropped_quietly(
    serving: Server, recording: bytes, tmp_path: Path
) -> None:
    """A 409 on a report ends the job without a done or a failure."""
    serving.upload, serving.cancel_at = recording, "downloading"
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert (serving.summary, serving.reason) == (None, None)
    assert serving.stages == ["downloading"]


def test_an_upload_over_the_cap_is_refused(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The download stops past the cap, and the job fails saying so."""
    serving.upload = recording
    assert len(recording) > 500_000
    monkeypatch.setattr(video, "MAX_BYTES", 500_000)
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason == "the upload is over 0.5 MB"


def test_the_client_refuses_what_it_cannot_run(
    serving: Server, monkeypatch: pytest.MonkeyPatch
) -> None:
    """No token, a job of another kind, links for other files than asked."""
    with pytest.raises(InvalidInputError, match="expected the worker's token"):
        worker.Client(serving.url, "")
    serving.kind = "lidar"
    with pytest.raises(InvalidInputError, match="a job this worker cannot run"):
        worker.Client(serving.url, TOKEN).claim()
    assert worker.Client(serving.url, TOKEN).claim() is None  # no job left: 204
    client = worker.Client(serving.url, TOKEN)
    monkeypatch.setattr(client, "_post", lambda *_, **__: (200, {"urls": {"a": "x"}}))
    with pytest.raises(InvalidInputError, match="not for the files asked"):
        client.links(worker.Job("job-2", "vo", "video", ""), {"a": 1, "b": 2})


def test_process_prints_its_stages_and_its_reason(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The child's side: a stage per line, then why it stopped."""
    (tmp_path / "upload.video").write_text("not a video")
    assert worker.process(tmp_path / "upload.video", "video", tmp_path / "job") == 1
    lines = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert lines[0] == {"stage": "importing"}
    assert "not a readable video" in lines[-1]["reason"]


def test_a_child_out_of_memory_fails_as_an_internal_error(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Under a memory limit too small to start Python, the child ends without a result."""
    serving.upload = recording
    monkeypatch.setattr(worker, "MEMORY_LIMIT", 50 * 2**20)
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason == "an internal error"
    assert serving.stages == ["downloading"]
