"""The worker, against a stand-in for the upload server: its protocol on a local port.

The stand-in serves one job's upload, takes the bundle's files by PUT, and records every
report. The job runs the real import, C++ pipeline and bake, on the room.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
from collections.abc import Iterator
from dataclasses import dataclass, field, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slam_it.edge.options import Options
from slam_it.eval import tum, video
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo, worker
from tests.conftest import FIXTURES
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS
from tests.eval.recordings import stray_zip
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import Room, blank, write_room

TOKEN = "a-worker-token"


@pytest.fixture(autouse=True)
def profiles(monkeypatch: pytest.MonkeyPatch) -> None:
    """Point the worker, and the children it starts, at the fixture lens profiles."""
    monkeypatch.setenv("SLAM_IT_LENS_PROFILES", str(FIXTURES / "eval" / "lens_profiles"))


@dataclass
class Server:
    """What the stand-in holds and was told."""

    upload: bytes
    kind: str = "stray"
    cancel_at: str | None = None
    fail_at: str | None = None
    refuse_failed: bool = False
    slow: bool = False
    put_delay: float = 0.0
    download: str | None = None
    redirect: str | None = None
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

        def _redirect(self) -> None:
            self.send_response(302)
            self.send_header("Location", str(state.redirect))
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self) -> None:
            """Serve the upload, slowly if asked; at ``/moved``, a redirect."""
            if self.path == "/moved":
                self._redirect()
                return
            self.send_response(200)
            self.send_header("Content-Length", str(len(state.upload)))
            self.end_headers()
            step = 1 << 16 if state.slow else len(state.upload) or 1
            for start in range(0, len(state.upload), step):
                self.wfile.write(state.upload[start : start + step])
                if state.slow:
                    time.sleep(0.01)

        def do_PUT(self) -> None:
            """Take one of the bundle's files, after a delay if asked."""
            path = urllib.parse.unquote(self.path.removeprefix("/put/"))
            state.files[path] = self._body()
            time.sleep(state.put_delay)
            self._answer(200)

        def do_POST(self) -> None:
            """Answer a claim or a report."""
            if self.headers.get("Authorization") != f"Bearer {TOKEN}":
                self._answer(401)
                return
            body = json.loads(self._body() or b"{}")
            if state.redirect:
                self._redirect()
                return
            if self.path == "/api/worker/claim":
                if not state.jobs:
                    self._answer(204)
                    return
                job = state.jobs.pop()
                upload = {"job": job, "kind": "vo", "upload": state.kind}
                link = state.download or f"{state.url}/upload"
                self._answer(200, {**upload, "download": link})
                return
            action = self.path.rsplit("/", 1)[-1]
            if action == "progress":
                state.stages.append(body["stage"])
                if body["stage"] == state.fail_at:
                    self._answer(500, {})
                    return
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
            elif state.refuse_failed:
                self._answer(409, {})
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
    """Return the room as a Stray Scanner zip's bytes, at its 30 fps setting: every frame kept."""
    folder = tmp_path_factory.mktemp("recording")
    return stray_zip(folder / "room.zip", room.sequence(FRAMES), divider=2).read_bytes()


def test_a_job_runs_end_to_end(serving: Server, recording: bytes, tmp_path: Path) -> None:
    """Downloaded, imported, run in C++, baked, uploaded and reported, every stage in turn."""
    require_program("cpp")
    serving.upload = recording
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason is None
    assert _distinct(serving.stages) == list(worker.STAGES)
    assert serving.summary is not None
    assert serving.summary["frames"] == FRAMES
    assert serving.summary["intrinsics"] == "stray-scanner"
    assert serving.summary["mode"] == "monocular"
    assert serving.summary["lost"] is False
    assert serving.summary["ran_to"] == serving.summary["clip_seconds"]
    info = json.loads(serving.files["bundle.json"])
    assert list(info["backends"]) == ["cpp"]
    assert info["sequence"] == worker.SEQUENCE == "your-video"  # the viewer's name for it
    assert (info["truth"], info["backends"]["cpp"]["alignment"]) == (True, "sim3")
    assert {"times.npy", "cpp/records.json", "cpp/feature_records.npy"} <= set(serving.files)
    assert list((tmp_path / "work").iterdir()) == []  # the job's folder removed


def test_a_recording_with_depth_runs_rgbd(serving: Server, tmp_path: Path) -> None:
    """A Stray Scanner zip with LiDAR depth runs RGB-D: tracked from its first frame, metric."""
    require_program("cpp")
    zipped = stray_zip(tmp_path / "room.zip", room.sequence(FRAMES), divider=2, depth=True)
    serving.upload = zipped.read_bytes()
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason is None
    assert serving.summary is not None
    assert serving.summary["mode"] == "rgbd"
    assert (serving.summary["tracked"], serving.summary["tracked_from"]) == (FRAMES, 0.0)
    info = json.loads(serving.files["bundle.json"])
    assert (info["mode"], info["backends"]["cpp"]["alignment"]) == ("rgbd", "se3")


def test_the_summary_says_where_a_run_lost_the_track(
    rendered_room: list[room.Frame], tmp_path: Path
) -> None:
    """Blank frames lose the track, which ends the run: the rest of the clip is not run.

    The summary says so, and when tracking started and stopped, in the clip's own time.
    """
    blanks = OPTIONS.lost_frames
    clip = [
        *rendered_room[:10],
        *map(blank, rendered_room[10 : 10 + blanks]),
        *rendered_room[10 + blanks :],
    ]
    folders = write_room(tmp_path / "clip", clip)
    run = vo.run("tum", folders.tum, "rgbd", tmp_path / "run", options=OPTIONS)
    times = folders.timestamps - folders.timestamps[0]
    summary = worker.summarize(run, folders.timestamps, "nominal")
    assert summary["frames"] == 10 + blanks < FRAMES
    assert summary["lost"] is True
    assert (summary["tracked_from"], summary["tracked_to"]) == (0.0, round(times[9], 3))
    assert (summary["ran_to"], summary["clip_seconds"]) == (
        round(times[9 + blanks], 3),
        round(times[-1], 3),
    )


def test_the_summary_of_a_whole_run_says_when_tracking_began(
    room_runs: RoomRun, room_folders: Room
) -> None:
    """A monocular run tracks from its first map on, and to the clip's end if not lost."""
    run = room_runs("tum", "monocular", "python")
    times = room_folders.timestamps
    summary = worker.summarize(run, times, "nominal")
    assert (summary["frames"], summary["lost"]) == (FRAMES, False)
    assert summary["ran_to"] == summary["clip_seconds"] == round(times[-1] - times[0], 3)
    assert 0 < summary["tracked_from"] < summary["tracked_to"] <= summary["ran_to"]
    assert summary["tracked"] == len(run.timestamps)


def _distinct(stages: list[str]) -> list[str]:
    """Return the stages reported, each run of repeats (heartbeats) as one."""
    return [s for i, s in enumerate(stages) if i == 0 or s != stages[i - 1]]


def test_a_long_stage_is_reported_again_as_the_jobs_heartbeat(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """With a short heartbeat, a stage the child takes a while over is reported more than once."""
    require_program("cpp")
    serving.upload = recording
    monkeypatch.setattr(worker, "HEARTBEAT", 0.05)
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.summary is not None
    assert _distinct(serving.stages) == list(worker.STAGES)
    assert len(serving.stages) > len(worker.STAGES)


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
    assert _distinct(serving.stages) == ["downloading", "importing"]


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
    links = {"urls": {"a": "http://127.0.0.1:9/a"}}
    monkeypatch.setattr(client, "_post", lambda *_, **__: (200, links))
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


def test_only_the_jobs_own_refusals_are_its_reason(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """A missing upload is a reason; a file missing from the worker's own setup is not."""
    assert worker.process(tmp_path / "none.zip", "stray", tmp_path / "job") == 1
    last = capsys.readouterr().out.splitlines()[-1]
    assert json.loads(last) == {"reason": "the upload is missing"}
    (tmp_path / "upload.zip").write_bytes(b"x")

    def missing(*_: object) -> dict[str, Any]:
        raise FileNotFoundError(2, "No such file or directory", "/opt/worker/bin/cmake")

    monkeypatch.setattr(worker, "_work", missing)
    with pytest.raises(FileNotFoundError):  # the worker's: an internal error, logged
        worker.process(tmp_path / "upload.zip", "stray", tmp_path / "job")
    assert "reason" not in capsys.readouterr().out


def test_a_child_out_of_memory_fails_as_an_internal_error(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Under a memory limit too small to start Python, the child ends without a result."""
    serving.upload = recording
    monkeypatch.setattr(worker, "MEMORY_LIMIT", 50 * 2**20)
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason == "an internal error"
    assert _distinct(serving.stages) == ["downloading"]


def test_a_report_that_does_not_reach_the_server_lets_the_job_go_on(
    serving: Server, recording: bytes, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """A 500 on the "running" report is logged, and the job still ends done."""
    require_program("cpp")
    serving.upload, serving.fail_at = recording, "running"
    with caplog.at_level(logging.WARNING, logger=worker.__name__):
        worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.summary is not None
    assert serving.reason is None
    assert "the running report did not reach the server" in caplog.text


def test_a_failure_report_the_server_refuses_does_not_stop_the_worker(
    serving: Server, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """A 409 on the failure report (the job let go meanwhile) is logged; serve returns."""
    serving.upload, serving.kind, serving.refuse_failed = b"not a video", "video", True
    with caplog.at_level(logging.WARNING, logger=worker.__name__):
        worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason is None
    assert "the failure was not reported" in caplog.text


def test_an_internal_error_is_logged_for_the_operator(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:  # fmt: skip
    """The owner is told "an internal error"; the log has the job's id and the traceback."""
    require_program("cpp")
    serving.upload = recording

    def broken(*_: object) -> None:
        raise RuntimeError("the upload broke")

    monkeypatch.setattr(worker, "upload", broken)
    with caplog.at_level(logging.ERROR, logger=worker.__name__):
        worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.reason == "an internal error"
    assert "job job-1: an internal error" in caplog.text
    assert "RuntimeError: the upload broke" in caplog.text


def test_transfers_beat_as_the_jobs_heartbeat(
    serving: Server, recording: bytes, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A slow download and slow uploads report their stage again, as the child's stages do."""
    require_program("cpp")
    serving.upload, serving.slow, serving.put_delay = recording, True, 0.06
    monkeypatch.setattr(worker, "HEARTBEAT", 0.05)
    monkeypatch.setattr(worker, "_CHUNK", 1 << 14)
    worker.serve(worker.Client(serving.url, TOKEN), tmp_path / "work", once=True)
    assert serving.summary is not None
    assert serving.stages.count("downloading") > 1
    assert serving.stages.count("uploading") > 1


STAND_IN = """
import json, subprocess, sys, time
pidfile, mode = sys.argv[1], sys.argv[2]
print(json.dumps({"stage": "running"}), flush=True)
sleeper = subprocess.Popen(["sleep", "30"])
with open(pidfile, "w") as file:
    file.write(str(sleeper.pid))
if mode == "orphan":
    print(json.dumps({"summary": {"frames": 1}}), flush=True)
    sys.exit(0)
time.sleep(30)
"""
"""A stand-in for a job's child: reports a stage, starts a grandchild, then waits or ends."""


def _alive(pid: int) -> bool:
    """Whether a process runs (a zombie has ended)."""
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except (FileNotFoundError, ProcessLookupError):  # gone, or going as it is read
        return False
    return state != "Z"


def _gone(pid: int, seconds: float = 5.0) -> bool:
    """Whether a process ends within a few seconds."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not _alive(pid):
            return True
        time.sleep(0.05)
    return False


@pytest.mark.parametrize("ending", ["timeout", "abandoned", "orphan"])
def test_a_job_leaves_no_process_behind(ending: str, tmp_path: Path) -> None:
    """Past its deadline, let go by a 409, or ending with an orphan: its whole group is killed.

    The stand-in child starts a grandchild, as vo.run starts the C++ program.
    """
    script, pidfile = tmp_path / "child.py", tmp_path / "grandchild.pid"
    script.write_text(STAND_IN)
    argv = [sys.executable, str(script), str(pidfile), "orphan" if ending == "orphan" else "wait"]

    def report(stage: str) -> None:
        if ending == "abandoned" and stage == "running":
            deadline = time.monotonic() + 5.0
            while not pidfile.exists() and time.monotonic() < deadline:
                time.sleep(0.01)  # the grandchild has started
            raise worker.AbandonedError(stage)

    if ending == "timeout":
        with pytest.raises(worker.RefusedError, match="took longer"):
            worker._run_child(argv, report, 2.0)
    elif ending == "abandoned":
        with pytest.raises(worker.AbandonedError):
            worker._run_child(argv, report, 30.0)
    else:
        assert worker._run_child(argv, report, 30.0) == {"frames": 1}
    assert _gone(int(pidfile.read_text()))


STOPPED = """
import contextlib, sys
from pathlib import Path
from slam_it.orchestration import worker

class Client:
    def progress(self, job, stage):
        pass

    def failed(self, job, reason):
        pass

stand_in, pidfile, work = sys.argv[1:4]
run_child = worker._run_child
worker.download = lambda url, path, cap, beat: path.write_bytes(b"upload")
worker._run_child = lambda argv, report, timeout: run_child(
    [sys.executable, stand_in, pidfile, "wait"], report, timeout
)
with contextlib.suppress(KeyboardInterrupt), worker._stopping_on_signals():
    worker.handle(Client(), worker.Job("job-1", "vo", "stray", ""), Path(work))
print("stopped", flush=True)
"""
"""A worker, its signals set as ``serve`` sets them, running one job whose child is the
stand-in."""


@pytest.mark.parametrize("signals", [("SIGTERM",), ("SIGINT", "SIGTERM")])
def test_a_stopped_worker_leaves_no_process_or_folder_behind(
    signals: tuple[str, ...], tmp_path: Path
) -> None:
    """Stopped by a SIGTERM, or by a Ctrl-C and at once the demo's SIGTERM: the job goes too.

    The job runs in a session of its own, which no signal to the worker reaches, so only the
    worker's clean-up ends it; the second signal must not cut that short.
    """
    script, stand_in, pidfile = tmp_path / "worker.py", tmp_path / "child.py", tmp_path / "pid"
    work = tmp_path / "work"
    script.write_text(STOPPED)
    stand_in.write_text(STAND_IN)
    work.mkdir()
    argv = [sys.executable, str(script), str(stand_in), str(pidfile), str(work)]
    with subprocess.Popen(argv, stdout=subprocess.PIPE, text=True) as running:
        deadline = time.monotonic() + 20.0
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(0.01)  # the job's grandchild has started
        for name in signals:
            running.send_signal(getattr(signal, name))
        out, _ = running.communicate(timeout=20.0)
    assert out.strip() == "stopped"
    assert _gone(int(pidfile.read_text()))
    assert list(work.iterdir()) == []


def test_the_worker_sets_its_signal_handlers_only_while_it_serves(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Its own while serve runs; the caller's again once main returns."""
    before = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
    seen: list[object] = []
    monkeypatch.setattr(
        worker, "serve", lambda *_, **__: seen.append(signal.getsignal(signal.SIGTERM))
    )
    monkeypatch.setenv(worker.TOKEN_VARIABLE, TOKEN)
    assert worker.main(["serve", "--server", "http://127.0.0.1:9"]) == 0
    assert seen == [worker._stop]
    assert (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)) == before


def test_the_child_inherits_no_secret(monkeypatch: pytest.MonkeyPatch) -> None:
    """Only CHILD_ENVIRONMENT's variables pass: never the worker's token, nor anything else."""
    monkeypatch.setenv(worker.TOKEN_VARIABLE, TOKEN)
    monkeypatch.setenv("SOME_CLOUD_SECRET", "s3cr3t")
    env = worker.child_environment()
    assert worker.TOKEN_VARIABLE not in env
    assert "SOME_CLOUD_SECRET" not in env
    assert env["PATH"] == os.environ["PATH"]
    assert env["SLAM_IT_LENS_PROFILES"] == str(FIXTURES / "eval" / "lens_profiles")


@pytest.mark.parametrize(
    ("url", "good"),
    [
        ("https://slam.example.org", True),
        ("http://127.0.0.1:8000", True),
        ("http://[::1]:8000", True),
        ("http://localhost:8000", True),
        ("http://192.168.1.20:8770", True),
        ("http://slam-it.railway.internal:8000", True),
        ("http://sia-e2e-server:8000", True),
        ("http://2130706433:8000", True),  # 127.0.0.1, written as a number
        ("http://134744072/", False),  # 8.8.8.8, written as a number
        ("http://0x8080808/", False),  # and in hexadecimal
        ("http://8%2e8%2e8%2e8/", False),  # 8.8.8.8, its dots escaped: urllib unescapes them
        ("http://127%2e0%2e0%2e1:8000", True),  # judged as urllib connects: 127.0.0.1
        ("http://example%2ecom/", False),
        ("http://[2001:4860:4860::8888%25lo]/", False),  # a public address with a scope id
        ("http://[fe80::1%25eth0]:8000", False),  # any scope id
        ("http://169.254.169.254/", False),  # link-local: a cloud's metadata service
        ("http://[2002:808:808::1]/", False),  # 6to4, for 8.8.8.8
        ("http://[fd12:3456::1]:8000", True),  # IPv6 unique-local, as a private network gives
        ("http://\u00ad/", False),  # a host IDNA cannot encode
        ("http://[::1/", False),  # brackets that hold no address
        ("http://slam.example.org", False),
        ("http://8.8.8.8", False),
        ("file:///etc/passwd", False),
        ("ftp://127.0.0.1/x", False),
        ("", False),
    ],
)
def test_only_safe_urls_are_followed(url: str, good: bool) -> None:
    """An https URL goes anywhere; plain http only to a host off the public internet."""
    if good:
        assert worker.check_url(url, "server") == url
    else:
        with pytest.raises(InvalidInputError):
            worker.check_url(url, "server")


def test_a_token_the_server_refuses_ends_the_worker(
    serving: Server, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A 401 on a claim is a wrong token, which no retry mends: serve stops, main exits 2."""
    wrong = "a-wrong-token-of-at-least-32-characters"
    with pytest.raises(InvalidInputError, match="token: the server refused it"):
        worker.serve(worker.Client(serving.url, wrong), tmp_path / "work", once=True)
    monkeypatch.setenv(worker.TOKEN_VARIABLE, wrong)
    assert worker.main(["serve", "--server", serving.url, "--work", str(tmp_path / "w")]) == 2
    assert serving.jobs == ["job-1"]  # nothing was claimed


def test_a_job_whose_folder_cannot_be_made_fails_and_the_worker_goes_on(
    serving: Server, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """A full or unwritable work folder fails the job as an internal error, logged; no raise."""
    work = tmp_path / "work"
    work.write_text("a file, not a folder")
    job = worker.Job("job-1", "vo", "stray", f"{serving.url}/upload")
    with caplog.at_level(logging.ERROR, logger="slam_it.orchestration.worker"):
        worker.handle(worker.Client(serving.url, TOKEN), job, work)
    assert serving.reason == "an internal error"
    assert "job job-1: an internal error" in caplog.text


def test_the_server_is_never_followed_through_a_redirect(serving: Server) -> None:
    """A 3xx from the server's API is an error, never followed: the token would go along."""
    serving.redirect = f"{serving.url}/elsewhere"
    with pytest.raises(urllib.error.HTTPError, match="302"):
        worker.Client(serving.url, TOKEN).claim()


@pytest.mark.parametrize(
    ("target", "good"),
    [
        ("{url}/upload", True),
        ("http://8.8.8.8/x", False),
        ("http://8%2e8%2e8%2e8/x", False),
        ("ftp://127.0.0.1:9/x", False),
    ],
)
def test_a_downloads_redirect_goes_only_where_a_link_may(
    target: str, good: bool, serving: Server, tmp_path: Path
) -> None:
    """A download follows a redirect only to a URL check_url accepts."""
    serving.upload, serving.redirect = b"bytes", target.format(url=serving.url)
    path = tmp_path / "upload.zip"
    if good:
        worker.download(f"{serving.url}/moved", path, 100)
        assert path.read_bytes() == b"bytes"
    else:
        with pytest.raises(InvalidInputError, match="download's redirect"):
            worker.download(f"{serving.url}/moved", path, 100)


def test_a_download_link_to_a_local_file_is_refused(serving: Server) -> None:
    """A claim whose upload link is ``file://`` is refused before anything is read."""
    serving.download = "file:///etc/passwd"
    with pytest.raises(InvalidInputError, match="download link"):
        worker.Client(serving.url, TOKEN).claim()


class _Claimed(BaseException):
    """A claim the test did not expect: the worker went past the checks it makes first."""


def _claim_unexpected(*_: object, **__: object) -> None:
    raise _Claimed


def test_the_worker_refuses_to_start_wrong(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """No server address, a public plain-http one, or no lens profiles: code 2, and why.

    Each is refused before any claim: one would raise here at once, never wait on a server.
    """
    monkeypatch.setattr(worker.Client, "claim", _claim_unexpected)
    monkeypatch.setenv(worker.TOKEN_VARIABLE, TOKEN)
    assert worker.main(["serve", "--server", "", "--work", str(tmp_path)]) == 2
    assert "expected an http or https URL" in capsys.readouterr().err
    assert worker.main(["serve", "--server", "http://slam.example.org"]) == 2
    assert "plain http only to a private host" in capsys.readouterr().err
    monkeypatch.setenv("SLAM_IT_LENS_PROFILES", str(tmp_path))
    assert worker.main(["serve", "--server", "http://127.0.0.1:9", "--work", str(tmp_path)]) == 2
    assert "no usable lens profile" in capsys.readouterr().err


def test_a_job_runs_with_features_for_its_frames_size(tmp_path: Path) -> None:
    """A 16:9 clip imported at 854x480 runs with 1,334 features; its other settings, ORB-SLAM's."""
    intrinsics = np.array([600.0, 600.0, 426.5, 239.5])
    tum.write_camera(tmp_path, tum.Camera("pinhole", intrinsics, 854, 480, 1000.0, None))
    assert worker.options_for(tmp_path) == replace(Options(), n_features=1334)
