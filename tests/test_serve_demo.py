"""scripts/serve_demo.py: settings the server takes, and a server and worker that stop together."""

from __future__ import annotations

import importlib.util
import os
import re
import signal
import socket
import stat
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np
import pytest

from slam_it_all_server.config import from_environment

REPO = Path(__file__).resolve().parents[1]


def _script() -> ModuleType:
    """Import scripts/serve_demo.py as a module."""
    spec = importlib.util.spec_from_file_location("serve_demo", REPO / "scripts/serve_demo.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_demos_settings_are_the_servers(tmp_path: Path) -> None:
    """The server reads them; everything is kept in the data folder; the secrets are new."""
    demo = _script()
    env = demo.environment(tmp_path, "a-demo-invite-code", "http://127.0.0.1:1")
    settings = from_environment(env)
    assert settings.storage == f"folder:{tmp_path / 'files'}"
    assert settings.database.parent == tmp_path
    assert (settings.viewer / "upload.html").is_file()
    assert not settings.staging
    assert settings.keep_uploads  # so that a recording can be run again
    again = demo.environment(tmp_path, "a-demo-invite-code", "http://127.0.0.1:1")
    assert again["SLAM_IT_ALL_WORKER_TOKEN"] != env["SLAM_IT_ALL_WORKER_TOKEN"]


def test_the_demos_own_invite_code_is_three_words_the_server_takes(tmp_path: Path) -> None:
    """Three of 256 words, of four letters each: short to type, long enough for the server."""
    demo = _script()
    assert len(set(demo.WORDS)) == len(demo.WORDS) == 256
    assert all(re.fullmatch("[a-z]{4}", word) for word in demo.WORDS)
    codes = {demo.invite_code() for _ in range(20)}
    assert len(codes) > 1  # new each time
    for code in codes:
        assert re.fullmatch("[a-z]{4}-[a-z]{4}-[a-z]{4}", code)
        env = demo.environment(tmp_path, code, "http://127.0.0.1:1")
        assert from_environment(env).invite_codes == (code,)


def _free_port() -> int:
    """Return a port nothing listens on now."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_the_demos_folder_is_the_same_every_run(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Without --data, the same folder each run, so a run deletes what an earlier one kept."""
    demo = _script()
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    assert demo.default_data() == tmp_path / "slam-it-all" / "demo"
    monkeypatch.delenv("XDG_DATA_HOME")
    assert demo.default_data() == Path.home() / ".local" / "share" / "slam-it-all" / "demo"


def test_a_port_in_use_is_refused(tmp_path: Path) -> None:
    """Something listening on the port already: refused, before anything starts."""
    with socket.socket() as other:
        other.bind(("127.0.0.1", 0))
        other.listen()
        port = other.getsockname()[1]
        command = [sys.executable, str(REPO / "scripts/serve_demo.py"), "--port", str(port)]
        tmp_path.chmod(0o755)  # as a folder a user gives may be
        done = subprocess.run(
            [*command, "--data", str(tmp_path)], capture_output=True, text=True, check=False
        )
    assert done.returncode == 1
    assert f"serve_demo: port {port} is in use" in done.stderr
    assert list(tmp_path.iterdir()) == []
    assert stat.S_IMODE(tmp_path.stat().st_mode) == 0o700  # its owner's alone


def test_the_demo_serves_until_interrupted(tmp_path: Path) -> None:
    """It serves the upload page with the code it was given; Ctrl-C stops the server and worker.

    The worker shares the script's output, so the output ends only once the worker has.
    """
    port = _free_port()
    command = [sys.executable, str(REPO / "scripts/serve_demo.py"), "--port", str(port)]
    profiles = REPO / "tests" / "fixtures" / "eval" / "lens_profiles"
    demo = subprocess.Popen(
        [*command, "--data", str(tmp_path), "--code", "a-demo-invite-code"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        env={**os.environ, "SLAM_IT_ALL_LENS_PROFILES": str(profiles)},
    )
    try:
        deadline = time.monotonic() + 30.0
        page = ""
        while time.monotonic() < deadline and not page:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1.0) as answer:
                    page = answer.read().decode()
            except (urllib.error.URLError, OSError):
                time.sleep(0.2)
        assert "SLAM It All! on your own video" in page
        time.sleep(1.0)  # the worker starts once the server answers
        demo.send_signal(signal.SIGINT)
        out, _ = demo.communicate(timeout=30.0)
    finally:
        if demo.poll() is None:
            demo.kill()
    assert demo.returncode == 0
    assert f"Open http://127.0.0.1:{port}/  invite code: a-demo-invite-code" in out
    assert "\x1b[30;47m" not in out  # no QR code: a phone cannot reach 127.0.0.1
    assert (tmp_path / "jobs.sqlite3").is_file()
    with socket.socket() as probe:  # nothing listens on it any more
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", port))
        probe.listen()


_MODULES = {
    " ": (False, False),
    "\u2580": (True, False),
    "\u2584": (False, True),
    "\u2588": (True, True),
}
"""Each character the QR code is drawn with, as its upper and lower module, dark or light."""


def test_the_qr_code_scans_as_the_pages_address() -> None:
    """Drawn back from its characters, the code the demo prints reads, to OpenCV, as the address.

    Each line is black on white, so the code reads the same on a dark terminal and a light one.
    """
    url = "http://192.168.1.20:8770/"
    lines = _script().qr_code(url).splitlines()
    assert all(line.startswith("\x1b[30;47m") and line.endswith("\x1b[0m") for line in lines)
    cells = [re.sub(r"\x1b\[[0-9;]*m", "", line) for line in lines]
    rows = [[_MODULES[c][half] for c in line] for line in cells for half in (0, 1)]
    image = np.where(np.kron(np.array(rows), np.ones((8, 8), dtype=bool)), 0, 255).astype(np.uint8)
    # OpenCV's detector finds no code at an image's edge, whatever its quiet zone: a camera sees
    # the terminal around it, so a white margin stands in for that.
    framed = cv2.copyMakeBorder(image, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    found, _, _ = cv2.QRCodeDetector().detectAndDecode(framed)
    assert found == url


class _Process:
    """A stand-in for a running process: it ends after a number of polls."""

    def __init__(self, polls: int, code: int) -> None:
        self.polls, self.code = polls, code
        self.returncode: int | None = None

    def poll(self) -> int | None:
        self.polls -= 1
        if self.polls < 0:
            self.returncode = self.code
        return self.returncode


def test_the_demo_stops_when_either_of_its_processes_does(monkeypatch: pytest.MonkeyPatch) -> None:
    """The server stopping gives its code; the worker stopping first gives 1."""
    demo = _script()
    monkeypatch.setattr(demo.time, "sleep", lambda _: None)
    assert demo.watch(_Process(2, 3), _Process(10, 0)) == 3
    assert demo.watch(_Process(10, 0), _Process(2, 0)) == 1


STUBBORN = (
    "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
    "print('ready', flush=True); time.sleep(60)"
)
"""A process that ignores a SIGTERM."""


def test_each_process_is_given_its_time_then_killed(monkeypatch: pytest.MonkeyPatch) -> None:
    """A SIGTERM, then STOP_WAIT seconds to finish; a process still there is then killed."""
    demo = _script()
    monkeypatch.setattr(demo, "STOP_WAIT", 0.5)
    with subprocess.Popen([sys.executable, "-c", STUBBORN], stdout=subprocess.PIPE) as stubborn:
        assert stubborn.stdout is not None
        assert stubborn.stdout.readline() == b"ready\n"
        demo._end(stubborn)
    assert stubborn.returncode == -signal.SIGKILL
    with subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]) as willing:
        demo._end(willing)
    assert willing.returncode == -signal.SIGTERM


def test_on_the_local_network_every_page_asks_for_the_code(tmp_path: Path) -> None:
    """With --lan, the settings are staging's: every page behind the invite code."""
    env = _script().environment(tmp_path, "a-demo-invite-code", "http://1.2.3.4:1", staging=True)
    assert from_environment(env).staging
