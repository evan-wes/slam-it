"""scripts/serve_demo.py: settings the server takes, and a server and worker that stop together."""

from __future__ import annotations

import importlib.util
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from types import ModuleType

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
    again = demo.environment(tmp_path, "a-demo-invite-code", "http://127.0.0.1:1")
    assert again["SLAM_IT_ALL_WORKER_TOKEN"] != env["SLAM_IT_ALL_WORKER_TOKEN"]


def _free_port() -> int:
    """Return a port nothing listens on now."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_a_port_in_use_is_refused(tmp_path: Path) -> None:
    """Something listening on the port already: refused, before anything starts."""
    with socket.socket() as other:
        other.bind(("127.0.0.1", 0))
        other.listen()
        port = other.getsockname()[1]
        command = [sys.executable, str(REPO / "scripts/serve_demo.py"), "--port", str(port)]
        done = subprocess.run(
            [*command, "--data", str(tmp_path)], capture_output=True, text=True, check=False
        )
    assert done.returncode == 1
    assert f"serve_demo: port {port} is in use" in done.stderr
    assert list(tmp_path.iterdir()) == []


def test_the_demo_serves_until_interrupted(tmp_path: Path) -> None:
    """It serves the upload page with the code it was given; Ctrl-C stops the server and worker.

    The worker shares the script's output, so the output ends only once the worker has.
    """
    port = _free_port()
    command = [sys.executable, str(REPO / "scripts/serve_demo.py"), "--port", str(port)]
    demo = subprocess.Popen(
        [*command, "--data", str(tmp_path), "--code", "a-demo-invite-code"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
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
    assert (tmp_path / "jobs.sqlite3").is_file()
    with socket.socket() as probe:  # nothing listens on it any more
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", port))
        probe.listen()
