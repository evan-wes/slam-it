"""``python -m slam_it_server``: it refuses missing settings, and stops promptly when asked."""

from __future__ import annotations

import contextlib
import json
import os
import signal
import socket
import sqlite3
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import replace
from pathlib import Path

import pytest

from slam_it_server import __main__ as entry
from slam_it_server.__main__ import SHUTDOWN, listening, server_config
from tests.server.conftest import CODE, TOKEN, VIEWER, make_settings


def test_missing_settings_end_it_with_code_2() -> None:
    """With no settings in the environment, it says which is missing and exits with 2."""
    env = {k: v for k, v in os.environ.items() if not k.startswith("SLAM_IT_")}
    done = subprocess.run(
        [sys.executable, "-m", "slam_it_server"],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert done.returncode == 2
    assert "SLAM_IT_INVITE_CODES is not set" in done.stderr


def test_it_refuses_to_run_as_root(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """As root it says so and exits with 2, before reading a setting."""
    monkeypatch.setattr(os, "geteuid", lambda: 0)
    assert entry.main([]) == 2
    assert capsys.readouterr().err == "slam_it_server: refusing to run as root\n"


def test_a_job_list_of_another_format_ends_it_with_code_2(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """An earlier server's job list: named, and exit 2, not a traceback."""
    database = tmp_path / "jobs.sqlite3"
    made = sqlite3.connect(database)
    made.execute("CREATE TABLE jobs (id TEXT PRIMARY KEY)")
    made.commit()
    made.close()
    settings = make_settings(tmp_path)
    monkeypatch.setattr(entry, "from_environment", lambda: replace(settings, database=database))
    assert entry.main([]) == 2
    assert f"{database}: a job list of format 0" in capsys.readouterr().err


def test_a_viewer_without_its_page_ends_it_with_code_2_before_the_job_list_is_made(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """A wrong SLAM_IT_VIEWER: named, exit 2, and no job list made."""
    settings = replace(make_settings(tmp_path), viewer=tmp_path / "no-viewer")
    monkeypatch.setattr(entry, "from_environment", lambda: settings)
    assert entry.main([]) == 2
    assert "SLAM_IT_VIEWER" in capsys.readouterr().err
    assert not settings.database.exists()


def _claim(url: str) -> None:
    """Hold a worker's claim open for up to 30 s."""
    request = urllib.request.Request(
        f"{url}/api/worker/claim",
        json.dumps({"kinds": ["vo"], "wait": 30}).encode(),
        {"Authorization": f"Bearer {TOKEN}", "Content-Type": "application/json"},
    )
    with contextlib.suppress(urllib.error.URLError, OSError):  # cut off by the server stopping
        urllib.request.urlopen(request, timeout=40.0).close()


def test_a_held_claim_does_not_hold_up_stopping(tmp_path: Path) -> None:
    """Asked to stop while a claim waits, it stops within SHUTDOWN, not the claim's wait."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    url = f"http://127.0.0.1:{port}"
    env = {
        **os.environ,
        "SLAM_IT_INVITE_CODES": CODE,
        "SLAM_IT_WORKER_TOKEN": TOKEN,
        "SLAM_IT_SIGNING_KEY": "a-signing-key-of-at-least-32-characters",
        "SLAM_IT_STORAGE": f"folder:{tmp_path / 'files'}",
        "SLAM_IT_PUBLIC_URL": url,
        "SLAM_IT_DATABASE": str(tmp_path / "jobs.sqlite3"),
        "SLAM_IT_VIEWER": str(VIEWER),
    }
    server = subprocess.Popen(
        [sys.executable, "-m", "slam_it_server", "--port", str(port)],
        env=env,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            try:
                urllib.request.urlopen(f"{url}/healthz", timeout=1.0).close()
                break
            except (urllib.error.URLError, OSError):
                time.sleep(0.1)
        threading.Thread(target=_claim, args=(url,), daemon=True).start()
        time.sleep(0.5)
        start = time.monotonic()
        server.send_signal(signal.SIGTERM)
        server.wait(timeout=20.0)
        assert time.monotonic() - start < SHUTDOWN + 5.0
    finally:
        if server.poll() is None:
            server.kill()
            server.wait()


def test_the_host_colons_listens_on_ipv6_and_ipv4() -> None:
    """``::`` takes connections on both; an IPv4 address on IPv4 alone."""
    both = listening("::", 0)
    try:
        both.listen()
        port = both.getsockname()[1]
        for family, address in ((socket.AF_INET, "127.0.0.1"), (socket.AF_INET6, "::1")):
            with socket.socket(family) as client:
                client.connect((address, port))
    finally:
        both.close()
    alone = listening("127.0.0.1", 0)
    try:
        assert alone.family == socket.AF_INET
    finally:
        alone.close()


def test_uvicorn_keeps_no_access_log(tmp_path: Path) -> None:
    """Uvicorn keeps no access log: a request's path holds a token or a signed link.

    The server's own lines name jobs by their id instead.
    """
    settings = make_settings(tmp_path)
    config = server_config(settings)
    assert config.access_log is False
    assert config.server_header is False
    assert config.timeout_graceful_shutdown == SHUTDOWN
