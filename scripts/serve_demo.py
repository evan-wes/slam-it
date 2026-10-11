"""Run "your own video" on this machine: the upload server, with a folder as storage, and a worker.

docs/design/2026-10-09-run-viewer.md, section 5.3: the demo, before anything is deployed. The
job list, the uploads and the results all go in one folder (``--data``, by default a new one in
the system's temporary folder). The worker token and the signing key are made afresh for each
run; the invite code is too, unless given, and is printed with the page's address.

With ``--lan`` the server listens on every interface, and its links use this machine's address
on the local network, so a phone on the same Wi-Fi can open the page and upload. Every page then
asks for the invite code (HTTP Basic auth: any name, the code as the password), as staging does.
Ctrl-C, or a SIGTERM, stops the server and the worker together, and the server stops if the
worker does. The worker inherits only what it needs of this shell's environment.

Example::

    python scripts/serve_demo.py --lan
"""

from __future__ import annotations

import argparse
import contextlib
import os
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

from slam_it_all.orchestration import worker as worker_module

REPO = Path(__file__).resolve().parent.parent
"""The repository, whose viewer folder holds the pages."""

PORT = 8770
"""The port the server listens on, by default."""


def lan_address() -> str:
    """Return this machine's address on the local network.

    Connecting a UDP socket sends nothing; it only picks the interface a packet would leave by.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect(("192.168.0.1", 9))
        return str(probe.getsockname()[0])


def environment(data: Path, code: str, url: str, *, staging: bool = False) -> dict[str, str]:
    """Return the server's settings for a demo keeping everything in `data`.

    Parameters
    ----------
    data : pathlib.Path
        The folder for the job list, the uploads and the results.
    code : str
        The invite code.
    url : str
        The server's address, which folder storage's links point at.
    staging : bool, optional
        Whether every page asks for the invite code.

    Returns
    -------
    dict of str to str
        The ``SLAM_IT_ALL_*`` variables (:mod:`slam_it_all_server.config`), its secrets new.
    """
    return {
        "SLAM_IT_ALL_INVITE_CODES": code,
        "SLAM_IT_ALL_WORKER_TOKEN": secrets.token_urlsafe(32),
        "SLAM_IT_ALL_SIGNING_KEY": secrets.token_urlsafe(32),
        "SLAM_IT_ALL_STORAGE": f"folder:{data / 'files'}",
        "SLAM_IT_ALL_PUBLIC_URL": url,
        "SLAM_IT_ALL_DATABASE": str(data / "jobs.sqlite3"),
        "SLAM_IT_ALL_VIEWER": str(REPO / "viewer"),
        "SLAM_IT_ALL_STAGING": "1" if staging else "0",
    }


def _stop(signum: int, frame: object) -> None:
    """End the script on a SIGTERM as on Ctrl-C, so that it stops its server and worker."""
    del signum, frame
    raise KeyboardInterrupt


def _refuse_a_used_port(host: str, port: int) -> None:
    """Refuse a port something already listens on, whose health check would pass for ours.

    ``SO_REUSEADDR``, as the server sets it, lets connections of a run just stopped linger.
    """
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, port))
        except OSError:
            raise SystemExit(f"serve_demo: port {port} is in use") from None


def watch(server: subprocess.Popen[bytes], worker: subprocess.Popen[bytes]) -> int:
    """Wait while both run; return the server's code if it stops, 1 if the worker does first."""
    while True:
        if server.poll() is not None:
            return int(server.returncode)
        if worker.poll() is not None:
            print("serve_demo: the worker stopped, so the demo stops too", file=sys.stderr)
            return 1
        time.sleep(0.5)


def _wait_until_up(url: str, child: subprocess.Popen[bytes], seconds: float = 30.0) -> None:
    """Wait for the server's health check to answer, or for it to have ended."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and child.poll() is None:
        with (
            contextlib.suppress(urllib.error.URLError, OSError),
            urllib.request.urlopen(f"{url}/healthz", timeout=1.0),
        ):
            return
        time.sleep(0.2)
    raise SystemExit("serve_demo: the server did not start")


def main(arguments: list[str]) -> int:
    """Run the server and a worker until Ctrl-C (the module docstring).

    Parameters
    ----------
    arguments : list of str
        The arguments after the script's name.

    Returns
    -------
    int
        0 once stopped; the server's code if it stopped first; 1 if the worker did.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--lan", action="store_true", help="listen on the local network too")
    parser.add_argument("--code", help="the invite code (default: a new one)")
    parser.add_argument("--data", type=Path, help="the folder to keep everything in")
    args = parser.parse_args(arguments)
    data = args.data or Path(tempfile.mkdtemp(prefix="slam-it-all-demo-"))
    data.mkdir(parents=True, exist_ok=True)
    code = args.code or secrets.token_urlsafe(9)
    address = lan_address() if args.lan else "127.0.0.1"
    url = f"http://{address}:{args.port}"
    settings = environment(data, code, url, staging=args.lan)
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    _refuse_a_used_port(host, args.port)
    # Ctrl-C works even when started with SIGINT ignored (in the background of a script).
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _stop)
    server = subprocess.Popen(
        [sys.executable, "-m", "slam_it_all_server", "--host", host, "--port", str(args.port)],
        env={**os.environ, **settings},
    )
    worker = None
    try:
        _wait_until_up(f"http://127.0.0.1:{args.port}", server)
        inherited = {k: os.environ[k] for k in worker_module.CHILD_ENVIRONMENT if k in os.environ}
        token = {worker_module.TOKEN_VARIABLE: settings["SLAM_IT_ALL_WORKER_TOKEN"]}
        command = [sys.executable, "-m", "slam_it_all.orchestration.worker", "serve"]
        worker = subprocess.Popen(
            [*command, "--server", f"http://127.0.0.1:{args.port}", "--work", str(data / "work")],
            env={**inherited, **token},
        )
        print(f"\nOpen {url}/  invite code: {code}\nEverything is kept in {data}\n", flush=True)
        return watch(server, worker)
    except KeyboardInterrupt:
        return 0
    finally:
        for child in (worker, server):
            if child is not None and child.poll() is None:
                child.terminate()
                child.wait(10)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
