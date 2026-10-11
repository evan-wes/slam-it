"""The worker: runs the upload server's jobs, holding nothing but its token.

docs/design/2026-10-09-run-viewer.md, section 5.3. The server (``server/``) keeps the job list
and the bucket's keys; a worker connects out to it over HTTPS, so it can run anywhere: on
Railway beside the server, or later on a machine with a GPU. A job's kind says what it runs
(``vo`` now), and a worker claims only the kinds it can.

**A job** (:func:`handle`):

1. download the upload, a video or a Stray Scanner zip, from the link the claim gave, capped at
   :data:`slam_it_all.eval.video.MAX_BYTES`;
2. in a child process (:func:`process`), with a memory limit (``setrlimit``, which the C++
   program it starts inherits) and a wall-clock timeout:

   - import it (:mod:`slam_it_all.eval.video`), its intrinsics chosen from the lens profiles
     (:func:`slam_it_all.eval.lenses.choose`);
   - run the C++ pipeline on it, monocular, recording features;
   - bake the bundle (:func:`slam_it_all.orchestration.bundles.bake`);

3. upload the bundle's files to the links the server gives for them;
4. report it done, or failed with a reason fit to show: an import's or the pipeline's own
   message, the worker's paths taken out; anything unexpected only as an internal error.

Each stage is reported as it starts, and again every :data:`HEARTBEAT` seconds while the child
works on it: the job's heartbeat.

**The protocol**, JSON over HTTPS, every request carrying ``Authorization: Bearer <token>``:

- ``POST /api/worker/claim`` ``{"kinds": [...], "wait": seconds}``: a job, ``{"job", "kind",
  "upload": "video" | "stray", "download"}``, or 204 when none came within the wait;
- ``POST /api/worker/jobs/<job>/progress`` ``{"stage"}``: 409 if the job is no longer the
  worker's (cancelled or expired), which ends it;
- ``POST /api/worker/jobs/<job>/files`` ``{"files": [{"path", "bytes"}, ...]}``: an upload
  link (PUT) for each of the bundle's files, ``{"urls": {path: url}}``;
- ``POST /api/worker/jobs/<job>/done`` ``{"summary"}``, or ``.../failed`` ``{"reason"}``.

Run as a program::

    SLAM_IT_ALL_WORKER_TOKEN=... python -m slam_it_all.orchestration.worker serve --server URL
        [--work DIR] [--once]
    python -m slam_it_all.orchestration.worker process UPLOAD KIND OUT  # the child's side
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import resource
import selectors
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from slam_it_all.eval import lenses, video
from slam_it_all.foundations.errors import InvalidInputError, SlamItAllError
from slam_it_all.orchestration import bundles, vo

KINDS = ("vo",)
"""The kinds of job this worker runs."""

UPLOADS = ("video", "stray")
"""The kinds of upload a ``vo`` job takes: a video, or a Stray Scanner zip."""

STAGES = ("downloading", "importing", "running", "baking", "uploading")
"""A job's stages, in order, each reported as it starts."""

MEMORY_LIMIT = 6 * 2**30
"""The address space a job's child process may use, in bytes (``RLIMIT_AS``)."""

TIMEOUT = 20 * 60.0
"""The longest a job's child process may take, in seconds."""

WAIT = 25.0
"""How long a claim waits for a job, in seconds (the server holds the request)."""

RETRY = 5.0
"""How long the worker waits after the server could not be reached, in seconds."""

HEARTBEAT = 30.0
"""How often a job's current stage is reported again while its child works, in seconds: the
server fails a job it has not heard from for a few of these."""

SEQUENCE = "your-video"
"""The folder an upload is imported into: a run names its sequence after it, and the viewer
shows that name."""

TOKEN_VARIABLE = "SLAM_IT_ALL_WORKER_TOKEN"
"""The environment variable that holds the worker's token."""

_CHUNK = 1 << 20
"""The bytes moved at a time in a download or an upload."""


class AbandonedError(Exception):
    """The server says the job is no longer this worker's: it was cancelled or expired."""


class RefusedError(Exception):
    """A job failed for a reason fit to show its owner."""


@dataclass(frozen=True)
class Job:
    """A job the server handed this worker.

    Attributes
    ----------
    id : str
        The job.
    kind : str
        What it runs (one of :data:`KINDS`).
    upload : str
        What was uploaded (one of :data:`UPLOADS`).
    download : str
        The upload's link.
    """

    id: str
    kind: str
    upload: str
    download: str


class Client:
    """Talks to the upload server, as a worker, over HTTP(S) with the standard library.

    Parameters
    ----------
    server : str
        The server's base URL.
    token : str
        The worker's token.
    """

    def __init__(self, server: str, token: str) -> None:
        """Hold the server's address and the token."""
        if not token:
            raise InvalidInputError("token: expected the worker's token")
        self.server = server.rstrip("/")
        self.token = token

    def _post(self, path: str, body: dict[str, Any], timeout: float = 60.0) -> tuple[int, Any]:
        """POST JSON; return the status and the JSON answer (None for no body)."""
        request = urllib.request.Request(
            self.server + path,
            data=json.dumps(body).encode(),
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                data = response.read()
                return response.status, json.loads(data) if data else None
        except urllib.error.HTTPError as error:
            if error.code == 409:
                raise AbandonedError(path) from None
            raise

    def claim(self, kinds: Sequence[str] = KINDS, wait: float = WAIT) -> Job | None:
        """Wait for a job of one of the kinds; None if none came within the wait."""
        status, answer = self._post(
            "/api/worker/claim", {"kinds": list(kinds), "wait": wait}, wait + 30.0
        )
        if status == 204 or answer is None:
            return None
        job = Job(
            str(answer["job"]), str(answer["kind"]), str(answer["upload"]), str(answer["download"])
        )
        if job.kind not in kinds or job.upload not in UPLOADS:
            raise InvalidInputError(
                f"claim: a job this worker cannot run ({job.kind}, {job.upload})"
            )
        return job

    def progress(self, job: Job, stage: str) -> None:
        """Report the stage a job has reached (its heartbeat); :class:`AbandonedError` on 409."""
        self._post(f"/api/worker/jobs/{job.id}/progress", {"stage": stage})

    def links(self, job: Job, files: dict[str, int]) -> dict[str, str]:
        """Ask for an upload link for each file, given by its path and size."""
        listed = [{"path": path, "bytes": size} for path, size in files.items()]
        _, answer = self._post(f"/api/worker/jobs/{job.id}/files", {"files": listed})
        urls = {str(k): str(v) for k, v in answer["urls"].items()}
        if set(urls) != set(files):
            raise InvalidInputError("files: the server's links are not for the files asked")
        return urls

    def done(self, job: Job, summary: dict[str, Any]) -> None:
        """Report a job done."""
        self._post(f"/api/worker/jobs/{job.id}/done", {"summary": summary})

    def failed(self, job: Job, reason: str) -> None:
        """Report a job failed, with a reason fit to show its owner."""
        self._post(f"/api/worker/jobs/{job.id}/failed", {"reason": reason})


def download(url: str, path: Path, cap: int) -> None:
    """Download a link to a file, refusing more than `cap` bytes."""
    read = 0
    with urllib.request.urlopen(url, timeout=60.0) as response, path.open("wb") as file:
        while chunk := response.read(_CHUNK):
            read += len(chunk)
            if read > cap:
                raise RefusedError(f"the upload is over {cap / 10**6:g} MB")
            file.write(chunk)


def upload(url: str, path: Path) -> None:
    """Upload a file to a link with PUT."""
    with path.open("rb") as file:
        request = urllib.request.Request(
            url, data=file, method="PUT", headers={"Content-Length": str(path.stat().st_size)}
        )
        with urllib.request.urlopen(request, timeout=300.0):
            pass


def _limit_memory() -> None:
    """Cap the child's address space (inherited by every process it starts)."""
    resource.setrlimit(resource.RLIMIT_AS, (MEMORY_LIMIT, MEMORY_LIMIT))


def _work(upload_path: Path, kind: str, out: Path, report: Callable[[str], None]) -> dict[str, Any]:
    """Import an upload, run it and bake its bundle into ``out/bundle``; return a summary."""
    sequence = out / SEQUENCE
    report("importing")
    if kind == "stray":
        video.import_stray(upload_path, sequence)
        source = "stray-scanner"
    else:
        folder = lenses.default_folder()
        profiles = lenses.load(folder) if folder.is_dir() else []
        chosen = lenses.choose(video.probe(upload_path), profiles)
        video.import_video(upload_path, sequence, chosen)
        source = chosen.source
    report("running")
    run = vo.run("tum", sequence, "monocular", out / "run", backend="cpp", record_features=True)
    report("baking")
    bundles.bake([out / "run"], sequence, out / "bundle")
    return {
        "frames": len(run.diagnostics),
        "tracked": len(run.timestamps),
        "keyframes": len(run.final_keyframes()[0]),
        "points": len(run.final_points()),
        "intrinsics": source,
    }


def _safe(message: str, *paths: Path) -> str:
    """Return a message without the worker's own paths in it."""
    for path in paths:
        message = message.replace(str(path) + "/", "").replace(str(path), "")
    return message


def process(upload_path: str | Path, kind: str, out: str | Path) -> int:
    """Do a job's local work (the child's side): print each stage, then the result, as JSON.

    Parameters
    ----------
    upload_path : str or pathlib.Path
        The upload.
    kind : str
        What it is (one of :data:`UPLOADS`).
    out : str or pathlib.Path
        The job's folder: the bundle goes in ``out/bundle``.

    Returns
    -------
    int
        0, having printed ``{"stage"}`` lines and a last ``{"summary"}`` line; 1, having printed
        ``{"reason"}``, if the import or the pipeline refused it.
    """
    folder = Path(out)

    def report(stage: str) -> None:
        print(json.dumps({"stage": stage}), flush=True)

    try:
        summary = _work(Path(upload_path), kind, folder, report)
    except (SlamItAllError, FileNotFoundError) as error:
        print(json.dumps({"reason": _safe(str(error), folder, Path(upload_path))}), flush=True)
        return 1
    print(json.dumps({"summary": summary}), flush=True)
    return 0


def _message(line: str) -> dict[str, Any]:
    """Return a child's line as its JSON message, or an empty one for any other output."""
    try:
        message = json.loads(line)
    except ValueError:
        return {}
    return message if isinstance(message, dict) else {}


def _run_child(
    upload_path: Path, kind: str, out: Path, report: Callable[[str], None], timeout: float
) -> dict[str, Any]:
    """Run :func:`process` in a child process within the limits; return its summary.

    The child's output is waited on with a deadline, so a child that stops talking is still
    stopped in time; lines that are not its JSON messages are ignored. Its current stage is
    reported again after every :data:`HEARTBEAT` seconds without one.
    """
    command = [sys.executable, "-m", "slam_it_all.orchestration.worker", "process"]
    child = subprocess.Popen(
        [*command, str(upload_path), kind, str(out)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        preexec_fn=_limit_memory,  # noqa: PLW1509 -- the worker runs no threads
    )
    if child.stdout is None:  # piped above
        raise RuntimeError("the job's process has no output")
    deadline = time.monotonic() + timeout
    result: dict[str, Any] = {}
    current = "downloading"  # the stage reported last, before the child's first
    with selectors.DefaultSelector() as waiting:
        waiting.register(child.stdout, selectors.EVENT_READ)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                child.kill()
                child.wait()
                raise RefusedError(f"took longer than {timeout / 60:g} minutes")
            if not waiting.select(min(remaining, HEARTBEAT)):
                report(current)  # still at it
                continue
            line = child.stdout.readline()
            if not line:
                break  # the child closed its output: it has ended
            message = _message(line)
            if "stage" in message:
                current = str(message["stage"])
                report(current)
            elif message:
                result = message
    child.wait()
    if "summary" in result:
        return dict(result["summary"])
    if "reason" in result:
        raise RefusedError(str(result["reason"]))
    raise RuntimeError(f"the job's process ended with {child.returncode} and no result")


def handle(client: Client, job: Job, work: Path, timeout: float = TIMEOUT) -> None:
    """Run one job end to end (the module docstring), and report how it ended.

    Parameters
    ----------
    client : Client
        The server.
    job : Job
        The job.
    work : pathlib.Path
        A folder for the job's files, removed when it ends.
    timeout : float, optional
        The longest its child process may take, in seconds.
    """
    folder = Path(tempfile.mkdtemp(prefix=f"job-{job.id[:8]}-", dir=work))
    try:
        client.progress(job, "downloading")
        suffix = ".zip" if job.upload == "stray" else ".video"
        upload_path = folder / f"upload{suffix}"
        download(job.download, upload_path, video.MAX_BYTES)
        summary = _run_child(
            upload_path, job.upload, folder, lambda stage: client.progress(job, stage), timeout
        )
        client.progress(job, "uploading")
        bundle = folder / "bundle"
        files = {
            p.relative_to(bundle).as_posix(): p.stat().st_size
            for p in sorted(bundle.rglob("*"))
            if p.is_file()
        }
        for path, url in client.links(job, files).items():
            upload(url, bundle / path)
        client.done(job, summary)
    except AbandonedError:
        pass  # the server has let the job go: nothing to report
    except RefusedError as refusal:
        client.failed(job, str(refusal))
    except Exception:  # a job's failure must never stop the worker
        with contextlib.suppress(Exception):
            client.failed(job, "an internal error")
    finally:
        shutil.rmtree(folder, ignore_errors=True)  # the job's own folder, made above


def serve(client: Client, work: Path, *, once: bool = False, timeout: float = TIMEOUT) -> None:
    """Claim and run jobs until stopped (or one, with `once`).

    Parameters
    ----------
    client : Client
        The server.
    work : pathlib.Path
        The folder jobs work in.
    once : bool, optional
        Stop after the first job.
    timeout : float, optional
        The longest a job's child process may take, in seconds.
    """
    work.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            job = client.claim()
        except (urllib.error.URLError, OSError, ValueError) as error:
            print(f"worker: no job ({error}); trying again in {RETRY:g} s", file=sys.stderr)
            time.sleep(RETRY)  # the server is away, or answered nonsense
            continue
        if job is None:
            continue
        handle(client, job, work, timeout)
        if once:
            return


def main(arguments: Sequence[str]) -> int:
    """Serve jobs, or do one job's local work (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, or :func:`process`'s code.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it_all.orchestration.worker")
    commands = parser.add_subparsers(dest="command", required=True)
    serving = commands.add_parser("serve", help="claim and run the server's jobs")
    serving.add_argument("--server", required=True, help="the upload server's URL")
    serving.add_argument(
        "--work", type=Path, default=Path(tempfile.gettempdir()) / "slam-it-all-worker"
    )
    serving.add_argument("--once", action="store_true", help="stop after one job")
    processing = commands.add_parser("process", help="a job's local work (the worker's child)")
    processing.add_argument("upload", type=Path)
    processing.add_argument("kind", choices=UPLOADS)
    processing.add_argument("out", type=Path)
    args = parser.parse_args(arguments)
    if args.command == "process":
        return process(args.upload, args.kind, args.out)
    with contextlib.suppress(KeyboardInterrupt):  # Ctrl-C ends the worker, quietly
        serve(Client(args.server, os.environ.get(TOKEN_VARIABLE, "")), args.work, once=args.once)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
