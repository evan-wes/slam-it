"""The worker: runs the upload server's jobs, holding nothing but its token.

docs/design/2026-10-09-run-viewer.md, section 5.3. The server (``server/``) keeps the job list
and the bucket's keys; a worker connects out to it over HTTPS, so it can run anywhere: on
Railway beside the server, or later on a machine with a GPU. A job's kind says what it runs
(``vo`` now), and a worker claims only the kinds it can.

**A job** (:func:`handle`):

1. download the upload, a video or a Stray Scanner zip, from the link the claim gave, capped at
   :data:`slam_it.eval.video.MAX_BYTES`;
2. in a child process (:func:`process`), with a memory limit (``setrlimit``, which the C++
   program it starts inherits) and a wall-clock timeout:

   - import it (:mod:`slam_it.eval.video`), its intrinsics chosen from the lens profiles
     (:func:`slam_it.eval.lenses.choose`);
   - run the C++ pipeline on it, recording features: RGB-D if the import wrote LiDAR depth,
     monocular otherwise (:data:`RUN_MODES`), with the features for its frames' size
     (:func:`slam_it.orchestration.vo.options_for`);
   - bake the bundle (:func:`slam_it.orchestration.bundles.bake`);

3. upload the bundle's files to the links the server gives for them;
4. report it done, or failed with a reason fit to show: an import's or the pipeline's own
   message, the worker's paths taken out; anything unexpected only as an internal error.

Each stage is reported as it starts, and again every :data:`HEARTBEAT` seconds while it lasts,
the downloads and uploads included: the job's heartbeat. A report that does not reach the server
is logged and the job carries on; a 409 lets it go.

**Supervision.** The child starts a session of its own, so it and every process it starts (the
C++ program among them) are one process group. Whenever a job ends other than cleanly (its
timeout, a 409, an error, the worker stopped), the whole group is killed, so a job never
outlives its turn. A SIGTERM stops the worker as a Ctrl-C does, and a second signal is ignored
while it unwinds, so the group is killed and the job's folder removed however it is stopped. The
child inherits only the environment it needs (:data:`CHILD_ENVIRONMENT`), never the worker's
token. Its stderr is the worker's, and anything unexpected is logged with the job's id, for the
operator; the owner sees only the reason fit to show.

**Trust.** The worker refuses a server or a link that is not ``http`` or ``https``, and plain
``http`` to a host that may be on the public internet (:func:`check_url`). It follows no
redirect from the server or an upload link, and a download's only to a URL it would accept.

**The protocol**, JSON over HTTPS, every request carrying ``Authorization: Bearer <token>``:

- ``POST /api/worker/claim`` ``{"kinds": [...], "wait": seconds}``: a job, ``{"job", "kind",
  "upload": "video" | "stray", "download"}``, or 204 when none came within the wait;
- ``POST /api/worker/jobs/<job>/progress`` ``{"stage"}``: 409 if the job is no longer the
  worker's (cancelled or expired), which ends it;
- ``POST /api/worker/jobs/<job>/files`` ``{"files": [{"path", "bytes"}, ...]}``: an upload
  link (PUT) for each of the bundle's files, ``{"urls": {path: url}}``;
- ``POST /api/worker/jobs/<job>/done`` ``{"summary"}`` (:func:`summarize`), or ``.../failed``
  ``{"reason"}``.

Run as a program (the lens profiles are required, :func:`slam_it.eval.lenses.database`)::

    SLAM_IT_WORKER_TOKEN=... python -m slam_it.orchestration.worker serve --server URL
        [--work DIR] [--once]
    python -m slam_it.orchestration.worker process UPLOAD KIND OUT  # the child's side
"""

from __future__ import annotations

import argparse
import contextlib
import ipaddress
import json
import logging
import os
import re
import resource
import selectors
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from http.client import HTTPMessage
from pathlib import Path
from typing import IO, Any

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import lenses, runs, tum, video
from slam_it.foundations.errors import InvalidInputError, SlamItError
from slam_it.orchestration import bundles, vo

KINDS = ("vo",)
"""The kinds of job this worker runs."""

UPLOADS = ("video", "stray")
"""The kinds of upload a ``vo`` job takes: a video, or a Stray Scanner zip."""

STAGES = ("downloading", "importing", "running", "baking", "uploading")
"""A job's stages, in order, each reported as it starts."""

MEMORY_LIMIT = 6 * 2**30
"""The address space a job's child process may use, in bytes (``RLIMIT_AS``)."""

FILE_LIMIT = 2 * 2**30
"""The largest file a job's child process may write, in bytes (``RLIMIT_FSIZE``): a frame's
PNG is at most a few MB and a run's feature records a few hundred."""

TIMEOUT = 20 * 60.0
"""The longest a job's child process may take, in seconds."""

WAIT = 25.0
"""How long a claim waits for a job, in seconds (the server holds the request)."""

RETRY = 5.0
"""How long the worker waits after the server could not be reached, in seconds."""

TOKEN_REFUSED = (401, 403)
"""The answers to a claim that say the worker's token is wrong, which no retry mends: they
end the worker (:func:`serve`)."""

HEARTBEAT = 30.0
"""How often a job's current stage is reported again while its child works, in seconds: the
server fails a job it has not heard from for a few of these."""

RUN_MODES = ("monocular", "rgbd")
"""The modes a job runs in: RGB-D when the import wrote LiDAR depth, monocular otherwise. The
status page names each (``viewer/src/jobs.js`` ``MODES``)."""

SEGMENTS = 20
"""The most segments a summary lists, the first ones: a clip that loses the track often has many,
and the summary must fit what the server keeps (``slam_it_server.app.SUMMARY_BYTES``, 4,096;
20 pairs of times take about 400 of them). ``segment_count`` gives them all."""

SEQUENCE = "your-video"
"""The folder an upload is imported into: a run names its sequence after it, and the viewer
shows that name."""

REASON_LENGTH = 500
"""The most of a failure's reason a report carries: what the server keeps
(``slam_it_server.app.REASON_LENGTH``), so that no reason, however an upload makes it, is long
enough for the server to refuse the report."""

TOKEN_VARIABLE = "SLAM_IT_WORKER_TOKEN"
"""The environment variable that holds the worker's token."""

CHILD_ENVIRONMENT = (
    "PATH", "HOME", "LANG", "LC_ALL", "TMPDIR", "PYTHONPATH", "LD_LIBRARY_PATH", "CONDA_PREFIX",
    "SLAM_IT_LENS_PROFILES", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
)  # fmt: skip
"""The environment variables a job's child inherits: what Python, the lens profiles and the C++
program need. The worker's token and anything else are left out."""

REQUEST_TIMEOUT = 60.0
"""How long a request to the server, or a download, may stall, in seconds."""

UPLOAD_TIMEOUT = 90.0
"""How long one of a bundle's uploads may stall, in seconds. A stalled upload sends no heartbeat,
so this is within the server's silence limit less a heartbeat (``SILENCE`` in
``slam_it_server.config``): the worker gives up before the server does."""

CLAIM_MARGIN = 30.0
"""How much longer than its wait a claim's request may take, in seconds."""

PRIVATE_NETWORKS = tuple(
    ipaddress.ip_network(network)
    for network in (
        "127.0.0.0/8",
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
        "::1/128",
        "fc00::/7",
    )
)
"""The addresses plain ``http`` may reach (:func:`check_url`): loopback, and the private ranges
(RFC 1918, IPv6's unique-local). Not link-local (a cloud's metadata service), 6to4 or NAT64."""

_LABEL = re.compile(r"[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?")
"""One DNS label, as a host name's lowercase part between dots."""

_CHUNK = 1 << 20
"""The bytes moved at a time in a download or an upload."""

_LOG = logging.getLogger(__name__)
"""The worker's log, for its operator (the module docstring)."""


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


def check_url(url: str, what: str) -> str:
    """Return a URL the worker may use, refusing one that is not ``http(s)`` to a safe host.

    ``https`` goes anywhere; plain ``http`` only to a host that cannot be on the public
    internet: an address in :data:`PRIVATE_NETWORKS`, ``localhost``, a one-label name (a
    container network's), or Railway's private network (``*.railway.internal``). The host is
    judged as urllib connects to it, its escapes undone.

    Parameters
    ----------
    url : str
        The URL.
    what : str
        What it is, for the error.

    Returns
    -------
    str
        The URL.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not such a URL.
    """
    expected = f"{what}: expected an http or https URL with a host, got {url!r}"
    try:
        parts = urllib.parse.urlsplit(url)
        host = urllib.parse.unquote(parts.hostname or "")  # urllib unescapes it to connect
    except ValueError:  # brackets that hold no address
        raise InvalidInputError(expected) from None
    if parts.scheme not in ("http", "https") or not host:
        raise InvalidInputError(expected)
    if parts.scheme == "http" and not _private_host(host):
        raise InvalidInputError(f"{what}: plain http only to a private host, got {host!r}")
    return url


def _private_host(host: str) -> bool:
    """Whether a host, as urllib connects to it, cannot be on the public internet.

    :func:`check_url`'s rule. A host the C library reads as an address, in any of its forms
    ("127.0.0.1", "2130706433", "0x7f000001"), must be in :data:`PRIVATE_NETWORKS`; one with a
    scope id is judged without it, so "fe80::1%eth0" is link-local and refused. A name must be
    DNS labels, which no escape or scope id is: ``localhost``, one label (a container's name),
    or under ``railway.internal``.
    """
    try:
        found = socket.getaddrinfo(host, None, flags=socket.AI_NUMERICHOST)
    except (socket.gaierror, UnicodeError, ValueError):  # a name, not an address: none looked up
        labels = host.split(".")
        if not all(_LABEL.fullmatch(label) for label in labels):
            return False
        return host == "localhost" or len(labels) == 1 or host.endswith(".railway.internal")
    address = ipaddress.ip_address(found[0][4][0])
    return any(address in network for network in PRIVATE_NETWORKS)


class _Redirects(urllib.request.HTTPRedirectHandler):
    """Follows a redirect only where one may lead.

    Nowhere from the server's API or an upload link, which never redirect, and whose redirect
    would carry the request's headers, the worker's token among them; from a download link, only
    to a URL :func:`check_url` allows.
    """

    def __init__(self, *, follow: bool) -> None:
        """Hold whether a redirect is followed at all."""
        super().__init__()
        self.follow = follow

    def redirect_request(  # noqa: PLR0913, PLR0917 -- urllib's signature
        self,
        req: urllib.request.Request,
        fp: IO[bytes],
        code: int,
        msg: str,
        headers: HTTPMessage,
        newurl: str,
    ) -> urllib.request.Request | None:
        """Follow none (the 3xx is raised as an HTTPError), or one to a URL check_url allows."""
        if not self.follow:
            return None
        check_url(newurl, "download's redirect")
        return super().redirect_request(req, fp, code, msg, headers, newurl)


_API = urllib.request.build_opener(_Redirects(follow=False))
"""Opens the server's API and the upload links, following no redirect."""

_DOWNLOADS = urllib.request.build_opener(_Redirects(follow=True))
"""Opens a download link, following a redirect only to a URL :func:`check_url` allows."""


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
        self.server = check_url(server, "server").rstrip("/")
        self.token = token

    def _post(
        self, path: str, body: dict[str, Any], timeout: float = REQUEST_TIMEOUT
    ) -> tuple[int, Any]:
        """POST JSON; return the status and the JSON answer (None for no body)."""
        request = urllib.request.Request(
            self.server + path,
            data=json.dumps(body).encode(),
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
            method="POST",
        )
        try:
            with _API.open(request, timeout=timeout) as response:
                data = response.read()
                return response.status, json.loads(data) if data else None
        except urllib.error.HTTPError as error:
            if error.code == 409:
                raise AbandonedError(path) from None
            raise

    def claim(self, kinds: Sequence[str] = KINDS, wait: float = WAIT) -> Job | None:
        """Wait for a job of one of the kinds; None if none came within the wait."""
        status, answer = self._post(
            "/api/worker/claim", {"kinds": list(kinds), "wait": wait}, wait + CLAIM_MARGIN
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
        check_url(job.download, "claim's download link")
        return job

    def progress(self, job: Job, stage: str) -> None:
        """Report the stage a job has reached (its heartbeat); :class:`AbandonedError` on 409."""
        self._post(f"/api/worker/jobs/{job.id}/progress", {"stage": stage})

    def links(self, job: Job, files: dict[str, int]) -> dict[str, str]:
        """Ask for an upload link for each file, given by its path and size."""
        listed = [{"path": path, "bytes": size} for path, size in files.items()]
        _, answer = self._post(f"/api/worker/jobs/{job.id}/files", {"files": listed})
        urls = {str(k): check_url(str(v), "upload link") for k, v in answer["urls"].items()}
        if set(urls) != set(files):
            raise InvalidInputError("files: the server's links are not for the files asked")
        return urls

    def done(self, job: Job, summary: dict[str, Any]) -> None:
        """Report a job done."""
        self._post(f"/api/worker/jobs/{job.id}/done", {"summary": summary})

    def failed(self, job: Job, reason: str) -> None:
        """Report a job failed, its reason fit to show its owner, cut to :data:`REASON_LENGTH`."""
        self._post(f"/api/worker/jobs/{job.id}/failed", {"reason": reason[:REASON_LENGTH]})


class _Beat:
    """Calls a report every :data:`HEARTBEAT` seconds, when asked whether it is due."""

    def __init__(self, report: Callable[[], None]) -> None:
        """Start the clock."""
        self.report, self.last = report, time.monotonic()

    def __call__(self) -> None:
        """Report if a heartbeat has passed since the last."""
        if time.monotonic() - self.last >= HEARTBEAT:
            self.report()
            self.last = time.monotonic()


class _Reading:
    """A file read for an upload, calling `beat` per block (http.client reads it in blocks)."""

    def __init__(self, file: IO[bytes], beat: Callable[[], None]) -> None:
        """Hold the file and the beat."""
        self.file, self.beat = file, beat

    def read(self, size: int = -1) -> bytes:
        """Read a block, and beat if one is due."""
        self.beat()
        return self.file.read(size)


def download(url: str, path: Path, cap: int, beat: Callable[[], None] = lambda: None) -> None:
    """Download a link to a file, refusing more than `cap` bytes; `beat` is called per chunk."""
    read = 0
    with (
        _DOWNLOADS.open(url, timeout=REQUEST_TIMEOUT) as response,
        path.open("wb") as file,
    ):
        while chunk := response.read(_CHUNK):
            beat()
            read += len(chunk)
            if read > cap:
                raise RefusedError(f"the upload is over {cap / 10**6:g} MB")
            file.write(chunk)


def upload(url: str, path: Path, beat: Callable[[], None] = lambda: None) -> None:
    """Upload a file to a link with PUT; `beat` is called per block read."""
    with path.open("rb") as file:
        request = urllib.request.Request(
            url,
            data=_Reading(file, beat),
            method="PUT",
            headers={"Content-Length": str(path.stat().st_size)},
        )
        with _API.open(request, timeout=UPLOAD_TIMEOUT):
            pass


def _limit_resources() -> None:
    """Cap the child's address space and file sizes (inherited by every process it starts)."""
    resource.setrlimit(resource.RLIMIT_AS, (MEMORY_LIMIT, MEMORY_LIMIT))
    resource.setrlimit(resource.RLIMIT_FSIZE, (FILE_LIMIT, FILE_LIMIT))


def child_environment() -> dict[str, str]:
    """Return the environment a job's child inherits: :data:`CHILD_ENVIRONMENT`, as set."""
    return {name: os.environ[name] for name in CHILD_ENVIRONMENT if name in os.environ}


def _work(upload_path: Path, kind: str, out: Path, report: Callable[[str], None]) -> dict[str, Any]:
    """Import an upload, run it and bake its bundle into ``out/bundle``; return a summary."""
    sequence = out / SEQUENCE
    report("importing")
    if kind == "stray":
        video.import_stray(upload_path, sequence)
        source = "stray-scanner"
    else:
        chosen = lenses.choose(video.probe(upload_path), lenses.database())
        video.import_video(upload_path, sequence, chosen)
        source = chosen.source
    report("running")
    mode = "rgbd" if (sequence / "depth.txt").is_file() else "monocular"  # LiDAR depth imported
    camera = tum.read_camera(sequence)
    options = vo.options_for(camera.width, camera.height)
    run = vo.run(
        "tum", sequence, mode, out / "run", backend="cpp", options=options, record_features=True
    )
    report("baking")
    bundles.bake([out / "run"], sequence, out / "bundle")
    times = vo.open_source("tum", sequence, mode).times
    return summarize(run, times, source, (camera.width, camera.height))


def summarize(
    run: runs.Run, times: NDArray[np.float64], intrinsics: str, size: tuple[int, int]
) -> dict[str, Any]:
    """Return a job's summary: what its run made, and what of the clip it ran and tracked.

    What of the clip it ran and tracked is :func:`slam_it.eval.runs.tracking`'s: a segment per
    map, a lost track starting the next. The status page says why the tracked parts can be fewer
    than the clip's (``viewer/src/jobs.js``), so that a short result is not taken for a cropped
    video, and warns when the frames arrived smaller than the import's short side.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The job's run.
    times : ndarray of float64, shape (F,)
        The imported clip's frames' times, in seconds
        (:attr:`slam_it.orchestration.vo.Source.times`).
    intrinsics : str
        Where its intrinsics came from: ``"stray-scanner"``, or a
        :attr:`slam_it.eval.lenses.Chosen.source`.
    size : tuple of int
        The imported frames' width and height, upright.

    Returns
    -------
    dict
        ``frames`` run, ``tracked``, the final maps' ``keyframes`` and ``points``, the run's
        ``mode`` (``"rgbd"`` for a recording with LiDAR depth), ``intrinsics``, ``size``;
        ``clip_seconds``, the clip's last frame's time; ``tracked_from`` and ``tracked_to``, the
        first and last tracked frames' times (null if none was tracked); ``segments``, each
        map's first and last tracked frames' times, the first :data:`SEGMENTS` of them, and
        ``segment_count``, all of them; ``ran_to``, the last frame run's time; every time since
        the clip's first frame, to the millisecond; ``losses``, the frames the track was lost
        on; and ``resets``, the maps lost too soon to keep.
    """
    span = runs.tracking(run, times)
    segments = [[s.tracked_from, s.tracked_to] for s in span.segments[:SEGMENTS]]
    return {
        "frames": span.frames,
        "tracked": span.tracked,
        "keyframes": len(run.final_keyframes()[0]),
        "points": len(run.final_points()),
        "mode": run.mode,
        "intrinsics": intrinsics,
        "size": [int(size[0]), int(size[1])],
        "clip_seconds": span.seconds,
        "tracked_from": span.tracked_from,
        "tracked_to": span.tracked_to,
        "segments": segments,
        "segment_count": len(span.segments),
        "ran_to": span.ran_to,
        "losses": span.losses,
        "resets": span.resets,
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
    folder, upload = Path(out), Path(upload_path)

    def report(stage: str) -> None:
        print(json.dumps({"stage": stage}), flush=True)

    try:
        if not upload.is_file():
            raise InvalidInputError("the upload is missing")
        summary = _work(upload, kind, folder, report)
    except SlamItError as error:  # anything else is the worker's own: no reason, logged
        print(json.dumps({"reason": _safe(str(error), folder, upload)}), flush=True)
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
    argv: Sequence[str], report: Callable[[str], None], timeout: float
) -> dict[str, Any]:
    """Run a job's child (:func:`process`) within the limits; return its summary.

    The child starts a session of its own, so it and every process it starts are one group,
    killed whole when it ends, however it ends: past its deadline, when a report raises (a
    409), or with an orphan left. Its output is waited on with a deadline, so a child that
    stops talking is still stopped in time; lines that are not its JSON messages are ignored.
    Its current stage is reported again after every :data:`HEARTBEAT` seconds without one.

    Parameters
    ----------
    argv : sequence of str
        The child's command line.
    report : callable
        Reports a stage; :class:`AbandonedError` ends the job.
    timeout : float
        The longest the child may take, in seconds.

    Returns
    -------
    dict
        The child's summary.

    Raises
    ------
    RefusedError
        If the child took too long, or refused the upload with a reason.
    AbandonedError
        If a report found the job let go.
    RuntimeError
        If the child ended without a result (out of memory, a crash).
    """
    with subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        text=True,
        env=child_environment(),
        preexec_fn=_limit_resources,  # noqa: PLW1509 -- the worker runs no threads
        start_new_session=True,
    ) as child:
        try:
            result = _supervise(child, report, timeout)
        finally:
            with contextlib.suppress(ProcessLookupError):  # its group, whatever is left of it
                os.killpg(child.pid, signal.SIGKILL)
    if "summary" in result:
        return dict(result["summary"])
    if "reason" in result:
        raise RefusedError(str(result["reason"]))
    raise RuntimeError(f"the job's process ended with {child.returncode} and no result")


def _too_long(timeout: float) -> RefusedError:
    """Return the refusal of a job past its deadline of `timeout` seconds."""
    return RefusedError(f"took longer than {timeout / 60:g} minutes")


def _supervise(
    child: subprocess.Popen[str], report: Callable[[str], None], timeout: float
) -> dict[str, Any]:
    """Follow a child's output until its result and its end, or its deadline; return the result.

    The result is its ``summary`` or ``reason`` line; ending without one gives an empty dict.
    """
    if child.stdout is None:  # piped by the caller
        raise RuntimeError("the job's process has no output")
    deadline = time.monotonic() + timeout
    result: dict[str, Any] = {}
    current = "downloading"  # the stage reported last, before the child's first
    with selectors.DefaultSelector() as waiting:
        waiting.register(child.stdout, selectors.EVENT_READ)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise _too_long(timeout)
            if not waiting.select(min(remaining, HEARTBEAT)):
                report(current)  # still at it
                continue
            line = child.stdout.readline()
            if not line:
                child.wait()
                return result  # the child closed its output: it has ended
            message = _message(line)
            if "stage" in message:
                current = str(message["stage"])
                report(current)
            elif "summary" in message or "reason" in message:
                # Its last word: wait for it to end, not for its output to close, which a
                # process it left behind may hold open.
                try:
                    child.wait(timeout=max(deadline - time.monotonic(), 0.0))
                except subprocess.TimeoutExpired:
                    raise _too_long(timeout) from None
                return message


def _report_failure(client: Client, job: Job, reason: str) -> None:
    """Report a job failed; a report that cannot be made is logged, never raised."""
    try:
        client.failed(job, reason)
    except Exception as error:  # the job is over either way: the worker carries on
        _LOG.warning("job %s: the failure was not reported (%s)", job.id, error)


def handle(client: Client, job: Job, work: Path, timeout: float = TIMEOUT) -> None:
    """Run one job end to end (the module docstring), and report how it ended.

    It never raises: a job's failure, however it comes, its folder's making included, never
    stops the worker.

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
    folder: Path | None = None

    def report(stage: str) -> None:
        """Report a stage; one that does not reach the server is logged, and the job goes on."""
        try:
            client.progress(job, stage)
        except OSError as error:  # a URLError among them; a 409 is AbandonedError, raised
            _LOG.warning(
                "job %s: the %s report did not reach the server (%s)", job.id, stage, error
            )

    try:
        folder = Path(tempfile.mkdtemp(prefix=f"job-{job.id[:8]}-", dir=work))
        report("downloading")
        suffix = ".zip" if job.upload == "stray" else ".video"
        upload_path = folder / f"upload{suffix}"
        download(job.download, upload_path, video.MAX_BYTES, _Beat(lambda: report("downloading")))
        argv = [sys.executable, "-m", "slam_it.orchestration.worker", "process"]
        summary = _run_child([*argv, str(upload_path), job.upload, str(folder)], report, timeout)
        report("uploading")
        bundle = folder / "bundle"
        files = {
            p.relative_to(bundle).as_posix(): p.stat().st_size
            for p in sorted(bundle.rglob("*"))
            if p.is_file()
        }
        beat = _Beat(lambda: report("uploading"))
        for path, url in client.links(job, files).items():
            upload(url, bundle / path, beat)
        client.done(job, summary)
    except AbandonedError:
        _LOG.info("job %s: let go by the server", job.id)
    except RefusedError as refusal:
        _report_failure(client, job, str(refusal))
    except Exception:  # a job's failure must never stop the worker
        _LOG.exception("job %s: an internal error", job.id)
        _report_failure(client, job, "an internal error")
    finally:
        if folder is not None:
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

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the lens profiles are not there (:func:`slam_it.eval.lenses.database`), or the
        server refuses the worker's token (:data:`TOKEN_REFUSED`).
    """
    lenses.database()  # required, so that no video silently falls to a fallback
    work.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            job = client.claim()
        except (OSError, ValueError) as error:  # a URLError is an OSError
            if isinstance(error, urllib.error.HTTPError) and error.code in TOKEN_REFUSED:
                raise InvalidInputError(
                    f"token: the server refused it (HTTP {error.code})"
                ) from None
            _LOG.warning("no job (%s); trying again in %g s", error, RETRY)
            time.sleep(RETRY)  # the server is away, or answered nonsense
            continue
        if job is None:
            continue
        handle(client, job, work, timeout)
        if once:
            return


def _stop(signum: int, frame: object) -> None:
    """Stop the worker, once: further SIGINTs and SIGTERMs are ignored while it unwinds.

    It raises :class:`KeyboardInterrupt`, so that a job's processes are killed and its folder
    removed on the way out (:func:`_run_child`, :func:`handle`), and no second signal (the
    demo's ``terminate()`` after a Ctrl-C that reached both) can cut that short.
    """
    for each in (signal.SIGINT, signal.SIGTERM):
        signal.signal(each, signal.SIG_IGN)
    raise KeyboardInterrupt


@contextlib.contextmanager
def _stopping_on_signals() -> Iterator[None]:
    """Make a SIGINT or a SIGTERM stop the worker (:func:`_stop`) within the block alone.

    The handlers it found are put back when the block ends, so a caller of :func:`main` keeps
    its own.
    """
    previous = {each: signal.signal(each, _stop) for each in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for each, handler in previous.items():
            signal.signal(each, handler)


def main(arguments: Sequence[str]) -> int:
    """Serve jobs, or do one job's local work (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0; 2 if the server's address, the token or the lens profiles are wrong; or
        :func:`process`'s code.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.orchestration.worker")
    commands = parser.add_subparsers(dest="command", required=True)
    serving = commands.add_parser("serve", help="claim and run the server's jobs")
    serving.add_argument("--server", required=True, help="the upload server's URL")
    serving.add_argument(
        "--work", type=Path, default=Path(tempfile.gettempdir()) / "slam-it-worker"
    )
    serving.add_argument("--once", action="store_true", help="stop after one job")
    processing = commands.add_parser("process", help="a job's local work (the worker's child)")
    processing.add_argument("upload", type=Path)
    processing.add_argument("kind", choices=UPLOADS)
    processing.add_argument("out", type=Path)
    args = parser.parse_args(arguments)
    if args.command == "process":
        return process(args.upload, args.kind, args.out)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s worker %(levelname)s %(message)s", stream=sys.stderr
    )
    try:
        client = Client(args.server, os.environ.get(TOKEN_VARIABLE, ""))
        # Ctrl-C or a SIGTERM ends it, quietly, once the job's clean-up has run
        with contextlib.suppress(KeyboardInterrupt), _stopping_on_signals():
            serve(client, args.work, once=args.once)
    except InvalidInputError as error:
        print(f"worker: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
