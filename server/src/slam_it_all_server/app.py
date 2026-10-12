"""The server's routes: the pages, the owner's API, the worker's API, and folder storage's links.

docs/design/2026-10-09-run-viewer.md, sections 5.3 and 5.4.

**The pages** come from the viewer's folder (only the files :data:`PAGES` matches): the upload
page at ``/``, a job's status at ``/r/<token>``, and the run viewer (``/index.html``), which
reads a done job's bundle from ``/r/<token>/bundle/``. A bundle's files are redirected to
storage's own links, never proxied.

**The owner's API:**

- ``GET /api/kept``: ``{"uploads": bool}``, whether uploads are kept with their results (the
  demo's :attr:`~slam_it_all_server.config.Settings.keep_uploads`), which the upload page says;
- ``POST /api/jobs`` ``{"code", "upload": "video" | "stray", "bytes"}``: a job, and the link to
  upload to, signed for exactly that size;
- ``POST /api/jobs/<token>/uploaded``: queues the job, once storage holds the whole upload;
- ``GET /api/jobs/<token>``: how the job is going.

**The worker's API** is the one :mod:`slam_it_all.orchestration.worker` describes, the worker's
token checked on every request.

**The guards:**

- an invite code to make a job; on staging, for every page too (HTTP Basic auth, its password a
  code);
- the size cap, checked against the declared size and signed into the upload link;
- one job worked on at a time, and :data:`~slam_it_all_server.config.QUEUE` open at most;
- an upload moved, once confirmed, where its owner's link cannot reach it, and deleted once
  its import ends (where the demo keeps uploads, once the job is deleted); whatever a job's
  links may write later is deleted when it ends, and again
  once every link it signed has expired and an upload begun in time could have arrived
  (:data:`~slam_it_all_server.config.UPLOAD_GRACE`);
- every job deleted :data:`~slam_it_all_server.config.RETENTION` after it ends, its files
  first, all in its own folder of storage (:func:`job_prefix`), so that a deletion that fails
  is tried again (:func:`sweep`);
- at most :data:`slam_it_all_server.jobs.CODE_JOBS` jobs open per invite code;
- a result's files only of the names and kinds a bundle has (:data:`RESULT_PATH`);
- results reachable only through their token, pages marked ``noindex``, and no ``Referer``
  sent, so a token never leaves in one; the log names jobs by their id alone;
- a Content Security Policy (:func:`content_policy`): scripts from this site, the import map
  by its hash and three.js's CDN alone, connections to this site and storage alone;
- request bodies capped at :data:`BODY_BYTES`, except folder storage's signed uploads.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import contextlib
import hashlib
import hmac
import json
import logging
import re
import uuid
from collections.abc import AsyncIterator, Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any, Literal

import boto3
from botocore.config import Config
from fastapi import Depends, FastAPI, Header, HTTPException, Request, Response
from fastapi.responses import FileResponse, RedirectResponse
from pydantic import BaseModel, Field, StrictInt

from slam_it_all_server import config, jobs
from slam_it_all_server.config import Settings
from slam_it_all_server.jobs import Job, Jobs
from slam_it_all_server.storage import FolderStorage, S3Storage, Storage, StorageError

PAGES = re.compile(r"^(index\.html|upload\.html|status\.html|app\.css|src/[a-z_]+\.js)$")
"""The viewer folder's files that are served: its pages, their style and their modules."""

RESULT_FILES = 1000
"""The most files a result may have."""

RESULT_BYTES = 10**9
"""The most bytes a result may hold: a bound on a worker gone wrong, well above a bundle's size."""

RESULT_PATH = re.compile(r"([a-z]+/)?[a-z0-9_]+\.(json|npy|bin)")
"""A result's file, matched whole: a bundle's files are JSON, numpy arrays and image chunks, at
most one folder deep (``slam_it_all.orchestration.bundles``)."""

SUMMARY_BYTES = 4096
"""The largest summary of a job done, as JSON."""

REASON_LENGTH = 500
"""The longest reason for a failure that is kept, in characters."""

BODY_BYTES = 64 * 2**10
"""The largest request body outside folder storage's uploads, in bytes."""

SWEEP = 15.0
"""How often jobs whose time is up are swept, in seconds."""

CLAIM_POLL = 1.0
"""How often a worker's waiting claim looks for a job, in seconds."""

CLAIM_WAIT = 60.0
"""The longest a worker's claim is held, in seconds."""

CDN = "https://cdn.jsdelivr.net/npm/three@0.186.0/"
"""Where the run viewer loads three.js from (``viewer/index.html``'s import map): the content
policy allows scripts from this path, not the rest of the CDN."""

_HEADERS = {
    "X-Robots-Tag": "noindex, nofollow",
    "Referrer-Policy": "no-referrer",
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
}
"""Headers on every response."""

_OPEN = ("/api/worker/", "/storage/", "/healthz")
"""Paths staging's Basic auth leaves open: they carry their own token or signature."""

_LOG = logging.getLogger("slam_it_all_server")
"""The server's log: jobs by their id, never by their token, nor a link."""


def job_prefix(job_id: str) -> str:
    """Return the storage prefix of a job's files: its upload, the upload's copy and its result.

    One folder a job, so that deleting it deletes them all.
    """
    return f"jobs/{job_id}"


def upload_key(job_id: str) -> str:
    """Return the storage key of a job's upload, which its owner's upload link writes."""
    return f"{job_prefix(job_id)}/upload"


def import_key(job_id: str) -> str:
    """Return the storage key a confirmed upload is moved to, out of its owner's link's reach."""
    return f"{job_prefix(job_id)}/import"


def result_prefix(job_id: str) -> str:
    """Return the storage prefix of a job's result."""
    return f"{job_prefix(job_id)}/result"


def make_storage(settings: Settings) -> Storage:
    """Return the storage the settings name."""
    if settings.storage == "s3":
        client = boto3.client(
            "s3",
            endpoint_url=settings.s3["endpoint"],
            aws_access_key_id=settings.s3["key_id"],
            aws_secret_access_key=settings.s3["secret"],
            region_name=settings.s3["region"],
            config=Config(signature_version="s3v4"),
        )
        return S3Storage(client, settings.s3["bucket"])
    root = Path(settings.storage.removeprefix("folder:"))
    return FolderStorage(root, settings.public_url, settings.signing_key)


def sweep(job_list: Jobs, store: Storage, *, keep_uploads: bool = False) -> int:
    """Sweep the job list: end the jobs whose time is up, then clear and delete their files.

    A job's files are deleted before it is (:mod:`slam_it_all_server.jobs`): a deletion that
    fails is logged, and the job stays listed for the next sweep to try again.

    Parameters
    ----------
    job_list : Jobs
        The job list.
    store : Storage
        The storage.
    keep_uploads : bool, optional
        Whether an ended job's upload copy stays until the job is deleted (the demo's,
        :attr:`~slam_it_all_server.config.Settings.keep_uploads`).

    Returns
    -------
    int
        How many deletions failed, each logged.
    """
    for job in job_list.end_overdue():
        _LOG.info("job %s: %s (%s)", job.id, job.state, job.reason)
    steps: list[tuple[list[Job], Callable[[Job], None]]] = [
        (job_list.links_to_close(), lambda job: _close_link(job_list, store, job)),
        (job_list.files_to_clear(), lambda job: _clear_files(job_list, store, job, keep_uploads)),
        (job_list.to_delete(), lambda job: _delete(job_list, store, job)),
    ]
    failed = 0
    for found, step in steps:
        for job in found:
            try:
                step(job)
            except Exception:  # a storage or database failure: the next sweep tries again
                failed += 1
                _LOG.exception("job %s: a sweep's deletion failed", job.id)
    return failed


def _close_link(job_list: Jobs, store: Storage, job: Job) -> None:
    """Delete whatever an ended job's links wrote, their grace over, then record it."""
    store.delete(upload_key(job.id))
    if job.state != "done":  # a result link of a job that failed may have written since
        store.delete_prefix(result_prefix(job.id))
    job_list.close_link(job.id)


def _clear_files(job_list: Jobs, store: Storage, job: Job, keep_uploads: bool) -> None:
    """Delete an ended job's upload, its copy unless kept, its result unless done; record it."""
    store.delete(upload_key(job.id))
    if not keep_uploads:
        store.delete(import_key(job.id))
    if job.state != "done":
        store.delete_prefix(result_prefix(job.id))
    job_list.clear_files(job.id)


def _delete(job_list: Jobs, store: Storage, job: Job) -> None:
    """Delete every file of a job kept its time, its folder whole, then the job."""
    store.delete_prefix(job_prefix(job.id))
    job_list.delete(job.id)
    _LOG.info("job %s: deleted", job.id)


def _best_effort(what: str, job: Job, call: Callable[[], None]) -> None:
    """Delete something now, leaving it to the sweep if it fails (logged)."""
    try:
        call()
    except Exception:  # the sweep deletes it later
        _LOG.exception("job %s: %s failed; the sweep will try again", job.id, what)


class NewJob(BaseModel):
    """An owner's request for a job."""

    code: str = Field(max_length=200)
    upload: Literal["video", "stray"]
    size: StrictInt = Field(gt=0, alias="bytes")


class Claim(BaseModel):
    """A worker's claim."""

    kinds: list[str] = Field(max_length=10)
    wait: float = Field(ge=0, le=CLAIM_WAIT)


class Progress(BaseModel):
    """A worker's report of a stage."""

    stage: str = Field(max_length=40)


class ResultFile(BaseModel):
    """One of a result's files."""

    path: str = Field(max_length=200)
    size: StrictInt = Field(ge=0, alias="bytes")


class Files(BaseModel):
    """A result's files, as a worker is about to upload them."""

    files: list[ResultFile] = Field(max_length=RESULT_FILES)


class Done(BaseModel):
    """A worker's report of a job done."""

    summary: dict[str, Any]


class Failed(BaseModel):
    """A worker's report of a job failed."""

    reason: str = Field(max_length=10_000)


def _matches(given: str, secrets: tuple[str, ...]) -> bool:
    """Whether a value is one of the secrets, each compared in constant time."""
    found = False
    for secret in secrets:
        found |= hmac.compare_digest(given.encode(), secret.encode())
    return found


def _staging_allows(request: Request, settings: Settings) -> bool:
    """Whether a request may pass staging's Basic auth: an open path, or a code as its password."""
    if request.url.path.startswith(_OPEN):
        return True
    scheme, _, encoded = request.headers.get("authorization", "").partition(" ")
    if scheme.lower() != "basic":
        return False
    try:
        credentials = base64.b64decode(encoded, validate=True).decode()
    except (binascii.Error, UnicodeDecodeError):
        return False
    return _matches(credentials.partition(":")[2], settings.invite_codes)


def content_policy(viewer: Path, store: Storage) -> str:
    """Return the pages' Content Security Policy.

    Scripts come from this site, three.js's path on its CDN (:data:`CDN`), and the inline import
    maps of the viewer's ``index.html``, each allowed by its hash; styles from this site and
    Google Fonts; media from this site and the visitor's own chosen file (``blob:``);
    connections go to this site and to storage's own origin (the bucket, for the uploads and a
    bundle's redirected files); nothing may frame the pages.

    Parameters
    ----------
    viewer : pathlib.Path
        The viewer's folder.
    store : Storage
        The storage, whose links' origin connections may reach.

    Returns
    -------
    str
        The policy.

    Raises
    ------
    slam_it_all_server.config.SettingsError
        If the viewer has no ``index.html``, or an import map the policy cannot read
        (:func:`import_maps`).
    """
    return _policy(import_maps(viewer), store)


def import_maps(viewer: Path) -> list[str]:
    """Return the CSP sources of the viewer page's inline import maps, by their hashes.

    Parameters
    ----------
    viewer : pathlib.Path
        The viewer's folder (``SLAM_IT_ALL_VIEWER``).

    Returns
    -------
    list of str
        Each map's ``'sha256-...'`` source.

    Raises
    ------
    slam_it_all_server.config.SettingsError
        If the folder has no ``index.html``, or an import map not written as
        ``<script type="importmap">``: the policy would allow none, and the viewer would break
        in the browser alone.
    """
    index = viewer / "index.html"
    if not index.is_file():
        raise config.SettingsError(
            f"SLAM_IT_ALL_VIEWER: {index} is not there, the run viewer's page"
        )
    text = index.read_text()
    maps = re.findall(r'<script type="importmap">(.*?)</script>', text, re.DOTALL)
    if "importmap" in text and not maps:
        written = '<script type="importmap">'
        raise config.SettingsError(
            f"SLAM_IT_ALL_VIEWER: {index} has an import map not written as {written}"
        )
    return [
        "'sha256-" + base64.b64encode(hashlib.sha256(m.encode()).digest()).decode() + "'"
        for m in maps
    ]


def _policy(maps: list[str], store: Storage) -> str:
    """Return the Content Security Policy for the import maps' sources and a store."""
    return "; ".join(
        [
            "default-src 'self'",
            " ".join(["script-src 'self'", CDN, *maps]),
            "style-src 'self' https://fonts.googleapis.com",
            "font-src https://fonts.gstatic.com",
            "img-src 'self' blob: data:",
            "media-src 'self' blob:",  # the upload page reads a video's length from the file
            f"connect-src 'self' {store.origin}",
            "object-src 'none'",
            "base-uri 'self'",
            "form-action 'self'",
            "frame-ancestors 'none'",
        ]
    )


def _body_refused(request: Request) -> Response | None:
    """Refuse a body over :data:`BODY_BYTES`, or one sent without a length, outside storage."""
    if request.url.path.startswith("/storage/"):
        return None
    if "transfer-encoding" in request.headers:
        return Response("a request body must give its length", 411)
    try:
        length = int(request.headers.get("content-length", "0"))
    except ValueError:
        return Response("the request's length is not a number", 400)
    return Response("the request is too large", 413) if length > BODY_BYTES else None


@dataclass(frozen=True)
class _Server:
    """What the routes share: the settings, the storage and the job list."""

    settings: Settings
    store: Storage
    jobs: Jobs

    def owned(self, token: str) -> Job:
        """Return the job a link names, or answer 404."""
        job = self.jobs.by_token(token)
        if job is None:
            raise HTTPException(404, "no such job: its link is wrong, or it has been deleted")
        return job

    def page(self, name: str) -> FileResponse:
        """Return one of the viewer folder's files."""
        return FileResponse(self.settings.viewer / name, headers={"Cache-Control": "no-cache"})


def _working(call: Callable[[], Job]) -> Job:
    """Change a worker's job, answering 409 if it is no longer the worker's."""
    try:
        return call()
    except jobs.StateError:
        raise HTTPException(409, "the job is no longer yours") from None


def make_app(
    settings: Settings, store: Storage | None = None, job_list: Jobs | None = None
) -> FastAPI:
    """Return the server's app.

    Parameters
    ----------
    settings : Settings
        Its settings.
    store : Storage, optional
        Its storage (default: the one the settings name).
    job_list : Jobs, optional
        Its job list (default: the settings' database).

    Returns
    -------
    fastapi.FastAPI
        The app.

    Raises
    ------
    slam_it_all_server.config.SettingsError
        If the viewer cannot be served (:func:`import_maps`), before anything is made.
    slam_it_all_server.jobs.SchemaError
        If the job list is of another format.
    """
    maps = import_maps(settings.viewer)  # a viewer without its page is refused first
    server = _Server(
        settings,
        make_storage(settings) if store is None else store,
        Jobs(settings.database) if job_list is None else job_list,
    )

    @contextlib.asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        """Sweep the job list while the server runs."""

        async def sweeping() -> None:
            while True:
                try:
                    await asyncio.to_thread(
                        sweep, server.jobs, server.store, keep_uploads=server.settings.keep_uploads
                    )
                except Exception:  # the list itself failed: logged, and tried again
                    _LOG.exception("a sweep failed")
                await asyncio.sleep(SWEEP)

        task = asyncio.create_task(sweeping())
        yield
        task.cancel()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    headers = {**_HEADERS, "Content-Security-Policy": _policy(maps, server.store)}

    @app.middleware("http")
    async def guard(
        request: Request, call_next: Callable[[Request], Awaitable[Response]]
    ) -> Response:
        """Apply staging's Basic auth and the body cap, and add the headers every answer has."""
        if settings.staging and not _staging_allows(request, settings):
            answer = Response(
                "an invite code is needed", 401, {"WWW-Authenticate": 'Basic realm="SLAM It All!"'}
            )
        else:
            answer = _body_refused(request) or await call_next(request)
        answer.headers.update(headers)
        if request.url.path.startswith("/api/"):
            answer.headers["Cache-Control"] = "no-store"
        return answer

    _owner_api(app, server)
    _worker_api(app, server)
    if isinstance(server.store, FolderStorage):
        _folder_storage(app, server.store)
    _pages(app, server)  # last: its catch-all would shadow the routes after it
    return app


def _owner_api(app: FastAPI, server: _Server) -> None:
    """Add the owner's API: make a job, confirm its upload, ask how it is going."""

    @app.get("/api/kept")
    def kept() -> dict[str, bool]:
        """Say whether uploads are kept with their results, for the upload page to say."""
        return {"uploads": server.settings.keep_uploads}

    @app.post("/api/jobs", status_code=201)
    def create(request: NewJob) -> dict[str, str]:
        """Make a job, and give the link to upload to."""
        if not _matches(request.code, server.settings.invite_codes):
            raise HTTPException(403, "that invite code is not one we know")
        if request.size > config.MAX_BYTES:
            raise HTTPException(413, f"the file is over {config.MAX_BYTES / 10**6:g} MB")
        owner = hmac.new(
            server.settings.signing_key, request.code.encode(), hashlib.sha256
        ).hexdigest()
        try:
            job = server.jobs.create(request.upload, request.size, owner)
        except jobs.OwnerFullError:
            raise HTTPException(
                429, f"this invite code has {jobs.CODE_JOBS} jobs open: wait for one to end"
            ) from None
        except jobs.FullError:
            raise HTTPException(503, "the queue is full: please try again later") from None
        _LOG.info("job %s: made for %s of %d bytes, by %s", job.id, job.upload, job.size, owner[:8])
        link = server.store.upload_link(upload_key(job.id), job.size, int(config.UPLOAD_WAIT))
        return {"token": job.token, "upload_url": link, "link": f"/r/{job.token}"}

    @app.post("/api/jobs/{token}/uploaded")
    def uploaded(token: str) -> dict[str, str]:
        """Queue a job once its whole upload is in storage."""
        job = server.owned(token)
        if job.state != "waiting":
            raise HTTPException(409, f"the job is {job.state}, not waiting for its upload")
        if server.store.size(upload_key(job.id)) != job.size:
            raise HTTPException(400, "the upload has not arrived whole")
        try:
            server.store.move(upload_key(job.id), import_key(job.id))  # out of the link's reach
        except FileNotFoundError:  # moved by a confirmation a moment before
            raise HTTPException(409, "the upload was confirmed already") from None
        except Exception:  # storage's own error (S3's among them): the job stays waiting
            _LOG.exception("job %s: the upload could not be moved", job.id)
            raise HTTPException(503, "storage did not answer: please confirm again") from None
        try:
            job = server.jobs.queue(token)
        except jobs.StateError:
            raise HTTPException(409, "the job is no longer waiting for its upload") from None
        _LOG.info("job %s: queued", job.id)
        return {"state": job.state}

    @app.get("/api/jobs/{token}")
    def status(token: str) -> dict[str, Any]:
        """Say how a job is going."""
        job = server.owned(token)
        answer: dict[str, Any] = {
            "state": job.state,
            "upload": job.upload,
            "created": job.created,
            "reason": job.reason,
        }
        if job.state == "queued":
            answer["ahead"] = server.jobs.ahead(job)
        if job.state in jobs.ENDED:
            answer["deleted"] = job.expires  # when the job and its result are deleted
        if job.state == "done":
            answer["summary"] = job.summary
            answer["bundle"] = f"/r/{job.token}/bundle/"
        return answer


def _result_files(request: Files) -> dict[str, int]:
    """Return a result's files by path, refusing repeats, bad paths, and too many bytes."""
    listed = {f.path: f.size for f in request.files}
    if len(listed) != len(request.files):
        raise HTTPException(400, "files: a path is listed twice")
    if sum(listed.values()) > RESULT_BYTES:
        raise HTTPException(413, f"files: over {RESULT_BYTES / 10**6:g} MB in all")
    for path in listed:
        if not RESULT_PATH.fullmatch(path):
            raise HTTPException(400, f"files: not a bundle's file: {path!r}")
    return listed


def _result_links(store: Storage, job_id: str, listed: dict[str, int]) -> dict[str, str]:
    """Return an upload link for each of a result's files, for its size, for an hour."""
    prefix, seconds = result_prefix(job_id), config.LINK_SECONDS
    return {
        path: store.upload_link(f"{prefix}/{path}", size, seconds) for path, size in listed.items()
    }


def _check_arrived(store: Storage, job: Job) -> None:
    """Refuse a job done before every file of its result is in storage, whole."""
    for path, size in job.files.items():
        if store.size(f"{result_prefix(job.id)}/{path}") != size:
            raise HTTPException(400, f"files: {path} has not arrived whole")


def _worker_api(app: FastAPI, server: _Server) -> None:
    """Add the worker's API (:mod:`slam_it_all.orchestration.worker`), behind its token."""

    def worker(authorization: Annotated[str, Header()] = "") -> None:
        """Refuse a request without the worker's token."""
        scheme, _, token = authorization.partition(" ")
        if scheme != "Bearer" or not _matches(token, (server.settings.worker_token,)):
            raise HTTPException(401, "a worker's token is needed")

    guarded = [Depends(worker)]

    @app.post("/api/worker/claim", dependencies=guarded, response_model=None)
    async def claim(request: Claim) -> dict[str, str] | Response:
        """Give the worker a job, waiting for one up to the claim's wait."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + request.wait
        while (job := await asyncio.to_thread(server.jobs.claim, request.kinds)) is None:
            if loop.time() >= deadline:
                return Response(status_code=204)
            await asyncio.sleep(CLAIM_POLL)
        _LOG.info("job %s: claimed", job.id)
        link = server.store.download_link(import_key(job.id))
        return {"job": job.id, "kind": job.kind, "upload": job.upload, "download": link}

    @app.post("/api/worker/jobs/{job_id}/progress", dependencies=guarded)
    def progress(job_id: str, request: Progress) -> dict[str, str]:
        """Record a stage; once the import has ended, delete the upload unless uploads are kept."""
        try:
            before, job = server.jobs.progress(job_id, request.stage)
        except jobs.StateError:
            raise HTTPException(409, "the job is no longer yours") from None
        except ValueError as error:
            raise HTTPException(400, str(error)) from None
        if before == "importing" and job.state != "importing" and not server.settings.keep_uploads:
            key = import_key(job.id)
            _best_effort("deleting the upload", job, lambda: server.store.delete(key))
        return {}

    @app.post("/api/worker/jobs/{job_id}/files", dependencies=guarded)
    def files(job_id: str, request: Files) -> dict[str, dict[str, str]]:
        """Record a result's files, and give an upload link for each."""
        listed = _result_files(request)
        _working(lambda: server.jobs.record_files(job_id, listed))
        return {"urls": _result_links(server.store, job_id, listed)}

    @app.post("/api/worker/jobs/{job_id}/done", dependencies=guarded)
    def done(job_id: str, request: Done) -> dict[str, str]:
        """End a job done, once every file of its result is in storage."""
        if len(json.dumps(request.summary)) > SUMMARY_BYTES:
            raise HTTPException(413, "summary: too large")
        _check_arrived(server.store, _working(lambda: server.jobs.held(job_id)))
        job = _working(lambda: server.jobs.done(job_id, request.summary))
        _LOG.info("job %s: done", job.id)
        if not server.settings.keep_uploads:
            key = import_key(job.id)
            _best_effort("deleting the upload", job, lambda: server.store.delete(key))
        return {}

    @app.post("/api/worker/jobs/{job_id}/failed", dependencies=guarded)
    def failed(job_id: str, request: Failed) -> dict[str, str]:
        """End a job failed, deleting what of its result arrived, and its upload unless kept."""
        job = _working(lambda: server.jobs.fail(job_id, request.reason[:REASON_LENGTH]))
        _LOG.info("job %s: failed (%s)", job.id, job.reason)
        key, prefix = import_key(job.id), result_prefix(job.id)
        if not server.settings.keep_uploads:
            _best_effort("deleting the upload", job, lambda: server.store.delete(key))
        _best_effort("deleting the result", job, lambda: server.store.delete_prefix(prefix))
        return {}


def _folder_storage(app: FastAPI, folder: FolderStorage) -> None:
    """Add folder storage's routes: a signed link's PUT and GET, served by the server itself."""

    def checked(request: Request, method: str, key: str) -> tuple[int, int]:
        """Check a link's signature; return the size it was signed for, and its expiry."""
        query = request.query_params
        try:
            expires, size = int(query.get("expires", "")), int(query.get("size", ""))
            folder.verify(method, key, expires, size, query.get("signature", ""))
        except (ValueError, StorageError):
            raise HTTPException(403, "the link is not valid") from None
        return size, expires

    @app.put("/storage/{key:path}")
    async def put_file(key: str, request: Request) -> dict[str, str]:
        """Take a file of exactly the size its link was signed for, whole before it expires."""
        size, expires = checked(request, "PUT", key)
        if request.headers.get("content-length") != str(size):
            raise HTTPException(400, "the upload is not the size its link was signed for")
        target = folder.path(key)
        target.parent.mkdir(parents=True, exist_ok=True)
        staged = target.with_name(f".{target.name}.{uuid.uuid4().hex}.part")
        written = 0
        try:
            with staged.open("wb") as file:
                async for chunk in request.stream():
                    written += len(chunk)
                    if written > size:
                        raise HTTPException(400, "the upload is larger than its link allows")
                    await asyncio.to_thread(file.write, chunk)
            if written != size:
                raise HTTPException(400, "the upload ended short")
            if expires < folder.clock():  # begun in time, it arrived after: nothing kept
                raise HTTPException(403, "the link expired before the upload arrived whole")
            staged.replace(target)
        finally:
            staged.unlink(missing_ok=True)
        return {}

    @app.get("/storage/{key:path}")
    def get_file(key: str, request: Request) -> FileResponse:
        """Serve a file its link names."""
        checked(request, "GET", key)
        path = folder.path(key)
        if not path.is_file():
            raise HTTPException(404)
        # Never as a page of this site: whatever a file holds, it is a download.
        return FileResponse(
            path,
            media_type="application/octet-stream",
            headers={"Content-Disposition": "attachment"},
        )


def _pages(app: FastAPI, server: _Server) -> None:
    """Add the pages, a done job's bundle, robots.txt and the health check."""

    @app.get("/")
    def upload_page() -> FileResponse:
        """Serve the upload page."""
        return server.page("upload.html")

    @app.get("/r/{token}")
    def status_page(token: str) -> FileResponse:
        """Serve a job's status page, which reads its token from its address."""
        del token
        return server.page("status.html")

    @app.get("/r/{token}/bundle/{path:path}")
    def bundle_file(token: str, path: str) -> RedirectResponse:
        """Redirect to a done job's file in storage."""
        job = server.owned(token)
        if job.state != "done" or path not in job.files:
            raise HTTPException(404)
        link = server.store.download_link(f"{result_prefix(job.id)}/{path}")
        return RedirectResponse(link, 307)

    @app.get("/robots.txt")
    def robots() -> Response:
        """Ask every crawler to stay away."""
        return Response("User-agent: *\nDisallow: /\n", media_type="text/plain")

    @app.get("/healthz")
    def health() -> dict[str, bool]:
        """Say the server is up."""
        return {"ok": True}

    @app.get("/{name:path}")
    def viewer_file(name: str) -> FileResponse:
        """Serve one of the viewer folder's files that :data:`PAGES` allows."""
        if not PAGES.match(name) or not (server.settings.viewer / name).is_file():
            raise HTTPException(404)
        return server.page(name)
