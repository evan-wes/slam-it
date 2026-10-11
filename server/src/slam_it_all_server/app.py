"""The server's routes: the pages, the owner's API, the worker's API, and folder storage's links.

docs/design/2026-10-09-run-viewer.md, sections 5.3 and 5.4.

**The pages** come from the viewer's folder (only the files :data:`PAGES` matches): the upload
page at ``/``, a job's status at ``/r/<token>``, and the run viewer (``/index.html``), which
reads a done job's bundle from ``/r/<token>/bundle/``. A bundle's files are redirected to
storage's own links, never proxied.

**The owner's API:**

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
- an upload deleted once its import ends, and every job deleted with its files
  :data:`~slam_it_all_server.config.RETENTION` after it ends;
- results reachable only through their token, pages marked ``noindex``, and no ``Referer``
  sent, so a token never leaves in one;
- request bodies capped at :data:`BODY_BYTES`, except folder storage's signed uploads.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import contextlib
import hmac
import json
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
from slam_it_all_server.storage import FolderStorage, S3Storage, Storage, StorageError, check_key

PAGES = re.compile(r"^(index\.html|upload\.html|status\.html|app\.css|src/[a-z_]+\.js)$")
"""The viewer folder's files that are served: its pages, their style and their modules."""

RESULT_FILES = 1000
"""The most files a result may have."""

RESULT_BYTES = 10**9
"""The most bytes a result may hold: a bound on a worker gone wrong, well above a bundle's size."""

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

_HEADERS = {
    "X-Robots-Tag": "noindex, nofollow",
    "Referrer-Policy": "no-referrer",
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
}
"""Headers on every response."""

_OPEN = ("/api/worker/", "/storage/", "/healthz")
"""Paths staging's Basic auth leaves open: they carry their own token or signature."""


def upload_key(job_id: str) -> str:
    """Return the storage key of a job's upload."""
    return f"uploads/{job_id}"


def result_prefix(job_id: str) -> str:
    """Return the storage prefix of a job's result."""
    return f"results/{job_id}"


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


def sweep(job_list: Jobs, store: Storage) -> jobs.Swept:
    """Sweep the job list, deleting the files of the jobs it ended or deleted.

    An ended job's upload goes, and a failed or deleted job's result (:meth:`Jobs.sweep`).
    """
    swept = job_list.sweep()
    for job in swept.ended:
        store.delete(upload_key(job.id))
        if job.state == "failed":
            store.delete_prefix(result_prefix(job.id))
    for job in swept.deleted:
        store.delete(upload_key(job.id))
        store.delete_prefix(result_prefix(job.id))
    return swept


class NewJob(BaseModel):
    """An owner's request for a job."""

    code: str = Field(max_length=200)
    upload: Literal["video", "stray"]
    size: StrictInt = Field(gt=0, alias="bytes")


class Claim(BaseModel):
    """A worker's claim."""

    kinds: list[str] = Field(max_length=10)
    wait: float = Field(ge=0, le=60)


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
    """
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
                with contextlib.suppress(Exception):  # a sweep that fails is tried again
                    await asyncio.to_thread(sweep, server.jobs, server.store)
                await asyncio.sleep(SWEEP)

        task = asyncio.create_task(sweeping())
        yield
        task.cancel()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)

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
        answer.headers.update(_HEADERS)
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

    @app.post("/api/jobs", status_code=201)
    def create(request: NewJob) -> dict[str, str]:
        """Make a job, and give the link to upload to."""
        if not _matches(request.code, server.settings.invite_codes):
            raise HTTPException(403, "that invite code is not one we know")
        if request.size > config.MAX_BYTES:
            raise HTTPException(413, f"the file is over {config.MAX_BYTES / 10**6:g} MB")
        try:
            job = server.jobs.create(request.upload, request.size)
        except jobs.FullError:
            raise HTTPException(503, "the queue is full: please try again later") from None
        return {
            "token": job.token,
            "upload_url": server.store.upload_link(upload_key(job.id), job.size),
            "link": f"/r/{job.token}",
        }

    @app.post("/api/jobs/{token}/uploaded")
    def uploaded(token: str) -> dict[str, str]:
        """Queue a job once its whole upload is in storage."""
        job = server.owned(token)
        if job.state != "waiting":
            raise HTTPException(409, f"the job is {job.state}, not waiting for its upload")
        if server.store.size(upload_key(job.id)) != job.size:
            raise HTTPException(400, "the upload has not arrived whole")
        try:
            job = server.jobs.queue(token)
        except jobs.StateError:
            raise HTTPException(409, "the job is no longer waiting for its upload") from None
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
    try:
        for path in listed:
            check_key(path)
    except StorageError as error:
        raise HTTPException(400, str(error)) from None
    return listed


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
        while (job := server.jobs.claim(request.kinds)) is None:
            if loop.time() >= deadline:
                return Response(status_code=204)
            await asyncio.sleep(CLAIM_POLL)
        link = server.store.download_link(upload_key(job.id))
        return {"job": job.id, "kind": job.kind, "upload": job.upload, "download": link}

    @app.post("/api/worker/jobs/{job_id}/progress", dependencies=guarded)
    def progress(job_id: str, request: Progress) -> dict[str, str]:
        """Record a stage; once the import has ended, delete the upload."""
        try:
            before, job = server.jobs.progress(job_id, request.stage)
        except jobs.StateError:
            raise HTTPException(409, "the job is no longer yours") from None
        except ValueError as error:
            raise HTTPException(400, str(error)) from None
        if before == "importing" and job.state != "importing":
            server.store.delete(upload_key(job.id))
        return {}

    @app.post("/api/worker/jobs/{job_id}/files", dependencies=guarded)
    def files(job_id: str, request: Files) -> dict[str, dict[str, str]]:
        """Record a result's files, and give an upload link for each."""
        listed = _result_files(request)
        _working(lambda: server.jobs.record_files(job_id, listed))
        keys = {path: f"{result_prefix(job_id)}/{path}" for path in listed}
        return {
            "urls": {path: server.store.upload_link(keys[path], listed[path]) for path in listed}
        }

    @app.post("/api/worker/jobs/{job_id}/done", dependencies=guarded)
    def done(job_id: str, request: Done) -> dict[str, str]:
        """End a job done, once every file of its result is in storage."""
        if len(json.dumps(request.summary)) > SUMMARY_BYTES:
            raise HTTPException(413, "summary: too large")
        job = _working(lambda: server.jobs.working(job_id))
        for path, size in job.files.items():
            if server.store.size(f"{result_prefix(job.id)}/{path}") != size:
                raise HTTPException(400, f"files: {path} has not arrived whole")
        job = _working(lambda: server.jobs.done(job_id, request.summary))
        server.store.delete(upload_key(job.id))
        return {}

    @app.post("/api/worker/jobs/{job_id}/failed", dependencies=guarded)
    def failed(job_id: str, request: Failed) -> dict[str, str]:
        """End a job failed, deleting its upload and whatever of its result arrived."""
        job = _working(lambda: server.jobs.fail(job_id, request.reason[:REASON_LENGTH]))
        server.store.delete(upload_key(job.id))
        server.store.delete_prefix(result_prefix(job.id))
        return {}


def _folder_storage(app: FastAPI, folder: FolderStorage) -> None:
    """Add folder storage's routes: a signed link's PUT and GET, served by the server itself."""

    def checked(request: Request, method: str, key: str) -> int:
        """Check a link's signature; return the size it was signed for."""
        query = request.query_params
        try:
            expires, size = int(query.get("expires", "")), int(query.get("size", ""))
            folder.verify(method, key, expires, size, query.get("signature", ""))
        except (ValueError, StorageError):
            raise HTTPException(403, "the link is not valid") from None
        return size

    @app.put("/storage/{key:path}")
    async def put_file(key: str, request: Request) -> dict[str, str]:
        """Take a file of exactly the size its link was signed for."""
        size = checked(request, "PUT", key)
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
                    file.write(chunk)
            if written != size:
                raise HTTPException(400, "the upload ended short")
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
        return FileResponse(path)


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
