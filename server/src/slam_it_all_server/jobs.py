"""The job list: one SQLite table, every change one transaction.

docs/design/2026-10-09-run-viewer.md, section 5.3. A job's life::

    waiting ─▶ queued ─▶ downloading ─▶ importing ─▶ running ─▶ baking ─▶ uploading ─▶ done
       │          │          └──────────────────────┬────────────────────────┘
       └──────────┴─▶ expired                       └─▶ failed

- **waiting**: made for an upload of a declared kind and size, by someone holding an invite
  code. It expires after :data:`~slam_it_all_server.config.UPLOAD_WAIT` unless its upload is
  confirmed.
- **queued**: its upload is in storage. It expires after :data:`QUEUE_WAIT` unless a worker
  claims it.
- **the worker's stages** (:data:`STAGES`): a worker has it, one job at a time. It fails if not
  heard from for :data:`~slam_it_all_server.config.SILENCE`.
- **done**, **failed**, **expired**: kept :data:`~slam_it_all_server.config.RETENTION` so that its
  link says how it ended, then deleted with its files.

A job has two names, neither made from the other: its ``id``, which workers and storage keys
use, and its ``token``, the owner's 128-bit link. A worker never learns a job's token.
"""

from __future__ import annotations

import json
import secrets
import sqlite3
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from slam_it_all_server.config import QUEUE, RETENTION, SILENCE, UPLOAD_WAIT

STAGES = ("downloading", "importing", "running", "baking", "uploading")
"""The worker's stages, in order (``slam_it_all.orchestration.worker.STAGES``; a test keeps the
two the same, since the server never imports the worker)."""

ENDED = ("done", "failed", "expired")
"""The states a job ends in."""

UPLOADS = ("video", "stray")
"""What may be uploaded: a video, or a Stray Scanner zip."""

KINDS = ("vo",)
"""The kinds of job there are: what a worker runs."""

QUEUE_WAIT = 24 * 3600.0
"""How long a queued job waits for a worker, in seconds, before it expires."""

_SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    id TEXT PRIMARY KEY,
    token TEXT NOT NULL UNIQUE,
    kind TEXT NOT NULL,
    upload TEXT NOT NULL,
    size INTEGER NOT NULL,
    state TEXT NOT NULL,
    created REAL NOT NULL,
    heard REAL NOT NULL,
    expires REAL NOT NULL,
    reason TEXT,
    summary TEXT,
    files TEXT NOT NULL DEFAULT '{}'
)
"""


class FullError(Exception):
    """The queue is full: the owner is asked to try later."""


class StateError(Exception):
    """A job is not in a state that allows this (for a worker: it is no longer its job)."""


@dataclass(frozen=True)
class Job:
    """A job, as the list holds it.

    Attributes
    ----------
    id : str
        Its name for workers and storage.
    token : str
        Its owner's link.
    kind : str
        What it runs (one of :data:`KINDS`).
    upload : str
        What is uploaded (one of :data:`UPLOADS`).
    size : int
        The upload's declared size, in bytes.
    state : str
        Where it is (the module docstring).
    created : float
        When it was made, in seconds since the epoch.
    heard : float
        When it last changed or a worker last reported on it.
    expires : float
        When it expires, fails for silence, or is deleted, by its state.
    reason : str or None
        Why it failed or expired.
    summary : dict or None
        The worker's summary of a job done.
    files : dict of str to int
        The result's files, by path, with their sizes.
    """

    id: str
    token: str
    kind: str
    upload: str
    size: int
    state: str
    created: float
    heard: float
    expires: float
    reason: str | None
    summary: dict[str, Any] | None
    files: dict[str, int]

    @property
    def working(self) -> bool:
        """Whether a worker has it."""
        return self.state in STAGES


def _job(row: sqlite3.Row) -> Job:
    """Return a row as a job."""
    summary = row["summary"]
    return Job(
        id=row["id"],
        token=row["token"],
        kind=row["kind"],
        upload=row["upload"],
        size=row["size"],
        state=row["state"],
        created=row["created"],
        heard=row["heard"],
        expires=row["expires"],
        reason=row["reason"],
        summary=None if summary is None else json.loads(summary),
        files={str(k): int(v) for k, v in json.loads(row["files"]).items()},
    )


@dataclass(frozen=True)
class Swept:
    """What a sweep changed: the jobs it ended, and the jobs it deleted."""

    ended: list[Job]
    deleted: list[Job]


class Jobs:
    """The job list, in an SQLite file.

    Parameters
    ----------
    path : pathlib.Path
        The file, made if it is not there.
    clock : callable, optional
        Seconds since the epoch (default: :func:`time.time`).
    """

    def __init__(self, path: Path, clock: Callable[[], float] = time.time) -> None:
        """Open the list, making its table if need be."""
        self.path, self.clock = path, clock
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._changing() as db:
            db.execute(_SCHEMA)

    @contextmanager
    def _changing(self) -> Iterator[sqlite3.Connection]:
        """Yield a connection inside one write transaction, committed if nothing raised."""
        db = sqlite3.connect(self.path, isolation_level=None, timeout=10.0)
        db.row_factory = sqlite3.Row
        try:
            db.execute("BEGIN IMMEDIATE")
            try:
                yield db
            except BaseException:
                db.execute("ROLLBACK")
                raise
            db.execute("COMMIT")
        finally:
            db.close()

    def _one(self, db: sqlite3.Connection, column: str, value: str) -> Job | None:
        """Return the job whose `column` is `value`, if there is one."""
        row = db.execute(f"SELECT * FROM jobs WHERE {column} = ?", (value,)).fetchone()
        return None if row is None else _job(row)

    def _set(self, db: sqlite3.Connection, job_id: str, **values: Any) -> Job:  # noqa: ANN401
        """Change a job's columns and return it."""
        columns = ", ".join(f"{name} = ?" for name in values)
        db.execute(f"UPDATE jobs SET {columns} WHERE id = ?", (*values.values(), job_id))
        job = self._one(db, "id", job_id)
        if job is None:
            raise StateError("no such job")
        return job

    def _working(self, db: sqlite3.Connection, job_id: str) -> Job:
        """Return a job a worker has, refusing one it no longer has."""
        job = self._one(db, "id", job_id)
        if job is None or not job.working:
            raise StateError("not a job in progress")
        return job

    def create(self, upload: str, size: int, kind: str = "vo") -> Job:
        """Make a job waiting for its upload; :class:`FullError` if :data:`QUEUE` are open."""
        if upload not in UPLOADS or kind not in KINDS:
            raise StateError(f"not an upload or a kind of job there is: {upload}, {kind}")
        now = self.clock()
        with self._changing() as db:
            (open_jobs,) = db.execute(
                f"SELECT COUNT(*) FROM jobs WHERE state NOT IN ({', '.join('?' * len(ENDED))})",
                ENDED,
            ).fetchone()
            if open_jobs >= QUEUE:
                raise FullError("the queue is full")
            job_id, token = secrets.token_hex(16), secrets.token_urlsafe(16)
            db.execute(
                "INSERT INTO jobs (id, token, kind, upload, size, state, created, heard, expires)"
                " VALUES (?, ?, ?, ?, ?, 'waiting', ?, ?, ?)",
                (job_id, token, kind, upload, size, now, now, now + UPLOAD_WAIT),
            )
            job = self._one(db, "id", job_id)
        assert job is not None  # made above
        return job

    def by_token(self, token: str) -> Job | None:
        """Return the job a link names, if there is one."""
        with self._changing() as db:
            return self._one(db, "token", token)

    def by_id(self, job_id: str) -> Job | None:
        """Return a job by its id, if there is one."""
        with self._changing() as db:
            return self._one(db, "id", job_id)

    def working(self, job_id: str) -> Job:
        """Return a job a worker has; :class:`StateError` if it is no longer the worker's."""
        with self._changing() as db:
            return self._working(db, job_id)

    def ahead(self, job: Job) -> int:
        """Return how many jobs will be worked on before a queued one."""
        with self._changing() as db:
            stages = ", ".join("?" * len(STAGES))
            (count,) = db.execute(
                f"SELECT COUNT(*) FROM jobs WHERE state IN ({stages})"
                " OR (state = 'queued' AND created < ?)",
                (*STAGES, job.created),
            ).fetchone()
        return int(count)

    def queue(self, token: str) -> Job:
        """Queue a job whose upload is confirmed (its caller checks the upload's size)."""
        now = self.clock()
        with self._changing() as db:
            job = self._one(db, "token", token)
            if job is None or job.state != "waiting":
                raise StateError("not a job waiting for its upload")
            return self._set(db, job.id, state="queued", heard=now, expires=now + QUEUE_WAIT)

    def claim(self, kinds: Sequence[str]) -> Job | None:
        """Give a worker the oldest queued job of its kinds, if none is in progress."""
        now = self.clock()
        with self._changing() as db:
            stages = ", ".join("?" * len(STAGES))
            if db.execute(f"SELECT 1 FROM jobs WHERE state IN ({stages})", STAGES).fetchone():
                return None  # one job at a time
            wanted = [k for k in kinds if k in KINDS]
            if not wanted:
                return None
            places = ", ".join("?" * len(wanted))
            row = db.execute(
                f"SELECT id FROM jobs WHERE state = 'queued' AND kind IN ({places})"
                " ORDER BY created LIMIT 1",
                wanted,
            ).fetchone()
            if row is None:
                return None
            return self._set(db, row["id"], state=STAGES[0], heard=now, expires=now + SILENCE)

    def progress(self, job_id: str, stage: str) -> tuple[str, Job]:
        """Record a worker's report of a stage: a new one, or the same again (a heartbeat).

        Returns
        -------
        tuple of str and Job
            The stage the job was at before, and the job.
        """
        if stage not in STAGES:
            raise ValueError(f"stage: not one of {', '.join(STAGES)}")
        now = self.clock()
        with self._changing() as db:
            job = self._working(db, job_id)
            if STAGES.index(stage) < STAGES.index(job.state):
                raise ValueError(f"stage: {stage} comes before {job.state}")
            return job.state, self._set(db, job_id, state=stage, heard=now, expires=now + SILENCE)

    def record_files(self, job_id: str, files: dict[str, int]) -> Job:
        """Record the result's files, as a worker is about to upload them."""
        now = self.clock()
        with self._changing() as db:
            self._working(db, job_id)
            return self._set(db, job_id, files=json.dumps(files), heard=now, expires=now + SILENCE)

    def done(self, job_id: str, summary: dict[str, Any]) -> Job:
        """End a job done, its result kept for :data:`RETENTION`."""
        now = self.clock()
        with self._changing() as db:
            self._working(db, job_id)
            return self._set(
                db,
                job_id,
                state="done",
                summary=json.dumps(summary),
                heard=now,
                expires=now + RETENTION,
            )

    def fail(self, job_id: str, reason: str) -> Job:
        """End a job a worker has as failed, with a reason fit to show its owner."""
        now = self.clock()
        with self._changing() as db:
            self._working(db, job_id)
            return self._set(
                db, job_id, state="failed", reason=reason, heard=now, expires=now + RETENTION
            )

    def sweep(self) -> Swept:
        """End the jobs whose time is up, and delete the ended ones kept long enough.

        Returns
        -------
        Swept
            The jobs ended (as they are now) and the jobs deleted (as they were), so that their
            files can be deleted.
        """
        now = self.clock()
        ended, deleted = [], []
        with self._changing() as db:
            rows = db.execute("SELECT * FROM jobs WHERE expires <= ?", (now,)).fetchall()
            for job in map(_job, rows):
                if job.state in ENDED:
                    db.execute("DELETE FROM jobs WHERE id = ?", (job.id,))
                    deleted.append(job)
                    continue
                state, reason = {
                    "waiting": ("expired", "the upload did not arrive in time"),
                    "queued": ("expired", "no worker took it in time"),
                }.get(job.state, ("failed", "the worker stopped answering"))
                ended.append(
                    self._set(
                        db, job.id, state=state, reason=reason, heard=now, expires=now + RETENTION
                    )
                )
        return Swept(ended, deleted)
