"""The server's settings, read from the environment: no deployment's specifics are in the repo.

``server/settings.template.env`` lists every variable, with placeholder values.

- ``SLAM_IT_INVITE_CODES``: the codes that may upload, comma-separated.
- ``SLAM_IT_WORKER_TOKEN``: the token workers present.
- ``SLAM_IT_SIGNING_KEY``: the key folder storage signs its links with.
- ``SLAM_IT_STORAGE``: ``s3``, or ``folder:<path>`` to keep files on this machine.
- ``SLAM_IT_S3_ENDPOINT``, ``SLAM_IT_S3_BUCKET``, ``SLAM_IT_S3_KEY_ID``,
  ``SLAM_IT_S3_SECRET``, ``SLAM_IT_S3_REGION``: the bucket, for ``s3``.
- ``SLAM_IT_PUBLIC_URL``: the server's own address, for the links folder storage makes.
- ``SLAM_IT_DATABASE``: the job list's SQLite file.
- ``SLAM_IT_VIEWER``: the run viewer's folder (``viewer/``), which the server serves.
- ``SLAM_IT_STAGING``: ``1`` puts every page behind an invite code (HTTP Basic auth).
- ``SLAM_IT_KEEP_UPLOADS``: ``1`` keeps each upload with its result until the job is
  deleted, :data:`RETENTION` after it ends, rather than deleting it once imported: the demo's
  (``scripts/serve_demo.py``), so that its recordings can be run again. With folder storage
  only, so that a deployment, on S3, never keeps one.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

MAX_BYTES = 500 * 10**6
"""The largest upload, video or zip, in bytes (the import's cap)."""

QUEUE = 5
"""The most jobs waiting, queued or running at once: past it, the page asks to try later."""

RETENTION = 7 * 24 * 3600.0
"""How long a finished job's result is kept, in seconds."""

UPLOAD_WAIT = 15 * 60.0
"""How long a job waits for its upload, in seconds, before it expires: its upload link's life
too, so that a link outlives no job waiting for it."""

UPLOAD_GRACE = 3600.0
"""How long after a link expires an upload begun in time may still be arriving, in seconds. A
link is checked when an upload starts, not when it ends, so what a job's links wrote is cleared
a last time only once every link has expired and this has passed: for a job that expired
waiting, 135 minutes after it was made, which 500 MB takes at about 0.5 Mbit/s (an estimate)."""

SILENCE = 120.0
"""How long a running job may go unheard from, in seconds (four of the worker's heartbeats),
before it fails."""

KEEP_FREE = 10 * MAX_BYTES
"""With uploads kept (the demo's), the least free space under folder storage for a new job, in
bytes: ten uploads' worth, so that kept uploads, each up to :data:`MAX_BYTES` for
:data:`RETENTION`, never fill the disk."""

LINK_SECONDS = 3600
"""How long a signed link lasts, in seconds."""

CODE_LENGTH = 12
"""The shortest invite code, in characters: codes are typed, so not long, but never guessable."""

SECRET_LENGTH = 32
"""The shortest worker token and signing key, in characters."""


class SettingsError(Exception):
    """A setting is missing or malformed."""


@dataclass(frozen=True)
class Settings:
    """The server's settings (the module docstring names their variables)."""

    invite_codes: tuple[str, ...]
    worker_token: str
    signing_key: bytes
    storage: str
    public_url: str
    database: Path
    viewer: Path
    staging: bool
    s3: Mapping[str, str]
    keep_uploads: bool = False


def _required(env: Mapping[str, str], name: str) -> str:
    """Return a variable's value, refusing one unset or empty."""
    value = env.get(name, "").strip()
    if not value:
        raise SettingsError(f"{name} is not set")
    return value


def from_environment(env: Mapping[str, str] | None = None) -> Settings:
    """Read the settings from the environment (the module docstring).

    Parameters
    ----------
    env : mapping of str to str, optional
        The variables (default: this process's environment).

    Returns
    -------
    Settings
        The settings.

    Raises
    ------
    SettingsError
        If a variable is missing or malformed.
    """
    env = os.environ if env is None else env
    codes = tuple(c.strip() for c in _required(env, "SLAM_IT_INVITE_CODES").split(",") if c.strip())
    if any(len(code) < CODE_LENGTH for code in codes):
        raise SettingsError(f"SLAM_IT_INVITE_CODES: each code needs {CODE_LENGTH} characters")
    secrets = {
        name: _required(env, name) for name in ("SLAM_IT_WORKER_TOKEN", "SLAM_IT_SIGNING_KEY")
    }
    for name, value in secrets.items():
        if len(value) < SECRET_LENGTH:
            raise SettingsError(f"{name}: needs {SECRET_LENGTH} characters")
    storage = _required(env, "SLAM_IT_STORAGE")
    if storage != "s3" and not storage.startswith("folder:"):
        raise SettingsError("SLAM_IT_STORAGE: expected s3 or folder:<path>")
    s3: dict[str, str] = {}
    if storage == "s3":
        for key in ("ENDPOINT", "BUCKET", "KEY_ID", "SECRET", "REGION"):
            s3[key.lower()] = _required(env, f"SLAM_IT_S3_{key}")
    keep_uploads = env.get("SLAM_IT_KEEP_UPLOADS", "0") == "1"
    if keep_uploads and storage == "s3":
        raise SettingsError(
            "SLAM_IT_KEEP_UPLOADS: only with folder storage (the demo's); a deployment "
            "deletes each upload once imported"
        )
    return Settings(
        invite_codes=codes,
        worker_token=secrets["SLAM_IT_WORKER_TOKEN"],
        signing_key=secrets["SLAM_IT_SIGNING_KEY"].encode(),
        storage=storage,
        public_url=_required(env, "SLAM_IT_PUBLIC_URL").rstrip("/"),
        database=Path(_required(env, "SLAM_IT_DATABASE")),
        viewer=Path(_required(env, "SLAM_IT_VIEWER")),
        staging=env.get("SLAM_IT_STAGING", "0") == "1",
        s3=s3,
        keep_uploads=keep_uploads,
    )
