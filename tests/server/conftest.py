"""The upload server's fixtures: settings for a folder on this machine, a clock, and the app.

The server is tested as the design asks (docs/design/2026-10-09-run-viewer.md, section 6): with
FastAPI's test client and folder storage, its links signed and served by the app itself.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from slam_it_all_server.app import make_app
from slam_it_all_server.config import Settings
from slam_it_all_server.jobs import Jobs
from slam_it_all_server.storage import FolderStorage

VIEWER = Path(__file__).resolve().parents[2] / "viewer"
"""The repository's viewer folder, which holds the pages."""

CODE = "an-invite-code"
"""The invite code the tests' settings accept."""

TOKEN = "a-worker-token-of-at-least-32-characters"
"""The worker token the tests' settings accept."""


@dataclass
class Clock:
    """A clock the tests move: seconds since the epoch."""

    now: float = 1_000_000.0

    def __call__(self) -> float:
        """Return the time."""
        return self.now


def make_settings(
    folder: Path, url: str = "http://testserver", *, staging: bool = False
) -> Settings:
    """Return settings that keep files and the job list in a folder."""
    return Settings(
        invite_codes=(CODE, "another-invite-code"),
        worker_token=TOKEN,
        signing_key=b"a-signing-key-of-at-least-32-characters",
        storage=f"folder:{folder / 'files'}",
        public_url=url,
        database=folder / "jobs.sqlite3",
        viewer=VIEWER,
        staging=staging,
        s3={},
    )


@dataclass
class Server:
    """An app and what it holds."""

    client: TestClient
    jobs: Jobs
    store: FolderStorage
    clock: Clock
    settings: Settings


@pytest.fixture
def server(tmp_path: Path) -> Server:
    """Return the app on a fresh folder, its job list on a clock the test moves."""
    settings = make_settings(tmp_path)
    clock = Clock()
    jobs = Jobs(settings.database, clock)
    store = FolderStorage(tmp_path / "files", settings.public_url, settings.signing_key)
    client = TestClient(make_app(settings, store, jobs))
    return Server(client, jobs, store, clock, settings)


@pytest.fixture
def staging(server: Server) -> TestClient:
    """Return a client of the same app with staging's Basic auth on."""
    settings = replace(server.settings, staging=True)
    return TestClient(make_app(settings, server.store, server.jobs))


WORKER = {"Authorization": f"Bearer {TOKEN}"}
"""The worker's headers."""
