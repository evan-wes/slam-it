"""The caps and names that the import, the worker, the server and the pages each hold, held equal.

The server never imports slam_it, and the pages are JavaScript, so each keeps its own copy.
"""

from __future__ import annotations

import re

from slam_it.eval import lenses, video
from slam_it.orchestration import worker
from slam_it_server import app, config, jobs
from tests.server.conftest import VIEWER

PAGE = (VIEWER / "src" / "jobs.js").read_text()
"""The pages' logic module."""


def _constant(name: str) -> str:
    """Return the source of one of the pages' constants."""
    found = re.search(rf"^export const {name} = (.*?);$", PAGE, re.MULTILINE | re.DOTALL)
    assert found is not None, name
    return found.group(1)


def test_the_caps_are_the_imports() -> None:
    """The server's and the page's size cap, and the page's length cap, are the import's."""
    assert config.MAX_BYTES == video.MAX_BYTES
    assert float(_constant("MAX_BYTES")) == video.MAX_BYTES
    assert float(_constant("MAX_SECONDS")) == video.MAX_SECONDS


def test_the_names_are_the_workers() -> None:
    """Stages, kinds and uploads; the page's stages, uploads and intrinsics sources too."""
    assert jobs.STAGES == worker.STAGES
    assert jobs.KINDS == worker.KINDS
    assert jobs.UPLOADS == worker.UPLOADS
    assert tuple(re.findall(r'\["(\w+)", "', _constant("STAGES"))) == worker.STAGES
    keys = r'^\s*"?([\w-]+)"?:'
    assert tuple(re.findall(keys, _constant("UPLOADS"), re.MULTILINE)) == worker.UPLOADS
    sources = tuple(re.findall(keys, _constant("INTRINSICS"), re.MULTILINE))
    assert sources == ("stray-scanner", *lenses.SOURCES)  # the worker names a zip's source


def test_a_failures_reason_is_cut_where_the_server_cuts_it() -> None:
    """The worker never posts a reason longer than the server keeps, so it is never refused."""
    assert worker.REASON_LENGTH == app.REASON_LENGTH


def test_the_heartbeat_is_well_inside_the_silence() -> None:
    """The server fails a job it has not heard from for SILENCE: at least three heartbeats."""
    assert config.SILENCE >= 3 * worker.HEARTBEAT


def test_a_stalled_transfer_ends_before_the_server_gives_up() -> None:
    """A stalled upload or download sends no heartbeat: it fails in SILENCE less a heartbeat."""
    assert worker.UPLOAD_TIMEOUT + worker.HEARTBEAT <= config.SILENCE
    assert worker.REQUEST_TIMEOUT + worker.HEARTBEAT <= config.SILENCE


def test_the_retention_the_pages_state_is_the_servers() -> None:
    """The status page's constant, and the upload page's words, say the server's RETENTION."""
    days = config.RETENTION / 86400
    assert float(_constant("RETENTION_DAYS")) == days
    upload = (VIEWER / "upload.html").read_text()
    stated = re.search(r"kept\s+for\s+(\d+)\s+days", upload)
    assert stated is not None
    assert float(stated.group(1)) == days


def test_the_upload_window_the_page_states_is_the_servers() -> None:
    """The upload page says how long an upload has: its link's life, UPLOAD_WAIT."""
    upload = (VIEWER / "upload.html").read_text()
    stated = re.search(r"uploaded\s+within\s+(\d+)\s+minutes", upload)
    assert stated is not None
    assert float(stated.group(1)) == config.UPLOAD_WAIT / 60


def test_the_workers_claim_is_within_the_servers_bound() -> None:
    """The worker's claim waits no longer than the server holds one."""
    assert worker.WAIT <= app.CLAIM_WAIT
