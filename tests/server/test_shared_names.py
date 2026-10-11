"""The caps and names that the import, the worker, the server and the pages each hold, held equal.

The server never imports slam_it_all, and the pages are JavaScript, so each keeps its own copy.
"""

from __future__ import annotations

import re

from slam_it_all.eval import lenses, video
from slam_it_all.orchestration import worker
from slam_it_all_server import config, jobs
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


def test_the_heartbeat_is_well_inside_the_silence() -> None:
    """The server fails a job it has not heard from for SILENCE: at least three heartbeats."""
    assert config.SILENCE >= 3 * worker.HEARTBEAT
