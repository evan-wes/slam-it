"""The showcase: runs baked by us, listed on the home page and opened in the run viewer by name.

Folder storage stands in for the bucket, as in the rest of the server's tests; S3's links, which
carry the calibration page's content type, are signed offline
(:func:`tests.server.conftest.make_s3`).
"""

from __future__ import annotations

import base64
import re
import shutil
import urllib.parse
from dataclasses import replace
from pathlib import Path
from typing import NoReturn, cast

import pytest
from fastapi.testclient import TestClient

from slam_it_server.app import SHOWCASE_MARKER, import_maps, make_app, showcase_prefix
from slam_it_server.config import LINK_SECONDS, SettingsError
from slam_it_server.jobs import Jobs
from slam_it_server.storage import FolderStorage
from tests.server.conftest import CODE, VIEWER, Server, make_s3

REPO = Path(__file__).resolve().parents[2]
"""The repository, whose server image's recipe copies the viewer's pages."""

NAMES = ("kitti-05", "room")
"""The showcase's runs: the room's bundle is in storage, the other's is not."""

BUNDLE = {
    "bundle.json": b'{"sequence": "room"}',
    "cpp/poses.npy": b"\x93NUMPY",
    "images/chunk_0000.bin": b"\xff\xd8\xff",
}
"""The room's bundle: its description, an array and an image chunk."""

CALIBRATION = b"<!doctype html><title>calibration</title>"
"""The room's calibration page, kept beside its bundle."""

UNSERVED = {
    "calibration/rig.png": b"\x89PNG",
    "track.jpg": b"\xff\xd8\xff",
    "plot.svg": b"<svg/>",
}
"""Files beside the room's bundle that are never served: its calibration page embeds its own."""


@pytest.fixture
def showcase(server: Server) -> Server:
    """Return the server with a showcase of NAMES, the room's files in its storage.

    Its storage reads the test's clock, so that a link can expire.
    """
    store = FolderStorage(
        server.store.root, server.store.public_url, server.store.key, server.clock
    )
    for path, data in {**BUNDLE, "calibration.html": CALIBRATION, **UNSERVED}.items():
        file = store.path(f"{showcase_prefix('room')}/{path}")
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(data)
    settings = replace(server.settings, showcase=NAMES)
    client = TestClient(make_app(settings, store, server.jobs))
    return replace(server, client=client, store=store, settings=settings)


def _redirect(client: TestClient, path: str) -> str:
    """Return where a GET of a path redirects to, a 307."""
    answer = client.get(path, follow_redirects=False)
    assert answer.status_code == 307, answer.text
    return str(answer.headers["location"])


def test_the_home_page_lists_only_the_showcase_and_the_upload_page_moves(showcase: Server) -> None:
    """Each run linked by its name, and nothing else; the upload page at /upload, unlinked.

    The page has no script and no inline style, which the content policy would refuse. The upload
    page is not linked from it (the demo shows the viewer alone), but answers at its address.
    """
    answer = showcase.client.get("/")
    assert answer.status_code == 200
    assert answer.headers["content-type"].startswith("text/html")
    page = answer.text
    for name in NAMES:
        assert f'<a class="primary" href="/s/{name}/">{name}</a>' in page
    assert "/upload" not in page
    assert "<script" not in page
    assert "style=" not in page
    upload = showcase.client.get("/upload")
    assert upload.status_code == 200
    assert "SLAM It! on your own video" in upload.text
    policy = answer.headers["content-security-policy"]
    assert policy == upload.headers["content-security-policy"]
    assert answer.headers["x-robots-tag"] == "noindex, nofollow"


def _head(page: str) -> str:
    """Return a page's ``<head>``, its title left out."""
    head = page.split("<head>", 1)[1].split("</head>", 1)[0]
    return re.sub(r"\s*<title>.*?</title>", "", head)


def test_the_showcase_page_is_the_upload_pages_kind_and_holds_its_marker_once() -> None:
    """``viewer/showcase.html``: the upload page's head, no script or inline style, one marker.

    Its title aside, its head is the upload page's. It is read by the server, never served by
    its own name.
    """
    page = (VIEWER / "showcase.html").read_text()
    assert _head(page) == _head((VIEWER / "upload.html").read_text())
    assert "<title>SLAM It!</title>" in page
    assert re.search(r"<script|<style|\sstyle=", page) is None
    assert page.count(SHOWCASE_MARKER) == 1


@pytest.mark.parametrize(
    ("page", "message"),
    [
        (None, "is not there, the showcase's page"),
        ("<main></main>", "needs <!-- runs --> once"),
        ("<!-- runs --><!-- runs -->", "needs <!-- runs --> once"),
    ],
)
def test_a_showcase_page_that_cannot_be_filled_stops_the_server(
    server: Server, tmp_path: Path, page: str | None, message: str
) -> None:
    """With a showcase, no page, or a marker missing or repeated: refused before anything is made.

    Without a showcase, the page is not read.
    """
    viewer = tmp_path / "viewer"
    viewer.mkdir()
    shutil.copy(VIEWER / "index.html", viewer / "index.html")
    if page is not None:
        (viewer / "showcase.html").write_text(page)
    database = tmp_path / "never.sqlite3"
    settings = replace(server.settings, viewer=viewer, database=database, showcase=NAMES)
    with pytest.raises(SettingsError, match=rf"SLAM_IT_VIEWER: .*{re.escape(message)}"):
        make_app(settings, server.store)
    assert not database.exists()
    make_app(replace(settings, showcase=()), server.store)


def test_the_servers_image_holds_the_showcase_page() -> None:
    """The image copies the viewer's files one by one: the showcase's page among them."""
    recipe = (REPO / "docker" / "server.Dockerfile").read_text()
    copied = re.search(r"^COPY (viewer/.*?) /opt/viewer/$", recipe, re.MULTILINE | re.DOTALL)
    assert copied is not None
    assert "viewer/showcase.html" in copied.group(1).split()


def test_without_a_showcase_the_home_page_is_the_upload_page(server: Server) -> None:
    """As before the showcase: the upload page at the root, and at /upload too; no run found."""
    for path in ("/", "/upload"):
        answer = server.client.get(path)
        assert answer.status_code == 200
        assert "SLAM It! on your own video" in answer.text
    for path in ("/s/room/", "/s/room/bundle/bundle.json", "/s/room/calibration.html"):
        assert server.client.get(path, follow_redirects=False).status_code == 404


def test_a_showcase_run_opens_in_the_run_viewer(showcase: Server) -> None:
    """Its address, with or without its slash, opens the viewer on its bundle's path.

    The viewer page is the one the status page links to, under the same content policy: its
    import map allowed by its hash, connections to storage.
    """
    for path in ("/s/room/", "/s/room"):
        assert _redirect(showcase.client, path) == "/index.html?bundle=%2Fs%2Froom%2Fbundle%2F"
    page = showcase.client.get("/s/room/")
    assert page.status_code == 200
    assert "SLAM It! run viewer" in page.text
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(str(page.url)).query)
    assert query["bundle"] == ["/s/room/bundle/"]
    policy = page.headers["content-security-policy"]
    maps = import_maps(showcase.settings.viewer)
    assert maps
    assert all(source in policy for source in maps)
    assert f"connect-src 'self' {showcase.store.origin}" in policy


def test_a_showcase_runs_bundle_is_redirected_by_a_link_made_each_time(showcase: Server) -> None:
    """Each of its files redirected to storage's link for it, by a link made for the request.

    So a visitor's page still loads a file after the links it was given first have expired. A
    run listed whose file storage does not hold: redirected, and storage answers 404.
    """
    for path, data in BUNDLE.items():
        link = _redirect(showcase.client, f"/s/room/bundle/{path}")
        assert urllib.parse.urlsplit(link).path == f"/storage/showcase/room/{path}"
        assert showcase.client.get(link).content == data
    first = _redirect(showcase.client, "/s/room/bundle/bundle.json")
    showcase.clock.now += LINK_SECONDS + 1
    assert showcase.client.get(first).status_code == 403  # that link has expired
    again = _redirect(showcase.client, "/s/room/bundle/bundle.json")
    assert showcase.client.get(again).content == BUNDLE["bundle.json"]
    missing = _redirect(showcase.client, "/s/kitti-05/bundle/bundle.json")
    assert showcase.client.get(missing).status_code == 404


def test_the_calibration_page_is_redirected_to_be_answered_as_a_page(showcase: Server) -> None:
    """S3's link asks the bucket to answer it as HTML; a bundle's file, as stored.

    Folder storage serves the calibration page as it serves every file, as a download.
    """
    client = TestClient(make_app(showcase.settings, make_s3(), showcase.jobs))
    link = urllib.parse.urlsplit(_redirect(client, "/s/room/calibration.html"))
    assert link.path.endswith("/showcase/room/calibration.html")
    query = urllib.parse.parse_qs(link.query)
    assert query["response-content-type"] == ["text/html; charset=utf-8"]
    bundle = urllib.parse.urlsplit(_redirect(client, "/s/room/bundle/cpp/poses.npy"))
    assert bundle.path.endswith("/showcase/room/cpp/poses.npy")
    assert "response-content-type" not in urllib.parse.parse_qs(bundle.query)
    stored = showcase.client.get(_redirect(showcase.client, "/s/room/calibration.html"))
    assert stored.content == CALIBRATION
    assert stored.headers["content-disposition"] == "attachment"


@pytest.mark.parametrize(
    "path",
    [
        "/s/other/",
        "/s/other",
        "/s/other/bundle/bundle.json",
        "/s/other/calibration.html",
        "/s/Room/",
        "/s/room/bundle/index.html",
        "/s/room/bundle/calibration.html",
        "/s/room/bundle/cpp/deep/x.npy",
        "/s/room/bundle/",
        "/s/room/bundle.json",
        "/s/room/index.html",
        "/s/room/notes.txt",
        "/s/room/track.jpg",
        "/s/room/plot.svg",
        "/s/room/calibration/rig.png",
        "/s/room/calibration.htm",
        "/s/room/Calibration.html",
        "/s/room/calibration/calibration.html",
        "/s/room/calibration.html/x",
        "/showcase.html",
        "/s/room/bundle/%2e%2e/%2e%2e/jobs/j/result/bundle.json",
        "/s/room/bundle/..%2f..%2fjobs/j/import",
        "/s/room/bundle/%2fetc/passwd.json",
        "/s/room/bundle//etc/x.json",
        "/s/%2e%2e/bundle/bundle.json",
        "/s/room%2f..%2fjobs/bundle/bundle.json",
        "/s/room/calibration.html%2f..%2f..%2fjobs/j/import",
        "/s/room/%2e%2e/%2e%2e/jobs/j/import",
        "/s/room/bundle/bundle.json%00.png",
    ],
)
def test_a_run_or_file_outside_the_showcase_answers_404(showcase: Server, path: str) -> None:
    """A name not listed; a file of another name, kind or folder; or a path that climbs.

    Images kept beside the bundle are among them, and the showcase's page by its own name. The
    escaped paths reach the server with their ``..``, slashes and bytes decoded, and are refused
    there, never redirected.
    """
    answer = showcase.client.get(path, follow_redirects=False)
    assert answer.status_code == 404
    assert "location" not in answer.headers


class _NoJobs:
    """A job list that fails a test on any use."""

    def __getattr__(self, name: str) -> NoReturn:
        """Fail: the showcase has no use for the job list."""
        raise AssertionError(f"the showcase used the job list: {name}")


def test_the_showcase_never_reads_the_job_list(showcase: Server) -> None:
    """Its pages and links come from the settings and storage alone, so they hold no job token."""
    client = TestClient(make_app(showcase.settings, showcase.store, cast(Jobs, _NoJobs())))
    answers = {
        "/": 200,
        "/upload": 200,
        "/s/room/": 307,
        "/s/other/": 404,
        "/s/room/calibration.html": 307,
    }
    for path, status in answers.items():
        assert (path, client.get(path, follow_redirects=False).status_code) == (path, status)
    assert client.get(_redirect(client, "/s/room/bundle/bundle.json")).status_code == 200


def test_staging_puts_the_showcase_behind_a_code(showcase: Server) -> None:
    """On staging, the home page, the upload page and the showcase's routes need a code."""
    staging = replace(showcase.settings, staging=True)
    client = TestClient(make_app(staging, showcase.store, showcase.jobs))
    code = {"Authorization": "Basic " + base64.b64encode(f"anyone:{CODE}".encode()).decode()}
    answers = {
        "/": 200,
        "/upload": 200,
        "/s/room/": 307,
        "/s/room/bundle/bundle.json": 307,
        "/s/room/calibration.html": 307,
    }
    for path, status in answers.items():
        assert (path, client.get(path, follow_redirects=False).status_code) == (path, 401)
        answer = client.get(path, headers=code, follow_redirects=False)
        assert (path, answer.status_code) == (path, status)
