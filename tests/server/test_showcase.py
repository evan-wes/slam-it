"""The showcase: runs baked by us, listed on the home page and opened in the run viewer by name.

Folder storage stands in for the bucket, as in the rest of the server's tests; S3's links, which
carry a file's content type, are signed offline (:func:`tests.server.conftest.make_s3`).
"""

from __future__ import annotations

import base64
import urllib.parse
from dataclasses import replace
from typing import NoReturn, cast

import pytest
from fastapi.testclient import TestClient

from slam_it_server.app import import_maps, make_app, showcase_prefix
from slam_it_server.config import LINK_SECONDS
from slam_it_server.jobs import Jobs
from slam_it_server.storage import FolderStorage
from tests.server.conftest import CODE, Server, make_s3

NAMES = ("kitti-05", "room")
"""The showcase's runs: the room's bundle is in storage, the other's is not."""

BUNDLE = {
    "bundle.json": b'{"sequence": "room"}',
    "cpp/poses.npy": b"\x93NUMPY",
    "images/chunk_0000.bin": b"\xff\xd8\xff",
}
"""The room's bundle: its description, an array and an image chunk."""

BESIDE = {
    "calibration.html": b"<!doctype html><title>calibration</title>",
    "calibration/rig.png": b"\x89PNG",
    "track.jpg": b"\xff\xd8\xff",
    "calibration/plot.svg": b"<svg/>",
}
"""The files kept beside the room's bundle, for its calibration page."""


@pytest.fixture
def showcase(server: Server) -> Server:
    """Return the server with a showcase of NAMES, the room's files in its storage.

    Its storage reads the test's clock, so that a link can expire.
    """
    store = FolderStorage(
        server.store.root, server.store.public_url, server.store.key, server.clock
    )
    for path, data in {**BUNDLE, **BESIDE}.items():
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


def test_the_home_page_lists_the_showcase_and_the_upload_page_moves(showcase: Server) -> None:
    """Each run linked by its name, and the upload page at /upload, under the same policy.

    The page has no script and no inline style, which the content policy would refuse.
    """
    answer = showcase.client.get("/")
    assert answer.status_code == 200
    assert answer.headers["content-type"].startswith("text/html")
    page = answer.text
    for name in NAMES:
        assert f'<a class="primary" href="/s/{name}/">{name}</a>' in page
    assert '<a class="primary" href="/upload">' in page
    assert "<script" not in page
    assert "style=" not in page
    upload = showcase.client.get("/upload")
    assert upload.status_code == 200
    assert "SLAM It! on your own video" in upload.text
    policy = answer.headers["content-security-policy"]
    assert policy == upload.headers["content-security-policy"]
    assert answer.headers["x-robots-tag"] == "noindex, nofollow"


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


def test_the_files_beside_a_bundle_are_redirected_as_their_kind(showcase: Server) -> None:
    """S3's link asks the bucket to answer with the file's content type; a bundle's file, none.

    Folder storage serves the calibration page as it serves every file, as a download.
    """
    client = TestClient(make_app(showcase.settings, make_s3(), showcase.jobs))
    kinds = {
        "calibration.html": "text/html; charset=utf-8",
        "calibration/rig.png": "image/png",
        "track.jpg": "image/jpeg",
        "calibration/plot.svg": "image/svg+xml",
    }
    for path, kind in kinds.items():
        link = urllib.parse.urlsplit(_redirect(client, f"/s/room/{path}"))
        assert link.path.endswith(f"/showcase/room/{path}")
        assert urllib.parse.parse_qs(link.query)["response-content-type"] == [kind]
    bundle = urllib.parse.urlsplit(_redirect(client, "/s/room/bundle/cpp/poses.npy"))
    assert bundle.path.endswith("/showcase/room/cpp/poses.npy")
    assert "response-content-type" not in urllib.parse.parse_qs(bundle.query)
    stored = showcase.client.get(_redirect(showcase.client, "/s/room/calibration.html"))
    assert stored.content == BESIDE["calibration.html"]
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
        "/s/room/calibration.htm",
        "/s/room/calibration/page.html",
        "/s/room/calibration/deep/x.png",
        "/s/room/other/x.png",
        "/s/room/calibration/X.PNG",
        "/s/room/calibration/.png",
        "/s/room/bundle/%2e%2e/%2e%2e/jobs/j/result/bundle.json",
        "/s/room/bundle/..%2f..%2fjobs/j/import",
        "/s/room/bundle/%2fetc/passwd.json",
        "/s/room/bundle//etc/x.json",
        "/s/%2e%2e/bundle/bundle.json",
        "/s/room%2f..%2fjobs/bundle/bundle.json",
        "/s/room/calibration%2f..%2f..%2fx.png",
        "/s/room/%2e%2e/%2e%2e/jobs/j/import",
        "/s/room/bundle/bundle.json%00.png",
    ],
)
def test_a_run_or_file_outside_the_showcase_answers_404(showcase: Server, path: str) -> None:
    """A name not listed; a file of another name, kind or folder; or a path that climbs.

    The escaped paths reach the routes with their ``..``, slashes and bytes decoded, so it is
    the routes that refuse them, never a redirect.
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
    for path in ("/", "/upload", "/s/room/", "/s/other/", "/s/room/calibration.html"):
        assert client.get(path, follow_redirects=False).status_code in (200, 307, 404)
    assert client.get(_redirect(client, "/s/room/bundle/bundle.json")).status_code == 200


def test_staging_puts_the_showcase_behind_a_code(showcase: Server) -> None:
    """On staging, the home page, the upload page and the showcase's routes need a code."""
    staging = replace(showcase.settings, staging=True)
    client = TestClient(make_app(staging, showcase.store, showcase.jobs))
    code = {"Authorization": "Basic " + base64.b64encode(f"anyone:{CODE}".encode()).decode()}
    for path in ("/", "/upload", "/s/room/", "/s/room/bundle/bundle.json", "/s/room/track.jpg"):
        assert client.get(path, follow_redirects=False).status_code == 401
        assert client.get(path, headers=code, follow_redirects=False).status_code in (200, 307)
