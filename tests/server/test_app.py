"""The server's routes, through FastAPI's test client: a job's life, the guards, and expiry."""

from __future__ import annotations

import base64
import hashlib
import json
import logging
import shutil
import urllib.parse
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient

from slam_it_server import app as server_app
from slam_it_server import jobs
from slam_it_server.config import (
    KEEP_FREE,
    LINK_SECONDS,
    MAX_BYTES,
    QUEUE,
    RETENTION,
    SILENCE,
    UPLOAD_GRACE,
    UPLOAD_WAIT,
    SettingsError,
)
from slam_it_server.storage import FolderStorage
from tests.server.conftest import CODE, CODES, TOKEN, VIEWER, WORKER, Clock, Server

UPLOAD = b"not really a video, but storage never looks"
"""An upload's bytes."""


def _make(
    server: Server, upload: str = "video", size: int = len(UPLOAD), code: str = CODE
) -> dict[str, str]:
    """Make a job; return the server's answer."""
    answer = server.client.post("/api/jobs", json={"code": code, "upload": upload, "bytes": size})
    assert answer.status_code == 201, answer.text
    return dict(answer.json())


def _queued(server: Server) -> dict[str, str]:
    """Make a job, upload it and confirm it; return the server's answer to its making."""
    job = _make(server)
    assert server.client.put(job["upload_url"], content=UPLOAD).status_code == 200
    answer = server.client.post(f"/api/jobs/{job['token']}/uploaded")
    assert answer.json() == {"state": "queued"}
    return job


def _claim(server: Server) -> dict[str, str]:
    """Claim a job as the worker, without waiting."""
    answer = server.client.post(
        "/api/worker/claim", json={"kinds": ["vo"], "wait": 0}, headers=WORKER
    )
    assert answer.status_code == 200, answer.text
    return dict(answer.json())


def _report(server: Server, job: str, action: str, body: dict[str, Any]) -> Any:  # noqa: ANN401
    """POST a worker's report; return the response."""
    return server.client.post(f"/api/worker/jobs/{job}/{action}", json=body, headers=WORKER)


def _status(server: Server, token: str) -> dict[str, Any]:
    """Return a job's status."""
    answer = server.client.get(f"/api/jobs/{token}")
    assert answer.status_code == 200, answer.text
    return dict(answer.json())


def _run(server: Server, files: dict[str, bytes]) -> tuple[dict[str, str], dict[str, str]]:
    """Queue, claim and finish a job, its result the files; return the owner's and worker's."""
    job = _queued(server)
    claimed = _claim(server)
    for stage in ("importing", "running", "baking", "uploading"):
        assert _report(server, claimed["job"], "progress", {"stage": stage}).status_code == 200
    listed = [{"path": path, "bytes": len(data)} for path, data in files.items()]
    urls = _report(server, claimed["job"], "files", {"files": listed}).json()["urls"]
    for path, data in files.items():
        assert server.client.put(urls[path], content=data).status_code == 200
    answer = _report(server, claimed["job"], "done", {"summary": {"frames": 2}})
    assert answer.status_code == 200, answer.text
    return job, claimed


def test_an_owner_makes_a_job_and_uploads_it(server: Server) -> None:
    """Made waiting, its upload sent to the signed link, then confirmed: queued, none ahead."""
    job = _make(server)
    assert job["link"] == f"/r/{job['token']}"
    assert _status(server, job["token"])["state"] == "waiting"
    assert server.client.put(job["upload_url"], content=UPLOAD).status_code == 200
    assert server.client.post(f"/api/jobs/{job['token']}/uploaded").json() == {"state": "queued"}
    status = _status(server, job["token"])
    assert (status["state"], status["ahead"], status["upload"]) == ("queued", 0, "video")
    assert list(server.store.root.glob("jobs/*/upload")) == []  # moved out of the link's reach
    (stored,) = server.store.root.glob("jobs/*/import")
    assert stored.read_bytes() == UPLOAD


@pytest.mark.parametrize(
    ("body", "code"),
    [
        ({"code": "not-a-code-we-know", "upload": "video", "bytes": 10}, 403),
        ({"code": CODE, "upload": "video", "bytes": MAX_BYTES + 1}, 413),
        ({"code": CODE, "upload": "tarball", "bytes": 10}, 422),
        ({"code": CODE, "upload": "video", "bytes": 0}, 422),
        ({"code": CODE, "upload": "video", "bytes": True}, 422),
        ({"code": CODE, "upload": "video", "bytes": "10"}, 422),
        ({"code": CODE, "upload": "video"}, 422),
    ],
)
def test_a_job_needs_a_code_we_know_and_a_size_within_the_cap(
    server: Server, body: dict[str, Any], code: int
) -> None:
    """An unknown code, a size over the cap, an unknown upload or a size not a count: refused."""
    assert server.client.post("/api/jobs", json=body).status_code == code
    assert server.jobs.by_token("x") is None


def test_an_upload_link_takes_its_size_only(server: Server) -> None:
    """Fewer or more bytes, a changed signature, or the link used to GET: all refused."""
    job = _make(server)
    link = job["upload_url"]
    assert server.client.put(link, content=UPLOAD[:-1]).status_code == 400
    assert server.client.put(link, content=UPLOAD + b"!").status_code == 400
    assert (
        server.client.put(link.replace("signature=", "signature=0"), content=UPLOAD).status_code
        == 403
    )
    assert server.client.put(link.replace("size=", "size=1"), content=UPLOAD).status_code == 403
    assert server.client.get(link).status_code == 403
    assert list(server.store.root.glob("jobs/*/upload")) == []  # nothing kept, not even a part
    assert server.client.post(f"/api/jobs/{job['token']}/uploaded").status_code == 400
    assert server.client.put(link, content=UPLOAD).status_code == 200


def test_a_job_is_queued_once_and_only_whole(server: Server) -> None:
    """Confirmed before its upload arrives: refused; once queued, a second confirmation too."""
    job = _make(server)
    token = job["token"]
    assert server.client.post(f"/api/jobs/{token}/uploaded").status_code == 400
    server.client.put(job["upload_url"], content=UPLOAD)
    assert server.client.post(f"/api/jobs/{token}/uploaded").status_code == 200
    assert server.client.post(f"/api/jobs/{token}/uploaded").status_code == 409
    assert server.client.post("/api/jobs/no-such-token/uploaded").status_code == 404
    assert server.client.get("/api/jobs/no-such-token").status_code == 404


def test_an_upload_confirmed_as_its_job_expires_is_deleted(
    server: Server, monkeypatch: pytest.MonkeyPatch
) -> None:
    """S3 moves an upload by copying it, then deleting it: the job can expire in between.

    The sweep that ends it finds no copy yet, and clears nothing again. The confirmation is
    refused, and the copy, which nothing would import, is deleted at once.
    """
    job = _make(server)
    server.client.put(job["upload_url"], content=UPLOAD)

    def slow(key: str, to: str) -> None:
        held = server.store.path(key).read_bytes()  # S3's copy begins
        server.clock.now += UPLOAD_WAIT  # the job expires; a sweep clears its files, no copy yet
        assert server_app.sweep(server.jobs, server.store) == 0
        server.store.path(to).write_bytes(held)  # the copy lands
        server.store.delete(key)

    monkeypatch.setattr(server.store, "move", slow)
    answer = server.client.post(f"/api/jobs/{job['token']}/uploaded")
    assert answer.status_code == 409
    assert _status(server, job["token"])["state"] == "expired"
    assert list(server.store.root.glob("jobs/*/import")) == []


def _keeping(server: Server) -> Server:
    """Return the same server, its list and storage, keeping uploads as the demo does."""
    settings = replace(server.settings, keep_uploads=True)
    app = server_app.make_app(settings, server.store, server.jobs)
    return replace(server, client=TestClient(app), settings=settings)


def test_the_demo_refuses_a_job_when_its_disk_is_nearly_full(
    server: Server, monkeypatch: pytest.MonkeyPatch
) -> None:
    """With uploads kept, a new job needs KEEP_FREE free under folder storage; else, none."""
    keeping = _keeping(server)
    disk = SimpleNamespace(free=KEEP_FREE - 1)
    monkeypatch.setattr(shutil, "disk_usage", lambda path: disk)
    answer = keeping.client.post("/api/jobs", json={"code": CODE, "upload": "video", "bytes": 1})
    assert (answer.status_code, answer.json()["detail"]) == (
        503,
        "the demo's disk is nearly full: wait for older runs to go",
    )
    _make(server)  # a deployment keeps no upload: its disk is not its to watch
    disk.free = KEEP_FREE
    _make(keeping)


def test_a_worker_runs_a_job_through_the_api(server: Server) -> None:
    """Claimed, downloaded, reported stage by stage, its result uploaded, done; then viewable."""
    job = _queued(server)
    claimed = _claim(server)
    assert (claimed["kind"], claimed["upload"]) == ("vo", "video")
    assert server.client.get(claimed["download"]).content == UPLOAD
    assert _status(server, job["token"])["state"] == "downloading"
    _report(server, claimed["job"], "progress", {"stage": "importing"})
    assert len(list(server.store.root.glob("jobs/*/import"))) == 1  # still importing
    _report(server, claimed["job"], "progress", {"stage": "running"})
    assert list(server.store.root.glob("jobs/*/import")) == []  # the import has ended
    files = {"bundle.json": b"{}", "cpp/poses.npy": b"\x93NUMPY"}
    listed = [{"path": path, "bytes": len(data)} for path, data in files.items()]
    urls = _report(server, claimed["job"], "files", {"files": listed}).json()["urls"]
    for path, data in files.items():
        server.client.put(urls[path], content=data)
    bundle = f"/r/{job['token']}/bundle/"
    assert server.client.get(f"{bundle}bundle.json").status_code == 404  # not until done
    assert _report(server, claimed["job"], "done", {"summary": {"frames": 2}}).status_code == 200
    status = _status(server, job["token"])
    assert (status["state"], status["summary"]) == ("done", {"frames": 2})
    assert status["bundle"] == bundle
    assert status["deleted"] == pytest.approx(server.clock.now + RETENTION)
    redirect = server.client.get(f"{status['bundle']}cpp/poses.npy", follow_redirects=False)
    assert redirect.status_code == 307
    stored = server.client.get(redirect.headers["location"])
    assert stored.content == b"\x93NUMPY"
    assert stored.headers["content-type"] == "application/octet-stream"
    assert stored.headers["content-disposition"] == "attachment"
    assert server.client.get(f"{status['bundle']}cpp/other.npy").status_code == 404
    assert server.client.get(f"/r/{claimed['job']}/bundle/bundle.json").status_code == 404


def test_a_job_is_done_only_when_its_result_has_arrived_whole(server: Server) -> None:
    """A listed file missing, or short: done refused, the job still the worker's."""
    _queued(server)
    claimed = _claim(server)
    listed = [{"path": "a.npy", "bytes": 3}, {"path": "b.npy", "bytes": 3}]
    urls = _report(server, claimed["job"], "files", {"files": listed}).json()["urls"]
    server.client.put(urls["a.npy"], content=b"abc")
    answer = _report(server, claimed["job"], "done", {"summary": {}})
    assert (answer.status_code, answer.json()["detail"]) == (
        400,
        "files: b.npy has not arrived whole",
    )
    assert _report(server, claimed["job"], "progress", {"stage": "uploading"}).status_code == 200


def test_a_failed_job_keeps_nothing_but_its_reason(server: Server) -> None:
    """Failed: its upload and its partial result deleted, the reason shown to its owner."""
    job = _queued(server)
    claimed = _claim(server)
    urls = _report(
        server, claimed["job"], "files", {"files": [{"path": "a.npy", "bytes": 1}]}
    ).json()["urls"]
    server.client.put(urls["a.npy"], content=b"a")
    reason = "x" * 600
    assert _report(server, claimed["job"], "failed", {"reason": reason}).status_code == 200
    assert list(server.store.root.rglob("*.npy")) == []
    assert list(server.store.root.glob("jobs/*/import")) == []
    status = _status(server, job["token"])
    assert (status["state"], status["reason"]) == ("failed", reason[:500])


def test_the_workers_api_needs_its_token(server: Server) -> None:
    """No token, a wrong one, or another scheme: 401; with it and no job, 204."""
    claim = {"kinds": ["vo"], "wait": 0}
    for headers in ({}, {"Authorization": "Bearer wrong"}, {"Authorization": f"Basic {TOKEN}"}):
        assert (
            server.client.post("/api/worker/claim", json=claim, headers=headers).status_code == 401
        )
    assert server.client.post("/api/worker/claim", json=claim, headers=WORKER).status_code == 204
    assert _report(server, "x", "progress", {"stage": "running"}).status_code == 409


def test_a_claim_waits_for_a_job_up_to_its_wait(
    server: Server, monkeypatch: pytest.MonkeyPatch
) -> None:
    """With nothing queued, the claim is held and answered 204 at its wait."""
    monkeypatch.setattr(server_app, "CLAIM_POLL", 0.01)
    claim = {"kinds": ["vo"], "wait": 0.05}
    assert server.client.post("/api/worker/claim", json=claim, headers=WORKER).status_code == 204
    claim["wait"] = 61
    assert server.client.post("/api/worker/claim", json=claim, headers=WORKER).status_code == 422


def test_a_job_no_longer_the_workers_answers_409(server: Server) -> None:
    """Reports on a job waiting, or already ended, are refused with 409, which ends it there."""
    job = _make(server)
    waiting = server.jobs.by_token(job["token"])
    assert waiting is not None
    assert _report(server, waiting.id, "progress", {"stage": "importing"}).status_code == 409
    assert _report(server, waiting.id, "done", {"summary": {}}).status_code == 409
    _queued(server)
    claimed = _claim(server)
    _report(server, claimed["job"], "failed", {"reason": "no"})
    assert _report(server, claimed["job"], "failed", {"reason": "no"}).status_code == 409
    assert _report(server, claimed["job"], "files", {"files": []}).status_code == 409


@pytest.mark.parametrize(
    ("files", "code"),
    [
        ([{"path": "../uploads/x", "bytes": 1}], 400),
        ([{"path": "/etc/x", "bytes": 1}], 400),
        ([{"path": "a", "bytes": 1}, {"path": "a", "bytes": 2}], 400),
        ([{"path": "a", "bytes": server_app.RESULT_BYTES + 1}], 413),
        ([{"path": f"f{k}", "bytes": 1} for k in range(server_app.RESULT_FILES + 1)], 422),
        ([{"path": "a", "bytes": -1}], 422),
        ([{"path": "index.html", "bytes": 1}], 400),
        ([{"path": "cpp/deep/x.npy", "bytes": 1}], 400),
        ([{"path": "a.npy\n", "bytes": 1}], 400),
    ],
)
def test_a_results_files_are_checked(
    server: Server, files: list[dict[str, Any]], code: int
) -> None:
    """Not a bundle's file (climbing, absolute, a page, too deep), twice, too many: refused."""
    _queued(server)
    claimed = _claim(server)
    assert _report(server, claimed["job"], "files", {"files": files}).status_code == code


def test_a_stage_out_of_order_is_refused(server: Server) -> None:
    """A stage before the job's, or one not known: 400."""
    _queued(server)
    claimed = _claim(server)
    _report(server, claimed["job"], "progress", {"stage": "running"})
    assert _report(server, claimed["job"], "progress", {"stage": "importing"}).status_code == 400
    assert _report(server, claimed["job"], "progress", {"stage": "resting"}).status_code == 400


def test_the_queue_is_capped(server: Server) -> None:
    """Past QUEUE open jobs, the owner is asked to try later."""
    for n in range(QUEUE):
        _make(server, code=CODES[n % len(CODES)])
    answer = server.client.post("/api/jobs", json={"code": CODE, "upload": "video", "bytes": 1})
    assert (answer.status_code, answer.json()["detail"]) == (
        503,
        "the queue is full: please try again later",
    )


def test_the_sweep_deletes_what_has_expired(server: Server) -> None:
    """An upload never confirmed goes at UPLOAD_WAIT; a result, then its job, RETENTION after."""
    waiting = _make(server)
    server.client.put(waiting["upload_url"], content=UPLOAD)
    done, claimed = _run(server, {"bundle.json": b"{}"})
    server.clock.now += UPLOAD_WAIT
    assert server_app.sweep(server.jobs, server.store) == 0
    assert _status(server, waiting["token"])["reason"] == "the upload did not arrive in time"
    assert list(server.store.root.glob("jobs/*/upload")) == []
    assert server.store.path(f"{server_app.result_prefix(claimed['job'])}/bundle.json").is_file()
    server.clock.now += RETENTION
    assert server_app.sweep(server.jobs, server.store) == 0
    assert list((server.store.root / "jobs").iterdir()) == []  # each job's folder, whole
    assert server.client.get(f"/api/jobs/{done['token']}").status_code == 404
    assert server.client.get(f"/r/{done['token']}/bundle/bundle.json").status_code == 404


def test_the_demo_keeps_an_upload_until_its_job_is_deleted(server: Server) -> None:
    """With keep_uploads, a confirmed upload stays through its import and the job's end.

    Done or failed, it stays when the job's files are cleared (a failed job's result does not),
    and goes with the job's folder, whole, RETENTION after the job ended. The upload page asks
    which: a deployment's server says uploads are not kept.
    """
    keeping = _keeping(server)
    assert keeping.client.get("/api/kept").json() == {"uploads": True}
    assert server.client.get("/api/kept").json() == {"uploads": False}
    _, done = _run(keeping, {"bundle.json": b"{}"})
    _queued(keeping)
    failed = _claim(keeping)
    for stage in ("importing", "running", "baking", "uploading"):
        assert _report(keeping, failed["job"], "progress", {"stage": stage}).status_code == 200
    listed = {"files": [{"path": "bundle.json", "bytes": 2}]}
    url = _report(keeping, failed["job"], "files", listed).json()["urls"]["bundle.json"]
    assert keeping.client.put(url, content=b"{}").status_code == 200
    assert _report(keeping, failed["job"], "failed", {"reason": "lost"}).status_code == 200
    server.clock.now += LINK_SECONDS + UPLOAD_GRACE  # every file cleared, every link closed
    assert server_app.sweep(server.jobs, server.store, keep_uploads=True) == 0
    for job in (done, failed):
        assert server.store.path(server_app.import_key(job["job"])).read_bytes() == UPLOAD
    assert server.store.path(f"{server_app.result_prefix(done['job'])}/bundle.json").is_file()
    assert not server.store.path(server_app.result_prefix(failed["job"])).exists()
    server.clock.now += RETENTION
    assert server_app.sweep(server.jobs, server.store, keep_uploads=True) == 0
    assert list((server.store.root / "jobs").iterdir()) == []


def _job_id(job: dict[str, str]) -> str:
    """Return a job's id, from its upload link (``/storage/jobs/<id>/upload?...``)."""
    return job["upload_url"].split("/storage/jobs/", 1)[1].split("/", 1)[0]


def test_an_upload_that_lands_after_its_link_expired_is_deleted(server: Server) -> None:
    """An upload begun in time but arriving after its link expired is deleted, its grace over.

    S3 checks a link when a request starts, so such a file can land after its job has ended
    and its files were cleared: the sweep deletes it once every link the job signed has
    expired (LINK_SECONDS after it ended) and UPLOAD_GRACE has passed.
    """
    job = _make(server)
    server.clock.now += UPLOAD_WAIT
    assert server_app.sweep(server.jobs, server.store) == 0
    assert _status(server, job["token"])["state"] == "expired"
    late = server.store.path(server_app.upload_key(_job_id(job)))
    late.parent.mkdir(parents=True, exist_ok=True)
    late.write_bytes(UPLOAD)  # as a PUT the bucket let start in time would, now
    server.clock.now += LINK_SECONDS + UPLOAD_GRACE - 1
    assert server_app.sweep(server.jobs, server.store) == 0
    assert late.is_file()
    server.clock.now += 1
    assert server_app.sweep(server.jobs, server.store) == 0
    assert not late.exists()


def _uploading(server: Server) -> tuple[dict[str, str], dict[str, str]]:
    """Queue and claim a job, and get a link for one result file; return the claim and links."""
    _queued(server)
    claimed = _claim(server)
    listed = [{"path": "a.npy", "bytes": 1}]
    urls = _report(server, claimed["job"], "files", {"files": listed}).json()["urls"]
    return claimed, dict(urls)


def test_a_result_written_after_its_job_failed_is_deleted(server: Server) -> None:
    """A result link still writing after its job failed: deleted again, its grace over.

    The worker's links last LINK_SECONDS; a job failed for silence while its worker uploads
    has its result cleared, and what the links write after is cleared again later. A done
    job's result is kept.
    """
    _, urls = _uploading(server)
    server.clock.now += SILENCE
    assert server_app.sweep(server.jobs, server.store) == 0  # failed, its result cleared
    assert server.client.put(urls["a.npy"], content=b"a").status_code == 200  # the link holds
    assert len(list(server.store.root.rglob("*.npy"))) == 1
    server.clock.now += LINK_SECONDS + UPLOAD_GRACE
    assert server_app.sweep(server.jobs, server.store) == 0
    assert list(server.store.root.rglob("*.npy")) == []
    _, kept = _run(server, {"bundle.json": b"{}"})
    server.clock.now += LINK_SECONDS + UPLOAD_GRACE
    assert server_app.sweep(server.jobs, server.store) == 0
    assert server.store.path(f"{server_app.result_prefix(kept['job'])}/bundle.json").is_file()


def test_a_put_that_outlives_its_link_is_refused_and_not_kept(server: Server) -> None:
    """A PUT whose link held when it started but had expired by its end: refused, none kept.

    The clock moves past the link's expiry while the body is read, between its two halves.
    """
    clock = Clock()
    store = FolderStorage(server.store.root, server.store.public_url, server.store.key, clock)
    client = TestClient(server_app.make_app(server.settings, store, server.jobs))
    link = store.upload_link("uploads/late", len(UPLOAD), 10)
    half = len(UPLOAD) // 2

    def body() -> Iterator[bytes]:
        yield UPLOAD[:half]
        clock.now += 11.0  # the link has expired; the rest arrives after
        yield UPLOAD[half:]

    answer = client.put(link, content=body(), headers={"Content-Length": str(len(UPLOAD))})
    assert (answer.status_code, answer.json()["detail"]) == (
        403,
        "the link expired before the upload arrived whole",
    )
    assert list(store.root.rglob("*late*")) == []


@pytest.mark.parametrize(
    ("error", "status"), [(FileNotFoundError("gone"), 409), (OSError("storage is away"), 503)]
)
def test_a_confirmation_whose_move_fails_says_so(
    error: Exception, status: int, server: Server,
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture,
) -> None:  # fmt: skip
    """Moved a moment before, 409; storage failing, 503 and a line by the job's id.

    Either way the job is still waiting.
    """
    job = _make(server)
    assert server.client.put(job["upload_url"], content=UPLOAD).status_code == 200

    def fail(key: str, to: str) -> None:
        raise error

    monkeypatch.setattr(server.store, "move", fail)
    with caplog.at_level(logging.ERROR, logger="slam_it_server"):
        answer = server.client.post(f"/api/jobs/{job['token']}/uploaded")
    assert answer.status_code == status
    logged = f"job {_job_id(job)}: the upload could not be moved" in caplog.text
    assert logged == (status == 503)
    assert _status(server, job["token"])["state"] == "waiting"


class _Flaky(FolderStorage):
    """Folder storage whose next result deletion fails."""

    failing = True

    def delete_prefix(self, prefix: str) -> None:
        """Fail once, then delete."""
        if self.failing:
            self.failing = False
            raise OSError("storage is away")
        super().delete_prefix(prefix)


def test_a_deletion_that_fails_is_tried_again(
    server: Server, caplog: pytest.LogCaptureFixture
) -> None:
    """The job stays listed until its files are gone: the next sweep deletes both; logged."""
    flaky = _Flaky(server.store.root, server.store.public_url, server.store.key)
    done, claimed = _run(server, {"bundle.json": b"{}"})
    server.clock.now += RETENTION
    with caplog.at_level(logging.ERROR, logger="slam_it_server"):
        assert server_app.sweep(server.jobs, flaky) == 1
    assert f"job {claimed['job']}: a sweep's deletion failed" in caplog.text
    assert server.store.path(f"{server_app.result_prefix(claimed['job'])}/bundle.json").is_file()
    assert server.client.get(f"/api/jobs/{done['token']}").status_code == 200  # still listed
    assert server_app.sweep(server.jobs, flaky) == 0
    assert list(server.store.root.rglob("*.json")) == []
    assert server.client.get(f"/api/jobs/{done['token']}").status_code == 404


def test_a_replayed_upload_link_never_reaches_the_worker_and_is_deleted(server: Server) -> None:
    """A second PUT through the owner's link is never what the worker reads, and is deleted.

    Confirmed, the upload is moved out of the link's reach; what the link writes after is
    deleted when the job ends (here, its worker gone silent).
    """
    job = _queued(server)
    assert server.client.put(job["upload_url"], content=b"x" * len(UPLOAD)).status_code == 200
    claimed = _claim(server)
    assert server.client.get(claimed["download"]).content == UPLOAD
    assert len(list(server.store.root.glob("jobs/*/upload"))) == 1  # the replay, for now
    server.clock.now += UPLOAD_WAIT
    assert server_app.sweep(server.jobs, server.store) == 0
    assert list(server.store.root.glob("jobs/*/upload")) == []


def test_one_code_opens_at_most_two_jobs(server: Server) -> None:
    """A third job by the same code waits for one to end; another code's goes ahead."""
    for _ in range(jobs.CODE_JOBS):
        _make(server)
    answer = server.client.post("/api/jobs", json={"code": CODE, "upload": "video", "bytes": 1})
    assert answer.status_code == 429
    assert "has 2 jobs open" in answer.json()["detail"]
    _make(server, code=CODES[1])


def test_the_log_names_jobs_by_id_never_by_token(
    server: Server, caplog: pytest.LogCaptureFixture
) -> None:
    """Made and queued: logged with the job's id and its code's hash, never its token."""
    with caplog.at_level(logging.INFO, logger="slam_it_server"):
        job = _queued(server)
    made = server.jobs.by_token(job["token"])
    assert made is not None
    assert f"job {made.id}: made for video" in caplog.text
    assert f"job {made.id}: queued" in caplog.text
    assert job["token"] not in caplog.text
    assert CODE not in caplog.text


@pytest.mark.parametrize("path", ["/", "/r/a-token", "/api/jobs/x", "/robots.txt", "/nothing"])
def test_every_answer_is_kept_from_search_and_sends_no_referrer(server: Server, path: str) -> None:
    """Pages, the API, robots.txt and a 404 alike."""
    answer = server.client.get(path)
    assert answer.headers["x-robots-tag"] == "noindex, nofollow"
    assert answer.headers["referrer-policy"] == "no-referrer"
    assert answer.headers["x-content-type-options"] == "nosniff"
    if path.startswith("/api/"):
        assert answer.headers["cache-control"] == "no-store"


def test_robots_are_asked_to_stay_away(server: Server) -> None:
    """robots.txt disallows everything."""
    assert server.client.get("/robots.txt").text == "User-agent: *\nDisallow: /\n"


@pytest.mark.parametrize(
    ("path", "kind"),
    [
        ("/", "SLAM It! on your own video"),
        ("/r/some-token", "SLAM It! run"),
        ("/index.html", "SLAM It! run viewer"),
        ("/upload.html", "SLAM It! on your own video"),
        ("/app.css", "--accent"),
        ("/src/jobs.js", "export function describe"),
        ("/src/main.js", "loadBundle"),
    ],
)
def test_the_pages_are_served(server: Server, path: str, kind: str) -> None:
    """The upload page at the root, the status page at a job's link, the viewer and its files."""
    answer = server.client.get(path)
    assert answer.status_code == 200
    assert kind in answer.text
    if path.endswith(".js"):
        assert answer.headers["content-type"].startswith("text/javascript")


@pytest.mark.parametrize(
    "path",
    [
        "/package.json",
        "/tsconfig.json",
        "/logic.d.ts",
        "/tests/jobs.test.js",
        "/node_modules/three/package.json",
        "/src/../package.json",
        "/src/%2e%2e/package.json",
        "/src/nothing.js",
        "/src/sub/x.js",
        "/storage/uploads/x",
    ],
)
def test_nothing_else_of_the_viewers_folder_is_served(server: Server, path: str) -> None:
    """Its tools, tests and modules folder, and any path out of it: not found, or refused."""
    assert server.client.get(path).status_code in (403, 404)


def test_a_body_too_large_or_of_no_stated_length_is_refused(server: Server) -> None:
    """Over the body cap: 413; sent in chunks with no length: 411."""
    padding = "x" * server_app.BODY_BYTES
    body = {"code": padding, "upload": "video", "bytes": 1}
    assert server.client.post("/api/jobs", json=body).status_code == 413

    def chunks() -> Iterator[bytes]:
        yield b'{"code": "'
        yield b'x", "upload": "video", "bytes": 1}'

    assert server.client.post("/api/jobs", content=chunks()).status_code == 411


def _basic(password: str) -> dict[str, str]:
    """Return a Basic auth header."""
    return {"Authorization": "Basic " + base64.b64encode(f"anyone:{password}".encode()).decode()}


def test_staging_puts_every_page_behind_a_code(server: Server, staging: TestClient) -> None:
    """Pages and the owner's API need a code; the worker's API, storage and health do not."""
    answer = staging.get("/")
    assert answer.status_code == 401
    assert answer.headers["www-authenticate"] == 'Basic realm="SLAM It!"'
    assert answer.headers["x-robots-tag"] == "noindex, nofollow"
    assert staging.get("/", headers=_basic("wrong-code-here")).status_code == 401
    assert staging.get("/", headers={"Authorization": "Basic %%%"}).status_code == 401
    assert staging.get("/", headers=_basic(CODE)).status_code == 200
    assert staging.get("/api/jobs/x", headers=_basic("another-invite-code")).status_code == 404
    claim = {"kinds": ["vo"], "wait": 0}
    assert staging.post("/api/worker/claim", json=claim, headers=WORKER).status_code == 204
    assert staging.get("/healthz").json() == {"ok": True}
    job = _make(server)
    assert staging.put(job["upload_url"], content=UPLOAD).status_code == 200


def test_a_job_the_sweep_ends_keeps_none_of_its_files(server: Server) -> None:
    """A worker gone silent: the sweep fails its job, then deletes its upload and result."""
    claimed, urls = _uploading(server)
    server.client.put(urls["a.npy"], content=b"a")
    assert len(list(server.store.root.glob("jobs/*/import"))) == 1
    server.clock.now += SILENCE
    assert server_app.sweep(server.jobs, server.store) == 0
    job = server.jobs.by_id(claimed["job"])
    assert job is not None
    assert (job.state, job.reason) == ("failed", "the worker stopped answering")
    assert list(server.store.root.glob("jobs/*/import")) == []
    assert list(server.store.root.rglob("*.npy")) == []


def test_the_pages_carry_a_content_policy_that_allows_their_own_scripts(server: Server) -> None:
    """The pages' Content Security Policy allows their own scripts and connections alone.

    Scripts from this site, three.js's CDN and the import map by its hash; connections to this
    site and storage; nothing frames the pages.
    """
    index = (server.settings.viewer / "index.html").read_text()
    script = index.split('<script type="importmap">', 1)[1].split("</script>", 1)[0]
    digest = base64.b64encode(hashlib.sha256(script.encode()).digest()).decode()
    policy = server.client.get("/").headers["content-security-policy"]
    directives = dict(part.strip().split(" ", 1) for part in policy.split(";"))
    assert directives["script-src"] == f"'self' {server_app.CDN} 'sha256-{digest}'"
    assert directives["connect-src"] == "'self' http://testserver"
    assert directives["media-src"] == "'self' blob:"  # the upload page reads a video's length
    assert directives["default-src"] == "'self'"
    assert directives["frame-ancestors"] == "'none'"
    assert server.client.get("/api/jobs/x").headers["content-security-policy"] == policy


def test_the_policy_allows_three_js_alone_of_its_cdn() -> None:
    """Scripts from one path of the CDN alone: three.js's, at its version.

    Every module the import map names is under it, and nothing else the CDN serves.
    """
    assert server_app.CDN.endswith("/")
    assert urllib.parse.urlsplit(server_app.CDN).path.startswith("/npm/three@")
    index = (VIEWER / "index.html").read_text()
    script = index.split('<script type="importmap">', 1)[1].split("</script>", 1)[0]
    modules = json.loads(script)
    named = [*modules["imports"].values(), *modules["integrity"]]
    assert named
    assert all(url.startswith(server_app.CDN) for url in named)


def test_a_viewer_whose_import_map_cannot_be_read_stops_the_server(tmp_path: Path) -> None:
    """No page, or an import map written otherwise: refused when the app is made.

    Not found later by a browser that cannot load the viewer.
    """
    store = FolderStorage(tmp_path / "files", "http://testserver", b"k" * 32)
    with pytest.raises(SettingsError, match=r"SLAM_IT_VIEWER: .*index\.html is not there"):
        server_app.content_policy(tmp_path, store)
    (tmp_path / "index.html").write_text('<script type="importmap" nonce="x">{}</script>')
    with pytest.raises(SettingsError, match="an import map not written as"):
        server_app.content_policy(tmp_path, store)
