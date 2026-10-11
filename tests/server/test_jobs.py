"""The job list: a job's life, one job worked on at a time, the caps, and the sweep's lists."""

from __future__ import annotations

import sqlite3
from pathlib import Path

import pytest

from slam_it_all.orchestration import worker
from slam_it_all_server import jobs
from slam_it_all_server.config import QUEUE, RETENTION, SILENCE, UPLOAD_WAIT
from slam_it_all_server.jobs import CODE_JOBS, QUEUE_WAIT, Jobs
from tests.server.conftest import Clock


@pytest.fixture
def clock() -> Clock:
    """Return a clock the test moves."""
    return Clock()


@pytest.fixture
def listed(tmp_path: Path, clock: Clock) -> Jobs:
    """Return an empty job list on the clock."""
    return Jobs(tmp_path / "jobs.sqlite3", clock)


def _queued(listed: Jobs, owner: str = "owner-a", upload: str = "video") -> jobs.Job:
    """Make a job and queue it; return it."""
    job = listed.create(upload, 1, owner)
    return listed.queue(job.token)


def test_a_job_lives_from_waiting_to_done(listed: Jobs, clock: Clock) -> None:
    """Made, queued, claimed, through every stage, its files recorded, done; kept a week."""
    job = listed.create("video", 1000, "owner-a")
    assert (job.state, job.kind, job.size, job.owner) == ("waiting", "vo", 1000, "owner-a")
    assert job.expires == clock.now + UPLOAD_WAIT
    assert len(bytes.fromhex(job.id)) == 16
    assert job.token != job.id
    assert len(job.token) >= 22  # 128 bits, URL-safe
    assert listed.by_token(job.token) == job
    clock.now += 1
    assert listed.queue(job.token).state == "queued"
    claimed = listed.claim(["vo"])
    assert claimed is not None
    assert (claimed.id, claimed.state, claimed.expires) == (
        job.id,
        "downloading",
        clock.now + SILENCE,
    )
    for stage in worker.STAGES[1:]:
        before, now = listed.progress(job.id, stage)
        assert (now.state, before) == (stage, worker.STAGES[worker.STAGES.index(stage) - 1])
    before, again = listed.progress(job.id, "uploading")  # a heartbeat
    assert (before, again.state) == ("uploading", "uploading")
    assert listed.held(job.id).state == "uploading"
    assert listed.record_files(job.id, {"bundle.json": 12}).files == {"bundle.json": 12}
    done = listed.done(job.id, {"frames": 3})
    assert (done.state, done.summary, done.expires) == (
        "done",
        {"frames": 3},
        clock.now + RETENTION,
    )
    with pytest.raises(jobs.StateError):
        listed.progress(job.id, "uploading")  # no longer the worker's


def test_one_job_is_worked_on_at_a_time(listed: Jobs) -> None:
    """A second queued job waits until the first ends; the oldest goes first."""
    first, second = (listed.create("video", 1, f"owner-{n}") for n in "ab")
    listed.queue(second.token)
    listed.queue(first.token)
    claimed = listed.claim(["vo"])
    assert claimed is not None
    assert claimed.id == first.id  # made first
    assert listed.ahead(listed.by_id(second.id) or second) == 1
    assert listed.claim(["vo"]) is None
    listed.fail(first.id, "no")
    claimed = listed.claim(["vo"])
    assert claimed is not None
    assert claimed.id == second.id


def test_a_worker_claims_only_its_kinds(listed: Jobs) -> None:
    """A worker of other kinds is given nothing."""
    _queued(listed, upload="stray")
    assert listed.claim(["lidar"]) is None
    assert listed.claim([]) is None
    assert listed.claim(["lidar", "vo"]) is not None


def test_the_queue_is_capped(listed: Jobs) -> None:
    """Past QUEUE open jobs, a new one is refused; an ended one frees its place."""
    made = [listed.create("video", 1, f"owner-{n}") for n in range(QUEUE)]
    with pytest.raises(jobs.FullError):
        listed.create("video", 1, "owner-new")
    listed.queue(made[0].token)
    claimed = listed.claim(["vo"])
    assert claimed is not None
    listed.fail(claimed.id, "no")
    listed.create("video", 1, "owner-new")


def test_one_code_opens_at_most_code_jobs(listed: Jobs) -> None:
    """The code's own cap holds before the queue's; an ended job frees one of its places."""
    theirs = [listed.create("video", 1, "owner-a") for _ in range(CODE_JOBS)]
    with pytest.raises(jobs.OwnerFullError):
        listed.create("video", 1, "owner-a")
    listed.create("video", 1, "owner-b")  # another code is not held up
    listed.queue(theirs[0].token)
    claimed = listed.claim(["vo"])
    assert claimed is not None
    listed.fail(claimed.id, "no")
    listed.create("video", 1, "owner-a")


def test_a_stage_never_goes_back_and_must_be_known(listed: Jobs) -> None:
    """A stage before the job's, or one not in STAGES, is refused."""
    _queued(listed)
    job = listed.claim(["vo"])
    assert job is not None
    listed.progress(job.id, "running")
    with pytest.raises(ValueError, match="comes before"):
        listed.progress(job.id, "importing")
    with pytest.raises(ValueError, match="not one of"):
        listed.progress(job.id, "resting")


def test_only_a_worker_job_takes_reports(listed: Jobs) -> None:
    """A waiting or queued job, or no job at all, is not a worker's to report on."""
    job = listed.create("video", 1, "owner-a")
    for report in (
        lambda: listed.progress(job.id, "importing"),
        lambda: listed.record_files(job.id, {}),
        lambda: listed.done(job.id, {}),
        lambda: listed.fail(job.id, "x"),
        lambda: listed.held("no-such-job"),
    ):
        with pytest.raises(jobs.StateError):
            report()
    with pytest.raises(jobs.StateError):
        listed.queue("no-such-token")
    listed.queue(job.token)
    with pytest.raises(jobs.StateError):
        listed.queue(job.token)  # queued once
    with pytest.raises(ValueError, match="not an upload or a kind"):
        listed.create("tarball", 1, "owner-a")


def test_jobs_whose_time_is_up_are_ended(listed: Jobs, clock: Clock) -> None:
    """A silent worker's job fails after SILENCE; a job with no upload expires after UPLOAD_WAIT."""
    waiting = listed.create("video", 1, "owner-a")
    working = _queued(listed, "owner-b")
    assert listed.claim(["vo"]) is not None
    assert listed.end_overdue() == []
    clock.now += SILENCE
    assert [(j.id, j.state, j.reason) for j in listed.end_overdue()] == [
        (working.id, "failed", "the worker stopped answering")
    ]
    clock.now += UPLOAD_WAIT - SILENCE
    assert [(j.id, j.state, j.reason) for j in listed.end_overdue()] == [
        (waiting.id, "expired", "the upload did not arrive in time")
    ]


def test_a_queued_job_waits_a_day_for_a_worker(listed: Jobs, clock: Clock) -> None:
    """Queued and never claimed, it expires after QUEUE_WAIT."""
    _queued(listed, upload="stray")
    clock.now += QUEUE_WAIT - 1
    assert listed.end_overdue() == []
    clock.now += 1
    assert [(j.state, j.reason) for j in listed.end_overdue()] == [
        ("expired", "no worker took it in time")
    ]


def test_an_upload_link_is_closed_once_it_has_expired(listed: Jobs, clock: Clock) -> None:
    """A job past waiting has its link's writes to clear once the link has expired, once."""
    waiting = listed.create("video", 1, "owner-a")
    queued = _queued(listed, "owner-b")
    assert listed.links_to_close() == []
    clock.now += UPLOAD_WAIT
    listed.end_overdue()  # the waiting one expires with its link
    assert {j.id for j in listed.links_to_close()} == {waiting.id, queued.id}
    listed.close_link(queued.id)
    assert [j.id for j in listed.links_to_close()] == [waiting.id]


def test_an_ended_jobs_files_are_cleared_then_it_is_deleted(listed: Jobs, clock: Clock) -> None:
    """Ended, its files are to clear at once; kept RETENTION, it is to delete, until deleted."""
    job = _queued(listed)
    listed.claim(["vo"])
    assert listed.files_to_clear() == []  # still the worker's
    listed.done(job.id, {})
    assert [j.id for j in listed.files_to_clear()] == [job.id]
    listed.clear_files(job.id)
    assert listed.files_to_clear() == []
    clock.now += RETENTION - 1
    assert listed.to_delete() == []
    clock.now += 1
    assert [j.id for j in listed.to_delete()] == [job.id]  # until its files are gone
    assert [j.id for j in listed.to_delete()] == [job.id]
    listed.delete(job.id)
    assert listed.to_delete() == []
    assert listed.by_token(job.token) is None


def test_a_status_read_does_not_wait_on_a_change(tmp_path: Path, clock: Clock) -> None:
    """With another connection holding the write lock, a job is still read at once (WAL)."""
    listed = Jobs(tmp_path / "jobs.sqlite3", clock)
    job = listed.create("video", 1, "owner-a")
    other = sqlite3.connect(tmp_path / "jobs.sqlite3", isolation_level=None, timeout=0.1)
    try:
        other.execute("BEGIN IMMEDIATE")
        assert listed.by_token(job.token) == job
        assert listed.ahead(job) == 0
        assert listed.files_to_clear() == []
    finally:
        other.execute("ROLLBACK")
        other.close()


def test_the_list_survives_a_restart(tmp_path: Path, clock: Clock) -> None:
    """A second list on the same file sees the first one's jobs."""
    job = Jobs(tmp_path / "jobs.sqlite3", clock).create("video", 5, "owner-a")
    assert Jobs(tmp_path / "jobs.sqlite3", clock).by_token(job.token) == job
