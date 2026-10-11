"""The job list: a job's life, one job worked on at a time, the queue's cap, and the sweep."""

from __future__ import annotations

from pathlib import Path

import pytest

from slam_it_all.orchestration import worker
from slam_it_all_server import jobs
from slam_it_all_server.config import QUEUE, RETENTION, SILENCE, UPLOAD_WAIT
from slam_it_all_server.jobs import QUEUE_WAIT, Jobs
from tests.server.conftest import Clock


@pytest.fixture
def clock() -> Clock:
    """Return a clock the test moves."""
    return Clock()


@pytest.fixture
def listed(tmp_path: Path, clock: Clock) -> Jobs:
    """Return an empty job list on the clock."""
    return Jobs(tmp_path / "jobs.sqlite3", clock)


def test_a_job_lives_from_waiting_to_done(listed: Jobs, clock: Clock) -> None:
    """Made, queued, claimed, through every stage, its files recorded, done; kept a week."""
    job = listed.create("video", 1000)
    assert (job.state, job.kind, job.size, job.expires) == (
        "waiting",
        "vo",
        1000,
        clock.now + UPLOAD_WAIT,
    )
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
    first, second = (listed.create("video", 1) for _ in range(2))
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
    listed.queue(listed.create("stray", 1).token)
    assert listed.claim(["lidar"]) is None
    assert listed.claim([]) is None
    assert listed.claim(["lidar", "vo"]) is not None


def test_the_queue_is_capped(listed: Jobs) -> None:
    """Past QUEUE open jobs, a new one is refused; an ended one frees its place."""
    made = [listed.create("video", 1) for _ in range(QUEUE)]
    with pytest.raises(jobs.FullError):
        listed.create("video", 1)
    listed.queue(made[0].token)
    claimed = listed.claim(["vo"])
    assert claimed is not None
    listed.fail(claimed.id, "no")
    listed.create("video", 1)


def test_a_stage_never_goes_back_and_must_be_known(listed: Jobs) -> None:
    """A stage before the job's, or one not in STAGES, is refused."""
    listed.queue(listed.create("video", 1).token)
    job = listed.claim(["vo"])
    assert job is not None
    listed.progress(job.id, "running")
    with pytest.raises(ValueError, match="comes before"):
        listed.progress(job.id, "importing")
    with pytest.raises(ValueError, match="not one of"):
        listed.progress(job.id, "resting")


def test_only_a_worker_job_takes_reports(listed: Jobs) -> None:
    """A waiting or queued job, or no job at all, is not a worker's to report on."""
    job = listed.create("video", 1)
    for report in (
        lambda: listed.progress(job.id, "importing"),
        lambda: listed.record_files(job.id, {}),
        lambda: listed.done(job.id, {}),
        lambda: listed.fail(job.id, "x"),
        lambda: listed.working("no-such-job"),
    ):
        with pytest.raises(jobs.StateError):
            report()
    with pytest.raises(jobs.StateError):
        listed.queue("no-such-token")
    listed.queue(job.token)
    with pytest.raises(jobs.StateError):
        listed.queue(job.token)  # queued once
    with pytest.raises(jobs.StateError):
        listed.create("tarball", 1)


def test_the_sweep_ends_what_ran_out_of_time(listed: Jobs, clock: Clock) -> None:
    """A silent worker's job fails after SILENCE; a job with no upload expires after UPLOAD_WAIT."""
    waiting = listed.create("video", 1)
    working = listed.create("video", 1)
    listed.queue(working.token)
    assert listed.claim(["vo"]) is not None
    assert listed.sweep().ended == []
    clock.now += SILENCE
    swept = listed.sweep().ended
    assert [(j.id, j.state, j.reason) for j in swept] == [
        (working.id, "failed", "the worker stopped answering")
    ]
    clock.now += UPLOAD_WAIT - SILENCE
    swept = listed.sweep().ended
    assert [(j.id, j.state, j.reason) for j in swept] == [
        (waiting.id, "expired", "the upload did not arrive in time")
    ]


def test_a_queued_job_waits_a_day_for_a_worker(listed: Jobs, clock: Clock) -> None:
    """Queued and never claimed, it expires after QUEUE_WAIT."""
    job = listed.create("stray", 1)
    listed.queue(job.token)
    clock.now += QUEUE_WAIT - 1
    assert listed.sweep().ended == []
    clock.now += 1
    swept = listed.sweep()
    assert [(j.state, j.reason) for j in swept.ended] == [("expired", "no worker took it in time")]


def test_an_ended_job_is_deleted_after_retention(listed: Jobs, clock: Clock) -> None:
    """Done, it is kept RETENTION, then deleted, and the sweep says so."""
    job = listed.create("video", 1)
    listed.queue(job.token)
    listed.claim(["vo"])
    listed.done(job.id, {})
    clock.now += RETENTION - 1
    assert listed.sweep().deleted == []
    clock.now += 1
    swept = listed.sweep()
    assert [j.id for j in swept.deleted] == [job.id]
    assert listed.by_token(job.token) is None


def test_the_list_survives_a_restart(tmp_path: Path, clock: Clock) -> None:
    """A second list on the same file sees the first one's jobs."""
    job = Jobs(tmp_path / "jobs.sqlite3", clock).create("video", 5)
    assert Jobs(tmp_path / "jobs.sqlite3", clock).by_token(job.token) == job
