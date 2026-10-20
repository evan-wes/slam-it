"""Compute budgets: a process's peak memory, pinning and capping a program, judging its frames."""

from __future__ import annotations

import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from slam_it.eval import budgets, runs
from slam_it.foundations.errors import InvalidInputError

STATUS = "Name:\tslam_it_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n"
"""A ``/proc/<pid>/status`` excerpt: the peak resident set is ``VmHWM``, not ``VmPeak``."""


def test_peak_memory_reads_the_high_water_mark(tmp_path: Path) -> None:
    """The resident set's high-water mark, kilobytes to bytes; not the address space's peak."""
    status = tmp_path / "status"
    status.write_text(STATUS)
    assert budgets.peak_memory(status) == 47248 * 1024


@pytest.mark.parametrize("text", ["Name:\tx\nVmRSS:\t 41000 kB\n", "VmHWM:\t 47248 MB\n", ""])
def test_peak_memory_is_none_where_the_system_does_not_say(tmp_path: Path, text: str) -> None:
    """No VmHWM line, or one not in kilobytes, or no file at all: None."""
    status = tmp_path / "status"
    status.write_text(text)
    assert budgets.peak_memory(status) is None
    assert budgets.peak_memory(tmp_path / "no_such_file") is None


GROW = """
from slam_it.eval import budgets
before = budgets.peak_memory()
held = bytearray(64 * 2**20)
held[:: 2**12] = b"x" * len(held[:: 2**12])  # touch every page
print(before, budgets.peak_memory())
"""
"""A fresh process's peak before and after it holds 64 MiB more."""


@pytest.mark.skipif(sys.platform != "linux", reason="/proc/self/status is Linux's")
def test_peak_memory_grows_with_what_the_process_holds() -> None:
    """Holding 64 MiB more raises a fresh process's peak by about that much.

    In a child process: this one's peak, raised by whatever ran before, could already hide it.
    """
    done = subprocess.run([sys.executable, "-c", GROW], capture_output=True, text=True, check=True)
    before, after = map(int, done.stdout.split())
    assert after - before >= 60 * 2**20


def test_allowed_cpus_reads_the_affinity_list(tmp_path: Path) -> None:
    """The CPUs taskset allows, as Linux lists them; None where there is no such list."""
    status = tmp_path / "status"
    status.write_text("Name:\tslam_it_vo\nCpus_allowed:\tf0\nCpus_allowed_list:\t4-7\n")
    assert budgets.allowed_cpus(status) == [4, 5, 6, 7]
    status.write_text("Cpus_allowed_list:\t0-3,8,10-11\n")
    assert budgets.allowed_cpus(status) == [0, 1, 2, 3, 8, 10, 11]
    for text in [
        "Name:\tx\n",
        "Cpus_allowed_list:\t7-4\n",
        "Cpus_allowed_list:\tf0\n",
        "Cpus_allowed_list:\t1-\n",
        "Cpus_allowed_list:\t0-3,7-4\n",
        "Cpus_allowed_list:\t1_0\n",
    ]:  # each refused by the native readers too
        status.write_text(text)
        assert budgets.allowed_cpus(status) is None
    assert budgets.allowed_cpus(tmp_path / "no_such_file") is None
    if sys.platform == "linux":
        assert budgets.allowed_cpus() == sorted(os.sched_getaffinity(0))


def test_memory_cap_reads_the_groups_memory_max(tmp_path: Path) -> None:
    """The process's version-2 group's memory.max; None without a cap or such a group."""
    group = tmp_path / "root" / "app.slice" / "run-1.scope"
    group.mkdir(parents=True)
    cgroup = tmp_path / "cgroup"
    cgroup.write_text("0::/app.slice/run-1.scope\n")
    (group / "memory.max").write_text("8589934592\n")
    assert budgets.memory_cap(cgroup, tmp_path / "root") == 8 * 2**30
    (group / "memory.max").write_text("max\n")
    assert budgets.memory_cap(cgroup, tmp_path / "root") is None
    cgroup.write_text("1:name=systemd:/app.slice\n")
    assert budgets.memory_cap(cgroup, tmp_path / "root") is None
    assert budgets.memory_cap(tmp_path / "no_such_file", tmp_path / "root") is None


@pytest.mark.skipif(os.geteuid() == 0, reason="root reads any file")
def test_a_file_the_process_may_not_read_says_nothing(tmp_path: Path) -> None:
    """An unreadable status or memory.max (a sandbox's) is None, as in C++ and Rust.

    So a program that cannot read them still writes its run.
    """
    status = tmp_path / "status"
    status.write_text(STATUS + "Cpus_allowed_list:\t4-7\n")
    group = tmp_path / "root" / "run-1.scope"
    group.mkdir(parents=True)
    (group / "memory.max").write_text("8589934592\n")
    cgroup = tmp_path / "cgroup"
    cgroup.write_text("0::/run-1.scope\n")
    status.chmod(0)
    (group / "memory.max").chmod(0)
    try:
        assert budgets.peak_memory(status) is None
        assert budgets.allowed_cpus(status) is None
        assert budgets.memory_cap(cgroup, tmp_path / "root") is None
    finally:
        status.chmod(0o600)
        (group / "memory.max").chmod(0o600)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"name": ""}, "budget: expected a name"),
        ({"cores": 0}, "cores: expected a positive integer, got 0"),
        ({"cores": -2}, "cores: expected a positive integer"),
        ({"cores": 2.0}, "cores: expected a positive integer"),
        ({"cores": True}, "cores: expected a positive integer"),
        ({"memory": 0}, "memory: expected a positive integer"),
        ({"memory": -1}, "memory: expected a positive integer"),
    ],
)
def test_a_budget_checks_itself(change: dict[str, object], message: str) -> None:
    """A budget of no cores or memory is refused when it is made, not run unbudgeted."""
    fields = {"name": "small", "cores": 2, "memory": 2**30, "stands_for": "a test", **change}
    with pytest.raises(InvalidInputError, match=message):
        budgets.Budget(**fields)  # type: ignore[arg-type]


LAPTOP = budgets.BUDGETS["laptop-cpu"]
"""The milestone-2 budget: four cores."""


@pytest.mark.parametrize(
    ("offered", "performance", "pinned"),
    [
        (range(24), "0-7\n", [4, 5, 6, 7]),  # a hybrid CPU: its performance cores, the highest
        (range(24), "0-3,8,10-11\n", [3, 8, 10, 11]),  # a CPU list with single CPUs
        (range(8), None, [4, 5, 6, 7]),  # one kind of core: the highest offered
        ([0, 2, 5, 9, 11], "0-11\n", [2, 5, 9, 11]),  # only what this process may run on
    ],
)
def test_cpus_pins_the_highest_fast_cores_offered(
    offered: range | list[int],
    performance: str | None,
    pinned: list[int],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A budget's cores are performance cores where there are two kinds, CPU 0 left alone.

    No topology: each CPU is a core of its own, whatever hyperthreads the machine running the
    test has (a CI runner's pair CPUs 2 and 3).
    """
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: set(offered))
    listed = tmp_path / "cpus"
    if performance is not None:
        listed.write_text(performance)
    assert budgets.cpus(LAPTOP, performance=listed, topology=tmp_path / "none") == pinned


def test_cpus_pins_one_logical_cpu_a_physical_core(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """On a machine with hyperthreads, a budget's cores are physical (PR 9's panel, L12).

    Eight cores of two logical CPUs each, CPU n's sibling n + 8: four cores, the highest, each
    once, by its lowest logical CPU this process may run on.
    """
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: set(range(16)))
    topology = tmp_path / "cpu"
    for cpu in range(16):
        folder = topology / f"cpu{cpu}" / "topology"
        folder.mkdir(parents=True)
        (folder / "thread_siblings_list").write_text(f"{cpu % 8},{cpu % 8 + 8}\n")
    absent = tmp_path / "none"
    assert budgets.cpus(LAPTOP, performance=absent, topology=topology) == [4, 5, 6, 7]
    # Cores 1, 2 and 3 twice each, cores 4 and 5 by their second CPUs: the highest four cores.
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {1, 9, 2, 10, 3, 11, 12, 13})
    assert budgets.cpus(LAPTOP, performance=absent, topology=topology) == [2, 3, 12, 13]
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: {8, 9, 10, 0, 1})
    with pytest.raises(InvalidInputError, match="needs 4 cores, this machine offers 3"):
        budgets.cpus(LAPTOP, performance=absent, topology=topology)


def test_cpus_refuses_a_budget_the_machine_cannot_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Fewer fast cores than the budget's: refused, not quietly run on slow ones.

    No topology, as above: three CPUs are three cores on any machine running the test.
    """
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: set(range(24)))
    listed = tmp_path / "cpus"
    listed.write_text("0-2\n")
    with pytest.raises(InvalidInputError, match="laptop-cpu: needs 4 cores, this machine offers 3"):
        budgets.cpus(LAPTOP, performance=listed, topology=tmp_path / "none")


def test_command_caps_memory_without_swap_and_pins_the_cores() -> None:
    """The scope's memory cap kills rather than swaps; taskset pins; the program runs last."""
    assert budgets.command(LAPTOP, [4, 5, 6, 7], ["slam_it_vo", "--mode", "rgbd"]) == [
        *("systemd-run", "--user", "--scope", "--quiet"),
        *("-p", "MemoryMax=8589934592", "-p", "MemorySwapMax=0", "--"),
        *("taskset", "--cpu-list", "4,5,6,7", "slam_it_vo", "--mode", "rgbd"),
    ]


def _timed_run(
    steps: list[tuple[float, float, float, float]],
    period: float = 0.05,
    loops: dict[int, float] | None = None,
) -> runs.Run:
    """Return a run of frames `period` apart, each with its (read, build, track, map) seconds.

    `loops` gives the frames loop closing ran on their seconds; the rest have none.
    """
    times = 100.0 + period * np.arange(len(steps))
    looped = loops or {}
    ran = {"fusion": "ran", "adjustment": "ran", "culling": "ran"}
    diagnostics: list[dict[str, Any]] = [
        {"timestamp": float(t), "read_seconds": r, "build_seconds": b, "track_seconds": k,
         "map_seconds": m, "loop_seconds": looped.get(i, 0.0), "wait_seconds": 0.0,
         "asked": m > 0.0, "refused": False, "queue": 0, "schedule": ran if m > 0.0 else None,
         "global_ba": [], "global_ba_seconds": 0.0}
        for i, (t, (r, b, k, m)) in enumerate(zip(times, steps, strict=True))
    ]  # fmt: skip
    return runs.Run("euroc", "MH_01_easy", "stereo", "cpp", np.zeros(0), np.zeros((0, 4, 4)),
                    np.zeros((0, 4, 4)), diagnostics=diagnostics)  # fmt: skip


def test_judge_holds_building_and_tracking_to_the_frame_period() -> None:
    """Latency is building plus tracking; reading and mapping are judged apart from it."""
    steps = [(0.004, 0.010, 0.005, 0.0)] * 17
    steps += [
        (0.004, 0.030, 0.025, 0.200),
        (0.100, 0.012, 0.003, 0.0),
        (0.004, 0.020, 0.010, 0.100),
    ]
    judged = budgets.judge(_timed_run(steps))
    assert judged.count == 20
    assert judged.deadline == pytest.approx(0.05)
    assert judged.late == 1  # 0.030 + 0.025 > 0.05; the slow read and the mapping do not count
    assert judged.latency.median == pytest.approx(0.015)
    assert judged.latency.max == pytest.approx(0.055)
    assert judged.read.max == pytest.approx(0.100)
    assert judged.mapped == 2
    assert judged.mapping is not None
    assert (judged.mapping.median, judged.mapping.max) == pytest.approx((0.150, 0.200))
    assert judged.mapping_load == pytest.approx(0.300 / (19 * 0.05 + 0.05))
    # The run (1 s) is shorter than the window: its busiest stretch is the whole run.
    assert judged.window == pytest.approx(1.0)
    assert judged.mapping_peak_load == pytest.approx(judged.mapping_load)


def test_judge_reports_loop_closing_by_its_load() -> None:
    """Loop closing's time is its own thread's: neither latency nor mapping counts it."""
    steps = [(0.004, 0.010, 0.005, 0.0)] * 19 + [(0.004, 0.010, 0.005, 0.100)]
    plain = budgets.judge(_timed_run(steps))
    assert (plain.searched, plain.looping, plain.loop_load) == (0, None, 0.0)
    judged = budgets.judge(_timed_run(steps, loops={19: 0.250}))
    assert judged.searched == 1
    assert judged.looping is not None
    assert judged.looping.max == pytest.approx(0.250)
    assert judged.loop_load == pytest.approx(0.250 / (19 * 0.05 + 0.05))
    assert (judged.latency, judged.late, judged.mapping) == (
        plain.latency,
        plain.late,
        plain.mapping,
    )


def test_judge_finds_mappings_busiest_window() -> None:
    """Mapping's peak load is its busiest window's: a burst a whole-run load averages away."""
    steps = [(0.001, 0.002, 0.003, 0.0)] * 400  # 20 s at 20 Hz
    steps[100:110] = [(0.001, 0.002, 0.003, 0.08)] * 10  # 0.8 s of mapping within 0.5 s
    judged = budgets.judge(_timed_run(steps), window=1.0)
    assert judged.mapping_load == pytest.approx(0.8 / 20.0)
    assert (judged.window, judged.mapping_peak_load) == pytest.approx((1.0, 0.8))
    assert budgets.judge(_timed_run(steps)).mapping_peak_load == pytest.approx(0.08)


def test_a_short_runs_busiest_window_is_the_run() -> None:
    """A run shorter than the window is not diluted by the time after it.

    3 s of mapping in a 2-s run is a load of 1.5 over the run and over its busiest window, not
    0.3.
    """
    steps = [(0.001, 0.002, 0.003, 0.3 if k % 4 == 1 else 0.0) for k in range(40)]  # 2 s
    judged = budgets.judge(_timed_run(steps))
    assert judged.window == pytest.approx(2.0)
    assert judged.mapping_load == pytest.approx(1.5)
    assert judged.mapping_peak_load == pytest.approx(1.5)


def test_judge_has_no_mapping_spread_without_a_keyframe() -> None:
    """A run that mapped nothing has no mapping times and no load."""
    judged = budgets.judge(_timed_run([(0.001, 0.002, 0.003, 0.0)] * 3))
    assert (judged.mapped, judged.mapping, judged.mapping_load) == (0, None, 0.0)
    assert judged.mapping_peak_load == 0.0


def test_judge_finds_the_backlog_a_mapping_thread_would_build() -> None:
    """In order, Lindley's recursion over the keyframes: the longest a keyframe would wait.

    Keyframes every 0.25 s, each mapped in 0.375 s: each waits 0.125 s longer than the last,
    the fourth 0.375 s; a run of one keyframe waits for none.
    """
    steps = [(0.001, 0.002, 0.003, 0.375 if k % 5 == 0 else 0.0) for k in range(20)]
    judged = budgets.judge(_timed_run(steps))
    assert judged.threaded is False
    assert judged.backlog == pytest.approx(0.375)
    assert (judged.asked, judged.refused, judged.queue, judged.aborted) == (4, 0, 0, 0)
    one = [(0.001, 0.002, 0.003, 0.5)] + [(0.001, 0.002, 0.003, 0.0)] * 3
    assert budgets.judge(_timed_run(one)).backlog == 0.0


@pytest.mark.parametrize(
    ("times", "seconds", "message"),
    [
        ([0.0, 1.0], [0.1], "expected one mapping time per time"),
        ([1.0, 0.0], [0.1, 0.1], "times: must be ascending"),
        ([0.0, 1.0], [0.1, -0.1], "seconds: must not be negative"),
        ([0.0, float("nan")], [0.1, 0.1], "times"),
    ],
)
def test_the_backlog_refuses_what_is_not_a_run_of_keyframes(
    times: list[float], seconds: list[float], message: str
) -> None:
    """One mapping time per arrival, finite, the arrivals ascending (B3b's review panel, H9)."""
    with pytest.raises(InvalidInputError, match=message):
        budgets.backlog(np.array(times), np.array(seconds))


def test_judge_measures_a_threaded_runs_mapping_as_it_ran() -> None:
    """Threaded: the queue, the keyframes refused, the local BAs aborted, tracking's wait.

    The backlog, a measure for runs made in order, is not given: the queue was measured.
    """
    run = _timed_run([(0.001, 0.002, 0.003, 0.1 if k in (2, 4) else 0.0) for k in range(6)])
    run = replace(run, threaded=True)
    run.diagnostics[1].update(asked=True, refused=True, queue=3, wait_seconds=0.004)
    run.diagnostics[4]["schedule"] = {"fusion": "ran", "adjustment": "aborted", "culling": "ran"}
    judged = budgets.judge(run)
    assert (judged.threaded, judged.backlog) == (True, None)
    assert (judged.asked, judged.refused, judged.queue, judged.aborted) == (3, 1, 3, 1)
    assert judged.waited.max == pytest.approx(0.004)
    for key, bad, message in [
        ("asked", 1, "frame 2's asked must be true or false"),
        ("queue", "3", "frame 2's queue must be a number"),
        ("wait_seconds", -1.0, "frame 2's wait_seconds must be finite and at least 0"),
        ("schedule", "ran", "frame 2's schedule must be an object or null"),
    ]:
        broken = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 3)
        broken.diagnostics[2][key] = bad
        with pytest.raises(InvalidInputError, match=message):
            budgets.judge(broken)


def test_judge_refuses_what_it_cannot_judge() -> None:
    """One frame has no period; a frame without its steps' times cannot be judged."""
    with pytest.raises(InvalidInputError, match="needs two frames"):
        budgets.judge(_timed_run([(0.001, 0.002, 0.003, 0.0)]))
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 2)
    del run.diagnostics[1]["build_seconds"]
    with pytest.raises(InvalidInputError, match="a frame has no build_seconds"):
        budgets.judge(run)
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 2)
    del run.diagnostics[0]["timestamp"]
    with pytest.raises(InvalidInputError, match="a frame has no timestamp"):
        budgets.judge(run)


@pytest.mark.parametrize(
    ("times", "message"),
    [
        ([1.0, 1.0, 1.1], "frame 1's timestamp does not follow frame 0's"),
        ([2.0, 1.9, 1.8], "frame 1's timestamp does not follow frame 0's"),
        ([1.0, float("nan"), 1.2], "timestamps must be finite"),
        ([1.0, 1.1, float("inf")], "timestamps must be finite"),
    ],
)
def test_judge_refuses_times_that_do_not_advance(times: list[float], message: str) -> None:
    """Repeated, descending or non-finite times give no period: refused, not judged."""
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * len(times))
    for entry, t in zip(run.diagnostics, times, strict=True):
        entry["timestamp"] = t
    with pytest.raises(InvalidInputError, match=message):
        budgets.judge(run)


@pytest.mark.parametrize("bad", [None, "0.5", True, [0.1]])
@pytest.mark.parametrize("key", ["timestamp", *runs.STEPS])
def test_judge_refuses_a_value_that_is_not_a_number(key: str, bad: object) -> None:
    """A null, a string, a bool or a list is refused by name, not converted or crashed on."""
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 3)
    run.diagnostics[2][key] = bad
    with pytest.raises(InvalidInputError, match=f"frame 2's {key} must be a number"):
        budgets.judge(run)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -0.001])
@pytest.mark.parametrize("step", runs.STEPS)
def test_judge_refuses_a_step_time_it_cannot_judge(step: str, bad: float) -> None:
    """A step's time must be finite and at least 0: a NaN would pass for on time."""
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 3)
    run.diagnostics[2][step] = bad
    with pytest.raises(InvalidInputError, match=f"frame 2's {step} must be finite"):
        budgets.judge(run)


@pytest.mark.parametrize("window", [0.0, -1.0, float("nan"), float("inf")])
def test_judge_refuses_a_window_that_is_no_time(window: float) -> None:
    """Mapping's busiest stretch needs a positive, finite window."""
    with pytest.raises(InvalidInputError, match="window: expected a positive time"):
        budgets.judge(_timed_run([(0.001, 0.002, 0.003, 0.0)] * 3), window=window)
