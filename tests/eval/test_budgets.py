"""Compute budgets: a process's peak memory, pinning and capping a program, judging its frames."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

from slamlab.eval import budgets, runs
from slamlab.foundations.errors import InvalidInputError

STATUS = "Name:\tslamlab_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n"
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


@pytest.mark.skipif(sys.platform != "linux", reason="/proc/self/status is Linux's")
def test_peak_memory_grows_with_what_the_process_holds() -> None:
    """Holding 64 MiB more raises this process's peak by at least that much."""
    before = budgets.peak_memory()
    assert before is not None
    held = bytearray(64 * 2**20)
    held[:: 2**12] = b"x" * len(held[:: 2**12])  # touch every page
    after = budgets.peak_memory()
    assert after is not None
    assert after >= before
    assert after >= len(held)


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
    """A budget's cores are performance cores where there are two kinds, CPU 0 left alone."""
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: set(offered))
    listed = tmp_path / "cpus"
    if performance is not None:
        listed.write_text(performance)
    assert budgets.cpus(LAPTOP, performance=listed) == pinned


def test_cpus_refuses_a_budget_the_machine_cannot_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Fewer fast cores than the budget's: refused, not quietly run on slow ones."""
    monkeypatch.setattr(os, "sched_getaffinity", lambda pid: set(range(24)))
    listed = tmp_path / "cpus"
    listed.write_text("0-2\n")
    with pytest.raises(InvalidInputError, match="laptop-cpu: needs 4 cores, this machine offers 3"):
        budgets.cpus(LAPTOP, performance=listed)


def test_command_caps_memory_without_swap_and_pins_the_cores() -> None:
    """The scope's memory cap kills rather than swaps; taskset pins; the program runs last."""
    assert budgets.command(LAPTOP, [4, 5, 6, 7], ["slamlab_vo", "--mode", "rgbd"]) == [
        *("systemd-run", "--user", "--scope", "--quiet"),
        *("-p", "MemoryMax=8589934592", "-p", "MemorySwapMax=0", "--"),
        *("taskset", "--cpu-list", "4,5,6,7", "slamlab_vo", "--mode", "rgbd"),
    ]


def _timed_run(steps: list[tuple[float, float, float, float]], period: float = 0.05) -> runs.Run:
    """Return a run of frames `period` apart, each with its (read, build, track, map) seconds."""
    times = 100.0 + period * np.arange(len(steps))
    diagnostics = [
        {"timestamp": float(t), "read_seconds": r, "build_seconds": b, "track_seconds": k,
         "map_seconds": m}
        for t, (r, b, k, m) in zip(times, steps, strict=True)
    ]  # fmt: skip
    return runs.Run("euroc", "MH_01_easy", "stereo", "cpp", np.zeros(0), np.zeros((0, 4, 4)),
                    diagnostics=diagnostics)  # fmt: skip


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
    assert judged.keyframes == 2
    assert judged.mapping is not None
    assert (judged.mapping.median, judged.mapping.max) == pytest.approx((0.150, 0.200))
    assert judged.mapping_load == pytest.approx(0.300 / (19 * 0.05 + 0.05))


def test_judge_has_no_mapping_spread_without_a_keyframe() -> None:
    """A run that mapped nothing has no mapping times and no load."""
    judged = budgets.judge(_timed_run([(0.001, 0.002, 0.003, 0.0)] * 3))
    assert (judged.keyframes, judged.mapping, judged.mapping_load) == (0, None, 0.0)


def test_judge_refuses_what_it_cannot_judge() -> None:
    """One frame has no period; a frame without its steps' times cannot be judged."""
    with pytest.raises(InvalidInputError, match="needs two frames"):
        budgets.judge(_timed_run([(0.001, 0.002, 0.003, 0.0)]))
    run = _timed_run([(0.001, 0.002, 0.003, 0.0)] * 2)
    del run.diagnostics[1]["build_seconds"]
    with pytest.raises(InvalidInputError, match="a frame has no build_seconds"):
        budgets.judge(run)
