"""Compute budgets: a process's peak memory."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from slamlab.eval import budgets

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
