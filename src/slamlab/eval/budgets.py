"""Compute budgets: what a program costs, and running it as a smaller machine would.

- :func:`peak_memory`: the calling process's largest resident set, which a program records in
  its run (:mod:`slamlab.eval.runs`); C++'s ``slamlab::eval::budgets`` and Rust's
  ``slamlab_eval::budgets`` read it the same way.

Still to come with PR 9's harness: the budgets themselves, pinning and capping a program to
one, and judging a run's frames against their real-time deadline.
"""

from __future__ import annotations

from pathlib import Path

STATUS = Path("/proc/self/status")
"""Where Linux states the calling process's memory."""


def peak_memory(status: str | Path = STATUS) -> int | None:
    """Return the calling process's largest resident set so far, in bytes.

    Linux's ``VmHWM``, the high-water mark of the process's resident set. It starts again at
    ``exec``, so unlike ``getrusage``'s ``ru_maxrss`` it does not count the process that
    spawned this one (which, from a Python harness, would be a hundred megabytes or more).

    Parameters
    ----------
    status : str or pathlib.Path, optional
        The status file to read (default :data:`STATUS`).

    Returns
    -------
    int or None
        The bytes, or None where the system does not say: no such file, or no ``VmHWM`` line
        in kilobytes.

    Examples
    --------
    >>> import sys
    >>> sys.platform != "linux" or peak_memory() > 0
    True
    """
    try:
        text = Path(status).read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    for line in text.splitlines():
        words = line.split()
        if len(words) == 3 and words[0] == "VmHWM:" and words[1].isdigit() and words[2] == "kB":
            return int(words[1]) * 1024
    return None
