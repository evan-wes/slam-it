"""Compute budgets: running a program as a smaller machine would, and judging its frames.

There is no edge hardware, so a budget is simulated on this machine (the ladder's design doc,
"Compute budgets"): a program is pinned to the budget's cores with ``taskset`` and its memory
capped by a systemd scope (``MemoryMax``, no swap), so going over the cap kills it. What a
budget cannot reproduce (another microarchitecture, thermal throttling, power modes) it does
not claim.

- :data:`BUDGETS`: the budgets by name; ``laptop-cpu`` for milestone 2.
- :func:`cpus`: the CPUs a budget pins a program to: one logical CPU on each of as many
  physical cores (a core's hyperthreads are not cores of its own), this machine's performance
  cores on a CPU that has two kinds.
- :func:`command`: a command line wrapped to run within a budget.
- :func:`peak_memory`, :func:`allowed_cpus` and :func:`memory_cap`: what a program records in
  its run (:mod:`slam_it.eval.runs`) of the process that ran it: its largest resident set, the
  CPUs it could run on and its memory cap. C++'s ``slam_it::eval::budgets`` and Rust's
  ``slam_it_eval::budgets`` read them the same way.
- :func:`judge`: a run's frames against their real-time deadline, from the time each step
  took (the run format's :data:`slam_it.eval.runs.STEPS`).

**Real time, as ORB-SLAM's threads divide it** (decided for PR 9, 2026-10-07). A frame must
be built and tracked before the next arrives, its deadline the sequence's frame period: what
ORB-SLAM's tracking thread does. The clock starts with the frame's images decoded, as a
camera delivers them, so reading them is reported but not held to the deadline. Mapping a
keyframe runs after tracking in the same thread here, so a run stays deterministic; it is
judged as ORB-SLAM's local-mapping thread would be, by its load: its total time over the
sequence's, which must stay below 1 for a thread of its own to keep up, and by the backlog a
thread of its own would build, Lindley's recursion over the keyframes as they came. Closing
loops on a keyframe runs after mapping it, and is reported the same way, as its own thread's
load.

**Threaded** (milestone 2B's B3b, the run format's handshakes), mapping runs in a thread of its
own, and is judged as it ran: by the keyframes waiting for it when tracking asked for one
(its queue), the keyframes it refused, and the local BAs it aborted; and tracking's latency
holds the time it waited for the map. The load and the busiest window are still reported, and
the backlog, a measure for runs made in order, is not.

The budgeted runs are tests marked ``budget``, outside layer 1: ``python -m pytest -m budget``.
"""

from __future__ import annotations

import os
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import runs
from slam_it.foundations.errors import InvalidInputError

STATUS = Path("/proc/self/status")
"""Where Linux states the calling process's memory and the CPUs it may run on."""

CGROUP = Path("/proc/self/cgroup")
"""Where Linux names the calling process's control group."""

CGROUP_ROOT = Path("/sys/fs/cgroup")
"""Where the control groups (version 2) are mounted."""

WINDOW = 10.0
"""The window, in seconds, over which :func:`judge` finds mapping's busiest stretch: long
enough to hold a burst of keyframes' local BAs (a few seconds of camera motion), short enough
that the run's average does not hide it."""

PERFORMANCE_CORES = Path("/sys/devices/cpu_core/cpus")
"""Where Linux lists a hybrid CPU's performance cores (Intel's ``cpu_core``)."""

TOPOLOGY = Path("/sys/devices/system/cpu")
"""Where Linux describes each CPU, its physical core's logical CPUs among it
(``cpu<n>/topology/thread_siblings_list``)."""


@dataclass(frozen=True)
class Budget:
    """A compute budget: the machine a run is held to.

    Attributes
    ----------
    name : str
        Its name, e.g. ``"laptop-cpu"``.
    cores : int
        The CPU cores the program may run on.
    memory : int
        The most memory it may hold, in bytes; past it, it is killed.
    stands_for : str
        The machine it stands in for.
    """

    name: str
    cores: int
    memory: int
    stands_for: str

    def __post_init__(self) -> None:
        """Check the budget: a name, at least one core and a byte of memory.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If the name is empty, or the cores or memory are not positive integers.
        """
        if not isinstance(self.name, str) or not self.name:
            raise InvalidInputError(f"budget: expected a name, got {self.name!r}")
        for key in ("cores", "memory"):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise InvalidInputError(
                    f"{self.name}: {key}: expected a positive integer, got {value!r}"
                )


BUDGETS = {
    "laptop-cpu": Budget("laptop-cpu", cores=4, memory=8 * 2**30, stands_for="a laptop, no GPU"),
}
"""The budgets by name (the ladder's design doc, "Compute budgets"). The Pi- and Jetson-class
budgets come with milestone 7."""


_CPU_RANGE = re.compile(r"([0-9]+)(?:-([0-9]+))?")
"""One part of a Linux CPU list: a CPU, or an ascending range of them."""


def _cpu_list(text: str) -> set[int]:
    """Read a Linux CPU list, e.g. ``0-7,16``; raise ValueError if it is not one."""
    out: set[int] = set()
    for part in text.strip().split(","):
        match = _CPU_RANGE.fullmatch(part)
        if match is None:
            raise ValueError(f"not a CPU list: {text!r}")
        first, last = int(match[1]), int(match[2] or match[1])
        if last < first:
            raise ValueError(f"not a CPU list: {text!r}")
        out.update(range(first, last + 1))
    return out


def _core_of(cpu: int, topology: Path) -> int:
    """Return a logical CPU's physical core, as its lowest sibling; itself if unlisted."""
    siblings = topology / f"cpu{cpu}" / "topology" / "thread_siblings_list"
    try:
        return min(_cpu_list(siblings.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        return cpu


def cpus(
    budget: Budget,
    *,
    performance: str | Path = PERFORMANCE_CORES,
    topology: str | Path = TOPOLOGY,
) -> list[int]:
    """Return the CPUs a budget pins a program to.

    A budget's cores are physical cores (PR 9's review panel, L12): one logical CPU of each,
    the lowest this process may run on, so that the program's threads never share a core's
    hyperthreads. On a hybrid CPU, its performance cores (a budget's cores are fast ones),
    else any; of those this process may run on, the highest-numbered cores, which leaves
    CPU 0, where the system's interrupts land, alone.

    Parameters
    ----------
    budget : Budget
        The budget.
    performance : str or pathlib.Path, optional
        The file listing the performance cores (default :data:`PERFORMANCE_CORES`); a CPU
        without one has one kind of core.
    topology : str or pathlib.Path, optional
        The folder describing each CPU (default :data:`TOPOLOGY`); a CPU it does not list is
        a core of its own.

    Returns
    -------
    list of int
        ``budget.cores`` CPUs, ascending, each on a core of its own.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If this machine offers fewer physical cores.
    """
    offered = os.sched_getaffinity(0)
    listed = Path(performance)
    if listed.is_file():
        offered &= _cpu_list(listed.read_text(encoding="utf-8"))
    cores = physical(offered, topology=topology)
    if len(cores) < budget.cores:
        raise InvalidInputError(
            f"{budget.name}: needs {budget.cores} cores, this machine offers {len(cores)}"
        )
    return cores[-budget.cores :]


def physical(offered: set[int], *, topology: str | Path = TOPOLOGY) -> list[int]:
    """Return one logical CPU of each physical core among those offered: its lowest offered.

    Parameters
    ----------
    offered : set of int
        Logical CPUs.
    topology : str or pathlib.Path, optional
        The folder describing each CPU (default :data:`TOPOLOGY`); a CPU it does not list is
        a core of its own.

    Returns
    -------
    list of int
        A CPU a core, ascending.
    """
    one_a_core: dict[int, int] = {}
    for cpu in sorted(offered):
        one_a_core.setdefault(_core_of(cpu, Path(topology)), cpu)
    return sorted(one_a_core.values())


def command(budget: Budget, pinned: Sequence[int], argv: Sequence[str]) -> list[str]:
    """Return a command line wrapped to run within a budget.

    ``systemd-run --scope`` caps its memory (``MemoryMax``, and no swap, so going over the cap
    kills it rather than slowing it) and ``taskset`` pins it to its CPUs; each runs the next
    in its own process, so the program's peak memory is its own.

    Parameters
    ----------
    budget : Budget
        The budget.
    pinned : sequence of int
        The CPUs (:func:`cpus`).
    argv : sequence of str
        The program and its arguments.

    Returns
    -------
    list of str
        The wrapped command line.

    Examples
    --------
    >>> command(BUDGETS["laptop-cpu"], [4, 5, 6, 7], ["slam_it_vo", "--help"])[-4:]
    ['--cpu-list', '4,5,6,7', 'slam_it_vo', '--help']
    """
    return [
        *("systemd-run", "--user", "--scope", "--quiet"),
        *("-p", f"MemoryMax={budget.memory}", "-p", "MemorySwapMax=0", "--"),
        *("taskset", "--cpu-list", ",".join(map(str, pinned))),
        *argv,
    ]


def _status_lines(status: str | Path) -> list[list[str]]:
    """Return a status file's lines split into words; none if it cannot be read."""
    try:
        text = Path(status).read_text(encoding="utf-8")
    except OSError:
        return []
    return [line.split() for line in text.splitlines()]


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
        The bytes, or None where the system does not say: a file it cannot read, or no
        ``VmHWM`` line in kilobytes.

    Examples
    --------
    >>> import sys
    >>> sys.platform != "linux" or peak_memory() > 0
    True
    """
    for words in _status_lines(status):
        if len(words) == 3 and words[0] == "VmHWM:" and words[1].isdigit() and words[2] == "kB":
            return int(words[1]) * 1024
    return None


def allowed_cpus(status: str | Path = STATUS) -> list[int] | None:
    """Return the CPUs the calling process may run on: its affinity, as a budget pins it.

    Linux's ``Cpus_allowed_list``, which ``taskset`` sets.

    Parameters
    ----------
    status : str or pathlib.Path, optional
        The status file to read (default :data:`STATUS`).

    Returns
    -------
    list of int or None
        The CPUs, ascending, or None where the system does not say: a file it cannot read, or
        no ``Cpus_allowed_list`` line that reads as a CPU list.

    Examples
    --------
    >>> import sys
    >>> sys.platform != "linux" or len(allowed_cpus()) > 0
    True
    """
    for words in _status_lines(status):
        if len(words) == 2 and words[0] == "Cpus_allowed_list:":
            try:
                return sorted(_cpu_list(words[1])) or None
            except ValueError:
                return None
    return None


def memory_cap(cgroup: str | Path = CGROUP, root: str | Path = CGROUP_ROOT) -> int | None:
    """Return the calling process's memory cap: its control group's ``memory.max``.

    A budget's ``systemd-run --scope -p MemoryMax=...`` sets it on the scope the program runs
    in (control groups version 2). A cap set on an enclosing group only is not reported.

    Parameters
    ----------
    cgroup : str or pathlib.Path, optional
        The file naming the process's control group (default :data:`CGROUP`).
    root : str or pathlib.Path, optional
        Where the control groups are mounted (default :data:`CGROUP_ROOT`).

    Returns
    -------
    int or None
        The bytes, or None where there is no cap (``max``) or the system does not say: a file
        it cannot read, no version-2 group (a ``0::`` line), or no ``memory.max`` in it.
    """
    try:
        lines = Path(cgroup).read_text(encoding="utf-8").splitlines()
        group = next(line.removeprefix("0::") for line in lines if line.startswith("0::"))
        text = (Path(root) / group.lstrip("/") / "memory.max").read_text(encoding="utf-8")
    except (OSError, StopIteration):
        return None
    value = text.strip()
    return int(value) if value.isdigit() and int(value) > 0 else None


@dataclass(frozen=True)
class Spread:
    """How a time is spread over frames.

    Attributes
    ----------
    median : float
        Its median, in seconds.
    p95 : float
        Its 95th percentile (numpy's linear interpolation).
    max : float
        Its largest.
    """

    median: float
    p95: float
    max: float


def _spread(seconds: NDArray[np.float64]) -> Spread:
    """Return how times are spread."""
    return Spread(
        float(np.median(seconds)), float(np.percentile(seconds, 95)), float(np.max(seconds))
    )


@dataclass(frozen=True)
class Frames:
    """A run's frames judged against their real-time deadline (see the module docstring).

    Attributes
    ----------
    count : int
        The frames run.
    deadline : float
        The sequence's frame period, in seconds: the median spacing of the frames' times.
    read, build, track : Spread
        Each step's time per frame.
    latency : Spread
        Building and tracking a frame: what the deadline holds.
    late : int
        The frames whose latency passed the deadline.
    mapped : int
        The keyframes mapped: every keyframe but the one that starts the map, whose mapping is
        part of initialisation (in ``track_seconds``).
    mapping : Spread or None
        Mapping's time per keyframe mapped; None if none was.
    mapping_load : float
        Mapping's total time over the sequence's (the frames' span plus one period): below 1,
        a mapping thread of its own would keep up on average.
    window : float
        The window ``mapping_peak_load`` is over, in seconds: the judgement's ``window``, or
        the run's length (its span plus one period) if that is shorter.
    mapping_peak_load : float
        Mapping's largest load over any window of ``window`` seconds that starts at a frame:
        below 1, a mapping thread of its own would keep up there too. On a run no longer than
        the judgement's window it is the run's own load, ``mapping_load``.
    searched : int
        The keyframes loop closing ran on.
    looping : Spread or None
        Loop closing's time per keyframe it ran on, a correction's included; None if none.
    loop_load : float
        Loop closing's total time over the sequence's, as ``mapping_load``.
    threaded : bool
        Whether the run's steps ran in threads of their own.
    waited : Spread
        The time tracking waited for the map per frame (part of its latency); 0 in order.
    asked : int
        The frames that wanted a keyframe.
    refused : int
        Those mapping could not take: stopped, or busy and its queue full. None in order.
    queue : int
        The most keyframes waiting for mapping when a frame asked; 0 in order.
    aborted : int
        The local BAs aborted, a keyframe having been queued or a stop requested; none in order.
    backlog : float or None
        Run in order: the longest a keyframe would have waited for a mapping thread of its own,
        in seconds, by Lindley's recursion over the keyframes as they came,
        ``W' = max(0, W + S - A)`` with ``S`` a keyframe's mapping time and ``A`` the time to
        the next (PR 9's review panel, L8): an upper bound, as ORB-SLAM also refuses keyframes
        and aborts its local BA. None for a threaded run, whose queue was measured.
    """

    count: int
    deadline: float
    read: Spread
    build: Spread
    track: Spread
    latency: Spread
    late: int
    mapped: int
    mapping: Spread | None
    mapping_load: float
    window: float
    mapping_peak_load: float
    searched: int
    looping: Spread | None
    loop_load: float
    threaded: bool
    waited: Spread
    asked: int
    refused: int
    queue: int
    aborted: int
    backlog: float | None


def _number(entry: dict[str, object], key: str, k: int) -> float:
    """Return a diagnostic's number: a JSON number (not a bool), or InvalidInputError."""
    if key not in entry:
        raise InvalidInputError(f"judge: a frame has no {key}")
    value = entry[key]
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise InvalidInputError(f"judge: frame {k}'s {key} must be a number, got {value!r}")
    return float(value)


def _times(run: runs.Run) -> NDArray[np.float64]:
    """Return a run's frame times, checked finite and strictly increasing."""
    times = np.array([_number(d, "timestamp", k) for k, d in enumerate(run.diagnostics)])
    if not np.all(np.isfinite(times)):
        raise InvalidInputError("judge: the frames' timestamps must be finite")
    if np.any(np.diff(times) <= 0.0):
        k = int(np.argmax(np.diff(times) <= 0.0)) + 1
        raise InvalidInputError(f"judge: frame {k}'s timestamp does not follow frame {k - 1}'s")
    return times


def _steps(run: runs.Run) -> dict[str, NDArray[np.float64]]:
    """Return each step's time per frame (:data:`slam_it.eval.runs.STEPS`), checked."""
    out = {}
    for key in runs.STEPS:
        seconds = np.array([_number(d, key, k) for k, d in enumerate(run.diagnostics)])
        bad = ~np.isfinite(seconds) | (seconds < 0.0)
        if np.any(bad):
            raise InvalidInputError(
                f"judge: frame {int(np.argmax(bad))}'s {key} must be finite and at least 0"
            )
        out[key.removesuffix("_seconds")] = seconds
    return out


def _flags(run: runs.Run, key: str) -> NDArray[np.bool_]:
    """Return a handshake's flag per frame, checked a JSON bool."""
    out = []
    for k, d in enumerate(run.diagnostics):
        if key not in d:
            raise InvalidInputError(f"judge: a frame has no {key}")
        if not isinstance(d[key], bool):
            raise InvalidInputError(f"judge: frame {k}'s {key} must be true or false")
        out.append(d[key])
    return np.array(out, dtype=bool)


def _aborted(run: runs.Run) -> int:
    """Return the local BAs a run aborted: its keyframes' schedules whose BA was aborted."""
    count = 0
    for k, d in enumerate(run.diagnostics):
        if "schedule" not in d:
            raise InvalidInputError("judge: a frame has no schedule")
        schedule = d["schedule"]
        if schedule is None:
            continue
        if not isinstance(schedule, dict) or "adjustment" not in schedule:
            raise InvalidInputError(f"judge: frame {k}'s schedule must be an object or null")
        count += schedule["adjustment"] == "aborted"
    return count


def backlog(times: NDArray[np.float64], seconds: NDArray[np.float64]) -> float:
    """Return the longest a keyframe waits for a mapping thread of its own (Lindley's recursion).

    Parameters
    ----------
    times : ndarray of float64, shape (K,)
        Each keyframe's arrival, its frame's time, ascending.
    seconds : ndarray of float64, shape (K,)
        Each one's mapping time.

    Returns
    -------
    float
        The largest wait ``W``, from ``W_0 = 0`` and ``W_{n+1} = max(0, W_n + S_n - A_n)``,
        ``A_n`` the time from keyframe ``n`` to the next; 0 for fewer than two keyframes.

    Examples
    --------
    >>> backlog(np.array([0.0, 0.5, 1.0]), np.array([0.75, 0.75, 0.75]))
    0.5
    """
    wait = longest = 0.0
    for gap, service in zip(np.diff(times), seconds[:-1], strict=True):
        wait = max(0.0, wait + float(service) - float(gap))
        longest = max(longest, wait)
    return longest


def judge(run: runs.Run, *, window: float = WINDOW) -> Frames:
    """Judge a run's frames against their real-time deadline.

    The split is the edge pipeline's, as ORB-SLAM's threads divide the work: building and
    tracking a frame hold its deadline, and mapping and loop closing are judged by their loads.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run, each diagnostic with its time and its steps' times
        (:data:`slam_it.eval.runs.STEPS`).
    window : float, optional
        The seconds over which mapping's busiest stretch is found (default :data:`WINDOW`).

    Returns
    -------
    Frames
        The judgement.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the run has fewer than two frames (no period), a frame's time, a step's time or a
        handshake (:data:`slam_it.eval.runs.HANDSHAKES`) is missing or not of its type, the
        frames' times are not finite and strictly increasing, a step's time or the wait is
        negative or not finite, or `window` is not positive and finite.
    """
    if not (np.isfinite(window) and window > 0.0):
        raise InvalidInputError(f"judge: window: expected a positive time, got {window}")
    entries = run.diagnostics
    if len(entries) < 2:
        raise InvalidInputError(f"judge: needs two frames for their period, got {len(entries)}")
    times = _times(run)
    steps = _steps(run)
    deadline = float(np.median(np.diff(times)))
    latency = steps["build"] + steps["track"]
    mapped = steps["map"][steps["map"] > 0.0]
    looped = steps["loop"][steps["loop"] > 0.0]
    # The busiest window: mapping's time over [t_i, t_i + width), for each frame i, the width
    # no longer than the run, whose load a shorter run's window would otherwise dilute.
    length = times[-1] - times[0] + deadline
    width = min(window, length)
    ends = np.searchsorted(times, times + width)
    cumulative = np.concatenate([[0.0], np.cumsum(steps["map"])])
    windows = cumulative[ends] - cumulative[: len(times)]
    waited = np.array([_number(d, "wait_seconds", k) for k, d in enumerate(entries)])
    if np.any(~np.isfinite(waited) | (waited < 0.0)):
        k = int(np.argmax(~np.isfinite(waited) | (waited < 0.0)))
        raise InvalidInputError(f"judge: frame {k}'s wait_seconds must be finite and at least 0")
    queue = np.array([_number(d, "queue", k) for k, d in enumerate(entries)])
    keyframes = steps["map"] > 0.0
    return Frames(
        count=len(entries),
        deadline=deadline,
        read=_spread(steps["read"]),
        build=_spread(steps["build"]),
        track=_spread(steps["track"]),
        latency=_spread(latency),
        late=int(np.sum(latency > deadline)),
        mapped=len(mapped),
        mapping=_spread(mapped) if len(mapped) else None,
        mapping_load=float(np.sum(mapped) / length),
        window=float(width),
        mapping_peak_load=float(np.max(windows) / width),
        searched=len(looped),
        looping=_spread(looped) if len(looped) else None,
        loop_load=float(np.sum(looped) / length),
        threaded=run.threaded,
        waited=_spread(waited),
        asked=int(np.sum(_flags(run, "asked"))),
        refused=int(np.sum(_flags(run, "refused"))),
        queue=int(np.max(queue)),
        aborted=_aborted(run),
        backlog=None if run.threaded else backlog(times[keyframes], steps["map"][keyframes]),
    )
