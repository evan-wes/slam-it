"""A card's segments, grouped into drives, each camera's in order.

A dashcam writes each camera's recording as short segments (:mod:`slam_it.eval.dashcam.d016`).
:func:`scan` lists a card's (or its copy's) segments; :func:`find_drives` groups them into
drives, a new drive wherever one camera's segments leave a gap longer than ``max_gap`` between
one start and the next, and gives each drive its front and rear segments.
"""

from __future__ import annotations

import datetime as dt
from dataclasses import dataclass
from pathlib import Path

from slam_it.eval.dashcam import d016
from slam_it.foundations.errors import InvalidInputError

MAX_GAP = 360.0
"""The longest time, in seconds, from one segment's start to the next within a drive: longer
than a 5-minute segment and the second or so between segments, shorter than a stop."""


@dataclass(frozen=True)
class Segment:
    """One recorded segment.

    Attributes
    ----------
    path : pathlib.Path
        The file.
    camera : str
        ``"F"`` (front) or ``"R"`` (rear).
    start : datetime.datetime
        Its start, naive local time, from its name.
    """

    path: Path
    camera: str
    start: dt.datetime

    @property
    def seconds(self) -> float:
        """The start as seconds on the camera's clock (:func:`d016.clock_seconds`)."""
        return d016.clock_seconds(self.start)


@dataclass(frozen=True)
class Drive:
    """One drive: each camera's segments, in time order.

    Attributes
    ----------
    front, rear : tuple of Segment
        The front camera's segments, and the rear's that start within the drive.
    """

    front: tuple[Segment, ...]
    rear: tuple[Segment, ...]

    @property
    def start(self) -> dt.datetime:
        """The drive's first segment's start."""
        return min(s.start for s in self.front + self.rear)

    @property
    def end(self) -> dt.datetime:
        """The drive's last segment's start."""
        return max(s.start for s in self.front + self.rear)

    def segments(self, camera: str) -> tuple[Segment, ...]:
        """Return one camera's segments.

        Parameters
        ----------
        camera : str
            ``"F"`` or ``"R"``.

        Returns
        -------
        tuple of Segment
            Its segments, in order.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If `camera` is neither.
        """
        if camera not in d016.FOLDERS:
            raise InvalidInputError(f"camera: expected F or R, got {camera!r}")
        return self.front if camera == "F" else self.rear


def scan(root: str | Path) -> list[Segment]:
    """List the segments under a card's root, both cameras, in time order.

    Files whose names are not a segment's are left out.

    Parameters
    ----------
    root : str or pathlib.Path
        The card's root, or a copy of it, holding ``video/F`` and ``video/R``.

    Returns
    -------
    list of Segment
        Every segment, by start, then camera.

    Raises
    ------
    FileNotFoundError
        If neither camera's folder exists.
    """
    base = Path(root)
    found: list[Segment] = []
    folders = [base.joinpath(*parts) for parts in d016.FOLDERS.values()]
    if not any(folder.is_dir() for folder in folders):
        raise FileNotFoundError(f"no video/F or video/R under {base}")
    for folder in folders:
        if not folder.is_dir():
            continue
        for path in folder.iterdir():
            if d016.NAME.fullmatch(path.name) is None or not path.is_file():
                continue
            camera, start = d016.segment_start(path.name)
            found.append(Segment(path, camera, start))
    return sorted(found, key=lambda s: (s.start, s.camera))


def find_drives(segments: list[Segment], max_gap: float = MAX_GAP) -> list[Drive]:
    """Group segments into drives.

    A drive is a run of front segments, each starting within `max_gap` of the one before; the
    rear segments starting from its first front segment's start, less `max_gap`, to its last's,
    plus `max_gap`, are its rear. A drive the front camera did not record is not found.

    Parameters
    ----------
    segments : list of Segment
        The segments (:func:`scan`).
    max_gap : float, optional
        The longest time from one segment's start to the next within a drive, in seconds.

    Returns
    -------
    list of Drive
        The drives, in time order.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `max_gap` is not positive.
    """
    if not max_gap > 0.0:
        raise InvalidInputError(f"max_gap: expected a positive time, got {max_gap}")
    front = sorted((s for s in segments if s.camera == "F"), key=lambda s: s.start)
    rear = sorted((s for s in segments if s.camera == "R"), key=lambda s: s.start)
    runs: list[list[Segment]] = []
    for segment in front:
        if runs and segment.seconds - runs[-1][-1].seconds <= max_gap:
            runs[-1].append(segment)
        else:
            runs.append([segment])
    drives = []
    for run in runs:
        first, last = run[0].seconds - max_gap, run[-1].seconds + max_gap
        mine = tuple(s for s in rear if first <= s.seconds <= last)
        drives.append(Drive(tuple(run), mine))
    return drives


def window(
    segments: tuple[Segment, ...], start: dt.datetime | None, end: dt.datetime | None
) -> tuple[Segment, ...]:
    """Return the segments that start in ``[start, end]``; an open end takes every one.

    Parameters
    ----------
    segments : tuple of Segment
        The segments.
    start, end : datetime.datetime or None
        The window, naive local times.

    Returns
    -------
    tuple of Segment
        Those starting inside it, in order.
    """
    return tuple(
        s
        for s in segments
        if (start is None or s.start >= start) and (end is None or s.start <= end)
    )
