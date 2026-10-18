"""Reading a dashcam's burned-in overlay: its GPS track, and its clock against the video's.

The overlay (:mod:`slam_it.eval.dashcam.d016`) is white text over whatever the camera sees, read
with Tesseract (``pytesseract``, the optional ``dashcam`` extra):

- **Each sampled frame** (every :data:`EVERY` seconds, decoded straight through) gives its
  overlay box; its white pixels (every channel above :data:`WHITE_LEVEL`, within
  :data:`WHITE_SPREAD` of each other: a bright but coloured background is not white) are ANDed
  with the frame before's and after's, since the overlay holds still within its second while the
  scene moves; Tesseract reads that, its characters limited to the overlay's. Next to the
  clock's tick the AND mixes two seconds' digits and reads the strokes they share, often the
  old second's, which would time the tick a frame late: so a frame whose clock reads other than
  a neighbour's is read again, alone.
- **Each read** is parsed (:func:`slam_it.eval.dashcam.d016.parse_overlay`), its fields range
  checked, and dated: a read of its segment's date. Every read of the clock times the camera;
  only those whose fix parsed too vote for a second's fix.
- **The lag**: the overlay's clock runs as the segment's start (its name) plus the lag plus the
  video's time, and shows that time's whole second. So a read of second ``N`` at video time
  ``t`` holds the lag in ``[N - start - t, N - start - t + 1)``, and the lag is the middle of
  the span most reads' intervals share (a misread's lies elsewhere): over a drive's many
  seconds it narrows to the time between reads, where the median of the reads' own values
  would be half a second early. A read whose interval's middle is more than
  :data:`CLOCK_SLACK` from the lag is a misread.
- **Each second** of the overlay's clock gets the value most of its reads agree on, and the first
  video time that showed it.

The camera's lag puts its frames on its overlay's clock; the two cameras' clocks differ, so each
camera's lag is measured. Times are on the camera's local clock as :func:`d016.clock_seconds`
gives them; a fix is taken to be the receiver's position at its second's start.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from concurrent.futures import ProcessPoolExecutor
from contextlib import nullcontext
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import video
from slam_it.eval.dashcam import d016
from slam_it.eval.dashcam.drives import Segment
from slam_it.foundations.errors import InvalidInputError

EVERY = 0.2
"""The time between the frames read, in seconds: five reads to each second of the clock."""

WHITE_LEVEL = 200
"""The least value, in every channel, of the overlay's white."""

WHITE_SPREAD = 50
"""The most a white pixel's channels differ."""

CLOCK_SLACK = 1.5
"""How far, in seconds, a read's clock may stray from the camera's lag and be kept."""

MIN_WIDTH = 1000
"""Tesseract reads small text better scaled up: a box narrower than this is scaled to it."""

BORDER = 20
"""White pixels framed around the text before Tesseract reads it."""


def white(rgb: NDArray[np.uint8]) -> NDArray[np.bool_]:
    """Return the overlay's white pixels in an RGB box (the module docstring)."""
    lo = rgb.min(axis=2).astype(np.int16)
    hi = rgb.max(axis=2).astype(np.int16)
    return np.asarray((lo > WHITE_LEVEL) & (hi - lo < WHITE_SPREAD))


def ocr(strokes: NDArray[np.bool_]) -> tuple[str, str]:
    """Read the overlay's two lines from its strokes, with Tesseract.

    Parameters
    ----------
    strokes : ndarray of bool, shape (H, W)
        The text's pixels.

    Returns
    -------
    tuple of str
        The first line that holds a fix and the first that holds a clock (spaces removed),
        each empty if none was read.

    Raises
    ------
    ImportError
        If ``pytesseract`` (the ``dashcam`` extra) or the Tesseract program is missing.
    """
    import pytesseract  # noqa: PLC0415 -- the optional extra, imported where it is used

    page = np.where(strokes, 0, 255).astype(np.uint8)  # dark text on white
    if page.shape[1] < MIN_WIDTH:
        factor = MIN_WIDTH / page.shape[1]
        tall = round(page.shape[0] * factor)
        rows = np.minimum((np.arange(tall) / factor).astype(int), page.shape[0] - 1)
        cols = np.minimum((np.arange(MIN_WIDTH) / factor).astype(int), page.shape[1] - 1)
        page = page[rows[:, None], cols[None, :]]
    page = np.pad(page, BORDER, constant_values=255)
    config = f"--psm 6 -c tessedit_char_whitelist={d016.OVERLAY_CHARACTERS}"
    lines = [
        ln.replace(" ", "") for ln in pytesseract.image_to_string(page, config=config).split("\n")
    ]
    first = next((ln for ln in lines if d016.FIX_LINE.search(ln)), "")
    second = next((ln for ln in lines if d016.CLOCK_LINE.search(ln)), "")
    return first, second


@dataclass(frozen=True)
class Reading:
    """One frame's overlay, as read.

    Attributes
    ----------
    video_s : float
        The frame's time in its segment, in seconds since its first frame.
    first, second : str
        The lines as read (spaces removed).
    overlay : slam_it.eval.dashcam.d016.Overlay
        What parsed of them.
    """

    video_s: float
    first: str
    second: str
    overlay: d016.Overlay


def read_segment(segment: Segment, *, every: float = EVERY, jobs: int = 1) -> list[Reading]:
    """Read a segment's overlay, frame by sampled frame (the module docstring).

    Parameters
    ----------
    segment : Segment
        The segment.
    every : float, optional
        The time between the frames read, in seconds.
    jobs : int, optional
        Tesseract processes run at once.

    Returns
    -------
    list of Reading
        One per sampled frame, in order.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If no overlay box is known for the segment's size, or `jobs` is not positive.
    """
    if jobs < 1:
        raise InvalidInputError(f"jobs: expected at least 1, got {jobs}")
    box = d016.overlay_box(*video.segment_size(segment.path))
    times, masks = [], []
    for time, frame in video.sample_frames(segment.path, every=every, box=box):
        times.append(time)
        masks.append(white(frame))
    held = [
        masks[k] & masks[max(k - 1, 0)] & masks[min(k + 1, len(masks) - 1)]
        for k in range(len(masks))
    ]
    with ProcessPoolExecutor(jobs) if jobs > 1 else nullcontext() as pool:

        def read_all(pages: list[NDArray[np.bool_]]) -> list[tuple[str, str]]:
            if pool is None:
                return [ocr(page) for page in pages]
            return list(pool.map(ocr, pages, chunksize=16))

        readings = [
            Reading(t, a, b, d016.parse_overlay(a, b))
            for t, (a, b) in zip(times, read_all(held), strict=True)
        ]
        clocks = [r.overlay.clock for r in readings]
        n = len(clocks)
        again = [
            k
            for k in range(n)
            if (k > 0 and clocks[k - 1] != clocks[k]) or (k < n - 1 and clocks[k + 1] != clocks[k])
        ]
        for k, (a, b) in zip(again, read_all([masks[k] for k in again]), strict=True):
            readings[k] = Reading(times[k], a, b, d016.parse_overlay(a, b))
    return readings


@dataclass(frozen=True)
class Fix:
    """One second of the overlay's clock.

    Attributes
    ----------
    clock : float
        The second, on the camera's clock (:func:`d016.clock_seconds`).
    latitude, longitude : float
        The fix most reads of the second agree on, in degrees.
    mph : int
        Its speed, in miles an hour.
    agree, reads : int
        The reads that agree, of those kept for the second.
    segment : str
        The segment that first showed it.
    tick : float
        The video time there that first showed it, in seconds.
    """

    clock: float
    latitude: float
    longitude: float
    mph: int
    agree: int
    reads: int
    segment: str
    tick: float


@dataclass(frozen=True)
class Track:
    """A camera's overlay, read over its segments.

    Attributes
    ----------
    fixes : list of Fix
        One per second read, in clock order.
    lag : float
        The camera's overlay clock less its segments' names' clock, at the same video time, in
        seconds (the module docstring).
    reads, kept : int
        The frames read, and the reads kept: their clock read, dated right and on time (a
        second's fix is voted among those whose fix parsed too).
    """

    fixes: list[Fix]
    lag: float
    reads: int
    kept: int


def shared_middle(lows: Sequence[float]) -> float:
    """Return the middle of the span the most intervals ``[low, low + 1)`` share.

    Parameters
    ----------
    lows : sequence of float
        The intervals' starts.

    Returns
    -------
    float
        The middle of the first span of the greatest depth.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are no intervals.
    """
    if not lows:
        raise InvalidInputError("lows: none given")
    # An interval's end sorts before another's start at the same time: they are half-open.
    events = sorted([(a, 1) for a in lows] + [(a + 1.0, -1) for a in lows])
    depth = best = 0
    span = (float(lows[0]), float(lows[0]) + 1.0)
    for k, (at, step) in enumerate(events[:-1]):
        depth += step
        if depth > best:
            best = depth
            span = (at, events[k + 1][0])
    return float(span[0] + span[1]) / 2.0


def track(segments: Sequence[Segment], readings: Sequence[Sequence[Reading]]) -> Track:
    """Turn each segment's reads into one track, and the camera's lag (the module docstring).

    Parameters
    ----------
    segments : sequence of Segment
        The segments, one camera's.
    readings : sequence of sequence of Reading
        Each segment's reads (:func:`read_segment`).

    Returns
    -------
    Track
        The fixes, one a second, and the lag.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the two sequences differ in length, or no read's clock parsed and was dated right.
    """
    if len(segments) != len(readings):
        raise InvalidInputError("segments, readings: expected one list of reads a segment")
    # Every read of a clock times the camera, whether or not its fix parsed too.
    timed: list[tuple[float, float, Segment, float, tuple[float, float, int] | None]] = []
    total = 0
    for segment, reads in zip(segments, readings, strict=True):
        total += len(reads)
        for r in reads:
            o = r.overlay
            if o.clock is None or o.clock.date() != segment.start.date():
                continue
            clock = d016.clock_seconds(o.clock)
            low = clock - segment.seconds - r.video_s
            value = None
            if o.latitude is not None and o.longitude is not None and o.mph is not None:
                value = (o.latitude, o.longitude, o.mph)
            timed.append((clock, low, segment, r.video_s, value))
    if not timed:
        raise InvalidInputError("readings: no read of a clock dated as its segment")
    lag = shared_middle([c[1] for c in timed])
    kept = [c for c in timed if abs(c[1] + 0.5 - lag) <= CLOCK_SLACK]
    seconds: dict[float, list[tuple[Segment, float, tuple[float, float, int]]]] = {}
    for clock, _, segment, video_s, value in kept:
        if value is not None:
            seconds.setdefault(clock, []).append((segment, video_s, value))
    fixes = []
    for clock in sorted(seconds):
        group = seconds[clock]
        (lat, lon, mph), agree = Counter(value for _, _, value in group).most_common(1)[0]
        first, tick, _ = min(group, key=lambda g: (g[0].seconds, g[1]))
        fixes.append(Fix(clock, lat, lon, mph, agree, len(group), first.path.name, tick))
    return Track(fixes, lag, total, len(kept))


def read(segments: Sequence[Segment], *, every: float = EVERY, jobs: int = 4) -> Track:
    """Read a camera's overlay over its segments: :func:`read_segment` each, then :func:`track`.

    Parameters
    ----------
    segments : sequence of Segment
        One camera's segments, in order.
    every : float, optional
        The time between the frames read, in seconds.
    jobs : int, optional
        Tesseract processes run at once.

    Returns
    -------
    Track
        The camera's fixes and lag.
    """
    return track(segments, [read_segment(s, every=every, jobs=jobs) for s in segments])
