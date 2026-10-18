"""Importing one camera's share of a drive as one TUM-format sequence.

A drive's segments are decoded straight through, in order, and each frame timed on the camera's
overlay clock: its segment's start (its name) plus the camera's lag (:mod:`.overlay`) plus its
time since the segment's first frame. So the run's frames and the GPS track
(:mod:`slam_it.eval.gnss`) share one clock, and a gap between segments stays a gap. The frames,
the lens and the static mask are written as :func:`slam_it.eval.video.import_segments` writes
them; a local recording has no upload cap, only :data:`LIMITS`, which the caller can widen.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import lenses, video
from slam_it.eval.dashcam.drives import Segment
from slam_it.foundations.errors import InvalidInputError

LIMITS = video.Limits(video.SEGMENT_SECONDS, max_frames=20_000, short_side=video.SHORT_SIDE)
"""A drive import's limits: a segment of up to 10 minutes (the D016 records 1 or 5), 20,000
frames in all (11 minutes at 30 fps), the frames' short side an upload's. A longer import is
asked for by widening them, as the command's ``--max-frames`` does."""


def import_drive(  # noqa: PLR0913 -- the segments, the folder, the lens; how, keyword-only
    segments: Sequence[Segment],
    out: str | Path,
    lens: lenses.Pinhole | lenses.Fisheye | lenses.RadTan,
    *,
    lag: float,
    mask: NDArray[np.uint8] | None = None,
    limits: video.Limits = LIMITS,
) -> list[tuple[str, str]]:
    """Import one camera's segments as one sequence (the module docstring).

    A damaged segment is skipped, with a warning (:func:`slam_it.eval.video.import_segments`).

    Parameters
    ----------
    segments : sequence of Segment
        One camera's segments, in time order.
    out : str or pathlib.Path
        The folder written: new or empty.
    lens : slam_it.eval.lenses.Pinhole, Fisheye or RadTan
        The camera's lens at the segments' stored size (a calibration's, as the config holds it).
    lag : float
        The camera's overlay clock less its segments' names', in seconds (:mod:`.overlay`).
    mask : ndarray of uint8, shape (H, W), optional
        The static mask at the stored size, 0 where features are dropped (:mod:`.mask`).
    limits : slam_it.eval.video.Limits, optional
        The longest segment, the most frames in all, and the frames' short side.

    Returns
    -------
    list of tuple of str
        Each damaged segment skipped: its file name, and why.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are no segments, they are of more than one camera, every one is damaged, or
        the import refuses them (:func:`slam_it.eval.video.import_segments`).
    """
    if not segments:
        raise InvalidInputError("segments: none given")
    cameras = {s.camera for s in segments}
    if len(cameras) != 1:
        raise InvalidInputError(f"segments: expected one camera's, got {sorted(cameras)}")
    ordered = sorted(segments, key=lambda s: s.start)
    return video.import_segments(
        [s.path for s in ordered],
        [s.seconds + lag for s in ordered],
        out,
        lenses.Chosen(lens, "given"),
        limits=limits,
        mask=mask,
        source={"kind": "dashcam", "profile": "d016", "camera": ordered[0].camera, "lag": lag},
    )
