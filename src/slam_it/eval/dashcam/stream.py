"""A segment's GPS records, read from its data stream with their video times.

The D016 writes a 160-byte record about each second into a data stream beside the video
(:func:`slam_it.eval.dashcam.d016.decode_record`). A record carries no presentation time, so it
is given the time of the video where it lies in the file (:func:`slam_it.eval.video.data_packets`).
Its speed and course are a second, independent source beside the overlay's; its time is the
receiver's UTC, whose offset from the overlay's local clock is not assumed.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from slam_it.eval import video
from slam_it.eval.dashcam import d016


@dataclass(frozen=True)
class Timed:
    """A record and where it lies in its segment.

    Attributes
    ----------
    video_s : float
        The segment's video time where the record lies, in seconds since its first frame.
    record : slam_it.eval.dashcam.d016.Record
        The record.
    """

    video_s: float
    record: d016.Record


def records(segment: str | Path) -> list[Timed]:
    """Return a segment's GPS records, in file order; packets of another size are skipped.

    Parameters
    ----------
    segment : str or pathlib.Path
        A D016 segment.

    Returns
    -------
    list of Timed
        Each record with its video time.

    Raises
    ------
    FileNotFoundError
        If the segment does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it is not a readable MPEG-TS video.
    """
    return [
        Timed(time, d016.decode_record(raw))
        for time, raw in video.data_packets(segment)
        if len(raw) == d016.RECORD_SIZE
    ]
