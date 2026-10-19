"""The Terunsoul D016 dashcam's formats: its files, its burned-in overlay, its GPS records.

A product's formats, as its recordings show them, not anyone's data:

- **Files.** Each camera records a drive as short MPEG-TS segments (1 or 5 minutes, as set),
  ``video/F/YYYYMMDD_HHMMSSF.ts`` for the front camera and ``video/R/...R.ts`` for the rear,
  named by their start on the camera's local clock (:data:`NAME`). A front segment and its rear
  one start within a few seconds of each other.
- **The overlay**, bottom left of every frame, two lines in a font of a fixed pixel size (so its
  box does not scale with the frame, :data:`OVERLAY_BOXES`)::

      N:12.34567 W:123.45678 12mph
      2026/01/02 03:04:05

  the fix, its speed in mph, and the camera's local clock. Each camera burns in its own clock,
  and the two channels' differ, so :mod:`slam_it.eval.dashcam.overlay` measures each against
  its segments' video time.
- **The GPS records**, a data stream in each segment (stream 2, ``bin_data``), one 160-byte
  record about each second (:func:`decode_record`): the receiver's UTC time, a fix flag, the
  speed in knots and the course over ground. Its latitude and longitude are scrambled (they move
  where the true fix does not), so they are not offered; the overlay's are the readable ones.
"""

from __future__ import annotations

import calendar
import datetime as dt
import math
import re
import struct
from dataclasses import dataclass

from slam_it.foundations.errors import InvalidInputError

NAME = re.compile(r"(?P<date>\d{8})_(?P<time>\d{6})(?P<camera>[FR])\.ts")
"""A segment's file name: its start on the camera's local clock, and its camera."""

FOLDERS = {"F": ("video", "F"), "R": ("video", "R")}
"""Each camera's folder under a card's root (or a copy of it)."""

OVERLAY_BOXES = {(3840, 2160): (40, 1840, 1040, 150), (2560, 1440): (30, 1190, 960, 150)}
"""The overlay's box ``(x, y, w, h)``, in pixels, per frame size ``(width, height)``: placed
by eye on a frame of each size. Its font is a fixed pixel size, so a size not listed needs its
own box."""

OVERLAY_CHARACTERS = "0123456789.:/NSEWmph"
"""Every character the overlay prints but the space: the OCR's whitelist."""

FIX_LINE = re.compile(r"([NS]):(\d{1,2}\.\d{5})([EW]):(\d{1,3}\.\d{5})(\d{1,3})mph")
"""The overlay's first line with its spaces removed: the fix and the speed in mph."""

CLOCK_LINE = re.compile(r"(\d{4})/(\d{2})/(\d{2})(\d{2}):(\d{2}):(\d{2})")
"""The overlay's second line with its spaces removed: the local date and time."""

RECORD_SIZE = 160
"""A GPS record's bytes."""

KNOT = 1852.0 / 3600.0
"""A knot, in metres per second: the records' speed unit (1 knot = 1.15078 mph)."""

MPH = 0.44704
"""A mile an hour, in metres per second: the overlay's speed unit."""


def clock_seconds(moment: dt.datetime) -> float:
    """Return a naive local time as seconds, read as if it were UTC.

    The camera's clock is local and its zone unrecorded, so times are kept on that clock, never
    converted: this is a number to subtract and compare, not an instant.

    Parameters
    ----------
    moment : datetime.datetime
        A naive local time.

    Returns
    -------
    float
        Its seconds since 1970-01-01 00:00 on the same clock.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `moment` carries a zone.
    """
    if moment.tzinfo is not None:
        raise InvalidInputError("moment: expected a naive local time")
    return calendar.timegm(moment.timetuple()) + moment.microsecond / 1e6


def segment_start(name: str) -> tuple[str, dt.datetime]:
    """Return a segment's camera and start from its file name (:data:`NAME`).

    Parameters
    ----------
    name : str
        The file's name, e.g. ``20260102_030405F.ts``.

    Returns
    -------
    tuple
        ``"F"`` or ``"R"``, and the start as a naive local time.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the name is not a segment's, or its date or time is not one.
    """
    match = NAME.fullmatch(name)
    if match is None:
        raise InvalidInputError(f"{name}: not a D016 segment's name (YYYYMMDD_HHMMSS[F|R].ts)")
    try:
        start = dt.datetime.strptime(match["date"] + match["time"], "%Y%m%d%H%M%S")
    except ValueError:
        raise InvalidInputError(f"{name}: no such date or time") from None
    return match["camera"], start


def overlay_box(width: int, height: int) -> tuple[int, int, int, int]:
    """Return the overlay's box on a frame of this size (:data:`OVERLAY_BOXES`).

    Parameters
    ----------
    width, height : int
        The frame's size.

    Returns
    -------
    tuple of int
        ``(x, y, w, h)``, in pixels.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If no box is known for the size.
    """
    box = OVERLAY_BOXES.get((width, height))
    if box is None:
        known = ", ".join(f"{w}x{h}" for w, h in OVERLAY_BOXES)
        raise InvalidInputError(f"no overlay box for {width}x{height} frames (known: {known})")
    return box


@dataclass(frozen=True)
class Overlay:
    """What one frame's overlay said, the lines read as far as they parsed.

    Attributes
    ----------
    latitude, longitude : float or None
        The fix, in degrees (north and east positive); None if the first line did not parse.
    mph : int or None
        The speed, in miles an hour.
    clock : datetime.datetime or None
        The local date and time; None if the second line did not parse or is no date.
    """

    latitude: float | None
    longitude: float | None
    mph: int | None
    clock: dt.datetime | None


def parse_overlay(first: str, second: str) -> Overlay:
    """Parse an overlay's two lines, as read, their spaces ignored.

    A field out of range (a latitude past 90°, a speed of 130 mph or more, a 13th month, a 60th
    second) counts as not read: a misread digit is more likely than such a value.

    Parameters
    ----------
    first, second : str
        The lines as read.

    Returns
    -------
    Overlay
        The fields that parsed.
    """
    fix = FIX_LINE.search(first.replace(" ", ""))
    latitude = longitude = None
    mph = None
    if fix is not None:
        lat, lon, speed = float(fix[2]), float(fix[4]), int(fix[5])
        if lat <= 90.0 and lon <= 180.0 and speed < 130:
            latitude = lat if fix[1] == "N" else -lat
            longitude = lon if fix[3] == "E" else -lon
            mph = speed
    clock = None
    when = CLOCK_LINE.search(second.replace(" ", ""))
    if when is not None:
        try:
            year, month, day, hour, minute, seconds = (int(v) for v in when.groups())
            clock = dt.datetime(year, month, day, hour, minute, seconds)
        except ValueError:
            clock = None
    return Overlay(latitude, longitude, mph, clock)


@dataclass(frozen=True)
class Record:
    """One GPS record of the data stream.

    Attributes
    ----------
    utc : datetime.datetime or None
        The receiver's time, naive UTC; None if the record's date is not one.
    valid : bool
        Whether the receiver had a fix (``A``) or not (``V``).
    speed : float
        The speed over ground, in metres per second (recorded in knots).
    course : float
        The course over ground, in degrees.
    """

    utc: dt.datetime | None
    valid: bool
    speed: float
    course: float


def decode_record(raw: bytes) -> Record:
    """Decode one 160-byte GPS record.

    Its layout, little-endian: six int32, the hour, minute and second (UTC) then the year, month
    and day, at offset 0; the fix flag (``A`` or ``V``) at 0x18; the hemispheres at 0x19 and
    0x1a; two float64 at 0x20 and 0x28, a latitude and longitude scrambled, not read; then two
    float32 at 0x30 and 0x34, the speed in knots and the course in degrees.

    Parameters
    ----------
    raw : bytes
        The record, :data:`RECORD_SIZE` bytes.

    Returns
    -------
    Record
        The record's time, fix flag, speed and course.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the record is not :data:`RECORD_SIZE` bytes, or its speed or course is not finite.
    """
    if len(raw) != RECORD_SIZE:
        raise InvalidInputError(f"record: expected {RECORD_SIZE} bytes, got {len(raw)}")
    hour, minute, second, year, month, day = struct.unpack_from("<6i", raw, 0)
    speed, course = struct.unpack_from("<2f", raw, 0x30)
    if not (math.isfinite(speed) and math.isfinite(course)):
        raise InvalidInputError("record: a speed or course that is not finite")
    try:
        utc: dt.datetime | None = dt.datetime(year, month, day, hour, minute, second)
    except ValueError:
        utc = None
    return Record(utc, raw[0x18:0x19] == b"A", float(speed) * KNOT, float(course))
