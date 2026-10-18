"""Reading a dashcam's burned-in overlay: the track from the reads, and Tesseract on a segment."""

from __future__ import annotations

import datetime as dt
import math
import shutil
from pathlib import Path

import numpy as np
import pytest
from PIL import Image, ImageDraw, ImageFont

from slam_it.eval.dashcam import d016, drives, overlay
from slam_it.foundations.errors import InvalidInputError
from tests.eval.dashcam_recordings import scene, segment_name, write_segment

START = dt.datetime(2026, 1, 2, 3, 4, 5)
"""The first segment's start."""


def _reading(video_s: float, clock: dt.datetime | None, lat: float = 12.5) -> overlay.Reading:
    first = f"N:{lat:.5f}W:123.45678 12mph"
    second = "" if clock is None else f"{clock:%Y/%m/%d %H:%M:%S}"
    return overlay.Reading(video_s, first, second, d016.parse_overlay(first, second))


def test_the_lag_is_where_most_reads_agree_and_misreads_are_dropped(tmp_path: Path) -> None:
    """The lag is the span most clock reads share; a misread clock or date is not kept."""
    segment = drives.Segment(tmp_path / segment_name(START, "F"), "F", START)
    lag = 2.43
    reads = []
    for k in range(25):  # five seconds, a read every 0.2 s
        t = 0.2 * k
        shown = START + dt.timedelta(seconds=math.floor(lag + t))
        reads.append(_reading(t, shown, lat=12.5 + 0.001 * math.floor(lag + t)))
    reads[7] = _reading(1.4, START + dt.timedelta(seconds=40))  # a misread clock
    reads[8] = _reading(1.6, START.replace(year=2025))  # a misread date
    reads[9] = _reading(1.8, None)  # unread
    read = overlay.track([segment], [reads])
    assert read.lag == pytest.approx(lag, abs=0.1)
    assert (read.reads, read.kept) == (25, 22)
    clocks = [f.clock - segment.seconds for f in read.fixes]
    assert clocks == [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert [f.latitude for f in read.fixes] == pytest.approx([12.502 + 0.001 * k for k in range(6)])
    assert read.fixes[1].tick == pytest.approx(0.6)  # 2.43 + 0.6 is the first past 3
    with pytest.raises(InvalidInputError, match="no read of a clock"):
        overlay.track([segment], [[_reading(0.0, None)]])
    with pytest.raises(InvalidInputError, match="one list"):
        overlay.track([segment], [])


def test_the_shared_middle_is_the_span_most_intervals_hold() -> None:
    """The middle is that of the deepest span of the unit intervals, an outlier aside."""
    assert overlay.shared_middle([0.0, 0.2, 0.4]) == pytest.approx(0.7)
    assert overlay.shared_middle([0.0, 0.2, 5.0]) == pytest.approx(0.6)
    with pytest.raises(InvalidInputError, match="none"):
        overlay.shared_middle([])


def test_white_is_bright_in_every_channel_and_nearly_grey() -> None:
    """A pixel is the overlay's white only if bright in every channel and nearly grey."""
    pixels = np.array([[[255, 255, 255], [250, 210, 205], [255, 255, 120], [180, 180, 180]]])
    assert overlay.white(pixels.astype(np.uint8)).tolist() == [[True, True, False, False]]


SIZE = (640, 360)
BOX = (10, 290, 440, 64)
"""A small frame's size, and its overlay's box, for the test's segments."""


def _burned(rng: np.random.Generator, text: tuple[str, str]) -> np.ndarray:
    picture = Image.fromarray(np.repeat(scene(rng, SIZE[1], SIZE[0])[:, :, None], 3, axis=2))
    draw = ImageDraw.Draw(picture)
    font = ImageFont.load_default(size=22)
    draw.text((BOX[0] + 6, BOX[1] + 4), text[0], fill=(255, 255, 255), font=font)
    draw.text((BOX[0] + 6, BOX[1] + 34), text[1], fill=(255, 255, 255), font=font)
    return np.asarray(picture)


@pytest.mark.skipif(shutil.which("tesseract") is None, reason="the Tesseract program is missing")
def test_a_segments_overlay_is_read_into_fixes_and_the_lag(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Tesseract reads a burned-in overlay into a fix a second, and the camera's lag."""
    pytest.importorskip("pytesseract")
    monkeypatch.setitem(d016.OVERLAY_BOXES, SIZE, BOX)
    lag, rate = 2.47, 5
    rng = np.random.default_rng(5)
    segments = []
    for start in (START, START + dt.timedelta(seconds=3)):
        pictures = []
        for k in range(3 * rate):
            shown = math.floor(lag + (start - START).total_seconds() + k / rate)
            lat = 12.34567 + 0.0001 * shown
            clock = START + dt.timedelta(seconds=shown)
            text = (f"N:{lat:.5f} W:123.45678 {20 + shown}mph", f"{clock:%Y/%m/%d %H:%M:%S}")
            pictures.append(_burned(rng, text))
        path = write_segment(tmp_path / segment_name(start, "F"), pictures, rate=rate)
        segments.append(drives.Segment(path, "F", start))
    read = overlay.read(segments, every=1 / rate, jobs=2)
    assert read.lag == pytest.approx(lag, abs=0.5 / rate)  # to half the time between reads
    assert read.kept >= 0.6 * read.reads
    seconds = [round(f.clock - segments[0].seconds) for f in read.fixes]
    assert set(seconds) <= set(range(2, 9))
    assert len(seconds) >= 4  # a synthetic font's fix line reads less often than its clock
    for fix, shown in zip(read.fixes, seconds, strict=True):
        assert fix.latitude == pytest.approx(12.34567 + 0.0001 * shown, abs=1e-9)
        assert fix.longitude == pytest.approx(-123.45678)
        assert fix.mph == 20 + shown
