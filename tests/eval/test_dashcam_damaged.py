"""A damaged segment among a drive's: skipped with a warning by every step, never the drive.

Damaged (:class:`slam_it.eval.video.DamagedSegmentError`): a file that does not open as
MPEG-TS, or whose reading raises partway; a segment whose decoder conceals a few corrupt frames
is not, and is kept.
"""

from __future__ import annotations

import datetime as dt
import json
import warnings
from collections.abc import Iterator
from pathlib import Path

import av
import numpy as np
import pytest

from slam_it.eval import lenses, tum, video
from slam_it.eval.dashcam import calibrate, d016, drives, ingest, mask, overlay
from slam_it.foundations.errors import InvalidInputError
from tests.eval.dashcam_recordings import card

START = dt.datetime(2026, 1, 2, 3, 4, 5)
"""The first segment's start."""

MINUTE = dt.timedelta(minutes=1)

CAMERA = lenses.Pinhole(150.0, 150.0, 79.5, 44.5, 160, 90)
"""The synthetic segments' camera, at their 160x90."""


def _front(root: Path, damaged: tuple[int, ...]) -> tuple[drives.Segment, ...]:
    """Write three front segments a minute apart, those numbered in `damaged` as garbage."""
    paths = card(root, "F", [START + k * MINUTE for k in range(3)], frames=10, rate=5)
    for k in damaged:
        paths[k].write_bytes(np.random.default_rng(k).integers(0, 256, 5000, np.uint8).tobytes())
    return drives.find_drives(drives.scan(root))[0].front


def test_a_damaged_segment_among_good_ones_is_skipped_with_a_warning(tmp_path: Path) -> None:
    """The import goes on past a damaged segment, warns of it, and lists it as skipped."""
    front = _front(tmp_path / "card", damaged=(1,))
    out = tmp_path / "out"
    with pytest.warns(video.SkippedSegmentWarning, match=front[1].path.name):
        skipped = ingest.import_drive(front, out, CAMERA, lag=0.0)
    assert [name for name, _ in skipped] == [front[1].path.name]
    assert "not a readable MPEG-TS segment" in skipped[0][1]
    frames = tum.read_frames(out)
    starts = [d016.clock_seconds(START + k * MINUTE) for k in (0, 2)]
    expected = [s + k / 5 for s in starts for k in range(10)]
    assert frames.timestamps == pytest.approx(expected, abs=1e-5)
    source = json.loads((out / "source.json").read_text())
    assert source["skipped"] == [{"segment": name, "reason": why} for name, why in skipped]
    assert source["segments"] == [s.path.name for s in front]


def test_a_segment_failing_partway_is_taken_back_whole(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Frames a segment gave before it failed are removed with it, files and all."""
    front = _front(tmp_path / "card", damaged=())
    decode = video._decode_segment

    def failing(path: Path, limits: video.Limits) -> Iterator[tuple[float, av.VideoFrame]]:
        for k, decoded in enumerate(decode(path, limits)):
            if path == front[1].path and k == 4:
                raise video.DamagedSegmentError(f"{path.name}: undecodable (a test's)")
            yield decoded

    monkeypatch.setattr(video, "_decode_segment", failing)
    out = tmp_path / "out"
    with pytest.warns(video.SkippedSegmentWarning, match="a test's"):
        ingest.import_drive(front, out, CAMERA, lag=0.0)
    frames = tum.read_frames(out)
    assert len(frames.timestamps) == 20
    assert sorted(p.name for p in (out / "rgb").iterdir()) == sorted(p.name for p in frames.images)
    source = json.loads((out / "source.json").read_text())
    assert source["frames"] == {"decoded": 20, "kept": 20}


def test_every_segment_damaged_fails_and_leaves_nothing(tmp_path: Path) -> None:
    """With nothing left to import, the import fails, naming why, and writes no folder."""
    front = _front(tmp_path / "card", damaged=(0, 1, 2))
    with (
        pytest.warns(video.SkippedSegmentWarning),
        pytest.raises(InvalidInputError, match="every one is damaged"),
    ):
        ingest.import_drive(front, tmp_path / "out", CAMERA, lag=0.0)
    assert not (tmp_path / "out").exists()


def test_a_segment_with_corrupt_frames_still_decodes_and_is_kept(tmp_path: Path) -> None:
    """Zeroed payload bytes: the decoder conceals what it cannot decode; nothing is skipped."""
    front = _front(tmp_path / "card", damaged=())
    raw = bytearray(front[1].path.read_bytes())
    packets = len(raw) // 188  # MPEG-TS packets, each a 4-byte header and its payload
    for k in range(packets // 3, packets // 3 + 5):
        raw[k * 188 + 8 : k * 188 + 108] = bytes(100)
    front[1].path.write_bytes(bytes(raw))
    with warnings.catch_warnings():
        warnings.simplefilter("error", video.SkippedSegmentWarning)
        skipped = ingest.import_drive(front, tmp_path / "out", CAMERA, lag=0.0)
    assert skipped == []
    assert len(tum.read_frames(tmp_path / "out").timestamps) > 20


def test_the_mask_skips_a_damaged_segment(tmp_path: Path) -> None:
    """The mask is measured from the segments that read; with none, it is refused."""
    front = _front(tmp_path / "card", damaged=(1,))
    paths = [s.path for s in front]
    with pytest.warns(video.SkippedSegmentWarning, match=paths[1].name):
        static = mask.measure(paths, every=0.2, width=160, min_size=20, close=2, grow=1)
    assert static.shape == (90, 160)
    with (
        pytest.warns(video.SkippedSegmentWarning),
        pytest.raises(InvalidInputError, match="every one is damaged"),
    ):
        mask.measure([paths[1]])


def test_the_overlay_skips_a_damaged_segment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The overlay is read from the segments that open, and the skipped are named."""
    front = _front(tmp_path / "card", damaged=(1,))
    monkeypatch.setitem(d016.OVERLAY_BOXES, (160, 90), (0, 60, 160, 30))
    shown = f"{START:%Y/%m/%d %H:%M:%S}"
    monkeypatch.setattr(overlay, "ocr", lambda _: ("N:12.34567W:123.45678 12mph", shown))
    with pytest.warns(video.SkippedSegmentWarning, match=front[1].path.name):
        read = overlay.read(front, jobs=1)
    assert [name for name, _ in read.skipped] == [front[1].path.name]
    assert read.reads == 20
    with (
        pytest.warns(video.SkippedSegmentWarning),
        pytest.raises(InvalidInputError, match="every one is damaged"),
    ):
        overlay.read(front[1:2], jobs=1)


def test_the_calibration_frames_skip_a_damaged_segment(tmp_path: Path) -> None:
    """Frames are extracted from the segments that read, and the folder holds those alone."""
    front = _front(tmp_path / "card", damaged=(1,))
    folder = tmp_path / "frames"
    with pytest.warns(video.SkippedSegmentWarning, match=front[1].path.name):
        names = calibrate.extract_frames(front, folder, lag=0.0, fps=5.0, width=160)
    assert len(names) == 20
    assert sorted(p.name for p in folder.iterdir()) == sorted(names)
    with (
        pytest.warns(video.SkippedSegmentWarning),
        pytest.raises(InvalidInputError, match="every one is damaged"),
    ):
        calibrate.extract_frames(front[1:2], tmp_path / "none", lag=0.0, fps=5.0, width=160)
