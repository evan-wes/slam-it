"""A damaged segment among a drive's: skipped with a warning by every step, never the drive.

Damaged (:class:`slam_it.eval.video.DamagedSegmentError`): a file that does not open as
MPEG-TS, or whose reading raises partway; a segment whose decoder conceals a few corrupt frames
is not, and is kept. Nor is one whose read says "try again" (EAGAIN), as FFmpeg's does at a
ragged end: it is read again, to its last frame, by every reader, an upload's too.
"""

from __future__ import annotations

import datetime as dt
import errno
import json
import warnings
from collections.abc import Callable, Collection, Iterator
from dataclasses import dataclass, field
from pathlib import Path

import av
import numpy as np
import pytest

from slam_it.eval import lenses, tum, video
from slam_it.eval.dashcam import calibrate, d016, drives, ingest, mask, overlay
from slam_it.foundations.errors import InvalidInputError
from tests.eval.dashcam_recordings import card, hooded, record, scene, write_segment
from tests.eval.recordings import encode

START = dt.datetime(2026, 1, 2, 3, 4, 5)
"""The first segment's start."""

MINUTE = dt.timedelta(minutes=1)

SECOND = dt.timedelta(seconds=1)

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


def _segment_frames(path: Path) -> list[tuple[float, bytes]]:
    """Return a segment's frames, every one, as the dashcam's steps sample them, with times."""
    return [(time, picture.tobytes()) for time, picture in video.sample_frames(path, every=0.0)]


def _upload_frames(path: Path) -> list[tuple[float, bytes]]:
    """Return an upload's frames, as its import decodes them, with times."""
    return [(time, frame.to_ndarray().tobytes()) for time, frame in video._decode(path)]


READERS: dict[str, tuple[str, Callable[[Path], list[tuple[float, bytes]]]]] = {
    "a segment's frames": ("_open_segment", _segment_frames),
    "a segment's data packets": ("_open_segment", video.data_packets),
    "an upload's frames": ("_open", _upload_frames),
}
"""Each reader of a recording: the opener in :mod:`slam_it.eval.video` it reads through, and
what it reads, with times."""


def _recording(folder: Path, *, upload: bool) -> Path:
    """Write a recording of 10 frames: a phone's video, or a segment with 3 GPS records."""
    pictures = [hooded(scene(np.random.default_rng(k), 90, 160)) for k in range(10)]
    if upload:
        return encode(folder / "phone.mov", pictures)
    records = {k: record(START + k * SECOND) for k in (0, 4, 9)}
    return write_segment(folder / "segment.ts", pictures, rate=5, records=records)


class _SaysTryAgain:
    """A real container whose read says "try again" (EAGAIN) where told to, as FFmpeg's may.

    Its demuxing raises :class:`av.error.BlockingIOError` before reading each packet whose index,
    counted through the file from 0, is in `before`, or before every read if `always`. Nothing
    is lost, as nothing is in FFmpeg: the next demuxing reads on where the file was left.
    """

    def __init__(
        self, container: av.container.InputContainer, before: Collection[int], *, always: bool
    ) -> None:
        """Wrap `container`, nothing read yet: `read` packets read, `full` of them not empty."""
        self.container, self.before, self.always = container, set(before), always
        self.read = self.full = self.said = 0

    @property
    def streams(self) -> av.container.streams.StreamContainer:
        """Return the real container's streams."""
        return self.container.streams

    def demux(self, *streams: av.stream.Stream) -> Iterator[av.Packet[av.stream.Stream]]:
        """Yield the real container's packets, saying "try again" where told to."""
        packets = self.container.demux(*streams)
        while True:
            if self.always or self.read in self.before:
                self.before.discard(self.read)
                self.said += 1
                raise av.error.BlockingIOError(errno.EAGAIN, "Resource temporarily unavailable")
            packet = next(packets, None)
            if packet is None:
                return
            self.read += 1
            self.full += packet.size > 0
            yield packet

    def __enter__(self) -> _SaysTryAgain:
        """Enter, as a container does."""
        return self

    def __exit__(self, *_: object) -> None:
        """Close the real container."""
        self.container.close()


@dataclass
class _Plan:
    """Where the containers opened say "try again", and each one made, in order."""

    before: set[int] = field(default_factory=set)
    always: bool = False
    made: list[_SaysTryAgain] = field(default_factory=list)


def _saying_try_again(monkeypatch: pytest.MonkeyPatch, opener: str) -> _Plan:
    """Wrap each container `opener` opens in a :class:`_SaysTryAgain`, as the returned plan says."""
    real = getattr(video, opener)
    plan = _Plan()

    def opened(path: Path, **options: bool) -> _SaysTryAgain:
        plan.made.append(_SaysTryAgain(real(path, **options), plan.before, always=plan.always))
        return plan.made[-1]

    monkeypatch.setattr(video, opener, opened)
    return plan


def test_a_segment_whose_last_read_says_try_again_is_read_to_its_last_frame(
    tmp_path: Path,
) -> None:
    """A ragged end, bytes with no sync byte after the last packet: kept, every frame decoded.

    FFmpeg's read says "try again" (EAGAIN) there, before its parser has given the last frame,
    and PyAV's own decoding stops at it, short of the frames its threads hold.
    """
    front = _front(tmp_path / "card", damaged=())
    pictures = [hooded(scene(np.random.default_rng(7), 90, 160)) for _ in range(10)]
    whole = write_segment(tmp_path / "whole.ts", pictures, rate=5, lengths=True)
    ragged = front[1].path
    ragged.write_bytes(whole.read_bytes() + bytes(188))  # a TS packet's worth, no sync byte
    decoded: list[av.VideoFrame] = []
    with av.open(str(ragged), format="mpegts") as container:
        container.streams.video[0].thread_type = "AUTO"
        with pytest.raises(av.error.BlockingIOError):
            decoded.extend(container.decode(video=0))  # each frame given before it raises
    assert len(decoded) < 10
    expected = _segment_frames(whole)
    assert len(expected) == 10
    assert _segment_frames(ragged) == expected
    with warnings.catch_warnings():
        warnings.simplefilter("error", video.SkippedSegmentWarning)
        assert ingest.import_drive(front, tmp_path / "out", CAMERA, lag=0.0) == []
    assert len(tum.read_frames(tmp_path / "out").timestamps) == 30


@pytest.mark.parametrize("reader", READERS)
def test_reads_that_say_try_again_lose_nothing(
    reader: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """EAGAIN before the first read, mid-stream, before the last packet and at the end.

    Each is read again: everything is read, as when no read says it.
    """
    opener, read = READERS[reader]
    path = _recording(tmp_path, upload=opener == "_open")
    plan = _saying_try_again(monkeypatch, opener)
    expected = read(path)
    assert len(expected) == (3 if "data" in reader else 10)
    full = plan.made[-1].full
    plan.before = {0, full // 2, full - 1, full}
    assert read(path) == expected
    assert plan.made[-1].said == 4


@pytest.mark.parametrize("reader", READERS)
def test_a_read_that_keeps_saying_try_again_is_given_up(
    reader: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A read going nowhere is not read again forever: past the retries, the error stands."""
    opener, read = READERS[reader]
    path = _recording(tmp_path, upload=opener == "_open")
    plan = _saying_try_again(monkeypatch, opener)
    plan.always = True
    refused = InvalidInputError if opener == "_open" else video.DamagedSegmentError
    with pytest.raises(refused, match=r"\[Errno 11\]") as caught:
        read(path)
    assert isinstance(caught.value, video.DamagedSegmentError) == (opener == "_open_segment")
    assert plan.made[-1].said == video._READ_RETRIES + 1
