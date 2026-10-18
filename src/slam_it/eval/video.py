"""Importing a phone's recording into a TUM-format folder, for the pipeline to run on.

Two kinds of upload (docs/design/2026-10-09-run-viewer.md, section 5):

- **A video** (:func:`import_video`), from any phone, its intrinsics chosen from its metadata
  (:func:`slam_it.eval.lenses.choose`) or given.
- **A Stray Scanner recording** (:func:`import_stray`): the zip Stray Scanner's Share button
  makes on an iPhone or iPad with LiDAR. Its video, its camera matrix (exact intrinsics),
  ARKit's pose for each frame, which becomes the run's ground truth, and, if it holds them, its
  LiDAR depth maps, which make it an RGB-D recording.

Either becomes a TUM-format folder (:mod:`slam_it.eval.tum`), which the readers in all three
languages already take: ``rgb/`` and ``rgb.txt``, ``camera.txt`` (a pinhole), for a Stray
Scanner recording ``groundtruth.txt`` and, with depth, ``depth/`` and ``depth.txt``, and
``source.json``, what the import used. The folder must be new or empty; it is written beside,
in a hidden ``.<name>.<id>.importing`` folder, and moved onto it once whole, so an import that
fails leaves nothing (:func:`slam_it.eval.tum.staged`, which every import to TUM format
shares).

**Frames.**

- Decoded with PyAV (FFmpeg): this is the one module that imports it, by a contract.
- Turned upright: the rotation the file declares (a phone stores a portrait video on its side)
  is applied, and the intrinsics are turned with it.
- Timed by each frame's presentation time, since a phone's frame rate varies. A frame closer
  than :data:`MIN_SPACING` to the last one kept is skipped, so 60 fps becomes 30.
- Scaled down, never up, so that the short side is :data:`SHORT_SIDE` pixels as TUM's and
  EuRoC's are (16:9 becomes 854 by 480), the intrinsics scaled with them.
- Kept in colour, as PNG (:func:`slam_it.eval.images.write_png`).

**Caps.** An upload over :data:`MAX_BYTES` or a video over :data:`MAX_SECONDS` is refused, and
so is a video of more than :data:`MAX_DECODED` frames, or whose frames are longer than
:data:`MAX_LONG_SIDE`, more than :data:`MAX_ASPECT` to 1, or not all of one size. A frame
written is then at most :data:`MAX_ASPECT` times :data:`SHORT_SIDE` long.

**A dataset's video** (:mod:`slam_it.eval.advio`) is imported by :func:`import_video` with its
own :class:`Limits`, a longer video and another short side, and may give each frame's time
from the dataset's own clock, so that its ground truth's times match.

**A dashcam's segments** (:mod:`slam_it.eval.dashcam`), local recordings, never uploads, are
MPEG-TS files, opened with FFmpeg's MPEG-TS demuxer alone and files as the only protocol
(:data:`_SEGMENT_DEMUXER`). They are sampled in
time (:func:`sample_frames`), their data stream's packets read (:func:`data_packets`), or
joined, in order, into one sequence (:func:`import_segments`): each frame timed by its
segment's start on the caller's clock plus its time in the segment, and written as a video's
are, within the caller's :class:`Limits` and with no upload cap. A card holds the odd damaged
segment (:class:`DamagedSegmentError`), which the join skips whole, with a warning.

**Containers.** An upload is opened with FFmpeg's mov/mp4 demuxer alone, which phones' videos
and Stray Scanner's are in, and with files as its only protocol. Any other container is
refused, so the playlist and reference formats that open other files (``hls``, ``concat``,
``dash``) never read an upload.

**Intrinsics** (:class:`slam_it.eval.lenses.Chosen`) describe the frames as the file
stores them, before the declared rotation, at the file's size, as a camera matrix or a lens
profile describes the sensor. A fisheye lens is undistorted to pinhole at the output size
(:func:`slam_it.eval.lenses.undistortion`), each frame's channels resampled by rung 0's
bilinear :func:`slam_it.foundations.image.remap`. A radial-tangential lens
(:class:`slam_it.eval.lenses.RadTan`, a dataset's calibration) is kept: its frames are written
as recorded, and ``camera.txt`` says ``radtan``, which the pipeline undistorts keypoint by
keypoint.

**Stray Scanner zips** are read in place, never unpacked under the names they contain. Only
``rgb.mp4``, ``camera_matrix.csv`` and ``odometry.csv`` are opened, and, for each frame the
import keeps, its LiDAR maps, ``depth/NNNNNN.png`` and ``confidence/NNNNNN.png`` (the frame's
index, six digits), each size checked against its own cap before and while it is read, and a
map's size, as it will decode, against :data:`STRAY_MAP_SIDE` before it is decoded, from its
bytes in memory; anything else is ignored. A name that is absolute, climbs out of a folder or
appears twice refuses the zip, and so does one the zip cannot give back whole. The CSVs are
parsed strictly: numbers only, and one ``odometry.csv`` row per video frame.

**Stray Scanner's depth.** A zip with depth maps imports as RGB-D: every frame kept needs its
depth and confidence maps, or the zip is refused; each pair is turned upright with its frame,
kept where ARKit's confidence is at least the import's threshold (high by default) and brought
to the frame's size (:func:`slam_it.eval.lidar.registered_depth`), and ``camera.txt`` takes
LiDAR's depth scale and ``bf``. A zip without depth maps imports as before, for a monocular
run.

**Stray Scanner's own conventions**, read in its source
(https://github.com/strayrobots/scanner, ``StrayScanner/Helpers/``):
- ``OdometryEncoder.swift`` writes each pose's rotation already in our camera's axes (x right, y
  down, z forwards): ARKit's camera rotation times a half turn about x
  (``q_WA * q_AC``, ``q_AC = (1, 0, 0, 0)``). So the poses are used as written.
- ``DatasetEncoder.swift`` and ``VideoEncoder.swift`` stamp the first frame at -1/60 s, before
  the video's session starts, which an MP4 hides behind an edit list. So ``rgb.mp4`` is decoded
  with its edit list ignored, and every frame has its ``odometry.csv`` row.

**Metadata.** ``source.json`` records the intrinsics' source and the metadata fields the import
read (:data:`METADATA_KEYS`), and whether a mask was given. Location tags and every other field
are never copied.

**A mask** (:func:`import_video`'s ``mask``): an 8-bit gray image at the video's stored size, 0
where no feature is to be tracked (a camera's burned-in overlay, the car's own body). It is
carried through the frames' own geometry and written as ``mask.png``
(:data:`slam_it.eval.tum.MASK_FILE`), which a run reads: scaled down as the frames are, a pixel
masked if any stored pixel under it is (:func:`_scaled_mask`); then, for a fisheye lens, each
written pixel taking the scaled mask's pixel nearest where the undistortion samples, masked if
that is outside it; then turned upright with the frames.

**Assumptions the test clips check** (the design doc's section 5, "Checked first"): that
PyAV's rotation is counter-clockwise on a real portrait clip, as it is on the files PyAV
writes; that a Stray Scanner zip holds its three files at its root or in one folder; that
``camera_matrix.csv`` describes the video at its stored size; that ``odometry.csv`` has the
columns ``timestamp, frame, x, y, z, qx, qy, qz, qw`` (others ignored); and that Stray
Scanner's conventions, read in its source, hold on a real recording.

Run as a program::

    python -m slam_it.eval.video VIDEO OUT [--source auto|profile|35mm|nominal]
        [--profiles DIR] [--mask PNG]
    python -m slam_it.eval.video VIDEO OUT --intrinsics FX FY CX CY [--mask PNG]
    python -m slam_it.eval.video RECORDING.zip OUT --stray [--min-confidence {0,1,2}]
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import re
import sys
import warnings
import zipfile
import zlib
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

import av
import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.eval import images, lenses, lidar, runs, tum
from slam_it.foundations import image, so3
from slam_it.foundations.arrays import as_image
from slam_it.foundations.errors import InvalidInputError

MAX_BYTES = 500 * 10**6
"""The largest upload, video or zip, in bytes."""

MAX_SECONDS = 60.0
"""The longest video, in seconds."""

SHORT_SIDE = 480
"""The short side frames are scaled down to, in pixels."""

MAX_LONG_SIDE = 7680
"""The longest frame side accepted, in pixels: a phone's 8K video."""

MAX_ASPECT = 3.0
"""The most a frame's long side may be its short side's multiple: with frames scaled to a
:data:`SHORT_SIDE` short side, a frame written is at most 1440 by 480."""

MAX_DECODED = round(MAX_SECONDS * 240)
"""The most frames a video may have: :data:`MAX_SECONDS` at 240 fps, a phone's fastest slow
motion. Frames out of time order are skipped, so this bounds the decoding, not the time."""


@dataclass(frozen=True)
class Limits:
    """What an import takes and makes: the longest video, its most frames, the short side.

    Attributes
    ----------
    max_seconds : float
        The longest video, in seconds.
    max_frames : int
        The most frames it may have.
    short_side : int
        The short side frames are scaled down to, in pixels.
    """

    max_seconds: float = MAX_SECONDS
    max_frames: int = MAX_DECODED
    short_side: int = SHORT_SIDE

    def __post_init__(self) -> None:
        """Refuse limits that would take nothing: none positive."""
        if not (self.max_seconds > 0.0 and self.max_frames >= 1 and self.short_side >= 1):
            raise InvalidInputError(f"limits: expected each positive, got {self}")


UPLOAD = Limits()
"""An upload's limits: :data:`MAX_SECONDS`, :data:`MAX_DECODED` and :data:`SHORT_SIDE`."""

SEGMENT_SECONDS = 600.0
"""The longest recorded segment sampled (:func:`sample_frames`), in seconds: a dashcam records
a few minutes a segment."""

MIN_SPACING = 1.0 / 40.0
"""The least time between two frames kept, in seconds: a 60 fps video keeps every other frame,
and a 30 fps one, its frame times jittering by a few milliseconds, keeps every frame."""

METADATA_KEYS = lenses.METADATA_KEYS
"""The metadata fields the import reads and records, the intrinsics chosen from them: the only
ones it ever copies."""

STRAY_CAPS = {"rgb.mp4": MAX_BYTES, "camera_matrix.csv": 4096, "odometry.csv": 64 * 2**20}
"""The Stray Scanner files the import opens, each with the most bytes it reads of it."""

STRAY_MAPS = {"depth": 2**20, "confidence": 2**20}
"""Stray Scanner's per-frame LiDAR maps, ``<kind>/NNNNNN.png``, each with the most bytes read of
one: ARKit's are 256x192, 96 KiB as raw 16-bit depth, and a PNG adds little to its raw size."""

STRAY_MAP_SIDE = 1024
"""The largest side a LiDAR map may have, checked before it is decoded."""

_FRAME_NUMBER = re.compile(r"[0-9]{6}")
"""A LiDAR map's frame number: six ASCII digits (``str.isdigit`` takes others, as ``²``)."""

ODOMETRY_COLUMNS = ("timestamp", "frame", "x", "y", "z", "qx", "qy", "qz", "qw")
"""The ``odometry.csv`` columns the import reads: ARKit's camera pose for each frame."""

_CHUNK = 1 << 20
"""The bytes copied at a time out of a zip."""

_DEMUXER = "mov"
"""The only demuxer an upload is opened with: FFmpeg's mov/mp4 family (the module docstring)."""

_OPEN_OPTIONS = {"protocol_whitelist": "file", "enable_drefs": "0"}
"""How an upload is opened: as a file alone, never following the references a MOV may hold."""

_SEGMENT_DEMUXER = "mpegts"
"""The demuxer a dashcam's segments are opened with (:func:`import_segments`): MPEG-TS, the
local recordings of :mod:`slam_it.eval.dashcam`, never an upload."""

_SEGMENT_OPTIONS = {"protocol_whitelist": "file"}
"""How a segment is opened: as a file alone."""


class DamagedSegmentError(InvalidInputError):
    """A recorded segment that cannot be read through: a damaged file.

    Raised when the segment does not open as MPEG-TS, has no video stream or no stated frame
    size, its demuxing or decoding raises, a frame has no presentation time, or it holds no
    frame at all. A decoder that conceals a corrupt frame and goes on raises nothing, and a
    file cut short decodes the frames it holds: neither is damage. A missing file, or one over
    the caller's limits, is the caller's to fix, not damage. The dashcam's steps skip a
    damaged segment (:func:`import_segments`).
    """


class SkippedSegmentWarning(UserWarning):
    """A damaged segment skipped (:class:`DamagedSegmentError`), with its name and why."""


_ZIP_ERRORS = (zipfile.BadZipFile, zlib.error, EOFError, RuntimeError, NotImplementedError)
"""What reading a zip's entry raises when the zip is corrupt, encrypted or compressed in a way
Python does not read."""

_REMAP_ORDER = ("cpp", "rust", "python")
"""The backends that undistort a frame, fastest first: one 854x480 channel takes 2.8 ms in C++,
4.4 ms in Rust and 30 ms in Python (median of 30 calls, measured 2026-10-11 on the desktop this
was built on). All three give the same bytes, as rung 0's parity tests hold."""


def output_size(width: int, height: int, short_side: int = SHORT_SIDE) -> tuple[int, int]:
    """Return the size frames are scaled to: the short side `short_side`, never larger.

    Parameters
    ----------
    width, height : int
        The frame's size.
    short_side : int, optional
        The short side scaled to (default: :data:`SHORT_SIDE`).

    Returns
    -------
    tuple of int
        The width and height, the long side rounded to an even number.
    """
    short, long = min(width, height), max(width, height)
    if short <= short_side:
        return width, height
    scaled = 2 * round(long * short_side / short / 2)
    return (scaled, short_side) if width >= height else (short_side, scaled)


@dataclass(frozen=True)
class _Frames:
    """The frames written: each kept frame's time and its index among the decoded frames."""

    times: list[float]
    decoded: list[int]
    count: int
    camera: lenses.Pinhole | lenses.RadTan
    rotation: int


@dataclass(frozen=True)
class _Geometry:
    """How each of a video's frames is resampled, and the camera of the frames written.

    The frames' size as stored and as written, the undistortion map (None but for a fisheye)
    and the counter-clockwise quarter turns, then the upright frames' camera.
    """

    stored: tuple[int, int]
    size: tuple[int, int]
    mapping: NDArray[np.float64] | None
    turns: int
    camera: lenses.Pinhole | lenses.RadTan


def _check_size(path: Path, cap: int) -> None:
    """Refuse a file over its cap."""
    size = path.stat().st_size
    if size > cap:
        raise InvalidInputError(f"{path.name}: {size} bytes, over the cap of {cap}")


def _quarter_turns(rotation: float) -> int:
    """Return a declared rotation as counter-clockwise quarter turns, refusing any other."""
    turns = round(rotation / 90.0)
    if abs(rotation - 90.0 * turns) > 1e-6:
        raise InvalidInputError(f"rotation: expected a multiple of 90 degrees, got {rotation}")
    return turns % 4


def _open(path: Path, *, edits: bool = True) -> av.container.InputContainer:
    """Open an upload with the mov/mp4 demuxer alone, as a file alone (the module docstring).

    Parameters
    ----------
    path : pathlib.Path
        The video.
    edits : bool, optional
        Whether to honour the video's edit list; Stray Scanner's hides its first frame.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not a readable mov/mp4 video, or has no video stream.
    """
    options = dict(_OPEN_OPTIONS) if edits else {**_OPEN_OPTIONS, "ignore_editlist": "1"}
    try:
        container = av.open(str(path), format=_DEMUXER, options=options)
    except av.error.FFmpegError as error:
        raise InvalidInputError(f"{path.name}: not a readable video ({error})") from None
    if not container.streams.video:
        container.close()
        raise InvalidInputError(f"{path.name}: no video stream")
    return container


def _decode(
    path: Path, *, edits: bool = True, limits: Limits = UPLOAD
) -> Iterator[tuple[float, av.VideoFrame]]:
    """Yield each decoded frame with its time since the first, refusing a video too long."""
    with _open(path, edits=edits) as container:
        first = None
        try:
            for index, frame in enumerate(container.decode(video=0)):
                if index >= limits.max_frames:
                    raise InvalidInputError(f"{path.name}: more than {limits.max_frames} frames")
                if frame.time is None:
                    raise InvalidInputError(f"{path.name}: a frame without a presentation time")
                first = frame.time if first is None else first
                elapsed = frame.time - first
                if elapsed > limits.max_seconds:
                    raise InvalidInputError(f"{path.name}: longer than {limits.max_seconds:g} s")
                yield elapsed, frame
        except av.error.FFmpegError as error:
            raise InvalidInputError(f"{path.name}: undecodable ({error})") from None


def _undistorted(picture: NDArray[np.uint8], mapping: NDArray[np.float64]) -> NDArray[np.uint8]:
    """Resample an RGB picture's channels at a map of source coordinates, on the fastest backend."""
    built = backends.available()
    backend = next(name for name in _REMAP_ORDER if name in built)
    channels = [image.remap(picture[:, :, c], mapping, backend=backend) for c in range(3)]
    return np.ascontiguousarray(np.stack(channels, axis=-1))


def _matrix(lens: lenses.Pinhole | lenses.Fisheye | lenses.RadTan) -> lenses.Pinhole:
    """Return a lens's camera matrix and image size."""
    return lens if isinstance(lens, lenses.Pinhole) else lens.camera


def _geometry(
    frame: av.VideoFrame, lens: lenses.Pinhole | lenses.Fisheye | lenses.RadTan, short_side: int
) -> _Geometry:
    """Return how a video's frames are resampled, from its first frame and its intrinsics."""
    stored = _matrix(lens)
    if (frame.width, frame.height) != (stored.width, stored.height):
        raise InvalidInputError(
            f"intrinsics: given for {stored.width}x{stored.height}, the video is "
            f"{frame.width}x{frame.height}"
        )
    turns = _quarter_turns(frame.rotation)
    size = output_size(frame.width, frame.height, short_side)
    scaled = lens.scaled(*size)
    mapping = None
    if isinstance(scaled, lenses.Fisheye):
        camera: lenses.Pinhole | lenses.RadTan
        camera, mapping = lenses.undistortion(scaled)
    else:
        camera = scaled  # a pinhole, or a radial-tangential lens kept
    return _Geometry((frame.width, frame.height), size, mapping, turns, camera.turned(turns))


def _kept(elapsed: float, times: list[float]) -> bool:
    """Whether a frame is kept: the first, then each :data:`MIN_SPACING` after the last kept.

    A frame out of time order is never that far after, so it is skipped too.
    """
    return not times or elapsed - times[-1] >= MIN_SPACING


def _picture(frame: av.VideoFrame, geometry: _Geometry) -> NDArray[np.uint8]:
    """Return a frame resampled: scaled, undistorted if the lens is a fisheye, turned upright."""
    width, height = geometry.size
    rgb = frame.reformat(width=width, height=height, format="rgb24", interpolation="AREA")
    picture: NDArray[np.uint8] = np.asarray(rgb.to_ndarray(), dtype=np.uint8)  # rgb24: 8-bit
    if geometry.mapping is not None:
        picture = _undistorted(picture, geometry.mapping)
    return np.rot90(picture, geometry.turns)


def _scaled_mask(mask: NDArray[np.uint8], size: tuple[int, int]) -> NDArray[np.uint8]:
    """Scale a mask down to `size` (width, height): a pixel is masked if any pixel under it is.

    Each output pixel covers ``[i s, (i + 1) s)`` of the stored rows (and columns), `s` the
    stored size over the output's; every stored pixel that range touches counts.
    """
    out = mask
    for axis, target in ((0, size[1]), (1, size[0])):
        stored = out.shape[axis]
        step = stored / target
        starts = np.floor(np.arange(target) * step).astype(np.int64)
        ends = np.minimum(np.ceil((np.arange(target) + 1) * step).astype(np.int64), stored)
        out = np.stack(
            [
                np.take(out, range(a, b), axis=axis).min(axis=axis)
                for a, b in zip(starts, ends, strict=True)
            ],
            axis=axis,
        )
    return np.where(out != 0, 255, 0).astype(np.uint8)


def _mask_picture(mask: NDArray[np.uint8], geometry: _Geometry) -> NDArray[np.uint8]:
    """Carry a stored-size mask through the frames' geometry (the module docstring)."""
    scaled = _scaled_mask(mask, geometry.size)
    if geometry.mapping is not None:
        h, w = scaled.shape
        col = np.floor(geometry.mapping[..., 0] + 0.5).astype(np.int64)
        row = np.floor(geometry.mapping[..., 1] + 0.5).astype(np.int64)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        scaled = np.where(inside, scaled[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)], 0)
        scaled = scaled.astype(np.uint8)
    return np.ascontiguousarray(np.rot90(scaled, geometry.turns))


def _write_frames(  # noqa: PLR0913 -- the video, the folder, the lens; how, keyword-only
    video: Path,
    out: Path,
    lens: lenses.Pinhole | lenses.Fisheye | lenses.RadTan,
    *,
    edits: bool = True,
    limits: Limits = UPLOAD,
    clock: Sequence[float] | None = None,
    mask: NDArray[np.uint8] | None = None,
) -> _Frames:
    """Write a video's frames into a TUM-format folder: ``rgb/``, ``rgb.txt``, ``camera.txt``.

    Each frame is timed by the video, since its first, or by `clock`, one time a decoded frame.
    A `mask` at the frames' stored size is written as ``mask.png``, carried through their
    geometry (the module docstring).
    """
    writer = _Writer(out, video.name, lens, limits=limits)
    for index, time, frame in _clocked(video, edits=edits, limits=limits, clock=clock):
        writer.add(index, time, frame)
    return writer.finish(mask)


def _clocked(
    video: Path, *, edits: bool, limits: Limits, clock: Sequence[float] | None
) -> Iterator[tuple[int, float, av.VideoFrame]]:
    """Yield each decoded frame's index, its time (the video's, or `clock`'s) and the frame."""
    count = 0
    for index, (elapsed, frame) in enumerate(_decode(video, edits=edits, limits=limits)):
        count = index + 1
        if clock is not None and index >= len(clock):
            raise InvalidInputError(f"{video.name}: more frames than the {len(clock)} timed")
        yield index, (elapsed if clock is None else clock[index]), frame
    if clock is not None and count and count != len(clock):  # none at all: "no frames"
        raise InvalidInputError(f"{video.name}: {count} frames, {len(clock)} timed")


class _Writer:
    """Writes timed frames into a TUM-format folder, as :func:`_write_frames` describes.

    The frames added since a :meth:`mark` can be taken back, the files written for them
    removed: a damaged segment's (:func:`import_segments`). `name` is what an error calls the
    frames.
    """

    def __init__(
        self,
        out: Path,
        name: str,
        lens: lenses.Pinhole | lenses.Fisheye | lenses.RadTan,
        *,
        limits: Limits,
    ) -> None:
        """Start writing into `out`, nothing written yet."""
        self.out, self.name, self.lens, self.limits = out, name, lens, limits
        self.times: list[float] = []
        self.decoded: list[int] = []
        self.listed: list[tuple[str, str]] = []
        self.geometry: _Geometry | None = None
        self.count = 0

    def add(self, index: int, time: float, frame: av.VideoFrame) -> None:
        """Write a decoded frame, its index among the decoded frames, if it is kept."""
        self.count = index + 1
        geometry = self.geometry or _geometry(frame, self.lens, self.limits.short_side)
        self.geometry = geometry
        if (frame.width, frame.height) != geometry.stored:
            width, height = geometry.stored
            raise InvalidInputError(
                f"{self.name}: frame {index} is {frame.width}x{frame.height}, the first "
                f"{width}x{height}: a video's frames must all be one size"
            )
        if not _kept(time, self.times):
            return
        stamp = f"{time:.6f}"
        images.write_png(self.out / "rgb" / f"{stamp}.png", _picture(frame, geometry))
        self.listed.append((stamp, f"rgb/{stamp}.png"))
        self.times.append(float(stamp))  # as rgb.txt has it, for the ground truth to match
        self.decoded.append(index)

    def mark(self) -> tuple[int, int, _Geometry | None]:
        """Return where the writing stands, for :meth:`rollback`."""
        return len(self.listed), self.count, self.geometry

    def rollback(self, mark: tuple[int, int, _Geometry | None]) -> None:
        """Take back the frames added since `mark`, removing the files written for them.

        Each file is one this writer wrote since the mark: kept frames' times increase, so no
        two share a name.
        """
        kept, count, geometry = mark
        for _, file in self.listed[kept:]:
            (self.out / file).unlink()
        del self.listed[kept:], self.times[kept:], self.decoded[kept:]
        self.count, self.geometry = count, geometry

    def finish(self, mask: NDArray[np.uint8] | None) -> _Frames:
        """Write ``rgb.txt``, ``camera.txt`` and the mask, if any; return the frames written."""
        geometry = self.geometry
        if geometry is None:
            raise InvalidInputError(f"{self.name}: no frames")
        if mask is not None:
            width, height = geometry.stored
            if mask.shape != (height, width):
                raise InvalidInputError(
                    f"mask: expected the video's stored size {width}x{height}, got "
                    f"{mask.shape[1]}x{mask.shape[0]}"
                )
            images.write_png(self.out / tum.MASK_FILE, _mask_picture(mask, geometry))
        camera = geometry.camera
        matrix = _matrix(camera)
        tum.write_list(self.out / "rgb.txt", self.listed)
        tum.write_camera(
            self.out,
            tum.Camera(
                "radtan" if isinstance(camera, lenses.RadTan) else "pinhole",
                camera.intrinsics()
                if isinstance(camera, lenses.RadTan)
                else np.array([matrix.fx, matrix.fy, matrix.cx, matrix.cy]),
                matrix.width,
                matrix.height,
                tum.UNUSED_DEPTH_SCALE,  # no depth maps
                None,
            ),
            note="imported by slam_it.eval.video (source.json says from what)",
        )
        return _Frames(self.times, self.decoded, self.count, camera, 90 * geometry.turns)


def _write_source(
    out: Path, kind: str, chosen: lenses.Chosen, frames: _Frames, read: dict[str, Any]
) -> None:
    """Write ``source.json``: the upload's kind, the intrinsics and their source, what was read.

    `read` is what the upload gave beside its frames: its ``metadata``, the fields read, and
    its ``depth``, the least LiDAR confidence kept (null without depth).
    """
    camera = _matrix(frames.camera)
    distortion = frames.camera.distortion if isinstance(frames.camera, lenses.RadTan) else None
    record: dict[str, Any] = {
        "kind": kind,
        "intrinsics": {
            "source": chosen.source,
            "profile": chosen.profile,
            "fx": camera.fx,
            "fy": camera.fy,
            "cx": camera.cx,
            "cy": camera.cy,
            "width": camera.width,
            "height": camera.height,
            "distortion": None if distortion is None else list(distortion),
        },
        "rotation": frames.rotation,
        "frames": {"decoded": frames.count, "kept": len(frames.times)},
        **read,
    }
    tum.write_source(out, record)


def import_video(  # noqa: PLR0913 -- the video, the folder, the lens; a dataset's, keyword-only
    video: str | Path,
    out: str | Path,
    chosen: lenses.Chosen,
    *,
    limits: Limits = UPLOAD,
    clock: Sequence[float] | None = None,
    truth: tuple[NDArray[np.float64], NDArray[np.float64]] | None = None,
    source: Mapping[str, Any] | None = None,
    mask: ArrayLike | None = None,
) -> None:
    """Import a video into a TUM-format folder (the module docstring).

    Parameters
    ----------
    video : str or pathlib.Path
        The video file.
    out : str or pathlib.Path
        The folder written: new or empty.
    chosen : slam_it.eval.lenses.Chosen
        The intrinsics, for the frames as stored, at the video's size, and their source.
    limits : Limits, optional
        The longest video, its most frames and the frames' short side (default: an upload's,
        :data:`UPLOAD`).
    clock : sequence of float, optional
        Each decoded frame's time, in seconds, on a dataset's clock, in place of the video's
        own times since its first frame.
    truth : tuple of ndarray, optional
        A dataset's ground truth, its times on the clock and ``T_world_cam``, written as
        ``groundtruth.txt`` with the frames, so the folder appears whole or not at all.
    source : mapping, optional
        What a dataset's import names it by in ``source.json``: its ``kind`` and the fields
        that identify it, as the sequence and its calibration (default ``{"kind": "video"}``,
        an upload).
    mask : array_like of uint8, shape (H, W), optional
        The features a run drops, those on a 0, at the video's stored size (before its
        declared rotation): written as ``mask.png``, carried through the frames' geometry (the
        module docstring).

    Raises
    ------
    FileNotFoundError
        If the video does not exist.
    slam_it.foundations.errors.InvalidInputError
        If the folder is not new or empty, the video is over a cap, not a readable video, has
        no frames, declares a rotation that is not a quarter turn, or is not the intrinsics'
        size; if a fisheye lens leaves no rectangle undistorted; or if a clock is given that
        does not time each decoded frame once; or if the mask is not uint8 of the video's
        stored size.
    """
    keep = None if mask is None else as_image(mask, "mask")
    path = Path(video)
    if not path.is_file():
        raise FileNotFoundError(f"no video at {path}")
    _check_size(path, MAX_BYTES)
    metadata = probe(path).metadata
    with tum.staged(out) as folder:
        frames = _write_frames(path, folder, chosen.lens, limits=limits, clock=clock, mask=keep)
        if truth is not None:
            runs.write_trajectory(folder / "groundtruth.txt", *truth)
        named = dict(source or {"kind": "video"})
        kind = str(named.pop("kind", "video"))
        read = {**named, "metadata": metadata, "depth": None, "mask": keep is not None}
        _write_source(folder, kind, chosen, frames, read)


def _open_segment(path: Path) -> av.container.InputContainer:
    """Open a recorded segment, MPEG-TS, as a file alone; refuse one with no video stream."""
    if not path.is_file():
        raise FileNotFoundError(f"no segment at {path}")
    try:
        container = av.open(str(path), format=_SEGMENT_DEMUXER, options=dict(_SEGMENT_OPTIONS))
    except av.error.FFmpegError as error:
        raise DamagedSegmentError(
            f"{path.name}: not a readable MPEG-TS segment ({error})"
        ) from None
    if not container.streams.video:
        container.close()
        raise DamagedSegmentError(f"{path.name}: no video stream")
    return container


def segment_size(segment: str | Path) -> tuple[int, int]:
    """Return a recorded segment's frame size, ``(width, height)``, from its stream.

    Raises
    ------
    FileNotFoundError
        If the segment does not exist.
    DamagedSegmentError
        If it is not a readable MPEG-TS video or does not state its size.
    """
    path = Path(segment)
    with _open_segment(path) as container:
        stream = container.streams.video[0]
        width, height = stream.codec_context.width, stream.codec_context.height
    if not (width and height):
        raise DamagedSegmentError(f"{path.name}: its frame size is not stated")
    return int(width), int(height)


def _decode_segment(path: Path, limits: Limits) -> Iterator[tuple[float, av.VideoFrame]]:
    """Yield a segment's frames, decoded straight through, with each one's time since its first.

    Straight through: seeking into a segment whose recording dropped data decodes garbage until
    the next intact keyframe.
    """
    with _open_segment(path) as container:
        container.streams.video[0].thread_type = "AUTO"
        first = None
        try:
            for frame in container.decode(video=0):
                if frame.time is None:
                    raise DamagedSegmentError(f"{path.name}: a frame without a presentation time")
                first = frame.time if first is None else first
                elapsed = frame.time - first
                if elapsed > limits.max_seconds:
                    raise InvalidInputError(f"{path.name}: longer than {limits.max_seconds:g} s")
                yield elapsed, frame
        except av.error.FFmpegError as error:
            raise DamagedSegmentError(f"{path.name}: undecodable ({error})") from None


def sample_frames(  # noqa: PLR0913 -- the segment; how it is sampled, keyword-only
    segment: str | Path,
    *,
    every: float,
    width: int | None = None,
    box: tuple[int, int, int, int] | None = None,
    gray: bool = False,
    max_seconds: float = SEGMENT_SECONDS,
) -> Iterator[tuple[float, NDArray[np.uint8]]]:
    """Yield a segment's frames sampled in time, decoded straight through.

    Parameters
    ----------
    segment : str or pathlib.Path
        A recorded MPEG-TS segment (a dashcam's).
    every : float
        The time between samples, in seconds: the first frame at or after each multiple of it,
        counted from the segment's first frame, is kept; 0 keeps every frame.
    width : int, optional
        The width each frame is scaled to, its aspect kept, before `box`; the stored width if
        None.
    box : tuple of int, optional
        ``(x, y, w, h)``, the part of each (scaled) frame kept, in pixels.
    gray : bool, optional
        Gray (H, W) frames in place of RGB (H, W, 3).
    max_seconds : float, optional
        The longest segment, in seconds.

    Yields
    ------
    tuple
        The frame's time since the segment's first frame, in seconds, and the frame, uint8.

    Raises
    ------
    FileNotFoundError
        If the segment does not exist.
    DamagedSegmentError
        If the segment cannot be read through (:class:`DamagedSegmentError`).
    slam_it.foundations.errors.InvalidInputError
        If `every` is negative or not finite, `width` is not positive, `box` is not inside the
        frame, or the segment is longer than `max_seconds`.
    """
    if not (np.isfinite(every) and every >= 0.0):
        raise InvalidInputError(f"every: expected a finite time >= 0, got {every}")
    if width is not None and width <= 0:
        raise InvalidInputError(f"width: expected a positive width, got {width}")
    path = Path(segment)
    due = 0.0
    for elapsed, frame in _decode_segment(path, Limits(max_seconds=max_seconds)):
        if elapsed + 1e-9 < due:
            continue
        due = (np.floor(elapsed / every + 1e-9) + 1.0) * every if every > 0 else 0.0
        out_w = frame.width if width is None else width
        out_h = frame.height if width is None else 2 * round(frame.height * width / frame.width / 2)
        picture = frame.reformat(
            width=out_w, height=out_h, format="gray" if gray else "rgb24", interpolation="AREA"
        ).to_ndarray()
        if box is not None:
            x, y, w, h = box
            if not (x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= out_w and y + h <= out_h):
                raise InvalidInputError(f"box: {box} is not inside the {out_w}x{out_h} frame")
            picture = picture[y : y + h, x : x + w]
        yield float(elapsed), np.ascontiguousarray(picture, dtype=np.uint8)


def data_packets(segment: str | Path) -> list[tuple[float, bytes]]:
    """Return a segment's data-stream packets, each with the video's time where it lies.

    A data packet (a dashcam's GPS records) carries no presentation time of its own, so it is
    given the time of the last video packet before it in the file, since the segment's first
    (0 before any).

    Parameters
    ----------
    segment : str or pathlib.Path
        A recorded MPEG-TS segment.

    Returns
    -------
    list of tuple
        Each data packet's video time, in seconds, and its bytes, in file order.

    Raises
    ------
    FileNotFoundError
        If the segment does not exist.
    DamagedSegmentError
        If it is not a readable MPEG-TS video, or its demuxing raises.
    """
    path = Path(segment)
    out: list[tuple[float, bytes]] = []
    with _open_segment(path) as container:
        video = container.streams.video[0]
        data = [s for s in container.streams if s.type == "data"]
        first: float | None = None
        last = 0.0
        try:
            for packet in container.demux(video, *data):
                base = video.time_base
                if packet.stream.type == "video" and packet.pts is not None and base:
                    time = float(packet.pts * base)
                    first = time if first is None else first
                    last = max(last, time - first)
                elif packet.stream.type == "data" and packet.size:
                    out.append((last, bytes(packet)))
        except av.error.FFmpegError as error:
            raise DamagedSegmentError(f"{path.name}: unreadable ({error})") from None
    return out


def import_segments(  # noqa: PLR0913 -- the segments, their starts, the folder, the lens; how
    segments: Sequence[str | Path],
    starts: Sequence[float],
    out: str | Path,
    chosen: lenses.Chosen,
    *,
    limits: Limits,
    mask: ArrayLike | None = None,
    source: Mapping[str, Any] | None = None,
) -> list[tuple[str, str]]:
    """Import recorded segments, in order, as one sequence into a TUM-format folder.

    A dashcam records a drive as short MPEG-TS segments (:mod:`slam_it.eval.dashcam`). Each is
    decoded straight through, and each frame timed on one clock: its segment's start plus its
    time since the segment's first frame. A gap between segments stays a gap. Frames are
    resampled, kept and written as :func:`import_video` writes them, with the lens, the mask and
    ``source.json``; no upload cap applies (a local recording), only `limits`.

    A damaged segment (:class:`DamagedSegmentError`) is skipped whole, the frames it gave
    before failing taken back, with a :class:`SkippedSegmentWarning`; ``source.json`` lists
    each skipped, and why, under ``skipped``.

    Parameters
    ----------
    segments : sequence of str or pathlib.Path
        The segments, in time order, all one frame size.
    starts : sequence of float
        Each segment's first frame's time on the run's clock, in seconds, increasing.
    out : str or pathlib.Path
        The folder written: new or empty.
    chosen : slam_it.eval.lenses.Chosen
        The intrinsics, at the segments' stored size, and their source.
    limits : Limits
        The longest segment, the most frames in all, and the frames' short side.
    mask : array_like of uint8, shape (H, W), optional
        The features a run drops, those on a 0, at the stored size.
    source : mapping, optional
        What ``source.json`` names the drive by: its ``kind`` (default ``"segments"``) and the
        fields that identify it.

    Returns
    -------
    list of tuple of str
        Each damaged segment skipped: its file name, and why.

    Raises
    ------
    FileNotFoundError
        If a segment does not exist.
    slam_it.foundations.errors.InvalidInputError
        If there are no segments, `starts` does not give one increasing time a segment, every
        segment is damaged, one is over `limits`, the frames are of more than one size or not
        the intrinsics', or the mask is not uint8 of the stored size.
    """
    paths = [Path(s) for s in segments]
    begin = np.asarray(starts, dtype=np.float64)
    if not paths:
        raise InvalidInputError("segments: none given")
    if begin.shape != (len(paths),) or not np.all(np.isfinite(begin)):
        raise InvalidInputError(f"starts: expected {len(paths)} finite times, got {begin.shape}")
    if np.any(np.diff(begin) <= 0.0):
        raise InvalidInputError("starts: the segments' starts must increase")
    keep = None if mask is None else as_image(mask, "mask")
    skipped: list[tuple[str, str]] = []
    with tum.staged(out) as folder:
        writer = _Writer(folder, f"{paths[0].name}..{paths[-1].name}", chosen.lens, limits=limits)
        for path, start in zip(paths, begin, strict=True):
            mark = writer.mark()
            try:
                _add_segment(writer, path, float(start), limits)
            except DamagedSegmentError as error:
                writer.rollback(mark)
                skipped.append((path.name, str(error)))
                warnings.warn(
                    f"skipped a damaged segment, {error}", SkippedSegmentWarning, stacklevel=2
                )
        if len(skipped) == len(paths):
            listed = "; ".join(reason for _, reason in skipped)
            raise InvalidInputError(f"segments: every one is damaged ({listed})")
        frames = writer.finish(keep)
        named = dict(source or {"kind": "segments"})
        kind = str(named.pop("kind", "segments"))
        read = {
            **named,
            "segments": [p.name for p in paths],
            "starts": [float(s) for s in begin],
            "metadata": {},
            "depth": None,
            "mask": keep is not None,
            "skipped": [{"segment": name, "reason": reason} for name, reason in skipped],
        }
        _write_source(folder, kind, chosen, frames, read)
    return skipped


def _add_segment(writer: _Writer, path: Path, start: float, limits: Limits) -> None:
    """Add a segment's frames to `writer`, timed from `start`; refuse one with no frames."""
    index = first = writer.count
    for elapsed, frame in _decode_segment(path, limits):
        if index >= limits.max_frames:
            raise InvalidInputError(f"segments: more than {limits.max_frames} frames")
        writer.add(index, start + elapsed, frame)
        index += 1
    if index == first:
        raise DamagedSegmentError(f"{path.name}: no frames")


@dataclass(frozen=True)
class _StrayEntries:
    """A Stray Scanner zip's entries the import may open.

    Its files by name, and its LiDAR maps by kind and frame.
    """

    files: dict[str, zipfile.ZipInfo]
    maps: dict[tuple[str, int], zipfile.ZipInfo]


def _stray_map(name: PurePosixPath) -> tuple[str, int] | None:
    """Return a LiDAR map's kind and frame from its name, ``[folder/]<kind>/NNNNNN.png``."""
    if len(name.parts) not in (2, 3) or name.parent.name not in STRAY_MAPS:
        return None
    if name.suffix != ".png" or not _FRAME_NUMBER.fullmatch(name.stem):
        return None
    return name.parent.name, int(name.stem)


def _stray_entries(archive: zipfile.ZipFile) -> _StrayEntries:
    """Return a Stray Scanner zip's entries the import opens, refusing a zip that is unsafe."""
    found: dict[str, zipfile.ZipInfo] = {}
    maps: dict[tuple[str, int], zipfile.ZipInfo] = {}
    for info in archive.infolist():
        name = PurePosixPath(info.filename)
        if name.is_absolute() or ".." in name.parts or "\\" in info.filename:
            raise InvalidInputError(f"zip: an entry's name leaves its folder ({info.filename!r})")
        lidar_map = None if info.is_dir() else _stray_map(name)
        if lidar_map is not None:
            kind, frame = lidar_map
            if lidar_map in maps:
                raise InvalidInputError(f"zip: {kind}/{frame:06d}.png appears twice")
            if (
                info.file_size > STRAY_MAPS[kind]
            ):  # named as we name it: the zip's name is free text
                raise InvalidInputError(
                    f"zip: {kind}/{frame:06d}.png is over its cap of {STRAY_MAPS[kind]}"
                )
            maps[lidar_map] = info
            continue
        if info.is_dir() or name.name not in STRAY_CAPS:
            continue
        if len(name.parts) > 2:
            continue  # deeper than one folder: not the recording's own file
        if name.name in found:
            raise InvalidInputError(f"zip: {name.name} appears twice")
        if info.file_size > STRAY_CAPS[name.name]:
            raise InvalidInputError(f"zip: {name.name} is over its cap of {STRAY_CAPS[name.name]}")
        found[name.name] = info
    missing = [name for name in STRAY_CAPS if name not in found]
    if missing:
        raise InvalidInputError(f"zip: no {', '.join(missing)}")
    return _StrayEntries(found, maps)


def _read_entry(archive: zipfile.ZipFile, info: zipfile.ZipInfo, cap: int) -> Iterator[bytes]:
    """Yield an entry's bytes in chunks, refusing one that runs past its cap."""
    read = 0
    try:
        with archive.open(info) as entry:
            while chunk := entry.read(_CHUNK):
                read += len(chunk)
                if read > cap:
                    raise InvalidInputError(f"zip: {info.filename} runs past its cap of {cap}")
                yield chunk
    except _ZIP_ERRORS as error:
        name = PurePosixPath(info.filename).name
        raise InvalidInputError(f"zip: {name} cannot be read ({error})") from None


def _read_map(
    archive: zipfile.ZipFile, entries: _StrayEntries, kind: str, frame: int
) -> NDArray[np.uint16] | NDArray[np.uint8]:
    """Return a frame's LiDAR map, decoded from its bytes, its size checked first.

    The size checked is the one Pillow will decode, every header chunk parsed
    (:mod:`slam_it.eval.images`), against :data:`STRAY_MAP_SIDE`.
    """
    info = entries.maps.get((kind, frame))
    if info is None:
        raise InvalidInputError(f"zip: no {kind}/{frame:06d}.png for frame {frame}")
    data = b"".join(_read_entry(archive, info, STRAY_MAPS[kind]))
    try:
        if kind == "depth":
            return images.read_depth(data, STRAY_MAP_SIDE)
        return images.read_gray(data, STRAY_MAP_SIDE)
    except InvalidInputError as error:
        raise InvalidInputError(f"zip: {kind}/{frame:06d}.png: {error}") from None


def _write_depth(
    archive: zipfile.ZipFile,
    entries: _StrayEntries,
    out: Path,
    frames: _Frames,
    min_confidence: int,
) -> None:
    """Write each kept frame's LiDAR depth, registered to it, and the camera for an RGB-D run.

    The depth maps go in ``depth/``, listed in ``depth.txt``; ``camera.txt`` is rewritten with
    LiDAR's depth scale and ``bf``.
    """
    camera = _matrix(frames.camera)
    turns = frames.rotation // 90
    listed = []
    for time, frame in zip(frames.times, frames.decoded, strict=True):
        stamp = f"{time:.6f}"
        depth = np.rot90(_read_map(archive, entries, "depth", frame), turns)
        confidence = np.rot90(_read_map(archive, entries, "confidence", frame), turns)
        try:
            registered = lidar.registered_depth(
                np.ascontiguousarray(depth, dtype=np.uint16),
                np.ascontiguousarray(confidence, dtype=np.uint8),
                (camera.width, camera.height),
                min_confidence,
            )
        except InvalidInputError as error:
            raise InvalidInputError(f"zip: frame {frame}'s LiDAR maps: {error}") from None
        images.write_depth(out / "depth" / f"{stamp}.png", registered)
        listed.append((stamp, f"depth/{stamp}.png"))
    tum.write_list(out / "depth.txt", listed)
    tum.write_camera(
        out,
        tum.Camera(
            "pinhole",
            np.array([camera.fx, camera.fy, camera.cx, camera.cy]),
            camera.width,
            camera.height,
            lidar.DEPTH_SCALE,
            lidar.bf(camera.fx),
        ),
        note="imported by slam_it.eval.video with LiDAR depth (source.json says from what)",
    )


def _text(data: bytes, name: str) -> str:
    """Decode a CSV's bytes as UTF-8, refusing anything else."""
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        raise InvalidInputError(f"{name}: not UTF-8 text") from None


def probe(video: str | Path) -> lenses.Video:
    """Return what a video's intrinsics are chosen from: its metadata, stored size and rate.

    Parameters
    ----------
    video : str or pathlib.Path
        The video file.

    Returns
    -------
    slam_it.eval.lenses.Video
        Its fields of :data:`METADATA_KEYS`, its frame size as stored (before any declared
        rotation), and its average frame rate, if it gives one.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not a readable mov/mp4 video, has no video stream, or its frames are over
        :data:`MAX_LONG_SIDE` long or :data:`MAX_ASPECT` to one.
    """
    path = Path(video)
    with _open(path) as container:
        stream = container.streams.video[0]
        long, short = max(stream.width, stream.height), min(stream.width, stream.height)
        if long > MAX_LONG_SIDE:
            raise InvalidInputError(f"{path.name}: frames over {MAX_LONG_SIDE} pixels long")
        if short <= 0 or long > MAX_ASPECT * short:
            raise InvalidInputError(f"{path.name}: frames more than {MAX_ASPECT:g} to 1")
        found = dict(container.metadata)
        rate = stream.average_rate
        return lenses.Video(
            {key: str(found[key]) for key in METADATA_KEYS if key in found},
            stream.width,
            stream.height,
            float(rate) if rate else None,
        )


def quarter_turns(video: str | Path) -> int:
    """Return the counter-clockwise quarter turns a video's declared rotation turns it upright by.

    A dataset's calibration describes its frames upright; :class:`slam_it.eval.lenses.Chosen`
    describes them as stored, so the calibration is turned back by these turns.

    Parameters
    ----------
    video : str or pathlib.Path
        The video file.

    Returns
    -------
    int
        0 to 3, as ``numpy.rot90`` counts them.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If it is not a readable mov/mp4 video, has no frames, or declares a rotation that is
        not a quarter turn.
    """
    path = Path(video)
    for _, frame in _decode(path):
        return _quarter_turns(frame.rotation)
    raise InvalidInputError(f"{path.name}: no frames")


def _csv_rows(text: str, name: str) -> list[list[str]]:
    """Return a CSV's rows, refusing one Python's reader cannot parse (as a field too long)."""
    try:
        return list(csv.reader(io.StringIO(text)))
    except csv.Error as error:
        raise InvalidInputError(f"{name}: not a CSV ({error})") from None


def _numbers(row: Sequence[str], where: str) -> list[float]:
    """Parse a CSV row of numbers, refusing anything else."""
    try:
        values = [float(cell) for cell in row]
    except ValueError:
        raise InvalidInputError(f"{where}: expected numbers, got {list(row)}") from None
    if not all(np.isfinite(values)):
        raise InvalidInputError(f"{where}: expected finite numbers")
    return values


def _camera_matrix(text: str, width: int, height: int) -> lenses.Pinhole:
    """Read ``camera_matrix.csv``: a 3x3 matrix, without skew, its last row 0 0 1."""
    rows = [row for row in _csv_rows(text, "camera_matrix.csv") if row]
    matrix = np.array([_numbers(row, f"camera_matrix.csv:{n}") for n, row in enumerate(rows, 1)])
    if matrix.shape != (3, 3):
        raise InvalidInputError(f"camera_matrix.csv: expected 3 rows of 3, got {matrix.shape}")
    if matrix[0, 1] != 0.0 or matrix[1, 0] != 0.0 or list(matrix[2]) != [0.0, 0.0, 1.0]:
        raise InvalidInputError("camera_matrix.csv: expected [[fx 0 cx] [0 fy cy] [0 0 1]]")
    return lenses.Pinhole(matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2], width, height)


def _odometry(text: str) -> NDArray[np.float64]:
    """Read ``odometry.csv``'s :data:`ODOMETRY_COLUMNS`, one row per frame, frames in order."""
    reader = iter(_csv_rows(text, "odometry.csv"))
    header = [cell.strip() for cell in next(reader, [])]
    missing = [column for column in ODOMETRY_COLUMNS if column not in header]
    if missing:
        raise InvalidInputError(f"odometry.csv: no column {', '.join(missing)}")
    at = [header.index(column) for column in ODOMETRY_COLUMNS]
    rows = []
    for number, row in enumerate(reader, start=2):
        if not row:
            continue
        if len(row) != len(header):
            raise InvalidInputError(f"odometry.csv:{number}: expected {len(header)} cells")
        values = _numbers([row[i] for i in at], f"odometry.csv:{number}")
        rows.append(values)
    table = np.array(rows, dtype=np.float64).reshape(-1, len(ODOMETRY_COLUMNS))
    if not np.array_equal(table[:, 1], np.arange(len(table))):
        raise InvalidInputError("odometry.csv: expected frames 0, 1, 2, ... in order")
    return table


def _stray_poses(table: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return each odometry row's ``T_world_cam``: Stray Scanner writes it in our camera's axes.

    Its rotation is ARKit's camera rotation times a half turn about x (the module docstring),
    so the quaternion is used as written.
    """
    quaternions = table[:, 5:9]
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms == 0.0):
        raise InvalidInputError("odometry.csv: a zero quaternion")
    poses = np.repeat(np.eye(4)[None], len(table), axis=0)
    poses[:, :3, :3] = so3.from_quaternion(quaternions / norms)
    poses[:, :3, 3] = table[:, 2:5]
    return poses


def import_stray(recording: str | Path, out: str | Path, min_confidence: int = lidar.HIGH) -> None:
    """Import a Stray Scanner recording's zip into a TUM-format folder, with ground truth.

    Its LiDAR depth, if it holds any, is imported too, for an RGB-D run (the module docstring).

    Parameters
    ----------
    recording : str or pathlib.Path
        The zip Stray Scanner's Share button makes.
    out : str or pathlib.Path
        The folder written: new or empty.
    min_confidence : int, optional
        The least LiDAR confidence kept (:data:`slam_it.eval.lidar.HIGH` by default).

    Raises
    ------
    FileNotFoundError
        If the zip does not exist.
    slam_it.foundations.errors.InvalidInputError
        If the folder is not new or empty, the zip is over its cap, unsafe, not a zip, lacks a
        file, holds malformed CSVs or one odometry row per frame, has LiDAR maps but not a
        readable depth and confidence map for every frame kept, or its video is refused as
        :func:`import_video` refuses one.
    """
    path = Path(recording)
    if not path.is_file():
        raise FileNotFoundError(f"no recording at {path}")
    _check_size(path, MAX_BYTES)
    with tum.staged(out) as folder:
        video = folder / ".upload.mp4"  # our own name, never the zip's
        try:
            archive = zipfile.ZipFile(path)
        except zipfile.BadZipFile:
            raise InvalidInputError(f"{path.name}: not a zip") from None
        with archive:
            entries = _stray_entries(archive)
            files = entries.files
            with video.open("wb") as file:
                for chunk in _read_entry(archive, files["rgb.mp4"], STRAY_CAPS["rgb.mp4"]):
                    file.write(chunk)
            texts = {
                name: _text(b"".join(_read_entry(archive, files[name], STRAY_CAPS[name])), name)
                for name in ("camera_matrix.csv", "odometry.csv")
            }
            stored = probe(video)
            camera = _camera_matrix(texts["camera_matrix.csv"], stored.width, stored.height)
            table = _odometry(texts["odometry.csv"])
            frames = _write_frames(video, folder, camera, edits=False)
            video.unlink()
            if len(table) != frames.count:
                raise InvalidInputError(
                    f"odometry.csv: {len(table)} rows for the video's {frames.count} frames"
                )
            depth = min_confidence if entries.maps else None
            if depth is not None:
                _write_depth(archive, entries, folder, frames, depth)
        poses = _stray_poses(table)[frames.decoded]
        runs.write_trajectory(folder / "groundtruth.txt", frames.times, poses)
        read = {
            "metadata": {},
            "depth": None if depth is None else {"min_confidence": depth},
            "mask": False,
        }
        _write_source(folder, "stray", lenses.Chosen(camera, "stray-scanner"), frames, read)


def main(arguments: Sequence[str]) -> int:
    """Import an upload from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a one-line summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.video")
    parser.add_argument("upload", type=Path, help="the video, or a Stray Scanner zip")
    parser.add_argument("out", type=Path, help="the TUM-format folder to write")
    given = parser.add_mutually_exclusive_group()
    given.add_argument("--stray", action="store_true", help="the upload is a Stray Scanner zip")
    given.add_argument(
        "--intrinsics", type=float, nargs=4, metavar=("FX", "FY", "CX", "CY"),
        help="the video's pinhole intrinsics, for its frames as stored",
    )  # fmt: skip
    given.add_argument(
        "--source", choices=("auto", *lenses.SOURCES), default="auto",
        help="the intrinsics' source (default: the best that has them)",
    )  # fmt: skip
    parser.add_argument("--profiles", type=Path, help="the lens profiles (default: fetched)")
    parser.add_argument(
        "--mask", type=Path,
        help="an 8-bit gray PNG at the video's stored size, 0 where features are dropped",
    )  # fmt: skip
    parser.add_argument(
        "--min-confidence", type=int, choices=(0, lidar.MEDIUM, lidar.HIGH),
        help="with --stray, the least LiDAR confidence kept (default: high)",
    )  # fmt: skip
    args = parser.parse_args(arguments)
    if args.min_confidence is not None and not args.stray:
        parser.error("--min-confidence: only with --stray")
    if args.mask is not None and args.stray:
        parser.error("--mask: not with --stray")
    mask = None if args.mask is None else images.read_gray(args.mask)
    if args.stray:
        threshold = lidar.HIGH if args.min_confidence is None else args.min_confidence
        import_stray(args.upload, args.out, threshold)
    elif args.intrinsics:
        fx, fy, cx, cy = args.intrinsics
        video = probe(args.upload)
        pinhole = lenses.Pinhole(fx, fy, cx, cy, video.width, video.height)
        import_video(args.upload, args.out, lenses.Chosen(pinhole, "given"), mask=mask)
    else:
        # The database is needed, and so required, only when a profile may be chosen.
        wanted = args.source in ("auto", "profile")
        profiles = lenses.database(args.profiles) if wanted else []
        chosen = lenses.choose(probe(args.upload), profiles, args.source)
        import_video(args.upload, args.out, chosen, mask=mask)
    record = json.loads((args.out / "source.json").read_text())
    depth = record["depth"]
    print(
        f"{args.out}: {record['frames']['kept']} frames, "
        f"{record['intrinsics']['width']}x{record['intrinsics']['height']}, "
        f"intrinsics from {record['intrinsics']['source']}"
        + ("" if depth is None else f", LiDAR depth of confidence {depth['min_confidence']} up")
        + (", masked" if record["mask"] else "")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
