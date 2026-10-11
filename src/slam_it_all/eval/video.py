"""Importing a phone's recording into a TUM-format folder, for the pipeline to run on.

Two kinds of upload (docs/design/2026-10-09-run-viewer.md, section 5):

- **A video** (:func:`import_video`), from any phone, its intrinsics chosen from its metadata
  (:func:`slam_it_all.eval.lenses.choose`) or given.
- **A Stray Scanner recording** (:func:`import_stray`): the zip Stray Scanner's Share button
  makes on an iPhone or iPad with LiDAR. Its video, its camera matrix (exact intrinsics) and
  ARKit's pose for each frame, which becomes the run's ground truth.

Either becomes a TUM-format folder (:mod:`slam_it_all.eval.tum`) without depth, which the
readers in all three languages already take: ``rgb/`` and ``rgb.txt``, ``camera.txt`` (a
pinhole), ``groundtruth.txt`` for a Stray Scanner recording, and ``source.json``, what the
import used. The folder must be new or empty; it is written beside, in a hidden
``.<name>.<id>.importing`` folder, and moved onto it once whole, so an import that fails leaves
nothing (one killed outright leaves that folder, which nothing reads: it is safe to delete).

**Frames.**

- Decoded with PyAV (FFmpeg): this is the one module that imports it, by a contract.
- Turned upright: the rotation the file declares (a phone stores a portrait video on its side)
  is applied, and the intrinsics are turned with it.
- Timed by each frame's presentation time, since a phone's frame rate varies. A frame closer
  than :data:`MIN_SPACING` to the last one kept is skipped, so 60 fps becomes 30.
- Scaled down, never up, so that the short side is :data:`SHORT_SIDE` pixels as TUM's and
  EuRoC's are (16:9 becomes 854 by 480), the intrinsics scaled with them.
- Kept in colour, as PNG (:func:`slam_it_all.eval.images.write_png`).

**Caps.** An upload over :data:`MAX_BYTES` or a video over :data:`MAX_SECONDS` is refused.

**Intrinsics** (:class:`slam_it_all.eval.lenses.Chosen`) describe the frames as the file
stores them, before the declared rotation, at the file's size, as a camera matrix or a lens
profile describes the sensor. A fisheye lens is undistorted to pinhole at the output size
(:func:`slam_it_all.eval.lenses.undistortion`), each frame's channels resampled by rung 0's
bilinear :func:`slam_it_all.foundations.image.remap`, so the pipeline only sees pinhole frames.

**Stray Scanner zips** are read in place, never unpacked under the names they contain. Only
``rgb.mp4``, ``camera_matrix.csv`` and ``odometry.csv`` are opened, each size checked against
its own cap before and while it is read; anything else is ignored. A name that is absolute,
climbs out of a folder or appears twice refuses the zip. The CSVs are parsed strictly: numbers
only, and one ``odometry.csv`` row per video frame.

**Metadata.** ``source.json`` records the intrinsics' source and the metadata fields the import
read (:data:`METADATA_KEYS`). Location tags and every other field are never copied.

**Assumptions the test clips check** (the design doc's section 5, "Checked first"): that
PyAV's rotation is counter-clockwise on a real portrait clip, as it is on the files PyAV
writes; that a Stray Scanner zip holds its three files at its root or in one folder; that
``camera_matrix.csv`` describes the video at its stored size; that ``odometry.csv`` has the
columns ``timestamp, frame, x, y, z, qx, qy, qz, qw`` (others ignored); and that ARKit's
camera has x right, y up and z backwards.

Run as a program::

    python -m slam_it_all.eval.video VIDEO OUT [--source auto|profile|35mm|nominal]
        [--profiles DIR]
    python -m slam_it_all.eval.video VIDEO OUT --intrinsics FX FY CX CY
    python -m slam_it_all.eval.video RECORDING.zip OUT --stray
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import shutil
import sys
import uuid
import zipfile
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import Any

import av
import numpy as np
from numpy.typing import NDArray

from slam_it_all import backends
from slam_it_all.eval import images, lenses, runs, tum
from slam_it_all.foundations import image, so3
from slam_it_all.foundations.errors import InvalidInputError

MAX_BYTES = 500 * 10**6
"""The largest upload, video or zip, in bytes."""

MAX_SECONDS = 60.0
"""The longest video, in seconds."""

SHORT_SIDE = 480
"""The short side frames are scaled down to, in pixels."""

MIN_SPACING = 1.0 / 40.0
"""The least time between two frames kept, in seconds: a 60 fps video keeps every other frame,
and a 30 fps one, its frame times jittering by a few milliseconds, keeps every frame."""

METADATA_KEYS = lenses.METADATA_KEYS
"""The metadata fields the import reads and records, the intrinsics chosen from them: the only
ones it ever copies."""

STRAY_CAPS = {"rgb.mp4": MAX_BYTES, "camera_matrix.csv": 4096, "odometry.csv": 64 * 2**20}
"""The Stray Scanner files the import opens, each with the most bytes it reads of it."""

ODOMETRY_COLUMNS = ("timestamp", "frame", "x", "y", "z", "qx", "qy", "qz", "qw")
"""The ``odometry.csv`` columns the import reads: ARKit's camera pose for each frame."""

ARKIT_TO_CAMERA = np.diag([1.0, -1.0, -1.0, 1.0])
"""ARKit's camera axes (x right, y up, z backwards) to ours (x right, y down, z forwards)."""

_CHUNK = 1 << 20
"""The bytes copied at a time out of a zip."""

_REMAP_ORDER = ("rust", "cpp", "python")
"""The backends that undistort a frame, fastest first (7, 10 and 34 ms a channel at 854x480);
all three give the same bytes, as rung 0's parity tests hold."""


def output_size(width: int, height: int) -> tuple[int, int]:
    """Return the size frames are scaled to: the short side :data:`SHORT_SIDE`, never larger.

    Parameters
    ----------
    width, height : int
        The frame's size.

    Returns
    -------
    tuple of int
        The width and height, the long side rounded to an even number.
    """
    short, long = min(width, height), max(width, height)
    if short <= SHORT_SIDE:
        return width, height
    scaled = 2 * round(long * SHORT_SIDE / short / 2)
    return (scaled, SHORT_SIDE) if width >= height else (SHORT_SIDE, scaled)


@dataclass(frozen=True)
class _Frames:
    """The frames written: each kept frame's time and its index among the decoded frames."""

    times: list[float]
    decoded: list[int]
    count: int
    camera: lenses.Pinhole
    rotation: int
    metadata: dict[str, str]


@contextlib.contextmanager
def _staged(out: Path) -> Iterator[Path]:
    """Yield a new hidden folder beside `out`, moved onto it if the block completes.

    Refuses an `out` that exists and is not empty; removes the hidden folder if the block
    fails.
    """
    if out.exists() and any(out.iterdir()):
        raise InvalidInputError(f"{out}: expected a new or empty folder for the import")
    out.parent.mkdir(parents=True, exist_ok=True)
    staging = out.with_name(f".{out.name}.{uuid.uuid4().hex}.importing")
    staging.mkdir()
    try:
        yield staging
        staging.replace(out)  # onto a new or empty folder
    except BaseException:
        shutil.rmtree(staging)  # the import's own, made above
        raise


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


def _decode(path: Path) -> Iterator[tuple[float, av.VideoFrame]]:
    """Yield each decoded frame with its time since the first, refusing a video too long."""
    try:
        container = av.open(str(path))
    except av.error.FFmpegError as error:
        raise InvalidInputError(f"{path.name}: not a readable video ({error})") from None
    with container:
        if not container.streams.video:
            raise InvalidInputError(f"{path.name}: no video stream")
        first = None
        try:
            for frame in container.decode(video=0):
                if frame.time is None:
                    raise InvalidInputError(f"{path.name}: a frame without a presentation time")
                first = frame.time if first is None else first
                elapsed = frame.time - first
                if elapsed > MAX_SECONDS:
                    raise InvalidInputError(f"{path.name}: longer than {MAX_SECONDS:g} s")
                yield elapsed, frame
        except av.error.FFmpegError as error:
            raise InvalidInputError(f"{path.name}: undecodable ({error})") from None


def _undistorted(picture: NDArray[np.uint8], mapping: NDArray[np.float64]) -> NDArray[np.uint8]:
    """Resample an RGB picture's channels at a map of source coordinates, on the fastest backend."""
    built = backends.available()
    backend = next(name for name in _REMAP_ORDER if name in built)
    channels = [image.remap(picture[:, :, c], mapping, backend=backend) for c in range(3)]
    return np.ascontiguousarray(np.stack(channels, axis=-1))


def _write_frames(video: Path, out: Path, lens: lenses.Pinhole | lenses.Fisheye) -> _Frames:
    """Write a video's frames into a TUM-format folder's ``rgb/`` and ``rgb.txt``."""
    stamps = ["# timestamp filename"]
    times: list[float] = []
    decoded: list[int] = []
    stored = lens.camera if isinstance(lens, lenses.Fisheye) else lens
    upright: lenses.Pinhole | None = None
    mapping: NDArray[np.float64] | None = None
    size = (0, 0)
    turns = 0
    count = 0
    for index, (elapsed, frame) in enumerate(_decode(video)):
        count = index + 1
        if upright is None:
            if (frame.width, frame.height) != (stored.width, stored.height):
                raise InvalidInputError(
                    f"intrinsics: given for {stored.width}x{stored.height}, the video is "
                    f"{frame.width}x{frame.height}"
                )
            turns = _quarter_turns(frame.rotation)
            size = output_size(frame.width, frame.height)
            scaled = lens.scaled(*size)
            if isinstance(scaled, lenses.Fisheye):
                pinhole, mapping = lenses.undistortion(scaled)
            else:
                pinhole = scaled
            upright = pinhole.turned(turns)
        elif times and elapsed - times[-1] < MIN_SPACING:
            continue
        if times and elapsed <= times[-1]:
            continue
        rgb = frame.reformat(width=size[0], height=size[1], format="rgb24", interpolation="AREA")
        picture: NDArray[np.uint8] = np.asarray(rgb.to_ndarray(), dtype=np.uint8)  # rgb24: 8-bit
        if mapping is not None:
            picture = _undistorted(picture, mapping)
        stamp = f"{elapsed:.6f}"
        images.write_png(out / "rgb" / f"{stamp}.png", np.rot90(picture, turns))
        stamps.append(f"{stamp} rgb/{stamp}.png")
        times.append(float(stamp))  # as rgb.txt has it, for the ground truth to match
        decoded.append(index)
    if upright is None:
        raise InvalidInputError(f"{video.name}: no frames")
    (out / "rgb.txt").write_text("\n".join(stamps) + "\n")
    (out / tum.CAMERA_FILE).write_text(
        "# imported by slam_it_all.eval.video (source.json says from what)\n"
        "model pinhole\n"
        f"intrinsics {upright.fx!r} {upright.fy!r} {upright.cx!r} {upright.cy!r}\n"
        f"size {upright.width} {upright.height}\n"
        "depth_scale 1000  # no depth maps\n"
    )
    return _Frames(times, decoded, count, upright, 90 * turns, probe(video).metadata)


def _write_source(out: Path, kind: str, source: str, profile: str | None, frames: _Frames) -> None:
    """Write ``source.json``: the upload's kind, the intrinsics and their source, what was read."""
    camera = frames.camera
    record: dict[str, Any] = {
        "kind": kind,
        "intrinsics": {
            "source": source,
            "profile": profile,
            "fx": camera.fx,
            "fy": camera.fy,
            "cx": camera.cx,
            "cy": camera.cy,
            "width": camera.width,
            "height": camera.height,
        },
        "rotation": frames.rotation,
        "frames": {"decoded": frames.count, "kept": len(frames.times)},
        "metadata": frames.metadata,
    }
    (out / "source.json").write_text(json.dumps(record, indent=1) + "\n")


def import_video(video: str | Path, out: str | Path, chosen: lenses.Chosen) -> None:
    """Import a video into a TUM-format folder (the module docstring).

    Parameters
    ----------
    video : str or pathlib.Path
        The video file.
    out : str or pathlib.Path
        The folder written: new or empty.
    chosen : slam_it_all.eval.lenses.Chosen
        The intrinsics, for the frames as stored, at the video's size, and their source.

    Raises
    ------
    FileNotFoundError
        If the video does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If the folder is not new or empty, the video is over a cap, not a readable video, has
        no frames, declares a rotation that is not a quarter turn, or is not the intrinsics'
        size; or if a fisheye lens leaves no rectangle undistorted.
    """
    path = Path(video)
    if not path.is_file():
        raise FileNotFoundError(f"no video at {path}")
    _check_size(path, MAX_BYTES)
    with _staged(Path(out)) as folder:
        frames = _write_frames(path, folder, chosen.lens)
        _write_source(folder, "video", chosen.source, chosen.profile, frames)


def _stray_entries(archive: zipfile.ZipFile) -> dict[str, zipfile.ZipInfo]:
    """Return a Stray Scanner zip's entries the import opens, refusing a zip that is unsafe."""
    found: dict[str, zipfile.ZipInfo] = {}
    for info in archive.infolist():
        name = PurePosixPath(info.filename)
        if name.is_absolute() or ".." in name.parts or "\\" in info.filename:
            raise InvalidInputError(f"zip: an entry's name leaves its folder ({info.filename!r})")
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
    return found


def _read_entry(archive: zipfile.ZipFile, info: zipfile.ZipInfo, cap: int) -> Iterator[bytes]:
    """Yield an entry's bytes in chunks, refusing one that runs past its cap."""
    read = 0
    with archive.open(info) as entry:
        while chunk := entry.read(_CHUNK):
            read += len(chunk)
            if read > cap:
                raise InvalidInputError(f"zip: {info.filename} runs past its cap of {cap}")
            yield chunk


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
    slam_it_all.eval.lenses.Video
        Its fields of :data:`METADATA_KEYS`, its frame size as stored (before any declared
        rotation), and its average frame rate, if it gives one.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If it is not a readable video, or has no video stream.
    """
    path = Path(video)
    try:
        container = av.open(str(path))
    except av.error.FFmpegError as error:
        raise InvalidInputError(f"{path.name}: not a readable video ({error})") from None
    with container:
        if not container.streams.video:
            raise InvalidInputError(f"{path.name}: no video stream")
        stream = container.streams.video[0]
        found = dict(container.metadata)
        rate = stream.average_rate
        return lenses.Video(
            {key: str(found[key]) for key in METADATA_KEYS if key in found},
            stream.width,
            stream.height,
            float(rate) if rate else None,
        )


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
    rows = [row for row in csv.reader(io.StringIO(text)) if row]
    matrix = np.array([_numbers(row, f"camera_matrix.csv:{n}") for n, row in enumerate(rows, 1)])
    if matrix.shape != (3, 3):
        raise InvalidInputError(f"camera_matrix.csv: expected 3 rows of 3, got {matrix.shape}")
    if matrix[0, 1] != 0.0 or matrix[1, 0] != 0.0 or list(matrix[2]) != [0.0, 0.0, 1.0]:
        raise InvalidInputError("camera_matrix.csv: expected [[fx 0 cx] [0 fy cy] [0 0 1]]")
    return lenses.Pinhole(matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2], width, height)


def _odometry(text: str) -> NDArray[np.float64]:
    """Read ``odometry.csv``'s :data:`ODOMETRY_COLUMNS`, one row per frame, frames in order."""
    reader = csv.reader(io.StringIO(text))
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


def _arkit_poses(table: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return each odometry row's ``T_world_cam`` in our camera axes."""
    quaternions = table[:, 5:9]
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms == 0.0):
        raise InvalidInputError("odometry.csv: a zero quaternion")
    poses = np.repeat(np.eye(4)[None], len(table), axis=0)
    poses[:, :3, :3] = so3.from_quaternion(quaternions / norms)
    poses[:, :3, 3] = table[:, 2:5]
    out: NDArray[np.float64] = poses @ ARKIT_TO_CAMERA
    return out


def import_stray(recording: str | Path, out: str | Path) -> None:
    """Import a Stray Scanner recording's zip into a TUM-format folder, with ground truth.

    Parameters
    ----------
    recording : str or pathlib.Path
        The zip Stray Scanner's Share button makes.
    out : str or pathlib.Path
        The folder written: new or empty.

    Raises
    ------
    FileNotFoundError
        If the zip does not exist.
    slam_it_all.foundations.errors.InvalidInputError
        If the folder is not new or empty, the zip is over its cap, unsafe, not a zip, lacks a
        file, holds malformed CSVs or one odometry row per frame, or its video is refused as
        :func:`import_video` refuses one.
    """
    path = Path(recording)
    if not path.is_file():
        raise FileNotFoundError(f"no recording at {path}")
    _check_size(path, MAX_BYTES)
    with _staged(Path(out)) as folder:
        video = folder / ".upload.mp4"  # our own name, never the zip's
        try:
            archive = zipfile.ZipFile(path)
        except zipfile.BadZipFile:
            raise InvalidInputError(f"{path.name}: not a zip") from None
        with archive:
            entries = _stray_entries(archive)
            with video.open("wb") as file:
                for chunk in _read_entry(archive, entries["rgb.mp4"], STRAY_CAPS["rgb.mp4"]):
                    file.write(chunk)
            texts = {
                name: _text(b"".join(_read_entry(archive, entries[name], STRAY_CAPS[name])), name)
                for name in ("camera_matrix.csv", "odometry.csv")
            }
        stored = probe(video)
        camera = _camera_matrix(texts["camera_matrix.csv"], stored.width, stored.height)
        table = _odometry(texts["odometry.csv"])
        frames = _write_frames(video, folder, camera)
        video.unlink()
        if len(table) != frames.count:
            raise InvalidInputError(
                f"odometry.csv: {len(table)} rows for the video's {frames.count} frames"
            )
        poses = _arkit_poses(table)[frames.decoded]
        runs.write_trajectory(folder / "groundtruth.txt", frames.times, poses)
        _write_source(folder, "stray", "stray-scanner", None, replace(frames, metadata={}))


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
    parser = argparse.ArgumentParser(prog="python -m slam_it_all.eval.video")
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
    args = parser.parse_args(arguments)
    if args.stray:
        import_stray(args.upload, args.out)
    elif args.intrinsics:
        fx, fy, cx, cy = args.intrinsics
        video = probe(args.upload)
        pinhole = lenses.Pinhole(fx, fy, cx, cy, video.width, video.height)
        import_video(args.upload, args.out, lenses.Chosen(pinhole, "given"))
    else:
        folder = args.profiles or lenses.default_folder()
        profiles = lenses.load(folder) if folder.is_dir() else []
        chosen = lenses.choose(probe(args.upload), profiles, args.source)
        import_video(args.upload, args.out, chosen)
    record = json.loads((args.out / "source.json").read_text())
    print(
        f"{args.out}: {record['frames']['kept']} frames, "
        f"{record['intrinsics']['width']}x{record['intrinsics']['height']}, "
        f"intrinsics from {record['intrinsics']['source']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
