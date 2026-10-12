"""Phone recordings of the room, made for the tests: videos, and Stray Scanner zips.

The videos are encoded with PyAV, losslessly (x264 at qp 0; Stray Scanner's, x265 lossless),
so an imported frame is the rendered one up to the colour conversion's rounding. Shared by the
import's tests (``tests/eval/test_video.py``) and the worker's
(``tests/orchestration/test_worker.py``).
"""

from __future__ import annotations

import zipfile
from collections.abc import Mapping
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
from numpy.typing import NDArray

from slam_it.eval import lenses
from slam_it.foundations import so3
from tests.edge import room

ROOM_CAMERA = lenses.Pinhole(
    room.SMALL.fx, room.SMALL.fy, room.SMALL.cx, room.SMALL.cy, room.SMALL.width,
    room.SMALL.height,
)  # fmt: skip
"""The room's camera, as a pinhole."""

APPLE = {
    "com.apple.quicktime.make": "Apple",
    "com.apple.quicktime.model": "iPhone 14 Pro",
    "com.apple.quicktime.camera.lens_model": "iPhone 14 Pro back camera 6.86mm f/1.78",
    "com.apple.quicktime.location.ISO6709": "+52.5163+013.3777+034.000/",
    "com.apple.quicktime.software": "17.0",
}
"""Apple-style tags, a location among them."""


def rgb(gray: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return a gray image as RGB."""
    return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))


def encode(
    path: Path,
    pictures: list[NDArray[np.uint8]],
    *,
    rate: int = 30,
    rotation: int = 0,
    metadata: dict[str, str] | None = None,
) -> Path:
    """Encode RGB pictures as a lossless H.264 video, with a declared rotation and tags."""
    options = {"movflags": "use_metadata_tags"} if metadata else {}
    with av.open(str(path), "w", format="mov", options=options) as out:
        for key, value in (metadata or {}).items():
            out.metadata[key] = value
        stream = out.add_stream("libx264", rate=rate, options={"qp": "0"})
        stream.height, stream.width = pictures[0].shape[:2]
        stream.pix_fmt = "yuv420p"
        if rotation:
            stream.set_display_rotation(rotation)
        for picture in pictures:
            for packet in stream.encode(av.VideoFrame.from_ndarray(picture, format="rgb24")):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return path


STRAY_TIMESCALE = 60
"""Stray Scanner's video timescale: a frame is stamped at its ARKit frame number over 60."""

STRAY_DIVIDER = 1
"""Stray Scanner's default frame-rate divider (``RecordSessionViewController.swift``'s
``FpsDividers``, unset): every ARKit frame, 60 fps. 2 is its 30 fps setting."""

ARKIT_FROM_OURS = np.diag([1.0, -1.0, -1.0])
"""Our camera's axes (x right, y down, z forwards) to ARKit's (x right, y up, z backwards):
a half turn about x, its own inverse."""

Q_AC = np.array([1.0, 0.0, 0.0, 0.0])
"""Stray Scanner's ``q_AC`` (``OdometryEncoder.swift``), xyzw: a half turn about x."""


def encode_stray(
    path: Path, pictures: list[NDArray[np.uint8]], *, divider: int = STRAY_DIVIDER
) -> Path:
    """Encode pictures as Stray Scanner's ``rgb.mp4``, in HEVC, stamped as its encoder stamps.

    ``DatasetEncoder.swift`` keeps one ARKit frame in `divider` and passes the frame counter
    before its increment, from -1, so frames are stamped -1, divider - 1, 2 divider - 1, ... over
    :data:`STRAY_TIMESCALE`, after a session started at 0: the first frame sits behind the MP4's
    edit list. ``VideoEncoder.swift`` encodes HEVC, whose B-frames are decoded out of order.
    """
    options = {"use_editlist": "1", "avoid_negative_ts": "disabled"}
    with av.open(str(path), "w", format="mp4", options=options) as out:
        params = {"x265-params": "lossless=1:bframes=4:log-level=error"}
        stream = out.add_stream("libx265", rate=STRAY_TIMESCALE, options=params)  # its time base
        stream.height, stream.width = pictures[0].shape[:2]
        stream.pix_fmt = "yuv420p"
        stream.time_base = Fraction(1, STRAY_TIMESCALE)
        for i, picture in enumerate(pictures):
            frame = av.VideoFrame.from_ndarray(picture, format="rgb24")
            frame.pts, frame.time_base = divider * i - 1, Fraction(1, STRAY_TIMESCALE)
            for packet in stream.encode(frame):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return path


def stray_quaternion(pose: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the quaternion Stray Scanner writes for a pose: ARKit's rotation times ``q_AC``.

    The pose is ``T_world_cam``; ARKit's camera rotation is its rotation in ARKit's axes, and
    ``OdometryEncoder.swift`` writes ``q_WA * q_AC``.
    """
    world_arkit = pose[:3, :3] @ ARKIT_FROM_OURS
    written = world_arkit @ so3.from_quaternion(Q_AC)
    out: NDArray[np.float64] = so3.to_quaternion(written)
    return out


def stray_zip(
    path: Path,
    rendered: list[room.Frame],
    *,
    folder: str = "2026-10-10--12-00-00/",
    files: Mapping[str, bytes | str] | None = None,
    names: list[str] | None = None,
    divider: int = STRAY_DIVIDER,
) -> Path:
    """Write the room as a Stray Scanner zip, as its encoders write one: video, matrix, poses.

    Each rendered frame is one the encoder kept (:func:`encode_stray`), with its pose. `files`
    replaces or adds entries, by name in the folder; `names` adds entries named as given, with
    a small body.
    """
    pictures = [rgb(f.gray) for f in rendered]
    video_bytes = encode_stray(path.with_suffix(".mp4"), pictures, divider=divider).read_bytes()
    c = room.SMALL
    matrix = f"{c.fx},0.0,{c.cx}\n0.0,{c.fy},{c.cy}\n0.0,0.0,1.0\n"
    rows = ["timestamp, frame, x, y, z, qx, qy, qz, qw"]
    for i, f in enumerate(rendered):
        values = [100.0 + f.timestamp, *f.T_world_cam[:3, 3], *stray_quaternion(f.T_world_cam)]
        x, y, z, qx, qy, qz, qw = (repr(float(v)) for v in values[1:])
        rows.append(f"{float(values[0])!r}, {i}, {x}, {y}, {z}, {qx}, {qy}, {qz}, {qw}")
    entries: dict[str, bytes | str] = {
        "rgb.mp4": video_bytes,
        "camera_matrix.csv": matrix,
        "odometry.csv": "\n".join(rows) + "\n",
        "depth/000000.png": b"not read",
    }
    entries.update(files or {})
    with zipfile.ZipFile(path, "w") as archive:
        for name, body in entries.items():
            archive.writestr(folder + name, body)
        for name in names or []:
            archive.writestr(name, b"x")
    return path
