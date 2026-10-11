"""Phone recordings of the room, made for the tests: videos, and Stray Scanner zips.

The videos are encoded with PyAV, losslessly (x264 at qp 0), so an imported frame is the
rendered one up to the colour conversion's rounding. Shared by the import's tests
(``tests/eval/test_video.py``) and the worker's (``tests/orchestration/test_worker.py``).
"""

from __future__ import annotations

import zipfile
from collections.abc import Mapping
from pathlib import Path

import av
import numpy as np
from numpy.typing import NDArray

from slam_it_all.eval import lenses, video
from slam_it_all.foundations import so3
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


def stray_zip(
    path: Path,
    rendered: list[room.Frame],
    *,
    folder: str = "2026-10-10--12-00-00/",
    files: Mapping[str, bytes | str] | None = None,
    names: list[str] | None = None,
) -> Path:
    """Write the room as a Stray Scanner zip: the video, the camera matrix, ARKit's poses.

    `files` replaces or adds entries, by name in the folder; `names` adds entries named as
    given, with a small body.
    """
    video_bytes = encode(path.with_suffix(".mp4"), [rgb(f.gray) for f in rendered]).read_bytes()
    c = room.SMALL
    matrix = f"{c.fx},0.0,{c.cx}\n0.0,{c.fy},{c.cy}\n0.0,0.0,1.0\n"
    rows = ["timestamp, frame, x, y, z, qx, qy, qz, qw"]
    for i, f in enumerate(rendered):
        arkit = f.T_world_cam @ video.ARKIT_TO_CAMERA  # the diagonal is its own inverse
        values = [100.0 + f.timestamp, *arkit[:3, 3], *so3.to_quaternion(arkit[:3, :3])]
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
