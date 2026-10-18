"""Dashcam recordings made for the tests: MPEG-TS segments, GPS records, rig trajectories.

The segments are encoded with PyAV, losslessly (x264 at qp 0), as a D016 writes them: a video
stream and, if given, a data stream of records. Their pictures are a scene that moves (random
blocks, shifted each frame) under a border that does not (a hood across the bottom).
"""

from __future__ import annotations

import datetime as dt
import math
import struct
from collections.abc import Mapping, Sequence
from pathlib import Path

import av
import numpy as np
from numpy.typing import NDArray

from slam_it.eval.dashcam import d016
from slam_it.foundations import so3

HOOD = 0.8
"""Where the hood starts, as a fraction of the height."""


def write_segment(
    path: Path,
    pictures: Sequence[NDArray[np.uint8]],
    *,
    rate: int = 10,
    records: Mapping[int, bytes] | None = None,
    lengths: bool = False,
) -> Path:
    """Write RGB pictures as an MPEG-TS segment, a data packet after each frame `records` names.

    With `lengths`, each video PES packet states its length, as some cameras write them; FFmpeg
    leaves it unstated by default.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    options = {"omit_video_pes_length": "0"} if lengths else {}
    with av.open(str(path), "w", format="mpegts", options=options) as out:
        stream = out.add_stream("libx264", rate=rate, options={"qp": "0"})
        stream.height, stream.width = pictures[0].shape[:2]
        stream.pix_fmt = "yuv420p"
        data = out.add_data_stream() if records else None
        for k, picture in enumerate(pictures):
            frame = av.VideoFrame.from_ndarray(picture, format="rgb24")
            frame.pts = k
            for packet in stream.encode(frame):
                out.mux(packet)
            if data is not None and records is not None and k in records:
                packet = av.Packet(records[k])
                packet.stream = data
                packet.pts = packet.dts = round(k / rate * 90_000)  # MPEG-TS's 90 kHz clock
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return path


def scene(
    rng: np.random.Generator, height: int, width: int, *, block: int = 24
) -> NDArray[np.uint8]:
    """Return a gray scene of random blocks, its grid shifted at random: its edges move."""
    rows = (np.arange(height) + rng.integers(block)) // block
    cols = (np.arange(width) + rng.integers(block)) // block
    levels = rng.integers(30, 170, size=(rows.max() + 1, cols.max() + 1))
    return np.asarray(levels[rows[:, None], cols[None, :]], dtype=np.uint8)


def hooded(picture: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return a gray picture as RGB with a striped hood across its bottom (:data:`HOOD`)."""
    out = picture.copy()
    top = round(HOOD * picture.shape[0])
    stripes = ((np.arange(picture.shape[1]) // 6) % 2 * 120 + 40).astype(np.uint8)
    out[top:, :] = stripes[None, :]
    return np.ascontiguousarray(np.repeat(out[:, :, None], 3, axis=2))


def card(
    root: Path, camera: str, starts: Sequence[dt.datetime], *, frames: int = 10, rate: int = 5
) -> list[Path]:
    """Write a camera's segments under a card's root, a moving scene under a hood."""
    rng = np.random.default_rng(len(starts) + ord(camera))
    paths = []
    for start in starts:
        pictures = [hooded(scene(rng, 90, 160)) for _ in range(frames)]
        path = root / "video" / camera / segment_name(start, camera)
        paths.append(write_segment(path, pictures, rate=rate))
    return paths


def record(
    utc: dt.datetime, *, valid: bool = True, knots: float = 10.0, course: float = 90.0
) -> bytes:
    """Return a D016 GPS record (:func:`slam_it.eval.dashcam.d016.decode_record`'s layout)."""
    raw = bytearray(d016.RECORD_SIZE)
    struct.pack_into("<6i", raw, 0, utc.hour, utc.minute, utc.second, utc.year, utc.month, utc.day)
    raw[0x18:0x1B] = (b"A" if valid else b"V") + b"NW"
    struct.pack_into("<2d", raw, 0x20, 1234.5678, 9876.5432)  # scrambled, never read
    struct.pack_into("<2f", raw, 0x30, knots, course)
    return bytes(raw)


def segment_name(start: dt.datetime, camera: str) -> str:
    """Return a D016 segment's file name."""
    return f"{start:%Y%m%d_%H%M%S}{camera}.ts"


def pose(rotation: NDArray[np.float64], centre: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return ``T_cam_world`` of a camera with ``R_world_cam`` = `rotation` at `centre`."""
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ centre
    return out


FRONT_FROM_CAR = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
"""``R_front_car``: the car's x forward, y left, z up; the camera's x right, y down, z forward."""


def drive(times: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the car's ``R_world_car`` and position at each time: a winding drive, with bumps.

    It moves at about 8 m/s, its heading swinging back and forth, its pitch and roll rocking
    by a degree or two.
    """
    heading = 0.6 * np.sin(0.21 * times) + 0.3 * np.sin(0.53 * times + 1.0)
    pitch = np.radians(1.5) * np.sin(1.3 * times)
    roll = np.radians(1.0) * np.sin(0.9 * times + 0.5)
    rotations = so3.exp(np.stack([roll, np.zeros_like(times), np.zeros_like(times)], axis=1))
    rotations = (
        so3.exp(np.stack([np.zeros_like(times), pitch, np.zeros_like(times)], axis=1)) @ rotations
    )
    rotations = (
        so3.exp(np.stack([np.zeros_like(times), np.zeros_like(times), heading], axis=1)) @ rotations
    )
    step = np.gradient(times)
    speed = 8.0 + np.sin(0.4 * times)
    position = np.cumsum(
        np.stack([speed * np.cos(heading), speed * np.sin(heading), 0.1 * np.sin(times)], axis=1)
        * step[:, None],
        axis=0,
    )
    return rotations, position


def rig_poses(
    times: NDArray[np.float64],
    camera_from_car: NDArray[np.float64],
    lever: NDArray[np.float64],
    scale: float,
) -> dict[float, NDArray[np.float64]]:
    """Return a camera's ``T_cam_world`` by time, its map in units of `scale` metres.

    Parameters
    ----------
    times : ndarray
        The times.
    camera_from_car : ndarray, shape (3, 3)
        ``R_cam_car``.
    lever : ndarray, shape (3,)
        The camera's centre in the car's frame, in metres.
    scale : float
        Metres per map unit.
    """
    rotations, position = drive(times)
    out = {}
    for t, r, p in zip(times, rotations, position, strict=True):
        world_cam = r @ camera_from_car.T
        out[float(t)] = pose(world_cam, (p + r @ lever) / scale)
    return out


def turn(axis: Sequence[float], degrees: float) -> NDArray[np.float64]:
    """Return a rotation of some degrees about an axis."""
    unit = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    return so3.exp(unit * math.radians(degrees))
