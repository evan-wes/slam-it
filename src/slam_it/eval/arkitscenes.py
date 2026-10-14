"""ARKitScenes' raw captures: an iPad's colour frames with LiDAR depth, imported to TUM format.

Baruch et al., "ARKitScenes: A Diverse Real-World Dataset for 3D Indoor Scene Understanding
Using Mobile RGB-D Data", NeurIPS 2021 Datasets and Benchmarks
(https://github.com/apple/ARKitScenes). A raw capture's folder, as its download script unpacks
the assets milestone 2A reads, holds, each file named ``<video_id>_<seconds>.<ext>``:

- ``vga_wide/``: the wide camera's colour frames at 640x480, 30 Hz;
- ``vga_wide_intrinsics/``: each frame's ``.pincam``, one line ``width height fx fy cx cy``;
- ``lowres_depth/`` and ``confidence/``: LiDAR depth (16-bit, millimetres) and its confidence
  (8-bit, 0 to 2) at 256x192, 60 Hz, registered to the wide camera (:mod:`slam_it.eval.lidar`);
- ``lowres_wide.traj``: ARKit's poses, a line each, ``seconds rx ry rz tx ty tz``: the
  world-to-camera rotation (axis-angle, radians) and translation (metres), which ARKitScenes'
  own loader inverts into the camera's pose, in this package's camera axes (z forward, y down).

**The import** (:func:`import_capture`) writes the TUM-format folder an RGB-D run reads: ``rgb/``,
each frame a relative symbolic link to the capture's; ``depth/``, each frame's nearest depth map
within :data:`slam_it.eval.tum.MAX_DIFFERENCE`, kept by confidence and brought to 640x480
(:func:`slam_it.eval.lidar.registered_depth`); ``rgb.txt`` and ``depth.txt``; ``camera.txt``, a
pinhole of the frames' median intrinsics, its ``depth_scale`` and ``bf`` LiDAR's;
``groundtruth.txt``, ARKit's poses inverted and interpolated at each frame's time
(:func:`at_frames`); and ``source.json``, the capture and the least confidence kept. The folder
is written whole or not at all (:func:`slam_it.eval.tum.staged`). ARKit's poses are its own
visual-inertial estimate, not independent truth: an evaluation names them a reference.

**The reference at the frames' times.** ARKit's poses come at 10 Hz, half a frame between the
30 Hz frames. Paired by time, as the scoring pairs a run with its reference
(:func:`slam_it.eval.tum.read_sequence`), each pose would go to the two frames either side of
it, half a frame off: the relative error would then compare frames holding one pose, and so
measure the camera's own motion. Interpolated at each frame's time instead, every frame has its
own reference pose. Leaving each of the ten captures' poses out in turn and interpolating it
from its neighbours 0.2 s apart misses it by a median of 1.6 mm and 0.28 degrees (95th
percentile 4.4 mm and 0.81 degrees); a frame's span is half that, and such errors grow with the
span's square.

ARKit reports each frame's own intrinsics, which follow the lens's focus. On captures 42444946
and 42446016, ``fx`` spans 522.2 to 545.0 and 522.7 to 545.6, against medians of 532.3 and
534.7, and 95.7% and 98.1% of frames are within 1% of the median. The fixed camera of their
median reads the rest as a small zoom.

Run as a program::

    python -m slam_it.eval.arkitscenes CAPTURE OUT [--min-confidence {0,1,2}]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import images, lidar, runs, timestamps, tum
from slam_it.foundations import se3, so3
from slam_it.foundations.arrays import MAX_IMAGE_SIDE
from slam_it.foundations.errors import InvalidInputError

TRAJECTORY = "lowres_wide.traj"
"""The capture's file of ARKit's poses."""

MAX_GAP = 0.15
"""The longest span between two of ARKit's poses, in seconds, that a frame's reference pose is
interpolated over: one and a half times their 0.1 s spacing. A frame in a longer gap has none."""


def _stamped(folder: Path, suffix: str) -> dict[float, Path]:
    """Return a folder's ``<video_id>_<seconds><suffix>`` files by their time in seconds."""
    if not folder.is_dir():
        raise InvalidInputError(f"{folder}: the capture has no such folder")
    found: dict[float, Path] = {}
    for path in folder.glob(f"*{suffix}"):
        _, _, stamp = path.name.removesuffix(suffix).partition("_")
        try:
            seconds = float(stamp)
        except ValueError:
            raise InvalidInputError(f"{path}: expected <video_id>_<seconds>{suffix}") from None
        if not math.isfinite(seconds):
            raise InvalidInputError(f"{path}: expected a finite time")
        found[seconds] = path
    if not found:
        raise InvalidInputError(f"{folder}: no {suffix} files")
    return found


def read_intrinsics(capture: Path) -> tum.Camera:
    """Read the frames' intrinsics: their median, at their size.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a ``.pincam`` is not six finite numbers, the frames differ in size, or the size is
        outside 1 to :data:`~slam_it.foundations.arrays.MAX_IMAGE_SIDE`.
    """
    rows = []
    for path in _stamped(capture / "vga_wide_intrinsics", ".pincam").values():
        try:
            values = [float(v) for v in path.read_text().split()]
        except ValueError:
            raise InvalidInputError(f"{path}: expected six numbers") from None
        if len(values) != 6 or not all(map(math.isfinite, values)):
            raise InvalidInputError(f"{path}: expected six finite numbers")
        rows.append(values)
    table = np.asarray(rows)
    if np.any(table[:, :2] != table[0, :2]):
        raise InvalidInputError(f"{capture / 'vga_wide_intrinsics'}: the frames differ in size")
    width, height = int(table[0, 0]), int(table[0, 1])
    if not (1 <= width <= MAX_IMAGE_SIDE and 1 <= height <= MAX_IMAGE_SIDE):
        raise InvalidInputError(f"{capture}: frames of {width}x{height}")
    pinhole = np.median(table[:, 2:6], axis=0)
    return tum.Camera("pinhole", pinhole, width, height, lidar.DEPTH_SCALE, lidar.bf(pinhole[0]))


def read_trajectory(capture: Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read ARKit's poses as the camera's: times in seconds and ``T_world_cam``.

    Raises
    ------
    FileNotFoundError
        If the capture has no :data:`TRAJECTORY`.
    slam_it.foundations.errors.InvalidInputError
        If a line is not seven finite numbers, or the times do not increase.
    """
    path = capture / TRAJECTORY
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        try:
            values = [float(v) for v in line.split()]
        except ValueError:
            raise InvalidInputError(f"{path}:{number}: expected numbers") from None
        if len(values) != 7 or not all(map(math.isfinite, values)):
            raise InvalidInputError(f"{path}:{number}: expected seven finite numbers")
        rows.append(values)
    if not rows:
        raise InvalidInputError(f"{path}: no poses")
    table = np.asarray(rows)
    if np.any(np.diff(table[:, 0]) <= 0.0):
        raise InvalidInputError(f"{path}: the times do not increase")
    cam_world = np.tile(np.eye(4), (len(table), 1, 1))
    cam_world[:, :3, :3] = so3.exp(table[:, 1:4])
    cam_world[:, :3, 3] = table[:, 4:7]
    return table[:, 0], se3.inverse(cam_world)


def at_frames(
    times: NDArray[np.float64], poses: NDArray[np.float64], frame_times: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Interpolate the reference at each frame's time, between poses at most :data:`MAX_GAP` apart.

    The position along a straight line, the rotation along the shortest arc between the two
    poses either side (``R0 exp(s log(R0^T R1))``), at the frame's share ``s`` of the span. A
    frame before the first pose, after the last, or in a longer gap has none.

    Parameters
    ----------
    times : ndarray of float64, shape (G,)
        The reference's times, ascending.
    poses : ndarray of float64, shape (G, 4, 4)
        Its ``T_world_cam``.
    frame_times : ndarray of float64, shape (F,)
        The frames' times.

    Returns
    -------
    tuple of ndarray
        The frames' times that have a reference pose, and the pose at each.
    """
    frames = np.asarray(frame_times, dtype=np.float64)
    inside = (frames >= times[0]) & (frames <= times[-1])
    before = np.clip(np.searchsorted(times, frames, side="right") - 1, 0, max(len(times) - 2, 0))
    after = np.minimum(before + 1, len(times) - 1)
    span = times[after] - times[before]
    kept = inside & (span <= MAX_GAP)
    first, second, at = before[kept], after[kept], frames[kept]
    share = np.divide(at - times[first], span[kept], out=np.zeros(len(at)), where=span[kept] > 0.0)
    start, end = poses[first, :3, :3], poses[second, :3, :3]
    turn = so3.log(np.transpose(start, (0, 2, 1)) @ end)
    out = np.tile(np.eye(4), (len(at), 1, 1))
    out[:, :3, :3] = start @ so3.exp(share[:, None] * turn)
    out[:, :3, 3] = (1.0 - share[:, None]) * poses[first, :3, 3] + share[:, None] * poses[
        second, :3, 3
    ]
    return at, out


def import_capture(capture: str | Path, out: str | Path, min_confidence: int = lidar.HIGH) -> None:
    """Import an ARKitScenes raw capture into a TUM-format RGB-D folder (the module docstring).

    Parameters
    ----------
    capture : str or pathlib.Path
        The capture's folder, its assets unpacked.
    out : str or pathlib.Path
        The folder written: new or empty.
    min_confidence : int, optional
        The least LiDAR confidence kept (:data:`slam_it.eval.lidar.HIGH` by default).

    Raises
    ------
    FileNotFoundError
        If a file the import reads does not exist.
    slam_it.foundations.errors.InvalidInputError
        If `out` is not new or empty, or an asset is malformed.
    """
    source = Path(capture)
    camera = read_intrinsics(source)
    times, poses = read_trajectory(source)
    frames = _stamped(source / "vga_wide", ".png")
    depths = _stamped(source / "lowres_depth", ".png")
    confidences = _stamped(source / "confidence", ".png")
    depth_times = np.array(sorted(depths))
    with tum.staged(out) as target:
        rgb, listed_depth = [], []
        for time in sorted(frames):
            stamp = f"{time:.6f}"
            link = target / "rgb" / f"{stamp}.png"
            link.parent.mkdir(parents=True, exist_ok=True)
            link.symlink_to(os.path.relpath(frames[time], link.parent))
            rgb.append((stamp, f"rgb/{stamp}.png"))
            nearest = float(depth_times[timestamps.nearest(depth_times, np.array([time]))[0]])
            if abs(nearest - time) > tum.MAX_DIFFERENCE or nearest not in confidences:
                continue
            try:
                depth = lidar.registered_depth(
                    images.read_depth(depths[nearest]),
                    images.read_gray(confidences[nearest]),
                    (camera.width, camera.height),
                    min_confidence,
                )
            except InvalidInputError as error:
                raise InvalidInputError(f"{depths[nearest]}: {error}") from None
            images.write_depth(target / "depth" / f"{stamp}.png", depth)
            listed_depth.append((stamp, f"depth/{stamp}.png"))
        tum.write_list(target / "rgb.txt", rgb)
        tum.write_list(target / "depth.txt", listed_depth)
        tum.write_camera(
            target, camera, note=f"imported by slam_it.eval.arkitscenes from {source.name}"
        )
        stamped = np.array([float(stamp) for stamp, _ in rgb])  # as rgb.txt has them
        runs.write_trajectory(target / "groundtruth.txt", *at_frames(times, poses, stamped))
        tum.write_source(
            target,
            {
                "kind": "arkitscenes",
                "capture": source.name,
                "depth": {"min_confidence": min_confidence},
            },
        )


def main(arguments: Sequence[str]) -> int:
    """Import a capture from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a one-line summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.arkitscenes")
    parser.add_argument("capture", type=Path, help="the capture's folder, its assets unpacked")
    parser.add_argument("out", type=Path, help="the TUM-format folder to write")
    parser.add_argument(
        "--min-confidence", type=int, default=lidar.HIGH, choices=(0, lidar.MEDIUM, lidar.HIGH),
        help="the least LiDAR confidence kept (default: high)",
    )  # fmt: skip
    args = parser.parse_args(arguments)
    import_capture(args.capture, args.out, args.min_confidence)
    frames = tum.read_frames(args.out)
    with_depth = sum(depth is not None for depth in frames.depths)
    print(
        f"{args.out}: {len(frames.images)} frames, {with_depth} with depth, of {args.capture.name}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
