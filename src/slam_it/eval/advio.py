"""ADVIO: an iPhone's video of walks through malls, a metro station and offices, to TUM format.

Cortés, Solin, Rahtu and Kannala, "ADVIO: An Authentic Dataset for Visual-Inertial Odometry",
ECCV 2018 (https://github.com/AaltoVision/ADVIO; CC BY-NC 4.0). A sequence's folder,
``advio-NN``, holds what this module reads:

- ``iphone/frames.mov``: the video, 1280x720 at 60 fps, stored on its side; its declared
  rotation turns it upright, 720x1280;
- ``iphone/frames.csv``: each frame's time, a line ``seconds,number``, numbering the video's
  frames from 1, on the clock every file of the sequence shares;
- ``ground-truth/pose.csv``: the reference poses at 100 Hz, a line
  ``seconds,x,y,z,qw,qx,qy,qz``, made from the IMU and points fixed by hand on videos and floor
  plans: looser than motion capture.

ADVIO calls the poses the camera's, but their axes are the phone's (x right, y up, z out of the
screen; while walking, their z axis points back along the walk and their y axis up): the
IMU's. The import moves them to the camera by the calibration's ``T_cam_imu``, as ZJU-SenseTime's
body poses are moved (:mod:`slam_it.eval.zju`).

**The calibration** is not in a sequence's archive. ADVIO's repository publishes one per
capture batch, in Kalibr's format (``calibration/iphone-0N.yaml``): the upright frame's pinhole
and radial-tangential distortion (k1, k2, p1, p2), and ``T_cam_imu``. :data:`CALIBRATIONS` says
which file a sequence takes, as ADVIO's calibration README does. Sequences 18 and 19's
(``iphone-01.yaml``) is suspect: its ``cy`` (619.31) differs from that README's table (639.31),
and its tangential ``p2`` (0.0473, in the table too) moves a full frame's edge by about 50
pixels, where the other batches' tangential terms move it by one to three. The file is read as
published; a result on those two sequences says which calibration it used.

**The import** (:func:`import_sequence`) runs the video through
:func:`slam_it.eval.video.import_video` with ADVIO's limits (:data:`MAX_SECONDS`): its frames
timed by ``frames.csv``, kept distorted with a radial-tangential camera, scaled to a short side
of 480 (or as asked) and thinned from 60 to 30 fps, with ``groundtruth.txt``, the poses moved
to the camera, on the frames' clock. The folder is written whole or not at all, and
``source.json`` names it ADVIO's, with the sequence and the calibration file it used.

Run as a program::

    python -m slam_it.eval.advio SEQUENCE CALIBRATIONS OUT [--short-side N]

``CALIBRATIONS`` is the folder holding ADVIO's four calibration files.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import lenses, sensor_yaml, tum, video
from slam_it.foundations import se3, so3
from slam_it.foundations.errors import InvalidInputError

CALIBRATIONS = (
    (range(1, 13), "iphone-02.yaml"),
    (range(13, 18), "iphone-03.yaml"),
    (range(18, 20), "iphone-01.yaml"),
    (range(20, 24), "iphone-04.yaml"),
)
"""Each capture batch's sequences and its calibration file."""

MAX_SECONDS = 600.0
"""The longest video an ADVIO import takes, in seconds: ten minutes, over one and a half times
the longest sequence's (advio-22, 385.9 s by its ``frames.csv``)."""

_RIGID_TOLERANCE = 1e-4
"""How far a calibration's rotation may be from orthonormal: Kalibr prints sixteen digits, and
ADVIO's are orthonormal to 1e-15, so this refuses only a matrix that is not a rotation."""

RATE = 60
"""ADVIO's iPhone frame rate, in frames a second."""

_NAME = re.compile(r"advio-(\d{2})")
"""A sequence's folder name."""


def calibration_name(sequence: Path) -> str:
    """Return the calibration file a sequence takes, from its folder's name, ``advio-NN``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the name is not ``advio-NN`` of a published sequence, 01 to 23.
    """
    found = _NAME.fullmatch(sequence.name)
    number = int(found.group(1)) if found else 0
    for batch, name in CALIBRATIONS:
        if number in batch:
            return name
    raise InvalidInputError(f"{sequence}: expected a folder named advio-01 to advio-23")


def _block(text: str, key: str, path: Path) -> str:
    """Return a YAML file's text after ``key:``, refusing a file without it."""
    _, found, rest = text.partition(f"{key}:")
    if not found:
        raise InvalidInputError(f"{path}: no `{key}`")
    return rest


def read_calibration(path: Path) -> tuple[lenses.RadTan, NDArray[np.float64]]:
    """Read a Kalibr calibration: the upright frames' lens, and ``T_cam_imu``.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If it lacks a field, its distortion is not radial-tangential of four values, or
        ``T_cam_imu`` is not a rigid transform.
    """
    text = sensor_yaml.read_text(path)
    width = int(sensor_yaml.setting(text, "image_width", path))
    height = int(sensor_yaml.setting(text, "image_height", path))
    intrinsics = sensor_yaml.numbers(_block(text, "intrinsics", path), "data", path)
    distortion = _block(text, "distortion", path)
    if sensor_yaml.setting(distortion, "type", path) != "radial-tangential":
        raise InvalidInputError(f"{path}: expected radial-tangential distortion")
    coefficients = sensor_yaml.numbers(distortion, "data", path)
    if len(intrinsics) != 4 or len(coefficients) != 4:
        raise InvalidInputError(f"{path}: expected 4 intrinsics and 4 distortion values")
    rows = re.findall(r"\[([^\]]*)\]", _block(text, "T_cam_imu", path))[:4]
    try:
        matrix = np.array([[float(v) for v in row.split(",")] for row in rows])
    except ValueError:
        raise InvalidInputError(f"{path}: `T_cam_imu` holds a non-number") from None
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise InvalidInputError(f"{path}: expected `T_cam_imu` as four rows of four numbers")
    rotation = matrix[:3, :3]
    orthonormal = np.allclose(rotation @ rotation.T, np.eye(3), atol=_RIGID_TOLERANCE)
    rigid = orthonormal and np.linalg.det(rotation) > 0
    if not rigid or not np.array_equal(matrix[3], [0.0, 0.0, 0.0, 1.0]):
        raise InvalidInputError(f"{path}: `T_cam_imu` is not a rigid transform")
    matrix[:3, :3] = so3.project_to_so3(rotation)  # printed digits' rounding, made exact
    fx, fy, cx, cy = intrinsics
    k1, k2, p1, p2 = coefficients
    lens = lenses.RadTan(lenses.Pinhole(fx, fy, cx, cy, width, height), (k1, k2, p1, p2))
    return lens, matrix


def _rows(path: Path, columns: int) -> list[list[float]]:
    """Return a CSV's rows as `columns` finite numbers each, refusing anything else."""
    rows = []
    with path.open(newline="") as file:
        for number, row in enumerate(csv.reader(file), start=1):
            try:
                values = [float(v) for v in row]
            except ValueError:
                raise InvalidInputError(f"{path}:{number}: expected numbers") from None
            if len(values) != columns or not all(map(math.isfinite, values)):
                raise InvalidInputError(f"{path}:{number}: expected {columns} finite numbers")
            rows.append(values)
    if not rows:
        raise InvalidInputError(f"{path}: no rows")
    return rows


def read_frame_times(sequence: Path) -> list[float]:
    """Read each video frame's time, in seconds, from ``iphone/frames.csv``.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If a row is not a time and its frame's number, the numbers do not count from 1, or the
        times do not increase.
    """
    path = sequence / "iphone" / "frames.csv"
    rows = np.asarray(_rows(path, 2))
    if not np.array_equal(rows[:, 1], np.arange(1, len(rows) + 1)):
        raise InvalidInputError(f"{path}: the frames are not numbered 1, 2, 3, ...")
    if np.any(np.diff(rows[:, 0]) <= 0.0):
        raise InvalidInputError(f"{path}: the times do not increase")
    return [float(t) for t in rows[:, 0]]


def read_ground_truth(
    sequence: Path, cam_imu: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read the ground truth as the camera's poses: times in seconds and ``T_world_cam``.

    Parameters
    ----------
    sequence : pathlib.Path
        The sequence's folder.
    cam_imu : ndarray of float64, shape (4, 4)
        The calibration's ``T_cam_imu``.

    Raises
    ------
    FileNotFoundError
        If ``ground-truth/pose.csv`` does not exist.
    slam_it.foundations.errors.InvalidInputError
        If a row is not 8 finite numbers with a non-zero quaternion, or the times do not
        increase.
    """
    path = sequence / "ground-truth" / "pose.csv"
    table = np.asarray(_rows(path, 8))
    if np.any(np.diff(table[:, 0]) <= 0.0):
        raise InvalidInputError(f"{path}: the times do not increase")
    wxyz = table[:, 4:8]
    norms = np.linalg.norm(wxyz, axis=1)
    if not np.all(norms > 0.0):
        raise InvalidInputError(f"{path}: a zero quaternion")
    world_imu = np.tile(np.eye(4), (len(table), 1, 1))
    world_imu[:, :3, :3] = so3.from_quaternion(np.roll(wxyz / norms[:, None], -1, axis=1))
    world_imu[:, :3, 3] = table[:, 1:4]
    return table[:, 0], world_imu @ se3.inverse(cam_imu)


def import_sequence(
    sequence: str | Path,
    calibrations: str | Path,
    out: str | Path,
    short_side: int = video.SHORT_SIDE,
) -> None:
    """Import an ADVIO sequence into a TUM-format folder (the module docstring).

    Parameters
    ----------
    sequence : str or pathlib.Path
        The unzipped sequence, e.g. ``.../advio/advio-01``.
    calibrations : str or pathlib.Path
        The folder holding ADVIO's calibration files.
    out : str or pathlib.Path
        The folder written: new or empty.
    short_side : int, optional
        The short side frames are scaled down to (default:
        :data:`slam_it.eval.video.SHORT_SIDE`).

    Raises
    ------
    FileNotFoundError
        If a file the import reads does not exist.
    slam_it.foundations.errors.InvalidInputError
        If a file is malformed, ``frames.csv`` does not time each frame once, or the video is
        refused as :func:`slam_it.eval.video.import_video` refuses one.
    """
    folder = Path(sequence)
    upright, cam_imu = read_calibration(Path(calibrations) / calibration_name(folder))
    clock = read_frame_times(folder)
    times, poses = read_ground_truth(folder, cam_imu)
    recording = folder / "iphone" / "frames.mov"
    stored = upright.turned(-video.quarter_turns(recording))
    limits = video.Limits(MAX_SECONDS, round(MAX_SECONDS * RATE), short_side)
    chosen = lenses.Chosen(stored, "given")
    named = {"kind": "advio", "sequence": folder.name, "calibration": calibration_name(folder)}
    video.import_video(
        recording, out, chosen, limits=limits, clock=clock, truth=(times, poses), source=named
    )


def main(arguments: Sequence[str]) -> int:
    """Import a sequence from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a one-line summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.advio")
    parser.add_argument("sequence", type=Path, help="the unzipped sequence, advio-NN")
    parser.add_argument("calibrations", type=Path, help="the folder of ADVIO's calibrations")
    parser.add_argument("out", type=Path, help="the TUM-format folder to write")
    parser.add_argument(
        "--short-side", type=int, default=video.SHORT_SIDE,
        help=f"the short side frames are scaled down to (default: {video.SHORT_SIDE})",
    )  # fmt: skip
    args = parser.parse_args(arguments)
    import_sequence(args.sequence, args.calibrations, args.out, args.short_side)
    frames = tum.read_frames(args.out)
    camera = tum.read_camera(args.out)
    print(
        f"{args.out}: {len(frames.images)} frames, {camera.width}x{camera.height} "
        f"({camera.model}), of {args.sequence.name}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
