"""The ZJU-SenseTime VISLAM benchmark: handheld phones' sequences, imported to TUM format.

Li et al., "Survey and evaluation of monocular visual-inertial SLAM algorithms for augmented
reality", Virtual Reality & Intelligent Hardware, 2019 (http://www.zjucvg.net/eval-vislam/).
Sixteen sequences, A0-A7 from a Xiaomi Mi 8 and B0-B7 from an iPhone X, each unzipped into a
folder of sensors, every sensor a folder with a ``sensor.yaml`` and a ``data.csv``:

- ``camera``: ``data.csv`` holds ``t[s], filename`` per frame, the images in ``images/`` (the
  benchmark's page says ``data/``; both are read); ``sensor.yaml`` gives the pinhole
  ``intrinsic: camera: [fx, fy, cx, cy]``, its radial-tangential ``distortion: [k1, k2, p1,
  p2]``, and the camera's pose in the body frame, ``extrinsic: q: [x, y, z, w]`` and ``p``;
- ``groundtruth``: ``t[s], qx, qy, qz, qw, px, py, pz``, its sensor's pose in the world at
  400 Hz, that sensor's pose in the body frame in its own ``sensor.yaml``;
- ``imu`` and ``vicon``, which the import does not read.

Every quaternion is Hamilton, stored ``xyzw``, and normalised before use. The camera and the
ground truth share a clock.

**The import** (:func:`import_sequence`) writes the TUM-format folder the pipeline already
reads (:mod:`slam_it.eval.tum`), so no backend needs a reader of its own: ``rgb/``, each frame a
relative symbolic link to the sequence's image (nothing copied, so the folder keeps its place
beside the sequence's); ``rgb.txt``, stamped to the microsecond; ``camera.txt``, a pinhole, or
``radtan`` if the distortion is not zero; and ``groundtruth.txt``, the camera's pose,
``T_world_cam = T_world_gt T_gt_body T_body_cam``.

Run as a program::

    python -m slam_it.eval.zju SEQUENCE OUT
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
from collections.abc import Sequence
from pathlib import Path, PurePosixPath

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import images, runs, sensor_yaml, tum
from slam_it.foundations import se3, so3
from slam_it.foundations.arrays import MAX_IMAGE_SIDE
from slam_it.foundations.errors import InvalidInputError

IMAGE_FOLDERS = ("images", "data")
"""Where a sequence's camera folder may keep its images: what the sequences hold, then what the
benchmark's page describes."""


def _pose(text: str, path: Path) -> NDArray[np.float64]:
    """Return a sensor.yaml's ``extrinsic`` as a 4x4 transform: the sensor's pose in the body."""
    block = text.split("extrinsic:", 1)
    if len(block) != 2:
        raise InvalidInputError(f"{path}: no `extrinsic` block")
    q = sensor_yaml.numbers(block[1], "q", path)
    p = sensor_yaml.numbers(block[1], "p", path)
    if len(q) != 4 or len(p) != 3 or not all(map(math.isfinite, [*q, *p])) or not any(q):
        raise InvalidInputError(
            f"{path}: expected an extrinsic of a non-zero q (xyzw) and a 3-vector p"
        )
    pose = np.eye(4)
    pose[:3, :3] = so3.from_quaternion(np.asarray(q) / np.linalg.norm(q))
    pose[:3, 3] = p
    return pose


def read_camera(sequence: Path) -> tuple[tum.Camera, NDArray[np.float64]]:
    """Read a sequence's camera: its intrinsics, at its images' size, and its pose in the body.

    Raises
    ------
    FileNotFoundError
        If ``camera/sensor.yaml`` does not exist.
    slam_it.foundations.errors.InvalidInputError
        If the camera is not a radial-tangential pinhole, a list is malformed, or no image says
        its size.
    """
    path = sequence / "camera" / "sensor.yaml"
    text = sensor_yaml.read_text(path)
    if sensor_yaml.setting(text, "camera_model", path) != "pinhole":
        raise InvalidInputError(f"{path}: expected camera_model: pinhole")
    if sensor_yaml.setting(text, "distortion_model", path) != "radtan":
        raise InvalidInputError(f"{path}: expected distortion_model: radtan")
    intrinsic = text.split("intrinsic:", 1)[-1].split("extrinsic:", 1)[0]
    pinhole = sensor_yaml.numbers(intrinsic, "camera", path)
    distortion = sensor_yaml.numbers(intrinsic, "distortion", path)
    if (
        len(pinhole) != 4
        or len(distortion) != 4
        or not all(map(math.isfinite, [*pinhole, *distortion]))
    ):
        raise InvalidInputError(f"{path}: expected 4 finite intrinsics and 4 distortion terms")
    if any(distortion):
        model, values = "radtan", np.array([*pinhole, *distortion, 0.0])
    else:
        model, values = "pinhole", np.array(pinhole)
    width, height = _image_size(sequence)
    return tum.Camera(model, values, width, height, 1000.0, None), _pose(text, path)


def _images(sequence: Path) -> Path:
    """Return the folder a sequence keeps its camera's images in."""
    for name in IMAGE_FOLDERS:
        folder = sequence / "camera" / name
        if folder.is_dir():
            return folder
    raise InvalidInputError(f"{sequence / 'camera'}: no `images` or `data` folder")


def _image_size(sequence: Path) -> tuple[int, int]:
    """Return the size of the sequence's first image, as it decodes, without decoding it."""
    first = min(_images(sequence).glob("*.png"), default=None)
    if first is None:
        raise InvalidInputError(f"{_images(sequence)}: no PNG images")
    width, height = images.size(first)
    if not (1 <= width <= MAX_IMAGE_SIDE and 1 <= height <= MAX_IMAGE_SIDE):
        raise InvalidInputError(f"{first}: {width}x{height}, outside 1 to {MAX_IMAGE_SIDE}")
    return width, height


def _rows(path: Path, columns: int) -> list[list[str]]:
    """Return a data.csv's rows, comments skipped, each of at least `columns` fields."""
    rows = []
    with path.open(newline="") as file:
        for number, row in enumerate(csv.reader(file), start=1):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < columns:
                raise InvalidInputError(f"{path}:{number}: expected {columns} fields")
            rows.append([field.strip() for field in row])
    if not rows:
        raise InvalidInputError(f"{path}: no rows")
    return rows


def _time(text: str, where: str) -> float:
    """Return a timestamp in seconds, refusing one that is not a finite number."""
    try:
        value = float(text)
    except ValueError:
        raise InvalidInputError(f"{where}: expected a time in seconds, got {text!r}") from None
    if not math.isfinite(value):
        raise InvalidInputError(f"{where}: expected a finite time")
    return value


def read_ground_truth(sequence: Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Read the ground truth as the body's poses: times in seconds and ``T_world_body``.

    Raises
    ------
    FileNotFoundError
        If ``groundtruth/data.csv`` or its ``sensor.yaml`` does not exist.
    slam_it.foundations.errors.InvalidInputError
        If a row is not 8 finite numbers with a non-zero quaternion, or the times do not
        increase.
    """
    folder = sequence / "groundtruth"
    gt_body = se3.inverse(
        _pose(sensor_yaml.read_text(folder / "sensor.yaml"), folder / "sensor.yaml")
    )
    times, poses = [], []
    for row in _rows(folder / "data.csv", 8):
        values = [_time(v, str(folder / "data.csv")) for v in row[:8]]
        if not np.linalg.norm(values[1:5]) > 0.0:
            raise InvalidInputError(f"{folder / 'data.csv'}: a zero quaternion at t={values[0]}")
        pose = np.eye(4)
        pose[:3, :3] = so3.from_quaternion(np.asarray(values[1:5]) / np.linalg.norm(values[1:5]))
        pose[:3, 3] = values[5:8]
        times.append(values[0])
        poses.append(pose @ gt_body)
    stamps = np.asarray(times)
    if np.any(np.diff(stamps) <= 0.0):
        raise InvalidInputError(f"{folder / 'data.csv'}: the times do not increase")
    return stamps, np.asarray(poses)


def import_sequence(sequence: str | Path, out: str | Path) -> None:
    """Import a ZJU-SenseTime sequence into a TUM-format folder (the module docstring).

    Parameters
    ----------
    sequence : str or pathlib.Path
        The unzipped sequence, e.g. ``.../zju/B0``.
    out : str or pathlib.Path
        The folder written: new or empty.

    Raises
    ------
    FileNotFoundError
        If a file the import reads does not exist.
    slam_it.foundations.errors.InvalidInputError
        If `out` is not new or empty, or the camera, its frames or the ground truth are
        malformed: a frame's image missing, its times not increasing.
    """
    source, target = Path(sequence), Path(out)
    if target.exists() and any(target.iterdir()):
        raise InvalidInputError(f"{target}: expected a new or empty folder for the import")
    camera, body_cam = read_camera(source)
    folder = _images(source)
    frames = []
    for row in _rows(source / "camera" / "data.csv", 2):
        name = PurePosixPath(row[1])
        if name.is_absolute() or len(name.parts) != 1 or name.name in ("", ".", ".."):
            raise InvalidInputError(
                f"{source / 'camera' / 'data.csv'}: {row[1]!r} is not a file in {folder.name}/"
            )
        time, image = _time(row[0], str(source / "camera" / "data.csv")), folder / name.name
        if not image.is_file():
            raise InvalidInputError(f"{image}: the frame's image is missing")
        frames.append((time, image))
    if any(b[0] <= a[0] for a, b in itertools.pairwise(frames)):
        raise InvalidInputError(f"{source / 'camera' / 'data.csv'}: the times do not increase")
    times, world_body = read_ground_truth(source)
    (target / "rgb").mkdir(parents=True, exist_ok=True)
    listed = []
    for time, image in frames:
        stamp = f"{time:.6f}"
        link = target / "rgb" / f"{stamp}.png"
        link.symlink_to(os.path.relpath(image, link.parent))
        listed.append((stamp, f"rgb/{stamp}.png"))
    tum.write_list(target / "rgb.txt", listed)
    tum.write_camera(target, camera, note=f"imported by slam_it.eval.zju from {source.name}")
    runs.write_trajectory(target / "groundtruth.txt", times, world_body @ body_cam)


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
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.zju")
    parser.add_argument("sequence", type=Path, help="the unzipped sequence, e.g. zju/B0")
    parser.add_argument("out", type=Path, help="the TUM-format folder to write")
    args = parser.parse_args(arguments)
    import_sequence(args.sequence, args.out)
    frames = len(tum.read_frames(args.out).images)
    print(f"{args.out}: {frames} frames of {args.sequence.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
