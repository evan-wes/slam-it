"""The EuRoC MAV datasets: stereo frames, their ground-truth poses, and the cameras' models.

Burri et al., "The EuRoC micro aerial vehicle datasets", IJRR 2016
(https://doi.org/10.3929/ethz-b-000690084). A sequence in the ASL format holds ``mav0/``, and
in it, per sensor, a folder with a ``sensor.yaml`` and a ``data.csv``:

- ``cam0`` and ``cam1``: the left and right cameras of a hardware-synchronised stereo pair,
  752x480 8-bit gray PNGs at 20 Hz (``timestamp [ns],filename`` per frame).
- ``state_groundtruth_estimate0``: the body's pose in the world at 200 Hz, ``timestamp [ns],
  p_RS_R (x, y, z) [m], q_RS (w, x, y, z)`` and then velocities and biases: ``T_world_body``.

**Cameras.** Each ``sensor.yaml`` gives a pinhole with radial-tangential distortion
(``intrinsics: [fu, fv, cu, cv]``, ``distortion_coefficients: [k1, k2, p1, p2]``), read here
into the ``radtan`` layout ``[fx, fy, cx, cy, k1, k2, p1, p2, k3]`` with ``k3 = 0``, and the
camera's pose in the body frame, ``T_BS``: ``T_body_cam`` (the design doc's ``T_a_b``).

**Poses.** A frame's ground truth is the left camera's ``T_world_cam = T_world_body
T_body_cam0``, from the body pose nearest in time, if one is within ``max_difference``
seconds; frames without one, or whose right image is missing, are dropped. The quaternions
are Hamilton, stored ``w`` first in the file (unlike TUM).

**Frames and ground truth.** :func:`read_frames` reads what a run consumes: every frame with
both images, and the cameras; it never reads the ground truth, and the native readers mirror
it. :func:`read_sequence` reads what an evaluation scores: the frames that have a pose.
"""

from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slam_it.eval import timestamps
from slam_it.foundations import se3, so3
from slam_it.foundations.arrays import MAX_IMAGE_SIDE
from slam_it.foundations.errors import InvalidInputError

MAX_DIFFERENCE = 0.0025
"""The largest time difference, in seconds, between a frame and its ground-truth pose: half
the ground truth's 200 Hz period."""

NANOSECONDS = 1e9
"""The data files' timestamps per second."""


@dataclass(frozen=True)
class Camera:
    """One EuRoC camera.

    Attributes
    ----------
    intrinsics : ndarray of float64, shape (9,)
        ``radtan`` intrinsics ``[fx, fy, cx, cy, k1, k2, p1, p2, 0]``.
    T_body_cam : ndarray of float64, shape (4, 4)
        The camera's pose in the body frame.
    shape : tuple of int
        Its images' ``(rows, cols)``.
    """

    intrinsics: NDArray[np.float64]
    T_body_cam: NDArray[np.float64]
    shape: tuple[int, int]


def right_from_left(left: Camera, right: Camera) -> NDArray[np.float64]:
    """Return the rigid transform from left-camera to right-camera coordinates.

    Parameters
    ----------
    left, right : Camera
        The stereo pair's cameras, each with its pose in the body frame.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``T_right_left = T_body_right^-1 T_body_left``.
    """
    return np.asarray(se3.inverse(right.T_body_cam) @ left.T_body_cam, dtype=np.float64)


@dataclass(frozen=True)
class Frames:
    """A EuRoC sequence's stereo frames, as a run reads them (no ground truth).

    Attributes
    ----------
    root : pathlib.Path
        The sequence folder (the one holding ``mav0``).
    timestamps : ndarray of float64, shape (F,)
        Each frame's timestamp, in seconds, ascending.
    left, right : list of pathlib.Path
        Each frame's left (``cam0``) and right (``cam1``) image.
    left_camera, right_camera : Camera
        The two cameras.
    """

    root: Path
    timestamps: NDArray[np.float64]
    left: list[Path]
    right: list[Path]
    left_camera: Camera
    right_camera: Camera

    @property
    def T_right_left(self) -> NDArray[np.float64]:  # noqa: N802 -- the frame convention's name
        """The rigid transform from left-camera to right-camera coordinates."""
        return right_from_left(self.left_camera, self.right_camera)


@dataclass(frozen=True)
class Sequence:
    """A EuRoC sequence's stereo frames that have a ground-truth pose.

    Attributes
    ----------
    root : pathlib.Path
        The sequence folder (the one holding ``mav0``).
    timestamps : ndarray of float64, shape (F,)
        Each frame's timestamp, in seconds.
    left, right : list of pathlib.Path
        Each frame's left (``cam0``) and right (``cam1``) image.
    T_world_cam : ndarray of float64, shape (F, 4, 4)
        Each frame's ground-truth pose of the left camera.
    left_camera, right_camera : Camera
        The two cameras.
    """

    root: Path
    timestamps: NDArray[np.float64]
    left: list[Path]
    right: list[Path]
    T_world_cam: NDArray[np.float64]
    left_camera: Camera
    right_camera: Camera

    @property
    def T_right_left(self) -> NDArray[np.float64]:  # noqa: N802 -- the frame convention's name
        """The rigid transform from left-camera to right-camera coordinates."""
        return right_from_left(self.left_camera, self.right_camera)


def _numbers(text: str, key: str, path: Path) -> list[float]:
    """Return the numbers of a ``key: [a, b, ...]`` list in a sensor.yaml.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the key is missing or a value is not a number.
    """
    found = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", text, flags=re.MULTILINE)
    if found is None:
        raise InvalidInputError(f"{path}: no `{key}: [...]` list")
    try:
        return [float(v) for v in found.group(1).replace("\n", " ").split(",") if v.strip()]
    except ValueError as error:
        raise InvalidInputError(f"{path}: `{key}` holds a non-number ({error})") from None


def _setting(text: str, key: str, path: Path) -> str:
    """Return a ``key: value`` setting's value in a sensor.yaml.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the key is missing.
    """
    found = re.search(rf"^\s*{key}:\s*(\S+)", text, flags=re.MULTILINE)
    if found is None:
        raise InvalidInputError(f"{path}: no `{key}` setting")
    return found.group(1)


def read_camera(path: str | Path) -> Camera:
    """Read a camera's ``sensor.yaml``.

    Parameters
    ----------
    path : str or pathlib.Path
        The file, e.g. ``mav0/cam0/sensor.yaml``.

    Returns
    -------
    Camera
        Its intrinsics, pose in the body frame and image shape.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    slam_it.foundations.errors.InvalidInputError
        If a field is missing or malformed, the model is not a radial-tangential pinhole, or
        ``T_BS`` is not a rigid transform.
    """
    file = Path(path)
    # Comments run from '#' to the end of a line; no value holds one.
    text = re.sub(r"#[^\n]*", "", file.read_text())
    if _setting(text, "camera_model", file) != "pinhole":
        raise InvalidInputError(f"{file}: expected camera_model: pinhole")
    if _setting(text, "distortion_model", file) != "radial-tangential":
        raise InvalidInputError(f"{file}: expected distortion_model: radial-tangential")
    pinhole = _numbers(text, "intrinsics", file)
    distortion = _numbers(text, "distortion_coefficients", file)
    resolution = _numbers(text, "resolution", file)
    block = re.search(r"T_BS:(.*?\])", text, flags=re.DOTALL)
    pose = _numbers(block.group(1), "data", file) if block else []
    if (
        len(pinhole) != 4 or len(distortion) != 4 or len(resolution) != 2 or len(pose) != 16
    ):  # the format's fixed lengths
        raise InvalidInputError(
            f"{file}: expected 4 intrinsics, 4 distortion coefficients, a 2-value resolution "
            "and a 16-value T_BS"
        )
    if not all(map(math.isfinite, [*pinhole, *distortion, *resolution, *pose])):
        raise InvalidInputError(f"{file}: expected finite numbers")
    if not all(v == int(v) and 1 <= v <= MAX_IMAGE_SIDE for v in resolution):
        raise InvalidInputError(
            f"{file}: expected a resolution of two integers from 1 to {MAX_IMAGE_SIDE}"
        )
    transform = np.asarray(pose).reshape(4, 4)
    se3.check_transform(transform.reshape(1, 4, 4))
    return Camera(
        intrinsics=np.array([*pinhole, *distortion, 0.0]),
        T_body_cam=transform,
        shape=(int(resolution[1]), int(resolution[0])),
    )


def _frames(folder: Path) -> dict[int, Path]:
    """Return a camera folder's frames, by timestamp in nanoseconds.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a row is not ``timestamp,filename``, or a timestamp repeats.
    """
    path = folder / "data.csv"
    out = {}
    with path.open(newline="") as file:
        for number, row in enumerate(csv.reader(file), start=1):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 2 or not row[0].strip().isdigit():  # two columns
                raise InvalidInputError(f"{path}:{number}: expected `timestamp,filename`")
            stamp = int(row[0])
            if stamp in out:
                raise InvalidInputError(f"{path}:{number}: timestamp {stamp} repeats")
            out[stamp] = folder / "data" / row[1].strip()
    return out


def _ground_truth(path: Path) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
    """Return the ground truth's timestamps (ns, ascending) and ``T_world_body`` matrices.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the file has no rows, a row is not at least 8 finite numbers with a non-zero
        quaternion, or a timestamp repeats.
    """
    stamps, values = [], []
    with path.open(newline="") as file:
        for number, row in enumerate(csv.reader(file), start=1):
            if not row or row[0].lstrip().startswith("#"):
                continue
            try:
                stamps.append(int(row[0]))
                values.append([float(v) for v in row[1:8]])
            except (ValueError, IndexError) as error:
                raise InvalidInputError(f"{path}:{number}: expected numbers ({error})") from None
            if (
                len(values[-1]) != 7
                or not all(map(math.isfinite, values[-1]))
                or not np.linalg.norm(values[-1][3:7]) > 0.0
            ):  # position and quaternion
                raise InvalidInputError(
                    f"{path}:{number}: expected 7 finite values, a non-zero quaternion"
                )
    if not stamps:
        raise InvalidInputError(f"{path}: no ground-truth poses")
    order = np.argsort(stamps, kind="stable")
    times = np.asarray(stamps, dtype=np.int64)[order]
    if np.any(np.diff(times) == 0):
        raise InvalidInputError(f"{path}: a timestamp repeats")
    table = np.asarray(values)[order]
    wxyz = table[:, 3:7] / np.linalg.norm(table[:, 3:7], axis=1, keepdims=True)
    poses = np.tile(np.eye(4), (len(table), 1, 1))
    poses[:, :3, :3] = so3.from_quaternion(wxyz[:, [1, 2, 3, 0]])
    poses[:, :3, 3] = table[:, :3]
    return times, poses


def _stereo_stamps(mav: Path) -> tuple[NDArray[np.int64], dict[int, Path], dict[int, Path]]:
    """Return the timestamps (ns, ascending) with both images, and each camera's frames."""
    left, right = _frames(mav / "cam0"), _frames(mav / "cam1")
    stamps = np.asarray(sorted(set(left) & set(right)), dtype=np.int64)
    if len(stamps) == 0:
        raise InvalidInputError(f"{mav.parent}: no frame has both a left and a right image")
    return stamps, left, right


def read_frames(root: str | Path) -> Frames:
    """Read a EuRoC sequence's stereo frames as a run consumes them, without ground truth.

    Parameters
    ----------
    root : str or pathlib.Path
        The sequence folder, holding ``mav0`` (e.g. ``MH_01_easy``).

    Returns
    -------
    Frames
        Every frame with both a left and a right image, in time order, and the cameras.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a file is malformed, or no frame has both images.
    FileNotFoundError
        If a camera's files are missing.
    """
    mav = Path(root) / "mav0"
    left_camera = read_camera(mav / "cam0" / "sensor.yaml")
    right_camera = read_camera(mav / "cam1" / "sensor.yaml")
    stamps, left, right = _stereo_stamps(mav)
    return Frames(
        root=Path(root),
        timestamps=stamps.astype(np.float64) / NANOSECONDS,
        left=[left[int(s)] for s in stamps],
        right=[right[int(s)] for s in stamps],
        left_camera=left_camera,
        right_camera=right_camera,
    )


def read_sequence(root: str | Path, max_difference: float = MAX_DIFFERENCE) -> Sequence:
    """Read a EuRoC sequence's stereo frames and associate their ground-truth poses.

    Parameters
    ----------
    root : str or pathlib.Path
        The sequence folder, holding ``mav0`` (e.g. ``MH_01_easy``).
    max_difference : float, optional
        The largest frame-to-pose time difference accepted, in seconds.

    Returns
    -------
    Sequence
        The frames that have both images and a pose within `max_difference`, in time order.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `max_difference` is negative, a file is malformed, or no frame has a pose.
    FileNotFoundError
        If a camera's or the ground truth's files are missing.
    """
    if not (np.isfinite(max_difference) and max_difference >= 0.0):
        raise InvalidInputError(f"max_difference: must be non-negative, got {max_difference}")
    mav = Path(root) / "mav0"
    left_camera = read_camera(mav / "cam0" / "sensor.yaml")
    right_camera = read_camera(mav / "cam1" / "sensor.yaml")
    stamps, left, right = _stereo_stamps(mav)
    times, body = _ground_truth(mav / "state_groundtruth_estimate0" / "data.csv")
    nearest = timestamps.nearest(times, stamps)  # in nanoseconds, exactly
    kept = np.abs(times[nearest] - stamps) <= round(max_difference * NANOSECONDS)
    if not np.any(kept):
        raise InvalidInputError(
            f"{root}: no frame has a ground-truth pose within {max_difference} s"
        )
    frames = stamps[kept]
    return Sequence(
        root=Path(root),
        timestamps=frames.astype(np.float64) / NANOSECONDS,
        left=[left[int(s)] for s in frames],
        right=[right[int(s)] for s in frames],
        T_world_cam=body[nearest[kept]] @ left_camera.T_body_cam,
        left_camera=left_camera,
        right_camera=right_camera,
    )
