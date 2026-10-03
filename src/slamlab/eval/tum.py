"""The TUM RGB-D benchmark: colour frames, their ground-truth poses, and the cameras' models.

Sturm et al., "A benchmark for the evaluation of RGB-D SLAM systems", IROS 2012
(https://cvg.cit.tum.de/data/datasets/rgbd-dataset). A sequence folder holds ``rgb.txt``
(``timestamp filename`` per colour frame), ``groundtruth.txt`` (``timestamp tx ty tz qx qy qz
qw`` from a motion-capture system at about 100 Hz), and the images.

**Poses.** The ground truth is the camera's optical centre and orientation in the
motion-capture world, ``T_world_cam`` with the camera looking along +z (the design doc's
convention). Its quaternions are Hamilton, stored xyzw like ours, and printed to four
decimals, so they are normalised before use.

**Association.** Colour frames and ground truth are sampled independently. Each frame takes
the ground-truth sample nearest in time, if one is within ``max_difference`` seconds (TUM's
own ``associate.py`` uses 0.02 s); frames without one are dropped.

**Intrinsics.** The ROS default for each sensor (``freiburg1``, ``2``, ``3``) is published as a
pinhole with Brown-Conrady (radial-tangential) distortion. :data:`INTRINSICS` holds them in
the ``radtan`` layout ``[fx, fy, cx, cy, k1, k2, p1, p2, k3]``; ``freiburg3`` images are
already undistorted.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import so3

INTRINSICS: dict[str, NDArray[np.float64]] = {
    "freiburg1": np.array([517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]),
    "freiburg2": np.array([520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172]),
    "freiburg3": np.array([535.4, 539.2, 320.1, 247.6, 0.0, 0.0, 0.0, 0.0, 0.0]),
}
"""Each sensor's colour camera, as ``radtan`` intrinsics (TUM's published calibration)."""

MAX_DIFFERENCE = 0.02
"""The largest time difference, in seconds, between a frame and its ground-truth pose."""


@dataclass(frozen=True)
class Sequence:
    """A TUM sequence's colour frames that have a ground-truth pose.

    Attributes
    ----------
    root : pathlib.Path
        The sequence folder.
    sensor : str
        ``"freiburg1"``, ``"freiburg2"`` or ``"freiburg3"``, from the folder's name.
    timestamps : ndarray of float64, shape (F,)
        Each frame's timestamp, in seconds.
    images : list of pathlib.Path
        Each frame's colour image.
    T_world_cam : ndarray of float64, shape (F, 4, 4)
        Each frame's ground-truth pose.
    """

    root: Path
    sensor: str
    timestamps: NDArray[np.float64]
    images: list[Path]
    T_world_cam: NDArray[np.float64]

    @property
    def intrinsics(self) -> NDArray[np.float64]:
        """The sensor's ``radtan`` intrinsics."""
        return INTRINSICS[self.sensor].copy()


def _table(path: Path, columns: int) -> list[tuple[int, list[str]]]:
    """Return a TUM text file's rows with their line numbers, skipping comments and blanks.

    Raises
    ------
    ValueError
        If a row has fewer than `columns` fields.
    """
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) < columns:
            raise ValueError(f"{path}:{number}: expected {columns} fields, got {len(fields)}")
        rows.append((number, fields))
    return rows


def _poses(path: Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the ground truth's timestamps (ascending) and ``T_world_cam`` matrices.

    Raises
    ------
    ValueError
        If the file has no rows, or a row is not 8 finite numbers with a non-zero quaternion.
    """
    rows = _table(path, 8)
    if not rows:
        raise ValueError(f"{path}: no ground-truth poses")
    values = np.empty((len(rows), 8))
    for k, (number, fields) in enumerate(rows):
        try:
            values[k] = [float(v) for v in fields[:8]]
        except ValueError as error:
            raise ValueError(f"{path}:{number}: expected numbers ({error})") from None
        if not np.all(np.isfinite(values[k])) or not np.linalg.norm(values[k, 4:8]) > 0.0:
            raise ValueError(f"{path}:{number}: expected finite values and a non-zero quaternion")
    values = values[np.argsort(values[:, 0], kind="stable")]
    quaternions = values[:, 4:8] / np.linalg.norm(values[:, 4:8], axis=1, keepdims=True)
    poses = np.tile(np.eye(4), (len(values), 1, 1))
    poses[:, :3, :3] = so3.from_quaternion(quaternions)
    poses[:, :3, 3] = values[:, 1:4]
    return values[:, 0], poses


def _nearest(times: NDArray[np.float64], stamps: NDArray[np.float64]) -> NDArray[np.int64]:
    """Return, for each stamp, the index of the nearest of the ascending `times`."""
    if len(times) == 1:
        return np.zeros(len(stamps), dtype=np.int64)
    right = np.clip(np.searchsorted(times, stamps), 1, len(times) - 1)
    left = right - 1
    closer_left = np.abs(times[left] - stamps) <= np.abs(times[right] - stamps)
    return np.asarray(np.where(closer_left, left, right), dtype=np.int64)


def read_sequence(root: str | Path, max_difference: float = MAX_DIFFERENCE) -> Sequence:
    """Read a TUM RGB-D sequence's colour frames and associate their ground-truth poses.

    Parameters
    ----------
    root : str or pathlib.Path
        The sequence folder, e.g. ``rgbd_dataset_freiburg1_xyz``.
    max_difference : float, optional
        The largest frame-to-pose time difference accepted, in seconds.

    Returns
    -------
    Sequence
        The frames that have a pose within `max_difference`, in time order.

    Raises
    ------
    ValueError
        If the folder's name does not say its sensor, `max_difference` is negative, a file is
        malformed (named with its line), or no frame has a pose.
    FileNotFoundError
        If ``rgb.txt`` or ``groundtruth.txt`` is missing.
    """
    if not (np.isfinite(max_difference) and max_difference >= 0.0):
        raise ValueError(f"max_difference: must be non-negative, got {max_difference}")
    folder = Path(root)
    sensor = next((s for s in INTRINSICS if s in folder.name), None)
    if sensor is None:
        raise ValueError(f"{folder.name}: expected 'freiburg1', '2' or '3' in the folder's name")
    frames = _table(folder / "rgb.txt", 2)
    truth_times, truth_poses = _poses(folder / "groundtruth.txt")
    try:
        stamps = np.array([float(fields[0]) for _, fields in frames])
    except ValueError as error:
        raise ValueError(f"{folder / 'rgb.txt'}: expected numeric timestamps ({error})") from None
    nearest = _nearest(truth_times, stamps)
    kept = np.flatnonzero(np.abs(truth_times[nearest] - stamps) <= max_difference)
    if kept.size == 0:
        raise ValueError(f"{folder}: no frame has a ground-truth pose within {max_difference} s")
    images = [folder / frames[k][1][1] for k in kept]
    return Sequence(folder, sensor, stamps[kept], images, truth_poses[nearest[kept]])
