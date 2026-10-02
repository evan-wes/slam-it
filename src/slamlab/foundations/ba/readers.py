"""Read bundle-adjustment problems from standard formats: BAL and COLMAP's text model.

**BAL** (Bundle Adjustment in the Large, Agarwal et al. 2010). Each camera is a Rodrigues
rotation, a translation, a focal length and two radial terms; it looks down ``-z`` with ``y``
up and the image origin at the centre: ``P = R X + t``, ``p = -P / P_z``,
``u = f (1 + k1 |p|² + k2 |p|⁴) p``. Flipping the camera's ``y`` and ``z`` axes,
``F = diag(1, -1, -1)``, turns that into this package's convention (``z`` forward, ``y`` down,
design doc "Conventions"): ``T_cam_world = [F R, F t]``, pixels ``(x, -y)``, and the
``"radial"`` model ``[f, 0, 0, k1, k2]``. As in BAL and Ceres's BAL solver, ``f, k1, k2`` are
free and the principal point is fixed.

**COLMAP text** (``cameras.txt``, ``images.txt``, ``points3D.txt``). Poses are world-to-camera
Hamilton quaternions ``(qw, qx, qy, qz)`` and translations. COLMAP puts the centre of the
top-left pixel at ``(0.5, 0.5)``, so pixels and principal points shift by ``-0.5``. Its camera
models map onto this package's:

- ``SIMPLE_PINHOLE``, ``SIMPLE_RADIAL``, ``RADIAL`` → ``"radial"`` (missing terms zero);
- ``PINHOLE`` → ``"pinhole"``;
- ``PINHOLE`` with ``OPENCV`` → ``"radtan"`` (missing terms zero).

A reconstruction mixing families, or using another model, is rejected. Every parameter
starts fixed. :attr:`ColmapModel.refinable` marks the ones the source model has, for the
caller to free.
"""

from __future__ import annotations

import bz2
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import se3, so3
from slamlab.foundations.ba.problem import Array, BAProblem, Mask
from slamlab.foundations.errors import InvalidInputError

FLIP = np.diag([1.0, -1.0, -1.0])
"""BAL's camera frame to this package's: flip ``y`` and ``z``."""

COLMAP_PIXEL_OFFSET = 0.5
"""COLMAP's pixel-centre coordinate minus this package's (OpenCV's)."""

_RADIAL_FAMILY = {
    "SIMPLE_PINHOLE": [0, 1, 2],
    "SIMPLE_RADIAL": [0, 1, 2, 3],
    "RADIAL": [0, 1, 2, 3, 4],
}
"""For each COLMAP model mapped onto ``"radial"``: where its parameters go."""

_RADTAN_FAMILY = {"PINHOLE": [0, 1, 2, 3], "OPENCV": [0, 1, 2, 3, 4, 5, 6, 7]}
"""For each COLMAP model mapped onto ``"radtan"``: where its parameters go."""


def read_bal(path: str | Path) -> BAProblem:
    """Read a BAL problem (plain text, or ``.bz2``).

    Parameters
    ----------
    path : str or Path
        The file.

    Returns
    -------
    BAProblem
        The problem in this package's conventions: ``"radial"`` cameras, one intrinsics group
        per camera, ``f, k1, k2`` free and the principal point fixed.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the file is truncated or its counts do not match its contents.
    """
    opener = bz2.open if str(path).endswith(".bz2") else open
    with opener(path, "rt") as handle:
        values = handle.read().split()
    try:
        n_cams, n_points, n_obs = (int(v) for v in values[:3])
        numbers = np.asarray(values[3:], dtype=np.float64)
    except ValueError as error:
        raise InvalidInputError(f"BAL file {path}: not a BAL problem ({error})") from error
    expected = 4 * n_obs + 9 * n_cams + 3 * n_points
    if numbers.size != expected:
        raise InvalidInputError(
            f"BAL file {path}: expected {expected} numbers, found {numbers.size}"
        )
    obs = numbers[: 4 * n_obs].reshape(n_obs, 4)
    cams = numbers[4 * n_obs : 4 * n_obs + 9 * n_cams].reshape(n_cams, 9)
    points = numbers[4 * n_obs + 9 * n_cams :].reshape(n_points, 3)
    t_cam_world = np.tile(np.eye(4), (n_cams, 1, 1))
    t_cam_world[:, :3, :3] = FLIP @ so3.exp(cams[:, :3])
    t_cam_world[:, :3, 3] = cams[:, 3:6] @ FLIP.T
    zeros = np.zeros(n_cams)
    intrinsics = np.stack([cams[:, 6], zeros, zeros, cams[:, 7], cams[:, 8]], axis=1)
    fixed = np.zeros((n_cams, 5), dtype=bool)
    fixed[:, 1:3] = True
    return BAProblem.create(
        "radial",
        intrinsics,
        se3.inverse(t_cam_world),
        points,
        obs[:, 2:] * [1.0, -1.0],
        obs[:, 0].astype(np.int64),
        obs[:, 1].astype(np.int64),
        intrinsics_idx=np.arange(n_cams),
        fixed_intrinsics=fixed,
    )


@dataclass(frozen=True)
class ColmapModel:
    """A COLMAP reconstruction as a bundle-adjustment problem, with its identifiers.

    Attributes
    ----------
    problem : BAProblem
        The problem, every parameter fixed.
    refinable : ndarray of bool, shape (K, P)
        The intrinsics the source camera model has (the others are padding, zero).
    camera_ids, image_ids, point_ids : ndarray of int64
        COLMAP's identifiers, in the order of the problem's intrinsics groups, poses and
        points.
    image_names : list of str
        The images' names, in pose order.
    """

    problem: BAProblem
    refinable: Mask
    camera_ids: NDArray[np.int64]
    image_ids: NDArray[np.int64]
    point_ids: NDArray[np.int64]
    image_names: list[str]


def _lines(path: Path, *, keep_blank: bool = False) -> list[list[str]]:
    """Return a COLMAP text file's non-comment lines, split into fields.

    ``images.txt`` alternates a header line with a keypoint line that is blank for an image
    with no keypoints, so there blank lines are kept (all but a trailing one).
    """
    with path.open() as handle:
        rows = [line.split() for line in handle if not line.startswith("#")]
    if keep_blank:
        return rows[:-1] if rows and not rows[-1] and len(rows) % 2 else rows
    return [row for row in rows if row]


def _cameras(rows: list[list[str]]) -> tuple[str, Array, Mask, NDArray[np.int64]]:
    """Map COLMAP cameras onto one of this package's models."""
    models = {row[1] for row in rows}
    if models <= set(_RADIAL_FAMILY):
        name, family, size = "radial", _RADIAL_FAMILY, 5
    elif models == {"PINHOLE"}:
        name, family, size = "pinhole", {"PINHOLE": [0, 1, 2, 3]}, 4
    elif models <= set(_RADTAN_FAMILY):
        name, family, size = "radtan", _RADTAN_FAMILY, 9
    else:
        raise InvalidInputError(f"COLMAP cameras: unsupported model mix {sorted(models)}")
    intrinsics = np.zeros((len(rows), size))
    refinable = np.zeros((len(rows), size), dtype=bool)
    for k, row in enumerate(rows):
        slots = family[row[1]]
        intrinsics[k, slots] = [float(v) for v in row[4 : 4 + len(slots)]]
        refinable[k, slots] = True
    centre = slice(1, 3) if name == "radial" else slice(2, 4)
    intrinsics[:, centre] -= COLMAP_PIXEL_OFFSET
    return name, intrinsics, refinable, np.array([int(row[0]) for row in rows], dtype=np.int64)


def read_colmap_text(directory: str | Path) -> ColmapModel:
    """Read a COLMAP text reconstruction (``cameras.txt``, ``images.txt``, ``points3D.txt``).

    Parameters
    ----------
    directory : str or Path
        The directory holding the three files.

    Returns
    -------
    ColmapModel
        The problem and COLMAP's identifiers.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a camera model is unsupported or the models mix families.
    """
    root = Path(directory)
    model, intrinsics, refinable, camera_ids = _cameras(_lines(root / "cameras.txt"))
    group_of = {cid: k for k, cid in enumerate(camera_ids)}
    image_rows = _lines(root / "images.txt", keep_blank=True)
    headers, keypoint_rows = image_rows[0::2], image_rows[1::2]
    point_rows = _lines(root / "points3D.txt")
    point_ids = np.array([int(row[0]) for row in point_rows], dtype=np.int64)
    point_index = {pid: k for k, pid in enumerate(point_ids)}
    t_cam_world = np.tile(np.eye(4), (len(headers), 1, 1))
    cam_idx, pt_idx, pixels = [], [], []
    for c, (header, keypoints) in enumerate(zip(headers, keypoint_rows, strict=True)):
        qw, qx, qy, qz, tx, ty, tz = (float(v) for v in header[1:8])
        t_cam_world[c, :3, :3] = so3.from_quaternion([qx, qy, qz, qw])
        t_cam_world[c, :3, 3] = [tx, ty, tz]
        triples = np.asarray(keypoints, dtype=np.float64).reshape(-1, 3)
        seen = triples[:, 2] >= 0
        cam_idx += [c] * int(np.count_nonzero(seen))
        pt_idx += [point_index[int(pid)] for pid in triples[seen, 2]]
        pixels.append(triples[seen, :2] - COLMAP_PIXEL_OFFSET)
    problem = BAProblem.create(
        model,
        intrinsics,
        se3.inverse(t_cam_world),
        np.array([[float(v) for v in row[1:4]] for row in point_rows]).reshape(-1, 3),
        np.concatenate(pixels) if pixels else np.zeros((0, 2)),
        np.asarray(cam_idx, dtype=np.int64),
        np.asarray(pt_idx, dtype=np.int64),
        intrinsics_idx=np.array([group_of[int(h[8])] for h in headers], dtype=np.int64),
    )
    return ColmapModel(
        problem=problem,
        refinable=refinable,
        camera_ids=camera_ids,
        image_ids=np.array([int(h[0]) for h in headers], dtype=np.int64),
        point_ids=point_ids,
        image_names=[h[9] for h in headers],
    )
