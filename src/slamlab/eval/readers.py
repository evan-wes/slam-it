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
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import TypeVar

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import camera, se3, so3
from slamlab.foundations.ba import BAProblem
from slamlab.foundations.errors import InvalidInputError

T = TypeVar("T")

Array = NDArray[np.float64]
Mask = NDArray[np.bool_]

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
    with opener(path, "rt", encoding="ascii") as handle:
        values = handle.read().split()
    try:
        n_cams, n_points, n_obs = (int(v) for v in values[:3])
        numbers = np.asarray(values[3:], dtype=np.float64)
    except ValueError as error:
        raise InvalidInputError(f"BAL file {path}: not a BAL problem ({error})") from error
    if min(n_cams, n_points, n_obs) < 0:
        raise InvalidInputError(f"BAL file {path}: negative counts in the header")
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
    fixed = np.zeros((n_cams, camera.MODELS["radial"][1]), dtype=bool)
    fixed[:, list(camera.CENTRE_INDICES["radial"])] = True
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


def _lines(path: Path) -> list[tuple[int, str]]:
    """Return a COLMAP text file's non-comment lines with their line numbers, unsplit.

    Blank lines are kept: in ``images.txt`` an image with no keypoints has a blank keypoint
    line after its header.
    """
    with path.open(encoding="utf-8") as handle:
        return [
            (number, line.rstrip("\n"))
            for number, line in enumerate(handle, start=1)
            if not line.startswith("#")
        ]


def _floats(values: list[str]) -> list[float]:
    """Parse a list of fields as floats."""
    return [float(v) for v in values]


def _parse(path: Path, number: int, parse: Callable[[], T]) -> T:
    """Run ``parse`` on one line, turning a malformed line into an error naming it."""
    try:
        return parse()
    except (ValueError, IndexError, KeyError) as error:
        raise InvalidInputError(f"{path.name}:{number}: malformed or dangling ({error})") from error


def _cameras(path: Path) -> tuple[str, Array, Mask, NDArray[np.int64]]:
    """Map COLMAP cameras onto one of this package's models."""
    rows = [(n, line.split()) for n, line in _lines(path) if line.strip()]
    models = {row[1] for _, row in rows if len(row) > 1}
    if models <= set(_RADIAL_FAMILY):
        name, family = "radial", _RADIAL_FAMILY
    elif models == {"PINHOLE"}:
        name, family = "pinhole", {"PINHOLE": [0, 1, 2, 3]}
    elif models <= set(_RADTAN_FAMILY):
        name, family = "radtan", _RADTAN_FAMILY
    else:
        raise InvalidInputError(f"COLMAP cameras: unsupported model mix {sorted(models)}")
    size = camera.MODELS[name][1]
    intrinsics = np.zeros((len(rows), size))
    refinable = np.zeros((len(rows), size), dtype=bool)
    camera_ids = np.zeros(len(rows), dtype=np.int64)
    for k, (number, row) in enumerate(rows):
        slots = family[row[1]]

        def parse(row: list[str] = row, slots: list[int] = slots) -> list[float]:
            if len(row) != 4 + len(slots):
                raise ValueError(f"{row[1]} takes {len(slots)} parameters")
            return [float(v) for v in row[4:]]

        intrinsics[k, slots] = _parse(path, number, parse)
        refinable[k, slots] = True
        camera_ids[k] = _parse(path, number, partial(int, row[0]))
    intrinsics[:, list(camera.CENTRE_INDICES[name])] -= COLMAP_PIXEL_OFFSET
    return name, intrinsics, refinable, camera_ids


@dataclass(frozen=True)
class _Image:
    """One image of ``images.txt``: its pose, camera, name and keypoints."""

    line: int
    """The header's line number in ``images.txt``."""
    image_id: int
    t_cam_world: Array
    camera_id: int
    name: str
    keypoints: Array
    """(K, 3): x, y and the observed point's id (-1 for none)."""


def _images(path: Path) -> list[_Image]:
    """Parse ``images.txt``: header lines, each followed by its (possibly blank) keypoints."""
    lines = _lines(path)
    if lines and not lines[-1][1].strip() and len(lines) % 2:
        lines = lines[:-1]  # a trailing newline after the last keypoint line
    if len(lines) % 2:
        raise InvalidInputError(f"{path.name}: expected a keypoint line after every image")
    images = []
    for (number, header), (key_number, keypoints) in zip(lines[0::2], lines[1::2], strict=True):

        def parse_header(header: str = header) -> tuple[int, Array, int, str]:
            fields = header.split(maxsplit=9)  # the name may contain spaces
            qw, qx, qy, qz, tx, ty, tz = (float(v) for v in fields[1:8])
            pose = np.eye(4)
            pose[:3, :3] = so3.from_quaternion([qx, qy, qz, qw])
            pose[:3, 3] = [tx, ty, tz]
            return int(fields[0]), pose, int(fields[8]), fields[9]

        def parse_keypoints(keypoints: str = keypoints) -> Array:
            values = np.asarray(keypoints.split(), dtype=np.float64)
            if values.size % 3:
                raise ValueError("keypoints come in (x, y, point id) triples")
            return values.reshape(-1, 3)

        image_id, pose, camera_id, name = _parse(path, number, parse_header)
        images.append(
            _Image(
                number, image_id, pose, camera_id, name, _parse(path, key_number, parse_keypoints)
            )
        )
    return images


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
        If a camera model is unsupported or the models mix families, or a line is malformed
        or refers to a camera or point that does not exist (the error names the line).
    """
    root = Path(directory)
    model, intrinsics, refinable, camera_ids = _cameras(root / "cameras.txt")
    group_of = {int(cid): k for k, cid in enumerate(camera_ids)}
    images = _images(root / "images.txt")
    points_path = root / "points3D.txt"
    point_rows = [(n, line.split()) for n, line in _lines(points_path) if line.strip()]
    point_ids = np.array(
        [_parse(points_path, n, partial(int, row[0])) for n, row in point_rows],
        dtype=np.int64,
    )
    xyz = np.array(
        [_parse(points_path, n, partial(_floats, row[1:4])) for n, row in point_rows]
    ).reshape(-1, 3)
    point_index = {int(pid): k for k, pid in enumerate(point_ids)}
    images_path = root / "images.txt"
    cam_idx, pt_idx, pixels = [], [], []
    for c, image in enumerate(images):
        seen = image.keypoints[:, 2] >= 0
        cam_idx += [c] * int(np.count_nonzero(seen))
        pt_idx += [
            _parse(images_path, image.line, partial(point_index.__getitem__, int(pid)))
            for pid in image.keypoints[seen, 2]
        ]
        pixels.append(image.keypoints[seen, :2] - COLMAP_PIXEL_OFFSET)
    t_cam_world = np.array([image.t_cam_world for image in images]).reshape(-1, 4, 4)
    problem = BAProblem.create(
        model,
        intrinsics,
        se3.inverse(t_cam_world),
        xyz,
        np.concatenate(pixels) if pixels else np.zeros((0, 2)),
        np.asarray(cam_idx, dtype=np.int64),
        np.asarray(pt_idx, dtype=np.int64),
        intrinsics_idx=np.array(
            [
                _parse(images_path, i.line, partial(group_of.__getitem__, i.camera_id))
                for i in images
            ],
            dtype=np.int64,
        ),
    )
    return ColmapModel(
        problem=problem,
        refinable=refinable,
        camera_ids=camera_ids,
        image_ids=np.array([image.image_id for image in images], dtype=np.int64),
        point_ids=point_ids,
        image_names=[image.name for image in images],
    )
