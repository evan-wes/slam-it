"""Rectified stereo: Bouguet's rectification, and ORB-SLAM's matching along rows.

**Rectification** turns both cameras about their centres so that their image rows become
epipolar lines: a point at row ``v`` in the left image lies on row ``v`` in the right one,
and its depth follows from the column difference alone. It follows Bouguet's algorithm, as
OpenCV's ``stereoRectify`` does:

1. Split the relative rotation in half: each camera turns half the way toward the other, so
   both share an orientation and the distortion each image suffers stays small.
2. Turn both about the axis that brings their baseline onto the x axis.
3. Give both one pinhole camera: focal length the smallest of the four input focal lengths
   (so the rectified views keep the narrower field), principal point at the image centre
   ``((cols - 1) / 2, (rows - 1) / 2)``. (OpenCV averages the focal lengths and moves the
   principal point to keep the valid region; the centre is simpler and enough here.)

With ``T_right_left`` mapping left-camera to right-camera coordinates
(``p_right = R p_left + t``), the rectified right camera sits at ``(baseline, 0, 0)`` in the
rectified left frame, so a point at depth ``Z`` appears ``f baseline / Z`` columns further
left in the right image. A pair whose right camera lies to the left of the left one is
refused (its disparities would be negative, and matching would find only false matches):
swap the cameras.

:func:`rectification_maps` gives, for each rectified pixel, where to sample the original
image (through each camera's own model, distortion included), for
:func:`slamlab.foundations.image.remap`. The maps are float64 and agree across the three
languages to rounding (the rotations come from SO(3)'s exponential and logarithm); the
remap's rounding to a 32nd of a pixel makes the rectified images agree exactly but for a map
value within rounding of a 64th of a pixel, which is vanishingly rare.

**Matching** (:func:`match`) pairs ORB features across a rectified pair as ORB-SLAM's
``ComputeStereoMatches`` does (Mur-Artal and Tardos, T-RO 2017):

1. A left keypoint's candidates are the right keypoints within one pyramid level of it,
   within ``band`` times the right keypoint's level scale of its row, and at most
   ``f`` columns to its left (a point no nearer than the baseline). The nearest by Hamming
   distance is kept if it is under ``max_distance`` (ORB-SLAM's 75, midway between its
   matcher's thresholds).
2. The column is refined on the left keypoint's pyramid level. The right keypoint's column
   is carried to that level (the pixel nearest its centre, in integers); a ``2 window + 1``
   square patch around the left keypoint slides along the right row over ``2 search + 1``
   integer offsets around it, scored by zero-mean SAD ``sum |n (L - R) - (sum L - sum R)|``
   (``n`` the patch's pixel count: the zero-mean sum of absolute differences times ``n``, in
   integers). The first smallest sum wins and must not be an end; the V (equiangular) fit
   through it and its neighbours gives the sub-pixel offset
   ``(d1 - d3) / (2 max(d1 - d2, d3 - d2))`` (Shimizu and Okutomi, IJCV 2005), within half a
   pixel since the first smallest sum makes ``d1 > d2 <= d3``. The refined column is mapped
   to the full image as keypoints are
   (:func:`slamlab.foundations.image.full_image_coordinate`).
3. The disparity ``x_left - x_right`` must lie in ``(0, f)``; the depth is
   ``f baseline / disparity``.
4. Matches whose best sum exceeds ``outlier_factor`` times the median (ORB-SLAM's 2.1) are
   dropped.

The features carry the pyramid they were extracted on (:class:`slamlab.foundations.orb.Features`),
and every keypoint is checked against it: inside its level, with ``xy`` exactly its level
pixel's full-image coordinate.

Departures from ORB-SLAM, each measured or a fix:

- **Zero-mean SAD and the V fit.** ORB-SLAM2 subtracts each patch's centre pixel and fits a
  parabola; ORB-SLAM3 drops the subtraction. The centre is a FAST corner, the steepest point
  of its patch, so subtracting it shifts the whole patch by an amount that changes with the
  sub-pixel shift: on area-sampled pairs ORB-SLAM2's sum over-estimates disparity by a median
  0.07 to 0.27 pixels, the parabola alone by 0.07, and zero-mean SAD with the V fit by under
  0.002, while staying invariant to a brightness offset between the cameras.
- **Exact coordinates.** Keypoints and refined columns map between levels and the full image
  exactly, not by ``s^l`` (see :func:`slamlab.foundations.image.full_image_coordinate`).
- **A non-positive disparity is dropped.** ORB-SLAM clamps it to 0.01, a point near infinity.
- **The outlier filter keeps matches equal to the limit, and every match when the median sum
  is 0.** ORB-SLAM's strict test drops them all then.
- **The search's bounds are checked exactly.** ORB-SLAM's ``iniu = u0 + L - w`` should read
  ``u0 - L - w``.

The sums are integers, and every float is computed by the same sequence of IEEE operations
in each language (none fused into a multiply-add), so the three languages find the same
matches with the same depths.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import camera, orb, se3, so3
from slamlab.foundations.arrays import (
    MAX_IMAGE_SIDE,
    as_batch,
    as_count,
    as_image,
    as_number,
    as_real,
    holds_integers,
)
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from slamlab.foundations.image import full_image_coordinate, level_shapes, pyramid
from slamlab.foundations.matching import MAX_DISTANCE, distances

MAX_SEARCH = 64
"""The largest half-range :attr:`Options.search` accepts, in level pixels."""

MIN_BASELINE = 1e-9
"""Camera centres closer than this (in the translation's units) count as coinciding."""

OUTSIDE = -2.0
"""Where a rectification map sends a pixel whose ray points behind the original camera: a
coordinate outside every image, which :func:`slamlab.foundations.image.remap` reads as 0."""


@dataclass(frozen=True)
class Rectification:
    """A rectified stereo pair.

    Attributes
    ----------
    rotation_left : ndarray of float64, shape (3, 3)
        ``R_rect_left``: turns left-camera coordinates into the rectified frame.
    rotation_right : ndarray of float64, shape (3, 3)
        ``R_rect_right``: turns right-camera coordinates into the rectified right frame.
    intrinsics : ndarray of float64, shape (4,)
        Both rectified cameras' pinhole intrinsics ``[f, f, cx, cy]``.
    baseline : float
        The distance between the camera centres, in the translation's units.
    shape : tuple of int
        The images' ``(rows, cols)``, original and rectified alike.
    left_model, right_model : str
        The original cameras' models.
    left_intrinsics, right_intrinsics : ndarray of float64
        The original cameras' intrinsics.
    """

    rotation_left: NDArray[np.float64]
    rotation_right: NDArray[np.float64]
    intrinsics: NDArray[np.float64]
    baseline: float
    shape: tuple[int, int]
    left_model: str
    left_intrinsics: NDArray[np.float64]
    right_model: str
    right_intrinsics: NDArray[np.float64]


@dataclass(frozen=True)
class Options:
    """Stereo-matching settings (ORB-SLAM's).

    Attributes
    ----------
    max_distance : int
        A match's Hamming distance must be below this, from 1 to
        :data:`slamlab.foundations.matching.MAX_DISTANCE` + 1.
    band : float
        The row band, in multiples of the right keypoint's level scale, positive.
    window : int
        The SAD patch's half-width, from 1 to :data:`slamlab.foundations.orb.EDGE_THRESHOLD`.
    search : int
        The SAD search's half-range, from 2 (a parabola needs both neighbours) to
        :data:`MAX_SEARCH`.
    outlier_factor : float
        Matches whose best sum exceeds this times the median are dropped; at least 1.
    """

    max_distance: int = 75
    band: float = 2.0
    window: int = 5
    search: int = 5
    outlier_factor: float = 2.1

    def __post_init__(self) -> None:
        """Check the settings' types and ranges.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting has the wrong type or is out of range.
        """
        set_ = object.__setattr__  # frozen: store the checked, normalised values
        set_(self, "max_distance", as_count(self.max_distance, "max_distance", 1, MAX_DISTANCE + 1))
        band = as_number(self.band, "band")
        if not (math.isfinite(band) and band > 0.0):
            raise InvalidInputError(f"band: must be positive and finite, got {band}")
        set_(self, "band", band)
        set_(self, "window", as_count(self.window, "window", 1, orb.EDGE_THRESHOLD))
        set_(self, "search", as_count(self.search, "search", 2, MAX_SEARCH))
        factor = as_number(self.outlier_factor, "outlier_factor")
        if not (math.isfinite(factor) and factor >= 1.0):
            raise InvalidInputError(f"outlier_factor: must be at least 1, got {factor}")
        set_(self, "outlier_factor", factor)


@dataclass(frozen=True)
class StereoMatches:
    """Left keypoints matched along their rows, ordered by left index.

    Attributes
    ----------
    left : ndarray of int64, shape (K,)
        The left keypoints' indices.
    right : ndarray of int64, shape (K,)
        Their right keypoints' indices.
    right_x : ndarray of float64, shape (K,)
        The refined right column, in full-image pixels.
    disparity : ndarray of float64, shape (K,)
        ``x_left - right_x``, positive.
    depth : ndarray of float64, shape (K,)
        ``focal baseline / disparity``, in the baseline's units.
    """

    left: NDArray[np.int64]
    right: NDArray[np.int64]
    right_x: NDArray[np.float64]
    disparity: NDArray[np.float64]
    depth: NDArray[np.float64]


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.stereo`` submodule provides.

    Camera models cross as their :data:`slamlab.foundations.camera.MODELS` codes.
    """

    def rectify(  # noqa: PLR0913 -- two cameras, their pose, the image shape
        self,
        *,
        model_left: int,
        intrinsics_left: NDArray[np.float64],
        model_right: int,
        intrinsics_right: NDArray[np.float64],
        T_right_left: NDArray[np.float64],  # noqa: N803 -- the frame convention's name
        rows: int,
        cols: int,
    ) -> dict[str, Any]:
        """Rectify a pair; return the rotations, the rectified intrinsics and the baseline."""
        ...

    def rectification_map(  # noqa: PLR0913 -- one camera, its rectification, the shape
        self,
        *,
        model: int,
        intrinsics: NDArray[np.float64],
        rotation: NDArray[np.float64],
        rectified: NDArray[np.float64],
        rows: int,
        cols: int,
    ) -> NDArray[np.float64]:
        """Return the (rows, cols, 2) map from rectified pixels to the original image."""
        ...

    def match(  # noqa: PLR0913 -- two images, their features, the camera, the settings
        self,
        *,
        left_image: NDArray[np.uint8],
        right_image: NDArray[np.uint8],
        left: dict[str, Any],
        right: dict[str, Any],
        focal: float,
        baseline: float,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Match features along rows; return a dict of :class:`StereoMatches`' fields."""
        ...


def _rectify(
    intrinsics: tuple[NDArray[np.float64], NDArray[np.float64]],
    focal_indices: tuple[tuple[int, ...], tuple[int, ...]],
    transform: NDArray[np.float64],
    shape: tuple[int, int],
) -> dict[str, Any]:
    """Bouguet's rectification (reference)."""
    rotation, translation = transform[:3, :3], transform[:3, 3]
    baseline = math.sqrt(float(translation @ translation))
    if baseline <= MIN_BASELINE:
        raise DegenerateError("T_right_left: the camera centres coincide")
    half = so3.exp(-0.5 * so3.log(rotation))  # each camera turns half the way
    t = half @ translation
    if abs(t[0]) <= abs(t[1]):
        raise InvalidInputError(
            "T_right_left: the baseline is closer to vertical; only side-by-side pairs rectify"
        )
    if t[0] > 0.0:
        raise InvalidInputError(
            "T_right_left: the right camera lies left of the left one; swap the cameras"
        )
    target = np.array([-1.0, 0.0, 0.0])  # the left camera's centre, seen from the right
    axis = np.cross(t, target)
    norm = math.sqrt(float(axis @ axis))
    norm_t = math.sqrt(float(t @ t))
    if norm > 0.0:
        axis = axis * (math.acos(min(abs(float(t[0])) / norm_t, 1.0)) / norm)
    onto_x = so3.exp(axis)
    focal = min(
        float(params[i])
        for params, indices in zip(intrinsics, focal_indices, strict=True)
        for i in indices
    )
    rows, cols = shape
    return {
        "rotation_left": onto_x @ half.T,
        "rotation_right": onto_x @ half,
        "intrinsics": np.array([focal, focal, (cols - 1) / 2.0, (rows - 1) / 2.0]),
        "baseline": baseline,
    }


def _rectification_map(
    model: str,
    intrinsics: NDArray[np.float64],
    rotation: NDArray[np.float64],
    rectified: NDArray[np.float64],
    shape: tuple[int, int],
) -> NDArray[np.float64]:
    """Each rectified pixel's source in the original image (reference)."""
    rows, cols = shape
    f, _, cx, cy = rectified
    v, u = np.mgrid[0:rows, 0:cols].astype(np.float64)
    x, y = (u - cx) / f, (v - cy) / f
    r = rotation
    # The ray in the original camera, R^T (x, y, 1), one product and sum at a time (no BLAS,
    # whose fused multiply-adds would round differently from the compiled cores).
    ray = np.stack(
        [
            r[0, 0] * x + r[1, 0] * y + r[2, 0],
            r[0, 1] * x + r[1, 1] * y + r[2, 1],
            r[0, 2] * x + r[1, 2] * y + r[2, 2],
        ],
        axis=-1,
    )
    out = np.full((rows, cols, 2), OUTSIDE)
    front = ray[..., 2] > camera.MIN_DEPTH
    if np.any(front):
        out[front] = camera.project(model, intrinsics, ray[front])
    return out


def _as_shape(shape: tuple[int, int]) -> tuple[int, int]:
    """Check an image shape: two integers from 1 to MAX_IMAGE_SIDE."""
    values = tuple(shape)
    if len(values) != 2 or any(
        isinstance(n, (bool, np.bool_))
        or not isinstance(n, (int, np.integer))
        or not 1 <= n <= MAX_IMAGE_SIDE
        for n in values
    ):
        raise InvalidInputError(
            f"shape: expected (rows, cols), each from 1 to {MAX_IMAGE_SIDE}, got {shape!r}"
        )
    return int(values[0]), int(values[1])


def _decode_rectification(raw: dict[str, Any], codes: backends.Codes) -> dict[str, Any]:
    """Return a compiled backend's rectification, each array checked (never cast)."""
    del codes  # the stereo module crosses no codes
    check = backends.checked_array
    if not isinstance(raw["baseline"], float):
        raise RuntimeError(f"stereo baseline: backend returned {type(raw['baseline']).__name__}")
    return {
        "rotation_left": check(raw["rotation_left"], np.float64, 2, "stereo rotation_left"),
        "rotation_right": check(raw["rotation_right"], np.float64, 2, "stereo rotation_right"),
        "intrinsics": check(raw["intrinsics"], np.float64, 1, "stereo intrinsics"),
        "baseline": raw["baseline"],
    }


def _decode_map(raw: Any, codes: backends.Codes) -> NDArray[np.float64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's map, checked to be a float64 (rows, cols, 2) array."""
    del codes
    out: NDArray[np.float64] = backends.checked_array(raw, np.float64, 3, "stereo map")
    if out.shape[2] != 2:
        raise RuntimeError(f"stereo map: backend returned shape {out.shape}")
    return out


def rectify(  # noqa: PLR0913 -- two cameras, their pose, the image shape
    left_model: str,
    left_intrinsics: ArrayLike,
    right_model: str,
    right_intrinsics: ArrayLike,
    T_right_left: ArrayLike,  # noqa: N803 -- the frame convention's name
    *,
    shape: tuple[int, int],
    backend: str = "python",
) -> Rectification:
    """Rectify a calibrated stereo pair (Bouguet's algorithm).

    Parameters
    ----------
    left_model, right_model : {"pinhole", "radtan", "radial"}
        The cameras' models.
    left_intrinsics, right_intrinsics : array_like
        Their intrinsics (:mod:`slamlab.foundations.camera` gives each model's layout).
    T_right_left : array_like, shape (4, 4)
        The rigid transform from left-camera to right-camera coordinates.
    shape : tuple of int
        The images' ``(rows, cols)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Rectification
        The rectifying rotations, the shared pinhole camera and the baseline.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a model or its intrinsics are invalid, `T_right_left` is not one rigid transform,
        the shape is out of range, or the baseline is closer to vertical than horizontal.
    slamlab.foundations.errors.DegenerateError
        If the camera centres coincide.
    """
    left_code, left_params = camera.as_model(left_model, left_intrinsics)
    right_code, right_params = camera.as_model(right_model, right_intrinsics)
    flat, lead = as_batch(T_right_left, (4, 4), "T_right_left")
    if lead != ():
        raise InvalidInputError(f"T_right_left: expected one (4, 4) transform, got {lead}")
    se3.check_transform(flat)
    transform = flat[0]
    rows, cols = _as_shape(shape)
    result = backends.call(
        backend,
        __name__,
        "rectify",
        reference=lambda: _rectify(
            (left_params, right_params),
            (camera.FOCAL_INDICES[left_model], camera.FOCAL_INDICES[right_model]),
            transform,
            (rows, cols),
        ),
        arguments={
            "model_left": left_code,
            "intrinsics_left": left_params,
            "model_right": right_code,
            "intrinsics_right": right_params,
            "T_right_left": transform,
            "rows": rows,
            "cols": cols,
        },
        decode=_decode_rectification,
    )
    return Rectification(
        **result,
        shape=(rows, cols),
        left_model=left_model,
        left_intrinsics=left_params,
        right_model=right_model,
        right_intrinsics=right_params,
    )


def _map(  # noqa: PLR0913 -- one camera, its rotation, the rectified camera and shape
    model: str,
    intrinsics: ArrayLike,
    rotation: ArrayLike,
    rectified: NDArray[np.float64],
    *,
    shape: tuple[int, int],
    backend: str,
) -> NDArray[np.float64]:
    """Check one side of a rectification and compute its map on the chosen backend."""
    code, params = camera.as_model(model, intrinsics)
    turn = as_real(rotation, "rotation")
    if turn.shape != (3, 3):
        raise InvalidInputError(f"rotation: expected (3, 3), got {turn.shape}")
    so3.check_rotation(turn.reshape(1, 3, 3))
    return backends.call(
        backend,
        __name__,
        "rectification_map",
        reference=lambda: _rectification_map(model, params, turn, rectified, shape),
        arguments={
            "model": code,
            "intrinsics": params,
            "rotation": turn,
            "rectified": rectified,
            "rows": shape[0],
            "cols": shape[1],
        },
        decode=_decode_map,
    )


def rectification_maps(
    rectification: Rectification, *, backend: str = "python"
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return where each rectified pixel samples the original left and right images.

    Parameters
    ----------
    rectification : Rectification
        The pair, from :func:`rectify`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    left, right : ndarray of float64, shape (rows, cols, 2)
        For each rectified pixel, the ``(x, y)`` to sample in that original image with
        :func:`slamlab.foundations.image.remap`; :data:`OUTSIDE` where the pixel's ray points
        behind the camera.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the rectification's rotations or intrinsics are malformed.
    """
    rows, cols = _as_shape(rectification.shape)
    rectified = as_real(rectification.intrinsics, "intrinsics")
    camera.check_intrinsics("pinhole", rectified)
    left = _map(
        rectification.left_model, rectification.left_intrinsics, rectification.rotation_left,
        rectified, shape=(rows, cols), backend=backend,
    )  # fmt: skip
    right = _map(
        rectification.right_model, rectification.right_intrinsics, rectification.rotation_right,
        rectified, shape=(rows, cols), backend=backend,
    )  # fmt: skip
    return left, right


@dataclass(frozen=True)
class _Side:
    """One camera's features, as the matcher reads them."""

    xy: NDArray[np.float64]
    level: NDArray[np.int64]
    level_xy: NDArray[np.int64]
    descriptors: NDArray[np.uint8]
    levels: int
    scale_factor: float


def _refine(
    left_level: NDArray[np.int64],
    right_level: NDArray[np.int64],
    left_pixel: tuple[int, int],
    right_column: int,
    options: Options,
) -> tuple[float, int] | None:
    """Slide the left patch along the right row; return the sub-pixel offset and its sum.

    The offset is from `right_column`, in level pixels; ``None`` if the search would leave the
    level or its best offset is an end.
    """
    w, reach = options.window, options.search
    x, y = left_pixel
    if right_column - reach - w < 0 or right_column + reach + w >= right_level.shape[1]:
        return None
    patch = left_level[y - w : y + w + 1, x - w : x + w + 1]
    count, total = patch.size, int(patch.sum())
    sums = []
    for step in range(-reach, reach + 1):
        c = right_column + step
        other = right_level[y - w : y + w + 1, c - w : c + w + 1]
        sums.append(int(np.abs(count * (patch - other) - (total - int(other.sum()))).sum()))
    best = int(np.argmin(sums))  # the first smallest: d1 > d2 <= d3 below
    if best in (0, 2 * reach):
        return None
    d1, d2, d3 = sums[best - 1 : best + 2]
    delta = (d1 - d3) / (2 * max(d1 - d2, d3 - d2))  # the V fit: |delta| <= 1/2
    return (best - reach) + delta, d2


def _nearest_candidate(  # noqa: PLR0913 -- the keypoint, both sides, the camera, the settings
    i: int,
    left: _Side,
    right: _Side,
    hamming: NDArray[np.int64],
    *,
    scale: NDArray[np.float64],
    focal: float,
    options: Options,
) -> int | None:
    """Return left keypoint `i`'s nearest right candidate (ties: the lowest index), or None."""
    x_left, y_left = float(left.xy[i, 0]), float(left.xy[i, 1])
    candidates = (
        (np.abs(right.level - left.level[i]) <= 1)
        & (np.abs(right.xy[:, 1] - y_left) <= options.band * scale[right.level])
        & (right.xy[:, 0] >= x_left - focal)
        & (right.xy[:, 0] <= x_left)
        & (hamming < options.max_distance)
    )
    if not np.any(candidates):
        return None
    return int(np.flatnonzero(candidates)[np.argmin(hamming[candidates])])


def _nearest_column(x: int, from_cols: int, to_cols: int) -> int:
    """Return the column of a level `to_cols` wide nearest pixel `x`'s centre on another."""
    return ((2 * x + 1) * to_cols) // (2 * from_cols)


def _drop_outliers(
    found: list[tuple[int, int, float, float, float, int]], factor: float
) -> StereoMatches:
    """Drop matches whose best sum exceeds `factor` times the median; return the rest."""
    if found:
        costs = sorted(f[5] for f in found)
        limit = factor * costs[len(costs) // 2]
        found = [f for f in found if f[5] <= limit]
    return StereoMatches(
        left=np.array([f[0] for f in found], dtype=np.int64),
        right=np.array([f[1] for f in found], dtype=np.int64),
        right_x=np.array([f[2] for f in found], dtype=np.float64),
        disparity=np.array([f[3] for f in found], dtype=np.float64),
        depth=np.array([f[4] for f in found], dtype=np.float64),
    )


def _match(  # noqa: PLR0913 -- two images, their features, the camera, the settings
    images: tuple[NDArray[np.uint8], NDArray[np.uint8]],
    left: _Side,
    right: _Side,
    *,
    focal: float,
    baseline: float,
    options: Options,
) -> StereoMatches:
    """Match features along rows (reference)."""
    levels, factor = left.levels, left.scale_factor
    rows, cols = images[0].shape
    shapes = level_shapes(rows, cols, levels, factor)
    pyramids = [[p.astype(np.int64) for p in pyramid(im, levels, factor)] for im in images]
    scales = [1.0]
    for _ in range(1, levels):
        scales.append(scales[-1] * factor)
    scale = np.asarray(scales)
    hamming = distances(left.descriptors, right.descriptors)
    found: list[tuple[int, int, float, float, float, int]] = []
    for i in range(len(left.level)):
        j = _nearest_candidate(
            i, left, right, hamming[i], scale=scale, focal=focal, options=options
        )
        if j is None:
            continue
        lvl = int(left.level[i])
        level_cols = shapes[lvl][1]
        right_cols = shapes[int(right.level[j])][1]
        column = _nearest_column(int(right.level_xy[j, 0]), right_cols, level_cols)
        pixel = (int(left.level_xy[i, 0]), int(left.level_xy[i, 1]))
        refined = _refine(pyramids[0][lvl], pyramids[1][lvl], pixel, column, options)
        if refined is None:
            continue
        offset, cost = refined
        right_x = float(full_image_coordinate(column + offset, level_cols, cols))
        disparity = float(left.xy[i, 0]) - right_x
        if not 0.0 < disparity < focal:
            continue
        found.append((i, j, right_x, disparity, focal * baseline / disparity, cost))
    return _drop_outliers(found, options.outlier_factor)


def _as_side(features: orb.Features, name: str) -> _Side:
    """Check one camera's features: one keypoint per descriptor, and a valid pyramid."""
    keypoints = features.keypoints
    xy = as_real(keypoints.xy, f"{name}.xy")
    count = len(xy)
    level = np.asarray(keypoints.level)
    level_xy = np.asarray(keypoints.level_xy)
    descriptors = np.asarray(features.descriptors)
    if (
        xy.shape != (count, 2)
        or level.shape != (count,)
        or level_xy.shape != (count, 2)
        or not holds_integers(level)
        or not holds_integers(level_xy)
        or descriptors.shape != (count, orb.DESCRIPTOR_BYTES)
        or descriptors.dtype != np.uint8
    ):
        raise InvalidInputError(f"{name}: expected one keypoint and descriptor per row")
    levels = as_count(features.levels, f"{name}.levels", 1, orb.MAX_LEVELS)
    factor = as_number(features.scale_factor, f"{name}.scale_factor")
    if not (math.isfinite(factor) and factor > 1.0):
        raise InvalidInputError(f"{name}.scale_factor: must be finite and above 1, got {factor}")
    return _Side(
        xy=xy,
        level=level.astype(np.int64),
        level_xy=level_xy.astype(np.int64),
        descriptors=np.ascontiguousarray(descriptors),
        levels=levels,
        scale_factor=factor,
    )


def _check_positions(side: _Side, shapes: list[tuple[int, int]], name: str) -> None:
    """Check every keypoint against its pyramid: inside its level, ``xy`` its exact map."""
    if len(side.level) == 0:
        return
    if side.level.min() < 0 or side.level.max() >= len(shapes):
        raise InvalidInputError(f"{name}: levels must lie in [0, {len(shapes)})")
    sizes = np.asarray(shapes, dtype=np.int64)[side.level]  # each keypoint's (rows, cols)
    x, y = side.level_xy[:, 0], side.level_xy[:, 1]
    if np.any(x < 0) or np.any(y < 0) or np.any(x >= sizes[:, 1]) or np.any(y >= sizes[:, 0]):
        raise InvalidInputError(f"{name}: level_xy must lie inside its level")
    rows, cols = shapes[0]
    exact = np.column_stack(
        [full_image_coordinate(x, sizes[:, 1], cols), full_image_coordinate(y, sizes[:, 0], rows)]
    )
    if not np.array_equal(side.xy, exact):
        raise InvalidInputError(
            f"{name}: xy must be level_xy mapped to the full image (orb.Keypoints.xy)"
        )


def _check_inside(side: _Side, shapes: list[tuple[int, int]], margin: int, name: str) -> None:
    """Check every keypoint's level pixel lies at least `margin` inside its level."""
    for lvl, (rows, cols) in enumerate(shapes):
        pixels = side.level_xy[side.level == lvl]
        if len(pixels) and not (
            np.all(pixels[:, 0] >= margin)
            and np.all(pixels[:, 0] < cols - margin)
            and np.all(pixels[:, 1] >= margin)
            and np.all(pixels[:, 1] < rows - margin)
        ):
            raise InvalidInputError(
                f"{name}: keypoints must lie at least {margin} pixels inside their level"
            )


def _decode_matches(raw: dict[str, Any], codes: backends.Codes) -> StereoMatches:
    """Rebuild :class:`StereoMatches` from a compiled backend's dict, each array checked."""
    del codes
    check = backends.checked_array
    return StereoMatches(
        left=check(raw["left"], np.int64, 1, "stereo left"),
        right=check(raw["right"], np.int64, 1, "stereo right"),
        right_x=check(raw["right_x"], np.float64, 1, "stereo right_x"),
        disparity=check(raw["disparity"], np.float64, 1, "stereo disparity"),
        depth=check(raw["depth"], np.float64, 1, "stereo depth"),
    )


def match(  # noqa: PLR0913 -- two images, their features, the camera, the settings
    left_image: ArrayLike,
    right_image: ArrayLike,
    left: orb.Features,
    right: orb.Features,
    *,
    focal: float,
    baseline: float,
    options: Options | None = None,
    backend: str = "python",
) -> StereoMatches:
    """Match ORB features along the rows of a rectified pair (ORB-SLAM's stereo matching).

    Parameters
    ----------
    left_image, right_image : array_like of uint8, shape (H, W)
        The rectified images.
    left, right : slamlab.foundations.orb.Features
        Their features, extracted on one pyramid (the same ``levels`` and ``scale_factor``).
    focal : float
        The rectified focal length, in pixels (:attr:`Rectification.intrinsics`' first).
    baseline : float
        The distance between the camera centres; depths come out in its units.
    options : Options, optional
        The matching settings (default: ORB-SLAM's).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    StereoMatches
        The matched left keypoints, with their right column, disparity and depth.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If an image is malformed or the two differ in shape, `focal` or `baseline` is not
        positive and finite, a feature set is malformed, the two sets come from different
        pyramids or one too deep for the image, a keypoint lies outside its level or its
        ``xy`` is not its level pixel's full-image coordinate, or a left keypoint lies
        closer than ``options.window`` to its level's border.
    """
    first = as_image(left_image, "left_image")
    second = as_image(right_image, "right_image")
    if first.shape != second.shape:
        raise InvalidInputError(
            f"right_image: expected the left image's shape {first.shape}, got {second.shape}"
        )
    focal = as_number(focal, "focal")
    baseline = as_number(baseline, "baseline")
    for name, value in (("focal", focal), ("baseline", baseline)):
        if not (math.isfinite(value) and value > 0.0):
            raise InvalidInputError(f"{name}: must be positive and finite, got {value}")
    settings = Options() if options is None else options
    sides = (_as_side(left, "left"), _as_side(right, "right"))
    if (sides[1].levels, sides[1].scale_factor) != (sides[0].levels, sides[0].scale_factor):
        raise InvalidInputError(
            "right: extracted on another pyramid than left (levels and scale_factor differ)"
        )
    shapes = level_shapes(first.shape[0], first.shape[1], sides[0].levels, sides[0].scale_factor)
    for side, name in zip(sides, ("left", "right"), strict=True):
        _check_positions(side, shapes, name)
    _check_inside(sides[0], shapes, settings.window, "left")
    crossing = [
        {
            "xy": s.xy,
            "level": s.level,
            "level_xy": s.level_xy,
            "descriptors": s.descriptors,
            "scale_factor": s.scale_factor,
            "levels": s.levels,
        }
        for s in sides
    ]
    return backends.call(
        backend,
        __name__,
        "match",
        reference=lambda: _match(
            (first, second),
            sides[0],
            sides[1],
            focal=focal,
            baseline=baseline,
            options=settings,
        ),
        arguments={
            "left_image": first,
            "right_image": second,
            "left": crossing[0],
            "right": crossing[1],
            "focal": focal,
            "baseline": baseline,
            "options": asdict(settings),
        },
        decode=_decode_matches,
    )
