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
left in the right image.

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
2. The column is refined on the left keypoint's pyramid level: a ``2 window + 1`` square
   patch around the left keypoint, minus its centre value, slides along the right row over
   ``2 search + 1`` integer offsets around the right keypoint; the offset with the smallest
   sum of absolute differences wins (it must not be an end), and a parabola through it and
   its neighbours gives the sub-pixel offset ``(d1 - d3) / (2 (d1 + d3 - 2 d2))``, kept
   within one pixel.
3. The disparity ``x_left - x_right`` must lie in ``(0, f)``; the depth is
   ``f baseline / disparity``.
4. Matches whose best sum exceeds ``outlier_factor`` times the median (ORB-SLAM's 2.1) are
   dropped.

The sums are integers, and every float is one IEEE operation on exact inputs, so the three
languages find the same matches with the same depths.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import camera, orb, se3, so3
from slamlab.foundations.arrays import MAX_IMAGE_SIDE, as_batch, as_image, as_real
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from slamlab.foundations.image import level_shapes, pyramid
from slamlab.foundations.matching import distances

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
        A match's Hamming distance must be below this, from 1 to 257.
    band : float
        The row band, in multiples of the right keypoint's level scale, positive.
    window : int
        The SAD patch's half-width, from 1 to :data:`slamlab.foundations.orb.EDGE_THRESHOLD`.
    search : int
        The SAD search's half-range, from 2 to 64.
    outlier_factor : float
        Matches whose best sum exceeds this times the median are dropped; at least 1.
    """

    max_distance: int = 75
    band: float = 2.0
    window: int = 5
    search: int = 5
    outlier_factor: float = 2.1

    def __post_init__(self) -> None:
        """Check the settings.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting is out of range.
        """
        if not 1 <= self.max_distance <= 8 * orb.DESCRIPTOR_BYTES + 1:
            raise InvalidInputError(f"max_distance: must be from 1 to 257, got {self.max_distance}")
        if not (math.isfinite(self.band) and self.band > 0.0):
            raise InvalidInputError(f"band: must be positive and finite, got {self.band}")
        if not 1 <= self.window <= orb.EDGE_THRESHOLD:
            raise InvalidInputError(
                f"window: must be from 1 to {orb.EDGE_THRESHOLD}, got {self.window}"
            )
        if not 2 <= self.search <= 64:  # a parabola needs both neighbours
            raise InvalidInputError(f"search: must be from 2 to 64, got {self.search}")
        if not (math.isfinite(self.outlier_factor) and self.outlier_factor >= 1.0):
            raise InvalidInputError(
                f"outlier_factor: must be at least 1, got {self.outlier_factor}"
            )


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
        orb_options: dict[str, Any],
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
    target = np.array([math.copysign(1.0, t[0]), 0.0, 0.0])
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
        isinstance(n, bool) or not isinstance(n, (int, np.integer)) or not 1 <= n <= MAX_IMAGE_SIDE
        for n in values
    ):
        raise InvalidInputError(
            f"shape: expected (rows, cols), each from 1 to {MAX_IMAGE_SIDE}, got {shape!r}"
        )
    return int(values[0]), int(values[1])


def _decode_rectification(raw: dict[str, Any], codes: backends.Codes) -> dict[str, Any]:
    """Return a compiled backend's rectification as float64 arrays and a float."""
    del codes  # the stereo module crosses no codes
    return {
        "rotation_left": np.asarray(raw["rotation_left"], dtype=np.float64).reshape(3, 3),
        "rotation_right": np.asarray(raw["rotation_right"], dtype=np.float64).reshape(3, 3),
        "intrinsics": np.asarray(raw["intrinsics"], dtype=np.float64).reshape(4),
        "baseline": float(raw["baseline"]),
    }


def _decode_map(raw: Any, codes: backends.Codes) -> NDArray[np.float64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's map as a float64 (rows, cols, 2) array."""
    del codes
    out = np.asarray(raw, dtype=np.float64)
    if out.ndim != 3 or out.shape[2] != 2:
        raise RuntimeError(f"stereo: backend returned a map of shape {out.shape}")
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


def _refine(
    left_level: NDArray[np.int64],
    right_level: NDArray[np.int64],
    left_pixel: tuple[int, int],
    right_column: int,
    options: Options,
) -> tuple[float, int] | None:
    """Slide the left patch along the right row; return the sub-pixel offset and its sum."""
    w, reach = options.window, options.search
    x, y = left_pixel
    if right_column - reach - w < 0 or right_column + reach + w >= right_level.shape[1]:
        return None
    patch = left_level[y - w : y + w + 1, x - w : x + w + 1] - left_level[y, x]
    sums = []
    for step in range(-reach, reach + 1):
        c = right_column + step
        other = right_level[y - w : y + w + 1, c - w : c + w + 1] - right_level[y, c]
        sums.append(int(np.abs(patch - other).sum()))
    best = int(np.argmin(sums))  # ties: the leftmost offset
    if best in (0, 2 * reach):
        return None
    d1, d2, d3 = sums[best - 1], sums[best], sums[best + 1]
    curvature = 2 * (d1 + d3 - 2 * d2)
    if curvature == 0:  # a flat valley: no sub-pixel position
        return None
    delta = (d1 - d3) / curvature
    if not -1.0 <= delta <= 1.0:
        return None
    return (best - reach) + delta, d2


def _match(  # noqa: PLR0913 -- two images, their features, the camera, the settings
    images: tuple[NDArray[np.uint8], NDArray[np.uint8]],
    left: _Side,
    right: _Side,
    *,
    focal: float,
    baseline: float,
    orb_options: orb.Options,
    options: Options,
) -> StereoMatches:
    """Match features along rows (reference)."""
    left_levels = [
        p.astype(np.int64) for p in pyramid(images[0], orb_options.levels, orb_options.scale_factor)
    ]
    right_levels = [
        p.astype(np.int64) for p in pyramid(images[1], orb_options.levels, orb_options.scale_factor)
    ]
    scales = [1.0]
    for _ in range(1, orb_options.levels):
        scales.append(scales[-1] * orb_options.scale_factor)
    scale = np.asarray(scales)
    hamming = distances(left.descriptors, right.descriptors)
    found: list[tuple[int, int, float, float, float, int]] = []
    for i in range(len(left.level)):
        x_left, y_left = float(left.xy[i, 0]), float(left.xy[i, 1])
        lvl = int(left.level[i])
        candidates = (
            (np.abs(right.level - lvl) <= 1)
            & (np.abs(right.xy[:, 1] - y_left) <= options.band * scale[right.level])
            & (right.xy[:, 0] >= x_left - focal)
            & (right.xy[:, 0] <= x_left)
            & (hamming[i] < options.max_distance)
        )
        if not np.any(candidates):
            continue
        j = int(np.flatnonzero(candidates)[np.argmin(hamming[i][candidates])])
        right_column = math.floor(float(right.xy[j, 0]) / scales[lvl] + 0.5)
        x, y = int(left.level_xy[i, 0]), int(left.level_xy[i, 1])
        refined = _refine(left_levels[lvl], right_levels[lvl], (x, y), right_column, options)
        if refined is None:
            continue
        offset, cost = refined
        right_x = scales[lvl] * (right_column + offset)
        disparity = x_left - right_x
        if not 0.0 < disparity < focal:
            continue
        found.append((i, j, right_x, disparity, focal * baseline / disparity, cost))
    if found:
        costs = sorted(f[5] for f in found)
        limit = options.outlier_factor * costs[len(costs) // 2]
        found = [f for f in found if f[5] <= limit]
    return StereoMatches(
        left=np.array([f[0] for f in found], dtype=np.int64),
        right=np.array([f[1] for f in found], dtype=np.int64),
        right_x=np.array([f[2] for f in found], dtype=np.float64),
        disparity=np.array([f[3] for f in found], dtype=np.float64),
        depth=np.array([f[4] for f in found], dtype=np.float64),
    )


def _as_side(features: orb.Features, shapes: list[tuple[int, int]], name: str) -> _Side:
    """Check one camera's features against the pyramid they came from."""
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
        or not np.can_cast(level.dtype, np.int64)
        or not np.can_cast(level_xy.dtype, np.int64)
        or descriptors.shape != (count, orb.DESCRIPTOR_BYTES)
        or descriptors.dtype != np.uint8
    ):
        raise InvalidInputError(f"{name}: expected one keypoint and descriptor per row")
    if count and (level.min() < 0 or level.max() >= len(shapes)):
        raise InvalidInputError(f"{name}: levels must lie in [0, {len(shapes)})")
    return _Side(
        xy=xy,
        level=level.astype(np.int64),
        level_xy=level_xy.astype(np.int64),
        descriptors=np.ascontiguousarray(descriptors),
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
    """Rebuild :class:`StereoMatches` from a compiled backend's dict."""
    del codes
    return StereoMatches(
        left=np.asarray(raw["left"], dtype=np.int64),
        right=np.asarray(raw["right"], dtype=np.int64),
        right_x=np.asarray(raw["right_x"], dtype=np.float64),
        disparity=np.asarray(raw["disparity"], dtype=np.float64),
        depth=np.asarray(raw["depth"], dtype=np.float64),
    )


def match(  # noqa: PLR0913 -- two images, their features, the camera, the settings
    left_image: ArrayLike,
    right_image: ArrayLike,
    left: orb.Features,
    right: orb.Features,
    *,
    focal: float,
    baseline: float,
    orb_options: orb.Options | None = None,
    options: Options | None = None,
    backend: str = "python",
) -> StereoMatches:
    """Match ORB features along the rows of a rectified pair (ORB-SLAM's stereo matching).

    Parameters
    ----------
    left_image, right_image : array_like of uint8, shape (H, W)
        The rectified images.
    left, right : slamlab.foundations.orb.Features
        Their features, extracted with `orb_options`.
    focal : float
        The rectified focal length, in pixels (:attr:`Rectification.intrinsics`' first).
    baseline : float
        The distance between the camera centres; depths come out in its units.
    orb_options : slamlab.foundations.orb.Options, optional
        The settings the features were extracted with (for their pyramid).
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
        positive and finite, a feature set is malformed, or a left keypoint lies closer than
        ``options.window`` to its level's border.
    """
    first = as_image(left_image, "left_image")
    second = as_image(right_image, "right_image")
    if first.shape != second.shape:
        raise InvalidInputError(
            f"right_image: expected the left image's shape {first.shape}, got {second.shape}"
        )
    for name, value in (("focal", focal), ("baseline", baseline)):
        if not (math.isfinite(value) and value > 0.0):
            raise InvalidInputError(f"{name}: must be positive and finite, got {value}")
    extraction = orb.Options() if orb_options is None else orb_options
    settings = Options() if options is None else options
    shapes = level_shapes(
        first.shape[0], first.shape[1], extraction.levels, extraction.scale_factor
    )
    sides = (_as_side(left, shapes, "left"), _as_side(right, shapes, "right"))
    _check_inside(sides[0], shapes, settings.window, "left")
    crossing = [
        {"xy": s.xy, "level": s.level, "level_xy": s.level_xy, "descriptors": s.descriptors}
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
            focal=float(focal),
            baseline=float(baseline),
            orb_options=extraction,
            options=settings,
        ),
        arguments={
            "left_image": first,
            "right_image": second,
            "left": crossing[0],
            "right": crossing[1],
            "focal": float(focal),
            "baseline": float(baseline),
            "orb_options": asdict(extraction),
            "options": asdict(settings),
        },
        decode=_decode_matches,
    )
