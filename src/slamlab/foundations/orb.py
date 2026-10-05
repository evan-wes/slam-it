"""ORB keypoints as ORB-SLAM detects them: FAST corners spread by a quadtree, with orientations.

ORB (Rublee et al., "ORB: an efficient alternative to SIFT or SURF", ICCV 2011) is FAST
corners with an intensity-centroid orientation and a rotated binary descriptor. ORB-SLAM
(Mur-Artal et al., T-RO 2015 and 2017) extracts them so they cover the image evenly; this
module follows its extractor:

- **Pyramid:** ``levels`` levels at ``scale_factor`` (:func:`slamlab.foundations.image.
  pyramid`), each with a share of the feature budget that falls geometrically with the level
  (:func:`level_budgets`).
- **FAST-9/16 corners** (Rosten and Drummond, ECCV 2006): a pixel is a corner at threshold
  ``t`` if 9 contiguous pixels of the 16 on the radius-3 circle around it are all brighter
  than it by more than ``t``, or all darker by more than ``t``. Its score (:func:`fast_scores`)
  is the largest such ``t``, OpenCV's ``cornerScore``; corners are the strict maxima of the
  score over their 3x3 neighbourhood (non-maximum suppression).
- **Cells:** keypoints lie at least :data:`EDGE_THRESHOLD` pixels inside the level, so the
  orientation patch and the descriptor's pattern never leave it. That region is cut into cells
  of about :data:`CELL_SIZE` pixels; a cell keeps its corners at ``fast_threshold`` (20), or,
  if it has none, those at ``min_fast_threshold`` (7), so low-texture areas still contribute.
- **Quadtree distribution** (:func:`distribute`): the region is split into square-ish nodes,
  each node holding more than one corner is quartered, the most crowded first once the next
  round would overshoot the level's budget, until there are as many nodes as the budget;
  each node keeps its strongest corner.
- **Orientation:** the angle of the intensity centroid ``(m10, m01)`` over the radius-15 disc
  (:data:`UMAX`), quantised to :data:`ANGLE_BINS` bins of 12 degrees, as the ORB paper
  discretises it for its pattern lookup. Bin ``k`` is the one whose direction
  :data:`ROTATIONS` ``[k]`` has the largest dot product with ``(m10, m01)``.

**Exact by construction**, as :mod:`slamlab.foundations.image` is: scores, moments and bins
are integers, the bin directions are fixed-point constants, and level coordinates map to the
full image by one float64 product, so the three languages return identical keypoints.

Where this differs from ORB-SLAM, deliberately: non-maximum suppression runs over the whole
level rather than inside each cell, so a corner on a cell's edge yields to a stronger one
across it; quadtree ties break by raster order rather than by memory address; and the
orientation is a bin rather than a float angle.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations.arrays import MAX_IMAGE_SIDE, as_image
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.image import level_shapes, pyramid

EDGE_THRESHOLD = 19
"""Keypoints lie at least this many pixels inside their level (ORB-SLAM's): the rotated
descriptor pattern reaches at most 18.4 pixels from its centre."""

CELL_SIZE = 30
"""The target side of a detection cell, in pixels (ORB-SLAM's)."""

FAST_CIRCLE = (
    (0, 3), (1, 3), (2, 2), (3, 1), (3, 0), (3, -1), (2, -2), (1, -3),
    (0, -3), (-1, -3), (-2, -2), (-3, -1), (-3, 0), (-3, 1), (-2, 2), (-1, 3),
)  # fmt: skip
"""The 16 ``(dx, dy)`` offsets of FAST's radius-3 Bresenham circle, in order around it."""

FAST_ARC = 9
"""The contiguous circle pixels a FAST-9 corner needs."""

HALF_PATCH_SIZE = 15
"""The radius of the orientation patch (ORB's 31-pixel patch)."""

UMAX = (15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3)
"""The orientation disc's half-width at each row offset ``|v|`` (OpenCV's and ORB-SLAM's
``umax``, made symmetric under a 90-degree turn)."""

ANGLE_BINS = 30
"""Orientation bins, 12 degrees each."""

ROTATION_BITS = 14
"""The fixed-point bits of :data:`ROTATIONS`."""

ROTATIONS = (
    (16384, 0), (16026, 3406), (14968, 6664), (13255, 9630), (10963, 12176),
    (8192, 14189), (5063, 15582), (1713, 16294), (-1713, 16294), (-5063, 15582),
    (-8192, 14189), (-10963, 12176), (-13255, 9630), (-14968, 6664), (-16026, 3406),
    (-16384, 0), (-16026, -3406), (-14968, -6664), (-13255, -9630), (-10963, -12176),
    (-8192, -14189), (-5063, -15582), (-1713, -16294), (1713, -16294), (5063, -15582),
    (8192, -14189), (10963, -12176), (13255, -9630), (14968, -6664), (16026, -3406),
)  # fmt: skip
"""``round(2^14 (cos, sin))`` of ``12 k`` degrees for each bin ``k``: the bin directions, and
the rotations of the descriptor's pattern."""

MAX_LEVELS = 32
"""The most pyramid levels :class:`Options` accepts."""


@dataclass(frozen=True)
class Options:
    """ORB extraction settings (ORB-SLAM's defaults for 640x480 images).

    Attributes
    ----------
    n_features : int
        The feature budget over all levels, from 1 to ``2^20``.
    levels : int
        Pyramid levels, from 1 to :data:`MAX_LEVELS`.
    scale_factor : float
        Each level's downscale from the previous one, above 1.
    fast_threshold : int
        The FAST threshold a cell tries first, from 0 to 254.
    min_fast_threshold : int
        The threshold for cells with no corner at `fast_threshold`, from 0 to
        `fast_threshold`.
    """

    n_features: int = 1000
    levels: int = 8
    scale_factor: float = 1.2
    fast_threshold: int = 20
    min_fast_threshold: int = 7

    def __post_init__(self) -> None:
        """Check the settings.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting is out of range.
        """
        if not 1 <= self.n_features <= MAX_IMAGE_SIDE:
            raise InvalidInputError(
                f"n_features: must be from 1 to {MAX_IMAGE_SIDE}, got {self.n_features}"
            )
        if not 1 <= self.levels <= MAX_LEVELS:
            raise InvalidInputError(f"levels: must be from 1 to {MAX_LEVELS}, got {self.levels}")
        if not (math.isfinite(self.scale_factor) and self.scale_factor > 1.0):
            raise InvalidInputError(
                f"scale_factor: must be finite and above 1, got {self.scale_factor}"
            )
        if not 0 <= self.fast_threshold <= 254:  # the largest FAST score
            raise InvalidInputError(
                f"fast_threshold: must be from 0 to 254, got {self.fast_threshold}"
            )
        if not 0 <= self.min_fast_threshold <= self.fast_threshold:
            raise InvalidInputError(
                "min_fast_threshold: must be from 0 to fast_threshold, "
                f"got {self.min_fast_threshold}"
            )


@dataclass(frozen=True)
class Keypoints:
    """ORB keypoints, one row per keypoint, ordered by level, then row, then column.

    Attributes
    ----------
    xy : ndarray of float64, shape (N, 2)
        Pixel coordinates in the full image: the level's pixel times ``scale_factor^level``.
    level : ndarray of int64, shape (N,)
        The pyramid level each was detected on.
    level_xy : ndarray of int64, shape (N, 2)
        The integer pixel in its level.
    response : ndarray of int64, shape (N,)
        The FAST score.
    angle : ndarray of int64, shape (N,)
        The orientation bin, in ``[0, ANGLE_BINS)``: an angle of ``12 * angle`` degrees,
        measured from the x axis toward the y axis (clockwise on screen, y pointing down).
    """

    xy: NDArray[np.float64]
    level: NDArray[np.int64]
    level_xy: NDArray[np.int64]
    response: NDArray[np.int64]
    angle: NDArray[np.int64]


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.orb`` submodule provides."""

    def fast_scores(self, image: NDArray[np.uint8]) -> NDArray[np.int64]:
        """Return every pixel's FAST-9 score."""
        ...

    def distribute(
        self,
        xy: NDArray[np.int64],
        response: NDArray[np.int64],
        bounds: tuple[int, int, int, int],
        target: int,
    ) -> NDArray[np.int64]:
        """Return the indices of the corners a quadtree keeps."""
        ...

    def detect(self, image: NDArray[np.uint8], options: dict[str, Any]) -> dict[str, Any]:
        """Detect oriented keypoints; return a dict of :class:`Keypoints`' fields."""
        ...


def _offsets() -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """Return the orientation disc's pixel offsets ``(u, v)``, row by row."""
    us: list[int] = []
    vs: list[int] = []
    for v in range(-HALF_PATCH_SIZE, HALF_PATCH_SIZE + 1):
        width = UMAX[abs(v)]
        us.extend(range(-width, width + 1))
        vs.extend([v] * (2 * width + 1))
    return np.asarray(us, dtype=np.int64), np.asarray(vs, dtype=np.int64)


_DISC_U, _DISC_V = _offsets()


def level_budgets(n_features: int, levels: int, scale_factor: float) -> list[int]:
    """Split the feature budget over the levels, falling geometrically with the level.

    Level ``l`` gets ``round(d_l)``, half up, with ``d_0 = n (1 - f) / (1 - f^L)``,
    ``d_{l+1} = f d_l`` and ``f = 1 / scale_factor`` (``f^L`` by repeated multiplication);
    the last level gets what remains, so the budgets sum to `n_features` (ORB-SLAM's split).

    Parameters
    ----------
    n_features : int
        The total budget.
    levels : int
        The number of levels.
    scale_factor : float
        The pyramid's scale factor, above 1.

    Returns
    -------
    list of int
        One budget per level.
    """
    factor = 1.0 / scale_factor
    power = 1.0
    for _ in range(levels):
        power *= factor
    desired = n_features * (1.0 - factor) / (1.0 - power)
    budgets = []
    for _ in range(levels - 1):
        budgets.append(math.floor(desired + 0.5))
        desired *= factor
    budgets.append(max(n_features - sum(budgets), 0))
    return budgets


def _fast_scores(image: NDArray[np.uint8]) -> NDArray[np.int64]:
    """Every pixel's FAST-9 score (reference)."""
    rows, cols = image.shape
    out = np.full((rows, cols), -1, dtype=np.int64)
    if rows <= 6 or cols <= 6:  # the circle needs 3 pixels on every side
        return out
    centre = image[3 : rows - 3, 3 : cols - 3].astype(np.int16)
    # Each circle pixel minus the centre, then the same 16 again so every arc is contiguous.
    differences = np.stack(
        [
            image[3 + dy : rows - 3 + dy, 3 + dx : cols - 3 + dx].astype(np.int16) - centre
            for dx, dy in FAST_CIRCLE
        ],
        axis=-1,
    )
    wrapped = np.concatenate([differences, differences[..., : FAST_ARC - 1]], axis=-1)
    low = wrapped[..., : len(FAST_CIRCLE)].copy()
    high = low.copy()
    for shift in range(1, FAST_ARC):
        arc = wrapped[..., shift : shift + len(FAST_CIRCLE)]
        np.minimum(low, arc, out=low)
        np.maximum(high, arc, out=high)
    # The best arc's smallest margin, brighter or darker, less one: the largest threshold the
    # pixel passes, since a corner needs every margin strictly above the threshold.
    margin = np.maximum(low.max(axis=-1), (-high).max(axis=-1)).astype(np.int64)
    out[3 : rows - 3, 3 : cols - 3] = np.maximum(margin - 1, -1)
    return out


def _local_maxima(scores: NDArray[np.int64]) -> NDArray[np.bool_]:
    """Pixels whose score is strictly above all eight neighbours'."""
    padded = np.pad(scores, 1, constant_values=-1)
    rows, cols = scores.shape
    out = np.ones((rows, cols), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx or dy:
                out &= scores > padded[1 + dy : rows + 1 + dy, 1 + dx : cols + 1 + dx]
    return out


@dataclass
class _Node:
    """A quadtree node: a box ``[x0, x1) x [y0, y1)`` and the corners inside it."""

    x0: int
    x1: int
    y0: int
    y1: int
    members: list[int]

    def split(self, xy: NDArray[np.int64]) -> list[_Node]:
        """Quarter the box at its rounded-up midpoint; return the non-empty quarters."""
        mx = self.x0 + (self.x1 - self.x0 + 1) // 2
        my = self.y0 + (self.y1 - self.y0 + 1) // 2
        quarters = [
            _Node(self.x0, mx, self.y0, my, []),
            _Node(mx, self.x1, self.y0, my, []),
            _Node(self.x0, mx, my, self.y1, []),
            _Node(mx, self.x1, my, self.y1, []),
        ]
        for i in self.members:
            x, y = int(xy[i, 0]), int(xy[i, 1])
            quarters[(x >= mx) + 2 * (y >= my)].members.append(i)
        return [q for q in quarters if q.members]


def _distribute(
    xy: NDArray[np.int64],
    response: NDArray[np.int64],
    bounds: tuple[int, int, int, int],
    target: int,
) -> NDArray[np.int64]:
    """Spread corners with a quadtree (reference)."""
    if len(xy) == 0 or target == 0:
        return np.zeros(0, dtype=np.int64)
    x0, x1, y0, y1 = bounds
    width, height = x1 - x0, y1 - y0
    roots = max(1, (2 * width + height) // (2 * height))  # round(width / height), half up
    edges = [x0 + (k * width) // roots for k in range(roots + 1)]
    nodes = [_Node(edges[k], edges[k + 1], y0, y1, []) for k in range(roots)]
    for i in range(len(xy)):
        nodes[bisect.bisect_right(edges, int(xy[i, 0])) - 1].members.append(i)
    nodes = [node for node in nodes if node.members]
    while len(nodes) < target:
        # A box of one pixel cannot be split further: its corners coincide.
        crowded = [
            k
            for k, node in enumerate(nodes)
            if len(node.members) > 1 and (node.x1 - node.x0) * (node.y1 - node.y0) > 1
        ]
        if not crowded:
            break
        if len(nodes) + 3 * len(crowded) > target:  # a full round would overshoot
            crowded.sort(key=lambda k: (-len(nodes[k].members), k))
        quartered: dict[int, list[_Node]] = {}
        count = len(nodes)
        for k in crowded:
            if count >= target:
                break
            quartered[k] = nodes[k].split(xy)
            count += len(quartered[k]) - 1
        nodes = [child for k, node in enumerate(nodes) for child in quartered.get(k, [node])]
    kept = [max(node.members, key=lambda i: (response[i], -i)) for node in nodes]
    return np.asarray(sorted(kept), dtype=np.int64)


def _orientation(image: NDArray[np.uint8], xy: NDArray[np.int64]) -> NDArray[np.int64]:
    """Each keypoint's orientation bin, from its disc's intensity centroid."""
    if len(xy) == 0:
        return np.zeros(0, dtype=np.int64)
    values = image[xy[:, 1:2] + _DISC_V, xy[:, 0:1] + _DISC_U].astype(np.int64)
    m10, m01 = values @ _DISC_U, values @ _DISC_V
    directions = np.asarray(ROTATIONS, dtype=np.int64)
    dots = m10[:, None] * directions[:, 0] + m01[:, None] * directions[:, 1]
    bins: NDArray[np.int64] = np.argmax(dots, axis=1).astype(np.int64)  # ties: the lowest
    return bins


def _detect_level(
    image: NDArray[np.uint8], budget: int, options: Options
) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    """One level's distributed corners: their pixels and scores."""
    rows, cols = image.shape
    x0, y0 = EDGE_THRESHOLD, EDGE_THRESHOLD
    x1, y1 = cols - EDGE_THRESHOLD, rows - EDGE_THRESHOLD
    if x1 <= x0 or y1 <= y0 or budget == 0:
        return np.zeros((0, 2), dtype=np.int64), np.zeros(0, dtype=np.int64)
    scores = _fast_scores(image)
    maxima = _local_maxima(scores)
    width, height = x1 - x0, y1 - y0
    n_cols, n_rows = max(width // CELL_SIZE, 1), max(height // CELL_SIZE, 1)
    cell_w, cell_h = -(-width // n_cols), -(-height // n_rows)
    keep = np.zeros_like(maxima)
    for top in range(y0, y1, cell_h):
        for left in range(x0, x1, cell_w):
            cell = np.s_[top : min(top + cell_h, y1), left : min(left + cell_w, x1)]
            strong = maxima[cell] & (scores[cell] >= options.fast_threshold)
            if not strong.any():
                strong = maxima[cell] & (scores[cell] >= options.min_fast_threshold)
            keep[cell] = strong
    ys, xs = np.nonzero(keep)  # raster order
    xy = np.column_stack([xs, ys]).astype(np.int64)
    response = scores[ys, xs]
    kept = _distribute(xy, response, (x0, x1, y0, y1), budget)
    return xy[kept], response[kept]


def _detect(image: NDArray[np.uint8], options: Options) -> Keypoints:
    """Detect oriented keypoints on every level (reference)."""
    levels = pyramid(image, options.levels, options.scale_factor)
    budgets = level_budgets(options.n_features, options.levels, options.scale_factor)
    xy_parts: list[NDArray[np.float64]] = []
    level_parts: list[NDArray[np.int64]] = []
    pixel_parts: list[NDArray[np.int64]] = []
    response_parts: list[NDArray[np.int64]] = []
    angle_parts: list[NDArray[np.int64]] = []
    scale = 1.0
    for level, (pixels, budget) in enumerate(zip(levels, budgets, strict=True)):
        if level > 0:
            scale *= options.scale_factor
        xy, response = _detect_level(pixels, budget, options)
        xy_parts.append(xy.astype(np.float64) * scale)
        level_parts.append(np.full(len(xy), level, dtype=np.int64))
        pixel_parts.append(xy)
        response_parts.append(response)
        angle_parts.append(_orientation(pixels, xy))
    return Keypoints(
        xy=np.concatenate(xy_parts).reshape(-1, 2),
        level=np.concatenate(level_parts),
        level_xy=np.concatenate(pixel_parts).reshape(-1, 2),
        response=np.concatenate(response_parts),
        angle=np.concatenate(angle_parts),
    )


def _decode_keypoints(raw: dict[str, Any], codes: backends.Codes) -> Keypoints:
    """Rebuild :class:`Keypoints` from a compiled backend's dict."""
    del codes  # the ORB module crosses no codes
    return Keypoints(
        xy=np.asarray(raw["xy"], dtype=np.float64).reshape(-1, 2),
        level=np.asarray(raw["level"], dtype=np.int64),
        level_xy=np.asarray(raw["level_xy"], dtype=np.int64).reshape(-1, 2),
        response=np.asarray(raw["response"], dtype=np.int64),
        angle=np.asarray(raw["angle"], dtype=np.int64),
    )


def _decode_indices(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's int64 array, checked (never cast)."""
    del codes
    out = np.asarray(raw)
    if out.dtype != np.int64:
        raise RuntimeError(f"orb: backend returned {out.dtype}, not int64")
    return out


def fast_scores(image: ArrayLike, *, backend: str = "python") -> NDArray[np.int64]:
    """Return every pixel's FAST-9 score: the largest threshold at which it is a corner.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The image.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64, shape (H, W)
        The score ``t`` such that the pixel is a corner at every threshold up to ``t``: some
        9 contiguous circle pixels are all brighter than it by more than ``t``, or all darker
        by more than ``t``. -1 where it is a corner at no threshold, including within 3
        pixels of the border. OpenCV's ``FAST`` reports the same scores.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array.
    """
    pixels = as_image(image, "image")
    return backends.call(
        backend,
        __name__,
        "fast_scores",
        reference=lambda: _fast_scores(pixels),
        arguments={"image": pixels},
        decode=_decode_indices,
    )


def _as_corners(
    xy: ArrayLike, response: ArrayLike, bounds: tuple[int, int, int, int], target: int
) -> tuple[NDArray[np.int64], NDArray[np.int64], tuple[int, int, int, int], int]:
    """Check :func:`distribute`'s arguments; return them as int64 arrays and Python ints."""
    points, strengths = np.asarray(xy), np.asarray(response)
    if points.ndim != 2 or points.shape[1] != 2 or not np.can_cast(points.dtype, np.int64):
        raise InvalidInputError(
            f"xy: expected an (N, 2) integer array, got {points.dtype} {points.shape}"
        )
    if strengths.shape != (len(points),) or not np.can_cast(strengths.dtype, np.int64):
        raise InvalidInputError(
            f"response: expected {len(points)} integers, got {strengths.dtype} {strengths.shape}"
        )
    box = tuple(bounds)
    if len(box) != 4 or any(
        isinstance(b, bool) or not isinstance(b, (int, np.integer)) for b in box
    ):
        raise InvalidInputError(f"bounds: expected four integers, got {bounds!r}")
    x0, x1, y0, y1 = (int(b) for b in box)
    if not (0 <= x0 < x1 <= MAX_IMAGE_SIDE and 0 <= y0 < y1 <= MAX_IMAGE_SIDE):
        raise InvalidInputError(
            f"bounds: expected a non-empty box inside [0, {MAX_IMAGE_SIDE}]^2, got {box}"
        )
    corners = points.astype(np.int64)
    inside = (
        (corners[:, 0] >= x0) & (corners[:, 0] < x1) & (corners[:, 1] >= y0) & (corners[:, 1] < y1)
    )
    if not np.all(inside):
        raise InvalidInputError("xy: every corner must lie inside bounds")
    if isinstance(target, bool) or not isinstance(target, (int, np.integer)):
        raise InvalidInputError(f"target: must be an integer, got {type(target).__name__}")
    if not 0 <= int(target) <= MAX_IMAGE_SIDE:
        raise InvalidInputError(f"target: must be from 0 to {MAX_IMAGE_SIDE}, got {target}")
    return corners, strengths.astype(np.int64), (x0, x1, y0, y1), int(target)


def distribute(
    xy: ArrayLike,
    response: ArrayLike,
    bounds: tuple[int, int, int, int],
    target: int,
    *,
    backend: str = "python",
) -> NDArray[np.int64]:
    """Spread corners evenly with ORB-SLAM's quadtree; return the ones kept.

    The box is first cut into ``round(width / height)`` columns. Then, while there are fewer
    nodes than `target`, every node holding more than one corner is quartered at its
    midpoint (rounded up), dropping empty quarters; once a full round could overshoot
    `target`, nodes are quartered one at a time, the most crowded first (ties: the earlier
    node), until there are `target` nodes. Each node keeps its highest-response corner (ties:
    the earliest).

    Parameters
    ----------
    xy : array_like of int, shape (N, 2)
        Integer pixel coordinates, each inside `bounds`.
    response : array_like of int, shape (N,)
        Each corner's strength.
    bounds : tuple of int
        ``(x_min, x_max, y_min, y_max)``: the half-open box the corners lie in, non-empty and
        inside ``[0, MAX_IMAGE_SIDE]`` on both axes.
    target : int
        How many corners to keep, from 0 to :data:`slamlab.foundations.arrays.MAX_IMAGE_SIDE`.
        Fewer are kept when the corners run out; up to two more when the last quartering adds
        several nodes at once, or more when the initial columns alone outnumber `target`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of int64
        The indices of the kept corners, ascending.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If an argument is malformed, or a corner lies outside `bounds`.
    """
    corners, strengths, box, count = _as_corners(xy, response, bounds, target)
    return backends.call(
        backend,
        __name__,
        "distribute",
        reference=lambda: _distribute(corners, strengths, box, count),
        arguments={"xy": corners, "response": strengths, "bounds": box, "target": count},
        decode=_decode_indices,
    )


def detect(
    image: ArrayLike, options: Options | None = None, *, backend: str = "python"
) -> Keypoints:
    """Detect ORB keypoints as ORB-SLAM does: spread FAST corners, oriented, on every level.

    Parameters
    ----------
    image : array_like of uint8, shape (H, W)
        The grayscale image.
    options : Options, optional
        The settings (default: ORB-SLAM's for 640x480 images).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Keypoints
        About ``options.n_features`` keypoints, fewer on an image with little texture.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, or a pyramid level would be empty.
    """
    pixels = as_image(image, "image")
    settings = Options() if options is None else options
    level_shapes(pixels.shape[0], pixels.shape[1], settings.levels, settings.scale_factor)
    return backends.call(
        backend,
        __name__,
        "detect",
        reference=lambda: _detect(pixels, settings),
        arguments={"image": pixels, "options": asdict(settings)},
        decode=_decode_keypoints,
    )
