"""ORB features as ORB-SLAM extracts them: spread FAST corners, oriented, with rBRIEF descriptors.

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

- **Descriptors (rBRIEF):** 256 intensity comparisons on the level blurred by a 7-tap
  Gaussian (:data:`BLUR_KERNEL`, sigma 2), between the pairs of OpenCV's learned pattern
  (:data:`PATTERN`, from Rublee et al.) rotated to the keypoint's bin: bit ``i`` is set when
  the pair's first pixel is darker than its second, packed least significant bit first into
  :data:`DESCRIPTOR_BYTES` bytes, as OpenCV and ORB-SLAM pack them. The rotated pairs are
  rounded from the fixed-point :data:`ROTATIONS`, half up.

**Exact by construction**, as :mod:`slamlab.foundations.image` is: scores, moments, bins and
descriptor bits are integers, the bin directions are fixed-point constants, and level
coordinates map to the full image by one float64 product, so the three languages return
identical features.

:data:`PATTERN` is OpenCV's ``bit_pattern_31_``, under OpenCV's BSD licence (its notice is
next to the table and in ``THIRD_PARTY_NOTICES``).

Where this differs from ORB-SLAM, deliberately: non-maximum suppression runs over the whole
level rather than inside each cell, so a corner on a cell's edge yields to a stronger one
across it; quadtree ties break by raster order rather than by memory address; and the
orientation is a bin rather than a float angle.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations.arrays import (
    MAX_IMAGE_SIDE,
    as_count,
    as_image,
    holds_integers,
)
from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.image import (
    as_scale_factor,
    blur,
    full_image_coordinate,
    level_shapes,
    pyramid,
)

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

MAX_FEATURES = 1 << 20
"""The largest feature budget :class:`Options` and :func:`distribute` accept: a bound on the
work one call may ask for, far above any real budget (ORB-SLAM extracts 1000 to 2000)."""

MAX_FAST_THRESHOLD = 254
"""The largest FAST threshold: a margin of 255, the most an 8-bit difference can be, less
one (a corner needs every margin strictly above the threshold)."""

BLUR_KERNEL = (287, 537, 781, 886, 781, 537, 287)
"""The descriptor's smoothing: a 7-tap Gaussian, sigma 2 (ORB-SLAM's), in integers summing to
``2^12`` (:func:`slamlab.foundations.image.blur`)."""

DESCRIPTOR_BYTES = 32
"""A descriptor's length: 256 bits."""

# rBRIEF's learned test pairs (x1, y1, x2, y2): OpenCV's bit_pattern_31_
# (modules/features2d/src/orb.cpp), learned by Ethan Rublee, Vincent Rabaud and Gary Bradski
# for the ORB paper. Reproduced under OpenCV's licence for that file:
#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2009, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
PATTERN = (
    (8, -3, 9, 5), (4, 2, 7, -12), (-11, 9, -8, 2), (7, -12, 12, -13), (2, -13, 2, 12),
    (1, -7, 1, 6), (-2, -10, -2, -4), (-13, -13, -11, -8), (-13, -3, -12, -9), (10, 4, 11, 9),
    (-13, -8, -8, -9), (-11, 7, -9, 12), (7, 7, 12, 6), (-4, -5, -3, 0), (-13, 2, -12, -3),
    (-9, 0, -7, 5), (12, -6, 12, -1), (-3, 6, -2, 12), (-6, -13, -4, -8), (11, -13, 12, -8),
    (4, 7, 5, 1), (5, -3, 10, -3), (3, -7, 6, 12), (-8, -7, -6, -2), (-2, 11, -1, -10),
    (-13, 12, -8, 10), (-7, 3, -5, -3), (-4, 2, -3, 7), (-10, -12, -6, 11), (5, -12, 6, -7),
    (5, -6, 7, -1), (1, 0, 4, -5), (9, 11, 11, -13), (4, 7, 4, 12), (2, -1, 4, 4),
    (-4, -12, -2, 7), (-8, -5, -7, -10), (4, 11, 9, 12), (0, -8, 1, -13), (-13, -2, -8, 2),
    (-3, -2, -2, 3), (-6, 9, -4, -9), (8, 12, 10, 7), (0, 9, 1, 3), (7, -5, 11, -10),
    (-13, -6, -11, 0), (10, 7, 12, 1), (-6, -3, -6, 12), (10, -9, 12, -4), (-13, 8, -8, -12),
    (-13, 0, -8, -4), (3, 3, 7, 8), (5, 7, 10, -7), (-1, 7, 1, -12), (3, -10, 5, 6),
    (2, -4, 3, -10), (-13, 0, -13, 5), (-13, -7, -12, 12), (-13, 3, -11, 8), (-7, 12, -4, 7),
    (6, -10, 12, 8), (-9, -1, -7, -6), (-2, -5, 0, 12), (-12, 5, -7, 5), (3, -10, 8, -13),
    (-7, -7, -4, 5), (-3, -2, -1, -7), (2, 9, 5, -11), (-11, -13, -5, -13), (-1, 6, 0, -1),
    (5, -3, 5, 2), (-4, -13, -4, 12), (-9, -6, -9, 6), (-12, -10, -8, -4), (10, 2, 12, -3),
    (7, 12, 12, 12), (-7, -13, -6, 5), (-4, 9, -3, 4), (7, -1, 12, 2), (-7, 6, -5, 1),
    (-13, 11, -12, 5), (-3, 7, -2, -6), (7, -8, 12, -7), (-13, -7, -11, -12), (1, -3, 12, 12),
    (2, -6, 3, 0), (-4, 3, -2, -13), (-1, -13, 1, 9), (7, 1, 8, -6), (1, -1, 3, 12),
    (9, 1, 12, 6), (-1, -9, -1, 3), (-13, -13, -10, 5), (7, 7, 10, 12), (12, -5, 12, 9),
    (6, 3, 7, 11), (5, -13, 6, 10), (2, -12, 2, 3), (3, 8, 4, -6), (2, 6, 12, -13),
    (9, -12, 10, 3), (-8, 4, -7, 9), (-11, 12, -4, -6), (1, 12, 2, -8), (6, -9, 7, -4),
    (2, 3, 3, -2), (6, 3, 11, 0), (3, -3, 8, -8), (7, 8, 9, 3), (-11, -5, -6, -4),
    (-10, 11, -5, 10), (-5, -8, -3, 12), (-10, 5, -9, 0), (8, -1, 12, -6), (4, -6, 6, -11),
    (-10, 12, -8, 7), (4, -2, 6, 7), (-2, 0, -2, 12), (-5, -8, -5, 2), (7, -6, 10, 12),
    (-9, -13, -8, -8), (-5, -13, -5, -2), (8, -8, 9, -13), (-9, -11, -9, 0), (1, -8, 1, -2),
    (7, -4, 9, 1), (-2, 1, -1, -4), (11, -6, 12, -11), (-12, -9, -6, 4), (3, 7, 7, 12),
    (5, 5, 10, 8), (0, -4, 2, 8), (-9, 12, -5, -13), (0, 7, 2, 12), (-1, 2, 1, 7),
    (5, 11, 7, -9), (3, 5, 6, -8), (-13, -4, -8, 9), (-5, 9, -3, -3), (-4, -7, -3, -12),
    (6, 5, 8, 0), (-7, 6, -6, 12), (-13, 6, -5, -2), (1, -10, 3, 10), (4, 1, 8, -4),
    (-2, -2, 2, -13), (2, -12, 12, 12), (-2, -13, 0, -6), (4, 1, 9, 3), (-6, -10, -3, -5),
    (-3, -13, -1, 1), (7, 5, 12, -11), (4, -2, 5, -7), (-13, 9, -9, -5), (7, 1, 8, 6),
    (7, -8, 7, 6), (-7, -4, -7, 1), (-8, 11, -7, -8), (-13, 6, -12, -8), (2, 4, 3, 9),
    (10, -5, 12, 3), (-6, -5, -6, 7), (8, -3, 9, -8), (2, -12, 2, 8), (-11, -2, -10, 3),
    (-12, -13, -7, -9), (-11, 0, -10, -5), (5, -3, 11, 8), (-2, -13, -1, 12), (-1, -8, 0, 9),
    (-13, -11, -12, -5), (-10, -2, -10, 11), (-3, 9, -2, -13), (2, -3, 3, 2), (-9, -13, -4, 0),
    (-4, 6, -3, -10), (-4, 12, -2, -7), (-6, -11, -4, 9), (6, -3, 6, 11), (-13, 11, -5, 5),
    (11, 11, 12, 6), (7, -5, 12, -2), (-1, 12, 0, 7), (-4, -8, -3, -2), (-7, 1, -6, 7),
    (-13, -12, -8, -13), (-7, -2, -6, -8), (-8, 5, -6, -9), (-5, -1, -4, 5), (-13, 7, -8, 10),
    (1, 5, 5, -13), (1, 0, 10, -13), (9, 12, 10, -1), (5, -8, 10, -9), (-1, 11, 1, -13),
    (-9, -3, -6, 2), (-1, -10, 1, 12), (-13, 1, -8, -10), (8, -11, 10, -6), (2, -13, 3, -6),
    (7, -13, 12, -9), (-10, -10, -5, -7), (-10, -8, -8, -13), (4, -6, 8, 5), (3, 12, 8, -13),
    (-4, 2, -3, -3), (5, -13, 10, -12), (4, -13, 5, -1), (-9, 9, -4, 3), (0, 3, 3, -9),
    (-12, 1, -6, 1), (3, 2, 4, -8), (-10, -10, -10, 9), (8, -13, 12, 12), (-8, -12, -6, -5),
    (2, 2, 3, 7), (10, 6, 11, -8), (6, 8, 8, -12), (-7, 10, -6, 5), (-3, -9, -3, 9),
    (-1, -13, -1, 5), (-3, -7, -3, 4), (-8, -2, -8, 3), (4, 2, 12, 12), (2, -5, 3, 11),
    (6, -9, 11, -13), (3, -1, 7, 12), (11, -1, 12, 4), (-3, 0, -3, 6), (4, -11, 4, 12),
    (2, -4, 2, 1), (-10, -6, -8, 1), (-13, 7, -11, 1), (-13, 12, -11, -13), (6, 0, 11, -13),
    (0, -1, 1, 4), (-13, 3, -9, -2), (-9, 8, -6, -3), (-13, -6, -8, -2), (5, -9, 8, 10),
    (2, 7, 3, -9), (-1, -6, -1, -1), (9, 5, 11, -2), (11, -3, 12, -8), (3, 0, 3, 5),
    (-1, 4, 0, 10), (3, -6, 4, 5), (-13, 0, -10, 5), (5, 8, 12, 11), (8, 9, 9, -6),
    (7, -4, 8, -12), (-10, 4, -10, 9), (7, 3, 12, 4), (9, -7, 10, -2), (7, 0, 12, -2),
    (-1, -6, 0, -11),
)  # fmt: skip
"""rBRIEF's 256 test pairs ``(x1, y1, x2, y2)``, in pixels from the keypoint (OpenCV's)."""


@dataclass(frozen=True)
class Options:
    """ORB extraction settings (ORB-SLAM's defaults for 640x480 images).

    Attributes
    ----------
    n_features : int
        The feature budget over all levels, from 1 to :data:`MAX_FEATURES`.
    levels : int
        Pyramid levels, from 1 to :data:`MAX_LEVELS`.
    scale_factor : float
        Each level's downscale from the previous one, above 1.
    fast_threshold : int
        The FAST threshold a cell tries first, from 1 to :data:`MAX_FAST_THRESHOLD`.
    min_fast_threshold : int
        The threshold for cells with no corner at `fast_threshold`, from 1 to
        `fast_threshold`. (From 1, FAST's corners are OpenCV's exactly: at 0, OpenCV
        suppresses a corner of score 0 against non-corners it also scores 0.)
    """

    n_features: int = 1000
    levels: int = 8
    scale_factor: float = 1.2
    fast_threshold: int = 20
    min_fast_threshold: int = 7

    def __post_init__(self) -> None:
        """Check the settings' types and ranges.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting has the wrong type or is out of range.
        """
        set_ = object.__setattr__  # frozen: store the checked, normalised values
        set_(self, "n_features", as_count(self.n_features, "n_features", 1, MAX_FEATURES))
        set_(self, "levels", as_count(self.levels, "levels", 1, MAX_LEVELS))
        set_(self, "scale_factor", as_scale_factor(self.scale_factor))
        threshold = as_count(self.fast_threshold, "fast_threshold", 1, MAX_FAST_THRESHOLD)
        set_(self, "fast_threshold", threshold)
        set_(
            self,
            "min_fast_threshold",
            as_count(self.min_fast_threshold, "min_fast_threshold", 1, threshold),
        )


@dataclass(frozen=True)
class Keypoints:
    """ORB keypoints, one row per keypoint, ordered by level, then row, then column.

    Attributes
    ----------
    xy : ndarray of float64, shape (N, 2)
        Coordinates in the full image: the level pixel mapped through the pyramid's resizes
        exactly (:func:`slamlab.foundations.image.full_image_coordinate`); on level 0, the
        pixel itself.
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


@dataclass(frozen=True)
class Features:
    """ORB features: keypoints and their descriptors, row for row, and the pyramid they span.

    Attributes
    ----------
    keypoints : Keypoints
        The keypoints.
    descriptors : ndarray of uint8, shape (N, DESCRIPTOR_BYTES)
        Each keypoint's 256-bit rBRIEF descriptor.
    scale_factor : float
        The pyramid's scale factor (:attr:`Options.scale_factor`).
    levels : int
        Its number of levels (:attr:`Options.levels`). With the image's shape, these two give
        every level's shape (:func:`slamlab.foundations.image.level_shapes`), so a consumer
        such as :func:`slamlab.foundations.stereo.match` needs no extraction settings.
    """

    keypoints: Keypoints
    descriptors: NDArray[np.uint8]
    scale_factor: float
    levels: int


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

    def extract(self, image: NDArray[np.uint8], options: dict[str, Any]) -> dict[str, Any]:
        """Extract features: :class:`Keypoints`' fields, and ``descriptors``."""
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


def _rotated_patterns() -> NDArray[np.int64]:
    """Return :data:`PATTERN` rotated to every bin: shape (ANGLE_BINS, 256, 4).

    ``(x, y)`` turns to ``(x c - y s, x s + y c) / 2^14``, rounded half up, for each bin's
    fixed-point ``(c, s)``.
    """
    base = np.asarray(PATTERN, dtype=np.int64)
    half = 1 << (ROTATION_BITS - 1)
    out = np.empty((ANGLE_BINS, len(PATTERN), 4), dtype=np.int64)
    for k, (c, s) in enumerate(ROTATIONS):
        for j in (0, 2):
            x, y = base[:, j], base[:, j + 1]
            out[k, :, j] = (x * c - y * s + half) >> ROTATION_BITS
            out[k, :, j + 1] = (x * s + y * c + half) >> ROTATION_BITS
    return out


_PATTERNS = _rotated_patterns()


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


def _detect_levels(levels: list[NDArray[np.uint8]], options: Options) -> Keypoints:
    """Detect oriented keypoints on every level of a pyramid (reference)."""
    budgets = level_budgets(options.n_features, options.levels, options.scale_factor)
    xy_parts: list[NDArray[np.float64]] = []
    level_parts: list[NDArray[np.int64]] = []
    pixel_parts: list[NDArray[np.int64]] = []
    response_parts: list[NDArray[np.int64]] = []
    angle_parts: list[NDArray[np.int64]] = []
    rows, cols = levels[0].shape
    for level, (pixels, budget) in enumerate(zip(levels, budgets, strict=True)):
        xy, response = _detect_level(pixels, budget, options)
        xy_parts.append(
            np.column_stack(
                [
                    full_image_coordinate(xy[:, 0], pixels.shape[1], cols),
                    full_image_coordinate(xy[:, 1], pixels.shape[0], rows),
                ]
            )
        )
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


def _detect(image: NDArray[np.uint8], options: Options) -> Keypoints:
    """Detect oriented keypoints (reference)."""
    return _detect_levels(pyramid(image, options.levels, options.scale_factor), options)


def _describe(levels: list[NDArray[np.uint8]], keypoints: Keypoints) -> NDArray[np.uint8]:
    """Compute every keypoint's rBRIEF descriptor on its blurred level (reference)."""
    bits = np.zeros((len(keypoints.level), len(PATTERN)), dtype=bool)
    for level, pixels in enumerate(levels):
        rows = np.flatnonzero(keypoints.level == level)
        if len(rows) == 0:
            continue
        smooth = blur(pixels, BLUR_KERNEL)
        x, y = keypoints.level_xy[rows, 0:1], keypoints.level_xy[rows, 1:2]
        pairs = _PATTERNS[keypoints.angle[rows]]  # (n, 256, 4)
        first = smooth[y + pairs[..., 1], x + pairs[..., 0]]
        second = smooth[y + pairs[..., 3], x + pairs[..., 2]]
        bits[rows] = first < second
    packed: NDArray[np.uint8] = np.packbits(
        bits.reshape(-1, DESCRIPTOR_BYTES, 8), axis=-1, bitorder="little"
    ).reshape(-1, DESCRIPTOR_BYTES)
    return packed


def _extract(image: NDArray[np.uint8], options: Options) -> Features:
    """Detect keypoints and describe them (reference)."""
    levels = pyramid(image, options.levels, options.scale_factor)
    keypoints = _detect_levels(levels, options)
    return Features(
        keypoints=keypoints,
        descriptors=_describe(levels, keypoints),
        scale_factor=options.scale_factor,
        levels=options.levels,
    )


def _decode_keypoints(raw: dict[str, Any], codes: backends.Codes) -> Keypoints:
    """Rebuild :class:`Keypoints` from a compiled backend's dict, each array checked."""
    del codes  # the ORB module crosses no codes
    check = backends.checked_array
    return Keypoints(
        xy=check(raw["xy"], np.float64, 2, "orb xy"),
        level=check(raw["level"], np.int64, 1, "orb level"),
        level_xy=check(raw["level_xy"], np.int64, 2, "orb level_xy"),
        response=check(raw["response"], np.int64, 1, "orb response"),
        angle=check(raw["angle"], np.int64, 1, "orb angle"),
    )


def _decode_features(raw: dict[str, Any], codes: backends.Codes) -> Features:
    """Rebuild :class:`Features` from a compiled backend's dict, each field checked."""
    scale_factor, levels = raw["scale_factor"], raw["levels"]
    if not isinstance(scale_factor, float) or type(levels) is not int:
        raise RuntimeError(
            f"orb features: backend returned scale_factor {type(scale_factor).__name__} and "
            f"levels {type(levels).__name__}, expected float and int"
        )
    return Features(
        keypoints=_decode_keypoints(raw, codes),
        descriptors=backends.checked_array(raw["descriptors"], np.uint8, 2, "orb descriptors"),
        scale_factor=scale_factor,
        levels=levels,
    )


def _decode_scores(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's FAST scores, checked (never cast)."""
    del codes
    out: NDArray[np.int64] = backends.checked_array(raw, np.int64, 2, "orb fast_scores")
    return out


def _decode_kept(raw: Any, codes: backends.Codes) -> NDArray[np.int64]:  # noqa: ANN401 -- a backend's array
    """Return a compiled backend's kept indices, checked (never cast)."""
    del codes
    out: NDArray[np.int64] = backends.checked_array(raw, np.int64, 1, "orb distribute")
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
        decode=_decode_scores,
    )


def _as_corners(
    xy: ArrayLike, response: ArrayLike, bounds: tuple[int, int, int, int], target: int
) -> tuple[NDArray[np.int64], NDArray[np.int64], tuple[int, int, int, int], int]:
    """Check :func:`distribute`'s arguments; return them as int64 arrays and Python ints."""
    points, strengths = np.asarray(xy), np.asarray(response)
    if points.ndim != 2 or points.shape[1] != 2 or not holds_integers(points):
        raise InvalidInputError(
            f"xy: expected an (N, 2) integer array, got {points.dtype} {points.shape}"
        )
    if strengths.shape != (len(points),) or not holds_integers(strengths):
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
    count = as_count(target, "target", 0, MAX_FEATURES)
    return corners, strengths.astype(np.int64), (x0, x1, y0, y1), count


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
        How many corners to keep, from 0 to :data:`MAX_FEATURES`.
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
        decode=_decode_kept,
    )


def _run(  # noqa: PLR0913 -- the call's two sides and its decoder
    image: ArrayLike,
    options: Options | None,
    *,
    backend: str,
    name: str,
    reference: Callable[[NDArray[np.uint8], Options], backends.ResultT],
    decode: Callable[[Any, backends.Codes], backends.ResultT],
) -> backends.ResultT:
    """Check an image and settings, then run one extractor function on the chosen backend."""
    pixels = as_image(image, "image")
    settings = Options() if options is None else options
    level_shapes(pixels.shape[0], pixels.shape[1], settings.levels, settings.scale_factor)
    return backends.call(
        backend,
        __name__,
        name,
        reference=lambda: reference(pixels, settings),
        arguments={"image": pixels, "options": asdict(settings)},
        decode=decode,
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
    return _run(
        image, options, backend=backend, name="detect", reference=_detect, decode=_decode_keypoints
    )


def extract(
    image: ArrayLike, options: Options | None = None, *, backend: str = "python"
) -> Features:
    """Extract ORB features as ORB-SLAM does: :func:`detect`, then rBRIEF descriptors.

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
    Features
        The keypoints :func:`detect` finds, and a 256-bit descriptor for each.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `image` is not a non-empty (H, W) uint8 array, or a pyramid level would be empty.
    """
    return _run(
        image, options, backend=backend, name="extract", reference=_extract, decode=_decode_features
    )
