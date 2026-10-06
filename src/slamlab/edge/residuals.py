"""Observation errors: each level's variance and the chi-square test (ORB-SLAM's).

A keypoint detected on pyramid level ``l`` is located to about ``s^l`` pixels (one at level
0), so its error's variance is ``s^(2 l)`` (ORB-SLAM's ``mvLevelSigma2``). A map point's
observation is an outlier when its squared reprojection error over that variance exceeds the
95% chi-square threshold: 2 dof for a keypoint, 3 for one with a right column, whose
predicted column ``u - f b / z`` is compared too (ORB-SLAM's stereo observations). Tracking,
bundle adjustment, triangulation and fusion all use this one test.
"""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.edge.frames import Camera
from slamlab.edge.map import Map
from slamlab.edge.options import Options
from slamlab.foundations.arrays import as_real
from slamlab.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


def sigma2(m: Map, level: IntArray) -> Array:
    """Each level's variance: ``s^(2 l)`` (one pixel at level 0).

    Parameters
    ----------
    m : Map
        The map (its scale factor).
    level : ndarray of int64
        The levels.

    Returns
    -------
    ndarray of float64
        The variances.
    """
    return np.asarray(m.scale_factor ** (2.0 * level), dtype=np.float64)


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.residuals`` submodule provides."""

    def chi2(
        self,
        camera: dict[str, Any],
        local: NDArray[np.float64],
        xy: NDArray[np.float64],
        right: NDArray[np.float64],
        variance: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Return each observation's chi-square error (infinite behind the camera)."""
        ...


def _chi2(camera: Camera, local: Array, xy: Array, right: Array, variance: Array) -> Array:
    """Each observation's chi-square error (reference)."""
    front = local[:, 2] > 0.0
    error = np.full(len(local), np.inf)
    if np.any(front):
        pixels = camera.project(local[front])
        squared = np.sum((pixels - xy[front]) ** 2, axis=1)
        predicted = pixels[:, 0] - camera.focal * camera.baseline / local[front, 2]
        stereo = ~np.isnan(right[front])
        squared[stereo] += (predicted[stereo] - right[front][stereo]) ** 2
        error[front] = squared / variance[front]
    return error


def chi2(  # noqa: PLR0913 -- the camera and the four per-observation arrays
    camera: Camera,
    local: ArrayLike,
    xy: ArrayLike,
    right: ArrayLike,
    variance: ArrayLike,
    *,
    backend: str = "python",
) -> Array:
    """Return each observation's chi-square error.

    ``(|π(X) - x|² + (π_u(X) - f b / Z - u_r)²) / σ²``, the right-column term only where the
    observation has one; infinite for a point not in front of the camera.

    Parameters
    ----------
    camera : slamlab.edge.frames.Camera
        The camera.
    local : array_like, shape (N, 3)
        The points, in the camera's frame.
    xy : array_like, shape (N, 2)
        Their observed pixels.
    right : array_like, shape (N,)
        Their observed right columns (NaN for none).
    variance : array_like, shape (N,)
        Their levels' variances, positive.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N,)
        The errors.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the arrays are not finite (NaN allowed in `right`), not one row per observation,
        or a variance is not positive.
    """
    points = as_real(local, "local").reshape(-1, 3)
    n = len(points)
    pixels = as_real(xy, "xy").reshape(-1, 2)
    columns = as_real(right, "right", allow_nan=True).reshape(-1)
    variances = as_real(variance, "variance").reshape(-1)
    if len(pixels) != n or columns.shape != (n,) or variances.shape != (n,):
        raise InvalidInputError("chi2: expected one row per observation in every array")
    if np.any(variances <= 0.0):
        raise InvalidInputError("variance: must be positive")
    return backends.call(
        backend,
        __name__,
        "chi2",
        reference=lambda: _chi2(camera, points, pixels, columns, variances),
        arguments={
            "camera": camera.as_dict(),
            "local": points,
            "xy": pixels,
            "right": columns,
            "variance": variances,
        },
        decode=lambda raw, codes: backends.checked_array(raw, np.float64, 1, "chi2 errors"),
    )


def thresholds(right: Array, options: Options) -> Array:
    """Each observation's chi-square threshold: 3 dof with a right column, else 2.

    Parameters
    ----------
    right : ndarray of float64, shape (N,)
        The observed right columns (NaN for none).
    options : Options
        The thresholds.

    Returns
    -------
    ndarray of float64, shape (N,)
        The thresholds.
    """
    return np.where(np.isnan(right), options.chi2_mono, options.chi2_stereo)
