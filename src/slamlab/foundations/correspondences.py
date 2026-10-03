"""Building blocks for estimating geometry from correspondences between two views.

Unchecked helpers shared by the two-view estimators (:mod:`slamlab.foundations.epipolar`,
:mod:`slamlab.foundations.homography`) and, later, absolute pose: homogeneous and normalised
image points, Hartley's normalisation, the null vector of a linear system, and
:class:`PairEstimator`, the RANSAC estimator they all instantiate. They take arrays their
callers have already validated (with :mod:`slamlab.foundations.arrays`) and check nothing
themselves.

The C++ core mirrors them in ``slamlab/foundations/correspondences.hpp``.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

RANK_TOLERANCE = 1e-10
"""The 8-point system's second-smallest singular value must exceed this fraction of its
largest; below it the solution is not unique."""

Matrix3 = NDArray[np.float64]
"""A (3, 3) essential or fundamental matrix."""


def homogeneous(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Append a 1 to each 2D point.

    Parameters
    ----------
    points : ndarray of float64, shape (N, 2)
        The points.

    Returns
    -------
    ndarray of float64, shape (N, 3)
        ``(x, y, 1)`` per point.
    """
    return np.column_stack([points, np.ones(points.shape[0])])


def normalised_plane(bearings: NDArray[np.float64]) -> NDArray[np.float64]:
    """Map bearings with positive z to homogeneous normalised image points ``f / f_z``.

    Parameters
    ----------
    bearings : ndarray of float64, shape (N, 3)
        Bearings with positive z.

    Returns
    -------
    ndarray of float64, shape (N, 3)
        ``(x, y, 1)`` per bearing.
    """
    return np.asarray(bearings / bearings[:, 2:3], dtype=np.float64)


def hartley_normalisation(points: NDArray[np.float64]) -> NDArray[np.float64] | None:
    """Return Hartley's similarity: centroid to the origin, mean distance √2.

    Parameters
    ----------
    points : ndarray of float64, shape (N, 2)
        The points.

    Returns
    -------
    ndarray of float64, shape (3, 3), or None
        ``T`` with ``T (x, y, 1)`` normalised; None if every point coincides.
    """
    centroid = points.mean(axis=0)
    spread = float(np.mean(np.linalg.norm(points - centroid, axis=1)))
    if not spread > 0.0:
        return None
    scale = np.sqrt(2.0) / spread
    return np.array(
        [[scale, 0.0, -scale * centroid[0]], [0.0, scale, -scale * centroid[1]], [0.0, 0.0, 1.0]]
    )


def null_vector(system: NDArray[np.float64]) -> NDArray[np.float64] | None:
    """Solve ``A v = 0`` for a unit ``v`` by SVD, if the solution is unique.

    Parameters
    ----------
    system : ndarray of float64, shape (M, 9)
        ``A``, with M >= 8.

    Returns
    -------
    ndarray of float64, shape (9,), or None
        The right singular vector of the smallest singular value; None if the
        second-smallest is below :data:`RANK_TOLERANCE` of the largest.
    """
    _, singular, vt = np.linalg.svd(system, full_matrices=True)
    if not singular[7] > RANK_TOLERANCE * singular[0]:
        return None
    return np.asarray(vt[-1], dtype=np.float64)


class PairEstimator:
    """A :class:`ransac.Estimator` over correspondence pairs whose model is a 3x3 matrix.

    The same solver serves as the minimal and the least-squares fit (the 8-point algorithm,
    the DLT), so one class carries E, F and H: what differs is the solver and the residual.

    Parameters
    ----------
    data : tuple of ndarray
        The validated correspondences, as the solver takes them (two arrays of N rows).
    solver : callable
        Fits a model to rows of both arrays; returns None when they do not determine one.
    residual : callable
        Returns every pair's squared residual under a model.
    sample_size : int
        The minimal sample size.
    """

    def __init__(
        self,
        data: tuple[NDArray[np.float64], NDArray[np.float64]],
        solver: Callable[[NDArray[np.float64], NDArray[np.float64]], Matrix3 | None],
        residual: Callable[[Matrix3], NDArray[np.float64]],
        sample_size: int,
    ) -> None:
        self._data, self._solver, self._residual = data, solver, residual
        self._sample_size = sample_size

    @property
    def sample_size(self) -> int:
        """The minimal sample size."""
        return self._sample_size

    @property
    def size(self) -> int:
        """The number of correspondences."""
        return int(self._data[0].shape[0])

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[Matrix3]:
        """Fit a minimal sample (the same solver as the least-squares fit)."""
        return self.fit_nonminimal(sample)

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[Matrix3]:
        """Fit a subset; no model if it does not determine one."""
        found = self._solver(self._data[0][subset], self._data[1][subset])
        return [] if found is None else [found]

    def residuals(self, model: Matrix3) -> NDArray[np.float64]:
        """Return every pair's squared residual."""
        return self._residual(model)
