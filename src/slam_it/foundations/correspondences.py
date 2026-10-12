"""Building blocks for estimating geometry from correspondences between two views.

Unchecked helpers shared by the two-view estimators (:mod:`slam_it.foundations.epipolar`,
:mod:`slam_it.foundations.homography`) and, later, absolute pose: homogeneous and normalised
image points, Hartley's normalisation, the null vector of a linear system, and
:class:`PairEstimator`, the RANSAC estimator they all instantiate. They take arrays their
callers have already validated (with :mod:`slam_it.foundations.arrays`) and check nothing
themselves.

The C++ core mirrors them in ``slam_it/foundations/correspondences.hpp``.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from slam_it.foundations.errors import DegenerateError

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


def normalised_xy(bearings: NDArray[np.float64]) -> NDArray[np.float64]:
    """Map bearings with positive z to normalised image points ``(f_x / f_z, f_y / f_z)``.

    Parameters
    ----------
    bearings : ndarray of float64, shape (N, 3)
        Bearings with positive z.

    Returns
    -------
    ndarray of float64, shape (N, 2)
        ``(x, y)`` per bearing: :func:`normalised_plane` without its ones.
    """
    return np.asarray(bearings[:, :2] / bearings[:, 2:3], dtype=np.float64)


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
        ``A``.

    Returns
    -------
    ndarray of float64, shape (9,), or None
        The right singular vector of the smallest singular value; None if the
        second-smallest is below :data:`RANK_TOLERANCE` of the largest, or there are fewer
        than eight rows (the solution is then not unique).
    """
    if system.shape[0] < 8:
        return None
    _, singular, vt = np.linalg.svd(system, full_matrices=True)
    if not singular[7] > RANK_TOLERANCE * singular[0]:
        return None
    return np.asarray(vt[-1], dtype=np.float64)


class PairEstimator:
    """A :class:`ransac.Estimator` over correspondence pairs whose model is a 3x3 matrix.

    One class carries E, F and H: what differs is the solvers and the residual. The
    least-squares solver also fits minimal samples (the 8-point algorithm, the DLT) unless a
    separate minimal solver is given (the 5-point solvers, which return several models).

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
    minimal : callable, optional
        Fits a minimal sample, returning every model it allows (default: `solver`).
    """

    def __init__(
        self,
        data: tuple[NDArray[np.float64], NDArray[np.float64]],
        solver: Callable[[NDArray[np.float64], NDArray[np.float64]], Matrix3 | None],
        residual: Callable[[Matrix3], NDArray[np.float64]],
        sample_size: int,
        minimal: Callable[[NDArray[np.float64], NDArray[np.float64]], list[Matrix3]] | None = None,
    ) -> None:
        self._data, self._solver, self._residual = data, solver, residual
        self._sample_size, self._minimal = sample_size, minimal

    @property
    def sample_size(self) -> int:
        """The minimal sample size."""
        return self._sample_size

    @property
    def size(self) -> int:
        """The number of correspondences."""
        return int(self._data[0].shape[0])

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[Matrix3]:
        """Fit a minimal sample: every model the minimal solver allows."""
        if self._minimal is None:
            return self.fit_nonminimal(sample)
        return self._minimal(self._data[0][sample], self._data[1][sample])

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[Matrix3]:
        """Fit a subset; no model if it does not determine one."""
        found = self._solver(self._data[0][subset], self._data[1][subset])
        return [] if found is None else [found]

    def residuals(self, model: Matrix3) -> NDArray[np.float64]:
        """Return every pair's squared residual."""
        return self._residual(model)


def unique(found: Matrix3 | None, what: str) -> Matrix3:
    """Return a solver's matrix, or raise as the C++ core does when the data determine none.

    Parameters
    ----------
    found : ndarray of float64, shape (3, 3), or None
        A solver's result; None when the correspondences do not determine a unique matrix.
    what : str
        The matrix's name ("E", "F", "H"), for the message.

    Returns
    -------
    ndarray of float64, shape (3, 3)
        `found`.

    Raises
    ------
    slam_it.foundations.errors.DegenerateError
        If `found` is None.
    """
    if found is None:
        raise DegenerateError(f"{what}: the correspondences do not determine a unique solution")
    return found
