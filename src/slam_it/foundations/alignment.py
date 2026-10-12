"""Point-set alignment (Umeyama): the rigid or similarity transform between two point sets.

Given corresponding points ``x_i`` (source) and ``y_i`` (target), find the rotation R,
translation t and, optionally, scale s minimising ``Σ w_i |y_i - (s R x_i + t)|²``
(Umeyama, "Least-squares estimation of transformation parameters between two point
patterns", TPAMI 1991). This is the closed-form SE(3)/Sim(3) alignment behind trajectory
error (ATE) and behind comparing a reconstruction with ground truth "up to gauge".

The algorithm: centre both sets on their weighted means; take the SVD of their weighted
cross-covariance ``Σ = U D Vᵀ``; ``R = U S Vᵀ`` with ``S = diag(1, 1, ±1)`` choosing a
proper rotation; ``s = tr(D S) / σ_x²``; ``t = μ_y - s R μ_x``.

Degenerate inputs (fewer than three points, all points collinear or coincident) do not
determine the rotation, and raise :class:`slam_it.foundations.errors.DegenerateError`
rather than returning an arbitrary answer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations.arrays import as_batch, as_real
from slam_it.foundations.errors import DegenerateError, InvalidInputError

DEGENERACY_RATIO = 1e-10
"""The cross-covariance's second singular value must exceed this fraction of its first;
below it the points are (numerically) collinear and the rotation is undetermined."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.alignment`` submodule provides."""

    def umeyama(
        self,
        source: NDArray[np.float64],
        target: NDArray[np.float64],
        weights: NDArray[np.float64],
        with_scale: bool,
    ) -> NDArray[np.float64]:
        """Align (N, 3) points; return the (4, 4) matrix ``[[s R, t], [0, 1]]``."""
        ...


@dataclass(frozen=True)
class Alignment:
    """A similarity transform ``y = s R x + t``; ``scale == 1`` for a rigid alignment.

    Attributes
    ----------
    rotation : ndarray of float64, shape (3, 3)
        The rotation R.
    translation : ndarray of float64, shape (3,)
        The translation t.
    scale : float
        The scale s (exactly 1.0 when scale was not estimated).
    """

    rotation: NDArray[np.float64]
    translation: NDArray[np.float64]
    scale: float

    def as_matrix(self) -> NDArray[np.float64]:
        """Return the 4x4 matrix ``[[s R, t], [0, 1]]``.

        Returns
        -------
        ndarray of float64, shape (4, 4)
            The similarity transform.
        """
        out = np.eye(4)
        out[:3, :3] = self.scale * self.rotation
        out[:3, 3] = self.translation
        return out

    def apply(self, points: ArrayLike) -> NDArray[np.float64]:
        """Map source points into the target frame: ``s R x + t``.

        Parameters
        ----------
        points : array_like, shape (..., 3)
            Points in the source frame.

        Returns
        -------
        ndarray of float64, shape (..., 3)
            The points in the target frame.
        """
        p = as_real(points, "points")
        return np.asarray(self.scale * p @ self.rotation.T + self.translation, dtype=np.float64)


def _umeyama(
    source: NDArray[np.float64],
    target: NDArray[np.float64],
    weights: NDArray[np.float64],
    with_scale: bool,
) -> NDArray[np.float64]:
    """Compute the Umeyama alignment as a 4x4 similarity matrix (reference)."""
    w = weights / weights.sum()
    mu_x = w @ source
    mu_y = w @ target
    xc = source - mu_x
    yc = target - mu_y
    cov = (yc * w[:, None]).T @ xc
    u, d, vt = np.linalg.svd(cov)
    if d[1] <= DEGENERACY_RATIO * d[0]:
        raise DegenerateError("umeyama: points are collinear or coincident")
    sign = np.diag([1.0, 1.0, np.sign(np.linalg.det(u) * np.linalg.det(vt))])
    rotation = u @ sign @ vt
    scale = (
        float(np.trace(np.diag(d) @ sign) / (w @ np.sum(xc * xc, axis=1))) if with_scale else 1.0
    )
    out = np.eye(4)
    out[:3, :3] = scale * rotation
    out[:3, 3] = mu_y - scale * rotation @ mu_x
    return out


def align(
    source: ArrayLike,
    target: ArrayLike,
    *,
    weights: ArrayLike | None = None,
    with_scale: bool = False,
    backend: str = "python",
) -> Alignment:
    """Find the rigid (or similarity) transform that best maps `source` onto `target`.

    Parameters
    ----------
    source, target : array_like, shape (N, 3)
        Corresponding points, N >= 3.
    weights : array_like, shape (N,), optional
        Non-negative weights (default: all equal). At least three must be positive.
    with_scale : bool, optional
        Estimate a scale too (Sim(3)), e.g. for monocular reconstructions (default: rigid).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Alignment
        ``R``, ``t`` and ``s`` with ``target ≈ s R source + t``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the inputs are not finite real (N, 3) arrays of equal length, or the weights are
        negative, of the wrong length, or fewer than three are positive.
    slam_it.foundations.errors.DegenerateError
        If the points are collinear or coincident, so the rotation is undetermined.
    """
    src, lead = as_batch(source, (3,), "source")
    tgt, lead_t = as_batch(target, (3,), "target")
    if len(lead) != 1 or lead != lead_t:
        raise InvalidInputError(
            f"source and target: expected equal (N, 3) shapes, got {lead} vs {lead_t}"
        )
    w = np.ones(src.shape[0]) if weights is None else as_real(weights, "weights")
    if w.shape != (src.shape[0],) or np.any(w < 0.0):
        raise InvalidInputError("weights: expected N non-negative values")
    if np.count_nonzero(w) < 3:
        raise DegenerateError("umeyama: fewer than three points with positive weight")
    matrix = backends.dispatch(backend, __name__, "umeyama", _umeyama, src, tgt, w, with_scale)
    scale = float(np.cbrt(np.linalg.det(matrix[:3, :3]))) if with_scale else 1.0
    return Alignment(rotation=matrix[:3, :3] / scale, translation=matrix[:3, 3].copy(), scale=scale)
