"""Sim(3) RANSAC: the similarity between two keyframes' matched points (ORB-SLAM's ``Sim3Solver``).

Loop closing asks which similarity ``S_12`` carries the points a candidate keyframe (camera 2)
sees onto the matching points the new keyframe (camera 1) sees (Mur-Artal and Tardós, T-RO
2017, section VI; Campos et al., T-RO 2021, section VII). Each match is a pair of 3D points,
``p1`` in camera 1's frame and ``p2`` in camera 2's, and ``p1 = S_12 p2 = s R p2 + t`` for a
true match. A monocular map drifts in scale, so ``s`` is estimated; a stereo or RGB-D map
measures its scale, and ``s`` is held at 1 (ORB-SLAM's ``mbFixScale``).

**The model** of a sample of three matches is the least-squares similarity
:func:`slam_it.foundations.alignment.align` gives (Umeyama's), with its scale or rigid: the same
answer as Horn's quaternion method, which ORB-SLAM's ``ComputeSim3`` uses, with Umeyama's guard
against a reflection. A larger set is fitted the same way, so the local optimisation's refits
are least-squares alignments of the inliers.

**A match is an inlier** when each point, carried into the other camera, projects near where
its partner projects: ``|π(S_12 p2) - π(p1)|² < χ² σ1²`` in image 1 and
``|π(S_12⁻¹ p1) - π(p2)|² < χ² σ2²`` in image 2, with ``σ²`` each keypoint's level variance and
``χ²`` the caller's bound (ORB-SLAM: 9.21, the 99% bound with two degrees of freedom). As in
``CheckInliers``, the projections of the points themselves stand for the observations, so a
match whose point camera 2 did not observe (one found in a neighbour of keyframe 2) is tested
the same way. Here a datum's squared residual is the larger of the two ratios
``|e1|² / σ1²`` and ``|e2|² / σ2²``, with RANSAC's threshold ``sqrt(χ²)``: a match is under it
exactly when it passes both tests. A point carried behind the other camera has an infinite
residual, never an inlier (ORB-SLAM projects it regardless).

**The loop** is rung 0's LO-MSAC (:mod:`slam_it.foundations.ransac`), run to its end and its
best model kept; ORB-SLAM's ``iterate`` returns the first sample with more than its minimum of
inliers. The caller sets the sample count, as ORB-SLAM's ``SetRansacParameters`` derives it.
The consensus is not refined here: loop closing refines it against the keypoints' pixels on
GTSAM (:func:`slam_it.foundations.graphs.refine_similarity`, ``OptimizeSim3``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it.foundations import alignment, camera, ransac
from slam_it.foundations.arrays import as_batch, as_real
from slam_it.foundations.errors import DegenerateError, InvalidInputError
from slam_it.foundations.solve import Solution

Array = NDArray[np.float64]

SAMPLE_SIZE = 3
"""A minimal sample: three matches determine a similarity (or a rigid motion)."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.sim3`` submodule provides.

    ``sim3_ransac`` takes the camera model by its code (``camera.MODELS``' index) and
    :class:`slam_it.foundations.ransac.Options`' fields as one dict, and returns a dict of
    ``model`` (4, 4), ``inliers`` (N,) and ``summary`` (as
    :func:`slam_it.foundations.ransac.summary_from_backend` reads it).
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def sim3_ransac(  # noqa: PLR0913 -- the matches, their camera, the settings
        self,
        *,
        points1: NDArray[np.float64],
        points2: NDArray[np.float64],
        sigma2_1: NDArray[np.float64],
        sigma2_2: NDArray[np.float64],
        model: int,
        intrinsics: NDArray[np.float64],
        fixed_scale: bool,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Estimate the similarity between two cameras' matched points robustly."""
        ...


def inverse(similarity: Array) -> Array:
    """Return the inverse of ``[[s R, t], [0, 1]]``: ``[[Rᵀ / s, -Rᵀ t / s], [0, 1]]``.

    Parameters
    ----------
    similarity : ndarray of float64, shape (4, 4)
        The similarity, its scale positive.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        Its inverse.
    """
    block = similarity[:3, :3]
    scale2 = float(np.sum(block[:, 0] ** 2))  # a column of s R has length s
    out = np.eye(4)
    out[:3, :3] = block.T / scale2
    out[:3, 3] = -out[:3, :3] @ similarity[:3, 3]
    return out


def _carried(similarity: Array, points: Array) -> Array:
    """Return ``s R p + t`` for each point."""
    return np.asarray(points @ similarity[:3, :3].T + similarity[:3, 3], dtype=np.float64)


@dataclass(frozen=True)
class SimilarityEstimator:
    """Umeyama's alignment, tested by reprojection in both images: a :class:`ransac.Estimator`.

    Attributes
    ----------
    points1, points2 : ndarray of float64, shape (N, 3)
        Each match's point in camera 1's frame and in camera 2's, in front of each.
    image1, image2 : ndarray of float64, shape (N, 2)
        Their projections into their own images.
    sigma2_1, sigma2_2 : ndarray of float64, shape (N,)
        Each match's variance in image 1 and in image 2, px².
    model : str
        The camera model, the same for both images.
    intrinsics : ndarray of float64, shape (K,)
        Its intrinsics.
    fixed_scale : bool
        Fit rigid motions (``s = 1``).
    """

    points1: Array
    points2: Array
    image1: Array
    image2: Array
    sigma2_1: Array
    sigma2_2: Array
    model: str
    intrinsics: Array
    fixed_scale: bool

    @classmethod
    def create(  # noqa: PLR0913 -- the matches, their camera
        cls,
        points1: Array,
        points2: Array,
        sigma2_1: Array,
        sigma2_2: Array,
        *,
        model: str,
        intrinsics: Array,
        fixed_scale: bool,
    ) -> SimilarityEstimator:
        """Return the estimator for checked matches, each point projected into its own image.

        Parameters
        ----------
        points1, points2 : ndarray of float64, shape (N, 3)
            The matches' points, in front of their cameras.
        sigma2_1, sigma2_2 : ndarray of float64, shape (N,)
            Their variances.
        model : str
            The camera model.
        intrinsics : ndarray of float64, shape (K,)
            Its intrinsics.
        fixed_scale : bool
            Fit rigid motions.

        Returns
        -------
        SimilarityEstimator
            The estimator.
        """
        return cls(
            points1=points1,
            points2=points2,
            image1=camera.project(model, intrinsics, points1),
            image2=camera.project(model, intrinsics, points2),
            sigma2_1=sigma2_1,
            sigma2_2=sigma2_2,
            model=model,
            intrinsics=intrinsics,
            fixed_scale=fixed_scale,
        )

    @property
    def sample_size(self) -> int:
        """Three matches."""
        return SAMPLE_SIZE

    @property
    def size(self) -> int:
        """The number of matches."""
        return len(self.points1)

    def _fit(self, rows: NDArray[np.int64]) -> list[Array]:
        """Return the least-squares similarity carrying ``points2[rows]`` onto ``points1[rows]``."""
        try:
            found = alignment.align(
                self.points2[rows], self.points1[rows], with_scale=not self.fixed_scale
            )
        except DegenerateError:
            return []
        return [found.as_matrix()]

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[Array]:
        """Return the similarity of three matches.

        Parameters
        ----------
        sample : ndarray of int64, shape (3,)
            The matches.

        Returns
        -------
        list of ndarray of float64, shape (4, 4)
            ``S_12``, or none for collinear or coincident points.
        """
        return self._fit(sample)

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[Array]:
        """Return the least-squares similarity of three or more matches.

        Parameters
        ----------
        subset : ndarray of int64, shape (M,)
            The matches.

        Returns
        -------
        list of ndarray of float64, shape (4, 4)
            ``S_12``, or none for collinear or coincident points.
        """
        return self._fit(subset)

    def residuals(self, model: Array) -> Array:
        """Return each match's squared residual: its larger error over its variance.

        Parameters
        ----------
        model : ndarray of float64, shape (4, 4)
            ``S_12``.

        Returns
        -------
        ndarray of float64, shape (N,)
            ``max(|e1|² / σ1², |e2|² / σ2²)``; infinity where a point is carried behind the
            other camera.
        """
        into1 = _carried(model, self.points2)
        into2 = _carried(inverse(model), self.points1)
        front = (into1[:, 2] > camera.MIN_DEPTH) & (into2[:, 2] > camera.MIN_DEPTH)
        out = np.full(self.size, np.inf)
        if not np.any(front):
            return out
        error1 = camera.project(self.model, self.intrinsics, into1[front]) - self.image1[front]
        error2 = camera.project(self.model, self.intrinsics, into2[front]) - self.image2[front]
        out[front] = np.maximum(
            np.sum(error1**2, axis=1) / self.sigma2_1[front],
            np.sum(error2**2, axis=1) / self.sigma2_2[front],
        )
        return out


def _variances(value: ArrayLike, count: int, name: str) -> Array:
    """Check one variance per match: finite and positive."""
    arr = as_real(value, name)
    if arr.shape != (count,) or np.any(arr <= 0.0):
        raise InvalidInputError(f"{name}: expected {count} positive variances")
    return np.ascontiguousarray(arr)


def sim3_ransac(  # noqa: PLR0913 -- the matches, the settings, their camera
    points1: ArrayLike,
    points2: ArrayLike,
    sigma2_1: ArrayLike,
    sigma2_2: ArrayLike,
    options: ransac.Options,
    *,
    model: str,
    intrinsics: ArrayLike,
    fixed_scale: bool,
    backend: str = "python",
) -> Solution[ransac.Consensus[Array], ransac.Summary]:
    """Estimate ``S_12`` from two cameras' matched points robustly (the module docstring).

    Parameters
    ----------
    points1, points2 : array_like, shape (N, 3)
        Each match's point in camera 1's frame and in camera 2's, in front of each
        (``Z > camera.MIN_DEPTH``), N >= 3.
    sigma2_1, sigma2_2 : array_like, shape (N,)
        Each match's variance in image 1 and in image 2, px², positive (ORB-SLAM: its
        keypoint's level variance).
    options : slam_it.foundations.ransac.Options
        The settings; ``threshold`` is the square root of the chi-square bound (ORB-SLAM:
        ``sqrt(9.21)``).
    model : {"pinhole", "radtan", "radial"}
        The camera model, the same for both images.
    intrinsics : array_like, shape (K,)
        Its intrinsics.
    fixed_scale : bool
        Estimate a rigid motion, the scale held at 1 (stereo and RGB-D).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slam_it.foundations.solve.Solution
        ``S_12`` (``[[s R, t], [0, 1]]``) and its inliers, and the run's
        :class:`~slam_it.foundations.ransac.Summary`.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the points are not finite (N, 3) arrays of equal length, N < 3, a point is not in
        front of its camera, a variance is not positive, or the camera is invalid.
    slam_it.foundations.errors.DegenerateError
        If no sample gave a similarity (every sample collinear).
    """
    one, lead = as_batch(points1, (3,), "points1")
    two, lead2 = as_batch(points2, (3,), "points2")
    if len(lead) != 1 or lead != lead2 or lead[0] < SAMPLE_SIZE:
        raise InvalidInputError(
            f"points1, points2: expected equal (N, 3) arrays, N >= {SAMPLE_SIZE}"
        )
    if np.any(one[:, 2] <= camera.MIN_DEPTH) or np.any(two[:, 2] <= camera.MIN_DEPTH):
        raise InvalidInputError("points1, points2: every point must be in front of its camera")
    first = _variances(sigma2_1, lead[0], "sigma2_1")
    second = _variances(sigma2_2, lead[0], "sigma2_2")
    code, params = camera.as_model(model, intrinsics)
    one, two = np.ascontiguousarray(one), np.ascontiguousarray(two)
    return ransac.run(
        lambda: ransac.estimate(
            SimilarityEstimator.create(
                one, two, first, second, model=model, intrinsics=params, fixed_scale=fixed_scale
            ),
            options,
        ),
        {
            "points1": one,
            "points2": two,
            "sigma2_1": first,
            "sigma2_2": second,
            "model": code,
            "intrinsics": params,
            "fixed_scale": fixed_scale,
        },
        options,
        summary=ransac.summary_from_backend,
        backend=backend,
        module_name=__name__,
        function="sim3_ransac",
    )
