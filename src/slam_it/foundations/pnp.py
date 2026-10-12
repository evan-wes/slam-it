"""Absolute pose (PnP): the camera pose from world points and their bearings, robustly.

Given N world points ``Xᵢ`` and the unit bearings ``yᵢ`` along which a calibrated camera sees
them, find ``T_cam_world = [R | t]``. This is how a tracker places each new frame against the
map it has built, and how a relocaliser recovers. Three correspondences are the minimum
(:mod:`slam_it.foundations.p3p`); this module adds the other three pieces of a robust
estimate, in the pattern :mod:`slam_it.foundations.epipolar` set for the essential matrix.

**The reprojection error.** A pose's residual for one correspondence is in the **normalised
image plane**: the point's projection ``π(R X + t)``, with ``π(p) = (p_x / p_z, p_y / p_z)``,
minus the bearing's ``(y_x / y_z, y_y / y_z)``. A pixel threshold ``θ`` becomes ``θ / focal``
there, as for E's Sampson error, so the bearings need positive z (pinhole views). A point
behind the camera (``p_z <= 0``) has an infinite residual: it is never an inlier.

**EPnP** (Lepetit, Moreno-Noguer and Fua, "EPnP: An Accurate O(n) Solution to the PnP
Problem", IJCV 81(2), 2009) is the least-squares solver, for five or more points
(:data:`EPNP_MINIMUM`). It writes
every world point as a weighted sum of four **control points**, ``Xᵢ = Σⱼ αᵢⱼ cⱼ`` with
``Σⱼ αᵢⱼ = 1`` (the centroid and the three principal axes, scaled by the spread along each), so
the unknowns are the control points in the camera: 12 numbers. Each correspondence gives two
equations linear in them, ``M x = 0`` (M is 2N x 12). In the noise-free case x lies in the
null space of ``MᵀM``, spanned by its eigenvectors of the smallest eigenvalues, ``vₖ``:
``x = Σₖ βₖ vₖ``. The β follow from requiring the control points' six pairwise distances to be
the same in the camera as in the world, quadratic equations in β, linearised in three ways
(one, two or three of the ``vₖ``; Lepetit et al.'s N = 1 case is the first β of N = 4) and
each polished by Gauss-Newton on the six distances (:data:`POLISH_MAX_STEPS`). Each gives the
camera points ``Σⱼ αᵢⱼ cⱼ`` (their sign chosen so they lie in front), and the pose aligning the
world points to them (:func:`slam_it.foundations.p3p.rigid_pose`); the one with the
lowest reprojection error wins. It follows OpenCV's implementation of the paper.

EPnP needs the points to span three dimensions: on a plane, the third control point has no
spread and ``Σⱼ αᵢⱼ cⱼ`` does not determine it, so planar points raise
:class:`~slam_it.foundations.errors.DegenerateError` (:data:`PLANAR_TOLERANCE`), as OpenCV's
does. IPPE (Collins and Bartoli, IJCV 2014) is the planar solver. On a planar scene PnP
RANSAC still works: P3P samples it, and with LO's EPnP refits unable to run, the refinement's
refine-and-reselect rounds (:func:`slam_it.foundations.ransac.refine_consensus`) bring the
pose to the noise floor (median 1.2 times it, against 2.0 with one refinement).

**The refinement.** :func:`refine_pose` minimises the inliers' robust reprojection error,
``½ Σ ρ(wᵢ |rᵢ|²)`` with ρ Cauchy's loss and ``wᵢ`` each correspondence's weight (its
information, as ORB-SLAM's ``PoseOptimization`` weights a keypoint by its pyramid level; 1 by
default), by Levenberg-Marquardt (:mod:`slam_it.foundations.nls`) on SE(3): the step
``T ← T exp(δ)`` (right perturbation, ``(ρ, θ)`` order), with the analytic Jacobian
``∂π/∂p · [R | -R [X]ₓ]``. Each residual and its Jacobian are scaled by ``√wᵢ``, so the loss
acts on whitened residuals and its scale is in whitened units.

**RANSAC.** :func:`pnp_ransac` runs LO-MSAC (:mod:`slam_it.foundations.ransac`) with a P3P
minimal solver and EPnP for the least-squares refits, then refines the consensus and keeps the
result only if it lowers the MSAC cost (:func:`slam_it.foundations.ransac.refine_consensus`),
as E-RANSAC does.

The C++ core (``slam_it/foundations/pnp.hpp``) implements every function.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations import nls, p3p, ransac, se3, so3
from slam_it.foundations.arrays import as_correspondences, as_weights
from slam_it.foundations.correspondences import normalised_xy
from slam_it.foundations.errors import DegenerateError, InvalidInputError
from slam_it.foundations.solve import Solution

EPNP_MINIMUM = 5
"""Correspondences EPnP takes. With four, ``MᵀM``'s null space is four-dimensional and the
linearisations are approximate: on exact four-point samples, the exact pose came back on 38%
(OpenCV's EPnP: 12%); from five on, on every one."""

PLANAR_TOLERANCE = 1e-12
"""The points are planar for EPnP when their covariance's smallest eigenvalue is below this
fraction of its largest."""

POLISH_MAX_STEPS = 10
"""At most this many Gauss-Newton steps polish EPnP's β on the six control-point distances."""
POLISH_TOLERANCE = 1e-14
"""Polishing stops when a step is below this, relative to β."""

REFINE_ITERATIONS = 20
"""Levenberg-Marquardt iterations of the pose refinement (it starts near the optimum)."""

_PAIRS = ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3))
"""The control points' six pairs, in the rows of EPnP's distance system."""

_MONOMIALS = ((0, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (0, 3), (1, 3), (2, 3), (3, 3))
"""The ten products ``βₖ βₗ``, in the columns of EPnP's distance system (OpenCV's order)."""

_APPROXIMATIONS = ((0, 1, 3, 6), (0, 1, 2), (0, 1, 2, 3, 4))
"""The columns each linearisation keeps: β₁ with β₁…β₄ (N = 4); β₁, β₂ (N = 2); β₁…β₃ (N = 3)."""

Pose = NDArray[np.float64]


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.pnp`` submodule provides.

    ``pnp_ransac`` takes :class:`slam_it.foundations.ransac.Options`' fields as one dict and
    returns a dict: ``model`` (4, 4), ``inliers`` (N,) bool and ``summary`` (see
    :func:`slam_it.foundations.ransac.refined_summary_from_backend`).
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def epnp(
        self, points: NDArray[np.float64], bearings: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Return EPnP's pose ``T_cam_world`` from (N, 3) correspondences, N >= 5."""
        ...

    def refine_pose(
        self,
        points: NDArray[np.float64],
        bearings: NDArray[np.float64],
        pose: NDArray[np.float64],
        scale: float,
        weights: NDArray[np.float64],
    ) -> dict[str, Any]:
        """Refine a pose on (M, 3) weighted correspondences: ``pose`` and its NLS ``summary``."""
        ...

    def pnp_ransac(
        self,
        points: NDArray[np.float64],
        bearings: NDArray[np.float64],
        options: dict[str, Any],
        minimal_solver: int,
        refine: bool,
    ) -> dict[str, Any]:
        """Run LO-MSAC with a P3P solver (an index into ``p3p.SOLVERS``), then refine."""
        ...


# --- the reprojection error ---------------------------------------------------------------


def _reprojection(
    pose: Pose, points: NDArray[np.float64], image: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return squared normalised-plane reprojection errors; infinity behind the camera."""
    in_camera = points @ pose[:3, :3].T + pose[:3, 3]
    depth = in_camera[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        errors = np.sum((in_camera[:, :2] / depth[:, None] - image) ** 2, axis=1)
    return np.asarray(np.where(depth > 0.0, errors, np.inf), dtype=np.float64)


def reprojection_errors(
    pose: ArrayLike, points: ArrayLike, bearings: ArrayLike
) -> NDArray[np.float64]:
    """Return each correspondence's squared reprojection error in the normalised plane.

    Parameters
    ----------
    pose : array_like, shape (4, 4)
        ``T_cam_world``.
    points : array_like, shape (N, 3)
        World points.
    bearings : array_like, shape (N, 3)
        Their unit bearings, positive z.

    Returns
    -------
    ndarray of float64, shape (N,)
        ``|π(R X + t) - (y_x/y_z, y_y/y_z)|²``; infinity for a point behind the camera.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose is not a rigid transform or the correspondences are invalid.
    """
    matrix = _as_pose(pose)
    world, rays = as_correspondences(points, bearings, 0)
    return _reprojection(matrix, world, normalised_xy(rays))


def _as_pose(value: ArrayLike) -> Pose:
    """Validate one rigid (4, 4) transform."""
    matrix = np.ascontiguousarray(np.asarray(value, dtype=np.float64))
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise InvalidInputError(f"pose: expected one finite (4, 4) matrix, got {matrix.shape}")
    se3.check_transform(matrix[None])
    return matrix


# --- EPnP ---------------------------------------------------------------------------------


def _deterministic_sign(vectors: NDArray[np.float64]) -> NDArray[np.float64]:
    """Flip each column so its largest-magnitude entry is positive (the first, on a tie).

    An eigenvector's sign is arbitrary and differs between LAPACK and Eigen; fixing it makes
    every backend build the same control points and null space.
    """
    largest = np.argmax(np.abs(vectors), axis=0)
    signs = np.sign(vectors[largest, np.arange(vectors.shape[1])])
    return np.asarray(vectors * np.where(signs == 0.0, 1.0, signs), dtype=np.float64)


def _control_points(points: NDArray[np.float64]) -> NDArray[np.float64] | None:
    """Return EPnP's four world control points, shape (4, 3); None if the points are planar.

    The centroid, then the centroid plus each principal axis scaled by the points' RMS spread
    along it (largest first).
    """
    centroid = points.mean(axis=0)
    centred = points - centroid
    values, vectors = np.linalg.eigh(centred.T @ centred)
    if not values[0] > PLANAR_TOLERANCE * values[2]:
        return None
    axes = _deterministic_sign(vectors[:, ::-1]) * np.sqrt(values[::-1] / len(points))
    return np.vstack([centroid, centroid + axes.T])


def _distance_system(
    basis: NDArray[np.float64], controls: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return EPnP's ``L`` (6 x 10) and ``ρ`` (6): ``L · (βₖβₗ) = ρ`` keeps the distances.

    `basis` holds the null-space vectors ``vₖ`` as rows of 12 (each four control points).
    """
    vectors = basis.reshape(4, 4, 3)
    rows = []
    for a, b in _PAIRS:
        dv = vectors[:, a] - vectors[:, b]
        rows.append([(1.0 if k == m else 2.0) * float(dv[k] @ dv[m]) for k, m in _MONOMIALS])
    rho = np.array([float(np.sum((controls[a] - controls[b]) ** 2)) for a, b in _PAIRS])
    return np.array(rows), rho


def _initial_betas(
    system: NDArray[np.float64], rho: NDArray[np.float64], kind: int
) -> NDArray[np.float64]:
    """Linearise the distance equations in one of three ways (OpenCV's approximations).

    ``kind`` 0 keeps ``β₁βₖ`` (k = 1..4); 1 keeps ``β₁², β₁β₂, β₂²``; 2 also ``β₁β₃, β₂β₃``. The
    least-squares products are factored back into β, with the signs OpenCV chooses.
    """
    products = np.linalg.lstsq(system[:, list(_APPROXIMATIONS[kind])], rho, rcond=None)[0]
    betas = np.zeros(4)
    if kind == 0:
        first = math.sqrt(abs(products[0]))
        sign = -1.0 if products[0] < 0.0 else 1.0
        betas[0] = first
        betas[1:] = sign * products[1:] / first if first > 0.0 else 0.0
        return betas
    if products[0] < 0.0:
        betas[0] = math.sqrt(-products[0])
        betas[1] = math.sqrt(-products[2]) if products[2] < 0.0 else 0.0
    else:
        betas[0] = math.sqrt(products[0])
        betas[1] = math.sqrt(products[2]) if products[2] > 0.0 else 0.0
    if products[1] < 0.0:
        betas[0] = -betas[0]
    if kind == 2 and betas[0] != 0.0:
        betas[2] = products[3] / betas[0]
    return betas


def _monomials(betas: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the ten products ``βₖ βₗ`` in :data:`_MONOMIALS` order."""
    return np.array([betas[k] * betas[m] for k, m in _MONOMIALS])


def _polish_betas(
    betas: NDArray[np.float64], system: NDArray[np.float64], rho: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Polish β by Gauss-Newton on ``L · (βₖβₗ) = ρ``; a step is kept only if it helps."""
    residual = rho - system @ _monomials(betas)
    for _ in range(POLISH_MAX_STEPS):
        jacobian = np.zeros((len(_PAIRS), 4))
        for column, (k, m) in enumerate(_MONOMIALS):
            jacobian[:, k] += system[:, column] * betas[m]
            jacobian[:, m] += system[:, column] * betas[k]
        step = np.linalg.lstsq(jacobian, residual, rcond=None)[0]
        moved = betas + step
        moved_residual = rho - system @ _monomials(moved)
        if not np.linalg.norm(moved_residual) < np.linalg.norm(residual):
            break
        betas, residual = moved, moved_residual
        if np.linalg.norm(step) <= POLISH_TOLERANCE * np.linalg.norm(betas):
            break
    return betas


def _epnp(points: NDArray[np.float64], bearings: NDArray[np.float64]) -> Pose | None:
    """Solve PnP by EPnP (reference, unchecked); None if the points are planar or degenerate."""
    controls = _control_points(points)
    if controls is None:
        return None
    alphas = np.column_stack([np.ones(len(points)), points]) @ np.linalg.inv(
        np.column_stack([np.ones(4), controls])
    )
    image = normalised_xy(bearings)
    system = np.zeros((2 * len(points), 12))
    for j in range(4):
        system[0::2, 3 * j] = alphas[:, j]
        system[0::2, 3 * j + 2] = -alphas[:, j] * image[:, 0]
        system[1::2, 3 * j + 1] = alphas[:, j]
        system[1::2, 3 * j + 2] = -alphas[:, j] * image[:, 1]
    _, vectors = np.linalg.eigh(system.T @ system)
    basis = _deterministic_sign(vectors[:, :4]).T
    distances, rho = _distance_system(basis, controls)
    best: tuple[float, Pose] | None = None
    for kind in range(len(_APPROXIMATIONS)):
        betas = _polish_betas(_initial_betas(distances, rho, kind), distances, rho)
        camera = alphas @ (betas @ basis).reshape(4, 3)
        if np.sum(camera[:, 2]) < 0.0:
            camera = -camera
        pose = p3p.rigid_pose(points, camera)
        if pose is None:
            continue
        error = float(np.sum(_reprojection(pose, points, image)))
        if best is None or error < best[0]:
            best = (error, pose)
    return None if best is None else best[1]


def epnp(points: ArrayLike, bearings: ArrayLike, *, backend: str = "python") -> Pose:
    """Estimate the camera pose from five or more correspondences by EPnP.

    Parameters
    ----------
    points : array_like, shape (N, 3)
        World points, N >= 5, spanning three dimensions.
    bearings : array_like, shape (N, 3)
        Their unit bearings, positive z.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``T_cam_world``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the correspondences are invalid or fewer than five.
    slam_it.foundations.errors.DegenerateError
        If the points are planar (or collinear): EPnP's control points are undetermined.
    """
    world, rays = as_correspondences(points, bearings, EPNP_MINIMUM)

    def reference(w: NDArray[np.float64], r: NDArray[np.float64]) -> NDArray[np.float64]:
        pose = _epnp(w, r)
        if pose is None:
            raise DegenerateError("epnp: the points are planar, the control points undetermined")
        return pose

    return backends.dispatch(backend, __name__, "epnp", reference, world, rays)


# --- the refinement -----------------------------------------------------------------------


class ReprojectionProblem:
    """Least squares on the reprojection errors of correspondences, over SE(3).

    One residual block of two per correspondence: ``π(R X + t) - x`` in the normalised plane;
    a point behind the camera has an infinite squared norm, as in RANSAC's scoring, since its
    projection ``x / z`` with both negative can land near the observation.
    The state is ``T_cam_world``; a step is ``T exp(δ)`` with ``δ = (ρ, θ)``, so the camera
    point moves by ``R (ρ - [X]ₓ θ)`` to first order and the Jacobian is
    ``∂π/∂p · [R | -R [X]ₓ]``, ``∂π/∂p = [[1/z, 0, -x/z²], [0, 1/z, -y/z²]]``.

    Parameters
    ----------
    points : ndarray of float64, shape (M, 3)
        The inliers' world points.
    image : ndarray of float64, shape (M, 2)
        Their bearings' normalised image points.
    weights : ndarray of float64, shape (M,), optional
        Each block's weight, positive: its residual and Jacobian are scaled by the weight's
        square root (default: all 1).
    """

    def __init__(
        self,
        points: NDArray[np.float64],
        image: NDArray[np.float64],
        weights: NDArray[np.float64] | None = None,
    ) -> None:
        self._points, self._image = points, image
        self._weights = np.ones(len(points)) if weights is None else weights
        self._root = np.sqrt(self._weights)

    def _projection(self, state: Pose) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return the camera points and the residuals."""
        in_camera = self._points @ state[:3, :3].T + state[:3, 3]
        return in_camera, in_camera[:, :2] / in_camera[:, 2:3] - self._image

    def squared_norms(self, state: Pose) -> NDArray[np.float64]:
        """Return each block's squared residual.

        Parameters
        ----------
        state : ndarray of float64, shape (4, 4)
            The pose.

        Returns
        -------
        ndarray of float64, shape (M,)
            The squared residuals; infinity for a point behind the camera, so a step that
            moves one there has an infinite cost and is rejected (cheirality).
        """
        return _reprojection(state, self._points, self._image) * self._weights

    def linearize(self, state: Pose, weights: NDArray[np.float64]) -> nls.DenseSystem:
        """Linearise at `state` into weighted normal equations over ``(ρ, θ)``.

        Parameters
        ----------
        state : ndarray of float64, shape (4, 4)
            The pose.
        weights : ndarray of float64, shape (M,)
            The blocks' weights.

        Returns
        -------
        slam_it.foundations.nls.DenseSystem
            The normal equations, six parameters.
        """
        in_camera, residuals = self._projection(state)
        x, y, z = in_camera[:, 0], in_camera[:, 1], in_camera[:, 2]
        projection = np.zeros((len(z), 2, 3))
        projection[:, 0, 0] = projection[:, 1, 1] = 1.0 / z
        projection[:, 0, 2] = -x / z**2
        projection[:, 1, 2] = -y / z**2
        rotation = state[:3, :3]
        motion = np.concatenate(
            [np.broadcast_to(rotation, (len(z), 3, 3)), -rotation @ so3.hat(self._points)], axis=2
        )
        return nls.DenseSystem.build(
            (projection @ motion) * self._root[:, None, None],
            residuals * self._root[:, None],
            np.asarray(weights, dtype=np.float64),
        )

    @staticmethod
    def retract(state: Pose, delta: NDArray[np.float64]) -> Pose:
        """Step on SE(3): ``T exp(δ)``.

        Parameters
        ----------
        state : ndarray of float64, shape (4, 4)
            The pose.
        delta : ndarray of float64, shape (6,)
            ``(ρ, θ)``.

        Returns
        -------
        ndarray of float64, shape (4, 4)
            The moved pose.
        """
        return np.asarray(state @ se3.exp(delta), dtype=np.float64)


def refine_reference(
    pose: Pose,
    points: NDArray[np.float64],
    bearings: NDArray[np.float64],
    scale: float,
    weights: NDArray[np.float64] | None = None,
) -> Solution[Pose, nls.Summary]:
    """Refine a pose on validated correspondences by Levenberg-Marquardt (the reference).

    Unchecked: the caller has validated everything :func:`refine_pose` checks.

    Parameters
    ----------
    pose : ndarray of float64, shape (4, 4)
        The starting ``T_cam_world``, every point in front of the camera.
    points : ndarray of float64, shape (M, 3)
        World points.
    bearings : ndarray of float64, shape (M, 3)
        Their unit bearings, positive z.
    scale : float
        The Cauchy loss's scale, a whitened reprojection error in the normalised plane
        (positive).
    weights : ndarray of float64, shape (M,), optional
        Each correspondence's weight, positive (default: all 1).

    Returns
    -------
    slam_it.foundations.solve.Solution
        The refined pose and the solve's summary.
    """
    problem = ReprojectionProblem(points, normalised_xy(bearings), weights)
    options = nls.Options(loss=nls.Loss("cauchy", scale), max_iterations=REFINE_ITERATIONS)
    return nls.solve(problem, pose, options)


def refine_pose(  # noqa: PLR0913 -- the correspondences, their weights, the loss, the backend
    pose: ArrayLike,
    points: ArrayLike,
    bearings: ArrayLike,
    *,
    scale: float,
    weights: ArrayLike | None = None,
    backend: str = "python",
) -> Solution[Pose, nls.Summary]:
    """Refine a pose by minimising the inliers' robust reprojection error over SE(3).

    Parameters
    ----------
    pose : array_like, shape (4, 4)
        The starting ``T_cam_world``, every point in front of the camera.
    points : array_like, shape (M, 3)
        The inliers' world points, M >= 3.
    bearings : array_like, shape (M, 3)
        Their unit bearings, positive z.
    scale : float
        The Cauchy loss's scale, a whitened reprojection error in the normalised plane
        (positive).
    weights : array_like, shape (M,), optional
        Each correspondence's weight, its information in the units of `scale` (default: all
        1); positive and finite. Not relative: a common factor k on the weights acts as
        ``1/√k`` on the scale.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slam_it.foundations.solve.Solution
        The refined pose and the solve's summary.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose or the correspondences are invalid, M < 3, `scale` is not positive, a
        weight is not positive and finite, or a point is behind the starting camera.
    """
    matrix = _as_pose(pose)
    world, rays = as_correspondences(points, bearings, p3p.SAMPLE_SIZE)
    if not (np.isfinite(scale) and scale > 0.0):
        raise InvalidInputError(f"scale: must be positive and finite, got {scale}")
    w = np.ones(len(world)) if weights is None else as_weights(weights, (len(world),), "weights")
    if not np.all(w > 0.0):
        raise InvalidInputError(
            f"weights: expected positive values, one per correspondence, got {w.min()}"
        )
    if np.any(world @ matrix[2, :3] + matrix[2, 3] <= 0.0):
        raise InvalidInputError("pose: a point is behind the starting camera")
    return backends.call(
        backend,
        __name__,
        "refine_pose",
        reference=lambda: refine_reference(matrix, world, rays, scale, w),
        arguments={"points": world, "bearings": rays, "pose": matrix, "scale": scale, "weights": w},
        decode=lambda raw, codes: Solution(
            state=np.asarray(raw["pose"], dtype=np.float64),
            summary=nls.summary_from_backend(raw["summary"], codes),
        ),
    )


# --- RANSAC -------------------------------------------------------------------------------


@dataclass(frozen=True)
class PoseEstimator:
    """P3P and EPnP with the reprojection error, as a :class:`ransac.Estimator`.

    Attributes
    ----------
    points : ndarray of float64, shape (N, 3)
        World points.
    bearings : ndarray of float64, shape (N, 3)
        Their unit bearings, positive z.
    solver : str
        The P3P solver (:data:`slam_it.foundations.p3p.SOLVERS`).
    """

    points: NDArray[np.float64]
    bearings: NDArray[np.float64]
    solver: str

    @property
    def sample_size(self) -> int:
        """Three correspondences."""
        return p3p.SAMPLE_SIZE

    @property
    def size(self) -> int:
        """The number of correspondences."""
        return len(self.points)

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[Pose]:
        """Return every pose P3P gives for three correspondences.

        Parameters
        ----------
        sample : ndarray of int64, shape (3,)
            The correspondences.

        Returns
        -------
        list of ndarray of float64, shape (4, 4)
            The poses (none for collinear points).
        """
        return p3p.solve_reference(self.points[sample], self.bearings[sample], self.solver)

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[Pose]:
        """Return EPnP's pose for five or more correspondences.

        Parameters
        ----------
        subset : ndarray of int64, shape (M,)
            The correspondences.

        Returns
        -------
        list of ndarray of float64, shape (4, 4)
            The pose, or none for fewer than five or planar points.
        """
        if len(subset) < EPNP_MINIMUM:
            return []
        pose = _epnp(self.points[subset], self.bearings[subset])
        return [] if pose is None else [pose]

    def residuals(self, model: Pose) -> NDArray[np.float64]:
        """Return the squared reprojection errors in the normalised plane.

        Parameters
        ----------
        model : ndarray of float64, shape (4, 4)
            The pose.

        Returns
        -------
        ndarray of float64, shape (N,)
            The errors; infinity behind the camera.
        """
        return _reprojection(model, self.points, normalised_xy(self.bearings))


def pnp_ransac_reference(
    points: NDArray[np.float64],
    bearings: NDArray[np.float64],
    options: ransac.Options,
    minimal_solver: str,
    refine: bool,
) -> Solution[ransac.Consensus[Pose], ransac.RefinedSummary]:
    """Run PnP RANSAC and the refinement (the reference), unchecked.

    Parameters
    ----------
    points : ndarray of float64, shape (N, 3)
        Validated world points, N >= 3.
    bearings : ndarray of float64, shape (N, 3)
        Their unit bearings, positive z.
    options : ransac.Options
        The settings.
    minimal_solver : {"ding", "lambda_twist"}
        The P3P solver.
    refine : bool
        Refine the best pose on its inliers.

    Returns
    -------
    slam_it.foundations.solve.Solution
        The pose, its inliers and the :class:`~slam_it.foundations.ransac.RefinedSummary`.

    Raises
    ------
    slam_it.foundations.errors.DegenerateError
        If no sample gave a pose.
    """

    def refined(model: Pose, inliers: NDArray[np.intp]) -> Solution[Pose, nls.Summary]:
        return refine_reference(model, points[inliers], bearings[inliers], options.threshold)

    return ransac.estimate_refined(
        PoseEstimator(points, bearings, minimal_solver),
        options,
        refined if refine else None,
        minimum=p3p.SAMPLE_SIZE,
    )


def pnp_ransac(  # noqa: PLR0913 -- the solver choices are keyword-only
    points: ArrayLike,
    bearings: ArrayLike,
    options: ransac.Options,
    *,
    minimal_solver: str = "ding",
    refine: bool = True,
    backend: str = "python",
) -> Solution[ransac.Consensus[Pose], ransac.RefinedSummary]:
    """Estimate the camera pose robustly: LO-MSAC, then an optional refinement.

    Parameters
    ----------
    points : array_like, shape (N, 3)
        World points, N >= 3.
    bearings : array_like, shape (N, 3)
        Their unit bearings, positive z.
    options : ransac.Options
        The settings; ``threshold`` is a reprojection error in the normalised plane (a pixel
        threshold divided by the focal length).
    minimal_solver : {"ding", "lambda_twist"}, optional
        The P3P solver (default Ding et al.'s). The least-squares refits inside LO are EPnP.
    refine : bool, optional
        Refine the best pose on its inliers (:func:`refine_pose`, Cauchy scale ``threshold``),
        keeping it only if it lowers the MSAC cost (default: on).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slam_it.foundations.solve.Solution
        ``T_cam_world`` and its inliers, and the
        :class:`~slam_it.foundations.ransac.RefinedSummary`: the RANSAC run (whose
        ``inlier_count`` and ``score`` describe the returned pose) and the refinement's solve.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the correspondences are invalid or fewer than three, or `minimal_solver` is
        unknown.
    slam_it.foundations.errors.DegenerateError
        If no sample gave a pose.
    """
    if minimal_solver not in p3p.SOLVERS:
        raise InvalidInputError(
            f"minimal_solver: expected one of {p3p.SOLVERS}, got {minimal_solver!r}"
        )
    world, rays = as_correspondences(points, bearings, p3p.SAMPLE_SIZE)
    return ransac.run(
        lambda: pnp_ransac_reference(world, rays, options, minimal_solver, refine),
        {
            "points": world,
            "bearings": rays,
            "minimal_solver": p3p.SOLVERS.index(minimal_solver),
            "refine": refine,
        },
        options,
        summary=ransac.refined_summary_from_backend,
        backend=backend,
        module_name=__name__,
        function="pnp_ransac",
    )
