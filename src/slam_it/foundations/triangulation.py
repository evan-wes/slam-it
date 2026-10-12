"""Triangulation: a 3D point from the rays that observe it, two-view optimal or N-view linear.

Every ray is a **bearing vector** ``f`` (unit norm, in its camera's frame), so the methods do
not depend on the camera model. A camera pose is ``T_cam_world = [R | t]``: a world point X
lies on the ray ``f`` when ``R X + t`` is a positive multiple of ``f``.

**Two views, optimal** (:func:`two_view`). With noisy rays, the two never meet. Lindstrom's
``niter2`` ("Triangulation made easy", CVPR 2010) moves both image points the least distance
(in the normalised image plane, ``x = f / f_z``) that makes them satisfy the epipolar
constraint ``x₂ᵀ E x₁ = 0`` exactly, with ``E = [t]ₓ R`` of the relative pose ``T_c2_c1``. Two
Newton-like iterations reach the optimum to machine precision for all practical
configurations, and are non-iterative in practice. The corrected rays then intersect exactly:
from ``d₂ x̂₂ = d₁ R x̂₁ + t``, crossing with ``x̂₂`` gives the depth
``d₁ = -((x̂₂ × t) · (x̂₂ × R x̂₁)) / |x̂₂ × R x̂₁|²``. This is the optimal (maximum-likelihood under
isotropic Gaussian noise in the normalised plane) two-view point, the same one Hartley and
Sturm's polynomial method finds, at a fraction of the cost. It needs ``f_z > 0`` (a pinhole
view); a wide-angle camera would need a bearing-space method instead (Lee and Civera, 2019).

**N views, linear** (:func:`n_view`). Each observing view gives ``[f]ₓ (R X + t) = 0``, three
rows (two independent) linear in the homogeneous point ``X̃``. The stacked system's smallest
right singular vector is the direct linear transform (DLT) estimate (Hartley and Zisserman,
section 12.2, here on bearings). It minimises an algebraic error, not a geometric one, and is
the usual start for bundle adjustment. Before solving, the world is shifted to the observing
cameras' mean centre and scaled by their mean distance from it, so the translation columns are
of the same order as the rotation columns.

**What each point reports.**

- ``in_front``: the point lies ahead along every ray that observes it, ``f · (R X + t) > 0``
  (cheirality).
- ``parallax``: the largest angle at the point between the directions to two observing camera
  centres. Small parallax means a poorly determined depth; callers set their own threshold
  (an initialiser typically wants a degree or more).
- A point whose rays are parallel is NaN, with ``in_front`` false and ``parallax`` 0: the
  corrected rays do not cross, or the DLT's ``X̃`` is at infinity, or its system leaves a
  whole line of solutions (rays along the line through the centres). Nothing is raised:
  such points are counted, not fatal.

The C++ core (``slam_it/foundations/triangulation.hpp``) implements both functions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations import se3, so3
from slam_it.foundations.arrays import as_batch, as_bearing_pairs, as_bearings, as_mask
from slam_it.foundations.errors import InvalidInputError

PARALLEL_SINE = 1e-12
"""Rays meeting at an angle whose sine is below this are parallel, and their point is at
infinity. The DLT uses the same ratio between the homogeneous point's last coordinate and the
rest, and between its system's third and first singular values."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.triangulation`` submodule provides.

    Each returns a (P, 5) array: the point (3 columns), ``in_front`` as 0 or 1, and
    ``parallax`` in radians.
    """

    def two_view(
        self,
        relative_pose: NDArray[np.float64],
        bearings1: NDArray[np.float64],
        bearings2: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Triangulate (P, 3) bearing pairs through the (4, 4) pose ``T_c2_c1``."""
        ...

    def n_view(
        self,
        poses: NDArray[np.float64],
        bearings: NDArray[np.float64],
        observed: NDArray[np.bool_],
    ) -> NDArray[np.float64]:
        """Triangulate (P, V, 3) bearings seen from (V, 4, 4) poses where ``observed``."""
        ...


@dataclass(frozen=True)
class Triangulation:
    """Triangulated points, with their cheirality and parallax.

    Attributes
    ----------
    points : ndarray of float64, shape (P, 3)
        The points, in the world frame of the poses (camera 1's frame for :func:`two_view`);
        NaN where the rays are parallel.
    in_front : ndarray of bool, shape (P,)
        Whether each point lies ahead along every ray that observes it.
    parallax : ndarray of float64, shape (P,)
        The largest angle at each point between two observing camera centres, in radians.
    """

    points: NDArray[np.float64]
    in_front: NDArray[np.bool_]
    parallax: NDArray[np.float64]

    @staticmethod
    def unpack(packed: NDArray[np.float64]) -> Triangulation:
        """Build from a backend's (P, 5) array.

        Parameters
        ----------
        packed : ndarray of float64, shape (P, 5)
            Point, ``in_front`` (0 or 1), parallax.

        Returns
        -------
        Triangulation
            The unpacked result.
        """
        return Triangulation(
            points=np.ascontiguousarray(packed[:, :3]),
            in_front=packed[:, 3] > 0.5,
            parallax=np.ascontiguousarray(packed[:, 4]),
        )


def _angle(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the angle between rows of `a` and `b`, from ``atan2`` (accurate at 0 and π)."""
    return np.asarray(
        np.arctan2(np.linalg.norm(np.cross(a, b), axis=-1), np.sum(a * b, axis=-1)),
        dtype=np.float64,
    )


def triangulate_pairs(
    relative_pose: NDArray[np.float64],
    bearings1: NDArray[np.float64],
    bearings2: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Triangulate bearing pairs with Lindstrom's niter2, unchecked; return (P, 5).

    The reference behind :func:`two_view`: point, ``in_front`` (0 or 1), parallax per row.
    """
    rotation, t = relative_pose[:3, :3], relative_pose[:3, 3]
    essential = so3.hat(t) @ rotation
    e2 = essential[:2, :2]
    x1 = bearings1 / bearings1[:, 2:3]
    x2 = bearings2 / bearings2[:, 2:3]
    # niter2, with the paper's (x, x') = (x₂, x₁): the constraint is x₂ᵀ E x₁ = 0.
    n2 = (x1 @ essential.T)[:, :2]  # ∂c/∂x₂ = E x₁
    n1 = (x2 @ essential)[:, :2]  # ∂c/∂x₁ = Eᵀ x₂
    a = np.einsum("pi,ij,pj->p", n2, e2, n1)
    b = 0.5 * (np.sum(n2 * n2, axis=1) + np.sum(n1 * n1, axis=1))
    c = np.einsum("pi,ij,pj->p", x2, essential, x1)
    d = np.sqrt(np.maximum(b * b - a * c, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        lam = c / (b + d)
        d2 = lam[:, None] * n2
        d1 = lam[:, None] * n1
        n2 = n2 - d1 @ e2.T
        n1 = n1 - d2 @ e2
        lam = lam * 2.0 * d / (np.sum(n2 * n2, axis=1) + np.sum(n1 * n1, axis=1))
    x2c = x2.copy()
    x1c = x1.copy()
    x2c[:, :2] -= lam[:, None] * n2
    x1c[:, :2] -= lam[:, None] * n1
    # Intersect the corrected rays: d₂ x̂₂ = d₁ R x̂₁ + t.
    rotated = x1c @ rotation.T
    across = np.cross(x2c, rotated)
    along = np.cross(x2c, np.broadcast_to(t, x2c.shape))
    denominator = np.sum(across * across, axis=1)
    lengths = np.linalg.norm(x2c, axis=1) * np.linalg.norm(rotated, axis=1)
    parallel = ~(np.sqrt(denominator) > PARALLEL_SINE * lengths)
    with np.errstate(divide="ignore", invalid="ignore"):
        depth1 = -np.sum(along * across, axis=1) / denominator
    points = depth1[:, None] * x1c
    centre2 = -rotation.T @ t
    in_cam2 = points @ rotation.T + t
    in_front = (np.sum(points * bearings1, axis=1) > 0.0) & (
        np.sum(in_cam2 * bearings2, axis=1) > 0.0
    )
    parallax = _angle(points, points - centre2)
    bad = parallel | ~np.all(np.isfinite(points), axis=1)
    return _pack(points, in_front, parallax, bad)


def _pack(
    points: NDArray[np.float64],
    in_front: NDArray[np.bool_],
    parallax: NDArray[np.float64],
    bad: NDArray[np.bool_],
) -> NDArray[np.float64]:
    """Pack points, cheirality and parallax into (P, 5); bad points become NaN, behind, 0."""
    out = np.empty((points.shape[0], 5))
    out[:, :3] = np.where(bad[:, None], np.nan, points)
    out[:, 3] = np.where(bad, 0.0, in_front.astype(np.float64))
    out[:, 4] = np.where(bad, 0.0, parallax)
    return out


def _n_view(
    poses: NDArray[np.float64], bearings: NDArray[np.float64], observed: NDArray[np.bool_]
) -> NDArray[np.float64]:
    """Triangulate by DLT on bearings (reference); return (P, 5)."""
    rotations, translations = poses[:, :3, :3], poses[:, :3, 3]
    centres = -np.einsum("vji,vj->vi", rotations, translations)  # c = -Rᵀ t
    out = np.empty((bearings.shape[0], 5))
    for p in range(bearings.shape[0]):
        views = np.flatnonzero(observed[p])
        mean = centres[views].mean(axis=0)
        scale = float(np.mean(np.linalg.norm(centres[views] - mean, axis=1)))
        scale = scale if scale > 0.0 else 1.0
        rows = []
        for v in views:
            # In the normalised world X = mean + scale · X', so R X + t = scale (R X' + t').
            t_norm = (translations[v] + rotations[v] @ mean) / scale
            skew = so3.hat(bearings[p, v])
            rows.append(np.hstack([skew @ rotations[v], (skew @ t_norm)[:, None]]))
        singular, vt = np.linalg.svd(np.vstack(rows))[1:]
        homogeneous = vt[-1]
        w = homogeneous[3]
        # Rays along one line leave a two-dimensional null space: the point is undetermined.
        undetermined = not singular[2] > PARALLEL_SINE * singular[0]
        at_infinity = undetermined or not abs(w) > PARALLEL_SINE * float(
            np.linalg.norm(homogeneous[:3])
        )
        point = mean + scale * homogeneous[:3] / w if not at_infinity else np.full(3, np.nan)
        in_cams = np.einsum("vij,j->vi", rotations[views], point) + translations[views]
        in_front = bool(np.all(np.sum(in_cams * bearings[p, views], axis=1) > 0.0))
        directions = point - centres[views]
        pairs = [(i, j) for i in range(len(views)) for j in range(i + 1, len(views))]
        parallax = max(float(_angle(directions[i], directions[j])) for i, j in pairs)
        out[p] = _pack(
            point[None], np.array([in_front]), np.array([parallax]), np.array([at_infinity])
        )[0]
    return out


def two_view(
    relative_pose: ArrayLike,
    bearings1: ArrayLike,
    bearings2: ArrayLike,
    *,
    backend: str = "python",
) -> Triangulation:
    """Triangulate bearing pairs optimally, through a known relative pose (Lindstrom's niter2).

    Parameters
    ----------
    relative_pose : array_like, shape (4, 4)
        ``T_c2_c1``, camera 1's frame to camera 2's, with a non-zero translation.
    bearings1, bearings2 : array_like, shape (P, 3)
        Unit bearing vectors of each point in camera 1 and camera 2, with positive z.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Triangulation
        The points in camera 1's frame, their cheirality and parallax.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose is not a rigid transform or has zero translation, or the bearings are not
        matching (P, 3) unit vectors with positive z.
    """
    pose, lead = as_batch(relative_pose, (4, 4), "relative_pose")
    if lead != ():
        raise InvalidInputError(f"relative_pose: expected one (4, 4) pose, got {lead}")
    se3.check_transform(pose)
    if not np.linalg.norm(pose[0, :3, 3]) > 0.0:
        raise InvalidInputError("relative_pose: zero translation; the depth is undetermined")
    b1, b2 = as_bearing_pairs(bearings1, bearings2, 0)
    packed = backends.dispatch(backend, __name__, "two_view", triangulate_pairs, pose[0], b1, b2)
    return Triangulation.unpack(packed)


def n_view(
    poses: ArrayLike,
    bearings: ArrayLike,
    observed: ArrayLike | None = None,
    *,
    backend: str = "python",
) -> Triangulation:
    """Triangulate points seen in two or more views, by the DLT on bearing vectors.

    Parameters
    ----------
    poses : array_like, shape (V, 4, 4)
        Each view's ``T_cam_world``.
    bearings : array_like, shape (P, V, 3)
        Each point's unit bearing in each view (ignored where not observed).
    observed : array_like of bool, shape (P, V), optional
        Which views observe each point (default: all). Each point needs two.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Triangulation
        The points in the world frame, their cheirality and parallax.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a pose is not a rigid transform, the shapes disagree, an observed bearing is not
        a unit vector, or a point is observed in fewer than two views.
    """
    flat_poses, lead = as_batch(poses, (4, 4), "poses")
    if len(lead) != 1:
        raise InvalidInputError(f"poses: expected (V, 4, 4), got {lead}")
    se3.check_transform(flat_poses)
    rays, ray_lead = as_batch(bearings, (3,), "bearings")
    if len(ray_lead) != 2 or ray_lead[1] != lead[0]:
        raise InvalidInputError(f"bearings: expected (P, {lead[0]}, 3), got {(*ray_lead, 3)}")
    mask = np.ones(ray_lead, dtype=bool) if observed is None else as_mask(observed, ray_lead)
    rays = rays.reshape((*ray_lead, 3))
    as_bearings(rays[mask], "bearings")
    if np.any(mask.sum(axis=1) < 2):
        raise InvalidInputError("observed: every point needs at least two observing views")
    packed = backends.dispatch(backend, __name__, "n_view", _n_view, flat_poses, rays, mask)
    return Triangulation.unpack(packed)
