"""The gauge: which similarity motions of the world the data leaves free, and their removal.

A bundle adjustment's cost is unchanged when the whole world moves by a similarity: a
translation ``t``, a rotation ``ω`` and (for monocular data) a scale ``s``. Fixed variables
and priors pin some of these seven directions. Each anchor contributes rows of the
derivative of what it pins with respect to ``g = (t, ω, s)``:

- a known position ``X`` (a fixed point, a fixed pose's centre, a translation prior's pose):
  ``dX = t + hat(ω) X + s X``, the rows ``[I, -hat(X), X]``;
- a known orientation (a fixed pose, a rotation prior's pose): the rows ``[0, I, 0]``;
- metric depth: the row ``[0, 0, 1]``.

The **remaining gauge** is the null space N of those rows. Removing exactly N, no more, is
what Triggs et al. (1999, section 9) require: fixing more biases the solution, fixing less
leaves the system singular.

N is removed pose by pose. At pose c, N moves the camera's world centre and orientation by
``M = [[I, -hat(C), C], [0, I, 0]] N``. Its step is confined to the complement of
``J_cᵀ M``, where ``J_c`` maps a right-perturbation step of ``T_cam_world`` to the change in
centre and orientation: along the gauge directions, the pose's centre and orientation hold.
A pose that sees six directions is simply fixed. N then shrinks to the directions the pose
does not see, and the next pose takes the rest. For scale about a reference this is the
familiar baseline-keeping gauge; for the whole similarity, it fixes one pose and confines a
second.

Known positions on free poses (priors) are read at the current state, so the confinement is
rebuilt at every state; fixed points and fixed poses are constant.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from slamlab.foundations import so3
from slamlab.foundations.ba.problem import POSE_DIM, Array, IndexArray

GENERATORS = 7
"""The similarity's directions: translation (3), rotation (3), scale (1)."""

TOLERANCE = 1e-9
"""Singular values below this, relative to the scene's extent, count as zero."""


@dataclass(frozen=True)
class Anchors:
    """What pins the world's similarity, before any gauge is applied."""

    points: Array
    """Fixed known positions in the world, shape (Np, 3)."""
    centre_cams: IndexArray
    """Poses whose current centre is pinned (fixed poses, translation priors)."""
    orientation_cams: IndexArray
    """Poses whose current orientation is pinned (fixed poses, rotation priors)."""
    metric: bool
    """Whether metric depth pins the scale."""

    def with_fixed(self, cam: int) -> Anchors:
        """Return these anchors with pose ``cam`` fixed (its centre and orientation)."""
        return replace(
            self,
            centre_cams=np.append(self.centre_cams, cam),
            orientation_cams=np.append(self.orientation_cams, cam),
        )


def _rows(anchors: Anchors, centres: Array) -> Array:
    """Return the anchors' rows of ``d(pinned) / d(t, ω, s)``, shape (q, 7)."""
    positions = np.concatenate([anchors.points, centres[anchors.centre_cams]])
    blocks = [
        np.concatenate(
            [
                np.broadcast_to(np.eye(3), (positions.shape[0], 3, 3)),
                -so3.hat(positions),
                positions[:, :, None],
            ],
            axis=2,
        ).reshape(-1, GENERATORS)
    ]
    if anchors.orientation_cams.size:
        blocks.append(np.hstack([np.zeros((3, 3)), np.eye(3), np.zeros((3, 1))]))
    if anchors.metric:
        blocks.append(np.eye(1, GENERATORS, GENERATORS - 1))
    return np.concatenate(blocks)


def _null(matrix: Array, rank: int) -> Array:
    """Return an orthonormal basis of the null space of a rank-``rank`` matrix's rows."""
    if matrix.shape[0] == 0:
        return np.eye(matrix.shape[1])
    vt = np.linalg.svd(matrix, full_matrices=True)[2]
    return np.asarray(vt[rank:].T, dtype=np.float64)


def _rank(matrix: Array, tolerance: float) -> int:
    """Return the number of singular values above ``tolerance``."""
    if matrix.size == 0:
        return 0
    return int(np.count_nonzero(np.linalg.svd(matrix, compute_uv=False) > tolerance))


def remaining(anchors: Anchors, centres: Array, tolerance: float) -> Array:
    """Return the remaining gauge N, an orthonormal (7, k) basis."""
    rows = _rows(anchors, centres)
    return _null(rows, _rank(rows, tolerance))


def pose_image(centre: Array, null: Array) -> Array:
    """Return how N moves a camera's world centre and orientation, shape (6, k)."""
    generator = np.zeros((POSE_DIM, GENERATORS))
    generator[:3, :3] = np.eye(3)
    generator[:3, 3:6] = -so3.hat(centre)
    generator[:3, 6] = centre
    generator[3:, 3:6] = np.eye(3)
    return generator @ null


def name(null: Array) -> str:
    """Name the remaining gauge: ``none``, ``scale``, ``first_pose``, ``similarity``, ``partial``.

    ``scale`` is a scale about some point (no rotation), ``first_pose`` the rigid motions
    (no scale).
    """
    k = null.shape[1]
    if k == 0:
        return "none"
    if k == GENERATORS:
        return "similarity"
    if k == POSE_DIM and np.allclose(null[GENERATORS - 1], 0.0, atol=TOLERANCE):
        return "first_pose"
    if k == 1 and np.allclose(null[3:6], 0.0, atol=TOLERANCE):
        return "scale"
    return "partial"


@dataclass(frozen=True)
class Plan:
    """Which poses remove the gauge, decided once from the start."""

    fixed: IndexArray
    """Poses fixed outright (each saw all six of its directions)."""
    confined: IndexArray
    """Poses whose step is confined, in the order they take their share."""
    ranks: IndexArray
    """How many directions each confined pose removes."""


def _preference(cam: int, singular: Array, rank: int) -> float:
    """Rank candidates seeing the same number of directions.

    A pose that would be fixed: the lowest index first. One that would be confined: the one
    seeing its directions best (largest smallest singular value).
    """
    if rank == POSE_DIM:
        return -float(cam)
    return float(singular[rank - 1]) if rank else 0.0


def plan(anchors: Anchors, centres: Array, candidates: IndexArray, tolerance: float) -> Plan:
    """Choose the poses that remove the remaining gauge, greedily.

    At each turn the candidate seeing the most remaining directions takes them. Among
    poses that see all six, the lowest index is fixed; among the others, the one seeing them
    best (largest smallest singular value: for scale, the farthest from its centre).

    Parameters
    ----------
    anchors : Anchors
        What is pinned.
    centres : ndarray, shape (C, 3)
        The cameras' world centres at the start.
    candidates : ndarray of int
        The free observed poses, in index order.
    tolerance : float
        The absolute rank tolerance.
    """
    null = remaining(anchors, centres, tolerance)
    fixed: list[int] = []
    confined: list[int] = []
    ranks: list[int] = []
    left = list(candidates)
    while null.shape[1] and left:
        best, best_rank, best_second = -1, 0, 0.0
        for c in left:
            singular = np.asarray(
                np.linalg.svd(pose_image(centres[c], null), compute_uv=False), dtype=np.float64
            )
            rank = int(np.count_nonzero(singular > tolerance))
            second = _preference(c, singular, rank)
            # Beat the best by a relative 1e-12, so ties go to the lowest index in both cores.
            if rank > best_rank or (
                rank == best_rank and second > best_second + 1e-12 * abs(best_second)
            ):
                best, best_rank, best_second = c, rank, second
        rank = best_rank
        if rank == 0:
            break
        (fixed if rank == POSE_DIM else confined).append(best)
        if rank < POSE_DIM:
            ranks.append(rank)
        null = null @ _null(pose_image(centres[best], null), rank)
        left.remove(best)
    return Plan(
        np.array(fixed, dtype=np.int64),
        np.array(confined, dtype=np.int64),
        np.array(ranks, dtype=np.int64),
    )


def _pose_jacobian(centre: Array) -> Array:
    """Return ``d(centre, orientation) / dδ`` for a right perturbation of ``T_cam_world``."""
    jacobian = np.zeros((POSE_DIM, POSE_DIM))
    jacobian[:3, :3] = -np.eye(3)
    jacobian[:3, 3:] = so3.hat(centre)
    jacobian[3:, 3:] = -np.eye(3)
    return jacobian


def _first_largest(values: Array) -> int:
    """Return the first index within a relative 1e-12 of the largest value.

    Near-ties then resolve the same way in both cores despite last-bit rounding.
    """
    return int(np.flatnonzero(values >= values.max() * (1.0 - 1e-12))[0])


def _canonical_span(matrix: Array, rank: int) -> list[Array]:
    """Return an orthonormal basis of a matrix's column span that depends on the span alone.

    From the span's projector P: repeatedly take the axis with the largest ``P_kk``, its
    normalised column, and deflate. Any two bases of the same span give the same vectors, so
    both cores agree whatever their SVDs return.
    """
    u = np.linalg.svd(matrix, full_matrices=False)[0][:, :rank]
    projector = u @ u.T
    out = []
    for _ in range(rank):
        k = _first_largest(np.diagonal(projector))
        vector = projector[:, k] / np.sqrt(projector[k, k])
        out.append(vector)
        projector = projector - np.outer(vector, vector)
    return out


def complement(normals: list[Array], size: int) -> Array:
    """Return an orthonormal basis of the complement of ``normals``, deterministically.

    One Householder reflection per normal, each mapping it onto its largest remaining axis,
    which is then dropped; the other axes stay orthogonal to it. For one normal this is the
    reflection's other columns.
    """
    basis = np.eye(size)
    for normal in normals:
        coordinates = basis.T @ normal
        unit = coordinates / np.linalg.norm(coordinates)
        k = _first_largest(np.abs(unit))
        v = unit.copy()
        v[k] += 1.0 if unit[k] >= 0.0 else -1.0
        reflection = np.eye(unit.shape[0]) - 2.0 * np.outer(v, v) / (v @ v)
        basis = np.delete(basis @ reflection, k, axis=1)
    return basis


def confinement(
    anchors: Anchors, centres: Array, confined: IndexArray, ranks: IndexArray
) -> list[Array]:
    """Return each confined pose's step basis at the current centres, shape (6, 6 - rank).

    ``anchors`` include the poses the plan fixed. N is rebuilt from them at these centres,
    with the dimension the plan found (the confined ranks' sum), then each confined pose
    takes its share as the plan decided.
    """
    null = _null(_rows(anchors, centres), GENERATORS - int(np.sum(ranks)))
    bases = []
    for c, rank in zip(confined, ranks, strict=True):
        image = pose_image(centres[c], null)
        normals = _canonical_span(_pose_jacobian(centres[c]).T @ image, int(rank))
        bases.append(complement(normals, POSE_DIM))
        null = null @ _null(image, int(rank))
    return bases
