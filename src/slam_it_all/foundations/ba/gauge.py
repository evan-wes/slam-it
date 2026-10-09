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

The rows are built in scene-normalised coordinates, ``X' = (X - origin) / scale`` (the
anchors' and observed cameras' mean and RMS spread at the start): ``g`` acts about the
world origin, so in world coordinates the rotation and scale directions shrink like
``1 / |X|`` far from it, and a rank test there fails for geo-referenced scenes. A change of
origin and unit is an invertible map of ``g``, so the spans -- and the gauge -- are the same.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace

import numpy as np

from slam_it_all.foundations import so3
from slam_it_all.foundations.ba.problem import POSE_DIM, Array, IndexArray
from slam_it_all.foundations.errors import DegenerateError

GENERATORS = 7
"""The similarity's directions: translation (3), rotation (3), scale (1)."""

RIGID_DIM = 6
"""The rigid motions' directions: translation (3), rotation (3)."""

TOLERANCE = 1e-9
"""Singular values below this count as zero. Rows and images are in scene-normalised
coordinates (see :class:`Frame`), so their entries are of order 1 and it is in effect
relative."""

TIE_RTOL = 1e-12
"""Two scores within this relative margin are a tie, broken by the lower index. Both cores
then choose alike despite last-bit rounding between their SVDs."""


@dataclass(frozen=True)
class Frame:
    """Scene-normalised coordinates for the gauge: ``X' = (X - origin) / scale``."""

    origin: Array
    """The mean of the anchors' and observed cameras' world positions, shape (3,)."""
    scale: float
    """Their RMS distance from ``origin`` (1 if they coincide)."""

    @classmethod
    def of(cls, positions: Array) -> Frame:
        """Return the frame centred on ``positions`` (N, 3) and scaled by their spread."""
        if positions.shape[0] == 0:
            return cls(np.zeros(3), 1.0)
        origin = positions.mean(axis=0)
        spread = float(np.sqrt(np.mean(np.sum((positions - origin) ** 2, axis=1))))
        return cls(origin, spread if spread > 0.0 else 1.0)

    def __call__(self, positions: Array) -> Array:
        """Return world positions (..., 3) in this frame."""
        return np.asarray((positions - self.origin) / self.scale, dtype=np.float64)


@dataclass(frozen=True)
class Anchors:
    """What pins the world's similarity, before any gauge is applied."""

    points: Array
    """Fixed known positions in the world, shape (Np, 3)."""
    centre_cams: IndexArray
    """Poses whose current world centre is pinned (fixed poses, translation priors)."""
    orientation_cams: IndexArray
    """Poses whose current world orientation is pinned (fixed poses, rotation priors)."""
    metric: bool
    """Whether metric depth pins the scale."""
    frame: Frame
    """The scene-normalised coordinates the rows are built in."""

    def with_fixed(self, cam: int) -> Anchors:
        """Return these anchors with pose ``cam`` fixed (its centre and orientation)."""
        return replace(
            self,
            centre_cams=np.append(self.centre_cams, cam),
            orientation_cams=np.append(self.orientation_cams, cam),
        )


@dataclass(frozen=True)
class Plan:
    """Which poses remove the gauge, decided once from the start."""

    fixed: IndexArray
    """Poses fixed outright (each saw all six of its directions)."""
    confined: IndexArray
    """Poses whose step is confined, in the order they take their share."""
    ranks: IndexArray
    """How many directions each confined pose removes."""
    free: int
    """Directions no free pose could take: left in the problem (0 unless degenerate)."""


NO_PLAN = Plan(
    fixed=np.zeros(0, dtype=np.int64),
    confined=np.zeros(0, dtype=np.int64),
    ranks=np.zeros(0, dtype=np.int64),
    free=0,
)
"""Nothing to remove."""


def _rows(anchors: Anchors, centres: Array) -> Array:
    """Return the anchors' rows of ``d(pinned) / d(t, ω, s)`` in the frame, shape (q, 7).

    ``centres`` are every camera's world centres, shape (C, 3).
    """
    positions = anchors.frame(np.concatenate([anchors.points, centres[anchors.centre_cams]]))
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


@contextmanager
def _svd_failure_is_degenerate() -> Iterator[None]:
    """Raise numpy's SVD failure (``LinAlgError``, on a non-finite matrix) as Degenerate.

    The compiled cores report the same failure as ``Degenerate``, so the gauge fails the same
    way on every backend.
    """
    try:
        yield
    except np.linalg.LinAlgError as error:
        raise DegenerateError(
            "gauge: the SVD of the pinned directions failed (a non-finite anchor or camera centre)"
        ) from error


def _null(matrix: Array, rank: int) -> Array:
    """Return an orthonormal basis of the null space of a rank-``rank`` matrix's rows."""
    if matrix.shape[0] == 0:
        return np.eye(matrix.shape[1])
    with _svd_failure_is_degenerate():
        vt = np.linalg.svd(matrix, full_matrices=True)[2]
    return np.asarray(vt[rank:].T, dtype=np.float64)


def _singular(matrix: Array) -> Array:
    """Return a matrix's singular values, largest first (none for an empty matrix)."""
    if matrix.size == 0:
        return np.zeros(0)
    with _svd_failure_is_degenerate():
        return np.asarray(np.linalg.svd(matrix, compute_uv=False), dtype=np.float64)


def remaining(anchors: Anchors, centres: Array) -> Array:
    """Return the remaining gauge N.

    Parameters
    ----------
    anchors : Anchors
        What is pinned.
    centres : ndarray, shape (C, 3)
        Every camera's centre in the world frame.

    Returns
    -------
    ndarray, shape (7, k)
        An orthonormal basis of N, over ``(t, ω, s)`` in the anchors' frame.
    """
    rows = _rows(anchors, centres)
    return _null(rows, int(np.count_nonzero(_singular(rows) > TOLERANCE)))


def _pose_image(centre: Array, null: Array) -> Array:
    """Return how N moves a camera's centre and orientation, shape (6, k), in the frame.

    ``centre`` is the camera's centre in the frame; rows are the centre's change, then the
    world orientation's.
    """
    generator = np.zeros((POSE_DIM, GENERATORS))
    generator[:3, :3] = np.eye(3)
    generator[:3, 3:6] = -so3.hat(centre)
    generator[:3, 6] = centre
    generator[3:, 3:6] = np.eye(3)
    return generator @ null


def classify(null: Array) -> str:
    """Name a remaining gauge: ``none``, ``scale``, ``first_pose``, ``similarity``, ``partial``.

    ``scale`` is a scale about some point (no rotation), ``first_pose`` the rigid motions
    (no scale). The tests read only N's dimension and its ω and s rows, which a change of
    origin and unit leaves alone.

    Parameters
    ----------
    null : ndarray, shape (7, k)
        The remaining gauge, from :func:`remaining`.

    Returns
    -------
    str
        Its name.
    """
    k = null.shape[1]
    if k == 0:
        return "none"
    if k == GENERATORS:
        return "similarity"
    if k == RIGID_DIM and np.allclose(null[GENERATORS - 1], 0.0, atol=TOLERANCE):
        return "first_pose"
    if k == 1 and np.allclose(null[3:6], 0.0, atol=TOLERANCE):
        return "scale"
    return "partial"


def _tie_break(cam: int, singular: Array, rank: int) -> float:
    """Score candidates seeing the same number of directions (higher wins).

    A pose that would be fixed: the lowest index (``-cam``). One that would be confined: the
    one seeing its directions best, its smallest retained singular value (for scale, the
    farthest from the centre).
    """
    if rank == POSE_DIM:
        return -float(cam)
    return float(singular[rank - 1]) if rank else 0.0


def _best_candidate(
    anchors: Anchors, centres: Array, left: list[int], null: Array
) -> tuple[int, int]:
    """Return the candidate that sees the most of N (ties by :func:`_tie_break`), and how many."""
    best, best_rank, best_score = -1, 0, 0.0
    for c in left:
        singular = _singular(_pose_image(anchors.frame(centres[c]), null))
        rank = int(np.count_nonzero(singular > TOLERANCE))
        score = _tie_break(c, singular, rank)
        if rank > best_rank or (
            rank == best_rank and score > best_score + TIE_RTOL * abs(best_score)
        ):
            best, best_rank, best_score = c, rank, score
    return best, best_rank


def plan(anchors: Anchors, centres: Array, candidates: IndexArray) -> Plan:
    """Choose the poses that remove the remaining gauge, greedily.

    At each turn the candidate seeing the most remaining directions takes them: a pose that
    sees all six is fixed, any other is confined. N then shrinks to what it did not see.

    Parameters
    ----------
    anchors : Anchors
        What is pinned.
    centres : ndarray, shape (C, 3)
        Every camera's centre in the world frame, at the start.
    candidates : ndarray of int
        The free observed poses, in index order.

    Returns
    -------
    Plan
        The poses, and what no pose could take.
    """
    null = remaining(anchors, centres)
    fixed: list[int] = []
    confined: list[int] = []
    ranks: list[int] = []
    left = list(candidates)
    while null.shape[1] and left:
        best, rank = _best_candidate(anchors, centres, left, null)
        if rank == 0:
            break
        if rank == POSE_DIM:
            fixed.append(best)
        else:
            confined.append(best)
            ranks.append(rank)
        null = null @ _null(_pose_image(anchors.frame(centres[best]), null), rank)
        left.remove(best)
    return Plan(
        np.array(fixed, dtype=np.int64),
        np.array(confined, dtype=np.int64),
        np.array(ranks, dtype=np.int64),
        free=int(null.shape[1]),
    )


def _pose_jacobian(centre: Array) -> Array:
    """Return ``d(centre, orientation) / dδ`` for a right perturbation of ``T_cam_world``.

    ``centre`` is the camera's world centre; the result is in world units.
    """
    jacobian = np.zeros((POSE_DIM, POSE_DIM))
    jacobian[:3, :3] = -np.eye(3)
    jacobian[:3, 3:] = so3.hat(centre)
    jacobian[3:, 3:] = -np.eye(3)
    return jacobian


def _first_largest(values: Array) -> int:
    """Return the first index within :data:`TIE_RTOL` of the largest value."""
    return int(np.flatnonzero(values >= values.max() * (1.0 - TIE_RTOL))[0])


def _canonical_span(matrix: Array, rank: int) -> list[Array]:
    """Return an orthonormal basis of a matrix's column span that depends on the span alone.

    From the span's projector P: repeatedly take the axis with the largest ``P_kk``, its
    normalised column, and deflate. Any two bases of the same span give the same vectors, so
    both cores agree whatever their SVDs return.
    """
    with _svd_failure_is_degenerate():
        u = np.linalg.svd(matrix, full_matrices=False)[0][:, :rank]
    projector = u @ u.T
    out = []
    for _ in range(rank):
        k = _first_largest(np.diagonal(projector))
        vector = projector[:, k] / np.sqrt(projector[k, k])
        out.append(vector)
        projector = projector - np.outer(vector, vector)
    return out


def _complement(normals: list[Array], size: int) -> Array:
    """Return an orthonormal (size, size - len(normals)) basis orthogonal to ``normals``.

    One Householder reflection per normal, each mapping it onto its largest remaining axis,
    which is then dropped; the other axes stay orthogonal to it. For one normal this is the
    reflection's other columns. Deterministic, so both cores take the same coordinates.
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


def confinement(anchors: Anchors, centres: Array, chosen: Plan) -> list[Array]:
    """Return each confined pose's step basis at the current centres.

    ``anchors`` include the poses the plan fixed. N is rebuilt from them at these centres,
    with the dimension the plan left (its confined ranks plus what it could not remove), and
    each confined pose takes its share as the plan decided. The normal ``J_cᵀ M`` is taken in
    world units: ``M``'s centre rows scale back by the frame's scale.

    Parameters
    ----------
    anchors : Anchors
        What is pinned, the plan's fixed poses included.
    centres : ndarray, shape (C, 3)
        Every camera's current centre in the world frame.
    chosen : Plan
        The plan.

    Returns
    -------
    list of ndarray, shape (6, 6 - rank)
        One basis per confined pose, in the plan's order, over its right-perturbation step.
    """
    dimension = int(np.sum(chosen.ranks)) + chosen.free
    null = _null(_rows(anchors, centres), GENERATORS - dimension)
    to_world = np.diag([anchors.frame.scale] * 3 + [1.0] * 3)
    bases = []
    for c, rank in zip(chosen.confined, chosen.ranks, strict=True):
        image = _pose_image(anchors.frame(centres[c]), null)
        normals = _canonical_span(_pose_jacobian(centres[c]).T @ to_world @ image, int(rank))
        bases.append(_complement(normals, POSE_DIM))
        null = null @ _null(image, int(rank))
    return bases
