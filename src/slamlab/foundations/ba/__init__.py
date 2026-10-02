"""Bundle adjustment: refine camera poses, intrinsics and 3D points against their images.

The cost is a robust sum over three kinds of residual block,

    cost = ½ Σ_m ρ(w_m |π(K_g(m), T_c(m) X_p(m)) - u_m|²)          reprojection, 2 rows
         + ½ Σ_d ρ_d(w_d (1/z_c(d)(X_p(d)) - (s_c d̃_d + t_c))²)      depth prior, 1 row
         + ½ Σ_c |W_c log(T̄_c⁻¹ T_c)|²                                pose prior, 6 rows

minimised by :func:`slamlab.foundations.nls.solve`, each kind with its own loss.

**Reprojection.** ``T_c`` is camera c's world-to-camera pose, ``K_g`` its intrinsics group,
``X_p`` a world point, ``u_m`` the observed pixel and ``w_m`` the observation's weight (an
information, ``1/σ²`` in px⁻²).

**Depth priors.** An observation can carry a prior inverse depth ``d̃`` (from a depth network
or a depth sensor) with its own weight and robust loss. The prior is mapped through a
per-camera affine ``(s_c, t_c)``:

- for a metric prior, keep the affine fixed at ``(1, 0)``;
- for an affine-invariant one (a relative-depth network's disparity), free it, so the solve
  finds each frame's scale and shift.

Inverse depth, not depth, because networks predict it and it stays finite for far points.

**Pose priors.** A camera can carry a prior pose ``T̄_c`` (from odometry or a previous solve),
weighted separately in translation and rotation. The error is ``log(T̄_c⁻¹ T_c)`` with
``T_c`` world-to-camera, so it is expressed in the world frame like the poses' steps.

**Parameters.**
- Poses are perturbed on the right of ``T_cam_world``, ``T ⊕ δ = T exp(δ)``, so ``δ = (ρ, θ)``
  is expressed in the world frame.
- Points are plain 3-vectors.
- Intrinsics are shared through ``intrinsics_idx``: several cameras can use one group, or
  each its own.
- Depth affines are per camera.

Any pose, intrinsics group, affine or point can be held fixed. Callers pass and receive
camera-to-world poses ``T_world_cam`` (design doc, "Conventions").

**The Schur complement.** Points are eliminated before the solve. Each point's 3x3 block
is inverted, and the reduced camera system (poses, intrinsics and affines) is factored by a
:class:`ReducedSystemSolver` (dense Cholesky here; a sparse solver arrives with larger
problems). The points then follow by back-substitution.

**Gauge.** Reprojection errors don't change if the whole scene moves rigidly, or, for a
monocular camera, scales. Those directions are removed, not left to the damping:

- ``"first_pose"`` (SE(3) gauge): if no pose is fixed, the first free observed pose is.
- ``"similarity"`` (Sim(3) gauge): as ``"first_pose"``. Then, unless two fixed poses
  already pin the scale, the free pose farthest from a fixed one may not move its centre
  along that baseline. Its 6-dimensional step is confined to the 5-dimensional subspace
  orthogonal (to first order) to the change ``dc = -ρ + cross(c, θ)`` of its centre ``c``
  along the baseline. The basis of that subspace is a Householder reflection, deterministic
  in every backend.
- ``"none"``: the caller's fixed variables or priors define the gauge.
- ``"auto"`` (the default) picks by what the data observes:
  - ``"none"`` when any pose has a prior (it anchors the scene);
  - ``"first_pose"`` when a metric depth prior (a fixed affine) fixes the scale;
  - ``"similarity"`` otherwise.

**Degenerate data is dropped, and counted:**
- observations of a point behind its camera at the start;
- every observation of a free point seen fewer than twice, or with less than
  ``min_parallax_deg`` between its rays.

A point with no parallax has an undetermined depth. Freezing it at a wrong depth would bias
the cameras that see it. Free poses, intrinsics groups and affines left with nothing
observing them are held fixed.

**Focal observability.** When intrinsics are free, the result reports their standard
deviations from the marginal covariance at the solution (the inverse of the reduced camera
system, under the final robust weights). A free group whose focal length is uncertain by
more than ``focal_std_ratio`` of itself is counted as unobservable: the data does not pin
it. Those deviations assume the weights are true informations.
"""

from slamlab.foundations.ba.problem import (
    AFFINE_DIM,
    COUNT_NAMES,
    GAUGES,
    POSE_DIM,
    BAOptions,
    BAProblem,
    BAResult,
    BASummary,
    anchor_points,
)
from slamlab.foundations.ba.schur import DenseCholesky, ReducedSystemSolver
from slamlab.foundations.ba.solve import Implementation, solve

__all__ = [
    "AFFINE_DIM",
    "COUNT_NAMES",
    "GAUGES",
    "POSE_DIM",
    "BAOptions",
    "BAProblem",
    "BAResult",
    "BASummary",
    "DenseCholesky",
    "Implementation",
    "ReducedSystemSolver",
    "anchor_points",
    "solve",
]
