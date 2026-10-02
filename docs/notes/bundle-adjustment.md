# Bundle adjustment

Design note for `slamlab.foundations.ba` (the Python reference) and
`slamlab/foundations/ba.hpp` (the C++ core, bound as `slamlab._cpp.foundations.ba`).

## The problem

Given images of a scene, bundle adjustment refines everything at once:
- the camera poses;
- the camera intrinsics, where they are not known;
- the 3D points.

It does this by minimising how far each point's projection lands from where it was
observed. It is the last step of structure from motion and the back end of every keyframe
SLAM system. Milestone 3 uses it to refine reifyscene's cameras.

```
cost = ½ Σ_m ρ(w_m |π(K_g(m), T_c(m) X_p(m)) - u_m|²)
```

There is one 2-row residual block per observation m. The block uses the camera's
world-to-camera pose `T_c`, its intrinsics group `K_g`, a world point `X_p`, the observed
pixel `u_m`, and a weight `w_m` (an information, `1/σ²` in px⁻²). The robust loss ρ applies
per observation. The solver is the NLS loop of [nls.md](nls.md): this module supplies its
`Problem`, a `LinearSystem` and a retraction.

## The interface

A problem crosses as a struct of arrays (design doc, "Conventions"):

| Array | Shape | Meaning |
|---|---|---|
| `intrinsics` | (K, P) | one row per intrinsics group |
| `poses` | (C, 4, 4) | camera-to-world `T_world_cam` (inverted internally) |
| `points` | (N, 3) | world points |
| `observations` | (M, 2) | pixels |
| `cam_idx`, `pt_idx` | (M,) int64 | each observation's camera and point |
| `intrinsics_idx` | (C,) int64 | each camera's group: shared, per camera, or anything between |
| `weights` | (M,) | observation weights; 0 excludes one |
| `fixed_poses`, `fixed_intrinsics`, `fixed_points` | bool | variables held fixed |

Intrinsics are fixed by default, as for a calibrated camera. Both cores check every index
and size, so a C++ caller gets the same protection as a Python one.

## The maths

**Parameters.** A pose is perturbed on the right of `T_cam_world`: `T ⊕ δ = T exp(δ)`, with
`δ = (ρ, θ)`. Because `T_cam_world` maps world to camera, a right perturbation moves the world
side, so `δ` is expressed in the world frame. For a point `X` seen at `p = R X + t`:

```
dp/dρ = R,    dp/dθ = -R hat(X),    dp/dX = R
```

These are chained through the camera's Jacobian `dπ/dp`, which [camera-models.md](camera-models.md)
provides along with `dπ/dK`. Points are plain 3-vectors, and intrinsics are added to.

**The Schur complement.** The normal equations split into a camera block (poses and
intrinsics), a point block that is block-diagonal with one 3x3 per point, and their
coupling:

```
[ H_cc  H_cp ] [δc]   [g_c]
[ H_pc  H_pp ] [δp] = [g_p]
```

Eliminating the points gives the **reduced camera system**:

```
S = H_cc - H_cp H_pp⁻¹ H_pc,    S δc = -(g_c - H_cp H_pp⁻¹ g_p)
```

It is as large as the cameras are, not the points. The points then follow by
back-substitution: `δp = -H_pp⁻¹ (g_p + H_pc δc)`.
- **Damping.** Levenberg-Marquardt's damping goes on `H_pp`'s diagonal before it is
  inverted, and on `S` in the reduced coordinates. The step therefore equals the damped full
  solve exactly.
- **The solver seam.** `S` is factored by a `ReducedSystemSolver`, an interface with one
  method: dense Cholesky now, sparse with milestone 4's larger problems.
- **Python.** The reference forms `S` vectorised: one block for every ordered pair of a
  point's observations, processed in chunks to bound memory.

## Gauge

Reprojection errors don't change if the whole scene moves rigidly, or, for a monocular
camera, scales: 7 directions in which the cost is flat. Left in, they make `H` singular. LM's
damping hides that, but undamped Gauss-Newton fails and the solution drifts. They are removed
instead:
- **`first_pose` (SE(3)).** If no observed pose is fixed, the first free observed one is.
- **`similarity` (Sim(3), the default).** As above. Then, unless two fixed poses at
  different places already pin the scale, the free pose farthest from a fixed one keeps its
  distance from it. Its camera centre moves as `dc = -ρ + c × θ` under a step (from
  `c = -Rᵀt`). Requiring `bᵀ dc = 0`, along the unit baseline `b`, is one linear constraint
  on its 6-vector, `a = (-b, b × c)`. Its step is confined to the 5-dimensional complement of
  `a`.
  - The basis comes from the Householder reflection that maps `a` onto its largest axis:
    every other column is orthogonal to `a`. It is deterministic, so both backends take the
    same coordinates and the same damped steps; an SVD null space would not be.
  - It is recomputed at each linearisation, so the baseline is held to first order. It
    drifts at second order (1e-4 relative in the tests).
- **`none`.** The caller's fixed variables define the gauge, such as fixed control points or
  a fixed rig.

## Degenerate data

Each of these is dropped, not frozen, and counted in the summary:
- **Points behind a camera at the start.** Their observations are dropped.
- **Free points seen fewer than twice, or with less than `min_parallax_deg` (1°) between
  their rays.** Parallax is the largest angle from a point's first ray to any other, within
  a factor of two of the largest pairwise angle. Such a point's depth is undetermined, and
  freezing it at a wrong depth would bias the cameras that see it, so all its observations
  are dropped.
- **Free poses and intrinsics groups that nothing observes.** These are held fixed.

During the solve, a step that moves a point behind a camera gives that observation an
infinite cost, so the step is rejected rather than raising an error.

## How it's checked

- **The Schur step equals the dense solve.** The damped step from the Schur system matches
  Cholesky on the full Jacobian, assembled in the same free coordinates, to 1e-10 relative.
  The test uses radtan with free intrinsics and Huber weights. The gradient and Hessian
  diagonal match to 1e-12.
- **The Jacobian.** The analytic Jacobian (poses, radtan intrinsics, points, and the gauge
  basis) matches `nls.numeric_jacobian`.
- **The optimum.**
  - With the trivial loss, it matches SciPy's `trf` over the independent parameterisation
    `T0 · Exp(δ)`, `X0 + d`: costs to 1e-9, reprojections to 1e-5 px.
  - With Huber, a robust cost written independently in numpy has a numerical gradient
    below 1e-8 of its starting value at our solution.
- **Statistics.** On σ = 1 px noise, `2·cost/dof` lies in [0.85, 1.15]. On noise-free data
  the reprojections return to 1e-8 px.
- **Intrinsics.** Focal lengths started 3% off are recovered to 1e-7 relative.
- **Gauge.**
  - Under undamped Gauss-Newton with the similarity gauge, the gauge pose is unchanged
    bit for bit, and the baseline holds to 1e-4.
  - Two explicitly fixed poses add no gauge pose.
- **Degeneracy.** A single-view point, a point 10⁷ away and an unobserved camera are
  dropped or held, counted, and returned unchanged.
- **Parity.** C++ against the reference, with radtan, two intrinsics groups and a Cauchy
  loss: same termination, iterations and counts; poses to 1e-9, points to 1e-8. On the smoke
  scene they agree to 1e-15.
- **Natively.** The same model is checked in C++: convergence to zero reprojection error,
  the gauge, a dropped point, and the core's own index check.

## What was learned

- **SciPy is the wrong oracle for robust BA.** On a 12-point scene with Huber, `trf` used up
  its evaluation budget at a cost three times ours, and took 50 s doing it. The robust
  optimum is now pinned by an independent gradient instead. That is the third time a
  library has needed a referee, after manif ([lie-groups.md](lie-groups.md)) and SciPy's
  linear case ([nls.md](nls.md)).
- **IRLS converges linearly on outliers.** Dropping `ρ''` (see [nls.md](nls.md)) costs
  little on inliers, but on Huber's outlier blocks the solve took 108 iterations to reach
  its floor. Stopping on a 1e-12 relative cost decrease left a gradient near 1e-4 along the
  scene's poorly determined depths. Tests that compare optima therefore run to the step
  floor, and compare in reprojection space, where the data constrains the solution.
- **Fixed means bit-exact.** The first version returned fixed poses 1 ulp off, from the
  round trip `T_world_cam → T_cam_world → T_world_cam`. Unmoved poses are now returned
  as given, in both cores.
