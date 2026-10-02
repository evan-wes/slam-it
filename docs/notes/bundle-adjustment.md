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
per observation. Two optional kinds of block join it (see "Priors" below): depth priors and
pose priors. The solver is the NLS loop of [nls.md](nls.md): this module supplies its
`Problem`, a `LinearSystem` and a retraction, and gives each kind of block its own loss
through the loop's per-block loss.

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

Optional prior arrays:

| Array | Shape | Meaning |
|---|---|---|
| `depth_prior`, `depth_prior_weights` | (M,) | a prior inverse depth per observation (NaN for none) |
| `depth_affine`, `fixed_depth_affine` | (C, 2), (C,) | each camera's prior-to-model `(scale, shift)` |
| `pose_prior`, `pose_prior_weights` | (C, 4, 4), (C, 2) | prior camera-to-world poses; `(translation, rotation)` informations |

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

## Anchored inverse-depth points

A point can be stored as one number instead of three: an inverse depth `ρ` along the ray of
an **anchor** observation (camera h, pixel `u_a`). This is DSO's host frame and DROID-SLAM's
per-pixel depth. It behaves well where XYZ does not:
- far points, where `ρ → 0` is finite and XYZ runs off to infinity;
- low-parallax points, whose depth is poorly constrained but whose direction is not.

`point_anchor` names each point's anchor (-1 for XYZ); `anchor_points` anchors every free
point at its first observation.

The anchor pixel gives a ray `m = (x, y, 1)` on h's normalised plane, undistorted with h's
current intrinsics, and

```
X_world = T_h⁻¹ (m / ρ)
```

Any block that sees the point has a world-point Jacobian `J_X`, chained three ways:

| Through | `d X_world` |
|---|---|
| ρ | `-R_hᵀ m / ρ²` |
| the host pose (right perturbation, world frame) | `[-I, hat(X_world)]` |
| the host intrinsics | `R_hᵀ (dm/dK) / ρ`, with `dm/dK = -(∂π/∂m)⁻¹ ∂π/∂K` |

The last row is the implicit function theorem on `π(K, m) = u_a`: if the focal length
changes, the anchor ray swings so that it still projects to the same pixel. For radtan this
saves differentiating the undistortion's Newton iteration.

**The anchor observation has no reprojection block.** Its error is identically zero, since it
defines the ray. A depth prior on it remains, and acts directly as a prior on `ρ`.

**In the Schur system,** an anchored point keeps the 3x3 point machinery. Its two unused
dimensions are inert: an identity Hessian, no coupling and no gradient, so their step is
exactly zero. A mask hides them from the solver's coordinates. A block on an anchored point
touches the host's pose and intrinsics columns as well as its own camera's, and those can be
the same columns when the observing camera is the host. The C++ scatters therefore
accumulate repeated indices explicitly; Eigen's indexed `+=` would keep only one.

**Validity.** An anchor must be one of the point's own observations. A point whose anchor
lies behind its camera at the start cannot be placed, and is dropped. A step that sends
`ρ ≤ 0` makes the point invalid, so the step is rejected. Anchored points start on their ray
at the depth the caller gave.

## Priors

**Depth priors.** An observation can carry a prior inverse depth `d̃`, from a depth network
or a sensor. Its residual is `√w (1/z - (s_c d̃ + t_c))`, a 1-row block coupled to the point,
the pose and that camera's affine `(s_c, t_c)`:
- **Metric priors** keep the affine fixed at `(1, 0)`.
- **Affine-invariant priors** (a relative-depth network's disparity) free it, and the solve
  finds each frame's scale and shift. Shape of Motion and MegaSaM align monocular depth this
  way.
- **Why inverse depth.** Networks predict it, it stays finite for far points, and its noise is
  closer to uniform than depth's.
- **Its own loss.** `BAOptions.depth_loss` (typically Huber or Cauchy) applies to these blocks
  only, so a network's failures on sky or reflections don't drag the cameras.

**Pose priors.** A camera can carry a prior pose `T̄`, from odometry, a previous solve or
GNSS. Its residual is `W log(T̄_cam_world⁻¹ T_cam_world)`, with separate translation and
rotation informations. It is a 6-row block on that pose alone, with the Jacobian
`W Jr⁻¹(e)` (right perturbation). Its loss is plain least squares.

**The Schur system with three kinds of block.** Every block contributes
`J_cᵀ W J_c` to the camera side. Blocks that see a point (reprojection, depth) also
contribute to that point's 3x3 block and the coupling. In Python the coupling blocks have
different widths (6+P for reprojection, 8 for depth), so they are zero-padded to the widest,
with padding columns that add exact zeros. In C++ each block carries its own column list.

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
- **`none`.** The caller's fixed variables or priors define the gauge, such as fixed
  control points, a fixed rig, or pose priors.
- **`auto` (the default).** Picks by what anchors the scene:
  - `none` when any pose has a prior;
  - `first_pose` when a metric depth prior (a fixed affine) fixes the scale;
  - `similarity` otherwise.

  The summary records the gauge actually applied.

## Degenerate data

Each of these is dropped, not frozen, and counted in the summary:
- **Points behind a camera at the start.** Their observations are dropped.
- **Free points seen fewer than twice.** Such a point's depth is undetermined, and freezing
  it at a wrong depth would bias the cameras that see it, so its observation is dropped.
- **Optionally, free points with less than `min_parallax_deg` between their rays** (default
  0, off). Parallax is the largest angle from a point's first ray to any other, within a
  factor of two of the largest pairwise angle. The default was 1° until real data
  disagreed: see "What was learned".
- **Free poses and intrinsics groups that nothing observes.** These are held fixed.

During the solve, a step that moves a point behind a camera gives that observation an
infinite cost, so the step is rejected rather than raising an error.

## Focal observability

When intrinsics are free, the result reports each free parameter's standard deviation. It is
the square root of the diagonal of `S⁻¹`, where `S` is the undamped reduced camera system at
the solution under the final robust weights: points marginalised by the Schur complement,
poses by the inverse. It assumes the weights are true informations. A free group whose fx or
fy deviation exceeds `focal_std_ratio` (1%) of itself is counted in `unobservable_focal`.

The classic failure it catches is focal length against depth. With pure translation and no
rotation, `u = f (X - t_x)/Z` is unchanged by `f → kf, Z → kZ`, so no amount of data pins f.

## Readers, and Ceres as the reference

**BAL** (`read_bal`). BAL's cameras look down `-z` with `y` up: `P = R X + t`,
`p = -P/P_z`, `u = f (1 + k1|p|² + k2|p|⁴) p`.
- **Frames.** Flipping the camera's `y` and `z` axes (`F = diag(1, -1, -1)`) gives
  `T_cam_world = [F R, F t]` and pixels `(x, -y)`.
- **Camera model.** BAL's camera is exactly the `radial` model `[f, cx, cy, k1, k2]`, added
  for it. It is radtan with one focal length and no tangential terms, computed through
  radtan by a fixed 9x5 map.
- **What is refined.** BAL and Ceres refine `f, k1, k2` but not the principal point, which
  needed per-parameter intrinsics masks: `fixed_intrinsics` is `(K, P)`.

**COLMAP text** (`read_colmap_text`).
- **Pixels.** COLMAP puts pixel centres at `(0.5, 0.5)`, so the reader shifts pixels and
  principal points by -0.5.
- **Models.** SIMPLE_PINHOLE, SIMPLE_RADIAL and RADIAL map onto `radial`; PINHOLE onto
  `pinhole`; PINHOLE and OPENCV onto `radtan`. Absent terms are zero, and `refinable` marks
  the ones the source model has.

**The reference.** `slamlab.eval.colmap_reference` builds a pycolmap reconstruction from a
problem and runs COLMAP's bundle adjuster, which is Ceres underneath. pyceres was the plan,
but its reprojection factors are gone from current releases. COLMAP's adjuster is the same
Ceres, on the same RADIAL model, and is also what reifyscene's reconstructions come from.

**On BAL Ladybug-49** (49 cameras, 7776 points, 31843 observations; 31 behind their camera):

| Solver | RMS reprojection error | Time |
|---|---|---|
| start | 7.314 px | |
| slamlab, Python | 0.915 px | ~60 s |
| slamlab, C++ (dense reduced solve) | 0.9147 px | ~2 s |
| COLMAP's Ceres | 0.9147 px | 0.4 s |

The dense reduced solve is the gap. Milestone 4 brings the sparse one, and the
`ReducedSystemSolver` seam is where it plugs in. `scripts/demo_bal.py` reproduces the table
and, with `--rerun`, saves a recording of the cameras and points before and after.

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
- **Priors.**
  - The Schur-versus-dense and Jacobian checks include depth blocks (with free affines) and
    pose priors, one of them with zero rotation weight.
  - Metric depth priors recover the true scene from a start scaled 1.5x. Without them, the
    similarity gauge keeps the 1.5x scale.
  - A prior known only as `(1/z - 0.05)/2` gives back the affine `(2, 0.05)` to 1e-8.
  - Pose priors at the truth return the truth, with the auto gauge resolved to `none`.
- **Anchored points.**
  - The Schur, Jacobian and parity tests anchor every third point; one of them carries a
    depth prior at its anchor. They run on radtan with free intrinsics, which exercises the
    `dm/dK` path.
  - On noise-free data, a fully anchored problem reaches zero cost, with each `ρ` equal to
    its point's `1/z` in the host to 1e-12.
  - A wrong anchor is rejected, and an anchor behind its camera drops its point.
- **Focal observability.** On the same scene with three amounts of rotation:

  | Rotation spread | σ_f | Flagged |
  |---|---|---|
  | 0.3 rad | under 1 px; the truth lies within 3σ | no |
  | 0.04 rad | 9 px (1.8%) | yes |
  | none | about 1400 px | yes |

- **Parity.** As above, with every kind of block. The intrinsics agree to 1e-8 relative in a
  scene where the distortion is poorly conditioned (k3 near -6).
- **Readers and Ceres.**
  - BAL files (plain and bz2) read back so that our model reproduces BAL's own projection
    equation, written independently in the test, to 1e-10 px.
  - COLMAP text models of four camera types reproduce pycolmap's projections of the same
    files, and mixed models are rejected.
  - On a BAL problem with `f, k1, k2` free, our optimum matches COLMAP's Ceres BA in both
    backends: RMS error to 1e-8 relative, focal lengths to 1e-7.
- **Natively.** The same model is checked in C++:
  - convergence to zero reprojection error;
  - the gauge;
  - a dropped point;
  - metric depth recovering the scale;
  - the core's own index check.

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
- **A small deviation estimate is not small enough.** The first focal test used a scene with
  0.04 rad of rotation between views, assuming it would count as well observed. The check
  disagreed: σ_f = 9 px, and the estimate was 1σ off the truth. The check was right and the
  assumption was wrong. Weak rotation makes focal length weakly observable, which is why
  calibration needs varied views.
- **A triangulation heuristic is not a BA rule.** The first default dropped points with less
  than 1° of parallax. On BAL's Ladybug-49, that left out 37 points (239 observations), which
  kept their starting positions, and the RMS error ended at 1.536 px against COLMAP's
  0.915 px. With the filter off, ours reaches 0.9147 px, the same as COLMAP. COLMAP applies
  its 1.5° check when it *creates* points; bundle adjustment then refines low-parallax points
  well, since damping handles their weak depth. The filter is now off by default.
- **Fixed means bit-exact.** The first version returned fixed poses 1 ulp off, from the
  round trip `T_world_cam → T_cam_world → T_world_cam`. Unmoved poses are now returned
  as given, in both cores.
