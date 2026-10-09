# Bundle adjustment

Design note for `slam_it_all.foundations.ba` (the Python reference) and
`slam_it_all/foundations/ba.hpp` (the C++ core, bound as `slam_it_all._cpp.foundations.ba`).

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
an **anchor** observation (camera h, pixel `u_a`). The parametrisation is Civera, Davison &
Montiel's (2008) for monocular SLAM. It is DSO's host frame (Engel, Koltun & Cremers 2018)
and DROID-SLAM's per-pixel depth. It behaves well where XYZ does not:
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

**Validity.**
- An anchor must be one of the point's own observations, and a fixed point cannot be
  anchored (its world position would still move with the host).
- A point whose anchor lies behind its camera at the start cannot be placed, and is dropped.
- So is one whose anchor pixel the starting intrinsics cannot undistort: it lies beyond the
  radius where the lens model folds back.
- During the solve, a step that sends `ρ ≤ 0`, or pushes an anchor pixel out of the
  invertible region, makes the point invalid, so the step is rejected. Both cores behave
  alike here. The Python reference used to raise instead (a panel finding).
- Anchored points start on their ray at the depth the caller gave.

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

**Pose priors.** A camera can carry a prior pose `T̄_world_cam`, from odometry, a previous
solve or GNSS. Its residual is decoupled: `(C - C̄, log(R̄_wcᵀ R_wc))`, the camera centre's
offset and the world orientation's, with separate translation and rotation informations. It
is a 6-row block on that pose alone. Under a right perturbation `T_cw exp(δ)`,
`δ = (ρ, θ)`, the centre moves by `-ρ + [C]× θ` and the rotation error by
`-Jr⁻¹(e_r) R_cw θ`. Its loss is plain least squares.
- **Why decoupled.** A GNSS fix gives a centre and no attitude. The translation half must
  then pin the centre whatever rotation the prior stores. The first version used
  `log(T̄_cw⁻¹ T_cw)`, whose translation half is `R̄ᵀ(t - t̄)`, with `t = -R C`. That compares
  world-to-camera translations through the stored rotation:
  - it is blind to a rotation of the world about the origin, which moves the centre but not
    `t`, so the gauge was left with a free direction (σ_f infinite);
  - a prior stored with identity rotation pulled the centres 0.1-0.8 m off.

  The decoupled form is what GTSAM's GPS factor uses for the position.

**The Schur system with three kinds of block.** Every block contributes
`J_cᵀ W J_c` to the camera side. Blocks that see a point (reprojection, depth) also
contribute to that point's 3x3 block and the coupling. In Python the coupling blocks have
different widths (6+P for reprojection, 8 for depth), so they are zero-padded to the widest,
with padding columns that add exact zeros. In C++ each block carries its own column list.

## Gauge

Reprojection errors don't change if the whole scene moves by a similarity: a rigid motion
and, for a monocular camera, a scale. Those are 7 directions `g = (t, ω, s)` in which the
cost is flat. Left in, they make `H` singular. LM's damping hides that, but undamped
Gauss-Newton fails and the solution drifts. Triggs et al. (1999, §9) require them removed
**minimally**: fixing more biases the solution, fixing fewer leaves the covariance singular.

**What is already pinned** (`ba/gauge.py`). Fixed variables and priors pin some of the 7
directions. Each contributes rows of the derivative of what it pins with respect to `g`:
- a known position `X` (a fixed point, a fixed pose's centre, the centre of a pose with a
  translation prior): `dX = t + ω × X + s X`, the rows `[I, -[X]×, X]`;
- a known orientation (a fixed pose, a pose with a rotation prior): `[0, I, 0]`;
- metric depth (a depth prior with a fixed affine): `[0, 0, 1]`.

Positions on free poses are read at the current state. A prior on a pose nothing observes
pins nothing the scene can feel, so it does not count. The **remaining gauge** is the null
space N of these rows: anything from nothing (`none`) to all 7.

**In the scene's own frame.** `g` acts about the world origin. Far from it, the rotation and
scale directions shrink like `1/|X|`, so a rank test in world coordinates fails for
geo-referenced scenes (UTM, ECEF, a COLMAP model aligned to GNSS). The first version lost
the whole gauge from about 5e4 m: no pose fixed, σ_f infinite. The rows are therefore built
in coordinates centred on the observed cameras and anchor points and scaled by their RMS
spread. A change of origin and unit is an invertible map of `g`, so the spans, and the
gauge, are the same.

**And the solve in a centred world.** That fixed the rank test, not the steps. A step
`T_cw exp(δ)` rotates a camera about the world origin, so far from it a rotation step of θ
swings the centre by about `|θ|² |C| / 2`: half a metre per step at 1e4 m. Nothing in the
linear model sees that. From a start 0.05 off, the baseline the scale gauge holds drifted by
0.8, and LM ran from 9 iterations to its cap. At ECEF range (6.4e6 m), σ_f went infinite.
Both cores therefore solve in a world shifted by the mean centre of the observed cameras
(summed in index order, so they agree to the bit), and shift back. Only observed cameras
count: an unregistered camera left at the identity, at the world origin, first pulled the
centre a sixth of the way back to it, and the crawl returned. Every residual is unchanged by a shift, so
this is exact. Fixed and unmoved variables come back as given. Up to 6.4e6 m, the solve now
takes the same iterations, holds the same baseline and gives the same σ_f as at the origin.
Centring changes the steps' chart, so the gauge's second-order drift, and with it where in
the gauge orbit the solution lands, moved by up to 1e-4 on the fixture. The costs agree to
1e-13 and the intrinsics to 1e-9.

**Removing N, pose by pose.** At pose c, N moves the camera's world centre and orientation
by `M = [[I, -[C]×, C], [0, I, 0]] N`. A step moves them by `J_c δ`, where
`J_c = [[-I, [C]×], [0, -I]]` for a right perturbation of `T_cam_world` (from `C = -Rᵀt`,
`dC = -ρ + C × θ`). Confining the step to the complement of `J_cᵀ M` holds the centre and
orientation still along the gauge directions.
- A pose that sees all six of its directions is simply fixed. N then shrinks to the
  directions the pose does not see, and the next pose takes the rest.
- Greedy order: the pose seeing the most remaining directions goes first. Among poses that
  would be fixed, the lowest index; among the others, the one seeing its directions best
  (the largest smallest singular value of M).
- For the whole similarity this fixes the first observed pose. Scale about it remains, and
  the farthest pose is confined to keep its baseline: `J_cᵀ M` is then `a = (-b, b × C)`,
  the gradient of the baseline length. That is the classic Sim(3) gauge, and the code used
  it before it was generalised.
- Rotation-only priors leave translation and scale. A pose's orientation does not see them,
  so it can take at most 3, and two poses are confined.
- The bases are rebuilt at each linearisation, so the gauge holds to first order and drifts
  at second order (1e-4 relative on the baseline in the tests).
- **Both cores must take the same coordinates**, or their damped steps differ. The basis is
  built from the span of `J_cᵀ M` alone, never from an SVD's arbitrary basis: from the span's
  projector, repeatedly the column of the largest diagonal entry, deflated. Then one
  Householder reflection per vector onto its largest remaining axis, which is dropped.
  Near-ties (in that axis, or between candidate poses) go to the first index within 1e-12,
  so last-bit rounding cannot split the cores.

**Choices** (`BAOptions.gauge`):
- **`auto` (the default)** removes exactly N. The summary names what it was: `none`,
  `first_pose` (the rigid motions: metric depth alone), `scale` (a full prior on one camera,
  or one fixed pose), `similarity` (nothing anchored), or `partial` (anything else).
  Examples of `partial`: one fixed point leaves rotation and scale about it, two fixed points
  the rotation about their line, translation priors on two cameras the rotation about their
  baseline, attitude priors translation and scale.
- **`similarity`** counts only fixed poses as anchors.
- **`scale`** removes only the scale about a reference (the first fixed pose, else the first
  pose with a full prior).
- **`first_pose`** fixes the first observed pose unless one is fixed.
- **`none`** fixes nothing.

The summary records the fixed pose (`gauge_pose`), the first confined one (`confined_pose`),
and any directions no free pose could take (`free_gauge`). The last is 0 unless the
geometry is degenerate, for example the only free pose at the centre of a free scale.

The rule took three tries, each caught by the review panel.
1. "Any pose prior → `none`, metric depth → `first_pose`, else `similarity`". This froze a
   pose at its perturbed start on motion-only BA. With one pose prior it left scale free, and
   focal length read as unobservable.
2. Two yes/no flags, rigid motion pinned and scale pinned, mapped onto the four named gauges.
   Any partial anchoring fell through to fixing a whole pose, which biased the result. On
   noise-free data the cost stayed at 5 to 29 where `none` reached 1e-21. A prior on an
   unobserved camera also counted as the rigid anchor.
3. The null space, as above. The four partial cases now reach the noise-free optimum in
   both cores, and the fixture holds two of them. Run 3 then found two flaws underneath:
   - the translation prior's residual did not pin the centre the rows assumed (see
     Priors);
   - the rank test was in world coordinates, so it failed far from the origin.

   Both are fixed, with tests in both cores.

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
poses by the inverse. That alone assumes the weights are the true informations (inverse
noise variances). Weights are rarely that: unit weights on 0.5 px noise overstate σ by a
factor of 2. So by default (`scale_deviations`) the variances are multiplied by the
a-posteriori variance factor

    σ̂₀² = Σ wᵢ sᵢ / (m - n),

the weighted residual sum of squares over the redundancy (Förstner & Wrobel 2016, ch. 4).
- `sᵢ` are the blocks' squared weighted norms, and `wᵢ` their final IRLS weights. That is the
  numerator that pairs with the information `Jᵀ W J` the deviations come from. Under the
  trivial loss it is `2 cost`. Under a robust loss, `2 cost = Σ ρ(sᵢ)` would count each
  outlier by its `ρ`, which grows without bound (Huber) or slowly (Cauchy), and inflate σ̂₀².
  With 5% of pixels moved 20 px under Huber, σ_f came out 2.5x too large.
- m counts weighted residual rows: 2 per reprojection, 1 per depth prior, and 3 per weighted
  half of a pose prior. A translation-only prior's rotation rows carry nothing.
- n counts the free parameters.
- A prior on a pose that never moves (fixed, unobserved, or fixed by the gauge) is a
  constant. It is left out of the problem, like the views of a dropped point, so its cost
  inflates neither σ̂₀² nor the reported cost.

It is reported as `variance_factor`, near 1 when the weights match the noise, and NaN
without redundancy (m <= n), when nothing is scaled. A free group whose fx or fy deviation exceeds
`focal_std_ratio` (1%) of itself is counted in `unobservable_focal`.

The classic failure it catches is focal length against depth. With pure translation and no
rotation, `u = f (X - t_x)/Z` is unchanged by `f → kf, Z → kZ`, so no amount of data pins f.

## Readers, and Ceres as the reference

**BAL** (`slam_it_all.eval.readers.read_bal`). BAL's cameras look down `-z` with `y` up:
`P = R X + t`, `p = -P/P_z`, `u = f (1 + k1|p|² + k2|p|⁴) p`.
- **Frames.** Flipping the camera's `y` and `z` axes (`F = diag(1, -1, -1)`) gives
  `T_cam_world = [F R, F t]` and pixels `(x, -y)`.
- **Camera model.** BAL's camera is exactly the `radial` model `[f, cx, cy, k1, k2]`, added
  for it. It is radtan with one focal length and no tangential terms, computed through
  radtan by a fixed 9x5 map.
- **What is refined.** BAL and Ceres refine `f, k1, k2` but not the principal point, which
  needed per-parameter intrinsics masks: `fixed_intrinsics` is `(K, P)`.

**COLMAP text** (`slam_it_all.eval.readers.read_colmap_text`). Malformed lines and dangling ids
raise `InvalidInputError` naming the file and line.
- **Pixels.** COLMAP puts pixel centres at `(0.5, 0.5)`, so the reader shifts pixels and
  principal points by -0.5.
- **Models.** SIMPLE_PINHOLE, SIMPLE_RADIAL and RADIAL map onto `radial`; PINHOLE onto
  `pinhole`; PINHOLE and OPENCV onto `radtan`. Absent terms are zero, and `refinable` marks
  the ones the source model has.

**The reference.** `slam_it_all.eval.colmap_reference` builds a pycolmap reconstruction from a
problem and runs COLMAP's bundle adjuster, which is Ceres underneath. pyceres was the plan,
but its reprojection factors are gone from current releases. COLMAP's adjuster is the same
Ceres, on the same RADIAL model, and is also what reifyscene's reconstructions come from.

What COLMAP refines is not a separate option: it follows the problem's own
`fixed_intrinsics` (`colmap_reference.refinement`). Anything COLMAP's adjuster cannot
represent is rejected rather than silently dropped: priors, anchors, observation weights,
fixed poses or points, a free principal point, or a per-group mix of free and fixed
entries. So the comparison is always on the identical problem.

**On BAL Ladybug-49** (49 cameras, 7776 points, 31843 observations; 31 behind their camera):

| Solver | RMS reprojection error | Time |
|---|---|---|
| start | 7.314 px | |
| SLAM It All!, Python | 0.915 px | ~60 s |
| SLAM It All!, C++ (dense reduced solve) | 0.9147 px | ~2 s |
| COLMAP's Ceres | 0.9147 px | 0.4 s |

The dense reduced solve is the gap. Milestone 4 brings the sparse one, and the
`ReducedSystemSolver` seam is where it plugs in. `scripts/demo_bal.py` reproduces the table
and, with `--rerun`, saves a recording of the cameras and points before and after.

## Across reifyscene's investigation slice

The investigation slice is ten DAVIS scenes plus spin (`runs/slice_*`, rebuilt
2026-09-30). For each DAVIS scene, `scripts/compare_ba_colmap.py` reads the seed reconstruction
reifyscene's `pose_refine` stage exports: VGGSfM tracks and VGGT cameras, one shared PINHOLE
camera, before reifyscene's own BA. It refines the seed with SLAM It All! and with COLMAP's
Ceres, on the identical problem:
- focal lengths free, principal point fixed;
- trivial loss;
- a 300-iteration cap for both solvers.

spin has no seed: its run takes another route through pose_refine. It is listed rather than
skipped. Measured on 2026-10-02 at this commit:

| Scene | Start | ours | COLMAP | Pushed behind (ours / COLMAP) | σ̂₀ | σ_f (ours) | f ours / COLMAP |
|---|---|---|---|---|---|---|---|
| lab-coat | 4.854 | 0.880 | 0.910 | 0 / 7 | 0.68 | **2.3** | 483 / 504 |
| blackswan | 5.267 | 0.518 | 0.522 | 0 / 0 | 0.41 | 26 | 1191 / 1848 |
| crossing | 3.260 | 0.358 | 0.351 | 0 / 0 | 0.27 | 528 | 4362 / 3886 |
| dogs-scale | 4.467 | 0.807 | 0.812 | 0 / 30 | 0.63 | 5.3 | 524 / 758 |
| gold-fish | 3.616 | 0.672 | 0.674 | 0 / 0 | 0.54 | inf | 2874 / 3217 |
| india | 5.314 | 0.489 | 0.574 | 0 / 121 | 0.39 | inf | 1521 / 1515 |
| judo | 3.405 | 0.364 | 0.362 | 0 / 542 | 0.27 | 241 | 2451 / 5422 |
| lindy-hop | 4.033 | 0.680 | 0.680 | 0 / 471 | 0.55 | 51 | 1726 / 1771 |
| miami-surf | 4.881 | 0.657 | 0.657 | 0 / 0 | 0.53 | 2333 | 9854 / 10322 |
| motocross-bumps | 5.437 | 0.384 | 0.390 | 0 / 4 | 0.34 | inf | 898 / 909 |

RMS is in px, over the observations in front at the start and in both results. σ̂₀ is the
a-posteriori noise (px), and σ_f is scaled by it (see
[Focal observability](#focal-observability)). The table comes from this invocation. The
comparison script is generic, and the slice and its run layout belong to reifyscene, so they
appear only here:

```
RUNS=$REIFYSCENE_DATA_DIR/runs
python scripts/compare_ba_colmap.py --iterations 300 $(for s in lab-coat blackswan crossing \
  dogs-scale gold-fish india judo lindy-hop miami-surf motocross-bumps spin; do
  echo "$s=$RUNS/slice_$s/pose_refine/colmap_seed"; done)
```

What the slice shows that lab-coat alone would not:
- **The two solvers agree on reprojection error.** Within 0.01 px on 8 of 10 scenes. Ours is
  lower on lab-coat (0.03 px) and india (0.08 px), where COLMAP pushed points behind their
  cameras.
- **Focal length is unobservable on 9 of the 10 scenes.** Only lab-coat pins it (σ_f = 2.3
  px, 0.5%). Elsewhere σ_f runs from 5.3 px (dogs-scale, 1.0%, at the threshold) to
  infinite: DAVIS clips are short handheld moves with little rotation, so focal length and
  depth trade off. The focal lengths wander with the
  iteration count, from 1341 to 1191 on blackswan between 100 and 300 iterations. The two
  solvers land far apart (2451 against 5422 on judo) at essentially the same cost. Five
  scenes never converge within 300 iterations, for the same reason. Judged on lab-coat alone,
  freeing the focal length would have looked sound.
- **COLMAP lets points cross behind their cameras.** It did so for up to 542 observations
  (judo); Ceres has no cheirality guard. Ours rejects such steps.
- **Speed.** The first dense Schur build formed one small product per *pair* of a point's
  blocks, each heap-allocated: quadratic in track length, 95% of the run time on judo
  (34-view tracks). It now merges each point's blocks once per linearisation into one
  coupling over the point's distinct columns (they share their camera's intrinsics), does one
  product per point, and scatters only the upper triangle, column by column, in memory
  order. The results are identical (same RMS, focal lengths and iterations); the slice's run
  times fell 2.3–5.1x:

  | Scene | Before (s) | After (s) | COLMAP (s) |
  |---|---|---|---|
  | lab-coat | 20.3 | 6.7 | 4.9 |
  | blackswan | 50.3 | 15.0 | 12.7 |
  | crossing | 75.9 | 18.2 | 9.2 |
  | dogs-scale | 54.8 | 19.0 | 5.1 |
  | gold-fish | 59.4 | 21.9 | 11.9 |
  | india | 13.3 | 5.5 | 0.4 |
  | judo | 564.7 | 110.9 | 36.4 |
  | lindy-hop | 90.6 | 29.2 | 15.1 |
  | miami-surf | 34.6 | 15.2 | 8.0 |
  | motocross-bumps | 6.3 | 3.2 | 1.3 |

  The remaining gap (1.2–4x, 14x on india) is not measured. It is not simply dense against
  sparse: COLMAP solves with Ceres' `DENSE_SCHUR` up to 50 images, `SPARSE_SCHUR` up to
  1000 and `ITERATIVE_SCHUR` beyond. So on lab-coat (47 images) and judo (34) both solvers
  factor densely. Ceres' threads and cheaper Jacobian evaluation are the likelier causes
  there; above 50 images sparsity adds to them. Milestone 4's sparse reduced solver
  addresses the second, not the first.

This is the critical-motion degeneracy of self-calibration. Sturm (1997) classifies the
camera motions under which constant intrinsics are not determined, and Kahl, Triggs & Åström
(2000) extend the analysis to intrinsics that may vary. Translation with little rotation is the
textbook case for focal length, and a short handheld clip is close to it.

For reifyscene: its `pose_refine` frees the focal length on exactly these scenes. The slice
suggests that is unconstrained on 9 of 10, so milestone 3 should hold focal fixed (from VGGT)
unless `unobservable_focal` is 0.

## Toward larger problems

The reduced camera system is solved densely: O(C³) per step, fine for tens of cameras. For
milestone 4's hundreds to thousands, the options behind `ReducedSystemSolver` are:
- **Sparse Cholesky of the reduced system.** Cameras that share no point leave zeros in `S`.
  This is Ceres' `SPARSE_SCHUR`, which COLMAP runs between 50 and 1000 images.
- **Inexact Newton with preconditioned CG.** CG on `S` never forms its factor. The
  products `S v` can even be computed from the blocks without forming `S`. Agarwal, Snavely,
  Seitz & Szeliski (2010) compare the preconditioners, block-Jacobi on the camera blocks
  among them, on the BAL problems the reader here loads.
- **Square-root BA.** Demmel, Sommer, Cremers & Usenko (2021) marginalise the points by QR
  (a nullspace projection) instead of the Schur complement. That keeps the system's
  conditioning, so single precision suffices.
- **Power BA.** Weber, Demmel, Chan & Cremers (2023) expand the inverse Schur complement
  as a power series. In their words, it challenges "the state-of-the-art iterative
  methods" on BAL.

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
  - Partial anchoring, in both cores: one fixed point, two fixed points, translation priors
    on two cameras, and attitude priors all resolve to `partial` and reach the noise-free
    optimum (cost under 1e-18).
  - A prior on a camera nothing observes changes nothing: the same gauge and deviations as
    without it.
  - An explicit `scale` gauge with no reference is rejected by both cores.
  - The options reject `partial`, which is reported only.
  - Translation priors whose stored rotation is identity give the same solution as with the
    true rotation. Three of them leave no gauge, and σ_f is finite.
  - Scenes 1e4, 1e6 and 6.4e6 m from the origin, started 0.05 off, get the same gauge
    pose, confined pose, iterations (within 1), camera centres (to 1e-6 m) and σ_f (to
    1e-5) as at the origin.
  - `free_gauge` counts what a fixed recipe leaves: 1 for `first_pose` on monocular data
    (the scale), 7 for `none` with nothing pinned, 0 under `auto`.
  - Unit tests of the gauge in both cores:
    - the remaining dimension and name for each kind of anchor;
    - the complement's orthonormality;
    - the plan's choices, including a direction no pose can take (counted in `free_gauge`)
      and a 1e6 m offset.
- **Degeneracy.** A single-view point, a point 10⁷ away and an unobserved camera are
  dropped or held, counted, and returned unchanged.
- **Parity.** A committed fixture, `tests/fixtures/foundations/ba.json`, holds fifteen
  solves by the reference (`generate_ba.py`):
  - each gauge `auto` resolves to;
  - each explicit recipe (`first_pose`, `scale`, `none`);
  - two partial cases: attitude priors, which confine two poses, and GNSS-style translation
    priors on two cameras, stored with identity rotation;
  - Gauss-Newton, Huber with outliers, degenerate drops, radial with f, k1, k2 free, a
    scene with every kind of block, and a pose prior 1.2 rad from the truth (where `Jr⁻¹`
    is far from the identity).

  Each case records every option it ran with. Each backend must reproduce its outputs under
  per-field tolerances (poses to 1e-9, points to 1e-8, intrinsics to 1e-8 relative,
  deviations to 1e-6 relative) and its termination, iterations, gauge and counts exactly. There is no closed-form optimum to oracle against,
  so the fixture pins the reference, whose correctness the other checks here establish.
  A live test also compares C++ against the reference with radtan, two intrinsics groups
  and a Cauchy loss. On the smoke scene they agree to 1e-15.
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
  | 0.02 rad | 8 px (1.7%) | yes |
  | none | about 700 px | yes |

  The deviations are scaled: σ̂₀² is near 0.25 for the 0.5 px noise under unit weights, and
  the same scene with weights 1/0.5² gives the same deviations. Unscaled, they are the
  scaled ones over σ̂₀, in both backends. With 15% of pixels moved 20 px under Cauchy,
  σ̂₀² stays between 0.15 and 0.35. Translation-only priors add 3 rows each to the
  redundancy, not 6.

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
  calibration needs varied views. That 9 px was itself twice too large: it took unit weights
  as the information of 0.5 px noise. Scaled by the variance factor it is 4.3 px (0.9%), and
  the weak case moved to 0.02 rad.
- **A triangulation heuristic is not a BA rule.** The first default dropped points with less
  than 1° of parallax. On BAL's Ladybug-49, that left out 37 points (239 observations), which
  kept their starting positions, and the RMS error ended at 1.536 px against COLMAP's
  0.915 px. With the filter off, ours reaches 0.9147 px, the same as COLMAP. COLMAP applies
  its 1.5° check when it *creates* points; bundle adjustment then refines low-parallax points
  well, since damping handles their weak depth. The filter is now off by default.
- **Fixed means bit-exact.** The first version returned fixed poses 1 ulp off, from the
  round trip `T_world_cam → T_cam_world → T_world_cam`. Unmoved poses are now returned
  as given, in both cores.
- **A slow crawl can be a residual's fault, not the problem's.** With translation-only
  priors at weight 1e2 to 1e4, LM took 40 to 100 iterations, or ran to the cap, with no gauge
  as well. I first wrote that down as a property of the geometry. The reviewer suspected the
  residual, and was right. Its translation half pinned the rotation about the world origin
  only at second order: a near-null direction, which is exactly what makes LM crawl and
  makes rounding grow between the cores (they ended 1e-3 apart). With the decoupled residual
  the same problems converge in about 10 iterations at weights 1e2 to 1e4, and the cores
  agree to 3e-15. Weak priors (weight 1) still take 20 to 30, against 6 under Gauss-Newton:
  a softly pinned direction has curvature far below Marquardt's `λ diag(JᵀJ)`, so the
  damping shrinks its steps until λ decays. That is LM's own behaviour, not a defect.
- **A fixture's tolerance must leave the cores room to stop together.** At a cost tolerance
  of 1e-12, a cost creeping down by about that much per step let last-bit rounding decide
  which step crosses it. On one case Python stopped after 9 iterations and C++ after 12, at
  the same optimum. The fixture solves to 1e-10.
- **σ_f on a weakly conditioned scene moves with rounding order.** On lab-coat, one
  algorithm gave σ_f = 3.41 and 3.53 px before and after the Schur speedup changed the
  summation order. Later 2.32 and 2.29 px after scaling, with identical results otherwise.
  Read it to two figures; the 1% rule's verdicts did not change.

## References

- Agarwal, Snavely, Seitz & Szeliski (2010). Bundle adjustment in the large. ECCV.
- Civera, Davison & Montiel (2008). Inverse depth parametrization for monocular SLAM. IEEE
  Transactions on Robotics.
- Demmel, Sommer, Cremers & Usenko (2021). Square root bundle adjustment for large-scale
  reconstruction. CVPR.
- Engel, Koltun & Cremers (2018). Direct sparse odometry. IEEE TPAMI.
- Förstner & Wrobel (2016). Photogrammetric Computer Vision. Springer.
- Kahl, Triggs & Åström (2000). Critical motions for auto-calibration when some intrinsic
  parameters can vary. Journal of Mathematical Imaging and Vision.
- Sturm (1997). Critical motion sequences for monocular self-calibration and uncalibrated
  Euclidean reconstruction. CVPR.
- Triggs, McLauchlan, Hartley & Fitzgibbon (1999). Bundle adjustment -- a modern synthesis.
  Vision Algorithms: Theory and Practice.
- Weber, Demmel, Chan & Cremers (2023). Power bundle adjustment for large-scale 3D
  reconstruction. CVPR.
