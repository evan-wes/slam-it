# Absolute pose: P3P, EPnP, the pose refinement and PnP RANSAC

Design note for `slamlab.foundations.p3p` and `slamlab.foundations.pnp` (the Python
references), and `slamlab/foundations/p3p.hpp` and `pnp.hpp` (the C++ core). The two-view and
five-point notes cover relative pose; this is its counterpart, a frame placed against known
3D points.

## The problem

Two-view geometry starts a map: two frames, their relative motion, the first points. Every
frame after that is placed against the map: its matches to known 3D points give its pose.
That is **absolute pose**, or PnP (perspective-n-point): given world points `Xᵢ` and the
bearings `yᵢ` along which the camera sees them, find `T_cam_world = [R | t]` with
`λᵢ yᵢ = R Xᵢ + t` for depths `λᵢ > 0`. A tracker runs it on every frame, and a relocaliser
runs it when tracking is lost.

The pieces mirror E's:

- a **minimal solver** for RANSAC's samples: three points give at most four poses (P3P);
- a **least-squares solver** for LO-RANSAC's refits, from many points (EPnP);
- a **refinement** on the geometric error (the reprojection error), from RANSAC's consensus;
- **RANSAC** around them.

## The interface

| Function | Does |
|---|---|
| `p3p.p3p(points, bearings, solver=)` | every pose three correspondences allow, (K, 4, 4), K ≤ 4 |
| `pnp.epnp(points, bearings)` | EPnP's pose from five or more correspondences |
| `pnp.refine_pose(pose, points, bearings, scale=)` | the pose refined on the robust reprojection error; a `Solution` with `nls.Summary` |
| `pnp.pnp_ransac(points, bearings, options, minimal_solver=, refine=)` | LO-MSAC with P3P and EPnP refits, then the refinement; a `ransac.RefinedSummary` |
| `pnp.reprojection_errors(pose, points, bearings)` | squared errors in the normalised plane; infinity behind the camera |

`solver` and `minimal_solver` are `"ding"` (the default) or `"lambda_twist"`; `refine`
defaults to on. Thresholds and the Cauchy scale are in the normalised image plane (pixels over
the focal length), as for E. The C++ core implements all of it.

`pnp_ransac` returns the same summary type as `essential_ransac`: `ransac.RefinedSummary`, the
RANSAC run nesting the refinement's solve, with whether the refined model was kept. What the
two share, LO-MSAC then "refine the consensus, keep it only if the MSAC cost drops", is generic
in `ransac` (`refine_consensus`, `estimate_refined`), in both languages. Each refinement
problem stays with its model: the Sampson problem in `epipolar`, the reprojection problem in
`pnp`. This settles PR 3's deferred A6 (where refinements live).

## The maths

**P3P.** Rigid motion keeps distances, so each pair of points gives an equation in the depths
alone: `λᵢ² − 2 bᵢⱼ λᵢ λⱼ + λⱼ² = aᵢⱼ`, with `bᵢⱼ = yᵢ·yⱼ` and `aᵢⱼ = |Xᵢ − Xⱼ|²`. Dividing one
by another removes the scale and leaves two **conics** (quadratic curves in two unknowns),
whose intersections, at most four, are the solutions. Both solvers intersect them the same
way:

1. Every combination `C₁ + s C₂` passes through the intersections, and for the roots of a
   cubic, `det(C₁ + s C₂) = 0`, it is **a pair of lines**. One real root is enough, in closed
   form (Cardano or Viète), then polished by Newton's method.
2. Each line meets a conic in a **quadratic**, whose positive roots are depth ratios; one
   distance equation fixes the scale.
3. The depths are **polished** by Newton's method on the three distance equations.
4. The pose is the **rigid alignment** of the world triangle to the camera's `λᵢ yᵢ`.

The classic route, a quartic in one depth ratio (Grunert 1841, Kneip et al. 2011), is slower
and less stable.

- **Lambda Twist** (Persson and Nordberg, ECCV 2018) works on the depths themselves and splits
  the line pair with an eigen-decomposition: the degenerate conic has a zero eigenvalue, so
  `Λᵀ D₀ Λ = σ₁ (e₁·Λ)² + σ₂ (e₂·Λ)²`, which vanishes on `(e₁ ± √(−σ₂/σ₁) e₂)·Λ = 0`. It uses the
  cubic's largest root.
- **Ding et al.** (CVPR 2023) works on depth ratios, after reordering the points so that the
  reference distance is the triangle's longest side, which keeps the conics' coefficients
  bounded. It splits the line pair without an eigen-decomposition: for `C = p qᵀ + q pᵀ`,
  `−adj(C) = w wᵀ` with `w = p × q`, so `C + [w]ₓ = 2 q pᵀ` has rank one and its column and row
  are the lines. When the cubic has a single real root, the two real intersections lie on one
  line (the other holds a complex pair), so the second line is skipped once the first gave
  solutions.

Both follow PoseLib's implementations, with four departures:
- polishing to convergence, not a fixed step count;
- splitting the line pair on its largest entry;
- the pose by alignment rather than the papers' `R = Y X⁻¹`;
- dropping candidates whose depths cannot satisfy the distance equations.

The last two came from real data (below).

**EPnP** (Lepetit, Moreno-Noguer and Fua, IJCV 2009) writes each world point as a weighted sum
of four control points, `Xᵢ = Σⱼ αᵢⱼ cⱼ`: the centroid and the three principal axes, scaled
by the points' spread. The unknowns are then the control points in the camera, twelve numbers.
Each correspondence gives two linear equations, `M x = 0`, so `x = Σₖ βₖ vₖ` over `MᵀM`'s
smallest eigenvectors. The β come from keeping the control points' six distances, quadratic
equations linearised three ways (OpenCV's "approximations") and each polished by Gauss-Newton.
Each gives camera points, and their rigid alignment to the world points gives a pose; the one
with the lowest reprojection error wins. Eigenvectors' signs are fixed (largest entry
positive) so LAPACK and Eigen build the same basis.

**The refinement** is Levenberg-Marquardt (`foundations.nls`) on SE(3), minimising
`½ Σ ρ(|π(R Xᵢ + t) − xᵢ|²)` with ρ Cauchy's loss, scale the RANSAC threshold. The step is
`T exp(δ)` with `δ = (ρ, θ)`, so the camera point moves by `R (ρ − [X]ₓ θ)` to first order and
the Jacobian is `∂π/∂p · [R | −R [X]ₓ]`.

**RANSAC** samples three correspondences, scores by MSAC on the squared reprojection error
(infinite behind the camera, so such a point is never an inlier), refits EPnP inside LO, and
refines the consensus.

## How it's checked

- **Fixtures** (`generate_pose.py`, 50-digit mpmath, nothing imported from slamlab):
  - P3P: the true pose among the solutions, on generic samples, each of Ding's three
    orderings, and 1e-2 and 1e-6 inside the danger cylinder. The 1e-6 case catches a solver
    that merges a quadratic's close roots.
  - EPnP: exact on 5 and 20 points.
  - The refinement: from a perturbed start to the truth.
  - RANSAC: on noise-free inliers and far outliers, the true pose and exactly the clean
    correspondences, for each solver, refined or not, with whether the refinement ran.
- **Oracles.**
  - PoseLib's P3P solutions are among ours, and the counts agree.
  - OpenCV's EPnP has the same accuracy as ours under noise.
  - SciPy's `least_squares` reaches the same minimum as the refinement, given one residual per
    correspondence so the robust loss sees the same quantity.
  - The analytic Jacobian matches finite differences.
- **Properties.**
  - Every P3P solution is a rigid motion putting each point in front, along its bearing.
  - The two P3P solvers find the same poses.
  - EPnP is exact on exact data from five points.
  - Planar points, four points, collinear points and coinciding bearings are refused or give
    no pose.
- **Backends.** Python and C++ return the same P3P poses in the same order (1e-10), and PnP
  RANSAC draws identical runs (inliers, iterations, refinement kept).
- **Native C++ tests** for both modules.

## Measured

**Near degeneracy** (`p3p_degeneracy_probe.py`). This counts how
often the true pose is missing from the solutions on exact samples, for 2000 random triangles
with the camera a relative margin inside the **danger cylinder**. The cylinder stands over the
triangle's circumcircle; on it, two of the four solutions coincide.

| Inside the cylinder by | Ding | Lambda Twist | PoseLib (Ding's) |
|---|---|---|---|
| 1e-2 | 3 | 8 | 6 |
| 1e-3 | 11 | 34 | 15 |
| 1e-4 | 36 | 85 | 28 |
| 1e-6 | 141 | 237 | 133 |

That is why Ding's is the default. Far away (a 1 m triangle at 1000 m) every solver loses
about 18%, to the conditioning, not the method.

**EPnP's minimum.** On exact four-point samples, the exact pose comes back on 38% of samples
(OpenCV's EPnP: 12%). With four points `MᵀM`'s null space is four-dimensional and the
linearisations are approximate. From five points it is exact on every sample, so five is the
minimum.

**Synthetic** (`pnp_sweep.py`): 40 scenes, 200 points, 1 px noise
at f = 520, 40% outliers, 2 px threshold. The error is the rotation error over the floor's,
the floor being the refinement started at the true pose on the true inliers. Times are the
C++ backend's, per RANSAC run.

| Configuration | Error / floor (median, p90) | Samples | ms |
|---|---|---|---|
| Ding, refined | 1.21, 1.94 | 47 | 1.2 |
| Lambda Twist, refined | 1.21, 1.94 | 47 | 1.1 |
| Ding, unrefined | 1.55, 3.07 | 47 | 1.1 |
| OpenCV `solvePnPRansac` (AP3P) + `solvePnPRefineLM` | 2.21, 5.38 | | 1.6 |

The refinement closes most of the gap to the floor. OpenCV's least-squares refinement is not
robust, so the inliers' tail pulls it.

**TUM RGB-D** (`scripts/demo_absolute_pose.py --survey`). Reference frames are taken every 40
frames, and each places the frames 15 and 30 later. The points come either from the reference
frame's depth map (metric, independent of slamlab) or from two-view initialisation with the
frame 30 later, scaled by the true baseline. A placement needs 20 inliers, as a tracker would
require. Errors are medians against the motion capture, rotation then camera position.

| Sequence | Points | Placed / rejected | Ding, refined | Unrefined | OpenCV |
|---|---|---|---|---|---|
| fr1/xyz | depth | 29 / 8 | 0.78°, 1.26 cm | 0.74°, 1.43 cm | 0.74°, 1.16 cm |
| fr1/xyz | two-view | 24 / 0 | 0.75°, 0.86 cm | 0.69°, 0.92 cm | 0.77°, 0.80 cm |
| fr1/desk | depth | 17 / 9 | 1.16°, 1.93 cm | 1.15°, 1.79 cm | 1.17°, 2.26 cm |
| fr1/desk | two-view | 8 / 0 | 1.06°, 1.75 cm | 1.13°, 1.69 cm | 1.06°, 1.51 cm |
| fr3/long_office_household | depth | 117 / 3 | 0.31°, 0.65 cm | 0.35°, 0.78 cm | 0.32°, 0.65 cm |
| fr3/long_office_household | two-view | 122 / 0 | 0.39°, 0.73 cm | 0.42°, 0.79 cm | 0.40°, 0.78 cm |

- On real data slamlab and OpenCV agree to within the data's noise. The errors are the data's
  (rolling shutter, blur, depth noise, motion-capture sync), not the solvers'. Lambda Twist
  gives the same results as Ding's (the same consensus).
- The refinement helps where the data are good (fr3) and is neutral on fr1, whose errors are
  dominated by the sensor.
- The rejected frames are matching failures, not solver failures. On fr1's fast, blurred
  stretch, the ground-truth pose explains under 5% of some frames' matches; RANSAC then finds
  3 to 7 inliers, and the gate rejects the pose.
- fr1/desk's two-view row is small: 22 of 30 references do not initialise. Some of those
  references crash `twoview.initialize` (see the open items).

## What was learned

- **Real data finds what synthetic data does not.** Two P3P bugs survived every synthetic
  test, including the oracles, and surfaced in the first TUM survey. SIFT detects several
  keypoints at one pixel (one per orientation), so RANSAC samples triples with coinciding
  bearings.
  - With two coinciding bearings, the scale's denominator `τ² − 2bτ + 1` (whose minimum
    `1 − b²` is zero) rounded negative: Python raised, and C++ produced a NaN.
  - With three coinciding bearings and points that are not collinear, no pose exists. The
    candidates' depths cannot satisfy the distance equations, and the closed form `Y X⁻¹`
    returned a matrix 0.97 from a rotation, which RANSAC then scored.

  Both are fixed. The scale is guarded, and the pose is the rigid alignment, so every pose
  returned is a rotation. Candidates whose polished depths miss the distance equations by more
  than 1e-6 of the largest squared distance are dropped (solvable samples reach about 1e-15).
  The captured triple is a regression test.
- **"Double root" is a tolerance, and the wrong one merges true solutions.** Near the danger
  cylinder two true poses can be 1e-6 apart. A discriminant treated as zero below a relative
  1e-12 merged them into one pose between the two, 4e-7 from each. Only a slightly negative
  discriminant is a double root.
- **A minimal set is not always enough for a least-squares solver.** EPnP's paper allows four
  points, but its linearisations are approximate there; measuring showed it.
- **Oracles need the same problem, not just the same name.** SciPy's robust loss applies per
  residual component, ours per correspondence. The first comparison disagreed by 5% for that
  reason alone.

## References

- Y. Ding, J. Yang, V. Larsson, C. Olsson and K. Åström, "Revisiting the P3P Problem", CVPR
  2023.
- M. Persson and K. Nordberg, "Lambda Twist: An Accurate Fast Robust Perspective Three Point
  (P3P) Solver", ECCV 2018.
- V. Lepetit, F. Moreno-Noguer and P. Fua, "EPnP: An Accurate O(n) Solution to the PnP
  Problem", IJCV 81(2), 2009.
- J. A. Grunert, "Das Pothenotische Problem in erweiterter Gestalt", 1841; R. Haralick et al.,
  "Review and analysis of solutions of the three point perspective pose estimation problem",
  IJCV 13(3), 1994.
- L. Kneip, D. Scaramuzza and R. Siegwart, "A novel parametrization of the
  perspective-three-point problem", CVPR 2011.
- T. Collins and A. Bartoli, "Infinitesimal Plane-Based Pose Estimation", IJCV 109(3), 2014
  (IPPE, the planar solver EPnP is not).
- V. Larsson et al., PoseLib (github.com/PoseLib/PoseLib), the implementations followed and
  the test oracle; OpenCV's `epnp.cpp`.
- J. Sturm et al., "A benchmark for the evaluation of RGB-D SLAM systems", IROS 2012.
