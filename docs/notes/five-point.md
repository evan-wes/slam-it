# The 5-point solver and the Sampson refinement

Design note for `slamlab.foundations.fivepoint` and the E-RANSAC options and refinement in
`slamlab.foundations.epipolar` (the Python references), and `slamlab/foundations/fivepoint.hpp`
and `epipolar.hpp` (the C++ core). The two-view note covers the 8-point algorithm, E-RANSAC's
loop and the initialiser that uses them.

## The problem

PR 2's E-RANSAC used the 8-point algorithm as its minimal solver, and the two-view note
recorded two weaknesses.

- **The sample is large.** An all-inlier sample has probability `w⁸` for an inlier ratio `w`;
  with 40% outliers that is 1.7%, so RANSAC draws hundreds of samples.
- **Its models are poor and its refits stop short.** Eight noisy matches give a barely
  constrained E, projected onto the essential manifold after the fact. Local optimisation's
  8-point refits rescue the result, but they minimise an algebraic error, so LO-RANSAC stops
  at the 8-point fit to its inliers: the "floor".

E has five degrees of freedom, so five matches determine it up to a finite set. A **minimal
solver** that uses the essential constraints exactly makes an all-inlier sample likelier
(`w⁵`: 7.8% at 40% outliers) and gives better models from it. A **refinement** that minimises
the geometric (Sampson) error, not an algebraic one, can go below the floor.

## The interface

| Function | Does |
|---|---|
| `fivepoint.essential_5pt(b1, b2, solver=)` | every essential matrix five bearing pairs allow, (K, 3, 3), K ≤ 10 |
| `fivepoint.real_roots(poly)` | a polynomial's distinct real roots, by Sturm sequences and bisection |
| `epipolar.essential_ransac(b1, b2, options, minimal_solver=, refine=)` | LO-MSAC with a chosen minimal solver, then the refinement |
| `epipolar.refine_essential(E, b1, b2, scale=)` | E refined on inlier pairs' robust Sampson distance; a `Solution` with `nls.Summary` |
| `twoview.Options(essential_solver=, refine=)` | the same two choices for the initialiser |

`solver` is `"nister"` (the default) or `"stewenius"`; `minimal_solver` and `essential_solver`
also accept `"8pt"`, PR 2's behaviour. `refine` defaults to on. The C++ core implements all of
it; the options cross the bindings as codes (`codes()["minimal_solver"]`) or, inside
`twoview.Options`' dict, by name.

## The maths

**The shared front end.** Each bearing pair gives `f₂ᵀ E f₁ = 0`, linear in E's nine entries.
Five give a 5x9 system with a four-dimensional null space, `E = x X + y Y + z Z + W`. E must
also satisfy Demazure's cubic constraints, `det E = 0` and `2 E Eᵀ E − tr(E Eᵀ) E = 0`; with E
substituted they are ten cubic equations in `x, y, z`, a 10x20 coefficient matrix over the
monomials of degree up to three.

**Nistér (2004).** In Nistér's monomial order, Gauss-Jordan elimination leaves six rows that
pair up into a 3x3 matrix `B(z)` of polynomials, which must be singular. `det B(z)` has degree
ten. Its real roots are isolated by a Sturm sequence and bisection, and `(x, y)` is B's null
vector at each root. Forming the determinant loses digits on some samples, so each solution is
**polished** by three Gauss-Newton steps on the ten original cubics. On 300 exact random
problems the worst error in E fell from 7e-4 to 2e-11.

**Stewénius, Engels and Nistér (2006).** In graded reverse lexicographic order, elimination
expresses the ten cubic monomials in the ten lower ones. That is the **action matrix** of
multiplication by `x`, whose eigenvectors are the solutions (a Gröbner-basis method). One
10x10 eigen-decomposition replaces the polynomial and its roots. It is accurate to about
1e-9 without polishing, and is a third faster than Nistér in C++.

**Only essential matrices are returned.** With no baseline every `[t]ₓ R` fits, and the cubics
have a continuum of solutions instead of ten points. The solvers' roots are then arbitrary,
and some do not give an essential matrix at all: singular values like (1.19, 0.71, 0.30) were
seen. A solution is kept only if, scaled to norm √2, `σ₁ − σ₂` and `σ₃` are below
`ESSENTIAL_TOLERANCE = 1e-6`. That is far above the solvers' precision on well-posed samples
and far below any junk. What survives on a pure rotation fits the sample with an arbitrary t;
RANSAC's scoring and two-view's homography deal with that case.

**The Sampson refinement.** The state is the motion `(R, t)` with `|t| = 1`, and
`E = [t]ₓ R`. The residuals are each inlier's **signed** Sampson distance,
`r = x₂ᵀ E x₁ / √g` with `g` the sum of the epipolar lines' first two coordinates squared. The
problem is solved by Levenberg-Marquardt (`foundations.nls`) with a Cauchy loss whose scale is
RANSAC's threshold, so the consensus's few remaining outliers barely pull.

- **Steps** live on E's five-dimensional manifold. Rotation is updated as `R exp(δθ)`. The
  translation moves in its tangent plane and is renormalised, `t ← normalise(t + B δt)`, with
  B an orthonormal basis of the plane at t. B is built from the coordinate axis least aligned
  with t, so every backend picks the same one.
- **The Jacobian** is analytic, by the chain rule. `∂r/∂E` comes from the quotient rule on
  `c/√g`. The directions of E along the five coordinates are `[t]ₓ R [eₖ]ₓ` for rotation and
  `[bⱼ]ₓ R` for translation. A test checks the resulting gradient against central
  differences.
- **The start** is one of E's four decomposed poses. They all give ±E, but each backend's SVD
  orders and signs them differently, and LM's path depends on the start. The rule: the
  rotation with the largest trace, and the translation sign that makes its largest component
  positive. Before this rule Python and C++ agreed to only 4e-5; with it they agree to 1e-15.
- **The solve** stops after at most 20 iterations (`REFINE_ITERATIONS`). From RANSAC's
  consensus it takes 5 to 8.

`essential_ransac(refine=True)` refines the consensus on its inliers. It keeps the result only
if the MSAC cost over all matches is lower, then recomputes the inliers and score
(`polish_consensus`). A refinement that makes things worse is never kept.

## How it's checked

- **The true E is among the solutions.** On 50 random exact problems, for both solvers and
  both backends, one solution matches the truth to 1e-8. Every solution has singular values
  (1, 1, 0) and satisfies the five equations to 1e-9.
- **The two solvers agree.** Nistér and Stewénius return the same number of solutions, equal
  to 1e-7, and the C++ core matches the reference.
- **PoseLib as an oracle.** Every PoseLib `essential_matrix_5pt` solution that satisfies the
  cubic constraint is one of ours, to 1e-8. The filter must be on the cubic constraint, not
  the epipolar residuals. Any matrix in the null space satisfies the five equations to 1e-17,
  so a residual filter passed a PoseLib matrix with `σ₃ = 3.6e-4`, which looked like a root
  we had missed.
- **The refinement.**
  - The gradient matches finite differences.
  - From a perturbed start on exact data, both backends converge to the true E (cost
    1e-20).
  - From RANSAC's consensus, SciPy's `least_squares` on the same problem (same retraction,
    Cauchy loss of the same scale, tolerances 1e-15) reaches the same cost to 1e-6 and the
    same E to 1e-4. Ours stops on its function tolerance, where the cost is flat.
- **Run for run.** For every minimal solver, refined or not, Python and C++ draw the same
  samples, stop on the same iteration, find the same inliers and agree on E to 1e-12. The
  same holds for two-view initialisation.
- **Native tests** (`cpp/tests/foundations/test_fivepoint.cpp`, `test_epipolar.cpp`) cover
  the solvers on deterministic samples, Sturm roots, every solver in RANSAC, and the
  refinement.

## Measured

Forty synthetic scenes: 200 points, 1 px noise at f = 520, 40% outliers, 2 px threshold.
Errors are E's max-entry error over the floor (the 8-point fit to the true inliers). Times are
the C++ backend's wall time per RANSAC run (`five_point_sweep.py`).

| Minimal solver | Refine | Error / floor (median, p90) | Samples | Recall | ms |
|---|---|---|---|---|---|
| 8-point | no | 1.06, 3.48 | 738 | 0.915 | 8.4 |
| 8-point | yes | 0.84, 3.19 | 738 | 0.949 | 8.4 |
| Nistér | no | 0.93, 4.40 | 110 | 0.923 | 5.5 |
| **Nistér** | **yes** | **0.71, 3.21** | **110** | **0.950** | **5.4** |
| Stewénius | no | 0.93, 4.40 | 110 | 0.923 | 3.5 |
| Stewénius | yes | 0.71, 3.21 | 110 | 0.950 | 3.5 |

- The 5-point needs a seventh of the samples, and its models are better before any refinement.
- The refinement is what goes below the floor.
- Nistér and Stewénius draw identical runs (same samples, same models), so only their speed
  differs. The pure-Python reference is about 50 times slower with Nistér (2-3 ms per sample)
  than its C++ core.

**TUM fr1/xyz**, the two-view note's 38 frame pairs, on the same SIFT matches
(`tum_solver_survey.py`). Errors are median degrees on the initialised
pairs:

| E setting | Initialised | Rotation | Translation direction | Wrong (> 20°) |
|---|---|---|---|---|
| 8-point, no refine (PR 2) | 22 / 38 | 0.88 | 4.16 | 2 |
| 8-point, refine | 22 / 38 | 0.72 | 2.29 | 2 |
| **Nistér or Stewénius, refine** | **24 / 38** | **0.49** | **2.33** | **2** |
| Nistér or Stewénius, no refine | 24 / 38 | 0.83 | 2.83 | 2 |

The unrefined 8-point row reproduces PR 2's survey exactly. With the 5-point solvers two pairs
that were `ambiguous` now initialise. OpenCV's 5-point RANSAC on all 38 pairs has a median
error of 1.32° rotation and 5.54° direction, and 12 wrong; that includes the 13 low-parallax
pairs we decline. Try it with `scripts/demo_two_view.py --essential-solver stewenius
--no-refine`.

## What was learned

- **Precision matters in a minimal solver.** Nistér's determinant loses digits. Three
  Gauss-Newton steps on the original constraints are cheap and recover them.
- **"It fits the data" is not "it is a solution".** Any null-space matrix fits five epipolar
  equations exactly. Essentialness is the cubic constraint, which is what the oracle must be
  filtered on, and the reason degenerate samples can return junk.
- **Cross-backend parity reaches into choices that "don't matter".** All four decomposed
  poses are valid starts for the refinement. Leaving the choice to each SVD cost five orders
  of magnitude of agreement.
- **A better E moves model selection.** The two-view test's desk scene (80% planar) had H
  winning GRIC by a few points with the 8-point. The refined E closed that gap, so the scene
  is now 90% planar, where H wins by 15 to 80 points for every solver. A test that sits on a
  margin that thin is testing the noise.
- **The refinement needs RANSAC's start.** From an 8-point fit to contaminated data, 20 LM
  iterations do not converge (the Cauchy cost is not convex). From the consensus they take 5
  to 8. `polish_consensus` keeps a refinement only when it helps, so the cap is safe there.

## References

- D. Nistér, "An efficient solution to the five-point relative pose problem", TPAMI 26(6),
  2004.
- H. Stewénius, C. Engels and D. Nistér, "Recent developments on direct relative
  orientation", ISPRS J. Photogrammetry and Remote Sensing 60(4), 2006.
- M. Demazure, "Sur deux problèmes de reconstruction", INRIA RR-882, 1988.
- R. Hartley and A. Zisserman, *Multiple View Geometry*, 2nd ed., section 11.4.3 (the Sampson
  error).
- V. Larsson et al., PoseLib (github.com/PoseLib/PoseLib), the test oracle.
- K. Lebeda, J. Matas and O. Chum, "Fixing the locally optimized RANSAC", BMVC 2012.
