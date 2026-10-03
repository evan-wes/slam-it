# Two-view geometry and initialisation

Design note for `slamlab.foundations.epipolar`, `.homography`, `.twoview` and
`.correspondences` (the Python references) and their headers in `slamlab/foundations/` (the C++
core). Triangulation and RANSAC have their own notes.

## The problem

A monocular SLAM system starts with two frames and their feature matches, and nothing else:
no pose, no depth, no scale. It has to find how the camera moved between them (a rotation,
and a translation direction) and triangulate the first map points. If it cannot do that
reliably, it should say so and wait for the next frame, because a wrong start poisons
everything built on it.

Two models describe two views of a static scene:

- **the essential matrix** `E = [t]ₓ R`: the general case, any scene, one constraint per
  match (`f₂ᵀ E f₁ = 0`, the match lies on its epipolar line);
- **a homography** `H ∝ R + t nᵀ / d`: a plane `nᵀ X = d`, or a camera that only rotated
  (`t = 0`), two constraints per match (`x₂ ∝ H x₁`).

E is degenerate exactly where H is the right model (a plane gives the 8-point system a
family of solutions; a rotation gives `E = 0`), so an initialiser needs both and a way to
choose.

## The interface

| Function | What it does |
|---|---|
| `epipolar.essential_8pt(b1, b2)` | E from 8+ bearing pairs: normalised 8-point, projected onto the essential manifold |
| `epipolar.fundamental_8pt(u1, u2)` | F from 8+ pixel pairs: normalised 8-point, rank 2 |
| `epipolar.sampson_essential`, `sampson_fundamental` | each match's squared Sampson distance |
| `epipolar.essential_ransac`, `fundamental_ransac` | LO-MSAC with the 8-point solver |
| `epipolar.decompose_essential(E)` | E's four candidate poses |
| `homography.homography_dlt(x1, x2)` | H from 4+ point pairs: normalised DLT |
| `homography.sampson_homography` | each match's squared Sampson distance |
| `homography.homography_ransac` | LO-MSAC with the DLT |
| `homography.decompose_homography(H)` | Malis and Vargas: four `(R, t/d, n)` candidates, or one for a rotation |
| `twoview.initialize(b1, b2, options)` | both RANSACs, GRIC, the decomposition, the choice of motion |

The pieces the estimators share live below them, so that `epipolar` and `homography` are
independent siblings that the 5-point solver and absolute pose can build on too: the input
checks in `foundations.arrays` (`as_bearing_pairs`, `as_point_pairs`, `as_matrix3`), the
unchecked building blocks in `foundations.correspondences` (homogeneous and normalised image
points, Hartley's normalisation, the null vector, `PairEstimator`), and `ransac.run`, which
runs an estimator with the reference or a compiled backend.

`initialize` returns an `Initialization`: a **status** (`initialized`, `pure_rotation`,
`low_parallax`, `ambiguous`), the model that decided and whether it was a fallback, the
relative pose `T_c2_c1` with unit translation, the inliers, the triangulated points, both
GRIC scores, each candidate's support, and both RANSAC summaries. Not initialising is a
status, not an exception: only a pair from which neither RANSAC finds any model raises
`Degenerate`.

## The maths

**The 8-point algorithm, normalised.** Each match gives one equation linear in E's nine
entries, `(x₂ ⊗ x₁) · vec(E) = 0`, and eight or more give `A vec(E) = 0`, solved by the right
singular vector of A with the smallest singular value (Longuet-Higgins 1981). Hartley ("In
defense of the eight-point algorithm", 1997) showed that the columns of A must be of similar
size first: each image's points are shifted to their centroid and scaled to a mean distance
of √2, and the solution is mapped back. E is then projected onto the essential manifold,
`U diag(1, 1, 0) Vᵀ`; F onto rank 2.

That normalisation was not optional here. The first version ran the 8-point algorithm
directly on the bearing vectors, which are unit vectors and looked well scaled. They are
not: for a ±27° field of view, x and y are about a third of z, and the columns of A differ
by up to an order of magnitude. On a 200-match scene with 1 px noise, E fitted to every
inlier had **8 times the error** with bearings (0.19 in E's entries) as with normalised
image points (0.023), and RANSAC's least-squares refits stalled far from the optimum. E is
now fitted on the normalised image points `f / f_z`, normalised, then projected after
mapping back (the normalisation does not preserve the essential structure).

**The Sampson distance** is RANSAC's residual: the first-order distance a match must move,
in both images together, to satisfy the model (Hartley and Zisserman, 4.2.6 and 11.4.3):

```
E or F:  e² = (x₂ᵀ M x₁)² / ((M x₁)₀² + (M x₁)₁² + (Mᵀ x₂)₀² + (Mᵀ x₂)₁²)
H:       e² = εᵀ (J Jᵀ)⁻¹ ε,  ε = the first two rows of x₂ × H x₁,  J = ∂ε/∂(x₁, y₁, x₂, y₂)
```

For H it matched the true geometric distance to four digits with 1e-3 noise in the normalised
plane (about half a pixel; checked against a Nelder-Mead minimisation). Both are distances in the same joint image space, which is what
lets GRIC compare them. For E the units are the normalised image plane, so a pixel threshold
becomes `θ / focal`.

**Decompositions.** E has four poses: `R ∈ {U W Vᵀ, U Wᵀ Vᵀ}` (W a quarter turn about z,
U and V made proper rotations) and `t = ±u₃` (Hartley and Zisserman, result 9.19). The second
rotation is the "twisted pair", the first turned half a turn about the baseline, and the
fixture checks exactly that closed form.

H uses Malis and Vargas' analytical decomposition ("Deeper understanding of the homography
decomposition for vision-based control", INRIA RR-6303, 2007), the method behind OpenCV's
`decomposeHomographyMat`. Scale H to a middle singular value of 1; `S = HᵀH - I` is zero for
a rotation, and otherwise its minors give two plane normals in closed form, each with a
`t*` and `R = H (I - (2/v) t* nᵀ)`. With `(R, t, n)` and `(R, -t, -n)` that is four
candidates. Two details matter on real data:

- the square roots of the minors are clamped at zero, because rounding takes a zero minor
  just below it and the square root to NaN;
- the rotation is **projected onto SO(3)**. The closed form assumes an exact
  `H = R + t nᵀ`; for an estimated H it is only near a rotation. On TUM fr1/xyz pairs it was
  off by more than the 1e-6 our rotation checks allow, so the C++ triangulation refused the
  candidate while the unchecked Python reference used it: the backends diverged. Both now
  project, in the four-candidate path and, after the review panel found it unprojected, in
  the pure-rotation branch too (a well-estimated rotation reached it with a pose 3e-4 off
  orthonormal).

**Thresholds per model.** Under image noise σ, a match's squared Sampson distance is about
σ²χ²₁ to E, which puts one constraint on it, but σ²χ²₂ to H, which puts two. One threshold
therefore kept 95% of E's true inliers but 86% of H's (measured on 20 000 matches). E-RANSAC
uses `threshold_px` and H-RANSAC that times `√(5.991 / 3.841)`, the ratio of the two 95% χ²
quantiles (Hartley and Zisserman, table 4.1; ORB-SLAM uses 3.841 and 5.991). Re-measured on
the 38 TUM pairs below, it changed no decision: the same 22 initialise, the same 2 are wrong;
the two pairs H decides lose a little accuracy (median rotation error 0.77° to 0.88°), the
cost of admitting the noisier half of H's true inliers.

**GRIC** (Torr, "Geometric motion segmentation and model selection", 1998) chooses between
E and H:

```
GRIC = Σ min(e²/σ², λ₃ (r - d)) + λ₁ d n + λ₂ k
```

with `r = 4` (a match is two image points), `d` the model's dimension (E 3, H 2), `k` its
parameters (E 5, H 8), and Torr's `λ₁ = ln r`, `λ₂ = ln(r n)`, `λ₃ = 2`. The fit term
charges each match its squared distance in noise units, capped; the penalty charges each
match for the dimensions the model leaves free. H fits a plane or a rotation as well as E
with one dimension less, so it wins there; on a general scene its residuals are large, so E
wins.

**GRIC over the explained matches only.** The cap is `λ₃ (r - d)`: 2 for E, 4 for H. An
outlier fits neither model and tells nothing about which is right, but counting it costs H
twice what it costs E. Scored over every match, a pure rotation with 30% outliers chose E by
8 points. Both scores are now taken over the union of the two RANSACs' inliers, and H wins
the rotation clearly.

**Choosing the motion.** Each candidate of the winning model triangulates the model's
inliers (Lindstrom's optimal two-view method). The candidate with the most points in front
of both cameras is best. Then:

1. a pure rotation (H's single candidate, zero translation) is `pure_rotation`;
2. fewer than `min_points` (50) of its points with `min_parallax_deg` (1°) of parallax is
   `low_parallax`. This is checked before ambiguity, because it is the cause the caller can
   act on (wait for more motion), and because it usually makes the candidates hard to tell
   apart too; a noisy pure rotation usually ends here, its H not quite a rotation;
3. a runner-up with more than `ambiguity_ratio` (0.75) of the best's support is `ambiguous`,
   the same ratio ORB-SLAM uses. Support here is every inlier triangulated in front of both
   cameras, whatever its parallax; ORB-SLAM's `CheckRT` also counts low-parallax points
   behind a camera as good, which makes its candidates harder to tell apart.

**Falling back to E.** On TUM fr1/xyz, a desk seen from half a metre, GRIC preferred H on
most pairs, rightly: with a few degrees of parallax, H explains most matches within the
noise. But H's two physically valid motions then had similar support, so the initialiser
refused almost everything. When H is ambiguous and E on its own is unambiguous with enough
parallax, E now decides, and `fallback` says so. Three rules were measured on the same 38
pairs before choosing (the owner chose the fallback):

| Rule | Initialised | Direction off by > 20° |
|---|---|---|
| Refuse on an ambiguous H (ORB-SLAM's) | 3 | 0 |
| **E decides when H is ambiguous** | **22** | **2** (OpenCV's 5-point: 3 of the same 22) |
| The same, only if E agrees with an H candidate within 10° | 18 | 1 |

The agreement check would have cut one wrong start, but its threshold was tuned on this one
sequence and it still accepts the worst case, where E and H agree with each other and are
both wrong.

## How it's checked

- **Fixtures** (`epipolar.json`, `homography.json`, `twoview.json`, from
  `generate_twoview.py`, 50-digit mpmath):
  - noise-free cases expect the true matrices built from the pose (and plane), scaled as the
    solvers scale them; noisy cases expect each solver's defining solution (the smallest
    eigenvector of `AᵀA` for the normalised system, then mpmath's SVD for the projection);
  - Sampson distances are their closed forms;
  - E's decomposition is held to `(R, ±t̂)` and the twisted pair as an unordered set, and H's
    to contain the true motion and plane;
  - whole RANSAC runs use exact inliers and outliers, so the expected model is the true one;
  - initialisation plants outliers at least ten thresholds from the true model (closer ones
    are inliers to any estimator: in forward motion every epipolar line passes near the
    image centre, so random matches often land on one), and expects the true motion, the
    true points over the baseline, and the right status on a general scene, forward motion,
    a plane (where E-RANSAC finds no model) and a pure rotation.
- **Structure:** E's singular values are (1, 1, 0); F has rank 2 and unit norm; every
  candidate of E rebuilds `[t]ₓ R` and every candidate of H rebuilds H over random motions;
  a noisy H's candidates are rotations.
- **Degeneracy:** coplanar points do not determine E or F; three collinear points do not
  determine H; rank-one matrices are rejected by both decompositions.
- **RANSAC accuracy:** over ten scenes with 1 px noise and 40% outliers, E-RANSAC's median
  error is within 1.5 times the noise floor (the 8-point fit to the true inliers), and plain
  RANSAC's is more than twice that. Over 40 scenes, LO-RANSAC's median is 1.03 times the
  floor and plain RANSAC's 4 to 5 times.
- **Statuses:** noisy general, planar and forward scenes initialise with the right model;
  a rotation and a 1 cm baseline do not; a desk-like scene triggers the fallback.
- **Parity:** Python and C++ give identical runs on the same seed: the same RANSAC
  iterations and refinements, inliers, statuses, candidate counts and triangulated sets,
  with poses within 1e-10 and GRIC scores within 1e-12 relative.

## On real data

`scripts/demo_two_view.py` runs the initialiser on a TUM RGB-D pair, with SIFT matches from
OpenCV. On fr1/xyz frames 0 and 30 (1 s, 35 cm): 434 matches, H wins GRIC (1593 against
1748) but is ambiguous, and E decides; rotation error 0.47°, translation direction 2.4°,
against OpenCV's 5-point RANSAC on the same matches at 1.30° and 5.1°. The C++ core takes
4-6 ms; the Python reference 2.7 s.

`--survey` runs 38 pairs of the sequence (every 40th frame, gaps of 15 and 30 frames):

```
python scripts/demo_two_view.py --survey
```

22 initialise (19 of them by the E fallback), with median errors of 0.88° and 4.2° (OpenCV on
the same pairs: 1.12° and 4.3°); 2 are off by more than 20° in direction (OpenCV: 3); 13
report `low_parallax` and 3 remain `ambiguous`.

**The ground truth is not uniformly good.** The survey also prints how well the
motion-capture pose explains each pair's matches: the median Sampson distance under the true
E. Across the 38 pairs it ranges from 0.3 px to 62 px, with a median of 1.9 px; the first
pairs are near 3.8 px, and frames 200 and 230 disagree with their images by 56 px. Errors
of a degree or so against it are at its own accuracy on many pairs, so comparisons with
OpenCV are the sharper measure.

## What was learned

- **Normalise even "well-scaled" data.** Unit bearing vectors looked balanced and were not;
  Hartley's normalisation was the difference between a RANSAC that reached the noise floor
  and one that did not.
- **The 8-point algorithm is a weak minimal solver.** With 8 noisy matches its E is barely
  better than a guess (median error 0.84 on the test scene); local optimisation's
  least-squares refits do most of the work. The 5-point solver (the next PR) needs a smaller
  sample and gives better minimal models.
- **Residuals with different degrees of freedom need different thresholds.** The review panel
  found that one Sampson threshold silently dropped 14% of H's inliers.
- **The refits minimise an algebraic error.** LO-RANSAC stops at the 8-point fit to its
  inliers, whose MSAC cost was 4% (with 40% outliers) to 15% (with none) above the true
  model's in the 40-scene sweep. A refinement minimising
  the Sampson distance itself, with `foundations.nls` over the essential manifold, would go
  below that floor (deferred).
- **Model selection is where real data bites.** Synthetic scenes are either clearly planar
  or clearly not; a desk is neither, and GRIC's choice of H was right while being useless.
- **Formulas assume exact inputs.** Malis and Vargas' rotation is exact only for an exact H;
  the cross-backend tests caught that it is not on estimated ones.

## References

- H. C. Longuet-Higgins, "A computer algorithm for reconstructing a scene from two
  projections", Nature 293, 1981.
- R. Hartley, "In defense of the eight-point algorithm", TPAMI 19(6), 1997.
- R. Hartley and A. Zisserman, *Multiple View Geometry*, 2nd ed., chapters 4, 9 and 11.
- E. Malis and M. Vargas, "Deeper understanding of the homography decomposition for
  vision-based control", INRIA RR-6303, 2007.
- P. H. S. Torr, "Geometric motion segmentation and model selection", Phil. Trans. R. Soc.
  A 356, 1998.
- R. Mur-Artal, J. M. M. Montiel and J. D. Tardós, "ORB-SLAM: a versatile and accurate
  monocular SLAM system", T-RO 31(5), 2015.
- J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers, "A benchmark for the
  evaluation of RGB-D SLAM systems", IROS 2012.
