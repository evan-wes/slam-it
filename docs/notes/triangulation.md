# Triangulation

Design note for `slam_it.foundations.triangulation` (the Python reference) and
`slam_it/foundations/triangulation.hpp` (the C++ core).

## The problem

Two cameras with a known relative pose see the same point. Each sees it along a ray, and the
point is where the rays meet. With noise they never quite meet, so "where they meet" needs a
definition. Two-view initialisation needs it for every inlier, so it must be cheap. Bundle
adjustment needs a starting point for tracks seen in many views, so the N-view case must be
robust, if not optimal.

Rays are **bearing vectors**: unit directions in each camera's frame. The methods then work
for any camera model that can unproject a pixel, and two-view geometry (which takes bearings
too) passes its rays straight through.

## The interface

| Function | Input | Method |
|---|---|---|
| `two_view(T_c2_c1, bearings1, bearings2)` | one relative pose, (P, 3) pairs | Lindstrom's optimal correction, then exact intersection |
| `n_view(poses, bearings, observed)` | (V, 4, 4) `T_cam_world`, (P, V, 3), (P, V) mask | DLT on bearings, normalised |

Both return a `Triangulation`: the points, `in_front` (cheirality: ahead along every observing
ray, `f · (R X + t) > 0`) and `parallax` (the largest angle at the point between two observing
camera centres). Neither filters: a caller decides what parallax is enough, and counts what
it drops. A point with parallel rays is NaN, behind, with zero parallax. Nothing is raised
for it, because a few such points in a batch are normal.

## The maths

**Two views: Lindstrom's niter2.** Work in the normalised image plane, `x = f / f_z`. The
maximum-likelihood point under isotropic Gaussian image noise is the one whose projections
are closest to the observations. Equivalently: move the two image points the least total
distance that makes them satisfy the epipolar constraint `x₂ᵀ E x₁ = 0`, with `E = [t]ₓ R`.
The corrected rays then meet exactly. Hartley and Sturm solve this with a degree-6 polynomial.
Lindstrom ("Triangulation made easy", CVPR 2010) shows two Newton-like updates of a Lagrange
multiplier reach the same optimum to machine precision in practice:

```
n₂ = S E x₁,  n₁ = S Eᵀ x₂,  Ẽ = S E Sᵀ     (S takes the first two rows)
a = n₂ᵀ Ẽ n₁,  b = ½(|n₂|² + |n₁|²),  c = x₂ᵀ E x₁,  d = √(b² - ac)
λ = c / (b + d)
n₂ ← n₂ - Ẽ λn₁,  n₁ ← n₁ - Ẽᵀ λn₂
λ ← λ · 2d / (|n₂|² + |n₁|²)
x̂₂ = x₂ - Sᵀ λn₂,  x̂₁ = x₁ - Sᵀ λn₁
```

The corrected rays satisfy `d₂ x̂₂ = d₁ R x̂₁ + t`. Crossing both sides with `x̂₂` removes `d₂`,
so `d₁ = -((x̂₂ × t) · (x̂₂ × R x̂₁)) / |x̂₂ × R x̂₁|²`, and the point is `d₁ x̂₁`. The scale of
`E` cancels out of `λn`, so the translation's length does not matter.

The normalised plane needs `f_z > 0`, so `two_view` takes pinhole views only and says so.
A wide-angle or fisheye camera needs a method in bearing space. Lee and Civera ("Triangulation:
why optimize?", BMVC 2019) compare those, and find a weighted midpoint does nearly as well as
the optimum at a fraction of the cost.

**N views: the DLT on bearings.** A point `X` on the ray `f` of a camera `[R | t]` satisfies
`f × (R X + t) = 0`. In homogeneous coordinates that is `[f]ₓ [R | t] X̃ = 0`: three rows, two
independent, linear in `X̃`. Stacking every observing view gives `A X̃ = 0`, and the right
singular vector of `A` with the smallest singular value is the least-squares answer
(Hartley and Zisserman, section 12.2, written for bearings instead of pixels).

It minimises an algebraic error, not a geometric one, so it is a starting point for bundle
adjustment, not a final answer. Its weak spot is conditioning: the rotation columns of `A`
are of order 1 and the translation column of order `|t|`. So the world is first shifted to
the observing cameras' mean centre and scaled by their mean distance from it. This is
Hartley's normalisation, applied to the cameras instead of the image points. The "far from
the origin" fixture case, with cameras a kilometre out, holds the result to 1e-12 against the
oracle.

**When the answer does not exist.**

| Situation | Detection | Result |
|---|---|---|
| Rays parallel (two views) | `|x̂₂ × R x̂₁|` below `1e-12` of its factors | NaN |
| DLT point at infinity | `|w|` below `1e-12 |X̃[:3]|` | NaN |
| Rays along the line through the centres (any view count) | `σ₃ ≤ 1e-12 σ₁`: a two-dimensional null space | NaN |
| Point behind a camera | `f · (R X + t) ≤ 0` | a point, with `in_front` false |
| Zero translation | `|t| = 0` | `InvalidInput`: every depth is undetermined |

The third row was found by the tests. With identical rays from two centres along them, numpy's
LAPACK happened to return the point at infinity from the two-dimensional null space, while
Eigen's Jacobi SVD returned a finite point somewhere on the line. Neither answer is wrong;
the problem is undetermined. Testing the singular values says so explicitly, and both
languages agree.

## How it's checked

- **Oracle fixture** (`tests/fixtures/foundations/triangulation.json`, from
  `generate_triangulation.py`, 50-digit mpmath):
  - noise-free cases expect the true points;
  - noisy two-view cases expect the minimiser of the reprojection error, by Gauss-Newton
    in 50 digits from the rays' closest approach (no closed form shared with the code);
  - noisy N-view cases expect the DLT's defining solution, the smallest eigenvector of `AᵀA`
    for the normalised system, from mpmath's symmetric eigensolver;
  - hard cases: a 5 mm baseline (low parallax), a reversed translation (every point behind),
    cameras a kilometre from the origin.

  Both backends match within 1e-9 (the worst case is 1.4e-10, the reversed pose).
- **Optimality:** no point within 0.1 mm of `two_view`'s has a lower reprojection error.
- **Hard cases:** points on the baseline, parallel rays in the DLT, a 90° parallax by
  construction, partial tracks.
- **The C++ core checks its own input:** a non-unit bearing or a point seen once fails in the
  core, not only at the Python entry point.

## References

- P. Lindstrom, "Triangulation made easy", CVPR 2010.
- R. Hartley and P. Sturm, "Triangulation", CVIU 68(2), 1997.
- R. Hartley and A. Zisserman, *Multiple View Geometry*, 2nd ed., section 12.2.
- S. H. Lee and J. Civera, "Triangulation: why optimize?", BMVC 2019.
