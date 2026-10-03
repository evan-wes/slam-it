# Non-linear least squares on manifolds

Design note for `slamlab.foundations.nls` (the Python reference) and
`slamlab/foundations/nls.hpp` (the C++ core).

## The problem

Bundle adjustment, PnP refinement, pose-graph optimisation and calibration share one problem
shape: they minimise a sum of squared residuals over a state that is not a flat vector.
Poses and rotations live on Lie groups, and points are often re-parameterised. The
residuals come in small **blocks**: one reprojection error (2 rows), one prior (1 row), or
one relative-pose error (6 rows). A few blocks are wrong, through bad matches or moving
objects, so the cost must not let them dominate.

This module is the solver loop that all of those share. It doesn't know what the state
is: the problem supplies the cost, the linearisation and the step rule.

```
cost(x) = ½ Σ_i ρ(|r_i(x)|²)
```

## The interface

A `Problem` provides three things:

| Method | Returns |
|---|---|
| `squared_norms(x)` | each block's `s_i = |r_i(x)|²` |
| `linearize(x, w)` | a `LinearSystem`: the normal equations with block weights `w` |
| `retract(x, δ)` | the state moved by a tangent step δ (`T exp(δ)` for a pose) |

A `LinearSystem` provides the gradient `g = Jᵀ W r`, the Hessian diagonal of `H = Jᵀ W J`,
and `solve(d)` for `(H + diag(d)) δ = -g`, or nothing if that system is not positive
definite.

**Why the linear system is a separate seam.** The solver never forms `H` itself.
`DenseSystem` does: it builds the full matrix and factors it by Cholesky, which suits tests
and small problems. Bundle adjustment will supply a Schur-complement system instead (commit
6), which eliminates the points and factors only the reduced camera system. The loop that
decides damping, acceptance and termination is identical for both. The alternative,
handing the solver a Jacobian, would have tied it to one sparsity pattern.

**Generic in the core, concrete in the bindings.** In C++, `solve` is a template constrained
by the `Problem` and `LinearSystem` concepts. A test problem and BA compile into the same
loop with no virtual dispatch. The template isn't bound to Python; the backends meet at the
concrete solvers built on it. So C++/Python parity for this loop is checked through
bundle adjustment's fixture, and here by running the same test problems natively in both
languages.

## The maths

**Robust losses, by iteratively reweighted least squares (IRLS).** Each block gets the
weight `w_i = ρ'(s_i)`, and the step solves the weighted Gauss-Newton system. The robust
cost's gradient is `Σ ρ'(s_i) J_iᵀ r_i`, which is exactly `Jᵀ W r`. So the fixed point is the
robust optimum, not an approximation of it.

The Hessian approximation drops the `2ρ''(s) J_iᵀ r_i r_iᵀ J_i` term. Triggs et al.'s
correction, which Ceres uses, keeps that term and rescales the residual and Jacobian to stay
positive semi-definite. Dropping it keeps the system PSD with no rescaling, and costs
convergence speed only where `ρ''` is large: blocks near the loss's knee.

| Loss | ρ(s) | ρ'(s) |
|---|---|---|
| trivial | `s` | `1` |
| Huber (scale c) | `s` if `s ≤ c²`, else `2c√s - c²` | `1`, else `c/√s` |
| Cauchy (scale c) | `c² log(1 + s/c²)` | `1/(1 + s/c²)` |

These are Ceres' `HuberLoss(c)`/`CauchyLoss(c)` and SciPy's `least_squares(loss=...,
f_scale=c)`, so a cost from either compares directly.

**Levenberg-Marquardt:**
- **Damping.** `D = μ · clamp(diag H, 1e-6, 1e32)`, Marquardt's scale-invariant choice. The
  clamp, from Ceres, keeps a parameter that the residuals don't observe from producing a
  zero diagonal and an unbounded step.
- **Acceptance.** The predicted decrease of the quadratic model is
  `½ δᵀ(D δ - g)`, which follows from `(H + D)δ = -g` with no product by H. The step is
  accepted when the actual decrease exceeds `1e-3` of the predicted decrease and the cost
  actually fell.
- **Update.** Nielsen's rule on success, `μ ← μ max(1/3, 1 - (2ρ - 1)³)`. On a rejection, or
  an indefinite system, `μ ← μ ν` with `ν` doubling each consecutive time.

**Gauss-Newton** takes the undamped step and stops at the first one that doesn't lower the
cost, rather than accepting it. Without a line search, that is the honest behaviour.

**Termination** follows Ceres:
- the gradient's max-norm is below a tolerance;
- an accepted step's relative cost decrease is below a tolerance;
- the step's norm is below a tolerance. It is absolute, because a state on a manifold has
  no norm to make it relative;
- or the iteration cap is reached.

Only the first three count as converged.

**The solver contract.** The returned state is always the lowest-cost one visited, and a
`Summary` records the costs, the termination reason and every step tried. Hitting the
cap is not an error. `strict=True` turns any unconverged stop into `NotConvergedError`
(`ErrorKind::kNotConverged` in C++).

## How it's checked

The same test problems run in both languages:
- **Rosenbrock from (-1.2, 1):**
  - LM reaches (1, 1) to 1e-10.
  - GN's first undamped step lands at (1, -3.84), raising the cost from 12.1 to 1171, so
    GN stops there with `no_decrease`.
  - From (0.5, 0.5), GN converges in two steps. This is the textbook reason damping exists,
    pinned as a test.
- **An unobserved parameter.** The undamped system is singular, so GN stops with
  `singular`. LM's clamped diagonal copes and leaves the parameter untouched.
- **A line fit with three gross outliers:**
  - plain least squares is dragged more than 0.3 off;
  - Cauchy recovers the line to within 0.02;
  - the trivial loss matches the exact `lstsq` solution to 1e-10.
- **Robust rotation averaging on SO(3).** Blocks are `log(Rᵀ R_i)`, with the Jacobian
  `-Jl⁻¹(r_i)` and the retraction `R exp(δ)`. This exercises a real manifold state, and the
  result stays orthonormal to 1e-12. The hand-derived Jacobian is checked with
  `numeric_jacobian`, the module's checker for every future Jacobian.
- **The trace:** accepted costs are non-increasing, and at the iteration cap the result is
  still no worse than the start.
- **Invalid settings** are rejected in both languages.

## What was learned

- **SciPy is a cross-check, not ground truth.** SciPy's `trf` stops about 8e-9 from the
  exact least-squares answer, even with all tolerances at 1e-15. This solver reaches
  1e-11. On the linear case the exact answer is `lstsq`. The robust cases are pinned by the
  robust gradient vanishing at the optimum, with SciPy at 1e-7 as a cross-check. This is
  the same lesson as manif in [lie-groups.md](lie-groups.md): when a library and the code
  disagree, an independent referee decides.
- **Cost-based acceptance has a resolution floor.** Near the optimum, a step changes the
  cost by about `g·δ`. Once that falls below `eps · cost`, no step can be seen to help.
  - For the Cauchy line fit, that leaves a gradient near 2e-9 (with a parameter error near
    3e-11).
  - With tight tolerances, the solver then spends its last iterations rejecting steps that
    are rounding noise, until the damping grows enough that the step-norm test fires.
  - The default `function_tolerance = 1e-6` stops well before that. Tests that want the
    floor set the tolerances explicitly.
