# RANSAC

Design note for `slam_it.foundations.ransac` and `slam_it.foundations.rng` (the Python
references) and `slam_it/foundations/ransac.hpp` and `rng.hpp` (the C++ core).

## The problem

Feature matches between two images are mostly right and partly wrong. A least-squares fit to
all of them is dragged anywhere by the wrong ones, and a robust loss only helps once the
estimate is already close. RANSAC (Fischler and Bolles, 1981) needs no starting estimate. It
fits the model to random **minimal samples**, the least data that determine it (8 matches for
the 8-point essential matrix, 4 for a homography), and keeps the model most of the data
agree with. One all-inlier sample is enough, and the number of samples needed to draw one
with high probability can be computed.

This module is the loop. The models are **estimators**: a minimal solver, a least-squares
(non-minimal) solver and a residual. The loop never knows which model it is fitting.

## The interface

| Estimator member | Returns |
|---|---|
| `sample_size`, `size` | the minimal sample size `s`, the number of data `N` |
| `fit_minimal(sample)` | every model the sample gives (several for some solvers; none if degenerate) |
| `fit_nonminimal(subset)` | the least-squares models for at least `s` data |
| `residuals(model)` | every datum's squared residual |

`estimate(estimator, options)` returns a `Solution` (the shared solver contract,
`foundations.solve`) holding a `Consensus` (the model and its inlier mask) and a `Summary`:

| Field | Meaning |
|---|---|
| `iterations` | minimal samples drawn |
| `termination`, `converged` | `confidence` (converged) or `max_iterations` |
| `degenerate_samples` | samples that gave no model |
| `local_optimizations` | refinements run |
| `inlier_count`, `score` | the best model's inliers and MSAC cost |

The inlier mask is part of the estimate, not the summary: callers use it as data (to
triangulate, to refit). The summary holds what describes the run.

**Errors.** Reaching `max_iterations` is a normal result with `converged` false. A run in which
no sample gives a model has no estimate to return, so it raises `Degenerate`. Fewer data than
a minimal sample, or settings out of range, are `InvalidInput`.

**Generic in the core, concrete in the bindings.** In C++, `estimate` is a template over the
`Estimator` concept, and it is not bound. The backends meet at the concrete two-view
estimators, whose bindings each instantiate it.

## The maths

**MSAC scoring.** A model's cost is `Σ min(r_i², θ²)`. Plain RANSAC maximises the inlier count,
so it cannot tell a model that fits its inliers to 0.1 px from one that fits them to 0.9 px.
The truncated quadratic can, at the same cost (Torr and Zisserman, 2000). Lower is better, and
a candidate replaces the best only if strictly lower.

**Adaptive stopping.** With inlier fraction `w` and sample size `s`, one sample is all inliers
with probability `wˢ`, so after `k` samples the chance of never having drawn one is
`(1 - wˢ)ᵏ`. Setting that to `1 - p` gives

```
k = log(1 - p) / log(1 - wˢ)
```

with `w` from the best model so far. The bound shrinks as better models are found, and the
loop stops at the first iteration at or past it. Both logs are taken as `log1p`, which
matters when `wˢ` is small: `log(1 - 1e-10)` loses most of its digits, `log1p(-1e-10)` none.

**Local optimisation.** A model from a minimal sample of inliers is still noisy, because the
sample is the least data that determine it. Chum, Matas and Kittler (2003) refine each new
best model; Lebeda, Matas and Chum ("Fixing the locally optimized RANSAC", BMVC 2012) settled
which refinement works. Here, each time a new best model is found:

1. **Iterative least squares.** Refit with the non-minimal solver to the inliers, `lo_steps`
   times (4), the threshold shrinking linearly from `3θ` to `θ`. The wide first threshold
   pulls in inliers the noisy model just missed; the last one is the run's threshold.
2. **Inner RANSAC.** `lo_inner_iterations` times (10), fit the non-minimal solver to a random
   subset of the current inliers, `3s` of them but at most half, and run step 1 from that.
   Several non-minimal starts escape a poor local optimum that step 1 alone would stay in.

A refined model is kept only if it lowers the MSAC cost. New best models are rare after the
first few samples, so refinement runs a handful of times per run. On the line-fitting test it
lowers the final cost by 0.5% with the same 13 samples. Its effect on the two-view estimators is
measured in the two-view note.

**Degenerate samples.** An estimator returns no model for a sample it cannot fit. That sample
is counted and the loop moves on. The count is diagnostic: a run with many degenerate samples
is on data the model barely fits (a near-planar scene for the essential matrix).

## Random numbers, and parity between languages

The C++ and Python backends have to be compared. With different random generators they would
test different hypotheses, and only their statistics could be compared. So both use one small
generator, implemented identically in each language:

- **PCG32** (O'Neill, 2014): a 64-bit LCG state with a permuted 32-bit output, seeded as the
  reference library's `pcg32_srandom_r`. Both implementations reproduce the library's
  published output for seed 42, stream 54.
- **Bounded draws** reject outputs below `2³² mod bound`, so they have no modulo bias.
- **Samples of distinct indices** use Floyd's algorithm, which makes exactly `count` draws
  whatever happens, so both languages consume the stream identically.

What parity can and cannot promise: the two backends draw the same samples, but their solvers
(numpy's LAPACK, Eigen) differ at the level of rounding, and so do their residuals. A datum
whose residual sits within rounding of the threshold could be an inlier in one and not the
other. On data without such ties, which is any data with real noise, the backends give the
same inliers, the same iteration count and the same sequence of local optimisations, and
models equal to rounding. The line-fitting test checks exactly this: the C++ test
reproduces the Python run's 13 iterations, 2 refinements and its line to 1e-12.

## How it's checked

- **The generator:** the reference library's published output in both languages; the same
  draws from the same seed; uniformity of samples; independence of streams; bounds.
- **The loop, through a line estimator** (in both languages, on identical data):
  - recovers `y = 2x + 1` and exactly the non-outliers;
  - stops at the first iteration at or past the bound;
  - local optimisation lowers the cost on the same samples;
  - an iteration cap is a result, not an error;
  - degenerate samples are counted, and a run where every sample is degenerate raises;
  - settings out of range are rejected.
- **The concrete estimators** (two-view geometry) add parity fixtures for whole runs.

## References

- M. A. Fischler and R. C. Bolles, "Random sample consensus", CACM 24(6), 1981.
- P. H. S. Torr and A. Zisserman, "MLESAC: a new robust estimator with application to
  estimating image geometry", CVIU 78, 2000.
- O. Chum, J. Matas and J. Kittler, "Locally optimized RANSAC", DAGM 2003.
- K. Lebeda, J. Matas and O. Chum, "Fixing the locally optimized RANSAC", BMVC 2012.
- M. E. O'Neill, "PCG: a family of simple fast space-efficient statistically good algorithms
  for random number generation", HMC-CS-2014-0905, 2014.
- J. Bentley and B. Floyd, "Programming pearls: a sample of brilliance", CACM 30(9), 1987.
