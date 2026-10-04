# The Rust core: rung 0 in a third language

PR 5 brought rung 0's geometry and optimisation to Rust: SO(3), SE(3), the camera models,
Umeyama alignment, PCG32, the generic least-squares solver and bundle adjustment. PR 6 added
the estimation core: correspondences, triangulation, RANSAC, the five-point, epipolar,
homography and two-view modules, P3P and PnP. With it Python, C++ and Rust have the same
capabilities at the end of milestone 1, and Rust is a lock-step backend. This note is about the
port itself: how the C++ core's design carried over, where Rust changed it, and the measured
comparison.

## The problem

The same algorithms, three times, need to take the same decisions and fail the same way:
- the parity fixtures hold every language to the Python reference;
- bundle adjustment's fixture compares iteration counts, terminations and gauge codes exactly
  (`"the two cores step alike"`), so the Rust solve has to take the same decisions at every
  step, not merely land near the same optimum;
- RANSAC draws its samples from PCG32, which every language implements alike, so a seed
  draws the same samples everywhere, and the fixtures compare inlier sets and iteration
  counts: a minimal solver has to return its models in the same order, and every
  comparison has to break ties the same way.

## The interface

Nothing new on the Python side: every module the Rust core implements is reached through the
same `backend=` argument (`backend="rust"`), and `slamlab_rs.foundations.<module>` mirrors the
Python module, as `slamlab._cpp` does. The Protocol test checks the binding's function and
parameter names against each module's `Implementation`.

Outside Python, `rust/slamlab-foundations` is an ordinary crate with no Python dependency:

```text
cargo run --release --example bal -- ~/datasets/bal/ladybug/problem-49-7776-pre.txt
```

solves a BAL problem from Rust alone, as the edge budgets will need.

## The port, module by module

**Linear algebra: nalgebra.** Its fixed-size matrices (`Matrix3<f64>`, `Vector6<f64>`) are the
closest match to Eigen's, so the comparison measures the code rather than the library, and the
cores read alike. Two differences mattered:
- nalgebra's SVD is thin: a wide matrix gets only as many right singular vectors as it has
  rows. The gauge's null spaces need all of them, so a wide matrix is padded with zero rows
  first (which changes neither its row space nor its right singular vectors).
- nalgebra stores dense matrices column-major, like Eigen's default, so the Schur
  complement's upper-triangle updates walk memory in the same order as in C++.

**Templates and concepts became traits.** The C++ solver is a template over a `Problem`
concept; the Rust one is generic over a `Problem` trait with associated `State` and `System`
types, and `LinearSystem` is a trait the dense system and bundle adjustment's Schur system both
implement. The C++ `ReducedSystemSolver` concept is a trait too, so milestone 4's sparse
solver goes behind the same seam in both languages. `solve::Summary` is a trait with an
associated `Termination` type; the C++ concept only names fields.

**RANSAC's estimator is a trait too.** `ransac::Estimator` has an associated `Model` and the
four operations of C++'s concept. Its one addition is a default: `fit_minimal` falls back to
`fit_nonminimal`, which is what the 8-point F and the DLT do, so those two estimators
implement four methods, not five. The refinement callback takes the consensus's inlier
indices in all three languages (panel A6 on PR 4). A summary that nests another (BA's, and
the refined RANSAC run's) implements `solve::Summary` by delegation, as Python's
properties do (A5).

**Errors.** C++23's `std::expected<T, Error>` and Rust's `Result<T, Error>` are the same
contract, but Rust's `?` makes the error path a single character where C++ needs
`if (!x) return std::unexpected(x.error());`. Outside the tests, there is no `unwrap` in
the cores. An `expect` appears only where a value cannot be absent, such as an SVD asked for
its `U`; on a public function it is documented under `# Panics`. Indexing relies on the
checks every entry point runs first, as in C++. A bad index panics in Rust, where in C++ it
is undefined behaviour.

**Where nalgebra and Eigen differ, the port matches Eigen and numpy explicitly.**
- nalgebra has no eigenvectors for a general matrix. Stewénius' five-point solver takes the
  eigenvalues of its action matrix from the real Schur form, and each eigenvector as the
  null vector of `A - λI`, by SVD.
- nalgebra's symmetric eigen-decomposition does not sort its eigenvalues; Eigen's and numpy's
  are ascending. EPnP's null space and control points sort them first.
- A rank-deficient least-squares solve depends on where the cutoff lies.
  `linalg::least_squares` takes it explicitly: EPnP uses numpy's `lstsq` default
  (`max(m, n) ε`), since Python is the reference, and the five-point polish uses Eigen's
  `JacobiSVD` one (`min(m, n) ε`), as the C++ core does.
- Ties go to the first maximum, as Eigen's `maxCoeff` and numpy's `argmax` pick it, in
  column-major order for a matrix. nalgebra's `iamax` agrees on vectors; P3P's line split
  needs it on a matrix, and walks the entries itself.

**Enums carry their names.** Each crossing enum (camera models, NLS methods, losses and
terminations, BA gauges, RANSAC's terminations, the minimal solvers, two-view's models and
statuses) has `ALL` (its values in code order) and `name()`. The bindings
build `codes()` from them and read choices by name through them. Settling this mechanism
closed panel item A27 in both languages: the C++ BA binding's own name tables went, and every
C++ enum that crosses has a core `name()` too.

**Options cross through serde.** The binding reads `BAOptions`, `ransac.Options` and
`twoview.Options` with `#[serde(deny_unknown_fields)]` structs at every depth (`pythonize` turns the dict into
them), so a field added on one side only fails loudly. The C++ binding's key check now
words its errors the way serde does (``missing field `x` ``, ``unknown field `x` ``), so both
fail with the same message.

**Tests sit at the end of each file.** A file's native tests are one inline
`#[cfg(test)] mod tests { ... }` at its end. `scripts/check_layers.py` holds the crate's
modules to the ladder from their `crate::` and `super::` paths, and drops only that trailing
module. Every other `#[cfg(test)]`, every glob import and every re-export in a `mod.rs`
fails the check, because it could hide a dependency.

**The batch layer is one macro.** C++ binds each function with a lambda over `map_batch`;
the Rust binding declares each with `batched!`, which writes the `#[pyfunction]` from the
Protocol's signature and maps the batch item by item with any fixed-size nalgebra type as
the item.

## How it's checked

- The parity fixtures, for every module on every backend: SO(3) and SE(3) against the
  mpmath oracle, the cameras, Umeyama, all fifteen bundle-adjustment cases, and every
  estimator's models, inlier sets and iteration counts.
- Rust is a lock-step backend since PR 6 (`backends.LOCKSTEP_BACKENDS`): every module's
  property tests (round trips, group laws, Jacobians against finite differences, every BA
  and RANSAC behaviour test) run on all three backends, and a compiled backend missing a
  module or a function fails them. Layer 1 runs 1,778 tests, none skipped.
- PCG32's reference output and Floyd samples are one fixture,
  `tests/fixtures/foundations/rng.json`, read by the Python tests, the native C++ tests
  (through nlohmann_json) and the Rust tests (through serde_json); panel item A9 for PCG32.
- Native Rust tests mirror the native C++ ones, so the crate is checked without Python.
- On real data, the TUM demos agree across the cores decision for decision: the two-view
  survey's 38 pairs and the absolute-pose survey's frames print the same statuses, models,
  inlier counts and errors on C++ and Rust (`scripts/demo_two_view.py --survey --backend
  rust`, likewise `demo_absolute_pose.py`).

## Measured

On BAL's ladybug-49 (49 cameras, 7,776 points, 31,843 observations), with the BAL demo's
settings, every backend reaches the same solution (`scripts/demo_bal.py --backend all`):

| | RMS (px) | Iterations | Time |
|---|---|---|---|
| Python | 0.9147 | 33 | 63.8 s |
| C++ | 0.9147 | 33 | 1.35-1.49 s |
| Rust | 0.9147 | 33 | 1.29-1.44 s |
| COLMAP (Ceres), one thread | 0.9147 | -- | 0.64-0.71 s |
| COLMAP (Ceres), all 24 cores | 0.9147 | -- | 0.35-0.44 s |

Our cores run on one thread. COLMAP's Ceres uses every core by default
(`num_threads = -1`), and ladybug is large enough (63,686 residuals) for it to do so, so
the fair comparison is its one-thread row: Ceres is about twice as fast. Why has not been
profiled. It is not a dense reduced system against a sparse one: at 50 images or fewer
COLMAP chooses Ceres's `DENSE_SCHUR`, which forms ladybug's 49-camera reduced system densely
too. Candidates are Ceres's fixed-size Schur eliminator (`SchurEliminator<2, 3, 9>`), its
Jacobian evaluation, and its iteration count (the call used here does not report it). The
ranges are separate runs on a shared machine.

The native benchmarks (Google Benchmark in `cpp/bench/`, criterion in
`rust/slamlab-foundations/benches/`), on the same machine, one core:

| Benchmark | C++ | Rust |
|---|---|---|
| Bundle adjustment, ladybug-49 | 1.430 s | 1.441 s |
| `so3::exp` | 14.9 ns | 11.7 ns |
| `so3::log` | 30.7 ns | 17.5 ns |
| `se3::log` | 73.9 ns | 89.5 ns |
| radtan projection | 10.6 ns | 20.0 ns |

Bundle adjustment, where the time goes into dense linear algebra and the Schur elimination, is
level. The small operations differ either way by up to a factor of two. They have not been
profiled, so this note gives no cause. One plausible cause is ruled out: both projections
copy the intrinsics (C++ into a heap-allocated `Eigen::VectorXd`, Rust into a stack array),
yet C++ is the faster one there.

The robust estimators run on one fixed synthetic scene, built from the same closed-form
expressions in both benches: 500 matches, 30% outliers, half a pixel of noise at a 500 px
focal length, a one-pixel threshold and seed 7, each estimator refined. Both cores take the
same decisions on it, printed before the timings: E-RANSAC keeps 352 inliers after 36
iterations, PnP RANSAC 350 after 17.

| Benchmark | C++ | Rust |
|---|---|---|
| E-RANSAC (Stewénius, refined) | 2.29 ms | 2.69 ms |
| PnP RANSAC (Ding's P3P, EPnP refits, refined) | 2.40 ms | 2.25 ms |

They are within 20% of each other, and neither leads both; as for the small operations, the
gap has not been profiled. Through Python, on TUM's fr1/xyz pair 0-30 (434 matches), two-view
initialisation takes 5 ms on either compiled core and 134 ms in the numpy reference.

Run them with:

```text
SLAMLAB_BAL=<problem.txt> build/native/bench/foundations_bench_foundations
SLAMLAB_BAL=<problem.txt> cargo bench --manifest-path rust/Cargo.toml --bench foundations
```

Code lines, counted without blank lines, comments, docstrings or tests (a line with any code
on it counts once; Python's modules include their `Implementation` Protocols and backend
dispatch, a C++ module is its header and its source):

| Module | Python | C++ | Rust |
|---|---|---|---|
| SO(3) | 223 | 211 | 226 |
| SE(3) | 184 | 161 | 131 |
| Cameras | 192 | 237 | 212 |
| Alignment | 77 | 53 | 74 |
| PCG32 | 40 | 57 | 60 |
| NLS and `solve` | 349 | 462 | 545 |
| Bundle adjustment | 1344 | 1855 | 2015 |
| *Geometry and optimisation* | *2409* | *3036* | *3263* |
| Correspondences | 59 | 155 | 173 |
| Triangulation | 167 | 221 | 187 |
| RANSAC | 283 | 347 | 405 |
| Five-point | 262 | 551 | 572 |
| Epipolar | 338 | 489 | 456 |
| Homography | 193 | 241 | 220 |
| Two-view | 270 | 375 | 395 |
| P3P | 310 | 444 | 448 |
| PnP | 286 | 391 | 400 |
| Rust's `linalg` | -- | -- | 19 |
| *Estimation* | *2168* | *3214* | *3275* |
| **Total** | **4577** | **6250** | **6538** |

The compiled cores are about the same size, and both are about 40% larger than vectorised
numpy for the same algorithms: a quarter to a third for geometry and optimisation, half for
estimation. rustfmt's one-argument-per-line layout accounts for much of Rust's
small excess. (These counts replace PR 5's, whose counting rule was not recorded; every
module is counted the same way here.)

## What was learned

- **Decision-level agreement survives a change of linear-algebra library.** The values are
  not bit-identical: ladybug's final costs differ by 9e-12 relative between C++ and Rust,
  and the BA fixture holds values to about 1e-9. But the decisions are identical: iteration
  counts, terminations and gauge codes. The C++ and Python cores were written so that their
  decisions depend on spans, not on the bases an SVD happens to return (a canonical span
  from the projector, the Householder complement, a tie-break margin). That design, made for
  Eigen against numpy, carried nalgebra too: all fifteen BA cases match on the first run,
  iteration counts included.
- **A thin SVD is a trap for null spaces.** Taking the "last columns of V" silently returns
  too few vectors when the matrix is wide; the padding is a one-line fix, but only a test of
  a wide matrix shows it is needed.
- **A library's defaults are part of the algorithm.** nalgebra's general SVD has no iteration
  cap: on a NaN matrix it never returns, where LAPACK and Eigen stop. Rust's `f64::max`
  drops NaN, where numpy's keeps it, and a NaN gradient could have read as convergence. Both
  were found by the review panel and fixed in both cores. The fixes are one capped SVD helper,
  NaN-propagating norms, and finiteness checks on every input ahead of its tolerance
  comparisons, since NaN passes all of them.
- **Where the languages still differ, on purpose.** A gauge SVD that fails at the start of a
  solve (a non-finite matrix, or in Rust one past the iteration cap) is a `Degenerate` error
  in Rust and in Python, which maps numpy's `LinAlgError` to it (PR 6, panel A20). C++'s
  Eigen SVD always returns a result. All three check their inputs finite first, so the path
  is defensive and practically unreachable.
- **A port is a second reading of the reference.** Porting the Sampson refinement showed that
  C++'s `refine_essential` dereferenced `nls::solve`'s result unchecked. A match at both
  epipoles makes its Sampson distance 0 / 0, the solve rejects the non-finite start, and the
  C++ backend segfaulted, taking the Python process with it, where Python and Rust raise
  `InvalidInputError`. PR 6 fixed C++ and added a test on every backend. A RANSAC refinement
  cannot reach it: every core counts a match as an inlier only when `r < threshold²`, which
  NaN and infinity fail.
- **Decision-level agreement held for RANSAC too.** The same PCG32 streams, comparisons that
  NaN fails, and first-maximum ties give identical inlier sets and iteration counts on the
  synthetic benches and through both TUM surveys, though nalgebra and Eigen differ in the
  last bits.
- **Strictness is cheaper with serde.** `deny_unknown_fields` replaces a hand-written key
  check at every depth, which is why the C++ binding now borrows serde's wording.

## References

- nalgebra: <https://nalgebra.rs>.
- PyO3 and rust-numpy: <https://pyo3.rs>, <https://github.com/PyO3/rust-numpy>.
- pythonize: <https://github.com/davidhewitt/pythonize>.
- criterion.rs: <https://github.com/bheisler/criterion.rs>; Google Benchmark:
  <https://github.com/google/benchmark>.
- Agarwal, Snavely, Seitz and Szeliski, "Bundle Adjustment in the Large", ECCV 2010 (the BAL
  problems).
