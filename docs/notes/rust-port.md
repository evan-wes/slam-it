# The Rust core: rung 0's geometry and optimisation

PR 5 brings rung 0's geometry and optimisation to Rust: SO(3), SE(3), the camera models, Umeyama
alignment, PCG32, the generic least-squares solver and bundle adjustment. PR 6 adds the
estimation core (RANSAC and the minimal solvers), after which Python, C++ and Rust have the
same capabilities at the end of milestone 1. This note is about the port itself: how the C++
core's design carried over, where Rust changed it, and the measured comparison.

## The problem

The same algorithms, three times, need to agree to the last few bits and fail the same way:
- the parity fixtures hold every language to the Python reference;
- bundle adjustment's fixture compares iteration counts, terminations and gauge codes exactly
  (`"the two cores step alike"`), so the Rust solve has to take the same decisions at every
  step, not merely land near the same optimum.

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

**Errors.** C++23's `std::expected<T, Error>` and Rust's `Result<T, Error>` are the same
contract, but Rust's `?` makes the error path a single character where C++ needs
`if (!x) return std::unexpected(x.error());`. Outside the tests, there is no `unwrap` in
the cores. An `expect` appears only where a value cannot be absent, such as an SVD asked for
its `U`; on a public function it is documented under `# Panics`. Indexing relies on the
checks every entry point runs first, as in C++. A bad index panics in Rust, where in C++ it
is undefined behaviour.

**Enums carry their names.** Each crossing enum (camera models, NLS methods, losses and
terminations, BA gauges) has `ALL` (its values in code order) and `name()`. The bindings
build `codes()` from them and read choices by name through them. Settling this mechanism
closed panel item A27 in both languages: the C++ BA binding's own name tables went, and every
C++ enum that crosses has a core `name()` too.

**Options cross through serde.** The binding reads `BAOptions` with
`#[serde(deny_unknown_fields)]` structs at every depth (`pythonize` turns the dict into
them), so a field added on one side only fails loudly. The C++ binding's key check now
words its errors the way serde does (``missing field `x` ``, ``unknown field `x` ``), so both
fail with the same message.

**The batch layer is one macro.** C++ binds each function with a lambda over `map_batch`;
the Rust binding declares each with `batched!`, which writes the `#[pyfunction]` from the
Protocol's signature and maps the batch item by item with any fixed-size nalgebra type as
the item.

## How it's checked

- The parity fixtures, for every module on every backend: SO(3) and SE(3) against the
  mpmath oracle, the cameras, Umeyama, and all fifteen bundle-adjustment cases, iteration
  counts and codes included.
- Each module the Rust core implements in full runs its property tests (round trips, group
  laws, Jacobians against finite differences, every BA behaviour test) on all three backends.
- PCG32's reference output and Floyd samples are one fixture,
  `tests/fixtures/foundations/rng.json`, read by the Python tests, the native C++ tests
  (through nlohmann_json) and the Rust tests (through serde_json); panel item A9 for PCG32.
- Native Rust tests mirror the native C++ ones, so the crate is checked without Python.

## Measured

On BAL's ladybug-49 (49 cameras, 7,776 points, 31,843 observations), with the BAL demo's
settings, every backend reaches the same solution (`scripts/demo_bal.py --backend all`):

| | RMS (px) | Iterations | Time |
|---|---|---|---|
| Python | 0.9147 | 33 | 63.8 s |
| C++ | 0.9147 | 33 | 1.49 s |
| Rust | 0.9147 | 33 | 1.44 s |
| COLMAP (Ceres) | 0.9147 | -- | 0.36 s |

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
level. The small operations differ either way by up to a factor of two, from how each
library's small fixed-size code is compiled and from how much each port copies:
- the Rust projection expands every model's intrinsics into a fixed nine-element array;
- the C++ one passes an `Eigen::Ref` straight through.

Run them with:

```text
SLAMLAB_BAL=<problem.txt> build/native/bench/foundations_bench_foundations
SLAMLAB_BAL=<problem.txt> cargo bench --manifest-path rust/Cargo.toml --bench foundations
```

Code lines, counted without blank lines, comments or tests:

| Module | Python | C++ | Rust |
|---|---|---|---|
| SO(3) | 194 | 202 | 217 |
| SE(3) | 184 | 158 | 128 |
| Cameras | 104 | 237 | 215 |
| Alignment | 50 | 50 | 67 |
| PCG32 | 40 | 57 | 60 |
| NLS and `solve` | 348 | 461 | 519 |
| Bundle adjustment | 946 | 1806 | 1931 |
| **Total** | **1866** | **2971** | **3137** |

The compiled cores are about the same size, and both are 60-70% larger than vectorised numpy
for the same algorithms. rustfmt's one-argument-per-line layout accounts for much of Rust's
small excess.

## What was learned

- **Bit-level agreement survives a change of linear-algebra library.** The C++ and Python
  cores were written so that their decisions depend on spans, not on the bases an SVD
  happens to return (a canonical span from the projector, the Householder complement, a
  tie-break margin). That design, made for Eigen against numpy, carried nalgebra too: all
  fifteen BA cases match on the first run, iteration counts included.
- **A thin SVD is a trap for null spaces.** Taking the "last columns of V" silently returns
  too few vectors when the matrix is wide; the padding is a one-line fix, but only a test of
  a wide matrix shows it is needed.
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
