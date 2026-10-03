# Changelog

All notable changes to this project are documented here. Format is a
loose variant of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
with 4-digit `MAJOR.MINOR.PATCH.MICRO` version numbers.

## [0.2.1.0] - 2026-10-02

Two-view geometry, milestone 1's slice B up to the 5-point solver: from feature matches
between two calibrated views to their relative motion and first map points, or a reason
why not. Python reference and C++ core in lock-step, with identical RANSAC runs on the
same seed.

### Added

- **The solver contract** (`foundations.solve`): the `Summary` Protocol and the generic
  `Solution[State, Summary]` every iterative solver returns. Each solver owns its summary
  (`nls.Summary`, `ransac.Summary`).
- **PCG32** (`foundations.rng`): one random generator in every backend (the reference
  library's output, an unbiased bounded draw, Floyd's sampling), so a seed draws the same
  RANSAC samples in Python and C++.
- **Triangulation** (`foundations.triangulation`): two-view optimal (Lindstrom's niter2)
  and N-view DLT on bearing vectors, with cheirality and parallax per point; parallel or
  undetermined points are NaN and counted.
- **RANSAC** (`foundations.ransac`): LO-MSAC, generic over an estimator: MSAC scoring, the
  adaptive bound, and Lebeda et al.'s local optimisation.
- **Epipolar geometry** (`foundations.epipolar`): the normalised 8-point essential and
  fundamental matrices, their Sampson distances, RANSAC, and E's four-pose decomposition.
- **Homographies** (`foundations.homography`): the normalised DLT, its Sampson distance,
  RANSAC, and Malis and Vargas' decomposition.
- **Two-view initialisation** (`foundations.twoview`): E and H by RANSAC, Torr's GRIC over
  the matches either explains, the motion chosen by triangulation, and a status
  (`initialized`, `pure_rotation`, `low_parallax`, `ambiguous`) instead of an exception;
  an ambiguous H falls back to E.
- **`slamlab.eval.tum`**: TUM RGB-D sequences with their ground-truth poses and calibrations.
- **`scripts/demo_two_view.py`**: initialisation on a TUM pair against the ground truth and
  OpenCV. On fr1/xyz frames 0 and 30: 0.47 deg rotation and 2.4 deg translation-direction
  error (OpenCV's 5-point RANSAC: 1.30 and 5.1); 4 ms in C++.
- Design notes: `triangulation.md`, `ransac.md`, `two-view.md`.

### Changed

- NLS's `SolveSummary` is `nls.Summary`, and its `Solution` is `solve.Solution`.
- The bindings' array copy helpers moved from `ba.cpp` to `common.hpp`.
- `opencv-python-headless` joins the dev extra (the demo's feature matching only).

### Fixed

- An estimated homography's candidate rotations are projected onto SO(3); the closed form
  is only exact for an exact H.

### Tests

- `so3.log` at both ends of the `arccos` domain: traces a few ulps past 3 and -1.
- Fixtures from a 50-digit mpmath oracle for triangulation, the 8-point solvers, the DLT,
  Sampson distances and both decompositions, and ground-truth fixtures for RANSAC runs and
  initialisation.

## [0.2.0.0] - 2026-10-02

The bundle-adjustment spine of milestone 1: rung 0's geometry, from Lie groups to a bundle
adjuster that matches Ceres on real data. Python reference and C++ core in lock-step
throughout, each component with a design note in `docs/notes/`.

### Added

- **Lie groups** (`foundations.so3`, `.se3`, `.alignment`). SO(3) and SE(3) exp/log,
  Jacobians and their inverses, quaternions, projection onto SO(3), and Umeyama alignment
  (SE(3)/Sim(3)). Parity fixtures come from a 50-digit mpmath oracle using independent
  algorithms; SE(3)'s Q coefficients use exact power series below 1 rad.
- **Camera models** (`foundations.camera`): pinhole, radtan, and radial (COLMAP's RADIAL,
  BAL's camera, computed through radtan by a fixed linear map). Projection, Jacobians with
  respect to the point and the intrinsics, and Newton unprojection to bearing vectors.
- **Non-linear least squares on manifolds** (`foundations.nls`). Gauss-Newton and
  Levenberg-Marquardt over a `Problem`/`LinearSystem` seam; Huber and Cauchy losses by IRLS,
  with a loss per block; a full iteration trace; strict mode; a numeric-Jacobian checker.
- **Bundle adjustment** (`foundations.ba`):
  - robust reprojection over poses, intrinsics (shared or per camera, fixable per
    parameter) and points, with the points eliminated by the Schur complement behind a
    `ReducedSystemSolver` seam;
  - points as XYZ or anchored inverse depths;
  - depth priors through per-camera affines (metric or relative), and pose priors with
    decoupled centre and orientation halves (a GNSS-style centre-only prior works);
  - the gauge removed exactly: the null space of what fixed poses, fixed points, pose
    priors and metric depth pin, removed by fixing or confining as few poses as it takes
    (partial anchoring such as two control points or attitude priors included), in the
    scene's own frame, and the solve runs in a world centred on the cameras, so
    geo-referenced coordinates (up to ECEF range) solve as at the origin;
  - degenerate data dropped and counted, and constant prior blocks left out;
  - marginal deviations of free intrinsics, scaled by the a-posteriori variance factor
    (reported; IRLS-weighted, so outliers do not inflate it), with unobservable focal
    lengths flagged;
  - a parity fixture of fifteen reference solves (every gauge `auto` resolves to, every
    explicit recipe, partial ones, Gauss-Newton, robust losses, degeneracy, radial, priors
    far from the truth, anchors) that the C++ core reproduces.
- **Readers** (`slamlab.eval.readers`): BAL (plain or bz2) and COLMAP's text model.
- **`slamlab.eval`**: COLMAP's Ceres bundle adjuster as a reference, and reprojection
  metrics.
- **Demos**:
  - `scripts/demo_bal.py`: on BAL Ladybug-49 both solvers reach 0.9147 px.
  - `scripts/compare_ba_colmap.py`: slamlab against COLMAP on any COLMAP text models. On
    reifyscene's ten DAVIS slice scenes (invocation in the BA note):
    Reprojection errors agree within 0.01 px on 8 of 10. Focal length is unobservable on 9
    of 10. COLMAP lets points cross behind cameras; ours never does.
- **Dev dependencies**: pycolmap and rerun-sdk.

### Changed

- The design doc fixes the boundary conventions every binding and fixture bakes in. It also
  sets the per-PR review rule: PRs are slices, steps are commits, and the panel runs once
  per PR. Its status and open items are brought up to date, and the panel's deferrals are
  recorded.
- `scripts/check_layers.py` applies rung 0's internal order (and BA's) to C++ includes, as
  import-linter does to Python imports, chains through unlisted helpers included.
- `camera.unproject_or_nan`: per-pixel unprojection with NaN where the lens cannot be
  inverted (it was BA's private helper).
- pycolmap is pinned once, in the `colmap` extra, which the development setup installs.

### Performance

- The C++ Schur elimination does one product per point, not one per pair of its
  observations. Results are identical, and reifyscene's slice runs 2.3-5.1x faster (judo
  565 s to 111 s).

## [0.1.2.1] - 2026-10-02

### Changed

- **The review panel runs from reifyscene sessions.**
  - Work on slamlab happens from sessions started in reifyscene, so the two projects progress
    together. The panel no longer assumes a session started inside slamlab.
  - The agents are renamed `slamlab-hygiene-reviewer`, `slamlab-architecture-reviewer` and
    `slamlab-algorithm-reviewer`, and the skill `/slamlab-review-panel`, so they cannot clash
    with a future reifyscene panel.
  - Every path in them is now relative to the slamlab root (`REPO`), which the skill passes
    to each reviewer. Before, a reviewer launched from reifyscene would have read
    reifyscene's `CLAUDE.md`.
  - They are symlinked into reifyscene's git-ignored `.claude/`, so they are registered there
    and still single-sourced in slamlab.
  - `CLAUDE.md` gains a "Where sessions run" section.

## [0.1.2.0] - 2026-10-02

### Added

- **Review process in two layers.**

  **Layer 1** (`scripts/check.sh`) runs on every PR, locally and in CI. It holds all three
  languages to one standard:
  - Python: numpy-style docstrings, full type hints and `mypy --strict`, through ruff;
  - C++: clang-format, clang-tidy, and a Doxygen run that fails on any undocumented
    declaration;
  - Rust: rustfmt, and clippy pedantic with `missing_docs` on private items too;
  - all three: jscpd duplication detection and import-linter dependency contracts;
  - the C++, Rust and Python test suites, with every backend required:
    `SLAMLAB_REQUIRE_BACKENDS` turns a missing or broken extension into a failure, not a
    skipped parity test.

  CI builds the conda env with `setup_slamlab.sh` from a pinned, checksum-verified Miniforge
  (read-only token, 30-minute timeout) and runs the same script.

  **Layer 2** (`/review-panel`) runs on PATCH-or-larger changes, with three reviewer agents in
  `.claude/agents/`:
  - hygiene (Sonnet);
  - architecture (Opus);
  - algorithm (Opus, with web search and a reference list grouped by rung).

  Reports are saved in `docs/reviews/`, and each finding's resolution is decided by the owner.

- **Process documents.**
  - A PR template whose "How this was built" section records the agentic workflow.
  - A project `CLAUDE.md` with the conventions, layout, review rules and merge rule.

- **The panel reviewed this change three times** (`docs/reviews/2026-10-02-feature-review-panel.md`).
  - Run 1: 31 findings, 1 of them must-fix.
  - Run 2: 19 findings, none must-fix.
  - Run 3: all three reviewers stated the first version satisfies them, and raised 7 smaller
    items.
  - Run 4 (architecture and algorithm): run 3 resolved. 6 small items were fixed or
    recorded without a fifth run, as the owner chose.

  The owner chose to resolve every finding here. The changes below are that work.

- **C++ and Rust enforce the ladder too.**
  - One C++ library per rung (`slamlab_foundations`) and one Rust crate per rung
    (`slamlab-foundations`, previously `slamlab-core`).
  - `scripts/check_layers.py` reads import-linter's contracts from `pyproject.toml` and
    fails layer 1 on a forbidden cross-rung include, CMake link or crate dependency. It
    covers quoted and angle-bracket includes, sources outside any rung, every Cargo
    dependency table (dev, build, target), renamed and `workspace = true` crates, and
    helper crates that would relay a forbidden rung. Every crate under `rust/` is checked,
    including Cargo's implicit workspace members. There is a test for each form.
  - `check.sh` tests every rung crate: `cargo test --workspace`.

- **An error contract shared by the three languages.**
  - C++23 `std::expected<T, Error>` and Rust `Result<T, Error>`, with one list of kinds:
    `InvalidInput`, `Degenerate` and `NotConverged`.
  - Python raises the matching exception from `slamlab.foundations.errors`. `as_vec3` now
    raises `InvalidInputError`.
  - Iterative solvers will return their best estimate plus a solve summary, as in Ceres and
    SciPy. `NotConverged` is kept for strict mode or divergence (design doc).

### Changed

- **One package per rung, mirrored in every language.**
  - Python: `slamlab.foundations` (rung 0), plus stub packages for `edge`, `vi`, `classic`,
    `dense`, `learned`, `eval` and `orchestration`. Each stub's docstring states its rung,
    milestone and compute budget.
  - C++ and Rust: `cpp/src/foundations/` with `slamlab::foundations`, and
    `slamlab_core::foundations`.
  - Extension submodules: `slamlab._cpp.foundations.so3` and `slamlab_rs.foundations.so3`,
    so names never collide across rungs.
  - Import-linter contracts now cover whole packages:
    - the ladder's layers, with independent sibling branches;
    - visual-inertial builds only on edge, and learned only on dense;
    - the layers contract is exhaustive, so a new top-level package must be placed;
    - no PyTorch, GPU stack or torch-based geometry library on the CPU rungs;
    - no depth-model imports in any algorithm package;
    - slamlab never imports reifyscene.

- **Bindings take and return float64 numpy arrays** (nanobind `ndarray`, the Rust `numpy`
  crate), so batched data will cross without per-element Python objects. Both refuse
  anything else (another dtype, a strided view, a misaligned buffer) rather than silently
  converting it. They are one file
  per module, with thin roots that only wire the rungs.

- **Dispatch moved next to each algorithm.**
  - `slamlab.foundations.so3.hat(v, backend=...)` validates once, then computes with the
    Python reference or calls the chosen extension.
  - `slamlab.backends` now only locates compiled implementations. It reports an installed
    extension that fails to load with its real error, not as "not built".
  - Each algorithm module types its compiled backend with an `Implementation` Protocol. One
    generic test walks every extension submodule and checks its functions and parameter
    names against it. This replaces the mypy exemption for the extensions.

- **Stricter input validation.** `slamlab.foundations.arrays.as_vec3` rejects booleans,
  complex numbers, strings, `None` and masked arrays, as well as NaN and infinity, instead of
  coercing them.

- **Parity fixtures state their tolerance policy.**
  - `tests/conftest.assert_matches_fixture` applies the policy. An exact policy also
    compares the sign of zeros.
  - `hat`'s fixture is compared exactly, and includes values not representable in float32
    (0.1, 1/3, 1e-300), so a float32 backend now fails.
  - The native C++ and Rust tests pin every matrix entry with the basis vectors, and check
    skew-symmetry.

- **Smaller changes.**
  - pytest markers `gpu`, `budget` and `slow`; `check.sh` runs the CPU subset.
  - `check.sh` rejects unknown arguments, and takes its C++ file lists from git.
  - The `dev` extra is the single list of the toolchain's pip layer.
  - The setup script fails unless all three backends load, and derives compiler names from
    the host architecture.
  - The license is declared with an SPDX string.
  - Doxygen writes XML only.

- **Scaffold brought up to the standard.**
  - Numpy-style docstrings, Doxygen and rustdoc throughout.
  - Docstring examples run as tests.
  - Python tests: 14 -> 87.

## [0.1.1.0] - 2026-10-02

### Added

- **Design doc for the SLAM ladder** (`docs/design/2026-10-02-slamlab-ladder.md`). It covers the
  rungs from first-principles geometry to MegaSaM-style dense SLAM, including visual-inertial;
  each language's role; nine milestones, two of them reifyscene payoffs; the one-way boundary
  with reifyscene; edge results from budget-matched simulation; datasets and metrics; and
  renaming before the repository goes public.

- **Three-language scaffold.** One placeholder function, `hat` (the skew-symmetric matrix of a
  3-vector), is implemented in Python (`slamlab.so3`), C++ (`slamlab_core`, bound with nanobind
  as `slamlab._cpp`) and Rust (`slamlab-core`, bound with PyO3 as `slamlab_rs`).
  `slamlab.backends` picks an implementation at runtime. A parity test checks every built backend
  against shared fixtures in `tests/fixtures/`. The C++ and Rust cores also build and test
  standalone, without Python.

- **Conda environment, matching reifyscene.** `scripts/setup_slamlab.sh` builds the env with
  Python 3.11, cmake, ninja, a C++ compiler and Rust, all from conda-forge, then installs the pip
  layer from `scripts/locks/slamlab.lock` (`scripts/gen_env_lock.sh`). Each env gets its own C++
  build folder, `build/<env>`; two Python 3.11 envs sharing one CMake tree silently reused each
  other's cached compiler.

- **CI** (`.github/workflows/test.yml`): runs the Python suite with both extensions built, the
  standalone C++ tests (ctest), and Rust fmt, clippy and tests.

## [0.1.0.0] - 2026-10-02

### Added

- **Repository bootstrap.** `main`/`develop` branches, the `VERSION` file as the single
  version source, this changelog, the PolyForm Noncommercial 1.0.0 license, and the two tag workflows
  (`v<version>` on develop version bumps, `release-v<version>` plus a GitHub Release on
  develop -> main merges).
