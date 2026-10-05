# Changelog

All notable changes to this project are documented here. Format is a
loose variant of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
with 4-digit `MAJOR.MINOR.PATCH.MICRO` version numbers.

## [0.3.0.0] - 2026-10-04

Milestone 1 is complete: Python, C++ and Rust have the same capabilities. The Rust core
gains the estimation half of rung 0, and Rust becomes a lock-step backend. Every SVD in the
Rust core now goes through faer, since nalgebra's were not accurate on ill-conditioned input.
Porting the refinement found a crash in the C++ core, fixed here. Four deferred panel items
are closed, and this PR's own panel findings are resolved.

### Added

- **Correspondences and triangulation in Rust** (`slamlab_foundations::correspondences`,
  `triangulation`): the shared checks and normalisations, two-view triangulation (niter2)
  and the n-view DLT.
- **RANSAC in Rust** (`slamlab_foundations::ransac`): LO-MSAC over an `Estimator` trait,
  drawing the same PCG32 samples as Python and C++, and the refined run that keeps a
  refinement only if it lowers the MSAC cost.
- **Five-point, epipolar and homography in Rust:** Nistér's and Stewénius' solvers, the
  normalised 8-point E and F, Sampson errors, E's decomposition and its refinement on the
  essential manifold, the normalised DLT, Malis and Vargas' decomposition of H, and E-, F-
  and H-RANSAC.
- **Two-view initialisation in Rust** (`slamlab_foundations::twoview`): E and H by RANSAC,
  GRIC model selection and the choice of motion by triangulation.
- **P3P and PnP in Rust:** Lambda Twist and Ding et al.'s P3P, EPnP, the pose refinement on
  SE(3) and PnP RANSAC.
- **The `slamlab_rs` bindings** for every module above, with a shared RANSAC crossing that
  reads options through serde.
- **RANSAC benchmarks** in both native suites: E-RANSAC and PnP RANSAC on one fixed
  synthetic scene. Both cores keep the same inliers in the same number of iterations;
  C++ takes 2.29 and 2.40 ms, Rust 3.39 and 2.05 ms.
- **`linalg`** (Rust): faer's SVD (`svd`, `svd3`, `singular_values3`) and real
  eigendecomposition (`eigen_real`) for the whole crate, and `least_squares`, the
  minimum-norm SVD solve with an explicit cutoff, so each caller states whose convention
  (numpy's or Eigen's) it follows.

### Changed

- **Rust is a lock-step backend.** Every module's tests run on Python, C++ and Rust, and a
  compiled backend missing a module or a function fails them; the skip for a module Rust
  lacked is gone. The demos run Rust by default, and the TUM two-view and absolute-pose
  surveys print the same results on C++ and Rust.
- **The refinement callback takes the inlier indices in every language** (panel A6 on
  PR 4); Python passed a mask.
- **BA's solve has its own module** (`ba/solve.rs`, `ba/solve.cpp`), as Python has
  `ba/solve.py`, so the layer check holds it to the top of the package (panel A13, A24).
  Rust callers name `ba::solve::solve`.
- **Nesting summaries implement `solve::Summary` in Rust** by delegation: `ba::Summary` and
  `ransac::RefinedSummary` (panel A5).
- **Five-point solutions come back in one canonical order in every language** (panel L3):
  each E signed so its first entry within 1e-6 of the largest magnitude is positive (a pure
  translation's skew E ties its two largest entries), then sorted row-major. RANSAC keeps
  the first of equally good models, so the cores' decisions no longer depend on each
  library's null-space basis or Schur form.
- **The refinement callback may fail, in every language** (panel A2): a failed round is not
  kept, where the refiners used to unwrap. `start_motion` returns an error for an E that does
  not decompose, in C++ and Rust.
- **Rust's Stewénius solver** takes its eigenvectors from one eigendecomposition per sample,
  as Eigen and numpy do, and **EPnP's beta steps** use Householder QR when clearly full rank;
  together they recover most of the cost of the accurate SVD.
- **`linalg`** is crate-private and an optional layer at the bottom of rung 0's contract
  (panel A4, A8); the design doc records the cores' runtime dependencies (A7).
- **Python's `start_motion`** raises for an E that does not decompose, as the compiled cores
  return an error (A13).
- **`docs/notes/rust-port.md`** covers the estimation core: the port's choices, the SVD
  finding and the faer switch, the RANSAC benchmarks and the TUM results, and one code-size
  table for all of rung 0, counted by one stated rule.

### Fixed

- **Rust's SVDs are accurate on ill-conditioned input** (panel L1, L2). nalgebra 0.35's 3x3
  SVD squares the condition number, so Rust's Umeyama, and so P3P, returned rotations off by
  up to 0.7 on thin triangles; its dynamic SVD left the 8-point E and F up to 6e-3 from
  Python's on a few near-planar scenes. Every SVD now goes through faer, which matches
  LAPACK to roundoff on both, scaling its input into a safe range first as LAPACK does
  (faer alone gave wrong factors below about 1e-154).
- **C++ `refine_essential` no longer segfaults** on a match at both epipoles, whose Sampson
  distance is 0 / 0. `epipolar::refine` and `pnp::refine` return the solve's error and the
  public refinements raise it as `InvalidInputError`, as Python and Rust do.
- **A failed gauge SVD is `DegenerateError` in Python** too, not numpy's `LinAlgError`
  (panel A20).

### Tests

- **An epipole match raises `InvalidInputError` on every backend** (the crash above).
- **The layer check's facade cases** follow the move: the facade including a package module
  fails as `problem`, and `super::solve` from below fails in Rust.
- **`ill_conditioned.json`** (50-digit mpmath) holds every backend to near-planar 8-point
  scenes, a near-rank-deficient E and thin-triangle Umeyama, the inputs the old SVD got
  wrong.
- **The compiled-side tests run on every compiled backend** (panel A1), not only C++, and
  one test requires every module with an `Implementation` Protocol of each.
- **Native Rust tests** for each new module's main cases, the SVD's graded and thin cases,
  and the failure paths (no model from any sample, an epipole match, a rank-1 E, a point
  behind the camera); the rest of each module's behaviour is covered through the lock-step
  tests.

## [0.2.4.0] - 2026-10-04

The Rust core reaches rung 0's geometry and optimisation, at parity with C++. This is the
first of two PRs that give Python, C++ and Rust the same capabilities at the end of
milestone 1; PR 6 adds the estimation core. Two deferred panel items are closed.

### Added

- **SO(3) and SE(3) in Rust** (`slamlab_foundations::so3`, `se3`): every function, term for
  term with the C++ core, on nalgebra's fixed-size matrices.
- **Camera models and Umeyama alignment in Rust**: pinhole, radtan and radial, with the
  projection Jacobian and the Newton undistortion.
- **PCG32 in Rust** (`slamlab_foundations::rng`), drawing the same samples as Python and
  C++.
- **Non-linear least squares in Rust** (`slamlab_foundations::nls`, `solve`): the generic
  Gauss-Newton and Levenberg-Marquardt loop over `Problem` and `LinearSystem` traits.
- **Bundle adjustment in Rust** (`slamlab_foundations::ba`): the gauge, layout, Schur system
  over a `ReducedSystemSolver` and the bundle's blocks. All fifteen BA parity cases match the
  Python reference, iteration counts included.
- **The `slamlab_rs` bindings** for every module above. They share one batch layer (the
  `batched!` macro), and the BA options are read through serde, with unknown fields denied at
  every depth.
- **Native benchmarks:** Google Benchmark (C++, `cpp/bench/`) and criterion (Rust) time the
  same SO(3), SE(3) and camera operations, and a whole bundle adjustment on a BAL problem.
  On ladybug-49 the C++ solve takes 1.430 s and the Rust solve 1.441 s.
- **`cargo run --example bal`:** bundle adjustment on a BAL problem from Rust alone, with no
  Python.
- **`docs/notes/rust-port.md`:** how the port went, and the C++-versus-Rust speed and size
  comparison.

### Changed

- **`demo_bal.py --backend`** takes `rust` and `all`. On ladybug-49, all three backends
  reach 0.9147 px in 33 iterations.
- **The property tests run on Rust:** every module Rust implements runs them on all three
  backends.
- **Enum names come from the cores (A27).** C++ gains core `name()` functions for the camera
  models, NLS methods and losses, and BA gauges. The BA binding's own tables are gone, and one
  `enum_by_name` reads every choice by name.
- **Options-dict errors read as serde's** (``missing field `x` ``, ``unknown field `x` ``),
  in both bindings.
- **Every compiled backend implements each module it has in full.** The Protocol test and
  `require_function` enforce it for Rust as for C++, and `backends.COMPILED_BACKENDS`
  replaces the hand-written lists (panel A1, A6).
- **Enum codes leave the Rust binding by their position in `ALL`,** the list that names and
  decodes them, and fail loudly for a value missing from it; the Python side decodes every
  code through `backends.decode_name`, which rejects a code outside its table (panel A3,
  A12).
- **`ba::Problem::with_defaults`,** in both cores, fills a problem's optional arrays as
  `BAProblem.create` does; the native BAL readers use it (panel A11).

### Fixed

- **A NaN gradient entry never reads as convergence** (panel L1). Rust's `f64::max` and
  Eigen's default `maxCoeff` dropped NaN from the gradient's norm; both cores now propagate
  it, as the reference does.
- **A non-finite matrix no longer hangs the gauge's SVD** (panel L2). nalgebra's general SVD
  has no iteration cap; one capped, finiteness-checked helper replaces it.
- **A failed SVD while planning the gauge is an error** (`Degenerate`), not a free similarity
  read as "nothing pins anything" (panel L12, Rust).
- **The native cores reject non-finite input** in the rotation, skew, transform and
  quaternion checks and in Umeyama, in both languages (panel L5, H2). NaN passes every
  tolerance comparison.

### Tests

- **PCG32's parity values move to a shared fixture (A9),**
  `tests/fixtures/foundations/rng.json`. The Python, native C++ and Rust tests all read it.
  The native C++ tests read fixtures through nlohmann_json, a test-only conda package.
- **Layer 1 builds and lints the benchmarks,** and holds `cpp/bench` to the ladder like the
  rest of the code.
- **`check_layers.py` holds Rust's modules inside a rung to the internal contracts,** reading
  their `crate::` and `super::` paths, as it does C++'s includes, and fails on what it cannot
  follow: glob imports, re-exports in a facade, test code outside the trailing test module
  (panel A2, A8). It also resolves quoted relative includes to the rung they land in (A9).
- **The native BAL readers are tested against the Python reader,** every field and default,
  through a shared fixture, `tests/fixtures/eval/bal.json` (panel A4, A10, L10).
- **Native Rust tests for bundle adjustment:** a noise-free scene recovered, degenerate
  points dropped, malformed problems rejected, and the Schur reduction (panel H1).

## [0.2.3.0] - 2026-10-03

Absolute pose, completing milestone 1's slice B: a frame placed against known 3D points, from
three correspondences to a refined, robust pose. Python reference and C++ core in lock-step,
with identical RANSAC runs on the same seed. Three deferred panel items folded in.

### Added

- **P3P** (`foundations.p3p`): Ding et al.'s solver (CVPR 2023, the default) and Lambda Twist
  (ECCV 2018) behind a `solver` option, following PoseLib's implementations; every pose
  returned is a rigid motion with every point in front. Ding's always intersects both lines of
  the degenerate conic (PoseLib's single-root shortcut dropped true poses near the danger
  cylinder).
- **EPnP** (`pnp.epnp`): Lepetit et al.'s least-squares PnP from five points, following
  OpenCV's implementation; planar points raise `Degenerate`.
- **The pose refinement** (`pnp.refine_pose`): Levenberg-Marquardt on SE(3) of the
  Cauchy-robust reprojection error, with an analytic Jacobian.
- **PnP RANSAC** (`pnp.pnp_ransac`): LO-MSAC with a P3P minimal solver and EPnP refits, the
  consensus refined and kept only if the MSAC cost drops.
- **Generic refined RANSAC** (`ransac.RefinedSummary`, `refine_consensus`,
  `estimate_refined`, in both languages), shared by E-RANSAC and PnP RANSAC: refine the
  consensus, re-collect the inliers and repeat while the MSAC cost drops
  (`REFINE_ROUNDS = 4`; `RefinedSummary.rounds`).
- **`correspondences.normalised_xy`** and **`p3p.rigid_pose`**, shared helpers.
- **`backends.call`**: one dispatch for functions whose two sides differ, decoding the compiled
  result with the module's `codes()` (PR 2 panel A8).
- **TUM depth maps**: `tum.Sequence.depths` and `tum.DEPTH_SCALE`.
- **Demo** `scripts/demo_absolute_pose.py`: PnP on TUM with points from depth or from two-view
  initialisation, against the motion capture and OpenCV, with a survey mode.
- **Parity fixtures** `p3p.json` and `pnp.json` (`generate_pose.py`), and the design note
  `docs/notes/absolute-pose.md`.

### Changed

- `epipolar.EssentialSummary` is now `ransac.RefinedSummary` (`ransac::RefinedSolution` in
  C++); `essential_summary_from_backend` is `ransac.refined_summary_from_backend`.
- E-RANSAC's refinement now runs in refine-and-reselect rounds: on the synthetic sweep the
  median error is 0.78 of the 8-point floor (0.71 with one refinement), the p90 2.94 (3.21),
  the recall 0.953 (0.950).
- `ransac.run` takes the reference, the compiled arguments and the summary decoder.
- The design doc's pose convention: rung 0's estimators return the transform they name
  (`T_cam_world`, `T_c2_c1`); trajectory results stay `T_world_cam`.
- `codes()` keys are qualified: `ransac_termination` and `nls_termination` everywhere (PR 3
  panel A7); `nls.summary_from_backend` takes the codes dict.
- BA's options cross the bindings as one nested dict of `BAOptions`' fields, choices by name,
  read strictly at every depth (PR 2 panel A7).
- `nls` sits one layer lower in rung 0's order, so `ransac` may use it.
- The bindings share one enum-code decoder, `enum_names` and the `rows3`/`rows2` helpers;
  `pnp.codes()` carries `minimal_solver`.

### Fixed

- `twoview.initialize` no longer raises when H-RANSAC returns a rank-1 homography (duplicate
  keypoints at one pixel, TUM fr1/desk): such an H is no model, and E decides.

## [0.2.2.0] - 2026-10-03

The 5-point solver and the Sampson refinement of E (milestone 1, slice B): E-RANSAC samples
five matches instead of eight and refines its consensus on the geometric error. Python
reference and C++ core in lock-step.

### Added

- **The 5-point solvers** (`foundations.fivepoint`): Stewénius' action matrix (the default)
  and Nistér's degree-10 polynomial with Sturm-sequence root isolation, behind a `solver`
  option; each solution is polished to convergence on the cubic constraints. Only essential
  solutions are returned.
- **The Sampson refinement** (`epipolar.refine_essential`): Levenberg-Marquardt on the inliers'
  robust (Cauchy) Sampson distance over E's five-dimensional manifold, with an analytic
  Jacobian, returning an `nls.Summary`.
- **E-RANSAC options**: `essential_ransac(minimal_solver=, refine=)` and
  `twoview.Options(essential_solver=, refine=)`; the demo's `--essential-solver` and
  `--no-refine`.
- **`epipolar.EssentialSummary`**: E-RANSAC's summary nests the RANSAC run's and the
  refinement's, with whether the refined E was kept; the two-view result carries it.
- **Parity fixtures** `fivepoint.json` and `refine_essential` in `epipolar.json`.
- **Design note** `docs/notes/five-point.md`, with the synthetic sweep, the TUM survey and the
  near-degenerate recall.
- PoseLib in the dev extra, as a test-only oracle; an import-linter contract keeps every test
  oracle out of the library.

### Changed

- E-RANSAC's default minimal solver is Stewénius' 5-point, and it refines its consensus by
  default (kept only if the MSAC cost drops). On TUM fr1/xyz the initialiser now initialises
  24 of 38 pairs (was 22) at a median 0.49° rotation and 2.33° translation-direction error (was
  0.88° and 4.16°). `essential_solver="8pt", refine=False` reproduces 0.2.1.0.
- `essential_ransac` returns an `EssentialSummary` instead of a `ransac.Summary`.
- A compiled backend's NLS summary crosses the bindings as one nested dict
  (`nls.summary_from_backend`), shared by BA and the refinement.
- The design doc's binding conventions say how a choice field crosses (by name inside an
  options dict, as a code on its own).

### Tests

- The two-view fallback test's desk scene is 90% planar (was 80%): the refined E erased H's
  GRIC margin on the old scene.

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
- **Correspondences** (`foundations.correspondences`): the unchecked building blocks the
  two-view estimators share (homogeneous and normalised points, Hartley's normalisation, the
  null vector, `PairEstimator`), with their input checks in `foundations.arrays` and
  `ransac.run`, so `epipolar` and `homography` are independent siblings.
- **Epipolar geometry** (`foundations.epipolar`): the normalised 8-point essential and
  fundamental matrices, their Sampson distances, RANSAC, and E's four-pose decomposition.
- **Homographies** (`foundations.homography`): the normalised DLT, its Sampson distance,
  RANSAC, and Malis and Vargas' decomposition.
- **Two-view initialisation** (`foundations.twoview`): E and H by RANSAC, Torr's GRIC over
  the matches either explains, the motion chosen by triangulation, and a status
  (`initialized`, `pure_rotation`, `low_parallax`, `ambiguous`) instead of an exception;
  an ambiguous H falls back to E. H-RANSAC's threshold is E's scaled for its two degrees of
  freedom.
- **`slamlab.eval.tum`**: TUM RGB-D sequences with their ground-truth poses and calibrations.
- **`scripts/demo_two_view.py`**: initialisation on a TUM pair against the ground truth and
  OpenCV. On fr1/xyz frames 0 and 30: 0.47 deg rotation and 2.4 deg translation-direction
  error (OpenCV's 5-point RANSAC: 1.30 and 5.1); 4-6 ms in C++. `--survey` runs 38 pairs.
- An import-linter contract keeps `cv2` out of slamlab.
- Design notes: `triangulation.md`, `ransac.md`, `two-view.md`.

### Changed

- NLS's `SolveSummary` is `nls.Summary`, and its `Solution` is `solve.Solution`.
- The bindings' array copy helpers moved from `ba.cpp` to `common.hpp`.
- `opencv-python-headless` joins the dev extra (the demo's feature matching only).

### Fixed

- An estimated homography's candidate rotations, and a pure rotation's, are projected onto
  SO(3); the closed form is only exact for an exact H.
- `opencv-python-headless` is in the dev extra, not only the lock.

### Review

- The full panel (hygiene, architecture, algorithm) ran on the slice: 3 must-fix, 13
  should-fix and 10 notes, each decided with the owner
  (`docs/reviews/2026-10-02-feature-two-view.md`).

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
