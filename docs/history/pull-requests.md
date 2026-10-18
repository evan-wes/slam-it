# Pull-request history

SLAM It!'s history was rewritten on 2026-10-18, before going public, and its GitHub repository
was recreated. The first repository's pull requests, 1–22 (2026-10-02 to 2026-10-17), can
no longer be read on GitHub; the project was named slamlab, then SLAM It All!, then SLAM It! over
that time. Their merge commits and release tags name them "slam-it pull request N (first
repository)". This page keeps each one's title, branch, merge date, size, merge commit and
description. Pull requests in the current repository start again at 1 and live on GitHub.

Descriptions are as written at the time, except that paths on the author's machine and links to
private notes and sessions are removed, links to the old repository's files point into this one
(or name the file, where it no longer exists), and headings are shifted to fit this page.
Numbers such as "#12" inside a description refer to the first repository's pull requests, listed
here. Each date is its merge commit's own, in the author's time zone, as the changelog dates
releases. The merge commit in each heading is its ID in this history; commit IDs quoted in
descriptions, review reports, notes and commit messages are the first repository's, and
[`commit-map.txt`](commit-map.txt) gives each one's ID in this history.

## The first repository

### <a id="slam-it-pr-1"></a>slam-it pull request 1: Design doc for the SLAM ladder and a three-language scaffold

Merged 2026-10-02 · `feature/scaffold-and-design` → `develop` · 1 commit, +973 −2 in 27 files · merge commit `2eb4e661ec3845d980071c8e9fea8830a43c4cff`

#### Summary

- **Design doc** (`docs/design/2026-10-02-slamlab-ladder.md`): the SLAM ladder from first-principles geometry to MegaSaM-style dense SLAM, plus visual-inertial. It covers:
  - the role of each language (Python reference, C++ performance core, Rust subset);
  - nine milestones, two of which pay off in reifyscene;
  - the one-way boundary with reifyscene;
  - edge results from budget-matched simulation;
  - datasets and metrics;
  - renaming the repo before it goes public.
- **Three-language scaffold.** One placeholder, `hat(v)`, is implemented in Python, C++ (`slamlab._cpp`, nanobind) and Rust (`slamlab_rs`, PyO3). `slamlab.backends` picks one at runtime, and a parity test checks each against shared fixtures. The C++ and Rust cores also build standalone.
- **Conda environment matching reifyscene.** `scripts/setup_slamlab.sh` builds it with Python 3.11 and a conda-forge toolchain (cmake, a C++ compiler, Rust), then installs a locked pip layer. Each environment gets its own C++ build folder.
- **CI** (`test.yml`) has three jobs: Python with both extensions built, standalone C++ (ctest), and Rust (fmt, clippy, tests).
- Version 0.1.0.0 → 0.1.1.0.

#### Test plan

- [x] Locally: pytest passes 14/14 with all three backends loaded; ruff is clean.
- [x] Locally: the standalone C++ test passes (ctest).
- [x] Locally: the Rust unit test passes; clippy (`-D warnings`) and rustfmt are clean.
- [x] `setup_slamlab.sh` ran end to end on a fresh environment, using that environment's own compiler.
- [x] After a C++ source change, the next import rebuilds the extension.
- [x] Both packages build as wheels the way CI builds them.
- [ ] The CI workflow passes on this PR. It runs for the first time here.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-2"></a>slam-it pull request 2: Two-layer review process, and a first version that satisfies the review panel

Merged 2026-10-02 · `feature/review-panel` → `develop` · 8 commits, +4,346 −288 in 74 files · merge commit `a82589bbd21f1a169fb426dbe9d3bac99a10e04f`

#### Summary

- **Two-layer review process.**
  - **Layer 1:** `scripts/check.sh`, 16 deterministic checks that run on every PR, locally and in CI. It holds Python, C++ and Rust to one documentation and quality standard, and enforces the ladder's layering in all three languages: import-linter for Python, `scripts/check_layers.py` for C++ and Rust.
  - **Layer 2:** `/review-panel`, with three reviewer agents (hygiene, architecture, algorithm) for PATCH-or-larger changes. Reports are saved in `docs/reviews/`, and the owner decides every finding.
- **The scaffold, rebuilt to satisfy the panel:**
  - one package per rung, mirrored as a C++ library and a Rust crate per rung;
  - each algorithm dispatches through its own `backend=` argument;
  - bindings take and return float64 numpy arrays, one binding file per module;
  - one error contract across the three languages (`std::expected`, `Result`, matching Python exceptions);
  - iterative solvers will return their best estimate plus a solve summary;
  - a generic Protocol test over every compiled submodule;
  - parity fixtures carry an explicit tolerance policy;
  - strict, shared input validation.
- **Process documents:** a project `CLAUDE.md`, this PR template, and the design doc updated to match the code.
- Version `0.1.1.0` -> `0.1.2.0` (PATCH).

#### How this was built

slamlab is built through a human-directed agentic workflow. Claude Code agents write and review the code. The owner makes the design decisions, decides every review finding, and approves every merge. This PR is the first to go through the review panel, which reviewed the panel's own introduction.

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Built the review process and scaffold, applied the owner's decisions after each run, and verified with layer 1 and a fresh-env build |
| Hygiene review | `hygiene-reviewer` | sonnet | Runs 1-3; signed off in run 3 |
| Architecture review | `architecture-reviewer` | opus | Runs 1-4; satisfied from run 3, with the last items fixed after run 4 |
| Algorithm review | `algorithm-reviewer` | opus | Runs 1-4; declared the version PR-ready in run 4 |

**Review panel:** full panel in runs 1-3; architecture and algorithm only in run 4, by the owner's choice. Run 4's items were small and were fixed without a fifth run, also by the owner's choice, and verified by layer 1. The reviewers ran as general-purpose agents carrying each agent file's role and model, because the session started outside the repo.
Report: [`docs/reviews/2026-10-02-feature-review-panel.md`](../../docs/reviews/2026-10-02-feature-review-panel.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 1 | 1 | 0 | 0 | 0 |
| should-fix | 24 | 24 | 0 | 0 | 0 |
| note | 38 | 33 | 4 | 1 | 0 |

The 4 deferred notes are milestone 1 tasks recorded in the design doc. They need code that doesn't exist yet: exp/log hard cases, the bindings' error translation test, the solve-summary details, and the extra exp/log edge cases. The waived note is the `hat` placeholder's limited scope.

**Owner decisions**

- **Run 1:**
  - Resolve all 31 findings in this PR, including the package-per-rung restructure. "We are just getting the repo stood up, so do all changes needed to make the first version satisfy all of the reviewers."
  - Approved the restructure design: stub packages, mirrored submodules, per-algorithm dispatch.
- **Run 2:**
  - float64 numpy arrays at the binding boundary;
  - per-rung C++ and Rust enforcement now;
  - the C++23 `std::expected` / Rust `Result` error contract with one shared list of kinds;
  - all other fixes.
- **Run 3:**
  - all fixes;
  - iterative solvers return the estimate plus a solve summary (Ceres and SciPy style);
  - confirm with the architecture and algorithm reviewers only.
- **Run 4:** all fixes and records; proceed to the PR without a fifth run.
- **Merge rule:** slamlab PRs may be merged by Claude, but only after a selected Yes on a yes/no prompt for that PR. Granted 2026-10-02 and enforced by a hook.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally, 16 checks and 103 Python tests, with all three backends required.
- [x] A freshly built env (`setup_slamlab.sh --unlocked`) passed all of layer 1 after the run 1 restructure.
- [x] Probes, each shown to fail and then reverted:
  - import-linter breaks on a torch import and on a sibling import;
  - a missing required backend fails the suite, rather than skipping;
  - the Protocol test catches a renamed parameter in both languages;
  - the layer check catches every bypass form the reviewers found, each with a test.
- [ ] CI passes on this PR. It is the first CI run since the C++23, per-rung crate and numpy-binding changes.

#### Version

`0.1.1.0` -> `0.1.2.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-3"></a>slam-it pull request 3: Run the review panel from reifyscene sessions

Merged 2026-10-02 · `fix/panel-from-reifyscene` → `develop` · 1 commit, +92 −36 in 12 files · merge commit `2df4895c02d84bc1c5cd22617d3dff33c0439fa1`

#### Summary

- Work on slamlab now happens from sessions started in **reifyscene**, so the two projects progress together. The review panel no longer assumes a session started inside slamlab.
- The agents are renamed `slamlab-hygiene-reviewer`, `slamlab-architecture-reviewer` and `slamlab-algorithm-reviewer`, and the skill `/slamlab-review-panel`, so they cannot clash with a future reifyscene panel.
- Every path in the agents and the skill is anchored to the slamlab root (`REPO`), which the skill passes to each reviewer. Before this, a reviewer launched from reifyscene would have read reifyscene's `CLAUDE.md`.
- `CLAUDE.md` gains a "Where sessions run" section. The design doc, PR template, reviews README and report template use the new names. The historical review report is left as written.
- Outside this repo, and not part of the PR: the agents and skill are symlinked into reifyscene's git-ignored `.claude/`, and a reifyscene memory entry points to this `CLAUDE.md`.
- Version `0.1.2.0` -> `0.1.2.1` (MICRO).

#### How this was built

slamlab is built through a human-directed agentic workflow. Claude Code agents write and review the code; the owner makes the design decisions, decides every review finding, and approves every merge.

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Renamed and re-anchored the panel files, and set up the reifyscene symlinks and memory |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Not run: MICRO change |
| Architecture review | `slamlab-architecture-reviewer` | opus | Not run: MICRO change |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Not run: MICRO change |

**Review panel:** layer 1 only (MICRO change: process files and docs, no code).

**Owner decisions**

- Keep the process centred on reifyscene, never running sessions inside slamlab. Quote: "I need to keep the process centric to reifyscene so things progress together".
- Wire the panel in with three changes, scoped to reifyscene: (a) a memory pointer, (b) symlinks into reifyscene's git-ignored `.claude/`, and (c) this PR.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (16 checks, 103 tests).
- [x] The symlinks in reifyscene resolve to the renamed files, and the agent frontmatter reads `slamlab-*-reviewer`.
- [ ] CI passes.
- [ ] The renamed agents and skill register in a new reifyscene session. Agents and skills are discovered at session start, so this is checked next session.

#### Version

`0.1.2.0` -> `0.1.2.1` (MICRO), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-4"></a>slam-it pull request 4: BA spine: rung 0 geometry, NLS and bundle adjustment (milestone 1, PR 1)

Merged 2026-10-02 · `feature/ba-spine` → `develop` · 42 commits, +39,755 −523 in 102 files · merge commit `afbab9be5fb7fd645dadb8f389d40097f5349439`

#### Summary

PR 1 of milestone 1, the bundle-adjustment spine: rung 0's geometry and estimation, from Lie groups to a bundle adjuster that matches Ceres on real data. Python reference and C++ core in lock-step throughout. Each component has a design note in `docs/notes/`.

- **Lie groups** (`so3`, `se3`, `alignment`), with mpmath-oracled parity fixtures.
- **Camera models:** pinhole, radtan and radial (COLMAP `RADIAL`, BAL). Each has projection, Jacobians and per-pixel unprojection.
- **Non-linear least squares on manifolds:** Gauss-Newton and Levenberg-Marquardt, IRLS robust losses per block, and a full trace.
- **Bundle adjustment:**
  - Schur complement behind a `ReducedSystemSolver` seam.
  - XYZ or anchored inverse-depth points.
  - Depth priors through per-camera affines.
  - Decoupled pose priors: centre and orientation. A GNSS-style centre-only prior works.
  - **The gauge removed exactly:** the null space of what fixed poses, fixed points, priors and metric depth pin, including partial anchoring. It is built in the scene's own frame.
  - The solve runs in a world centred on the observed cameras, so geo-referenced scenes (up to ECEF range) solve as at the origin.
  - Calibrated intrinsics deviations (the a-posteriori variance factor), with unobservable focal lengths flagged.
- **`slamlab.eval`:** BAL and COLMAP readers, COLMAP's Ceres BA as a reference, reprojection metrics, and two demos.
  - On BAL Ladybug-49 both solvers reach 0.9147 px.
  - On reifyscene's ten-scene DAVIS slice, reprojection agrees within 0.01 px on 8 of 10. Focal length is unobservable on 9 of 10, the critical-motion degeneracy.

#### How this was built

slamlab is built through a human-directed agentic workflow: Claude Code agents write and review the code; the owner makes the design decisions, decides every review finding, and approves every merge. This section is the record of that process for this PR.

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | Built the spine as semantic commits, then applied every fix the owner chose |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran in runs 1-3 |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran in runs 1-3 |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran in runs 1-5 (alone in runs 4-5, whose fixes were algorithmic) |

**Review panel:** full panel in runs 1-3, then the algorithm reviewer alone in runs 4-5. Each run re-reviewed the previous run's fixes.
Report: [`docs/reviews/2026-10-02-feature-ba-spine.md`](../../docs/reviews/2026-10-02-feature-ba-spine.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 10 | 10 | 0 | 0 | 0 |
| should-fix | 53 | 53 | 1 | 0 | 0 |
| note | 29 | 19 | 8 | 6 | 0 |

Five findings were split, partly fixed and partly deferred or waived, so they count in two columns: H12, A3, H20, A27, H33.

**How each must-fix was resolved**

| Run | Finding | Resolution |
|---|---|---|
| 1 | H1: C++ read anchor indices before checking their length | Index checks come first (aa41aec) |
| 1 | A1: no BA parity fixture | Fixture of reference solves, now 15 cases, reproduced exactly by C++ (dbfb56a, later extended) |
| 1 | A2: reference and C++ failed differently on an unvalidated problem | `BAProblem` validates however it is built, and its arrays are read-only copies (aa41aec, e14132e) |
| 1 | L1: the auto gauge froze a pose on motion-only BA, and left scale free with one prior | Minimal gauge (3d0a5a0), then the general null-space gauge (f24d033) |
| 1 | L2: an anchor pixel outside the invertible region made the cores fail differently | Dropped at the start, step rejected mid-solve, in both cores (3d0a5a0) |
| 2 | L11: partial anchoring (fixed points, attitude or GNSS priors) fixed a whole pose and biased the result | General null-space gauge with pose-by-pose confinement, owner's choice ("General gauge now") (f24d033) |
| 3 | L17: the pose prior's translation half did not pin the camera centre | Decoupled residual `(C − C̄, log(R̄_wcᵀ R_wc))`, owner's choice (e14132e) |
| 3 | L18: the gauge was lost from about 5e4 m from the origin | Gauge rows built in the scene's own frame (e14132e) |
| 4 | L23: far from the origin, steps about the world origin drifted the gauge and stalled LM | Solve in a centred world, owner's choice (centre only) (05fcae5) |
| 5 | L27: an unobserved placeholder camera pulled the centre off the scene | Centre on the observed cameras (ba7bf0e) |

**Owner decisions**

- **Design choices made in the course of the work:**
  - Build anchored inverse depth now.
  - Add the radial model.
  - Use COLMAP's Ceres BA as the reference.
  - Use rerun for visualisation.
  - Judge on the whole investigation slice, not lab-coat alone; spin is listed, and reifyscene is left unchanged.
  - Make the speed fixes (2.3-5.1x, identical results).
- **Per-run decisions, all made through selectable prompts:**
  - Run 1: fix all five must-fixes; should-fix and notes as recommended.
  - Run 2: the general gauge rather than classify-and-refuse; its design (rename to `confined_pose`, report `partial`) approved separately; the rest as recommended.
  - Run 3: decoupled prior residual, normalised gauge frame, the rest as recommended.
  - Run 4: centre only, no rescaling; the rest as recommended.
  - Run 5: as recommended, then open the PR rather than run a sixth review.
- **Deferred, recorded in the design doc** ("Deferred from PR 1's review panel"):
  - batched APIs for standalone callers (A11);
  - a sparse reduced solver in rung 0 (A12);
  - the summary's move to `foundations.solve` with RANSAC in PR 2 (A3);
  - a per-block step tolerance (L6);
  - Triggs' robust-loss correction (L7);
  - facade edge cases in the layer check (A24);
  - lifting the gauge to `foundations` with pose-graph optimisation (A26);
  - one enum-name mechanism (A27);
  - σ_f with the gauge left free (L30).
- **Waived, each with its reason in the report:**
  - H12's minor items;
  - H20's style items;
  - L16 and L32 (termination differences at the rounding floor);
  - L22 (redundancy counting);
  - H33's fixture regex.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally on the final code (ruff, mypy --strict, import-linter's 10 contracts, the C++/Rust layering check, clang-format, clang-tidy, doxygen, ctest with 7 executables, rustfmt, clippy, Rust tests, jscpd, pytest).
- [x] CI passes (layer-1 job, 6m22s)
- Parity: every Python/C++ test runs in both cores; the BA fixture's 15 cases match in both.
- Far-origin tests at 1e4, 1e6 and 6.4e6 m, from a start 0.05 off and with an unobserved camera, match the at-origin solve.
- Slice comparison against COLMAP, re-measured after the variance-factor change: table in `docs/notes/bundle-adjustment.md`.

#### Version

`0.1.2.1` -> `0.2.0.0` (MINOR: the milestone-1 batch), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-5"></a>slam-it pull request 5: Milestone 1 slice B: two-view geometry (triangulation, RANSAC, E/F/H, initialisation)

Merged 2026-10-03 · `feature/two-view` → `develop` · 27 commits, +28,218 −157 in 79 files · merge commit `5d5606450a39d45ff04449ace0b2abad5f1fc811`

#### Summary

Milestone 1, slice B up to the 5-point solver: **two-view geometry**, from feature matches between two calibrated views to their relative motion and first map points, or a stated reason why not. Python reference and C++ core in lock-step; the two backends make identical RANSAC runs on the same seed.

- **`foundations.solve`**: the shared solver contract (`Summary` Protocol / concept, generic `Solution`); each solver owns its summary (settles PR 1's panel A3).
- **`foundations.rng`**: PCG32, identical in both backends, so RANSAC samples match sample for sample.
- **`foundations.triangulation`**: Lindstrom's optimal two-view method and the N-view DLT on bearings, with cheirality and parallax per point.
- **`foundations.ransac`**: generic LO-MSAC (MSAC scoring, adaptive bound, Lebeda et al.'s local optimisation); `ransac.run` dispatches to a backend.
- **`foundations.correspondences`** (+ validators in `arrays`): the shared two-view building blocks, so `epipolar` and `homography` are independent siblings for PR 3 and PR 4.
- **`foundations.epipolar`**: normalised 8-point E and F, Sampson distances, RANSAC, E's decomposition.
- **`foundations.homography`**: normalised DLT, Sampson distance, RANSAC, Malis-Vargas decomposition.
- **`foundations.twoview.initialize`**: E and H by RANSAC (H's threshold scaled for its two degrees of freedom), Torr's GRIC over the matches either explains, the motion chosen by triangulation, a status instead of an exception, and E deciding when H is ambiguous.
- **`eval.tum`** and **`scripts/demo_two_view.py`**: TUM RGB-D fr1/xyz against the ground truth and OpenCV; `--survey` runs 38 pairs.
- Design notes `triangulation.md`, `ransac.md`, `two-view.md`; design doc status, conventions and deferrals.

On fr1/xyz frames 0 and 30: initialised (E after an ambiguous H), 0.47° rotation and 2.4° translation-direction error, against OpenCV's 5-point RANSAC at 1.30° and 5.1°; 4-6 ms in C++. Over 38 pairs: 22 initialise, median 0.88° / 4.2° (OpenCV 1.12° / 4.3°), 2 wrong (OpenCV 3).

Rust is out of scope (the curated subset; PR 5). The 5-point solver is PR 3, absolute pose PR 4; both branch from `develop` after this lands. No other PR is open, so there is no merge order to follow.

#### How this was built

slamlab is built through a human-directed agentic workflow: Claude Code agents write and review the code; the owner makes the design decisions, decides every review finding, and approves every merge.

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Built the slice as semantic commits (layer 1 green on each), the fixtures from a 50-digit mpmath oracle, the TUM measurements, and every panel fix |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran in run 1; not re-run in run 2 (its fixes were mechanical) |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran in runs 1 and 2 |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran in runs 1 and 2, with numerical probes on both backends and TUM |

**Review panel:** full panel (run 1), then architecture and algorithm on the fixes (run 2).
Report: [`docs/reviews/2026-10-02-feature-two-view.md`](../../docs/reviews/2026-10-02-feature-two-view.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 3 | 3 | 0 | 0 | 0 |
| should-fix | 17 | 17 (A7: BA's migration deferred) | 0 | 0 | 0 |
| note | 13 | 7 | 3 | 3 | 0 |

Must-fix resolutions:
- **L1:** a `pure_rotation` pose was not in SO(3) (the decomposition's single-candidate branch returned the scaled H). Projected in both backends; a test reaches the branch.
- **A1:** shared helpers lived in `epipolar`, making homography → epipolar → triangulation a false layering. Moved to `arrays`, `correspondences` and `ransac.run`; epipolar and homography are siblings.
- **A2:** OpenCV was in the lock but not the dev extra. Added.

**Owner decisions**

- The slice split: this PR is two-view relative pose; 5-point is PR 3, absolute pose PR 4, the Rust showcase PR 5.
- A3: each solver owns its summary; `foundations.solve` holds the shared contract.
- A shared PCG32 for exact RANSAC parity across backends.
- TUM RGB-D fr1/xyz for the demo.
- When H is ambiguous, E decides if it is unambiguous with enough parallax (chosen over refusing, 3 of 38 pairs, and an E-H agreement check, 18 and 1 wrong), after measuring all three on 38 pairs.
- Run 1: every finding decided as recommended (must-fix individually, the rest in bulk); L2 committed after the re-measured TUM table showed no decision changed.
- Deferred to the design doc's open items: BA's options as a dict (A7), a generic `backends.dispatch` (A8), PCG32's parity values as a fixture (A9), state-of-the-art pointers (L5). Waived: H9, A10, L3 (no action needed).
- Run 2: every finding decided as recommended; no run 3 (layer 1 only).

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (1170 Python tests passed, 337 skipped; 14 C++ test programs; Rust; ruff, mypy --strict, import-linter, layering, clang-format, clang-tidy, doxygen, jscpd)
- [ ] CI passes
- Parity: fixtures from a 50-digit mpmath oracle or exact ground truth for triangulation, the 8-point solvers, the DLT, Sampson distances, both decompositions, whole RANSAC runs and initialisation; noisy runs agree decision for decision across backends.
- `python scripts/demo_two_view.py` and `--survey` on TUM fr1/xyz reproduce the numbers above and in `docs/notes/two-view.md`.

#### Version

`0.2.0.0` -> `0.2.1.0` (PATCH: new modules), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-6"></a>slam-it pull request 6: feat: the 5-point solver and the Sampson refinement of E (0.2.2.0)

Merged 2026-10-03 · `feature/five-point` → `develop` · 12 commits, +7,960 −269 in 43 files · merge commit `e9cdeccad9b2fad5d113d613998b059204b04b97`

#### Summary

Milestone 1, slice B: the 5-point essential matrix and the Sampson refinement of E.

- **5-point solvers** (`foundations.fivepoint`): Stewénius' action matrix (the default) and Nistér's degree-10 polynomial with Sturm sequences, both polished to convergence on the cubic constraints. Only essential solutions are returned, without duplicates.
- **Sampson refinement** (`epipolar.refine_essential`): Levenberg-Marquardt (`foundations.nls`) on the inliers' Cauchy-robust Sampson distance over E's 5-DOF manifold, with an analytic Jacobian. E-RANSAC keeps the refined E only if the MSAC cost drops.
- **E-RANSAC** takes `minimal_solver=` (`stewenius`, `nister`, `8pt`) and `refine=`, and returns an `EssentialSummary` that nests the RANSAC run and the refinement's solve. `twoview.Options` gains `essential_solver` and `refine`; the demo gains `--essential-solver` and `--no-refine`.
- **Measured.**

  | | Error vs 8-point floor | RANSAC samples |
  |---|---|---|
  | 8-point (PR 2) | 1.06 | 738 |
  | 5-point | 0.93 | 110 |
  | 5-point + refinement | **0.71** | 110 |

  (40 synthetic scenes, 40% outliers, 1 px noise.)

  On TUM fr1/xyz the initialiser now initialises 24 of 38 pairs (was 22), at a median 0.49° rotation and 2.33° translation-direction error (was 0.88° and 4.16°). `essential_solver="8pt", refine=False` reproduces PR 2 exactly. The design note is [`docs/notes/five-point.md`](../../docs/notes/five-point.md).
- **Shared plumbing.** An NLS summary crosses the bindings as one nested dict (BA and the refinement). There are parity fixtures for `essential_5pt`, `refine_essential` and every RANSAC solver choice, and an import-linter contract keeps the test oracles (PoseLib, mpmath, manifpy, SciPy) out of the library.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Python reference, C++ core and bindings, tests, fixtures, measurements, docs |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | ran: run 1 (full PR), run 2 (fixes) |
| Architecture review | `slamlab-architecture-reviewer` | opus | ran: run 1 (full PR), run 2 (fixes) |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | ran: run 1 (full PR), run 2 (fixes) |

**Review panel:** full panel, two runs.
Report: [`docs/reviews/2026-10-03-feature-five-point.md`](../../docs/reviews/2026-10-03-feature-five-point.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 2 | 2 | 0 | 0 | 0 |
| should-fix | 15 | 15 | 0 | 0 | 0 |
| note | 19 | 13 | 3 | 3 | 0 |

The must-fixes:
- **A1:** parity fixtures for the new compiled functions.
- **L1:** a test for the repeated-pair degenerate sample.

**Owner decisions**

- Scope: Nistér's solver, with Stewénius' behind a toggle, and the deferred Sampson refinement in this PR. The 8-point stays a third option, and refinement is on by default.
- **Default switched to Stewénius after the re-measure.** The algorithm reviewer found Nistér losing the true E on near-degenerate samples. Polishing both solvers to convergence left them identical on generic scenes and equally fast. Near degeneracy (forward motion, 1% baseline, 300 exact samples) Nistér misses 105 times and Stewénius 18, so the owner chose Stewénius.
- All run-1 recommendations accepted (must-fixes individually, the rest in bulk).
  - Deferred to the design doc's open items:
    - A6: where refinements live, decided before PR 4.
    - L5: refining inside LO.
    - A7: `codes()` key naming.
  - Waived:
    - A8: `ransac.run`'s pass-through; later removed by the A4/A5 fix anyway.
    - A10: the three parameter names.
    - L7: the refinement stops ~0.006° from SciPy's optimum.
- All run-2 recommendations accepted; no run 3 (layer 1 only).

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (1228 Python tests, ctest 15/15, clang-tidy, mypy --strict, import-linter, jscpd)
- [ ] CI passes
- Python and C++ agree run for run: same samples, iterations, inliers and refinement, with E to 1e-12, for every solver with and without refinement, and through two-view initialisation.
- With the polish switched off, 7 of `fivepoint.json`'s 12 cases fail.
- The refinement is checked three ways:
  - against SciPy's `least_squares` on the same problem (same cost to 1e-6);
  - against finite differences (the gradient);
  - from a perturbed start on exact data (it converges to the true E).
- PoseLib is an oracle: every PoseLib solution that satisfies the cubic constraint is one of ours.

#### Version

`0.2.1.0` -> `0.2.2.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-7"></a>slam-it pull request 7: feat: absolute pose — P3P, EPnP, pose refinement and PnP RANSAC (0.2.3.0)

Merged 2026-10-03 · `feature/absolute-pose` → `develop` · 14 commits, +12,432 −551 in 61 files · merge commit `c2592c408e7724d69756d5e6b77d552b347d679f`

#### Summary

Milestone 1, slice B, completed: absolute pose. A frame is placed against known 3D points, from three correspondences up to a refined, robust pose. The Python reference and C++ core are in lock-step, with identical RANSAC runs on the same seed.

- **P3P** (`foundations.p3p`): Ding et al. (CVPR 2023, the default) and Lambda Twist (ECCV 2018), behind a `solver` option, following PoseLib's implementations with five departures, each measured (see the note). Every pose returned is rigid, with every point in front.
- **EPnP** (`pnp.epnp`, Lepetit et al. IJCV 2009, following OpenCV): from five points. Its four-point linearisations are approximate (measured), and planar points raise `Degenerate`.
- **The pose refinement** (`pnp.refine_pose`): Levenberg-Marquardt on SE(3) of the Cauchy-robust reprojection error, with an analytic Jacobian and cheirality enforced. It reaches SciPy's minimum of the same cost.
- **PnP RANSAC** (`pnp.pnp_ransac`): LO-MSAC with P3P samples and EPnP refits, then **refine-and-reselect rounds**, which are generic in `ransac` and shared with E-RANSAC. Each round refines on the inliers, keeps the result only if the MSAC cost drops, and re-collects the inliers.
- **Measured** (the full tables are in [`docs/notes/absolute-pose.md`](../../docs/notes/absolute-pose.md)):
  - **Near the P3P danger cylinder**, 1e-8 outside it, Ding grossly misses 42 of 2000 true poses, against Lambda Twist's 120 and PoseLib's 477.
  - **Synthetic PnP**, against the noise floor (40% outliers, 1 px):

    | | Error / floor (median, p90) |
    |---|---|
    | slamlab | 1.25, 1.95 |
    | slamlab, planar scene | 1.21, 1.96 |
    | OpenCV `solvePnPRansac` + `solvePnPRefineLM` | 2.21, 5.38 |

  - **TUM RGB-D**, three sequences, with points from depth or from two-view initialisation: the median errors match OpenCV's within the data's noise. The best is fr3/long_office_household: 0.31° and 0.61 cm (OpenCV 0.32° and 0.65 cm).
- **Folded-in deferred items:**
  - one generic `backends.call` dispatch (PR 2 A8);
  - qualified `codes()` keys, `ransac_termination` and `nls_termination` (PR 3 A7);
  - BA's options as one nested dict (PR 2 A7);
  - where refinements live (PR 3 A6): problems stay with their models, and the consensus refinement is generic in `ransac`.
- **Fixed:** `twoview.initialize` raised on a rank-1 homography (PR 2's code, found on TUM fr1/desk). Such an H is now no model, and E decides.
- **Data:** TUM depth maps (`tum.Sequence.depths`), the demo `scripts/demo_absolute_pose.py`, and the parity fixtures `p3p.json` and `pnp.json` (50-digit mpmath).

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Python reference, C++ core and bindings, tests, fixtures, TUM measurements, docs |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | ran: run 1 (full PR) |
| Architecture review | `slamlab-architecture-reviewer` | opus | ran: run 1 (full PR) |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | ran: run 1 (full PR), run 2 (the fixes) |

**Review panel:** full panel (run 1), then the algorithm reviewer on the fixes (run 2).
Report: [`docs/reviews/2026-10-03-feature-absolute-pose.md`](../../docs/reviews/2026-10-03-feature-absolute-pose.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 1 | 1 | 0 | 0 | 0 |
| should-fix | 13 | 13 | 0 | 0 | 0 |
| note | 14 | 6 | 4 | 4 | 0 |

The must-fix:
- **L7 (run 2):** the fixture case meant to guard Ding's second line had stopped discriminating, because rounding its parameters turned the cubic's single real root into three. It is now an exact float64 sample, and a test asserts that it still discriminates.

The should-fixes included three found by measurement:
- **L1:** PoseLib's single-root shortcut dropped true poses near the danger cylinder.
- **L2:** planar PnP RANSAC got no local optimisation and sat at 2× the floor.
- **L3:** the far-field "18% lost" figure was an artefact of an absolute tolerance.

**Owner decisions**

- **Scope:**
  - Ding's solver as the default, with Lambda Twist behind a toggle; EPnP, refinement and PnP RANSAC;
  - the extra TUM sequences fr1/desk and fr3/long_office_household;
  - the four deferred items folded in;
  - the two-view rank-1 H crash fixed in this PR, as an H that is no model so E decides.
- **Run 1:**
  - L2 fixed with the refine-reselect loop, generic for E and PnP.
  - A1: the design doc's pose convention changed rather than the code. Estimators return the transform they name (`T_cam_world`, `T_c2_c1`); trajectory results stay `T_world_cam`.
  - Every other recommendation accepted in bulk.
    - Deferred to the design doc:
      - A6: the callback's form, in PR 5;
      - A7: in-library depth loading, in milestone 2;
      - A8: one feature front end, in milestone 2;
      - L6: SQPnP or IPPE as a planar refit.
    - Waived:
      - H7: two dispatch forms, by design;
      - A4: `ransac` using `nls`;
      - L5: near-tangent candidates, harmless after L1.
- **The loop for E as well:** the owner kept it, accepting a median of 0.78 of the 8-point floor, against 0.71 with one refinement, for a lower tail (p90 2.94 against 3.21) and a higher recall (0.953 against 0.950). TUM two-view results are unchanged within noise (24/38 initialised).
- **Run 2:** algorithm reviewer only.
  - L7 (must-fix) fixed.
  - The rest accepted in bulk:
    - L9: the rounds' stop reason deferred to the design doc;
    - L10: an H validity check inside H-RANSAC, already an open item, so waived.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally: ruff, mypy --strict, import-linter, layering, clang-format, clang-tidy, doxygen, clippy, jscpd; ctest 17/17; pytest 1313 passed (344 skipped, the Rust subset)
- [ ] CI passes
- Every number in the note was re-measured after the fixes and reproduced independently by the algorithm reviewer (run 2). The probes are `p3p_degeneracy_probe.py` and `pnp_sweep.py [--planar]`, plus `scripts/demo_absolute_pose.py --survey` on each TUM sequence.

#### Version

`0.2.2.0` -> `0.2.3.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-8"></a>slam-it pull request 8: Rust core: rung 0's geometry and optimisation, at parity with C++ (PR 5)

Merged 2026-10-04 · `feature/rust-core` → `develop` · 21 commits, +11,836 −347 in 87 files · merge commit `7ccf66031f9eeb0d797c8bf8d20faa90457ab9f8`

#### Summary

Milestone 1, slice C, part 1 of 2: rung 0's geometry and optimisation in the Rust core, at parity with C++. PR 6 adds the estimation core (RANSAC, triangulation, the two-view and absolute-pose solvers), after which Python, C++ and Rust have the same capabilities.

- **The Rust core (`rust/slamlab-foundations`), on nalgebra:**
  - SO(3) and SE(3);
  - pinhole, radtan and radial cameras;
  - Umeyama alignment;
  - PCG32;
  - the generic Gauss-Newton/Levenberg-Marquardt solver, written over `Problem`/`LinearSystem` traits;
  - bundle adjustment: gauge, layout, Schur system over a `ReducedSystemSolver` trait, and the bundle itself.

  It builds without Python: `cargo run --example bal` solves a BAL problem from Rust alone.
- **The bindings (`slamlab_rs`):**
  - one `batched!` macro writes each batched binding;
  - BA's options are read through serde, with unknown fields denied at every depth;
  - core errors are raised as the slamlab exception of their kind.
- **Parity:** all 15 BA fixture cases match the Python reference on the first run, iteration counts and gauge codes included. Every property test of every module Rust implements now runs on all three backends.
- **Measured.** On BAL ladybug-49 every backend reaches 0.9147 px in 33 iterations. Native benchmarks (Google Benchmark for C++, criterion for Rust):

  | | C++ | Rust |
  |---|---|---|
  | Bundle adjustment, ladybug-49 | 1.430 s | 1.441 s |
  | `so3::exp` / `so3::log` | 14.9 / 30.7 ns | 11.7 / 17.5 ns |
  | `se3::log` / radtan projection | 73.9 / 10.6 ns | 89.5 / 20.0 ns |

  Single-threaded COLMAP (Ceres) takes 0.64-0.71 s on the same problem. Why it is about twice as fast has not been profiled; the note lists the candidates.
- **Deferred items closed:**
  - **A9:** PCG32's draws are one fixture that Python, native C++ (through nlohmann_json) and Rust all read.
  - **A27:** every crossing enum's names come from its core's `name()`, in both C++ and Rust.
- **New enforcement:**
  - `check_layers.py` holds Rust's modules inside a rung to the internal-order contracts.
  - Every compiled backend must implement in full each module it has.
  - The native BAL readers are tested field by field against the Python reader.
- **Docs:** [`docs/notes/rust-port.md`](../../docs/notes/rust-port.md), plus the design doc, CLAUDE.md and the README status.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Rust core and bindings, C++ A27 and fixes, benchmarks in both languages, tests, fixtures, docs |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | ran: run 1 (full PR) |
| Architecture review | `slamlab-architecture-reviewer` | opus | ran: run 1 (full PR), run 2 and run 3 (the fixes) |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | ran: run 1 (full PR), run 2 (the fixes) |

**Review panel:** full panel (run 1), then architecture and algorithm on the fixes (run 2), then architecture on the run-2 fixes (run 3).
Report: [`docs/reviews/2026-10-04-feature-rust-core.md`](../../docs/reviews/2026-10-04-feature-rust-core.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 0 | 0 | 0 | 0 | 0 |
| should-fix | 22 | 21 | 0 | 1 | 0 |
| note | 19 | 12 | 2 | 4 | 1 |

The fixes that matter most:
- **L1:** a NaN gradient entry could read as convergence, because Rust's `f64::max` and Eigen's default `maxCoeff` drop NaN. Both cores now propagate NaN.
- **L2:** nalgebra's general SVD has no iteration cap, so a NaN matrix hung forever. A capped helper replaces it, and native checks reject non-finite input in both cores (L5, H2).
- **A1:** the "each Rust module in full" rule was stated in the design doc but enforced by nothing. The tests enforce it now.
- **L3, L4, L11:** the note's comparisons were corrected:
  - COLMAP is now compared single-threaded;
  - "bit-level agreement" became "decision-level agreement";
  - two unbacked explanations were removed.

**Owner decisions**

- **Scope:** two PRs. Rust RANSAC is brought into milestone 1 (in PR 6), so all three languages end milestone 1 with the same capabilities.
- **Technology:** nalgebra for Rust's linear algebra; serde for BA's options.
- **Parity rule:** once PR 6 completes rung 0 in Rust, the tests will require every rung-0 module of Rust. Until then, each module Rust has must be complete.
- **Public presentation** (committed to `develop` separately): a showcase page and run viewer like reifyscene's, starting at milestone 2.
- **Run 1:** accepted all recommendations.
  - H3 was waived: the messages' literal tolerances match C++ and Python word for word.
  - A5 was deferred to PR 6: whether `ba::Summary` implements the summary trait.
- **Run 2:** accepted all.
  - A13 was deferred with A24: the BA facade holds `solve`.
  - A15 was disputed: nalgebra's 3x3 SVD routine returns on NaN.
- **Run 3:** accepted all.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (1704 pytest, 20 skipped; 18 ctest; cargo fmt, clippy pedantic and tests clean)
- [ ] CI passes
- `demo_bal.py --backend all`: Python, C++ and Rust each reach 0.9147 px in 33 iterations on ladybug-49.
- `cargo run --release --example bal` reaches the same result with no Python.

#### Version

`0.2.3.0` -> `0.2.4.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-9"></a>slam-it pull request 9: PR 6: the Rust estimation core, lock-step Rust, milestone 1 complete (0.3.0.0)

Merged 2026-10-04 · `feature/rust-estimation` → `develop` · 22 commits, +15,321 −697 in 77 files · merge commit `2681729d4079adefe38d0adf27be520006d77cc7`

#### Summary

Milestone 1 is complete: Python, C++ and Rust have the same capabilities, and Rust is a lock-step backend.

- **The Rust core gains the estimation half of rung 0:** correspondences, triangulation, LO-MSAC RANSAC, the five-point solvers, epipolar geometry, homographies, two-view initialisation, P3P and PnP, with their `slamlab_rs` bindings.
- **Rust is lock-step.** Every module's tests run on Python, C++ and Rust (1,830 tests, none skipped), and a compiled backend missing a module or a function fails them.
- **Every SVD in the Rust core goes through faer.** nalgebra's were not backward stable: they gave wrong P3P poses on thin triangles, and 8-point E/F up to 6e-3 off on near-planar scenes. A new ill-conditioned fixture guards this on every backend.
- **Cross-language fixes:**
  - five-point solutions come back in one canonical order in all three languages;
  - the RANSAC refinement callback may fail in all three;
  - a C++ segfault (`refine_essential` on a match at both epipoles) is fixed.
- **Closes four deferred panel items:** A5, A6 (PR 4), A13/A24 (BA's `solve` in its own module) and A20.
- **Benchmarks:** E-RANSAC and PnP RANSAC in both native suites. The port note (`docs/notes/rust-port.md`), design doc, README and CHANGELOG are updated.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | The Rust estimation core and bindings, the lock-step flip, the panel fixes |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran (run 1) |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran (runs 1 and 2) |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran (runs 1 and 2); found the nalgebra and faer SVD issues by probing |

**Review panel:** full panel, two runs. Report: [`docs/reviews/2026-10-04-feature-rust-estimation.md`](../../docs/reviews/2026-10-04-feature-rust-estimation.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 3 | 3 | 0 | 0 | 0 |
| should-fix | 16 | 15 | 1 | 0 | 0 |
| note | 18 | 12 | 4 | 2 | 0 |

All three must-fix findings were SVD accuracy:
- **L1, L2:** nalgebra's 3x3 and dynamic SVDs → faer, verified against LAPACK first.
- **L8:** faer does not scale its input → power-of-two scaling outside a safe range, as LAPACK does.

**Owner decisions**

- **SVD library:** faer over `nalgebra-lapack` or a hand-written SVD. It is tested and widely used, keeps the crate cargo-only for the edge rung, and was verified on the reviewer's probes before the switch.
- **Performance:** investigate the cost, and keep the two measured fixes: Stewénius by eigendecomposition, and EPnP's QR beta steps.
- **Canonical five-point order in all three languages now** (not just a reworded note).
- **The refinement callback is fallible everywhere**; a failed round is not kept.
- **Deferred to the design doc:** the stop reason for refinement rounds (A6), C++'s P3P polish threshold (L6), EPnP's agreement on ill-posed scenes (L13), and a stack-only 3x3 SVD before the edge rung (A11).
- **Waived:** A5 (Rust paths differ by design; recorded) and H10 (`LOCKSTEP_BACKENDS` is kept for later rungs).
- **The C++ segfault fix** was approved mid-PR through a selectable prompt.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (1,830 tests, none skipped)
- [ ] CI passes
- The TUM two-view survey (38 pairs) and the absolute-pose survey print identical results on C++ and Rust.
- Same decisions on the bench scene: E-RANSAC 352 inliers in 36 iterations, PnP 350 in 17.

| Native bench | C++ | Rust |
|---|---|---|
| E-RANSAC | 2.29 ms | 3.39 ms |
| PnP RANSAC | 2.40 ms | 2.05 ms |
| BA, ladybug-49 | 1.430 s | 1.27 s |

#### Version

`0.2.4.0` -> `0.3.0.0` (MINOR: milestone 1 complete), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-10"></a>slam-it pull request 10: PR 7: the front end and evaluation — images, ORB, matching, stereo, datasets, metrics (0.3.1.0)

Merged 2026-10-05 · `feature/front-end` → `develop` · 18 commits, +31,534 −194 in 114 files · merge commit `919d572d39a18ace58ccb8ba248ad6a68c4e72ba`

#### Summary

Milestone 2's first slice: the front end and the evaluation, in Python, C++ and Rust in lock-step, bit-identical across the three.

- **Fixed-point images** (`foundations.image`): gray (OpenCV's `cvtColor`, bit for bit), separable integer blur, bilinear resizing, pyramids and remapping, all integer arithmetic. Pyramid positions map to the full image exactly (`full_image_coordinate`), where ORB-SLAM's `x s^l` is up to 2 px off on a coarse level.
- **ORB as ORB-SLAM extracts it** (`foundations.orb`): FAST-9 (OpenCV's scores exactly), 30-px cells with thresholds 20 and 7, the quadtree, intensity-centroid orientation in 30 bins, rBRIEF with OpenCV's learned pattern (BSD, attributed). Features record the pyramid they span.
- **Matching** (`foundations.matching`): Hamming distances and ORB-SLAM's filters (cap, ratio test, one match per descriptor, rotation histogram).
- **Stereo** (`foundations.stereo`): Bouguet's rectification (OpenCV's rotations to 1e-15) and maps; ORB-SLAM's row matching with zero-mean SAD and a V fit (median disparity error under 0.002 px, where ORB-SLAM2's centre-subtracted SAD is off by up to 0.27 px); every keypoint checked against its features' pyramid.
- **Datasets and native readers** (`eval.images`, `eval.tum`, `eval.euroc`; C++ `slamlab_eval` on stb_image, Rust `slamlab-eval` on `png`): PNG frames and depth maps, TUM depth in metres, the EuRoC ASL reader.
- **Metrics and the run format** (`eval.metrics`, `eval.runs`, `eval.timestamps`): ATE (Sim(3)/SE(3)), RPE paired by time (a tracking gap is not one step), TUM's closest-first association, and the run folder every backend's pipeline will write.
- **The front end is rung 0's**, recorded in the design doc (the classic rung reuses it), with exact pyramid coordinates and exact floats (scalar, unfused, `-ffp-contract=off`) as conventions.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | Built the slice in three languages, the fixtures and generators, the demos on ORB, the note; applied every decided finding |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran twice: run 1 on the slice, run 2 on the fixes |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran twice |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran twice, with probes against TUM's tools, evo, ORB-SLAM2 and real EuRoC frames |

**Review panel:** full panel, two runs.
Report: [`docs/reviews/2026-10-05-feature-front-end.md`](../../docs/reviews/2026-10-05-feature-front-end.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 8 | 8 | 0 | 0 | 0 |
| should-fix | 31 | 30 | 1 | 0 | 0 |
| note | 20 | 14 | 6 | 0 | 0 |

(H19, a note of seven small items, is counted fixed: five fixed, two deferred.)

Every must-fix:
- **H1/L1:** `stereo.match` segfaulted (C++) and panicked (Rust) on finite but huge keypoint `xy`. Fixed: every core checks keypoints against their pyramid.
- **H2:** `Options` checked ranges, not types. Fixed: one shared check in `arrays`.
- **H3:** `associate` and `evaluate` did not check that times ascend. Fixed, with TUM's greedy association.
- **H4:** `read_run` leaked raw parser exceptions. Fixed, naming the file and key.
- **A1:** `stereo.match` took the extraction's settings on trust. Fixed: features carry `scale_factor` and `levels`.
- **A2:** the design doc did not place the front end in rung 0. Fixed.
- **H20 (run 2):** a braced check list in C++ indexed the pyramid before the level was checked. Fixed: the checks run in order.

**Owner decisions**

- Building the slice: the front end in `foundations` (rung 0); fixed-point, exact images; stb_image (vendored) and the `png` crate, outside the cores; OpenCV's learned rBRIEF pattern.
- Run 1, through selectable prompts:
  - A1: features carry their pyramid.
  - L2: zero-mean SAD with the V fit.
  - L4: the exact inverse map, not ORB-SLAM's `x s^l`.
  - L5: RPE pairs by time.
  - Every other must-fix: fix.
  - The rest accepted in bulk.
- Run 2, through selectable prompts:
  - H20: fix.
  - A15: rename `eval.timing` to `eval.timestamps`.
  - L12: lower stereo's `search` bound to 1.
  - The rest accepted in bulk.
- Deferred, each in the design doc's open items:
  - To PR 8: guided matching and the full-image-to-level map (A10, A17); the run format's golden folder and native writers (A9).
  - To PR 9's real time: reusing the extraction's pyramid in stereo (A1 follow-up).
  - Backlog: shared Rust fixture helpers (H8); one fastest-backend choice (H18/A13); C++'s `at()` copies and `matching`'s popcount temporary (H19); the state of the art for later rungs (L11).

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally, on every commit (exit code captured)
- [x] Offline CI (`offline_ci.sh`) passed at `7fb97d7`: a clean checkout of committed files, a fresh `slamlab-ci` env built from the lock by `setup_slamlab.sh`, and `scripts/check.sh` as `test.yml` runs it (`7fb97d71c616…	feature/front-end	merged=no	passed`)
- [ ] GitHub CI: not run; both attempts hit GitHub's Actions outage of 2026-10-05 ("The job was not acquired by Runner of type hosted"), not this code
- Parity:
  - Images, FAST scores, keypoints, descriptors, matches and stereo matches are bit-identical in Python, C++ and Rust on every fixture.
  - On TUM frames, and on EuRoC V1_01 frames 0, 400, 1200 and 2000 (467-641 stereo matches each).
- OpenCV: gray and FAST exact; descriptors 0.15 bits apart; rectifying rotations to 1e-15.
- Stereo, on area-sampled pairs: median disparity error under 0.002 px, and under 0.06 px on every level.
- EuRoC rectified frames: 563 (V1_01) and 529 (MH_01) stereo matches.
- Demos on ORB, against SIFT before:
  - Two-view, TUM fr1/xyz frames 0 and 30: 237 matches; rotation 0.88 deg and translation direction 2.27 deg, against 0.63 and 2.32.
  - Absolute-pose survey: 32 frames placed, median 0.77 deg and 0.98 cm, against 0.76 deg and 1.11 cm.
- Extraction time (640x480, 1000 features): C++ 23 ms, Rust 35 ms, Python 160 ms.

#### Version

`0.3.0.0` -> `0.3.1.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-11"></a>slam-it pull request 11: ci: offline CI gates PRs; GitHub runs on demand and for main (0.3.1.1)

Merged 2026-10-05 · `chore/offline-ci` → `develop` · 1 commit, +145 −9 in 9 files · merge commit `b8cfe4a4522d2b93a8cef4b9413eddb6de9db013`

#### Summary

CI moves off GitHub Actions for PRs. The Actions minutes are shared with other projects, and today's Actions outage held up PR 7's merge, although everything had passed locally.

- **`scripts/offline_ci.sh`** does what `.github/workflows/test.yml` does, on the developer's machine:
  - a clean checkout of committed files, in `~/.cache/slamlab-offline-ci/`, cleaned with `git clean -ffdx` each run;
  - a merge with `develop` first, when the branch is not based on it, as GitHub tests a PR;
  - its own `slamlab-ci` env, built from the lock by `setup_slamlab.sh` and recreated when the lock or setup script changes;
  - then `scripts/check.sh`, under an 8G memory cap.

  Each result is appended to `~/.cache/slamlab-offline-ci/results.tsv`.
- **The Test workflow** runs on `workflow_dispatch` and on pushes to `main` only. The tag workflows are unchanged.
- **The PR template** records the offline CI result in place of "CI passes".
- **Docs:** `CLAUDE.md`, the design doc (Review process, Environment), `check.sh`'s header and the review-panel skill say how CI runs.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | The script, the workflow trigger, the template and docs |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | not run: MICRO change (CI tooling) |
| Architecture review | `slamlab-architecture-reviewer` | opus | not run: MICRO change |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | not run: no algorithm code |

**Review panel:** layer 1 only (MICRO change).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 0 | 0 | 0 | 0 | 0 |
| should-fix | 0 | 0 | 0 | 0 | 0 |
| note | 0 | 0 | 0 | 0 | 0 |

**Owner decisions** (selectable prompts, 2026-10-05)

- Run CI offline in most cases; the GitHub workflow on manual trigger and pushes to `main` only.
- Record the offline result as a PR-template line.
- Commit the script to the repo (it began as a personal tool).

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally
- [x] Offline CI: `scripts/offline_ci.sh` passed at `5d83063`, reusing `slamlab-ci` (the lock is unchanged): `5d8306344e18…  chore/offline-ci  merged=no  passed  …/5d8306344e18-20261005-161605.log`
- The same script, as a personal copy, passed PR 7 (`7fb97d7`) from a freshly built env (`--fresh-env`) during the outage. GitHub's own run on the merged `develop` agreed after the outage.

#### Version

`0.3.1.0` -> `0.3.1.1` (MICRO), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-12"></a>slam-it pull request 12: PR 8: the edge VO pipeline — ORB-SLAM-lite in three languages, native programs (0.3.2.0)

Merged 2026-10-07 · `feature/edge-vo` → `develop` · 27 commits, +46,968 −324 in 213 files · merge commit `074f608afb8e8614918a990d5c2c66a5e1d51477`

#### Summary

Milestone 2's second slice: the edge VO pipeline, ORB-SLAM-lite in monocular, stereo and RGB-D, in Python, C++ and Rust in lock-step, and native programs that run a whole sequence without Python.

- **The edge rung** (`slamlab.edge`, `slamlab_edge`, `slamlab-edge`), each stage a function of explicit state:
  - `options`: 76 settings, ORB-SLAM2's defaults and constants, each refused by name;
  - `frames`, `map`: the C++ and Rust maps index each point's observations, as ORB-SLAM's `MapPoint` does;
  - `residuals`, `adjust` (local BA on rung 0's solver), `initialization` (depth; monocular on ORB-SLAM2's rules);
  - `tracking` (motion model, local map, pose refinement weighted by level, PnP fallback), `mapping`, `pipeline`.
- **Rung 0 additions:**
  - guided matching (`features_in_area`, `search_by_projection`; the native cores search a feature grid that finds exactly what Python's brute force finds);
  - `image.level_coordinate`;
  - H-RANSAC's rank rule;
  - `pnp.refine_pose` weights per correspondence.
- **Eval:** native TUM and EuRoC readers; a TUM-format folder's own `camera.txt`, with `bf` for RGB-D; the native run writer, held to a golden folder.
- **The native programs** `slamlab_vo` (C++ and Rust), `slamlab.orchestration.vo.run` (one interface, three backends) and `scripts/run_vo.py`. `backends.program` refuses a Rust program older than its inputs.
- **Parity:**
  - Each stage's decisions match exactly on a synthetic room in every backend; its numbers to 1e-12, or 1e-9 through a bundle adjustment.
  - Whole runs of the room (Python in-process, C++ and Rust as programs) take the same decisions, with poses within 6e-15 m. They are held to a committed golden record.
  - A gap room exercises the failure path, the recovery and the keyframe hold.
- **Results** (`docs/notes/edge-vo.md`): TUM and EuRoC with ORB-SLAM2's settings. Each whole-sequence ATE is the median of five starts, with its range.
  - RGB-D 1.1 to 5.4 cm, stereo 5.3 and 5.4 cm, monocular 1.1 to 11 cm.
  - 2 to 30 ms of tracking and mapping a frame in C++.
  - No RGB-D or stereo run loses the track; 6 of 20 monocular runs do, where ORB-SLAM would relocalise (milestone 4).
  - On each sequence's first 300 frames, all three backends take the same decisions on every frame, with poses within 8e-12.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | Built the slice in three languages, the room and its golden record, the native programs, the note and its results; applied every decided finding |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran twice: run 1 on the slice, run 2 on the fixes |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran twice |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran twice, with probes on TUM and EuRoC (start-frame sweeps, segment splits, reverted fixes) |

**Review panel:** full panel, two runs.
Report: [`docs/reviews/2026-10-06-feature-edge-vo.md`](../../docs/reviews/2026-10-06-feature-edge-vo.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 1 | 1 | 0 | 0 | 0 |
| should-fix | 38 | 37 | 1 | 0 | 0 |
| note | 34 | 21 | 9 | 2 | 0 |

Two notes are not in the columns: L12 (state of the art for later rungs) needed no action, and H31 is covered by H21's test.

The must-fix:
- **A1 (run 1):** the edge rung had no parity fixture; every cross-language check compared against Python live, so a rule changed in all three languages at once would pass. Fixed: every backend's runs of the room are held to a committed golden record (`tests/fixtures/edge/room_runs.json`), regenerated only deliberately (`09eb6a2`).

**Owner decisions**

- The design (2026-10-05, recorded in `docs/design/2026-10-05-edge-vo.md`):
  - stage-level parity, with whole-sequence agreement;
  - native programs in this PR, and one PR;
  - a lost track ends the run;
  - synthetic fixtures;
  - results on TUM and EuRoC without a hard ATE gate;
  - `SlamInput`/`SlamResult` wait for milestone 3;
  - `camera.txt` as the calibration track's export.
- While building: the native maps index each point's observations in this PR. MH_01 stereo went from 0.49 s to 21 ms a frame.
- Run 1, through selectable prompts (2026-10-06):
  - A1: a golden record;
  - L1: weight pose refinement by pyramid level;
  - L2: monocular initialisation on ORB-SLAM2's rules;
  - A2: every threshold in `Options`, and RGB-D's `bf` from the camera;
  - the rest accepted in bulk.
- Run 2, through selectable prompts (2026-10-07):
  - L13: a floor of 100 keypoints with depth now; ORB-SLAM's reset of a map lost early later (milestone 4).
  - A15: the descriptor distances and the stereo matcher's settings into `Options`.
    - **Built differently from the prompt's wording:** the prompt said "nest". The settings are flat `stereo_*` fields, and `Options.stereo_matching()` (C++ `stereo_matching(options)`, Rust `options.stereo_matching()`) assembles the matcher's options from them, as `extraction()` already does for the extractor.
    - This keeps one flat table per language, the one that `run.json`, `--option KEY=VALUE` and the refusal tables are built on. Say if you want a nested struct instead.
  - L15: report the median of five starts.
  - The rest accepted in bulk.
- Applying L15 (2026-10-07):
  - **Lost runs:** a run that loses the track counts as a failure in the median, as ORB-SLAM's tables mark one.
  - **The losses themselves:** documented, with relocalisation deferred to milestone 4.
  - **Start frames:** the extra starts begin one to four frames after the frame the first run initialises from. The old rule left two monocular sequences with five identical runs.
- Deferred, each in the ladder's open items:
  - **Milestone 4:** ORB-SLAM's early reset (L13), relocalising monocular runs that lose the track (L15), and lifting the edge camera into rung 0 (A8).
  - **PR 9:** the per-call feature grid and the allocating `svd3` (A11).
  - **Backlog:**
    - shorter native stage functions (H6);
    - loud preconditions in `refresh` (H14);
    - the map's count invariants (H15);
    - keyframe-time growth on long sequences (H19);
    - native option checks from the tables (A13);
    - solve summaries in the diagnostics (L9);
    - the two-view fixture's duplicate matches (L11);
    - the Rust extension in layer 1 (A19, predates this PR).
- Waived:
  - H17: Python `assert`s for narrowing, the repo's style.
  - A10: `FileNotFoundError` in Python against `InvalidInputError` natively for a missing file, the documented decision for `eval`'s readers.
- No run 3: each run-2 fix answers one finding, with a test where it changes behaviour, and none changes a design. A run 3 is yours to ask for.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally on every commit, exit code captured; at `8065548`, 2985 pytest plus every ctest and cargo test
- [x] Offline CI: `scripts/offline_ci.sh` passed at `6649e05`, the branch head, in a `slamlab-ci` env rebuilt from scratch (this PR changes `setup_slamlab.sh`): `6649e057f27f…  feature/edge-vo  merged=no  passed  …/6649e057f27f-20261007-121222.log`; 2985 pytest plus every ctest and cargo test
- Parity:
  - **The room:** stage decisions exact, numbers to 1e-12 (1e-9 through BA); whole runs to 6e-15 m in every mode and backend. The gap room fails once and recovers twice in every case, and agrees again with every new setting moved off its default.
  - **TUM and EuRoC:**
    - first 300 frames: Python, C++ and Rust take the same decisions, with poses within 8e-12;
    - whole sequences: C++ and Rust take the same decisions, with poses within 8e-12.
- Whole-sequence ATE, medians of five starts, every frame (keyframes):
  - **TUM RGB-D:** fr1/xyz 1.09 (1.38) cm; fr1/desk 3.13 (2.79) cm, against ORB-SLAM2's 1.6; fr3/long_office 5.36 (3.72) cm, against 1.0.
  - **TUM monocular:**
    - fr1/xyz 1.10 (1.30) cm, against ORB-SLAM's 0.90;
    - fr1/desk lost in 3 of 5; the two that finish have 2.34 and 2.47 cm over keyframes, against 1.69;
    - fr3/long_office 9.85 (7.90) cm, against 3.45.
  - **EuRoC stereo:** MH_01 5.44 cm and V1_01 5.25 cm, against ORB-SLAM3's 2.9 and 3.5.
  - **EuRoC monocular:** MH_01 11.20 cm (lost in 2 of 5) and V1_01 6.67 cm (lost in 1 of 5), against ORB-SLAM3's 1.6 and 3.3.
- **What the spread shows:** the review measured the code before its fixes over ten starts. The median is the same on MH_01 stereo (5.17 against 5.14 cm), so the fixes' effect on these sequences is within one run's start-frame spread.
- **Privacy:** nothing from the private rig recording is in the repo. The rig calibration design names `bf` as coming from the external rig config, and the template holds a placeholder.

#### Version

`0.3.1.1` -> `0.3.2.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-13"></a>slam-it pull request 13: PR 9: budgets and real time — the laptop-cpu harness, C++ and Rust VO in real time (0.3.3.0)

Merged 2026-10-08 · `feature/real-time` → `develop` · 25 commits, +8,819 −1,245 in 98 files · merge commit `caf5e211605f65aa96820a9afdbda92012e76126`

#### Summary

Milestone 2's third slice, PR 9: the budget harness, and the C++ and Rust pipelines brought to real time within a laptop-CPU budget.

- **The budget harness** (`slamlab.eval.budgets`):
  - `laptop-cpu`, 4 performance cores and 8 GiB;
  - a program pinned with `taskset` and memory-capped by a `systemd-run` scope with no swap, so going over the cap kills it;
  - `judge`, each frame's building plus tracking against its deadline (the frame period), reading reported apart, and mapping judged by its load over the run and over its busiest 10 seconds;
  - `scripts/run_vo.py --budget`, and the opt-in budget tests (`python -m pytest -m budget`).
- **The run format, version 2:**
  - each frame times its steps (`runs.STEPS`: reading, building, tracking, mapping);
  - each program records the process that ran it: peak memory (`VmHWM`), the CPUs it could run on, and its memory cap;
  - Python has its own program, `python -m slamlab.orchestration.vo`, so a budget runs all three backends the same way.
- **Frame building, exact (every bit unchanged):**
  - FAST's segment test over a row at once, branch-free;
  - the blur's and the resize's sums laid out to vectorise;
  - one pyramid per image, as ORB-SLAM's `ComputeStereoMatches` reads it (PR 7's A1).
- **Local BA, 2.5x as fast in C++ and 3.8x in Rust:** the normal equations over the coordinates the reduced system reads only, one triangle mirrored, a coupling layout fixed per problem, fixed-size blocks. The results are no longer the same to the bit; the owner chose to relax that, verified three ways:
  - **per solve**, on 3,136 saved local BAs: the same iterations and termination, and no chi-square flips;
  - **parity:** the fixtures pass unchanged;
  - **whole runs:** all 100 (ten sequence-modes, five starts, C++ and Rust) take PR 8's decisions on every frame.
- **The two A11 items:**
  - Rust's 3x3 SVD is a port of Eigen's `JacobiSVD`: on the stack, and C++'s factors to the bit, held by a golden that Eigen writes;
  - each C++ and Rust frame builds its feature grid once, through one constructor (`edge::make_frame`, `Frame::new`).
- **Results** (`docs/notes/real-time.md`), on the edge note's five TUM and EuRoC sequences, in both modes, within `laptop-cpu`, with no frame late in either language. Building and tracking a frame takes a median of:
  - 9-13 ms on TUM and EuRoC monocular;
  - 21-26 ms on EuRoC stereo,

  against deadlines of 32 and 50 ms. Before this PR it took 23-72 ms in C++ and 34-113 ms in Rust. Mapping's busiest 10 seconds stays below a thread's budget everywhere: 0.92 at most, from 2.28. Peak memory is 21-133 MiB.
- **Layer 1:**
  - the native tests run hardened (`-D_GLIBCXX_ASSERTIONS`);
  - the C++ benchmarks build apart, unhardened, in `build/bench`.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | Built the slice in three languages; profiled, sped up and verified the pipelines; ran the per-solve, parity and whole-run checks; wrote the note; applied every decided finding |
| Hygiene review | `slamlab-hygiene-reviewer` | sonnet | Ran twice: run 1 on the slice, run 2 on the fixes |
| Architecture review | `slamlab-architecture-reviewer` | opus | Ran twice, with an unhardened benchmark build to measure A12 |
| Algorithm review | `slamlab-algorithm-reviewer` | opus | Ran twice, with a hardened rebuild (L1), a mirror-removal mutation (H10), and Eigen cross-checks of the SVD port |

**Review panel:** full panel, two runs.
Report: [`docs/reviews/2026-10-08-feature-real-time.md`](../../docs/reviews/2026-10-08-feature-real-time.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 4 | 4 | 0 | 0 | 0 |
| should-fix | 32 | 32 | 0 | 0 | 0 |
| note | 16 | 12 | 3 | 0 | 0 |

Rows are the report's, one per finding however many reviewers raised it. One note is not in the columns: L11, context with nothing to do. H31's docs were fixed and its test was waived; it needs a test-only parameter, and no input reaches the cap.

The must-fix findings:
- **H1/L2 (run 1):** `budgets.judge` accepted times that do not advance and step times that are NaN or negative, and returned a silent false verdict. It now refuses them by name (`1915567`).
- **A1 (run 1):** a `Budget` of 0 cores pinned the program to every CPU. A budget now checks itself when it is made (`1915567`).
- **L1 (run 1):** the C++ blur formed a span past the row's end when the kernel was wider than the image. It is guarded, and layer 1's native tests now run hardened, which aborts on the old code (`c286113`).
- **L13/A13/H18 (run 2):** a run shorter than 10 s had its busiest-window load diluted below its whole-run load. The window is now the shorter of the two (`8fa9567`).

**Owner decisions**

- **The three rules (2026-10-07):**
  - one thread;
  - the deadline holds building plus tracking, and mapping is judged by its load;
  - single-threaded fixes first, which were enough;
  - the clock starts with the decoded images.
- **Placement:** the harness lives in `slamlab.eval`; `run_vo.py --budget` replaces a separate script; each program records its own peak memory.
- **"Speed up local BA now"**, then **"relax bit identity, verify it the way you described"**: per solve, parity, and whole runs statistically.
- **"Implement both"** A11 items.
- **Run 1:** every must-fix and should-fix fixed. A8 was recorded now rather than deferred to PR 10. The notes went as recommended: L8 and L12 deferred to milestone 7, L10 to the ladder's backlog.
- **Run 2:** everything fixed; A12 as a second, unhardened benchmark tree in layer 1. Only H31's test was waived.
- **Commit `2db3289`'s message** claims a fix of a 0-bit blur shift by -1 that the base never had. The report and the CHANGELOG correct it (L3); the history is not rewritten.

**Merge order:** this is the only open PR, so there is no order to follow.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (exit 0, all 20 stages, pytest 3180 passed); the budget tests pass (`python -m pytest -m budget`, 13).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `347b829` (its line in `~/.cache/slamlab-offline-ci/results.tsv`: `347b8298be594734bf22dbc5733fcf61c1079fe4  feature/real-time  merged=no  passed  $HOME/.cache/slamlab-offline-ci/logs/347b8298be59-20261008-113743.log`).
- **Local BA per solve:** 3,136 problems saved from whole runs, solved by the old and new code, in C++ and Rust:
  - the same iterations and termination everywhere, with no chi-square flips;
  - poses within 1.7e-14, points within 1.5e-9 relative.
- **Whole runs:**
  - all 100 runs (ten sequence-modes, five starts, C++ and Rust) make PR 8's decisions frame by frame, so the five-start medians are PR 8's;
  - the first 300 frames of each sequence-mode take the same decisions in Python, C++ and Rust;
  - after the panel's BA (H10) and frame-constructor (A7) changes, MH_01 (stereo and monocular) and fr3/long_office monocular are byte-identical to the runs before them, in both languages.
- **Rust's 3x3 SVD:** bit-identical to Eigen's `JacobiSVD` on 39,005 matrices; held by the 21-case golden in C++ and Rust, signed zeros included.

#### Version

`0.3.2.0` -> `0.3.3.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-14"></a>slam-it pull request 14: Rename slamlab to SLAM It All! (slam_it_all, slam-it-all; 0.3.4.0)

Merged 2026-10-08 · `chore/rename-slam-it-all` → `develop` · 3 commits, +3,594 −3,333 in 447 files · merge commit `f34e3f5c59d71f331430f9fcbc855501f14be205`

#### Summary

The project is renamed **SLAM It All!** (a pun on "damn it all!") before it goes public. The working name, slamlab, collided with SLAMLab, the University of Zaragoza group behind ORB-SLAM3, and with `slamlab` on PyPI. Both new names are free on PyPI and crates.io.

The rename is a clean break with no alias, and nothing behaves differently.

- **`slam_it_all` in code:**
  - the Python package;
  - the C++ namespace and include directory;
  - the libraries and programs (`slam_it_all_foundations`, `slam_it_all_vo`);
  - the extension modules `slam_it_all._cpp` and `slam_it_all_rs`.
- **`slam-it-all` for everything else:**
  - the distribution;
  - the Rust crates;
  - the conda envs (`slam-it-all`, `slam-it-all-ci`) and their lock;
  - the review panel (`/slam-it-all-review-panel`, `slam-it-all-*-reviewer`);
  - the repository, once it is renamed after this merges.
- **Other forms:**
  - `SLAM_IT_ALL_` prefixes the CMake options and environment variables;
  - `SlamItAllError` is the base exception;
  - `slam-it-all-run` names the run format, so runs written before the rename no longer load.
- **"SLAM It All!" is the display name in prose.** CLAUDE.md's conventions state which form goes where.
- **History keeps the old name:** the CHANGELOG's earlier entries and the review reports in `docs/reviews/`. Their guide, `docs/reviews/README.md`, is updated.
- **Edits beyond the rename** (the panel's H11):
  - `scripts/compare_ba_colmap.py`'s column `slamlab` is now `ours`, like the table's other columns, and the BA note's table header matches;
  - `scripts/demo_bal.py` right-aligns COLMAP's label to the longer name;
  - two Cargo descriptions, one test docstring and one function docstring each lose a word to fit 100 columns;
  - the README's benchmark build is two commands;
  - CLAUDE.md and `pyproject.toml` lose their "working name" remarks;
  - the ladder's "Rename before going public" item now records the rename;
  - from the review:
    - `check.sh` names the panel's real command;
    - the ladder's relative link to reifyscene works;
    - the ladder title and package docstring read "SLAM It All! -- a learning ladder…".

**Merge order:** this is the only open PR, so there is no order to follow.

**After it merges** (each step waits for your OK; the plan agreed before the PR):
1. Rename the GitHub repo to `evan-wes/slam-it-all`. GitHub redirects the old links.
2. Update the merge exception: your global CLAUDE.md and `ALLOWED_REPO` in the owner's merge-guard hook. I'll show you the exact edit to approve. Until then, merges in the renamed repo are blocked.
3. Rename the local pieces:
   - the folders, to the SLAM It! checkout;
   - the old `slamlab` and `slamlab-ci` envs, to remove once the new ones are in use;
   - reifyscene's `.claude/` symlinks;
   - a private scripts folder;
   - my memory notes;
   - the old offline-CI cache.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | Previewed the rename on a separate clone, then applied it with ordered rules (identifiers by kind, then prose, then hand-worded lines); formatted and rewrapped; built the new env; recorded runs before and after; applied the decided findings |
| Hygiene review | `slam-it-all-hygiene-reviewer` | sonnet | Ran: reverse-mapped every changed file back to the old names and read every edit beyond the rename |
| Architecture review | `slam-it-all-architecture-reviewer` | opus | Not run: the owner chose the hygiene reviewer alone, since nothing changes behaviour and layer 1 holds the layer contracts |
| Algorithm review | `slam-it-all-algorithm-reviewer` | opus | Not run: no algorithm changed |

**Review panel:** hygiene only, the owner's choice. The agent types were not registered, because reifyscene's symlinks still name the old files, so it ran as a general-purpose agent following the renamed agent file.
Report: [`docs/reviews/2026-10-08-chore-rename-slam-it-all.md`](../../docs/reviews/2026-10-08-chore-rename-slam-it-all.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 1 | 1 | 0 | 0 | 0 |
| should-fix | 6 | 6 | 0 | 0 | 0 |
| note | 7 | 6 | 1 | 0 | 0 |

**The must-fix finding, H1:** my rewrap of two `se3.py` docstrings merged the `backend` parameter into `xi`'s description, so the generated docs lost it. Layer 1 does not check docstring structure. Both are restored (`4f064f4`). I checked every other place the rewrap touched, and they are intact.

**Owner decisions**

- **The name:** "SLAM It All!", with `slam_it_all` in code and `slam-it-all` for the repo and URL, chosen for readability over a single spelling.
- **The order:** the rename comes next, as its own PR, before PR 10, so the viewer and the demo start with the new name.
- **The review:** the hygiene reviewer alone.
- **History:** kept as written.
- **The findings:** every one fixed as recommended. H12 is deferred to the post-merge steps above. H7 keeps the "SLAM It All!'s" possessives.

#### Verification

- [x] Layer 1: `scripts/check.sh --env slam-it-all` passes locally, run on the rename and again after the fixes (exit 0, all 20 stages, pytest 3180 passed).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `b7eb33c` (its line in `~/.cache/slam-it-all-offline-ci/results.tsv`: `b7eb33c7057a01b8555f8ed7a765b28913bdd3f3  chore/rename-slam-it-all  merged=no  passed  $HOME/.cache/slam-it-all-offline-ci/logs/b7eb33c7057a-20261008-214743.log`).
- **The env:** `scripts/setup_slam_it_all.sh` built `slam-it-all` from the lock. `slam_it_all` 0.3.4.0 imports with all three backends, and both `slam_it_all_vo` programs are found.
- **Unchanged behaviour:**
  - The comparison covered 12 runs: C++ and Rust whole runs on five sequence-modes (MH_01 stereo and monocular, V1_01 stereo, fr1/desk RGB-D, fr3/long_office monocular), and Python's first 300 frames of fr1/xyz RGB-D and MH_01 stereo.
  - Runs on `develop` and on this branch match byte for byte: `trajectory.txt`, the keyframes, the map points, every frame's diagnostics apart from the step timings, and `run.json` apart from the format name and process readings.
- **No old name left:** `git grep -i slamlab` finds only the ladder's rename item, the CHANGELOG and `docs/reviews/` (reports only).

#### Version

`0.3.3.0` -> `0.3.4.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-15"></a>slam-it pull request 15: The run viewer: solver summaries, lifetimes, feature records, the bake and the page (0.3.5.0)

Merged 2026-10-10 · `feature/run-viewer` → `develop` · 17 commits, +13,046 −1,077 in 165 files · merge commit `a8f3b6a51b40cfa4c9a83aad112abdcb558e77bb`

#### Summary

PR 10 of milestone 2: the run viewer, planned in
[`docs/design/2026-10-09-run-viewer.md`](../../docs/design/2026-10-09-run-viewer.md)
(sections 1 to 4); what was built and how it was checked is
[`docs/notes/viewer.md`](../../docs/notes/viewer.md).

- **Repeatable C++ runs.** Eigen blocked a large product by the cache sizes of the core that
  ran the first one, which differ between a hybrid CPU's core types, so a C++ run's last bits
  depended on where it started. Every rung now compiles with `EIGEN_NO_CPUID`, so every
  program, test and the extension block by Eigen's own defaults on every core.
- **What a run records**, in Python, C++ and Rust:
  - each frame's solver summaries (initialisation, motion model, PnP, local map, mapping and
    every BA solve);
  - every point's and keyframe's life (run format v3: made, removed, why);
  - opt-in per-keypoint feature records with their fates (unmatched, rejected, tracked,
    made), streamed to disk.

  The three languages agree exactly on all of it.
- **The bake** (`python -m slam_it_all.orchestration.bundles`): one sequence's runs, one per
  backend, with its images, into a bundle any static server can serve. Frames are renumbered
  across backends by time; each run is aligned to the ground truth by its own ATE alignment,
  or left in its own frame if it cannot be placed.
- **The page** (`viewer/`): the frame with its keypoints' fates, the map after the frame with
  every backend's trajectory and the truth, the solvers' summaries, plots and step times, the
  frames where the backends decided differently marked. Plain JavaScript modules, three.js
  from a CDN, no build step; `scripts/view_bundle.py` serves it locally.
- **Layer 1** gains the page's checks: `node --check`, `node --test`, `tsc --checkJs` (twice:
  the page, and its logic modules without the DOM) and prettier, its tools pinned in
  `viewer/package-lock.json`.
- **The edge note's ten sequence-modes** were baked into bundles, kept outside the repo
  (2.0 GB). With features recorded, every run stays real time.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Built the 7 planned commits and the fixes of both review runs; ran the comparison, budget and real-time runs, the bundles and the browser checks |
| Hygiene review | `slam-it-all-hygiene-reviewer` | sonnet | Ran: run 1 on the slice, run 2 on the fixes |
| Architecture review | `slam-it-all-architecture-reviewer` | opus | Ran: run 1 on the slice, run 2 on the fixes |
| Algorithm review | `slam-it-all-algorithm-reviewer` | opus | Ran: run 1 on the slice, run 2 on the fixes |

**Review panel:** full panel, twice: run 1 on the finished slice (`de57cf4...b93d66d`), run 2
on the fixes (`b93d66d...c78601a`), since they changed each area substantially.
Report: [`docs/reviews/2026-10-10-feature-run-viewer.md`](../../docs/reviews/2026-10-10-feature-run-viewer.md).

Counts after merging the findings two reviewers shared (run 1 + run 2):

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 1 + 0 | 1 | 0 | 0 | 0 |
| should-fix | 11 + 4 | 15 | 0 | 0 | 0 |
| note | 11 + 4 | 10 | 4 | 1 | 0 |

The must-fix (L1 / H3): a run the ATE alignment could not place (a monocular run that never
initialised) failed every backend's bake, and left a half-written folder that blocked a
retry. Fixed in 038aaf3: such a run is baked unscored in its own frame, the page says so, and
the bundle is written beside its folder and moved onto it whole.

**Owner decisions**

- While building (design doc, section 8): stages return their summaries with their results;
  the C++ repeatability fix inside PR 10; feature records as two plain arrays; a keypoint's
  point is the one its fate concerns; the bake in orchestration, a bundle's arrays as `.npy`
  files; a pinned `viewer/package.json` for the type check, with three.js's own types.
- Run 1: every finding fixed as recommended, "unmatched" by its words (option a, not widening
  "rejected"); deferred to the ladder's backlog: removals out of the core (A6, the lifetimes'
  up to 26 MiB of peak memory), the pipeline in the formats (A9), the BA solve record in rung 0
  (A10), motion-model outliers searched again (L7, pre-existing, moves decisions); waived: H8.
- Run 2: every finding fixed as recommended.
- After the panel: four commit messages held six wrong or misleading statements (a disproved
  claim, figures without their settings, an estimate given as a measurement, a disk-usage size,
  a miscount). They were reworded with the files unchanged, so every commit has a new SHA; the
  report lists each correction ("Commit messages corrected").

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (exit 0; pytest 3265 passed, ctest 36,
  cargo test, node --test 32, both tsc checks, prettier, jscpd).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `fcce3b1`, in the env recreated from the lock
  for this PR's first run:
  `fcce3b15f3f1be3b3030e36b2dd0c3660ce5e972  feature/run-viewer  merged=no  passed  $HOME/.cache/slam-it-all-offline-ci/logs/fcce3b15f3f1-20261010-213135.log`
- **No decision moves.** The rename's twelve comparison runs (C++ and Rust whole on five
  sequence-modes, Python's first 300 frames of two), after each commit against the one before:
  trajectories, every existing diagnostics field and, from v3, the final keyframes and map
  byte for byte. The C++ fix moved C++'s last bits once and no decision; `EIGEN_NO_CPUID` then
  left all five C++ runs byte-identical, on performance and efficiency cores.
- **Real time.** The real-time note's 20 whole runs within `laptop-cpu`, features off and on:
  no frame late in any of the 40; recording features moves latency by noise (median -0.5 to
  +0.1 ms); medians within 0.5 ms of PR 9's; PR 9's decisions and final keyframes on every
  run. Budget tests (`pytest -m budget`) 13 passed.
- **The page**, in headless Chromium: all ten edge-note bundles load without an error and
  scrub to their last frame; a run left unaligned shows as such, the truth hidden; a bundle
  baked before the table check is refused with a clear message; phone width checked.

#### Version

`0.3.4.0` -> `0.3.5.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-16"></a>slam-it pull request 16: Your own video: the import, the intrinsics, the worker and the upload server (0.3.6.0)

Merged 2026-10-12 · `feature/your-own-video` → `develop` · 44 commits, +13,720 −115 in 89 files · merge commit `54ea34cb39b54d886d5a61afaf1db621f10d07a8`

#### Summary

PR 11 of milestone 2, which closes it: your own video, planned in
[`docs/design/2026-10-09-run-viewer.md`](../../docs/design/2026-10-09-run-viewer.md)
(section 5); what was built and how it was checked is
[`docs/notes/your-own-video.md`](../../docs/notes/your-own-video.md).
A phone video, or a Stray Scanner recording, is uploaded through a page, imported with the best
intrinsics to be had, run through the C++ pipeline by a worker, and opened in the run viewer.

- **The import** (`slam_it_all.eval.video`, PyAV in the optional `video` extra, kept in that
  module by a contract): a video opened with FFmpeg's mov/mp4 demuxer alone, turned upright by
  its declared rotation, timed by presentation time, scaled to a 480-pixel short side,
  undistorted to pinhole if its lens is a fisheye, and written as a TUM-format folder;
  frames over 7680 pixels, 3 to 1 or of more than one size, and videos over 14,400 frames, are
  refused. Stray Scanner zips read in place, their poses used as its encoders write them.
- **The intrinsics** (`slam_it_all.eval.lenses`): a Gyroflow lens profile (CC0, fetched at a
  pinned commit, 1,047 of its 1,164 phone profiles usable), matched by phone, lens and the
  profile's own focal length against the video's; else the 35mm-equivalent focal length; else a
  nominal field of view. Fisheye undistortion through OpenCV's model, held to `cv2.fisheye`.
- **The worker** (`slam_it_all.orchestration.worker`): claims jobs over HTTP(S) with its token;
  in a child process under memory and file-size limits and a timeout, imports, runs and bakes
  each, with a heartbeat; a job's processes are one group, killed whole however it ends or the
  worker is stopped; only safe links followed; a refused token ends it.
- **The upload server** (`server/`, `slam_it_all_server`, which never imports `slam_it_all`): the
  job list in SQLite with its format kept; S3 or folder storage through signed links, a job's
  files in one folder; the guards (invite codes, caps, one job at a time, uploads deleted after
  import, results after 7 days, a Content Security Policy, never run as root). The local demo
  keeps each upload with its result, so that a recording can be run again, deleted with it once 7
  days old while the demo runs; the server refuses that setting with S3. Its settings come from the environment, from
  `server/settings.template.env`.
- **The pages** (`viewer/upload.html`, `status.html`), the images (`docker/`) and
  `scripts/serve_demo.py`, which runs it all on this machine with a three-word invite code and,
  on the local network, prints the page's address as a QR code for a phone's camera.
- **A result shorter than the clip** is said as such: a run ends where it loses the track, so the
  status page says what of the clip was tracked and where the run stopped, and the viewer's
  timeline shows the whole sequence, each backend's frames by state and the part no run reached
  ("not processed: tracking was lost"); `bundle.json` records the sequence's `extent` for it,
  which the page requires (format version 1 kept).

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | Built the planned commits, the fixes of three review runs and a commit-message audit; ran the synthetic recordings, the images' end-to-end jobs and the probes |
| Hygiene review | `slam-it-all-hygiene-reviewer` | sonnet | Ran: run 1 on the slice, runs 2 and 3 on the fixes |
| Architecture review | `slam-it-all-architecture-reviewer` | opus | Ran: run 1 on the slice, runs 2 and 3 on the fixes |
| Algorithm review | `slam-it-all-algorithm-reviewer` | opus | Ran: run 1 on the slice, runs 2 and 3 on the fixes |
| Security pass | general-purpose agent, its own brief | opus | Ran beside the panel each time (the design's decision 5): the server, the worker, hostile uploads, the pages, the images, the demo; run 4 alone, on the demo's kept uploads |
| Commit-message audit | four general-purpose agents | opus | After run 3, every message against its commit's diff, code, logs and measurements |

**Review panel:** full panel and the security pass, three times: run 1 on the finished slice
(`63b61da...e909a76`), run 2 on its fixes (`e909a76...d056110`), run 3 on run 2's fixes
(`d056110...0966cd8`), each run's fixes having changed every area again.
Reports: [`docs/reviews/2026-10-11-feature-your-own-video.md`](../../docs/reviews/2026-10-11-feature-your-own-video.md)
and [its security pass](../../docs/reviews/2026-10-11-feature-your-own-video-security.md).

The panel's counts (runs 1 + 2 + 3), after merging the findings two reviewers shared:

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 5 + 2 + 0 | 7 | 0 | 0 | 0 |
| should-fix | 13 + 9 + 3 | 25 (one, H5, in part) | 0 | 0 | 0 |
| note | 11 + 7 + 6 | 14 | 6 | 0 | 0 |

Four notes needed no action (L9, H17, H18, L16). The security pass (runs 1 + 2 + 3): must-fix
1 + 0 + 0, should-fix 10 + 3 + 1, notes 9 + 3 + 3; all fixed but one note deferred (S20), and
three fixed with a part deferred to before going public (S2, S11, S15). Where a security
finding overlaps a panel one, it is counted in both tables. Run 4, a focused security check of
the demo's kept uploads (`f51e1f6...88c4151`), without the panel: must-fix 1, should-fix 2,
notes 5; six fixed (`99cf1d6`), two notes waived (S34, S36).

The must-fixes:
- **Run 1:** a job's processes outlived its timeout or a 409, and a failed report stopped the
  worker (H1 / A1 / S1, `8bf170f`); a sweep deleted job rows before their files (A2 / S7,
  `98ae08a`); errors were swallowed unlogged (H2 / A7, `8bf170f` and `98ae08a`); Stray
  Scanner's rotations were turned twice (L1, `88190df`); the fisheye model's failure paths were
  untested (L2, `c08684d`).
- **Run 2:** a SIGTERM to the worker, or the demo's Ctrl-C, left its job running (A13 / S22),
  fixed in `92699e7`; profile matching never compared a profile's own focal length with the
  clip's (L10), fixed in `13aa4dd` and, for clips without a tag, `e1ed117`.

**Owner decisions**

- While building: a signed PUT for the declared size; a 30 s heartbeat and a 2-minute silence
  limit; the pages in `viewer/`; settings from the environment with a template; a contract
  keeping the server from `slam_it_all`; the import's folder named "your-video".
- Run 1: every must-fix fixed; the should-fixes as recommended, S2's token out of the path and
  S11's self-hosted fonts and HSTS deferred to before going public; S6 as the waiting job
  expiring with its 15-minute link, at most 2 open jobs per code, the code kept only as a
  keyed hash; notes as recommended, A8, A9, A10, L6, L7, L8 and S20 deferred (the ladder's open
  items), L9 needing no action.
- Run 2: every finding as recommended (A13 and L10 fixed; H17 and H18 no action); a run 3 and a
  rebuild of the images approved.
- Run 3: every finding as recommended; for L15, the band around the nominal main camera alone,
  reversing the median rule kept in run 2; L16 no action; no run 4.
- The commit-message audit: 21 messages corrected by rewording, the files unchanged, so every
  commit from `d5ba41c` on has a new SHA (the reports say which, "Commit messages corrected");
  three statements the code still contradicted made true: a refused token ends the worker
  (`9a8da4b`), a job's folder is made inside its own failure handling (`9a8da4b`), the remap's
  fastest backend first (`d14ffe5`).
- Offline CI, its env built afresh because this PR changes the lock, got Rust 1.99.0 unpinned
  (the working env had 1.98.1), whose clippy adds a pedantic lint that three existing asserts
  failed: the asserts were met and Rust pinned to 1.99.0 in the setup script, as clang is, the
  working env updated to it (`cc4491f`).
- After the PR opened: the demo prints the page's address as a QR code with `--lan`, black on
  white whatever the terminal's colours, the invite code kept out of it (`ca8d432`; `segno` in
  the dev extra and the lock).
- After the first phone clip, whose run lost the track at 5 s of a longer video and looked
  cropped in the viewer: the status page and the viewer say what was tracked and what was not
  (`2bbc0e8`, `a4b843f`, `c9b69d2`), relocalisation left out; the bundle's format kept
  at version 1 with the new field required, PR 10's ten edge-note bundles given it outside the
  repo; and the demo's invite code three short words (`50110f6`).
- So that recordings can be run again: the demo keeps each upload with its result for their 7
  days, a job's files in one folder deleted together, the single bucket lifecycle rule this
  allows accepted (`4799172`); a focused security check of it, not the full panel (run 4), its
  must-fix the page promising a deletion only a running demo makes, fixed with a demo folder the
  same every run and words that say so (`99cf1d6`), every finding decided as recommended.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally at `325c7f9`, on Rust 1.99.0 (exit 0; pytest 3572
  passed, ctest 36, cargo test, clippy, node --test 49, both tsc checks, prettier, jscpd, 21
  import contracts kept).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `325c7f9`, in the env recreated from the lock and
  the pinned Rust (the first run of it, at `534bbdb`, failed clippy alone: the lint `cc4491f`
  meets):
  `325c7f94763d2b72e33fd6def0937d6c0e7d460b  feature/your-own-video  merged=no  passed  $HOME/.cache/slam-it-all-offline-ci/logs/325c7f94763d-20261012-122825.log`
- **The images**, rebuilt from `325c7f9` (the server's 286 MB, the worker's 498 MB, as `docker
  images` reports them): one job through both containers from the room's Stray Scanner zip, as
  Stray writes its default (HEVC, 60 fps, 80 frames: 40 kept, 34 tracked, ground truth in the
  bundle, the summary not lost and run to the clip's end), and one from the room's video
  (nominal intrinsics); after each, the result in the job's folder and no upload or copy of one
  left, as a deployment keeps none; run as root, the server's entrypoint drops to uid 10001
  with `NoNewPrivs: 1`.
- **The demo's kept uploads**, on a demo run on a scratch folder: the printed line, `/api/kept`,
  the folder at 0700 and, in a headless browser, the upload page's sentence once the server
  answered; each rule broken in turn, each caught by a test (the commits list them).
- **The reviewers' probes**, rerun on the fixes: no job process left after a timeout, a 409 or
  a SIGTERM, and none in 8 trials of a demo-style Ctrl-C; no public, escaped or scoped public
  host accepted by `check_url`; on the Gyroflow database, no main-camera clip matched to
  another lens's profile.
- **The timeline and the status sentence**, in a headless browser: the viewer at 1280 and 390 px
  wide on the golden bundle and on a copy with its runs set lost at the last frame; the status
  page with a done job's status served from a file. PR 10's ten edge-note bundles, given the
  extent outside the repo, each load with the viewer's own loader; fr1_desk RGB-D shows its
  first 0.6 s not run (no depth map), none was lost.
- **Real phone clips:** two from the iPhone 14 Pro so far, through the demo, kept outside the
  repo: one recorded in the browser, whose lost track looked like a cropped video; one Stray
  Scanner recording, the first real zip imported, which tracked about half its length before
  losing the track while panning. The Pixel 8, the Galaxy A15 and the rest of the note's "What
  waits for the real clips" are still to come.

#### Version

`0.3.5.0` -> `0.3.6.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-17"></a>slam-it pull request 17: Rename SLAM It All! to SLAM It! (0.3.7.0)

Merged 2026-10-12 · `chore/rename-slam-it` → `develop` · 3 commits, +4,197 −3,912 in 499 files · merge commit `f5877c75699a7f306d8b1fb22222364eb1ea66b0`

#### Summary

The project is renamed from **SLAM It All!** to **SLAM It!**: shorter, easier to remember, and what the demo does to a video. `slam-it`, `slam_it` and `slamit` were free on PyPI and crates.io, and `evan-wes/slam-it` on GitHub, on 2026-10-12.

The rename is a clean break with no alias, and nothing computes differently.

- **`slam_it` in code:**
  - the Python package, and the server's, `slam_it_server`;
  - the C++ namespace and include directory;
  - the libraries and programs (`slam_it_foundations`, `slam_it_vo`);
  - the extension modules `slam_it._cpp` and `slam_it_rs`.
- **`slam-it` for everything else:**
  - the distribution and the Rust crates;
  - the conda envs (`slam-it`, `slam-it-ci`) and their lock;
  - the review panel (`/slam-it-review-panel`, `slam-it-*-reviewer`);
  - the Docker images, the viewer's package, the demo's folder (`~/.local/share/slam-it/demo`) and the planned subdomain;
  - the repository, once it is renamed after this merges.
- **Other forms:**
  - `SLAM_IT_` prefixes the CMake options, macros and environment variables, the server's and the worker's settings among them;
  - `SlamItError` is the base exception;
  - the formats are `slam-it-run` and `slam-it-bundle`, so runs and bundles written before the rename no longer load.
- **History keeps the old names:** the CHANGELOG's earlier entries and the review reports in `docs/reviews/`. Their guide, `docs/reviews/README.md`, is updated.
- **Edits beyond the rename:**
  - the ladder's note on the name records both renames and the name check, and its packaging section says reifyscene's pin is a direct git reference to v0.3.7.0 or later;
  - the README's line under the title drops the old pun: "A codebase for a SLAM pipeline in three languages";
  - CLAUDE.md's naming rule says environment variables take `SLAM_IT_`;
  - two items join the ladder's open items (A3, L2 below);
  - ruff format, clang-format and rustfmt joined lines the shorter names freed; nothing was rewrapped by hand.

**Merge order:** this is the only open PR, so there is no order to follow.

**After it merges** (each step waits for your OK, one per message):
1. Rename the GitHub repo to `evan-wes/slam-it` and update the local remote. GitHub redirects the old links.
2. Update the merge exception: your global CLAUDE.md and `ALLOWED_REPO` in the owner's merge-guard hook. Until then, merges in the renamed repo are blocked.
3. Rename the ten edge-note bundles' format id (`rename_bundle_format.py --write`), so the viewer opens them again.
4. Move the folders to the SLAM It! checkout, deleting the git-ignored CMake build trees there (`build/native`, `build/bench`, `build/slam-it-all`), which CMake refuses once moved.
5. The envs: `slam-it` (made for this PR) and `slam-it-ci` (made by offline CI) set up again at the new folder; the old `slam-it-all` and `slam-it-all-ci` envs removed, each its own step.
6. reifyscene's `.claude/` symlinks, a private scripts folder, my memory notes.
7. The old caches (`~/.cache/slam-it-all`, `~/.cache/slam-it-all-offline-ci`, `~/.cache/slamlab-offline-ci`) and the last rename's backups, each its own step.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | claude-opus-5-5 | The rename (paths and the five name forms), the history and the ladder's note, the new env, the review's fixes |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | Ran, as a general-purpose agent following its file: reifyscene's links name the old files until step 6 |
| Architecture review | `slam-it-architecture-reviewer` | opus | Ran, the same way |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | Ran, the same way; ran the old and new programs on real data |

**Review panel:** full panel, on `e428943...6696616`.
Report: [`docs/reviews/2026-10-12-chore-rename-slam-it.md`](../../docs/reviews/2026-10-12-chore-rename-slam-it.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 0 | 0 | 0 | 0 | 0 |
| should-fix | 2 | 2 | 0 | 0 | 0 |
| note | 11 | 4 | 5 | 2 | 0 |

**Owner decisions**

- The name, "SLAM It!" with its exclamation mark; everything renamed, a clean break; the rename next, before phase A.
- Every finding as recommended (H1, H2, H3, A4, A5 and L1 fixed in `478a885`; A1, A2 and H4 deferred to the post-merge steps; A3 and L2 to the ladder's open items; H5 and L3 waived), and the README's line in the owner's own words.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally (exit 0) in the new `slam-it` env, built by the renamed setup script, on `478a885`'s tree (`11e9310` adds only the review report): pytest 3572 passed, ctest 36, cargo test, clippy, node --test 49, 21 import contracts kept.
- [x] Offline CI: `scripts/offline_ci.sh` passed at `11e9310`, in a new `slam-it-ci` env it built from the lock and a new cache (`~/.cache/slam-it-offline-ci`):
  `11e9310f950b344dbfc0d6a3be02c25f453ffee5  chore/rename-slam-it  merged=no  passed  $HOME/.cache/slam-it-offline-ci/logs/11e9310f950b-20261012-151953.log`
- **Nothing computes differently:**
  - every backend matches the room's golden record (24 cases), which the rename did not touch;
  - the algorithm reviewer ran the old and new programs on TUM fr1/desk (RGB-D, 133 frames; monocular, 150) and EuRoC MH_01 (stereo, 200) in all three languages: the 9 runs are identical file for file, features and lifetimes included, but for the per-frame timings, the measured peak memory and `run.json`'s format name (rechecked by me).

#### Version

`0.3.6.0` -> `0.3.7.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-18"></a>slam-it pull request 18: Milestone 2A's A1: a public test set, and phone data imported better (0.3.8.0)

Merged 2026-10-13 · `feature/2a-a1-measure-and-import` → `develop` · 15 commits, +5,014 −271 in 45 files · merge commit `f580d2739d5f027a22f5ca8fcf8125823e4a81c1`

#### Summary

Milestone 2A's first part, A1 (`docs/design/2026-10-12-phone-robust-vo.md`, section 4): a public test set to measure phone-robust VO against, and phone data imported better.

- **Dataset imports to TUM format**, ground truth moved to the camera:
  - `eval.zju`: ZJU-SenseTime's phones;
  - `eval.advio`: ADVIO's iPhone video, through the video import with a dataset's limits, kept distorted with its batch's Kalibr calibration;
  - `eval.arkitscenes`: ARKitScenes' raw captures as RGB-D, ARKit's poses interpolated at each frame's time and named a reference.
- **LiDAR depth as RGB-D** (`eval.lidar`): high confidence by default, medium on request, nearest-neighbour registration. A Stray Scanner zip's depth is imported too, and the demo's worker runs it RGB-D, naming the mode in its summary.
- **Features by frame size** (`edge.options.features_for`, `vo.options_for`): ORB-SLAM's 1,000 at 640x480, scaled with the area within 1,000 to 2,000. The demo and the measure share it.
- **`scripts/measure_testset.py`**: a config's sequences run and measured: the share tracked, when tracking began and ended, the ATE naming its reference, and the p95 latency against the frame period. `--camera nominal` gives what a guessed lens costs. The config stays outside the repo; `scripts/testset.template.toml` is its format.
- **Upload warnings, never refusals:** the status page warns about a video that arrived under 480 pixels on its short side, and the upload page advises the Camera app.
- **The results**, `docs/notes/test-set.md`, from public datasets alone, each change read against three RANSAC seeds:
  - 61 of 77 runs lose the track, and 65% of the set's frames are never run: A2's case;
  - a guessed lens costs accuracy in step with its error, most in RGB-D, but not the track;
  - 720 pixels on ADVIO keeps no longer track at nearly twice the latency, so 480 stays;
  - medium LiDAR confidence changes nothing, so high stays the default.
- **Hardening from the panel and the security pass:**
  - every import staged and recorded in `source.json`;
  - `eval.images` reads PNG alone, sizes checked before decoding, and Pillow's errors made ours;
  - a Stray zip's maps capped and read from memory.

No private recording, path or result is in the repository.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | The imports, depth, features, measure, worker and viewer changes, the measurements and the note |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | Ran, Run 1 |
| Architecture review | `slam-it-architecture-reviewer` | opus | Ran, Runs 1 and 2 |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | Ran, Runs 1 and 2 |
| Security pass | general-purpose agent | opus | Ran, Runs 1 and 2 |

**Review panel:** full panel.
Reports: [`docs/reviews/2026-10-13-feature-2a-a1-measure-and-import.md`](../../docs/reviews/2026-10-13-feature-2a-a1-measure-and-import.md) and [its security pass](../../docs/reviews/2026-10-13-feature-2a-a1-measure-and-import-security.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 3 | 3 | 0 | 0 | 0 |
| should-fix | 15 | 14 | 1 | 0 | 0 |
| note | 21 | 16 | 4 | 1 | 0 |

The counts cover both reports, both runs: 33 panel findings and 6 security findings. Partly deferred:
- L4: Stray Scanner's per-frame intrinsics;
- A14: one shared staging helper;
- L9: the scoring's own pairing of a sparse reference (route (a)).

L3, waived in Run 1, was reopened by Run 2's L9 and fixed by interpolating ARKitScenes' reference. Every deferral is in the ladder's open items ("Deferred from milestone 2A's A1 review panel").

How the must-fixes were resolved:
- **A1:** the viewer's short side and modes are held to Python's by `test_shared_names.py` (c832973).
- **L1:** the measure scores the ATE alone (`runs.evaluate_absolute`, 753b738). Run 2 found L1's premise wrong: the measure already scored ARKitScenes, since each 10 Hz pose served two frames. 753b738's message claimed otherwise, and 5871936's message corrects it. `evaluate_absolute` stays.
- **L9:** with each pose serving two frames, ARKitScenes' relative error measured the camera's own motion. ARKitScenes' reference is now interpolated at each frame's time (`arkitscenes.at_frames`, 5871936). The leave-one-out error is a median of 1.6 mm and 0.28°.

**Owner decisions**

- The design (2026-10-12):
  - A1 before A2;
  - a public test set, the owner's recordings a check on the demo;
  - high LiDAR confidence by default, medium measured;
  - Stray zips with depth run RGB-D;
  - warnings never refuse an upload.
- The feature floor at 1,000, ORB-SLAM's own minimum (2026-10-13, selectable prompt).
- ADVIO as a dataset importer: frames kept distorted with a radtan camera, `frames.csv` as the clock, the dataset's limits, a short-side option (2026-10-13, selectable prompt).
- Panel Run 1 (2026-10-13, selectable prompts): fix both must-fixes; should-fixes and notes as recommended, including the deferrals of H6 with A7, L7 and L8, part of L4, and L3's waiver.
- Panel Run 2 (2026-10-13, selectable prompts):
  - L9 by route (b), interpolating at import, which reopened L3;
  - S6 by reading PNG alone in `eval.images`;
  - the rest as recommended, including the deferrals of A10, part of A14, and L9's route (a);
  - 753b738's false claim noted in the next commit's message.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally at 51a8f31 (3,693 passed).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `51a8f31` (3,693 passed), its line in `~/.cache/slam-it-offline-ci/results.tsv`: `51a8f31e7d5b777a51e3f965237f07193cf68c4f  feature/2a-a1-measure-and-import  merged=no  passed  51a8f31e7d5b-20261013-185359.log`
- **The measurements** are in `docs/notes/test-set.md`:
  - every batch at seeds 0, 1 and 2;
  - the VO checked deterministic for a seed: a rerun of zju-B4 wrote a byte-identical trajectory.

#### Version

`0.3.7.0` -> `0.3.8.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-19"></a>slam-it pull request 19: Milestone 2A, A2: a new map after a lost track (0.3.9.0)

Merged 2026-10-14 · `feature/2a-a2-new-map` → `develop` · 15 commits, +5,422 −1,394 in 100 files · merge commit `878dd2690622d9c79acf7c935e7e4fc9db0c1f5a`

#### Summary

Milestone 2A's second part, A2 (`docs/design/2026-10-12-phone-robust-vo.md`, section 5): a lost track no longer ends a run. Its map closes and the next frame starts a new one, as ORB-SLAM3's atlas does, so a run goes on to the sequence's end.

- **A new map after a lost track**, in Python, C++ and Rust, which decide alike:
  - a map lost with `reset_keyframes` (10) keyframes or fewer is reset, as ORB-SLAM3 resets its active map: its keyframes and points removed, its frames' poses dropped;
  - a larger map is kept;
  - each map numbers its own keyframes and points from 0, and the run's ids continue across maps, so the map, mapping and BA code is unchanged;
  - `min_init_depth_points` is gone (open item L13): any frame with more than 500 keypoints starts a depth map, as in ORB-SLAM.
- **Run format version 4:**
  - each pose, keyframe and point names its map, and each frame's diagnostic too, checked by the writers in all three languages and by `read_run`;
  - `trajectory.txt` marks each map's first pose with a `# map <n>` line;
  - `Run.maps()` and `Run.reset_maps()` name a run's maps and those reset.
- **Scoring by map:** each map is aligned on its own and the errors pooled. A run whose reference is too sparse for the RPE is placed by its ATE alone (A1's deferred A10). A run that cannot be placed raises `DegenerateError`, naming each map's cause.
- **The bake (bundle v2) and the viewer** show one map at a time. The timeline is split by map, and a reset map is drawn broken.
- **The demo's summary and status page** give the stretches tracked, the losses and the maps reset.
- **The lost room**, blanked twice, holds the three languages to the golden record across new maps. A stereo or RGB-D run reaches a third map there.
- **The results**, in `docs/notes/test-set.md`, from the public test set at seeds 0, 1 and 2:
  - **Share tracked:** 87% of the set's frames, against A1's 31%; a run's median share is 90%, against 20%.
  - **Seed sensitivity:** the seed moves a run's share by at most 13 points, against A1's 76.
  - **Up to the first loss:** every frame A1 ran, 87,108 of them, is decided alike.
  - **Losses and maps:** 348 losses over 61 runs, 55 maps reset. A kept map spans a median of 4.5 to 24.6 s by dataset and mode, each map in its own frame, until 2B joins them.
  - **Why the track is lost:** over half the losses are placed frames whose local map kept too few inliers.
  - **Real time:** it breaks only while a monocular map starts again on ADVIO. There are 105 late frames, all failed two-view starts, the slowest 0.43 s; this is deferred as an open item.

No private recording, path or result is in the repository.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | The new map in three languages, run format v4, scoring, bundle v2, the viewer, the worker and status page, the lost room, the test-set rerun and the note, and the panel's fixes |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | Ran, Run 1 |
| Architecture review | `slam-it-architecture-reviewer` | opus | Ran, Runs 1 and 2 |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | Ran, Runs 1 and 2 |

**Review panel:** full panel; Run 2 re-ran architecture and algorithm on the fixes.
Report: [`docs/reviews/2026-10-14-feature-2a-a2-new-map.md`](../../docs/reviews/2026-10-14-feature-2a-a2-new-map.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 3 | 3 | 0 | 0 | 0 |
| should-fix | 15 | 14 | 1 | 0 | 0 |
| note | 19 | 11 | 6 | 2 | 0 |

The counts cover both runs, 37 findings. A finding several reviewers raised is counted under each ID: H1, A1 and L1; A6 and L2; A7 and H5; A14 and L11.

Partly resolved:
- A8: the three languages' close and dispatch made alike; the rest is deferred with the atlas.
- A13: deferred, though its design-doc part was fixed as L5.
- A16 and L10: fixed, with a part of each waived (pairing twice per map; moving the blank stretch).

Every deferral is in the ladder's open items ("Deferred from milestone 2A's A2 review panel").

How the must-fix was resolved:
- **H1 (with A1 and L1):** `evaluate` turned an invalid `rpe_delta` into "no RPE" by catching every input error from a map's RPE. A1's deferred A10 fix had introduced this.
  - Now `rpe_delta` is checked before any map is scored.
  - "No pair" is an explicit test through `metrics.interval_pairs`, and nothing is caught (b682f3c).
  - The four bad values are tested.

**Owner decisions**

- A2's design (2026-10-13, selectable prompts):
  - reset a lost map of 10 keyframes or fewer;
  - drop `min_init_depth_points`;
  - score each map on its own alignment, plus pooled figures;
  - table which check failed at each loss.
- Each map's ids from 0, offset across maps: described to the owner before building, and recorded in the design (f5f4df4).
- Panel Run 1 (2026-10-14, selectable prompts):
  - H1 fixed fully;
  - A6 by the docs and `# map <n>` lines;
  - the other should-fixes and the notes as recommended, A4 deferred;
  - a new open item for the two-view sample limit.
- Panel Run 2 (2026-10-14, selectable prompts): the three should-fixes fixed, and the notes as recommended.

#### Verification

- [x] Layer 1: `scripts/check.sh` passes locally at bb6ab32 with 6d9b3f7's docs in the tree (25 checks; 3,756 passed). 3f1c736 adds the review report only.
- [x] Offline CI: `scripts/offline_ci.sh` passed at `3f1c736` (3,756 passed), its line in `~/.cache/slam-it-offline-ci/results.tsv`: `3f1c7367f29cedc2f2ec81e1a45d20056716f0e9  feature/2a-a2-new-map  merged=no  passed  3f1c7367f29c-20261014-114532.log`
- **Parity:** Python, C++ and Rust decide alike on the golden record's 16 cases. Its room and gap cases are unchanged by this PR, so no decision moved where the track is never lost.
- **The measurements**, in `docs/notes/test-set.md`:
  - the test set at seeds 0, 1 and 2, measured at 6e609fa;
  - the panel's fixes after it change how a lost map is closed only in form, the golden record's room and gap cases unchanged, and leave the ATE's computation as it was;
  - every frame A1 ran was checked frame by frame against A2's.

#### Version

`0.3.8.0` -> `0.3.9.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-20"></a>slam-it pull request 20: Milestone 2B, B1: place recognition and relocalisation (0.3.10.0)

Merged 2026-10-16 · `feature/2b-b1-place-recognition` → `develop` · 33 commits, +17,409 −1,251 in 192 files · merge commit `d14e48f6084bc9294c856b83007cb33b047031d4`

#### Summary

Milestone 2B's first part, B1 (`docs/design/2026-10-14-online-slam.md`, section 3): place recognition and relocalisation. A frame that tracking fails is relocalised against the whole active map through a bag of words, as ORB-SLAM's is, and a lost track is given 3 s to be found again before its map closes, as visual-only ORB-SLAM3's recently lost track is. The three languages relocalise alike.

- **Place recognition's vocabulary** (`slam_it.foundations.vocabulary`, C++ and Rust twins): DBoW2's tree of binary words over our own ORB descriptors (k = 10, 5 levels, 99,961 words in the committed default), alike in the three languages to the bit, held to a brute-force parity fixture. Its folder is eval's (`slam_it.eval.vocabularies`, with readers in `slam_it_eval` and `slam-it-eval`): five `.npy` arrays, a header array whose format and descriptor versions every reader checks, and the arrays' SHA-256, which Python checks before any program runs.
- **The training tool** (`scripts/train_vocabulary.py`), and two 6-level vocabularies kept outside the repository, named in `slam_it/eval/vocabularies/local.json` with the commands that rebuild them.
- **Matching by word, the keyframe database and relocalisation** (`matching.search_by_bow`, `edge.database`, `edge.relocalization`, three languages): ORB-SLAM's SearchByBoW, KeyFrameDatabase and `Tracking::Relocalization`, with its thresholds and MLPnP's RANSAC budget.
- **The pipelines relocalise:** a frame the motion model and the local fallback fail is relocalised; a kept map is lost once `relocalize_seconds` (3) have passed since its first failed frame, a map of 10 keyframes or fewer after 5 failures. `Options.relocalize`, on; the vocabulary required with it, recorded in `run.json` (run format version 5, a clean break).
- **The two-view sample limit:** a monocular start's RANSAC draws at most 200 samples a model, as ORB-SLAM's initialiser does.
- **Counted and shown:** relocalised frames in `runs.tracking` and the measure; the status page says how many frames tracking failed and found again; the viewer marks them and shows the Relocalisation stage; the ground truth faint gold, growing with the trajectory, whole on a toggle; the Solver panel keeps 260 px.
- **The measurement scripts:** `measure_testset.py --option`, `measure_sharded.py` (side by side, several seeds, each result kept as it ends), `compare_testsets.py`.
- **Tests:** the found room and the window room in the golden record (24 cases; the 20 earlier ones unchanged); a golden bundle of the found room for the viewer; relocalisation's settings each moved where it alone decides; native tests of the vocabulary, its reader (20 shared cases), the database, relocalisation and the pipeline relocalising.
- **The results**, in `docs/notes/test-set.md`, the public test set at seed 0, against the same code with relocalisation off: 31 runs relocalise 123 frames, all but one consistent with their map's error against the ground truth; maps fall from 359 to 268 and losses from 348 to 231; but 6.4% fewer frames are tracked (214,583 to 200,897 of 246,529), the 3 s waited through costing more than relocalisation finds. A relocalised frame takes a median of 14.3 to 15.4 ms against a deadline of 32 to 33 ms, none late, made alone. The window is kept for B1 and revisited with B4's merging.

**Corrections to this branch's own commit messages** (left as history):
- 80c960a says ORB-SLAM3 restarts its 3 s on a relocalised frame that fails the local map. It does not: `Tracking::Relocalization` never sets `mState`, so `mTimeStampLost` keeps the first failure's time, as here. The real departures: every failed frame tries the motion model and the local map before relocalising, and the first failed frame relocalises too (panel run 1, L3).
- c3f20da says the agreement of the backends with eight relocalisation settings off their defaults shows the natives read them. Six of the eight decided nothing there; 43d7650 moves each where it alone decides, and tests that it does (panel run 2, L9).

No private recording, path or result is in the repository.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | B1 in three languages, the scripts, the viewer and status page, the measurements and the note, and both panel runs' fixes |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | Ran twice: on the slice, then on the fixes |
| Architecture review | `slam-it-architecture-reviewer` | opus | Ran twice: on the slice, then on the fixes |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | Ran twice, with web search: on the slice, then on the fixes |

**Review panel:** full panel, two runs. Report: [`docs/reviews/2026-10-16-feature-2b-b1-place-recognition.md`](../../docs/reviews/2026-10-16-feature-2b-b1-place-recognition.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 6 | 6 | 0 | 0 | 0 |
| should-fix | 29 | 29 | 0 | 0 | 0 |
| note | 26 | 20 | 5 | 1 | 0 |

Run 1 found 37 (5 must-fix: the C++ reader's out-of-range reads, the native programs trusting the vocabulary, no vocabulary fixture or native tests, the 3 s window and relocalisation's settings untested natively, a culled keyframe leaving the database untested). Run 2, on the fixes, found 24 (1 must-fix: six of the moved settings decided nothing). Every must-fix was decided by evan-wes and fixed; how each was resolved is in the report's tables.

**Owner decisions**

- The 3 s recently-lost window accepted for B1, though it costs tracked frames on the test set; revisited with B4's merging.
- The status page counts frames found again, not times the track was dropped.
- The measurement scripts in the repository; ZJU B0 as the viewer's example bundle; the Solver panel's 260 px; the ground truth faint gold, growing, with a "whole truth" toggle.
- The 16G memory cap for the 24-worker measurement.
- Run 1: all must-fix findings fixed; the native readers check the vocabulary's header themselves, the checksums staying Python's (the ladder's one recorded exception); run format version 5, a clean break; every should-fix and note as recommended (A12 deferred to B2's design, A13 to after this merge, L7 to B4's design, L8 waived).
- Run 2: L9 fixed; every should-fix and note as recommended (A21 and L14 deferred to the ladder's open items); the viewer's `placed_by ?? []` kept, as it serves no older bundle.
- Layer 1 runs once per PR, when it is ready to merge; the commits since ran the checks their changes touch, each naming them.
- The design doc's corrections (run 2, A17), approved and committed on develop as c41a541.
- The two kept 6-level vocabularies converted to the new format outside the repository, additively (run 2, H22).

#### Verification

- [x] Layer 1: `scripts/check.sh`, run once on the finished branch, inside offline CI (below).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `7d99604` (`7d99604cdf0141141d8bb43031ea19949273e34c  feature/2b-b1-place-recognition  merged=yes  passed  $HOME/.cache/slam-it-offline-ci/logs/7d99604cdf01-20261016-131453.log`): every layer-1 check, 4,110 Python tests, on the branch merged with develop at c41a541.
- Each commit since the first panel ran the checks its change touches (named in its message): ruff, mypy, import-linter, `check_layers.py`, clang-format, clang-tidy, clippy, rustfmt, ctest, cargo test, jscpd, the viewer's checks, and the pytest files of what it changed.

#### Version

`0.3.9.0` -> `0.3.10.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-21"></a>slam-it pull request 21: Milestone 2B, B2: the covisibility graph (0.3.11.0)

Merged 2026-10-17 · `feature/2b-b2-covisibility` → `develop` · 18 commits, +13,256 −3,822 in 159 files · merge commit `7e17b6fa35babe1191dc2c37d5ec16bc12e3be16`

#### Summary

Milestone 2B's second part, B2 (`docs/design/2026-10-14-online-slam.md`, section 8): the covisibility graph. The window of the last 10 keyframes is gone: tracking's local map, the local fallback, local BA, keyframe culling, triangulation and fusion take their keyframes from the graph, as ORB-SLAM's do, in the three languages alike.

- **The settings grouped by stage** (features, matching, stereo, initialization, tracking, adjustment, keyframes, mapping, covisibility, relocalization, loss, and the seed), each named `group.field` on a command line, in an error and in a run's record: 97 in all. A clean break.
- **The covisibility graph** (`slam_it.edge.covisibility`, C++ and Rust twins): two keyframes' weight counted from the map's observations when asked; a keyframe's links (at least 15 shared, or its best); tracking's local map, K1 and K2 as the papers describe it, to 80, and the frame's reference keyframe; the strong links (100). Held to a brute-force fixture in the three languages.
- **The spanning tree:** each keyframe's parent (`Map.kf_parent`), set as ORB-SLAM's first `UpdateConnections` sets it; a keyframe leaves the map only by `covisibility.cull` (`SetBadFlag`: its children reparented, then it erased, the root refused); `map.erase_keyframe` refuses a keyframe that is still a parent. Kept for B3's essential graph.
- **The stages off the window:** local BA over the newest keyframe and every link; culling over its links; triangulation over its best 10 (monocular 20); fusion over its best 10 (20) and each one's best 5, in ORB-SLAM3's order; the keyframe decision against the reference keyframe; the local fallback against the last tracked frame's local map; relocalisation's groups from each candidate's best links. What fusion and local BA take is public (`mapping.fuse_targets`, `mapping.local_ba_keyframes`) and held to the fixture.
- **Run format 6** (`keyframe_parents.npy`, `keyframe_links.npy`) and **bundle format 3**, in every language; the **viewer draws the graph** on a toggle, as each map ended.
- **Tests:** the fixture (5 cases, every K2 route, the local map at its cap, a tree repaired in rounds, fusion's counts each binding); the golden record regenerated (every room keeps its states and tracked frames; 12 cases make more keyframes), every map of it one tree; B2's settings moved in the whole runs the languages are compared on; the keyframe decision deciding both ways against two references; the run format's checks in the three languages.
- **The results**, in `docs/notes/test-set.md`, the public test set at seed 0 against B1's runs:
  - 208,906 of 246,529 frames tracked against 200,897 (84.7% against 81.5%), in 226 maps against 268, losses 231 to 177; the losses that began with a local map left too few inliers 152 to 91;
  - 34 frames relocalised in 19 runs against 123 in 31, 33 of them within three times their map's error;
  - the ladder's L15, closed: none of the 20 later-started monocular runs on TUM and EuRoC is lost, where PR 8 lost 6;
  - made alone on the five largest maps, tracking keeps its deadline but for 172 of 35,066 frames, and mapping, judged as a thread of its own, is loaded 1.7 to 2.6 times over: inline and never busy, it neither refuses a keyframe nor cuts a local BA short, as ORB-SLAM's mapping thread does. Put to evan-wes, who left the stand-in for ORB-SLAM's threads to B3's design.

**Corrections to this branch's own commit messages** (left as history):
- 902b9a4 says the settings that comments name are `group.field` in the C++ and Rust edge code and its tests and in the Python tests' docstrings. Some in the C++ and Python tests were not, nor were `relocalization.py`'s docs (panel run 1, H4); db9dd1f finishes them.
- 7ce41f2 says the fixture's fusion counts are "low enough to bind". The monocular one bound on no case (panel run 2, L8); 78fde39 makes each bind and asserts it, as evan-wes chose to correct it.

No private recording, path or result is in the repository.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code | Opus 5.5 | B2 in three languages, the run and bundle formats, the viewer's graph, the measurements and the note, and both panel runs' fixes |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | Ran twice: on the slice, then on the fixes |
| Architecture review | `slam-it-architecture-reviewer` | opus | Ran twice: on the slice, then on the fixes |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | Ran twice, with ORB-SLAM2 and ORB-SLAM3's source: on the slice, then on the fixes |

**Review panel:** full panel, two runs. Report: [`docs/reviews/2026-10-16-feature-2b-b2-covisibility.md`](../../docs/reviews/2026-10-16-feature-2b-b2-covisibility.md).

| Severity | Found | Fixed | Deferred | Waived | Disputed |
|---|---|---|---|---|---|
| must-fix | 5 | 5 | 0 | 0 | 0 |
| should-fix | 22 | 22 | 0 | 0 | 0 |
| note | 20 | 18 | 1 | 1 | 0 |

Run 1 found 32 (5 must-fix rows, 4 distinct: culling an empty map, Python's run-format-6 checks untested, B2's settings never moved in a whole run, the spanning tree without an owner). Run 2, on the fixes, found 15, none must-fix. Every finding was decided by evan-wes; how each was resolved is in the report's tables.

**Owner decisions**

- B2's design: the graph counted when asked, the tree alone stored; the papers' K2, not the source's `break`; the settings grouped by stage; the viewer's graph on a toggle.
- The latency, put with its numbers as the design asked: B2 records them, and the stand-in for ORB-SLAM's threads (the busy mapping thread's keyframe refusal and BA interrupt, with loop closing's) is decided in B3's design; the lost map's strong-links record stays timed in its frame until B4's atlas.
- Run 1: every must-fix fixed; for A1, `covisibility.cull` added now, with the erase that refuses a parent, rather than waiting for B3's second caller; every should-fix and note as recommended (A4's every-fixture check and A7's graph through time deferred to the ladder's open items, A9's comparison by summary and L5 to B3's outline, L7 waived, A10's normalising write waived).
- Run 2: every finding fixed; 7ce41f2's claim corrected in 78fde39's message, history kept.
- Layer 1 runs once per PR, when it is ready to merge; the commits ran the checks their changes touch, each naming them. The 24-worker measurement under a 16G cap.

#### Verification

- [x] Layer 1: `scripts/check.sh` on the finished slice before the panel (390d67d: 25 checks, pytest 4,197 passed), and on the final head inside offline CI (below).
- [x] Offline CI: `scripts/offline_ci.sh` passed at `64e42ad` (`64e42ade58efc8104a315dabc3af797e3f2645ce  feature/2b-b2-covisibility  merged=no  passed  $HOME/.cache/slam-it-offline-ci/logs/64e42ade58ef-20261016-233731.log`): every layer-1 check (25), 4,250 Python tests, on the branch, develop (9a4f499) its ancestor.
- Offline CI's first run, on 29a85e0, failed on jscpd alone (its other 24 checks passed, 4,250 Python tests): two clones the panel's fixes added, whose commits had not run jscpd. 64e42ad makes each one, and the run above is on it.
- Each commit since the first panel ran the checks its change touches (named in its message): ruff, mypy, clang-format, clang-tidy, doxygen, rustfmt, clippy, ctest, cargo test, the viewer's checks, and the pytest files of what it changed, with the golden-record and parity tests where a change reached the pipeline.

#### Version

`0.3.10.0` -> `0.3.11.0` (PATCH), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

### <a id="slam-it-pr-22"></a>slam-it pull request 22: A feature mask: drop features on an overlay or the car's body (0.3.11.1)

Merged 2026-10-17 · `feature/2b-feature-mask` → `develop` · 3 commits, +1,084 −104 in 24 files · merge commit `e5c5272318bb266e487d9009990a9acfc27dff28`

#### Summary

- **A feature mask** in Python, C++ and Rust: a TUM-format run folder's optional `mask.png`
  (8-bit, the frames' size, 0 = ignore). Each feature whose keypoint's nearest pixel is 0 is
  dropped after extraction (`edge.frames.unmasked`, the frame builders' `mask`; a stereo
  pair's left image only). ORB extraction and its cross-language parity are unchanged; without
  a mask every run is as before.
- **The video import takes a mask** (`import_video(mask=...)`, `--mask PNG`) at the video's
  stored size, carried through the frames' scaling, fisheye undistortion and rotation;
  `source.json` records whether one was given.
- **Why:** dashcam footage has a burned-in overlay (GPS, speed, clock) and the car's own body
  in view, which never move with the scene and would be tracked as static features.
- **Not in this PR:** `run.json` does not record the mask; its keys are a checked set, so that
  waits for the next run-format bump (B4's format 9).

Independent of the other open work: branched from `develop`, nothing stacked on it. B3's PR,
not yet opened, rebases onto `develop` after this merges.

#### How this was built

**Agents**

| Role | Agent | Model | What it did |
|---|---|---|---|
| Implementation | Claude Code (subagent) | claude-opus-5-5 | the filter, the readers, the runs' plumbing and the import's mask in three languages, with tests |
| Hygiene review | `slam-it-hygiene-reviewer` | sonnet | not run: MICRO change |
| Architecture review | `slam-it-architecture-reviewer` | opus | not run: MICRO change |
| Algorithm review | `slam-it-algorithm-reviewer` | opus | not run: MICRO change |

**Review panel:** none (a MICRO change; the panel runs on PATCH and larger).

**Owner decisions**

- The design (2026-10-17): filter after extraction rather than inside FAST detection, so ORB
  and its parity are untouched (the budget is not redistributed: a frame loses the features
  that fell on the mask); `mask.png`, 0 = ignore (COLMAP's convention); the import carries the
  mask; its own branch off `develop`.
- "we don't need full layer 1 since we have no masks for it anyway": touched checks only.
- Asked whether to run the review panel: skipped, as a MICRO change.
- Offline CI skipped for this PR, by the owner's call.

#### Verification

- [ ] Layer 1: not run (owner's call; touched checks below)
- [ ] Offline CI: skipped (owner's call)
- Touched checks, each exit 0, run under an 8 GB memory scope:
  - Python: ruff check, ruff format, mypy (204 files), lint-imports (22 contracts kept),
    `scripts/check_layers.py`, jscpd (0 clones);
  - `pytest` with every backend required, on `tests/edge/test_frames.py`,
    `tests/eval/test_tum.py`, `tests/eval/test_video.py` and `tests/test_backends.py`:
    162 passed;
  - `tests/orchestration/test_vo.py`: a masked run in each backend, and golden-record cases
    with no mask (tum RGB-D, tum monocular, EuRoC stereo) in all three backends reproducing
    exactly: 12 passed;
  - C++: build at -j4, ctest 41/41, clang-format and clang-tidy on the touched files,
    doxygen with 0 warnings;
  - Rust: cargo fmt, clippy -D warnings, cargo test (slam-it-edge, slam-it-eval,
    slam-it-orchestration), `maturin develop --release`, the native program rebuilt.

#### Version

`0.3.11.0` -> `0.3.11.1` (MICRO), CHANGELOG updated.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
