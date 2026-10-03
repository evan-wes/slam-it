# Review panel: `feature/absolute-pose`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-03

| | |
|---|---|
| **Branch** | `feature/absolute-pose` |
| **Range** | `a0709ca...4a4d078` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`): pytest 1301 passed, ctest 17/17 |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L1 | should-fix | algorithm | `p3p.py` `_ding`, `p3p.cpp` `ding` | Ding's "single real root: skip the second line" break (inherited from PoseLib) drops true poses near the danger cylinder; worst just outside it, which the probe never sampled | fix: always intersect both lines; probe and fixture outside the cylinder; re-measure | fixed (4c0bc5e) -- decided by evan-wes, 2026-10-03 |
| L2 | should-fix | algorithm | `pnp.py` `PoseEstimator.fit_nonminimal` | On planar scenes EPnP refuses, LO does nothing, and PnP RANSAC lands at 2.0x the floor (p90 8.4x); the docstring says planar scenes are handled | fix: refine, re-collect inliers, repeat until stable (generic in `refine_consensus`); planar test | fixed (182c552) -- decided by evan-wes, 2026-10-03 |
| L3 | should-fix | algorithm | `docs/notes/absolute-pose.md`, probe | "Far away every solver loses 18%" is the absolute 1e-6 test at 1000 m, not a solver loss (relative test: 3 of 2000) | fix: relative translation test, restate | fixed (f786d3c; probe updated) -- decided by evan-wes, 2026-10-03 |
| A1 | should-fix | architecture | `p3p.py`, `pnp.py`, design doc Conventions | PnP returns `T_cam_world`, the doc says results are `T_world_cam` | fix the doc: estimators return the transform they name | fixed (f786d3c) -- decided by evan-wes, 2026-10-03 |
| A2 | should-fix | architecture | `cpp/bindings/foundations/pnp.cpp` `codes()` | `minimal_solver`'s table is missing from `pnp.codes()`, unlike `epipolar.codes()` | fix | fixed (ece7779) -- decided by evan-wes, 2026-10-03 |
| A3 | should-fix | architecture | `pnp.normalised` vs `correspondences.normalised_plane` | The normalised-plane projection exists twice, in both languages | fix: one shared helper in `correspondences` | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H1 | should-fix | hygiene | `pnp.py` `_epnp`, `p3p.py` `_pose` | "Align, then build a 4x4 pose" written twice | fix | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H2 | should-fix | hygiene | `p3p.py`/`p3p.cpp` internals | Internal helpers take `(bearings, points)`, the API `(points, bearings)` | fix: `(points, bearings)` throughout | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H3 | should-fix | hygiene | `p3p.py` `_ding` | Point reordering by nested ifs; `a`, `b` computed twice | fix | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H4 | should-fix | hygiene | `ransac.py` `run`, `run_refined` | The two duplicate the `backends.call` wiring; `target` tuple is awkward | fix (with A5) | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H5 | should-fix | hygiene | `pnp.py` `ReprojectionProblem` | The refinement does not reject a step that moves a point behind the camera | fix (with L4): infinite cost when any z <= 0; test | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H6 | should-fix | hygiene | `scripts/demo_absolute_pose.py` | Unnamed numbers, an ownerless "(open item)", long `survey`/`main` | fix | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| H7 | note | hygiene | `backends.py` | `dispatch` and `call` coexist | waive: two forms by design (shared positional arguments vs differing sides), both documented | waived: two forms by design (shared positional arguments vs differing sides), both documented -- decided by evan-wes, 2026-10-03 |
| H8 | note | hygiene | `test_pnp.py` | `test_the_refinement_lowers_the_error` silently falls back to Python | fix: parametrise over the lockstep backends | fixed (182c552) -- decided by evan-wes, 2026-10-03 |
| H9 | note | hygiene | `p3p.py`, `test_tum.py` | Validation repeated in `solve_reference` (needed); no malformed `depth.txt` test | fix: add the test; keep the validation | fixed (0f36d41; the validation stays) -- decided by evan-wes, 2026-10-03 |
| A4 | note | architecture | `pyproject.toml`, `ransac.py` | `ransac` now depends on `nls`; a separate refined-RANSAC module would have kept them siblings | waive: NLS is light; revisit if a third refined estimator appears | waived: NLS is light; revisit if a third refined estimator appears -- decided by evan-wes, 2026-10-03 |
| A5 | note | architecture | `ransac.run_refined` | Hard-codes its callers' `minimal_solver`/`refine` binding signature | fix with H4: callers pass their extra arguments | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| A6 | note | architecture | `ransac.py`/`ransac.hpp` `refine_consensus` | The callback gets a mask in Python, indices in C++ | defer (design doc, PR 5: align when Rust gains RANSAC) | deferred (design doc, deferred from PR 4's panel) -- decided by evan-wes, 2026-10-03 |
| A7 | note | architecture | `eval/tum.py`, demo | Depth PNGs are decoded in the demo (cv2), not the reader | defer (design doc open items, milestone 2) | deferred (design doc, deferred from PR 4's panel) -- decided by evan-wes, 2026-10-03 |
| A8 | note | architecture | demo scripts | SIFT front end reimplemented per demo | defer (design doc open items, milestone 2's front end) | deferred (design doc, deferred from PR 4's panel) -- decided by evan-wes, 2026-10-03 |
| L4 | note | algorithm | `pnp.py` `ReprojectionProblem` | Cheirality ignored inside LM | fix with H5 | fixed (0f36d41) -- decided by evan-wes, 2026-10-03 |
| L5 | note | algorithm | `p3p.py` `quadratic_roots` | `REAL_TOLERANCE` turns near-real complex pairs into near-tangent poses | waive: harmless once L1 is fixed (RANSAC scores them) | waived: harmless once L1 is fixed (RANSAC scores them) -- decided by evan-wes, 2026-10-03 |
| L6 | note | algorithm | `pnp.py` | SQPnP (ECCV 2020) handles planar and non-planar in one globally optimal solver | defer (design doc open items, with IPPE) | deferred (design doc, found while building PR 4) -- decided by evan-wes, 2026-10-03 |

Also decided with the findings: the two-view crash on a rank-1 homography (found while
building, PR 2's code) is fixed in this PR (ece7779) -- decided by evan-wes, 2026-10-03.

Measured effect of the fixes (the absolute-pose note): Ding misses 42 true poses grossly of
2000 1e-8 outside the danger cylinder (485 with the shortcut); PnP RANSAC on a planar scene
1.21x the floor (2.02x before the rounds); E-RANSAC's refined median 0.78 of the 8-point floor
(0.71 with one refinement), p90 2.94 (3.21).

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `git diff a0709ca...4a4d078` (56 files). Read in full: `src/slamlab/foundations/{pnp,p3p}.py`, `ransac.py` diff, `epipolar.py` diff, `ba/solve.py` diff, `nls.py` diff, `arrays.py` diff, `backends.py` diff, `eval/tum.py` diff, `tests/foundations/test_pnp.py`, `test_p3p.py` (test names), `test_tum.py`, `test_backends.py`, `scripts/demo_absolute_pose.py`, `cpp/bindings/foundations/pnp.cpp`, C++ constants/NOLINT usage. Skimmed the C++ solver bodies for constants and style only (not line by line). Did not review the generated fixtures or the CHANGELOG/notes prose.

**Summary:** Clean, well-documented, production-quality code. Validation at every public boundary, named and justified tolerances, no bare excepts or swallowed errors, and tests that assert on values and cross-check against OpenCV, SciPy and PoseLib. No must-fix findings. The should-fix items are readability and small duplication points, plus one unguarded corner in the pose refinement and some demo-script hygiene.

#### Findings

##### [should-fix] Pose-from-alignment construction is written twice
- **Where:** `src/slamlab/foundations/pnp.py:316-320` (in `_epnp`) and `src/slamlab/foundations/p3p.py:283-293` (`_pose`)
- **Issue:** Both call `alignment.align`, catch `DegenerateError`, and assemble `pose[:3,:3], pose[:3,3] = fitted.rotation, fitted.translation`. Identical logic, only the error-handling shape differs slightly.
- **Why it matters:** A change to how a pose is assembled (e.g. a determinant check, or a different return type) has to be made in two places and can drift.
- **Evidence:** pnp.py `pose = np.eye(4); pose[:3, :3], pose[:3, 3] = fitted.rotation, fitted.translation` vs p3p.py `_pose`.
- **Suggested fix:** Have `_epnp` call `p3p._pose`, or lift one shared helper into `alignment` (e.g. `align_pose(source, target) -> Pose | None`).

##### [should-fix] Argument order flips between private helpers
- **Where:** `src/slamlab/foundations/p3p.py:363` (`_lambda_twist(bearings, points)`), `:467` (`_ding(bearings, points)`), `:303` (`_finish(depths, bearings, points, geometry)`), `:283` (`_pose(depths, bearings, points)`) — while the public API and `solve_reference` use `(points, bearings)`.
- **Issue:** Internals reverse the order used everywhere else, and the C++ port mirrors it (`cpp/src/foundations/p3p.cpp:211` carries a `NOLINT(bugprone-easily-swappable-parameters): (bearings, points), as in Python`).
- **Why it matters:** Both are `(3,3)` float arrays, so a swap passes type-checking and shape checks and fails only numerically — easy to introduce when adding a caller.
- **Suggested fix:** Use `(points, bearings)` consistently in all internal functions in both Python and C++.

##### [should-fix] `_ding` point-reordering branch is hard to read
- **Where:** `src/slamlab/foundations/p3p.py:469-476`
- **Issue:** Nested `if/elif` choosing an ordering by hand-written index lists makes the reader verify the permutation logic manually; the intent ("make the 2–3 distance the longest") is only in a comment. `a01, a02, a12` are also recomputed after reordering, and `a = a01/a12, b = a02/a12` is recomputed again inside `_ding_depths` (`:516`).
- **Suggested fix:** Choose the longest pair with `np.argmax` over the three squared distances and map to a fixed permutation table; compute `a, b` once and pass them in `geometry`.

##### [should-fix] `run` / `run_refined` duplicate the backend-call wiring
- **Where:** `src/slamlab/foundations/ransac.py` — `run` (end of file) and `run_refined` (just above `refined_summary_from_backend`); also `from_backend` vs `decoder` overlap.
- **Issue:** Both build `{**data, "options": asdict(options), ...}`, unpack `module_name, function = target`, and call `backends.call(..., decode=decoder(...))`; they differ only in the two extra keys and the summary decoder. The `target: tuple[str, str]` parameter is an awkward pair-of-strings pattern.
- **Suggested fix:** Have `run_refined` call a shared private helper (or `run` with extra kwargs), or pass `module_name`/`function` as two keyword arguments.

##### [should-fix] Pose refinement does not guard iterates crossing behind the camera
- **Where:** `src/slamlab/foundations/pnp.py:384-431` (`ReprojectionProblem._projection` / `linearize`); only the *starting* pose is checked (`refine_pose` `:518`).
- **Issue:** The residual `x/z` and Jacobian `1/z` are unguarded. If an LM step makes `z <= 0` for some point, the cost is finite but wrong (or inf/NaN at `z=0`, which `nls` rejects at `nls.py:623`).
- **Why it matters:** `_reprojection` (used by RANSAC) treats `z<=0` as infinite, but the refinement treats it as a normal residual — inconsistent definitions of the same cost.
- **Evidence:** No failing case constructed; risk is low since the refinement starts near the optimum with Cauchy loss and few iterations.
- **Suggested fix:** In `squared_norms`, return a non-finite/infinite cost when any `z <= 0` so LM rejects that step; add a test starting with a point barely in front.

##### [should-fix] Demo script: magic numbers, long functions, unowned note
- **Where:** `scripts/demo_absolute_pose.py` — `:260/:274` survey stride `40`, offsets `(15, 30)`, `skipped[...] += 2`; OpenCV `iterationsCount=10000`, `confidence=0.999`, `len(inliers) < 4`; comment at `:166` "(open item)"; `survey` and `main` each ~50 lines with multiple phases.
- **Issue:** Survey constants are documented in the docstring ("every 40 frames… 15 and 30 later") but not named; the "open item" comment about rank-deficient homography is a TODO without an owner/issue reference; `main` mixes arg parsing, single-frame reporting and survey dispatch.
- **Suggested fix:** Add `SURVEY_STRIDE`, `SURVEY_OFFSETS`, `OPENCV_ITERATIONS`, `OPENCV_CONFIDENCE`; replace "(open item)" with a reference to the milestone note/issue; split `main` into `report_single` and `report_survey`.

##### [note] Two backend dispatch mechanisms now coexist
- **Where:** `src/slamlab/backends.py` — `dispatch` (array in/out) and `call` (reference-lambda + dict + `decode`).
- **Issue:** `call` is documented as serving "the rest", which is reasonable, but `dispatch` could be a thin wrapper over `call` (with `decode=lambda raw, _: np.asarray(raw, dtype=np.float64)`).
- **Why it matters:** Two code paths that both do `check_name` → reference → `function(...)` are two places to update. Not urgent.

##### [note] `test_the_refinement_lowers_the_error` silently falls back to Python
- **Where:** `tests/foundations/test_pnp.py` — `backend="cpp" if "cpp" in backends.available() else "python"`
- **Issue:** The test passes on whichever backend is present, so a missing C++ build silently changes what is tested. Minor, since CI requires cpp (`test_required_backends_are_available`); parametrising over `LOCKSTEP_BACKENDS` with `require_function` like the other tests would be more consistent.

##### [note] Smaller items
- Collinear-sample handling and solver-name validation are checked twice (`p3p.p3p` `:619` and again in `solve_reference` `:570`). The second check is needed because `PoseEstimator` calls `solve_reference` directly, so this is noted, not a request.
- `tests/eval/test_tum.py` covers depth association (including unsorted stamps) but not a malformed `depth.txt` line, even though `_stamped` now names the file and line.

#### Checked and fine
- Input validation at public boundaries (`as_correspondences`, `_as_pose`, `refine_pose`, `pnp_ransac`): shapes, finiteness, unit bearings, positive z, minimum counts, scale, solver name — with messages naming the bad value.
- No bare `except`, no swallowed errors in library code; `LinAlgError`/`DegenerateError` are caught narrowly with documented fallback semantics.
- Named, justified constants with measurement-backed reasons (`DISTANCE_TOLERANCE`, `REAL_TOLERANCE`, `PLANAR_TOLERANCE`, `COLLINEAR_TOLERANCE`, `EPNP_MINIMUM`), mirrored in the C++ headers with the same values.
- Numpy-style docstrings with shapes, frames (`T_cam_world`), the `(ρ, θ)` retraction order and normalised-plane units; module docstrings explain placement and cite the papers; Doxygen on bindings with `@throws`.
- Generic `backends.call` is well documented and tested (`test_call_*`); qualified `ransac_termination`/`nls_termination` keys are consistent across Python, C++ and tests (`test_the_compiled_codes_match`); the BA nested-options dict and the generic `RefinedSummary`/`refine_consensus`/`estimate_refined` remove the E-RANSAC-specific duplication cleanly.
- Tests assert on values and cover edge cases: exact-data EPnP at N = 5, 6, 20, 200; planar, too-few, behind-camera, collinear, coinciding bearings, bad input; finite-difference Jacobian check; SciPy least-squares equivalence; OpenCV and PoseLib cross-checks; per-backend fixtures.
- No debug prints in library code, no commented-out code, no hard-coded paths (the demo defaults to `../datasets/tum`, consistent with CLAUDE.md); `print` is confined to the demo and fixture-generator scripts.
- Vectorisation is appropriate; per-sample Python loops are confined to the minimal solvers, which are inherently scalar.
- `eval/tum.py` depth association is correct (sorts a copy of the depth stamps, same tolerance, `None` per frame when out of range, all `None` without `depth.txt`).

### Architecture review

**Scope:** `git diff a0709ca...4a4d078` in the repository root (6 commits, 56 files). I read the new `foundations.p3p` and `foundations.pnp` modules (Python, C++ headers/sources, bindings), the refactored `ransac`, `epipolar`, `homography`, `twoview`, `ba/solve`, `backends.call`, the `pyproject.toml` layer change, `check_layers` test, `eval.tum` depth association, `scripts/demo_absolute_pose.py`, the p3p/pnp fixtures, and the design-doc edits.

**Summary:** Structurally sound and faithful to the design. P3P and PnP sit on rung 0 in sensible layers, each with its own module, `Implementation` Protocol, lock-step C++ core, thin one-file bindings, and tolerance-policy fixtures. The folded-in refactors (`backends.call`, qualified `codes()` keys, BA options-as-one-dict, generic `RefinedSummary`) genuinely remove five hand-rolled dispatch copies and the essential-only crossing header with no shims left behind. No must-fix. The main points are a divergence between PnP's public pose convention and the design doc's rule for results, a gap in pnp's own `codes()` table for its `minimal_solver` code, and a duplicated normalised-plane helper.

#### Findings

##### [should-fix] PnP returns `T_cam_world` to callers, against the design doc's "results are `T_world_cam`" rule
- **Where:** `src/slamlab/foundations/pnp.py:327` (`epnp`), `:480` (`refine_pose`), `:657` (`pnp_ransac`); `src/slamlab/foundations/p3p.py` `p3p()`; `cpp/include/slamlab/foundations/pnp.hpp`, `p3p.hpp`; `docs/design/2026-10-02-slamlab-ladder.md:162-164`
- **Issue:** The design doc states "Inside a solver, a camera pose is the world-to-camera `T_cam_world` … Results returned to callers are camera-to-world `T_world_cam`." BA follows that rule (`ba/problem.py:51` takes `T_world_cam` and inverts internally). The new public PnP/P3P entry points take and return `T_cam_world`. (Two-view's relative `T_c2_c1` is also not clearly covered by the rule.)
- **Why it matters:** Rung 0 now has two conventions at its public surface, with the doc's rule unstated for one. Milestone 2's tracker will chain PnP poses with BA poses, and a silent inversion mismatch there is the classic SLAM bug. The design doc is the authority; either the code or the doc should change so they agree.
- **Evidence:** `pnp.pnp_ransac` docstring: "Returns … ``T_cam_world`` and its inliers"; design doc line 164: "Results returned to callers are camera-to-world `T_world_cam`".
- **Suggested fix:** Prefer amending the doc rather than the code — `T_cam_world` is the standard PnP output (OpenCV, PoseLib) and is unambiguous by name. Narrow the rule in "Conventions": trajectory-level results (`SlamResult`, BA problems) use `T_world_cam`; rung-0 geometric estimators (P3P/PnP/two-view) return the transform they name (`T_cam_world`, `T_c2_c1`), stated in every signature. Record the decision in the absolute-pose note's interface section.

##### [should-fix] `pnp.codes()` omits the `minimal_solver` table that `pnp_ransac`'s code crosses as
- **Where:** `cpp/bindings/foundations/pnp.cpp:23` (`codes() { return refined_codes(); }`), `src/slamlab/foundations/pnp.py:709`, `tests/foundations/test_pnp.py:276-281`
- **Issue:** `pnp_ransac` sends `minimal_solver` as `p3p.SOLVERS.index(...)` and the binding decodes it as `p3p::Solver`, but `pnp.codes()` exposes only `ransac_termination`/`nls_termination`, so pnp's own test cannot hold that code. It is held only indirectly via `test_p3p.py:179` against the p3p submodule. E-RANSAC, the precedent this PR mirrors, does include `minimal_solver` in `epipolar.codes()`, checked by `test_epipolar.py:401`.
- **Why it matters:** The doc says "a test holds each module's Python tables to its `codes()`". If a solver were added to `p3p::Solver` and pnp were left unchanged, nothing in pnp's tests would fail. A rule held in one module and not its sibling is how the next module gets it wrong.
- **Evidence:** `test_pnp.py:276` `test_the_compiled_codes_match` asserts only the two termination tables; `epipolar.cpp` `codes()` has `out["minimal_solver"] = solvers`.
- **Suggested fix:** Add `out["minimal_solver"]` (from `p3p::name`) to `pnp.codes()` and assert `tuple(codes["minimal_solver"]) == p3p.SOLVERS` in `test_pnp.py`.

##### [should-fix] Normalised-plane projection now exists twice in rung 0, in both languages
- **Where:** `src/slamlab/foundations/pnp.py:137` (`normalised`) vs `src/slamlab/foundations/correspondences.py:46` (`normalised_plane`); `cpp/src/foundations/pnp.cpp:279` vs `cpp/include/slamlab/foundations/correspondences.hpp:39`
- **Issue:** `pnp.normalised(b)` is `correspondences.normalised_plane(b)[:, :2]` — the same concept (bearings → normalised image points, positive z) redefined in the consumer module rather than shared from `correspondences`, which already sits below `pnp`.
- **Why it matters:** The review's "shared components across rung 0 live in one place" rule. Milestone 2's tracker will use both relative and absolute pose on the same bearings; two helpers that differ only in returning a homogeneous coordinate invite drift (e.g. in how z <= 0 is handled).
- **Evidence:** `pnp.py:137-139` returns `bearings[:, :2] / bearings[:, 2:3]`; `correspondences.py:46-59` returns `bearings / bearings[:, 2:3]`.
- **Suggested fix:** Use `correspondences.normalised_plane(...)[:, :2]` in pnp, or add a 2-column variant beside `normalised_plane` in `correspondences` (Python and C++) and use it from both.

##### [note] Generic RANSAC now depends on NLS; a separate "refined RANSAC" module would have kept them siblings
- **Where:** `pyproject.toml:142` (`"(nls)"` moved into its own layer), `src/slamlab/foundations/ransac.py:64,415-705`, `cpp/include/slamlab/foundations/ransac.hpp` (now includes `nls.hpp`)
- **Issue:** This was the owner's decision (PR 3 A6), recorded in the doc. The cost: every RANSAC user, including future edge-rung users that never refine (F, H), now compiles and imports NLS; `ransac.py` grew by ~220 lines of a second concept (refined summary, consensus polish, refined dispatch), and rung 0's layer order changed to permit it.
- **Why it matters:** Not a defect — NLS is light and both are rung 0. But the alternative (a `foundations.refined` module in the `epipolar | homography | pnp` layer or between it and ransac/nls) would have kept ransac and nls independent and left the layer order untouched. Worth revisiting if a third refined estimator appears or `ransac` keeps growing.
- **Suggested fix:** None now; revisit if `ransac` keeps growing.

##### [note] `ransac.run_refined` hard-codes its callers' binding signature
- **Where:** `src/slamlab/foundations/ransac.py:556-605`
- **Issue:** The generic module builds `{**data, "options", "minimal_solver", "refine"}` for the compiled call, so every refined-RANSAC binding must take exactly a `minimal_solver: int` and `refine: bool` — a convention of its two current users (E, PnP) baked into the generic layer. `ransac.run` had the same pattern, so this is consistent, but a refined estimator with a different choice argument would have to bypass the helper.
- **Suggested fix:** None now; if a third user differs, let the caller pass the full `arguments` mapping (as `backends.call` already allows).

##### [note] Python and C++ `refine_consensus` callbacks take the inlier set differently
- **Where:** `src/slamlab/foundations/ransac.py` (`refine(model, mask: NDArray[bool])`) vs `cpp/include/slamlab/foundations/ransac.hpp` (`refine(model, const Indices&)`)
- **Issue:** The generic callback receives a boolean mask in Python and an index list in C++. Both are idiomatic and parity tests cover the outcome; when Rust gains RANSAC (PR 5), pick one form so the three generic signatures line up.

##### [note] TUM depth maps: decoding lives in the demo, not the reader
- **Where:** `src/slamlab/eval/tum.py` (`DEPTH_SCALE`, `Sequence.depths` as paths), `scripts/demo_absolute_pose.py:142-152`
- **Issue:** The reader associates depth files and documents their encoding, but loading/scaling the 16-bit PNG happens in the script via cv2 (which a contract forbids in `slamlab`). Fine for a demo; milestone 2's RGB-D mode will need an in-library depth loader without cv2.
- **Suggested fix:** Backlog: decide the PNG dependency (Pillow/imageio or a minimal 16-bit PNG reader) for `eval.tum` when milestone 2 arrives.

##### [note] Feature detection/matching is reimplemented per demo script
- **Where:** `scripts/demo_absolute_pose.py:74-116` (`features`, `match`), `scripts/demo_two_view.py:48` (`matches`)
- **Issue:** Each script has its own SIFT + ratio-test front end. jscpd doesn't flag it and it is outside the library by design until the edge rung owns features (milestone 2), but a third demo should not copy it again.
- **Suggested fix:** Backlog: fold both into the edge rung's front end when it lands (or a single `scripts/` helper if a third demo comes first).

#### Checked and fine
- Dependency direction: `p3p` (fivepoint layer) imports only alignment/so3/correspondences/arrays/errors/backends; `pnp` (epipolar layer) imports only lower modules; neither touches eval/orchestration/reifyscene. C++ includes match (`p3p.cpp`: alignment, so3; `pnp.cpp`: alignment, se3, so3, nls, ransac, p3p).
- `check_layers` test updated sensibly (camera→nls is now allowed; the forbidden-include example became camera→ransac).
- Language split: C++ implements every function in both `Implementation` Protocols (`codes`, `p3p`; `codes`, `epnp`, `refine_pose`, `pnp_ransac`); Rust untouched, within its curated subset (PR 5).
- Bindings are thin: one file per module (`cpp/bindings/foundations/p3p.cpp`, `pnp.cpp`), float64 arrays in/out, `unwrap` for the error contract, no algorithm logic; `rows3`/`rows2`/`enum_code`/`matrix4` lifted into `common.hpp` replacing per-file copies.
- Error contract: `Result<…>` with InvalidInput/Degenerate in the cores; Python raises the matching `InvalidInputError`/`DegenerateError`; validation once at each public boundary (`arrays.as_correspondences` mirrored by `correspondences::check_correspondences`), with `*_reference` functions as unchecked internals.
- Nested summary: `ransac.RefinedSummary` delegates `iterations`/`termination`/`converged` to the primary run, and C++ returns the named `RefinedSolution` — matches the doc's PR 3 A11 rule; refinement problems stay with their models (`pnp.ReprojectionProblem`, `epipolar.SampsonProblem`).
- `backends.call` only locates and decodes; it knows the `codes()` binding convention but nothing about what algorithms compute. The five hand-rolled dispatches (BA, refine_essential, decompose_homography, twoview, ransac.run) now use it, with unit tests.
- Clean breaks: `EssentialSummary`, `essential_summary_from_backend`, `essential_crossing.hpp` and the unqualified `termination` key removed outright, no aliases; BA's 19 scalar option args became one strict nested dict.
- Parity fixtures (`tests/fixtures/foundations/p3p.json`, `pnp.json`, generator `generate_pose.py`) state a per-function tolerance and reason, and include RANSAC/NLS summaries.
- Compute budget: nothing new pulls heavy dependencies; cv2 stays in scripts; CPU-rung contracts unchanged.
- Design doc updated: PR 4 status, closed items (A6, A7, A7/A8 from PR 2), new open items (rank-deficient H crash, IPPE).
- Ladder placement: absolute pose belongs on rung 0 per the ladder table ("RANSAC, triangulation, PnP"); the demo and note sit in `scripts/` and `docs/notes/` like prior slices.

### Algorithm review

**Scope:** PR 4 (`feature/absolute-pose`), `a0709ca...4a4d078`. Read in full: `src/slamlab/foundations/p3p.py`, `src/slamlab/foundations/pnp.py`, the `ransac.py` additions (`refine_consensus`, `estimate_refined`) and existing LO loop, the C++ `p3p.cpp`/`pnp.cpp` (for parity of the branches I flag), `docs/notes/absolute-pose.md`, `tests/foundations/test_p3p.py`, `test_pnp.py`, the fixture headers, and the evaluation path of `scripts/demo_absolute_pose.py`. Checked against Hartley & Zisserman, Lepetit et al. (IJCV 2009), Ding et al. (CVPR 2023), Persson & Nordberg (ECCV 2018), and the PoseLib and OpenCV sources. Ran experiments (scratch scripts outside the repo) on HEAD 4a4d078 with the C++ backend and the Python reference.

**Summary:** The mathematics is correct: I re-derived Ding's two conics and both line–conic eliminations, Lambda Twist's eigenvector formula, and the pencil-cubic cofactor identity by hand and found no errors; EPnP's linearisations match OpenCV, the right-perturbation Jacobian is the standard form, and the fixtures state their tolerance policy. Nothing blocks the slice. Three should-fix findings, all evidenced: (1) Ding's "single real root → skip the second line" shortcut (inherited from PoseLib) is the main source of gross P3P misses near the danger cylinder; (2) PnP RANSAC on planar scenes gets no local optimisation at all and lands at ~2x the noise floor (p90 8x), while the docstring says planar scenes are handled; (3) the note's "every solver loses 18% far away" is a measurement artefact, not a solver loss.

#### Findings

##### [should-fix] Ding's single-root early exit drops true poses near the danger cylinder
- **Where:** `src/slamlab/foundations/p3p.py:733` (`if poses and single: break`); `cpp/src/foundations/p3p.cpp:497`.
- **Issue:** The skip is exact in theory (with one real cubic root, the two real intersections lie on one line). In floating point the `single` flag is unreliable exactly where it matters: near the cylinder the cubic has two nearly equal roots and the sign of Δ is decided by rounding; and the line that carries the complex pair has a slightly negative quadratic discriminant, which `REAL_TOLERANCE` accepts as a double root, yielding a near-tangent candidate whose distance residual is tiny and passes `DISTANCE_TOLERANCE`. If that line comes first (column/row order is arbitrary), `poses` is non-empty, the loop breaks, and the line holding the true solutions is never intersected. PoseLib does the same (`if (n_sols > 0 && G) break;` in `PoseLib/solvers/p3p.cc`), so this is inherited, not introduced.
- **Why it matters:** These are gross misses (closest solution error ~1, not a precision loss), and they make up most of the misses the note's "Near degeneracy" table attributes to the method — the stated reason Ding is the default. Removing the skip costs one extra quadratic per sample.
- **Evidence:** Python reference, `P.cylinder` from `p3p_degeneracy_probe.py`, 2000 samples; gross = closest-pose error > 1e-3, precision = 1e-6..1e-3; "no break" = `cubic_root` patched to always return `single=False`:

  | margin (negative = outside) | with break: gross, precision | no break: gross, precision |
  |---|---|---|
  | +1e-4 | 5, 30 | 4, 31 |
  | +1e-6 | 67, 76 | 36, 80 |
  | −1e-6 | 64, 59 | 32, 65 |
  | −1e-7 | 244, 86 | 39, 92 |
  | −1e-8 | 485, 87 | 39, 94 |

  At +1e-6, total misses fall from 143 to 116 (PoseLib's own count in the note: 133). C++ backend shows the same just outside (−1e-8: Ding 583 misses, PoseLib 557). The PR's probe only samples inside the cylinder (margin > 0), so the worst region (just outside) is not measured or tested.
- **Suggested fix:** Always intersect both lines (as Lambda Twist already does here), or skip the second line only when the first line's quadratic had a clearly positive discriminant; list it as a fifth departure from PoseLib. Extend the probe and the fixture to negative margins (outside the cylinder) and re-measure the table.

##### [should-fix] PnP RANSAC has no local optimisation on planar scenes and stays ~2x off the floor
- **Where:** `src/slamlab/foundations/pnp.py` `PoseEstimator.fit_nonminimal` (returns `[]` when `_epnp` refuses planar points); module docstring: "P3P and the refinement below handle planar scenes". Same in `cpp/src/foundations/pnp.cpp`.
- **Issue:** On planar points EPnP returns nothing, so every LO refit in `_least_squares`/`_local_optimization` is a no-op: the result is a single minimal sample's consensus plus one refinement on that consensus, with no inlier re-collection after refinement.
- **Why it matters:** Planar structure (walls, floors, tabletops) is common in SLAM, and the rung's tracker will use this function. No test exercises `pnp_ransac` on planar points (the only planar test is EPnP's refusal).
- **Evidence:** Same synthetic setup as the note (200 points, 1 px at f=520, 40% outliers, 2 px threshold, 40 seeds, C++ backend), rotation error / floor (floor = refinement from the truth on the true inliers):

  | Scene | `pnp_ransac` median | p90 | + one more `refine_pose` on re-collected inliers (median, p90) |
  |---|---|---|---|
  | planar (z=0) | 2.02 | 8.41 | 1.48, 3.66 |
  | non-planar (thickness 0.5) | 1.12 | 1.71 | 1.23, 1.79 |

  Example seed 0 (planar): pnp_ransac 2.24°, refining again from its result on its inliers 1.35°, floor 0.60°. (EPnP itself is good on near-planar points: at thickness 1e-3 its median is 0.62° vs OpenCV EPnP 1.24°, p90 56°; it refuses only around 1e-7, which is correct.)
- **Suggested fix:** Use the reprojection refinement as LO's non-minimal fit when EPnP refuses (or always), as PoseLib's `AbsolutePoseEstimator::refine_model` does for LO (nonlinear bundle adjustment, truncated loss, scale = threshold, 25 iterations; Lebeda, Matas & Chum, "Fixing the Locally Optimized RANSAC", BMVC 2012). A cheaper alternative: iterate refine → re-collect inliers → refine until the inlier set stops changing. Add a planar `pnp_ransac` test with an error bound, and qualify the docstring claim.

##### [should-fix] The "far away, every solver loses about 18%" result is an absolute-tolerance artefact
- **Where:** `docs/notes/absolute-pose.md` ("Far away (a 1 m triangle at 1000 m) every solver loses about 18%, to the conditioning, not the method"); `p3p_degeneracy_probe.py` (miss = max-abs 4x4 error > 1e-6).
- **Issue:** At 1000 m the translation is ~1000, so an absolute 1e-6 is a relative 1e-9, below what the problem's conditioning allows; the solvers do not lose the pose.
- **Evidence:** 2000 samples at 1000 m (C++): Ding 363 misses by the absolute 1e-6 test, but only 3 by a relative test (rotation max-abs and translation error / distance > 1e-6), 1 above 1e-3; Lambda Twist 363 vs 3 (3 above 1e-3, 2 samples with no solution).
- **Suggested fix:** Measure translation relative to the camera distance; restate the far-field result as "≈0.1% gross misses; precision limited by conditioning".

##### [note] The refinement's cost ignores cheirality inside LM
- **Where:** `pnp.py` `ReprojectionProblem._projection` (and C++).
- A point can cross z = 0 during LM and still project near its observation (x/z with both negative), and the cost does not penalise it. In practice this is safe: the start is near the optimum, a non-finite cost is rejected by `nls`, and `refine_consensus` rescores with infinite residuals behind the camera, so such a pose would be discarded. Worth one sentence in the docstring.

##### [note] `REAL_TOLERANCE` produces spurious near-tangent poses
- **Where:** `p3p.py` `quadratic_roots`.
- Accepting a slightly negative discriminant as a double root means a complex pair close to the real axis yields an extra "pose" that passes the distance check. Harmless on its own (RANSAC scores and discards it), but it is the mechanism behind the first finding and explains some of the larger solution counts seen without the break.

##### [note] State of the art for the non-minimal solver and LO
- SQPnP (G. Terzakis and M. Lourakis, "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem", ECCV 2020, https://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf) is globally optimal for any n, handles planar and non-planar in one solver, and is in OpenCV (`SOLVEPNP_SQPNP`); it would be a better least-squares refit than EPnP plus a planar fallback if the rung ever wants one.
- EPnP is the right classic choice for a learning rung; Ding et al. 2023 is the current P3P reference, so the default is current.
- PoseLib's LO refits by nonlinear refinement rather than an algebraic solver (see the planar finding) — context, not a defect.

#### Checked and fine
- Ding conics: `first`/`second` = x² − 2m₀₁xy + (1−a)y² + 2am₁₂y − a and x² − 2m₀₂x + 1 − b(y² − 2m₁₂y + 1); both eliminations in `_ding_depths` re-derived by hand; reordering makes a₁₂ the longest side in all three branches.
- Lambda Twist: `_twist_depths` scale equations, `_eigen_known_zero` eigenvector formula (first two rows of (M − σI)e = 0), the corrected p₀ sign.
- `pencil_cubic`: cofactor-column construction and det(A + sB) = det A + s⟨B, cof A⟩ + s²⟨A, cof B⟩ + s³ det B (valid for any 3x3).
- `cubic_root`: Cardano/Viète, the Viète argument (3q/2p)√(−3/p) and largest root; Newton polish with accept-if-decreasing and a cap.
- `_scaled` guards the τ² − 2bτ + 1 denominator; the coinciding-bearings regression test is real-data-captured.
- Pose from triangles: rigid alignment (every returned pose is a rotation); distance-residual rejection relative to the largest squared distance; collinear refusal.
- EPnP vs OpenCV `epnp.cpp`: control points (√(λ/n) scaling), null-space vectors smallest-first, L₆ₓ₁₀ monomial order, the three β approximations including sign rules, GN over all four β. The 5-point minimum is justified (null space is 2-D at n = 5, so the N = 2 linearisation is exact). Planar → `DegenerateError`. Deterministic eigenvector signs for backend parity.
- Refinement: right perturbation T·exp(δ), (ρ, θ) order, Jacobian ∂π/∂p · [R | −R[X]ₓ] (finite-difference test present); Cauchy scale = threshold, consistently compared with θ² on squared norms; iteration cap with NLS summary exposed; input validation including points in front of the start.
- RANSAC: MSAC with infinite residual behind the camera; seeded PCG32; `refine_consensus` keeps a refinement only if the MSAC cost drops and reports it in `RefinedSummary`; `DegenerateError` when no sample gives a model. Failure is observable throughout.
- Tests/fixtures: fixtures built in 50-digit mpmath with stated `comparison`/`rtol`/`atol`/`reason`, including 1e-6 inside the cylinder; oracles (PoseLib solution counts, OpenCV EPnP, SciPy with matched per-correspondence loss); backend parity at 1e-10.
- TUM evaluation: per-frame rotation and camera-centre errors against mocap; depth points metric; two-view points scaled by the true baseline (appropriate for a monocular initialiser, stated in the note).

Sources:
- [PoseLib p3p.cc (Ding solver's single-root break)](https://raw.githubusercontent.com/PoseLib/PoseLib/master/PoseLib/solvers/p3p.cc)
- [PoseLib absolute_pose.cc (LO refit by bundle adjustment)](https://raw.githubusercontent.com/PoseLib/PoseLib/master/PoseLib/robust/estimators/absolute_pose.cc)
- [Terzakis & Lourakis, SQPnP, ECCV 2020](https://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf)
- Lebeda, Matas & Chum, "Fixing the Locally Optimized RANSAC", BMVC 2012
