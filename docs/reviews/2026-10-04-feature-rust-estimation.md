# Review panel: `feature/rust-estimation`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-04

| | |
|---|---|
| **Branch** | `feature/rust-estimation` |
| **Range** | `b66269c...57c9d8d` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`; 1,778 tests, none skipped) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L1 | must-fix | algorithm | `alignment.rs:82`, every `Matrix3::svd` | nalgebra's fixed 3x3 SVD works on `MᵀM` (squares κ): Rust P3P/Umeyama give wrong poses on thin triangles | fix (one backward-stable SVD in `linalg`) | fixed (b0542af; cost recovered in 8f25432, b0f0b2f): every SVD through faer, verified on the probes first -- decided by evan-wes, 2026-10-04 |
| L2 | must-fix | algorithm | `linalg.rs:31-36` | nalgebra's dynamic SVD is not backward stable on graded matrices: Rust 8-point E/F off by up to 6e-3 | fix (same helper) | fixed (b0542af): same faer helper; ill_conditioned.json guards it -- decided by evan-wes, 2026-10-04 |
| L3 | should-fix | algorithm | `fivepoint.rs:800,861`; `rust-port.md:20` | Five-point solutions come back in different orders per backend; lock-step rests on no near-ties | fix the note; defer a canonical order (design doc) | fixed (32975f3): one canonical order in Python, C++ and Rust -- decided by evan-wes, 2026-10-04 |
| A1 | should-fix | architecture | `test_{fivepoint,p3p,pnp,twoview,epipolar}.py` | Compiled-side tests still pinned to `cpp` after the lock-step flip | fix | fixed (c28a955; test_backends_agree in 32975f3) -- decided by evan-wes, 2026-10-04 |
| A2 | should-fix | architecture | `ransac.rs:670`, refiners in both cores | The infallible refiner unwraps a fallible refine: three failure modes if it ever fails | fix (fallible refiner, failed round not kept) | fixed (6020220): the refiner may fail in every language; a failed round is not kept -- decided by evan-wes, 2026-10-04 |
| H1 | should-fix | hygiene | `epipolar.rs:520-535` | Public `start_motion` panics on unchecked input | fix (return `Result`) | fixed (6020220): start_motion returns a Result -- decided by evan-wes, 2026-10-04 |
| H2 | should-fix | hygiene | `ransac.rs` `local_optimization` | "Cannot fail" comment, yet the failure is swallowed with `break` | fix (`expect` with the invariant) | fixed (60c51ee) -- decided by evan-wes, 2026-10-04 |
| H3 | should-fix | hygiene | `epipolar.rs`, binding docs | Frame and convention statements thin in the new Rust docs | fix | fixed (60c51ee) -- decided by evan-wes, 2026-10-04 |
| H4 | should-fix | hygiene | `slamlab-py/src/foundations/*` | 27 copies of the same `needless_pass_by_value` allow | fix (one allow per crate) | fixed (60c51ee) -- decided by evan-wes, 2026-10-04 |
| H5 | should-fix | hygiene | `common.rs` | `as_slice().map_err` written four times; `slice`/`slice2` twins | fix | fixed (60c51ee) -- decided by evan-wes, 2026-10-04 |
| H6 | should-fix | hygiene | `correspondences.rs:~141` | Tolerance literal in an error message | fix | fixed (60c51ee): literal kept to match Python's message, tied to the constant by a compile-time assertion in each core -- decided by evan-wes, 2026-10-04 |
| H7 | should-fix | hygiene | new Rust modules | Native Rust tests light; CHANGELOG overstates them | fix (focused tests) | fixed (60c51ee, 9caa763): focused native tests; CHANGELOG wording matches -- decided by evan-wes, 2026-10-04 |
| L4 | note | algorithm | `p3p.rs:94`; `rust-port.md:82` | numpy's `argmax` is row-major, not column-major as the note says | fix the wording | fixed (60c51ee, 9caa763) -- decided by evan-wes, 2026-10-04 |
| L5 | note | algorithm | `epipolar.rs:782-791` | The refiner's justification checks a different E from the start | fix (subsumed by A2) | fixed (6020220, with A2) -- decided by evan-wes, 2026-10-04 |
| L6 | note | algorithm | `cpp/src/foundations/p3p.cpp:199-203` | C++ depth polish stops earlier than Python/Rust (relative LU threshold) | defer (design doc) | deferred (design doc, PR 6's panel; 9caa763) -- decided by evan-wes, 2026-10-04 |
| L7 | note | algorithm | `fivepoint.rs:826-834` | A repeated eigenvalue in Stewénius gives one eigenvector | fix (one doc sentence) | fixed (8f25432): the eigendecomposition that replaced the per-eigenvalue SVDs returns independent eigenvectors -- decided by evan-wes, 2026-10-04 |
| A3 | note | architecture | `ba.hpp:238` | Every `ba` header includes the facade that declares `solve` | fix the design-doc wording; stays deferred | fixed the wording (9caa763); stays deferred in the design doc -- decided by evan-wes, 2026-10-04 |
| A4 | note | architecture | `linalg.rs`; `pyproject.toml:135` | Rust's `linalg` has no place in the layer contracts | fix (optional layer) | fixed (9caa763): optional `(linalg)` layer -- decided by evan-wes, 2026-10-04 |
| A5 | note | architecture | `ba/mod.rs:21`, `ransac.rs:610` | Rust's public paths differ from C++ by design | waive: recorded (A5, A13/A24) | waived: recorded in the design doc (A5, A13/A24) -- decided by evan-wes, 2026-10-04 |
| H8 | note | hygiene | `ransac.rs` | Three ways to count inliers | fix (one helper) | fixed (60c51ee) -- decided by evan-wes, 2026-10-04 |
| H9 | note | hygiene | several | Small duplication and shadowing | fix | fixed in part (60c51ee): refine dict shared, read_options, map/transpose; the per-module POLISH_TOLERANCE mirror Python's, and `self::refine` keeps the Python parameter name -- decided by evan-wes, 2026-10-04 |
| H10 | note | hygiene | `backends.py:31` | `LOCKSTEP_BACKENDS` now equals `BACKENDS` | waive: kept for later rungs (design doc) | waived: kept for later rungs (design doc) -- decided by evan-wes, 2026-10-04 |
| H11 | note | hygiene | `rust-port.md` | One overlong line | fix | fixed (9caa763) -- decided by evan-wes, 2026-10-04 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `git diff b66269c...57c9d8d` (55 files), read in context. Read in full or nearly: `rust/slamlab-py/src/common.rs`, `ransac_crossing.rs`, the epipolar and pnp bindings, and the twoview binding options. In the Rust core: `ransac.rs`, `linalg.rs`, `nls::solve_cauchy`, and most of `epipolar.rs` and `pnp.rs`. Skimmed: `twoview.rs` and the constants in `correspondences.rs` and `fivepoint.rs`. Read in full: the Python, C++ and script diffs, CHANGELOG, `rust-port.md`, README and VERSION. `fivepoint.rs`, `p3p.rs`, `homography.rs`, `triangulation.rs` and the benches were not read line by line, only grepped for `unwrap`/`expect`/`allow`, magic numbers and function length.

**Summary:** This is production quality overall. Every item is documented in the `# Arguments`/`# Returns`/`# Errors` style of PR 5. Tolerances are named and justified. Inputs are validated at the public boundaries, and the `expect`s in library code are each documented invariants. Nothing is must-fix. The findings are:
- binding boilerplate;
- a few documentation gaps on conventions;
- one contradictory "cannot fail, so ignore it" spot in `ransac.rs`;
- a public panicking function;
- native Rust tests that are thin for the volume of new code.

#### Findings

##### [should-fix] `start_motion` is a public function that panics on valid-looking input
- **Where:** `rust/slamlab-foundations/src/epipolar.rs:520-535`. A twin is at `rust/slamlab-foundations/src/pnp.rs:~448`: `pnp_ransac`'s doc has a `# Panics` section that reads "Never".
- **Issue:** `pub fn start_motion(&Matrix3)` calls `.expect("every caller has checked E finite and of rank 2")`. The crate is meant to run standalone, without Python's checks, so this is a public API whose caller must have validated first. CLAUDE.md says to validate at public boundaries and not to `expect` fallible input in library code.
- **Why it matters:** A standalone caller can crash `start_motion` with a non-finite E, because it takes unchecked input. `pnp_ransac`'s `# Panics: Never` misuses the heading: the `expect` is inside a closure, and clippy does not require the section there.
- **Evidence:** `decompose_essential(essential).expect("every caller has checked E finite and of rank 2")`
- **Suggested fix:** Make `start_motion` `pub(crate)`, or have it return `Result` or `Option`. Drop the "Never" `# Panics` section from `pnp_ransac`; the invariant comment already explains the `expect`.

##### [should-fix] RANSAC local optimisation says an error "cannot fail", then swallows it silently
- **Where:** `rust/slamlab-foundations/src/ransac.rs`, in `local_optimization`, around the `// The draw is at most half the inliers, so it cannot fail.` comment. Also `count()`, which does `usize::try_from(value).unwrap_or(0)`.
- **Issue:** The comment claims an invariant, but the code handles the failure with `else { break; }` for both `u32::try_from` and `rng.sample`. `estimate` handles the same `rng.sample` failure with `?`. If the invariant broke, local optimisation would stop silently and the run would look normal. This is the inconsistency and the swallowed error the rubric asks about.
- **Why it matters:** There are now two policies for one condition. A future change to the draw size would degrade quality with no signal.
- **Suggested fix:** Pick one policy. Either propagate the error (`local_optimization` returns `Result`), or `expect` with the invariant message, as the rest of the crate does for documented invariants. Do the same for `count`, or state that its clamp to 0 is unreachable after `check_options`.

##### [should-fix] Frame and convention statements are thin in the new Rust docs and bindings
- **Where:**
  - `rust/slamlab-foundations/src/epipolar.rs:195-232` (`essential_8pt`, `fundamental_8pt`, `sampson_*`);
  - `rust/slamlab-py/src/foundations/epipolar.rs:15-170`;
  - `rust/slamlab-py/src/foundations/pnp.rs:~15-70`.
- **Issue:** The core module doc defers the maths to the Python docstring, which is acceptable. But the per-function docs say only "Estimates E" and "E.". They do not give the direction convention (`x2ᵀ E x1 = 0`, with 1 the first camera) or the candidate-pose convention. The binding docs give shapes such as "(N, 3) bearings in cameras 1 and 2", but not units or frames. For example, `decompose_essential` says "A new (4, 4, 4) array" without saying which map (`T_2_1`?), and `epnp` says "(N, 3) world points".
- **Why it matters:** CLAUDE.md says "State shapes, units and frame conventions". These are the functions where a transpose or an inverse is easiest to get wrong.
- **Suggested fix:** Add one line to each:
  - "`x2ᵀ E x1 = 0`, bearings 1 then 2";
  - the pose a candidate maps (`T_cam2_cam1`, or whichever it is, with t's scale);
  - "world frame, metres or arbitrary units".

##### [should-fix] 27 copies of the same `#[allow(clippy::needless_pass_by_value, reason = ...)]` in the bindings
- **Where:** `rust/slamlab-py/src/foundations/` (`epipolar.rs` 8, `homography.rs` 4, `pnp.rs` 3, `camera.rs` 3, `triangulation.rs` 2, and others), plus `common.rs`.
- **Issue:** The same four-line attribute sits on nearly every `#[pyfunction]`. It repeats one justification and adds about 100 lines of noise. A new binding that forgets it fails clippy, which is friction, not a safeguard.
- **Suggested fix:** Put one `#![allow(clippy::needless_pass_by_value, reason = "...")]` at the top of each bindings module, or on the `foundations` module, giving the PyO3 reason once. Alternatively, set it in `[lints]` for the `slamlab-py` crate only.

##### [should-fix] Repeated `as_slice().map_err(...)` and a needless `slice`/`slice2` pair
- **Where:** `rust/slamlab-py/src/common.rs`: `batch`, `slice`, `slice2` and `one` each carry `.as_slice().map_err(|e| PyValueError::new_err(e.to_string()))`.
- **Issue:** `slice` and `slice2` have identical bodies and differ only in the array's dimension type. The same conversion is written four times.
- **Suggested fix:** One generic helper, for example `fn contiguous<'a, T: Element, D: Dimension>(array: &'a PyReadonlyArray<'_, T, D>) -> PyResult<&'a [T]>`. Use it in all four, and drop `slice2` if the callers allow.

##### [should-fix] Hard-coded tolerance literal in an error message
- **Where:** `rust/slamlab-foundations/src/correspondences.rs:~141`.
- **Issue:** The message reads `format!("{name}: expected unit vectors within 1e-09")`, but the test uses `BEARING_TOLERANCE`. If the constant changes, the message becomes wrong.
- **Suggested fix:** Interpolate `{BEARING_TOLERANCE:e}`, and include the offending norm, as in "errors name the bad value".

##### [should-fix] Native Rust tests are light for the amount of new code, and some public items have none natively
- **Where:**
  - `rust/slamlab-foundations/src/linalg.rs`: 1 test; neither `least_squares` nor the iteration cap is exercised.
  - `homography.rs`, `twoview.rs`, `p3p.rs`: 1 test each, happy path only.
  - `fivepoint.rs`, `epipolar.rs`: 2 each.
- **Issue:** CHANGELOG and `rust-port.md` say "Native Rust tests for every new module, mirroring the native C++ ones". The behaviour is covered, but through the Python lock-step tests, which exercise the cdylib. The native tests cover the main case and, in places, one bad input. The crate is meant to run standalone, yet no native test covers these edge cases of the new modules:
  - the epipole match's `InvalidInput`;
  - `refine` with an infinite start;
  - `refine_consensus`'s exit paths with a failing refiner;
  - degenerate samples;
  - the SVD returning `None` on non-convergence.
- **Suggested fix:** Add a handful of focused native tests:
  - `least_squares` against a known rank-deficient solution;
  - `refine` and `refine_essential` on the epipole match, returning `InvalidInput`;
  - `estimate` with every sample degenerate, returning `Degenerate`.

  At minimum, soften the CHANGELOG sentence to match.

##### [note] Three ways to count or select inliers in `ransac.rs`
- **Where:** `rust/slamlab-foundations/src/ransac.rs`: `below`, `mask`, and the inline `.filter(|r| **r < threshold2).count()` (in `estimate`, twice, and in `refine_consensus`).
- **Issue:** The `r < threshold2` rule is written several ways. Parity depends on the exact comparison (NaN fails it), so one helper would keep it consistent.
- **Suggested fix:** An `inlier_count(&residuals, t2)` helper, or `below(...).len()`.

##### [note] Smaller duplication and shadowing
- **Where:**
  - `rust/slamlab-foundations/src/{fivepoint,pnp,p3p}.rs`: three separate `POLISH_TOLERANCE = 1e-14` constants.
  - `rust/slamlab-py/src/foundations/{epipolar,pnp}.rs`: `refine_essential` and `refine_pose` build the same `{state, summary}` dict by hand.
  - `ransac_crossing.rs::options`: the parameter shadows the function name, and `refined_summary` uses a `match` where a `.map` (transposed) would do.
  - `rust/slamlab-foundations/src/pnp.rs`: `pnp_ransac`'s `refine: bool` argument forces `self::refine(...)` inside its closure.
- **Why it matters:** None is a defect; each is a small consistency cost.
- **Suggested fix:** Optional. A shared `refine_dict(py, key, item, summary)` helper in `ransac_crossing` or `nls_crossing`. Rename the flag, or the closure's call, to avoid the `self::` path.

##### [note] `LOCKSTEP_BACKENDS` is now the same as `BACKENDS`
- **Where:** `src/slamlab/backends.py:31`.
- **Issue:** `frozenset(BACKENDS)` is correct and documented, but the constant no longer distinguishes anything. It is kept so tests and the future edge rung can split the backends again, as the design doc says (lines 256-258).
- **Why it matters:** Not a defect. Worth removing if the edge rung never uses it.

##### [note] Doc formatting
- **Where:** `docs/notes/rust-port.md`, the "Options cross through serde" paragraph.
- **Issue:** One line runs well past the file's 100-column wrap since `ransac.Options` and `twoview.Options` were added.
- **Suggested fix:** Rewrap.

#### Checked and fine
- **Rust doc style:** every new item has `# Arguments`/`# Returns`/`# Errors`, and the private-item docs are consistent with PR 5. `#[must_use]` is applied appropriately.
- **`unwrap()`/`expect()`:** every hit in the diff is in `#[cfg(test)]` code, benches, or the documented-invariant `expect`s (`DenseSystem::build` row counts, `decompose_essential`, the refiners).
- **Tolerances:** each is named with a one-line justification (`RANK_TOLERANCE`, `ESSENTIAL_TOLERANCE`, `PLANAR_TOLERANCE`, `GRIC_LAMBDA3`, and others). The `Cargo.toml` lint relaxations (`neg_cmp_op_on_partial_ord`, `many_single_char_names`) carry reasons.
- **Input validation:** `check_options` in `ransac.rs` and `twoview.rs`, `check_bearing_pairs` and `check_point_pairs`, and `refine_pose`'s pose, scale and cheirality checks all name the bad setting and value. The serde options use `deny_unknown_fields`.
- **The C++ fix:**
  - `epipolar::refine` and `pnp::refine` return `Result`, and the headers' `@return` text is updated.
  - The `.value()` calls in the RANSAC refiners carry an explanatory comment.
  - The new test asserts the exact message "initial cost is not finite" on every lock-step backend.
  - `pnp::refine`'s unreachable case is covered through `refine_pose`.
- **Fold-ins:** A6, A5, A13/A24 and A20 are applied consistently across Python, C++ and Rust. The `check_layers` facade change and its tests are coherent. The gauge's `_svd_failure_is_degenerate` context manager is a clean single mechanism, and it is tested.
- **Python:** the docstrings are updated for the indices change (ascending `intp`), and the `require_function` skip path is gone. The `backends` docstrings were checked for stale "chosen set" wording and have none; the only remaining occurrences are in `docs/reviews`, which is historical.
- **Stale text:** a grep for "chosen set", "PR 6 adds" and "Rust subset" finds only historical review reports, and the intended "Rust subset" naming for `slamlab_rs`.
- **Function sizes:** nothing in the new Rust is over about 100 lines. The longest are `scored` (82), `two_view` (73), `gric` (66) and `decompose_homography` (66), and each does one thing.
- **Duplication:** layer 1's duplication check is clean. The Rust, C++ and Python parallels are intentional and not reported.
- **CHANGELOG, README and VERSION** are consistent with the PR's contents (0.3.0.0, the milestone close).

### Architecture review

**Scope:** `git diff b66269c...57c9d8d` on `feature/rust-estimation` (9 commits, 55 files). Read:
- the Rust core modules added (`ransac.rs`, `epipolar.rs`, `pnp.rs`, `linalg.rs`, `ba/solve.rs`, `ba/mod.rs`, `lib.rs`);
- the Rust bindings (`common.rs`, `ransac_crossing.rs`, `epipolar.rs`, `twoview.rs`, `mod.rs`);
- the C++ changes (`ba/solve.cpp`, `epipolar.cpp`/`pnp.cpp` refine, headers, bench);
- `scripts/check_layers.py`, the `pyproject.toml` contracts, `backends.py`, `conftest.py`, `test_backends.py`, and how each `tests/foundations/test_*.py` uses the backends;
- the design doc's "Languages", crossing and backlog sections.

One read-only probe compared Rust's and C++'s `codes()` tables. Importing `slamlab` in that probe triggered scikit-build's editable rebuild into `build/`; no tracked file changed.

**Summary:** The PR is structurally sound:
- every new Rust module mirrors its Python and C++ counterpart on the right layer of rung 0;
- the bindings are thin type conversion, and the crossing formats match the C++ ones;
- the BA facade move and the A5/A6 decisions are consistent across the three languages, and documented where they diverge.

The lock-step flip is incomplete. Several compiled-side tests are still pinned to C++, so the doc's claim that every module's tests run on all three backends is not yet true. And the RANSAC refiners' new unwrap (`.value()` in C++, `.expect` in Rust) leaves three different failure modes on a path argued to be unreachable but not proven so.

#### Findings

##### [should-fix] The lock-step flip left compiled-side tests pinned to C++
- **Where:**
  - `tests/foundations/test_fivepoint.py:78` (`test_backends_agree`) and `:172` (`test_the_compiled_solver_codes_match`);
  - `tests/foundations/test_p3p.py:76` and `:206`;
  - `tests/foundations/test_pnp.py:290` (`test_backends_agree_on_ransac`) and `:321` (`test_the_compiled_codes_match`);
  - `tests/foundations/test_twoview.py:239`;
  - `tests/foundations/test_epipolar.py:416`.

  The claim they contradict is at `docs/design/2026-10-02-slamlab-ladder.md:252-256`.
- **Issue:** Each of these calls `require_function("cpp", ...)` directly instead of parametrising over `backends.COMPILED_BACKENDS` or `LOCKSTEP_BACKENDS`. Two contracts are therefore never tested for Rust:
  - the code tables (`codes()`) for `fivepoint`, `p3p`, `pnp`, `twoview` and `epipolar`. Choices such as `minimal_solver` cross as integer codes, so a reordered Rust enum would silently pick the wrong solver;
  - the run-for-run "draws the same samples" agreement for PnP RANSAC, and the five-point and P3P solution sets.

  The design doc now says "every module's tests run on all three backends … so a compiled module that falls behind its Python reference fails layer 1". For these tests that is false.
- **Why it matters:** These checks guard the seam the flip is about. They were C++-only while Rust was a chosen set; now that Rust is lock-step they should cover it, or the next Rust enum change slips through. The parity fixtures cover returned values, but not the code tables Python sends.
- **Evidence:** `chosen = require_function("cpp", fivepoint.__name__, "essential_5pt")`, with the same pattern at each line above. I compared the tables by hand and they match today (`fivepoint True`, `p3p True`, `pnp True`, `twoview True`, `epipolar True`), so this is a missing guard, not a current bug. `test_ba.py:600` and `test_twoview.py:155` show the intended pattern: they are parametrised over the backends.
- **Suggested fix:** Parametrise these tests over `backends.COMPILED_BACKENDS` (or `LOCKSTEP_BACKENDS` minus Python). Then add a structural check to `tests/test_backends.py`: for each compiled backend, the set of slamlab modules with an `Implementation` Protocol must equal the set of extension submodules. That makes "every backend has every module" one assertion, instead of a property spread across each module's tests (which the current docstring of `test_every_compiled_module_matches_its_protocol` relies on).

##### [should-fix] The infallible refiner turns an unexpected refinement failure into three different failures
- **Where:**
  - `rust/slamlab-foundations/src/ransac.rs:670` (`Refiner` returns `nls::Solution`, not `Result`) and `:645`;
  - `rust/slamlab-foundations/src/epipolar.rs:790` (`.expect("the inliers' Sampson distances are finite")`) and `rust/slamlab-foundations/src/pnp.rs:637`;
  - `cpp/src/foundations/epipolar.cpp:489-491` and `cpp/src/foundations/pnp.cpp:430-432` (`.value()`);
  - `cpp/include/slamlab/foundations/ransac.hpp:370,399`.
- **Issue:** The C++ segfault fix made `refine` fallible, but the refinement callback in both cores still returns a bare solution, so each refiner has to unwrap. If the invariant ever fails, the outcome differs by language:
  - Python's reference propagates `InvalidInputError`;
  - C++ throws `std::bad_expected_access`, which nanobind surfaces as a generic exception. This contradicts the design doc's rule that nothing throws on the cores' hot path;
  - Rust panics, which PyO3 surfaces as `PanicException`: a `BaseException`, not a `slamlab.foundations.errors` type.

  For PnP the invariant holds exactly: the refinement starts from the very model whose `reprojection` produced the inlier mask. For E it holds only approximately. `refine` starts from `start_motion(E)` re-composed as `[t]x R`, which equals the RANSAC model only when that model lies exactly on the essential manifold. Finite Sampson distances under E do not imply finite ones under the re-composed E.
- **Why it matters:** This breaks the shared-error-contract rule ("every language fails the same way") on a path whose safety is argued in a comment rather than enforced by types. The `# Panics: Never` doc on `essential_ransac` overstates it. Whether the edge case is reachable in floating point is for the algorithm reviewer.
- **Evidence:** `pub type Refiner<'a, Model> = &'a dyn Fn(&Model, &[usize]) -> nls::Solution<Model>;` and `.expect("the inliers' Sampson distances are finite")`. Rust's refinement starts at `start_motion(essential)`, not at `essential` (`epipolar.rs`, `refine`).
- **Suggested fix:** Make the refiner fallible in all three languages: `Fn(&Model, &[usize]) -> Result<nls::Solution<Model>>` in Rust, a `Result`-returning invocable in C++, and the matching Python Callable. In `refine_consensus`, treat a failed round as "not kept" (break, as for a round that does not lower the cost), and record it in the summary. This removes both unwraps, leaves one failure mode, and fits the signature change A6 already made in all three languages. If the panel prefers to keep the invariant, at least have C++ and Rust map the failure to a typed `Degenerate`/`InvalidInput` instead of throwing or panicking.

##### [note] C++'s `ba.hpp` still declares `solve`, and every `ba` module includes it
- **Where:** `cpp/include/slamlab/foundations/ba.hpp:238`, included by `cpp/src/foundations/ba/{bundle,layout,schur,gauge}.hpp`.
- **Issue:** This is the gap the PR records in the design doc's backlog (A24/A13). The exposure is wider than "a package module that includes the header": every internal `ba` header already includes it, so `bundle` or `layout` could call `solve` without `check_layers` seeing an upward edge. The Rust side is closed: `ba::solve::solve`, with no re-export.
- **Suggested fix:** When the deferred facade work happens, move the declaration to a public `slamlab/foundations/ba/solve.hpp`, mirroring Python's `ba/solve.py` and Rust's `ba/solve.rs`, and map that header to the `solve` layer. Recording it as it stands is acceptable for this PR.

##### [note] Rust's `linalg` has no place in the layer contracts
- **Where:** `rust/slamlab-foundations/src/linalg.rs`; `pyproject.toml:135` ("Rung 0's internal order").
- **Issue:** `linalg` is a Rust-only module, used by `correspondences`, `triangulation` and `ba/gauge`. No contract lists it, so `check_layers` constrains it only through chains. Its intended position (below `correspondences`, depending on nothing in the crate) is implicit.
- **Suggested fix:** Add it as an optional layer, for example `"(linalg)"` beside `arrays`. import-linter accepts an optional layer that Python lacks, so its position is declared rather than inferred. Alternatively, state in `check_layers.py`'s docstring that core-only helper modules are deliberately unlisted.

##### [note] The Rust public paths for BA's solve and the refined solution differ from C++ by design
- **Where:** `rust/slamlab-foundations/src/ba/mod.rs:21` (`pub mod solve;`), `ransac.rs:610`; design doc, "Where the summary lives".
- **Issue:** Each language reaches BA's solve by a different path, and the refined solution has a different shape in Rust:
  - Python calls `ba.solve(...)`, which the package re-exports;
  - C++ calls `ba::solve(...)`;
  - Rust calls `ba::solve::solve(...)`, because `check_layers` forbids re-exports in `mod.rs`;
  - Rust's `RefinedSolution` is a `solve::Solution` alias, where C++ has a named struct.

  Both differences are recorded (A5, A13/A24) and coherent. A standalone Rust caller just sees a different path from the other two languages. This needs no change; the later batched/standalone API decision (A11) should cover it.

#### Checked and fine
- **Dependency direction:** the new Rust modules' `crate::` edges follow rung 0's contract order (`twoview` → `epipolar`/`homography`/`triangulation`/`ransac` → `correspondences` → `se3`/`so3`). `ba/solve.rs` uses only facade (`problem`-layer) items plus `bundle`/`layout`, as the BA package contract requires. There are no new dependencies on higher rungs.
- **The BA facade move:** `solve`, `centring_origin` and `shifted` live in `ba/solve.rs` and `ba/solve.cpp`, mirroring Python's `ba/solve.py`. `_CPP_FACADES` maps `ba` to `("problem", "problem")`, the binding calls `core::solve::solve`, and `centring_origin` is public in all three languages.
- **A5:** Rust's `ba::Summary` and `ransac::RefinedSummary` implement `solve::Summary` by delegating to the primary run, matching Python's properties. C++ keeps named structs. The design doc records the split, and `Solution` stays a named struct for BA everywhere.
- **A6:** the refiner takes ascending inlier indices in Python (`np.flatnonzero`), C++ (`Indices`) and Rust (`&[usize]`). The Python test asserts dtype and order.
- **A20:** the Python gauge maps `LinAlgError` to `DegenerateError`. The Rust capped SVD moved from `ba/gauge.rs` to the crate-level `linalg::svd`, so the crate has one SVD rather than one per module.
- **Bindings:**
  - one file per module, doing only type conversion (`rows`, `one`, `item_array`, `decode`/`by_name`), with the logic in the cores;
  - options cross as strict serde dicts (`deny_unknown_fields`), matching C++'s `check_keys`;
  - results cross as nested dicts with integer termination codes, shared through `ransac_crossing.rs` as C++ shares them through `ransac_crossing.hpp`.
- **Estimator contract:** Rust's `Estimator` trait, C++'s concept and Python's Protocol have the same five members. Rust's default `fit_minimal = fit_nonminimal` is a harmless convenience with the same semantics.
- **Generic in the core, concrete in the bindings:** generic RANSAC is not bound to Python in any core; only the concrete estimators cross.
- **`backends.py`:** `LOCKSTEP_BACKENDS = frozenset(BACKENDS)` and the removal of the chosen-set skip in `conftest.require_function` are clean breaks, with no shim. The design doc says how a later rung would bring a skip back.
- **The C++ `refine` signature change** (it now returns `Result`) is propagated to `refine_essential` and `refine_pose`, and documented in the headers. There are no other callers.
- **Benchmarks:** they stay outside the cores (`cpp/bench/`, `benches/`), build identical closed-form scenes in C++ and Rust, and print their decisions, so the timings compare like with like.
- **Dependencies:** no heavy, GPU or depth-model dependency enters rung 0 in any language. The Rust crate's dependencies are unchanged apart from lint configuration.

### Algorithm review

**Scope:** `git diff b66269c...57c9d8d` (PR 6, `feature/rust-estimation`). Read all of the non-test code in `rust/slamlab-foundations/src/{linalg,nls,correspondences,triangulation,ransac,fivepoint,epipolar,homography,twoview,p3p,pnp}.rs`, compared line by line with `cpp/src/foundations/*.cpp` and `src/slamlab/foundations/*.py`. Also read the C++ `refine` fix, the Python gauge, RANSAC and test changes, and `docs/notes/rust-port.md`. Evidence came from experiments outside the repo: scratch Rust programs against the same nalgebra 0.35.0 the crate builds with, and comparisons through the Python bindings on all three backends.

**Summary:** The port follows its references closely. Control flow, tolerances, NaN-failing comparisons and first-max tie-breaks all match the C++ core, and P3P, EPnP, triangulation and RANSAC take the same decisions wherever I tested them. The weak point is the linear algebra under the port, not its logic: neither of nalgebra's SVDs is backward stable.
- The fixed 3x3 SVD works on `MᵀM`, which squares the condition number. Through Umeyama, it makes Rust's P3P return wrong poses on thin triangles.
- The dynamic SVD behind `linalg::svd` has errors of about 1e-4 on some graded matrices. It makes the public `essential_8pt` and `fundamental_8pt` differ from Python by up to 6e-3, where C++ agrees to 2.5e-10.

The existing fixtures exercise neither case. Separately, the five-point solvers return the same solutions in different orders on every backend, which contradicts the note's lock-step premise.

#### Findings

##### [must-fix] nalgebra's fixed 3x3 SVD squares the condition number: Rust P3P and Umeyama return wrong poses on thin triangles
- **Where:** `rust/slamlab-foundations/src/alignment.rs:82` (`cov.svd(true, true)`), which P3P's `finish` (`p3p.rs:363`) and EPnP (`pnp.rs:368`) both use. The same routine is used at:
  - `epipolar.rs:168`, `:185`, `:445` and `:737`;
  - `homography.rs:334`;
  - `fivepoint.rs:323`;
  - `so3.rs:490`.

  The premise is stated in `linalg.rs:6-7`: "Fixed 2x2 and 3x3 matrices take nalgebra's closed-form routines ... and need no helper".
- **Issue:** For `Matrix3<f64>`, nalgebra 0.35 sends `svd()` and `singular_values()` to `svd3::svd_ordered3`. That routine takes a symmetric eigen-decomposition of `MᵀM` and then a QR of `M V`, a method from GPU and graphics work (McAdams et al.). It is neither a closed form nor the Jacobi SVD that Eigen's `JacobiSVD` (the C++ core) and LAPACK (Python) compute.
  - `V`'s errors grow like ε·κ², and `U = qr(M V)` amplifies them by κ again.
  - A three-point cross-covariance has rank 2, so a thin triangle (small σ2/σ1) puts garbage in `U`'s second column, and the Umeyama rotation is wrong.
- **Why it matters:** P3P is RANSAC's minimal solver for PnP, and random triples are often thin.
  - Rust returns materially wrong poses where Python and C++ are accurate. That changes PnP RANSAC's models and decisions, and its accuracy.
  - The rank decisions `singular[1] > RANK_TOLERANCE·singular[0]` in `decompose_essential` and `decompose_homography` also differ near the threshold.
- **Evidence:**
  - **Three-point Umeyama, Rust only** (scratch program; `cov` built from 3 points with a known rotation, 2000 trials per row):
    ```
    thin 1e0:  Matrix3 svd |USVᵀ-M|/s1 2.9e-13; Umeyama |R - R_true| 1.8e-9
    thin 1e-2: Matrix3 svd |USVᵀ-M|/s1 1.8e-9;  Umeyama |R - R_true| 6.8e-2
    thin 1e-3: Matrix3 svd |USVᵀ-M|/s1 1.9e-8;  Umeyama |R - R_true| 6.7e-1
    ```
  - **The public API on all three backends** (`nalgebra-svd-probe/umeyama_three_points.py`), worst rotation error over 2000 triangles:
    ```
    thinness 0.01:  python 2.87e-09, cpp 1.14e-09, rust 4.11e-01
    thinness 0.001: python 6.34e-08, cpp 4.63e-08, rust 6.17e-01
    ```
  - **End-to-end `p3p.p3p`** (`.../p3p_thin_triangles.py`): the best pose's error against the truth, and in brackets how many of 1000 trials exceed 1e-6:
    ```
    thin 0.1  ding: python 2.5e-09 (0), cpp 1.9e-09 (0), rust 3.3e-05 (2)
    thin 0.01 ding: python 1.1e-03 (2), cpp 1.7e-03 (2), rust 3.2e-01 (87)
    thin 0.001 ding: python 1.2e-02 (20), cpp 4.1e-02 (23), rust 7.0e-01 (937)
    ```
    Lambda Twist behaves the same: 78 bad poses against 4 at 0.01, 956 against 53 at 0.001.
  - **A rank-2 E with σ = (1, σ2, 0)** under random rotations: for σ2 = 1e-8, `Matrix3::singular_values()[1]` lands in [7.1e-9, 1.0e-8], an error of up to 30%. numpy gives exactly 1.000e-8.
  - **A generic 3x3 with σ = (1, 1e-4, 1e-7):** the null vector is off by 2.5e-8, against about 2e-12 expected from a backward-stable SVD.
  - **References:** Golub & Van Loan, *Matrix Computations*, §8.6 (forming `AᵀA` squares the condition number); Demmel & Veselić, "Jacobi's method is more accurate than QR", SIAM J. Matrix Anal. Appl. 13(4), 1992.
- **Suggested fix:**
  - Route every 3x3 SVD through one backward-stable helper in `linalg`. A one-sided Jacobi SVD for small matrices is about 60 lines, and it is the algorithm Eigen's `JacobiSVD` (the C++ core) uses, so it would also tighten parity. The alternative is `nalgebra-lapack`, which matches numpy.
  - Correct the `linalg.rs` module doc and the rust-port note.
  - Add known-answer tests with thin triangles and near-rank-1 E and H, compared against numpy fixtures, on every backend.

##### [must-fix] nalgebra's dynamic SVD (`linalg::svd`) is not backward stable on graded matrices: Rust's 8-point E and F differ from the references by up to 6e-3
- **Where:** `rust/slamlab-foundations/src/linalg.rs:31-36` and `:54-62`. Its consumers:
  - `correspondences.rs:113` (`null_vector`: the 8-point E and F and the H DLT, including every LO refit);
  - `triangulation.rs:184` (DLT);
  - `fivepoint.rs:688` (polish) and `:828` (Stewénius eigenvectors);
  - `pnp.rs:206-210` (EPnP `lstsq`);
  - `ba/gauge.rs:190`, `:210` and `:310`.
- **Issue:** On a small fraction of matrices whose singular values are spread out, nalgebra 0.35's Golub-Kahan SVD returns factorisations with backward error far above ε. The error is in the trailing singular pairs, which are exactly the null vectors and rank decisions these algorithms read. Changing `eps` does not fix it; ε, 5ε, 1e-20 and 1e-12 all show it.
- **Why it matters:**
  - The public `essential_8pt` and `fundamental_8pt` give wrong results on realistic inputs: near-planar and noisy.
  - E-RANSAC's local optimisation always refits with the 8-point algorithm (`fit_nonminimal`), so the defect is on the main robust path, and there Rust decides differently from C++.
  - The parity fixtures and TUM surveys pass only because the failures are rare: about 0.1-0.2% of systems.
- **Evidence** (probe saved at `nalgebra-svd-probe/`):
  - **A survey of `|U S Vᵀ - M|`** over random matrices with σ1 = 1 and the rest log-uniform in [1e-8, 1], using the exact call `try_svd(.., 5ε, 10_000)`:
    ```
    3x3: > 1e-12 in 70/3000, worst 1.4e-4;  10x4: 28/3000, 4.5e-5
    9x9: 22/3000, 1.2e-4;                  12x9: 16/3000, 5.7e-5
    ```
  - **One case cross-checked against LAPACK:** nalgebra gives σ8 = 1.098303e-6 and σ9 = 2.957285e-7; numpy gives 1.098281e-6 and 2.957226e-7. The two null vectors differ by 1.8e-3.
  - **Realistic Hartley-normalised 8-point systems** (20 matches, noise from 0 to 1e-3, half of them near-planar; `main.rs` plus `compare.py`): 42 of 20000 null vectors are worse than 1e3 times the backward-stable bound. The worst `|v_nalgebra - v_lapack|` is 3.6e-3.
  - **End-to-end through the bindings** (`end_to_end_8pt.py`, 20000 scenes), scenes whose result differs from Python's by more than 1e-6, up to sign:
    ```
    F rust: 13/20000, worst 3.0e-03     F cpp: 0/20000, worst 2.4e-10
    E rust: 17/20000, worst 6.0e-03     E cpp: 0/20000, worst 2.5e-11
    ```
  - **Unaffected:** the five-point null space has exactly-zero singular values (`|A v_k|/σ1 ≤ 5e-16` over 200k vectors), and the Schur eigenvalues match numpy to 1e-13 (5000 random 10x10 matrices).
  - **Related upstream reports:** nalgebra issues [#1072](https://github.com/dimforge/nalgebra/issues/1072) and [#1313](https://github.com/dimforge/nalgebra/issues/1313).
- **Suggested fix:** The same helper as the previous finding: a one-sided Jacobi SVD, which suits these sizes (at most 20x12 per call, BA's gauge included), or `nalgebra-lapack`. Then add a `linalg` test with a stored graded matrix and its LAPACK factorisation, and an 8-point parity case on a near-planar noisy scene, so a regression shows up.

##### [should-fix] The five-point solvers return their solutions in different orders on every backend, so lock-step RANSAC relies on there being no near-ties
- **Where:** `rust/slamlab-foundations/src/fivepoint.rs:800` (Nistér: roots in `z`, which depends on the basis) and `:861` (Stewénius: eigenvalues in Schur-diagonal order). The claim is in `docs/notes/rust-port.md:20`: "a minimal solver has to return its models in the same order".
- **Issue:** The solution sets agree across backends; the orders do not.
  - The 4-dimensional null-space basis is arbitrary and differs between nalgebra, Eigen and LAPACK. Nistér's `z` coordinate, and so the order of the sorted roots, depends on it.
  - Stewénius' order follows each library's Schur form.
  - This was already true between Python and C++; the port inherits it.

  `ransac::best_of` keeps the first of equal costs, and `estimate` runs LO, which draws from the shared PCG32 stream, on each new best. Where two models' MSAC costs differ only at roundoff level, the backends can pick different models, and their random streams then diverge. A sample whose models explain only its own five points is such a case.
- **Why it matters:** The note promises decision-level lock-step and says it relies on identical ordering. That holds on the benches and surveys, but not by construction.
- **Evidence:** 2000 random samples per solver, half near-planar, comparing `fivepoint.essential_5pt` across backends:
  ```
  stewenius cpp vs rust: same set, different order 1843/2000 (worst matched diff 3.3e-10)
  nister    cpp vs rust: same set, different order 1786/2000; different count 16/2000
  nister python vs cpp:  different count 18/2000 (pre-existing Nistér sensitivity)
  ```
  On a noise-free scene, `essential_ransac(..., refine=True)` reports a different `refined` flag per backend: over 5 seeds, Rust's 8pt never refines, while C++'s refines in 4. That comparison is roundoff against roundoff.
- **Suggested fix:** Either give all three languages a canonical, basis-independent order for minimal-solver output (for example, sign-normalise each E and sort its entries lexicographically), or reword the note: agreement holds where costs are separated by more than roundoff, and is not guaranteed at near-ties.

##### [note] P3P line-split tie-break: Python's `argmax` is row-major, Eigen and Rust are column-major
- **Where:** `rust/slamlab-foundations/src/p3p.rs:94` and `:576-578`; `docs/notes/rust-port.md:82`. The Python side is in `src/slamlab/foundations/p3p.py`: `np.unravel_index(np.argmax(np.abs(rank_one)), (3, 3))`.
- **Issue:** The doc says the first maximum is picked "as Eigen's `maxCoeff` and numpy's `argmax` pick it, in column-major order". On a C-ordered matrix, numpy's `argmax` is row-major, so on an exact tie Python picks a different row and column from C++ and Rust. It is harmless: a rank-one matrix's rows and columns are parallel, and `ding_depths` uses only ratios. P3P's order and values agreed in 3000/3000 Ding samples and in 2999/3000 Lambda Twist samples; the one exception is the thin-triangle effect above.
- **Suggested fix:** Correct the comment and the note's wording.

##### [note] The refiner's `.expect` and `.value()` justification checks a different E from the one the solve starts at
- **Where:** `rust/slamlab-foundations/src/epipolar.rs:782-791`, and its C++ counterpart `cpp/src/foundations/epipolar.cpp:485-491`.
- **Issue:** The comment says the inliers' Sampson distances are finite because `r < θ²`. But `refine` starts from `start_motion(model)`, which is `[t]x R` rebuilt from the model's decomposition, not the model itself. For a match exactly at both epipoles, the model's g can be tiny but positive while the rebuilt E's g is exactly 0. That would be a panic in Rust and a `bad_expected_access` in C++. My probe found no reachable case: 40 forward-motion matches plus 3 matches exactly at (0, 0, 1), over 5 seeds, 3 solvers and 3 backends, all marked inliers, and none crashed. PnP's justification is exact, because the same `reprojection` evaluates the same pose.
- **Suggested fix:** Reword the comment to cite the start E, or map an `Err` from the refiner to "refinement not run" instead of panicking.

##### [note] C++'s P3P depth polish stops earlier than Python and Rust on near-singular Jacobians
- **Where:** `cpp/src/foundations/p3p.cpp:199-203` uses `FullPivLU::isInvertible()`, with a relative threshold of about 3ε. Rust's `rust/slamlab-foundations/src/p3p.rs:305` uses `full_piv_lu().solve`, which returns None only for an exactly zero pivot (`full_piv_lu.rs:245`); numpy's `LinAlgError` also fires only on exact singularity.
- **Issue:** Rust matches the Python reference here; C++ is the outlier. This is outside the PR's scope: a backlog item if exact three-way agreement matters.

##### [note] A repeated real eigenvalue in Stewénius gives one eigenvector, not two
- **Where:** `rust/slamlab-foundations/src/fivepoint.rs:826-834`.
- **Issue:** Each eigenvector is taken as the SVD null vector of `A - λI`, so both copies of a repeated λ return the same vector, and `keep` drops one. For a non-defective double eigenvalue, Eigen and numpy return two independent vectors. Two distinct solutions with exactly the same `x` is a measure-zero case. It is worth one sentence in the module doc.

#### Checked and fine
- **Correspondences:** Hartley normalisation and `null_vector` match C++ and Python, including the `!(s7 > tol·s0)` rank test and its behaviour with fewer than 8 rows. Padding with zeros to 9 rows leaves the row space unchanged.
- **Triangulation:** two-view `niter2` and the n-view DLT match C++ statement for statement: the normalised world frame, the parallel and undetermined tests that NaN fails, cheirality, and parallax.
- **LO-MSAC:** the loop matches C++ and Python:
  - the adaptive bound via `ln_1p`;
  - iterative least squares with a shrinking threshold;
  - the inner non-minimal RANSAC with `min(factor·s, |I|/2)`;
  - a strict `<` for a new best;
  - the refine-and-reselect rounds, with the unchanged-inliers exit.

  `f64::min` ignores NaN where `np.minimum` and `cwiseMin` keep it, but no residual can be NaN: every residual function turns non-finite values into +inf through comparisons that NaN fails.
- **E, F and H:** the Sampson errors and the Malis–Vargas decomposition (`amax`, the first `iamax` of |diag|, clamped radicands, projection to SO(3)) match C++. nalgebra's `iamax`/`iamin` use strict `>`/`<`, the same first-max rule as Eigen's `maxCoeff`.
- **The Sampson refinement:** its 5-dof tangent parametrisation, its dE directions and its retraction match C++ and Python. So does `start_motion`: the largest trace with the first max, and the translation's sign set by its largest-magnitude component.
- **Two-view:** GRIC, the H threshold scale, the stable sort, and the ambiguity, low-parallax and pure-rotation rules match C++.
- **P3P:** the cubic root (Cardano/Viète, then Newton), the quadratic solved without cancellation, `scaled`, the known-zero eigen split and the adjugate line split all match. The solution order agreed with Python and C++ in 5999 of 6000 samples.
- **EPnP:** the ascending sort of nalgebra's unsorted symmetric eigen-decomposition, `deterministic_sign`, numpy's `lstsq` cutoff and the beta polish agree with Python and C++ to within 7e-10 over 3000 noisy 12-point problems.
- **Eigenvalues:** nalgebra's Schur eigenvalues agree with numpy to 1e-13 over 5000 random 10x10 matrices, and the five-point null space is exact.
- **The C++ `refine` fix:** it propagates `nls::solve`'s `Result` correctly, and the new `test_refine_rejects_a_match_at_both_epipoles` covers all three backends. The Python gauge's mapping of `LinAlgError` to `DegenerateError` matches Rust.
- **The methods are current for rung 0:** Lambda Twist (Persson & Nordberg, ECCV 2018), Ding et al. (CVPR 2023), Stewénius' action matrix, and LO-RANSAC with MSAC scoring (Lebeda et al., BMVC 2012).

The probe scripts are saved for reuse in `nalgebra-svd-probe/` (`src/main.rs`, `compare.py`, `end_to_end_8pt.py`, `umeyama_three_points.py`, `p3p_thin_triangles.py`). No repository files were modified.
