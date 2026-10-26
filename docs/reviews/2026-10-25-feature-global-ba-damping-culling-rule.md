# Review panel: `feature/global-ba-damping-culling-rule`

## Run 1 -- 2026-10-25

| | |
|---|---|
| **Branch** | `feature/global-ba-damping-culling-rule` |
| **Range** | `8e9ae9c1...0aa70550` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus): PATCH change |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | at `97de38f3`, whose tree equals `0aa70550`'s except `CHANGELOG.md`: all 29 checks passed (`scripts/check.sh --env slam-it`, exit 0; pytest 5,216 passed, 1 skipped, 19 deselected) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `rust/slam-it-edge/tests/loops.rs:173-175,200-201` | The RANSAC test's doc comment now sits on the new global-BA test; the RANSAC test has none | fix: move it back, give the new test its own | fixed (ca605382: the RANSAC test's doc comment back, global BA's solve test one of its own) -- decided by evan-wes, 2026-10-25 |
| H2 | should-fix | hygiene | `tests/edge/test_merges.py:348,538`; `CHANGELOG.md` | No test shows a native weld reads `loops.near_fraction`/`weld_options`, though the CHANGELOG says each backend reads the four settings | fix: a weld parity test with the setting moved, held to Python and to the default run | fixed (ca605382: each compiled backend's weld held to Python's with `loops.min_depth_ratio` at 0.9 and global BA off, leaving observations out where the default leaves none; e6ae221a: the CHANGELOG's sentence names it) -- decided by evan-wes, 2026-10-25 |
| H3 | should-fix | hygiene | `rust/slam-it-edge/src/loops.rs:3539-3542` | `#[allow(clippy::too_many_arguments)]` on 7-parameter `weld_solve` is dead and its reason wrong | fix: delete it | fixed (20245e69: deleted) -- decided by evan-wes, 2026-10-25 |
| H4 | should-fix | hygiene | `tests/fixtures/edge/generate_covisibility.py:196,279` | `connected`'s documented empty result is tested in no language | fix: an isolated-keyframe fixture case, all three languages | fixed (ca605382: a case whose newest keyframe shares no point, its links and connected keyframes empty, read by every backend and both natives) -- decided by evan-wes, 2026-10-25 |
| H5 | should-fix | hygiene | `src/slam_it/edge/database.py:341-347`, `cpp/src/edge/database.cpp:250-255` | Local `sharing` (shares a point) beside `Candidates.sharing` (shares a word) | fix: rename to `connected_ids` / `connected_rows` | fixed (20245e69: `connected_ids` in Python, `connected_rows` in C++) -- decided by evan-wes, 2026-10-25 |
| H6 | should-fix | hygiene | `src/slam_it/edge/options.py:590-606,365-368` and twins | New settings' docs are chained fragments, not self-contained | fix: a complete first sentence per field | fixed (20245e69: a complete sentence per culling and global BA setting, in the three languages) -- decided by evan-wes, 2026-10-25 |
| H7 | should-fix | hygiene | `src/slam_it/edge/loops.py:1617,1645` and twins | BA docs cite an unnamed quantity ("0.99 so") and an unrecorded probe; languages word the owner's decision differently | fix: name the quantity, point to the CHANGELOG, word the three alike | fixed (20245e69: the scale carrying the cameras' centres named, 0.55 under GTSAM's damping and 0.99 under g2o's; the welds' figures by the CHANGELOG; the three languages worded alike) -- decided by evan-wes, 2026-10-25 |
| H8 | note | hygiene | `src/slam_it/edge/loops.py:87-93`, `options.py:368-373`, `graphs.py` | Measurements and incident dates in docstrings will go stale | fix (with H6/H7): state the rule and point to the design | fixed (97c1acc5: rung 0's docstrings; 20245e69: the settings' and loop closing's state the rule and point to the design) -- decided by evan-wes, 2026-10-25 |
| H9 | note | hygiene | `tests/fixtures/edge/generate_global_ba_slide.py` | `global_ba_slide.json` can't be regenerated from the repo | defer (backlog: a dump script for the loop room's global BA) | deferred (the online-SLAM design's section 12: a dump script for the loop room's global BA) -- decided by evan-wes, 2026-10-25 |
| H10 | note | hygiene | `graphs.cpp:1125-1163`, `test_graphs.cpp:817`, `test_database.py:191`, `test_loops.py:821` | Small readability and test-hygiene nits | fix | fixed (97c1acc5: C++'s `solving`; the stop test's bare 6 replaced by where a count ignoring the reset would end, in every language; C++'s near scene an `expected` vector. ca605382: `LoopKeyframes` once in conftest; the 20-times-tolerance bound explained; Python's and Rust's emulations name `minModelFidelity`) -- decided by evan-wes, 2026-10-25 |
| H11 | note | hygiene | `rust/slam-it-foundations/src/graphs.rs:1656` | Rust `kept_observations` is `pub`, indexes unchecked, no `# Panics` | fix (by A4) | fixed (97c1acc5: by A4, Rust's `kept_observations` private) -- decided by evan-wes, 2026-10-25 |
| H12 | note | hygiene | `CHANGELOG.md:7` | Heading date 2026-10-24 vs the version commit's 2026-10-25 | fix: the final version commit's date | fixed (e6ae221a: 2026-10-25, the version commit's date) -- decided by evan-wes, 2026-10-25 |
| A1 | should-fix | architecture | `graphs.py:167-168`, `graphs.hpp:77-79`, `graphs.rs:112-113`; edge options | `initial_damping` is λ or τ depending on a flag; `loops.global_damping` (τ) sits beside `loops.essential_damping` (λ) | fix: a separate τ field (`damping_scale`) in all three; rename the setting `loops.global_damping_scale` | fixed (97c1acc5: `graphs.Options.damping_scale`, g2o's τ, in the three languages and both bindings; `initial_damping` GTSAM's absolute first λ alone, its docs exact. 20245e69: `loops.global_damping_scale`) -- decided by evan-wes, 2026-10-25 |
| A2 | should-fix | architecture | `graphs.py:142-143,675-677,1275-1283` and twins | The small-steps stop and an all-left-out bundle both report `FUNCTION`, whose docs cover neither | fix: a `SMALL_STEPS` termination (counted as converged) in all three; nothing-kept observable through A3's count | fixed (97c1acc5: `Termination.SMALL_STEPS`, code 4, converged, in the enum, the code tables and both bindings of every language; each termination's docs say what ends there, `FUNCTION`'s a solve with nothing to solve; nothing kept observable by `left_out`) -- decided by evan-wes, 2026-10-25 |
| A3 | should-fix | architecture | `Adjustment` in all three; global BA and weld records | The near filter drops observations without counting them | fix: count them in `Adjustment` and both bindings, and record it with global BA's and the weld's solves | fixed (97c1acc5: `left_out` in the solve summary, through both bindings, the edge bindings and the native programs' diagnostics, and loop closing's solve records for global BA, the weld and the essential graph. No run-format bump: the stage summaries in `diagnostics.jsonl` are free-form, and no reader needs the new keys) -- decided by evan-wes, 2026-10-25 |
| A4 | should-fix | architecture | `graphs.py:573,1208`, `graphs.hpp:237`, `graphs.rs:1656` | `largest_hessian_diagonal` (Python) and `kept_observations` (all three) public without need; the latter unvalidated | fix: internal in all three, tested through `bundle_adjust` | fixed (97c1acc5: `_largest_hessian_diagonal`; the near filter's kept observations internal in all three languages, tested through `bundle_adjust` and its counts) -- decided by evan-wes, 2026-10-25 |
| A5 | should-fix | architecture | `src/slam_it/edge/options.py:601-606,656,936` and twins | `loops.near_fraction` reuses loop closing's "near" and is unbounded | fix: rename `loops.min_depth_ratio`, bounded to [0, 1) | fixed (20245e69: `loops.min_depth_ratio`, in [0, 1) in the three checkers, tested in each) -- decided by evan-wes, 2026-10-25 |
| A6 | note | architecture | `loops.py:1609,1640,1677,2307` and twins | Each BA's settings split between `*_options` and the call site's `near` | waive: acceptable while both BAs share one setting, and `test_merges.py` pins both | waived: acceptable while both BAs share one setting, and `test_merges.py` pins both -- decided by evan-wes, 2026-10-25 |
| A7 | note | architecture | `docs/design/2026-10-14-online-slam.md:660-661` | Local BA's near-plane guard (a later PR) is untracked and would add a second rung-0 near rule | fix: track it in the design's plan, sharing one rung-0 function | fixed (46b027dd: section 7's plan tracks local BA's near-plane guard, one rung-0 function shared by `foundations.ba` and `foundations.graphs`, the setting's group decided there) -- decided by evan-wes, 2026-10-25 |
| A8 | note | architecture | `src/slam_it/foundations/graphs.py:50,1392` | Rung-0 docs name a rung-1 setting and policy | fix: describe the behaviour only | fixed (97c1acc5: rung 0's docs describe the behaviour only) -- decided by evan-wes, 2026-10-25 |
| A9 | note | architecture | `docs/design/2026-10-14-online-slam.md:282-286,648-661,811-825` | The as-built record lacks the measurement (and whether the loop sequences ran); section 12's deferred item has no as-built pointer | fix: as-built measurement incl. the loop sequences, and the pointer | fixed (46b027dd: section 11's L2 records the measurement, the loop sequences included, threaded beside the ORB-SLAM3 anchor; section 12's dump item points to `global_ba_slide.json`) -- decided by evan-wes, 2026-10-25 |
| L1 | should-fix | algorithm | `CHANGELOG.md:88`; `loops.py:1609`; `graphs.cpp:715` | zju-A1's 0.637 is the non-robust global BA fitting ~5k gross outliers at the inliers' expense (17 points cross behind), not a correction | fix: correct the CHANGELOG and the design's L2 note; count kept observations ending behind or near (with A3). Defer: where zju-A1's 1.97e8 start cost comes from, a gross-outlier gate before global BA, a crossing as a failed trial | fixed (97c1acc5: `ended_near`, the kept observations ending behind their camera or nearer than the limit, in the summary and the solve records; 46b027dd and e6ae221a: the design's L2 note and the CHANGELOG read zju-A1's two dumps with the reviewer's figures); deferred (the online-SLAM design's section 12: the 1.97e8 starting cost's source, a gross-outlier gate before global BA, a crossing as a failed trial) -- decided by evan-wes, 2026-10-25 |
| L2 | note | algorithm | `tests/edge/test_loops.py:798-806` | L4's test is right but its stated mechanism is slightly off (a near-flat valley, not the exact gauge) | fix: reword, and assert both solves' baseline/median-depth ratios agree | fixed (ca605382: the docstring names the near-flat valley beside the exact gauge; both solves' gauge-free ratio, the cameras' mean step over the median depth, asserted to agree within 1%) -- decided by evan-wes, 2026-10-25 |
| L3 | note | algorithm | `graphs.py:1208`, `graphs.cpp:1079-1100`, `graphs.rs:1656-1680` | The near filter acts at the start only, uses one global median, and its effect on welds is unmeasured | fix: document start-only; defer per-keyframe medians (backlog) | fixed (97c1acc5: the filter documented as acting at the start only, in every language); deferred (the online-SLAM design's section 12: per-keyframe medians) -- decided by evan-wes, 2026-10-25 |
| L4 | note | algorithm | `loops.py:87-94`; design 476-483 | The remaining departure rests on one sequence each way | defer (backlog: count near-check passes of connected-but-unlinked groups over the test set, C++ and Rust) | deferred (the online-SLAM design's section 12: count the near check's connected-but-unlinked passes over the test set, C++ and Rust) -- decided by evan-wes, 2026-10-25 |
| L5 | note | algorithm | `CHANGELOG.md` "Measured" | The CHANGELOG doesn't say which change causes each regression | fix: one attribution sentence per regression; repeat the threaded runs before reading them | fixed (e6ae221a: one sentence per regression on which change makes it, the threaded and RGB-D ones not attributed and why) -- decided by evan-wes, 2026-10-25 |
| L6 | note | algorithm | `graphs.py:610`, `graphs.cpp:407` | Marquardt (diagonal) damping would make the filter's damping role unneeded | waive: this rung follows ORB-SLAM3's g2o damping; context only | waived: this rung follows ORB-SLAM3's g2o damping; context only -- decided by evan-wes, 2026-10-25 |
| L7 | note | algorithm | `src/slam_it/edge/mapping.py:1186` | Culling tests only 15-point links, where ORB-SLAM3 tests every covisible keyframe (up to 100) | defer (backlog: compare keyframe counts with the ORB-SLAM3 anchor) | deferred (the online-SLAM design's section 12: culling's candidates against ORB-SLAM3's, keyframe counts from the ORB-SLAM3 anchor) -- decided by evan-wes, 2026-10-25 |
| L8 | note | algorithm | `graphs.py:1275`; `kept_observations` | An all-left-out bundle ends converged; `kept_observations` is unvalidated | fix (by A2, A3, A4) | fixed (97c1acc5: by A2 to A4) -- decided by evan-wes, 2026-10-25 |

**After the fixes** (`97c1acc5`, `20245e69`, `ca605382`, `46b027dd`, `e6ae221a`):

- The rooms' golden records, regenerated at `97c1acc5`: no decision or pose changes in any case or language. The three loop rooms' global BA ends as `small_steps` where it read `function` (after 7, 10 and 9 iterations, monocular, RGB-D and stereo, as before), and every solve record gains `left_out` and `ended_near`, 0 in each. The later commits change no record: the records' tests pass at `20245e69`, and its successors touch tests and documents alone.
- Each commit's touched checks pass (formatters, linters, clang-tidy, doxygen, clippy, mypy and jscpd on what it changes; the native tests of the touched modules; pytest of the touched modules with the rooms' records: 537 tests for `97c1acc5`, 845 for `20245e69`, 564 for `ca605382`).
- Layer 1 at `e6ae221a` with this report in its tree, as committed but for this line (`scripts/check.sh --env slam-it`): exit 0, all 29 checks ok (pytest 5,253 passed, 1 skipped, 19 deselected).
- The reviewers were not run again: each fix is the one its finding recommends, as decided.

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `git diff 8e9ae9c1...0aa70550` (55 files, 7 commits, 0.3.17.2 to 0.3.18.0). I read every source, test and generator hunk in C++, Rust and Python, the changed docs, and the CHANGELOG entry. I re-read the touched functions in full at HEAD. Regenerated fixtures and golden records were checked only for the claims made about them. Read-only: I built nothing in the checkout. I ran one throwaway clippy probe in a scratch crate outside the repository.

**Summary:** The slice is clean and production quality: validation is in every language, failures name the bad value, there are no `unwrap`, prints or TODOs in library code, and every new declaration is documented. The tests are strong because they assert values. I found no must-fix. There are seven should-fix items, all small: a misattached doc comment, a dead lint suppression, one coverage gap that the CHANGELOG's wording overstates, an unverified `connected` empty result, a confusing local name and two documentation-quality points. The notes are minor. The CHANGELOG figures I spot-checked match the measurement files.

#### Findings

##### [should-fix] A Rust test carries another test's doc comment, and the RANSAC test lost its own
- **Where:** `rust/slam-it-edge/tests/loops.rs:173-175` and `:200-201`
- **Issue:** The new `global_ba_damps_as_g2o_and_stops_as_orb_slam3_and_the_weld_keeps_gtsam_s_damping` was inserted between the doc comment and the `#[test]` of the RANSAC test. The comment `/// The RANSAC budget: SetRansacParameters(0.99, 15, 300).` now documents the global-BA test. `the_ransac_budget_is_orb_slams` has no doc comment, and the new test has no description of its own.
- **Why it matters:** A comment that contradicts the code it sits on is misleading. The project documents everything, and no tool catches this because `tests/` items are not covered by the missing-docs lints.
- **Evidence:**
```
173: /// The RANSAC budget: `SetRansacParameters(0.99, 15, 300)`.
174: #[test]
175: fn global_ba_damps_as_g2o_and_stops_as_orb_slam3_and_the_weld_keeps_gtsam_s_damping() {
...
200: #[test]
201: fn the_ransac_budget_is_orb_slams() {
```
- **Suggested fix:** Move the RANSAC comment back above `the_ransac_budget_is_orb_slams`. Give the new test a doc comment of its own, like the C++ twin's `check_solves` brief.

##### [should-fix] No native test shows the welding BA reads `loops.near_fraction` or `weld_options`, though the CHANGELOG says each backend reads the four settings
- **Where:** `tests/edge/test_merges.py:348` (the weld test) and `:538` (backend parity). Native side: `cpp/tests/edge/test_loops.cpp:60` and `rust/slam-it-edge/tests/loops.rs:174`. The CHANGELOG claim is at `CHANGELOG.md`, "The weld and global BA" under "### Tests".
- **Issue:** `test_every_backend_reads_global_ba_s_settings` (`tests/edge/test_loops.py:748`) moves all four settings and compares each native with Python, but only for global BA. For the weld:
  - `test_the_weld_keeps_gtsam_s_damping_and_global_ba_takes_g2o_s` monkeypatches `graphs.bundle_adjust`, so it records the Python reference path only.
  - `test_every_backend_merges_as_python_does` runs at default settings.
  - The C++ and Rust tests construct `weld_options(5)` and check its fields. No native test calls `weld` or `merge`.
- **Why it matters:** `tests/edge/test_loops.py:748` records the rule (H21) that a native which hard-coded a setting must be caught. A native `weld` that hard-coded 1e-3 for `near`, or used `global_ba_options`, would pass unless the small merge fixture happens to have near observations or a cost that moves. The CHANGELOG sentence "each compiled backend reads the four settings (H21's rule)" holds for global BA only.
- **Evidence:** `rg weld cpp/tests/edge/test_loops.cpp` gives only lines 5, 57, 69-75, which are the option constructors and the file header. The `merge` calls in `tests/edge/test_merges.py` use the default `OPTIONS`.
- **Suggested fix:** Add a sibling of `test_every_backend_merges_as_python_does` with `loops.near_fraction` moved (for example 0.9, as `MOVED_GLOBAL_BA` does). Compare the weld's summaries and map with Python, and assert they differ from the default run. Otherwise reword the CHANGELOG sentence to "global BA".

##### [should-fix] The `#[allow(clippy::too_many_arguments)]` on `weld_solve` is dead, and its reason is wrong
- **Where:** `rust/slam-it-edge/src/loops.rs:3539-3542`
- **Issue:** `weld_solve(problem, camera, state, used, robust, iterations, near)` has 7 parameters. Clippy's threshold fires above 7.
- **Why it matters:** An unneeded suppression with a justification string (`"the problem, its state, which observations, and the solve's three settings"`) misleads. It also hides a real lint if the function later grows.
- **Evidence:** A scratch crate outside the repo, built with the env's clippy, flagged `eight(...)` as "too many arguments (8/7)" and did not flag the 7-parameter `seven(...)`. The other allows in the file are on functions with 8 or more.
- **Suggested fix:** Delete the attribute. The `new_optimizer` allow on `mod ffi` (8 parameters, `graphs.rs:696`) is needed and fine.

##### [should-fix] `connected`'s documented empty result is tested in no language
- **Where:** `tests/fixtures/edge/generate_covisibility.py:196` and `:279`; the contract is in `src/slam_it/edge/covisibility.py` (`connected`: "none if no keyframe shares a point"), `cpp/include/slam_it/edge/covisibility.hpp` and `rust/slam-it-edge/src/covisibility.rs`.
- **Issue:** The generator asserts only `any(c["connected"] != c["links"])`. I checked the fixture: 0 of the 52 keyframe rows have an empty `connected`, and none have empty `links`. Every backend's `connected` is therefore exercised only on keyframes that have neighbours. The loop-candidate tests (n = 0 sharing) always have other keyframes sharing points with the query.
- **Why it matters:** Empty input is the edge case the docs promise. `connected` feeds loop-candidate exclusion, so a mishandled empty list (for example the best-link fallback in `links_of`) would go unseen.
- **Evidence:** `python -I` over `tests/fixtures/edge/covisibility.json` printed "0 empty connected" for all five cases.
- **Suggested fix:** Add a fixture case with an isolated keyframe (or a one-keyframe map) and assert that its `connected` and `links` are empty in all three languages. The generator's existing `assert` pattern can guard that the case exists.

##### [should-fix] A local named `sharing` means "shares a point" beside `Candidates.sharing`, which means "shares a word"
- **Where:** `src/slam_it/edge/database.py:341-347` and `cpp/src/edge/database.cpp:250-255`
- **Issue:** In `_loop_candidates`, `sharing = {...connected...}` holds the ids of keyframes sharing a point with the query. Three lines later `replace(chosen, sharing=found.sharing, ...)` copies `Candidates.sharing`, the count of keyframes sharing a word. C++ repeats it with `auto sharing = connected(m, row);`.
- **Why it matters:** The same word means two things on adjacent lines, in the function that implements the change. The CHANGELOG and docs all have to qualify it ("sharing no point with it that share a word").
- **Evidence:**
```
sharing = {int(k) for k in m.kf_id[covisibility.connected(m, row)[0]]}
...
return replace(chosen, sharing=found.sharing, scored=found.scored)
```
- **Suggested fix:** Rename the locals to `connected_ids` (Python) and `connected_rows` (C++).

##### [should-fix] The new setting docs are chained fragments, not self-contained
- **Where:** `src/slam_it/edge/options.py:590-606` (also `:365-368`), `cpp/include/slam_it/edge/options.hpp:354-367`, `rust/slam-it-edge/src/options.rs:530-543`
- **Issue:** Each field's text continues the previous one's trailing "...". Examples: `global_damping` begins "...from this damping, a fraction of..."; `global_small_decrease` is only "...that fraction (0.1%)."; `redundant_observers` begins "...this many (4)".
- **Why it matters:** Doxygen briefs, rustdoc members, IDE hovers and numpydoc parameter entries are read one at a time. `global_small_decrease` alone says nothing, and reordering fields breaks the prose.
- **Suggested fix:** Give each field a complete first sentence and keep the shared explanation in the group docstring. For example: "`global_small_decrease`: the fraction of the error below which an iteration counts as a small step (ORB-SLAM3's 1e-3)".

##### [should-fix] Two references in BA docs the reader cannot follow
- **Where:** `src/slam_it/edge/loops.py:1617` and `:1645`. The C++ and Rust twins are at `cpp/include/slam_it/edge/loops.hpp` (`weld_options`) and `rust/slam-it-edge/src/loops.rs` (`weld_options`).
- **Issue:**
  - `global_ba_options` says GTSAM's damping let the map slide "(0.55 on the loop room's failing run, 0.99 so)". The quantity (the scale of the similarity carrying the cameras' centres onto where they started, as `tests/edge/test_loops.py` defines it) is not named, and "0.99 so" is unclear.
  - `weld_options` cites "the five welds the F2 split probe dumped". The probe is described nowhere in the repo except the CHANGELOG.
  - The C++ and Rust twins say "by the owner's decision" with no date, where Python says "decided with evan-wes on 2026-10-23".
- **Why it matters:** A docstring figure with no named quantity cannot be checked, and a pointer to a probe that is not recorded is a dead end.
- **Suggested fix:** Name the quantity ("scale 0.55 under GTSAM's damping, 0.99 under this one"). Point to "CHANGELOG 0.3.18.0, Measured" for the weld figures. Word the three languages alike.

##### [note] Measurements and incident dates in module and option docstrings will go stale
- **Where:** `src/slam_it/edge/loops.py:87-93` (ATE "29.6 to 90.7 cm", "C++, inline"), `src/slam_it/edge/options.py:368-373` ("the revisit room's map lost, 2026-10-21"), and the "since 0.3.18.0" asides in the `graphs.py` module docstring.
- **Issue:** These are history and run results. The repo already does this ("0.3.17.2's review panel, L5"), so it is not a defect. The CHANGELOG and the design doc carry the same figures, so they will drift in two places.
- **Suggested fix:** Where a docstring can state the rule and point to the design section, do that.

##### [note] `global_ba_slide.json` cannot be regenerated from the repository
- **Where:** `tests/fixtures/edge/generate_global_ba_slide.py:3-4` and `:12-19`
- **Issue:** The generator reads `.npy` dumps written by "a temporary probe", and says the files are not in the repository. This is honestly documented, but it is the only fixture here whose generator cannot reproduce it.
- **Suggested fix (backlog):** Keep the dump hook or a small dump script that can run on a loop-room run, or record where the dump lives, so the "tenth of the points" selection can be redone.

##### [note] Small readability and test-hygiene nits
- **Where and issue:**
  - `cpp/src/foundations/graphs.cpp:1125-1163`: `const Reached solved_by` reads "solved by what?". `solving` or `free` would read better.
  - `cpp/tests/foundations/test_graphs.cpp:817, 869, 885`: `near_scene_keeps(from_end)` counts observations from the end. The Python and Rust twins carry a parallel `expected` vector, which is clearer.
  - `tests/edge/test_database.py:191` adds a `LoopKeyframes` alias for a tuple that `tests/edge/conftest.py:288-425` spells out six times. Define it once in conftest.
  - Test constants: `1.0 - 20 * SCALE_TOLERANCE` (`tests/edge/test_loops.py:821`) and `ends > 6` have no stated reason. The Python and Rust GTSAM emulations inline `1e-3` where the C++ one names `kMinFidelity`.

##### [note] Rust `kept_observations` is `pub`, indexes without checking, and has no `# Panics`
- **Where:** `rust/slam-it-foundations/src/graphs.rs:1656`
- **Issue:** `poses[c]` and `points[p]` rely on the caller's earlier `check_bundle`. The doc says "checked", which states the precondition, and `missing_panics_doc` does not fire on indexing. Fine as it stands. A `# Panics` line or `pub(crate)` would make it explicit.

##### [note] CHANGELOG heading date versus the commit date
- **Where:** `CHANGELOG.md:7` (`## [0.3.18.0] - 2026-10-24`)
- **Issue:** The version commit `0aa70550` is dated 2026-10-25 03:01 -0700, and the entry cites measurements "2026-10-23 to 25". Check the date against the project's convention (commit or merge day).

#### Checked and fine
- **Validation:**
  - `near` (finite, at least 0) is checked in Python (`graphs.py`), C++ (`graphs.cpp`, `bundle_adjust`) and Rust (`graphs.rs`, `bundle_adjust`) with messages naming the value, and tested for -1e-3, inf and NaN.
  - `small_steps`, `small_decrease` and `scaled_damping` are checked in all three languages, and the four new `loops.*` settings are checked in the options sets of all three.
  - Python's P at least 1 plus every-point-observed rule means `kept_observations` never sees an empty array.
  - C++ `static_cast<int>` on the settings is safe, since `kMaxSetting` is 2^30.
- **Production hygiene:**
  - No `unwrap`, `expect`, prints, TODOs or commented-out code in library code (the `unwrap`s are in tests).
  - GTSAM failures in the new `largest_hessian_diagonal` and `_params` paths are inside the existing catch-all that maps to `NotConverged`.
  - The remaining `noqa`, `allow` and `type: ignore` entries carry reasons.
- **Naming and constants:** The damping constants are named, documented and tied to GTSAM and g2o source (`ABSOLUTE_DAMPING_FACTOR`, `ABSOLUTE_DAMPING_LIMIT`, `SCALED_DAMPING_FACTOR`, `SCALED_DAMPING_LIMIT` and their C++ and Rust twins).
- **Documentation:**
  - Every new declaration has numpy, Doxygen or rustdoc with shapes, frames (`T_world_cam`) and the median rule (the floor(M/2)-th smallest, with the depth summation order).
  - `loop_candidates` still reads `covisibility.min_points` through its group neighbours, so its `options` docs are not stale.
  - The design and note edits are consistent with the code.
- **Duplication:** The `links` and `connected` bindings share `links_out` (C++ and Rust), and `checked_links` is shared in C++. `weld` and `global_bundle_adjust` use `weld_options` and `global_ba_options` in one place per language. The cross-language twins (constants, `largest_hessian_diagonal`, GTSAM emulations) are intentional.
- **Tests that assert values:**
  - Damping schedules: the first three iterations are compared with an emulated GTSAM search, scaled and absolute.
  - Stop rule: it ends at the third small decrease in a row after a reset.
  - Near filter: exact keep flags, kept-only start error, and untouched points, cameras and the held camera bit for bit.
  - Edge and error cases: all-observations-left-out, bad `near`, and bad options.
  - Culling rule: both values (3 and 4) in all three languages.
  - Loop candidates: 1 to 14 shared points are excluded, and none shared is a candidate.
  - Connected keyframes: held to a brute-force fixture.
  - L4 (scale), L5 (the λ bound) and L8 (the stereo half-held case) are closed.
- **Performance:**
  - C++ and Rust use GTSAM's `hessianDiagonal`, so one extra linearisation per scaled solve.
  - Python's per-factor loop in `largest_hessian_diagonal` is acceptable for the reference.
  - `bundle_adjust` now solves only the reached cameras and points, so the early return avoids needless graph work.
- **CHANGELOG spot-checks against the results:**
  - From `results/g_rule_2026-10-23/connected/compare_mono_{cpp,rust}-inline.md`, the monocular inline figures all reproduce:
    - maps 160 to 161 in each
    - losses 143 to 139 and 141 to 138
    - loops 13 to 13 and 13 to 15
    - merges 21 to 23 and 21 to 21
    - wrong-place 2 to 1
    - median ATE 29.39 to 24.81 and 28.02 cm over 56 sequences
    - worst 1,235.5 to 1,795.2 cm
    - peak RSS 1,497 to 3,478 and 1,458 to 2,682 MiB
  - 35 of the 52 keyframe rows in `covisibility.json` have `connected` differing from `links`.
  - `global_ba_slide.json` has 103 keyframes, 537 points and 3,930 observations.
  - Settings count 142 in the Python and Rust tests.
  - The tree at HEAD equals the measured commit 789fe641 apart from CHANGELOG, VERSION and docs.
- **C++ `options.cpp` refactor:** `check_reals` extracted from `check_options` is behaviour-preserving, with array sizes updated (28 and 23).
- **Out of this PR's goal (acknowledged in the CHANGELOG):** local BA's near-plane guard is deferred to a later PR. The grouping and near-check still read the 15-point links, and that is the documented departure. Whether the grouping should read connected keyframes is the algorithm reviewer's question, not hygiene's.

### Architecture review

**Scope:** `8e9ae9c1...0aa70550` (7 commits, 0.3.17.2 -> 0.3.18.0). Read in full: the rung-0 solve and filter (`src/slam_it/foundations/graphs.py`, `cpp/include/slam_it/foundations/graphs.hpp`, `cpp/src/foundations/graphs.cpp`, `rust/slam-it-foundations/src/graphs.rs`, `rust/slam-it-foundations/cxx/graphs_bridge.*`, both `graphs` bindings); the edge changes (`loops`, `options`, `covisibility`, `database` in all three languages, and the `covisibility` bindings); the fixture generators; the test diffs; the design and notes edits; the CHANGELOG entry. For context I also read `src/slam_it/foundations/nls.py`, `src/slam_it/foundations/ba/` and `src/slam_it/edge/adjust.py`.

**Summary:** The structure is sound. Dependencies point only from edge down to foundations. Damping and stop are solve options in rung 0, the filter is a problem argument beside `huber`, and the global/weld policy is built in `loops` from edge settings. The three languages mirror each other closely, and `covisibility.connected` is placed, dispatched, typed in its Protocol and bound the way the project requires. There are no must-fix findings. The should-fix findings cover:
- one option field that means two different things, and the settings named after it;
- two new ways a solve can end that the `Termination` enum does not report or document;
- the filter dropping observations without counting them;
- two rung-0 helpers that are public without needing to be;
- a setting name that clashes with an existing concept in loop closing.

#### Findings

##### [should-fix] `graphs.Options.initial_damping` means λ or τ depending on a flag, and its docs say λ in all three languages
- **Where:** `src/slam_it/foundations/graphs.py:167-168`, `cpp/include/slam_it/foundations/graphs.hpp:77-79`, `rust/slam-it-foundations/src/graphs.rs:112-113`. The edge mirror is `src/slam_it/edge/options.py:585-595` and `:650-653`, `cpp/include/slam_it/edge/options.hpp:351,359`, `rust/slam-it-edge/src/options.rs:527,534`.
- **Issue:** With `scaled_damping=False`, `initial_damping` is GTSAM's absolute first λ. With `True`, it is g2o's τ, which gets multiplied by the largest Hessian diagonal (up to about 1e11). In all three languages the field's own doc still says "Levenberg-Marquardt's first damping (GTSAM's `lambdaInitial`)". Edge repeats the problem:
  - `loops.global_damping` (τ, relative, 1e-5) sits next to `loops.essential_damping` (absolute λ, `setUserLambdaInit(1e-16)`) with the same suffix.
  - Rung 0's other Levenberg-Marquardt solver uses a third meaning: `nls.Options.initial_damping` (`src/slam_it/foundations/nls.py:237`) is a μ scaled by the clamped diagonal.
- **Why it matters:** The same number changes meaning by about 11 orders of magnitude depending on a bool. Copying `essential_damping`'s value into `--option loops.global_damping=...`, or reusing `Options(initial_damping=…)` for the later local-BA guard, silently does something else. g2o keeps the two apart: separate `initialLambda` and `tau` properties, and `computeLambdaInit` uses the user λ when it is above 0, otherwise τ·max diag.
- **Evidence:** `graphs.py:167` "Levenberg-Marquardt's first ``λ``, added to the normal equations' diagonal (GTSAM's ``lambdaInitial``…)" against `_params` (`graphs.py:622-623`): `params.setlambdaInitial(options.initial_damping * (largest …))`.
- **Suggested fix:** Preferred: give τ its own field in `graphs.Options` (e.g. `damping_scale` or `tau`) in all three languages, keep `initial_damping` absolute, and rename the edge setting to say it is relative (`loops.global_tau` or `loops.global_damping_scale`). Minimum: rewrite `initial_damping`'s doc in all three languages to state both meanings.

##### [should-fix] Two new ways a solve ends both report `Termination.FUNCTION`, whose docs cover neither
- **Where:**
  - Python: `src/slam_it/foundations/graphs.py:142-143` (doc), `:675-677` (small-steps stop), `:1275-1283` (nothing kept).
  - C++: `cpp/include/slam_it/foundations/graphs.hpp:48-50`, `cpp/src/foundations/graphs.cpp:463-465` and `:1128-1135`.
  - Rust: `rust/slam-it-foundations/src/graphs.rs:59`, `:992-994` and `:1734-1745`.
- **Issue:** ORB-SLAM3's small-steps stop ends as `FUNCTION`/converged. So does a bundle whose observations were all left out (0 iterations, errors 0). In every language, `FUNCTION`'s doc reads "Converged: an iteration lowered the error by no more than the tolerances, or left it at zero."
- **Why it matters:** The termination reason is part of the solve summary contract (CLAUDE.md, "Error contract"). Global BA's and merges' summaries are written into the run records (`src/slam_it/edge/loops.py:1344`, `:2050` via `_solve_record`). Two consequences:
  - A measurement cannot tell how often the new stop decides. The CHANGELOG's "12 converging" does not say by which rule.
  - A global BA that solved nothing looks the same as one that converged at once with a perfect fit.
- **Suggested fix:** Add a `SMALL_STEPS` reason, counted in `CONVERGED`, to the enum and its code tables in all three languages. Make "nothing kept" observable too, either with its own reason or through the count in the next finding. Minimum: document both paths on `FUNCTION`.

##### [should-fix] The near filter drops observations without counting them
- **Where:** `Adjustment` at `src/slam_it/foundations/graphs.py:301-316`, `cpp/include/slam_it/foundations/graphs.hpp:200`, `rust/slam-it-foundations/src/graphs.rs:247`. Precedent to follow: `src/slam_it/foundations/ba/__init__.py:56` ("Degenerate data is dropped, and counted") and `src/slam_it/edge/adjust.py:26-27` (local BA's dropped observations, which a run records per keyframe).
- **Issue:** `bundle_adjust(near=…)` leaves observations out but returns no count. The two figures that justify the filter came from probes, not from runs:
  - the design's "real maps hold points 1e-9 to 1e-4 of the median depth";
  - the CHANGELOG's "at most 125 observations of a dump".
- **Why it matters:** This is a new step that silently drops data, sitting next to two BAs that both count what they drop. Nothing in a run's diagnostics shows when it fires or how much it removes. That conflicts with "make failure observable".
- **Suggested fix:** Add a count of left-out observations to `Adjustment` (or `Summary`) in all three languages and both bindings' result dicts, and record it with global BA's and the weld's solve records.

##### [should-fix] Two rung-0 helpers became public without needing to be
- **Where:** `src/slam_it/foundations/graphs.py:573` (`largest_hessian_diagonal`); `src/slam_it/foundations/graphs.py:1208`, `cpp/include/slam_it/foundations/graphs.hpp:237`, `rust/slam-it-foundations/src/graphs.rs:1656` (`kept_observations`).
- **Issue:**
  - **`largest_hessian_diagonal`:** In Python it is public, takes `gtsam.NonlinearFactorGraph`/`gtsam.Values`, and is the only public function in the module whose signature exposes GTSAM types. Only `_params` (`:622`) calls it, and no test does. Its C++ twin is in an anonymous namespace (`cpp/src/foundations/graphs.cpp:390`) and its Rust twin is a private bridge function.
  - **`kept_observations`:** It is public in all three languages and checks nothing. C++ indexes `poses[static_cast<std::size_t>(observations.cameras[m])]` (`cpp/src/foundations/graphs.cpp:1089`), which is undefined behaviour out of range. Rust panics (`graphs.rs:1671`). Python raises `IndexError` or `ValueError` (empty M), not `InvalidInputError`. It is exposed only so tests can call it, and the `bundle_adjust` tests already pin the kept set through the starting error.
- **Why it matters:** The public interface should stay small and stable, and a public core entry point should either validate its input or not be public.
- **Suggested fix:** Rename the Python one `_largest_hessian_diagonal`. Make `kept_observations` internal (`_` in Python, anonymous or detail namespace in C++, `pub(crate)` in Rust) and test it through `bundle_adjust`, or validate its inputs and return `Result`/raise `InvalidInputError`.

##### [should-fix] `loops.near_fraction` reuses "near", which already names a different concept in loop closing, and the "fraction" has no upper bound
- **Where:** `src/slam_it/edge/options.py:601-606`, `:656`, `:936`; `cpp/include/slam_it/edge/options.hpp:367`; `rust/slam-it-edge/src/options.rs:542`. The existing concept is at `src/slam_it/edge/loops.py:87` ("the near check (a group holding one of the keyframe's links is passed over)"), the paragraph this PR edits, and in the design's section 10, item 2.
- **Issue:** Within the same settings group and module, "near" now means two unrelated things: covisibility proximity of a candidate's group, and depth relative to the camera plane. The setting is also allowed anywhere in [0, ∞) (`_MAY_BE_ZERO`), so 2.0 leaves out more than half the observations. Every other `*_fraction`/ratio setting is bounded.
- **Suggested fix:** Use a name that says depth, e.g. `loops.min_depth_ratio` or `loops.near_depth_fraction`, and bound it to [0, 1) in all three checkers. On placement: global BA and the weld both read it from `loops`. The weld already takes its iterations from `adjustment`, and local BA's guard comes later (`docs/design/2026-10-14-online-slam.md:660-661`). Record in the design whether the setting moves to `adjustment` when local BA adopts it.

##### [note] Each BA's solve settings are split between `*_options` and the call sites
- **Where:** `src/slam_it/edge/loops.py:1609`, `:1640`, `:1677`, `:2307`; `cpp/src/edge/loops.cpp:1997`, `:2326`; `rust/slam-it-edge/src/loops.rs:2836`, `:3676`.
- `global_ba_options`/`weld_options` name the damping and stop, while `near` is read at each call site. The CHANGELOG says "Each BA's solve is named in one place per language", which is only true for the solve options. Keeping `near` out of `graphs.Options` is right, because the pose graph and similarity refinement have no observations. The split is acceptable while both BAs share one setting, and `tests/edge/test_merges.py` pins both together. If the two ever diverge, return the filter alongside the options.
- Placing these functions in `loops` rather than beside `Options.extraction`/`Options.stereo_matching` in `edge.options` (`cpp/include/slam_it/edge/options.hpp:608,614`) is defensible, since only loop closing reads them.

##### [note] Rung 0 now has two near-plane rules and two names for Nielsen damping; the later guard is not tracked
- **Where:** `foundations.ba`, local BA's solver, drops behind-camera observations at `camera.MIN_DEPTH` (`src/slam_it/foundations/ba/layout.py:198`). Its `nls` names Nielsen's constants `_DAMPING_GROWTH`/`max_damping` (`src/slam_it/foundations/nls.py:237-238,493`). `graphs` adds the median-relative `kept_observations` and `SCALED_DAMPING_FACTOR`/`SCALED_DAMPING_LIMIT` (`graphs.py:121,131`).
- No code is duplicated (GTSAM runs the schedule). But "Local BA's near-plane guard is a later PR" (`docs/design/2026-10-14-online-slam.md:660-661`) would put a second implementation of the same rule into `foundations.ba`, in three languages. That PR should share one rung-0 function. It also needs tracking somewhere real (section 7's plan or the ladder's open items); today it is one sentence inside an as-built paragraph.

##### [note] Rung-0 docs name a rung-1 setting and policy
- **Where:** `src/slam_it/foundations/graphs.py:50` ("loop closing's global BA takes both, its welding BA neither") and `:1392` ("loop closing's global and welding BAs pass ``loops.near_fraction``, 1e-3").
- This is a documentation-only upward reference, and it goes stale if edge's choice changes. The C++ and Rust docs do not do it. Describe the behaviour in rung 0 and leave who passes what to `loops.global_ba_options`'s docs, which already say it.

##### [note] Design docs: two gaps in the as-built record
- **Measurement:** The plan item (`docs/design/2026-10-14-online-slam.md:282-286`) and L2 (`:648-649`) say the damping PR is "measured on the loop sequences and the test set". The as-built text (`:651-661`) records no measurement. The CHANGELOG reports the test set and per-call dumps but not the loop sequences (EuRoC/TUM), which carry the bar. The as-built text should say whether they were run. The numbers themselves are the algorithm reviewer's lane.
- **Section 12's deferred item:** That item (`docs/design/2026-10-14-online-slam.md:811-825`) proposes dumping the loop room's shrinking global BA and comparing GTSAM's λ with g2o's. `tests/fixtures/edge/global_ba_slide.json` is exactly that. The ladder's entry was updated (`docs/design/2026-10-02-slam-it-ladder.md:1220-1224`), but this item has no as-built pointer.

#### Checked and fine
- **Dependency direction:** Edge depends only on foundations. No foundations source, test or fixture generator imports edge; `tests/fixtures/foundations/generate_graphs.py` restates the solve as `GLOBAL_BA` and only names the edge function in a docstring. The Rust bridge's `largest_hessian_diagonal` stays inside `slam-it-foundations`. No new packages, so no import-linter or `check_layers.py` contract is needed.
- **`covisibility.connected`:**
  - It sits in the right module, as a graph query next to `links`.
  - It validates once (`maps.as_keyframe_row`) and dispatches by `backend=` (`src/slam_it/edge/covisibility.py:433`). Its Protocol entry is at `:92`.
  - Each native has one binding per module that only converts types, through a shared `links_out` (`cpp/bindings/edge/covisibility.cpp:57-61`, `rust/slam-it-py/src/edge/covisibility.rs:62-70`).
  - It is held to the brute-force `covisibility.json` in all three languages.
  - Each language's `loop_candidates` calls its own core `connected`.
- **The links-versus-connected departure:** It is documented identically in `src/slam_it/edge/loops.py:87-94`, `cpp/src/edge/loops.cpp:317-321`, `rust/slam-it-edge/src/loops.rs:612-616` and section 10, item 2.
- **Solve structure across languages:** Python `_params`, C++ `solve_params`, and Rust choosing (λ0, factor, bound) in Rust and passing plain numbers to a thin bridge. The small-steps counter is the same in all three. The `kept`/`reached`/graph-building refactor is mirrored in C++ and Rust.
- **Error contract:** A linearisation failure on the scaled-damping path ends as `NotConverged` in all three: Python's `try` around `_params`, C++ `solve`'s `try`, Rust's `gtsam_failed`.
- **Validation and bindings:** The new `graphs.Options` fields are validated in each language and cross through `as_dict`, with the key list in `cpp/bindings/foundations/graphs.cpp:37-53` and `rust/slam-it-py/src/foundations/graphs.rs`. `near` is validated once in Python before dispatch and again in each core, as standalone cores must.
- **Settings:** The four new settings appear in all three languages' tables and range checks (Python `_MAY_BE_ZERO`/`_FRACTIONS`; C++ `for_each_setting` and `check_reals`; Rust `named_settings`). The count of 142 is asserted in Python and Rust. `run.json` reads settings as a free dict, so no run-format bump is needed.
- **Culling rule:** `mapping.redundant_observers` is 4 in all three languages, documented in options and in `docs/notes/edge-vo.md`.
- **Fixtures:** `graphs.json` adds the global solve and the filter as parity cases. `global_ba_slide.json` is an input fixture, and its generator states that it is a one-off capture from an out-of-repo dump. The foundations fixture sits under foundations and the edge one under edge.
- **Other rules:** No new runtime dependency, no compatibility shims, and Rust stays within its agreed subset.

### Algorithm review

**Scope:** `git diff 8e9ae9c1...0aa70550` (7 commits, 0.3.17.2 -> 0.3.18.0). I read the changed algorithm code in all three languages: `cpp/src/foundations/graphs.cpp`, `rust/slam-it-foundations/src/graphs.rs` with its `cxx/graphs_bridge.*`, `src/slam_it/foundations/graphs.py`, the edge `loops`, `covisibility`, `database`, `options` and `mapping` culling in each language, and their tests and fixture generators.

I checked the PR against these sources:
- ORB-SLAM3 at 4452a3c: `Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp`, `LocalMapping.cc`, `KeyFrameDatabase.cc`, `KeyFrame.cc`, `LoopClosing.cc`.
- GTSAM 4.3.0: `LevenbergMarquardtOptimizer.cpp`, `internal/LevenbergMarquardtPolicy.h`, `LevenbergMarquardtParams.h`.
- Madsen, Nielsen and Tingleff (2004) for the gain ratio, and Triggs et al. (2000) for gauge freedom.

I checked the measurement files in `../results/g_rule_2026-10-23/`, `../results/f2_split_2026-10-23/` and `../results/wrong_loops_2026-10-24/`. I also ran four read-only probes on the dumped problems through the installed `_cpp` backend, pinned to cores 0-1 and capped at 6 GB. Nothing was built. The commands and outputs are in the findings.

**Summary:** The three changes are faithful to their sources.
- **Damping:** λ0, Nielsen's schedule and Mur-Artal's stop match g2o/ORB-SLAM3 line for line, apart from the departures the PR already documents.
- **Culling rule:** equivalent to ORB-SLAM3's `nObs > 3` in all three languages.
- **Candidate exclusion:** matches `GetConnectedKeyFrames`.

Every CHANGELOG and commit figure I checked is backed by a results file. One reading is wrong. On zju-A1's first dump, the 0.637 "scale" is presented neutrally as a cost decrease. It is not a correction: the non-robust global BA fits the map to about 5k gross outliers, the inliers get much worse, and 17 points end up behind their cameras. The rest are notes: on the L4 test's wording, the near filter's limits, the evidence behind the remaining departure, and where the measured regressions come from.

#### Findings

##### [should-fix] On zju-A1's first dump, global BA lowers its cost by bending the map toward ~5k gross outliers, not by correcting it
- **Where:** `CHANGELOG.md:88` (the "0.637 and 0.842 ... their cost falling" sentence). Mechanism: `src/slam_it/edge/loops.py:1609` (`global_ba_options`, no robust loss, like ORB-SLAM3's `GlobalBundleAdjustemnt(..., bRobust=false)`), and `cpp/src/foundations/graphs.cpp:715` (a point behind its camera gets a constant `(2fx, 2fx)` residual with a zero Jacobian).
- **Issue:** The question was whether 0.637 is a legitimate correction or a gauge slide. It is neither.
  - It is not a gauge slide: the cost falls 63%, and a pure gauge move cannot lower the cost.
  - It is not a correction either: the step serves the outliers. When global BA starts, 4,879 of 73,017 observations (6.7%) have χ² ≥ 1e4 (over ~100 px). The non-robust solve moves the whole map to reduce them, and the 56,421 inliers pay for it.
  - The camera moves are far from a uniform rescale. Cameras' distance from the held keyframe changes by a median factor of 1.21 and a 90th percentile of 4.14. The first and second halves of the trajectory rescale by 0.547 and 0.920.
  - 57 observations of 17 points that started as inliers cross behind their cameras during the solve. There they hold a constant residual with no gradient, so nothing can bring them back. They make up 30% of the final cost the BA reports.
  - The summary still says `function`/converged, so nothing outside the BA can see any of this.
  - The second dump (0.842) does look like a real reconciliation: half of its 100 to 1e4 χ² observations become inliers, and none cross behind.
- **Why it matters:**
  - The final runs agree with the per-call picture. zju-A1 goes from 15.35 to 24.88 cm in C++ and from 15.35 to 86.07 cm in Rust, with Rust's tracked share falling 0.746 -> 0.590 (`../results/g_rule_2026-10-23/connected/compare_mono_{cpp,rust}-inline.md`).
  - In the attribution probe, G alone gives 15.94 cm and the rule alone 27.19 cm, but both together give 89.08 cm (`attrib/table.md`). So the damping does damage on the inconsistent maps the rule produces.
  - Develop's "no step" on these dumps was safe by accident. The branch now acts on them.
- **Evidence:** a probe solving the dump with `loops.global_ba_options(Options())` and `near=1e-3`, on the cpp backend. My recomputed costs match the BA's reported 1.9694e8 -> 7.215e7 exactly.
  ```
  zju-A1 ba_1346388_0 (189 cams, 73,017 obs, none left out)
    start χ²<5.991:   56,421 obs, cost 2.775e4 -> 3.431e7, still <5.991 after: 0.493
    start χ² 1e4-1e5:  3,871 obs, cost 7.685e7 -> 1.290e7
    start χ² ≥1e5:     1,008 obs, cost 1.133e8 -> 1.656e7
    share χ²<5.991 0.773 -> 0.423; median χ² 0.887 -> 16.1
    kept obs that crossed behind during the solve: 57 (17 points), cost 3,924 -> 2.179e7
  zju-A1 ba_1346388_1: share 0.732 -> 0.674; median χ² 1.05 -> 2.58;
    χ² 100-1e4: 15,278 obs, 1.405e7 -> 3.473e5, 51% inliers after; crossed behind: 0
  ```
  I also ran a counterfactual with Huber at √5.991 and the same solve. The inlier share rises (0.773 -> 0.816 and 0.732 -> 0.951), but the camera scale moves to 0.359 and 1.099, and 96 and 30 observations still go behind. A robust loss alone does not fix this; the map handed to global BA is the anomaly.
- **Suggested fix:**
  - In this PR, correct the CHANGELOG and the design's L2 note. 0.842 is a reconciliation. 0.637 fits gross outliers at the expense of inliers, which ORB-SLAM3's non-robust loop BA would also do.
  - Make it observable: report the number of kept observations that end behind, or nearer than the filter, in the BA summary or global BA's log.
  - Backlog:
    - find where zju-A1's 1.97e8 starting cost comes from (the weld, the fuse, or the essential graph);
    - consider a gross-outlier gate before global BA, as ORB-SLAM3's local BA drops `chi2>5.991 || !isDepthPositive` between passes and COLMAP filters observations over 4 px between its global BA refinements;
    - consider treating a camera-plane crossing as a failed trial.

##### [note] The L4 test is meaningful; its stated mechanism is slightly off
- **Where:** `tests/edge/test_loops.py:798-806` ("its cost does not change along it, so nothing but the damping holds it").
- **Issue:**
  - For any λ > 0, each LM step is exactly orthogonal to the exact scale-gauge vector g at its linearisation point. Since Jg = 0, gᵀ(JᵀJ+λI)⁻¹Jᵀr = λ⁻¹(Jg)ᵀr = 0.
  - What slides is a near-flat valley: the cameras and well-constrained points rescale, while far, weakly constrained points barely move. That is gauge plus cost-flat depth modes.
  - The scaled λ0 damps those near-null directions. GTSAM's absolute 1e-5 does not.
- **Evidence:** a probe on `global_ba_slide.json`:
  ```
  G (scaled):     cost 1749.68 -> 481.336; Umeyama cams 0.9968, points 0.9994
  GTSAM absolute: cost 1749.68 -> 481.301; Umeyama cams 0.7322, points 0.9114
  baseline/median-depth ratio 0.01109 -> 0.01378 in BOTH (gauge-invariant)
  absolute, 1/2/3/5 it: cams 0.9509 / 0.9265 / 0.8996 / 0.8434
  ```
  Both solves make the same gauge-invariant correction (ratio 0.01109 -> 0.01378) and end at costs 7.3e-5 apart. The 0.7322 is pure drift. So the test pins exactly the right property: no mid-run scale jump. Its `< 0.8` counter-assertion and the 1e-3 cost check make it discriminating.
- **Suggested fix:**
  - Optionally, reword the docstring and also assert that the two solves' baseline/median-depth ratios agree, which proves the difference is drift.
  - Context: the free gauge held by LM damping is standard practice. Zhang, Gallego and Scaramuzza found free gauge, gauge fixation and a gauge prior equally accurate (RA-L 2018).
  - A drift-free alternative would be to rescale the solution back about the held camera after the solve. That is exact, since a monocular BA's cost is invariant to it, but it departs from ORB-SLAM3. Backlog only.

##### [note] Limits of the near filter
- **Where:** `src/slam_it/foundations/graphs.py:1208`, `cpp/src/foundations/graphs.cpp:1079-1100`, `rust/slam-it-foundations/src/graphs.rs:1656-1680`.
- **Issue:**
  1. **Scope:** it acts only at the start. Points can still cross the camera plane mid-solve (previous finding), where GTSAM's constant-residual convention catches them.
  2. **Median:** it is one median over every observation. ORB-SLAM's own depth statistic is per keyframe (`KeyFrame::ComputeSceneMedianDepth`), which would be insensitive to a merged map's parts having different scales. At 1e-3 this rarely matters.
  3. **Negative median:** if more than half the depths are ≤ 0, the limit drops to ≤ 0 and only `z > 0` is tested.
  4. **Unreached points and cameras:** they come back unchanged, in the pre-BA frame, while the rest of the map may move (0.637 above). It is at most 125 of 241,436 observations (advio-19).
  5. **Weld:** the filter is harmless there and consistent with the weld's post-solve behind-camera check, which `forget_observations` acts on. But its effect on welds is unmeasured: all five dumped welds are identical with and without it (`../results/f2_split_2026-10-23/cpp/table.md`, "exclude" against "exclude+near1e-3").
- **Evidence:** a probe confirms the CHANGELOG's range.
  - tum-freiburg2_360_hemisphere: min z/median 1.6e-8, approx. largest point diagonal 7.4e20, 9.2e10 after the filter.
  - advio-20: 7.3e13 -> 1.4e12.
- **Suggested fix:** for the record or backlog, say in the docstring that the filter acts at the start only. Consider per-keyframe medians.

##### [note] The remaining departure (near check on 15-point links) rests on one sequence each way
- **Where:** `src/slam_it/edge/loops.py:87-94`; `docs/design/2026-10-14-online-slam.md:476-483`; C++ `linked_rows` (`cpp/src/edge/loops.cpp:326`); Rust `linked` (`rust/slam-it-edge/src/loops.rs:631`).
- **Issue:**
  - ORB-SLAM3 aborts a candidate whose group holds any keyframe connected to the query (`LoopClosing.cc:587, 628-637`), and does the same at 922/932 and 1152.
  - The probe comparing full ORB-SLAM3 (F1) with retrieval-only (F1r, as built) covers 20 sequences, C++ only, one seed. It shows equal medians (99.04 cm), sums of 7,842.6 against 7,787.3 cm, 5 against 7 loops, and 1 against 1 wrong-place event.
  - The difference is essentially arkitscenes-42446137 (90.70 against 29.60) set against zju-A1 (18.93 against 24.88) (`../results/wrong_loops_2026-10-24/f1r_all/table.md`).
  - Mechanically, the departure lets through groups whose member shares 1 to 14 points with the query. That is a weaker form of the very aliasing this PR fixes in retrieval.
- **Suggested fix:** keep it, since the owner decided it and it is documented. Backlog: count, across the full test set and in Rust, how often the near check passes a group with a connected-but-unlinked member, and whether those loops land at the right place.

##### [note] Measured regressions: the CHANGELOG does not say which change causes them
- **Where:** `CHANGELOG.md` "Measured" bullets.
- **Issue:** All figures check out (list under "Checked and fine"). The entry does not say:
  - **Monocular inline (attribution probe):** the regressions on the 20 probed sequences are the culling rule's, and G alone improves.
    - Per sequence: advio-03 117.79 -> 313.48 (rule alone) against 104.69 (G alone); advio-19, -21, -10 and zju-B4 regress under the rule alone, and G alone equals develop.
    - Summed ATE over the 20: develop 5,363.6, rule alone 7,025.0, G alone 4,617.0, both 7,918.6.
    - Sources: `../results/g_rule_2026-10-23/attrib/table.md`, `../results/wrong_loops_2026-10-24/evidence/regressions.txt`.
  - **Threaded:** the candidates' change itself coincides with a large median shift: C++ 15.30 -> 20.46 and Rust 19.30 -> 29.95 (`connected/compare_mono_rule_vs_fix_*-thr.md`). That exceeds the ~20% run-to-run spread the entry cites, though these are single runs.
  - **RGB-D:** the regression (median 4.33 -> 5.37 cm, and arkitscenes-42444946's merge at 1.77 m / 166°) is not attributed to any of the four changes.
- **Suggested fix:** add one attribution sentence per regression, so the owner's trade-off for the rule (starvation fixed, ATE cost on the inline set) is explicit. Repeat the threaded runs before reading anything into them.

##### [note] State of the art: diagonal (Marquardt) damping would remove the need for the filter's damping role
- **Where:** `src/slam_it/foundations/graphs.py:610` (`_params`), `cpp/src/foundations/graphs.cpp:407`.
- **Issue:**
  - g2o's damping, λI with λ0 = τ·max diag(H), lets one degenerate observation set the damping for every variable. That is exactly the 1e13 to 1e23 failure the filter patches.
  - Marquardt's scaling, λ·diag(JᵀJ), damps each variable in proportion to its own curvature. It is what Ceres uses, with the diagonal clamped to [1e-6, 1e32], and what GTSAM offers as `diagonalDamping = true`.
  - The rung's goal is ORB-SLAM3 fidelity, so this is context, not a defect.
- **Evidence:** Marquardt, "An Algorithm for Least-Squares Estimation of Nonlinear Parameters", SIAM J. Appl. Math. 11(2), 1963; Ceres `solver.h` (`min_lm_diagonal`/`max_lm_diagonal`, https://ceres-solver.googlesource.com/ceres-solver/+/1.14.0/include/ceres/solver.h); g2o's scheme follows Madsen, Nielsen and Tingleff, "Methods for Non-Linear Least Squares Problems", IMM DTU, 2004 (http://www2.imm.dtu.dk/pubdb/edoc/imm3215.pdf).

##### [note] Culling now uses ORB-SLAM3's threshold but still tests only the 15-point links
- **Where:** `src/slam_it/edge/mapping.py:1186` (`covisibility.links`); documented in `src/slam_it/edge/covisibility.py`.
- **Issue:** ORB-SLAM3 tests every covisible keyframe, up to 100 (`LocalMapping.cc:910, 1060`). With the stricter rule, this pre-existing departure compounds it: even more keyframes survive than ORB-SLAM3 would keep. Peak RSS on advio-06 goes 1,497 -> 3,478 MiB. The denser maps are what produced the 1-to-14-point candidates.
- **Suggested fix:** backlog. The ORB-SLAM3 anchor runs in progress can compare keyframe counts directly.

##### [note] Smaller items
- **All observations left out:** `bundle_adjust` returns `converged`/`function` with 0 iterations (`src/slam_it/foundations/graphs.py:1275`). That is observable via `iterations == 0`, but the error contract has `Degenerate` for exactly this.
- **Unvalidated public `kept_observations`** (hygiene reviewer's lane):
  - Python `graphs.kept_observations` validates nothing; a NaN `near` silently keeps every observation in front.
  - C++ `kept_observations` (`cpp/include/slam_it/foundations/graphs.hpp`) indexes `poses[cameras[m]]` unchecked.

#### Checked and fine
- **λ0:** g2o's `computeLambdaInit` (`optimization_algorithm_levenberg.cpp:171-185`) takes the max |H_jj| over `indexMapping()`, which holds the non-fixed vertices only, poses and marginalised points alike.
  - C++/Rust `hessianDiagonal()` and Python's whitened-Jacobian column sums cover exactly the free cameras and points the kept observations reach. Held cameras are excluded, and a held camera's edge feeds its point only, as in g2o.
  - g2o's left update on T_cw and GTSAM's right update on T_wc give the same Jacobian magnitudes, ([X_c]×, ∓I).
  - GTSAM's ½ in its error leaves H = JᵀJ unchanged, so λ0 is numerically g2o's.
  - It is computed at the start values, as g2o does at iteration 0.
- **Nielsen schedule:** GTSAM 4.3.0's `increase/decreaseLevenbergMarquardtLambda` (`internal/LevenbergMarquardtPolicy.h:29-55`): λ·=ν then ν·=2 on a failed trial; λ·=max(1/3, 1-(2ρ-1)³) and ν=2 on an accepted step. That is g2o's `_ni`, without the 2/3 cap.
  - The gain ratios agree: g2o's `computeScale` = xᵀ(λx+b) is GTSAM's `linearizedCostChange` expressed in χ² units, except for g2o's +1e-3.
  - The documented departures are accurate: the 2/3 cap, `minModelFidelity` 1e-3 against ρ > 0, and no 10-trial cap, so 13 trials reach 1e32 from 1e6 (2^91).
- **Stop:** matches lines 157-166 in every language. It is measured against the iteration's starting χ², strict `<`, the count persists across iterations and resets on a large step, and the third step is kept. The relative-tolerance check precedes it harmlessly.
- **Test emulation:** the damping-schedule test emulates GTSAM's search (fails at λ0, 2λ0 and 8λ0, then steps at 64λ0) and holds every backend to 1e-9. The stop test pins the reset.
- **Near filter, cross-language:** the median is the floor(M/2)-th smallest, with the same summation order. Behind means depth ≤ 0. The threshold is scale-invariant, which suits monocular maps. Tests cover near, behind, every observation left out, and bad `near`.
- **Culling:** our `others ≥ 4` is ORB-SLAM3's `nObs > 3` (`LocalMapping.cc:997, 1001`). `weight > 4` never decides, since five observing keyframes weigh at least 5. Level ≤ level+1, and close-depth points only for stereo/RGB-D, in all three languages (`mapping.py:1166`, `cpp/src/edge/mapping.cpp:641`, `rust/slam-it-edge/src/mapping.rs:1111`).
- **`covisibility.connected`:** equals `mConnectedKeyFrameWeights = KFcounter` (`KeyFrame.cc:461`; `GetConnectedKeyFrames` at 228-235), computed live. `ranked` excludes 0-weight rows, so the links fallback cannot add a non-sharer. The candidate exclusion matches `KeyFrameDatabase.cc:613, 626`. The 1-to-14-point tests and the brute-force fixture are sound.
- **Citations and wiring:** every ORB-SLAM3 line citation in the commits, CHANGELOG and design is correct (`LoopClosing.cc:587/628/637/922/932/1152`, `LocalMapping.cc:966/997/1001`). Every global BA site (loop, merge, threaded) goes through `global_bundle_adjust`.
- **Figures verified against the results files:**
  - Per call: 14/24 no step, 18 at the damping limit; 24/24 stepping, 12 converged and 12 at max iterations; C++ and Rust agree to 3.31e-12; 22 dumps within 0.974 to 1.004; zju-A0 4e-4; at most 125 left out; 0.27 to 51 s against 0.15 to 28 s.
  - Every aggregate in the inline/threaded × mono/RGB-D tables.
  - rule+G without the candidates' change: loops 24/29, wrong 9/9, threaded 15.30/19.30.
  - The merges' step times and RSS figures.
  - The near-check probe's 29.6/90.7 and 24.9/18.9.
  - The wrong-place geometry (0.1 to 0.5 m claimed against 7.1 to 9.8 m true).
  - The L4 fixture's 0.7322/0.9968/7.3e-5, which I reproduced.
  - Layer 1 ran on 97de38f3, which differs from HEAD only in `CHANGELOG.md`.

Sources:
- [ORB-SLAM3 modified g2o, `optimization_algorithm_levenberg.cpp`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp) (local copy at 4452a3c)
- [Zhang, Gallego and Scaramuzza, "On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation", IEEE RA-L 3(3):2710-2717, 2018](https://www.research-collection.ethz.ch/handle/20.500.11850/334931?show=full)
- [Ceres Solver 1.14.0 `solver.h` (min/max_lm_diagonal)](https://ceres-solver.googlesource.com/ceres-solver/+/1.14.0/include/ceres/solver.h)
- [COLMAP incremental mapper options (filter_max_reproj_error 4.0, ba_global_max_refinements 5), mirror](https://huggingface.co/camenduru/ceres-solver/blob/main/include/colmap/sfm/incremental_mapper.h)
- [Madsen, Nielsen and Tingleff, Methods for Non-Linear Least Squares Problems, 2004](http://www2.imm.dtu.dk/pubdb/edoc/imm3215.pdf)
