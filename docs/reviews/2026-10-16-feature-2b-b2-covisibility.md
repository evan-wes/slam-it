# Review panel: `feature/2b-b2-covisibility`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-16

| | |
|---|---|
| **Branch** | `feature/2b-b2-covisibility` |
| **Range** | `9a4f499...390d67d` (base: `origin/develop`; merge base `9a4f499`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed (`scripts/check.sh` in the `slam-it` env, exit 0: 25 checks; pytest 4197 passed, 13 deselected) |

### Findings and resolutions

Two reviewers found the new settings left at their defaults in a whole run (H3, A2), the native
`erase_keyframe` leaving a dangling parent (A1, L3), the Python reference's KxK count (H7, A8)
and the old names (H4, H8, A5); all three found the summary's `base - 1` (H11, A11, L2). Each is
decided once, under its first ID.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `edge/mapping.py` `cull_keyframes`; C++ and Rust twins | On a map with no keyframes, Rust panics, C++ reads out of bounds, Python raises an undocumented error | fix (`check_newest` in the three; the empty case tested in each) | fixed (c13a438: `check_newest` in the three; the empty case tested in each) -- decided by evan-wes, 2026-10-16 |
| H2 | must-fix | hygiene | `eval/runs.py` `_check_parents`, `_check_links`; `tests/eval/test_runs.py` | Python's run-format-6 checks and round trip are untested, though the CHANGELOG says all three writers refuse a bad parent or link | fix (each refusal branch, write and read side; parents and links in the round trip) | fixed (7ce41f2: 14 write cases, both on reading, the round trip) -- decided by evan-wes, 2026-10-16 |
| H3 | must-fix | hygiene | `tests/orchestration/datasets.py` `MOVED_OPTIONS` | B2's new settings are never moved off their defaults in a whole run (B1's H21 rule) | fix (the fusion and covisibility settings moved, with the decisions still changing) | fixed (7ce41f2: the seven moved; each of fusion's three counts, `min_points` and `strong_points` changes the room's decisions put back alone, the local map's two caps cannot there and the fixture holds them) -- decided by evan-wes, 2026-10-16 |
| H4 | should-fix | hygiene | `edge/relocalization.py`; edge and orchestration tests (Py, C++) | Comments and test labels still name flat settings, against 902b9a4 and the CHANGELOG | fix (renamed) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| H5 | should-fix | hygiene | `edge/options.py` `__post_init__` | A string or `None` for `features.n_features` raises `TypeError`, not `InvalidInputError` | fix (the fields checked before `extraction()`; a test case) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| H6 | should-fix | hygiene | `edge/map.py` parent check; `need_keyframe`'s `reference`; `relocalize_locally`'s `last` (three languages) | New checks without a refusal test | fix (a case per check in the three suites) | fixed (7ce41f2) -- decided by evan-wes, 2026-10-16 |
| H7 | should-fix | hygiene | `edge/covisibility.py` `_weights` | The Python reference counts the whole KxK matrix to read one keyframe's row | fix (one row counted, as the cores do; the full matrix for `strong_links` only) | fixed (64afa25: `links` 194.3 to 3.3 ms, `local_map` 195.9 to 86.7 ms at 500 keyframes on the reviewer's map) -- decided by evan-wes, 2026-10-16 |
| H8 | should-fix | hygiene | the pipelines' and programs' vocabulary errors (three languages) | Errors name `relocalize`, a setting that no longer exists | fix (`relocalization.enabled`, and the tests matching them) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| H9 | should-fix | hygiene | `tests/edge/test_database.py` | The "no covisibility" case no longer disables it, and frame 25 scores one keyframe, so the group sum is not tested | fix (relabelled; a frame scoring two linked keyframes, strictly higher) | fixed (7ce41f2) -- decided by evan-wes, 2026-10-16 |
| H10 | should-fix | hygiene | `edge/covisibility.py` private functions | No one-line docstrings; `_decode_map` wraps `maps.decode` for nothing | fix | fixed (64afa25) -- decided by evan-wes, 2026-10-16 |
| H11 | note | hygiene | `LocalTracking.summary` (three languages) | Records `base - 1` when there is no reference keyframe (also A11, L2) | fix (-1 kept) | fixed (db9dd1f; tested in the three) -- decided by evan-wes, 2026-10-16 |
| H12 | note | hygiene | Rust `covisibility::local_map`, `runs.rs` `check_links`; `tracking.py`; `tests/fixtures/eval/generate_run.py` | Repetition, a vague name, the run fixture's flat settings | fix | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| H13 | note | hygiene | `viewer/src/bundle.js` | Parents' and links' ids not range-checked on load (the bake checks them) | fix (checked against K) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| A1 | must-fix | architecture | `map.erase_keyframe` (three languages); culling | The spanning tree's invariant has no owner: native `erase_keyframe` leaves a dangling parent silently, Python raises (also L3) | fix (`erase_keyframe` refuses a keyframe that is still a parent, documented and tested in the three; a single SetBadFlag helper when B3 brings a second caller) | fixed (c13a438: `covisibility.cull` added now in the three, as evan-wes chose, the one way a keyframe leaves the map, refusing the map's first; `erase_keyframe` refuses a parent in the three) -- decided by evan-wes, 2026-10-16 |
| A2 | must-fix | architecture | `MOVED_OPTIONS` | Same as H3 | see H3 | fixed with H3 (7ce41f2) -- decided by evan-wes, 2026-10-16 |
| A3 | should-fix | architecture | `covisibility.adopt`; `tests/edge/test_covisibility.py` | Nothing holds the tree to being a tree: `adopt` on an older keyframe can close a cycle, and the room test does not walk the chains | fix (`adopt` documented as ORB-SLAM's first `UpdateConnections`, for a keyframe being inserted; the room's and golden record's chains walked to the root) | fixed (c13a438: `adopt`'s docs, the room's chains; 7ce41f2: every map of the golden record) -- decided by evan-wes, 2026-10-16 |
| A4 | should-fix | architecture | `tests/fixtures/edge/covisibility.json` | States no oracle or tolerance policy | fix (both stated); a check over every fixture deferred to the ladder's open items | fixed (7ce41f2); the check over every fixture deferred (the ladder's open items, 471f7fd) -- decided by evan-wes, 2026-10-16 |
| A5 | should-fix | architecture | the vocabulary errors; `relocalization.py`; `runs.hpp`, `runs.rs` | Old names (H8, H4), and the C++ and Rust format notes stop at version 5 | fix (version 6 described in both) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| A6 | should-fix | architecture | the pipelines' `close` (three languages) | The lost map's strong-links record is a stand-in until B4, not marked as one | fix (one sentence each) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| A7 | note | architecture | the viewer's graph | The graph shown is the map's last, drawn through time | fix (said on the toggle and in the bundle's docs); a graph over time deferred to the backlog | fixed (db9dd1f); the graph through time deferred (the ladder's open items, 471f7fd) -- decided by evan-wes, 2026-10-16 |
| A8 | note | architecture | `edge/covisibility.py` | Same as H7 | see H7 | fixed with H7 (64afa25) -- decided by evan-wes, 2026-10-16 |
| A9 | note | architecture | `scripts/compare_testsets.py`; the note's conditions | The script cannot read B1's format-5 runs, so the note's B1 half was not made as it says | fix (the note says where B1's half comes from); comparing by each batch's own summary taken to B3's design | fixed (471f7fd: the note); comparing by each batch's summary deferred (B3's outline, 471f7fd) -- decided by evan-wes, 2026-10-16 |
| A10 | note | architecture | `edge/options.py` `_GROUPS`, `__post_init__` | `_GROUPS` repeats `Options`' fields; normalising writes into shared frozen groups | fix (`_GROUPS` from `fields(Options)`); waive the write (it stores the same values) | fixed (db9dd1f: the groups from `Options`' fields); the normalising write waived: it stores the values it checked -- decided by evan-wes, 2026-10-16 |
| A11 | note | architecture | `LocalTracking.summary` | Same as H11 | see H11 | fixed with H11 (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| A12 | note | architecture | `edge/__init__.py` | The rung's budget line says nothing of mapping's measured load | fix (a clause pointing to the ladder's item) | fixed (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| L1 | should-fix | algorithm | `tests/edge/test_mapping.py`; the C++ and Rust mapping tests | How each stage uses the graph has no known-answer test; the keyframe-decision test passes B1's reference | fix (fusion's targets, local BA's free and fixed, a cull with children, two references deciding differently; the tree property over the golden record) | fixed (7ce41f2: `fuse_targets` and `local_ba_keyframes` public and held to the fixture in every backend and natively; two references deciding both ways; the golden record's trees) -- decided by evan-wes, 2026-10-16 |
| L2 | should-fix | algorithm | `LocalTracking.summary` | Same as H11 | see H11 | fixed with H11 (db9dd1f) -- decided by evan-wes, 2026-10-16 |
| L3 | should-fix | algorithm | `map.erase_keyframe` | Same as A1 | see A1 | fixed with A1 (c13a438) -- decided by evan-wes, 2026-10-16 |
| L4 | note | algorithm | `edge/covisibility.py` docstring | ORB-SLAM's source counts weak links when it re-sorts (SetBadFlag, ORB-SLAM3's culling, capped at 100) | fix (a line in the docstring; the cap of 100 named in B3's outline) | fixed (db9dd1f: the docstring; 471f7fd: the cap of 100 in B3's outline) -- decided by evan-wes, 2026-10-16 |
| L5 | note | algorithm | `edge/mapping.py` local BA | No guard for a local BA with no fixed keyframe (ORB-SLAM3 abandons it); 19% of monocular-heavy BAs have one | defer (B3's design, with the mapping threads) | deferred (B3's outline, the online-SLAM design's section 4, 471f7fd) -- decided by evan-wes, 2026-10-16 |
| L6 | note | algorithm | B2's local map | The paper's K2 makes a larger local map than ORB-SLAM's source does | fix (named in B3's outline, for its latency comparison) | fixed (471f7fd: B3's outline) -- decided by evan-wes, 2026-10-16 |
| L7 | note | algorithm | — | State of the art: the graph is still current sparse SLAM's; learned rungs use frame graphs | waive: context for later rungs | waived: context for later rungs -- decided by evan-wes, 2026-10-16 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `9a4f499...390d67d` (feature/2b-b2-covisibility), all seven commits. I read the new covisibility module and its C++/Rust/PyO3/nanobind twins in full, plus the diffs of options, map, mapping, tracking, relocalization, database, pipeline, the run format, bundles, vo, the viewer, the tests, fixture generators, CHANGELOG, docs and notes. Paths below are relative to the repository root. Probe scripts are in `/tmp/scratch/hyg/`. `git status` is clean. Importing the editable `slam_it._cpp` for one probe triggered a cmake rebuild of the git-ignored `build/` tree.

**Summary:** The covisibility code is well documented. The three languages are held to an independent brute-force fixture. Options validation, the native map checks and the viewer geometry are sound. The CHANGELOG and note numbers I summed are consistent. Three things are not production quality yet. `cull_keyframes` on a map with no keyframes is a Rust panic, a C++ out-of-bounds read, and a confusing Python error. The new Python run-format validators have no refusal test. The seven new settings are never moved off their defaults in the whole-run native-vs-Python parity test, which is the H21 rule from B1's panel. The CHANGELOG also claims the comments name every setting `group.field`, and that is not true.

### Findings

#### [must-fix] `cull_keyframes` on a map without keyframes: Rust panics, C++ reads out of bounds, Python gives a confusing error
- **Where:** `src/slam_it/edge/mapping.py:971-974` and `:990`, `cpp/src/edge/mapping.cpp:909-915`, `rust/slam-it-edge/src/mapping.rs:1107-1110`.
- **Issue:** Each language now takes the newest keyframe as `keyframes - 1` and reads its links. Unlike `need_keyframe`, `triangulate_new_points` and `fuse_points`, `cull_keyframes` calls only `check_setup`, not `check_newest`. With zero keyframes, Rust computes `usize` 0 - 1. C++ passes `size_t(-1)` to `shared_points`, which indexes `kf_start[SIZE_MAX]`. Python's `as_keyframe_row` raises `k: must be from 0 to -1, got -1`, which the docstring's Raises section does not list. Before B2 the window of an empty map was empty, so all three languages did nothing.
- **Why it matters:** A public function now has three different behaviours on the same degenerate input. In the Rust extension the failure is a `PanicException`, which derives from `BaseException` and escapes `except Exception`. The C++ read is undefined behaviour, and libstdc++ assertions would abort. The pipeline cannot reach it, because `process` inserts a keyframe first, but the public boundary is unguarded and untested.
- **Evidence:** `/tmp/scratch/hyg/empty_cull.py` calls `mapping.cull_keyframes(maps.empty(8, 1.2), cam, "monocular", Options(), backend=b)`. Output: `python InvalidInputError k: must be from 0 to -1, got -1`, `cpp returned 0`, `rust PanicException index out of bounds: the len is 1 but the index is 18446744073709551615`. `tests/edge/test_mapping.py:253-272` (`test_rejects_malformed_arguments`) has no `cull_keyframes` case, and neither do the C++ or Rust mapping tests.
- **Suggested fix:** Use `check_newest` (or an explicit "map has no keyframes" InvalidInput) in the three `cull_keyframes`. Add the empty-map case to the three test suites and to the Python docstring's Raises.

#### [must-fix] Run format 6's Python validators and round trip are untested
- **Where:** `src/slam_it/eval/runs.py:410-449` (`_check_parents`, `_check_links`, called at `:628-629` and `:767-776`), `tests/eval/test_runs.py`.
- **Issue:** `_check_parents` has four refusal branches: shape or dtype, out of range or self, not the same map. `_check_links` has five: shape or dtype, order or range or weight below 1, across maps, not ascending. Python has no test for any of them. C++ (`cpp/tests/eval/test_runs.cpp:402-426`) and Rust (`runs.rs:1497`, `bad_parents_and_links_are_refused`) do. `test_round_trip` (`tests/eval/test_runs.py:77-104`) never compares `keyframe_parents` or `keyframe_links` after read, and nothing tests `read_run` on a bad `keyframe_parents.npy`.
- **Why it matters:** The Python writer is the reference for the format. A parent in another map, or an unsorted link list, would be written or read silently. The CHANGELOG's "the run format's writers refuse a bad parent or link in the three languages" is false for Python.
- **Evidence:** `git grep -n "parents\|links" tests/eval/test_runs.py` at HEAD shows only the `_run()` constructor lines 64-65 and the file listing at 555-558. The existing refusal tests (`test_write_run_checks_everything_before_writing`, `test_read_run_checks_the_lives`, `test_read_run_checks_the_maps`) cover lives and maps only.
- **Suggested fix:** Add parametrized cases for each branch to the write-side refusal test and a read-side test. Add parents and links to the round trip.

#### [must-fix] B2's seven new settings are not moved off their defaults in the whole-run parity test (the H21 rule)
- **Where:** `tests/orchestration/datasets.py:134-170` (`MOVED_OPTIONS`), `tests/orchestration/test_vo.py` (`test_the_backends_agree_off_the_defaults`).
- **Issue:** `mapping.fuse_neighbours`, `mapping.mono_fuse_neighbours` and `mapping.fuse_second_neighbours` are never set to a non-default value in any test. They appear only in the options tables and in the generic "refuses a bad value" loop. `covisibility.min_points`, `local_keyframes` and `local_neighbours` are non-default only in the module-level fixture cases (`covisibility.json`). They are not moved in the whole-run test either.
- **Why it matters:** B1's panel made this a must-fix (L1, then H21): "a native that hard-coded a setting would agree at the defaults". C++ `fuse_targets` (`cpp/src/edge/mapping.cpp:~70-100`) or Rust `mapping.rs:995-1001` could hard-code 10, 20 or 5 and every test would pass. `MOVED_OPTIONS` already moves `mapping.triangulation_neighbours` and `mapping.mono_triangulation_neighbours`, the analogous settings.
- **Evidence:** The `MOVED_OPTIONS` dict at HEAD lists 33 settings, none of the seven. `git grep -E "fuse_neighbours|fuse_second_neighbours" -- tests cpp/tests rust/slam-it-edge/tests` returns no hits outside the options tables.
- **Suggested fix:** Add the seven to `MOVED_OPTIONS` (for example `mapping.fuse_neighbours: 6`, `mapping.mono_fuse_neighbours: 12`, `mapping.fuse_second_neighbours: 3`, `covisibility.min_points: 12`, `local_keyframes: 20`, `local_neighbours: 6`). Confirm `test_the_moved_settings_change_the_decisions` still sees a difference, and regenerate nothing else.

#### [should-fix] The comments still carry flat setting names, against the CHANGELOG and commit 902b9a4
- **Where:** `src/slam_it/edge/relocalization.py:16`, `:18`, `:238`, `:522` (`reloc_min_inliers`, `reloc_inlier_ratio`, `The reloc_* settings`), `tests/edge/test_relocalization.py:74`, `:87`, `:116` (`reloc_min_matches`, `reloc_min_inliers`, `reloc_narrow_inliers`), `tests/edge/test_pipeline.py:122` (`init_feature_factor`), `cpp/tests/edge/test_initialization.cpp:105` (message "init_min_matches"), `cpp/tests/edge/test_map.cpp:195-197` ("rejects an empty window", for `covisibility.local_neighbours = 0`).
- **Issue:** The CHANGELOG says "The comments name every setting `group.field`, as the code does, and none describes the window". Commit 902b9a4 says the Python tests' docstrings were done. `relocalization.py` lines 16-18 and 238 sit in paragraphs that this PR rewrapped, and they kept the old names. Bare names (`lost_frames`, `reset_keyframes`, `relocalize_seconds`, `recovery_tracked`) also remain in `tests/edge/test_pipeline.py:154,168,170,202,235,249,337`, `cpp/tests/edge/test_pipeline.cpp:183,248-249` and `tests/edge/test_relocalization.py:37`.
- **Why it matters:** A reader searching for `relocalization.min_inliers` misses the docs that explain it, and the committed record claims a cleanup that did not happen.
- **Evidence:** `git grep -n -P "(?<![\w.])reloc_[a-z_*]+" 390d67d -- src tests cpp rust` lists the lines above.
- **Suggested fix:** Rename these, or soften the CHANGELOG claim.

#### [should-fix] `Options(features=FeatureOptions(n_features="a"))` now raises `TypeError`, not `InvalidInputError`
- **Where:** `src/slam_it/edge/options.py:566-570` (`__post_init__` calls `self.extraction()` before the per-field `_checked` loop), `:651-652` (`features.n_features * factor`).
- **Issue:** The old `__post_init__` passed the raw fields to `orb.Options`, which validates them. The new `extraction()` multiplies and applies `min()` to the unchecked value first.
- **Why it matters:** A string or `None` for a public settings field escapes the error contract (`InvalidInputError` is documented).
- **Evidence:** `Options(features=FeatureOptions(n_features="a"))` gives `TypeError '<' not supported between instances of 'int' and 'str'`. `None` gives `TypeError unsupported operand type(s) for *: 'NoneType' and 'int'`. `orb.Options("a", ...)` gives `InvalidInputError n_features: must be an integer, got str`. `float('nan')` and `True` are refused properly.
- **Suggested fix:** Run the per-field checks first, or check `features.n_features` before `extraction()`. Add a case to `test_settings_out_of_range_are_refused`.

#### [should-fix] New invariants and arguments have no refusal test
- **Where:** the `kf_parent` check in `src/slam_it/edge/map.py:678-680`, `cpp/src/edge/map.cpp:324-330`, `rust/slam-it-edge/src/map.rs:432-441`. The `reference` argument of `need_keyframe` (`src/slam_it/edge/mapping.py:389`, `cpp/src/edge/mapping.cpp:~699`, `rust/slam-it-edge/src/mapping.rs:153`). The `last` argument of `relocalize_locally` (`tracking.py:1120`).
- **Issue:** `test_a_map_checks_its_structure` (`tests/edge/test_map.py:233-252`) checks `pt_id`, `kf_start`, `kp_point`, `gone` and `gone_pt_how`, but not a parent that is the keyframe itself or is not in the map. No test passes a bad `reference` (negative or past the last row) to `need_keyframe` in any language, and none passes bad ids as `last`.
- **Why it matters:** The new `reference` argument, bounds-checked in every language, makes the keyframe decision read an arbitrary keyframe's keypoints. The checks exist, but only a test keeps them.
- **Suggested fix:** Add one parametrized case per check in the three suites.

#### [should-fix] The Python reference counts the whole K×K weight matrix to read one row
- **Where:** `src/slam_it/edge/covisibility.py:102-118` (`_weights`), `:134-137` (`_links`), `:151` (`_local_map`), `:179` (`_adopt`). Callers: `mapping.py` `_fuse_targets` (1 + up to 20 `links` calls per keyframe), `_triangulate_new_points`, `_cull_keyframes`, `_process`, and `database._best` (one call per scored keyframe).
- **Issue:** The C++ and Rust twins count one row (`shared_points(m, k)`). The Python `_links` builds a K×K int64 matrix and takes `[k]`. "Counted when asked" does not require counting every pair.
- **Why it matters:** The Python backend is the reference and is run in tests and budgets. The cost and memory grow with K². On a synthetic path map of about 360 observations per keyframe, `links()` for the newest keyframe takes 13 ms at 50 keyframes, 72 ms at 200 and 196 ms at 500. `local_map` takes 17, 79 and 189 ms, and is called once per tracked frame. The largest test-set map holds 2,244 keyframes, a 40 MB matrix per call.
- **Evidence:** `.../scratchpad/hyg/time_links.py` (run with `python -I`).
- **Suggested fix:** Give `_weights` a row variant: the observers of keyframe `k`'s points via one `np.isin` and a bincount. Keep the full matrix for `strong_links` and `reparent`.

#### [should-fix] Pipeline errors name a setting that no longer exists
- **Where:** `src/slam_it/edge/pipeline.py:438,440`, `src/slam_it/orchestration/vo.py:231,233`, `cpp/src/edge/pipeline.cpp:81,84`, `cpp/src/orchestration/vo.cpp:819,822`, `rust/slam-it-edge/src/pipeline.rs:357,362`, `rust/slam-it-orchestration/src/vo.rs:998,1001`.
- **Issue:** `"vocabulary: relocalize needs one"` and `"given, but relocalize is off"` refer to `relocalize`, which is now `relocalization.enabled`. A user who passed `--option relocalization.enabled=false` is told about a setting they never named. The tests pin the old wording.
- **Suggested fix:** Say `relocalization.enabled` in the three languages' messages and in the tests that match them.

#### [should-fix] The database test's "no covisibility" case no longer disables covisibility, and frame 25 does not exercise the group sum
- **Where:** `tests/edge/test_database.py:89-107` (`test_a_group_adds_its_scored_neighbours`), `:109-120` (the `"no covisibility"` id).
- **Issue:** `covisibility.min_points = 1 << 20` used to leave a keyframe with no covisible neighbours. Under the new rule each keyframe keeps its single best link.
- **Evidence:** `.../scratchpad/hyg/scratch_tests/test_probe.py` shows that at `min_points = 2**20` keyframe 0 has links `[1]`, keyframe 1 `[2]`, keyframe 2 `[3]` and keyframe 3 `[2]`. For frame 25, `candidates` scores a single keyframe (`scored 1`, candidates `[2]`). So `score == own` and `grouped >= alone` hold with equality, and the test does not check that a group adds its scored neighbours.
- **Suggested fix:** Relabel the parametrized id. Choose a frame, or a lowered `relocalization.common_words`, that scores two linked keyframes, and assert that the grouped score is strictly higher.

#### [should-fix] The Python reference functions in `covisibility.py` lack docstrings
- **Where:** `src/slam_it/edge/covisibility.py:134` `_links`, `:140` `_local_map`, `:176` `_adopt`, `:187` `_reparent`, `:213` `_strong_links`, `:248` `_decode_map`.
- **Issue:** `mapping.py` and `tracking.py` give every private reference function a one-line docstring ("... (reference)"), and CLAUDE.md says "documented, everywhere". Ruff does not require them for private names. `_decode_map` also wraps `maps.decode` with the same signature, and `mapping.py` passes `decode=maps.decode` directly.
- **Suggested fix:** Add one-line docstrings and drop the wrapper.

#### [note] `LocalTracking.summary` records `base - 1` when there is no reference keyframe
- **Where:** `src/slam_it/edge/tracking.py:181` (`reference=base + self.reference_id`), `cpp/src/edge/tracking.cpp:348`, `rust/slam-it-edge/src/tracking.rs:91`.
- **Issue:** `reference_id` is -1 when the frame matched no point, so the run records `base - 1`. In the first map that is -1, but in a later map it is a valid-looking keyframe id of the previous map.
- **Why it matters:** The case is unreachable today, because a placed frame has matches that map points observe. The sentinel should still survive the offset.
- **Suggested fix:** `-1 if self.reference_id < 0 else base + self.reference_id`, in all three.

#### [note] Small repetition and naming
- **Where:** `rust/slam-it-edge/src/covisibility.rs` `local_map` repeats `keyframes.push(row); inside[row] = true;` three times (C++ and Python use an `add` helper). `rust/slam-it-eval/src/runs.rs:419-431` (`check_links`) has the same error message in two branches. The variable `before` in `tracking.py:1120` is vague (`checked` in `covisibility.local_map`). `tests/fixtures/eval/generate_run.py:138` keeps a flat `settings` dict (`n_features`, `window`, `chi2_mono`) although format 6 nests settings by group and `scripts/compare_testsets.py:251` reads `settings["loss"]["lost_frames"]`.

#### [note] The viewer loader does not range-check ids
- **Where:** `viewer/src/bundle.js:337-338`, `viewer/src/geometry.js` `keyframeSegments`.
- **Issue:** `keyframe_parents` and `keyframe_links` are shape-checked, but their ids are not checked against K. A corrupted bundle would draw a line to the origin, and an id past K gives NaN lives. The bake (`_check_links`, `_check_parents`) already validates them, so this is only about hand-edited bundles.

### Checked and fine
- `src/slam_it/edge/covisibility.py`, `cpp/include/slam_it/edge/covisibility.hpp`, `cpp/src/edge/covisibility.cpp` and `rust/slam-it-edge/src/covisibility.rs` have module and function docs with shapes, ties and ORB-SLAM sources. The departure from ORB-SLAM's K2 is stated. The three implementations agree on tie-breaks (lower id, first maximum) and on the empty and unknown-point cases.
- `tests/fixtures/edge/generate_covisibility.py` computes its expectations from sets, independently of the module. It asserts that every K2 route and the cap are exercised, and all three languages read the same JSON.
- `src/slam_it/edge/options.py`: every name in `_FRACTIONS` and `_MAY_BE_ZERO` exists in the 97 settings (checked by script). Unknown names and groups are refused by name. `with_settings` and `settings()` round-trip. The generic bad-value loop covers every setting in Python, C++ and Rust.
- The `kf_parent` columns follow insert, erase, `gone_*` archive, `as_dict` and `decode` in the three languages. The native `check_map` validates length and parent.
- The viewer geometry (`keyframeSegments`, `treePairs`, `linkPairs`) is documented and tested for lives and maps. The graph toggle, aria state and colour tokens are consistent.
- Doc numbers: the test-set note's dataset table sums to the CHANGELOG's figures (208,906 / 200,897 tracked, 226 / 268 maps, 177 / 231 losses, 34 / 123 relocalised, 325 / 535 stretches, 13,699 waited frames, 0.38 / 0.25 keyframes per tracked frame). VERSION, CHANGELOG, import-linter layering and Cargo.lock are consistent.
- No debug prints, commented-out code, stray TODOs, hard-coded paths or `unwrap` and `expect` in library code were added; the `unwrap` calls are in `#[cfg(test)]` modules.

### Architecture review

**Scope:** Run 1 of `feature/2b-b2-covisibility`, `9a4f499...390d67d` (7 commits, 158 files). I read the following in full:
- the covisibility module in all three languages, with its bindings and its Protocol;
- the grouped settings in all three languages, their crossings (`options_in`) and the command lines;
- the diffs to map, tracking, mapping, database, relocalisation and the pipeline (Py/C++/Rust);
- the run format 6 writers and readers in all three languages, the bundle and the viewer;
- the scripts, the test option sets, the fixture generator, and the design and ladder changes.

Paths are relative to the repository root. I wrote two probe scripts (outside the repo) and ran them in the `slam-it` env:
- `/tmp/scratch/arch/erase_parent.py`
- `/tmp/scratch/arch/erase_parent_native.py`

**Summary:** The structure is right:
- The graph sits in the edge rung in its own module, at a layer of its own above the map, and is mirrored module for module in C++ and Rust.
- The bindings only convert types. B1's `best_covisible` stand-in is gone, not kept as a shim.
- Both format breaks (run 6, bundle 3) are clean in all three languages.

Two must-fix:
1. The spanning tree is a new cross-keyframe invariant with no single owner. The native `erase_keyframe` breaks it silently, where Python raises.
2. The seven settings B2 adds are never moved off their defaults in a whole run. That is the gap H21 and B1's L1 closed for earlier settings.

The rest are small: leftover old names, the fixture's missing policy, a stand-in not marked as one, and notes for later.

### Findings

#### [must-fix] The spanning tree's invariant has no owner: native `erase_keyframe` silently breaks it, Python's raises
- **Where:**
  - `cpp/src/edge/map.cpp:600` and `rust/slam-it-edge/src/map.rs:1078` (`erase_keyframe`);
  - `src/slam_it/edge/map.py:678-680` (the check) and `:1291` (`erase_keyframe`, whose Raises omit this);
  - the callers that keep the order by hand: `src/slam_it/edge/mapping.py:986`, `cpp/src/edge/mapping.cpp:900` (`cull`), `rust/slam-it-edge/src/mapping.rs:1132`;
  - the run writers' checks: `src/slam_it/eval/runs.py:410`, `cpp/src/eval/runs.cpp` `check_parents`, `rust/slam-it-eval/src/runs.rs` `check_parents`.
- **Issue:**
  - B2 makes `kf_parent` a reference to a living keyframe. Python's check enforces it on every `Map`; natively, `check_map` enforces it only at the binding boundary.
  - The map-layer operation that removes keyframes neither keeps nor checks it. The order "reparent, then erase" is a convention kept by culling in each language.
  - On a keyframe that is still a parent, Python raises `InvalidInputError` (undocumented). The C++ and Rust cores return success, with a map their own `check_map` rejects.
- **Why it matters:**
  - The same function fails differently across languages: an error-contract inconsistency.
  - Natively the bad parent travels all the way to disk:
    - `Pipeline::lifetimes` turns it into a run id;
    - the writers' `check_parents` accept it (a culled keyframe is still a same-map id in the run);
    - the viewer draws the line.
  - It is latent today, since the one caller reparents first. But B3's loop closing and B4's merging remove and replace keyframes, and the essential graph will walk this tree.
- **Evidence:** the compiled `erase_keyframe` called directly, with no decode, on room keyframe id 2, which is keyframe 1's parent:
  ```
  cpp core returned; kf_id [0 1 3] kf_parent [-1  2 -1]
  rust core returned; kf_id [0 1 3] kf_parent [-1  2 -1]
  python InvalidInputError map.kf_parent: a parent must be -1 or another of its keyframes
  ```
- **Suggested fix:** give the invariant one owner, in all three languages:
  - add a `covisibility.cull(m, k, options)` that reparents, then erases (ORB-SLAM's `SetBadFlag`), and have culling, B3 and B4 call it;
  - make `map.erase_keyframe` refuse a keyframe that is still another's parent (`InvalidInput`, documented in all three, with native tests).

  I recommend this over documenting a precondition that only Python happens to enforce.

#### [must-fix] B2's new settings never leave their defaults in a whole run (H21, B1's L1)
- **Where:** `tests/orchestration/datasets.py:134` (`MOVED_OPTIONS`). These settings appear in no test, native or Python:
  - `mapping.fuse_neighbours`, `mapping.mono_fuse_neighbours`, `mapping.fuse_second_neighbours`.

  These are moved only in the fixture's direct calls (`tests/fixtures/edge/generate_covisibility.py:212-213`):
  - `covisibility.min_points`, `.local_keyframes`, `.local_neighbours`, `.strong_points`.
- **Issue:** two kinds of native bug would agree with Python on every test:
  - mapping that hard-coded fusion's 10/20/5;
  - tracking or mapping that handed the graph a default `Options`.
- **Why it matters:** this is the gap that B1's panel made must-fix (L1: nine relocalisation settings, fixed through `MOVED_OPTIONS`) and that H21 set the rule for.
- **Evidence:** `git grep -n "fuse_neighbours\|fuse_second_neighbours" 390d67d -- tests cpp/tests rust/slam-it-edge/tests` finds nothing. `MOVED_OPTIONS`' diff adds no `covisibility.*` or `mapping.fuse_*` key.
- **Suggested fix:**
  - Add all seven to `MOVED_OPTIONS`. The moved room is compared live, so the golden record needs no regenerating.
  - Pick values that move the room's decisions: `local_keyframes` low enough to cap the room's local map, `strong_points` low enough to change `keyframe_links`, which `_assert_same_run` compares.
  - Keep `test_the_moved_settings_change_the_decisions` green.

#### [should-fix] Nothing holds the spanning tree to being a tree
- **Where:**
  - `src/slam_it/edge/covisibility.py:176` (`_adopt`), with the C++ and Rust `adopt`;
  - `tests/edge/test_covisibility.py:147`;
  - `src/slam_it/edge/map.py:678`;
  - the run writers' `check_parents`.
- **Issue:**
  - `adopt` takes the keyframe sharing the most points, of any age. On an older keyframe with no parent, it can take a newer one and close a cycle.
  - The pipeline calls it only on the newest keyframe, so the tree holds today, but the public contract does not say so.
  - No check catches a cycle: not the map's, not the run writers', and not the room test. That test's docstring says "one tree rooted at its first keyframe" but checks only that each parent exists and is linked.
- **Why it matters:** B3's essential graph walks this tree, and a cycle there is an endless walk.
- **Evidence:** adopting rows 0 to 3 of the room's truth map, in order, gave parents `[-1 2 3 2]`: a 2-3 cycle. `Map`'s check accepted it, and the room test's assertions would pass on it.
- **Suggested fix:**
  - Restrict `adopt`'s candidates to older keyframes, as at insertion and in ORB-SLAM's first `UpdateConnections`, or document that it is for the newest keyframe.
  - Make the room test walk each parent chain to row 0 within K steps.
  - Consider an O(K) acyclicity check in the run writers.

#### [should-fix] The covisibility fixture states no oracle or tolerance policy
- **Where:** `tests/fixtures/edge/covisibility.json` (top-level keys: `description`, `cases` only); `tests/fixtures/edge/generate_covisibility.py:218-221`.
- **Issue:** CLAUDE.md requires every parity fixture to state its tolerance policy. `room_runs.json` and the foundations fixtures carry `oracle` and `tolerance`; this one carries neither.
- **Suggested fix:**
  - Add `"oracle": "brute force over point sets"` and `"tolerance": {"comparison": "exact", "rtol": 0.0, "atol": 0.0, "reason": "integer ids and counts"}`.
  - Consider a generic test that every `tests/fixtures/**/*.json` states both: a contract import-linter cannot express.

#### [should-fix] The clean breaks left old names and an incomplete version history behind
- **Where:**
  - Error messages that name the flat `relocalize`, a setting that no longer exists:
    - `src/slam_it/edge/pipeline.py:438,440`
    - `src/slam_it/orchestration/vo.py:231,233`
    - `cpp/src/edge/pipeline.cpp:81,84`
    - `cpp/src/orchestration/vo.cpp:819,822`
    - `rust/slam-it-edge/src/pipeline.rs:357,362`
    - `rust/slam-it-orchestration/src/vo.rs:998,1001`
  - `src/slam_it/edge/relocalization.py:16,18,238`, which still say `reloc_min_inliers` and `reloc_inlier_ratio`.
  - `cpp/include/slam_it/eval/runs.hpp:27-29` and `rust/slam-it-eval/src/runs.rs:19-21`: the version notes stop at 5, while Python's `VERSION` doc lists 6.
- **Issue:** the PR's own rule is that a setting is named `group.field` "in an error", and `CHANGELOG.md:60` says the comments name every setting that way. Also, two of the three format records omit what version 6 added.
- **Suggested fix:** say `relocalization.enabled` in the six messages, rename the three docstring mentions, and add "6 the settings grouped, each keyframe's parent and the strong links" to the C++ and Rust notes.

#### [should-fix] The lost map's strong-links record is a stand-in until B4, but the code does not say so
- **Where:** `src/slam_it/edge/pipeline.py:584` (`_close`), `cpp/src/edge/pipeline.cpp:216`, `rust/slam-it-edge/src/pipeline.rs:568` (`close`).
- **Issue:** you decided that this record stays timed in the lost frame until B4's atlas keeps maps whole. It costs 58 to 360 ms on that frame (the note). No language's docs mark it as temporary. CLAUDE.md asks that deliberate placeholders say so.
- **Suggested fix:** one sentence in each `close`'s docs: a stand-in until B4, its cost, and the design's section 4.

#### [note] The viewer scrubs a single end-of-map snapshot of the graph through time
- **Where:** `viewer/src/geometry.js` (`keyframeSegments`), `viewer/index.html:121`, `src/slam_it/orchestration/bundles.py`.
- **Issue:** the run records each keyframe's last parent, and the strong links only at close or at the run's end. Each line is drawn while both its keyframes live. So at frame f the viewer shows:
  - parents assigned later, by reparenting;
  - links that were weaker at f;
  - but never links to keyframes culled before the map closed.

  This follows from design section 8 item 7, which was decided, but nothing on the page says it.
- **Suggested fix:**
  - Say "as the map ended" in the toggle's title and in the bundle docstring.
  - If a true over-time view is wanted, edge lives (made, removed) in a later format fit the lives pattern. That would go to the backlog.

#### [note] The Python reference builds the dense K x K weight matrix for every one-keyframe question
- **Where:** `src/slam_it/edge/covisibility.py:102` (`_weights`), called by `_links`, `_adopt` and `_reparent` (`:134`, `:176`, `:187`).
- **Issue:** the cores compute one keyframe's shared points (`shared_points`). The reference builds all K² pairs on every call.
  - Calls: about 1 per tracked frame, plus about 16 per keyframe (26 monocular: fusion asks once per target, up to 20).
  - At the test set's largest map, 2,244 keyframes, that is 40 MB and every pair, per call.
  - It is correct, but a Python run's cost grows quadratically with the map.
- **Suggested fix:** compute row k only, as the cores do. The reference stays as readable. This is for the algorithm reviewer to weigh.

#### [note] `compare_testsets.py` at HEAD cannot read the B1 baseline the note compares against
- **Where:**
  - `scripts/compare_testsets.py:249,274`, where `endings` and `latencies` call `runs.read_run` on the base;
  - `src/slam_it/eval/runs.py:684`, which refuses anything but version 6;
  - `docs/notes/test-set.md:342`.
- **Issue:** B1's runs are format 5 with flat settings. Run as the note says, the script stops at `endings(base)`. Please state how the base half was produced. The format breaks again in B3 and B4.
- **Suggested fix:** for the next breaks, have the code that wrote each batch summarise it, and have the comparison read only the summaries and `results.jsonl`. That avoids shims. For the algorithm reviewer and the owner.

#### [note] `Options` writes into the frozen groups it holds
- **Where:** `src/slam_it/edge/options.py:572` (`object.__setattr__(owner, ...)`), and `_GROUPS` at `:677`.
- **Issue:**
  - Normalising writes into group objects that a caller passed in, or that another `Options` shares through `with_settings`' `replace`. It is harmless today, since the writes repeat the same, hash-stable values.
  - `_GROUPS` also repeats `Options`' field list: a group missing from it would be skipped by `_fields` (only the count test would notice).
- **Suggested fix:** derive `_GROUPS` from `fields(Options)`. Optionally have each group check itself, named by a ClassVar prefix.

#### [note] The local map summary reports `base - 1` as the reference when there is none
- **Where:** `src/slam_it/edge/tracking.py:181`, `cpp/src/edge/tracking.cpp:348`, `rust/slam-it-edge/src/tracking.rs:91` (`base + reference_id`).
- **Issue:** with `reference_id == -1` on the second or a later map, the diagnostic names the previous map's last keyframe. It is latent: the pipeline only tracks a placed frame against its local map, and a placed frame always has matched points.
- **Suggested fix:** keep -1 when there is none. For the hygiene or algorithm reviewer.

#### [note] The edge rung's declared budget does not yet mention B2's measured mapping load
- **Where:** `src/slam_it/edge/__init__.py` ("real time under `laptop-cpu`").
- **Issue:** you have decided this and it is recorded in the ladder ("Found while building PR 9") and in the note. The rung's own declaration, where the budget lives, says nothing of mapping's load of 1.7 to 2.6 on large maps until B3's design.
- **Suggested fix:** one clause pointing to the ladder item. Not a re-opening of the decision.

### Checked and fine
- Layers:
  - `covisibility` sits at `(residuals) | (covisibility)` above `map` (`pyproject.toml:194-203`) and imports only `options`, `map`, `backends` and rung 0;
  - database, tracking, initialisation, mapping and the pipeline reach it from above;
  - the C++ includes and Rust `crate::` paths keep the same order (`check_layers.py` green in layer 1).
- Placement: in the edge rung, not rung 0, as the ladder has it. The classic rung reuses place recognition only; the database's "moves to rung 0 rather than copied" rule covers a later view graph.
- Bindings and dispatch:
  - one file per module (`cpp/bindings/edge/covisibility.cpp`, `rust/slam-it-py/src/edge/covisibility.rs`), wired as `edge.covisibility` in both extensions;
  - type conversion only;
  - the `Implementation` Protocol's names and parameters match both; validation happens once in Python and in the cores, through `backends.call` only.
- The error kinds of `links`, `local_map`, `adopt` and `reparent` (`InvalidInput` for a bad row or point) agree across the languages and are tested in all three.
- Grouped settings:
  - one list per language (C++ `for_each_setting`, Rust's exhaustive destructuring macro, Python `_fields`);
  - the bindings check exact keys at both levels;
  - native defaults are held to Python's (`test_a_run_records_its_settings`);
  - `--option` takes dotted keys everywhere; no flat-name shim is left.
- Run format 6 and bundle 3:
  - versions bumped in every writer and reader;
  - parents and links checked alike in all three languages;
  - the golden record carries both, and `_assert_same_run` compares them;
  - the viewer refuses version 2.
- Viewer: the graph's logic is in `geometry.js` (no DOM, Node-tested), the toggle in `main.js` and `scene_view.js`.
- No new runtime dependency (`serde_json` is a dev-dependency of the edge crate's tests, as in foundations). No reifyscene, GPU or depth-model import. Rust stays within the edge VO.
- Docs:
  - the ladder's edge-rung text, L15's closure and the mapping-thread item moved to B3 match the code;
  - design section 8 (counted when asked, only the tree stored, the papers' K2, every listed user of the window moved) matches the implementation;
  - the CHANGELOG and VERSION (0.3.11.0) are present.

### Algorithm review

**Scope:** `9a4f499...390d67d` on `feature/2b-b2-covisibility`. I read these files in full: the new covisibility module in Python, C++ and Rust (`src/slam_it/edge/covisibility.py`, `cpp/src/edge/covisibility.cpp`, `rust/slam-it-edge/src/covisibility.rs`). I read the B2 changes to mapping, tracking, adjust, pipeline, database, relocalization, initialization and map in all three languages, the fixture generator `tests/fixtures/edge/generate_covisibility.py`, the stage tests, the golden record, design sections 8–9 and the test-set note's B2 section. References: ORB-SLAM (Mur-Artal, Montiel & Tardós, T-RO 2015, sections III.D, V.D and VI.A–E; https://arxiv.org/abs/1502.00956). Source: ORB-SLAM2 (https://github.com/raulmur/ORB_SLAM2) and ORB-SLAM3 (https://github.com/UZ-SLAMLab/ORB_SLAM3), downloaded to `/tmp/scratch/orbsrc/` and read there.

**Summary:** The graph, K1/K2, the reference keyframe, local BA's free and fixed sets, culling, triangulation and fusion neighbours, and SetBadFlag's reparenting all match ORB-SLAM's papers. Where they differ from the source, it is a documented, decided choice. The three languages agree with each other and with an independent brute force, on the committed fixture and on 150 extra random maps. The test-set runs' spanning trees are valid. I found no correctness defect in what the pipeline does. The findings are:
- a test gap: the stages' use of the graph is held only by parity and the golden record;
- a wrong id in a diagnostic summary for a case the pipeline never reaches;
- `erase_keyframe` not keeping the spanning tree consistent;
- notes on how ORB-SLAM's source differs from the paper rule, the local-BA gauge, and the state of the art.

### Findings

#### [should-fix] How each stage uses the graph has no known-answer test; the keyframe-decision test still uses B1's rule
- **Where:** `tests/edge/test_mapping.py:133-150` (`reference=m.keyframes - 1` at :145), `tests/edge/test_mapping.py:224-233`, `tests/edge/conftest.py:106`. The rules under test are `src/slam_it/edge/mapping.py:868` (`_fuse_targets`), `:1060` (local BA's set) and `:986` (reparent, then erase), with their C++ and Rust twins.
- **Issue:** The covisibility module is held to a brute force, but the stages that consume it are not:
  - which keyframes each stage takes is checked only by cross-backend parity on the room and by the golden record, which stores "the Python reference pipeline, as it decided when the record was made". That covers fusion's targets (the best 10 or 20, then each one's best 5, without the newest keyframe or repeats), local BA's free set (the new keyframe and its links) and fixed set (keyframe 0 and every other observer), culling over the links with reparenting first, and triangulation's best links;
  - the one test of the keyframe decision passes the newest keyframe as the reference, which was B1's rule, so it cannot fail if the reference were ignored;
  - the stage tests' truth map is built with `maps.insert_keyframe`, which sets every `kf_parent` to -1. So the culling test and the stage parity tests never cull a keyframe that has children, and never exercise K2's child and parent steps. Only the golden record's pipeline runs do.
- **Why it matters:** These selections are the part of B2 that changes decisions. A wrong rule changed the same way in all three languages, for example the newest keyframe left in fusion's targets or the wrong fixed set, passes once the record is regenerated. The note's measurements would then be of that rule.
- **Evidence:** `grep -n "fuse_targets\|\.free\b\|\.fixed\b\|reparent\|kf_parent\|reference=" tests/edge/*.py tests/orchestration/*.py` finds no test outside `test_covisibility.py` that checks the target list, `MappingSummary.free`/`fixed`, or a cull with children. The only hits are the two `reference=` call sites and `test_map.py:156` (`gone_kf_parent`). The C++ and Rust mapping tests have no such checks either.
- **Suggested fix:** Add small known-answer tests on the fixture's maps, or on the truth map with parents adopted:
  - `_fuse_targets`' list and its order (ORB-SLAM3's), with the monocular count;
  - local BA's `free`/`fixed` against sets computed by hand;
  - a cull of a keyframe with children, leaving the fixture's `reparented` parents;
  - `need_keyframe` giving different answers for two references that track different counts.

  Also add a property check over the golden record's runs: one root per map, no cycles, parents alive. My script below confirms these hold on the test set.

#### [should-fix] `LocalTracking.summary(base)` reports `base - 1` as the reference when there is none
- **Where:** `src/slam_it/edge/tracking.py:181`, `cpp/src/edge/tracking.cpp:348`, `rust/slam-it-edge/src/tracking.rs:91`
- **Issue:** `reference = base + reference_id`. When the frame matched no point (`reference_id = -1`, the documented "none"), the summary names the run's keyframe `base - 1`, which is the previous map's last keyframe.
- **Why it matters:** Any caller of the public method gets a wrong id instead of -1. The pipeline never hits it, because a placed frame always has some K1 keyframe (the Rust pipeline even errors if it did not), so this is latent.
- **Evidence:** `LocalTracking(..., reference=-1, reference_id=-1).summary(10).reference` prints `9`.
- **Suggested fix:** `base + id if id >= 0 else -1` in all three languages.

#### [should-fix] `erase_keyframe` breaks the spanning tree; the native cores do it silently
- **Where:** `src/slam_it/edge/map.py:1291`, `cpp/src/edge/map.cpp:600-640`, `cpp/include/slam_it/edge/map.hpp:294-298`, `rust/slam-it-edge/src/map.rs:1078`
- **Issue:** Erasing a keyframe that has children leaves them pointing at an id that is gone:
  - the C++ and Rust cores neither reparent nor refuse;
  - Python refuses only as a side effect of `Map.__post_init__`, with an error that does not name the cause;
  - none of the three documents that the children must be reparented first. Only `cull`/`_cull_keyframes` do it (reparent, then erase).
- **Why it matters:** Any other use of `erase_keyframe`, such as a future atlas or merge, silently corrupts the tree in native code. B3's essential graph depends on that tree.
- **Evidence:** Erasing id 1 from the fixture's `repair` map with every backend raises `InvalidInputError map.kf_parent: a parent must be -1 or another of its keyframes`. That message comes from Python's decode; `map.cpp:600-640` drops `kf_parent[row]` without looking at the children.
- **Suggested fix:** Have all three cores refuse a keyframe with children (`InvalidInput`, documented), or make reparent-then-erase the single public "SetBadFlag".

#### [note] ORB-SLAM's source counts weak links wherever its sorted list is rebuilt
- **Where:** `src/slam_it/edge/covisibility.py:30-34` (the "Unlike ORB-SLAM" paragraph), `:187-210` (reparent)
- **Issue:** In both sources, `UpdateConnections` stores every keyframe that shares at least one point (`mConnectedKeyFrameWeights = KFcounter`, ORB-SLAM2 `KeyFrame.cc:367`, ORB-SLAM3 `:461`). `UpdateBestCovisibles` re-sorts all of those, with no threshold of 15 (ORB-SLAM2 `:138`). It runs on every `AddConnection` and `EraseConnection` (`:135`, `:553-566`). Consequences:
  - in SetBadFlag, each child's list is re-sorted (`:466`) before `GetVectorCovisibleKeyFrames` (`:500`), so a child can join a candidate it shares 1 to 14 points with. Here such a child falls back to the culled keyframe's parent;
  - ORB-SLAM3's culling calls `UpdateBestCovisibles()` first (`LocalMapping.cc:909-910`), so it tests every keyframe sharing a point with the new one. It stops after 100 (`:1049`, `count>100`).

  The paper's rule (at least 15, or the best one) used here is cleaner and consistent across stages. The source differences are worth a line in that paragraph. ORB-SLAM3's cap of 100 is relevant to B3's mapping-load design.

#### [note] Local BA has no guard against having no fixed keyframe; monocular often runs with one
- **Where:** `src/slam_it/edge/mapping.py:1056-1063`, `src/slam_it/edge/adjust.py:197-232`
- **Issue:** Across the test set's 78,917 local BAs, none had zero fixed keyframes, but 15,113 (19%) had exactly one; 14,405 of those were monocular-only runs. One fixed pose leaves monocular's scale gauge to the damping (the Sim(3) gauge; Strasdat, Montiel & Davison, RSS 2010). ORB-SLAM2 behaves the same. ORB-SLAM3 abandons the BA when none is fixed ("There are 0 fixed KF in the optimizations, LBA aborted", `Optimizer.cc:1182-1184`). B2's wider free set (every link, not a window of 10) makes the zero case more reachable.
- **Evidence:** My scan of `datasets/results/2b-b2/testset/seed-0/*/run/diagnostics.jsonl` printed `BA runs 78917 fixed==0 0 fixed==1 15113`.
- **Suggested fix:** None needed for B2. For the backlog: a zero-fixed guard as ORB-SLAM3 has, which is observable through `MappingSummary.fixed`.

#### [note] The paper's K2 builds a larger local map than ORB-SLAM's own source
- The source's `break` (ORB-SLAM2 `Tracking.cc:1328`, ORB-SLAM3 the same) ends the expansion at the first parent added, so ORB-SLAM's actual local map is K1 plus a few keyframes.
- B2's local map (median 38 keyframes, at most 535) and its tracking cost (a median of 3.8 to 5.0 ms against 0.6 to 0.8) partly reflect the decided paper rule.
- B3's latency design should not compare these figures with ORB-SLAM's timings as if the local maps were the same size.

#### [note] State of the art
- A covisibility graph with a spanning tree is still the structure of current sparse feature SLAM (Campos et al., ORB-SLAM3, T-RO 2021, https://arxiv.org/abs/2007.11898).
- The learned rungs replace it with frame graphs built from flow or proximity: Teed & Deng, DROID-SLAM, NeurIPS 2021 (https://arxiv.org/abs/2108.10869); Lipson, Teed & Deng, Deep Patch Visual SLAM, ECCV 2024 (https://arxiv.org/abs/2408.01654); Murai, Dexheimer & Davison, MASt3R-SLAM, CVPR 2025 (https://arxiv.org/abs/2412.12392).
- This is context for later rungs, not a defect here.

### Checked and fine
- **Links:** at least `min_points` (15) shared points, or the single best, most shared first with ties by the lower id. This matches the paper's section III.D and `UpdateConnections`' fallback. The orderings are total, so all three languages are deterministic.
- **K1/K2:** match `UpdateLocalKeyFrames` (paper V.D; ORB-SLAM2 `Tracking.cc:1282-1331`). Only K1 is iterated, as the source's end iterator does. The 80 check sits at the top of each iteration. "Each added before the next is looked for" is the source's `mnTrackReferenceForFrame` marking. K1 comes from the placed frame's inlier points, which `_optimize_pose` has already cleared of outliers, as `TrackWithMotionModel` discards them.
- **Reference keyframe:** K1's first is `pKFmax`, taken before the search. `need_keyframe` counts its points with 2 or 3 observations (`TrackedMapPoints(nMinObs)`) and validates the row in all three languages.
- **adopt:** matches the first `UpdateConnections`. Keyframe ids are per map, so "id 0 has no parent" matches ORB-SLAM3's `GetInitKFid()`. The parent is set again after fusion if needed, and monocular initialisation gives the second keyframe its parent.
- **reparent:** matches SetBadFlag's rounds (strict `>`, candidates growing, the rest given the culled keyframe's parent). It cannot create a cycle: candidates are only the parent and children already rejoined. The culled keyframe's links are dropped first. Erasing points left with weight 2 or less with a culled keyframe matches `EraseObservation`'s `nObs<=2`.
- **Local BA:** the new keyframe plus its links are free; keyframe 0 and the other observers are fixed, as in `LocalBundleAdjustment` (paper VI.D; ORB-SLAM2 `Optimizer.cc:529`).
- **Culling:** tests the links computed before the loop, by id, so rows shifting after an erase do no harm (all three languages). Keyframe 0 is skipped and observations are updated after each cull, as `KeyFrameCulling` does.
- **Triangulation, fusion, relocalisation:** triangulation takes the best 10, or 20 monocular (ORB-SLAM2). Fusion's targets follow ORB-SLAM3's order with ORB-SLAM2's counts; Rust's `0..targets.len()` is evaluated once. Relocalisation's groups take `GetBestCovisibilityKeyFrames(10)`.
- **Strong links:** at least 100 shared points, as `GetCovisiblesByWeight(100)`. They include parent–child pairs, which matches how ORB-SLAM's viewer draws them.
- **Three-language agreement:**
  - `tests/edge/test_covisibility.py` passes (81 tests);
  - my extra check runs 150 seeded random maps (3 to 39 keyframes, skipped ids, varied settings) through all three backends against the generator's set rules: 10,314 comparisons, 0 mismatches;
  - the golden record holds every backend to the same runs.
- **Run outputs:** all 244 maps in the seed-0 test set have a valid tree at the end (one root, the map's first keyframe; no cycles; parents alive in the same map), and every strong link has at least 100 points and stays within one map.
- **L15:** confirmed from `datasets/results/2b-b2/l15`. The 20 later starts have no loss and no relocalised frame. One frame fails, in MH_01's start at frame 130. Start 0 of MH_01 relocalises one frame.
- **Docs:** the claims in the design and the module docstrings about ORB-SLAM's source agree with the source: the stale stored weights, the `break` in K2, the K1 order by memory address, ORB-SLAM2's double-counted second neighbours, and ORB-SLAM3's counts.

Scripts saved for reuse (outside the repo):
- `check_run_spanning_trees.py` — the spanning-tree check over a seed's runs.
- `covisibility_random_parity.py` — the random-map check of the three backends against the brute force.

Throwaway probes are in `/tmp/scratch/algo/`.
