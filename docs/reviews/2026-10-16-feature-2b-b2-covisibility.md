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

## Run 2 -- 2026-10-16

| | |
|---|---|
| **Branch** | `feature/2b-b2-covisibility` |
| **Range** | `390d67d...c9d4a30` (Run 1's fixes; base `origin/develop`, merge base `9a4f499`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | not run on this head, by evan-wes's rule of 2026-10-16: it runs once, when the PR is ready to merge (offline CI); each fix commit ran the checks its change touches |

### Findings and resolutions

Every Run 1 finding is fixed as decided, the three reviewers agree, with two of its doc fixes
in Python alone (H17, A13). None is must-fix. Two reviewers found the new public functions'
refusals untested (H14, A14); each is decided once, under its first ID.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H14 | should-fix | hygiene | `mapping.fuse_targets`, `local_ba_keyframes`; `covisibility.cull` natively; Rust's refusal asserts | The new public functions' refusals are untested; natively `cull`'s row check is never called; Rust asserts the new refusals with `is_err()` only (also A14) | fix (an empty map, and Python's unknown mode, for both in the three suites; `cull` past the end and on an empty map natively; Rust by `ErrorKind::InvalidInput`) | fixed (78fde39) -- decided by evan-wes, 2026-10-16 |
| H15 | should-fix | hygiene | the three covisibility tests' map builders | The blank points sit on every camera centre, so culling makes NaN normals and Python warns | fix (the points in front of the cameras; the culled map's normals held finite) | fixed (78fde39: the culled maps' normals held finite; pytest clean with RuntimeWarning as an error) -- decided by evan-wes, 2026-10-16 |
| H16 | should-fix | hygiene | `CHANGELOG.md` | It describes the branch's own intermediate states (a Rust panic, 194 to 3 ms) as changes since 0.3.10.0 | fix (culling an empty map is refused, where it did nothing; the timing left to the commit and report) | fixed (8dde603) -- decided by evan-wes, 2026-10-16 |
| H17 | should-fix | hygiene | C++ and Rust `adopt`, `LocalMapSummary.reference`; the C++ `cull` binding | Run 1's doc fixes reached Python only (also A13) | fix (the sentence in the C++ and Rust docs; the binding's throws) | fixed (f14cab7) -- decided by evan-wes, 2026-10-16 |
| H18 | should-fix | hygiene | `edge/covisibility.py` | The pair set written three ways, concatenated ranges two ways; a docstring that does not parse | fix (one helper for each, the single-row count kept fast; the docstring reworded) | fixed (f14cab7: `_unique_pairs` and `_runs`; 0 mismatches in 17,984 comparisons against 390d67d's reference) -- decided by evan-wes, 2026-10-16 |
| H19 | note | hygiene | `viewer/src/bundle.js` `keyframeIds` | One number carries two schemas; the lower bound untested | fix (the negative cases tested) | fixed (78fde39) -- decided by evan-wes, 2026-10-16 |
| H20 | note | hygiene | `docs/notes/test-set.md` | "which no later commit on the branch changes" is no longer literally true | fix (no later commit changes its decisions) | fixed (8dde603) -- decided by evan-wes, 2026-10-16 |
| H21 | note | hygiene | `options.py`, `test_covisibility.py`, the generator, two Python tests, `cull_keyframes`' docstring | Small items | fix | fixed (78fde39: the tests' items; f14cab7: the source's) -- decided by evan-wes, 2026-10-16 |
| A13 | should-fix | architecture | C++ and Rust `adopt`; the `cull` binding | Same as H17 | see H17 | fixed with H17 (f14cab7) -- decided by evan-wes, 2026-10-16 |
| A14 | should-fix | architecture | the new public selections | Same as H14 | see H14 | fixed with H14 (78fde39) -- decided by evan-wes, 2026-10-16 |
| A15 | note | architecture | `map.erase_keyframe` | The root rule lives in `cull` alone; erase still erases a childless keyframe 0 | fix (erase's docs say it guards parents, not the root, callers above going through `cull`); revisiting with a second caller, in B3's outline | fixed (f14cab7: erase's docs in the three; 8dde603: the revisit in B3's outline) -- decided by evan-wes, 2026-10-16 |
| A16 | note | architecture | the online-SLAM design, section 8 items 3 and 9 | Who owns the tree's invariant, and what the fixture now holds, are not recorded | fix (both items) | fixed (8dde603) -- decided by evan-wes, 2026-10-16 |
| A17 | note | architecture | `test_mapping` in the three languages | Mapping's selections are tested in covisibility's test files | fix (the mapping tests' docstrings name where) | fixed (78fde39) -- decided by evan-wes, 2026-10-16 |
| L8 | should-fix | algorithm | `tests/fixtures/edge/generate_covisibility.py` `FUSION`; commit 7ce41f2's message | The fixture's monocular count does not bind, though the generator and 7ce41f2 say each does | fix (counts that each bind, asserted by the generator; the fixture regenerated) | fixed (78fde39: counts 1, 2 and 2, each asserted to bind; 7ce41f2's claim corrected in 78fde39's message, as evan-wes chose, history kept) -- decided by evan-wes, 2026-10-16 |
| L9 | note | algorithm | `covisibility.cull` | `SetBadFlag`'s second guard (`mbNotErase`, loop closing's) belongs in `cull` with B3 | fix (named in B3's outline) | fixed (8dde603: B3's outline) -- decided by evan-wes, 2026-10-16 |

### Hygiene review

**Scope:** Run 2, `390d67d...c9d4a30` on `feature/2b-b2-covisibility`: c13a438, 7ce41f2, 64afa25, db9dd1f, 471f7fd and the report in c9d4a30. Paths are relative to the repository root.

I read the diffs in full, plus `src/slam_it/edge/covisibility.py` and the touched parts of the other modules. I covered the Python, C++ and Rust cores, both bindings, the tests, the fixture and its generator, the viewer, the CHANGELOG and the docs.

Checks I ran, all read-only:
- The Python edge, eval and database tests (525 passed), `tests/test_backends.py` and `tests/edge/test_pipeline.py` (74 passed), and `cargo test -p slam-it-edge --test covisibility --test mapping` (pass).
- I regenerated the covisibility fixture from a copy of its generator and it is byte-identical to the committed JSON.
- Probes are in `/tmp/scratch/hyg2/`.
- Importing the editable extension re-ran the cmake install into the git-ignored `build/`. `git status` is clean.

**Summary:** All 13 of my Run 1 findings are fixed as decided, in the languages where each applies, with two documentation gaps (finding 4). The new code (`covisibility.cull`, `erase_keyframe`'s refusal, `fuse_targets`, `local_ba_keyframes`, the row-counting reference, the viewer check) validates its inputs, mirrors across the three languages and is documented to a good standard. I found no must-fix. The should-fix items are:
- refusal tests that are missing or weak on the new public functions;
- a new test fixture that feeds NaN into the culled maps and warns;
- a CHANGELOG entry that describes the branch's own intermediate states as changes since 0.3.10.0;
- documentation fixes that reached the Python docs only;
- the same index logic written several ways in the Python reference.

### Findings

#### [should-fix] The new public `fuse_targets` and `local_ba_keyframes` have no refusal test, and `cull`'s row check has none natively
- **Where:** `src/slam_it/edge/mapping.py` (`fuse_targets`, `local_ba_keyframes`, which document Raises for an unknown mode and a map with no keyframe), `cpp/src/edge/mapping.cpp` (`has_keyframes` calls in both), `rust/slam-it-edge/src/mapping.rs:1046` and `:1067`. `cpp/src/edge/covisibility.cpp` `cull` and `rust/slam-it-edge/src/covisibility.rs` `cull` (the `check_row` call). Rust refusals at `rust/slam-it-edge/tests/covisibility.rs:94` and `:114`.
- **Issue:** The new functions are tested only for their values, against the fixture. No test passes them an empty map, an unknown mode (Python) or invalid options (natives). `tests/edge/test_mapping.py` `test_rejects_malformed_arguments` gained a `cull_keyframes` empty case and `need_keyframe` reference cases, but no case for these two. Natively, `cull` with a row out of range or on an empty map is never called. `check_row` is exercised only through `links`, and a forgotten `check_row` in `cull` would be an out-of-bounds read in C++, as H1 was. In Rust the two new refusals (`cull` of keyframe 0, `erase_keyframe` of a parent) are asserted with `.is_err()` only. A refusal for the wrong reason would pass. The C++ twin uses `is_invalid_input`, and the Python twin matches the message.
- **Why it matters:** This is the Run 1 H1/H6 pattern again: a checked boundary with nothing keeping the check. Every must-fix of Run 1 came from an unguarded empty map.
- **Evidence:** `git grep -n "fuse_targets\|local_ba_keyframes" HEAD -- tests cpp/tests rust/slam-it-edge/tests` finds only the fixture comparisons: `tests/edge/test_covisibility.py:149-157`, `cpp/tests/edge/test_covisibility.cpp:174-180`, `rust/slam-it-edge/tests/covisibility.rs:198-206`. Live, all three backends refuse an empty map with `m: expected a map with at least one keyframe`, so the behaviour is right and only unpinned.
- **Suggested fix:** Add an empty-map case, and an unknown-mode case in Python, for both functions to the three suites. Add `cull` on a row past the end and on an empty map to the C++ and Rust covisibility tests. Assert the Rust refusals with `.unwrap_err().kind == ErrorKind::InvalidInput`, as `check_map`'s tests do.

#### [should-fix] The cull tests build maps whose points sit on every camera centre, so culling feeds NaN into the result and Python warns
- **Where:** `tests/edge/test_covisibility.py:59` (`pt_xyz=np.zeros((p, 3))` with identity keyframe poses), `cpp/tests/edge/test_covisibility.cpp:60`, `rust/slam-it-edge/tests/covisibility.rs:64`. These are used by the new `test_a_culled_keyframe_leaves_after_its_children`, `check_cull` (C++) and `check_cull` (Rust).
- **Issue:** `fixture_map` was safe while only the graph read the map. Now `erase_keyframe` refreshes the normals of the points that lose an observation. A point at the origin seen from a camera at the origin gives 0/0, so the culled map carries NaN `pt_normal`. The tests compare only `kf_id` and `kf_parent`, so the NaN is never noticed.
- **Why it matters:** The suite now emits unexplained `RuntimeWarning`s, and the new tests quietly run `erase_keyframe` on geometry it is not defined for. Nothing makes warnings errors (`pyproject.toml` `addopts` has no `-W error`), so the next real divide-by-zero would look like this one.
- **Evidence:** The pytest summary for `tests/edge/test_covisibility.py` reports `RuntimeWarning: invalid value encountered in divide` at `src/slam_it/edge/map.py:487` for `test_a_culled_keyframe_leaves_after_its_children[path-1-python]` and `[path-3-python]`. My probe `.../hyg2/probe_nan.py` culls every non-root row of every case. Output: `path-1 row 4: python warnings ['invalid value encountered in divide'], NaN normals 1 of 54`, and `cpp` and `rust` also give 1 NaN normal with no warning. `path-3 row 2` gives 2 NaN normals in all three.
- **Suggested fix:** Put the blank points in front of the cameras in the three builders, for example at (0, 0, 1) to match `kp_bearing` (0, 0, 1). Assert the culled map's `pt_normal` is finite, so the tests would notice.

#### [should-fix] The CHANGELOG describes fixes to this branch's own intermediate states as changes since 0.3.10.0
- **Where:** `CHANGELOG.md:67-70` (the "Changed" bullet "Culling keyframes refuses a map without any ... where Rust panicked and C++ read out of bounds ... `covisibility.links` from 194 to 3 ms").
- **Issue:** The Rust panic and the C++ out-of-bounds read existed only in B2's earlier commits. In the released base the empty-map case was a no-op.
- **Why it matters:** A reader of 0.3.11.0 is told the previous release panicked. The user-visible change is that culling an empty map now raises `InvalidInput`, and the entry hides that. Likewise `covisibility.links` did not exist at 0.3.10.0, so "194 to 3 ms" compares two states of this branch. That belongs in the commit message and the report, which already hold it.
- **Evidence:** At the base, `git show 9a4f499:src/slam_it/edge/mapping.py` loops `for kf_id in m.kf_id[m.window(options.window)][:-1]`, which is empty for an empty map and returns it unchanged. Rust's `cull_keyframes` at that commit does `ids.pop()` on an empty `Vec` and returns `Ok`.
- **Suggested fix:** Say "Culling keyframes refuses a map without any (it did nothing before)". Drop the panic and out-of-bounds wording and the 194-to-3 ms clause from the CHANGELOG. I re-timed that figure (194.3 to 3.2 ms at 500 keyframes), so it is true, only misplaced.

#### [should-fix] Run 1's documentation fixes reached the Python docs only
- **Where:**
  - A3's `adopt` caveat: it is "for the keyframe being inserted ... given an older keyframe without a parent, it may take a newer one whose own chain leads back to it, a cycle". It is in `src/slam_it/edge/covisibility.py:384-` but not in `cpp/include/slam_it/edge/covisibility.hpp:72-76` or `rust/slam-it-edge/src/covisibility.rs:225-235`.
  - H11's "-1 when the frame was matched to no point" is in Python's `LocalMapSummary.reference` doc (`tracking.py`) but not in `cpp/include/slam_it/edge/tracking.hpp:50` or `rust/slam-it-edge/src/tracking.rs:45`.
  - The C++ `cull` binding's `@throws CoreError for a row out of range` (`cpp/bindings/edge/covisibility.cpp:92`) omits the refusal of the map's first keyframe. The Rust binding says "from the core".
- **Issue:** CLAUDE.md asks for conventions stated in all three languages' docs, and the cores are what a future caller (B3's loop closing, B4's merging) reads.
- **Why it matters:** The cycle caveat is exactly the hazard A3 raised for B3's essential graph, and only the Python docs carry it.
- **Suggested fix:** One sentence each in the C++ header and the Rust rustdoc. Fix the binding's `@throws`.

#### [should-fix] The Python reference builds "each observation once" three ways and "concatenated ranges" two ways
- **Where:** `src/slam_it/edge/covisibility.py:117` (`_weights`), `:135-137` (`_observing`), `:148-152` (`_pairs`), and `:126-127` against `:163`.
- **Issue:**
  - `np.unique(np.column_stack([m.kp_point[rows], m.keypoint_keyframe()[rows]]), axis=0)` is written in `_weights`, in `_observing` and (as `_pairs`) a third time. `_pairs` was added for exactly this, but `_weights` does not use it. `_local_map` computes the pair set twice per call (`_observing`, then `_pairs`).
  - "The ranges start..start+size, concatenated" is `np.arange(reps.sum()) - np.repeat(cumsum - reps, reps)` plus `starts` in `_weights`, and `np.repeat(start - cumsum + sizes, sizes) + np.arange(sizes.sum())` in `_shared_in`. The second has no comment, and its docstring ("from `_pairs`'s, made once") does not parse.
- **Why it matters:** This is the same logic written two ways within one language, which jscpd cannot see. The index arithmetic is the part of the reference most likely to be touched for B3's cap work. My random check (300 maps with duplicates, empty keyframes and `-1`) found `_shared` and `_shared_in` equal to `_weights[k]`, so the code is right. The cost is readability.
- **Suggested fix:** Have `_weights` and `_observing` start from `_pairs`. Give "concatenated ranges" one named helper, for example `_runs(starts, sizes)`, used by both. Reword the `_shared_in` docstring.

#### [note] The viewer's `keyframeIds` packs two array schemas into `stride`, and its lower bound is untested
- **Where:** `viewer/src/bundle.js` `keyframeIds`, `viewer/tests/bundle.test.js` (the added `parents[k - 1] = k` and a link to `k`).
- **Issue:** `stride === 1` means parents (-1 allowed) and `stride === 3` means links (two ids, then a weight). The rule is evaluated per element as `id < (stride === 1 ? -1 : 0)`, and the JSDoc explains it. The tests cover only an id equal to K. A link with id -1 or a parent of -2, the `id < ...` branch, is not tested. That is the same class as H6, but the viewer is a lower-stakes reader.
- **Suggested fix:** Add the two negative cases. Optionally pass `{ stride, least }` instead of one number carrying both meanings.

#### [note] The test-set note's condition "which no later commit on the branch changes" is no longer literally true
- **Where:** `docs/notes/test-set.md:336-337`.
- **Issue:** It says the measured pipeline is commit 8e9b39b's, which no later commit changes. After this run's fixes, c13a438 and db9dd1f changed `mapping.py`, `mapping.cpp`, `mapping.rs`, `tracking.*` and others.
- **Why it matters:** The commits say "No decision of the golden record moves", and I found no decision change (the new guards are unreachable from the pipeline). So the measurement stands. A reader checking `git diff 8e9b39b..HEAD` would still find the claim contradicted by the diff.
- **Suggested fix:** "no later commit changes its decisions (the golden record is unmoved)".

#### [note] Small items
- `src/slam_it/edge/options.py:15,18`: `import dataclasses` and `from dataclasses import ...` both appear, for one `is_dataclass` use. Importing `is_dataclass` with the others would do.
- `tests/edge/test_covisibility.py:197`: `case["keyframe_lives"][2]` is the removal-reason row because the golden record stores lives by column (`generate_room_runs.py:136`, `.T.tolist()`). A one-line comment would save the next reader a trip.
- `tests/fixtures/edge/generate_covisibility.py:116`: `_fuse_targets` takes `first` as a parameter but reads the second count from the global `FUSION`.
- `test_a_parent_is_not_erased` and the new `relocalize_locally` bad-`last` loop are parametrized by backend, but both refusals happen in Python's check before dispatch. The native refusals are held by the C++ and Rust suites. This is fine, but the parametrization over `backend` suggests native coverage the test does not give.
- `src/slam_it/edge/mapping.py` `cull_keyframes` docstring: the rewrap leaves "when more than" on a line of its own.

### Checked and fine

Run 1 findings in my lane, each checked against its recorded resolution:
- **H1** (`cull_keyframes` on an empty map): `check_newest` is in the three. The Run 1 probe now gives `InvalidInputError m: expected a map with at least one keyframe` for python, cpp and rust. The Python Raises is updated, and the empty case is tested in all three suites.
- **H2** (Python run-format-6 validators): 14 write-side cases (6 parents, 8 links), covering every branch of `_check_parents` and `_check_links`. Two read-side cases and `keyframe_parents` and `keyframe_links` in the round trip. Counts match the report.
- **H3 and A2** (`MOVED_OPTIONS`): all seven settings are moved. I re-ran the gap room (C++ backend, four cases) with each put back alone at its default. `fuse_neighbours` changes the decisions in the 2 non-monocular cases, and `mono_fuse_neighbours` and `min_points` in the 2 monocular ones. `fuse_second_neighbours` and `strong_points` change them in all 4. `local_keyframes` and `local_neighbours` change nothing. That is exactly what the `datasets.py` docstring and report say, and the capped fixture cases hold the last two.
- **H4 and A5** (old names): no flat `reloc_*`, `init_min_matches` or similar remains in the comments, docstrings or test labels (`git grep` at HEAD). The C++ and Rust format notes give version 6.
- **H5:** `n_features` as a string or `None` now gives `InvalidInputError features.n_features: must be an integer, got ...`. I also tried `scale_factor`, `init_factor`, `levels` and the two thresholds with wrong types, and all are named by field. Test cases added.
- **H6:** `kf_parent` (itself, not in the map, below -1) is refused and tested in the three. A `need_keyframe` reference out of range is tested in the three; Rust has only the past-the-end case because the type is `usize`. A bad `last` for `relocalize_locally` is tested in the three.
- **H7, A8 and H10:** I re-timed the claim on the Run 1 map. `links` takes 2.9, 3.2 and 3.2 ms at 50, 200 and 500 keyframes. `local_map` takes 12.4, 37.7 and 85.5 ms. This matches the commit message (3.3 and 86.7 at 500). `_shared` and `_shared_in` equal `_weights[k]` on 300 random maps (duplicates, empty keyframes, `-1`); empty and unknown-point maps are fine. Every private reference function has its one-line docstring (only the nested `add` in `_local_map` has none), and `_decode_map` is gone.
- **H8:** all six vocabulary errors, and the tests that match them, say `relocalization.enabled`.
- **H9:** the database test now scores two keyframes and asserts the group's score is strictly above its best candidate's own. The case is renamed "best link only".
- **H11 and A11:** `-1` survives the offset in the three languages, and each has a test. Only the docs are incomplete, see finding 4.
- **H12:** the Rust `add` helper, the single `check_links` refusal, `checked` in `relocalize_locally`, and the nested run-fixture settings (`run.json` and `run_values.json` regenerated) are all done.
- **H13:** `keyframeIds` checks parents and links against K. Positive overflow is tested; see the note for the negative bound.
- **A1 and L3:**
  - `covisibility.cull` is added in Python, C++ and Rust, with bindings and the Protocol entry. It reparents, then erases, and refuses keyframe 0 with a message naming the keyframe.
  - `erase_keyframe` refuses a parent in the three, with identical messages and docs in all three languages.
  - The C++ private `cull` helper is gone, so the cull logic exists once per language.
  - The Python `cull` test is checked against the fixture's `reparented` parents in every backend.
- **A3:** the Python `adopt` doc and the room's parent chains are done, and so is the golden-record tree test. Native docs are missing, see finding 4.
- **A4:** the fixture states its oracle and tolerance. Regenerating it from the committed generator reproduces the JSON exactly. The ladder's deferral text matches the fixtures on disk.
- **A6:** the stand-in sentence is in the three pipelines' `close`, with numbers matching the note (58 to 360 ms, section 4).
- **A7, A10, A12 and L4:** the toggle title and bundle docstring say "as each map ended". `_GROUPS` now comes from `Options`' own fields (same 11 groups, same order). `edge/__init__.py` carries the load clause (1.7 to 2.6, five largest, matching the note and ladder). The docstring line on ORB-SLAM's source and the cap of 100 in B3's outline are present.
- **L1:** `fuse_targets` and `local_ba_keyframes` are public, bound and documented in the three languages, and held to the fixture. The generator computes them by its own brute force, with a sentinel that fusion's second, newest and repeat routes are all hit. The keyframe-decision test decides both ways against two references, in every backend.
- **A9, L5 and L6:** these are recorded in the test-set note and B3's outline (online-SLAM design section 4) as the report says.
- **Production hygiene of the new code:** no `print`, TODO, `unwrap()` or `expect()` in library code, bare `except`, commented-out code or hard-coded path in the Run 2 diff. Public inputs are validated and the errors name the value. Python and native error messages for the new refusals are identical.
- **Report and CHANGELOG counts:** the report's tallies (32 findings; 5 must-fix rows; "other 27" is 25 fixed, 1 deferred, 1 waived) match its table. The CHANGELOG's "seven settings" and "5 cases" are right, and the "culling an empty map, a bad parent, reference keyframe or point id" test claims hold in all three languages.
- **Not run:** layer 1 as a whole (clang-tidy, doxygen, clippy, tsc, prettier, jscpd), per the brief.

### Architecture review

**Scope:** Run 2 of `feature/2b-b2-covisibility`, range `390d67d...c9d4a30`: c13a438 (cull, erase refusal, empty-map culling), 7ce41f2 (stage selections public and held to the fixture, settings moved, run-format tests), 64afa25 (the Python reference counts rows), db9dd1f (smaller findings), 471f7fd (docs) and c9d4a30 (the report).
- I checked each of Run 1's architecture findings (A1 to A12) against its recorded resolution, along with L1, L3, H1, H3 and H7 where they touch architecture.
- I read the new public surface in all three languages: `covisibility.cull`, `mapping.fuse_targets`, `mapping.local_ba_keyframes`, their bindings and their Protocol entries.
- I read design section 8, the B3 outline in section 4, and the ladder's open items.
- Commands run at `c9d4a30`, working tree clean before and after:
  - `scripts/check_layers.py` passed;
  - `lint-imports`: 22 contracts kept, 0 broken;
  - `pytest tests/test_backends.py -k "matches_its_protocol or has_every_module"`: 4 passed, cpp and rust.
- One probe, run on the Python backend only (no extension import): `/tmp/scratch/arch2/root_rule.py`.

**Summary:**
- A1 is fixed as you chose, in all three languages:
  - `covisibility.cull` (reparent, then erase) is now the only caller of `erase_keyframe`;
  - `erase_keyframe` refuses a keyframe that is still a parent, with the same kind and message in each language, documented and tested natively;
  - `cull` refuses the map's first keyframe.
- The stages now call the same helpers that the new public selections wrap, in every language. The fixture's expectations come from an independent brute force.
- Layering, bindings and Protocols hold.
- Two should-fix items remain:
  - A3's doc fix reached Python only;
  - the two new public mapping functions have refusals that no test exercises.
- Three notes are about where the new rules are recorded.

### Findings

#### [should-fix] A3 is fixed in Python only: the C++ and Rust `adopt` docs do not say it is for the keyframe being inserted
- **Where:**
  - `cpp/include/slam_it/edge/covisibility.hpp:72-76` and `rust/slam-it-edge/src/covisibility.rs:225-235`, against `src/slam_it/edge/covisibility.py:384` (the only `adopt` doc c13a438 changed);
  - also `cpp/bindings/edge/covisibility.cpp:92`, the `cull` binding's doc.
- **Issue:**
  - A3's resolution was to document `adopt` as ORB-SLAM's first `UpdateConnections`, for a keyframe being inserted, since on an older keyframe it can close a cycle. c13a438 added that sentence to the Python docstring alone. The C++ header and the rustdoc still say only "Give a keyframe its parent, if it has none yet".
  - Separately, the `cull` binding's doc says `@throws CoreError for a row out of range`, but its core also refuses the map's first keyframe.
- **Why it matters:**
  - The cores build and are used without Python: the native programs, and B3's loop closing in C++ and Rust.
  - A native caller reading the header learns nothing of the precondition whose breach is an endless walk in B3's essential graph. The tree check A3 asked for, walking each chain to its root, runs only in Python's tests and the golden-record test.
- **Evidence:** `git show c13a438 -- cpp/include/slam_it/edge/covisibility.hpp rust/slam-it-edge/src/covisibility.rs` adds only `cull`. The `adopt` declarations are unchanged at HEAD.
- **Suggested fix:**
  - Add the Python docstring's sentence to the C++ `@brief` and to the Rust doc ("for the keyframe being inserted, the map's newest; on an older one without a parent it may take a newer one whose chain leads back to it, a cycle").
  - Add "or the map's first keyframe" to the `cull` binding's `@throws`.

#### [should-fix] The new public selections refuse an empty map, and Python's `fuse_targets` an unknown mode, but no test exercises any of these refusals
- **Where:**
  - Python: `src/slam_it/edge/mapping.py:893` (`fuse_targets`: `check_mode`, `_check_newest`) and `:941` (`local_ba_keyframes`: `_check_newest`);
  - C++: `cpp/src/edge/mapping.cpp:909`, `:919` (`check_options`, `has_keyframes`);
  - Rust: `rust/slam-it-edge/src/mapping.rs:1046`, `:1067`;
  - the existing refusal tests: `tests/edge/test_mapping.py:292` (`test_rejects_malformed_arguments`), `cpp/tests/edge/test_mapping.cpp:186`, `rust/slam-it-edge/tests/mapping.rs:164`.
- **Issue:** these are public, bound functions in all three languages. Each has refusal branches, and every test call of them is a fixture success case. It is the gap that H1 (`cull_keyframes` on an empty map) and H6 closed for their neighbours in this same PR.
- **Why it matters:**
  - The natives' `has_keyframes` branch is the only thing between an empty map and `keyframe_count(m) - 1` underflowing in `targets_of` and `local_rows`. That is the exact fault H1 found in culling.
  - Python checks first, so through Python only the Python check is held. The native branch is held by nothing.
- **Evidence:**
  - `git grep -n "fuse_targets\|local_ba_keyframes" c9d4a30 -- tests cpp/tests rust/slam-it-edge/tests` finds only the fixture comparisons (`test_covisibility.py:155-157`, `test_covisibility.cpp:174-179`, `tests/covisibility.rs:198-204`).
  - The probe shows the refusals exist: `fuse_targets InvalidInputError m: expected a map with at least one keyframe`, the same for `local_ba_keyframes`, and `mode: expected one of ('monocular', 'stereo', 'rgbd'), got 'lidar'`.
- **Suggested fix:**
  - Add the empty-map case for both functions, plus the bad mode for `fuse_targets`, to `test_rejects_malformed_arguments` (every backend).
  - Add the empty-map case for both functions beside the `cull_keyframes` empty case in the C++ and Rust mapping tests.

#### [note] The root rule lives only in `cull`; `erase_keyframe` still erases a childless keyframe 0
- **Where:**
  - `src/slam_it/edge/covisibility.py:463` (`cull`'s id-0 refusal), `cpp/src/edge/covisibility.cpp:268`, `rust/slam-it-edge/src/covisibility.rs:315`;
  - `erase_keyframe`: `src/slam_it/edge/map.py:1291`, `cpp/include/slam_it/edge/map.hpp:302`, `rust/slam-it-edge/src/map.rs:1081`;
  - local BA's gauge, `src/slam_it/edge/mapping.py:1155` (`m.kf_id == 0` held fixed).
- **Issue:**
  - `erase_keyframe` owns the half of the invariant that mattered for A1: no dangling parent. The CHANGELOG and docs call `cull` "the one way a keyframe leaves the map", but that is a convention for callers above.
  - `erase_keyframe` is public, bound and in the Protocol. It erases any leaf, including keyframe 0, which is the tree's root and local BA's gauge.
  - Probe: `cull(root)` gives `InvalidInputError k: keyframe 0 is the map's first...`. `erase_keyframe` on the same keyframe 0 with its children orphaned succeeds, and the map's lowest id becomes 1. The Rust map unit test does the same (`map.rs:1286`, `erase_keyframe(&mut m, 0).unwrap()`).
- **Why it matters:** this is latent: `cull` is the only caller today (grep over `src`, `cpp/src`, `rust/*/src`). B3 and B4, whose callers were A1's reason, would bypass the root rule by calling the map layer directly.
- **Suggested fix:**
  - No code change now. Say in `erase_keyframe`'s docs that it guards parents, not the root, and that callers above the map go through `cull`.
  - Revisit when B3 or B4 adds a second caller, or when merging renumbers a map's root, since the rule is keyed to id 0.

#### [note] Design section 8 does not yet record who owns the tree's invariant, or what the fixture now holds
- **Where:** `docs/design/2026-10-14-online-slam.md`, section 8: item 3 (the spanning tree) and item 9 (parity).
- **Issue:**
  - Item 3 describes reparenting but not the rules this run added: a keyframe leaves only by `cull`, `erase_keyframe` refuses a parent, and the root is never culled.
  - Item 9 lists what the fixture tests ("the weights, the links, the best, the local map, and a culled keyframe's children") and says each stage "keeps its native tests". The fixture now also holds `cull`, `fuse_targets` and `local_ba_keyframes` in every backend.
  - Neither item contradicts the code; both are now incomplete. B3's design starts from this document, and B3 is the next user of the tree.
- **Suggested fix:** change the doc, not the code. Add one sentence to item 3 and extend item 9's list. This PR already edits the design doc, so it can go here; otherwise directly on `develop` under the docs rule.

#### [note] Mapping's selections are tested in covisibility's test files, in all three languages
- **Where:** `tests/edge/test_covisibility.py:146` (`test_the_stages_take_the_graphs_keyframes`), `cpp/tests/edge/test_covisibility.cpp:20` (now includes `slam_it/edge/mapping.hpp`), `rust/slam-it-edge/tests/covisibility.rs:10` (imports `slam_it_edge::mapping`).
- **Issue:**
  - Sharing the brute-force fixture's maps is right. But the tests of `slam_it.edge.mapping` functions now live in the tests of `covisibility`, the layer below.
  - CLAUDE.md says tests mirror the packages. A reader of `test_mapping.*`, whose docstrings list what they cover, finds no test of `fuse_targets` or `local_ba_keyframes`.
- **Suggested fix:** either name the covisibility tests in the three `test_mapping` docstrings, or move the test functions into the mapping tests, reading the same fixture. Low priority.

### Checked and fine
- **A1 (with L3):** fixed as decided, in all three languages.
  - `cull` exists in Python, C++ and Rust (`covisibility.py:463`, `covisibility.cpp:268`, `covisibility.rs:315`), is bound in both extensions and is in the Protocol.
  - Culling calls it everywhere, and C++'s private helper is gone.
  - `erase_keyframe` refuses a parent with the same message, documented in the Raises section, the header and the rustdoc.
  - The refusal is tested natively (`check_cull` in C++ and Rust, and the Rust map unit test) and through Python. `cull`'s refusal of id 0 is tested in all three.
  - The route by which a native dangling parent reached disk is closed.
- **A2 (with H3):** the seven settings are in `MOVED_OPTIONS`. The local map's two caps cannot move the room's decisions. The fixture's capped local maps hold them, as you decided; a native tracking call that passed the graph a wrong `Options` would stay invisible in the room, as that decision accepts.
- **A3:** the room's chains are walked to the root, and every map of the golden record is one tree rooted at its first keyframe (`test_the_golden_record_keeps_one_tree_a_map`). The doc half: see the first finding.
- **A4:** `covisibility.json` states its oracle and an exact tolerance. The deferred check over every fixture is in the ladder's open items, and its description is accurate: `svd3.json` and `png.json` state neither, and most others state an oracle alone.
- **A5:** all six vocabulary errors say `relocalization.enabled`; `relocalization.py` uses the group names; `runs.hpp` and `runs.rs` describe version 6.
- **A6:** all three pipelines' `close` docs mark the strong-links record as a stand-in until B4 and cite section 4.
- **A7:** the toggle's title and the bundle docstring say "as each map ended". The graph through time is in the ladder's open items.
- **A8 (with H7):** the reference counts one row (`_shared`, `_shared_in`), and `strong_links` keeps the matrix. No boundary or contract changed.
- **A9:** the test-set note says where B1's half comes from, and section 4's B3 outline takes "compare by each batch's own summary".
- **A10:** `_GROUPS` is derived from `get_type_hints(Options)`, filtered to dataclasses: a pure computation at import, with no side effect. The normalising write is waived as recorded.
- **A11 (with H11, L2):** -1 is kept when there is no reference, in all three languages, and tested in each.
- **A12:** the budget line in `edge/__init__.py` points to mapping's load and to B3's design.
- **L1, architecture side:** the stages call the same code the public functions wrap.
  - Python: `_fuse_points` calls `_fuse_targets` (`mapping.py:982`), and `_process` calls `_local_ba_keyframes` (`:1154`).
  - C++ and Rust: `fuse_points` and `fuse_targets` share `targets_of`, and `process` and `local_ba_keyframes` share `local_rows` (internal linkage in C++, private in Rust).
  - The fixture's `_fuse_targets` and `local_ba` come from the generator's own set rules, and every branch is asserted to be exercised (`FUSED`).
- **H1:** `check_newest` is used in all three `cull_keyframes`, and the empty case is tested in each.
- **New public surface:**
  - `fuse_targets` returns ids and `local_ba_keyframes` returns rows. Each is the unit its consumer takes (fusion looks targets up by id; `bundle_adjust` takes rows), and both are documented.
  - Both Protocol entries match the nanobind and PyO3 signatures, confirmed by `test_every_compiled_module_matches_its_protocol` for cpp and rust.
  - The bindings only convert types: int64 through `vector_array`, and `indices_array` for rows.
- **Layering:** `cull` calls `map.erase_keyframe` (covisibility above map), and mapping calls covisibility. `check_layers.py` and import-linter both pass at `c9d4a30`.
- **Error contract:** the new refusals (a parent, the root, an empty map) have the same kind and message in all three languages.
- **Ladder and placement:** nothing moved rungs, no new dependency, Rust stays within the edge VO, and there is no reifyscene, GPU or depth-model import.
- **VERSION and CHANGELOG:** `0.3.11.0` is unchanged within the PR, and the CHANGELOG's new entries match the code.

### Algorithm review

**Scope:** Run 2 of `feature/2b-b2-covisibility`, `390d67d...c9d4a30` (c13a438, 7ce41f2, 64afa25, db9dd1f, 471f7fd, c9d4a30). I read these in full:
- `src/slam_it/edge/covisibility.py` and the diffs of its C++ and Rust twins and bindings;
- the mapping, map, tracking and options diffs in all three languages;
- `tests/fixtures/edge/generate_covisibility.py` and the regenerated `covisibility.json`;
- the new Python, C++ and Rust tests, `MOVED_OPTIONS`, and the design and ladder changes.

My references were the ORB-SLAM2 and ORB-SLAM3 source, read from Run 1's local copies (`KeyFrame.cc` `SetBadFlag`, `SetNotErase` and `SetErase`; `LocalMapping.cc` `SearchInNeighbors` and `KeyFrameCulling`), and the ORB-SLAM paper (Mur-Artal et al., T-RO 2015, https://arxiv.org/abs/1502.00956). Layer 1 was not run.

These tests ran at c9d4a30, with both extensions current with their sources:
- pytest:
  - `test_covisibility.py` and `test_map.py`: 156 passed;
  - the new mapping and tracking tests: 43 passed;
  - `test_vo.py`'s golden-record, off-the-defaults and moved-settings tests: 85 passed;
  - `test_database.py` and `tests/eval/test_runs.py`: 170 passed.
- ctest `edge_test_{covisibility,map,mapping,tracking}`: 4 passed.
- `cargo test -p slam-it-edge` (covisibility, mapping, tracking): 10 passed.

**Summary:** Every Run 1 finding in my lane is fixed as decided, in all three languages. The cull added in this run matches `SetBadFlag`, with keyframe 0 refused. The new row counting gives exactly what the old K×K matrix gave. The fusion brute force follows ORB-SLAM3's order. The moved-settings finding holds, and I checked why: the local map's two caps cannot matter in the room. One gap is left. The fixture's monocular fusion count does not bind, so no fixture test would catch a wrong monocular count. The generator's docstring and 7ce41f2's message both say it binds. There is also one note for B3.

### Findings

#### [should-fix] The fixture's monocular fusion count does not bind, against what the generator and 7ce41f2 say
- **Where:**
  - `tests/fixtures/edge/generate_covisibility.py:103-104`: `FUSION = {"fuse_neighbours": 2, "mono_fuse_neighbours": 3, "fuse_second_neighbours": 2}`, with the docstring "low enough that each binds on the fixture's maps";
  - commit 7ce41f2's message: "with fusion's counts low enough to bind";
  - the three tests that read it: `tests/edge/test_covisibility.py:147`, `cpp/tests/edge/test_covisibility.cpp:169`, `rust/slam-it-edge/tests/covisibility.rs:190`.
- **Issue:** The newest keyframe has 1, 4, 2, 1 and 1 links in the five cases (ties, repair, path-1, path-2, path-3). A monocular count of 3 can only bind in `repair`. There, the one link left out (id 4) returns as a second neighbour, so the targets come out the same. Any monocular count of 3 or more gives every case's fixture answer.
- **Why it matters:** L1 asked for a known-answer test of fusion's targets "with the monocular count". Today a core whose monocular branch hard-codes ORB-SLAM2's 20 passes the fixture, in the native tests and in every backend through Python. Only the whole-run moved test catches it. That test does catch it: I re-ran the leave-one-out, and `mono_fuse_neighbours` put back at 20 changes tum/monocular and euroc/monocular. So the gap is in the known-answer layer and in the stated claim, not in the end-to-end coverage. Under your rule on commit messages, the claim in 7ce41f2 contradicts this finding on an unmerged branch.
- **Evidence:**
  - The module itself, all three backends, monocular targets compared with the fixture, other counts as in the fixture:
    ```
    mono_fuse_neighbours 2 -> every case still matches the fixture: False
    mono_fuse_neighbours 3 -> every case still matches the fixture: True
    mono_fuse_neighbours 4 -> every case still matches the fixture: True
    mono_fuse_neighbours 20 -> every case still matches the fixture: True
    ```
  - A brute-force mutation check, each count put back at its default:
    - `fuse_neighbours=10` changes `repair/other`;
    - `fuse_second_neighbours=5` changes `path-2`;
    - `mono_fuse_neighbours=20` changes no case;
    - ORB-SLAM2's order changes `repair` and `path-1`.
- **Suggested fix:**
  - Use `FUSION = {"fuse_neighbours": 1, "mono_fuse_neighbours": 2, "fuse_second_neighbours": 2}`, or `(2, 1, 2)`. Searching every triple up to (5, 5, 4), these are the only two where each count, put back at its default, changes some case and the monocular targets differ from the others. `FUSED` stays non-zero with either: second 12, newest 6, repeat 4.
  - Add a generator assertion that each count binds, as it already asserts `ADDED` and `FUSED`.
  - Regenerate the fixture. Only `fusion` and `fuse_targets` change.

#### [note] `SetBadFlag`'s second guard belongs in `cull` when B3 brings loop closing
- **Where:** `src/slam_it/edge/covisibility.py:463` (`cull`), `cpp/src/edge/covisibility.cpp` `cull`, `rust/slam-it-edge/src/covisibility.rs` `cull`.
- **Issue:** `SetBadFlag` has two early returns. One is for id 0, which is ported here, as a refusal. The other is `mbNotErase`, which loop closing sets on the keyframes it is testing. The keyframe is then marked `mbToBeErased` and erased later by `SetErase`. `SetErase` never clears the guard for a keyframe that has loop edges, so such a keyframe is never culled (ORB-SLAM2 `KeyFrame.cc:437-451, 459-463`; ORB-SLAM3 `:581-585`). Nothing in the repo mentions it.
- **Why it matters:** Not for B2, which has no loop closing. Once B3 adds loop edges and the essential graph, culling must spare those keyframes. `cull` is now the one way out of the map, which makes it the natural place for that check.
- **Suggested fix:** A line in B3's outline, beside L4 to L6.

### Checked and fine
- **L1:** fixed, apart from the monocular count above.
  - `fuse_targets` and `local_ba_keyframes` are public, bound and in the Protocol in all three languages.
  - The generator, run into a scratch folder, reproduces the committed fixture exactly.
  - Its brute force matches ORB-SLAM3's `SearchInNeighbors`: the first level whole, then the second neighbours of the first level only (`imax` fixed), skipping the newest and repeats (`LocalMapping.cc:717-746`). The counts are ORB-SLAM2's (`:457-478`).
  - The keyframe-decision test passes the same frame against two references and requires opposite answers, so a rule that ignores the reference fails one of them. Its count follows `TrackedMapPoints(nMinObs)`, including the `nKFs<=2` case.
  - The golden-record tree test covers 40 maps and 48 culled keyframes (24 in window/tum/rgbd). It checks one root, the map's first; living same-map parents; and every chain reaching the root within |alive| steps.
  - Local BA's fixed set is the existing adjust rule (every other observer, plus id 0), tested in `test_adjust.py`.
- **L2 (H11, A11):** -1 is kept in all three languages and tested in all three (`test_tracking.py`, `test_tracking.cpp`, `tests/tracking.rs`).
- **L3 (A1):** fixed.
  - `erase_keyframe` refuses a keyframe that is still a parent, documented in all three. `cull` is its only caller in the cores (grep).
  - `cull` refuses id 0, as `SetBadFlag` does (ORB-SLAM2 `if(mnId==0) return`; ORB-SLAM3 `GetInitKFid()`). Raising is better than ORB-SLAM's silent return, and culling skips id 0 first, so a run never hits it.
  - Random check: 200 seeded maps, every backend. `cull` equals the brute-force reparent followed by the erase. Id 0 was refused 600 times and erasing a parent 3,762 times, with 0 mismatches. The native refusals, which the Python wrappers pre-empt, are held by `check_cull` in the C++ and Rust tests.
- **H7/A8, the row counting:** correct.
  - `_shared_in`'s index works out to `start_i + j` within each point's run. `_observing` and `_pairs` remove repeated observations, as `_weights` did, and handle empty input.
  - The 390d67d reference and the new one were compared row for row: links, adopt, reparent and local_map, on 200 maps. The maps included 724 keyframes with a repeated observation, keyframes with no point, and empty frames. There were 0 differences.
  - A second check put every backend against the brute force on 120 maps with repeated observations: 11,694 comparisons, 0 mismatches.
  - The golden record passes in every backend.
  - Timing on the hygiene reviewer's map at 50, 200 and 500 keyframes: links 2.9, 3.3 and 3.4 ms; local_map 12.2, 38.9 and 89.6 ms. These agree with 64afa25.
- **H3/A2, the moved settings:** the leave-one-out reproduces (`moved_settings_bite.py`, C++):
  - fusion's three counts, `min_points` and `strong_points` each change 2 to 4 cases;
  - the two caps change nothing.

  The reason given is right. I wrapped the Python reference's `_local_map` over the gap room's four Python runs at `MOVED_OPTIONS`. In all 112 calls, K1 was already every keyframe of the map (at most 10), K2 added none, and caps of (1, 1) gave the same local map as (80, 10). The fixture holds each core's reading of the caps. That tracking passes its settings on to `local_map` is checked by reading the code only (`tracking.cpp:300`, `tracking.rs:836,962`).
- **H1:** `check_newest` runs before the newest keyframe's links are read, and the empty map is tested in all three languages.
- **A3:** `adopt`'s docs are accurate, and both the room's chains and the golden record's are walked to the root.
- **A4:** the fixture states its oracle and an exact policy, and the tests compare exactly.
- **H9:** the group test (common words lowered, at least 2 scored, the group's score above its own) holds the accumulated score of `DetectRelocalizationCandidates`.
- **L4:** the new docstring paragraph matches the source (the `UpdateBestCovisibles` re-sort; `KeyFrameCulling` at `:909` and `count>100` at `:1049`), and the cap of 100 is in B3's outline.
- **L5, L6:** both are in B3's outline (online-SLAM design, section 4), as decided.
- **L7:** waived. Run 2 adds no new method, so Run 1's state-of-the-art note stands.

Scripts saved for reuse (outside the repo):
- `covisibility_old_vs_new_and_stages.py`: the old reference against the new, plus cull, fusion targets and local BA's keyframes in every backend against the brute force.
- `local_map_caps_probe.py`: whether the local map's caps can matter on the gap room.

The throwaway probes are in `/tmp/scratch/algo2/`. The repository was not modified (`git status` clean). Importing `slam_it` ran a no-op cmake install into the git-ignored `build/`.
