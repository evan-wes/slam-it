# Review panel: `feature/2b-b4-merging-maps`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N".
     Every path is relative to the repository root, never absolute or home-relative. -->

## Run 1 -- 2026-10-19

| | |
|---|---|
| **Branch** | `feature/2b-b4-merging-maps` |
| **Range** | `be801e8a09c97b59b26a4de9ee3f7f1f805f7bdb...7dfd04f2513a5bf33c1663ec53cced0b4d3bc210` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh --env slam-it-b4`, exit 0, at `7dfd04f2`; pytest 5,058 passed, 1 skipped, 17 deselected) |

The owner's brief: the threaded merge is option B, accepted for this PR; findings about the merge
design itself are tagged "follow-up: merge design" and go to the follow-up that reviews map merging
against ORB-SLAM3 and the other reference systems. Section 15's measurements and the recorded known
gaps were deferred before the panel.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A1 | must-fix | architecture | `viewer/src/frames.js:161`, `src/slam_it/orchestration/bundles.py:250`, `src/slam_it/eval/runs.py:1232` | Run format 9 lets a map's frames go back and come apart; the bake, viewer and run summaries still assume one rising stretch per map (a merge's joined stretch is drawn blank) | fix | fixed (85e45bbd, c5ded424) -- decided by evan-wes, 2026-10-19 |
| A2 | must-fix | architecture | `rust/slam-it-edge/src/pipeline.rs:2398` | Rust's loop closing holds the atlas mutex through each keyframe's search, so threaded tracking waits on loop detection | fix | fixed (d795af2f) -- decided by evan-wes, 2026-10-19 |
| L1 | must-fix | algorithm | `src/slam_it/edge/atlas.py:421`, `cpp/src/edge/atlas.cpp:204`, `rust/slam-it-edge/src/atlas.rs:436` | Monocular merge: the merged-away map's frames keep offsets in its old units in the final trajectory | fix | fixed (d0235820) -- decided by evan-wes, 2026-10-19 |
| L2 | must-fix | algorithm | `src/slam_it/edge/pipeline.py:1374`, `cpp/src/edge/pipeline.cpp:107`, `rust/slam-it-edge/src/pipeline.rs:1908` | A loop being confirmed is carried across a monocular merge in the old map's units | fix | fixed (d0235820) -- decided by evan-wes, 2026-10-19 |
| H1 | should-fix | hygiene | `src/slam_it/edge/loops.py:982`, `:1156` | Python's loop and merge detection written twice; the native cores share one driver | defer (merge-design follow-up, with A15's split) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| H2 | should-fix | hygiene | `rust/slam-it-edge/src/pipeline.rs:3303`, `cpp/src/edge/atlas.cpp:188`, `src/slam_it/edge/atlas.py:391` | A kept map that is not there: three contracts, Rust swallows it, `0` defaults in close/merge | fix | fixed (cf368ef4; Rust's `Kept` in d795af2f) -- decided by evan-wes, 2026-10-19 |
| H3 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:712`, `:599`; `src/slam_it/edge/atlas.py:421`; `scripts/compare_testsets.py:133` | Stated behaviours with no direct test (`held()`, `TrackingState.merged` scaled, the atlas's merge, multi-folder offsets) | fix | fixed (d0235820, 1813a0d1, 7bdad99b) -- decided by evan-wes, 2026-10-19 |
| H4 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:2128` and twins | `start_session` does not refuse a finished run | fix | fixed (1813a0d1) -- decided by evan-wes, 2026-10-19 |
| H5 | should-fix | hygiene | `src/slam_it/edge/loops.py:2042`, `src/slam_it/edge/pipeline.py:1358` and twins | `_offset` written twice per language; two different `_scaled` | fix the `_scaled` name; defer the shared helper to A4 | fixed (f08ac701); the shared id-offset helper deferred (A4, the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| H6 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:1133`, `:1205`; `src/slam_it/orchestration/vo.py:827` | Positional tuples past readable (`_place`, `_locate`, `_Turns`) | fix | fixed (f08ac701) -- decided by evan-wes, 2026-10-19 |
| H7 | should-fix | hygiene | `src/slam_it/edge/options.py:582` and twins | The weld settings' docs only make sense read together | fix | fixed (f08ac701) -- decided by evan-wes, 2026-10-19 |
| H8 | note | hygiene | `src/slam_it/edge/pipeline.py:2094`, `:1706` | Small readability items | fix the trivial ones | fixed (f08ac701) -- decided by evan-wes, 2026-10-19 |
| H9 | note | hygiene | `src/slam_it/edge/atlas.py:743` | `Atlas._runs_of` rebuilds an ndarray every call | defer (Python is not held to real time) | deferred (the ladder's open item "Python's run-id tables rebuilt per call (H9)") -- decided by evan-wes, 2026-10-19 |
| H10 | note | hygiene | `src/slam_it/edge/loops.py:2349` | `Crossing` defined three times in `edge` | fix | fixed (f08ac701) -- decided by evan-wes, 2026-10-19 |
| H11 | note | hygiene | `tests/conftest.py:31` | The pytest collection workaround cites no upstream issue | fix (comment) | fixed (7bdad99b) -- decided by evan-wes, 2026-10-19 |
| H12 | note | hygiene | `scripts/compare_testsets.py:133` | Session offsets computed in two places | fix | fixed (7bdad99b) -- decided by evan-wes, 2026-10-19 |
| H13 | note | hygiene | `src/slam_it/orchestration/bundles.py:459` | The bake refuses a multi-session run without saying why | fix (message and docstring) | fixed (7bdad99b) -- decided by evan-wes, 2026-10-19 |
| H14 | note | hygiene | `src/slam_it/edge/pipeline.py:908` | follow-up: merge design (database re-quantised in the pause; bespoke id remap; unlocked kept read) | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| A3 | should-fix | architecture | `src/slam_it/edge/pipeline.py` (`_switch`) | No pipeline test takes up a kept map, in any language or scheme | fix | fixed (488b1a42) -- decided by evan-wes, 2026-10-19 |
| A4 | should-fix | architecture | `src/slam_it/edge/pipeline.py`, `loops.py:2010` and twins | follow-up: merge design -- a merge's id and scale remap re-implemented by each consumer | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| A5 | should-fix | architecture | `docs/design/2026-10-02-slam-it-ladder.md:1189`, `:1163` | A15 and A9 records understate what B4 added | fix (the records); the splits go to the follow-up | fixed (7080221c) the records; the splits deferred, the first step of the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)" -- decided by evan-wes, 2026-10-19 |
| A6 | should-fix | architecture | `docs/design/2026-10-02-slam-it-ladder.md:546` | The ladder still describes relocalisation in the active map only | fix | fixed (7080221c) -- decided by evan-wes, 2026-10-19 |
| A7 | should-fix | architecture | `src/slam_it/edge/loops.py:3289`, `:3411` | Python's `detect_merge` and `merge` leave GTSAM's failure out of their error contract | fix | fixed (f08ac701) -- decided by evan-wes, 2026-10-19 |
| A8 | note | architecture | `cpp/include/slam_it/edge/atlas.hpp:84` | follow-up: merge design -- the kept maps shared three ways; C++'s doc contradicts itself | fix the C++ doc; defer the design | fixed (cf368ef4) the C++ doc; the sharing design deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)", with H14) -- decided by evan-wes, 2026-10-19 |
| A9 | note | architecture | `src/slam_it/edge/pipeline.py:250` | `LostSummary` now names any closed map | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| A10 | note | architecture | `scripts/measure_testset.py` | The measure's config keeps `folder` beside `folders` | fix | fixed (7bdad99b, c5ded424) -- decided by evan-wes, 2026-10-19 |
| L3 | note | algorithm | `src/slam_it/edge/loops.py:2326` | follow-up: merge design -- the essential graph moves points the weld just solved | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| L4 | note | algorithm | `src/slam_it/edge/pipeline.py:1742` | follow-up: merge design -- threaded merges skip global BA unless one was superseded; the pause includes the whole-map graph | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| L5 | note | algorithm | `src/slam_it/edge/relocalization.py:703` | Relocalisation candidates grow with the kept maps; a healthy map can be left | defer (section 15's measurements) | deferred (the ladder's "B4's measurements, not run"; the 2B design's section 15) -- decided by evan-wes, 2026-10-19 |
| L6 | note | algorithm | `src/slam_it/edge/loops.py` (`detect_merge`) | A monocular merge's scale is unbounded and cannot be undone | defer (merge-design follow-up) | deferred (the ladder's open item "Map merging reviewed against the reference systems (the follow-up to B4)") -- decided by evan-wes, 2026-10-19 |
| L7 | note | algorithm | `src/slam_it/edge/loops.py:2178` | The weld inherits B3's deferred damping issue | defer (the damping PR measures welds too) | deferred (the 2B design's section 11, B3's damping item, L2) -- decided by evan-wes, 2026-10-19 |
| L8 | note | algorithm | `tests/foundations/test_graphs.py` | The Huber test is behavioural, not a known answer | fix | fixed (5ffcf500) -- decided by evan-wes, 2026-10-19 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

**Run 1 decisions** (evan-wes, 2026-10-19): each of the four must-fix findings fixed (A1, A2,
L1, L2), with A1's choice for a merged-away map: the header leaves it out, its frames the map it
merged into. The cheap should-fix findings and notes fixed (H2 to H8, H10 to H13, A3, A5's
records, A6, A7, A8's doc, A10, L8). The merge design's findings (H1, H14, A4, A9, L3, L4, L6,
with A5's splits as its first step and A8's sharing design) go to one ladder open item, the
follow-up that reviews map merging against the reference systems, where option C is one outcome
beside an ORB-SLAM3-faithful staged merge; L5 to section 15's measurements, L7 to B3's damping
PR, H9 to a minor open item. The fixes are commits d0235820 to c5ded424 on this branch, the
version still 0.3.17.0. Layer 1 at c5ded424: all checks passed (`scripts/check.sh --env
slam-it-b4`, exit 0; pytest 5,095 passed, 1 skipped, 19 deselected; the ThreadSanitizer rooms 6
passed). Its run before, on the same code, failed three threaded C++ monocular cases (the loop
room's two, its final ATE 6.61 cm against 2 cm, the known flaky case in the ladder's B3b items;
the merge room's, its first frame after the merge placed by relocalisation, not the motion
model), each of which then passed three runs in isolation.

### Hygiene review

**Scope:** B4 (milestone 2B, merging maps), `be801e8a...7dfd04f2` (v0.3.16.2 to v0.3.17.0). I read in full or by diff:
- Python: `src/slam_it/edge/{atlas,loops,pipeline,database,relocalization,map,options,covisibility}.py`, `src/slam_it/foundations/graphs.py`, `src/slam_it/eval/{runs,loops,sevenscenes}.py`, `src/slam_it/orchestration/{vo,bundles}.py`.
- C++ and Rust: the atlas, the merge and detection code, the pipeline's merge, switch and session paths, the bindings, and the options.
- Scripts and tests: `scripts/{measure_testset,compare_testsets,measure_sharded,check.sh}`, the new and changed tests, and the viewer diff.
- Golden records: shape only, as briefed.

I did not re-report anything layer 1 enforces, the deferred items you listed, or the Python/C++/Rust parallelism.

**Summary:** The change is clean, thoroughly documented, and tested by value with named tolerances. I found no must-fix. The weak spots are:
- The Python reference duplicates the loop/merge detection driver that the C++ and Rust cores already share.
- Three languages, three different contracts for "the kept map you asked for is absent", and Rust swallows it.
- A few stated behaviours (culling spares a merge waiting, `TrackingState.merged` scaling, the atlas carrying records through a merge) have no direct test.
- `start_session` has no finished-run guard.

Findings tagged "follow-up: merge design" are for the ORB-SLAM3 design pass, not for B4.

#### Findings

##### [should-fix] Python's loop and merge detection are written twice; the native cores share one driver
- **Where:** `src/slam_it/edge/loops.py:982` (`_search`) and `:1156` (`_search_merge`); `:1010` (`_detect`) and `:1200` (`_detect_merge`). Compare `cpp/src/edge/loops.cpp:1169` (`detect_region`, called at `:2178` and `:2530`) and `rust/slam-it-edge/src/loops.rs:2018` (`detect_region`, used by `detect` at `:2112` and `detect_merge` at `:2214`).
- **Issue:** `_detect_merge` repeats `_detect` step for step: the `min_keyframes` skip, `_carry` the pending region, the `dropped` and `held` bookkeeping, the `Outcome.DROPPED` or `NONE` branch, building `Pending(...)` for a started region, and `_settle`. The differences are the searched map and database (`_Searched`), the candidate source, and wrapping the pending in `Merging`. `_search_merge` is `_search` with a candidate source and map number swapped in. C++ and Rust already factored this into `detect_region`, so the reference no longer has the structure the cores say they mirror ("Python's `_detect_merge`, step for step").
- **Why it matters:** A fix to confirmation or drop logic (the `held` and `misses` rules) must be made in two places. The copies can drift, and jscpd cannot see it because the identifiers differ. The Python-vs-native comparison tests then fail on a drift they cannot localise.
- **Evidence:** `_detect` `:1019-1054` and `_detect_merge` `:1219-1253` differ only in `_Searched(m, db, own=True)` against `_Searched(other.map, other.database, own=False)`, `pending` against `pending.pending`, and `_settle` against `_settle_merge`.
- **Suggested fix:** Extract one `_detect_in(...)` taking the searched-map resolver and a `search` callable. `_detect` and `_detect_merge` become thin wrappers that add the map number. `_merge_candidates` and `_relocalize_across` (`database.py`, `relocalization.py`) also share the `(score, number, keyframe)` collect-and-sort idiom with the same `(-c[0], c[1], c[2])` key. That can share a helper at the same time.

##### [should-fix] Looking up a kept map that is not there has three different contracts, and two of them hide it
- **Where:**
  - Python: `src/slam_it/edge/atlas.py:391` and `:424` (`self._kept.pop(number)`, a bare `KeyError`).
  - C++: `cpp/src/edge/atlas.cpp:188-201` (`kept_at`, `resume`, `merge`).
  - Rust: `rust/slam-it-edge/src/pipeline.rs:3303-3306`; `rust/slam-it-edge/src/atlas.rs:414` and `:436`.
  - `value_or(0)` / `unwrap_or(0)`: `cpp/src/edge/atlas.cpp:150` and `:206`; `rust/slam-it-edge/src/atlas.rs:364` and `:438`.
- **Issue:**
  - C++ `kept_at` returns `kept_.size()` when the number is absent and the caller then indexes `kept_[at]`. That is out of range, and `atlas.hpp` states no precondition for `resume` or `merge`.
  - Rust `Atlas::resume` returns `Option`, and `switch` ignores `None`: `if let Some(kept) = kept { shared.owner.resume(...) }`, then carries on with `tracking.change_map()`. The map left has already been closed, so tracking continues in no map and reports nothing.
  - Python raises a `KeyError` that does not name the map.
  - `close` and `merge` add to the existing `map_id.value_or(0)` / `unwrap_or(0)` use. If no map is current, `merged_into[0] = matched` is silently recorded. Python asserts here (`_current()`, `atlas.py:731`).
- **Why it matters:** The pipeline only passes numbers it just got from `atlas.indexed()`, so this is an internal invariant. If it is ever broken, C++ aborts or has UB, Rust carries on with a corrupt atlas, and Python gives an unhelpful `KeyError`. The standard is "failure observable, no ignored results".
- **Evidence:** Rust (`pipeline.rs:3303`): `let kept = lock(&shared.atlas).resume(number); if let Some(kept) = kept { shared.owner.resume(kept.map, kept.database); } tracking.change_map();`. C++ (`atlas.cpp:188`): `return static_cast<std::size_t>(std::ranges::find(kept_, number, &Indexed::number) - kept_.begin());`.
- **Suggested fix:**
  - Pick one contract: "unknown number is `InvalidInput` (Rust `fail`, C++ `Result`, Python `InvalidInputError` naming the number)".
  - Make Rust `switch` propagate it, and replace the `0` defaults in `close` and `merge` with the same error or an assert.
  - Rust and C++ also keep a kept map's run ids in a parallel vector (`kept` / `kept_runs`, `kept_` / `kept_runs_`), edited in lockstep at three sites. Python's `Kept` bundles them. Make the native `Kept` a struct holding both, which removes the lockstep bookkeeping.

##### [should-fix] Stated behaviours with no direct test
- **Where / what:**
  1. `LoopState.held()` (`src/slam_it/edge/pipeline.py:712`; Rust `rust/slam-it-edge/src/pipeline.rs:663`; C++ `cpp/src/edge/pipeline.cpp:1355`). No test sets `merging` or `waiting` and reads `held()`. The CHANGELOG states "culling spares the last keyframe of a merge being confirmed" and the design spares a merge waiting's current keyframe (`SetNotErase`).
  2. `TrackingState.merged` and `change_map` (`pipeline.py:599`, `:587`) and their C++ and Rust twins. Python's in-process threaded merge test is RGB-D only (`tests/orchestration/test_merge_room_threads.py:83`), so `G`'s scale is 1.0 and the velocity and `anchor_relative` scaling is never exercised away from 1. The native threaded monocular runs assert only the room's bar and `placed_by == MOTION`, which a wrong scale (or its inverse) would likely still pass, because the motion model is only an initial guess.
  3. `Atlas._merge` (`src/slam_it/edge/atlas.py:421-446`) and its twins carrying `_past_points`, `_past_keyframes` and the culled relatives (`/= scale`) into the matched world. There is no `tests/edge/test_atlas.py`; only the merge room's final ATE depends on it.
  4. `scripts/compare_testsets.py:133-137` (multi-folder offsets) has no test; `tests/test_compare_testsets.py` only adapts to the `folders` rename.
- **Why it matters:** (1) If a waiting merge's current keyframe is culled before tracking makes the merge, `merge_transform` raises "not in the map" and the run ends. (2) and (3) are exactly the monocular, scale-carrying paths where a sign or inverse slip passes on rigid inputs.
- **Suggested fix:**
  - A unit test per language for `held()` over the pending, merging and waiting combinations.
  - One `TrackingState.merged` test with `G` scaled (assert the translation scaled, the rotation unchanged, ids offset, `None` motions kept).
  - One `Atlas` test that merges a small hand-built record set with a scaled `G` and checks the carried past points, poses and culled relatives.

##### [should-fix] `Pipeline.start_session` does not refuse a finished run, while `track` does
- **Where:** `src/slam_it/edge/pipeline.py:2128` (the finished guard in `track` is `:1958`); `rust/slam-it-edge/src/pipeline.rs:3188` (guard at `:3023`); `cpp/src/edge/pipeline.cpp:1763` (guard at `:1644`).
- **Issue:** After `finish()`, `start_session()` runs `_leave`, which closes the map through `atlas.close(..., reset=...)`. A small map is reset, which rewrites its lives and drops its frames from `trajectory()`, on a run that is meant to be over. The docstring's `Raises` section does not mention it.
- **Why it matters:** It is a public method. The only caller (`vo`) is correct today. The misuse is silent, and a one-line guard matches the sibling method.
- **Suggested fix:** Raise `InvalidInputError("start_session: the run is finished")` in all three languages, add it to `Raises`, and test it beside the existing `start_session` tests.

##### [should-fix] `_offset` is written twice per language
- **Where:**
  - Python: `src/slam_it/edge/loops.py:2042` and `src/slam_it/edge/pipeline.py:1358` (byte-for-byte the same).
  - Rust: `rust/slam-it-edge/src/loops.rs:3221` (returns a `Vec`) and `rust/slam-it-edge/src/pipeline.rs:1822` (in place).
  - C++: `cpp/src/edge/loops.cpp:1641` (returns) and `cpp/src/edge/pipeline.cpp:78` (in place).
- **Issue:** The same "move ids on by an offset, -1 kept" in two forms. A related trap: Python's `_scaled` in `loops.py:2010` scales a similarity's whole top three rows, while `_scaled` in `pipeline.py:1349` scales only a motion's translation. Two different functions have one name, in modules that import each other.
- **Why it matters:** Id moving is the heart of this change, since every holder of ids must remap them across a merge. A missed edit in one copy is hard to see.
- **Suggested fix:** One public helper next to the map code (e.g. `maps.offset_ids`) used by both modules in each language. Rename the pipeline's `_scaled` to `_scale_translation`.

##### [should-fix] Positional tuples have grown past readable
- **Where:** `src/slam_it/edge/pipeline.py:1133` (`_place` returns a 6-tuple: frame, summary, matched, wait, view, moved), `:1205` (`_locate`, a 5-tuple); `:1458` (`_close_loops`, a 3-tuple); `src/slam_it/orchestration/vo.py:827` (`_Turns`, a triple of lists unpacked positionally, with `turns[0][0].camera` at `:866`).
- **Issue:** `_place` went from 5 to 6 positional values and the call sites unpack by position. The same file already has `_Tracking` for `_track_frame`'s result, so a named result is the local convention.
- **Why it matters:** Adding a seventh value is a silent reordering risk. `turns[0][0].camera` is unreadable.
- **Suggested fix:** A frozen dataclass or `NamedTuple` for `_place`/`_locate`'s result and for `_Turns` (`sources`, `offsets`, `order`).

##### [should-fix] The weld settings' docs only make sense read together
- **Where:** `src/slam_it/edge/options.py:582-589`, `cpp/include/slam_it/edge/options.hpp`, `rust/slam-it-edge/src/options.rs` (`weld_neighbours`, `weld_widen`, `weld_rounds`).
- **Issue:** One sentence is spread across three attribute docs: "...widened to this many when it holds fewer...", then "...each member's best this many links taken in...", then "...at most this many times". Each attribute's doc starts or ends with an ellipsis.
- **Why it matters:** An IDE or Doxygen hover on `weld_widen` shows an orphan fragment, and the numpy-doc convention is one self-contained entry per attribute.
- **Suggested fix:** Give each setting a sentence that stands alone, and keep the three-step explanation in the class docstring.

##### [note] Small readability items
- `Pipeline._leave(dated, small)` (`pipeline.py:2094`) is called with `reset_keyframes + 1` to turn a `<=` rule into a `<` rule. A `reset_below` or `max_reset` parameter would not need the `+ 1`.
- `_Threads.merge_waiting` (`pipeline.py:1706`) calls `self.generation.next()` twice in a row, once to supersede the running BA and once to start the new generation. Add a one-line comment.
- Python's `Indexed` ("a map with its database and number") says nothing about what it is; it sits beside `Kept`, which is almost the same thing plus run ids. Consider `NumberedMap` or documenting the relation.
- Cosmetic: `tests/edge/test_pipeline.py` (around line 296) lost its parametrize layout, now one value per line with a trailing comment. The `stride` JSDoc in `viewer/src/bundle.js:215` wraps awkwardly after the edit.

##### [note] `Atlas._runs_of` rebuilds an ndarray from a list on every call
- **Where:** `src/slam_it/edge/atlas.py:743-749`, via `run_ids` / `point_run_ids`. `_Step.fates` makes up to three calls per frame, `_track_frame` makes one, and a keyframe adds one more.
- **Issue:** `np.array(runs, dtype=np.int64)` costs O(map size) per call. It replaces the old `base + ids`, which was O(frame size).
- **Evidence:** I measured 2.3 ms for 100,000 entries and 3.5 ms for 300,000 (conda env `slam-it`).
- **Why it matters:** About 10 ms per frame on a large map. Python is the reference and not held to real time, so it is only a note.
- **Suggested fix:** Keep the run-id tables as growing ndarrays (amortised doubling), or cache the array and invalidate it in `_extend`.

##### [note] `Crossing = dict[str, Any]` is now defined three times in `edge`
- **Where:** `src/slam_it/edge/loops.py:2349`, `mapping.py:64`, `relocalization.py:73` (B4 added the third).
- **Suggested fix:** One definition in `slam_it.backends`.

##### [note] `tests/conftest.py:31` `pytest_configure` is a workaround for pytest 9.1.1's collection bug
- **Where:** `tests/conftest.py:31`; pytest is pinned at 9.1.1 in `scripts/locks/slam-it.lock`.
- **Issue:** It mutates `config.args` in place for a third-party bug. It is untested, cites no upstream issue, and says nothing about when to remove it. It is also outside B4's goal, although it is its own commit (`7a14f3a4`).
- **Suggested fix:** Link the upstream issue or pytest version, and add a one-line test with node-ID arguments.

##### [note] Session offsets are computed in two places
- **Where:** `scripts/compare_testsets.py:133-137` special-cases one folder as `[0.0]`, while `scripts/measure_testset.py:515` always calls `vo.session_offsets`.
- **Suggested fix:** One `measure_testset.offsets_of(entry)`; `vo.session_offsets` already returns `[0.0]` for one folder.

##### [note] `bundles.py` cannot bake a multi-session run, and its message does not say so
- **Where:** `src/slam_it/orchestration/bundles.py:459-460`; the run's `sequence` is `a+b`.
- **Issue:** The bake refuses with "the runs are of a+b, not <folder>". The docs and the CHANGELOG say nothing about it, although B4's headline feature is the viewer drawing merges.
- **Suggested fix:** Say it in the module docstring, or name the cause in the message. Not a B4 blocker.

##### [note] follow-up: merge design
Not for B4. Three observations from reading option B, for the ORB-SLAM3 pass.
- **Cost inside the pause:** `MapOwner.apply_merge` calls `_sync_database` (`src/slam_it/edge/pipeline.py:908`, `:989`), which re-adds every absorbed keyframe to the matched map's database. `Database.add` re-quantises each keyframe's descriptors through the vocabulary tree (`database.py`, `quantize` and `bow`) while tracking waits. Moving the already-computed vectors with their ids offset would avoid it. ORB-SLAM3 avoids it because its keyframe database is atlas-wide.
- **Bespoke id remapping:** Per-map ids plus run-id tables mean every holder of ids needs its own remap across a merge: `_carried`, `TrackingState.merged`, `MapOwner.apply_merge`'s anchors, the `_offset` helpers, `Merged.{keyframe,point}_offset`. ORB-SLAM3's atlas-unique `mnId` needs none of that. B4 chose the per-map numbering (design section 14, item 1), but the remap cost is spread over three languages.
- **Unlocked read of `kept`:** C++ `Atlas::kept()` (`cpp/include/slam_it/edge/atlas.hpp`) is an unlocked span. The protocol it relies on is that changes happen only with the threads settled or drained, which holds under option B and ThreadSanitizer is clean. Option C (merging without blocking tracking) breaks that assumption, so the atlas will need a real read protocol.

#### Checked and fine
- **Python docs:**
  - Docstrings are numpy-style and state shapes, frames, dtypes and the `S_cw` / `T_cam_world` conventions throughout `atlas.py`, the merge section of `loops.py`, `database.py` (`Indexed`, `MergeCandidates`) and `relocalization.py` (`Across`).
  - Module docstrings state purpose and ladder position, including the new `sevenscenes.py`.
  - C++ Doxygen (`@brief`, `@return` with the `InvalidInput` cases) and Rust rustdoc are complete and consistent with Python on the new items.
- **Validation at public boundaries:**
  - `detect_merge`, `merge`, `merge_candidates`, `relocalize_across` and `bundle_adjust(huber=...)` check duplicate map numbers, unknown pending map, missing keyframes, pyramids, vocabularies and non-positive thresholds, with messages that name the value.
  - `Options` allows `loss.relocalize_seconds` 0 but not negative or non-finite (all three languages).
  - `read_config`, `vo.check` and the `--sequence` parsing refuse bad input.
- **Library hygiene:**
  - No debug prints, TODOs, commented-out code or hard-coded paths in added lines (`/home/`, `/tmp/`, `~/` searched).
  - Only one `except` was added (`sevenscenes.read_pose`, re-raised as `InvalidInputError ... from None`).
  - Every Rust `#[allow]` and C++ `NOLINT` carries a reason.
  - Rust library code has no `unwrap` or `expect` on fallible input beyond the documented id conversions in `atlas.rs`.
  - Magic numbers are named and justified (`_RIGID_TOLERANCE`, `INTRINSICS`, `FPS`, `MERGE_*`, the room bars).
- **Tests:**
  - They assert on values with documented tolerances and cover edge cases: duplicates, an unknown map, differing pyramids, no map, vocabulary mismatch, a small map skipped, a merge dropped after two misses, and wrong places judged.
  - Each compiled backend is compared with the reference.
  - The ThreadSanitizer helper is shared (`run_sanitized`) rather than copied.
  - The session offsets have doctests and a native twin.
- **Rebase resolutions:**
  - `src/slam_it/orchestration/bundles.py` is coherent: version 5, the file list and the "bake again" note agree with `VERSION = 5`.
  - `viewer/src/scene_view.js` is coherent: the header comment, `living`, `graphObjects` and the new merge line all agree.
  - No stale "bundle version 4" or "run format 8" references remain outside history.
- **Fixtures:** the three `room_runs_*.json` have the same shape, 27 cases each and no machine paths.
- **`sevenscenes` CLI:** follows the sibling importers' `main` convention.
- **Version sources:** `VERSION` and `pyproject.toml` agree. The `loops` and `pipeline` module sizes are the deferred A15; I did not re-raise them.

### Architecture review

**Scope:** `feature/2b-b4-merging-maps`, `be801e8a...7dfd04f2` (126 files). I read the full diffs for the edge rung (`atlas`, `loops`, `database`, `relocalization`, `map`, `options`, `pipeline`) in Python, C++ and Rust. I also read the full diffs for the rung-0 Huber loss (`graphs`), the C++ and Rust bindings, run format 9 (`src/slam_it/eval/runs.py`, `cpp/src/eval/runs.cpp`), bundle 5 (`src/slam_it/orchestration/bundles.py`, `viewer/src/bundle.js`, `viewer/src/scene_view.js`), sessions (`src/slam_it/orchestration/vo.py`, and the native programs where needed), `src/slam_it/eval/sevenscenes.py`, `scripts/measure_testset.py`, `pyproject.toml`, `scripts/check.sh` and the two design docs. I also read the viewer's unchanged map logic (`viewer/src/frames.js`, `loops.js`, `geometry.js`) and `eval.runs.tracking`, because they consume run format 9.

**Summary:**
- **What is sound:** the slice is built in all three languages. The merge's public API is mirrored in all three, and the bindings stay thin. The Huber loss sits in rung 0 behind the existing GTSAM bridge. Run format 9 is written the same way by all three writers. The threaded merge reuses B3b's one stop/release handshake.
- **Two must-fix problems:**
  - Run format 9 drops the rule that maps rise and that each map's frames form one stretch. The bake and the viewer still depend on that rule. A run that merges (the merge room itself) or that relocalises into a kept map is drawn wrong or shows an empty scene.
  - In Rust, the threaded loop closing holds the atlas's mutex through every keyframe's search. Threaded tracking, which takes that mutex on every frame, therefore waits on loop detection. C++ and Python do not.
- **Smaller points:** a test gap on taking up a kept map; the merge's id remap is copied into every consumer (follow-up: merge design); and the ladder's records are stale, including how much B4 grew the A15 and A9 modules.

#### Findings

##### [must-fix] Run format 9 lets a map's frames go back and come apart, but the bake, the viewer and the run summaries still assume one rising stretch per map
- **Where:**
  - `viewer/src/frames.js:161-169` (`mapRows`, a binary search over `poseMaps`), used at `viewer/src/scene_view.js:379` and `:588`;
  - `viewer/src/loops.js:271-286` (`cycleRows`);
  - `viewer/src/geometry.js:133-160` (`mapDepths`: a revisited map's depth is overwritten by its last stretch);
  - `viewer/src/bundle.js:120,132` ("never descending");
  - `viewer/src/frames.js:128-143` (`mapsShown` picks the map shown from the records' `map`, the map the frame was tracked in);
  - `src/slam_it/orchestration/bundles.py:250-268`:
    - `_maps_info`: `frames` spans the frames tracked in the map, `tracked` counts the frames that end in it, and a map merged away is absent from the header although the records name it;
    - lines 327-331: the records' `map` column is copied unchanged.
  - `src/slam_it/eval/runs.py:1232-1236` (`tracking()` makes one segment per map, from its first to its last time; the worker's status page shows these);
  - `src/slam_it/eval/runs.py:24-27` (`trajectory.txt` documented as each map's poses "in turn", with a `# map` line "before each map's first").
- **Issue:** Section 14 item 7 and `src/slam_it/eval/runs.py:57-59` state that a run's maps "need not rise, nor a map's frames be one stretch". The writers' checks were relaxed to match. Their consumers were not updated:
  - **Relocalisation into a kept map:** after this change, `pose_maps` goes back (0, 1, 0). `mapRows` then returns the wrong rows.
  - **A merge:** the records name the map each frame was tracked in, while every bundle array (poses, keyframes, points) holds the map it ends in. For the frames between a new map's start and its merge, the viewer shows a map that has no rows, keyframes or points. The scene is empty. This happens in the merge room, the PR's own scenario. `tests/orchestration/test_bundles.py::test_a_merge_is_baked_as_made` bakes exactly that run, but checks only the merge array.
  - **Coverage:** neither golden bundle (`tests/fixtures/orchestration/bundle`, `found_bundle`) has a merge or a revisited map.
- **Why it matters:**
  - Item 7's "the viewer draws the merges" is not met for the merge itself: the stretch that the merge joins is blank in the viewer. Any phone video whose track relocalises into an older map is drawn with one map's poses in another map's world.
  - The status page's segments overlap.
  - Bundle 5 is new in this PR, so this is the cheap moment to fix its contract.
- **Evidence:**
  - Code: `mapRows(poseMaps, map)` is `[lowerBound(poseMaps, 1, map), lowerBound(poseMaps, 1, map + 1)]`. The test asserts `sorted(set(run.tracked_maps.tolist())) == [0, 1]` and `run.maps().tolist() == [0]`, so map 1's frames are named in the records but map 1 is in no bundle array.
  - A Node probe importing `viewer/src/frames.js` at HEAD:
    ```
    pose_maps [0,0,0,1,1,1,0,0]: mapRows(0) -> [0, 3]   mapRows(1) -> [3, 8]   (map 0's rows 6-7 drawn as map 1's)
    merge: records' map [0,0,0,1,1,1,0,0], final pose_maps all 0: mapsShown -> [0,0,0,1,1,1,0,0], mapRows(final, 1) -> [8, 8] (empty)
    ```
- **Suggested fix:**
  - Choose one meaning for "the map at a frame" in bundle 5: the map the frame ends in.
    - The bake writes the records' `map` column through each merge's destination. Diagnostics already carry `loop.merge.map`; recording the merged-into table in the run explicitly would be cleaner.
    - The bake drops merged-away maps from the header, or lists them with what they joined.
  - Replace `mapRows`/`cycleRows` with a map's runs of rows (`geometry.js` already has `mapRuns`), and make `MapInfo.frames` and `mapDepths` use runs too.
  - Give `eval.runs.tracking` one segment per stretch, and fix the `trajectory.txt` docstring.
  - Bake a merge-room golden bundle and one with a revisited map (see the test-gap finding below), and test the viewer's map selection on both.

##### [must-fix] Rust's loop closing holds the atlas's mutex through every keyframe's search, so threaded tracking waits on loop detection
- **Where:**
  - `rust/slam-it-edge/src/pipeline.rs:2397-2409`, in `search_loops`: `let mut closing = lock(&shared.closing); let atlas = lock(&shared.atlas);` then `search(...)`, which runs `loops::detect` and, when maps are kept, `loops::detect_merge` over every kept map (`:1957-2000`).
  - Tracking takes the same mutex on every frame: `:1705` (`extend`), `:1738` (`run_id`), `:3046`/`:2117` (`record`/`tick`), `:3068-3075` (fates), and `place_pending`/`place_frame` at `:3395-3411`.
  - Tracking's relocalisation also holds it through `relocalize_everywhere` (`:1488`).
- **Issue:** At base, Rust's threaded loop closing never took the atlas lock. B4 takes it before the search and holds it until the search ends, even with no kept maps. Every threaded Rust run therefore has tracking blocked for the rest of any loop-detection search (database scoring, matching by word, Sim(3) RANSAC, GTSAM refinement, and the merge search over every kept map).
  - C++ reads the kept maps through `Atlas::kept()` without the mutex (`cpp/src/edge/pipeline.cpp:186`).
  - Python takes its lock only to list them (`Atlas.indexed()`).
- **Why it matters:**
  - Section 12, item 3's design is that tracking does not wait on the other threads' work. Option B's only intended pause is the merge itself.
  - This wait is not recorded: `wait_seconds` counts the owner's hold and the threads' settling, not the atlas mutex. Section 13 and 15's latency bar would see late Rust frames with no stated cause.
  - It also makes the three languages differ in how the threads share state, which the "our design against ORB-SLAM's scheme" comparison assumes is equal.
- **Evidence:** at base `be801e8a`, `rust/slam-it-edge/src/pipeline.rs` locks `shared.atlas` only in mapping and the global BA (lines 1792, 1852, 1996, 2030), not in loop closing. At HEAD, `search_loops` takes it around the whole search.
- **Suggested fix:**
  - Do not hold the atlas for detection. Run `loops::detect` without it.
  - For `detect_merge`, take a snapshot of the kept maps under the lock and release it at once. For example, keep them as `Arc<Indexed>` (or an `Arc<[Indexed]>` swapped by `close`/`resume`/`merge`) and clone the `Arc`s.
  - Do the same in `relocalize_everywhere`.

##### [should-fix] No pipeline test takes up a kept map, in any language or scheme
- **Where:**
  - `src/slam_it/edge/pipeline.py` (`_switch`, `MapOwner.resume`, `TrackingState.change_map`);
  - `cpp/src/edge/pipeline.cpp` (`Pipeline::switch_map`, which lets the baseline's lock go while the threads settle);
  - `rust/slam-it-edge/src/pipeline.rs:3303` (`resume`).
  - The tests found are stage parity only: `tests/edge/test_relocalization.py:326-370`.
- **Issue:** Relocalisation in every map (item 2) is tested as a stage. The step that consumes it is never run, in any language, inline or threaded, and C++'s baseline releases and retakes its lock there without ThreadSanitizer coverage. That step: the frame found in a kept map, the map left kept or reset, the kept map taken up with its database, and the `left` record.
- **Why it matters:**
  - The stop rule asks that the consumer works.
  - This is also the path that produces the revisited maps in the first must-fix, so there is no record or bundle to hold that contract either.
- **Suggested fix:** add a variant of the merge room with merging kept from firing (for example, `loops.enabled` off). Make the camera fail again over the first map's region, so that relocalisation takes map 0 up. Hold all three languages to a bar on it (`left` recorded, `tracked_maps` going 0, 1, 0). Run it threaded and under `tsan` in both C++ schemes, and bake it as a golden bundle.

##### [should-fix] (follow-up: merge design) A merge's id and scale remap is re-implemented by each consumer
- **Where:**
  - Python:
    - `src/slam_it/edge/pipeline.py`: `_carried`, `TrackingState.merged`, the `_offset` calls in `Pipeline._track`, and its own `_scaled` and `_offset`;
    - `src/slam_it/edge/loops.py:2010-2045`: a different `_scaled` (it scales all three rows, where the pipeline's scales the translation) and the same `_offset`;
    - `src/slam_it/edge/atlas.py:421-446`.
  - C++:
    - `cpp/src/edge/pipeline.cpp:78-109` (`carry_through`), `:1343`, `:1551`, `:2009-2011`;
    - `cpp/src/edge/loops.cpp:1641`.
  - Rust:
    - `rust/slam-it-edge/src/pipeline.rs:1822` and `:1905-1911`;
    - `rust/slam-it-edge/src/loops.rs:3169`, `:3221`.
- **Issue:** `loops.merge` returns two offsets and `G`. Then each consumer applies "id + offset, -1 kept" and `G`'s scale itself: the loop being confirmed, the last frame, the anchor, the native owners' anchor points, the frame's matched, tracked and made ids, and the atlas's records. That is about seven call sites per language. The helpers share names across modules but differ in meaning.
- **Why it matters:** option C's core is to "remap tracking's state once". A missed consumer silently names the wrong points, and nothing in the type system catches it.
- **Suggested fix:** give the merge result one remap API in each language (`Merged.carry_points(ids)`, `carry_keyframes(ids)`, `carry_pending(pending)`, `carry_motion(T)`). Consumers call it rather than re-deriving the rule. Do this when option C is designed, in the module the next finding proposes.

##### [should-fix] The two splits meant to come before B4 (A15 `loops`, A9 the map's owner) are worse than their records say
- **Where:**
  - The ladder: `docs/design/2026-10-02-slam-it-ladder.md:1189-1197` (A15), `:1163-1166` (A9), and `:1118-1121` (B4's "Left open", which does not list A9).
  - The module sizes:
    - `loops`: Python 2,413 → 3,494 lines, C++ 1,736 → 2,664, Rust 2,381 → 3,821;
    - `pipeline`: Python 1,755 → 2,264, C++ 2,032 (from 1,543), Rust 2,811 → 3,511.
  - Reaching into `loops` for Sim(3) operations: `src/slam_it/edge/atlas.py:56,427-436` (`loops.scale_of`, `loops.rigid_of`); `cpp/src/edge/atlas.cpp:213,224`; `rust/slam-it-edge/src/atlas.rs:19`.
- **Issue:**
  - A15 asked that, when B4 lands, common-region detection be split out "rather than growing `loops` or having merging reach into it". B4 put merge detection, the weld and the merge into `loops` (+1,081, +928 and +1,440 lines over base). The atlas and pipeline now reach into `loops` for Sim(3) operations.
  - A9 asked for an owner module "before B4's merges enter through it". They enter through `MapOwner.apply_merge`/`take_merged` inside `pipeline`.
  - A15's record still quotes the sizes at `eedb319` (2,208 / 2,284 / 1,685). A9 is not listed as open after B4. I am not disputing the owner's deferral of A15; the record understates it.
- **Suggested fix:**
  - Update A15 with the current sizes and what B4 added, and list A9 as still open after B4.
  - Make both splits the first step of the option-C follow-up, since option C rewrites exactly this code:
    - a `merging` module per language (`detect_merge`, `merge_first`, `merge_transform`, `merge`, the weld);
    - an `owner` module (`MapOwner`, `View`).
  - These two modules need new layers in the edge contract.

##### [should-fix] The ladder still describes the edge rung as relocalising in the active map only, with no map reuse
- **Where:** `docs/design/2026-10-02-slam-it-ladder.md`:
  - `:546` ("no map reuse");
  - `:551-555` ("relocalised against the whole active map ... joining maps is 2B's B4");
  - `:613-614` ("relocalisation against the active map before a new one is started");
  - `:412` (eval's dataset imports leave out `sevenscenes`);
  - `:1244-1253` (A2's "An atlas, with milestone 2B" item is still open, though B4 keeps maps whole with their descriptors and observations).
- **Issue:** The design doc is the authority, and B4 changed what these lines describe: relocalisation now searches every map (L7), kept maps are taken up and merged, and 7-Scenes is imported.
- **Suggested fix:** change the doc, not the code. Update those lines, add `sevenscenes` to the eval row, and close A2's atlas item, or reduce it to what remains.

##### [should-fix] Python's `detect_merge` and `merge` leave GTSAM's failure out of their error contract
- **Where:** `src/slam_it/edge/loops.py:3289-3338` and `:3411-3463`. Compare `cpp/include/slam_it/edge/loops.hpp:424-431` and `:518-527` ("GTSAM's failure").
- **Issue:** Both run GTSAM solves (`refine`, and the weld through `graphs.bundle_adjust`), so they can raise `NotConvergedError`. In the same module, `detect` and `correct` document it (`:3208`, `:3264`); these two list only `InvalidInputError`. Rust says "a stage's".
- **Suggested fix:** list `NotConvergedError` in both Python docstrings, and name it in Rust's `# Errors`, as C++ does.

##### [note] (follow-up: merge design) The kept maps are shared three different ways
- **Where:**
  - `src/slam_it/edge/atlas.py:246-269` (locked, returns a list);
  - `cpp/include/slam_it/edge/atlas.hpp:93-98` (an unlocked span, safe by protocol: only tracking changes it, with the threads settled or drained). The class doc at `:84-86` says only tracking reads it; the method doc says loop closing does too.
  - `rust/slam-it-edge/src/atlas.rs:268` (no lock of its own; `Mutex<Atlas>`).
- **Note:** Under option B each scheme is safe; TSan covers C++. Option C has loop closing solve on the kept maps while tracking runs, so these protocols stop holding. The follow-up should choose one sharing design for all three, for example an immutable snapshot swapped whole like the views. Fix the C++ doc contradiction meanwhile.

##### [note] `LostSummary` now names any closed map
- **Where:** `src/slam_it/edge/pipeline.py:250-265`, with its C++ and Rust mirrors.
- **Note:** It now records a lost map, a map left for a kept one (`left`) and a session's closed map. A name like `ClosedSummary` would say what it is; a clean break costs nothing now.

##### [note] The measure's config keeps `folder` beside `folders`
- **Where:** `scripts/measure_testset.py` (`FOLDER_KEYS`).
- **Note:** Two spellings for one sequence keep the old config loading. The no-shims policy suggests `folders` alone, a list of one for a single sequence.

#### Checked and fine
- **import-linter:** `sevenscenes` is placed in eval's layers. The edge order holds: `pipeline` > `atlas` > `loops | relocalization` > `database`. C++ includes and Rust `crate::` paths follow it. No new contract is needed beyond the module splits proposed above.
- **Runtime dependencies:** none added. GTSAM stays confined to `graphs`. The Huber loss crosses the existing bridge (0 means no loss), and each core validates it.
- **Parity of capability:** these are in all three cores and both bindings, with `Implementation` Protocols updated and checked by `tests/test_backends.py`:
  - `merge_candidates`, `relocalize_across`, `detect_merge`, `merge_first`, `merge_transform`, `merge`;
  - the new settings (`weld_*`, `session_keyframes`);
  - `Indexed`/`MapRef`, mirrored in C++ and Rust.
- **Bindings:** they only convert types (`edge_crossing::indexed_in`, the merge dicts). Cores return `std::expected`/`Result` with the shared kinds.
- **Run format 9:** `keyframe_merges.npy` and `tracked_maps.npy` are written and checked the same way by the Python and C++ writers (`_check_edges`/`check_edges`), and the Rust writer also writes format 9.
- **Bundle 5:** it carries the merges, loaded with id checks and drawn in their own colour. Only the map-order assumptions above are wrong.
- **Sessions:** layered correctly. The edge rung exposes `start_session`; the sequence list, the clock offsets and the shared-camera check live in orchestration, the same in all three programs.
- **Threaded merge (option B):** it reuses B3b's single stop/release handshake (`Writer.stopped`/`owner.stopped`) in all three languages and in both C++ schemes, and the merge room under ThreadSanitizer is in layer 1.
- **One bar for all languages:** the merge room holds each language to it, with a golden record per language and no identical-decision test.
- **Placement:** Rust stays within the edge rung. The 7-Scenes importer is a Python-only eval import, like the test set's, and its licence is recorded.
- **Ladder records:** the deferrals and option C are recorded in the ladder's open items, with option B's tests as its required behaviour.

### Algorithm review

**Scope:** `be801e8a...7dfd04f2` (B4, milestone 2B: merging maps). I read the merge path in full in Python: `src/slam_it/edge/loops.py` (merge detection, `merge`, the weld, the essential graph), `atlas.py`, `database.py` (merge candidates), `relocalization.py` (`relocalize_across`), the merge, relocalisation and session parts of `pipeline.py`, the Huber change in `src/slam_it/foundations/graphs.py`, and `src/slam_it/eval/sevenscenes.py`. I compared the C++ and Rust twins of `merge`, `moved_into`, `absorb`, `rehang`, the weld, `Atlas::merge` and the loop carry, and I read the tests (`tests/edge/test_merges.py`, `tests/orchestration/test_merge_room*.py`, `tests/foundations/test_graphs.py`). References: ORB-SLAM3 (Campos et al., T-RO 2021, section VI) and its source (`LoopClosing.cc` `MergeLocal`/`Run`, `KeyFrameDatabase.cc` `DetectNBestCandidates`); ORBSLAM-Atlas (Elvira, Tardós & Montiel, IROS 2019); Strasdat et al. (RSS 2010) for Sim(3). I ran the merge room in Python and C++, and small read-only probes on the `TwoMaps` fixtures.

**Summary:** The merge itself is sound. The Sim(3) `G = S⁻¹T` matches ORB-SLAM3's `mSold_new`. Carrying the active map into the matched map's units is consistent: each keyframe becomes `rigid_of(T G⁻¹)`, points move by `G`, and culled-keyframe offsets are divided by `s`. The weld follows the paper's choice of which keyframes are fixed. The essential graph runs in the active map's units, so its edges hold, and the Huber loss matches g2o's in all three languages. Two monocular scale bugs remain, in all three languages, outside `loops.merge`:
- **Final trajectory:** frames tracked in the map that was merged away keep their offsets from their reference keyframes in the old map's units.
- **Loop being confirmed:** a loop carried across a merge keeps the old map's units.

Neither is in the recorded known gaps. Both are one-line fixes, and neither has a test.

#### Findings

##### [must-fix] Monocular merge: frames from the merged-away map are placed using offsets in its old units
- **Where:** `src/slam_it/edge/atlas.py:421-446` (`Atlas._merge`); `cpp/src/edge/atlas.cpp:204-236` (`Atlas::merge`); `rust/slam-it-edge/src/atlas.rs:436-469` (`Atlas::merge`).
- **Issue:** Each tracked frame's placement stores `relative = T_frame T_ref⁻¹` (`atlas.py:463`), in the active map's units. The final trajectory places the frame at `relative @ anchor` (`atlas.py:649`), where `anchor` is the reference keyframe's pose in the merged map, which is in the matched map's units.
  - `_merge` divides each culled keyframe's relative translation by `s = scale_of(G⁻¹)` (`atlas.py:438-441`). It does nothing to `_placements[*].relative` for frames tracked in the current map.
  - So every non-keyframe frame of the absorbed map is offset from its reference keyframe by `t_rel` where `t_rel / s` is right.
  - Where a frame's reference was later culled, the chain mixes units: the frame's step is unscaled and the culled steps are scaled.
  - Stereo and RGB-D are unaffected (`s = 1`).
- **Why it matters:** This is the final trajectory that runs are scored on (section 10, item 8). Every monocular merge with `s ≠ 1` misplaces the absorbed map's frames by `|1 − 1/s|` times their offset from their reference keyframe. Section 15's multi-session EuRoC monocular bar will be measured on this output.
- **Evidence:**
  - **Known-answer probe.** I ran a pytest probe on `tests/edge/conftest.py`'s `two_rigid` and `two_scaled` fixtures, loaded with `-p tests.edge.conftest`. It builds an `Atlas` and keeps the matched map. In the active map it places a frame exactly where active keyframe 4 is, tracked against keyframe 2 (0.31 m apart in truth). It then runs `loops.merge` (the merge found on `FOUND_AT`, global BA off), then `Atlas.merge`, and reads `Atlas.trajectory`:
    ```
    rigid:  merge scale 1.0000; … the frame placed 0.0147 m from its true place, keyframe 4 itself 0.0062 m
    scaled: merge scale 0.7043; … the frame placed 0.0795 m from its true place, keyframe 4 itself 0.0072 m
    scaled, placements' relative translations divided by scale_of(G⁻¹) in _merge: the frame placed 0.0126 m
    ```
  - **The merge room cannot see it.** The monocular merge's `s` there is 0.5457 (Python and C++ agree; final ATE 1.847 cm in both). With the fix patched into Python, the room's final ATE went from 1.85 to 1.99 cm, under the 2.8 cm bound either way. The room's second map changes its own scale by about 1.4× over its 74 frames: aligned to the truth as tracked, its first 25 frames come out at 15.8 m per map unit and its last 25 at 11.2. That drift dominates the frames' offsets, so the room is no test of this.
- **Suggested fix:** In all three `Atlas::merge`, divide the translation of `relative` by `scale_of(G⁻¹)` for each placement whose `map` is the current map, exactly as `_culled` is handled. Add a known-answer test like the probe above (scaled two-map fixture, a frame placed by its reference keyframe through the merge, its final pose within ~1.5 cm). Each native atlas needs it too, or a Python-visible check through the native run.

##### [must-fix] A loop being confirmed is carried across a monocular merge in the old map's units
- **Where:** `src/slam_it/edge/pipeline.py:1374` (`_carried`: `similarity=pending.similarity @ sim3.inverse(to_world)`); `cpp/src/edge/pipeline.cpp:107`; `rust/slam-it-edge/src/pipeline.rs:1908`.
- **Issue:** A pending loop's `S` maps the world into its last keyframe's camera in the active map's units, with scale σ, the in-map drift. After the merge, the active keyframes are rigid poses in the matched map's units. `S G⁻¹` has scale σ·s, not σ.
  - `loops._carry` composes it with a rigid relative pose in matched units (`src/slam_it/edge/loops.py:723`: `_relative(current, last) @ pending.similarity`), so the step between the two keyframes is scaled by `1/s`.
  - Were the loop to close, `_correct` composes the same way (`loops.py:1628`).
- **Why it matters:** For any `s` far from 1, the carried loop's projection search (`_confirm_by_projection`) starts from the wrong pose, so it likely misses twice and the loop is dropped. Refinement re-estimates the scale from matches, so I do not see it corrupting the map; the cost is a lost loop. Stereo and RGB-D are unaffected. No test carries a pending loop through a merge. ORB-SLAM3 does not carry it at all (`mg2oLoopSlw` is untouched by `MergeLocal`/`Run`), so carrying it is an improvement, but it has to be in the right units.
- **Evidence:** A numeric check composes as `_carried` then `_carry` do (repo's `loops._relative`, `loops.rigid_of`, `sim3.inverse`), with `s` = 0.5457 (the merge room's), a loop that places the last keyframe exactly, and the next keyframe 12.5 cm on:
  ```
  S G^-1 (as built):                      carried scale 0.5457; keyframe centre off by 0.1042 (step 0.1251, matched units)
  S G^-1 / s (units of the merged map):  carried scale 1.0000; keyframe centre off by 0.0000
  ```
- **Suggested fix:** `S' = diag(1/s, 1/s, 1/s, 1) · S · G⁻¹` with `s = scale_of(G⁻¹)`: scale the output rows, as `TrackingState.merged` scales the translations of tracking's motions. Apply it in all three languages, and add a stage test that carries a pending loop through `two_scaled`'s merge and confirms it on the next keyframe.

##### [note] (follow-up: merge design) The essential graph moves points the welding BA just solved
- **Where:** `src/slam_it/edge/loops.py:2326` (`_move_by_graph(m, start, optimised, {})`), with twins in C++ and Rust (`finish_correction(..., {}, ...)`).
- **Issue:** Both windows are held in the graph, but a point is moved by its reference keyframe's correction. So a weld-solved point whose reference keyframe lies outside the windows is moved a second time. ORB-SLAM3's source avoids this: its graph corrects only `vpCurrentMapMPs`, which excludes the window's points (`MergeLocal`, `OptimizeEssentialGraph(..., vpCurrentMapKFs, vpCurrentMapMPs)`). A related detail: cross-map links that end at a free keyframe are measured between the pre-weld poses that `G` placed, not the welded ones.
- **Evidence:**
  - **Narrow windows.** A probe on the two-map fixtures at `test_merges.NARROW` (global BA off) found 158 of 1,882 weld points with a reference outside the windows (157 of 1,881 scaled). The graph moved them by a median 3.1 mm, at most 2.4 cm (5.2 mm and 2.3 cm scaled). Their chi² in the held welded keyframes rose: p90 from 0.21 to 0.46 (rigid), 0.14 to 0.22 (scaled). Small here.
  - **The merge room.** None of the weld points had a reference outside the windows, because the windows cover both maps.
- **Suggested fix:** Pass `moved_by` mapping each weld point to a welded keyframe, as the loop passes `mnCorrectedReference`, so the graph leaves it where the weld put it. Weigh this in the follow-up's review of the merge design.

##### [note] (follow-up: merge design) Threaded merges skip global BA unless one was superseded; the pause includes the whole-map essential graph
- **Where:** `src/slam_it/edge/pipeline.py:1742-1752` (`merge_waiting`, `options=self._inline_ba_off`); the design's section 14, item 4.
- **Issue:**
  - **Global BA.** Inline runs global BA after every merge. Threaded runs one only where it superseded one (ORB-SLAM3's `bRelaunchBA`). The design's item 4 says "global BA after". So inline and threaded merges end differently, and threaded keeps whatever the weld and graph leave, including the point issue above.
  - **The pause.** Under option B it covers the weld, the essential graph over the whole merged map (wider than ORB-SLAM3's source, which spans only the rest of the active map) and the database resync. The pause therefore grows with both maps' size, and is unmeasured (section 15 is deferred).
- **Suggested fix:** Route both points to the merge-design follow-up: whether threaded merges should always start a global BA, and the pause's size on real maps.

##### [note] Relocalisation in every map: candidates grow with the kept maps, and a healthy active map can be left
- **Where:** `src/slam_it/edge/relocalization.py:703` (`_relocalize_across`); `src/slam_it/edge/database.py:319-332` (the per-map 75% cut).
- **Issue:** Each map's 75% group cut is relative to that map's own best, so every kept map adds candidates whatever its absolute score. Relocalisation then tries all of them on each failed frame during the 3 s window. A kept map's candidate that outscores the active map's takes the track, even on a momentary failure. The active map is then kept, or reset if it has 10 keyframes or fewer. This matches the paper's description and was decided for B4 (item 2), so it is not a defect.
- **Suggested fix:** When section 15 runs (the kidnap, 7-Scenes), measure the relocalisation candidates per failed frame and how often the track leaves a healthy map. If either is high, consider an atlas-wide floor (75% of the best accumulated score over all maps, after per-map word counting), or trying the active map's candidates first.

##### [note] A monocular merge's scale is unbounded, and a merge cannot be undone
- **Where:** `src/slam_it/edge/loops.py` (`detect_merge`, documented); design section 14, item 3.
- **Issue:** This is visual-only ORB-SLAM3's behaviour too. A merge at a wrong place with an extreme `s` would rescale a whole map irreversibly; the guard is three confirmations, as for a loop. Multi-session and multi-robot systems usually add a consistency check on the links between maps: pairwise-consistency maximum clique (Mangelson et al., "Pairwise Consistent Measurement Set Maximization for Robust Multi-robot Map Merging", ICRA 2018, https://robots.engin.umich.edu/publications/jmangelson-2018a.pdf) or graduated non-convexity in the back end (Tian et al., "Kimera-Multi", T-RO 2022, https://arxiv.org/abs/2106.14386). This is context for the follow-up, not a B4 defect.
- **Suggested fix:** Optionally, a wide sanity bound on `s` (for example 0.05 to 20), recorded when it refuses. The section 15 measurement should judge wrong-place merges, as the measure does.

##### [note] The welding BA inherits B3's deferred damping issue (L2)
- **Where:** `src/slam_it/edge/loops.py:2178-2266` (`_weld_solve`, `_weld`).
- **Issue:** The weld is the merge's main correction and goes through `graphs.bundle_adjust` with GTSAM's absolute λ (1e-5). The damping PR from B3 is not yet landed. `_weld` applies its result whatever the termination, which is recorded in `WeldSummary`, as ORB-SLAM's local BA would apply it. I am not re-raising L2.
- **Suggested fix:** When the damping PR runs, have it measure the welds as well, not just global BA.

##### [note] The Huber loss test is behavioural, not a known answer
- **Where:** `tests/foundations/test_graphs.py` (`test_a_huber_loss_holds_bundle_adjustment_against_an_outlier`).
- **Issue:** The test checks that cameras move less and that the backends agree. It would pass for a per-component Huber as well. For the 3-row stereo factor, per-component and whole-vector Huber differ.
- **Evidence:** I checked the semantics directly. One stereo observation, whitened error (3, 4, 0), k = √7.815. The expected initial error is 10.0702 if the loss acts on the norm (g2o's chi² kernel), 11.7537 if per component, and 12.5 with no loss. Python, C++ and Rust all gave `initial_error` 10.0702.
- **Suggested fix:** Optionally, add this as a known-answer test, which pins GTSAM's `Block` reweighting.

#### Checked and fine
- **Merge transform.** `merge_transform` `G = S⁻¹T` matches ORB-SLAM3's `mSold_new = gSw2c · gScw1`. It is rigid in stereo and RGB-D, where the scale is fixed throughout.
- **Carrying the map.** `_moved_into` gives keyframes `rigid_of(T G⁻¹)` and moves points and archived points by `G`. Culled offsets are divided by `s`, and points are refreshed, so their distance ranges are rescaled. C++ and Rust match it step for step.
- **Essential graph units.** The absorbed nodes start in active units (`_scaled(p, s)`), and the edges are measured between priors scaled alike, so the graph is consistent. `_move_by_graph` returns rigid poses. The graph covers the whole merged map with both windows held, as in the paper.
- **Welding window.** ORB-SLAM3's: 25 best links, widened by 12 per member, up to 5 rounds.
- **Welding BA.** The paper's gauge: the matched map's observers outside the windows held, or the matched window when there are none. Huber first, outliers left out, 10 more iterations, failing observations forgotten: the same structure as `LocalBundleAdjustment`.
- **Fusion and tree.**
  - Fusion keeps the matched point and drops the active one (`SearchAndFuse` → `Replace`).
  - `_rehang` follows ORB-SLAM3's `ChangeParent` chain, leaving a single root (id 0), so `covisibility.cull`'s rule stands.
  - Merge edges are spared by culling.
- **Merge detection.** It runs loop detection's stages against another map. Merge priority over a loop matches ORB-SLAM3's `Run`. Resetting misses on a confirmation fixes ORB-SLAM3's slip. Per-map candidates, then the best 3 overall, is a documented and reasonable departure.
- **Merge known-answer tests.** `tests/edge/test_merges.py` puts every keyframe within 3 cm of the truth for rigid and for 0.7-scaled monocular maps, with wide and narrow windows, and holds the compiled backends to Python.
- **Threaded tracking.** `TrackingState.merged` scales the velocity and the offset from the anchor by `scale_of(G)`, which is correct (the inline velocity gap is recorded and deferred).
- **Huber.** Thresholds `√χ²(2 dof)` and `√χ²(3 dof)` on the whitened norm, verified in all three languages above. Non-positive thresholds are refused.
- **Relocalisation across maps.** One vocabulary is checked. Ties go to the lower map number, then the lower id. Another map's rejected matches are dropped from the fates. Leaving a map keeps or resets it by the loss rule.
- **Sessions.** They follow ORB-SLAM3's `ChangeDataset` (a map under 12 keyframes is reset, otherwise kept).
- **7-Scenes import.**
  - Monocular only, since depth and colour are unregistered.
  - Poses read as `T_world_cam`, rotations projected onto SO(3).
  - 525 px focal as DSAC uses, documented.
- **Three languages.** The merge is mirrored step for step. The merge room's monocular run gives the same merge scale and the same final ATE in Python and C++. Both bugs above are shared, not divergent.
