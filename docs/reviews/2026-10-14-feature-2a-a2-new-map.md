# Review panel: `feature/2a-a2-new-map`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-14

| | |
|---|---|
| **Branch** | `feature/2a-a2-new-map` |
| **Range** | `0c258a4...96764f9` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed (`scripts/check.sh` in the `slam-it` env, exit 0: 25 checks; pytest 3740 passed) |

### Findings and resolutions

Three reviewers found the same must-fix (H1, A1, L1), two the same `trajectory.txt` finding (A6,
L2) and two the same summary-size test (H5, A7); each is decided once, under its first ID.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `src/slam_it/eval/runs.py` `_scored` | `evaluate` turns an invalid `rpe_delta` into "no RPE": the per-map `except InvalidInputError` catches bad arguments too | fix (`rpe_delta` checked in `evaluate`; "no pair" an explicit test through a pairing helper in `metrics`, no `except`; tests for -1, 0, NaN, inf) | fixed (b682f3c: `rpe_delta` checked in `evaluate`, `metrics.interval_pairs`, nothing caught) -- decided by evan-wes, 2026-10-14 |
| H2 | should-fix | hygiene | `runs.py` `_scored` | Too long; mixes association, alignment, RPE and the nothing-scored error | fix (a `_score_map` helper, with H1) | fixed (b682f3c: `_score_map`) -- decided by evan-wes, 2026-10-14 |
| H3 | should-fix | hygiene | `runs.py` `_check_maps`; `rust/slam-it-eval/src/runs.rs` `check_maps` | The map checks' errors name neither the bad row and value nor the lengths | fix (the first bad row and its value, the lengths; C++ too) | fixed (367daa6: the first bad row and its value, and the lengths, in all three languages) -- decided by evan-wes, 2026-10-14 |
| H4 | should-fix | hygiene | `edge/pipeline.py` `_close`; `runs.py` `Evaluation.ate_rmse` | `assert`s for invariants in library code | fix (`_close` takes the map, as C++ and Rust do; `ate_rmse` without the assert) | fixed (b682f3c: `ate_rmse` without the assert; 75ac019: `_close` takes the map) -- decided by evan-wes, 2026-10-14 |
| H5 | note | hygiene | `worker.py` `SEGMENTS`; server `SUMMARY_BYTES` | 20 segments fitting 4,096 bytes is a comment, not a test | as A7 | fixed (as A7, 85fa052) -- decided by evan-wes, 2026-10-14 |
| H6 | note | hygiene | `rust/slam-it-edge/src/pipeline.rs` `to_id` | `expect` in library code, unreachable for a `usize` count | waive (unreachable; the message says why) | waived: unreachable for a `usize` count, and its message says why -- decided by evan-wes, 2026-10-14 |
| H7 | note | hygiene | `edge/pipeline.py` `_date`, `_close` | The two walk the map's arrays with the same `base + id` arithmetic | defer (2B's atlas, with A9) | deferred (the ladder's open items, A2's panel: an atlas, with 2B) -- decided by evan-wes, 2026-10-14 |
| A1 | must-fix | architecture | `runs.py` `_scored`, `evaluate` | As H1 | as H1 | fixed (as H1, b682f3c) -- decided by evan-wes, 2026-10-14 |
| A2 | should-fix | architecture | `runs.py` `_scored`; `bundles._score`; `run_vo.score`; `measure_testset` | "No map can be aligned" is `InvalidInput`; a collinear run was `Degenerate`, and the message names the wrong cause | fix (`DegenerateError`, each map's cause named; callers catch it) | fixed (b682f3c: `DegenerateError`, each map's cause named; the bake and `run_vo` catch it) -- decided by evan-wes, 2026-10-14 |
| A3 | should-fix | architecture | `runs.py` format text; `bundles.py` `records.json`; `viewer/src/frames.js` | The diagnostics' `map` key decides what the bake and viewer show, but the format neither names nor checks it | fix (the minimum: `map`, `state` and `lost` named as v4's per-frame keys, `map` checked in `write_run`/`read_run`) | fixed (367daa6: the minimum, `map`, `state` and `lost` in the format's text, `map` checked by `write_run` and `read_run`) -- decided by evan-wes, 2026-10-14 |
| A4 | should-fix | architecture | the three run programs' pose pop; the pipeline's `close` | A reset map's poses are dropped by each run program, its keyframes and points by the pipeline | defer (2B's atlas: the pipeline owns the trajectory record) | deferred (the ladder's open items, A2's panel: an atlas, with 2B, the pipeline owning the trajectory record) -- decided by evan-wes, 2026-10-14 |
| A5 | should-fix | architecture | `runs.py` `final_points`, `final_keyframes`, format text; `tracking`; `bundles.py`; `run_vo.py` | No map-aware queries on `Run`; `final_*` say "the final map" and points "the world frame" | fix (`Run.maps()`, `Run.reset_maps()` used by all three; the docs say each map's own frame) | fixed (367daa6: `Run.maps()`, `Run.reset_maps()`, used by `tracking`, the bake and `run_vo`; the docs say each map's own frame) -- decided by evan-wes, 2026-10-14 |
| A6 | should-fix | architecture | `runs.py` format text; ladder's Metrics; the three writers | `trajectory.txt` holds several maps' frames with nothing marking them, yet is offered to the TUM tools | fix (the docs say so and to split by `pose_maps.npy`; a `# map <n>` line before each map's poses in all three writers) | fixed (367daa6: a `# map <n>` line before each map's poses in all three writers; the format's text and the ladder) -- decided by evan-wes, 2026-10-14 |
| A7 | should-fix | architecture | `worker.py`; `tests/server/test_shared_names.py` | `SEGMENTS` fitting `SUMMARY_BYTES` is not held by a test | fix (a worst-case summary held under `SUMMARY_BYTES`) | fixed (85fa052: the longest summary, 586 bytes) -- decided by evan-wes, 2026-10-14 |
| A8 | note | architecture | the three pipelines' `State` | `State.LOST` is both a frame's outcome and a passing pipeline state; lost-with-no-map handled three ways; an unreachable Rust arm | fix in part (H4 makes the three alike; the unreachable arm goes); the rest defer (2B's atlas) | fixed in part (75ac019: `_close` alike in three languages, the branch for LOST between frames gone); the rest deferred (the ladder's open items, A2's panel: an atlas) -- decided by evan-wes, 2026-10-14 |
| A9 | note | architecture | the three pipelines | Map numbering, id bases and the ledger are the seed of an atlas | defer (2B's design: an `edge.atlas` module) | deferred (the ladder's open items, A2's panel: an atlas, with 2B) -- decided by evan-wes, 2026-10-14 |
| A10 | note | architecture | ladder's `SlamResult` sketch | The reifyscene boundary assumes one world frame | defer (the ladder's open items, before milestone 3) | deferred (the ladder's open items, A2's panel: before milestone 3) -- decided by evan-wes, 2026-10-14 |
| A11 | note | architecture | `eval/runs.py` | The format's I/O and its scoring share one 1,156-line module | defer (the ladder's open items: `eval.scoring`) | deferred (the ladder's open items, A2's panel: `eval.scoring`) -- decided by evan-wes, 2026-10-14 |
| A12 | note | architecture | `viewer/src/scene_view.js` `placed`, `showOthers` | Which maps share a frame is decided in an untested view module | fix (to `frames.js`, tested) | fixed (85fa052: `placed` and `sharesFrame` in `frames.js`, tested) -- decided by evan-wes, 2026-10-14 |
| A13 | note | architecture | `docs/notes/test-set.md`; design §2 | The seeds and the loss table come from scripts outside the repo; design §2 says "fewer than 10" | defer (`--seed` and the loss classification in the measure, the ladder's open items); §2 as L5 | deferred (the ladder's open items, A2's panel: `--seed` and the loss table in the measure); design §2 fixed as L5 (cb377fd) -- decided by evan-wes, 2026-10-14 |
| L1 | must-fix | algorithm | `runs.py:1071` | As H1 | as H1 | fixed (as H1, b682f3c) -- decided by evan-wes, 2026-10-14 |
| L2 | should-fix | algorithm | `runs.py:14-26`, `210-221`; the writers | As A6, and `final_*`'s "the final map" | as A6 and A5 | fixed (as A6 and A5, 367daa6) -- decided by evan-wes, 2026-10-14 |
| L3 | should-fix | algorithm | the three pipelines' unit tests | No test sits on the reset boundary, so `<` for `<=` would pass | fix (`reset_keyframes` 1 and 0 on the 1-keyframe map, in all three) | fixed (75ac019: `reset_keyframes` 1 and 0 on the one-keyframe map, in all three languages) -- decided by evan-wes, 2026-10-14 |
| L4 | should-fix | algorithm | `tests/edge/test_pipeline.py`; the golden record | A map with no points, and a third map's run ids, are not tested end to end | fix (a Python test from a frame without depth through two losses; a third map in the golden record's lost room) | fixed (75ac019: a Python test from a frame without depth through two losses; the lost room blanked twice, a third map in stereo and RGB-D) -- decided by evan-wes, 2026-10-14 |
| L5 | note | algorithm | design §2; `options.py` | ORB-SLAM3's deciding check is `<=10`; it gives a small map no grace before LOST | fix (design §2 corrected; the lite difference stated beside the decision) | fixed (cb377fd: design §2 says 10 or fewer; §5 states the grace frames' difference) -- decided by evan-wes, 2026-10-14 |
| L6 | note | algorithm | `_scored`; the measure's per-map ATE | A short map's own Sim(3) fit makes its ATE small | fix (a sentence in the test-set note) | fixed (cb377fd: the test-set note's caveat) -- decided by evan-wes, 2026-10-14 |
| L7 | note | algorithm | `_check_maps` and twins | The map arrays are checked alone, not against one another | defer (the ladder's open items) | deferred (the ladder's open items, A2's panel) -- decided by evan-wes, 2026-10-14 |
| L8 | note | algorithm | -- | Context for 2B (Atlas, multi-session SLAM); the note's two-view comparison confirmed | waive (context, nothing to change) | waived: context for 2B, nothing to change -- decided by evan-wes, 2026-10-14 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

Decided by evan-wes on 2026-10-14, by selectable prompts: H1 (with A1 and L1) fixed fully; A6
(with L2) by the docs and a `# map <n>` line before each map's poses; the other should-fixes as
recommended (A4 deferred); the notes as recommended, with a new open item for the two-view
sample limit behind A2's late frames.

### Hygiene review

## Hygiene review

**Scope:** Diff 0c258a4...96764f9 in the repository root. I read the Python in full: `edge/{pipeline,options,map,initialization,tracking}.py`, `eval/runs.py`, `orchestration/{bundles,vo,worker}.py`, `scripts/run_vo.py`. I read the Rust `pipeline.rs` and `runs.rs` diffs and grepped the C++, Rust and viewer diffs for debug output, TODOs and `unwrap`/`expect`. I did not run the tests, and I read the viewer JS and C++ sources only by grep, so those are not covered in depth.

**Summary:** The code is clean and the documentation is unusually good. Frame conventions and id offsets are stated, and the three languages mirror each other closely. One real production-quality defect: the new per-map scoring in `runs.evaluate` swallows an invalid `rpe_delta` (details below). Everything else is small: a long function, asserts in library code, and error messages that don't name the bad value.

### Findings

#### [must-fix] `evaluate` silently drops the RPE for every map when `rpe_delta` is invalid
- **Where:** `src/slam_it/eval/runs.py`, in `_scored`, the `try: ... metrics.relative_error(...) except InvalidInputError: drift = None` block.
- **Issue:** The comment says "no matched frame of the map has a pair", but `metrics.relative_error` raises `InvalidInputError` for several other reasons. One is `delta: must be positive and finite` (`metrics.py:206`). Another is the `tolerance` check (`metrics.py:210`). Both are caught and turned into `None`.
- **Why it matters:** Before this change, `evaluate(run, ..., rpe_delta=-1.0)` or `rpe_delta=nan` raised with a message naming the value. Now it returns an `Evaluation` whose maps all have `relative=None`, and the pooled RPE is `None`. The caller gets a plausible-looking result with the RPE missing and no error. A caller error is being hidden at a public boundary. No test in `tests/eval/test_runs.py` passes `rpe_delta` (grep finds none), so this is also untested.
- **Evidence:** The `except` clause is as quoted above. The `metrics.py` raises are at lines 201 (shape), 206 (delta), 210 (tolerance), 213 (scale) and 220 (no pair). Only line 220 is the case the code intends to tolerate.
- **Suggested fix:** Validate `rpe_delta` (positive and finite when given) in `evaluate` before the loop, so bad input still raises. Then catch only the no-pair case. A dedicated check for "has a pair" or a narrower error type would let the `except` stop catching unrelated `InvalidInputError`s. Add a test for a negative or NaN `rpe_delta`.

#### [should-fix] `_scored` is too long and mixes four jobs
- **Where:** `src/slam_it/eval/runs.py`, `_scored` (about 50 lines, two nested `try`s, a `# noqa: PLR0913`).
- **Issue:** One function does the association, the alignment and its fallback to unscored, the per-map RPE with its fallback, and the error for "nothing scored".
- **Why it matters:** This is the most important new logic in the PR, and it is the function hiding the bug above.
- **Suggested fix:** Split out `_score_map(...) -> MapError | None` (association, alignment, optional RPE). `_scored` then loops over maps, collects `unscored`, and raises when nothing was scored. The `most` bookkeeping for the error message would then come from the helper.

#### [should-fix] `_check_maps` and the Rust `check_maps` errors do not name the bad value
- **Where:** `src/slam_it/eval/runs.py`, in `_check_maps` ("maps must be numbers from 0, never descending"); `rust/slam-it-eval/src/runs.rs`, in `check_maps` ("expected one map per row", same ordering message).
- **Issue:** The Python shape/dtype message is good. The range/order message reports neither the offending value nor its row. The Rust length message does not give the lengths.
- **Why it matters:** A corrupt `pose_maps.npy` from a native backend is exactly what this check exists to catch, and the message will not say where it is bad.
- **Suggested fix:** Include the first offending row and its value, and in Rust the actual and expected lengths.

#### [should-fix] `assert` used for invariants in library code
- **Where:** `src/slam_it/edge/pipeline.py`, `_close` (`assert m is not None`); `src/slam_it/eval/runs.py`, `Evaluation.ate_rmse` (`assert pooled is not None`).
- **Issue:** These asserts disappear under `python -O`.
- **Why it matters:** Minor. Both are genuine invariants, `__post_init__` guarantees the second, and the failure would still surface as an `AttributeError`. They are not input validation.
- **Suggested fix:** For `_close`, take the map as an argument, as the Rust `close(&mut self, m: &Map)` does. That removes the assert and aligns the two languages. For `ate_rmse`, leave it or raise a plain `RuntimeError`.

#### [note] `worker.SEGMENTS = 20` depends on the server's `SUMMARY_BYTES` only by a comment
- **Where:** `src/slam_it/orchestration/worker.py:132`; `server/src/slam_it_server/app.py:89`.
- **Issue:** The 4,096-byte cap lives in another package, and the "20 pairs take about 400 bytes" claim is unchecked.
- **Why it matters:** The test `test_worker.py:246` only monkeypatches `SEGMENTS` to 3. If the summary grows, nothing fails until the server rejects a job at runtime.
- **Suggested fix:** Add a test that builds a worst-case summary (20 segments plus the other fields) and asserts `len(json.dumps(...)) < SUMMARY_BYTES`.

#### [note] Rust `to_id` uses `expect("a run's ids fit in i64")`
- **Where:** `rust/slam-it-edge/src/pipeline.rs`, `to_id`.
- **Issue:** This is an `expect` in library code. It is unreachable in practice for a `usize` count, and the message states the reason, so I am not asking for a change. I am mentioning it only because the brief lists `expect` in library code as a smell.

#### [note] Duplicate bookkeeping between `_date` and `_close`
- **Where:** `src/slam_it/edge/pipeline.py`, `_date` and `_close`.
- **Issue:** Both walk the map's arrays and write `_past_points`, `_past_keyframes` and the lives with the same `base + id` arithmetic.
- **Why it matters:** The logic is consistent today, and the Rust code does the same. If it grows (for example in 2B map joining), a `_retire_point` / `_retire_keyframe` helper would keep the two paths from drifting. No action needed now.

### Checked and fine
- No stray TODOs, debug prints, commented-out code or hard-coded paths in the Python, C++, Rust or viewer diffs. The only `print`s are the CLI's output in `scripts/run_vo.py`, which are intended.
- Docstrings are numpy-style, Doxygen and rustdoc. The module docstrings, the `Pipeline` "Maps" paragraph, `Lifetimes` and `Options.reset_keyframes` state conventions: ids offset across maps, a map's own world frame, and per-map Sim(3) scale.
- Names and structure: `Segment`, `MapError`, `Evaluation`, `LostSummary` and the `State.LOST` semantics are clear. The "only a frame's result has LOST" rule is documented in Python and Rust.
- Run format v4, bundle v2 and the `Removal.RESET` / `REMOVALS` code lists agree across Python, `runs.rs` (`VERSION`, `REMOVALS = 4`) and the docs. Python and Rust `close` match line for line, including the state reset.
- Validation: `reset_keyframes` is in `_MAY_BE_ZERO`, so it is validated as a non-negative integer. `write_run` and `read_run` check the three map arrays before writing and after reading, and the Rust tests cover descending, negative and short maps.
- Dropping `min_init_depth_points` and `Outcome.TOO_LITTLE_DEPTH` is complete: docs, options and the initialisation docstring are all updated.
- `tracking`, `Evaluation` pooling (`_rms`) and the bundle `_aligned` / `_moved` per-map alignment are vectorised per map, with no per-element loops. `_aligned` copies the poses first, so unaligned maps stay as they were.
- `_run_here` in `vo.py` drops a reset map's poses by popping the trailing rows with that map number. That is correct because the maps are contiguous.
- The CHANGELOG entry and the VERSION bump (0.3.9.0) are present, and the entry reads as a specific, checkable list.

### Architecture review

## Architecture review

**Scope:** `git diff 0c258a4...96764f9` (7 commits, 96 files) on `feature/2a-a2-new-map`. Paths below are relative to the repository root. I read the edge pipeline in all three languages (`src/slam_it/edge/`, `cpp/{include,src}/slam_it/edge`, `rust/slam-it-edge/src`), the run format and its native writers (`eval/runs.py`, `cpp/src/eval/runs.cpp`, `rust/slam-it-eval/src/runs.rs`), the three run programs (`orchestration/vo.py`, `cpp/src/orchestration/vo.cpp`, `rust/slam-it-orchestration/src/vo.rs`), `bundles.py`, `worker.py`, the viewer and status-page modules, `scripts/run_vo.py` and `scripts/measure_testset.py`, the tests and the golden record generator, and the design-doc and notes changes. I checked all of it against the ladder doc and §5 of the 2A design doc. My probe scripts are in `/tmp/scratch/a2/panel/` (`arch_rpe_delta.py`, `arch_rpe_delta_base.py`, `arch_line_kind.py`). They are read-only and the working tree is still clean.

**Summary:** A2 is structurally sound. The new-map logic stays inside the edge rung, and it lands in Python, C++ and Rust together. The golden record now covers the lost and kept rooms. Run format v4 and bundle v2 are clean breaks, each with a version check, and the design doc records what was built. There is one must-fix: `runs.evaluate` now silently accepts an invalid `rpe_delta`, a regression from the base. The should-fixes share one theme: v4 leaves the meaning of "maps" implicit in several places. The per-frame `map` key is not in the format spec, `trajectory.txt` mixes frames, `final_points` still says "the final map", consumers each work out the reset encoding themselves, and the rule that drops a reset map's poses is split between the pipeline and the three run programs.

### Findings

#### [must-fix] `runs.evaluate` no longer refuses an invalid `rpe_delta`: the per-map RPE `except` swallows it
- **Where:** `src/slam_it/eval/runs.py:1062-1072` (`_scored`); `src/slam_it/eval/runs.py:1115-1156` (`evaluate` and its Raises section); `src/slam_it/eval/metrics.py:160-216` (`relative_error`)
- **Issue:** `_scored` calls `metrics.relative_error` once per map and treats any `InvalidInputError` as "this map has no pair a frame period apart" (`except InvalidInputError:  # no matched frame of the map has a pair`). But `relative_error` raises that same error when `delta`, `tolerance` or `scale` is out of range, and nothing checks `rpe_delta` before the loop. So an invalid argument from the caller is reported as "no RPE". The `evaluate` docstring's Raises section no longer mentions the case either. The design's error contract is also bent here: a generic invalid-input error is being caught to steer control flow.
- **Why it matters:** This is a public argument whose check used to exist and is now gone. A wrong sign, NaN or inf in `rpe_delta` produces an `Evaluation` with every RPE null. That looks exactly like the legitimate "reference too sparse" result A10 introduced, so nobody can tell the two apart.
- **Evidence:** I ran `arch_rpe_delta.py` at HEAD on `tests/fixtures/eval/run`, using its own poses as truth. `rpe_delta=None` gives an RPE of 0.0. `rpe_delta=-1.0`, `nan` and `inf` all return normally, with every map's relative error `None` and the pooled RPE `None`. I ran the same call against the base's `runs.py` (0c258a4, via `arch_rpe_delta_base.py`): `rpe_delta -1.0 -> raised InvalidInputError delta: must be positive and finite, got -1.0`.
- **Suggested fix:**
  - Check `rpe_delta` once in `evaluate` (positive and finite, via `as_number`) before any map is scored, and list it under Raises.
  - Make "no pair" an explicit test instead of a caught exception, so the `except` clause can go. One way: a small pairing helper in `metrics` that both `relative_error` and `_scored` use.

#### [should-fix] "No map can be aligned" is now reported as InvalidInput; a collinear trajectory used to be Degenerate
- **Where:** `src/slam_it/eval/runs.py:1057-1079`; `src/slam_it/orchestration/bundles.py:116-130`; `scripts/run_vo.py:86-91`; `scripts/measure_testset.py:193-207`
- **Issue:** `_scored` now treats a per-map `DegenerateError` as "unscored". When no map scores, it raises `InvalidInputError`. The ladder's shared error kinds (ladder doc lines 156-158) put "collinear points, too few correspondences" under `Degenerate`. The callers now catch `InvalidInputError` only, so a data condition (a run that cannot be placed) and malformed input share one kind. As a result, `bundles._score` cannot tell an unplaceable run from a broken one. The message also misleads: for 5 matched frames on a line it says "at most 5 of a map's frames have a ground-truth pose (3 needed, …)".
- **Why it matters:** The error contract should be the same in every module; this changes it silently at a public function. It also makes the bake's broad catch broader than it needs to be.
- **Evidence:** `arch_line_kind.py`, same run and truth with every centre on the x axis. HEAD: `InvalidInputError - no map can be aligned: at most 5 of a map's frames have a ground-truth pose (3 needed, their centres not on a line)`. Base: `DegenerateError - umeyama: points are collinear or coincident`.
- **Suggested fix:**
  - Raise `DegenerateError` for "no map can be aligned", since both causes are `Degenerate`'s, and name the cause for each map in the message.
  - Have `bundles._score` and `run_vo.score` catch `DegenerateError`.
  - Have `measure_testset` catch both, since it raises its own "no ground truth".

#### [should-fix] The diagnostics' `map` key is load-bearing but not in run format v4's specification
- **Where:** `src/slam_it/eval/runs.py:38-44` (the diagnostics bullet: "otherwise what the pipeline wants to report (tracked points, state)"); `runs.py:459-462` and `551-562` (only `timestamp` is checked); `src/slam_it/orchestration/bundles.py:240` (`named = … d.get("map")`) and bundles.py's `records.json` bullet; `viewer/src/frames.js:120-150` (`mapsShown` reads `records.map ?? []`); `viewer/src/scene_view.js` `show`/`showMap`
- **Issue:** Design §5.2 says "each frame's diagnostic names its map". The bake's per-map `frames` and the viewer's choice of map to show come only from that key. Yet neither the run format docstring nor the bundle's `records.json` description names it. `write_run` and `read_run` accept a v4 run without it, and so do the C++ and Rust writers' checks.
- **Why it matters:** A run can pass every v4 check and lack `map`, for example from another writer or a future backend. Such a run bakes with every map's `frames` as `[0, 0]`. In the viewer, `mapAt` then returns null on every frame, and `showMap(null)` hides the trajectory, keyframes and points: an empty scene for a run that has poses. Before A2, the viewer drew the trajectory whatever the diagnostics held.
- **Evidence:** In `bundles.py:240`, a missing key becomes -1, so `frames[named == map_id]` is empty for every map. In `frames.js`, `mapsShown` fills -1 and `mapAt` returns `null` for `map < 0`. In `scene_view.js`, `showMap(run, null)` sets `rows = [0, 0]` and `shownMap = -1`, so the shader's `abs(map - shownMap) < 0.5` hides every item.
- **Suggested fix:** Pick one:
  - (cleaner) Store the per-frame map as a checked array, as `pose_maps` already is: `frame_maps.npy`, `(F,)` int64, -1 where there is no map. All three writers would write it and all three checks would hold it.
  - (minimum) Specify `map` (and `state` and `lost`, which `tracking` reads) as v4's per-frame keys in the format docstring and the bundle docstring, and check them in `write_run`/`read_run`.

#### [should-fix] A reset map's poses are dropped by each run program, while its keyframes and points are reset by the pipeline
- **Where:** `src/slam_it/orchestration/vo.py:683-686`; `cpp/src/orchestration/vo.cpp:469-475`; `rust/slam-it-orchestration/src/vo.rs:627-638`; compare `src/slam_it/edge/pipeline.py` `_close` (RESET lives) and its C++ and Rust twins
- **Issue:** The 2026-10-13 decision says a reset map has "its keyframes and points removed, its frames' poses dropped". Two layers apply it:
  - The pipeline, in all three languages, marks the map's keyframes and points `RESET` in its lifetimes ledger.
  - The trajectory, which the pipeline streams frame by frame, is pruned by its consumer. That is the same pop loop written three more times, once in each run program.

  The pipeline's own result only says `lost.reset` and `map`. Any other consumer of `Pipeline` has to know to pop that map's poses: a live demo, or milestone 3's adapter path. In ORB-SLAM3 the tracker owns the trajectory record (`mlRelativeFramePoses`, `mlbLost`), and `Tracking::ResetActiveMap` marks the reset map's frames lost in it, the same object that resets the map.
- **Why it matters:** One rule now lives in six places. 2B will change what happens to a lost map (relocalisation before a new map, merging), so it has to change all of them. The golden record catches drift between the three programs, but not a new consumer that forgets the rule.
- **Suggested fix:** Give the pipeline the trajectory record, as it already has the lifetimes ledger: `Pipeline.trajectory()` returns the kept frames' times, poses and maps, and `_close` drops a reset map's entries. The run programs then read it at the end, as they read `lifetimes()`. Deferring this to 2B's atlas design (backlog) is reasonable.

#### [should-fix] `Run` has no map-aware queries: consumers work out maps and resets themselves, and `final_points` mixes frames
- **Where:**
  - `src/slam_it/eval/runs.py:210-231`: `final_points` and `final_keyframes` say "the final map".
  - `runs.py:21` and `runs.py:161`: points are "in the world frame".
  - `runs.py:933-941`: `tracking` works out resets from keyframe lives.
  - `bundles.py:243` and `bundles.py:251`: the map set from `keyframe_maps`, resets from lives.
  - `scripts/run_vo.py:195`: the map set from `keyframe_maps`.
- **Issue:** v4 encodes "which maps a run has" and "which were reset" only indirectly: through `keyframe_maps`, and through a keyframe life whose code is `reset`. Three consumers each decode that, two of them outside `eval`, and they disagree on where the map set comes from (`pose_maps` in `tracking`, `keyframe_maps` in the bake and `run_vo`). Meanwhile `Run.final_points()` and `final_keyframes()` still promise "the final map". They now return every kept map's geometry concatenated, each map in its own world frame (and, monocular, its own scale), with no map label. The format text still says the points are "in the world frame".
- **Why it matters:** The encoding has leaked out of the module that owns the format, so a 2B change such as merging maps touches every consumer. And the first caller to use `final_points()` as a point cloud (milestone 3's adapter, a map export) will get pieces in different frames without any error.
- **Suggested fix:**
  - Add `Run.maps()` (every map number, in order) and `Run.reset_maps()` next to `final_points`, and use them in `tracking`, the bake and `run_vo`.
  - Document `final_points`/`final_keyframes` as "every kept map's, each in its map's frame", or give them a `map` argument.
  - Change "in the world frame" to "in its map's world frame" in the format text and on `Run.points`.

#### [should-fix] `trajectory.txt` now holds several maps in different frames, while the format still presents it as ready for the TUM tools
- **Where:** `src/slam_it/eval/runs.py:14-17` ("what ORB-SLAM and the TUM tools write, so other systems' trajectories evaluate the same way"); ladder doc line 730 ("`trajectory.txt` in TUM's format (so other systems' trajectories score the same way)"); written by `runs.py:482`, `cpp/src/eval/runs.cpp` and `rust/slam-it-eval/src/runs.rs`
- **Issue:** In v4, `trajectory.txt` contains every kept map's poses, each in its own world frame and (monocular) scale. Which map a line belongs to is recorded only in `pose_maps.npy`. Given a multi-map run, evo or TUM's `evaluate_ate.py` fits one alignment over pieces in different frames and reports a meaningless ATE, and nothing in the file warns about it. For comparison, ORB-SLAM3's `System::SaveTrajectoryEuRoC` writes only the map with the most keyframes (`pBiggerMap`). That is from memory of its `System.cc`; worth confirming against the source.
- **Why it matters:** Formats are meant to be explicit, and the design doc's interop claim no longer holds. The test-set note's ADVIO runs keep about 8 maps each.
- **Suggested fix:** State the multi-map content in the format docstring and in the ladder's Metrics section. Then choose one, and have the doc say which:
  - mark map boundaries inside the file with a `# map <n>` comment (the format already allows `#` comments);
  - write one TUM file per map;
  - write only the largest map, as ORB-SLAM3 does, keeping `pose_maps.npy` for the full record.

#### [should-fix] Whether `SEGMENTS` fits the server's `SUMMARY_BYTES` is stated in a docstring, not held by `test_shared_names`
- **Where:** `src/slam_it/orchestration/worker.py:132-135`; `server/src/slam_it_server/app.py:89` and `:678` (413 "summary: too large"); `tests/server/test_shared_names.py:52-54` (`REASON_LENGTH`, the existing pattern)
- **Issue:** The worker caps a summary at 20 segments "so that a summary fits the server's 4,096 bytes". The cap is load-bearing: a 60 s clip (`video.MAX_SECONDS`) can lose the track every few frames, which means hundreds of segments and more than 4,096 bytes without the cap. Whether it fits depends on `SUMMARY_BYTES` and on how many digits a time can have. It is the one size relation crossing the `slam_it`/`slam_it_server` boundary that no test holds; `REASON_LENGTH` has one.
- **Why it matters:** A summary that does not fit is refused at `done`, after the whole run and bake, and the job fails.
- **Suggested fix:** Add a test in `test_shared_names.py`: build the worst-case `summarize` output (`SEGMENTS` pairs at `MAX_SECONDS`, the longest intrinsics source and mode) and assert `len(json.dumps(...)) <= app.SUMMARY_BYTES`.

#### [note] `State` serves as both the pipeline's state and the frame's outcome
- **Where:**
  - `src/slam_it/edge/pipeline.py:428`, `:510` and `:590`
  - `cpp/src/edge/pipeline.cpp:110`
  - `rust/slam-it-edge/src/pipeline.rs:399` and `:414`
- **Issue:**
  - `_record_failure` sets `state = LOST` for an instant, as a signal to `track`, which then closes the map. The documented invariant ("never `LOST`") therefore holds only between calls.
  - Rust keeps an arm that can no longer be reached: `State::Lost => Step::default()`.
  - The impossible case "lost with no map" is handled three different ways: Python `assert`s; C++ and Rust skip the close, which would leave the state at `Lost` for the rest of the run.
- **Suggestion:** Low priority. Flag the loss on the step (`_Step.lost`) and keep the pipeline's own state two-valued; frame results keep `LOST`.

#### [note] 2B will need an atlas; A2's bookkeeping is a seed of one
- When a map is kept, its points' and keyframes' positions go into the ledger and its `Map` (descriptors, observations) is dropped. 2B will relocalise against lost maps and merge them, which needs them whole.
- Map numbering, id bases, the ledger and the closed maps together are ORB-SLAM3's `Atlas`. In this PR they grew `Pipeline` in all three languages: `pipeline.py` from 548 to 660 lines, `pipeline.rs` from 754 to 864, `pipeline.cpp` from 401 to 441.
- Recommendation for the 2B design: an `edge.atlas` module, placed in the edge rung's import-linter order and in `check_layers.py`.

#### [note] The reifyscene boundary's `SlamResult` sketch assumes one world frame
- Ladder doc lines 630-637: `T_world_cam: (N, 4, 4)`, all in one world. A run is now several maps.
- Before milestone 3 settles `foundations.interfaces`, the doc should say how the adapter takes a multi-map run: the largest map, per-frame map ids, or only after 2B joins the maps.

#### [note] `runs.py` mixes the format's I/O and its scoring
- The file grew from 985 to 1,156 lines. Design §5.2 names `runs.evaluate` and `runs.tracking`, so this is not a divergence from the design.
- A module above `runs` in eval's order is a candidate split for the backlog, for example `eval.scoring` holding `Segment`, `Tracking`, `MapError`, `Evaluation` and `evaluate*`.

#### [note] The viewer's rule for which maps share a frame lives in a view module, untested
- **Where:** `viewer/src/scene_view.js:99-101` (`placed`) and `:335-347` (`showOthers`).
- **What:** The rule for overlaying another backend's map on the one shown is decision logic: both maps placed on the truth, or both in their own frames with the same id. It sits in a `*_view.js` file, which Node does not test and the no-DOM check excludes.
- **Suggestion:** Move it into `frames.js` beside `mapsShown`, `mapAt` and `mapRows`, and test it there.

#### [note] Out of lane, for the algorithm and hygiene reviewers
- The A2 loss table and the seed 1 and 2 runs in `docs/notes/test-set.md` come from code outside the repo: the note's "local wrapper" for seeds, and a loss classification that `measure_testset.py` does not contain. A `--seed` option and the classification in the measure would let the note be regenerated from the repo.
- Design doc §2 says ORB-SLAM3 resets a map "of fewer than 10 keyframes". The A2 decisions say "10 keyframes or fewer, as ORB-SLAM3 resets its active map". The code follows the decision; the algorithm reviewer should settle which description of ORB-SLAM3 is right.

### Checked and fine
- **Dependency direction:**
  - No new cross-rung imports. The edge rung refers to eval and orchestration only in docstrings.
  - No new packages or modules, so the import-linter contracts and `check_layers.py` need nothing new.
  - The worker still talks to the server over HTTP only.
- **Rung placement:** map closing and reset are in the edge rung, scoring is in eval, the bake, worker and run programs are in orchestration, and `Removal.RESET` sits in `edge.map` beside the other removal codes.
- **Language split:**
  - The change landed in Python, C++ and Rust together, with native tests (`cpp/tests/edge/test_pipeline.cpp`, `rust/slam-it-edge/tests/pipeline.rs`).
  - The golden record gained the lost and kept rooms and holds every backend (`test_vo`).
  - `MOVED_OPTIONS` moves `reset_keyframes` off its default, so a native backend that ignored it would fail.
  - No binding changed; the pipeline still runs natively, not across the binding. Rust stays within its agreed subset.
- **Options:**
  - `reset_keyframes` was added to `kIntegerOptions`, to `named_settings!` and to the may-be-zero checks in all three languages.
  - `min_init_depth_points` and `TOO_LITTLE_DEPTH` were removed everywhere, with no shim.
  - The outcome codes were renumbered consistently, and `codes()` derives from the enums.
- **Run format v4 and bundle v2:**
  - The three writers write the same three map arrays, and their `check_maps` checks are equivalent, each returning `InvalidInput`.
  - `read_run` and the viewer's `loadBundle` refuse other versions (clean breaks).
- **Error contract (native):** maps are validated with `InvalidInput` in C++ (`std::expected`) and Rust (`Result`), matching Python.
- **Compute budgets:** no new dependencies; the edge rung stays CPU-only; the lifetimes ledger grows as it did before A2.
- **Viewer:**
  - The new map logic (`mapsShown`, `mapAt`, `mapRows`, `mapDepths`, `mapFrustums`, `mapAttribute`) is in logic modules, tested by Node and covered by `tsconfig.logic.json`'s glob.
  - `compare.js` adds `map` to the decisions compared across backends.
- **Design docs:** the ladder's edge-rung bullet, open items L13 and A10, and §5 "as built" of the 2A design were all updated in this PR.
- **Privacy:** the test-set note's figures are all from public datasets; no private recording or its details appear.

### Algorithm review

## Algorithm review

**Scope:** `0c258a4...96764f9` on `feature/2a-a2-new-map` (7 commits). I read in full: the pipeline's close, reset and new-map logic in all three languages (`src/slam_it/edge/pipeline.py`, `cpp/src/edge/pipeline.cpp`, `rust/slam-it-edge/src/pipeline.rs`); the change to start a depth map without a depth count (`initialization.*`, `options.*`); dropping a reset map's poses in each orchestrator (`orchestration/vo.py`, `cpp/src/orchestration/vo.cpp`, `rust/slam-it-orchestration/src/vo.rs`); per-map scoring (`src/slam_it/eval/runs.py`: `_scored`, `Evaluation`, `tracking`); and per-map alignment in the bake (`orchestration/bundles.py`). I also read the edge, eval and orchestration tests, the golden-record generator, `scripts/measure_testset.py`, the design notes and the test-set note.

I compared the code against ORB-SLAM3's source: `Tracking.cc` and `System.cc` from UZ-SLAMLab/ORB_SLAM3 master. I also ran experiments from scratch scripts in `/tmp/scratch/a2/panel/`.

**Summary:** The new-map mechanism is correct and matches ORB-SLAM3. The reset rule (`<= reset_keyframes`) matches ORB-SLAM3's deciding check. A reset map's poses are dropped as `ResetActiveMap` drops them. Numbering each map's keyframes from 0 keeps local BA's gauge (`m.kf_id == 0`) correct. Run ids continue correctly across maps: I ran a 4-map room (whose first map has no points) on Python, C++ and Rust, and all three decide the same. Each map is aligned on its own (Sim(3) for monocular, SE(3) otherwise) and the per-map errors are pooled correctly.

There is one must-fix. `evaluate` now swallows an invalid `rpe_delta`. The rest are test gaps at the new paths' edges, and a run-format trap: `trajectory.txt` now mixes the frames of several maps.

### Findings

#### [must-fix] `evaluate` turns an invalid `rpe_delta` into "no RPE" instead of refusing it
- **Where:** `src/slam_it/eval/runs.py:1071` (`except InvalidInputError: drift = None` in `_scored`); the docstring's Raises section at `runs.py:1115` onward.
- **Issue:** The `except` is meant for one case: a map with no frame pair a period apart. But `metrics.relative_error` raises `InvalidInputError` for every bad argument, including a `delta` that is not positive and finite. So `evaluate(run, t, P, rpe_delta=-1.0)` (or `0`, `nan`, `inf`) no longer fails. It returns an `Evaluation` with every map's RPE None. At base `0c258a4` the same call raised.
- **Why it matters:** A caller's mistake looks exactly like the legitimate result "the reference is too sparse for an RPE". That breaks "validate inputs at public boundaries; no swallowed errors". The broad `except` would also hide any future bug in what `_scored` passes to `relative_error`.
- **Evidence:** `scratchpad/a2/panel/rpe_delta.py` at HEAD prints:
  ```
  rpe_delta=-1.0: no error; rpe_translation_rmse=None
     metrics.relative_error: InvalidInputError: delta: must be positive and finite, got -1.0
  rpe_delta=0.0: no error; rpe_translation_rmse=None
  rpe_delta=nan: no error; rpe_translation_rmse=None
  rpe_delta=inf: no error; rpe_translation_rmse=None
  ```
- **Suggested fix:** Check `rpe_delta` (positive and finite) once, at the top of `evaluate` or `_scored`, before the loop. Then narrow the catch to the no-pair case only. Either check that the map has a pair before calling, or have `relative_error` report "no pair" in a way the caller can tell apart from bad input. Add a test that `evaluate(..., rpe_delta=-1.0)` raises. This is a few lines.

#### [should-fix] `trajectory.txt` now joins several maps' frames into one file
- **Where:** `src/slam_it/eval/runs.py:14-17` (the docstring says the file is "what ORB-SLAM and the TUM tools write, so other systems' trajectories evaluate the same way"), `runs.py:482`, and the native writers. The same single-frame wording remains at `runs.py:21-26` ("in the world frame"; "The final map is the rows still alive") and `runs.py:210-221` (`final_points` / `final_keyframes`: "still in the final map").
- **Issue:** In v4 the file holds every kept map's poses back to back. Each map has its own world frame and, in monocular, its own scale, and nothing in the file marks where one map ends. The same goes for `final_points()` and `final_keyframes()`: they now return every kept map's points and keyframes, each in its own frame, not "the final map".
- **Why it matters:** The file is offered for outside tools. evo's `evo_ape` or TUM's `evaluate_ate.py` would fit one alignment to the whole file and report a meaningless ATE without any warning. The repo's own scoring is right only because it reads `pose_maps.npy`. ORB-SLAM3 avoids this: `System::SaveTrajectoryEuRoC` writes only the map with the most keyframes. It skips frames whose reference keyframe is in another map (`if(!pKF || pKF->GetMap() != pBiggerMap) continue;`, `System.cc` around lines 672-745), and `SaveTrajectoryEuRoC(filename, pMap)` writes a chosen map.
- **Evidence:** `tests/fixtures/eval/run/trajectory.txt` at HEAD has five lines: two from map 0, then three from map 1 (`pose_maps: [0, 0, 1, 1, 1]`), with no separator. ORB-SLAM3 source: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/System.cc
- **Suggested fix:** At minimum, update the docstrings: the file spans maps and must be split with `pose_maps.npy`, and `final_*` returns every kept map's rows. Better, also write one TUM file per map (`trajectory_map<k>.txt`), or follow ORB-SLAM3 and add a largest-map file, so outside tools get a file they can align.

#### [should-fix] No test sits on the reset boundary, so a `<` vs `<=` slip in one language would pass
- **Where:** `tests/edge/test_pipeline.py:145-198` (`reset_keyframes` 10 vs 0 on a 1-keyframe map); `cpp/tests/edge/test_pipeline.cpp:147` (`{10, 0}`); `rust/slam-it-edge/tests/pipeline.rs:86` (`[10, 0]`); `tests/orchestration/datasets.py:176` (`KEPT_OPTIONS`, `reset_keyframes=0`).
- **Issue:** Every test puts the map's keyframe count strictly inside one side of the rule. With `<`, a 1-keyframe map is still reset at 10 and still kept at 0. The golden record's lost room is no help either: its map 0 has 3 keyframes (1 in stereo) and runs at 10 or 0. `MOVED_OPTIONS`' `reset_keyframes=9` runs on the gap room, which never loses the track.
- **Why it matters:** This is the comparison the owner's decision turns on ("10 keyframes or fewer"). ORB-SLAM3 itself writes it two ways: `KeyFramesInMap()<10` at `Tracking.cc:2019` and `<=10` at `Tracking.cc:2273`. That makes it a likely place for an off-by-one, and nothing here would catch one.
- **Evidence:** The rule is `m.keyframes <= self.options.reset_keyframes` (`pipeline.py:512`), `std::cmp_less_equal(...)` (`pipeline.cpp:181`), and `m.kf_id.len() <= self.options.reset_keyframes` (`pipeline.rs:505`). All three are correct today. In my 4-map experiment (below), `reset_keyframes=1` reset the 1-keyframe maps identically in all three, but no test fixes that.
- **Suggested fix:** In the three unit tests, use `reset_keyframes = 1` (equal to the map's keyframe count, so it must reset) beside `0` (one below, so it must keep), instead of 10 and 0.

#### [should-fix] A map with no points, and a third map, are not tested end to end
- **Where:** `tests/edge/test_pipeline.py:238` (`test_a_frame_with_little_depth_starts_the_map`: a strip of depth, so more than 0 points, and no frames tracked after the start); `tests/edge/test_initialization.py` (`kept=0` tests the start alone); the golden record's lost room (one loss, so two maps).
- **Issue:** Dropping `min_init_depth_points` rests on one claim: "a map started from too little depth to track loses the track and is reset". No test runs that path: a 0-point RGB-D map tracked through motion, PnP and the local map on empty arrays, then lost and reset. The run-id offsets (`kf_base`/`pt_base = len(lives)`) are also only tested with two maps. Offsetting by the previous map's count instead of the running total would pass every current test and break only from the third map on.
- **Why it matters:** These are the new paths' edge cases. Their only coverage today is C++ runs on the test set, and those are not in the suite and not compared across languages.
- **Evidence:** Both are correct now. `scratchpad/a2/panel/three_maps.py` ran a room whose frame 0 has no depth and which has two blank stretches, giving 4 maps in RGB-D and 2 in monocular, on all three backends with `reset_keyframes=1`. RGB-D: `lost [{'keyframes': 1, 'points': 0, 'reset': True}, {..., 'points': 606, 'reset': True}, {..., 'points': 607, 'reset': True}]`, then `kf lives [[0, 5, 3], [6, 14, 3], [15, 26, 3], [27, -1, -1]]`. Each backend printed `rgbd cpp decides as python: True`, `rgbd rust decides as python: True`, `monocular cpp ...: True`, `monocular rust ...: True` (states, maps, lives, map arrays, pose maps).
- **Suggested fix:** Add a Python pipeline test that starts from a frame with no depth, loses the track twice and checks map 2's run ids. Also add a third map (a second blank stretch) to one golden-record room so all three languages are held to the offsets. The scratch script above can be the starting point.

#### [note] ORB-SLAM3 gives a small map no grace before it is lost; the design's citation is slightly off
- **Where:** `docs/design/2026-10-12-phone-robust-vo.md:62` ("resetting one of fewer than 10 keyframes"); `src/slam_it/edge/options.py:227-230`.
- **Issue:** In ORB-SLAM3 the check that decides is `if(pCurrentMap->KeyFramesInMap()<=10) ResetActiveMap()` (`Tracking.cc:2273`), so "10 or fewer" is right. The PR's rule and its options docstring ("up to 10") match it. The design note's "fewer than 10" quotes the `<10` branch at line 2019, which doesn't decide. ORB-SLAM3 also sends a map of 10 keyframes or fewer straight to LOST on its first failed frame (`else if(pCurrentMap->KeyFramesInMap()>10) mState = RECENTLY_LOST; else mState = LOST;`, line 1966). Larger maps get up to 3 s of bag-of-words relocalisation first (line 2006). The lite pipeline gives every map `lost_frames` (5) frames of local PnP. That is a reasonable lite choice, but it is a difference worth stating in the design next to the decision.
- **Evidence:** https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc (lines 1966, 2006, 2019, 2273 on master).

#### [note] A short map's own Sim(3) fit makes its ATE look small
- **Where:** `src/slam_it/eval/runs.py` `_scored`, and the per-map ATE in `measure_testset.py` / `run_vo.py`.
- **Issue:** As decided, each map is aligned on its own, and that is right for maps in separate frames. But a 7-DoF fit to a map of a few frames, or one that barely moves, leaves almost no error by construction. ATE also grows with a trajectory's length, so a run broken into many short maps scores lower than one long map. The pooled figure weights by frames, which limits this, and the test-set note already says A2's ATE does not compare with A1's.
- **Suggested context:** Report each map's duration or path length beside its ATE. For figures that compare across maps of different lengths, use a distance-based relative error. Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018 (https://doi.org/10.1109/iros.2018.8593941), covers relative error over sub-trajectory lengths for exactly this, along with the choice of alignment per sensor.

#### [note] The run format checks each map array alone, not against the others
- **Where:** `runs._check_maps` (Python) and the C++ and Rust `check_maps`.
- **Issue:** Each array is checked to be non-negative and never descending. Nothing checks that every map named in `pose_maps` has a keyframe in `keyframe_maps`, or that a map whose keyframes were removed as `reset` has no poses. The scoring, `tracking` (`resets`) and the bake's `_maps_info` all rely on those two facts. Worth adding if the format is read from outside sources.

#### [note] Where the field is on lost tracks (context for 2B)
- The new map follows ORB-SLAM3's Atlas (Campos et al., "ORB-SLAM3", T-RO 2021), which later joins maps on place recognition. A current learned take on joining disjoint pieces is Lipson & Deng, "Multi-Session SLAM with Differentiable Wide-Baseline Pose Optimization", CVPR 2024 (https://arxiv.org/abs/2404.15263): it tracks across disjoint videos under one global frame. This is context for 2B's map joining, not a defect: this rung deliberately implements the classic method.
- The test-set note's two-view comparison is right. ORB-SLAM3's `TwoViewReconstruction(K, sigma = 1.0, iterations = 200)` (`include/TwoViewReconstruction.h:38`) against this repo's `max_iterations = 10000` (`foundations/twoview.py:165`).

### Checked and fine
- **Closing a map (Python, C++, Rust):** `_date()` runs before `_close()`, so removals on the lost frame keep their own codes and only the points and keyframes still in the map are marked `reset`. The tracking state is fully cleared: last frame, velocity, monocular reference and previous points, failure count, frames since recovery.
- **Monocular restart:** after a loss the next frame gets `init_feature_factor` times the features, because the state is NOT_INITIALIZED, as ORB-SLAM's `mpIniORBextractor` is used. The lost frame is not reused as the reference, as with ORB-SLAM3's `CreateMapInAtlas` and `ResetActiveMap`.
- **Gauge:** each map numbers its keyframes from 0, so local BA's fixed keyframe (`mapping.py:991`, `m.kf_id == 0`) and culling's kept keyframe are each map's own first keyframe. Each map's first keyframe is at identity.
- **Run ids:** each map's offsets are the counts of everything made before it (`len(lives)` at its start). Fates, lives and lifetimes all use them consistently. Checked by inspection and with the 4-map experiment on all three backends.
- **Reset map's poses:** dropped by popping that map's poses off the end of the trajectory in all three orchestrators, which is equivalent to ORB-SLAM3's `ResetActiveMap` marking the active map's frames lost (`Tracking.cc` around lines 3880-3905).
- **Per-map alignment:** Sim(3) for monocular and SE(3) otherwise, each map against its own matched frames. A collinear map raises `DegenerateError`, which is caught and listed in `unscored`. Fewer than 3 matched frames is listed the same way. No scored map at all raises.
- **RPE:** pairs stay inside one map. A monocular map's RPE uses that map's own Sim(3) scale. The pooled RMS is computed correctly (`test_each_map_is_aligned_on_its_own` checks it against the errors concatenated).
- **A10:** the bake now aligns each map by its ATE alone, so a reference too sparse for the RPE still places the run with its RPE None. Tested for both `evaluate` and `evaluate_absolute`.
- **Bake:** `_aligned` and `_moved` move each map's poses, keyframes and points by that map's own similarity. Recovering the rotation by `det^(1/3)` is correct for s > 0.
- **`tracking()`:** one segment per map that has poses, losses counted from diagnostics, resets from keyframe lives. Tested with a kept map, a reset map and a third map.
- **Depth start:** the frame needs more than `min_init_features_depth` keypoints, counted with or without depth. This matches ORB-SLAM3's `StereoInitialization` (`N>500`). A map with no points is lost after `lost_frames` and reset in all three backends (my experiment).
- **Golden record:** the lost and kept rooms cover all 4 dataset/mode cases on all 3 backends. The targeted tests pass at HEAD: `tests/eval/test_runs.py` and `tests/edge/test_pipeline.py`, 17 passed.

## Run 2 -- 2026-10-14

| | |
|---|---|
| **Branch** | `feature/2a-a2-new-map` |
| **Range** | `96764f9...cb377fd` (the fixes of Run 1's decisions) |
| **Reviewers** | architecture (opus), algorithm (opus): the areas the fixes changed most |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed at cb377fd (`scripts/check.sh` in the `slam-it` env, exit 0: 25 checks; pytest 3756 passed) |

Both reviewers found every Run 1 fix in their lanes resolves its finding as decided. New findings
are numbered on from Run 1's; L11 is A14 seen from the algorithm lane.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A14 | should-fix | architecture | `cpp/src/eval/runs.cpp`, `rust/slam-it-eval/src/runs.rs` `check` | A3's check of the diagnostics' `map` went into Python's writer and reader alone; the C++ and Rust writers, documented as its mirrors, accept what Python refuses | fix (the same check in both `check`s, Python's messages, the four refusals tested natively) | fixed (bb6ab32: `check_frame_maps` in C++ and Rust, Python's messages, the four refusals tested natively) -- decided by evan-wes, 2026-10-14 |
| A15 | should-fix | architecture | `CHANGELOG.md` 0.3.9.0; design §5.2 | The entry does not record the fixes' public changes: `DegenerateError` for "no map can be aligned", the `# map <n>` lines, the checked `map` key, `Run.maps`/`reset_maps`, `metrics.interval_pairs` | fix (the entry and §5.2) | fixed (6d9b3f7: the 0.3.9.0 entry and design §5.2) -- decided by evan-wes, 2026-10-14 |
| A16 | note | architecture | `metrics.relative_error`; `_score_map` | "No pair" is still `InvalidInputError` in `relative_error`, its message lost the tolerance, and `_score_map` pairs twice | fix (no pair `DegenerateError`, the tolerance back in the message); the double pairing waived: a sort per map | fixed (bb6ab32: no pair is `DegenerateError`, the tolerance back in the message); the double pairing waived: a sort per map -- decided by evan-wes, 2026-10-14 |
| A17 | note | architecture | `bundles._maps_info`; `Run.diagnostics` | The bake keeps a fallback `read_run` now rules out; `Run.diagnostics` names only the timestamp | fix (the fallback dropped; the attribute names `map`) | fixed (bb6ab32: the fallback dropped; `Run.diagnostics` names `map`) -- decided by evan-wes, 2026-10-14 |
| A18 | note | architecture | the ladder's link to this report | The report is not committed at cb377fd | fix (this report is committed in the PR) | fixed (this report, committed in the PR) -- decided by evan-wes, 2026-10-14 |
| L9 | should-fix | algorithm | design §5, item 1 | Run 1's L5 wording, carried into the fix: ORB-SLAM3 loses a small map at once only when its frame-to-frame step fails; a local-map failure gets up to 3 s of relocalisation whatever the map's size | fix (the reviewer's sentence) | fixed (6d9b3f7: the reviewer's sentence) -- decided by evan-wes, 2026-10-14 |
| L10 | note | algorithm | `tests/orchestration/datasets.py` `LOST_AT` | The second stretch cuts monocular map 1 from 12 tracked frames to 3 (kept) and none (reset) | fix (said in `LOST_AT`'s docstring); moving the stretch waived: the room has 40 frames | fixed (bb6ab32: `LOST_AT`'s docstring); moving the stretch waived: the room has 40 frames -- decided by evan-wes, 2026-10-14 |
| L11 | note | algorithm | as A14 | As A14 | as A14 | fixed (as A14, bb6ab32) -- decided by evan-wes, 2026-10-14 |
| L12 | note | algorithm | `_score_map` | "its centres are on a line" also fires for a degenerate truth | fix ("its centres, or the truth's, are on a line") | fixed (bb6ab32: "its centres, or the truth's, are on a line") -- decided by evan-wes, 2026-10-14 |

Decided by evan-wes on 2026-10-14, by selectable prompts: the three should-fixes fixed, the
notes as recommended (the double pairing and moving the blank stretch waived). Layer 1 passed
at bb6ab32 with 6d9b3f7's docs in the tree (25 checks; pytest 3756 passed). The fixes were
small and in Run 2's lanes, so no Run 3 was held.

### Architecture review

## Architecture review (Run 2)

**Scope:** The Run 1 fixes, `git diff 96764f9...cb377fd`: 5 commits and 29 files in the repository root. Paths below are relative to that root.
- I read every changed file, plus the neighbours the fixes depend on: the C++ and Rust `check()`, `close()` and `track()`, `bundles._maps_info`, `worker.summarize`, and the server's `SUMMARY_BYTES` check.
- I compared the golden record before and after the change.
- I ran a probe of `evaluate`'s refusals and error kinds: `/tmp/scratch/a2/panel2/arch2_probe.py`.
- I ran the targeted tests with `-p no:cacheprovider`: `tests/eval/test_runs.py`, `test_metrics.py`, `tests/edge/test_pipeline.py` and `tests/server/test_shared_names.py`. All 155 passed, and the working tree is unchanged.

**Summary:** Each fix in my lane resolves its finding as decided. One fix adds a new inconsistency across the three languages:
- A3's check of the diagnostics' `map` key went into Python's `write_run` and `read_run` only.
- The C++ and Rust writers, which document themselves as mirrors of `slam_it.eval.runs.write_run`, still accept a run Python now refuses.

Separately, the 0.3.9.0 CHANGELOG entry does not record two contract changes the fixes made: the error kind `evaluate` raises when no map can be aligned, and the run format's new `# map <n>` lines. The rest are small notes.

### Findings

#### [should-fix] The diagnostics' `map` check exists only in Python's writer; the C++ and Rust `write_run` accept what Python refuses
- **Where:**
  - Python: `src/slam_it/eval/runs.py:396-420` (`_check_frame_maps`), called at `:547` (`write_run`) and `:677` (`read_run`).
  - C++: `cpp/src/eval/runs.cpp:249-258` (`check`'s diagnostics loop); `cpp/include/slam_it/eval/runs.hpp:191-202` (`write_run`'s documented refusals).
  - Rust: `rust/slam-it-eval/src/runs.rs:477-486` (`check`) and `:557-572` (`write_run`'s `# Errors`).
  - Tests: `cpp/tests/eval/test_runs.cpp:306-366` (`check_refusals`).
- **Issue:**
  - 367daa6 made Python refuse four kinds of bad diagnostics: a `map` that is not null or a map number, a `True`, maps that descend, and a map with keyframes that no frame names. Four new parametrized cases in `tests/eval/test_runs.py` test them.
  - The native `check()` still tests only that each diagnostic has a `timestamp` and finite numbers.
  - Both native writers say they are Python's `write_run`: the C++ header says "the C++ mirror of `slam_it.eval.runs.write_run`", and Rust's doc says `slam_it.eval.runs.write_run`. Each lists its refusals.
  - Run 1 found the three writers' map checks equivalent. That held at 96764f9 and no longer does.
  - The same commit applied H3 "in all three languages", but not A3. The native refusal suites mirror Python's other refusals case by case (`maps that descend are refused`, `a negative map is refused`) but have none of the four new ones.
- **Why it matters:**
  - The design's rule is that each C++ and Rust implementation is held to its Python reference, with the same error contract.
  - Suppose a native run program drops or misorders `map`, for example when 2B's atlas reworks map numbering. The native program writes the run and exits successfully. The run is refused only later, when Python's `read_run` runs: in the bake, the measure or the worker, after the whole run.
  - Today both native programs write `map` on every frame (`cpp/src/orchestration/vo.cpp:446`, `rust/slam-it-orchestration/src/vo.rs:606`). So no current run is affected, and the refusal stays observable. That is why this is a should-fix rather than a must-fix.
- **Evidence:**
  - C++ `check()` ends with only:
    ```cpp
    const bool stamped = std::ranges::any_of(entry, [](const auto& item) { return item.first == "timestamp"; });
    if (!stamped || !finite(entry)) { return fail(... "]: expected an object with a timestamp, its numbers finite"); }
    ```
  - Rust's is the same: `if !entry.iter().any(|(key, _)| key == "timestamp") || !finite(entry)`.
  - `grep -rn '"map"\|frame_maps\|names map' cpp/src/eval rust/slam-it-eval/src` finds nothing.
- **Suggested fix:**
  - Add a `check_frame_maps` to C++ `check()` and Rust `check()`. It should accept a `map` that is null or a non-negative integer, never descending, and require every `keyframe_maps` value to be named on a frame. Use Python's messages.
  - Add the four refusal cases to `check_refusals` and to the Rust tests, and extend both doc comments.
  - If the owner meant "the minimum" to be Python-only, then say so instead in the two native doc comments and in A3's Resolution, so the mirror claim is not false.

#### [should-fix] The 0.3.9.0 entry does not record the fixes' public changes: `evaluate`'s new error kind, and the trajectory's map lines
- **Where:** `CHANGELOG.md:28-30` (the "Run format version 4" bullet), `:47-62` ("Changed"); `docs/design/2026-10-12-phone-robust-vo.md` §5.2.
- **Issue:** The fixes changed public behaviour that the entry, which describes 0.3.9.0 against 0.3.8.0, does not mention:
  - **Error kind.** At 0.3.8.0, `evaluate` and `evaluate_absolute` raised `InvalidInputError` when "fewer than three frames match" (base docstring, `runs.py:931` and `:970` at 0c258a4). They now raise `DegenerateError` for that case and name each map's cause.
  - **Run format.** `trajectory.txt` gains `# map <n>` lines, and the writers and `read_run` now refuse diagnostics whose `map` keys are malformed.
  - **New public API:** `Run.maps()`, `Run.reset_maps()` and `metrics.interval_pairs`.
  - **Design doc.** §5.2, the "as built" text of run format v4, mentions neither the map lines nor the check. Only the ladder's Metrics section has the map lines.
- **Why it matters:**
  - The error kind is a change of contract at a public function. 0.3.8.0's bake caught `InvalidInputError` for exactly this case, so any caller written that way now fails with an uncaught error.
  - The map lines change a file the format offers to outside tools.
  - A reader of the CHANGELOG or the design learns of neither.
- **Evidence:** The CHANGELOG diff in the range touches only "The lost room" bullet (`git diff 96764f9...cb377fd -- CHANGELOG.md`).
- **Suggested fix:**
  - Under "Changed", add one line: "no map can be aligned" is now `DegenerateError`, naming each map's cause.
  - Extend the v4 bullet with the `# map <n>` lines, the checked `map` key, and `Run.maps`/`reset_maps`.
  - Add one sentence to §5.2 for the same two format points. (The hygiene reviewer may also report the CHANGELOG; I raise it for the contract change.)

#### [note] `metrics.relative_error` still reports "no pair" as `InvalidInputError`, the same kind as a bad argument
- **Where:** `src/slam_it/eval/metrics.py:257-258`; `src/slam_it/eval/runs.py` `_score_map`.
- **Issue:**
  - H1's root cause was that one error kind covered both a caller's mistake and a data condition. The fix sidesteps it: `_score_map` checks `interval_pairs` first, and nothing is caught.
  - But the public `relative_error` still raises `InvalidInputError` for "no pose has another delta s later". Under A2's decision, a data condition of too few correspondences is `Degenerate`, so the next caller of `relative_error` can fall into the same trap.
  - Two smaller points:
    - The message lost its tolerance value. Base: `f"no pose has another {interval} s (+-{window} s) later"`. Now: `"... within the tolerance"`.
    - `_score_map` computes each map's pairs twice: once to test, and again inside `relative_error`.
- **Suggestion:** Low priority; defer, or bundle it with A11's `eval.scoring` split:
  - make "no pair" a `DegenerateError` (or let `relative_error` take precomputed `pairs`);
  - restore the tolerance value to the message.

#### [note] Two leftovers around A3's check
- **The bake's fallback.** `src/slam_it/orchestration/bundles.py:246-251` now comments "read_run holds every map named on a frame", but it keeps the `[0, 0]` fallback for a map with no named frame. That branch is now unreachable for a run that `read_run` accepted. Drop it, or label it as defensive.
- **The `Run` docstring.** `Run.diagnostics` (`runs.py:169-170`) still says only "each with a `timestamp`". The `map` key that `write_run` now requires is stated only in the module docstring.

#### [note] The ladder links to a report that is not committed yet
- `docs/design/2026-10-02-slam-it-ladder.md:1012` links `../reviews/2026-10-14-feature-2a-a2-new-map.md`.
- At cb377fd the file is untracked (`git ls-tree cb377fd docs/reviews` does not list it).
- It needs to be committed in this PR, as A1's reports were (51a8f31), or the link is dead on `develop`.

### Checked and fine
- **H1/A1/L1 (`rpe_delta`):**
  - `rpe_delta` is checked once, through `as_number`, before any map is scored. The "no pair" case is decided by `metrics.interval_pairs`, which `relative_error` also uses, and nothing is caught.
  - Probe output:
    - -1, 0, NaN and inf each raise `rpe_delta: must be positive and finite`.
    - A bad `rpe_delta` on a run that also cannot be aligned raises the input error first.
- **A2 (error kind):**
  - "No map can be aligned" is now `DegenerateError` and names each map's cause. Probe: "map 0: 0 frames matched, 3 needed"; a run with no poses gives "the run has no poses".
  - This matches the layer below, since `metrics.absolute_error` already raises `Degenerate` for fewer than three poses.
  - Narrowing the bake, `run_vo` and `measure_testset` to catch only `DegenerateError` is safe. Every other refusal in `_scored` needs input that `read_run` or the truth readers already refuse.
- **H2/H4:** `_score_map` is split out. `Evaluation.ate_rmse` has no assert and relies on `__post_init__` refusing an empty `maps`. Python's `_close(m)` takes the map, as C++ and Rust do, and all three reset the same tracking state.
- **A8 (part):** All three `track()` methods now dispatch alike (tracking, or else initialize), and all three close only when a map exists. Rust's unreachable `State::Lost` arm is gone.
- **A5:**
  - The bake, `run_vo` and `tracking` now use `Run.maps()` and `Run.reset_maps()`. A grep of `src/` and `scripts/` finds no other code decoding reset lives or the map set.
  - `_scored` and `tracking` iterate the maps with poses, as they should.
- **A6:**
  - All three writers put a `# map <n>` line before each map's first pose.
  - The C++ and Rust golden tests compare where the comment lines fall against Python's.
  - The readers skip `#` lines. The format text and the ladder's Metrics section say to align one map at a time.
- **H3:** The messages for a bad map row and for wrong lengths read the same in Python, C++ and Rust.
- **L3/L4:**
  - The reset boundary (`reset_keyframes` 1 and 0) is tested in all three languages.
  - In the golden record only the 8 lost and kept cases changed. The room and gap cases are byte-identical, checked by a JSON diff of 96764f9 against cb377fd.
  - A stereo or RGB-D third map now holds all three languages to the cross-map id offsets.
- **A7:**
  - The summary-size test builds its case from the real names (`RUN_MODES`, `lenses.SOURCES`, `MAX_SECONDS`, `MAX_LONG_SIDE`), so it follows them if they change.
  - The server measures the same way: `len(json.dumps(summary))` (`app.py:678`). The 586 bytes leave a wide margin under 4,096.
- **A12:** `placed` and `sharesFrame` now live in `frames.js`, a logic module with no imports and no DOM. `scene_view.js` imports them, and Node tests them case by case.
- **Dependencies:**
  - No new modules or packages, so the import-linter contracts and `check_layers.py` need nothing new.
  - `tests/server/test_shared_names.py` already bridged `slam_it` and the server.
  - No bindings changed, and Rust stays within its agreed subset.
- **Deferred findings:** A4, A9, H7, the rest of A8, A10, A11, A13 and L7 are each recorded under the ladder's open items with their IDs. Design §2 now says 10 or fewer (L5), and §5 states the grace-frame difference from ORB-SLAM3.

### Algorithm review

## Algorithm review (Run 2)

**Scope:** The fixes in `git diff 96764f9...cb377fd` on `feature/2a-a2-new-map` (b682f3c, 367daa6, 75ac019, 85fa052, cb377fd; 29 files). Paths are relative to the repository root.
- **Read in full:** `src/slam_it/eval/{metrics,runs}.py` (`interval_pairs`, `_score_map`, `_scored`, `_check_maps`, `_check_frame_maps`, `_trajectory_text`, `Run.maps`/`reset_maps`, `Evaluation`); `track`/`_close` in `src/slam_it/edge/pipeline.py`, `cpp/src/edge/pipeline.cpp` and `rust/slam-it-edge/src/pipeline.rs`; `check_maps`/`trajectory_text` in `cpp/src/eval/runs.cpp` and `rust/slam-it-eval/src/runs.rs`; the callers (`bundles.py`, `scripts/run_vo.py`, `scripts/measure_testset.py`); the new tests in all three languages; the golden record and its generator; `viewer/src/frames.js`; and the design, ladder and test-set note changes.
- **References:** ORB-SLAM3's `Tracking.cc` (UZ-SLAMLab/ORB_SLAM3 master, the copy fetched for Run 1) and evo's `file_interface.py`.
- **Scratch probes:** in `/tmp/scratch/a2/panel2/`: `probe_scoring.py` and `golden_compare.py`, with the base record in `room_runs_base.json`.
- **Tests run at cb377fd:** `tests/eval/test_runs.py`, `tests/eval/test_metrics.py` and `tests/edge/test_pipeline.py`, 146 passed; `tests/orchestration/test_vo.py -k "golden_record or lost_track"` on every backend, 61 passed. The working tree is unchanged.

**Summary:** Every fix in my lane resolves its finding as the owner decided, and none brings a numerical or cross-language regression:
- L1/H1/A1: an invalid `rpe_delta` is refused, and no `except` remains.
- A2: "no map can be aligned" is now `Degenerate`, with each map's cause named.
- L2/A6: `# map <n>` lines appear in all three writers, at the same lines.
- L3: the reset boundary is tested in all three languages.
- L4: a map with no points, and a third map, are tested.
- A8: `_close` is alike in all three languages.

One new inaccuracy came in through cb377fd, and it is my own Run 1 wording, copied: design §5 misstates when ORB-SLAM3 gives a small map its relocalisation grace. The other three items are notes.

### Findings

#### [should-fix] Design §5's account of ORB-SLAM3's grace before LOST is only half right (my Run 1 note L5's wording)
- **Where:** `docs/design/2026-10-12-phone-robust-vo.md:118-121` ("it sends a map of 10 keyframes or fewer straight to LOST on its first failed frame, and gives a larger one up to 3 s of relocalisation by bag of words").
- **Issue:** In visual-only ORB-SLAM3, map size decides only when the **frame-to-frame step** fails (`TrackWithMotionModel`, then `TrackReferenceKeyFrame`): a map of more than 10 keyframes goes to `RECENTLY_LOST`, a smaller one to `LOST`. When that step succeeds but `TrackLocalMap` fails, the state becomes `RECENTLY_LOST` whatever the map's size. So a small map also gets up to 3 s of `Relocalization()` before it goes LOST and is reset.
  - The 3 s holds for visual-only sensors. Inertial ones get `time_recently_lost` (5 s) of IMU prediction instead.
  - The lite pipeline counts both kinds of failure alike in `lost_frames`. That is the real difference to state, and 2B's relocalisation design will build on it.
- **Why it matters:** The commit's own title is "ORB-SLAM3's reset described right". As written, the sentence would mislead 2B's design of a `RECENTLY_LOST`-style state. I introduced the wording in Run 1 (L5), and the fix carried it over.
- **Evidence:** `Tracking.cc` (master):
  - 1959-1975: `if (!bOK) { ... else if(pCurrentMap->KeyFramesInMap()>10) { mState = RECENTLY_LOST; ... } else { mState = LOST; } }`, the frame-to-frame branch.
  - 2143-2158: `if(bOK) mState = OK; else if (mState == OK) { ... mState=RECENTLY_LOST; // visual to lost }`. The local-map failure has no keyframe-count test.
  - 2006: `if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK) mState = LOST;`
  - 48 and 1993: `time_recently_lost(5.0)` for inertial sensors.
  - 2273: `if(pCurrentMap->KeyFramesInMap()<=10) { mpSystem->ResetActiveMap(); return; }`
  - Source: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc
- **Suggested fix:** A one-sentence docs edit, for example: "ORB-SLAM3 loses a map of 10 keyframes or fewer at once when its frame-to-frame tracking fails, and gives any other failure up to 3 s of bag-of-words relocalisation (visual-only) before LOST; the lite pipeline, without the bag of words, gives every map `lost_frames` (5) failed frames, of either kind, of PnP against its local map."

#### [note] The second blank stretch costs the monocular golden cases most of their second map's tracking
- **Where:** `tests/orchestration/datasets.py` (`LOST_AT = (12, 31)`); `tests/fixtures/edge/room_runs.json`.
- **Issue:** The golden record now has a third map in stereo and RGB-D, as decided. But the shared second stretch (frames 31-35) cuts monocular map 1 short:
  - At base it tracked 12 frames, pinned in both the lost and the kept cases.
  - Now it tracks 3 frames (kept case only), and the lost/monocular cases hold no poses at all.
  - Map 1's keyframes and points are unchanged (3 and 255). It made no keyframe after frame 29 at base either, so what is lost is 9 frames of monocular motion-model and local-map tracking in a second map, held across backends at `SOLVER_TOLERANCE`. Mapping coverage is not affected.
  - On the plus side, the zero-pose lost/monocular case now checks that every backend writes, and Python reads, a run whose maps were all reset.
- **Evidence:** `golden_compare.py`. Base: `lost/tum/monocular poses/map {1: 12}`, `kept/tum/monocular {0: 5, 1: 12}`. Head: `lost/tum/monocular {}`, `kept/tum/monocular {0: 5, 1: 3}`. EuRoC is the same.
- **Suggestion:** This is acceptable as is. If the room's `FRAMES` grows for another reason, move the second stretch later so that monocular map 1 tracks longer. Otherwise, add one line to `LOST_AT`'s docstring: monocular map 1 now tracks 3 frames.

#### [note] Out of lane (architecture): only Python's `write_run` checks the diagnostics' `map` keys
- **Where:** `src/slam_it/eval/runs.py` `_check_frame_maps` (called by `write_run` and `read_run`); `cpp/src/eval/runs.cpp` `check` (lines 250-258) and `rust/slam-it-eval/src/runs.rs` `check` (lines 477-485) check only `timestamp`.
- **Issue:** Before 367daa6 the three writers refused the same runs. Now C++ and Rust `write_run` accept a run whose `map` keys descend, are not integers, or omit a map that has keyframes, and Python's `write_run` refuses that run. Their docs still call each one "the mirror of `slam_it.eval.runs.write_run`". `read_run` catches such a run when it is read, so the failure stays observable. A3's resolution named only `write_run`/`read_run`, so this may be what was intended. Flagged for the architecture reviewer.

#### [note] The "centres on a line" message blames the map when the truth is degenerate
- **Where:** `src/slam_it/eval/runs.py` `_score_map` (`f"map {map_id}: its centres are on a line ({error})"`).
- **Issue:** Umeyama's test is on the cross-covariance's second singular value (`foundations/alignment.py:111`). It fires when either point set is collinear or coincident. A truth held still while the estimate moves therefore reports "map 0: its centres are on a line".
- **Evidence:** `probe_scoring.py`, with a static truth: `DegenerateError - no map can be aligned: map 0: its centres are on a line (umeyama: points are collinear or coincident); ...`
- **Suggestion:** "its centres, or the truth's, are on a line". This is cosmetic.

### Checked and fine
- **L1 (H1, A1):**
  - `rpe_delta` is checked once in `_scored`, before any map is scored. `-1`, `0`, `nan` and `inf` raise `rpe_delta: must be positive and finite`, and `True` is refused by `as_number`.
  - "No pair" is now an explicit `len(metrics.interval_pairs(...))` test, with nothing caught.
  - `interval_pairs` returns exactly `relative_error`'s pairs: probe at `delta` 0.05, 0.1 and 0.5 on 60 poses, `np.array_equal` True.
  - The pooled RPE and ATE equal the per-map errors pooled by hand exactly (`==` True).
- **No error the old `except` hid is now reachable on checked input:**
  - `associate` returns `ours` ascending, so the matched times stay strictly ascending.
  - The tolerance `0.5 * period` is finite and non-negative.
  - Whenever Umeyama does not raise, the Sim(3) scale is positive: `tr(DS) >= d0 + d1 - d2 > 0` once `d1 > 1e-10 d0`.
  - So the bake's narrowed `except DegenerateError` cannot newly fail a valid bundle.
- **A2:** `DegenerateError` with each map's cause, for a collinear estimate, a static truth, a run with no poses ("the run has no poses") and a 2-frame map. All four callers catch it (`bundles._score`, `run_vo.score`, `measure_testset`; the worker does not score).
- **L3:** The reset boundary pair `[1, 0]` is used on the one-keyframe map in Python, C++ and Rust, with the expected `reset` derived from `reset_keyframes > 0`. It catches a `<` for `<=` slip, and a slip the other way.
- **L4:**
  - The Python test runs a map of no points through PnP and the local map on empty arrays, loses it after `lost_frames` and resets it.
  - It then checks map 2's keyframe at identity and its run ids.
  - Its point-id check cannot distinguish a running total from the previous map's count, because map 0 has no points. The golden record does: its three RGB-D maps hold 852, 896 and 605 points, and every backend is held to their `point_lives` and `point_maps`.
- **Golden record:** Every lost and kept case differs from base only from frame 31 on (first summary difference at frame 31, first state difference at 35), so no decision before the second stretch changed. The losses at frames 16 and 35 name maps 0 and 1, and both are reset at `reset_keyframes` 10 and kept at 0.
- **A8:** `_close(m)` takes the map in all three languages, after the frame is dated (`_date`/`date()` first). The `LOST`-between-frames branch is gone in all three. The unreachable "lost with no map" case is now handled the same way everywhere: the close is skipped and the next frame initialises.
- **L2/A6:**
  - The `# map <n>` line goes before each map's first pose in all three writers. The C++ and Rust tests compare comment positions with the Python fixture, and Python's `read_trajectory` reads the file back.
  - evo's `csv_read_matrix` skips any line starting with `#`, anywhere in the file (https://github.com/MichaelGrupp/evo/blob/master/evo/tools/file_interface.py), so outside tools still read the file. As documented, it is up to the user to split it by map.
- **A5:** `Run.maps()` and `reset_maps()` mean what the code they replace computed. A reset map always has a keyframe removed as `reset`, because culling keeps keyframe 0.
- **A3:** `_check_frame_maps` agrees with what the pipelines write: the lost frame names the map it closed, and monocular frames before a start are null. It holds on every golden case.
- **H3 messages:** The first bad row and its value are reported by the same rule in all three `check_maps`.
- **`Evaluation.ate_rmse`:** It is the same pooled RMS as before; `__post_init__` guarantees one map.
- **Docs:**
  - Design §2's "10 keyframes or fewer" matches `Tracking.cc:2273`.
  - The L6 caveat cites Zhang & Scaramuzza, IROS 2018, correctly.
  - The L7 open item is worded correctly.
  - The two-view sample-limit item (200 against 10,000) matches `TwoViewReconstruction`.
- **Viewer:** `sharesFrame` is logically equivalent to the inline expression it replaced.
