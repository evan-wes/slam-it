# Review panel: `feature/2b-b3b-threads`

## Run 1 -- 2026-10-18

| | |
|---|---|
| **Branch** | `feature/2b-b3b-threads` |
| **Range** | `6b0a4136...afbbcc24` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | at `afbbcc24`: every check passed (pytest 4,853 passed, 1 skipped: the dashcam overlay's OCR test, the env having no Tesseract), except the viewer's two `tsc` runs and `prettier`, which failed because the worktree's `viewer/node_modules` was not installed; after `npm ci --prefix viewer` all three passed |

The owner's rule for this PR (2026-10-18): only must-fix findings are fixed; the rest are deferred
together. Later that day the owner added the cheap findings to this pass (H2, H4 to H9, H13's
fallback, A4 to A8), fixed in commits of their own; the rest stay deferred, as the resolutions
say. The reviewers were asked to verify two suspected issues found while reading the code for
another design (the view and the correction count read apart; a keyframe queued across a
correction): both are confirmed (A1/L2, A2/L1).

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `src/slam_it/edge/pipeline.py:733-743` | Python `MapOwner.close()` keeps tracking's pending counts, and the next map applies them | fix | fixed (89abab02) -- decided by evan-wes, 2026-10-18 |
| H2 | should-fix | hygiene | `tests/edge/test_threads.py:31-35` | Python handshake tests assert inside threads, where pytest does not fail on them | defer | fixed (1da6647b) -- decided by evan-wes, 2026-10-18 |
| H3 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:1294-1340` | Python's threaded loop correction, global BA and `follow` are untested | defer | deferred (the ladder's open items, Deferred from milestone 2B's B3b review panel) -- decided by evan-wes, 2026-10-18 |
| H4 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:1169-1171` | Python `idle()` can report idle while mapping holds a keyframe (with A7, L4) | defer | fixed (676446ec, with A7 and L4) -- decided by evan-wes, 2026-10-18 |
| H5 | should-fix | hygiene | `src/slam_it/edge/pipeline.py:1505-1510` | A failed threaded Python run leaves its worker threads running | defer | fixed (4f5ea6dd, with A8) -- decided by evan-wes, 2026-10-18 |
| H6 | should-fix | hygiene | `rust/slam-it-edge/src/threads.rs:437-447` | Rust `Worker` hangs `drain()` if a step panics (with A4) | defer | fixed (c9b48512, with A4) -- decided by evan-wes, 2026-10-18 |
| H7 | should-fix | hygiene | `scripts/measure_sharded.py:67-133` | Default workers changed, docs stale, a bad `--cores` blames `--jobs` | defer | fixed (2ccbe576) -- decided by evan-wes, 2026-10-18 |
| H8 | should-fix | hygiene | several | Stale, misplaced or incomplete documentation | defer | fixed (0c654d67) -- decided by evan-wes, 2026-10-18 |
| H9 | should-fix | hygiene | `mapping.need_keyframe`, `budgets.backlog`, `loops.propagate_global_ba` | New public arguments and functions are not validated | defer | fixed (41610b1d) -- decided by evan-wes, 2026-10-18 |
| H10 | note | hygiene | `tests/orchestration/test_loop_room.py` | A flaky gate test ships (owner-deferred) | note | deferred (design section 12's deferred list, with L3; the ladder's open items, Deferred from milestone 2B's B3b review panel) -- decided by evan-wes, 2026-10-18 |
| H11 | note | hygiene | the loop room | Python's threaded loop room is worse than in order, and unasserted | note | deferred (the ladder's open items, Deferred from milestone 2B's B3b review panel, with A10) -- decided by evan-wes, 2026-10-18 |
| H12 | note | hygiene | the three pipelines | `follow` reads `corrections` after the view (with A1, L2) | with A1 | fixed (1fded9b3, with A1) -- decided by evan-wes, 2026-10-18 |
| H13 | note | hygiene | `src/slam_it/edge/pipeline.py` | File size and fallbacks | note | fixed in part (8149defc: the whole-map fallback asserted against in debug builds); the file split deferred (the ladder's open items, Deferred from milestone 2B's B3b review panel, with A9) -- decided by evan-wes, 2026-10-18 |
| A1 | must-fix | architecture | `pipeline.py:1586-1590`, `pipeline.cpp:1429-1432`, `pipeline.rs:2572-2577` | Tracking reads the view and the correction count apart, and the view twice a frame (L2: should-fix) | fix: the view stamped with its count, read once a frame | fixed (1fded9b3) -- decided by evan-wes, 2026-10-18 |
| A2 | must-fix | architecture | `pipeline.py:1244-1258`, `pipeline.cpp:238-260`, `pipeline.rs:1759-1797` | A keyframe queued across a correction is inserted at its uncorrected pose (same as L1) | fix: re-placed by its reference keyframe | fixed (0dd5bfbc: re-placed by its reference keyframe; design section 12) -- decided by evan-wes, 2026-10-18 |
| A3 | should-fix | architecture | `cpp/tests/edge/test_pipeline.cpp`, `scripts/check.sh:92-105` | ThreadSanitizer never runs a correction, a global BA or relocalisation | defer | fixed (979fc85c: one threaded loop case under ThreadSanitizer, both C++ schemes) -- decided by evan-wes, 2026-10-18 |
| A4 | should-fix | architecture | `threads.rs:435-447`, `pipeline.rs:2157-2163` | Rust: a panicking step hangs the run, a panicking global BA is swallowed (with H6) | defer | fixed (c9b48512, with H6) -- decided by evan-wes, 2026-10-18 |
| A5 | should-fix | architecture | design section 12, item 6 | A lost map: the design says tracking moves on at once, the code waits | defer | fixed (c8db65a1: section 12, item 6 says what the code does) -- decided by evan-wes, 2026-10-18 |
| A6 | should-fix | architecture | `src/slam_it/edge/__init__.py:36-41` | The edge rung's compute budget still describes B3b as to come | defer | fixed (6c82be5c) -- decided by evan-wes, 2026-10-18 |
| A7 | note | architecture | `src/slam_it/edge/pipeline.py:1169-1171` | Python's idle test has a window the native ones don't (with H4) | note | fixed (676446ec, with H4) -- decided by evan-wes, 2026-10-18 |
| A8 | note | architecture | `src/slam_it/edge/threads.py:244-245` | Python's threads end only through `finish()` | note | fixed (4f5ea6dd, with H5) -- decided by evan-wes, 2026-10-18 |
| A9 | note | architecture | the pipeline modules | The map's owner is growing inside the pipeline modules | note | deferred (the ladder's open items, Deferred from milestone 2B's B3b review panel, with H13's file split) -- decided by evan-wes, 2026-10-18 |
| A10 | note | architecture | the loop room | Python's threaded run of the loop room closes no loop (in memory, `tests/edge/support.py`'s options; H11's run through `vo.run` closed one) | note | deferred (the ladder's open items, Deferred from milestone 2B's B3b review panel, with H11) -- decided by evan-wes, 2026-10-18 |
| L1 | must-fix | algorithm | `pipeline.py:1642-1645`, `pipeline.cpp:1408-1411`, `pipeline.rs:2658-2667` | A keyframe in flight across a correction enters the corrected map at its uncorrected pose (same as A2) | fix: re-placed by its reference keyframe | fixed (0dd5bfbc, with A2) -- decided by evan-wes, 2026-10-18 |
| L2 | should-fix | algorithm | the three pipelines | Tracking reads the view twice and the correction count apart (same as A1) | fix with A1 | fixed (1fded9b3, with A1) -- decided by evan-wes, 2026-10-18 |
| L3 | note | algorithm | `cpp/src/edge/loops.cpp:1499-1562` | The flaky threaded loop-room case is neither issue: a global BA that shrank the map | note (the deferred investigation) | deferred (design section 12's deferred list: the flaky case's investigation, with this finding's pointer) -- decided by evan-wes, 2026-10-18 |
| L4 | note | algorithm | `src/slam_it/edge/pipeline.py:1169-1171` | Python's `idle()` misses a keyframe taken but not yet flagged busy (with H4) | note | fixed (676446ec, with H4) -- decided by evan-wes, 2026-10-18 |
| L5 | note | algorithm | -- | Field context | note | noted -- decided by evan-wes, 2026-10-18 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope.** `git diff 6b0a4136...afbbcc24` (B3b, 114 files), read in context. The new threading code was read in full in all three languages: `src/slam_it/edge/threads.py`, `pipeline.py` (including `MapOwner`, `_Threads`, `Pipeline`), `atlas.py`, the `mapping`, `adjust`, `loops` and `covisibility` additions, `cpp/include/slam_it/edge/threads.hpp`, `pipeline.hpp`, `cpp/src/edge/pipeline.cpp`, `rust/slam-it-edge/src/threads.rs` and `pipeline.rs`. Also read: the run program changes (`orchestration/vo.py`, `vo.cpp`, `vo.rs`), the run format and budget judge (`eval/runs.py`, `eval/budgets.py`), the two measure scripts, the bindings, the options, and the new and changed tests in Python, C++ and Rust. Four checks were run outside the repo: a reproduction of the stale-counts defect, a mutation of `Writer.stopped()` against its test, a count of which threaded code paths the Python threaded tests reach, and Python threaded and in-order runs of the loop room.

**Summary.** The code is well documented and consistent across the three languages, and the C++ and Rust handshake tests are properly ordered with flags and events. There is one real defect: the Python threaded `MapOwner` keeps tracking's pending counts across a lost map and applies them to the next map. The rest is test strength and robustness in the Python reference, a few stale or misplaced docs, and one hang risk in the Rust worker. Per the owner's rule severity was graded strictly, so everything except the first finding is should-fix or note.

### Findings

#### [must-fix] Python `MapOwner.close()` keeps tracking's pending counts, and the next map applies them
- **Where:** `src/slam_it/edge/pipeline.py:733-743` (`close`), with `_counts` at `:623`, `count` at `:665-682` and `apply_counts` at `:684-690`. The C++ and Rust twins clear them: `cpp/src/edge/pipeline.cpp:1113-1114`, `rust/slam-it-edge/src/pipeline.rs:1054`.
- **Issue:** Threaded, tracking's counts of points seen and found are queued in `_counts` until the next mapping step applies them. `close()` drops the map, view and database but not `_counts`. A map's point ids restart at 0 (`maps.empty`, `next_pt_id=0`), so after a lost track the first mapping step of the new map adds the closed map's counts to the new map's points of the same ids. Tracking's counts for the last frames before the loss are almost always still queued, because they accumulate between keyframes.
- **Why it matters:** `pt_visible` and `pt_found` are wrong for the new map's first points. Culling uses their ratio, so good points can be culled or bad ones kept. The error is silent and hits every lost-then-restarted map in a threaded Python run. C++ and Rust do it correctly, so this also breaks the three languages' parity of behaviour.
- **Evidence:** run against the branch:
  ```python
  o = MapOwner(None, Writer()); o.start(m)             # m: the room's first RGB-D map
  o.count(m.pt_id[:3], m.pt_id[:2]); o.close()         # tracking's counts for the old map, then the loss
  o.start(m); before = o.map.pt_visible[:3]            # the next map, ids from 0
  o.apply_counts(); o.map.pt_visible[:3]               # -> before [1 1 1], after [2 2 2]
  ```
  `len(o._counts)` is 1 after `close()`. `test_a_lost_map_settles_the_threads_then_closes` (`tests/edge/test_pipeline_threads.py:180`) compares states and maps only, so it cannot see this.
- **Suggested fix:** In `close()`, clear `_counts` under `_counting`, as C++ and Rust do. Add a test that counts, closes, restarts and applies, and asserts the new map's counts are unchanged.

#### [should-fix] Python handshake tests assert inside threads, where pytest does not fail on them
- **Where:** `tests/edge/test_threads.py:31-35`, the same pattern at `:37-42` and `:57-68`.
- **Issue:** The core assertions of `test_a_stop_waits_for_mapping_to_finish_and_aborts_its_ba` (`assert writer.abort.wait(WAIT)`, `assert not stopped.is_set()`, the "stop waited for mapping" claim) run in `map_one` and `stop`. An exception in a thread does not fail the test, and `pyproject.toml` has no `filterwarnings = error`. The main thread checks only `stopped.is_set()` and `not writer.stopping()`. The C++ and Rust tests store flags and assert on the main thread, and Rust's `thread::scope` re-raises panics, so only Python is affected.
- **Why it matters:** The test cannot detect the handshake it is named for failing.
- **Evidence:** with `threads.py` copied and the wait in `Writer.stopped()` removed (`if not holding: self._take()`), the unchanged test against the copy gives `8 passed, 1 warning` (`PytestUnhandledThreadExceptionWarning ... AssertionError: assert not True`).
- **Suggested fix:** Have each thread store its booleans, as the C++ test does, and assert them after `join`; or wrap thread bodies in a helper that records exceptions and re-raises on join.

#### [should-fix] Python threaded loop correction, global BA and `follow` after a correction are untested
- **Where:** `src/slam_it/edge/pipeline.py:1294-1305` (the correction branch of `_Threads._close_loops`), `:1317-1340` (`_global_ba`), `:549-569` (`TrackingState.follow`); tests `tests/edge/test_pipeline_threads.py` and `tests/orchestration/test_loop_room.py` (whose docstring says "Python's threaded run is measured, not tested").
- **Issue:** The threaded Python tests use the sideways room, which has no loop.
- **Evidence:** wrapping the functions and running `tests/edge/test_pipeline_threads.py`: 9 passed, with `_close_loops` called 2 times, `loops.detect` 14, and `loops.correct`, `_global_ba` and `loops.propagate_global_ba` 0 each.
- **Why it matters:** The path where the Python reference has the most thread interaction (`stopped()` nested in `reading()`, `Generation`, `atlas.record` after a correction) is exercised by nothing. A hand run of the Python threaded loop room: 280 of 280 frames tracked, 1 loop, global BA applied, all threads gone at the end, no exception; so it works today, but nothing guards it. The final ATE was 7.38 cm against 4.67 cm in order (note below).
- **Suggested fix:** One Python test that runs the loop room (or a smaller loop) threaded and asserts a loop is closed, a global BA is applied and no thread is left.

#### [should-fix] Python `_Threads.idle()` can report idle while mapping holds a keyframe
- **Where:** `src/slam_it/edge/pipeline.py:1169-1171` and `:1244-1246`. C++ uses `mapping_.unfinished() == 0` (`cpp/src/edge/pipeline.cpp:138`), Rust `unfinished()` (`pipeline.rs:2076`).
- **Issue:** `idle` is `not self._busy and queue.waiting() == 0`. `Worker._run` takes the item (waiting becomes 0) before `_map` sets `_busy = True`, so there is a window where mapping holds a keyframe but reads as idle; tracking then takes a keyframe at once instead of interrupting the BA, queuing or refusing. A monocular map can get a keyframe queued that should have been refused.
- **Why it matters:** A rare scheduling difference from C++ and Rust, in the reference whose job is to be the same design.
- **Suggested fix:** Use `queue.unfinished_tasks` (as the native cores use `unfinished`) and drop `_busy`.

#### [should-fix] A failed threaded Python run leaves its worker threads running
- **Where:** `src/slam_it/edge/pipeline.py:1505-1510` (`finish`), `src/slam_it/orchestration/vo.py:810` (`tracker.finish()`, not in a `finally`).
- **Issue:** If `settle(apply=True)` raises a thread's error, `self._threads.stop()` is never reached, and `_finished` is already True, so a second `finish()` is a no-op. The C++ and Rust `finish()` always stop the threads. In `_run_here`, an error from `track` skips `finish()` altogether.
- **Why it matters:** Each failed run leaks two daemon threads blocked on their queues: a cost for a long-lived caller such as the upload worker or a test session.
- **Suggested fix:** try/finally around `settle` in `finish()`, and a try/finally or context manager around the loop in `_run_here`.

#### [should-fix] Rust `Worker` hangs `drain()` if a step panics
- **Where:** `rust/slam-it-edge/src/threads.rs:437-447` (the worker loop), docs at `:400-403` and `:511-513`.
- **Issue:** `Worker`'s doc promises that a failing step drops later items "so that draining never hangs". If `step` panics, the thread unwinds without calling `queue.done()` for that item, so `unfinished` never reaches 0, and `settle()` (on a lost track and at `finish()`) waits in `drain()` forever. The `stop()` comment says the panic "has already reported it", but nothing reports it to the waiting thread.
- **Why it matters:** An indexing bug in a worker becomes a silent hang of the native program, not an error. The workspace does not set `panic = "abort"`.
- **Suggested fix:** Catch the panic in the worker loop (`catch_unwind`), or a drop guard that calls `done()` and records an error such as "step panicked".

#### [should-fix] `measure_sharded.py`: the default workers changed, the docs are stale, and a bad `--cores` blames `--jobs`
- **Where:** `scripts/measure_sharded.py:67-76` (`groups_of`, no Parameters or Returns), `:132-133`, and the module docstring ("`--jobs` is the workers, at most the CPUs ... by default, all of them").
- **Issue:** `main` now always goes through `budgets.physical`, so the default (`--cores 1`) uses one logical CPU per physical core, not every CPU as before; the docstring still says "all of them" and the changelog does not say so. `wanted = jobs * cores` is passed to `cpus_for`, which reports `--jobs: expected 1 to N, got <wanted>` (e.g. `--jobs 8 --cores 4` on 24 cores prints "got 32"; `--cores 0` prints "got 0" under `--jobs`).
- **Why it matters:** The error names a value the user did not type, and the tool silently runs fewer workers than before.
- **Suggested fix:** Validate `--cores` first, name the product in the message (`--jobs x --cores`), update the docstring and changelog, give `groups_of` full numpy sections.

#### [should-fix] Stale, misplaced or incomplete documentation
- `src/slam_it/edge/mapping.py:30`: "Only the Python reference takes these so far, until B3b's compiled ports." The C++ and Rust cores take `Interrupts` in this PR; only the Python bindings do not expose them. The refusal messages (`mapping.py:1365` "only the Python reference is interrupted so far", `adjust.py:388`) say "so far" the same way. Say that the bindings do not cross them, with no "so far".
- `cpp/tests/edge/test_pipeline.cpp:365-369`: `run_tests` lost its comment; the stranded `@brief The test body. @return The exit code.` sits on `asking()`'s block, giving `asking()` two `@brief`s, and `run_tests` (`:565`) is undocumented.
- `rust/slam-it-py/src/edge/mapping.rs:21-31`: the `need_keyframe` rustdoc lists `idle` and `since_keyframe` but not the new `keyframes` argument (the C++ binding documents it).
- `tests/edge/test_options.py:115`: the docstring says "lists all 132"; the assertion is 134.
- `src/slam_it/edge/atlas.py:34` says "Every method holds the atlas's lock"; the properties `frames`, `frame`, `map_id`, `keyframe_base`, `point_base` and `run_ids` read unlocked (safe only under the GIL), unlike C++, where each getter locks.
- `src/slam_it/edge/mapping.py:503-505` (`need_keyframe` Raises): "the depths are infinite, the matches are not one per depth" lost its "or".
- Suggested fix: edit each in place; none changes behaviour.

#### [should-fix] New public arguments and functions are not validated
- **Where:** `mapping.need_keyframe` (`since_keyframe`, `keyframes`, `idle`), `budgets.backlog` (`src/slam_it/eval/budgets.py`), `loops.propagate_global_ba` (`src/slam_it/edge/loops.py`), and the `MapOwner.count`/`apply_counts` duplication.
- **Issue:** `need_keyframe` passes a negative or boolean count straight to the compiled stage. `backlog` is public but does not check that `times` and `seconds` have the same length, are finite and ascending; a length mismatch raises a bare `ValueError` from `zip(strict=True)`. `propagate_global_ba` does not check that `solved` came from `now`.
- **Why it matters:** CLAUDE.md asks for validation at public boundaries with messages naming the bad value; today only `judge` calls `backlog`, with checked data.
- **Suggested fix:** Check the arguments with `InvalidInputError` at the Python boundary. Also, `MapOwner.count` and `apply_counts` (`pipeline.py:681-682`, `:689-690`) write the same two `maps.count` calls twice; a small `_counted(m, visible, found)` helper removes the duplicate.

#### [note] A flaky gate test ships (owner-deferred)
`test_every_backend_closes_the_loop_room` (C++ threaded, TUM monocular) missed its 2 cm bound at 8.99 cm in one of four runs, as the design doc's "Deferred for this PR" records. A check in layer 1 and offline CI that fails at random is a hygiene cost; it is recorded as deferred, so not graded.

#### [note] Python threaded loop room is worse than in order, and unasserted (for the algorithm reviewer)
Single quiet-machine runs of the RGB-D loop room, Python:

| Run | Final ATE | ATE as tracked |
|---|---|---|
| In order | 4.67 cm | 8.27 cm |
| Threaded | 7.38 cm | 19.95 cm |

Both tracked 280 of 280 frames in 1 map with 1 loop. The threaded final ATE is above the 7 cm bound the other backends are held to, and nothing asserts it.

#### [note] `follow` reads `corrections` after the view (all three languages)
`pipeline.py` `_track`, `pipeline.cpp:1430-1431` and `pipeline.rs` `track_frame` read the view first and `corrections` second. A correction landing between the two reads makes the last frame follow the old view once; the window is nanoseconds. Reading `corrections` first would make it impossible, since the writers publish the view before they count it.

#### [note] File size and fallbacks
- `src/slam_it/edge/pipeline.py` is 1655 lines; `MapOwner` and `_Threads` could be modules of their own (for the architecture reviewer).
- `view_of` in Rust (`pipeline.rs:729-730`, `unwrap_or_else(... collect())`) and C++ (`rows ? sub_map : m`) fall back to copying the whole map if `view_keyframes` fails. Documented as "cannot fail", but a silent fallback would hide a bug; a debug assertion would be cheap.

### Checked and fine
- C++ `Writer`, `Queue`, `Generation` and `Worker` (`threads.hpp/.cpp`): lock order, RAII, nested stop inside a hold, an error that drops later items so draining cannot hang. Rust's `Writer`, `Queue` and `Generation` are the same design.
- The ordering of the C++ and Rust pipelines' locks and joins: C++ `kLocks` has no inversion (tracking never waits on the writer; `map()` takes `Mapping` before the map lock as `stopped()` takes `Stopped` before it). Rust's `held` mutex nests under `atlas`, `closing` and the view locks in one order only. Threads are stopped before they are joined, in both `Drop` impls.
- `Pipeline::finish()` in C++ and Rust stops the threads even if `settle` fails; `Pipeline` moves are safe because the shared state is on the heap.
- C++ `idle()` and Rust `idle()` use `unfinished()`. The `Atlas` is internally locked in C++ and Rust, and the Rust pipeline shares it behind a lock. The view is an immutable `shared_ptr`/`Arc` that tracking holds for its frame.
- C++ and Rust `MapOwner::close()` clear pending counts, and C++/Rust `apply_counts` skip points that are gone. The stop flags are atomics with relaxed loads, right for a flag.
- Native tests: ordered by events and flags, named wait constants (`kWait`, `kQuiet`), both threading schemes run, TSAN in `check.sh`, lost-map, stop-refuses-keyframes and view-equals-map tests in both natives. The `--views` golden check covers all 27 cases (verified: 27 in the fixture).
- Python `adjust`, `mapping`, `loops` and `covisibility` tests for abort, interrupts, `view_keyframes`/`sub_map` and `propagate_global_ba` assert on values; `propagate_global_ba` is checked on every backend against a rigid-motion oracle.
- New options validated in all three languages and tested; the setting counts (132 to 134) agree.
- Run format 8 validation (`threaded`/`locks` as bools, `locks` implies `threaded`), the handshake keys, and `budgets.judge` checks for the new keys. Tests for `physical`, `backlog`, the budget judge and `groups_of`.
- No debug prints (only `measure_sharded.py`'s status line), no TODO/FIXME, no commented-out code, no hard-coded paths. The one swallowed fallback, `_core_of` for an unlisted CPU, is documented.
- CHANGELOG and VERSION agree with the code (checked: stop codes 7 and 3, 27 golden cases, run format 8, settings counts). The design doc's deferred items match what the code and tests show.

Nothing in the repository was changed; layer 1 was not re-run; scratch files used are outside the repo.

### Architecture review

**Scope.** `git diff 6b0a4136...afbbcc24` (114 files, B3b, v0.3.14.0), focused on the threaded pipeline in all three languages: `src/slam_it/edge/{pipeline,threads,atlas,mapping,covisibility,map,loops}.py`, `cpp/{include,src}/slam_it/edge/{pipeline,threads,atlas,mapping}`, `rust/slam-it-edge/src/{pipeline,threads,atlas}.rs`; also the rung 0 stop flag, the bindings, the import-linter and `check_layers` contracts, `scripts/check.sh`'s ThreadSanitizer step, the budget judge, run format 8, and the design doc's section 12. Both suspected issues were checked in all three languages and in both C++ schemes. Probes outside the repository ran Python's threaded pipeline on the loop room (all three modes), and the C++ and Rust programs threaded on it (3 cases × 3 runs each).

**Summary.** The layering is sound: `atlas` and `threads` sit correctly in the edge rung's contract, rung 0 gains only a generic stop flag, every new crossing has its Python reference, Protocol and thin binding, and mapping is interrupted through an interface rather than by importing threads. Both suspected issues are real, in all three languages, and both are must-fix: the correction-follow race (tracking reads the view and the correction count apart; rare, bounded effect), and stale queued keyframes (a keyframe queued across a correction is inserted at its uncorrected pose; certain whenever a keyframe waits for mapping at a correction; never shows on the loop room, where mapping keeps up). The ThreadSanitizer layer never runs a correction or a global BA, which is how both slipped through.

**The two suspected issues, verified.** (1) Tracking reads the view and the count separately: confirmed in Python, C++ (our design, `kView`) and Rust; the C++ baseline `kLocks` is immune, because tracking holds the map's lock through its frame; worse than suspected, since the view is also read twice per frame (first finding). (2) A queued keyframe is mapped from its uncorrected pose: confirmed in all three languages and both C++ schemes; ORB-SLAM avoids it in two ways, not one (second finding).

### Findings

#### [must-fix] Tracking reads the view and the correction count apart, and reads the view twice a frame
- **Where:** reads `src/slam_it/edge/pipeline.py:1586-1590` and `:1003`; `cpp/src/edge/pipeline.cpp:1429-1432` and `:750`; `rust/slam-it-edge/src/pipeline.rs:2572-2577` and `:1411`. Writes `src/slam_it/edge/pipeline.py:714-715`; `cpp/src/edge/pipeline.cpp:1070-1076` (with `:1118-1125`); `rust/slam-it-edge/src/pipeline.rs:958-961` (with `:1069-1076`).
- **Issue:** Each frame reads the published view and `corrections` separately and passes them to `follow`; tracking's step (`_track_frame`, `track_step`) then reads the view a second time. Every writer publishes the corrected view first and counts the correction after (C++ `corrected()`: `changed()` publishes, then `++corrections_`; Rust `stopped()`: `self.changed(&mut held)?`, then `fetch_add`; Python `apply_correction`: `self._view = m`, then `self.corrections += 1`). Three interleavings lose the move: old view, new count (`follow` uses up the new count against the old view, nothing moves, and the frame is tracked on the new view); new view, old count (the view is read between publish and count, `follow` sees no change, the frame is tracked and re-anchored on the new view, so the next frame's `follow` moves nothing either); a correction between `follow` and the second read. In each case the frame after a correction is predicted from the last frame's uncorrected pose against corrected points.
- **Why it matters:** The code and docs promise the opposite: section 10, item 8 (as built) and the pipeline docstrings say the last frame moves with its keyframe after a correction; `cpp/include/slam_it/edge/pipeline.hpp:7-10` and `CHANGELOG.md:31` say tracking reads "the view it took for its frame". When the race hits, the motion model's search misses by the size of the loop correction; the frame falls back to local PnP or relocalisation, and a recovery holds back keyframes for `tracking.max_frames` frames, or the frame goes untracked. The window is microseconds per frame and opens once per correction: rare, but a race in the handshake, and no test exercises it.
- **Evidence:** Python `_track`: `view = self.owner.view` … `self.tracking.follow(view, self.owner.corrections)`, then `_track_frame`: `view = owner.view` (`pipeline.py:1003`). C++ `track_frame`: `if (const View view = owner.view(); view.map) { follow(tracking_, *view.map, owner.corrections()); }`, then `track_step`: `View view = owner.view();` (`:750`). C++ `MapOwner::corrected()`: `if (auto ok = changed(); !ok) {…} ++corrections_;`.
- **Suggested fix:** Stamp a published view with the correction count inside the writer's turn, where the count is stable (a field on C++'s and Rust's `View`; in Python a (map, count) pair assigned at once). Read the view once per frame and pass it into tracking's step; call `follow(view, view.corrections)`. Add a native test that holds a stop, applies a correction, then tracks a frame.

#### [must-fix] A keyframe queued across a correction is inserted at its uncorrected pose
- **Where:** Python `src/slam_it/edge/pipeline.py:1244-1258` (`_Threads._map`) and `:1111-1120` (`_still_mapped` fixes only the points); C++ `cpp/src/edge/pipeline.cpp:238-260` (`PipelineThreads::map`); Rust `rust/slam-it-edge/src/pipeline.rs:1759-1797` (`map_keyframe`); the insertion `src/slam_it/edge/mapping.py:1263` and `:1268`; the stop `src/slam_it/edge/threads.py:71-98`, `cpp/src/edge/threads.cpp:38-58`, `rust/slam-it-edge/src/threads.rs:145-160`.
- **Issue:** Our stop is granted between any two keyframes: `stopped()` waits only for mapping's current turn to end, and mapping's next turn waits until the stop is released. So a keyframe waiting for mapping when loop closing corrects is inserted afterwards; the same when a global BA's result is carried onto the map. It goes in at the pose tracking gave it on the pre-correction view: `insert_keyframe(m, frame, tracked.pose, tracked.points)`. Keyframes get stuck there two ways: queued while mapping was busy (stereo and RGB-D queue up to `mapping.max_queued`), or asked during the search (loop closing holds the map while it searches; a hold does not stop mapping, so `stopping()` stays false and tracking keeps asking for keyframes).
- **Why it matters:** The keyframe sits off the corrected map by the loop's correction (a Sim(3) in monocular). With depth, its new points are made from that pose (`mapping.py:1268`); triangulation pairs it with corrected neighbours through the wrong pose; any repair is left to a local BA, which the scheduling skips while keyframes wait and aborts when one arrives; `propagate_global_ba` keeps the offset (a keyframe made since moves with its parent, its relative pose kept, `src/slam_it/edge/loops.py:1539`); the keyframe's own frame takes the keyframe's final pose. ORB-SLAM never does this: `LocalMapping::Run` reaches `Stop()` only in the `else` branch of `CheckNewKeyFrames()`, so mapping stops only with its queue empty; ORB-SLAM3's `CorrectLoop` also calls `EmptyQueue()` (`ProcessNewKeyFrame` on each), so queued keyframes are corrected with the current keyframe's neighbours; `LocalMapping::Release` deletes any keyframe still queued. (From this reviewer's reading of ORB-SLAM's source; the algorithm reviewer can confirm.) Section 12, item 3 mentions ORB-SLAM3's queue-emptying only as a slip of ORB-SLAM's; what our design does with the queue at a correction is not designed.
- **Evidence:** The code path is deterministic: no language changes a request's pose. Frequency (probes on this HEAD, 2026-10-18, machine load about 3 on 24 CPUs): loop room, C++ and Rust, threaded, 18 runs: loop closing held the map for 15-39 ms per loop and no keyframe was asked inside the hold (an estimate from each loop's `loop_seconds` and its global BA's end, assuming frames were fed on time); the room's bar cannot see this defect, and the probe gives no support for linking it to the deferred flaky monocular case. Python, threaded, same room: no loop closed in any mode, but mapping was behind: keyframes were already queued on 37 RGB-D and 50 stereo frames that asked for one. Inference: where mapping falls behind, keyframes wait in the queue most of the time, so a correction catches them; B2 measured mapping's load at 1.74-2.63 on the test set's five largest maps (section 9); section 13's runs were not made.
- **Suggested fix:** A design fork, to decide with the owner and record in section 12; both options cover the global BA case. ORB-SLAM's way: under the stop, before `loops.correct`, insert the queued keyframes (insertion and their links only, as `EmptyQueue`'s `ProcessNewKeyFrame` does), so the correction moves them with their neighbours; mapping finishes them after the release. Lighter: each request carries its reference keyframe's id, its pose relative to it, and the correction count when tracked (what `TrackingState.follow` already keeps); mapping re-places the request by its reference keyframe when the count has moved. Fix it before section 13's comparison: the `kLocks` baseline inherits the defect.

#### [should-fix] ThreadSanitizer never runs a correction, a global BA or relocalisation
- **Where:** `cpp/tests/edge/test_pipeline.cpp:432-433`, `:536-538` and `:574-577` (every threaded case passes `std::nullopt` as the vocabulary, so loops and relocalisation are off); `scripts/check.sh:92-105`.
- **Issue:** Under TSAN, loop closing returns at once. None of these run under TSAN: the stop inside a hold, and `corrected()`; the generation's supersede and the global BA thread; `propagate_global_ba` under the stop; relocalisation's hold, and `follow`. The loop room's threaded bar runs these paths, but only in the normal build; `cpp/tests/edge/test_threads.cpp` tests the handshakes on their own, not combined.
- **Why it matters:** Section 12, item 13 makes TSAN the race check for the handshakes; the paths where two writers meet are exactly the ones it skips, and both findings above live there.
- **Suggested fix:** Run one threaded case with loops on under TSAN, in both schemes: the loop room, or a short loop path.

#### [should-fix] Rust: a panicking step hangs the run, and a panicking global BA is swallowed
- **Where:** `rust/slam-it-edge/src/threads.rs:435-447` (the worker loop never reaches `queue.done()` after a panic); `rust/slam-it-edge/src/pipeline.rs:2157-2163` (`let _ = thread.join();`).
- **Issue:** Hang: a panic in mapping's or loop closing's step ends the thread without `done()`, so `drain()` in `settle` or `finish` waits forever. Swallowed: a panicking global BA leaves no error, and the run finishes as if it had run. Python's worker catches and keeps every exception; C++ terminates.
- **Why it matters:** Only alongside another bug (the cores should not panic), but then the program hangs (the measurement scripts would wait on it) or reports success; the error contract differs across the languages.
- **Suggested fix:** Wrap the step in `catch_unwind` and keep the result as an `Error`, or a guard that calls `done()` on unwind; turn a join's `Err` into `steps.keep(...)`.

#### [should-fix] A lost map: the design says tracking moves on at once, the code waits
- **Where:** `docs/design/2026-10-14-online-slam.md:701-704` (item 6); `src/slam_it/edge/pipeline.py:1462-1465`; `cpp/src/edge/pipeline.cpp:1227-1233`; `rust/slam-it-edge/src/pipeline.rs:2359-2364`.
- **Issue:** Item 6 says "tracking closes it by a message to mapping … and tracking starts the next map at once". All three languages instead settle in tracking's thread: drain mapping and loop closing, supersede and join any global BA, then close. The docstrings describe the code, not the design.
- **Why it matters:** The design doc and the code diverge, and the lost frame's latency includes the drain.
- **Suggested fix:** Change the doc (recommended): the code's way is simpler and safe, and its cost is recorded in `wait_seconds`. Say so in item 6.

#### [should-fix] The edge rung's compute budget still describes B3b as still to come
- **Where:** `src/slam_it/edge/__init__.py:36-41`.
- **Issue:** It says "B3's design gives tracking, mapping and loop closing threads of their own in B3b", in the future tense, and does not declare what a threaded run uses: up to four threads (tracking, mapping, loop closing, a global BA) against `laptop-cpu`'s 4 physical cores.
- **Why it matters:** `CLAUDE.md` requires each package docstring to state its compute budget, and the budget harness relies on it.
- **Suggested fix:** Rewrite that paragraph.

#### [note] Python's idle test has a window the native ones don't
`src/slam_it/edge/pipeline.py:1169-1171`, `:1246`. `idle()` checks `_busy`, which is set inside the step only after `queue.get()` returns; in between, a keyframe already taken reads as idle, so a monocular keyframe can be queued behind it. C++ and Rust count the item in hand under the queue's lock (`unfinished()`); Python's `queue.Queue.unfinished_tasks` would give the same.

#### [note] Python's threads end only through `finish()`
`src/slam_it/edge/threads.py:244-245`. They are daemon threads, so a threaded `Pipeline` dropped without `finish()` leaves two threads blocked. C++ (`~PipelineThreads`) and Rust (`Drop`) join their threads on destruction.

#### [note] The map's owner is growing inside the pipeline modules
`src/slam_it/edge/pipeline.py:597-800`, `cpp/include/slam_it/edge/pipeline.hpp:195-590`, `rust/slam-it-edge/src/pipeline.rs:576-1153` (`pipeline.rs` is 2,738 lines). `MapOwner`, its `View` and the publishing are the design's "one owner", a concept of its own; B4's merges enter through it. The atlas was extracted for the same reason; an `edge` owner module before B4 would keep the pipeline as the composition.

#### [note] Python's threaded run of the loop room closes no loop
Probe conditions: in memory, `tests/edge/support.py`'s `CAMERA` and `OPTIONS` (not run through `vo.run`). RGB-D: 263 of 280 frames tracked; stereo: 255 of 280; monocular: 10 maps, 189 frames tracked. Item 13 makes it measured, not tested, so this is not a defect; but the readable reference of the threaded design never runs a correction under threads.

### Checked and fine
- **Dependency direction:** `(atlas)` and `(threads)` are placed in the edge rung's exhaustive layers contract; `threads` sits at the bottom and imports nothing of the rung. `scripts/check_layers.py` derives C++ and Rust's internal order from that same contract. `threads.hpp` includes only `foundations/error.hpp`; `threads.rs` uses only `slam_it_foundations`. No new import-linter contract is needed.
- **Rung 0:** the stop flag (nls and graphs; C++ `const std::atomic<bool>*`, Rust `Stop`, Python a callable) is a generic solver concept, like g2o's `setForceStopFlag`. It brings a new termination code in each language, and a compiled backend refuses a Python `stop`.
- **Language split:** `view_keyframes`, `sub_map` and `propagate_global_ba`, plus mapping's schedule and `need_keyframe`'s new arguments, all have Python references, Protocol entries and thin one-file bindings. The threaded pipeline stays native-program-only, like the pipeline before it.
- **Algorithms stay unaware of threads:** mapping is interrupted through `mapping.Interrupts` (a Python Protocol, a C++ struct of callbacks, a Rust trait); no algorithm module imports threads or the pipeline.
- **Deadlocks and lock order:** the `mapping`, `held` and `stopped` handshakes, including a stop inside a hold, are sound in all three languages. Rust's lock order is consistent: `held` before `atlas`, `held` before `closing`, never reversed. In C++ `kLocks`, loop closing corrects only after its search has released the map's lock. The global BA's stop flag is kept alive by the `shared_ptr` captured in its thread.
- **Teardown:** C++ ends `PipelineThreads` before `Shared`, and each worker's thread is started last. Rust joins its threads in `Drop`, with the shared state behind `Arc`.
- **Errors from threads:** they are kept and raised in tracking's thread on the next frame (Rust panics aside, above). `settle` drains mapping before loop closing, so no keyframe is dropped between them.
- **Run format 8:** a clean break with no shims; `--locks` and `--views` are refused where they don't apply.
- **Budgets:** physical cores are read from sysfs through an explicit `topology` argument; Lindley's backlog applies only to runs made in order.
- **Inline default:** unchanged, and the records are untouched. `kInlineView` / `InlineView` checks that tracking on a view decides as it does on the map.

### Algorithm review

**Scope.** `git diff 6b0a4136343c97a86f302367527b8d9ab92ea7dd...afbbcc242fb630904e1a262b4ad8ffdaba13479b`. The threaded paths were read in full in all three languages: `src/slam_it/edge/pipeline.py`, `src/slam_it/edge/threads.py`, `src/slam_it/edge/mapping.py` (`_process`), `cpp/src/edge/pipeline.cpp`, `cpp/include/slam_it/edge/pipeline.hpp`, `cpp/include/slam_it/edge/threads.hpp`, `cpp/src/edge/loops.cpp` (global BA and its propagation), `cpp/src/foundations/graphs.cpp` (solve), `rust/slam-it-edge/src/pipeline.rs`, and section 12 of the design; compared with ORB-SLAM3's source (UZ-SLAMLab/ORB_SLAM3 master: `Tracking.cc`, `LocalMapping.cc`, `LoopClosing.cc`). Evidence: 33 threaded C++ monocular and 8 threaded C++ RGB-D runs of the loop room, with the tests' `OPTIONS`, fed at the room's rate, four at a time, on scratch copies outside the repository.

**Summary.** The handshakes (stop, release, abort, generations, the queue rules of `NeedNewKeyFrame`) follow ORB-SLAM3 faithfully. Both suspected issues are real, in all three languages, with different severity: issue 2 (must-fix), a keyframe in flight across a correction enters the corrected map at its uncorrected pose, which ORB-SLAM3 prevents with `EmptyQueue`, `Release` and `mMutexMapUpdate`; issue 1 (should-fix), the view and the correction count are read apart, a window of microseconds whose consequence is one or two frames predicted from a stale pose, with nothing in the map corrupted. Neither explains the flaky threaded loop-room case: in the repeats, the large failure coincides with a global BA that shrank the map about 3.6 times, and the 5 runs that had a keyframe in flight across the correction all passed.

### Findings

#### [must-fix] A keyframe in flight across a correction enters the corrected map at its uncorrected pose (suspected issue 2: confirmed)
- **Where:**
  - The request carries the pose as tracked: `src/slam_it/edge/pipeline.py:1642-1645` (`_hand_over`), `cpp/src/edge/pipeline.cpp:1408-1411`, `rust/slam-it-edge/src/pipeline.rs:2658-2667`.
  - Mapping inserts it unchanged: `src/slam_it/edge/pipeline.py:1252-1258` (`_Threads._map`), `cpp/src/edge/pipeline.cpp:257-260`, `rust/slam-it-edge/src/pipeline.rs:1786-1797`.
  - Where the pose is used: `src/slam_it/edge/mapping.py:1263` (`maps.insert_keyframe(m, frame, tracked.pose, tracked.points)`), then `:1268` (depth points made from that pose), `:1272` (triangulation) and `:1277` (published to tracking before local BA).
  - Corrections that neither drain nor drop the queue: `src/slam_it/edge/pipeline.py:1295-1300` and `:1326-1331`, `cpp/src/edge/pipeline.cpp:371-393` and `:422-437`, `rust/slam-it-edge/src/pipeline.rs:1926-1949` and `:1975-1985`. The C++ baseline (`kLocks`) shares the same paths.
- **Issue:** A keyframe request holds the pose tracked against its frame's view; nothing between the request and the insertion reconciles that pose with a loop correction or a global BA applied in between. Two ways: (a) the keyframe is queued, or taken by mapping and waiting for its turn, while loop closing searches and corrects with the map held; (b) a frame tracked on a pre-correction view asks for its keyframe after the stop is released (`stopping()` is then false, and tracking holds no lock across its frame). Either way, mapping inserts the keyframe into the corrected map at a pose in the old world frame: RGB-D and stereo depth points are created from that pose, new points are triangulated from it against corrected neighbours, and the map is published to tracking before local BA runs. Section 12 (`docs/design/2026-10-14-online-slam.md:668`) lists `CorrectLoop`'s `EmptyQueue` only as a slip in ORB-SLAM's sharing and drops it, with nothing in its place.
- **What ORB-SLAM3 does:** `CorrectLoop` calls `mpLocalMapper->RequestStop(); mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue`; `EmptyQueue` is `while(CheckNewKeyFrames()) ProcessNewKeyFrame();`, which inserts each queued keyframe and calls `UpdateConnections`, so the keyframes are in `mvpCurrentConnectedKFs` and the essential graph and are corrected with the map. `LocalMapping::Release()` ends with `for(...mlNewKeyFrames...) delete *lit; mlNewKeyFrames.clear();`: keyframes queued during the stop are dropped, after both `CorrectLoop` and `RunGlobalBundleAdjustment`. `Tracking::Track` holds `pCurrentMap->mMutexMapUpdate` through its frame, keyframe creation included, and `NeedNewKeyFrame` returns false while `mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()`. So no keyframe enters ORB-SLAM3's map in a stale world frame. The C++ baseline lacks `EmptyQueue` and the drop as well, so section 13's comparison is not against ORB-SLAM's scheme on this point.
- **Why it matters:** The keyframe's error equals the correction where it was tracked: on the room, measured from the jump in the as-tracked pose, 6-13 cm and 1.0-1.7° in monocular, 20 cm and 2.4° in RGB-D; on real loops tens of centimetres to metres, plus a scale change in monocular. Frames tracked with the keyframe as reference are placed relative to its wrong pose (`Atlas.place_frame` stores the relative pose) and keep that error once local BA moves the keyframe. When local BA is skipped (a stereo or RGB-D keyframe waiting, or a stop requested, e.g. the global BA being applied), the keyframe stays wrong. When the keyframe is far off, local BA's outlier pass after 5 iterations can discard its observations. This is a race that corrupts the map. **Caveat:** on the room, local BA absorbed the error every time; the must-fix rests on the broken invariant and real-data correction sizes, not on a measured failure.
- **Evidence:** In 5 of the 33 threaded monocular runs (16, 60, 68, 71, 75), the keyframe asked for on the frame after the loop's keyframe was mapped after the correction. Run 16: the as-tracked pose jumps 13.2 cm and 1.0° between frames 234 and 235, so frames 233 and 234 were tracked in the old world; keyframe 233's two published views (dated frame 235) hold 46 and 45 keyframes against 28 before the correction, so it was mapped after the correction; its local BA ran from cost 2015 to 781. All five passed, at 1.48-1.65 cm final ATE. The first local BA after a correction starts high in every run (1598-2808), so the cost alone does not single this keyframe out.
- **Suggested fix:** Stamp each request with its reference keyframe's id, `T_frame_ref`, and the correction count read with the view the frame used (next finding). At insertion, if the owner's count has moved, place the keyframe at `T_frame_ref · T_ref(now)` (ORB-SLAM's `UpdateLastFrame` rule, which `follow` already applies to the last frame); this covers loop corrections and global BA, and both paths (a) and (b); mirroring `EmptyQueue` plus a `Release`-style drop alone would miss (b). Add a handshake test in each language: queue a request, apply a correction, check the inserted pose moved with its reference.

#### [should-fix] Tracking reads the view twice and the correction count apart from both (suspected issue 1: confirmed; rare and bounded)
- **Where:** Python `src/slam_it/edge/pipeline.py:1588-1590` (view and count for `follow`) and `:1003` (view again for tracking); the writer sets the view, then counts (`:712-715`). C++ `cpp/src/edge/pipeline.cpp:1430-1431`, then `:750`; the writer publishes inside `changed()`, then `++corrections_` (`:1070-1076`, `:1118-1125`). Rust `rust/slam-it-edge/src/pipeline.rs:2573-2576`, then `:1411`; the writer `changed()`, then `fetch_add` (`:959-961`).
- **Issue:** The view for `follow`, the count, and the view for tracking are three separate reads, and the writers publish before they count. If a correction publishes between the two view reads, the last frame is not moved but the frame is tracked against the corrected view. If publish and increment both land before the count read, the correction is consumed against the old view and never followed; the same for one frame if the count is read between publish and increment. This contradicts the module docstring (`src/slam_it/edge/pipeline.py:72`: "the one it took for the whole frame"). C++ uses an atomic count and a mutex-guarded view, so this is a logical race, not a data race; TSAN cannot see it.
- **What ORB-SLAM3 does:** `Track()` takes `mMutexMapUpdate` and reads `GetMapChangeIndex`/`SetLastMapChange` under it; `UpdateLastFrame` re-places the last frame every frame (`mLastFrame.SetPose(Tlr * pRef->GetPose())`); `CorrectLoop` moves the poses and calls `IncreaseChangeIndex` under the same mutex.
- **Why it matters:** When it happens, one frame is predicted from its pre-correction pose against the corrected map; the velocity then spans the jump, so the next frame is off too. The prediction error is the correction size (6-20 cm, 1-2.4° on the room); the motion-model search may miss; local PnP and relocalisation do not use the pose, so tracking recovers, but then no keyframe is made for `tracking.max_frames` frames. The map is never corrupted. The window is microseconds in C++ and Rust, a GIL switch over a few bytecodes in Python. In all 33 runs, every frame after a correction was placed by the motion model.
- **Evidence:** the read and write orders at the lines above, and the ORB-SLAM3 code quoted.
- **Suggested fix:** Publish the count with the view (a `View` field, or a `(map, corrections)` pair swapped together); read the view once per frame and pass it to `track_step` / `_track_frame`. The previous finding's stamp needs the same count.

#### [note] The flaky threaded loop-room case is neither issue; its signature is a global BA that shrank the map
- **Where:** `cpp/src/edge/loops.cpp:1499-1562` (`global_bundle_adjust`), `cpp/src/foundations/graphs.cpp:265-310`.
- **Evidence:** 2 of 33 monocular runs exceeded the 2 cm bound: run 21 badly (15.22 cm final ATE, against 2.3-3.9 cm as tracked in the others), run 73 narrowly (2.16 cm); the passing runs 1.27-1.90 cm. In run 21 the as-tracked trajectory jumps 35.4 cm where the global BA was applied (frame 268 to 269); at the loop's correction it jumped 10.3 cm. Sim(3)-aligning the final positions to the as-tracked positions of the frames between the correction and the global BA (isolating the global BA's effect) gives scale 0.276 in run 21, 0.821 in run 73, 0.93-1.32 in the other 31. Neither failing run had a keyframe in flight across the correction. The global BA solves a copy taken inside the correction's stop, so neither suspected issue can reach it. Its solve summary looked ordinary: 3738 to 2180 over 10 iterations, ending at `max_iterations`. RGB-D: 8 of 8 runs at 4.77-4.79 cm.
- **Pointer for the deferred investigation:** ORB-SLAM3's visual global BA also leaves scale free (`Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false)`: 10 iterations, non-robust, only the first keyframe fixed). Start from the copy itself: dump it from a run like 21, solve it inline, and compare GTSAM's absolute `lambdaInitial` with g2o's tau·max(diag H) initial damping. Outside this PR's fixes; deferred by the owner.

#### [note] Python's `idle()` misses a keyframe taken but not yet flagged busy
- **Where:** `src/slam_it/edge/pipeline.py:1169-1171`, `:1246`; `src/slam_it/edge/threads.py:266-271`.
- Between `Worker`'s `get()` and `_busy = True`, `idle()` returns True with nothing waiting; in that gap a monocular keyframe can be queued while mapping starts another. C++ and Rust count the item in hand (`cpp/src/edge/pipeline.cpp:138`, `rust/slam-it-edge/src/pipeline.rs:2076-2078`), as ORB-SLAM3 does with `SetAcceptKeyFrames(false)` before `ProcessNewKeyFrame`. The window is a few bytecodes, and Python's threaded run is not held to the bar. Counting unfinished items, as the native code does, closes it.

#### [note] Field context
ORB-SLAM3's three-thread split (Campos et al., T-RO 2021) remains the reference for feature-based visual SLAM and is the right model for this rung. Learned successors keep the same front/back split, e.g. DPV-SLAM (Lipson, Teed & Deng, "Deep Patch Visual SLAM", ECCV 2024, https://arxiv.org/abs/2408.01654) runs loop closure and global optimisation in a backend beside odometry. Context, not a defect.

### Checked and fine
- **Stop handshake.** `Writer` matches `RequestStop`/`isStopped`/`Release`: a stop waits for mapping's turn and aborts its local BA; mapping cannot start a keyframe while a stop is pending; a stop inside a hold does not wait for itself. Python `threads.py:54-137`, C++ `threads.hpp`, Rust likewise.
- **Keyframe scheduling** matches `NeedNewKeyFrame`: refused while stopping; monocular only when idle; stereo and RGB-D interrupt the local BA and queue while fewer than `max_queued` (3) wait. Each queued keyframe sets the abort flag (`InsertKeyFrame`'s `mbAbortBA=true`).
- **Local mapping** skips fusion while a keyframe waits, and skips local BA and culling while a keyframe waits or a stop is requested, as `LocalMapping::Run` does.
- **Global BA generations** match `mnFullBAIdx`/`mbStopGBA`: the stop flag is per generation and atomic, and a superseded result is dropped.
- **`propagate_global_ba`** matches `RunGlobalBundleAdjustment` (`cpp/src/edge/loops.cpp:1564-1618`): solved keyframes take their result; the others move with their parent (child·parent⁻¹·parent_new, parents first by BFS); new points move with their reference keyframe's pre-application pose.
- **`follow`** keeps the velocity, a relative motion that a rigid correction applied to both frames leaves unchanged. Non-keyframes are placed relative to their reference in the view they were tracked on, which stays consistent across corrections.
- **`settle` and `finish`** drain mapping, then loop closing, then join the global BA threads; a lost map supersedes a running global BA first (section 12, item 6).
- **C++ shared state:** `corrections_` is atomic, `view_` is under `view_mutex_`, and `closing_.pending` is touched only under the writer's hold or turn. No data race, consistent with layer 1's TSAN.
- **C++ baseline:** tracking holds the map's lock through its frame, so issue 1 cannot occur there; issue 2 can.

Sources: ORB-SLAM3 `LoopClosing.cc`, `LocalMapping.cc`, `Tracking.cc` (https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/master/src); Deep Patch Visual SLAM (arXiv 2408.01654; ECCV 2024).
