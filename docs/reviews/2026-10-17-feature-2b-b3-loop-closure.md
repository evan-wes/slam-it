# Review panel: `feature/2b-b3-loop-closure`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-17

| | |
|---|---|
| **Branch** | `feature/2b-b3-loop-closure` |
| **Range** | `28c80dc9bc10e93c5789dbd15f218a0c0586ca78...5d857bbb6a03daa913665c7c51cc6d22a289b052` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; 4,513 tests) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `foundations/graphs.py:863-868` | `refine_similarity` accepts half-NaN and infinite `pixels2` rows (same as L1) | fix | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |
| H2 | should-fix | hygiene | `foundations/graphs.py:479-514` | Python `_solve` lets GTSAM's errors escape; C++/Rust raise `NotConverged` (same as A1) | fix | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |
| H3 | should-fix | hygiene | `loops.py`, `sim3.py`, `graphs.py` | A similarity's scale, validation and the hat map written several ways (with A5) | fix (with A5) | deferred (design section 14, item 10, with A5: a B4 panel item) -- decided by evan-wes, 2026-10-17 |
| H4 | should-fix | hygiene | `tests/edge/test_loops.py`, `tests/foundations/test_graphs.py` | Loop closing's and `refine_similarity`'s public refusals untested | fix | fixed (c1a1dc5: graphs' refusals; fe228fb: loop closing's twelve; e4ca44a: their table typed) -- decided by evan-wes, 2026-10-17 |
| H5 | should-fix | hygiene | `loops.py:1687`, `:1248` | `search_by_similarity`'s `window`/`distance` and `essential_graph` under-checked | fix | fixed (fe228fb) -- decided by evan-wes, 2026-10-17 |
| H6 | should-fix | hygiene | `docker/worker.Dockerfile`, `pyproject.toml` | The worker image's Python `gtsam` is unlocked; its comment's rationale is wrong (same as A3) | fix (with A3) | fixed (8d9151e) -- decided by evan-wes, 2026-10-17 |
| H7 | should-fix | hygiene | `pipeline.py:821` and twins | `loops.enabled` is a no-op without the keyframe database (same as A9) | fix (with A9) | fixed (81aecfe) -- decided by evan-wes, 2026-10-17 |
| H8 | should-fix | hygiene | `loops.cpp:1562`, `loops.py:1351` | Long `correct`/`judge_loops`; a comment contradicting a redundant lookup | fix the comment and lookup; split in B4 | fixed (fe228fb: the comment and the lookups); the splits deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel, with A15) -- decided by evan-wes, 2026-10-17 |
| H9 | should-fix | hygiene | `loops.py:1418`, `docs/notes/edge-vo.md:198` | Stale references (`as_dict`, `room_runs.json`) | fix | fixed (fe228fb: the Protocol; 758e00b: edge-vo.md) -- decided by evan-wes, 2026-10-17 |
| H10 | should-fix | hygiene | `scripts/compare_testsets.py:130` | `summarize` hard-codes `"tum"` for the ground truth | fix | fixed (fdb578f) -- decided by evan-wes, 2026-10-17 |
| H11 | should-fix | hygiene | `loops.py:1394-1401` | A per-point Python loop where one vector call does it | fix | fixed (fe228fb) -- decided by evan-wes, 2026-10-17 |
| H12 | should-fix | hygiene | `graphs.rs:553,571`, `graphs_bridge.cpp` | Unnamed dimensions in the GTSAM bridge | fix | fixed (f0d87c6) -- decided by evan-wes, 2026-10-17 |
| H13 | should-fix | hygiene | `docs/notes/test-set.md` (B3) | "Closer than as tracked on 20 of 22" cannot be reproduced from the table | fix | fixed (758e00b: the measure stated, the figures given) -- decided by evan-wes, 2026-10-17 |
| H14 | note | hygiene | `graphs.rs:533-585` | Rust `evaluate`'s "Never panics" rests on an invariant | fix the comment | fixed (f0d87c6) -- decided by evan-wes, 2026-10-17 |
| H15 | note | hygiene | Dockerfile / `setup_slam_it.sh`; `Cargo.toml` lints | Copies held equal by comment only | with A3 | fixed (8d9151e: tests/test_build_copies.py) -- decided by evan-wes, 2026-10-17 |
| H16 | note | hygiene | `test_loop_room.py:39-42`, `measure_testset.py:118-121` | Independent copies of the wrong-place thresholds (with A7) | with A7 | fixed (2d4cc93, with A7: one definition) -- decided by evan-wes, 2026-10-17 |
| H17 | note | hygiene | various | Smaller items: `loops` size, `fmt: skip` blocks, `build.rs` exit, a failed correction aborting the run | note; the last to B3b's design | noted: the module's size (split with A15), the `fmt: skip` blocks and `build.rs`'s exit kept as they are; the failed correction's question deferred (design section 12, item 14) -- decided by evan-wes, 2026-10-17 |
| A1 | must-fix | architecture | `foundations/graphs.py:479-512` | Python's graphs let GTSAM's `RuntimeError` escape (same as H2) | fix | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |
| A2 | must-fix | architecture | `tests/fixtures/foundations/` | Rung 0's `graphs` has C++ and Rust but no parity fixture | fix | fixed (04c388d) -- decided by evan-wes, 2026-10-17 |
| A3 | must-fix | architecture | `docker/worker.Dockerfile:28-59` | The worker image installs PyPI's GTSAM, unpinned, against its comments (same as H6) | fix | fixed (8d9151e) -- decided by evan-wes, 2026-10-17 |
| A4 | must-fix | architecture | `scripts/offline_ci.sh:36`, `.github/workflows/test.yml:19` | Offline CI's 8 GB cap cannot build GTSAM's wrapper; the workflow's 30 min timeout | owner's decision (cap) | fixed (cd81b10: 16 GB for the wrapper step alone, the timeout 120 min) -- decided by evan-wes, 2026-10-17 |
| A5 | should-fix | architecture | `sim3.py`, `loops.py`, `graphs.py` and twins | Sim(3) has no home as a group in rung 0 | defer to B4 | deferred (design section 14, item 10: a B4 panel item) -- decided by evan-wes, 2026-10-17 |
| A6 | should-fix | architecture | `foundations/graphs.py:139-146` | `graphs.Options` crosses through a hand-written dict, not `asdict` | fix | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |
| A7 | should-fix | architecture | `test_loop_room.py`, `measure_testset.py` | The wrong-place judgment is defined twice, outside `slam_it.eval` | fix | fixed (2d4cc93) -- decided by evan-wes, 2026-10-17 |
| A8 | should-fix | architecture | `tests/orchestration/test_vo.py` | Nothing holds the languages' diagnostics to one schema | fix | fixed (4fb8045) -- decided by evan-wes, 2026-10-17 |
| A9 | should-fix | architecture | `options.py:490`, `pipeline.py:821` and twins | `loops.enabled` silently does nothing without `relocalization.enabled` (same as H7) | fix | fixed (81aecfe) -- decided by evan-wes, 2026-10-17 |
| A10 | should-fix | architecture | `pyproject.toml`, `check_layers.py`, `graphs.py:295,301` | No contract confines GTSAM to `foundations.graphs` | fix | fixed (c1a1dc5: private converters; 99ea75a: the contract and check_layers.py) -- decided by evan-wes, 2026-10-17 |
| A11 | should-fix | architecture | `edge/__init__.py`, `foundations/__init__.py` | The rung docstrings do not place `loops`, `graphs`, `sim3` | fix | fixed (aab20b6) -- decided by evan-wes, 2026-10-17 |
| A12 | note | architecture | `CMakeLists.txt:98`, `build.rs` | Everything that links rung 0 now builds GTSAM | note | deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel) -- decided by evan-wes, 2026-10-17 |
| A13 | note | architecture | `_cpp` | `_cpp` exports GTSAM's symbols | fix (`--exclude-libs`) | fixed (f0d87c6) -- decided by evan-wes, 2026-10-17 |
| A14 | note | architecture | `essential_graph`, `Window`/`Search` | The module APIs mirror each other unevenly | with B4 | deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel) -- decided by evan-wes, 2026-10-17 |
| A15 | note | architecture | `loops` | About 2,200 lines per language; B4 reuses half | split in B4 | deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel) -- decided by evan-wes, 2026-10-17 |
| A16 | note | architecture | `cpp/include/slam_it/foundations/graphs.hpp:219` | Stale: a point behind its camera "contributes nothing" | fix | fixed (f0d87c6) -- decided by evan-wes, 2026-10-17 |
| L1 | must-fix | algorithm | `foundations/graphs.py:863-868` | `refine_similarity` accepts infinite or half-NaN `pixels2` rows (same as H1) | fix | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |
| L2 | should-fix | algorithm | `graphs.py:479-514` and twins | Global BA's LM damping does not scale with the problem; a no-op on advio-13 | own PR after B3, measured | deferred (design section 11, Deferred from B3's review panel, and section 7: its own measured PR after B3) -- decided by evan-wes, 2026-10-17 |
| L3 | note | algorithm | `graphs.py:466-469` | In the Sim(3) refinement a point behind its camera costs nothing | document | fixed (c1a1dc5: documented) -- decided by evan-wes, 2026-10-17 |
| L4 | note | algorithm | GTSAM `BetweenFactor<Similarity3>` | Its Jacobian is first-order only | document | fixed (c1a1dc5: documented) -- decided by evan-wes, 2026-10-17 |
| L5 | note | algorithm | `loops.py:1122-1128` | Loop fusion can merge two of the group's points: an unlisted departure | document | fixed (fe228fb: in the departures) -- decided by evan-wes, 2026-10-17 |
| L6 | note | algorithm | design section 11 | The bar compares one seed-0 run without merging against a median of 10 with Atlas | document | fixed (758e00b: section 11 states the protocol difference) -- decided by evan-wes, 2026-10-17 |
| L7 | note | algorithm | design section 10, `measure_testset.py:273-331` | "Poor pose" judges the final poses, not those right after the correction | document | fixed (758e00b: section 10's wording follows the code) -- decided by evan-wes, 2026-10-17 |
| L8 | note | algorithm | `test_loop_room.py` | The RGB-D/stereo bounds are 3-5x looser than monocular's | backlog | deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel) -- decided by evan-wes, 2026-10-17 |
| L9 | note | algorithm | -- | Guards against wrong loops in current back ends (GNC, PCM) | backlog | deferred (the ladder's open items, Deferred from milestone 2B's B3 review panel) -- decided by evan-wes, 2026-10-17 |
| L10 | note | algorithm | `graphs.bundle_adjust` docstring | A point seen once is free along its ray, not "constrained only along" it (and A2) | fix the wording | fixed (c1a1dc5) -- decided by evan-wes, 2026-10-17 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review


**Scope:** `28c80dc9...5d857bb` (feature/2b-b3-loop-closure, 0.3.11.1 to 0.3.12.0), 167 files, read at HEAD. I read in full: `src/slam_it/edge/loops.py`, `foundations/graphs.py`, `foundations/sim3.py`, and the diffs of `map.py`, `database.py`, `covisibility.py`, `mapping.py`, `options.py`, `pipeline.py`, `eval/runs.py`, `orchestration/{vo,bundles}.py`, `scripts/{measure,compare}_testsets.py`, the test files, `build.rs`, the cxx bridge and the Dockerfile and setup script. I read most of Rust `loops.rs`, `graphs.rs` and the C++ `correct`, `solve` and `in_view`. I skimmed the rest of C++/Rust (bindings, writers, `vo`), the viewer, and the docs. I did not review the 5,788-line fixtures line by line. I ran three read-only probes outside the repo (`git status` stayed clean).

**Summary:** This is careful, well-documented work. Docstrings state conventions (`S_cw`, GTSAM's `Similarity3` mapping, tangent order) and ORB-SLAM departures. Rust library code has no `unwrap`/`expect`/`panic`. There are no debug prints, dead code, TODOs or hard-coded paths. Earlier duplicates were folded into shared helpers (`maps.replace_point`, `maps.keyframe_row`), and the counts in CHANGELOG and the notes match their tables. One validation hole at a public boundary is must-fix. The rest is mostly duplicated validation and arithmetic, untested refusal paths, a few over-long functions, and a worker-image reproducibility gap.

### Findings

#### [must-fix] `graphs.refine_similarity` silently accepts half-NaN and infinite `pixels2` rows
- **Where:** `src/slam_it/foundations/graphs.py:863-868`. The contract is stated at `:826-829`.
- **Issue:** The documented contract is that a `pixels2` row is finite, or wholly NaN for "image 2 does not observe this match". The code builds `unseen` only from fully-NaN rows, then runs `as_batch` on `np.nan_to_num(raw2)`. So `[nan, 210]` becomes `[0.0, 210.0]`, and `[inf, 210]` becomes `[1.8e308, 210]`. Both pass the finiteness check and count as real image-2 observations. Rust's core is stricter and rejects both (`graphs.rs:1005-1011`: `finite(p) || p.iter().all(is_nan)` else "matches: expected finite values"). Python is the reference and public boundary, and it does not enforce what Rust enforces.
- **Why it matters:** A caller bug (one coordinate lost) turns into a wrong chi-square and a biased `OptimizeSim3` result with no error. The compiled backends never see the bad row either, because Python has already rewritten it. `loops._refine` only produces clean rows today, so this is a latent hole, but it is a public function.
- **Evidence (run):**
  ```
  pixels2[1,0]=nan  -> accepted; chi2 row 1 = [2331.6, 5746.2]   (image-2 error computed against pixel (0, 210))
  pixels2[1,0]=inf  -> accepted; chi2 row 1 = [789.5, inf]       (+ RuntimeWarning: overflow in matmul)
  ```
- **Suggested fix:** Validate on the raw array: `unseen = isnan(raw2).all(axis=1)`, then require `np.isfinite(raw2[~unseen]).all()` before any `nan_to_num`, or just keep NaN rows through `as_batch` with a finiteness check on the seen rows only. Add the two cases to `test_bad_graphs_are_refused`, which currently has no `refine_similarity` case (see the test-gap finding).

#### [should-fix] Python `_solve` does not map GTSAM failures to the shared error contract
- **Where:** `graphs.py:479-514` (no `try`); contrast `cpp/src/foundations/graphs.cpp:267-304` (`catch` to `NotConverged`, "GTSAM failed: ...") and `rust/slam-it-foundations/src/graphs.rs:701-706` (`gtsam_failed`).
- **Issue:** CLAUDE.md says Python raises the matching `slam_it.foundations.errors` exception. C++ and Rust convert any GTSAM exception to `NotConverged`. The Python reference lets a raw GTSAM `RuntimeError`/`IndexError` escape.
- **Why it matters:** The three languages fail differently on the same input, and callers catching `NotConverged` miss Python's.
- **Suggested fix:** Wrap the optimizer construction and `iterate()` in `try/except Exception` and raise `NotConverged` with the message. This is the one place a broad catch is justified; say so in a comment.

#### [should-fix] The same similarity logic is written several ways within Python
- **Where:**
  - Scale of `[[sR,t],[0,1]]`: `loops.py:111` (`norm(col 0)`), `sim3.py:96` (`sum(col 0 ** 2)`), `graphs.py:291` (`cbrt(det)`), `loops.py:1528` (`cbrt(det)`).
  - Validating a similarity: `loops.py:1523-1534` (`_similarity`) and `graphs.py:546-563` (`_poses`).
  - Cross product matrix: `graphs.py:311` (`_hat`) and `so3.hat` (`so3.py:329`).
- **Issue:** `_similarity` checks orthonormality with `np.allclose(..., atol=1e-6)`, an unnamed copy of `so3.ROTATION_TOLERANCE`. `allclose` also applies its default `rtol=1e-5`, so it is looser than `check_rotation`, which `_poses` uses. A similarity can pass `loops.detect`'s check and then be refused inside `graphs.refine_similarity`, with a different message. I checked `graphs._hat(v)` against `so3.hat(v)` for a (3,) vector: identical.
- **Suggested fix:** Put one `scale_of` and one validated "split a similarity" next to `sim3.inverse` (rung 0). Have `loops._similarity` call `check_rotation`, and delete `graphs._hat` in favour of `so3.hat`.

#### [should-fix] Loop closing's public refusals are untested
- **Where (code):** `loops.py:1523-1550` (`_similarity`, `_keypoint_ids`), `:1846-1847` (`group_points` linked rows), `:2000-2001` (`fuse_by_similarity` negative id), `:2053-2054` (`global_bundle_adjust` without keyframe 0), `:2115-2118` (`detect` mode and pending similarity), `:2168-2173` (`correct`). Also `graphs.refine_similarity`'s `huber`, `information` and empty-matches refusals (`graphs.py:869-873`), and the `Options` fields other than `max_iterations`.
- **Evidence:** `tests/edge/test_loops.py` has `pytest.raises` at only four places (`:342`, `:473`, `:647`, `:656`), all "not in the map". `tests/foundations/test_graphs.py::test_bad_graphs_are_refused` covers pose graphs and BA only. The missing `refine_similarity` case is how the must-fix above got through.
- **Suggested fix:** One parametrised "bad arguments are refused" test per module, asserting the message names the argument.

#### [should-fix] `search_by_similarity` and `essential_graph` are public but under-checked
- **Where:** `loops.py:1687` (`window`, `distance`), `:1248` (`essential_graph`).
- **Issue (run):** with `window` NaN, -8 or inf the call is refused, but by `matching.features_in_area` with "radius: must be finite and positive", which names the wrong argument. `window` is only validated if some point is in view. `distance=-5` is accepted and returns 0 matches. `distance=10**9` is accepted and matches everything (338 matches in my probe), although `Options` bounds the same quantity to `matching.MAX_DISTANCE`. `essential_graph` has no `backend=`, no input checks and no `Raises`, though its name is public and it is called directly by two tests. `len(start)` or `len(before)` differing from `m.keyframes` would index wrongly.
- **Suggested fix:** Validate `window` (finite, > 0) and `distance` (`as_count(..., 0, MAX_DISTANCE)`) up front, as `Options` does. Make `essential_graph` private (`_essential_graph`), or validate it.

#### [should-fix] The worker image's Python `gtsam` is unlocked, and the stated rationale is wrong
- **Where:** `docker/worker.Dockerfile:27-45, 49-56`; `pyproject.toml:15-18`; `scripts/gen_env_lock.sh`.
- **Issue:** The Dockerfile comment says GTSAM is built "without its Python wrapper: the worker runs the native program, which carries GTSAM". But `slam_it.orchestration.vo` imports `edge.pipeline`, which imports `loops`, then `graphs`, then `import gtsam` at module top, so the worker's Python needs `gtsam`. The second stage runs `pip install --constraint slam-it.lock "<wheel>[video]"`. The wheel declares `gtsam>=4.3`, and `gen_env_lock.sh` strips gtsam from the lock. pip therefore pulls whatever PyPI serves, unpinned. This is a second, different GTSAM build in an image that is otherwise digest-pinned. The `pyproject.toml` comment "no index is asked for it" is untrue for this path. PyPI does serve `gtsam==4.3.0`: `pip download` returned a cp312 wheel; I did not check cp311.
- **Why it matters:** Nothing in layer 1 builds the image, so I could not verify it. Reproducibility of the demo worker regresses, and the comment misleads.
- **Suggested fix:** Pin gtsam (hash or version) in the image's constraints, or make the `graphs` import lazy so the native-program path does not need it. Correct both comments.

#### [should-fix] `loops.enabled` is silently a no-op without the keyframe database
- **Where:** `pipeline.py:821`, `cpp/src/edge/pipeline.cpp:408`, `rust/.../pipeline.rs:938`; the dependency is documented only in `LoopOptions.enabled` (`options.py`).
- **Evidence (run):** `Options().with_settings({"relocalization.enabled": False})` is accepted with `loops.enabled` still True. `scripts/measure_testset.py:78` advertises `--option relocalization.enabled=false` as "measures without relocalisation". Since B3 that measure also silently loses loop closing, so its final ATE equals the tracked ATE.
- **Suggested fix:** Either record it in the run, or say in the measure's docs that it also turns loops off. Or validate and force an explicit `loops.enabled=false`. Pick one so the ablation cannot be misread.

#### [should-fix] Long multi-phase functions, and a comment that contradicts the code
- **Where:**
  - `cpp/src/edge/loops.cpp:1562-1677`: `correct` is about 115 lines, with `NOLINT(readability-function-cognitive-complexity)` ("Python's `_correct`, step for step"). A second suppression sits at `:318` for `refine`.
  - Rust `correct` (`loops.rs:2208-2290`) is about 82 lines. Python `_correct` (`loops.py:1328-1382`) is 55.
  - `scripts/measure_testset.py:273-335`: `judge_loops` is 62 lines, two judgements in one loop body.
- **Contradicting comment (`loops.py:1351`, `:1356-1359`, Rust `loops.rs` equivalent):** the code re-looks-up every moved keyframe's row (`moved_now = [maps.keyframe_row(...)]`), then a comment says "the keyframes are the same rows before and after: nothing here adds or removes one". C++ skips the lookup and relies on that. Either the lookup is dead code or the comment is wrong. Also, `loops.py:1347` indexes `moved_rows[moved_ids.index(keyframe)]`, where `zip(moved_ids, moved_rows)` says it directly.
- **Suggested fix:** Split `correct` into move/replace/fuse, collect new links, and optimise (graph plus BA). Drop the NOLINT and the redundant lookups. Split `judge_loops` into a place-judgement and a relative-pose judgement.

#### [should-fix] Stale references
- **Where:** `loops.py:1418` says "a `Pending` and a `Loop` as their `:meth:`~Pending.as_dict``". Neither class has `as_dict`; the module uses `_pending_as_dict`/`_loop_as_dict` (`:1553`, `:1566`). Also `docs/notes/edge-vo.md:198` still points at `tests/fixtures/edge/room_runs.json`, which this PR deletes. The same stale path remains in `docs/design/2026-10-05-edge-vo.md:138` and `docs/design/2026-10-09-run-viewer.md:145`; those are dated design records, so lower priority.
- **Suggested fix:** Correct the docstring. Update the living note to `room_runs_<backend>.json`.

#### [should-fix] `compare_testsets.summarize` hard-codes `"tum"` for ground truth
- **Where:** `scripts/compare_testsets.py:130`: `runs.ground_truth("tum", folders[row["name"]])`.
- **Issue:** This PR adds `Entry.dataset` and EuRoC sequences, but `folders` drops the dataset. Today it only matters for pre-B3 batches, which are all TUM-format, so it works by accident.
- **Suggested fix:** Map name to `Entry` and pass `entry.dataset`.

#### [should-fix] A per-point Python loop where one vector call does it
- **Where:** `loops.py:1394-1401` (`_move_by_graph`) calls `m.keyframe_rows(m.pt_reference_kf[p:p+1])[0]` once per point. `_move_keyframes` (`:1149`) and `_replace_matched` (`:1166`) do the same with one-element `point_rows` calls.
- **Why it matters:** Python is the reference, never real time, so the cost is bounded. But `keyframe_rows` is already vectorised, so hoisting `m.keyframe_rows(m.pt_reference_kf)` above the loop is a one-line change that removes O(P) numpy calls from every loop correction.

#### [should-fix] Unnamed dimensions in the Rust/C++ GTSAM bridge
- **Where:** `rust/slam-it-foundations/src/graphs.rs:553` (`values.len() == 19`) and `:571` (`== 16`); `cxx/graphs_bridge.cpp:102/108/114` (`tangent += 6/7/3`) and `:130-132` (the same 6/7/3 written again as a nested ternary); also `:40` and `:50` (16).
- **Issue:** The pose/similarity/point sizes (16, 19, 6, 7, 3) are literals. The kind-to-dimension mapping is written twice in one function. Python and Rust already have `POSE_DIM` and `SIMILARITY_DIM`.
- **Suggested fix:** Name them (`MATRIX_VALUES`, `PROJECTION_VALUES = MATRIX_VALUES + 3`) and add one `tangent_dimension(Kind)` helper in the bridge.

#### [should-fix] A count in the B3 note cannot be reproduced from its table
- **Where:** `docs/notes/test-set.md`, B3 bullet "The final trajectory is closer to the truth than the poses as tracked on 20 of the 22 runs and the same on 2".
- **Evidence:** Parsing that table's Final and As tracked columns (one-decimal cm) gives 18 closer, 3 equal (MH_01 stereo, V1_01 mono, V1_01 stereo) and 1 farther (fr1_desk mono 1.9 vs 1.6). That fr1_desk row compares keyframes (Final) against frames (As tracked), per the paragraph under the table. The text may be counting the frames' final ATE at exact precision, but the note neither says so nor shows those figures.
- **Why it matters:** This is the kind of claim your commit-message rule says to check against its source.
- **Suggested fix:** State the metric, or give the exact figures. I checked the other counts (16/22 bar, 10 loops, 209,148/208,906, maps 225/226, losses 176/177, 18 loops in 14 runs, 77 runs, 27 cases) and they agree with the tables.

#### [note] Rust `evaluate` says "Never panics", but depends on an invariant
`graphs.rs:533-585`: `residual.copy_from_slice(...)` panics if the C++ side's `dimension` differed from the residual length. All call sites pass consistent values (2 or 3), so this cannot happen today, but the doc comment states more than the code guarantees.

#### [note] Copies kept equal by comment only
- The Dockerfile's GTSAM CMake options mirror `setup_slam_it.sh`'s `GTSAM_OPTIONS` ("Keep the options equal..."). The Eigen block already works this way.
- `slam-it-foundations/Cargo.toml` copies the workspace lint tables.
- No check holds either pair equal, so they can drift.

#### [note] Intentionally independent copies of the "wrong place" thresholds
`tests/orchestration/test_loop_room.py:39-42` re-defines `WRONG_DISTANCE`/`WRONG_ANGLE` (radians) and re-computes the viewing angle, alongside `scripts/measure_testset.py:118-121` (degrees). This is reasonable as an independent oracle. A comment saying the values are held equal on purpose would stop silent drift.

#### [note] Smaller items
- `loops.py` is 2,187 lines: the algorithm, ~500 lines of crossing/decoding, and 8 public wrappers. Rust (2,290) and C++ (1,685) are similar. This is fine today. Split when B3b adds the thread.
- 33 new `# fmt: skip` blocks (10 in `loops.py`, 15 in `test_loops.py`, 8 in `test_graphs.py`) bypass the formatter to keep calls dense.
- `build.rs` ends with `process::exit(1)` after a `cargo:warning`. A `panic!` with the message is the conventional failure for a build script.
- For the architecture/algorithm reviewers: a GTSAM failure inside `loops::correct` (C++/Rust) propagates through `close_loops` and aborts the whole run. Whether a failed correction should be recorded and skipped is a design question.

### Checked and fine
- **Rust library code:** `loops.rs`, `graphs.rs`, `sim3.rs` and the `slam-it-py` bindings have no `unwrap`/`expect`/`panic!`. Test code does. The two `expect`s in `pipeline.rs` (`to_index`, `to_id`) are old and state their invariants.
- **Hygiene sweep of the diff:** no commented-out code, no TODO/FIXME, no `println!` outside `build.rs`/CLI scripts, no bare `except`, no hard-coded paths.
- **Tolerances and constants:** thresholds and tolerances are named and justified (`GRAPH_TOLERANCE`, `LOOP_ATE`, `SURFACE`, `TRACKED`, `WRONG_*`, `POOR_*`, the `LoopOptions` fields with their ORB-SLAM sources).
- **Settings validation:** all `loops.*` settings are range-checked by `Options._checked` (counts >= 1, fractions in (0, 1], distances <= `MAX_DISTANCE`, `loops.confidence` < 1).
- **Dedup:** `_merge_points` became `maps.replace_point`, `_keyframe_row` became `maps.keyframe_row`, and `stretches`/`kind`/`relocalised_errors` moved into `measure_testset` with no copy left in `compare_testsets`.
- **Run format and bundle:** `_check_loops` validates the loop edges. `read_run` checks `trajectory_tracked.txt` has the same times. Both writers and the viewer carry version 7 / 4.
- **Documentation:** module docstrings state purpose, ladder position and ORB-SLAM departures. Python, C++ and Rust docs state shapes, frames (`S_cw`, `T_world_cam`, GTSAM tangent order, the `Similarity3` translation convention) and `Raises`/`@throws`/`# Errors`.
- **Tests:** they assert on values. There are Jacobian-vs-finite-difference tests, ring and known-answer solves for each backend, behind-camera cost, solver termination reasons, degenerate and bad-shape cases for `sim3_ransac`, the stage-parity tests against Python, per-language golden records with an inputs check, and the loop-room bar with its numbers explained in the docstring.
- **Python solve rule:** the termination rule in `_solve` equals C++'s (`previous` vs `current + decrease`).
- **Viewer:** JSDoc, tests and the CSS variable pair are consistent.
- **Version/CHANGELOG:** 0.3.12.0 matches its scope (PATCH-level feature batch under a MINOR milestone, as the 2B milestones have been numbered).
### Architecture review


**Scope:** `git diff 28c80dc9...5d857bbb` on `feature/2b-b3-loop-closure`: 167 files across `src/slam_it/{foundations,edge,eval,orchestration}`, `cpp/` (cores, bindings, the VO program), `rust/` (including the `cxx` bridge in `rust/slam-it-foundations/cxx/`), tests and fixtures, `scripts/` (setup, offline CI, the measures), `docker/worker.Dockerfile`, `pyproject.toml`, `CMakeLists.txt`, the viewer, and the design docs (ladder and online-SLAM sections 10 and 11). I read these against `CLAUDE.md`, the ladder doc's "Languages", "Package layout" and "Dependency contracts" sections, and the online-SLAM design's section 10.

**Summary:** The structure is sound and follows the design:
- Loop closing sits at the edge rung's `(loops) | (relocalization)` layer.
- GTSAM is kept inside rung 0's `graphs`: it is out of every C++ header, and on the Rust side it sits behind a private `ffi` module, the only place `unsafe` is allowed.
- Bindings stay thin, one file per module.
- The owner's decision of 2026-10-17 (a golden record per language, one bar for all three) is carried through the tests.

The problems are at the edges of the new dependency:
- When GTSAM fails, Python raises a different error from C++ and Rust.
- Rung 0's `graphs` has no parity fixture.
- The worker image takes an unpinned GTSAM from PyPI, while its comments say otherwise.
- The PR's own gate cannot plausibly build the env this PR needs.

Also, Sim(3) as a group still has no home in rung 0.

### Findings

#### [must-fix] Python's graphs let GTSAM's RuntimeError escape; C++ and Rust map the same failure to NotConverged
- **Where:** `src/slam_it/foundations/graphs.py:479-512` (`_solve`, `optimizer.iterate()` at 492, with no translation); against `cpp/src/foundations/graphs.cpp:264-304` (`catch (const std::exception&)` returns `kNotConverged`) and `rust/slam-it-foundations/src/graphs.rs:692-705` (`gtsam_failed` returns `NotConverged`).
- **Issue:** The C++ and Rust cores document and return `NotConverged` when GTSAM throws. The Python reference catches nothing, so GTSAM's wrapper raises a bare `RuntimeError`, which is not a `SlamItError`. The `Raises` sections of `optimize_pose_graph`, `refine_similarity` and `bundle_adjust` list only `InvalidInputError`.
- **Why it matters:** The design says "The Python reference raises the same ones, so every language fails the same way" (ladder doc, "error contract"). The same failure inside a loop's correction would end a Python run with `RuntimeError` but a native run with `NotConvergedError`. The worker, which catches `SlamItError`, would treat the Python failure as a crash. Validation makes this path rare (a `CustomFactor` callback failing, or numerical trouble), but the contract is explicit.
- **Evidence:** A probe in the `slam-it` env shows GTSAM's wrapper raising `(<class 'RuntimeError'>, ...) Attempting to at the key "5", which does not exist in the Values.`; no `except` appears anywhere in `graphs.py` except around `check_rotation`.
- **Suggested fix:** In `_solve`, wrap the optimiser's construction and `iterate()` in a `try`/`except RuntimeError` that raises `NotConvergedError(f"GTSAM failed: {error}") from error`, as the cores do. Then add `NotConvergedError` to the three docstrings' `Raises`.

#### [must-fix] Rung 0's `graphs` has C++ and Rust implementations but no parity fixture
- **Where:** `tests/fixtures/foundations/` has a `sim3.json` (from `generate_sim3.py`) but nothing for `graphs`. `tests/foundations/test_graphs.py:174-336` holds each backend to a known answer; `cpp/tests/foundations/test_graphs.cpp` reads no fixture.
- **Issue:** `CLAUDE.md` says: "Every C++/Rust implementation has a Python reference, a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The ladder adds that "the summary fields go into the parity fixtures". Every other compiled rung-0 module has a fixture, including this PR's `sim3`. `graphs` has none. Its three languages are compared only indirectly, through the edge rung's live comparisons in `tests/edge/test_loops.py:560-635`. Nothing at rung 0 holds the summaries (iterations, termination) of the three backends to one another.
- **Why it matters:** The cores build their factors differently (Python uses GTSAM's own pinhole factors; C++ and Rust use their own). The stopping rule is also re-implemented in each language. Rung 0 is where a drift in either should surface, before it shows up as an edge-rung tolerance failure that is hard to trace. This divergence from the documented rule is not recorded anywhere.
- **Evidence:** `ls tests/fixtures/foundations/` shows `sim3.json` and no `graphs*`; `grep -i fixture cpp/tests/foundations/test_graphs.cpp` finds nothing.
- **Suggested fix:** Recommended (code changes): add `generate_graphs.py` and `graphs.json` with the ring pose graph (SE(3) and Sim(3)), both refinement rounds, and a monocular and a stereo BA, using Python's results as expected values under a stated tolerance; then check them with `check_fixture` on all three backends. Alternative (doc changes): if the owner accepts the known-answer tests plus the edge-level comparisons as `graphs`' parity, record that exception in the ladder's "Parity" paragraph and in `CLAUDE.md`.

#### [must-fix] The worker image installs PyPI's GTSAM, unpinned, while its comments say the image carries no Python wrapper
- **Where:**
  - `docker/worker.Dockerfile:28-31` says "but without its Python wrapper: the worker runs the native program, which carries GTSAM".
  - `docker/worker.Dockerfile:59`: `pip install --constraint /tmp/slam-it.lock "${wheel}[video]"`.
  - `pyproject.toml:16-19`: `dependencies = [..., "gtsam>=4.3"]`, with the comment "no index is asked for it".
  - `scripts/gen_env_lock.sh` now drops `gtsam` from the lock.
- **Issue:**
  - The wheel declares `gtsam>=4.3`, and the lock no longer pins it, so the final stage resolves GTSAM from PyPI at whatever version is newest when the image is built. That build is not ours: it is not compiled with `EIGEN_NO_CPUID` and is not static.
  - The worker cannot do without it: `worker.py:93` imports `vo`, `vo.py:76` imports `edge.pipeline`, `pipeline.py:82` imports `loops`, and `loops` imports `foundations.graphs`, whose line 54 is `import gtsam` at module level.
- **Why it matters:**
  - The deployed image holds an unpinned binary dependency: a future GTSAM release that changes `CustomFactor` or `LevenbergMarquardtOptimizer` breaks the worker on import.
  - The comments, the CHANGELOG ("the worker's image build GTSAM 4.3.0 from its tag with `EIGEN_NO_CPUID`") and design section 10, item 1 ("and the worker's image the same way") all describe something the image does not do.
- **Evidence:** `pip install --dry-run --only-binary=:all: --python-version 3.11 --platform manylinux2014_x86_64 gtsam==4.3.0` returns `Would install gtsam-4.3.0` (`gtsam-4.3.0-cp311-cp311-manylinux2014_x86_64...whl`), so the image's resolve succeeds silently with PyPI's build.
- **Suggested fix:** Three options:
  - Build and install the wrapper in the image as `setup_slam_it.sh` does (costly).
  - Pin `gtsam==4.3.0` with hashes in a lock the image uses, and say in the Dockerfile that the Python side is PyPI's build, imported but never used to solve.
  - Import `gtsam` lazily inside the reference paths and move it to an extra the worker does not install. This option changes the ladder's "Runtime dependencies", so the doc changes first.

  Whichever is chosen, keep the GTSAM version and options in one place that both `setup_slam_it.sh` and the Dockerfile read, or hold their copies equal with a test, as `tests/server/test_shared_names.py` does. Today it is a comment ("Keep the options equal"), and `EIGEN_NO_CPUID` is the determinism guarantee.

#### [must-fix] The PR's gate cannot plausibly build the env this PR requires
- **Where:**
  - `scripts/offline_ci.sh:36`: `CAP=(systemd-run ... -p MemoryMax=8G -p MemorySwapMax=2G --)`, applied to `setup_slam_it.sh` at line 86.
  - `scripts/setup_slam_it.sh:124` ("the largest needs more than 8 GB") and `:167` (`ninja ... -j1 gtsam_py`).
  - `docs/design/2026-10-14-online-slam.md:424` ("over 8 GB to compile, so that step runs one file at a time under a 16 GB cap").
  - `.github/workflows/test.yml:19` (`timeout-minutes: 30`).
- **Issue:**
  - This PR changes `setup_slam_it.sh`, so offline CI's `LOCK_HASH` changes and the `slam-it-ci` env is recreated from scratch. That means building GTSAM and its Python wrapper under an 8 GB cap, below what the PR itself says the wrapper needs.
  - The GitHub workflow had about 30 to 40 minutes of fresh setup already (`offline_ci.sh`'s own estimate) against a 30-minute timeout. It now adds GTSAM, plus a one-file-at-a-time wrapper build.
- **Why it matters:** Offline CI gates every PR, and its result goes in the PR template. As configured, it is likely to fail at setup, by an OOM kill or by heavy swapping, before layer 1 runs. The GitHub workflow will likely time out on the next push to `main`.
- **Evidence:** `~/.cache/slam-it-offline-ci/results.tsv` has no entry for `5d857bb`; the last recorded run is B2's `64e42ade`.
- **Suggested fix:** Ask the owner what cap the GTSAM step should run under: the design says 16 GB, and the global rules forbid raising a cap silently. Then make `offline_ci.sh` run setup under that cap, and raise the workflow's timeout or cache the GTSAM build keyed on `GTSAM_BUILD_ID`. Run offline CI on this head before a merge.

#### [should-fix] Sim(3) has no home as a group in rung 0; its operations are scattered, three times over
- **Where:**
  - Python: `src/slam_it/foundations/sim3.py:82` (`inverse`, inside the RANSAC module); `src/slam_it/edge/loops.py:98,114` (`scale_of`, `rigid_of`, in the edge rung); `src/slam_it/foundations/graphs.py:289` (`_similarity_parts`, which takes the scale as `cbrt(det)` where the others use a column's norm); `graphs.py:311` (`_hat`, a copy of `so3.hat`).
  - C++: `cpp/include/slam_it/edge/loops.hpp:35,40`, `cpp/include/slam_it/foundations/sim3.hpp:32`.
  - Rust: `rust/slam-it-edge/src/loops.rs:39,54`, `rust/slam-it-foundations/src/sim3.rs:28`.
- **Issue:** The rung-0 docstring (`foundations/__init__.py`) and the ladder's rung-0 row list Sim(3) among the Lie groups, but there is no Sim(3) group module. `so3` and `se3` are groups, while `sim3` is RANSAC that happens to export the inverse. The scale and the rigid part of a similarity live in the edge rung, and `graphs` decomposes a similarity its own way.
- **Why it matters:** The ladder says groups shared by several rungs belong in rung 0, not copied. B4's merge needs the same Sim(3) operations (merging maps by a Sim(3)), and so will the classic rung. Two definitions of a similarity's scale (`cbrt(det)` and a column's norm) agree only on exact inputs.
- **Evidence:** `grep -n "def scale_of\|def rigid_of\|def inverse\|def _similarity_parts"` across the three languages, as listed above.
- **Suggested fix:** Give Sim(3)'s group operations (inverse, scale, rigid part, compose) one rung-0 module in each language. Rename the RANSAC module to say what it estimates (`sim3_ransac` or similar), or keep `sim3` as the group and move the RANSAC out. Have `graphs` and `edge.loops` use it, and `so3.hat` in place of `_hat`. This does not pre-empt the ladder's open item on taking Lie groups from a library: it gives that decision one place to act on.

#### [should-fix] `graphs.Options` crosses through a hand-written dict, not `dataclasses.asdict`
- **Where:** `src/slam_it/foundations/graphs.py:139-146`; every other rung-0 module uses `asdict(...)` (`ransac.py:678`, `twoview.py:540`, `matching.py:706`, ...).
- **Issue:** The ladder fixes "a solver whose settings are a frozen dataclass passes `dataclasses.asdict(options)` ... so a field added on one side fails loudly". Here, a field added to `Options` but not to `as_dict` passes the binding's `check_keys` unchanged, and the compiled backends silently ignore it.
- **Suggested fix:** Return `asdict(self)`, or drop `as_dict` and pass `asdict(chosen)` at the three call sites.

#### [should-fix] The "wrong place" judgment of a loop is defined twice, outside `slam_it.eval`
- **Where:** `tests/orchestration/test_loop_room.py:39-42,83-84` (`WRONG_DISTANCE = 3.0`, `WRONG_ANGLE = math.radians(60.0)`) and `scripts/measure_testset.py:118-127,263-331` (`WRONG_DISTANCE = 3.0`, `WRONG_ANGLE = 60.0` in degrees, `judge_loops`).
- **Issue:** Design sections 10 and 11 define one bar for the rooms and for real data. Its loop judgment (distance, viewing angle, poor pose) is an evaluation metric, yet it lives as two copies with the angle in different units. Commit `92f824b` already had to change one copy to viewing directions, which the test also uses.
- **Suggested fix:** Move `judge_loops` and its constants into `slam_it.eval` (`eval.runs` or a new module in eval's contract). Have both the loop-room test and the measure call it.

#### [should-fix] Nothing holds the three languages' diagnostics to one schema now that the agreement tests are gone
- **Where:** `tests/orchestration/test_vo.py`, where the cross-backend `_assert_same_run` and `test_the_backends_agree*` were removed. The JSON writers are `cpp/src/orchestration/vo.cpp` (`detection_json`, `correction_json`) and `rust/slam-it-orchestration/src/vo.rs:618,663`, against Python's `DetectionSummary`/`CorrectionSummary.as_dict`. `src/slam_it/eval/runs.py` does not validate stage summaries.
- **Issue:** Each language's record is now checked only against its own golden. A key misspelled or missing in a native JSON writer would be baked into that language's regenerated record and pass. The viewer (`solver.js` `detectionText`/`correctionText`) and the measures read these keys by name. The edge stage comparisons cover the bindings' dicts, not the programs' JSON writers.
- **Why it matters:** The run format says "C++ and Rust write the same files". Without a schema check, one backend's viewer panel or measure can break silently.
- **Suggested fix:** Add a schema-only test on the loop room. For each backend, collect the set of key paths and value types of each stage's records over the run, and require them to equal Python's. This compares shapes, not decisions, so it stays within the owner's rule.

#### [should-fix] `loops.enabled` silently does nothing without `relocalization.enabled`
- **Where:** `src/slam_it/edge/options.py:490` ("It needs the keyframe database, so `relocalization.enabled` too"); the gate is at `src/slam_it/edge/pipeline.py:821`, `cpp/src/edge/pipeline.cpp:408` and `rust/slam-it-edge/src/pipeline.rs:935-938`.
- **Issue:** The keyframe database belongs to place recognition, which both loop closing and relocalisation use, but its existence is switched by `relocalization.enabled` alone. A run with relocalisation off and `loops.enabled` true (its default) closes no loop, and its `run.json` still records `loops.enabled: true`.
- **Suggested fix:** Either refuse the combination in `Options` validation (`InvalidInputError`: "loops.enabled needs relocalization.enabled"), or keep the database whenever either feature is on, as ORB-SLAM always keeps its `KeyFrameDatabase`. Do it in all three languages.

#### [should-fix] New contract: GTSAM confined to `foundations.graphs`
- **Where:** `pyproject.toml`'s contracts and `scripts/check_layers.py` (whose `_CPP_FILE_HEADERS` mechanism, line 106, is the model). `src/slam_it/foundations/graphs.py:295,301` exposes the public `to_similarity3` and `from_similarity3`, which take and return `gtsam` types.
- **Issue:** The ladder says GTSAM is "behind rung 0's `slam_it.foundations.graphs`". Today that holds only by convention. C++ keeps GTSAM out of `graphs.hpp`, and Rust keeps `ffi` private, but Python's public API leaks `gtsam.Similarity3`.
- **Suggested fix:**
  - Add an import-linter contract "Only foundations.graphs may import gtsam" (forbidden, with one `ignore_imports`, as for Pillow).
  - Have `check_layers.py` allow `<gtsam/...>` includes only in `cpp/src/foundations/graphs.cpp`, plus the Rust bridge.
  - Make the two converters private; the tests can reach them as `_to_similarity3`.

#### [should-fix] The rung docstrings do not place the new modules
- **Where:** `src/slam_it/edge/__init__.py`, which lists every module but `loops`, and whose milestone and budget text stops at B2 ("decided in B3's design"); `src/slam_it/foundations/__init__.py`, which lists "SO(3)/SE(3)/Sim(3)" but neither `graphs` nor GTSAM nor Sim(3) RANSAC.
- **Issue:** `CLAUDE.md` makes each package docstring the statement of its rung, milestone and budget. A reader of `slam_it.edge` cannot see that loop closing lives there, or that rung 0 now carries a factor-graph back end.
- **Suggested fix:** Add `loops` (B3, ORB-SLAM3's `LoopClosing`) to the edge index, with loop closing's load in its budget line. Add `graphs` (GTSAM: pose graph, Sim(3) refinement, global BA) and `sim3` (Sim(3) RANSAC) to rung 0's.

#### [note] Everything that links rung 0 now builds GTSAM
- **Where:** `CMakeLists.txt:98` (`target_link_libraries(slam_it_foundations PRIVATE gtsam)`); `rust/slam-it-foundations/build.rs` (needs `SLAM_IT_GTSAM_PREFIX` or `CONDA_PREFIX`, otherwise `exit(1)`; compiles the bridge as C++20 with the cores' flags repeated by hand); `scripts/check.sh` (the bridge gets no clang-tidy).
- **Issue:** `slam_it_eval` / `slam-it-eval` (the PNG and dataset readers), every rung-0 test and benchmark, and the Rust crates now need GTSAM. In Rust they also need a C++ toolchain to build. "Builds standalone, for a possible phone target" now includes cross-building GTSAM.
- **Suggested fix:** This is acceptable as decided. If the phone target or eval-only builds matter later, a default-on cargo feature or CMake option around `graphs` keeps one crate and one library per rung. Weigh it in the ladder's "hand-built parts" item.

#### [note] `_cpp` exports GTSAM's symbols; a test process holds three static copies of GTSAM
- **Evidence:** `nm -D --defined-only` shows 3,511 `gtsam::` symbols exported from `_cpp`, 8,710 from the wrapper, and 0 from `slam_it_rs`.
- **Issue:** Python's default `RTLD_LOCAL` loading keeps the copies apart today. Anything loaded `RTLD_GLOBAL` could bind one copy's calls to another's: in the worker image, PyPI's build against ours.
- **Suggested fix:** Link `_cpp` with `-Wl,--exclude-libs,ALL` (or a version script), so the static GTSAM stays private to the extension, as Rust's `cdylib` already keeps it.

#### [note] The module APIs mirror each other unevenly
- **Where:** `essential_graph` is public in Python (`src/slam_it/edge/loops.py:1248`) and Rust (`rust/slam-it-edge/src/loops.rs:2035`) but internal in C++ (`cpp/src/edge/loops.cpp:1155`). The search settings are bundled as `Window` in C++ and `Search` in Rust.
- **Issue:** The Protocol checks only the dispatched functions, so these differences go unnoticed. Align them when B4 reuses the essential graph.

#### [note] `loops` is about 2,200 lines per language, and B4 reuses half of it
- **Issue:** Detection's stages (`NewDetectCommonRegions`) and the essential graph with global BA are what B4's merge uses again (design section 14, items 3 and 4). When B4 lands, split common-region detection, and perhaps the correction's solves, out of `loops` into modules of their own, rather than growing `loops` or having merging reach into it.

#### [note] Stale C++ header documentation (hygiene reviewer's lane)
- **Where:** `cpp/include/slam_it/foundations/graphs.hpp:219`.
- **Issue:** The header still says a point behind its camera "contributes nothing (zero residual and Jacobians)". Since `57c2f3b`, the implementation returns `2 fx` in each row, as the Python and Rust docs say.

### Checked and fine
- **Dependency direction:**
  - `edge.loops` imports only lower edge modules and `foundations`, and nothing from `eval` or `orchestration`.
  - `backends` is unchanged and still only locates implementations.
  - The `(loops) | (relocalization)` layer and the rung-0 layers `(ba) | (graphs) | (twoview)` and `(... | sim3 | ...)` are right.
  - C++ and Rust mirror them (`check_layers.py` passed in layer 1).
- **Binding boundary:** one file per module (`cpp/bindings/foundations/{graphs,sim3}.cpp`, `cpp/bindings/edge/loops.cpp`, `rust/slam-it-py/src/{foundations/{graphs,sim3},edge/loops}.rs`). Bindings only convert types; float64 arrays and dicts cross with `codes()` tables, and the roots only wire modules.
- **Dispatch and validation:** each new module has an `Implementation` Protocol and `backends.call` dispatch, and validates once at its public entry.
- **Rust bridge:** `cxx/graphs_bridge.{hpp,cpp}` is GTSAM glue only, with no geometry. The evaluator slot is per problem (no globals), `unsafe` is allowed in `mod ffi` alone under the crate's `deny`, and the bridge is covered by clang-format and doxygen.
- **C++:** GTSAM is kept out of `graphs.hpp` and linked PRIVATE; the cores return `std::expected` and `Result` with the shared error kinds.
- **Formats:** run format 7 and bundle 4 are clean breaks. Python, C++ and Rust writers, the reader, the bake and the viewer all moved together. `compare_testsets.py` reads measure-recorded details, not old formats, so there is no shim.
- **Tests follow the owner's decision:** a golden record per language, the loop room's bar for all three backends, the stage comparisons with Python kept, the cross-language whole-run tests removed, and H21 held per language.
- **Design docs:** section 10's as-built notes (GTSAM's own build, factors per language, behind-camera constant, Python's pinhole factors) and the ladder's "Runtime dependencies" were updated with the dependency; scipy left the oracle contract as decided.
- **Budgets:** no GPU or PyTorch; GTSAM is static, single-threaded, without TBB or Boost features; `eval.budgets` reports loop closing's load.
- **Pipeline:** the final trajectory's records in `Pipeline` are scheduled to move to `edge.atlas` in B3b (design section 12, item 1), so this is not scaffolding hardening.

**Disclosure:** to count exported symbols I imported `slam_it._cpp`, which triggered the editable install's automatic rebuild into the git-ignored `build/slam-it`. No tracked file changed; the working tree is clean at `5d857bb`.

### Algorithm review


**Scope:** `28c80dc9...5d857bbb` (B3, loop closure). I read these in full: `src/slam_it/foundations/{sim3,graphs}.py`, `src/slam_it/edge/loops.py`, and the B3 diffs of `edge/{database,covisibility,map,mapping,pipeline}.py` and `scripts/measure_testset.py` (the loop judge). I also read `cpp/src/foundations/graphs.cpp`, the C++/Rust `loops` code at every place where similarities are composed or a threshold decides, and the tests (`tests/foundations/test_{graphs,sim3}.py`, `tests/edge/test_loops.py`, `tests/orchestration/test_loop_room.py`, `generate_sim3.py`). Design §10–11 and the test-set note's B3 section were the brief.

References I compared against:
- ORB-SLAM3's source: `LoopClosing.cc`, `KeyFrameDatabase.cc`, `Sim3Solver.cc`, `Optimizer.cc`.
- g2o's `EdgeSim3` and Levenberg–Marquardt.
- GTSAM 4.3.0's own source in `build/gtsam-4.3.0/src`.
- The ORB-SLAM3 paper (T-RO 2021), whose Table II and §VIII.A I read in the PDF text.

Evidence I gathered:
- `pytest tests/foundations/test_graphs.py tests/foundations/test_sim3.py`: 69 passed.
- Probe scripts in the session scratchpad, three of them kept in a private scripts folder.
- Importing `slam_it` set off the editable install's rebuild check. It ran a no-op rebuild in the git-ignored `build/slam-it` ("ninja: no work to do"). `git status` is still clean.

**Summary:** The Python reference follows ORB-SLAM3 closely, and its listed departures from ORB-SLAM3 are correct readings of bugs in that source. I re-derived the conventions (similarities, the right-perturbation Jacobians on GTSAM's charts, g2o's error terms against GTSAM's `BetweenFactor`) and checked them numerically; they are right. The thresholds match ORB-SLAM3's hard-coded values, and the published figures in the bar match the papers. There is one must-fix: input validation in `graphs.refine_similarity`. There is one should-fix: global BA's LM damping does not scale with the problem, which made global BA a no-op on advio-13. I reproduced that mechanism.

### Findings

#### [must-fix] `graphs.refine_similarity` accepts infinite or half-NaN rows of `pixels2`
- **Where:** `src/slam_it/foundations/graphs.py:863-868`
- **Issue:** `np.nan_to_num(raw2)` runs *before* `as_batch` checks for finite values, and only rows that are all NaN are put back to NaN afterwards. So:
  - a row `[nan, v]` becomes `[0, v]`, a real observation at column 0;
  - `inf` becomes ±1.8e308.
  The compiled backends receive the converted array, so C++'s own check in `refine_similarity` (`pixels2.isFinite().rowwise().all() || unseen`) never sees the bad values. All three backends take the input without complaint.
- **Why it matters:** This is a public rung-0 boundary whose contract is "a row may be NaN: image 2 does not observe that match". A partial NaN or an infinity is silently turned into a wrong observation, with an overflow and an infinite chi² instead of an `InvalidInputError`. Loop closing only passes whole-NaN rows, so no measured run is affected.
- **Evidence:** `refine_similarity_pixels2_validation.py`, with row 3 of `pixels2` set to `[nan, v]`:
  ```
  python accepted; chi2 row 3: [ 1383.61914531 29382.23666089]
  cpp accepted; chi2 row 3: [ 1383.61914531 29382.23666089]
  rust accepted; chi2 row 3: [ 1383.61914531 29382.23666089]
  ```
  With `[inf, v]`, all three accept it and return chi² `[101.55, inf]`; Python also warns "overflow encountered in matmul".
- **Suggested fix:** Before any conversion, refuse a row that is neither all finite nor all NaN, as the C++ core does. Add both cases, for every backend, to `test_bad_graphs_are_refused`.

#### [should-fix] Global BA gives up at GTSAM's damping limit on a single near-plane observation: the LM damping does not scale with the problem
- **Where:**
  - Python: `src/slam_it/foundations/graphs.py:479-514` (`_solve`)
  - C++: `cpp/src/foundations/graphs.cpp:269-290`
  - Rust: `rust/slam-it-foundations/cxx/graphs_bridge.cpp:269-275`
  - The caller is `src/slam_it/edge/loops.py:1306-1325` (`_global_bundle_adjust`).
- **Issue:** The solves use GTSAM's LM defaults, all in absolute terms:
  - λ₀ = 1e-5;
  - λ multiplied by 10 per failed try, up to `lambdaUpperBound` = 1e5;
  - damping λ·I, since `diagonalDamping` is off.

  A BA's Hessian diagonal for poses is about f²·Σinfo, between 1e6 and 1e9. Even at the bound, the damping barely shortens a Gauss–Newton step. When one point sits just in front of a camera's image plane, which can happen after the correction moves points with their reference keyframes, no step that LM can try lowers the error. LM hits the bound in its first iteration and global BA returns the map unchanged.

  ORB-SLAM's global BA runs on g2o's LM, which starts at λ = τ·max diag(H) (τ = 1e-5) and multiplies λ by a growing ν on each failure, so its damping follows the problem's scale. (Madsen, Nielsen & Tingleff, *Methods for Non-Linear Least Squares Problems*, 2004, §3.2; Kümmerle et al., "g2o", ICRA 2011.) GTSAM offers the same remedy as Marquardt's diagonal scaling, `LevenbergMarquardtParams::setDiagonalDamping`.
- **Why it matters:** One bad observation turns global BA into a no-op, with no robust loss there to absorb it. The summary does record the outcome (`damping_limit`, not converged), so the failure is observable.
- **Evidence:**
  - The test-set note on advio-13: global BA "started from an error of 1.4 × 10^26 … and stopped after one iteration at its damping limit, leaving the map as the essential graph had."
  - Reproduced with `gtsam_ba_near_plane_damping.py`. The setup is 6 cameras and 200 points with 1 px noise. One point is placed `z` metres in front of camera 5's image plane, which puts it behind the other five cameras, so those observations cost the constant.

  | z (m) | `graphs.bundle_adjust` | GTSAM, `setDiagonalDamping(True)` | GTSAM, upper bound 1e20 |
  |---|---|---|---|
  | 1e-2 | 1.081e9 → 1.081e9, 1 iteration, `damping_limit` | 1.081e9 → 4.368e6, 10 iterations | → 6.403e7 |
  | 1e-4 | 1.059e13 → unchanged, `damping_limit` | → 4.919e11, 3 iterations | no change |
- **Suggested fix:**
  1. Turn on diagonal (Marquardt) damping, or derive λ₀ from max diag(H) and lift the upper bound, in all three languages. Measure the change on the loop sequences, since it changes decisions and the records.
  2. Optionally, the owner can decide whether global BA should leave out observations not in front of their camera by a margin and report how many it left out. ORB-SLAM's local BA drops edges that fail `isDepthPositive()`; its global BA does not.
  3. Add a rung-0 test in which a BA with one near-plane observation still lowers the error.

#### [note] In the Sim(3) refinement, a point carried behind its camera costs nothing
- **Where:** `src/slam_it/foundations/graphs.py:466-469`, mirrored in C++ and Rust.
- In the refinement's second round, which has no robust loss, a step that carries an inlier behind its camera lowers that match's cost to 0. This is the same incentive commit 57c2f3b removed from BA. The effect is self-limiting: that match's chi² becomes infinite and it drops out of the inlier count.
- The choice is deliberate, per that commit. Consider either GTSAM's constant (2·fx per row, no gradient) here too, for one convention across rung 0, or a sentence in the docstring saying why the two differ.

#### [note] GTSAM 4.3's `BetweenFactor<Similarity3>` Jacobian is only first-order
- `Similarity3::ChartAtOrigin::Local` has no Jacobian overload. So `HasLocalJacobians<Similarity3>` is false, and `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR` (on by default) has no effect for it. The monocular essential graph therefore linearises with the plain between-Jacobians. `Pose3` is exact. ORB-SLAM's g2o `EdgeSim3` differentiates numerically, so its Jacobian is exact.
- Measured with `gtsam_sim3_between_jacobian.py`:
  - at |e| = 0.13 and 0.79, the maximum Jacobian error is 0.087 and 0.52 for Sim(3), against 1e-10 for Pose3.
  - On a 30-node Sim(3) ring with an exact loop edge, the solutions with approximate and exact (numeric `CustomFactor`) Jacobians agree within 2.7 µm, and within 24 µm with 2.4× scale and 20° rotation drift.
- This is negligible in practice. It is worth a sentence in `graphs.py`.

#### [note] Loop fusion can merge two of the loop group's points: a departure not in the docstring's list
- **Where:** `src/slam_it/edge/loops.py:1122-1128`
- `_fuse_by_similarity` replaces points immediately. If a keypoint has just taken loop point A in this pass, a later loop point B that projects onto the same keypoint runs `replace_point(lose=A, keep=B)`, so two group points are merged.
- ORB-SLAM's `SearchAndFuse` defers its replacements (`vpReplacePoint`). The keyframe's original point is replaced by A, and B's later `Replace` on that already-bad point does nothing, so A and B are never merged.
- This is rare, but it is undocumented. Either skip keypoints already fused in this pass, or add it to the docstring's list of departures.

#### [note] Measurement protocol against the published figures
- ORB-SLAM3's EuRoC values are "the median after 10 executions" (§VIII.A).
- Its pure-visual results rely on Atlas multi-map merging; V1_03 monocular is named as solved thanks to it.
- The bar compares a single seed-0 run, without merging (which arrives in B4). Several of the "<95% tracked" misses may be what B4 addresses.
- The Published column itself matches Table II exactly: monocular 0.016/0.027/0.028/0.138/0.072/0.033/0.015/0.033 and stereo 0.029/0.019/0.024/0.085/0.052/0.035/0.025/0.061.
- Section 11 should state the protocol difference.

#### [note] The loop judge's "poor pose" measures something different from the design's wording
- Design §10 says "the two keyframes' relative pose after the correction".
- `scripts/measure_testset.py:273-331` judges the final keyframe poses at the run's end, which include every later local BA and loop. Its docstring says so.
- Align the design's wording with the code, or record the poses right after each correction.

#### [note] The loop room's RGB-D and stereo bounds are 3–5× looser than monocular's on exact synthetic depth
- **Where:** `tests/orchestration/test_loop_room.py`
- The final-ATE bounds are 7 cm (RGB-D) and 5.5 cm (stereo) against 2 cm (monocular); the bounds on the points' distance to the walls are 11 cm against 6.5 cm.
- The rendered depth is exact, and `LOOP_PITCH`'s docstring records RGB-D tracking jumping by 3° when looking level at a wall.
- With bounds this loose, the rooms cannot catch a regression on the depth path. Backlog: find the cause (it is outside B3's goal).

#### [note] State of the art: guarding against loops at the wrong place
- advio-13's wrong-place loop passed every ORB-SLAM3 check, including three confirmations at once. ORB-SLAM's temporal consistency is the only guard here.
- Back ends now usually reject inconsistent loops themselves:
  - Graduated Non-Convexity (Yang, Antonante, Tzoumas & Carlone, RA-L 2020, arXiv:1909.08605), which GTSAM ships as `GncOptimizer`;
  - PCM (Mangelson et al., ICRA 2018).
- Retrieval has moved to learned global descriptors (Berton & Masone, "MegaLoc", CVPR 2025, arXiv:2502.17237).
- DPV-SLAM (Lipson, Teed & Deng, ECCV 2024, arXiv:2408.01654) is the learned-edge counterpart: proximity loops plus classical retrieval with a pose graph that corrects scale drift.
- This is context for later rungs, not a defect in this classic rung.

#### [note] Smaller items, some outside my lane
- **Missing parity fixture (architecture reviewer's lane):** `foundations.graphs` has C++ and Rust implementations but no parity fixture in `tests/fixtures/foundations/`, which CLAUDE.md asks for. The known-answer tests per backend, and the native central-difference Jacobian tests in C++ and Rust, do cover correctness.
- **Docstring wording:** `graphs.bundle_adjust` says a point seen once is "constrained only along its ray". It is the reverse: such a point is free along its ray and constrained across it.

### Checked and fine
- **Sim(3) basics:** `sim3.inverse`, `scale_of` and `rigid_of` are right. The conversion to GTSAM uses `t' = t/s` for `Similarity3`'s `s(Rp + t')`, and a test checks it against `transformFrom`.
- **Factor Jacobians:**
  - Projection and similarity Jacobians use right perturbation in GTSAM's tangent order: (ω, v) for poses, (ω, v, λ) for similarities. I derived `d_forward = sR[-[p]×, I, p]` and `d_backward = [[b]×, -I, -b]` by hand from GTSAM's wedge.
  - The right-column row `+bf/z²` is correct.
  - All of these are checked by central differences in all three languages.
- **Sim3Solver:**
  - The inlier test is 9.21·σ² in both images, with point projections standing in for observations; residuals are squared against a squared threshold with a strict `<`.
  - `ransac_budget` equals `SetRansacParameters`.
  - Umeyama with scale gives the same answer as Horn's (asymmetric) scale for the same rotation.
  - Samples are seeded.
- **ORB-SLAM3 thresholds:** 20/15/50/20/80; windows 8/5/3/4; distances TH_LOW × 1.5 and × 1.0; confirmations 30, then 50 (strictly more), then 100; three confirmations close a loop; two misses drop it.
- **OptimizeSim3:** Huber at √10, 5 iterations; matches with chi² > 10 in either image dropped; fewer than 10 left fails; then 10 or 5 iterations with no kernel, inliers counted the same way. A match that image 2 does not observe gets no error there; ORB-SLAM3 compares normalised coordinates with pixels for it.
- **NewDetectCommonRegions:** the loop being confirmed is carried as `T_cl · S_lw`; a candidate with zero confirmations is not carried; a new candidate replaces the old one; the best second search wins.
- **DetectNBestCandidates:** keyframes sharing more than 0.8·max words; scores accumulated over the best 10 *scored* links (ORB-SLAM3 reads stale scores here); the best-scored keyframe stands for its group; top 3; linked keyframes left out.
- **The listed departures:** each one I checked against ORB-SLAM3's source is a genuine bug fix: the inverted `bDifferentKFs` in Sim3Solver, `OptimizeSim3` given `pKFi`, the unrefined `gScw_estimation`, and the group widening.
- **CorrectLoop:** `S_iw = T_iw · T_wc · S_cw`; points carried by `S_corr⁻¹ · T_iw`; poses set to `(R, t/s)`; matched points replace or join the keyframe's; the loop's new links exclude previous neighbours and the moved keyframes; the loop edge is added after the graph.
- **Essential graph:**
  - Each edge measures `S_iw · S_wj`, and GTSAM's `BetweenFactor` error, written out, is the same group element g2o's `EdgeSim3` takes the log of.
  - Unit information; the matched keyframe fixed through `NonlinearEquality` (exact, under test); λ₀ = 1e-16; 20 iterations.
  - The edge selection matches `OptimizeEssentialGraph`, and points move with their corrected or reference keyframe, whose reference stays valid through `refresh_points`.
- **Global BA:**
  - No robust loss; keyframe 0 fixed (never culled); 10 iterations.
  - The behind-camera constant matches `GenericProjectionFactor` and `GenericStereoFactor`, and `Cal3_S2Stereo` with b = bf/fx gives `u - bf/z`.
  - The free monocular scale gauge does not wander: over 6 seeds, each converged in 4 iterations with the scale within 1%.
- **Final trajectory:** follows `SaveTrajectoryTUM`. A keyframe's own frame is placed relative to it with the identity, a culled keyframe keeps `T_c_parent` (`mTcp`), the chain through culled ancestors is right, reset maps are dropped, and the last frame moves with its keyframe after a correction while the motion model's velocity, being relative, stays valid.
- **LM driver:** stops by the same rule in all three languages and records non-convergence (`damping_limit`, `max_iterations`) in the run's diagnostics.
- **Loop judge:**
  - The viewing-direction angle compares the z-columns of the true `T_world_cam`.
  - EuRoC's ground truth is carried to the left camera (`T_world_body · T_body_cam0`), as ORB-SLAM3's `EuRoC_left_cam` ground truth is.
  - ATE alignment is Sim(3) for monocular and SE(3) otherwise, as ORB-SLAM3's §VIII states.
  - The relative-pose errors are computed correctly.
- **Tests:** synthetic loops both rigid and in scale; the stage comparisons between backends; the loop room's bar in all three languages.

Sources:
- [ORB-SLAM3 (arXiv:2007.11898)](https://arxiv.org/pdf/2007.11898)
- [Graduated Non-Convexity (arXiv:1909.08605)](https://arxiv.org/pdf/1909.08605)
- [GTSAM GncOptimizer docs](https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/GncOptimizer_8h.html)
- [MegaLoc (arXiv:2502.17237)](https://arxiv.org/pdf/2502.17237)
- [Deep Patch Visual SLAM (ECCV 2024)](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/272_ECCV_2024_paper.php) / [arXiv:2408.01654](https://arxiv.org/pdf/2408.01654)

Files:
- `src/slam_it/foundations/graphs.py`
- `src/slam_it/edge/loops.py`
- `cpp/src/foundations/graphs.cpp`
- `rust/slam-it-foundations/cxx/graphs_bridge.cpp`
- `scripts/measure_testset.py`
- `refine_similarity_pixels2_validation.py`
- `gtsam_ba_near_plane_damping.py`
- `gtsam_sim3_between_jacobian.py`
