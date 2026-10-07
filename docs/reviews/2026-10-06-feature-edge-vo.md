# Review panel: `feature/edge-vo`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-06

| | |
|---|---|
| **Branch** | `feature/edge-vo` |
| **Range** | `6c04d7bf8c2d3ad84bb56dd5904d0f6e80c1968e...6151c45ba8fa0467ce2f1bb246cc98cc8c37a19f` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`): 2814 pytest, 32 ctest, every cargo test |

### Findings and resolutions

Duplicates across reviewers are cross-referenced ("same as") and resolved together.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A1 | must-fix | architecture | `tests/fixtures/` (no `edge/`), `tests/edge/conftest.py`, `CLAUDE.md:72` | The edge rung has no parity fixture: every cross-language check compares with the Python reference computed live, so a change made in all three languages at once passes | fix: (a), document the live-parity policy and commit a golden record of the room runs | fixed (09eb6a2): option (a), the documented live-parity policy and a golden record of the room runs -- decided by evan-wes, 2026-10-06 |
| H1 | should-fix | hygiene | `options.py:85`, `options.hpp:86`, `options.rs:94` | `max_frames` documented as a keyframe-interval cap; it only sets the post-recovery window (same as L5) | fix | fixed (eb74c18) -- decided by evan-wes, 2026-10-06 |
| H2 | should-fix | hygiene | `frames.py:152`, `frames.py:314` | `Frame` has no validation; the reference and the natives fail differently (raw `ValueError` vs a message) on a malformed frame | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H3 | should-fix | hygiene | `pipeline.py:111` | `Pipeline`'s private and derived state are constructor arguments | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H4 | should-fix | hygiene | `tracking`, `mapping` in three languages | ORB-SLAM constants unnamed (0.8/1.2, 2.5/4.0, 0.8/0.75 ratios, 2-or-3 observations); the in-view test written twice per language | fix | fixed (ace10d7) -- decided by evan-wes, 2026-10-06 |
| H5 | should-fix | hygiene | `tracking`/`mapping` in three languages, `options.cpp:61` | Small helpers written twice within a language (`_check_mode`, `_local`/`to_local`, `centre_of`, inlined centres, redundant `nan_to_num`, two identical C++ check arrays) | fix | fixed (aee51bf, ace10d7) -- decided by evan-wes, 2026-10-06 |
| H6 | should-fix | hygiene | `tracking.rs:447,560,672`, `tracking.cpp:339,408,477`, `initialization.cpp:156` | Native stage functions of 70-95 lines doing build, search and refine in one body | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| H7 | should-fix | hygiene | `test_runs.cpp:143`, `test_vo.cpp:107`, `vo.rs:784`, `runs.rs:533`, `tum.rs:512`, ... | Native tests use fixed scratch folders under the shared temp dir | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H8 | should-fix | hygiene | `scripts/run_vo.py:88` | A malformed ground truth reads as "no ground truth" | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H9 | should-fix | hygiene | `frames.cpp:105`, `frames.rs:273` | The native frame builders have no native tests; their refusals are untested | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H10 | should-fix | hygiene | `slamlab_vo.cpp:44`, `vo.rs:96`, `vo.py:70` | The exit-code contract is tested for `InvalidInput` only; C++ relies on `ErrorKind`'s order (same as A9) | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H11 | should-fix | hygiene | `tests/edge/conftest.py:191` | `run_room` uses `backend="cpp"` without `require_backend` | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H12 | should-fix | hygiene | `slamlab-eval/src/lib.rs:1`, `CLAUDE.md:37` | Stale descriptions of `eval` and the languages table | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H13 | should-fix | hygiene | `tum.cpp:163,171`, `tum.rs:264` | Native `camera.txt` and number-parse errors do not name the bad value | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H14 | note | hygiene | `map.rs:547`, `map.cpp:103` | Unchecked `refresh` helpers would mask a violated precondition | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| H15 | note | hygiene | `map.py:547`, `map.cpp:272` | `Map` does not check the count invariants `cull_points` divides by | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| H16 | note | hygiene | `pipeline.py:137`, `pipeline.hpp:96`, `pipeline.rs` | `last_tracked` has no caller in any language | fix (drop it) | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| H17 | note | hygiene | `tracking.py:170`, `pipeline.py:232`, `mapping.py:745` | Python `assert` for narrowing and as a postcondition | waive: the repo's style (rung 0 does the same) | waived: the repo's style (rung 0 asserts the same way) -- decided by evan-wes, 2026-10-06 |
| H18 | note | hygiene | `cpp/tests/edge/`, `edge-vo.md` §2 | Test layout and the design doc's stale module list (same as A12) | fix the design doc; defer the test layout | fixed (aee51bf): the design doc's module list; the test layout deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| H19 | note | hygiene | -- | Backlog: the remaining keyframe-time growth on MH_01; Python's per-point loops | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| A2 | should-fix | architecture | `pipeline.py:43`, `tracking.py:45`, `mapping.py:46`, `initialization.py:36`, `vo.py:58` (and C++/Rust copies) | Thirteen thresholds and two factors live outside `Options`: not recorded in a run, not settable, defined three times; `RGBD_BF` belongs to the sensor | fix (decision: what moves, where `bf` lives) | fixed (ace10d7): the thresholds in `Options`; `bf` a `camera.txt` key, the TUM sensors' 40 from the readers' table -- decided by evan-wes, 2026-10-06 |
| A3 | should-fix | architecture | `tests/orchestration/test_vo.py:52` | No layer-1 test runs the pipelines' recovery path in more than one backend | fix | fixed (09eb6a2) -- decided by evan-wes, 2026-10-06 |
| A4 | should-fix | architecture | `mapping.py:139`, `edge_crossing.hpp:418`, ladder doc :297 | `mode` crosses by name, against the design doc's convention for argument choices | fix by amending the design doc and the C++ comment | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A5 | should-fix | architecture | `backends.py:52,240`, `check.sh:85`, `setup_slamlab.sh:107` | The Rust program's location depends on cargo's environment; nothing rebuilds it before pytest | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A6 | should-fix | architecture | `tum.cpp:27`, `tum.rs:33`, `generate_sequences.py` | No fixture holds the native readers' freiburg2 and freiburg3 calibrations | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A7 | note | architecture | `map.py:639` | `frame_dict` lives in `edge.map`, not with `Frame` | fix | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A8 | note | architecture | `frames.py:38` | Backlog (milestone 4): `edge.frames.Camera` is a candidate for rung 0 | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel"; milestone 4) -- decided by evan-wes, 2026-10-06 |
| A9 | note | architecture | `slamlab_vo.cpp:43` | Exit codes defined three ways (same as H10) | fix with H10 | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A10 | note | architecture | `vo.py` | `vo.run` raises `FileNotFoundError` in Python and `InvalidInputError` natively for a missing file | waive: the documented `eval` reader decision | waived: the documented decision for `eval`'s readers -- decided by evan-wes, 2026-10-06 |
| A11 | note | architecture | `edge::search`, `svd3` | Backlog (PR 9): the feature grid rebuilt per call; the allocating `svd3` | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel"; PR 9) -- decided by evan-wes, 2026-10-06 |
| A12 | note | architecture | `edge-vo.md:86` | The design's §2 module list is stale (same as H18) | fix with H18 | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| A13 | note | architecture | `options.cpp:48` | C++ `check_options` lists the fields by hand | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| L1 | should-fix | algorithm | `tracking.py:232`, `pnp.py:478` | Pose refinement drops PoseOptimization's per-level information weighting (+24% centre error in a Monte Carlo); Cauchy in all rounds and left image only are undocumented | fix (decision: weight it, or document it) | fixed (87acf3d; the note, c0f8289): rung 0's `refine_pose` weighted by level in three languages; Cauchy and left image only listed as lite departures -- decided by evan-wes, 2026-10-06 |
| L2 | should-fix | algorithm | `edge-vo.md:175`, `initialization.py:238`, `twoview.py:167` | The note misexplains MH_01's late start (two-view succeeds from frame 6; the 100-point rule refuses); initialisation departs from ORB-SLAM (kept points, 100 vs 50, a fixed search window) | fix (decision: which rules) | fixed (9c521de; the note, c0f8289): ORB-SLAM2's rules (points above 0.36 deg, matches followed, 100 points); the note corrected -- decided by evan-wes, 2026-10-06 |
| L3 | should-fix | algorithm | `edge-vo.md:171,201` | The DSO comparison is not like for like; ORB-SLAM3's numbers are medians of ten; the fr1/desk row's count | fix | fixed (c0f8289) -- decided by evan-wes, 2026-10-06 |
| L4 | should-fix | algorithm | `pipeline.py:224,235` (and C++/Rust) | After a failed frame the stale velocity still predicts; the code's comment and the note say no motion | fix (clear it on failure) | fixed (c22e30e) -- decided by evan-wes, 2026-10-06 |
| L5 | should-fix | algorithm | `options.py:50,85`, `pipeline.py:43` | Setting docstrings misstate ORB-SLAM (`max_frames`, `min_depth_features`, `RECOVERY_WIDEN`) (same as H1) | fix | fixed (eb74c18) -- decided by evan-wes, 2026-10-06 |
| L6 | note | algorithm | `pipeline.py:276`, `vo.py:337` | The last frame is not re-anchored after local BA; runs hold tracking-time poses, so every-frame ATE is stricter than ORB-SLAM's protocol | add a keyframe-trajectory column; defer re-anchoring | fixed (c0f8289): a keyframe-trajectory ATE column; re-anchoring deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| L7 | note | algorithm | `adjust.py:200`, `mapping.py:881` | Local BA's loss and depth model differ from ORB-SLAM's stereo edge; BA runs at two keyframes where ORB-SLAM waits for three | document; fix the three-keyframe rule | fixed (eb74c18; documented, c0f8289): local BA from three keyframes -- decided by evan-wes, 2026-10-06 |
| L8 | note | algorithm | `edge-vo.md:150` | The EuRoC runs used TUM's `close_depth` and `max_frames`, not ORB-SLAM2's EuRoC values (35, 20) | fix (in the results runs) | fixed (c0f8289) -- decided by evan-wes, 2026-10-06 |
| L9 | note | algorithm | `adjust.py`, `tracking.py` | Solve summaries are discarded | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| L10 | note | algorithm | `homography.py:69` | The homography condition margin is evidenced only in a docstring | fix (a property test) | fixed (aee51bf) -- decided by evan-wes, 2026-10-06 |
| L11 | note | algorithm | `test_twoview.py:248` | Backlog: why the two-view fixture has duplicate matches | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel") -- decided by evan-wes, 2026-10-06 |
| L12 | note | algorithm | -- | State of the art: ORB-SLAM3, DPVO, DPV-SLAM, MASt3R-SLAM are later rungs | no action | no action -- decided by evan-wes, 2026-10-06 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 6c04d7b...6151c45` (BASE_SHA..HEAD_SHA), repo the repository root. Read in full: all of `src/slamlab/edge/*.py`, `src/slamlab/orchestration/vo.py`, `scripts/run_vo.py`, the `backends.program` / `eval.tum` / `eval.euroc` diffs, the design doc and `docs/notes/edge-vo.md`, the CHANGELOG, and the edge/orchestration/eval tests. Read most of the C++ core (`options`, `map`, `frames`, `initialization`, `tracking`, `edge_crossing.hpp`, `eval/runs`, `eval/tum`, `eval/text`, `orchestration/vo`, `apps/slamlab_vo`) and Rust (`slamlab-edge` `options`, `pipeline`, `adjust` and most of `tracking`; `slamlab-orchestration`). The rest of `mapping`, `adjust` and `pipeline` in C++ and `map`/`mapping` in Rust, and the PyO3 bindings, were only grepped. I ran two targeted probes outside the repo and the working tree stayed clean. Layer-1 items (format, lint, docstring presence, jscpd) are not re-reported.

**Summary:** Production quality is high. Boundaries validate and name the bad value, there are no swallowed exceptions or `unwrap`/`expect` in non-test Rust (checked by script), no debug prints or stray TODOs, and every `#[allow]`/`NOLINT` carries a reason. Every `Options` setting is live in code. Parity tests assert on values and cover edge cases. I found **no must-fix**. The should-fix items are mostly one documentation lie (`max_frames`), one inconsistency at the `Frame` boundary, unnamed ORB-SLAM constants and helpers written twice within a language, a few test gaps, and a few stale docs. No rig-specific data appears in the diff (privacy grep clean; the fixture intrinsics are TUM and EuRoC public values).

### Findings

#### [should-fix] `Options.max_frames` is documented as a keyframe-interval cap, but only governs the post-recovery window
- **Where:** `src/slamlab/edge/options.py:85-87`, `cpp/include/slamlab/edge/options.hpp:86`, `rust/slamlab-edge/src/options.rs:94`. Actual uses: `src/slamlab/edge/pipeline.py:256,272`, `cpp/src/edge/pipeline.cpp:201,215`, `rust/slamlab-edge/src/pipeline.rs:409,440`.
- **Issue:** All three docs say "A keyframe is due at the latest this many frames after the last (`mMaxFrames`: the frame rate)". Nothing forces a keyframe after `max_frames`. The setting is only the "recent recovery" window (`since_recovery < max_frames`) and the "hold keyframes until the map has more than `max_frames`" rule. `docs/notes/edge-vo.md` itself says the frame-count conditions "always hold, so these decide".
- **Why it matters:** Someone tuning `--option max_frames=N` to force keyframes gets a different behaviour, and the three docs contradict the code in lock-step.
- **Evidence:** `grep -n max_frames src/slamlab/edge/*.py cpp/src/edge/*.cpp rust/slamlab-edge/src/*.rs` shows only the recovery uses above.
- **Suggested fix:** Reword in all three languages to what it does (relocalisation hold-off window, ORB-SLAM's `mnLastRelocFrameId + mMaxFrames`). Consider renaming it `recovery_frames`.

#### [should-fix] `Frame` is the one core dataclass with no validation, so the reference and the natives fail differently on a malformed frame
- **Where:** `src/slamlab/edge/frames.py:152-181` (no `__post_init__`); `frames.py:314-325` (`rgbd` extracts ORB features before checking the depth map's shape and the baseline); compare `cpp/bindings/edge/edge_crossing.hpp:316-320` and `cpp/src/edge/frames.cpp:120-127`, which check first.
- **Issue:** `Map`, `Camera`, `Options` and `Projections` all validate at construction. `Frame` does not, although it is the input of every stage and of `Pipeline.track`.
- **Why it matters:** For a hand-built frame with 595 depth rows and 600 keypoints, `initialization.initialize_from_depth(bad, Options())` raised a raw `ValueError: operands could not be broadcast together with shapes (595,) (600,)`. With `backend="cpp"` and `"rust"` it raised `ValueError: frame: expected one row per keypoint in every array`. The same call gives different messages, and neither is the `InvalidInputError` that the error contract promises.
- **Evidence:** Probe script run under `conda run -n slamlab`; output as quoted above.
- **Suggested fix:** Add `Frame.__post_init__` checking the lengths of `bearings`, `depth`, `right`, the descriptors and the keypoints, and their dtypes. In `rgbd`, validate before `orb.extract`, as the C++ version does.

#### [should-fix] `Pipeline` exposes its private and derived state as constructor arguments
- **Where:** `src/slamlab/edge/pipeline.py:111-122`.
- **Issue:** `@dataclass` makes `state`, `map`, `_last_frame`, `_last_tracked`, `_velocity`, `_reference`, `_failures` and `_since_recovery` init parameters. Evidence from `inspect.signature(Pipeline.__init__)`: `(self, mode, camera, options=<factory>, state=<State.NOT_INITIALIZED>, map=None, _last_frame=None, _last_tracked=None, _velocity=None, _reference=None, _failures=0, _since_recovery=None)`.
- **Why it matters:** `Pipeline("rgbd", cam, state=State.TRACKING)` builds an inconsistent pipeline (tracking with no map). It then fails on `assert self.map is not None` in `_track`. The native pipelines construct only through `create`/`new`, so this is a Python-only wart.
- **Suggested fix:** `field(init=False, repr=False)` for the private fields, and for `state` and `map`. Tests that poke `run._reference` (`tests/edge/test_pipeline.py:80`) would then use a small accessor or observable behaviour.

#### [should-fix] ORB-SLAM constants are unnamed, and the "in view" window test is written twice per language, six sites in all
- **Where:**
  - 0.8 and 1.2 distance margins: `src/slamlab/edge/tracking.py:331-332` and `mapping.py:658`; `cpp/src/edge/tracking.cpp:256-257` and `mapping.cpp:481-482`; `rust/slamlab-edge/src/tracking.rs:291-292` and `mapping.rs:875-876`.
  - 2.5 and 4.0 window factors: `tracking.py:573`, `tracking.cpp:435`, `tracking.rs:593-597`.
  - Ratio tests: 0.8 at `tracking.py:583`, `tracking.cpp:451`, `tracking.rs:616`; 0.75 at `tracking.py:677`, `tracking.cpp:499`, `tracking.rs:692`.
  - Other literals: `mapping.py:202` (`2 if m.keyframes <= 2 else 3`), `tracking.py:249` (`< 3` usable matches), `tracking.py:244` (Cauchy scale `sqrt(chi2_mono)/focal`, no comment on why).
- **Issue:** Docstrings explain the values ("0.8 of its minimum and 1.2 of its maximum", "ORB-SLAM's `RadiusByViewingCos`"), but the code has bare literals. `tracking._in_view` and `mapping._fuse_target` (and their C++/Rust twins) each re-implement "distance within [0.8 d_min, 1.2 d_max] and view cosine at least the limit".
- **Why it matters:** Changing a margin means 6 edits in 3 languages, with only the parity tests to catch a miss. The design doc says "every threshold above, named", yet these few are not.
- **Suggested fix:** Module constants (`DISTANCE_MARGIN_NEAR=0.8`, `DISTANCE_MARGIN_FAR=1.2`, `NARROW_WINDOW=2.5`, `WIDE_WINDOW=4.0`, `LOCAL_MAP_RATIO`, `RELOCALIZE_RATIO`, `MIN_OBSERVATIONS_FEW_KEYFRAMES`) and one shared "matchable distance and angle" helper per language.

#### [should-fix] Small helpers written twice within one language
- **Where:**
  - Python: `_check_mode` in `src/slamlab/edge/tracking.py:195` and `mapping.py:147` (also `_local` at `tracking.py:227` and `mapping.py:177`, and `_rows` in `map.py:268` and `mapping.py:187` with different meanings).
  - Camera centre `-R^T t` is inlined at `tracking.py:329,451` and `mapping.py:654`, although `Map.centres()` (`map.py:209`) exists.
  - `np.nan_to_num(pixels, nan=-1.0)` before `Camera.inside` (`tracking.py:328,440`) is redundant: `inside` already returns False for NaN with no warning (checked under `python -W error`).
  - Rust: `local` (`rust/slamlab-edge/src/tracking.rs:59`) vs `to_local` (`mapping.rs:50`); `centre_of` (`tracking.rs:72`) vs `Map::centre` (`map.rs:211`).
  - C++: `local` (`cpp/src/edge/tracking.cpp:35`) vs `to_local` (`mapping.cpp:39`); `centre_of` (`tracking.cpp:42`) vs `centre` (`map.cpp:319`).
  - C++ `check_options` (`cpp/src/edge/options.cpp:61-96`): the arrays `counts` and `more` run the identical at-least-1 check.
- **Why it matters:** The classic way two copies drift: for example, one frame convention fix applied to one copy of `R X + t`.
- **Suggested fix:** Put the pose application and centre in one place per language (`Map` or `frames`), one `options.check_mode`, and merge the two C++ arrays.

#### [should-fix] Native stage functions are 70-95 lines doing build, search and refine in one body
- **Where:** `rust/slamlab-edge/src/tracking.rs:447` (`track_last_frame`, 86 lines), `:560` (`track_local_map`, 94), `:672` (`relocalize_locally`, 71); `cpp/src/edge/tracking.cpp:339`, `:408`, `:477` (68-71 each); `cpp/src/edge/initialization.cpp:156` (`initialize_monocular`, 87).
- **Issue:** Each runs the phases validate, project and select candidates, build projections, search, refine and bookkeep inline. Python's longest is 50 lines (`mapping._triangulate`), so the natives are the outliers.
- **Suggested fix:** Extract `project_last_frame_points`, `local_map_projections` and `placed_by_pnp` helpers while keeping the public stage functions, so stage parity is unaffected. Fine to defer to the backlog.

#### [should-fix] Native tests use fixed scratch folders under the shared temp dir
- **Where:** `cpp/tests/eval/test_runs.cpp:143,167`, `cpp/tests/eval/test_sequences.cpp:130`, `cpp/tests/orchestration/test_vo.cpp:107`, `rust/slamlab-orchestration/src/vo.rs:784`, `rust/slamlab-eval/src/runs.rs:533,570`, `rust/slamlab-eval/src/tum.rs:512` (names like `slamlab_test_vo`, `slamlab_rust_test_runs`).
- **Issue:** The names are constant. Two layer-1 or offline-CI runs at once (several sessions share this machine) write and `remove_dir_all` the same folders. A failed assertion also leaves the folder behind, and the Rust `vo.rs` test never pre-cleans.
- **Suggested fix:** Per-process unique dirs (include the PID) with a drop-guard cleanup.

#### [should-fix] `scripts/run_vo.py::ground_truth` turns a malformed ground-truth file into "not scored"
- **Where:** `scripts/run_vo.py:88-92`.
- **Issue:** `except (FileNotFoundError, InvalidInputError): return None` conflates "this sequence has no ground truth" (fine) with "the ground truth is corrupt" (an error). The user sees only `no ground truth: not scored`.
- **Suggested fix:** Catch `FileNotFoundError` only, or print the `InvalidInputError` message the way `score` does.

#### [should-fix] Native frame builders have no native tests, and nothing exercises their error paths
- **Where:** `cpp/src/edge/frames.cpp:105-179` (`monocular`, `rgbd`, `stereo_pair`), `rust/slamlab-edge/src/frames.rs:273-400`. Native tests only reach `depth_at`/`check_camera` (`cpp/tests/edge/test_map.cpp:184`, `rust/slamlab-edge/src/frames.rs:427-446`).
- **Issue:** The branches for depth map shape not equal to the image's, baseline of 0 for RGB-D or stereo, and a non-pinhole stereo camera are exercised only in Python (`tests/edge/test_frames.py:80-89`), and these natives are not called from Python. Their happy path is covered only through the `slamlab_vo` subprocess runs.
- **Suggested fix:** A `test_frames.cpp` and a `tests/frames.rs` (or inline tests) with the three refusals and a one-frame build per mode.

#### [should-fix] The exit-code to error-kind contract is tested for `InvalidInput` only
- **Where:** `src/slamlab/orchestration/vo.py:70-76,368-372`; `cpp/apps/orchestration/slamlab_vo.cpp:44` (`kExitFailed + static_cast<int>(kind)`, which relies on enum order); `rust/slamlab-orchestration/src/vo.rs:96-103`; `tests/orchestration/test_vo.py:120-131`.
- **Issue:** `Degenerate` (4) and `NotConverged` (5) are never produced or checked end to end, in any language. The C++ side depends on `ErrorKind`'s declaration order with no `static_assert`.
- **Suggested fix:** A `static_assert` on the three enumerators in C++, a Rust unit test for `exit_code`, and a Python test that maps codes 4 and 5 (a fake program script is enough).

#### [should-fix] `tests/edge/conftest.py::run_room` hard-codes `backend="cpp"` without `require_backend`
- **Where:** `tests/edge/conftest.py:191,194,199`.
- **Issue:** `tests/conftest.py` states that every test needing a backend goes through `require_backend`. Here the session fixture `runs` raises `BackendNotBuiltError` (an error, not a skip) where the C++ extension is not built. It also runs the C++ ORB extraction under tests that look like Python-only pipeline tests.
- **Suggested fix:** Call `require_backend("cpp")` in the fixture, or build frames with the Python backend and accept the slower tests.

#### [should-fix] Stale documentation of what `eval` and the native programs contain
- **Where:** `rust/slamlab-eval/src/lib.rs:1-5` ("Mirrors the Python package `slamlab.eval` where a standalone Rust pipeline needs it: reading image files"); `CLAUDE.md:37-38` (Languages table: "`eval`'s native PNG readers").
- **Issue:** The crate now also holds `tum`, `euroc`, `text` and `runs` (readers and run writer), and orchestration holds `slamlab_vo`. The module list in `lib.rs` grew; its prose did not.
- **Suggested fix:** One sentence each.

#### [should-fix] Native `camera.txt` and number-parse errors do not name the bad value
- **Where:** `cpp/src/eval/tum.cpp:163,171`, `rust/slamlab-eval/src/tum.rs:264` ("`<path>: expected numbers`"; the timestamp errors name the line but not the token).
- **Issue:** Python's equivalent includes the offending text (`expected numbers (could not convert string to float: 'x')`, `tum.py:268`).
- **Why it matters:** Native programs are what a user of `slamlab_vo` sees. The project's standard is that messages name the bad value.
- **Suggested fix:** Include the token and its field name.

#### [note] Unchecked `refresh` helpers mask a violated precondition
- **Where:** `rust/slamlab-edge/src/map.rs:547,558,563`; `cpp/src/edge/map.cpp:103-104,122`.
- **Detail:** Rust uses `distinctive_descriptor(..).unwrap_or(0)`, `.min().unwrap_or(-1)` and `.position(..).unwrap_or(0)`. If a point had no observations these would give a meaningless id or an index panic. C++ dereferences `*chosen` and `*find_if(...)` unchecked. Public entry points (`refresh_points`) do check non-emptiness, and the internal callers satisfy it today. A `debug_assert!` or an error return would make a future violation loud.

#### [note] `Map` does not check count invariants that `cull_points` relies on
- **Where:** `src/slamlab/edge/map.py:547-572` (`_check`) and `cpp/src/edge/map.cpp:272-306`; the division is at `mapping.py:360`.
- **Detail:** `pt_visible >= 1`, `found <= visible`, and finite poses and points are not validated. The division gives NaN with a RuntimeWarning (Python) or a silent NaN (natives) if `pt_visible` is 0. Operations preserve the invariants, so this only matters for hand-built or decoded maps.

#### [note] Unused API in all three languages
- `Pipeline.last_tracked` (`src/slamlab/edge/pipeline.py:137`), `Pipeline::last_tracked()` (Rust) and `Pipeline::last_tracked()` (`cpp/include/slamlab/edge/pipeline.hpp:96`) have no caller. Speculative surface; drop or use it.

#### [note] Python `assert` used as narrowing and as a postcondition
- **Where:** `src/slamlab/edge/tracking.py:170`, `pipeline.py:232,233,248`, `mapping.py:745`.
- **Detail:** The natives return `InvalidInput` in the same spots (`pipeline.rs:322-327,364-367`). `rung 0`'s `twoview.py:451` uses the same style, so it is consistent with the repo. `mapping.py:745` is the only one with a variable (`newest_id`) that exists solely for the assert.

#### [note] Test layout and design-doc drift
- C++ tests for `options`, `frames` and `residuals` live in `test_map.cpp` and `test_adjust.cpp` rather than mirroring the modules ("Tests mirror the packages").
- `docs/design/2026-10-05-edge-vo.md` section 2 still lists `initialize_*` and `predict` under `edge.tracking` and omits `initialization`, `residuals` and `adjust`. The note `docs/notes/edge-vo.md` is accurate, so cross-link or amend the plan.
- `slamlab.backends._RUST_PROGRAMS` ignores `CARGO_TARGET_DIR` (documented as checkout-only, decision 2).

#### [note] Backlog (outside this PR's goal)
- The note itself says keyframe time grows from 57 to 160 ms over MH_01 and is "not yet profiled". Candidates are the per-point allocations in `refresh` (a `Vec<Descriptor>` and an O(n² log n) `distinctive_descriptor` per refreshed point) and the feature grid rebuilt on every `search` call.
- The Python reference's per-point loops and `Map._check` on every `replace` are acknowledged as "the plain statement of the rules".

### Checked and fine
- **Validation and error contract:** `Options`, `Camera`, `Map`, `Tracked`, mode, `widen`, rows and ids are validated with messages naming the value. `check_options`, `check_map` and `check_camera` are mirrored in C++ and Rust, and the tests include the three refusals.
- **Error handling:** No bare `except`. The two `except DegenerateError: return None` blocks implement the documented "two views do not initialise" and "RANSAC finds no pose" outcomes. Rust non-test `unwrap`/`expect` count is zero (scripted). All `#[allow]` and `NOLINT` carry reasons. No debug prints (`scripts/run_vo.py` prints are its CLI output).
- **Docs:** Module docstrings state purpose, ladder position and ORB-SLAM source. Conventions are stated (`T_cam_world`, `T_world_cam` in runs, units, virtual baseline, exit codes). `edge-vo.md` is clear and honest about what is not done.
- **Run writer:** The native writer is atomic (temp file then rename), validates before writing anything, and is held to a golden folder. The JSON and `.npy` writers are documented as hand-written by decision.
- **Tests:** Parity tests assert exact integers and toleranced floats with named tolerances (`TOLERANCE`, `SOLVER_TOLERANCE`) and ATE bounds with stated provenance. `test_a_run_records_its_settings` pins the three languages' option lists and defaults. Lost-track, empty-frame, bad-shape and bad-mode edge cases are present.
- **Privacy:** No topics, frame ids, rig intrinsics, rates, file names or paths. The fixture `camera.txt` uses TUM-like values and the EuRoC `sensor.yaml` is the public calibration. The design docs mention only the generic "calibration side track" (those files pre-date this PR).
- **Release hygiene:** CHANGELOG, VERSION (0.3.2.0) and CLAUDE.md native-program text are consistent with the diff. Build and script plumbing (`check.sh`, `setup_slamlab.sh`, CMake apps glob, `rerun-sdk` declared and locked) is clean.

### Architecture review

## Architecture review

**Scope:** `feature/edge-vo`, `6c04d7b...6151c45` (10 commits, 190 files). I read the ladder design doc, the edge VO design doc and note, CLAUDE.md and the finding format. In all three languages I looked at the new edge rung, `slamlab.orchestration.vo` with the native programs, the native readers and run writer, the rung-0 additions (guided matching, `level_coordinate`, H-RANSAC's rank rule), `backends.program`, and the layer tooling (`pyproject.toml` contracts, `check_layers.py`, `CMakeLists.txt`, Cargo manifests, `check.sh`, `setup_slamlab.sh`). I also read the bindings and the tests in `tests/edge` and `tests/orchestration`. I did not re-run layer 1. I ran one read-only probe: the Python reference on the room, under a 4G memory cap.

**Summary:** The structure is right. The edge rung depends on rung 0 only. The generic pieces (search by projection, the feature grid, the level map) went into rung 0. The programs sit in orchestration, above `eval` and `edge`, and contain no logic of their own. The bindings only convert types, and the new internal-order contract covers the edge rung in all three languages. There is one must-fix: the edge rung has no parity fixture. Every check across languages compares against the Python reference computed live in the same test session, which breaks the CLAUDE.md rule and the edge design's own plan. The should-fixes are about how parity is held in the places the tests do not reach: thresholds outside `Options`, a recovery path no test compares across backends, the TUM tables, one binding convention, and where the Rust program is built and looked for.

### Findings

#### [must-fix] The edge rung has no parity fixture; every check across languages is a live comparison with the in-session Python reference
- **Where:** `tests/fixtures/` (no `edge/`); `tests/edge/conftest.py:1-10`; `CLAUDE.md:72-74`; `docs/design/2026-10-05-edge-vo.md:129-133`
- **Issue:** CLAUDE.md requires every C++/Rust implementation to have "a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The edge design planned stage parity "as rung 0 ... on fixture states" and synthetic sequences in "`tests/fixtures/edge/`, generated". What the PR does instead: it renders the room when the tests run and compares each backend with the Python reference computed in the same session, at the tolerances in `tests/edge/support.py`. The reason for the change ("would run to megabytes as stored fixtures") is written only in a test docstring. Neither design doc, the note's decisions nor CLAUDE.md records it.
- **Why it matters:**
  - The native suites cannot hold the edge cores to Python. ctest and cargo test each build their own scene (`cpp/tests/edge/scene.hpp`, `rust/slamlab-edge/tests/scene/mod.rs`) and test properties of it.
  - A behaviour change made in all three languages at once passes every parity test. Lock-step commits make that the normal case. Only the room's ATE bound would notice, and nothing records what the pipeline decided.
  - The same rule was a must-fix on PR 1 (A1) and PR 3.
- **Evidence:** `git ls-tree -r --name-only 6151c45 tests/fixtures | grep edge` prints nothing.
- **Suggested fix:** The owner decides between two options. I recommend (a):
  - **(a)** Amend CLAUDE.md and edge-vo §3 to state the edge rung's policy: live stage parity on deterministic generated states, the tolerance policy in `tests/edge/support.py`, and the reason. Then commit a small golden record of the room runs that `test_vo` holds every backend to, so a change made in all three languages fails a test. Per dataset and mode it would hold the frames tracked, the inlier counts, the keyframe timestamps, the map size, and the poses at `SOLVER_TOLERANCE`.
  - **(b)** Commit compact stage states as fixtures.

#### [should-fix] Thirteen named thresholds and two unnamed factors live outside `edge.Options`, defined three times and recorded nowhere
- **Where:**
  - `src/slamlab/edge/pipeline.py:43-50`: `RECOVERY_WIDEN`, `RECOVERY_WIDE_FRAMES`, `RECOVERY_TRACKED`.
  - `tracking.py:45,49,573`: `NARROW_VIEW_COS`, `MIN_MOTION_INLIERS`, and the unnamed 2.5 / 4.0 radius factors.
  - `mapping.py:46-64`: `TRIANGULATION_NEIGHBOURS`, `EPIPOLAR_CHI2`, `EPIPOLE_DISTANCE2`, `MIN_BASELINE_DEPTH`, `FIRST_REF_RATIO`, `RECENT_KEYFRAMES`.
  - `initialization.py:36`: `INIT_BA_ITERATIONS`.
  - `orchestration/vo.py:58,256`: `RGBD_BF`.
  - Each has a copy in `cpp/include/slamlab/edge/{pipeline,tracking,mapping,initialization}.hpp`, in `cpp/include/slamlab/orchestration/vo.hpp:25` and in the Rust modules of the same names.
  - The stereo matcher also runs at each language's own default settings (`frames.stereo_pair(...)` without `stereo_options` in `vo.py`; `MatchOptions{}` in `vo.cpp`).
- **Issue:** Edge-vo §2 says `edge.Options` holds "every threshold above, named and defaulted to ORB-SLAM's, crossing as a dict". These values do not cross the bindings, are not in `run.json`'s `settings`, and cannot be set with `--option`. `test_a_run_records_its_settings` holds the native defaults of `Options` to Python's; nothing holds these.
- **Why it matters:**
  - A run folder does not record everything that decided it.
  - A drift in one copy is caught only if some stage-parity state takes that branch. The recovery constants are never reached (next finding).
  - `RGBD_BF` belongs to the sensor: ORB-SLAM keeps `Camera.bf` in each camera's settings file. Here it is applied to every TUM-format folder, including `camera.txt` exports of other RGB-D sensors (the calibration side track's), which then run with the Kinect's value.
- **Suggested fix:**
  - Move the tunable values into `Options`. They then cross the bindings, are recorded and can be overridden, and the existing settings test holds their native defaults.
  - Make `bf` either an `Options` field or a `camera.txt` key.
  - For any value that stays a constant, expose it from each binding and compare all three in one test.

#### [should-fix] No layer-1 test runs the pipelines' recovery path in more than one backend
- **Where:** `tests/orchestration/test_vo.py:52-69` (`test_the_backends_agree`); the room (`FRAMES = 40`); `src/slamlab/edge/pipeline.py:230-280`, `cpp/src/edge/pipeline.cpp:139-225`, `rust/slamlab-edge/src/pipeline.rs:321-440`
- **Issue:** The three pipeline loops are compared only by whole-run parity, and the room never takes the fallback. These parts are implemented three times and compared only on the TUM/EuRoC runs in the note, outside layer 1:
  - the `since_recovery` counter;
  - the widened search (`RECOVERY_WIDEN`, `RECOVERY_WIDE_FRAMES`);
  - the 50-inlier rule;
  - the held-keyframe rule;
  - the velocity reset after a failure.

  `relocalize_locally` itself has stage parity; the loop around it does not.
- **Evidence:** I ran the Python pipeline on the room in each mode, wrapping `tracking.track_last_frame` and `tracking.relocalize_locally` to count calls:
  ```
  rgbd      frames 40 tracked 40 {'relocalize': 0, 'last_frame_failed': 0}
  stereo    frames 40 tracked 40 {'relocalize': 0, 'last_frame_failed': 0}
  monocular frames 40 tracked 33 {'relocalize': 0, 'last_frame_failed': 0}
  ```
  The probe is at `/tmp/scratch/probe_recovery.py`. It is not saved anywhere permanent; rescue it if wanted.
- **Suggested fix:** Add a room variant that forces a recovery, for example a folder written with a run of frames dropped (a jump the motion model cannot bridge). Run it through `vo.run` with every backend, assert the same `(state, tracked, keyframe)` sequence, and assert that a recovery happened.

#### [should-fix] `mode` crosses the edge bindings by name, against the design doc's convention
- **Where:** `src/slamlab/edge/mapping.py:139` (`_arguments(..., mode=mode)`) and `tracking.py:541,652`; `cpp/bindings/edge/edge_crossing.hpp:418`; `rust/slamlab-py/src/edge/edge_crossing.rs:616`; `cpp/include/slamlab/edge/options.hpp:18`; `docs/design/2026-10-02-slamlab-ladder.md:297-300`
- **Issue:** The design doc says a choice field inside an options dataclass crosses by name, and "a choice passed as its own argument ... crosses as a code". Rung 0's camera `model` argument crosses as a code. The edge stage functions take `mode` as their own argument but pass its name. The C++ `Mode` comment says "with the codes they cross the bindings as", which describes the binding wrongly.
- **Why it matters:** There are now two conventions for choices passed as arguments, and the next rung has no single rule to follow.
- **Suggested fix:** I recommend amending the design doc to say the edge's `mode` crosses by name, or that new argument choices do, and fixing the C++ comment. The alternative is to pass codes, with a `codes()` entry. PR 3 settled the solver-choice question the same way, with a doc amendment.

#### [should-fix] Where the Rust program is built depends on cargo's environment, and nothing rebuilds it from its sources
- **Where:** `src/slamlab/backends.py:52,240`; `scripts/check.sh:85`; `scripts/setup_slamlab.sh:107`
- **Issue:**
  - `program()` looks only in `<checkout>/rust/target/release/`, but the scripts build into cargo's default target directory. With `CARGO_TARGET_DIR` set, or `build.target-dir` in a cargo config, the binary goes elsewhere. `program()` then either reports "not built", or silently runs an older binary left in `rust/target/release`.
  - The C++ program is rebuilt with the extension on import, and `program()` triggers that import. The Rust binary is rebuilt only by `check.sh` and `setup_slamlab.sh`. A pytest run after editing a Rust edge module therefore compares Python with the previous build.
- **Why it matters:** This is behaviour that depends on the environment without an explicit setting. The parity tests can pass or fail against code that is not the code being tested. CI starts from a clean checkout and is not affected.
- **Suggested fix:**
  - Pass `--target-dir "${ROOT_DIR}/rust/target"` in both scripts.
  - Say in `program()`'s docstring and in CLAUDE.md's native-programs line that the Rust program is not rebuilt automatically.
  - Optionally, have `require_program` fail when the binary is older than the crate sources.

#### [should-fix] No fixture holds the native readers' freiburg2 and freiburg3 calibrations
- **Where:** `cpp/src/eval/tum.cpp:27-29`; `rust/slamlab-eval/src/tum.rs:33-50`; `tests/fixtures/eval/generate_sequences.py` (its TUM folders are `rgbd_dataset_freiburg1_fixture`, `tum_camera` and `tum_no_depth`)
- **Issue:** The only folder named for a sensor is freiburg1. The C++ and Rust copies of the fr2 and fr3 tables, nine numbers each, are never compared with `tum.INTRINSICS`. fr3/long_office_household is one of the PR's reported sequences.
- **Suggested fix:** Add freiburg2 and freiburg3 folders (text files only) to `generate_sequences.py`.

#### [note] `frame_dict` lives in `edge.map`, not with `Frame`
- **Where:** `src/slamlab/edge/map.py:639`
- `Camera.as_dict`, `Tracked.as_dict` and `Map.as_dict` each sit on their own type. The frame's crossing form belongs in `frames.py` as `Frame.as_dict()`. Moving it would not change the layering.

#### [note] (backlog, milestone 4) `edge.frames.Camera` is a candidate for rung 0
- **Where:** `src/slamlab/edge/frames.py:38`
- Camera models and intrinsics are already rung 0's. Once the classic rung needs a camera with an image size (and the chi-square test by pyramid level in `edge.residuals`), lift them to `foundations`. The siblings cannot share them otherwise, and they should not be copied.

#### [note] The exit codes are defined three ways
- **Where:** `cpp/apps/orchestration/slamlab_vo.cpp:43`; `rust/slamlab-orchestration/src/vo.rs` (`exit_code`); `src/slamlab/orchestration/vo.py:65-75`
- C++ computes `kExitFailed + static_cast<int>(kind)` from the implicit order of `ErrorKind`. Rust maps each kind explicitly; Python indexes a tuple. End to end, only `InvalidInput` and the usage code are tested. Give C++ an explicit `exit_code(ErrorKind)` in `vo.hpp`, mirroring Rust's.

#### [note] `vo.run` fails differently by backend for a missing file
- The Python backend raises `FileNotFoundError`; the programs give `InvalidInputError`. This follows the decision in the design doc for `eval`'s readers and is documented in `vo.run`. It still means one `backend=` function fails two ways. `vo.run` could translate one form into the other if a caller needs a single type.

#### [note] (backlog, PR 9) Real-time items this rung inherits
- `edge::search` and Rust's `search` rebuild the `FeatureGrid` on every call. Edge-vo §2 says "per-frame".
- The prediction projects onto SO(3) every frame, which in Rust goes through the allocating `svd3`. The open item A11 tied a fixed-size `svd3` to "the edge milestone".

#### [note] Edge-vo §2's module list is stale
- **Where:** `docs/design/2026-10-05-edge-vo.md:86`
- It puts `initialize_*` in `edge.tracking` and lists no `options`, `residuals`, `adjust` or `initialization` modules. The note describes what was built. Update §2, or mark the doc as the plan as it stood.

#### [note] C++ `check_options` lists the fields by hand
- **Where:** `cpp/src/edge/options.cpp:48-131`
- The fields appear in the struct, in `kIntegerOptions`/`kRealOptions`, and again in five check lists. All 44 are covered today. A field added to the tables but missed in the check lists would go unchecked in C++, whereas Python checks every field by its type. Driving the checks from the tables, with a per-field range class, would close that. (This one is mostly the hygiene reviewer's.)

### Checked and fine
- The ladder's direction holds:
  - `edge` imports only `foundations` and `backends` (Python imports, `slamlab_add_rung(edge DEPENDS slamlab_foundations)`, the `slamlab-edge` crate).
  - Orchestration sits on top, over `edge` and `eval`.
  - Nothing imports reifyscene.
  - The edge's runtime dependencies are numpy, Eigen and nalgebra only; no file I/O and no depth model.
- The new "edge rung's internal order" contract matches every module in all three languages. `check_layers.py` now covers `cpp/apps/<rung>/`, with a test.
- The rung-0 additions are generic and in the right place: `Projections`, `SearchOptions`, `search_by_projection`, `features_in_area`, `level_coordinate`, and the H-RANSAC rank rule. All have fixtures, and two-view's rank-1 workaround was removed cleanly.
- The bindings only convert types, one file per module, with shared crossing helpers in the style of `common.hpp`. Map, frame, camera, tracked frame and options are all read strictly with `check_keys`. Every bound edge module has an `Implementation` Protocol, and `COMPILED_RUNGS` covers `edge` for both compiled backends.
- Each public Python stage function validates once, then calls `backends.call`. The native cores re-check the camera, the options and the map, and return `Result`/`expected` with the shared error kinds.
- The native programs are thin `main`s. The logic is in the `slamlab_orchestration` library and the `slamlab-orchestration` crate, tested by ctest and cargo.
- `Request.arguments` passes only the options that differ from the defaults. That is safe, because `test_a_run_records_its_settings` holds the native defaults of `Options` to Python's.
- The native-only `pt_observations` index is derived data. It is validated by `check_map`, rebuilt by `index_observations` on every crossing, and documented in `map.hpp` and the note.
- The C++ program's lookup goes through the package's `bin/` after importing the extension (which triggers the editable rebuild), never through `PATH`, as decided.
- No globals, environment reads or import-time side effects in the new code (the one module-level constant is the Rust program's path in `backends`).
- No compatibility shims: `tum.Sequence.sensor` was replaced by `camera` outright.
- No new runtime dependency: `serde_json` is a dev-dependency only, and the JSON and `.npy` writers are hand-written, per decision 1.
- Privacy: the `camera.txt` fixtures are synthetic values close to the TUM benchmark's (525/525.5/319.5/239.5). Nothing in the diff is specific to the private rig.
- The ladder doc's Package-layout and Languages rows were updated for `slamlab_vo` and the `slamlab-orchestration` crate.
- The edge package docstring states its compute budget.

### Algorithm review

## Algorithm review

**Scope:** `6c04d7b...6151c45` (feature/edge-vo, PR 8), 10 commits. I read the whole Python reference in `src/slamlab/edge/` (options, residuals, frames, map, adjust, initialization, tracking, mapping, pipeline) and the rung-0 diffs (matching guided search, image `level_coordinate`, homography rank rule, twoview, stereo). I also read `orchestration/vo.py`, `scripts/run_vo.py`, `eval/runs.evaluate`, and `docs/notes/edge-vo.md` with its design doc, and spot-checked the C++ observation index (87bb419). I checked the algorithms line by line against the ORB-SLAM2 source (raulmur/ORB_SLAM2 master: Tracking.cc, LocalMapping.cc, ORBmatcher.cc, Optimizer.cc, Initializer.cc, System.cc, the Examples yaml files) and against ORB-SLAM3's Tracking.cc. I checked the published numbers against ORB-SLAM (T-RO 2015) Table III, ORB-SLAM2 (T-RO 2017) Tables II and III, and ORB-SLAM3 (T-RO 2021) Table II.

**Summary:** Tracking and local mapping follow ORB-SLAM2's own code closely. Almost every threshold and rule I checked matches the source line for line, and I found no bug that breaks the pipeline: **no must-fix**. There are five should-fix findings:
- Pose refinement ignores the per-level weighting that ORB-SLAM's PoseOptimization uses, and nothing documents this.
- Monocular initialisation has undocumented departures from ORB-SLAM, and the note misexplains MH_01's late start. An experiment shows rung 0's two-view succeeds from frame 6; it is the edge rung's 100-point rule that refuses every early map.
- One results comparison (with DSO) needs qualifying.
- The velocity rule after a failed frame does not match its documentation.
- Several setting docstrings misstate ORB-SLAM.

### Findings

#### [should-fix] Pose refinement drops the per-level information weighting of ORB-SLAM's PoseOptimization
- **Where:** `src/slamlab/edge/tracking.py:232-261` (`_optimize_pose`; the solve is `pnp.refine_pose(current, xyz[use], frame.bearings[use], scale=scale)` at :251). Rung 0's `refine_pose` (`src/slamlab/foundations/pnp.py:478`) takes no weights. The C++ and Rust versions are the same (parity).
- **Issue:** Every match pulls with equal weight in the solve. The chi-square classification right after it divides by `s^(2l)`, so the two use different noise models. ORB-SLAM2 sets each edge's information to `invSigma2 = 1/s^(2l)` (Optimizer.cc:299-300 mono, 335-336 stereo; paper eq. 2: "Σ the covariance matrix associated to the scale of the keypoint"). Two other departures are stated in the module docstring but not in the note:
  - the solve uses Cauchy in all four rounds, where ORB-SLAM uses Huber and removes the kernel from round 3 (Optimizer.cc:407-408, 436-437);
  - it uses the left image only.

  The missing weighting is not stated anywhere. The note's step 2 says only "four rounds of rung 0's refine_pose".
- **Why it matters:** Every reported frame pose comes from this solve. Coarse-level keypoints (σ up to 1.2^7 = 3.6 px) pull as hard as level-0 ones.
- **Evidence:**
  - Monte Carlo of pose-only Gauss-Newton: 150 points, levels drawn with ORB's per-level share, σ_l = 1.2^l, f = 525, 300 trials, no outliers. Camera-centre RMSE is **3.21 mm unweighted vs 2.58 mm information-weighted (+24%)**. The script is `pose_level_weighting_mc.py`.
  - The test room's cross-keyframe residuals fit that noise model: std 0.42 / 0.50 / 0.52 / 0.72 px for levels 0-3.
- **Suggested fix:** Give rung 0's `refine_pose` per-correspondence weights (`1/s^(2l)`, robust scale in whitened units) in all three languages. Otherwise, list "unweighted, Cauchy, left image only" as lite departures in the note and the tracking docstring.

#### [should-fix] The note misexplains MH_01 monocular's late start, and initialisation has undocumented departures from ORB-SLAM
- **Where:**
  - `docs/notes/edge-vo.md:175-176` ("no earlier pair of frames passed two-view initialisation");
  - `src/slamlab/edge/initialization.py:238-259`: only `result.triangulated` points are kept, then `m.points < init_min_matches` at :258;
  - `src/slamlab/foundations/twoview.py:167` (`min_parallax_deg = 1.0`);
  - `initialization.py:147-168`: the search window is centred at `keypoints.xy[rows]` (:153).
- **Issue:**
  - (a) The note's explanation is wrong. Rung 0's two-view returns INITIALIZED from frame 6 onwards. The edge rung then refuses every early map because fewer than 100 points survive.
  - (b) The edge map keeps only points with ≥ 1° parallax, then requires 100 of them. ORB-SLAM keeps every good point with parallax > 0.36° (Initializer.cc:892-893, `cosParallax<0.99998`). It needs only the 51st-largest parallax above 1° (Initializer.cc:900). It then requires ≥ 100 tracked points (ORB-SLAM2 Tracking.cc:692), relaxed to **50** in ORB-SLAM3 (Tracking.cc:2589, "TODO Check, originally 100 tracks"). The note compares MH_01 with ORB-SLAM3.
  - (c) `match_for_initialization` claims to be `SearchForInitialization`, but it searches each reference keypoint at its own position. ORB-SLAM searches at the position where the keypoint was last matched and updates it every frame (ORBmatcher.cc:425, 514-517), so parallax can build up beyond the 100 px window.
- **Why it matters:** 38% of MH_01 is untracked (2288 of 3682 frames), and the note's MH_01 comparisons rest on that. The wrong explanation points a reader at the wrong component (rung 0's two-view).
- **Evidence:** I replayed the pipeline's initialisation loop on MH_01 with the cpp backend (`edge_mono_init_probe.py`):
  ```
  frame 6:  148 matches, 142 inliers; rung-0 triangulated 55; ORB-SLAM-kept 72
  frame 39: 108 matches, 100 inliers; rung-0 triangulated 57; ORB-SLAM-kept 97
  frame 54: 111 matches, 104 inliers; rung-0 triangulated 55; ORB-SLAM-kept 98
  ```
  Two-view returned INITIALIZED with 51-72 triangulated points at frames 6, 8, 39, 43 … 135, and `initialize_monocular` refused each one. With ORB-SLAM3's 50-point rule, frame 6 would start the map. On these early pairs, a following window found the same first success (frame 6), so (c) is a fidelity gap, not the cause here.
- **Suggested fix:**
  - Correct the note's sentence.
  - Decide and document the rule: keep the 0.36°-1° points as ORB-SLAM does, and/or use ORB-SLAM3's 50.
  - Either implement the following window (the Pipeline carries the matched positions) or list the fixed window as a lite departure.
  - Re-run MH_01 and V1_01 monocular afterwards.

#### [should-fix] The EuRoC monocular comparison with DSO is not like for like
- **Where:** `docs/notes/edge-vo.md:171`, `:201-202`, `:209-210`; table row `:167`.
- **Issue:**
  - "its error over the frames it tracks is below DSO's (3.7 and 6.0 cm against 4.6 and 8.9 cm)": the MH_01 figure covers 62% of the sequence (frames 1393-3682; the first 70 s are untracked). ORB-SLAM3 Table II, where the DSO row comes from, reports errors "for all the frames in the trajectory".
  - "each the median of five runs": ORB-SLAM3's numbers are medians of ten executions (§VII-A). ORB-SLAM's and ORB-SLAM2's are medians of five.
  - "342 of 409, lost" counts frames up to the loss, while the other rows count against the whole sequence.
- **Why it matters:** A reader takes "below DSO's" as a ranking, and the note's stated purpose is a fair comparison.
- **Evidence:** ORB-SLAM3 (arXiv:2007.11898), Table II caption and §VII-A; note row 171 ("2288 of 3682").
- **Suggested fix:** Drop the MH_01 ranking against DSO, or state its coverage right beside it. Write the fr1/desk row as "342 of 596 (lost at 409)". Say ten runs for ORB-SLAM3, and cite the source tables.

#### [should-fix] After a failed frame the motion model still applies the stale velocity; the code and note say it predicts no motion
- **Where:**
  - `src/slamlab/edge/pipeline.py:224-228`: `_record_failure` leaves `_velocity` unchanged;
  - `:235-237`: the comment "No velocity (just initialised, or after a failure): predict no motion";
  - the module docstring at `:11-12`; `docs/notes/edge-vo.md:111-112`;
  - the same logic in `cpp/src/edge/pipeline.cpp:118-146` and `rust/slamlab-edge/src/pipeline.rs:289-330`.
- **Issue:** The velocity is only cleared by `_accept` on the first success after a failure. Every frame in between is predicted as velocity @ last tracked pose.
- **Evidence:** In-memory stub (stages patched, last motion 5 cm along x):
  ```
  attempt 0 (after 0 failures): predicted translation x = 0.050 m
  attempt 1 (after 1 failures): predicted translation x = 0.050 m
  attempt 2 (after 2 failures): predicted translation x = 0.050 m
  ```
- **Why it matters:** The documented decision is not what runs, and recovery after failures (fr1/desk monocular loses the track) behaves differently from what the note describes. The behaviour itself is defensible.
- **Suggested fix:** Clear the velocity in `_record_failure` in all three languages, or document the rule that actually runs. Add a pipeline test for the prediction after a failure.

#### [should-fix] Setting docstrings misstate ORB-SLAM
- **Where:**
  - `src/slamlab/edge/options.py:85-87` (`max_frames`: "A keyframe is due at the latest this many frames after the last"). The setting is only used for the post-recovery window (`pipeline.py:256, 272`). In ORB-SLAM, condition c1a still needs c2, so no keyframe is ever forced by a frame count.
  - `options.py:50-51` (`min_depth_features`: "Keypoints with depth … (ORB-SLAM: 500)"). ORB-SLAM2's StereoInitialization tests `mCurrentFrame.N>500`, which counts all keypoints (Tracking.cc:511).
  - `pipeline.py:43-45` (`RECOVERY_WIDEN`: "SearchLocalPoints' th = 5"). ORB-SLAM sets th to 5; here it is 5 × th, so 15 for RGB-D.
- **Why it matters:** The settings claim ORB-SLAM's semantics, so a reader tuning against ORB-SLAM is misled.
- **Suggested fix:** Reword each docstring, or change the code to match ORB-SLAM.

#### [note] The last frame is not re-anchored after local BA, and the trajectory holds the tracking-time poses
`pipeline.py:276-280`: local BA moves a new keyframe's pose in the map, but `_accept` stores the pose from before BA. The next velocity therefore contains the BA correction. ORB-SLAM re-anchors the last frame to its reference keyframe every frame (Tracking.cc:807, `UpdateLastFrame`). It also saves every frame relative to its BA-refined reference keyframe (System.cc:342). Run poses here are the tracking-time estimates (`orchestration/vo.py:337-338`), so the note's every-frame ATE is stricter than ORB-SLAM's protocol. The published TUM monocular numbers are keyframe trajectories (ORB-SLAM Table III). Scoring the final `keyframe_poses.npy` as well would give a like-for-like column.

#### [note] Local BA's robust loss and depth model differ from ORB-SLAM's stereo edge
- `adjust.py:200-202` sets only the reprojection loss. The depth prior keeps BAOptions' default trivial loss, whereas ORB-SLAM's 3-row stereo edge has Huber at √7.815 (Optimizer.cc:570, 638).
- Huber stays on in the second pass; ORB-SLAM removes it (Optimizer.cc:685, 701).
- The inverse-depth prior with information `(fb)²/σ²` treats disparity as independent of u with variance σ². ORB-SLAM's isotropic (u, v, u_r) edge implies var(d) = 2σ² and cov(u, d) = σ².

The docstring states the prior but not these consequences. I tested a Huber depth loss on the room's truth map with 10% of depths read 2.5× too far, and it did not help: maximum keyframe-centre error was 29 mm with the trivial loss vs 58 mm with Huber (17 mm for both on clean data). So I suggest documenting this, not changing it.

Also, `mapping.py:881` runs local BA when the map has two keyframes; ORB-SLAM runs it only when `KeyFramesInMap()>2` (LocalMapping.cc:80).

#### [note] The EuRoC runs did not use ORB-SLAM2's EuRoC settings beyond the feature count
The note (lines 150-152) says the runs used ORB-SLAM's settings. ORB-SLAM2's `Examples/Stereo/EuRoC.yaml` has `ThDepth: 35` and `Camera.fps: 20` (so mMaxFrames is 20). The runs used close_depth 40 and max_frames 30, because `vo.py` sets no per-dataset values. The effect is small; either say so in the note or apply the values.

#### [note] Solve summaries are discarded
`adjust._bundle_adjust` and `tracking._optimize_pose` drop the BA and `refine_pose` summaries. Rung 0's LM accepts only cost-decreasing steps (`nls.py:43`), so nothing can diverge silently, and fixed iteration counts are ORB-SLAM's design. Still, recording iterations, initial and final cost, and termination in `diagnostics.jsonl` would make the long-sequence drift (fr3/long_office) investigable.

#### [note] The homography condition margin is evidenced only in a docstring
`homography.py:69-75` cites a 40,000-scene sweep (worst valid case 3.7e-4 against the 1e-4 threshold). No test guards it: the new test covers only the singular duplicate case. A property test using the sweep's worst valid configuration, in all three languages, would protect the margin.

#### [note] Backlog: the root cause of the singular H
The twoview fixture has 38 of 88 matches sharing one pixel (`test_twoview.py:248-254`). The rank rule handles the symptom; where duplicate matches come from upstream is outside this PR.

#### [note] State of the art
ORB-SLAM-lite is the right classic for this rung. For context:
- ORB-SLAM3 (Campos et al., T-RO 2021) is the current classic reference.
- Learned sparse VO/SLAM has moved on: DPVO (Teed, Lipson & Deng, NeurIPS 2023) and DPV-SLAM (Lipson, Teed & Deng, ECCV 2024, arXiv:2408.01654) match DROID-SLAM on EuRoC at a fraction of its cost.
- MASt3R-SLAM (CVPR 2025) is the newest dense direction.

All of these are later ladder rungs, not defects here.

### Checked and fine
- **NeedNewKeyFrame:** run single-threaded it reduces to c2. nMinObs is 2/3 on weighted Observations(); thRefRatio is 0.4/0.75/0.9; close points use 100/70; inliers must exceed 15. Matches Tracking.cc.
- **TrackWithMotionModel:**
  - the velocity is T_cur_last;
  - windows are 15/7, retried at 2× under 20 matches;
  - at least 10 inliers are required;
  - the tlc forward/backward level rule has the right sign;
  - the orientation check is on and the right column is gated by radius.
- **SearchLocalPoints / isInFrustum:**
  - visibility gates: distance 0.8-1.2, view cosine 0.5;
  - PredictScale uses ceil and clamps;
  - search window: RadiusByViewingCos 2.5/4 at 0.998, th 1 (3 for RGB-D);
  - matching: ratio 0.8 only when the second best is on the same level;
  - acceptance and counting: 30 inliers (50 after recovery); visible and found counts match ORB-SLAM's increments.
- **CreateNewMapPoints:**
  - candidates: neighbours 10/20; baseline compared with b, or with 1% of the (n−1)/2 median depth;
  - matching: epipolar band 3.84σ², epipole 100·s^l without depth, TH_LOW, no ratio test;
  - triangulation: parallax rules including the stereo cosine and its else-if asymmetry;
  - acceptance: cheirality through the chi-square test, 5.991/7.815 reprojection, scale consistency 1.5·s.
  - Lindstrom niter2 replaces the DLT, which is better.
- **SearchInNeighbors / Fuse:** projection gates, levels l−1..l, radius 3·s^l, chi-square gating, TH_LOW, replacement by observation count, summed counts, and refreshed descriptors all match.
- **Culling:**
  - MapPointCulling uses 0.25, nObs ≤ 2/3 at age ≥ 2, and the recent list until age 3. Initial points are culled as in ORB-SLAM, whose ProcessNewKeyFrame also puts them in the recent list.
  - KeyFrameCulling uses 90%, 3 observers at level ≤ l+1, close depth only, and never culls KF0.
  - EraseObservation erases at nObs ≤ 2, with stereo observations weighted 2.
- **MapPoint:** the distinctive descriptor uses the (N−1)/2 median; UpdateNormalAndDepth's distances match; the normal is normalised (documented, and more principled).
- **CreateNewKeyFrame depth points:** sorted by depth, with the break after 100 points and ThDepth.
- **Local BA structure:** the window is free, KF0 and outside observers are fixed, it runs 5 + 10 iterations, drops chi-square and cheirality outliers, and handles the case of fewer than two fixed keyframes through the "auto" gauge.
- **Chi-square test:** variance `s^(2l)`, 2/3 dof, and right column `u − fb/z` are consistent with keypoints kept in the distorted image. The RGB-D virtual bf = 40 matches TUM1.yaml.
- **`level_coordinate`:** I verified the algebra; it is the exact inverse of `full_image_coordinate`.
- **H-RANSAC rank-below-3 rule:** sound, measured in the Hartley-normalised frame.
- **Conventions:** `T_cam_world`, `T_c2_c1`, centres `−Rᵀt`, and the prediction projected onto SO(3).
- **Evaluation:**
  - EuRoC ground truth is moved to cam0 through `T_BS`, and alignment is Sim(3) for monocular and SE(3) otherwise, as in ORB-SLAM3 §VII.
  - Stereo poses are the rectified left camera's, 0.62° from cam0 on MH_01. Positions are unaffected; the per-frame RPE translation changes by about 0.3 mm, which is negligible.
- **Published numbers** all verified:
  - ORB-SLAM Table III: 0.90 / 1.69 / 3.45 cm, keyframes, 7-DoF, median of 5;
  - ORB-SLAM2: fr1/desk 0.016, fr3/office 0.010, MH_01 and V1_01 0.035;
  - ORB-SLAM3 Table II: stereo 0.029/0.035, monocular 0.016/0.033, DSO 0.046/0.089.
- **Determinism:** RANSAC and the synthetic room are seeded.
- **Native observation index (87bb419):** attach, detach and the row shift in `erase_keyframe` are correct, and the binding checks the index against a rebuilt one (`map.cpp:301`).
- **Privacy:** nothing in the diff is specific to the rig. The `camera.txt` fixtures use generic 525 / 319.5 intrinsics.

Sources:
- [ORB-SLAM3, arXiv:2007.11898](https://arxiv.org/abs/2007.11898)
- [ORB-SLAM2, arXiv:1610.06475](https://arxiv.org/abs/1610.06475)
- [ORB-SLAM, arXiv:1502.00956](https://arxiv.org/abs/1502.00956)
- [ORB_SLAM2 source](https://github.com/raulmur/ORB_SLAM2)
- [ORB_SLAM3 source](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [DPV-SLAM, arXiv:2408.01654](https://arxiv.org/abs/2408.01654)
- [DPV-SLAM, ECCV 2024](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/272_ECCV_2024_paper.php)
