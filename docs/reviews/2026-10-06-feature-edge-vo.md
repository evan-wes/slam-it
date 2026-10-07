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

## Run 2 -- 2026-10-07

| | |
|---|---|
| **Branch** | `feature/edge-vo` |
| **Range** | `6c04d7bf8c2d3ad84bb56dd5904d0f6e80c1968e...e0e82087cc0dabfcec7715c7d3bc527b9a88a3a0` (base: `origin/develop`); the scope is run 1's fixes, `6151c45..e0e8208` |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus): the fixes changed each one's area substantially (a hygiene batch across 48 files, 20 settings in three languages, behaviour changes in tracking, mapping and initialisation, a golden record) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed at `09eb6a2` (`scripts/check.sh`, exit 0): 2869 pytest, every ctest and cargo test; the later commits are docs only |

### Findings and resolutions

Numbering continues from run 1. Duplicates across reviewers are cross-referenced ("same as")
and resolved together.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H20 | should-fix | hygiene | `test_initialization.py:154` | L2's parallax rule has no test that fails without it: the room's pair has no inlier between 0.36 and 1 degree | fix | fixed (7684f34) -- decided by evan-wes, 2026-10-07 |
| H21 | should-fix | hygiene | `tests/edge/`, native tests | None of the 20 new settings is ever run off its default; the native check lists are complete but nothing pins them (with A22, H31) | fix | fixed (7684f34): a table-driven refusal test per language; the gap room with every new setting moved off its default, every native against Python -- decided by evan-wes, 2026-10-07 |
| H22 | should-fix | hygiene | `pipeline.cpp:134`, `pipeline.rs` | L4's native fixes are untested (same as A14) | fix with A14 | fixed (7684f34, with A14) -- decided by evan-wes, 2026-10-07 |
| H23 | should-fix | hygiene | `frames.py:177` | `Frame.__post_init__` (H2's fix) is untested; its message does not name the received dtype | fix | fixed (349dbb8: the message; 7684f34: the tests) -- decided by evan-wes, 2026-10-07 |
| H24 | should-fix | hygiene | `tracking.py:435`, `mapping.rs:75`, `adjust.cpp:182`, ... | H4 and H5 not fully done: an inlined camera centre, `check_setup` unused at seven sites, the in-view cosine written two ways, bare literals | fix | fixed (349dbb8) -- decided by evan-wes, 2026-10-07 |
| H25 | should-fix | hygiene | `initialization.py:4`, `options.py:124`, `tracking.py:357`, `edge_crossing.hpp:4`, ... | Docs the fixes left misstating the code | fix | fixed (349dbb8) -- decided by evan-wes, 2026-10-07 |
| H26 | should-fix | hygiene | `test_vo.py:75` | `test_the_gap_forces_a_recovery` asserts less than it claims, and in Python's TUM RGB-D only (with A14) | fix with A14 | fixed (7684f34, with A14) -- decided by evan-wes, 2026-10-07 |
| H27 | should-fix | hygiene | `runs.rs:536`, `tum.rs:574`, `vo.rs:804` | H7 half done in Rust: no drop guard, so a failed test leaves a PID-named folder | fix | fixed (29134df) -- decided by evan-wes, 2026-10-07 |
| H28 | note | hygiene | `generate_room_runs.py:64`, `room_runs.json` | The golden record is hard to review: 8,343 lines, a copied tolerance, the gap written twice, two identical cases (same as A20) | fix with A20 | fixed (7684f34, with A20) -- decided by evan-wes, 2026-10-07 |
| H29 | note | hygiene | various | Smaller doc and error-message items | fix | fixed (349dbb8: docs and messages; 7684f34, 29134df: the tests) -- decided by evan-wes, 2026-10-07 |
| H30 | note | hygiene | the gap room | The gap room recovers twice, not once (its velocity spans the gap) | document in the fixture | fixed (7684f34: the gap room's docs) -- decided by evan-wes, 2026-10-07 |
| H31 | note | hygiene | `options.cpp`, `options.rs` | A13's hand-kept check lists grew by 20 entries (with H21, A22) | covered by H21's test | covered by H21's test (7684f34) -- decided by evan-wes, 2026-10-07 |
| A14 | should-fix | architecture | `datasets.py:106`, `test_vo.py:75`, `pipeline.py:264`, `edge-vo.md:180` | The failure path and the keyframe hold run in no cross-backend test; the native half of L4 has none; the note says the gap room loses the track (same as H22, H26) | fix | fixed (7684f34): a blank frame and max_frames 2 in the gap room, its test in every case, the golden record regenerated -- decided by evan-wes, 2026-10-07 |
| A15 | should-fix | architecture | `vo.py:294`, `vo.cpp:181`, `vo.rs:269`, `edge-vo.md:160` | A2's stereo-matcher settings and `TH_LOW`/`TH_HIGH` were neither fixed nor deferred; the note says every threshold is a setting | decision | fixed (0c699ba): th_low, th_high and the stereo matcher's settings in Options (flat stereo_* fields, as the extraction's), passed through every vo source -- decided by evan-wes, 2026-10-07 |
| A16 | should-fix | architecture | ladder doc :454 | The ladder doc still says the edge's stage parity runs on fixture states | fix | fixed (8065548) -- decided by evan-wes, 2026-10-07 |
| A17 | should-fix | architecture | `rig-calibration.md:54`, `edge-vo.md:111` | The rig-calibration export has no `bf`, so an RGB-D run on it would now be refused | fix | fixed (8065548) -- decided by evan-wes, 2026-10-07 |
| A18 | should-fix | architecture | `tests/orchestration/conftest.py:23`, `backends.py:52`, `run_vo.py:211` | The stale-program check's crate list is hand-kept, ignores manifests, and guards pytest only | fix | fixed (e8b1338) -- decided by evan-wes, 2026-10-07 |
| A19 | note | architecture | `check.sh:82`, `setup_slamlab.sh:107` | The Rust extension has no rebuild in layer 1 and no stale check (predates the PR) | defer | deferred (ladder design doc, open items: "Deferred from PR 8's review panel"; 8065548) -- decided by evan-wes, 2026-10-07 |
| A20 | note | architecture | `generate_room_runs.py:29`, `test_vo.py:121` | Small gaps in the golden record: the copied tolerance, no recorded inputs, stale extra cases unnoticed, a generator importing a conftest (same as H28) | fix | fixed (7684f34) -- decided by evan-wes, 2026-10-07 |
| A21 | note | architecture | `generate_pose.py` | The weighted `refine_pose` fixture case cannot tell weighted from unweighted | fix | fixed (29134df) -- decided by evan-wes, 2026-10-07 |
| A22 | note | architecture | ladder doc :823, `options.rs` | A13's open item names C++ only; Rust lists the fields by hand too (with H21, H31) | fix | fixed (8065548: the open item; 7684f34: the tests) -- decided by evan-wes, 2026-10-07 |
| A23 | note | architecture | `map.py:751` | `to_camera` and `camera_centre` belong with A8's move to rung 0 | fix (the open item) | fixed (8065548) -- decided by evan-wes, 2026-10-07 |
| L13 | should-fix | algorithm | `initialization.py:95`, `pipeline.py:215` | A depth start from a frame with almost no depth now ends the run: ORB-SLAM's `N > 500` relies on a reset this pipeline does not have | decision | fixed (d7565ce): min_init_depth_points (100); the reset deferred (ladder design doc, open items: "Deferred from PR 8's review panel", milestone 4; 8065548) -- decided by evan-wes, 2026-10-07 |
| L14 | should-fix | algorithm | `edge-vo.md:255` | The note says the DSO row is a median of ten runs; ORB-SLAM3 reprints DSO's authors' figure | fix | fixed (8065548) -- decided by evan-wes, 2026-10-07 |
| L15 | note | algorithm | `edge-vo.md:223` | MH_01 stereo's 3.16 to 6.24 cm is start-frame variance: the pre-fix code has the same median over ten starts (5.17 against 5.14 cm) | decision (how results are reported) | fixed (8065548): the whole-sequence ATE is the median over five starts, with its range, a run that loses the track counted as a failure (decided by evan-wes, 2026-10-07); the 6 of 20 monocular runs that lose it deferred (ladder design doc, open items: "Deferred from PR 8's review panel", milestone 4) -- decided by evan-wes, 2026-10-07 |
| L16 | note | algorithm | `edge-vo.md:267` | MH_01 monocular's 11.2 cm is drift between the newly tracked first minute and the rest (3.77 cm on the frames the old run tracked) | fix | fixed (8065548) -- decided by evan-wes, 2026-10-07 |
| L17 | note | algorithm | `initialization.py:239,286,95`, `pipeline.py:185` | Four small departures in monocular initialisation remain (no reprojection test, counting after outliers, two off-by-ones, the next reference) | fix | fixed (d7565ce: ORB-SLAM's strict counts; 8065548: the other two documented) -- decided by evan-wes, 2026-10-07 |
| L18 | note | algorithm | `pnp.py:524`, `pnp.hpp:86`, `pnp.rs:548` | `refine_pose`'s weights are information in the scale's units, not relative | fix (the doc) | fixed (349dbb8) -- decided by evan-wes, 2026-10-07 |
| L19 | note | algorithm | `options.py:155`, `options.hpp:138`, `options.rs:145` | `min_epipole_distance2`'s docstring describes 100·s^(2l); the code uses 100·s^l, as ORB-SLAM | fix (the doc) | fixed (349dbb8) -- decided by evan-wes, 2026-10-07 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

**After the fixes.** Layer 1 passed at every fix commit and at `8065548` (2985 pytest, every
ctest and cargo test). No run 3: each fix answers one finding above, with a test where it
changes behaviour, and none changes a design. Applying L15's decision, the five starts showed
6 of 20 monocular runs losing the track; how a lost run counts in the median (as a failure)
and the losses themselves (relocalisation, milestone 4) were decided with the owner on
2026-10-07.

### Hygiene review

## Hygiene review

**Scope:** Run 2, `git diff 6151c45..e0e8208` on `feature/edge-vo` in the repository root. I read in full the Python edge package (`options`, `pipeline`, `tracking`, `mapping`, `initialization`, parts of `map` and `frames`), `tests/orchestration/*`, the golden-record generator and its JSON, the delta's diffs in `tests/edge`, `eval/tum`, `pnp`, `vo`, `scripts/`, and the C++ and Rust options, pipeline, `pnp`, `tum`, `vo` and frame tests. I also read `docs/notes/edge-vo.md`, the CHANGELOG, CLAUDE.md and the design-doc diffs. I spot-checked C++ and Rust use of every new Options field, and grepped the delta for privacy, debug leftovers and stale names. I ran three probes from the session scratchpad, outside the repo. The editable install re-ran its build step, and `git status` stayed clean.

**Summary:** The fixes are sound and mostly complete. The 20 Options fields are live in all three languages, and their defaults and ORB-SLAM citations match ORB-SLAM2 (`ThDepth`, `th = 5`, the 0.998, 2.5 and 4 radii, the 0.8 and 1.2 margins, 3.84, 0.01, the neighbour counts, the 0.9, 3 and 50 values). H1-H3, H8-H13, H16 and H18 check out. I found **no must-fix**. The should-fix items are test gaps and leftovers from H4, H5 and H7. On a strict reading of "tests exist for the behaviour", the first four findings would be must-fix. I kept them at should-fix because Run 1 graded the equivalent gap (H9) that way. The privacy grep on the delta is clean: no topics, frame ids, rig intrinsics or paths. The only `$HOME/...` paths are in the review report, as in earlier PRs' reports.

### Findings

#### [should-fix] The monocular parallax rule (L2) has no test that fails without it
- **Where:** `tests/edge/test_initialization.py:154-170` (`test_monocular_keeps_points_of_orb_slams_parallax`); also `:96` (`test_monocular_matches_the_reference`), the only check of the C++ and Rust versions.
- **Issue:** The docstring says "Every two-view point is kept, and the inliers between 0.36 and 1 degree too". The assertions are that kept contains the triangulated set, that all kept points are in front, and that their cosine is below `init_parallax_cos`. All of these hold under the old behaviour (keep only two-view's own points). The room's pair (frames 0 and 10) has no inlier between 0.36 and 1 degree.
- **Why it matters:** The core of L2's fix could be reverted in any language and nothing fails. Parity runs the same pair, so C++ and Rust are equally unexercised. The test also calls the private `_kept_points`.
- **Evidence:** My probe on that pair printed `status initialized inliers 69 triangulated 69 kept 69`. Kept equals triangulated.
- **Suggested fix:** Add a pair, or synthetic bearings, with near points above 1 degree and far points between 0.36 and 1 degree. Assert `len(kept) > triangulated` through `initialize_monocular` in every backend.

#### [should-fix] None of the 20 new settings is ever run off its default
- **Where:** `tests/edge/test_tracking.py`, `test_mapping.py`, `test_initialization.py` and `tests/orchestration/*`. The new helpers `maps.in_distance_range`, `to_camera` and `camera_centre` (`src/slamlab/edge/map.py:751-815`) and their C++ and Rust twins have no direct test either.
- **Issue:** Every stage-parity call passes `OPTIONS` with default values for the new fields. Only `ba_min_keyframes`, `min_init_features_depth`, `init_min_matches`, `pnp_min_inliers` and the motion windows are varied, and none of those are among the 20 new ones.
- **Why it matters:** A2's purpose was one definition per threshold, read by every language. A native that hard-codes 0.998, or swaps `min_distance_margin` and `max_distance_margin` while keeping the defaults, passes everything. The run-settings test pins defaults only.
- **Evidence:** `grep` for each new field name in `tests`, `cpp/tests` and `rust/*/tests` finds nothing. The native range tests (`cpp/tests/edge/test_map.cpp:172-175`, `rust/slamlab-edge/src/options.rs:549-562`) touch only `window`, `ref_range` and a NaN `chi2_mono`. I set each of the 67 fields to 0, -1 or NaN through `initialize_from_depth` with `backend="cpp"` and `"rust"`. All were rejected by name; `levels` and `scale_factor` were caught first by the pyramid check. So the hand-written C++ and Rust check lists are complete today, but nothing pins that.
- **Suggested fix:**
  - Run stage parity with each new setting moved so the Python result differs from the default result, asserting the difference, and compare the natives.
  - Add a table-driven test per language over `kIntegerOptions`, `kRealOptions` and the Rust settings, setting each field out of range and expecting `InvalidInput`. This also keeps A13's hand lists honest.

#### [should-fix] The L4 native fixes (velocity cleared on failure) are untested
- **Where:** `cpp/src/edge/pipeline.cpp:134`, `rust/slamlab-edge/src/pipeline.rs` (`record_failure`). The only test is Python's `tests/edge/test_pipeline.py:74`.
- **Issue:** The C++ and Rust pipelines never see a failed frame followed by a tracked one in any test. Whole-run parity and the golden record cover only rooms where every frame after the start is tracked.
- **Why it matters:** Deleting `velocity_.reset()` or `self.velocity = None` fails nothing.
- **Evidence:** In `tests/fixtures/edge/room_runs.json`, the only frames with 0 tracked are the monocular pre-start frames, and every gap case tracks every frame. The native pipeline tests (`cpp/tests/edge/test_pipeline.cpp:110-125`, `rust/slamlab-edge/tests/pipeline.rs:83-101`) only run empty frames until the track is lost.
- **Suggested fix:** Add a room variant with one blanked frame, so the gauge changes if the next prediction does. Put it in `datasets.py` and the golden record, or write a native test that tracks, fails once, then tracks.

#### [should-fix] `Frame.__post_init__` (H2's fix) is untested
- **Where:** `src/slamlab/edge/frames.py:177-199`; `tests/edge/test_frames.py` is untouched by the delta.
- **Issue:** The three refusals (descriptor count, non-float64 or non-array, wrong shape) are never exercised. Also `frame.{name}: expected a float64 array` does not name the dtype or type received.
- **Why it matters:** This is the new boundary check every stage relies on, and it has no test.
- **Evidence:** `grep -rn "one descriptor\|expected a float64 array" tests/` finds nothing.
- **Suggested fix:** A parametrised test of `replace(frame, depth=...)` with a float32 array, a wrong-length array and a missing descriptor, matching the messages. Put the received dtype or shape in the message.

#### [should-fix] H4 and H5 are not fully done
- **Where and evidence:**
  - Python still inlines the camera centre: `src/slamlab/edge/tracking.py:435` has `-predicted[:3, :3].T @ predicted[:3, 3]`. C++ and Rust use `camera_centre(predicted)` (`tracking.cpp:271`, `tracking.rs:363`).
  - Rust's new `check_setup` (`rust/slamlab-edge/src/frames.rs:105`) is used only by `tracking.rs:62,323,633`. The `check_camera(..)?; check_options(..)?;` pair is still written out at `mapping.rs:75-76,458-459,1055-1056`, `adjust.rs:238-239` and `pipeline.rs:112-113`. In C++ the pair remains in `adjust.cpp:182-185`.
  - H4's "in-view test written twice" was fixed for the distance half only. The cosine half is still written two ways per language: `view_cos >= limit` after dividing by distance (`tracking.py:318`, `tracking.cpp:237`, `tracking.rs:252`) against `offset @ normal >= limit * distance` (`mapping.py:643`, `mapping.cpp:482`, `mapping.rs:869`).
  - Literals Run 1 named are still bare: the `< 3` usable matches (`tracking.py:231`, `tracking.cpp:183`, `tracking.rs:181`) and `age >= 2` in point culling (`mapping.py:331`, `mapping.cpp:730`, `mapping.rs:262`). The Cauchy scale `sqrt(chi2_mono)/focal` (`tracking.py:226`) still has no comment saying why.
- **Why it matters:** These are the drift pattern H4 and H5 were meant to remove, and the resolutions mark them fixed.
- **Suggested fix:** Use `maps.camera_centre`; use `check_setup` at the remaining sites; add one `in_view_angle` helper per language; name the two counts (`p3p.SAMPLE_SIZE` for the 3) or comment them.

#### [should-fix] Docs the fixes left misstating the code
- **Where and issue:**
  - `src/slamlab/edge/initialization.py:4-6` says the first frame "with enough keypoints with depth" starts the map, "a point created at every one of them". After eb74c18 it counts every keypoint, with points only where depth exists, and uses `>=` (`test_from_depth_counts_every_keypoint` pins the boundary). `docs/notes/edge-vo.md:53` says "more than 500 keypoints", an off-by-one against the code.
  - `src/slamlab/edge/options.py:124-137`: the `...` chain is broken. `min_keyframe_inliers` ("...and still tracks more than this many (15)") now follows `ref_min_observations_early`, so it reads as nonsense. Make the entries self-contained.
  - `src/slamlab/edge/tracking.py:357-358`: `in_view`'s `options` is documented as "The viewing-angle limit", but it now takes the distance margins too.
  - The L1 level weighting appears only as a code comment (`tracking.py:233`, `tracking.cpp` `refine_round`). The module docstring (`tracking.py:16-19`), `optimize_pose`, and the C++ and Rust headers do not say that matches are weighted by `1/s^(2l)`.
  - `cpp/bindings/edge/edge_crossing.hpp:4-5` and `rust/slamlab-py/src/edge/edge_crossing.rs:3-4` cite the removed `slamlab.edge.map.frame_dict`. A7 moved it to `Frame.as_dict`. `grep -rn frame_dict` finds only these two.
  - `cpp/include/slamlab/edge/pipeline.hpp:112`: `record_failure`'s doc omits "ends the velocity". The Python and Rust docs were updated.
- **Why it matters:** The fixes changed behaviour and the docs next to it did not follow.
- **Suggested fix:** One-line edits at each place.

#### [should-fix] `test_the_gap_forces_a_recovery` asserts less than it claims
- **Where:** `tests/orchestration/test_vo.py:75-93`.
- **Issue:** The docstring says "every frame is tracked". The test checks `state == "tracking"`, which stays true for a frame that failed once (only 5 in a row lose the track). It also asserts the recovery only in Python, for TUM RGB-D.
- **Why it matters:** The test cannot catch a gap frame that goes untracked.
- **Evidence:** My probe ran all four gap cases: each takes the fallback and tracks all 30 frames (`tum rgbd relocalize calls [True, True] frames 30 tracked 30`; the same for the other three modes, with 23 tracked for the monocular ones). The test would not notice if that changed.
- **Suggested fix:** Add `assert len(run.timestamps) == len(run.diagnostics)`, and parametrise over `CASES`. The natives are held to Python's decisions by parity and the golden record.

#### [should-fix] H7 is half done in Rust
- **Where:** `rust/slamlab-eval/src/runs.rs:536,567,577`, `tum.rs:574,609`, `rust/slamlab-orchestration/src/vo.rs:804,826`.
- **Issue:** The names now include the PID and pre-clean, but cleanup is a trailing `remove_dir_all(...).unwrap()`. C++ got a drop guard (`ScratchDir`); Rust did not, although Run 1 asked for one.
- **Why it matters:** A failed assertion leaves `slamlab_*_<pid>` behind, and now each failing run leaves a new folder instead of reusing one fixed name.
- **Suggested fix:** A small `Drop` guard in each test module, or `tempfile` as a dev-dependency.

#### [note] The golden record is hard to review
- **Where:** `tests/fixtures/edge/generate_room_runs.py:64-76`, `room_runs.json`.
- **Detail:** The record is 130 kB and 8,343 lines (`json.dumps(indent=1)`: five lines per frame, about 24 per pose). The generator promises "its diff shows them [the decisions]". The repo has a layout helper for this, `tests/fixtures/foundations/_layout.py`. The `1e-9` tolerance is a literal copy of `SOLVER_TOLERANCE`. The gap expression `rendered[:GAP[0]] + rendered[GAP[1]:]` is written in both `tests/orchestration/conftest.py:91` and the generator (`:51`); a `gap_frames` helper in `datasets.py` would hold it once. The `tum/monocular` and `euroc/monocular` cases are identical.

#### [note] Smaller doc and error-message items
- `src/slamlab/edge/mapping.py:584-589`, `src/slamlab/orchestration/vo.py:10-14` and `docs/design/2026-10-05-edge-vo.md:134-137` have leftover wrap artefacts. The last one splits `` `tests/fixtures/edge/ room_runs.json` `` across a line break inside code.
- `aligned_view_cos` reads "within this cosine" in all three languages; it means a cosine above it. The C++ and Rust `max_frames` one-liners drop "once the map holds more than this many".
- `weights: expected positive values, one per correspondence` (`pnp.py:544`, C++ and Rust) names no value.
- The "one bf" message (`tum.py`, `tum.cpp:164`) reads as if `bf` were required; it is optional.
- `cpp/tests/foundations/fixture_support.hpp` now holds `ScratchDir`, but its file brief says it only reads fixtures.
- `test_frames_need_no_ground_truth` (`tests/eval/test_tum.py:139`) now also tests the optional `bf`.
- `tests/orchestration/datasets.py` imports `OPTIONS` from a conftest.
- `_tilted_view`'s `while fitted < 2000` loop (`tests/foundations/test_homography.py`) has no bound.

#### [note] The gap room recovers twice, not once
- **Detail:** In my probe, each case calls the fallback twice. The recovery frame's velocity spans the ten dropped frames, so the next frame is also predicted ten frames ahead and also falls back. This is a property of the fixture, not a defect, and consecutive real frames would not do it. It is worth knowing when reading the golden record (algorithm reviewer's lane).

#### [note] A13 (C++ and Rust check lists by hand) grew by 20 entries
- **Detail:** The deferral stands. My probe shows the lists complete today (see the second finding). The table-driven test suggested there closes it cheaply.

### Checked and fine
- **Options docs against ORB-SLAM2:** the 20 new fields' values and citations, and `Options`'s `max_frames` doc against `pipeline` (`recent`, `held`). The Python, C++ and Rust defaults match. The Python and native range classes (fractions versus positives) match field for field.
- **Wiring:** each new field is read in all three languages, spot-checked at the code sites (`tracking`, `mapping`, `initialization`, `pipeline`).
- **C++ and Rust consistency:** C++ and Rust `need_keyframe`, `track_local_map` and `initialize_monocular` mirror the Python rules, including the monocular ratio override and `ba_min_keyframes`.
- **H1:** `max_frames` is documented correctly everywhere; `RECOVERY_WIDEN` is gone.
- **H10:** exit codes have an explicit `exit_code` in C++ and Rust, with a unit test in each and a Python fake-program test for 3, 4 and 5.
- **H9, H11, H12, H13, H16, H18, A4-A7:** the native frame tests, `require_backend("cpp")`, the eval and CLAUDE.md descriptions, token-naming parse errors (asserted in both natives), `last_tracked` removal, the design-doc module list, the `mode` convention text, and `--target-dir rust/target` with the stale-program check all hold. The freiburg2 and freiburg3 fixtures hold the `bf`.
- **`pnp.refine_pose` weights:** the three implementations are equivalent, with documented shapes, validation, and a SciPy-referenced weighted test.
- **Dataset and run tests:** `datasets.py` and `conftest.py` are clean. The settings test still pins every native default through `run.settings == options.as_dict()`.
- **Version docs:** CHANGELOG 0.3.2.0's findings count (1, 23, 20) and ATE and timing ranges match the table. The note's start frames, 96% track rates and ORB-SLAM comparison all agree with the table. CLAUDE.md's edge-fixture policy matches the code.
- **Leftovers:** no debug prints, TODOs or non-test `unwrap`/`expect` in the delta; the `print` in `generate_room_runs.py` is CLI output. Every `#[allow]` carries a reason.

### Architecture review

## Architecture review

**Scope:** Run 2 of PR 8 (`feature/edge-vo`). I reviewed the delta `git diff 6151c45..e0e8208`: 9 commits, 100 files, in repo the repository root.
- **Read in full:** the doc changes (CLAUDE.md, the ladder doc, the edge-vo design, the note), every Python, C++ and Rust edge source and binding diff, rung 0's `refine_pose` change in all three languages, the eval readers' `bf` change, `orchestration/vo` in all three languages, the scripts, the golden record and its generator, `tests/orchestration/{datasets,conftest,test_vo}.py`, and the Run 1 report.
- **Probes:** three read-only probes of the room, run under a 4G cap with outputs in a temp dir; the working tree stayed clean.
- **Not done:** I did not re-run layer 1.

**Summary:** No must-fix. The fixes for A1, A2, A4–A7, A9 and A12 are correct and follow the owner's decisions:
- live parity is documented and a golden record is committed, with every backend held to it;
- the thresholds moved into `Options`;
- `bf` is a `camera.txt` key, with `SENSOR_BF` for the freiburg sensors, and an RGB-D run without it is refused.

The three languages stay in lock-step: every new setting is used in all three, and the backends agree on the room to about 1e-14. The five should-fixes are about what the fixes did not carry through:
- A3 is only half done: the failure path, including the native half of L4, and the keyframe hold run in no test across backends.
- Part of A2 was dropped without being fixed or deferred.
- Two design docs still describe the old behaviour.
- The stale-program check depends on a hand-kept list of crates.

### Findings

#### [should-fix] The failure path and the keyframe hold run in no cross-backend test; the native half of L4 has no test at all (A3's remainder)
- **Where:**
  - `tests/orchestration/datasets.py:106-110` (`GAP`, `GAP_OPTIONS`)
  - `tests/orchestration/test_vo.py:75`
  - `src/slamlab/edge/pipeline.py:264` (`held = recent and m.keyframes > options.max_frames`)
  - `cpp/src/edge/pipeline.cpp:147` and `rust/slamlab-edge/src/pipeline.rs:303` (L4's `velocity` reset)
  - `docs/notes/edge-vo.md:180-181`
- **Issue:**
  - Run 1's A3 listed five parts of the pipeline loop that no test compared across backends. The gap room now covers the PnP recovery and the widened window.
  - It never reaches `record_failure`: the frame after the motion model fails is always placed by the fallback. So no test, in any language, holds L4's velocity reset in C++ or Rust. Commit c22e30e added a Python test only, and the native pipeline tests only feed empty frames until the track is lost.
  - The held-keyframe rule never decides: the room's maps hold at most 7 keyframes, against `max_frames = 30`.
  - The note says "every mode loses the track and recovers"; in fact no frame is lost or counted as failed.
- **Why it matters:** these are the rules that L4 and L5 just changed in three languages at once. Only a cross-backend run or the golden record would catch one language drifting, and neither reaches them.
- **Evidence:** three probes on the room, in the scratch dir; two are saved as `edge_gap_blank_frame_*.py`.
  - **The gap room as it is**, Python, in all four `CASES`: every case shows `relocalize: 2, relocalized: 2, failures: 0`.
  - **One blank frame added** (zeros, no depth), inserted at index 10 of the gap room. I compared Python with L4's rule against the rule before L4:
    ```
    tum rgbd       decisions differ at [11, 12, 14]      max pose diff 0.0037
    tum monocular  decisions differ at [11, 23, 25, 27]  max pose diff 0.0016
    euroc stereo   decisions differ at [11]              max pose diff 0.0029
    ```
    So this variant would catch a regression of L4 in any one language.
  - **The native programs today**, on the gap room with a blank frame after the gap: C++ and Rust make the same decisions as Python, with poses within 7e-15. L4 is correct in all three languages; only the test is missing. With `max_frames=1`, the gap room changes monocular's keyframe decisions, so the hold then decides.
- **Suggested fix:**
  - Add one blank frame mid-run to the gap room, and lower `max_frames` in `GAP_OPTIONS` so the map holds more keyframes than it at the recovery.
  - Regenerate the golden record.
  - Have the gap test also assert that a failure was recorded.
  - Reword the note: "the motion model fails and the fallback recovers".

#### [should-fix] A2's stereo-matcher settings were neither fixed nor deferred, and the note now says every threshold is a setting
- **Where:**
  - `src/slamlab/orchestration/vo.py:294` (`frames.stereo_pair(...)` without `stereo_options`)
  - `cpp/src/orchestration/vo.cpp:181` (`MatchOptions{}`)
  - `rust/slamlab-orchestration/src/vo.rs:269` (`MatchOptions::default()`)
  - `docs/notes/edge-vo.md:160-162`
- **Issue:** Run 1's A2 named the stereo matcher's settings explicitly; they run at each language's own defaults. The resolution records only "the thresholds in `Options`; `bf`", and the ladder doc's open items do not mention them. Meanwhile the note now says: "Every threshold is a setting … 67 in all, so each is recorded in a run's `run.json` and defined once per language." That is not true of:
  - the matcher's Hamming limit 75, band 2.0, SAD window 5, search 5 and outlier factor 2.1;
  - `TH_LOW` / `TH_HIGH`, still defined three times.
- **Why it matters:** a stereo run's `run.json` does not record what matched its pairs, and part of a finding disappeared without a decision.
- **Evidence:** the three calls quoted above. `grep -n "stereo" docs/design/2026-10-02-slamlab-ladder.md` shows no open item for them.
- **Suggested fix:** the owner decides. ORB-SLAM2 also hard-codes these in `ComputeStereoMatches` and `ORBmatcher`, so I recommend (b).
  - **(a)** Add `stereo_*` settings to `edge.Options` and pass them through all three `vo` sources.
  - **(b)** Record the item in the open items, and qualify the note: "every edge threshold; the stereo matcher's and the descriptor thresholds are ORB-SLAM's constants".

#### [should-fix] The ladder design doc still says the edge's stage parity runs on fixture states
- **Where:** `docs/design/2026-10-02-slamlab-ladder.md:454`
- **Issue:** the line reads: "Parity is held per stage, on fixture states, and per sequence … the three backends' ATEs within a stated tolerance of each other." A1's decision amended CLAUDE.md and edge-vo §3. What was built is different: states built from the room when the tests run, the same decisions and poses within 1e-9, and a golden record.
- **Why it matters:** the ladder doc is the authority, and the next rungs will plan their parity from it. My Run 1 suggested fix named only CLAUDE.md and edge-vo §3, so this gap is mine.
- **Evidence:** the line as quoted; edge-vo §3's "As built (PR 8's panel, A1)" says otherwise.
- **Suggested fix:** amend the doc (the code is right) with one sentence pointing to edge-vo §3.

#### [should-fix] The rig-calibration design's export has no `bf`, so an RGB-D run on it would now be refused
- **Where:**
  - `docs/design/2026-10-05-rig-calibration.md:54-57`
  - `docs/design/2026-10-05-edge-vo.md:111-113`
- **Issue:** the export is specified as "a `camera.txt` with the model, intrinsics, image size and depth scale … so every backend's VO runs on the rig". Since ace10d7, every backend refuses an RGB-D run on such a folder with "an RGB-D run needs the depth sensor's bf". The edge-vo design's native-readers line lists the same four keys.
- **Why it matters:** the side track would be built to a spec that no longer works. This was the very case that motivated A2: `bf` belongs to the sensor.
- **Evidence:**
  - `vo.py` `_tum_source` and `vo.cpp` `tum_source` refuse when `bf` is missing;
  - `test_rgbd_needs_the_depth_sensors_bf` checks the refusal for every backend.
- **Suggested fix:**
  - Amend the rig design: the export writes `bf`, taken from the rig config, which lives outside the repo; the repo's template holds a placeholder, so no rig value enters the repo.
  - Say what `bf` means for a depth sensor: the virtual baseline times fx, which sets the close-depth threshold and the virtual right column.
  - Add `bf` to edge-vo's native-readers line.

#### [should-fix] The stale-program check's crate list is hand-kept, and only pytest gets the check
- **Where:**
  - `tests/orchestration/conftest.py:23-38`
  - `src/slamlab/backends.py:52`
  - `scripts/run_vo.py:211`
- **Issue:**
  - `RUST_PROGRAM_CRATES` copies by hand the path dependencies in `rust/slamlab-orchestration/Cargo.toml`. A crate the program gains later (for example at milestone 4) would silently escape the check.
  - Only `src/**/*.rs` is compared; changes to `Cargo.toml` and `Cargo.lock` are not.
  - The checkout's `rust/` path is worked out a second time, beside `backends._RUST_PROGRAMS`.
  - `scripts/run_vo.py --backend rust`, which produces the note's numbers, runs whatever binary is there.
- **Why it matters:** a guard against silent staleness that can itself go stale silently.
- **Evidence:** the tuple at `conftest.py:23-28`; `conftest.py:35` `Path(__file__).resolve().parents[2] / "rust"`.
- **Suggested fix:**
  - Derive the crates from the manifests: read them with `tomllib` and follow the `path =` dependencies from `slamlab-orchestration`.
  - Include each crate's `Cargo.toml` and `rust/Cargo.lock` in the comparison.
  - Consider putting the check behind `backends.program("rust", …)`, raising a "stale" `BackendNotBuiltError` subclass. Then `run_vo.py` and pytest share one check, and the path lives in one place.

#### [note] The Rust extension has no rebuild in layer 1 and no stale check
- **Where:** `scripts/check.sh:82-91`, `scripts/setup_slamlab.sh:107`
- **What:** every `backend="rust"` stage-parity test goes through `slamlab_rs`, which is built only by `setup_slamlab.sh`'s `maturin develop`. After a Rust edit, `check.sh` rebuilds the program, so the whole-run and golden tests are current, while stage parity runs the old extension.
- **Status:** this predates the delta; A5's fix only made the asymmetry visible. Offline CI builds from clean, so it is not affected.
- **Suggestion:** have `check.sh` run `maturin develop --release -m rust/slamlab-py/Cargo.toml` before pytest, or apply the same staleness check to the extension.

#### [note] Small gaps in the golden record
- **Where:** `tests/fixtures/edge/generate_room_runs.py:29,69-72`, `tests/orchestration/test_vo.py:121`
- **What:**
  - The generator copies 1e-9 instead of importing `SOLVER_TOLERANCE`, which its own reason string names.
  - The record holds no inputs (`FRAMES`, `GAP`, the two settings). A failure after a change to `tests/edge/conftest.py`'s `OPTIONS` or to the renderer therefore reads as a pipeline change. Recording them, and asserting them first, would say which changed.
  - Only the parametrised cases are looked up, so a stale extra case in the record goes unnoticed.
  - The generator is a tool run outside pytest, but it imports constants from a pytest `conftest`. That conftest pulls in `tests.conftest` and its environment-driven `REQUIRED_BACKENDS`. `FRAMES` and `OPTIONS` would sit better in `tests/edge/support.py`.

#### [note] The weighted `refine_pose` fixture case cannot tell weighted from unweighted
- **Where:** `tests/fixtures/foundations/generate_pose.py` (the case "noise-free, 20 points, weighted by pyramid level")
- **What:** on exact correspondences every positive weighting has the same minimum, the true pose, so this case cannot catch a backend that ignores or misapplies the weights. The check that can is `test_refining_matches_scipy_least_squares[weighted=True]`: noisy data, every lock-step backend, and SciPy is a dev dependency, so it always runs. A noisy weighted case with its expected cost would make the fixture discriminating. This is mostly the algorithm reviewer's lane.

#### [note] A13's open item names C++ only; Rust lists the fields by hand too
- **Where:** `docs/design/2026-10-02-slamlab-ladder.md:823`; `rust/slamlab-edge/src/options.rs` (`check_options`, `named_settings!`)
- **What:**
  - Both languages cover all 67 fields today:
    - C++: integers 3 + 32 = 35, reals 18 + 14 = 32;
    - Rust: 65 by name, plus `fast_threshold` and `min_fast_threshold` through `orb::check_options`.
  - The fixes added 23 fields to each hand-kept list, so the deferred risk grew.
  - Widen the item to name Rust.

#### [note] `to_camera` and `camera_centre` belong with A8's move to rung 0
- **Where:** `src/slamlab/edge/map.py:751,769`, with twins in `map.hpp` and `map.rs`; `mapping._world`, their inverse, stays in `mapping.py`.
- **What:** these are generic SE(3) helpers. Rung 0's `se3.act` validates its input and takes one transform per point, so an unchecked one-pose helper in the edge is justified for the hot loops. When A8 lifts the camera to rung 0 at milestone 4, these should go too. Add them to A8's open item.

### Checked and fine
- **A1:** the live-parity policy is in CLAUDE.md and edge-vo §3. `tests/fixtures/edge/room_runs.json` states its tolerance policy and is applied by `assert_matches_fixture`. Every backend, Python included, is held to it, and regenerating it is one documented command.
- **A2:** 23 thresholds moved into `Options` in all three languages, with the same names and defaults. A script check found each new setting read in all three languages, and the settings test holds the native defaults to Python's.
- **A2's `bf`:**
  - It is optional in all three readers, with the same validation.
  - `SENSOR_BF` / `kSensorBf` are used for the freiburg sensors, matching ORB-SLAM2's `TUM1/2/3.yaml`.
  - `sequences.json` holds `bf`, including `null`, for all three languages.
  - An RGB-D run without it is refused by every backend.
- **A4:** the ladder doc's amendment and the C++ `Mode` comment.
- **A5:** `--target-dir rust/target` in both scripts (`check.sh` changes to the repo root first); the documentation; the build runs before pytest.
- **A6:** the freiburg2 and freiburg3 fixture folders are compared exactly in C++ and Rust.
- **A7:** `Frame.as_dict`.
- **A9:** an explicit `exit_code` switch in `vo.hpp`; codes 3, 4 and 5 are tested.
- **A12:** §2 is marked as the plan as it stood.
- **Rung 0's `refine_pose`:**
  - the weights are validated in all three cores with the same error kind and message;
  - Python's argument is optional, the natives' required;
  - `pnp_ransac` passes ones;
  - the only caller is `edge.tracking`, in all three languages;
  - the `Implementation` Protocol, the thin bindings and the fixture were all updated.
- **Dependency direction:** `map → options` and `frames → options` stay within the edge's declared internal order. `check_setup` is in the native `frames`, `check_mode` in `options`. There are no new crates, libraries or runtime dependencies, nothing imports reifyscene, and no GPU dependency entered the edge.
- **Monocular initialisation:** splitting it into a match stage and an initialise stage mirrors ORB-SLAM's `MonocularInitialization`. The Pipeline carries the matched positions identically in all three languages, and the bindings only convert types.
- **The gap room's recovery:** it forces two recoveries in all four cases, and the backends agree.
- **Deferred items:** all 11 deferred findings appear in the ladder doc's open items, described accurately (A13's wording aside, above).
- **Privacy:** nothing rig-specific in the delta. The synthetic room's `camera.txt` uses TUM's public `bf` of 40. The committed review report contains absolute local paths (the home directory and the scratch dir); they are harmless while the repo is private and worth a sweep before it goes public.

### Algorithm review

## Algorithm review

**Scope:** Run 2 covers the delta `6151c45..e0e8208` on `feature/edge-vo` (PR 8). I read the code changes in c22e30e (L4), eb74c18 (L5, L7, H1), 87acf3d (L1), 9c521de (L2), ace10d7 (A2/H4 Options and `bf`), aee51bf (the L10 test) and 09eb6a2 (the gap room and the golden record), in Python, C++ and Rust. I also read the note at c0f8289 and the report at e0e8208.

**References and method:**
- The ORB-SLAM2 sources were fetched today (raulmur/ORB_SLAM2 master: `Tracking.cc`, `LocalMapping.cc`, `Optimizer.cc`, `Initializer.cc`, `ORBmatcher.cc`, `Examples/*.yaml`), with ORB-SLAM3's `Tracking.cc` and the text of the ORB-SLAM3 paper (arXiv:2007.11898).
- For the results I ran 39 whole-sequence runs on public EuRoC MH_01 and TUM fr1/desk. They used the Rust `slamlab_vo`, built from `git archive` copies of 6151c45 and e0e8208 in the session scratchpad; one copy was patched to revert L1. The repo's working tree was not touched (`git status` is clean), and no background process is still running.

**Summary:**
- Every fix Run 1 resolved as fixed (L1, L2, L3, L4, L5, L6, L7, L8, L10) is correct and matches the ORB-SLAM2 lines it cites. Its intent is the same in all three languages. There is no must-fix.
- **Two should-fix:**
  - L5's "count every keypoint" depth start copies ORB-SLAM's `N > 500` but not the reset that makes it safe. A first frame with almost no depth now starts a map of 0 to 17 points and ends the run (demonstrated below).
  - The L3 rewrite of the note added a new misstatement about the DSO row.
- **On the results:** the MH_01 stereo change from 3.16 to 6.24 cm is the start frame, not a fix.
  - The pre-fix code over ten starts has a median of 5.17 cm (range 3.16 to 7.47); HEAD's is 5.14 cm (range 4.11 to 6.24).
  - Reverting L1, L7 and L8 together at the old start reproduces 3.16 cm exactly.
  - The note's reading in "How much one run says" is sound, and these measurements make it stronger.

### Findings

#### [should-fix] A depth start from a frame with almost no depth now ends the run: ORB-SLAM's `N > 500` relies on a reset this pipeline does not have
- **Where:**
  - The start: `src/slamlab/edge/initialization.py:95-97`, `cpp/src/edge/initialization.cpp:142`, `rust/slamlab-edge/src/initialization.rs:127`.
  - The lost rule that ends the run: `src/slamlab/edge/pipeline.py:215-220`, `cpp/src/edge/pipeline.cpp:145`, `rust/slamlab-edge/src/pipeline.rs:304`.
  - `tests/edge/test_initialization.py:56` starts a 5-point map on purpose.
- **Issue:**
  - eb74c18 counts every keypoint, as ORB-SLAM2's `StereoInitialization` does (`Tracking.cc:511`, `if(mCurrentFrame.N>500)`), and makes points only where there is depth.
  - ORB-SLAM can afford that rule because a track lost soon after initialisation resets the system:
    - ORB-SLAM2 `Tracking.cc:474-476`: `if(mpMap->KeyFramesInMap()<=5) { ... "Track lost soon after initialisation, reseting..." ... mpSystem->Reset();`
    - ORB-SLAM3 `Tracking.cc:2019-2021`: `if (pCurrentMap->KeyFramesInMap()<10) mpSystem->ResetActiveMap();`
  - The lite pipeline has no reset: after `lost_frames` failures the run ends. The old floor of 500 keypoints *with depth* did the reset's job. Removing it without adding the reset makes a first frame with little valid depth fatal, where the pre-fix code, ORB-SLAM2 and ORB-SLAM3 would all wait or restart.
  - The same gap applies to an early monocular loss. L2 now starts monocular maps earlier, from lower-parallax points.
- **Why it matters:** Several kinds of first frame would end the run:
  - an RGB-D frame facing a far wall or a window;
  - a dark or textureless near field;
  - a stereo pair whose matching fails on a repeated pattern.

  In every case a map of a few points is accepted as initialised, and after `lost_frames` frames the run is over. A map of **zero** points is also accepted.
- **Evidence:** `edge_sparse_depth_start_probe.py` uses the room in RGB-D mode at HEAD. Frame 0's depth map is blanked except at K random pixels:
  ```
  keep=None: frame 0 keypoints 605, with depth 605, state tracking, map points 605 -> ended after 25 frames, tracked 25, final state tracking
  keep=2000: frame 0 keypoints 605, with depth 17, state tracking, map points 17  -> ended after 6 frames, tracked 1, final state lost
  keep=200:  frame 0 keypoints 605, with depth 1,  state tracking, map points 1   -> ended after 6 frames, tracked 1, final state lost
  keep=20:   frame 0 keypoints 605, with depth 0,  state tracking, map points 0   -> ended after 6 frames, tracked 1, final state lost
  ```
- **Suggested fix:**
  - **Recommended:** implement ORB-SLAM's remedy. When the track is lost with at most 5 keyframes, go back to `NOT_INITIALIZED` and drop the map instead of ending the run, in all three languages. Add a room test and a line in the note's step 1. This also covers an early monocular loss.
  - **If that is out of this PR's scope:** keep a floor on keypoints with depth as a documented lite stand-in for the reset (for example `min_new_points` = 100, or a new setting), and defer the reset to the backlog.

#### [should-fix] The note now says the DSO row is a median of ten runs; ORB-SLAM3 reprints it as DSO's authors reported it
- **Where:** `docs/notes/edge-vo.md:255-256`.
- **Issue:** The note reads: "ORB-SLAM3's EuRoC numbers, and the DSO row beside them (Campos et al., T-RO 2021, Table II), are medians of ten runs over every frame." The paper says otherwise:
  - Table II's caption: "Except where noted, we show results reported by the authors of each system, for all the frames in the trajectory, comparing with the processed GT."
  - §VII-A: "Our reported values are the median after 10 executions."

  Only ORB-SLAM3's own rows are medians of ten. The DSO row is DSO's authors' figure, and ORB-SLAM3 does not state how it was obtained.
- **Why it matters:** The L3 fix introduced this, in the paragraph whose whole purpose is a like-for-like comparison.
- **Evidence:** arXiv:2007.11898, p. 12: Table II caption and §VII-A (from pdftotext of the arXiv PDF).
- **Suggested fix:** "ORB-SLAM3's EuRoC numbers are medians of ten runs over every frame; the DSO row beside them is DSO's authors' figure, as ORB-SLAM3 reprints it."

#### [note] MH_01 stereo going from 3.16 to 6.24 cm is start-frame variance: the pre-fix code has the same median over ten starts
- **Where:** `docs/notes/edge-vo.md:223-234` ("How much one run says") and `:265-266`.
- **Detail:** The note measures HEAD's spread but not the pre-fix code's, so it shows that one run says little without showing that the fixes left the distribution alone. I measured both. The settings are the note's: 1200 features; `close_depth` 35 and `max_frames` 20 at HEAD; 40 and 30 for the pre-fix code, as its run used. The C++ and Rust programs take identical decisions.

  ATE over every frame, cm:

  | Code | First frame | ATE (cm) | Median |
  |---|---|---|---|
  | HEAD e0e8208 | 0-9 | 6.24 5.73 4.11 4.58 5.44 5.62 5.33 4.95 4.77 4.79 | **5.14** |
  | pre-fix 6151c45 | 1, 2, 3, 4, 28, 29-33 | 3.16 3.75 3.87 6.34 4.88 5.46 5.55 6.21 7.47 4.71 | **5.17** |
  | HEAD, L1 reverted (unit weights) | 0-4 | 5.12 5.25 6.41 4.55 6.26 | 5.25 |
  | HEAD, L7+L8 reverted (`ba_min_keyframes=2`, 40/30) | 1-5 | 5.74 4.52 5.64 4.87 5.48 | 5.48 |
  | HEAD, L1+L7+L8 reverted | 1 | 3.16, 539 keyframes: the pre-fix run exactly | |

  The pre-fix depth start needs 500 keypoints with depth. Frames 5 to 27 have fewer, so its starts 5-9 all begin at frame 28; I used starts 29-33 for five more distinct ones.

  What the runs show:
  - **3.16 cm was the pre-fix code's best of ten starts.** HEAD's median is the same and its spread is narrower.
  - **Part of the change is L5 itself.** It moved MH_01's first keyframe from frame 1 (501 points) to frame 0 (477 points), which is a start perturbation.
  - **No single fix is a systematic cause.** Reverting L1 alone, or L7 and L8 alone, leaves the median at 5.3-5.5 cm; each rule change moves one run by 1 to 2 cm either way.
  - **The rest of the delta is neutral over the whole sequence.** Reverting L1, L7 and L8 together reproduces the old run bit for bit. So L4, the recovery window and the Options/`bf` refactors leave all 3682 frames unchanged; the commits had checked them only on the first 300.
  - **L4 barely acts on MH_01.** Every run's failures lie in frames 3662-3667.
  - **fr1/desk RGB-D (starts 0, 18-21):**
    - pre-fix: 2.36 2.95 2.85 3.17 3.22 (median 2.95);
    - HEAD: 3.43 2.56 3.13 2.98 3.16 (median 3.13);
    - HEAD with L1 reverted: 3.53 3.99 2.58 3.14 2.82 (median 3.14).

    The old 2.36 was again its best start, and the medians differ by 0.2 cm within a 1 cm spread.
  - **L1's Monte Carlo gain (+24% without weights) does not show at sequence level.** It is neutral on both sequences. That is worth one sentence in the note, so the weighting is read as the right noise model rather than a measured accuracy gain.
- **Suggestion:** Add one sentence with the pre-fix spread (the same median), and for the long sequences give the median over starts as the headline number.
- **Evidence:** the run folders are in the session scratchpad. Scoring script: `edge_score_runs.py`. Sweep driver: `edge_start_sweep.sh` (archives a revision, optionally patches it, builds it and runs N starts).

#### [note] MH_01 monocular's 11.2 cm is drift between the newly tracked first minute and the rest, not worse tracking
- **Where:** `docs/notes/edge-vo.md:267-269` and the table row.
- **Detail:** L2 made the run start at frame 140, which adds 1254 tracked frames (the first 63 s).
  - Over the frames the pre-fix run tracked (t ≥ 1403636649.41), HEAD's error is **3.77 cm**; the pre-fix run had 3.72 cm on the same frames.
  - The newly tracked segment alone, under its own Sim(3), is **1.45 cm**. The two segments' Sim(3) scales are 0.534 and 0.518.
  - The 11.2 cm comes from aligning both segments with one similarity: the drift built up through the nearly stationary stretch from 20 to 40 s (0.57 m of ground-truth path) sits between them.

  Saying this beside "against ORB-SLAM3's 1.6 cm" would show that the comparison now includes MH_01's slow start, which ORB-SLAM3's loop closure and map merging absorb. Separately, "close to ORB-SLAM" for 2.47 against 1.69 cm on fr1/desk (46% above) is generous wording.
- **Evidence:** `edge_mono_segment_split.py` (all 11.2; early 1.45; late 3.77).

#### [note] Monocular initialisation: four small departures from ORB-SLAM2 remain, none of which binds on MH_01
- **(a) No reprojection test on kept points.** `_kept_points` (`initialization.py:239-251`, and C++/Rust `kept_points`) has none. CheckRT drops points that reproject beyond th2 = 4σ² (`Initializer.cc:874, 885`; th2 = `4.0*mSigma2` at `:494-497`).
- **(b) "100 points after BA" counts after outliers are forgotten.** The edge's initial BA is local BA's two passes (20, then 10 iterations) and forgets chi-square outliers before counting (`initialization.py:286-297`). ORB-SLAM2's `GlobalBundleAdjustemnt(map,20)` forgets none (`Optimizer::BundleAdjustment` has no `EraseObservation`), then checks `TrackedMapPoints(1)<100` (`Tracking.cc:692`). The comment at `:297` is right about the threshold but not about what is counted.
- **(c) Off by one at both thresholds.** ORB-SLAM starts on `N>500` (`Tracking.cc:511`) and keeps a reference on `mvKeys.size()>100` (`:569`, `:590`). The edge accepts exactly 500 and exactly 100 (`initialization.py:95`, `pipeline.py:185`), and the note's step 1 says "more than 500".
- **(d) The next reference after too few matches.** With fewer than 100 matches ORB-SLAM deletes the initializer, so the *next* frame becomes the reference (`Tracking.cc:603-608`); the edge uses the current frame. The note already documents this.
- **Evidence:** `edge_mono_init_rules_probe.py` on MH_01 frames 0-140. At all 21 pairs that two-view initialises, 0 kept points reproject beyond 2 px, and the points after the initial BA equal the points kept (72 to 101). The map starts at frame 140, as the note says.
- **Suggested fix:** Document (a) and (b) as lite departures; align (c) or document it.

#### [note] `refine_pose`'s weights are absolute information in the scale's units, not relative; uniformly tiny weights stop on the absolute gradient tolerance
- **Where:** `src/slamlab/foundations/pnp.py:524`, `cpp/include/slamlab/foundations/pnp.hpp:86`, `rust/slamlab-foundations/src/pnp.rs:548` ("its information relative to the others").
- **Detail:**
  - With a robust loss, multiplying every weight by k is the same as dividing `scale` by √k, so a common factor changes the answer.
  - The NLS gradient tolerance (1e-10, absolute, as in Ceres) makes uniformly tiny weights stop early while reporting converged.
  - The edge is correct and unaffected: it passes 1/s^(2l) in [0.078, 1], with level 0 at 1 and the scale in level-0 whitened units.
- **Evidence:** `pnp_weight_scale_probe.py` (60 points, 20% outliers):
  ```
  w=1, scale s           centre error 6.791 mm  iterations 7   converged True reason function
  w=4, scale s           centre error 8.985 mm  iterations 10  converged True reason function
  w=1, scale s/2         centre error 8.985 mm  iterations 10  converged True reason function
  w=1e-8, scale s*1e-4   centre error 5.565 mm  iterations 3   converged True reason gradient   (same problem up to a cost factor)
  w=1e-12, scale s       centre error 61.644 mm iterations 0   converged True reason gradient   (never moved)
  ```
- **Suggested fix:** Change the doc to "information in the units of `scale` (whitened); a common factor k on the weights acts as 1/√k on the scale". Optionally, scale the gradient tolerance by the initial cost.

#### [note] `min_epipole_distance2`'s docstring describes a different rule from the code
- **Where:** `src/slamlab/edge/options.py:155-157`, `cpp/include/slamlab/edge/options.hpp:138`, `rust/slamlab-edge/src/options.rs:145`.
- **Detail:** "At least this squared distance from the epipole, in pixels at its level" reads as 100·s^(2l). The code uses 100·s^l (`mapping.py:409`), as ORB-SLAM does (`ORBmatcher.cc:747`, `100*pKF2->mvScaleFactors[kp2.octave]`). The code is right; the docstring should say "100 times the level's scale factor s^l (ORB-SLAM's mixed units)".

### Checked and fine
- **L4 (c22e30e):** All three languages clear the velocity in `record_failure`. `accept` still requires `failures == 0`, so the first frame after a failure also predicts no motion. That is the lite counterpart of ORB-SLAM2 using `TrackReferenceKeyFrame` while `mnId < mnLastRelocFrameId+2` (`Tracking.cc:307`). The test checks both frames.
- **L5/H1 (eb74c18):**
  - In every mode, th = 5 on the recovery frame and the next (`Tracking.cc:1189-1190`).
  - The radius is th · {2.5, 4} · s^l, as `SearchByProjection` computes it.
  - 50 inliers within `max_frames`, and held keyframes once nKFs > mMaxFrames.
  - The `max_frames` doc is right: c1b holds when mapping is idle, so c2 decides.
- **L7 (eb74c18):** BA runs when `keyframes >= 3`, counted after insertion and before keyframe culling. That matches `LocalMapping.cc:80` (`KeyFramesInMap()>2`). Culling still runs every time, and the code is identical in all three languages.
- **L1 (87acf3d):**
  - The cost is ρ(w‖r‖²), J and r are scaled by √w, and the IRLS weights come from the whitened norms. That equals g2o's kernel on eᵀΩe with Ω = invSigma2·I (`Optimizer.cc:299-300`, δ = √5.991). The relative function tolerance is unaffected.
  - The `variance[use[matched]]` indexing is correct.
  - SciPy is the oracle for every backend.
  - Weights of 0, inf or the wrong length are refused.
  - `pnp_ransac` stays unweighted.
- **L2 (9c521de):**
  - The search starts at `vbPrevMatched` (`ORBmatcher.cc:425`), moves with each match (`:517`) and resets with the reference.
  - Kept points are inliers in front of both views with cos < 0.99998 (`Initializer.cc:857-892`).
  - The matches are passed in rather than re-matched.
  - C++ and Rust `kept_points` and `set_reference` match Python.
- **Options (ace10d7):** I checked every ORB-SLAM-attributed default against the sources and the Examples yaml.
  - Windows 15/7/1/3/5, factors 2.5/4 at 0.998, ratios 0.9/0.8/0.75, margins 0.8/1.2, view cosine 0.5.
  - Inliers 10/30/50, `thRefRatio` 0.4/0.75/0.9, nMinObs 2/3, 15, close 100/70.
  - nn 10/20, 3.84σ², 0.01, 0.9998, 1.5·s, fuse 3, BA 5+10 and 20.
  - Culling 0.25/3/2/3, keyframe culling 0.9/3.
  - `bf` 40 matches TUM1, TUM2 and TUM3.yaml. EuRoC.yaml has `ThDepth` 35, fps 20, and 1200/1000 features, which the note's runs use.
- **L6:** The keyframe ATE comes from the final map's keyframes, Sim(3) for monocular and SE(3) otherwise, as `KeyFrameTrajectory.txt`.
- **L8:** The run settings are confirmed in `run.json`.
- **L10:** 2000 seeded steep-plane scenes run in every backend; the least condition is 4.5e-4 against the 1e-4 threshold.
- **L3:** Run 1's three points are fixed: coverage 96%, ORB-SLAM3's ten runs, and the fr1/desk row.
- **A1/A3, from the algorithm side:** the gap room forces the PnP recovery in every mode, and the golden record holds the decisions.
- **State of the art:** nothing new since Run 1's L12.

**Scripts saved** (all in a private scripts folder, outside the repo):
- `edge_score_runs.py`
- `edge_start_sweep.sh`
- `edge_sparse_depth_start_probe.py`
- `edge_mono_init_rules_probe.py`
- `edge_mono_segment_split.py`
- `pnp_weight_scale_probe.py`

Sources:
- [ORB_SLAM2 source](https://github.com/raulmur/ORB_SLAM2): `src/Tracking.cc`, `LocalMapping.cc`, `Optimizer.cc`, `Initializer.cc`, `ORBmatcher.cc`, `Examples/{Stereo,Monocular}/EuRoC.yaml`, `Examples/RGB-D/TUM{1,2,3}.yaml`
- [ORB_SLAM3 source](https://github.com/UZ-SLAMLab/ORB_SLAM3): `src/Tracking.cc`
- [Campos et al., ORB-SLAM3, T-RO 2021, arXiv:2007.11898](https://arxiv.org/abs/2007.11898): Table II caption, §VII-A
- [Mur-Artal and Tardós, ORB-SLAM2, T-RO 2017, arXiv:1610.06475](https://arxiv.org/abs/1610.06475)
