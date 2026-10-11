# Review panel: `feature/run-viewer`

<!-- Written by /slam-it-all-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-10

| | |
|---|---|
| **Branch** | `feature/run-viewer` |
| **Range** | `de57cf4...b93d66d` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slam-it-all-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; pytest 3260 passed, node --test 25 passed), on the tree committed as `b93d66d`; budget tests (`pytest -m budget`) 13 passed |

Findings that more than one reviewer raised share a row, with all their IDs. On A11 / L5 the two
reviewers differ on how faer reads the cache sizes; the crates' source settles it: on Linux both
`gemm-common` 0.19 and `private-gemm-x86` 0.1.20 read sysfs over every CPU, once per process,
with cpuid only as a fallback.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L1 / H3 | must-fix | algorithm, hygiene | `src/slam_it_all/orchestration/bundles.py:220, 295-326` | a run the ATE alignment cannot place (fewer than 3 truth matches, or collinear centres) fails the whole bake; any failure after the first write leaves a half-written folder the bake then refuses | fix | fixed (038aaf3): an unalignable run stays in its own frame, unscored; the bundle is written beside its folder and moved onto it whole; tested with a never-initialised run and a failed bake -- decided by evan-wes, 2026-10-10 |
| A1 / L6 | should-fix | architecture, algorithm | `cpp/include/slam_it_all/foundations/linalg.hpp:29`; `slam_it_all_vo.cpp:66`; `module.cpp:17` | repeatable C++ rounding depends on each entry point calling `fix_product_blocking`; the native tests and the benchmarks do not | fix | fixed (07a505f): `EIGEN_NO_CPUID` PUBLIC on every rung, `fix_product_blocking` removed, a native test holds it; C++ runs byte-identical on both core types; the AVX caution recorded (2174b0e) -- decided by evan-wes, 2026-10-10 |
| A2 | should-fix | architecture | ladder doc :78-88, :353, :651; `orchestration/__init__.py`; `eval/__init__.py:9` | the ladder and the package docstrings do not place JavaScript, `viewer/`, the bake or JPEG encoding | fix | fixed (2174b0e) -- decided by evan-wes, 2026-10-10 |
| A3 | should-fix | architecture | `pyproject.toml` | no import-linter contract keeps `vo` below `bundles` | fix | fixed (038aaf3) -- decided by evan-wes, 2026-10-10 |
| A4 | should-fix | architecture | `viewer/tsconfig.json` | the page's logic/view split is unchecked: the logic modules type-check with the DOM | fix | fixed (adbced9): `tsconfig.logic.json`, the logic modules without the DOM -- decided by evan-wes, 2026-10-10 |
| H4 / A5 | should-fix | hygiene, architecture | `bundles.py:124, 285-326, 322` | `bake` mixes phases, restates eval's ground-truth reader (as `run_vo.py:96` does) and re-derives the alignment's kind by a boolean index | fix | fixed (038aaf3): `eval.runs.ground_truth`, `AbsoluteError.similarity`, `bake` in helpers -- decided by evan-wes, 2026-10-10 |
| H1 | should-fix | hygiene | `viewer/src/main.js:90-279` | `main()` is one undocumented closure; the playback logic cannot be tested | fix | fixed (adbced9): `playback.js`, tested -- decided by evan-wes, 2026-10-10 |
| H2 | should-fix | hygiene | `viewer/src/main.js:157-173` | choosing a backend races: a slower earlier load overwrites a later choice | fix | fixed (adbced9) -- decided by evan-wes, 2026-10-10 |
| H5 | should-fix | hygiene | `scripts/check.sh`; `viewer/src/*.js` | no JavaScript formatter; 43 lines run past 100 columns | fix | fixed (adbced9): prettier 3.9.10, pinned, 100 columns -- decided by evan-wes, 2026-10-10 |
| L2 | should-fix | algorithm | `edge/pipeline.py:87-88`; `eval/runs.py:77-78`; `viewer/src/frames.js:10`; C++ and Rust `Fate` | "unmatched" is documented as "no stage matched it"; it means "not matched by the frame's last tracking stage" | fix (the words) | fixed (038aaf3, adbced9, 2174b0e): the words, option (a) -- decided by evan-wes, 2026-10-10 |
| L3 | should-fix | algorithm | `viewer/src/image_view.js:110`; `viewer/src/geometry.js:22-25` | keypoints are drawn half a pixel up and left of the pixel-centre convention; the frustum's corners likewise | fix | fixed (adbced9): `images.onCanvas`, tested -- decided by evan-wes, 2026-10-10 |
| L4 | should-fix | algorithm | `tests/orchestration/test_bundles.py:83` | no test checks that the bake moves points and keyframes with the trajectory | fix | fixed (038aaf3): the projection check under SE(3) and Sim(3) -- decided by evan-wes, 2026-10-10 |
| A6 | note | architecture | `edge/pipeline.py:308, 372, 401`; `pipeline.hpp:309`; Rust's | the edge core keeps every removed point and keyframe until the run ends (up to +26 MiB); `FrameResult` could carry the removals for the recorder instead | defer (ladder backlog) | deferred (ladder, "Deferred from PR 10's review panel") -- decided by evan-wes, 2026-10-10 |
| H6 / A7 | note | hygiene, architecture | `viewer/src/frames.js:6`; `compare.js:10`; `main.js:19` | the page's copies of `FATES` and `STEPS` are not pinned across the Python/JS boundary; a backend the page does not know is dropped silently | fix | fixed (038aaf3, adbced9): `bundle.json` carries the fates and steps; the page refuses others, and a backend it does not know -- decided by evan-wes, 2026-10-10 |
| A8 | note | architecture | `viewer/index.html`; `viewer/package.json` | three.js's version is pinned in two places | fix | fixed (adbced9) -- decided by evan-wes, 2026-10-10 |
| A11 / L5 | note | architecture, algorithm | `linalg.hpp:9`; `docs/notes/viewer.md:67`; ladder :288; commit `cb88de2` | "Rust's products block by fixed sizes" is wrong for faer: read once per machine; inert below an inner dimension of 512 | fix (the words) | fixed (2174b0e) -- decided by evan-wes, 2026-10-10 |
| L8 | note | algorithm | `edge/adjust.py:245-246`; `viewer/src/solver.js:63` | the second local-BA solve's initial cost is over the first's inliers, so two solves read as more progress than happened | fix (the hint) | fixed (adbced9) -- decided by evan-wes, 2026-10-10 |
| H7 | note | hygiene | `viewer/src/solver_view.js:47, 150` | `Math.max(...array)` throws past about 10^5 values | fix | fixed (adbced9) -- decided by evan-wes, 2026-10-10 |
| H9 | note | hygiene | `viewer/src/scene_view.js:81`; `solver_view.js:101` | `SceneView` and `SolverView` have no class-level JSDoc | fix | fixed (adbced9) -- decided by evan-wes, 2026-10-10 |
| A9 | note | architecture | `run.json`; `bundle.json` | the run and bundle formats are the edge rung's and do not name the pipeline | defer (ladder backlog) | deferred (ladder, "Deferred from PR 10's review panel") -- decided by evan-wes, 2026-10-10 |
| A10 | note | architecture | `edge.adjust.Solve` and its ports | the BA solve record is per rung; it moves to `foundations.nls` when a second rung records solves | defer (ladder backlog) | deferred (ladder, "Deferred from PR 10's review panel") -- decided by evan-wes, 2026-10-10 |
| L7 | note | algorithm | `edge/tracking.py:788, 815` | pre-existing: the local-map search re-matches motion-model outliers, which ORB-SLAM2 skips | defer (ladder backlog) | deferred (ladder, "Deferred from PR 10's review panel") -- decided by evan-wes, 2026-10-10 |
| H8 | note | hygiene | `bundles.py:128` | `_truth` reads any `FileNotFoundError` as "no ground truth" | waive | waived: the bake opens the sequence (`vo.open_source`) before reading its truth, so a missing folder or frame list fails first; what is left missing is the ground-truth file -- decided by evan-wes, 2026-10-10 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** PR 10 on `feature/run-viewer`, `de57cf4...b93d66d` (154 files). I read in full: `src/slam_it_all/orchestration/bundles.py`, `scripts/view_bundle.py`, all of `viewer/src/*.js` (main, bundle, npy, frames, geometry, compare, solver, solver_view, scene_view) and the viewer test helpers. I also read the `eval/runs.py` v3 diff, the `CLAUDE.md`, `check.sh`, `.jscpd.json` and `run_vo.py` diffs, and the test names for the bake and `view_bundle`. For the C++ and Rust per-frame summary changes I only grepped the diff for debug output, TODOs, `unwrap`/`expect` and private-rig references (nothing real); I did not read those hunks line by line.

**Summary:** This is production quality. Public boundaries are validated with messages that name the bad value, documentation states shapes, dtypes, frames and units, and there are no TODOs, debug prints, hard-coded paths, shims or private-rig specifics. The golden bundle is held by both the Python bake tests and the JS tests, and the JS stage hints are pinned to the recorded keys. I found no must-fix. The should-fixes are one monolithic JS function, a race in backend switching, an all-or-nothing bake that can leave a half-written folder, and no JS formatter in layer 1.

### Findings

#### [should-fix] `main()` in the page is a ~230-line closure with no doc and many responsibilities
- **Where:** `viewer/src/main.js:90` to `viewer/src/main.js:279`
- **Issue:** One undocumented `async function main()` does bundle loading, DOM wiring, backend selection, the agreement badge, playback clock, keyboard handling, follow/fit and error display. Mutable state (`run`, `frame`, `playing`, `clock`, `last`) is shared by nested closures. The rest of the viewer is cleanly split into pure logic modules plus thin views; this is the one place that is not.
- **Why it matters:** The playback clock and the seek and step logic are testable-in-principle logic that currently cannot be tested under `node --test`. The file is hard to read and change safely.
- **Evidence:** `async function main() {` has no JSDoc, unlike every other function in `viewer/src`. `tick`/`setPlaying`/`seek` are nested closures over `frame`/`clock`.
- **Suggested fix:** Pull playback (a small `Player` with `frame`, `clock`, `seek`, `tick` and an `onFrame` callback) and the agreement badge into their own modules. Then `main()` only wires. Give `main()` a JSDoc line.

#### [should-fix] Choosing a backend can race: a slower earlier load overwrites a later choice
- **Where:** `viewer/src/main.js:153` to `viewer/src/main.js:170` (`choose`)
- **Issue:** `run = await held;` is followed by the UI updates, with no check that this is still the latest choice. Clicking `cpp` and then `rust` while `cpp`'s arrays are still loading can end with `run`, the pressed button and the scene disagreeing.
- **Why it matters:** The page can show a different backend from the one the buttons say. It is rare, but the backends' files are tens of MB.
- **Evidence:** `choose` has no token or generation check between `await held` and `scene.setRun(run)`.
- **Suggested fix:** Keep a `chosen` name. After the `await`, `if (chosen !== name) return;`.

#### [should-fix] `bake()` writes straight into the output folder, so a failure leaves a half-written bundle that blocks the retry
- **Where:** `src/slam_it_all/orchestration/bundles.py:285` to `src/slam_it_all/orchestration/bundles.py:326`
- **Issue:** The input checks (one sequence, one per backend, contiguous range) all run before the first write. After that, images, truth and each backend are written in place, and `bundle.json` is written last. If a write or `_write_backend` raises (`runs.evaluate` on bad truth, disk full, or `folder.mkdir(parents=True)` on a name collision), `out` is left non-empty with no `bundle.json`. A rerun then fails with "expected a new or empty folder". `runs.write_run` in the same package already replaces files whole for this reason.
- **Why it matters:** A multi-minute bake of thousands of JPEGs fails late, and the user must delete by hand.
- **Evidence:** `if target.exists() and any(target.iterdir()): raise ...` at the top, with no cleanup on a later failure.
- **Suggested fix:** Bake into a temporary sibling folder and `rename` it onto `out` when `bundle.json` is done. Or remove what was written on failure, only if `bake` created `out`.

#### [should-fix] `bake()` is long and mixes phases; one line indexes a tuple by a boolean
- **Where:** `src/slam_it_all/orchestration/bundles.py:285` to `src/slam_it_all/orchestration/bundles.py:341` (`bake`); `src/slam_it_all/orchestration/bundles.py:332` (`_ALIGNMENTS[first.mode == "monocular"]`)
- **Issue:** `bake` validates, opens the source, computes the frame union, checks ranges, writes images, times, truth and the camera/info dict, and writes each backend. The truth-centres block and the `info` dict are natural helpers, like the existing `_backend_info` and `_write_images`. `_ALIGNMENTS[...] [bool]` is clever indexing where a conditional reads directly: `"sim3" if first.mode == "monocular" else "se3"`.
- **Why it matters:** Readability only; the logic is correct.
- **Evidence:** `"truth": None if truth is None else {"alignment": _ALIGNMENTS[first.mode == "monocular"]},`
- **Suggested fix:** Extract `_write_truth(...)` and `_describe(...)`, and use the explicit conditional.

#### [should-fix] No formatter or linter for the JavaScript; long lines already exist
- **Where:** `scripts/check.sh` (JS block); for example `viewer/src/scene_view.js:88`, `viewer/src/scene_view.js:176`, `viewer/src/solver_view.js:169`, `viewer/src/main.js:155`, `viewer/src/bundle.js:195`
- **Issue:** Layer 1 runs `node --check`, `node --test` and `tsc --checkJs`, but nothing enforces style. Python, C++ and Rust all have a formatter, and JS code already runs past 100 columns (some lines at 140 to 190 characters, e.g. the `camera.position.copy(...)` chain in `fit()`). That breaks the convention of the other three languages.
- **Why it matters:** The style will drift, and diffs will carry formatting noise.
- **Evidence:** `viewer/src/scene_view.js` `fit()`: `this.camera.position.copy(centre).add(this.camera.up.clone().multiplyScalar(reach * 0.6)).add(new THREE.Vector3(reach * 0.6, reach * 0.3, 0));`
- **Suggested fix:** Add a pinned prettier (with `printWidth` 100) to `viewer/package.json`, with `--check` in `check.sh`. It is free and offline once locked, which fits the no-paid-dependencies rule.

#### [note] JS mirrors of Python constants are only partly pinned by tests
- **Where:** `viewer/src/compare.js:9` (`STEPS`), `viewer/src/frames.js:6` (`FATES`), `viewer/src/main.js:19` (`BACKENDS`)
- **Issue:** `STAGES` hints are checked against the golden records' keys (`viewer/tests/solver.test.js:34`), and `FATES[UNMATCHED]` is spot-checked. `STEPS` and the full `FATES` order are not compared with `slam_it_all.eval.runs.STEPS` or `FATES`. `BACKENDS` silently drops any backend a bundle holds that is not named there. Because the parallel Python/C++/Rust code is intentional, this is not duplication, but the JS copies can drift unnoticed.
- **Suggested fix:** Have the golden-bundle test assert `STEPS` against the record keys, as it does for stages. Have `main` report a backend in the bundle that it does not know.

#### [note] `Plot.set` spreads a large array into `Math.max`
- **Where:** `viewer/src/solver_view.js:44` (`Math.max(...finite, 1e-9)`), and the same in `setRun` (`Math.max(...own, ...)`)
- **Issue:** Spreading is limited by engine argument and stack limits (roughly 10^5 values). MH_01 is 3,682 frames, so it is fine today, but a long run will throw `RangeError`.
- **Suggested fix:** Use a `reduce` loop.

#### [note] `_truth` treats any `FileNotFoundError` as "no ground truth"
- **Where:** `src/slam_it_all/orchestration/bundles.py:128`
- **Issue:** This is right for the sequence-without-truth case. `bake` has already checked that the folder name matches and opened the source, so a wrong path would have failed earlier. A reader that raises `FileNotFoundError` for a different missing file would still be read as "no truth".
- **Suggested fix:** None required. Narrow it if the readers can name the file.

#### [note] `SceneView` and `SolverView` have constructor docs but no class-level description
- **Where:** `viewer/src/scene_view.js:81`, `viewer/src/solver_view.js:101`
- **Issue:** `Plot` and the module headers explain intent; the two exported classes have no JSDoc summary line.

### Checked and fine
- `bundles.py`: the module docstring states every file, shape, dtype, frame convention, alignment (Sim(3) for monocular, SE(3) otherwise) and units. Inputs are validated with named messages (`_read`, `_frames_of`, range check). `_aligned` divides out the scale with `cbrt(det)`, which also handles reflections and SE(3). The `_lives` renumbering is correct. There is no swallowed exception, and the magic numbers `CHUNK` and `QUALITY` are named and justified.
- `bundles.py` has no duplicated logic of its own. It reuses `runs.evaluate`, `budgets.judge`, `metrics.associate` and `images.encode_jpeg` rather than re-deriving them.
- `scripts/view_bundle.py`: it binds to 127.0.0.1 only, and `translate_path` goes through the stdlib normalisation, so the page and bundle folders cannot be escaped. The `no-cache` reason is stated, `KeyboardInterrupt` is deliberate and cleaned up, and `tests/test_view_bundle.py` covers the routing.
- `eval/runs.py` v3: `_check_lives` and `_check_keypoints` validate every constraint with clear messages. The version bump is a clean break (no v2 reader or shim), and `FeatureWriter` rewrites fixed-length headers with the reason documented. `Run.final_points` and `final_keyframes` are documented, and `run_vo.py` uses them consistently.
- `viewer/src/npy.js` and `bundle.js`: header parsing is strict (magic, version, Fortran order, dtype, shape) with messages that name the file. Each `.npy` is viewed in place, which is safe because numpy's 64-byte header alignment makes the offset a multiple of 4. Shape checks cover every array. The `Fetch` injection lets the tests run on the golden bundle.
- `frames.js`, `geometry.js`, `solver.js`, `compare.js`: pure logic, no DOM, the binary searches are right, and JSDoc states shapes and units. The `alive()` (JS) and shader (`removed > frame`) semantics agree, and `NEVER = 2**30` is explained.
- Duplication: jscpd is extended to JS and the doc-comment ignore pattern is added, with no JS duplication of note. No debug output, `TODO`/`FIXME`, hard-coded paths, `unwrap`/`expect` on fallible input, or commented-out code in the diff outside tests.
- Project rules: nothing specific to the private rig recording appears (the only phone reference in `docs/notes/viewer.md` is a generic pointer to PR 11). No compatibility shims were added, and the version 2 to 3 change is a clean break.
- `CLAUDE.md` and `check.sh` changes are accurate and consistent with each other (the JS row in the language table, the layer-1 description, the pathspec).
- Tests: `tests/orchestration/test_bundles.py` covers the golden layout, alignment, renumbering, no-truth, header contents and refusals. The JS tests cover npy, bundle loading, frames, compare, geometry and solver on the golden bundle.

### Architecture review

## Architecture review

**Scope:** PR 10 (the run viewer), `de57cf4...b93d66d` on `feature/run-viewer`: 154 files across 7 commits. I read the Python edge, eval and orchestration changes (`pipeline`, `map`, `adjust`, `tracking`, `eval/runs`, `eval/images`, `orchestration/vo`, `orchestration/bundles`); the C++ and Rust edge, eval and orchestration headers and sources plus their binding crossings; the CMake and `check.sh` changes; all of `viewer/` and `scripts/view_bundle.py`; and the golden fixtures and their tests. I read them against the ladder doc, the run-viewer design doc (§1–4, §8) and `docs/notes/viewer.md`. The repository was not modified. My scratch files are only in the session scratchpad.

**Summary:** Architecturally sound, with no must-fix.
- **Dependency direction:** correct in all three languages, and checked below. The C++/Rust `eval` writers stay independent of the edge rung: they take plain spans and slices, and orchestration converts the edge types.
- **Language split:** held. The new native code has a Python reference and exact parity on the room. Bindings stay thin. Rust stays within its subset.
- **Bake and page placement:** the bake and the page sit where §8 decided.
- **Findings:** some rules are kept by convention and nothing checks them: Eigen's product blocking, the orchestration order, and the viewer's logic/view split. The ladder doc and the package docstrings have not caught up with a fourth language and with the bake. The bake restates two rules that `eval` owns.

### Findings

#### [should-fix] Repeatable C++ runs depend on each entry point remembering a call; the tests and the benchmark do not make it
- **Where:** `cpp/include/slam_it_all/foundations/linalg.hpp:29`. Called only at `cpp/apps/orchestration/slam_it_all_vo.cpp:66` and `cpp/bindings/module.cpp:17`. The rule is stated at `CLAUDE.md` (Conventions) and in the ladder doc around line 282.
- **Issue:** "Every C++ entry point calls `fix_product_blocking` first" is a rule that nothing checks. `git grep fix_product_blocking` finds two callers.
  - Every native test `main` (`cpp/tests/**`) skips it.
  - The benchmark `main` (`cpp/bench/foundations/bench_foundations.cpp:210`) skips it.
  - So will any future app, unless someone remembers.
- **Why it matters:** the guarantee in the ladder's Conventions ("a run rounds the same on every core") holds only by memory. The project already handles the same kind of invariant structurally: `-ffp-contract=off` is a PUBLIC compile option on every rung library, so tests and bindings that inline core arithmetic compile the same way.
- **Evidence:**
  - `CMakeLists.txt:33` sets `target_compile_options(slam_it_all_${rung} PUBLIC -ffp-contract=off)`.
  - Eigen 5.0 in the env, `Eigen/src/Core/util/Memory.h:980` has `#if !defined(EIGEN_NO_CPUID)`. Without CPUID, `queryCacheSizes` sets `l1 = l2 = l3 = -1` (line 1309).
  - `GeneralBlockPanelKernel.h:74-77`: `manage_caching_sizes_helper` then returns `defaultL1/L2/L3CacheSize`, which are `32*1024`, `256*1024` and `2*1024*1024` on x86. These are exactly `kL1CacheBytes`, `kL2CacheBytes` and `kL3CacheBytes`.
  - `EIGEN_CPUID` is used nowhere else in Eigen, so the define has no other effect.
- **Suggested fix:** in `slam_it_all_add_rung`, beside `-ffp-contract=off`, add `target_compile_definitions(slam_it_all_${rung} PUBLIC EIGEN_NO_CPUID)`.
  - Every translation unit that links a rung then uses Eigen's defaults: program, extension, tests, benchmarks and future apps.
  - Results stay byte-identical to today's fix on x86.
  - `fix_product_blocking`, its two calls and the per-entry-point rule can then go. The doc sentence becomes "fixed at compile time".
  - The `test_linalg.cpp` check can be rewritten to assert the defaults are in effect.

#### [should-fix] The ladder doc and the package docstrings do not yet place the fourth language and the bake
- **Where:**
  - `docs/design/2026-10-02-slam-it-all-ladder.md`:
    - lines 78-84: the Languages table lists Python, C++ and Rust only;
    - line 651: "deterministic checks in all three languages";
    - line 88: Pillow is in `eval.images` "which reads image files", but it now also encodes JPEG;
    - line 353: the orchestration row says "End-to-end pipelines".
  - `src/slam_it_all/orchestration/__init__.py`: lists only `vo`, and is unchanged in this PR.
  - `src/slam_it_all/eval/__init__.py:9`: describes `images` as "reading image files".
- **Issue:** `CLAUDE.md` gained a JavaScript row (`viewer/`, three.js from a CDN, `tsc --checkJs`) and layer 1 runs JS checks. The design doc, which is the stated authority on the language split, did not change. The bake is a new module in a package whose docstring and ladder row describe only pipelines.
- **Why it matters:**
  - `CLAUDE.md` and the design doc now disagree about which languages the project has.
  - PR 11 adds a fifth top-level home (`server/`, its own package) and a worker in orchestration. Without these anchors, both would land with no place in the ladder's statement of structure.
- **Evidence:** `grep -n -i 'javascript\|viewer/\|three.js' docs/design/2026-10-02-slam-it-all-ladder.md` finds nothing. Neither `__init__.py` file is in the diff stat.
- **Suggested fix:** change the docs; the code is right.
  - Add a JavaScript row to the Languages table: the page only, its dev tools pinned in `viewer/package-lock.json`, three.js from a CDN.
  - Add `viewer/` under Package layout.
  - Change "all three" to "every" in Review process.
  - Widen Pillow's line to "reads image files and encodes the bake's JPEGs".
  - Widen the orchestration row to "End-to-end pipelines, and the bake of their runs for the viewer".
  - List `bundles` in `orchestration/__init__.py`, and JPEG encoding in `eval/__init__.py`.

#### [should-fix] Orchestration now has an internal order, and no contract holds it
- **Where:** `pyproject.toml` import-linter contracts. Foundations, `ba`, edge and eval each have an internal-order contract; orchestration has none.
- **Issue:** `bundles` builds on `vo` (`bundles.py` imports `orchestration.vo` for `open_source`), and `vo` must never depend on the bake. `vo` is the program the budget runs time, and the ladder keeps the pipelines independent of the viewer. With one module this needed no contract. With two now, and PR 11's `worker` on top of both, it does.
- **Why it matters:** if the run program imported the bake or the worker, the budget's measured program would grow by the viewer's code paths, and nothing would fail.
- **Evidence:** a scratch import-linter config, outside the repo and run at HEAD, holds the proposed contract with `layers = (worker) / bundles / vo` on `slam_it_all.orchestration`. Output: `Orchestration's internal order (proposed) KEPT ... Contracts: 1 kept, 0 broken.`
- **Suggested fix:** add an "Orchestration's internal order" contract with `"(worker)", "bundles", "vo"`, the optional-layer form eval uses. `check_layers.py` needs nothing, since the bake is Python-only.

#### [should-fix] The viewer's logic/view split is stated as a rule, but nothing checks it
- **Where:** `viewer/tsconfig.json` (one config for all of `src/**`, with `"lib": ["es2022", "dom", "dom.iterable"]`). The rule is stated in the `CLAUDE.md` JavaScript row ("Logic modules never touch the page, so Node tests them") and in design doc §4.
- **Issue:** the logic modules are `bundle`, `npy`, `frames`, `compare`, `solver`, `images`, `geometry` and `format`. They are type-checked with the DOM library. A `document` or `getComputedStyle` reference inside a function body would pass `tsc`, and would also pass `node --test` unless that exact path ran. The split is the JS counterpart of the layer contracts that import-linter and `check_layers.py` enforce in the other three languages.
- **Why it matters:** if the split erodes, Node can no longer test the logic, and the golden-bundle tests stop covering what the page computes.
- **Evidence:** I ran `tsc` on the eight logic modules with a scratch tsconfig using `"lib": ["es2022"]` (no DOM). The only errors were `bundle.js(122,25)` and `npy.js(36,20)`: `Cannot find name 'TextDecoder'`. So the modules are DOM-free today apart from `TextDecoder`, which Node also provides.
- **Suggested fix:** add `viewer/tsconfig.logic.json` listing the logic modules with `"lib": ["es2022"]`, plus a one-line ambient declaration of `TextDecoder`, and a second `tsc -p` line in `check.sh`. Optionally, a node test could assert that no logic module imports `three`.

#### [should-fix] The bake restates two rules that eval owns
- **Where:**
  - `src/slam_it_all/orchestration/bundles.py:124`: `tum.read_sequence(sequence) if dataset == "tum" else euroc.read_sequence(sequence)`. This is identical to `scripts/run_vo.py:96`.
  - `bundles.py:82` and `bundles.py:322`: `_ALIGNMENTS[first.mode == "monocular"]`. This re-derives the rule that `eval/runs.py:798` applies (`with_scale=run.mode == "monocular"`).
- **Issue:** two copies of "how to read a sequence's ground truth". Also, the bundle's `truth.alignment` label is computed beside the alignment rather than read from it.
- **Why it matters:** if `runs.evaluate`'s choice ever changes (for example, Sim(3) for a phone run whose depth is unscaled), `bundle.json` will name an alignment the runs were not moved by. The page shows that label next to the ATE. PR 11's worker will call the bake on new kinds of input, which is where such a rule is likely to move.
- **Evidence:** the quoted lines. `metrics.AbsoluteError` already carries `alignment` and `scale` (`eval/metrics.py:50-62`), but not which kind was fitted.
- **Suggested fix:**
  - Add `eval.runs.ground_truth(dataset, sequence) -> (times, T_world_cam) | None`, used by both `bundles` and `run_vo.py`.
  - Have `Evaluation` (or `AbsoluteError`) record whether scale was estimated, so the bake reads the label rather than re-deriving it.

#### [note] A run's whole history now lives in the edge core
- **Where:**
  - Python: `src/slam_it_all/edge/pipeline.py:308` (`_gone_points`), `:401` (`_date`), `:372` (`lifetimes`).
  - C++: `cpp/include/slam_it_all/edge/pipeline.hpp:309` (`gone_points_` and siblings, indexed by every id ever made), `cpp/src/edge/pipeline.cpp` (`Pipeline::date`, `Pipeline::lifetimes`).
  - Rust: the same.
- **What happens:** the map's `gone_*` archive is cleared every frame, but the pipeline keeps every removed point's position and every removed keyframe's pose until the run ends. The viewer note measures this at up to +26 MiB peak, for example MH_01 stereo going from 133 to 159 MiB.
- **How others do it:** ORB-SLAM2 also never frees culled `MapPoint`s; they are flagged bad and leaked. So the cost is not unusual, and the design chose to always record lives.
- **Why it may matter later:** this is recording state inside the real-time core, while the feature records were deliberately streamed by the recorder. It will matter for the deferred live camera, or any long-running use. An alternative that keeps the core's state bounded: `FrameResult` carries the frame's removals (ids, last positions or poses, how removed) and its count of new ids, and orchestration's recorder accumulates or streams them. That is the same path poses and diagnostics already take. Backlog, not this PR.

#### [note] The page keeps its own copies of the run format's code tables
- **Where:** `viewer/src/frames.js:6` (`FATES`) and `viewer/src/compare.js:10` (`STEPS`), copies of `eval.runs.FATES` and `eval.runs.STEPS`.
- **What exists:** the frames test catches a reordering of made/tracked indirectly, through record counts. The names themselves are pinned by nothing across the Python/JS boundary.
- **Option:** the page "reads only a bundle", so `bundle.json` could carry `fates`, `removals` and `steps`. The page would then read the tables, and the golden-bundle layout test would hold them.

#### [note] three.js's version is pinned in two places
- **Where:** `viewer/index.html` (import map, `three@0.186.0`) and `viewer/package.json` (`@types/three` 0.186.0).
- **Risk:** if the CDN version is bumped alone, `tsc --checkJs` checks the 3D module against stale types.
- **Option:** a small node test that reads both files and compares the versions.

#### [note] The run and bundle formats are shaped by the edge rung, and do not say which pipeline made a run
- **What is edge-specific:** `run.json` and `bundle.json` name the backend but not the pipeline or rung. Several things in the "rung-agnostic" run format and the page are the edge rung's own:
  - `eval.runs.FATES` and `REMOVALS`;
  - `record_features`;
  - the page's `solver.js` `STAGES`.
- **When it matters:** that is fine for milestone 2. The ladder's "rungs side by side" will need a pipeline field and rung × backend keys in `backends`. Under the no-shims rule, that is a version bump when it comes.

#### [note] The BA solve record is per rung
- **Where:** `edge.adjust.Solve` and its C++ and Rust counterparts, each a JSON-friendly projection of rung 0's `nls.Summary`.
- **When it matters:** when a second rung records BA solves (the classic rung's global BA), move the projection to `foundations.nls`, as the ladder does for shared components.

#### [note] The doc's reason Rust "already repeats" is imprecise
- **Where:** ladder doc line 288 ("Rust's products block by fixed sizes already") and `linalg.hpp:9`.
- **What actually happens:** faer's products go through `gemm-common` 0.19 (in `rust/Cargo.lock`). Its `cache.rs:73-215` reads cache sizes once per process from sysfs, iterating over all of `/sys/devices/system/cpu/cpu*`.
- **Consequence:** Rust's blocking is the same on every core of one machine, which is why runs repeat as measured. It is not fixed across machines: where faer's products are large enough to block, they may round differently elsewhere, for example on a Railway worker.
- **Suggested fix:** reword to "block by sizes read once for the whole machine". This is also a pointer for the algorithm reviewer.

### Checked and fine
- **Python layering:**
  - `bundles` imports only eval (`budgets`, `euroc`, `images`, `metrics`, `runs`, `tum`), `foundations.errors` and `orchestration.vo`;
  - `vo` does not import `bundles`;
  - no algorithm rung imports eval or orchestration;
  - `eval.runs` stays independent of the edge rung (it refers to the edge enums in docs only, and `tests/eval/test_runs.py:102-109` pins its codes to them).
- **Pillow:** stays in `eval.images` alone; the bake gets JPEGs through `encode_jpeg`.
- **C++ and Rust layering:**
  - `eval`'s `FeatureWriter` and the v3 writer take spans and slices, with no edge types;
  - orchestration converts `edge::Fates`;
  - CMake `DEPENDS` and the crate dependencies are unchanged (no `Cargo.toml` in the diff);
  - no new runtime dependency in any core.
- **Language split:**
  - the new C++/Rust code (stage summaries, lifetimes, `Removal`, `Fate`, the streamed writer) has a Python reference;
  - `room_runs.json` gains lives, fate counts and summaries;
  - `test_the_backends_agree` compares lives and feature records exactly, and summaries with costs within BA tolerance;
  - the eval run fixtures are extended;
  - the `Implementation` Protocols gained `codes()`;
  - Rust stays within rung 0, edge, eval and its programs;
  - the bake is Python-only, as decided.
- **Bindings:** `edge_crossing` (C++/Rust) and the tracking and initialization bindings only turn summaries into dicts, with enums as codes through `codes()`. They contain no algorithm logic.
- **Error contract:** the native writers report failure as `std::expected` / `Result` with `InvalidInput`, including I/O, per eval's established rule. Python raises `InvalidInputError`.
- **Clean break:** `runs.VERSION` is 3 and `_read_info` refuses v2. The bundle format is versioned (1), and the page refuses other versions. No shims.
- **Stage summaries:** each stage returns its summary with its result, consistent with the ladder's solver-summary contract, using rung 0's termination names.
- **`record_features`:** travels in `Request`, not `Options`, so it changes no decision and no setting. It is streamed, so memory stays flat, and the measured budget runs are unchanged with it on.
- **Import-time side effect:** `fix_product_blocking` at `NB_MODULE` touches only the `_cpp` module's own Eigen statics (NB_STATIC, header-only Eigen). Nothing leaks to other extensions.
- **Viewer boundaries:**
  - the page reads only a bundle, and no Python package depends on `viewer/`;
  - its npm dependencies are development-only, pinned by the lockfile, with `node_modules` ignored;
  - the golden bundle holds the format from both sides (Python checks the layout, JS reads the values).
- **Compute budgets:** nothing GPU- or torch-related enters the CPU rungs. three.js lives only in the page. Computing fates on every frame costs noise-level time, as measured.
- **`scripts/view_bundle.py`:** imports nothing from `slam_it_all`, binds `127.0.0.1`, and has a test (following the `test_run_vo.py` precedent).
- **Privacy:** the diff has nothing specific to the owner's rig recording. The golden bundle comes from the synthetic room.

### Algorithm review

## Algorithm review

**Scope:** `de57cf4...b93d66d` (PR 10, the run viewer, 7 commits). I read in full:
- the Eigen blocking fix (`cpp/include/slam_it_all/foundations/linalg.hpp`, both entry points, the Eigen 3.5 headers in the env);
- the stage summaries, fates, lifetimes and removal dating in Python, C++ and Rust (`edge/{pipeline,tracking,mapping,map,adjust,initialization}`);
- the run format v3 readers and writers (`eval/runs`, `cpp/src/eval/runs.cpp`);
- the bake (`src/slam_it_all/orchestration/bundles.py`) and its tests;
- the page's logic and drawing (`viewer/src/{frames,geometry,compare,solver,npy,bundle,scene_view,image_view,main,format}.js`) and its tests.

I checked each against ORB-SLAM2's `Tracking.cc`/`LocalMapping.cc`, Umeyama (1991) and Sturm et al. (2012). Probes ran from the scratchpad. One side effect: importing `slam_it_all` rebuilt the git-ignored editable extension in `build/slam-it-all` from HEAD's unchanged sources. The working tree is clean, HEAD is unmoved, and nothing tracked was modified. The probes are kept for checking fixes, in a private scripts folder:
- `viewer_fate_probe.py`
- `viewer_bake_unalignable_probe.py`
- `edge_motion_outlier_rematch_probe.py`
- `viewer_bake_alignment_invariant_probe.py`

**Summary:** The algorithms are sound:
- The Eigen fix is the right mechanism, correctly placed.
- The fate rule, the lifetimes and the removal dating agree in all three languages and follow ORB-SLAM2's order of local-mapping steps.
- The bake's alignment and frame renumbering are mathematically right; I checked the projection invariant numerically.

One must-fix: a run the ATE alignment cannot place (fewer than 3 truth matches, or collinear centres) fails the whole bake and leaves a half-written bundle. The should-fixes:
- "unmatched" is documented as "no stage matched it", but in this run format it means "the last stage did not match it";
- keypoints are drawn half a pixel off the project's pixel-centre convention;
- no test checks that the bake moves points and keyframes with the trajectory.

### Findings

#### [must-fix] A run the ATE alignment cannot place fails the whole bake, and leaves a half-written bundle
- **Where:** `src/slam_it_all/orchestration/bundles.py:220` (`runs.evaluate` unguarded). Also `bundles.py:295-296` (folder, images, `times.npy` and `truth.npy` written before any run is scored) and `:323`. The raisers are `src/slam_it_all/eval/runs.py:794` (`len(ours) < 3`) and `src/slam_it_all/eval/metrics.py:113` (`DegenerateError` for collinear centres, `foundations/alignment.py:111-112`). `bake`'s Raises section (`bundles.py:270-279`) does not list `DegenerateError`.
- **Issue:** With ground truth, every run must have at least 3 frames matched to the truth and non-collinear centres, or `bake` raises. This hits a monocular run that never initialised, or a track lost in its first frames. When it raises, the bundle folder already holds `images/`, `times.npy` and `truth.npy`, with no `bundle.json`. A second bake into the same folder is then refused as "not empty".
- **Why it matters:**
  - The design says "A lost track ends the run, and the viewer shows where and why" (`docs/design/2026-10-09-run-viewer.md:376`). Yet the runs that fail earliest are the ones the bake cannot take whenever the sequence has truth.
  - One backend's unalignable run takes every backend's bundle down with it.
  - PR 11's worker will bake Stray Scanner recordings, which have truth.
- **Evidence:** A monocular room run of 5 frames (the room initialises at frame 10), baked with truth (`viewer_bake_unalignable_probe.py`):
  ```
  tracked frames: 0 of 5
  bake raised: InvalidInputError only 0 frames have a ground-truth pose; need 3
  left behind: ['images', 'images/chunk_0000.bin', 'images/spans.npy', 'times.npy', 'truth.npy']
  re-bake raised: InvalidInputError .../bake_case/bundle: expected a new or empty folder for the bundle
  ```
- **Suggested fix:**
  - When `evaluate` raises `InvalidInputError` for too few matches, or `DegenerateError`, write `score: null` and leave that run in its own frame. Mark it per backend (e.g. `"aligned": false`) so the page can say the run could not be aligned.
  - Score and check every run before writing anything, or bake into a temporary sibling and rename it on success (as `write_run`'s `_replace` does per file).
  - Add a test with a run that never initialises, baked with truth.

#### [should-fix] "Unmatched" is documented as "no stage matched it", but a motion-model or PnP match the last stage did not repeat is recorded as unmatched
- **Where:**
  - `src/slam_it_all/edge/pipeline.py:87-88` (`UNMATCHED`: "no stage matched it");
  - `viewer/src/frames.js:10` (hint: "matched to no point");
  - `src/slam_it_all/eval/runs.py:77-78` ("matched to no point");
  - the same wording in the C++ and Rust `Fate` docs.

  The note's own definition (`docs/notes/viewer.md:55`, "none of these") is accurate.
- **Issue:** REJECTED is taken from the last tracking stage's matches only. On a placed frame that stage is the local map, whose `matched` holds the motion stage's *inliers* plus its own new matches (`tracking.py:810`). A keypoint the motion model matched and its refinement rejected is therefore recorded "unmatched", unless the local-map search happens to re-match it. The same goes for the motion model's matches on a frame PnP then handled. Whether a motion outlier shows as rejected or unmatched thus depends on an unrelated search.
- **Why it matters:** The legend's hover text tells the reader something false about a keypoint the pipeline did match and reject.
- **Evidence:** On the synthetic room (`viewer_fate_probe.py`), keypoints recorded unmatched that an earlier stage on the frame had matched:

  | Mode | Unmatched, though an earlier stage matched them | Earlier-stage matches |
  |---|---|---|
  | RGB-D | 84 | 8,485 |
  | Monocular | 128 | 5,753 |
  | Stereo | 138 | 6,549 |

  Which of these end up rejected rather than unmatched is down to re-matching: 193 of 363 RGB-D motion outliers were re-matched to the same point (see the pre-existing note below).
- **Suggested fix:** Two options:
  - **(a)** Fix the words, in all four places and the C++/Rust docs: "not matched by the frame's last tracking stage, and no point made of it". I recommend this now.
  - **(b)** Widen REJECTED to any tracking stage's match on the frame, the last stage's point taking precedence. This changes the golden records in three languages, so it is the owner's call.

  No reference convention binds here: ORB-SLAM2's `FrameDrawer` draws only matched map points, never rejections.

#### [should-fix] Keypoints are drawn half a pixel up and to the left
- **Where:** `viewer/src/image_view.js:110` (`context.arc(keypoints.xy[i * 2], keypoints.xy[i * 2 + 1], …)`), and `viewer/src/geometry.js:22-25` (frustum corners at `(0,0)` and `(w,h)`).
- **Issue:** The project puts the top-left pixel's centre at `(0, 0)` (ladder design doc, "Pixel coordinates follow OpenCV"). A canvas puts it at `(0.5, 0.5)`. Every keypoint is therefore drawn 0.5 image pixels up and left of where it was found. Likewise, the image's edges are at -0.5 and w-0.5, not 0 and w.
- **Why it matters:** The offset is systematic, and it grows with the panel's CSS scale. The project holds this convention exactly everywhere else ("pixel centres" in CLAUDE.md's conventions).
- **Suggested fix:** Draw at `(x + 0.5, y + 0.5)`, ideally through a small pure helper that Node tests. Put the frustum corners at `(-0.5, -0.5)` to `(w - 0.5, h - 0.5)`.

#### [should-fix] No test checks that the bake moves points and keyframes with the trajectory
- **Where:** `tests/orchestration/test_bundles.py:83` checks only that aligned centres sit on the truth, and that keyframe rotations stay orthonormal.
- **Issue:** Nothing checks `points.npy`, or keyframe translations, against the run. A bake that forgot the scale on points under Sim(3), or applied the inverse similarity, would pass.
- **Evidence:** The code is right; I checked the invariant by hand (`viewer_bake_alignment_invariant_probe.py`). On the room's monocular run (Sim(3), scale 9.70), each baked point's normalised projection into each baked keyframe matches the run's own to `4.99e-08`, over 8 keyframes × 952 points.
- **Suggested fix:** Assert that projection invariant, to float32 tolerance, for both the `se3` and `sim3` cases.

#### [note] Rust's "fixed sizes" holds for nalgebra, not for faer
- **Where:** `cpp/include/slam_it_all/foundations/linalg.hpp:9` ("as Rust's products already do"); commit `cb88de2` ("Rust, whose products block by fixed sizes"); `docs/notes/viewer.md:67`.
- **Issue:**
  - Rust's SVD, eigendecomposition and least squares go through faer (`rust/slam-it-all-foundations/src/linalg.rs`).
  - faer's products use the `gemm` and `private-gemm-x86` crates. Both read the L1, L2 and L3 sizes once, by cpuid on the calling core: `gemm-common-0.19.0/src/cache.rs:475` (OnceCell) and `private-gemm-x86-0.1.20/src/cache.rs:356` (OnceLock). Neither has a setter.
  - `kernel_params` derives the summation block from L1's size and associativity. It is clamped to `.max(512).min(k)`, so only products whose inner dimension exceeds 512 can round differently between the core types.
  - The measured byte-identical Rust runs show no such product arose on these sequences. A DLT or eight-point refit over more than 512 correspondences is where one could.
- **Suggestion:** Reword the claim to match: nalgebra blocks by fixed sizes; faer's blocking is cache-detected but inert below k = 512, as measured. No code change.

#### [note] Nothing makes a future C++ entry point call the blocking fix
- **Where:** `cpp/tests/foundations/test_linalg.cpp`.
- **Issue:** The test checks the setter only. The fix holds today: both entry points call it first (`slam_it_all_vo.cpp:66`, `module.cpp:17`), the libraries are `STATIC` and `_cpp` is `NB_STATIC` (one DSO, one `manage_caching_sizes` static), and there is no OpenMP and no threading. But nothing makes the next `main` call it.
- **Suggestion:** Have `scripts/check_layers.py` require the call in every `int main` and `NB_MODULE`. Alternatively, expose `Eigen::l1CacheSize()` through the extension and assert it in pytest.
- **Caution:** The cores build for SSE2 only (`CMAKE_CXX_FLAGS` is empty and only `__SSE2__` is defined). If AVX is ever enabled, Eigen's vectorised reductions over unaligned maps choose their scalar head by the runtime address (`Eigen/src/Core/Redux.h:290`, `first_default_aligned`). That is a second, independent threat to bit-repeatability, which this fix does not cover.

#### [note] Pre-existing: the local-map search re-matches motion-model outliers, which ORB-SLAM2 prevents
- **Where:** `src/slam_it_all/edge/tracking.py:788` (`np.setdiff1d(local, already)`, where `already` holds only motion inliers) and `:815` (visible counts). This predates PR 10.
- **Issue:** ORB-SLAM2's `TrackWithMotionModel` sets `pMP->mnLastFrameSeen = mCurrentFrame.mnId` on each discarded outlier, and `SearchLocalPoints` skips points so marked: `if(pMP->mnLastFrameSeen == mCurrentFrame.mnId) continue;` ([Tracking.cc](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc); Mur-Artal & Tardós, ORB-SLAM2, T-RO 2017). Here a rejected point is projected, searched and counted visible again.
- **Evidence:** On the room (`edge_motion_outlier_rematch_probe.py`), motion outliers re-matched to the same point by the local-map search, and how many of those became inliers:

  | Mode | Motion outliers | Re-matched to the same point | Then inliers |
  |---|---|---|---|
  | RGB-D | 363 | 193 | 1 |
  | Monocular | 263 | 50 | 2 |
  | Stereo | 242 | 62 | 0 |

- **Effects:**
  - it inflates `local_map.matches`;
  - it adds visible counts to points tracking has just rejected, lowering the found ratio `MapPointCulling` uses;
  - it drives the rejected/unmatched split above.

  Outside this PR's goal: a backlog item.

#### [note] The second local-BA solve's initial cost is on the first solve's inliers only
- **Where:** `src/slam_it_all/edge/adjust.py:245-246` (outlier weights zeroed, as ORB-SLAM's `setLevel(1)` in `LocalBundleAdjustment`); `viewer/src/solver.js:63` (hint "cost before → after").
- **Issue:** The second solve's initial cost sits below the first solve's final cost, so two solves shown in a row read as more progress than happened.
- **Suggestion:** Add "the second on the first's inliers" to the hint.

### Checked and fine
- **Eigen fix.**
  - `setCpuCacheSizes` overrides the static `CacheSizes` that `manage_caching_sizes` queries once (`GeneralBlockPanelKernel.h:87`).
  - 32 KiB / 256 KiB / 2 MiB are indeed Eigen's x86 defaults (`:48-50`).
  - GEMM, triangular solves and blocked LLT updates all block through it.
  - The call precedes any product in both entry points; the libraries are static and the cores single-threaded.
- **The fate rule is identical in Python, C++ and Rust.**
  - Precedence is rejected, then tracked, then made (`_Step.fates`, C++ `mark`, Rust `Fates::mark`).
  - `matched` is each stage's pre-refinement matches. PnP's are the descriptor matches with a bearing in front, all -1 when it stops early.
  - Tracked comes from the final inliers, since `optimize_pose` sets outliers to -1.
  - At initialisation, made is every observed point.
  - Mapping's made is taken after triangulation and before fusion, with depth points kept through culling. This follows ORB-SLAM2's order (CreateNewKeyFrame, then MapPointCulling, CreateNewMapPoints, SearchInNeighbors), so "tracked, culled, then made again" on one keyframe is genuine behaviour.
- **Removal dating.**
  - The codes map to ORB-SLAM2's sites: CULLED is MapPointCulling and KeyFrameCulling; FUSED is SearchInNeighbors' `Replace`; WEAK is `EraseObservation`'s `nObs<=2`, with stereo observations weighted 2.
  - Every point erasure is labelled at its call site; removals inside initialisation are dated on the frame that started the map.
  - Ids are monotonic; the pipeline's frame counter equals the diagnostics line, since skipped RGB-D frames make neither a call nor a line.
  - The final map is the alive rows, and every consumer (evaluation, rerun, the header) reads `final_*`.
  - The mapping summary's counts are point-count differences in step order, and their sum is tested against the lives.
- **Bake alignment.**
  - `[[sR, t]]` maps the estimate onto the truth (`metrics.py`).
  - The rotation is `sR / cbrt(det)`, with det > 0 by Umeyama's sign fix (`alignment.py:113`); translations become `sR t + t0`, and points `sR p + t0`.
  - Sim(3) for monocular and SE(3) otherwise is the standard practice: Umeyama, TPAMI 1991; Sturm et al., IROS 2012; [evo](https://github.com/MichaelGrupp/evo)'s `--align --correct_scale`. Each backend is aligned by its own ATE alignment, as `eval.runs.evaluate` scores it.
- **Frame renumbering.**
  - Bundle frames are the union of the runs' diagnostics times, matched exactly, with a clear refusal otherwise.
  - Each run's span must be contiguous; lives, feature records and pose frames are mapped by diagnostics line.
  - The test with a run starting at frame 5 covers it.
- **Page logic.**
  - `alive()` and the shader use the same half-open `[made, removed)` rule, so a point made and fused on one frame is never shown, consistent with "the map after the frame".
  - float32 attributes are exact below 2^24, and `NEVER = 2^30` is exact.
  - The `.npy` header length is parsed, not assumed.
  - Latency is build plus track, as `budgets.judge`; the RPE is over one frame period, as evaluated; radians become degrees; the up axis follows the truth's world.
- **Streamed feature files.** The in-place header rewrite relies on numpy's growth-axis padding, mirrored in C++ (`npy_header`, `kGrowthDigits`).
- **BA summaries.** Costs are finite by construction, since `nls` refuses a non-finite initial cost, so the JSON writers' refusal of non-finite numbers cannot fire on a normal run.

Sources:
- [ORB-SLAM2 Tracking.cc](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc)
- [evo: Python package for the evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo)
- [evo on PyPI](https://pypi.org/project/evo)

## Run 2 -- 2026-10-10

The fixes of Run 1's findings, reviewed by all three: each changed its area substantially.

| | |
|---|---|
| **Branch** | `feature/run-viewer` |
| **Range** | `b93d66d...c78601a` (Run 1's head to the fixes and Run 1's report) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slam-it-all-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; pytest 3263 passed, node --test 32 passed), on the tree committed as `2174b0e`; the C++ comparison runs byte-identical to before on both core types |

Every Run 1 fix was found to do what its finding asked. A16 was checked before it was
recommended: with `exhaustive = true`, all five internal-order contracts are kept.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L9 | should-fix | algorithm | `bundles.py:121-124`; `test_bundles.py`; `scripts/run_vo.py:87-91` | the collinear-centres branch of L1's fix has no test, and `run_vo.py`'s `score` still raises on that case | fix | fixed (9d02e02): a run on a line tested through the bake and `run_vo.py`, which now reports it unscored -- decided by evan-wes, 2026-10-10 |
| A12 | should-fix | architecture | `eval/runs.py:760`; `orchestration/vo.py:84` | the list of datasets is in both eval and orchestration | fix | fixed (9d02e02): `runs.DATASETS`, `vo.DATASETS` removed, `open_source` refuses an unknown dataset -- decided by evan-wes, 2026-10-10 |
| A13 | should-fix | architecture | `tests/orchestration/test_bundles.py:26-30, 90` | nothing holds the golden bundle's fates and steps to `runs.FATES` and `runs.STEPS` | fix | fixed (9d02e02) -- decided by evan-wes, 2026-10-10 |
| A14 / H11 | should-fix | architecture, hygiene | `viewer/tsconfig.logic.json` | the no-DOM check covers only the modules it lists | fix | fixed (da7cea1): every module but `main.js` and `*_view.js` -- decided by evan-wes, 2026-10-10 |
| A15 / L10 | note | architecture, algorithm | `viewer/src/scene_view.js:88-105`; `main.js:278-282` | a run left in its own frame is drawn over the truth, with the truth's up axis | fix | fixed (da7cea1): the truth hidden and the camera's up for an unaligned run; checked in a browser -- decided by evan-wes, 2026-10-10 |
| A16 | note | architecture | `pyproject.toml`, the internal-order contracts | they could be exhaustive, so that a new module needs a layer | fix | fixed (9d02e02) -- decided by evan-wes, 2026-10-10 |
| L11 | note | algorithm | ladder :289-291; `docs/notes/viewer.md:68-71` | across machines faer may also vectorise differently (pulp's run-time dispatch), not only block | fix (the words) | fixed (f75c5d9) -- decided by evan-wes, 2026-10-10 |
| H10 | note | hygiene | `bundles.py`, `bake` | a bake killed hard leaves a hidden `.baking` folder beside the bundle's | fix (say so) | fixed (9d02e02) -- decided by evan-wes, 2026-10-10 |

### Hygiene review

## Hygiene review

**Scope:** Run 2 of PR 10, `b93d66d...c78601a`, my lane only. I read these in full: `orchestration/bundles.py`, `viewer/src/playback.js`, `viewer/src/main.js` and `viewer/tsconfig.logic.json`. I read these diffs: `eval/runs.py`, `eval/metrics.py`, `scripts/run_vo.py`, `pyproject.toml`, `scripts/check.sh`, `bundle.js`, `images.js`, `solver_view.js`, the C++ removal of `linalg.hpp`, and the new tests for the bake, `ground_truth`, `Playhead` and product blocking.

**Summary:** Each fix in my lane (H1, H2, H4, H5, H6, H7, H9, and H3 from the bake side) does what its finding asked. The fixes introduced no defects. I have two small notes and no must-fix or should-fix findings.

### Findings

#### [note] A bake killed hard leaves a hidden staging folder
- **Where:** `src/slam_it_all/orchestration/bundles.py`, the `staging` block at the end of `bake`.
- **Issue:** `except BaseException: shutil.rmtree(staging)` covers errors and Ctrl-C. A SIGKILL or power loss leaves `.<name>.<uuid>.baking` beside `out`. Nothing reads or sweeps it.
- **Why it matters:** It is harmless and hidden, and the next bake uses a new uuid. Disk is only wasted until someone deletes the folder.
- **Evidence:** `staging = target.with_name(f".{target.name}.{uuid.uuid4().hex}.baking")`
- **Suggested fix:** None needed. Optionally mention the `.baking` suffix in the `bake` docstring so a user knows it is safe to delete.

#### [note] The logic tsconfig's file list is hand-maintained
- **Where:** `viewer/tsconfig.logic.json`, the `include` list.
- **Issue:** A new DOM-free module has to be added by hand, or it silently stays outside the no-DOM check.
- **Why it matters:** The A4 guarantee, that logic modules never touch the DOM, erodes quietly.
- **Suggested fix:** None needed now. If it bites, add a node test that every `viewer/src` module not named `*_view`, `main` or `scene_view` appears in the list.

### Checked and fine
- **H3 / L1 (bake):**
  - `_score` catches only `InvalidInputError` and `DegenerateError`, and its docstring says why those are the only refusals left.
  - The unplaced run is baked in its own frame with `alignment` and `score` null.
  - Staging is created beside `out` and moved on with `staging.replace`, and the temp folder is removed on any exit.
  - Tests cover a never-initialised run beside an aligned one, and a failed bake that leaves nothing behind and can be re-run.
- **H4 / A5:**
  - `bake` is now small. `_bundle_frames`, `_write_truth`, `_write` and `_score` each do one thing.
  - `runs.ground_truth` validates the dataset and has a test, including the bad-dataset message. `run_vo.py` uses it, and the duplicate `Truth` and `ground_truth` are gone.
  - `AbsoluteError.similarity` replaces the boolean-index derivation, and the docstrings are updated.
  - `_ALIGNMENTS` is deleted, and the stale `_truth` is gone with no leftover references.
- **A3:** The import-linter contract for orchestration is in place with a stated reason.
- **L4:** The new `_bearings` check pins points and keyframes under both SE(3) and Sim(3).
- **H1:**
  - `Playhead` is DOM-free and JSDoc'd, and it takes its clock from the caller.
  - `bindPlayback` guards against double scheduling with `pending`.
  - The 4 tests cover clamping, speed, stopping at the end, restarting from the first frame, and a stopped playhead.
- **H2:** `choose` records `chosen` before the await and discards a stale result afterwards. The loads are cached per backend as promises, so a switch is cheap.
- **H6 / A7:**
  - `loadBundle` compares `fates` and `steps` against the page's tables and names the table that differs.
  - It rejects backends the page does not know. The bundle docstring and the JSDoc typedef match.
- **H5:** Prettier is pinned at 3.9.10 with a 100-column config, it runs in `check.sh`, and the formatted files pass.
- **H7:** `largest()` iterates instead of spreading, skips non-finite values, and takes a floor. Both call sites are converted.
- **H9:** `SolverView` now has class-level JSDoc, and `scene_view.js` is in the diff.
- **L3:** `images.onCanvas` carries the pixel-centre convention in its docstring.
- **C++ (A1 / L6):**
  - `linalg.hpp`, its test and every call site are removed, and no references remain outside the review report.
  - `test_product_blocking.cpp` fails the build if `EIGEN_NO_CPUID` is missing.
  - The `Fate` doc comment now says what the code does.
- **Stale material:** no debug prints, commented-out code, TODOs or hard-coded paths in the changed files.

### Architecture review

## Architecture review

**Scope:** Run 2 of PR 10 (`feature/run-viewer`). I reviewed only the fixes made after Run 1: `b93d66d...c78601a` (07a505f, 038aaf3, adbced9, 2174b0e; I read c78601a's report table for the decisions). For each fix in my lane (A1, A2, A3, A4, A5, A7, A8, A11), I checked whether it does what its finding asked and whether it introduced anything new. I did not re-raise the deferred or waived findings (A6, A9, A10, L7, H8). I read the changed files and the neighbouring code each fix relies on (`orchestration/vo.py`, `eval/runs.py`, `viewer/src/scene_view.js`, `viewer/src/main.js`, the golden-bundle tests). Probes ran outside the repo: scratch tsconfigs and an import-linter config in the session scratchpad, plus a compile-commands scan. I also ran the already-built `foundations_test_product_blocking`. The repository was not modified and HEAD did not move.

**Summary:** Every fix in my lane does what its finding asked, and I found no must-fix.
- **A1:** the compile-time define reaches every translation unit that uses Eigen.
- **A2, A11:** the ladder doc now places the fourth language and the bake, and the wording is corrected.
- **A3:** the new orchestration contract holds.
- **A5:** the bake reads its alignment from the evaluation.

Three small should-fixes are left, where a fix only half-closes what it set out to:
- the dataset list now lives in two layers;
- nothing in layer 1 holds the bundle's fates and steps against the Python ones;
- the logic-module type check covers only the files it lists.

### Findings

#### [should-fix] The dataset list now exists in both orchestration and eval
- **Where:** `src/slam_it_all/eval/runs.py:760-761` (the literal `("tum", "euroc")` in `ground_truth`), and `src/slam_it_all/orchestration/vo.py:84` (`DATASETS = ("tum", "euroc")`). Each layer also branches on the names: `vo.py:207-208` (the check and its message), `vo.py:603` (`open_source`: anything that is not `"tum"` is treated as EuRoC) and `scripts/run_vo.py:199`.
- **Issue:** A5 moved "how to read a sequence's ground truth" into eval, the layer that owns the readers. But the list of datasets that have readers stayed in orchestration, and `runs.ground_truth` now keeps its own copy. eval cannot import orchestration, so nothing keeps the two copies in step.
- **Why it matters:** PR 11 adds a third dataset (Stray Scanner recordings). If it is added to `vo.DATASETS` and `open_source` but not to `runs.ground_truth`, the bake raises `InvalidInputError` on every phone bundle. `bake` calls `runs.ground_truth` outside `_score`, so this is a whole-bake failure, not "unscored". This is the same kind of problem A5 was about: a lower layer's rule restated in two places.
- **Evidence:** `git grep -n '"tum", "euroc"' -- src scripts` finds `runs.py:760` and `vo.py:84`. No test compares them; `tests/eval/test_runs.py:454` only checks that `"kitti"` is refused.
- **Suggested fix:**
  - Give eval the list: `runs.DATASETS`, beside `ground_truth`, the lowest module that dispatches on it.
  - Have `vo.check`, `run_vo.py`'s `choices` and the error messages use it, and drop `vo.DATASETS` (a clean break).
  - Optionally, have `open_source` refuse an unknown dataset instead of falling through to EuRoC. That branch predates this PR.

#### [should-fix] The bundle's fates and steps are checked only when a bundle is loaded; layer 1 does not tie them to the Python tables
- **Where:**
  - `tests/orchestration/test_bundles.py:26-30` (`_keys`: a list becomes the string `"list"`) and `:90` (the golden test compares the layout only);
  - `tests/fixtures/orchestration/bundle/bundle.json:27-38`;
  - `viewer/tests/bundle.test.js:111`.
- **Issue:** A7 asked for the page's copies of `FATES` and `STEPS` to be held to `eval.runs`.
  - The fix has the bake write both tables into `bundle.json`, and `loadBundle` refuses a bundle whose tables differ from the page's. That catches drift, but only when a bundle is opened.
  - In layer 1, the JS tests hold the page to the golden `bundle.json`. No Python test holds the golden `bundle.json`'s values to `runs.FATES` / `runs.STEPS`, because the layout test compares only JSON keys and value types.
- **Why it matters:** Suppose a fate or step is added consistently in Python, C++ and Rust (`test_runs.py:108` keeps `Fate` and `FATES` together). Then:
  - the Python layout test still passes, since the keys are unchanged;
  - the JS tests still pass against the stale golden;
  - the first real bundle baked afterwards is refused by the page.

  Layer 1 stays green while the page and the bake disagree.
- **Evidence:** `_keys(value)` returns `type(value).__name__` for anything that is not a dict, so `_layout` sees `"fates": "list"`. `git grep -n "FATES\|STEPS" -- tests/orchestration/test_bundles.py` finds nothing.
- **Suggested fix:** one assertion in `test_the_bake_keeps_the_golden_bundles_layout`, or a test of its own: the golden `bundle.json`'s `fates` and `steps` equal `list(runs.FATES)` and `list(runs.STEPS)`. Then a change in Python fails until the golden is regenerated, and the JS tests fail until the page is updated, which closes the chain.

#### [should-fix] The no-DOM type check covers only the files it lists, so a new logic module is unchecked by default
- **Where:** `viewer/tsconfig.logic.json:5-16` (an explicit `include` of nine files); `scripts/check.sh:110`. The rule is stated as checked at `CLAUDE.md:39` ("Logic modules never touch the page (checked: `tsc` without the DOM)").
- **Issue:** A4 is fixed for today's modules. I confirmed in a scratch config that a listed module which imports `three` or touches `document` fails, as it should. But the list opts modules in. A new `viewer/src/*.js` that is not added to the list is type-checked only with the DOM, so it can touch the page and still be counted as logic. The ladder contract in `pyproject.toml:118-124` uses `exhaustive = true` for exactly this reason.
- **Why it matters:** PR 11 adds page code. The split that keeps the logic under `node --test` would then depend on remembering to edit a list, which is the convention-only state A4 was raised against. The views already follow a naming convention (`main.js`, `*_view.js`), so it can serve as the boundary.
- **Evidence:** I ran a scratch tsconfig in the session scratchpad that extends `viewer/tsconfig.json`, with `"lib": ["es2022"]`, `"include": ["logic.d.ts", "src/*.js"]` and `"exclude": ["src/main.js", "src/*_view.js"]`. `--listFilesOnly` selects the same nine logic modules as `tsconfig.logic.json`, and `tsc -p` exits 0.
- **Suggested fix:** switch `tsconfig.logic.json` to that opt-out form, so every module is logic unless named as a view. Add a sentence to `CLAUDE.md` or the viewer note saying the view files are `main.js` and `*_view.js`.

#### [note] The 3D view picks its world (truth drawn, z up) once per bundle; alignment is now per run
- **Where:** `viewer/src/scene_view.js:88-105` (the up axis and the truth line, set once in the constructor from `bundle.info.truth`); `viewer/src/main.js:278-282` (only the other runs' lines are filtered by alignment).
- **What happens:** After A5 and L1, `bundle.json` records each run's alignment separately (`"sim3"`, `"se3"` or null), and the header says when a run is "not aligned". But `SceneView` still draws the truth and uses the truth's z-up whenever the bundle has ground truth. An unaligned run, which is in its own first-camera frame, is therefore drawn over the truth, and `fit` frames both. For the tested case (a monocular run that never initialised) nothing shows, since there are no poses. A run rejected for collinear centres would be drawn in the wrong frame next to the truth.
- **Option:** in `setRun`, hide the truth line and use the run's own up axis (-y) when `run.info.alignment === null`. The algorithm reviewer may want to confirm the frame semantics.

#### [note] The internal-order contracts could be exhaustive, as the ladder's is
- **Where:** `pyproject.toml:189` (orchestration), and likewise `:139`, `:173` and `:220`.
- **What exists:** the A3 contract (`(bundles)` above `vo`) is in place and kept. Like the eval and edge contracts, it constrains only the modules it lists. PR 11's `worker` will be unconstrained until someone adds it, so `vo` could import it without anything failing.
- **Evidence:** a scratch import-linter config with `exhaustive = true` on the orchestration and eval contracts, run at HEAD: `Orchestration's internal order (exhaustive, scratch) KEPT`, `Eval's internal order (exhaustive, scratch) KEPT`, `2 kept, 0 broken`.
- **Option:** add `exhaustive = true` to the internal-order contracts. Then a new module fails until it is given a layer, which is the ladder contract's stated rule. The backlog is a fine place for this.

### Checked and fine
- **A1, how the define reaches the code:**
  - `EIGEN_NO_CPUID` is a PUBLIC definition in `slam_it_all_add_rung` (`CMakeLists.txt:38`). Every program, test, benchmark and the extension links a rung, so all of them get it.
  - Compile commands: `build/native` 76/76 and `build/bench` 41/41 translation units have it. The extension has 67/78; the 11 without it are nanobind's own sources, which do not include Eigen. So Eigen's inline cache query is compiled one way everywhere it is used.
  - `foundations_test_product_blocking` prints `ok`.
  - `fix_product_blocking` and `linalg.hpp` are gone from the code and docs (`git grep` finds only the review report).
- **A1, behaviour and docs:**
  - The new test checks both the define and the defaults that result from it.
  - CLAUDE.md, the ladder (`:284-293`), the viewer note and the CHANGELOG describe the compile-time fix.
  - On ARM the sizes are now Eigen's ARM defaults (16 KiB / 512 KiB / 512 KiB), not the x86 sizes the removed code forced on every architecture. Repeatability on one machine still holds, and the ladder already rules out cross-architecture identity (FMA packets, `:279-282`). If equal blocking across architectures is ever wanted, `EIGEN_DEFAULT_L{1,2,3}_CACHE_SIZE` pins it.
- **A2:**
  - the ladder has a JavaScript row in Languages (`:85`), Pillow's JPEG encoding (`:90-91`), the widened orchestration row with `bundles` above `vo`, and `viewer/` placed outside the package (`:358-361`);
  - "every language" now appears in Review process;
  - `orchestration/__init__.py` and `eval/__init__.py` list the bake, the JPEG encoding and `ground_truth`.
- **A3:** the contract exists. `vo` imports nothing from `bundles`, and `bundles` imports only eval, `foundations.errors` and `vo`.
- **A5:**
  - `runs.ground_truth` replaces both copies, and the `run_vo.py` copy is deleted. `runs` importing `tum` and `euroc` is allowed by eval's internal order. `vo` already imported both, so the budgeted program loads nothing new.
  - The alignment label is read from `AbsoluteError.similarity`, which is Python-only, so there is no native counterpart to keep in step.
  - `bake` is now split into small helpers, and every check runs before the first write.
- **A7:** `loadBundle` refuses other fates, other steps and an unknown backend, and `bundle.test.js` tests all three. The page needs no `removals` table, since it never uses removal codes.
- **A8:** `viewer/tests/page.test.js` holds the import map's three.js version to `@types/three`.
- **A11:** the docs now say that faer reads its cache sizes once per run for the whole machine.
- **Writing a bundle (H3, architectural angle):**
  - the bundle is staged in a uuid-named sibling on the same filesystem, so `Path.replace` onto the new or empty `out` is a single atomic rename;
  - on any failure, the staging folder is removed;
  - there are no globals and no import-time side effects.
- **Deferrals:** A6, A9 and A10 are recorded accurately in the ladder's backlog (`:920-940`), with the report linked.
- **Layering in the page:** `bundle.js` now imports `compare.js` and `frames.js` for their tables. There is no runtime import cycle (`compare.js` imports only `frames.js`), and no logic module imports a view or `three`.

Paths: `src/slam_it_all/eval/runs.py`, `src/slam_it_all/orchestration/vo.py`, `tests/orchestration/test_bundles.py`, `viewer/tsconfig.logic.json`, `viewer/src/scene_view.js`, `pyproject.toml`, `CMakeLists.txt`.

### Algorithm review

## Algorithm review

**Scope:** Run 2 of the panel on PR 10, covering only the fixes made after Run 1: `b93d66d...c78601a` on `feature/run-viewer`, commits 07a505f, 038aaf3, adbced9 and 2174b0e (c78601a is the report). I read the following in full:
- `src/slam_it_all/orchestration/bundles.py` at HEAD;
- the `eval/runs.py`, `eval/metrics.py` and `scripts/run_vo.py` diffs, plus `runs.evaluate`, `metrics.absolute_error`/`relative_error`, `foundations/alignment.py` and the truth readers they rely on;
- `CMakeLists.txt` and `cpp/tests/foundations/test_product_blocking.cpp`, checked against Eigen 5.0.1-dev's headers in the env;
- in the viewer: `images.js`, `geometry.js`, `image_view.js`, `playback.js`, `solver.js` and `bundle.js`, the alignment handling in `main.js` and `scene_view.js`, and their tests;
- the doc changes in 2174b0e, with gemm-common 0.19.0 / private-gemm-x86 0.1.20 / pulp 0.22.3 sources checked for the Rust wording.

The repository is untouched: the tree is clean and HEAD is still c78601a. Probes ran from the session scratchpad. Two are saved in a private scripts folder: `viewer_bake_invariant_mutants_probe.py` and `viewer_score_collinear_probe.py`.

**Summary:** Every fix in my lane does what its finding asked, and none of them introduces a numerical fault:
- **L1:** an unalignable run now bakes unscored in its own frame, and a failed bake leaves nothing behind.
- **L2:** the fate wording is accurate in all three languages, the page and the docs.
- **L3:** keypoints and frustum corners now follow the pixel-centre convention.
- **L4:** the new invariant test catches real mistakes.
- **L5:** the Rust wording matches the crates.
- **L6:** `EIGEN_NO_CPUID` matches the old runtime fix exactly, and reaches every translation unit that includes Eigen.
- **L8:** the hint matches the solves.

What is left: one should-fix (the collinear-centres branch the fix claims to handle has no test, and `run_vo.py` still crashes on that case) and two notes.

### Findings

#### [should-fix] The collinear-centres branch of the L1 fix has no test, and `run_vo.py` still crashes on that case
- **Where:**
  - `src/slam_it_all/orchestration/bundles.py:121-124` (`_score` catches `(InvalidInputError, DegenerateError)`);
  - `tests/orchestration/test_bundles.py` (`test_a_run_its_alignment_cannot_place_stays_in_its_own_frame`, which covers only the too-few-matches path);
  - `scripts/run_vo.py:87-91` (`score` catches `InvalidInputError` only).
- **Issue:** The module docstring, `_score`'s docstring, the page's "not aligned" hint and `docs/notes/viewer.md` all say a run whose centres lie on a line stays unaligned. No test exercises that case: narrowing `_score`'s `except` to `InvalidInputError`, as `run_vo.score` does, would still pass every test. The fix also routed the bake and `run_vo.py` through the shared `runs.ground_truth`, but the two consumers now treat the same refusal differently. The bake places the run unscored; the script raises `DegenerateError` after the run has been written.
- **Why it matters:** The branch is rare on real data, since a noisy track is almost never collinear to `DEGENERACY_RATIO = 1e-10`. But it is a documented behaviour with no test holding it, and the two scorers have already drifted apart.
- **Evidence:** `viewer_score_collinear_probe.py` scores a room RGB-D run (12 frames) with its centres forced onto a line:
  ```
  bake _score: None
  run_vo score raised: DegenerateError umeyama: points are collinear or coincident
  ```
- **Suggested fix:**
  - Add a unit test: `_score(replace(run, poses=<collinear centres>), truth) is None`. It takes a few lines and no extra pipeline run.
  - Catch `DegenerateError` in `run_vo.score` as well, printing the reason as it does for the other refusal.

#### [note] A run left in its own frame is drawn with the truth's line and the truth's up axis
- **Where:**
  - `viewer/src/scene_view.js:88-105`: the truth's `LineSegments` and `camera.up = (0, 0, 1)` are set once per bundle from `bundle.info.truth`;
  - `viewer/src/main.js:278-282`: only the *other backends'* tracks are filtered by alignment.
- **Issue:** In a bundle with ground truth, choosing a run whose `alignment` is null shows its trajectory, keyframes and points in its first camera's frame (y down). They appear on top of the truth's centres, which are in the truth's world (z up), and `fit()` frames both together. The header's hint correctly says the run is "shown in its own frame and not scored". The 3D panel still overlays two unrelated frames.
- **Why it matters:** The effect is small today, because an unalignable run has at most a couple of tracked frames, often none. But it is a frame mix, and the comment at `scene_view.js:88` ("without one, a run's world is its first camera's") no longer covers every case.
- **Suggestion:** When the chosen run's `alignment` is null, hide the truth line and use `up = (0, -1, 0)`, as for a bundle without truth.

#### [note] Across machines, faer's results can also differ by SIMD dispatch, not only by blocking
- **Where:**
  - `docs/design/2026-10-02-slam-it-all-ladder.md:289-291` ("Rust's products (faer's) block by cache sizes read once per run for the whole machine ... though another machine may block them differently");
  - `docs/notes/viewer.md:68-71`.
- **Issue:** The sentence is now correct. `gemm-common-0.19.0/src/cache.rs:73-215` walks every `/sys/devices/system/cpu/cpu*/cache/index*` with a last-one-wins merge, behind a `OnceCell` (`:475`). `private-gemm-x86-0.1.20/src/cache.rs:8-150, 356` does the same behind a `OnceLock`. So the result is the same on every core of one machine. But faer also picks its kernels at run time through pulp (`pulp-0.22.3/src/core_arch/x86/mod.rs`, `is_x86_feature_detected!`): x86-64-v3 (AVX2 + FMA) here, and v4 (AVX-512) on many cloud hosts. So a Rust run on another machine can round differently even where no product is large enough to block. The C++ cores, compiled for SSE2 with `EIGEN_NO_CPUID`, have no runtime dispatch of their own.
- **Why it may matter:** This is context for PR 11's Railway worker. A Rust bundle baked there may not match a desktop run bit for bit, while C++ should. No code change is needed. If the sentence is touched again, "block or vectorise them differently" covers both causes.

### Checked and fine
- **L6 / A1, `EIGEN_NO_CPUID`:**
  - In Eigen 5.0.1-dev, `queryCacheSizes` (`Memory.h:1279-1311`) sets -1 without `EIGEN_CPUID`, and `CacheSizes` (`GeneralBlockPanelKernel.h:74-77`) then falls back to 32 KiB / 256 KiB / 2 MiB on x86. Those are the sizes `fix_product_blocking` set, so the change is result-preserving.
  - `queryCacheSizes` has no other caller in Eigen.
  - Every translation unit that includes Eigen gets the define: `build/native` 76 of 76, `build/bench` 41 of 41, `build/slam-it-all` 67 of 78. The 11 without it are nanobind's own sources, which have no Eigen include path, so there is no ODR split.
  - The golden bundle's arrays did not change. Its `records.json` files differ only in `read_seconds`, `build_seconds` and `track_seconds`.
  - The native test catches both a missing define and a non-default size.
- **The AVX caution (ladder :293-295):** it is accurate. With SSE2, a reduction's scalar head depends on an address mod 16, and glibc and Eigen allocations are 16-byte aligned, so the head is fixed by the code. AVX's 32-byte packets would make it depend on where the heap happens to place an allocation.
- **L1:**
  - `_score` runs before anything is written.
  - The "only refusals left" claim holds: `read_trajectory` and the truth readers already run `_as_stamped` / `check_transform`, so `evaluate` can only refuse for fewer than 3 matches, no RPE pair, or a degenerate alignment. Sim(3)'s scale is positive whenever `align` returns.
  - Probe at HEAD: a 5-frame monocular run bakes with `truth: true, alignment: null, score: null`, and nothing is left beside the bundle.
  - Staging plus `Path.replace` onto a new or empty folder is atomic on Linux, and the failure test covers it.
- **L4:** the bearing test in `test_each_backend_is_moved_onto_the_truth` discriminates. `viewer_bake_invariant_mutants_probe.py` gives:

  | Case | Max bearing error |
  |---|---|
  | Correct bake, RGB-D (SE(3)) | 4.5e-8 |
  | Correct bake, monocular (Sim(3), scale 9.70) | 4.0e-8 |
  | Points not scaled, monocular | 1.18 |
  | Points not moved | 0.21 / 0.63 |
  | Inverse similarity | 0.31 / 0.75 |

  So every mistake fails the test's `atol=1e-5`, against a correct bake's ~4e-8.
- **`AbsoluteError.similarity`:** it is `with_scale` itself, so the bundle's `"sim3"` / `"se3"` comes from the fit that was run, and it is tested.
- **L2:** the new wording is right in Python, C++, Rust, `eval.runs.FATES`, `frames.js` and the viewer note. UNMATCHED means not made and not matched by the last stage, since tracked is a subset of the last stage's matches. The note's "rejected only if the local map's search matches it again" is accurate. The L7 deferral text matches ORB-SLAM2's `mnLastFrameSeen` skip in `Tracking.cc` and Run 1's numbers.
- **L3:**
  - `images.onCanvas` adds 0.5 on a canvas sized to `camera.width × camera.height` (`image_view.js:40-41`) with the bitmap drawn at that size.
  - Frustum corners at (-0.5, -0.5) and (w-0.5, h-0.5) are the image's edges under the OpenCV centre convention.
  - Both are tested with exact values.
- **L8:** the hint "the second solve is over the first's inliers, so it starts lower" holds on the saved real-data bundles. Of 1,566 two-solve local BAs (MH01-stereo and desk-rgbd, all backends), the second started lower in 1,466 and equal in 100 (no outliers dropped), never higher. The room fixture's 23 are all equal, since it has no outliers. The first solve's final cost and the second's initial cost use the same Huber loss, so the comparison is like for like.
- **L5 / A11:** "read once per run for the whole machine" matches both gemm crates' sysfs walk. `docs/notes/viewer.md:180` ("C++'s ... block by fixed sizes") is still true.
- **`Playhead`:** it advances to the last frame whose time is at or before the clock, and stops at the end. Float32 times are adequate for any sequence length the datasets have.
- **`runs.ground_truth`:** it returns the same arrays the old `run_vo.ground_truth` did, as tested against `tum.read_sequence`.
