# Review panel: `feature/2a-a1-measure-and-import`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-13

| | |
|---|---|
| **Branch** | `feature/2a-a1-measure-and-import` |
| **Range** | `2aa96fe...a533b13` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed (`scripts/check.sh` in the `slam-it` env, exit 0: 25 checks; pytest 3661 passed) |
| **Security** | [the security pass](2026-10-13-feature-2a-a1-measure-and-import-security.md), whose S1 overlaps H2 and A4 |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `scripts/measure_testset.py:~103`; `worker.py` `options_for` | The features-by-size recipe is written twice; the worker reads `camera.txt` twice | fix (one setup helper both use, with A6; the camera read once) | fixed (753b738: `vo.options_for`, which the worker and the measure share; the camera read once) -- decided by evan-wes, 2026-10-13 |
| H2 | should-fix | hygiene | `zju.py:118-127`; `video.py:~610` | PNG-header size parsing is hand-written twice | fix (with S1: sizes from Pillow's own parse, in `images`) | fixed (84508ce: sizes from Pillow's own parse, `images.size`) -- decided by evan-wes, 2026-10-13 |
| H3 | should-fix | hygiene | `zju.py:~95`, `video.py` `_write_frames`, `options.py` `features_for`, `advio.py` `read_calibration` | Magic numbers: the no-depth `1000.0`, `1 << 30`, the `1e-4` rigid tolerance and its unreconciled comment | fix (named; `MAX_IMAGE_SIDE` in `features_for`; the tolerance named and its comment made true) | fixed (9ae0734: `tum.UNUSED_DEPTH_SCALE`, `MAX_IMAGE_SIDE`, `_RIGID_TOLERANCE` with a true comment) -- decided by evan-wes, 2026-10-13 |
| H4 | should-fix | hygiene | `zju.py:~215` | A `data.csv` name is joined to the images folder unchecked | fix (with S5: a name leaving the images folder refused) | fixed (84508ce) -- decided by evan-wes, 2026-10-13 |
| H5 | should-fix | hygiene | `scripts/measure_testset.py:~130` | `budgets.judge` runs under a blanket `suppress` | fix (the reason recorded beside p95) | fixed (753b738: `unjudged` says why) -- decided by evan-wes, 2026-10-13 |
| H6 | should-fix | hygiene | `src/slam_it/eval/video.py` | 937 lines with two jobs: the decoder and Stray Scanner's zip | defer (Stray's reader to `eval.stray`, with A7; the ladder's open items) | deferred (the ladder's open items, A1's panel, c832973) -- decided by evan-wes, 2026-10-13 |
| H7 | note | hygiene | `viewer/src/jobs.js` `SHORT_SIDE` | The page's `SHORT_SIDE` is not held equal to Python's | fix (A1) | fixed (c832973, with A1) -- decided by evan-wes, 2026-10-13 |
| H8 | note | hygiene | `viewer/upload.html` | "A Stray Scanner recording runs RGB-D" is broader than the behaviour | fix ("a recording with LiDAR depth") | fixed (c832973) -- decided by evan-wes, 2026-10-13 |
| A1 | must-fix | architecture | `viewer/src/jobs.js:53,56`; `tests/server/test_shared_names.py` | `SHORT_SIDE` and `MODES` are not held equal to Python's, as CLAUDE.md requires | fix (both asserted in `test_shared_names.py`) | fixed (c832973: `SHORT_SIDE` and `MODES`, against `worker.RUN_MODES`, in `test_shared_names.py`) -- decided by evan-wes, 2026-10-13 |
| A2 | should-fix | architecture | `scripts/measure_testset.py:56,146` | The measure imports the worker, so needs PyAV, and ties the yardstick to the status page's summary | fix (the tracking span in `eval.runs`; worker and measure build on it; the measure drops the worker) | fixed (753b738: `runs.tracking` and `runs.LOST`; the measure no longer imports the worker, tested) -- decided by evan-wes, 2026-10-13 |
| A3 | should-fix | architecture | `arkitscenes.py:159-182`, `zju.py:219-228`, `advio.py:244-245` | The new importers are not staged (a failed ARKitScenes import leaves a half folder), and record nothing of what made them | fix (one staging helper for all four imports; ADVIO's truth written inside the stage; a `source.json` naming the importer and, with depth, its threshold) | fixed (9ae0734: `tum.staged` and `tum.write_source` in all four imports; ADVIO's truth inside the stage) -- decided by evan-wes, 2026-10-13 |
| A4 | should-fix | architecture | `zju.py:122`; `video.py:610, 617-641` | PNG parsing outside `images`; each LiDAR map goes through a scratch file because `images` reads only paths | fix (with S1: `images` decodes bytes with a size bound; the scratch file dropped) | fixed (84508ce: `images` decodes bytes with a size bound; the scratch file gone) -- decided by evan-wes, 2026-10-13 |
| A5 | should-fix | architecture | `eval/__init__.py`; `worker.py:17`; ladder `:367` | The package map, the worker's docstring and the ladder's layout row describe 0.3.7.0 | fix (the three updated) | fixed (c832973) -- decided by evan-wes, 2026-10-13 |
| A6 | note | architecture | `edge/options.py:423` | `features_for` sits in a module the cores mirror, but is Python-only | fix (with H1: its docstring says it is a Python-only setup helper; the `Options` built in one place) | fixed (753b738: one `vo.options_for`; `features_for`'s docstring says Python's alone) -- decided by evan-wes, 2026-10-13 |
| A7 | note | architecture | `src/slam_it/eval/video.py` | Stray Scanner's zip format lives inside the decoder | defer (with H6) | deferred (with H6) -- decided by evan-wes, 2026-10-13 |
| A8 | note | architecture | `scripts/measure_testset.py:174` | p95 is kept without the frame period it is read against | fix (`deadline` and `late` recorded and tabled) | fixed (753b738: `deadline` and `late`; the table shows the p95 of the period) -- decided by evan-wes, 2026-10-13 |
| L1 | must-fix | algorithm | `scripts/measure_testset.py:169`; `runs.py:838-846`; `test_arkitscenes.py:81-83` | ARKitScenes is never scored: its 10 Hz reference fails the RPE, which takes the ATE with it | fix (the ATE scored on its own; tests with a reference sparser than the frames) | fixed (753b738: `runs.evaluate_absolute`; tests with a reference at a third of the frame rate) -- decided by evan-wes, 2026-10-13 |
| L2 | should-fix | algorithm | `zju.py:168-180`; `test_zju.py:35-49` | The ground truth's own extrinsic is identity in the test, so its inversion and order are untested | fix (a test with a turned, offset ground-truth extrinsic) | fixed (9ae0734: a turned, offset ground-truth extrinsic, to 1e-12) -- decided by evan-wes, 2026-10-13 |
| L3 | note | algorithm | `arkitscenes.py:98-128` | ARKitScenes' reference sits half a frame (16 ms) from each paired frame: about 3-10 mm | waive in code: a reference is not interpolated; the results note says so | waived: a reference is not interpolated; the results note says so -- decided by evan-wes, 2026-10-13; reopened by Run 2's L9 and fixed by interpolating at the frames' times (5871936), decided by evan-wes, 2026-10-13 |
| L4 | note | algorithm | `arkitscenes.py:94` | ARKitScenes' per-frame fx spans about ±2%; the median hides it | fix (the spread in the docstring and the note); defer Stray's per-frame intrinsics (the ladder's open items) | fixed (9ae0734: the spread, measured, in the docstring); deferred (the ladder's open items, c832973): Stray Scanner's per-frame intrinsics -- decided by evan-wes, 2026-10-13 |
| L5 | note | algorithm | `advio.py:23, 59` | iphone-01's `p2` (0.0473) moves edge pixels up to 54 px: likely wrong, beside its `cy` | fix (the docstring flags `p2`; 18-19 checked before the note uses them) | fixed (9ae0734: `p2` flagged, its shift measured) -- decided by evan-wes, 2026-10-13 |
| L6 | note | algorithm | `scripts/measure_testset.py:118-130` | `--camera nominal` keeps the true `bf`, so an RGB-D run's baseline moves with the guess | fix (`bf` scaled with the guessed focal, the baseline kept) | fixed (753b738: `bf` scaled with the guess, the baseline kept) -- decided by evan-wes, 2026-10-13 |
| L7 | note | algorithm | ADVIO `gyro.csv` | ADVIO's gyroscope runs about 0.3 s behind its frames | defer (milestone 6, the ladder's open items) | deferred (the ladder's open items, milestone 6, c832973) -- decided by evan-wes, 2026-10-13 |
| L8 | note | algorithm | the test set | ScanNet++ gives the LiDAR path an independent reference | defer (the ladder's test set, for a later revision) | deferred (the ladder's open items, c832973) -- decided by evan-wes, 2026-10-13 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 2aa96fed...a533b13c` (commits 257ea81..a533b13). I read in full the new readers (`zju.py`, `advio.py`, `arkitscenes.py`, `lidar.py`, `sensor_yaml.py`), `video.py`, `scripts/measure_testset.py`, and the diffs of `options.py`, `worker.py`, `lenses.py`, `euroc.py`, `images.py` and the viewer. I only skimmed the tests, checking that coverage exists.
**Summary:** This is clean, well-documented code with strong boundary validation. Errors name the file, line and value, there are no swallowed exceptions, and the tolerances and caps carry stated reasons. I found no must-fix items. The findings are within-language duplication that the copy-paste tool cannot see, a few small magic numbers, and one doc claim that is slightly broader than the behaviour.

### Findings

#### [should-fix] The features-by-size recipe is written twice
- **Where:** `scripts/measure_testset.py:~103-106` (`measure`) and `src/slam_it/orchestration/worker.py` (`options_for`).
- **Issue:** `worker.options_for(sequence)` reads the camera and returns `Options(n_features=features_for(w, h))`. `measure` re-implements the same thing inline, `Options(n_features=features_for(camera.width, camera.height)) if by_size else Options()`, after reading the camera itself.
- **Why it matters:** The script and the demo worker are meant to apply the same rule. If one changes (say a different floor or extra options), the measurements stop describing what the worker runs.
- **Evidence:** `measure` already imports `worker` and calls `worker.summarize`.
- **Suggested fix:** Use `worker.options_for(folder) if by_size else Options()`. Keep the `camera` read in `measure` only for the size fields. `worker._work` also reads `tum.read_camera(sequence)` twice, once in `options_for` and once for the summary. Have `options_for` take the `tum.Camera`, or read it once.

#### [should-fix] The PNG-header size read is written twice, and the PNG signature check three times
- **Where:** `src/slam_it/eval/zju.py:118-127` (`_image_size`) and `src/slam_it/eval/video.py:~608-616` (`_read_map`).
- **Issue:** Both check the `\x89PNG\r\n\x1a\n` signature and `IHDR`, read big-endian width and height at 16:24, and range-check the result. The two differ only in the cap and the error text.
- **Why it matters:** This is hostile-input parsing, the kind where a fix in one copy should not miss the other.
- **Suggested fix:** Add `images.png_size(data: bytes) -> tuple[int, int]` that raises `InvalidInputError`. Each caller applies its own cap, `MAX_IMAGE_SIDE` or `STRAY_MAP_SIDE`.

#### [should-fix] Magic numbers and the repeated `1000.0` depth scale
- **Where:**
  - `zju.py:~95` and `video.py` (`_write_frames`, the `1000.0,  # no depth maps` argument)
  - `src/slam_it/edge/options.py` `features_for`, `1 << 30`
  - `src/slam_it/eval/advio.py` `read_calibration`, `atol=1e-4`
- **Issue:**
  - The placeholder depth scale `1000.0` is an unnamed literal in two places. `lidar.DEPTH_SCALE` is the same value with a name, and the `video.py` comment explains it while `zju.py` does not.
  - `1 << 30` is the upper bound for a frame side in `features_for`. `foundations.arrays.MAX_IMAGE_SIDE` exists for exactly this.
  - The `1e-4` rotation tolerance is not named. The later comment says "Kalibr's, to 1e-6", which is a different figure from the 1e-4 check and is not reconciled with it.
- **Suggested fix:** Name each one, for example a `NO_DEPTH_SCALE` constant, `MAX_IMAGE_SIDE`, and `_RIGID_TOLERANCE` with a one-line reason. Use `MAX_IMAGE_SIDE` rather than 2^30 in `features_for`.

#### [should-fix] `zju.import_sequence` joins a CSV filename to the images folder without a containment check
- **Where:** `src/slam_it/eval/zju.py:~215`, `images / row[1]`.
- **Issue:** A `data.csv` row such as `../../x.png` or an absolute path makes the importer symlink to any file that exists. Everywhere else in this PR the code is careful about names that leave their folder (the Stray zip). Datasets are trusted input, so this is a consistency and hygiene point rather than a vulnerability.
- **Suggested fix:** Refuse a `row[1]` that is absolute, contains `..`, or has more than one path part, and name the row in the message.

#### [should-fix] `measure` calls `budgets.judge` under a blanket `contextlib.suppress`
- **Where:** `scripts/measure_testset.py:~130`.
- **Issue:** `suppress(InvalidInputError)` hides any invalid-input failure from `judge`, not only the "fewer than two frames" case the comment names. A real failure would show up as a silent `-` in the p95 column.
- **Suggested fix:** Check `len(run.frames) < 2` explicitly, or record the message in `unscored` the way the ATE path does.

#### [should-fix] `video.py` has grown to 937 lines with two different jobs
- **Where:** `src/slam_it/eval/video.py`.
- **Issue:** It now holds video decode and import, Stray zip safety, LiDAR map reading, CSV parsing and the CLI.
- **Suggested fix:** Not a blocker. Consider moving the Stray zip reading (`_stray_entries`, `_read_entry`, `_read_map`, `_write_depth`, the CSV parsers) into `eval/stray.py` in a later PR.

#### [note] `SHORT_SIDE = 480` is mirrored in `viewer/src/jobs.js`
- **Where:** `viewer/src/jobs.js` (`SHORT_SIDE`).
- **Issue:** It is commented as mirroring `slam_it.eval.video.SHORT_SIDE`, and nothing enforces the match. A test that reads the Python constant, or having the worker put it in the summary, would keep the two from drifting.

#### [note] The upload page says "A Stray Scanner recording runs RGB-D"
- **Where:** `viewer/upload.html`.
- **Issue:** A Stray zip without depth maps still imports and runs monocular, as the video.py docstring states. The sentence is slightly broader than the behaviour.

### Checked and fine
- **Input validation:** the new readers (ZJU, ADVIO, ARKitScenes, lidar, sensor_yaml) check shapes, finiteness, zero quaternions, monotonic times, size caps and frame numbering. Messages name the file, line and value.
- **Docstrings:** numpy-style with shapes, dtypes and frame conventions (`T_world_cam`, xyzw/wxyz, mm vs m). The module docstrings state where each reader sits and what it writes.
- **Error handling:** no bare except, no swallowed errors, no stray prints (the CLI prints are in `main`) and no TODOs.
- **sensor_yaml extraction:** EuRoC's private helpers moved there, so there is no second copy.
- **`RadTan.turned`:** the tangential swap `(p1, p2) -> (-p2, p1)` is documented with its reasoning.
- **`features_for`:** bounds, scaling and rounding are justified against ORB-SLAM's own settings, and the docstring gives worked values.
- **Stray depth import:** it is all-or-nothing per kept frame, with caps checked before and while reading and a scratch file always unlinked.
- **Tests:** present for every new module and for the worker's RGB-D path, the measure script (including malformed configs and failed sequences) and the viewer. Only `lidar.registered_depth` and `sensor_yaml` lack direct test files. `lidar` is covered in `test_arkitscenes.py`, including threshold and aspect errors.
- **Config and paths:** `measure_testset.py` keeps machine paths in an external TOML with a template, not in the repo.

### Architecture review

## Architecture review

**Scope:** `feature/2a-a1-measure-and-import`, `2aa96fe...a533b13` (7 commits, 32 files). I read all of the changed Python modules, the measure script and its template, the import-linter changes, the viewer changes and the CHANGELOG and design-doc edits. I also read their neighbours: `eval/__init__.py`, `eval/tum.py`, `eval/images.py`, `eval/budgets.py`, `orchestration/vo.py`, `tests/server/test_shared_names.py`, and the native TUM readers in `cpp/src/eval/tum.cpp` and `rust/slam-it-eval/src/tum.rs`. Design docs: `2026-10-02-slam-it-ladder.md` and `2026-10-12-phone-robust-vo.md`.

**Summary:** The slice fits the ladder well:
- The new readers are Python-only importers in `eval`, and they write the TUM folder that all three native readers already take, so no core code changed.
- Shared parts were moved into shared modules instead of being copied: `sensor_yaml` comes out of `euroc`, and `lidar` serves both Stray Scanner and ARKitScenes.
- The eval layer contract places each new module correctly.

Most findings are about places where this slice skips a mechanism the codebase already has:
- The check that holds the page's constants equal to Python's.
- The staged, recorded import that `video.py` set up.
- The rule that measuring a pipeline never needs the `video` extra (which the measure script gets around by importing the worker).

### Findings

#### [must-fix] The page's two new copies of Python's names are not held equal to Python's
- **Where:** `viewer/src/jobs.js:53` (`SHORT_SIDE = 480`), `viewer/src/jobs.js:56` (`MODES`); `tests/server/test_shared_names.py` (not changed by this PR)
- **Issue:** `SHORT_SIDE` copies `slam_it.eval.video.SHORT_SIDE`; its own JSDoc says so. `MODES` keys the worker's run modes (`"monocular"`, `"rgbd"`, from `slam_it.edge.options.MODES`). Neither is added to `test_shared_names.py`, which is where every other constant the page and Python both hold (`MAX_BYTES`, `MAX_SECONDS`, `STAGES`, `UPLOADS`, `INTRINSICS`, `RETENTION_DAYS`) is held equal.
- **Why it matters:** CLAUDE.md states the rule: "the constants both sides hold are held equal by `tests/server/test_shared_names.py`". If `video.SHORT_SIDE` changes, the status page warns at the wrong size and no test fails. A new mode (A2's map segments may bring summary changes) would show up raw in the mode row.
- **Evidence:** `git grep -n -E 'SHORT_SIDE|_constant\("MODES' HEAD -- tests server` finds nothing outside the page and its own JS test.
- **Suggested fix:** Add two asserts to `test_shared_names.py`:
  - `float(_constant("SHORT_SIDE")) == video.SHORT_SIDE`;
  - the keys of `MODES` equal the modes the worker can run, a subset of `options.MODES`.

#### [should-fix] The test-set measure depends on the upload worker's job summary, and so needs PyAV
- **Where:** `scripts/measure_testset.py:56` (`from slam_it.orchestration import vo, worker`), `:146` (`worker.summarize(run, times, "nominal" if nominal else "exact", size)`); `src/slam_it/orchestration/worker.py:487`
- **Issue:** The measure takes its tracking span (`tracked_from`, `tracked_to`, `ran_to`, `lost`) from `worker.summarize`, which is the JSON the status page shows.
  - Importing the worker brings in `eval.video` (`worker.py:91`), which imports `av` at module level (`video.py:114`). So measuring plain TUM folders requires the `video` extra.
  - The script also passes `"exact"` as `intrinsics`, which is not one of the documented values (`"stray-scanner"` or a `Chosen.source`).
- **Why it matters:**
  - The ladder doc says "The pipeline never needs the `video` extra: of orchestration, only the worker imports `eval.video`". Import-linter cannot see `scripts/`, so this script gets around that rule unchecked.
  - It ties 2A's yardstick to the demo's UI contract. A2 §4 rewrites this summary into per-map segments, so the measure would change whenever the status page does.
- **Evidence:** I imported each target with `av` blocked (probe at `<scratchpad>/no_av_probe.py`):
  ```
  scripts/measure_testset.py: needs PyAV (No module named 'av' (blocked by probe))
  scripts/run_vo.py: imports without PyAV
  slam_it.orchestration.vo: imports without PyAV
  ```
- **Suggested fix:**
  - Move the tracking span into `slam_it.eval.runs`, e.g. `runs.tracking(run, times)` returning a small dataclass. `eval` may import `edge` for `State.LOST`.
  - Have `worker.summarize` and the measure both build on it, and drop the measure's import of the worker.
  - A2's segments then change in one place.

#### [should-fix] The new importers drop the import contract `video.py` set up: a staged write, and a record of what was imported
- **Where:** `src/slam_it/eval/arkitscenes.py:159-182`, `src/slam_it/eval/zju.py:219-228`, `src/slam_it/eval/advio.py:244-245`; the contract is in `src/slam_it/eval/video.py:260` (`_staged`)
- **Issue:** All four importers take the same "new or empty folder" and write the same TUM folder, but they behave differently on failure:
  - `video.py` stages its writes beside the target and moves the folder into place only when it is whole, so a failed import leaves nothing behind.
  - **ARKitScenes** writes straight into `out` and decodes depth maps inside its write loop.
  - **ZJU** also writes straight into `out`.
  - **ADVIO** publishes the video import's folder first, then writes `groundtruth.txt` into it after publication. Its `source.json` says `kind: "video"`, source `"given"`, with no mention of ADVIO.

  What gets recorded also differs: ARKitScenes' `min_confidence` is kept nowhere in its output (`camera.txt`'s note at `:180` names only the source), whereas Stray's `source.json` records `depth.min_confidence`. A secondary point: ARKitScenes passes `registered_depth`'s error through without naming the file, while the Stray path wraps it with the frame.
- **Why it matters:**
  - A failed ARKitScenes import leaves a half-written folder, and rerunning the import refuses it.
  - The design measures medium confidence beside high (2A, "Depth"), yet an imported ARKitScenes folder cannot say which threshold made it, so a results table depends on folder names.
- **Evidence:** A synthetic 3-frame capture whose last depth map has the wrong aspect (probe at `<scratchpad>/probe_scripts/arkit_partial.py`):
  ```
  first import refused: size 64x48: expected the depth map's 32x20 aspect, at least its size
  left behind: ['depth', 'depth/3112.887000.png', 'depth/3112.920000.png', 'rgb', 'rgb/3112.887000.png', ...]
  rerun refused: .../out: expected a new or empty folder for the import
  ```
- **Suggested fix:**
  - Move `_staged` into a helper at eval's bottom layer (e.g. in `tum`, which already holds `write_list`/`write_camera`) and use it in all four importers.
  - Let `import_video` take the ground truth so ADVIO writes it inside the stage, or have ADVIO wrap the call in the stage itself.
  - Write a `source.json` for ZJU and ARKitScenes too: the importer, the source, and `depth.min_confidence` where depth is involved.

#### [should-fix] PNG header parsing is copied into two importers, and LiDAR maps pass through a scratch file, because `images` reads only from paths
- **Where:** `src/slam_it/eval/zju.py:122`, `src/slam_it/eval/video.py:610` (two hand-written IHDR parsers); `video.py:617-624` and `:641` (each zip map is written to `.map.png` and decoded from there)
- **Issue:** `slam_it.eval.images` is the one module that reads image files (its docstring, and Pillow's contract). This PR adds two copies of a PNG-size sniffer outside it. It also writes two scratch files per frame to call `images.read_depth`/`read_gray`, which take paths only, even though `images` already imports `io` for JPEG.
- **Why it matters:** The checks on image files are now spread across three modules. The scratch round trip exists only because of a gap in the `images` interface.
- **Suggested fix:** Add `images.png_size(header: bytes)` and decoding from bytes (e.g. `read_depth`/`read_gray` accepting `bytes`, or `decode_*` variants) in `images`. Use them in `zju` and `video`.

#### [should-fix] The package map and the worker's docstring still describe the structure as it was before this PR
- **Where:**
  - `src/slam_it/eval/__init__.py` (not changed): it lists every eval module except `zju`, `advio`, `arkitscenes`, `lidar` and `sensor_yaml`, and calls `video` "importing a phone's recording" only.
  - `src/slam_it/orchestration/worker.py:17`: "run the C++ pipeline on it, monocular, recording features".
  - The ladder doc's package-layout row for `slam_it.eval` (line 367): it names only `video` and `lenses` as Python-only imports.
- **Issue:** Five new modules are missing from the package's index. The worker's description of a job contradicts `worker.py:455`. The design's package table does not say the dataset importers are Python-only and write TUM folders for the native readers.
- **Why it matters:** These are the maps that show a module's place in the ladder. They now describe the codebase as it stood at 0.3.7.0.
- **Suggested fix:** Change the docs (the code is right):
  - In this PR, update both docstrings.
  - Extend the ladder doc's `eval` row: "dataset imports to TUM format (`zju`, `advio`, `arkitscenes`; `lidar`, `sensor_yaml`), Python only".

#### [note] `features_for` lives in a module C++ and Rust mirror, but exists only in Python
- **Where:** `src/slam_it/edge/options.py:423`; the mirrors are `cpp/include/slam_it/edge/options.hpp` and `rust/slam-it-edge/src/options.rs`
- **Issue:** The design puts the feature count "where a run is set up … no core changes", and the code honours that. But the helper sits in the edge rung's settings module, whose native mirrors will not have it. Meanwhile the worker (`options_for`, `worker.py:467`) and the measure (`measure_testset.py:140-142`) each build `Options(n_features=features_for(...))` themselves.
- **Suggestion:** Either put one helper in `orchestration.vo` (where runs are set up) and use it from both, or keep it where it is and say in its docstring that it is a Python-only setup helper by decision. Both keep `vo.run`'s default unchanged, as the owner decided.

#### [note] `video.py` now carries Stray Scanner's zip format beside the decoder
- **Where:** `src/slam_it/eval/video.py` (682 → 937 lines; `_StrayEntries`, `_stray_map`, `_read_map`, `_write_depth`, the CSV and odometry readers, `import_stray`)
- **Issue:** ADVIO is now a separate importer layered on `video`. Stray Scanner's reader, with its new LiDAR maps, is the same kind of thing but still lives inside the decoder module.
- **Suggestion:** Backlog. Move it to `eval.stray` above `video` (in the same layer as `advio`). This pairs naturally with sharing the staging helper from the importer finding above.

#### [note] The measure reports p95 latency but not the frame period the design says to read it against
- **Where:** `scripts/measure_testset.py:174`; design `2026-10-12-phone-robust-vo.md` §3 ("the 95th-percentile latency against the frame period")
- **Issue:** `budgets.judge` returns `deadline` (the frame period) and `late`, but only `latency.p95` is kept.
- **Suggestion:** Record `deadline` (and `late`) in each result and in the table. Change the code, not the doc. The algorithm reviewer owns what the measure contains.

### Checked and fine
- Dependency direction: SLAM It! still never imports reifyscene. No rung imports `eval` or `orchestration`; `edge.options` gains nothing heavy (no PyTorch or GPU stack); `eval` and `orchestration` stay within their layers.
- Eval's internal order: `lidar` and `sensor_yaml` sit at the bottom with `images`/`timestamps`. `zju` and `arkitscenes` sit beside `video` and `budgets`, above `runs`. `advio` is alone above `video`. The layering commentary matches.
- PyAV stays in `eval.video`. `advio` reaches it only through `video`. The "only the worker imports the video import" contract covers `advio` too, through indirect imports.
- Shared components moved, not copied: EuRoC now uses `sensor_yaml`, and `lidar.registered_depth` serves both Stray and ARKitScenes.
- No core changes, as designed. Options that differ from the defaults reach the native programs as flags (`vo.py:181`). The native TUM readers accept any camera model by name (`cpp/src/eval/tum.cpp:169`, `rust/slam-it-eval/src/tum.rs:299`), so a `radtan` `camera.txt` needs nothing new. Depth written by Pillow goes through the C++ program in the worker's RGB-D test.
- The cross-check `FEATURES_BOUNDS[0] > min_init_features_depth` is tested, so the floor of 1,000 stays tied to the RGB-D start condition.
- Viewer: `jobs.js` stays free of the DOM and `status_view.js` only places text. The server treats the summary as opaque (4 KiB cap), and the new `mode` and `size` fields fit.
- Private data: the test-set config stays outside the repo, and the repo holds only `scripts/testset.template.toml`.
- The design-doc change (floor 1,000, with its reason) is recorded in the 2A doc. The VERSION and CHANGELOG step (0.3.7.0 → 0.3.8.0) matches the design's plan for A1.
- No new runtime dependency: `tomllib` and `csv` are in the standard library.

### Algorithm review

## Algorithm review

**Scope:** `feature/2a-a1-measure-and-import`, `2aa96fe...a533b13` (7 commits). I read in full: `src/slam_it/eval/{zju,advio,arkitscenes,lidar,sensor_yaml}.py`, the diffs to `video.py`, `lenses.py`, `images.py`, `euroc.py`, `edge/options.py` and `orchestration/worker.py`, `scripts/measure_testset.py`, and the tests for each. I checked each import's frame convention against the real public data in `datasets/{zju,advio,arkitscenes}` and the developer's `datasets/imported/advio-01`. I did not open `rig`, `phone` or `testset.toml`. The scratch scripts are in `/tmp/algo-review/`. The PR's import and lens tests pass (222 passed). The working tree is still clean at `a533b13`. One side effect: importing `slam_it` set off scikit-build's editable rebuild into the git-ignored `build/`.

**Summary:** The geometry is right. All three ground-truth conventions hold on real data:
- **ZJU:** `T_world_body · T_body_cam`.
- **ADVIO:** poses in the IMU's axes, moved by `inverse(T_cam_imu)`.
- **ARKitScenes:** world-to-camera axis-angle, inverted.

The radial-tangential quarter-turn rule, the scaling, and the nearest-neighbour LiDAR registration are also correct. There is one real bug: the measure script never scores an ARKitScenes sequence. ARKitScenes' reference is 10 Hz, and the RPE that `runs.evaluate` computes alongside the ATE cannot pair frames at that rate, so the ATE is thrown away. The synthetic fixture uses a 50 Hz reference, which hides this.

### Findings

#### [must-fix] The measure script never scores an ARKitScenes sequence: the RPE fails on the 10 Hz reference and takes the ATE with it
- **Where:** `scripts/measure_testset.py:169` (`scored = runs.evaluate(run, *truth)`), which reaches `src/slam_it/eval/runs.py:838-846`. Also `tests/eval/test_arkitscenes.py:81-83`: the fixture's trajectory is at 50 Hz.
- **Issue:** `lowres_wide.traj` is about 10 Hz. On disk, every capture has a third as many poses as frames: 925 for 2,822, 1,121 for 3,386, and so on for all ten. `runs.evaluate` always computes the RPE with `delta` = the run's frame period (33 ms) and a tolerance of ±16.5 ms. The frames that match a reference pose are about 100 ms apart, so no pair exists and `metrics.relative_error` raises `InvalidInputError`. `measure()` catches it as `unscored`, and the ATE it had already computed is lost.
- **Why it matters:** Design section 3's measure includes "ATE ... for ARKitScenes, against ARKit's poses". Every ARKitScenes row of the test set, which is the LiDAR/RGB-D path, will read `ate: null`. The failure is visible in `unscored`, but the deliverable does not do its job for one of the four datasets. The reference rate is 10 Hz on every capture, so this is not specific to one run.
- **Evidence:** On real capture 42444946, a perfect 30 Hz run (reference poses at the vga frame times) gives:
  ```
  $ conda run -n slam-it python -I /tmp/algo-review/ark_measure.py .../arkitscenes/42444946
  InvalidInputError: no pose has another 0.03299999999580905 s (+-0.016499999997904524 s) later
  with rpe_delta=0.1: ATE 0.0 matched 924 of 2802
  ```
  The PR's tests miss it because the fixture writes poses at 3112.88, .90, .92, ... (every 20 ms), denser than its frames.
- **Suggested fix:** Score the ATE independently of the RPE in `measure()`. Either call `metrics.associate` and `metrics.absolute_error` directly, or pass `rpe_delta` equal to the matched reference spacing. The alternative, or an addition, is to interpolate the reference to each frame in the import (see the note on the half-frame offset below). Add tests: an ARKitScenes fixture whose trajectory is sparser than its frames, and a measure test with such a reference.

#### [should-fix] ZJU's ground-truth extrinsic is identity in the test, so the `T_gt_body` inversion and its order are untested
- **Where:** `src/slam_it/eval/zju.py:168-180`; `tests/eval/test_zju.py:35-49` (`q: [0, 0, 0, 1]`, `p: [0, 0, 0]`).
- **Issue:** `poses.append(pose @ gt_body)` with `gt_body = inverse(T_body_gt)` is only exercised with `T_body_gt = I`. Each of these mistakes would still pass every test: `pose @ T_body_gt`, `gt_body @ pose`, or dropping the term.
- **Why it matters:** All 16 published sequences have an identity ground-truth extrinsic, so no current result is affected. But the composition the docstring promises (`T_world_gt T_gt_body T_body_cam`) is unproven. The camera extrinsic, by contrast, is well tested: its non-zero `p` catches an inversion.
- **Evidence:** `GROUND_TRUTH` in `tests/eval/test_zju.py:35-49`. Every `datasets/zju/*/groundtruth/sensor.yaml` has `q: [0, 0, 0, 1]`, `p: [0, 0, 0]`.
- **Suggested fix:** Add a test case with a non-trivial ground-truth extrinsic (a rotation plus a translation) and assert `T_world_gt · inv(T_body_gt) · T_body_cam`.

#### [note] ARKitScenes' reference sits half a frame from every frame it is paired with
- **Where:** `src/slam_it/eval/arkitscenes.py:98-128`.
- **Issue:** `lowres_wide.traj` is stamped between the 30 Hz `vga_wide` frames. Over the matched pairs, |Δt| has a median of 16.2 ms and a 95th percentile of 17.0 ms, close to the 20 ms tolerance. At the captures' speeds (median 0.15-0.23 m/s, 95th percentile 0.31-0.63 m/s, median 12-21°/s) that is a systematic error of about 3-10 mm and up to about 0.4°, and only a third of the tracked frames are scored.
- **Suggested fix:** Optionally, interpolate the pose at each vga frame time in the import: linear for the position, slerp for the rotation, only between poses at most about 0.15 s apart. Say so in the docstring. This also fixes the must-fix above. Small next to centimetre-level VO error, so it is a note.

#### [note] ARKitScenes' per-frame focal length varies by ±2%; the median hides that
- **Where:** `src/slam_it/eval/arkitscenes.py:94`.
- **Issue:** The docstring says the per-frame intrinsics "vary slightly". Measured `fx` across a capture runs from 522.2 to 545.0 (median 532.3) on 42444946, and from 522.7 to 545.6 on 42446016. 96-98% of frames are within 1% of the median, and the largest frame-to-frame jump is 1.3-1.5%. With a fixed focal length, focus breathing reads as a small zoom.
- **Suggested fix:** Put the measured spread in the docstring or the results note. Newer Stray Scanner versions also write per-frame `fx, fy, cx, cy` in `odometry.csv`, and their `camera_matrix.csv` is "from the final recorded frame" ([format.md](https://github.com/strayrobots/scanner/blob/main/docs/format.md)). That is the same limitation in the existing Stray import, for the backlog.

#### [note] ADVIO's iphone-01 calibration (sequences 18-19) looks wrong beyond its `cy`
- **Where:** `src/slam_it/eval/advio.py:23, 59`.
- **Issue:** `iphone-01.yaml` has `p2 = 0.0473`, ten to fifty times the other batches' tangential terms. At full resolution its tangential part moves edge pixels by up to **53.9 px**. The other three batches move them by 1.1, 1.1 and 2.5 px. The README's table has the same values ([calibration README](https://github.com/AaltoVision/ADVIO/blob/master/calibration/README.md)). An iPhone lens decentred that much is implausible. Undistortion still converges over the whole frame in all three backends.
- **Suggested fix:** The module already flags the `cy` mismatch. Also flag `p2`, and check 18-19 before they join the test set (they are not downloaded yet).

#### [note] `--camera nominal` keeps the true `bf`, so an RGB-D run's virtual baseline moves with the focal guess
- **Where:** `scripts/measure_testset.py:118-130`.
- **Issue:** The pipeline uses `baseline = bf / fx` (`edge/frames.py:52`) and a close depth of `close_depth × baseline` (`edge/mapping.py:254`). With the true `bf` and a guessed `fx`, the close threshold scales by `fx_true / fx_guess`. With the design doc's 23% error that is about 2.6 m instead of 3.2 m. "What a guessed lens costs" then also includes a different near/far split.
- **Suggested fix:** Use `bf = lidar.BASELINE * guess.fx`, or `own.bf * guess.fx / own.fx`, or document the effect.

#### [note] ADVIO's `gyro.csv` runs about 0.3 s behind its frames' clock (milestone 6)
- **Issue:** The ground truth's rotation rate matches `arkit.csv`, which is stamped like `frames.csv`, at a lag of 0.00 s (r ≥ 0.992 on advio-01, 05, 09, 13 and 15). It matches `gyro.csv` only at a lag of 0.28-0.31 s. ADVIO's README says "All time stamps are synchronized". This does not affect A1, since no IMU is used. It will matter when milestone 6 uses ADVIO's IMU.
- **Evidence:** `/tmp/algo-review/advio_lag.py` gives `rot speed: GT(t) best matches gyro(t-0.28) corr 0.992` (advio-01) and `t-0.31` (advio-15).

#### [note] State of the art: a stronger reference for the LiDAR path
- ARKitScenes' poses are ARKit's own visual-inertial estimate, and the PR correctly calls them a reference. ScanNet++ has iPhone 13 Pro RGB (1920x1440) and LiDAR (256x192) streams. Its iPhone poses come from COLMAP, registered to a sub-millimetre laser scan: an independent reference for the same sensor path. Yeshwanth, Liu, Nießner and Dai, "ScanNet++: A High-Fidelity Dataset of 3D Indoor Scenes", ICCV 2023, [arXiv:2308.11417](https://arxiv.org/abs/2308.11417). This is context for a later test-set revision, not a defect.

### Checked and fine
- **ZJU conventions on real data:** xyzw extrinsic, `T_body_cam` as "sensor to body", shared clock. Two-view rotations from ORB features plus the essential matrix, against the imported ground truth, on B0:
  - median error 1.7°, against 16.7° if `q` were read as wxyz and 19.7° without the extrinsic;
  - the best time offset is 0 (1.14°, against 1.24° at ±15 ms and 1.95° at +60 ms);
  - A0 agrees (1.93° against 4.8°).
- **ADVIO ground truth in the IMU's axes:** this is the PR's observation, and it holds. Relative rotations over 1.5 s against the integrated gyro:
  - identity mapping: 0.84-0.97° median;
  - camera axes `diag(1,-1,-1)`: 6.4-24°;
  - ARKit landscape axes `Rz(±90)`: 6.4-18°;
  - tested on advio-01, 13 and 15 (advio-07 turns too little to discriminate).
- **ADVIO end to end, on the imported advio-01:**
  - two-view rotations against `groundtruth.txt`: median error 0.26° at zero offset, against 3.78° if the poses had not been moved by `T_cam_imu`;
  - estimated/true rotation-angle ratio 1.018, so the published focal length is within about 2%. That partly answers the design doc's "published calibration, to be checked".
  - `camera.txt` matches `upright.turned(1).scaled(...).turned(3)` exactly.
- **ADVIO video:** `frames.csv` has as many rows as the video has frames (15,589 and 9,099). The declared rotation of -90 gives 3 turns, and a decoded frame turned that way is upright (checked by eye). `quaternion wxyz → xyzw` roll; Kalibr's `T_cam_imu` is IMU to camera, so `T_world_imu · T_cam_imu⁻¹` is right. The batch-to-file mapping matches the README. The largest video (173 MB) is under the 500 MB cap.
- **ARKitScenes poses and registration:** Depth reprojection between frames about 0.5 s apart, using the imported poses, median intrinsics and high-confidence depth registered to 640x480. Median relative depth disagreement:
  - as imported: 0.2-0.4% (7 of 8 pairs; one 1.3%, one 7.4%);
  - poses not inverted: 2-44%;
  - in ARKit's y-up axes: 1-48%.

  This confirms the inversion, the CV axes, and LiDAR covering the same field of view as `vga_wide`. Depth pairs to colour by nearest time within 20 ms; at 60 Hz the nearest is at most about 8 ms away, usually exact.
- **RadTan quarter turn:** derived by hand. Substituting `(x, y) = (-y', x')` into Brown-Conrady gives `p1' = -p2`, `p2' = p1`, with the radial terms unchanged, as coded. `test_a_radial_tangential_lens_turns_with_its_image` checks projection after 1-4 turns to 1e-9.
- **RadTan scaling:** the distortion is unchanged, and the pixel-centre rule `(c+0.5)s-0.5` holds under the non-uniform 1280x720 → 854x480 scale.
- **Undistortion:** Newton converges over the full 480x854 and 854x480 frames for all four ADVIO calibrations in Python, C++ and Rust; projecting back round-trips to ≤ 8e-13 px.
- **`registered_depth`:** colour pixel `c` takes depth index `floor((c+0.5)·w/W)`, the pixel-centre-correct nearest neighbour. The aspect is checked, confidence is a `>=` threshold, and zero means no depth.
- **Stray depth:** the maps are turned by the same `rot90` turns as the RGB frame. They are paired by decoded-frame index with the edit list ignored, which agrees with `odometry.csv`. The units are millimetres, and confidence is 0/1/2, as in Stray's [format.md](https://github.com/strayrobots/scanner/blob/main/docs/format.md). A kept frame without its maps is refused, and the failure is visible.
- **`BASELINE = 0.08`:** ORB-SLAM2's `TUM1.yaml` has `Camera.bf: 40` at `fx = 517.3`, a baseline of 0.077 m, and `ThDepth: 40`, giving a 3.2 m close depth here, inside the LiDAR's range.
- **`features_for`:** the anchors match ORB-SLAM2's settings: 1,000 at 640x480 (TUM), 1,200 at 752x480 (EuRoC), 2,000 at 1241x376 (KITTI), with the same FAST thresholds and pyramid. The 1,000 floor exceeds `min_init_features_depth` (500). The tests cover both bounds and both orientations.
- **ATE alignment:** Sim(3) for monocular, since there is no metric scale. SE(3) for RGB-D: LiDAR and ARKit are both metric.
- **Thinning 60 → 30 fps on the dataset clock:** `MIN_SPACING` is 25 ms. The clock must time every decoded frame exactly once, or the import is refused.
- **`sensor_yaml`:** splitting on commas and whitespace reads EuRoC's multi-line `T_BS` exactly as before.
- **Seeded randomness and test oracles:** randomised tests use seeded generators. ADVIO's test builds its ground truth from known camera poses through a non-trivial `T_cam_imu`, and ARKitScenes' test checks the inversion against a known rotation and translation.

Sources:
- [ADVIO repository and README](https://github.com/AaltoVision/ADVIO)
- [ADVIO calibration README](https://github.com/AaltoVision/ADVIO/blob/master/calibration/README.md)
- [Stray Scanner data format](https://github.com/strayrobots/scanner/blob/main/docs/format.md)
- [ARKitScenes repository (raw-data loader, `TrajStringToMatrix`)](https://github.com/apple/ARKitScenes)
- [ORB-SLAM2 settings files (TUM1/EuRoC/KITTI yaml)](https://github.com/raulmur/ORB_SLAM2/tree/master/Examples)
- [ScanNet++, ICCV 2023 (arXiv:2308.11417)](https://arxiv.org/abs/2308.11417)

## Run 2 -- 2026-10-13

| | |
|---|---|
| **Branch** | `feature/2a-a1-measure-and-import` |
| **Range** | `a533b13...c832973` (the fixes of run 1's findings) |
| **Reviewers** | architecture (opus), algorithm (opus): the two whose areas the fixes changed most; with the security pass's run 2 |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed on c832973 (`scripts/check.sh`, exit 0; pytest 3687 passed) |

Every run-1 finding resolved as fixed holds, by both reviewers. One run-1 diagnosis did not:
L1's premise (the measure could not score ARKitScenes) was wrong, as L9 explains; its code is
correct and kept. L3, waived in run 1, is reopened and fixed by L9's route (b).

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A9 | should-fix | architecture | `advio.py:253-256`; `video.py:474`; `tum.py:350` | ADVIO's `source.json` names only its importer, not the sequence or its calibration; the record's shape is defined nowhere | fix (`import_video` takes the dataset's record; `write_source` names the common keys, refuses no `kind`) | fixed (26f9d6d) -- decided by evan-wes, 2026-10-13 |
| A10 | note | architecture | `bundles.py:114-127`; `run_vo.py:88` | The bake aligns through `runs.evaluate`, so a reference without RPE pairs leaves a run unaligned | defer (A2's per-map scoring; the ladder's open items) | deferred (the ladder's open items, 26f9d6d) -- decided by evan-wes, 2026-10-13 |
| A11 | note | architecture | `runs.py:848` | `runs.tracking` raises a bare `KeyError` on a run that reports no `state` | fix (`.get`: a run that reports no state never reports lost) | fixed (26f9d6d) -- decided by evan-wes, 2026-10-13 |
| A12 | note | architecture | `worker.py:128, 464`; `test_worker.py:184, 203` | `RUN_MODES` is held to the page, but not to the modes a job runs in | fix (the worker's tests assert the mode is in `RUN_MODES`) | fixed (26f9d6d) -- decided by evan-wes, 2026-10-13 |
| A13 | note | architecture | `images.py:15-18, 52, 181` | `read_rgb` takes neither bytes nor a bound; the public alias `Source` clashes with `vo.Source` | fix (`read_rgb` as the others; the alias private) | fixed (73a0195) -- decided by evan-wes, 2026-10-13 |
| A14 | note | architecture | `tum.py:42-44, 313`; `bundles.py:386-394`; `lenses.py:967-987` | `tum`'s docs do not say it owns staging and records; the mechanism is written out twice more | fix the docs now; defer one neutral helper (the ladder's open items) | fixed in part (26f9d6d: `tum`'s docstring and the package map); deferred (the ladder's open items, 26f9d6d): one staging helper -- decided by evan-wes, 2026-10-13 |
| L9 | must-fix | algorithm | `tum.py:517`; `run_vo.py:88`; `bundles.py:125`; `measure_testset.py:174` | ARKitScenes' 10 Hz poses each serve two frames, so its RPE measures the camera's motion; round one's L1 premise was wrong | route (b): interpolate ARKitScenes' reference at the frames' times (reopens L3) | fixed (5871936: `arkitscenes.at_frames`, within 0.15 s; 753b738's wrong claim corrected in 5871936's message); the root pairing deferred (the ladder's open items, 26f9d6d) -- decided by evan-wes, 2026-10-13 |
| L10 | should-fix | algorithm | `runs.py:43-44, 908-910`; `measure_testset.py:26-28`; `CHANGELOG.md:49-50`; tests | Docs and tests say ARKitScenes matches every third frame with no RPE pairs; on the imported folder it was two in three, and the RPE paired | fix (follow L9's route) | fixed (5871936) -- decided by evan-wes, 2026-10-13 |
| L11 | note | algorithm | `advio.py:23-27` | A two-view check on advio-18 and -19 favours dropping iphone-01's `p2`, and cannot judge its `cy` | the results note says so | fixed (the results note) -- decided by evan-wes, 2026-10-13 |

### Architecture review

## Architecture review

**Scope:** Run 2 of the panel on `feature/2a-a1-measure-and-import`, covering the fix range `a533b13...c832973`: 84508ce, 753b738, 9ae0734 and c832973, 26 files.
- I read the whole diff.
- I also read the neighbours the fixes touch: `orchestration/bundles.py` (`_score`, `bake`), `scripts/run_vo.py`, `eval/budgets.py` (how it checks diagnostic keys), `eval/lenses.py` (`fetch`), `viewer/src/jobs.js`, `server/src/slam_it_server/app.py` (`REASON_LENGTH`), the eval and orchestration import-linter contracts in `pyproject.toml`, the 2A design doc's sections 3-6, and the first run's report and security pass.
- I ran the tests that cover the fixed findings: `test_shared_names`, `test_tum`, `test_runs`, `test_images`, `test_zju`, `test_arkitscenes`, the new `test_vo` case and the measure's no-PyAV test. 184 passed.
- I wrote two probes in the session scratchpad (`.../scratchpad/arch2/`), ran under a 4G memory cap. The working tree is unchanged, HEAD is still c832973, and only the two untracked review files are present.

**Summary:** Every first-run architecture finding marked "fixed" holds, and the fixes respect the ladder: no new cross-rung imports, no contract changes needed, no native code touched. There is one should-fix. ADVIO's `source.json` names only its importer, because `import_video` takes just a `kind` string, so A3's stated aim ("a results table need not rely on folder names") is met for ZJU and ARKitScenes but not ADVIO. The rest are notes:
- the bake's alignment still fails on ARKitScenes' sparse reference;
- `runs.tracking` has a latent `KeyError`;
- `RUN_MODES` is declared but not used by the code that picks the mode;
- the `images` API is uneven;
- the stage-and-move helper's new home in `tum` cannot serve the other two copies of the mechanism.

### First-run findings (A1-A8)
- **A1 holds.** `test_shared_names.py:35` holds `SHORT_SIDE` to `video.SHORT_SIDE`, and `:48-49` holds the page's `MODES` keys to `worker.RUN_MODES`, which must sit within `options.MODES`. A12 covers a remaining gap between `RUN_MODES` and the worker's behaviour.
- **A2 holds.**
  - `runs.tracking` and `runs.LOST` are in eval, and `LOST` is held to `State.LOST` by a test, as `FATES` is held to `Fate`.
  - The worker's `summarize` builds on `tracking`.
  - The measure imports only `vo`, `eval` and `edge.options`. A subprocess test (`tests/test_measure_testset.py`, `test_the_measure_never_needs_the_video_import`) proves that `av`, `eval.video` and the worker are never loaded.
  - A11 covers a latent error-contract gap in `tracking`.
- **A3 holds.**
  - `tum.staged` is used by all four imports: `video.py:515` and `:826`, `zju.py:218`, `arkitscenes.py:160`, and ADVIO through `import_video`.
  - ADVIO's ground truth is written inside the stage (`video.py:515-517`).
  - ZJU and ARKitScenes write records, ARKitScenes' including `min_confidence`.
  - An ARKitScenes map refusal names its file.
  - Tests check that a failed import leaves nothing behind, for every importer.
  - A9 covers ADVIO's record.
- **A4 holds.** `images` reads bytes with a size bound checked after Pillow has parsed the headers, and `zju` uses `images.size`. `git grep 'IHDR\|x89PNG\|\.map\.png\|scratch' -- src scripts` finds no hand-written PNG parsing and no scratch file left. A13 is a small API note.
- **A5 holds.** `eval/__init__.py` lists all five new modules, the worker's docstring describes RGB-D and features-by-size, and the ladder's `eval` row names the dataset imports as Python-only. A14 notes that `tum`'s own docstring does not yet mention what it now owns.
- **A6 holds.** `vo.options_for` (`vo.py:355`) is the one helper, used by the worker (`worker.py:466`) and the measure (`measure_testset.py:146`). `features_for`'s docstring says it is Python-only by design.
- **A7 is deferred**, as decided. It is recorded in the ladder's open items, under A1's panel.
- **A8 holds.** `deadline`, `late` and `unjudged` are recorded (`measure_testset.py:181-186`), and the table shows "p95 of the period ms".

### Findings

#### [should-fix] A9: ADVIO's `source.json` names only its importer; `import_video(kind=)` cannot carry the sequence it came from
- **Where:**
  - `src/slam_it/eval/advio.py:253-256` (`import_sequence`)
  - `src/slam_it/eval/video.py:474` (`kind: str = "video"`) and `:517`
  - `src/slam_it/eval/tum.py:350` (`write_source`: "at least `kind`", not checked)
  - for comparison: `zju.py:230`, `arkitscenes.py:191-198`
- **Issue:** The fix gives the three dataset importers three different record shapes:
  - ZJU writes `{"kind": "zju", "sequence": "B0", "depth": null}`.
  - ARKitScenes writes `{"kind": "arkitscenes", "capture": ..., "depth": {"min_confidence": ...}}`.
  - ADVIO goes through `import_video`, whose only hook is the `kind` string. Its record therefore names `"advio"` but not the sequence (`advio-18`) or the calibration file (`iphone-01.yaml`).

  `write_source` documents only "at least `kind`", so the record's shape is defined nowhere.
- **Why it matters:**
  - Commit 9ae0734 states A3's purpose: records "naming the sequence or capture … so a results table need not rely on folder names". For ADVIO, the folder name is still the only link to the sequence.
  - ADVIO's docstring (`advio.py:27`) now says "a result on those two sequences says which calibration it used". Today only the intrinsics' values in the record back that up, not the file.
  - `kind` is public API added in this range. Changing its shape now costs one signature; later it costs a migration of records.
- **Evidence:** `tests/eval/test_advio.py:121-123` asserts only `kind`, `intrinsics.source` and `distortion`. `tests/eval/test_zju.py` (last test) asserts `{"kind": "zju", "sequence": "B0", "depth": None}`, and `test_arkitscenes.py` asserts `"capture"`.
- **Suggested fix:**
  - Replace `kind: str` with one mapping of the dataset's identifying fields, `kind` among them, and merge it into `_write_source`'s record. ADVIO would pass `{"kind": "advio", "sequence": folder.name, "calibration": calibration_name(folder)}`.
  - Have `tum.write_source` name the common keys (`kind`, the source's name, `depth`) and refuse a record without `kind`.

#### [note] A10: The bake still ties the ATE alignment to the RPE, so an ARKitScenes run bakes unaligned
- **Where:** `src/slam_it/orchestration/bundles.py:114-127` (`_score` calls `runs.evaluate`); `scripts/run_vo.py:88` has the same pattern.
- **Issue:** L1 was fixed for the measure by adding `evaluate_absolute`. The viewer's bake still aligns through `evaluate`, so a reference too sparse for the RPE (ARKitScenes' 10 Hz poses) drops the alignment too: the run "stays in its own frame, unscored". `run_vo.py` prints "not scored" for the same reason.
- **Evidence:** I built the synthetic run from `test_the_ate_alone_scores_against_a_reference_too_sparse_for_the_rpe` (30 Hz frames, reference at 10 Hz). Probe `scratchpad/arch2/bake_sparse.py` gives:
  ```
  evaluate_absolute matched: 60
  bundles._score: None
  bundles._score, dense truth: Evaluation
  ```
- **Suggestion:** Defer to A2. Its section 5.2-5.3 reworks the bake's scoring per map anyway. There, align with `evaluate_absolute` and add the RPE only where pairs exist, with the header's `rpe_*` fields nullable. It is outside A1's goal, since no A1 deliverable bakes an ARKitScenes run.

#### [note] A11: `runs.tracking` reads `state` as if the run format required it
- **Where:** `src/slam_it/eval/runs.py:848` (`run.diagnostics[-1]["state"] == LOST`)
- **Issue:** The run format requires only `timestamp` per diagnostic (`runs.py:33-34`, and `write_run` checks only that). `state` is "what the pipeline wants to report". `tracking` is now public API in the cross-rung format module, and on a valid `Run` without `state` it raises a bare `KeyError`. The sibling `budgets.judge` raises `InvalidInputError` for a missing key (`budgets.py:371`).
- **Why it matters:** Today it is latent, because all three backends write `state` (`cpp/src/orchestration/vo.cpp:440`, `rust/slam-it-orchestration/src/vo.rs:595`). But a `KeyError` would escape the measure's per-sequence `except (SlamItError, FileNotFoundError)` (`measure_testset.py:248`) and abort the whole test set rather than fail one row.
- **Evidence:** Probe `scratchpad/arch2/tracking_state.py`, a `Run` whose diagnostics hold only timestamps, prints `KeyError 'state'`.
- **Suggestion:** Use `run.diagnostics[-1].get("state") == LOST`, which matches the format: a run that reports no state never reports lost.

#### [note] A12: `RUN_MODES` is what the page is held to, but not what the worker chooses from
- **Where:** `src/slam_it/orchestration/worker.py:128` (`RUN_MODES`) and `:464` (`mode = "rgbd" if ... else "monocular"`, string literals); `tests/orchestration/test_worker.py:184, 203`, which compare against literals.
- **Issue:** The link from the page's `MODES` to `RUN_MODES` is tested. The link from `RUN_MODES` to the modes a job actually runs in is not: a new mode in `_work` would pass every test with `RUN_MODES` unchanged.
- **Suggestion:** Have the two worker tests assert `serving.summary["mode"] in worker.RUN_MODES`, or have `_work` pick from `RUN_MODES`.

#### [note] A13: The `images` API added here is uneven
- **Where:** `src/slam_it/eval/images.py:15-18` (the module docstring: "Each reader takes a file, or a file's bytes … and optionally the longest side it accepts"), `:181` (`read_rgb(path: str | Path)`), `:52` (`Source = str | Path | bytes`)
- **Issue:**
  - `read_rgb` takes neither bytes nor `max_side`, so the docstring overclaims.
  - The public alias `images.Source` shares its name with `slam_it.orchestration.vo.Source` (`vo.py:488`), a sequence's frame source.
- **Suggestion:**
  - Give `read_rgb` the same signature (one line: `_rgb(_open8(path, max_side), path)`), or narrow the docstring to the two readers.
  - Make the alias `_Source`, or name it `ImageSource`.

#### [note] A14: The stage-and-move helper's new home serves the importers only; `tum`'s own map does not say it owns it
- **Where:**
  - `src/slam_it/eval/tum.py:313` (`staged`: the `.importing` suffix, the message "for the import")
  - `src/slam_it/orchestration/bundles.py:386-394` (an inline copy, `.baking`)
  - `src/slam_it/eval/lenses.py:967-987` (an inline copy, `.fetching`)
  - `tum.py:42-44` (the "Writing one" paragraph) and the `tum` bullet in `eval/__init__.py`
- **Issue:**
  - Moving the importers' copy into `tum` was the right step, and it was my first-run suggestion. But the same mechanism is still written out twice more.
  - `lenses` sits below `tum` in eval's layer contract, so it cannot use `staged`.
  - `staged`'s wording is import-specific, so the bake cannot adopt it as is. Both copies predate this PR.
  - Separately, `tum`'s module docstring and the package map still describe it as the TUM reader plus `write_list`/`write_camera`, not the staging and `source.json` that every import now shares.
- **Suggestion:**
  - Backlog: one neutral helper at eval's bottom layer (or in `tum`, parameterised by suffix and noun) that all three use.
  - Now: add a sentence on `staged` and `write_source` to `tum`'s "Writing one" paragraph and to its bullet in `eval/__init__.py`.

### Checked and fine
- Layering:
  - `tum.staged` and `write_source` (eval's fourth layer) are used by `video`, `zju` and `arkitscenes` above them, and by `advio` only through `video`.
  - `runs.tracking` (the third layer) is used by `worker` and the measure.
  - `vo.options_for` sits in orchestration's bottom layer and imports only `edge.options`.
  - `pyproject.toml` is unchanged, and no new contract is needed.
- eval still does not import edge: `runs.LOST` is a string held equal to `State.LOST` by a test, following the `FATES` precedent.
- No C++ or Rust files changed in the range. The native readers still read only Python-written PNGs, and no parity fixture is affected.
- `scripts/` is outside import-linter, and the subprocess test is the right mechanism to keep the measure off PyAV and the worker.
- `worker.REASON_LENGTH` is held equal to `app.REASON_LENGTH`, which is the server's own cut (`app.py:691`). The worker applies its cut in `Client.failed` before posting.
- `images`' bomb-warning guard (`warnings.catch_warnings`) is process-global on Python 3.11. No code in `slam_it` reads images on threads (`git grep` finds no threading), so it holds today.
- `tum.staged` keeps relative links right because the stage is a sibling of `out`. It refuses a used folder, removes itself on any failure, and each importer has a test that a failure leaves nothing.
- `runs.evaluate` now builds on the shared `_matched`. Pairing is written once, and what `evaluate` returns is unchanged.
- `features_for`'s bound now uses `MAX_IMAGE_SIDE`. Its docstring's reference to `orchestration.vo.options_for` is a doc cross-reference, not an import upward.
- The deferred items (H6/A7, L4, L7, L8) are recorded in the ladder's open items with links to both reports.
- The 0.3.8.0 CHANGELOG entry describes the added API accurately: staging and records, `evaluate_absolute` and `tracking`, images from bytes, `options_for`, the reason cut.

### Algorithm review

## Algorithm review

**Scope:** `feature/2a-a1-measure-and-import`, fix range `a533b13...c832973` (84508ce, 753b738, 9ae0734, c832973). I read the full diff of `eval/{runs,tum,images,video,zju,advio,arkitscenes}.py`, `orchestration/{vo,worker}.py`, `edge/options.py`, `scripts/measure_testset.py`, their tests, the ladder doc and the CHANGELOG. I also re-read `tum.read_sequence`, `runs.ground_truth`, `metrics.{associate,relative_error}`, and the two other scorers, `scripts/run_vo.py` and `orchestration/bundles.py`. The touched test files pass: 462 passed. I ran probes against the public datasets (`arkitscenes`, `advio`, `imported/{arkitscenes-*,advio-18,advio-19}`). I did not open `rig`, `phone` or `testset.toml`. The probes are in `/tmp/algo-review2/`. HEAD is unchanged and the tree is untouched. One side effect, as in the first run: importing `slam_it` ran scikit-build's editable rebuild check into the git-ignored `build/`, which reported "no work to do".

**Summary:** Every fix does what its resolution says. The refactor is exact: `_matched` and `runs.tracking` give bit-identical results to a533b13. However, re-checking L1 on the real data path showed that **my first-run L1 was wrong about the mechanism**:
- The measure does not read ARKitScenes' raw 10 Hz trajectory. It reads the imported folder through `runs.ground_truth` and then `tum.read_sequence`, which gives each pose to both frames on either side of it.
- So the old measure did score ARKitScenes, and `evaluate_absolute` changes nothing on that path.
- The same mechanism makes every RPE on an ARKitScenes folder measure the camera's own motion instead of the tracking error. `run_vo.py` prints that RPE and the viewer shows it (L9).
- The fix's docstrings describe the raw-trajectory behaviour, not this path (L10).

**First run's findings:**
- **L1: the code holds, but the premise was wrong.** `evaluate_absolute` is correct:
  - It gives an ATE of 4e-16 m (924 matched) on 42444946 and 6e-16 m (2,267 matched) on 42446016, with a perfect run against the raw reference.
  - That ATE equals `evaluate(..., rpe_delta=0.1)`'s.
  - The new tests do separate it from the old code, because their sparse reference sits on frame times.

  The premise fails on the measure's real path. a533b13's `runs.evaluate`, given a real C++ run of 42445021 and the truth the measure reads (`runs.ground_truth("tum", folder)`), scores with no error: ATE 0.2027 m, 301 matched, 137 RPE pairs. My first-run probe paired against `arkitscenes.read_trajectory` directly, which the measure never does. I'm sorry for the misdiagnosis. Its consequences are L9 and L10.
- **L2: holds.** The new test separates every wrong composition I listed. Their distances from the asserted pose are: not inverted 1.92, wrong side 1.13, `T_body_gt @ pose` 1.90, term dropped 0.96, camera before ground truth 1.13. The tolerance is 1e-12. The quaternion `[0.6, 0, 0, 0.8]` is exactly unit.
- **L3: waived.** No change. L9's route (b) would reopen it.
- **L4: holds.** The docstring's figures reproduce exactly:
  - 42444946: 522.2-545.0, median 532.3, 95.7% of frames within 1%.
  - 42446016: 522.7-545.6, median 534.7, 98.1% within 1%.
  - All ten captures: 93.7-99.3% within 1%.
  - The ladder doc's "about 4%" is right: 22.8 / 532.3 = 4.3%.
  - "Follows the lens's focus" fits Apple's own forum answer on per-frame `ARCamera.intrinsics`.
- **L5: holds.** The largest tangential shift on a full 720x1280 frame of iphone-01 is 53.9 px sampled at distorted pixels (the image's real edge), and 47.5 px sampled every 4 px as undistorted points (the main session's way). "About 50" covers both. The other batches give 1.1, 1.1-1.2 and 2.5 px, so "one to three" is fair. Sequences 18 and 19 are now imported; see L11.
- **L6: holds.** On real 42445021 with `--camera nominal`:
  - `bf` is 43.746 = 0.08 × 546.831, so the baseline is kept.
  - The tracked span is identical to the exact camera's (509/1,449, lost at 17.1 s); the ATE is 18.1 cm against 20.3 cm.
  - Everything that reads the baseline uses metres (`mapping.py:254` close depth, `:598` parallax, `:684` the keyframe baseline check), so none of it moves with the guess.
- **L7 and L8: deferred.** They are recorded in the ladder doc's open items, and the wording matches the first run's measurements (lag 0.28-0.31 s; ScanNet++, ICCV 2023).

### Findings

#### [must-fix] L9: ARKitScenes' reference reaches every scorer as per-frame copies, so its RPE measures the camera's motion, not the tracking error
- **Where:**
  - `src/slam_it/eval/tum.py:517`: `read_sequence`'s `nearest = timestamps.nearest(truth_times, stamps)` lets many frames take one pose and re-stamps that pose at each frame's time.
  - It is reached through `runs.ground_truth` by:
    - `scripts/run_vo.py:88`;
    - `src/slam_it/orchestration/bundles.py:125`, whose score the viewer shows at `viewer/src/main.js:52-53`;
    - `scripts/measure_testset.py:174`.
- **Issue:** The ARKitScenes import writes ARKit's 10 Hz poses to `groundtruth.txt`, and they sit half a frame (16-17 ms) between the 30 Hz frames.
  - `tum.read_sequence` gives each frame the nearest pose within 20 ms, so each pose goes to both frames around it. On 42445021, 477 poses become 614 frame-stamped "true" samples.
  - The RPE pairs frames one period (33 ms) apart, and every pair it finds is two frames holding the same pose. The true relative motion is therefore the identity, and the RPE reads the run's own 33 ms motion.
- **Why it matters:**
  - Every ARKitScenes run that `run_vo.py` scores, or that is baked for the viewer, reports an RPE that is mostly camera motion. ARKitScenes is the only reference for the LiDAR/RGB-D path.
  - `bundles._score` and `run_vo` still throw the ATE alignment away whenever the RPE cannot pair, which is L1's rationale, fixed only in the measure. So pairing the reference correctly would leave ARKitScenes bundles unaligned unless they fall back too.
- **Evidence:** A perfect tracker (the reference interpolated at every frame) scored the way `run_vo` and the bundles score it (`/tmp/algo-review2/ark_perfect_run.py`). Every value should be 0:

  | Capture | RPE pairs sharing one pose | RPE translation RMSE | RPE rotation RMSE | Own frame-to-frame motion (RMS) |
  |---|---|---|---|---|
  | 42445021 | 137 of 137 | 10.2 mm | 0.54° | 8.0 mm |
  | 42444946 | 489 of 489 | 24.3 mm | 1.45° | 20.7 mm |
  | 42446016 | 1,162 of 1,162 | 5.8 mm | 0.46° | 6.3 mm |

  The ATE is barely affected. A perfect run scores 3.4, 11.3 and 2.4 mm on the measure's path, against 2.8, 9.7 and 2.1 mm paired one-to-one: this is L3's offset, now on two frames in three instead of one in three.
- **Suggested fix:** two routes; the owner picks.
  - **(a) Pair the reference one-to-one.** This is the rule of TUM's `associate.py`, and `metrics.associate` already implements it. Either `runs.ground_truth` returns `groundtruth.txt` as written, or `read_sequence` pairs with `metrics.associate`.
    - The RPE then has no pairs on ARKitScenes and refuses.
    - `run_vo` and `bundles._score` then fall back to `evaluate_absolute`, as the measure now does.
    - An RPE at the reference's own spacing (`rpe_delta=0.1`) is an option. Relative error over longer sub-trajectories is the standard recommendation anyway (Zhang & Scaramuzza, IROS 2018).
    - TUM's 100 Hz reference should give the same pairs, but check the golden records.
  - **(b) Interpolate the reference at the frame times in the import:** linear for the position, slerp for the rotation, as L3 first suggested. This reopens L3's waiver. In exchange, every frame gets a pose at its own time, the RPE is right, and the 3-11 mm floor goes.

#### [should-fix] L10: The fix's docs and tests say ARKitScenes scores every third frame with no RPE pairs; on the imported folder it scores two frames in three, and the RPE pairs
- **Where:** `src/slam_it/eval/runs.py:43-44` and `:908-910`, `scripts/measure_testset.py:26-28`, `CHANGELOG.md:49-50`, `tests/test_measure_testset.py:170`, `tests/eval/test_runs.py:286`.
- **Issue:** These say ARKitScenes' poses "match every third frame, and no two matched frames are a period apart". That is true of the raw trajectory paired one-to-one, but not of the path the measure takes (L9). The measure also reports `matched` 301 on 42445021, which counts 164 independent poses about twice.
- **Why it matters:** The results note will quote the matched counts and the reason the ATE is scored alone. As written, both misstate what was scored.
- **Evidence:** `/tmp/algo-review2/ark_truth_path.py`:
  ```
  runs.ground_truth: 614 stamps (frame times), 477 distinct poses
  OLD runs.evaluate on the measure's truth: ATE 0.2027 m matched 301; RPE pairs 137 -- scored, no error
  evaluate_absolute on the raw 10 Hz reference (one-to-one): ATE 0.2107 m matched 164
  ```
- **Suggested fix:** Follow L9's route. With (a) the text becomes true as written. With (b) it should say every frame is scored, and that `evaluate_absolute` is for a reference that is sparse and sits on frame times.

#### [note] L11: ADVIO 18-19, now imported: a two-view check favours dropping iphone-01's `p2`, and cannot judge its `cy`
- **Where:** `src/slam_it/eval/advio.py:23-27` (the flag L5 added).
- **Issue:** I took the same ORB matches 15 frames apart and the same seeded RANSAC, undistorted them with variants of the calibration, and compared the rotations against `groundtruth.txt`:
  - advio-18: median error 0.63° as published, against 0.45° with `p2 = 0`.
  - advio-19: 0.47° against 0.43°.
  - Paired over 116 and 144 pairs, `p2 = 0` is better on 59% of pairs, by a median of 0.08° and 0.035°. `p2 = 0.00473` (a dropped digit) scores the same as `p2 = 0`.
  - Swapping `cy` for the README's 639.31 changes nothing (median difference 0.000° and 0.001°). Two-view rotation is nearly blind to the principal point, so this neither confirms nor clears the `cy` mismatch.
  - The angle ratios stay at 1.00-1.03, so `fx` is not in question.
- **Why it matters:** This is weak but consistent evidence that the published `p2` is wrong. Worth one line in the results note beside "which calibration it used". A self-calibration (a BA with `p2` and `cy` free) would settle it.
- **Evidence:** `/tmp/algo-review2/advio_calib_check2.py imported/advio-18 15 150` (and the same for advio-19).

### Checked and fine
- **The `_matched` refactor is exact.** a533b13's `evaluate` against c832973's over 400 seeded cases (noisy, sparse, jittered, monocular and RGB-D, several `rpe_delta` and `max_difference` values): 215 scored bit-identically, including the sign of zeros, and 185 refused with identical messages. Malformed inputs are refused identically too. (`refactor_parity.py`)
- **`runs.tracking` keeps the worker's summary identical:** the dicts and their value types match a533b13's `summarize` over 300 seeded runs. `runs.LOST == State.LOST` is tested, and `State` is a `StrEnum`. `tracking` does not validate `times`, but both callers pass the source's own times.
- **The measure end to end on real ARKitScenes 42445021 (C++):** ATE 20.3 cm (301 matched); p95 8 of 33 ms; `deadline` is the frame period from `rgb.txt`'s stamps; `late` 0; `unjudged` null.
- **`vo.options_for`:** 854x480 gives 1,334 features and 320x240 gives ORB-SLAM's defaults. `features_for`'s bound is now `MAX_IMAGE_SIDE` (2^20), which only changes validation.
- **Staged imports:** the stage is a sibling of the folder at the same depth, so the relative frame links stay right once it is moved. ADVIO's ground truth is written inside the stage, its poses to 1e-9 in the test. ZJU, ARKitScenes and ADVIO all leave nothing behind on failure, as tested. The ZJU frame-name check refuses absolute paths, `..` and sub-folders.
- **`images` from bytes:** the size is checked against the bound after Pillow has parsed every header chunk and before any pixel is decoded. Errors raised during `load()` reach the generator's handler and become `InvalidInputError`. Depth decoding is unchanged.
- **`MAX_SECONDS`:** advio-22's last `frames.csv` stamp is 385.9 s, the longest of the 01-22 sequences on disk.

Sources:
- [TUM RGB-D benchmark tools (associate.py, evaluate_ate.py, evaluate_rpe.py)](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools); Sturm et al., IROS 2012
- [Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf)
- [Apple Developer Forums: ARCamera.intrinsics changes between frames](https://developer.apple.com/forums/thread/827789)
- [ARCamera.intrinsics documentation](https://developer.apple.com/documentation/arkit/arcamera/intrinsics)
- [ADVIO calibration README](https://github.com/AaltoVision/ADVIO/blob/master/calibration/README.md)
