# Review panel: `feature/front-end`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-05

| | |
|---|---|
| **Branch** | `feature/front-end` |
| **Range** | `59bccd8d6ce7f1b6ed9017ee45c75bb577c146cd...5b4f709ee617b4bc24a85fc50c465da8b5e963bf` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`) |

### Findings and resolutions

Duplicates across reviewers are cross-referenced ("same as") and resolved together.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `stereo.py:579`, `stereo.cpp:373`, `stereo.rs:513` | `stereo.match` segfaults (C++) and panics (Rust) on finite but huge keypoint `xy` (same as L1) | fix | fixed (a3a54b9): every core checks keypoints against their pyramid; overflow-free bounds -- decided by evan-wes, 2026-10-05 |
| H2 | must-fix | hygiene | `orb.py:233`, `matching.py:69`, `stereo.py:132` | `Options` dataclasses check ranges, not types: floats and bools reach the backends and fail three different ways | fix | fixed (b536911): `arrays.as_count`/`as_flag`/`as_number` in every `Options` -- decided by evan-wes, 2026-10-05 |
| H3 | must-fix | hygiene | `metrics.py:195`, `runs.py:349` | `associate` and `evaluate` never check that times ascend; unsorted input silently mis-pairs | fix | fixed (35141c1): `associate` and `evaluate` check order (`eval.timing.as_times`) -- decided by evan-wes, 2026-10-05 |
| H4 | must-fix | hygiene | `runs.py:266` | `read_run` raises raw `JSONDecodeError`/`KeyError`/`AttributeError` instead of `InvalidInputError` | fix | fixed (35141c1): `read_run` raises `InvalidInputError`, naming the file and key -- decided by evan-wes, 2026-10-05 |
| H5 | should-fix | hygiene | `metrics.py:191` | `associate` claims TUM's `associate.py` but is not its global greedy (same as L6) | fix | fixed (35141c1): TUM's closest-first greedy, exclusive tolerance -- decided by evan-wes, 2026-10-05 |
| H6 | should-fix | hygiene | `tum.py:161`, `euroc.py:264`, `metrics.py:229` | Nearest-time lookup written three ways (same as A8) | fix | fixed (35141c1): one `eval.timing.nearest` -- decided by evan-wes, 2026-10-05 |
| H7 | should-fix | hygiene | `bindings/foundations/{orb,stereo}` | `orb.Options` marshalling duplicated per binding language (same as A4) | fix | fixed (a3a54b9): `orb_crossing` in both binding languages -- decided by evan-wes, 2026-10-05 |
| H8 | should-fix | hygiene | `rust/slamlab-foundations/src/{image,matching,orb,stereo}.rs` | Rust fixture-reading test helpers copied into four modules | defer | deferred (design doc open items, PR 7's panel) -- decided by evan-wes, 2026-10-05 |
| H9 | should-fix | hygiene | `orb`, `stereo`, `matching` in three languages | Unnamed constants (254, 257, 64, 0.1); messages hard-code constant values; `MAX_IMAGE_SIDE` doubles as the feature cap | fix | fixed (b536911): named constants; C++/Rust `check_range` messages built from them -- decided by evan-wes, 2026-10-05 |
| H10 | should-fix | hygiene | `runs.py:200` | `write_run` writes `run.json` before validating; not atomic | fix | fixed (35141c1): checked whole before writing; each file replaced via a temporary -- decided by evan-wes, 2026-10-05 |
| H11 | should-fix | hygiene | `runs.py:324` | `evaluate`'s `max_difference=0.01` is unnamed and unexplained | fix | fixed (35141c1): `runs.MAX_DIFFERENCE` = 0.02 s (TUM's) -- decided by evan-wes, 2026-10-05 |
| H12 | should-fix | hygiene | `euroc.py:175-229` | EuRoC reader accepts NaN positions, duplicate stamps, non-integer resolutions; error paths untested | fix | fixed (35141c1) -- decided by evan-wes, 2026-10-05 |
| H13 | should-fix | hygiene | `image.py:93`, `orb.py:604`, `matching.py:208`, `stereo.py:311` | Backend-result decoders duplicated, with different cast policies | fix | fixed (b536911, a3a54b9): `backends.checked_array`, never cast -- decided by evan-wes, 2026-10-05 |
| H14 | should-fix | hygiene | `matching.py:122` | Python `_match` holds the full N x M matrix (and a copy); `distances`' N x M output undocumented | fix | fixed (b536911): `_match` takes the first set in blocks; `distances` documents N x M -- decided by evan-wes, 2026-10-05 |
| H15 | should-fix | hygiene | `cpp/src/eval/png.cpp:41` | Native `read16` mislabels a missing file; stb_image skips CRC checks (undocumented difference) | fix | fixed (35141c1): missing files named as such; CRC difference documented; truncated-file tests -- decided by evan-wes, 2026-10-05 |
| H16 | should-fix | hygiene | tests | Test gaps (huge `xy`, rectification-map rejects, corrupt PNG, kind-less Rust asserts, private `_PATTERNS`, EuRoC error paths) | fix | fixed (b536911, a3a54b9, 35141c1) -- decided by evan-wes, 2026-10-05 |
| H17 | should-fix | hygiene | `orb.rs:668`, `stereo.py:519` | Long functions where the C++ side is already split | fix | fixed (a3a54b9): Rust `ScoreWindow`/`cell_thresholds`; Python `_nearest_candidate`, `_drop_outliers` -- decided by evan-wes, 2026-10-05 |
| H18 | note | hygiene | `scripts/demo_*.py` | Fastest-backend choice written twice (same as A13) | defer | deferred (design doc open items, PR 7's panel) -- decided by evan-wes, 2026-10-05 |
| H19 | note | hygiene | various | Smaller items: stale `arrays` docstring, `_as_angles` on `[]` and bools, C++ `static_assert`, Pillow decompression-bomb error, Rust `expect` message, C++ `at()` copies, `_POPCOUNT` temporary | fix the first five; defer the last two | first five fixed (b536911); `at()` copies and `_POPCOUNT` deferred (design doc open items) -- decided by evan-wes, 2026-10-05 |
| A1 | must-fix | architecture | `stereo.py:642`, `stereo.hpp:112`, `stereo.rs:498` | `stereo.match` takes `orb_options` on trust (wrong depths, no error, on a mismatch) and rebuilds both pyramids | fix (decision: how) | fixed (a3a54b9): `Features` carry `scale_factor` and `levels`; `orb_options` gone; pyramid reuse deferred to PR 9 (design doc open items) -- decided by evan-wes, 2026-10-05 |
| A2 | must-fix | architecture | design doc `:47`, `:63`, `:385`, `:700`; `foundations/__init__.py` | Design doc never records the front end in rung 0; still lists ORB under the edge rung | fix | fixed (96dbdef) -- decided by evan-wes, 2026-10-05 |
| A3 | should-fix | architecture | design doc `:77`, `:299`, `:357`; `CLAUDE.md:38`; `eval/__init__.py` | `slamlab_eval`/`slamlab-eval` missing from the doc's tables and CLAUDE.md; `eval` docstring stale | fix | fixed (35141c1, 96dbdef) -- decided by evan-wes, 2026-10-05 |
| A4 | should-fix | architecture | `bindings/foundations/{orb,stereo}` | `orb.Options` and features cross twice per language; add `orb_crossing` (same as H7) | fix | fixed (a3a54b9) (with H7) -- decided by evan-wes, 2026-10-05 |
| A5 | should-fix | architecture | `cpp/src/eval/png.cpp:10` | stb_image exports its API (and process-global setters) from `libslamlab_eval.a` | fix | fixed (35141c1): `STB_IMAGE_STATIC`; no `stbi_*` symbol exported (was 42) -- decided by evan-wes, 2026-10-05 |
| A6 | should-fix | architecture | `eval/images.py`, `eval/png.hpp`, `png.rs` | Native readers' names (`read8`/`read16`) and error kinds differ from Python's | fix | fixed (35141c1, 96dbdef): `read_gray`/`read_rgb`/`read_depth`; error kinds in the design doc -- decided by evan-wes, 2026-10-05 |
| A7 | should-fix | architecture | `stereo.cpp:24`, `stereo.rs:51` | C++ and Rust stereo hard-code each camera model's focal layout | fix | fixed (a3a54b9): `camera::focal_indices` / `Model::focal_indices` -- decided by evan-wes, 2026-10-05 |
| A8 | should-fix | architecture | `tum.py`, `euroc.py`, `metrics.py` | Nearest-time association three times (same as H6) | fix | fixed (35141c1) (with H6) -- decided by evan-wes, 2026-10-05 |
| A9 | note | architecture | `eval/runs.py` | Run format has no language-neutral fixture; native writers' home and .npy writer undecided | defer (PR 8) | deferred (PR 8; design doc open items) -- decided by evan-wes, 2026-10-05 |
| A10 | note | architecture | `foundations/matching.py` | Guided matching belongs in rung 0 beside `matching` | defer (PR 8) | deferred (PR 8; design doc open items) -- decided by evan-wes, 2026-10-05 |
| A11 | note | architecture | `CMakeLists.txt:75` | `slamlab_eval` (stb_image) compiled in wheel builds that never link it | fix | fixed (35141c1): built only when `NOT SKBUILD OR SLAMLAB_BUILD_TESTS` -- decided by evan-wes, 2026-10-05 |
| A12 | note | architecture | `orb.py:241` | Image-side limit doubles as the feature cap (with H9) | fix | fixed (b536911) (with H9: `orb.MAX_FEATURES`) -- decided by evan-wes, 2026-10-05 |
| A13 | note | architecture | `scripts/demo_*.py` | Demos each pick a backend (same as H18) | defer | deferred (with H18) -- decided by evan-wes, 2026-10-05 |
| L1 | must-fix | algorithm | `stereo.cpp:373`, `stereo.rs:513`, `stereo.py:579` | `stereo.match` crashes on keypoint coordinates it never range-checks (same as H1) | fix | fixed (a3a54b9) (with H1) -- decided by evan-wes, 2026-10-05 |
| L2 | should-fix | algorithm | `stereo.{py,cpp,rs}` SAD | ORB-SLAM2's centre subtraction causes a 0.07-0.27 px disparity bias; ORB-SLAM3 dropped it; the note blames pixel locking | fix (decision: which variant) | fixed (a3a54b9): zero-mean SAD with the V fit; median error under 0.002 px -- decided by evan-wes, 2026-10-05 |
| L3 | should-fix | algorithm | `stereo.py:241` | `rectify` accepts a pair whose right camera is on the left; matching then finds only false matches | fix | fixed (a3a54b9): `InvalidInput`, "swap the cameras" -- decided by evan-wes, 2026-10-05 |
| L4 | should-fix | algorithm | `orb.py:559` | Keypoint full-image `xy = level_xy * s^l` ignores the aligned-centre resize: up to 1.9 px off on coarse levels | fix (decision: convention) | fixed (a3a54b9): the exact inverse `((2x + 1) n_0 - n_l) / (2 n_l)` -- decided by evan-wes, 2026-10-05 |
| L5 | should-fix | algorithm | `runs.py:352`, `metrics.py:144` | RPE pairs consecutive matched poses, so a tracking gap counts as one step | fix | fixed (35141c1): RPE pairs by time (delta in seconds, a tolerance) -- decided by evan-wes, 2026-10-05 |
| L6 | should-fix | algorithm | `metrics.py:190` | `associate` is not TUM's greedy and does not check order (same as H3, H5) | fix | fixed (35141c1) (with H3, H5) -- decided by evan-wes, 2026-10-05 |
| L7 | note | algorithm | `orb.py:649` | FAST equals OpenCV's only for thresholds of at least 1 | fix | fixed (b536911): thresholds start at 1 -- decided by evan-wes, 2026-10-05 |
| L8 | note | algorithm | `matching.py:16` | ORB-SLAM's rotation histogram actually bins at 30 degrees; ours at 12 | fix (document) | fixed (96dbdef): documented; kept in view in the open items -- decided by evan-wes, 2026-10-05 |
| L9 | note | algorithm | `stereo.py:510`, `docs/notes/front-end.md:86` | Stereo departures from ORB-SLAM undocumented; the note's NaN claim about ORB-SLAM is wrong; two branches unreachable | fix | fixed (a3a54b9): branches removed, departures listed, the note corrected -- decided by evan-wes, 2026-10-05 |
| L10 | note | algorithm | `CMakeLists.txt:29` | Exact float paths rely on code shape; add `-ffp-contract=off` before the ARM builds | fix | fixed (a3a54b9): `-ffp-contract=off`, PUBLIC on the core targets -- decided by evan-wes, 2026-10-05 |
| L11 | note | algorithm | -- | State of the art for later rungs (equiangular sub-pixel fit, XFeat, SuperPoint/LightGlue, 4-DoF VI alignment, KITTI segments) | defer (design doc open items) | deferred (design doc open items) -- decided by evan-wes, 2026-10-05 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** PR 7 (`feature/front-end`), `59bccd8...5b4f709`. I read all 77 non-vendored, non-fixture files in full: the Python `foundations` (`image`, `orb`, `matching`, `stereo`, `arrays`, `camera`) and `eval` (`images`, `euroc`, `metrics`, `runs`, `tum`) modules, the C++ and Rust cores, bindings, PNG readers, benches, demos, tests and fixture generators (`generate_image.py`, `generate_stereo.py`, `_layout.py`), plus the design note, CHANGELOG, THIRD_PARTY_NOTICES and pyproject changes. I skipped `stb_image.h`'s contents and the generated fixture data. Beyond reading, I ran probes against the built extensions (scripts in `/tmp/scratch/`). No repo files were changed; `git status` is clean and HEAD is `5b4f709`. REPO = the repository root.

**Summary:** The code is well documented and cleanly structured. Docstrings state shapes, dtypes and conventions, suppressions are all justified, and I found no stray prints or TODOs. Bit-identical parity held in my fuzz: 120 random images from 1 to 89 px with random options gave identical `detect`/`extract` results on C++ and Rust. There are four must-fixes, all missing input validation at public boundaries. The worst is a segfault (C++) and a panic (Rust) from `stereo.match` on finite but huge keypoint coordinates. The rest are should-fixes: logic written two or three ways, constants that are not named, and some test gaps.

### Findings

#### [must-fix] `stereo.match` segfaults (C++) and panics (Rust) on finite but huge keypoint `xy`
- **Where:** `src/slamlab/foundations/stereo.py:579-619` (`_as_side` and `_check_inside`), `cpp/src/foundations/stereo.cpp:55-67,373-376`, `rust/slamlab-foundations/src/stereo.rs:299-313,368-372,513`.
- **Issue:** The Python wrapper only checks that `xy` is finite. The cores' `check_side` checks neither finiteness nor range. `check_inside` looks only at the left `level_xy`, never at `xy`. The matcher then does `column = floor(right.xy[0] / scale + 0.5)` and casts it to int64:
  - C++: the float-to-int cast of a value outside int64 is undefined behaviour. The following `column - reach - w` and `column + reach + w` arithmetic then runs on garbage, and the loop indexes out of bounds.
  - Rust: `as i64` saturates to `i64::MAX`, so `column + (reach + w) as i64` overflows (it wraps in release). The code then indexes `right.at(y, c)` out of range and panics.
  - Python reference: correctly returns no matches.
- **Why it matters:** A public, Python-callable function crashes the interpreter on finite input. This happens whenever `xy` is inconsistent with `level_xy` (features from another image or pyramid, or hand-built). The Python `Keypoints` docs define `xy = level_xy * scale^level`, but nothing enforces it.
- **Evidence:** In `probe2b.py`, both feature sets use the same keypoints with every `xy[:,0]` set to `sx`, then `stereo.match(img, img, left, left, focal=100.0, baseline=0.1, backend=be)`:
  ```
  python 1e+300 OK 0 | python 1e+19 OK 0
  cpp    1e300 / 1e19 -> "Segmentation fault (core dumped)"
  rust   1e300 / 1e19 -> PanicException: index out of bounds: the len is 52000 but the index is 9223372036854780742
  ```
- **Suggested fix:**
  - In the cores, test `0 <= xy/scale + 0.5 < level.cols` in floating point before the int cast and skip the candidate otherwise. Or require `xy == level_xy * scale^level` exactly, which is computable identically in all three languages.
  - Check the same thing in `_as_side`.
  - Add a backend-parametrised test with huge and negative `xy`.

#### [must-fix] `Options` dataclasses (orb, matching, stereo) do not check types, so the three backends fail differently
- **Where:** `src/slamlab/foundations/orb.py:233-259`, `src/slamlab/foundations/matching.py:69-82`, `src/slamlab/foundations/stereo.py:132-153`.
- **Issue:** `__post_init__` range-compares only. Floats, bools and ints-as-bools pass, then reach each backend through `asdict`.
- **Why it matters:** This breaks the "every bad input fails the same way, with `InvalidInputError`" contract (CLAUDE.md). The failures also give no message naming the bad value.
- **Evidence** (`probe1.py`):
  ```
  orb.Options(n_features=2.5), Options(levels=True), Options(fast_threshold=20.5) -> constructed OK
  extract n_features=50.5: python OK | cpp RuntimeError "std::bad_cast" | rust ValueError "options: TypeError: 'float' object cannot be interpreted as an integer"
  match check_orientation=3: python OK | rust OK | cpp RuntimeError "std::bad_cast"
  stereo window=2.5: python TypeError "slice indices must be integers" | cpp bad_cast | rust ValueError
  orb.Options(n_features='a') -> TypeError "'<=' not supported between instances of 'int' and 'str'"
  ```
  The same applies to `stereo.match(focal='a')` and `focal=None`, which give a bare `TypeError` instead of `InvalidInputError`.
- **Suggested fix:**
  - Promote `image._as_count` (`image.py:279-285`, which already rejects bool and non-int with named messages) to `arrays` and use it in the three `__post_init__`s.
  - Add an analogous check for `check_orientation` (must be bool) and a real-number check for the float settings and `focal`/`baseline`.
  - Add tests for non-integer and bool settings on each backend.

#### [must-fix] `metrics.associate` documents ascending inputs but does not check them, and `runs.evaluate` never checks `run.timestamps`
- **Where:** `src/slamlab/eval/metrics.py:195-223` (docstring says "ascending"; the checks stop at 1D and finite), `src/slamlab/eval/runs.py:349-352`.
- **Issue:** `np.searchsorted` needs sorted `reference`. `evaluate` validates the ground truth through `_as_stamped`, which enforces ascending, but passes `run.timestamps` straight in. A directly constructed `Run` has no ordering check; only `read_run` has one.
- **Why it matters:** An unsorted input silently yields wrong pairs and therefore a wrong ATE.
- **Evidence:** `metrics.associate([1.0, 2.0, 3.0], [3.0, 1.0, 2.0], 0.1)` returns `([0, 1], [1, 2])`. Time 3.0 is dropped although reference 3.0 exists at index 0.
- **Suggested fix:** Check `np.all(np.diff(x) > 0)` for both arrays in `associate` and raise `InvalidInputError` naming the argument. Add a test.

#### [must-fix] `runs.read_run` raises raw `JSONDecodeError`, `KeyError` and `AttributeError` instead of the documented `InvalidInputError`
- **Where:** `src/slamlab/eval/runs.py:266-297` (docstring at 258-263 promises `InvalidInputError` "if a file is malformed").
- **Issue:** Unguarded `json.loads` (lines 266 and 279), `info.get` on a non-dict, and `info["mode"]`, `info["backend"]`, `info["dataset"]` and `info["sequence"]`.
- **Evidence** (`probe4.py`):
  ```
  run.json "{not json"            -> JSONDecodeError
  run.json "[]"                   -> AttributeError: 'list' object has no attribute 'get'
  run.json {format,version only}  -> KeyError: 'mode'
  ```
- **Suggested fix:** Wrap the parsing, check `isinstance(info, dict)` and required keys, and raise `InvalidInputError` with the file and key. Add tests next to `test_read_run_rejects_unknown_info`.

#### [should-fix] `associate` claims to be "as TUM's `associate.py` does" but pairs differently
- **Where:** `src/slamlab/eval/metrics.py:191-241`.
- **Issue:** TUM's script is a global greedy match: all pairs under the tolerance sorted by gap, each side used once. This code takes each time's single nearest reference and then drops the losers, so a time whose nearest is claimed does not fall back to its second nearest.
- **Evidence:** `associate([0.0, 0.004], [0.0045, 0.005], 0.01)` returns one pair, `([1], [0])`. TUM's greedy gives two: `(0.0, 0.005)` and `(0.004, 0.0045)`.
- **Suggested fix:** Implement the greedy version or reword the docstring. For the algorithm reviewer: ATE/RPE scores will differ from published numbers on dense data.

#### [should-fix] The "nearest in sorted times" logic is written three ways
- **Where:** `src/slamlab/eval/tum.py:161-168` (`_nearest`), `src/slamlab/eval/euroc.py:264-271` (inline, plus a `len == 1` special case), `src/slamlab/eval/metrics.py:229-233`.
- **Issue:** Three separately written searchsorted-and-compare blocks, each with its own edge handling for 1 and 2 elements.
- **Suggested fix:** One `nearest_index(times, stamps)` in `metrics` (the lowest eval module), used by all three.

#### [should-fix] ORB-options marshalling is duplicated in the C++ bindings and in the Rust bindings
- **Where:** C++: `cpp/bindings/foundations/orb.cpp:32-47` versus `cpp/bindings/foundations/stereo.cpp:92-94,151-158` (a second key list, `kOrbKeys`, identical to `kOrbOptionKeys`). Rust: `rust/slamlab-py/src/foundations/orb.rs:20-34,179-189` versus `rust/slamlab-py/src/foundations/stereo.rs:170-184,308-316` (`OptionsIn` and `OrbOptionsIn`).
- **Why it matters:** A new `orb.Options` field has to be added in two places per language, and a miss would only show up at runtime.
- **Suggested fix:** Share one reader per language (a small header or module next to `image_crossing`).

#### [should-fix] Rust fixture-reading test helpers are copied into four modules
- **Where:** `rust/slamlab-foundations/src/image.rs:598-644`, `.../matching.rs:283-310`, `.../orb.rs:907-940`, `.../stereo.rs:565-600` (`fixture`/`functions`, `count`, `list`, `image`, descriptor and feature readers).
- **Issue:** The C++ side shares these through `fixture_support.hpp`, `image_support.hpp` and `test_support.hpp`; Rust repeats them.
- **Suggested fix:** A `#[cfg(test)] pub(crate) mod test_support` in `slamlab-foundations`.

#### [should-fix] Unnamed magic numbers, and error messages that hard-code constants
- **Where:**
  - FAST max threshold 254: `orb.py:251`, `cpp/src/foundations/orb.cpp:522`, `rust/.../orb.rs:274`. Only Python has a one-word comment.
  - Descriptor-distance cap 257 and SAD half-range 64: `stereo.py:140,148`, `cpp/src/foundations/stereo.cpp:315,325`, `rust/.../stereo.rs:262,275`. The 64 has no stated rationale.
  - Inline `0.1` for the three-maxima rule: `matching.py:143`, `cpp/src/foundations/matching.cpp:87`, `rust/.../matching.rs:200`.
  - Messages that spell out values already held as constants: C++ and Rust `"must be from 1 to 1048576"`, `"from 1 to 32"` and `"from 1 to 19"`.
  - `MAX_IMAGE_SIDE` reused as the bound for `n_features`, `target` and descriptor budgets (`orb.py:241,701`; C++ `kMaxSide`).
- **Why it matters:** The messages drift if a constant changes. Reusing an image-side limit as a feature-count limit is a coupling no reader would expect.
- **Suggested fix:** Named, justified constants (`MAX_FAST_SCORE`, `MAX_SEARCH`, `MAX_FEATURES`, `MIN_SECONDARY_FRACTION`), formatted into the messages.

#### [should-fix] `write_run` writes `run.json` before it finishes validating, and is not atomic
- **Where:** `src/slamlab/eval/runs.py:200-224`.
- **Issue:** `run.json` is written (line 218) before `write_trajectory` validates the timestamps and poses (line 219). A bad run therefore leaves a half-written folder, which `read_run` would then misread. A non-JSON `settings` value such as `np.int64` raises `TypeError`, not `InvalidInputError`.
- **Evidence:** `write_run` with descending timestamps raises `InvalidInputError` and leaves `['run.json']`.
- **Suggested fix:** Validate everything first (`_as_stamped` for the trajectory too, and `json.dumps` of the settings), then write. Writing to a temp folder and renaming would make it atomic.

#### [should-fix] `evaluate`'s default `max_difference=0.01` is unnamed and disagrees with the datasets
- **Where:** `src/slamlab/eval/runs.py:324`. Compare `tum.py:52` (0.02, TUM's own default) and `euroc.py:36` (0.0025).
- **Issue:** A third tolerance with no constant and no stated reason, and the docstring does not give the default.
- **Suggested fix:** Name it and justify it, or make the dataset readers' constants the defaults.

#### [should-fix] EuRoC reader validation gaps
- **Where:** `src/slamlab/eval/euroc.py:195-229` (`_ground_truth`), `175-192` (`_frames`), `171` (`shape`).
- **Issue:**
  - NaN or inf positions pass: only the quaternion norm is checked, and `float('nan')` parses.
  - Duplicate timestamps in `data.csv` silently keep the last file.
  - `resolution` values are truncated with `int()` and never checked for being positive integers.
  - The malformed-row paths in `_frames` and `_ground_truth` have no tests.
- **Evidence:** A ground-truth row `1000,nan,0,0,1,0,0,0` yields `T_world_body` with translation `[nan 0. 0.]`. Two `1000,*.png` rows yield `{1000: 'b.png'}`.
- **Suggested fix:** Require finite values, reject duplicate stamps and non-positive or non-integer resolutions, and add tests.

#### [should-fix] Python backend-result decoders are duplicated and apply different policies
- **Where:** `src/slamlab/foundations/image.py:93-104`, `.../orb.py:604-633`, `.../matching.py:208-222`, `.../stereo.py:311-328`.
- **Issue:** `_decode_image`, `_decode_indices` and `_decode_distances` are the same "asarray, check dtype, raise RuntimeError" function. `_decode_distances` is also used for `pairs`, which is a misleading name. Some decoders say "never cast", while `orb._decode_keypoints` and `stereo._decode_matches` and `_decode_rectification` cast silently with `dtype=`.
- **Suggested fix:** One `backends.checked_array(raw, dtype, ndim, name)` and one policy.

#### [should-fix] `matching` allocates memory proportional to N×M with no bound
- **Where:** `src/slamlab/foundations/matching.py:122-130,157-169`, `cpp/src/foundations/matching.cpp:120-133`, `rust/slamlab-foundations/src/matching.rs:106-114`. [backlog candidate]
- **Issue:** `MAX_DESCRIPTORS = 2^20` caps each set, not the product.
  - Python `_match` builds the full N×M int64 matrix plus a `.copy()` for the second-nearest, for 16·N·M bytes in total. For example, 20k×20k is about 6.4 GB.
  - C++ and Rust `distances` reserve N·M.
  - C++ and Rust `match` are already streaming.
- **Suggested fix:** Chunk `_match` by rows, as `_distances` already does with `_CHUNK`, so it holds only best and second per row. Document or cap `distances`'s product.

#### [should-fix] Native PNG readers: misleading error for a missing file, and no CRC checking
- **Where:** `cpp/src/eval/png.cpp:41,61`; `third_party/stb/stb_image.h:4599`.
- **Issue:**
  - `read16` of a missing file reports "expected a 16-bit single-channel image". `stbi_is_16_bit` returns 0 when it cannot open the file (`stb_image.h:7712-7716`), so the check sends it down the wrong branch. The file is also opened twice.
  - stb_image does "no CRC checking". The Python reader rejects a bad-CRC PNG (`cannot identify image file`), and Rust's `png` crate rejects it by default. The docs promise the three read a file "the same way", but corrupt files are not held to that.
- **Suggested fix:** Check that the file opens first, so `read16` reports unreadable. State the CRC difference in `png.hpp`. Add a corrupt-file case to the native tests and the shared fixtures.

#### [should-fix] Test gaps
- **Where:** various.
- **Issue:**
  - No test feeds `stereo.match` inconsistent or huge `xy` (the first must-fix).
  - No negative-input tests for `stereo.rectification_maps` (non-rigid rotation, bad intrinsics).
  - No native test of a corrupt PNG.
  - Rust `matching.rs:370-371` asserts `.is_err()` without checking the kind, while the lines above it do.
  - `tests/foundations/test_orb.py:165` reads the private `orb._PATTERNS`.
  - `euroc._frames` and `_ground_truth` error branches are untested (see the EuRoC finding).
  - There is no test for `Options` type errors, `associate` unsorted input, or `read_run` malformed JSON.
- **Suggested fix:** Add tests alongside the fixes above.

#### [should-fix] Long functions where the C++ side already has a better structure
- **Where:** `rust/slamlab-foundations/src/orb.rs:668-722` (`level_corners`, about 55 lines: score window, cell thresholds, non-maximum suppression and scan in closures), `src/slamlab/foundations/stereo.py:519-576` (`_match`, about 58 lines).
- **Issue:** C++ splits the first into `ScoreWindow`, `cell_thresholds` and `level_corners`. Python `_match` inlines candidate selection that C++ and Rust keep in `nearest`.
- **Suggested fix:** Extract a score-window helper in Rust and `_nearest_candidate` in Python.

#### [note] `scripts/` duplicates the "fastest built backend" choice
- **Where:** `scripts/demo_absolute_pose.py:76-78` (`_backend`) and `scripts/demo_two_view.py:64` (inline `next(b for b in ("cpp", "rust", "python") ...)`).
- **Issue:** The same preference order, written twice.
- **Suggested fix:** A `backends.fastest()` helper. [backlog]

#### [note] Smaller items
- **`arrays.py` module docstring is stale** (`src/slamlab/foundations/arrays.py:1-10`). It says every backend "sees the same clean float64 data", but images are now uint8 (`as_image`). `MAX_IMAGE_SIDE` is defined mid-file after its first mention.
- **`matching._as_angles`** (`src/slamlab/foundations/matching.py:198-205`). A literal `[]` is float64, so it raises "expected 0 integers, got float64 (0,)", which is confusing. `np.can_cast(bool, int64)` is true, so boolean angles are accepted. The same cast check is in `orb._as_corners`. (`matching.match` with `[]` angles fails the same way on all three backends.)
- **C++ `matching::distance`** (`cpp/src/foundations/matching.cpp:108-118`) `memcpy`s 8-byte words and has no `static_assert(kDescriptorBytes % 8 == 0)`. The Rust twin has a compile-time assert at `matching.rs:14`. Without it, changing the constant is a silent out-of-bounds read.
- **`images._open`** (`src/slamlab/eval/images.py:51-56`) catches only `OSError`, so Pillow's `DecompressionBombError` escapes unmapped (verified). Real datasets are far below the threshold. [backlog]
- **Rust `expect("inside")`** (`rust/slamlab-foundations/src/orb.rs:339-340`) states no invariant. The message should say "keypoint lies at least `EDGE_THRESHOLD` inside its level".
- **`at()` is copied** into three C++ anonymous namespaces (`cpp/src/foundations/image.cpp:21`, `orb.cpp:21`, `stereo.cpp:48`). [backlog]
- **`_POPCOUNT[block]`** (`matching.py:129`) makes a (256, M, 32) int64 temporary per chunk, which is roughly 130 MB at M = 2000. `np.bitwise_count` (NumPy 2) or uint64 views would avoid it. [backlog]

### Checked and fine
- **Documentation:** numpy, Doxygen and rustdoc docstrings carry shapes, dtypes, frame names and rounding rules throughout. The module docstrings (`image`, `orb`, `matching`, `stereo`) say what each does and how it relates to ORB-SLAM, including the deliberate departures. `docs/notes/front-end.md`, the CHANGELOG, the VERSION bump (0.3.1.0, the PATCH digit for new features) and the design doc and `CLAUDE.md` binding-boundary edits are consistent with the code.
- **Licensing:** the rBRIEF pattern's BSD notice sits beside each of the three copies and in `THIRD_PARTY_NOTICES`. stb_image is vendored unmodified and attributed.
- **Parity probe:** `probe3.py` ran 120 random image sizes (1-89 px) and random options through `detect` and `extract` on C++ and Rust against Python, with 0 mismatches, including images too small for any keypoint.
- **Edge inputs:** empty feature sets, empty descriptor sets, empty `distribute` and 1×1 images behave identically on all three backends (`probe6.py`).
- **Python PNG handling:** truncated, bad-CRC and corrupt-stream PNGs all become `InvalidInputError` naming the file.
- **Suppressions:** every `noqa`, `NOLINT` and `#[allow]` carries a stated reason. There are no debug prints, commented-out code, or TODOs without an owner.
- **Core error handling:** C++ cores use `[[nodiscard]] Result<>` consistently with no swallowed results. Rust `expect`s are on invariants only, apart from the message-quality note above.
- **Tests:** the fixture generators are independent of the code under test: `generate_image.py` works from definitions with exact `Fraction`s, `generate_stereo.py` uses OpenCV for the rotations and brute force for the matches. The ATE/RPE, EuRoC, runs and image tests assert values (known offsets, planted disparities, exact round trips, OpenCV agreement bounds).
- **Architecture hygiene:** import-linter layers and the Pillow-only-in-`eval.images` contract are updated. The native benches use identical synthetic frames in both languages.

### Architecture review

**Scope:** slamlab PR 7, `feature/front-end`, `59bccd8...5b4f709` (9 commits). Repository root: the repository root, and every path below is relative to it. I reviewed the 77 non-vendored, non-fixture files: the Python modules `foundations.{image,orb,matching,stereo,arrays,camera}` and `eval.{images,tum,euroc,metrics,runs}`; the C++ cores, headers and bindings; the Rust crates `slamlab-foundations`, `slamlab-eval` and `slamlab-py`; CMake, Cargo, `pyproject.toml`, the design doc, `CLAUDE.md`, the demos and the benches. I also reviewed how stb_image is integrated (CMake, `cpp/src/eval/png.cpp`, `THIRD_PARTY_NOTICES`) and the fixture generators `tests/fixtures/{foundations/generate_{image,orb,matching,stereo},eval/generate_png,foundations/_layout}.py`. I ran timing and probe scripts against the built backends; the scripts are in the session scratchpad, outside the repo.

**Summary:** The layering is sound. The four new rung-0 modules sit correctly in the internal order. The front end pulls nothing heavy into rung 0. Pillow is held in `eval.images` by contract. The native PNG decoders are confined to an `eval` library and crate that no rung can link. Bindings are thin, one file per module, with an `image_crossing` helper. Two must-fixes:
- `stereo.match` depends on an `orb_options` argument that defaults and is never checked against the features it receives. A mismatch silently returns wrong depths, and the API also forces both pyramids to be rebuilt on the real-time path.
- The design doc, which is the authority on the ladder, was not updated for the decided move of the front end into rung 0.

The rest are smaller structural cleanups.

### Findings

#### [must-fix] `stereo.match` takes the extraction's settings on trust, and rebuilds both pyramids
- **Where:** `src/slamlab/foundations/stereo.py:642` (`orb_options: orb.Options | None = None`), `:686`, `:530-535`; `cpp/include/slamlab/foundations/stereo.hpp:112-115`; `cpp/src/foundations/stereo.cpp:346-348`; `rust/slamlab-foundations/src/stereo.rs:498-499`
- **Issue:** `Features` does not carry the pyramid it was extracted on. `match` therefore needs the images plus a separate `orb_options`, which defaults to `orb.Options()`, and rebuilds both pyramids from them. Nothing checks that those options are the ones the features were extracted with.
- **Why it matters:**
  - **Correctness:** if the extraction used anything but the defaults and the caller leaves the argument out (the natural mistake), every level above 0 is refined on the wrong scales. The result has no error and depths that are off by up to 70%.
  - **Cost:** the edge pipeline's stereo mode (PR 8, real time in PR 9) builds each pyramid twice per frame in all three languages.
  - **Timing:** this is the API that PR 8's three pipelines will be built on, so it is cheapest to fix now.
- **Evidence:**
  - **Mismatch probe:** features extracted with `orb.Options(scale_factor=1.3)` on a pair shifted by 10 px (cpp backend). Passing the extraction's options gives 395 matches with median disparity 10.01. Omitting `orb_options` raises no error and gives 352 matches, with median disparity by level of 10.01, 10.28, 10.36, 12.83, 11.63, 11.51, 12.47 and 17.14 for levels 0 to 7 (true value 10).
  - **Timing:** on a 752x480 frame, `image.pyramid` takes 2.9 ms (cpp) / 1.2 ms (rust) through the binding, and `stereo.match` takes 7.0 / 4.9 ms. Two of those pyramids repeat work `extract` already did.
- **Suggested fix:**
  - **Structural (recommended):** make the extraction's pyramid parameters part of its result, for example `Features` carrying `scale_factor` and `levels`, or an extraction result holding the pyramid. Split the cores into `image::pyramid` plus `orb::extract_levels(levels, options)`, and have `stereo.match` take the two pyramids (or extraction results) instead of images plus `orb_options`. ORB-SLAM's `ComputeStereoMatches` reads the extractor's `mvImagePyramid` the same way.
  - **Minimal (if the API must stay):** make `orb_options` required and validate `xy == level_xy * scale_factor**level` exactly, which is how the extractor computes `xy`, in Python and in both cores. Return `InvalidInput` on a mismatch.

#### [must-fix] The design doc does not record that the front end is in rung 0, and still assigns ORB to the edge rung
- **Where:**
  - `docs/design/2026-10-02-slamlab-ladder.md:47-48` (the rung-0 box: "camera models, Lie groups, two-view geometry, BA solver")
  - `:63-64` (rung 0's row omits the front end; the edge row says "**ORB features**, tracking, keyframes, local BA")
  - `:385` (PR 7's row says "a PNG decoder in each core")
  - `:700-701` ("the cores read PNGs natively")
  - `src/slamlab/foundations/__init__.py:3-4` (lists milestone 1's contents only, "(design doc, milestone 1)")
- **Issue:** The owner decided to put the front end in `slamlab.foundations` so that milestone 4's classic rung can reuse it, and the code carries that decision out correctly. But the design doc never states it, and its ladder table still places ORB features in the edge rung. The doc also describes the PNG decoders as being "in each core", while the code, CHANGELOG and runtime-dependencies paragraph put them outside the cores. The rung-0 package docstring, where the design says a rung declares its contents, is also stale.
- **Why it matters:** The design doc is the authority reviewers and agents check placement against. As written, a PR 8 or milestone 4 agent would read ORB as edge-rung code and either put front-end code there or "correct" this PR. The decision exists only in the PR's history.
- **Evidence:** `grep -n -i "front end\|ORB"` on the design doc at HEAD: the only placement statement is line 64, under Edge.
- **Suggested fix:** Change the doc, not the code:
  - Add the front end (fixed-point images, ORB, Hamming matching, rectified stereo) to rung 0's box and row, and drop "ORB features" from the edge row (or say "on rung 0's ORB front end").
  - Record the decision and its reason (reuse by `classic`) in the milestone 2 section.
  - Fix PR 7's row and the A7 line to "outside the cores (`slamlab_eval`, `slamlab-eval`)".
  - Update the `slamlab.foundations` docstring to name the front end and milestone 2.

#### [should-fix] The new `eval` C++ library and Rust crate are missing from the doc's tables and `CLAUDE.md`; `eval`'s docstring is stale
- **Where:**
  - `docs/design/2026-10-02-slamlab-ladder.md:77` ("Rust: Rung 0 and the edge rung … The other rungs have no Rust")
  - `:299` (`slamlab.eval | -- | -- |`)
  - `:357` ("The dataset readers they need are copies of `slamlab.eval`'s")
  - `CLAUDE.md:38` ("Rung 0 and the edge VO only")
  - `src/slamlab/eval/__init__.py:9-10` ("the C++ and Rust cores read PNGs natively") and `:15-17` ("Still to come: ATE … and RPE")
- **Issue:** The runtime-dependencies paragraph was updated, but the authoritative Languages and Package-layout tables still say `eval` has no C++ or Rust, and that Rust covers only rung 0 and the edge rung. The native-benchmark paragraph still describes copied readers, even though shared ones now exist (and a `foundations` bench cannot link them). The `eval` package docstring omits the new `metrics` and `runs` modules, and still lists ATE/RPE as future work.
- **Why it matters:** The architecture contract ("Rust stays within its agreed subset unless the design doc changed") now contradicts itself across the two documents.
- **Suggested fix:** Change the docs:
  - Add `slamlab_eval` and `slamlab-eval` to the Package-layout row.
  - Widen the Rust row and the `CLAUDE.md` Languages table to "rung 0, the edge rung, and `eval`'s native readers".
  - Restate where native benches that need dataset readers live (`cpp/bench/eval/`, or a copy as before).
  - Update the `eval` docstring.

#### [should-fix] `orb.Options` (and features) cross the binding in two places in each language
- **Where:** C++ `cpp/bindings/foundations/orb.cpp:31-47` (`kOrbOptionKeys`, `orb_options`) and `cpp/bindings/foundations/stereo.cpp:92-94, 153-158` (`kOrbKeys`, inline parse); Rust `rust/slamlab-py/src/foundations/orb.rs:20-34, 179-190` (`OptionsIn`, `read_options`) and `rust/slamlab-py/src/foundations/stereo.rs:170-184, 308-316` (`OrbOptionsIn`, inline conversion).
- **Issue:** Each binding language has two independent copies of the `orb.Options` crossing, plus a features crossing used only by stereo. The project's convention for a crossing shared by several modules is a `*_crossing` helper (`nls_crossing`, `ransac_crossing` and, in this PR, `image_crossing`).
- **Why it matters:** A field added to `orb.Options` has to be added in four places. Strict key checks catch an omission at runtime, but jscpd does not flag blocks this short.
- **Suggested fix:** Add `cpp/bindings/foundations/orb_crossing.hpp` and `rust/slamlab-py/src/foundations/orb_crossing.rs`, holding the options reader, the keypoints-dict writer and the features reader. Use them from `orb` and `stereo`.

#### [should-fix] The vendored stb_image exports its whole API, including process-global setters, from `libslamlab_eval.a`
- **Where:** `cpp/src/eval/png.cpp:10-13`; `CMakeLists.txt:75-76`
- **Issue:** `STB_IMAGE_STATIC` is not defined, so `STBIDEF` is `extern` (`stb_image.h:395-398`). Every `stbi_*` function gets external linkage in slamlab's static library. That includes the process-global flag setters `stbi_set_flip_vertically_on_load`, `stbi_set_unpremultiply_on_load` and `stbi_convert_iphone_png_to_rgb`.
- **Why it matters:** Any binary that links `slamlab_eval` alongside another copy of stb_image (a viewer, a reifyscene native tool, a second third-party library) gets either duplicate-symbol errors or a silently shared copy. Through the shared copy, another component's `stbi_set_flip_vertically_on_load(1)` would flip slamlab's frames. That is a hidden global crossing the library boundary.
- **Evidence:** `nm -C --defined-only build/native/libslamlab_eval.a | grep -cE " [TBDW] stbi"` returns `42`, among them `T stbi_convert_iphone_png_to_rgb` and `T stbi_failure_reason`.
- **Suggested fix:** Add `#define STB_IMAGE_STATIC` before the include in `png.cpp`, which makes the decoder private to that translation unit.

#### [should-fix] The native readers' error kinds and names do not mirror Python's
- **Where:**
  - `src/slamlab/eval/images.py:48-50` (missing file: `FileNotFoundError`) against `cpp/src/eval/png.cpp:30-36, 47-49` and `rust/slamlab-eval/src/png.rs:67-77` (missing file: `InvalidInput`)
  - names: Python `read_gray` / `read_rgb` / `read_depth`, against native `read8` / `read16` / `read_gray` (`cpp/include/slamlab/eval/png.hpp:42-55`, `rust/slamlab-eval/src/png.rs:115, 144, 179`)
- **Issue:** The same reader fails differently by language. A missing frame is `FileNotFoundError` in Python but `InvalidInput` natively, and the design doc defines that kind as "shape, range or finiteness". The function names also diverge (`read_depth` against `read16`).
- **Why it matters:** PR 8's native pipelines will be compared step by step with the Python one, and the error contract is meant to be "one shape in every language". These readers have no binding, so `test_backends` and the error-parity tests cannot catch the drift.
- **Suggested fix:** Mirror the names (`read_depth` in C++ and Rust). In the design doc's error-contract section, record that `eval`'s native readers report I/O failures as `InvalidInput` with the path in the message, and that Python raises `FileNotFoundError`. Alternatively, give the `eval` library and crate their own `Io` kind; rung 0 needs none, since it never touches files.

#### [should-fix] C++ and Rust stereo hard-code each camera model's focal-length layout
- **Where:** `cpp/src/foundations/stereo.cpp:24-30` (`smallest_focal`: `kRadial ? intrinsics(0) : min(intrinsics(0), intrinsics(1))`); `rust/slamlab-foundations/src/stereo.rs:51`. Compare Python's `stereo.py:383`, which uses `camera.FOCAL_INDICES` (`camera.py:40`).
- **Issue:** Knowledge of each model's intrinsics layout lives in the camera module in Python, but is copied into stereo in C++ and Rust.
- **Why it matters:** A new model (milestone 6's fisheye for TUM-VI) updates `camera` and silently gets "indices 0 and 1" from stereo in two languages, while Python fails loudly on the table lookup.
- **Suggested fix:** Expose the focal indices (or a `focal_lengths(model, intrinsics)` helper) from `camera` in C++ and Rust, and use it in stereo.

#### [should-fix] Nearest-timestamp association now exists three times in `eval`
- **Where:** `src/slamlab/eval/tum.py:161-168` (`_nearest`), `src/slamlab/eval/euroc.py:264-271` (inline copy) and `src/slamlab/eval/metrics.py:190-241` (`associate`)
- **Issue:** EuRoC re-implements TUM's searchsorted-and-clip association inline instead of reusing it. The copies have already drifted: TUM checks `len(times) == 1` before clipping, while EuRoC clips first (to the inverted range `[1, 0]`) and patches the result afterwards.
- **Why it matters:** Association is now a concept with three users. Under the "new concepts get their own module" rule, it should live in one place.
- **Suggested fix:** Lift `_nearest` into one `eval` helper (for example `slamlab.eval.timing`) used by `tum` and `euroc`, and build `metrics.associate` on it.

#### [note] The run format has no language-neutral fixture, and its native writers have no home yet (backlog, PR 8)
- **Where:** `src/slamlab/eval/runs.py:1-16`; `tests/eval/test_runs.py` (round trips only, no golden folder)
- **Issue:** The format is written by every backend's pipeline, but it is defined only by the Python module.
- **Why it matters:** PR 8 will run into three related gaps:
  - **Fixture:** C++ and Rust will need a golden run folder in `tests/fixtures/eval/`, which `read_run` validates, to write against.
  - **Dependency:** they will need an `.npy` writer. A new dependency is "a change to this document first", per the design doc.
  - **Home:** a native pipeline needs both `slamlab_edge` and `slamlab_eval`, so it must sit above `eval`, in orchestration. The Package-layout table gives orchestration no C++ or Rust.
- **Suggested fix:** Decide all three when PR 8 is scoped:
  - a golden folder in `tests/fixtures/eval/run/`;
  - a hand-written `.npy` writer or a named dependency;
  - the home of the native executables: `slamlab_orchestration` / `slamlab-orchestration`, or an apps directory held by `check_layers.py`.

#### [note] Guided matching belongs beside `matching` in rung 0 (backlog, PR 8)
- **Where:** `src/slamlab/foundations/matching.py`
- **Issue:** `matching` is brute force only. PR 8's tracking ("map points matched by projection") needs a windowed, guided matcher and a per-frame feature grid, ORB-SLAM's `SearchByProjection` and `GetFeaturesInArea`.
- **Suggested fix:** Following this PR's reasoning (the classic rung reuses the front end), put the generic parts in `foundations.matching`, not in `edge`.

#### [note] `slamlab_eval` is compiled in wheel builds that never link it
- **Where:** `CMakeLists.txt:75-87`. `_cpp` links only `slamlab_foundations`, yet `build/slamlab/libslamlab_eval.a` exists in the editable build.
- **Suggested fix:** Optionally guard the `eval` rung with `if(NOT SKBUILD OR SLAMLAB_BUILD_TESTS)` or `EXCLUDE_FROM_ALL`, so `pip install` does not compile stb_image. This costs build time only.

#### [note] The image-side limit doubles as the feature budget's cap
- **Where:** `src/slamlab/foundations/orb.py:241` (`n_features`) and `:701` (`distribute`'s `target`, both bounded by `MAX_IMAGE_SIDE`); `cpp/include/slamlab/foundations/orb.hpp:165`. `matching` has its own `MAX_DESCRIPTORS`.
- **Suggested fix:** Give `orb` a `MAX_FEATURES` constant, so that changing the image limit does not silently change the feature limit.

#### [note] The demos still each pick a backend with their own code
- **Where:** `scripts/demo_two_view.py:63`; `scripts/demo_absolute_pose.py:76-78`
- **Issue:** A8 asked for one shared helper. The ORB move removed most of the duplication, and the fastest-backend selection is the remaining copy.
- **Suggested fix:** Trivial now; fold it into a shared script helper when a third demo arrives.

### Checked and fine
- **Rung-0 internal order:** `image` (beside `correspondences`), `orb` (beside `nls`), `matching` (camera layer) and `stereo` (epipolar layer) each import only lower layers in all three languages, and `check_layers.py` holds the C++ includes and Rust paths to the same contract.
- **Pillow:** forbidden everywhere except `slamlab.eval.images`. Algorithm rungs cannot reach `eval` (layers contract). `pillow` is in `dependencies` as the design doc now records.
- **Native decoders:** `slamlab_eval` (DEPENDS `slamlab_foundations`, stb include `SYSTEM PRIVATE`) and `slamlab-eval` (the `png` crate and its pure-Rust deflate deps only) sit in the `eval` layer, where no rung crate or library may link them. `rust/slamlab-eval/src/lib.rs` documents this.
- **Edge budget:** the front end adds no heavy dependencies to rung 0 (numpy; Eigen; nalgebra/faer). OpenCV is a test oracle and demo reference only, and the `cv2` ban still holds.
- **Bindings:** one file per module plus `image_crossing` in both languages. They are type conversion only, uint8 images per the updated boundary rule, `noconvert` on arrays, and options read strictly as dicts.
- **Dispatch:** every public function validates once (`arrays.as_image`, `camera.as_model`, made public rather than reached into) and dispatches through `backends.call`. Each module has an `Implementation` Protocol that `test_backends` walks.
- **Error contract (front end):** `InvalidInput` and `Degenerate` are used consistently across Python, C++ and Rust for images, ORB, matching and rectification.
- **Standalone builds:** the cores build without Python. The benches (`orb_extract`) live in `cpp/bench/foundations` and `benches/`, held to the ladder.
- **Fixtures:**
  - The parity fixtures state their policies (exact for integer work, tolerance for the rectification maps).
  - The generators use independent oracles where one exists (brute-force FAST, hand-written BT.601 in `generate_png.py`) and say plainly where the reference is its own oracle (the quadtree and the extractor, like `ba.json`).
  - `_layout.py` removes the duplicated JSON compactor.
- **Metrics:** ATE and RPE reuse rung 0's Umeyama `alignment` rather than re-implementing it. `runs` and `metrics` sit in `eval`, so no rung depends on the viewer.
- **Licensing:** `THIRD_PARTY_NOTICES` covers the OpenCV pattern (all three copies located) and stb_image v2.30. stb's failure reason is thread-local in this version.
- **Import time:** there are no import-time side effects beyond cheap, deterministic constant tables (`_PATTERNS`, `_DISC_U/V`, `_POPCOUNT`).

### Algorithm review

**Scope:** `git diff 59bccd8...5b4f709` (feature/front-end, PR 7), excluding vendored `third_party/stb/stb_image.h` and fixture data. I read in full: `src/slamlab/foundations/{image,orb,matching,stereo}.py`, `src/slamlab/eval/{metrics,euroc,runs,images,tum}.py`, the C++ cores `cpp/src/foundations/{image,stereo}.cpp` plus the relevant parts of `orb.cpp` and `matching.cpp`, and the matching Rust parts (`orb.rs`, `stereo.rs`, `matching.rs`). I also read the tests (`tests/foundations/test_{image,orb,stereo}.py`, `tests/eval/test_metrics.py`) and the generators (`generate_{image,orb,matching,stereo}.py`).

I compared against these references: ORB-SLAM2 `Frame.cc` and ORB-SLAM3 `Frame.cc`/`ORBmatcher.cc` (fetched from GitHub), OpenCV's FAST/ORB/stereoRectify/cvtColor (cv2 5.0.0 in the env), Rublee et al. ICCV 2011, Sturm et al. IROS 2012, Burri et al. IJRR 2016. I ran read-only experiments; the scripts are in the session scratchpad. Two of them are saved for the fix work: `stereo_xy_crash.py` and `stereo_subpixel_bias.py`.

**Summary:** This is a careful, faithful port of ORB-SLAM's front end.
- The fixed-point arithmetic is sound: every intermediate is bounded and int64, and I found no overflow.
- FAST matches OpenCV exactly for thresholds of 1 and above, also on a real TUM frame.
- The rectifying rotations are cvStereoRectify's.
- The fixtures use independent oracles and cover the hard cases.

**One must-fix:** `stereo.match` never range-checks keypoint `xy`. An out-of-range input that passes validation segfaults the C++ core and panics the Rust core.

**The main should-fixes:**
- The stereo sub-pixel step uses ORB-SLAM2's centre-pixel normalisation, which ORB-SLAM3 dropped. That normalisation causes most of the measured disparity bias, and the note blames pixel locking instead.
- `rectify` silently accepts a pair whose "right" camera is on the left.
- Keypoint full-image coordinates are off by up to about 1.9 px on coarse levels.
- The run evaluator's RPE counts a tracking gap as one step.

### Findings

#### [must-fix] `stereo.match` crashes the C++ (segfault) and Rust (panic) cores on keypoint coordinates it never range-checks
- **Where:**
  - C++: `cpp/src/foundations/stereo.cpp:373-374` (unchecked `static_cast<std::int64_t>(std::floor(xy / scale + 0.5))`), `:105` (bounds check that overflows), `:55-67` (`check_side` checks counts and levels only).
  - Rust: `rust/slamlab-foundations/src/stereo.rs:513` (`as i64`, saturating), `:368-369` (`column + (reach + w) as i64` wraps in release), `:469-473` (the lint waiver's stated premise, "columns are at most 2^20", is enforced nowhere).
  - Python: `src/slamlab/foundations/stereo.py:579-604` (`_as_side` checks only that `xy` is finite; `_check_inside` checks the left `level_xy`, never `xy`).
- **Issue:** The matcher reads each keypoint's full-image `xy` for its candidate window and its right column, but `xy` is never checked against the image or against `level_xy`.
  - In C++, if `xy / scale` reaches 2^63, the cast is undefined. On x86 it yields INT64_MIN, `column - reach - w` then overflows, the bounds check passes, and the SAD loop reads far out of bounds.
  - In Rust, the cast saturates to `i64::MAX`, the upper check wraps negative, and indexing panics.
  - For standalone C++ callers, a NaN coordinate also gets through: every NaN comparison in `nearest()` is false, so the candidate is not skipped, and `floor(NaN)` is cast to int64.
- **Why it matters:** A memory-unsafe read is reachable from the public Python API with input that passed validation. It kills the interpreter on the `cpp` backend. It also breaks the design doc's contract: "The cores check index bounds and sizes and return `InvalidInput` … Standalone C++ and Rust callers get the same protection."
- **Evidence:** `stereo_xy_crash.py`: one left and one right keypoint, both with `level_xy` (30, 30)/(25, 30) but `xy = (1e300, 30)`, on a 60x80 image with `levels=1`:
  ```
  python  accepted keypoint xy=1e300 (level_xy 30): matches 0
  rust    pyo3_runtime.PanicException: index out of bounds: the len is 4800 but the index is 9223372036854778202
  cpp     Segmentation fault (core dumped)
  ```
- **Suggested fix:**
  - In the shared Python check and in both cores' `check_side`, require every keypoint's `xy` to be finite and inside the image (0 <= x <= cols-1, 0 <= y <= rows-1). Ideally also require it to be consistent with `level_xy * s^level`.
  - Make the refine bounds test overflow-free: compare in f64 before casting, or use checked arithmetic.
  - Add a rejects case on all three backends.

#### [should-fix] The stereo SAD subtracts each patch's centre pixel (ORB-SLAM2's), which causes most of the sub-pixel bias; ORB-SLAM3 dropped it, and the note blames pixel locking
- **Where:**
  - Code: `src/slamlab/foundations/stereo.py:500-505`, `cpp/src/foundations/stereo.cpp:108-118`, `rust/slamlab-foundations/src/stereo.rs:373-382`.
  - Docs: `docs/notes/front-end.md:83-85` and the test docstring/tolerance at `tests/foundations/test_stereo.py:199-215` (`atol=0.35`).
  - Fixture oracle: `tests/fixtures/foundations/generate_stereo.py:130-161`.
- **Issue:** `patch - patch[centre]` is ORB-SLAM2's `IL = IL - IL.at<float>(w,w)`. ORB-SLAM3's `Frame::ComputeStereoMatches` uses plain SAD on the raw patches. The keypoint is a FAST corner, the steepest point of its patch, so subtracting its value adds a large offset across the whole patch that changes with the sub-pixel shift.
- **Why it matters:** The bias is systematic: disparity is over-estimated in every row of the table below, so depth is under-estimated. Stereo is scored with SE(3) alignment, which removes no scale, so this would go straight into PR 8's metric depths and ATE. At 10 px disparity, 0.27 px is a 2.7% depth error. The note's explanation ("pixel locking, a known bias … that ORB-SLAM shares") does not match the measurement. The test's 0.35 px tolerance was also measured on a pair where only the right view is bilinearly interpolated, which blurs it, so the test is pessimistic as well.
- **Evidence:** `stereo_subpixel_bias.py`. Both views are area-sampled from one 10x-supersampled texture, so neither is blurred relative to the other. Level-0 matches from `stereo.match` (cpp), re-refined with each variant. Each cell shows the median error (px) and the share of matches within 0.1 px:

  | true d | slamlab (ORB-SLAM2) | ORB-SLAM3 plain SAD + parabola | zero-mean SAD + parabola | zero-mean SAD + V fit |
  |---|---|---|---|---|
  | 9.0 | +0.138, 2% | -0.001, 99% | +0.000, 99% | +0.000, 99% |
  | 9.2 | +0.067, 43% | -0.074, 99% | -0.077, 99% | -0.002, 99% |
  | 9.6 | +0.273, 35% | +0.067, 99% | +0.069, 99% | +0.001, 99% |
  | 9.8 | +0.253, 29% | +0.074, 100% | +0.077, 100% | +0.002, 100% |

  Even at an exact integer shift, only 2% of slamlab's matches are within 0.1 px.
- **Suggested fix:**
  - Drop the centre subtraction. Two options:
    - ORB-SLAM3's plain SAD.
    - Zero-mean SAD, which keeps a brightness-offset invariance that plain SAD loses and stays integer-exact as `sum |n(L-R) - (sum L - sum R)|` with n = (2w+1)^2.
  - Record the choice as a documented departure, citing ORB-SLAM3.
  - Correct the note and the test docstring, regenerate the fixture's `_sad_refine`, and test on an area-sampled pair with a tighter bound (for example a median within 0.1 px).
  - Optional: the equiangular (V) fit, `delta = (d1 - d3) / (2 max(d1 - d2, d3 - d2))`, suits SAD better than a parabola (Shimizu & Okutomi, IJCV 2005). It is still one IEEE division of integers, so it keeps exact parity.

#### [should-fix] `rectify` accepts a pair whose "right" camera is on the left; disparities then come out negative and `match` returns only false matches
- **Where:** `src/slamlab/foundations/stereo.py:241-247` (`target = copysign(1, t[0])`) and its docstring `:16-19`; mirrors at `cpp/src/foundations/stereo.cpp:253-258` and `rust/slamlab-foundations/src/stereo.rs:126-132`; fixture case `generate_stereo.py:296-300` ("pinhole pair, right camera to the left").
- **Issue:** When `t[0] > 0` after the half-turn, the rectified right camera sits at (-b, 0, 0) in the rectified left frame, not at (+b, 0, 0) as documented. Points then appear `f b / Z` columns further *right* in the right image. `match` only searches 0 < d < f, so it misses every true match.
- **Why it matters:** A mislabelled pair (cam1 passed as left) succeeds without error, and stereo then yields false matches with wrong depths instead of failing.
- **Evidence:** `rect_sign.py` projects a point 3 m ahead through each rectification:
  ```
  t_x=-0.12: disparity u_l-u_r = +20.317  (f b/Z = 20.317)
  t_x=+0.12: disparity u_l-u_r = -19.982  (f b/Z = 19.982)
  ```
  On a pair with true disparity -9.6, `stereo.match` (cpp) returned 12 matches, all false.
- **Suggested fix:** Raise `InvalidInput` when `t[0] > 0` ("the right camera lies left of the left one; swap the cameras"), in all three languages, and turn the fixture case into a rejects case.

#### [should-fix] Keypoint full-image coordinates ignore the aligned-centre resize: up to about 1.9 px off on coarse levels
- **Where:** `src/slamlab/foundations/orb.py:559` (`xy * scale`) and the `Keypoints.xy` docstring `:268-269`; `cpp/src/foundations/orb.cpp:438`; `rust/slamlab-foundations/src/orb.rs:778`.
- **Issue:**
  - The module's own resize samples source `s = (d + 1/2) n_in / n_out - 1/2`, so a level-l pixel `x` lies at `(x + 1/2) n_0 / n_l - 1/2` in the full image.
  - `x * s^l` drops the half-pixel term (0.5(s^l - 1), which is 1.29 px at level 7).
  - It also uses `s^l` where the true ratio is `n_0/n_l` (for example 640/179 = 3.575 against 1.2^7 = 3.583).
  - This is ORB-SLAM's and OpenCV's `pt *= scale`.
- **Why it matters:**
  - It gives a systematic, level-dependent offset in every observation PR 8's PnP, triangulation and BA will consume: about 0.36-0.64 of ORB-SLAM's per-level sigma = s^l at levels 6-7.
  - It also contradicts the design doc's pixel convention, which this PR's resize otherwise honours exactly.
  - Stereo disparities are unaffected to first order, since both sides use the same map.
- **Evidence:** `levelmap.py` composes the resize maps exactly (rationals) for a 640x480, 8-level, 1.2 pyramid. Each line shows the level, the level pixel x, `x*s^l`, the exact full-image coordinate, and the difference:
  ```
  L1  x=513  615.600 vs 616.085  (+0.485)   L3  x=350  604.800 vs 605.770 (+0.970)
  L5  x=237  589.732 vs 590.940  (+1.208)   L6  x=194  579.281 vs 581.182 (+1.901)
  L7  x=19    68.080 vs  69.221  (+1.140)
  ```
- **Suggested fix:** The owner chooses between:
  - Use the exact inverse per axis, `((2x + 1) n_0 - n_l) / (2 n_l)`. That is one IEEE division of exact integers, so it keeps exact parity. The stereo column mapping would become its forward map.
  - Keep ORB-SLAM's convention and record it as a deliberate departure from the pixel convention, with the size of the offset.

#### [should-fix] `runs.evaluate`'s frame-to-frame RPE pairs consecutive *matched* poses, so a tracking gap counts as one step
- **Where:** `src/slamlab/eval/runs.py:352-358`; `metrics.relative_error`'s `delta` in poses, `src/slamlab/eval/metrics.py:144-187`.
- **Issue:** After association, a run that lost tracking (or frames without ground truth) leaves gaps. `relative_error(..., delta=1)` then pairs poses across the gap and reports the whole gap's drift as one step.
- **Why it matters:**
  - Sturm et al. define RPE over a fixed interval Delta.
  - TUM's `evaluate_rpe.py` pairs poses by timestamp (Delta in seconds, default 1 s), so gaps do not pose as one-frame drift.
  - The milestone 2 pipelines will lose tracking.
- **Evidence:** `rpe_gap.py` uses 300 frames at 30 Hz with 2 mm of drift per frame:
  ```
  every frame        : matched 300, RPE trans RMSE   2.00 mm, max    2.0 mm
  lost frames 100-159: matched 240, RPE trans RMSE   8.01 mm, max  119.9 mm
  ```
- **Suggested fix:** Pair by time: Delta in seconds, either TUM's 1 s or the nominal frame period with a tolerance. Skip pairs whose interval does not match, and report Delta's unit.

#### [should-fix] `metrics.associate` claims TUM's `associate.py` but pairs differently, and silently mis-pairs unsorted input
- **Where:** `src/slamlab/eval/metrics.py:190-241`.
- **Issue:**
  - `associate` pairs each time with its nearest reference, then drops losers. TUM's script is a global greedy over all pairs within tolerance, so a time whose nearest reference was taken can still take its second-nearest.
  - TUM uses `<` for the tolerance; this uses `<=`.
  - Neither input is checked to ascend, and `searchsorted` needs that.
- **Evidence:** `assoc.py`:
  ```
  unsorted reference [2,0,1] vs times [0,1,2] (all exact): (array([0, 1]), array([1, 2]))   # time 2.0 lost
  conflict a=[0, .004], b=[.003, .010]: slamlab [(1, 0)]   TUM associate.py [(0, 1), (1, 0)]
  ```
  (`runs.evaluate` validates the ground truth's order, so this bites direct callers.)
- **Suggested fix:** Raise `InvalidInput` unless both arrays ascend. Then either implement TUM's greedy association or drop the claim and document the difference.

#### [note] FAST equals OpenCV only for thresholds of 1 and above
- **Where:** `src/slamlab/foundations/orb.py:649-652` (docstring), `docs/notes/front-end.md:44-45`, `Options` range checks `orb.py:251-259` (0 is accepted).
- **Issue:** OpenCV's suppression compares against a buffer where non-corners score 0, so a corner of score 0 is always suppressed. slamlab compares true scores.
- **Evidence:** `fastcv.py`, TUM fr1/xyz frame 0: exact match at t = 1, 7, 20, 40. At t = 0, slamlab finds 14093 corners against OpenCV's 12940; all 1153 extra are slamlab's.
- **Suggested fix:** Say "for thresholds >= 1", or make the lower bound 1.

#### [note] ORB-SLAM's rotation histogram actually bins at 30 degrees
- **Where:** `src/slamlab/foundations/matching.py:16-21`.
- **Issue:** ORB-SLAM3's `ORBmatcher.cc` has `const float factor = 1.0f/HISTO_LENGTH; … int bin = round(rot*factor);`. That gives 30-degree bins (only bins 0-12 are used, and 0 degrees is split between bins 0 and 12). slamlab's 12-degree bins are clearly what was intended, but they are 2.5 times narrower than what ORB-SLAM runs.
- **Evidence:** `rothist.py` with ground-truth essential matrices on six TUM pairs. Recall of true matches (Sampson < 2 px) is 0.90-1.00 with 12-degree bins and 0.91-1.00 with 30-degree bins, so the effect is small here.
- **Suggested fix:** State the difference in the docstring and the note, and revisit if PR 8's tracking loses inliers under fast rotation.

#### [note] Stereo departures from ORB-SLAM are undocumented, and two branches are unreachable
- **Where:** `src/slamlab/foundations/stereo.py:510-515`, `cpp/src/foundations/stereo.cpp:130-137`, the Rust mirror, and `docs/notes/front-end.md:86-87`.
- **Issue:**
  - `argmin` takes the first minimum, so d1 > d2. That makes the curvature at least 2(d1 - d2) > 0 and |delta| <= 1/2. The "flat valley" and |delta| > 1 rejections can never fire.
  - ORB-SLAM's strict `dist < bestDist` gives the same guarantee, so the note's claim that ORB-SLAM "divides by zero and the NaN passes its range check" is incorrect.
  - Three improvements over ORB-SLAM are not listed among the departures:
    - disparity <= 0 is rejected (ORB-SLAM clamps it to 0.01);
    - the outlier filter keeps matches at equality, and keeps all of them when the median is 0, where ORB-SLAM drops every match;
    - the corrected search-bound check (ORB-SLAM's `iniu = u0 + L - w` is a known slip).
- **Suggested fix:** Keep the branches as documented defensive checks or remove them. Correct the note, and list the stereo departures the way the ORB ones are listed.

#### [note] The exact float paths are safe from FMA contraction only because of how the code is written
- **Where:** `CMakeLists.txt:29` (no `-ffp-contract=off`).
- **Issue:** On x86-64 the baseline ISA has no FMA, so today's parity holds. On aarch64 (milestone 7), GCC contracts aggressively by default outside ISO C modes, and clang's default is `on`.
  - The only candidate path, `disparity = x - s*(c + delta)`, survives today only because `right_x` is also stored.
  - Remap's `x*32 + 0.5` is exact either way.
- **Suggested fix:** Add `-ffp-contract=off` to the core targets. The design doc's exactness argument then becomes a guarantee before the ARM builds.

#### [note] State of the art, for the backlog (context, not defects)
- **Sub-pixel matching:** equiangular fitting for SAD and error cancellation (Shimizu & Okutomi, ICCV 2001; IJCV 63(3), 2005). See the stereo should-fix.
- **Learned and accelerated features**, for the later rungs and the edge-budget comparison:
  - XFeat (Potje et al., CVPR 2024) runs in real time on CPU;
  - SuperPoint (DeTone et al., CVPRW 2018) with LightGlue (Lindenberger et al., ICCV 2023).
- **Trajectory evaluation for later milestones:**
  - Zhang & Scaramuzza (IROS 2018) align visual-inertial runs in 4 DoF (position and yaw), which milestone 6 will need beside Sim(3)/SE(3), and recommend relative error over sub-trajectory lengths.
  - KITTI's official metric (milestone 4) is translational error over 100-800 m segments (Geiger et al., CVPR 2012).

### Checked and fine
- **Gray:** OpenCV's 15-bit RY15/GY15/BY15 weights (9798 + 19235 + 3735 = 2^15), rounded half up; bit-exact against `cvtColor` (test).
- **Blur:** reflect-101 is correct for size 1 and for radius > size. Intermediates are at most 255 * 2^30, in int64. `BLUR_KERNEL` is `getGaussianKernel(7, 2) * 4096`, with the centre 886 absorbing the rounding.
- **Resize:** the aligned-centre position is an exact rational; the carry and both clamps are correct; numerators are below 2^41 and products below 2^31. Level sizes equal ORB-SLAM's for VGA. OpenCV's exact-2x switch to INTER_AREA is equivalent.
- **Remap:** clamping before quantising keeps every cast in range; the zero border is right; the 5-bit weight products are exact.
- **FAST:** the circle equals OpenCV's `offsets16`; the score is cornerScore's (best 9-arc minimum margin, minus 1); suppression is strict 3x3. The departure to whole-level suppression is documented.
- **Orientation:** UMAX reproduces ORB-SLAM's construction. The intensity centroid uses the unblurred level with y pointing down. The bin is chosen by int64 dot products; |m10| * 2^14 is about 4.4e10, so there is no overflow.
- **rBRIEF:** the rotation `(x c - y s, x s + y c)`, `first < second`, and LSB-first packing all equal `computeOrbDescriptor`. The rotated reach of 18 stays under `EDGE_THRESHOLD`. The OpenCV comparison (mean under 1 bit) confirms sign, order and pattern.
- **Detection region and cells:** the region [19, n-19) equals ORB-SLAM's (FAST on [16, n-16) with its 3-px margin), and the cell fallback 20 to 7 matches.
- **Quadtree:** the midpoint ceiling and the selective phase are equivalent to `DistributeOctTree`. `max(1, round(w/h))` fixes ORB-SLAM's division by zero on tall images.
- **Level budgets:** equal ORB-SLAM's [217, 181, 151, 126, 105, 87, 73, 60].
- **Matching:**
  - Hamming distance, the cap of 50, and a strict ratio test (ties fail).
  - Uniqueness keeps the nearest regardless of order, which is better than ORB-SLAM's greedy.
  - `ComputeThreeMaxima`'s 10% rule and first-bin tie-break are reproduced.
  - `count < 0.1*h` agrees with the exact `10*count < h` for every h < 1000 (checked).
  - The C++ and Rust cores validate the angles.
- **Rectification:** R1 = W r_r^T and R2 = W r_r with r_r = Rodrigues(-omega/2), and W from acos(|t_x|/|t|): exactly cvStereoRectify. Rows are epipolar (test), and the maps equal `initUndistortRectifyMap` to 1e-4.
- **Stereo constants:** `thOrbDist` = 75 = (TH_HIGH + TH_LOW)/2, the level window +-1, the 2*s_R band, the (0, f) disparity window, and 2.1 = 1.5 * 1.4.
- **ATE:** Umeyama on camera centres, Sim(3) for monocular and SE(3) otherwise; the source/target order is right.
- **RPE:** follows Sturm et al.'s E_i = (Q_i^-1 Q_{i+Delta})^-1 (P_i^-1 P_{i+Delta}), which is also evo's convention; TUM's script computes P_rel Q_rel^-1, which has the same angle. The RPE is scaled by the Sim(3) scale.
- **EuRoC:**
  - The w-first quaternions are reordered to xyzw.
  - T_world_cam0 = T_world_body T_body_cam0, and T_right_left = T_body_cam1^-1 T_body_cam0.
  - On V1_01 and MH_01, camera stamps coincide with ground-truth stamps to 300 ns, so nearest-pose association is exact (checked).
- **TUM depth:** values over 5000, with 0 read as NaN.
- **Fixtures:** independent oracles (exact rationals for images, brute-force FAST and matching) and hard cases (a kernel wider than the image, one pixel, remap ties and far-outside points, coincident corners, empty sets, a cap at equality, ratio 1). Every random generator is seeded.

### Sources
- [ORB-SLAM3 Frame.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc), [ORB-SLAM3 ORBmatcher.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/ORBmatcher.cc) (Campos et al., T-RO 2021)
- [ORB-SLAM2 Frame.cc](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Frame.cc) (Mur-Artal & Tardós, T-RO 2017)
- Shimizu & Okutomi, "Sub-Pixel Estimation Error Cancellation on Area-Based Matching", IJCV 63(3):207-224, 2005, [doi:10.1007/s11263-005-6878-5](https://unpaywall.org/10.1007%2FS11263-005-6878-5); "Precise Sub-Pixel Estimation on Area-Based Matching", [ICCV 2001](https://mlanthology.org/iccv/2001/shimizu2001iccv-precise)
- Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018, [record](https://infoscience.epfl.ch/record/271842?ln=en), toolbox [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)
- Potje et al., "XFeat: Accelerated Features for Lightweight Image Matching", CVPR 2024, [arXiv:2404.19174](https://arxiv.org/abs/2404.19174)
- DeTone et al., "SuperPoint", CVPRW 2018, [arXiv:1712.07629](https://arxiv.org/abs/1712.07629); Lindenberger et al., "LightGlue", ICCV 2023, [arXiv:2306.13643](https://arxiv.org/abs/2306.13643)
- Geiger, Lenz & Urtasun, "Are we ready for autonomous driving? The KITTI vision benchmark suite", CVPR 2012, [PDF](https://www.cvlibs.net/publications/Geiger2012CVPR.pdf)
