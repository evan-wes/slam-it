# Review panel: `feature/real-time`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-08

| | |
|---|---|
| **Branch** | `feature/real-time` |
| **Range** | `7bccdd7...aeb6121` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; pytest 3069 passed); budget tests (`pytest -m budget`) 13 passed |

Findings that more than one reviewer raised share a row, with all their IDs.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 / L2 | must-fix | hygiene, algorithm | `src/slamlab/eval/budgets.py:257-300` | `judge` accepts repeated, descending or non-finite timestamps and negative or NaN step times; returns a zero/negative deadline and an infinite/negative load | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| A1 | must-fix | architecture | `src/slamlab/eval/budgets.py:51-75` | `Budget` accepts 0 or negative cores and memory; 0 cores pins to every CPU | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| L1 | must-fix | algorithm | `cpp/src/foundations/image.cpp:333-344` | C++ blur calls `subspan` past the row's end when the kernel is wider than the image (UB; aborts under libstdc++ assertions) | fix | fixed (c286113): guarded as Rust's; native tests built with `-D_GLIBCXX_ASSERTIONS`, which abort on the old code -- decided by evan-wes, 2026-10-08 |
| H2 | should-fix | hygiene | `src/slamlab/orchestration/vo.py:603` | Python's program catches only `FileNotFoundError`; other I/O errors exit 1 with a traceback where the natives exit 3 | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| H3 | should-fix | hygiene | `tests/eval/test_budgets.py:35-45` | the peak-memory test cannot fail once the process has run other tests | fix | fixed (1915567): measured in a fresh child process -- decided by evan-wes, 2026-10-08 |
| H4 | should-fix | hygiene | `cpp/src/foundations/image.cpp:360; image.rs:536` | the blur's 64-bit vertical branch (13-15-bit kernels) is untested | fix | fixed (c286113) -- decided by evan-wes, 2026-10-08 |
| H5 / A5 / L4 | should-fix | all three | `rust/slamlab-foundations/src/linalg.rs` | `svd3`'s bit-identity to Eigen has no in-repo test; the note's lesson overstates it (consumers still differ by an ulp) | fix | fixed (22b1319: a golden written by Eigen, held in C++ and Rust; 2b2a686: the lesson reworded) -- decided by evan-wes, 2026-10-08 |
| H6 | should-fix | hygiene | `tests/foundations/test_ba.py; generate_ba.py` | no parity case with anchored points hosted by fixed cameras and observed by free ones; no C++ test of `kept`/`compacted` | fix | fixed (7f59961): the parity test runs again with fixed hosts; the C++ reduction is held through it, as the rest of C++'s BA internals are, not by a native test of `detail` -- decided by evan-wes, 2026-10-08 |
| H7 | should-fix | hygiene | `docs/design/2026-10-02-slamlab-ladder.md:827` | ladder's busiest-window figures (0.50, 0.88) contradict the note's (0.52, 0.92) | fix | fixed (2b2a686) -- decided by evan-wes, 2026-10-08 |
| H8 / A6 / L5 | should-fix | all three | `cpp/src/foundations/ba/common.hpp:17-22; bundle.rs:20` | the stack blocks' column bound restates radtan's 9 intrinsics; a larger model would overflow in C++ release builds | fix | fixed (7f59961) -- decided by evan-wes, 2026-10-08 |
| H9 | should-fix | hygiene | `rust/slamlab-foundations/src/image.rs:370,391,577` | Rust image code hides invariants behind `expect` and a silent `unwrap_or(u8::MAX)` | fix | fixed (c286113) -- decided by evan-wes, 2026-10-08 |
| H10 | should-fix | hygiene | `bundle.cpp:520-531; schur.hpp:291; bundle.rs; schur.rs` | needless work: both Hessian triangles accumulated, `weighted` sized to every coupling row, `kept()` recomputed per linearisation (the lower triangle is read by `hessian_diagonal`, so it needs a mirror, not removal) | fix | fixed (7f59961): the lower triangle is read (`hessian_diagonal`, confined poses), so it is mirrored once, not dropped; whole runs byte-identical -- decided by evan-wes, 2026-10-08 |
| H11 | should-fix | hygiene | `src/slamlab/orchestration/vo.py:219-290,514-520` | readability knots in `_setting`, `parse` and the timing names | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| H12 | should-fix | hygiene | `scripts/run_vo.py:251-269` | `--budget` silently ignored with `--reuse`; 'unknown (an in-process run)' for any missing peak | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| H13 | should-fix | hygiene | `orb.cpp:91; orb.rs:449; real-time.md:106` | 'the circle walked twice round': the walk is once round plus an arc | fix | fixed (c286113, 2b2a686) -- decided by evan-wes, 2026-10-08 |
| A2 | should-fix | architecture | `cpp/bindings/foundations/{orb,stereo}.cpp; image_crossing.hpp:43` | C++ pyramid bindings convert arrays where Rust's (and C++'s image bindings) refuse them | fix | fixed (c286113) -- decided by evan-wes, 2026-10-08 |
| A3 | should-fix | architecture | `src/slamlab/eval/runs.py:18; budgets.py:279-288` | the four step-time keys are a cross-language contract the run format does not state | fix | fixed (1915567): `runs.STEPS`, documented in all three run modules -- decided by evan-wes, 2026-10-08 |
| A4 | should-fix | architecture | `src/slamlab/eval/budgets.py:220-299` | the busiest-window mapping load the docs rely on is not computed by the harness | fix | fixed (1915567): `Frames.mapping_peak_load` -- decided by evan-wes, 2026-10-08 |
| A7 | should-fix | architecture | `frames.hpp:89; frames.rs:150; bindings; test scenes` | every Frame constructor restates the grid; bindings do a core construction step | fix | fixed (1b0d791): `edge::make_frame`, `Frame::new` -- decided by evan-wes, 2026-10-08 |
| A8 | should-fix | architecture | `src/slamlab/eval/runs.py:5-10` | a run does not record the budget it ran within | defer (PR 10) | fixed (1915567): `cpus` and `memory_cap` in run.json version 2 (the owner chose to record it now rather than defer it to PR 10) -- decided by evan-wes, 2026-10-08 |
| A9 | should-fix | architecture | `ladder :22-24,:83,:94,:343,:825; real-time.md:7; CLAUDE.md:37; pyproject.toml:68-72; orchestration/__init__.py` | stale or overclaiming docs: 'every TUM and EuRoC sequence', 'faer for every SVD', BA scope, eval subset, budget marker, Python's program | fix | fixed (2b2a686) -- decided by evan-wes, 2026-10-08 |
| A10 | should-fix | architecture | `scripts/run_vo.py:39-69` | `run_vo.py` keeps a second `--option` parser | fix | fixed (1915567): `vo.parse_setting` -- decided by evan-wes, 2026-10-08 |
| L3 | should-fix | algorithm | `CHANGELOG.md:62-63; real-time.md:113-115` | the '0-bit rounding' Fixed entry describes a bug the base never had | fix | fixed (2b2a686): the entry and the note's sentence removed; commit 2db3289's message keeps the claim, corrected here -- decided by evan-wes, 2026-10-08 |
| A11 | note | architecture | `cpp/src/orchestration/vo.cpp:548; vo.rs:730; vo.py:598` | peak memory read in the native libraries' `run()`, but in Python's `main()` | fix | fixed (1915567) -- decided by evan-wes, 2026-10-08 |
| H14 | note | hygiene | `src/slamlab/eval/budgets.py:288,297` | `Frames.keyframes` excludes the initialising keyframe; name it for what it counts | fix | fixed (1915567): `Frames.mapped` -- decided by evan-wes, 2026-10-08 |
| H15 / L6 | note | hygiene, algorithm | `rust/slamlab-foundations/src/linalg.rs:357` | the Jacobi sweep has no cap (as Eigen) | fix | fixed (22b1319): 64 sweeps -- decided by evan-wes, 2026-10-08 |
| H16 | note | hygiene | `image.cpp:392-421; image.rs:648-676` | `check_pyramid` messages do not name the bad level, as Python's do | fix | fixed (c286113) -- decided by evan-wes, 2026-10-08 |
| H17 | note | hygiene | `slamlab-eval/src/lib.rs:6; ladder run-format bullet` | two over-long prose lines | fix | fixed (2b2a686) -- decided by evan-wes, 2026-10-08 |
| L7 | note | algorithm | `cpp/src/edge/mapping.cpp:866-871` | keyframe creation is timed as mapping; ORB-SLAM2 does it in tracking | fix (note says so) | fixed (1915567, 2b2a686): stated in the orchestration docstring and the note -- decided by evan-wes, 2026-10-08 |
| L8 | note | algorithm | `src/slamlab/eval/budgets.py:299` | a FIFO backlog (Lindley's recursion) is the 'thread of its own' measure | defer (milestone 7) | deferred (ladder, "Deferred from PR 9's review panel", milestone 7) -- decided by evan-wes, 2026-10-08 |
| L9 | note | algorithm | `docs/notes/real-time.md` | the verification evidence lives outside the repo; the point metric is relative to max(1, |x|) | fix (note says so) | fixed (2b2a686) -- decided by evan-wes, 2026-10-08 |
| L10 | note | algorithm | `bundle.cpp:528,542; bundle.rs` | the host's own observation of an anchored point is analytically null; pairwise products leave eps noise | defer (ladder backlog) | deferred (ladder, "Deferred from PR 9's review panel") -- decided by evan-wes, 2026-10-08 |
| L11 | note | algorithm | `--` | context: the rewrite is the standard reduced camera system (Triggs, g2o); ORB-SLAM's timings support the note | none needed | no action: context -- decided by evan-wes, 2026-10-08 |
| L12 | note | algorithm | `src/slamlab/eval/budgets.py:89` | a budget's cores are logical CPUs; SMT machines need topology | defer (milestone 7) | deferred (ladder, "Deferred from PR 9's review panel", milestone 7) -- decided by evan-wes, 2026-10-08 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 7bccdd7...aeb6121` on `feature/real-time` (76 files, +5094/-1073). I read every changed source file in Python, C++ and Rust, plus the tests, the real-time note, the CHANGELOG, the ladder doc and the CLAUDE.md edits. REPO is the repository root. I ran a few read-only probes against the Python code and the built `slamlab_vo` binary. Layer-1 items (format, lint, docstring presence, duplication) are not re-reported.
**Summary:** This is careful, well-documented work. All three languages carry full doc comments, the run-format change is validated on read and write in every language, and the front-end speed-ups are covered by exact fixtures. One production-quality gap needs fixing before merge: `budgets.judge`, the harness's verdict function, accepts input that makes it report nonsense. The rest are should-fix items. Most are test gaps for the new claims (the 64-bit blur branch, "bit-identical to Eigen", the new kept-coordinate BA paths), plus one doc contradiction and some small clarity and efficiency items.

### Findings

#### [must-fix] `budgets.judge` accepts timestamps and step times it cannot judge, and returns a silent false verdict
- **Where:** `src/slamlab/eval/budgets.py:257-300` (validation is only on lines 275-284).
- **Issue:** `judge` checks the frame count and that the keys exist. It never checks that the timestamps are finite and strictly ascending, or that the step times are finite and non-negative. A repeated or descending timestamp gives a zero or negative `deadline`. `mapping_load` then divides by `times[-1] - times[0] + deadline`, which can be zero. A NaN step time makes `latency > deadline` false, so the frame is counted on time.
- **Why it matters:** `judge` is the pass/fail verdict of the real-time harness. `runs.read_run` only checks that each diagnostic has a `timestamp`, so a corrupt run on disk (a reader bug, a bad timer) is judged without complaint. CLAUDE.md asks that failure be observable.
- **Evidence:** I built runs with `Run(...)` and called `budgets.judge`:
  - Timestamps `[1,1,1]`, mapped frames: `deadline=0.0, late=3, mapping_load=inf`, with a numpy "divide by zero" RuntimeWarning.
  - Same timestamps, no mapped frames: `mapping_load=nan`.
  - Timestamps `[3,2,1]`: `deadline=-1.0, mapping_load=-0.1`.
  - `build_seconds=NaN` on every frame: `late=0`, `mapping_load=0.0`, no error. A negative `build_seconds` is also accepted.
  - `tests/eval/test_budgets.py` tests only the one-frame case and the missing-key case.
- **Suggested fix:** Raise `InvalidInputError` naming the frame index when a timestamp is non-finite or not strictly increasing, or when a step time is non-finite or negative. Add one test per case.

#### [should-fix] Python's program handles only `FileNotFoundError`, so other I/O errors traceback with exit 1 where the natives exit 3
- **Where:** `src/slamlab/orchestration/vo.py:603` (`except FileNotFoundError`); the `main` docstring at 579-597 and the module docstring promise "the same command line, exit codes and messages".
- **Issue:** `main` maps `FileNotFoundError` and `SlamlabError` to `EXIT_FAILED`. Any other `OSError` (an unwritable `--out`, for one) escapes as a traceback with exit 1.
- **Why it matters:** `_run_program` then reports it as "failed (exit code 1)". A budget run therefore sees a different failure kind per backend.
- **Evidence:** `python -m slamlab.orchestration.vo --dataset tum --sequence tests/fixtures/eval/sequences/tum_camera --mode rgbd --out /root/run --count 0` ends in `PermissionError: [Errno 13] ...` with a full traceback. The built native `slamlab_vo` with the same arguments prints `slamlab_vo: /root/run: Permission denied` and exits 3. The new test `test_pythons_program_fails_a_missing_file_as_the_natives_do` covers only the missing-file case.
- **Suggested fix:** Catch `OSError` rather than `FileNotFoundError`, and add a parametrised case to that test.

#### [should-fix] `test_peak_memory_grows_with_what_the_process_holds` cannot fail once the process has run other tests
- **Where:** `tests/eval/test_budgets.py:35-45`.
- **Issue:** The docstring says holding 64 MiB "raises this process's peak by at least that much". The assertions are `after >= before` (always true) and `after >= len(held)` (true whenever the process already peaks above 64 MiB). In the full suite, earlier tests have raised the high-water mark far above that.
- **Why it matters:** The test passes if `peak_memory` returns any constant. It is the only test that reads the real `/proc/self/status` through the Python function.
- **Evidence:** In a fresh interpreter it works: before 39 MiB, after 103 MiB. After an earlier 300 MiB allocation was freed: `before 331 after 331 | the test's asserts pass: True | grew by 0 MiB`.
- **Suggested fix:** Run the measurement in a child process (`python -c`, which prints the peak before and after a 64 MiB touch) so the baseline is clean. Alternatively, delete the test; `test_every_program_records_its_peak_memory` already asserts `peak > 2**20` per program.

#### [should-fix] The 64-bit branch of the new blur has no test in any language
- **Where:** C++ `cpp/src/foundations/image.cpp:360-363`; Rust `rust/slamlab-foundations/src/image.rs:536-540`. Fixture cases are in `tests/fixtures/foundations/generate_image.py:185-203`.
- **Issue:** The blur now picks 32-bit vertical sums when `2*bits + 8 <= 32` and 64-bit above that. Every fixture and ORB kernel is at most 12 bits (the fixture kernels sum to 16, 64, 4, 4, 1 and 8; ORB's is 12-bit). Kernels of 13 to 15 bits (`MAX_KERNEL_BITS` is 15) therefore never run `blur_columns::<u64>` or `<std::uint64_t>`. The branch condition itself, with its `+ 8` margin, is also untested.
- **Why it matters:** An off-by-one in the width threshold would overflow silently, and the PR claims every pixel stays exact.
- **Suggested fix:** Add a fixture case with a 13-to-15-bit kernel (for example `[1, 2**15 - 2, 1]`) on a noise image and on an all-255 image. The all-255 case is the overflow trap; its expected output is 255.

#### [should-fix] "Rust's `svd3` is bit-identical to Eigen" has no committed oracle
- **Where:** `rust/slamlab-foundations/src/linalg.rs:417-433`; tests at 574-615; claims in `docs/notes/real-time.md` ("on 39,005 test matrices its U, σ and V equal Eigen's exactly") and the CHANGELOG ("C++'s factors to the bit").
- **Issue:** The 39,005-matrix check ran outside the repo. The committed tests are the old accuracy test plus `svd3_orders_and_signs_as_eigen`, which checks three hand-made matrices. The downstream so3 and alignment fixtures use tolerances, not bit equality.
- **Why it matters:** The note justifies porting the algorithm "not just the contract" so that decisions hanging on roundoff agree across languages. Nothing in the repo would fail if the port drifted by an ulp. The room's golden record would catch it only if a decision flipped.
- **Suggested fix:** Generate a small golden (about 20 matrices, including graded, rank-deficient and negative-diagonal cases) from `Eigen::JacobiSVD`, with the bit patterns of U, σ and V. Check it with `assert_eq!` in Rust and in a C++ test.

#### [should-fix] The BA rewrite's new "drop the unread coordinates" paths are only partly covered in the repo
- **Where:** `cpp/src/foundations/ba/bundle.cpp:429-478, 513-545`; `rust/slamlab-foundations/src/ba/bundle.rs` (`lay_out_blocks`, `accumulate`); `tests/foundations/test_ba.py:450-490`; `tests/fixtures/foundations/generate_ba.py:180-250`.
- **Issue:** The parity case (`_every_kind`) and the fixture's `_every_kind` never set `fixed_poses`. All poses are free or confined, so dropped columns arise only from fixed intrinsics and affines. The only fixture with fixed poses (`_explicit_none`) has no anchored points. Rung 0's new `column < 0` guards therefore never see an anchored point whose host is fixed while its observer is free. The edge rung does not exercise it either; it uses fixed poses but no anchors. The 3,136-problem check (per solve, in the note) lives outside the repo. Rust has a unit test for `Reduction::kept` and `compacted` (`schur.rs:450-474`); C++ has none.
- **Why it matters:** This is the exact structure the speed-up introduced, and a future user of rung 0 (milestone 3 onward) could hit it.
- **Suggested fix:** Add one compiled-vs-reference parity case with several fixed poses, anchored points hosted by both fixed and free cameras, and a host that is the observer. Optionally add a small C++ test of `Reduction::kept` and `compacted` mirroring Rust's.

#### [should-fix] The ladder doc's "after" numbers contradict the note and the CHANGELOG
- **Where:** `docs/design/2026-10-02-slamlab-ladder.md:827`.
- **Issue:** It says "busiest 10 seconds ... (MH_01 stereo 1.15 to 0.50, fr3/long_office monocular 2.28 to 0.88, in C++)". `docs/notes/real-time.md` ("Mapping's load" table) gives 1.15 / 0.52 and 2.28 / 0.92 for C++. The CHANGELOG says 0.92 for the busiest 10 seconds. The 0.50 in the ladder doc is the note's whole-run load before the speed-ups, not an after figure.
- **Why it matters:** Three documents give two sets of numbers for the PR's headline claim.
- **Suggested fix:** Use the note's 0.52 and 0.92, or point to the note and drop the figures.

#### [should-fix] C++ hard-codes the block-size bounds that Rust derives
- **Where:** `cpp/src/foundations/ba/common.hpp:17-22`, against `rust/slamlab-foundations/src/ba/bundle.rs:20-23`.
- **Issue:** `kMaxIntrinsics = 9` and `kMaxBlockColumns = 2 * (6 + kMaxIntrinsics)` use literals (the camera models and `kPoseDim` already exist). Rust uses `Model::RadTan.intrinsics_len()` and `POSE_DIM`.
- **Why it matters:** The fixed-bound Eigen types only assert in debug builds. A tenth intrinsic added to a camera model would overflow the stack block silently in release builds in C++. In Rust it would be a bounds-check panic.
- **Suggested fix:** Derive the C++ bound from `camera.hpp` (a `constexpr` model intrinsic count) and `kPoseDim`, or add a `static_assert` tying them together.

#### [should-fix] Rust's image code hides invariants behind `expect` and `unwrap_or(u8::MAX)`
- **Where:** `rust/slamlab-foundations/src/image.rs:370, 391, 577-579`.
- **Issue:** `i32::try_from(sample.fraction).expect("an 11-bit fraction")` re-derives a bound the type could carry. `u8::try_from(...).unwrap_or(u8::MAX)` saturates silently, with a comment that the fallback "never applies"; if it ever did, it would hide a wrong image. The generic `blur_columns` needs six trait bounds plus `u8: TryFrom<T>` only to support that conversion. C++ uses a plain `static_cast`.
- **Why it matters:** These are exactly the library-code patterns the project's standard warns about.
- **Suggested fix:**
  - Store `AxisSample.fraction` as `i32` (it is at most 2^11).
  - Use `debug_assert!` plus a plain narrowing for the final pixel, or write a small helper `fn byte(sum) -> u8` with `debug_assert!(sum <= 255)`.
  - Consider passing the width as a const generic or an enum instead of the trait soup.

#### [should-fix] Needless work in the new Schur path
- **Where:**
  - `cpp/src/foundations/ba/bundle.cpp:520-531` and `rust/slamlab-foundations/src/ba/bundle.rs:932-941`: `accumulate` forms both triangles of `hessian_cc`.
  - `cpp/src/foundations/ba/schur.hpp:310` and `rust/slamlab-foundations/src/ba/schur.rs:365`: elimination overwrites the strict lower triangle from the upper.
  - `schur.hpp:291` and `schur.rs:350`: `weighted` is allocated with the total number of coupling rows per `eliminate` call, though only a point's own rows are ever used (`topRows(count)`).
  - `bundle.cpp` `linearize` and `bundle.rs:1016`: `reduction(state).compacted()` re-runs `kept()` (clone, sort, dedup) on every linearisation, though `lay_out_blocks` already computed it once and the free and confined lists never change.
- **Issue:** The lower triangle of `hessian_cc` is dead work, so about half the dot products in the loop that the note calls half of mapping's time are wasted. The `weighted` buffer scales with the problem; the largest per-point count would do. The `kept()` result is recomputed per call.
- **Why it matters:** The PR's goal is speed, and the stated rule is no needless work in hot loops. Restricting the accumulation to `ci <= cj` leaves the upper triangle bit-identical.
- **Suggested fix:** Accumulate only `ci <= cj`. Size `weighted` by the largest per-point count (it can live in `CouplingLayout`). Cache the compacted free and confined lists (only the basis is state-dependent).

#### [should-fix] Python `vo.py` has small readability knots
- **Where:** `src/slamlab/orchestration/vo.py:219-224, 228-290, 514-520`.
- **Issue:**
  - `_setting` rejects `1_0` with `float("nan" if "_" in number else number)` plus a second `float("nan")` fallback in the `except`, then an `isfinite` test. That is clever where a regex like `_integer`'s would say it directly.
  - `parse` calls `_integer(flag, value)` for `--first` and `--count` only to validate, discards the result, then calls `int(...)` on them again. Validation is interleaved with the loop.
  - In `_run_here`, `read` and `built` are instants while the diagnostics keys are durations; Rust's `track` uses the same names for elapsed offsets, C++'s for instants.
  - C++ `record(..., double build)` and Rust `record(..., build: f64)` shadow the free function `build`.
- **Suggested fix:** Use a regex for the float. Parse each value once. Name the instants `after_read` and `after_build`, and the record arguments `read_seconds` and `build_seconds`.

#### [should-fix] `run_vo.py` silently ignores `--budget` with `--reuse`
- **Where:** `scripts/run_vo.py:251-269`.
- **Issue:** With `--reuse` the run is only read, so `--budget` (like `--first`, `--count` and `--option`) does nothing, and the script prints no "within ..." line. Separately, a run with no recorded peak prints "unknown (an in-process run)" (`run_vo.py`, `summarize`), which is wrong for a native run on a system with no `VmHWM`. The budget banner is printed after the run, not before.
- **Suggested fix:** `parser.error` when `--reuse` is combined with `--budget`. Say "not recorded" for a `None` peak.

#### [should-fix] The "circle walked twice round" comment is inaccurate
- **Where:** `cpp/src/foundations/orb.cpp:91`; `rust/slamlab-foundations/src/orb.rs:449`; `docs/notes/real-time.md:106`.
- **Issue:** The loop runs `circle.size() + kFastArc` = 25 steps, once round plus one arc, not twice round (32 steps). `arc_score`'s "twice round" (a doubled array) is correct.
- **Why it matters:** A reader checking the loop bound against the comment will suspect a bug.
- **Suggested fix:** Say "once round, plus an arc, so every wrap-around run is counted".

#### [note] `Frames.keyframes` counts frames with `map_seconds > 0`, which excludes the initialising keyframe
- **Where:** `src/slamlab/eval/budgets.py:288, 297`; `scripts/run_vo.py` `summarize`.
- **Issue:** The initialising keyframe's mapping time sits in `track_seconds`. The attribute `keyframes` is documented as "the frames mapped as keyframes", but the summary prints "mapping N keyframes" right under a line whose "keyframes" counts the map's. The test documents the off-by-one, but the name does not.
- **Suggested fix:** Rename it `mapped` or `mapped_keyframes`, or say "excluding the one that starts the map" in the docstring.

#### [note] `jacobi_svd3` loops until convergence with no sweep cap
- **Where:** `rust/slamlab-foundations/src/linalg.rs` (`jacobi_svd3`, the `while !finished` loop).
- **Issue:** This matches Eigen and converges quadratically, and the input is checked finite. A defensive cap of about 30 sweeps returning `None` would make a pathological non-termination observable; that is the library-code standard.

#### [note] `check_pyramid` messages differ from `as_pyramid`'s
- **Where:** `cpp/src/foundations/image.cpp:392-421`; `rust/slamlab-foundations/src/image.rs:648-676`; `src/slamlab/foundations/image.py:287-333`.
- **Issue:** Python names the bad level (`levels[1]`) when a level is malformed. C++ and Rust use the bare name for every level. Python validates first, so only direct native callers see the difference.

#### [note] Two over-long prose lines
- **Where:** `rust/slamlab-eval/src/lib.rs:6` (120 columns) and `docs/design/2026-10-02-slamlab-ladder.md` (the run-format bullet, about 124 columns). rustfmt does not reflow doc comments.

### Checked and fine
- Privacy rule: no rig-bag topic names, frame ids, intrinsics, rates, sizes, file names, dataset paths or results anywhere in the diff. The note and CHANGELOG name only public TUM and EuRoC sequences and the dev machine's CPU model.
- Doc comments in all three languages: new items have `@brief`/`@param`/`@return`, rustdoc `# Arguments`/`# Errors`, and numpy-style docstrings. Frame conventions and units are stated (seconds, bytes, MiB, metres).
- Run format version 2 (`peak_memory`): validated on write and read in Python, C++ and Rust (non-integers, negatives, bool and a missing key are rejected). The golden fixtures are regenerated, and there is no shim for version 1, as the no-shim rule requires.
- Per-step timing: Python, C++ and Rust compute `track_seconds = total - map_seconds` consistently. `test_every_frame_times_its_steps` pins the initialising-keyframe convention.
- Blur and resize index arithmetic: I checked the interior/border split and the 32-bit sum bounds by hand (255·2^15 and 255·2^22 both fit).
- `stereo.match` / `orb.extract_levels` rewiring: all callers updated (Python, bindings, edge frames in C++ and Rust, tests). Rejection tests cover count, shape, type and non-sequence. No stale `seconds` key readers remain.
- Frame grid: `check_frame` still validates `grid.size()` against the keypoint count, so a grid of the wrong size is caught. The bindings build the grid in `frame_in`, and the change is documented.
- Budget helpers: `cpus` and `command` have unit tests that assert values. The killed-program path has a layer-1 test (`kill -9 $$`) and a budgeted test.
- No debug prints (only CLI output), no TODOs, no swallowed exceptions, no hard-coded paths. VERSION and CHANGELOG are consistent (0.3.3.0 for a feature batch).
- BA and ORB parity: the existing parity, oracle and golden tests (`test_backend_parity`, the ba/orb fixtures, the room's golden record) cover the rewritten BA, FAST and blur on all three backends, apart from the gaps listed above.

Housekeeping, so nothing is a surprise: I overwrote three scratch files (`probe.py`, `probe2.py`, `probe3.py`) in the session scratchpad, which held earlier throwaway probes, and removed an empty directory I had created there. Importing `slamlab.backends` once triggered the editable install's incremental build output under the git-ignored `build/`. `git status` in the repo was clean before and after.

### Architecture review

## Architecture review

**Scope:** `git -C REPO diff 7bccdd7...aeb6121` on `feature/real-time` (76 files, 11 commits). I read the following in full at `aeb6121`:
- the harness: `src/slamlab/eval/budgets.py`, the C++ and Rust `eval::budgets`, and the run-format changes in all three languages;
- the programs: the three `vo` programs and `scripts/run_vo.py`;
- the edge rung: the `edge.frames`/`edge.pipeline` changes in all three languages;
- the foundations: the `orb.extract_levels`, `image.as_pyramid`/`check_pyramid` and `stereo.match` changes; the C++ and Rust BA rewrite (`ba/schur.hpp`, `ba/bundle.cpp`, `common.hpp`, `schur.rs`, `bundle.rs`); `linalg.rs`;
- the bindings; the tests and fixtures; the ladder, CLAUDE.md, `pyproject.toml`, `docs/notes/real-time.md` and the related lines of `edge-vo.md`, `front-end.md` and `rust-port.md`.

Probes were run read-only in the `slamlab` env. Importing `slamlab._cpp` triggered the editable install's rebuild into the git-ignored `build/`; `git status` stayed clean.

**Summary:** Structurally the slice is sound:
- **Dependency direction holds:** `eval.budgets` sits on top of eval's internal order, the programs depend on it, and no rung does.
- **The three languages stay in lock-step:** every interface change (pyramid reuse, step timing, the Delivered/Reader/build split, the BA rewrite) lands in the same shape in all three.
- **The harness is placed as decided.**

The findings concern:
- one public type of the harness that does not validate itself (must-fix);
- a binding-boundary inconsistency in the new pyramid bindings;
- three contracts the docs rely on that live only in prose or in out-of-repo scripts;
- a few doc statements the PR made stale or overstates.

### Findings

#### [must-fix] A `Budget` of 0 or negative cores or memory is accepted; a 0-core budget pins to every CPU
- **Where:** `src/slamlab/eval/budgets.py:51-75` (`Budget`, no checks), `:122`, `:151-156`. Consumed by `src/slamlab/orchestration/vo.py:324,560` and `scripts/run_vo.py:250`.
- **Issue:** `Budget` is a public frozen dataclass with no validation. `cpus` returns `sorted(offered)[-budget.cores :]`:
  - for `cores=0` this is the whole list (`-0 == 0`);
  - for `cores=-2` it drops the two lowest CPUs;
  - `command` then writes `MemoryMax=-1`.

  Both contradict `cpus`'s documented return ("`budget.cores` CPUs").
- **Why it matters:** the budget is the harness's contract. A mistyped budget runs unbudgeted, but the run is still reported "within" it. Every other settings dataclass in the project checks itself when it is made (`edge.options.Options`, `orb.Options`; "The options checked themselves when they were made", `vo.py:186-187`). This overlaps the hygiene reviewer's validation lane; I raise it because it is the harness's public type.
- **Evidence:**
  ```
  Budget('zero', cores=0, memory=2**30, ...) -> cpus: [0, 1, 2, 3, 4, 5, 6, 7]
  Budget('neg', cores=-2, memory=-1, ...)    -> cpus: [2, 3, 4, 5, 6, 7]; command: [..., '-p', 'MemoryMax=-1', ...]
  ```
- **Suggested fix:** add a `__post_init__` that requires a non-empty name, `cores` an int ≥ 1 and `memory` an int ≥ 1, raising `InvalidInputError`, plus one test.

#### [should-fix] The C++ pyramid bindings silently convert arrays; Rust's, and C++'s own image bindings, refuse them
- **Where:**
  - `cpp/bindings/foundations/orb.cpp:131` (`nb::arg("levels")`);
  - `cpp/bindings/foundations/stereo.cpp:152` (`left_levels`, `right_levels`);
  - `cpp/bindings/foundations/image_crossing.hpp:43` (`levels_in`);
  - compare `orb.cpp:129` (`nb::arg("image").noconvert()`) and `rust/slamlab-py/src/foundations/orb.rs` `extract_levels`.
- **Issue:** the list-of-images arguments have no `.noconvert()`, so nanobind converts each level: int64, float64 and non-contiguous arrays are accepted and copied to uint8. The Rust binding raises `TypeError` for the same input, as do C++'s single-image bindings.
- **Why it matters:** the binding boundary ("uint8 for images") is meant to hold the same way in both compiled backends. The Python entry points validate first (`as_pyramid`), so only direct `_cpp` callers are exposed, but C++ and Rust now fail differently on one input.
- **Evidence:** `_cpp.foundations.orb.extract_levels([int64 10x10 filled with 300, int64 8x8], opts)` → accepted. The same with float64 → accepted. `slamlab_rs.foundations.orb.extract_levels(<int64 levels>, opts)` → `TypeError`. `_cpp...orb.extract(int64 image, opts)` → `TypeError`.
- **Suggested fix:** add `.noconvert()` on the three list arguments, after checking that nanobind's list caster passes the flag to its elements; otherwise take an `nb::sequence` and cast each item without conversion. Add one test that passes int64 levels to both compiled modules directly.

#### [should-fix] The per-frame step times are a cross-language contract that the run format does not state
- **Where:**
  - `src/slamlab/eval/runs.py:18-19` (diagnostics "free-form but each with a `timestamp`");
  - `src/slamlab/eval/budgets.py:279-288` (`judge` requires `{read,build,track,map}_seconds`);
  - `src/slamlab/orchestration/vo.py:20-30` (the keys defined);
  - the writers: `cpp/src/orchestration/vo.cpp` `record`, `rust/slamlab-orchestration/src/vo.rs` `record`.
- **Issue:**
  - Three programs write four timing keys, and `eval.budgets.judge` refuses a run without them. Yet the run format (version 2) still calls diagnostics free-form, and the keys' meaning is defined only in the orchestration docstring: a layer above the module that reads them.
  - `judge` also counts a frame as "mapped" by `map_seconds > 0.0` (`budgets.py:288`). That rule is implied by the tests (`tests/orchestration/test_vo.py` `test_every_frame_times_its_steps`), not stated.
- **Why it matters:**
  - A writer in any language that drops or renames a key passes `write_run` and fails only in `judge`.
  - The run viewer (milestone 10) will read the same folder.
  - When dense or learned rungs write runs, `judge`'s ORB-SLAM thread split (build+track vs map) will need to be explicit rather than assumed.
- **Suggested fix:** state the four keys in the run format's documentation (`runs.py`, `runs.hpp`, `runs.rs`) as part of version 2: units, initialisation counted in `track_seconds`, and `map_seconds` being 0 exactly for frames not mapped. Define their names once (e.g. `budgets.STEPS`, which the test's `STEPS` duplicates) and have the orchestration docstrings point there. Say in `judge`'s docstring that it judges the edge pipeline's split.

#### [should-fix] The mapping-burst metric the docs rely on is not computed by the harness
- **Where:** `docs/notes/real-time.md:94-96,226-238`; ladder `docs/design/2026-10-02-slamlab-ladder.md:826`; `src/slamlab/eval/budgets.py:220-299` (`Frames` has only the whole-run `mapping_load`).
- **Issue:**
  - The note justifies the BA rewrite by the busiest 10 seconds' mapping load (1.15 → 0.52, 2.28 → 0.92).
  - It claims "every run's busiest window stays below 1", and the ladder repeats that.
  - Neither `judge` nor `run_vo.py --budget` computes a windowed load, and no code in the repo does (`git grep -i busiest` matches docs only).
- **Why it matters:** rule 1 says mapping is "judged by its load", and the burst is where the note found the failure. A regression in burst load is invisible to the harness the note documents, and the published numbers cannot be reproduced from the repo.
- **Suggested fix (recommend code):** add `mapping_peak_load` to `Frames`: the largest load over a sliding window, with the window a `judge` parameter defaulting to 10 s. Print it in `run_vo.py`. Otherwise, say in the note that it was computed outside the harness.

#### [should-fix] Rust's 3x3 SVD is documented as bit-identical to Eigen's, but no test holds it
- **Where:**
  - code: `rust/slamlab-foundations/src/linalg.rs:407-433`; its tests at `:573-616` check ordering, signs and accuracy, not equality with Eigen;
  - docs: `docs/notes/real-time.md:158-163,272-275`; ladder `:942-944`.
- **Issue:** the docs say Rust's 3x3 SVDs now give Eigen's factors to the bit, and that this matters because decisions at near-ties hang on roundoff. The evidence is a 39,005-matrix probe kept outside the repo.
- **Why it matters:** an edit to `jacobi_svd3`, or a conda-forge Eigen update, would break the cross-language property silently. The room's golden record catches it only if a decision on the synthetic room moves.
- **Suggested fix:** add a cross-backend test asserting exact equality of `cpp` and `rust` on the 3x3 consumers (`so3.project_to_so3`, `alignment.umeyama`) over a few hundred graded, rank-1/2, rotation-like and extreme-scale matrices. It uses existing bindings and needs no new fixture format.

#### [should-fix] The BA stack-block bounds restate the camera models' largest intrinsics count
- **Where:** `cpp/src/foundations/ba/common.hpp:16-20` (`kMaxIntrinsics = 9`, `kMaxBlockColumns = 2 * (6 + kMaxIntrinsics)`); `rust/slamlab-foundations/src/ba/bundle.rs:19-23` (`MAX_INTRINSICS = Model::RadTan.intrinsics_len()`).
- **Issue:** both languages bound the stack Jacobian by "radtan is the largest model", a fact that belongs to `camera`, and nothing checks it at compile time. C++ has no constexpr intrinsics length at all: `camera.cpp:135` holds it in a runtime check. The literal `6` stands for `kPoseDim`.
- **Why it matters:** a camera model with more parameters (OpenCV's rational model has 14) would compile.
  - In C++ release builds, Eigen's `MaxColsAtCompileTime` assert is compiled out, so `conservativeResize` past the bound writes beyond the stack buffer.
  - Rust would panic instead.

  The same input would then fail differently in each language, and neither at build time.
- **Suggested fix:** add `constexpr Index intrinsics_size(Model)` to `camera.hpp`, mirroring Rust's `const fn intrinsics_len`. Take the maximum over all models in both bounds, with a `static_assert` or const assertion, and use `kPoseDim`.

#### [should-fix] Every C++ and Rust `Frame` constructor restates the feature grid, the bindings included
- **Where:**
  - the field: `cpp/include/slamlab/edge/frames.hpp:89`, `rust/slamlab-edge/src/frames.rs:150`;
  - the bindings: `cpp/bindings/edge/edge_crossing.hpp:330`, `rust/slamlab-py/src/edge/edge_crossing.rs:447`;
  - the test helpers: `cpp/tests/edge/scene.hpp:92`, `cpp/tests/edge/test_map.cpp:48`, `rust/slamlab-edge/tests/scene/mod.rs:119`, `rust/slamlab-edge/tests/pipeline.rs:99`, `rust/slamlab-edge/src/map.rs:1087`.
- **Issue:** `grid` is a public field derived from `features.keypoints`. Its invariant lives in the private `assemble` (`frames.cpp:25`, `frames.rs:261`) and is restated by hand in two bindings and five test helpers. The bindings now perform a core construction step rather than only converting types (CLAUDE.md: "Bindings only convert types").
- **Why it matters:** `check_frame` catches a grid of the wrong size, but not a stale grid of the right size (keypoints edited after construction), which would search the wrong positions silently. Every future frame producer (milestone 7's mapping thread, a replay tool) must remember the rule.
- **Suggested fix:** make one public core constructor (e.g. `edge::make_frame(timestamp, features, bearings, depth, right)` / `Frame::new`) that builds the grid, and route `assemble`, both bindings' `frame_in` and the test scenes through it. Alternatively, make `grid` private behind an accessor.

#### [should-fix] A run does not record the budget it ran within
- **Where:** `src/slamlab/eval/runs.py:5-10` (`run.json` keys); `src/slamlab/orchestration/vo.py:555-560`; `scripts/run_vo.py:266-268` (the budget is printed, then lost).
- **Issue:** version 2 records the program's peak memory but not whether it was pinned or capped, or how. A budgeted and an unbudgeted run folder cannot be told apart.
- **Why it matters:** the design doc's Metrics are "Cost, per budget", with a headline chart of "one point per rung, language and budget". The run viewer (PR 10) reads only the folder. The format changed version in this PR; recording this now avoids a version 3.
- **Suggested fix:** each program records what it actually ran under, as it records its peak memory: its CPU affinity (`sched_getaffinity`) and its cgroup's `memory.max` (via `/proc/self/cgroup`). That is truthful and needs no flag. Alternatively, defer it explicitly to PR 10 in the ladder.

#### [should-fix] Docs and config comments the PR made stale, and one overclaim
- **Where and what (recommend changing the docs; the code is right):**
  - **Overclaim:** ladder `:22-24` and `docs/notes/real-time.md:7-8` say "every TUM and EuRoC sequence". The results cover five sequences, ten sequence-mode pairs (the edge note's set, `edge-vo.md:219-230`), not MH_04/05, V1_03, V2_03 and so on. Say "every sequence of the edge note's set".
  - **faer:** ladder `:94` "faer for every SVD" is no longer true; the 3x3 SVDs are the Jacobi port. `docs/notes/rust-port.md:46-47,286` say the same and need a forward pointer to the real-time note.
  - **BA scope:** ladder `:825` "Rung 0's BA now forms only the coordinates it solves" holds for C++ and Rust only; the Python reference is unchanged, as the note says (`real-time.md:134`).
  - **Rust subset:** ladder `:83-84,343` and CLAUDE.md `:37-38` describe the native eval libraries as "readers and run writer". They now also hold `budgets` (peak memory), and the Rust-subset rule should list it.
  - **pyproject:** `pyproject.toml:68-72` says "GPU and budget-capped runs get their own opt-in scripts when those rungs arrive", and the `budget` marker says "(edge rungs, milestone 7)". Both predate PR 9's `python -m pytest -m budget`.
  - **orchestration docstring:** `src/slamlab/orchestration/__init__.py:5-6` says "in Python or a native program"; Python now has a program of its own too.
- **Why it matters:** the design doc is the authority on the language split, and these lines now contradict the code.
- **Evidence:** `git grep -n "faer for every SVD\|every TUM and EuRoC" aeb6121 -- docs` gives ladder `:24,:94` and `real-time.md:8`. `budgets.py` and `linalg.rs:1-16` as quoted.
- **Suggested fix:** change the docs as listed.

#### [should-fix] `run_vo.py` keeps a second parser for `--option KEY=VALUE`
- **Where:** `scripts/run_vo.py:39-69` vs `src/slamlab/orchestration/vo.py:208-225`.
- **Issue:** PR 9 made `vo.parse`/`_setting` the canonical reader of the flag, held to the native programs' rules. The script still converts with `kind(value)`, so it accepts `window=1_000`, which every program refuses.
- **Why it matters:** two readers of one flag drift apart, and the script is the main CLI for budgeted runs.
- **Suggested fix:** make `_setting` public (e.g. `vo.parse_setting`) and use it in `run_vo.py`.

#### [note] Peak memory is recorded in the native libraries' `run()` but in Python's `main()`
- **Where:** `cpp/src/orchestration/vo.cpp:548`, `rust/slamlab-orchestration/src/vo.rs:730`, `src/slamlab/orchestration/vo.py:598`.
- **Issue:** in C++ and Rust, `vo::run` is a library function that native tests also call in-process (`cpp/tests/orchestration/test_vo.cpp:122`, `vo.rs:892`), so those runs record the test process's peak. Python measures only in the program's `main`, as `runs.py` states ("null when the run was not a program's of its own"). It is harmless today, since only the programs call `run` in production. But the rule is stated per program and implemented per library in two of the three languages.
- **Suggested fix:** move the measurement to the programs' `main` (`slamlab_vo.cpp`, `main.rs`), or document that the native `run()` is the program body.

### Checked and fine
- **Dependency direction:**
  - `eval.budgets` is placed on top of eval's internal order (`pyproject.toml:211-216`) and imports only `eval.runs` and `foundations.errors`;
  - `orchestration.vo` → `eval.budgets` is allowed, and no rung imports eval;
  - `backends` is untouched and still only locates;
  - nothing references reifyscene.
- **C++ and Rust layering:** `eval::budgets` and `slamlab_eval::budgets` live in the eval library and crate and are used only by orchestration. No core includes eval. The BA internals stay private (`cpp/src/foundations/ba/` `detail` namespace; Rust `mod schur`), and `linalg` stays `pub(crate)`.
- **Lock-step:**
  - `extract_levels`, `as_pyramid`/`check_pyramid` and the pyramid-taking `stereo.match` are in all three languages, with `Implementation` Protocols updated;
  - the `extract_levels` fixture is held exactly (`tests/fixtures/foundations/orb.json`) and the rejection tests run on every backend;
  - `FrameResult.track_seconds`/`map_seconds` and the Delivered/Reader/build split are mirrored in all three programs.
- **A1's design:** the frame builder owns the pyramids, rather than `Features` holding them, so keyframes do not pin pyramids in the map. This matches ORB-SLAM's `mvImagePyramid` use.
- **The BA rewrite:** it is mirrored structurally between C++ and Rust (`Reduction::kept`/`compacted`, a per-problem `CouplingLayout` shared via `shared_ptr`/`Rc`, fixed-size blocks by kind). The Python reference stays the readable statement, as the language split intends.
- **Portability of the image speed-ups:** they are portable scalar code that autovectorises. The diff adds no intrinsics, `unsafe`, threads, `-march` flags or globals, which keeps the standalone and ARM path open for milestone 7.
- **`eval.budgets` side effects:** none at import time, and no hidden state. The `/proc` and `/sys` paths are explicit parameters with defaults.
- **Run format version 2:** a clean break with no version-1 path. The writers in all three languages, the golden folder and `run_values.json` changed together.
- **Python's program:** it mirrors the native command line, exit codes and messages (including the missing-file kind), so a budget runs every backend as a program.
- **Mapping thread:** its deferral to milestone 7 is recorded in the ladder and the note, and the edge rung stays single-threaded and deterministic.
- **The edge rung's budget statement:** `src/slamlab/edge/__init__.py` ("real time under `laptop-cpu`") matches what was built.
- **Runtime dependencies:** no new ones in any language; faer's footprint shrank.
- **Privacy:** no rig-bag specifics in the diff (scanned for bag, topic, sensor and home-path terms).

### Algorithm review

## Algorithm review

**Scope:** PR 9 "Budgets and real time", `git diff 7bccdd7...aeb6121` (76 files), read against `docs/notes/real-time.md`. I read these in full: `src/slamlab/eval/budgets.py`; `cpp/src/eval/budgets.cpp` and `rust/slamlab-eval/src/budgets.rs`; the step timers in `cpp/src/edge/pipeline.cpp`, `rust/slamlab-edge/src/pipeline.rs` and `cpp/src/orchestration/vo.cpp`; FAST, blur and resize in `cpp/src/foundations/{orb,image}.cpp` and `rust/slamlab-foundations/src/{orb,image}.rs`; the BA rewrite in `cpp/src/foundations/ba/{bundle.cpp,schur.hpp,common.hpp}` and `rust/slamlab-foundations/src/ba/{bundle.rs,schur.rs}`; `rust/slamlab-foundations/src/linalg.rs`; the stereo and frame-grid changes. All experiments ran in the session scratchpad. Two side effects: importing `slamlab` triggered scikit-build's editable rebuild into the git-ignored `build/`, and `git status` is still clean at aeb6121.

**Summary:** The algorithms are sound.
- **FAST:** the run-count segment test is provably the same as the old score test.
- **Overflow:** all the 32-bit bounds hold.
- **BA:** the Schur complement, back-substitution, repeated columns, inverse-depth points and gauge basis are all correct over the compacted coordinates.
- **SVD:** Rust's 3x3 SVD is a faithful port of the installed Eigen 5.0.1-dev `JacobiSVD`. I confirmed it bit-identical on 5,000 further hard matrices.

Two things need fixing. The C++ blur's interior pass breaks `std::span::subspan`'s precondition when the kernel is wider than the image, and the repo's own image test aborts under libstdc++ assertions. `budgets.judge` accepts timestamps that do not increase and returns a zero or negative deadline and an infinite or negative load. The note also needs three corrections: the "0-bit rounding" fix it claims was not a bug in the base, the SVD's bit-identity has no in-repo test, and its downstream consumers still differ by an ulp.

### Findings

#### [must-fix] C++ blur: `subspan` precondition broken when the kernel is wider than the image (the repo's own test aborts under assertions)
- **Where:** `cpp/src/foundations/image.cpp:333-344`
- **Issue:** `first = min(radius, cols)` and `last = max(first, cols - radius)`. When `taps - 1 > cols`, the interior is empty (`last == first`), yet the loop still runs `row.subspan(at(k), at(last - first))` for every tap. For `k > cols` that violates `std::span::subspan`'s `offset <= size()` precondition. That is undefined behaviour: pointer arithmetic past one-past-the-end. Rust guards the same loop with `if last > first` (`rust/slamlab-foundations/src/image.rs:518`). The base code did not use spans here, so this is new in this PR.
- **Why it matters:** `blur` is public and bound to Python, and it accepts any image of 1x1 or larger. The parity fixture's own "one pixel" case (`[[200]]`, kernel `[1,2,1]`) triggers it. Today's `-O3 -DNDEBUG` build runs it harmlessly, but a hardened toolchain (`_GLIBCXX_ASSERTIONS`, which several distributions enable by default) aborts.
- **Evidence:** I rebuilt the C++ tests in scratch with `-O1 -D_GLIBCXX_ASSERTIONS` and without `NDEBUG`, so Eigen's assertions were on too.
  - `foundations_test_image` aborts with `span:450 ... subspan(size_type, size_type) const ... Assertion '__offset <= size()' failed.` (exit 134).
  - A standalone `image::blur` of a 1x1 image with `{1,2,1}` aborts the same way.
  - The other tests built this way pass: orb, ba, stereo, gauge, nls, and edge frames/map/adjust/pipeline.
- **Suggested fix:** Wrap the interior tap loop in `if (last > first)`, as Rust does. If you want it pinned, add a case to the hardened build or a C++ test with `cols < taps - 1`, e.g. a 7-tap kernel on a 2-column image.

#### [must-fix] `budgets.judge` accepts non-increasing or non-finite timestamps and returns a meaningless judgement
- **Where:** `src/slamlab/eval/budgets.py:285-299` (`judge`, line 257)
- **Issue:** The deadline is `median(diff(times))` and the load divides by `times[-1] - times[0] + deadline`. Nothing checks that the timestamps increase and are finite, or that the step times are finite and non-negative. A missing `timestamp` key raises a raw `KeyError`, not `InvalidInputError`.
- **Why it matters:** `judge` is the public measurement of the milestone's goal. If a run read from disk or built by hand has its frames out of order, the judgement is silently wrong: only a numpy `RuntimeWarning` appears, and a "load below 1" can come from a negative denominator. The shipped programs write increasing timestamps, so no figure in the note is affected.
- **Evidence:** I ran this probe:
  ```
  duplicate timestamps [1.0, 1.0]:      deadline 0.0, late 2 of 2, load inf   (RuntimeWarning: divide by zero)
  decreasing timestamps [2.0,1.9,1.8]:  deadline -0.0999..., late 3, load -0.1666...
  ```
- **Suggested fix:** Raise `InvalidInputError` unless the timestamps are finite and strictly increasing, which makes the deadline and span positive. Do the same for negative or non-finite step times, and handle a missing `timestamp` like the step keys. Add one test per case.

#### [should-fix] The "0-bit kernel's rounding half" fix describes a bug the base did not have
- **Where:** `CHANGELOG.md:62-63` (under Fixed); `docs/notes/real-time.md:113-115`; commit 2db3289's message
- **Issue:** The record says the old rounding "had been a shift by -1, undefined in C++". At base 7bccdd7 the C++ blur called `round_shift(sum, 2 * bits)`, which divides: `scale = pow2(bits)`, then `(value + scale/2) / scale`. With `bits = 0`, `pow2(0) = 1` and `scale/2 = 0`, so there is no shift and no undefined behaviour. Rust's old `round_shift` used `(1_i64 << bits) >> 1`, also 0. The shift by -1 can only have existed in an intermediate draft of this PR.
- **Why it matters:** The CHANGELOG and the design note are the project's record. A Fixed entry for a bug that never shipped misleads anyone tracing undefined behaviour later.
- **Evidence:** `git show 7bccdd7:cpp/src/foundations/image.cpp` lines 35-38, and `git show 7bccdd7:cpp/include/slamlab/foundations/image.hpp` lines 26-32 (`pow2` is a multiply loop).
- **Suggested fix:** Drop the CHANGELOG entry and the note's sentence. At most, say the new vertical pass keeps a 0-bit kernel's rounding half at 0, as `round_shift` did.

#### [should-fix] Rust `svd3`'s bit-identity to Eigen has no in-repo test, and the note's lesson overstates what it buys
- **Where:** `rust/slamlab-foundations/src/linalg.rs:167-438` (tests at the end of the file); `docs/notes/real-time.md:161-163, 272-276`
- **Issue:**
  1. The 39,005-matrix bit-identity sweep lives outside the repo. In-repo, only one diagonal case is checked exactly against Eigen's behaviour (`svd3_orders_and_signs_as_eigen`). An Eigen upgrade (the env is on 5.0.1-dev, and 3.3 to 3.4 already changed this loop's threshold) or a later edit to `jacobi_svd3` would break the property unnoticed.
  2. The SVD's factors do match. But its consumers still differ across languages, because nalgebra and Eigen round the 3x3 product `u * v_t` differently. So "where a decision hangs on roundoff, the two languages would have differed" is still true one step downstream.
- **Evidence:**
  - *SVD factors:* I built a scratch C++ program (Eigen `JacobiSVD<Matrix3d, FullU|FullV>`, `-O3 -ffp-contract=off`) and a scratch Rust program containing a verbatim copy of `linalg.rs:167-405`. On 5,000 hard matrices (random, graded 1e±8, rank 1 plus 1e-9, scaled 1e±200, near-repeated σ, skew plus 1e-12 I), U, σ and V are bitwise identical: 0 differences.
  - *Consumer:* `so3.project_to_so3(A, backend='cpp')` and `backend='rust'` differ bitwise on 2,637 of 5,000 matrices, at most 2.8e-17. Recomputing `U V^T` in sequential order `(t0+t1)+t2` reproduces Rust on 5,000 of 5,000 and C++ on 2,363 of 5,000.
- **Suggested fix:**
  - Commit a small table of hard 3x3 matrices with Eigen's exact U, σ and V (hex floats). Assert it in a Rust test, and in a C++ test, so an Eigen upgrade also shows up.
  - Reword the lesson: the factors agree to the bit, while consumers that multiply them still agree to an ulp, not to the bit.

#### [should-fix] The block's fixed column capacity is not tied to the camera models
- **Where:** `cpp/src/foundations/ba/common.hpp:19-25`; `cpp/src/foundations/ba/bundle.cpp:635`; `rust/slamlab-foundations/src/ba/bundle.rs:20`
- **Issue:** `kMaxIntrinsics = 9` is a literal ("radtan's nine"). `couple()` then `conservativeResize`s a bounded Eigen matrix (MaxCols = 30) to `width + 6 + n_params`. Adding a model with more than 9 parameters would exceed the bound. With `NDEBUG`, `eigen_assert` is off, so that is a silent stack overflow. Rust takes the bound from `Model::RadTan.intrinsics_len()`, so it would panic instead, but it also assumes RadTan is the largest model.
- **Why it matters:** This is latent today (models have 4, 9 and 5 parameters), but the failure mode in C++ is memory corruption, not an error.
- **Suggested fix:** Add a `static_assert`, or a check when the problem is validated, that every model's parameter count is at most `kMaxIntrinsics`. In Rust, take the maximum over all models.

#### [note] Rust's Jacobi sweep has no cap (as Eigen); it terminates in practice
- **Where:** `rust/slamlab-foundations/src/linalg.rs:357`
- **Context:** It faithfully mirrors Eigen's `while (!finished)`, which has no cap (`JacobiSVD.h:745`). Cyclic two-sided Jacobi with the threshold `max(min_positive, 2ε·maxDiag)` converges: Forsythe & Henrici, "The cyclic Jacobi method for computing the principal values of a complex matrix", Trans. AMS 94 (1960); Demmel & Veselić, "Jacobi's method is more accurate than QR", SIAM J. Matrix Anal. Appl. 13(4), 1992.
- **Evidence:** A Python port with the same operation order, on 20,007 adversarial matrices (entries scaled 1e±300, zero diagonal, subnormal, skew, rank 1, near-repeated σ, mixed 1e-300 and 1 entries): at most 6 sweeps, singular values within 1e-13·σ_max of LAPACK.
- **Optional:** The project's standard asks iterative methods for a cap. A generous one (say 64 sweeps, returning `None`) cannot change any result where Eigen terminates.

#### [note] Rule 1's split puts keyframe creation on the mapping side, where ORB-SLAM2 does it in tracking
- **Where:** `cpp/src/edge/mapping.cpp:866-871` (`insert_keyframe`, `create_points_from_depth` inside `process`, timed as `map_seconds` at `cpp/src/edge/pipeline.cpp:236`)
- **Context:** In ORB-SLAM2, `Tracking::CreateNewKeyFrame` builds the keyframe and, for stereo and RGB-D, its close map points in the tracking thread, then hands it to local mapping (Mur-Artal & Tardós, "ORB-SLAM2", T-RO 2017; github.com/raulmur/ORB_SLAM2 `src/Tracking.cc`). The latency judged here leaves out that step. It is probably small next to local BA, but the note could say so, or time it with tracking. This does not re-open rule 1; it is about which steps count as "tracking".

#### [note] A run's whole-run load hides bursts; a single-queue backlog is the "thread of its own" measure
- **Where:** `src/slamlab/eval/budgets.py:299`; `docs/notes/real-time.md` ("Mapping's load")
- **Context:** The note shows the gap itself: a whole-run load of 0.84 against 2.28 in the busiest 10 s. The busiest-window figures come from a script outside the repo (`budget_mapping.py`), so `judge` cannot reproduce the note's table. Lindley's recursion is cheap and exact for a FIFO mapping thread: W_{n+1} = max(0, W_n + S_n − A_{n+1}), with keyframe arrival gaps A and mapping times S (Lindley, "The theory of queues with a single server", Proc. Cambridge Phil. Soc. 48, 1952). Its maximum wait is the lag a mapping thread would build up. It is an upper bound, because ORB-SLAM also holds back keyframes and aborts its local BA. Worth adding to `judge` alongside the load.

#### [note] The verification evidence lives outside the repo
- **Where:** `docs/notes/real-time.md`, "Verifying what is no longer the same to the bit"
- **Context:** The BA per-solve comparison is `ba_regression`, and its 3,136 problems sit in `../results/real_time_2026-10-07/ba_problems/`. I could not find the SVD sweep's harness. The note does not say where either lives.
- **Also:** "points to 1.5e-9 relative" is relative to max(1, |x|) (`ba_compare.cpp:69-70`), so it is absolute for coordinates under 1 m.
- **Suggestion:** Point to the kit and the metric.

#### [note] Repeated columns: the host's own observation of an anchored point is analytically null
- **Where:** `cpp/src/foundations/ba/bundle.cpp:528, 542`; `rust/slamlab-foundations/src/ba/bundle.rs` (`accumulate`)
- **Context:** The pairwise scatter over repeated columns is correct: it builds (J_obs+J_host)ᵀ(J_obs+J_host) and (J_obs+J_host)ᵀ W J_p. For the anchor's own observation, though, x_cam = ray/ρ does not depend on the host pose, the intrinsics or ρ. Its columns cancel exactly, and summing pairwise products leaves about ε|J|² of noise rather than zero.
- **Optional:** Merging repeated columns once at layout time, then summing the Jacobian columns before the products, would cancel the noise at the Jacobian level and cost fewer products. It is harmless today, as the per-solve check shows.

#### [note] Context: the rewrite matches the standard reduced camera system
- **Context:** Fixed vertices left out of the Hessian is g2o's handling of fixed vertices (Kümmerle et al., "g2o: A General Framework for Graph Optimization", ICRA 2011). Eliminating points into a dense reduced camera system is the standard formulation (Triggs et al., "Bundle Adjustment — A Modern Synthesis", 1999, §6). Dense Cholesky fits local BA (about 60 free camera coordinates). The global BA of milestone 4 will want sparse Schur or preconditioned CG (Agarwal, Snavely, Seitz & Szeliski, "Bundle Adjustment in the Large", ECCV 2010).
- **Timing reference:** ORB-SLAM's Table I reports a tracking median of 30.57 ms and local BA at 296 of 384 ms of local mapping (Mur-Artal, Montiel & Tardós, T-RO 2015, [arXiv:1502.00956](https://arxiv.org/pdf/1502.00956)). That supports the note's "local BA about four fifths of mapping".

#### [note] A budget's "cores" are logical CPUs
- **Where:** `src/slamlab/eval/budgets.py:89` (`cpus`)
- **Context:** This machine has no SMT, so 4 CPUs are 4 cores. On a machine with SMT, the highest-numbered 4 logical CPUs can be 2 physical cores. That matters once mapping gets a thread of its own (milestone 7). Check `/sys/devices/system/cpu/cpu*/topology/thread_siblings_list` then.

### Checked and fine
- **FAST segment test (C++ and Rust):** `longest run ≥ 9` with `diff > floor` holds exactly when `best − 1 ≥ floor`. A 25-step walk covers every wrapping arc, and u8 run counters reach at most 25. Arc scores are exact in int16 (d ∈ [−255, 255], score ∈ [−1, 254]) and the floor is validated to [1, 254]. Window reads stay at least 3 px inside the level. A random cross-check (60 images: noise, binary 0/255, low contrast) gave identical `fast_scores` for Python, C++ and Rust. This matches OpenCV's SSE2 FAST run counting (`modules/features2d/src/fast.cpp`; Rosten & Drummond, ECCV 2006).
- **Blur bounds:** horizontal sums are at most 255·2^15 (fits int32/u32); vertical sums in u32 are exactly the cases with 2b+8 ≤ 32 (255·2^24 + 2^23 < 2^32), else u64. Integer sums do not depend on order. A cross-check of 0-, 2-, 6- and 15-bit kernels on 1x1 to 9x2 images matched in all three languages.
- **Resize bounds:** at most 255·2^11 across and 255·2^22 down (< 2^31), with the fraction in [0, 2^11) by `axis_samples`, and rounding identical to the old `round_shift`.
- **Schur elimination and back-substitution (C++ and Rust):** S = H_cc − Σ E_p H_p⁻¹ E_pᵀ on the upper triangle of ascending distinct columns, then mirrored; b = g_c − Σ E_p H_p⁻¹ g_p; Δp = −H_p⁻¹(g_p + E_pᵀ Δc) with Δc expanded over the kept coordinates. Points coupled to no kept coordinate are handled.
- **Kept coordinates:** `Reduction::compacted` is correct. Positions come from one sorted kept list, which also serves `lay_out_blocks`. The confined poses' rows stay contiguous, so the block-diagonal basis carries over unchanged. `camera_variances` and `intrinsics_positions` index the reduced space, which compaction leaves unchanged.
- **Inverse-depth points:** the unused dimensions get zero coupling columns, inert identity blocks and damping on `head(dims)` only, so their step is exactly zero.
- **Fixed-size accumulation:** the C++ `Map<Matrix<Rows, Dynamic>>` over a bounded `Block` uses outer stride = rows, which is correct. The block's rows and its residual's always match by kind (2, 1 or 6). Block widths equal `block_columns` lengths, anchored blocks included.
- **Hardened build:** with libstdc++ assertions and Eigen's own assertions on, the BA, ORB, stereo and edge tests pass. Only the image test (the blur finding above) aborts.
- **Rust SVD port vs the installed Eigen 5.0.1-dev** (`SVD/JacobiSVD.h:700-800`, `misc/RealSvd2x2.h`, `Jacobi/Jacobi.h:54-117`): the following all match Eigen:
  - scale by the largest |entry|, with non-finite input refused (Eigen: `InvalidInput`);
  - `maxDiagEntry` threshold and the `min_positive` floor;
  - the 2x2 symmetrising rotation, `makeJacobi` (same operation order, sign handling), the rotation product and transpose;
  - `applyOnTheLeft/Right` semantics, including the identity skip;
  - sign moved into U, sorting with the first maximum kept, the stop at zero.

  Zero, identity and NaN behave as Eigen does.
- **Peak memory:** VmHWM read by the program itself is the right measure. In a check, a fresh child of a 400 MB Python process read `VmHWM` 2.3 MB, while its `ru_maxrss` was 421 MB.
- **`judge` on well-formed runs:** the deadline is the median spacing, latency is build + track with a strict `>`, load is Σ map / (span + period), p95 uses linear interpolation. The step timers bracket the right calls: read covers decode and rectification; build covers pyramid, extraction, matching and depth; track is the whole call minus `process`.
- **Shared pyramid:** `extract_levels`, the `check_pyramid` shape checks and stereo reading the extraction's pyramid all match ORB-SLAM's `ComputeStereoMatches`. Python, C++ and Rust validate alike.
- **Frame grid:** each `Frame` owns its `FeatureGrid`, built from `features.keypoints` at assembly. Nothing mutates the keypoints afterwards (grep across `cpp/src/edge` and `rust/slamlab-edge/src`).
- **Verification design:** checking per solve (iterations, termination, chi-square flips), by parity, and over all 100 whole-run decisions is a sound way to accept a change that is no longer bit-identical. The BA fixture covers the hard cases: anchored points whose host observes them ("every kind of block"), confined poses ("partial: rotation priors confine two poses"), fixed poses ("explicit none with two fixed poses") and `intrinsics_std`.
