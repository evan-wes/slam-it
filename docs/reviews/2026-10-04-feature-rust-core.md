# Review panel: `feature/rust-core`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-04

| | |
|---|---|
| **Branch** | `feature/rust-core` |
| **Range** | `3853b8cd40e1901229da88f31518260ff24d59a2...4ad3eb9e54850d978262a4ec1d9d4a9278ed2a07` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`): 1683 pytest passed, 20 skipped; 17 ctest; cargo fmt, clippy pedantic and tests clean |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `rust/.../src/ba/{bundle,layout,schur,mod}.rs` | No native Rust tests for the BA modules | fix | fixed (6a72801) -- decided by evan-wes, 2026-10-04 |
| H2 | should-fix | hygiene | `rust/.../src/alignment.rs` | `umeyama` does not reject non-finite weights or points | fix | fixed (6af1f77) -- decided by evan-wes, 2026-10-04 |
| H3 | should-fix | hygiene | `camera.rs`, `so3.rs` | Error messages repeat tolerance values that are named constants | waive | waived: the literals match the C++ and Python messages word for word, which is deliberate -- decided by evan-wes, 2026-10-04 |
| H4 | should-fix | hygiene | `alignment.rs`, `slamlab-py/.../ba.rs` | Rustdoc arguments omit frames, units and shapes | fix | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| H5 | should-fix | hygiene | `slamlab-py/.../ba.rs` `solve`, `ba/bundle.rs` `adjust` | Long functions that mix phases | fix (binding) | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| H6 | note | hygiene | `slamlab-py/.../ba.rs` `code_of` | Overflow and "none" both map to -1 | fix (comment) | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| H7 | note | hygiene | benches, example | Test/bench-only `unwrap` and prints | no action | waived: no action, appropriate in bench and example code -- decided by evan-wes, 2026-10-04 |
| A1 | should-fix | architecture | `tests/test_backends.py`, `tests/conftest.py`, `backends.py`, design doc | "Each Rust module in full" is stated as enforced but nothing enforces it; "never gates a merge" is no longer true | fix | fixed (3f99fc9) -- decided by evan-wes, 2026-10-04 |
| A2 | should-fix | architecture | `scripts/check_layers.py` | Rust's module order inside rung 0 is not checked | fix | fixed (1c04584) -- decided by evan-wes, 2026-10-04 |
| A3 | should-fix | architecture | `nls_crossing.rs`, `slamlab-py/.../ba.rs` | Enum codes leave by discriminant but are named and decoded by position in `ALL` | fix | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| A4 | should-fix | architecture | `cpp/bench`, Rust `benches`/`examples`, design doc | Native benchmarks have no place in the documented layout; the native BAL readers have no parity check | fix | fixed (1c04584) -- decided by evan-wes, 2026-10-04 |
| A5 | note | architecture | `ba/mod.rs` | `ba::Summary` could implement `solve::Summary` by delegation | defer | deferred (design doc open items, to PR 6) -- decided by evan-wes, 2026-10-04 |
| A6 | note | architecture | tests | Compiled-backend lists are literals | fix | fixed (3f99fc9) -- decided by evan-wes, 2026-10-04 |
| A7 | note | architecture | `tests/fixtures/foundations/rng.json` | The fixture lacks the stated `tolerance` field | fix | fixed (3f99fc9) -- decided by evan-wes, 2026-10-04 |
| L1 | should-fix | algorithm | `nls.rs` `try_step` (and C++ `nls.hpp`) | A NaN gradient entry is dropped from the norm and can report convergence | fix | fixed (f195ecf) -- decided by evan-wes, 2026-10-04 |
| L2 | should-fix | algorithm | `ba/gauge.rs` | nalgebra's general SVD has no iteration cap; a non-finite matrix hangs | fix | fixed (6af1f77) -- decided by evan-wes, 2026-10-04 |
| L3 | should-fix | algorithm | `docs/notes/rust-port.md` | COLMAP's time is multi-threaded, beside single-core backends | fix | fixed (3223ddb) -- decided by evan-wes, 2026-10-04 |
| L4 | should-fix | algorithm | `docs/notes/rust-port.md` | The radtan explanation is wrong; "bit-level agreement" overstates it | fix | fixed (3223ddb) -- decided by evan-wes, 2026-10-04 |
| L5 | note | algorithm | `so3.rs`, `se3.rs` checks; Cholesky paths | Native checks let NaN through where Python validates first | fix | fixed (6af1f77) -- decided by evan-wes, 2026-10-04 |
| L6 | note | algorithm | `so3.rs`, `alignment.rs` | The 3x3 SVD is a different algorithm; checked, fine | no action | waived: no action, checked and fine -- decided by evan-wes, 2026-10-04 |
| L7 | note | algorithm | `ba/bundle.rs` `anchors` | An exactly singular anchor Jacobian is handled differently from C++ | fix (comment) | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| L8 | note | algorithm | `camera.rs` `unproject` | An unreachable error branch | fix | fixed (b506d1e) -- decided by evan-wes, 2026-10-04 |
| L9 | note | algorithm | -- | faer is faster for large dense Cholesky | no action | waived: no action, context for milestone 4's sparse solver -- decided by evan-wes, 2026-10-04 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `git diff 3853b8c...4ad3eb9` on `feature/rust-core`. Read `alignment.rs`, `schur.rs` (head), `ba.rs` and `common.rs` (binding) in full or large part, `camera.rs`/`so3.rs`/`se3.rs` by constant, signature and message, and scanned all `ba/*.rs`, `nls.rs`, `rng.rs` and the `slamlab-py` crate for panics, `#[allow]`, magic numbers, function length and test placement, plus the `test_ba.py` diff. Did not read `nls.rs`, `bundle.rs`, `layout.rs`, `gauge.rs` or the C++/docs hunks line by line; did not re-run layer 1.

**Summary:** This is clean, consistently documented code: rustdoc sections throughout, named tolerance constants with comments, every `#[allow]` carries a `reason`, library `expect`s limited to explicitly requested SVD factors, `unwrap`s confined to tests. The main weaknesses are thin native Rust tests for the BA modules, missing finiteness checks on some public Rust entry points, and a few small drift hazards.

#### [should-fix] BA modules have no native Rust tests (`bundle`, `layout`, `schur`, `mod`)
- **Where:** `rust/slamlab-foundations/src/ba/{bundle,layout,schur,mod}.rs` -- only `ba/gauge.rs:513` has a `mod tests`
- **Issue:** ~2,700 lines of BA code (`adjust`, layout, Schur elimination, problem validation in `ba/mod.rs:330-565`) have no `#[test]`. `#[test]` counts per Rust crate: alignment 2, gauge 4, camera 4, error 1, nls 6, rng 3, se3 4, so3 7, and zero in the four BA files. No `rust/slamlab-foundations/tests/` directory either.
- **Why it matters:** Rust BA is covered only through Python parity tests (`tests/foundations/test_ba.py` `["cpp","rust"]`) and the `bal` example. A regression in Schur or layout cannot be caught by `cargo test` alone, which the Rust crate should support standalone. The Schur `Reduction` round-trips (`reduce_*`/`expand`) and the validation branches are cheap to unit-test.
- **Evidence:** `grep -n 'mod tests'` lists only gauge, se3, camera, so3, nls, error, rng, alignment.
- **Suggested fix:** Add small native tests: `Reduction` (`expand(reduce_vector(v))` equals the projection), the Schur solve against a dense solve on a tiny problem, a table of `validate` rejections checking each message, and one 3-camera BA recovery in `bundle.rs`.
- **Reviewer note:** If the panel treats "tests exist for the behaviour" as must-fix, this qualifies; Python parity covers behaviour, which is why I rate it should-fix.

#### [should-fix] `umeyama` (public Rust API) does not check finiteness and says nothing about it
- **Where:** `rust/slamlab-foundations/src/alignment.rs:36-58`
- **Issue:** `weights.iter().any(|&w| w < 0.0)` is false for NaN. A NaN or inf weight, or a NaN point, passes the checks and yields NaN/garbage; `d[1] <= DEGENERACY_RATIO * d[0]` is false when either is NaN, so the NaN result is returned as `Ok`.
- **Why it matters:** The Python wrapper validates via `as_real`/`as_batch`, so bindings are protected, but a direct Rust caller of `umeyama` is not, while `ba/mod.rs:341,352,448` do check `is_finite`, so the crate is inconsistent with itself.
- **Suggested fix:** Reject non-finite weights and points with `InvalidInput`, naming the argument; add a test.

#### [should-fix] Error messages hard-code tolerance values that are also named constants
- **Where:** `rust/slamlab-foundations/src/camera.rs:255` ("Z must exceed 1e-09" vs `MIN_DEPTH`), `so3.rs:219` ("1e-09" vs `SKEW_TOLERANCE`), `so3.rs:259` ("1e-06" vs `ROTATION_TOLERANCE`), `so3.rs:389` ("1e-06" vs `QUATERNION_TOLERANCE`)
- **Issue:** The literal in the message duplicates the constant and will drift if the constant changes.
- **Suggested fix:** `format!("... {MIN_DEPTH:e}")` or similar; if byte-identical wording with C++/Python is the reason for literals, keep them but add a test pinning constant to message.

#### [should-fix] Rustdoc argument lines omit frames, units and dtype in places
- **Where:** `rust/slamlab-foundations/src/alignment.rs:19-24` ("Source points, N >= 3" -- no frame, unit, or that points are `Vector3<f64>` in a common frame; the result's direction, source to target, is not stated); `rust/slamlab-py/src/foundations/ba.rs:93-101` ("Every field of `BAProblem`, by keyword" defers entirely to the Python docs)
- **Issue:** CLAUDE.md asks for conventions to be stated, not assumed.
- **Suggested fix:** State "maps source to target: y ≈ s R x + t" and units/frames; in the BA binding, list the array shapes and dtypes, or at least link the `Implementation` Protocol by path.

#### [should-fix] Two long functions mix phases
- **Where:** `rust/slamlab-foundations/src/ba/bundle.rs` `adjust` (~78 lines); `rust/slamlab-py/src/foundations/ba.rs:130-233` `solve` (~93 lines)
- **Issue:** `solve` builds the `Problem` from 19 arguments, runs the solve, then marshals ~14 outputs in one function.
- **Suggested fix:** Split `solve` into `read_problem(...)` and `write_solution(py, &solution, params)`; consider a `ProblemIn` struct for the 19 arguments, which would also remove the `too_many_arguments` allow. Split `adjust` by phase if it has distinct stages.

#### [note] `code_of` silently maps an unconvertible index to -1
- **Where:** `rust/slamlab-py/src/foundations/ba.rs:117`
- **Detail:** `i64::try_from(i).ok().unwrap_or(-1)` conflates "none" with "overflow". The overflow is unreachable in practice, so a note; a comment saying so would suffice.

#### [note] Test-only `unwrap`/`expect` and `eprintln!`/`println!` in bench and example
- **Where:** `rust/slamlab-foundations/benches/foundations.rs`, `rust/slamlab-foundations/examples/bal/main.rs`
- **Detail:** Appropriate in test, bench and example code. The bench skips with a message when `SLAMLAB_BAL` is unset, which is fine.

#### Checked and fine
- `clippy.toml` `doc-valid-idents` is justified; all 17 `#[allow]`s have specific, true `reason`s.
- Tolerances are named constants with doc comments (`so3.rs:17-29`, `se3.rs:18`, `camera.rs:58,62`, `alignment.rs:13`).
- Library `expect`s are limited to requested SVD `U`/`Vᵀ`; the "Panics: Never" doc is accurate.
- `ba/mod.rs` problem and option validation covers lengths, finiteness, signs and the `min_parallax_deg`/`focal_std_ratio` ranges, with messages naming the bad field.
- The binding layer is neat: `by_name`/`enum_names` derive from core `name()`; the serde `deny_unknown_fields` options struct is documented with no stray comments; contiguity and shape errors name the shape.
- Python parity tests are parametrised over `cpp`/`rust` and assert values (`atol=1e-8`, exact index checks) and error messages, including the serde-worded options errors.
- No debug prints or commented-out code in library code, no hard-coded paths in the diff, no unowned TODOs.
- Module docstrings and rustdoc `# Arguments`/`# Returns`/`# Errors` sections are present and useful in the files read.

### Architecture review

**Scope:** `git diff 3853b8cd...4ad3eb9e` (7 commits, `feature/rust-core`, PR 5). Read the full diff of docs, build files, scripts, C++ core/binding edits and Python/test changes; in the Rust tree the crate roots (`lib.rs`, `error.rs`, `solve.rs`), the BA facade (`ba/mod.rs`), the public surface of `nls.rs`, `rng.rs`, the whole `slamlab_rs` binding layer (`common.rs`, `foundations/{mod,so3,se3,camera,alignment,ba,nls_crossing}.rs`), benches and the example. Compared against `tests/test_backends.py`, `tests/conftest.py`, `src/slamlab/backends.py`, `scripts/check_layers.py`, the import-linter contracts and the design doc.

**Summary:** The port follows the design: the Rust core builds without Python and depends only on nalgebra; bindings are one file per module and only convert types (serde/pythonize sit in the bindings crate, never the core); the extension tree mirrors the Python packages; A27 removes the last binding-side name tables in C++. The weaknesses are at the edges of the new "Rust implements each module in full" rule: the design doc says the tests enforce it but nothing does, and several docstrings and the doc still describe the old subset rule. Two more gaps: the intra-crate order of Rust's now ten-module rung 0 is not checked, and enum codes leave the Rust binding through two different sources of truth. No must-fix.

#### [should-fix] The "each Rust module in full" rule is stated as enforced, but no test enforces it, and the old wording contradicts it
- **Where:** `tests/test_backends.py:219` (`if name in backends.LOCKSTEP_BACKENDS: assert provided == members`); `src/slamlab/backends.py:28-33`; `tests/conftest.py:57-60,82-84`; `docs/design/2026-10-02-slamlab-ladder.md` "Languages per module"
- **Issue:** The design doc now says "Rust implements a curated subset and never gates a merge. The tests enforce both rules. Each module Rust implements, it implements in full, and its tests run on all three backends." But the Protocol test still only checks `provided <= members` for Rust, so a Rust module missing a function passes; `require_function` still skips a missing Rust function silently ("curated subset"); `LOCKSTEP_BACKENDS`' docstring still says "a Rust module may lack functions". "Never gates a merge" is also no longer true: `check.sh` sets `SLAMLAB_REQUIRE_BACKENDS=python,cpp,rust` and the BA, SO(3) and SE(3) property tests now take `backends.BACKENDS`, so a Python BA change Rust does not follow fails layer 1.
- **Why it matters:** The new rule exists only in prose. A partial Rust binding added in PR 6 (e.g. RANSAC without `refine_consensus`) would pass `test_backends`, and tests using `require_function` would quietly skip it. The doc tells contributors something the code does not do.
- **Evidence:** `test_every_compiled_module_matches_its_protocol` asserts `provided == members` only in the `LOCKSTEP_BACKENDS` branch. Every module `slamlab_rs` exposes at HEAD (alignment, ba, camera, se3, so3) is complete, so the tighter check already passes.
- **Suggested fix:** Make the code match the new doc rule: in `test_backends`, assert `provided == members` for every compiled backend (the module-level subset is already expressed by Rust simply not exposing a module); make `require_function` fail rather than skip when Rust exposes a module but lacks the function; reword `LOCKSTEP_BACKENDS` (it now means "implements every module", not "every function"); replace "never gates a merge" with what is now true (Rust's modules gate a merge; Rust need not implement every module until PR 6).

#### [should-fix] Rust's internal module order inside rung 0 is not checked
- **Where:** `scripts/check_layers.py` docstring ("Rust's modules inside a crate are not checked"); `rust/slamlab-foundations/src/**`
- **Issue:** Before this PR the crate had only `so3`. It now has `error`, `solve`, `rng`, `so3`, `se3`, `alignment`, `camera`, `nls`, `ba/{mod,gauge,layout,schur,bundle}` -- the same structure as the C++ library, for which `check_layers.py` enforces "Rung 0's internal order" and "Bundle adjustment's internal order" (chains included). The Rust side has no such check.
- **Why it matters:** The order is respected today (every `use crate::`/`super::` line checked: e.g. `ba/schur.rs` uses only `super::ReducedSystemSolver` and `crate::nls::LinearSystem`; `layout`/`schur` do not use each other). But nothing keeps it so, and PR 6 adds `ransac`, `triangulation`, `epipolar`, `pnp`, `twoview` to this crate, which is when a backward `use` (e.g. `camera` using `nls`) would most likely land unnoticed. One source of truth for three languages is the design's stated goal.
- **Evidence:** `git grep -nE '^\s*use (crate|super)::' rust/slamlab-foundations/src` lists the edges; no script reads them.
- **Suggested fix:** Extend `check_layers.py` to Rust modules: a file is the module its path names (`ba/mod.rs` the facade, as `_CPP_FACADES` handles for C++); edges from `use crate::<m>` and `super::<m>` paths, excluding `#[cfg(test)]` modules as `cpp/tests` is excluded; the same pyproject contracts. In this PR or as the first commit of PR 6, before the estimation modules land; update CLAUDE.md's "Rust's modules inside a crate are not checked."

#### [should-fix] Enum codes leave the Rust binding by discriminant but are named and decoded by position in `ALL`
- **Where:** `rust/slamlab-py/src/foundations/nls_crossing.rs:169` (`summary.termination as i64`); `rust/slamlab-py/src/foundations/ba.rs:215` (`summary.gauge as i64`); vs `common.rs` `decode`/`enum_names`, which index `ALL`
- **Issue:** `codes()` builds each name table from `E::ALL` in array order and `decode` maps a code to `ALL[code]`, but the result encodes `Gauge`/`Termination` with the explicit discriminant (`= 0 … = 6`). Two hand-maintained lists must agree and no test ties them. C++ has a single source (`static_cast<E>(k)` for `k < kCount`, both ways).
- **Why it matters:** Reordering `ALL`, or inserting a variant with a discriminant out of step, makes Python decode a wrong termination or gauge name with no loud failure; the BA parity fixture would catch it only for the values its 15 cases happen to produce. This is the cross-language error-contract consistency A27 set out to settle.
- **Evidence:** `nls.rs:284-311` declares `Gradient = 0 …` and separately `pub const ALL: [Self; 7] = [Self::Gradient, …]`; `git grep 'ALL\['` finds no test asserting `ALL[i] as usize == i`.
- **Suggested fix:** One mechanism both ways: a core `fn code(self) -> usize` / `fn from_code(usize) -> Option<Self>` derived from `ALL`, and encode with `code()` (or `ALL.iter().position(..)`). At minimum, a `const` assertion or unit test per crossing enum that `ALL[i] as usize == i`.

#### [should-fix] Native benchmarks and their dataset readers have no place in the design doc's layout
- **Where:** `cpp/bench/foundations/{bal_reader.hpp,bench_foundations.cpp}`, `rust/slamlab-foundations/{benches/foundations.rs,examples/bal/reader.rs}`, `CMakeLists.txt` (`SLAMLAB_BUILD_BENCHMARKS`); design doc "Package layout" (`slamlab.eval` holds "Dataset readers, … benchmarks, budget harness")
- **Issue:** The PR adds a new top-level structure (`cpp/bench/<rung>/`, Rust `benches/` and `examples/`) and a third and fourth copy of the BAL conversion (axis flip, intrinsics layout, centred pixels) beside `slamlab.eval.readers.read_bal`. The design doc assigns readers and benchmarks to `eval` and is silent on native benchmarks; CLAUDE.md's layout section does not mention `cpp/bench` either. `check_layers` and `check.sh` were extended correctly, so the gap is documentation, not tooling.
- **Why it matters:** Milestone 7's budget harness will need native timing on edge targets, so where native benchmarks live and how they relate to `eval`'s harness is a design decision, not an accident of this PR. The native BAL readers carry geometric conventions with no test against the Python reader; the note's C++-vs-Rust headline (1.430 vs 1.441 s) assumes they solve the same problem, which is only confirmed indirectly through `demo_bal.py` (which uses the Python reader).
- **Evidence:** The design doc mentions benchmarks only at lines 14, 78, 267. `bal_reader.hpp`'s header claims "The same conversion as the Python reader … and the Rust example's", a parity claim with no test.
- **Suggested fix:** Change the doc, not the code: add a line to "Package layout" and CLAUDE.md (native benchmarks live in `cpp/bench/<rung>/` and each crate's `benches/`; developer tools outside the cores, held to the ladder by `check_layers`; `eval` keeps the Python benchmarks and budget harness). For the readers, a cheap parity anchor: have the example and the C++ bench print the starting RMS and assert it equals `demo_bal.py`'s, or check a tiny BAL text fixture against `read_bal` in the native tests.

#### [note] Rust could let nested summaries satisfy `solve::Summary`, as Python does; the design doc is silent on it
- **Where:** `rust/slamlab-foundations/src/ba/mod.rs:221` (`ba::Summary` does not implement `solve::Summary`); design doc "A summary that nests others"
- **Issue:** The doc says C++ returns a named struct because its concept "needs data members". Rust's `Summary` is a trait with methods, so `ba::Summary` could delegate `iterations`/`converged`/`termination` to `self.solve`, as Python's properties do. The doc does not say which Rust follows; today it copies the C++ named-struct pattern, consistent but not forced.
- **Suggested fix:** Decide before PR 6 brings `RefinedSummary` to Rust and record it in "Where the summary lives". Recommendation: implement `solve::Summary` for `ba::Summary` by delegation, so every Rust solver summary has the same three fields, keeping the named `Solution` structs.

#### [note] Compiled-backend lists are now literals in tests
- **Where:** `tests/foundations/test_ba.py:450,534,546,599`, `tests/foundations/test_so3.py:133` (`["cpp", "rust"]`); `tests/test_backends.py:56,185,229`
- **Issue:** These were derived from `backends` (`LOCKSTEP_BACKENDS - {"python"}`) and are now hand-written; a new backend or PR 6's lock-step change must edit several places.
- **Suggested fix:** Add `backends.COMPILED_BACKENDS` (or `BACKENDS[1:]`) and use it; fits PR 6's switch of Rust to lock-step.

#### [note] `rng.json` does not follow the fixture schema the conventions describe
- **Where:** `tests/fixtures/foundations/rng.json`; CLAUDE.md ("a parity fixture … that states its tolerance policy"); `tests/conftest.load_fixture` docstring ("`description`, `tolerance` … and `cases`")
- **Issue:** The fixture has `reference`/`samples` but no `tolerance` and no `cases`. Exact integer comparison is right for PCG32; only the shape diverges. (Hygiene's area; mentioned because it is now a schema three languages read.)
- **Suggested fix:** Add `"tolerance": {"comparison": "exact", "rtol": 0, "atol": 0}`, or document that non-array fixtures are exempt.

#### Checked and fine
- **Dependency direction:** `slamlab-foundations` depends only on nalgebra (criterion and serde_json are dev-dependencies); pyo3/numpy/pythonize/serde live only in `slamlab-py`, so the core builds without Python, as `cargo run --example bal` demonstrates. No reifyscene, torch or GPU dependency anywhere, so edge budgets are unaffected.
- **Rung placement:** everything is rung 0, inside the agreed Rust subset; RANSAC and the estimators correctly deferred to PR 6.
- **Bindings are thin:** one file per module; `batched!`/`map_batch` only convert float64 C-contiguous arrays; `nls_crossing.rs` mirrors C++'s `nls_crossing.hpp` as a shared non-module helper; `ba.rs` only marshals, validation stays in the core (`check_problem`, `check_options`); `py.detach` releases the GIL for the solve.
- **Extension tree mirroring:** `slamlab_rs.foundations.{alignment,ba,camera,se3,so3}` match the Python modules and pass the Protocol parameter-name test.
- **Error contract:** `error.rs` has the shared kinds and `core_error` maps each to its `slamlab.foundations.errors` class. Options-dict failures are `ValueError` with the same wording in both bindings (checked by the updated `test_ba` parametrisation).
- **A27:** core `name()` for `camera::Model`, `nls::Method`/`LossKind`, `ba::Gauge` in C++; binding tables and per-binding `decode`/`by_name` gone; `enum_by_name` in `common.hpp` also replaces `twoview`'s private copy.
- **A9:** one fixture, read by Python, native C++ (nlohmann_json, test-only, linked only into test executables) and Rust (serde_json dev-dependency).
- **Generic core, concrete bindings:** `nls::Problem`/`LinearSystem` and `ba::ReducedSystemSolver` are traits mirroring the C++ concepts; only concrete `ba.solve` is bound, so no compiled hot loop calls Python. `DenseCholesky` is the hard-wired default in both languages.
- **Shared components stay in rung 0:** `solve::Solution`/`Summary` and the Lie groups are reused by `ba`, not copied.
- **Layer tooling:** `check_layers.py` adds `cpp/bench` to the rung roots; `check.sh` builds, tidies and lints benchmarks and all Rust targets; benchmarks are off by default behind an explicit CMake option, and the BAL benchmark is gated by an explicit `SLAMLAB_BAL`.
- **No compatibility shims:** old binding name tables and messages removed outright, tests updated to the new wording.
- **Design doc and CHANGELOG:** the doc records the two-PR slice C plan and closes A9 and A27; the A6 retarget to PR 6 and the bump to 0.2.4.0 are present.

### Algorithm review

**Scope:** `git diff 3853b8cd...4ad3eb9` (PR 5, `feature/rust-core`). Read in full: `rust/slamlab-foundations/src/{so3,se3,camera,alignment,rng,nls}.rs`, `ba/{gauge,schur}.rs`; the numerically sensitive parts of `ba/bundle.rs`, `ba/mod.rs`, and both benchmark harnesses. Compared against `cpp/src/foundations/{so3,camera,nls}.cpp`, `cpp/include/slamlab/foundations/nls.hpp`, `cpp/src/foundations/ba/{schur.hpp,bundle.cpp}`, and the Python reference. Checked `docs/notes/rust-port.md` against those sources. Probes: the nalgebra 0.35 source in `~/.cargo/registry`; a scratch crate (built `--offline`, in the session scratchpad) on nalgebra SVD and NaN behaviour; a Python script timing COLMAP and the cpp/rust backends on ladybug-49; all under `systemd-run ... MemoryMax=8G`. (Importing `slamlab` made the editable install rebuild its git-ignored `build/slamlab` directory; no tracked file changed, `git status` clean.)

**Summary:** The port is faithful term by term to the C++ core: SO(3)/SE(3) Taylor switches, the near-π axis rule and the Barfoot Q series, the camera Newton undistortion and radial-to-radtan chain, Nielsen damping, the gain ratio and terminations, and the gauge logic including the zero-row padding of wide matrices. Re-measured ladybug-49: C++ 1.46-1.49 s, Rust 1.42 s, 33 iterations each with the same termination. Nothing is wrong in the formulations. The findings are two NaN/non-finite robustness gaps where nalgebra behaves differently from Eigen/numpy, and two problems in the note: its COLMAP time is multi-threaded, and some of its explanations are wrong or overstated.

#### [should-fix] A NaN gradient entry can be reported as convergence: the Rust gradient max drops NaN
- **Where:** `rust/slamlab-foundations/src/nls.rs` `try_step`: `let gradient_norm = gradient.iter().fold(0.0_f64, |m, g| m.max(g.abs()));`
- **Issue:** Rust's `f64::max` returns the other argument when one is NaN, so NaN entries are silently left out of the gradient's infinity norm. The Python reference uses `np.max(np.abs(gradient), initial=0.0)`, which propagates NaN; `NaN <= tol` is false, so Python never stops on a NaN gradient.
- **Why it matters:** If one gradient entry is NaN (a non-finite Jacobian at a finite cost) and the others are under `gradient_tolerance`, Rust stops with `Termination::Gradient`, which counts as converged, claiming convergence on a corrupted linearisation, while Python takes the other branch: a decision-level divergence from the reference. (C++ `cwiseAbs().maxCoeff()` with NaN is implementation-defined under Eigen's default `PropagateFast`.) The empty-gradient case is fine in all three (0, stops with `Gradient`).
- **Evidence:** In the scratch crate: `[f64::NAN, 0.0, 1e-12].iter().fold(0.0_f64, |m, x| m.max(x.abs()))` printed `fold max with NaN: 0.000000000001`.
- **Suggested fix:** Propagate NaN, e.g. `fold(0.0, |m, g| if g.is_nan() || m.is_nan() { f64::NAN } else { m.max(g.abs()) })`, and the same in C++ via `maxCoeff<Eigen::PropagateNaN>()` to keep the cores in step; add a native test with a NaN gradient entry.

#### [should-fix] nalgebra's general SVD has no iteration cap: a non-finite matrix hangs instead of failing
- **Where:** `rust/slamlab-foundations/src/ba/gauge.rs`: `null_space` (`padded.svd(false, true)`), `singular_values` (`svd(false, false)`), `canonical_span` (`svd(true, false)`)
- **Issue:** `Matrix::svd` calls `new_unordered(..., max_niter = 0)`, meaning no iteration limit. With a NaN or Inf in the matrix, the Golub-Kahan loop never converges. LAPACK (numpy) and Eigen terminate and return NaN or an error code. The fixed 3x3 path (`project_to_so3`, Umeyama) uses nalgebra's separate `svd3` routine, which does terminate on NaN.
- **Why it matters:** A hang is the one failure a caller cannot observe or recover from (CLAUDE.md: "make failure observable"). In this PR the gauge matrices are built from validated finite inputs, and poses at ±1e308 through all three backends did not hang, so the hang is not shown reachable from `ba::solve` today; but any intermediate overflow is a hazard, and it matters more for PR 6, where DLT, eight-point and homography take SVDs of data-derived dynamic matrices and the native crate is meant to run standalone, without Python's finiteness checks.
- **Evidence:** Scratch probe `svdprobe` (7x7 `DMatrix`, `.svd(false, true)`) under `timeout 10`:
  ```
  ok: [3.745..., 3.179..., ...]  [exit 0]
  nan:   [exit 124]
  inf:   [exit 124]
  ```
  (124 = killed by the timeout.) nalgebra 0.35 `svd.rs:93` passes `max_niter` 0 to `try_new_unordered`.
- **Suggested fix:** Route every dynamic SVD through one helper using `try_svd(compute_u, compute_v, f64::EPSILON * 5.0, cap)`, mapping `None` to `Degenerate`/`NotConverged`, and check finiteness first; add a native test feeding a NaN matrix and expecting an error, not a hang.

#### [should-fix] The note compares single-core backends with a multi-threaded COLMAP time without saying so
- **Where:** `docs/notes/rust-port.md` "Measured" table ("COLMAP (Ceres) | 0.9147 | -- | 0.36 s"); `src/slamlab/eval/colmap_reference.py:221-235`
- **Issue:** COLMAP's `solver_options.num_threads` defaults to `-1` (all cores, 24 here), and ladybug's 63,686 residuals exceed COLMAP's `min_num_residuals_for_cpu_multi_threading` of 50,000, so Ceres runs threaded. The C++ (Eigen without OpenMP) and Rust (nalgebra) cores run on one core, and the note says "one core" for its native table.
- **Why it matters:** The comparison overstates Ceres's lead by about 2x; a reader learning from the ladder would take away the wrong reason for the gap.
- **Evidence:** COLMAP with the demo's settings (tolerance 1e-6, 100 iterations), twice each:
  ```
  COLMAP threads=-1: 0.368 s / 0.352 s
  COLMAP threads=1:  0.714 s / 0.702 s
  cpp 1.489 s / 1.458 s, rust 1.422 s / 1.416 s (33 iterations, 'function')
  ```
- **Suggested fix:** Report COLMAP single-threaded (`solver.num_threads = 1`, ~0.71 s) beside the single-core backends, or both numbers with the thread count stated. The remaining ~2x deserves one sentence: Ceres's Schur solver exploits the block structure.

#### [should-fix] Two claims in the note are contradicted by the code or the measurements
- **Where:** `docs/notes/rust-port.md` "Measured" (the radtan explanation) and "What was learned" ("Bit-level agreement survives a change of linear-algebra library"; "need to agree to the last few bits")
- **Issue:**
  1. The note explains the projection gap (C++ 10.6 ns, Rust 20.0 ns) by Rust copying the intrinsics into a nine-element array while "the C++ one passes an `Eigen::Ref` straight through." It doesn't: `expand` in `cpp/src/foundations/camera.cpp:245-253` builds `Expanded{.model = model, .intrinsics = intrinsics}` where `Expanded::intrinsics` is an `Eigen::VectorXd`, a heap-allocated copy per call, more expensive than Rust's stack array, and C++ is still faster. The stated cause is unmeasured and points the wrong way.
  2. The results are not bit-identical: ladybug final costs C++ 13308.457897884755 vs Rust 13308.457898007191 (9e-12 relative); initial costs differ in the last digits. The BA fixture holds values to `rtol` 1e-9 / `atol` 1e-8-1e-9; only the codes (iterations, terminations, gauges) are exact.
- **Why it matters:** The note is a teaching artefact. "Bit-level" teaches the wrong lesson; what carried over is decision-level agreement, which the design (decisions depend on spans, not bases) was built to give without last-bit agreement.
- **Suggested fix:** Say "decision-level agreement (exact iteration counts, terminations and gauge codes; values within 1e-9)"; drop the radtan causal claim or replace it with a profiled one.

#### [note] Native cores differ on NaN where Python validates first
- **Where:** `so3.rs` `check_rotation`, `vee`, `se3::check_transform` (nalgebra `amax` ignores NaN via `simd_max`); `nls.rs` `DenseSystem::solve`, `ba/schur.rs` `eliminate`/`camera_variances`, `ReducedSystemSolver` in `ba/mod.rs:101-102` (nalgebra `cholesky()`)
- **Issue:** nalgebra's Cholesky returns `None` on a NaN pivot (`try_sqrt` tests `>= 0`); Eigen's LLT only fails on `x <= 0`, so it passes NaN through as `Success`; LAPACK `potrf` fails on NaN. A Gauss-Newton solve on a NaN system therefore ends `Singular` in Rust and `NoDecrease` in C++. Separately, a rotation with a NaN entry passes the Rust `check_rotation` (`amax` skips the NaN row and column of the Gram matrix; `det <= 0.0` is false for NaN). Through Python every input is finite-checked first (`arrays.py:64`), so this only affects native callers (the BAL example, future edge budgets).
- **Suggested fix:** None needed for parity through Python. If the crate is meant for standalone use, add an `is_finite` check to the native `check_*` functions.

#### [note] The 3x3 SVD is a different algorithm from the general one; checked
- **Where:** `so3.rs` `project_to_so3`, `alignment.rs` `umeyama` (`Matrix3::svd`)
- **Issue:** For `Matrix3<f64>`, nalgebra 0.35 dispatches `svd` to `svd3::svd_ordered3`, an eigendecomposition of MᵀM followed by QR (McAdams et al.), not Golub-Kahan. Forming MᵀM squares the condition number of V. A small cosmetic slip in that routine (`swap(&mut rho0, &mut rho2)` in the third branch) is harmless.
- **Evidence (probe):** 1e5 random seeded matrices: worst reconstruction 6.5e-15, worst singular-value difference from the general `DMatrix` SVD 2.7e-15, worst U orthogonality 2.0e-15. Rank-1, rank-2 and an ill-conditioned matrix (σ₃ = 1.67e-10) all reconstruct to ~1e-14 with the correct σ₃. A NaN matrix terminates. The SVD fixture cases pass on Rust, including "reflection-like".
- **Suggested fix:** None required; consider a native Umeyama test near `DEGENERACY_RATIO` pinning the classification.

#### [note] Undocumented divergence when the anchor's pixel Jacobian is exactly singular
- **Where:** `rust/slamlab-foundations/src/ba/bundle.rs` `anchors`: `-d_pixel_d_ray.try_inverse()? * ...`
- **Issue:** When `det(d_pixel_d_ray) == 0`, Rust drops the anchor's Jacobian (`None`), while C++ (`bundle.cpp:454`, `.inverse()`) produces inf/NaN entries and Python's `np.linalg.inv` would raise. It needs an exactly zero determinant at a converged undistortion, so it is practically unreachable.
- **Suggested fix:** Document the choice in a comment, or make the cores agree; the Rust behaviour is arguably the right one.

#### [note] A branch in `unproject` cannot be reached and has no test
- **Where:** `rust/slamlab-foundations/src/camera.rs` `unproject`: `let Some(step) = jac.lu().solve(&residual) else { ... "singular distortion Jacobian" }`
- **Issue:** The `det <= 0` check just before it guarantees an invertible 2x2, so this error path is dead. C++ has no equivalent.
- **Suggested fix:** `try_inverse` with an `expect` explaining why it cannot fail, or keep it and note it as a guard.

#### [note] Context on the field
- For dense Cholesky in Rust, faer is much faster than nalgebra at the sizes milestone 4 will reach: faer's published benchmarks give n=256: 162 µs vs nalgebra 607 µs; n=1024: 7.2 ms vs 122.8 ms ([faer-rs](https://github.com/cramt/faer-rs), [faer-cholesky docs](https://docs.rs/crate/faer-cholesky/^0.16.0)). At ladybug's 441-wide reduced system the time is dominated by Jacobians and elimination, not the factorisation; this belongs with milestone 4's sparse solver seam, not this PR.
- The remaining ~2x to single-threaded Ceres matches the standard block-sparse Schur exploitation (Triggs et al., "Bundle Adjustment -- A Modern Synthesis", 1999; Agarwal et al., "Bundle Adjustment in the Large", ECCV 2010).

#### Checked and fine
- **SO(3)** (`so3.rs`): `sinc`, the half-versine form, the Jacobian and inverse-Jacobian series and thresholds are term for term with `so3.cpp`; `log` uses `atan2` with the clamped cosine; the near-π symmetric-part axis has the same sign rule. nalgebra's `imax`/`iamax` take the first of equal values (like `np.argmax` and Eigen `maxCoeff`), so Shepperd's branch and the axis column choice match.
- **SE(3)** (`se3.rs`): the Q matrix checked against Barfoot's closed form; series coefficients derived term by term (1/(2k+3)!, 1/(2k+4)!, (k+1)/(2k+5)!); ten terms below θ=1 truncate at ~1e-23. `left_jacobian_inverse` = [J⁻¹, −J⁻¹QJ⁻¹; 0, J⁻¹] is correct.
- **Cameras** (`camera.rs`): radtan distortion and Jacobian match the OpenCV form; intrinsics columns k1, k2, p1, p2, k3 correct; the radial-to-radtan chain (column fx + column fy, then one to one) is correct; the Newton loop, `det <= 0` guard, 30-iteration cap and `NotConverged` reporting are identical to C++.
- **Umeyama** (`alignment.rs`): weighted means and covariance, the `det(U)·det(Vᵀ)` reflection sign, and scale = `d·sign / spread` follow Umeyama (TPAMI 1991); the σ₂/σ₁ collinearity test is right.
- **PCG32** (`rng.rs`): `pcg32_srandom_r`, XSH-RR and bounded rejection with `(-bound) % bound` match O'Neill's reference; Floyd's sampling is correct, and the shared `rng.json` pins the draws.
- **NLS** (`nls.rs`): loss ρ and IRLS weights (Huber 2c√s − c², weight c/√s; Cauchy c²·log1p(s/c²), weight 1/(1+s/c²)); Nielsen damping, `predicted = ½(δᵀDδ − gᵀδ)` (Ceres's model decrease), NaN ratio when the prediction is not positive, and the acceptance rule all match `nls.hpp:349-441`. `clamp` agrees with Eigen's `cwiseMax`/`cwiseMin` on finite values and NaN; `check_options` guarantees `min ≤ max`, so `clamp` cannot panic. An empty gradient gives 0 in all three cores.
- **Cholesky:** nalgebra's failure rule (pivot zero or negative) matches Eigen LLT's `x <= 0` for finite matrices; both read the lower triangle and the Schur matrix is symmetrised first. `Cholesky::inverse()` is `solve_mut(I)`, the same as Eigen's `LLT::solve(Identity)`.
- **Gauge** (`gauge.rs`): zero-row padding of wide matrices keeps the row space and right singular vectors; the general `svd()` sorts descending and dynamic matrices never take the 2x2/3x3 special paths; `canonical_span` is a projector-based, basis-independent span; the Householder `complement`, tie-break margins and `classify` rules match C++; the native wide-matrix null-space test exists.
- **Schur** (`schur.rs`): point blocks with inert identity on unused dimensions, damped elimination, upper-triangle accumulation then symmetrisation, back-substitution, and `camera_variances` returning infinity when singular all match `schur.hpp:205-260`.
- **Parity coverage:** all property tests parametrised over `backends.BACKENDS`; BA fixture codes compared exactly; the hard SO(3) cases ("reflection-like", near π, Taylor boundaries) are in the shared fixtures.
- **BA benchmark fairness:** both Release (`-O3`, no `-march`, no OpenMP), one thread, the same options (100 iterations, `gauge none`) and the same reader; reproduced C++ 1.46-1.49 s and Rust 1.42 s through the bindings.

Probe scripts in the session scratchpad (outside the repo): `svdprobe/`, `colmap_threads.py`, `hang.py`.

Sources: [faer-rs](https://github.com/cramt/faer-rs); [faer-cholesky docs](https://docs.rs/crate/faer-cholesky/^0.16.0); [nalgebra issue #1072 (SVD accuracy)](https://github.com/dimforge/nalgebra/issues/1072).

## Run 2 -- 2026-10-04

| | |
|---|---|
| **Branch** | `feature/rust-core` |
| **Range** | `4ad3eb9e54850d978262a4ec1d9d4a9278ed2a07...f80e4896dd6bb2ec507039ed5a719129fe810cc5` (the fixes after run 1) |
| **Reviewers** | architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`): 1692 pytest passed, 20 skipped; 18 ctest; cargo clean |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L10 | should-fix | algorithm | `cpp/tests/foundations/test_bal_reader.cpp` | The C++ reader test's comparison passes NaN | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| L11 | should-fix | algorithm | `docs/notes/rust-port.md` | The new explanation of the COLMAP gap contradicts COLMAP's own solver choice (DENSE_SCHUR at 49 images) | fix | fixed (979aaae) -- decided by evan-wes, 2026-10-04 |
| L12 | note | algorithm | `ba/gauge.rs` plan | A failed SVD in the gauge plan reads as "nothing pins anything", not a rejected step | fix | fixed (169cfad; Rust only: C++'s SVD cannot hang, and both cores check the inputs finite) -- decided by evan-wes, 2026-10-04 |
| L13 | note | algorithm | `slamlab-py` `encode` | -1 decodes as the last enum name in Python (same as A12) | fix (with A12) | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| L14 | note | algorithm | `docs/notes/rust-port.md` | "finiteness checks ahead of every tolerance comparison" overstates (gauge `classify` uses `amax`) | fix | fixed (979aaae) -- decided by evan-wes, 2026-10-04 |
| A8 | should-fix | architecture | `scripts/check_layers.py` | The Rust order check misses glob imports, `pub use` in `mod.rs`, a mid-file `#[cfg(test)]`, and `//` in strings | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| A9 | should-fix | architecture | `cpp/tests/foundations/test_bal_reader.cpp`, `check_layers.py` | A relative `../../bench/...` include escapes the cross-rung check | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| A10 | should-fix | architecture | `tests/fixtures/eval/generate_bal.py` | The BAL fixture omits `intrinsics_idx` and every defaulted field | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| A11 | should-fix | architecture | `ba/mod.rs`, `cpp/.../ba.hpp` | `Problem::with_defaults` is Rust-only; the C++ bench fills defaults by hand | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| A12 | should-fix | architecture | `slamlab-py/src/common.rs` | `encode`'s -1 fallback reads as a valid name; the pin tests cannot catch a variant missing from `ALL` | fix | fixed (169cfad) -- decided by evan-wes, 2026-10-04 |
| A13 | note | architecture | `ba/mod.rs`, `_CPP_FACADES` | The BA facade stands for `problem` but holds `solve` (A24, now in two languages) | defer | deferred (design doc open items, with A24) -- decided by evan-wes, 2026-10-04 |
| A14 | note | architecture | Rust `examples/`, `cpp/bench` | Where the BAL readers live is asymmetric | no action | waived: no action -- decided by evan-wes, 2026-10-04 |
| A15 | note | architecture | `so3.rs` `project_to_so3` | Uses nalgebra's SVD without the cap | dispute | disputed: nalgebra sends a 3x3 matrix to its separate svd3 routine, which run 1's probe showed returns on NaN; the hang is only in the general path, which the capped helper now covers -- decided by evan-wes, 2026-10-04 |

### Architecture review (run 2)

**Scope:** Run 2 of PR 5 (`feature/rust-core`): `git diff 4ad3eb9...f80e489` (8 commits). Checked the fixes against the run-1 findings A1-A7 in `docs/reviews/2026-10-04-feature-rust-core.md`: `scripts/check_layers.py` and `tests/test_check_layers.py`; `tests/test_backends.py`, `tests/conftest.py`, `src/slamlab/backends.py`; the BAL fixture and the native BAL readers and tests; the Rust binding changes (`common::encode`, `ba.rs`, `nls_crossing.rs`); the new `ba::Problem::with_defaults`; the finiteness changes in both cores; the CLAUDE.md and design-doc edits. Probed the new Rust path checker and the C++ rung check with scratch repos outside the tree, and called the three backends on NaN input.

**Summary:** A1, A3 (but one point), A4, A6 and A7 are fixed correctly, and A5 is properly deferred. A2 works: on the real tree the Rust module graph is complete and right. But it has blind spots that CLAUDE.md's new wording ("held to the same contracts") does not admit: glob imports (`use super::*`) followed by a bare path to a sibling module, re-exports (`pub use`) in a package's `mod.rs`, and `#[cfg(test)]` anywhere but right before the trailing test module. Each passes silently today. Separately, the fixes introduce three new weaknesses: the new C++ test uses a relative include (`../../bench/...`) the cross-rung check cannot see; the BAL fixture leaves out exactly the fields the readers fill by default, including `intrinsics_idx`; and `encode`'s `-1` fallback becomes a valid-looking name on the Python side. No must-fix.

#### [should-fix] The Rust order check has silent gaps: glob imports, re-exports in `mod.rs`, and a `#[cfg(test)]` not at the end of the file
- **Where:** `scripts/check_layers.py:247-357` (`_RUST_PATH`, `_rust_code`, `_rust_paths`, `_rust_target`, `_rust_graph`); the claim it supports is in `CLAUDE.md:55-57` and the design doc.
- **Issue:** Edges come only from literal `crate::`/`super::` paths. Three ordinary Rust patterns create a real upward dependency the checker never sees:
  1. **Glob imports.** `use super::*;` in `ba/schur.rs` brings in `ba`'s private `mod bundle;`; `bundle::adjust()` is then a bare path and records no edge.
  2. **Re-exports.** `pub use bundle::adjust;` in `ba/mod.rs`, then `use super::adjust;` in `ba/gauge.rs`. The checker maps this to the facade, which counts as `problem` (the bottom layer), so a `gauge` -> `bundle` dependency passes. `mod.rs`'s own bare paths to its children (`layout::make_layout`, `bundle::adjust` at `ba/mod.rs:644-645`) also produce no edges; harmless today only because the facade counts as `solve` (the top layer) when it does the using.
  3. **`#[cfg(test)]` cut-off.** `_rust_code` drops everything after the *first* `#[cfg(test)]` in the file, though its docstring says it drops only "the `#[cfg(test)]` module at its end". A test-only helper in the middle of a file would silently exempt every line below it.

  A smaller fourth gap: `//` is stripped even inside string literals, so `"http://x"; use crate::se3;` on one line hides the `use`.
- **Why it matters:** A2 exists because PR 6 adds five estimation modules to this crate. Glob imports and re-exports are idiomatic Rust, and either one turns the check into a false pass. "Chains count, as in C++" reads as parity it does not have.
- **Evidence:** Each case run through `check_rust_internal` on a fake crate with the real `pyproject.toml` orders:
  ```
  glob_then_bare     use super::*; ... bundle::adjust();           -> PASS (no violation)
  reexport_launder   mod.rs: pub use bundle::adjust; gauge: use super::adjust; -> PASS
  cfg_test_midfile   #[cfg(test)] fn helper() {} \n use super::bundle;   -> PASS
  string_url         const U: &str = "http://x"; use crate::se3;  (in so3)  -> PASS
  ```
  Controls are caught (`/* crate::se3 */`, `$crate::se3`, inline `mod` blocks: false positives, which at least fail loudly).
- **Suggested fix:** Make the checker fail loudly instead of guessing: report any `use crate::…::*` or `use super::*` outside the test module, and any `pub use` in a rung crate's `mod.rs`/`lib.rs` (none exists today); cut the test module only where `#[cfg(test)]` is directly followed by `mod tests` and nothing follows its closing brace, otherwise report an error; one test per rule; list the remaining limits in the module docstring next to the C++ facade caveat (A24).

#### [should-fix] The new C++ BAL test's relative include escapes the cross-rung check
- **Where:** `cpp/tests/foundations/test_bal_reader.cpp:11` (`#include "../../bench/foundations/bal_reader.hpp"`); `scripts/check_layers.py` `check_cpp`, which matches only `slamlab/<rung>/` includes (`_INCLUDE`, line 58).
- **Issue:** The first include in the tree that crosses from `cpp/tests` into `cpp/bench`, by a relative path. `check_cpp` never resolves relative quoted includes, so the same pattern pointed at another rung would pass.
- **Why it matters:** The design doc says native benchmarks "are held to the ladder like the code they time". With `#include "../../bench/<rung>/..."` now a precedent, a foundations test or bench can depend on an edge rung's bench header unseen.
- **Evidence:** A fake repo with `cpp/tests/foundations/test_x.cpp` containing `#include "../../bench/edge/foo.hpp"` gives `check_cpp(...) == []`; the control, `#include "slamlab/edge/foo.hpp"` from `cpp/bench/foundations/bar.hpp`, gives `rung 'foundations' may not include from rung 'edge'`.
- **Suggested fix:** Resolve quoted relative includes in `check_cpp` (as `_include_graph` already does with `..`) and take the target's rung from its resolved path under any `_CPP_RUNG_ROOTS` entry; or give test targets `cpp/bench` as an include directory and extend the regex. Add a test case either way.

#### [should-fix] The BAL fixture leaves out the fields the native readers fill by default, so the defaults can drift
- **Where:** `tests/fixtures/eval/generate_bal.py:36-45`; `cpp/tests/foundations/test_bal_reader.cpp:83-104`; the Rust reader's test in `rust/slamlab-foundations/examples/bal/reader.rs:117+`; `rust/slamlab-foundations/src/ba/mod.rs:206-238` (`with_defaults`); `cpp/bench/foundations/bal_reader.hpp`.
- **Issue:** The fixture holds `model, poses, intrinsics, fixed_intrinsics, points, observations, cam_idx, pt_idx`. It omits `intrinsics_idx` (part of the BAL conversion itself, one group per camera, which both native readers override) and every defaulted array (`weights`, `fixed_poses`, `fixed_points`, `depth_prior(_weights)`, `depth_affine`, `fixed_depth_affine`, `pose_prior(_weights)`, `point_anchor`). The C++ test also never checks `model`. These defaults now live in three hand-written places (`BAProblem.create`, `Problem::with_defaults`, the C++ reader), and nothing ties them together.
- **Why it matters:** A4's goal was that "the three cannot drift apart". Two of the three can disagree on intrinsics grouping or a default without any test failing, changing what the benchmarks time.
- **Evidence:** `bal.json`'s `expected` keys are `['model', 'poses', 'intrinsics', 'fixed_intrinsics', 'points', 'observations', 'cam_idx', 'pt_idx']`.
- **Suggested fix:** Have `generate_bal.py` write every `BAProblem` field, and both native tests compare all of them, `model` included.

#### [should-fix] `ba::Problem::with_defaults` exists in the Rust core only; C++ has no equivalent
- **Where:** `rust/slamlab-foundations/src/ba/mod.rs:185-239`; missing from `cpp/include/slamlab/foundations/ba.hpp:94+`; `cpp/bench/foundations/bal_reader.hpp` sets every default by hand.
- **Issue:** The constructor belongs in the core (the native form of `BAProblem.create`'s defaults; standalone callers need it), but it makes the two cores' public BA surfaces differ, and the C++ bench repeats the logic inline.
- **Why it matters:** The cores are meant to mirror each other and the Python reference; a Rust-only constructor is how the defaults end up in three places.
- **Suggested fix:** Add the same constructor to the C++ core (e.g. a static `Problem::with_defaults(...)`) and use it in `bal_reader.hpp`; then the extended BAL fixture covers both cores' defaults. Or record a Rust-only choice in the design doc.

#### [should-fix] `encode`'s `-1` fallback reads as the last enum name on the Python side
- **Where:** `rust/slamlab-py/src/common.rs:272-288`; read in `src/slamlab/foundations/ba/solve.py:208` and `nls.summary_from_backend`; pin tests at `ba/mod.rs:688-694`, `camera.rs:496-502`, `nls.rs:1203+`.
- **Issue:** Python indexes a list with the code, so `-1` silently gives the last name (`"partial"` for gauges) instead of an error. The doc comment says the core tests rule this out, but each test iterates over `ALL`, so a variant added to the enum but not to `ALL` passes and crosses as `-1`.
- **Why it matters:** The one failure mode left produces a plausible wrong answer instead of failing loudly.
- **Evidence:** `codes["gauge"][-1]` is a valid Python index; the test body never looks at variants outside `ALL`.
- **Suggested fix:** Make `encode` return `PyResult<i64>` and raise on a missing value (or `expect`), or make the pin tests complete with an exhaustive `match` checking `ALL.contains(&v)`. Rejecting negative codes on the Python side costs one line.

#### [note] The BA facade stands for `problem` but also holds `solve`; same as C++ (A24)
- **Where:** `rust/slamlab-foundations/src/ba/mod.rs:550-615` (`centring_origin`, `solve`), `scripts/check_layers.py:64-66` (`_CPP_FACADES`).
- **Issue:** `use super::solve;` or `super::centring_origin(...)` from `ba/bundle.rs` or `ba/gauge.rs` is an upward dependency on the `solve` layer, but maps to `problem` and passes. C++ has the same gap (`ba.hpp` declares `solve`): the deferred A24, now in two languages.
- **Suggested fix:** When A24 is taken up, move `solve`, `centring_origin` and `shifted` into `ba/solve.rs`, mirroring Python's `ba/solve.py`, so `super::solve` resolves to the module and is checked. Note under A24 in the design doc that it now covers Rust too.

#### [note] Where the BAL readers live and are tested is asymmetric
- **Where:** `rust/slamlab-foundations/examples/bal/reader.rs` (pulled into `benches/foundations.rs` via `#[path]`; tested through `[[example]] test = true`); `cpp/bench/foundations/bal_reader.hpp`, tested from `cpp/tests/foundations/`.
- **Issue:** The Rust reader is shared code under `examples/`; the C++ reader lives under `bench/`; the C++ test of a bench tool sits in a directory that otherwise mirrors package modules. All of it works, is documented, and stays outside the cores.
- **Suggested fix:** None now. If a second dataset reader arrives, give each language one home for bench support, with tests next to it.

#### [note] (numerical reviewer) The capped SVD is private to `ba/gauge`; `so3::project_to_so3` still uses the uncapped SVD
- **Where:** `rust/slamlab-foundations/src/ba/gauge.rs:28-49`; `rust/slamlab-foundations/src/so3.rs:488-490` (`matrix.svd(true, true)`).
- **Issue:** The helper's comment says nalgebra's uncapped SVD "never returns on a non-finite matrix". `project_to_so3` is public, infallible, and documented as taking "any real 3x3 matrix". Python rejects NaN before calling it, but a direct Rust caller with a NaN matrix would hang. If more modules need a capped SVD, it belongs in a shared rung-0 module.

#### Checked and fine
- A1: the Protocol test requires `provided == members` for every compiled backend; `require_function` skips only a module Rust lacks and fails on a missing function; `LOCKSTEP_BACKENDS`' docstring, `backends.function`'s docs and the design doc agree.
- A2 on the real tree: the dumped Rust graph matches the actual `use` lines; `_check_graph` and the `verb` parameter generalise the C++ path cleanly; the bindings crate is excluded; inline `mod` blocks and block comments err toward false positives.
- A3: no `as i64` enum casts remain in `slamlab-py`; codes go through `encode` from the same `ALL` as `decode` and `enum_names`.
- A4: native benchmarks placed in CLAUDE.md and the design doc; one fixture read by the generator, the Python reader test, the Rust example test and the C++ test (gaps above).
- A5 recorded with a trigger (PR 6); A6 `COMPILED_BACKENDS` used in all parametrisations; A7 `rng.json` has its tolerance.
- Error contracts: the new finiteness checks use the same kind and messages in both cores; all three backends raise `InvalidInputError` on NaN for `vee`, `log`, `from_quaternion`; the C++ `PropagateNaN` gradient norm matches Rust and numpy.
- The BA binding split (`solution_dict`) only converts types; the docstring lists the argument shapes.
- Native Rust BA tests live in the crate's trailing test module, with no Python dependency.
- No new crate dependencies, nothing heavy reachable from rung 0, no import-time side effects, no compatibility shims.

### Algorithm review (run 2)

**Scope:** Run 2 of PR 5, the fixes in `git diff 4ad3eb9...f80e489` (8 commits), against run-1 findings L1-L8 and H2/A3. Read the changed algorithm code and tests in both cores (`rust/slamlab-foundations/src/{nls,alignment,so3,se3,camera}.rs`, `src/ba/{gauge,mod,bundle,schur}.rs`, `cpp/include/slamlab/foundations/nls.hpp`, `cpp/src/foundations/{so3,se3,alignment}.cpp`, the native tests), the BAL fixture and readers, and `docs/notes/rust-port.md`.

**Summary:** Every run-1 algorithm finding is fixed correctly. The NaN-propagating gradient norm is right in both cores and its new tests would fail without the fix; the capped SVD gives the same results as before on finite input and cannot hang; the finiteness checks are in the right places. Nothing is must-fix. The two should-fix items are small: a NaN-blind comparison in the new C++ BAL-reader test (with a fixture that leaves out `intrinsics_idx`), and a new cause in the rust-port note that COLMAP's own solver choice contradicts.

#### [should-fix] The C++ BAL-reader test lets NaN through, and the shared fixture leaves out `intrinsics_idx`
- **Where:** `cpp/tests/foundations/test_bal_reader.cpp:50`; `tests/fixtures/eval/generate_bal.py` (`expected`); `tests/fixtures/eval/bal.json`
- **Issue:** `close()` rejects only when `std::abs(got[i] - expected[i]) > atol`, which is false for NaN, so a NaN pose, point or observation from the C++ reader passes (the Rust test checks `(g - w).abs() <= atol`, the right way round). The fixture also lacks `intrinsics_idx`, which each reader sets on its own (Python `np.arange(n_cams)` at `src/slamlab/eval/readers.py:116`, Rust in `examples/bal/reader.rs`); nothing checks they agree.
- **Why it matters:** `intrinsics_idx` decides whether BAL's per-camera intrinsics stay per-camera or collapse into one group, which changes the problem completely; a drift there, or a NaN from the C++ reader, would pass unseen.
- **Evidence:** `if (std::abs(got[i] - expected[i]) > atol) { return false; }`; the fixture's `expected` keys omit `intrinsics_idx`.
- **Suggested fix:** `!(std::abs(...) <= atol)` (or `std::isfinite` first); add `intrinsics_idx` to the fixture and assert it in all three tests.

#### [should-fix] The rust-port note's new explanation of the COLMAP gap contradicts COLMAP's own solver choice
- **Where:** `docs/notes/rust-port.md:100-105` ("Ceres builds and factors the reduced camera system block by block; ours forms it as one dense matrix. That is the likely difference")
- **Issue:** ladybug-49 has 49 images; COLMAP chooses `DENSE_SCHUR` at 50 images or fewer on CPU, which also forms the reduced camera system as one dense matrix (441 square), and `colmap_reference.solve` does not override that.
- **Why it matters:** L4 was resolved by removing an unprofiled cause; this replaces it with another, one the evidence argues against, and would steer milestone 4's sparse-solver work as if it closed this gap.
- **Evidence:** pycolmap 4.2.1 in the slamlab env: `ceres.max_num_images_direct_dense_cpu_solver 50`, `use_gpu False`; COLMAP's `src/colmap/estimators/bundle_adjustment.cc` sets `DENSE_SCHUR` when `num_images <= max_num_images_direct_dense_cpu_solver`. Unmeasured candidates: Ceres's fixed-size `SchurEliminator<2,3,9>` specialisations, and the iteration count (the table shows `--` for COLMAP; fewer iterations alone could explain 2x).
- **Suggested fix:** Report COLMAP's iteration count, and either profile or say "not profiled"; drop the dense-matrix explanation.

#### [note] A failed SVD in the gauge plan reads as "nothing pins anything", not as a rejected step
- **Where:** `rust/slamlab-foundations/src/ba/gauge.rs` `remaining`/`plan`/`best_candidate`, and the doc comments on `null_space` and `singular_values`
- **Issue:** Inside `confinement` the NaN fallback works as documented (the step is rejected). In the plan, decided once at the start, a failed SVD gives NaN singular values, rank 0, a NaN 7x7 basis classified `Similarity`, every candidate rank 0, nothing confined, `free = 7`: the solve runs with the whole similarity free and reports "similarity". The doc comment's "the step is rejected rather than taken" does not hold on this path.
- **Why it matters:** Little in practice: inputs are checked finite, nalgebra scales before iterating so a finite matrix cannot overflow, and every gauge SVD is at most 7 wide, far from the cap. It is defence in depth and not silent (`free_gauge = 7` is reported), but the claim and the behaviour differ.
- **Evidence:** the path above; the new test asserts `rank_of(&singular_values(&m)) == 0`, which encodes the "sees nothing" reading.
- **Suggested fix:** Reword the comments, or propagate the SVD failure as a `Result` from `remaining`/`plan` so `solve` returns an error, which fits "failure is observable" better.

#### [note] `encode`'s -1 decodes on the Python side as the last enum name (architecture's area)
- **Where:** `rust/slamlab-py/src/common.rs` `encode`; `src/slamlab/foundations/nls.py:751`, `ba/solve.py:208`, `ransac.py:405`
- **Issue:** -1 indexes the last name ("singular", "partial"); `codes_are_positions` iterates over `ALL` only and cannot catch a variant missing from it.
- **Why it matters:** Unreachable today, but it would misreport a termination with no error.
- **Suggested fix:** Have the Python decoders reject negative codes, or `encode` return a `PyErr`.

#### [note] "finiteness checks ahead of every tolerance comparison" claims a little more than the code does
- **Where:** `docs/notes/rust-port.md:162-166`
- **Issue:** `gauge.rs` `classify` still uses nalgebra `amax()` (drops NaN), and C++ `gauge.cpp:233,236` the default `maxCoeff()`; both reachable only from the NaN-basis path above.
- **Suggested fix:** Say "ahead of every input check", or leave it.

#### Checked and fine
- **L1, Rust:** `max_abs` is NaN when any entry is, 0 for an empty vector; the new test (NaN at index 2, at the minimum) fails without the fix; after a NaN gradient the step-norm and acceptance tests cannot read as convergence either.
- **L1, C++:** `maxCoeff<Eigen::PropagateNaN>()` is valid for the installed Eigen 5.0.1-dev; probed at `-O0` and `-O3 -march=native`: with NaN at index 1, 2 or 7 the default gives 0 and PropagateNaN NaN; the C++ test's NaN at index 2 discriminates.
- **L1, Python:** `DenseSystem.build` refuses a non-finite Jacobian, asserted for both methods; the compiled cores differ on purpose (not converged instead of an error), as the test docstring says.
- **L2:** `try_svd(5·EPSILON, 10_000)` uses `svd()`'s epsilon and sorts the same way, so finite results are unchanged (the BA fixture confirms); the finiteness pre-check catches NaN and inf before any iteration.
- **L5/H2:** finiteness checked first in `check_rotation`, `vee`, `from_quaternion`, `check_transform`, `umeyama`, both cores, with native tests.
- **L7:** the comment is accurate. **L8:** the `expect` cannot fire: the guard uses the same determinant formula `try_inverse` does, which fails only on an exact zero; a NaN Jacobian gives `Some(NaN)` and ends `NotConverged`.
- **A3:** `encode`/`decode` both use the position in `ALL`; `codes_are_positions` pins every crossing enum.
- **`Problem::with_defaults`:** every default matches `BAProblem.create`.
- **Native BA tests:** sound (recovery to 1e-9/1e-8 with convergence asserted, single-view drop, invalid inputs with messages, the reduction round trip and diagonal).
- **BAL fixture:** inputs not exactly representable in float32; `atol 1e-13` (about 45 ulp at translations of about 5) suits it; policy stated.
- **L3/L4:** the one-thread COLMAP row and "decision-level agreement" are accurate; the 9e-12 cost difference is stated.
- **Rust tests:** `cargo test -p slamlab-foundations --all-targets` at f80e489 passed 42 tests plus the BAL reader test; the repository was left unmodified.

Sources: [COLMAP source](https://github.com/colmap/colmap); [COLMAP bundle_adjustment.cc mirror](https://huggingface.co/camenduru/ceres-solver-v1/blob/main/colmap/src/optim/bundle_adjustment.cc).

