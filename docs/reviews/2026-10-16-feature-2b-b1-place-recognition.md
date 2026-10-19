# Review panel: `feature/2b-b1-place-recognition`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-16

| | |
|---|---|
| **Branch** | `feature/2b-b1-place-recognition` |
| **Range** | `03f3d21...e6d293a` (base: `origin/develop`; merge base `fd530cb`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | all checks passed (`scripts/check.sh` in the `slam-it` env, exit 0: 25 checks; pytest 3968 passed) |

### Findings and resolutions

Three reviewers found the native programs' unchecked vocabulary (H8, A1, L5), two the stale
docstrings (H2, A7), the compatibility path for older runs (H9, A9) and the default vocabulary's
shared, writable arrays and double read (H13, H14, A10); the missing native tests are H3, A2,
A14, L1 and L2 in part. Each is decided once, under its first ID.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `cpp/src/foundations/vocabulary.cpp` `read`, `check_children`, `parse_shape` | C++'s reader aborts or reads out of bounds on malformed arrays (parents used before they are checked, an int32 sum that overflows, a leaf's span, `stoull`), where Rust refuses them | fix (`check_parents` before `depths`, the sum in int64, no span for a leaf, `from_chars`; a native test for each) | fixed (152b394: the check in int64, a leaf's first child not read, the reader's shapes by `from_chars`, the tree checked before its depths; tested natively in 152b394's reader cases and c6ed83a's rung-0 tests) -- decided by evan-wes, 2026-10-16 |
| H2 | should-fix | hygiene | `edge/__init__.py`, `foundations/__init__.py`, `edge/tracking.py`, the three pipelines' comments, `edge-vo.md` | Docstrings and comments still say the edge rung has no bag of words | fix | fixed (806f8eb) -- decided by evan-wes, 2026-10-16 |
| H3 | should-fix | hygiene | `cpp/tests`, `rust/slam-it-*` tests | No native tests for the vocabulary, database, relocalisation or `search_by_bow`; the native pipeline tests turn relocalisation off | fix (with A2, L1 and L2) | fixed (c6ed83a; the readers' refusals in 152b394) -- decided by evan-wes, 2026-10-16 |
| H4 | should-fix | hygiene | `foundations/vocabulary.py` `read`, `record` | A malformed `vocabulary.json` raises `KeyError`, `AttributeError` or `UnicodeDecodeError`, not `InvalidInputError` | fix (one helper checks its keys and types) | fixed (152b394: the header and checksums are `.npy` arrays, `training.json` must be an object, every fault `InvalidInputError`) -- decided by evan-wes, 2026-10-16 |
| H5 | should-fix | hygiene | `vocabulary.py`; `pyproject.toml`; `matching.py`; the natives | `np.bitwise_count` needs NumPy 2.0, the floor is 1.26; Hamming distance written several ways | fix (`numpy>=2.0`; one Hamming function a language, in `orb`) | fixed (8c45675) -- decided by evan-wes, 2026-10-16 |
| H6 | should-fix | hygiene | `edge/database.py` `candidates`; its C++ and Rust twins | The database checks a word vector more weakly than the vocabulary does | fix (the vocabulary's check, public, used by the database) | fixed (8c45675) -- decided by evan-wes, 2026-10-16 |
| H7 | should-fix | hygiene | `edge/relocalization.py` `_try` and its twins; `tracking.py`; Rust `pnp_consensus!` | Long multi-phase functions; Python repeats the PnP block the natives factored; a macro that writes into its caller | fix (Python's split and shared, the macro a returned struct) | fixed (7412a7a: `PnpRun`/`run_pnp` shared in Python, `_try` split; Rust's macro is `PnpRun::record` and `kept`) -- decided by evan-wes, 2026-10-16 |
| H8 | should-fix | hygiene | the native run writers; `orb.hpp`, `orb.rs` | `vocabulary.json` pasted into `run.json` unchecked; the descriptor-version constants unused | as A1 | fixed (152b394), as A1 -- decided by evan-wes, 2026-10-16 |
| H9 | should-fix | hygiene | `eval/runs.py` `_read_info` | `setdefault("vocabulary", None)` keeps 0.3.9.0's runs readable: a compatibility path | fix (run format version 5, the key required) | fixed (152b394: run format version 5, `vocabulary` required; 2a3eb34: the status page's `relocalizations ?? 0`). -- decided by evan-wes, 2026-10-16. The viewer's `placed_by ?? []` is kept: bundles carried `placed_by` before B1 (the golden bundle at fd530cb has it), so it types an unchecked column rather than serving older bundles -- decided by evan-wes, 2026-10-16 (run 2's prompt) |
| H10 | should-fix | hygiene | `tests/orchestration/test_vo.py`; the native programs' tests | `--vocabulary`, the flag settings and the vocabulary refusals are untested on the command line | fix | fixed (2a3eb34) -- decided by evan-wes, 2026-10-16 |
| H11 | should-fix | hygiene | `scripts/measure_sharded.py` | One worker's failure loses every result | fix (each result written as it completes; a failed future recorded) | fixed (2a3eb34) -- decided by evan-wes, 2026-10-16 |
| H12 | should-fix | hygiene | `tests/foundations/test_vocabulary.py` | The python case of a backend test passes having asserted nothing | fix (the compiled backends only) | fixed (152b394: the test went with the bindings' `read`; the native readers are tested natively against the reader's fixture) -- decided by evan-wes, 2026-10-16 |
| H13 | note | hygiene | `vocabulary.py` `default` | The cached default hands out writable arrays | fix (read-only arrays) | fixed (152b394: a read vocabulary's arrays are read-only views) -- decided by evan-wes, 2026-10-16 |
| H14 | note | hygiene | `orchestration/vo.py` | `vo.run` reads and checksums the vocabulary twice | fix (one read) | fixed (152b394) -- decided by evan-wes, 2026-10-16 |
| H15 | note | hygiene | `vocabulary.py`, `train_vocabulary.py`, Rust `database.rs`, `relocalization.rs`, `tracking.Relocalization` | A `:func:` to a package; "10,000 frames" where 9,969 had descriptors; zip files left open; a conversion written twice; a confusing name | fix (the first four); defer the rename (B2, which rewrites tracking's local map) | fixed (152b394: the `:func:`, the 9,969 images, the archives closed; 7412a7a: Rust's node conversion written once, `database::words_of`); the rename deferred (B2, which rewrites tracking's local map) -- decided by evan-wes, 2026-10-16 |
| A1 | must-fix | architecture | the native readers; `vo.cpp`, `vo.rs`; the run writers; `CHANGELOG.md` | The native programs take a vocabulary Python refuses (another descriptor, a garbled checksum) and paste a malformed `vocabulary.json` into an unreadable `run.json`, exit 0 | fix (decision below) | fixed (152b394: each reader checks the header's format and descriptor versions; the checksums Python's, recorded as the ladder's one exception in 806f8eb; `run.json`'s record written from what was read; the CHANGELOG corrected in 806f8eb) -- decided by evan-wes, 2026-10-16 |
| A2 | must-fix | architecture | `tests/fixtures/foundations/`; `cpp/tests/foundations/`; `vocabulary.rs` | The vocabulary is the only compiled rung-0 module without a parity fixture or a native test | fix (`generate_vocabulary.py` from a brute force, exact; C++ and Rust held to it, the reader's refusals with it) | fixed (c6ed83a: `tests/fixtures/foundations/vocabulary.json`, from a brute force, exact; the reader's refusals in 152b394's `vocabulary_cases.json`) -- decided by evan-wes, 2026-10-16 |
| A3 | should-fix | architecture | `vocabulary.py`, `.cpp`, `.rs` `read`, `write`, `record` | Rung 0 now reads files, which the ladder keeps in eval | fix (the folder's I/O in `slam_it.eval` and the native eval libraries; a contract keeps `json` and `hashlib` out of the rungs) | fixed (152b394: `slam_it.eval.vocabularies` and its native twins; the contract "The algorithm rungs read no files") -- decided by evan-wes, 2026-10-16 |
| A4 | should-fix | architecture | `edge_crossing.hpp` `database_in`; both `vocabulary_crossing`s | The new crossings do not check their keys; C++ and Rust differ | fix | fixed (152b394) -- decided by evan-wes, 2026-10-16 |
| A5 | should-fix | architecture | the vocabulary's crossings | Its tree arrays cross as int32, where the ladder says int64 | doc (the ladder records the exception: they cross as stored) | fixed (806f8eb, doc) -- decided by evan-wes, 2026-10-16 |
| A6 | should-fix | architecture | `viewer/tests/golden.js` | The viewer's tests read the edge golden record and rebuild the bake's columns | fix (the found room baked into a golden bundle the tests load) | fixed (db27735) -- decided by evan-wes, 2026-10-16 |
| A7 | should-fix | architecture | the package docstrings; the pipelines' comments | As H2 | as H2 | fixed (806f8eb), as H2 -- decided by evan-wes, 2026-10-16 |
| A8 | should-fix | architecture | `docs/design/2026-10-02-slam-it-ladder.md` | The ladder does not record what B1 built and closed (the database's rung, A8's rest, the two-view item, A13, L15) | fix (doc) | fixed (806f8eb, doc) -- decided by evan-wes, 2026-10-16 |
| A9 | note | architecture | `eval/runs.py` | As H9 | as H9 | fixed, as H9 -- decided by evan-wes, 2026-10-16 |
| A10 | note | architecture | `vocabulary.default`; `vo.py` | As H13 and H14 | as H13, H14 | fixed (152b394), as H13 and H14 -- decided by evan-wes, 2026-10-16 |
| A11 | note | architecture | `pyproject.toml` edge contract | The database can sit beside BA rather than above the residuals | fix | fixed (b1c70de) -- decided by evan-wes, 2026-10-16 |
| A12 | note | architecture | `edge.database.best_covisible`; `edge.Options` | Scaffolding to retire in B2; 92 flat settings | defer (B2's design) | deferred (B2's design section: the covisibility module replaces `best_covisible`; the settings grouped) -- decided by evan-wes, 2026-10-16 |
| A13 | note | architecture | the design's section 3, item 9 | Rust's keep-or-drop decision is due after this PR | defer (after the merge, as planned) | deferred (after the merge: the online-SLAM design's section 3, item 9) -- decided by evan-wes, 2026-10-16 |
| A14 | note | architecture | `cpp/tests/edge/`, the Rust edge tests | No native tests for the database or relocalisation | as H3 | fixed (c6ed83a), as H3 -- decided by evan-wes, 2026-10-16 |
| L1 | must-fix | algorithm | `pipeline.cpp` `record_failure`; `pipeline.rs`; the golden rooms; `MOVED_OPTIONS` | The 3 s recently-lost window, and nine relocalisation settings off their defaults, are untested in C++ and Rust | fix (a golden room that relocalises with `reset_keyframes` 0 and a window shorter than its blanks; the settings moved in H21's check) | fixed (c3f20da: the window room, its settings and relocalisation's off their defaults in the record and in the stage parity; c6ed83a: the native pipelines relocalising) -- decided by evan-wes, 2026-10-16 |
| L2 | must-fix | algorithm | `database.cpp`, `database.rs` `erase`; the three pipelines' syncs | A culled keyframe leaving the database is untested in all three languages | fix (native `erase` tests; a pipeline test with a cull) | fixed (c6ed83a: native `erase` tests and a Python pipeline test culling; c3f20da: the window room culls 23 keyframes in RGB-D before relocalising) -- decided by evan-wes, 2026-10-16 |
| L3 | should-fix | algorithm | `edge/pipeline.py` docstring; 80c960a's message | The stated ORB-SLAM3 departure, that it restarts the time on a relocalised frame that fails the local map, is not one: both time from the first failure; the real departures are trying motion and the fallback first, and relocalising the first failed frame | fix (the docstring and the PR's description; 80c960a's message raised with the owner) | fixed (806f8eb: the docstring; the PR's description corrects 80c960a's message, kept as history) -- decided by evan-wes, 2026-10-16 |
| L4 | should-fix | algorithm | `vocabulary.py` `_majority`, `_seeds`, the weights | Training's DBoW2 rules have no known-answer tests, though the kept vocabularies' rebuilds rest on them | fix (a tied bit, a weight of log 3, a tiny training's checksums pinned) | fixed (c6ed83a) -- decided by evan-wes, 2026-10-16 |
| L5 | note | algorithm | `vo.cpp`, `vo.rs` | As A1 | as A1 | fixed (152b394), as A1 -- decided by evan-wes, 2026-10-16 |
| L6 | note | algorithm | `vocabulary.py` training | Seeding by D, as DBoW2's code, not D²; N counts images with descriptors | fix (N stated in the docstring) | fixed (152b394: the module docstring states N) -- decided by evan-wes, 2026-10-16 |
| L7 | note | algorithm | `edge/pipeline.py` | B4 needs a kept map's database; ORB-SLAM3 counts words across maps | defer (B4's design) | deferred (B4's design section: a kept map's database, and whether words are counted across maps) -- decided by evan-wes, 2026-10-16 |
| L8 | note | algorithm | -- | The state of the art: DBoW2 still what ORB-based CPU SLAM ships; learned retrieval for later rungs | no action | waived: an observation for the learned rungs (milestones 8 and 9), nothing to change in B1 -- decided by evan-wes, 2026-10-16 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `03f3d21...e6d293a` (B1, 14 commits, 130 files): Python `foundations/vocabulary.py`, `edge/database.py`, `edge/relocalization.py`, `edge/pipeline.py`, `orchestration/vo.py`, `eval/runs.py`; the C++ and Rust twins (vocabulary, database, relocalization, matching, pipeline, vo programs, bindings); the scripts (`train_vocabulary`, `measure_testset`, `measure_sharded`, `compare_testsets`); the viewer JS; the Python tests; the docs and CHANGELOG. I read the files in full and ran four scratch repros outside the repo. `git status` is clean.

**Summary:** The code is carefully built. Docs are good, the Python reference is clean, and the parity tests have an independent oracle. Rust is the most defensive of the three. There is one must-fix: C++ `vocabulary::read` crashes or has undefined behaviour on malformed arrays. Rust handles the same inputs and returns an error. The other findings are stale docs that now contradict the code, no native tests for about 3,000 lines of new C++ and Rust, a few Python error-contract gaps, and duplicated validation and logic.

### Findings

#### [must-fix] C++ `vocabulary::read` aborts or has undefined behaviour on malformed arrays; Rust returns `InvalidInput`
- **Where:** `cpp/src/foundations/vocabulary.cpp:366-369` (with `depths` at 180-186), `:241-248` (`check_children`), `:102` (`parse_shape`). The Rust guard is at `rust/slam-it-foundations/src/vocabulary.rs:403-408`.
- **Issue:** `read` is documented to return `InvalidInput` for a missing or malformed array (`vocabulary.hpp:73-74`). It breaks that in four ways:
  1. `read` calls `depths(out.parents)` before `check(out)` has validated `parents`. `depths` indexes `out[static_cast<size_t>(parents[node])]`, so a negative or forward parent reads out of bounds. Rust gates the same call with an `ordered` test.
  2. In `check_children`, `first + count` is `int32 + int32`. It overflows for `first` near INT_MAX, so the range check is bypassed (UB) and the next line, `subspan(first, count)`, runs out of range.
  3. For a leaf (`count == 0`) with a garbage `first`, `subspan(first, 0)` still requires `offset <= size`. Python and Rust accept such a file.
  4. `std::stoull` in `parse_shape` throws `std::out_of_range` for an all-digit shape that overflows. That is an uncaught exception, where Rust returns an error.
- **Why it matters:** The native programs read `--vocabulary` themselves. Python's checksum check only guards the `vo.run` path, and a hand-built folder with consistent checksums, or a corrupt file, takes down the run instead of giving a message.
- **Evidence:** I compiled the shipped `vocabulary.cpp` with the repo's flags (`-D_GLIBCXX_ASSERTIONS`) and ASan/UBSan, and ran it on copies of the committed default with one array altered each time:
  - `parents[5] = -7`: `Assertion '__n < this->size()' failed` in `depths`. Without assertions it is an unreported OOB read (the release build printed a normal error, by luck).
  - `children[1] = [2147483647, 1]`: UBSan reports `signed integer overflow: 1 + 2147483647`, then `subspan ... Assertion '__offset <= size()' failed`.
  - A leaf with `children = [2e9, 0]`: `subspan` assertion abort (Python's `Vocabulary(...)` and Rust's `check` accept it).
  - A `shape` of `(99999999999999999999999,)`: `terminate called after throwing 'std::out_of_range' what(): stoull`, exit 134.
- **Suggested fix:**
  - In `read`, call `check_parents` (already written) before `depths`, as Rust does.
  - Do the `first + count` arithmetic in int64.
  - Skip the span check when `count == 0`.
  - Catch the `stoull` overflow, or parse with `std::from_chars`.
  - Add a native test for each case (see the testing finding).

#### [should-fix] Docs and comments that B1 made false
- **Where:**
  - `src/slam_it/edge/__init__.py:5`: "without its covisibility graph, bag of words or second thread". The module list (`:8-16`) also omits `database` and `relocalization`.
  - `src/slam_it/edge/tracking.py:13`: "No bag of words: this recovers only near the local map; a lost track closes the map" (it now relocalises first). Also `:945-946`, the `relocalize_locally` docstring.
  - `docs/notes/edge-vo.md:27`: the bullet is headed "**no bag of words**: no relocalisation against the whole map", and its own parenthetical says a bag of words now relocalises. The reflowed line near `:243` is over 100 columns.
  - `src/slam_it/foundations/__init__.py:5` does not list the vocabulary.
  - The comments at `pipeline.py:657`, `cpp/src/edge/pipeline.cpp:367` and `rust/slam-it-edge/src/pipeline.rs:793` say "the lite pipeline has none [bag of words]".
- **Issue:** The package docstrings state each rung's contents, and these now describe the pipeline as it was before B1. The pipeline comments are narrowly true (no BoW reference-keyframe tracking) but read as if there is no BoW.
- **Suggested fix:** Update the edge and foundations package docstrings and module lists, reword the tracking docstrings and the `edge-vo.md` bullet, and make the three pipeline comments say what is still true.

#### [should-fix] No native tests for the new C++ and Rust cores; the native pipeline tests switch relocalisation off
- **Where:**
  - There is no `cpp/tests/foundations/test_vocabulary.cpp` and no `cpp/tests/edge/test_database.cpp` or `test_relocalization.cpp`.
  - There are no `#[cfg(test)]` modules in `rust/slam-it-foundations/src/vocabulary.rs`, `rust/slam-it-edge/src/database.rs` or `relocalization.rs`, and no `rust/slam-it-edge/tests/relocalization.rs`.
  - `cpp/tests/foundations/test_matching.cpp` and the Rust `matching.rs` tests run the shared fixture function by function (`grep search_by_bow cpp/tests` gives nothing), so the 671 lines of new `search_by_bow` cases are never run natively.
  - `cpp/tests/edge/test_pipeline.cpp:40` and `rust/slam-it-edge/tests/pipeline.rs:35` set `relocalize = false`.
- **Issue:** The convention (CLAUDE.md: "Tests mirror the packages", and the native fixture tests for every other rung-0 function) is not followed for about 3,000 new lines. They are exercised only through the Python-driven parity tests and the Python-run native programs. `ctest` and `cargo test` alone never run them, and the native `.npy` readers' error paths (malformed header, wrong dtype, truncated, Fortran order, bad tree) have no test anywhere. The must-fix crash above is what that gap hid.
- **Suggested fix:** Run the `search_by_bow` fixture cases in both native matching tests. Add native tests for the readers' error paths and for database insert/erase/candidates. Keep one native pipeline test with relocalisation on (the found room).

#### [should-fix] `vocabulary.read` and `record` raise `KeyError`, `AttributeError` and `UnicodeDecodeError`, not `InvalidInputError`
- **Where:** `src/slam_it/foundations/vocabulary.py:716-740` (`read`), `:670-677` (`record`).
- **Issue:** `read` indexes `info["k"]` and `info["levels"]` directly and calls `info.get("sha256", {}).get(name)`. `record` catches only `JSONDecodeError`, while `read` also catches `UnicodeDecodeError`. Both docstrings promise `InvalidInputError`. `--vocabulary` is a user-supplied folder.
- **Evidence:** A copy of the default with `"k"` deleted from `vocabulary.json` gives `KeyError 'k'`. With `"sha256": ["x"]` it gives `AttributeError: 'list' object has no attribute 'get'`. `record()` on a non-UTF-8 `vocabulary.json` gives `UnicodeDecodeError`.
- **Suggested fix:** Validate the key set and types once, in one helper shared by `read` and `record`, and raise `InvalidInputError` naming the file.

#### [should-fix] `np.bitwise_count` needs NumPy 2.0; `pyproject.toml` says `numpy>=1.26`; Hamming distance is written three ways in Python
- **Where:** `vocabulary.py:248, 321, 493`; `pyproject.toml:16`. `matching.py:92, 305-312` has a fourth, the `_POPCOUNT` table.
- **Issue:**
  - `np.bitwise_count` first appeared in NumPy 2.0 (from my recollection of its release notes, which I could not check against an installed NumPy 1.x). The declared floor is 1.26, so the vocabulary would fail at call time on a NumPy the project says it supports. The env locks 2.4.6, so no test sees it.
  - Inside `vocabulary.py` the same distance is `_hamming`, `_assign_distance` and an inline copy in `_quantize`.
  - C++ and Rust repeat it as `hamming` in vocabulary and `distance` in matching. The two are sibling modules in one layer and cannot import each other, so a shared function in `orb`, below both, is the clean home.
- **Suggested fix:** Raise the floor to `numpy>=2.0`, or use the table. Keep one Hamming function per language, in `orb`.

#### [should-fix] The same validation written two ways, the second weaker
- **Where:**
  - `database.py:329-332`: `candidates` checks only the length and ascent of the word ids, against `vocabulary._as_vector` (`:446-453`), which also requires integer ids and finite values.
  - `cpp/src/edge/database.cpp:106-112` repeats `check_vector`, and `rust/slam-it-edge/src/database.rs:230-235` does it inline. Both are weaker than the vocabulary module's (`vocabulary.cpp:203-212`, `vocabulary.rs:519-533`).
- **Why it matters:** Float ids are truncated silently. `np.ascontiguousarray(np.asarray([1.9, 2.5]), dtype=np.int64)` gives `[1 2]`, which `candidates` would accept. The docstring (`:327`) says it applies `score`'s checks, and it does not for the compiled backends.
- **Suggested fix:** Make the vocabulary check public and call it from the database in each language.

#### [should-fix] Long multi-phase functions; the Python reference re-inlines what the natives factored
- **Where:** `relocalization.py:385-460` (`_try`, 75 lines), `cpp/src/edge/relocalization.cpp:191-287` (about 95 lines), `rust/slam-it-edge/src/relocalization.rs:421-517` (about 95 lines).
- **Issue:** Each function does matching by word, the RANSAC budget, PnP, refinement and projection in one body, with early returns that rebuild the same `_Attempt`. C++ and Rust already split out `match_by_word` and `run_pnp`/`pnp_consensus`; Python inlines both, and `tracking.py:912-927` repeats the PnP try/except/record block that `relocalization.py:429-442` writes again.
- **Rust:** `pnp_consensus!` (`tracking.rs:184`) is a macro that assigns into the caller's summary fields. A small struct returned by `run_pnp` would do the same without hiding the mutation.
- **Suggested fix:** Split `_try` into match, RANSAC and refine steps, and factor the Python PnP-and-record block the way the natives did. The name `_try` could also say what it tries (`_try_candidate`).

#### [should-fix] Native programs write `vocabulary.json` into `run.json` unvalidated; the descriptor-version constants in C++ and Rust are dead
- **Where:** `cpp/src/eval/runs.cpp:486`, `rust/slam-it-eval/src/runs.rs:640-641`, set at `cpp/src/orchestration/vo.cpp:803-812` and `rust/slam-it-orchestration/src/vo.rs:1010-1016`. The constants are `orb.hpp:77` and `orb.rs:94`.
- **Issue:**
  - `Run.vocabulary` is a raw string spliced into `run.json` with no check that it is a JSON object. Python's `write_run` checks (`runs.py:530`), and the module docstring says `write_run` "checks everything before it writes". A malformed `vocabulary.json` would produce an unreadable `run.json`.
  - `kDescriptorVersion` and Rust's `DESCRIPTOR_VERSION` are defined and never used, and nothing holds the three equal. The native `read` checks neither the descriptor version nor the checksums. The design doc's "a run whose descriptor differs refuses it" is therefore true only through `vo.run`.
- **Suggested fix:** Parse the text into the existing JSON value type before writing, and either read `descriptor_version` natively and compare it, or delete the constants and state in the doc that the check is Python's.

#### [should-fix] A compatibility shim for old runs, against "No compatibility shims"
- **Where:** `src/slam_it/eval/runs.py:632` (`info.setdefault("vocabulary", None)  # a run from before relocalisation has none`); `runs.py:11`; CHANGELOG ("Runs and bundles written by 0.3.9.0 still read").
- **Issue:** CLAUDE.md says there are no users and to make clean breaks. This is new, and `03f3d21` has no `setdefault` in this file. Nothing in this PR needs an old run: the B1 comparison reruns the same code with relocalisation off.
- **Suggested fix:** The owner decides whether to bump the run `VERSION` and require the key, or to waive the rule for this one case.

#### [should-fix] The new command-line surface is untested
- **Where:** `tests/orchestration/test_vo.py`.
- **Issue:** Nothing covers:
  - `--vocabulary` in `vo.parse`;
  - `Request.arguments()` with a vocabulary or with a flag setting (`relocalize` as `true`/`false`);
  - `vo.check`'s two vocabulary refusals (`vo.py:230-233`);
  - `vo.run(vocabulary=...)` (the default fill-in, the refusal when relocalisation is off);
  - a native program given `--option relocalize=maybe` ("expected true or false"), or `--vocabulary` repeated, in C++ and Rust.
  - The parse-error test's docstring says the cases are "the native programs' own tests", but none of the new ones are there. `grep "true or false" tests cpp/tests` finds nothing.
- **Suggested fix:** Add the cases, parametrised over the three programs where they apply.

#### [should-fix] `measure_sharded.py` loses every finished result if one worker fails
- **Where:** `scripts/measure_sharded.py:144-151`; `_work` at `:86-93`.
- **Issue:** `results.jsonl` is written only after all futures finish. `measured` catches `SlamItError` and `FileNotFoundError` only. Any other exception in a worker (`OSError`, `MemoryError`, `BrokenProcessPool`) propagates out of `future.result()` after the pool has run for a long time, and no `results.jsonl` is written. The runs sit on disk, unlisted. `_work` also converts `Entry` to a tuple of strings and back, though a `NamedTuple` pickles.
- **Suggested fix:** Append each result to `results.jsonl` as it completes, or catch per future and record `{"name": ..., "failed": ...}`.

#### [should-fix] A test that passes without testing
- **Where:** `tests/foundations/test_vocabulary.py:254` (`test_every_backend_reads_the_default_as_python_does`: `if backend == "python": return`).
- **Issue:** The python case reports a pass having asserted nothing.
- **Suggested fix:** Parametrise over the compiled backends only.

#### [note] The shared default `Vocabulary` is mutable
`default()` is `functools.cache`d and hands the same writable arrays to every caller (`vocabulary.py:680-689`), and `fields()` shares them with the bindings. Setting `flags.writeable = False` in `__post_init__` would back up the "frozen" claim.

#### [note] `vo.run` reads and checksums the default vocabulary twice, uncached
`vo.py:491` reads it to validate and `vo.py:675` reads it again in `_run_here`, though `vocabulary.default()` exists. It costs tens of milliseconds, not an error.

#### [note] Smaller items
- `vocabulary.py:16` has `:func:`slam_it.edge`` pointing at a package.
- The `DEFAULT` docstring says "10,000 frames" but `vocabulary.json` records `images: 9969` (those that yielded descriptors).
- `train_vocabulary.py:62-73` leaves every `ZipFile` open (the tar handle is noqa'd with a reason).
- Rust's `usize::try_from(n).unwrap_or(usize::MAX)` is written in both `database.rs:97` and `relocalization.rs:554`.
- `tracking.Relocalization` (the local fallback's result) now sits beside the `relocalization` module, and the name is confusing in all three languages.

### Checked and fine
- **Docs and validation of the new modules:** the module and function docstrings of `vocabulary`, `database` and `relocalization` state shapes, dtypes, tie-breaks, summation order and ORB-SLAM departures. `Vocabulary.__post_init__` and Rust `check` validate the tree thoroughly. Option validation covers the new settings, the bool type, `reloc_confidence` below 1 and `reloc_narrow_distance`. Python and Rust `bow`/`score`/`quantize` reject bad words and malformed vectors.
- **Rust library code:** no `unwrap`, `expect` or `panic!` outside tests in the new files. The pnp macro is the only construct I'd change.
- **Suppressions and hygiene:** every `noqa` and `NOLINT` has a reason; no debug prints, TODOs, commented-out code or hard-coded paths in the library (CLI scripts print by design).
- **Tests that hold up:**
  - `search_by_bow` has an independent brute-force oracle (`generate_matching.py`) and invalid-input cases.
  - The `DESCRIPTOR_SHA256` pin test works.
  - Python tests for the database, relocalization (the RANSAC-budget table, the narrow search, a noise frame, the bad-candidate refusal) and the pipeline's loss rules assert on values.
  - The found room runs in all three backends.
  - `test_compare_testsets` and `test_measure_sharded` assert on outputs.
- **Pipeline control flow:** the C++, Rust and Python `_place`/`record_failure`/`sync_database` flow is direct.
- **Viewer JS:** typed JSDoc, `TRUTH_OPACITY` named, `truthRange` is correct (checked by hand), and `foundFrames`/`foundHint` are small and tested.
- **Docs elsewhere:** the CHANGELOG numbers agree with `docs/notes/test-set.md` (123 frames, 359 to 268 maps, 348 to 231 losses, 17,439 frames). No data-file or packaging problem, since `scikit-build` ships the package tree.
- **Repo state:** I changed nothing; all scratch files are in the scratchpad.

### Architecture review

## Architecture review

**Scope:** `git diff 03f3d21...e6d293a` (feature/2b-b1-place-recognition, 15 commits c0c9175..e6d293a, 130 files). Read in full: the Python vocabulary, matching-by-word, keyframe-database, relocalisation, pipeline and options modules, their C++ and Rust cores and bindings, `orchestration.vo` in all three languages, `eval.runs` and the native run writers, pyproject's contracts, the viewer and status-page changes, and the scripts. I checked them against the ladder doc and develop's `2026-10-14-online-slam.md`.

**Summary:** The structure is sound. Dependency direction holds. Each new concept has its own module (`foundations.vocabulary`, `edge.database`, `edge.relocalization`). Matching by word takes nodes as arrays, so it never imports the vocabulary. The edge rung is handed a `Vocabulary` and never reads files. Bindings stay thin, and the rung-0 and edge contracts were extended. There are two must-fixes:
- **A-1:** The native programs do not check a vocabulary the way Python does. They exit 0 on one Python refuses, and can write a `run.json` that `read_run` refuses. This breaks the ladder's rule that standalone C++ and Rust callers get the same protection.
- **A-2:** The vocabulary is the only rung-0 compiled module without a parity fixture or a native test.

The rest are doc and contract drift.

### Findings

#### [must-fix] A-1: Native programs accept a vocabulary Python refuses; a malformed `vocabulary.json` gives exit 0 and an unreadable run
- **Where:**
  - The native readers check the five arrays only: `cpp/src/foundations/vocabulary.cpp:315` and `rust/slam-it-foundations/src/vocabulary.rs:360`. Their own headers say Python does the rest: `cpp/include/slam_it/foundations/vocabulary.hpp:8-10`, `rust/slam-it-foundations/src/vocabulary.rs:6-8`.
  - The programs read `vocabulary.json` as plain text: `cpp/src/orchestration/vo.cpp:793-823`, `rust/slam-it-orchestration/src/vo.rs:1010-1027`. The run writers paste that text into `run.json` unchecked: `cpp/src/eval/runs.cpp:486`, `rust/slam-it-eval/src/runs.rs:640`.
  - The descriptor-version constants are defined and never read: `cpp/include/slam_it/foundations/orb.hpp:77` (`kDescriptorVersion`), `rust/slam-it-foundations/src/orb.rs:94` (`DESCRIPTOR_VERSION`).
  - The only check is in the Python launcher: `src/slam_it/orchestration/vo.py:491`.
  - `CHANGELOG.md:24` states the three languages check the checksums and the descriptor version.
- **Issue:** Python's `vocabulary.read` checks the format, its version, the descriptor version and each array's SHA-256. The C++ and Rust readers skip all four and trust the caller. So:
  - `slam_it_vo` run without Python accepts a vocabulary trained on another descriptor.
  - Given a `vocabulary.json` that is not JSON, it exits 0 and writes a `run.json` that is not JSON either.
  - Python's own program, with the same command line, refuses both.
- **Why it matters:** This breaks the ladder's Conventions rule (lines 331-333): "Standalone C++ and Rust callers get the same protection, and both languages fail the same way". It also breaks the stated purpose of the standalone builds ("for the edge budgets and a possible later phone target", lines 135-136). It contradicts B1's design §3.1: "a run whose descriptor differs refuses it". The failure surfaces later, at `read_run`, instead of when the program runs.
- **Evidence:** Two probes on HEAD's build. The first copied the default vocabulary, set `descriptor_version` to 99 and garbled one checksum:
  ```
  foreign python: refused: ...: trained on descriptor version 99, this ORB's is 1: its words mea…
  foreign cpp : read, k = 10 levels = 5
  foreign rust : read, k = 10 levels = 5
  notjson cpp : read, k = 10 levels = 5     (python: refused, not JSON)
  ```
  The second ran `slam_it_vo --vocabulary <copy whose vocabulary.json is cut short> --count 5` on the synthetic room. C++ and Rust both exit 0, then `runs.read_run(out)` fails with `run.json: not JSON (Expecting property name enclosed in double quotes: line 2 column 1 (char 2439))`.
- **Suggested fix:** Recommended: the code changes.
  - The native readers check `format`, `version` and `descriptor_version` against their own constants, which makes `kDescriptorVersion` and `DESCRIPTOR_VERSION` live.
  - The programs refuse a `vocabulary.json` they cannot vouch for, instead of embedding it.
  - Both need the ladder's PR 8 rule "nothing native reads JSON" amended for this one file's header fields. Another option is to carry those fields in a small `.npy` header array, which the native readers already parse.
  - Checking checksums natively needs a hand-written SHA-256. If that is too much, record "checksums are the launcher's" as an explicit exception in the ladder's Conventions.

  The alternative is the doc route: write the whole exception into the ladder, and correct `CHANGELOG.md:24` either way.

#### [must-fix] A-2: The vocabulary has no parity fixture and no native test
- **Where:**
  - `tests/fixtures/foundations/` has no `vocabulary.json` and no `generate_vocabulary.py`. Every other rung-0 module with a binding has a fixture (alignment through twoview; checked by listing `cpp/bindings/foundations/*.cpp` against the fixtures).
  - `cpp/tests/foundations/` has no `test_vocabulary.cpp`, and `rust/slam-it-foundations/src/vocabulary.rs` has no `#[cfg(test)]`.
  - Compiled-backend parity is live only: `tests/foundations/test_vocabulary.py:250,266,294`.
- **Issue:** CLAUDE.md says every C++ or Rust implementation has "a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The ladder's parity section says the same. The live-comparison exception is the edge rung's alone. In this PR, `matching.search_by_bow` got exactly that: a brute-force oracle fixture. The vocabulary did not.
- **Why it matters:** If quantise, bow or score changes the same way in all three languages, the live parity test still passes. Today only the edge golden record's found room would notice. Also, the C++ and Rust cores (including the new `.npy` reader and `check`) are never tested in a build without Python. Rung-0 modules elsewhere test natively against their fixture: `fixture_support.hpp` in C++, and in Rust a `read_to_string` of the fixture inside the module's tests.
- **Evidence:** `ls tests/fixtures/foundations/` at e6d293a. `git ls-tree cpp/tests/foundations/` has no `test_vocabulary.cpp`. `grep -c '#\[test\]'` on the Rust `vocabulary.rs` returns 0.
- **Suggested fix:**
  - Add `generate_vocabulary.py`: a small fixed tree, seeded descriptors, and expected words, nodes, vectors and scores from an independent brute force (Python ints' bit counts), with policy `exact`.
  - Hold Python, C++ (`cpp/tests/foundations/test_vocabulary.cpp`) and Rust (a `#[cfg(test)]` module) to it.
  - Include the reader's refusals: a bad `.npy` header, the wrong dtype, Fortran order.

#### [should-fix] A-3: Rung 0's cores now read files, which the ladder keeps in eval
- **Where:**
  - Python: `src/slam_it/foundations/vocabulary.py:615` (`write`), `:651` (`record`), `:692` (`read`); json, hashlib and pathlib in rung 0.
  - C++: `cpp/src/foundations/vocabulary.cpp` (an `.npy` reader, `<fstream>`, `<filesystem>`).
  - Rust: `rust/slam-it-foundations/src/vocabulary.rs` (`read_npy`, `std::fs::read`).
- **Issue:** Before this PR no rung-0 or edge module, in any language, read a file outside its tests (checked by grep at fd530cb). The ladder says "the algorithm rungs take arrays, never files" (lines 92-94). It puts the native readers, and the `.npy` writer, in `slam_it_eval` and `slam-it-eval`. Now `.npy` handling is split across rungs: the reader is in foundations, the writer in eval. The reader also follows eval's file-error convention (InvalidInput natively, FileNotFoundError in Python) without being in eval. Every non-test caller of `read`, `write` and `record` is in orchestration or scripts; the edge rung only takes a `Vocabulary`.
- **Why it matters:** The cores stop being array-only. File-format knowledge drifts into a second place, and the classic rung's later use would inherit it.
- **Suggested fix:** Recommended: the code moves.
  - Move the folder I/O to `slam_it.eval.vocabularies`, with `slam_it_eval` and `slam-it-eval` twins next to the run writer's `.npy` code.
  - Keep `Vocabulary`, `train`, `quantize`, `bow` and `score` in rung 0.
  - Add a forbidden contract (`json`, `hashlib` out of `foundations` and `edge`) so that "arrays, never files" is enforced.
  - This also gives A-1's native checks a natural home.

  The alternative is the doc route: B1's §3.1 placed the files in rung 0, so amend the ladder's line 93-94 with an explicit exception.

#### [should-fix] A-4: The new dict crossings do not hold their keys exactly, and C++ and Rust differ
- **Where:**
  - `cpp/bindings/edge/edge_crossing.hpp:506` (`database_in`, no `check_keys`), against `rust/slam-it-py/src/edge/edge_crossing.rs:774,799`, which refuses extra keys.
  - `vocabulary_in` checks no keys in either language: `cpp/bindings/foundations/vocabulary_crossing.hpp:52`, `rust/slam-it-py/src/foundations/vocabulary_crossing.rs:24,40`.
- **Issue:** The other edge crossings call `check_keys`: map, frame, options, camera and tracked at `edge_crossing.hpp:231,321,368,391,411`. The two new ones do not. For the database, C++ accepts a dict with extra keys and Rust refuses it.
- **Why it matters:** A field added on one side should fail loudly (ladder line 314). Here the same input passes in one language and fails in the other.
- **Suggested fix:** Add `check_keys` with `kDatabaseKeys` and `kVocabularyKeys` in C++, and the same exact-keys check in Rust's `vocabulary_in`.

#### [should-fix] A-5: The vocabulary's index arrays cross the binding boundary as int32; the ladder says int64
- **Where:** `cpp/bindings/foundations/vocabulary_crossing.hpp:29-32`, `cpp/bindings/foundations/vocabulary.cpp:58-60`, `rust/slam-it-py/src/foundations/vocabulary_crossing.rs:41-43`. These are the first int32 arrays in any binding (`git grep int32 fd530cb -- cpp/bindings rust/slam-it-py/src` finds only an image kernel's uint32).
- **Issue:** The ladder (lines 305-307) says structured data crosses with "`int64` index arrays". The tree's `parents`, `children` and `words` cross as int32, as their files store them.
- **Suggested fix:** Recommended: the doc. Record in the Conventions that a vocabulary's tree arrays cross as stored (int32), because of the file size. Or widen them in `vocabulary.fields`.

#### [should-fix] A-6: The viewer's tests read the edge rung's golden record and rebuild the bake's columns by hand
- **Where:** `viewer/tests/golden.js:20-42` (`foundRoomRecords` reads `tests/fixtures/edge/room_runs.json` and rebuilds the columns), used at `viewer/tests/solver.test.js:52` and `viewer/tests/timeline.test.js:104`.
- **Issue:** The viewer reads only bundles (ladder; CLAUDE.md). Its tests of a relocalised frame instead read the edge golden record's internal layout (`cases["found/tum/rgbd"]`, `summaries`, space-joined `states`) and imitate `orchestration.bundles` in JavaScript. The golden bundle has the `relocalization` key, but no relocalised frame.
- **Why it matters:** The timeline and solver display of a relocalised frame is never checked against a real bake. The viewer's tests are also tied to the edge record's format, which B2 regenerates whole (online-SLAM §4).
- **Suggested fix:** Bake the found room into the golden bundle (or into a second golden bundle) and have `foundRoomRecords` load that bundle through `loadBundle`.

#### [should-fix] A-7: Package docstrings and the three pipelines' comments still say there is no bag of words
- **Where:**
  - `src/slam_it/edge/__init__.py:5` ("without its covisibility graph, bag of words or second thread"); its module list (lines 8-16) lacks `database` and `relocalization`.
  - `src/slam_it/foundations/__init__.py:3-6` lacks place recognition (milestone 2B).
  - `src/slam_it/edge/pipeline.py:656-657`, `cpp/src/edge/pipeline.cpp:367`, `rust/slam-it-edge/src/pipeline.rs:793` ("the lite pipeline has none").
- **Issue:** The package docstring is where a rung declares its scope, milestone and budget (ladder lines 690-692; CLAUDE.md). Both docstrings now misstate the rung.
- **Suggested fix:**
  - Update both docstrings: rung 0's place recognition, and the edge rung's relocalisation and two new modules.
  - Reword the comment. Its point now is that the lite pipeline has a bag of words but does not track the reference keyframe by it (ORB-SLAM's `TrackReferenceKeyFrame`). That is a backlog item for the algorithm reviewer.

#### [should-fix] A-8: The ladder does not record what B1 built and closed
- **Where:**
  - `docs/design/2026-10-02-slam-it-ladder.md:549-552`: 2B places "the keyframe database, an inverted index" under "place recognition in rung 0". The code, as decided ("beside the map"), puts it in `edge.database`.
  - Open items B1 closed but still listed as open: the rest of A8 (lines 1015-1022: flag the loss on the step, keep the pipeline's state two-valued; done per online-SLAM §3.5) and "A monocular start in real time" (lines 1037-1040; done in c0c9175, measured).
  - A13 (lines 1031-1033) is partly met by `measure_sharded.py --seeds` and `--option`.
  - L15 (lines 994-999) still says "the lite pipeline ends the run". Relocalisation now exists, but those five-start runs were not measured again.
- **Suggested fix:** Recommended: the doc.
  - Say the keyframe database is the edge rung's. Add a sentence that the generic inverted index moves to rung 0 when the classic rung needs retrieval, rather than being copied.
  - Mark A8's rest and the two-view item "Closed by milestone 2B's B1".
  - Update A13 and L15 to their current state.

#### [note] A-9: The run format stays at version 4 with an optional new key
- `src/slam_it/eval/runs.py` `_read_info`: `info.setdefault("vocabulary", None)  # a run from before relocalisation has none`. The CHANGELOG says "Runs and bundles written by 0.3.9.0 still read". The viewer's `?? 0` and `placed_by ?? []` do the same for old bundles.
- This is a compatibility path, which the no-shims policy would normally refuse. It follows the design's "still version 4", and A2's runs are compared against. Worth deciding knowingly; bumping to version 5 is the clean-break alternative.

#### [note] A-10: `vocabulary.default()` is a process-wide cache that only tests use; the Python run reads the vocabulary twice
- `src/slam_it/foundations/vocabulary.py:680` (`@functools.cache`): its only caller is `tests/edge/support.py:64`. It hands out shared numpy arrays that callers can still write to (a frozen dataclass does not freeze them).
- `orchestration.vo` reads and hashes the same folder twice in-process (`vo.py:491` and `:675`); with a 6-level vocabulary, that is about 50 MB hashed twice.
- Either the orchestration uses one read, or `default()` moves to the tests.

#### [note] A-11: The edge contract can be tighter
- `pyproject.toml:197-199` stacks `(adjust)` > `(residuals)` > `(database)`, which lets the residuals and BA import the keyframe database.
- The database needs only `map` and `options`. Making it a sibling (`"(adjust) | (database)"` over `(residuals)`) would say so and enforce it, in import-linter and `check_layers.py` alike.

#### [note] A-12: Scaffolding to retire in B2, and the flat `Options`
- `edge.database.best_covisible` (`src/slam_it/edge/database.py:19,209`, with C++ and Rust twins) is correctly marked as standing in until B2. B2 should replace it with its covisibility module rather than grow it in `database`.
- `edge.Options` now has 92 flat fields. Before B2 to B4 add loop-closure and merging settings, consider nested groups (`Options.relocalization`, ...). The conventions already let nested dataclasses cross as nested dicts; `--option` would need dotted keys.

#### [note] A-13: Rust's keep-or-drop decision is due after this PR
- Online-SLAM §3.9 says Rust's cost is measured after B1 and the language kept or dropped. Nothing in the PR records it as pending; it belongs on the post-merge list.

#### [note] A-14 (for the hygiene reviewer): no native unit tests for `edge.database` or `edge.relocalization`
- `cpp/tests/edge/` has a test for every other edge module but none for these two, and the Rust files have no `#[test]`.
- The edge rung's live-comparison rule covers their parity. Standalone native behaviour, such as the database's duplicate and erase errors, is tested only through Python.

### Checked and fine
- Dependency direction: `foundations.vocabulary` sits beside `matching` and above `orb`. Matching by word takes node arrays and never imports the vocabulary. `edge.database` is on `map` and `options`; `relocalization` on `tracking`; the pipeline on top. No module reaches into another's private names (diff grep).
- The edge rung never reads a file. Orchestration resolves the default vocabulary and passes `Vocabulary` objects into `Pipeline` and `Pipeline::create`.
- `slam_it.backends` is unchanged and knows nothing of vocabularies.
- The language split:
  - Each binding is one file per module (`foundations/vocabulary`, `edge/database`, `edge/relocalization`), with shared crossing headers. The bindings only convert types; the algorithms live in the cores.
  - Every new module has an `Implementation` Protocol, held by `test_backends`.
  - The cores return `expected`/`Result` with InvalidInput, matching Python's `InvalidInputError`.
  - Rust stays inside its rung-0 and edge subset.
- Contracts: the new modules are placed in the exhaustive rung-0 and edge layer lists (import-linter and `check_layers.py`, layer 1 green). No new runtime dependency in any language: the `.npy` reader is hand-written, and Cargo and CMake are unchanged.
- Compute budget:
  - numpy only, no GPU stack.
  - A relocalised frame takes a median of 14.3 to 15.4 ms against deadlines of 32 to 33 ms, measured alone and pinned.
  - A keyframe's words are timed as mapping, as ORB-SLAM's mapping thread would make them.
  - The default vocabulary is about 6.1 MB, which fits pi-class.
- The two-view sample cap is an edge option passed into rung 0's generic `twoview.Options`; rung 0's default is unchanged.
- No reifyscene import and no import-time side effects (`DEFAULT` is a path constant). The default vocabulary's files ship in the wheel: scikit-build-core copies all package files and `.gitignore` does not exclude `.npy`.
- Scripts: `measure_sharded.py` and `compare_testsets.py` use only the public functions of `measure_testset.py` (an owner-decided neighbour import). The training tool is Python-only, as designed.
- Upload server untouched. The worker's summary gains `relocalizations`, which the status page reads through the API.

Files outside the repository, nothing in it touched:
- Probe scripts and outputs: `/tmp/scratch/arch/probe_native_vocab.py`, `/tmp/scratch/arch/probe_native_run.py`, and the runs in `/tmp/scratch/arch/`.
- A-1's reproduction, saved for reuse: `probe_native_vocabulary_checks.py`.

### Algorithm review

## Algorithm review

**Scope:** `feature/2b-b1-place-recognition`, `03f3d21...e6d293a` (15 commits, merge base fd530cb). Paths are relative to the repository root.

I read every changed algorithm file in full and checked it against its tests:
- the vocabulary: `src/slam_it/foundations/vocabulary.py` and its C++ and Rust twins;
- matching by word in `foundations/matching.py`;
- `edge/database.py` and `edge/relocalization.py`, each with its C++ and Rust twin;
- the loss flow in `edge/pipeline.py`, `pipeline.cpp` and `pipeline.rs`;
- the two-view cap (c0c9175), `scripts/train_vocabulary.py`, the golden record's rooms, and the B1 section of `docs/notes/test-set.md`.

Every ORB-SLAM3 and DBoW2 detail cited below was checked against the reference source: UZ-SLAMLab/ORB_SLAM3 master (`Tracking.cc`, `KeyFrameDatabase.cc`, `ORBmatcher.cc`, `MLPnPsolver.cpp`) and dorian3d/DBoW2 (`TemplatedVocabulary.h`). Run on HEAD's tree: `pytest tests/foundations/test_vocabulary.py tests/edge/test_database.py tests/edge/test_relocalization.py tests/foundations/test_matching.py` gave 354 passed. A read-only statistics script for the default vocabulary is at `/tmp/scratch/algo/vocab_stats.py`.

**Summary:** The slice follows its references closely.
- The vocabulary matches DBoW2's code.
- `DetectRelocalizationCandidates`, `SearchByBoW` and `Tracking::Relocalization` match ORB-SLAM3 constant for constant, including MLPnP's sample count.
- Every stated departure is real and documented, with one exception: a claim about ORB-SLAM3's loss timer is wrong.

The defects are in the tests, not the algorithm. Neither native pipeline is tested on the 3 s recently-lost rule, and nothing tests a culled keyframe leaving the database, in any language. Both paths read correctly today, and every B1 number was measured on C++.

### Findings

#### [must-fix] The recently-lost window, and most of B1's settings off their defaults, are untested in C++ and Rust
- **Where:**
  - `cpp/src/edge/pipeline.cpp:351-360` (`record_failure`; the window test is line 359) and `rust/slam-it-edge/src/pipeline.rs:729-737`.
  - The tests: `cpp/tests/edge/test_pipeline.cpp:40` and `rust/slam-it-edge/tests/pipeline.rs:35` set `relocalize = false`. `tests/orchestration/datasets.py:182` has `KEPT_OPTIONS = replace(OPTIONS, reset_keyframes=0, relocalize=False)`. `tests/orchestration/datasets.py:132-167` is `MOVED_OPTIONS`.
- **Issue:** The rule "a map above `reset_keyframes` is lost once `frame.timestamp - since > relocalize_seconds`" is ORB-SLAM3's RECENTLY_LOST window. Python tests it (`tests/edge/test_pipeline.py`, `test_a_large_map_is_lost_in_time_and_a_small_one_in_frames`). C++ and Rust do not, and no golden room reaches it:
  - the lost room's maps are small, so they are reset (removal code 3) under the `lost_frames` rule;
  - the kept room turns relocalisation off; its docstring says the window is "longer than the room";
  - the found room is never lost.

  H21's rule ("with the newer settings off their defaults, a native program makes Python's decisions", `tests/orchestration/test_vo.py:200-209`) was also not extended to B1's 15 new settings. `MOVED_OPTIONS` gained only `init_ransac_iterations`. The stage parity tests move `reloc_min_matches`, `covisible_points`, `reloc_common_words`, `reloc_group_score` and `reloc_neighbours`. These nine are never off their defaults in any native test: `reloc_inlier_ratio`, `reloc_min_inliers`, `reloc_confidence`, `reloc_ransac_iterations`, `reloc_window`, `reloc_narrow_inliers`, `reloc_narrow_window`, `reloc_narrow_distance`, `relocalize_seconds`. Only `relocalize` goes through a program's `--option`.
- **Why it matters:** This rule decides when a kept map is closed. Every figure in the B1 note is from the C++ backend (`docs/notes/test-set.md`, B1 "Conditions"). Any of these native regressions would pass layer 1:
  - `>=` written for `>`;
  - `lost_since_` not cleared on a tracked frame;
  - the time measured from the last failure instead of the first;
  - a hard-coded `relocalize_seconds`.
- **Evidence:** In `tests/fixtures/edge/room_runs.json`, every case's `keyframe_lives` holds only removal code 3 (reset), or no removal at all. The `found/*` cases are `tracking` on every frame, all in map 0.
- **Suggested fix:** Add a golden room with relocalisation on and `reset_keyframes=0`, and move `relocalize_seconds` below the blank stretch, e.g. 0.05 s, which loses the map on the third blank frame at 30 Hz. That separates the window from `lost_frames`, which would lose it on the fifth. Run it with the other reloc settings moved off their defaults too, under H21's two-sided check.

#### [must-fix] A culled keyframe leaving the keyframe database is untested in all three languages
- **Where:**
  - `cpp/src/edge/database.cpp:139` (`erase`, declared public at `cpp/include/slam_it/edge/database.hpp:75`) and `rust/slam-it-edge/src/database.rs:112` (`pub fn erase`).
  - The syncs: `src/slam_it/edge/pipeline.py:618-629`, `cpp/src/edge/pipeline.cpp:321-349`, `rust/slam-it-edge/src/pipeline.rs:749-768`.
- **Issue:** The bindings build the compiled index with `insert` only (`cpp/bindings/edge/edge_crossing.hpp:531`, `rust/slam-it-py/src/edge/edge_crossing.rs:831`). So the compiled `erase` is reached only by a native pipeline's sync after mapping culls a keyframe. No room culls one: removal code 0 never appears in any of the golden record's 20 cases. No C++ or Rust test calls `erase` (`git grep "erase(" cpp/tests rust/slam-it-edge/tests` finds only unrelated map erases). Python's `Database.erase` has a unit test (`tests/edge/test_database.py:43`). No test checks that a pipeline's database holds exactly its map's keyframes after culling.
- **Why it matters:** A culled keyframe left in the index is returned as a candidate.
  - Python's `_try` and the native `row_of` then raise InvalidInput, which aborts the whole run.
  - C++'s `db.keyframes.at` throws if an index entry outlives its keyframe.

  The 77-run test set (C++) ran without such an abort, so the path works today; what is missing is a guard against regression. Both are untested public items on algorithm code.
- **Evidence:** The grep and golden-record inspection above. `tests/edge/conftest.py:138` builds `room_database` directly, never through a pipeline.
- **Suggested fix:**
  - C++ and Rust unit tests: add three keyframes, erase one, then check that no word lists it and that `candidates` never returns it; erasing it again is refused.
  - A pipeline test in all three languages, or a golden room with a setting that makes `KeyFrameCulling` fire, asserting that the database equals the map's keyframes after the cull and that a later relocalisation runs.

#### [should-fix] The stated ORB-SLAM3 departure on the loss timer is not one
- **Where:** `src/slam_it/edge/pipeline.py:28-30`, which says ORB-SLAM3 "restarts its time when a relocalised frame fails the local map". Commit 80c960a's message says the same (body lines 12-14), and so does the owner's decision as worded in the panel brief.
- **Issue:** ORB-SLAM3's code does not do this.
  - `Tracking::Relocalization` never assigns `mState`. It sets the pose, the matches and `mnLastRelocFrameId`.
  - After `TrackLocalMap`, `Track` runs `if(bOK) mState = OK; else if (mState == OK) { ... mState=RECENTLY_LOST; ... mTimeStampLost = mCurrentFrame.mTimeStamp; }`.
  - A recently lost frame that relocalises and then fails the local map has `bOK == false` while `mState` is still `RECENTLY_LOST`, so `mTimeStampLost` keeps the first failure's time.

  Both systems therefore measure the 3 s from the first failed frame. The real departures are that every failed frame here tries the motion model and the local fallback first, and that the very first failed frame relocalises too. In ORB-SLAM3 the frame that turns OK into RECENTLY_LOST does not call `Relocalization`; relocalising starts on the next frame. The docstring's "every failed frame tries all three" covers this only implicitly.
- **Why it matters:** The ladder records why it departs from its reference. Here it names a departure that does not exist, and the owner's decision was framed on it.
- **Evidence:** ORB_SLAM3 master, `src/Tracking.cc`: the RECENTLY_LOST branch (`bOK = Relocalization(); if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK) mState = LOST;`), the end-of-`Track` block quoted above, and the end of `Relocalization` (`mnLastRelocFrameId = mCurrentFrame.mnId; ... return true;`, no `mState`).
- **Suggested fix:** Correct the docstring and the PR description. Commit 80c960a's message is on an unmerged branch, so per the owner's global rule, raise the contradiction with them rather than leave it standing. The code needs no change.

#### [should-fix] Training's DBoW2 rules have no known-answer tests, yet "rebuilt exactly" rests on them
- **Where:** `src/slam_it/foundations/vocabulary.py:456-461` (`_majority`, a tie sets the bit), `:472-486` (`_seeds`), `:599-601` (IDF); `tests/foundations/test_vocabulary.py`; `src/slam_it/foundations/vocabularies/local.json`.
- **Issue:** The tests leave three gaps:
  - nothing tests the tie rule;
  - the IDF `log(N/N_i)` is checked only where it is 0 (`test_a_word_in_every_image_weighs_nothing`);
  - determinism is checked only by training twice in one process.

  The two 6-level vocabularies are kept outside the repository with their SHA-256 and a rebuild command ("rebuilt exactly"). Those checksums depend on these rules and on numpy's `Generator` streams. NEP 19 gives `Generator` no stream guarantee across versions: "ignoring the precise stream values"; reproduce "by using the exact same version of numpy", pinned here at 2.4.6 in `scripts/locks/slam-it.lock`. A change to `train`, or a numpy upgrade, would surface only when someone rebuilds from about 15 GB of images.
- **Suggested fix:** Add three known-answer tests:
  - `_majority` of an even count with a tied bit;
  - a word in 1 of 3 images weighs `log 3`;
  - a tiny training run (the `_clusters` set) whose five arrays' SHA-256 are pinned in the test.

#### [note] Native programs take a vocabulary without its checksum and descriptor-version checks (architecture's lane)
- **Where:** `cpp/src/orchestration/vo.cpp:796`, `rust/slam-it-orchestration/src/vo.rs:1012`.
- The cores say this on purpose: Python checks before a native run is given the folder. But `slam_it_vo --vocabulary X` run on its own will score with words from another descriptor. The design (section 3, item 1) says such a run refuses the vocabulary. Checking `descriptor_version` in `vocabulary.json` natively would be cheap. For the architecture reviewer.

#### [note] Two DBoW2 code details the training follows, for the record
- Seeding is weighted by D(x), as DBoW2's `initiateClustersKMpp` does, not by D(x)² as standard k-means++ does (Arthur and Vassilvitskii, "k-means++: The Advantages of Careful Seeding", SODA 2007). DBoW2's own comment says D², but its code uses `F::distance` unsquared. The docstring states the D weighting correctly.
- `N` counts only images with at least one descriptor: 9,969 of the 10,000 drawn for the default. DBoW2's `NDocs = training_features.size()` counts every image. Each weight differs by log(10000/9969) ≈ 0.003, against weights of 2.1 to 9.2. Negligible, but unstated.

#### [note] B4 will need kept maps' databases; ORB-SLAM3 counts words across maps
- `src/slam_it/edge/pipeline.py:557` drops a map's database when the map is closed.
- ORB-SLAM3's `DetectRelocalizationCandidates(F, pMap)` counts shared words over every map's keyframes and filters by map only when collecting candidates. Other maps therefore shift its `maxCommonWords` and `bestAccScore` thresholds. B1's database per map is cleaner.
- B4's recognition across maps needs a kept map's database, kept or rebuilt from its descriptors. Worth deciding in B4's design section.

#### [note] State of the art: DBoW2 is still what ORB-based CPU SLAM ships; learned retrieval has moved on
For the edge rung's goal and budget the choice is right. The newer methods belong to milestones 8 and 9, not to this rung:
- ORB-SLAM3 (Campos et al., T-RO 2021) still uses DBoW2.
- Learned global descriptors with DINOv2 backbones lead place recognition but need a GPU: SALAD (Izquierdo and Civera, CVPR 2024), BoQ (Ali-bey et al., CVPR 2024), MegaLoc (Berton and Masone, 2025, [arXiv:2502.17237](https://arxiv.org/abs/2502.17237)).
- MASt3R-SLAM (Murai, Dexheimer and Davison, CVPR 2025, [arXiv:2412.12392](https://arxiv.org/abs/2412.12392)) retrieves with an incremental ASMK over MASt3R's encoder features, and uses it for loop closure and relocalisation.
- iBoW-LCD (Garcia-Fidalgo and Ortiz, RA-L 3(4) 2018, [arXiv:1802.05909](https://arxiv.org/abs/1802.05909)) builds a binary vocabulary online, which sidesteps the training-set question B1 set out to measure.

### Checked and fine
- **Quantising:** each level goes to the nearest child with strict `<`, so the lowest index wins ties, as DBoW2's `transform`. A leaf reached above level 2 is its own direct-index node. Level 2 of 5 is the same depth as ORB-SLAM's `levelsup = 4` of 6.
- **Word vectors and scores:** each word's count times its IDF, L1-normalised. The score is `1 - |v-w|_1/2`, computed over shared words as Σ(|v|+|w|-|v-w|)/2, DBoW2's `L1Scoring` identity (Nistér and Stewénius 2006). Terms are summed in a fixed order, so the three languages agree exactly.
- **Centres and k-means:** the bitwise majority with a tie setting the bit equals `FORB::meanValue`'s `sum >= ceil(n/2)`, MSB first. k-means is DBoW2's trivial case for ≤k descriptors and stops when no assignment changes. The final labels are the argmin against the final centres, so every training descriptor descends to its own leaf.
- **The default vocabulary, measured:** 99,961 words; 99,959 leaves at depth 5 and 2 at depth 4; 100 nodes at level 2; no zero-weight word. So DBoW2's dropping of stopped words from the feature vector never comes into play.
- **Candidates (`DetectRelocalizationCandidates`):** int(0.8·max) as `floor`; strictly more words; 10 best covisible neighbours; strictly above 0.75 of the best group; the group's best-scored keyframe stands for it; each candidate kept once. All match ORB-SLAM3. The stated departures (unscored neighbours add nothing; best first; ties by id) are documented.
- **`SearchByBoW`:** only the keyframe's features with a map point are matched; TH_LOW 50; a strict 0.75 ratio; a node with one descriptor passes, as ORB-SLAM's `bestDist2 = 256` does; the rotation check runs. The fixture's oracle is an independent brute force under an exact policy.
- **`Relocalization`:** fewer than 15 matches discards a candidate. MLPnP's `SetRansacParameters` is reproduced exactly, including the truncation, the epsilon bump and the `e^3` count: 35 samples at the defaults, 14 at 15 matches. Then: under 10 inliers the candidate is dropped; a 10 px search with distance 100; with 30 < inliers < 50 a 3 px search with distance 64; 50 inliers place the frame. `sFound` is RANSAC's inliers; outliers are cleared before the first search; the second optimisation's outliers are kept until the last. The searches have no ratio test (ratio 1 cannot reject under `best > ratio*second`) and check rotation against the keyframe's keypoint angle. The stated departures (P3P with LO-MSAC run to the end, best first; 4 px at every level; skipping points behind the camera) were confirmed against ORB-SLAM3's source.
- **After relocalisation:** the next `max_frames` frames need 50 inliers, as `TrackLocalMap` with `mnLastRelocFrameId + mMaxFrames`, and no keyframe is inserted while the map holds more than `max_frames`, as `NeedNewKeyFrame`. The window's `> 3.0` is strict, as ORB-SLAM3's.
- **The two-view cap (200 samples a model):** E is a 5-point sample and H a 4-point one. At 50% inliers, 200 samples find an all-inlier E sample with probability ≈ 0.998, against ≈ 0.54 for ORB-SLAM's 8-point sets. Adaptive stopping is kept, and the backend test pins the limit.
- **Failure is observable:** each candidate's summary records how far it got; RANSAC finding no pose is flagged as degenerate; Python refuses a vocabulary of another version or descriptor, or with a changed checksum; the descriptor constants are pinned by SHA-256.
- **Parity and determinism:** RANSAC is seeded; ties are broken by id throughout; words, nodes, vectors, scores and candidates agree exactly across languages; poses agree at the solver tolerance. The golden record's found room stays in map 0 in all four modes.
- **The measured results:** the "found again in the right place" check reuses the ATE's per-map alignment. One false placement (fr2_360_hemisphere monocular, 198.6 cm) is reported openly.

Sources:
- [ORB_SLAM3 Tracking.cc](https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/src/Tracking.cc), [KeyFrameDatabase.cc](https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/src/KeyFrameDatabase.cc), [ORBmatcher.cc](https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/src/ORBmatcher.cc), [MLPnPsolver.cpp](https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/src/MLPnPsolver.cpp)
- [DBoW2 TemplatedVocabulary.h](https://raw.githubusercontent.com/dorian3d/DBoW2/master/include/DBoW2/TemplatedVocabulary.h)
- [NEP 19, the random number generator policy](https://numpy.org/neps/nep-0019-rng-policy.html)
- [MegaLoc, arXiv:2502.17237](https://arxiv.org/pdf/2502.17237)
- [MASt3R-SLAM, arXiv:2412.12392](https://arxiv.org/pdf/2412.12392)
- [iBoW-LCD, arXiv:1802.05909](https://arxiv.org/pdf/1802.05909)

## Run 2 -- 2026-10-16

| | |
|---|---|
| **Branch** | `feature/2b-b1-place-recognition` |
| **Range** | `03f3d21...98fed93` (base: `origin/develop`); the fixes since Run 1: `e6d293a..98fed93` |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered `slam-it-*-reviewer` agents |
| **Layer 1** | not run on this head, by evan-wes's instruction of 2026-10-16: it runs once, when the PR is ready to merge; each fix commit ran the checks its change touches |

### Findings and resolutions

Every Run 1 finding was found resolved as recorded, except L1, which the algorithm reviewer
(L9) and the hygiene reviewer (H16) found half met, and H10, which H17 finds met in part. IDs
continue Run 1's; H21 here is this run's, not PR 8's finding of that name.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H16 | should-fix | hygiene | `tests/edge/test_relocalization.py` `MOVED`, `MOVED_CASES` | Six of the eight moved relocalisation settings change no decision; the test checks only the bundle | as L9 | fixed (43d7650), as L9 -- decided by evan-wes, 2026-10-16 |
| H17 | should-fix | hygiene | `tests/orchestration/test_vo.py`; the native programs' `run` | No test runs a native program without a vocabulary, with one and relocalisation off, or with one its reader refuses (A1's symptom) | fix (a test over every backend: exit 3, the folder named, no `run.json`) | fixed (6ead1d5) -- decided by evan-wes, 2026-10-16 |
| H18 | should-fix | hygiene | the vocabulary and database crossings | A4's exact-key checks have no test | fix (missing and extra keys, both compiled backends; Rust's check once, at the top of `vocabulary_in`) | fixed (6ead1d5) -- decided by evan-wes, 2026-10-16 |
| H19 | should-fix | hygiene | `foundations/orb.py` `distance` | Public and unvalidated: an int8 array gives 32 where every bit differs | fix (uint8 and DESCRIPTOR_BYTES checked, `InvalidInputError`, a test) | fixed (618e5aa) -- decided by evan-wes, 2026-10-16 |
| H20 | should-fix | hygiene | `cpp/tests/edge/test_relocalization.cpp`; Rust's edge tests | `one_word` written twice in C++; an empty frame built three times in Rust | fix (the scene's `one_word`; an `empty_frame` in Rust's scene) | fixed (43d7650) -- decided by evan-wes, 2026-10-16 |
| H21 | should-fix | hygiene | `docs/design/2026-10-14-online-slam.md` section 3 | The plan's B1 section describes the old vocabulary layout | as A17 | as A17 -- decided by evan-wes, 2026-10-16 |
| H22 | note | hygiene | `../vocabularies/{arkitscenes-6,bovisa-6}`; `local.json`; CHANGELOG | The two kept 6-level folders are format version 1 and no longer read | fix (convert them outside the repo, the arrays untouched; say so in `local.json`) | fixed (both folders converted outside the repository by `convert_vocabulary_v1_to_v2.py`, which only adds the three new files after checking each array's SHA-256, and read back; c20f759's `local.json` says they are format version 2) -- decided by evan-wes, 2026-10-16 |
| H23 | note | hygiene | `cpp/src/eval/vocabularies.cpp`, `rust/slam-it-eval/src/vocabularies.rs` | The native `.npy` header parsers accept garbage in the padding, and echo a corrupt `descr` raw | fix (a Python-only case for the padding; `descr` escaped) | fixed (618e5aa: the 20th reader case, refused by Python alone; `descr` quoted) -- decided by evan-wes, 2026-10-16 |
| H24 | note | hygiene | `edge/tracking.py` `PnpRun`; Rust's two ways of applying a `PnpRecord` | Co-null fields re-checked; the record applied two ways | fix (one way in Rust; the Python check narrowed once) | fixed (618e5aa: `PnpRun.kept` in Python; Rust's `RecordsPnp` and `pnp_kept`) -- decided by evan-wes, 2026-10-16 |
| H25 | note | hygiene | `eval/vocabularies.py`, `generate_vocabulary.py`, Rust `read_npy` | Readability: positional header indices, a shapes dict rebuilt per call, a discarded read-back, an 85-line function | fix | fixed (618e5aa) -- decided by evan-wes, 2026-10-16 |
| H26 | note | hygiene | the ladder; `edge-vo.md:292`; `vocabulary.py`, `matching.py` imports | A 132-column line, a pre-B1 statement in the present tense, two import styles of `orb` | fix | fixed (618e5aa: the imports; 4b14a90: the ladder's line, `edge-vo.md`) -- decided by evan-wes, 2026-10-16 |
| A15 | should-fix | architecture | `eval/vocabularies.py:62`; `foundations/vocabularies/` | The committed vocabulary's files are still rung 0's package data, eval reaching into `foundations/` by path | fix (`slam_it.eval.vocabularies` a package, `default/` and `local.json` in it) | fixed (c20f759) -- decided by evan-wes, 2026-10-16 |
| A16 | should-fix | architecture | `scripts/check_layers.py`; `pyproject.toml` | "The algorithm rungs read no files" holds Python only; nothing keeps the native cores from reading files again | fix (`<fstream>`, `<filesystem>`, `<cstdio>` and `std::fs` refused in the rungs' native sources, outside tests) | fixed (50dd7d1; the ladder in 4b14a90) -- decided by evan-wes, 2026-10-16 |
| A17 | should-fix | architecture | develop's `docs/design/2026-10-14-online-slam.md` | Section 3's vocabulary files, checksums, descriptor record, run format version and `--vocabulary` default contradict the code | fix (a docs commit on develop, its wording for evan-wes's approval) | fixed on develop (c41a541), its wording approved by evan-wes -- decided by evan-wes, 2026-10-16 |
| A18 | note | architecture | the ladder's package table | `vocabularies` and the native vocabulary reader missing from eval's row | fix | fixed (4b14a90) -- decided by evan-wes, 2026-10-16 |
| A19 | note | architecture | the ladder's exception; `eval/vocabularies.py`'s docstring | A standalone native run records the checksums its folder claims, unverified | fix (one clause: a run's record is verified only when `vo.run` launched it) | fixed (4b14a90) -- decided by evan-wes, 2026-10-16 |
| A20 | note | architecture | `pyproject.toml`'s no-files contract | It could forbid `pathlib`, `io`, `os`, `shutil`, `tempfile`, `zipfile`, `tarfile`, `pickle` too; it cannot see `open()`; its source list must grow with the rungs | fix (the modules added; the limits stated) | fixed (50dd7d1) -- decided by evan-wes, 2026-10-16 |
| A21 | note | architecture | `cpp/src/eval/runs.cpp`, `vocabularies.cpp` and Rust's twins | `.npy` written in one native eval module and read in another | defer (B4, when a second native reader arrives: one `npy` module) | deferred (the ladder's open items, with B4; 4b14a90) -- decided by evan-wes, 2026-10-16 |
| L9 | must-fix | algorithm | `tests/edge/test_relocalization.py` `MOVED`; `WINDOW_OPTIONS`; the narrow search | L1 half met: six of the eight moved settings, and the whole narrow search (its last optimisation, the outliers held), decide nothing in any test; three docstrings and c3f20da's message say otherwise | fix (cases where each setting decides, per-setting reversions asserted, `ransac_budget` tested natively off its defaults, the docstrings corrected; c3f20da's claim corrected in the PR's description) | fixed (43d7650: `DECISIVE`, a test per setting, the narrow search placing, the budget natively off its defaults; c3f20da's claim corrected in the PR's description) -- decided by evan-wes, 2026-10-16 |
| L10 | should-fix | algorithm | `vocabularies/local.json`; CHANGELOG; `vocabulary.py` `train` | "Rebuilt exactly" can fail on an AVX-512 CPU: numpy's own float64 log there | fix (a rebuild held to the four integer arrays' SHA-256, the weights to their rule within a few ULP, said in `local.json` and the CHANGELOG) | fixed (c20f759) -- decided by evan-wes, 2026-10-16 |
| L11 | note | algorithm | `tests/orchestration/datasets.py` `WINDOW_BLANKS` | Misstates why frame 33 is tried; undersells that the room checks relocalising before the window | fix | fixed (43d7650) -- decided by evan-wes, 2026-10-16 |
| L12 | note | algorithm | `tests/fixtures/foundations/generate_vocabulary.py` | Ties covered by chance, not design | fix (the generator asserts a tie at a visited node on every level) | fixed (618e5aa) -- decided by evan-wes, 2026-10-16 |
| L13 | note | algorithm | the PR | L3's description half, and c3f20da's claim, need the PR's body | fix (in the PR's body) | fixed (the PR's body) -- decided by evan-wes, 2026-10-16 |
| L14 | note | algorithm | `tests/orchestration/datasets.py` `MOVED_OPTIONS` (H21 of PR 8) | 11 of its 33 settings decide nothing on the gap room | defer (the ladder's open items: H21's check made per setting) | deferred (the ladder's open items; 4b14a90) -- decided by evan-wes, 2026-10-16 |

### Hygiene review

## Hygiene review

**Scope:** Run 2, `e6d293a..98fed93` (nine commits, 121 files), run 1's H1-H15 checked one by one, and the new code read in full: `slam_it.eval.vocabularies` and its C++/Rust twins (`cpp/src/eval/vocabularies.cpp`, `rust/slam-it-eval/src/vocabularies.rs`), the reader fixture and its 19 cases, the vocabulary parity fixture, the native tests (vocabulary, database, relocalisation, pipeline), the window room, the `PnpRun` and `words_of` refactors, `orb.distance`, `measure_sharded`, and the docs and CHANGELOG edits. Layer 1 was not run on this head.

Checks I ran myself:
- 320 Python tests (`test_vocabularies`, `test_runs`, `test_vocabulary`, `test_orb`) pass.
- `cargo test -p slam-it-eval vocabularies` passes (3 tests).
- A fuzz of 400 mutated copies of the reader fixture through the C++ `slam_it_vo --vocabulary`, with no crash. Details are under H1.
- A one-setting-at-a-time ablation of the window room's moved relocalisation settings. Details are in the first finding.

Importing the C++ backend made the editable install rebuild `build/slam-it`, which is untracked. `git status` is clean.

**Summary:** All fifteen of run 1's hygiene findings are resolved as the report says, with two qualifications. H3/L1's "settings off their defaults" tests do not exercise six of the eight settings they claim to, and H10's command-line refusals are covered only in part (the native programs' vocabulary refusals are untested). The new reader code is well documented, validates before it allocates, and held up under fuzzing. There is no must-fix. The six should-fix items are about test strength, one duplicated test helper, an unvalidated public kernel, and a stale design doc.

### Findings

#### [should-fix] The moved relocalisation settings are mostly not exercised, so a native that hard-codes one would still pass (L1 resolved only in part)
- **Where:** `tests/edge/test_relocalization.py:162-199` (`MOVED`, `MOVED_CASES`, `test_the_moved_settings_change_what_relocalisation_decides`); `tests/orchestration/datasets.py` (`WINDOW_OPTIONS`); c3f20da's message ("so the agreement shows the natives read them").
- **Issue:** The test checks that all eight moved settings together change Python's result against all-default. The `MOVED_CASES` docstring says "every setting it moves is used". That is false for six of the eight. Parity with Python proves a native reads a setting only if that setting alone changes a decision on a compared case.
- **Why it matters:** L1 was a must-fix because a native could hard-code `reloc_*` and nothing would fail. For six settings, that is still true.
- **Evidence:** I reset one moved setting at a time to its default and compared the whole `relocalize` result (summary, pose, tracked points and `matched`). Script: `/tmp/scratch/panel2/ablate2.py`.

  | Setting reset to default | Frame 30 (wide search) | Frame 6 (narrow search) |
  |---|---|---|
  | `reloc_window` | changes | changes |
  | `reloc_confidence` | no change | changes |
  | `reloc_inlier_ratio` | no change | no change |
  | `reloc_min_inliers` | no change | no change |
  | `reloc_ransac_iterations` | no change | no change |
  | `reloc_narrow_inliers` | no change | no change |
  | `reloc_narrow_window` | no change | no change |
  | `reloc_narrow_distance` | no change | no change |

  So only `reloc_window` and `reloc_confidence` are shown to be read; `recovery_tracked` is one of the case's own parameters.
- **Suggested fix:**
  - Make the test per-setting: for each moved setting, assert that resetting it alone changes Python's result on at least one case. It will then fail until a case shows each setting.
  - For the three budget settings (`reloc_inlier_ratio`, `reloc_confidence`, `reloc_ransac_iterations`), add native assertions on `ransac_budget` with moved values. It is a pure function, and `check_budget` only tests it at defaults.
  - Pick inputs for the narrow-search settings (a narrow window or distance that rejects matches) and for `reloc_min_inliers` (a value above what the frame reaches).
  - Correct the docstring and the CHANGELOG's "relocalisation's settings off their defaults" until the claim holds.

#### [should-fix] The native programs' vocabulary refusals are untested; H10 and A1 are covered at the library level only
- **Where:** `tests/orchestration/test_vo.py:691-762`; `cpp/src/orchestration/vo.cpp:794-802`; `rust/slam-it-orchestration/src/vo.rs:979-984,1010-1014`.
- **Issue:** The new tests cover Python's `parse`, `check` and `run`, and that every program refuses `--vocabulary` twice and `relocalize=maybe`. Nothing runs C++'s or Rust's `slam_it_vo` and asserts any of these:
  - a relocalising run with no `--vocabulary` (the default settings, so the likeliest mistake when run by hand) is refused;
  - `--vocabulary` with `relocalize=false` is refused;
  - a folder the reader refuses (another descriptor, a malformed array) exits non-zero with a message naming the folder, and writes no `run.json`. This was A1's symptom: exit 0 and an unreadable `run.json`.
- **Why it matters:** A1 was a must-fix. The readers are tested natively against 19 cases, but the three-line wiring that turns a reader error into an exit code has no regression test in either language. `grep "needs one\|but relocalize is off" cpp/tests rust` finds only the sources.
- **Evidence:** The behaviour is right today. `slam_it_vo ... --mode rgbd --count 0` with no vocabulary prints `--vocabulary: relocalize needs one`, exit 3. With `relocalize=false` and a folder it prints `given, but relocalize is off`, exit 3, and no output folder is created. No test pins it.
- **Suggested fix:** Add a test parametrised over `BACKENDS` (the pattern at `test_vo.py:747-762`) with these cases. For the third, copy the reader fixture with `header.npy`'s descriptor version changed. Assert exit 3, the folder in stderr, and no `run.json`.

#### [should-fix] The vocabulary and database crossings now hold their keys exactly, with no test (A4)
- **Where:** `cpp/bindings/foundations/vocabulary_crossing.hpp:51-60`; `cpp/bindings/edge/edge_crossing.hpp:499-511`; `rust/slam-it-py/src/foundations/vocabulary_crossing.rs:14-40`.
- **Issue:** 152b394 adds `check_keys` in C++ and a field-count check in Rust. No test passes a missing or extra key. `grep -rn "unknown\|missing\|extra" tests/foundations/test_vocabulary.py tests/edge/test_database.py` finds nothing relevant, and other crossings have such tests (`tests/foundations/test_ba.py:569`).
- **Why it matters:** It is the same class of drift A4 was raised for, and nothing keeps it fixed.
- **Evidence:** I probed both backends. An extra key and a missing `weights` each raise `ValueError`. C++ says `unknown field 'bogus'` or `missing field 'weights'`; Rust says `expected exactly the keys [...]` for both. Script: `.../panel2/keys.py`.
- **Suggested fix:**
  - Parametrise a test over the compiled backends for the vocabulary (`quantize`, `bow`) and for the database in `candidates` and `best_covisible`.
  - Optionally move Rust's check out of `field` (`vocabulary_crossing.rs:35`), which re-runs it for each of the seven keys, into one check at the top of `vocabulary_in`, as C++ does.

#### [should-fix] `orb.distance` is public and unvalidated; a signed dtype silently gives wrong distances
- **Where:** `src/slam_it/foundations/orb.py:399-418` (and its callers in `matching.py`, `vocabulary.py`).
- **Issue:** 8c45675 replaced four private popcounts with one public function. It has no dtype or width check, and its docstring has no `Raises`. Every internal caller passes validated uint8, but the function is exported, and the repo's rule is to validate public boundaries.
- **Why it matters:** `np.bitwise_count` counts the magnitude of a signed value, so a descriptor array of the wrong dtype gives wrong distances with no error. The C++ and Rust `distance` take a fixed `Descriptor`, so they cannot be misused this way.
- **Evidence:**
  - `orb.distance(np.zeros(32, np.int8), np.full(32, -1, np.int8))` returns `32`, where every bit differs and the answer is 256.
  - Two `(2, 5)` uint8 arrays return `[40 40]` with no width error.
- **Suggested fix:** Check `dtype == np.uint8` and `shape[-1] == DESCRIPTOR_BYTES` (O(1) per call), raise `InvalidInputError`, add a `Raises` section and a test.

#### [should-fix] A helper written twice in C++ tests, and an empty frame built three times in Rust tests
- **Where:** `cpp/tests/edge/scene.hpp:121` and `cpp/tests/edge/test_relocalization.cpp:27` (identical `one_word()`); `rust/slam-it-edge/tests/pipeline.rs:97-103` and `:161-169`, and `rust/slam-it-edge/tests/relocalization.rs:~44-50`.
- **Issue:**
  - c6ed83a added `scene::one_word()` to the shared `scene.hpp`, and `test_relocalization.cpp` keeps its own copy. It already includes `scene.hpp`.
  - Rust builds the same empty `Features`/`Frame::new` inline in the existing pipeline test, again in the new `empty_at`, and a third time in `relocalization.rs`. C++ shares `empty_frame()`/`empty_at`.
- **Why it matters:** This is duplication jscpd cannot see within one language. The two C++ vocabularies can drift apart.
- **Suggested fix:** Delete the local copy in `test_relocalization.cpp` and use `scene::one_word()`. Add an `empty_frame(timestamp)` to Rust's `scene/mod.rs`, as `one_word` was.

#### [should-fix] The online-SLAM plan's B1 section still describes the old design
- **Where:** `docs/design/2026-10-14-online-slam.md:95-110` (and the sentence at 108-109 about `vocabulary.json` recording the descriptor's settings).
- **Issue:** Item 1 says "The vocabulary, in rung 0 ... its files ... a `vocabulary.json` naming its k, levels, training set, seed and the SHA-256 of each array". After A3 and A1 the folder is eval's, the format is version 2 (`header.npy`, `sha256.npy`, `training.json`), and the native readers do not check the checksums. 806f8eb updated the ladder doc but not this plan.
- **Why it matters:** This plan is the live specification for B2-B4, which build on the vocabulary. A reader following it would build against a layout that no longer exists.
- **Suggested fix:** Edit item 1 to the as-built layout, or add one line saying B1's panel moved the folder to `slam_it.eval.vocabularies` (format version 2) and pointing to the ladder doc.

#### [note] The owner's two kept 6-level vocabularies are version-1 folders and no longer read
- **Where:** `../vocabularies/arkitscenes-6` and `../vocabularies/bovisa-6` (each holds a `vocabulary.json`); `src/slam_it/foundations/vocabularies/local.json`; `CHANGELOG.md`.
- **Issue:** `vocabularies.read` on either folder gives a bare `FileNotFoundError: .../header.npy`.
- **Evidence:** I ran `vocabularies.read("../vocabularies/bovisa-6")`.
- **Why it matters:** The arrays are unchanged and `local.json`'s checksums still hold. But the only documented route is a rebuild (hours, about 15 GB). Neither the CHANGELOG nor `local.json` says the two kept folders need rewriting.
- **Suggested fix (for the owner):** Convert them once (write `header.npy` and `sha256.npy` from the existing arrays), or say in `local.json` and the CHANGELOG that they need a rebuild.

#### [note] Hand-written native `.npy` parsers are looser than numpy's, and echo raw bytes in one message
- **Where:** `cpp/src/eval/vocabularies.cpp:51-72` (`header_value`), `:155`; `rust/slam-it-eval/src/vocabularies.rs:71-78`, `:160`.
- **Issue:**
  - The parsers find keys by substring and ignore the rest of the dict, so they accept a header with garbage in its padding, which numpy's `literal_eval` refuses.
  - The "unexpected .npy type" message prints the raw `descr` bytes of a corrupt file, unescaped.
- **Evidence:** In the 400-mutation fuzz of `slam_it_vo --vocabulary` (`.../panel2/fuzz.py`) there was no crash and no exit outside the controlled refusals. The only C++-accepts/Python-refuses cases were 15 `header.npy` files damaged after the closing brace. One stderr contained a byte that is not valid UTF-8 (`UnicodeDecodeError` in my harness).
- **Why it matters:** The data stay fully checked for type, shape and length, so this is harmless to results. It is still a difference from "fail the same way", and the fixture's 19 cases do not pin it.
- **Suggested fix:** Optionally add a padding-garbage case marked as Python-only, and quote or escape `descr` in messages.

#### [note] `PnpRun`'s two co-null fields make callers re-check
- **Where:** `src/slam_it/edge/tracking.py:245-285`; `tracking.py:987`; `relocalization.py:453` (`run.pose is None or run.inliers is None or run.count < needed`). `rust/slam-it-edge/src/relocalization.rs:486-491` vs `tracking.rs:896-912`.
- **Issue:**
  - `pose` and `inliers` are None together, so the `inliers is None` test only narrows the type for mypy. `replace(summary, **run.recorded())` also splats a dict, which hides field names from mypy.
  - Rust applies one `PnpRecord` to two summary types in two ways: four assignments in `try_candidate`, a destructure and struct update in `relocalize_locally`.
- **Suggested fix:** Optional. Return `PnpRun | None`, or make the fields non-optional on a success type, and use one way of applying the record.

#### [note] Small readability items in `eval/vocabularies.py`, its fixture generator and Rust's `read_npy`
- **Where:**
  - `src/slam_it/eval/vocabularies.py:151` (`(root / "header.npy").read_bytes()` repeats the path), `:152` and `:272` (positional header indices `0, 1, 2, 3, 4`, and a literal `5` that is `len(HEADER)`), `:252` (a constant `shapes` dict rebuilt on each call).
  - `tests/fixtures/eval/generate_vocabulary.py:177` (`json.loads(OUT.read_text())  # it reads back` discards its result).
  - `rust/slam-it-eval/src/vocabularies.rs:89-175` (`read_npy` is about 85 lines in one function, where the C++ twin splits `header_span`, `parse_shape` and `item_size`); the array length `5` in `[[u8; SHA256_BYTES]; 5]` where `ARRAYS.len()` is a constant expression.
- **Suggested fix:** Optional. Index the header by `HEADER.index(...)` or return a small record, hoist the shapes dict to a module constant, and drop the pointless line.

#### [note] Docs leftovers
- **Where:**
  - `docs/design/2026-10-02-slam-it-ladder.md`, the line added by 806f8eb ("the frame's step now, and the pipeline's state is two-valued. `_date` and `_close` ...") is 132 columns; the other added doc lines are at most 100.
  - `docs/notes/edge-vo.md:292` ("Ours are odometry ... with no relocalisation to recover a run that loses the track") is a present-tense statement of a pre-B1 measurement. This is H2's residue; the ladder says those runs have not been measured again, so add "before B1".
  - `src/slam_it/foundations/vocabulary.py:40-43` and `matching.py:70-74` import `orb` as a module and also import names from it (`from ... import orb` plus `from ...orb import DESCRIPTOR_BYTES`).
- **Suggested fix:** Reflow the one line, add the qualifier, and use one import style per module.

### Run 1 hygiene findings, checked

| ID | Result |
|---|---|
| H1 | Resolved. The int64 span check, the skipped leaf span, `from_chars` shapes and `check_parents` before `depths` are in `foundations/vocabulary.cpp` and `eval/vocabularies.cpp`. Native tests cover the parent, int32, leaf-too-deep, overflow and cut-short cases. The fuzz showed no abort or out-of-bounds read. |
| H2 | Resolved in the docstrings and comments, except the `edge-vo.md:292` leftover above. |
| H3 | Native tests exist for the vocabulary, database, relocalisation, `search_by_bow` and the relocalising pipelines. Settings coverage is the first finding. |
| H4 | Resolved. Every malformed-file fault is `InvalidInputError`, tested by the 19 cases. |
| H5 | Resolved. One `distance` per language, and no other popcount remains in `src`, `cpp` or `rust`. `numpy>=2.0`; the lock has 2.4.6. See the validation finding. |
| H6 | Resolved. All three databases call the vocabulary's vector check; `test_database.py` covers non-integer words. |
| H7 | Resolved. `_match_by_word` and `run_pnp`/`PnpRun`; Rust's macro is `record`/`kept`. See the `PnpRun` note. |
| H8 | Resolved. The descriptor-version constants are read, and the run writers write `Header`. |
| H9 | Resolved. Run format 5 requires `vocabulary`, `check_record` runs on read and write, and the status page defaults are gone. The viewer's `placed_by ?? []` is left pending the owner, as the report says. |
| H10 | Resolved in part; see the second finding. |
| H11 | Resolved. Results are streamed, a failed worker is a result, and `test_a_failed_worker_is_its_runs_result_and_the_others_are_kept` asserts values. |
| H12 | Resolved. The test went away with the binding's `read`. |
| H13, H14 | Resolved. The arrays are read-only, and `vo.run` reads the vocabulary once. Natively, the program reads the folder again by design. |
| H15 | Resolved. The `:func:` is fixed, `DEFAULT` says 9,969 of 10,000, the archives are closed through an `ExitStack`, and `words_of` is written once in Rust. |

### Checked and fine
- **Python reader (`eval/vocabularies.py`):** its docstring states every file's dtype and shape, and the exception for checksums. `_load` refuses big-endian, Fortran order and a shape its data do not fill before allocating, and returns read-only views. `write` validates `info` before writing anything and writes each file whole. Errors name the file and the bad value.
- **C++/Rust reader parity:** the 19 cases are run by all three readers (15 are refused by all three, 2 are read, 2 are refused by Python alone), with the same messages in all three. The Rust and C++ headers and docs are complete.
- **Rung 0 / eval boundary:** the new contract keeps `json` and `hashlib` out of the rungs, and the eval layer order is updated. The tree's `check` is fully before `depths`, with no unchecked index use in C++ or Rust.
- **Vocabulary parity fixture:** `tests/fixtures/foundations/vocabulary.json` comes from an independent brute force (nothing imported from `slam_it`). C++, Rust and Python hold it exactly, and each language's refusal cases are present.
- **Training rules (L4):** tie-bit, weight and tiny-training SHA tests assert values, and the fragility on numpy's Generator is documented.
- **Window room:** the Python test asserts the placed frame, the map, the lost frame, the culled keyframes' lives and "fewer than `lost_frames` failures". The golden record has 24 cases and `room_inputs` records the new constants.
- **Native database and relocalisation tests:** C++ and Rust test erase, double insert, malformed vector, placement at the true pose with every point, an empty frame, and the MLPnP budget.
- **Golden found bundle:** the layout test asserts frame 34 is relocalised in map 0, and the viewer tests read it through `loadBundle`/`loadTrack` instead of rebuilding columns.
- **Other items:** `measure_sharded` closes its streams through an `ExitStack` and reports a failed worker's error. `train_vocabulary` closes its archives. CMake and the test crates glob the new test files. No debug prints, TODOs or commented-out code in the added files.

Scratch files, all outside the repository (`/tmp/scratch/panel2/`): `ablate.py`, `ablate2.py`, `keys.py`, `fuzz.py`.

### Architecture review

## Architecture review

**Scope:** Run 2 of `feature/2b-b1-place-recognition`. I read the nine fix commits `e6d293a..98fed93` (121 files) in full in my area and checked them against Run 1's A1-A14, leaving out the deferred A12 and A13. I used the whole PR (`03f3d21...98fed93`) for context. I also checked the ladder doc and develop's `docs/design/2026-10-14-online-slam.md` (cited at 03f3d21) against the code. Paths are relative to the repository root.

What I ran:
- `lint-imports`: 22 contracts kept.
- `scripts/check_layers.py`: passed.
- A targeted pytest run (`tests/test_backends.py`, `tests/eval/test_vocabularies.py`, `tests/eval/test_runs.py`, `tests/foundations/test_vocabulary.py`, `tests/edge/test_database.py`): 250 passed.
- Two probes: the compiled crossings' key checks, and the native programs given three copies of the committed vocabulary (the default itself, one whose header names another descriptor, and one with an array changed under its checksum).

The repo's Rust program was stale, so I built a fresh one with `--locked` into the scratchpad's target dir rather than `rust/target`. Nothing in the repository was modified.

**Summary:** Every Run 1 architecture finding in scope is resolved as the report says. The probes confirm A1 and A4 on built code. The new boundary holds:
- rung 0 keeps the tree;
- eval reads its folder in all three languages;
- the run format is version 5, with no compatibility path;
- one Hamming distance per language.

There is no must-fix. There are three should-fixes:
1. The committed vocabulary's files still live in rung 0's package directory, and eval reaches into it by path.
2. "The algorithm rungs read no files" is enforced in Python only, though A3 was found in all three languages.
3. Develop's online-SLAM design now contradicts the code in several places.

The notes are small doc gaps and things to watch.

### Findings

#### [should-fix] The committed vocabulary's files are still rung 0's package data; eval reaches into `foundations/` by path
- **Where:**
  - `src/slam_it/eval/vocabularies.py:62`
  - `src/slam_it/foundations/vocabularies/default/`, including `header.npy` and `sha256.npy`, which 152b394 added there
  - `src/slam_it/foundations/vocabularies/local.json`
  - `CHANGELOG.md:32` ("in `vocabularies/local.json`")
- **Issue:** 152b394 is titled "a vocabulary's folder is eval's", and the ladder (lines 95-97) now says eval reads the rungs' files for them. The reader moved; the files did not.
  - `slam_it/foundations/` now holds a format-version-2 folder and a JSON record of the local vocabularies. No code there knows either format: `grep -rn vocabularies src/slam_it/foundations` finds only docstrings pointing at eval.
  - Eval finds the folder by walking into a sibling package's directory:
    ```python
    DEFAULT = Path(__file__).parents[1] / "foundations" / "vocabularies" / "default"
    ```
  - import-linter cannot see this dependency.
- **Why it matters:**
  - A change to the folder's format edits eval's code and rung 0's data.
  - A rename or move inside rung 0's tree breaks the default silently, until a test reads it.
  - It undercuts the boundary the fix set up: the rungs "read no files", but they still hold them.
- **Evidence:** `ls src/slam_it/foundations/vocabularies/default` lists `children.npy descriptors.npy header.npy parents.npy sha256.npy training.json weights.npy words.npy`. The `DEFAULT` line is quoted above.
- **Suggested fix:** Recommended: change the code.
  - Turn `slam_it.eval.vocabularies` into a package (`eval/vocabularies/__init__.py`), with `default/` and `local.json` beside it, and `DEFAULT = Path(__file__).parent / "default"`.
  - Eval's internal-order contract still treats it as one layer, and `wheel.packages` ships it unchanged.
  - Update the online-SLAM doc's §3.2 table (lines 124 and 127) and the CHANGELOG path.
  - The alternative is a doc change: record in the ladder that the committed vocabulary is rung 0's package data, which eval locates.

#### [should-fix] "The algorithm rungs read no files" is enforced for Python only
- **Where:**
  - `pyproject.toml:265-279`, the contract.
  - `scripts/check_layers.py:156-165`: forbidden contracts are reduced to their `slam_it.*` modules, so `json` and `hashlib` drop out.
  - `scripts/check_layers.py:690`: angle-bracket includes count as system headers and go unchecked.
- **Issue:** A3 was found in all three languages: C++ `vocabulary.cpp` used `<fstream>` and `<filesystem>`, and Rust `vocabulary.rs` used `std::fs::read`.
  - The fix holds Python by contract.
  - The native cores are clean today: no `<fstream>`, `<filesystem>` or `<cstdio>` in `cpp/{src,include/slam_it}/{foundations,edge}`, and `std::fs` only in the rung crates' trailing test modules (`rng.rs:150`, `orb.rs:1172` and the like).
  - Nothing keeps them clean. The ladder says eval reads the rungs' files "as its native libraries do for the C++ and Rust cores" (lines 95-97), and that the three languages "share one source of truth" for layering (line 445).
- **Why it matters:** A native core could start reading a file again and layer 1 would still pass. These cores are the ones the edge budgets and a possible phone target take standalone.
- **Evidence:** The `check_layers.py` excerpt, in which only modules starting with `slam_it.` are subtracted:
  ```python
  forbidden = {m.removeprefix(prefix) for m in contract["forbidden_modules"] if m.startswith(prefix)}
  ```
- **Suggested fix:** Give the contract a native reading in `check_layers.py`:
  - for the contract's source rungs, refuse `<fstream>`, `<filesystem>` and `<cstdio>` in `cpp/include/slam_it/<rung>` and `cpp/src/<rung>`;
  - refuse `std::fs` and `std::fs::File` in rung crates' `src`, outside the trailing `#[cfg(test)] mod tests`, which the script already splits off.
  - Or have the ladder say that the native side is unchecked.

#### [should-fix] Develop's online-SLAM design contradicts the code
- **Where:** `docs/design/2026-10-14-online-slam.md` at 03f3d21. The branch carries fd530cb's older copy, and the merge keeps develop's.
- **Issue:** Each of these is now false:
  - **Lines 95-99:** the vocabulary's "files, `.npy` arrays all three languages read ... and a `vocabulary.json` naming its k, levels, training set, seed and the SHA-256 of each array". The code instead has:
    - folder format version 2;
    - `header.npy`, int64 (5,): format version, descriptor version, k, levels, words;
    - `sha256.npy`, uint8 (5, 32);
    - `training.json`, read by Python only;
    - all of it read by `slam_it.eval.vocabularies`, `slam_it_eval` and `slam-it-eval`, not by rung 0.
  - **Lines 105-107:**
    - The doc says the arrays "must match their recorded checksums"; only Python checks them (the ladder's exception, lines 339-343).
    - The doc says the run records "its `vocabulary.json`, the checksums with it". The record is now the header's numbers plus each array's SHA-256, with no training-set name.
  - **Lines 109-112:** the doc says "its `vocabulary.json` records the descriptor's settings (the patch, the smoothing, the test pattern)". A descriptor version number stands for them now (`orb.DESCRIPTOR_VERSION`, `header.npy[1]`), and every reader checks it.
  - **Line 149:** "The run format, still version 4". It is version 5, with `vocabulary` required.
  - **Lines 141-143 (from B1's original commits):** "on, a run needs a vocabulary, the committed one unless `--vocabulary` names another". That holds for `vo.run` only. Every program, Python's included, refuses relocalisation without `--vocabulary`. I checked: `python -m slam_it.orchestration.vo ...` gives "`--vocabulary: relocalize needs one`".
  - **Lines 157-160:** incomplete rather than wrong. Parity also gained the window room (24 cases), the found golden bundle, and native tests.
- **Suggested fix:** Change the doc; the code is the decided fix. Commit it on develop once the owner approves the wording, under the docs-only rule.

#### [note] The ladder's package table omits eval's new module
- **Where:** `docs/design/2026-10-02-slam-it-ladder.md:377`.
- **Issue:** The Python column lists eval's modules one by one (`video`, `lenses`, `zju`, ...) but not `vocabularies`. The C++/Rust column, "(native PNG and dataset readers, run writer, a program's process for its run)", leaves out the vocabulary reader.
- **Suggested fix:** Add both. This branch already edits the ladder.

#### [note] A standalone native run records the checksums its folder claims, not ones it checked
- **Evidence:** The probe's "tampered" copy changes `weights.npy` under its recorded SHA-256.
  - Python refuses it: "its SHA-256 is not the recorded one".
  - C++ and Rust both exit 0, and `read_run(out).vocabulary == vocabularies.record(DEFAULT)` is `True`.
- **Issue:** This follows from the decided exception and is not a re-raise of A1. The ladder (lines 339-343) and the module docstring say such a program "trusts its arrays". They do not say that its `run.json` then names checksums its arrays may not have.
- **Suggested fix:** Add one clause to the exception, so that a reader of a run knows its `vocabulary` record is verified only when `vo.run` launched it. `vo.run` is the only launcher in the repository; the worker and `measure_testset` go through it.

#### [note] The Python contract could cover more of what reading a file takes, and must grow with the rungs
- **What it could add:** `pathlib`, `io`, `os`, `shutil`, `tempfile`, `zipfile`, `tarfile` and `pickle`. No algorithm rung imports any of them today. I listed every top-level import in the six source packages: numpy, math, typing, dataclasses, collections, enum, bisect, itertools, time, contextlib.
- **What it cannot see:** builtin `open()` and `np.load`, so the contract's name still claims more than it checks.
- **What it must grow with:** its source list is explicit, not exhaustive. When `slam_it.calib` lands (the ladder's layers at line 414 already list it), it has to be added by hand.

#### [note] Natively, the `.npy` format is written in one eval module and read in another
- **Where:** `cpp/src/eval/runs.cpp:395` writes and `cpp/src/eval/vocabularies.cpp:76` reads; Rust's are `rust/slam-it-eval/src/runs.rs:305` and `vocabularies.rs:94`.
- **Issue:** Run 1's A3 worry, `.npy` handling split across rungs, is gone: both now sit in eval. When a second native reader arrives (for example B4's kept-map arrays), give the format one `npy` module beside `text` and `png` rather than copying the reader.

### Checked and fine
- **A1:** Native readers check the format and descriptor versions against `kVersion` / `VERSION` and `kDescriptorVersion` / `DESCRIPTOR_VERSION`, which are live now.
  - Probe: the foreign header is refused with exit 3 by C++ and Rust, with Python's message.
  - The default's record written by both native programs equals Python's.
  - The CHANGELOG is corrected.
- **A2:** `tests/fixtures/foundations/vocabulary.json` comes from an independent generator, which imports only numpy and the fixtures' layout helper.
  - C++ (`cpp/tests/foundations/test_vocabulary.cpp`) and Rust's trailing test module are held to it.
  - The reader's 19 cases in `tests/fixtures/eval/` run in all three languages; CMake picks up the native tests by glob.
- **A3:** The folder's I/O is in `slam_it.eval.vocabularies` and its native twins.
  - Rung 0's C++ and Rust headers say "takes a tree's arrays, never its files".
  - The bindings' `read` is gone, and the `Implementation` Protocol is `quantize` / `bow` / `score`. `test_backends` passes.
- **A4:** Probe: an extra key and a missing key are refused by both compiled backends, for the vocabulary and for the database (`check_keys` with `kVocabularyKeys` / `kDatabaseKeys`; Rust's exact-length check with every key read).
- **A5:** The int32 tree exception is recorded in the ladder (lines 313-315) and matches the crossings.
- **A6:** `viewer/tests/golden.js` loads `found_bundle/` through `loadBundle` / `loadTrack` and no longer reads `room_runs.json`. `test_bundles.py` holds the found bundle to what the bake writes.
- **A7:** The edge and rung-0 package docstrings name the vocabulary, the database and relocalisation. The three pipeline comments now say the words only relocalise.
- **A8:** The ladder records the database as the edge rung's (it moves to rung 0 only when the classic rung needs it), closes A8's rest and the two-view item, marks A13 as met in part, and states L15's current state.
- **A9:** Run format version 5 in all three languages, with `vocabulary` required and checked by `check_record` on write and read. The golden run fixture is version 5. No `setdefault` is left.
- **A10:** `vo.run` reads the vocabulary once, and a read vocabulary's arrays are read-only. `default()` remains a lazy cache used only by tests, which is harmless now.
- **A11:** The edge order is `(adjust) | (database)` over `(residuals)`, in import-linter and `check_layers.py` alike.
- **A14:** Native database and relocalisation tests now exist (`cpp/tests/edge/test_database.cpp`, `test_relocalization.cpp`, `rust/slam-it-edge/tests/database.rs`, `relocalization.rs`).
- **Eval's order:**
  - `vocabularies` sits at the bottom, with `runs` above it: C++ `runs.hpp` includes `vocabularies.hpp`, Rust uses `crate::vocabularies`.
  - `slam_it_eval` DEPENDS only on `slam_it_foundations`, and the `slam-it-eval` crate depends only on `slam-it-foundations`.
  - No new runtime dependency in any language; numpy's floor is 2.0.
- **One Hamming distance per language:** `orb.distance` / `orb::distance`, called by matching, stereo, the vocabulary, and the edge rung's map and mapping. No other popcount is left in the cores.
- **Ladder exceptions:** the int32 crossing and the checksums-Python's-alone exception match the code, and `vo.run` reads the folder before every program it launches.
- **Error contract:** a missing file is `FileNotFoundError` in Python and `InvalidInput` naming the file natively, as the ladder says. A malformed header, descriptor or tree fails with the same message in all three languages.
- **Edge relocalisation split:** `PnpRun` / `run_pnp` live in tracking in all three languages, and relocalisation calls them from the layer above. No cross-module private names.
- **Other:** no reifyscene import and no import-time side effects (`DEFAULT` is a path constant). `slam_it.backends` is unchanged. Rust's exact-keys check remains a private helper in each crossing, as it already was before B1.

Files outside the repository:
- Probes:
  - `/tmp/scratch/panel2/probe_keys.py`
  - `/tmp/scratch/panel2/probe_native_vocab2.py`
- The probes' runs: `/tmp/scratch/panel2/work/`
- The scratch Rust build: `/tmp/scratch/panel2/target/` (it can be deleted).
- Saved for reuse: `probe_native_vocabulary_header.py`, the native-vocabulary probe for the new header format. It replaces Run 1's `probe_native_vocabulary_checks.py`, which reads the old `vocabulary.json` and no longer works; I left that file in place.
- The repo's `rust/target/release/slam_it_vo` is older than `rust/slam-it-edge/src/pipeline.rs`. It needs a rebuild before layer 1.

### Algorithm review

## Algorithm review

**Scope:** This run covers the fixes made since Run 1, `e6d293a..98fed93` (the nine commits from 152b394 to 98fed93), on `feature/2b-b1-place-recognition`. I checked L1 to L6 against what the Run 1 report says was done (L7 deferred, L8 waived). Files read in full:
- the window room: `tests/orchestration/datasets.py`, its golden cases, and the test in `tests/orchestration/test_vo.py`;
- `tests/edge/test_relocalization.py`;
- the native pipeline, database and relocalisation tests in C++ and Rust;
- `tests/fixtures/foundations/generate_vocabulary.py` and its fixture;
- training's new known-answer tests;
- `orb.distance` in all three languages;
- `tracking.run_pnp` / `PnpRun`, the split of `relocalization._try`, and Rust's `PnpRun::record` / `kept`;
- the corrected loss-timer docstring in `src/slam_it/edge/pipeline.py`, against ORB_SLAM3 master `src/Tracking.cc`.

Paths are relative to the repository root.

Tests run, with nothing in the repository modified:
- **Python:** the vocabulary, orb-distance, database, relocalisation and database-through-culling tests: 118 passed.
- **`test_vo.py -k "window or golden_record or found or moved"`:** 107 passed, all backends.
- **C++:** built in a scratch tree, `ctest -R 'edge|vocabular|matching|orb|eval'`: 17 of 17 passed.
- **Rust:** `cargo test -p slam-it-edge -p slam-it-foundations -p slam-it-eval` in a scratch target: all passed.

For the panel: `rust/target/release/slam_it_vo` is older than `rust/slam-it-edge/src/pipeline.rs`, because 806f8eb changed only a comment there. Until layer 1 rebuilds it, `test_vo.py` fails 28 Rust cases with `StaleProgramError`. For the Rust runs above I built HEAD's program in scratch and pointed `slam_it.backends._RUST_PROGRAMS` at it.

Scratch scripts are in `/tmp/scratch/panel2/scripts/`.

**Summary:**
- **Resolved:** L2, L3 (docstring), L4, L5 and L6 are resolved as the report says. The fixes change no decision: the golden record's 20 earlier cases are byte-identical to e6d293a's.
- **The loss timer (L1's first half) is now tested well natively.** The 0.25 s frame steps make the 3.0 s boundary exact in float64. The window room separates the time rule from `lost_frames`, and it also tests ORB-SLAM3's order: relocalise first, judge the window only after a failure.
- **L1's second half is not met.** Of the eight relocalisation settings the fix moved off their defaults, six change no decision in any test. No test can see the narrow search's result. A native that ignored any of them would pass.
- **Smaller items:** one should-fix on whether the vocabulary weights really rebuild exactly, and four notes.

### Findings

#### [must-fix] L1 is only half resolved: six of the eight moved relocalisation settings, and the whole narrow search, decide nothing in any test
- **Where:**
  - `tests/edge/test_relocalization.py:162-198`: `MOVED`, `MOVED_CASES`, and `test_the_moved_settings_change_what_relocalisation_decides`, which checks only the bundle (`assert moved.summary.as_dict() != default.summary.as_dict()`).
  - `tests/orchestration/datasets.py:216-238`: `WINDOW_OPTIONS`.
  - The narrow search: `src/slam_it/edge/relocalization.py:345-381` (`_project_more`), `cpp/src/edge/relocalization.cpp:90-131`, and `rust/slam-it-edge/src/relocalization.rs` `project_more`.
  - The native budget tests use the defaults only: `cpp/tests/edge/test_relocalization.cpp:89-96` and its Rust twin.
- **Issue:**
  - **Test:** I reverted each moved setting alone to its default and compared Python's result. In the stage parity's two moved cases, only `reloc_window` and `reloc_confidence` change anything. Across the window room's four golden cases (run with the C++ program, which the record holds equal to Python), likewise. Never decisive anywhere:
    - `reloc_inlier_ratio`, `reloc_min_inliers`, `reloc_ransac_iterations`: every RANSAC run stops on confidence after 1 or 2 samples, with inliers far above either threshold;
    - `reloc_narrow_inliers`, `reloc_narrow_window`, `reloc_narrow_distance`.
  - **The narrow search:** in the default "narrow search" case (frame 6, `recovery_tracked=175`), the steps are refined 69, the wide search adds 117, re-optimised 168, the narrow search adds 3. Since 171 < 175, `_project_more` returns the second optimisation's result and drops the narrow search's matches. A native that skipped the narrow search, or ran it with any window or distance, would agree with Python.
  - **The "moved" narrow case:** `reloc_window=2.0` makes the wide search add 100, and 69 + 100 = 169 < 175. So neither the second optimisation nor the narrow search runs. The "69, not 168" difference that c3f20da cites comes from `reloc_window`, not from any narrow setting.
  - **The golden record:** no candidate in any case reaches the narrow search. Only `window/tum/rgbd` frame 33 runs the wide search (59 → 115).
  - **Coverage:** the narrow search's final optimisation, and the rule that the second optimisation's outliers keep their matches until the last, run in no test in any language.
- **Why it matters:**
  - L1 was a must-fix, recorded as fixed. Three docstrings now state a property that is false:
    - `MOVED`: "a native that hard-coded one would decide unlike Python on the frames below";
    - `MOVED_CASES`: "every setting it moves is used";
    - `WINDOW_OPTIONS`: the same claim.
  - c3f20da's message makes the same claim ("so the agreement shows the natives read them"). It is on an unmerged branch, so under the owner's rule the contradiction should be raised with them.
  - The native narrow search and final optimisation read correctly against Python (checked), but nothing guards them against regression.
- **Evidence** (`scripts/reloc_sensitivity.py`, `scripts/window_sensitivity.py`, `scripts/narrow_trace.py`):
  ```
  frame 30 (wide, moved): only reloc_window reverted -> CHANGES
  frame 6 (narrow, moved): reloc_confidence, reloc_window -> CHANGES; the other six: no change
  window tum/rgbd:  changes: reloc_window, recovery_tracked, redundant_*, relocalize_seconds
  window euroc/stereo: changes: reloc_confidence, relocalize_seconds
  window monocular (both): changes: reloc_confidence, reloc_window, redundant_observers, relocalize_seconds
  narrow search (default): optimize 69; search w=10 +117; optimize 168; search w=3 +3   (discarded: 171 < 175)
  narrow search, moved:    optimize 69; search w=2 +100                                 (169 < 175: stops)
  ```
- **Suggested fix:**
  - **The narrow search:** add stage-parity cases where each setting decides, all found with `scripts/decisive_probe.py` and `scripts/narrow_place_probe.py`:
    - `(6, recovery_tracked=169)` places the frame through the narrow search's final optimisation (168 → +3 → 169);
    - `(6, recovery_tracked=170)` with `reloc_narrow_window=2.0`, with `reloc_narrow_distance=50`, or with `reloc_narrow_inliers=168`: each one decides.
  - **The RANSAC settings:**
    - `(6, recovery_tracked=175, reloc_ransac_iterations=1)` ends on `max_iterations`;
    - `(6, …, reloc_inlier_ratio=1.0)` is not refined;
    - `(30, recovery_tracked=150, reloc_min_inliers=85)` is dropped before RANSAC.
  - Test natively `ransac_budget` with non-default settings.
  - Replace the bundle assertion with a per-setting one: reverting any one moved setting changes Python's result in at least one case. Then the docstrings' claim is checked instead of asserted.

#### [should-fix] "Rebuilt exactly" for the vocabulary weights depends on the CPU, as the new test itself says
- **Where:**
  - `src/slam_it/foundations/vocabulary.py:582`: `weights[seen > 0] = np.log(n_images / seen[seen > 0])`, on an array.
  - `src/slam_it/foundations/vocabularies/local.json:2`: "each rebuilt exactly … the rebuild's arrays must have these SHA-256", with a weights SHA-256 at lines 21 and 44.
  - `CHANGELOG.md:29`: "the same arrays on a rebuild".
  - Against `tests/foundations/test_vocabulary.py:319`: "The weights are `np.log`'s, whose last bit may differ between CPUs, so they are checked against their rule rather than pinned."
- **Issue:** numpy 2.4 has its own AVX-512F float64 log, `AVX512F_log_DOUBLE` in `numpy/_core/src/umath/loops_exponent_log.dispatch.c.src`: a table-based range reduction and series, not libm's. It is chosen at runtime on AVX-512F CPUs. This machine reports `found: X86_V3`, `not_found: X86_V4, AVX512_ICL, AVX512_SPR`. Here `np.log` equals `math.log` bit for bit on 200,000 ratios of the form 9969/n. So the committed and kept weights are libm's, and a rebuild on an AVX-512 machine may differ in the last bit and fail its weights SHA-256 although nothing changed. The L4 fix spotted this for the tiny training but left the claim standing for the real vocabularies.
- **Why it matters:** the record promises a check, a bit-exact rebuild, that can fail on correct hardware. It fails loudly, not silently, but it would send someone looking for a training change that does not exist.
- **Suggested fix:** hold a rebuild to the four integer arrays' SHA-256 exactly, and the weights to their rule within a few ULP, as the new test does. State this in `local.json`'s `_about` and in the CHANGELOG line. Computing the IDF with a scalar log per distinct `N_i` narrows the exposure but does not remove it, since glibc selects FMA variants of `log` by CPU. The tolerance route is the robust one.

#### [note] The window room's docstring misstates why frame 33 is tried, and undersells what the room tests
- **Where:** `tests/orchestration/datasets.py:206-209`, `WINDOW_BLANKS`: "so that the frame after them … is tried within its short `relocalize_seconds`".
- **Detail:**
  - Frame 33 comes 0.067 s after the first failed frame (31), past the 0.05 s window. It is tried, and relocalised, because the window is judged only on a frame that fails every attempt, and frame 32 at 0.033 s was within it.
  - That is ORB-SLAM3's order: `bOK = Relocalization(); if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK) mState = LOST;` (`Tracking.cc`, the RECENTLY_LOST branch).
  - So the room also catches a native that judged the window before trying: it would lose the map on frame 33. Worth saying so in the docstring.

#### [note] The vocabulary fixture covers ties by chance, not by design
- **Where:** `tests/fixtures/foundations/generate_vocabulary.py` `_between` and `_descriptors`.
- **Detail:**
  - `_between(a, b)` is a tie only when the two centres' distance is even.
  - It is a tie only at a node the descriptor actually reaches. The centres are independent random descriptors, not nested, so a descriptor "between" two grandchildren usually descends elsewhere. Of the 7 such descriptors, only the root's intended tie occurs.
  - Even so, the fixture has 14 ties at visited nodes on all three levels, two of them three-way at the root, so the lowest-index rule is well covered today.
  - Suggest the generator assert that ties at visited nodes occur on every level, so a regeneration with another seed or tree keeps the coverage.

#### [note] L3's PR-description half has no PR to land in yet
- `gh pr list --repo evan-wes/slam-it --head feature/2b-b1-place-recognition` returns `[]`.
- The correction of 80c960a's loss-timer claim, and now c3f20da's narrow-search claim (first finding), need to go into the PR body when it is opened.

#### [note] Backlog, outside B1: the same bundle-only check weakens H21's `MOVED_OPTIONS`
- Reverting each of its 33 settings alone on the gap room (C++ program, four cases; `scripts/moved_options_sensitivity.py`), 11 change no decision:
  - `stereo_search`, `init_parallax_cos`, `min_motion_inliers`, `recovery_tracked`;
  - `min_distance_margin`, `max_distance_margin`, `ref_min_observations_early`, `min_epipole_distance2`;
  - `init_ba_iterations`, `init_ransac_iterations`, `reset_keyframes`.
- Some are moved decisively elsewhere:
  - `init_ransac_iterations` in `test_initialization.py`'s backend test;
  - `recovery_tracked` in the relocalisation stage parity;
  - `reset_keyframes` in the kept and window rooms.
- The per-setting check above would harden `test_the_moved_settings_change_the_decisions` too.

### Checked and fine
- **L1, the timer, natively:** `check_lost_relocalizing` and `relocalizing_a_large_map_is_lost_in_time_and_a_small_one_in_frames` step at 0.25 s, so the 13th failure sits at exactly 3.0 s after the first. These tests catch:
  - `>=` written for `>`;
  - the time taken from the last failure or from the last tracked frame;
  - a hard-coded window;
  - the `lost_frames` rule for small maps;
  - relocalisation tried on every failed frame (`summary.relocalization` set).

  `check_relocalized` places a frame the fallback cannot reach in map 0, at its pose to 1e-6. Both pass in C++ and Rust.
- **The window room's record** matches the commit's claims:
  - stereo and RGB-D are relocalised on frame 33 in map 0 and lost (kept) on frame 38, after 3 failed frames (`lost_frames` is 5);
  - RGB-D culls 23 keyframes on frames 8 to 30;
  - monocular loses 2-keyframe kept maps on frames 11 and 25.

  A timer not cleared on a tracked frame would lose the second stretch on frame 36, so it is caught.
- **L2 across languages:** the RGB-D record pins `sharing: 2` at frame 33 after the culls, and monocular pins `sharing: 2` after its cull on frame 8. A native database holding culled keyframes would share many more, or fail on `row_of`, so it could not reproduce them. The native `erase` tests check the word lists, the candidates, a double erase and an empty index. The Python test checks the database against the map's keyframes after every frame of a culling run.
- **L3:** the corrected statement matches `Tracking.cc`:
  - `mTimeStampLost` is set only when the state leaves OK (the frame-tracking failure branch, and the end-of-`Track` `else if (mState == OK)`);
  - `Relocalization()` sets `mnLastRelocFrameId` but never `mState`;
  - the frame that loses the track does not relocalise; a RECENTLY_LOST frame only relocalises.

  The design doc's loss-flow summary (`2026-10-14-online-slam.md:79-83`) also matches, including the reset of a LOST map with 10 keyframes or fewer (`KeyFramesInMap()<=10`) at the end of `Track`.
- **L4:**
  - The tie test matches DBoW2's `FORB::meanValue` (`N2 = size/2 + size%2`, set when `sum >= N2`).
  - The log 1.5 / 0 / log 3 test matches `setNodeWeights`: a word with `Ni = 0` keeps the Node constructor's weight 0, the first of two equal leaves takes the descriptor, and the tolerances are explicit (`rtol=0, atol=1e-15`; `rtol=1e-15`).
  - The tiny training is pinned by its four integer arrays.
- **L5 and L6:** the C++ and Rust readers refuse another format or descriptor version. The module docstring states what `N` counts.
- **The vocabulary parity fixture:** the brute force is independent (nothing imported from `slam_it`). It covers leaves on levels 1 and 2, the direct node of a leaf reached above level 2, a weight-0 word dropped from vectors (`[3, 3, 3]` gives an empty vector), and empty and disjoint vectors scoring 0. Its exact policy is applied in Python, and natively by `==`. serde_json has `float_roundtrip` on.
- **One Hamming distance:** the same 64-bit popcount in C++ and Rust, and a broadcasting `bitwise_count` in Python. Every call site keeps its shapes. The tiny-training SHA-256, pinned before 8c45675, still passes. The numpy floor is 2.0.
- **`run_pnp` / `PnpRun`:** Python's `recorded()` and Rust's `record()` / `kept()` reproduce the old blocks field for field: degenerate means no pose, and the inlier cap is applied after recording. `_try`'s split keeps the order of the checks. The C++ and Rust narrow-search code mirrors Python's (both conditions strict, outliers held, the skip set the merged points).
- **Golden record:** 24 cases; the 20 earlier ones are identical to e6d293a's, as is the tolerance policy.

Sources:
- [ORB_SLAM3 Tracking.cc (master)](https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/src/Tracking.cc): `Track` lines 1938-2012, 2142-2163, 2270-2288; `Relocalization` lines 3755-3775.
- [DBoW2 FORB.cpp](https://raw.githubusercontent.com/dorian3d/DBoW2/master/src/FORB.cpp) and [TemplatedVocabulary.h](https://raw.githubusercontent.com/dorian3d/DBoW2/master/include/DBoW2/TemplatedVocabulary.h) (Gálvez-López and Tardós, "Bags of Binary Words for Fast Place Recognition in Image Sequences", T-RO 2012).
- [numpy v2.4.0 loops_exponent_log.dispatch.c.src](https://raw.githubusercontent.com/numpy/numpy/v2.4.0/numpy/_core/src/umath/loops_exponent_log.dispatch.c.src) (`AVX512F_log_DOUBLE`).
- [numba PR 8027](https://github.com/numba/numba/pull/8027), on numpy dispatching transcendentals to SVML on AVX-512.
- [Fedora/AlmaLinux numpy patch on SVML gating](https://git.almalinux.org/rpms/python3.14-numpy/src/branch/c9s/fix-test_validate_transcendentals-x86_64.patch).
- Campos et al., "ORB-SLAM3", T-RO 2021; Mur-Artal and Tardós, "ORB-SLAM2", T-RO 2017.
