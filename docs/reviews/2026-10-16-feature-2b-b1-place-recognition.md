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
| H9 | should-fix | hygiene | `eval/runs.py` `_read_info` | `setdefault("vocabulary", None)` keeps 0.3.9.0's runs readable: a compatibility path | fix (run format version 5, the key required) | fixed (152b394: run format version 5, `vocabulary` required; 2a3eb34: the status page's `relocalizations ?? 0`). -- decided by evan-wes, 2026-10-16. The viewer's `placed_by ?? []` is kept, pending evan-wes's word: bundles carried `placed_by` before B1 (the golden bundle at fd530cb has it), so it types an unchecked column rather than serving older bundles |
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
