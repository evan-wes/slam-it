# Review panel: `chore/public-readiness`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N".
     Every path is relative to the repository root, never absolute or home-relative. -->

## Run 1 -- 2026-10-18

| | |
|---|---|
| **Branch** | `chore/public-readiness` |
| **Range** | `545f152e31e0eb9d676d1a6d4c701983f0ad0867...a77382a8870c6f948ab94c8ffdf436cfc8e84767` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet) alone: a version-micro PR, by the owner's choice |
| **How they ran** | registered slam-it-* agent |
| **Layer 1** | all checks passed (`scripts/check.sh`, exit 0; 25 checks; 4,267 tests) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `docs/history/pull-requests.md:13-16` | The intro says commit IDs are the first repository's, but each heading's merge commit is this history's | fix | fixed (238d59bb: the intro says each heading's merge commit is this history's ID, and IDs in descriptions the first repository's) -- decided by evan-wes, 2026-10-18 |
| H2 | should-fix | hygiene | `docs/history/pull-requests.md:3-5` | Merge dates are UTC, unstated, and 8 of 22 differ by a day from the CHANGELOG's local dates | fix | fixed (238d59bb: each date is its merge commit's own, in the author's time zone; the intro rewrapped) -- decided by evan-wes, 2026-10-18 |
| H3 | should-fix | hygiene | `src/slam_it/foundations/homography.py:328-338` | The OpenCV-derived homography decomposition has no in-source attribution, unlike P3P | fix | fixed (e64d533e: a one-line pointer to the notices in the three homography files, as P3P's) -- decided by evan-wes, 2026-10-18 |
| H4 | should-fix | hygiene | `tests/test_no_local_paths.py:25-26` | The guard's path patterns need a trailing slash, so common spellings pass | fix | fixed (189dc6d7: no trailing slash needed; the Windows form added; each spelling in the self-test) -- decided by evan-wes, 2026-10-18 |
| H5 | should-fix | hygiene | `tests/test_no_local_paths.py:150-160` | The guard's self-test checks line numbers only; the exception and file scan are untested | fix | fixed (189dc6d7: the self-test asserts (line, kind) pairs; tests of an allowed mention's scope and of the binary skip) -- decided by evan-wes, 2026-10-18 |
| H6 | note | hygiene | `tests/test_no_local_paths.py:9-10` | Name digests are a tripwire, not a redaction | waive | waived: the guard is a tripwire against committing the name, not a redaction -- decided by evan-wes, 2026-10-18 |
| H7 | note | hygiene | `rust/slam-it-py/pyproject.toml` | The `slam-it-rs` wheel's metadata does not carry the MPL file's licence | defer | deferred (`docs/design/2026-10-02-slam-it-ladder.md`, open items: deferred from the readiness PR's review panel) -- decided by evan-wes, 2026-10-18 |
| H8 | note | hygiene | `THIRD_PARTY_NOTICES.md:9,17,25,36,45` | Leftovers: a hard wrap kept by the byte-identical move, em dashes in headings, `UMAX` for C++'s `kUmax` | fix (two of three) | fixed (e64d533e: the notices' headings without em dashes; C++'s table named `kUmax`); the moved code's hard wrap waived: the move stays byte-identical -- decided by evan-wes, 2026-10-18 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `chore/public-readiness`, `545f152e31e0eb9d676d1a6d4c701983f0ad0867...a77382a8870c6f948ab94c8ffdf436cfc8e84767` (5 commits, 40 files). I read in full:
- `rust/slam-it-foundations/src/linalg/jacobi_svd3.rs` and `linalg.rs`
- `tests/test_no_local_paths.py`
- `THIRD_PARTY_NOTICES.md`, the README licence section, the CHANGELOG entry and `LICENSES/*`
- the intro and structure of `docs/history/pull-requests.md` and `docs/history/commit-map.txt`
- the reviewer prompts, skill and finding format
- the PoseLib headers, the doc corrections, and the `pyproject.toml` and VERSION changes

I also ran probes outside the repository, listed under "Checked and fine".

**Summary:** This is clean, careful work. The SVD move is byte-identical, the notices match the tree, and the guard test passes and is well documented. There are no must-fix items. Five should-fix items are small and are mostly accuracy gaps in `docs/history/pull-requests.md`, one inconsistent in-source attribution, and gaps in the guard's patterns and self-test. A few notes follow.

### Findings

#### [should-fix] `pull-requests.md` says commit IDs are the first repository's, but its "merge commit" IDs are this history's
- **Where:** `docs/history/pull-requests.md:13-16` (the intro) and the 22 `merge commit` fields (first at `docs/history/pull-requests.md:22`).
- **Issue:** The intro says commit IDs quoted "in descriptions, review reports, notes and commit messages are the first repository's" and that `commit-map.txt` gives each one's ID in this history. The 22 merge-commit IDs in the page headers are not first-repository IDs. They are this history's IDs, and they are not in the map's old column.
- **Why it matters:** A reader who follows the intro and looks up `2eb4e661...` in the map's old column finds nothing. The page uses one ID space in the headers and the other in the bodies, and says so nowhere.
- **Evidence:**
  - Of the 22 merge-commit IDs, 0 are in the old column of `docs/history/commit-map.txt`, 22 are in its new column, and 22 resolve in `git rev-list --all`.
  - The `v0.1.1.0` tag annotation names `2eb4e661ec3845d980071c8e9fea8830a43c4cff`, PR 1's merge commit on the page, as its target in this history.
  - The 39 short IDs quoted in descriptions all prefix-match the old column and none match this history.
- **Suggested fix:** Add one sentence to the intro: "The merge commit in each heading is this history's ID; IDs inside descriptions are the first repository's (see `commit-map.txt`)." Or print the old ID too.

#### [should-fix] The page's merge dates are UTC, which the intro does not say, and they disagree with the CHANGELOG's dates
- **Where:** `docs/history/pull-requests.md:3-5` (the intro) and the `Merged` lines.
- **Issue:** Every date on the page equals the merge commit's UTC date. The CHANGELOG and the commit timestamps use the owner's local time (UTC-7).
- **Why it matters:** 8 of the 22 PRs show a date one day later than the local date. PR 22 reads "Merged 2026-10-18", but its merge commit is `2026-10-17T19:38:04-07:00` and the CHANGELOG dates 0.3.11.1 as 2026-10-17. The intro's "(2026-10-02 to 2026-10-18)" has the same offset.
- **Evidence:** I compared each `Merged <date>` with `git log -1 --format=%cI <merge commit>`. The mismatches are PRs 4, 9, 12, 13, 14, 15, 18 and 22.
- **Suggested fix:** Say "dates are UTC, as GitHub shows them". Also re-wrap the stray short line in the intro's second paragraph ("1–22 (2026-10-02 / to 2026-10-18)").

#### [should-fix] OpenCV's homography decomposition has no in-source attribution, while the P3P files now do
- **Where:** `src/slam_it/foundations/homography.py:79-80` and `:328-338` and `:434-438`, `cpp/src/foundations/homography.cpp`, `rust/slam-it-foundations/src/homography.rs`. Compare `cpp/src/foundations/p3p.cpp:1` and `rust/slam-it-foundations/src/p3p.rs:1`.
- **Issue:** `THIRD_PARTY_NOTICES.md:28-34` says the decomposition "follows OpenCV's `HomographyDecompInria`": its helper functions, its table of plane normals and the order of its four motions. The only mention of OpenCV in the three sources is one tolerance constant (`homography.py:79`). The decomposition itself is credited only to Malis and Vargas. This PR gave the PoseLib-derived files a header line pointing at the notices, but not these.
- **Why it matters:** The attribution lives in one place only. A reader of the BSD-derived code never learns it is derived.
- **Evidence:** `grep -in "opencv\|inria" ` over the three homography sources returns only the constant and doc mentions of "normalised"; there is no `Samson Yilma` or `THIRD_PARTY_NOTICES` text in them.
- **Suggested fix:** Add the same one-line header, "see THIRD_PARTY_NOTICES.md", to the decomposition section of the three files, or accept the gap deliberately. The notices' scope is the owner's call; this finding is only about the inconsistency.

#### [should-fix] The guard's path patterns need a trailing slash, so common spellings of a local path pass
- **Where:** `tests/test_no_local_paths.py:25-26`.
- **Issue:** `FORBIDDEN` requires a slash after the user or folder name: the home-folder pattern is `/home/[^/\s]+/|/Users/[^/\s]+/`, and the `~/projects/` entry also ends in a slash.
- **Why it matters:** These forms are not caught, though they leak the same information:
  - a home path ending the token (a command such as `cd` followed by a home folder, or a quoted path)
  - a macOS `/Users/<name>` path ending the token
  - the Windows form `C:\Users\<name>\...`
  - `~/projects` with nothing after it
- **Evidence:** I ran `_mentions` on lines of each kind. Only the forms that end in a slash were reported; the others returned nothing. Loosening the patterns to drop the final slash (a name made of word characters, dots and hyphens after `/home/` or `/Users/`) and running `git grep -E` over the tree gives 0 hits outside this test, so the fix costs no false positives today.
- **Suggested fix:** Drop the required trailing slash, add a `C:\\Users\\` alternative, and add each new spelling to `test_the_guard_sees_each_kind_of_mention`.

#### [should-fix] The guard's self-test checks line numbers only, and the exception and file scan are untested
- **Where:** `tests/test_no_local_paths.py:150-160` (`test_the_guard_sees_each_kind_of_mention`), and `ALLOWED` at `:46-48`.
- **Issue:** The test asserts that lines 1 to 10 are reported, but not which kind each line is. The label comes from `WHAT[int(m.lastgroup[1:])]`, which couples the regex group index to dict order. Reordering `FORBIDDEN` would not break the test, and the label of a hit is what a failure message prints. The `ALLOWED` exception (the Eigen address passes in one file only, and fails anywhere else) and the binary-file skip have no test either.
- **Why it matters:** The guard is a privacy check ahead of going public. Its mislabelling or exception logic can break silently.
- **Evidence:** I spot-checked labels by hand and they are correct today; the gap is that nothing pins them.
- **Suggested fix:** Assert `(line, what)` pairs in the self-test. Add a small test that an allowed `(path, hit)` pair is exempt only for its own path, perhaps by factoring the per-file scan out of the main test.

#### [note] "Held as digests" hides the owner's name from grep, not from a determined reader
- **Where:** `tests/test_no_local_paths.py:9-10` and `:36-40`.
- **Issue:** Each word of a name is hashed alone with unsalted SHA-256. A single common word can be recovered by hashing a list of first names and surnames.
- **Why it matters:** The guard keeps the name out of the tree and works as a guard. Treat it as a tripwire, not a redaction, because the repository is going public. The docstring's "is not written here" is accurate and not misleading.
- **Suggested fix:** None needed. If the aim is ever secrecy, a guard that stores digests cannot give it.

#### [note] Licence metadata for the Rust crate and the `slam-it-rs` wheel does not mention the MPL file
- **Where:** `rust/Cargo.toml:19`, `rust/slam-it-py/pyproject.toml`; compare `pyproject.toml:12-13`.
- **Issue:** The main wheel's metadata now carries `LICENSE`, `THIRD_PARTY_NOTICES.md` and all six `LICENSES/*` texts. I confirmed this by generating the metadata with scikit-build-core outside the repository. The separate `slam-it-rs` wheel, which compiles the MPL-licensed `jacobi_svd3.rs`, sets no `license-files`, and the workspace `license` is PolyForm only. The CHANGELOG says "the wheel carries both".
- **Why it matters:** The crate and the wheel are unpublished (`publish = false`), so it is harmless today. It will matter if `slam-it-rs` is ever distributed.
- **Suggested fix:** None for this PR. Record it for whenever the wheel is distributed.

#### [note] Small leftovers in text moved or written in this PR
- **Where:** `rust/slam-it-foundations/src/linalg/jacobi_svd3.rs:305-307`, `THIRD_PARTY_NOTICES.md:9,17,36,45`, `THIRD_PARTY_NOTICES.md:25`.
- **Issue:**
  - `singular_values3`'s doc keeps a garbled hard wrap from the original: "(which every rank test, / written so NaN fails it, then / rejects)". It was kept because the move is byte-identical. I diffed the old block against the new file and it is unchanged, which is the right call for a move.
  - Four notice headings use em dashes; the repository's other prose (README, CHANGELOG, CLAUDE.md, design docs) uses none.
  - The notices name the C++ orientation table `UMAX`, but the C++ constant is `kUmax`.
- **Suggested fix:** Reflow the doc sentence in a later commit. The other two are optional.

### Checked and fine
- **`jacobi_svd3.rs`:** the 280-line block moved from `linalg.rs` is byte-identical (diffed). The MPL notice follows Exhibit A, and the Eigen copyright lines match the Eigen headers installed in the conda env verbatim. The module doc says what the file is and why it is separate.
- **Callers of the move:** all `use` paths changed (`alignment.rs`, `epipolar.rs`, `fivepoint.rs`, `homography.rs`, `so3.rs`). The intra-doc links in `linalg.rs` resolve. The C++ test and fixture descriptions were updated. The remaining old name, in `docs/design/2026-10-02-slam-it-ladder.md:1291`, sits in a record of what PR 9 did, so keeping it is right. Tests for the SVD stay in `linalg.rs` and are unchanged.
- **Notices against the tree:** the `LICENSES/` file names, the paths in the notices, the three ORB pattern copies each carrying the BSD notice, the 11 Gyroflow profiles, the test and fixture files named, and the Eigen copyright union (Jacob 2009-2010; Guennebaud 2009, 2013-2016) all check out. `THIRD_PARTY_NOTICES.md` is the new name everywhere it is cited in live files.
- **Packaging and version:** the `license-files` globs resolve, and the generated wheel metadata lists all eight files. The `PolyForm-Noncommercial-1.0.0` expression is set in `pyproject.toml`, the Rust workspace and the server. `LICENSE` is the PolyForm text. VERSION is 0.3.11.2 with no stale version strings, and the CHANGELOG entry's counts (22 PRs, 531 commit IDs, 6 licence texts, 3 guard tests) match.
- **`docs/history`:**
  - `commit-map.txt` has 531 unique, sorted, 40-hex pairs. All 531 new IDs exist in this history and none of the old IDs do.
  - Every commit ID quoted in the CHANGELOG, notes, reviews and design docs resolves through the map or this history, with 6 exceptions: 4 short IDs in `docs/reviews/2026-10-11-feature-your-own-video.md` and 2 in `docs/reviews/2026-10-11-feature-your-own-video-security.md`. One of them is the Gyroflow database commit and the rest are non-commit hashes, so I did not chase them.
  - Tag annotations ("slam-it pull request N, first repository") and merge-commit subjects match the intro. All 22 PRs have the same header shape.
- **Guard test:** it passes in 1.3 s with three tests. Its docstrings are numpy-style, and the `ALLOWED` exception is scoped to a (path, match) pair, not a whole file. Labels were correct in my probe.
- **Panel files, README and CLAUDE.md:** the path rule is added consistently to the three agents, the skill, the finding format and the report template. The README licence section links resolve. The CLAUDE.md change is a reflow only.
- **`git diff --check`:** clean, apart from whitespace in the verbatim ARKitScenes licence text.

## After the decisions -- 2026-10-18

The owner decided every finding as recommended. The fixes were folded into the commits they
belong to; layer 1 runs again on the final head, and offline CI before the merge.

The guard's stricter patterns (H4) flag this report's own verbatim quote of one of them, in H4's
text above. Reviews are kept verbatim, so the guard allows that one mention in this file alone,
as it allows Eigen's notice in the Eigen-derived file.
