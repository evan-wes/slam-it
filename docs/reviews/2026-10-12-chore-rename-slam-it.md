# Review panel: `chore/rename-slam-it`

<!-- Written by /slam-it-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-12

| | |
|---|---|
| **Branch** | `chore/rename-slam-it` |
| **Range** | `e428943...6696616` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | general-purpose with the agent files (`.claude/agents/slam-it-*-reviewer.md`): reifyscene's symlinks name the old files until the post-merge step |
| **Layer 1** | all checks passed (`scripts/check.sh` in the new `slam-it` env, exit 0; pytest 3572 passed, ctest 36, node --test 49) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | should-fix | hygiene | `CHANGELOG.md:9` | The entry's first sentence puts "!," and "!:" back to back, the shape the last rename's H7 fixed | fix ("The project is renamed from SLAM It All! to SLAM It!, which is ...") | fixed (478a885) -- decided by evan-wes, 2026-10-12 |
| H2 | should-fix | hygiene | `CHANGELOG.md:10-12` | "Only the formats' names differ" is too broad: the environment variables, modules, crates, images and the demo's folder change too | fix (narrowed: nothing computes differently, every name differs; a deployment sets the new `SLAM_IT_*` variables; the old demo folder is not read; with L1's byte comparison cited) | fixed (478a885: narrowed, with L1's comparison cited) -- decided by evan-wes, 2026-10-12 |
| H3 | note | hygiene | `README.md:6` | The README's pun explains the old name ("damn it all!") | fix (the owner's wording) | fixed (478a885: the pun dropped; "A codebase for a SLAM pipeline in three languages", the owner's wording) -- decided by evan-wes, 2026-10-12 |
| H4 | note | hygiene | `CLAUDE.md:179-186`; the PR template; `scripts/offline_ci.sh` | Out-of-repo state (hook, global CLAUDE.md, symlinks, envs, caches) keeps the old names until the post-merge steps, which must run in order | defer (the post-merge steps, their order in the PR body) | deferred (the post-merge steps, in order: the repo, then the hook and the global CLAUDE.md) -- decided by evan-wes, 2026-10-12 |
| H5 | note | hygiene | e.g. `rust/slam-it-edge/Cargo.toml:4` | About 44 "SLAM It!'s" possessives remain, as the last rename decided | no action | waived: the possessives stay, as the last rename decided -- decided by evan-wes, 2026-10-12 |
| A1 | note | architecture | `../bundles/edge-note-2026-10-10/` (outside the repo) | The ten edge-note bundles stop opening at 0.3.7.0; renaming their format id is not in the post-merge list | fix (a post-merge step: `rename_bundle_format.py --write`) | deferred (a post-merge step: `rename_bundle_format.py --write` on the ten bundles) -- decided by evan-wes, 2026-10-12 |
| A2 | note | architecture | `scripts/check.sh:69-74`; local state | The folder move breaks the plain CMake build trees; removing the old envs and caches is not in the plan | fix (in the folder-move step, delete `build/native`, `build/bench`, `build/slam-it-all`; the old envs and caches removed, each its own step) | deferred (the post-merge steps: the build trees deleted at the folder move; the old envs and caches removed, each its own step) -- decided by evan-wes, 2026-10-12 |
| A3 | note | architecture | `docker/`; `worker.py:131` vs `config.py:117` | No test reads the Docker files' names, nor holds the worker's token variable to the server's | defer (the ladder's open items) | deferred (the ladder's open items, 478a885) -- decided by evan-wes, 2026-10-12 |
| A4 | note | architecture | `CLAUDE.md:51-54` | The naming rule leaves out environment variables, which take `SLAM_IT_` | fix | fixed (478a885) -- decided by evan-wes, 2026-10-12 |
| A5 | note | architecture | ladder `:576-579`, `:750-754` | reifyscene's pin can only use tags from v0.3.7.0 on; the note on the name does not record that `slam-it` was checked free | fix (a clause in Packaging; the check recorded: free on PyPI and crates.io, 2026-10-12) | fixed (478a885) -- decided by evan-wes, 2026-10-12 |
| L1 | note | algorithm | `CHANGELOG.md:9-12`; the PR body | "Nothing behaves differently" deserves more than the golden record; whole runs on TUM and EuRoC, before and after, are byte for byte the same in all three backends but for `run.json`'s format | fix (cite the comparison: 9 of 9 runs, 36 of 36 feature and lifetime files) | fixed (478a885: the comparison cited, rechecked by the main session: the 9 runs identical file for file but for the per-frame timings, the measured peak memory and `run.json`'s format name, which the review's script drops) -- decided by evan-wes, 2026-10-12 |
| L2 | note | algorithm | `vo.py:735-737`; `slam_it_vo.cpp:56-58`; `main.rs:40-44` | Pre-existing: Python's program prints the final map's counts, C++'s and Rust's every keyframe and point made | defer (the ladder's open items) | deferred (the ladder's open items, 478a885) -- decided by evan-wes, 2026-10-12 |
| L3 | note | algorithm | the `slam-it` env's extensions | The extensions were built before the formatter pass; offline CI's clean build closes the gap | no action (offline CI before the merge Yes) | waived: offline CI builds every extension from a clean checkout before the merge Yes -- decided by evan-wes, 2026-10-12 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `chore/rename-slam-it`, `e428943...6696616` (498 files; 214 moves, 277 in-place edits). Method, as in the 2026-10-08 precedent: I reverse-mapped HEAD to the old spellings (`SlamItError`, `SLAM_IT_`, `slam_it`, `slam-it`, `SLAM It!` back to the `...All` forms) and diffed that against BASE for every changed file. That includes the 7 files git reports as add/delete pairs, which I compared to their old paths by hand. The residue is the set of non-mechanical edits. I also scanned living files for leftover old names, mixed forms, and prose around the "!". Nothing was run beyond git and grep, so Layer 1 is taken from your brief.

**Summary:** The rename is clean. A case-insensitive scan of `slam_it_all`, `slam-it-all`, `SLAM_IT_ALL`, `SlamItAll` and "It All" over every living file (excluding `docs/reviews/20*` and `CHANGELOG.md`) finds only the intended history (README pun, ladder 752, `your-own-video.md:9`, and `correspondences.py:142`, which are "reads it all" and "allows"). I found no mixed forms by kind, no `slamlab` outside intended history, no stray old paths, and no rewrap damage. The only non-rename residue is formatter output in code (clang-format, rustfmt, ruff) plus the new CHANGELOG entry and the ladder's rename paragraph. The findings are all wording: the CHANGELOG lead sentence reads badly, its "nothing behaves differently" is too broad, and the README pun no longer explains the new name.

### Findings

#### [should-fix] CHANGELOG lead sentence has the awkward "!" punctuation the last rename's H7 flagged
- **Where:** `CHANGELOG.md:9`
- **Issue:** "The project is renamed SLAM It!, from SLAM It All!: shorter, easier to remember..." puts "!," and "!:" back to back. The "!" reads as a sentence end, so the clause after the colon hangs loose. The reversed order (new name first, then "from old") is also clumsy.
- **Why it matters:** It is the first line of the entry people read for this release. The precedent (H7) fixed the same shape in `CLAUDE.md` and the ladder title.
- **Evidence:** `The project is renamed SLAM It!, from SLAM It All!: shorter, easier to remember, and what the`
- **Suggested fix:** "SLAM It All! is renamed SLAM It! -- shorter, easier to remember, and what the demo does to a video." Or "The project is renamed from SLAM It All! to SLAM It!, which is shorter, easier to remember, and what the demo does to a video."

#### [should-fix] CHANGELOG "Nothing behaves differently ... only the formats' names differ" is too broad
- **Where:** `CHANGELOG.md:10-12`, against the `### Changed` list at `:17-34`
- **Issue:** The same entry says environment variables change from `SLAM_IT_ALL_*` to `SLAM_IT_*`, the demo's default folder moves from `~/.local/share/slam-it-all/demo` to `~/.local/share/slam-it/demo`, the module paths, Docker image and entrypoint names change, and the lock file is renamed. So "only the formats' names differ" is false.
- **Why it matters:**
  - A deployment that sets `SLAM_IT_ALL_INVITE_CODES`, `SLAM_IT_ALL_WORKER_TOKEN`, `SLAM_IT_ALL_SIGNING_KEY`, `SLAM_IT_ALL_DATABASE` or `SLAM_IT_ALL_VIEWER` fails at startup, because those are `_required` in `server/src/slam_it_server/config.py`.
  - An existing local demo folder is silently orphaned.
  - Precedent H3 flagged the same kind of overclaim ("to the byte").
- **Suggested fix:** Narrow it, for example: "The code takes the same decisions and the room's runs still match their golden record; what differs is every name: the formats' ids, the environment variables, the module and crate names, and the demo's default folder. A deployment must set the new `SLAM_IT_*` variables, and data in the old folder is not read."

#### [note] README pun explains the old name, not the new one
- **Where:** `README.md:6` (and `docs/design/2026-10-02-slam-it-ladder.md:752`, which keeps "damn it all!" as the old name's pun, correctly)
- **Issue:** `*A pun on "damn it all!": slam_it in code, slam-it for the repository.*` still derives the name from "damn it all!". "SLAM It!" is no longer that phrase. The CHANGELOG gives a different reason ("what the demo does to a video").
- **Why it matters:** The README's explanation of the name now matches neither the title above it nor the CHANGELOG. This one is the owner's call, since it is wording about the name's meaning.
- **Suggested fix:** Something like "*A pun on "slam it!" (or "damn it!"): ...*", or drop the pun and keep only the `slam_it` / `slam-it` half.

#### [note] Out-of-repo state still names the old forms, and the first offline-CI run starts cold
- **Where:** `CLAUDE.md:179-186`, `.github/pull_request_template.md` (the `~/.cache/slam-it-offline-ci/results.tsv` line), `scripts/offline_ci.sh`
- **Issue:** The docs now name the new repo (`evan-wes/slam-it`), cache directory and folder paths. The hook (`ALLOWED_REPO="evan-wes/slam-it-all"`), global CLAUDE.md, symlinks, conda envs and the old `~/.cache/slam-it-all-offline-ci` still carry the old names until the planned post-merge steps.
- **Why it matters:** This is intended (H12 precedent). Until the repo, hook and global CLAUDE.md are renamed, the documented `gh pr merge <n> --repo evan-wes/slam-it` is blocked or points at a repo that does not exist yet. The order matters: rename the GitHub repo first, then the hook and global CLAUDE.md, before the next merge.
- **Suggested fix:** None in this PR. Keep the sequence in the PR body.

#### [note] About 44 "SLAM It!'s" possessives remain
- **Where:** e.g. `rust/slam-it-edge/Cargo.toml:4` ("SLAM It!'s edge rung in Rust"), `server/src/slam_it_server/__init__.py:1`
- **Issue:** The "!'s" form stays, as decided for the last rename (H7). The count is the same order as before.
- **Why it matters:** Nothing new. Noted only so the deliberate carry-over is visible.

### Checked and fine
- **Leftover old names:** a case-insensitive scan for `slam[_ -]?it[_ -]?all`, `SlamItAll` and "It All" found none in living files, including `.github/`, `docker/`, `viewer/`, `scripts/`, `server/`, `rust/`, `cpp/`, `tests/` and the lock files. Nothing in tracked paths still has `all` in its name except the 2026-10-08 report, which is history.
- **Wrong or mixed forms:** I looked for `slam_it…-…`, `slam-it…_…`, `SLAM-IT`, `Slam-It`, `slamit` and `SlamIt` outside `SlamItError`. The only hit is `docker/worker.Dockerfile:40` (`slam_it-*.whl`), which is correct for a normalized wheel filename.
- **Naming rule:** `CLAUDE.md:51-54` states which form goes where and matches the code, the CHANGELOG and the diff. `slamlab` appears only in the ladder's rename history (`:750-751`).
- **Format ids:** `slam-it-run` and `slam-it-bundle` agree across Python, C++, Rust, the viewer JS, the fixtures and the viewer test. The CHANGELOG entry's list matches the diff: env vars, `SlamItError`, `scripts/setup_slam_it.sh`, the ladder filename, the lock, the Docker names, the viewer package and the subdomain. `slam-it.timedilated.com` is consistent in the ladder, run-viewer design and your-own-video note.
- **Rewraps and code spans:** the reverse-mapped residue holds only tool reformatting of code. That is C++ test calls, Rust `use` and `temp_dir` lines, `server/.../app.py:357`, `config.py:113-120` and `tests/test_check_layers.py:198`. Each is a legitimate result of the shorter names, and none touches a docstring, a code span or a numpydoc boundary. Markdown has no residual rewraps, so the BA-note and se3 damage from last time is not repeated.
- **Prose around the "!":** `CLAUDE.md:11-12` now says "inside the SLAM It! repository". `SKILL.md:3` is reworded, `report-template.md:3` is on one line, and the ladder and package titles use " -- ". `docs/reviews/README.md` points at the new skill path, and the README link and `CLAUDE.md` merge section name the new repo.
- **History:** the earlier CHANGELOG entries and `docs/reviews/20*` are unchanged. The ladder's rename paragraph now covers both renames accurately ("0.3.4.0, then 0.3.7.0"). VERSION is 0.3.7.0 and matches the new entry's heading and date.
- **Scaffold and CI:** `.github/workflows/test.yml` and `pull_request_template.md` have consistent env, script and agent names. The skill and agent files carry `slam-it-*` names throughout, and the hygiene agent file's front matter matches.

### Architecture review

## Architecture review

**Scope:** `chore/rename-slam-it`, e428943 (origin/develop)...6696616, one commit, 498 files. I read the whole diff for the build and layering files (`pyproject.toml`, `CMakeLists.txt`, `rust/Cargo.toml` and every crate manifest, `rust/slam-it-py/pyproject.toml`, `scripts/check_layers.py` and its test, `src/slam_it/backends.py`, the bindings' module roots and error translation), the server and worker settings, `docker/`, `.github/`, `scripts/{check,offline_ci,setup_slam_it,gen_env_lock}.sh`, `scripts/serve_demo.py`, the format constants and fixtures, the viewer, `CLAUDE.md`, the ladder and the panel files. To cover the rest I ran a script over every changed file (below). I also looked at the state outside the repo: `../bundles`, `../results`, the conda envs, `~/.cache`, and reifyscene's tracked files and `.claude/`.

**Summary:** The rename is mechanical, complete and structurally neutral. Every contract still holds under the new names in all three languages, and the layering checks see exactly the same graphs as at the base. Inside the repo, nothing is still addressed by an old name. There is no must-fix or should-fix. The notes are about consequences outside the repo that the post-merge plan does not yet list: the edge note's bundles, the folder move's build trees and leftover envs, the Docker files that no test reads, and the naming rule.

### Findings

#### [note] The edge note's ten bundles stop opening at 0.3.7.0, and the step that renames them is not in the post-merge plan
- **Where:** `../bundles/edge-note-2026-10-10/*/bundle.json` (outside the repo); `viewer/src/bundle.js:208-209`; `docs/notes/viewer.md:146-150`; `docs/notes/your-own-video.md:245`
- **Issue:** All 10 bundles (2.0 GB) still say `"format": "slam-it-all-bundle"`. The viewer at HEAD refuses them with "expected slam-it-bundle version 1". The viewer note cites these bundles, and the demo checklist's fallback is to "keep a baked bundle at hand (`scripts/view_bundle.py`) in case the network fails". A script that rewrites them already exists: `rename_bundle_format.py`, which dry-runs unless given `--write`, changes only the `format` key and refuses any other format. But the brief's post-merge list does not include running it.
- **Why it matters:** After the merge, the demo's offline fallback fails until someone remembers this step. Re-baking is not an alternative: the runs behind the bundles are old-format too.
- **Evidence:** `find ../bundles -name bundle.json -exec grep -l slam-it-all-bundle {} + | wc -l` gives 10 of 10. In `../results/`, 155 `run.json` files say `slam-it-all-run`, and 605 more say `slamlab-run`, left by the first rename and never migrated.
- **Suggested fix:** Add "run `rename_bundle_format.py <bundles> --write`" to the post-merge steps, after the merge: develop's viewer still expects the old id until then. The runs in `../results/` can stay as historical records, as they did after the first rename.

#### [note] The folder move breaks the plain-CMake build trees, and old envs and caches are not in the plan
- **Where:** `scripts/check.sh:69-74` (`cmake -S . -B build/native`, `build/bench`); local state
- **Issue:** CMake refuses a build tree whose source folder has moved. Layer 1's two configure steps, and everything after them (the C++ build, clang-tidy `-p build/native`, ctest), would fail after the move to the SLAM It! checkout. scikit-build-core clears its own tree (`build/slam-it`) when the source moves (`scikit_build_core/cmake.py:168`, "Original src {} != {}, clearing cache"), so only the plain trees are affected. Separately, the plan says "rebuild envs" but never mentions removing the old ones. Still on disk: the `slam-it-all` and `slam-it-all-ci` envs, `~/.cache/slam-it-all`, `~/.cache/slam-it-all-offline-ci`, `~/.cache/slamlab-offline-ci` (left by the first rename), and the git-ignored `build/slam-it-all`.
- **Why it matters:** The first layer-1 run after the move fails with an error that points at CMake, not at the move.
- **Evidence:** In the scratchpad I configured an empty `project(x NONE)` into `build/native`, moved the folder and configured again: "CMake Error: The current CMakeCache.txt directory .../new/build/native/CMakeCache.txt is different than the directory .../old/build/native where CMakeCache.txt was created", exit 1. `conda env list` shows `slam-it`, `slam-it-all` and `slam-it-all-ci`.
- **Suggested fix:** In the folder-move step, delete `build/native`, `build/bench` and `build/slam-it-all`; they are git-ignored and get rebuilt. Add removal of the old envs and caches to the plan, each as its own step with its own Yes.

#### [note] The Docker files are the one place names live that no test reads
- **Where:** `docker/worker.Dockerfile:38-50`, `docker/server.Dockerfile:13-27`, `docker/server-entrypoint.sh:9`; `src/slam_it/orchestration/worker.py:131` versus `server/src/slam_it_server/config.py:117`
- **Issue:** I checked every Docker name by hand, and all are right at HEAD:
  - the lock path;
  - the wheel glob `slam_it-*.whl`;
  - `python -m slam_it_server`, `slam_it.eval.lenses` and `slam_it.orchestration.worker`;
  - `SLAM_IT_VIEWER`, `SLAM_IT_LENS_PROFILES` and `SLAM_IT_DATABASE`.

  But no test reads `docker/`. Also, `tests/server/test_shared_names.py` does not check that `worker.TOKEN_VARIABLE` equals the server's `SLAM_IT_WORKER_TOKEN`. The demo maps one to the other by name (`scripts/serve_demo.py:284`), so a drift would pass every test, and only the deploy note's "`SLAM_IT_WORKER_TOKEN` as the server's" would go wrong.
- **Why it matters:** Two renames in four days, and the next setting rename, would surface only at the first deploy. That failure is loud (required settings are refused at startup), but it comes late.
- **Evidence:** `grep -rn -i 'docker\|entrypoint' tests/` finds nothing.
- **Suggested fix (backlog):** A small test that does three things:
  - checks each `SLAM_IT_*` name in `docker/` against a name the code reads;
  - imports each `python -m` module;
  - asserts that `worker.TOKEN_VARIABLE` is among the settings the server documents.

#### [note] The naming rule leaves out environment variables
- **Where:** `CLAUDE.md:51-54`
- **Issue:** The rule reads "macros and CMake options use `SLAM_IT_`", and says everything else uses `slam-it`. Environment variables take `SLAM_IT_` too, as the CHANGELOG entry says (`CHANGELOG.md:28-29`). They are the forms layer 1 checks least. The gap predates this PR (the first rename's H13 added the rule), but this PR edits the line.
- **Suggested fix:** Change it to "macros, CMake options and environment variables use `SLAM_IT_`".

#### [note] The reifyscene boundary: nothing to change now, one optional line to record
- **Where:** `docs/design/2026-10-02-slam-it-ladder.md:576-579` (Packaging), `:750-754` (the note on the name)
- **Issue:** reifyscene has no adapter, no `slam` extra and no tracked reference to any SLAM It name (checked with `git grep` in reifyscene). Its `.claude/` symlinks are absolute paths to the old agent and skill files. They dangle now that this branch is checked out, which is why this review ran as general-purpose, and re-pointing them is already in the plan. Two optional records:
  - The pin's earliest usable tag is v0.3.7.0. Tags v0.3.4.0 to v0.3.6.0 ship `slam_it_all`, and v0.3.2.0 to v0.3.3.0 ship `slamlab`.
  - The note on the name says the first rename was forced by collisions (SLAMLab, PyPI's `slamlab`), but does not say whether `slam-it` was checked the same way. I could not check PyPI offline.
- **Suggested fix:** Optionally, one clause in Packaging: "a direct git reference to a tag ≥ v0.3.7.0". That also means a PyPI package with the same name can never be installed in its place. Optionally, one clause in the note on the name recording the collision check.

### Checked and fine
- **Mechanical change:** I applied the five substitutions to each changed file at base and compared it, ignoring whitespace, with HEAD. Only four files differ beyond the substitutions: `CHANGELOG.md`, `VERSION`, the ladder's note on the name, and a rustfmt reflow in `rust/slam-it-edge/src/tracking.rs` (a dropped trailing comma). There is no structural change and no alias or shim. The 7 deleted and re-added files are renames too.
- **Residual names:** `git grep` for the old name forms in HEAD, outside `docs/reviews/20*` and earlier CHANGELOG entries, finds only the ladder's history line (`:752`) and `README.md:6`. The README line is the "damn it all!" pun line, which is prose and the hygiene reviewer's call.
- **import-linter** (`--no-cache`): 21 contracts kept. The server is held apart from `slam_it` in both directions, with its own internal order. Contracts match modules by dotted descendant, so `slam_it` does not cover `slam_it_server`.
- **`check_layers.py`:** exit 0. It sees the same graphs at base and HEAD:
  - C++: 44 modules and 236 include edges;
  - Rust: 49 modules and 136 edges;
  - the same 4 `slam_it_add_rung` calls and the same 5 crates.

  Its prefixes are safe under the shorter name: `slam_it.` with the dot, `slam-it-` crates with `slam-it-py` exempt, and `slam_it_([a-z_]+)` scanned only inside the DEPENDS lists.
- **Cargo:** the workspace members, path dependencies and the `[[bin]] slam_it_vo` are consistent. `Cargo.lock` changes names only. `backends` finds the Rust program through the manifests, not a hard-coded crate name.
- **CMake:** the project, options, `slam_it_add_rung`, the `slam_it_<rung>` targets and the install destinations `slam_it/` and `slam_it/bin` are consistent.
- **Extension modules:**
  - `_cpp` is installed into `slam_it/`;
  - the bindings crate's lib, its `#[pymodule] fn` and maturin's `slam-it-rs` all name `slam_it_rs`;
  - `backends._EXTENSIONS` matches;
  - both bindings translate errors through `slam_it.foundations.errors`.

  A runtime probe in the `slam-it` env found all three backends, found `slam_it_vo` for C++ and Rust, and confirmed that no `slam_it_all*` module is importable.
- **Targeted tests:** `test_check_layers`, `test_backends`, `server/test_shared_names` and `server/test_config`: 112 passed.
- **Environment variables:** every read matches its writers.
  - Reads: `config.py`, `lenses.py`, the worker's `TOKEN_VARIABLE` and `CHILD_ENVIRONMENT`, the `conftest` backends list, and `SLAM_IT_BAL` in the C++ and Rust benchmarks.
  - Writers: the `check.sh` export, `serve_demo.py`, the Dockerfile `ENV` lines, the entrypoint and the settings template.

  Settings still under old names fail loudly, because the server refuses to start without its required variables.
- **CI:** `test.yml` uses the renamed setup script and env, and the tag workflows name no repo. `offline_ci.sh` has its own cache, checkout and env; env lookups match exact names (`grep -qx`), and it fetches from `REPO` on every run, so moving the folder does not break its persistent checkout.
- **Format ids:** Python, C++ `kFormat`, Rust `FORMAT`, the viewer's `FORMAT` and both fixtures agree. Tests hold them equal: the native run writers are compared with `run.json`, and `bundle.test.js` reads the bundle Python baked. The versions are unchanged (3 and 1), and `room_runs.json` is untouched.
- **Merge order:** the repo's `CLAUDE.md` at HEAD names `--repo evan-wes/slam-it`, while the hook still allows only `evan-wes/slam-it-all`. The planned order (merge, then the GitHub rename, then the hook) handles this. This PR's own merge uses the old form.

### Algorithm review

## Algorithm review

**Scope:** `chore/rename-slam-it`, `e428943caece6c2f431a39de5afc2a0922a53bf5...66966167ea74a4149c3b325bf707ce5d1c56697b` (one commit, 498 files). I focused on `src/slam_it/`, `cpp/` and `rust/` (323 file pairs) and also checked every other file in the tree for leftover changes. I also ran tests and before/after runs, described below.

**Summary:** The rename changes no numeric code. I put the old names back into BASE, substituted the new ones, and compared every touched file with HEAD. What is left is only the version bump, the CHANGELOG, the ladder's rename-history paragraph, and formatter line joins. No constant, operator, expression, literal, seed or RNG stream changed. On the dynamic side, the golden-record test passes 24/24 at HEAD. Whole runs on real TUM and EuRoC data are byte-identical before and after the rename in all three backends; the only difference is the `format` name in `run.json`. I have no must-fix or should-fix findings.

### Findings

#### [note] The PR's "nothing behaves differently" claim deserves stronger evidence than the golden record, and that evidence now exists
- **Where:** `CHANGELOG.md:9-12` (0.3.7.0 entry) and the PR's "How this was built" section.
- **Issue:** The claim rests on `test_every_backend_matches_the_golden_record`. That test compares poses with `"comparison": "close", "rtol": 1e-09, "atol": 1e-09` and summaries at `SOLVER_TOLERANCE`, on the 40-frame synthetic room only. Decisions are compared exactly.
- **Why it matters:** A change below 1e-9 (for example a lost compile flag), or one that shows only on real images, would still pass. So the record alone cannot prove "nothing". The claim is true, but the PR should cite the check that proves it. The previous rename PR (2026-10-08) did this with `byte_compare_runs.py`.
- **Evidence:** I ran the old and new programs on the same inputs and compared the outputs byte for byte with `python3 -I byte_compare_runs.py BEFORE AFTER`, plus `cmp` on the feature and lifetime files.
  - **Old programs used:**
    - C++: `build/slam-it-all/install/platlib/slam_it_all/bin/slam_it_all_vo`, built 2026-10-10 22:46. `git log --since='2026-10-10 22:40' e428943 -- cpp CMakeLists.txt` is empty, so it matches BASE's sources.
    - Rust: `rust/target/release/slam_it_all_vo`, built 2026-10-11 19:31. The only later Rust commit, cc4491f, changed test asserts only.
    - Python: BASE's `src/slam_it_all`, extracted with `git archive` and run in the `slam-it-all` env with `python -S`, so the old editable-install hook was bypassed.
  - **Cases, each run in C++, Rust and Python:**
    - `--dataset tum --sequence fr1_desk --mode rgbd --count 150`
    - `--mode monocular --count 150` on the same sequence
    - `--dataset euroc --sequence MH_01_easy --mode stereo --first 200 --count 200`
    - all with `--record-features`
  - **Result:** `9 of 9 folders the same`, with only `format: slam-it-all-run -> slam-it-run` differing. `feature_pixels.npy`, `feature_records.npy`, `keyframe_lives.npy` and `point_lives.npy` are identical in all 9 runs (36 of 36 files).
- **Suggested fix:** Cite this byte-for-byte comparison in the PR body next to the golden record (record the cases and the result). No code change.

#### [note] The Python program's summary line counts something different from the C++ and Rust programs (pre-existing, outside this PR's goal: backlog)
- **Where:**
  - `src/slam_it/orchestration/vo.py:735-737` prints `len(done.final_keyframes()[0])` and `len(done.final_points())`.
  - `cpp/apps/orchestration/slam_it_vo.cpp:56-58` and `rust/slam-it-orchestration/src/main.rs:40-44` print `keyframe_timestamps.size()` and `points.size()`, which are every keyframe and point the run ever made.
- **Issue:** For the same run, the three programs print different keyframe and point counts. The runs themselves agree.
- **Why it matters:** Anyone comparing backends by their stdout will think they diverge. That is a trap for the "same algorithm, three languages" story.
- **Evidence:** TUM fr1_desk rgbd, 150 frames:
  - Python prints `13 keyframes, 2202 points`; C++ and Rust print `13 keyframes, 8999 points`.
  - Monocular: Python prints `28 keyframes, 2074 points`; C++ and Rust print `32 keyframes, 7454 points`.
  - The files agree in all three backends: `points.npy` has (8999, 3) rows with 2202 still alive, and (7454, 3) with 2074 alive. Trajectories match: Python vs C++ max |Δt| = 3.3e-15 (rgbd) and 3.3e-14 (mono).
  - The same mismatch is in BASE's logs, so this PR did not introduce it.
- **Suggested fix:** Backlog. Make all three programs print the same counts, either all made or all final, or print both.

#### [note] The PyO3 extension in the `slam-it` env was built before the formatter pass; the clean-build check is offline CI
- **Where:**
  - `~/miniconda3/envs/slam-it/.../slam_it_rs/*.so` (built 14:13:53).
  - `build/slam-it/.../_cpp*.so` (built 14:13:41).
  - HEAD was committed 14:33.
- **Issue:** Files newer than the extensions are exactly the files the formatters touched:
  - five Rust files;
  - nine C++ test files.

  Only one of them is in the extension's crate graph: `rust/slam-it-edge/src/tracking.rs`. Its whole change is rustfmt joining a `use slam_it_foundations::matching::{...}` list onto one line.

  So the tested extension is equivalent to HEAD. Memory notes that `check.sh` does not rebuild `slam_it_rs`. The Rust program, `rust/target/release/slam_it_vo` (built 14:26), is newer than its sources.
- **Why it matters:** This is the one place where "layer 1 at HEAD" did not literally test binaries built from HEAD.
- **Evidence:** `find rust ... -newer slam_it_rs.so` and the residual diff below.
- **Suggested fix:** None needed in code. `scripts/offline_ci.sh` (clean checkout, fresh build) before the merge Yes closes the gap, as is already the practice.

### Checked and fine
- **Name-only diff in the algorithm code:** 323 pairs under `src/`, `cpp/` and `rust/`:
  - 309 are identical after substitution;
  - 13 are whitespace-only clang-format or rustfmt joins, each inspected line by line (C++ test files and Rust test or helper code only, no string contents changed);
  - 1 is a rustfmt `use` join (`tracking.rs:7`).

  `src/slam_it/` has no residual at all. Over the whole tree, the only other residuals are `VERSION`, `CHANGELOG.md`, `docs/design/...-ladder.md:750-754`, and ruff joins in `server/src/slam_it_server/{app,config}.py` and `tests/test_check_layers.py`.
- **No numeric change anywhere:** no constant, operator, expression, literal or tolerance changed in the cores, the bindings or the Python reference.
- **RNG:**
  - `Pcg32(seed, stream)` is seeded only from integers in all three languages: `ransac.hpp:281`, `ransac.rs:500`, `ransac.py:354`.
  - The stream is `planar ? 1 : 0` (`twoview.cpp:45`, `twoview.py:222`).
  - No string is hashed into a seed, and nothing derives a seed from `__name__` or `__module__` (searched for crc32, hashlib, `hash(`, SeedSequence, `default_rng(` and `DefaultHasher`).
- **Use of `__name__`:** it is used only for backend dispatch. `backends.implementation` uses `_PACKAGE_PREFIX = "slam_it."` with `startswith` and `removeprefix`, with no hard-coded prefix length.
- **Format IDs:** `slam-it-run` (`runs.py:65`, `runs.hpp:24`, `runs.rs:15`) and `slam-it-bundle` are only written into, and checked in, the `format` field. They are not part of any hash or key. `tests/fixtures/edge/room_runs.json` holds no project name and is unchanged.
- **Golden record:** I ran `tests/orchestration/test_vo.py::test_every_backend_matches_the_golden_record` in the `slam-it` env: `24 passed in 54.46s` (room and gap × 4 dataset/mode cases × python/cpp/rust).
- **Floating-point build flags:** both still applied to every rung through `slam_it_add_rung`:
  - `-ffp-contract=off` (`CMakeLists.txt:33`)
  - `EIGEN_NO_CPUID` (`CMakeLists.txt:38`)

  The CMake options `SLAM_IT_BUILD_TESTS` and `SLAM_IT_BUILD_BENCHMARKS` are defined and used consistently.
- **No leftover old names:** `git grep -i 'slam[_ -]?it[_ -]?all|SlamItAll'` at HEAD finds them only in history (the CHANGELOG, ladder line 752, `docs/reviews/`). No stale variable or option name can silently fall back to a default.
- **Compiled backends cannot skip silently:** `SLAM_IT_REQUIRE_BACKENDS` is exported in `scripts/check.sh:36` and read in `tests/conftest.py:22`.
- **Rust stale-program guard:** `backends._program_inputs(rust, "slam_it_vo")` returns 55 files across `slam-it-{edge,eval,foundations,orchestration}`. The old name returns `[]`, so the guard follows the renamed crates and was not silently disabled.
- **Toolchain parity between the old and new envs:** gcc and gxx 15.3.0, clang 23.1.2, rust 1.99.0, eigen 5.0.1, numpy 2.4.6 and openblas 0.3.34 are the same in both. The only differences are binutils and ld build numbers (102→103), the libabseil build and libsqlite (3.53.4→3.54.0).
- **Lens intrinsics input:** `PROFILES_COMMIT` is unchanged, and `~/.cache/slam-it/lens_profiles` is identical to the old cache (`diff -rq`, 1,164 files).
- **Budget harness:** it reads `/proc/self/{status,cgroup}`, so nothing depends on the program's name. Both names also fit the 15-character `comm` limit.
- **Docstrings of algorithms:** only qualified names changed (e.g. `slam_it.foundations.nls.Summary`). No description of an algorithm changed meaning; `p3p.py:52` and `metrics.py:13` read as before.
