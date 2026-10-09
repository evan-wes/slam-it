# Review panel: `chore/rename-slam-it-all`

<!-- Written by /slam-it-all-review-panel. One file per branch; later runs append "Run N". -->

## Run 1 -- 2026-10-08

| | |
|---|---|
| **Branch** | `chore/rename-slam-it-all` |
| **Range** | `06c90b3...2ccea36` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet) alone, as the owner chose: the rename changes no behaviour |
| **How they ran** | general-purpose with the agent file (`.claude/agents/slam-it-all-hygiene-reviewer.md`): reifyscene's symlinks still name the old files until the post-merge step |
| **Layer 1** | all checks passed (`scripts/check.sh --env slam-it-all`, exit 0, 20 stages; pytest 3180 passed) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `src/slam_it_all/foundations/se3.py:477-479, 504-506` | The rewrap merged the `backend` parameter into `xi`'s description in two public docstrings | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H2 | should-fix | hygiene | `pyproject.toml:9` | The distribution's description still says "(working name)" | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H3 | should-fix | hygiene | `CHANGELOG.md:10-12` | "Writes the same results, to the byte" is too broad: `run.json`'s `format` changes | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H4 | should-fix | hygiene | `docs/notes/bundle-adjustment.md:330-331` | The rewrap splits the inline code span `` `P = R X + t` `` across lines | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H5 | should-fix | hygiene | `docs/design/2026-10-02-slam-it-all-ladder.md:519-537` | `ruff format .` reformatted the `SlamInput`/`SlamResult` sketch (its aligned comments, a doubled blank line) | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H6 | should-fix | hygiene | `.claude/skills/slam-it-all-review-panel/report-template.md:3-4` | The template's HTML comment now ends with an orphan `-->` line | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H7 | should-fix | hygiene | `SKILL.md:3; CLAUDE.md:12; ladder:1; src/slam_it_all/__init__.py:1; ransac.py:487` | Prose where "SLAM It All!" reads awkwardly: a double possessive, a "!" read as a full stop, the title's lost colon, an exception named in prose | fix | fixed (4f064f4): the four places; the "SLAM It All!'s" possessives stay -- decided by evan-wes, 2026-10-08 |
| H8 | note | hygiene | `ladder:371, 539, 724; absolute-pose.md:111` | Module and distribution names bare in prose | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H9 | note | hygiene | `scripts/check.sh:6` | `/review-panel` was never the skill's name (pre-existing) | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H10 | note | hygiene | `docs/reviews/README.md:3` | The folder's living guide names the old skill path | fix | fixed (4f064f4): the guide is living, unlike the reports -- decided by evan-wes, 2026-10-08 |
| H11 | note | hygiene | the PR body; `docs/notes/bundle-adjustment.md:386` | Edits beyond the rename belong in the PR body; the BA note's table header should match the script's new `ours` column | fix | fixed (4f064f4: the BA note's header; the PR body lists the other edits) -- decided by evan-wes, 2026-10-08 |
| H12 | note | hygiene | `CLAUDE.md:161-169`; the owner's local setup | The hook, global CLAUDE.md, reifyscene's symlinks, envs and caches still say slamlab | defer (the post-merge steps) | deferred (the post-merge steps: the GitHub repo, the hook and global CLAUDE.md, the local folders, envs, symlinks and caches) -- decided by evan-wes, 2026-10-08 |
| H13 | note | hygiene | `CLAUDE.md` | The naming rule (which form goes where) is missing from the conventions every reviewer reads | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |
| H14 | note | hygiene | `docs/design/2026-10-02-slam-it-all-ladder.md:35` | The relative link to reifyscene needs four `..`, not three (pre-existing) | fix | fixed (4f064f4) -- decided by evan-wes, 2026-10-08 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `chore/rename-slam-it-all`, `06c90b39741f390c4e61a3c872ff07953e84c1d0...2ccea36d6153a3d6666da27b5757e4c0d524742e` (445 files: 197 renames, 234 modifications, 7 add/delete pairs). Method: I reverse-mapped HEAD to the old names (`slam_it_all`, `slam-it-all`, `SLAM It All!`, `SLAM_IT_ALL`, `SlamItAllError` back to the base spellings) and diffed that against BASE for every changed file. What is left is the complete set of non-mechanical edits: rewraps, reformats and wording changes. I read all of them. I then ran separate scans for the spelling form used in each context, for broken paths and links, and for prose and punctuation around the display name.

**Summary:** The rename is clean and consistent. `git grep -i slamlab` leaves only the intended places (ladder lines 707-708, the CHANGELOG, `docs/reviews/`), and I found no mixing of `slam_it_all`, `slam-it-all` and `SLAM_IT_ALL` by kind. One rewrap broke two public docstrings (must-fix). One stale "working name" marker was missed. The remaining findings are rewrap and prose polish, plus an overbroad CHANGELOG claim.

#### Findings

##### [must-fix] Rewrap corrupted the `backend` parameter docs in two public functions
- **Where:** `src/slam_it_all/foundations/se3.py:477-479` (`right_jacobian_inverse`) and `:504-506` (`left_jacobian_inverse`)
- **Issue:** The longer name pushed the `xi` description past 100 columns. It was rewrapped by joining lines across a numpydoc boundary. The `backend` parameter entry is now glued onto the end of the `xi` description, and the `:data:` line has lost its indentation.
- **Why it matters:** numpydoc and Sphinx no longer see a `backend` parameter on either function, and the `xi` entry renders garbled. Ruff's docstring checks do not catch it, which is why layer 1 passed. `:data:` is a Sphinx role, so the broken lines render wrongly in generated docs.
- **Evidence:**
  ```
  477:        Twists whose rotation angle is below
  478:    :data:`slam_it_all.foundations.so3.INVERSE_JACOBIAN_LIMIT`. backend : {"python", "cpp", "rust"},
  479:        optional Which implementation computes the result (default: the Python reference).
  ```
  BASE had `backend : {"python", "cpp", "rust"}, optional` on its own line, with its description on the next line.
- **Suggested fix:** In both places restore:
  ```
      xi : array_like, shape (..., 6)
          Twists whose rotation angle is below
          :data:`slam_it_all.foundations.so3.INVERSE_JACOBIAN_LIMIT`.
      backend : {"python", "cpp", "rust"}, optional
          Which implementation computes the result (default: the Python reference).
  ```
  The residual diff shows no other docstring damaged this way.

##### [should-fix] Stale "(working name)" in the package metadata
- **Where:** `pyproject.toml:9`
- **Issue:** `description = "A learning ladder for visual SLAM, in Python, C++ and Rust (working name)."` was missed. The README and CLAUDE.md "working name" markers were removed, and this one is the only leftover.
- **Why it matters:** It ends up in the distribution metadata of `slam-it-all`, and it contradicts the rename.
- **Evidence:** `git grep -n -i 'working name' -- . ':!docs/reviews' ':!CHANGELOG.md'` returns this line plus the intended ladder line 707.
- **Suggested fix:** Drop " (working name)".

##### [should-fix] CHANGELOG claims byte-identical results, but `run.json` changes
- **Where:** `CHANGELOG.md:10-12` against `CHANGELOG.md:30-31`
- **Issue:** The lead paragraph says every run "writes the same results, to the byte, as before the rename". The same entry says the run format is now `slam-it-all-run` (`src/slam_it_all/eval/runs.py:49`, `cpp/include/slam_it_all/eval/runs.hpp:21`, `rust/slam-it-all-eval/src/runs.rs:13`). So `run.json` differs in its `format` field, and `diagnostics.jsonl` carries wall-clock step timings.
- **Why it matters:** The claim is slightly false as written, and a reader comparing runs across the rename will find a difference.
- **Suggested fix:** Narrow it, for example: "takes the same decisions and writes the same trajectories, keyframes and points, to the byte; only `run.json`'s `format` name differs."

##### [should-fix] Inline code span split across a line in the BA note
- **Where:** `docs/notes/bundle-adjustment.md:330-331`
- **Issue:** The rewrap breaks `` `P = R X + t` `` mid-span: `` `P = R X +`` ends one line and `` t`, `` starts the next. It renders, but the raw text reads badly. A line ending in `+` is also the shape Markdown reads as a list marker if the wrapping shifts again.
- **Suggested fix:** Break the line before `` `P = R X + t` ``, so the span stays whole.

##### [should-fix] Design-doc code sketch was reformatted for no rename reason
- **Where:** `docs/design/2026-10-02-slam-it-all-ladder.md:519-537` (the `SlamInput` and `SlamResult` sketch)
- **Issue:** The hand-aligned trailing comments were collapsed to a single space, and the blank line between the two dataclasses was doubled. Something ran a Python formatter over the fenced block. Layer 1 does not format Markdown, so it was not required.
- **Why it matters:** It is churn in a rename PR, and it removes the aligned shape and unit comments that made the sketch quick to scan.
- **Suggested fix:** Restore the BASE block, which is the same text with the old alignment.

##### [should-fix] Review-panel report template ends in an orphan `-->`
- **Where:** `.claude/skills/slam-it-all-review-panel/report-template.md:3-4`
- **Issue:** The longer skill name pushed the HTML comment's closing `-->` onto its own line. Every generated report inherits this.
- **Suggested fix:** Shorten the comment so it fits on one line, for example `<!-- Written by /slam-it-all-review-panel. One file per branch; later runs append "Run N". -->`.

##### [should-fix] Prose where "SLAM It All!" reads awkwardly
- **Where:**
  - `.claude/skills/slam-it-all-review-panel/SKILL.md:3` ("Run SLAM It All!'s review panel on SLAM It All!'s current branch")
  - `CLAUDE.md:11-12` ("Never suggest starting a session inside SLAM It All! Consequences:")
  - `docs/design/2026-10-02-slam-it-all-ladder.md:1` and `src/slam_it_all/__init__.py:1` ("SLAM It All! A learning ladder...")
  - `src/slam_it_all/foundations/ransac.py:487-488` ("raises a SLAM It All!" then "error")
- **Issue:**
  - **SKILL.md:3:** the double possessive is clumsy in a line that shows up in skill listings.
  - **CLAUDE.md:12:** the "!" reads as ending the sentence, so "Consequences:" looks like a new sentence with no object.
  - **Title and docstring:** the "!" followed by a capitalised fragment loses the colon the old titles had. The repo's house style for this kind of break is ` -- `, as in the agent descriptions.
  - **ransac.py:487:** the docstring names an exception by prose and splits the name across a line break.
- **Suggested fix:**
  - **SKILL.md:3:** "Run the review panel on the current branch of the SLAM It All! repository".
  - **CLAUDE.md:12:** "Never suggest starting a session inside it." or "inside the SLAM It All! repository."
  - **Title and docstring:** "SLAM It All! -- a learning ladder for visual SLAM", or just "A learning ladder for visual SLAM".
  - **ransac.py:487:** "raises a :class:`~slam_it_all.foundations.errors.SlamItAllError`".
  - Optional: in Python, Rust and C++ docstrings, "this package's" avoids the clumsy "!'s" in the roughly 40 possessives.

##### [note] Identifiers in prose without code font
- **Where:** `docs/design/2026-10-02-slam-it-all-ladder.md:371` ("the slam_it_all module"), `:539` and `:724` ("installs slam-it-all editable", "also installs slam-it-all"), `docs/notes/absolute-pose.md:111` ("nothing imported from slam_it_all")
- **Issue:** These are the module or distribution name, but they sit bare in sentences. Neighbouring lines use "SLAM It All!" for the project, so a reader cannot tell whether the identifier or the project is meant.
- **Suggested fix:** Put them in backticks.

##### [note] Stale reference to the panel's slash command (pre-existing)
- **Where:** `scripts/check.sh:6` ("The reviewer panel (/review-panel)")
- **Issue:** The skill was never called `/review-panel`; it was `/slamlab-review-panel`, and is now `/slam-it-all-review-panel`. The rename is a natural moment to fix it.

##### [note] `docs/reviews/README.md` points at a skill path that no longer exists
- **Where:** `docs/reviews/README.md:3` (`/slamlab-review-panel`, `.claude/skills/slamlab-review-panel/`)
- **Issue:** I know `docs/reviews/*` is kept as history on purpose. This one file is a living guide to the folder, not a report, and its path is now dead. If your decision covers it, ignore this.

##### [note] Non-mechanical edits to name in the PR body
- **Where:**
  - `scripts/compare_ba_colmap.py:59` (the table column `"slamlab"` is now `"ours"`)
  - `scripts/demo_bal.py:104` (the COLMAP line label is right-aligned with `{'COLMAP (Ceres)':>19}`)
  - `rust/slam-it-all-eval/Cargo.toml:4` ("outside the algorithm cores" is now "outside the cores")
  - `rust/slam-it-all-foundations/Cargo.toml:4` ("-- builds" is now ": builds")
  - `tests/foundations/test_ba.py:1` (dropped "the" before "optimum")
  - `src/slam_it_all/foundations/ba/layout.py:148` (dropped "see")
  - `README.md:52-53` (the benchmark command lost its `&&` and became two lines, while the native-tests command above it keeps `&&`)
  - `CLAUDE.md:174` (a sentence removed)
- **Why it matters:** All are harmless and sensible. They are not renames, so the "How this was built" section should list them.
- **Related:** the BA note's table header (`docs/notes/bundle-adjustment.md:386`) says "SLAM It All!" while the script now prints `ours`.

##### [note] Where the owner's out-of-repo state still says the old name
- **Where:** `CLAUDE.md:161-169`, `.claude/skills/slam-it-all-review-panel/SKILL.md:23-24`
- **Issue:** The repo now documents `gh pr merge <n> --repo evan-wes/slam-it-all` and the heading "The one exception: SLAM It All!". Current state outside the repo:
  - the owner's merge-guard hook has `ALLOWED_REPO="evan-wes/slamlab"`.
  - the owner's global Claude Code instructions still says "The one exception: slamlab".
  - reifyscene's `.claude/` has four symlinks to `slamlab-*-reviewer.md` and `slamlab-review-panel`; they point at the old names.
  - The conda envs `slamlab` and `slamlab-ci` and the cache `~/.cache/slamlab-offline-ci` become orphans.
  - The new offline-CI cache starts empty, so the first `offline_ci.sh` run rebuilds the checkout and env.
- **Why it matters:** This is forward-looking by design. It still needs sequencing: rename the folders and the GitHub repo, then update the hook and global CLAUDE.md before the next merge, or the documented merge command will be blocked.

##### [note] Naming rule is not in CLAUDE.md
- **Where:** `CLAUDE.md`
- **Issue:** The which-form-where rule lives in the README, the ladder (lines 707-710) and the CHANGELOG, but not in CLAUDE.md, which every reviewer reads first.
- **Suggested fix:** Add one line to "Conventions".

##### [note] Broken sibling link (pre-existing, untouched)
- **Where:** `docs/design/2026-10-02-slam-it-all-ladder.md:35` (`../../../project/reifyscene`)
- **Issue:** From `docs/design/` that resolves to `<project folder>/project`, which does not exist. It needs four `..`. The PR does not cause it.

#### Checked and fine
- **Leftover old name:** `git grep -n -i slamlab` leaves only ladder lines 707-708, the new and older CHANGELOG entries, and `docs/reviews/` (872 lines in 11 files). No "slam lab", "slam_lab", "slam-lab" or "SLAMLab" outside the intended places. No tracked file or directory still carries the old name. The stale `src/slamlab/` directory is gone from the working tree.
- **Spelling by kind:** the code identifiers, crates, distributions, conda envs and lock, macros, CMake options, env vars, programs, run format and exception are consistent across Python, C++, Rust, CMake, Cargo, pyproject, CI and scripts. Regex scans for mixed separators (`slam_it_all-`, `slam-it-all_`, `-n slam_it_all`, `rust/slam_it_all…`, `cpp/include/slam-it-all`) found none.
- **Mangled symbol:** `tests/test_backends.py:152` correctly uses `_ZN11slam_it_all3hatE`; 11 is the right length prefix.
- **Display name:** no "SLAM It All!." or "!," collisions. "SLAM It All!" never appears inside backticks or as an identifier, and `slam_it_all` and `slam-it-all` appear in prose only in the places listed above.
- **References:** path references across all non-review files resolve to tracked files or directories. The only unresolved ones are old paths in historical CHANGELOG entries, the planned `rig-calibration` files, and fixture names in `test_check_layers.py`. Relative Markdown links resolve apart from the pre-existing one above. The ladder links in `edge-vo.md` and `rig-calibration.md` and the `docs/design/*slam-it-all-ladder.md` glob in the agents are correct.
- **Front matter:** the three agents and the skill parse as YAML, and each `name` matches its file.
- **Width:** no line was pushed past 100 columns by the rename. The clang-format, rustfmt and ruff-format reflows (C++ tests, Rust `use` lists, `test_check_layers.py`, `pyproject.toml` `source_modules`) are formatter output.
- **Scripts and CI:** `offline_ci.sh` is consistent (cache dir, checkout, `slam-it-all-ci`, the lock hash inputs). The `gen_env_lock.sh` exclude pattern covers both `slam-it-all` and `slam-it-all-rs`. `test.yml` calls `setup_slam_it_all.sh` and `check.sh --env slam-it-all`.
- **Layout, versioning, wording:** rust workspace members, the `[lib]` and `[[bin]]` names and the Doxyfile quoting are consistent. VERSION and the CHANGELOG heading agree on 0.3.4.0. The agents and skill are consistent with each other and with CLAUDE.md in tone and wording.
