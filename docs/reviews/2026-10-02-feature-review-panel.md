# Review panel: `feature/review-panel`

<!-- Written by /review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/review-panel` |
| **Range** | `874db0f...3cce3b9` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | general-purpose agents carrying each `.claude/agents/*.md` role and model (the session started outside the slamlab repo, so the project agents were not registered) |
| **Layer 1** | all 15 checks passed (`scripts/check.sh`) |
| **Process note** | The hygiene reviewer briefly ran `git checkout 3cce3b9` (detached HEAD), against its read-only rule, then switched back. The branch was verified unchanged at `3cce3b9` with a clean tree. |

The first panel run, reviewing the change that introduces the panel itself.

### Findings and resolutions

Overlapping findings are cross-referenced; they are decided together.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A1 | must-fix | architecture | `.github/workflows/test.yml`, `backends.py`, `tests/test_backends.py` | CI no longer asserts all three backends are built; a broken extension turns parity tests into skips and CI stays green (with H1, H2) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H1 | should-fix | hygiene | `src/slamlab/backends.py` (`_import`) | Any `ImportError` is reported as "not built", hiding a broken extension (with A1) | fix | fix (with A1) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H2 | should-fix | hygiene | `tests/test_backends.py` (`_require`) | Parity tests skip silently when a compiled backend is missing, including in CI (with A1) | fix | fix (with A1) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H3 | should-fix | hygiene | `src/slamlab/backends.py` (`Backend.hat`) | Docstring lacks Parameters/Returns/Raises; `name` undocumented | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H4 | should-fix | hygiene | `scripts/setup_slamlab.sh` header | Says "three editable installs", lists two | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H5 | should-fix | hygiene | `pyproject.toml` `dev` extra | Out of step with the real toolchain (`mypy`, `import-linter` missing) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H6 | should-fix | hygiene | `.github/workflows/test.yml` | Miniforge from `latest` with no pin; no `permissions`, no `timeout-minutes` | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H7 | should-fix | hygiene | `scripts/check.sh` | `--env` without a value dies obscurely; unknown arguments silently ignored | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H8 | should-fix | hygiene | `.claude/skills/review-panel/SKILL.md` | Ambiguous agent-availability sentence; `HEAD` variable name; dirty-tree check unspecified | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H9 | should-fix | hygiene | tests in all three languages | `1e-12` tolerance on an exact operation, unnamed (with L1) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L1 | should-fix | algorithm | `tests/test_backends.py`, `tests/fixtures/so3_hat.json` | Parity tests cannot detect a float32 regression: default `rtol=1e-7`, float32-exact fixture values (with H9) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L2 | should-fix | algorithm | `src/slamlab/so3.py` (`as_vec3`) | Accepts `None` (to NaN), strings, bools, complex, and non-finite values (with H11) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L6 | should-fix | algorithm | `.claude/agents/algorithm-reviewer.md` | Reference list misses rung-0 classics (Nistér, Hartley 8-pt, RANSAC, EPnP, Umeyama, ...) and current SOTA (MASt3R-SLAM, DPVO, Shape of Motion) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A2 | should-fix | architecture | `pyproject.toml` import-linter | Layers contract names one module; will not cover rung code. Proposes package-per-rung with layer, independence and no-torch-on-edge contracts | defer to milestone 1 step 0 | fix in this PR (restructure design approved) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A3 | should-fix | architecture | `src/slamlab/backends.py` | `Backend` is a per-algorithm facade that will grow with every rung; proposes compiled submodules mirroring the package tree, dispatch next to each algorithm | defer to milestone 1 step 0 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A4 | should-fix | architecture | `src/slamlab/so3.py` (`as_vec3`) | Generic validation helper placed in the SO(3) module; belongs in a rung-0 validation module | defer to milestone 1 step 0 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A5 | note | architecture | `scripts/check.sh` | Flat C++ globs will miss subdirectories | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A6 | note | architecture | `check.sh`, CI, env | Plan how layer 1 splits for GPU and budget rungs (pytest markers, a light edge env) | defer to the first PyTorch rung | fix in this PR -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A7 | note | architecture | `pyproject.toml` mypy override | Compiled extensions have no type stubs | defer to the second bound function | fix in this PR -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) with Protocols instead of stubs; run 2 judged it partially resolved (see A12) |
| A8 | note | architecture | design doc header | Status line says nothing is built | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L3 | note | algorithm | `rust/slamlab-py/src/lib.rs` | Docstring names `TypeError`; PyO3 raises `ValueError` | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L4 | note | algorithm | C++/Rust native tests | One `(v, u)` pair does not pin the matrix; no skew-symmetry check | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L5 | note | algorithm | `tests/fixtures/` | Rung-0 fixtures will need the hard cases (near 0, near pi, wrap) | defer to the milestone 1 plan | fix: tolerance policy and fixture format now (L1); the hard exp/log cases land with exp/log in milestone 1, as the reviewer suggested -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L7 | note | algorithm | `.claude/agents/algorithm-reviewer.md` | Checklist gaps: tolerance policy, quaternion double cover, angle wrap, cheirality, Jacobians, seeded RNG, ATE alignment | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| L8 | note | algorithm | `so3` in all languages | Placeholder scope | no action | no action (placeholder scope) -- decided by owner (evan-wes), 2026-10-02 |
| H10 | note | hygiene | `backends.py`, `slamlab-py` | Validation happens in `Backend`; raw extensions' own boundary is untested | fix (state in the `Backend` docstring) | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H11 | note | hygiene | `as_vec3` | No finiteness check (covered by L2) | fix with L2 | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H12 | note | hygiene | `backends.py` | Backend list repeated; `so3` imported two ways | defer with A3 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H13 | note | hygiene | `CLAUDE.md` | Merge rule conflicts with the owner's global "never merge" rule (same as A9) | dispute | fix: CLAUDE.md cites the owner's explicit grant -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| A9 | note | architecture | `CLAUDE.md` | Same as H13 | dispute | fix: CLAUDE.md cites the owner's explicit grant -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |
| H14 | note | hygiene | `Doxyfile`, PR template, `setup_slamlab.sh`, `pyproject.toml` | HTML output unneeded; relative report link; x86_64 compiler names; deprecated license table | fix (HTML, link, license); defer x86_64 to milestone 7 | fix (all parts, including the x86_64 compiler names) -- decided by owner (evan-wes), 2026-10-02; fixed (352fa25) |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

**Scope:** `874db0f...3cce3b9` on `feature/review-panel` (36 files). Read in full: `scripts/check.sh`, `scripts/setup_slamlab.sh`, `.github/workflows/test.yml`, `pyproject.toml`, `.jscpd.json`, `Doxyfile`, `.clang-*`, `.gitignore`, `src/slamlab/*`, `tests/test_backends.py`, C++/Rust sources, `CLAUDE.md`, PR template, `SKILL.md`, `report-template.md`, agent headers. `hat` treated as explicitly-marked placeholder.
**Summary:** Clean, well-documented change; shell scripts are sensible (`check.sh` deliberately omits `-e` to run all checks; `setup_slamlab.sh` uses `set -euo pipefail`). No must-fix items. The main weaknesses are error-handling that masks real backend breakage, a test design that can silently skip all compiled backends in CI, a few stale or inconsistent documentation spots, and an unpinned CI bootstrap.

(Process note: I ran `git checkout 3cce3b9` early in the review, which detached HEAD; I immediately switched back to `feature/review-panel`, which still sits at `3cce3b9` with a clean tree. No files were changed.)

#### Findings

##### [should-fix] `_import` swallows any ImportError as "not built"
- **Where:** `src/slamlab/backends.py:139-140` (and message at `:178-181`)
- **Issue:** `except ImportError: return None` catches every import failure — including a built extension that fails to load (missing shared lib, ABI mismatch, numpy ABI error). The original error is discarded, and `get()` then reports "not built in this environment … run scripts/setup_slamlab.sh".
- **Why it matters:** A broken backend is misreported as absent, with misleading advice; `available()` silently drops it, and parity tests skip it.
- **Evidence:** `except ImportError:\n    return None` then `raise ImportError(f"backend {name!r} is not built ...")` with no `from`.
- **Suggested fix:** Catch `ModuleNotFoundError` and return `None` only when `exc.name` is the extension itself; let other ImportErrors propagate (or chain with `raise ... from exc`).

##### [should-fix] Parity tests silently skip when a compiled backend is missing, including in CI
- **Where:** `tests/test_backends.py:19-23`; `.github/workflows/test.yml:176-180`
- **Issue:** `_require` calls `pytest.skip` when a backend isn't built. CI builds all three, but nothing asserts they're present — if the C++ or Rust extension fails to import, the suite stays green with skips.
- **Why it matters:** The parity tests exist to prove all three backends agree; CI could pass without checking two of them. Compounds the previous finding.
- **Evidence:** `if name not in backends.available(): pytest.skip(...)`.
- **Suggested fix:** Add an env switch (e.g. `SLAMLAB_REQUIRE_BACKENDS=cpp,rust`) set in CI and `check.sh` that turns the skip into a failure, or a test asserting `available() == ["python","cpp","rust"]` when that var is set.

##### [should-fix] `Backend.hat` docstring omits Parameters/Returns/Raises
- **Where:** `src/slamlab/backends.py:112-117`
- **Issue:** `CLAUDE.md` requires full numpy-style sections stating shape and dtype. This method has a one-line summary plus a pointer to `so3.hat`. Passes `ruff`, but not the usefulness standard. `Backend.name` is also undocumented, and `__init__` has no docstring (class docstring covers params).
- **Why it matters:** This is the public entry point callers use, and the only place stating that validation happens here rather than in the cores.
- **Evidence:** Docstring body: "Same contract as the reference, :func:`slamlab.so3.hat`; input is checked here …"
- **Suggested fix:** Add `Parameters` (shape (3,)), `Returns` (float64, shape (3,3)), `Raises` (`ValueError`), plus an `Attributes` entry for `name`.

##### [should-fix] `setup_slamlab.sh` header is stale ("three editable installs", lists two)
- **Where:** `scripts/setup_slamlab.sh:194-198`
- **Issue:** Text says "Then three editable installs" and lists two.
- **Why it matters:** Minor, but documentation in a process file CI executes.
- **Suggested fix:** "two editable installs".

##### [should-fix] `pyproject` dev extra out of sync with the real toolchain
- **Where:** `pyproject.toml:75` vs `scripts/setup_slamlab.sh:252-254` and lockfile
- **Issue:** `dev` extra lists `pytest`, `ruff`, `maturin` but not `mypy`, `import-linter`, or build requirements — the tools `check.sh` needs.
- **Why it matters:** Two sources of truth for the dev toolchain have already drifted; `pip install -e .[dev]` yields an env where `check.sh` fails.
- **Suggested fix:** Make the extra complete and have the unlocked branch of `setup_slamlab.sh` install `.[dev]`; or drop the extra and document the script as the only supported route.

##### [should-fix] CI bootstraps from unpinned, unverified downloads and lacks hardening
- **Where:** `.github/workflows/test.yml:166-170`
- **Issue:** Miniforge from `releases/latest/download/...` with no version pin or checksum. No `permissions:` block (only needs `contents: read`) and no `timeout-minutes`.
- **Why it matters:** The env is otherwise carefully locked/pinned (clang, jscpd, pip lock); an unpinned base installer undermines reproducibility. A conda-forge outage or hung build could hold a runner for the 6-hour default.
- **Suggested fix:** Pin a Miniforge release (or use `conda-incubator/setup-miniconda`), add `permissions: contents: read` and `timeout-minutes`.

##### [should-fix] `check.sh` argument handling is fragile
- **Where:** `scripts/check.sh:18-21`
- **Issue:** `--env` without a value hits `$2` under `set -u` → bare "unbound variable". Any other argument (e.g. typo `--evn slamlab`) is silently ignored and checks run against the host PATH. `setup_slamlab.sh` properly rejects unknown args.
- **Why it matters:** A typo yields confusing failures or silently checks the wrong environment.
- **Suggested fix:** `case` loop like `setup_slamlab.sh`: require a value for `--env`, `exit 2` on unknown arguments.

##### [should-fix] SKILL.md has a confusing sentence and an ambiguous variable name
- **Where:** `.claude/skills/review-panel/SKILL.md:57-59` and `:29-34`
- **Issue:** "If a project agent type is not available in the session (it is when the session runs inside the slamlab repo)" is ambiguous — reads as if it's *not* available inside the repo, the opposite of the intent. Step 1 names a variable `HEAD`, colliding with git's own `HEAD`. The dirty-tree check names no command.
- **Why it matters:** The skill is an instruction document; an agent following it literally may misread the fallback condition.
- **Suggested fix:** "Project agents are available when the session runs inside the slamlab repo; otherwise spawn …". Rename to `HEAD_SHA`. State the check (e.g. `git status --porcelain` must be empty).

##### [should-fix] Tests use `1e-12` tolerance on an exact operation; tolerance is an unnamed literal
- **Where:** `tests/test_backends.py:36,44`; `rust/slamlab-core/src/so3.rs:104`; `cpp/tests/test_so3.cpp:172`
- **Issue:** `hat` only rearranges/negates entries, so the fixture comparison can be exact. `atol=1e-12` is a magic number repeated across three languages, justified nowhere. The fixture test wouldn't catch a 1e-13 error.
- **Why it matters:** `hygiene-reviewer.md` requires named, justified tolerances; looser-than-needed tolerances weaken parity tests.
- **Suggested fix:** Exact equality for `test_hat_matches_fixture` (`assert_array_equal`); keep a named tolerance only for the cross-product test where rounding actually occurs.

##### [note] Backend-level validation masks each compiled core's own boundary behaviour
- **Where:** `src/slamlab/backends.py:118`; `rust/slamlab-py/src/lib.rs:121-122`
- **Issue:** `test_hat_rejects_wrong_shape` goes through `Backend.hat`, which validates first. The raw extensions aren't tested directly — the Rust binding documents a `TypeError` from PyO3, nanobind similarly. The "every backend rejects the same way" claim holds only through the wrapper.
- **Why it matters:** Acceptable by design, but the guarantee belongs to `Backend`, and nothing exercises the native boundary.
- **Suggested fix:** None required now; note it in `Backend`'s docstring.

##### [note] `as_vec3` checks shape only, not finiteness
- **Where:** `src/slamlab/so3.py:33-36`
- **Issue:** NaN/inf pass through silently. `hygiene-reviewer.md` lists finiteness as a boundary check.
- **Why it matters:** This is the explicitly-marked placeholder, so a note. When real SO(3) functions land, decide whether finiteness is validated at `as_vec3` or left to callers.

##### [note] Backend list and `so3` import duplicated in `backends.py`
- **Where:** `src/slamlab/backends.py:94,136,151,174`; `tests/test_backends.py:16`
- **Issue:** Names enumerated in `_COMPILED`, `get`'s check, `available`'s tuple, and tests' `BACKENDS`; the `"python"` branch of `_import` uses `importlib.import_module("slamlab.so3")` though `so3` is already imported at the top.
- **Why it matters:** Minor structure smell that will grow as backends are added.
- **Suggested fix:** Single `_MODULES = {"python": "slamlab.so3", **_COMPILED}` and derive the rest from it.

##### [note] `CLAUDE.md` merge rule conflicts with the owner's global instruction
- **Where:** `CLAUDE.md:261-264`
- **Issue:** The project file authorizes Claude to run `gh pr merge` after a selected Yes on a prompt for that PR. The owner's global instructions say Claude never merges any PR, and `settings.json` denies the command.
- **Why it matters:** Contradictory instructions to agents. This is a process decision outside hygiene — flagging only, not judging which is right; for the owner (or the architecture reviewer) to resolve.

##### [note] Smaller items
- **Where:** `Doxyfile:7`; `.github/pull_request_template.md:164`; `scripts/setup_slamlab.sh:243-244`
- **Issue:** `Doxyfile` sets `GENERATE_HTML = YES` but the check only needs warnings (`NO` is faster). The PR template's report link `../docs/reviews/...` is relative and may not resolve from a PR description. `setup_slamlab.sh` hard-codes `x86_64-conda-linux-gnu-*` compilers (breaks on aarch64). `license = { file = ... }` table form in `pyproject.toml` is deprecated in favor of an SPDX string.

#### Checked and fine
- `scripts/check.sh`: `set -uo pipefail` without `-e` is deliberate and documented; failure collection and exit logic are correct; globs correctly quoted in arrays; `conda run --no-capture-output` wrapper works.
- `setup_slamlab.sh`: strict mode, argument validation, clear conda/pip split, pinned clang and jscpd, per-env build-dir rationale documented inline; CC/CXX pinning explained.
- Duplication: the shared `as_vec3` replaces the earlier copy-pasted check.
- `hat` implementations: Python numpy-style, C++ Doxygen, Rust rustdoc all complete, state shape and row-major layout, and carry the "placeholder" marking; doctest and rustdoc examples execute as tests.
- Tool configs: `.clang-tidy` exclusions each justified inline; ruff ignore, mypy overrides, import-linter contracts sensible and commented; `.jscpd.json` ignores reasonable.
- `.gitignore` correctly versions `.claude/agents` and `.claude/skills` while ignoring local settings.
- Agent/skill/format files are mutually consistent (tools, models, report IDs, finding format); CHANGELOG entry matches the diff (24 tests collected); VERSION bumped to 0.1.2.0.
- Rust lint setup (`missing_docs`, pedantic, `unsafe_code = forbid`) and Rust/C++ implementations are minimal and correct.

### Architecture review

**Scope:** `874db0f...3cce3b9` on `feature/review-panel` — the full diff (check script, tool configs incl. import-linter/ruff/mypy in `pyproject.toml`, Cargo lints, `.clang-tidy`, `Doxyfile`, `.jscpd.json`, CI workflow, reviewer agents, `/review-panel` skill, PR template, CLAUDE.md, design-doc addition) plus the scaffold changes in `src/slamlab/{backends,so3,__init__}.py`, `cpp/`, `rust/`, `tests/test_backends.py`, and their neighbours `CMakeLists.txt` and `scripts/setup_slamlab.sh`.

**Summary:** The two-layer process is structurally sound and the standalone-core rule still holds (CMake adds nanobind only under `SKBUILD`; Rust tests run `-p slamlab-core` alone; bindings remain pure type conversion). One real regression in the parity guarantee: CI no longer asserts that all three backends are present, and the backend loader swallows any `ImportError`, so a broken compiled extension would silently turn every cpp/rust parity test into a skip and CI would still pass. The rest is about scale: the import-linter contracts and the `Backend` class are both written for exactly one module (`so3`) and will not cover rung code automatically when it arrives.

#### Findings

##### [must-fix] CI no longer proves the cpp and rust backends are present; a broken extension passes as green
- **Where:** `.github/workflows/test.yml` (removed the old line-26 assertion), `src/slamlab/backends.py:65`, `tests/test_backends.py` (`_require`), `scripts/setup_slamlab.sh` (last line prints `available()` but does not check it)
- **Issue:** On base, CI ran `assert b == ['python', 'cpp', 'rust']`. The new single job runs `setup_slamlab.sh` + `check.sh`, and neither asserts backend availability. `_import` treats *any* `ImportError` as "not built" (including an extension that was built but fails to load — undefined symbol, ABI mismatch, PyO3 init failure), and the parity tests then `pytest.skip`.
- **Why it matters:** The language split rests on parity fixtures ("Every C++/Rust implementation … checked against the Python reference"). Today a C++ or Rust extension that builds but cannot import produces a green CI run with zero parity coverage — the only signal is a skip count nobody reads.
- **Evidence:** base `test.yml:26`: `python -c "import slamlab; b = slamlab.backends.available(); print(b); assert b == ['python', 'cpp', 'rust'], b"` — deleted. `backends.py:63-66`: `try: return importlib.import_module(_COMPILED[name])` / `except ImportError: return None`. `test_backends.py`: `if name not in backends.available(): pytest.skip(...)`.
- **Suggested fix:** Restore the check inside `check.sh` (e.g. a `check "all backends built"` step, or a `SLAMLAB_REQUIRE_BACKENDS=python,cpp,rust` setting that makes `_require` fail instead of skip) so local and CI enforce it identically. Also narrow `_import` to return `None` only for `ModuleNotFoundError` whose `.name` is the extension itself, re-raising other load failures.

##### [should-fix] The import-linter layers contract names one module and will not cover rung code
- **Where:** `pyproject.toml:79-82` (`layers = ["slamlab.backends", "slamlab.so3"]`)
- **Issue:** The contract "Algorithms never depend on the backend selector" only constrains `slamlab.so3`. Every new algorithm module (camera models, SE(3), RANSAC, the BA solver) is outside it until someone remembers to add it. And the design doc's ladder rules — rung 0 depends on nothing above it, sibling branches stay independent, algorithms never depend on orchestration/evaluation — are not yet expressible because modules live flat at `slamlab.*` with no rung packages.
- **Why it matters:** A dependency rule that must be manually extended fails silently, which is exactly the failure layer 1 exists to prevent. Milestone 1 will add several modules at once.
- **Evidence:** `pyproject.toml:82`; the tree at HEAD has only `src/slamlab/{__init__,backends,so3}.py` — no per-rung packages.
- **Suggested fix:** At the start of milestone 1, give each rung its own package mirroring the ladder (e.g. `slamlab.foundations`, `slamlab.edge`, `slamlab.vi`, `slamlab.classic`, `slamlab.dense`, `slamlab.learned`, plus `slamlab.eval` / `slamlab.orchestration`) and replace this contract with package-level ones that cover new modules automatically: (a) layers `backends | (orchestration, eval) | (edge, classic, dense) | foundations` with `vi` above `edge` and `learned` above `dense`; (b) `independence` between the sibling branches `edge`, `classic`, `dense`; (c) `forbidden`: `foundations`, `edge`, `vi` may not import `torch`/`torchvision`/any GPU stack (`include_external_packages = true` is already set, so this works today and enforces the edge-budget rule even transitively); (d) `forbidden`: all algorithm packages may not import depth-model packages (`unidepth`, `moge`, `vggt`) — those go through the provider interface. Adding (c) now is possible if the rung packages are created now.

##### [should-fix] `Backend` is a per-algorithm facade that will grow with every rung and import every algorithm's input checks
- **Where:** `src/slamlab/backends.py:18`, `:38-45`, `:62`
- **Issue:** Dispatch is hard-coded per function (`Backend.hat` as a method), the "python" backend is literally `slamlab.so3`, and the selector imports `so3.as_vec3` to validate input for the compiled backends. Each new algorithm would add a method here and an import of that algorithm's validators, and the "python" backend cannot span more than one module. The compiled modules are also flat (`_cpp.hat`, `slamlab_rs.hat`), so names will collide across rungs (e.g. `project` in several camera models).
- **Why it matters:** Under the ladder, `slamlab.backends` becomes the one module that depends on every algorithm in every rung — a central bottleneck and a merge hotspot — and the validation contract for an algorithm lives in the selector rather than beside the algorithm.
- **Evidence:** `backends.py:44-45` `vec = so3.as_vec3(v)` / `return np.asarray(self._module.hat(vec.tolist()), ...)`; `backends.py:62` `return importlib.import_module("slamlab.so3")`.
- **Suggested fix:** Before milestone 1 adds a second algorithm: make compiled modules mirror the Python package tree (`_cpp.foundations.so3.hat`, `slamlab_rs.foundations.so3.hat` via nanobind/PyO3 submodules), let `backends` resolve `(backend, "foundations.so3")` → module, and keep validation and numpy conversion in a thin per-algorithm dispatch function living in the algorithm's own module (e.g. `slamlab.foundations.so3.hat(v, backend="python")` validating once, then calling `backends.module(backend, __name__)`). `backends` then knows only how to locate implementations, never what they compute. Clean break, no shim; the `hat` placeholder is a fine place to set the pattern.

##### [should-fix] `as_vec3` is a general array-validation helper placed in the SO(3) module
- **Where:** `src/slamlab/so3.py:15`
- **Issue:** `as_vec3` is public, not marked placeholder, and generic — camera models, SE(3), triangulation and PnP all need `as_vec3`/`as_mat3`/`(N,3)` checks. Placed in `so3`, other modules will either import `so3` just to validate a vector or copy the function.
- **Why it matters:** "Components shared by several rungs … live in rung 0, not copied" — shared helpers should exist once, not inside one algorithm module; jscpd will catch the copies only once they exceed 50 tokens.
- **Evidence:** `so3.py:15` `def as_vec3(...)`, consumed by `backends.py:44`.
- **Suggested fix:** Move to a rung-0 validation module (e.g. `slamlab.foundations.arrays`, or a private `slamlab.foundations._validate`) when the rung packages are created; the SO(3) code and per-algorithm dispatch import it from there.

##### [note] `check.sh` source lists are flat globs that will miss subdirectories
- **Where:** `scripts/check.sh:35`, `:48`
- **Issue:** `CPP_SOURCES=(cpp/include/slamlab/*.hpp cpp/src/*.cpp cpp/bindings/*.cpp cpp/tests/*.cpp)` and `clang-tidy ... cpp/src/*.cpp cpp/tests/*.cpp` will not see `cpp/src/foundations/*.cpp` once the C++ core is split by rung. (The Doxyfile is already `RECURSIVE = YES`, so it is fine.)
- **Suggested fix:** Build the lists from `git ls-files 'cpp/**/*.cpp' 'cpp/**/*.hpp'` (or run clang-tidy over the files in `compile_commands.json`) so the layout can change without editing the script.

##### [note] Layer 1 is a single serial job in a single env — plan now how it splits for the GPU and budget rungs
- **Where:** `scripts/check.sh`, `.github/workflows/test.yml`, `scripts/setup_slamlab.sh`
- **Issue:** One env with every dependency, and `check.sh` runs all test suites on `ubuntu-latest` (x86, no GPU). That is right for rung 0 and the edge VO. The learned rungs (milestones 5, 8, 9) bring PyTorch and GPU tests; milestone 7 brings budget-capped runs and an ARM build check.
- **Suggested fix:** When the first PyTorch rung lands, add pytest markers (`gpu`, `budget`, `slow`) so `check.sh` runs the CPU-only subset by default, with a separate opt-in script/job for the rest; and consider a separate light env/extra for the edge rungs so "no torch" is also proven by installation, not only by contract (c) above. The design doc's "Compute budgets" section is where each rung's resource declaration is to be named — it does not yet say where in code that declaration lives.

##### [note] Compiled extensions have no type stubs, so the binding contract is untyped
- **Where:** `pyproject.toml` mypy override (`module = ["slamlab._cpp", "slamlab_rs"]`, `ignore_missing_imports = true`)
- **Issue:** `Backend._module` is a `ModuleType`, so every compiled call is `Any`; signature drift between the Python reference and a binding is not caught by `mypy --strict`, only at runtime by parity tests.
- **Suggested fix:** Generate `.pyi` stubs (`nanobind.stubgen`; `pyo3-stub-gen` or hand-written for the Rust module) when the second bound function arrives, and drop the override. Acceptable for the placeholder.

##### [note] Design doc header is stale
- **Where:** `docs/design/2026-10-02-slamlab-ladder.md:3`
- **Issue:** "Status: draft for review … nothing here is built yet." The scaffold, three-language packaging and review process now exist. Recommend updating the doc (not the code): change the status line to reflect what exists.

##### [note] Merge rule in CLAUDE.md differs from the owner's global "never merge" rule (process, for the owner to confirm)
- **Where:** `CLAUDE.md:72`
- **Issue:** `Claude may merge slamlab PRs only after a selected Yes on a yes/no prompt`. The owner's global instructions state Claude never merges a PR in any project. This may be an intentional project-level exception, but the two conflict; the owner should confirm the exception explicitly.

#### Checked and fine
- One-way reifyscene boundary: forbidden contract present, and nothing in `src/` references reifyscene.
- C++ core builds standalone: `CMakeLists.txt` adds nanobind/Python only under `SKBUILD`, and `check.sh` configures `build/native` without it.
- Rust core builds standalone: `slamlab-core` has no dependencies, `cargo test -p slamlab-core` is run; PyO3 confined to `slamlab-py`.
- Bindings (`cpp/bindings/module.cpp`, `rust/slamlab-py/src/lib.rs`) only convert types — no algorithm logic.
- Backend selection goes through `slamlab.backends`; no language hard-coded elsewhere. `slamlab_rs` stays top-level as the design doc requires.
- Parity fixture exists in `tests/fixtures/so3_hat.json` and every backend is tested against it, including the new wrong-shape rejection test (one error contract across languages at the Python boundary).
- Rust used only within its agreed subset (SO(3) placeholder = rung 0).
- The `hat` placeholder is clearly marked temporary in all three languages and is not hardening into the design.
- Workspace-wide Rust lints (`[workspace.lints]` inherited per crate) and a recursive Doxyfile scale as crates/directories are added.
- The review-process structure (layer 1 / layer 2 split, single `check.sh` shared by local and CI, reviewer agents, report-per-branch records in `docs/reviews/`, PR template process record) is sound and consistent with the design doc's new "Review process" section.
- `check.sh` runs every check and reports all failures together; CI now reproduces the env from the lockfile.
- No import-time side effects or global state added; `__init__` imports only `backends`.

### Algorithm review

**Scope:** `874db0f...3cce3b9` (branch `feature/review-panel`): `src/slamlab/so3.py` (`hat`, new `as_vec3`), `src/slamlab/backends.py`, `cpp/include/slamlab/so3.hpp`, `cpp/src/so3.cpp`, `cpp/tests/test_so3.cpp`, `cpp/bindings/module.cpp`, `rust/slamlab-core/src/so3.rs`, `rust/slamlab-py/src/lib.rs`, `tests/test_backends.py`, `tests/fixtures/so3_hat.json`, and the role definition `.claude/agents/algorithm-reviewer.md`. Read against `CLAUDE.md` and `docs/design/2026-10-02-slamlab-ladder.md` (no `docs/notes/` yet). Probed all three built backends in the `slamlab` conda env with scratchpad-only scripts.

**Summary:** `hat` is correct and identical across Python, C++ and Rust; the row-major layout `[[0,-z,y],[z,0,-x],[-y,x,0]]` matches the standard `hat(v) u = v × u` convention. The main weakness is the parity harness: as written it cannot detect a backend that silently computes in float32, and its fixtures exercise only easy, float32-exact values — the harness rung 0's exp/log parity will copy, so worth tightening now. `as_vec3` checks shape but silently accepts `None` (→ NaN), strings, bools and non-finite values. The role definition is sound but should gain a few rung-0 classics and current dense-SLAM state of the art.

#### Findings

##### [should-fix] Parity tests cannot detect a float32 (or other precision) regression
- **Where:** `tests/test_backends.py:36`, `tests/test_backends.py:44`, `tests/fixtures/so3_hat.json`
- **Issue:** `np.testing.assert_allclose(..., atol=1e-12)` leaves `rtol` at its default `1e-7`, so the effective tolerance is relative 1e-7, not 1e-12. All fixture values (0, 1, 2, 3, -0.5, 4.25, -7) are also exactly representable in float32. A backend that downcasts to float32 passes both the fixture and the cross-product tests.
- **Why it matters:** CLAUDE.md makes "every C++/Rust implementation … checked against the Python reference" the guarantee, and the design doc flags float32 vs float64 as a deliberate choice for edge rungs. The harness that rung 0's `exp`/`log`/Jacobian parity will copy is blind to precision loss — exactly where near-zero `exp`/`log` loses accuracy.
- **Evidence:** Simulated float32 backend (`so3.hat(np.asarray(v, dtype=np.float32)).astype(np.float64)`) run through the same assertions:
  ```
  fixture test passes for float32 backend
  cross test passes for float32 backend
  ```
- **Suggested fix:** Pass `rtol=0` (or a deliberate rtol) explicitly everywhere; for `hat` (copy and negate only) use `assert_array_equal`. Add non-float32-exact fixture values (e.g. `0.1`, `1/3`, `1e-300`, `1e300`, `-0.0`). Record a tolerance policy in the fixture JSON (per-case `rtol`/`atol`) so later rungs set it deliberately.

##### [should-fix] `as_vec3` silently accepts non-numeric and non-finite input
- **Where:** `src/slamlab/so3.py:33` (`np.asarray(v, dtype=np.float64)`); reaches all three backends via `backends.py`
- **Issue:** Casting with `dtype=np.float64` coerces anything numpy can: `None` → `nan`, `"1"` → `1.0`, bools → 0/1, complex → drops the imaginary part with only a `ComplexWarning`. NaN/inf pass straight through. The docstring promises "sequence of float" and only rejects by shape.
- **Why it matters:** This is the shared boundary check the brief says rung 0 builds on, and CLAUDE.md says "validate inputs at public boundaries; make failure observable." A `None` that becomes NaN is an unobservable failure; under `exp`/`log` a NaN fails differently per language, showing up as a confusing parity failure rather than a clear input error.
- **Evidence:** Probe through `backends.get(b).hat(...)` (identical for python, cpp, rust):
  ```
  python none OK [0.0, -2.0, 1.0] [2.0, 0.0, nan]
  python str  OK [0.0, -3.0, 2.0] [3.0, 0.0, -1.0]
  python inf  OK [0.0, -2.0, 1.0] [2.0, 0.0, -inf]
  python complex ComplexWarning Casting complex values to real discards the imaginary part   (warnings promoted to errors)
  ```
- **Suggested fix:** `np.asarray(v)` first; reject dtype kinds outside integer/unsigned/float (decide on bool deliberately); then cast to float64; raise `ValueError` if `not np.all(np.isfinite(arr))` (or document that non-finite propagates, if that is the chosen policy). Add these cases to `test_hat_rejects_wrong_shape` or a sibling test.

##### [note] Rust binding docstring names the wrong exception
- **Where:** `rust/slamlab-py/src/lib.rs:12-13`
- **Issue:** Says PyO3 rejects a non-three-float input "with a Python `TypeError`"; PyO3 actually raises `ValueError` for a wrong-length sequence (nanobind/C++ raises `TypeError`). Callers normally go through `as_vec3`, so this only affects direct `slamlab_rs.hat` use, but the documented failure behaviour is wrong.
- **Evidence:** `slamlab_rs.hat([1.0, 2.0])` → `ValueError expected a sequence of length 3 (got 2)`; `slamlab._cpp.hat([1.0, 2.0])` → `TypeError: hat(): incompatible function arguments`.
- **Suggested fix:** Correct the docstring. (Doc-correctness; hygiene lane.)

##### [note] Native C++/Rust tests do not pin the matrix down
- **Where:** `cpp/tests/test_so3.cpp:10-21`, `rust/slamlab-core/src/so3.rs:49-64` (and `tests/test_backends.py:40-44`)
- **Issue:** Each checks `hat(v) u = v × u` for a single `(v, u)` pair — 3 equations on 9 entries, so a wrong matrix can pass. The full matrix is pinned by the Python fixture and the Rust doc-test, but the C++ core has no native full-matrix check, and none checks skew-symmetry (`hat(v)^T = -hat(v)`) or linearity directly.
- **Why it matters:** Acceptable for a placeholder, but the native C++ build runs without Python on edge budgets, where these are the only tests.
- **Suggested fix:** When rung 0 lands, test against the standard basis vectors (fixes every entry) plus skew-symmetry; add `vee(hat(v)) == v` once `vee` exists.

##### [note] Fixtures for rung 0 will need the numerically hard cases
- **Where:** `tests/fixtures/so3_hat.json`
- **Issue:** The placeholder fixture covers zero, small integers and quarter-fractions only — fine for `hat`. The role definition itself requires parity fixtures to "cover the numerically hard cases" (`algorithm-reviewer.md:57`); for `exp`/`log` those are |ω| ≈ 0 (Taylor branch), |ω| ≈ π (log ambiguity/antipodal axis), |ω| > π (wrap), and non-orthonormal input to `log`.
- **Evidence:** Solà, Deray, Atchuthan, "A micro Lie theory for state estimation in robotics", arXiv:1812.01537 (2018), §§ on SO(3) exp/log and their small-angle forms; Sophus and manif implement explicit small-angle branches (reference implementations: https://github.com/strasdat/Sophus, https://github.com/artivis/manif).
- **Suggested fix:** None now — carry into the milestone-1 plan.

##### [should-fix] Role definition: reference list misses rung-0 classics and current dense-SLAM SOTA
- **Where:** `.claude/agents/algorithm-reviewer.md:31-35`
- **Issue:** The list covers geometry (H&Z, Szeliski), Lie theory (Barfoot, Solà), BA (Triggs), VI (Forster), ORB-SLAM, DROID, MegaSaM. Rung 0 and the design-doc milestones name methods with no listed reference:
  - 5-point: Nistér, "An efficient solution to the five-point relative pose problem", IEEE TPAMI 2004.
  - 8-point normalisation (which the checklist itself names, line 45): Hartley, "In defense of the eight-point algorithm", IEEE TPAMI 1997.
  - RANSAC: Fischler & Bolles, CACM 1981.
  - PnP: Lepetit, Moreno-Noguer & Fua, "EPnP", IJCV 2009.
  - Sim(3): Strasdat, Montiel & Davison, "Scale Drift-Aware Large Scale Monocular SLAM", RSS 2010.
  - ATE alignment and metrics: Umeyama, IEEE TPAMI 1991; Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012 (ATE/RPE).
  - Loop closure (milestone 4): Gálvez-López & Tardós, "Bags of Binary Words…", IEEE T-RO 2012.
  - Dense BA with depth prior (milestone 3/5, "the full Shape-of-Motion recipe" per the design doc): Wang et al., "Shape of Motion: 4D Reconstruction from a Single Video", ICCV 2025, https://arxiv.org/abs/2407.13764 — whose reference implementation is already in reifyscene's `third_party/shape-of-motion` (verified present, alongside `vggt`, `VGGT-Long`, `Hierarchical-Localization`).
  - SOTA context for the learned rungs: Murai, Dexheimer & Davison, "MASt3R-SLAM: Real-Time Dense SLAM with 3D Reconstruction Priors", CVPR 2025 (https://openaccess.thecvf.com/content/CVPR2025/html/Murai_MASt3R-SLAM_Real-Time_Dense_SLAM_with_3D_Reconstruction_Priors_CVPR_2025_paper.html; arXiv:2412.12392); Teed, Lipson & Deng, "Deep Patch Visual Odometry", NeurIPS 2023 (edge-rung relevant).
  - Lie-group reference implementations for parity: Sophus, manif (links above).
- **Why it matters:** The reviewer is told to "say which reference you used"; without these it falls back on memory for exactly the methods rung 0 and milestones 2-5 implement.
- **Suggested fix:** Add the above, grouped by rung; keep the existing list.

##### [note] Role definition: checklist gaps worth adding
- **Where:** `.claude/agents/algorithm-reviewer.md:36-58`
- **Issue:** The checklist is well chosen (frames, left/right perturbation, Hamilton/JPL, Hartley normalisation, gauge freedom, degeneracies). A few standard traps are missing: an explicit parity tolerance policy (rtol/atol — see the first finding); quaternion double cover (`q` ≡ `-q` in parity comparisons); angle wrapping in `log` (keep within [0, π]); cheirality (points in front of both cameras) when disambiguating the four E-decompositions and in triangulation; left/right Jacobians of SO(3)/SE(3) and their small-angle forms (Barfoot; Solà §§ on Jacobians); seeded RNGs for RANSAC tests; and which alignment ATE uses (SE(3) vs Sim(3)) per rung.
- **Suggested fix:** Add as one-line bullets under the existing headings.

##### [note] Placeholder scope
- **Where:** `src/slamlab/so3.py:1-5`, `cpp/include/slamlab/so3.hpp:4-5`, `rust/slamlab-core/src/so3.rs:1-4`
- **Issue:** No `vee`, `exp`, `log` or Jacobians yet, and no design note in `docs/notes/`. All three files mark themselves as placeholders, so this is expected scope, not a defect.

#### Checked and fine
- `hat` produces the identical matrix in all three languages, row-major `[[0,-z,y],[z,0,-x],[-y,x,0]]`, matching the cross-product convention `hat(v) u = v × u` used by Barfoot (State Estimation for Robotics, `^` operator) and Solà et al. (arXiv:1812.01537); the Python doctest and Rust doc-test agree.
- Storage convention (`Mat3` as array of rows, row-major) is documented consistently in C++ and Rust and survives the nested-list return to numpy without transposition.
- Shape rejection is consistent across backends via the shared `as_vec3`, including `(3,1)` and `(1,3)` input (not mistakenly flattened).
- Integer and int64 input are correctly promoted to float64; large int (2**63) converts without overflow; float32 input is widened to float64 before computing.
- `-0.0` entries for zero components are harmless; IEEE NaN/inf propagate identically in all three languages (no backend divergence).
- `hat` is exact (negation and copy only — no rounding), so float64 is sufficient and no conditioning concerns apply.
- JSON fixtures round-trip doubles exactly (Python's `json` writes `repr`), so the fixture format is fit for later float-sensitive cases.
- Existing citations in the role definition are accurate (Solà et al. 2018; Barfoot CUP; Triggs et al. Vision Algorithms 1999; Forster et al. T-RO 2017; Teed & Deng DROID-SLAM NeurIPS 2021; Li et al. MegaSaM CVPR 2025), and the `third_party/` reference path exists.

## Run 2 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/review-panel` |
| **Range** | `874db0f...352fa25` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | general-purpose agents carrying each `.claude/agents/*.md` role and model |
| **Layer 1** | all 15 checks passed, locally and in a freshly built env (`setup_slamlab.sh --unlocked`, then `check.sh`) |
| **Process note** | All three reviewers stayed read-only; none moved `HEAD`. |

**Run 1 status:** all 31 findings resolved, except A7 (partially resolved: see A12) and L5
(partially resolved, as decided: the hard exp/log cases arrive with exp/log). No must-fix
findings in run 2.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A10 | should-fix | architecture | `so3.py`, both bindings | Bindings marshal through nested Python lists; on batched data this dominates C++/Rust cost and skews the three-language benchmark | fix: float64 ndarrays at the binding boundary, spans/slices in the cores | fix: float64 ndarrays at the binding boundary -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A11 | should-fix | architecture | `cpp/bindings/module.cpp`, `rust/slamlab-py/src/lib.rs` | One central binding file per extension that every rung will edit | fix: one binding file per module, a thin root that wires them | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A12 | should-fix | architecture | `tests/foundations/test_so3.py` | Protocol check is per-module, its parser breaks on common types, and nothing proves every extension submodule is covered (with H17) | fix: one generic test walking every extension submodule, `ast`-parsed signatures, public Protocol name | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| H17 | should-fix | hygiene | `tests/foundations/test_so3.py` (`_parameter_names`) | Regex signature parser breaks on subscripted/nested types (with A12) | fix with A12 | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A13 | should-fix | architecture | `CMakeLists.txt`, `rust/slamlab-core` | Ladder layering enforced in Python only; C++/Rust trees can import across rungs unchecked | fix now: per-rung C++ library and Rust crate, plus a layer-1 include check | fix now: per-rung crate and library plus a layer check -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A14 | should-fix | architecture | `pyproject.toml`, `learned/__init__.py` | `learned` may import `edge` and `classic`, contradicting its docstring | fix: forbidden contract | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| H15 | should-fix | hygiene | `tests/test_backends.py` | One test still skips via `pytest.skip`, bypassing the required-backends guard | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| H16 | should-fix | hygiene | fixture, `test_so3.py` | Fixture tolerance policy declared but never applied; "bit-exact" ignores signed zeros (with L10, L11) | fix: shared `assert_matches_fixture` applying the policy, sign bits checked when exact | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| L10 | note | algorithm | `test_so3.py` | Exact comparison ignores signed zeros (with H16) | fix with H16 | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| L11 | note | algorithm | `test_so3.py`, `conftest.py` | Tolerance policy declared, not consumed (with H16) | fix with H16 | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A15 | note | architecture | `pyproject.toml`, design doc | Layers contract not exhaustive (a new top-level package needs no place); no home yet for `SlamInput`/`SlamResult` and the depth-provider Protocol | fix: exhaustive contract; record the interfaces' home | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A16 | note | architecture | `pyproject.toml` | CPU-rung torch ban does not see through third-party packages (lietorch, pypose, kornia, theseus) | fix: forbid them by name; torch only ever in an extra | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| A17 | note | architecture | C++/Rust cores | Cross-language error contract for the standalone cores is undecided | owner decision | decided: C++23 std::expected / Rust Result with one shared error-kind list; Python ValueError subclasses in slamlab.foundations.errors -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| L9 | note | algorithm | `test_so3.py` | `CROSS_TOLERANCE_ULPS` is right but its docstring gives the wrong reason (FMA, not order) | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| L12 | note | algorithm | `arrays.py` | Masked arrays pass through with their masked values | fix: reject masked arrays | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| L13 | note | algorithm | `algorithm-reviewer.md` | ORB-SLAM3 is Campos et al. (T-RO 2021); log at exactly pi has a sign-ambiguous axis | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| H18 | note | hygiene | `setup_slamlab.sh` | `--env` with no value dies obscurely (the H7 defect, in the other script) | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |
| H19 | note | hygiene | this report | Run 1 resolutions still read "pending implementation" | fix | fixed when run 2 was recorded |
| H20 | note | hygiene | tests | No test for an unknown backend through `so3.hat`, or for the locator's non-module branch | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (51c9483) |

### Hygiene review

**Scope:** `874db0f...352fa25` on `feature/review-panel` (59 files). Read in full: `src/slamlab/{__init__,backends}.py`, `foundations/{__init__,arrays,so3}.py`, `tests/{conftest,test_backends}.py`, `tests/foundations/test_so3.py`, the hat fixture, `scripts/check.sh`, `scripts/setup_slamlab.sh`, `.github/workflows/test.yml`, `pyproject.toml`, `CMakeLists.txt`, `Doxyfile`, `.jscpd.json`, `.clang-tidy`, C++/Rust sources/tests/bindings, `CLAUDE.md`, `SKILL.md`, PR template, CHANGELOG, design-doc diff, algorithm-reviewer grep targets. Ran `pytest` in the `slamlab` env (70 passed — matches CHANGELOG's 70) and probed signed zeros / unknown-backend handling. `hat` treated as the marked placeholder. No HEAD-moving or worktree-touching git commands were run.

**Summary:** All 14 run-1 findings are resolved, and the fixes are of good quality — the package-per-rung restructure is clean, validation and dispatch sit beside the algorithm, `backends` is a pure locator, and the tests are strong (exact fixtures, basis vectors, a Protocol check, a required-backends guard). No must-fix items. Remaining issues are small: one test path that can still skip silently, a fixture claim its tests don't enforce, a declared-but-unused tolerance policy, a regex that will break on richer signatures, and the run-1 report's resolutions still reading "pending".

#### Run 1 findings

- **H1 `_import` swallows ImportError — resolved.** `backends._extension` (`backends.py:68-74`) returns `None` only for `ModuleNotFoundError` whose `.name` is the extension itself, re-raising anything else; `test_broken_extension_is_not_hidden` covers a broken extension and a missing dependency and asserts the original exception object propagates.
- **H2 Parity tests skip silently in CI — resolved (one gap, see S1).** `conftest.require_backend` fails instead of skipping when the backend is in `SLAMLAB_REQUIRE_BACKENDS`; `check.sh:35` sets it to all three; `test_required_backends_are_available`/`_are_known` guard it; `setup_slamlab.sh` also asserts `found == list(backends.BACKENDS)`.
- **H3 `Backend.hat` docstring — resolved.** Class gone; `foundations.so3.hat` has full Parameters/Returns/Raises/Examples; doctest runs.
- **H4 "three editable installs" — resolved.** Now "two editable installs", listing two.
- **H5 `dev` extra drift — resolved.** Extra now includes `mypy`, `import-linter`, `scikit-build-core`, `nanobind`; the unlocked branch of `setup_slamlab.sh` installs it via `tomllib` — one list.
- **H6 CI hardening — resolved.** Miniforge pinned with sha256 check; `permissions: contents: read`; `timeout-minutes: 30`. (Checksum value not verifiable offline; the note says layer 1 passes in a fresh env, which covers it.)
- **H7 `check.sh` args — resolved.** `--env` without a value and unknown args both `exit 2` with a message. (`setup_slamlab.sh` still has the old `$2` pattern — see N1.)
- **H8 SKILL.md wording — resolved.** Agent-availability sentence now unambiguous; `HEAD_SHA`/`BASE_SHA`; dirty-tree check names `git status --porcelain`.
- **H9 Unnamed `1e-12` — resolved.** Fixture comparison exact in all three languages; native C++/Rust tests compare exactly; the only tolerance left is the named `CROSS_TOLERANCE_ULPS`, justified in its docstring and scaled by `eps` and term magnitude.
- **H10 Raw-extension boundary — resolved.** `hat` and `arrays.py` state validation happens once in the Python entry point; the C++/Rust docs state validation is the caller's job; `test_compiled_module_matches_the_protocol` now checks the extension surface.
- **H11 Finiteness in `as_vec3` — resolved.** Rejects non-finite, bool, complex, str, None; `BAD_INPUTS` tests ten cases across every backend.
- **H12 Backend list repeated / `so3` imported two ways — resolved.** Single `BACKENDS` tuple and `_EXTENSIONS` dict; the "python" import branch is gone.
- **H13 CLAUDE.md merge rule — resolved.** `CLAUDE.md` cites the owner's 2026-10-02 grant; the owner's global Claude Code instructions does contain "The one exception: slamlab", and the hook the owner's merge-guard hook exists.
- **H14 Smaller items — resolved.** Doxygen XML-only; PR template report link absolute; compiler names from `uname -m`; SPDX license string.

#### Findings

##### [should-fix] A required-backend test path can still skip silently
- **Where:** `tests/test_backends.py:48-54` (`test_missing_implementation_is_reported`)
- **Issue:** Uses `if name not in backends.available(): pytest.skip(...)` instead of `require_backend(name)`, so it bypasses the `SLAMLAB_REQUIRE_BACKENDS` guard.
- **Why it matters:** `test_required_backends_are_available` would still fail in CI, so nothing is lost today — but it's two divergent skip mechanisms, and the next test copied from this one is unguarded.
- **Evidence:** `pytest.skip(f"{name} backend not built")` here vs `require_backend` in `tests/conftest.py:26-44`.
- **Suggested fix:** `require_backend(name)` and pass its result to `backends.implementation`.

##### [should-fix] Fixture tolerance policy is declared but never applied, and "bit-exact" is not enforced
- **Where:** `tests/fixtures/foundations/so3_hat.json` (`description`, `tolerance`); `tests/foundations/test_so3.py:23-34`
- **Issue:** The description says expected values are "bit-exact" and cases include `-0.0`, but `test_hat_matches_fixture_exactly` hardcodes `assert_array_equal`, which treats `-0.0 == 0.0`. The `tolerance` block (`rtol`/`atol`/`reason`) is never read for comparison; `test_fixture_declares_exact_comparison` only asserts the block is internally consistent.
- **Why it matters:** The policy is stated for later rungs to copy, but nothing consumes it — a future fixture could set `rtol=1e-9` and the parity test would silently still compare exactly (or vice versa). A backend returning `+0.0` where `-0.0` is expected passes despite "bit-exact".
- **Evidence:** `np.testing.assert_array_equal(np.array([0.0]), np.array([-0.0]))` passes; `test_so3.py:25` asserts `HAT["tolerance"] == {**HAT["tolerance"], "comparison": "exact", "rtol": 0.0, "atol": 0.0}` — it only checks the declared policy is exact.
- **Suggested fix:** A `conftest` helper `assert_matches_fixture(got, expected, tolerance)` that applies `rtol`/`atol` from the fixture, used by the parity test; replace the dict-merge assertion with direct comparisons; either compare signed zeros (`np.signbit`) or soften "bit-exact" to "exact".

##### [should-fix] `_parameter_names` parses signatures with a regex that breaks on common types
- **Where:** `tests/foundations/test_so3.py:112-123`
- **Issue:** `re.search(r"\((.*?)\)", text)` stops at the first `)` and splits on every `,`. Works for `hat(v: collections.abc.Sequence[float])`, but a signature like `def f(a: tuple[float, float], b: Annotated[list[float], FixedSize(3)])` yields spurious "parameters" (`float]`, `FixedSize(3`).
- **Why it matters:** This is the test every later rung's Protocol check relies on; it will fail confusingly (or pass/fail wrongly) as soon as a second bound function has a subscripted or nested type — nanobind renders `std::array` with `array.h` as `Annotated[...]`.
- **Suggested fix:** Parse the nanobind signature with `ast.parse(text + ": ...")`; for PyO3, prefer `inspect.signature` on `__text_signature__`.

##### [note] `setup_slamlab.sh` still dies obscurely on `--env` with no value
- **Where:** `scripts/setup_slamlab.sh:31` (`--env) ENV_NAME="$2"; shift 2`)
- **Issue:** Under `set -u` this gives a bare "unbound variable" — the same defect H7 fixed in `check.sh`. Unknown args are handled correctly.
- **Suggested fix:** Same `[[ $# -ge 2 && -n "$2" ]]` guard.

##### [note] Run 1 table and PR record still say "pending implementation"
- **Where:** `docs/reviews/2026-10-02-feature-review-panel.md`, Resolution column (31 rows); `SKILL.md` step 6
- **Issue:** All 31 resolutions read "…pending implementation". Step 6 requires `fixed (<commit>)` plus "decided by … on <date>" after fixes land, and there's no Run 2 section yet.
- **Why it matters:** The report ships in the PR as the decision record; as written it contradicts the CHANGELOG ("owner chose to resolve all of them here").
- **Suggested fix:** Update rows to `fixed (352fa25)` (plus `no action` for L8, the "hard cases land with exp/log" wording for L5) and append the Run 2 section when this review is recorded.

##### [note] Smaller dispatch-layer test gaps
- **Where:** `tests/foundations/test_so3.py`, `tests/test_backends.py`
- **Issue:** No test that `so3.hat(..., backend="fortran")` raises `ValueError` (verified it does: "unknown backend 'fortran'; expected one of python, cpp, rust"), only `backends.check_name` directly. `backends.implementation` is untested for a name that resolves to a non-module attribute (e.g. `"slamlab.foundations.so3.hat"`) — the `isinstance(found, ModuleType)` branch at `backends.py:136`. `BackendNotBuiltError` isn't tested through `hat` itself.
- **Suggested fix:** One parametrised test for unknown backend via `so3.hat`; one for the non-module-attribute branch.

#### Checked and fine
- `backends.py`: small, single-purpose, well documented, consistent with the design doc; `BackendNotBuiltError` subclasses `ImportError`; error messages name the bad value; module-name prefix guard rejects non-slamlab modules.
- `arrays.as_vec3`: rejects non-numeric, wrong shape, non-finite with messages naming the offender; dtype-kind handling explicit and documented with a named `_NUMERIC_KINDS` constant.
- `foundations/so3.hat`: validates once then dispatches; Protocol typed with `cast`; docstring has shapes, dtype, the cross identity, and a working doctest.
- Cross-language implementations: C++ Doxygen, Rust rustdoc and Python docstring agree on row-major layout and mark the placeholder; native tests pin all nine entries with basis vectors, check skew-symmetry and the cross identity exactly; the Rust test lint-allow carries a reason; the Rust binding docstring now correctly distinguishes `ValueError`/`TypeError`.
- Build/config: CMake globs recurse; test executables named from path (`foundations_test_so3`); `check.sh` file lists via `git ls-files` (incl. untracked-unignored) so new rung dirs are picked up; import-linter contracts (layers with independent siblings, VI-only-on-edge, no torch on CPU rungs, no depth models, never reifyscene) are commented and cover whole packages; pytest markers registered with `--strict-markers`.
- CI/setup: `setup_slamlab.sh` fails unless all three backends load; `check.sh` exports `SLAMLAB_REQUIRE_BACKENDS` for every run; workflow is read-only, time-boxed, checksum-pinned.
- Process docs: CHANGELOG matches the diff (70 tests collected/passed); VERSION bumped to 0.1.2.0; stub packages each carry rung, milestone and budget; design-doc status line and Package layout section agree with the code; algorithm-reviewer reference list (Nistér, Hartley, Umeyama, MASt3R-SLAM, …) and checklist additions (double cover, tolerance policy) are present; `.gitignore` still versions agents/skills.
- Duplication: the shared `as_vec3` and the single `BACKENDS` constant remove the earlier repeated lists; no same-language duplicate logic found beyond what jscpd covers.

### Architecture review

**Scope:** `874db0f...352fa25` on `feature/review-panel` (Run 2). Read all of `src/slamlab/**`, `tests/**`, `pyproject.toml`, `CMakeLists.txt`, `cpp/**`, `rust/**`, `scripts/check.sh`, `scripts/setup_slamlab.sh`, `.github/workflows/test.yml`, `CLAUDE.md`, the updated design doc, and the Run 1 report. Ran `lint-imports` on HEAD (5 kept), two import-linter probes on a scratchpad copy of the package (an unplaced top-level module; a torch-wrapping third-party package), and inspected the live binding signatures (`__nb_signature__`, `__text_signature__`) in the `slamlab` env.

**Summary:** The restructure is sound and fixes the Run 1 problems at the root: rungs are packages, the import-linter contracts cover whole packages (including indirect chains — verified by probe), `backends` is a pure locator at the bottom layer, dispatch and validation sit beside the algorithm, and the extension submodules mirror the tree. Every Run 1 finding is resolved except A7, which is only partly resolved. The new issues are about what happens as the ladder grows: the layering is enforced in Python only (not C++/Rust); the binding entry points are still single central files; the nested-list binding convention will distort the cross-language speed comparison and cost O(N) Python objects per call on batched data; and the Protocol check is hand-written per module, its signature parser breaks on common binding types, and nothing proves every extension submodule is covered. No must-fix.

#### Run 1 findings

- **A1 (must-fix, CI backends): resolved.** `scripts/check.sh:35` exports `SLAMLAB_REQUIRE_BACKENDS="python,cpp,rust"`; `tests/conftest.py:require_backend` fails instead of skipping when required; `backends._extension` (`backends.py:68-74`) returns `None` only for `ModuleNotFoundError` with `exc.name == module_name` and re-raises everything else; `tests/test_backends.py` covers broken-extension and missing-dependency cases.
- **A2 (layers contract names one module): resolved.** One package per rung, with layers, independence, vi-only-on-edge, no-torch-on-CPU, no-depth-model and no-reifyscene contracts (`pyproject.toml:92-129`). Probe confirmed indirect chains are caught (`slamlab.learned.x -> slamlab.providers -> slamlab.orchestration` reported BROKEN). Residual gaps: N5, N6.
- **A3 (`Backend` facade): resolved.** `backends.py` only locates (`implementation(backend, module_name)`); dispatch lives in `foundations/so3.py:70-82`; compiled submodules mirror the tree (`slamlab._cpp.foundations.so3`, `slamlab_rs.foundations.so3`). Binding *source* files are still central — N2.
- **A4 (`as_vec3` placement): resolved.** Now `src/slamlab/foundations/arrays.py`.
- **A5 (flat C++ globs): resolved.** `check.sh:51-52` uses `git ls-files` pathspecs (which match across `/`; verified they list `cpp/src/foundations/so3.cpp`); `CMakeLists.txt:13,30` uses `GLOB_RECURSE`.
- **A6 (GPU/budget split): resolved as scoped.** `gpu`/`budget`/`slow` markers declared under `--strict-markers` (`pyproject.toml:50-57`); `check.sh:77` runs `-m "not gpu and not budget"`; the design doc now states where a rung declares its budget and lists the light edge env and opt-in scripts as open items for milestones 5/7.
- **A7 (no stubs): partially resolved.** Per-algorithm Protocol, the mypy override removed, and `test_compiled_module_matches_the_protocol` exists — but the check is hand-written per module, compares parameter names only, uses a parser that breaks on realistic binding signatures, and has no completeness check. See N3.
- **A8 (stale status line): resolved.** Design doc now "the scaffold is built".
- **A9 (merge rule conflict): resolved.** `CLAUDE.md` cites the grant; the owner's global Claude Code instructions contains "The one exception: slamlab"; the owner's merge-guard hook exists.

#### Findings

##### [should-fix] N1: Bindings marshal through nested Python lists — this will dominate C++/Rust cost on batched data and skew the three-language benchmark
- **Where:** `src/slamlab/foundations/so3.py:26` (Protocol docstring), `:82`; `cpp/bindings/module.cpp:6`; `rust/slamlab-py/src/lib.rs` (`fn hat(v: [f64; 3]) -> [[f64; 3]; 3]`)
- **Issue:** The convention set here is "Bindings take and return plain nested lists (no numpy dependency in the cores)", so dispatch does `vec.tolist()` → `np.asarray(...)` per call. The rationale conflates two layers: the *cores* can stay numpy-free while the *bindings* use zero-copy buffers (nanobind `nb::ndarray`; `numpy`/`PyBuffer` in PyO3) and hand `std::span<const double>` / `&[f64]` to the cores.
- **Why it matters:** Milestone 1 brings batched operations (project N points, triangulation, RANSAC scoring, BA residuals). With lists, each call allocates O(N) Python float objects in both directions, so the design doc's "same algorithm, three languages" comparison of speed and memory would measure marshalling, not the algorithm, and the edge rung's real-time target would pay it per frame. `hat` is the pattern every later binding will copy.
- **Evidence:** `so3.py:82` `return np.asarray(impl.hat(vec.tolist()), dtype=np.float64)`; live nanobind signature `def hat(v: collections.abc.Sequence[float]) -> list[list[float]]`.
- **Suggested fix:** Decide the boundary type now and record it in the design doc's "Packaging" section: float64 C-contiguous ndarrays in/out of the bindings, spans/slices in the cores. Switching `hat` now is cheap and sets the precedent; the Protocol then types `NDArray[np.float64]`.

##### [should-fix] N2: Each extension's binding entry point is one central file every rung will edit
- **Where:** `cpp/bindings/module.cpp:14-20` (all submodules defined in one `NB_MODULE`); `rust/slamlab-py/src/lib.rs:11-42` (nested inline `#[pymodule] mod` blocks)
- **Issue:** The Python-side facade was removed (A3), but its compiled counterpart remains: every new bound module in every rung adds includes and `def_submodule` calls to `module.cpp` and nested modules to `lib.rs`. The C++ binding TU will include headers from every rung.
- **Why it matters:** The same central growing hotspot and merge-conflict point A3 was about, one layer down; it also blurs the per-rung mirroring the design doc promises.
- **Evidence:** `module.cpp:16-19` `foundations` / `so3` defined inline; `lib.rs` nests `mod foundations { mod so3 { ... } }` in one file.
- **Suggested fix:** Mirror bindings per rung/module — `cpp/bindings/foundations/so3.cpp` exposing `void bind_so3(nb::module_&)`, with `module.cpp` only creating rung submodules and calling the `bind_*` functions (bindings globbed recursively like the core); in Rust, one file per module (`src/foundations/so3.rs` with its own `#[pymodule]`) wired by `#[pymodule_export]`. Do it before the second bound module.

##### [should-fix] N3: The Protocol check is per-module, its parser breaks on common binding types, and nothing proves coverage
- **Where:** `tests/foundations/test_so3.py:93-123` (`test_compiled_module_matches_the_protocol`, `_parameter_names`); design doc "Dispatch" ("A test checks every extension submodule against that Protocol")
- **Issue:** (1) The test and its parser live in `test_so3.py`, so every later module copies them (jscpd will flag that). (2) `_parameter_names` takes the first `(...)` with a non-greedy regex and splits on `,`, so any parameter type containing a comma or parenthesis breaks it — including `tuple[float, float]` and nanobind's ndarray annotations, which N1 would introduce. (3) Nothing checks that every extension submodule has a Python counterpart with a Protocol, or that every Protocol is checked — an orphan or unchecked submodule passes silently, so the design doc's "every extension submodule" is not yet true by construction.
- **Why it matters:** This test is what replaced type stubs (A7); if it silently stops covering bindings, signature drift is caught only when a parity test happens to exercise the changed parameter.
- **Evidence:** Feeding the test's regex logic plausible signatures (scratchpad `parse.py`): `def project(T: tuple[float, float, float], p: list[float])` → `['T', 'float', 'float]', 'p']`; `def project(pts: typing.Annotated[NDArray[numpy.float64], dict(shape=(None, 3))], k: float)` → `['pts', 'dict(shape', '3']`.
- **Suggested fix:** Replace the per-module test with one generic test in `tests/test_backends.py`: for each compiled backend, walk the extension's submodule tree; for each leaf, `slamlab.<path>` must import and define the conventional Protocol (give it a documented, public-in-convention name rather than a private `_Implementation` reached into from tests); check every Protocol member exists with matching parameter names, parsing with `ast.parse(text)` on the `def ...` signature rather than a regex. Optionally compare annotations too, now that both bindings expose them.

##### [should-fix] N4: The ladder's layering is enforced in Python only; the C++ and Rust trees mirror it but nothing checks it
- **Where:** `CMakeLists.txt:13-14` (one `slamlab_core` library globbing all of `cpp/src`, one include root `cpp/include`); `rust/slamlab-core` (one crate, `pub mod foundations;`); design doc "Package layout" ("so a module's place in the ladder is visible from its path and enforceable by contract")
- **Issue:** `cpp/src/foundations/x.cpp` can `#include "slamlab/edge/..."`, and `slamlab_core::foundations` can `use crate::edge::...`, with no check failing under the current layout. import-linter only sees Python. A single C++ library also means an edge-budget/ARM build (milestone 7) compiles every rung.
- **Why it matters:** C++ is the performance core for rung 0 and edge, and Rust covers rung 0 and edge — exactly the rungs whose independence matters for edge budgets are the ones unguarded. The design doc's "enforceable by contract" currently holds for one language of three.
- **Evidence:** `CMakeLists.txt:13` `file(GLOB_RECURSE SLAMLAB_CORE_SOURCES CONFIGURE_DEPENDS cpp/src/*.cpp)` into one `add_library(slamlab_core ...)`; `rust/slamlab-core/src/lib.rs` single crate root.
- **Suggested fix:** Before milestone 2's first edge code in C++/Rust: Rust — one crate per rung (`slamlab-foundations`, `slamlab-edge` depending on it), so cargo enforces direction; C++ — one static library per rung (`slamlab_foundations`, `slamlab_edge` linking it) plus a small layer-1 check that parses `#include "slamlab/<rung>/..."` and rejects includes outside the rung's allowed set, sharing the layer list with `pyproject.toml`. Alternatively, if deferred, the design doc should say enforcement is Python-only for now.

##### [should-fix] N5: `learned` may import `edge` and `classic`; the contracts do not match its stated dependencies
- **Where:** `pyproject.toml:95-108`; `src/slamlab/learned/__init__.py:6-7`
- **Issue:** `learned` sits in the layer above `dense | edge | classic`, so it may import all three; only `vi` has a narrowing forbidden contract. The `learned` docstring says "Builds on `slamlab.dense` and `slamlab.foundations`", and the ladder diagram shows learned descending from dense only.
- **Why it matters:** The same asymmetry the vi contract fixes, left open on the GPU branch; a docstring claim no contract checks is exactly the drift the contracts exist to stop.
- **Evidence:** vi contract `source_modules = ["slamlab.vi"]`, `forbidden_modules = ["slamlab.dense", "slamlab.classic"]`; no counterpart for `slamlab.learned`.
- **Suggested fix:** Add a forbidden contract: `source = slamlab.learned`, `forbidden = slamlab.edge, slamlab.classic`. (If learned is meant to reuse classic loop closure later, change the docstring and design doc instead and say so.)

##### [note] N6: The layers contract is not exhaustive, so a new top-level package needs no declared place; the shared interface types have no home yet
- **Where:** `pyproject.toml:92-102`; design doc "Boundary with reifyscene" (`SlamInput`/`SlamResult`) and "Non-goals" (depth provider interface)
- **Issue:** Unlisted top-level modules are unconstrained in their own right (probe: a new `slamlab/providers.py` importing `torch`, `slamlab.learned` and `slamlab.orchestration` passed all five contracts until something below it imported it). `SlamInput`/`SlamResult` (used by every rung) and the depth-provider Protocol (needed by both `edge` RGB-D mode and `dense`, which are independent siblings) must live in `foundations` or a new package at or below it.
- **Suggested fix:** Use `containers = ["slamlab"]` with `exhaustive = true` (supported by the installed import-linter 2.15; requires containers and relative layer names) so a new package fails layer 1 until it is placed; record in the design doc where the interface types and the provider Protocol live (e.g. `slamlab.foundations.interfaces`).

##### [note] N7: The CPU-rung torch ban does not see through third-party packages; the env is today's real guard
- **Where:** `pyproject.toml:110-114`
- **Issue:** import-linter treats external packages as leaves. Probe: `slamlab/foundations/lie.py` importing a stand-in `lietorch` whose `__init__` imports `torch` → the CPU-rungs contract reported KEPT. It's covered today only because torch is absent from the env and `dependencies = ["numpy>=1.26"]`; it opens when milestone 5 adds torch to the single env. Torch-backed Lie-group and geometry libraries (lietorch, pypose, kornia, theseus) are exactly what rung-0 work is tempted to import.
- **Suggested fix:** Add those package names to the forbidden list now; when torch arrives, put it in an optional extra (never core `dependencies`); keep the planned light edge env as the actual guarantee. Also note `slamlab.eval` is meant to host the budget harness that runs edge rungs in that light env (design doc "Compute budgets"), yet `eval` sits above `learned` with no torch restriction — either restrict torch to an `eval` subpackage or keep eval's rung imports lazy.

##### [note] N8: Validation exists only at the Python entry point; the standalone cores' error contract is undecided
- **Where:** `cpp/include/slamlab/foundations/so3.hpp` (`hat` is `noexcept`, "Input validation (finiteness) is the caller's job"); `rust/slamlab-core/src/foundations/so3.rs:20-21`
- **Issue:** The design doc says the cores build standalone for edge and possible phone targets — there the core API *is* the public boundary, and it validates nothing. For `hat`, NaN-in → NaN-out is harmless, but milestone-1 functions that can genuinely fail (log near pi, degenerate eight-point, singular PnP) need a declared cross-language error contract (C++ exceptions or `std::expected`; Rust `Result`), mapped by the bindings to the same Python exception the reference raises.
- **Suggested fix:** Decide this at the start of milestone 1 and record it in the design doc's Languages section, alongside the parity-fixture tolerance policy.

#### Checked and fine
- `slamlab.backends` imports nothing from slamlab, sits at the bottom layer, is in the CPU contract, and knows nothing about what implementations compute; the `"python"` and non-slamlab-module errors are explicit and tested.
- `import-linter` passes on HEAD (5 kept); indirect chains through unlisted modules are caught (probe output: `slamlab.learned is not allowed to import slamlab.orchestration`).
- Layer order (orchestration > eval > learned|vi > dense|edge|classic > foundations > backends) matches the design doc; vi/learned independence and vi-on-edge-only are correctly expressed.
- Package docstrings state rung, milestone and budget as the design doc requires; stubs are clearly marked "Stub: no code yet"; `hat` stays an explicitly marked placeholder in all three languages.
- Standalone cores: nanobind only under `SKBUILD`; `slamlab-core` has no dependencies; Rust tests run `-p slamlab-core` alone; bindings contain no algorithm logic.
- Parity fixture lives at `tests/fixtures/foundations/so3_hat.json` with an explicit tolerance policy, a test pins the policy, and every backend is compared bit-exactly.
- Tests mirror the packages (`tests/foundations/`, `cpp/tests/foundations/`); CMake derives per-rung test names so same-named files in different rungs don't collide.
- The Rust subset is respected (rung 0 only); `slamlab_rs` stays top-level as the design doc requires.
- No import-time side effects or globals added; `SLAMLAB_REQUIRE_BACKENDS` is an explicit, test-only setting.
- No compatibility shims: the old flat `slamlab.so3`, `cpp/src/so3.cpp` and `rust/.../so3.rs` were removed outright.
- CI and local runs share one `check.sh`; source lists derive from git so new rung directories need no edits.

### Algorithm review

**Scope:** `874db0f...352fa25` (branch `feature/review-panel`), focusing on the changes in commit `352fa25`: `src/slamlab/foundations/arrays.py` (`as_vec3`), `src/slamlab/foundations/so3.py` (`hat` + dispatch), `tests/foundations/test_so3.py`, `tests/fixtures/foundations/so3_hat.json`, `cpp/{include,src,tests}/.../foundations/so3.*`, `rust/slamlab-core/src/foundations/so3.rs`, the bindings (`cpp/bindings/module.cpp`, `rust/slamlab-py/src/lib.rs`), `src/slamlab/backends.py`, `tests/conftest.py`, and the updated `.claude/agents/algorithm-reviewer.md`. Read against Run 1 in `docs/reviews/2026-10-02-feature-review-panel.md`. Re-ran the Run-1 float32 probe and invalid-input probe against all three built backends (python, cpp, rust) with scratchpad scripts only; `pytest tests/foundations` with `SLAMLAB_REQUIRE_BACKENDS=python,cpp,rust` → 58 passed.

**Summary:** Every algorithm finding from Run 1 is resolved: both probes now fail where they should (the exact parity test catches a float32 backend; every non-numeric/non-finite input is rejected identically on all three backends). The new cross-product tolerance (4 ulp scaled by max|v|·max|u|) is sound, though its docstring gives the wrong reason for it. The added citations are accurate. No new must-fix or should-fix issues — only notes.

#### Run 1 findings
- **L1 (float32 regression invisible) — resolved.** The parity test now uses `assert_array_equal`; the fixture declares `comparison: exact, rtol: 0, atol: 0` with a reason and adds 0.1, 1/3, 1e±300, 2.5e-8, 1/7, -0.0. Probe with a simulated float32 backend: the exact fixture test now catches 3 of 6 cases (two fail comparison; the 1e±300 case overflows to inf) and the cross test catches it in 100/100 cases; under the old default-`rtol` assertion only the 1e±300 case would have been caught.
- **L2 (`as_vec3` accepted None/str/bool/complex/non-finite) — resolved.** `None`, `"1"`, bool, complex, NaN, ±inf, object arrays of floats, `Decimal`, `Fraction`, Python ints that overflow int64 (object dtype), `timedelta64`, a scalar, and shapes (2,), (1,3), (3,1) all raise `ValueError` identically on python, cpp and rust. A longdouble 1e400 casts to inf and is caught by the finiteness check.
- **L3 (Rust docstring named the wrong exception) — resolved.** `rust/slamlab-py/src/lib.rs:224-226` now says `ValueError` for wrong length, `TypeError` for non-numeric elements.
- **L4 (native tests did not pin the matrix) — resolved.** `cpp/tests/foundations/test_so3.cpp:79-100` and `rust/slamlab-core/src/foundations/so3.rs:161-194` check all three basis vectors (all nine entries), skew-symmetry, and the cross product, all compared exactly — and exact is valid there because with v=(1,-2,3), u=(0.5,4,-1.5) every product and sum is representable, so no rounding occurs. Python adds the basis-vector test and a 100-sample seeded skew-symmetry and cross test.
- **L5 (rung-0 fixtures need the hard cases) — partially resolved, as decided.** Tolerance policy and fixture format are in place now; the near-0 / near-pi / wrap cases are deferred to milestone 1 with `exp`/`log`.
- **L6 (reference list) — resolved.** All suggested references added, grouped by rung (accuracy checked below).
- **L7 (checklist gaps) — resolved.** Double cover, log in [0, π], small-angle Jacobians, cheirality, tolerance policy with explicit `rtol`, float32-inexact values, seeded RNGs, and ATE alignment are all now in the checklist.
- **L8 (placeholder scope) — no action, as decided.** Still correctly marked placeholder in all three languages.

#### Findings

##### [note] `CROSS_TOLERANCE_ULPS` is sound, but its docstring gives the wrong reason
- **Where:** `tests/foundations/test_so3.py:18-20`
- **Issue:** The docstring says `hat(v) @ u` and `np.cross(v, u)` "sum the same products in a different order". Order cannot be the cause: each row has one exact zero term plus two products, and IEEE two-term addition is commutative, so summing the products without fused multiply-add, in any order, is bit-identical to `np.cross`. The actual difference is the BLAS matmul using FMA (one product left unrounded).
- **Why it matters:** The bound itself is fine: in first-order error analysis each method's error is ≤ 4u·M (u = eps/2, M = max|v|·max|u|; two rounded products of size ≤ M plus one rounded sum of size ≤ 2M, or 3u·M with FMA), so their difference is ≤ 8u·M = 4 eps·M — the constant is right. But the next person copying this pattern for `exp`/`log` will reason from the docstring, and "different order" is the wrong mental model.
- **Evidence:** Over 1e5 seeded samples, `h @ u` vs `np.cross` differ in 56,644 cases, max 1.85 eps·M (inside the 4 eps·M bound); a hand-written unfused `sum(h[i,j]*u[j])` vs `np.cross` differ in 0 of 2e4 cases.
- **Suggested fix:** Reword to e.g. "the BLAS matmul may fuse a multiply-add (FMA), so it rounds differently from `np.cross`; each is within 4u·max|v|·max|u|, hence 4 eps."

##### [note] Fixture records signed zeros, but the exact comparison ignores them
- **Where:** `tests/foundations/test_so3.py:32` (`np.testing.assert_array_equal`), `tests/fixtures/foundations/so3_hat.json` (`-0.0` entries)
- **Issue:** `assert_array_equal` treats `-0.0 == 0.0`, so "bit for bit" in the test docstring overstates what is checked. Today all three backends do match the fixture's sign bits (0 mismatches across all cases with `np.signbit`).
- **Why it matters:** Harmless for `hat`, but at milestone 1 `log` near π and anything built on `atan2` are sign-of-zero sensitive, and this exact-comparison pattern is the one that will be copied.
- **Suggested fix:** When the milestone-1 fixtures land, add a `np.signbit` equality check for `comparison: exact` (or reword the docstring to "equal values"). Carry into the milestone-1 plan.

##### [note] Fixture tolerance policy is declared, not consumed
- **Where:** `tests/foundations/test_so3.py:23-26`, `tests/conftest.py:46-61`
- **Issue:** The test asserts the policy is `exact` and then hard-codes `assert_array_equal`; nothing applies `comparison`/`rtol`/`atol` from the fixture generically. Correct for `hat`, but for milestone 1 a shared helper (e.g. `assert_matches_fixture(got, case, policy)` passing `rtol` explicitly and supporting per-case overrides) would keep the JSON policy and the assertion from drifting apart.
- **Suggested fix:** Defer to milestone 1 together with L5.

##### [note] `as_vec3` edge inputs still accepted (acceptable; worth documenting)
- **Where:** `src/slamlab/foundations/arrays.py:43-49`
- **Issue:** A numpy masked array passes through `np.asarray`, which silently uses the masked value (`np.ma.array([1,2,3], mask=[0,1,0])` → `[1,2,3]`); float16 is accepted (fine — widens exactly); integers above 2**53 lose precision silently when cast to float64. None is a correctness defect for a geometry library.
- **Suggested fix:** Optionally reject `np.ma.MaskedArray` or note the masked-array behaviour in the docstring. Low priority.

#### Checked and fine
- `hat` layout `[[0,-z,y],[z,0,-x],[-y,x,0]]` is identical in Python, C++ and Rust and matches `hat(v)u = v×u` (Barfoot's `^` operator; Solà, Deray & Atchuthan, arXiv:1812.01537); the doctest, the Rust doc-test and the basis-vector tests agree.
- Validation runs once in `so3.hat` before dispatch, so all three backends share one error surface; the probe shows identical messages on python, cpp and rust.
- The dtype-kind allow-list `"iuf"` correctly excludes bool, complex, object, string, datetime and timedelta; finiteness is checked after the float64 cast, so longdouble overflow is caught.
- Compiled results are converted back with `dtype=np.float64` and the test asserts the dtype, so a backend narrowing precision would be visible.
- The fixture's expected values are exact (copy and negate only), and JSON round-trips doubles via `repr`.
- Randomised test uses a seeded generator (`default_rng(0)`); the skew-symmetry check is exact.
- The native C++/Rust "exact" cross-product checks are valid: all products and sums for the chosen `v`, `u` are representable in binary64.
- Citations in `.claude/agents/algorithm-reviewer.md` are accurate: Hartley TPAMI 1997; Nistér TPAMI 2004; Fischler & Bolles CACM 1981; Lepetit, Moreno-Noguer & Fua IJCV 2009; Strasdat, Montiel & Davison RSS 2010; Umeyama TPAMI 1991; Sturm et al. IROS 2012; Gálvez-López & Tardós T-RO 2012; Teed, Lipson & Deng (DPVO) NeurIPS 2023; Forster et al. T-RO 2017; Teed & Deng (DROID-SLAM) NeurIPS 2021; Li et al. (MegaSaM) CVPR 2025; Murai, Dexheimer & Davison (MASt3R-SLAM) CVPR 2025, arXiv:2412.12392; and Wang et al., Shape of Motion, ICCV 2025, pp. 9660-9672, arXiv:2407.13764 — confirmed on [CVF open access](https://openaccess.thecvf.com/content/ICCV2025/html/Wang_Shape_of_Motion_4D_Reconstruction_from_a_Single_Video_ICCV_2025_paper.html) and [arXiv](https://arxiv.org/abs/2407.13764). One small imprecision: "ORB-SLAM papers (Mur-Artal et al.)" — correct for ORB-SLAM/ORB-SLAM2, but ORB-SLAM3 is Campos et al., T-RO 2021 (optional wording fix).
- The `third_party/` paths in the role definition exist in reifyscene (`shape-of-motion/preproc/DROID-SLAM`, `vggt`, `VGGT-Long`, `Hierarchical-Localization`).
- New checklist items are standard and correctly stated. For milestone 1, "log returns angles in [0, π]" should also mention that at exactly π the axis is ambiguous up to sign, so parity checks must accept ±ω there.

## Run 3 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/review-panel` |
| **Range** | `874db0f...966d820` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | general-purpose agents carrying each `.claude/agents/*.md` role and model |
| **Layer 1** | all 16 checks passed (`scripts/check.sh`) |
| **Process note** | All three reviewers stayed read-only. |

**Run 2 status:** all 19 findings resolved. **All three reviewers state that this first
version satisfies them.** No must-fix findings; two should-fix gaps in the new enforcement
tooling and four notes.

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| A18 | should-fix | architecture | `scripts/check_layers.py` | Misses `#include <slamlab/...>`, headers directly in `cpp/include/slamlab/`, Cargo dev/build/target dependency tables, renamed (`package =`) crates, and crates not named `slamlab-*` (all demonstrated by probe) | fix, with a fake-repo test per form | fix -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| A19 | should-fix | architecture | `scripts/check.sh` | Rust tests run for `slamlab-foundations` only; later rung crates would be skipped silently | fix: `cargo test --workspace --exclude slamlab-py` | fix -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| A20 | note | architecture | `tests/test_backends.py` | A misnamed top-level extension submodule is never visited by the Protocol test | fix: every top-level extension submodule must be a rung package | fix -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| A21 | note | architecture | `pyproject.toml` | Comment still names `_Implementation` | fix | fix -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| L14 | note | algorithm | `cpp/bindings/foundations/so3.cpp` | nanobind converts float32/strided input silently (Rust refuses), contradicting the docstring | fix: `.noconvert()` | fix -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| L15 | note | algorithm | design doc, error contract | `NotConverged` as a hard error discards the best estimate; standard solvers return the estimate plus a status | decide now in the design doc: iterative solvers return estimate + summary | decided: iterative solvers return the estimate plus a summary; NotConverged for strict mode or divergence (design doc) -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |
| H21 | note | hygiene | error contract | Binding error translation has no code or test yet (no fallible function exists) | record in the milestone-1 plan | recorded in the design doc's milestone-1 tasks -- decided by owner (evan-wes), 2026-10-02; fixed (d397203) |

### Hygiene review

**Scope:** `git diff 874db0f...966d820` (74 files), with changed Python/C++/Rust/scripts/tests read in full; run-2 fixes in `51c9483` cross-checked against the current tree. Did not re-run `scripts/check.sh` (layer 1 reported green on HEAD).
**Summary:** All six run-2 findings (H15–H20) are resolved. No new must-fix or should-fix issues; the first version satisfies me. One forward-looking note.

#### Run-2 findings
- **H15 (test path used `pytest.skip`) — resolved.** `tests/test_backends.py` has no direct `pytest.skip`; every backend-dependent test goes through `require_backend`, which calls `pytest.fail` when the backend is in `SLAMLAB_REQUIRE_BACKENDS` (including `test_missing_implementation_is_reported` and `test_every_compiled_module_matches_its_protocol`). `test_required_backends_are_known` / `test_required_backends_are_available` guard the guard.
- **H16 (tolerance policy declared, never applied) — resolved.** `tests/conftest.py:assert_matches_fixture` applies the fixture's policy: exact mode requires `rtol`/`atol` = 0 and checks `np.signbit` as well as values; close mode applies the given `rtol`/`atol` (never numpy's defaults); a malformed policy raises. `test_hat_matches_fixture` uses it; `test_fixture_declares_exact_comparison` pins the policy. The fixture grew to 155 lines of cases.
- **H17 (regex signature parser breaks on nested types) — resolved.** `_parameter_names` strips nested brackets (`_strip_brackets`) and then parses with `ast.parse`; PyO3 falls back to `inspect.signature`.
- **H18 (`--env` with no value dies obscurely) — resolved.** `scripts/setup_slamlab.sh:32` now fails with `--env needs an environment name` and `exit 2`, and also catches an empty string.
- **H19 (run-1 resolutions read "pending implementation") — resolved.** The only remaining occurrences are the quoted finding itself; the run-1 table now reads `fixed (51c9483)` with the owner's decision date.
- **H20 (no test for an unknown backend via `so3.hat`, or the locator's non-module branch) — resolved.** `test_hat_rejects_an_unknown_backend` and `test_hat_reports_an_unbuilt_backend` cover `so3.hat`; the `not-a-module` case (`slamlab.foundations.so3.hat`) covers the locator's non-module branch; `test_module_outside_slamlab_is_an_error` and the broken/absent-extension tests cover the rest.

#### Findings

##### [note] Error-contract translation is documented before any code exercises it
- **Where:** `cpp/include/slamlab/foundations/error.hpp:1-9`, `rust/slamlab-foundations/src/error.rs:1-6`, `src/slamlab/foundations/errors.py`
- **Issue:** The docs say the bindings translate an `Error` into the matching Python exception, but no fallible function exists yet (`hat` is infallible), so the mapping has no code and no test.
- **Why it matters:** The C++/Rust kinds and Python exceptions could drift with nothing to fail until the first fallible rung-0 function lands.
- **Evidence:** The C++ and Rust error tests only check the types in isolation; neither binding references `ErrorKind`.
- **Suggested fix:** None now (the scaffold is an explicit placeholder). When the first fallible function lands (milestone 1), add a parity test that each kind raises the matching exception from both bindings.

#### Checked and fine
- Input validation: `as_vec3` rejects masked arrays, bool, complex, string, None, wrong shape and non-finite values, each with a message naming the bad value; `BAD_INPUTS` covers all of them and runs for every backend.
- Backend locator: only an absent extension counts as "not built"; real import failures are surfaced unchanged, and the tests assert the exception's identity.
- Error contract: the three kinds match across Python, C++ (`std::expected`, `[[nodiscard]]` `fail`) and Rust (`Result`); all three are documented, and the C++/Rust core tests exercise success and failure.
- C++ and Rust cores are numpy-free, `noexcept`, and documented with `@param`/`@return` and `# Arguments`/`# Returns`.
- `rust/slamlab-py/src/foundations/so3.rs` returns `ValueError` for a non-contiguous or wrong-length array instead of unwrapping.
- The C++ binding's capsule ownership is sound and its `NOLINT`s are justified.
- C++ test policy: exact comparisons, because `hat` only copies and negates — the rationale is stated; the Python test's `CROSS_TOLERANCE_EPS` is derived and named.
- `scripts/check_layers.py` and `tests/test_check_layers.py` cover forbidden includes, downward includes, unknown rung directories, CMake links and Rust crate dependencies, with negative cases; the script derives its rules from the same `pyproject.toml` contracts as import-linter.
- Module docstrings state each module's role; `so3` is explicitly marked placeholder, so its limited scope is not a defect.
- No debug prints, commented-out code or stray TODOs; the only `# type`/`NOLINT`/`allow` suppressions seen each carry a reason.

### Architecture review

**Scope:** `git diff 874db0f...966d820` on `feature/review-panel` (Run 3). Read every changed source file in Python, C++, Rust, CMake, `pyproject.toml`, `scripts/check_layers.py`, `scripts/check.sh`, tests, and the design-doc diff; probed `check_layers.py` against a scratch copy of the tree at HEAD; inspected the built extensions.

**Summary:** All eight run-2 findings are resolved as the owner decided, and the structure now matches the design doc in all three languages. I am satisfied with this first version. Two small gaps remain in the new enforcement tooling — `check_layers.py` can be bypassed by common C++/Cargo idioms, and `check.sh` names the rung-0 crate explicitly, so later rung crates' tests would not run — plus one stale comment and one narrow gap in the generic Protocol test. Neither gap is a defect in the shipped code; nothing is must-fix.

#### Run 2 findings — status

| ID | Status | Evidence |
|---|---|---|
| A10 numpy boundary | **resolved** | `cpp/bindings/foundations/so3.cpp` takes `nb::ndarray<const double, nb::shape<3>, nb::c_contig, nb::device::cpu>` and returns an owned `nb::ndarray<nb::numpy, double, nb::shape<3,3>>`; `rust/slamlab-py/src/foundations/so3.rs` takes `PyReadonlyArray1<f64>` and returns `PyArray2<f64>`. The cores take `std::array` / `[f64; 3]`, with no numpy dependency. |
| A11 one binding file per module | **resolved** | `cpp/bindings/module.cpp` → `foundations/foundations.cpp` → `foundations/so3.cpp`; `rust/slamlab-py/src/lib.rs` → `foundations/mod.rs` → `foundations/so3.rs`. The roots only wire submodules. |
| A12 generic Protocol test | **resolved (one small gap, below)** | `tests/test_backends.py::test_every_compiled_module_matches_its_protocol` walks every rung submodule of both extensions and requires an `Implementation` Protocol, an exact function set, and matching parameter names; nanobind signatures are parsed with `ast` after bracket stripping. |
| A13 per-rung C++/Rust + layer check | **resolved (checker gaps, below)** | `slamlab_add_rung(foundations)`, the `rust/slamlab-foundations` crate, and `scripts/check_layers.py` deriving the allowed rungs from pyproject's contracts; `tests/test_check_layers.py` covers include, link, crate and unknown-rung cases. |
| A14 learned on dense only | **resolved** | `pyproject.toml` forbidden contract `slamlab.learned` → `slamlab.edge`, `slamlab.classic`, reflected in `allowed_dependencies`. |
| A15 exhaustive layers; interfaces' home | **resolved** | `exhaustive = true` on the layers contract; the design doc's "Shared interface types" names `slamlab.foundations.interfaces`, to be created with the first rung that needs it (tracked future work). |
| A16 torch libraries by name | **resolved** | The CPU-rung contract forbids `lietorch`, `pypose`, `kornia`, `theseus`; the comment and design doc say torch only ever goes in an extra; `dependencies = ["numpy>=1.26"]`. |
| A17 error contract | **resolved** | `cpp/include/slamlab/foundations/error.hpp` (`std::expected`, `ErrorKind`, `fail`) mirrors `rust/slamlab-foundations/src/error.rs` and `src/slamlab/foundations/errors.py`, each tested; the binding translation helper is tracked future work in the design doc ("arrives with the first fallible function"). |

#### Findings

##### [should-fix] `check_layers.py` misses common C++ include and Cargo dependency forms
- **Where:** `scripts/check_layers.py:30` (`_INCLUDE`), `:116-121` (rung dirs only), `:195` (`.get("dependencies")` only)
- **Issue:** It only matches quoted includes (`#include "slamlab/<rung>/..."`), so angle-bracket includes are unchecked. It only scans files inside `<root>/<rung>/` directories, so a header placed directly in `cpp/include/slamlab/` is never scanned. It reads only `[dependencies]`, not `[dev-dependencies]`, `[build-dependencies]` or `[target.*.dependencies]`. It does not resolve renamed crates (`foo = { package = "slamlab-dense", ... }`). And it skips any crate whose directory is not named `slamlab-*`.
- **Why it matters:** Every rung library gets the whole tree through `target_include_directories(... PUBLIC cpp/include)`, so a header-only or inline cross-rung use compiles and links without a `DEPENDS` — this include scan is the only C++ layering gate. On the Rust side, cargo enforces only what a manifest declares, so the manifest scan is the gate.
- **Evidence:** A scratch copy of HEAD with `cpp/src/edge/a.cpp` containing `#include <slamlab/classic/x.hpp>`; `cpp/include/slamlab/top.hpp` containing `#include "slamlab/dense/y.hpp"`; a `slamlab-edge` crate with `[dev-dependencies] slamlab-classic` and `base = { package = "slamlab-dense", ... }`; and a `slamlab-vi` crate with `[target.x86_64-unknown-linux-gnu.dependencies] slamlab-classic`. `python scripts/check_layers.py .` printed `C++ and Rust layering matches pyproject.toml` and exited 0.
- **Suggested fix:** Match `#include\s*[<"]slamlab/`; flag any source file under the rung roots that sits outside a rung directory; collect every dependency table, including `target.*`, and use each entry's `package` key when present; treat every workspace member except `slamlab-py` as a rung crate. Add one fake-repo test per form.

##### [should-fix] `check.sh` runs Rust tests for `slamlab-foundations` only
- **Where:** `scripts/check.sh:74` — `cargo test --manifest-path rust/Cargo.toml -p slamlab-foundations`
- **Issue:** The per-rung crate design means each new rung adds a crate (`slamlab-edge` next). Because the test step names one package, those crates' tests will not run in layer 1 unless someone remembers to edit this line. Clippy on the line above already uses `--workspace`.
- **Why it matters:** A new rung's Rust tests would be skipped without any error — the same kind of coverage loss `SLAMLAB_REQUIRE_BACKENDS` was built to prevent on the Python side.
- **Evidence:** The line quoted above, beside `cargo clippy ... --workspace --all-targets` on line 73.
- **Suggested fix:** `cargo test --workspace --exclude slamlab-py` (the cdylib bindings crate is the only one that cannot be tested standalone).

##### [note] The generic Protocol test skips extension submodules whose name is not a Python rung package
- **Where:** `tests/test_backends.py`, `test_every_compiled_module_matches_its_protocol` (the `rungs` / `found` comprehension)
- **Issue:** The walk starts from slamlab's Python packages and looks each one up on the extension. A misnamed top-level extension submodule (e.g. `slamlab._cpp.fundations`) is never visited, and `assert found` passes as long as any other rung is bound. Today only `foundations` exists, so nothing escapes yet.
- **Evidence:** Top-level module attributes are `['foundations']` on `slamlab._cpp` and `['slamlab_rs', 'foundations']` on `slamlab_rs`; any fix must allow the `slamlab_rs` self-reference.
- **Suggested fix:** Assert that every top-level `ModuleType` attribute of the extension is a rung package, with the extension's own name excepted.

##### [note] Stale Protocol name in the mypy comment
- **Where:** `pyproject.toml:84`
- **Issue:** The comment cites `slamlab.foundations.so3._Implementation`; the convention is now the public `Implementation` (as in `so3.py`, CLAUDE.md and the design doc). This is the hygiene reviewer's area, but it misstates the architectural convention.
- **Suggested fix:** Change the comment to `slamlab.foundations.so3.Implementation`.

#### Checked and fine
- Dependency direction: the layers contract is exhaustive and matches the design doc's table and ladder; the `vi` and `learned` forbidden contracts are in place; `backends` is the bottom layer and imports nothing from slamlab; the reifyscene ban runs one way.
- Dispatch: `hat` validates once through `foundations.arrays.as_vec3`, then dispatches; `backends.implementation` knows only module paths and correctly separates "not built" from "broken".
- Language split: the cores build standalone (`slamlab_foundations` static library with no Python; `slamlab-foundations` crate with no dependencies); bindings only convert types; Rust stays within rung 0; the parity fixture lives in `tests/fixtures/foundations/` with an explicit tolerance policy.
- Placement: SO(3), the shared array checks and the error classes are correctly in rung 0, and shared interfaces have a recorded home there.
- Compute budgets: each rung package docstring declares its budget; the `gpu`/`budget` markers are declared; the CPU-subset policy and the deferred light edge environment are in the design doc's open items.
- `hat` is clearly marked placeholder in all three languages and is not hardening into the design.
- No compatibility shims: the old `slamlab.so3`, `cpp/src/so3.cpp` and `rust/slamlab-core` were removed outright.
- Error contract: consistent across the three languages in kinds, naming and docs; the translation helper is correctly deferred until a fallible function exists.
- The design doc's status, packaging, binding boundary, error contract, layout, dispatch, contracts and review-process sections all match the code.

### Algorithm review

**Scope:** `874db0f...966d820` on `feature/review-panel`, focused on run 2's fixes in `51c9483`: `src/slamlab/foundations/{arrays,so3,errors}.py`, `tests/conftest.py`, `tests/foundations/{test_so3,test_errors}.py` and the hat fixture; the C++ core, error header and nanobind binding (`cpp/include/slamlab/foundations/{so3,error}.hpp`, `cpp/src/foundations/so3.cpp`, `cpp/bindings/foundations/so3.cpp`, `cpp/tests/foundations/test_error.cpp`); the Rust core and PyO3/numpy binding (`rust/slamlab-foundations/src/{so3,error}.rs`, `rust/slamlab-py/src/foundations/so3.rs`); the design doc's error-contract section; and `.claude/agents/algorithm-reviewer.md`. Re-ran the float32, invalid-input and signed-zero probes against all three built backends, plus a new probe of the raw extensions' buffer handling (scratch scripts only, outside the repo). `SLAMLAB_REQUIRE_BACKENDS=python,cpp,rust pytest tests/foundations tests/test_backends.py` → 79 passed. HEAD stayed at `966d820`, tree clean.

**Summary:** All five run-2 algorithm findings (L9–L13) are resolved, confirmed by probes on python, cpp and rust. The numpy boundary is correct in both bindings: results are new, memory-owning, C-contiguous float64 (3, 3) arrays, never aliased to the input. The two new findings are notes only — no must-fix or should-fix. On the algorithm side, this first version satisfies me.

#### Run 2 findings
- **L9 (`CROSS_TOLERANCE_EPS` docstring gave the wrong reason) — resolved.** `tests/foundations/test_so3.py:18-23` now names FMA in the BLAS matmul as the cause, and the bound is still correct: each method's error is ≤ 4u·M to first order (two rounded products plus one rounded sum; 3u·M with FMA), so they differ by ≤ 8u·M = 4 eps·M.
- **L10 (exact comparison ignored signed zeros) — resolved.** `tests/conftest.py:96-99` compares `np.signbit` after `assert_array_equal`. Probe: flipping a fixture case's `-0.0` entries to `+0.0` is caught; all three backends have 0 sign-bit mismatches across all fixture cases; `hat([-0.0, 0.0, -0.0])` gives identical sign bits `[0,0,0,1,0,0,1,1,0]` on python, cpp and rust.
- **L11 (tolerance policy declared, not consumed) — resolved.** `assert_matches_fixture` (`tests/conftest.py:70-103`) reads `comparison`/`rtol`/`atol` from the fixture; an exact policy is refused if it sets a non-zero `rtol`/`atol`; a `close` policy passes `rtol`/`atol` explicitly (never numpy's defaults); an unknown comparison fails. The parity test uses it. A simulated float32 backend fails 3 of 6 cases — exactly those holding values float32 cannot represent (0.1 and 1/3; 1e±300; 0.7 and 1/7); the three that pass contain only exactly-representable values.
- **L12 (masked arrays passed through) — resolved.** `arrays.py:46-47` rejects every `np.ma.MaskedArray` (including one with no mask — strict but defensible) with `InvalidInputError` on all three backends. `as_vec3` now returns `np.ascontiguousarray(..., float64).copy()`; a probe shows a strided input (`np.arange(6.0)[::2]`), a big-endian `>f8` input and a read-only buffer all give the correct result, and the input is never modified.
- **L13 (ORB-SLAM3 citation; log at π) — resolved.** `algorithm-reviewer.md:46-47` cites ORB-SLAM3 as Campos et al., T-RO 2021; `:60` states that at exactly π the axis is ambiguous up to sign, so parity checks must accept ±`omega`.

**Invalid-input probe (identical on all three backends):**
- Rejected with `InvalidInputError` and the same message: `None`, str, bool, complex, NaN, ±inf, masked, object dtype, `timedelta64`, `datetime64`, scalar, shape (1,3), shape (3,1), `np.matrix`, dict, set, generator, and an int ≥ 2**64 (object dtype).
- Accepted and widened exactly: float16, big-endian float64, read-only input.
- `uint64` `2**64-1` is accepted and rounds to the nearest float64, as the docstring warns.

#### Findings

##### [note] The C++ raw binding converts input silently, unlike its docstring and the Rust binding
- **Where:** `cpp/bindings/foundations/so3.cpp:22-23` (docstring) and `:45` (`nb::arg("v")` without `.noconvert()`); compare `rust/slamlab-py/src/foundations/so3.rs:12-14`.
- **Issue:** The docstring says "nanobind raises `TypeError` for anything else". Because implicit conversion is enabled, nanobind actually accepts float32 and strided arrays by copying them into a temporary contiguous float64 array. The Rust binding rejects both, so the two raw extensions disagree on the same input.
- **Why it matters:** No effect through `so3.hat`, since `as_vec3` always passes a contiguous float64 copy. But a direct caller — or a later batched benchmark — could have the C++ path pay for a hidden conversion copy while Rust refuses the same input, skewing the "same algorithm, three languages" comparison the boundary was redesigned for (A10). It is the template every later binding will copy.
- **Evidence:** raw-extension probe output:
  ```
  cpp  raw f32        -> OK float64 [0.0, -3.0, 2.0, ...]
  cpp  raw noncontig  -> OK float64 [0.0, -4.0, 2.0, ...]
  rust raw f32        -> TypeError 'ndarray' object is not an instance of 'ndarray'
  rust raw noncontig  -> ValueError The given array is not contiguous or is misaligned.
  ```
- **Suggested fix:** `nb::arg("v").noconvert()`, so the C++ boundary is as strict as its docstring and the Rust binding; otherwise reword the docstring to describe the conversion.

##### [note] `NotConverged` as a hard error discards the best estimate; decide the iterative-solver contract before milestone 1's solvers
- **Where:** `docs/design/2026-10-02-slamlab-ladder.md:95-106`; `src/slamlab/foundations/errors.py:30-31`; `cpp/include/slamlab/foundations/error.hpp:25-27`; `rust/slamlab-foundations/src/error.rs:18-19`.
- **Issue:** The contract makes reaching the iteration cap an error. Through `std::expected`, `Result` and an exception, that error carries no value. Standard nonlinear least-squares and robust estimators — Gauss-Newton, LM, BA, RANSAC with an adaptive iteration count — report non-convergence as a status alongside the best estimate. SLAM front ends routinely use a capped LM/BA result; real-time BA is iteration-capped by design.
- **Why it matters:** Later rungs would have to either treat a capped but improved solve as failure or work around the contract. The checklist's "report non-convergence" is better served by a summary than by an exception that throws the result away.
- **Evidence:** Ceres Solver returns `Solver::Summary` with `termination_type` (`CONVERGENCE`, `NO_CONVERGENCE`, `FAILURE`) next to the updated parameters (http://ceres-solver.org/nnls_solving.html#solver-summary). SciPy's `least_squares` returns `x` with `status` (0: "the maximum number of function evaluations is exceeded") (https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html). Triggs et al., "Bundle Adjustment — A Modern Synthesis" (Vision Algorithms 1999), treats stopping criteria as part of the solver's output, not as failure.
- **Suggested fix:** No code now — nothing fallible exists. In the milestone-1 plan, decide whether iterative solvers return the estimate plus a summary (iterations, final cost, converged flag), keeping `NotConverged` / `NotConvergedError` for a caller-requested strict mode or for runs that diverge or fail without an estimate. `InvalidInput` and `Degenerate` are right as hard errors.

#### Checked and fine
- C++ binding memory: a heap buffer freed by a capsule deleter; `Mat3Out(data, {3,3}, owner)` gets C-order strides (24, 8), and the row-major copy `data[3*i+j] = h[i][j]` matches `Mat3`'s array-of-rows layout. Results are new, writeable, never alias the input, and independent across calls (mutating one result does not affect the next).
- Rust binding memory: `PyArray1::from_vec(...).reshape([3,3])` gives a C-contiguous float64 (3, 3) array with strides (24, 8) whose base is a fresh array, not the input; it requires exact f64 and contiguity, and reports a wrong length with a clear message.
- Python reference: builds a new float64 array; its signed-zero behaviour is identical to C++ and Rust.
- `as_vec3`: checks the dtype allow-list and shape before casting and finiteness after the float64 cast (so longdouble overflow is still caught); returns a new contiguous copy, so a backend can never alias or modify the caller's data.
- `assert_matches_fixture`: sound for both policies. The `signbit` check also distinguishes `-NaN` from `+NaN` — harmless, since NaN is rejected upstream.
- `CROSS_TOLERANCE_EPS = 4` re-confirmed as the right first-order constant (8u = 4 eps), now with the correct reason.
- Error contract: one list of kinds in C++ (`enum class ErrorKind`, `std::expected`, the `fail` helper), Rust (`ErrorKind`, `Result`, `Display`) and Python (three `ValueError` subclasses under `SlamlabError`), each tested natively. The binding translation is explicitly deferred to the first fallible function, which is reasonable while nothing can fail. `InvalidInputError` as a `ValueError` subclass keeps `except ValueError` callers working.
- `hat` is still the marked placeholder; it matches `hat(v)u = v × u` (Barfoot, *State Estimation for Robotics*; Solà, Deray & Atchuthan, arXiv:1812.01537), is exact in all three languages, and the native cross-product checks are valid because every value in them is exactly representable.
- Fixture: includes float32-unrepresentable values (0.1, 1/3, 0.7, 1/7, 1e±300) and `-0.0` cases; JSON round-trips doubles exactly.
- Randomised tests are seeded (`default_rng(0)`).
- Reference list: the new citations are accurate, as are those checked in run 2.
