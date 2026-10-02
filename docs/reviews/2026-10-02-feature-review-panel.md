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
| A1 | must-fix | architecture | `.github/workflows/test.yml`, `backends.py`, `tests/test_backends.py` | CI no longer asserts all three backends are built; a broken extension turns parity tests into skips and CI stays green (with H1, H2) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H1 | should-fix | hygiene | `src/slamlab/backends.py` (`_import`) | Any `ImportError` is reported as "not built", hiding a broken extension (with A1) | fix | fix (with A1) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H2 | should-fix | hygiene | `tests/test_backends.py` (`_require`) | Parity tests skip silently when a compiled backend is missing, including in CI (with A1) | fix | fix (with A1) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H3 | should-fix | hygiene | `src/slamlab/backends.py` (`Backend.hat`) | Docstring lacks Parameters/Returns/Raises; `name` undocumented | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H4 | should-fix | hygiene | `scripts/setup_slamlab.sh` header | Says "three editable installs", lists two | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H5 | should-fix | hygiene | `pyproject.toml` `dev` extra | Out of step with the real toolchain (`mypy`, `import-linter` missing) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H6 | should-fix | hygiene | `.github/workflows/test.yml` | Miniforge from `latest` with no pin; no `permissions`, no `timeout-minutes` | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H7 | should-fix | hygiene | `scripts/check.sh` | `--env` without a value dies obscurely; unknown arguments silently ignored | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H8 | should-fix | hygiene | `.claude/skills/review-panel/SKILL.md` | Ambiguous agent-availability sentence; `HEAD` variable name; dirty-tree check unspecified | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H9 | should-fix | hygiene | tests in all three languages | `1e-12` tolerance on an exact operation, unnamed (with L1) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L1 | should-fix | algorithm | `tests/test_backends.py`, `tests/fixtures/so3_hat.json` | Parity tests cannot detect a float32 regression: default `rtol=1e-7`, float32-exact fixture values (with H9) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L2 | should-fix | algorithm | `src/slamlab/so3.py` (`as_vec3`) | Accepts `None` (to NaN), strings, bools, complex, and non-finite values (with H11) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L6 | should-fix | algorithm | `.claude/agents/algorithm-reviewer.md` | Reference list misses rung-0 classics (Nistér, Hartley 8-pt, RANSAC, EPnP, Umeyama, ...) and current SOTA (MASt3R-SLAM, DPVO, Shape of Motion) | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A2 | should-fix | architecture | `pyproject.toml` import-linter | Layers contract names one module; will not cover rung code. Proposes package-per-rung with layer, independence and no-torch-on-edge contracts | defer to milestone 1 step 0 | fix in this PR (restructure design approved) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A3 | should-fix | architecture | `src/slamlab/backends.py` | `Backend` is a per-algorithm facade that will grow with every rung; proposes compiled submodules mirroring the package tree, dispatch next to each algorithm | defer to milestone 1 step 0 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A4 | should-fix | architecture | `src/slamlab/so3.py` (`as_vec3`) | Generic validation helper placed in the SO(3) module; belongs in a rung-0 validation module | defer to milestone 1 step 0 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A5 | note | architecture | `scripts/check.sh` | Flat C++ globs will miss subdirectories | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A6 | note | architecture | `check.sh`, CI, env | Plan how layer 1 splits for GPU and budget rungs (pytest markers, a light edge env) | defer to the first PyTorch rung | fix in this PR -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A7 | note | architecture | `pyproject.toml` mypy override | Compiled extensions have no type stubs | defer to the second bound function | fix in this PR -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A8 | note | architecture | design doc header | Status line says nothing is built | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L3 | note | algorithm | `rust/slamlab-py/src/lib.rs` | Docstring names `TypeError`; PyO3 raises `ValueError` | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L4 | note | algorithm | C++/Rust native tests | One `(v, u)` pair does not pin the matrix; no skew-symmetry check | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L5 | note | algorithm | `tests/fixtures/` | Rung-0 fixtures will need the hard cases (near 0, near pi, wrap) | defer to the milestone 1 plan | fix: tolerance policy and fixture format now (L1); the hard exp/log cases land with exp/log in milestone 1, as the reviewer suggested -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L7 | note | algorithm | `.claude/agents/algorithm-reviewer.md` | Checklist gaps: tolerance policy, quaternion double cover, angle wrap, cheirality, Jacobians, seeded RNG, ATE alignment | fix | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| L8 | note | algorithm | `so3` in all languages | Placeholder scope | no action | no action (placeholder scope) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H10 | note | hygiene | `backends.py`, `slamlab-py` | Validation happens in `Backend`; raw extensions' own boundary is untested | fix (state in the `Backend` docstring) | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H11 | note | hygiene | `as_vec3` | No finiteness check (covered by L2) | fix with L2 | fix (as recommended) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H12 | note | hygiene | `backends.py` | Backend list repeated; `so3` imported two ways | defer with A3 | fix in this PR (restructure) -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H13 | note | hygiene | `CLAUDE.md` | Merge rule conflicts with the owner's global "never merge" rule (same as A9) | dispute | fix: CLAUDE.md cites the owner's explicit grant -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| A9 | note | architecture | `CLAUDE.md` | Same as H13 | dispute | fix: CLAUDE.md cites the owner's explicit grant -- decided by owner (evan-wes), 2026-10-02; pending implementation |
| H14 | note | hygiene | `Doxyfile`, PR template, `setup_slamlab.sh`, `pyproject.toml` | HTML output unneeded; relative report link; x86_64 compiler names; deprecated license table | fix (HTML, link, license); defer x86_64 to milestone 7 | fix (all parts, including the x86_64 compiler names) -- decided by owner (evan-wes), 2026-10-02; pending implementation |

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
