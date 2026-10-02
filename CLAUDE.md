# slamlab: working conventions

slamlab is a learning ladder for visual SLAM in Python, C++ and Rust: a hobby and learning
project, built through a human-directed agentic workflow. The design, the ladder and the
milestones are in `docs/design/2026-10-02-slamlab-ladder.md`. Read it before changing
structure.

## Environment

- Conda env `slamlab`, built by `scripts/setup_slamlab.sh` (Python 3.11 and the full toolchain
  from conda-forge, pip layer from `scripts/locks/slamlab.lock`). Run tools inside it, e.g.
  `conda run -n slamlab <cmd>`.
- Wrap builds and test suites in a memory cap and cap parallelism at 4 jobs:
  `CMAKE_BUILD_PARALLEL_LEVEL=4 CARGO_BUILD_JOBS=4 systemd-run --user --scope -p MemoryMax=8G -- ...`
- Datasets and run outputs live outside the repo, in `../datasets/` and `../runs/`.

## Languages

| Language | Role |
|---|---|
| Python (`src/slamlab`) | Reference implementation of every algorithm; orchestration, evaluation, learned rungs |
| C++ (`cpp/`, module `slamlab._cpp`) | Performance core: rung 0, edge rung, edge budgets |
| Rust (`rust/`, module `slamlab_rs`) | Rung 0 and the edge VO only |

## Layout: one package per rung, mirrored in every language

- Python: `slamlab.foundations` (rung 0), `.edge`, `.vi`, `.classic`, `.dense`, `.learned`,
  `.eval`, `.orchestration`. Each package docstring states its rung, milestone and compute
  budget. The import-linter contracts in `pyproject.toml` enforce the ladder's layers,
  sibling independence, no PyTorch on CPU rungs, and no depth-model imports.
- C++ mirrors it with one library per rung (`slamlab_foundations`, from `cpp/src/foundations/`,
  namespace `slamlab::foundations`); Rust with one crate per rung (`slamlab-foundations`).
  `scripts/check_layers.py` holds C++ includes, CMake links and crate dependencies to the
  same contracts. Bindings are one file per module (`cpp/bindings/<rung>/<module>.cpp`,
  `rust/slamlab-py/src/<rung>/<module>.rs`), and the extension submodules mirror the tree
  (`slamlab._cpp.foundations.so3`, `slamlab_rs.foundations.so3`).
- **Binding boundary:** bindings take and return float64 numpy arrays (nanobind `ndarray`,
  the Rust `numpy` crate); the cores stay numpy-free and take arrays, spans or slices.
- **Error contract:** fallible core functions return C++23 `std::expected<T, Error>` /
  Rust `Result<T, Error>` with one shared kind list (InvalidInput, Degenerate,
  NotConverged); Python raises the matching `slamlab.foundations.errors` exception.
- **Each algorithm validates its input once** (shared checks in
  `slamlab.foundations.arrays`), then computes with the Python reference or dispatches via
  its `backend=` argument. `slamlab.backends` only locates compiled implementations.
- Every C++/Rust implementation has a Python reference, a parity fixture in
  `tests/fixtures/<rung>/` that states its tolerance policy (applied by
  `tests/conftest.assert_matches_fixture`), and an `Implementation` Protocol in its Python
  module that `tests/test_backends.py` checks every extension submodule against. Bindings only convert types; algorithms live in
  the cores, which build without Python.
- Tests mirror the packages (`tests/foundations/`, `cpp/tests/foundations/`). Mark tests
  that need a GPU or a compute budget with `@pytest.mark.gpu` / `@pytest.mark.budget`;
  `check.sh` runs the CPU subset.

## Code standard (all three languages)

- **Documented, everywhere.** Python: numpy-style docstrings on every module, class and
  function, with full type hints (`mypy --strict`). C++: Doxygen `@file`, `@brief`, `@param`,
  `@return`, `@throws` on every declaration. Rust: rustdoc on every item, public or private,
  with `# Arguments` / `# Returns` / `# Errors` / `# Panics` as relevant. State shapes, units
  and frame conventions.
- **Production quality.** Validate inputs at public boundaries; make failure observable; no
  swallowed errors, debug prints, unexplained magic numbers or untested behaviour. Code that
  is deliberately a placeholder says so in its docs.
- **No compatibility shims.** There are no users: make clean breaks.

## Review process

**Layer 1, every PR:** `scripts/check.sh` must pass. It runs ruff (numpy docstrings, type
hints), mypy --strict, import-linter, clang-format, clang-tidy, doxygen (every declaration
documented), rustfmt, clippy (pedantic, missing docs), jscpd (duplication), and the C++,
Rust and Python test suites. CI runs the same script.

**Layer 2, the review panel (`/review-panel`):**
- **When:** on PATCH-or-larger changes: new algorithms, modules, rungs or milestones, a new
  language implementation, interface or boundary changes. Not on MICRO changes (fixes, docs,
  CI). A smaller change that touches one area can run that reviewer alone:
  `/review-panel algorithm | architecture | hygiene`.
- **Say which applies** when proposing a PR: full panel, one reviewer, or layer 1 only. The
  user may override either way.
- **Reviewers:** `hygiene-reviewer` (sonnet), `architecture-reviewer` (opus),
  `algorithm-reviewer` (opus, with web search). All call out code that is not production
  quality.
- **Reports are saved** to `docs/reviews/` and committed with the PR.
- **Findings are decided together.** Present each finding with a recommended action (fix,
  defer, waive, dispute); the user decides; only then apply and record it. Every must-fix
  needs an explicit decision. Never act on findings unilaterally.

## Git and PRs

- Feature branches off `develop`, PRs into `develop`. `main` only takes `develop` releases.
- Every PR with a code change bumps `VERSION` (4-digit MAJOR.MINOR.PATCH.MICRO) and adds a
  `CHANGELOG.md` entry.
- **PR descriptions follow `.github/pull_request_template.md`.** Its "How this was built"
  section records the agentic process: which agents did what, the panel's verdicts and how
  each finding was resolved, and the owner's decisions.
- **Merging:** Claude may merge slamlab PRs only after a selected **Yes** on a yes/no prompt
  for that specific PR, using `gh pr merge <n> --repo evan-wes/slamlab`. A typed instruction
  is never permission. Then verify that the commits landed on `develop`, check the tag, and
  ask before deleting the branch.
  This is a deliberate exception the owner granted on 2026-10-02 to their global "never
  merge a PR" rule. It is written into that rule in the owner's global Claude Code instructions ("The one
  exception: slamlab"), and a PreToolUse hook (the owner's merge-guard hook) blocks
  every merge except this exact form. The hook matches the phrase anywhere in a shell
  command, so edit text that contains it with the file editor, not a shell heredoc.

## Privacy

The repo is **private** and stays private until it goes public together with reifyscene, on
the owner's explicit instruction. "slamlab" is a working name, to be changed before then.
