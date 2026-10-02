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

- Every C++/Rust implementation has a Python reference and a shared parity fixture in
  `tests/fixtures/`. Bindings only convert types; algorithms live in the cores, which build
  without Python.
- Backends are reached through `slamlab.backends`, never hard-coded.

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

## Privacy

The repo is **private** and stays private until it goes public together with reifyscene, on
the owner's explicit instruction. "slamlab" is a working name, to be changed before then.
