# slamlab: working conventions

slamlab is a learning ladder for visual SLAM in Python, C++ and Rust: a hobby and learning
project, built through a human-directed agentic workflow. The design, the ladder and the
milestones are in `docs/design/2026-10-02-slamlab-ladder.md`. Read it before changing
structure.

## Where sessions run

Work on slamlab happens from sessions started in the **reifyscene** repository, so the two projects progress together. Never
suggest starting a session inside slamlab. Consequences:
- This file does not load automatically: a reifyscene memory entry points here, and it must
  be read before slamlab work.
- Address slamlab explicitly: `git -C $REPO ...`, and run its
  tools from its root.
- The review panel's agents (`.claude/agents/slamlab-*.md`) and skill
  (`.claude/skills/slamlab-review-panel/`) are symlinked into reifyscene's git-ignored
  `.claude/`, so they are registered in reifyscene sessions while staying single-sourced
  here. Every path in them is relative to the slamlab root, which the skill passes to each
  reviewer.

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
| C++ (`cpp/`, module `slamlab._cpp`) | Performance core: rung 0, edge rung, edge budgets; `eval`'s native readers, run writer and process readings (`slamlab_eval`); the native programs (`slamlab_orchestration`, `slamlab_vo`) |
| Rust (`rust/`, module `slamlab_rs`) | Rung 0 and the edge VO only; `eval`'s native readers, run writer and process readings (`slamlab-eval`); the native programs (`slamlab-orchestration`, `slamlab_vo`) |

## Conventions

Frames (`T_a_b`; solvers use `T_cam_world`), right perturbations, `(ρ, θ)` tangent order,
xyzw Hamilton quaternions, the array encoding of every geometric type, pixel centres, exact
pyramid coordinates (never `x s^l`), exact floats (scalar, unfused), Eigen storage and plain
Cholesky are fixed in the design doc's "Conventions" section. Read it before writing geometry
code or a binding.

## Layout: one package per rung, mirrored in every language

- Python: `slamlab.foundations` (rung 0), `.edge`, `.vi`, `.classic`, `.dense`, `.learned`,
  `.eval`, `.orchestration`. Each package docstring states its rung, milestone and compute
  budget. The import-linter contracts in `pyproject.toml` enforce the ladder's layers,
  sibling independence, no PyTorch on CPU rungs, and no depth-model imports.
- C++ mirrors it with one library per rung (`slamlab_foundations`, from `cpp/src/foundations/`,
  namespace `slamlab::foundations`); Rust with one crate per rung (`slamlab-foundations`).
  `scripts/check_layers.py` holds C++ includes and Rust `crate::`/`super::` paths (chains
  included, and the order inside a rung), CMake links and crate dependencies to the same
  contracts. Bindings are one file per module (`cpp/bindings/<rung>/<module>.cpp`,
  `rust/slamlab-py/src/<rung>/<module>.rs`), and the extension submodules mirror the tree
  (`slamlab._cpp.foundations.so3`, `slamlab_rs.foundations.so3`).
- **Binding boundary:** bindings take and return float64 numpy arrays, uint8 for images
  (nanobind `ndarray`, the Rust `numpy` crate); the cores stay numpy-free and take arrays,
  spans or slices.
- **Error contract:** fallible core functions return C++23 `std::expected<T, Error>` /
  Rust `Result<T, Error>` with one shared kind list (InvalidInput, Degenerate,
  NotConverged); Python raises the matching `slamlab.foundations.errors` exception.
  Iterative solvers return their best estimate plus a solve summary (iterations, costs,
  converged flag, termination reason); `NotConverged` is only for strict mode or divergence.
- **Each algorithm validates its input once** (shared checks in
  `slamlab.foundations.arrays`), then computes with the Python reference or dispatches via
  its `backend=` argument. `slamlab.backends` only locates compiled implementations.
- Every C++/Rust implementation has a Python reference, a parity fixture in
  `tests/fixtures/<rung>/` that states its tolerance policy (applied by
  `tests/conftest.assert_matches_fixture`), and an `Implementation` Protocol in its Python
  module that `tests/test_backends.py` checks every extension submodule against. The edge
  rung's stages compare live with the Python reference on states built from the synthetic
  room (stored, they would run to megabytes), at `tests/edge/support.py`'s tolerances; its
  fixture is a golden record of the room's runs (`tests/fixtures/edge/room_runs.json`) that
  every backend is held to, so a decision changed in all three languages fails until the
  record is regenerated. Bindings only convert types; algorithms live in the cores, which
  build without Python.
- Native benchmarks are developer tools outside the cores: `cpp/bench/<rung>/` (Google
  Benchmark, built with `-DSLAMLAB_BUILD_BENCHMARKS=ON`) and each crate's `benches/`
  (criterion), held to the ladder by `check_layers.py`; layer 1 builds them, never runs them.
  `slamlab.eval` keeps the Python benchmarks and the budget harness (`slamlab.eval.budgets`):
  a program pinned to a budget's cores and its memory capped, its frames judged against their
  deadline. Its runs are the tests marked `budget`, outside layer 1:
  `python -m pytest -m budget`.
- Native programs run a whole pipeline without Python: `cpp/apps/<rung>/<program>.cpp`
  (linked to the rung's library, installed into `slamlab/bin/` by scikit-build) and a rung
  crate's binary (built by cargo into `rust/target/release/`, `--target-dir rust/target`),
  found by `slamlab.backends.program` and held to the ladder by `check_layers.py`. Layer 1
  builds them and the Python tests run them (`slamlab.orchestration.vo` runs `slamlab_vo`).
  Python has its program too, with the same command line, `python -m slamlab.orchestration.vo`,
  so a budget runs all three backends the same way.
  The Rust program is not rebuilt on import: after changing a Rust crate, rebuild it (layer 1
  does), or the tests fail it as older than its sources.
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
Rust and Python test suites. CI runs the same script, offline: `scripts/offline_ci.sh`
repeats `.github/workflows/test.yml` on a clean checkout with its own env built from the
lock, gates every PR, and its result goes in the PR template. The GitHub workflow runs on
demand and on pushes to `main` (the Actions minutes are shared with other projects).

**PRs are slices, not steps.** A PR makes meaningful progress toward a stated goal: a slice
of a milestone (e.g. milestone 1's "BA spine"). The individual steps are semantic commits
inside it (`feat(foundations): ...`, `test(...)`, `docs(notes): ...`), each with layer 1
green. Prefer few, substantial PRs over many small ones.

**Layer 2, the review panel (`/slamlab-review-panel`):**
- **When:** once per PR, on the finished slice, against the PR's stated goal. Never after
  every incremental change: reviewing each step lets each run's findings steer the next step,
  and the work drifts. Not on MICRO PRs (fixes, docs, CI). A small PR that touches one area
  can run that reviewer alone: `/slamlab-review-panel algorithm | architecture | hygiene`.
- **Findings outside the PR's goal go to a backlog by default**, not into the PR.
- **Stop rule for a component:** it is done when its oracle and property tests pass within
  the stated tolerance, its named hard cases pass, and its consumer works.
- **Say which applies** when proposing a PR: full panel, one reviewer, or layer 1 only. The
  user may override either way.
- **Reviewers:** `slamlab-hygiene-reviewer` (sonnet), `slamlab-architecture-reviewer` (opus),
  `slamlab-algorithm-reviewer` (opus, with web search). All call out code that is not
  production quality.
- **Reports are saved** to `docs/reviews/` and committed with the PR.
- **Findings are decided together.** Present each finding with a recommended action (fix,
  defer, waive, dispute); the user decides; only then apply and record it. Every must-fix
  needs an individual, explicit decision; should-fix findings and notes may be decided in
  bulk. Never act on findings unilaterally.

## Git and PRs

- Feature branches off `develop`, PRs into `develop`. `main` only takes `develop` releases.
- **Docs-only changes skip the PR:** once the owner has finalized the wording, commit them
  directly on `develop` (layer 1 green) and push. They need no version bump or CHANGELOG
  entry. Anything that touches code, tests or tooling still goes through a PR.
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
