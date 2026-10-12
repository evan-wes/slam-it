# Working conventions for SLAM It All!

SLAM It All! is a learning ladder for visual SLAM in Python, C++ and Rust: a hobby and learning
project, built through a human-directed agentic workflow. The design, the ladder and the
milestones are in `docs/design/2026-10-02-slam-it-all-ladder.md`. Read it before changing
structure.

## Where sessions run

Work on SLAM It All! happens from sessions started in the **reifyscene** repository, so the two projects progress together. Never
suggest starting a session inside the SLAM It All! repository. Consequences:
- This file does not load automatically: a reifyscene memory entry points here, and it must
  be read before SLAM It All! work.
- Address SLAM It All! explicitly: `git -C $REPO ...`, and run
  its tools from its root.
- The review panel's agents (`.claude/agents/slam-it-all-*.md`) and skill
  (`.claude/skills/slam-it-all-review-panel/`) are symlinked into reifyscene's git-ignored
  `.claude/`, so they are registered in reifyscene sessions while staying single-sourced
  here. Every path in them is relative to the SLAM It All! root, which the skill passes to each
  reviewer.

## Environment

- Conda env `slam-it-all`, built by `scripts/setup_slam_it_all.sh` (Python 3.11 and the full
  toolchain from conda-forge, pip layer from `scripts/locks/slam-it-all.lock`). Run tools inside it,
  e.g. `conda run -n slam-it-all <cmd>`.
- Wrap builds and test suites in a memory cap and cap parallelism at 4 jobs:
  `CMAKE_BUILD_PARALLEL_LEVEL=4 CARGO_BUILD_JOBS=4 systemd-run --user --scope -p MemoryMax=8G -- ...`
- Datasets and run outputs live outside the repo, in `../datasets/` and `../runs/`.

## Languages

| Language | Role |
|---|---|
| Python (`src/slam_it_all`) | Reference implementation of every algorithm; orchestration, evaluation, learned rungs |
| C++ (`cpp/`, module `slam_it_all._cpp`) | Performance core: rung 0, edge rung, edge budgets; `eval`'s native readers, run writer and process readings (`slam_it_all_eval`); the native programs (`slam_it_all_orchestration`, `slam_it_all_vo`) |
| Rust (`rust/`, module `slam_it_all_rs`) | Rung 0 and the edge VO only; `eval`'s native readers, run writer and process readings (`slam-it-all-eval`); the native programs (`slam-it-all-orchestration`, `slam_it_all_vo`) |
| JavaScript (`viewer/`) | The run viewer's page, and the upload and status pages beside it: plain modules, no build step, three.js from a CDN, pinned by its hashes. The viewer reads only a bundle (`slam_it_all.orchestration.bundles`), on its own site; the upload and status pages talk to the upload server's API. Logic modules (all but `main.js` and `*_view.js`) never touch the page (checked: `tsc` without the DOM), so Node tests them; JSDoc types, checked by `tsc --checkJs`; formatted by prettier |
| Python (`server/`, `slam_it_all_server`) | The upload server (FastAPI, uvicorn, boto3): the job list, storage's links and the guards, for the run viewer's "your own video". A package of its own that never imports `slam_it_all`, tested in `tests/server/` |

## Conventions

Frames (`T_a_b`; solvers use `T_cam_world`), right perturbations, `(ρ, θ)` tangent order,
xyzw Hamilton quaternions, the array encoding of every geometric type, pixel centres, exact
pyramid coordinates (never `x s^l`), exact floats (scalar, unfused), Eigen's product blocking
fixed (every rung compiles with `EIGEN_NO_CPUID`, so a run repeats on every core of a hybrid
CPU), Eigen storage and plain Cholesky are fixed in the design doc's "Conventions" section.
Read it before writing geometry code or a binding.

**The project's name** is SLAM It All! in prose. Code uses `slam_it_all` (the Python package,
the C++ namespace, libraries and programs, the extension modules); everything else uses
`slam-it-all` (the distribution, the Rust crates, the conda envs, the repository); macros and
CMake options use `SLAM_IT_ALL_`, and types `SlamItAll`.

## Layout: one package per rung, mirrored in every language

- Python: `slam_it_all.foundations` (rung 0), `.edge`, `.vi`, `.classic`, `.dense`, `.learned`,
  `.eval`, `.orchestration`. Each package docstring states its rung, milestone and compute
  budget. The import-linter contracts in `pyproject.toml` enforce the ladder's layers,
  sibling independence, no PyTorch on CPU rungs, and no depth-model imports.
- The upload server, `slam_it_all_server` (`server/src/`), is import-linter's second root
  package: it never imports `slam_it_all` (nor PyAV, Pillow, OpenCV or reifyscene: the process
  holding the storage keys never parses a video), `slam_it_all` never imports it or its web
  stack, and its own modules keep an order (`__main__`, `app`, `jobs | storage`, `config`).
  The worker (`slam_it_all.orchestration.worker`) talks to it over HTTP only; the constants
  both sides hold are held equal by `tests/server/test_shared_names.py`.
- C++ mirrors it with one library per rung (`slam_it_all_foundations`, from `cpp/src/foundations/`,
  namespace `slam_it_all::foundations`); Rust with one crate per rung (`slam-it-all-foundations`).
  `scripts/check_layers.py` holds C++ includes and Rust `crate::`/`super::` paths (chains
  included, and the order inside a rung), CMake links and crate dependencies to the same
  contracts. Bindings are one file per module (`cpp/bindings/<rung>/<module>.cpp`,
  `rust/slam-it-all-py/src/<rung>/<module>.rs`), and the extension submodules mirror the tree
  (`slam_it_all._cpp.foundations.so3`, `slam_it_all_rs.foundations.so3`).
- **Binding boundary:** bindings take and return float64 numpy arrays, uint8 for images
  (nanobind `ndarray`, the Rust `numpy` crate); the cores stay numpy-free and take arrays,
  spans or slices.
- **Error contract:** fallible core functions return C++23 `std::expected<T, Error>` /
  Rust `Result<T, Error>` with one shared kind list (InvalidInput, Degenerate,
  NotConverged); Python raises the matching `slam_it_all.foundations.errors` exception.
  Iterative solvers return their best estimate plus a solve summary (iterations, costs,
  converged flag, termination reason); `NotConverged` is only for strict mode or divergence.
- **Each algorithm validates its input once** (shared checks in
  `slam_it_all.foundations.arrays`), then computes with the Python reference or dispatches via
  its `backend=` argument. `slam_it_all.backends` only locates compiled implementations.
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
  Benchmark, built with `-DSLAM_IT_ALL_BUILD_BENCHMARKS=ON`) and each crate's `benches/`
  (criterion), held to the ladder by `check_layers.py`; layer 1 builds them, never runs them,
  the C++ ones in their own unhardened tree, `build/bench`, the one to time from (the tests'
  `build/native` runs with libstdc++'s assertions, which would slow a timing).
  `slam_it_all.eval` keeps the Python benchmarks and the budget harness
  (`slam_it_all.eval.budgets`): a program pinned to a budget's cores and its memory capped, its
  frames judged against their deadline. Its runs are the tests marked `budget`, outside layer 1:
  `python -m pytest -m budget`.
- Native programs run a whole pipeline without Python: `cpp/apps/<rung>/<program>.cpp`
  (linked to the rung's library, installed into `slam_it_all/bin/` by scikit-build) and a rung
  crate's binary (built by cargo into `rust/target/release/`, `--target-dir rust/target`),
  found by `slam_it_all.backends.program` and held to the ladder by `check_layers.py`. Layer 1
  builds them and the Python tests run them (`slam_it_all.orchestration.vo` runs `slam_it_all_vo`).
  Python has its program too, with the same command line, `python -m slam_it_all.orchestration.vo`,
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
hints) and mypy --strict over `src/`, `server/`, `tests/` and `scripts/`, import-linter,
clang-format, clang-tidy, doxygen (every declaration documented), rustfmt, clippy (pedantic,
missing docs), the run viewer's `node --check`, `node --test`, `tsc --checkJs` (twice: the
page, and its logic modules without the DOM) and prettier (its TypeScript, three.js types and
prettier pinned in `viewer/package-lock.json`, installed by `npm ci --prefix viewer`), jscpd
(duplication), and the C++, Rust and Python test suites. CI runs the same script, offline:
`scripts/offline_ci.sh` repeats `.github/workflows/test.yml` on a clean checkout with its own
env built from the lock, gates every PR, and its result goes in the PR template. The GitHub
workflow runs on demand and on pushes to `main` (the Actions minutes are shared with other
projects).

**PRs are slices, not steps.** A PR makes meaningful progress toward a stated goal: a slice
of a milestone (e.g. milestone 1's "BA spine"). The individual steps are semantic commits
inside it (`feat(foundations): ...`, `test(...)`, `docs(notes): ...`), each with layer 1
green. Prefer few, substantial PRs over many small ones.

**Layer 2, the review panel (`/slam-it-all-review-panel`):**
- **When:** once per PR, on the finished slice, against the PR's stated goal. Never after
  every incremental change: reviewing each step lets each run's findings steer the next step,
  and the work drifts. Not on MICRO PRs (fixes, docs, CI). A small PR that touches one area
  can run that reviewer alone: `/slam-it-all-review-panel algorithm | architecture | hygiene`.
- **Findings outside the PR's goal go to a backlog by default**, not into the PR.
- **Stop rule for a component:** it is done when its oracle and property tests pass within
  the stated tolerance, its named hard cases pass, and its consumer works.
- **Say which applies** when proposing a PR: full panel, one reviewer, or layer 1 only. The
  user may override either way.
- **Reviewers:** `slam-it-all-hygiene-reviewer` (sonnet), `slam-it-all-architecture-reviewer`
  (opus), `slam-it-all-algorithm-reviewer` (opus, with web search). All call out code that is not
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
- **Merging:** Claude may merge SLAM It All! PRs only after a selected **Yes** on a yes/no prompt
  for that specific PR, using `gh pr merge <n> --repo evan-wes/slam-it-all`. A typed instruction
  is never permission. Then verify that the commits landed on `develop`, check the tag, and
  ask before deleting the branch.
  This is a deliberate exception the owner granted on 2026-10-02 to their global "never
  merge a PR" rule. It is written into that rule in the owner's global Claude Code instructions ("The one
  exception: SLAM It All!"), and a PreToolUse hook (the owner's merge-guard hook) blocks
  every merge except this exact form. The hook matches the phrase anywhere in a shell
  command, so edit text that contains it with the file editor, not a shell heredoc.

## Privacy

The repo is **private** and stays private until it goes public together with reifyscene, on
the owner's explicit instruction.
