#!/usr/bin/env bash
set -uo pipefail

# Layer 1 of the review process: deterministic checks, run on every PR (locally and in CI).
# The reviewer panel (/review-panel) is layer 2 and runs only on larger changes.
#
# Runs every check even after a failure, then exits non-zero if any failed, so one run
# reports everything. Expects the slamlab conda env (scripts/setup_slamlab.sh) to be active,
# or pass --env NAME to run each tool through `conda run`.
#
# Every backend is required: SLAMLAB_REQUIRE_BACKENDS makes a C++ or Rust extension that is
# missing or fails to load a test failure, not a skipped parity test.
#
# Usage:
#   scripts/check.sh [--env NAME]

ENV_NAME=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)
      [[ $# -ge 2 && -n "$2" ]] || { echo "--env needs an environment name" >&2; exit 2; }
      ENV_NAME="$2"; shift 2 ;;
    *) echo "Unknown argument: $1 (usage: scripts/check.sh [--env NAME])" >&2; exit 2 ;;
  esac
done

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

RUN=()
if [[ -n "${ENV_NAME}" ]]; then
  CONDA_BIN="${CONDA_EXE:-$(command -v conda || echo "${HOME}/miniconda3/condabin/conda")}"
  RUN=("${CONDA_BIN}" run --no-capture-output -n "${ENV_NAME}")
fi
export SLAMLAB_REQUIRE_BACKENDS="python,cpp,rust"

failed=()
check() {
  local name="$1"; shift
  printf '\n== %s\n' "${name}"
  if "${RUN[@]}" "$@"; then
    echo "-- ${name}: ok"
  else
    echo "-- ${name}: FAILED"
    failed+=("${name}")
  fi
}

# Source lists come from git (tracked plus new, unignored files), so new rung directories are
# covered without editing this file.
mapfile -t CPP_FILES < <(git ls-files --cached --others --exclude-standard 'cpp/*.hpp' 'cpp/*.cpp')
mapfile -t CPP_TU < <(git ls-files --cached --others --exclude-standard 'cpp/src/*.cpp' 'cpp/tests/*.cpp')

# Python
check "ruff lint (numpy docstrings, type hints, style)" ruff check src tests
check "ruff format" ruff format --check src tests
check "mypy --strict" mypy
check "import-linter (dependency rules)" lint-imports

# C++
check "clang-format" clang-format --dry-run --Werror "${CPP_FILES[@]}"
check "cmake configure (compile_commands.json)" \
  cmake -S . -B build/native -G Ninja -DSLAMLAB_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
check "C++ build" cmake --build build/native -j4
check "clang-tidy" clang-tidy -p build/native --quiet "${CPP_TU[@]}"
check "doxygen (every declaration documented)" doxygen Doxyfile
check "C++ tests (ctest)" ctest --test-dir build/native --output-on-failure

# Rust
check "rustfmt" cargo fmt --manifest-path rust/Cargo.toml --all --check
check "clippy (pedantic, missing docs)" \
  cargo clippy --manifest-path rust/Cargo.toml --workspace --all-targets
check "Rust tests" cargo test --manifest-path rust/Cargo.toml -p slamlab-core

# All languages
check "jscpd (duplication)" jscpd --config .jscpd.json
check "pytest (parity, doctests; CPU subset)" python -m pytest -q -m "not gpu and not budget"

printf '\n'
if [[ ${#failed[@]} -gt 0 ]]; then
  printf 'FAILED (%d): %s\n' "${#failed[@]}" "$(IFS=', '; echo "${failed[*]}")"
  exit 1
fi
echo "All layer-1 checks passed."
