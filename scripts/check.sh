#!/usr/bin/env bash
set -uo pipefail

# Layer 1 of the review process: deterministic checks, run on every PR (locally and in CI).
# The reviewer panel (/review-panel) is layer 2 and runs only on larger changes.
#
# Runs every check even after a failure, then exits non-zero if any failed, so one run
# reports everything. Expects the slamlab conda env (scripts/setup_slamlab.sh) to be active,
# or pass --env NAME to run each tool through `conda run`.
#
# Usage:
#   scripts/check.sh [--env NAME]

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

RUN=()
if [[ "${1:-}" == "--env" ]]; then
  CONDA_BIN="${CONDA_EXE:-$(command -v conda || echo "${HOME}/miniconda3/condabin/conda")}"
  RUN=("${CONDA_BIN}" run --no-capture-output -n "$2")
fi

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

CPP_SOURCES=(cpp/include/slamlab/*.hpp cpp/src/*.cpp cpp/bindings/*.cpp cpp/tests/*.cpp)

# Python
check "ruff lint (numpy docstrings, type hints, style)" ruff check src tests
check "ruff format" ruff format --check src tests
check "mypy --strict" mypy
check "import-linter (dependency rules)" lint-imports

# C++
check "clang-format" clang-format --dry-run --Werror "${CPP_SOURCES[@]}"
check "cmake configure (compile_commands.json)" \
  cmake -S . -B build/native -G Ninja -DSLAMLAB_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
check "C++ build" cmake --build build/native -j4
check "clang-tidy" clang-tidy -p build/native --quiet cpp/src/*.cpp cpp/tests/*.cpp
check "doxygen (every declaration documented)" doxygen Doxyfile
check "C++ tests (ctest)" ctest --test-dir build/native --output-on-failure

# Rust
check "rustfmt" cargo fmt --manifest-path rust/Cargo.toml --all --check
check "clippy (pedantic, missing docs)" \
  cargo clippy --manifest-path rust/Cargo.toml --workspace --all-targets
check "Rust tests" cargo test --manifest-path rust/Cargo.toml -p slamlab-core

# All languages
check "jscpd (duplication)" jscpd --config .jscpd.json
check "pytest (parity, doctests)" python -m pytest -q

printf '\n'
if [[ ${#failed[@]} -gt 0 ]]; then
  printf 'FAILED (%d): %s\n' "${#failed[@]}" "$(IFS=', '; echo "${failed[*]}")"
  exit 1
fi
echo "All layer-1 checks passed."
