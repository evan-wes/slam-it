#!/usr/bin/env bash
set -uo pipefail

# Layer 1 of the review process: deterministic checks, run on every PR, locally and in CI
# (scripts/offline_ci.sh runs it from a clean checkout, as .github/workflows/test.yml does).
# The review panel (/slam-it-review-panel) is layer 2 and runs only on larger changes.
#
# Runs every check even after a failure, then exits non-zero if any failed, so one run
# reports everything. Expects the slam-it conda env (scripts/setup_slam_it.sh) to be active,
# or pass --env NAME to run each tool through `conda run`.
#
# Every backend is required: SLAM_IT_REQUIRE_BACKENDS makes a C++ or Rust extension that is
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
export SLAM_IT_REQUIRE_BACKENDS="python,cpp,rust"

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
# The Rust core's C++ bridge (rust/*/cxx/) is formatted and documented as the C++ core is; cargo
# compiles it, so it has no compile commands for clang-tidy.
mapfile -t CPP_FILES < <(git ls-files --cached --others --exclude-standard 'cpp/*.hpp' 'cpp/*.cpp' \
  'rust/*/cxx/*.hpp' 'rust/*/cxx/*.cpp')
mapfile -t CPP_TU < <(git ls-files --cached --others --exclude-standard 'cpp/src/*.cpp' 'cpp/tests/*.cpp' \
  'cpp/apps/*.cpp')
mapfile -t BENCH_TU < <(git ls-files --cached --others --exclude-standard 'cpp/bench/*.cpp')
mapfile -t JS_FILES < <(git ls-files --cached --others --exclude-standard 'viewer/*.js')

# Python
check "ruff lint (numpy docstrings, type hints, style)" ruff check src server tests scripts
check "ruff format" ruff format --check src server tests scripts
check "mypy --strict" mypy
check "import-linter (dependency rules)" lint-imports
check "C++/Rust layering (same contracts)" python scripts/check_layers.py

# C++
check "clang-format" clang-format --dry-run --Werror "${CPP_FILES[@]}"
# The native tests run hardened: libstdc++'s assertions (bounds of spans and containers) catch
# an out-of-range access that a release build would run silently (PR 9's panel, L1). The
# benchmarks build apart, unhardened, in build/bench, the tree to time from, so a timing never
# pays for the checks (PR 9's panel, A12).
check "cmake configure (compile_commands.json)" \
  cmake -S . -B build/native -G Ninja -DSLAM_IT_BUILD_TESTS=ON \
  -DSLAM_IT_BUILD_BENCHMARKS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-D_GLIBCXX_ASSERTIONS
check "cmake configure, benchmarks (unhardened)" \
  cmake -S . -B build/bench -G Ninja -DSLAM_IT_BUILD_TESTS=OFF \
  -DSLAM_IT_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=
check "C++ build" cmake --build build/native -j4
check "C++ benchmarks build" cmake --build build/bench -j4
# clang-tidy's clang does not know conda gcc's implicit include directory (<env>/include),
# where the test-only headers live (nlohmann_json); pass it explicitly.
ENV_INCLUDE="$("${RUN[@]}" python -c 'import os, sys; print(os.path.join(sys.prefix, "include"))')"
check "clang-tidy" clang-tidy -p build/native --quiet --extra-arg=-isystem"${ENV_INCLUDE}" \
  "${CPP_TU[@]}"
check "clang-tidy (benchmarks)" clang-tidy -p build/bench --quiet \
  --extra-arg=-isystem"${ENV_INCLUDE}" "${BENCH_TU[@]}"
check "doxygen (every declaration documented)" doxygen Doxyfile
check "C++ tests (ctest)" ctest --test-dir build/native --output-on-failure
# The C++ tests once more under ThreadSanitizer (the online-SLAM design's section 12, item 13):
# built apart in build/tsan, with debug information for the reports' stacks, so that a data
# race in the threads' handshakes or the threaded pipeline fails the check; both of the
# pipeline's threaded schemes run there (cpp/tests/edge/test_pipeline.cpp). Rust's threads are
# safe code. GCC's -Wmaybe-uninitialized misfires at -O2 on Eigen matrices held in a
# std::optional (RANSAC's best model), so that warning is off in this build alone.
check "cmake configure, ThreadSanitizer" \
  cmake -S . -B build/tsan -G Ninja -DSLAM_IT_BUILD_TESTS=ON -DSLAM_IT_BUILD_BENCHMARKS=OFF \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCMAKE_CXX_FLAGS=-fsanitize=thread -Wno-maybe-uninitialized" \
  -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread
check "C++ build, ThreadSanitizer" cmake --build build/tsan -j4
check "C++ tests under ThreadSanitizer (ctest)" \
  env TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1 \
  ctest --test-dir build/tsan --output-on-failure
# The native tests' scene has no loop, so once more with loops on (B3b's review panel, A3): the
# loop room threaded through the instrumented program, in both schemes, so that a loop's stop and
# correction, a global BA's thread and its result carried onto the map run under the sanitizer.
check "C++ loop room threaded under ThreadSanitizer" \
  env SLAM_IT_TSAN_VO=build/tsan/slam_it_vo \
  python -m pytest -q -m tsan tests/orchestration/test_loop_room_tsan.py

# Rust
check "rustfmt" cargo fmt --manifest-path rust/Cargo.toml --all --check
check "clippy (pedantic, missing docs)" \
  cargo clippy --manifest-path rust/Cargo.toml --workspace --all-targets
# Every rung crate; the cdylib bindings crate cannot be tested standalone.
check "Rust tests" cargo test --manifest-path rust/Cargo.toml --workspace --exclude slam-it-py
# The native program the rust backend runs (slam_it.backends.program), rebuilt for pytest,
# always into rust/target (whatever CARGO_TARGET_DIR says), where slam_it.backends looks.
check "Rust programs (release)" \
  cargo build --release --manifest-path rust/Cargo.toml --target-dir rust/target \
  -p slam-it-orchestration

# JavaScript: the run viewer's page, its tools installed by `npm ci --prefix viewer`
check "node --check (viewer)" bash -c 'for file; do node --check "$file" || exit 1; done' _ \
  "${JS_FILES[@]}"
# The test files by name: a folder is a module to some versions of Node, not a test directory.
check "node --test (viewer)" node --test viewer/tests/*.test.js
check "tsc --checkJs (viewer)" viewer/node_modules/.bin/tsc -p viewer
# The logic modules again, without the DOM: they never touch the page, so Node tests them.
check "tsc --checkJs, logic without the DOM (viewer)" \
  viewer/node_modules/.bin/tsc -p viewer/tsconfig.logic.json
check "prettier (viewer)" viewer/node_modules/.bin/prettier --check \
  viewer/src viewer/tests viewer/index.html viewer/upload.html viewer/status.html viewer/app.css \
  viewer/logic.d.ts

# All languages
# Docstrings, doc comments, `#include`s and Rust `use`s are not counted (.jscpd.json): files
# that bind one library import the same headers, which is not duplicated code.
check "jscpd (duplication)" jscpd --config .jscpd.json
check "pytest (parity, doctests; CPU subset)" \
  python -m pytest -q -m "not gpu and not budget and not tsan"

printf '\n'
if [[ ${#failed[@]} -gt 0 ]]; then
  printf 'FAILED (%d): %s\n' "${#failed[@]}" "$(IFS=', '; echo "${failed[*]}")"
  exit 1
fi
echo "All layer-1 checks passed."
