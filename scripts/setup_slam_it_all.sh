#!/usr/bin/env bash
set -euo pipefail

# Builds the slam-it-all conda environment and installs slam-it-all into it.
#
# Two layers, as in reifyscene's setup:
# - conda layer (conda-forge): Python and the native toolchain -- cmake, ninja, a C++
#   compiler, Rust (cargo/rustc) -- plus the layer-1 check tools: clang-format/clang-tidy
#   (with clang's own headers), Eigen, doxygen, and Node.js for jscpd. Nothing is installed
#   system-wide; removing the env removes the toolchain.
# - pip layer: build backends and Python dependencies, installed from
#   scripts/locks/slam-it-all.lock by default (pass --unlocked to re-resolve).
#
# Then two editable installs:
# - SLAM It All! itself, with its C++ extension slam_it_all._cpp (scikit-build-core + nanobind),
#   rebuilt automatically on import when C++ sources change;
# - slam_it_all_rs, the Rust extension (maturin + PyO3), rebuilt with
#   `maturin develop -m rust/slam-it-all-py/Cargo.toml` after Rust changes.
# and the native programs: C++'s are installed with SLAM It All! (and rebuilt with it); Rust's are
# built into rust/target with `cargo build --release --target-dir rust/target
# -p slam-it-all-orchestration` (scripts/check.sh rebuilds them; nothing does on import).
#
# Usage:
#   scripts/setup_slam_it_all.sh [--env NAME] [--unlocked]

ENV_NAME="slam-it-all"
PYTHON_VERSION="3.11"   # matches reifyscene's env, which also imports slam_it_all
UNLOCKED=0
CLANG_VERSION="23.1.2"
JSCPD_VERSION="5.4.0"
EIGEN_VERSION="5.0.1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)
      [[ $# -ge 2 && -n "$2" ]] || { echo "--env needs an environment name" >&2; exit 2; }
      ENV_NAME="$2"; shift 2 ;;
    --unlocked) UNLOCKED=1; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK_FILE="${ROOT_DIR}/scripts/locks/slam-it-all.lock"

CONDA_BIN="${CONDA_EXE:-$(command -v conda || echo "${HOME}/miniconda3/condabin/conda")}"
if [[ ! -x "${CONDA_BIN}" ]]; then
  echo "conda executable not found (checked CONDA_EXE, PATH, ~/miniconda3)." >&2
  exit 1
fi

# clang and clang-tools pinned together: clang-tidy needs clang's resource headers.
# Eigen is the C++ cores' linear algebra (header-only; found by CMake as Eigen3::Eigen).
# manifpy is a test-only Lie-group oracle, packaged for conda-forge only.
# nlohmann_json lets the native C++ tests read the shared parity fixtures (test-only);
# benchmark is Google Benchmark, for the native C++ benchmarks (cpp/bench/).
CONDA_PACKAGES=(cmake ninja cxx-compiler rust "clang-tools=${CLANG_VERSION}" "clang=${CLANG_VERSION}"
  "eigen=${EIGEN_VERSION}" doxygen nodejs manifpy nlohmann_json benchmark)

# --- conda layer ---------------------------------------------------------------
if "${CONDA_BIN}" env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  echo "conda env '${ENV_NAME}' exists; updating its toolchain."
  "${CONDA_BIN}" install -y -n "${ENV_NAME}" -c conda-forge "${CONDA_PACKAGES[@]}"
else
  "${CONDA_BIN}" create -y -n "${ENV_NAME}" -c conda-forge \
    "python=${PYTHON_VERSION}" "${CONDA_PACKAGES[@]}"
fi

# conda-forge's compilers ship no activation script that sets CC/CXX, so CMake would
# silently pick the system compiler. Pin the env's own compilers as env variables. Their
# names carry the host architecture (x86_64-conda-linux-gnu-cc, aarch64-conda-linux-gnu-cc).
ENV_PREFIX="$("${CONDA_BIN}" run -n "${ENV_NAME}" python -c 'import sys; print(sys.prefix)')"
HOST_TRIPLE="$(uname -m)-conda-linux-gnu"
for tool in cc c++; do
  [[ -x "${ENV_PREFIX}/bin/${HOST_TRIPLE}-${tool}" ]] || {
    echo "conda compiler ${HOST_TRIPLE}-${tool} not found in ${ENV_PREFIX}/bin" >&2; exit 1; }
done
"${CONDA_BIN}" env config vars set -n "${ENV_NAME}" \
  CC="${ENV_PREFIX}/bin/${HOST_TRIPLE}-cc" \
  CXX="${ENV_PREFIX}/bin/${HOST_TRIPLE}-c++" >/dev/null

run() { "${CONDA_BIN}" run --no-capture-output -n "${ENV_NAME}" "$@"; }

# --- pip layer -----------------------------------------------------------------
if [[ ${UNLOCKED} -eq 0 && -f "${LOCK_FILE}" ]]; then
  run python -m pip install -r "${LOCK_FILE}"
else
  # Resolve fresh from pyproject.toml: the runtime dependencies plus the `dev` extra (the
  # toolchain's pip layer), the `colmap` extra (the BA reference the tests compare to) and
  # the `video` extra (PyAV, for importing phone videos); then the server package's own.
  mapfile -t PIP_REQS < <(run python -c '
import tomllib
project = tomllib.load(open("'"${ROOT_DIR}"'/pyproject.toml", "rb"))["project"]
extras = project["optional-dependencies"]
print("\n".join(project["dependencies"] + extras["dev"] + extras["colmap"] + extras["video"]))')
  run python -m pip install "${PIP_REQS[@]}"
fi
# jscpd (duplication detection) installs into the env's own Node prefix.
run npm install -g --silent "jscpd@${JSCPD_VERSION}"
# The run viewer's type check (TypeScript and three.js's types), pinned by its lockfile, into
# viewer/node_modules: the page itself has no build step.
run npm ci --prefix "${ROOT_DIR}/viewer" --no-audit --no-fund

# --- slam-it-all ---------------------------------------------------------------
# --no-build-isolation: the editable C++ build rebuilds on import, so its build
# backend must live in this env rather than a throwaway isolated one.
# build-dir per env: envs with the same Python version would otherwise share one
# CMake build tree and silently reuse each other's cached compiler and paths.
run python -m pip install --no-build-isolation \
  -C build-dir="build/${ENV_NAME}" -e "${ROOT_DIR}"
# The upload server, a package of its own (server/), with its test client.
run python -m pip install -e "${ROOT_DIR}/server[test]"
(cd "${ROOT_DIR}" && run maturin develop --release -m rust/slam-it-all-py/Cargo.toml)
(cd "${ROOT_DIR}" && run cargo build --release --manifest-path rust/Cargo.toml \
  --target-dir rust/target -p slam-it-all-orchestration)

# Every backend must load, and find its programs: a C++ or Rust extension that built but
# cannot import, or a program that did not build, fails here.
run python -c "
import slam_it_all
from slam_it_all import backends
found = backends.available()
print('slam_it_all', slam_it_all.__version__, 'backends:', found)
assert found == list(backends.BACKENDS), f'expected {list(backends.BACKENDS)}, got {found}'
for name in backends.COMPILED_BACKENDS:
    print(name, 'slam_it_all_vo:', backends.program(name, 'slam_it_all_vo'))
"
