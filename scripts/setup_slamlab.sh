#!/usr/bin/env bash
set -euo pipefail

# Builds the slamlab conda environment and installs slamlab into it.
#
# Two layers, as in reifyscene's setup:
# - conda layer (conda-forge): Python and the native toolchain -- cmake, ninja, a C++
#   compiler, Rust (cargo/rustc) -- plus the layer-1 check tools: clang-format/clang-tidy
#   (with clang's own headers), doxygen, and Node.js for jscpd. Nothing is installed
#   system-wide; removing the env removes the toolchain.
# - pip layer: build backends and Python dependencies, installed from
#   scripts/locks/slamlab.lock by default (pass --unlocked to re-resolve).
#
# Then two editable installs:
# - slamlab itself, with its C++ extension slamlab._cpp (scikit-build-core + nanobind),
#   rebuilt automatically on import when C++ sources change;
# - slamlab_rs, the Rust extension (maturin + PyO3), rebuilt with
#   `maturin develop -m rust/slamlab-py/Cargo.toml` after Rust changes.
#
# Usage:
#   scripts/setup_slamlab.sh [--env NAME] [--unlocked]

ENV_NAME="slamlab"
PYTHON_VERSION="3.11"   # matches reifyscene's env, which also imports slamlab
UNLOCKED=0
CLANG_VERSION="23.1.2"
JSCPD_VERSION="5.4.0"

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
LOCK_FILE="${ROOT_DIR}/scripts/locks/slamlab.lock"

CONDA_BIN="${CONDA_EXE:-$(command -v conda || echo "${HOME}/miniconda3/condabin/conda")}"
if [[ ! -x "${CONDA_BIN}" ]]; then
  echo "conda executable not found (checked CONDA_EXE, PATH, ~/miniconda3)." >&2
  exit 1
fi

# clang and clang-tools pinned together: clang-tidy needs clang's resource headers.
CONDA_PACKAGES=(cmake ninja cxx-compiler rust "clang-tools=${CLANG_VERSION}" "clang=${CLANG_VERSION}"
  doxygen nodejs)

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
  # Resolve fresh from pyproject.toml: the runtime dependencies plus the `dev` extra,
  # the single list of the toolchain's pip layer.
  mapfile -t PIP_REQS < <(run python -c '
import tomllib
project = tomllib.load(open("'"${ROOT_DIR}"'/pyproject.toml", "rb"))["project"]
print("\n".join(project["dependencies"] + project["optional-dependencies"]["dev"]))')
  run python -m pip install "${PIP_REQS[@]}"
fi
# jscpd (duplication detection) installs into the env's own Node prefix.
run npm install -g --silent "jscpd@${JSCPD_VERSION}"

# --- slamlab -------------------------------------------------------------------
# --no-build-isolation: the editable C++ build rebuilds on import, so its build
# backend must live in this env rather than a throwaway isolated one.
# build-dir per env: envs with the same Python version would otherwise share one
# CMake build tree and silently reuse each other's cached compiler and paths.
run python -m pip install --no-build-isolation \
  -C build-dir="build/${ENV_NAME}" -e "${ROOT_DIR}"
(cd "${ROOT_DIR}" && run maturin develop --release -m rust/slamlab-py/Cargo.toml)

# Every backend must load: a C++ or Rust extension that built but cannot import fails here.
run python -c "
import slamlab
from slamlab import backends
found = backends.available()
print('slamlab', slamlab.__version__, 'backends:', found)
assert found == list(backends.BACKENDS), f'expected {list(backends.BACKENDS)}, got {found}'
"
