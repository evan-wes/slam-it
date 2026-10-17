#!/usr/bin/env bash
set -euo pipefail

# Builds the slam-it conda environment and installs slam-it into it.
#
# Two layers, as in reifyscene's setup:
# - conda layer (conda-forge): Python and the native toolchain -- cmake, ninja, a C++
#   compiler, Rust (cargo/rustc) -- plus the layer-1 check tools: clang-format/clang-tidy
#   (with clang's own headers), Eigen, doxygen, and Node.js for jscpd. Nothing is installed
#   system-wide; removing the env removes the toolchain.
# - pip layer: build backends and Python dependencies, installed from
#   scripts/locks/slam-it.lock by default (pass --unlocked to re-resolve).
#
# Then GTSAM, built from its release into the env (C++ library and Python wrapper; see its
# section), and two editable installs:
# - SLAM It! itself, with its C++ extension slam_it._cpp (scikit-build-core + nanobind),
#   rebuilt automatically on import when C++ sources change;
# - slam_it_rs, the Rust extension (maturin + PyO3), rebuilt with
#   `maturin develop -m rust/slam-it-py/Cargo.toml` after Rust changes.
# and the native programs: C++'s are installed with SLAM It! (and rebuilt with it); Rust's are
# built into rust/target with `cargo build --release --target-dir rust/target
# -p slam-it-orchestration` (scripts/check.sh rebuilds them; nothing does on import).
#
# Usage:
#   scripts/setup_slam_it.sh [--env NAME] [--unlocked]

ENV_NAME="slam-it"
PYTHON_VERSION="3.11"   # matches reifyscene's env, which also imports slam_it
UNLOCKED=0
CLANG_VERSION="23.1.2"
RUST_VERSION="1.99.0"   # pinned: a new release adds clippy lints, and CI builds its env afresh
JSCPD_VERSION="5.4.0"
EIGEN_VERSION="5.0.1"
# GTSAM, rung 0's factor-graph back end (slam_it.foundations.graphs, from milestone 2B's B3),
# built here from its release tarball (checked by its SHA-256, conda-forge's recipe's), not
# taken from conda-forge: see "GTSAM" below.
GTSAM_VERSION="4.3.0"
GTSAM_SHA256="4539afcd47d336864deb8399e0a9a5c283a638f605892c942ec46f3541fd4fb0"

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
LOCK_FILE="${ROOT_DIR}/scripts/locks/slam-it.lock"

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
# pyparsing is what GTSAM's wrapper generator (gtwrap) parses its interface files with, to
# build GTSAM's Python wrapper below.
CONDA_PACKAGES=(cmake ninja cxx-compiler "rust=${RUST_VERSION}" "clang-tools=${CLANG_VERSION}" "clang=${CLANG_VERSION}"
  "eigen=${EIGEN_VERSION}" doxygen nodejs manifpy nlohmann_json benchmark pyparsing)

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
  # GTSAM is left out: it is built below, never taken from an index.
  mapfile -t PIP_REQS < <(run python -c '
import tomllib
project = tomllib.load(open("'"${ROOT_DIR}"'/pyproject.toml", "rb"))["project"]
extras = project["optional-dependencies"]
runtime = [req for req in project["dependencies"] if not req.startswith("gtsam")]
print("\n".join(runtime + extras["dev"] + extras["colmap"] + extras["video"]))')
  run python -m pip install "${PIP_REQS[@]}"
fi
# jscpd (duplication detection) installs into the env's own Node prefix.
run npm install -g --silent "jscpd@${JSCPD_VERSION}"
# The run viewer's type check (TypeScript and three.js's types), pinned by its lockfile, into
# viewer/node_modules: the page itself has no build step.
run npm ci --prefix "${ROOT_DIR}/viewer" --no-audit --no-fund

# --- GTSAM ---------------------------------------------------------------------
# Built here, not taken from conda-forge, so that it repeats on every core: conda-forge's
# build blocks Eigen's products by the caches of the core that runs first, and on a hybrid CPU
# its global BA came out different in the last bits on a P-core and an E-core (2026-10-17);
# this one compiles with EIGEN_NO_CPUID, as the cores do (CMakeLists.txt). Static, METIS too, and
# position-independent, so the C++ extension, the native programs and the Rust bridge each carry
# it and need no library path at run time; with its own CCOLAMD and without CHOLMOD, never
# whatever SuiteSparse the env happens to hold; single-threaded (no TBB), like the cores; no Boost
# features; the Python wrapper built over the same library and installed with pip. Its
# pybind11 sources compile one at a time: the largest needs more than 8 GB. Skipped when this
# env already holds the build the options below make.
GTSAM_OPTIONS=(-DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-DEIGEN_NO_CPUID -ffp-contract=off"
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DGTSAM_BUILD_WITH_WERROR=OFF
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
  -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_INSTALL_CPPUNITLITE=OFF -DGTSAM_BUILD_PYTHON=ON
  -DGTSAM_USE_SYSTEM_CCOLAMD=OFF -DCMAKE_DISABLE_FIND_PACKAGE_CHOLMOD=ON)
GTSAM_BUILD_ID="${GTSAM_VERSION} ${GTSAM_SHA256} ${GTSAM_OPTIONS[*]}"
GTSAM_MARKER="${ENV_PREFIX}/share/slam-it/gtsam-build"
if [[ -f "${GTSAM_MARKER}" && "$(cat "${GTSAM_MARKER}")" == "${GTSAM_BUILD_ID}" ]]; then
  echo "GTSAM ${GTSAM_VERSION}: this env's build is current."
else
  # An env set up before GTSAM was built here holds conda-forge's package: remove it first,
  # so the two never share the prefix.
  # (Our own wrapper is pip's, which `conda list` reports from the pypi channel: left alone.)
  if "${CONDA_BIN}" list -n "${ENV_NAME}" --json | "${ENV_PREFIX}/bin/python" -c '
import json, sys
sys.exit(0 if any(p["name"] == "gtsam" and p.get("channel") != "pypi" for p in json.load(sys.stdin))
         else 1)'; then
    "${CONDA_BIN}" remove -y -n "${ENV_NAME}" gtsam
  fi
  GTSAM_DIR="${ROOT_DIR}/build/gtsam-${GTSAM_VERSION}"
  GTSAM_TARBALL="${GTSAM_DIR}/gtsam-${GTSAM_VERSION}.tar.gz"
  mkdir -p "${GTSAM_DIR}"
  if [[ ! -f "${GTSAM_TARBALL}" ]]; then
    curl -fsSL "https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz" \
      -o "${GTSAM_TARBALL}.part"
    mv "${GTSAM_TARBALL}.part" "${GTSAM_TARBALL}"
  fi
  echo "${GTSAM_SHA256}  ${GTSAM_TARBALL}" | sha256sum -c --quiet
  rm -rf "${GTSAM_DIR}/src" "${GTSAM_DIR}/build"
  mkdir -p "${GTSAM_DIR}/src"
  tar -xzf "${GTSAM_TARBALL}" -C "${GTSAM_DIR}/src" --strip-components=1
  run cmake -S "${GTSAM_DIR}/src" -B "${GTSAM_DIR}/build" -G Ninja \
    -DCMAKE_INSTALL_PREFIX="${ENV_PREFIX}" -DCMAKE_PREFIX_PATH="${ENV_PREFIX}" \
    -DCMAKE_C_COMPILER="${ENV_PREFIX}/bin/${HOST_TRIPLE}-cc" \
    -DCMAKE_CXX_COMPILER="${ENV_PREFIX}/bin/${HOST_TRIPLE}-c++" \
    -DPython3_EXECUTABLE="${ENV_PREFIX}/bin/python" "${GTSAM_OPTIONS[@]}"
  # The library and METIS with four jobs, the wrapper one file at a time; installing copies
  # what they built, never building GTSAM's other targets (its timing tools).
  run ninja -C "${GTSAM_DIR}/build" -j4 gtsam metis-gtsam
  run ninja -C "${GTSAM_DIR}/build" -j1 gtsam_py
  run cmake --install "${GTSAM_DIR}/build"
  (cd "${GTSAM_DIR}/build/python" && run python -m pip install . --no-deps --no-build-isolation)
  mkdir -p "$(dirname "${GTSAM_MARKER}")"
  echo "${GTSAM_BUILD_ID}" > "${GTSAM_MARKER}"
fi

# --- slam-it ---------------------------------------------------------------
# --no-build-isolation: the editable C++ build rebuilds on import, so its build
# backend must live in this env rather than a throwaway isolated one.
# build-dir per env: envs with the same Python version would otherwise share one
# CMake build tree and silently reuse each other's cached compiler and paths.
run python -m pip install --no-build-isolation \
  -C build-dir="build/${ENV_NAME}" -e "${ROOT_DIR}"
# Gyroflow's phone lens profiles (CC0), at the commit slam_it.eval.lenses pins, into the
# user's cache: the source of a phone video's intrinsics. Skipped when already there.
run python -m slam_it.eval.lenses fetch
# The upload server, a package of its own (server/), with its test client.
run python -m pip install -e "${ROOT_DIR}/server[test]"
(cd "${ROOT_DIR}" && run maturin develop --release -m rust/slam-it-py/Cargo.toml)
(cd "${ROOT_DIR}" && run cargo build --release --manifest-path rust/Cargo.toml \
  --target-dir rust/target -p slam-it-orchestration)

# Every backend must load, and find its programs: a C++ or Rust extension that built but
# cannot import, or a program that did not build, fails here.
run python -c "
import slam_it
from slam_it import backends
found = backends.available()
print('slam_it', slam_it.__version__, 'backends:', found)
assert found == list(backends.BACKENDS), f'expected {list(backends.BACKENDS)}, got {found}'
for name in backends.COMPILED_BACKENDS:
    print(name, 'slam_it_vo:', backends.program(name, 'slam_it_vo'))
"
