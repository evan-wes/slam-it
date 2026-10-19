#!/usr/bin/env bash
# Offline CI: what .github/workflows/test.yml does, on this machine, without GitHub Actions
# (whose minutes are shared with the owner's other projects). Every PR is gated by it; the
# GitHub workflow runs on demand and on pushes to main.
#
# CI's job is "layer 1 from a clean start": a fresh checkout (committed files only), the
# conda env built from scripts/locks/slam-it.lock by scripts/setup_slam_it.sh, then
# scripts/check.sh. This does the same:
#   1. A persistent checkout in ~/.cache/slam-it-offline-ci/slam-it, cloned from this repo
#      (no network), moved to REF and cleaned with `git clean -ffdx`, so no untracked file,
#      build tree or cache from earlier runs survives.
#   2. If BASE is not an ancestor of REF, REF is merged with BASE first, as GitHub tests a
#      PR's merge commit; a conflict fails the run.
#   3. The env `slam-it-ci` (never your working `slam-it` env), recreated from scratch when
#      the lock or setup script changed since it was built (or with --fresh-env); otherwise
#      setup_slam_it.sh updates it in place, as it would any existing env. Either way
#      setup_slam_it.sh then reinstalls SLAM It! and the Rust extension from the checkout.
#   4. scripts/check.sh --env slam-it-ci, exactly as CI runs it.
# Every step runs under an 8G memory cap with 4 build jobs, but one: setup's compile of GTSAM's
# Python wrapper, whose largest file needs more than 8 GB, runs in a sibling scope capped at 16G
# (decided by evan-wes on 2026-10-17, B3's review panel, A4). The log goes to
# ~/.cache/slam-it-offline-ci/logs/; a one-line result per run is appended to
# ~/.cache/slam-it-offline-ci/results.tsv (sha, ref, base merged?, result, log), to cite in
# a PR.
#
# Usage: scripts/offline_ci.sh [REF] [--base BASE] [--fresh-env]
#   REF   default: this repo's current branch head. BASE default: origin/develop.
#   Packages still download (conda, pip, crates); only the runner is local.
# Exit code: 0 if layer 1 passed, non-zero otherwise (setup failures included).
# Assumes conda at $CONDA_EXE, on PATH, or ~/miniconda3, and systemd-run --user for the cap.
# A full run takes roughly 30-40 min with a fresh env, less when it is reused.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${HOME}/.cache/slam-it-offline-ci"
CHECKOUT="${CACHE}/slam-it"
ENV_NAME="slam-it-ci"
CAP=(systemd-run --user --scope -q -p MemoryMax=8G -p MemorySwapMax=2G --)
# A scope systemd-run starts from inside CAP's is its sibling, not its child, so its own cap
# holds: setup_slam_it.sh runs the wrapper's compile through it, and nothing else.
export SLAM_IT_GTSAM_PYTHON_CAP="systemd-run --user --scope -q -p MemoryMax=16G -p MemorySwapMax=2G --"
export CMAKE_BUILD_PARALLEL_LEVEL=4 CARGO_BUILD_JOBS=4

REF="" BASE="origin/develop" FRESH=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --base) BASE="$2"; shift 2 ;;
    --fresh-env) FRESH=1; shift ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) REF="$1"; shift ;;
  esac
done
REF="${REF:-$(git -C "${REPO}" branch --show-current)}"
SHA="$(git -C "${REPO}" rev-parse --verify "${REF}^{commit}")" || { echo "no commit ${REF}" >&2; exit 2; }
BASE_SHA="$(git -C "${REPO}" rev-parse --verify "${BASE}^{commit}")" || { echo "no base ${BASE}" >&2; exit 2; }
CONDA_BIN="${CONDA_EXE:-$(command -v conda || echo "${HOME}/miniconda3/condabin/conda")}"
export CONDA_EXE="${CONDA_BIN}"

mkdir -p "${CACHE}/logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="${CACHE}/logs/${SHA:0:12}-${STAMP}.log"
exec > >(tee "${LOG}") 2>&1
echo "offline CI: ${REF} (${SHA}), base ${BASE} (${BASE_SHA:0:12}), log ${LOG}"

# --- 1. a clean checkout of committed files ------------------------------------------
if [[ ! -d "${CHECKOUT}/.git" ]]; then
  git clone --quiet --no-local "${REPO}" "${CHECKOUT}" || exit 1
fi
git -C "${CHECKOUT}" fetch --quiet --force "${REPO}" "${SHA}" "${BASE_SHA}" || exit 1
git -C "${CHECKOUT}" checkout --quiet --force --detach "${SHA}" || exit 1
git -C "${CHECKOUT}" clean -ffdxq || exit 1

# --- 2. GitHub tests a PR merged with its base ----------------------------------------
MERGED="no"
if ! git -C "${CHECKOUT}" merge-base --is-ancestor "${BASE_SHA}" "${SHA}"; then
  echo "${BASE} is not an ancestor of ${REF}: testing their merge, as GitHub would"
  git -C "${CHECKOUT}" -c user.name=offline-ci -c user.email=offline-ci@localhost \
    merge --quiet --no-ff --no-edit "${BASE_SHA}" || { echo "merge conflict with ${BASE}"; exit 1; }
  MERGED="yes"
fi

# --- 3. the env, from the lock --------------------------------------------------------
LOCK_HASH="$(cat "${CHECKOUT}/scripts/locks/slam-it.lock" "${CHECKOUT}/scripts/setup_slam_it.sh" | sha256sum | cut -c1-16)"
STAMP_FILE="${CACHE}/env.hash"
if "${CONDA_BIN}" env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  if [[ ${FRESH} -eq 1 || "$(cat "${STAMP_FILE}" 2>/dev/null)" != "${LOCK_HASH}" ]]; then
    echo "recreating ${ENV_NAME} (fresh requested, or the lock or setup script changed)"
    "${CONDA_BIN}" env remove -y -q -n "${ENV_NAME}" || exit 1
  fi
fi
(cd "${CHECKOUT}" && "${CAP[@]}" bash scripts/setup_slam_it.sh --env "${ENV_NAME}")
setup=$?
if [[ ${setup} -ne 0 ]]; then
  echo "SETUP FAILED (exit ${setup})"
  printf '%s\t%s\tmerged=%s\tsetup-failed\t%s\n' "${SHA}" "${REF}" "${MERGED}" "${LOG}" >> "${CACHE}/results.tsv"
  exit "${setup}"
fi
echo "${LOCK_HASH}" > "${STAMP_FILE}"

# --- 4. layer 1, as CI runs it ---------------------------------------------------------
(cd "${CHECKOUT}" && "${CAP[@]}" bash scripts/check.sh --env "${ENV_NAME}")
code=$?
result=$([[ ${code} -eq 0 ]] && echo passed || echo failed)
printf '%s\t%s\tmerged=%s\t%s\t%s\n' "${SHA}" "${REF}" "${MERGED}" "${result}" "${LOG}" >> "${CACHE}/results.tsv"
echo "offline CI ${result} for ${REF} (${SHA}) (exit ${code})"
exit "${code}"
