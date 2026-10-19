# The worker's image (docs/design/2026-10-09-run-viewer.md, section 5.6): slam_it with its C++
# program and PyAV, and the lens profiles. It holds no secret but the worker token, given in the
# environment with the server's address:
#
#   docker build -f docker/worker.Dockerfile -t slam-it-worker .
#   docker run -e SLAM_IT_WORKER_TOKEN=... -e SLAM_IT_SERVER=https://... slam-it-worker
#
# Built from the repository's root, in two stages: the first compiles the wheel, the second
# installs it without a compiler.

# Python 3.11, as the env's lock; pinned by digest, so a rebuild is of the same base.
ARG PYTHON=python:3.11-slim-trixie@sha256:e88e9763f943ec1834f992a4b51e0f24500486803e8bc534e5767af9ea65f6ce

FROM ${PYTHON} AS build
RUN apt-get update \
    && apt-get install -y --no-install-recommends g++ cmake ninja-build git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
# Eigen 5.0.1, the version the conda env pins, checked out at its tag's commit (header-only:
# installing it builds nothing).
ARG EIGEN_COMMIT=bc3b39870ecb690a623a3f49149a358b95c5781d
RUN git clone --quiet --depth 1 --branch 5.0.1 https://gitlab.com/libeigen/eigen.git /tmp/eigen \
    && test "$(git -C /tmp/eigen rev-parse HEAD)" = "${EIGEN_COMMIT}" \
    && cmake -S /tmp/eigen -B /tmp/eigen/build -G Ninja -DBUILD_TESTING=OFF \
        -DEIGEN_BUILD_TESTING=OFF -DEIGEN_BUILD_BLAS=OFF -DEIGEN_BUILD_LAPACK=OFF \
        -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_DEMOS=OFF \
    && cmake --install /tmp/eigen/build \
    && rm -rf /tmp/eigen
# GTSAM 4.3.0, rung 0's factor-graph back end, at its tag's commit, built as
# scripts/setup_slam_it.sh builds it (static, position-independent, EIGEN_NO_CPUID, no TBB, no
# Boost features), for the C++ program the worker runs, which links it. Its Python wrapper is
# not built here: Python's slam_it imports GTSAM (slam_it.foundations.graphs, which the worker's
# pipeline imports) but never solves with it in the worker, so the final stage installs PyPI's
# build, pinned by its hash (scripts/locks/worker-gtsam.lock). tests/test_build_copies.py holds
# the version and options equal to the setup script's GTSAM_VERSION and GTSAM_OPTIONS.
ARG GTSAM_COMMIT=71a25ca36c084cbad1f872e812d6d97fbadfdb05
RUN git clone --quiet --depth 1 --branch 4.3.0 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && test "$(git -C /tmp/gtsam rev-parse HEAD)" = "${GTSAM_COMMIT}" \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
        "-DCMAKE_CXX_FLAGS=-DEIGEN_NO_CPUID -ffp-contract=off" \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DGTSAM_BUILD_WITH_WERROR=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_INSTALL_CPPUNITLITE=OFF -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_USE_SYSTEM_CCOLAMD=OFF -DCMAKE_DISABLE_FIND_PACKAGE_CHOLMOD=ON \
    && cmake --build /tmp/gtsam/build -j4 --target gtsam metis-gtsam \
    && cmake --install /tmp/gtsam/build \
    && rm -rf /tmp/gtsam
COPY . /src
# The lock pins the build's own tools (scikit-build-core, nanobind) too; four compile jobs.
ENV PIP_CONSTRAINT=/src/scripts/locks/slam-it.lock CMAKE_BUILD_PARALLEL_LEVEL=4
RUN python -m pip wheel --no-cache-dir --no-deps --wheel-dir /wheels /src

FROM ${PYTHON}
# tini as PID 1 reaps any process a killed job leaves behind, which Python as PID 1 would not.
RUN apt-get update \
    && apt-get install -y --no-install-recommends tini \
    && rm -rf /var/lib/apt/lists/*
COPY scripts/locks/slam-it.lock scripts/locks/worker-gtsam.lock /tmp/
COPY --from=build /wheels /tmp/wheels
# GTSAM's wrapper first, alone, its hash checked; then the wheel, which finds it installed and
# pins the rest, GTSAM's dependencies included, by the env's lock.
RUN python -m pip install --no-cache-dir --no-deps --only-binary=:all: \
        --requirement /tmp/worker-gtsam.lock \
    && wheel="$(ls /tmp/wheels/slam_it-*.whl)" \
    && python -m pip install --no-cache-dir --constraint /tmp/slam-it.lock "${wheel}[video]" \
    && python -m pip check \
    && rm -rf /tmp/wheels /tmp/slam-it.lock /tmp/worker-gtsam.lock
# Gyroflow's lens profiles (CC0), at the commit slam_it.eval.lenses pins: fetched here, once,
# so a job never reaches out for them.
ENV SLAM_IT_LENS_PROFILES=/opt/lens_profiles PYTHONUNBUFFERED=1
RUN python -m slam_it.eval.lenses fetch --into /opt/lens_profiles
RUN useradd --system --uid 10001 --create-home worker
USER worker
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["sh", "-c", "exec python -m slam_it.orchestration.worker serve --server \"$SLAM_IT_SERVER\""]
