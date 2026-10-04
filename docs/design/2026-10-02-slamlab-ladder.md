# slamlab: a learning ladder for visual SLAM

Status, 2026-10-04: milestone 1 is under way. The scaffold (package-per-rung layout in three
languages, conda environment, CI, two-layer review process) is merged. PR 1, the BA spine,
added rung 0's geometry and estimation in Python and C++: SO(3), SE(3), Umeyama alignment,
camera models, nonlinear least squares and bundle adjustment. PR 2 adds two-view geometry: triangulation, LO-MSAC RANSAC, the 8-point essential and
fundamental matrices, homographies, their decompositions, and two-view initialisation with
GRIC model selection, demonstrated on TUM RGB-D. PR 3 adds the 5-point solvers (Stewénius',
the default, and Nistér's) as E-RANSAC's minimal solver, and the Sampson refinement of E on
its manifold. PR 4 completes slice B with absolute pose: P3P (Ding et al.'s, the default, and
Lambda Twist), EPnP, the pose refinement on SE(3) and PnP RANSAC, demonstrated on three TUM
sequences. Slice C brings the Rust core to parity in two PRs. PR 5 adds rung 0's geometry and
optimisation: SO(3), SE(3), the camera models, Umeyama alignment, PCG32, the least-squares
solver and bundle adjustment, with native C++ and Rust benchmarks on a BAL problem, where the
two take the same time. PR 6 adds the estimation core: RANSAC, triangulation, the two-view and
absolute-pose solvers. After it, Python, C++ and Rust have the same capabilities.

## Purpose

slamlab is a localization-and-mapping project in its own right. It has two goals:

1. **Learning SLAM across the compute spectrum.** Every component is built by us and explained in
   a design note as it is built. The range runs from first-principles geometry to learned
   dense SLAM, and from a Pi-class compute budget to a large GPU.
2. **Real-time SLAM.** At least one rung runs in real time under an edge-class budget.

It is also the geometry source for [reifyscene](https://github.com/evan-wes/reifyscene). That is a
consumer relationship, not the project's identity (see [Boundary with reifyscene](#boundary-with-reifyscene)).

It is a hobby and learning project with no commercial use.

## Non-goals

- Wrapping existing SLAM systems as black boxes. Reference implementations (DROID-SLAM, MegaSaM,
  ORB-SLAM3) are for study and parity checks only.
- Owning depth networks. Monocular depth models (UniDepth, MoGe, VGGT) are *inputs*, reached
  through a small provider interface.
- Keeping things compatible. There are no users, so formats and APIs make clean breaks.

## The ladder

The ladder spans compute budgets. Every rung stands on rung 0.

```
              rung 0: foundations
   (camera models, Lie groups, two-view geometry, BA solver)
                        │
        ┌───────────────┼──────────────────────┐
        ▼               ▼                      ▼
   edge: sparse VO   classic heavy        dense BA + depth prior
        │           (SfM + loop closure)       │
        ▼                                      ▼
   visual-inertial                     learned dense (DROID-style)
                                               │
                                               ▼
                                     heaviest (MegaSaM-style)
```

| Rung | What it is | Compute target |
|---|---|---|
| **0: Foundations** | Camera models, SE(3)/Sim(3), 8/5-point, RANSAC, triangulation, PnP, Gauss-Newton/LM bundle adjustment with a Schur complement; state estimation, batch (Gauss-Newton/LM) and recursive (error-state filter) | CPU |
| **Edge: sparse VO** | Features, tracking, keyframes, local BA (ORB-SLAM-lite); optional RGB-D mode with predicted depth | Real-time under an edge budget |
| **Visual-inertial** | IMU preintegration shared by two estimators, a filter (MSCKF) and a smoother (sliding-window optimisation), on top of the edge VO | Edge budget |
| **Classic heavy** | Incremental SfM, global BA, place recognition, loop closure | Offline, minutes per clip |
| **Dense BA + depth prior** | Dense bundle adjustment over per-pixel depth, with a monocular depth prior and motion masks | GPU |
| **Learned dense** | DROID-style: correlation volumes, recurrent flow update, dense-BA layer | GPU, around real time |
| **Heaviest** | MegaSaM-style: uncertainty-weighted dense BA, focal-length optimisation, robust to dynamic scenes | Large GPU, offline |

## Languages

| Language | Role |
|---|---|
| **Python** | The readable reference implementation of every algorithm; orchestration, evaluation, and the learned rungs (PyTorch) |
| **C++** | The performance core: rung 0, the edge rung, the edge budgets, and dense-BA kernels if profiling calls for them. Showing C++ is a goal in its own right. |
| **Rust** | A chosen subset: rung 0 and the edge VO. Kept parallel and off the critical path: a rung never waits for its Rust modules, but a Rust module, once it exists, keeps pace with its reference. |

**Parity.** Shared fixtures (fixed inputs, expected outputs) live in `tests/fixtures/`. Every C++
and Rust implementation is checked against the Python reference. All three are benchmarked under
the same compute budgets, which gives a "same algorithm, three languages" comparison of speed,
memory and lines of code.

**Packaging.** There are three importable parts. The two compiled ones are optional. Each
algorithm's `backend=` argument picks an implementation at runtime, and `slamlab.backends`
locates the compiled ones and reports which are built.

| Import | Contents | Build |
|---|---|---|
| `slamlab` | Pure Python: reference implementations, orchestration, evaluation | Pure Python |
| `slamlab._cpp` | The C++ core | nanobind, built with scikit-build-core (CMake), inside the `slamlab` package |
| `slamlab_rs` | The Rust subset | PyO3, built with maturin, as its own top-level module |

The Rust module is top-level rather than `slamlab._rs`. maturin and scikit-build-core cannot
both build into one Python package, and installing a second distribution into `slamlab/` breaks
under editable installs.

C++ and Rust sources also build standalone (CMake, cargo) without Python, for the edge budgets
and a possible later phone target.

**The binding boundary is numpy arrays.**
- Bindings take and return float64 C-contiguous numpy arrays: nanobind's `ndarray` and the
  Rust `numpy` crate.
- The cores stay numpy-free, taking fixed-size arrays, spans or slices.
- Batched data therefore crosses the boundary without per-element Python objects, so the
  "same algorithm, three languages" comparison measures the algorithms, not marshalling.

**The error contract is one shape in every language.**
- Fallible core functions return a C++23 `std::expected<T, slamlab::foundations::Error>` or
  a Rust `Result<T, slamlab_foundations::error::Error>`. Nothing throws on the cores' hot
  paths.
- Both share one list of error kinds:
  - `InvalidInput`: shape, range or finiteness;
  - `Degenerate`: pure rotation, collinear points, too few correspondences;
  - `NotConverged`: strict mode was requested and the convergence criteria were not met, or
    the iteration diverged with no usable estimate. Reaching the iteration cap is not an
    error (see the solver contract below).
- The bindings raise the Python exception of the same kind, from `slamlab.foundations.errors`
  (`InvalidInputError`, `DegenerateError`, `NotConvergedError`, all `ValueError`
  subclasses). The Python reference raises the same ones, so every language fails the same
  way. The binding translation helper arrives with the first fallible function.
- **Iterative solvers return their best estimate with a summary,** not an error.
  - This covers Gauss-Newton, Levenberg-Marquardt, BA, and RANSAC with an adaptive iteration
    count. Each returns the estimate together with a solve summary: at least the iteration
    count, a converged flag and the termination reason, plus what that solver has to report
    (costs and a step trace for least squares; samples, inliers and the confidence reached
    for RANSAC).
  - A capped but improved solve is a normal, usable result: real-time BA is capped by
    design. Ceres Solver's `Solver::Summary` and SciPy's `least_squares` status follow the
    same pattern.
  - `NotConverged` is reserved for a caller-requested strict mode, and for runs that
    diverge or fail with no usable estimate. `InvalidInput` and `Degenerate` remain hard
    errors.
  - **Where the summary lives:** each solver owns its summary and its termination enum
    (`nls.Summary` and `nls.Termination`, `ransac.Summary` and `ransac.Termination`), since
    what is worth reporting differs. What they share lives in `foundations.solve`, once per
    language (settled in PR 2, panel A3 on PR 1):
    - Python: the `Summary` Protocol (`iterations`, `termination`, `converged`) and the
      generic `Solution[State, Summary]` dataclass;
    - C++: `cpp/include/slamlab/foundations/solve.hpp`, a `Summary` concept and
      `Solution<State, Summary>`, returned as `std::expected<Solution<…>, Error>`;
    - Rust: `slamlab_foundations::solve`, a `Summary` trait and `Solution<State, S>`.

    BA wraps the least-squares summary in its own. A solver with no estimate at all to
    return (RANSAC that never finds a valid hypothesis) raises `Degenerate`.

    **A summary that nests others** (BA's nests its least-squares solve;
    `ransac.RefinedSummary`, E-RANSAC's and PnP RANSAC's, nests the RANSAC run and the
    refinement's solve) names its primary run. Its `iterations`, `termination` and `converged` are that run's: a nested
    solve's convergence is read from its own summary, never from the outer one. In Python
    such a summary satisfies the `Summary` Protocol through properties delegating to the
    primary run, so it fits the generic `Solution`. The C++ concept needs data members, so
    C++ returns a named struct instead (`ba::Solution`, `ransac::RefinedSolution`), with
    the same `state` and `summary` fields (settled in PR 3, panel A11).
  - **Crossing the binding boundary:** the estimate crosses as float64 arrays and the
    summary as plain scalars, with the termination reason as an integer code. The Python
    entry point builds the dataclass, and there are no binding-side classes. This extends
    the numpy-array rule to scalars, and the summary fields go into the parity fixtures.

## Conventions

These are fixed in milestone 1, before any geometry code. They are the ones every binding
and fixture bakes in.

**Frames and transforms.**
- `T_a_b` maps coordinates in frame b to frame a: `p_a = T_a_b p_b`.
- Inside a solver, a camera pose is the world-to-camera `T_cam_world`, as in ORB-SLAM and
  DROID.
- Trajectory-level results returned to callers (BA's problems and results, `SlamResult`)
  are camera-to-world `T_world_cam`.
- Rung 0's geometric estimators return the transform they name, stated in every signature:
  P3P and PnP `T_cam_world` (what OpenCV and PoseLib return), two-view `T_c2_c1`. A tracker
  inverts PnP's pose once, where it writes the trajectory (settled in PR 4, panel A1).

**Lie groups.**
- **Perturbations are right (local):** `X ⊕ δ = X · Exp(δ)`, as in Solà et al. and GTSAM's
  manifold updates. Jacobians copied from Barfoot (left-perturbed) need converting with the
  adjoint.
- **Tangent order is translation first:** `(ρ, θ)` for SE(3), and `(ρ, θ, σ)` for Sim(3) when
  it arrives. This matches manif and Sophus. GTSAM's `Pose3` tangent is rotation first, so
  Jacobians compared against it appear block-swapped.
- **Quaternions are Hamilton, stored `(x, y, z, w)`,** like Eigen's `coeffs()` and SciPy. Note
  that Eigen's constructor `Quaterniond(w, x, y, z)` takes w first, so code and fixtures
  always build from xyzw arrays. `q` and `-q` are the same rotation, and parity checks compare
  them as equal.
- **`log` returns angles in [0, pi].** At exactly pi the axis is ambiguous up to sign.
- **`log` rejects input that is not a rotation** (beyond a tolerance shared by all three
  languages) with `InvalidInput`. `project_to_so3` repairs it explicitly, through an SVD.

**Array encodings at the boundary** (float64, C-contiguous, row-major):

| Quantity | Shape |
|---|---|
| 3-vector, tangent of SO(3) | `(..., 3)` |
| Rotation matrix | `(..., 3, 3)` |
| Quaternion | `(..., 4)`, xyzw |
| SE(3) pose | `(..., 4, 4)` homogeneous; last row checked to be `[0, 0, 0, 1]` |
| SE(3) tangent | `(..., 6)`, `(ρ, θ)` |
| Pinhole intrinsics | `(4,)`, `[fx, fy, cx, cy]` |
| Radial-tangential intrinsics | `(9,)`, `[fx, fy, cx, cy, k1, k2, p1, p2, k3]` |
| Pixels | `(..., 2)` |
| Bearing vectors | `(..., 3)`, unit norm |

**Pixel coordinates follow OpenCV:** the centre of the top-left pixel is `(0, 0)`. COLMAP's
convention (`(0.5, 0.5)`) is converted by the COLMAP reader. VGGT's principal point is checked
against this convention when its cameras are imported.

**C++ storage.** The cores use Eigen. Fixed-size Eigen types (`Eigen::Matrix3d`, `Vector3d`,
…) may appear in core headers. The bindings map numpy's row-major buffers with
`Eigen::Map<…, Eigen::RowMajor>`. Matrices are factorised with plain Cholesky (`Eigen::LLT`;
nalgebra's `Cholesky`; numpy's `cholesky`) in all three languages. A failed factorisation
inside a damped solver means "reject the step and raise the damping", not an error.

**Structured problems cross as structs of arrays.** A BA problem, for example, crosses as:
- float64 values;
- `int64` index arrays (`cam_idx`, `pt_idx`);
- bool masks (fixed variables);
- options as scalars, with integer codes for enums.

**Options cross as one dict of their dataclass's fields** (from PR 2): a solver whose
settings are a frozen dataclass (`ransac.Options`, `twoview.Options`) passes
`dataclasses.asdict(options)` as one `options` argument, and the binding reads each field by
name; a nested dataclass (`BAOptions.solver`, the losses) crosses as a nested dict. The binding refuses a dict whose keys are not exactly the dataclass's fields
(`check_keys` in `cpp/bindings/common.hpp`), so a field added on one side fails loudly. Results
cross as dicts of arrays and scalars, nested where they hold a sub-result (a RANSAC summary,
or a refined RANSAC's summary nesting the refinement's), with enums as integer codes and a
`codes()` function giving their names, each key qualified by its solver where two could clash
(`ransac_termination`, `nls_termination`); a test holds each module's Python tables to its
`codes()`. `backends.call` runs a function whose two sides differ (Python objects against
their crossing form) and decodes the compiled result with the module's `codes()`;
`backends.dispatch` remains for array functions whose two sides take the same arguments. A
choice field inside an options dataclass (`twoview.Options.essential_solver`) crosses by name,
as `asdict` gives it, and the binding maps it through the core's `name()`, failing on an
unknown name; a choice passed as its own argument (`essential_ransac`'s `minimal_solver`)
crosses as a code.

The cores check index bounds and sizes and return `InvalidInput`, even though the Python
entry point validates too. Standalone C++ and Rust callers get the same protection, and both
languages fail the same way.

**Generic in the core, concrete in the bindings.**
- Generic algorithms (the least-squares solver, RANSAC) are templates in C++ and traits in
  Rust, and generic over a Protocol in Python.
- The bindings expose only concrete instances (`bundle_adjust`, `essential_ransac`, …), so
  no compiled hot loop calls back into Python.
- A generic module with no concrete binding of its own (e.g. `slamlab.foundations.nls`) is
  tested natively in each core, and in parity through the concrete problems that use it.

**Languages per module.** Python and C++ are lock-step: C++ implements every function in a
compiled module's `Implementation` Protocol. Rust implements a chosen set of modules, each in
full: every function of a module it has, with that module's tests running on all three
backends, so a Rust module that falls behind its Python reference fails layer 1 like a C++ one.
A module Rust does not have yet holds nothing back. The tests enforce these rules
(`test_backends`, `conftest.require_function`). Once PR 6 completes rung 0 in Rust, they will
require every rung-0 module of Rust as they do of C++. Later rungs (the edge VO) return to a
chosen set.

## Package layout

**One package per rung,** mirrored in every language, so a module's place in the ladder is
visible from its path and enforceable by contract.

| Python package | C++ | Rust | Rung |
|---|---|---|---|
| `slamlab.foundations` | library `slamlab_foundations`, `slamlab::foundations` | crate `slamlab-foundations` | Rung 0 |
| `slamlab.edge` | library `slamlab_edge` | crate `slamlab-edge` | Edge VO |
| `slamlab.vi` | library `slamlab_vi` | -- | Visual-inertial |
| `slamlab.classic` | library `slamlab_classic` | -- | Classic heavy |
| `slamlab.dense` | -- | -- | Dense BA + depth prior |
| `slamlab.learned` | -- | -- | DROID- and MegaSaM-style |
| `slamlab.eval` | -- | -- | Dataset readers, external references (COLMAP), metrics, benchmarks, budget harness |
| `slamlab.orchestration` | -- | -- | End-to-end pipelines |

Packages for rungs that have no code yet are stubs: a docstring stating the rung, its
milestone and its compute budget. C++ libraries and Rust crates appear as each rung gets
code. The extension submodules mirror the tree: `slamlab._cpp.foundations.so3` and
`slamlab_rs.foundations.so3` implement `slamlab.foundations.so3`. Bindings are one file per
module (`cpp/bindings/<rung>/<module>.cpp`, `rust/slamlab-py/src/<rung>/<module>.rs`), with
thin roots that only wire them.

**Shared interface types** live in rung 0, so every rung can use them:
- `SlamInput`, `SlamResult` and the depth-provider Protocol (needed by both the edge rung's
  RGB-D mode and the dense rung, which are independent siblings) go in
  `slamlab.foundations.interfaces`, created with the first rung that needs them;
- the error classes are in `slamlab.foundations.errors`.

**Dispatch.**
- Each public algorithm validates its input once, using the shared checks in
  `slamlab.foundations.arrays`.
- It then computes with the Python reference, or calls the backend chosen by its
  `backend=` argument.
- `slamlab.backends` only locates compiled implementations and never knows what they
  compute.
- Each algorithm module types its compiled backend with a Protocol named `Implementation`. One
  generic test walks every extension submodule and checks it against the `Implementation`
  of the slamlab module at the same path: same functions, same parameter names. The parity
  tests check the values.

**Dependency contracts** (import-linter, `pyproject.toml`):
- **Layers,** top to bottom: `orchestration`, `eval`, `learned | vi`,
  `dense | edge | classic`, `foundations`, `backends`. Siblings joined by `|` are independent
  of each other. The contract is exhaustive: a new top-level package fails until it is
  placed.
- **Visual-inertial** builds on `edge` only; **learned** builds on `dense` only.
- **CPU rungs** (`foundations`, `edge`, `vi`, `backends`) never import PyTorch or a GPU
  stack, including torch-based geometry libraries (lietorch, pypose, kornia, theseus), which
  are banned by name because import-linter cannot see through them. PyTorch only ever goes
  in an optional extra, never in `dependencies`.
- **No algorithm package imports a depth model.** Depth models are inputs through a
  provider interface.
- **slamlab never imports reifyscene.**

**The same layering holds in C++ and Rust.** Each rung is its own C++ library and Rust
crate, so a rung can only link what it declares. `scripts/check_layers.py` reads the
contracts above from `pyproject.toml` and fails layer 1 on:
- a cross-rung `#include` that the contracts forbid;
- a forbidden `slamlab_add_rung(... DEPENDS ...)` link;
- a forbidden crate dependency.

The three languages therefore share one source of truth.

## Milestones

Learning comes first: the left branch in order. Two early stops on the right branch give
reifyscene a measured payoff.

| # | Milestone | Languages | Measured on |
|---|---|---|---|
| 1 | Rung 0: foundations | Py, C++, Rust | Synthetic scenes with exact answers |
| 2 | Sparse VO under a laptop-CPU budget | Py, C++, Rust | TUM RGB-D, EuRoC |
| 3 | **reifyscene payoff 1:** depth-aware, motion-masked BA refining VGGT's cameras | Py (+C++ solver) | reifyscene lab-coat masked PSNR; spin GT poses |
| 4 | Classic heavy: incremental SfM, place recognition, loop closure | Py, C++ | KITTI odometry |
| 5 | **reifyscene payoff 2:** DROID-style rung with DROID's published weights loaded, plus dense BA with a depth prior (the full Shape-of-Motion recipe) | Py (PyTorch) | Parity with the DROID reference; reifyscene lab-coat and spin |
| 6 | Visual-inertial, branching off milestone 2: filtering versus smoothing on the same IMU front end | Py, C++ | EuRoC, TUM-VI |
| 7 | Edge budgets: Pi-class and Jetson-class runs, ARM build check | C++ (Rust) | Same benchmarks, budget-capped |
| 8 | Train the DROID-style network ourselves | Py (PyTorch) | TartanAir subset; compare against DROID's weights |
| 9 | Heaviest rung: MegaSaM-style | Py (PyTorch) | Sintel, Bonn RGB-D Dynamic |

**Why milestone 3 sits where it does.** reifyscene's old `pose_refine` was a depth-blind BA with no
motion masks. Swapping its new poses in while keeping the old depth produced the "shadow" behind
moving people. Milestone 3 is the corrected version of that same idea: rung 0's solver, with a
depth prior and motion masks. It reuses what milestones 1 and 2 teach and needs only a small
adapter. Its target is the gap between plain VGGT and the old bundle-adjusted cameras on lab-coat
(27.42 vs 29.04 dB masked PSNR).

**Milestone 6: filtering versus smoothing.** SLAM is a state estimation problem, and its
estimators come in two families: batch (smoothing) and recursive (filtering). Rung 0 has the
first in NLS and BA. Milestone 6 builds the second and compares the two on the same input.

- **One front end, two estimators.** On-manifold IMU preintegration (Forster et al. 2017), the
  IMU noise and bias model, and visual-inertial initialisation are built once and feed both.
- **The filter.**
  - An error-state EKF on SO(3)/R³, with rung 0's right-perturbation convention.
  - Grown into the MSCKF (Mourikis & Roumeliotis 2007): a sliding window of cloned poses,
    with feature updates projected onto the left null space, so landmarks never enter the
    state.
  - First-estimate Jacobians, for observability-consistent updates.
  - OpenVINS is the study reference, not a dependency.
- **The smoother.** Rung 0's NLS over a keyframe window, with old states marginalised into a
  prior.
- **The comparison.**
  - Both estimators run on EuRoC and TUM-VI under the same budget.
  - Reported: ATE/RPE; consistency (NEES: whether the reported covariance matches the actual
    error); per-frame cost.
  - Expected, to be confirmed: the filter is cheaper per frame, and the smoother is more
    accurate, because it relinearises.
- **Placement.** It follows the BA-on-NLS split:
  - The generic parts go in `foundations`, beside NLS, as rung 0's recursive estimator:
    - a Gaussian state on a manifold (mean on the group, covariance in the tangent space);
    - an error-state filter, generic over its process and measurement models;
    - covariance propagation and marginalisation, shared by both estimators.
  - The IMU process model and the MSCKF go in `vi`.
  - The generic parts are built when milestone 6 needs them, not before.

## Boundary with reifyscene

- **The dependency runs one way.** slamlab never imports reifyscene. reifyscene reaches slamlab
  through one thin adapter, a new `GeometryInitializer` backend
  (`reifyscene/geometry/slam_wrapper.py`) beside `vggt_wrapper.py`. The existing contract (frames
  in, `GeometryArtifact` out) does not change.
- **slamlab's interface** uses plain arrays and its own result type:

  ```python
  # sketch -- settled during milestone 1/2
  @dataclass
  class SlamInput:
      frames: Sequence[np.ndarray]          # (H, W, 3) uint8
      timestamps: np.ndarray                # (N,) seconds
      intrinsics: np.ndarray | None         # (3, 3) or (N, 3, 3); estimated if None
      depth_priors: Sequence[np.ndarray] | None   # (H, W) metric or relative
      motion_masks: Sequence[np.ndarray] | None   # (H, W) bool, True = moving
      imu: ImuStream | None                 # visual-inertial rungs only

  @dataclass
  class SlamResult:
      T_world_cam: np.ndarray               # (N, 4, 4)
      intrinsics: np.ndarray                # (N, 3, 3)
      depth: np.ndarray | None              # (N, H, W) dense rungs
      confidence: np.ndarray | None         # (N, H, W)
      keyframes: np.ndarray                 # (K,) frame indices
  ```

- **Packaging.** During development, reifyscene installs slamlab editable from the sibling folder.
  reifyscene's `pyproject.toml` has a `slam` extra pinned to a slamlab git tag. The two change in
  lockstep: when slamlab's API changes, the adapter changes with it and the pin moves.
- **Yardsticks are split.** slamlab benchmarks on standard SLAM datasets. reifyscene-specific
  yardsticks stay in reifyscene: spin GT poses, `probe_depth_consistency.py` (dataset geometry
  95/84/74%, single-pass VGGT 50/30/17%), and lab-coat and spin masked PSNR.
- The same pattern will apply later to the query-graph transformer: its own repo, with a one-way
  dependency.

## Compute budgets (edge simulation)

There is no edge hardware, so edge results come from **budget-matched simulation** on this
machine, and are labelled that way.

- **What a budget enforces:**
  - CPU cores: pinned with `taskset`, with share capped by `CPUQuota`;
  - memory: `MemoryMax` through `systemd-run --scope`;
  - GPU: none, or a small slice;
  - a per-frame time deadline.
- **What it cannot reproduce:**
  - ARM microarchitecture performance;
  - thermal throttling;
  - Jetson power modes.

  QEMU aarch64 is used only to prove the code builds and runs on ARM. Its timings are never
  reported.
- **Proposed budgets** (to be tuned to published device specs):

  | Budget | Stands in for |
  |---|---|
  | `laptop-cpu`: 4 fast cores, 8 GB, no GPU | Laptop |
  | `pi-class`: 4 cores at a throttled quota, 4 GB, no GPU | Raspberry Pi 5 |
  | `jetson-class`: 6 cores, 8 GB, a small GPU slice | Jetson Orin Nano/NX |
  | `phone-class` | Later, if the phone target is taken on |
  | `desktop-gpu`: unconstrained | The heavy rungs |

- The harness is the same everywhere. If real hardware ever turns up, it runs there unchanged, and
  the simulated numbers can be checked.
- **Where a rung declares its budget:**
  - its package docstring states the target budgets and what it may depend on;
  - the import-linter contracts enforce the part that is checkable, namely no PyTorch or GPU
    stack on CPU rungs;
  - tests that need a GPU or a budget-capped run are marked `@pytest.mark.gpu` or
    `@pytest.mark.budget`.
  
  `scripts/check.sh` runs the CPU subset on every PR. GPU and budget runs get their own
  opt-in scripts when those rungs arrive, along with a separate light environment for the
  edge rungs, so that having no PyTorch is also proven by installation.

## Datasets

The data lives outside the repo, in `../datasets/` (that is, `../datasets`).
Disk space is checked before each download.

| Dataset | Tests | Arrives with |
|---|---|---|
| TUM RGB-D | Handheld indoor, RGB-D, GT poses, a dynamic subset | Milestone 1 (reader, two-view demo); milestone 2 |
| EuRoC MAV | Drone, stereo + IMU, mocap GT | Milestone 2 (vision), 6 (IMU) |
| TUM monoVO | Photometrically calibrated monocular | Milestone 2, optional |
| KITTI odometry | Driving, long trajectories, loops | Milestone 4 |
| TUM-VI | Visual-inertial, handheld | Milestone 6 |
| ETH3D SLAM | Online leaderboard | Optional, for comparing against published methods |
| TartanAir (subset) | Synthetic, hard motion; DROID's training data | Milestone 8 |
| Sintel, Bonn RGB-D Dynamic | Dynamic scenes | Milestone 9 |
| Replica / ScanNet | Dense map quality | Optional |
| 7-Scenes | Relocalization | If relocalization is added |

## Metrics

- **Trajectory:** ATE (after Sim(3) or SE(3) alignment, as the rung warrants) and RPE.
- **Map:** depth error against GT depth where it exists; reconstruction accuracy and completeness
  on Replica-style data.
- **Cost, per budget:** frames per second, per-frame latency (p50/p95), peak memory.
- **Headline chart:** accuracy against compute, one point per rung, language and budget.

## Design notes

Each component gets a note in `docs/notes/<component>.md`, written as it is built. A note covers
the problem, the maths, the design choices and their alternatives, how it was checked, and what
was learned. The notes double as showcase content when the repo goes public.

## Review process

Code is reviewed in two layers. The full rules are in `CLAUDE.md`.

- **Layer 1, every PR:** `scripts/check.sh` runs deterministic checks in all three languages,
  locally and in CI:
  - documentation present and well-formed: numpy-style in Python, Doxygen in C++, rustdoc in
    Rust;
  - type hints (`mypy --strict`);
  - lint and formatting;
  - dependency rules (import-linter);
  - duplication (jscpd);
  - every test suite.
- **Layer 2, the review panel (`/slamlab-review-panel`),** on PATCH-or-larger changes: three
  reviewer agents in `.claude/agents/`:
  - **hygiene:** cleanliness, readability, documentation quality;
  - **architecture:** modularity, the language split, the ladder's progression by compute
    budget;
  - **algorithm:** correctness against standard formulations, numerical robustness, and the
    state of the art, with citations.

  All three call out code that is not production quality. A smaller change can run one
  reviewer alone.
- **Results are documented and decided together.** Each run's report is saved in
  `docs/reviews/` and committed with the PR. The owner decides the action on every finding;
  the report records each resolution.
- **PR descriptions record the agentic process** (`.github/pull_request_template.md`): which
  agents did what, the panel's verdicts, and the owner's decisions.

## Public presentation

slamlab is presented the way reifyscene is: a static showcase page and an interactive run
viewer, in the same visual style (IBM Plex, panel layout, a frame scrubber), published with
GitHub Pages. Nothing is published before the repo goes public (see
[Licensing and publication](#licensing-and-publication)).

- **The showcase page** walks through the SLAM process on one sequence, stage by stage, with
  stills, short clips and the measured results, as reifyscene's page takes one frame apart.
- **The run viewer** replays a recorded run:
  - **image panel:** the current frame with its tracked features, matches, and RANSAC
    inliers and outliers;
  - **3D panel:** camera frustums, the estimated trajectory against ground truth, and the map
    points accumulating as the run is scrubbed;
  - **solver panel:** per-frame diagnostics from the solve summaries (RANSAC samples and
    inlier ratio, BA cost before and after, termination reasons);
  - **backend toggle:** the same run in Python, C++ and Rust, with matching trajectories and
    their timings. This is where the "same algorithm, three languages" comparison becomes
    visible;
  - later, **rungs side by side:** one sequence run by different rungs of the ladder.
- **It grows with the ladder.** Milestone 1 has components but no pipeline: its demos record
  single steps (two-view initialisation, PnP, BA) with rerun, and the Rust note compares the
  three languages in a table. The viewer starts at milestone 2, the first end-to-end run;
  loop closure (milestone 4) and the dense and learned rungs (milestones 5 and 9) add to it.
- **The run format is milestone 2's,** defined with its pipeline: trajectory, keyframes, map
  and per-frame diagnostics, written by the pipeline and read by both rerun and the viewer.
  It lives in `slamlab.eval`, so the algorithm rungs never depend on the viewer.
- **Run outputs stay outside the repo** (`../runs/`). The page ships only the assets built
  from them.

## Licensing and publication

- The repo is **private**. It goes public together with reifyscene, as one decision, and only on
  the owner's explicit instruction.
- Ported code (DROID-SLAM BSD-3, MegaSaM, and others) keeps its license and attribution in a
  `THIRD_PARTY_NOTICES` file. Model weights are downloaded, never committed. Non-commercial
  licenses (for example VGGT's weights) are acceptable for this hobby project.
- **Rename before going public.** "slamlab" is a working name. "SLAMLab" is the University of
  Zaragoza group behind ORB-SLAM3, and `slamlab` on PyPI is taken.

## Environment

slamlab uses a conda environment, matching reifyscene. `scripts/setup_slamlab.sh` builds it in
two layers:

- **conda layer (conda-forge):** Python 3.11 (matching reifyscene's env), cmake, ninja, a C++
  compiler and Rust. Nothing is installed system-wide.
- **pip layer:** installed from `scripts/locks/slamlab.lock`, which
  `scripts/gen_env_lock.sh` regenerates.

Each env builds the C++ extension in its own `build/<env name>` folder. Envs with the same Python
version otherwise share one CMake tree and silently reuse each other's cached compiler. This
matters at milestone 3, when reifyscene's env also installs slamlab.

## Open items

- **Final name**, before going public.
- **Exact budget numbers**, from published device specs.
- **Milestone 1 tasks carried from the review panel.** Closed by PR 1:
  - The bindings translate every error kind to its Python exception (`cpp/bindings/common.cpp`),
    and each kind is raised from the C++ backend in its algorithm's tests: `InvalidInput`
    throughout, `Degenerate` in alignment, `NotConverged` in camera, NLS and BA.
  - The `exp`/`log` fixture (mpmath-oracled) covers angles near 0, values either side of
    each Taylor switch, near pi, exactly pi (canonical sign), above pi (wrap) and near 2 pi.
    `log` rejects a non-orthonormal input (`InvalidInput`); `project_to_so3` is the explicit
    SVD projection.
  - Closed by PR 2: the `arccos` domain at both ends. The fixture adds rotations whose
    trace rounds just past 3 and just past -1 (symmetric, and with a small or near-pi angle);
    `log` clamps the cosine and takes the angle from `atan2`, and returns the exact angle.
  - Each termination reason maps explicitly to `converged`, as in Ceres (`CONVERGED`).

  Still open:
  - A time-cap termination reason for the compute budgets.
- **Deferred from PR 2's review panel**
  ([report](../reviews/2026-10-02-feature-two-view.md)):
  - Closed by PR 4: **BA's options as a dict (A7).** `BAOptions` crosses as one nested dict
    (choices by name), read strictly at every depth, like every other options dataclass.
  - Closed by PR 4: **a generic dispatch (A8).** `backends.call` takes the reference as a
    thunk, the compiled arguments by keyword and a decoder given the module's `codes()`; the
    five hand-rolled copies use it.
  - Closed by PR 5: **PCG32's parity values as a fixture (A9).** The reference library's
    output and the Floyd samples are `tests/fixtures/foundations/rng.json`, which the Python,
    native C++ (through nlohmann_json) and Rust tests read.
  - **The state of the art for robust estimation (L5).** Barath's SupeRANSAC (arXiv
    2506.04803, 2025) is a component-by-component checklist for this module; MAGSAC++
    (Barath et al., CVPR 2020) scores without a threshold by marginalising over σ; PROSAC
    (Chum and Matas, CVPR 2005) samples by match quality, cheap here since the matcher
    already has ratio-test scores.
- **Found while building PR 2:**
  - Closed by PR 3: **a Sampson-error refinement for E.** `epipolar.refine_essential`
    minimises the inliers' robust Sampson distance over the essential manifold with
    `foundations.nls`, and E-RANSAC keeps it when it lowers the MSAC cost: the median error
    went from 0.93 to 0.71 of the 8-point floor (see the five-point note).
  - **Initialisation on low-parallax scenes.** On TUM fr1/xyz, 13 of 38 pairs report
    `low_parallax` and 3 stay `ambiguous` (see the two-view note). Milestone 2's tracker
    retries on later frames; revisit the thresholds there, against its success rate.
- **Deferred from PR 3's review panel**
  ([report](../reviews/2026-10-03-feature-five-point.md)):
  - Closed by PR 4: **where refinements live (A6).** Each refinement problem stays with its
    model (`epipolar.SampsonProblem`, `pnp.ReprojectionProblem`); what E-RANSAC and PnP RANSAC
    share, the refined summary and "refine the consensus, keep it if the MSAC cost drops", is
    generic in `ransac` (`RefinedSummary`, `refine_consensus`, `estimate_refined`). The S²
    basis and retraction stay in `epipolar` until a second user appears.
  - **Refining inside LO (L5).** LO's refits stay algebraic (the 8-point for E, EPnP for
    pose); PoseLib refines on the geometric error inside LO, and SupeRANSAC (Barath, arXiv
    2506.04803, 2025) finishes E with LM on MAGSAC++-weighted Sampson error. PR 4 added the
    inlier-reselect and refine loop after LO (`refine_consensus`'s rounds); refining inside LO
    itself remains open.
  - Closed by PR 4: **`codes()` keys (A7).** RANSAC's termination is `ransac_termination`
    and NLS' `nls_termination` in every module; each decoder reads its own key.
- **Found while building PR 4:**
  - Closed by PR 4: **two-view crashed on a rank-1 homography.** Duplicate keypoints at one
    pixel let a rank-1 H win H-RANSAC (TUM fr1/desk); `twoview.initialize` raised from its
    decomposition. Such an H is now no model, and E decides.
  - **H-RANSAC can return a non-invertible H.** The two-view fix treats it as no model; the
    deeper fix is for the homography estimator to reject a fitted H of rank below 3, which
    changes H-RANSAC's results. Decide with the edge rung's tracker (milestone 2).
  - **A least-squares PnP for planar scenes (panel L6).** EPnP refuses planar points, so LO's
    refits do not run there; the refine-and-reselect rounds reach the noise floor anyway (the
    absolute-pose note). IPPE (Collins and Bartoli, IJCV 2014) is the planar solver; SQPnP
    (Terzakis and Lourakis, ECCV 2020; OpenCV's `SOLVEPNP_SQPNP`) is globally optimal for
    planar and non-planar points in one solver, the better LO refit if the rung wants one.
- **Deferred from PR 4's review panel**
  ([report](../reviews/2026-10-03-feature-absolute-pose.md)):
  - **One form for the refinement callback (A6).** `refine_consensus`'s callback takes an
    inlier mask in Python and inlier indices in C++; pick one when Rust gains RANSAC (PR 6).
  - **Depth maps in the library (A7).** `eval.tum` associates depth maps, but decoding the
    16-bit PNGs happens in the demo, with OpenCV, which `slamlab` may not import. Milestone 2's
    RGB-D mode needs an in-library loader: choose its PNG dependency then.
  - **Why the refinement rounds stopped (L9, run 2).** On planar scenes the cap of four
    rounds, not stable inliers, ends most runs, and `RefinedSummary` cannot say which
    (`rounds == REFINE_ROUNDS` is either). Add a stop reason (cost, stable, cap, too few
    inliers) when a caller needs it, with the summary-convention's termination enum.
  - **One feature front end (A8).** Each demo script has its own SIFT and ratio-test
    matching. Fold them into the edge rung's front end when it lands (milestone 2), or into
    one shared script helper if a third demo comes first.
- **Deferred from PR 1's review panel**
  ([report](../reviews/2026-10-02-feature-ba-spine.md)):
  - **Batched APIs for standalone callers (A11).** Batching over many inputs lives in the
    bindings (`cpp/bindings/common.hpp`), so C++ and Rust callers outside Python have no
    batched API. Decide before the edge rung (milestone 7), where C++ runs without Python.
  - **A sparse reduced solver (A12).** BA's reduced camera system is solved densely behind
    `ReducedSystemSolver`. Milestone 4's scenes need a sparse Cholesky or a preconditioned
    CG behind the same seam (see the BA note's state-of-the-art section). It belongs in
    rung 0, beside the dense one, not in `classic`. The C++ core is not yet parametrised on
    a reduced solver (it uses its dense one directly), so the seam has to be opened there
    first.
  - **A per-block step tolerance (L6).** NLS's absolute step tolerance never fires on BA's
    mixed scales (radians, pixels of focal length, metres). Replace it with a relative,
    per-block test.
  - **Facade edge cases in the layer check (A24, run 3).** `check_layers.py` merges a
    package's public header and its implementation into one node, so the header's own
    includes are checked as the implementation's; a facade with no `_CPP_FACADES` entry is
    exempt. Key facades by file kind, and fail on an unmapped one, when the next package gets
    a facade.
  - **The gauge in rung 0's own right (A26, run 3).** `ba/gauge` depends only on poses and
    points. When pose-graph optimisation (the classic rung's loop closure) needs the same
    null-space removal, lift it to `foundations.gauge` rather than importing it from `ba`.
  - Closed by PR 5: **one mechanism for enum names (A27, run 3).** Every enum that crosses a
    binding has its names in its core: `name()` in C++ (camera models, NLS methods and losses,
    BA gauges joined the terminations and solvers) and `ALL` with `name()` in Rust. Each
    binding's `codes()` is built from them, and choices are read by name through them.
  - **σ_f with the gauge left free (L30, run 5).** When a fixed recipe leaves gauge
    directions in the problem (`free_gauge > 0`), σ_f is infinite under `none` but finite
    under `first_pose`, where it depends on how the damping holds the free scale (0.13% off
    the minimal gauge's). Report σ as unavailable whenever `free_gauge > 0`, or document it.
  - **Triggs' robust-loss correction (L7).** NLS uses plain IRLS, which drops `ρ''`. The
    Triggs correction restores it, and converges faster on outliers. For milestone 3, whose
    data has outliers.
- **Milestone 3's inputs, to settle before its adapter is written:**
  - Where its correspondences come from: VGGT's tracks, or reifyscene's tracks
    (VGGSfM/CoTracker) via the dumped BA problem.
  - Which depth prior: VGGT's depth or a monocular model, metric or relative. This sets
    whether BA's per-frame scale/shift is free.
- **GPU and budget test runs:** their opt-in scripts and the light edge environment arrive
  with the first PyTorch rung (milestone 5) and the budget harness (milestone 7).
- **reifyscene's env** needs slamlab's build tools (cmake, compiler, Rust) when the adapter lands
  at milestone 3.
