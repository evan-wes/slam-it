# slamlab: a learning ladder for visual SLAM

Status, 2026-10-06: milestone 1 is complete, and milestone 2 is under way (see
[Milestones](#milestones)). The scaffold (package-per-rung layout in three
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
absolute-pose solvers. Python, C++ and Rust now have the same capabilities, and every module's
tests run on all three. Milestone 2's PR 7 adds the front end (fixed-point images, ORB as
ORB-SLAM extracts it, matching and rectified stereo) and the evaluation (dataset readers,
metrics, the run format). PR 8 adds the edge VO pipeline, ORB-SLAM-lite in monocular, stereo
and RGB-D, with native C++ and Rust programs that run a whole sequence and make the same
decisions as the Python reference ([the edge VO note](../notes/edge-vo.md)).

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
   (camera models, Lie groups, two-view geometry, BA solver,
    the front end: images, ORB, matching, stereo)
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
| **0: Foundations** | Camera models, SE(3)/Sim(3), 8/5-point, RANSAC, triangulation, PnP, Gauss-Newton/LM bundle adjustment with a Schur complement; state estimation, batch (Gauss-Newton/LM) and recursive (error-state filter); the front end: fixed-point images, ORB features, Hamming matching, rectified stereo | CPU |
| **Edge: sparse VO** | Tracking on rung 0's ORB front end, keyframes, local BA (ORB-SLAM-lite); monocular, stereo and RGB-D modes (RGB-D from a sensor now, predicted depth later) | Real-time under an edge budget |
| **Visual-inertial** | IMU preintegration shared by two estimators, a filter (MSCKF) and a smoother (sliding-window optimisation), on top of the edge VO | Edge budget |
| **Classic heavy** | Incremental SfM, global BA, place recognition, loop closure | Offline, minutes per clip |
| **Dense BA + depth prior** | Dense bundle adjustment over per-pixel depth, with a monocular depth prior and motion masks | GPU |
| **Learned dense** | DROID-style: correlation volumes, recurrent flow update, dense-BA layer | GPU, around real time |
| **Heaviest** | MegaSaM-style: uncertainty-weighted dense BA, focal-length optimisation, robust to dynamic scenes | Large GPU, offline |

## Languages

| Language | Role |
|---|---|
| **Python** | The readable reference implementation of every algorithm; orchestration, evaluation, and the learned rungs (PyTorch) |
| **C++** | The performance core: rung 0, the edge rung, the edge budgets, and dense-BA kernels if profiling calls for them; outside the cores, `eval`'s native readers and run writer (`slamlab_eval`) and the native VO program (`slamlab_vo`, in orchestration); and the calibration side track (`slamlab_calib`), beside Python, without Rust. Showing C++ is a goal in its own right. |
| **Rust** | Rung 0 and the edge rung, each in lock-step with Python and C++: every slice of those rungs lands in all three languages together; outside the cores, `eval`'s native readers and run writer (`slamlab-eval`) and the native VO program (`slamlab_vo`, crate `slamlab-orchestration`). The other rungs have no Rust. |

**Runtime dependencies.** Adding one, to the Python package or to a core, is a change to this
document first.
- **Python:** numpy. Pillow too, but in `slamlab.eval.images` alone, which reads image files
  (milestone 2 reads PNG frames and depth maps): the algorithm rungs take arrays, never files,
  and an import-linter contract keeps Pillow in that one module. `slamlab.eval.rosbag` reads
  ROS 1 bags by hand, with no dependency (the calibration side track).
- **C++:** Eigen. The `slamlab_eval` library reads PNGs with stb_image (vendored in
  `third_party/stb`, PNG support only; MIT or public domain), outside the algorithm cores.
- **Rust:** nalgebra, the matrix type, and faer for every SVD and the general
  eigendecomposition, single-threaded, behind the crate-private `linalg` (nalgebra's SVDs are
  not backward stable; see the Rust port's note). The `slamlab-eval` crate reads PNGs with
  the pure-Rust `png` crate, outside the algorithm cores.
- The three readers read a file the same way (gray from a gray file as stored, from a colour
  one through `image.to_gray`), held to shared PNG fixtures.
- The native run writers and VO programs add none (decided for PR 8): they write JSON and
  `.npy` by hand, and the programs take their options as command-line flags, so nothing
  native reads JSON.

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

**The native VO programs** (`slamlab_vo`, in C++ and in Rust; from PR 8) run a whole sequence
without Python: they read the dataset natively and write the run folder.
`slamlab.orchestration.vo.run(..., backend=)` runs the Python pipeline in-process, or launches
the chosen native program and reads its run back. scikit-build installs the C++ program into
the package (`slamlab/bin/`); `setup_slamlab.sh` and `check.sh` build the Rust one with cargo
(`rust/target/release/`); `slamlab.backends` locates both, never through `PATH`.

**The binding boundary is numpy arrays.**
- Bindings take and return float64 C-contiguous numpy arrays (uint8 for images): nanobind's
  `ndarray` and the Rust `numpy` crate.
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
- `eval`'s native readers (`slamlab_eval`, `slamlab-eval`) use the same kinds. Every failure
  there, a missing or unreadable file included, is `InvalidInput` naming the file, since the
  kinds have none for I/O; Python's `slamlab.eval` raises `FileNotFoundError` for a missing
  file instead.
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
    the same `state` and `summary` fields (settled in PR 3, panel A11). Rust's trait has
    methods, so there a nesting summary implements it by delegation, as in Python:
    `ba::Summary` and `ransac::RefinedSummary` implement `solve::Summary`, and a refined run
    returns the generic `solve::Solution` (settled in PR 6, panel A5 on PR 5). BA's own
    `Solution` stays a named struct in every language, since its estimate is several arrays.
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

**Images are uint8, and every image operation is exact** (from PR 7). A grayscale image is a
C-contiguous `(H, W)` uint8 array (RGB `(H, W, 3)`), `image[y, x]`, with the pixel convention
above; the cores hold it as an `Image` of rows, columns and row-major pixels. Colour
conversion, blur, resizing, pyramids and remapping are fixed-point integer arithmetic
(`slamlab.foundations.image` states each rounding), so the three languages' images are
bit-identical, and so are the FAST corners and descriptors computed from them: a fixture
compares them exactly, with no tolerance. Floating point could not promise that across
platforms, since a fused multiply-add rounds once where separate operations round twice.

**Pyramid coordinates are exact** (from PR 7). The resizes' aligned-centre maps compose, so a
position `x` on a level of `n_l` pixels lies at `((2x + 1) n_0 - n_l) / (2 n_l)` in a full
image of `n_0`, along each axis (`image.full_image_coordinate`, in all three languages), and a
full-image position `u` at `((2u + 1) n_l - n_0) / (2 n_0)` on the level. A keypoint's `xy` is
that map of its level pixel, and the cores refuse features whose `xy` is not. Never scale by
`s^l`, as ORB-SLAM and OpenCV do: it drops the half-pixel term and uses `s^l` for
`n_0 / n_l`, up to about 2 pixels off on a coarse level.

**Floats compared exactly are scalar, unfused operations** (from PR 7). Where the parity
fixtures compare floats with no tolerance (keypoint coordinates, stereo's columns, disparities
and depths), each value is a fixed sequence of scalar IEEE operations, written in the same
order in every language. Every C++ core library compiles with `-ffp-contract=off`, PUBLIC so
that code inlining a core header is compiled the same way; Rust never contracts on its own;
numpy evaluates one operation at a time. Eigen's and nalgebra's vectorised kernels may fuse
on ARM whatever the flag says (Eigen enables FMA packets wherever the target has them), so an
exactly compared path never goes through them; results that do, such as rectification, are
compared to a tolerance.

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
crosses as a code. The edge rung's `mode`, an argument of most of its stage functions, crosses
by name too, each binding reading it with its `mode_in` and failing on an unknown name: a name
reads in a test's failure message and is what a run's `run.json` records (PR 8's panel, A4);
rung 0's argument choices stay codes.

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

**Languages per module.** Rung 0 is lock-step in all three languages: C++ and Rust implement
every function of every compiled module's `Implementation` Protocol, and every module's tests
run on all three backends (`backends.LOCKSTEP_BACKENDS`), so a compiled module that falls
behind its Python reference fails layer 1. The tests enforce it (`test_backends`,
`conftest.require_function`). Rust was a chosen set of modules until PR 6 completed rung 0 in
it. The edge rung is lock-step from its first slice (decided when milestone 2 was planned):
each of milestone 2's PRs lands every module in Python, C++ and Rust together, so
`backends.COMPILED_RUNGS` lists both rungs for both compiled backends and nothing needs a skip.
A rung a compiled backend does not cover is simply absent from its entry.

## Package layout

**One package per rung,** mirrored in every language, so a module's place in the ladder is
visible from its path and enforceable by contract.

| Python package | C++ | Rust | Rung |
|---|---|---|---|
| `slamlab.foundations` | library `slamlab_foundations`, `slamlab::foundations` | crate `slamlab-foundations` | Rung 0 |
| `slamlab.edge` | library `slamlab_edge` | crate `slamlab-edge` | Edge VO |
| `slamlab.vi` | library `slamlab_vi` | -- | Visual-inertial |
| `slamlab.classic` | library `slamlab_classic` | -- | Classic heavy |
| `slamlab.calib` | library `slamlab_calib` | -- | Sensor calibration (the side track) |
| `slamlab.dense` | -- | -- | Dense BA + depth prior |
| `slamlab.learned` | -- | -- | DROID- and MegaSaM-style |
| `slamlab.eval` | library `slamlab_eval`, `slamlab::eval` (native PNG and dataset readers, run writer) | crate `slamlab-eval` (native PNG and dataset readers, run writer) | Dataset readers, external references (COLMAP), metrics, benchmarks, budget harness |
| `slamlab.orchestration` | library `slamlab_orchestration` and program `slamlab_vo` (`cpp/apps/orchestration/`; from PR 8) | crate `slamlab-orchestration` and its program `slamlab_vo` (from PR 8) | End-to-end pipelines |

Packages for rungs that have no code yet are stubs: a docstring stating the rung, its
milestone and its compute budget. C++ libraries and Rust crates appear as each rung gets
code. The extension submodules mirror the tree: `slamlab._cpp.foundations.so3` and
`slamlab_rs.foundations.so3` implement `slamlab.foundations.so3`. Bindings are one file per
module (`cpp/bindings/<rung>/<module>.cpp`, `rust/slamlab-py/src/<rung>/<module>.rs`), with
thin roots that only wire them.

**Shared interface types** live in rung 0, so every rung can use them:
- `SlamInput`, `SlamResult` and the depth-provider Protocol (needed by both the edge rung's
  RGB-D mode and the dense rung, which are independent siblings) go in
  `slamlab.foundations.interfaces`, created with the first rung that needs them. That is
  milestone 3 (decided for PR 8): PR 8's RGB-D mode reads the sensor's depth maps from the
  dataset, and predicted depth, the first provider, arrives with milestone 3;
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
  `dense | edge | classic | calib`, `foundations`, `backends`. Siblings joined by `|` are independent
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
- a forbidden crate dependency;
- inside a rung, a C++ `#include` or a Rust `crate::`/`super::` path that breaks the rung's
  or a package's internal order (such as "Rung 0's internal order"), chains through unlisted
  modules included.

The three languages therefore share one source of truth.

**Native benchmarks** are developer tools outside the cores: `cpp/bench/<rung>/` (Google
Benchmark, behind the CMake option `SLAMLAB_BUILD_BENCHMARKS`) and each Rust crate's
`benches/` (criterion), with a crate's `examples/` for standalone runs. They are held to the
ladder like the code they time, built and linted by layer 1, and never run by it. A
benchmark that reads dataset files links the native readers (`slamlab_eval`,
`slamlab-eval`), so it sits above them, in `cpp/bench/eval/` or the `slamlab-eval` crate's
`benches/`; a rung-0 benchmark builds its input in code (as `orb_extract` does).
`slamlab.eval` keeps the Python benchmarks and, from milestone 7, the budget harness, which
will drive the native benchmarks on the edge targets.

## Milestones

Learning comes first: the left branch in order. Two early stops on the right branch give
reifyscene a measured payoff.

| # | Milestone | Languages | Measured on |
|---|---|---|---|
| 1 | Rung 0: foundations | Py, C++, Rust | Synthetic scenes with exact answers |
| 2 | Sparse VO under a laptop-CPU budget: monocular, stereo and RGB-D, and the run viewer | Py, C++, Rust | TUM RGB-D, EuRoC |
| 3 | **reifyscene payoff 1:** depth-aware, motion-masked BA refining VGGT's cameras | Py (+C++ solver) | reifyscene lab-coat masked PSNR; spin GT poses |
| 4 | Classic heavy: incremental SfM, place recognition, loop closure | Py, C++ | KITTI odometry |
| 5 | **reifyscene payoff 2:** DROID-style rung with DROID's published weights loaded, plus dense BA with a depth prior (the full Shape-of-Motion recipe) | Py (PyTorch) | Parity with the DROID reference; reifyscene lab-coat and spin |
| 6 | Visual-inertial, branching off milestone 2: filtering versus smoothing on the same IMU front end | Py, C++ | EuRoC, TUM-VI |
| 7 | Edge budgets: Pi-class and Jetson-class runs, ARM build check | C++ (Rust) | Same benchmarks, budget-capped |
| 8 | Train the DROID-style network ourselves | Py (PyTorch) | TartanAir subset; compare against DROID's weights |
| 9 | Heaviest rung: MegaSaM-style | Py (PyTorch) | Sintel, Bonn RGB-D Dynamic |

**Milestone 2: sparse VO.** The first end-to-end pipeline: ORB-SLAM-lite in the edge rung, in
monocular, stereo and RGB-D modes, real time under the `laptop-cpu` budget. Rung 0 already has
its geometry (two-view, PnP, triangulation, BA); what is new is the front end, the pipeline's
state, evaluation and the budget harness. Every PR lands in Python, C++ and Rust together.

| PR | Slice |
|---|---|
| 7 | **Front end and evaluation.** Images (Pillow in `slamlab.eval`; native PNG readers outside the cores, `slamlab_eval` and `slamlab-eval`), grayscale pyramids, blur and bilinear remapping. ORB, our own: multi-scale FAST spread over the image, intensity-centroid orientation, rBRIEF descriptors, Hamming matching with a ratio test. Stereo rectification from EuRoC's calibration, row-constrained matching with subpixel refinement, depth from disparity. The EuRoC reader; TUM's depth maps in the library; the demos on the shared front end. ATE (Sim(3) alignment for monocular, SE(3) for stereo and RGB-D) and RPE; the run format. |
| 8 | **The pipeline.** Initialisation: two-view for monocular, retrying on later frames when the parallax is low; from depth for stereo and RGB-D, metric from the start. Tracking: a constant-velocity prediction, map points matched by projection, PnP RANSAC as the fallback, motion-only pose refinement. Keyframe selection; local mapping (triangulation, local BA over a keyframe window, culling of points and keyframes). Native programs (C++ and Rust) that run a whole sequence. Results on TUM (monocular, RGB-D) and EuRoC (monocular, stereo), with rerun recordings. Planned in [the edge VO design](2026-10-05-edge-vo.md); built as [the edge VO note](../notes/edge-vo.md) describes. |
| 9 | **Budgets and real time.** The `laptop-cpu` harness (pinned cores, capped memory, a per-frame deadline); latency (p50, p95) and peak memory; profiling the C++ and Rust pipelines to real time; the three-language comparison. Python is measured but not held to real time. |
| 10 | **The run viewer** (see [Public presentation](#public-presentation)), on the run format, used locally until the repo goes public. |

The open items aimed at milestone 2 close in these PRs: the depth-map loader and the shared
front end in PR 7; the low-parallax retry and the non-invertible homography in PR 8.

**The front end is rung 0's** (decided in PR 7). Fixed-point images, ORB features, Hamming
matching and rectified stereo live in `slamlab.foundations`, not in the edge rung: the
classic rung (milestone 4) matches the same features, so both build on one front end. The
edge rung holds what is its own, tracking, keyframes and local mapping.

**The edge rung is ORB-SLAM-lite** (decided for PR 8; the plan is
[the edge VO design](2026-10-05-edge-vo.md)):
- **What it leaves out of ORB-SLAM:** a sliding window of the last 10 keyframes stands in for
  the covisibility graph; no bag of words, so no relocalisation or loop closure (the classic
  rung's); one thread, mapping after tracking, so a run is deterministic; no map reuse.
- **A lost track ends the run,** recorded in its diagnostics.
- **Its state is a struct of arrays** in all three languages, so each stage function
  (initialise, track, map) crosses the bindings as dicts of arrays, as rung 0's do. Python
  runs the whole pipeline in-process; C++ and Rust run it in the native programs, not across
  the binding.
- **Parity is held per stage,** on fixture states, and per sequence, on synthetic box-room
  sequences with a known trajectory: every backend completes them, with an ATE under a stated
  bound and the three backends' ATEs within a stated tolerance of each other. TUM and EuRoC
  results are reported, not gated. *As built (PR 8's panel, A1, A16):* the states are built
  from the room when the tests run and compared live with the Python reference; on whole
  runs, every backend takes the same decisions, with poses within 1e-9, and is held to a
  golden record of the room's runs (the edge VO design, §3).
- **Its generic parts go in rung 0:** guided matching by projection and a feature grid in
  `foundations.matching`, the full-image-to-level map in `foundations.image`.

**A side track: rig calibration** (Python and C++; planned in
[the rig calibration design](2026-10-05-rig-calibration.md)). Beside milestone 2, and
independent of it: an RGB-D camera, a 2D lidar and a rangefinder calibrated against each other
from the rig's own motion, as a study in observability. Two PRs, C1 (reading ROS 1 bags,
Python) and C2 (`slamlab.calib`, Python and C++), in their own worktree.

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
| TUM RGB-D | Handheld indoor, RGB-D, GT poses, a dynamic subset | Milestone 1 (reader, two-view demo); milestone 2 (monocular and RGB-D) |
| EuRoC MAV | Drone, stereo + IMU, mocap GT | Milestone 2 (monocular and stereo), 6 (IMU) |
| Own recordings (private) | Described only in their configs, outside the repo | The calibration side track; never in the repo |
| TUM monoVO | Photometrically calibrated monocular | Milestone 2, optional |
| KITTI odometry | Driving, long trajectories, loops | Milestone 4 |
| TUM-VI | Visual-inertial, handheld | Milestone 6 |
| ETH3D SLAM | Online leaderboard | Optional, for comparing against published methods |
| TartanAir (subset) | Synthetic, hard motion; DROID's training data | Milestone 8 |
| Sintel, Bonn RGB-D Dynamic | Dynamic scenes | Milestone 9 |
| Replica / ScanNet | Dense map quality | Optional |
| 7-Scenes | Relocalization | If relocalization is added |

## Metrics

- **Trajectory:** ATE (after Sim(3) or SE(3) alignment, as the rung warrants) and RPE, as in
  Sturm et al.'s TUM benchmark (`slamlab.eval.metrics`): Sim(3) for monocular runs, SE(3) for
  stereo and RGB-D; the RPE's estimate scaled by the ATE's scale.
- **The run format** (`slamlab.eval.runs`), written by every backend's pipeline and read by the
  evaluation and the run viewer: a folder of `run.json` (dataset, sequence, mode, backend,
  settings), `trajectory.txt` in TUM's format (so other systems' trajectories score the same
  way), the keyframes and map points as `.npy`, and `diagnostics.jsonl`, one object per frame.
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
  locally and in CI (offline for every PR, `scripts/offline_ci.sh`; on GitHub on demand and
  for releases):
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
  three languages in a table. The viewer is built in milestone 2 (its last PR), on the first
  end-to-end runs, and used locally until the repo goes public; loop closure (milestone 4)
  and the dense and learned rungs (milestones 5 and 9) add to it.
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

**CI runs offline.** `scripts/offline_ci.sh` does what `.github/workflows/test.yml` does, on
the developer's machine: a clean checkout of the committed files (in
`~/.cache/slamlab-offline-ci/`, cleaned with `git clean -ffdx` each run), merged with
`develop` when it is not based on it, an env of its own, `slamlab-ci`, built from the lock by
`setup_slamlab.sh` (recreated whenever the lock or the setup script changes), then
`scripts/check.sh`. It gates every PR, whose template records its result. The GitHub workflow
runs on demand (`workflow_dispatch`) and on pushes to `main`; the GitHub Actions minutes are
shared with the owner's other projects.

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
  - Closed by PR 8: **H-RANSAC can return a non-invertible H.** The DLT refuses a fitted H
    whose smallest singular value, in the Hartley-normalised frame it fits in, is below
    `homography.CONDITION_TOLERANCE` (1e-4) of its largest: real homographies stay above
    3.7e-4 there, the H that fr1/desk's duplicate matches fit is at 1.9e-5. Two-view's
    workaround is gone. Duplicate matches can still fit an invertible H that explains only
    them (34 matches onto 2 pixels there); E wins against it.
  - **A least-squares PnP for planar scenes (panel L6).** EPnP refuses planar points, so LO's
    refits do not run there; the refine-and-reselect rounds reach the noise floor anyway (the
    absolute-pose note). IPPE (Collins and Bartoli, IJCV 2014) is the planar solver; SQPnP
    (Terzakis and Lourakis, ECCV 2020; OpenCV's `SOLVEPNP_SQPNP`) is globally optimal for
    planar and non-planar points in one solver, the better LO refit if the rung wants one.
- **Deferred from PR 4's review panel**
  ([report](../reviews/2026-10-03-feature-absolute-pose.md)):
  - Closed by PR 6: **one form for the refinement callback (A6).** `refine_consensus`'s
    callback takes the consensus's inlier indices, ascending, in all three languages.
  - Closed by PR 7: **depth maps in the library (A7).** `tum.read_depth` reads a depth map in
    metres through `slamlab.eval.images` (Pillow); C++'s `slamlab_eval` and Rust's
    `slamlab-eval` read PNGs natively, outside the cores.
  - **Why the refinement rounds stopped (L9, run 2).** On planar scenes the cap of four
    rounds, not stable inliers, ends most runs, and `RefinedSummary` cannot say which
    (`rounds == REFINE_ROUNDS` is either). Add a stop reason (cost, stable, cap, too few
    inliers, refinement failed) when a caller needs it, with the summary-convention's
    termination enum. Since PR 6 the refiner may fail (panel A2), and a failure in round 2 or
    later reads as "no improvement" (PR 6's panel, run 2, A6).
  - Closed by PR 7: **one feature front end (A8).** Both demos match slamlab's ORB features
    (`foundations.orb` and `foundations.matching`) in place of their own SIFT.
- **Deferred from PR 8's review panel**
  ([report](../reviews/2026-10-06-feature-edge-vo.md)):
  - **Re-anchoring the last frame after local BA (L6).** Local BA moves a new keyframe, but
    the pipeline keeps the frame's pose from before it, so the next velocity carries the
    correction; ORB-SLAM re-anchors the last frame to its reference keyframe every frame
    (`UpdateLastFrame`) and saves each frame relative to its BA-refined reference keyframe.
    A run's frame poses are the tracking-time estimates; the edge note scores the final
    keyframes too, ORB-SLAM's protocol, for the like-for-like column.
  - **The keyframe time's growth on long sequences (H19).** Keyframe time still grows along
    EuRoC MH_01 (see the edge note), unprofiled. The candidates: `refresh`'s per-point
    allocations and its O(n² log n) distinctive descriptor, and the feature grid rebuilt on
    every search (A11). Python's per-point loops stay, as the plain statement of the rules.
  - **The real-time items, with PR 9 (A11).** `edge::search` rebuilds its `FeatureGrid` on
    every call, where the design says per frame; the prediction's SO(3) projection runs
    through Rust's allocating `svd3` every frame (PR 6's A11, below).
  - **Lifting the edge camera into rung 0, with milestone 4 (A8).** `edge.frames.Camera` (a
    model and intrinsics with an image size and a baseline) and the chi-square test by
    pyramid level in `edge.residuals` belong in `foundations` once the classic rung needs
    them, rather than copied; with them the one-pose SE(3) helpers `edge.map.to_camera` and
    `camera_centre`, unchecked for the hot loops where rung 0's `se3.act` checks (run 2,
    A23).
  - **Shorter native stage functions (H6).** `track_last_frame`, `track_local_map` and
    `relocalize_locally` in C++ and Rust, and C++'s `initialize_monocular`, run 70-95 lines
    each (Python's longest is 50); helpers such as `project_last_frame_points`,
    `local_map_projections` and `placed_by_pnp` behind the same stage functions would leave
    stage parity untouched.
  - **The native option checks from the tables (A13; run 2, A22).** C++'s `check_options`
    lists the fields by hand in its range lists, beside `kIntegerOptions` and `kRealOptions`,
    and Rust's beside `named_settings!`; a field added to a table but not a list would go
    unchecked natively, where Python checks every field by its type. Run 2's table-driven
    tests (every setting refuses a bad value by name, in every language) now catch such a
    field; a range class per table entry would make the lists unnecessary.
  - **Loud preconditions in the map's refresh (H14).** Rust's `refresh` helpers fall back
    with `unwrap_or` and C++ dereferences its searches unchecked; both rely on every point
    having an observation, which `refresh_points` checks and the operations keep. A
    `debug_assert!` or an error would make a violation loud.
  - **The map's count invariants (H15).** `Map`'s checks do not hold `pt_visible >= 1`,
    `found <= visible` or finite poses and points, which `cull_points` divides by; the
    operations keep them, so only a hand-built or decoded map can break them.
  - **Solve summaries in the diagnostics (L9).** Local BA and pose refinement drop their
    summaries. Rung 0's LM only accepts steps that lower the cost, so nothing diverges
    silently, but the iterations, the costs and the termination in `diagnostics.jsonl`
    would make long-sequence drift (fr3/long_office) investigable.
  - **Where the two-view fixture's duplicate matches come from (L11).** 38 of its 88
    matches share one pixel; the homography's rank rule handles the symptom.
  - **ORB-SLAM's reset of a map lost early, with milestone 4 (run 2, L13).** ORB-SLAM2
    resets the system when the track is lost with five keyframes or fewer (ORB-SLAM3 the
    active map, under ten), which is what lets it start a depth map from any 500 keypoints and
    a monocular map from low parallax. The lite pipeline ends the run instead, so a depth start
    also needs `min_init_depth_points` (100) keypoints with depth, a stand-in. The reset drops
    the frames before it from the trajectory, as ORB-SLAM's does: it belongs with the classic
    rung's relocalisation and maps.
  - **Relocalising monocular runs that lose the track, with milestone 4 (run 2, L15).**
    Started one to four frames after the frame the first run initialises from, 6 of 20
    monocular runs on TUM and EuRoC lose the track (3 of 5 on fr1/desk), each deep into the
    run (44 to 128 keyframes), when its inliers fall below ORB-SLAM's 30. ORB-SLAM relocalises
    there with the bag of words (`Tracking::Relocalization`); the lite pipeline ends the run.
    The edge note counts a lost run as a failure in its five-start medians.
  - **The Rust extension in layer 1 (run 2, A19).** `scripts/check.sh` rebuilds the Rust
    program before pytest but not the extension `slamlab_rs` (`maturin develop`), so after a
    Rust change the stage-parity tests run the old extension while the whole-run tests run
    the new program. Rebuild it in `check.sh`, or give it the program's stale check
    (`backends.StaleProgramError`). Offline CI builds from clean and is not affected.
  - **The C++ edge tests' layout (H18).** Options' checks live in `test_map.cpp` and the
    residuals' in `test_adjust.cpp`, not in tests mirroring their modules.
- **Deferred from PR 7's review panel**
  ([report](../reviews/2026-10-05-feature-front-end.md)):
  - **Reusing the extraction's pyramid (A1), with PR 9's real time.** `stereo.match` rebuilds
    both pyramids `orb.extract` already built (2.9 ms in C++, 1.2 ms in Rust per 752x480
    pyramid through the binding). Give extraction a result that holds its pyramid, or split
    the cores into `image::pyramid` and an `orb::extract_levels`, as ORB-SLAM's
    `ComputeStereoMatches` reads the extractor's `mvImagePyramid`.
  - Closed by PR 8: **the run format's golden folder and its native writers (A9).** C++ and
    Rust write runs held to a golden folder in `tests/fixtures/eval/run/` that `read_run`
    validates, with hand-written `.npy` and JSON writers; the native pipeline links both the
    edge rung and `eval`, so it sits above `eval`, in orchestration (`slamlab_vo`, in C++ and
    in Rust; see the Package-layout table).
  - Closed by PR 8: **guided matching beside `matching` (A10).** `matching.features_in_area`
    and `matching.search_by_projection` (ORB-SLAM's `GetFeaturesInArea` and
    `SearchByProjection`) are in `foundations.matching`, for the classic rung too; the C++ and
    Rust cores search a feature grid that finds exactly what Python's brute force finds. The
    full-image-to-level map (run 2, A17) is `image.level_coordinate`, beside
    `full_image_coordinate` in all three languages, and stereo's level-to-level column is
    stated in its terms.
  - **The rotation histogram's bin width (L8).** ORB-SLAM's `ORBmatcher` bins orientation
    changes at 30 degrees (`round(rot / 30)`, bins 0 to 12), 2.5 times wider than slamlab's
    12-degree orientation bins; on six TUM pairs the recall of true matches was 0.90-1.00
    against 0.91-1.00. Revisit if PR 8's tracking loses inliers under fast rotation.
  - **Shared Rust fixture helpers (H8).** Four `slamlab-foundations` test modules each parse
    the shared fixtures with their own copies of `count`, `list` and `picture`; a
    `#[cfg(test)]` fixture module would hold them once.
  - **One "fastest built backend" choice (H18/A13).** The demos each pick a backend with
    their own code; `slamlab.backends` could offer it once.
  - **C++'s `at()` index helper (H19)** is copied into three anonymous namespaces
    (`image.cpp`, `orb.cpp`, `stereo.cpp`); an internal header would hold it once.
  - **`matching`'s popcount temporary (H19).** `_POPCOUNT[block]` makes a (256, M, 32) int64
    array per block of 256 rows, about 130 MB at M = 2000; `np.bitwise_count` (NumPy 2) or
    uint64 views avoid it.
  - **The state of the art, for later rungs (L11).** Learned features that run on a CPU,
    XFeat (Potje et al., CVPR 2024), and SuperPoint with LightGlue (DeTone et al., CVPRW
    2018; Lindenberger et al., ICCV 2023), for the edge-budget comparison. For evaluation,
    the 4-DoF (position and yaw) alignment of visual-inertial runs and relative error over
    sub-trajectory lengths (Zhang and Scaramuzza, IROS 2018) for milestone 6, and KITTI's
    translational error over 100-800 m segments (Geiger et al., CVPR 2012) for milestone 4.
- **Deferred from PR 6's review panel**
  ([report](../reviews/2026-10-04-feature-rust-estimation.md)):
  - **P3P's depth polish stops at a relative singularity in C++ only (L6).** C++ tests the
    Newton step's Jacobian with `FullPivLU::isInvertible()` (a threshold near 3ε), while
    Python's `np.linalg.solve` and Rust's `full_piv_lu().solve` stop only on an exactly zero
    pivot. Near-singular Jacobians therefore polish one step less in C++. Align it if exact
    three-way agreement on near-degenerate triangles matters.
  - **EPnP agrees across the cores only on well-determined problems (L13, run 2).** On noisy
    five-point problems, and on point clouds a few micrometres to centimetres across, C++ and
    Rust already differ from Python on about 1% and about 19% of scenes respectively: the choice
    between EPnP's approximations, and Gauss-Newton's accept test, amplify roundoff in the
    kernel's eigenvectors. Every backend is accurate on such scenes to their conditioning; only
    the decisions differ. A tie margin on the approximations' errors would be the place to
    start if it matters.
  - **A stack-only 3x3 SVD before the edge rung (A11, run 2).** Every 3x3 SVD in Rust now
    allocates (a faer call through `linalg::svd3`), and the edge VO calls `project_to_so3` and
    Umeyama per frame. Give `svd3` a fixed-size, backward-stable path (one-sided Jacobi, as
    Eigen's `JacobiSVD` uses for fixed sizes) behind the same signature, with the edge
    milestone. PR 8's panel (A11) moves it to PR 9, the rung's real-time PR.
- **Deferred from PR 5's review panel**
  ([report](../reviews/2026-10-04-feature-rust-core.md)):
  - Closed by PR 6: **nested summaries and the Rust `Summary` trait (A5).** `ba::Summary`
    and `ransac::RefinedSummary` implement `solve::Summary` by delegation; C++ keeps its
    named structs, since its concept needs data members. Recorded under "Where the summary
    lives".
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
    PR 5's panel (A13, run 2) found the same gap in Rust. PR 6 closed the part that
    `solve` caused: `solve`, `centring_origin` and `shifted` moved into `ba/solve.rs` and
    `ba/solve.cpp`, mirroring Python's `ba/solve.py`, so the facade maps to `problem` both
    ways and `super::solve` from inside `ba` is checked. Left open: C++'s public `ba.hpp`
    still declares `solve`, and every internal `ba` header includes it (PR 6's panel, A3), so
    `bundle` or `layout` could call `solve` unchecked; and an unmapped facade is still exempt.
    The fix is a public `slamlab/foundations/ba/solve.hpp` mapped to the `solve` layer.
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
