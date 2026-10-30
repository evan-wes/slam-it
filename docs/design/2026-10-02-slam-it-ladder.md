# SLAM It! -- a learning ladder for visual SLAM

Status, 2026-10-11: milestones 1 and 2 are complete (see [Milestones](#milestones)). The scaffold (package-per-rung layout in three
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
decisions as the Python reference ([the edge VO note](../notes/edge-vo.md)). PR 9 adds the
budget harness and brings the C++ and Rust pipelines to real time within a laptop-CPU budget
on every sequence of the edge note's set ([the real-time note](../notes/real-time.md)).
PR 10 adds the run viewer, which replays a run frame by frame for each backend side by side
([the viewer note](../notes/viewer.md)), and PR 11 closes milestone 2 with your own video: a
phone video or a Stray Scanner recording uploaded through a page, run by a worker and opened in
the viewer ([its note](../notes/your-own-video.md)).

## Purpose

SLAM It! is a localization-and-mapping project in its own right. It has two goals:

1. **Learning SLAM across the compute spectrum.** Every component is built by us, on the
   field's standard libraries where they serve, and explained in a design note as it is
   built: learning SLAM includes learning its tools (decided with evan-wes on 2026-10-17). The
   range runs from first-principles geometry to learned dense SLAM, and from a Pi-class
   compute budget to a large GPU.
2. **Real-time SLAM.** At least one rung runs in real time under an edge-class budget.

It is also the geometry source for [reifyscene](https://github.com/evan-wes/reifyscene). That is a
consumer relationship, not the project's identity (see [Boundary with reifyscene](#boundary-with-reifyscene)).

It is a hobby and learning project with no commercial use.

## Non-goals

- Wrapping existing SLAM systems as black boxes. Reference implementations (DROID-SLAM,
  MegaSaM, ORB-SLAM3) are for study and parity checks only; ORB-SLAM3 is also run here, as a
  yardstick, once milestone 2B is done. The libraries such systems are built from (GTSAM,
  Ceres, OpenCV and the like) are welcome ([Languages](#languages)).
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
   edge: sparse SLAM classic heavy        dense BA + depth prior
        │           (offline SfM)              │
        ▼                                      ▼
   visual-inertial                     learned dense (DROID-style)
                                               │
                                               ▼
                                     heaviest (MegaSaM-style)
```

| Rung | What it is | Compute target |
|---|---|---|
| **0: Foundations** | Camera models, SE(3)/Sim(3), 8/5-point, RANSAC, triangulation, PnP, Gauss-Newton/LM bundle adjustment with a Schur complement; state estimation, batch (Gauss-Newton/LM) and recursive (error-state filter); the front end: fixed-point images, ORB features, Hamming matching, rectified stereo; place recognition (milestone 2B) | CPU |
| **Edge: sparse VO, then SLAM** | Tracking on rung 0's ORB front end, keyframes, local BA (ORB-SLAM-lite, milestone 2); then relocalisation, a covisibility graph, loop closure and an atlas of maps (ORB-SLAM3's, milestones 2A and 2B); monocular, stereo and RGB-D modes (RGB-D from a sensor now, predicted depth later) | Real-time under an edge budget |
| **Visual-inertial** | IMU preintegration shared by two estimators, a filter (MSCKF) and a smoother (sliding-window optimisation), on top of the edge VO | Edge budget |
| **Classic heavy** | Incremental SfM and global BA over a whole clip, its loops found by rung 0's place recognition | Offline, minutes per clip |
| **Dense BA + depth prior** | Dense bundle adjustment over per-pixel depth, with a monocular depth prior and motion masks | GPU |
| **Learned dense** | DROID-style: correlation volumes, recurrent flow update, dense-BA layer | GPU, around real time |
| **Heaviest** | MegaSaM-style: uncertainty-weighted dense BA, focal-length optimisation, robust to dynamic scenes | Large GPU, offline |

## Languages

| Language | Role |
|---|---|
| **Python** | The readable reference implementation of every algorithm; orchestration, evaluation, and the learned rungs (PyTorch). Held to the same bar of results as C++ and Rust, on the synthetic rooms; never to real time |
| **C++** | The performance core: rung 0, the edge rung, the edge budgets, and dense-BA kernels if profiling calls for them; outside the cores, `eval`'s native readers, run writer and process readings for a budget (`slam_it_eval`) and the native VO program (`slam_it_vo`, in orchestration); and the calibration side track (`slam_it_calib`), beside Python, without Rust. Showing C++ is a goal in its own right. |
| **Rust** | Rung 0 and the edge rung, each in lock-step with Python and C++: every slice of those rungs lands in all three languages together; outside the cores, `eval`'s native readers, run writer and process readings for a budget (`slam-it-eval`) and the native VO program (`slam_it_vo`, crate `slam-it-orchestration`). The other rungs have no Rust. |
| **JavaScript** | The run viewer's page (`viewer/`, from PR 10), and the upload and status pages beside it (PR 11): plain modules with no build step, three.js from a CDN pinned by its hashes. The viewer reads only bundles, on its own site; the upload and status pages, the upload server's API. No algorithm runs in it: its logic modules work out what a frame shows and never touch the page, so Node tests them; `tsc --checkJs` checks their JSDoc types. Its development tools (TypeScript, three.js's types, prettier) are pinned in `viewer/package-lock.json`. |

**Runtime dependencies.** Well-tested libraries are welcome, the SLAM parts included (decided
with evan-wes on 2026-10-17): learning SLAM includes learning its standard tools, and no
library is kept out of the runtime so that a test can use it as an oracle; such a test checks
against something else. Adding one, to the Python package or to a core, is still a change to
this document first.
- **Python:** numpy. Pillow too, but in `slam_it.eval.images` alone, which reads image files
  (milestone 2 reads PNG frames and depth maps) and encodes the run viewer's JPEGs (PR 10): the
  algorithm rungs take arrays, never files, and an import-linter contract keeps Pillow in that
  one module; another keeps what reads, finds or hashes a file (`json`, `io`, `os`, `pathlib`,
  `hashlib`, ...) out of every algorithm rung, whose files eval reads for them (a
  place-recognition vocabulary's folder, `slam_it.eval.vocabularies`, from milestone 2B's B1),
  and `scripts/check_layers.py` holds the rungs' C++ and Rust to the same (no `<fstream>`,
  `<filesystem>` or `<cstdio>`, no `std::fs`, outside their tests).
  PyAV, an optional extra (`video`, from PR 11), decodes phone videos in
  `slam_it.eval.video` alone, kept there by a contract of its own. Its wheels bundle an
  FFmpeg built with GPL encoders (x264, x265); the project installs it from PyPI and ships no
  binaries. A dashcam's recordings (`slam_it.eval.dashcam`, from 2026-10-17) add the optional
  `dashcam` extra: PyAV again, decoding through `slam_it.eval.video`; pytesseract, reading the
  burned-in overlay with the Tesseract program (Apache-2.0, from conda-forge), in
  `slam_it.eval.dashcam.overlay` alone; and pycolmap, self-calibrating each camera's lens by
  SfM, in `slam_it.eval.dashcam.calibrate` and the BA reference `slam_it.eval.colmap_reference`
  alone. Contracts keep each in its module, and the pipelines reach neither.
  `slam_it.eval.rosbag` reads ROS 1 bags by hand, with no dependency (the calibration side
  track).
- **The upload server** (`server/`, the package `slam_it_server`, from PR 11): FastAPI
  with pydantic, which its requests are read with, uvicorn, and boto3 with botocore, with httpx
  for its tests. It never imports `slam_it`.
- **C++:** Eigen. The `slam_it_eval` library reads PNGs with stb_image (vendored in
  `third_party/stb`, PNG support only; MIT or public domain), outside the algorithm cores.
- **Rust:** nalgebra, the matrix type, and faer for the general SVDs and the general
  eigendecomposition, single-threaded, behind the crate-private `linalg` (nalgebra's SVDs are
  not backward stable; see the Rust port's note); the 3x3 SVDs, since PR 9, a port of Eigen's
  `JacobiSVD` in the same module (the real-time note). The `slam-it-eval` crate reads PNGs with
  the pure-Rust `png` crate, outside the algorithm cores.
- **GTSAM 4.3, from milestone 2B's B3,** in all three languages, behind rung 0's
  `slam_it.foundations.graphs`: the loop's Sim(3) refinement, the essential graph and global
  BA. Built by `setup_slam_it.sh` from its release (static, with `EIGEN_NO_CPUID`, as the cores
  build, so it repeats on every core), not taken from conda-forge, whose build did not; C++
  links it, Python uses its official bindings built over the same library, and Rust reaches it
  through a bridge with `cxx` (its runtime crate in `slam-it-foundations`, the one crate allowed
  `unsafe`, in that module alone) ([the 2B design](2026-10-14-online-slam.md), section 10).
  The worker's image builds the C++ library the same way, for the C++ program its runs use
  (`worker.py` runs backend `"cpp"`); its Python, which imports GTSAM but never solves with it,
  takes PyPI's 4.3.0 wheel, pinned by its hash (`scripts/locks/worker-gtsam.lock`). GTSAM is in
  no header: `graphs.cpp` and the Rust bridge's source include it, and nothing else does, in
  any language (the contracts below; B3's review panel, A3, A10 and run 2's A17, A21).
- The three readers read a file the same way (gray from a gray file as stored, from a colour
  one through `image.to_gray`), held to shared PNG fixtures.
- The native run writers and VO programs add none (decided for PR 8): they write JSON and
  `.npy` by hand, and the programs take their options as command-line flags, so nothing
  native reads JSON.

**Parity.** Shared fixtures (fixed inputs, expected outputs) live in `tests/fixtures/`. Every
C++ and Rust implementation is checked against the Python reference, stage by stage, within
stated tolerances. Whole runs are held to their results, not to one another (decided with
evan-wes on 2026-10-17, from milestone 2B's B3): each language keeps a golden record of its
own runs, its regression check, and all three must clear one bar on the same data, namely the
frames tracked, the maps, the loops closed with none at a wrong place, and the final
trajectory's error. C++ and Rust must also clear a latency bar; Python is never held to real
time. All three are benchmarked under the same compute budgets, which gives a "same
algorithm, three languages" comparison of speed, memory and lines of code.

**Packaging.** There are three importable parts. The two compiled ones are optional. Each
algorithm's `backend=` argument picks an implementation at runtime, and `slam_it.backends`
locates the compiled ones and reports which are built.

| Import | Contents | Build |
|---|---|---|
| `slam_it` | Pure Python: reference implementations, orchestration, evaluation | Pure Python |
| `slam_it._cpp` | The C++ core | nanobind, built with scikit-build-core (CMake), inside the `slam_it` package |
| `slam_it_rs` | The Rust subset | PyO3, built with maturin, as its own top-level module |

The Rust module is top-level rather than `slam_it._rs`. maturin and scikit-build-core cannot
both build into one Python package, and installing a second distribution into `slam_it/` breaks
under editable installs.

C++ and Rust sources also build standalone (CMake, cargo) without Python, for the edge budgets
and a possible later phone target.

**The native VO programs** (`slam_it_vo`, in C++ and in Rust; from PR 8) run a whole sequence
without Python: they read the dataset natively and write the run folder.
`slam_it.orchestration.vo.run(..., backend=)` runs the Python pipeline in-process, or launches
the chosen native program and reads its run back. scikit-build installs the C++ program into
the package (`slam_it/bin/`); `setup_slam_it.sh` and `check.sh` build the Rust one with
cargo (`rust/target/release/`); `slam_it.backends` locates both, never through `PATH`.

**The binding boundary is numpy arrays.**
- Bindings take and return float64 C-contiguous numpy arrays (uint8 for images): nanobind's
  `ndarray` and the Rust `numpy` crate.
- The cores stay numpy-free, taking fixed-size arrays, spans or slices.
- Batched data therefore crosses the boundary without per-element Python objects, so the
  "same algorithm, three languages" comparison measures the algorithms, not marshalling.

**The error contract is one shape in every language.**
- Fallible core functions return a C++23 `std::expected<T, slam_it::foundations::Error>` or
  a Rust `Result<T, slam_it_foundations::error::Error>`. Nothing throws on the cores' hot
  paths.
- Both share one list of error kinds:
  - `InvalidInput`: shape, range or finiteness;
  - `Degenerate`: pure rotation, collinear points, too few correspondences;
  - `NotConverged`: strict mode was requested and the convergence criteria were not met, or
    the iteration diverged with no usable estimate. Reaching the iteration cap is not an
    error (see the solver contract below).
- `eval`'s native readers (`slam_it_eval`, `slam-it-eval`) use the same kinds. Every failure
  there, a missing or unreadable file included, is `InvalidInput` naming the file, since the
  kinds have none for I/O; Python's `slam_it.eval` raises `FileNotFoundError` for a missing
  file instead.
- The bindings raise the Python exception of the same kind, from `slam_it.foundations.errors`
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
    - C++: `cpp/include/slam_it/foundations/solve.hpp`, a `Summary` concept and
      `Solution<State, Summary>`, returned as `std::expected<Solution<…>, Error>`;
    - Rust: `slam_it_foundations::solve`, a `Summary` trait and `Solution<State, S>`.

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
(`slam_it.foundations.image` states each rounding), so the three languages' images are
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

**A run rounds the same on every core** (from PR 10). Eigen blocks a large product by the
cache sizes of the core that runs its first one, and the blocks set the order of its sums; on a
hybrid CPU the core types report different caches, so the C++ program's runs differed in their
last bits by where the process started. Every rung compiles with `EIGEN_NO_CPUID` (PUBLIC, in
`CMakeLists.txt`), so Eigen blocks by its own default sizes on every core, in every program,
test and the extension, and a run repeats bit for bit; a native test fails without it. Rust's
products (faer's) block by cache sizes read once per run for the whole machine, so its runs
repeat on every core of one machine, though another machine may block or vectorise them
differently (faer picks its SIMD kernels when it runs). The
cores build for SSE2: enabling AVX would add a second threat, Eigen's vectorised reductions
choosing their scalar head by an operand's address at run time.

**Structured problems cross as structs of arrays.** A BA problem, for example, crosses as:
- float64 values;
- `int64` index arrays (`cam_idx`, `pt_idx`);
- bool masks (fixed variables);
- options as scalars, with integer codes for enums.

A place-recognition vocabulary's tree crosses as its files store it (milestone 2B's B1):
`parents`, `children` and `words` int32, not widened to int64, so the default vocabulary's
arrays are not doubled on every call (B1's panel, A5).

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
languages fail the same way. One exception, a vocabulary's checksums (B1's panel, A1): the native
readers (`slam_it_eval`, `slam-it-eval`) check a vocabulary's header (its format and descriptor
versions) and its tree, but not each array's SHA-256, which would need a hash written by hand
in two languages. Python checks them (`slam_it.eval.vocabularies.read`), and `vo.run` reads the
folder before any program runs; a native program run on its own trusts its arrays, and the
checksums its `run.json` records are its folder's claim, verified only when `vo.run` launched
it (the worker and the measures all run through `vo.run`).

**Generic in the core, concrete in the bindings.**
- Generic algorithms (the least-squares solver, RANSAC) are templates in C++ and traits in
  Rust, and generic over a Protocol in Python.
- The bindings expose only concrete instances (`bundle_adjust`, `essential_ransac`, …), so
  no compiled hot loop calls back into Python.
- A generic module with no concrete binding of its own (e.g. `slam_it.foundations.nls`) is
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
| `slam_it.foundations` | library `slam_it_foundations`, `slam_it::foundations` | crate `slam-it-foundations` | Rung 0 |
| `slam_it.edge` | library `slam_it_edge` | crate `slam-it-edge` | Edge VO |
| `slam_it.vi` | library `slam_it_vi` | -- | Visual-inertial |
| `slam_it.classic` | library `slam_it_classic` | -- | Classic heavy |
| `slam_it.calib` | library `slam_it_calib` | -- | Sensor calibration (the side track) |
| `slam_it.dense` | -- | -- | Dense BA + depth prior |
| `slam_it.learned` | -- | -- | DROID- and MegaSaM-style |
| `slam_it.eval` | library `slam_it_eval`, `slam_it::eval` (native PNG and dataset readers, run writer, a program's process for its run, a vocabulary's folder) | crate `slam-it-eval` (the same) | Dataset readers, external references (COLMAP), metrics, benchmarks, budget harness; a phone video's import and intrinsics (`video`, `lenses`, Python only, from PR 11); dataset imports to TUM format (`zju`, `advio`, `arkitscenes`, with `lidar` and `sensor_yaml`), Python only, from milestone 2A's A1, and 7-Scenes' (`sevenscenes`, from milestone 2B's B4); a place-recognition vocabulary's folder, the committed one with it (`vocabularies`, from milestone 2B's B1) |
| `slam_it.orchestration` | library `slam_it_orchestration` and program `slam_it_vo` (`cpp/apps/orchestration/`; from PR 8) | crate `slam-it-orchestration` and its program `slam_it_vo` (from PR 8) | End-to-end pipelines, the bake of their runs for the run viewer (`bundles`, Python only, above `vo`), and the worker that runs the upload server's jobs (`worker`, Python only, above `bundles`, from PR 11) |
| `slam_it_server` (`server/`, its own package) | -- | -- | The upload server (from PR 11): the job list, storage's links, the guards; never imports `slam_it` |

The pages, `viewer/` (from PR 10), sit outside the packages: the run viewer reads only the
bundles the bake writes, on its own site; the upload and status pages (from PR 11) talk to the
upload server's API. Nothing imports them.

Packages for rungs that have no code yet are stubs: a docstring stating the rung, its
milestone and its compute budget. C++ libraries and Rust crates appear as each rung gets
code. The extension submodules mirror the tree: `slam_it._cpp.foundations.so3` and
`slam_it_rs.foundations.so3` implement `slam_it.foundations.so3`. Bindings are one file per
module (`cpp/bindings/<rung>/<module>.cpp`, `rust/slam-it-py/src/<rung>/<module>.rs`), with
thin roots that only wire them.

**Shared interface types** live in rung 0, so every rung can use them:
- `SlamInput`, `SlamResult` and the depth-provider Protocol (needed by both the edge rung's
  RGB-D mode and the dense rung, which are independent siblings) go in
  `slam_it.foundations.interfaces`, created with the first rung that needs them. That is
  milestone 3 (decided for PR 8): PR 8's RGB-D mode reads the sensor's depth maps from the
  dataset, and predicted depth, the first provider, arrives with milestone 3;
- the error classes are in `slam_it.foundations.errors`.

**Dispatch.**
- Each public algorithm validates its input once, using the shared checks in
  `slam_it.foundations.arrays`.
- It then computes with the Python reference, or calls the backend chosen by its
  `backend=` argument.
- `slam_it.backends` only locates compiled implementations and never knows what they
  compute.
- Each algorithm module types its compiled backend with a Protocol named `Implementation`. One
  generic test walks every extension submodule and checks it against the `Implementation`
  of the `slam_it` module at the same path: same functions, same parameter names. The parity
  tests check the values.

**Dependency contracts** (import-linter, `pyproject.toml`):
- **Layers,** top to bottom: `orchestration`, `eval`, `learned | vi`,
  `dense | edge | classic | calib`, `foundations`, `backends`. Siblings joined by `|` are independent
  of each other. The contract is exhaustive: a new top-level package fails until it is
  placed.
- **Inside packages,** the order of rung 0's, bundle adjustment's, the edge rung's, eval's and
  orchestration's modules (orchestration's run program below its bake). These are exhaustive
  too, from PR 10: a new module fails until it is given a layer.
- **Visual-inertial** builds on `edge` only; **learned** builds on `dense` only.
- **CPU rungs** (`foundations`, `edge`, `vi`, `backends`) never import PyTorch or a GPU
  stack, including torch-based geometry libraries (lietorch, pypose, kornia, theseus), which
  are banned by name because import-linter cannot see through them. PyTorch only ever goes
  in an optional extra, never in `dependencies`.
- **No algorithm package imports a depth model.** Depth models are inputs through a
  provider interface.
- **SLAM It! never imports reifyscene.**
- **The upload server and `slam_it` stay apart** (from PR 11): the server never imports
  `slam_it`, PyAV, Pillow, OpenCV or reifyscene, so the process holding the storage keys
  never parses a video; `slam_it` never imports the server or its web stack; the server's
  own modules keep an order (`__main__`, `app`, `jobs | storage`, `config`).
- **The pipeline never needs the `video` extra:** of `orchestration`, only the worker imports
  `eval.video`, which alone imports PyAV.
- **GTSAM stays behind rung 0's factor graphs** (from milestone 2B's B3): of `slam_it`, only
  `foundations.graphs` imports `gtsam`, and its converters to GTSAM's types are private.

**The same layering holds in C++ and Rust.** Each rung is its own C++ library and Rust
crate, so a rung can only link what it declares. `scripts/check_layers.py` reads the
contracts above from `pyproject.toml` and fails layer 1 on:
- a cross-rung `#include` that the contracts forbid;
- a forbidden `slam_it_add_rung(... DEPENDS ...)` link;
- a forbidden crate dependency;
- inside a rung, a C++ `#include` or a Rust `crate::`/`super::` path that breaks the rung's
  or a package's internal order (such as "Rung 0's internal order"), chains through unlisted
  modules included;
- a file header (`<fstream>`, `<filesystem>`, `<cstdio>`) or `std::fs` in a rung kept from
  files, outside its tests;
- a GTSAM header included anywhere but a confined module's sources: `cpp/src/foundations/
  graphs.cpp` and `rust/slam-it-foundations/cxx/graphs_bridge.cpp`, never a header, since the
  env's compiler finds GTSAM's headers in the env's prefix whatever a target links.

The three languages therefore share one source of truth.

**Native benchmarks** are developer tools outside the cores: `cpp/bench/<rung>/` (Google
Benchmark, behind the CMake option `SLAM_IT_BUILD_BENCHMARKS`) and each Rust crate's
`benches/` (criterion), with a crate's `examples/` for standalone runs. They are held to the
ladder like the code they time, built and linted by layer 1, and never run by it. A
benchmark that reads dataset files links the native readers (`slam_it_eval`,
`slam-it-eval`), so it sits above them, in `cpp/bench/eval/` or the `slam-it-eval` crate's
`benches/`; a rung-0 benchmark builds its input in code (as `orb_extract` does).
`slam_it.eval` keeps the Python benchmarks and the budget harness (`slam_it.eval.budgets`,
PR 9), which runs a program pinned and capped and judges its frames against their deadline;
from milestone 7 it will drive the native benchmarks on the edge targets.

## Milestones

Learning comes first: the left branch in order. Two early stops on the right branch give
reifyscene a measured payoff. After milestone 2 come 2A and 2B, a complete SLAM system,
then milestone 3 (decided 2026-10-12, below).

| # | Milestone | Languages | Measured on |
|---|---|---|---|
| 1 | Rung 0: foundations | Py, C++, Rust | Synthetic scenes with exact answers |
| 2 | Sparse VO under a laptop-CPU budget: monocular, stereo and RGB-D; the run viewer, and your own phone video | Py, C++, Rust | TUM RGB-D, EuRoC |
| 2A | VO that survives a phone: a hard test set measured, settings by frame size, Stray Scanner's depth as RGB-D, a new map after a lost track | Py, C++, Rust | TUM RGB-D (its hard sequences), ZJU-SenseTime, ADVIO, ARKitScenes |
| 2B | Online SLAM: place recognition, relocalisation, the covisibility graph, loop closure, threads, merging maps | Py, C++, Rust | TUM RGB-D, EuRoC, 7-Scenes |
| 3 | **reifyscene payoff 1:** depth-aware, motion-masked BA refining VGGT's cameras | Py (+C++ solver) | reifyscene lab-coat masked PSNR; spin GT poses |
| 4 | Classic heavy: incremental SfM and global BA, offline (place recognition and loop closure from 2B) | Py, C++ | KITTI odometry |
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
| 7 | **Front end and evaluation.** Images (Pillow in `slam_it.eval`; native PNG readers outside the cores, `slam_it_eval` and `slam-it-eval`), grayscale pyramids, blur and bilinear remapping. ORB, our own: multi-scale FAST spread over the image, intensity-centroid orientation, rBRIEF descriptors, Hamming matching with a ratio test. Stereo rectification from EuRoC's calibration, row-constrained matching with subpixel refinement, depth from disparity. The EuRoC reader; TUM's depth maps in the library; the demos on the shared front end. ATE (Sim(3) alignment for monocular, SE(3) for stereo and RGB-D) and RPE; the run format. |
| 8 | **The pipeline.** Initialisation: two-view for monocular, retrying on later frames when the parallax is low; from depth for stereo and RGB-D, metric from the start. Tracking: a constant-velocity prediction, map points matched by projection, PnP RANSAC as the fallback, motion-only pose refinement. Keyframe selection; local mapping (triangulation, local BA over a keyframe window, culling of points and keyframes). Native programs (C++ and Rust) that run a whole sequence. Results on TUM (monocular, RGB-D) and EuRoC (monocular, stereo), with rerun recordings. Planned in [the edge VO design](2026-10-05-edge-vo.md); built as [the edge VO note](../notes/edge-vo.md) describes. |
| 9 | **Budgets and real time.** The `laptop-cpu` harness (pinned cores, capped memory, a per-frame deadline); latency (p50, p95) and peak memory; profiling the C++ and Rust pipelines to real time; the three-language comparison. Python is measured but not held to real time. Built as [the real-time note](../notes/real-time.md) describes. |
| 10 | **The run viewer** (see [Public presentation](#public-presentation)). Run format v3: per-frame solver summaries, point and keyframe lifetimes, and opt-in feature records, in all three languages. The bake into bundles, and the page. Planned in [the run viewer design](2026-10-09-run-viewer.md). |
| 11 | **Your own video.** Phone video import, with intrinsics from the best source (a Stray Scanner recording, a Gyroflow lens profile, the 35mm-equivalent focal length, a nominal field of view) and undistortion to pinhole. The upload server and its worker, packaged for Railway. Planned in [the run viewer design](2026-10-09-run-viewer.md). |

The open items aimed at milestone 2 close in these PRs: the depth-map loader and the shared
front end in PR 7; the low-parallax retry and the non-invertible homography in PR 8.

**The front end is rung 0's** (decided in PR 7). Fixed-point images, ORB features, Hamming
matching and rectified stereo live in `slam_it.foundations`, not in the edge rung: the
classic rung (milestone 4) matches the same features, so both build on one front end. The
edge rung holds what is its own, tracking, keyframes and local mapping.

**The edge rung is ORB-SLAM-lite** (decided for PR 8; the plan is
[the edge VO design](2026-10-05-edge-vo.md)):
- **What it leaves out of ORB-SLAM:** no loop closure (milestone 2B's B3); one thread, mapping
  after tracking, so a run is deterministic, and so mapping is never busy: it neither refuses a
  keyframe nor cuts a local BA short, as ORB-SLAM's mapping thread does (milestone 2B's B3b
  gives tracking, mapping and loop closing threads of their own); no map reuse (milestone 2B's
  B4 takes a kept map up again and merges maps).
- **The covisibility graph** (milestone 2B's B2), in place of a sliding window of the last 10
  keyframes, gives tracking's local map, local BA, keyframe culling, triangulation and fusion
  their keyframes, as ORB-SLAM's does, and keeps the spanning tree for B3's essential graph
  ([the 2B design](2026-10-14-online-slam.md), section 8).
- **A lost track is relocalised** through a bag of words, for up to 3 s, as visual-only
  ORB-SLAM3's recently lost track is: against the whole active map (milestone 2B's B1), and
  since B4 against every map the run keeps as well, as ORB-SLAM3's paper describes; found in a
  kept map, the track takes it up, the map it left kept or reset as a lost map is. Otherwise it
  closes its map (kept, or reset if it has 10 keyframes or fewer, such a map lost after 5
  failures) and the next frame starts a new one, as ORB-SLAM3's atlas does (milestone 2A's
  A2); B4 joins maps where a recognised place links them.
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
  golden record of the room's runs (the edge VO design, §3). *From milestone 2B's B3:* a
  golden record per language, and one bar for all three ([Languages](#languages), "Parity").
- **Its generic parts go in rung 0:** guided matching by projection and a feature grid in
  `foundations.matching`, the full-image-to-level map in `foundations.image`.

**A side track: rig calibration** (Python and C++; planned in
[the rig calibration design](2026-10-05-rig-calibration.md)). Beside milestone 2, and
independent of it: an RGB-D camera, a 2D lidar and a rangefinder calibrated against each other
from the rig's own motion, as a study in observability. Two PRs, C1 (reading ROS 1 bags,
Python) and C2 (`slam_it.calib`, Python and C++), in their own worktree.

**Milestones 2A and 2B: from odometry to a SLAM system** (decided 2026-10-12, before
milestone 3). The first real phone clips lost the track early: one recorded in the browser,
360x480 with its lens guessed, after 2 s of tracking; a Stray Scanner recording after 13 s,
while it panned and its map, which a single camera grows only from sideways motion, ran out of
points. Milestone 2's pipeline is odometry by design: a lost track ends the run, and nothing
relocalises. Before reifyscene's payoffs, the owner chose a complete SLAM system, ORB-SLAM3's:
tracking, local mapping and loop closing share a covisibility graph; place recognition
relocalises a lost track and closes loops; and an atlas of maps lets a run go on after a loss
and joins its maps when a place is seen again (Mur-Artal and Tardós 2017; Campos et al. 2021).
Both milestones are the real-time pipeline's, in Python, C++ and Rust, each planned in a design
doc of its own. Rust stays in both, as it was always meant to: no language is dropped partway
through the project.

- **2A: VO that survives a phone.**
  - A public test set with ground truth, measured by the share of each sequence tracked: TUM
    RGB-D's hard sequences (turning, a kidnap, fast sweeps, low texture), ZJU-SenseTime's phone
    sequences, ADVIO's iPhone video, and ARKitScenes for LiDAR depth
    ([the 2A design](2026-10-12-phone-robust-vo.md)).
  - The feature settings scaled to the frame size: ORB-SLAM's 1,000 features are for 640x480.
  - What a guessed lens costs, measured on Stray Scanner recordings against their exact
    intrinsics.
  - Stray Scanner's LiDAR depth, with its confidence, as RGB-D, so that a turning camera's
    keyframes still make points.
  - A new map after a lost track, as ORB-SLAM3 starts one, so that the rest of a clip is
    processed, in segments the viewer shows; ORB-SLAM's reset of a map lost early with it
    (open item L13).
  - A warning on the upload and status pages, never a refusal, for a video recorded in the
    browser (on an iPhone, 360x480 and no lens metadata) or frames under 480 pixels on their
    short side.
- **2B: online SLAM**, ORB-SLAM's own parts ([the 2B design](2026-10-14-online-slam.md)):
  - place recognition in rung 0, for the edge rung now and the classic rung later: a
    vocabulary of our own ORB descriptors' binary words (DBoW2's hierarchical k-means,
    Gálvez-López and Tardós 2012), trained by a Python tool on public sequences no evaluation
    uses, read by all three languages; and the keyframe database, an inverted index, the edge
    rung's, beside its map (B1); when the classic rung needs retrieval, the generic index moves
    to rung 0 rather than being copied;
  - relocalisation before a new map is started (ORB-SLAM's `Tracking::Relocalization`:
    candidates from the database, matches by word, PnP RANSAC, a guided search), against the
    active map (B1) and every kept map (B4);
  - the covisibility graph and the essential graph, in place of the window of 10 keyframes
    (B2; the essential graph's loop links come with loop closure);
  - loop closure: candidates checked for consistency, the loop's Sim(3) solved by RANSAC over
    Horn's closed form, points fused, the essential graph optimised in Sim(3) on GTSAM, then a
    global BA;
  - maps joined where a recognised place links them (ORB-SLAM3's merging);
  - measured on TUM and EuRoC, whose sequences close loops, and on 7-Scenes for relocalisation.
- **2B's forks, for its design** (ORB-SLAM's answer first):
  - the vocabulary: ORB-SLAM's is trained on Bovisa from its own descriptors; ours are not
    bit for bit theirs, so we train our own;
  - threads: ORB-SLAM closes loops, and runs its global BA, in threads of their own. Decided
    with evan-wes on 2026-10-17: B3 closes loops in the one worker, and B3b gives tracking,
    mapping and loop closing real threads, a design of our own compared with ORB-SLAM's scheme
    built in our C++, with a switch that runs their work in order
    ([the 2B design](2026-10-14-online-slam.md), section 4);
  - ties: candidate keyframes ranked by score with an explicit order among equals (their
    ids), so that the three languages pick the same.
- **Milestone 4 splits:** place recognition and loop closure move to 2B; milestone 4 keeps the
  offline rung, incremental SfM and global BA over a whole clip (KITTI), its loops found by 2B's
  place recognition.
- **After 2B, milestone 3.** Still open: whether the learned rungs (milestones 5, 8 and 9) come
  before reifyscene's payoffs or after.

**Why milestone 3 sits where it does.** reifyscene's old `pose_refine` was a depth-blind BA with no
motion masks. Swapping its new poses in while keeping the old depth produced the "shadow" behind
moving people. Milestone 3 is the corrected version of that same idea: rung 0's solver, with a
depth prior and motion masks. It reuses what milestones 1 and 2 teach and needs only a small
adapter. Its target is the gap between plain VGGT and the old bundle-adjusted cameras on lab-coat
(27.42 vs 29.04 dB masked PSNR). It follows milestones 2A and 2B, a complete SLAM system first.

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

- **The dependency runs one way.** SLAM It! never imports reifyscene. reifyscene
  reaches SLAM It! through one thin adapter, a new `GeometryInitializer` backend
  (`reifyscene/geometry/slam_wrapper.py`) beside `vggt_wrapper.py`. The existing contract (frames
  in, `GeometryArtifact` out) does not change.
- **SLAM It!'s interface** uses plain arrays and its own result type:

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

- **Packaging.** During development, reifyscene installs `slam-it` editable from the sibling
  folder. reifyscene's `pyproject.toml` has a `slam` extra pinned to a SLAM It! git tag, by a
  direct git reference, so that no package of the same name on PyPI can stand in for it; the
  earliest tag it can name is v0.3.7.0, the first that ships `slam_it`. The two change in
  lockstep: when SLAM It!'s API changes, the adapter changes with it and the pin moves.
- **Yardsticks are split.** SLAM It! benchmarks on standard SLAM datasets. reifyscene-specific
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
- **As built (PR 9; [the real-time note](../notes/real-time.md)):** `slam_it.eval.budgets` holds
  `laptop-cpu` (4 performance cores, 8 GiB). It pins with `taskset` (a user session here has no
  cpuset controller) and caps memory with `systemd-run --scope`, `MemoryMax` and no swap, so a
  program over the cap is killed. No `CPUQuota`: the laptop's cores are whole. The deadline is
  judged, not enforced: a frame past it is reported late, none is dropped. It holds building and
  tracking a frame, ORB-SLAM's tracking thread; mapping, in the same thread here so a run stays
  deterministic until milestone 2B's B3b gives it a thread of its own, is judged by its load,
  over the run and over its busiest 10 seconds. Each
  program records its own process in its run: its peak memory, the CPUs it could run on and its
  memory cap.
- **Where a rung declares its budget:**
  - its package docstring states the target budgets and what it may depend on;
  - the import-linter contracts enforce the part that is checkable, namely no PyTorch or GPU
    stack on CPU rungs;
  - tests that need a GPU or a budget-capped run are marked `@pytest.mark.gpu` or
    `@pytest.mark.budget`.
  
  `scripts/check.sh` runs the CPU subset on every PR. The budget runs are opt-in
  (`python -m pytest -m budget`, since PR 9); GPU runs get their own opt-in script when those
  rungs arrive, along with a separate light environment for the edge rungs, so that having no
  PyTorch is also proven by installation.

## Datasets

The data lives outside the repo, in `../datasets/` (that is,
`../datasets`). Disk space is checked before each download.

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
| ZJU-SenseTime VISLAM | Handheld phones (Xiaomi Mi 8, iPhone X): rapid rotation, shaking, occlusion; VICON GT | Milestone 2A |
| ADVIO | A real iPhone's video, walking indoors and out; GT from IMU and fixation points | Milestone 2A |
| ARKitScenes | iPad LiDAR depth with ARKit's poses (a reference) | Milestone 2A (a subset) |
| 7-Scenes | Relocalization | Milestone 2B |

## Metrics

- **Trajectory:** ATE (after Sim(3) or SE(3) alignment, as the rung warrants) and RPE, as in
  Sturm et al.'s TUM benchmark (`slam_it.eval.metrics`): Sim(3) for monocular runs, SE(3) for
  stereo and RGB-D; the RPE's estimate scaled by the ATE's scale.
- **The run format** (`slam_it.eval.runs`), written by every backend's pipeline and read by the
  evaluation and the run viewer: a folder of `run.json` (dataset, sequence, mode, backend,
  settings, and what the program recorded of its process: peak memory, CPUs, memory cap),
  `trajectory.txt` in TUM's format (so other systems' trajectories score the same way), the
  keyframes and map points as `.npy`, and `diagnostics.jsonl`, one object per frame. Since
  milestone 2A's A2 a run may hold several maps, each in its own frame: `pose_maps.npy` and
  its keyframe and point twins say whose each row is, and `trajectory.txt` marks each map's
  first pose with a `# map <n>` line, so an outside tool aligns one map at a time.
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

- **Layer 1, every PR:** `scripts/check.sh` runs deterministic checks in every language,
  locally and in CI (offline for every PR, `scripts/offline_ci.sh`; on GitHub on demand and
  for releases):
  - documentation present and well-formed: numpy-style in Python, Doxygen in C++, rustdoc in
    Rust;
  - type hints (`mypy --strict`);
  - lint and formatting;
  - dependency rules (import-linter);
  - duplication (jscpd);
  - every test suite.
- **Layer 2, the review panel (`/slam-it-review-panel`),** on PATCH-or-larger changes: three
  reviewer agents in `.claude/agents/`:
  - **hygiene:** cleanliness, readability, documentation quality;
  - **architecture:** modularity, the language split, the ladder's progression by compute
    budget;
  - **algorithm:** correctness against standard formulations, numerical robustness, and the
    state of the art, with citations.

  All three call out code that is not production quality. A smaller change can run one
  reviewer alone.
- **Measurement, once per PR** (decided with evan-wes on 2026-10-17): between commits, only
  the checks a change touches run. A PR's measurement is C++ and Rust on just the sequences
  its claim needs, sharded; the whole test set reruns only when a PR changes how tracking
  behaves across the board. Python is held to the bar on the synthetic rooms, in layer 1, and
  runs on real data only when asked.
- **Results are documented and decided together.** Each run's report is saved in
  `docs/reviews/` and committed with the PR. The owner decides the action on every finding;
  the report records each resolution.
- **PR descriptions record the agentic process** (`.github/pull_request_template.md`): which
  agents did what, the panel's verdicts, and the owner's decisions.

## Public presentation

SLAM It! is presented the way reifyscene is: a static showcase page and an interactive run
viewer, in the same visual style (IBM Plex, panel layout, a frame scrubber). It is hosted on
Railway, at `slam-it.timedilated.com`, because PR 11's uploads need a server and a worker.
A staging deploy on Railway's own address, with every page behind an invite code, comes first.
Nothing is published before the repo goes public (see
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
- **Your own video:** anyone with an invite code uploads a phone video, or a Stray Scanner
  recording for exact intrinsics and ARKit's trajectory as ground truth. It runs on Railway and
  opens in the viewer. An upload is deleted when its import ends, and a result after 7 days.
- **It grows with the ladder.** Milestone 1 has components but no pipeline: its demos record
  single steps (two-view initialisation, PnP, BA) with rerun, and the Rust note compares the
  three languages in a table. The viewer is built in milestone 2 (PR 10), on the first
  end-to-end runs, and your own video follows in its last PR (PR 11). Both are used locally, and
  on the staging deploy, until the repo goes public. Loop closure (milestone 2B) and the dense
  and learned rungs (milestones 5 and 9) add to it.
- **The run format is milestone 2's,** defined with its pipeline: trajectory, keyframes, map
  and per-frame diagnostics, written by the pipeline and read by both rerun and the viewer.
  It lives in `slam_it.eval`, so the algorithm rungs never depend on the viewer.
- **Run outputs stay outside the repo** (`../runs/`), and so do the bundles baked from them
  (`../bundles/`). The page ships only the assets built from them.

## Licensing and publication

- The repo is **private**. It goes public together with reifyscene, as one decision, and only on
  the owner's explicit instruction.
- SLAM It! is under the PolyForm Noncommercial License 1.0.0 (`LICENSE`). Ported code
  (DROID-SLAM BSD-3, MegaSaM, and others) keeps its license and attribution in a
  `THIRD_PARTY_NOTICES.md` file. Model weights are downloaded, never committed. Non-commercial
  licenses (for example VGGT's weights) are acceptable for this hobby project.
- **Renamed before going public (0.3.4.0, then 0.3.7.0).** The working name, slamlab, collided
  with SLAMLab, the University of Zaragoza group behind ORB-SLAM3, and with `slamlab` on PyPI,
  and became SLAM It All! (a pun on "damn it all!") in 0.3.4.0. In 0.3.7.0 it was shortened to
  SLAM It!, easier to remember, and what the demo does to a video: `slam_it` in code, `slam-it`
  for the repository, the distributions, the crates and the conda env. `slam-it`, `slam_it` and
  `slamit` were free on PyPI and crates.io, and `evan-wes/slam-it` on GitHub, when checked on
  2026-10-12.

## Environment

SLAM It! uses a conda environment, matching reifyscene. `scripts/setup_slam_it.sh` builds it
in two layers:

- **conda layer (conda-forge):** Python 3.11 (matching reifyscene's env), cmake, ninja, a C++
  compiler and Rust. Nothing is installed system-wide.
- **pip layer:** installed from `scripts/locks/slam-it.lock`, which
  `scripts/gen_env_lock.sh` regenerates.

Each env builds the C++ extension in its own `build/<env name>` folder. Envs with the same Python
version otherwise share one CMake tree and silently reuse each other's cached compiler. This
matters at milestone 3, when reifyscene's env also installs `slam-it`.

**CI runs offline.** `scripts/offline_ci.sh` does what `.github/workflows/test.yml` does, on
the developer's machine: a clean checkout of the committed files (in
`~/.cache/slam-it-offline-ci/`, cleaned with `git clean -ffdx` each run), merged with
`develop` when it is not based on it, an env of its own, `slam-it-ci`, built from the lock by
`setup_slam_it.sh` (recreated whenever the lock or the setup script changes), then
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
    metres through `slam_it.eval.images` (Pillow); C++'s `slam_it_eval` and Rust's
    `slam-it-eval` read PNGs natively, outside the cores.
  - **Why the refinement rounds stopped (L9, run 2).** On planar scenes the cap of four
    rounds, not stable inliers, ends most runs, and `RefinedSummary` cannot say which
    (`rounds == REFINE_ROUNDS` is either). Add a stop reason (cost, stable, cap, too few
    inliers, refinement failed) when a caller needs it, with the summary-convention's
    termination enum. Since PR 6 the refiner may fail (panel A2), and a failure in round 2 or
    later reads as "no improvement" (PR 6's panel, run 2, A6).
  - Closed by PR 7: **one feature front end (A8).** Both demos match SLAM It!'s ORB features
    (`foundations.orb` and `foundations.matching`) in place of their own SIFT.
- **Deferred from PR 8's review panel**
  ([report](../reviews/2026-10-06-feature-edge-vo.md)):
  - **Re-anchoring the last frame after local BA (L6).** Local BA moves a new keyframe, but
    the pipeline keeps the frame's pose from before it, so the next velocity carries the
    correction; ORB-SLAM re-anchors the last frame to its reference keyframe every frame
    (`UpdateLastFrame`) and saves each frame relative to its BA-refined reference keyframe.
    A run's frame poses are the tracking-time estimates; the edge note scores the final
    keyframes too, ORB-SLAM's protocol, for the like-for-like column.
  - Closed by PR 9: **the keyframe time's growth on long sequences (H19).** Profiled, local BA
    was four fifths of mapping, not the candidates named (`refresh`, the grid). Its problem
    grows with the map: on MH_01 stereo the median local BA holds 6.8k observations in the
    sequence's first quarter and 11k to 12k after, as more fixed keyframes see the window's
    points. Rung 0's C++ and Rust BA now form only the coordinates they solve (the Python
    reference is unchanged; [the real-time note](../notes/real-time.md)), which brought
    mapping's busiest 10 seconds below a thread's budget (MH_01 stereo 1.15 to 0.52,
    fr3/long_office monocular 2.28 to 0.92, in C++).
    Keyframe time still grows along a run; ORB-SLAM's mapping thread bounds it by stopping a
    BA when a keyframe arrives, which this rung's single thread cannot (below).
  - Closed by PR 9: **the real-time items (A11).** Each C++ and Rust frame builds its
    `FeatureGrid` once, when it is assembled, as ORB-SLAM's `Frame` does; Rust's 3x3 SVD runs
    on the stack (PR 6's A11, below).
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
  - Closed by milestone 2A's A2: **ORB-SLAM's reset of a map lost early (run 2, L13).** A map
    lost with `reset_keyframes` (10) keyframes or fewer is reset, as ORB-SLAM3 resets its active
    map, its frames' poses dropped from the trajectory; `min_init_depth_points`, the stand-in,
    is gone, so a depth map starts from any frame of more than 500 keypoints.
  - Closed by milestone 2B's B2: **Relocalising monocular runs that lose the track (run 2,
    L15).** Started one to four frames after the frame the first run initialises from, 6 of 20
    monocular runs on TUM and EuRoC lost the track (3 of 5 on fr1/desk), each deep into the
    run (44 to 128 keyframes), when its inliers fell below ORB-SLAM's 30. ORB-SLAM relocalises
    there with the bag of words (`Tracking::Relocalization`); the lite pipeline ended the run.
    Measured again with B2's covisibility graph
    ([the test-set note](../notes/test-set.md#b2-the-covisibility-graph)): none of the 20 loses
    the track, one frame of them fails, and none needs relocalising.
  - **The Rust extension in layer 1 (run 2, A19).** `scripts/check.sh` rebuilds the Rust
    program before pytest but not the extension `slam_it_rs` (`maturin develop`), so after a
    Rust change the stage-parity tests run the old extension while the whole-run tests run
    the new program. Rebuild it in `check.sh`, or give it the program's stale check
    (`backends.StaleProgramError`). Offline CI builds from clean and is not affected.
  - **The C++ edge tests' layout (H18).** Options' checks live in `test_map.cpp` and the
    residuals' in `test_adjust.cpp`, not in tests mirroring their modules.
- **Found while building PR 9:**
  - Closed by milestone 2B's B3b: **Mapping in a thread of its own (was milestone 7).** The
    edge rung maps after tracking in one thread, so runs are deterministic and the languages
    compare frame by frame; PR 9 judges mapping by its load, as a thread of its own would run
    it. A real mapping thread, and ORB-SLAM's interruption of a local BA when a keyframe
    arrives, were left for the edge targets' budgets, where keyframe bursts would otherwise
    delay frames. *B2 (2026-10-16):* with local BA over every link, mapping's load is 1.7 to 2.6
    on the test set's five largest maps, as no busy thread refuses a keyframe; how the edge rung
    stands in for ORB-SLAM's threads, local mapping's and loop closing's, is decided in B3's
    design (decided with evan-wes on 2026-10-16;
    [the test-set note](../notes/test-set.md#b2-the-covisibility-graph)). *2026-10-17:* real
    threads, a design of our own, in B3b ([the 2B design](2026-10-14-online-slam.md),
    section 4). *B3b (2026-10-18):* tracking, local mapping, loop closing and each global BA run
    in threads of their own, with ORB-SLAM's scheduling (a local BA interrupted when a keyframe
    arrives, keyframes queued or refused while mapping is busy) and its locking as C++'s
    baseline ([the 2B design](2026-10-14-online-slam.md), section 12); a run in order stays the
    default.
- **Found while designing milestone 2B's B3:**
  - **Hand-built parts a standard library could replace** (decided with evan-wes on
    2026-10-17). What rung 0 and the edge rung built by hand before libraries were welcome
    is weighed one part at a time, each against the library the field uses: local BA and rung
    0's least-squares solver (GTSAM, Ceres), the Lie groups (Sophus, GTSAM's), the ORB front
    end and the ban on OpenCV (OpenCV), the vocabulary and its database (DBoW2, fbow). Each
    is a decision of its own, never a rider on another PR.
  - **ORB-SLAM3 itself, once milestone 2B is done** (decided with evan-wes on 2026-10-17).
    Built outside the repository (GPLv3: run, never vendored or linked), run on the loop
    sequences and the test set, several runs each since it is threaded, and scored by our
    evaluation: the whole system's yardstick, and the bar's anchor where ORB-SLAM's papers
    publish no figure.
- **Deferred from milestone 2B's B4** ([the 2B design](2026-10-14-online-slam.md), section 14;
  decided by evan-wes on 2026-10-18):
  - **B4's measurements, not run.** Section 15's measurements were not run for B4's PR (no long
    runs for it): multi-session EuRoC, the bar (MH_01 to MH_03, MH_01 to MH_05 and V1_01 to
    V1_03, monocular and stereo, against ORB-SLAM3's Table V); TUM's fr2_360_kidnap; 7-Scenes
    across sequences (chess, fire, heads and stairs, each scene's sequences in one run, the
    scenes unpacked first, their size checked); the loss window, the test set at 3 s and at 0,
    which section 14's item 6 decides on (B1's 3 s stays the default meanwhile); and the test
    set and the loop sequences against B3b's. The measure takes them
    (`scripts/measure_testset.py`'s `folders`, run in turn as sessions, its merges judged), and
    `slam_it.eval.sevenscenes` imports 7-Scenes. With them, from B4's review panel (L5;
    decided by evan-wes on 2026-10-19): the relocalisation candidates tried per failed frame,
    which grow with the kept maps (each map's 75% cut is relative to its own best), and how
    often the track leaves a healthy map for a kept one on a momentary failure; if either is
    high, an atlas-wide floor, or the active map's candidates tried first.
  - **Left open after B4.** The 2B design's section 14, item 10 (Sim(3) as a group in rung 0;
    B3's review panel, A5 with H3), and the items below meant for when B4 lands: loop closing's
    APIs mirrored unevenly (A14), the `loops` split (A15) and GTSAM's native confinement (A24)
    from B3's panel, the map's owner in a module of its own (A9) from B3b's, and one native
    `.npy` module (A21) from B1's. B4 addresses none of them; A15's and A9's splits are the
    first step of the merge-design follow-up (below).
  - **Threaded map merging without blocking tracking (option C).** B4 merges threaded by pausing
    tracking at a frame boundary for the whole merge (option B; the 2B design's section 14, item
    4; decided by evan-wes on 2026-10-18). The later design is option C, the closest to
    ORB-SLAM3's `MergeLocal`: the heavy solves (the welding BA, the essential graph) with mapping
    stopped while tracking reads the old view, then a short pause at a frame boundary to swap in
    the merged map and remap tracking's state once. Option B's tests are its required
    behaviour: a threaded run of the merge room merges in each language, tracking's state after
    the merge names the merged map's ids, no thread is left
    (`tests/orchestration/test_merge_room_threads.py`), and a threaded merge runs under
    ThreadSanitizer in both of C++'s schemes (`tests/orchestration/test_merge_room_tsan.py`).
- **Deferred from milestone 2B's B4 review panel**
  ([report](../reviews/2026-10-19-feature-2b-b4-merging-maps.md); decided by evan-wes on
  2026-10-19):
  - **Map merging reviewed against the reference systems (the follow-up to B4).** B4's merge
    is option B, accepted for its PR. Its design is reviewed as a whole against ORB-SLAM3
    (`MergeLocal`, its `Atlas` and keyframe database) and the other reference systems, and the
    outcomes weighed side by side: option C, a merge that does not block tracking (above),
    beside an ORB-SLAM3-faithful staged merge. Its first step is the two splits B4 grew past
    their records (A5): `loops`' merging into a `merging` module and the map's owner into an
    `owner` module, in each language (A15 and A9, below). Then, from the panel:
    - **Python's loop and merge detection written twice (H1).** `_detect` and `_detect_merge`,
      `_search` and `_search_merge`, step for step, where the native cores share one driver
      (`detect_region`): one `_detect_in` taking the searched map and the search, and the
      `(score, number, keyframe)` ranking shared with relocalisation's.
    - **The pause's work, the id remap, the kept maps' sharing (H14, with A8's design).** The
      merge's pause re-quantises every absorbed keyframe into the matched map's database
      (`MapOwner.apply_merge`), where moving the vectors with their ids offset would do
      (ORB-SLAM3's database is atlas-wide); each holder of ids remaps them itself, where
      ORB-SLAM3's atlas-unique ids need no remap; and C++ reads the kept maps unlocked, safe by
      option B's protocol alone. One sharing design for the kept maps in all three languages
      (Rust's are shared snapshots since B4's fixes, A2).
    - **One remap API for a merge's result (A4).** `loops.merge` returns two offsets and `G`,
      and about seven consumers per language apply "id + offset, -1 kept" and `G`'s scale
      themselves (`_carried`, `TrackingState.merged`, the anchors, the atlas's records, the two
      `_offset` helpers): `Merged.carry_points`, `carry_keyframes`, `carry_pending` and
      `carry_motion`, or the like.
    - **`LostSummary` names any map closed (B4's A9).** Lost, left for a kept one, or a
      session's: `ClosedSummary` would say what it is.
    - **The essential graph moves points the weld just solved (L3).** A weld point whose
      reference keyframe lies outside the windows is moved again by its reference's correction;
      ORB-SLAM3's graph corrects only the rest of the active map's points.
    - **Threaded merges' global BA and the pause's size (L4).** Threaded, a global BA follows a
      merge only where one was superseded (inline, every merge), and the pause covers the
      essential graph over the whole merged map and the database's resync, so it grows with
      both maps, unmeasured.
    - **A monocular merge's unbounded scale (L6).** A merge at a wrong place with an extreme
      scale rescales a whole map, irreversibly: a wide sanity bound, recorded when it refuses,
      or a check of the links between maps (pairwise consistency, PCM; graduated non-convexity,
      GNC), weighed with section 15's judged merges.
    - **The kept maps' number in Rust's searches (H23).** `kept_maps` returns
      `map_id().unwrap_or(0)` as the tracked map's number, the default H2 removed from `close`
      and `merge`; only relocalisation reads it, so loop closing's search and `merge_maps` could
      take the kept maps alone.
    - **A merge whose matched map is no longer kept, under option C (L13).** With loop closing
      running while tracking switches or merges maps, Rust's `merge_maps` fails with
      `InvalidInput`, ending the run, where it should drop the merge; and C++'s `Atlas::kept`
      returns `MapRef`s pointing into its kept maps, which a change would leave dangling: C++
      should hold its kept maps shared, as Rust does.
    - **From a separate reference review** (a reference review of map merging against ORB-SLAM3
      and other systems, written 2026-10-19 outside the repository; it is committed with the
      follow-up): the keyframes mapped while a merge waits are never searched for a loop or a
      merge; the queue is drained by mapping each keyframe whole, where ORB-SLAM3's `EmptyQueue`
      only inserts them; every merge copies whole maps; and relocalisation in another map can
      drop a merge waiting.
  - **Why one deferred keyframe starves a monocular map (run 3's L14; decided by evan-wes on
    2026-10-21).** Threaded, a monocular frame that wants a keyframe while local mapping is still
    mapping the last one gets none (`NeedNewKeyFrame`: mapping not accepting keyframes, too few
    frames since the last), as ORB-SLAM3 decides. In the revisit room one such deferral, frame 184's
    keyframe taken at frame 186, starved map 1: over frames 197 to 210 its keyframes made 7 to 49
    new points each, where a passing run's made 50 to 111, its tracked points fell from about 250 to
    52, and it was lost at frame 213, where the room run in turn keeps it to the end. This machine's
    efficiency cores map a keyframe about half again as slowly as its performance cores, so such
    deferrals failed the threaded room tests at random: B4's run 3 measured the revisit room's
    monocular case in C++'s own scheme failing 4 of 77 runs over three sessions, the loop room's 2
    of 20 in C++ and 1 of 20 in Rust. The threaded room tests now run their native programs on the
    performance cores (`tests/orchestration/conftest.py`'s `fast_cores`), which closes the two flaky
    cases B3b and B4 recorded, passing 30 of 30 and 25 of 25 runs pinned: the loop room's (B3b's
    review panel, H10 and L3) and the merge room's (its first frame after the merge relocalised).
    The revisit room's case still failed 2 of 60 runs pinned, the same way: on the performance cores
    C++'s own scheme maps frame 183's keyframe in 29.2 to 29.9 ms (14 of 15 runs measured), close to
    frame 184's decision, its two views costing about 3.5 ms a keyframe (an exact 0.3 ms speedup of
    them was left out of B4). The owner accepted this residual for B4's merge (2026-10-21): until
    this item is done, about one layer-1 or offline CI run in 30 may fail on it. The question left is whether a monocular map should survive a deferred
    keyframe: we use ORB-SLAM2's 20 neighbours for monocular triangulation and fusion
    (`mapping.mono_triangulation_neighbours`, `mapping.mono_fuse_neighbours`), where ORB-SLAM3 uses
    30, and keyframe culling (90% of a keyframe's points seen by 3 others) ran on keyframes made
    nearly every frame in the failing runs. Measured on the test set, after B4: threaded on the
    efficiency cores against the performance cores, and with 30 neighbours. In the loop room's worst
    miss B3b's panel examined (15.22 cm), a global BA shrank the map (scale 0.276), a lead that
    stays with B3's damping item (the 2B design's section 11). *Since 0.3.18.0:* culling takes
    ORB-SLAM3's rule, more than 3 others (`mapping.redundant_observers` 4), which rescued the
    revisit room's deferred keyframe inline in all three languages (the investigation of
    2026-10-21), and global BA runs g2o's damping (the 2B design's section 11, L2); the threaded
    rooms' failure rates have not been measured again.
  - **Python's run-id tables rebuilt per call (H9).** `Atlas._runs_of` builds an array from a
    map's run-id list on every call, up to about five a frame (the reviewer measured 2.3 ms for
    100,000 entries, 3.5 ms for 300,000): grow the tables as arrays, or cache one until
    `_extend`. Minor: Python is not held to real time.
- **Deferred from milestone 2B's B3b** ([the 2B design](2026-10-14-online-slam.md), section 12;
  decided by evan-wes on 2026-10-18):
  - **B3b's measurements, not run.** Section 13's measurements were not run for B3b's PR (no long
    runs for it): the latency bar (at least 99% of the frames built and tracked within their
    deadline, C++ and Rust each alone on cores of its own), the accuracy threaded on the loop
    sequences and the test set against B3's runs in order, and our design against ORB-SLAM's
    scheme in C++. The scripts take them: `scripts/measure_testset.py`'s `--threaded`, `--locks`
    and `--budget`, and `scripts/measure_sharded.py`'s `--cores`.
  - **A failed loop correction (the 2B design's section 12, item 14; B3's review panel, H17).**
    Still open, with no decision yet: whether a GTSAM failure inside a loop's correction should
    be recorded and the loop skipped.
- **Deferred from milestone 2B's B3b review panel**
  ([report](../reviews/2026-10-18-feature-2b-b3b-threads.md); decided by evan-wes on
  2026-10-18):
  - **A Python test of a threaded loop (H3).** Python's threaded loop correction, its global
    BA and `follow` after a correction run in no test (the threaded tests' room has no loop):
    one that closes a loop threaded, applies a global BA and leaves no thread, once H11 and A10
    are understood.
  - **Python's threaded loop room (H11, A10).** Through `vo.run`, Python's threaded RGB-D run
    ended at 7.38 cm final ATE against 4.67 cm in order (one run each), above the 7 cm bound
    the natives are held to; in memory with the tests' settings, its threaded runs closed no
    loop (RGB-D 263 of 280 frames tracked, stereo 255, monocular in 10 maps). Find why before
    Python's threaded run is held to the bar.
  - **The map's owner, a module of its own (A9, with H13's file split).** `MapOwner`, its
    `View` and the publishing grow inside the pipeline modules (`pipeline.py` 1,655 lines and
    `pipeline.rs` 2,738 at the review); an `edge` owner module, before B4's merges enter
    through it, would keep the pipeline the composition. Still open after B4 (B4's review
    panel, A5): B4's merges enter through `MapOwner.apply_merge` and `take_merged` inside the
    pipeline modules, now `pipeline.py` 2,325 lines, `pipeline.cpp` 2,084 and `pipeline.rs`
    3,564 (at `5ffcf500`; 1,755, 1,543 and 2,811 at B4's base, `be801e8a`). An `owner` module
    per language (`MapOwner`, `View`) is the first step of the merge-design follow-up, with
    A15's split (B4's review panel, below).
- **Deferred from milestone 2B's B3 review panel**
  ([report](../reviews/2026-10-17-feature-2b-b3-loop-closure.md); decided by evan-wes on
  2026-10-17):
  - **The loop room's depth bounds (L8).** Its final-ATE bounds are 7 cm in RGB-D and 5.5 cm in
    stereo against 2 cm in monocular, and its points' distance to the walls 11 cm against 6.5,
    on exact rendered depth; RGB-D tracking jumps by 3 degrees looking level at a wall
    (`LOOP_PITCH`). Bounds this loose cannot catch a regression on the depth path: find the
    cause, then tighten them.
  - **Guarding against loops at a wrong place (L9).** advio-13's wrong-place loop passed every
    ORB-SLAM3 check, whose temporal consistency is the only guard here. Back ends now reject
    inconsistent loops themselves (Graduated Non-Convexity, Yang et al., RA-L 2020, GTSAM's
    `GncOptimizer`; PCM, Mangelson et al., ICRA 2018), and retrieval has moved to learned global
    descriptors (MegaLoc, Berton and Masone, CVPR 2025): context for the later rungs.
  - **Everything that links rung 0 builds GTSAM (A12).** `slam_it_eval` and `slam-it-eval`,
    every rung-0 test and benchmark, and the Rust crates need GTSAM, and Rust a C++ toolchain; a
    phone target would cross-build it. If that or an eval-only build comes to matter, a default-on
    CMake option and cargo feature around `graphs` keeps one library and one crate per rung:
    weigh it in "Hand-built parts a standard library could replace".
  - **Loop closing's APIs mirrored unevenly (A14).** `essential_graph` is public in Python and
    Rust but internal in C++, and the search settings are `Window` in C++ and `Search` in Rust;
    the Protocol checks only the dispatched functions. Align them when B4 reuses the essential
    graph. Still open after B4.
  - **`loops` split for B4 (A15, H8).** 2,208 lines in Python, 2,284 in Rust and 1,685 in C++
    (at `eedb319`; B3's review panel, run 2, H29), half of which B4's merge reuses (detection's
    stages, the essential graph and global BA: the online-SLAM design's section 14, items 3 and
    4). When B4 lands, split common-region detection, and perhaps the correction's solves, into
    modules of their own, rather than growing `loops` or having merging reach into it; and split
    `correct` (about 115 lines in C++, under a cognitive-complexity NOLINT) into moving,
    replacing and fusing, collecting the new links, and the solves, and
    `slam_it.eval.loops.judge_pairs` (which `judge_loops` calls) into its place and pose
    judgements. Still open after B4, and grown by it (B4's review panel, A5): B4 put merge
    detection, the weld and the merge into `loops`, now 3,498 lines in Python, 2,664 in C++ and
    3,823 in Rust (at `5ffcf500`; 2,413, 1,736 and 2,381 at B4's base, `be801e8a`), and the atlas
    and the pipeline reach into it for Sim(3)'s `scale_of` and `rigid_of`. A `merging` module
    per language (`detect_merge`, `merge_first`, `merge_transform`, `merge`, the weld), a new
    layer in the edge rung's contracts, is the first step of the merge-design follow-up, with
    B3b's owner module (A9) (B4's review panel, below).
  - **GTSAM's native confinement, widened with B4's bridges (run 2, A24).** `check_layers.py`'s
    `check_confined` scans `cpp/` and each crate's `cxx/` alone: a C++ bridge placed elsewhere
    in a crate, or another crate's `build.rs` linking GTSAM, would escape it. Today only
    `slam-it-foundations` has a bridge, its `ffi` module private. If B4 adds a bridge, scan every
    C++ file under `rust/` (but `target/`), or check that only `slam-it-foundations` depends on
    `cxx`. Still open after B4, which adds no bridge: its Huber loss crosses
    `slam-it-foundations`' own.
  - **The GitHub workflow's runner (run 2, H30 with A22).** Since B3 its setup builds GTSAM's
    Python wrapper, whose largest file needs more than 8 GB, uncapped; GitHub's standard Linux
    runner for a private repository may have less memory than that (the reviewers' recollection,
    not checked). Before the next release, run the workflow once by `workflow_dispatch`, or name
    a larger runner in `test.yml`. Offline CI, which gates each PR, runs the wrapper's compile
    in its own 16 GB scope; its first run on B3's head is the check of that scope.
- **Deferred from the readiness PR's review panel**
  ([report](../reviews/2026-10-18-chore-public-readiness.md)):
  - **The `slam-it-rs` wheel's licence files (readiness PR hygiene review, H7).** The wheel
    compiles the MPL-2.0 file `rust/slam-it-foundations/src/linalg/jacobi_svd3.rs`, but its
    metadata ships no `license-files`: no MPL text and no notices. Fix this before that wheel is
    ever distributed (the crates are `publish = false` today).
- **Deferred from milestone 2B's B2 review panel**
  ([report](../reviews/2026-10-16-feature-2b-b2-covisibility.md)):
  - **Every fixture states its oracle and tolerance (A4).** CLAUDE.md asks it of every parity
    fixture; B2's covisibility fixture states both now, but others state one or neither (among
    them `foundations/svd3.json`, `eval/png.json` and most of rung 0's, which give their oracle
    alone). A test over `tests/fixtures/**/*.json` would hold them to it, a contract the import
    and layer checks cannot express.
  - **The graph through time (A7).** The run records each keyframe's last parent and a map's
    strong links when it closed or the run ended, and the viewer draws them through the run
    while both their keyframes live: the graph as each map ended, not as it grew. A life for
    each link (made, removed), as points and keyframes have, would show it growing; the toggle
    says which it is meanwhile.
- **Deferred from milestone 2B's B1 review panel**
  ([report](../reviews/2026-10-16-feature-2b-b1-place-recognition.md)):
  - **One native `.npy` module, with B4 (run 2, A21).** `slam_it_eval` and `slam-it-eval` write
    `.npy` in their run writers and read it in their vocabulary readers. When a second native
    reader arrives (B4's kept maps), give the format one module beside `text` and `png` rather
    than copying the reader. Still open after B4, whose kept maps stay in memory: no native
    reader arrived.
  - **H21's check made per setting (run 2, L14).** `test_the_moved_settings_change_the_decisions`
    checks that `MOVED_OPTIONS` together change the gap room's decisions; reverting each alone,
    11 of its 33 settings change none (among them `stereo_search`, `init_parallax_cos`,
    `min_motion_inliers`, the distance margins and `min_epipole_distance2`), some moved
    decisively elsewhere. B1's relocalisation settings are checked one by one
    (`tests/edge/test_relocalization.py`'s `DECISIVE`); the same would harden H21's.
- **Deferred from milestone 2A's A2 review panel**
  ([report](../reviews/2026-10-14-feature-2a-a2-new-map.md)):
  - Closed by milestone 2B: **an atlas (A4, A9, H7, the rest of A8).** Map numbering, the id
    bases, the lifetimes ledger and the closed maps, the seed of ORB-SLAM3's `Atlas` grown inside
    `Pipeline`, are the edge rung's `atlas` module (`slam_it.edge.atlas`, with C++ and Rust
    twins, from B3b), in the edge rung's import order: it numbers the maps and their ids (a map's
    own ids, with their run ids in a table since B4), dates every life, and owns the final
    trajectory, a reset map's frames left out by it rather than by each run program. Since B4 it
    keeps each map the track leaves whole, with its descriptors, observations and keyframe
    database, which relocalisation in every map and merging search; a kept map is taken up
    again, or merged with the map tracked. B1 closed the rest: `State.LOST` was both a frame's
    outcome and, for an instant, the pipeline's state; the loss is flagged on the frame's step
    now, and the pipeline's state is two-valued.
  - **A run of several maps across the reifyscene boundary, before milestone 3 (A10).** The
    `SlamResult` sketch has every pose in one world; say whether the adapter takes the largest
    map, every map with its id, or waits for 2B to join them.
  - **The scoring out of `runs` (A11).** `eval.runs` holds the format's reading and writing and
    the scoring, over 1,200 lines: `eval.scoring`, above it, for `Segment`, `Tracking`,
    `MapError`, `Evaluation` and `evaluate*`.
  - **The test set's seeds and loss table from the repository (A13).** The note's seeds 1 and 2
    were set by a local wrapper, and its losses sorted by a local script: a `--seed` option and
    the losses' check in `measure_testset.py` would let the note be regenerated from the repo.
    In part by B1: `measure_sharded.py --seeds` sets the seeds, `--option` any setting, and
    `compare_testsets.py` sorts the failures by how they ended, all in the repository.
  - **The map arrays held against one another (L7).** Each is checked alone: that every map of
    `pose_maps` has a keyframe, and that a reset map has no poses, are what `tracking`, the bake
    and `Run.reset_maps` rely on.
  - Closed by milestone 2B's B1: **a monocular start in real time (from A2's measurement).** On
    ADVIO, a failed two-view start took up to 0.43 s, and every late frame of the test set was
    one: the two-view RANSAC drew up to 10,000 samples per model, where ORB-SLAM's
    `TwoViewReconstruction` draws 200. It draws 200 now (`init_ransac_iterations`): no decision
    of the test set's monocular runs moves at seeds 0, 1 and 2, and on the 7 ADVIO runs late in
    A2, made alone, the late frames fall from 108 to 0 (`docs/notes/test-set.md`).
- **Deferred from milestone 2A's A1 review panel**
  ([report](../reviews/2026-10-13-feature-2a-a1-measure-and-import.md),
  [security pass](../reviews/2026-10-13-feature-2a-a1-measure-and-import-security.md)):
  - **Stray Scanner's reader out of the decoder (H6, A7).** `eval.video` holds the decoder and
    Stray Scanner's zip format, its entries, LiDAR maps and CSVs, in one module of over 900
    lines. The zip's reading moves to `eval.stray`, above `video` beside `advio`, when it next
    grows.
  - **Stray Scanner's per-frame intrinsics (L4).** Newer Stray Scanner versions write each
    frame's `fx, fy, cx, cy` in `odometry.csv`, and `camera_matrix.csv` is the last frame's;
    the import reads the camera matrix alone. Focus moves the focal length (ARKitScenes' spans
    about 4% over a capture): take the frames' median, as ARKitScenes' import does.
  - **ADVIO's gyroscope lag, with milestone 6 (L7).** ADVIO's ground truth turns with
    `arkit.csv`, on the frames' clock, at no lag, but with `gyro.csv` only 0.28-0.31 s later on
    the sequences checked, though ADVIO says its clocks are synchronised. Measure and correct
    the lag before ADVIO's IMU is used.
  - **ScanNet++ for the LiDAR path (L8).** ARKitScenes' poses are ARKit's own estimate.
    ScanNet++ (Yeshwanth et al., ICCV 2023) gives an iPhone's RGB and LiDAR with poses
    registered to a laser scan: an independent reference, for a later revision of the test set.
  - **A reference paired one to one (round two's L9, route (a)).** `tum.read_sequence` gives
    each frame its nearest pose, so a reference sparser than the frames hands one pose to
    several, and the RPE then compares frames holding one pose. ARKitScenes' import
    interpolates its reference at the frames' times instead (route (b)); TUM's own
    `associate.py`, and evo, pair one to one. If another sparse reference joins, pair one to one
    at the root (`metrics.associate`), with the RPE at the reference's own spacing.
  - Closed by milestone 2A's A2: **the bake's alignment tied to the RPE (round two's A10).**
    `runs.evaluate` aligns each map by its ATE alone and gives its RPE where it has pairs, so a
    run whose reference has no pair a frame period apart is placed, its RPE null.
  - **One staging helper (round two's A14).** `tum.staged` serves every import; the bake
    (`.baking`) and the lens profiles' fetch (`.fetching`, in `lenses`, below `tum`) each write
    the same mechanism out again. One neutral helper at eval's bottom layer, named by suffix.
- **Deferred from the SLAM It! rename's review panel**
  ([report](../reviews/2026-10-12-chore-rename-slam-it.md)):
  - **The images' names held by a test (A3).** No test reads `docker/`: a setting a Dockerfile or
    the entrypoint names, or a module it runs, could drift from the code's unseen until a
    deploy, and nothing holds the worker's token variable to the server's. A test that checks
    each `SLAM_IT_*` name in `docker/` against one the code reads, imports each `python -m`
    module, and holds `worker.TOKEN_VARIABLE` to the server's settings.
  - **The programs' summary lines alike (L2).** Python's prints the final map's keyframes and
    points, C++'s and Rust's every one the run made, so the backends seem to disagree where
    their runs agree: all three print the same, or both.
- **Deferred from PR 11's review panel**
  ([report](../reviews/2026-10-11-feature-your-own-video.md),
  [security pass](../reviews/2026-10-11-feature-your-own-video-security.md)):
  - **The worker's protocol owned by the server (A8).** The server points at the worker's
    docstring for its API, keys its states to the `vo` worker's stages, and has no protocol
    version. When a second kind of job is planned (reifyscene's GPU worker), the protocol
    moves into the run-viewer design's section 5.3 with the server as its owner, with stages
    by kind, or "upload consumed" as a message of its own.
  - **A Kannala-Brandt camera in rung 0, with TUM-VI (A9).** `eval.lenses.Fisheye` is a full
    fisheye model outside rung 0, and `Pinhole` a fourth camera with a size. TUM-VI
    (milestone 6) is calibrated in Kalibr's equidistant (Kannala-Brandt) model: it joins
    `foundations.camera` in all three languages then, and `lenses` uses it; `Pinhole` folds
    into the camera lift (PR 8's A8).
  - **Before going public** (the security pass): the result's token out of every path
    (`/r#<token>`, the status page sending it in a header), so that no host's log holds it
    (S2); the fonts self-hosted and HSTS on the domain (S11); a token per worker (S15); the
    lock's hashes required in the images, and PyAV bumped for FFmpeg's fixes (S20).
  - **To the real clips** (with the note's list): profiles calibrated at a 16-row-padded size
    stretched rather than cropped (L6); a mirror in the display matrix read as a half turn
    (L7); the nominal field of view by aspect (L8).
- **Deferred from PR 10's review panel**
  ([report](../reviews/2026-10-10-feature-run-viewer.md)):
  - **Removals out of the core (A6).** The pipeline keeps every point and keyframe a run
    removes until the run ends, for their lives: up to 26 MiB more peak memory on the edge
    note's runs (ORB-SLAM2, too, keeps culled points, flagged bad). Each frame's result could
    carry its removals (ids, last positions or poses, why) and its new ids, for orchestration's
    recorder to keep or stream as it does poses and diagnostics, so that the core's state is
    bounded by its map: before a live camera or any long run.
  - **The pipeline in the run and bundle formats (A9).** `run.json` and `bundle.json` name the
    backend, not the pipeline or rung, and `FATES`, `REMOVALS`, `record_features` and the
    page's stages are the edge rung's. Rungs side by side need a pipeline field and entries by
    rung and backend: a version bump when it comes.
  - **The BA solve record in rung 0 (A10).** `edge.adjust.Solve` and its ports project rung
    0's `nls.Summary` for a run's records; when a second rung records its solves (the classic
    rung's global BA), the projection moves to `foundations.nls`.
  - **Motion-model outliers searched again (L7).** The local map's search projects and matches
    the points the motion model's refinement just rejected, skipping only its inliers, and
    counts them visible; ORB-SLAM2 marks each such outlier (`mnLastFrameSeen`) so that
    `SearchLocalPoints` skips it. On the room, 193 of 363 RGB-D motion outliers were matched
    again and 1 became an inlier. It inflates `local_map.matches` and lowers the found ratio
    `MapPointCulling` reads; the fix moves decisions, so it is a PR of its own.
- **Deferred from PR 9's review panel**
  ([report](../reviews/2026-10-08-feature-real-time.md)):
  - Closed by milestone 2B's B3b: **A mapping thread's backlog (L8).** `judge` measures mapping
    by its load over the run and over its busiest window; the lag a mapping thread of its own
    would build up is the waiting time of a FIFO queue, Lindley's recursion
    (`W' = max(0, W + S - A)`, keyframe arrivals A, mapping times S), an upper bound where
    ORB-SLAM also holds back keyframes and stops its local BA. *B3b:* `budgets.backlog` gives it
    for runs made in order; a threaded run's mapping is judged as it ran, by the keyframes asked
    for and refused, the largest queue and the local BAs aborted.
  - Closed by milestone 2B's B3b: **Physical cores for a budget (L12).** A budget's cores were
    logical CPUs: this machine has no SMT, but on one that has, the four highest-numbered CPUs
    can be two physical cores. *B3b:* `budgets.cpus` pins one logical CPU of each physical core,
    read from `topology/thread_siblings_list` (`budgets.physical`).
  - **The anchored point's own observation (L10).** A host's observation of its own anchored
    point is analytically null (the point is on the host's ray whatever the host pose,
    intrinsics or inverse depth), but the BA's pairwise products over its repeated columns leave
    roundoff of about eps times |J|^2 where an exact zero belongs. Merging repeated columns at
    layout time, summing the Jacobian's columns before the products, would cancel it and cost
    fewer products. Harmless today (the real-time note's per-solve check).
- **Deferred from PR 7's review panel**
  ([report](../reviews/2026-10-05-feature-front-end.md)):
  - Closed by PR 9: **reusing the extraction's pyramid (A1).** The stereo frame builder builds
    each image's pyramid once (`image.pyramid`), extracts from it (`orb.extract_levels`) and
    matches on it (`stereo.match` takes the two pyramids), as ORB-SLAM's
    `ComputeStereoMatches` reads its extractor's `mvImagePyramid`. Features holding their
    pyramid, the other way out, would have kept every keyframe's pyramid alive in the map.
  - Closed by PR 8: **the run format's golden folder and its native writers (A9).** C++ and
    Rust write runs held to a golden folder in `tests/fixtures/eval/run/` that `read_run`
    validates, with hand-written `.npy` and JSON writers; the native pipeline links both the
    edge rung and `eval`, so it sits above `eval`, in orchestration (`slam_it_vo`, in C++ and
    in Rust; see the Package-layout table).
  - Closed by PR 8: **guided matching beside `matching` (A10).** `matching.features_in_area`
    and `matching.search_by_projection` (ORB-SLAM's `GetFeaturesInArea` and
    `SearchByProjection`) are in `foundations.matching`, for the classic rung too; the C++ and
    Rust cores search a feature grid that finds exactly what Python's brute force finds. The
    full-image-to-level map (run 2, A17) is `image.level_coordinate`, beside
    `full_image_coordinate` in all three languages, and stereo's level-to-level column is
    stated in its terms.
  - **The rotation histogram's bin width (L8).** ORB-SLAM's `ORBmatcher` bins orientation
    changes at 30 degrees (`round(rot / 30)`, bins 0 to 12), 2.5 times wider than SLAM It!'s
    12-degree orientation bins; on six TUM pairs the recall of true matches was 0.90-1.00
    against 0.91-1.00. Revisit if PR 8's tracking loses inliers under fast rotation.
  - **Shared Rust fixture helpers (H8).** Four `slam-it-foundations` test modules each parse
    the shared fixtures with their own copies of `count`, `list` and `picture`; a
    `#[cfg(test)]` fixture module would hold them once.
  - **One "fastest built backend" choice (H18/A13).** The demos each pick a backend with
    their own code; `slam_it.backends` could offer it once. A third copy, the first in the
    library, is `eval.video`'s for undistortion, in another order (PR 11's panel, A10).
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
  - Closed by PR 9: **a stack-only 3x3 SVD (A11, run 2).** `linalg::svd3` and
    `singular_values3` port Eigen's `JacobiSVD` for a square matrix (two-sided Jacobi, not
    one-sided as this entry had it), fixed-size and without a workspace; they give Eigen's
    factors to the bit, so Rust's 3x3 SVDs now match C++'s exactly.
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
    first. *2026-10-17:* B3's essential graph and global BA are GTSAM's, which factors
    sparsely; local BA keeps the dense solver, and whether it moves to GTSAM is the item on
    hand-built parts (found while designing milestone 2B's B3).
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
    The fix is a public `slam_it/foundations/ba/solve.hpp` mapped to the `solve` layer.
  - **The gauge in rung 0's own right (A26, run 3).** `ba/gauge` depends only on poses and
    points. When pose-graph optimisation (milestone 2B's loop closure) needs the same
    null-space removal, lift it to `foundations.gauge` rather than importing it from `ba`.
    *2026-10-17:* B3's pose graph is GTSAM's, which holds a keyframe fixed instead, so this
    waits for another user.
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
- **GPU test runs:** their opt-in script and the light edge environment arrive with the first
  PyTorch rung (milestone 5). The budget runs have theirs since PR 9
  (`python -m pytest -m budget`).
- **reifyscene's env** needs SLAM It!'s build tools (cmake, compiler, Rust) when the adapter
  lands at milestone 3.
