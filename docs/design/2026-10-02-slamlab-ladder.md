# slamlab: a learning ladder for visual SLAM

Status: draft for review, 2026-10-02. Scoped in conversation; nothing here is built yet.

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
| **0: Foundations** | Camera models, SE(3)/Sim(3), 8/5-point, RANSAC, triangulation, PnP, Gauss-Newton/LM bundle adjustment with a Schur complement | CPU |
| **Edge: sparse VO** | Features, tracking, keyframes, local BA (ORB-SLAM-lite); optional RGB-D mode with predicted depth | Real-time under an edge budget |
| **Visual-inertial** | IMU preintegration and tight visual-inertial optimisation, on top of the edge VO | Edge budget |
| **Classic heavy** | Incremental SfM, global BA, place recognition, loop closure | Offline, minutes per clip |
| **Dense BA + depth prior** | Dense bundle adjustment over per-pixel depth, with a monocular depth prior and motion masks | GPU |
| **Learned dense** | DROID-style: correlation volumes, recurrent flow update, dense-BA layer | GPU, around real time |
| **Heaviest** | MegaSaM-style: uncertainty-weighted dense BA, focal-length optimisation, robust to dynamic scenes | Large GPU, offline |

## Languages

| Language | Role |
|---|---|
| **Python** | The readable reference implementation of every algorithm; orchestration, evaluation, and the learned rungs (PyTorch) |
| **C++** | The performance core: rung 0, the edge rung, the edge budgets, and dense-BA kernels if profiling calls for them. Showing C++ is a goal in its own right. |
| **Rust** | A chosen subset: rung 0 and the edge VO. Kept parallel and off the critical path, so it never blocks a rung. |

**Parity.** Shared fixtures (fixed inputs, expected outputs) live in `tests/fixtures/`. Every C++
and Rust implementation is checked against the Python reference. All three are benchmarked under
the same compute budgets, which gives a "same algorithm, three languages" comparison of speed,
memory and lines of code.

**Packaging.** There are three importable parts. The two compiled ones are optional;
`slamlab.backends` picks an implementation at runtime and reports which are built.

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
| 6 | Visual-inertial, branching off milestone 2 | Py, C++ | EuRoC, TUM-VI |
| 7 | Edge budgets: Pi-class and Jetson-class runs, ARM build check | C++ (Rust) | Same benchmarks, budget-capped |
| 8 | Train the DROID-style network ourselves | Py (PyTorch) | TartanAir subset; compare against DROID's weights |
| 9 | Heaviest rung: MegaSaM-style | Py (PyTorch) | Sintel, Bonn RGB-D Dynamic |

**Why milestone 3 sits where it does.** reifyscene's old `pose_refine` was a depth-blind BA with no
motion masks. Swapping its new poses in while keeping the old depth produced the "shadow" behind
moving people. Milestone 3 is the corrected version of that same idea: rung 0's solver, with a
depth prior and motion masks. It reuses what milestones 1 and 2 teach and needs only a small
adapter. Its target is the gap between plain VGGT and the old bundle-adjusted cameras on lab-coat
(27.42 vs 29.04 dB masked PSNR).

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

## Datasets

The data lives outside the repo, in `../datasets/` (that is, `../datasets`).
Disk space is checked before each download.

| Dataset | Tests | Arrives with |
|---|---|---|
| TUM RGB-D | Handheld indoor, RGB-D, GT poses, a dynamic subset | Milestone 2 |
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
- **Linear algebra in rung 0:** write our own small matrix types, or use Eigen (C++) and
  nalgebra (Rust). This is a learning choice, made at the start of milestone 1.
- **reifyscene's env** needs slamlab's build tools (cmake, compiler, Rust) when the adapter lands
  at milestone 3.
