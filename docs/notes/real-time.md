# Real time on a laptop-class CPU: the budget harness, and where the time went

Design note for `slamlab.eval.budgets`, the per-step times in the run format, Python's
program (`python -m slamlab.orchestration.vo`), and the speed-ups in rung 0 (`foundations.image`,
`foundations.orb`, `foundations.stereo`, `foundations.ba`, Rust's 3x3 SVD) and the edge rung's
frames. The plan was the ladder's PR 9 row; its three rules were decided with the owner on
2026-10-07. This note is what was built, where the time went, and how the C++ and Rust
pipelines came to run every sequence of the edge note's set (five TUM and EuRoC sequences, ten
runs of a sequence in a mode) in real time.

## The problem

Milestone 2 asks for sparse VO in real time under a laptop-CPU budget. PR 8's pipeline had no
way to say whether it was: a run recorded one time per frame, tracking and mapping together,
and its note found that building the frames, not the pipeline, set the frame rate (29 to 95
ms a frame in C++, 37 to 132 ms in Rust, reading the images included). PR 9 builds the
measurement, then makes the native pipelines fit it.

## Real time, as ORB-SLAM's threads divide it

ORB-SLAM gets real time from three threads: tracking takes every frame, local mapping takes
the keyframes in a thread of its own (dropping its bundle adjustment when a new keyframe
arrives), and loop closing the third. Its published real-time figure is the tracking thread's
time per frame; mapping need only keep up on average. Its examples read each image from disk
before the timed call, and extract its features inside it. The edge rung keeps one thread on
purpose, so a run is deterministic and the three languages agree frame by frame. Three rules
follow (decided 2026-10-07):

1. **The deadline holds building and tracking a frame**, the work of ORB-SLAM's tracking
   thread, against the sequence's frame period (the median spacing of its frames: 32 ms on
   TUM, nominally 30 Hz, and 50 ms on EuRoC's 20 Hz).
   Mapping, which runs after tracking in the same thread, is judged as a thread of its own
   would be: by its **load**, its total time over the sequence's, which must stay below 1. A
   second thread for mapping would make runs depend on thread timing, and the three languages
   could no longer be compared frame by frame; it is milestone 7's question. Mapping here
   includes making the keyframe and, with depth, its close points, which ORB-SLAM2's tracking
   thread does (`Tracking::CreateNewKeyFrame`) before it hands the keyframe over, so
   tracking's latency here leaves that step out.
2. **Threads inside frame building only if needed.** ORB-SLAM2 extracts a stereo pair's two
   images in two threads. Single-threaded fixes came first, and were enough.
3. **The clock starts with the frame's images decoded**, as a camera delivers them. Reading
   and decoding them (and rectifying a stereo pair, as ORB-SLAM2's examples do before the
   call) is reported, not held to the deadline.

## The harness

Each frame's diagnostics now time its four steps (`slamlab.orchestration.vo`):
`read_seconds` (delivering the images), `build_seconds` (the pyramid, the features, the stereo
matches or the depths), `track_seconds` (initialisation included) and `map_seconds` (0 but on
keyframes). The sources deliver decoded images and one builder makes the frame, so each step is
timed in one place per language.

`slamlab.eval.budgets` simulates a budget on this machine, as the ladder's "Compute budgets"
plans: there is no edge hardware.

- **`laptop-cpu`**: 4 cores and 8 GiB. The cores are performance cores: this machine's CPU
  (an Intel Core Ultra 9 285) has 8 of them and 16 efficiency cores, and a budget's cores are
  the highest-numbered performance cores this process may run on (4 to 7 here), which leaves
  CPU 0, where the system's interrupts land, alone. A machine with fewer is refused, not run
  on slow cores.
- **The cap**: `systemd-run --user --scope -p MemoryMax=... -p MemorySwapMax=0`, then
  `taskset --cpu-list`. Without swap, going over the cap kills the program rather than slowing
  it. (The user session here has no cpuset controller, so `taskset` pins, not `AllowedCPUs`.)
- **Peak memory** is the program's own: each program reads Linux's `VmHWM` from
  `/proc/self/status` before it writes its run, into `run.json`'s `peak_memory` (the run format's
  version 2), with what it ran within: its `cpus` (its affinity, `Cpus_allowed_list`) and its
  `memory_cap` (its control group's `memory.max`), so a run folder says whether, and how, it
  was budgeted. Measuring from outside does not work: `getrusage`'s `ru_maxrss` folds the parent's
  peak into the child at `exec`, so `true`, spawned from a bare Python, reads 12 MB, and from a
  test process a hundred megabytes or more. `VmHWM` starts again at `exec`.
- **Python's program**: `python -m slamlab.orchestration.vo`, with the native programs'
  command line, exit codes and messages, so a budget runs all three backends the same way.
  Python's in-process run records no peak memory: its process holds more than the run.
- **The judgement** (`budgets.judge`): the deadline (the median spacing of the frames' times),
  the median, 95th percentile and maximum of building plus tracking, the frames past the
  deadline, reading's spread apart, and mapping's time per keyframe, its load over the run and
  its load over its busiest 10 seconds (`mapping_peak_load`; the whole run's, on a run shorter
  than that). It refuses a run it cannot judge: times that do not advance, a time or step time
  that is not a number, negative or not finite.

`scripts/run_vo.py --budget laptop-cpu` runs a sequence within the budget and prints the
judgement; the budgeted tests (`python -m pytest -m budget`, outside layer 1) run the room
through every backend within `laptop-cpu`, each making the decisions it makes unbudgeted, and
kill a program over a 16 MiB budget.

## Where the time went

On each sequence's first 300 frames, within `laptop-cpu`, before PR 9's speed-ups (medians,
ms):

| | Deadline | C++ read | C++ build | C++ track | Rust read | Rust build | Rust track |
|---|---|---|---|---|---|---|---|
| TUM fr1/xyz, RGB-D | 32 | 6.7 | 23.8 | 1.3 | 3.3 | 37.1 | 1.0 |
| TUM fr1/xyz, monocular | 32 | 4.3 | 24.0 | 1.0 | 1.7 | 36.4 | 0.7 |
| EuRoC MH_01, monocular | 50 | 1.1 | 35.2 | 0.8 | 0.7 | 58.1 | 0.7 |
| EuRoC MH_01, stereo | 50 | 6.5 | 70.7 | 0.9 | 7.9 | 112.3 | 0.6 |

Building the frame was 95% of the latency or more; tracking took a millisecond. EuRoC stereo
missed its deadline in both languages, and Rust missed TUM's. Inside building: ORB extraction
(the pyramid's resize, FAST at each level, the Gaussian blur before the descriptors), and for
stereo, the matcher rebuilding both images' pyramids that extraction had just built.

Mapping was the other cost: on the long sequences, its load stayed below 1 over the whole run
(0.50 on MH_01 stereo, 0.84 on fr3/long_office monocular, in C++), but rose above it in the
busiest 10 seconds (1.15 and 2.28), where a mapping thread of its own would fall behind.
Profiled, local BA was about four fifths of mapping, half of it forming the normal equations
and half eliminating the points.

## Frame building: four exact speed-ups

Each of these leaves every pixel, keypoint and descriptor as it was, so every run before and
after them is the same to the bit.

- **FAST** (`orb`): the segment test comes first, for a whole row at once, as OpenCV's SSE2
  FAST makes it: the circle walked once round plus an arc (so a run that wraps past the start
  is counted whole), each centre's current run of brighter and of darker pixels counted in
  byte-wide, branch-free loops. A pixel scores at least the floor
  exactly when its longest run reaches nine, so only those (about one in nine) are scored,
  all 16 arcs' extremes at once in 16-bit lanes, and scores are kept in 16 bits. Detection on
  a 640x480 frame: 14.2 to 9.1 ms in C++.
- **The descriptors' blur** (`image`): rows a tap at a time, so they vectorise; the horizontal
  sums in 32 bits, and the vertical ones in 32 bits whenever 255 · 2^(2 bits) fits (ORB's
  12-bit kernel), 64 above. 8.1 to 1.6 ms a frame in C++. The review panel found the C++
  interior pass forming a span past the row's end when the kernel is wider than the image (the
  fixture's one-pixel case): it read nothing, but a span's precondition was broken, which a
  release build runs silently. It is guarded as Rust's was, and the native tests now run with
  libstdc++'s assertions, which catch it (the benchmarks build apart, unhardened).
- **The resize** (`image`): the pyramid's bilinear resize in 32-bit sums, four times as fast.
- **One pyramid per image (PR 7's A1)**: `orb.extract_levels` extracts from a pyramid the
  caller built (`image.as_pyramid`), and `stereo.match` takes both images' pyramids, as
  ORB-SLAM's `ComputeStereoMatches` reads its extractor's `mvImagePyramid`.

The first two halved C++'s frame building and cut Rust's by three quarters (Rust's FAST and
blur had not vectorised at all); the pyramid changes took a further quarter off C++'s stereo
frames, and an eighth off Rust's.

## Mapping: local BA over the coordinates it solves

Rung 0's bundle adjustment formed its camera-side normal equations over every camera-side
coordinate of the problem: every pose, every intrinsic, every depth affine, fixed or not. A
local BA has ten free keyframes and tens of fixed ones (medians of 23 to 72 along the long
sequences), so most of the matrix was built, then dropped by the reduction to the free
coordinates. Each residual block also kept
its own coupling to its point, merged per point before elimination.

The rewrite, in C++ and Rust:

- **Only the kept coordinates** (the free ones and the gauge-confined poses' rows) are ever
  formed: the camera-side Hessian, gradient and coupling are over them, and the reduction maps
  from them (`Reduction::kept`, `compacted`).
- **A layout fixed per problem** (`CouplingLayout`): each block's columns among the kept
  coordinates, and each point's coupling rows (its distinct kept coordinates, ascending), found
  once; every linearisation then adds each block straight into its point's rows.
- **Fixed-size blocks**: a block's rows are fixed by its kind (2 for a reprojection, 1 for a
  depth prior, 6 for a pose prior), so its Jacobian lives on the stack (C++ bounded Eigen
  types, Rust `SMatrix` per kind) and its small products unroll.
- **Elimination and back-substitution over the layout**, one product per point.

The sums now run in a different order, so the results are no longer the same to the bit as
before. Eigen orders some sums by a matrix's size, so the old bits were no more correct than
the new; keeping them would have blocked the restructuring. The owner chose to relax
bit-identity here, with the verification below.

## Two smaller items (PR 8's A11)

- **Each frame builds its feature grid once** (`edge.frames`, C++ and Rust): the grid
  `search_by_projection` reads is built when the frame is assembled, as ORB-SLAM's `Frame`
  assigns its features to a grid, not on every search (about 16 µs a frame saved). Python's
  search needs no grid.
- **Rust's 3x3 SVD on the stack** (`linalg::svd3`, `singular_values3`): two-sided Jacobi, a
  port of Eigen's `JacobiSVD` for a square matrix (2x2 SVDs swept over the off-diagonal pairs
  to 2ε of the largest diagonal entry, signs moved into U, values sorted), where faer
  allocated. It is the algorithm C++ runs, and gives C++'s bits: on 39,005 test matrices
  (random, graded, rotations, rank 1 and 2, essential-like, zero, scaled to 1e±300) its U, σ
  and V equal Eigen's exactly. 0.5 µs a call, against faer's 1 µs. A golden fixture of 20 hard
  matrices, written by Eigen itself (`foundations_test_svd3 --regenerate`), holds both: the C++
  test fails if Eigen's factors move, the Rust one if the port's do.

## Verifying what is no longer the same to the bit

Three checks, chosen with the owner:

1. **Per solve.** Every local BA of three whole runs and a short one (MH_01 stereo, V1_01
   stereo, fr3/long_office monocular, fr1/desk RGB-D: 3,136 problems) was saved from the old
   code with its solution, then solved by the new. (The problems and the comparison programs
   are kept outside the repository; a point's difference is relative to the larger of 1 and
   its size, so absolute below a metre.) In both languages, every solve takes the
   same iterations and ends the same way, and no residual crosses the chi-square threshold
   either way. Poses agree to 1.7e-14 and points to 1.5e-9 relative in C++, and to 1.5e-14
   and 3.2e-10 in Rust; old Rust and old C++ differed by as much (1.4e-14 and 2.3e-9), and new
   Rust lies as close to old C++ (2.0e-9). The solves take 31 s in place of 79 s in C++, and
   30 s in place of 114 s in Rust.
2. **Parity.** The parity fixtures and the room's golden record pass unchanged in all three
   languages.
3. **Whole runs.** All 100 of the edge note's whole runs (ten sequences and modes, five
   starts each, in C++ and in Rust) take the decisions PR 8's C++ took on the same start, on
   every frame: the same frames tracked, keyframes, points and lost runs, so the same ATE and
   the same five-start medians as the edge note's table, in both languages (in PR 8, Rust ran
   the first start only). The first 300 frames of each sequence take the same decisions in
   Python, C++ and Rust. Poses differ from PR 8's by at most 7e-9 m, but for one run:
   fr1/xyz monocular started from frame 3, one of that sequence's two poor starts (5.7 cm
   against 1.1), differs by up to 1e-5 m from its first frame on, and its C++ and Rust runs
   differ from each other by as much. Its two-view start is nearly degenerate, and its BA's
   flat directions take up the roundoff; its decisions and ATE are PR 8's.

The changes that kept every bit (frame building, the grid) and the ones that did not (local
BA, Rust's SVD) end in the same place: no decision on any frame of any run moved.

## Results

Every whole sequence, each native program within `laptop-cpu`, one run at a time, with
ORB-SLAM2's settings (as the edge note's results): building and tracking a frame, its median,
95th percentile and maximum, against the deadline; mapping's load over the run; the program's
peak memory.

| Sequence | Mode | Deadline | C++ | Rust | Load, C++ / Rust | Peak, C++ / Rust |
|---|---|---|---|---|---|---|
| fr1/xyz | RGB-D | 32 ms | 10.3 / 11.4 / 12.3 ms | 9.5 / 10.0 / 11.0 ms | 0.02 / 0.01 | 21 / 22 MiB |
| fr1/desk | RGB-D | 32 ms | 9.7 / 11.2 / 12.4 ms | 9.2 / 9.8 / 10.8 ms | 0.03 / 0.02 | 26 / 24 MiB |
| fr3/long_office | RGB-D | 32 ms | 9.2 / 10.2 / 11.2 ms | 8.9 / 9.4 / 10.6 ms | 0.01 / 0.01 | 32 / 33 MiB |
| fr1/xyz | monocular | 32 ms | 9.6 / 10.5 / 14.4 ms | 9.1 / 9.6 / 15.6 ms | 0.06 / 0.04 | 23 / 23 MiB |
| fr1/desk | monocular | 32 ms | 9.0 / 11.2 / 12.9 ms | 8.7 / 12.5 / 14.9 ms | 0.12 / 0.08 | 29 / 29 MiB |
| fr3/long_office | monocular | 32 ms | 8.9 / 9.8 / 12.8 ms | 8.8 / 9.2 / 15.5 ms | 0.37 / 0.33 | 63 / 63 MiB |
| MH_01 | stereo | 50 ms | 25.8 / 30.1 / 33.0 ms | 23.4 / 25.1 / 26.5 ms | 0.22 / 0.18 | 133 / 125 MiB |
| V1_01 | stereo | 50 ms | 21.2 / 24.8 / 29.0 ms | 22.2 / 23.7 / 25.0 ms | 0.03 / 0.02 | 56 / 53 MiB |
| MH_01 | monocular | 50 ms | 12.8 / 14.7 / 19.1 ms | 10.9 / 11.7 / 18.3 ms | 0.04 / 0.03 | 45 / 45 MiB |
| V1_01 | monocular | 50 ms | 10.2 / 12.6 / 14.6 ms | 9.9 / 11.0 / 16.4 ms | 0.02 / 0.01 | 32 / 30 MiB |

No frame of any run is late; the slowest, 33 ms, is two thirds of EuRoC's deadline, and TUM's
slowest, 15.6 ms, half of TUM's. Reading a frame, outside the deadline, takes a median of 0.6
to 7.8 ms more. Before PR 9 the same work took 23 to 25 ms a TUM frame in C++ and 34 to 38 in
Rust, and 57 to 72 and 83 to 113 ms a EuRoC stereo frame, past its deadline on every frame.
Rust now matches C++ or beats it, where it took half as long again: its FAST and blur
vectorise now, and its pipeline was already a fifth faster.

Python, on the first 100 frames of fr1/xyz, builds a frame in 140 ms and maps a keyframe in
1.9 s (RGB-D; 5.2 s monocular), and 0.5 s and 8.1 s for MH_01 stereo; it is the reference
for the rules, measured and not held to real time.

## Mapping's load

| Sequence | Mode | Load before / after, C++ | Rust | Busiest 10 s before / after, C++ | Rust |
|---|---|---|---|---|---|
| MH_01 | stereo | 0.50 / 0.22 | 0.43 / 0.18 | 1.15 / 0.52 | 1.02 / 0.45 |
| fr3/long_office | monocular | 0.84 / 0.37 | 0.75 / 0.33 | 2.28 / 0.92 | 2.08 / 0.81 |
| fr1/desk | monocular | 0.23 / 0.12 | 0.18 / 0.08 | 0.42 / 0.21 | 0.32 / 0.14 |

Over a whole run, mapping kept up before as well; in its busiest 10 seconds it did not, on the
two long sequences with the most keyframes (716 on MH_01 stereo, 682 on fr3/long_office
monocular). Now every run's busiest window stays below 1, the closest fr3/long_office
monocular's in C++ (0.92). A keyframe still costs more later in a run (MH_01 stereo's median,
by quarter of the sequence: 39, 55, 66 and 68 ms in C++), because local BA's problem grows
with the map: its median holds 6.8k observations in the first quarter and 11k to 12k after,
as more fixed keyframes see the window's points.

## What it does not do yet

- **Mapping in a thread of its own.** Mapping runs after tracking in the one thread, so the
  frame after a keyframe waits for it. Judged by its load, as rule 1 has it, mapping keeps up;
  in wall-clock time, a keyframe and its mapping together overrun the frame period on MH_01
  stereo (a 26 ms frame, then a 59 ms median mapping). A mapping thread, and ORB-SLAM's
  interruption of a local BA when a keyframe arrives, are milestone 7's, with the edge
  targets.
- **Real hardware.** `laptop-cpu` is four of this desktop's performance cores, pinned, not a
  laptop: the same microarchitecture, its caches and memory bandwidth shared with the rest of
  the machine, and no thermal throttling. The edge budgets (milestone 7) add ARM targets under
  simulation, built and run on QEMU but never timed there.
- **Reading the images** is outside the deadline: decoding the PNGs takes up to 7.8 ms a frame
  (a stereo pair, rectified), as ORB-SLAM's examples read theirs before the timed call.
- **Python** is the reference, measured and not held to real time.
- **Sampling profiles.** `perf` is unavailable on this machine (`kernel.perf_event_paranoid`
  is 4), so the profiles here come from the per-step timers and timed copies of the stages.

## What was learned

- **Measure the steps before speeding any of them up.** Tracking, the part PR 8 tuned, was a
  millisecond; building the frame was the latency, and inside mapping, local BA. The per-step
  times in every run, not a profiler, found both.
- **Exact and fast are compatible in fixed point.** The integer image operations kept every
  bit through a halving of their time: the speed-ups were in how the sums are laid out for
  the compiler (byte-wide runs, taps one at a time, sums only as wide as their bits need), not
  in the arithmetic.
- **In floating point, bit-identity can be the obstacle.** Eigen orders some sums by a
  matrix's size, so the old BA's bits were an accident of its layout, and keeping them ruled
  out the restructuring that made it two to four times as fast. Verifying the new code
  against the old per solve, by parity and on whole runs said what bit-identity had said:
  every whole run still takes the same decisions on every frame.
- **Port the algorithm, not just its contract.** Rust's SVD now gives C++'s factors to the bit
  because it is Eigen's Jacobi rotation for rotation; two backward-stable SVDs would have
  agreed only to roundoff. What is built from the factors still agrees only to an ulp
  (nalgebra and Eigen round a 3x3 product differently: `project_to_so3` differs in the last
  bit on about half of random matrices), so the decisions that hang on roundoff move one step
  further from the SVD, not away.
- **A process's peak memory is its own to report.** `ru_maxrss` of a child carries its
  parent's peak across `exec`; `VmHWM`, read by the program itself, does not.
