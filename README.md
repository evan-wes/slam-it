# SLAM It!

A learning ladder for visual SLAM, from first-principles geometry to learned dense SLAM,
implemented in Python, C++ and Rust and measured across compute budgets.

*A codebase for a SLAM pipeline in three languages.*

Status: milestones 1 and 2 are complete. Rung 0, the foundations, has the geometry, bundle
adjustment, two-view geometry and absolute pose in Python, C++ and Rust. On it, the edge rung's
visual odometry runs monocular, stereo and RGB-D in all three, in real time in C++ and Rust; the
run viewer replays a run frame by frame; and your own phone video can be uploaded, run and
viewed.

The design, the ladder and the milestones are in
[`docs/design/2026-10-02-slam-it-ladder.md`](docs/design/2026-10-02-slam-it-ladder.md), and
each component's note in [`docs/notes/`](docs/notes/).

## Layout

| Path | What |
|---|---|
| `src/slam_it/` | Python: reference implementations, orchestration, evaluation |
| `cpp/` | C++ core, one library per rung (`slam_it_foundations`), and the nanobind module `slam_it._cpp` (`cpp/bindings/`) |
| `rust/` | Cargo workspace: one crate per rung (`slam-it-foundations`, standalone) and `slam-it-py` (the `slam_it_rs` module) |
| `tests/` | Python tests, including parity tests against `tests/fixtures/` |
| `scripts/` | Environment setup and lockfile, `check.sh` (layer 1), `check_layers.py` (C++/Rust layering) |
| `viewer/` | The run viewer's page, and the upload and status pages (JavaScript, no build step) |
| `server/` | The upload server, `slam_it_server`: the job list, storage and guards |
| `docker/` | The server's and the worker's images |

## Setup

```bash
scripts/setup_slam_it.sh  # conda env "slam-it": Python 3.11, cmake, C++ compiler, Rust
conda activate slam-it
python -m pytest
```

The C++ extension rebuilds on import after C++ changes. After Rust changes:

```bash
maturin develop --release -m rust/slam-it-py/Cargo.toml
```

Native tests, without Python:

```bash
cmake -S . -B build/native -G Ninja -DSLAM_IT_BUILD_TESTS=ON && cmake --build build/native -j4
ctest --test-dir build/native
cargo test --manifest-path rust/Cargo.toml --workspace --exclude slam-it-py
```

Native benchmarks (C++ and Rust) and bundle adjustment from Rust alone, on a decompressed BAL
problem (see [`docs/notes/rust-port.md`](docs/notes/rust-port.md)):

```bash
cmake -S . -B build/bench -G Ninja -DSLAM_IT_BUILD_BENCHMARKS=ON
cmake --build build/bench -j4
SLAM_IT_BAL=<problem.txt> build/bench/bench/foundations_bench_foundations
SLAM_IT_BAL=<problem.txt> cargo bench --manifest-path rust/Cargo.toml --bench foundations
cargo run --release --manifest-path rust/Cargo.toml --example bal -- <problem.txt>
```

## The viewer and your own video

A bundle of runs (`python -m slam_it.orchestration.bundles`) opens in the run viewer, and
the upload demo runs on this machine, on the local network with `--lan`
(see [`docs/notes/your-own-video.md`](docs/notes/your-own-video.md)):

```bash
python scripts/view_bundle.py <bundle>
python scripts/serve_demo.py --lan
```
