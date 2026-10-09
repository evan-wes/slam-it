# SLAM It All!

A learning ladder for visual SLAM, from first-principles geometry to learned dense SLAM,
implemented in Python, C++ and Rust and measured across compute budgets.

*A pun on "damn it all!": `slam_it_all` in code, `slam-it-all` for the repository.*

Status: milestone 1 (rung 0, the foundations) is complete. Python, C++ and Rust each have
the geometry, bundle adjustment, two-view geometry and absolute pose, and every module's tests
run on all three.

The design, the ladder and the milestones are in
[`docs/design/2026-10-02-slam-it-all-ladder.md`](docs/design/2026-10-02-slam-it-all-ladder.md), and
each component's note in [`docs/notes/`](docs/notes/).

## Layout

| Path | What |
|---|---|
| `src/slam_it_all/` | Python: reference implementations, orchestration, evaluation |
| `cpp/` | C++ core, one library per rung (`slam_it_all_foundations`), and the nanobind module `slam_it_all._cpp` (`cpp/bindings/`) |
| `rust/` | Cargo workspace: one crate per rung (`slam-it-all-foundations`, standalone) and `slam-it-all-py` (the `slam_it_all_rs` module) |
| `tests/` | Python tests, including parity tests against `tests/fixtures/` |
| `scripts/` | Environment setup and lockfile, `check.sh` (layer 1), `check_layers.py` (C++/Rust layering) |

## Setup

```bash
scripts/setup_slam_it_all.sh  # conda env "slam-it-all": Python 3.11, cmake, C++ compiler, Rust
conda activate slam-it-all
python -m pytest
```

The C++ extension rebuilds on import after C++ changes. After Rust changes:

```bash
maturin develop --release -m rust/slam-it-all-py/Cargo.toml
```

Native tests, without Python:

```bash
cmake -S . -B build/native -G Ninja -DSLAM_IT_ALL_BUILD_TESTS=ON && cmake --build build/native -j4
ctest --test-dir build/native
cargo test --manifest-path rust/Cargo.toml --workspace --exclude slam-it-all-py
```

Native benchmarks (C++ and Rust) and bundle adjustment from Rust alone, on a decompressed BAL
problem (see [`docs/notes/rust-port.md`](docs/notes/rust-port.md)):

```bash
cmake -S . -B build/bench -G Ninja -DSLAM_IT_ALL_BUILD_BENCHMARKS=ON
cmake --build build/bench -j4
SLAM_IT_ALL_BAL=<problem.txt> build/bench/bench/foundations_bench_foundations
SLAM_IT_ALL_BAL=<problem.txt> cargo bench --manifest-path rust/Cargo.toml --bench foundations
cargo run --release --manifest-path rust/Cargo.toml --example bal -- <problem.txt>
```
