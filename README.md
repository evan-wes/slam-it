# slamlab

A learning ladder for visual SLAM, from first-principles geometry to learned dense SLAM,
implemented in Python, C++ and Rust and measured across compute budgets.

*Working name -- to be renamed before the repository goes public.*

Status: milestone 1 (rung 0, the foundations) is complete. Python, C++ and Rust each have
the geometry, bundle adjustment, two-view geometry and absolute pose, and every module's tests
run on all three.

The design, the ladder and the milestones are in
[`docs/design/2026-10-02-slamlab-ladder.md`](docs/design/2026-10-02-slamlab-ladder.md), and
each component's note in [`docs/notes/`](docs/notes/).

## Layout

| Path | What |
|---|---|
| `src/slamlab/` | Python: reference implementations, orchestration, evaluation |
| `cpp/` | C++ core, one library per rung (`slamlab_foundations`), and the nanobind module `slamlab._cpp` (`cpp/bindings/`) |
| `rust/` | Cargo workspace: one crate per rung (`slamlab-foundations`, standalone) and `slamlab-py` (the `slamlab_rs` module) |
| `tests/` | Python tests, including parity tests against `tests/fixtures/` |
| `scripts/` | Environment setup and lockfile, `check.sh` (layer 1), `check_layers.py` (C++/Rust layering) |

## Setup

```bash
scripts/setup_slamlab.sh            # conda env "slamlab": Python 3.11, cmake, C++ compiler, Rust
conda activate slamlab
python -m pytest
```

The C++ extension rebuilds on import after C++ changes. After Rust changes:

```bash
maturin develop --release -m rust/slamlab-py/Cargo.toml
```

Native tests, without Python:

```bash
cmake -S . -B build/native -G Ninja -DSLAMLAB_BUILD_TESTS=ON && cmake --build build/native -j4
ctest --test-dir build/native
cargo test --manifest-path rust/Cargo.toml --workspace --exclude slamlab-py
```

Native benchmarks (C++ and Rust) and bundle adjustment from Rust alone, on a decompressed BAL
problem (see [`docs/notes/rust-port.md`](docs/notes/rust-port.md)):

```bash
cmake -S . -B build/bench -G Ninja -DSLAMLAB_BUILD_BENCHMARKS=ON && cmake --build build/bench -j4
SLAMLAB_BAL=<problem.txt> build/bench/bench/foundations_bench_foundations
SLAMLAB_BAL=<problem.txt> cargo bench --manifest-path rust/Cargo.toml --bench foundations
cargo run --release --manifest-path rust/Cargo.toml --example bal -- <problem.txt>
```
