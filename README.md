# slamlab

A learning ladder for visual SLAM, from first-principles geometry to learned dense SLAM,
implemented in Python, C++ and Rust and measured across compute budgets.

*Working name -- to be renamed before the repository goes public.*

Status: scaffold. The three-language build pipeline works; no SLAM algorithms yet. The design,
the ladder and the milestones are in
[`docs/design/2026-10-02-slamlab-ladder.md`](docs/design/2026-10-02-slamlab-ladder.md).

## Layout

| Path | What |
|---|---|
| `src/slamlab/` | Python: reference implementations, orchestration, evaluation |
| `cpp/` | C++ core (`slamlab_core`) and its nanobind module `slamlab._cpp` |
| `rust/` | Cargo workspace: `slamlab-core` (standalone) and `slamlab-py` (the `slamlab_rs` module) |
| `tests/` | Python tests, including parity tests against `tests/fixtures/` |
| `scripts/` | Environment setup and lockfile |

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
cargo test --manifest-path rust/Cargo.toml -p slamlab-core
```
