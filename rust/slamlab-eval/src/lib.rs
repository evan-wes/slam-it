//! slamlab's evaluation helpers in Rust: dataset readers and the run writer, outside the
//! algorithm cores.
//!
//! Mirrors the Python package `slamlab.eval` where a standalone Rust pipeline needs it: reading
//! PNG images (`png`), TUM and EuRoC sequences (`tum`, `euroc`, with `text` for their files),
//! writing a run's folder (`runs`), and a program's own peak memory (`budgets`). It sits above every rung in the ladder
//! (`scripts/check_layers.py` holds it to that), so no rung crate may depend on it.

pub mod budgets;
pub mod euroc;
pub mod png;
pub mod runs;
pub mod text;
pub mod tum;
