//! slamlab's evaluation helpers in Rust: dataset readers outside the algorithm cores.
//!
//! Mirrors the Python package `slamlab.eval` where a standalone Rust pipeline needs it: reading
//! image files. It sits above every rung in the ladder (`scripts/check_layers.py` holds it to
//! that), so no rung crate may depend on it.

pub mod png;
