//! Rung 0 of slamlab's Rust core, foundations: the geometry every other rung builds on.
//!
//! One crate per rung, so cargo enforces the ladder's direction: a rung crate may depend only
//! on the rung crates below it (checked against `pyproject.toml`'s layers by
//! `scripts/check_layers.py`). Builds standalone, without Python, for the edge compute
//! budgets. Modules mirror the Python package (`so3` implements `slamlab.foundations.so3`).

pub mod alignment;
pub mod camera;
pub mod error;
pub mod se3;
pub mod so3;
