//! slamlab's Rust core.
//!
//! Builds standalone, without Python, for the edge compute budgets. Modules mirror the
//! Python package tree (`foundations::so3` implements `slamlab.foundations.so3`). The
//! Python bindings live in the separate `slamlab-py` crate.

pub mod foundations;
