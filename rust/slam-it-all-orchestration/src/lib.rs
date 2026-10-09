//! SLAM It All!'s native programs in Rust: a rung's pipeline over a dataset's sequence.
//!
//! Mirrors the Python package `slam_it_all.orchestration`. It sits at the top of the ladder
//! (`scripts/check_layers.py` holds it to that): it reads sequences and writes runs with
//! `slam-it-all-eval`, and runs the edge rung's pipeline.

pub mod vo;
