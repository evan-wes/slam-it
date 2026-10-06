//! The edge rung: sparse visual odometry, ORB-SLAM-lite, in lock-step with
//! `slamlab.edge` and the C++ core's `slamlab_edge` (design doc, milestone 2).
//!
//! Builds on `slamlab-foundations` alone. Every module mirrors the Python module of its name.

pub mod adjust;
pub mod frames;
pub mod initialization;
pub mod map;
pub mod mapping;
pub mod options;
pub mod pipeline;
pub mod residuals;
pub mod tracking;
