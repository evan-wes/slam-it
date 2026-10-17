//! The edge rung: sparse visual odometry, ORB-SLAM-lite, in lock-step with
//! `slam_it.edge` and the C++ core's `slam_it_edge` (design doc, milestone 2).
//!
//! Builds on `slam-it-foundations` alone. Every module mirrors the Python module of its name.

pub mod adjust;
pub mod covisibility;
pub mod database;
pub mod frames;
pub mod initialization;
pub mod map;
pub mod mapping;
pub mod options;
pub mod pipeline;
pub mod relocalization;
pub mod residuals;
pub mod tracking;
