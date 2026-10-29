//! A small exact scene shared by the integration tests (C++'s `cpp/tests/edge/scene.hpp`): 60
//! points before three keyframes, every keypoint at its point's projection, so a bundle
//! adjustment's optimum is the truth.
#![allow(dead_code, reason = "each integration test uses a part of the scene")]

use nalgebra::{Matrix4, Vector2, Vector3};
use slam_it_edge::frames::{Camera, Frame};
use slam_it_edge::map::{self, Map};
use slam_it_foundations::camera::Model;
use slam_it_foundations::orb::{Features, Keypoint};
use slam_it_foundations::vocabulary::{self, Vocabulary};

/// The scene's points.
pub const POINTS: usize = 60;
/// Its keyframes.
pub const KEYFRAMES: usize = 3;

/// The scene's camera: 250 px focal, 320 x 240, a 0.1 m baseline.
///
/// # Returns
///
/// The camera.
pub fn camera() -> Camera {
    Camera {
        model: Model::Pinhole,
        intrinsics: vec![250.0, 250.0, 160.0, 120.0],
        width: 320,
        height: 240,
        baseline: 0.1,
    }
}

/// The points: a 10 x 6 grid three metres out, its depths rippled so it is no plane.
///
/// # Returns
///
/// The points.
#[allow(clippy::cast_precision_loss, reason = "indices below 60, exact in f64")]
pub fn points() -> Vec<Vector3<f64>> {
    (0..POINTS)
        .map(|p| {
            let (i, j) = ((p % 10) as f64, (p / 10) as f64);
            Vector3::new(
                -1.0 + i * 2.0 / 9.0,
                -0.6 + j * 0.24,
                3.0 + 0.4 * (1.7 * p as f64).sin(),
            )
        })
        .collect()
}

/// A keyframe's true `T_cam_world`: the camera moves right, then up and forward.
///
/// # Arguments
///
/// * `k` - The keyframe.
///
/// # Returns
///
/// The pose.
pub fn pose(k: usize) -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    let t = match k {
        1 => Vector3::new(-0.2, 0.0, 0.0),
        2 => Vector3::new(-0.4, 0.05, 0.1),
        _ => Vector3::zeros(),
    };
    out.fixed_view_mut::<3, 1>(0, 3).copy_from(&t);
    out
}

/// A frame of the scene: one keypoint at each point's projection, on one level, each with its
/// own descriptor, and its depth and right column when asked.
///
/// # Arguments
///
/// * `t` - The camera's `T_cam_world`.
/// * `with_depth` - Whether the keypoints have depth.
/// * `level` - The keypoints' level.
///
/// # Returns
///
/// The frame.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    reason = "small indices and in-image pixels"
)]
pub fn frame_at(t: &Matrix4<f64>, with_depth: bool, level: usize) -> Frame {
    let cam = camera();
    let scale = 1.2_f64.powf(level as f64);
    let mut keypoints = Vec::new();
    let mut descriptors = Vec::new();
    let (mut bearings, mut depth, mut right) = (Vec::new(), Vec::new(), Vec::new());
    for (p, x) in points().iter().enumerate() {
        let local = t.fixed_view::<3, 3>(0, 0) * x + t.fixed_view::<3, 1>(0, 3);
        let px = cam.project(&local);
        keypoints.push(Keypoint {
            xy: [px.x, px.y],
            level,
            level_xy: [
                (px.x / scale).round() as usize,
                (px.y / scale).round() as usize,
            ],
            response: 0,
            angle: 0,
        });
        descriptors.push(std::array::from_fn(|b| ((p * 37 + b * 11) & 0xFF) as u8));
        bearings.push(cam.bearing(&Vector2::new(px.x, px.y)));
        depth.push(if with_depth { local.z } else { f64::NAN });
        right.push(if with_depth {
            px.x - cam.focal() * cam.baseline / local.z
        } else {
            f64::NAN
        });
    }
    let features = Features {
        keypoints,
        descriptors,
        scale_factor: 1.2,
        levels: 8,
    };
    Frame::new(0.0, features, bearings, depth, right).unwrap()
}

/// A keyframe's frame: its keypoints on level 0 (see [`frame_at`]).
///
/// # Arguments
///
/// * `k` - The keyframe.
/// * `with_depth` - Whether the keypoints have depth.
///
/// # Returns
///
/// The frame.
#[allow(clippy::cast_precision_loss, reason = "keyframes below 3")]
pub fn frame(k: usize, with_depth: bool) -> Frame {
    Frame {
        timestamp: k as f64,
        ..frame_at(&pose(k), with_depth, 0)
    }
}

/// The pose of a camera moved along its axis from the origin.
///
/// # Arguments
///
/// * `dz` - How far, in metres.
///
/// # Returns
///
/// Its `T_cam_world`.
pub fn moved(dz: f64) -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    out[(2, 3)] = -dz;
    out
}

/// A vocabulary of one word, weighing 1: every descriptor's, so every frame's vector is the same,
/// every keyframe is a relocalisation candidate, and matching by word compares every feature
/// with every point (the scene's descriptors tell its points apart).
///
/// # Returns
///
/// The vocabulary.
pub fn one_word() -> Vocabulary {
    Vocabulary {
        k: 2,
        levels: 1,
        descriptors: vec![[0; 32]; 2],
        parents: vec![-1, 0],
        children: vec![[1, 1], [0, 0]],
        words: vec![-1, 0],
        weights: vec![1.0],
    }
}

/// A frame's keypoints' vocabulary nodes by [`one_word`]: one node for every keypoint (the
/// vocabulary's one leaf, above the direct level), so matching by node compares every keypoint
/// with every other, as these tests did before keyframes kept their nodes.
///
/// # Arguments
///
/// * `f` - The frame.
///
/// # Returns
///
/// Its nodes.
pub fn nodes(f: &Frame) -> Vec<i64> {
    vocabulary::quantize(&one_word(), &f.features.descriptors).nodes
}

/// Appends a point at `xyz` with the columns a test map needs and the stages read: a blank
/// descriptor, a normal along +z, distances of 1, seen and found once, first and referenced by
/// keyframe 0. The caller sets its id.
///
/// # Arguments
///
/// * `m` - The map.
/// * `xyz` - The point.
pub fn push_point(m: &mut Map, xyz: Vector3<f64>) {
    m.pt_xyz.push(xyz);
    m.pt_descriptor.push([0; 32]);
    m.pt_normal.push(Vector3::z());
    m.pt_min_distance.push(1.0);
    m.pt_max_distance.push(1.0);
    m.pt_visible.push(1);
    m.pt_found.push(1);
    m.pt_first_kf.push(0);
    m.pt_reference_kf.push(0);
}

/// A frame with no keypoints, at a time.
///
/// # Arguments
///
/// * `timestamp` - The time.
///
/// # Returns
///
/// The frame.
///
/// # Panics
///
/// Never: an empty frame is well formed.
pub fn empty_frame(timestamp: f64) -> Frame {
    let features = Features {
        keypoints: Vec::new(),
        descriptors: Vec::new(),
        scale_factor: 1.2,
        levels: 8,
    };
    Frame::new(timestamp, features, Vec::new(), Vec::new(), Vec::new()).unwrap()
}

/// The scene's map: the first keyframe makes a point at each keypoint, the others observe
/// them all.
///
/// # Returns
///
/// The map.
///
/// # Panics
///
/// If a map operation fails (a bug in the scene).
#[allow(clippy::cast_possible_wrap, reason = "point ids below 60")]
pub fn scene_map() -> Map {
    let mut m = Map::empty(8, 1.2);
    let first = frame(0, true);
    map::insert_keyframe(&mut m, &first, &pose(0), &[-1; POINTS], &nodes(&first)).unwrap();
    let observers: Vec<Vec<usize>> = (0..POINTS).map(|p| vec![p]).collect();
    map::add_points(&mut m, &points(), &observers).unwrap();
    let ids: Vec<i64> = (0..POINTS as i64).collect();
    for k in 1..KEYFRAMES {
        let later = frame(k, true);
        map::insert_keyframe(&mut m, &later, &pose(k), &ids, &nodes(&later)).unwrap();
    }
    m
}
