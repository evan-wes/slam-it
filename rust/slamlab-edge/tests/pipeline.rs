//! Integration tests of the pipeline (`slamlab_edge::pipeline`) on the exact scene seen along
//! a sideways path: RGB-D and stereo track every frame exactly, monocular starts its map and
//! keeps the path's direction, and empty frames lose the track.

mod scene;

use nalgebra::{Matrix4, Vector3};
use slamlab_edge::frames::Frame;
use slamlab_edge::options::{Mode, Options};
use slamlab_edge::pipeline::{FrameResult, Pipeline, State};
use slamlab_foundations::error::ErrorKind;
use slamlab_foundations::orb::Features;

use scene::{camera, frame_at};

/// The path's frames.
const FRAMES: usize = 16;

/// The path: the camera moves 2 cm right each frame.
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn along(i: usize) -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    out[(0, 3)] = -0.02 * i as f64;
    out
}

/// The settings for a 60-point scene: the counts ORB-SLAM sizes for hundreds lowered.
fn small_scene() -> Options {
    Options {
        min_init_features: 20,
        min_init_features_depth: 20,
        init_min_matches: 20,
        ..Options::default()
    }
}

/// Runs the path through a pipeline.
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn run(mode: Mode) -> (Pipeline, Vec<FrameResult>) {
    let mut pipeline = Pipeline::new(mode, camera(), small_scene()).unwrap();
    let results = (0..FRAMES)
        .map(|i| {
            let frame = Frame {
                timestamp: i as f64,
                ..frame_at(&along(i), mode != Mode::Monocular, 0)
            };
            pipeline.track(&frame).unwrap()
        })
        .collect();
    (pipeline, results)
}

#[test]
fn depth_modes_track_every_frame_on_the_path() {
    for mode in [Mode::Rgbd, Mode::Stereo] {
        let (pipeline, results) = run(mode);
        assert_eq!(pipeline.state(), State::Tracking);
        assert!(results[0].keyframe);
        for (i, result) in results.iter().enumerate() {
            let pose = result.pose.expect("every frame is tracked");
            assert!((pose - along(i)).abs().max() < 1e-6);
        }
    }
}

#[test]
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn monocular_tracks_the_path_direction() {
    let (_, results) = run(Mode::Monocular);
    let first = results.iter().position(|r| r.pose.is_some()).unwrap();
    assert!(first < FRAMES - 4);
    let centre = |r: &FrameResult| -> Vector3<f64> {
        -r.pose.unwrap().fixed_view::<3, 1>(0, 3).into_owned()
    };
    let last = centre(&results[FRAMES - 1]);
    for (i, result) in results.iter().enumerate().skip(first) {
        let ratio = i as f64 / (FRAMES - 1) as f64;
        assert!((centre(result) - ratio * last).norm() < 1e-6 * last.norm());
    }
}

#[test]
fn empty_frames_lose_the_track() {
    let mut pipeline = Pipeline::new(Mode::Rgbd, camera(), small_scene()).unwrap();
    pipeline.track(&frame_at(&along(0), true, 0)).unwrap();
    let empty = Frame {
        timestamp: 1.0,
        features: Features {
            keypoints: Vec::new(),
            descriptors: Vec::new(),
            scale_factor: 1.2,
            levels: 8,
        },
        bearings: Vec::new(),
        depth: Vec::new(),
        right: Vec::new(),
    };
    for _ in 0..Options::default().lost_frames {
        assert!(pipeline.track(&empty).unwrap().pose.is_none());
    }
    assert_eq!(pipeline.state(), State::Lost);
    let mut monocular = camera();
    monocular.baseline = 0.0;
    let err = Pipeline::new(Mode::Rgbd, monocular, Options::default()).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}
