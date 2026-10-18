//! Integration tests of the pipeline (`slam_it_edge::pipeline`) on the exact scene seen along
//! a sideways path: RGB-D and stereo track every frame exactly, monocular starts its map and
//! keeps the path's direction, and empty frames lose the track, the next frame starting a new
//! map; relocalising, a kept map is lost in time and a small one in frames, and a frame the
//! fallback cannot reach is relocalised; the final trajectory follows the keyframes, and a reset
//! map's frames leave it.

mod scene;

use nalgebra::{Matrix4, Vector3};
use slam_it_edge::frames::Frame;
use slam_it_edge::map::Removal;
use slam_it_edge::options::{
    InitializationOptions, LoopOptions, Mode, Options, RelocalizationOptions,
};
use slam_it_edge::pipeline::{Fate, FrameResult, LostSummary, Pipeline, Placement, State};
use slam_it_foundations::error::ErrorKind;

use scene::{camera, empty_frame, frame_at, one_word};

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
        initialization: InitializationOptions {
            min_features: 20,
            min_features_depth: 20,
            min_matches: 20,
            ..InitializationOptions::default()
        },
        relocalization: RelocalizationOptions {
            enabled: false, // A2's loss rule; relocalizing() turns it on, with a vocabulary
            ..RelocalizationOptions::default()
        },
        loops: LoopOptions {
            enabled: false, // the keyframe database's other user; relocalizing() turns it on
            ..LoopOptions::default()
        },
        ..Options::default()
    }
}

/// Runs the path through a pipeline.
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn run(mode: Mode) -> (Pipeline, Vec<FrameResult>) {
    let mut pipeline = Pipeline::new(mode, camera(), small_scene(), None).unwrap();
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

/// The final trajectory: every tracked frame once, in order, its pose as tracked beside its
/// final one; a keyframe's own frame where its keyframe is now, the others placed by theirs
/// (here, where they were tracked: the path is exact).
#[test]
fn the_final_trajectory_follows_the_keyframes() {
    for mode in [Mode::Rgbd, Mode::Monocular] {
        let (pipeline, results) = run(mode);
        let trajectory = pipeline.trajectory();
        let tracked: Vec<&FrameResult> = results.iter().filter(|r| r.pose.is_some()).collect();
        let times: Vec<f64> = tracked.iter().map(|r| r.timestamp).collect();
        assert_eq!(trajectory.timestamps, times);
        assert!(trajectory.maps.iter().all(|&m| m == 0));
        for (k, result) in tracked.iter().enumerate() {
            assert_eq!(trajectory.tracked[k], result.pose.unwrap());
            assert!((trajectory.poses[k] - result.pose.unwrap()).abs().max() < 1e-6);
        }
        let m = pipeline.map().unwrap();
        for (k, &time) in m.kf_time.iter().enumerate() {
            if let Some(at) = times.iter().position(|&t| (t - time).abs() < 1e-12) {
                assert!((trajectory.poses[at] - m.kf_pose[k]).abs().max() < 1e-12);
            }
        }
        assert!(results.iter().all(|r| r.loop_seconds == 0.0)); // not relocalising: no loops
        assert_eq!(pipeline.lifetimes().keyframe_loops.len(), 0);
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
fn empty_frames_lose_the_track_and_the_next_frame_starts_a_new_map() {
    // The map has one keyframe: 1, the rule's boundary, resets it; 0 keeps it.
    for reset_keyframes in [1, 0] {
        let mut options = small_scene();
        options.loss.reset_keyframes = reset_keyframes;
        let mut pipeline = Pipeline::new(Mode::Rgbd, camera(), options, None).unwrap();
        let started = pipeline.track(&frame_at(&along(0), true, 0)).unwrap();
        assert_eq!(started.map, Some(0));
        let empty = empty_frame(1.0);
        let results: Vec<FrameResult> = (0..options.loss.lost_frames)
            .map(|_| pipeline.track(&empty).unwrap())
            .collect();
        assert!(results.iter().all(|r| r.pose.is_none() && r.map == Some(0)));
        let lost = results.last().unwrap();
        let reset = reset_keyframes > 0;
        assert_eq!(lost.state, State::Lost);
        assert_eq!(
            lost.summary.lost,
            Some(LostSummary {
                keyframes: 1,
                points: started.tracked,
                reset
            })
        );
        assert_eq!(pipeline.state(), State::NotInitialized);
        assert!(pipeline.map().is_none());
        let restarted = pipeline.track(&frame_at(&along(1), true, 0)).unwrap();
        assert_eq!((restarted.map, restarted.keyframe), (Some(1), true));
        // A reset map's frames leave the trajectory; a kept one's stay.
        assert_eq!(
            pipeline.trajectory().maps,
            if reset { vec![1] } else { vec![0, 1] }
        );
        let lives = pipeline.lifetimes();
        assert_eq!(lives.keyframe_maps, vec![0, 1]);
        let first = started.tracked;
        let (lost_at, code) = if reset {
            (
                i64::try_from(options.loss.lost_frames).unwrap(),
                Removal::Reset.code(),
            )
        } else {
            (-1, -1)
        };
        assert_eq!(lives.keyframe_lives[0], [0, lost_at, code]);
        for (id, (&map, life)) in lives.point_maps.iter().zip(&lives.point_lives).enumerate() {
            assert_eq!(map, i64::from(id >= first));
            assert_eq!(life[2], if id < first { code } else { -1 });
        }
        let first = i64::try_from(first).unwrap();
        for (fate, &point) in restarted.fates.fate.iter().zip(&restarted.fates.point) {
            assert!(*fate != Fate::Made || point >= first); // the run's ids follow map 0's
        }
    }
    let mut monocular = camera();
    monocular.baseline = 0.0;
    let err = Pipeline::new(Mode::Rgbd, monocular, Options::default(), None).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}

/// The settings relocalising: the scene's, with `relocalization.enabled` and `loops.enabled` on
/// and a lost map of more than `loss.reset_keyframes` keyframes kept.
fn relocalizing(reset_keyframes: usize) -> Options {
    let mut out = small_scene();
    out.relocalization.enabled = true;
    out.loops.enabled = true;
    out.loss.reset_keyframes = reset_keyframes;
    out
}

/// Relocalising (Python's `test_a_large_map_is_lost_in_time_and_a_small_one_in_frames`): a map
/// above `loss.reset_keyframes` is lost once more than `loss.relocalize_seconds` have passed since
/// the first failed frame (at 0.25 s a frame, on the fourteenth: 3.25 s, where the thirteenth's 3 s
/// is not more), one of `loss.reset_keyframes` or fewer on the `loss.lost_frames`-th failure,
/// whatever the time; each failed frame tried relocalisation.
#[test]
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn relocalizing_a_large_map_is_lost_in_time_and_a_small_one_in_frames() {
    for (reset_keyframes, step, lost_on) in [(0, 0.25, 14), (1, 0.25, 5), (1, 0.001, 5)] {
        let options = relocalizing(reset_keyframes);
        let mut pipeline = Pipeline::new(Mode::Rgbd, camera(), options, Some(one_word())).unwrap();
        assert!(
            pipeline
                .track(&frame_at(&along(0), true, 0))
                .unwrap()
                .keyframe
        );
        let failed: Vec<FrameResult> = (0..lost_on)
            .map(|k| pipeline.track(&empty_frame(step * (k + 1) as f64)).unwrap())
            .collect();
        let states: Vec<State> = failed.iter().map(|r| r.state).collect();
        let mut expected = vec![State::Tracking; lost_on - 1];
        expected.push(State::Lost);
        assert_eq!(
            states, expected,
            "reset_keyframes {reset_keyframes}, step {step}"
        );
        assert!(failed.iter().all(|r| r.summary.relocalization.is_some()));
    }
}

/// A frame the local fallback cannot reach is placed by relocalisation in the same map, at its
/// pose: after the path, 30 cm to the right, and an empty frame, the camera is back where the
/// map began, 25 px away, past the motion model's narrowed window.
#[test]
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn a_frame_out_of_the_fallbacks_reach_is_relocalized_in_its_map() {
    let mut options = relocalizing(0);
    options.tracking.pnp_min_inliers = 1 << 20; // the local fallback out of reach
    options.tracking.motion_window = 5.0; // 10 px widened: the 25 px back to the start out of reach
    let mut pipeline = Pipeline::new(Mode::Rgbd, camera(), options, Some(one_word())).unwrap();
    for i in 0..FRAMES {
        let mut frame = frame_at(&along(i), true, 0);
        frame.timestamp = i as f64;
        assert!(pipeline.track(&frame).unwrap().pose.is_some(), "frame {i}");
    }
    assert!(
        pipeline
            .track(&empty_frame(FRAMES as f64))
            .unwrap()
            .pose
            .is_none()
    );
    let mut back = frame_at(&along(0), true, 0);
    back.timestamp = (FRAMES + 1) as f64;
    let found = pipeline.track(&back).unwrap();
    assert_eq!((found.map, found.state), (Some(0), State::Tracking));
    assert_eq!(found.summary.placed_by, Some(Placement::Relocalization));
    assert!((found.pose.unwrap() - along(0)).abs().max() < 1e-6);
}
