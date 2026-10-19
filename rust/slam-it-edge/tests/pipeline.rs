//! Integration tests of the pipeline (`slam_it_edge::pipeline`) on the exact scene seen along
//! a sideways path: RGB-D and stereo track every frame exactly, monocular starts its map and
//! keeps the path's direction, and empty frames lose the track, the next frame starting a new
//! map; relocalising, a kept map is lost in time and a small one in frames, and a frame the
//! fallback cannot reach is relocalised; the final trajectory follows the keyframes, and a reset
//! map's frames leave it. Tracking on the view a writer would publish decides as on the map, and
//! threaded, the views published are a part of the map, their sizes kept; after a correction the
//! next frame follows its keyframe, and a keyframe queued across it is placed by its reference.

mod scene;

use nalgebra::{Matrix4, Vector3};
use slam_it_edge::frames::Frame;
use slam_it_edge::map::{Map, Removal};
use slam_it_edge::options::{
    InitializationOptions, LoopOptions, Mode, Options, RelocalizationOptions,
};
use slam_it_edge::pipeline::{
    Fate, FrameResult, LostSummary, Pipeline, Placement, State, Threading,
};
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
fn run(mode: Mode) -> (Pipeline, Vec<FrameResult>) {
    run_as(mode, Threading::Inline)
}

/// Runs the path through a pipeline whose steps run in turn, inline or each frame on a view.
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn run_as(mode: Mode, threading: Threading) -> (Pipeline, Vec<FrameResult>) {
    let mut pipeline = Pipeline::new(mode, camera(), small_scene(), None, threading).unwrap();
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

/// Each frame tracked on the view a writer would publish then decides as on the whole map
/// (`Threading::InlineView`): the same frames, poses, matches and stages, and the same map.
#[test]
fn tracking_on_a_view_decides_as_on_the_map() {
    for mode in [Mode::Rgbd, Mode::Stereo, Mode::Monocular] {
        let (whole, on_map) = run_as(mode, Threading::Inline);
        let (viewed, on_view) = run_as(mode, Threading::InlineView);
        assert_eq!(on_map.len(), on_view.len());
        for (a, b) in on_map.iter().zip(&on_view) {
            assert_eq!(
                (a.state, a.map, a.pose, a.tracked, a.keyframe, a.asked),
                (b.state, b.map, b.pose, b.tracked, b.keyframe, b.asked)
            );
            assert_eq!(a.summary, b.summary);
            assert_eq!(a.fates, b.fates);
        }
        // Printed, a monocular map's NaN depths compare equal.
        assert_eq!(
            format!("{:?}", whole.map()),
            format!("{:?}", viewed.map()),
            "{mode:?}: the same map"
        );
        assert!(
            viewed.completed().is_empty(),
            "in turn, nothing is published"
        );
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
        let mut pipeline =
            Pipeline::new(Mode::Rgbd, camera(), options, None, Threading::Inline).unwrap();
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
    let err = Pipeline::new(
        Mode::Rgbd,
        monocular,
        Options::default(),
        None,
        Threading::Inline,
    )
    .unwrap_err();
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

/// Starts a map on the path's first frame, a keyframe, with the one-word vocabulary, then tracks
/// `count` empty frames `step` seconds apart; returns their results, and checks that the last
/// alone loses the track.
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn lose_the_track(options: &Options, step: f64, count: usize) -> Vec<FrameResult> {
    let mut pipeline = Pipeline::new(
        Mode::Rgbd,
        camera(),
        *options,
        Some(one_word()),
        Threading::Inline,
    )
    .unwrap();
    assert!(
        pipeline
            .track(&frame_at(&along(0), true, 0))
            .unwrap()
            .keyframe
    );
    let failed: Vec<FrameResult> = (0..count)
        .map(|k| pipeline.track(&empty_frame(step * (k + 1) as f64)).unwrap())
        .collect();
    let states: Vec<State> = failed.iter().map(|r| r.state).collect();
    let mut expected = vec![State::Tracking; count - 1];
    expected.push(State::Lost);
    assert_eq!(states, expected, "step {step}, {count} frames");
    failed
}

/// Relocalising (Python's `test_a_large_map_is_lost_in_time_and_a_small_one_in_frames`): a map
/// above `loss.reset_keyframes` is lost once more than `loss.relocalize_seconds` have passed since
/// the first failed frame (at 0.25 s a frame, on the fourteenth: 3.25 s, where the thirteenth's 3 s
/// is not more), one of `loss.reset_keyframes` or fewer on the `loss.lost_frames`-th failure,
/// whatever the time; each failed frame tried relocalisation.
#[test]
fn relocalizing_a_large_map_is_lost_in_time_and_a_small_one_in_frames() {
    for (reset_keyframes, step, lost_on) in [(0, 0.25, 14), (1, 0.25, 5), (1, 0.001, 5)] {
        let failed = lose_the_track(&relocalizing(reset_keyframes), step, lost_on);
        assert!(failed.iter().all(|r| r.summary.relocalization.is_some()));
    }
}

/// Loop closing alone (Python's
/// `test_off_a_kept_map_is_lost_after_lost_frames_and_nothing_relocalizes`, its loop-closing
/// case): the keyframe database is kept, but no failed frame tries relocalisation, and a kept map
/// is lost on the `loss.lost_frames`-th failure (B3's review panel, run 2, A20).
#[test]
fn closing_loops_alone_relocalizes_nothing() {
    let mut options = relocalizing(0);
    options.relocalization.enabled = false;
    let failed = lose_the_track(&options, 0.001, options.loss.lost_frames);
    assert!(failed.iter().all(|r| r.summary.relocalization.is_none()));
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
    let mut pipeline = Pipeline::new(
        Mode::Rgbd,
        camera(),
        options,
        Some(one_word()),
        Threading::Inline,
    )
    .unwrap();
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

/// The scene's settings, a first keyframe's reference ratio raised to the others' (from 0.4),
/// so that [`thinning`]'s frames ask for a keyframe.
fn asking() -> Options {
    let mut out = small_scene();
    out.keyframes.first_ref_ratio = out.keyframes.ref_ratio;
    out
}

/// A frame of the path, RGB-D, at its time, that sees the scene's points from `2 i` on (24 at
/// most dropped): its tracking thins as the path goes, so that with [`asking`]'s settings it
/// asks for a keyframe once under `keyframes.ref_ratio` of its reference's points (from the
/// eighth).
#[allow(clippy::cast_precision_loss, reason = "frames below 16")]
fn thinning(i: usize) -> Frame {
    let full = frame_at(&along(i), true, 0);
    let dropped = (2 * i).min(24);
    let mut features = full.features;
    features.keypoints.drain(..dropped);
    features.descriptors.drain(..dropped);
    Frame::new(
        i as f64,
        features,
        full.bearings[dropped..].to_vec(),
        full.depth[dropped..].to_vec(),
        full.right[dropped..].to_vec(),
    )
    .unwrap()
}

/// Tracks the path's thinning frames through a pipeline.
fn track_path(pipeline: &mut Pipeline, frames: std::ops::Range<usize>) -> Vec<FrameResult> {
    frames
        .map(|i| pipeline.track(&thinning(i)).unwrap())
        .collect()
}

/// Threaded, RGB-D (`tests/edge/test_pipeline_threads.py`): every frame tracked on the true path;
/// each keyframe given mapped once, a completion on its frame with its schedule; the lives count
/// the first keyframe and each mapped one; the trajectory places every frame, in order; a
/// finished run tracks no more; and inline, nothing completes later.
#[test]
fn threaded_the_path_is_tracked_and_every_keyframe_mapped() {
    let mut pipeline =
        Pipeline::new(Mode::Rgbd, camera(), asking(), None, Threading::View).unwrap();
    let results = track_path(&mut pipeline, 0..FRAMES);
    pipeline.finish().unwrap();
    for (i, r) in results.iter().enumerate() {
        let pose = r.pose.expect("every frame tracked");
        assert!(
            (pose - along(i)).abs().max() < 1e-6,
            "frame {i} on the true path"
        );
        assert!(r.map_seconds == 0.0 && r.loop_seconds == 0.0);
    }
    let given: Vec<i64> = (1..FRAMES)
        .filter(|&i| results[i].keyframe)
        .map(|i| i64::try_from(i).unwrap())
        .collect();
    assert!(!given.is_empty(), "a keyframe was asked for");
    let done = pipeline.completed();
    let mut mapped: Vec<i64> = done
        .iter()
        .filter(|c| c.mapping.is_some())
        .map(|c| c.frame)
        .collect();
    mapped.sort_unstable();
    assert_eq!(mapped, given);
    assert!(
        done.iter()
            .filter(|c| c.mapping.is_some())
            .all(|c| c.schedule.is_some() && c.map_seconds.is_some())
    );
    let lives = pipeline.lifetimes();
    assert_eq!(lives.keyframe_times.len(), 1 + given.len());
    let views: Vec<_> = done.iter().filter_map(|c| c.view).collect();
    assert!(
        views.len() > given.len(),
        "a view at the start, and each keyframe's"
    );
    assert!(
        views
            .iter()
            .all(|v| v.keyframes >= 1 && v.points >= 1 && v.bytes > 0)
    );
    assert!(
        views.iter().all(|v| {
            v.keyframes <= lives.keyframe_times.len() && v.points <= lives.points.len()
        })
    );
    let trajectory = pipeline.trajectory();
    assert_eq!(trajectory.timestamps.len(), results.len());
    assert!(trajectory.timestamps.windows(2).all(|w| w[0] < w[1]));
    let err = pipeline.track(&thinning(0)).unwrap_err();
    assert_eq!(
        err.kind,
        ErrorKind::InvalidInput,
        "a finished run tracks no more"
    );
    let (mut inline, _) = run(Mode::Rgbd);
    inline.finish().unwrap();
    assert!(
        inline.completed().is_empty(),
        "inline, nothing completes later"
    );
}

/// While mapping is stopped, every keyframe tracking asks for is refused
/// (`test_a_stop_refuses_every_keyframe_asked`).
#[test]
fn threaded_a_stop_refuses_every_keyframe_asked() {
    let mut pipeline =
        Pipeline::new(Mode::Rgbd, camera(), asking(), None, Threading::View).unwrap();
    track_path(&mut pipeline, 0..FRAMES / 2);
    let writer = pipeline.writer().unwrap();
    let held = {
        let _stop = writer.stopped();
        track_path(&mut pipeline, FRAMES / 2..FRAMES)
    };
    pipeline.finish().unwrap();
    assert!(held.iter().any(|r| r.asked));
    assert!(
        held.iter()
            .filter(|r| r.asked)
            .all(|r| r.refused && !r.keyframe)
    );
}

/// A rigid move of the world, as a loop's correction moves a map: 20 degrees about the vertical
/// and 0.5 m along x, far past the motion model's search.
fn correction() -> Matrix4<f64> {
    let (sin, cos) = 20.0_f64.to_radians().sin_cos();
    #[rustfmt::skip]
    let g = Matrix4::new(
        cos, 0.0, sin, 0.5,
        0.0, 1.0, 0.0, 0.0,
        -sin, 0.0, cos, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );
    g
}

/// Moves a map's world by `g`, as a loop's correction moves it rigidly: a keyframe's
/// `T_cam_world` becomes `T_cam_world g⁻¹`, a point `x` becomes `g x`, its viewing direction
/// turned with it.
fn move_world(m: &mut Map, g: &Matrix4<f64>) {
    let inverse = g.try_inverse().unwrap();
    let rotation = g.fixed_view::<3, 3>(0, 0).into_owned();
    let shift = g.fixed_view::<3, 1>(0, 3).into_owned();
    for pose in &mut m.kf_pose {
        *pose *= inverse;
    }
    for x in &mut m.pt_xyz {
        *x = rotation * *x + shift;
    }
    for n in &mut m.pt_normal {
        *n = rotation * *n;
    }
}

/// After a correction, the next frame follows its keyframe
/// (`test_after_a_correction_the_next_frame_follows_its_keyframe`, B3b's review panel, A1): the
/// map corrected under a stop, the view published with it carries the new count; the next frame,
/// tracked on the one view it reads, moves its last frame with its keyframe first, so the motion
/// model places it, on the path in the corrected world.
#[test]
fn threaded_after_a_correction_the_next_frame_follows_its_keyframe() {
    let mut pipeline =
        Pipeline::new(Mode::Rgbd, camera(), asking(), None, Threading::View).unwrap();
    track_path(&mut pipeline, 0..FRAMES / 2);
    let g = correction();
    {
        let owner = pipeline.owner();
        let before = owner.view().unwrap().corrections;
        let corrected = owner
            .stopped(|m| {
                move_world(m, &g);
                Ok(Some(()))
            })
            .unwrap();
        assert!(corrected.is_some());
        let after = owner.view().unwrap();
        assert_eq!(
            after.corrections,
            before + 1,
            "the view carries the new count"
        );
        assert_eq!(after.corrections, owner.corrections());
    }
    let next = track_path(&mut pipeline, FRAMES / 2..FRAMES / 2 + 1);
    pipeline.finish().unwrap();
    assert_eq!(next[0].summary.placed_by, Some(Placement::Motion));
    let expected = along(FRAMES / 2) * g.try_inverse().unwrap();
    let pose = next[0].pose.expect("the frame tracked");
    assert!(
        (pose - expected).abs().max() < 1e-6,
        "on the path, in the corrected world"
    );
}

/// The scene's [`asking`] settings with local BA and keyframe culling off: a keyframe keeps the
/// pose it is inserted at, unless a correction moves the map.
fn still() -> Options {
    let mut out = asking();
    out.adjustment.min_keyframes = 1000;
    out.mapping.redundant_observers = 1000;
    out
}

/// A keyframe queued across a correction is placed by its reference
/// (`test_a_keyframe_queued_across_a_correction_is_placed_by_its_reference`, B3b's review panel,
/// A2): with mapping held off, the keyframes given wait while the map is corrected; once mapped,
/// each sits where its reference keyframe now is. With local BA and culling off ([`still`]),
/// every keyframe keeps the pose it was given: on the path, in the corrected world.
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "a path frame's time is its number"
)]
fn threaded_a_keyframe_queued_across_a_correction_is_placed_by_its_reference() {
    let mut pipeline = Pipeline::new(Mode::Rgbd, camera(), still(), None, Threading::View).unwrap();
    track_path(&mut pipeline, 0..FRAMES / 2);
    let g = correction();
    let writer = pipeline.writer().unwrap();
    let held = {
        let _hold = writer.held(); // the keyframes given wait
        let held = track_path(&mut pipeline, FRAMES / 2..3 * FRAMES / 4);
        // A stop inside the hold does not wait for itself.
        let corrected = pipeline
            .owner()
            .stopped(|m| {
                move_world(m, &g);
                Ok(Some(()))
            })
            .unwrap();
        assert!(corrected.is_some());
        held
    };
    track_path(&mut pipeline, 3 * FRAMES / 4..FRAMES);
    pipeline.finish().unwrap();
    assert!(
        held.iter().any(|r| r.keyframe),
        "a keyframe waited across the correction"
    );
    let m = pipeline.map().unwrap();
    let inverse = g.try_inverse().unwrap();
    for (time, pose) in m.kf_time.iter().zip(&m.kf_pose) {
        let i = *time as usize;
        assert!(
            (pose - along(i) * inverse).abs().max() < 1e-6,
            "frame {i}'s keyframe on the path, in the corrected world"
        );
    }
}

/// A frame's state, map and lost map, as a run's outcome is compared.
type Outcome = (State, Option<i64>, Option<LostSummary>);

/// Threaded, a lost map waits for the threads, then closes as inline: kept, and the next frame
/// starts map 1 (`test_a_lost_map_settles_the_threads_then_closes`).
#[test]
fn threaded_a_lost_map_settles_the_threads_then_closes() {
    let mut options = small_scene();
    options.loss.reset_keyframes = 0;
    let outcomes: Vec<Vec<Outcome>> = [Threading::Inline, Threading::View]
        .into_iter()
        .map(|threading| {
            let mut pipeline =
                Pipeline::new(Mode::Rgbd, camera(), options, None, threading).unwrap();
            let mut results = track_path(&mut pipeline, 0..1);
            for t in 1..=options.loss.lost_frames {
                #[allow(clippy::cast_precision_loss, reason = "a few frames")]
                results.push(pipeline.track(&empty_frame(t as f64)).unwrap());
            }
            results.extend(track_path(&mut pipeline, 1..2));
            pipeline.finish().unwrap();
            results
                .iter()
                .map(|r| (r.state, r.map, r.summary.lost))
                .collect()
        })
        .collect();
    assert_eq!(outcomes[0], outcomes[1]);
    assert_eq!(
        outcomes[1].last().map(|o| (o.0, o.1)),
        Some((State::Tracking, Some(1)))
    );
}
