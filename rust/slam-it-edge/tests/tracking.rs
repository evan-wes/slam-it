//! Integration tests of tracking (`slam_it_edge::tracking`) on the exact scene: refinement and
//! its outliers, the points in view, the motion model's level rule, the local map, and the
//! fallback. Parity with the Python reference is checked through the bindings by `tests/edge`.

mod scene;

use nalgebra::{Matrix3, Matrix4, Vector3};
use slam_it_edge::frames::Tracked;
use slam_it_edge::map::{self, Map};
use slam_it_edge::options::{Mode, Options};
use slam_it_edge::tracking::{
    LocalMapSummary, MotionSummary, in_view, optimize_pose, predict, relocalize_locally,
    track_last_frame, track_local_map,
};
use slam_it_foundations::error::ErrorKind;
use slam_it_foundations::ransac;

use scene::{POINTS, camera, frame, frame_at, moved, points, pose, scene_map};

/// The scene's point ids, one per keypoint.
#[allow(clippy::cast_possible_wrap, reason = "ids below 60")]
fn every_point() -> Vec<i64> {
    (0..POINTS as i64).collect()
}

/// The largest entry of two poses' difference.
fn gap(a: &Matrix4<f64>, b: &Matrix4<f64>) -> f64 {
    (a - b).abs().max()
}

#[test]
fn refinement_recovers_the_pose_and_drops_a_wrong_match() {
    let m = scene_map();
    let mut velocity = Matrix4::identity();
    velocity[(0, 3)] = 0.5;
    assert_eq!(predict(&pose(1), None), pose(1));
    assert!((predict(&pose(0), Some(&velocity))[(0, 3)] - 0.5).abs() < f64::EPSILON);
    let mut start = pose(2);
    let mut t = start.fixed_view_mut::<3, 1>(0, 3);
    t += Vector3::new(0.02, -0.01, 0.03);
    let mut points = every_point();
    points[5] = 40; // keypoint 5 matched to a point across the image
    points[40] = -1;
    let refined = optimize_pose(
        &m,
        &camera(),
        &frame(2, true),
        &Tracked {
            pose: start,
            points,
        },
        &Options::default(),
    )
    .unwrap();
    assert!(gap(&refined.pose, &pose(2)) < 1e-6);
    assert_eq!(refined.points[5], -1);
    assert_eq!(refined.inliers(), 58);
}

#[test]
fn every_point_is_in_view_and_none_from_behind() {
    let m = scene_map();
    let rows: Vec<usize> = (0..POINTS).collect();
    let seen = in_view(&m, &camera(), &pose(2), &rows, &Options::default()).unwrap();
    assert_eq!(seen.rows.len(), POINTS);
    let mut turned = pose(2);
    turned
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, -1.0)));
    let behind = in_view(&m, &camera(), &turned, &rows, &Options::default()).unwrap();
    assert_eq!(behind.rows, Vec::<usize>::new());
    let err = in_view(&m, &camera(), &pose(2), &[POINTS], &Options::default()).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}

/// The level scene: one keyframe at the origin, its keypoints on level 2, each with its point.
fn level_scene() -> Map {
    let last = frame_at(&Matrix4::identity(), false, 2);
    let mut m = Map::empty(8, 1.2);
    map::insert_keyframe(&mut m, &last, &Matrix4::identity(), &[-1; POINTS]).unwrap();
    let observers: Vec<Vec<usize>> = (0..POINTS).map(|p| vec![p]).collect();
    map::add_points(&mut m, &points(), &observers).unwrap();
    m
}

#[test]
fn moving_forward_searches_no_finer_level() {
    let m = level_scene();
    let last = frame_at(&Matrix4::identity(), false, 2);
    let last_tracked = Tracked {
        pose: Matrix4::identity(),
        points: every_point(),
    };
    let track = |mode, dz| {
        track_last_frame(
            &m,
            &camera(),
            &frame_at(&moved(dz), false, 1),
            &last,
            &last_tracked,
            &moved(dz),
            mode,
            &Options::default(),
        )
        .unwrap()
    };
    let forward = track(Mode::Rgbd, 0.3);
    assert!(forward.tracked.is_none());
    assert_eq!(
        forward.summary,
        MotionSummary {
            matches: 0,
            widened: true,
            inliers: None
        }
    );
    let back = track(Mode::Rgbd, -0.3);
    assert_eq!(
        back.summary,
        MotionSummary {
            matches: POINTS,
            widened: false,
            inliers: Some(POINTS)
        }
    );
    let back = back.tracked.unwrap();
    assert_eq!(back.inliers(), POINTS);
    assert!(gap(&back.pose, &moved(-0.3)) < 1e-9);
    assert_eq!(
        track(Mode::Monocular, 0.3).tracked.unwrap().inliers(),
        POINTS
    );
}

#[test]
fn the_local_map_and_the_fallback_place_the_frame() {
    let m = scene_map();
    let mut start = pose(2);
    let mut t = start.fixed_view_mut::<3, 1>(0, 3);
    t += Vector3::new(0.005, 0.0, -0.005);
    // The motion model matched the first few points: the keyframes seeing them are the local map.
    let mut seeded = Tracked {
        pose: start,
        points: vec![-1; POINTS],
    };
    for (p, id) in seeded.points.iter_mut().take(5).enumerate() {
        *id = i64::try_from(p).unwrap();
    }
    let options = Options::default();
    let local = track_local_map(
        &m,
        &camera(),
        &frame(2, true),
        &seeded,
        Mode::Rgbd,
        &options,
        false,
    )
    .unwrap();
    assert_eq!(local.tracked.inliers(), POINTS);
    assert_eq!((local.visible.len(), local.found.len()), (POINTS, POINTS));
    assert!(gap(&local.tracked.pose, &pose(2)) < 1e-6);
    assert_eq!(
        local.summary(0),
        LocalMapSummary {
            in_view: POINTS,
            matches: POINTS,
            inliers: POINTS,
            keyframes: local.keyframes,
            points: POINTS,
            reference: local.reference_id,
        }
    );
    assert!(local.keyframes > 0);
    let mut none = local.clone();
    none.reference_id = -1;
    assert_eq!(
        none.summary(10).reference,
        -1,
        "no reference keyframe: -1, not 9"
    );
    let last = &m.pt_id; // the last frame matched every point
    assert!(
        relocalize_locally(&m, &camera(), &frame(2, true), &[-2], &options).is_err(),
        "a last frame's point id below -1 is refused"
    );
    let recovery = relocalize_locally(&m, &camera(), &frame(2, true), last, &options).unwrap();
    let summary = recovery.summary;
    assert_eq!(
        (
            summary.candidates,
            summary.matches,
            summary.inliers,
            summary.refined
        ),
        (POINTS, Some(POINTS), Some(POINTS), Some(POINTS))
    );
    assert_eq!(summary.termination, Some(ransac::Termination::Confidence));
    assert!(summary.iterations.is_some() && !summary.degenerate);
    let placed = recovery.tracked.unwrap();
    assert_eq!(placed.inliers(), POINTS);
    assert!(gap(&placed.pose, &pose(2)) < 1e-6);
    let recovering = track_local_map(
        &m,
        &camera(),
        &frame(2, true),
        &seeded,
        Mode::Rgbd,
        &options,
        true,
    )
    .unwrap();
    assert_eq!(recovering.tracked.inliers(), POINTS);
    let short = Tracked {
        pose: start,
        points: vec![0],
    };
    let err = optimize_pose(&m, &camera(), &frame(2, true), &short, &options).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}
