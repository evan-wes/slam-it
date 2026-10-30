//! Integration tests of loop closing (`slam_it_edge::loops`) on the exact scene, seen twice: its
//! three keyframes, and the same three views again after a drift, which made their own points
//! where the scene's are (every keypoint on one level, [`LEVEL`]). The second visit closes a loop back to the first, and its correction
//! puts the drifted keyframes back where they were, onto the scene's points; the vocabulary has
//! one word, so every keyframe is a candidate.

mod scene;

use nalgebra::{Matrix4, Vector3};
use slam_it_edge::covisibility;
use slam_it_edge::database::Database;
use slam_it_edge::frames::Frame;
use slam_it_edge::loops::{self, Outcome, Search};
use slam_it_edge::map::{self, Map};
use slam_it_edge::options::{Mode, Options};
use slam_it_foundations::error::ErrorKind;
use slam_it_foundations::graphs;
use slam_it_foundations::nls::Stop;
use slam_it_foundations::so3;

use scene::{KEYFRAMES, POINTS, camera, frame_at, nodes, one_word, points, pose};

/// The drift the second visit has: a turn of 3 degrees about y and a shift.
///
/// # Returns
///
/// The drift, a rigid transform of the world.
fn drift() -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3::exp(&Vector3::new(0.0, 3.0_f64.to_radians(), 0.0)));
    out.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&Vector3::new(0.1, 0.0, 0.05));
    out
}

/// The level the views' keypoints are on: a point seen there is matchable from up to 1.44
/// times its distance, so the views, a few centimetres apart, all are within its range.
const LEVEL: usize = 2;

/// A keyframe's frame at a time, its keypoints on [`LEVEL`].
///
/// # Arguments
///
/// * `k` - The view.
/// * `timestamp` - The time.
///
/// # Returns
///
/// The frame.
fn view(k: usize, timestamp: f64) -> Frame {
    Frame {
        timestamp,
        ..frame_at(&pose(k), true, LEVEL)
    }
}

/// The scene seen twice: keyframes 0 to 2 the scene's map, keyframes 3 to 5 the same views
/// drifted, keyframe 3 making a point of its own at each keypoint, 4 and 5 observing them.
///
/// # Returns
///
/// The map, each keyframe given its parent.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "tiny counts"
)]
fn twice_seen() -> Map {
    let mut m = Map::empty(8, 1.2);
    let observers: Vec<Vec<usize>> = (0..POINTS).map(|p| vec![p]).collect();
    map::insert_keyframe(
        &mut m,
        &view(0, 0.0),
        &pose(0),
        &[-1; POINTS],
        &nodes(&view(0, 0.0)),
    )
    .unwrap();
    map::add_points(&mut m, &points(), &observers).unwrap();
    let first: Vec<i64> = (0..POINTS as i64).collect();
    for k in 1..KEYFRAMES {
        map::insert_keyframe(
            &mut m,
            &view(k, k as f64),
            &pose(k),
            &first,
            &nodes(&view(k, k as f64)),
        )
        .unwrap();
        covisibility::adopt(&mut m, k).unwrap();
    }
    let undo = drift().try_inverse().unwrap();
    let start = KEYFRAMES;
    map::insert_keyframe(
        &mut m,
        &view(0, 3.0),
        &(pose(0) * undo),
        &[-1; POINTS],
        &nodes(&view(0, 3.0)),
    )
    .unwrap();
    covisibility::adopt(&mut m, start).unwrap();
    let moved: Vec<Vector3<f64>> = points()
        .iter()
        .map(|p| drift().fixed_view::<3, 3>(0, 0) * p + drift().fixed_view::<3, 1>(0, 3))
        .collect();
    let observers: Vec<Vec<usize>> = (0..POINTS).map(|p| vec![start * POINTS + p]).collect();
    map::add_points(&mut m, &moved, &observers).unwrap();
    let second: Vec<i64> = (POINTS as i64..2 * POINTS as i64).collect();
    for k in 1..KEYFRAMES {
        let at = 3.0 + k as f64;
        map::insert_keyframe(
            &mut m,
            &view(k, at),
            &(pose(k) * undo),
            &second,
            &nodes(&view(k, at)),
        )
        .unwrap();
        covisibility::adopt(&mut m, start + k).unwrap();
    }
    m
}

/// The scene's points' ids in the map: the first visit's.
///
/// # Returns
///
/// 0 to 59.
#[allow(clippy::cast_possible_wrap, reason = "60 points")]
fn scene_ids() -> Vec<i64> {
    (0..POINTS as i64).collect()
}

/// Every keyframe in a database of one word.
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Returns
///
/// The database.
fn database(m: &Map) -> Database {
    let words = one_word();
    let mut db = Database::default();
    for k in 0..m.keyframes() {
        let rows = m.kf_start[k]..m.kf_start[k + 1];
        db.add(&words, m.kf_id[k], &m.kp_descriptor[rows]).unwrap();
    }
    db
}

/// The settings for a map of six keyframes and 60 points: searched from 6 keyframes, its
/// searches needing 50 matches, closed by `confirmations`.
///
/// # Arguments
///
/// * `confirmations` - The confirmations that close a loop.
///
/// # Returns
///
/// The settings.
fn settings(confirmations: usize) -> Options {
    let mut out = Options::default();
    out.loops.min_keyframes = 6;
    out.loops.fine_matches = 50;
    out.loops.temporal_matches = 50;
    out.loops.confirmations = confirmations;
    out
}

/// The largest difference between two poses' entries.
///
/// # Arguments
///
/// * `a`, `b` - The poses.
///
/// # Returns
///
/// It.
fn gap(a: &Matrix4<f64>, b: &Matrix4<f64>) -> f64 {
    (a - b).abs().max()
}

/// A similarity's scale and rigid part: `[[s R, t], [0, 1]]` projects through `[[R, t / s]]`.
#[test]
fn a_similarity_has_a_scale_and_a_rigid_part() {
    let mut scaled = pose(2) * drift();
    let rigid = scaled;
    for r in 0..3 {
        for c in 0..4 {
            scaled[(r, c)] *= 1.7;
        }
    }
    assert!((loops::scale_of(&scaled) - 1.7).abs() < 1e-15);
    assert!(gap(&loops::rigid_of(&scaled), &rigid) < 1e-15);
}

/// Global BA's solve: 10 iterations, g2o's damping from `tau` 1e-5, ORB-SLAM3's stop after three
/// small steps; a welding BA's: GTSAM's own damping and no stop; and the near-plane filter's
/// default.
#[test]
fn global_ba_damps_as_g2o_and_stops_as_orb_slam3_and_the_weld_keeps_gtsam_s_damping() {
    let options = Options::default();
    let stop = Stop::never();
    let global = loops::global_ba_options(&options, &stop);
    assert_eq!(global.max_iterations, 10);
    assert!(global.scaled_damping);
    assert_eq!(
        (
            global.damping_scale,
            global.small_steps,
            global.small_decrease
        ),
        (1e-5, 3, 1e-3)
    );
    let weld = loops::weld_options(5);
    let gtsam = graphs::Options::default();
    assert_eq!(weld.max_iterations, 5);
    assert!(!weld.scaled_damping);
    assert_eq!(
        (weld.initial_damping, weld.small_steps),
        (gtsam.initial_damping, 0)
    );
    assert_eq!(options.loops.min_depth_ratio, 1e-3);
}

/// The RANSAC budget: `SetRansacParameters(0.99, 15, 300)`.
#[test]
fn the_ransac_budget_is_orb_slams() {
    let options = Options::default();
    assert_eq!(loops::ransac_budget(15, &options), 1);
    assert_eq!(loops::ransac_budget(30, &options), 35);
    assert_eq!(loops::ransac_budget(1000, &options), 300);
}

/// Searched through its own pose, each of the first visit's keyframes finds every point it
/// observes on its own keypoint; at distance 0 too, its descriptors being its points'.
#[test]
fn a_keyframe_finds_its_points_through_its_own_pose() {
    let m = twice_seen();
    let rows: Vec<usize> = (0..POINTS).collect();
    for k in 0..KEYFRAMES {
        for distance in [0, 50] {
            let found = loops::search_by_similarity(
                &m,
                &camera(),
                k,
                &m.kf_pose[k],
                &rows,
                Search {
                    window: 5.0,
                    distance,
                },
                &Options::default(),
            )
            .unwrap();
            assert_eq!(found, m.kp_point[m.kf_start[k]..m.kf_start[k + 1]]);
        }
    }
}

/// The second visit matches the first by word, point for point.
#[test]
fn the_second_visit_matches_the_first_by_word() {
    let m = twice_seen();
    let found = loops::match_by_word(&m, &database(&m), 5, 0, &Options::default()).unwrap();
    let expected = scene_ids();
    assert_eq!(found, expected);
}

/// The refinement finds the drift's relative motion between the visits from a nudged start.
#[test]
fn the_refinement_finds_the_motion_between_the_visits() {
    let m = twice_seen();
    let truth = pose(2) * pose(0).try_inverse().unwrap();
    let mut start = truth;
    start[(0, 3)] += 0.05;
    let matched = scene_ids();
    let refined = loops::refine(
        &m,
        &camera(),
        5,
        0,
        &start,
        &matched,
        true,
        &Options::default(),
    )
    .unwrap()
    .expect("refined");
    assert_eq!(refined.inliers, POINTS);
    assert!(gap(&refined.similarity, &truth) < 1e-9);
}

/// The loop closes at the second visit's last keyframe, matched to the first visit's first,
/// placed at its true pose, confirmed by its two links.
#[test]
fn the_loop_closes_on_the_second_visit() {
    let m = twice_seen();
    let found = loops::detect(
        &m,
        &database(&m),
        &camera(),
        5,
        None,
        Mode::Rgbd,
        &settings(2),
    )
    .unwrap();
    assert_eq!(found.summary.outcome, Outcome::Closed);
    assert!(found.pending.is_none());
    let closed = found.closed.expect("closed");
    assert_eq!((closed.current, closed.matched), (5, 0));
    assert!(gap(&closed.similarity, &pose(2)) < 1e-9);
    assert_eq!(closed.points.len(), POINTS);
    let tried = &found.summary.candidates[0];
    assert_eq!(tried.matches, Some(POINTS));
    assert_eq!(tried.confirmations, Some(2));
}

/// With three confirmations needed, the loop starts on the middle keyframe, held by it and the
/// matched one, and closes on the next, carried there.
#[test]
fn a_loop_is_carried_until_it_closes() {
    let m = twice_seen();
    let db = database(&m);
    let started = loops::detect(&m, &db, &camera(), 4, None, Mode::Rgbd, &settings(3)).unwrap();
    assert_eq!(started.summary.outcome, Outcome::Started);
    let pending = started.pending.expect("carried");
    assert_eq!(pending.confirmations, 2);
    assert_eq!(pending.held(), [0, 4]);
    let closed = loops::detect(
        &m,
        &db,
        &camera(),
        5,
        Some(&pending),
        Mode::Rgbd,
        &settings(3),
    )
    .unwrap();
    assert_eq!(closed.summary.outcome, Outcome::Closed);
    assert!(closed.summary.confirmation.is_some_and(|c| c.confirmed));
    assert_eq!(closed.summary.candidates.len(), 0);
    assert_eq!(closed.closed.expect("closed").current, 5);
}

/// A map below `loops.min_keyframes` is not searched; a keyframe outside it is refused.
#[test]
fn a_small_map_is_skipped_and_a_stranger_refused() {
    let m = twice_seen();
    let db = database(&m);
    let mut fewer = settings(2);
    fewer.loops.min_keyframes = 7;
    let skipped = loops::detect(&m, &db, &camera(), 5, None, Mode::Rgbd, &fewer).unwrap();
    assert_eq!(skipped.summary.outcome, Outcome::Skipped);
    let error = loops::detect(&m, &db, &camera(), 9, None, Mode::Rgbd, &fewer).unwrap_err();
    assert_eq!(error.kind, ErrorKind::InvalidInput);
    assert!(
        error.message.contains("not in the map"),
        "{}",
        error.message
    );
}

/// The correction puts the second visit back at its true poses, its points replaced by the
/// scene's, a loop edge joining the visits; global BA keeps it there.
#[test]
fn the_correction_takes_the_drift_out() {
    let source = twice_seen();
    let found = loops::detect(
        &source,
        &database(&source),
        &camera(),
        5,
        None,
        Mode::Rgbd,
        &settings(2),
    )
    .unwrap();
    let closed = found.closed.expect("closed");
    for global_ba in [false, true] {
        let mut m = source.clone();
        let mut options = settings(2);
        options.loops.global_ba = global_ba;
        let summary = loops::correct(&mut m, &camera(), &closed, Mode::Rgbd, &options).unwrap();
        for k in 0..KEYFRAMES {
            assert!(
                gap(&m.kf_pose[KEYFRAMES + k], &pose(k)) < 1e-9,
                "keyframe {k}"
            );
        }
        assert_eq!(m.points(), POINTS);
        assert_eq!(m.loop_edges, [[0, 5]]);
        assert_eq!((summary.corrected, summary.replaced), (KEYFRAMES, POINTS));
        assert_eq!(summary.moved, POINTS);
        assert!(summary.links > 0 && summary.edges >= m.keyframes() - 1);
        assert_eq!(summary.global_ba.is_some(), global_ba);
        for (p, xyz) in m.pt_xyz.iter().zip(points()) {
            assert!((p - xyz).abs().max() < 1e-9);
        }
    }
}

/// Fusing the first visit's points into the second, through its true pose, replaces each of
/// its own points by the scene's.
#[test]
fn fusion_replaces_the_second_visits_points() {
    let mut m = twice_seen();
    let ids = scene_ids();
    let fused =
        loops::fuse_by_similarity(&mut m, &camera(), 5, &pose(2), &ids, &Options::default())
            .unwrap();
    assert_eq!(fused, POINTS);
    assert_eq!(m.points(), POINTS);
    assert_eq!(m.kp_point[m.kf_start[5]..m.kf_start[6]], ids);
}
