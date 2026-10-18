//! Integration tests of local mapping (`slam_it_edge::mapping`) on the exact scene: the
//! keyframe decision, points from depth, culling points and keyframes, triangulation, fusion,
//! and a whole mapping step. Parity with the Python reference is checked through the bindings
//! by `tests/edge`. What fusion and local BA take from the graph (`fuse_targets`,
//! `local_ba_keyframes`) is held to the covisibility fixture in `tests/covisibility.rs`.

mod scene;

use slam_it_edge::frames::Tracked;
use slam_it_edge::map::{self, Map, check_map};
use slam_it_edge::mapping::{
    create_points_from_depth, cull_keyframes, cull_points, fuse_points, fuse_targets,
    local_ba_keyframes, match_for_triangulation, need_keyframe, process, triangulate_new_points,
};
use slam_it_edge::options::{MappingOptions, Mode, Options, StereoOptions};

use scene::{POINTS, camera, frame, frame_at, moved, points, pose, scene_map};

/// Point ids `first` to `last - 1`, then -1 to the scene's size.
#[allow(clippy::cast_possible_wrap, reason = "ids below 60")]
fn ids(first: usize, last: usize) -> Vec<i64> {
    (0..POINTS)
        .map(|p| {
            if (first..last).contains(&p) {
                p as i64
            } else {
                -1
            }
        })
        .collect()
}

/// Adds a point at each of the scene points `from` to `count`, seen by keyframe `k`'s keypoints
/// of the same index (its earlier keypoints observe others).
fn add_scene_points(m: &mut Map, k: usize, from: usize, count: usize) {
    let xyz = &points()[from..count];
    let observers: Vec<Vec<usize>> = (from..count).map(|p| vec![m.kf_start[k] + p]).collect();
    map::add_points(m, xyz, &observers).unwrap();
}

#[test]
fn keyframes_and_points_follow_the_rules() {
    let built = scene_map();
    let f = frame(2, true);
    let need = |kept| {
        let tracked = Tracked {
            pose: pose(2),
            points: ids(0, kept),
        };
        need_keyframe(
            &built,
            &camera(),
            &f.depth,
            &tracked,
            built.keyframes() - 1,
            Mode::Rgbd,
            &Options::default(),
            true,
            None,
            None,
        )
        .unwrap()
    };
    assert!(need(16));
    assert!(!need(15));
    assert!(!need(60));
    let tracked = Tracked {
        pose: pose(2),
        points: ids(0, 16),
    };
    let past = built.keyframes();
    let refused = need_keyframe(
        &built,
        &camera(),
        &f.depth,
        &tracked,
        past,
        Mode::Rgbd,
        &Options::default(),
        true,
        None,
        None,
    );
    assert!(
        refused.is_err(),
        "a reference keyframe out of range is refused"
    );
    let mut m = built.clone();
    let ahead = moved(0.1);
    map::insert_keyframe(&mut m, &frame_at(&ahead, true, 0), &ahead, &[-1; POINTS]).unwrap();
    let mut every = m.clone();
    create_points_from_depth(&mut every, &camera(), &Options::default()).unwrap();
    assert_eq!(every.points(), 2 * POINTS);
    let mut nearest = m.clone();
    let options = Options {
        stereo: StereoOptions {
            close_depth: 1.0,
            ..StereoOptions::default()
        },
        mapping: MappingOptions {
            min_new_points: 10,
            ..MappingOptions::default()
        },
        ..Options::default()
    };
    create_points_from_depth(&mut nearest, &camera(), &options).unwrap();
    assert_eq!(nearest.points(), POINTS + 11);
    let mut counted = built;
    counted.pt_visible[0] = 10;
    counted.pt_found[0] = 1;
    cull_points(&mut counted, Mode::Rgbd, &Options::default()).unwrap();
    assert_eq!((counted.points(), counted.pt_id[0]), (POINTS - 1, 1));
}

#[test]
fn triangulation_recovers_the_scene() {
    let mut m = Map::empty(8, 1.2);
    map::insert_keyframe(&mut m, &frame(0, false), &pose(0), &ids(0, 0)).unwrap();
    add_scene_points(&mut m, 0, 0, 10);
    map::insert_keyframe(&mut m, &frame(2, false), &pose(2), &ids(0, 10)).unwrap();
    let pairs = match_for_triangulation(&m, &camera(), 1, 0, &Options::default()).unwrap();
    assert_eq!((pairs.len(), pairs[0]), (50, [70, 10]));
    triangulate_new_points(&mut m, &camera(), Mode::Monocular, &Options::default()).unwrap();
    assert_eq!(m.points(), POINTS);
    for (x, truth) in m.pt_xyz.iter().zip(points()).skip(10) {
        assert!((x - truth).norm() < 1e-9);
    }
}

/// The keyframes share a few points, so they are linked, and each made its own of the rest:
/// fusing merges those.
#[test]
fn fusion_merges_duplicates() {
    const SHARED: usize = 5;
    let mut m = Map::empty(8, 1.2);
    map::insert_keyframe(&mut m, &frame(0, false), &pose(0), &ids(0, 0)).unwrap();
    add_scene_points(&mut m, 0, 0, POINTS);
    map::insert_keyframe(&mut m, &frame(2, false), &pose(2), &ids(0, SHARED)).unwrap();
    add_scene_points(&mut m, 1, SHARED, POINTS);
    fuse_points(&mut m, &camera(), Mode::Rgbd, &Options::default()).unwrap();
    assert_eq!(m.points(), POINTS);
    #[allow(clippy::cast_possible_wrap, reason = "ids below 120")]
    let duplicated = POINTS as i64;
    for (row, &p) in m.kp_point.iter().enumerate() {
        let index = row % POINTS; // in its keyframe
        #[allow(clippy::cast_possible_wrap, reason = "indices below 60")]
        let shared = index as i64;
        assert!(
            if index < SHARED {
                p == shared
            } else {
                p >= duplicated
            },
            "row {row}"
        );
    }
    check_map(&m).unwrap();
}

#[test]
fn culling_and_a_whole_step_keep_the_map_valid() {
    let built = scene_map();
    let mut copy = built.clone();
    map::insert_keyframe(&mut copy, &frame(2, true), &pose(2), &ids(0, 60)).unwrap();
    cull_keyframes(&mut copy, &camera(), Mode::Rgbd, &Options::default(), &[]).unwrap();
    assert_eq!(copy.kf_id, vec![0, 2, 3]);
    check_map(&copy).unwrap();
    let mut none = Map::empty(8, 1.2);
    assert!(
        cull_keyframes(&mut none, &camera(), Mode::Rgbd, &Options::default(), &[]).is_err(),
        "culling refuses a map without keyframes"
    );
    assert!(fuse_targets(&none, Mode::Rgbd, &Options::default()).is_err());
    assert!(local_ba_keyframes(&none, &Options::default()).is_err());
    let mut bad = Options::default();
    bad.covisibility.local_neighbours = 0;
    assert!(fuse_targets(&copy, Mode::Rgbd, &bad).is_err());
    assert!(local_ba_keyframes(&copy, &bad).is_err());
    let mut m = built;
    let mut ahead = pose(2);
    ahead[(2, 3)] -= 0.05;
    let tracked = Tracked {
        pose: ahead,
        points: ids(0, 60),
    };
    process(
        &mut m,
        &camera(),
        &frame_at(&ahead, true, 0),
        &tracked,
        Mode::Rgbd,
        &Options::default(),
        &[],
        None,
    )
    .unwrap();
    assert_eq!(m.kf_id.last(), Some(&3));
    check_map(&m).unwrap();
}
