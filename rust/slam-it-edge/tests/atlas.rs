//! Integration tests of the atlas (`slam_it_edge::atlas`) through a monocular merge, on the exact
//! scene's map: frames tracked against the active map's keyframes, one keyframe then culled and
//! one point removed, all carried into the matched map's world and units by a scaled `G`, against
//! the known answer `rigid_of(T G⁻¹)` (as `tests/edge/test_atlas.py` holds Python's through a
//! whole merge); and the refusals of a kept map that is not there, and of a map closed or
//! merged with none tracked.

mod scene;

use std::f64::consts::PI;

use nalgebra::{Matrix4, Vector3};
use slam_it_edge::atlas::Atlas;
use slam_it_edge::loops::rigid_of;
use slam_it_edge::map::{self, Map, Removal};
use slam_it_edge::options::Options;
use slam_it_foundations::error::{Error, ErrorKind};
use slam_it_foundations::{sim3, so3};

use scene::{KEYFRAMES, POINTS, frame, moved, points, pose, scene_map};

/// The scale `G` carries the active map's world into the matched map's by: its world is the
/// matched map's scaled by 0.7, as a monocular map started apart has its own units.
const SCALE: f64 = 1.0 / 0.7;

/// The point removed from the active map before the merge.
const REMOVED: usize = 5;

/// Poses and points agree with the known answer to this: exact arithmetic, rounded.
const TOLERANCE: f64 = 1e-12;

/// `G`: a turn of 30 degrees about y, a shift and [`SCALE`].
fn to_world() -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(SCALE * so3::exp(&Vector3::new(0.0, PI / 6.0, 0.0))));
    out.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&Vector3::new(0.4, -0.1, 0.25));
    out
}

/// A camera's pose as the merge carries it into the matched map: `rigid_of(T G⁻¹)`.
fn carried(pose: &Matrix4<f64>) -> Matrix4<f64> {
    rigid_of(&(pose * sim3::inverse(&to_world())))
}

/// The records a monocular merge carries: two frames tracked in the active map, one against
/// keyframe 1, one against keyframe 2, which is then culled, so its frame is placed through its
/// parent; and the culled keyframe's and the removed point's last places.
#[test]
fn a_monocular_merge_carries_the_records_into_the_matched_maps_units() {
    let matched = scene_map();
    let mut active = scene_map();
    active.kf_parent = vec![-1, 0, 1];
    let mut atlas = Atlas::new(Options::default());
    atlas.start();
    atlas.date(Some(&matched));
    atlas.close(matched.clone(), None, false, true).unwrap();
    atlas.start();
    atlas.date(Some(&active));
    let through_culled = moved(0.2) * pose(2);
    atlas.place_frame(&active, 1.0, 1, &pose(2)).unwrap();
    atlas.place_frame(&active, 2.0, 2, &through_culled).unwrap();
    map::erase_keyframe(&mut active, 2).unwrap();
    let removed: Vec<bool> = (0..POINTS).map(|p| p == REMOVED).collect();
    map::erase_points(&mut active, &removed, Removal::Culled).unwrap();
    atlas.record(Some(&active));
    atlas.merge(0, &to_world()).unwrap();
    // The merged map: the matched map's keyframes, then the active map's left, carried.
    let mut merged: Map = matched;
    for k in 0..2 {
        map::insert_keyframe(
            &mut merged,
            &frame(k, false),
            &carried(&pose(k)),
            &[-1; POINTS],
        )
        .unwrap();
    }
    let trajectory = atlas.trajectory(Some(&merged));
    let expected = [carried(&pose(2)), carried(&through_culled)];
    assert_eq!(trajectory.poses.len(), 2);
    for (got, expected) in trajectory.poses.iter().zip(&expected) {
        assert!((got - expected).amax() < TOLERANCE);
    }
    assert_eq!(
        (trajectory.maps, trajectory.tracked_maps),
        (vec![0, 0], vec![1, 1])
    );
    let lives = atlas.lifetimes(Some(&merged));
    let (keyframe, point) = (KEYFRAMES + 2, POINTS + REMOVED);
    let g = to_world();
    let moved_to = g.fixed_view::<3, 3>(0, 0) * points()[REMOVED] + g.fixed_view::<3, 1>(0, 3);
    assert!((lives.keyframe_poses[keyframe] - carried(&pose(2))).amax() < TOLERANCE);
    assert!((lives.points[point] - moved_to).amax() < TOLERANCE);
    assert_eq!(
        (lives.keyframe_maps[keyframe], lives.point_maps[point]),
        (0, 0)
    );
}

/// A kept map that is not there is refused by its number, to take up or merge with; so is a map
/// closed or merged with none tracked.
#[test]
fn a_kept_map_that_is_not_there_is_refused_by_its_number() {
    let m = scene_map();
    let mut atlas = Atlas::new(Options::default());
    let refused =
        |result: Result<(), Error>| result.is_err_and(|e| e.kind == ErrorKind::InvalidInput);
    assert!(refused(atlas.close(m.clone(), None, false, false)));
    assert!(refused(atlas.merge(0, &to_world()).map(|_| ())));
    atlas.start();
    atlas.date(Some(&m));
    let resumed = atlas.resume(7);
    assert!(resumed.as_ref().is_err_and(|e| e.message.contains("map 7")));
    assert!(refused(resumed.map(|_| ())));
    assert!(refused(atlas.merge(7, &to_world()).map(|_| ())));
}
