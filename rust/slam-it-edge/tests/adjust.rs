//! Integration tests of the bundle adjustment (`slam_it_edge::adjust`) on the exact scene: it
//! recovers the scene and forgets an observation moved off its point. Parity with the Python
//! reference is checked through the bindings by `tests/edge`.

mod scene;

use nalgebra::{Matrix4, Vector3};
use slam_it_edge::adjust::bundle_adjust;
use slam_it_edge::options::Options;
use slam_it_foundations::error::ErrorKind;
use slam_it_foundations::nls::Stop;

use scene::{KEYFRAMES, POINTS, camera, points, pose, scene_map};

#[test]
#[allow(clippy::cast_precision_loss, reason = "indices below 60, exact in f64")]
fn recovers_the_scene_and_forgets_a_moved_observation() {
    let mut m = scene_map();
    for k in 1..KEYFRAMES {
        let mut t = m.kf_pose[k].fixed_view_mut::<3, 1>(0, 3);
        t += Vector3::new(0.01, -0.01, 0.02);
    }
    for (p, x) in m.pt_xyz.iter_mut().enumerate() {
        let s = p as f64;
        *x += 0.01 * Vector3::new(s.sin(), s.cos(), (2.0 * s).sin());
    }
    let moved = m.kf_start[2];
    m.kp_xy[moved][0] += 30.0; // point 0's third view, pulling it off along the first's ray
    let start = m.pt_xyz[0];
    let adjustment = bundle_adjust(
        &mut m,
        &camera(),
        &[0, 1, 2],
        &[true, false, false],
        &Options::default(),
        &Stop::never(),
    )
    .unwrap();
    assert_eq!((adjustment.solves.len(), adjustment.outliers), (2, 1));
    assert!(adjustment.solves[0].final_cost < adjustment.solves[0].initial_cost);
    assert_eq!(m.kf_pose[0], Matrix4::identity());
    let truth = points();
    let mut worst: f64 = 0.0;
    for k in 1..KEYFRAMES {
        worst = worst.max((m.kf_pose[k] - pose(k)).abs().max());
    }
    for (p, x) in m.pt_xyz.iter().enumerate().skip(1) {
        worst = worst.max((x - truth[usize::try_from(m.pt_id[p]).unwrap()]).norm());
    }
    assert!(worst < 1e-6, "worst error {worst}");
    assert_eq!(m.kp_point[moved], -1);
    assert_eq!(m.points(), POINTS);
    assert_eq!((m.pt_id[0], m.kp_point[0], m.kp_point[POINTS]), (0, 0, 0));
    // A point left with one inlier view goes back to where it started.
    assert_eq!(m.pt_xyz[0], start);
}

#[test]
fn no_keyframe_held_fixed_solves_nothing() {
    let mut m = scene_map();
    let mut t = m.kf_pose[1].fixed_view_mut::<3, 1>(0, 3);
    t += Vector3::new(0.01, -0.01, 0.02);
    let before = m.clone();
    let adjustment = bundle_adjust(
        &mut m,
        &camera(),
        &[0, 1, 2],
        &[false, false, false],
        &Options::default(),
        &Stop::never(),
    )
    .unwrap();
    assert_eq!(adjustment.solves.len(), 0);
    assert_eq!((adjustment.free, adjustment.fixed), (0, 0));
    assert_eq!(
        (m.kf_pose.clone(), m.pt_xyz.clone()),
        (before.kf_pose, before.pt_xyz)
    );
}

#[test]
fn rejects_bad_rows_and_flags() {
    let mut m = scene_map();
    let options = Options::default();
    let err = bundle_adjust(
        &mut m,
        &camera(),
        &[3],
        &[true, false, false],
        &options,
        &Stop::never(),
    )
    .unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
    let err =
        bundle_adjust(&mut m, &camera(), &[0], &[true], &options, &Stop::never()).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}
