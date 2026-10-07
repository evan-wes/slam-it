//! Integration tests of initialisation (`slamlab_edge::initialization`) on the exact scene:
//! from depth, the initialisation matches, and a monocular start. Parity with the Python
//! reference is checked through the bindings by `tests/edge`.

mod scene;

use nalgebra::Matrix4;
use slamlab_edge::initialization::{
    initialize_from_depth, initialize_monocular, match_for_initialization,
};
use slamlab_edge::options::Options;
use slamlab_foundations::error::ErrorKind;

use scene::{POINTS, camera, frame, points, pose};

#[test]
fn from_depth_puts_a_point_at_every_keypoint() {
    let f = frame(0, true);
    let options = Options {
        min_init_features_depth: 10,
        ..Options::default()
    };
    let started = initialize_from_depth(&f, &options).unwrap().unwrap();
    assert_eq!(started.map.keyframes(), 1);
    assert_eq!(started.map.points(), POINTS);
    assert_eq!(started.tracked.inliers(), POINTS);
    for (x, truth) in started.map.pt_xyz.iter().zip(points()) {
        assert!((x - truth).norm() < 1e-12);
    }
    let strict = Options {
        min_init_features_depth: 100,
        ..Options::default()
    };
    assert!(initialize_from_depth(&f, &strict).unwrap().is_none());
    let other = Options {
        levels: 4,
        ..Options::default()
    };
    let err = initialize_from_depth(&f, &other).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}

#[test]
fn monocular_starts_a_unit_depth_map() {
    let (first, last) = (frame(0, false), frame(2, false));
    let found = match_for_initialization(&first, &last, &Options::default()).unwrap();
    assert_eq!(found.len(), POINTS);
    assert!(found.iter().all(|m| m.first == m.second));
    let few = initialize_monocular(&camera(), &first, &last, &Options::default()).unwrap();
    assert!(few.is_none());
    let options = Options {
        init_min_matches: 20,
        ..Options::default()
    };
    let started = initialize_monocular(&camera(), &first, &last, &options)
        .unwrap()
        .unwrap();
    let m = &started.map;
    assert_eq!(m.keyframes(), 2);
    assert_eq!(m.kf_pose[0], Matrix4::identity());
    let mut depths: Vec<f64> = m.pt_xyz.iter().map(|x| x.z).collect();
    depths.sort_by(f64::total_cmp);
    let n = depths.len();
    let median = f64::midpoint(depths[(n - 1) / 2], depths[n / 2]);
    assert!((median - 1.0).abs() < 1e-12);
    let moved = m.kf_pose[1].fixed_view::<3, 1>(0, 3).normalize();
    let truth = pose(2).fixed_view::<3, 1>(0, 3).normalize();
    assert!((moved - truth).norm() < 1e-6);
}
