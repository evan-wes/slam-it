//! Integration tests of relocalisation (`slam_it_edge::relocalization`) on the exact scene's
//! map, its keyframes' words those of a vocabulary of one word: a frame of the scene is placed at
//! its pose, through a candidate, with every point; a frame with no features is not; and the
//! RANSAC budget is `MLPnP`'s.

mod scene;

use slam_it_edge::database::Database;
use slam_it_edge::frames::Frame;
use slam_it_edge::options::Options;
use slam_it_edge::relocalization::{ransac_budget, relocalize};
use slam_it_foundations::orb::Features;

use scene::{KEYFRAMES, POINTS, camera, frame, frame_at, moved, one_word, scene_map};

/// A frame of the scene is placed at its pose, through the keyframes sharing its word, with every
/// point; a frame with no features is not, and no candidate is tried.
#[test]
fn a_frame_of_the_scene_is_placed_and_an_empty_one_is_not() {
    let m = scene_map();
    let words = one_word();
    let mut db = Database::default();
    for k in 0..KEYFRAMES {
        let id = i64::try_from(k).unwrap();
        db.add(&words, id, &frame(k, true).features.descriptors)
            .unwrap();
    }
    let truth = moved(0.15);
    let found = relocalize(
        &m,
        &db,
        &words,
        &camera(),
        &frame_at(&truth, false, 0),
        &Options::default(),
    )
    .unwrap();
    let summary = &found.summary;
    assert!(summary.placed && summary.words == 1 && summary.sharing == KEYFRAMES);
    let tracked = found.tracked.expect("placed");
    assert!((tracked.pose - truth).abs().max() < 1e-6);
    assert_eq!(tracked.inliers(), POINTS);
    let last = summary.candidates.last().unwrap();
    assert_eq!((last.matches, last.found), (POINTS, Some(POINTS)));
    let features = Features {
        keypoints: Vec::new(),
        descriptors: Vec::new(),
        scale_factor: 1.2,
        levels: 8,
    };
    let empty = Frame::new(0.0, features, Vec::new(), Vec::new(), Vec::new()).unwrap();
    let none = relocalize(&m, &db, &words, &camera(), &empty, &Options::default()).unwrap();
    assert!(none.tracked.is_none() && !none.summary.placed);
    assert!(none.summary.words == 0 && none.summary.candidates.is_empty());
}

/// The RANSAC budget at the defaults: `MLPnP`'s, 35 samples for 100 matches, 14 at 15.
#[test]
fn the_ransac_budget_is_mlpnps() {
    let options = Options::default();
    assert_eq!(ransac_budget(100, &options), (50, 35));
    assert_eq!(ransac_budget(15, &options).1, 14);
}
