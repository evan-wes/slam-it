//! Integration tests of relocalisation (`slam_it_edge::relocalization`) on the exact scene's
//! map, its keyframes' words those of a vocabulary of one word: a frame of the scene is placed at
//! its pose, through a candidate, with every point; a frame with no features is not; and the
//! RANSAC budget is `MLPnP`'s.

mod scene;

use slam_it_edge::database::Database;
use slam_it_edge::options::Options;
use slam_it_edge::relocalization::{ransac_budget, relocalize};

use scene::{KEYFRAMES, POINTS, camera, empty_frame, frame, frame_at, moved, one_word, scene_map};

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
    let empty = empty_frame(0.0);
    let none = relocalize(&m, &db, &words, &camera(), &empty, &Options::default()).unwrap();
    assert!(none.tracked.is_none() && !none.summary.placed);
    assert!(none.summary.words == 0 && none.summary.candidates.is_empty());
}

/// The RANSAC budget: `MLPnP`'s, 35 samples for 100 matches at the defaults and 14 at 15; and
/// off them, each setting read (Python's `ransac_budget` gives the same): an inlier ratio of 0.4,
/// a minimum of 12 and a confidence of 0.95 need 40 of 100 in 46 samples, 20 if capped there,
/// and every one of 12, in one sample.
#[test]
fn the_ransac_budget_is_mlpnps() {
    let options = Options::default();
    assert_eq!(ransac_budget(100, &options), (50, 35));
    assert_eq!(ransac_budget(15, &options).1, 14);
    let mut moved = Options {
        reloc_inlier_ratio: 0.4,
        reloc_min_inliers: 12,
        reloc_confidence: 0.95,
        ..options
    };
    assert_eq!(ransac_budget(100, &moved), (40, 46));
    assert_eq!(ransac_budget(12, &moved), (12, 1));
    moved.reloc_ransac_iterations = 20;
    assert_eq!(ransac_budget(100, &moved), (40, 20));
}
