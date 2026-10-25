//! Integration tests of the keyframe database (`slam_it_edge::database`) on the exact scene's
//! map: the inverted index as keyframes enter and leave it, relocalisation's candidates without
//! an erased keyframe, loop closing's sharing no point with the keyframe, and the refusals (a
//! keyframe given twice, one erased that is not there, a malformed vector, a keyframe outside the
//! map or the database).

mod scene;

use slam_it_edge::covisibility::links;
use slam_it_edge::database::{Database, Words, candidates, loop_candidates};
use slam_it_edge::map::{Map, forget_observations};
use slam_it_edge::options::{CovisibilityOptions, Options};
use slam_it_foundations::error::ErrorKind;
use slam_it_foundations::vocabulary::BowVector;

use scene::{POINTS, scene_map};

/// The scene's three keyframes' words: keyframe `k` holds words `k`, `k + 1` and 9, so
/// neighbours share a word and every keyframe shares word 9.
fn words_of(k: i64) -> Words {
    Words {
        vector: BowVector {
            ids: vec![k, k + 1, 9],
            values: vec![0.25, 0.25, 0.5],
        },
        nodes: vec![0; POINTS],
    }
}

/// A word's keyframes; none for a word no keyframe holds.
fn holders(db: &Database, word: i64) -> Vec<i64> {
    db.inverted.get(&word).cloned().unwrap_or_default()
}

/// A keyframe erased, as culling removes it, leaves every word's list and is never a
/// candidate; erasing it again is refused.
#[test]
fn an_erased_keyframe_leaves_the_index_and_the_candidates() {
    let m = scene_map();
    let mut db = Database::default();
    for k in 0..3 {
        db.insert(k, words_of(k)).unwrap();
    }
    assert_eq!(
        (holders(&db, 1), holders(&db, 9)),
        (vec![0, 1], vec![0, 1, 2])
    );
    let options = Options {
        covisibility: CovisibilityOptions {
            min_points: 1,
            ..CovisibilityOptions::default()
        },
        ..Options::default()
    };
    let frame = BowVector {
        ids: vec![1, 2, 9],
        values: vec![0.25, 0.25, 0.5],
    };
    assert_eq!(candidates(&db, &m, &frame, &options).unwrap().sharing, 3);
    db.erase(1).unwrap();
    assert_eq!(
        (holders(&db, 1), holders(&db, 2), holders(&db, 9)),
        (vec![0], vec![2], vec![0, 2])
    );
    assert!(!db.keyframes.contains_key(&1));
    let after = candidates(&db, &m, &frame, &options).unwrap();
    assert_eq!(after.sharing, 2);
    assert!(!after.keyframes.is_empty() && !after.keyframes.contains(&1));
    assert_eq!(db.erase(1).unwrap_err().kind, ErrorKind::InvalidInput);
    db.erase(0).unwrap();
    db.erase(2).unwrap();
    assert!(db.inverted.is_empty() && db.keyframes.is_empty());
    let none = candidates(&db, &m, &frame, &options).unwrap();
    assert!(none.keyframes.is_empty() && none.sharing == 0);
}

/// A keyframe inserted twice is refused, the index unchanged; a frame vector whose words do not
/// ascend is refused.
#[test]
fn a_keyframe_twice_and_a_malformed_vector_are_refused() {
    let m = scene_map();
    let mut db = Database::default();
    db.insert(0, words_of(0)).unwrap();
    assert_eq!(
        db.insert(0, words_of(0)).unwrap_err().kind,
        ErrorKind::InvalidInput
    );
    assert_eq!(holders(&db, 0), vec![0]);
    let unsorted = BowVector {
        ids: vec![9, 1],
        values: vec![0.5, 0.5],
    };
    let err = candidates(&db, &m, &unsorted, &Options::default()).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidInput);
}

/// The scene's map with its first keyframe seeing only the first `n` points: its other
/// observations forgotten, it shares `n` points with each other keyframe, and those two still
/// share all 60.
fn first_sharing(n: usize) -> Map {
    let mut m = scene_map();
    let rows: Vec<usize> = (n..POINTS).collect(); // the first keyframe's keypoints come first
    forget_observations(&mut m, &rows).unwrap();
    m
}

/// Loop closing's candidates are never the keyframe or one sharing a point with it, however few:
/// the first keyframe, sharing 1 to 14 points with the third and so not linked to it
/// (`covisibility.min_points` 15), is never the third's candidate; sharing none, it is. A
/// keyframe outside the map or the database is refused.
#[test]
fn loop_candidates_share_no_point_with_the_keyframe() {
    let mut db = Database::default();
    for k in 0..3 {
        db.insert(k, words_of(k)).unwrap();
    }
    let options = Options::default();
    for n in 0..15 {
        let m = first_sharing(n);
        let linked: Vec<usize> = links(&m, 2, &options)
            .unwrap()
            .iter()
            .map(|link| link.row)
            .collect();
        assert_eq!(
            linked,
            [1],
            "sharing {n}: the second keyframe alone is linked"
        );
        let found = loop_candidates(&db, &m, 2, &options).unwrap();
        if n == 0 {
            assert_eq!(
                found.keyframes,
                [0],
                "sharing none, the first is the candidate"
            );
        } else {
            assert!(
                found.keyframes.is_empty() && found.sharing == 0,
                "sharing {n}, the first is no candidate"
            );
        }
    }
    let m = scene_map();
    let best_only = Options {
        covisibility: CovisibilityOptions {
            min_points: 1 << 20,
            ..CovisibilityOptions::default()
        },
        ..Options::default()
    };
    let found = loop_candidates(&db, &m, 2, &best_only).unwrap();
    assert!(found.keyframes.is_empty() && found.sharing == 0);
    assert_eq!(
        loop_candidates(&db, &m, 7, &best_only).unwrap_err().kind,
        ErrorKind::InvalidInput
    );
    db.erase(2).unwrap();
    assert_eq!(
        loop_candidates(&db, &m, 2, &best_only).unwrap_err().kind,
        ErrorKind::InvalidInput
    );
}
