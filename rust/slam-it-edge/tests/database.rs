//! Integration tests of the keyframe database (`slam_it_edge::database`) on the exact scene's
//! map: the inverted index as keyframes enter and leave it, relocalisation's candidates without
//! an erased keyframe, loop closing's outside a keyframe's links, and the refusals (a keyframe
//! given twice, one erased that is not there, a malformed vector, a keyframe outside the map or
//! the database).

mod scene;

use slam_it_edge::database::{Database, Words, candidates, loop_candidates};
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

/// Loop closing's candidates are never the keyframe or one linked to it; with every keyframe
/// linked to its best alone, the other is the one left; a keyframe outside the map or the
/// database is refused.
#[test]
fn loop_candidates_are_outside_a_keyframe_s_links() {
    let m = scene_map();
    let mut db = Database::default();
    for k in 0..3 {
        db.insert(k, words_of(k)).unwrap();
    }
    let linked = loop_candidates(&db, &m, 2, &Options::default()).unwrap();
    assert!(linked.keyframes.is_empty() && linked.sharing == 0);
    let best_only = Options {
        covisibility: CovisibilityOptions {
            min_points: 1 << 20,
            ..CovisibilityOptions::default()
        },
        ..Options::default()
    };
    let found = loop_candidates(&db, &m, 2, &best_only).unwrap();
    assert_eq!((found.sharing, found.keyframes.len()), (1, 1));
    assert_ne!(found.keyframes[0], 2);
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
