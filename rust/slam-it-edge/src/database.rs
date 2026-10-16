//! The keyframe database: the Rust core's mirror of `slam_it.edge.database` (ORB-SLAM's
//! `KeyFrameDatabase`), whose docstring states its rules: each keyframe's word vector and its
//! features' nodes, the inverted index from each word to its keyframes, and relocalisation's
//! candidates.

use std::collections::{BTreeMap, HashMap};

use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::orb::Descriptor;
use slam_it_foundations::vocabulary::{self, BowVector, Vocabulary};

use crate::map::Map;
use crate::options::Options;

/// What the vocabulary made of one keyframe.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Words {
    /// Its word vector.
    pub vector: BowVector,
    /// Each of its features' node at the direct index's level, in the map's keypoint order.
    pub nodes: Vec<usize>,
}

/// The words of a map's keyframes, and the keyframes that hold each word
/// (`slam_it.edge.database.Database`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Database {
    /// Each keyframe's words, by its id.
    pub keyframes: BTreeMap<i64, Words>,
    /// Each word's keyframes, by id ascending.
    pub inverted: HashMap<i64, Vec<i64>>,
}

/// Relocalisation's candidates for a frame (`slam_it.edge.database.Candidates`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Candidates {
    /// The candidates' ids, by their group's accumulated score, descending (ties: the lower id).
    pub keyframes: Vec<i64>,
    /// Each one's group's accumulated score.
    pub scores: Vec<f64>,
    /// The keyframes that share a word with the frame.
    pub sharing: usize,
    /// Those that shared enough to be scored.
    pub scored: usize,
}

/// What the vocabulary makes of a keyframe's or a frame's features: its word vector, and each
/// feature's node at the direct index's level.
///
/// # Arguments
///
/// * `vocabulary` - The vocabulary.
/// * `descriptors` - The features' descriptors.
///
/// # Errors
///
/// `vocabulary::bow`'s, which a checked vocabulary's own words never meet.
pub fn words_of(vocabulary: &Vocabulary, descriptors: &[Descriptor]) -> Result<Words> {
    let quantized = vocabulary::quantize(vocabulary, descriptors);
    let vector = vocabulary::bow(vocabulary, &quantized.words)?;
    let nodes = quantized
        .nodes
        .iter()
        .map(|&n| usize::try_from(n).unwrap_or(usize::MAX)) // a node index, never negative
        .collect();
    Ok(Words { vector, nodes })
}

impl Database {
    /// Puts a keyframe's words in the database and indexes them.
    ///
    /// # Arguments
    ///
    /// * `keyframe` - Its id.
    /// * `words` - Its word vector and nodes.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if it is already in the database.
    pub fn insert(&mut self, keyframe: i64, words: Words) -> Result<()> {
        if self.keyframes.contains_key(&keyframe) {
            return fail(
                ErrorKind::InvalidInput,
                format!("database: keyframe {keyframe} is already in it"),
            );
        }
        for &word in &words.vector.ids {
            let holders = self.inverted.entry(word).or_default();
            let at = holders.partition_point(|&k| k < keyframe);
            holders.insert(at, keyframe);
        }
        self.keyframes.insert(keyframe, words);
        Ok(())
    }

    /// Adds a keyframe: quantises its features, and indexes its words
    /// (`slam_it.edge.database.Database.add`).
    ///
    /// # Arguments
    ///
    /// * `vocabulary` - The vocabulary.
    /// * `keyframe` - Its id.
    /// * `descriptors` - Its features' descriptors, in the map's keypoint order.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if it is already in the database.
    pub fn add(
        &mut self,
        vocabulary: &Vocabulary,
        keyframe: i64,
        descriptors: &[Descriptor],
    ) -> Result<()> {
        self.insert(keyframe, words_of(vocabulary, descriptors)?)
    }

    /// Removes a keyframe (culled): it leaves the index
    /// (`slam_it.edge.database.Database.erase`).
    ///
    /// # Arguments
    ///
    /// * `keyframe` - Its id.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if it is not in the database.
    pub fn erase(&mut self, keyframe: i64) -> Result<()> {
        let Some(words) = self.keyframes.remove(&keyframe) else {
            return fail(
                ErrorKind::InvalidInput,
                format!("database: keyframe {keyframe} is not in it"),
            );
        };
        for word in &words.vector.ids {
            if let Some(holders) = self.inverted.get_mut(word) {
                holders.retain(|&k| k != keyframe);
                if holders.is_empty() {
                    self.inverted.remove(word);
                }
            }
        }
        Ok(())
    }
}

/// The keyframes that observe a point, by id, each once.
///
/// # Arguments
///
/// * `m` - The map.
/// * `p` - The point's row.
///
/// # Returns
///
/// Their ids, ascending.
fn observers_of(m: &Map, p: usize) -> Vec<i64> {
    let mut out: Vec<i64> = m.pt_observations[p]
        .iter()
        .map(|&row| m.kf_id[m.keyframe_of(row)])
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Each keyframe's best covisible keyframes, from the map's observations
/// (`slam_it.edge.database.best_covisible`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `keyframes` - The keyframes' ids.
/// * `options` - `covisible_points` and `reloc_neighbours`.
///
/// # Returns
///
/// Each keyframe's id to the ids of those observing at least `covisible_points` of its points,
/// the most shared first (ties: the lower id), at most `reloc_neighbours`; none for a keyframe
/// the map does not hold.
#[must_use]
pub fn best_covisible(m: &Map, keyframes: &[i64], options: &Options) -> BTreeMap<i64, Vec<i64>> {
    let mut out = BTreeMap::new();
    for &keyframe in keyframes {
        let mut best = Vec::new();
        if let Some(k) = m.keyframe_row(keyframe) {
            let mut points: Vec<usize> = (m.kf_start[k]..m.kf_start[k + 1])
                .filter_map(|row| m.point_row(m.kp_point[row]))
                .collect();
            points.sort_unstable();
            points.dedup();
            let mut shared: BTreeMap<i64, usize> = BTreeMap::new();
            for p in points {
                for other in observers_of(m, p) {
                    if other != keyframe {
                        *shared.entry(other).or_default() += 1;
                    }
                }
            }
            let mut ranked: Vec<(usize, i64)> = shared
                .into_iter()
                .filter(|&(_, count)| count >= options.covisibility.min_points)
                .map(|(other, count)| (count, other))
                .collect();
            ranked.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
            best = ranked
                .into_iter()
                .take(options.relocalization.neighbours)
                .map(|(_, other)| other)
                .collect();
        }
        out.insert(keyframe, best);
    }
    out
}

/// Finds the keyframes a frame may be relocalised against
/// (`slam_it.edge.database.candidates`, ORB-SLAM's `DetectRelocalizationCandidates`).
///
/// # Arguments
///
/// * `db` - The map's keyframe database.
/// * `m` - The map, whose observations give covisibility.
/// * `frame` - The frame's word vector.
/// * `options` - The `reloc_*` settings and `covisible_points`.
///
/// # Returns
///
/// The candidates, best first, with their scores; none if no keyframe shares a word.
///
/// # Errors
///
/// `InvalidInput` for a malformed vector.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "word counts are far below 2^52; the floor of a fraction of one is one"
)]
pub fn candidates(
    db: &Database,
    m: &Map,
    frame: &BowVector,
    options: &Options,
) -> Result<Candidates> {
    vocabulary::check_vector(frame)?;
    let mut shared: BTreeMap<i64, usize> = BTreeMap::new();
    for word in &frame.ids {
        if let Some(holders) = db.inverted.get(word) {
            for &keyframe in holders {
                *shared.entry(keyframe).or_default() += 1;
            }
        }
    }
    let mut out = Candidates {
        sharing: shared.len(),
        ..Candidates::default()
    };
    let Some(&most) = shared.values().max() else {
        return Ok(out);
    };
    let least = (options.relocalization.common_words * most as f64).floor() as usize;
    let mut scores: BTreeMap<i64, f64> = BTreeMap::new();
    for (&keyframe, &count) in &shared {
        if count > least {
            scores.insert(
                keyframe,
                vocabulary::score(frame, &db.keyframes[&keyframe].vector)?,
            );
        }
    }
    out.scored = scores.len();
    let weighed: Vec<i64> = scores.keys().copied().collect();
    let neighbours = best_covisible(m, &weighed, options);
    let mut groups: Vec<(f64, i64)> = Vec::with_capacity(scores.len());
    for (&keyframe, &own) in &scores {
        let (mut total, mut best, mut best_score) = (own, keyframe, own);
        for other in &neighbours[&keyframe] {
            if let Some(&theirs) = scores.get(other) {
                total += theirs;
                if theirs > best_score {
                    best = *other;
                    best_score = theirs;
                }
            }
        }
        groups.push((total, best));
    }
    let retained = options.relocalization.group_score
        * groups
            .iter()
            .map(|&(total, _)| total)
            .fold(f64::NEG_INFINITY, f64::max);
    groups.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    for (total, best) in groups {
        if total > retained && !out.keyframes.contains(&best) {
            out.keyframes.push(best);
            out.scores.push(total);
        }
    }
    Ok(out)
}
