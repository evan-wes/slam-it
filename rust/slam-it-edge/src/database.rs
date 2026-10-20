//! The keyframe database: the Rust core's mirror of `slam_it.edge.database` (ORB-SLAM's
//! `KeyFrameDatabase`), whose docstring states its rules: each keyframe's word vector and its
//! features' nodes, the inverted index from each word to its keyframes, and relocalisation's and
//! loop closing's and a merge's candidates.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::orb::Descriptor;
use slam_it_foundations::vocabulary::{self, BowVector, Vocabulary};

use crate::covisibility;
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

/// Each scored keyframe's best `count` links' ids (`slam_it.edge.database._best`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `keyframes` - The keyframes' ids.
/// * `count` - The links kept.
/// * `options` - The settings: `covisibility.min_points`.
///
/// # Returns
///
/// Each keyframe's id to its best links' ids, most shared first; none for a keyframe the map
/// does not hold.
fn best_links(
    m: &Map,
    keyframes: &[i64],
    count: usize,
    options: &Options,
) -> BTreeMap<i64, Vec<i64>> {
    keyframes
        .iter()
        .map(|&keyframe| {
            let best = m.keyframe_row(keyframe).map_or_else(Vec::new, |k| {
                covisibility::links_of(
                    m,
                    &covisibility::shared_points(m, k),
                    options.covisibility.min_points,
                )
                .into_iter()
                .take(count)
                .map(|link| m.kf_id[link.row])
                .collect()
            });
            (keyframe, best)
        })
        .collect()
}

/// A query's scoring: its groups, best first, and how many keyframes were weighed.
struct Scoring {
    /// Each group's accumulated score and best keyframe, by score descending (ties: the lower
    /// id).
    groups: Vec<(f64, i64)>,
    /// The keyframes sharing a word, but the excluded.
    sharing: usize,
    /// Those scored.
    scored: usize,
}

/// Scores the keyframes sharing a frame's words and accumulates them by group
/// (`slam_it.edge.database._scoring`).
///
/// # Arguments
///
/// * `db` - The database.
/// * `m` - The map.
/// * `frame` - The frame's word vector, checked.
/// * `excluded` - The keyframes left out, by id.
/// * `(common_words, neighbours)` - The fraction of the most words shared that a keyframe must
///   exceed to be scored, and the links its score is accumulated over.
/// * `options` - The settings: `covisibility.min_points`.
///
/// # Returns
///
/// The scoring.
///
/// # Errors
///
/// A score's error.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "word counts are far below 2^52; the floor of a fraction of one is one"
)]
fn scoring(
    db: &Database,
    m: &Map,
    frame: &BowVector,
    excluded: &[i64],
    (common_words, neighbours): (f64, usize),
    options: &Options,
) -> Result<Scoring> {
    let mut shared: BTreeMap<i64, usize> = BTreeMap::new();
    for word in &frame.ids {
        if let Some(holders) = db.inverted.get(word) {
            for &keyframe in holders.iter().filter(|k| !excluded.contains(k)) {
                *shared.entry(keyframe).or_default() += 1;
            }
        }
    }
    let Some(&most) = shared.values().max() else {
        return Ok(Scoring {
            groups: Vec::new(),
            sharing: 0,
            scored: 0,
        });
    };
    let least = (common_words * most as f64).floor() as usize;
    let mut scores: BTreeMap<i64, f64> = BTreeMap::new();
    for (&keyframe, &count) in &shared {
        if count > least {
            scores.insert(
                keyframe,
                vocabulary::score(frame, &db.keyframes[&keyframe].vector)?,
            );
        }
    }
    let weighed: Vec<i64> = scores.keys().copied().collect();
    let links = best_links(m, &weighed, neighbours, options);
    let mut groups: Vec<(f64, i64)> = Vec::with_capacity(scores.len());
    for (&keyframe, &own) in &scores {
        let (mut total, mut best, mut best_score) = (own, keyframe, own);
        for other in &links[&keyframe] {
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
    groups.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    Ok(Scoring {
        groups,
        sharing: shared.len(),
        scored: scores.len(),
    })
}

/// The groups' best keyframes, each once, scoring above `above`, at most `most`.
///
/// # Arguments
///
/// * `found` - The scoring.
/// * `above` - The score a group must exceed.
/// * `most` - The candidates kept.
///
/// # Returns
///
/// The candidates, with the scoring's counts.
fn chosen(found: &Scoring, above: f64, most: usize) -> Candidates {
    let mut out = Candidates {
        sharing: found.sharing,
        scored: found.scored,
        ..Candidates::default()
    };
    for &(total, best) in &found.groups {
        if out.keyframes.len() == most {
            break;
        }
        if total > above && !out.keyframes.contains(&best) {
            out.keyframes.push(best);
            out.scores.push(total);
        }
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
/// * `options` - The `relocalization` settings and `covisibility.min_points`.
///
/// # Returns
///
/// The candidates, best first, with their scores; none if no keyframe shares a word.
///
/// # Errors
///
/// `InvalidInput` for a malformed vector.
pub fn candidates(
    db: &Database,
    m: &Map,
    frame: &BowVector,
    options: &Options,
) -> Result<Candidates> {
    vocabulary::check_vector(frame)?;
    let settings = &options.relocalization;
    let found = scoring(
        db,
        m,
        frame,
        &[],
        (settings.common_words, settings.neighbours),
        options,
    )?;
    let Some(&(best_total, _)) = found.groups.first() else {
        return Ok(Candidates::default());
    };
    Ok(chosen(
        &found,
        settings.group_score * best_total,
        found.groups.len(),
    ))
}

/// Finds the keyframes a keyframe may close a loop with (`slam_it.edge.database
/// .loop_candidates`, ORB-SLAM3's `DetectNBestCandidates`): relocalisation's scoring of its
/// words with `loops.common_words` and `loops.neighbours`, the keyframe and every keyframe
/// linked to it left out, the best `loops.candidates` groups.
///
/// # Arguments
///
/// * `db` - The map's keyframe database, holding the keyframe.
/// * `m` - The map, holding the keyframe.
/// * `keyframe` - The keyframe's id.
/// * `options` - The `loops` settings and `covisibility.min_points`.
///
/// # Returns
///
/// The candidates, best first, with their scores.
///
/// # Errors
///
/// `InvalidInput` if the keyframe is not in the map or the database.
pub fn loop_candidates(
    db: &Database,
    m: &Map,
    keyframe: i64,
    options: &Options,
) -> Result<Candidates> {
    let Some(k) = m.keyframe_row(keyframe) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("keyframe {keyframe} is not in the map"),
        );
    };
    let Some(words) = db.keyframes.get(&keyframe) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("database: keyframe {keyframe} is not in it"),
        );
    };
    let mut excluded = vec![keyframe];
    excluded.extend(
        covisibility::links_of(
            m,
            &covisibility::shared_points(m, k),
            options.covisibility.min_points,
        )
        .iter()
        .map(|link| m.kf_id[link.row]),
    );
    let settings = &options.loops;
    let found = scoring(
        db,
        m,
        &words.vector,
        &excluded,
        (settings.common_words, settings.neighbours),
        options,
    )?;
    Ok(chosen(&found, f64::NEG_INFINITY, settings.candidates))
}

/// A map with its keyframe database, and its number among a run's maps
/// (`slam_it.edge.database.Indexed`).
#[derive(Clone, Debug)]
pub struct Indexed {
    /// The map's number: maps are numbered in the order they started.
    pub number: i64,
    /// The map.
    pub map: Map,
    /// Its keyframe database, holding every keyframe of the map.
    pub database: Database,
}

/// A merge's candidates, from every other map, best first
/// (`slam_it.edge.database.MergeCandidates`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MergeCandidates {
    /// Each candidate's map number.
    pub maps: Vec<i64>,
    /// Its id in that map.
    pub keyframes: Vec<i64>,
    /// Its group's accumulated score, descending (ties: the lower map number, then the lower id).
    pub scores: Vec<f64>,
    /// The keyframes, over every map, that share a word with the query.
    pub sharing: usize,
    /// Those scored.
    pub scored: usize,
}

/// A candidate of one of several maps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ranked {
    /// Its group's accumulated score.
    pub score: f64,
    /// Its map's number.
    pub map: i64,
    /// Its id in that map.
    pub keyframe: i64,
}

/// A map with its keyframe database and number, held elsewhere: an [`Indexed`] by reference,
/// for a search over maps owned apart (relocalisation's over the one tracked and the kept ones;
/// merge detection's over the kept ones, which the atlas shares, `crate::atlas::Atlas::kept`).
#[derive(Clone, Copy, Debug)]
pub struct MapRef<'a> {
    /// The map's number.
    pub number: i64,
    /// The map.
    pub map: &'a Map,
    /// Its keyframe database, holding every keyframe of the map.
    pub database: &'a Database,
}

impl<'a> From<&'a Indexed> for MapRef<'a> {
    fn from(indexed: &'a Indexed) -> Self {
        Self {
            number: indexed.number,
            map: &indexed.map,
            database: &indexed.database,
        }
    }
}

/// Checks that no two maps share a number.
///
/// # Arguments
///
/// * `numbers` - The maps' numbers.
/// * `name` - The argument's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for a number given twice.
pub fn check_numbers(numbers: impl IntoIterator<Item = i64>, name: &str) -> Result<()> {
    let mut seen = BTreeSet::new();
    if numbers.into_iter().all(|number| seen.insert(number)) {
        Ok(())
    } else {
        fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected each map once, by its number"),
        )
    }
}

/// Orders several maps' candidates best first: by score, descending; ties by the lower map
/// number, then the lower id (a merge's and relocalisation's order across maps).
///
/// # Arguments
///
/// * `found` - The candidates, reordered in place.
pub fn rank(found: &mut [Ranked]) {
    found.sort_by(|a, b| {
        b.score
            .total_cmp(&a.score)
            .then((a.map, a.keyframe).cmp(&(b.map, b.keyframe)))
    });
}

/// Finds the keyframes of other maps a keyframe may merge its map with
/// (`slam_it.edge.database.merge_candidates`, `DetectNBestCandidates`' merge list): in each
/// map, [`loop_candidates`]' scoring of the query's words, its words counted within it, nothing
/// left out, its best `loops.candidates` groups; then the best `loops.candidates` of every
/// map's, by accumulated score.
///
/// # Arguments
///
/// * `vector` - The query keyframe's word vector.
/// * `others` - The other maps, each with its database and number.
/// * `options` - The `loops` settings and `covisibility.min_points`.
///
/// # Returns
///
/// The candidates, best first.
///
/// # Errors
///
/// `InvalidInput` for a malformed vector or two maps with one number.
pub fn merge_candidates(
    vector: &BowVector,
    others: &[MapRef<'_>],
    options: &Options,
) -> Result<MergeCandidates> {
    vocabulary::check_vector(vector)?;
    check_numbers(others.iter().map(|other| other.number), "others")?;
    let settings = &options.loops;
    let mut found = Vec::new();
    let mut out = MergeCandidates::default();
    for other in others {
        let scored = scoring(
            other.database,
            other.map,
            vector,
            &[],
            (settings.common_words, settings.neighbours),
            options,
        )?;
        out.sharing += scored.sharing;
        out.scored += scored.scored;
        let best = chosen(&scored, f64::NEG_INFINITY, settings.candidates);
        found.extend(
            best.keyframes
                .iter()
                .zip(&best.scores)
                .map(|(&keyframe, &score)| Ranked {
                    score,
                    map: other.number,
                    keyframe,
                }),
        );
    }
    rank(&mut found);
    found.truncate(settings.candidates);
    for candidate in found {
        out.maps.push(candidate.map);
        out.keyframes.push(candidate.keyframe);
        out.scores.push(candidate.score);
    }
    Ok(out)
}
