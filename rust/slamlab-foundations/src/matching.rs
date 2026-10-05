//! Matching binary descriptors: the Rust core's mirror of `slamlab.foundations.matching`.
//!
//! Hamming distances, and ORB-SLAM's filtered nearest neighbours: a distance cap, Lowe's ratio
//! test, one match per second-set descriptor, and the rotation-consistency histogram (the
//! Python module's docstring defines each filter).

use crate::error::{ErrorKind, Result, check_range, fail, fail_setting};
use crate::orb::{ANGLE_BINS, DESCRIPTOR_BYTES, Descriptor};

/// The most descriptors a set may hold.
pub const MAX_DESCRIPTORS: usize = 1 << 20;
/// The largest Hamming distance: every bit differs.
pub const MAX_DISTANCE: i64 = 8 * 32;
const _: () = assert!(DESCRIPTOR_BYTES == 32, "update MAX_DISTANCE");
/// The rotation histogram's second and third bins are kept only if they hold at least this
/// fraction of the highest (ORB-SLAM's `ComputeThreeMaxima`).
pub const SECONDARY_FRACTION: f64 = 0.1;

/// Matching filters (`slamlab.foundations.matching.Options`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// The largest Hamming distance a match may have, from 0 to 256.
    pub max_distance: i64,
    /// The ratio test's bound, in (0, 1].
    pub ratio: f64,
    /// Keep only matches in the rotation histogram's three highest bins.
    pub check_orientation: bool,
}

impl Default for Options {
    /// ORB-SLAM's distance cap, a ratio of 0.8, and the rotation check.
    fn default() -> Self {
        Self {
            max_distance: 50,
            ratio: 0.8,
            check_orientation: true,
        }
    }
}

/// One match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Match {
    /// The first-set index.
    pub first: usize,
    /// The second-set index.
    pub second: usize,
    /// The Hamming distance.
    pub distance: i64,
}

/// Checks matching filters.
///
/// # Arguments
///
/// * `options` - The filters.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
pub fn check_options(options: &Options) -> Result<()> {
    check_range("max_distance", options.max_distance, 0, MAX_DISTANCE)?;
    if !(options.ratio.is_finite() && options.ratio > 0.0 && options.ratio <= 1.0) {
        return fail_setting("ratio", "must be in (0, 1]", options.ratio);
    }
    Ok(())
}

/// The Hamming distance between two descriptors: the bits in which they differ.
///
/// # Arguments
///
/// * `a` - A descriptor.
/// * `b` - Another.
///
/// # Returns
///
/// From 0 to 256.
#[must_use]
pub fn distance(a: &Descriptor, b: &Descriptor) -> i64 {
    a.as_chunks::<8>()
        .0
        .iter()
        .zip(b.as_chunks::<8>().0)
        .map(|(x, y)| i64::from((u64::from_le_bytes(*x) ^ u64::from_le_bytes(*y)).count_ones()))
        .sum()
}

/// Every pair's Hamming distance.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
///
/// # Returns
///
/// The distances, row-major (first set by second set).
///
/// # Errors
///
/// `InvalidInput` for a set of more than [`MAX_DESCRIPTORS`].
pub fn distances(descriptors1: &[Descriptor], descriptors2: &[Descriptor]) -> Result<Vec<i64>> {
    if descriptors1.len() > MAX_DESCRIPTORS || descriptors2.len() > MAX_DESCRIPTORS {
        return fail(ErrorKind::InvalidInput, "descriptors: too many descriptors");
    }
    Ok(descriptors1
        .iter()
        .flat_map(|a| descriptors2.iter().map(move |b| distance(a, b)))
        .collect())
}

/// Checks a descriptor set's size and its angles.
///
/// # Arguments
///
/// * `count` - The number of descriptors.
/// * `angles` - Their orientation bins.
/// * `name` - The angles' name, for the message.
///
/// # Errors
///
/// `InvalidInput` for too many descriptors, or angles that are not one bin per descriptor.
fn check_set(count: usize, angles: &[usize], name: &str) -> Result<()> {
    if count > MAX_DESCRIPTORS {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: too many descriptors"),
        );
    }
    if angles.len() != count {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected {count} integers, got {}", angles.len()),
        );
    }
    if angles.iter().any(|&a| a >= ANGLE_BINS) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: bins must lie in [0, {ANGLE_BINS})"),
        );
    }
    Ok(())
}

/// A descriptor's nearest index and distance in a set, and the second-nearest distance (the
/// best again on a tie); ties go to the lowest index.
///
/// # Arguments
///
/// * `query` - The descriptor.
/// * `set` - The set, non-empty.
///
/// # Returns
///
/// `(index, best, second)`.
fn nearest(query: &Descriptor, set: &[Descriptor]) -> (usize, i64, i64) {
    let mut out = (0, i64::MAX, i64::MAX);
    for (j, candidate) in set.iter().enumerate() {
        let d = distance(query, candidate);
        if d < out.1 {
            out = (j, d, out.1);
        } else if d < out.2 {
            out.2 = d;
        }
    }
    out
}

/// The rotation histogram's kept bins: its highest three, the lower two only if they hold at
/// least a tenth of the highest (ORB-SLAM's `ComputeThreeMaxima`).
///
/// # Arguments
///
/// * `counts` - The histogram.
///
/// # Returns
///
/// The kept bins, highest first.
#[allow(
    clippy::cast_precision_loss,
    reason = "match counts are at most 2^20, exact in f64"
)]
fn three_maxima(counts: &[i64; ANGLE_BINS]) -> Vec<usize> {
    let mut top: Vec<(i64, usize)> = Vec::with_capacity(4);
    for (k, &count) in counts.iter().enumerate() {
        let place = top
            .iter()
            .position(|&(c, _)| count > c)
            .unwrap_or(top.len());
        top.insert(place, (count, k));
        top.truncate(3);
    }
    let highest = top[0].0;
    let mut kept = vec![top[0].1];
    for &(count, k) in &top[1..] {
        if (count as f64) < SECONDARY_FRACTION * highest as f64 {
            break;
        }
        kept.push(k);
    }
    kept
}

/// Matches each descriptor of the first set to its nearest in the second, filtered (Python's
/// and C++'s `match`; `match` is a Rust keyword).
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
/// * `angles1` - The first set's orientation bins.
/// * `angles2` - The second set's.
/// * `options` - The filters.
///
/// # Returns
///
/// The kept matches, by first-set index.
///
/// # Errors
///
/// `InvalidInput` for a bad filter, a set too large, or angles that are not one bin in
/// `[0, ANGLE_BINS)` per descriptor.
#[allow(
    clippy::cast_precision_loss,
    reason = "Hamming distances are at most 256, exact in f64"
)]
pub fn match_descriptors(
    descriptors1: &[Descriptor],
    descriptors2: &[Descriptor],
    angles1: &[usize],
    angles2: &[usize],
    options: &Options,
) -> Result<Vec<Match>> {
    check_options(options)?;
    check_set(descriptors1.len(), angles1, "angles1")?;
    check_set(descriptors2.len(), angles2, "angles2")?;
    if descriptors1.is_empty() || descriptors2.is_empty() {
        return Ok(Vec::new());
    }
    // Each second-set descriptor's claimant: the nearest first-set match passing the filters
    // (ties: the lowest first-set index, which is seen first).
    let mut claim: Vec<Option<Match>> = vec![None; descriptors2.len()];
    for (i, query) in descriptors1.iter().enumerate() {
        let (j, best, second) = nearest(query, descriptors2);
        let passes = descriptors2.len() == 1 || (best as f64) < options.ratio * second as f64;
        if !passes || best > options.max_distance {
            continue;
        }
        if claim[j].is_none_or(|held| best < held.distance) {
            claim[j] = Some(Match {
                first: i,
                second: j,
                distance: best,
            });
        }
    }
    let mut out: Vec<Match> = claim.into_iter().flatten().collect();
    out.sort_by_key(|m| m.first);
    if options.check_orientation && !out.is_empty() {
        let rotation = |m: &Match| (angles1[m.first] + ANGLE_BINS - angles2[m.second]) % ANGLE_BINS;
        let mut counts = [0_i64; ANGLE_BINS];
        for m in &out {
            counts[rotation(m)] += 1;
        }
        let kept = three_maxima(&counts);
        out.retain(|m| kept.contains(&rotation(m)));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/matching.json`,
    //! which the Python and C++ tests share, and the core's own input checks.

    use super::*;
    use serde_json::Value;

    /// Reads the shared fixture's functions.
    fn functions() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/matching.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: Value = serde_json::from_str(&text).expect("the fixture is valid JSON");
        fixture["functions"].clone()
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a non-negative integer")).expect("fits a usize")
    }

    /// Reads a list with `read`.
    fn list<T>(value: &Value, read: impl Fn(&Value) -> T) -> Vec<T> {
        value.as_array().expect("a list").iter().map(read).collect()
    }

    /// Reads a descriptor set.
    fn descriptors(value: &Value) -> Vec<Descriptor> {
        list(value, |row| {
            let bytes = list(row, |v| u8::try_from(count(v)).expect("a byte"));
            bytes.try_into().expect("32 bytes")
        })
    }

    /// `distances` matches the fixture.
    #[test]
    fn distances_match_the_fixture() {
        for case in list(&functions()["distances"]["cases"], Clone::clone) {
            let got = distances(
                &descriptors(&case["input"][0]),
                &descriptors(&case["input"][1]),
            )
            .unwrap();
            let expected = list(&case["expected"], |row| {
                list(row, |v| v.as_i64().expect("an integer"))
            });
            assert_eq!(got, expected.concat(), "{}", case["name"]);
        }
    }

    /// `match_descriptors` matches the fixture.
    #[test]
    fn match_matches_the_fixture() {
        for case in list(&functions()["match"]["cases"], Clone::clone) {
            let input = &case["input"];
            let settings = &input[4];
            let options = Options {
                max_distance: settings["max_distance"].as_i64().expect("an integer"),
                ratio: settings["ratio"].as_f64().expect("a float"),
                check_orientation: settings["check_orientation"].as_bool().expect("a bool"),
            };
            let got = match_descriptors(
                &descriptors(&input[0]),
                &descriptors(&input[1]),
                &list(&input[2], count),
                &list(&input[3], count),
                &options,
            )
            .unwrap();
            let e = &case["expected"];
            let expected: Vec<Match> = (0..e["distances"].as_array().expect("a list").len())
                .map(|k| Match {
                    first: count(&e["pairs"][k][0]),
                    second: count(&e["pairs"][k][1]),
                    distance: e["distances"][k].as_i64().expect("an integer"),
                })
                .collect();
            assert_eq!(got, expected, "{}", case["name"]);
        }
    }

    /// Bad filters and angles are `InvalidInput`.
    #[test]
    fn rejects_invalid_input() {
        let defaults = Options::default();
        for bad in [
            Options {
                max_distance: 257,
                ..defaults
            },
            Options {
                ratio: 0.0,
                ..defaults
            },
        ] {
            assert_eq!(
                check_options(&bad).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        let set = vec![[0_u8; DESCRIPTOR_BYTES]; 3];
        for (angles1, angles2) in [(&[0, 1][..], &[0, 1, 2][..]), (&[0, 1, 2], &[0, 1, 30])] {
            assert_eq!(
                match_descriptors(&set, &set, angles1, angles2, &defaults)
                    .unwrap_err()
                    .kind,
                ErrorKind::InvalidInput
            );
        }
    }
}
